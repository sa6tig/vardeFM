//! FM radio channel simulator.
//!
//! Models what happens to modem audio between two FM radios:
//! the sending radio's audio response tilt, TX audio filtering, pre-emphasis, a CTCSS
//! tone, deviation limiting, the FM modulator, RF noise,
//! the IF filter, the discriminator (including threshold "clicks" at low signal),
//! de-emphasis, RX audio filtering, squelch, TX key-up delay, and sound-card clock offset.

use num_complex::Complex32;
use rand::rngs::StdRng;
use rand::SeedableRng;
use rand_distr::{Distribution, Normal};
use std::f32::consts::PI;
use vardefm_dsp::filter::{design_lowpass_fir, fir_complex, resample_cubic, Chain, FirstOrder};

#[derive(Clone, Debug)]
pub struct FmChannelConfig {
    pub fs: f32,
    /// Peak deviation in Hz for audio amplitude 1.0.
    pub deviation_hz: f32,
    /// TX pre-emphasis time constant in microseconds (None for the flat data port).
    pub tx_emphasis_us: Option<f32>,
    /// RX de-emphasis time constant in microseconds. A radio whose value differs from
    /// `tx_emphasis_us` passes the modem audio with a tilted response.
    pub rx_emphasis_us: Option<f32>,
    /// TX audio response tilt in dB per octave, -6 to +6: a sending radio whose audio
    /// response does not match the receiver's. The slope holds across `tx_band` down
    /// to 300 Hz and eases off over the octave below. The tilted audio is scaled back
    /// to the RMS it had before, as an operator sets the drive level on the signal the
    /// radio actually sends, so the tilt changes the spectrum but not the deviation.
    /// It acts before pre-emphasis and the deviation limiter, as a microphone or
    /// audio-stage mismatch does; a data port whose mismatch sits after the limiter
    /// (in the modulator) is not modelled.
    pub tx_tilt_db_per_octave: f32,
    /// A CTCSS tone the sending radio adds after its audio response and before the
    /// deviation limiter, as a sub-tone encoder does (None for none).
    pub ctcss: Option<Ctcss>,
    /// TX audio passband in Hz.
    pub tx_band: (f32, f32),
    /// RX audio passband in Hz.
    pub rx_band: (f32, f32),
    /// IF filter bandwidth in Hz (two-sided).
    pub if_bandwidth_hz: f32,
    /// Carrier-to-noise ratio in the IF bandwidth, dB. None = no noise.
    pub cnr_db: Option<f32>,
    /// Deviation limiter: audio is clipped at this level after pre-emphasis.
    pub clip_level: f32,
    /// Time from PTT until the carrier is up and the receiver unsquelches.
    pub keyup_delay_s: f32,
    /// Noise burst heard after the carrier drops.
    pub squelch_tail_s: f32,
    /// Receiver clock error relative to the transmitter, in ppm.
    pub clock_offset_ppm: f32,
    pub lead_silence_s: f32,
    pub trail_silence_s: f32,
    pub seed: u64,
}

impl FmChannelConfig {
    /// Mic/speaker path of a typical 12.5 kHz FM voice radio.
    pub fn narrow() -> Self {
        Self {
            fs: 48_000.0,
            deviation_hz: 2_500.0,
            tx_emphasis_us: Some(750.0),
            rx_emphasis_us: Some(750.0),
            tx_tilt_db_per_octave: 0.0,
            ctcss: None,
            tx_band: (300.0, 3_000.0),
            rx_band: (300.0, 3_000.0),
            if_bandwidth_hz: 11_000.0,
            cnr_db: None,
            clip_level: 1.0,
            keyup_delay_s: 0.05,
            squelch_tail_s: 0.1,
            clock_offset_ppm: 0.0,
            lead_silence_s: 0.2,
            trail_silence_s: 0.2,
            seed: 1,
        }
    }

    /// Flat 9600-baud data port path.
    pub fn wide() -> Self {
        Self {
            tx_emphasis_us: None,
            rx_emphasis_us: None,
            deviation_hz: 3_000.0,
            tx_band: (20.0, 7_000.0),
            rx_band: (20.0, 7_000.0),
            if_bandwidth_hz: 16_000.0,
            ..Self::narrow()
        }
    }

    pub fn by_name(name: &str) -> Option<Self> {
        match name {
            "narrow" => Some(Self::narrow()),
            "wide" => Some(Self::wide()),
            _ => None,
        }
    }
}

/// A CTCSS tone added to the transmitted audio.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ctcss {
    pub hz: f64,
    /// Peak deviation of the tone in Hz.
    pub deviation_hz: f32,
    /// Starting phase, in cycles.
    pub phase: f64,
}

/// Lowest frequency the TX tilt holds its slope to, Hz; over the octave below, the
/// response eases off.
const TILT_LOW_HZ: f32 = 300.0;

/// `audio` through the sending radio's response tilt, at the RMS it had before; None
/// when there is no tilt.
fn tx_tilt(cfg: &FmChannelConfig, audio: &[f32]) -> Option<Vec<f32>> {
    let s = cfg.tx_tilt_db_per_octave;
    if s == 0.0 {
        return None;
    }
    // The tilt spans the speech band at most: a radio's response does not keep
    // rising (or falling) to the 20 Hz edge of a data port.
    let lo = cfg.tx_band.0.max(TILT_LOW_HZ);
    assert!(lo < cfg.tx_band.1, "TX tilt needs a tx_band reaching above {TILT_LOW_HZ} Hz, not {:?}", cfg.tx_band);
    let mut tilt = Chain::tilt(cfg.fs, s, lo, cfg.tx_band.1);
    let mut out: Vec<f32> = audio.iter().map(|&x| tilt.process(x)).collect();
    let power = |v: &[f32]| v.iter().map(|&x| (x as f64).powi(2)).sum::<f64>();
    let (before, after) = (power(audio), power(&out));
    if after > 0.0 {
        let g = (before / after).sqrt() as f32;
        out.iter_mut().for_each(|x| *x *= g);
    }
    Some(out)
}

/// Pass one transmission (audio at `cfg.fs`, full scale = peak deviation) through the channel.
pub fn transmit(cfg: &FmChannelConfig, audio: &[f32]) -> Vec<f32> {
    let fs = cfg.fs;
    let lead = (cfg.lead_silence_s * fs) as usize;
    let trail = (cfg.trail_silence_s * fs) as usize;
    let keyup = (cfg.keyup_delay_s * fs) as usize;
    let tail = (cfg.squelch_tail_s * fs) as usize;
    let total = lead + audio.len() + trail;
    let carrier = lead..lead + audio.len();

    // --- Transmitter audio chain ---
    let mut tx_chain = Chain::bandpass4(fs, cfg.tx_band.0, cfg.tx_band.1);
    let mut pre = cfg.tx_emphasis_us.map(|us| FirstOrder::preemphasis(fs, us * 1e-6));
    let tilted = tx_tilt(cfg, audio);
    let audio = tilted.as_deref().unwrap_or(audio);
    let mut tx_audio = vec![0f32; total];
    for (i, &x) in audio.iter().enumerate() {
        let mut v = x;
        if let Some(p) = &mut pre {
            v = p.process(v);
        }
        if let Some(c) = &cfg.ctcss {
            let a = c.deviation_hz / cfg.deviation_hz;
            v += a * (2.0 * std::f64::consts::PI * (c.hz * i as f64 / fs as f64 + c.phase)).sin() as f32;
        }
        v = v.clamp(-cfg.clip_level, cfg.clip_level);
        tx_audio[lead + i] = tx_chain.process(v);
    }

    // --- FM modulator + RF noise ---
    let mut rng = StdRng::seed_from_u64(cfg.seed);
    let sigma = cfg.cnr_db.map(|c| (10f32.powf(-c / 10.0) * fs / cfg.if_bandwidth_hz / 2.0).sqrt());
    let normal = Normal::new(0.0f32, 1.0).unwrap();
    let mut phase = 0f32;
    let k = 2.0 * PI * cfg.deviation_hz / fs;
    let rf: Vec<Complex32> = (0..total)
        .map(|i| {
            let mut z = if carrier.contains(&i) {
                phase = (phase + k * tx_audio[i]) % (2.0 * PI);
                Complex32::from_polar(1.0, phase)
            } else {
                Complex32::new(0.0, 0.0)
            };
            if let Some(s) = sigma {
                z += Complex32::new(normal.sample(&mut rng), normal.sample(&mut rng)) * s;
            } else if !carrier.contains(&i) {
                // A tiny noise floor keeps the discriminator defined with no carrier.
                z += Complex32::new(normal.sample(&mut rng), normal.sample(&mut rng)) * 1e-3;
            }
            z
        })
        .collect();

    // --- Receiver: IF filter, discriminator, de-emphasis, audio filter ---
    let taps = design_lowpass_fir(129, cfg.if_bandwidth_hz / 2.0, fs);
    let delay = taps.len() / 2;
    let iff = fir_complex(&taps, &rf);
    let mut de = cfg.rx_emphasis_us.map(|us| FirstOrder::deemphasis(fs, us * 1e-6));
    let mut rx_chain = Chain::bandpass4(fs, cfg.rx_band.0, cfg.rx_band.1);
    let scale = fs / (2.0 * PI * cfg.deviation_hz);
    let mut out = vec![0f32; total];
    let mut prev = Complex32::new(1.0, 0.0);
    for i in 0..total {
        let z = iff[(i + delay).min(total - 1)];
        let mut v = (z * prev.conj()).arg() * scale;
        prev = z;
        if let Some(d) = &mut de {
            v = d.process(v);
        }
        out[i] = rx_chain.process(v);
    }

    // --- Squelch: closed until the carrier is up, open for a short tail afterwards ---
    let open = carrier.start + keyup..carrier.end + tail;
    for (i, v) in out.iter_mut().enumerate() {
        if !open.contains(&i) {
            *v = 0.0;
        }
    }

    if cfg.clock_offset_ppm != 0.0 {
        out = resample_cubic(&out, 1.0 + cfg.clock_offset_ppm as f64 * 1e-6);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clean_tone_passes_through() {
        let cfg = FmChannelConfig { lead_silence_s: 0.0, keyup_delay_s: 0.0, ..FmChannelConfig::narrow() };
        let tone: Vec<f32> = (0..48_000).map(|i| 0.3 * (2.0 * PI * 1000.0 * i as f32 / 48_000.0).sin()).collect();
        let out = transmit(&cfg, &tone);
        let peak = out[24_000..40_000].iter().fold(0f32, |m, v| m.max(v.abs()));
        assert!((peak - 0.3).abs() < 0.05, "peak {peak}");
    }

    /// Amplitude of the `freq` component of `x[seg]`, by a least-squares sine fit.
    fn amplitude(x: &[f32], seg: std::ops::Range<usize>, freq: f64) -> f64 {
        let (mut ss, mut cc, mut sc, mut ys, mut yc) = (0f64, 0f64, 0f64, 0f64, 0f64);
        for i in seg {
            let (s, c) = (2.0 * std::f64::consts::PI * freq * i as f64 / 48_000.0).sin_cos();
            (ss, cc, sc) = (ss + s * s, cc + c * c, sc + s * c);
            (ys, yc) = (ys + x[i] as f64 * s, yc + x[i] as f64 * c);
        }
        let det = ss * cc - sc * sc;
        let (a, b) = ((ys * cc - yc * sc) / det, (yc * ss - ys * sc) / det);
        a.hypot(b)
    }

    fn quiet_wide(tilt: f32) -> FmChannelConfig {
        FmChannelConfig {
            tx_tilt_db_per_octave: tilt,
            lead_silence_s: 0.0,
            keyup_delay_s: 0.0,
            ..FmChannelConfig::wide()
        }
    }

    fn tones(freqs: &[f64], amp: f32, n: usize) -> Vec<f32> {
        (0..n)
            .map(|i| {
                let t = i as f64 / 48_000.0;
                // Phases spread by index keep the peaks of the sum low.
                freqs
                    .iter()
                    .enumerate()
                    .map(|(k, f)| amp * (2.0 * std::f64::consts::PI * f * t + k as f64 * 2.4).sin() as f32)
                    .sum()
            })
            .collect()
    }

    /// Tones sent together keep their relative levels apart from the tilt: the slope
    /// from 1 kHz, levelling off below the 300 Hz floor, at the RMS they had.
    #[test]
    fn tx_tilt_shapes_tones_by_the_octave_down_to_its_floor_at_the_same_level() {
        let freqs = [100.0, 300.0, 500.0, 1000.0, 4000.0];
        let input = tones(&freqs, 0.05, 48_000);
        let seg = 24_000..44_000;
        let rms = |v: &[f32]| (v[seg.clone()].iter().map(|x| x * x).sum::<f32>() / seg.len() as f32).sqrt();
        let flat = transmit(&quiet_wide(0.0), &input);
        for slope in [-6.0f32, -3.0, 3.0, 6.0] {
            let out = transmit(&quiet_wide(slope), &input);
            let db =
                |f: f64| 20.0 * (amplitude(&out, seg.clone(), f) / amplitude(&flat, seg.clone(), f)).log10() as f32;
            for (f, octaves) in [(500.0, (0.5f32).log2()), (4000.0, 2.0)] {
                let got = db(f) - db(1000.0);
                assert!((got - slope * octaves).abs() < 0.7, "{slope} dB/oct at {f} Hz: {got:.2} dB from 1 kHz");
            }
            // The full slope would put 100 Hz 9.5 dB from 300 Hz; the floor eases it.
            let below = (db(100.0) - db(300.0)) * slope.signum();
            assert!(-6.5 < below && below < -2.0, "{slope} dB/oct: 100 Hz is {below:.2} dB from 300 Hz");
            let level = 20.0 * (rms(&out) / rms(&flat)).log10();
            assert!(level.abs() < 0.3, "{slope} dB/oct changed the level by {level:.2} dB");
        }
    }

    /// The tilt shapes the audio before the deviation limiter. Two tones that clip
    /// when sent flat stop clipping once a tilt makes one of them dominate at the same
    /// RMS, so no intermodulation product appears; a tilt after the limiter would
    /// reshape audio that had already clipped.
    #[test]
    fn tx_tilt_acts_before_the_limiter() {
        let input = tones(&[300.0, 4000.0], 0.6, 48_000);
        let seg = 24_000..44_000;
        let im_db = |slope: f32| {
            let out = transmit(&quiet_wide(slope), &input);
            let main = amplitude(&out, seg.clone(), 300.0).max(amplitude(&out, seg.clone(), 4000.0));
            20.0 * (amplitude(&out, seg.clone(), 3400.0) / main).log10()
        };
        let flat = im_db(0.0);
        assert!(flat > -40.0, "flat: no clipping to compare with ({flat:.1} dB)");
        for slope in [-6.0, 6.0] {
            let im = im_db(slope);
            assert!(im < -60.0, "{slope} dB/oct: 4000-2*300 Hz product at {im:.1} dB");
        }
    }

    /// The CTCSS tone joins after the tilt, so its deviation is what was asked for.
    #[test]
    fn ctcss_keeps_its_deviation_through_a_tilt() {
        let input = tones(&[1000.0, 3000.0], 0.2, 48_000);
        for slope in [-6.0, 0.0, 6.0] {
            let cfg = FmChannelConfig {
                ctcss: Some(Ctcss { hz: 67.0, deviation_hz: 300.0, phase: 0.0 }),
                ..quiet_wide(slope)
            };
            let got = amplitude(&transmit(&cfg, &input), 24_000..44_000, 67.0) * cfg.deviation_hz as f64;
            assert!((got / 300.0 - 1.0).abs() < 0.06, "{slope} dB/oct: CTCSS deviation {got:.0} Hz");
        }
    }
}

#[cfg(test)]
mod probe {
    use super::*;
    #[test]
    #[ignore]
    fn tone_snr_probe() {
        for (name, base) in [("narrow", FmChannelConfig::narrow()), ("wide", FmChannelConfig::wide())] {
            for cnr in [None, Some(30.0), Some(20.0), Some(12.0)] {
                let cfg = FmChannelConfig { cnr_db: cnr, lead_silence_s: 0.0, keyup_delay_s: 0.0, ..base.clone() };
                let n = 96_000;
                let tone: Vec<f32> = (0..n).map(|i| 0.35 * (2.0 * PI * 1000.0 * i as f32 / 48_000.0).sin()).collect();
                let out = transmit(&cfg, &tone);
                // least-squares fit of a 1 kHz sine, residual = noise+distortion
                let seg = 24_000..90_000;
                let (mut ss, mut cc, mut sc) = (0f64, 0f64, 0f64);
                let (mut ys, mut yc) = (0f64, 0f64);
                for i in seg.clone() {
                    let w = 2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 48_000.0;
                    let (s, c) = w.sin_cos();
                    ss += s * s;
                    cc += c * c;
                    sc += s * c;
                    ys += out[i] as f64 * s;
                    yc += out[i] as f64 * c;
                }
                let det = ss * cc - sc * sc;
                let a = (ys * cc - yc * sc) / det;
                let b = (yc * ss - ys * sc) / det;
                let (mut sig, mut err) = (0f64, 0f64);
                for i in seg {
                    let w = 2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 48_000.0;
                    let fit = a * w.sin() + b * w.cos();
                    sig += fit * fit;
                    err += (out[i] as f64 - fit).powi(2);
                }
                println!("{name} cnr={cnr:?}: SNR {:.1} dB", 10.0 * (sig / err).log10());
            }
        }
    }
}
