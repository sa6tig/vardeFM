//! FM radio channel simulator.
//!
//! Models what happens to modem audio between two FM radios:
//! TX audio filtering, pre-emphasis, deviation limiting, the FM modulator, RF noise,
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
    /// TX audio response tilt in dB per octave across `tx_band` above 300 Hz: a
    /// sending radio whose audio response does not match the receiver's. It keeps
    /// the power of audio with a flat spectrum across that band, so it reshapes the
    /// modem's carriers without changing their level much, and acts before the
    /// deviation limiter, like pre-emphasis.
    pub tx_tilt_db_per_octave: f32,
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

/// Lowest frequency the TX tilt reaches, Hz; below it the response levels off.
const TILT_LOW_HZ: f32 = 300.0;

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
    // The tilt spans the speech band at most: a radio's response does not keep
    // rising (or falling) to the 20 Hz edge of a data port.
    let tilt_lo = cfg.tx_band.0.max(TILT_LOW_HZ);
    let mut tilt = Chain::tilt(fs, cfg.tx_tilt_db_per_octave, tilt_lo, cfg.tx_band.1);
    let mut tx_audio = vec![0f32; total];
    for (i, &x) in audio.iter().enumerate() {
        let mut v = tilt.process(x);
        if let Some(p) = &mut pre {
            v = p.process(v);
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

    #[test]
    fn tx_tilt_shapes_tones_by_the_octave() {
        let peak_db = |slope: f32, freq: f32| {
            let cfg = FmChannelConfig {
                tx_tilt_db_per_octave: slope,
                lead_silence_s: 0.0,
                keyup_delay_s: 0.0,
                ..FmChannelConfig::wide()
            };
            let tone: Vec<f32> = (0..48_000).map(|i| 0.1 * (2.0 * PI * freq * i as f32 / 48_000.0).sin()).collect();
            let out = transmit(&cfg, &tone);
            20.0 * (out[24_000..40_000].iter().fold(0f32, |m, v| m.max(v.abs())) / 0.1).log10()
        };
        for slope in [-6.0, -3.0, 3.0, 6.0] {
            let gain = |freq: f32| peak_db(slope, freq) - peak_db(0.0, freq);
            let at_1k = gain(1000.0);
            for (freq, octaves) in [(500.0, -1.0), (4000.0, 2.0)] {
                let got = gain(freq) - at_1k;
                assert!((got - slope * octaves).abs() < 0.7, "{slope} dB/oct at {freq} Hz: {got:.2} dB from 1 kHz");
            }
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
