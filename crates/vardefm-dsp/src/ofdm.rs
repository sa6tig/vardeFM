//! Real-valued (audio) OFDM modulator and demodulator.
//!
//! Frame layout: `[preamble A][preamble B][data symbol 0] ... [data symbol n-1]`
//! - Preamble A uses only even FFT bins, so its time signal repeats every nfft/2
//!   samples. The receiver finds it with a Schmidl-Cox autocorrelation.
//! - Preamble B is a known BPSK symbol on every carrier. It gives exact timing
//!   (cross-correlation) and the initial channel estimate.
//! - Data symbols carry scattered pilots used to track gain, phase and timing drift
//!   (sound-card clocks differ by tens of ppm).

use crate::filter::Chain;
use crate::params::OfdmParams;
use crate::prng::Prng;
use num_complex::Complex32;
use rustfft::{Fft, FftPlanner};
use std::f32::consts::SQRT_2;
use std::sync::Arc;

const PILOT_TABLE_LEN: usize = 32;
/// Schmidl-Cox plateau threshold. With the metric normalized by the quieter half (and
/// the imbalance capped by `MAX_IMBALANCE_GAIN`), 0.4 keeps nearly all the detection
/// gain of 0.3 near the detection threshold (`detect_eval`) with fewer detections on
/// carrier noise.
const DETECT_THRESHOLD: f64 = 0.4;
/// P²/min(R1, R2)² is ρ²·max/min: within the balance gate it would let a window with
/// halves 10 dB apart pass at ρ² = 0.04. The energy ratio counts at most this much, so
/// such a window needs ρ² above 0.4/3. In `detect_eval` the cap took detections on a
/// weak carrier (5 dB CNR) from 6-10 per minute to 0-2, below the classic detector's
/// 1-10, and cost at most 3 of 100 bursts at any CNR (none significant).
const MAX_IMBALANCE_GAIN: f64 = 3.0;
/// Schmidl-Cox windows whose halves differ in energy by more than this ratio (10 dB)
/// get metric 0. The metric is normalized by the quieter half, so without the gate a
/// window straddling a level step (a tone keying up, a ring-down, the start of a
/// burst) would score up to the energy ratio. FM noise in one half of preamble A stays
/// inside the gate while its power is below 9 times the signal's (amplitude about 3
/// times); the tests reach twice the amplitude.
const MIN_HALF_ENERGY_RATIO: f64 = 0.1;
/// Fine timing searches the whole plateau, but at most this many cyclic prefixes
/// either side of its centre: a long plateau from a periodic tone would otherwise
/// give a search of thousands of offsets. A real plateau, skewed by noise, needed up
/// to about 3 cyclic prefixes.
const MAX_TIMING_SPAN_CP: usize = 4;
/// A click-hit symbol gets a per-carrier noise profile only when the mean pilot noise
/// of one half of the band exceeds the other's by this ratio (see `click_profile`).
const PROFILE_MIN_HALF_RATIO: f32 = 4.0;
/// Pilots are averaged over this many symbols either side.
const TRACK_HALF_WINDOW: usize = 4;
const MIN_PREAMBLE_CORR: f32 = 0.3;
/// A symbol whose pilot noise exceeds this many times the typical level gets its own
/// noise scale. Normal symbols almost never get there (the per-symbol estimate averages
/// 18-24 real noise samples).
const SCALE_THRESHOLD: f32 = 2.0;
/// TX low-pass cutoff relative to the top carrier frequency.
const TX_LOWPASS_FACTOR: f32 = 1.15;
/// Clip-and-filter passes before the final clamp.
const CLIP_FILTER_PASSES: usize = 2;

pub struct Ofdm {
    pub p: OfdmParams,
    fft: Arc<dyn Fft<f32>>,
    ifft: Arc<dyn Fft<f32>>,
    /// Carrier indices (0..num_carriers) that hold pilots.
    pub pilot_idx: Vec<usize>,
    /// Carrier indices that hold data.
    pub data_idx: Vec<usize>,
    pilot_table: Vec<Vec<f32>>,
    gains: Vec<f32>,
    pre_a: Vec<Complex32>,
    pre_b: Vec<Complex32>,
    /// Time-domain body (no cyclic prefix) of preamble B, unit energy.
    pre_b_body: Vec<f32>,
}

/// A demodulated frame.
pub struct Frame {
    /// Equalized data-carrier symbols, one Vec per OFDM symbol.
    pub symbols: Vec<Vec<Complex32>>,
    /// Estimated noise variance after equalization, per data carrier.
    pub noise_var: Vec<f32>,
    /// Noise scale per OFDM symbol (at least 1). The noise variance of symbol `i`,
    /// data carrier `j` is `noise_var[j] * symbol_noise_scale[i]` unless
    /// `carrier_noise_scale[i]` overrides it (see [`Frame::noise_at`]); it is large for
    /// a symbol hit by an FM click, a squelch pop or a short fade.
    pub symbol_noise_scale: Vec<f32>,
    /// Per symbol: empty when `symbol_noise_scale[i]` applies to every carrier, else
    /// one scale per data carrier (at least 1) that replaces it. Set for a symbol with
    /// a scale above 1 whose pilots show its noise to be uneven across the band (an
    /// interferer in part of the band, an FM click in a fast fade: up to 800 times the
    /// normal level mid-band against a few times on the lowest carriers). There the one
    /// per-symbol scale, an average over the pilots, overstates the noise on some
    /// carriers and understates it on others. See `click_profile` for when it is set.
    ///
    /// It helps when such symbols are frequent enough to break codewords (the test
    /// `coloured_clicks_need_the_per_carrier_profile`); on the simulated fading
    /// channels the gain is a few codewords in a hundred or none.
    ///
    /// [`Frame::noise_at`] uses it in place of the symbol's scale, so to change a
    /// symbol's noise, change both fields (or clear this one).
    pub carrier_noise_scale: Vec<Vec<f32>>,
    /// Average SNR per symbol (Es/N0) in dB.
    pub snr_db: f32,
    /// Sample index just past the end of the frame.
    pub end: usize,
}

impl Frame {
    /// Noise variance of symbol `i`, data carrier `j`.
    ///
    /// Panics if `i` or `j` is out of range (`symbols.len()`, `noise_var.len()`).
    pub fn noise_at(&self, i: usize, j: usize) -> f32 {
        let c = self.carrier_noise_scale.get(i).map_or(&[][..], |c| &c[..]);
        debug_assert!(
            c.is_empty() || c.len() == self.noise_var.len(),
            "carrier_noise_scale[{i}] has {} entries",
            c.len()
        );
        self.noise_var[j] * c.get(j).copied().unwrap_or(self.symbol_noise_scale[i])
    }
}

impl Ofdm {
    pub fn new(p: OfdmParams) -> Self {
        let mut planner = FftPlanner::new();
        let fft = planner.plan_fft_forward(p.nfft);
        let ifft = planner.plan_fft_inverse(p.nfft);
        let nc = p.num_carriers;

        let (pilot_idx, data_idx): (Vec<usize>, Vec<usize>) =
            (0..nc).partition(|&k| k % p.pilot_spacing == 0 || k == nc - 1);

        let mut rng = Prng::new(0x5eed_0001);
        let pilot_table = (0..PILOT_TABLE_LEN).map(|_| pilot_idx.iter().map(|_| rng.bpsk()).collect()).collect();

        let f0 = p.carrier_freq(0);
        let gains = (0..nc)
            .map(|k| {
                let octaves = (p.carrier_freq(k) / f0).log2();
                10f32.powf(p.tilt_db_per_octave * octaves / 20.0)
            })
            .collect();

        let mut rng = Prng::new(0x5eed_00a1);
        let pre_a = (0..nc)
            .map(|k| {
                let v = rng.bpsk();
                if (p.first_bin + k).is_multiple_of(2) {
                    Complex32::new(SQRT_2 * v, 0.0)
                } else {
                    Complex32::new(0.0, 0.0)
                }
            })
            .collect();
        let mut rng = Prng::new(0x5eed_00b2);
        let pre_b = (0..nc).map(|_| Complex32::new(rng.bpsk(), 0.0)).collect();

        let mut s =
            Self { p, fft, ifft, pilot_idx, data_idx, pilot_table, gains, pre_a, pre_b, pre_b_body: Vec::new() };
        let b = s.symbol_time(&s.pre_b);
        let mut body = b[s.p.ncp..].to_vec();
        let e = body.iter().map(|x| x * x).sum::<f32>().sqrt();
        body.iter_mut().for_each(|x| *x /= e);
        s.pre_b_body = body;
        s
    }

    pub fn data_carriers(&self) -> usize {
        self.data_idx.len()
    }

    /// Length in samples of a frame with `nsym` data symbols.
    pub fn frame_len(&self, nsym: usize) -> usize {
        (nsym + 2) * self.p.symbol_len()
    }

    fn pilot_value(&self, sym: usize, n: usize) -> f32 {
        self.pilot_table[sym % PILOT_TABLE_LEN][n]
    }

    /// One OFDM symbol (cyclic prefix + body) from carrier values.
    fn symbol_time(&self, carriers: &[Complex32]) -> Vec<f32> {
        let n = self.p.nfft;
        let mut x = vec![Complex32::new(0.0, 0.0); n];
        for (k, &v) in carriers.iter().enumerate() {
            let bin = self.p.first_bin + k;
            let v = v * self.gains[k];
            x[bin] = v;
            x[n - bin] = v.conj();
        }
        self.ifft.process(&mut x);
        let scale = 1.0 / (n as f32).sqrt();
        let body: Vec<f32> = x.iter().map(|c| c.re * scale).collect();
        let mut out = Vec::with_capacity(self.p.symbol_len());
        out.extend_from_slice(&body[n - self.p.ncp..]);
        out.extend_from_slice(&body);
        out
    }

    /// Build a frame. Each entry of `data` holds `data_carriers()` symbols.
    pub fn modulate(&self, data: &[Vec<Complex32>]) -> Vec<f32> {
        self.modulate_with_leader(data, 0.0)
    }

    /// Build a frame preceded by at least `leader_s` seconds of filler symbols, at the
    /// mode's reference level `p.tx_rms`.
    /// The leader covers the radio's key-up time (PTT to carrier up and squelch open)
    /// and lets the receiver's audio levels settle. The filler is random QPSK, so it
    /// cannot be mistaken for preamble A.
    pub fn modulate_with_leader(&self, data: &[Vec<Complex32>], leader_s: f32) -> Vec<f32> {
        self.modulate_with_leader_rms(data, leader_s, self.p.tx_rms)
    }

    /// Like [`Ofdm::modulate_with_leader`], at RMS level `rms` (1.0 = full scale).
    ///
    /// The frame is scaled to `rms` before clipping, so the level is exact unless peaks
    /// are clipped. Clipping splatters far outside the carriers, and in wide mode the
    /// audio goes straight to the radio's VCO, so the output is shaped:
    /// 1. A low-pass just above the top carrier removes the sidelobes of the symbol edges.
    /// 2. Clip and filter: what clipping at full scale would remove is low-passed and
    ///    subtracted. The correction is filtered forwards and backwards (zero phase), so it
    ///    stays aligned with the peaks it cuts; each pass leaves less overshoot.
    /// 3. A final clamp to ±1.0 catches what overshoot is left.
    ///
    /// The low-pass is linear and time-invariant and the corrections only touch peaks, so
    /// preamble A stays periodic enough for detection.
    pub fn modulate_with_leader_rms(&self, data: &[Vec<Complex32>], leader_s: f32, rms: f32) -> Vec<f32> {
        let mut out = self.frame_samples(data, leader_s);
        let now = (out.iter().map(|x| x * x).sum::<f32>() / out.len() as f32).sqrt();
        let g = rms / now.max(1e-12);
        out.iter_mut().for_each(|x| *x *= g);
        let lowpass = || Chain::lowpass4(self.p.fs, self.tx_lowpass_hz());
        lowpass().process_buf(&mut out);
        for _ in 0..CLIP_FILTER_PASSES {
            let mut e: Vec<f32> = out.iter().map(|x| x.clamp(-1.0, 1.0) - x).collect();
            if e.iter().all(|&v| v == 0.0) {
                break;
            }
            lowpass().process_buf(&mut e);
            e.reverse();
            lowpass().process_buf(&mut e);
            e.reverse();
            out.iter_mut().zip(&e).for_each(|(x, e)| *x += e);
        }
        out.iter_mut().for_each(|x| *x = x.clamp(-1.0, 1.0));
        out
    }

    /// Unscaled samples of a frame with its leader.
    fn frame_samples(&self, data: &[Vec<Complex32>], leader_s: f32) -> Vec<f32> {
        let nc = self.p.num_carriers;
        let n_leader = (leader_s / self.p.symbol_duration()).ceil() as usize;
        let mut out = Vec::with_capacity(self.frame_len(data.len()) + n_leader * self.p.symbol_len());
        let mut rng = Prng::new(0x5eed_00c3);
        for _ in 0..n_leader {
            let c: Vec<Complex32> = (0..nc).map(|_| Complex32::new(rng.bpsk(), rng.bpsk()) * (1.0 / SQRT_2)).collect();
            out.extend(self.symbol_time(&c));
        }
        out.extend(self.symbol_time(&self.pre_a));
        out.extend(self.symbol_time(&self.pre_b));
        for (i, d) in data.iter().enumerate() {
            assert_eq!(d.len(), self.data_idx.len(), "wrong number of data symbols");
            let mut c = vec![Complex32::new(0.0, 0.0); nc];
            for (j, &k) in self.data_idx.iter().enumerate() {
                c[k] = d[j];
            }
            for (n, &k) in self.pilot_idx.iter().enumerate() {
                c[k] = Complex32::new(self.pilot_value(i, n), 0.0);
            }
            out.extend(self.symbol_time(&c));
        }
        out
    }

    /// Cutoff of the TX splatter filter.
    pub fn tx_lowpass_hz(&self) -> f32 {
        self.p.carrier_freq(self.p.num_carriers - 1) * TX_LOWPASS_FACTOR
    }

    /// FFT of the window starting at `pos`, returning only the carrier bins.
    fn carriers_at(&self, rx: &[f32], pos: isize) -> Option<Vec<Complex32>> {
        let n = self.p.nfft;
        if pos < 0 || pos as usize + n > rx.len() {
            return None;
        }
        let pos = pos as usize;
        let mut x: Vec<Complex32> = rx[pos..pos + n].iter().map(|&v| Complex32::new(v, 0.0)).collect();
        self.fft.process(&mut x);
        let scale = 1.0 / (n as f32).sqrt();
        Some((0..self.p.num_carriers).map(|k| x[self.p.first_bin + k] * scale).collect())
    }

    /// Search `rx` from `from` for a frame. Returns the sample index where the
    /// body of preamble B starts.
    pub fn detect(&self, rx: &[f32], from: usize) -> Option<usize> {
        self.detect_with(rx, from, ScConfig::default())
    }

    /// [`Ofdm::detect`] with another Schmidl-Cox configuration (for evaluating them).
    pub fn detect_with(&self, rx: &[f32], from: usize, cfg: ScConfig) -> Option<usize> {
        // Filtering keeps preamble A periodic (the filter is linear and time-invariant),
        // so the autocorrelation still works on the filtered copy.
        let filtered;
        let sc: &[f32] = match self.p.detect_lowpass_hz {
            Some(f) => {
                let mut lp = Chain::lowpass4(self.p.fs, f);
                filtered = rx[from..].iter().map(|&x| lp.process(x)).collect::<Vec<f32>>();
                &filtered
            }
            None => &rx[from..],
        };
        self.detect_in(rx, sc, from, cfg)
    }

    /// Schmidl-Cox search over `sc` (which starts at `rx[from]`), confirmed on `rx`.
    fn detect_in(&self, rx: &[f32], sc: &[f32], from: usize, cfg: ScConfig) -> Option<usize> {
        let n = self.p.nfft;
        let h = n / 2;
        let ncp = self.p.ncp;
        let l = self.p.symbol_len();
        if sc.len() < 3 * l {
            return None;
        }
        let x = |i: usize| sc[i] as f64;
        // See `ScConfig` for how the metric is normalized: FM noise from a fade in either
        // half of preamble A must not push it below the threshold.
        let mut p = 0f64;
        let mut r = 0f64;
        let mut r1 = 0f64;
        for m in 0..h {
            p += x(m) * x(m + h);
            r += x(m + h).powi(2);
            r1 += x(m).powi(2);
        }
        let energy_floor = 1e-7 * h as f64;
        let mut plateau: Option<(usize, usize)> = None;
        let mut d = 0;
        while d + n + 2 * l < sc.len() {
            let metric = if r > energy_floor && r1 > energy_floor { cfg.metric(p, r1, r) } else { 0.0 };
            match (&mut plateau, metric > cfg.threshold) {
                (None, true) => plateau = Some((d, d)),
                (Some((_, end)), true) => *end = d,
                (Some((start, end)), false) => {
                    let (start, end) = (*start, *end);
                    plateau = None;
                    if end - start >= ncp / 4 {
                        // Noise in part of preamble A skews the plateau, so its centre
                        // can miss by more than the cyclic prefix: search all of it, up
                        // to `MAX_TIMING_SPAN_CP` cyclic prefixes either way.
                        let a0 = from + ((start + end) / 2).saturating_sub(ncp / 2);
                        let span = cfg.timing_span(end - start, ncp);
                        if let Some(b) = self.fine_timing(rx, a0 + l + ncp, span) {
                            return Some(b);
                        }
                    }
                }
                (None, false) => {}
            }
            p += x(d + h) * x(d + n) - x(d) * x(d + h);
            r += x(d + n).powi(2) - x(d + h).powi(2);
            r1 += x(d + h).powi(2) - x(d).powi(2);
            d += 1;
        }
        None
    }

    /// Cross-correlate with preamble B within `span` samples of `expected` and return
    /// the best start.
    fn fine_timing(&self, rx: &[f32], expected: usize, span: usize) -> Option<usize> {
        let n = self.p.nfft;
        let span = span as isize;
        let mut best = (0.0f32, 0usize);
        for off in -span..=span {
            let s = expected as isize + off;
            if s < 0 || s as usize + n > rx.len() {
                continue;
            }
            let w = &rx[s as usize..s as usize + n];
            let e = w.iter().map(|x| x * x).sum::<f32>().sqrt();
            if e <= 0.0 {
                continue;
            }
            let c = w.iter().zip(&self.pre_b_body).map(|(a, b)| a * b).sum::<f32>().abs() / e;
            if c > best.0 {
                best = (c, s as usize);
            }
        }
        (best.0 >= MIN_PREAMBLE_CORR).then_some(best.1)
    }

    /// Demodulate `nsym` data symbols of a frame whose preamble B starts at `b_start`.
    /// Returns `None` if `nsym` is 0 or the frame runs past the end of `rx`.
    pub fn demodulate(&self, rx: &[f32], b_start: usize, nsym: usize) -> Option<Frame> {
        self.demodulate_known(rx, b_start, nsym, &[])
    }

    /// Like [`demodulate`](Self::demodulate), but the data-carrier values of the first
    /// `known.len()` symbols are known (for example a header that has already been
    /// decoded). Those symbols train the channel estimate together with preamble B.
    pub fn demodulate_known(&self, rx: &[f32], b_start: usize, nsym: usize, known: &[Vec<Complex32>]) -> Option<Frame> {
        if nsym == 0 {
            return None;
        }
        let n = self.p.nfft;
        let l = self.p.symbol_len() as isize;
        let nc = self.p.num_carriers;
        // Centre the FFT window in the cyclic prefix. That leaves ncp/2 samples of
        // slack each way for sound-card clock drift without ever moving the window.
        let base = b_start as isize - (self.p.ncp / 2) as isize;

        let yb = self.carriers_at(rx, base)?;
        let ys: Vec<Vec<Complex32>> =
            (0..nsym).map(|i| self.carriers_at(rx, base + (i as isize + 1) * l)).collect::<Option<_>>()?;

        // Channel estimate: preamble B averaged with the known symbols. Clock drift moves
        // the timing by at most a sample or so over the header; the pilot tracking below
        // takes that out, so the symbols are simply averaged. (Removing each symbol's own
        // phase slope instead costs up to 2 dB at low SNR: the slope estimate is noisy and
        // its error grows across the band.)
        let nk = known.len().min(nsym);
        let mut obs: Vec<Vec<Complex32>> = Vec::with_capacity(nk + 1);
        obs.push((0..nc).map(|k| yb[k] / self.pre_b[k]).collect());
        for (i, d) in known[..nk].iter().enumerate() {
            assert_eq!(d.len(), self.data_idx.len(), "wrong number of known symbols");
            let mut o = vec![Complex32::new(0.0, 0.0); nc];
            for (j, &k) in self.data_idx.iter().enumerate() {
                o[k] = ys[i][k] / d[j];
            }
            for (m, &k) in self.pilot_idx.iter().enumerate() {
                o[k] = ys[i][k] * self.pilot_value(i, m);
            }
            obs.push(o);
        }
        // A header symbol hit by an FM click can still decode (the header code is rate
        // 0.24) and so be offered as training. At full weight it would corrupt the
        // estimate for every carrier of the burst, which the per-symbol noise scale
        // below cannot undo. So each training symbol is weighted by the inverse of its
        // noise, and `ntrain` is the effective number of symbols averaged.
        let weights = training_weights(&obs);
        let ntrain: f32 = weights.iter().sum();
        let acc: Vec<Complex32> =
            (0..nc).map(|k| obs.iter().zip(&weights).map(|(o, &w)| o[k] * w).sum::<Complex32>() / ntrain).collect();
        // Light [1 2 1]/4 smoothing across frequency (the audio channel response is
        // smooth). It still pays with the header averaged in: dropping it costs about
        // 0.5 dB in wide mode. Remove the linear phase from the timing offset first, or
        // smoothing would average over a rotating phase.
        let (slope, flat) = remove_slope(&acc);
        let hb: Vec<Complex32> = (0..nc)
            .map(|k| {
                let lo = flat[k.saturating_sub(1)];
                let hi = flat[(k + 1).min(nc - 1)];
                (lo + flat[k] * 2.0 + hi) * 0.25 * Complex32::from_polar(1.0, slope * k as f32)
            })
            .collect();
        // Noise the estimate adds to every equalized symbol, relative to one symbol's:
        // the kernel passes 3/8 of the noise of the `ntrain` averaged symbols.
        let est_noise = 0.375 / ntrain;

        // Pilot ratios against the channel estimate.
        let cs: Vec<Vec<Complex32>> = ys
            .iter()
            .enumerate()
            .map(|(i, y)| {
                self.pilot_idx.iter().enumerate().map(|(m, &k)| y[k] / (hb[k] * self.pilot_value(i, m))).collect()
            })
            .collect();

        // Average each pilot over neighbouring symbols, weighting each symbol by the
        // inverse of its noise. The first pass (equal weights) finds symbols hit by an
        // FM click or a fade; the second keeps them out of their neighbours' averages.
        let (_, dev) = track_pilots(&cs, &vec![1.0; nsym]);
        let (scale, _) = noise_model(&dev);
        let (avg, dev) = track_pilots(&cs, &scale);
        let (scale, pilot_var) = noise_model(&dev);

        // Interpolate between pilots and equalize. Each carrier only depends on nearby
        // pilots, so noisy pilots at one end of the band do not corrupt carriers at the
        // other end.
        let symbols: Vec<Vec<Complex32>> = (0..nsym)
            .map(|i| self.data_idx.iter().map(|&k| ys[i][k] / (hb[k] * self.interp_pilots(&avg[i], k))).collect())
            .collect();

        // Noise per data carrier, interpolated from the pilots, plus the noise of the
        // channel estimate.
        let raw_var: Vec<f32> = self
            .data_idx
            .iter()
            .map(|&k| {
                let (m, t) = self.pilot_pos(k);
                (pilot_var[m] * (1.0 - t) + pilot_var[m + 1] * t) * (1.0 + est_noise)
            })
            .collect();
        let noise_var: Vec<f32> = raw_var.iter().map(|v| v.max(1e-6)).collect();
        // A symbol with its own noise scale gets a per-carrier profile from its own
        // pilots when they show its noise to be uneven across the band
        // ([`click_profile`]), interpolated to the data carriers.
        let carrier_noise_scale: Vec<Vec<f32>> = (0..nsym)
            .map(|i| {
                if scale[i] <= 1.0 {
                    return Vec::new();
                }
                let Some(s) = click_profile(&dev[i], &pilot_var) else { return Vec::new() };
                self.data_idx
                    .iter()
                    .map(|&k| {
                        let (m, t) = self.pilot_pos(k);
                        s[m] * (1.0 - t) + s[m + 1] * t
                    })
                    .collect()
            })
            .collect();
        // The reported SNR averages the noise over all symbols, click-hit ones included,
        // as it did before there was a per-symbol scale. Scale the unclamped variance:
        // on a nearly noiseless frame the floor would otherwise be multiplied too.
        // It uses the per-symbol scales, not `carrier_noise_scale` (which only refines
        // the LLRs), so that it stays comparable with the link's SNR thresholds.
        let mean_var = raw_var.iter().sum::<f32>() / raw_var.len() as f32;
        let mean_scale = scale.iter().sum::<f32>() / nsym.max(1) as f32;

        Some(Frame {
            symbols,
            noise_var,
            symbol_noise_scale: scale,
            carrier_noise_scale,
            snr_db: -10.0 * (mean_var * mean_scale.max(1.0)).max(1e-6).log10(),
            end: (base + (nsym as isize + 1) * l + n as isize) as usize,
        })
    }

    /// Index of the pilot at or below carrier `k` and the fractional position towards the next one.
    fn pilot_pos(&self, k: usize) -> (usize, f32) {
        let hi = self.pilot_idx.partition_point(|&p| p < k).clamp(1, self.pilot_idx.len() - 1);
        let (k0, k1) = (self.pilot_idx[hi - 1], self.pilot_idx[hi]);
        (hi - 1, (k as f32 - k0 as f32) / (k1 - k0) as f32)
    }

    /// Interpolate pilot values at carrier `k`: linear in magnitude and in phase.
    fn interp_pilots(&self, pilots: &[Complex32], k: usize) -> Complex32 {
        let (m, t) = self.pilot_pos(k);
        let (a, b) = (pilots[m], pilots[m + 1]);
        let mag = a.norm() * (1.0 - t) + b.norm() * t;
        let dphi = (b * a.conj()).arg();
        Complex32::from_polar(mag, a.arg() + dphi * t)
    }
}

/// Noise scale per pilot of one symbol, or None when its noise looks flat across the
/// band. Each pilot's squared deviation `dev` over its usual level `pilot_var` is a
/// single noisy sample (2 degrees of freedom), so the profile is smoothed [1 2 1]/4
/// across pilots and is at least 1. Even smoothed it scatters several-fold around a
/// flat level, and on flat noise that cost more decoding than one scale per symbol
/// (on clicks over the whole band hitting every fourth symbol, narrow L6: 22 of 48
/// codewords against 37). So the profile is used only when the two halves of the band
/// differ by more than `PROFILE_MIN_HALF_RATIO` in mean deviation (6 pilots each, about
/// 12 degrees of freedom): then the same flat-click case decodes 36 of 48.
fn click_profile(dev: &[f32], pilot_var: &[f32]) -> Option<Vec<f32>> {
    let np = pilot_var.len();
    let r: Vec<f32> = dev.iter().zip(pilot_var).map(|(d, v)| d / v.max(1e-12)).collect();
    let half = |rr: &[f32]| rr.iter().sum::<f32>() / rr.len().max(1) as f32;
    let (a, b) = (half(&r[..np / 2]), half(&r[np / 2..]));
    if a.max(b) < PROFILE_MIN_HALF_RATIO * a.min(b) {
        return None;
    }
    Some((0..np).map(|m| ((r[m.saturating_sub(1)] + 2.0 * r[m] + r[(m + 1).min(np - 1)]) * 0.25).max(1.0)).collect())
}

/// How the Schmidl-Cox metric (correlation P between the two halves of the window,
/// energies R1 and R2) is normalized.
///
/// FM noise from a fade adds energy to one half of preamble A but little to P. The
/// classic P²/R2² then falls by the square of the energy ratio when the noise is in
/// the second half and ignores it in the first; P²/(R1·R2) falls by the ratio either
/// way; P²/min(R1, R2)² does not fall while the halves stay within the balance gate.
/// That last one is ρ²·max(R1, R2)/min(R1, R2), so it also lowers the bar for any
/// window of unequal halves; [`ScConfig::max_imbalance_gain`] limits how far. On the
/// simulator it detects the most bursts, on a static channel near the threshold as well
/// as on fading ones: see `detect_eval` in vardefm-modem.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScMetric {
    /// P² / R2²: the classic form.
    SecondHalf,
    /// P² / (R1·R2): the squared correlation coefficient.
    BothHalves,
    /// P² / min(R1, R2)², with the energy ratio capped by
    /// [`ScConfig::max_imbalance_gain`].
    QuieterHalf,
}

/// Schmidl-Cox detection settings. `Default` is what [`Ofdm::detect`] uses.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScConfig {
    pub metric: ScMetric,
    /// A plateau is where the metric exceeds this.
    pub threshold: f64,
    /// Windows whose halves differ in energy by more than this ratio get metric 0
    /// (0 = no gate).
    pub min_half_energy_ratio: f64,
    /// For [`ScMetric::QuieterHalf`]: the metric is ρ²·max(R1, R2)/min(R1, R2), and
    /// the energy ratio counts at most this much (infinity = no cap).
    pub max_imbalance_gain: f64,
    /// Search fine timing over the whole plateau, up to this many cyclic prefixes
    /// either way (0 = ±1 cyclic prefix around the plateau centre).
    pub max_span_cp: usize,
}

impl Default for ScConfig {
    fn default() -> Self {
        Self {
            metric: ScMetric::QuieterHalf,
            threshold: DETECT_THRESHOLD,
            min_half_energy_ratio: MIN_HALF_ENERGY_RATIO,
            max_imbalance_gain: MAX_IMBALANCE_GAIN,
            max_span_cp: MAX_TIMING_SPAN_CP,
        }
    }
}

impl ScConfig {
    /// The detector before fading hardening.
    pub const CLASSIC: Self = Self {
        metric: ScMetric::SecondHalf,
        threshold: 0.3,
        min_half_energy_ratio: 0.0,
        max_imbalance_gain: f64::INFINITY,
        max_span_cp: 0,
    };

    /// Metric of a window with correlation `p` between halves of energy `r1`, `r2`.
    fn metric(&self, p: f64, r1: f64, r2: f64) -> f64 {
        if p <= 0.0 || r1.min(r2) < self.min_half_energy_ratio * r1.max(r2) {
            return 0.0;
        }
        p * p
            / match self.metric {
                ScMetric::SecondHalf => r2 * r2,
                ScMetric::BothHalves => r1 * r2,
                ScMetric::QuieterHalf => {
                    let (lo, hi) = (r1.min(r2), r1.max(r2));
                    lo * hi / (hi / lo).min(self.max_imbalance_gain)
                }
            }
    }

    /// Fine-timing search span either side of the expected position, for a plateau of
    /// `plateau` samples.
    fn timing_span(&self, plateau: usize, ncp: usize) -> usize {
        if self.max_span_cp == 0 {
            ncp
        } else {
            (plateau / 2 + ncp / 2).clamp(ncp, self.max_span_cp * ncp)
        }
    }
}

/// Split the linear phase (the timing offset) off a per-carrier channel estimate.
/// Returns the slope in radians per carrier and the estimate without it.
fn remove_slope(raw: &[Complex32]) -> (f32, Vec<Complex32>) {
    let rot: Complex32 = raw.windows(2).map(|w| w[1] * w[0].conj()).sum();
    let slope = rot.arg();
    (slope, raw.iter().enumerate().map(|(k, &h)| h * Complex32::from_polar(1.0, -slope * k as f32)).collect())
}

/// Average each pilot over up to `TRACK_HALF_WINDOW` symbols either side, weighting
/// symbol `i` by `1 / scale[i]`. Returns the averages and each pilot's squared
/// deviation from its average, corrected for the pilot's own share of that average.
fn track_pilots(cs: &[Vec<Complex32>], scale: &[f32]) -> (Vec<Vec<Complex32>>, Vec<Vec<f32>>) {
    let nsym = cs.len();
    let np = cs.first().map_or(0, |c| c.len());
    let mut avgs = Vec::with_capacity(nsym);
    let mut devs = Vec::with_capacity(nsym);
    for i in 0..nsym {
        let lo = i.saturating_sub(TRACK_HALF_WINDOW);
        let hi = (i + TRACK_HALF_WINDOW + 1).min(nsym);
        let wsum: f32 = scale[lo..hi].iter().map(|s| 1.0 / s).sum();
        let avg: Vec<Complex32> =
            (0..np).map(|m| (lo..hi).map(|j| cs[j][m] / scale[j]).sum::<Complex32>() / wsum).collect();
        // With weights matched to the noise, the deviation's variance is the symbol's
        // own noise times (1 - its share of the average).
        let share = 1.0 / scale[i] / wsum;
        let dev = if share < 0.999 {
            let bias = 1.0 / (1.0 - share);
            (0..np).map(|m| (cs[i][m] / avg[m] - 1.0).norm_sqr() * bias).collect()
        } else {
            vec![0.0; np]
        };
        avgs.push(avg);
        devs.push(dev);
    }
    (avgs, devs)
}

/// Weight per channel observation (one per training symbol): 1 for a symbol whose
/// deviation from the plain average is at most `SCALE_THRESHOLD` times the typical
/// level, otherwise typical / own, so that a symbol hit by a click adds almost nothing.
/// Fewer than three observations cannot show an outlier and all get weight 1.
fn training_weights(obs: &[Vec<Complex32>]) -> Vec<f32> {
    let n = obs.len();
    if n < 3 {
        return vec![1.0; n];
    }
    let nc = obs[0].len();
    let mean: Vec<Complex32> = (0..nc).map(|k| obs.iter().map(|o| o[k]).sum::<Complex32>() / n as f32).collect();
    let e: Vec<f32> = obs.iter().map(|o| o.iter().zip(&mean).map(|(x, h)| (x - h).norm_sqr()).sum()).collect();
    let reference = robust_reference(&e);
    e.iter().map(|x| x / reference).map(|r| if r > SCALE_THRESHOLD { 1.0 / r } else { 1.0 }).collect()
}

/// Typical level of `e`: the mean over the values not far above the median, so that
/// the outliers (clicks) do not raise it.
fn robust_reference(e: &[f32]) -> f32 {
    let mut sorted = e.to_vec();
    sorted.sort_by(f32::total_cmp);
    let median = sorted[e.len() / 2];
    let (sum, cnt) = e.iter().filter(|&&x| x <= 3.0 * median).fold((0.0, 0), |(s, c), &x| (s + x, c + 1));
    (sum / cnt.max(1) as f32).max(1e-12)
}

/// Fit the pilot deviations to a separable noise model: variance = per-pilot variance
/// × per-symbol scale. Returns (scale per symbol, at least 1; variance per pilot).
///
/// A symbol's scale is 1 unless its noise is more than `SCALE_THRESHOLD` times the
/// typical level: with 9-12 pilots per symbol the estimate is noisy, and the point is
/// to catch symbols hit by a click, not to fine-tune the rest. (Scaling every symbol by
/// `max(1, e/mean)` cost a little at the 64QAM levels.)
fn noise_model(dev: &[Vec<f32>]) -> (Vec<f32>, Vec<f32>) {
    let nsym = dev.len();
    let np = dev.first().map_or(0, |d| d.len());
    let per_pilot = |scale: &[f32]| -> Vec<f32> {
        (0..np).map(|m| dev.iter().zip(scale).map(|(d, s)| d[m] / s).sum::<f32>() / nsym.max(1) as f32).collect()
    };
    let ones = vec![1.0; nsym];
    let var = per_pilot(&ones);
    if nsym < 3 {
        return (ones, var);
    }
    // Noise energy per symbol, each pilot relative to its own average.
    let e: Vec<f32> =
        dev.iter().map(|d| d.iter().zip(&var).map(|(x, v)| x / v.max(1e-12)).sum::<f32>() / np as f32).collect();
    let reference = robust_reference(&e);
    let scale: Vec<f32> = e.iter().map(|x| x / reference).map(|r| if r > SCALE_THRESHOLD { r } else { 1.0 }).collect();
    let var = per_pilot(&scale);
    (scale, var)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::qam::Modulation;

    fn roundtrip(p: OfdmParams, m: Modulation, lead: usize) {
        let ofdm = Ofdm::new(p);
        let nsym = 20;
        let mut rng = Prng::new(42);
        let bits = rng.bits(nsym * ofdm.data_carriers() * m.bits_per_symbol());
        let syms = m.map_all(&bits);
        let data: Vec<Vec<Complex32>> = syms.chunks(ofdm.data_carriers()).map(|c| c.to_vec()).collect();
        let mut rx = vec![0.0; lead];
        rx.extend(ofdm.modulate(&data));
        rx.extend(vec![0.0; 2000]);

        let b = ofdm.detect(&rx, 0).expect("frame not detected");
        let frame = ofdm.demodulate(&rx, b, nsym).expect("demod failed");
        let mut out = Vec::new();
        for s in frame.symbols.iter().flatten() {
            m.hard_bits(*s, &mut out);
        }
        assert_eq!(out, bits);
        assert!(frame.snr_db > 30.0, "snr {}", frame.snr_db);
        // No symbols is not a frame (the noise model would have no pilots to fit).
        assert!(ofdm.demodulate(&rx, b, 0).is_none());
    }

    #[test]
    fn clean_roundtrip_narrow() {
        roundtrip(OfdmParams::narrow(), Modulation::Qam64, 3333);
    }

    #[test]
    fn clean_roundtrip_wide() {
        roundtrip(OfdmParams::wide(), Modulation::Qam64, 1234);
    }

    fn random_frame(ofdm: &Ofdm, m: Modulation, nsym: usize, seed: u32) -> Vec<Vec<Complex32>> {
        let mut rng = Prng::new(seed);
        let bits = rng.bits(nsym * ofdm.data_carriers() * m.bits_per_symbol());
        m.map_all(&bits).chunks(ofdm.data_carriers()).map(|c| c.to_vec()).collect()
    }

    fn rms(x: &[f32]) -> f32 {
        (x.iter().map(|v| v * v).sum::<f32>() / x.len() as f32).sqrt()
    }

    /// Power in each [lo, hi) Hz band, from a Hann-windowed Welch average.
    fn band_power(x: &[f32], fs: f32, bands: &[(f32, f32)]) -> Vec<f32> {
        let n = 2048;
        let fft = FftPlanner::new().plan_fft_forward(n);
        let win: Vec<f32> =
            (0..n).map(|i| 0.5 - 0.5 * (2.0 * std::f32::consts::PI * i as f32 / n as f32).cos()).collect();
        let mut psd = vec![0f32; n / 2];
        for blk in x.windows(n).step_by(n / 2) {
            let mut b: Vec<Complex32> = blk.iter().zip(&win).map(|(&v, &w)| Complex32::new(v * w, 0.0)).collect();
            fft.process(&mut b);
            psd.iter_mut().zip(&b).for_each(|(p, c)| *p += c.norm_sqr());
        }
        let hz = fs / n as f32;
        bands
            .iter()
            .map(|&(lo, hi)| {
                psd.iter().enumerate().filter(|(i, _)| (lo..hi).contains(&(*i as f32 * hz))).map(|(_, p)| p).sum()
            })
            .collect()
    }

    #[test]
    fn frame_rms_matches_request() {
        for p in [OfdmParams::narrow(), OfdmParams::wide()] {
            let ofdm = Ofdm::new(p);
            let data = random_frame(&ofdm, Modulation::Qam16, 40, 7);
            // Levels with (almost) no clipping: only the filter can move the level.
            for want in [0.05, 0.1, 0.2] {
                let got = rms(&ofdm.modulate_with_leader_rms(&data, 0.1, want));
                assert!((got / want - 1.0).abs() < 0.03, "{} rms {got} want {want}", ofdm.p.name);
            }
            assert_eq!(ofdm.modulate(&data), ofdm.modulate_with_leader_rms(&data, 0.0, ofdm.p.tx_rms));
        }
    }

    #[test]
    fn clipping_splatter_is_filtered() {
        for p in [OfdmParams::narrow(), OfdmParams::wide()] {
            let ofdm = Ofdm::new(p);
            let fs = ofdm.p.fs;
            let top = ofdm.p.carrier_freq(ofdm.p.num_carriers - 1);
            let bands = [(ofdm.p.carrier_freq(0), top), (1.3 * top, fs / 2.0)];
            let data = random_frame(&ofdm, Modulation::Qam64, 60, 9);
            // Louder than the loudest level the modem uses (0.5).
            let level = 0.55;
            let tx = ofdm.modulate_with_leader_rms(&data, 0.1, level);
            assert!(tx.iter().all(|v| v.abs() <= 1.0));
            let pw = band_power(&tx, fs, &bands);
            let oob_db = 10.0 * (pw[1] / pw[0]).log10();
            // The same frame scaled and clipped without shaping, for comparison.
            let mut raw = ofdm.frame_samples(&data, 0.1);
            let g = level / rms(&raw);
            raw.iter_mut().for_each(|v| *v = (*v * g).clamp(-1.0, 1.0));
            let clipped = raw.iter().filter(|v| v.abs() >= 1.0).count() as f32 / raw.len() as f32;
            assert!(clipped > 0.04, "{}: only {clipped} of samples clipped", ofdm.p.name);
            let pr = band_power(&raw, fs, &bands);
            let raw_db = 10.0 * (pr[1] / pr[0]).log10();
            println!("{}: out-of-band {oob_db:.1} dB (clipped only: {raw_db:.1} dB)", ofdm.p.name);
            assert!(oob_db < -30.0, "{}: out-of-band power {oob_db:.1} dB", ofdm.p.name);
        }
    }

    #[test]
    fn clipped_frame_still_decodes() {
        for p in [OfdmParams::narrow(), OfdmParams::wide()] {
            let ofdm = Ofdm::new(p);
            let m = Modulation::Qpsk;
            let mut rng = Prng::new(5);
            let bits = rng.bits(20 * ofdm.data_carriers() * m.bits_per_symbol());
            let data: Vec<Vec<Complex32>> = m.map_all(&bits).chunks(ofdm.data_carriers()).map(|c| c.to_vec()).collect();
            let mut rx = vec![0.0; 777];
            rx.extend(ofdm.modulate_with_leader_rms(&data, 0.1, 0.5));
            rx.extend(vec![0.0; 2000]);
            let b = ofdm.detect(&rx, 0).expect("frame not detected");
            let frame = ofdm.demodulate(&rx, b, data.len()).expect("demod failed");
            let mut out = Vec::new();
            for s in frame.symbols.iter().flatten() {
                m.hard_bits(*s, &mut out);
            }
            assert_eq!(out, bits, "{}", ofdm.p.name);
            println!("{}: SNR at rms 0.5 without noise {:.1} dB", ofdm.p.name, frame.snr_db);
        }
    }

    /// In-band noise: random OFDM symbols (no preamble), scaled to `rms`.
    fn inband_noise(ofdm: &Ofdm, len: usize, rms_level: f32, seed: u32) -> Vec<f32> {
        let mut rng = Prng::new(seed);
        let nc = ofdm.p.num_carriers;
        let mut x = Vec::with_capacity(len + ofdm.p.symbol_len());
        while x.len() < len {
            let c: Vec<Complex32> = (0..nc).map(|_| Complex32::new(rng.bpsk(), rng.bpsk())).collect();
            x.extend(ofdm.symbol_time(&c));
        }
        x.truncate(len);
        let g = rms_level / rms(&x);
        x.iter_mut().for_each(|v| *v *= g);
        x
    }
    /// A frame after 1000 samples of silence, and where `detect` finds it.
    fn frame_and_position(ofdm: &Ofdm) -> (Vec<f32>, usize) {
        let data = random_frame(ofdm, Modulation::Qpsk, 6, 9);
        let mut rx = vec![0.0; 1000];
        rx.extend(ofdm.modulate_with_leader(&data, 0.1));
        rx.extend(vec![0.0; 2000]);
        let b = ofdm.detect(&rx, 0).expect("frame not detected");
        (rx, b)
    }

    /// How many of 20 noise bursts (seeds) in one half of preamble A's body, at
    /// `level` times its RMS, still let `cfg` find the frame at its clean position.
    fn detected_with_burst(ofdm: &Ofdm, cfg: ScConfig, second_half: bool, level: f32) -> usize {
        let (rx, clean) = frame_and_position(ofdm);
        let (n, l) = (ofdm.p.nfft, ofdm.p.symbol_len());
        // Preamble B's body starts at `clean`, so A's body at `clean - l`.
        let a_body = clean - l;
        let from = if second_half { a_body + n / 2 } else { a_body };
        let rms_a = rms(&rx[a_body..a_body + n]);
        (0..20)
            .filter(|&seed| {
                let mut rx = rx.clone();
                let noise = inband_noise(ofdm, n / 2, level * rms_a, 100 + seed);
                rx[from..from + n / 2].iter_mut().zip(&noise).for_each(|(x, v)| *x += v);
                ofdm.detect_with(&rx, 0, cfg).is_some_and(|b| b.abs_diff(clean) <= 2)
            })
            .count()
    }

    #[test]
    fn detects_with_a_noise_burst_in_either_half_of_preamble_a() {
        // A fade puts FM noise into part of preamble A. It adds energy to that half
        // but little correlation between the halves, so a metric normalized by the
        // noisy half drops below the threshold: the classic P²/R2² for noise in the
        // second half, P²/(R1·R2) for either. Normalized by the quieter half, with the
        // energy ratio capped at 3, it holds while the noisy half has up to 3 times the
        // energy of the clean one, then falls in proportion. Out of 20 (classic, R1·R2, min²) at
        // the time of writing, noise 2 times the RMS: narrow first half 6/0/14, second
        // 0/2/20; wide first 5/0/14, second 0/0/17. (Classic at its own threshold 0.3,
        // the others at 0.4.)
        let both = ScConfig { metric: ScMetric::BothHalves, ..ScConfig::default() };
        for p in [OfdmParams::narrow(), OfdmParams::wide()] {
            let ofdm = Ofdm::new(p);
            let name = ofdm.p.name;
            for second in [false, true] {
                let half = ["first", "second"][second as usize];
                for level in [1.2, 2.0] {
                    let got = detected_with_burst(&ofdm, ScConfig::default(), second, level);
                    let want = if level < 1.5 { 20 } else { 12 };
                    assert!(got >= want, "{name}, noise {level}x in the {half} half: {got}/20");
                    for (other, cfg) in [("classic", ScConfig::CLASSIC), ("R1·R2", both)] {
                        let o = detected_with_burst(&ofdm, cfg, second, level);
                        assert!(got >= o, "{name}, noise {level}x in the {half} half: {got}/20, {other} {o}/20");
                    }
                }
            }
            let classic = detected_with_burst(&ofdm, ScConfig::CLASSIC, true, 1.2);
            assert!(classic <= 5, "{name}: classic metric found {classic}/20 with noise in the second half");
        }
    }

    #[test]
    fn metric_gates_unbalanced_windows() {
        let cfg = ScConfig::default();
        // Balanced halves: P²/min(R1, R2)².
        assert!((cfg.metric(0.5, 1.0, 2.0) - 0.25).abs() < 1e-12);
        assert!((cfg.metric(0.5, 2.0, 1.0) - 0.25).abs() < 1e-12);
        // A perfectly correlated scaled copy, halves 20 times apart (a ring-down): would
        // score 20 without the gate.
        assert_eq!(cfg.metric(20f64.sqrt(), 1.0, 20.0), 0.0);
        assert_eq!(cfg.metric(20f64.sqrt(), 20.0, 1.0), 0.0);
        // Just inside the gate it counts, the imbalance capped: halves 9 times apart,
        // perfectly correlated (ρ² = 1), score 3, not 9.
        assert!((cfg.metric(3.0, 1.0, 9.0) - 3.0).abs() < 1e-12);
        let uncapped = ScConfig { max_imbalance_gain: f64::INFINITY, ..cfg };
        assert!((uncapped.metric(3.0, 1.0, 9.0) - 9.0).abs() < 1e-12);
        // Anti-correlated halves never count.
        assert_eq!(cfg.metric(-1.0, 1.0, 1.0), 0.0);
        // The classic metric: P²/R2², no gate.
        assert!((ScConfig::CLASSIC.metric(1.0, 100.0, 2.0) - 0.25).abs() < 1e-12);
    }

    #[test]
    fn timing_span_covers_the_plateau_up_to_the_cap() {
        let cfg = ScConfig::default();
        let ncp = 128;
        assert_eq!(cfg.timing_span(10, ncp), ncp);
        assert_eq!(cfg.timing_span(400, ncp), 200 + ncp / 2);
        // A tone plateau of 2 s at 48 kHz.
        assert_eq!(cfg.timing_span(96_000, ncp), MAX_TIMING_SPAN_CP * ncp);
        assert_eq!(ScConfig::CLASSIC.timing_span(96_000, ncp), ncp);
    }

    #[test]
    fn click_profile_spreads_a_lone_outlier() {
        // One pilot 100 times its usual noise, the others normal: the smoothing spreads
        // it over its neighbours instead of trusting one noisy sample.
        let var = vec![0.01; 6];
        let mut dev = vec![0.01; 6];
        dev[2] = 1.0;
        let s = click_profile(&dev, &var).expect("uneven halves");
        let want = [1.0, 103.0 / 4.0, 202.0 / 4.0, 103.0 / 4.0, 1.0, 1.0];
        for (got, want) in s.iter().zip(want) {
            assert!((got - want).abs() < 1e-3, "{s:?}");
        }
        // Never below 1, and the band edges repeat their own value.
        let s = click_profile(&[0.0, 0.0, 0.04], &[0.01; 3]).expect("uneven halves");
        assert_eq!(s[0], 1.0);
        assert!((s[2] - 3.0).abs() < 1e-5, "{s:?}");
    }

    #[test]
    fn click_profile_only_for_uneven_noise() {
        let var = vec![0.01; 12];
        // Flat noise, scattered as single samples are: no profile.
        let flat = [0.3, 2.1, 0.7, 1.4, 0.2, 1.1, 0.9, 2.6, 0.4, 1.0, 1.7, 0.5].map(|r| r * 50.0 * 0.01);
        assert!(click_profile(&flat, &var).is_none());
        // The upper half 3 times the lower: still treated as flat.
        let mild: Vec<f32> = (0..12).map(|m| if m < 6 { 0.1 } else { 0.3 }).collect();
        assert!(click_profile(&mild, &var).is_none());
        // 5 times: a profile.
        let uneven: Vec<f32> = (0..12).map(|m| if m < 6 { 0.1 } else { 0.5 }).collect();
        assert!(click_profile(&uneven, &var).is_some());
    }

    #[test]
    fn no_false_lock_on_keyed_tones_or_inband_noise() {
        // Tones at multiples of fs/(nfft/2) repeat every half window, so the
        // Schmidl-Cox metric sits near 1 while they last; keyed on and off, each
        // plateau ends and goes to fine timing, which must reject it. In-band noise
        // (random OFDM symbols, no preamble) must not lock either. Detections are
        // cheap, but each costs the receiver a header decode. The detection rate on FM
        // noise from a weak carrier (a few per minute at most) needs minutes of audio
        // to measure: only `detect_eval` in vardefm-modem checks it.
        let fs = 48_000.0f32;
        let len = (fs * 3.0) as usize;
        let keyed = |f: f32, shape: &dyn Fn(f32) -> f32| -> Vec<f32> {
            (0..len)
                .map(|i| {
                    let t = i as f32 / fs;
                    let env = if t % 0.5 < 0.3 { shape(t % 0.5) } else { 0.0 };
                    0.3 * env * (std::f32::consts::TAU * f * t).sin()
                })
                .collect()
        };
        for p in [OfdmParams::narrow(), OfdmParams::wide()] {
            let ofdm = Ofdm::new(p);
            let periodic = fs / (ofdm.p.nfft / 2) as f32 * 10.0;
            let cases: Vec<(&str, Vec<f32>)> = vec![
                ("keyed tone", keyed(periodic, &|_| 1.0)),
                ("keyed 1500 Hz", keyed(1500.0, &|_| 1.0)),
                ("ring-down", keyed(periodic, &|t| (-t / 0.02).exp())),
                ("rising tone", keyed(periodic, &|t| (t / 0.3).min(1.0).powi(3))),
            ];
            for (name, x) in cases {
                assert_eq!(ofdm.detect(&x, 0), None, "{}: false lock on {name}", ofdm.p.name);
            }
            let noise = inband_noise(&ofdm, (fs * 20.0) as usize, 0.2, 21);
            let (mut dets, mut pos) = (0, 0);
            while let Some(b) = ofdm.detect(&noise, pos) {
                dets += 1;
                pos = b + 1;
            }
            assert!(dets <= 2, "{}: {dets} detections in 20 s of in-band noise", ofdm.p.name);
        }
    }
}
