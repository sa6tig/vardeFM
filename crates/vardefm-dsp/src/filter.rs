//! Basic filters: biquads, first-order emphasis networks, FIR design and resampling.

use num_complex::Complex32;
use std::f32::consts::PI;

/// Filter state below this is set to zero. Exact silence after a signal decays
/// the state into subnormal numbers, which are very slow on x86 and stay there.
const FLUSH: f32 = 1e-30;

/// Second-order IIR section (transposed direct form II).
#[derive(Clone, Debug)]
pub struct Biquad {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    z1: f32,
    z2: f32,
}

impl Biquad {
    fn from_coeffs(b: [f32; 3], a: [f32; 3]) -> Self {
        Self { b0: b[0] / a[0], b1: b[1] / a[0], b2: b[2] / a[0], a1: a[1] / a[0], a2: a[2] / a[0], z1: 0.0, z2: 0.0 }
    }

    pub fn lowpass(fs: f32, f0: f32, q: f32) -> Self {
        let w0 = 2.0 * PI * f0 / fs;
        let (s, c) = w0.sin_cos();
        let alpha = s / (2.0 * q);
        Self::from_coeffs([(1.0 - c) / 2.0, 1.0 - c, (1.0 - c) / 2.0], [1.0 + alpha, -2.0 * c, 1.0 - alpha])
    }

    pub fn highpass(fs: f32, f0: f32, q: f32) -> Self {
        let w0 = 2.0 * PI * f0 / fs;
        let (s, c) = w0.sin_cos();
        let alpha = s / (2.0 * q);
        Self::from_coeffs([(1.0 + c) / 2.0, -(1.0 + c), (1.0 + c) / 2.0], [1.0 + alpha, -2.0 * c, 1.0 - alpha])
    }

    #[inline]
    pub fn process(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.z1;
        self.z1 = self.b1 * x - self.a1 * y + self.z2;
        self.z2 = self.b2 * x - self.a2 * y;
        if self.z1.abs() < FLUSH {
            self.z1 = 0.0;
        }
        if self.z2.abs() < FLUSH {
            self.z2 = 0.0;
        }
        y
    }
}

/// A cascade of filter sections applied in order.
#[derive(Clone, Debug, Default)]
pub struct Chain {
    biquads: Vec<Biquad>,
    first: Vec<FirstOrder>,
}

const BUTTER4_Q: [f32; 2] = [0.541_196_1, 1.306_563];

impl Chain {
    pub fn new() -> Self {
        Self::default()
    }

    /// 4th-order Butterworth bandpass made of a highpass and a lowpass.
    pub fn bandpass4(fs: f32, lo: f32, hi: f32) -> Self {
        let mut c = Self::new();
        for q in BUTTER4_Q {
            c.biquads.push(Biquad::highpass(fs, lo, q));
            c.biquads.push(Biquad::lowpass(fs, hi, q));
        }
        c
    }

    /// This chain followed by `other`. A chain runs its first-order sections before
    /// its biquads, so `other` may bring first-order sections only while this chain
    /// has no biquads; otherwise the sections would run out of order.
    pub fn then(mut self, other: Chain) -> Self {
        assert!(other.first.is_empty() || self.biquads.is_empty(), "Chain::then would reorder sections");
        self.first.extend(other.first);
        self.biquads.extend(other.biquads);
        self
    }

    /// 4th-order Butterworth highpass.
    pub fn highpass4(fs: f32, f: f32) -> Self {
        let mut c = Self::new();
        for q in BUTTER4_Q {
            c.biquads.push(Biquad::highpass(fs, f, q));
        }
        c
    }

    /// 4th-order Butterworth lowpass.
    pub fn lowpass4(fs: f32, f: f32) -> Self {
        let mut c = Self::new();
        for q in BUTTER4_Q {
            c.biquads.push(Biquad::lowpass(fs, f, q));
        }
        c
    }

    /// A response that rises by `db_per_octave` (falls, if negative; -6 to +6) from
    /// `lo` to `hi` Hz. It levels off over about an octave below `lo`, but keeps rising
    /// (or falling) for one to two octaves above `hi`, where the last shelf ends. Its gain
    /// keeps the power of a signal with a flat spectrum from `lo` to `hi`.
    ///
    /// Built from one first-order shelf per octave, each stepping the gain by
    /// `db_per_octave`, so the slope ripples by a fraction of a dB around the
    /// target. At ±6 dB/octave the shelves merge into a single zero or pole; past
    /// that a shelf spans more than an octave and the slope no longer holds at the
    /// band edges, so steeper tilts are refused.
    pub fn tilt(fs: f32, db_per_octave: f32, lo: f32, hi: f32) -> Self {
        let mut c = Self::new();
        if db_per_octave == 0.0 {
            return c;
        }
        assert!(
            db_per_octave.is_finite() && db_per_octave.abs() <= 6.0,
            "tilt {db_per_octave} dB/octave: must be within -6..6"
        );
        assert!(0.0 < lo && lo < hi && hi < fs / 2.0, "tilt band {lo}..{hi} Hz");
        // Bilinear prewarp: the analog frequency that lands on `f` Hz after the transform.
        let warp = |f: f32| 2.0 * fs * (PI * f / fs).tan();
        let step = 10f32.powf(db_per_octave.abs() / 20.0);
        let octaves = (hi / lo).log2().ceil() as i32;
        let mut corners = Vec::new();
        // One extra shelf past each edge, so the slope holds all the way to `lo` and `hi`.
        for k in -1..=octaves {
            // Corners stay below Nyquist, where the prewarp diverges.
            let corner = |f: f32| warp(f.min(0.49 * fs));
            let (lower, upper) = (corner(lo * 2f32.powi(k)), corner(lo * 2f32.powi(k) * step));
            // H(s) = (1 + s/wz) / (1 + s/wp): rising when the zero sits below the pole.
            corners.push(if db_per_octave > 0.0 { (lower, upper) } else { (upper, lower) });
        }
        // Mean power gain over `lo..hi` on a fine linear grid; the prewarp makes the
        // analog response at warp(f) the digital response at f.
        const GRID: usize = 512;
        let power = (0..GRID)
            .map(|i| {
                let w = warp(lo + (hi - lo) * (i as f32 + 0.5) / GRID as f32);
                corners.iter().map(|&(wz, wp)| (1.0 + (w / wz).powi(2)) / (1.0 + (w / wp).powi(2))).product::<f32>()
            })
            .sum::<f32>()
            / GRID as f32;
        let g = power.sqrt().recip();
        for (i, &(wz, wp)) in corners.iter().enumerate() {
            let n = if i == 0 { g } else { 1.0 };
            c.first.push(FirstOrder::from_analog(fs, n, n / wz, 1.0, 1.0 / wp));
        }
        c
    }

    pub fn push_first_order(&mut self, f: FirstOrder) {
        self.first.push(f);
    }

    #[inline]
    pub fn process(&mut self, mut x: f32) -> f32 {
        for f in &mut self.first {
            x = f.process(x);
        }
        for b in &mut self.biquads {
            x = b.process(x);
        }
        x
    }

    pub fn process_buf(&mut self, buf: &mut [f32]) {
        for x in buf {
            *x = self.process(*x);
        }
    }

    /// Clears the state of every section, as if no sample had been processed.
    pub fn reset(&mut self) {
        for f in &mut self.first {
            f.z = 0.0;
        }
        for b in &mut self.biquads {
            (b.z1, b.z2) = (0.0, 0.0);
        }
    }
}

/// First-order IIR from an analog prototype H(s) = (n0 + n1 s) / (d0 + d1 s).
#[derive(Clone, Debug)]
pub struct FirstOrder {
    b0: f32,
    b1: f32,
    a1: f32,
    z: f32,
}

impl FirstOrder {
    pub fn from_analog(fs: f32, n0: f32, n1: f32, d0: f32, d1: f32) -> Self {
        let k = 2.0 * fs;
        let a0 = d0 + d1 * k;
        Self { b0: (n0 + n1 * k) / a0, b1: (n0 - n1 * k) / a0, a1: (d0 - d1 * k) / a0, z: 0.0 }
    }

    /// FM pre-emphasis with time constant `tau` (e.g. 750 us), unity gain at 1 kHz.
    pub fn preemphasis(fs: f32, tau: f32) -> Self {
        let t2 = 1.0 / (2.0 * PI * 8000.0);
        let w = 2.0 * PI * 1000.0;
        let g = ((1.0 + (w * tau).powi(2)) / (1.0 + (w * t2).powi(2))).sqrt();
        Self::from_analog(fs, 1.0 / g, tau / g, 1.0, t2)
    }

    /// FM de-emphasis with time constant `tau`, unity gain at 1 kHz.
    pub fn deemphasis(fs: f32, tau: f32) -> Self {
        let w = 2.0 * PI * 1000.0;
        let g = (1.0 + (w * tau).powi(2)).sqrt();
        Self::from_analog(fs, g, 0.0, 1.0, tau)
    }

    #[inline]
    pub fn process(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.z;
        self.z = self.b1 * x - self.a1 * y;
        if self.z.abs() < FLUSH {
            self.z = 0.0;
        }
        y
    }
}

/// Windowed-sinc lowpass (Blackman window), unity DC gain.
pub fn design_lowpass_fir(ntaps: usize, cutoff_hz: f32, fs: f32) -> Vec<f32> {
    let fc = cutoff_hz / fs;
    let m = (ntaps - 1) as f32;
    let mut h: Vec<f32> = (0..ntaps)
        .map(|n| {
            let x = n as f32 - m / 2.0;
            let sinc = if x == 0.0 { 2.0 * fc } else { (2.0 * PI * fc * x).sin() / (PI * x) };
            let w = 0.42 - 0.5 * (2.0 * PI * n as f32 / m).cos() + 0.08 * (4.0 * PI * n as f32 / m).cos();
            sinc * w
        })
        .collect();
    let sum: f32 = h.iter().sum();
    h.iter_mut().for_each(|v| *v /= sum);
    h
}

/// Filter a complex signal with real taps (output has the same length, delayed by taps/2).
pub fn fir_complex(taps: &[f32], input: &[Complex32]) -> Vec<Complex32> {
    let n = taps.len();
    let mut out = Vec::with_capacity(input.len());
    for i in 0..input.len() {
        let mut acc = Complex32::new(0.0, 0.0);
        let start = (i + 1).saturating_sub(n);
        for (j, x) in input[start..=i].iter().rev().enumerate() {
            acc += x * taps[j];
        }
        out.push(acc);
    }
    out
}

/// Resample by `ratio` (output rate / input rate) with cubic Hermite interpolation.
/// Used to model sound-card clock differences of a few tens of ppm.
pub fn resample_cubic(input: &[f32], ratio: f64) -> Vec<f32> {
    if input.len() < 4 {
        return input.to_vec();
    }
    let n_out = ((input.len() as f64 - 3.0) * ratio).floor() as usize;
    let at = |i: isize| input[i.clamp(0, input.len() as isize - 1) as usize];
    (0..n_out)
        .map(|j| {
            let t = j as f64 / ratio;
            let i = t.floor() as isize;
            let f = (t - i as f64) as f32;
            let (y0, y1, y2, y3) = (at(i - 1), at(i), at(i + 1), at(i + 2));
            let c1 = 0.5 * (y2 - y0);
            let c2 = y0 - 2.5 * y1 + 2.0 * y2 - 0.5 * y3;
            let c3 = 0.5 * (y3 - y0) + 1.5 * (y1 - y2);
            ((c3 * f + c2) * f + c1) * f + y1
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tone_gain(f: &mut impl FnMut(f32) -> f32, freq: f32, fs: f32) -> f32 {
        let n = 48_000;
        let mut peak = 0.0f32;
        for i in 0..n {
            let y = f((2.0 * PI * freq * i as f32 / fs).sin());
            if i > n / 2 {
                peak = peak.max(y.abs());
            }
        }
        peak
    }

    #[test]
    fn emphasis_pair_is_roughly_flat() {
        let fs = 48_000.0;
        for freq in [500.0, 1000.0, 2500.0] {
            let mut pre = FirstOrder::preemphasis(fs, 750e-6);
            let mut de = FirstOrder::deemphasis(fs, 750e-6);
            let g = tone_gain(&mut |x| de.process(pre.process(x)), freq, fs);
            assert!((g - 1.0).abs() < 0.15, "{freq} Hz gain {g}");
        }
    }

    #[test]
    fn silence_after_a_signal_never_leaves_subnormal_state() {
        let fs = 48_000.0;
        // The slowest-decaying filters in use: the simulator's 20 Hz edge and de-emphasis.
        let mut c = Chain::bandpass4(fs, 20.0, 7000.0);
        c.push_first_order(FirstOrder::deemphasis(fs, 750e-6));
        let mut seed = 1u32;
        for _ in 0..48_000 {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            c.process(seed as f32 / u32::MAX as f32 - 0.5);
        }
        let ok = |z: f32| z == 0.0 || z.is_normal();
        for i in 0..20 * 48_000 {
            assert!(c.process(0.0).classify() != std::num::FpCategory::Subnormal, "output, sample {i}");
            assert!(c.biquads.iter().all(|b| ok(b.z1) && ok(b.z2)) && c.first.iter().all(|f| ok(f.z)), "sample {i}");
        }
    }

    #[test]
    fn then_is_the_two_chains_in_series() {
        let fs = 48_000.0;
        let (mut a, mut b) = (Chain::bandpass4(fs, 400.0, 6000.0), Chain::highpass4(fs, 400.0));
        let mut ab = Chain::bandpass4(fs, 400.0, 6000.0).then(Chain::highpass4(fs, 400.0));
        for i in 0..4800 {
            let x = (i as f32 * 0.05).sin() + if i == 100 { 1.0 } else { 0.0 };
            assert_eq!(ab.process(x), b.process(a.process(x)), "sample {i}");
        }
    }

    #[test]
    #[should_panic(expected = "reorder")]
    fn then_refuses_to_reorder_sections() {
        let fs = 48_000.0;
        let mut de = Chain::new();
        de.push_first_order(FirstOrder::deemphasis(fs, 750e-6));
        let _ = Chain::highpass4(fs, 400.0).then(de);
    }

    #[test]
    fn tilt_has_its_slope_across_the_band_and_keeps_a_flat_signals_power() {
        let fs = 48_000.0;
        let db = |c: &Chain, freq: f32| {
            let mut c = c.clone();
            20.0 * tone_gain(&mut |x| c.process(x), freq, fs).log10()
        };
        for (lo, hi) in [(300.0, 3000.0), (300.0, 7000.0)] {
            for slope in [-6.0, -3.0, 3.0, 6.0] {
                let c = Chain::tilt(fs, slope, lo, hi);
                // Across the band, in eighth-octave steps, the gain follows the slope,
                // measured from 1 kHz: within 0.5 dB where the carriers of both modes
                // lie (422 Hz to 6 kHz), within 1 dB out to the band edges.
                let at_1k = db(&c, 1000.0);
                let steps = (8.0 * (hi / lo).log2()).floor() as i32;
                for freq in (0..=steps).map(|k| lo * 2f32.powf(k as f32 / 8.0)) {
                    let want = slope * (freq / 1000.0f32).log2();
                    let got = db(&c, freq) - at_1k;
                    let tol = if (422.0..=6000.0).contains(&freq) { 0.5 } else { 1.0 };
                    assert!(
                        (got - want).abs() < tol,
                        "{lo}..{hi} {slope} dB/oct at {freq} Hz: {got:.2} dB, want {want:.2}"
                    );
                }
                // Equal tones spread evenly over the band keep their total power.
                let n = 32;
                let power = (0..n)
                    .map(|i| 10f32.powf(db(&c, lo + (hi - lo) * (i as f32 + 0.5) / n as f32) / 10.0))
                    .sum::<f32>()
                    / n as f32;
                assert!(
                    power.log10().abs() * 10.0 < 0.3,
                    "{lo}..{hi} {slope} dB/oct: power {:.2} dB",
                    10.0 * power.log10()
                );
            }
        }
        assert!(Chain::tilt(fs, 0.0, 300.0, 3000.0).first.is_empty());
    }

    #[test]
    fn tilt_refuses_slopes_it_cannot_hold() {
        for slope in [f32::NAN, f32::INFINITY, 6.5, -7.0] {
            let r = std::panic::catch_unwind(|| Chain::tilt(48_000.0, slope, 300.0, 3000.0));
            assert!(r.is_err(), "{slope} dB/octave was accepted");
        }
    }

    #[test]
    fn bandpass_passes_and_rejects() {
        let fs = 48_000.0;
        let mut bp = Chain::bandpass4(fs, 300.0, 3000.0);
        assert!(tone_gain(&mut |x| bp.process(x), 1000.0, fs) > 0.9);
        let mut bp = Chain::bandpass4(fs, 300.0, 3000.0);
        assert!(tone_gain(&mut |x| bp.process(x), 8000.0, fs) < 0.05);
    }
}
