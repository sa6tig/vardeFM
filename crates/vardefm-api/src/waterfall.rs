//! Waterfall rows: the power spectrum of the received audio, ten times a second.
//!
//! Each row transforms the first [`WINDOW`] samples of its [`HOP`] under a periodic Hann window;
//! the rest of the hop is not looked at. Levels are `u8` in [`DB_STEP`] steps from [`DB_FLOOR`],
//! and a full-scale sine on a bin centre reads 0 dBFS.
//!
//! The bin count is set per mode, because the wide mode reaches 6 kHz.

use rustfft::num_complex::Complex;
use rustfft::{Fft, FftPlanner};
use std::sync::Arc;

/// Samples at 48 kHz in one transform.
pub const WINDOW: usize = 4096;
/// Samples at 48 kHz from one row to the next: 10 rows per second.
pub const HOP: usize = 4800;
/// Rows per second.
pub const ROWS_PER_S: f32 = 48_000.0 / HOP as f32;
/// Spacing of the bins of a row, in Hz.
pub const BIN_HZ: f32 = 48_000.0 / WINDOW as f32;
/// Most bins a row can have: 0 Hz to half the sample rate.
pub const MAX_BINS: usize = WINDOW / 2 + 1;
/// Level of value 0 in a row, in dBFS; anything lower also reads 0.
pub const DB_FLOOR: f32 = -127.5;
/// Level per value step in a row, in dB. A value `v` is `DB_FLOOR + v * DB_STEP` dBFS.
pub const DB_STEP: f32 = 0.5;

/// Bins for the narrow mode: 0 to 3000 Hz, the usual count for this API. The carriers end at 2672 Hz.
pub const NARROW_BINS: u16 = 257;
/// Bins for the wide mode: 0 to 7500 Hz. The carriers end at 6000 Hz and the transmit low-pass
/// at 6900 Hz.
pub const WIDE_BINS: u16 = 641;

/// Bins per row for a vardeFM mode (`narrow` or `wide`).
pub fn bins_for_mode(mode: &str) -> Option<u16> {
    match mode {
        "narrow" => Some(NARROW_BINS),
        "wide" => Some(WIDE_BINS),
        _ => None,
    }
}

/// One row of the spectrum.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WaterfallRow {
    /// Count of rows made before this one; a jump shows rows lost on the way.
    pub seq: u64,
    /// First sample of the transform, on the receive clock.
    pub t: u64,
    /// Bin levels, the first at 0 Hz; see [`DB_STEP`].
    pub bins: Vec<u8>,
}

/// Turns 48 kHz audio into [`WaterfallRow`]s.
pub struct Waterfall {
    fft: Arc<dyn Fft<f32>>,
    /// Periodic Hann window, [`WINDOW`] points.
    window: Vec<f32>,
    buf: Vec<Complex<f32>>,
    scratch: Vec<Complex<f32>>,
    bins: usize,
    /// Samples of the current hop seen so far.
    at: usize,
    /// First sample of the current hop.
    start: u64,
    seq: u64,
}

impl Waterfall {
    /// A waterfall with `bins` bins per row.
    ///
    /// # Panics
    ///
    /// If `bins` is 0 or more than [`MAX_BINS`].
    pub fn new(bins: usize) -> Self {
        assert!((1..=MAX_BINS).contains(&bins), "a row has 1 to {MAX_BINS} bins, not {bins}");
        let fft = FftPlanner::new().plan_fft_forward(WINDOW);
        let step = 2.0 * std::f64::consts::PI / WINDOW as f64;
        let window = (0..WINDOW).map(|n| (0.5 - 0.5 * (step * n as f64).cos()) as f32).collect();
        let scratch = vec![Complex::default(); fft.get_inplace_scratch_len()];
        Self { fft, window, buf: vec![Complex::default(); WINDOW], scratch, bins, at: 0, start: 0, seq: 0 }
    }

    /// Bins per row.
    pub fn bins(&self) -> usize {
        self.bins
    }

    /// The audio has a discontinuity before the next sample: start a new hop there, so no row
    /// mixes audio from both sides of it.
    pub fn gap(&mut self) {
        self.at = 0;
    }

    /// Takes 48 kHz samples, the first at `t0` on the receive clock, and hands each finished
    /// row to `sink`.
    pub fn push(&mut self, t0: u64, data: &[f32], sink: &mut impl FnMut(WaterfallRow)) {
        let mut t = t0;
        let mut rest = data;
        while !rest.is_empty() {
            if self.at == 0 {
                self.start = t;
            }
            let room = HOP - self.at;
            let (now, later) = rest.split_at(room.min(rest.len()));
            self.fill(now);
            let filling = self.at < WINDOW;
            self.at += now.len();
            if filling && self.at >= WINDOW {
                self.row(sink);
            }
            if self.at >= HOP {
                self.at = 0;
            }
            t += now.len() as u64;
            rest = later;
        }
    }

    /// Stores the part of `samples`, the next of the hop, that falls in the window.
    fn fill(&mut self, samples: &[f32]) {
        let slots = self.buf.iter_mut().zip(&self.window).skip(self.at);
        for ((slot, w), &s) in slots.zip(samples) {
            *slot = Complex::new(s * w, 0.0);
        }
    }

    fn row(&mut self, sink: &mut impl FnMut(WaterfallRow)) {
        self.fft.process_with_scratch(&mut self.buf, &mut self.scratch);
        let bins = self.buf[..self.bins].iter().map(|x| level(x.norm_sqr())).collect();
        sink(WaterfallRow { seq: self.seq, t: self.start, bins });
        self.seq += 1;
    }
}

/// The value of a bin with raw power `power`.
fn level(power: f32) -> u8 {
    // A full-scale sine on a bin centre has magnitude WINDOW / 4 under a Hann window.
    let full_scale = (WINDOW / 4) as f32;
    let dbfs = 10.0 * (power / (full_scale * full_scale)).log10();
    // The cast saturates, and takes NaN, and the -inf of an empty bin, to 0.
    ((dbfs - DB_FLOOR) / DB_STEP + 0.5) as u8
}

#[cfg(test)]
mod tests {
    use super::*;
    use vardefm_dsp::OfdmParams;

    fn sine(hz: f32, amplitude: f32, n: usize) -> Vec<f32> {
        (0..n).map(|i| amplitude * (std::f32::consts::TAU * hz * i as f32 / 48_000.0).sin()).collect()
    }

    fn rows(w: &mut Waterfall, t0: u64, data: &[f32]) -> Vec<WaterfallRow> {
        let mut out = Vec::new();
        w.push(t0, data, &mut |r| out.push(r));
        out
    }

    fn peak(row: &WaterfallRow) -> (usize, u8) {
        let (k, v) = row.bins.iter().enumerate().max_by_key(|&(_, v)| *v).unwrap();
        (k, *v)
    }

    #[test]
    fn a_sine_lands_on_its_bin_at_its_level_in_dbfs() {
        let hz = 128.0 * BIN_HZ;
        let got = rows(&mut Waterfall::new(257), 0, &sine(hz, 1.0, HOP));
        assert_eq!(peak(&got[0]), (128, 255));

        let got = rows(&mut Waterfall::new(257), 0, &sine(hz, 0.1, HOP));
        let (k, v) = peak(&got[0]);
        assert_eq!(k, 128);
        // -20 dBFS.
        assert_eq!(DB_FLOOR + f32::from(v) * DB_STEP, -20.0);
    }

    #[test]
    fn narrow_bins_cover_0_to_3_khz() {
        let bins = NARROW_BINS as usize;
        assert_eq!((bins - 1) as f32 * BIN_HZ, 3000.0);
        let got = rows(&mut Waterfall::new(bins), 0, &sine(3000.0, 1.0, HOP));
        assert_eq!(got[0].bins.len(), bins);
        assert_eq!(peak(&got[0]).0, bins - 1);
    }

    #[test]
    fn wide_bins_cover_0_to_7500_hz() {
        let bins = WIDE_BINS as usize;
        assert_eq!((bins - 1) as f32 * BIN_HZ, 7500.0);
        let got = rows(&mut Waterfall::new(bins), 0, &sine(7500.0, 1.0, HOP));
        assert_eq!(got[0].bins.len(), bins);
        assert_eq!(peak(&got[0]).0, bins - 1);
    }

    #[test]
    fn each_mode_shows_every_carrier() {
        for mode in ["narrow", "wide"] {
            let p = OfdmParams::by_name(mode).unwrap();
            let top = p.carrier_freq(p.num_carriers - 1);
            let bins = bins_for_mode(mode).unwrap();
            assert!(f32::from(bins - 1) * BIN_HZ > top, "{mode}: top carrier at {top} Hz");
        }
        assert_eq!(bins_for_mode("vara"), None);
    }

    #[test]
    fn silence_and_bad_samples_read_the_floor_and_not_a_panic() {
        let got = rows(&mut Waterfall::new(257), 0, &vec![0.0; HOP]);
        assert!(got[0].bins.iter().all(|&v| v == 0));
        let got = rows(&mut Waterfall::new(257), 0, &vec![f32::NAN; HOP]);
        assert!(got[0].bins.iter().all(|&v| v == 0));
        let got = rows(&mut Waterfall::new(257), 0, &vec![f32::INFINITY; HOP]);
        assert_eq!(got.len(), 1);
    }

    #[test]
    fn rows_come_every_hop_stamped_with_their_first_sample() {
        let got = rows(&mut Waterfall::new(257), 1000, &vec![0.25; 3 * HOP]);
        let stamps: Vec<_> = got.iter().map(|r| (r.seq, r.t)).collect();
        assert_eq!(stamps, [(0, 1000), (1, 1000 + HOP as u64), (2, 1000 + 2 * HOP as u64)]);
        assert_eq!(ROWS_PER_S, 10.0);
    }

    #[test]
    fn block_size_does_not_change_the_rows() {
        let audio = sine(1000.0, 0.5, 2 * HOP + 100);
        let whole = rows(&mut Waterfall::new(641), 0, &audio);
        let mut w = Waterfall::new(641);
        let mut parts = Vec::new();
        let mut t = 0;
        for chunk in audio.chunks(777) {
            parts.extend(rows(&mut w, t, chunk));
            t += chunk.len() as u64;
        }
        assert_eq!(whole, parts);
    }

    #[test]
    fn a_gap_restarts_the_hop_so_no_row_spans_it() {
        let mut w = Waterfall::new(257);
        assert_eq!(rows(&mut w, 0, &vec![0.5; WINDOW - 1]).len(), 0);
        // One more sample would finish the row if the gap had not restarted it.
        let resumed = 20_000;
        w.gap();
        assert_eq!(rows(&mut w, resumed, &[0.5; 1]).len(), 0);
        let got = rows(&mut w, resumed + 1, &vec![0.5; WINDOW - 1]);
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].t, resumed);
    }

    #[test]
    #[should_panic(expected = "1 to 2049 bins")]
    fn refuses_more_bins_than_the_transform_has() {
        Waterfall::new(MAX_BINS + 1);
    }
}
