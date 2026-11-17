//! Streaming receiver: feed audio in chunks, get decoded bursts out.

use crate::busy::BusyDetector;
use crate::{Header, Modem, LEVELS};
use std::sync::Arc;
use vardefm_dsp::Complex32;

/// A received sample beyond this is a glitch: full scale is 1.
const MAX_SAMPLE: f32 = 4.0;

/// A received burst.
#[derive(Clone, Debug)]
pub struct RxBurst {
    pub header: Header,
    /// One entry per codeword: `Some(data)` if it decoded and passed its CRC.
    pub payload: Vec<Option<Vec<u8>>>,
    /// Average SNR per symbol (Es/N0) estimated from the pilots, dB.
    pub snr_db: f32,
    /// Absolute sample index (since the receiver was created) just past the burst's last
    /// symbol.
    pub end_sample: u64,
    /// Received audio level over the burst, dB relative to full scale.
    pub level: AudioLevel,
}

/// RMS and peak level of a stretch of audio, dB relative to full scale.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AudioLevel {
    pub rms_dbfs: f32,
    pub peak_dbfs: f32,
}

impl AudioLevel {
    /// Level of `samples` (silence reads -120 dBFS).
    pub fn of(samples: &[f32]) -> Self {
        let (mut sq, mut peak) = (0f64, 0f32);
        for &x in samples {
            sq += (x * x) as f64;
            peak = peak.max(x.abs());
        }
        let ms = sq / samples.len().max(1) as f64;
        Self { rms_dbfs: (10.0 * ms.max(1e-12).log10()) as f32, peak_dbfs: 20.0 * peak.max(1e-6).log10() }
    }
}

/// The payload symbols of a received burst, for a constellation view (see
/// [`Receiver::keep_symbols`]).
#[derive(Clone, Debug)]
pub struct RxSymbols {
    /// The [`RxBurst::end_sample`] of the burst they belong to.
    pub end_sample: u64,
    /// Equalized data-carrier symbols, one Vec per OFDM symbol of the payload, in the order of
    /// the modem's data carriers (`Ofdm::data_idx`).
    pub symbols: Vec<Vec<Complex32>>,
}

pub struct Receiver {
    modem: Arc<Modem>,
    buf: Vec<f32>,
    /// Absolute sample index of `buf[0]`.
    base: u64,
    /// Where to continue searching for preambles (index into `buf`).
    search: usize,
    /// A frame whose header decoded but whose payload has not fully arrived: (preamble-B index, header).
    pending: Option<(usize, Header)>,
    /// Whether something louder than the background is heard.
    detector: BusyDetector,
    /// The last good sample pushed.
    last: f32,
    /// Payload symbols kept since the last [`take_symbols`](Self::take_symbols); `None` when
    /// they are not kept.
    kept: Option<Vec<RxSymbols>>,
}

impl Receiver {
    pub fn new(modem: Arc<Modem>) -> Self {
        let detector = BusyDetector::new(&modem.ofdm.p);
        Self { modem, buf: Vec::new(), base: 0, search: 0, pending: None, detector, last: 0.0, kept: None }
    }

    /// Keep the payload symbols of every burst with a payload, for
    /// [`take_symbols`](Self::take_symbols). Off by default: they are copied only when asked for.
    pub fn keep_symbols(&mut self, on: bool) {
        self.kept = on.then(Vec::new);
    }

    /// The payload symbols kept since the last call, oldest first.
    pub fn take_symbols(&mut self) -> Vec<RxSymbols> {
        self.kept.as_mut().map(std::mem::take).unwrap_or_default()
    }

    /// Absolute index of the next sample to be pushed.
    pub fn position(&self) -> u64 {
        self.base + self.buf.len() as u64
    }

    /// True while a signal louder than the background is heard (a steady
    /// open-squelch hiss does not count) or a frame is being received.
    pub fn busy(&self) -> bool {
        self.pending.is_some() || self.detector.busy()
    }

    /// Drop all buffered audio (e.g. when our own transmitter keys up). The audio
    /// pushed next need not follow on from the last.
    pub fn reset(&mut self) {
        self.base += self.buf.len() as u64;
        self.buf.clear();
        self.search = 0;
        self.pending = None;
        self.detector.resume();
    }

    pub fn push(&mut self, samples: &[f32]) -> Vec<RxBurst> {
        let start = self.buf.len();
        self.buf.extend_from_slice(samples);
        // A NaN or Inf from the sound card, or a sample so far beyond full scale that a
        // filter overflows, would make every later output of an IIR filter NaN (the
        // busy detector's, the preamble search's) and spoil the FFT of the symbol it
        // falls in. The last good sample takes its place: in a steady tone that is a
        // much smaller step than 0.
        for x in &mut self.buf[start..] {
            if x.abs() <= MAX_SAMPLE {
                self.last = *x;
            } else {
                *x = self.last;
            }
        }
        self.detector.push(&self.buf[start..]);
        let mut out = Vec::new();
        while let Some(progress) = self.step(&mut out) {
            if !progress {
                break;
            }
        }
        self.trim();
        out
    }

    /// Absolute index just past the last of `nsym` symbols after preamble B at `b`. A
    /// [`Frame`](vardefm_dsp::ofdm::Frame)'s `end` is a symbol later: it covers the look-ahead
    /// the demodulator reads past the burst.
    fn burst_end(&self, b: usize, nsym: usize) -> u64 {
        let p = &self.modem.ofdm.p;
        self.base + (b + p.nfft + nsym * p.symbol_len()) as u64
    }

    /// Sample index (in `buf`) needed to demodulate `nsym` symbols after preamble B at `b`.
    fn needed(&self, b: usize, nsym: usize) -> usize {
        let p = &self.modem.ofdm.p;
        b - p.ncp / 2 + (nsym + 1) * p.symbol_len() + p.nfft
    }

    /// Level of the burst whose preamble B body starts at `b`, from the start of
    /// preamble A to the end of its last data symbol. `Frame::end` lies about a
    /// symbol later (the receiver's look-ahead), where a squelch tail could land.
    fn burst_level(&self, b: usize, nsym: usize) -> AudioLevel {
        let p = &self.modem.ofdm.p;
        let l = p.symbol_len();
        let start = b.saturating_sub(p.ncp + l);
        let end = (b + p.nfft + nsym * l).min(self.buf.len());
        AudioLevel::of(&self.buf[start.min(end)..end])
    }

    /// One processing step. Returns Some(true) if something changed and we should go again.
    fn step(&mut self, out: &mut Vec<RxBurst>) -> Option<bool> {
        let m = &self.modem;
        let hdr_syms = m.header_symbols();
        if let Some((b, h)) = self.pending {
            let nsym = hdr_syms + h.ncw as usize * m.symbols_per_codeword(h.level);
            if self.buf.len() < self.needed(b, nsym) {
                return Some(false);
            }
            self.pending = None;
            // The header decoded and passed its CRC, so its symbols are known: they
            // train the channel estimate for the payload.
            let known = m.header_carriers(&h);
            let Some(frame) = m.ofdm.demodulate_known(&self.buf, b, nsym, &known) else {
                self.search = b;
                return Some(true);
            };
            let payload = m.decode_payload(&h, &frame);
            let level = self.burst_level(b, nsym);
            let end_sample = self.burst_end(b, nsym);
            if let Some(kept) = &mut self.kept {
                kept.push(RxSymbols { end_sample, symbols: frame.symbols[hdr_syms..].to_vec() });
            }
            out.push(RxBurst { header: h, payload, snr_db: frame.snr_db, end_sample, level });
            self.search = frame.end.min(self.buf.len());
            return Some(true);
        }

        let l = m.ofdm.p.symbol_len();
        match m.ofdm.detect(&self.buf, self.search) {
            Some(b) => {
                if self.buf.len() < self.needed(b, hdr_syms) {
                    // Come back when the header has arrived; re-detect from just before it.
                    self.search = b.saturating_sub(3 * l);
                    return Some(false);
                }
                let header =
                    m.ofdm.demodulate(&self.buf, b, hdr_syms).and_then(|f| m.decode_header(&f).map(|h| (h, f)));
                match header {
                    None => self.search = b,
                    Some((h, f)) if h.ncw == 0 => {
                        // Measure the SNR the way a data burst does (with the header as
                        // training), so that rate adaptation sees the same numbers.
                        let known = m.header_carriers(&h);
                        let snr_db =
                            m.ofdm.demodulate_known(&self.buf, b, hdr_syms, &known).map_or(f.snr_db, |g| g.snr_db);
                        out.push(RxBurst {
                            header: h,
                            payload: Vec::new(),
                            snr_db,
                            end_sample: self.burst_end(b, hdr_syms),
                            level: self.burst_level(b, hdr_syms),
                        });
                        self.search = f.end.min(self.buf.len());
                    }
                    Some((h, _)) => {
                        debug_assert!((h.level as usize) < LEVELS.len());
                        self.pending = Some((b, h));
                    }
                }
                Some(true)
            }
            None => {
                // Nothing found. Keep enough history to catch a preamble that is still arriving.
                let keep = 3 * l + 2 * m.ofdm.p.nfft;
                self.search = self.search.max(self.buf.len().saturating_sub(keep));
                Some(false)
            }
        }
    }

    fn trim(&mut self) {
        let l = self.modem.ofdm.p.symbol_len();
        let keep_from = match self.pending {
            Some((b, _)) => b.saturating_sub(3 * l),
            None => self.search,
        };
        if keep_from > 20 * l {
            self.buf.drain(..keep_from);
            self.base += keep_from as u64;
            self.search -= keep_from;
            if let Some((b, _)) = &mut self.pending {
                *b -= keep_from;
            }
        }
    }
}
