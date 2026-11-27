//! Side-by-side evaluation of burst detectors through the FM channel simulator.
//!
//! Every detector sees exactly the same received audio, so differences are paired
//! and can be tested with McNemar's exact test instead of comparing two noisy rates.
//! For each CNR it reports, per detector:
//! - `burst`, the measure that counts: the whole burst (header and every codeword)
//!   decoded by the streaming `Receiver`, fed the recording in 10 ms chunks as a sound
//!   card delivers it. Where a search starts can change what Schmidl-Cox finds, so an
//!   offline search over the whole recording would not show what the receiver does;
//! - `hit`: the detector's first detection in the whole recording, searched from its
//!   start, lies within ±ncp/4 samples of the true position: the middle of the timings
//!   from which noisy bursts decode at the link's knee, which no detector defines (see
//!   `link_position`). A detector that always picks another point inside the cyclic
//!   prefix can decode every burst and still score few hits, so read `hit` as a
//!   diagnostic only. Where a detector's noiseless detection is that far off, the run
//!   says so; it prints the true position and each noiseless detection in any case.
//!
//! `genie` decodes from the true position: a known-timing reference, not a strict
//! bound. Another valid position inside the cyclic prefix puts the FFT windows on
//! other noise samples and can decode a recording that fails at the true position,
//! so a detector may occasionally beat it. The gap between `genie` and a detector
//! approximates what detection costs at that CNR.
//!
//! A separate pass counts detections and decoded headers per minute on audio that
//! holds no burst (see [`NOISE_CASES`]: a carrier at -10 dB CNR, which sounds like an
//! open squelch, a weak unmodulated carrier, white noise, keyed tones, and the modem's
//! own data symbols without their preambles).
//! A detection there costs a header decode; a decoded header there is a false burst.
//! The streaming receiver's CPU load on that audio, fed in 10 ms chunks like a sound
//! card, is reported per detector too.
//!
//! A CTCSS pass counts decoded bursts with a sub-tone on the carrier: every EIA tone at
//! 375, 750 and 1500 Hz deviation (the tables `busy_eval` uses), at each CTCSS CNR, on each
//! receive path that can hear one ([`ctcss_cases`]), after a row without a tone. The
//! simulated radio adds the tone at its modulator, after the deviation limiter. Each
//! cell has a genie column too: where it decodes and a detector does not, detection
//! lost the burst; where neither does, the tone broke demodulation. A tone once made
//! Schmidl-Cox miss every burst for many tones (#12). At level 1 and 20 dB CNR nothing
//! is lost now. At the knee, detection keeps its no-tone share of the genie's bursts.
//! The tone still breaks demodulation: at the quieter, denser levels 5 and 6, and at
//! the knee on narrow de-emphasised audio at 1500 Hz deviation.
//!
//! `cargo test --release -p vardefm-modem detector_eval -- --ignored --nocapture`
//!
//! Environment: `MODES` (narrow,wide), `TRIALS` per CNR (100), `LEVEL` (1), `NCW` (1),
//! `CNRS` (dB list, default per mode), `NOISE_SECONDS` per noise case (60),
//! `NOISE_SEED` (1, up to 2^32 - 1, for other noise in that pass), `CSV` (file to
//! append the burst results to). Three more change the bursts' path, not the noise
//! cases: `CHANNEL`, the radio audio response (matched, tx-pre-only or rx-de-only,
//! see [`CHANNELS`]), `TX_TILT`, dB/octave the sender tilts its audio by beyond
//! what the receiver expects (0), and `CTCSS`, the deviation in Hz of a CTCSS tone
//! the sending radio adds at its modulator, after the deviation limiter (0 = none;
//! wide mode only, as the narrow receive audio path filters sub-tones out).
//!
//! The separate CTCSS pass takes `CTCSS_TRIALS` per tone and deviation (10; 0 skips
//! the pass; trial `t` has the same noise in every row, so the rows are paired and
//! each rests on that many noise draws) and `CTCSS_CNRS` (dB list; 5,20 narrow and
//! 7,20 wide: the knee and well above it). It always sends over the matched channel
//! without extra tilt, whatever `CHANNEL`, `TX_TILT` and `CTCSS` say, so a run with
//! another `CHANNEL` or `TX_TILT` skips it and says so, rather than repeating the
//! matched figures under that link's heading.

use super::*;
use crate::busy_eval::CTCSS_DEVIATION_HZ;
use std::collections::{BTreeMap, HashMap};
use std::f32::consts::PI;
use std::io::Write;
use std::sync::{Arc, Mutex, OnceLock};
use std::time::Instant;
use vardefm_dsp::prng::Prng;
use vardefm_dsp::CTCSS_HZ;
use vardefm_sim::{transmit, Ctcss, FmChannelConfig};

/// The detector every other one is compared with in the paired test. Every detector
/// in [`Detector::ALL`] is evaluated.
const REFERENCE: Detector = Detector::SchmidlCox;

/// Leader length used by the evaluation, seconds (the `vardefm modem` default).
const TXDELAY: f32 = 0.15;

pub(crate) fn modem(mode: &str) -> Arc<Modem> {
    eval_modem(mode, 0.0, Detector::default())
}

/// A modem whose TX tilt is `tilt` dB/octave more than the mode's and whose receiver
/// uses `detector`.
fn eval_modem(mode: &str, tilt: f32, detector: Detector) -> Arc<Modem> {
    let mut p = OfdmParams::by_name(mode).unwrap();
    p.tilt_db_per_octave += tilt;
    let mut m = Modem::new(p, TXDELAY).with_detector(detector);
    // Trials already run on every core.
    m.decode_threads = 1;
    Arc::new(m)
}

/// Radio audio responses for `CHANNEL`: the simulator's own, where the receiving
/// radio's de-emphasis undoes the sending radio's pre-emphasis, then a sender with
/// pre-emphasis heard by a receiver without de-emphasis (about +6 dB/octave across
/// the band), and the reverse (about -6 dB/octave). The two mismatches use the
/// narrow channel's 750 us in either mode.
const CHANNELS: [&str; 3] = ["matched", "tx-pre-only", "rx-de-only"];

/// The FM channel of `mode` with response `channel` from [`CHANNELS`].
fn channel(mode: &str, channel: &str) -> FmChannelConfig {
    let base = FmChannelConfig::by_name(mode).unwrap();
    let us = FmChannelConfig::narrow().tx_emphasis_us;
    match channel {
        "matched" => base,
        "tx-pre-only" => FmChannelConfig { tx_emphasis_us: us, rx_emphasis_us: None, ..base },
        "rx-de-only" => FmChannelConfig { tx_emphasis_us: None, rx_emphasis_us: us, ..base },
        _ => panic!("CHANNEL: unknown channel {channel:?} (one of {})", CHANNELS.join(", ")),
    }
}

/// What a trial sends through and receives with: the receiving modem, whose detectors
/// are evaluated, the sending modem, which may tilt its audio differently, the
/// channel between them, without noise, and the deviation of a CTCSS tone the sending
/// radio adds at its modulator, after the deviation limiter (0 for none).
struct Link {
    rx: Arc<Modem>,
    /// The receiving modem once per detector of [`Detector::ALL`], in that order.
    receivers: Vec<Arc<Modem>>,
    tx: Arc<Modem>,
    chan: FmChannelConfig,
    ctcss_deviation_hz: f32,
}

impl Link {
    fn new(mode: &str, chan: &str, tx_tilt: f32) -> Self {
        let tx = eval_modem(mode, tx_tilt, Detector::default());
        let receivers = Detector::ALL.iter().map(|&d| eval_modem(mode, 0.0, d)).collect();
        Self { rx: modem(mode), receivers, tx, chan: channel(mode, chan), ctcss_deviation_hz: 0.0 }
    }

    fn matched(mode: &str) -> Self {
        Self::new(mode, "matched", 0.0)
    }

    /// The link with a CTCSS tone of `deviation_hz`. Only where the receive audio
    /// path passes sub-tones, as busy_eval models it: wide mode.
    fn with_ctcss(self, deviation_hz: f32) -> Self {
        let passes = self.chan.rx_band.0 < CTCSS_HZ[0];
        assert!(deviation_hz == 0.0 || passes, "CTCSS: the {} receive audio path filters it out", self.rx.ofdm.p.name);
        Self { ctcss_deviation_hz: deviation_hz, ..self }
    }

    /// The channel of trial `t` at `cnr` dB: its own noise, and the trial's CTCSS
    /// tone if the link has one, the standard tones in turn, each with its own phase.
    /// A tone always meets the payload and noise of its own trials, and unless the
    /// trials are a multiple of the tones, the first tones get one trial more: the
    /// sweep shows what a tone costs over all tones, the CTCSS pass what each one costs.
    fn trial_channel(&self, cnr: f32, t: u32) -> FmChannelConfig {
        let ctcss = (self.ctcss_deviation_hz > 0.0).then(|| Ctcss {
            hz: CTCSS_HZ[t as usize % CTCSS_HZ.len()],
            deviation_hz: self.ctcss_deviation_hz,
            phase: (2.0 * std::f64::consts::PI * (t as f64 * 0.618).fract()) as f32,
        });
        FmChannelConfig { cnr_db: Some(cnr), seed: channel_seed(cnr, t), ctcss, ..self.chan.clone() }
    }
}

pub(crate) fn burst(level: u8, ncw: usize, seed: u32) -> Burst {
    let mut rng = Prng::new(seed);
    let n = Modem::payload_bytes(level);
    Burst {
        header: Header { kind: Kind::Data, session: 0x42, level, ncw: ncw as u8, ..Default::default() },
        payload: (0..ncw).map(|_| (0..n).map(|_| rng.next_u32() as u8).collect()).collect(),
    }
}

/// The header of the burst whose preamble B starts at `b`, if it decodes.
pub(crate) fn header_at(m: &Modem, rx: &[f32], b: usize) -> Option<Header> {
    m.ofdm.demodulate(rx, b, m.header_symbols()).and_then(|f| m.decode_header(&f)?.known())
}

/// Whether the burst at `b`, whose header `h` has decoded, is the one sent: header
/// equal and every codeword decoded and equal, with the header as channel training
/// as in `Receiver::step`.
fn burst_ok(m: &Modem, rx: &[f32], b: usize, h: &Header, sent: &Burst) -> bool {
    if *h != sent.header {
        return false;
    }
    if h.ncw == 0 {
        return true;
    }
    let nsym = m.header_symbols() + h.ncw as usize * m.symbols_per_codeword(h.level);
    let Some(frame) = m.ofdm.demodulate_known(rx, b, nsym, &m.header_carriers(h)) else {
        return false;
    };
    let got = m.decode_payload(h, &frame);
    got.len() == sent.payload.len() && got.iter().zip(&sent.payload).all(|(got, want)| got.as_ref() == Some(want))
}

/// Whether the streaming `Receiver` on `m` makes `sent` of recording `rx`: its
/// first burst has the header sent and every codeword decoded as sent. The audio
/// comes as [`crate::tests::receive`] feeds it: in 10 ms chunks, then a second of
/// silence so that a burst at the very end completes.
fn received_ok(m: &Arc<Modem>, rx: &[f32], sent: &Burst) -> bool {
    let got = crate::tests::receive(&mut Receiver::new(m.clone()), rx);
    got.first().is_some_and(|g| {
        g.header == sent.header
            && g.payload.len() == sent.payload.len()
            && g.payload.iter().zip(&sent.payload).all(|(p, s)| p.as_ref() == Some(s))
    })
}

/// Outcome of one trial: (hit, burst) per detector of [`Detector::ALL`], then the
/// genie's burst.
struct Trial {
    det: Vec<(bool, bool)>,
    genie: bool,
}

/// Payload seed of trial `t`: every trial sends its own data, the same at every CNR.
pub(crate) fn payload_seed(t: u32) -> u32 {
    1 + t
}

/// Channel seed of trial `t` at `cnr` dB: every trial and every CNR point gets its
/// own noise, so the points of a curve do not share noise realisations. All
/// detectors still see the same recording within a trial, which keeps them paired.
fn channel_seed(cnr: f32, t: u32) -> u64 {
    (((cnr * 100.0).round() as i64 as u64) << 32) | t as u64
}

/// The audio the receiver gets when trial `t` sends `sent` over the `mode` channel
/// at `cnr` dB.
pub(crate) fn through_channel(m: &Modem, mode: &str, cnr: f32, sent: &Burst, t: u32) -> Vec<f32> {
    let cfg =
        FmChannelConfig { cnr_db: Some(cnr), seed: channel_seed(cnr, t), ..FmChannelConfig::by_name(mode).unwrap() };
    transmit(&cfg, &m.modulate(sent))
}

/// How a CTCSS tone on the carrier reaches the modem's receive audio.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum CtcssTap {
    /// Through the channel: the radio's tone encoder adds it at the modulator, and the
    /// receive chain (de-emphasis, audio band) treats it as it treats the burst.
    Channel,
    /// Added flat at its deviation to the received audio while the carrier is up and
    /// the squelch open, as at a discriminator tap that bypasses de-emphasis and the
    /// audio filter. The burst still takes the channel's receive chain.
    Discriminator,
}

/// A receive path for the CTCSS pass.
pub(crate) struct CtcssCase {
    pub name: &'static str,
    pub tap: CtcssTap,
    /// Low edge of the receive audio band in place of the channel's, Hz.
    pub rx_low_hz: Option<f32>,
}

/// The receive paths that can hear a CTCSS tone, per mode. Wide mode's data port is
/// flat down to 20 Hz. Narrow mode's mic/speaker channel filters the sub-tone out at
/// 300 Hz, so it is heard only on a radio whose audio output passes it: either
/// processed audio, de-emphasised (which lifts the tone by 10-13 dB relative to its
/// deviation), or a discriminator tap.
pub(crate) fn ctcss_cases(mode: &str) -> &'static [CtcssCase] {
    const WIDE: &[CtcssCase] = &[CtcssCase { name: "data port", tap: CtcssTap::Channel, rx_low_hz: None }];
    const NARROW: &[CtcssCase] = &[
        CtcssCase { name: "de-emphasised audio", tap: CtcssTap::Channel, rx_low_hz: Some(20.0) },
        CtcssCase { name: "discriminator", tap: CtcssTap::Discriminator, rx_low_hz: None },
    ];
    if mode == "narrow" {
        NARROW
    } else {
        WIDE
    }
}

/// Channel `cfg` with the receive band of case `c`.
pub(crate) fn ctcss_channel(cfg: &FmChannelConfig, c: &CtcssCase) -> FmChannelConfig {
    FmChannelConfig { rx_band: (c.rx_low_hz.unwrap_or(cfg.rx_band.0), cfg.rx_band.1), ..cfg.clone() }
}

/// What the receiver gets when `sent` goes over channel `cfg` (already set up for the
/// case, see [`ctcss_channel`]) with a CTCSS tone of `f` Hz at `dev` Hz deviation,
/// starting at phase `ph`, reaching the receiver through `tap`.
pub(crate) fn with_ctcss(
    m: &Modem,
    cfg: &FmChannelConfig,
    tap: CtcssTap,
    sent: &Burst,
    f: f32,
    dev: f32,
    ph: f32,
) -> Vec<f32> {
    let tx = m.modulate(sent);
    let toneless = (tap == CtcssTap::Discriminator).then(|| transmit(cfg, &tx));
    with_ctcss_on(cfg, tap, &tx, toneless.as_deref(), f, dev, ph)
}

/// [`with_ctcss`] for the modulated burst `tx`. A discriminator tap adds the tone to
/// `toneless`, what the channel gives without one, so a caller trying several tones on
/// one trial runs the channel once; the channel tap ignores it.
fn with_ctcss_on(
    cfg: &FmChannelConfig,
    tap: CtcssTap,
    tx: &[f32],
    toneless: Option<&[f32]>,
    f: f32,
    dev: f32,
    ph: f32,
) -> Vec<f32> {
    match tap {
        CtcssTap::Channel => {
            let ctcss = (dev > 0.0).then_some(Ctcss { hz: f, deviation_hz: dev, phase: ph });
            transmit(&FmChannelConfig { ctcss, ..cfg.clone() }, tx)
        }
        CtcssTap::Discriminator => {
            let mut rx = toneless.expect("a discriminator tap needs the toneless channel output").to_vec();
            assert_eq!(cfg.clock_offset_ppm, 0.0, "the tone span assumes no clock offset");
            // The tone starts at phase `ph` at carrier up, as `transmit` starts it, and
            // is heard while the carrier is up and the squelch open.
            let lead = (cfg.lead_silence_s * cfg.fs) as usize;
            let open = lead + (cfg.keyup_delay_s * cfg.fs) as usize..(lead + tx.len()).min(rx.len());
            let a = (dev / cfg.deviation_hz) as f64;
            let w = 2.0 * std::f64::consts::PI * f as f64 / cfg.fs as f64;
            for i in open {
                rx[i] += (a * (w * (i - lead) as f64 + ph as f64).sin()) as f32;
            }
            rx
        }
    }
}

fn trial(link: &Link, cnr: f32, level: u8, ncw: usize, truth: usize, t: u32) -> Trial {
    let m = &*link.rx;
    let sent = burst(level, ncw, payload_seed(t));
    let rx = transmit(&link.trial_channel(cnr, t), &link.tx.modulate(&sent));
    let tol = m.ofdm.p.ncp / 4;
    let det = Detector::ALL
        .iter()
        .zip(&link.receivers)
        .map(|(&d, r)| {
            let first = m.ofdm.detect_with(d, &rx, 0).at;
            (first.is_some_and(|b| b.abs_diff(truth) <= tol), received_ok(r, &rx, &sent))
        })
        .collect();
    let genie = header_at(m, &rx, truth).is_some_and(|h| burst_ok(m, &rx, truth, &h, &sent));
    Trial { det, genie }
}

/// Within this many dB of the best header SNR, a timing counts as among the best in
/// [`best_timing`]. The noiseless SNR has a broad top that rises slowly towards its
/// late edge and then falls off fast, so a narrow margin picks the late end of the
/// top, a few samples before the cliff: in narrow mode 0.5 dB put the true position
/// 16-32 samples before where noisy bursts stop decoding. 3 dB spans the top. Its
/// middle is still not the middle of the timings that decode in noise on every link (41
/// samples later in narrow mode with de-emphasis alone, 28 earlier in wide mode with a
/// TX tilt of +6 dB/octave), so it is only where [`decode_centre`] starts.
const BEST_TIMING_MARGIN_DB: f32 = 3.0;

/// Noiseless copies, each with its own payload, that [`best_timing`] averages over.
/// Where the link clips, the error depends on the payload; the average smooths it.
const BEST_TIMING_COPIES: u32 = 4;

/// Squared error and energy of the header symbols `known` demodulated from `rx` at
/// `b`, against the values sent. Unlike `Frame::snr_db` it needs no decisions, so it
/// holds on a link that distorts the burst.
fn header_error(m: &Modem, rx: &[f32], b: usize, known: &[Vec<Complex32>]) -> Option<(f32, f32)> {
    let f = m.ofdm.demodulate(rx, b, known.len())?;
    let pairs = || f.symbols.iter().flatten().zip(known.iter().flatten());
    Some((pairs().map(|(y, x)| (y - x).norm_sqr()).sum(), pairs().map(|(_, x)| x.norm_sqr()).sum()))
}

/// The timing at which `m` demodulates the header `h` of the bursts in `rxs` cleanest,
/// over all of them, searched within a cyclic prefix either side of `nominal`: the
/// middle of the run of timings around the best that are all within
/// [`BEST_TIMING_MARGIN_DB`] of it. Only that run counts, not a timing elsewhere that
/// happens to come as close (on a clipping link one can, at the edge of the search).
/// It tells where the burst is, to within the cyclic prefix, not where noisy bursts
/// decode best: [`decode_centre`] finds that, starting from here.
///
/// `Err` if none demodulates, or if the run reaches the first or last timing searched:
/// `nominal` leaves out the channel's group delay, so a link that delays or tilts the
/// burst enough moves the run past the edge, and the middle of what is left of it would
/// be early or late without a sign of it (in wide mode, de-emphasis alone with a TX tilt
/// of +6 dB/octave). A search two cyclic prefixes wide either side was tried and is no
/// way out: on a link that clips (wide mode, pre-emphasis alone) the noiseless header
/// came out cleanest more than half a cyclic prefix later than where Schmidl-Cox and
/// this search put the burst, and the genie decoded fewer noisy bursts from there than
/// the receiver did.
fn best_timing(m: &Modem, rxs: &[Vec<f32>], nominal: usize, h: &Header) -> Result<usize, String> {
    let ncp = m.ofdm.p.ncp;
    let known = m.header_carriers(h);
    let snr: Vec<(usize, f32)> = (nominal.saturating_sub(ncp)..=nominal + ncp)
        .filter_map(|b| {
            let (err, sig) = rxs
                .iter()
                .map(|rx| header_error(m, rx, b, &known))
                .collect::<Option<Vec<(f32, f32)>>>()?
                .into_iter()
                .fold((0.0, 0.0), |(e, s), (de, ds)| (e + de, s + ds));
            Some((b, 10.0 * (sig / f32::max(err, f32::MIN_POSITIVE)).log10()))
        })
        .collect();
    let Some(peak) = (0..snr.len()).max_by(|&a, &b| snr[a].1.total_cmp(&snr[b].1)) else {
        return Err("the noiseless burst does not demodulate near where it was sent".into());
    };
    let floor = snr[peak].1 - BEST_TIMING_MARGIN_DB;
    let near = |i: usize| snr[i].1 >= floor;
    let first = (0..peak).rev().take_while(|&i| near(i)).last().unwrap_or(peak);
    let last = (peak + 1..snr.len()).take_while(|&i| near(i)).last().unwrap_or(peak);
    if first == 0 || last == snr.len() - 1 {
        return Err(format!(
            "the best timings of the noiseless burst, {} to {}, reach the edge of the search, {} to {}",
            snr[first].0,
            snr[last].0,
            snr[0].0,
            snr[snr.len() - 1].0
        ));
    }
    Ok((snr[first].0 + snr[last].0) / 2)
}

/// Where preamble B of a `level` burst of `ncw` codewords lands after the channel,
/// and where each detector of [`Detector::ALL`] finds it on a noiseless copy. The
/// channel's delays depend neither on its noise nor on the payload (the burst layout
/// is fixed by level and size), so noiseless copies give it for every trial.
///
/// No detector defines the position, so neither `hit` nor `genie` favours one. It is
/// the middle of the timings from which the genie decodes noisy bursts at the link's
/// knee, without the CTCSS tone (the tone does not move the burst), found in three
/// steps:
/// - the timing at which noiseless copies demodulate their header cleanest
///   ([`best_timing`]), searched around where preamble B starts in the sent audio. It
///   finds the burst, but not the middle of what decodes: the noiseless header is
///   cleanest late among the timings that decode on some links and early on others;
/// - a CNR at which the genie decodes a quarter to three quarters of
///   [`POSITION_TRIALS`] noisy bursts from there ([`knee_trials`]). Well above the knee
///   every timing in the cyclic prefix decodes, and the counts would say nothing;
/// - on those trials, the middle of the run of timings that decode at least half as
///   many bursts as the best one ([`decode_centre`]).
///
/// The trials are seeded and are not the ones scored ([`POSITION_TRIAL`]), so the
/// position is the same in every run and is not fitted to the noise it is scored on.
/// With 384 trials in place of 32 it moves by at most 4 samples in narrow mode and 1 in
/// wide mode on the ten links the CI tests place.
///
/// It is the middle between the two timings where decoding falls to half, not the
/// timing that decodes most, and not the middle of the timings that decode nearly as
/// many as the best: where decoding falls off slowly on one side and fast on the other,
/// those differ (see `true_position_has_room_both_sides`).
///
/// `Err` when the link cannot be measured: its cleanest timings reach the edge of the
/// search ([`best_timing`]); the genie, decoding from the cleanest timing or from the
/// true position, does not decode every noiseless copy (header and every codeword); no
/// knee is found; or the timings that decode reach the edge of their search. Without
/// noise a link that loses a burst loses it whatever the detector, so its rows would
/// say nothing about detection. Strong TX tilts do that (from about ±18 dB/octave), and
/// in wide mode pre-emphasis without de-emphasis plus +6 dB/octave clips the burst.
fn link_position(link: &Link, level: u8, ncw: usize) -> Result<(usize, Vec<Option<usize>>), String> {
    let clean = FmChannelConfig { cnr_db: None, ..link.chan.clone() };
    let (truth, rx) = noiseless_position(&link.tx, &link.rx, &clean, level, ncw)?;
    let found = Detector::ALL.iter().map(|&d| link.rx.ofdm.detect_with(d, &rx, 0).at).collect();
    Ok((truth, found))
}

/// Whether the genie decodes `sent` from recording `rx` with preamble B at `b`: the
/// header, then every codeword.
fn genie_ok(m: &Modem, rx: &[f32], b: usize, sent: &Burst) -> bool {
    header_at(m, rx, b).is_some_and(|h| burst_ok(m, rx, b, &h, sent))
}

/// First trial number of the noisy trials that place the true position. The trials
/// that are scored count up from 0 and never reach it, so the position is not fitted to
/// the noise it is then scored on.
const POSITION_TRIAL: u32 = 1 << 30;

/// The CNRs, dB, between which [`knee_trials`] looks for the knee, and how often it
/// halves that range: down to an eighth of a dB.
const KNEE_RANGE_DB: (f32, f32) = (0.0, 32.0);
const KNEE_STEPS: u32 = 8;

/// Noisy trials at each CNR that [`knee_trials`] tries, which [`decode_centre`] then
/// counts decodes over at each timing.
const POSITION_TRIALS: u32 = 32;

/// The first trials of those, which tell whether a CNR is anywhere near the knee.
/// Six fail to tell one time in 32 at the middle of the knee; then the search goes on
/// to a CNR a little off the middle, which serves as well.
const KNEE_PROBE_TRIALS: u32 = 6;

/// How many cyclic prefixes either side of where it starts [`decode_centre`] searches
/// at most.
const CENTRE_REACH_NCP: isize = 2;

/// What a trial sent and what the receiver got.
type Recording = (Burst, Vec<f32>);

/// Trial `t` of the trials that place the true position: what it sends and what the
/// receiver gets over `clean` with noise at `cnr` dB.
fn position_trial(tx: &Modem, clean: &FmChannelConfig, cnr: f32, level: u8, ncw: usize, t: u32) -> Recording {
    let t = POSITION_TRIAL + t;
    let sent = burst(level, ncw, payload_seed(t));
    let cfg = FmChannelConfig { cnr_db: Some(cnr), seed: channel_seed(cnr, t), ..clean.clone() };
    let rx = transmit(&cfg, &tx.modulate(&sent));
    (sent, rx)
}

/// Noisy trials at the link's knee: [`POSITION_TRIALS`] of them at a CNR at which the
/// genie, decoding from `at`, decodes a quarter to three quarters, found by halving
/// [`KNEE_RANGE_DB`]. The knee is sharp: a quarter of a dB can take a link from 29 % to
/// 83 %. If no CNR tried lands there, the trials of the one that came closest to half.
/// `Err` if every CNR tried decodes all or none: the knee is outside the range.
///
/// A CNR whose first [`KNEE_PROBE_TRIALS`] trials all decode, or all fail, is taken to
/// lie on that side of the knee without running the rest: most of the CNRs tried are far
/// from it.
fn knee_trials(
    tx: &Modem,
    rx: &Modem,
    clean: &FmChannelConfig,
    level: u8,
    ncw: usize,
    at: usize,
) -> Result<Vec<Recording>, String> {
    const N: u32 = POSITION_TRIALS;
    let (mut lo, mut hi) = KNEE_RANGE_DB;
    let mut closest: Option<(u32, Vec<Recording>)> = None;
    for _ in 0..KNEE_STEPS {
        let cnr = (lo + hi) / 2.0;
        // Trials `from..to`: whether the genie decodes each, and the trial.
        let run = |from: u32, to: u32| {
            on_every_core(to - from, |t| {
                let (sent, audio) = position_trial(tx, clean, cnr, level, ncw, from + t);
                (genie_ok(rx, &audio, at, &sent), (sent, audio))
            })
        };
        let decoded = |trials: &[(bool, Recording)]| trials.iter().filter(|(ok, _)| *ok).count() as u32;
        let mut trials = run(0, KNEE_PROBE_TRIALS);
        let probe = decoded(&trials);
        if probe > 0 && probe < KNEE_PROBE_TRIALS {
            trials.extend(run(KNEE_PROBE_TRIALS, N));
            let n = decoded(&trials);
            let trials = trials.into_iter().map(|(_, trial)| trial).collect();
            if 4 * n >= N && 4 * n <= 3 * N {
                return Ok(trials);
            }
            let off = (2 * n).abs_diff(N);
            if n > 0 && n < N && closest.as_ref().is_none_or(|(c, _)| off < *c) {
                closest = Some((off, trials));
            }
            if 2 * n < N {
                lo = cnr;
            } else {
                hi = cnr;
            }
        } else if probe == 0 {
            lo = cnr;
        } else {
            hi = cnr;
        }
    }
    closest.map(|(_, trials)| trials).ok_or_else(|| {
        format!("the genie decodes all or none of the noisy bursts at every CNR tried, {lo} dB the last: no knee found")
    })
}

/// The middle of the timings from which the genie decodes the noisy recordings
/// `trials` (what was sent, what was received): of the run of timings, around the one
/// that decodes most, that all decode at least half as many. Each end of the run is
/// where the count crosses that half, between two timings an eighth of a cyclic prefix
/// apart.
///
/// The search goes from `around` outwards in steps of a quarter of a cyclic prefix,
/// then tries the timing half a step outside each end of the run. It starts a cyclic
/// prefix wide either side and grows by half a prefix on a side where the run reaches
/// its edge. `Err` if the run still reaches the edge [`CENTRE_REACH_NCP`] cyclic
/// prefixes out, or if no timing decodes a quarter of the trials: then the CNR is not
/// at the knee, and the run would say little.
fn decode_centre(m: &Modem, trials: &[Recording], around: usize) -> Result<usize, String> {
    // Timings are counted in eighths of a cyclic prefix from `around`; the search
    // steps by two of them.
    const PER_NCP: isize = 8;
    const STRIDE: usize = 2;
    let step = m.ofdm.p.ncp as isize / PER_NCP;
    let reach = CENTRE_REACH_NCP * PER_NCP;
    let timing = |k: isize| around as isize + k * step;
    let mut counts: BTreeMap<isize, usize> = BTreeMap::new();
    let count = |counts: &mut BTreeMap<isize, usize>, ks: &[isize]| {
        let per_trial = on_every_core(trials.len() as u32, |t| {
            let (sent, rx) = &trials[t as usize];
            ks.iter().map(|&k| timing(k) >= 0 && genie_ok(m, rx, timing(k) as usize, sent)).collect::<Vec<bool>>()
        });
        for (i, &k) in ks.iter().enumerate() {
            counts.insert(k, per_trial.iter().filter(|r| r[i]).count());
        }
    };
    let (mut from, mut to) = (-PER_NCP, PER_NCP);
    count(&mut counts, &(from..=to).step_by(STRIDE).collect::<Vec<_>>());
    let (first, last, best) = loop {
        let (&peak, &best) = counts.iter().max_by_key(|(_, &c)| c).expect("at least one timing");
        if 4 * best < trials.len() {
            return Err(format!("the genie decodes at most {best} of {} noisy bursts near the knee", trials.len()));
        }
        let in_run = |k: &isize| 2 * counts[k] >= best;
        let first = (from..=peak).rev().step_by(STRIDE).skip(1).take_while(in_run).last().unwrap_or(peak);
        let last = (peak..=to).step_by(STRIDE).skip(1).take_while(in_run).last().unwrap_or(peak);
        if (first == from && from == -reach) || (last == to && to == reach) {
            return Err(format!(
                "the timings that decode noisy bursts, {} to {}, reach the edge of the search, {} to {}",
                timing(first),
                timing(last),
                timing(from),
                timing(to)
            ));
        }
        if first > from && last < to {
            break (first, last, best);
        }
        let mut more = Vec::new();
        if first == from {
            from = (from - PER_NCP / 2).max(-reach);
            more.extend((from..first).step_by(STRIDE));
        }
        if last == to {
            to = (to + PER_NCP / 2).min(reach);
            more.extend((last..=to).step_by(STRIDE).skip(1));
        }
        count(&mut counts, &more);
    };
    count(&mut counts, &[first - 1, last + 1]);
    // Where the count crosses half the best going outwards (`out` is -1 or 1) from the
    // end `end` of the run: between the last timing that decodes at least half and
    // its neighbour.
    let half = best as f32 / 2.0;
    let cross = |end: isize, out: isize| {
        let inside = if 2 * counts[&(end + out)] >= best { end + out } else { end };
        let (c_in, c_out) = (counts[&inside] as f32, counts[&(inside + out)] as f32);
        inside as f32 + out as f32 * (c_in - half) / (c_in - c_out)
    };
    let middle = (cross(first, -1) + cross(last, 1)) / 2.0;
    Ok((around as f32 + middle * step as f32).round() as usize)
}

/// Where preamble B of the burst `tx` sends over `clean` would start if the channel did
/// not delay it.
fn nominal_position(tx: &Modem, clean: &FmChannelConfig) -> usize {
    let p = &tx.ofdm.p;
    (clean.lead_silence_s * clean.fs) as usize + (tx.leader_symbols() + 1) * p.symbol_len() + p.ncp
}

/// The true position of a `level` burst of `ncw` codewords that `tx` sends over the
/// noiseless channel `clean` and `rx` receives, as [`link_position`] defines it, and
/// the first noiseless copy. `Err` when the link cannot be measured.
///
/// Placing it sends some 50 to 150 noisy bursts and decodes each of the last 32 at a
/// dozen timings, so each link is placed once per process: the result depends only on
/// the two modems' parameters, the sender's leader, the channel and the burst's layout,
/// which make the key.
fn noiseless_position(
    tx: &Modem,
    rx: &Modem,
    clean: &FmChannelConfig,
    level: u8,
    ncw: usize,
) -> Result<(usize, Vec<f32>), String> {
    type Placed = Arc<OnceLock<Result<(usize, Vec<f32>), String>>>;
    static PLACED: OnceLock<Mutex<HashMap<String, Placed>>> = OnceLock::new();
    assert_eq!(clean.cnr_db, None, "the true position is measured without noise");
    let key = format!("{:?} {:?} {:?} {clean:?} {level} {ncw}", tx.ofdm.p, tx.txdelay, rx.ofdm.p);
    // The map is held only to find the link's cell, so links are placed side by side
    // and a second caller of one link waits for the first.
    let cell = PLACED.get_or_init(Default::default).lock().unwrap().entry(key).or_default().clone();
    cell.get_or_init(|| place(tx, rx, clean, level, ncw)).clone()
}

/// [`noiseless_position`], measured.
fn place(tx: &Modem, rx: &Modem, clean: &FmChannelConfig, level: u8, ncw: usize) -> Result<(usize, Vec<f32>), String> {
    let sent: Vec<Burst> = (0..BEST_TIMING_COPIES).map(|t| burst(level, ncw, payload_seed(t))).collect();
    let mut rxs: Vec<Vec<f32>> = sent.iter().map(|b| transmit(clean, &tx.modulate(b))).collect();
    let noiseless = |at: usize, what: &str| {
        let decoded = sent.iter().zip(&rxs).filter(|(b, a)| genie_ok(rx, a, at, b)).count();
        if decoded < sent.len() {
            return Err(format!("the genie decodes {decoded} of {} noiseless bursts at {what}", sent.len()));
        }
        Ok(())
    };
    let cleanest = best_timing(rx, &rxs, nominal_position(tx, clean), &sent[0].header)?;
    // Before the noisy trials: a link that loses the burst without noise has no knee.
    noiseless(cleanest, "the cleanest timing")?;
    let trials = knee_trials(tx, rx, clean, level, ncw, cleanest)?;
    let truth = decode_centre(rx, &trials, cleanest)?;
    noiseless(truth, "the true position")?;
    Ok((truth, rxs.swap_remove(0)))
}

/// What [`link_position`]'s detections say about each detector: a line for each one
/// that finds the noiseless burst further than `tol` samples from `truth`, or not at
/// all.
fn far_detections(truth: usize, found: &[Option<usize>], tol: usize) -> Vec<String> {
    Detector::ALL
        .iter()
        .zip(found)
        .filter(|(_, at)| !at.is_some_and(|b| b.abs_diff(truth) <= tol))
        .map(|(d, at)| format!("{d} finds the noiseless burst at {at:?}, the true position is {truth}"))
        .collect()
}

/// [`link_position`] where every detector must find the noiseless burst inside the
/// part of the cyclic prefix that leaves room for the channel (within ncp/2 of the
/// true position), as in the CI tests.
fn strict_link_position(link: &Link, level: u8, ncw: usize) -> usize {
    let (truth, found) = link_position(link, level, ncw).unwrap_or_else(|e| panic!("{e}"));
    let far = far_detections(truth, &found, link.rx.ofdm.p.ncp / 2);
    assert!(far.is_empty(), "{}", far.join("; "));
    truth
}

/// Where `m`'s own detector (Schmidl-Cox by default) finds preamble B of a `level`
/// burst of `ncw` codewords on a noiseless copy from the `mode` channel. `level_eval`
/// decodes from it. Unlike [`link_position`], which this evaluation uses, it is a
/// detector's position, so it favours that detector.
pub(crate) fn true_position(m: &Modem, mode: &str, level: u8, ncw: usize) -> usize {
    let clean = FmChannelConfig { cnr_db: None, ..FmChannelConfig::by_name(mode).unwrap() };
    let tx = m.modulate(&burst(level, ncw, payload_seed(0)));
    m.ofdm.detect(&transmit(&clean, &tx), 0).at.expect("burst not found on a clean channel")
}

/// `f(t)` for every trial `t` in `0..trials`, spread over every core, in trial
/// order. Each trial seeds itself, so the result does not depend on the threads.
pub(crate) fn on_every_core<R: Send>(trials: u32, f: impl Fn(u32) -> R + Sync) -> Vec<R> {
    let threads = std::thread::available_parallelism().map_or(1, |n| n.get()) as u32;
    let f = &f;
    let mut out: Vec<(u32, R)> = std::thread::scope(|s| {
        let workers: Vec<_> = (0..threads)
            .map(|th| s.spawn(move || (th..trials).step_by(threads as usize).map(|t| (t, f(t))).collect::<Vec<_>>()))
            .collect();
        workers.into_iter().flat_map(|w| w.join().unwrap()).collect()
    });
    out.sort_by_key(|(t, _)| *t);
    out.into_iter().map(|(_, r)| r).collect()
}

/// Run `trials` trials of a `level` burst of `ncw` codewords on every core, with
/// preamble B at `truth`.
fn run_trials_at(link: &Link, truth: usize, cnr: f32, level: u8, ncw: usize, trials: u32) -> Vec<Trial> {
    on_every_core(trials, |t| trial(link, cnr, level, ncw, truth, t))
}

/// [`run_trials_at`] where every detector must find the noiseless burst.
fn run_trials(link: &Link, cnr: f32, level: u8, ncw: usize, trials: u32) -> Vec<Trial> {
    run_trials_at(link, strict_link_position(link, level, ncw), cnr, level, ncw, trials)
}

/// Two-sided exact McNemar p-value for `b` and `c` discordant pairs.
fn mcnemar(b: u32, c: u32) -> f64 {
    let n = b + c;
    if n == 0 {
        return 1.0;
    }
    // Binomial(n, 1/2) terms in log space: 0.5^n alone underflows from n = 1075.
    let mut ln_term = n as f64 * 0.5f64.ln();
    let mut tail = 0.0;
    for i in 0..=b.min(c) {
        tail += ln_term.exp();
        ln_term += ((n - i) as f64 / (i + 1) as f64).ln();
    }
    (2.0 * tail).min(1.0)
}

/// Environment variable `name` parsed as `T`, or `default` when unset. A value that
/// does not parse, or is not Unicode, stops the run: falling back would quietly run
/// something else.
pub(crate) fn env_or<T: std::str::FromStr>(name: &str, default: T) -> T {
    match env_var(name) {
        Some(v) => v.trim().parse().unwrap_or_else(|_| panic!("{name}={v:?} does not parse")),
        None => default,
    }
}

/// Environment variable `name`, or `None` when unset. A value that is not Unicode
/// stops the run rather than reading as unset.
fn env_var(name: &str) -> Option<String> {
    match std::env::var(name) {
        Ok(v) => Some(v),
        Err(std::env::VarError::NotPresent) => None,
        Err(std::env::VarError::NotUnicode(v)) => panic!("{name}={v:?} is not Unicode"),
    }
}

/// Environment variable `name` as a comma-separated list of `T`, or `default` when
/// unset. As with [`env_or`], an item that does not parse stops the run.
pub(crate) fn env_list<T: std::str::FromStr>(name: &str, default: &str) -> Vec<T>
where
    T::Err: std::fmt::Display,
{
    let v = env_var(name).unwrap_or_else(|| default.to_string());
    v.split(',').map(|s| s.trim().parse().unwrap_or_else(|e| panic!("{name}: {s:?}: {e}"))).collect()
}

/// `MODES` as a list, `default` when unset, each checked to be a mode up front so
/// that a typo does not fail after the first mode has run.
pub(crate) fn env_modes(default: &str) -> Vec<String> {
    let modes: Vec<String> = env_list("MODES", default);
    for mode in &modes {
        assert!(OfdmParams::by_name(mode).is_some(), "MODES: unknown mode {mode:?} (narrow or wide)");
    }
    modes
}

/// Environment variable `name` as a switch: unset, `0` or `false` is off, `1` or
/// `true` is on. Anything else stops the run, as with [`env_or`].
pub(crate) fn env_flag(name: &str) -> bool {
    match env_var(name).as_deref().map(str::trim) {
        None | Some("0" | "false") => false,
        Some("1" | "true") => true,
        Some(v) => panic!("{name}={v:?}: use 1 or 0"),
    }
}

/// The noise case the streaming receiver's CPU load is measured on: a weak carrier
/// keeps the detectors searching without a burst to find.
const LOAD_CASE: &str = "weak carrier, 5 dB CNR";
/// The kinds of audio without a burst in the false-detection pass, for [`noise_case`].
const NOISE_CASES: [&str; 5] =
    ["carrier at -10 dB CNR", "weak carrier, 5 dB CNR", "white noise", "keyed tones", "data without preambles"];

/// Tone sets for the "keyed tones" case, Hz. 750 + 1500 Hz repeat every nfft/2
/// samples in both modes, like preamble A; then a roger beep and DTMF pairs.
const KEYED_TONES: [&[u64]; 6] = [&[750, 1500], &[1000], &[697, 1209], &[770, 1336], &[852, 1477], &[941, 1633]];
/// On and off time of each keyed tone, seconds.
const KEYED_TONE_S: f32 = 0.3;

/// Audio without a burst, `seconds` long at `fs` Hz: case `case` of [`NOISE_CASES`].
/// For one `seed`, every case and mode gets its own noise, so the minutes of a run are
/// independent. Two seeds can give a case the same noise (see [`noise_seeds`]).
///
/// The keyed tones stop after each 0.3 s: a tone that never stops cannot end a
/// Schmidl-Cox plateau, so it would never be evaluated as a detection. Each burst of
/// tone picks its set and starting phase at random, so the audio does not repeat.
///
/// The data case is what the receiver scans after a header that fails, or after it
/// misses a preamble: level 1 bursts of the longest size, leader and data, with the
/// preambles cut out, through the channel at 30 dB CNR.
fn noise_case(mode: &str, case: usize, seconds: f32, fs: f32, seed: u32) -> Vec<f32> {
    let n = (seconds * fs) as usize;
    assert!(n > 0, "{seconds} s of noise is less than one sample");
    let mode_index = OfdmParams::all().iter().position(|p| p.name == mode).expect("unknown mode");
    let (seed, prng_seed) = noise_seeds(seed, case, mode_index);
    // No silence around the carrier, so the audio is exactly `seconds` of noise.
    let base = FmChannelConfig {
        keyup_delay_s: 0.0,
        lead_silence_s: 0.0,
        trail_silence_s: 0.0,
        seed,
        ..FmChannelConfig::by_name(mode).unwrap()
    };
    let mut rng = Prng::new(prng_seed);
    match case {
        0 => transmit(&FmChannelConfig { cnr_db: Some(-10.0), ..base }, &vec![0.0; n]),
        1 => transmit(&FmChannelConfig { cnr_db: Some(5.0), ..base }, &vec![0.0; n]),
        2 => (0..n).map(|_| 0.3 * rng.uniform()).collect(),
        3 => {
            let rate = fs as u64;
            let slot = (KEYED_TONE_S * fs) as usize;
            let tones: Vec<(usize, u64)> = (0..n / slot + 1)
                .map(|_| (rng.next_u32() as usize % KEYED_TONES.len(), rng.next_u32() as u64 % rate))
                .collect();
            (0..n)
                .map(|i| {
                    let (k, on) = (i / slot, (i % slot) as u64);
                    if k % 2 == 1 {
                        return 0.0;
                    }
                    let (set, start) = tones[k];
                    // Phase from an integer index, exact however long the audio is.
                    KEYED_TONES[set]
                        .iter()
                        .map(|&f| 0.15 * (2.0 * PI * (((on + start) * f) % rate) as f32 / rate as f32).sin())
                        .sum()
                })
                .collect()
        }
        4 => {
            let m = modem(mode);
            let l = m.ofdm.p.symbol_len();
            let leader = m.ofdm.leader_symbols(TXDELAY) * l;
            let ncw = m.max_ncw(1);
            let mut audio = Vec::with_capacity(n + m.burst_samples(1, ncw));
            while audio.len() < n {
                let tx = m.modulate(&burst(1, ncw, rng.next_u32()));
                audio.extend(&tx[..leader]);
                audio.extend(&tx[leader + 2 * l..]);
            }
            audio.truncate(n);
            transmit(&FmChannelConfig { cnr_db: Some(30.0), ..base }, &audio)
        }
        _ => unreachable!("noise case {case}"),
    }
}

/// The channel seed and the generator seed of noise case `case` in mode `mode_index`
/// for `NOISE_SEED` `seed`. Within a run, which has one seed, every case and mode gets
/// its own of each. The channel seed is distinct across seeds too. The generator seed
/// is not and cannot be: it takes 32 bits, so the channel seed's high half is folded
/// into it, and seeds 1 and 1717986919 give the same one for case 2 in narrow mode.
fn noise_seeds(seed: u32, case: usize, mode_index: usize) -> (u64, u32) {
    assert!(case < 10 && mode_index < 10, "noise case {case}, mode {mode_index}: seeds would collide");
    let channel = u64::from(seed) * 100 + case as u64 * 10 + mode_index as u64;
    (channel, (channel ^ (channel >> 32)) as u32)
}

/// The CTCSS pass for one receive path: decoded bursts per detector and for the genie
/// (decoding from the true position), out of `trials`, for every tone at every
/// deviation, after a row without a tone. Where the genie decodes and a detector does
/// not, detection lost the burst; where neither does, the tone broke demodulation.
/// `Err` when the true position cannot be measured on that path, as in the sweep.
fn ctcss_pass(
    m: &Modem,
    mode: &str,
    case: &CtcssCase,
    cnr: f32,
    level: u8,
    ncw: usize,
    trials: u32,
) -> Result<(), String> {
    let base = ctcss_channel(&FmChannelConfig::by_name(mode).unwrap(), case);
    let clean = FmChannelConfig { cnr_db: None, ..base.clone() };
    // The true position as in `link_position`, which no detector defines.
    let (truth, _) = noiseless_position(m, m, &clean, level, ncw)?;
    // A receiver per detector, as in the main sweep.
    let receivers: Vec<Arc<Modem>> = Detector::ALL.iter().map(|&d| eval_modem(mode, 0.0, d)).collect();
    let names: Vec<&str> = Detector::ALL.iter().map(|d| d.name()).chain(["genie"]).collect();
    println!(
        "{mode}, {}: decoded bursts out of {trials} with a CTCSS tone, CNR {cnr} dB, at {CTCSS_DEVIATION_HZ:?} Hz \
         deviation",
        case.name
    );
    // The cells: no tone, then every tone at every deviation.
    let cells: Vec<(f32, f32)> = std::iter::once((0.0, 0.0))
        .chain(CTCSS_HZ.iter().flat_map(|&f| CTCSS_DEVIATION_HZ.iter().map(move |&dev| (f, dev))))
        .collect();
    // Per trial, per cell, per detector then the genie. A trial modulates its burst
    // once, and on a discriminator tap runs the channel once for all its cells.
    let res = on_every_core(trials, |t| {
        let sent = burst(level, ncw, payload_seed(t));
        let cfg = FmChannelConfig { cnr_db: Some(cnr), seed: channel_seed(cnr, t), ..base.clone() };
        let tx = m.modulate(&sent);
        let toneless = transmit(&cfg, &tx);
        cells
            .iter()
            .map(|&(f, dev)| {
                let rx = if dev == 0.0 {
                    toneless.clone()
                } else {
                    with_ctcss_on(&cfg, case.tap, &tx, Some(&toneless), f, dev, 0.7 * t as f32)
                };
                let genie = header_at(m, &rx, truth).is_some_and(|h| burst_ok(m, &rx, truth, &h, &sent));
                receivers.iter().map(|r| received_ok(r, &rx, &sent)).chain([genie]).collect::<Vec<bool>>()
            })
            .collect::<Vec<_>>()
    });
    // Per detector then the genie, per deviation, for the cells `first..` of one row.
    let row = |first: usize, devs: usize| -> Vec<Vec<u32>> {
        (0..names.len())
            .map(|d| (first..first + devs).map(|c| res.iter().filter(|r| r[c][d]).count() as u32).collect())
            .collect()
    };
    let print = |label: String, counts: &[Vec<u32>]| {
        let mut line = format!("  {label:9}");
        for (name, c) in names.iter().zip(counts) {
            line += &format!(" | {name}");
            c.iter().for_each(|ok| line += &format!(" {ok:4}"));
        }
        println!("{line}");
    };
    print("no tone".into(), &row(0, 1));
    let mut total = vec![vec![0u32; CTCSS_DEVIATION_HZ.len()]; names.len()];
    for (i, f) in CTCSS_HZ.into_iter().enumerate() {
        let counts = row(1 + i * CTCSS_DEVIATION_HZ.len(), CTCSS_DEVIATION_HZ.len());
        for (t, c) in total.iter_mut().zip(&counts) {
            t.iter_mut().zip(c).for_each(|(t, c)| *t += c);
        }
        print(format!("{f:6.1} Hz"), &counts);
    }
    print("all".into(), &total);
    Ok(())
}

/// Detections and decoded headers in `rx`, scanning on after each detection.
///
/// It searches on from each detection's `resume`, as `Receiver::step` does after a
/// header that fails, so each detector is scored on the searches it makes in the
/// receiver. After a known header the receiver skips the frame it announces (after
/// one of an unknown kind it searches on from the header's end); this pass searches
/// on from the detection's `resume` instead, so that every detection in the audio is
/// counted. A decoded header counts whatever its kind, so this does not go through
/// [`header_at`], which gives known kinds only: one of an unknown kind holds the
/// channel busy.
fn false_detections(m: &Modem, d: Detector, rx: &[f32]) -> (u32, u32) {
    let (mut dets, mut headers) = (0, 0);
    let mut found = m.ofdm.detect_with(d, rx, 0);
    // It ends: `detect_with` checks that a detection resumes past where it searched from.
    while let Some(b) = found.at {
        dets += 1;
        if m.ofdm.demodulate(rx, b, m.header_symbols()).and_then(|f| m.decode_header(&f)).is_some() {
            headers += 1;
        }
        found = m.ofdm.detect_with(d, rx, found.resume);
    }
    (dets, headers)
}

/// Processing time over audio duration of a `Receiver` using `detector` on `audio`,
/// fed in 10 ms chunks. The best of three runs, so that a cold cache or a busy core
/// does not set the figure.
fn receiver_load(mode: &str, detector: Detector, audio: &[f32]) -> f64 {
    let m = eval_modem(mode, 0.0, detector);
    let fs = m.fs() as f64;
    let chunk = (fs / 100.0) as usize;
    (0..3)
        .map(|_| {
            let mut rx = Receiver::new(m.clone());
            let t = Instant::now();
            for c in audio.chunks(chunk) {
                rx.push(c);
            }
            t.elapsed().as_secs_f64() / (audio.len() as f64 / fs)
        })
        .fold(f64::INFINITY, f64::min)
}

#[test]
#[ignore]
fn detector_eval() {
    let modes = env_modes("narrow,wide");
    let trials: u32 = env_or("TRIALS", 100);
    let level: u8 = env_or("LEVEL", 1);
    let ncw: usize = env_or("NCW", 1);
    let noise_s: f32 = env_or("NOISE_SECONDS", 60.0);
    let noise_seed: u32 = env_or("NOISE_SEED", 1);
    let ctcss: f32 = env_or("CTCSS", 0.0);
    let chan: String = env_or("CHANNEL", "matched".to_string());
    let chan = chan.trim();
    let tx_tilt: f32 = env_or("TX_TILT", 0.0);
    assert!(CHANNELS.contains(&chan), "CHANNEL: unknown channel {chan:?} (one of {})", CHANNELS.join(", "));
    assert!(tx_tilt.is_finite(), "TX_TILT={tx_tilt}: must be a finite dB/octave");
    assert!(ctcss.is_finite() && ctcss >= 0.0, "CTCSS={ctcss}: must be a deviation in Hz, 0 for none");
    let ctcss_trials: u32 = env_or("CTCSS_TRIALS", 10);
    assert!((level as usize) < LEVELS.len(), "LEVEL {level}: levels are 0 to {}", LEVELS.len() - 1);
    assert!(trials > 0, "TRIALS must be at least 1");
    assert!(noise_s > 0.0 && noise_s.is_finite(), "NOISE_SECONDS={noise_s}: must be a finite number more than 0");
    let modes: Vec<&str> = modes.iter().map(String::as_str).collect();
    // Checked up front, so that a typo or a bad NCW does not fail after the first mode
    // has run.
    for mode in &modes {
        let link = Link::new(mode, chan, tx_tilt).with_ctcss(ctcss);
        // A longer burst would modulate, but its header would not unpack (MAX_NCW) or
        // the modem would never send it (MAX_PAYLOAD_SECONDS): every trial would fail.
        let max = link.rx.max_ncw(level);
        assert!(ncw <= max, "NCW {ncw}: a level {level} burst in {mode} mode holds at most {max} codewords");
    }
    let cnr_list = |mode: &str| -> Vec<f32> {
        let default = match mode {
            "narrow" => "3,4,5,6,7,8,10",
            "wide" => "5,6,7,8,9,10,12",
            _ => panic!("no default CNRS for mode {mode:?}: set CNRS"),
        };
        let cnrs: Vec<f32> = env_list("CNRS", default);
        for c in &cnrs {
            // NaN or infinite noise gives an all-NaN recording, which would be written as data.
            assert!(c.is_finite(), "CNRS: {c} is not a finite number of dB");
        }
        cnrs
    };
    // At the sensitivity knee, where a tone that cost a dB or two would show, and well
    // above it.
    let ctcss_cnr_list = |mode: &str| -> Vec<f32> {
        let default = if mode == "narrow" { "5,20" } else { "7,20" };
        let cnrs: Vec<f32> = env_list("CTCSS_CNRS", default);
        for c in &cnrs {
            assert!(c.is_finite(), "CTCSS_CNRS: {c} is not a finite number of dB");
        }
        cnrs
    };
    for mode in &modes {
        cnr_list(mode);
        ctcss_cnr_list(mode);
    }
    const CSV_HEADER: &str = "mode,channel,tx_tilt,ctcss_dev_hz,level,ncw,cnr_db,trials,detector,hits,bursts";
    // Any path the OS accepts, Unicode or not.
    let mut csv = std::env::var_os("CSV").map(|p| {
        let mut f = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(&p)
            .unwrap_or_else(|e| panic!("CSV: cannot open {p:?}: {e}"));
        // A pipe or a terminal is never read (that would wait for input) and gets no
        // header, rather than one per run. A regular file gets the header when empty
        // (new, or from mktemp, say), and must already have it otherwise: appending rows
        // to a file with other columns would mix them up unnoticed.
        let meta = f.metadata().unwrap_or_else(|e| panic!("CSV: cannot read {p:?}: {e}"));
        if meta.is_file() {
            if meta.len() == 0 {
                writeln!(f, "{CSV_HEADER}").unwrap();
            } else {
                let old = std::fs::read(&p).unwrap_or_else(|e| panic!("CSV: cannot read {p:?}: {e}"));
                let head = old.split(|&b| b == b'\n').next().unwrap_or_default();
                let head = String::from_utf8_lossy(head);
                assert_eq!(head.trim_end_matches('\r'), CSV_HEADER, "CSV: {p:?} has other columns; use a new file");
            }
        }
        f
    });
    let reference = Detector::ALL.iter().position(|&d| d == REFERENCE).expect("reference detector not in ALL");
    println!(
        "level {level} ({}), {ncw} codeword(s), {trials} trials per point, txdelay {TXDELAY} s, \
         channel {chan}, TX tilt {tx_tilt:+} dB/octave, CTCSS {ctcss} Hz, noise seed {noise_seed}",
        LEVELS[level as usize].name
    );
    // The CTCSS pass builds its own link: the matched channel, no extra tilt.
    let default_link = chan == "matched" && tx_tilt == 0.0;
    let mut skipped = Vec::new();
    for &mode in &modes {
        let link = Link::new(mode, chan, tx_tilt).with_ctcss(ctcss);
        let m = &*link.rx;
        println!("\n{mode}: hits and decoded bursts out of {trials}; paired test against '{REFERENCE}'");
        // A link that loses the noiseless burst skips its bursts, not the whole run.
        let truth = match link_position(&link, level, ncw) {
            Ok((truth, found)) => {
                let mut line = format!("  true position {truth}; on the noiseless burst");
                for (d, at) in Detector::ALL.iter().zip(&found) {
                    match at {
                        Some(at) => line += &format!(" | {d} {:+}", *at as i64 - truth as i64),
                        None => line += &format!(" | {d} finds none"),
                    }
                }
                println!("{line}");
                for w in far_detections(truth, &found, m.ofdm.p.ncp / 4) {
                    println!("  warning: {w}; its hits mean little");
                }
                Some(truth)
            }
            Err(e) => {
                println!("  skipped: {e}");
                // A row per point, so that a sweep over links shows the gap.
                if let Some(f) = &mut csv {
                    for cnr in cnr_list(mode) {
                        writeln!(f, "{mode},{chan},{tx_tilt},{ctcss},{level},{ncw},{cnr},{trials},skipped,,").unwrap();
                    }
                }
                skipped.push(format!("{mode} bursts"));
                None
            }
        };
        if let Some(truth) = truth {
            for cnr in cnr_list(mode) {
                let res = run_trials_at(&link, truth, cnr, level, ncw, trials);
                let genie = res.iter().filter(|r| r.genie).count();
                let mut line = format!("  CNR {cnr:5.1} dB  genie {genie:4}");
                for (d, name) in Detector::ALL.iter().enumerate() {
                    let hits = res.iter().filter(|r| r.det[d].0).count();
                    let bursts = res.iter().filter(|r| r.det[d].1).count();
                    line += &format!(" | {name} hit {hits:4} burst {bursts:4}");
                    if d != reference {
                        let better = res.iter().filter(|r| r.det[d].1 && !r.det[reference].1).count() as u32;
                        let worse = res.iter().filter(|r| !r.det[d].1 && r.det[reference].1).count() as u32;
                        line += &format!(" (+{better}/-{worse}, p {:.3})", mcnemar(better, worse));
                    }
                    if let Some(f) = &mut csv {
                        writeln!(
                            f,
                            "{mode},{chan},{tx_tilt},{ctcss},{level},{ncw},{cnr},{trials},{name},{hits},{bursts}"
                        )
                        .unwrap();
                    }
                }
                if let Some(f) = &mut csv {
                    writeln!(f, "{mode},{chan},{tx_tilt},{ctcss},{level},{ncw},{cnr},{trials},genie,,{genie}").unwrap();
                }
                println!("{line}");
            }
        }
        if ctcss_trials > 0 && !default_link {
            println!("{mode}: CTCSS pass skipped: it runs on the matched channel without TX tilt only");
        } else if ctcss_trials > 0 {
            for cnr in ctcss_cnr_list(mode) {
                for case in ctcss_cases(mode) {
                    // Skipped like a link of the sweep: the rest of the run goes on.
                    if let Err(e) = ctcss_pass(m, mode, case, cnr, level, ncw, ctcss_trials) {
                        println!("{mode}, {}, CTCSS at {cnr} dB CNR: skipped: {e}", case.name);
                        skipped.push(format!("{mode} CTCSS pass, {}, {cnr} dB", case.name));
                    }
                }
            }
        }
        println!("{mode}: per minute of audio without a burst, detections / decoded headers");
        let mut load_audio = None;
        for (c, case) in NOISE_CASES.iter().enumerate() {
            let audio = noise_case(mode, c, noise_s, m.fs(), noise_seed);
            let mut line = format!("  {case:26}");
            let per_min = 60.0 / (audio.len() as f32 / m.fs());
            for d in Detector::ALL {
                let (n, h) = false_detections(m, d, &audio);
                line += &format!(" | {d} {:7.2} / {:5.2}", n as f32 * per_min, h as f32 * per_min);
            }
            println!("{line}");
            if *case == LOAD_CASE {
                load_audio = Some(audio);
            }
        }
        let load_audio = load_audio.unwrap_or_else(|| panic!("LOAD_CASE {LOAD_CASE:?} is not a noise case"));
        let mut line = format!("{mode}: streaming receiver CPU on '{LOAD_CASE}', 10 ms chunks:");
        for d in Detector::ALL {
            line += &format!(" {d} {:.2} %", 100.0 * receiver_load(mode, d, &load_audio));
        }
        println!("{line}");
    }
    // The rest of the run is still worth having, but it must not pass unnoticed.
    assert!(skipped.is_empty(), "not measured: {skipped:?}: see 'skipped' above");
}

#[test]
fn noise_seeds_are_distinct_within_a_run() {
    // Within a run every case and mode has its own channel seed and its own generator
    // seed, also where the channel seed passes 2^32, and where the cases straddle a
    // multiple of it (644245094: case 4 in the second mode lands on 15 * 2^32).
    let modes = OfdmParams::all().len();
    for seed in [1, 7, 42_949_672, 644_245_094, 1 + (1 << 30), 1_717_986_919, u32::MAX] {
        let seeds: Vec<(u64, u32)> =
            (0..NOISE_CASES.len()).flat_map(|c| (0..modes).map(move |m| noise_seeds(seed, c, m))).collect();
        for (what, mut unique) in [
            ("channel", seeds.iter().map(|s| s.0).collect::<Vec<u64>>()),
            ("generator", seeds.iter().map(|s| u64::from(s.1)).collect()),
        ] {
            unique.sort();
            unique.dedup();
            assert_eq!(unique.len(), seeds.len(), "seed {seed}, {what} seeds: {seeds:?}");
        }
    }
}

/// `false_detections` counts what is there: a noiseless burst gives a detection and a
/// decoded header, also when its header has a kind this version does not know, which
/// [`header_at`] does not give.
#[test]
fn false_detections_counts_a_header_of_any_kind() {
    for mode in OfdmParams::all().map(|p| p.name) {
        let link = Link::matched(mode);
        let m = &*link.rx;
        let clean = FmChannelConfig { cnr_db: None, ..link.chan.clone() };
        let sent = burst(1, 1, payload_seed(0));
        for (what, known, tx) in
            [("known", true, m.modulate(&sent)), ("unknown", false, m.modulate_with_kind(&sent, 5))]
        {
            let rx = transmit(&clean, &tx);
            for d in Detector::ALL {
                let at = m.ofdm.detect_with(d, &rx, 0).at.unwrap_or_else(|| panic!("{mode}: {d} missed the burst"));
                assert_eq!(header_at(m, &rx, at).is_some(), known, "{mode}, {what} kind, {d}");
                let (dets, headers) = false_detections(m, d, &rx);
                assert!(dets >= 1 && headers >= 1, "{mode}, {what} kind: {d} counts {dets} / {headers}");
            }
        }
    }
}

/// A run of best timings cut off by the edge of the search is refused, not centred on
/// what is left of it: here the search is moved a cyclic prefix early or late, so the
/// cleanest timing lies on its last or first timing.
#[test]
fn best_timing_refuses_a_run_at_the_edge_of_the_search() {
    for mode in OfdmParams::all().map(|p| p.name) {
        let link = Link::matched(mode);
        let m = &*link.rx;
        let clean = FmChannelConfig { cnr_db: None, ..link.chan.clone() };
        let sent = burst(1, 1, payload_seed(0));
        let rxs = vec![transmit(&clean, &link.tx.modulate(&sent))];
        let cleanest = best_timing(m, &rxs, nominal_position(&link.tx, &clean), &sent.header).unwrap();
        let ncp = m.ofdm.p.ncp;
        // A search centred on the cleanest timing has the run inside it.
        let centred = best_timing(m, &rxs, cleanest, &sent.header);
        assert_eq!(centred, Ok(cleanest), "{mode}");
        for nominal in [cleanest - ncp, cleanest + ncp] {
            let err = best_timing(m, &rxs, nominal, &sent.header).expect_err("centred a run the search cuts off");
            assert!(err.contains("edge of the search"), "{mode}: {err}");
        }
    }
}

/// The middle of the timings that decode does not depend on where its search starts,
/// as long as the search sees the run: started a cyclic prefix early or late it grows
/// that way and ends up where it does from the true position. Started so far off that
/// the run reaches the limit of the search, it refuses. And a link has no knee where
/// nothing decodes: ten cyclic prefixes from the burst.
#[test]
fn decode_centre_finds_the_run_from_either_side_and_refuses_one_at_the_edge() {
    let link = Link::matched("wide");
    let m = &*link.rx;
    let clean = FmChannelConfig { cnr_db: None, ..link.chan.clone() };
    let (truth, _) = link_position(&link, 1, 1).unwrap();
    let ncp = m.ofdm.p.ncp;
    let trials = knee_trials(&link.tx, m, &clean, 1, 1, truth).unwrap();
    for around in [truth - ncp, truth, truth + ncp] {
        let centre = decode_centre(m, &trials, around);
        assert!(
            centre.as_ref().is_ok_and(|c| c.abs_diff(truth) <= ncp / 16),
            "from {around}: {centre:?}, true position {truth}"
        );
    }
    for around in [truth - 3 * ncp / 2, truth + 3 * ncp / 2] {
        let err = decode_centre(m, &trials, around).expect_err("centred a run the search cuts off");
        assert!(err.contains("edge of the search"), "from {around}: {err}");
    }
    let err = knee_trials(&link.tx, m, &clean, 1, 1, truth + 10 * ncp).expect_err("found a knee where nothing decodes");
    assert!(err.contains("no knee found"), "{err}");
}

#[test]
fn on_every_core_keeps_trial_order() {
    assert_eq!(on_every_core(37, |t| t), (0..37).collect::<Vec<_>>());
    assert!(on_every_core(0, |t| t).is_empty());
}

#[test]
fn harness_sees_strong_bursts_and_no_false_headers() {
    for mode in OfdmParams::all().map(|p| p.name) {
        let link = Link::matched(mode);
        let m = &*link.rx;
        // The channel's delays must not depend on the payload, or noiseless copies
        // could not give the position for every trial.
        let (truth, found) = link_position(&link, 1, 1).unwrap();
        let far = far_detections(truth, &found, m.ofdm.p.ncp / 2);
        assert!(far.is_empty(), "{mode}: {}", far.join("; "));
        let reference = Detector::ALL.iter().position(|&d| d == REFERENCE).unwrap();
        let clean = FmChannelConfig { cnr_db: None, ..link.chan.clone() };
        for t in 1..4 {
            let tx = m.modulate(&burst(1, 1, payload_seed(t)));
            let at = m.ofdm.detect_with(REFERENCE, &transmit(&clean, &tx), 0).at;
            assert_eq!(at, found[reference], "{mode}: payload {t} moves the burst");
        }
        for r in run_trials_at(&link, truth, 25.0, 1, 1, 4) {
            assert!(r.genie, "{mode}: genie failed on a strong signal");
            assert!(r.det.iter().all(|&(_, ok)| ok), "{mode}: a detector missed a strong burst");
        }
        let audio = noise_case(mode, 0, 2.0, m.fs(), 1);
        for d in Detector::ALL {
            assert_eq!(false_detections(m, d, &audio).1, 0, "{mode}: {d} decoded a header from noise");
        }
    }
}

/// A link the genie cannot decode without noise is refused, not measured: here a TX
/// tilt of -18 dB/octave in wide mode, and +24 in narrow mode, for whichever reason
/// (`mismatched_links_still_carry_strong_bursts` shows the links that are measured).
#[test]
fn link_position_refuses_a_link_the_genie_cannot_decode() {
    for (mode, tilt) in [("wide", -18.0), ("narrow", 24.0)] {
        let r = link_position(&Link::new(mode, "matched", tilt), 1, 1);
        assert!(r.is_err(), "{mode} tilt {tilt}: measured a link nothing decodes: {r:?}");
    }
}

/// Whether the true position of the link has room on both sides at `cnr` dB, which
/// must be near its knee, where the timing matters: over 400 noisy trials the genie
/// decodes about as many bursts there, and a quarter cyclic prefix either side, as at
/// the best of the nine timings an eighth of a prefix apart within half a prefix.
/// `Err` says which timing falls short, with the counts.
fn room_both_sides(mode: &str, chan: &str, tilt: f32, cnr: f32) -> Result<(), String> {
    const TRIALS: u32 = 400;
    // 6 %: about two standard deviations of a count, plus what taking the best of nine
    // noisy counts adds to the best.
    const SLACK: usize = 24;
    let link = Link::new(mode, chan, tilt);
    let m = &*link.rx;
    let ncp = m.ofdm.p.ncp as isize;
    let (truth, _) = link_position(&link, 1, 1)?;
    let offsets: Vec<isize> = (-4..=4).map(|k| k * ncp / 8).collect();
    let res = on_every_core(TRIALS, |t| {
        let sent = burst(1, 1, payload_seed(t));
        let rx = transmit(&link.trial_channel(cnr, t), &link.tx.modulate(&sent));
        offsets.iter().map(|&o| genie_ok(m, &rx, (truth as isize + o) as usize, &sent)).collect::<Vec<bool>>()
    });
    let count = |i: usize| res.iter().filter(|r| r[i]).count();
    let counts: Vec<usize> = (0..offsets.len()).map(count).collect();
    let best = *counts.iter().max().unwrap();
    let what = format!("{mode} {chan} tilt {tilt} at {cnr} dB");
    println!("{what}: true position {truth}, offsets {offsets:?}, decoded {counts:?}");
    // The genie must decode here at all, or the comparison proves nothing.
    if best < TRIALS as usize / 4 {
        return Err(format!("{what}: the best timing decodes {best}; pick another CNR"));
    }
    for o in [-ncp / 4, 0, ncp / 4] {
        let got = counts[offsets.iter().position(|&x| x == o).unwrap()];
        if got + SLACK < best {
            return Err(format!(
                "{what}: {got} decoded {o:+} samples from the true position, {best} at best; offsets {offsets:?}, \
                 counts {counts:?}"
            ));
        }
    }
    Ok(())
}

/// The true position has room on both sides ([`room_both_sides`]) on three links, each
/// at a CNR where the genie decodes about half the bursts. A true position near either
/// edge of the timings that decode fails. The cleanest noiseless timing alone, which
/// was the true position before, did on narrow rx-de-only: at 5 dB a quarter prefix
/// later decoded 253 of 400 against 293 at best.
///
/// It claims no more than that. On narrow rx-de-only decoding falls off slowly towards
/// early timings and fast towards late ones, and further above the knee that shows: at
/// 5 dB, where the best timing decodes 294 of 400, a quarter prefix early decodes 269,
/// one short of the slack, and a quarter prefix late 294. So there the true position
/// lies about an eighth of a cyclic prefix before the middle of the timings that decode nearly as many as
/// the best, and `hit` is a little harder on a detector that lands early than its
/// bursts warrant. Telling those timings apart takes hundreds of trials per timing,
/// which placing every link cannot afford.
#[test]
fn true_position_has_room_both_sides() {
    for (mode, chan, cnr) in [("narrow", "matched", 4.0), ("wide", "rx-de-only", 6.0), ("narrow", "rx-de-only", 4.875)]
    {
        room_both_sides(mode, chan, 0.0, cnr).unwrap_or_else(|e| panic!("{e}"));
    }
}

/// [`true_position_has_room_both_sides`] on every link in [`ROOM_LINKS`]: too slow for
/// every CI run. It names every link that falls short, not only the first.
///
/// `cargo test --release -p vardefm-modem true_position_has_room_both_sides_on_every_link -- --ignored --nocapture`
#[test]
#[ignore]
fn true_position_has_room_both_sides_on_every_link() {
    let short: Vec<String> =
        ROOM_LINKS.iter().filter_map(|&(mode, chan, tilt, cnr)| room_both_sides(mode, chan, tilt, cnr).err()).collect();
    assert!(short.is_empty(), "{}", short.join("\n"));
}

/// Every link the CI tests place (mode, channel, TX tilt), with a CNR near its knee:
/// one at which the best timing decodes 100 to 320 of 400 bursts.
const ROOM_LINKS: [(&str, &str, f32, f32); 10] = [
    ("narrow", "matched", 0.0, 4.0),
    ("narrow", "tx-pre-only", 0.0, 4.0),
    ("narrow", "rx-de-only", 0.0, 4.875),
    ("narrow", "matched", -6.0, 4.75),
    ("narrow", "matched", 6.0, 4.75),
    ("wide", "matched", 0.0, 6.0),
    ("wide", "tx-pre-only", 0.0, 7.0),
    ("wide", "rx-de-only", 0.0, 6.0),
    ("wide", "matched", -6.0, 7.0),
    ("wide", "matched", 6.0, 8.5),
];

/// Every radio response, and a TX tilt of ±6 dB/octave on the matched response, still
/// carries a strong burst to the genie and to every detector. The genie check makes a
/// detector failure here a failure of detection, not of a channel nothing could
/// decode. Both at once can be too much: in wide mode, pre-emphasis and +6 dB/octave
/// clip the top carriers.
#[test]
fn mismatched_links_still_carry_strong_bursts() {
    let cases = CHANNELS.iter().map(|&c| (c, 0.0)).chain([("matched", -6.0), ("matched", 6.0)]);
    for (chan, tilt) in cases {
        for mode in OfdmParams::all().map(|p| p.name) {
            for r in run_trials(&Link::new(mode, chan, tilt), 25.0, 1, 1, 2) {
                assert!(r.genie, "{mode} {chan} tilt {tilt}: genie failed on a strong signal");
                assert!(r.det.iter().all(|&(_, ok)| ok), "{mode} {chan} tilt {tilt}: a detector missed a strong burst");
            }
        }
    }
}

/// The CTCSS path: the tone reaches the receiver at the deviation sent, and a
/// wide-mode link with a tone still carries strong bursts to the genie and to every
/// detector, for every standard tone at 1500 Hz deviation (twice the usual). Before
/// #15 high-passed its search, Schmidl-Cox lost bursts to a tone of more than about
/// 375 Hz deviation at any CNR.
#[test]
fn ctcss_tone_reaches_the_receiver_and_strong_bursts_get_through() {
    let deviation_hz = 1500.0;
    let link = Link::matched("wide").with_ctcss(deviation_hz);
    for t in 0..2 {
        let cfg = link.trial_channel(25.0, t);
        let tone = cfg.ctcss.clone().expect("the link adds a tone");
        let tx = link.tx.modulate(&burst(1, 1, payload_seed(t)));
        let rx = transmit(&cfg, &tx);
        // While the squelch is open: from key-up to the end of the burst.
        let (lead, keyup) = ((cfg.lead_silence_s * cfg.fs) as usize, (cfg.keyup_delay_s * cfg.fs) as usize);
        let heard = &rx[lead + keyup..lead + tx.len()];
        // The received amplitude at the tone's frequency.
        let w = 2.0 * std::f64::consts::PI * tone.hz as f64 / cfg.fs as f64;
        let (c, s) = heard.iter().enumerate().fold((0.0, 0.0), |(c, s), (i, &x)| {
            let ph = w * i as f64;
            (c + x as f64 * ph.cos(), s + x as f64 * ph.sin())
        });
        let amp = 2.0 * f64::hypot(c, s) / heard.len() as f64;
        let want = (deviation_hz / cfg.deviation_hz) as f64;
        assert!((amp / want - 1.0).abs() < 0.25, "{} Hz tone received at {amp:.3}, sent at {want:.3}", tone.hz);
    }
    // Trial `t` has tone `t`: every standard tone once.
    for (t, r) in run_trials(&link, 25.0, 1, 1, CTCSS_HZ.len() as u32).iter().enumerate() {
        let hz = CTCSS_HZ[t];
        assert!(r.genie, "genie failed on a strong signal with {hz} Hz CTCSS");
        assert!(r.det.iter().all(|&(_, ok)| ok), "a detector missed a strong burst with {hz} Hz CTCSS");
    }
}

#[test]
fn mcnemar_matches_known_values() {
    assert_eq!(mcnemar(0, 0), 1.0);
    assert!((mcnemar(0, 5) - 0.0625).abs() < 1e-12);
    assert!((mcnemar(2, 8) - 0.109375).abs() < 1e-12);
    assert_eq!(mcnemar(4, 4), 1.0);
    // Past the point where 0.5^n underflows.
    assert_eq!(mcnemar(1100, 1100), 1.0);
    // z = 4.26 with continuity correction; the exact value is about 2e-5.
    let p = mcnemar(1000, 1200);
    assert!(p > 1e-6 && p < 1e-4, "p {p}");
}
