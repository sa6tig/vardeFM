//! Side-by-side evaluation of burst detectors through the FM channel simulator.
//!
//! Every detector sees exactly the same received audio, so differences are paired
//! and can be tested with McNemar's exact test instead of comparing two noisy rates.
//! For each CNR it reports, per detector:
//! - `burst`, the measure that counts: the whole burst (header and every codeword)
//!   decoded by an offline equivalent of `Receiver::step`. It tries the header at each
//!   detection in turn, searches on after one that fails, and keeps the first header
//!   that decodes. A test checks it against the streaming `Receiver`;
//! - `hit`: the detector's first detection, whatever its header does, lies within
//!   ±ncp/4 samples of the true position, which is where Schmidl-Cox's preamble-B fine
//!   timing lands on a noiseless copy. A detector that always picks another point
//!   inside the cyclic prefix can decode every burst and still score few hits, so read
//!   `hit` as a diagnostic only.
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
//! `cargo test --release -p vardefm-modem detector_eval -- --ignored --nocapture`
//!
//! Environment: `MODES` (narrow,wide), `TRIALS` per CNR (100), `LEVEL` (1), `NCW` (1),
//! `CNRS` (dB list, default per mode), `NOISE_SECONDS` per noise case (60),
//! `NOISE_SEED` (1, for other noise in that pass), `CSV` (file to append the burst
//! results to). Three more change the bursts' path, not the noise cases: `CHANNEL`,
//! the radio audio response (matched, tx-pre-only or rx-de-only, see [`CHANNELS`]),
//! `TX_TILT`, dB/octave the sending radio tilts its audio by, -6 to 6 (0; the
//! simulator's `tx_tilt_db_per_octave`, at the audio's own RMS), and `CTCSS`, the
//! deviation in Hz of a CTCSS tone the sending radio adds after its audio response
//! (0 = none; wide mode only, as the narrow audio path filters sub-tones out).

use super::*;
use crate::busy_eval::CTCSS_HZ;
use std::f32::consts::PI;
use std::io::Write;
use std::sync::Arc;
use std::time::Instant;
use vardefm_dsp::ofdm::MF_THRESHOLD;
use vardefm_dsp::prng::Prng;
use vardefm_sim::{transmit, Ctcss, FmChannelConfig};

/// The detector every other one is compared with in the paired test. Every detector
/// in [`Detector::ALL`] is evaluated.
const REFERENCE: Detector = Detector::SchmidlCox;

/// Leader length used by the evaluation, seconds (the `vardefm modem` default).
const TXDELAY: f32 = 0.15;

pub(crate) fn modem(mode: &str) -> Arc<Modem> {
    eval_modem(mode, Detector::default())
}

/// The `mode` modem whose receiver uses `detector`.
fn eval_modem(mode: &str, detector: Detector) -> Arc<Modem> {
    let mut m = Modem::new(OfdmParams::by_name(mode).unwrap(), TXDELAY).with_detector(detector);
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

/// The FM channel of `mode` with response `channel` from [`CHANNELS`] and a sending
/// radio that tilts its audio by `tx_tilt` dB/octave.
fn channel(mode: &str, channel: &str, tx_tilt: f32) -> FmChannelConfig {
    let base = FmChannelConfig { tx_tilt_db_per_octave: tx_tilt, ..FmChannelConfig::by_name(mode).unwrap() };
    let us = FmChannelConfig::narrow().tx_emphasis_us;
    match channel {
        "matched" => base,
        "tx-pre-only" => FmChannelConfig { tx_emphasis_us: us, rx_emphasis_us: None, ..base },
        "rx-de-only" => FmChannelConfig { tx_emphasis_us: None, rx_emphasis_us: us, ..base },
        _ => panic!("CHANNEL: unknown channel {channel:?} (one of {})", CHANNELS.join(", ")),
    }
}

/// What a trial sends through and receives with: the modem at both ends, whose
/// detectors are evaluated, the channel between them, without noise, which holds every
/// mismatch of the two radios' audio responses, and the deviation of a CTCSS tone the
/// sending radio adds (0 for none).
struct Link {
    modem: Arc<Modem>,
    chan: FmChannelConfig,
    ctcss_hz: f32,
}

impl Link {
    fn new(mode: &str, chan: &str, tx_tilt: f32) -> Self {
        Self { modem: modem(mode), chan: channel(mode, chan, tx_tilt), ctcss_hz: 0.0 }
    }

    fn matched(mode: &str) -> Self {
        Self::new(mode, "matched", 0.0)
    }

    /// The link with a CTCSS tone of `deviation_hz`. Only where the TX audio path
    /// passes sub-tones, as busy_eval models it: wide mode.
    fn with_ctcss(self, deviation_hz: f32) -> Self {
        let passes = self.chan.tx_band.0 < CTCSS_HZ[0];
        assert!(deviation_hz == 0.0 || passes, "CTCSS: the {} TX audio path filters it out", self.modem.ofdm.p.name);
        Self { ctcss_hz: deviation_hz, ..self }
    }

    /// The CTCSS tone the sending radio adds in trial `t`: the standard tones in turn,
    /// each trial with its own phase.
    fn ctcss(&self, t: u32) -> Option<Ctcss> {
        (self.ctcss_hz != 0.0).then(|| Ctcss {
            hz: CTCSS_HZ[t as usize % CTCSS_HZ.len()] as f64,
            deviation_hz: self.ctcss_hz,
            phase: t as f64 * 0.618,
        })
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
    m.ofdm.demodulate(rx, b, m.header_symbols()).and_then(|f| m.decode_header(&f))
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
    m.decode_payload(h, &frame).into_iter().zip(&sent.payload).all(|(got, want)| got.as_ref() == Some(want))
}

/// What the receiver makes of one recording with a given detector.
struct Outcome {
    /// Where the detector first pointed, header decoded or not.
    first: Option<usize>,
    /// Whether the burst came out as sent.
    ok: bool,
}

/// Offline equivalent of `Receiver::step` with detector `d`: decode the header at
/// each detection in turn, searching on from a detection whose header fails, and
/// judge the burst of the first header that decodes. The test
/// `offline_receive_matches_the_streaming_receiver` checks it against `Receiver`.
fn receive(m: &Modem, d: Detector, rx: &[f32], sent: &Burst) -> Outcome {
    let first = m.ofdm.detect_with(d, rx, 0);
    let mut next = first;
    while let Some(b) = next {
        if let Some(h) = header_at(m, rx, b) {
            return Outcome { first, ok: burst_ok(m, rx, b, &h, sent) };
        }
        // As `Receiver::step`: search on from the failed detection itself. That
        // needs a detector that reports a position past the start of what it
        // matched; one that returns `b` again would hang the evaluation, so stop.
        next = m.ofdm.detect_with(d, rx, b);
        if let Some(n) = next {
            assert!(n > b, "detector returned {n} when searching from {b}: it would loop forever");
        }
    }
    Outcome { first, ok: false }
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

fn trial(link: &Link, cnr: f32, level: u8, ncw: usize, truth: usize, t: u32) -> Trial {
    let m = &*link.modem;
    let sent = burst(level, ncw, payload_seed(t));
    let cfg =
        FmChannelConfig { cnr_db: Some(cnr), seed: channel_seed(cnr, t), ctcss: link.ctcss(t), ..link.chan.clone() };
    let rx = transmit(&cfg, &link.modem.modulate(&sent));
    let tol = m.ofdm.p.ncp / 4;
    let det = Detector::ALL
        .iter()
        .map(|&d| {
            let o = receive(m, d, &rx, &sent);
            (o.first.is_some_and(|b| b.abs_diff(truth) <= tol), o.ok)
        })
        .collect();
    let genie = header_at(m, &rx, truth).is_some_and(|h| burst_ok(m, &rx, truth, &h, &sent));
    Trial { det, genie }
}

/// Where preamble B of a `level` burst of `ncw` codewords lands after the channel.
/// The channel's delays depend neither on its noise nor on the payload (the burst
/// layout is fixed by level and size), so one noiseless copy gives it for every trial.
/// The reference detector's fine timing on a noiseless copy without the CTCSS tone
/// defines the position (the tone does not move the burst).
/// Every detector must find it there within the hit tolerance, or a hit would mean
/// agreeing with the reference rather than finding the burst.
fn link_position(link: &Link, level: u8, ncw: usize) -> usize {
    let clean = FmChannelConfig { cnr_db: None, ..link.chan.clone() };
    let rx = transmit(&clean, &link.modem.modulate(&burst(level, ncw, payload_seed(0))));
    let ofdm = &link.modem.ofdm;
    let truth = ofdm.detect_with(REFERENCE, &rx, 0).expect("burst not found on a clean channel");
    for d in Detector::ALL {
        let at = ofdm.detect_with(d, &rx, 0);
        let near = at.is_some_and(|b| b.abs_diff(truth) <= ofdm.p.ncp / 4);
        assert!(near, "{d} finds the clean burst at {at:?}, {REFERENCE} at {truth}");
    }
    truth
}

/// Where preamble B of a `level` burst of `ncw` codewords lands after the `mode`
/// channel, sent and received by `m`.
pub(crate) fn true_position(m: &Modem, mode: &str, level: u8, ncw: usize) -> usize {
    let clean = FmChannelConfig { cnr_db: None, ..FmChannelConfig::by_name(mode).unwrap() };
    let tx = m.modulate(&burst(level, ncw, payload_seed(0)));
    m.ofdm.detect(&transmit(&clean, &tx), 0).expect("burst not found on a clean channel")
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

/// Run `trials` trials of a `level` burst of `ncw` codewords on every core.
fn run_trials(link: &Link, cnr: f32, level: u8, ncw: usize, trials: u32) -> Vec<Trial> {
    let truth = link_position(link, level, ncw);
    on_every_core(trials, |t| trial(link, cnr, level, ncw, truth, t))
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
/// does not parse stops the run: falling back would quietly run something else.
pub(crate) fn env_or<T: std::str::FromStr>(name: &str, default: T) -> T {
    match std::env::var(name) {
        Ok(v) => v.trim().parse().unwrap_or_else(|_| panic!("{name}={v:?} does not parse")),
        Err(_) => default,
    }
}

/// Environment variable `name` as a comma-separated list of `T`, or `default` when
/// unset. As with [`env_or`], an item that does not parse stops the run.
pub(crate) fn env_list<T: std::str::FromStr>(name: &str, default: &str) -> Vec<T>
where
    T::Err: std::fmt::Display,
{
    let v = std::env::var(name).unwrap_or_else(|_| default.to_string());
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
    match std::env::var(name).as_deref().map(str::trim) {
        Err(_) | Ok("0" | "false") => false,
        Ok("1" | "true") => true,
        Ok(v) => panic!("{name}={v:?}: use 1 or 0"),
    }
}

/// The kinds of audio without a burst in the false-detection pass, for [`noise_case`].
const NOISE_CASES: [&str; 5] =
    ["carrier at -10 dB CNR", "weak carrier, 5 dB CNR", "white noise", "keyed tones", "data without preambles"];

/// Tone sets for the "keyed tones" case, Hz. 750 + 1500 Hz repeat every nfft/2
/// samples in both modes, like preamble A; then a roger beep and DTMF pairs.
const KEYED_TONES: [&[u64]; 6] = [&[750, 1500], &[1000], &[697, 1209], &[770, 1336], &[852, 1477], &[941, 1633]];
/// On and off time of each keyed tone, seconds.
const KEYED_TONE_S: f32 = 0.3;

/// Audio without a burst, `seconds` long at `fs` Hz: case `case` of [`NOISE_CASES`].
/// Every case, mode and `seed` gets its own noise, so their minutes are independent.
///
/// The keyed tones stop after each 0.3 s: a tone that never stops cannot end a
/// Schmidl-Cox plateau, so it would never be evaluated as a detection. Each burst of
/// tone picks its set and starting phase at random, so the audio does not repeat.
///
/// The data case is what the receiver scans after a header that fails, or after it
/// misses a preamble: level 1 bursts of the longest size, leader and data, with the
/// preambles cut out, through the channel at 30 dB CNR.
fn noise_case(mode: &str, case: usize, seconds: f32, fs: f32, seed: u64) -> Vec<f32> {
    let n = (seconds * fs) as usize;
    assert!(n > 0, "{seconds} s of noise is less than one sample");
    let mode_index = OfdmParams::all().iter().position(|p| p.name == mode).expect("unknown mode");
    let seed = seed * 100 + case as u64 * 10 + mode_index as u64;
    // No silence around the carrier, so the audio is exactly `seconds` of noise.
    let base = FmChannelConfig {
        keyup_delay_s: 0.0,
        lead_silence_s: 0.0,
        trail_silence_s: 0.0,
        seed,
        ..FmChannelConfig::by_name(mode).unwrap()
    };
    let mut rng = Prng::new(seed as u32);
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

/// Detections and decoded headers in `rx`, scanning on after each detection.
fn false_detections(m: &Modem, d: Detector, rx: &[f32]) -> (u32, u32) {
    let (mut dets, mut headers) = (0, 0);
    let mut pos = 0;
    while let Some(b) = m.ofdm.detect_with(d, rx, pos) {
        dets += 1;
        if header_at(m, rx, b).is_some() {
            headers += 1;
        }
        pos = b + 1;
    }
    (dets, headers)
}

/// Processing time over audio duration of a `Receiver` using `detector` on `audio`,
/// fed in 10 ms chunks. The best of three runs, so that a cold cache or a busy core
/// does not set the figure.
fn receiver_load(mode: &str, detector: Detector, audio: &[f32]) -> f64 {
    let m = eval_modem(mode, detector);
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
    let noise_seed: u64 = env_or("NOISE_SEED", 1);
    let ctcss: f32 = env_or("CTCSS", 0.0);
    let chan: String = env_or("CHANNEL", "matched".to_string());
    let chan = chan.trim();
    let tx_tilt: f32 = env_or("TX_TILT", 0.0);
    assert!(CHANNELS.contains(&chan), "CHANNEL: unknown channel {chan:?} (one of {})", CHANNELS.join(", "));
    assert!(tx_tilt.is_finite() && tx_tilt.abs() <= 6.0, "TX_TILT={tx_tilt}: must be within -6..6 dB/octave");
    assert!(ctcss.is_finite() && ctcss >= 0.0, "CTCSS={ctcss}: must be a deviation in Hz, 0 for none");
    assert!((level as usize) < LEVELS.len(), "LEVEL {level}: levels are 0 to {}", LEVELS.len() - 1);
    assert!(trials > 0, "TRIALS must be at least 1");
    assert!(noise_s > 0.0, "NOISE_SECONDS={noise_s}: must be more than 0");
    let modes: Vec<&str> = modes.iter().map(String::as_str).collect();
    // Checked up front, so that a typo does not fail after the first mode has run.
    for mode in &modes {
        Link::new(mode, chan, tx_tilt).with_ctcss(ctcss);
    }
    let cnr_list = |mode: &str| -> Vec<f32> {
        let default = match mode {
            "narrow" => "3,4,5,6,7,8,10",
            "wide" => "5,6,7,8,9,10,12",
            _ => panic!("no default CNRS for mode {mode:?}: set CNRS"),
        };
        env_list("CNRS", default)
    };
    for mode in &modes {
        cnr_list(mode);
    }
    // `radio_tx_tilt`, not the `tx_tilt` of files from the second-modem tilt, whose
    // rows the header check must keep out.
    const CSV_HEADER: &str = "mode,channel,radio_tx_tilt,ctcss_hz,level,ncw,cnr_db,trials,detector,hits,bursts";
    let mut csv = std::env::var("CSV").ok().map(|p| {
        // Appending rows to a file with other columns would mix them up unnoticed.
        let old = std::fs::read_to_string(&p).unwrap_or_default();
        if let Some(head) = old.lines().next() {
            assert_eq!(head, CSV_HEADER, "CSV: {p} has other columns; use a new file");
        }
        let new = old.is_empty();
        let mut f = std::fs::OpenOptions::new().create(true).append(true).open(p).unwrap();
        if new {
            writeln!(f, "{CSV_HEADER}").unwrap();
        }
        f
    });
    let reference = Detector::ALL.iter().position(|&d| d == REFERENCE).expect("reference detector not in ALL");
    println!(
        "level {level} ({}), {ncw} codeword(s), {trials} trials per point, txdelay {TXDELAY} s, \
         channel {chan}, TX tilt {tx_tilt:+} dB/octave, CTCSS {ctcss} Hz, noise seed {noise_seed}",
        LEVELS[level as usize].name
    );
    for &mode in &modes {
        let link = Link::new(mode, chan, tx_tilt).with_ctcss(ctcss);
        let m = &*link.modem;
        // A longer burst would modulate, but its header would not unpack (MAX_NCW) or
        // the modem would never send it (MAX_PAYLOAD_SECONDS): every trial would fail.
        let max = m.max_ncw(level);
        assert!(ncw <= max, "NCW {ncw}: a level {level} burst in {mode} mode holds at most {max} codewords");
        println!("\n{mode}: hits and decoded bursts out of {trials}; paired test against '{REFERENCE}'");
        for cnr in cnr_list(mode) {
            let res = run_trials(&link, cnr, level, ncw, trials);
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
                    writeln!(f, "{mode},{chan},{tx_tilt},{ctcss},{level},{ncw},{cnr},{trials},{name},{hits},{bursts}")
                        .unwrap();
                }
            }
            if let Some(f) = &mut csv {
                writeln!(f, "{mode},{chan},{tx_tilt},{ctcss},{level},{ncw},{cnr},{trials},genie,,{genie}").unwrap();
            }
            println!("{line}");
        }
        println!("{mode}: per minute of audio without a burst, detections / decoded headers");
        let mut weak = Vec::new();
        for (c, case) in NOISE_CASES.iter().enumerate() {
            let audio = noise_case(mode, c, noise_s, m.fs(), noise_seed);
            let mut line = format!("  {case:26}");
            let per_min = 60.0 / (audio.len() as f32 / m.fs());
            for d in Detector::ALL {
                let (n, h) = false_detections(m, d, &audio);
                line += &format!(" | {d} {:7.2} / {:5.2}", n as f32 * per_min, h as f32 * per_min);
            }
            line += &format!(" | mf peak {:.3} (threshold {MF_THRESHOLD})", m.ofdm.mf_peak(&audio));
            println!("{line}");
            if c == 1 {
                weak = audio;
            }
        }
        let mut line = format!("{mode}: streaming receiver CPU on '{}', 10 ms chunks:", NOISE_CASES[1]);
        for d in Detector::ALL {
            line += &format!(" {d} {:.2} %", 100.0 * receiver_load(mode, d, &weak));
        }
        println!("{line}");
    }
}

#[test]
fn on_every_core_keeps_trial_order() {
    assert_eq!(on_every_core(37, |t| t), (0..37).collect::<Vec<_>>());
    assert!(on_every_core(0, |t| t).is_empty());
}

/// A CI guard on the matched filter's threshold, on noise the evaluation's noise cases
/// do not use: FM channel noise with other seeds and a fresh white-noise sequence, in
/// every mode, so a new mode must be calibrated. Setting `MF_MARGIN` prints the peaks.
#[test]
fn matched_filter_stays_below_threshold_on_unseen_noise() {
    let seconds = 5.0;
    for mode in OfdmParams::all().map(|p| p.name) {
        let m = modem(mode);
        let n = (seconds * m.fs()) as usize;
        let base = FmChannelConfig {
            keyup_delay_s: 0.0,
            lead_silence_s: 0.0,
            trail_silence_s: 0.0,
            ..FmChannelConfig::by_name(mode).unwrap()
        };
        let mut rng = Prng::new(0x0dd_5eed);
        let cases = [
            (
                "carrier at -10 dB",
                transmit(&FmChannelConfig { cnr_db: Some(-10.0), seed: 4242, ..base.clone() }, &vec![0.0; n]),
            ),
            ("carrier at 5 dB", transmit(&FmChannelConfig { cnr_db: Some(5.0), seed: 4243, ..base }, &vec![0.0; n])),
            ("white noise", (0..n).map(|_| 0.3 * rng.uniform()).collect()),
        ];
        for (name, audio) in cases {
            let peak = m.ofdm.mf_peak(&audio);
            if std::env::var_os("MF_MARGIN").is_some() {
                println!("{mode} {name}: mf peak {peak:.3}");
            }
            assert!(peak < MF_THRESHOLD, "{mode} {name}: mf peak {peak} reaches the threshold");
        }
    }
}

/// The harness itself: on a strong signal every detector and the genie decode every
/// burst, and no detector raises a false header on a carrier at -10 dB CNR. `hit` is
/// a diagnostic, so it is not required here.
#[test]
fn harness_sees_strong_bursts_and_no_false_headers() {
    for mode in OfdmParams::all().map(|p| p.name) {
        let link = Link::matched(mode);
        let m = &*link.modem;
        // The true position must not depend on the payload, or one copy could not give it.
        let truth = link_position(&link, 1, 1);
        let clean = FmChannelConfig { cnr_db: None, ..link.chan.clone() };
        for t in 1..4 {
            let tx = m.modulate(&burst(1, 1, payload_seed(t)));
            let at = m.ofdm.detect_with(REFERENCE, &transmit(&clean, &tx), 0);
            assert_eq!(at, Some(truth), "{mode}: payload {t} moves the burst");
        }
        for r in run_trials(&link, 25.0, 1, 1, 4) {
            assert!(r.genie, "{mode}: genie failed on a strong signal");
            assert!(r.det.iter().all(|&(_, ok)| ok), "{mode}: a detector missed a strong burst");
        }
        let audio = noise_case(mode, 0, 2.0, m.fs(), 1);
        for d in Detector::ALL {
            assert_eq!(false_detections(m, d, &audio).1, 0, "{mode}: {d} decoded a header from noise");
        }
    }
}

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

/// `TX_TILT` reaches the channel: on a clean channel, a tilted link's high carriers
/// gain on its low ones by the tilt times the octaves between them, within the
/// shelves' ripple and the band edges' easing.
#[test]
fn tx_tilt_reaches_the_channel() {
    for mode in OfdmParams::all().map(|p| p.name) {
        let power_ratio_db = |tilt: f32| {
            let link = Link::new(mode, "matched", tilt);
            let p = &link.modem.ofdm.p;
            // No squelch tail: its noise burst would fill the weak carriers' bins.
            let clean = FmChannelConfig { cnr_db: None, squelch_tail_s: 0.0, ..link.chan.clone() };
            let rx = transmit(&clean, &link.modem.modulate(&burst(1, 4, payload_seed(0))));
            // Mean single-bin periodogram over nfft-long blocks at carrier `k`.
            let power = |k: usize| {
                let w = 2.0 * std::f64::consts::PI * p.carrier_freq(k) as f64 / clean.fs as f64;
                let blocks: Vec<f64> = rx
                    .chunks_exact(p.nfft)
                    .map(|b| {
                        let (re, im) = b.iter().enumerate().fold((0f64, 0f64), |(re, im), (i, &x)| {
                            // Hann window: the tilt spreads carrier powers over tens of
                            // dB, which a bare block would leak into the weak ones.
                            let hann = 0.5 - 0.5 * (2.0 * std::f64::consts::PI * i as f64 / p.nfft as f64).cos();
                            let x = x as f64 * hann;
                            (re + x * (w * i as f64).cos(), im - x * (w * i as f64).sin())
                        });
                        re * re + im * im
                    })
                    .collect();
                blocks.iter().sum::<f64>() / blocks.len() as f64
            };
            let (lo, hi) = (2, p.num_carriers - 3);
            let octaves = (p.carrier_freq(hi) / p.carrier_freq(lo)).log2();
            (10.0 * (power(hi) / power(lo)).log10() as f32, octaves)
        };
        let (flat, octaves) = power_ratio_db(0.0);
        for tilt in [-6.0, -3.0, 3.0, 6.0] {
            let got = power_ratio_db(tilt).0 - flat;
            let want = tilt * octaves;
            assert!(
                (got - want).abs() < 2.0,
                "{mode} TX_TILT {tilt}: high carriers gained {got:.1} dB, want {want:.1}"
            );
        }
    }
}

/// A wide-mode radio passes the CTCSS tone it adds to the data port audio. At 750 Hz
/// deviation the genie and the matched filter decode every strong burst, whichever
/// standard tone. Schmidl-Cox does not: the tone's correlation over its nfft/2 lag
/// pulls the metric below its threshold. That is a defect of the existing detector,
/// so it is not asserted here.
#[test]
fn ctcss_tone_leaves_strong_bursts_to_the_matched_filter() {
    let link = Link::matched("wide").with_ctcss(750.0);
    let mf = Detector::ALL.iter().position(|&d| d == Detector::MatchedFilter).unwrap();
    for (t, r) in run_trials(&link, 25.0, 1, 1, CTCSS_HZ.len() as u32).iter().enumerate() {
        assert!(r.genie, "genie failed on a strong signal with {} Hz CTCSS", CTCSS_HZ[t]);
        assert!(r.det[mf].1, "the matched filter missed a strong burst with {} Hz CTCSS", CTCSS_HZ[t]);
    }
}

/// The matched filter's reason to exist, guarded in CI. Where bursts start to decode
/// and Schmidl-Cox lags behind the genie, the matched filter decodes all but at most
/// one of what the genie decodes; on the mismatch where it loses its advantage (wide,
/// de-emphasis only at the receiver) it decodes no fewer than the reference detector,
/// less one. A better detector passes; a worse threshold or template fails.
#[test]
fn matched_filter_holds_its_sensitivity_where_bursts_start_to_decode() {
    let mf = Detector::ALL.iter().position(|&d| d == Detector::MatchedFilter).unwrap();
    let reference = Detector::ALL.iter().position(|&d| d == REFERENCE).unwrap();
    let count = |res: &[Trial], i: usize| res.iter().filter(|r| r.det[i].1).count();
    for (mode, cnr) in [("narrow", 5.0), ("wide", 7.0)] {
        let res = run_trials(&Link::matched(mode), cnr, 1, 1, 40);
        let genie = res.iter().filter(|r| r.genie).count();
        assert!(genie >= 30, "{mode} {cnr} dB: genie decodes only {genie} of 40; pick another point");
        let got = count(&res, mf);
        assert!(got + 1 >= genie, "{mode} {cnr} dB: matched filter {got}, genie {genie}");
    }
    let res = run_trials(&Link::new("wide", "rx-de-only", 0.0), 7.0, 1, 1, 40);
    let (got, sc) = (count(&res, mf), count(&res, reference));
    assert!(got + 1 >= sc, "wide rx-de-only 7 dB: matched filter {got}, {REFERENCE} {sc}");
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

/// `receive` must judge each recording the way the streaming `Receiver` does when
/// it is fed the same audio in 10 ms chunks, including the recordings where the
/// detector or the header fails. It runs for every detector the receiver can use.
///
/// Each seed runs at three CNRs per mode. At the lowest even genie decodes nothing
/// and at the highest every detector so far decodes almost everything, so both
/// outcomes occur whatever the receiver's detector is, and `STREAM_SEEDS=1` works.
/// 12 seeds per mode by default.
#[test]
fn offline_receive_matches_the_streaming_receiver() {
    let seeds: u64 = env_or("STREAM_SEEDS", 12);
    for mode in OfdmParams::all().map(|p| p.name) {
        let cnrs = match mode {
            "narrow" => [3.0, 5.0, 8.0],
            "wide" => [5.0, 7.0, 10.0],
            _ => panic!("no CNRs for mode {mode}: add three, from where genie decodes nothing to almost all"),
        };
        let sent = burst(1, 1, 7);
        let tx = modem(mode).modulate(&sent);
        for detector in Detector::ALL {
            let m = eval_modem(mode, detector);
            let (mut total, mut ok) = (0, 0);
            for seed in 0..seeds {
                for cnr in cnrs {
                    let cfg = FmChannelConfig {
                        cnr_db: Some(cnr),
                        seed: 1000 + seed,
                        ..FmChannelConfig::by_name(mode).unwrap()
                    };
                    let mut rx = transmit(&cfg, &tx);
                    rx.extend(vec![0.0; m.fs() as usize]);
                    let mut receiver = Receiver::new(m.clone());
                    // 10 ms chunks, as a sound card delivers them.
                    let chunk = (m.fs() / 100.0) as usize;
                    let got: Vec<RxBurst> = rx.chunks(chunk).flat_map(|c| receiver.push(c)).collect();
                    let streamed = got.first().is_some_and(|g| {
                        g.header == sent.header
                            && g.payload.iter().zip(&sent.payload).all(|(p, s)| p.as_ref() == Some(s))
                    });
                    let offline = receive(&m, detector, &rx, &sent).ok;
                    assert_eq!(
                        offline, streamed,
                        "{mode} {detector} {cnr} dB seed {seed}: offline {offline}, streaming {streamed}"
                    );
                    total += 1;
                    ok += streamed as u32;
                }
            }
            // Both outcomes must occur, or the comparison proves little.
            assert!(ok > 0 && ok < total, "{mode} {detector}: {ok} of {total} bursts decoded");
        }
    }
}
