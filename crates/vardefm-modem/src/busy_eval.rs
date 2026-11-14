//! Side-by-side evaluation of channel-busy detectors on received audio.
//!
//! Every detector sees exactly the same audio, read back every millisecond. Each
//! case is either a signal the channel must be busy for or interference it must stay
//! idle on. Signals go through the FM channel simulator and arrive after 3 s of
//! closed squelch (or open-squelch hiss), with a random sub-block offset. Interference
//! starts after 3 s of closed squelch and lasts 10 s.
//!
//! Must be busy, per detector:
//! - `busy`: the fraction of the signal time (squelch open to carrier drop) reported
//!   busy, averaged over the trials. Pauses in speech longer than the hold time count
//!   against every detector alike;
//! - `ON`: median and worst time from the signal start to busy, ms; `missed` counts
//!   the trials that were never busy during the signal;
//! - `OFF`: median and worst time from the carrier drop to the end of the busy spell
//!   it falls in, ms, over the `held` trials still busy at the drop. It includes the
//!   simulator's 100 ms squelch tail. A detector that learned the signal as
//!   background before the drop is not counted, and neither is busy that comes back
//!   later (hiss returning after the burst).
//!
//! Must stay idle: `false`, the fraction of the interference time reported busy (mean
//! and worst trial), and `in`, how many trials were busy at all.
//!
//! Levels: open-squelch hiss at -27 dBFS RMS as measured on a real radio; CTCSS at
//! 375, 750 and 1500 Hz deviation through the wide channel (3 kHz peak deviation),
//! every EIA tone from 67.0 to 254.1 Hz in turn; mains hum at -30 dBFS with harmonics
//! up to 1 kHz falling as 1/k². These are not measured and are stress cases: CTCSS at
//! 1500 Hz deviation (twice the usual), hum at -20 dBFS under a voice at -30 dBFS, and
//! hiss above the band, which is added after the simulator because its receive
//! filter ends at 7 kHz while a data port's discriminator output need not.
//!
//! Every case draws its audio from a seed made of its name and the trial number, so
//! adding or reordering cases leaves the others unchanged.
//!
//! `cargo test --release -p vardefm-modem busy_eval -- --ignored --nocapture`
//!
//! Environment: `MODES` (narrow,wide), `TRIALS` per case (63, three per CTCSS tone),
//! `CNR` of the simulated signals (20 dB; the strong-carrier cases are at 40 dB).

use crate::busy::BusyDetector;
use crate::{Burst, Header, Kind, Modem, OfdmParams};
use std::f32::consts::PI;
use std::ops::Range;
use std::sync::Arc;
use vardefm_dsp::filter::Chain;
use vardefm_dsp::prng::Prng;
use vardefm_sim::{transmit, Ctcss, FmChannelConfig};

/// A busy detector for the mode with these parameters.
pub(crate) type MakeFn = fn(&OfdmParams) -> BusyDetector;

/// Detectors under evaluation.
pub(crate) const DETECTORS: &[(&str, MakeFn)] =
    &[("broadband", |p| BusyDetector::broadband(p.fs)), ("in-band", BusyDetector::new)];

const FS: f32 = 48_000.0;
/// The detectors are read after every `STEP` samples (1 ms).
const STEP: usize = 48;
/// Closed squelch (or learned background) before each signal or interference, seconds.
const LEAD_S: f32 = 3.0;
const INTERFERENCE_S: f32 = 10.0;
const HISS_RMS: f32 = 0.045;
const HUM_RMS: f32 = 0.0316;
/// Voice into the transmitter.
const VOICE_RMS: f32 = 0.1;
/// Received level of the quiet voice under strong hum, and of that hum.
const QUIET_VOICE_RMS: f32 = 0.03;
const STRONG_HUM_RMS: f32 = 0.1;
pub(crate) const CTCSS_HZ: [f32; 21] = [
    67.0, 71.9, 77.0, 82.5, 88.5, 94.8, 100.0, 107.2, 114.8, 123.0, 131.8, 141.3, 151.4, 162.2, 173.8, 186.2, 203.5,
    218.1, 233.6, 250.3, 254.1,
];
const CTCSS_DEVIATION_HZ: [f32; 3] = [375.0, 750.0, 1500.0];

struct Ctx {
    mode: &'static str,
    modem: Arc<Modem>,
    cnr: f32,
}

fn ctx(mode: &'static str, cnr: f32) -> Ctx {
    let mut m = Modem::new(OfdmParams::by_name(mode).unwrap(), 0.15);
    m.decode_threads = 1;
    Ctx { mode, modem: Arc::new(m), cnr }
}

/// Received audio and where the signal or interference in it lies, in samples.
struct Clip {
    audio: Vec<f32>,
    signal: Range<usize>,
}

#[derive(Clone, Copy, PartialEq)]
enum Expect {
    Busy,
    Idle,
}

struct Case {
    name: &'static str,
    expect: Expect,
    wide_only: bool,
    /// Build the clip for trial `t`.
    make: fn(&Ctx, &mut Prng, u32) -> Clip,
}

const CASES: &[Case] = &[
    Case { name: "VardeFM burst", expect: Expect::Busy, wide_only: false, make: burst },
    Case { name: "voice", expect: Expect::Busy, wide_only: false, make: voice_case },
    Case { name: "burst under open-squelch hiss", expect: Expect::Busy, wide_only: false, make: burst_under_hiss },
    Case { name: "quiet voice under strong hum", expect: Expect::Busy, wide_only: false, make: voice_under_hum },
    Case { name: "burst with CTCSS", expect: Expect::Busy, wide_only: true, make: burst_with_ctcss },
    Case { name: "voice with CTCSS", expect: Expect::Busy, wide_only: true, make: voice_with_ctcss },
    Case { name: "open-squelch hiss", expect: Expect::Idle, wide_only: false, make: hiss_case },
    Case { name: "mains hum 50 Hz", expect: Expect::Idle, wide_only: false, make: hum_50 },
    Case { name: "mains hum 60 Hz", expect: Expect::Idle, wide_only: false, make: hum_60 },
    Case { name: "DC offset steps", expect: Expect::Idle, wide_only: false, make: dc_steps },
    Case { name: "carrier, no audio", expect: Expect::Idle, wide_only: false, make: silent_carrier },
    Case { name: "carrier with CTCSS only", expect: Expect::Idle, wide_only: true, make: ctcss_carrier },
    Case { name: "strong carrier, no audio", expect: Expect::Idle, wide_only: false, make: strong_silent_carrier },
    Case { name: "strong carrier, CTCSS only", expect: Expect::Idle, wide_only: true, make: strong_ctcss_carrier },
    Case { name: "hiss above the band", expect: Expect::Idle, wide_only: true, make: hiss_above_band },
    Case { name: "hiss above band, chattering", expect: Expect::Idle, wide_only: true, make: hiss_above_band_chatter },
];

fn uniform(rng: &mut Prng) -> f32 {
    rng.next_u32() as f32 / u32::MAX as f32 * 2.0 - 1.0
}

fn samples(seconds: f32) -> usize {
    (seconds * FS) as usize
}

/// Between `lo` and `hi` seconds, in samples.
fn between(rng: &mut Prng, lo: f32, hi: f32) -> usize {
    samples(lo + (hi - lo) * (uniform(rng) + 1.0) / 2.0)
}

fn rms(v: &[f32]) -> f32 {
    (v.iter().map(|x| x * x).sum::<f32>() / v.len().max(1) as f32).sqrt()
}

fn scale(v: &mut [f32], k: f32) {
    v.iter_mut().for_each(|x| *x *= k);
}

fn add(a: &mut [f32], b: &[f32]) {
    a.iter_mut().zip(b).for_each(|(x, y)| *x += y);
}

/// A simulator or hiss seed, drawn from the trial's generator so that it depends on
/// the case name as well as the trial.
fn seed(rng: &mut Prng) -> u64 {
    rng.next_u32() as u64
}

/// The lead in front of every signal: 3 s plus up to one 10 ms block, so signals do
/// not all start on a block boundary.
fn lead(rng: &mut Prng) -> usize {
    samples(LEAD_S) + (rng.next_u32() % 480) as usize
}

/// `audio` sent over the simulated channel after the closed-squelch lead.
fn on_air(ctx: &Ctx, rng: &mut Prng, audio: &[f32]) -> Clip {
    on_air_at(ctx.cnr, ctx, rng, audio, None)
}

/// `audio` sent at `cnr` dB by a radio that adds the CTCSS tone `ctcss`.
fn on_air_at(cnr: f32, ctx: &Ctx, rng: &mut Prng, audio: &[f32], ctcss: Option<Ctcss>) -> Clip {
    let lead = lead(rng);
    let cfg = FmChannelConfig {
        cnr_db: Some(cnr),
        seed: seed(rng),
        lead_silence_s: lead as f32 / FS,
        trail_silence_s: 2.0,
        ctcss,
        ..FmChannelConfig::by_name(ctx.mode).unwrap()
    };
    let start = lead + samples(cfg.keyup_delay_s);
    Clip { audio: transmit(&cfg, audio), signal: start..lead + audio.len() }
}

/// Open-squelch hiss: a carrier far below the noise, scaled to `HISS_RMS`.
fn hiss(ctx: &Ctx, seed: u64, n: usize) -> Vec<f32> {
    let cfg = FmChannelConfig {
        cnr_db: Some(-10.0),
        seed,
        keyup_delay_s: 0.0,
        lead_silence_s: 0.0,
        trail_silence_s: 0.0,
        ..FmChannelConfig::by_name(ctx.mode).unwrap()
    };
    let mut h = transmit(&cfg, &vec![0.0; n]);
    let k = HISS_RMS / rms(&h);
    scale(&mut h, k);
    h
}

fn burst_audio(ctx: &Ctx, rng: &mut Prng) -> Vec<f32> {
    let (level, ncw) = (1, 4);
    let n = Modem::payload_bytes(level);
    let b = Burst {
        header: Header { kind: Kind::Data, session: 0x42, level, ncw, ..Default::default() },
        payload: (0..ncw).map(|_| (0..n).map(|_| rng.next_u32() as u8).collect()).collect(),
    };
    ctx.modem.modulate(&b)
}

/// Speech-like audio at `VOICE_RMS`: a pitched pulse train (100-150 Hz, drifting)
/// band-limited to 300-3400 Hz, in syllables of 120-300 ms with short gaps and
/// pauses of 200-500 ms between words.
fn voice(rng: &mut Prng, seconds: f32) -> Vec<f32> {
    let n = samples(seconds);
    let mut env = vec![0f32; n];
    let (mut i, mut left_in_word) = (0, 3);
    while i < n {
        if left_in_word == 0 {
            i += between(rng, 0.2, 0.5);
            left_in_word = 2 + rng.next_u32() % 4;
        }
        let len = between(rng, 0.12, 0.3);
        for (j, e) in env.iter_mut().skip(i).take(len).enumerate() {
            *e = (PI * j as f32 / len as f32).sin().powi(2);
        }
        i += len + between(rng, 0.03, 0.1);
        left_in_word -= 1;
    }
    let f0 = 100.0 + 50.0 * (uniform(rng) + 1.0) / 2.0;
    let mut bp = Chain::bandpass4(FS, 300.0, 3400.0);
    let mut phase = 0f32;
    let mut out: Vec<f32> = env
        .iter()
        .enumerate()
        .map(|(k, e)| {
            phase += f0 * (1.0 + 0.1 * (2.0 * PI * 0.7 * k as f32 / FS).sin()) / FS;
            let pulse = if phase >= 1.0 {
                phase -= 1.0;
                1.0
            } else {
                0.0
            };
            bp.process(pulse + 0.02 * uniform(rng)) * e
        })
        .collect();
    let k = VOICE_RMS / rms(&out);
    scale(&mut out, k);
    out
}

/// Mains hum: harmonics `k` = 1.. up to 1 kHz at amplitude 1/k², fundamental at `rms`.
fn hum(rng: &mut Prng, f: f32, rms: f32, n: usize) -> Vec<f32> {
    let harmonics: Vec<(f32, f32, f32)> = (1..=(1000.0 / f) as usize)
        .map(|k| (k as f32 * f, rms * 2f32.sqrt() / (k * k) as f32, PI * uniform(rng)))
        .collect();
    (0..n)
        .map(|i| {
            let t = i as f32 / FS;
            harmonics.iter().map(|&(fk, a, ph)| a * (2.0 * PI * fk * t + ph).sin()).sum()
        })
        .collect()
}

/// Trial `t`'s CTCSS tone, which the wide transmitter adds: tones in turn, then the
/// other deviation.
fn ctcss(t: u32) -> Ctcss {
    Ctcss {
        hz: CTCSS_HZ[t as usize % CTCSS_HZ.len()] as f64,
        deviation_hz: CTCSS_DEVIATION_HZ[t as usize / CTCSS_HZ.len() % CTCSS_DEVIATION_HZ.len()],
        phase: 0.0,
    }
}

/// White noise above the wide band (8th-order highpass at 7 kHz) at `HISS_RMS`.
fn hiss_above(rng: &mut Prng, n: usize) -> Vec<f32> {
    let mut hp = Chain::highpass4(FS, 7000.0).then(Chain::highpass4(FS, 7000.0));
    let mut h: Vec<f32> = (0..n).map(|_| hp.process(uniform(rng))).collect();
    let k = HISS_RMS / rms(&h);
    scale(&mut h, k);
    h
}

/// On/off gate starting on, with spells of `on` and `off` seconds (ranges).
fn gate(rng: &mut Prng, n: usize, on: (f32, f32), off: (f32, f32)) -> Vec<bool> {
    let mut g = Vec::with_capacity(n);
    let mut state = true;
    while g.len() < n {
        let (lo, hi) = if state { on } else { off };
        let len = between(rng, lo, hi).min(n - g.len());
        g.extend(std::iter::repeat_n(state, len));
        state = !state;
    }
    g
}

/// `interference` after the closed-squelch lead.
fn after_silence(rng: &mut Prng, interference: Vec<f32>) -> Clip {
    let lead = lead(rng);
    let mut audio = vec![0.0; lead];
    audio.extend(interference);
    Clip { signal: lead..audio.len(), audio }
}

fn burst(ctx: &Ctx, rng: &mut Prng, _: u32) -> Clip {
    let a = burst_audio(ctx, rng);
    on_air(ctx, rng, &a)
}

fn voice_case(ctx: &Ctx, rng: &mut Prng, _: u32) -> Clip {
    let a = voice(rng, 4.0);
    on_air(ctx, rng, &a)
}

/// Listening to open-squelch hiss when a station sends a burst. Its carrier quiets
/// the hiss; the hiss is back when the carrier drops.
fn burst_under_hiss(ctx: &Ctx, rng: &mut Prng, _: u32) -> Clip {
    let a = burst_audio(ctx, rng);
    let cfg = FmChannelConfig {
        cnr_db: Some(ctx.cnr),
        seed: seed(rng),
        keyup_delay_s: 0.0,
        squelch_tail_s: 0.0,
        lead_silence_s: 0.0,
        trail_silence_s: 0.0,
        ..FmChannelConfig::by_name(ctx.mode).unwrap()
    };
    let mut audio = hiss(ctx, seed(rng), lead(rng));
    let start = audio.len();
    audio.extend(transmit(&cfg, &a));
    let signal = start..audio.len();
    audio.extend(hiss(ctx, seed(rng), samples(2.0)));
    Clip { audio, signal }
}

/// A quiet voice received under strong hum from the sound card's ground loop.
fn voice_under_hum(ctx: &Ctx, rng: &mut Prng, t: u32) -> Clip {
    let a = voice(rng, 4.0);
    let mut clip = on_air(ctx, rng, &a);
    let k = QUIET_VOICE_RMS / rms(&clip.audio[clip.signal.clone()]);
    scale(&mut clip.audio, k);
    let f = if t.is_multiple_of(2) { 50.0 } else { 60.0 };
    let h = hum(rng, f, STRONG_HUM_RMS, clip.audio.len());
    add(&mut clip.audio, &h);
    clip
}

fn burst_with_ctcss(ctx: &Ctx, rng: &mut Prng, t: u32) -> Clip {
    let a = burst_audio(ctx, rng);
    on_air_at(ctx.cnr, ctx, rng, &a, Some(ctcss(t)))
}

fn voice_with_ctcss(ctx: &Ctx, rng: &mut Prng, t: u32) -> Clip {
    let a = voice(rng, 4.0);
    on_air_at(ctx.cnr, ctx, rng, &a, Some(ctcss(t)))
}

/// The squelch opens on hiss and stays open.
fn hiss_case(ctx: &Ctx, rng: &mut Prng, _: u32) -> Clip {
    let h = hiss(ctx, seed(rng), samples(INTERFERENCE_S));
    after_silence(rng, h)
}

fn hum_50(_: &Ctx, rng: &mut Prng, _: u32) -> Clip {
    let h = hum(rng, 50.0, HUM_RMS, samples(INTERFERENCE_S));
    after_silence(rng, h)
}

/// A 60 Hz period does not fit a 10 ms block, so the block power of 60 Hz hum swings.
fn hum_60(_: &Ctx, rng: &mut Prng, _: u32) -> Clip {
    let h = hum(rng, 60.0, HUM_RMS, samples(INTERFERENCE_S));
    after_silence(rng, h)
}

/// A DC-coupled input whose offset (0.05, -26 dBFS) comes and goes, as with a
/// squelch gate on the radio's side.
fn dc_steps(_: &Ctx, rng: &mut Prng, _: u32) -> Clip {
    let g = gate(rng, samples(INTERFERENCE_S), (0.5, 2.0), (0.5, 2.0));
    after_silence(rng, g.iter().map(|&on| if on { 0.05 } else { 0.0 }).collect())
}

/// A keyed carrier with no audio on it.
fn silent_carrier(ctx: &Ctx, rng: &mut Prng, _: u32) -> Clip {
    on_air(ctx, rng, &vec![0.0; samples(INTERFERENCE_S)])
}

/// A keyed carrier with only a CTCSS tone on it.
fn ctcss_carrier(ctx: &Ctx, rng: &mut Prng, t: u32) -> Clip {
    on_air_at(ctx.cnr, ctx, rng, &vec![0.0; samples(INTERFERENCE_S)], Some(ctcss(t)))
}

/// The strong-carrier cases separate what the carrier's own noise does (at 20 dB
/// CNR it is well above -40 dBFS) from what a CTCSS tone does.
const STRONG_CNR: f32 = 40.0;

fn strong_silent_carrier(ctx: &Ctx, rng: &mut Prng, _: u32) -> Clip {
    on_air_at(STRONG_CNR, ctx, rng, &vec![0.0; samples(INTERFERENCE_S)], None)
}

fn strong_ctcss_carrier(ctx: &Ctx, rng: &mut Prng, t: u32) -> Clip {
    on_air_at(STRONG_CNR, ctx, rng, &vec![0.0; samples(INTERFERENCE_S)], Some(ctcss(t)))
}

/// The squelch opens on hiss that lies entirely above the band.
fn hiss_above_band(_: &Ctx, rng: &mut Prng, _: u32) -> Clip {
    let h = hiss_above(rng, samples(INTERFERENCE_S));
    after_silence(rng, h)
}

/// As `hiss_above_band`, with the squelch opening and closing every 0.2-1.5 s.
fn hiss_above_band_chatter(_: &Ctx, rng: &mut Prng, _: u32) -> Clip {
    let n = samples(INTERFERENCE_S);
    let mut h = hiss_above(rng, n);
    for (x, on) in h.iter_mut().zip(gate(rng, n, (0.2, 1.5), (0.2, 1.5))) {
        if !on {
            *x = 0.0;
        }
    }
    after_silence(rng, h)
}

/// The detector's busy state after every `STEP` samples.
fn timeline(make: MakeFn, p: &OfdmParams, audio: &[f32]) -> Vec<bool> {
    let mut d = make(p);
    audio
        .chunks(STEP)
        .map(|c| {
            d.push(c);
            d.busy()
        })
        .collect()
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Metrics {
    /// Fraction of the signal time reported busy.
    busy: f32,
    /// From the signal start to busy, if it was busy during the signal at all.
    on_ms: Option<f32>,
    /// From the signal end to the end of the busy spell it falls in, if busy then.
    off_ms: Option<f32>,
}

fn measure(timeline: &[bool], signal: &Range<usize>) -> Metrics {
    let a = signal.start / STEP;
    let b = signal.end.div_ceil(STEP).min(timeline.len());
    let ms = |n: usize| n as f32 / FS * 1000.0;
    let during = &timeline[a..b];
    Metrics {
        busy: during.iter().filter(|&&x| x).count() as f32 / during.len().max(1) as f32,
        on_ms: during.iter().position(|&x| x).map(|i| ms(((a + i + 1) * STEP).saturating_sub(signal.start))),
        off_ms: (b > a && timeline[b - 1]).then(|| {
            let run = timeline[b..].iter().take_while(|&&x| x).count();
            ms((b + run) * STEP - signal.end)
        }),
    }
}

/// The clip of trial `t` of case `ci`, drawn from seed `name_seed(case) + t`.
fn clip(ctx: &Ctx, ci: usize, t: u32) -> Clip {
    let case = &CASES[ci];
    let mut rng = Prng::new(name_seed(case.name).wrapping_add(t));
    let clip = (case.make)(ctx, &mut rng, t);
    if case.expect == Expect::Busy {
        assert!(clip.signal.end + samples(1.0) <= clip.audio.len(), "{}: too short", case.name);
    }
    clip
}

/// Metrics of every detector (inner) for every trial (outer), on every core.
fn run_case(ctx: &Ctx, ci: usize, trials: u32) -> Vec<Vec<Metrics>> {
    let p = &ctx.modem.ofdm.p;
    let threads = std::thread::available_parallelism().map_or(1, |n| n.get()) as u32;
    let mut out: Vec<(u32, Vec<Metrics>)> = std::thread::scope(|s| {
        let workers: Vec<_> = (0..threads)
            .map(|th| {
                s.spawn(move || {
                    (th..trials)
                        .step_by(threads as usize)
                        .map(|t| {
                            let clip = clip(ctx, ci, t);
                            let m = DETECTORS
                                .iter()
                                .map(|(_, make)| measure(&timeline(*make, p, &clip.audio), &clip.signal))
                                .collect();
                            (t, m)
                        })
                        .collect::<Vec<_>>()
                })
            })
            .collect();
        workers.into_iter().flat_map(|w| w.join().unwrap()).collect()
    });
    out.sort_by_key(|(t, _)| *t);
    out.into_iter().map(|(_, m)| m).collect()
}

/// FNV-1a of the case name.
fn name_seed(name: &str) -> u32 {
    name.bytes().fold(0x811c_9dc5, |h, b| (h ^ b as u32).wrapping_mul(0x0100_0193))
}

/// "median/worst" in ms, or dashes when there is nothing to report.
fn median_max(mut v: Vec<f32>) -> String {
    if v.is_empty() {
        return "   -/   -".to_string();
    }
    v.sort_by(f32::total_cmp);
    format!("{:4.0}/{:4.0}", v[v.len() / 2], v[v.len() - 1])
}

fn env_or<T: std::str::FromStr>(name: &str, default: T) -> T {
    std::env::var(name).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

fn summary(expect: Expect, name: &str, res: &[Vec<Metrics>], d: usize) -> String {
    let m: Vec<Metrics> = res.iter().map(|r| r[d]).collect();
    let mean_busy = 100.0 * m.iter().map(|x| x.busy).sum::<f32>() / m.len() as f32;
    match expect {
        Expect::Busy => {
            let on = median_max(m.iter().filter_map(|x| x.on_ms).collect());
            let offs: Vec<f32> = m.iter().filter_map(|x| x.off_ms).collect();
            let held = offs.len();
            let off = median_max(offs);
            let missed = m.iter().filter(|x| x.on_ms.is_none()).count();
            format!(" | {name} {mean_busy:5.1}% ON {on} OFF {off} held {held:2} missed {missed:2}")
        }
        Expect::Idle => {
            let worst = 100.0 * m.iter().map(|x| x.busy).fold(0.0, f32::max);
            let any = m.iter().filter(|x| x.busy > 0.0).count();
            format!(" | {name} false {mean_busy:5.1}% max {worst:5.1}% in {any:2}/{}", m.len())
        }
    }
}

#[test]
#[ignore]
fn busy_eval() {
    let modes: String = env_or("MODES", "narrow,wide".to_string());
    let trials: u32 = env_or("TRIALS", 63);
    let cnr: f32 = env_or("CNR", 20.0);
    println!("{trials} trials per case, signals at {cnr} dB CNR");
    // Checked before anything runs, so a typo does not show up after minutes.
    let modes: Vec<&'static str> = modes
        .split(',')
        .map(|m| match m.trim() {
            "narrow" => "narrow",
            "wide" => "wide",
            m => panic!("MODES: unknown mode {m:?}, expected narrow and/or wide"),
        })
        .collect();
    for mode in modes {
        let ctx = ctx(mode, cnr);
        for (expect, title) in [
            (Expect::Busy, "must be busy: busy % of signal time, ON and OFF ms median/worst, held at the drop"),
            (Expect::Idle, "must stay idle: busy % of interference time, mean and worst trial"),
        ] {
            println!("\n{}, {title}", ctx.mode);
            for (ci, case) in CASES.iter().enumerate() {
                if case.expect != expect || (case.wide_only && ctx.mode != "wide") {
                    continue;
                }
                let res = run_case(&ctx, ci, trials);
                let mut line = format!("  {:30}", case.name);
                for (d, (name, _)) in DETECTORS.iter().enumerate() {
                    line += &summary(expect, name, &res, d);
                }
                println!("{line}");
            }
        }
    }
}

fn case(name: &str) -> usize {
    CASES.iter().position(|c| c.name == name).unwrap()
}

/// The harness itself: every detector is busy for a whole burst and lets go soon
/// after it, and is idle again within the history on a steady open-squelch hiss.
#[test]
fn harness_sees_a_burst_and_learns_hiss() {
    for mode in ["narrow", "wide"] {
        let ctx = ctx(mode, 20.0);
        for r in run_case(&ctx, case("VardeFM burst"), 2) {
            for (m, (name, _)) in r.iter().zip(DETECTORS) {
                assert!(m.busy > 0.4, "{mode}: {name} busy for {:.2} of a burst", m.busy);
                assert!(m.on_ms.unwrap() < 50.0 && m.off_ms.is_none_or(|o| o < 400.0), "{mode}: {name} {m:?}");
            }
        }
        for r in run_case(&ctx, case("open-squelch hiss"), 2) {
            for (m, (name, _)) in r.iter().zip(DETECTORS) {
                assert!(m.busy < 0.25, "{mode}: {name} busy for {:.2} of steady hiss", m.busy);
            }
        }
    }
}

/// What the band limit is for, on a few trials: out-of-band sound never reads busy
/// and does not hide a voice in the band.
#[test]
fn in_band_ignores_sound_outside_the_band() {
    let d = DETECTORS.iter().position(|(n, _)| *n == "in-band").unwrap();
    for mode in ["narrow", "wide"] {
        let ctx = ctx(mode, 20.0);
        for name in [
            "mains hum 50 Hz",
            "mains hum 60 Hz",
            "DC offset steps",
            "hiss above the band",
            "strong carrier, CTCSS only",
        ] {
            let ci = case(name);
            if CASES[ci].wide_only && mode != "wide" {
                continue;
            }
            for r in run_case(&ctx, ci, 2) {
                // The squelch-open click on a CTCSS tone reads busy for the hold time.
                assert!(r[d].busy < 0.02, "{mode}: in-band busy for {:.3} of {name}", r[d].busy);
            }
        }
        for r in run_case(&ctx, case("quiet voice under strong hum"), 2) {
            assert!(r[d].on_ms.is_some() && r[d].busy > 0.5, "{mode}: in-band missed a voice under hum: {:?}", r[d]);
        }
    }
}

/// The CTCSS case the low band edge is designed for: 254.1 Hz, the highest tone, at
/// every deviation up to 1500 Hz on a clean carrier. Only the click when the squelch
/// opens may read busy.
#[test]
fn in_band_ignores_the_highest_ctcss_tone() {
    let (_, make) = DETECTORS.iter().find(|(n, _)| *n == "in-band").unwrap();
    let ctx = ctx("wide", 20.0);
    let ci = case("strong carrier, CTCSS only");
    let top = CTCSS_HZ.len() as u32 - 1;
    for dev in 0..CTCSS_DEVIATION_HZ.len() as u32 {
        // `ctcss` takes the tone from `t % 21` and the deviation from `t / 21 % 3`.
        let t = dev * CTCSS_HZ.len() as u32 + top;
        let c = clip(&ctx, ci, t);
        let tl = timeline(*make, &ctx.modem.ofdm.p, &c.audio);
        let settled = (c.signal.start + samples(0.5)) / STEP..c.signal.end / STEP;
        let late = tl[settled].iter().filter(|&&b| b).count();
        assert_eq!(
            late, 0,
            "254.1 Hz at {} Hz deviation: busy for {late} ms after onset",
            CTCSS_DEVIATION_HZ[dev as usize]
        );
    }
}

#[test]
fn measure_reads_the_timeline() {
    // Busy from step 12 to 21 (ms) and again from 30 to 31, signal from sample
    // 10 * 48 + 24 to 20 * 48. The later spell is not part of turning off.
    let tl: Vec<bool> = (0..40).map(|i| (12..22).contains(&i) || (30..32).contains(&i)).collect();
    let m = measure(&tl, &(10 * STEP + 24..20 * STEP));
    assert_eq!(m.busy, 0.8);
    assert_eq!(m.on_ms, Some(2.5));
    assert_eq!(m.off_ms, Some(2.0));
    // Busy only before the drop: it let go early, so there is no turn-off time.
    let tl: Vec<bool> = (0..40).map(|i| (12..15).contains(&i)).collect();
    assert_eq!(measure(&tl, &(10 * STEP..20 * STEP)).off_ms, None);
    let m = measure(&[false; 40], &(10 * STEP..20 * STEP));
    assert_eq!((m.busy, m.on_ms, m.off_ms), (0.0, None, None));
}
