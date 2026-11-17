//! Decoding performance per mode and speed level, against a recorded baseline.
//!
//! Two error rates through the FM channel simulator, both from the known burst
//! position, so detection is left out (the detector evaluation, `detect_eval`,
//! covers it):
//! - per level, the codeword error rate (CER), with the sent header as channel
//!   training, as `Receiver::step` decodes the payload once the header has decoded;
//! - per mode and drive, the header error rate (HER): the header decoded from
//!   preamble B alone, as `Receiver::step` does first. The header is coded and
//!   modulated the same at every level, but `Modem::modulate` scales the whole
//!   burst to its level's drive (`Modem::tx_rms`), so the header of a high level
//!   goes out quieter. One point per mode and drive covers it, measured on
//!   one-codeword bursts of the lowest level sent at that drive.
//!
//! Why one point per drive should suffice: every symbol of a burst carries the same
//! mean power before scaling, so the header's level depends on the drive and hardly
//! on the burst's length or payload. Clipping, heavy at wide mode's 0.5 drive, does
//! depend on the header codeword: with one fixed header the useful level at that
//! drive varied by 0.3 dB between levels and codeword counts. So each trial draws
//! the header fields that change from burst to burst at random (`trial_header`);
//! for a level's codewords, that header is also the channel training.
//! `header_level_depends_only_on_drive` checks the claim as an estimate, not as a
//! measured HER: on the noiseless transmitted header, averaged over those draws, the
//! useful level less the clipping distortion (counted as noise at
//! `HEADER_REFERENCE_SNR_DB`) agrees within `HEADER_LEVEL_TOLERANCE_DB` at every
//! drive.
//!
//! A burst decodes only if both its header and its codewords do. At level 0's drive
//! the header's point lies above level 0's, so there the header limits a burst, and
//! its curve is far shallower than any level's: at level 1's point a few per cent of
//! headers still fail.
//!
//! What the gate does not cover:
//! - The receiver's SNR estimate, which the link's rate thresholds
//!   (`SNR_THRESHOLDS_DB`) are compared against. `level_table` reports it.
//! - Long bursts and clock drift: bursts hold up to `CODEWORDS` codewords and the
//!   channel presets have no clock offset, so pilot tracking over a long burst is
//!   not exercised.
//! - Error floors: the gate checks only where a rate crosses 10 %, so a change that
//!   loses a few per cent of codewords at every CNR (LLR saturation, a demapper
//!   edge case, a sync slip) still passes.
//! - `Receiver` itself: the gate repeats its decoding steps (header from preamble
//!   B, then the codewords with the header as training) rather than calling it, so
//!   a change to how `Receiver::step` trains or decodes moves no point.
//!
//! `perf/levels.toml` holds, per mode and drive, the CNR at which the HER crosses
//! 10 % and, per mode and level, the CNR at which the CER does. `level_gate` fails
//! when a point has moved by more than its margin (`GATE_MARGIN_DB`, `HEADER_MARGIN_DB` for the
//! shallower header curve): the rate must be below 10 % that far above the point and
//! above 10 % that far below. Each point decodes `GATE_BURSTS` bursts
//! of up to `CODEWORDS` codewords, or `HEADERS_PER_BURST` times as many headers.
//! Results do not depend on the number of threads. They may differ in the last bits
//! between platforms whose maths libraries round `ln`, `cos` or `powf` differently,
//! which moves points near the margin.
//!
//! `cargo test --release -p vardefm-modem level_gate -- --ignored --nocapture`
//!
//! `level_sweep` measures the points again; with `WRITE=1` it records them, and the
//! commit and command, in `perf/levels.toml`. Commit the code first, so the recorded
//! commit is the one measured.
//!
//! `WRITE=1 cargo test --release -p vardefm-modem level_sweep -- --ignored --nocapture`
//!
//! Environment: `MODES` (narrow,wide), `LEVELS` (0,1,..,6; `h` is the header at
//! every drive), `TRIALS` (bursts per point: 100 for the gate, 200 for the sweep's
//! fine grid), `WRITE` (sweep only).

use super::*;
use crate::detect_eval::{
    burst, codeword_errors, env_flag, env_list, env_modes, env_or, header_at, modem, on_every_core, payload_seed,
    through_channel, true_position,
};
use vardefm_dsp::prng::Prng;

/// Error rate at which a point is recorded and gated.
const TARGET_RATE: f32 = 0.1;
/// The gate measures this far either side of a level's recorded point, dB.
const GATE_MARGIN_DB: f32 = 0.25;
/// ... and this far for the header. Its curve falls only by about a factor of 1.5
/// per 0.25 dB around the point, so at `GATE_MARGIN_DB` the gate's rate above the
/// point would sit about 3 to 5 sigma (0.75 %) below 10 %, too near for a sample
/// that is not a replay. At 0.35 dB, on the recorded grids, it sits about 4.5 to 6
/// sigma below (least at wide mode's level 0 drive), and the rate below the point
/// 8 sigma or more above.
const HEADER_MARGIN_DB: f32 = 0.35;
/// Bursts per point of the gate. With 4 codewords per burst (2 at narrow level 0),
/// the binomial sigma at 10 % CER is 1.5 % (2.1 %). That takes codewords as
/// independent, which they are not quite: the interleaver spreads each codeword
/// over the whole burst, so a click hits all of them, and they share one channel
/// estimate. If a burst's codewords failed together, sigma would be twice that
/// (1.4 times at narrow level 0), and the level point with the least room, wide
/// level 6 at +0.25 dB, would sit about 2 sigma below 10 % rather than 4.5. The
/// sweep's warning uses the independent sigma too.
const GATE_BURSTS: u32 = 100;
/// The gate's first trial. Its bursts and noise are independent of the recorded
/// grid's (trials from 0), so the gate is a fresh sample rather than a replay of
/// the recording, and the sigmas above apply.
const GATE_FIRST_TRIAL: u32 = 1 << 20;
/// Bursts per point of the sweep's fine grid.
const RECORD_BURSTS: u32 = 200;
/// Bursts per point of the sweep's coarse search for the crossing.
const COARSE_BURSTS: u32 = 25;
/// Half-width of the sweep's fine grid in 0.25 dB steps: ±1 dB.
const FINE_STEPS: i32 = 4;
/// Times the sweep may move its fine grid before it gives up.
const MAX_MOVES: u32 = 4;
/// The sweep's search gives up above this CNR, dB: a level that still fails there
/// has an error floor.
const MAX_CNR_DB: i32 = 40;
/// ... and below this one, where nothing decodes.
const MIN_CNR_DB: i32 = -10;
/// The modes and targets a recorded baseline covers.
const ALL_MODES: &str = "narrow,wide";
const ALL_TARGETS: &str = "h,0,1,2,3,4,5,6";
/// Codewords per burst, or fewer where a level's burst holds fewer (2 at narrow
/// level 0).
const CODEWORDS: usize = 4;
/// Headers measured for each burst of a level. The header's shallow curve needs
/// more decisions than a level's: 1600 per gate point put sigma at 0.75 %. Header
/// trials send bursts of one codeword, so they are cheap.
const HEADERS_PER_BURST: u32 = 16;

/// Seed of the header fields that header trials draw, mixed with the trial.
const HEADER_SEED: u32 = 0x4ead_0001;

/// The recorded points, `perf/levels.toml`.
const BASELINE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/perf/levels.toml");

#[derive(serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct Baseline {
    source: Source,
    /// One header point per mode and drive.
    header: Vec<Point>,
    /// One codeword point per mode and level.
    level: Vec<Point>,
}

/// How a baseline was made, so a change to it can be reproduced.
#[derive(serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct Source {
    what: String,
    commit: String,
    command: String,
    /// Bursts per point of the 1 dB search.
    coarse_bursts: u32,
    /// Bursts per point of the recorded grid, `TRIALS`.
    bursts: u32,
    /// `CODEWORDS`: codewords per burst at most.
    codewords: usize,
    /// `HEADERS_PER_BURST`.
    headers_per_burst: u32,
}

#[derive(serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct Point {
    mode: String,
    /// The level; for the header, the lowest level sent at its drive.
    level: u8,
    name: String,
    /// The level's drive, `Modem::tx_rms`.
    drive: f32,
    /// CNR at which the error rate crosses `TARGET_RATE`, dB.
    cnr_db: f32,
    /// The fine grid the crossing comes from, for review in diffs.
    grid_db: Vec<f32>,
    rate: Vec<f32>,
}

/// An item of `LEVELS`: the header at every drive, or the codewords of a level.
#[derive(Clone, Copy, PartialEq, Debug)]
enum Spec {
    Header,
    Level(u8),
}

impl std::str::FromStr for Spec {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        if s == "h" {
            return Ok(Spec::Header);
        }
        let last = LEVELS.len() - 1;
        match s.parse() {
            Ok(l) if (l as usize) <= last => Ok(Spec::Level(l)),
            _ => Err(format!("levels are 0 to {last}, or h for the header")),
        }
    }
}

/// The levels that are the lowest sent at their drive: the header is measured once
/// per drive, on bursts of that level.
fn drive_levels(m: &Modem) -> Vec<u8> {
    let levels = 0..LEVELS.len() as u8;
    levels.filter(|&l| (0..l).all(|j| m.tx_rms(j) != m.tx_rms(l))).collect()
}

/// What a point measures: the header at the drive of a level, or the codewords of
/// a level.
#[derive(Clone, Copy, PartialEq, Debug)]
enum Target {
    Header(u8),
    Level(u8),
}

impl Target {
    /// The points `specs` stand for in mode `m`, in order.
    fn all(m: &Modem, specs: &[Spec]) -> Vec<Target> {
        let one = |s: &Spec| match *s {
            Spec::Header => drive_levels(m).into_iter().map(Target::Header).collect(),
            Spec::Level(l) => vec![Target::Level(l)],
        };
        specs.iter().flat_map(one).collect()
    }

    /// The level of the bursts sent.
    fn level(self) -> u8 {
        match self {
            Target::Header(l) | Target::Level(l) => l,
        }
    }

    fn name(self) -> &'static str {
        match self {
            Target::Header(_) => "header",
            Target::Level(l) => LEVELS[l as usize].name,
        }
    }

    /// How far either side of the recorded point the gate measures, dB.
    fn margin(self) -> f32 {
        match self {
            Target::Header(_) => HEADER_MARGIN_DB,
            Target::Level(_) => GATE_MARGIN_DB,
        }
    }

    /// `HER` or `CER`, for output.
    fn rate_name(self) -> &'static str {
        match self {
            Target::Header(_) => "HER",
            Target::Level(_) => "CER",
        }
    }

    fn label(self, mode: &str) -> String {
        match self {
            Target::Header(l) => format!("{mode} header at L{l} drive"),
            Target::Level(l) => format!("{mode} L{l} {}", self.name()),
        }
    }
}

fn load() -> Baseline {
    let text = std::fs::read_to_string(BASELINE).unwrap_or_else(|e| panic!("{BASELINE}: {e}"));
    toml::from_str(&text).unwrap_or_else(|e| panic!("{BASELINE}: {e}"))
}

impl Baseline {
    fn point(&self, mode: &str, target: Target) -> &Point {
        let list = match target {
            Target::Header(_) => &self.header,
            Target::Level(_) => &self.level,
        };
        let p = list.iter().find(|p| p.mode == mode && p.level == target.level());
        p.unwrap_or_else(|| panic!("{BASELINE} has no {}", target.label(mode)))
    }
}

/// One mode and target under test, with what all its trials share.
struct Case<'a> {
    m: &'a Modem,
    mode: &'a str,
    target: Target,
    /// The level of the bursts sent.
    level: u8,
    /// Codewords per burst: `CODEWORDS`, or fewer where the level's burst holds
    /// fewer; 1 for the header, whose bursts are kept short.
    ncw: usize,
    /// Where preamble B lands, the same in every trial.
    truth: usize,
}

impl<'a> Case<'a> {
    fn new(m: &'a Modem, mode: &'a str, target: Target) -> Self {
        let level = target.level();
        let ncw = match target {
            Target::Header(_) => 1,
            Target::Level(l) => m.max_ncw(l).min(CODEWORDS),
        };
        let truth = true_position(m, mode, level, ncw);
        Case { m, mode, target, level, ncw, truth }
    }

    /// Failed decisions of trial `t` at `cnr` dB, from the true position: whether
    /// the header fails, or how many codewords do.
    fn errors(&self, cnr: f32, t: u32) -> u32 {
        let m = self.m;
        let sent = trial_burst(self.level, self.ncw, t);
        let rx = through_channel(m, self.mode, cnr, &sent, t);
        if let Target::Header(_) = self.target {
            return u32::from(header_at(m, &rx, self.truth) != Some(sent.header));
        }
        codeword_errors(m, &rx, self.truth, &sent)
    }

    /// Trials and decisions for `bursts` bursts' worth: one per codeword of a level's
    /// burst, `HEADERS_PER_BURST` headers for each burst of the header.
    fn trials(&self, bursts: u32) -> (u32, usize) {
        match self.target {
            Target::Header(_) => (bursts * HEADERS_PER_BURST, (bursts * HEADERS_PER_BURST) as usize),
            Target::Level(_) => (bursts, bursts as usize * self.ncw),
        }
    }

    /// Error rate over `bursts` bursts' worth of trials at `cnr` dB, starting at
    /// trial `first`, on every core.
    fn rate(&self, cnr: f32, first: u32, bursts: u32) -> f32 {
        let (trials, decisions) = self.trials(bursts);
        let errors: u32 = on_every_core(trials, |t| self.errors(cnr, first + t)).iter().sum();
        errors as f32 / decisions as f32
    }
}

/// The burst that trial `t` sends: `ncw` codewords of payload at `level`, under
/// [`trial_header`].
fn trial_burst(level: u8, ncw: usize, t: u32) -> Burst {
    Burst { header: trial_header(level, ncw, t), ..burst(level, ncw, payload_seed(t)) }
}

/// The header that trial `t` sends: a data burst of `ncw` codewords at `level`,
/// with the fields that change from burst to burst in a session drawn at random.
/// Clipping depends on the header codeword, so a fixed header would measure one
/// codeword rather than the drive (see the module doc); it also sets the codewords'
/// channel training.
fn trial_header(level: u8, ncw: usize, t: u32) -> Header {
    // xorshift is linear: seeds that differ only in their low bits give first
    // outputs that differ only in theirs. The multiply spreads the trial over all
    // 32 bits.
    let mut rng = Prng::new(HEADER_SEED ^ t.wrapping_mul(0x9e37_79b9));
    let mut field = |bits: u32| rng.next_u32() >> (32 - bits);
    Header {
        kind: Kind::Data,
        session: field(8) as u8,
        level,
        ncw: ncw as u8,
        req_level: (field(16) % LEVELS.len() as u32) as u8,
        burst_id: field(3) as u8,
        ack_burst: field(3) as u8,
        ack_offset: field(24),
        bitmap: field(24),
        flags: field(1) as u8 * FLAG_POLL,
    }
}

/// Where `rates` (falling with `grid`) cross `target`, by interpolation of log10 of
/// the rate between the bracketing points, or linearly when one of them is 0.
/// `None` when the grid does not bracket the target. A waterfall bends down in log
/// scale, so the chord lies below it and the crossing comes out slightly low; on
/// the recorded grids the bias is well inside the gate's margin.
fn crossing(grid: &[f32], rates: &[f32], target: f32) -> Option<f32> {
    let i = rates.windows(2).position(|w| w[0] >= target && w[1] < target)?;
    let (x0, x1, y0, y1) = (grid[i], grid[i + 1], rates[i], rates[i + 1]);
    let t =
        if y1 > 0.0 { (y0.log10() - target.log10()) / (y0.log10() - y1.log10()) } else { (y0 - target) / (y0 - y1) };
    Some(x0 + t * (x1 - x0))
}

/// What [`fine_grid`] returns: the grid, its rates and the crossing; or the last
/// grid and its rates when no grid carries the gate.
type FineGrid = Result<(Vec<f32>, Vec<f32>, f32), (Vec<f32>, Vec<f32>)>;

/// The sweep's fine grid: whole steps of `1 / per_db` dB, `half` steps either side
/// of `centre` (in steps), with `rate` measured at each point. The grid is moved to
/// its own crossing of `target`, or by `half` steps when it does not cross, until
/// the crossing lies at least `margin` dB inside both ends, at most `MAX_MOVES`
/// times. Returns the grid, its rates and the crossing rounded to 0.01 dB; or, when
/// no grid carries the gate, the last grid and its rates.
fn fine_grid(
    mut centre: i32,
    per_db: f32,
    half: i32,
    target: f32,
    margin: f32,
    mut rate: impl FnMut(f32) -> f32,
) -> FineGrid {
    let mut moves = 0;
    loop {
        let grid: Vec<f32> = (centre - half..=centre + half).map(|i| i as f32 / per_db).collect();
        let rates: Vec<f32> = grid.iter().map(|&g| rate(g)).collect();
        let next = match crossing(&grid, &rates, target).map(|x| (x * 100.0).round() / 100.0) {
            Some(x) if grid[0] <= x - margin && x + margin <= grid[grid.len() - 1] => return Ok((grid, rates, x)),
            Some(x) => (x * per_db).round() as i32,
            None if rates[0] < target => centre - half,
            None => centre + half,
        };
        if moves == MAX_MOVES || next == centre {
            return Err((grid, rates));
        }
        println!("  fine grid moved: {:.2} -> {:.2} dB", centre as f32 / per_db, next as f32 / per_db);
        centre = next;
        moves += 1;
    }
}

/// The rate at `x` on a grid, interpolated in log10 between the neighbouring points
/// as in [`crossing`], or linearly next to a 0; clamped to the ends of the grid.
fn rate_at(grid: &[f32], rates: &[f32], x: f32) -> f32 {
    let Some(i) = grid.windows(2).position(|w| x <= w[1]) else {
        return rates[rates.len() - 1];
    };
    let (x0, x1, y0, y1) = (grid[i], grid[i + 1], rates[i], rates[i + 1]);
    let t = ((x - x0) / (x1 - x0)).clamp(0.0, 1.0);
    if y0 > 0.0 && y1 > 0.0 {
        10f32.powf(y0.log10() + t * (y1.log10() - y0.log10()))
    } else {
        y0 + t * (y1 - y0)
    }
}

/// The commit of the working tree, marked when tracked files have uncommitted
/// changes other than recorded baselines; untracked files do not count. "unknown"
/// without git or outside a repository.
fn commit() -> String {
    let git = |args: &[&str]| {
        std::process::Command::new("git")
            .args(args)
            .current_dir(env!("CARGO_MANIFEST_DIR"))
            .output()
            .ok()
            .filter(|o| o.status.success())
            .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
    };
    let Some(head) = git(&["rev-parse", "--short", "HEAD"]) else {
        return "unknown".into();
    };
    if git(&["status", "--porcelain", "--untracked-files=no", "--", ":/", ":(top,exclude,glob)**/perf/*.toml"])
        .is_some_and(|s| s.is_empty())
    {
        head
    } else {
        format!("{head}-dirty")
    }
}

/// Measures the 10 % point of `case`: a 1 dB search for the crossing from `start`,
/// then a 0.25 dB grid of ±1 dB around it, moved until it crosses 10 % with the
/// gate's two points on it. Returns the point and the whole dB the
/// search ended at. Warns when the curve is too flat for the gate to tell a shift of
/// the target's margin from noise.
fn sweep(case: &Case, start: i32, bursts: u32) -> (Point, i32) {
    let label = case.target.label(case.mode);
    let rate_name = case.target.rate_name();
    let mut seen = std::collections::BTreeMap::new();
    let mut at = |c: i32| *seen.entry(c).or_insert_with(|| case.rate(c as f32, 0, COARSE_BURSTS));
    // c ends as the last whole dB at which the rate is still 10 % or more.
    let mut c = start;
    while at(c) < TARGET_RATE {
        assert!(c > MIN_CNR_DB, "{label}: {rate_name} is below 10 % even at {c} dB");
        c -= 1;
    }
    while at(c + 1) >= TARGET_RATE {
        c += 1;
        assert!(c < MAX_CNR_DB, "{label}: {rate_name} is still 10 % or more at {MAX_CNR_DB} dB");
    }
    // Whole quarter dB steps, so the recorded grid reads as written. The search rests
    // on few bursts, so its crossing can be off by more than the grid allows, and
    // `fine_grid` moves the grid until it carries the gate. A grid that cannot stops
    // the run here, not after the whole sweep.
    let margin = case.target.margin();
    let measured = fine_grid(c * 4 + 2, 4.0, FINE_STEPS, TARGET_RATE, margin, |g| case.rate(g, 0, bursts));
    let (grid, rates, cnr_db) = measured.unwrap_or_else(|(grid, rates)| {
        panic!("{label}: the fine grid {grid:?} does not carry the gate: {rate_name} {rates:?}")
    });

    let line: String = grid.iter().zip(&rates).map(|(g, r)| format!(" {g:.2}:{r:.3}")).collect();
    println!("{label}: 10 % {rate_name} at {cnr_db:.2} dB;{line}");
    // The rate the gate would see on either side, interpolated on the grid, against
    // the binomial sigma, which is somewhat optimistic (see `GATE_BURSTS`).
    let decisions = case.trials(GATE_BURSTS).1;
    let sigma = (TARGET_RATE * (1.0 - TARGET_RATE) / decisions as f32).sqrt();
    for d in [-margin, margin] {
        let r = rate_at(&grid, &rates, cnr_db + d);
        if (r - TARGET_RATE).abs() < 3.0 * sigma {
            println!("  WARNING: {rate_name} {r:.3} at {:.2} dB is within 3 sigma of 10 %", cnr_db + d);
        }
    }
    let p = Point {
        mode: case.mode.into(),
        level: case.level,
        name: case.target.name().into(),
        drive: case.m.tx_rms(case.level),
        cnr_db,
        grid_db: grid,
        rate: rates,
    };
    (p, c)
}

/// Measures the header point of every mode and drive and the codeword point of
/// every mode and level; with `WRITE=1`, records them in `perf/levels.toml`.
#[test]
#[ignore]
fn level_sweep() {
    let modes = env_modes(ALL_MODES);
    let specs: Vec<Spec> = env_list("LEVELS", ALL_TARGETS);
    let bursts = env_or("TRIALS", RECORD_BURSTS);
    assert!(bursts > 0, "TRIALS must be at least 1");
    let write = env_flag("WRITE");
    // The tree as measured, not as it is when the run ends.
    let commit = commit();
    // The file holds every point from one run; a partial run would drop the rest.
    assert!(
        !write || (std::env::var("MODES").is_err() && std::env::var("LEVELS").is_err()),
        "WRITE=1 records every mode and level: unset MODES and LEVELS"
    );
    let (mut header, mut level) = (Vec::new(), Vec::new());
    for mode in &modes {
        let m = modem(mode);
        // Crossings mostly rise with the level; start each search at the last of its
        // kind.
        let (mut header_start, mut level_start) = (0i32, 0i32);
        for target in Target::all(&m, &specs) {
            let case = Case::new(&m, mode, target);
            let (list, start) = match target {
                Target::Header(_) => (&mut header, &mut header_start),
                Target::Level(_) => (&mut level, &mut level_start),
            };
            let (p, c) = sweep(&case, *start, bursts);
            list.push(p);
            *start = c;
        }
    }
    if write {
        // A grid that cannot carry the gate stops the run instead of reaching the file.
        for (points, margin) in [(&header, HEADER_MARGIN_DB), (&level, GATE_MARGIN_DB)] {
            for p in points {
                let at = format!("{} L{} {}", p.mode, p.level, p.name);
                check_point(&at, &p.grid_db, &p.rate, p.cnr_db, TARGET_RATE, margin);
            }
        }
        let b = Baseline {
            source: Source {
                what: "CNR at 10 % header and codeword error rate, decoded from the known burst position".into(),
                commit,
                command: format!(
                    "WRITE=1 TRIALS={bursts} cargo test --release -p vardefm-modem level_sweep -- --ignored"
                ),
                coarse_bursts: COARSE_BURSTS,
                bursts,
                codewords: CODEWORDS,
                headers_per_burst: HEADERS_PER_BURST,
            },
            header,
            level,
        };
        std::fs::write(BASELINE, toml::to_string(&b).unwrap()).unwrap();
        println!("wrote {BASELINE}");
    }
}

/// Fails when a header or level point has moved from `perf/levels.toml` by more
/// than its margin.
#[test]
#[ignore]
fn level_gate() {
    let modes = env_modes(ALL_MODES);
    let specs: Vec<Spec> = env_list("LEVELS", ALL_TARGETS);
    let bursts = env_or("TRIALS", GATE_BURSTS);
    assert!(bursts > 0, "TRIALS must be at least 1");
    let base = load();
    println!("baseline {} ({})", base.source.commit, base.source.command);
    let mut failed = Vec::new();
    for mode in &modes {
        let m = modem(mode);
        for target in Target::all(&m, &specs) {
            let p = base.point(mode, target);
            let case = Case::new(&m, mode, target);
            let margin = target.margin();
            let (below, above, ok) = gate(p.cnr_db, margin, TARGET_RATE, |c| case.rate(c, GATE_FIRST_TRIAL, bursts));
            println!(
                "{} at {:.2} dB: {} {below:.3} at -{margin} dB, {above:.3} at +{margin} dB {}",
                target.label(mode),
                p.cnr_db,
                target.rate_name(),
                if ok { "ok" } else { "FAIL" }
            );
            if !ok {
                failed.push(target.label(mode));
            }
        }
    }
    assert!(failed.is_empty(), "10 % point moved for {failed:?}");
}

/// The gate's verdict on a point recorded at `point`: `rate` measured `margin` dB
/// below and above it, and whether it still crosses `target` between them.
fn gate(point: f32, margin: f32, target: f32, mut rate: impl FnMut(f32) -> f32) -> (f32, f32, bool) {
    let below = rate(point - margin);
    let above = rate(point + margin);
    (below, above, above < target && below > target)
}

/// The gate passes a point at the curve's crossing and fails one shifted by twice
/// the margin, on either side, for both margins.
#[test]
fn gate_fails_a_shifted_point() {
    // Crosses 10 % at 5.5 dB.
    let curve = waterfall(5.0);
    for margin in [GATE_MARGIN_DB, HEADER_MARGIN_DB] {
        let ok = |point| gate(point, margin, TARGET_RATE, &curve).2;
        assert!(ok(5.5), "margin {margin}");
        assert!(!ok(5.5 - 2.0 * margin), "margin {margin}: a point recorded too low passed");
        assert!(!ok(5.5 + 2.0 * margin), "margin {margin}: a point recorded too high passed");
    }
}

/// Every mode and target decodes nearly everything far above its point and nearly
/// nothing far below, so the measurement path itself works: trial bursts, the
/// header's training and decoding from the known position.
#[test]
fn measurement_decodes_far_above_and_fails_far_below() {
    let bursts = 2;
    for mode in ALL_MODES.split(',') {
        let m = modem(mode);
        for target in [Target::Header(0), Target::Level(0), Target::Level(LEVELS.len() as u8 - 1)] {
            let case = Case::new(&m, mode, target);
            let label = target.label(mode);
            assert_eq!(case.rate(40.0, 0, bursts), 0.0, "{label} fails at 40 dB");
            assert!(case.rate(-5.0, 0, bursts) > 0.9, "{label} decodes at -5 dB");
        }
    }
}

#[test]
fn crossing_interpolates_in_log() {
    let grid = [1.0, 1.5, 2.0];
    assert_eq!(crossing(&grid, &[0.5, 0.1, 0.01], 0.1), Some(1.5));
    let x = crossing(&grid, &[1.0, 0.1, 0.01], 0.0316).unwrap();
    assert!((x - 1.75).abs() < 1e-3, "{x}");
    assert_eq!(crossing(&grid, &[0.3, 0.2, 0.0], 0.1), Some(1.75));
    assert_eq!(crossing(&grid, &[0.05, 0.02, 0.0], 0.1), None);
    // A rate exactly at the target, as a measured one can be, is not yet below it.
    assert_eq!(crossing(&[1.0, 1.5, 2.0, 2.5], &[0.2, 0.1, 0.1, 0.05], 0.1), Some(2.0));
}

/// A waterfall falling a decade per 0.5 dB from 1 at `x0` dB: it crosses 10 % at
/// `x0 + 0.5`.
#[cfg(test)]
fn waterfall(x0: f32) -> impl Fn(f32) -> f32 {
    move |x| 10f32.powf(-2.0 * (x - x0)).min(1.0)
}

/// `fine_grid` keeps a grid that carries the gate, moves one that does not, gives up
/// on a curve that never crosses, and counts its moves.
#[test]
fn fine_grid_moves_until_it_carries_the_gate() {
    let (per_db, half, margin) = (4.0, 4, 0.5);
    let x = |r: FineGrid| r.ok().map(|(_, _, x)| x);
    // Centred on the crossing (5.5 dB): kept.
    assert_eq!(x(fine_grid(22, per_db, half, 0.1, margin, waterfall(5.0))), Some(5.5));
    // The crossing too near the grid's top end for the gate: moved onto it.
    assert_eq!(x(fine_grid(19, per_db, half, 0.1, margin, waterfall(5.0))), Some(5.5));
    // ... and too near its bottom end.
    assert_eq!(x(fine_grid(25, per_db, half, 0.1, margin, waterfall(5.0))), Some(5.5));
    // Every rate above 10 %: moved up by half the grid, four times, until it crosses.
    assert_eq!(x(fine_grid(4, per_db, half, 0.1, margin, waterfall(5.0))), Some(5.5));
    // Every rate below 10 %: moved down.
    assert_eq!(x(fine_grid(40, per_db, half, 0.1, margin, waterfall(5.0))), Some(5.5));
    // A curve that never falls below 10 % (an error floor): gives up.
    let mut calls = 0;
    let budget = (MAX_MOVES as i32 + 1) * (2 * half + 1);
    let floor = |_| {
        calls += 1;
        assert!(calls <= budget, "fine_grid measured more than MAX_MOVES moves allow");
        0.5
    };
    assert!(fine_grid(0, per_db, half, 0.1, margin, floor).is_err());
    // Too far to reach in `MAX_MOVES` moves of half the grid: gives up.
    let far = (MAX_MOVES as i32 + 2) * half;
    assert!(fine_grid(22 + far, per_db, half, 0.1, margin, waterfall(5.0)).is_err());
    // A gate margin wider than the grid can hold: gives up rather than looping.
    assert!(fine_grid(22, per_db, half, 0.1, 1.5, waterfall(5.0)).is_err());
}

#[test]
fn rate_at_interpolates_in_log_and_clamps() {
    let (grid, rates) = ([1.0, 2.0, 3.0], [0.1, 0.001, 0.0]);
    assert!((rate_at(&grid, &rates, 1.5) - 0.01).abs() < 1e-5);
    assert!((rate_at(&grid, &rates, 2.5) - 0.0005).abs() < 1e-7);
    assert_eq!(rate_at(&grid, &rates, 0.0), 0.1);
    assert_eq!(rate_at(&grid, &rates, 9.0), 0.0);
}

#[test]
fn targets_parse() {
    let s: Vec<Spec> = "h, 0,6".split(',').map(|s| s.trim().parse().unwrap()).collect();
    assert_eq!(s, [Spec::Header, Spec::Level(0), Spec::Level(6)]);
    assert!("x".parse::<Spec>().is_err());
    assert!(LEVELS.len().to_string().parse::<Spec>().is_err());
}

/// How far the header's effective level may differ, at one drive, between the
/// bursts the gate measures and any other level or codeword count, dB. Well inside
/// `HEADER_MARGIN_DB`.
const HEADER_LEVEL_TOLERANCE_DB: f32 = 0.1;
/// Header trials averaged per level and codeword count by
/// `header_level_depends_only_on_drive`.
const HEADER_LEVEL_TRIALS: u32 = 32;
/// SNR at which clipping distortion is weighed against channel noise, dB. The
/// receiver's estimate on the header at the recorded header points is about -2 dB
/// (median) and at most about 1 dB (90th percentile); a higher value weighs the
/// distortion more.
const HEADER_REFERENCE_SNR_DB: f32 = 3.0;

/// What the receiver gets of the header's symbol bodies (the cyclic prefix is
/// dropped) over `trials` header trials of a `level` burst of `ncw` codewords, the
/// same bursts the gate sends: the useful level against the drive, dB, and the
/// signal-to-distortion ratio of the clipping, dB. Noiseless.
fn header_level(m: &Modem, level: u8, ncw: usize, trials: u32) -> (f32, f32) {
    let sl = m.ofdm.p.symbol_len();
    let start = (m.leader_symbols() + 2) * sl;
    let end = start + m.header_symbols() * sl;
    let bodies =
        |x: &[f32]| -> Vec<f32> { x[start..end].chunks(sl).flat_map(|s| s[m.ofdm.p.ncp..].to_vec()).collect() };
    // Linear reference: the same symbols scaled to the drive without clipping.
    let tiny = 1e-3;
    let (mut cl, mut ll, mut cc, mut n) = (0f64, 0f64, 0f64, 0usize);
    for t in 0..trials {
        let sent = trial_burst(level, ncw, t);
        let linear = bodies(&m.ofdm.modulate_with_leader_rms(&m.symbols(&sent), m.txdelay, tiny));
        let clipped = bodies(&m.modulate(&sent));
        let k = (m.tx_rms(level) / tiny) as f64;
        for (&c, &l) in clipped.iter().zip(&linear) {
            let (c, l) = (c as f64, l as f64 * k);
            (cl, ll, cc) = (cl + c * l, ll + l * l, cc + c * c);
        }
        n += linear.len();
    }
    // The clipped header is a times the linear one plus distortion.
    let a = cl / ll;
    let useful = a * a * ll;
    let distortion = cc - 2.0 * a * cl + useful;
    let level_db = 10.0 * (useful / n as f64).log10() - 20.0 * (m.tx_rms(level) as f64).log10();
    (level_db as f32, (10.0 * (useful / distortion).log10()) as f32)
}

/// The gate's header points, measured on one-codeword bursts of the lowest level at
/// each drive, stand for every level and codeword count at that drive: the header's
/// effective level differs by at most `HEADER_LEVEL_TOLERANCE_DB`. `modulate`
/// scales the whole burst to the drive, so the burst's length and payload could
/// move the header; and it clips, which depends on the header codeword, so the
/// gate draws the header's fields at random (`trial_header`) and this check
/// averages over the same draws. Effective level: useful level less the noise the
/// clipping adds at `HEADER_REFERENCE_SNR_DB`.
#[test]
fn header_level_depends_only_on_drive() {
    let effective = |(level_db, sdr_db): (f32, f32)| {
        let snr = 10f32.powf(HEADER_REFERENCE_SNR_DB / 10.0);
        level_db - 10.0 * (1.0 + snr / 10f32.powf(sdr_db / 10.0)).log10()
    };
    for mode in ALL_MODES.split(',') {
        let m = modem(mode);
        let drives = drive_levels(&m);
        let mut report = Vec::new();
        let mut worst = 0f32;
        // The gate's burst at each drive: one codeword at the lowest level there.
        let gate: Vec<(u8, (f32, f32))> =
            drives.iter().map(|&d| (d, header_level(&m, d, 1, HEADER_LEVEL_TRIALS))).collect();
        for level in 0..LEVELS.len() as u8 {
            let &(measured, at_gate) = gate.iter().find(|(d, _)| m.tx_rms(*d) == m.tx_rms(level)).unwrap();
            let reference = effective(at_gate);
            for ncw in [1, m.max_ncw(level)] {
                let (level_db, sdr_db) = if (level, ncw) == (measured, 1) {
                    at_gate
                } else {
                    header_level(&m, level, ncw, HEADER_LEVEL_TRIALS)
                };
                let d = effective((level_db, sdr_db)) - reference;
                report.push(format!(
                    "L{level} ncw {ncw}: level {level_db:+.3} dB, SDR {sdr_db:.1} dB, {d:+.3} dB against L{measured} ncw 1"
                ));
                worst = worst.max(d.abs());
            }
        }
        println!("{mode}: the header's effective level differs by up to {worst:.3} dB at one drive");
        assert!(
            worst <= HEADER_LEVEL_TOLERANCE_DB,
            "{mode}: the header's effective level differs by up to {worst:.3} dB at one drive:\n  {}",
            report.join("\n  ")
        );
    }
}

/// The header is measured at every drive a mode sends, once each.
#[test]
fn header_at_every_drive() {
    for mode in ALL_MODES.split(',') {
        let m = modem(mode);
        let levels = drive_levels(&m);
        assert_eq!(levels[0], 0);
        // Every distinct drive, in order of the first level sent at it.
        let mut drives: Vec<f32> = Vec::new();
        for d in (0..LEVELS.len() as u8).map(|l| m.tx_rms(l)) {
            if !drives.contains(&d) {
                drives.push(d);
            }
        }
        assert_eq!(levels.iter().map(|&l| m.tx_rms(l)).collect::<Vec<_>>(), drives, "{mode}");
    }
}

/// The committed baseline holds one consistent point per mode for the header and
/// per mode and level for the codewords, recorded with the settings the gate
/// measures with.
#[test]
fn baseline_covers_every_mode_and_level() {
    let base = load();
    let c = &base.source.commit;
    assert!(c != "unknown" && !c.ends_with("-dirty"), "{BASELINE} names no clean commit ({c:?}): record it again");
    let recorded = (base.source.codewords, base.source.headers_per_burst);
    assert_eq!(recorded, (CODEWORDS, HEADERS_PER_BURST), "{BASELINE} was recorded with other bursts: record it again");
    let specs: Vec<Spec> = ALL_TARGETS.split(',').map(|s| s.parse().unwrap()).collect();
    assert_eq!(specs.len(), LEVELS.len() + 1, "ALL_TARGETS does not list every level");
    let (mut headers, mut levels) = (0, 0);
    for mode in ALL_MODES.split(',') {
        let m = modem(mode);
        for target in Target::all(&m, &specs) {
            match target {
                Target::Header(_) => headers += 1,
                Target::Level(_) => levels += 1,
            }
            let p = base.point(mode, target);
            let at = format!("{BASELINE}: {}", target.label(mode));
            assert_eq!(p.name, target.name(), "{at} is named {:?}", p.name);
            // A changed drive table moves the points: record them again.
            assert_eq!(p.drive, m.tx_rms(target.level()), "{at} was recorded at another drive");
            check_point(&at, &p.grid_db, &p.rate, p.cnr_db, TARGET_RATE, target.margin());
        }
    }
    assert_eq!(base.header.len(), headers, "{BASELINE}: header entries for unknown modes or drives");
    assert_eq!(base.level.len(), levels, "{BASELINE}: entries for unknown modes or levels");
}

/// A recorded point is the crossing of its own grid, and the gate's two points lie
/// on that grid.
fn check_point(at: &str, grid: &[f32], rates: &[f32], point: f32, target: f32, margin: f32) {
    assert_eq!(grid.len(), rates.len(), "{at}: grid and rates differ in length");
    assert!(grid.windows(2).all(|w| w[0] < w[1]), "{at}: grid is not increasing");
    let x = crossing(grid, rates, target).unwrap_or_else(|| panic!("{at}: the grid does not cross the target"));
    assert!(((x * 100.0).round() / 100.0 - point).abs() < 1e-3, "{at}: recorded {point}, the grid crosses at {x}");
    assert!(grid[0] <= point - margin && point + margin <= grid[grid.len() - 1], "{at}: gate points off the grid");
}
