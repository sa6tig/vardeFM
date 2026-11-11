//! Decoding performance per mode and speed level, against a recorded baseline.
//!
//! Two error rates through the FM channel simulator, both from the known burst
//! position, so detection is left out (the detector evaluation, `detect_eval`,
//! covers it):
//! - per level, the codeword error rate (CER), with the sent header as channel
//!   training, as `Receiver::step` decodes the payload once the header has decoded;
//! - per mode, the header error rate (HER): the header decoded from preamble B
//!   alone, as `Receiver::step` does first. The header is coded and modulated the
//!   same at every level, so one point per mode covers it.
//!
//! A burst decodes only if both its header and its codewords do. The header's point
//! lies above level 0's, so there the header limits a burst, and its curve is far
//! shallower than any level's: at level 1's point a few per cent of headers still
//! fail.
//!
//! What the gate does not cover:
//! - The receiver's SNR estimate, which the link's rate thresholds
//!   (`SNR_THRESHOLDS_DB`) are compared against. `level_table` reports it.
//! - Long bursts and clock drift: bursts hold up to `CODEWORDS` codewords and the
//!   channel presets have no clock offset, so pilot tracking over a long burst is
//!   not exercised.
//!
//! `perf/levels.toml` holds, per mode, the CNR at which the HER crosses 10 % and, per
//! mode and level, the CNR at which the CER does. `level_gate` fails when a point has
//! moved by more than its margin (`GATE_MARGIN_DB`, `HEADER_MARGIN_DB` for the
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
//! Environment: `MODES` (narrow,wide), `LEVELS` (0,1,..,6; `h` is the header),
//! `TRIALS` (bursts per point: 100 for the gate, 200 for the sweep's fine grid),
//! `WRITE` (sweep only).

use super::*;
use crate::detect_eval::{
    burst, env_list, env_or, header_at, modem, on_every_core, payload_seed, received, true_position,
};

/// Error rate at which a point is recorded and gated.
const TARGET_RATE: f32 = 0.1;
/// The gate measures this far either side of a level's recorded point, dB.
const GATE_MARGIN_DB: f32 = 0.25;
/// ... and this far for the header. Its curve falls only by about a factor of 1.5
/// per 0.25 dB around the point, so at `GATE_MARGIN_DB` the gate's two rates would sit
/// within 3 sigma of 10 %.
const HEADER_MARGIN_DB: f32 = 0.5;
/// Bursts per point of the gate. With 4 codewords per burst (2 at narrow level 0),
/// the binomial sigma at 10 % CER is 1.5 % (2.1 %). That takes codewords as
/// independent, which they are not quite: the interleaver spreads each codeword
/// over the whole burst, so a click hits all of them, and they share one channel
/// estimate. The real spread is somewhat larger; the recorded curves clear the
/// margin by far more than that.
const GATE_BURSTS: u32 = 100;
/// Bursts per point of the sweep's fine grid.
const RECORD_BURSTS: u32 = 200;
/// Bursts per point of the sweep's coarse search for the crossing.
const COARSE_BURSTS: u32 = 25;
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
/// trials send the shortest burst, so they are cheap.
const HEADERS_PER_BURST: u32 = 16;

/// The recorded points, `perf/levels.toml`.
const BASELINE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/perf/levels.toml");

#[derive(serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct Baseline {
    source: Source,
    /// One header point per mode.
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
    /// The level; absent for the header.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    level: Option<u8>,
    name: String,
    /// CNR at which the error rate crosses `TARGET_RATE`, dB.
    cnr_db: f32,
    /// The fine grid the crossing comes from, for review in diffs.
    grid_db: Vec<f32>,
    rate: Vec<f32>,
}

/// What a point measures: the header, or the codewords of a level.
#[derive(Clone, Copy, PartialEq, Debug)]
enum Target {
    Header,
    Level(u8),
}

impl std::str::FromStr for Target {
    type Err = std::num::ParseIntError;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        if s == "h" {
            Ok(Target::Header)
        } else {
            s.parse().map(Target::Level)
        }
    }
}

impl Target {
    fn name(self) -> &'static str {
        match self {
            Target::Header => "header",
            Target::Level(l) => LEVELS[l as usize].name,
        }
    }

    /// How far either side of the recorded point the gate measures, dB.
    fn margin(self) -> f32 {
        match self {
            Target::Header => HEADER_MARGIN_DB,
            Target::Level(_) => GATE_MARGIN_DB,
        }
    }

    /// `HER` or `CER`, for output.
    fn rate_name(self) -> &'static str {
        match self {
            Target::Header => "HER",
            Target::Level(_) => "CER",
        }
    }

    fn label(self, mode: &str) -> String {
        match self {
            Target::Header => format!("{mode} header"),
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
        let (list, level) = match target {
            Target::Header => (&self.header, None),
            Target::Level(l) => (&self.level, Some(l)),
        };
        let p = list.iter().find(|p| p.mode == mode && p.level == level);
        p.unwrap_or_else(|| panic!("{BASELINE} has no {}", target.label(mode)))
    }
}

/// One mode and target under test, with what all its trials share.
struct Case<'a> {
    m: &'a Modem,
    mode: &'a str,
    target: Target,
    /// The level of the bursts sent: the target's, or 0 for the header.
    level: u8,
    /// Codewords per burst: `CODEWORDS`, or fewer where the level's burst holds
    /// fewer; 1 for the header, whose bursts are as short as they come.
    ncw: usize,
    /// Where preamble B lands, the same in every trial.
    truth: usize,
    /// The header's carriers, the channel training; every trial sends the same header.
    training: Vec<Vec<Complex32>>,
}

impl<'a> Case<'a> {
    fn new(m: &'a Modem, mode: &'a str, target: Target) -> Self {
        let (level, ncw) = match target {
            Target::Header => (0, 1),
            Target::Level(l) => (l, m.max_ncw(l).min(CODEWORDS)),
        };
        let truth = true_position(m, mode, level, ncw);
        let training = m.header_carriers(&burst(level, ncw, payload_seed(0)).header);
        Case { m, mode, target, level, ncw, truth, training }
    }

    /// Failed decisions of trial `t` at `cnr` dB, from the true position: whether
    /// the header fails, or how many codewords do.
    fn errors(&self, cnr: f32, t: u32) -> u32 {
        let (m, ncw) = (self.m, self.ncw);
        let (sent, rx) = received(m, self.mode, cnr, self.level, ncw, t);
        if self.target == Target::Header {
            return u32::from(header_at(m, &rx, self.truth) != Some(sent.header));
        }
        let nsym = m.header_symbols() + ncw * m.symbols_per_codeword(self.level);
        let Some(frame) = m.ofdm.demodulate_known(&rx, self.truth, nsym, &self.training) else {
            return ncw as u32;
        };
        let got = m.decode_payload(&sent.header, &frame);
        got.iter().zip(&sent.payload).filter(|(g, w)| g.as_ref() != Some(*w)).count() as u32
    }

    /// Trials and decisions for `bursts` bursts' worth: one per codeword of a level's
    /// burst, `HEADERS_PER_BURST` headers for each burst of the header.
    fn trials(&self, bursts: u32) -> (u32, usize) {
        match self.target {
            Target::Header => (bursts * HEADERS_PER_BURST, (bursts * HEADERS_PER_BURST) as usize),
            Target::Level(_) => (bursts, bursts as usize * self.ncw),
        }
    }

    /// Error rate over `bursts` bursts' worth of trials at `cnr` dB, on every core.
    fn rate(&self, cnr: f32, bursts: u32) -> f32 {
        let (trials, decisions) = self.trials(bursts);
        let errors: u32 = on_every_core(trials, |t| self.errors(cnr, t)).iter().sum();
        errors as f32 / decisions as f32
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

/// The commit of the working tree, marked when it has uncommitted changes other
/// than recorded baselines; "unknown" without git or outside a repository.
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
    if git(&["status", "--porcelain", "--", ":/", ":(top,exclude,glob)**/perf/*.toml"]).is_some_and(|s| s.is_empty()) {
        head
    } else {
        format!("{head}-dirty")
    }
}

/// Measures the 10 % point of `case`: a 1 dB search for the crossing from `start`,
/// then a 0.25 dB grid of ±1 dB around it. Returns the point and the whole dB the
/// search ended at. Warns when the curve is too flat for the gate to tell a shift of
/// the target's margin from noise.
fn sweep(case: &Case, start: i32, bursts: u32) -> (Point, i32) {
    let label = case.target.label(case.mode);
    let rate_name = case.target.rate_name();
    let mut seen = std::collections::BTreeMap::new();
    let mut at = |c: i32| *seen.entry(c).or_insert_with(|| case.rate(c as f32, COARSE_BURSTS));
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
    // Whole quarter dB steps, so the recorded grid reads as written.
    let centre = c * 4 + 2;
    let grid: Vec<f32> = (centre - 4..=centre + 4).map(|i| i as f32 / 4.0).collect();
    let rates: Vec<f32> = grid.iter().map(|&g| case.rate(g, bursts)).collect();
    let Some(x) = crossing(&grid, &rates, TARGET_RATE) else {
        panic!("{label}: the grid {grid:?} does not cross 10 % {rate_name}: {rates:?}");
    };
    let cnr_db = (x * 100.0).round() / 100.0;
    let line: String = grid.iter().zip(&rates).map(|(g, r)| format!(" {g:.2}:{r:.3}")).collect();
    println!("{label}: 10 % {rate_name} at {cnr_db:.2} dB;{line}");
    // The rate the gate would see on either side, interpolated on the grid, against
    // the binomial sigma, which is somewhat optimistic (see `GATE_BURSTS`).
    let decisions = case.trials(GATE_BURSTS).1;
    let sigma = (TARGET_RATE * (1.0 - TARGET_RATE) / decisions as f32).sqrt();
    let margin = case.target.margin();
    for d in [-margin, margin] {
        let r = rate_at(&grid, &rates, x + d);
        if (r - TARGET_RATE).abs() < 3.0 * sigma {
            println!("  WARNING: {rate_name} {r:.3} at {:.2} dB is within 3 sigma of 10 %", x + d);
        }
    }
    let level = match case.target {
        Target::Header => None,
        Target::Level(l) => Some(l),
    };
    let p =
        Point { mode: case.mode.into(), level, name: case.target.name().into(), cnr_db, grid_db: grid, rate: rates };
    (p, c)
}

/// Measures the header point of every mode and the codeword point of every mode and
/// level; with `WRITE=1`, records them in `perf/levels.toml`.
#[test]
#[ignore]
fn level_sweep() {
    let modes: Vec<String> = env_list("MODES", ALL_MODES);
    let targets: Vec<Target> = env_list("LEVELS", ALL_TARGETS);
    let bursts = env_or("TRIALS", RECORD_BURSTS);
    let write = std::env::var("WRITE").is_ok_and(|v| v == "1");
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
        // The level crossing mostly rises with the level; start each search at the last.
        let mut start = 0i32;
        for &target in &targets {
            let case = Case::new(&m, mode, target);
            match target {
                Target::Header => header.push(sweep(&case, 0, bursts).0),
                Target::Level(_) => {
                    let (p, c) = sweep(&case, start, bursts);
                    level.push(p);
                    start = c;
                }
            }
        }
    }
    if write {
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
    let modes: Vec<String> = env_list("MODES", ALL_MODES);
    let targets: Vec<Target> = env_list("LEVELS", ALL_TARGETS);
    let bursts = env_or("TRIALS", GATE_BURSTS);
    let base = load();
    println!("baseline {} ({})", base.source.commit, base.source.command);
    let mut failed = Vec::new();
    for mode in &modes {
        let m = modem(mode);
        for &target in &targets {
            let p = base.point(mode, target);
            let case = Case::new(&m, mode, target);
            let margin = target.margin();
            let below = case.rate(p.cnr_db - margin, bursts);
            let above = case.rate(p.cnr_db + margin, bursts);
            let ok = above < TARGET_RATE && below > TARGET_RATE;
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
    let t: Vec<Target> = "h, 0,6".split(',').map(|s| s.trim().parse().unwrap()).collect();
    assert_eq!(t, [Target::Header, Target::Level(0), Target::Level(6)]);
    assert!("x".parse::<Target>().is_err());
}

/// The committed baseline holds one consistent point per mode for the header and
/// per mode and level for the codewords, recorded with the settings the gate
/// measures with.
#[test]
fn baseline_covers_every_mode_and_level() {
    let base = load();
    let recorded = (base.source.codewords, base.source.headers_per_burst);
    assert_eq!(recorded, (CODEWORDS, HEADERS_PER_BURST), "{BASELINE} was recorded with other bursts: record it again");
    let modes: Vec<&str> = ALL_MODES.split(',').collect();
    assert_eq!(base.header.len(), modes.len(), "{BASELINE}: header entries for unknown modes");
    assert_eq!(base.level.len(), modes.len() * LEVELS.len(), "{BASELINE}: entries for unknown modes or levels");
    let targets: Vec<Target> = ALL_TARGETS.split(',').map(|s| s.parse().unwrap()).collect();
    assert_eq!(targets.len(), LEVELS.len() + 1, "ALL_TARGETS does not list every level");
    for mode in modes {
        for &target in &targets {
            let p = base.point(mode, target);
            let at = format!("{BASELINE}: {}", target.label(mode));
            assert_eq!(p.name, target.name(), "{at} is named {:?}", p.name);
            check_point(&at, &p.grid_db, &p.rate, p.cnr_db, TARGET_RATE, target.margin());
        }
    }
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
