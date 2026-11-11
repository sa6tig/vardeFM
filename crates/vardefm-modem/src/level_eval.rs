//! Decoding performance per mode and speed level, against a recorded baseline.
//!
//! The measure is the codeword error rate (CER) through the FM channel simulator,
//! decoded from the known burst position with the sent header as channel training,
//! as `Receiver::step` does once the header has decoded. Detection is left out: the
//! detector evaluation (`detect_eval`) covers it, and at low levels it would hide
//! the decoder behind the detector.
//!
//! `perf/levels.toml` holds, per mode and level, the CNR at which CER crosses 10 %.
//! `level_gate` fails when that point has moved by more than `GATE_MARGIN_DB`: CER
//! must be below 10 % that far above the point and above 10 % that far below.
//! Each point decodes `GATE_BURSTS` bursts of up to `CODEWORDS` codewords.
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
//! Environment: `MODES` (narrow,wide), `LEVELS` (0,1,..,6), `TRIALS` (bursts per
//! point: 100 for the gate, 200 for the sweep's fine grid), `WRITE` (sweep only).

use super::*;
use crate::detect_eval::{burst, channel_seed, env_or, modem, payload_seed, true_position};
use vardefm_sim::{transmit, FmChannelConfig};

/// Codeword error rate at which a level's point is recorded and gated.
const TARGET_CER: f32 = 0.1;
/// The gate measures this far either side of the recorded point, dB.
const GATE_MARGIN_DB: f32 = 0.25;
/// Bursts per point of the gate. With 4 codewords per burst, sigma at 10 % CER is
/// 1.5 %, taking codewords as independent. The simulator does not fade, so the
/// codewords of a burst see independent noise; only a burst that fails to
/// demodulate as a whole, which does not happen near the recorded points, ties them.
const GATE_BURSTS: u32 = 100;
/// Bursts per point of the sweep's fine grid.
const RECORD_BURSTS: u32 = 200;
/// Bursts per point of the sweep's coarse search for the crossing.
const COARSE_BURSTS: u32 = 25;
/// The sweep's search gives up above this CNR, dB: a level that still fails there
/// has an error floor.
const MAX_CNR_DB: i32 = 40;
/// Codewords per burst, or fewer where a level's burst holds fewer.
const CODEWORDS: usize = 4;

/// The recorded points, `perf/levels.toml`.
const BASELINE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/perf/levels.toml");

#[derive(serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct Baseline {
    source: Source,
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
    codewords: usize,
}

#[derive(serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct Point {
    mode: String,
    level: u8,
    name: String,
    /// CNR at which CER crosses `TARGET_CER`, dB.
    cnr_db: f32,
    /// The fine grid the crossing comes from, for review in diffs.
    grid_db: Vec<f32>,
    cer: Vec<f32>,
}

fn load() -> Baseline {
    let text = std::fs::read_to_string(BASELINE).unwrap_or_else(|e| panic!("{BASELINE}: {e}"));
    toml::from_str(&text).unwrap_or_else(|e| panic!("{BASELINE}: {e}"))
}

/// A comma-separated list from the environment, or `default` when unset.
fn env_list<T: std::str::FromStr>(name: &str, default: &str) -> Vec<T> {
    env_or(name, default.to_string()).split(',').map(|s| env_or_parse(name, s)).collect()
}

fn env_or_parse<T: std::str::FromStr>(name: &str, s: &str) -> T {
    s.trim().parse().unwrap_or_else(|_| panic!("{name}: {s:?} does not parse"))
}

fn codewords(m: &Modem, level: u8) -> usize {
    m.max_ncw(level).min(CODEWORDS)
}

/// Codewords of trial `t` that fail to decode, from the true position `truth`.
fn cw_errors(m: &Modem, mode: &str, cnr: f32, level: u8, truth: usize, t: u32) -> u32 {
    let ncw = codewords(m, level);
    let sent = burst(level, ncw, payload_seed(t));
    let cfg =
        FmChannelConfig { cnr_db: Some(cnr), seed: channel_seed(cnr, t), ..FmChannelConfig::by_name(mode).unwrap() };
    let rx = transmit(&cfg, &m.modulate(&sent));
    let nsym = m.header_symbols() + ncw * m.symbols_per_codeword(level);
    let Some(frame) = m.ofdm.demodulate_known(&rx, truth, nsym, &m.header_carriers(&sent.header)) else {
        return ncw as u32;
    };
    let got = m.decode_payload(&sent.header, &frame);
    got.iter().zip(&sent.payload).filter(|(g, w)| g.as_ref() != Some(*w)).count() as u32
}

/// Codeword error rate over `bursts` bursts at `cnr` dB, on every core.
fn cer(m: &Modem, mode: &str, level: u8, cnr: f32, bursts: u32) -> f32 {
    let truth = true_position(m, mode, level, codewords(m, level));
    let threads = std::thread::available_parallelism().map_or(1, |n| n.get()) as u32;
    let errors: u32 = std::thread::scope(|s| {
        let workers: Vec<_> = (0..threads)
            .map(|th| {
                s.spawn(move || {
                    (th..bursts)
                        .step_by(threads as usize)
                        .map(|t| cw_errors(m, mode, cnr, level, truth, t))
                        .sum::<u32>()
                })
            })
            .collect();
        workers.into_iter().map(|w| w.join().unwrap()).sum()
    });
    errors as f32 / (bursts as usize * codewords(m, level)) as f32
}

/// Where `rates` (falling with `grid`) cross `target`, by interpolation of log10 of
/// the rate between the bracketing points, or linearly when one of them is 0.
/// `None` when the grid does not bracket the target.
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

/// Measures the 10 % CER point of every mode and level: a 1 dB search for the
/// crossing, then a 0.25 dB grid of ±1 dB around it. Warns when the curve is too
/// flat for the gate to tell a shift of `GATE_MARGIN_DB` from noise.
#[test]
#[ignore]
fn level_sweep() {
    let modes: Vec<String> = env_list("MODES", "narrow,wide");
    let levels: Vec<u8> = env_list("LEVELS", "0,1,2,3,4,5,6");
    let bursts = env_or("TRIALS", RECORD_BURSTS);
    let write = std::env::var("WRITE").is_ok_and(|v| v == "1");
    let mut points = Vec::new();
    for mode in &modes {
        let m = modem(mode);
        // The crossing rises with the level; start the search a little below the last.
        let mut start = 0i32;
        for &level in &levels {
            let at = |c: i32| cer(&m, mode, level, c as f32, COARSE_BURSTS);
            let mut c = start;
            while at(c) < TARGET_CER {
                c -= 1;
            }
            while at(c + 1) >= TARGET_CER {
                c += 1;
                assert!(c < MAX_CNR_DB, "{mode} L{level}: CER is still 10 % or more at {MAX_CNR_DB} dB");
            }
            start = c - 2;
            // Whole quarter dB steps, so the recorded grid reads as written.
            let centre = c * 4 + 2;
            let grid: Vec<f32> = (centre - 4..=centre + 4).map(|i| i as f32 / 4.0).collect();
            let rates: Vec<f32> = grid.iter().map(|&g| cer(&m, mode, level, g, bursts)).collect();
            let Some(x) = crossing(&grid, &rates, TARGET_CER) else {
                panic!("{mode} L{level}: the grid {grid:?} does not cross 10 % CER: {rates:?}");
            };
            let cnr_db = (x * 100.0).round() / 100.0;
            let line: String = grid.iter().zip(&rates).map(|(g, r)| format!(" {g:.2}:{r:.3}")).collect();
            println!("{mode} L{level} {}: 10 % CER at {cnr_db:.2} dB;{line}", LEVELS[level as usize].name);
            // CER the gate would see on either side, interpolated on the grid.
            let sigma = (TARGET_CER * (1.0 - TARGET_CER) / (GATE_BURSTS as usize * codewords(&m, level)) as f32).sqrt();
            for d in [-GATE_MARGIN_DB, GATE_MARGIN_DB] {
                let r = rate_at(&grid, &rates, x + d);
                if (r - TARGET_CER).abs() < 3.0 * sigma {
                    println!("  WARNING: CER {r:.3} at {:.2} dB is within 3 sigma of 10 %", x + d);
                }
            }
            points.push(Point {
                mode: mode.clone(),
                level,
                name: LEVELS[level as usize].name.into(),
                cnr_db,
                grid_db: grid,
                cer: rates,
            });
        }
    }
    if write {
        let b = Baseline {
            source: Source {
                what: "CNR at 10 % codeword error rate, decoded from the known burst position".into(),
                commit: commit(),
                command: format!(
                    "WRITE=1 TRIALS={bursts} cargo test --release -p vardefm-modem level_sweep -- --ignored"
                ),
                coarse_bursts: COARSE_BURSTS,
                bursts,
                codewords: CODEWORDS,
            },
            level: points,
        };
        std::fs::write(BASELINE, toml::to_string(&b).unwrap()).unwrap();
        println!("wrote {BASELINE}");
    }
}

/// Fails when a level's 10 % CER point has moved from `perf/levels.toml` by more
/// than `GATE_MARGIN_DB`.
#[test]
#[ignore]
fn level_gate() {
    let modes: Vec<String> = env_list("MODES", "narrow,wide");
    let levels: Vec<u8> = env_list("LEVELS", "0,1,2,3,4,5,6");
    let bursts = env_or("TRIALS", GATE_BURSTS);
    let base = load();
    println!("baseline {} ({})", base.source.commit, base.source.command);
    let mut failed = Vec::new();
    for mode in &modes {
        let m = modem(mode);
        for &level in &levels {
            let p = base
                .level
                .iter()
                .find(|p| p.mode == *mode && p.level == level)
                .unwrap_or_else(|| panic!("{BASELINE} has no {mode} level {level}"));
            let below = cer(&m, mode, level, p.cnr_db - GATE_MARGIN_DB, bursts);
            let above = cer(&m, mode, level, p.cnr_db + GATE_MARGIN_DB, bursts);
            let ok = above < TARGET_CER && below > TARGET_CER;
            println!(
                "{mode} L{level} at {:.2} dB: CER {below:.3} at -{GATE_MARGIN_DB} dB, {above:.3} at +{GATE_MARGIN_DB} dB {}",
                p.cnr_db,
                if ok { "ok" } else { "FAIL" }
            );
            if !ok {
                failed.push(format!("{mode} L{level}"));
            }
        }
    }
    assert!(failed.is_empty(), "10 % CER point moved for {failed:?}");
}

#[test]
fn crossing_interpolates_in_log() {
    let grid = [1.0, 1.5, 2.0];
    assert_eq!(crossing(&grid, &[0.5, 0.1, 0.01], 0.1), Some(1.5));
    let x = crossing(&grid, &[1.0, 0.1, 0.01], 0.0316).unwrap();
    assert!((x - 1.75).abs() < 1e-3, "{x}");
    assert_eq!(crossing(&grid, &[0.3, 0.2, 0.0], 0.1), Some(1.75));
    assert_eq!(crossing(&grid, &[0.05, 0.02, 0.0], 0.1), None);
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
fn baseline_covers_every_mode_and_level() {
    let base = load();
    for mode in ["narrow", "wide"] {
        for level in 0..LEVELS.len() as u8 {
            let p = base.level.iter().find(|p| p.mode == mode && p.level == level);
            let p = p.unwrap_or_else(|| panic!("no {mode} level {level}"));
            assert_eq!(p.grid_db.len(), p.cer.len());
        }
    }
}
