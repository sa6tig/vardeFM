//! Rate adaptation dynamics: how often the speed level flips between bursts, and
//! whether it settles into cycles, when the link's SNR model does or does not match
//! the channel.
//!
//! Two stations (link + modulator + receiver, no TCP) run through the FM channel
//! simulator. A sends a steady stream to B. The transmitter and receiver use the true
//! modem parameters. Each scenario breaks one assumption of the rate choice:
//! - the link's per-level drive model is optimistic (a real radio whose drive or
//!   clipping differs from the simulator's table);
//! - the SNR handed to the link is offset (codewords decode worse than the SNR
//!   thresholds say, e.g. impairments the SNR estimate does not see);
//! - the SNR handed to the link jitters burst to burst (fading, interference).
//!
//! Run (release, a few minutes):
//!   cargo test --release -p vardefm-tnc --test level_dynamics -- --ignored --nocapture
//! Environment:
//!   LD_MODES=narrow,wide  LD_CNRS=10,14,...  LD_SCENARIOS=exact,snr+3,...
//!   LD_SECONDS=90  LD_SEED=1  LD_TRACE=dir (one CSV of received data bursts per run)

use std::sync::{Arc, Mutex};
use std::thread;
use vardefm_link::{Link, LinkConfig, LinkEvent, Node};
use vardefm_modem::{Modem, OfdmParams, Receiver, RxBurst};
use vardefm_sim::{transmit, FmChannelConfig};

const STEP: usize = 480;

/// How the link's view of the channel differs from the truth.
#[derive(Clone, Copy)]
struct Scenario {
    name: &'static str,
    /// Link believes each level goes out this much louder than it does, dB.
    drive_db: [f32; 7],
    /// Added to every SNR measurement handed to the link, dB.
    snr_offset_db: f32,
    /// Standard deviation of Gaussian noise added to each SNR measurement, dB.
    snr_jitter_db: f32,
}

const fn sc(name: &'static str, drive_db: [f32; 7], snr_offset_db: f32, snr_jitter_db: f32) -> Scenario {
    Scenario { name, drive_db, snr_offset_db, snr_jitter_db }
}
const D0: [f32; 7] = [0.0; 7];

const SCENARIOS: [Scenario; 8] = [
    sc("exact", D0, 0.0, 0.0),
    // Real 64QAM drive 2 dB below what the link assumes.
    sc("drive64+2", [0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 2.0], 0.0, 0.0),
    // Decoding 1.5 / 3 / 5 dB worse than the SNR says.
    sc("snr+1.5", D0, 1.5, 0.0),
    sc("snr+3", D0, 3.0, 0.0),
    // Beyond the hold's 3 dB escape margin.
    sc("snr+5", D0, 5.0, 0.0),
    // Burst-to-burst SNR scatter.
    sc("jitter1", D0, 0.0, 1.0),
    sc("jitter2", D0, 0.0, 2.0),
    // Both: worse decoding and scatter.
    sc("snr+2,jit1", D0, 2.0, 1.0),
];

/// Small deterministic Gaussian source (xorshift + Box-Muller).
struct Gauss(u64);

impl Gauss {
    fn uniform(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        ((self.0 >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }
    fn next(&mut self) -> f32 {
        let (u, v) = (self.uniform(), self.uniform());
        ((-2.0 * u.ln()).sqrt() * (2.0 * std::f64::consts::PI * v).cos()) as f32
    }
}

/// One station: link, modulator and receiver (as `vardefm_tnc::station`, minus the host protocol).
struct Station {
    node: Node,
    rx: Receiver,
    transmitting: bool,
}

fn env_list(name: &str) -> Option<Vec<String>> {
    std::env::var(name).ok().map(|v| v.split(',').map(|s| s.trim().to_string()).filter(|s| !s.is_empty()).collect())
}

fn env_f64(name: &str, default: f64) -> f64 {
    std::env::var(name).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

/// One data burst received by B.
#[derive(Clone, Copy)]
struct RxRec {
    t: f64,
    level: u8,
    ncw: u8,
    failed: u8,
    snr_db: f32,
}

struct RunResult {
    mode: String,
    scenario: &'static str,
    cnr_db: f32,
    bursts: Vec<RxRec>,
    retries: u64,
    delivered: usize,
    seconds: f64,
}

fn run(mode: &str, sc: Scenario, cnr_db: f32, seconds: f64, seed: u64) -> RunResult {
    let params = OfdmParams::by_name(mode).unwrap();
    let mut true_modem = Modem::new(params.clone(), 0.1);
    true_modem.decode_threads = 1;
    let true_modem = Arc::new(true_modem);
    let mut lp = params;
    for l in 0..7 {
        lp.level_tx_rms[l] *= 10f32.powf(sc.drive_db[l] / 20.0);
    }
    let link_modem = Arc::new(Modem::new(lp, 0.1));
    let mk = |s: u32| Station {
        node: Node::new(Link::new(link_modem.clone(), LinkConfig::default(), s)),
        rx: Receiver::new(true_modem.clone()),
        transmitting: false,
    };
    let mut st = [mk(seed as u32 * 2 + 1), mk(seed as u32 * 2 + 2)];
    let mut gauss = Gauss(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1);
    let channel = FmChannelConfig {
        cnr_db: Some(cnr_db),
        keyup_delay_s: 0.05,
        lead_silence_s: 0.0,
        ..FmChannelConfig::by_name(mode).unwrap()
    };
    st[1].node.link.set_mycalls(vec!["LA2BBB".into()]);
    st[1].node.link.set_listen(true);
    st[0].node.link.connect(0.0, "LA1AAA", "LA2BBB", &[]).unwrap();

    let fs = true_modem.fs() as f64;
    let mut incoming: [Vec<(u64, Vec<f32>)>; 2] = [Vec::new(), Vec::new()];
    let mut tx_end: [Option<u64>; 2] = [None, None];
    let mut t: u64 = 0;
    let mut tx_count = 0u64;
    let mut connected_at: Option<f64> = None;
    let mut bursts = Vec::new();
    let mut delivered = 0usize;
    let mut host_sent = 0usize;
    let chunk_data = vec![0x55u8; 4096];

    loop {
        let now = t as f64 / fs;
        if connected_at.is_some_and(|c| now - c >= seconds) || (connected_at.is_none() && now > 60.0) {
            break;
        }
        // Keep A's send buffer topped up once connected.
        if connected_at.is_some() && st[0].node.link.is_connected() && host_sent - delivered < 64 * 1024 {
            st[0].node.link.send_data(&chunk_data);
            host_sent += chunk_data.len();
        }
        for i in 0..2 {
            if tx_end[i].is_some_and(|e| e <= t) {
                tx_end[i] = None;
                st[i].transmitting = false;
                st[i].node.tx_done(now);
            }
        }
        for i in 0..2 {
            if tx_end[i].is_none() {
                let busy = st[i].rx.busy();
                if let Some((burst, _)) = st[i].node.poll(now, busy) {
                    let audio = true_modem.modulate(&burst);
                    st[i].transmitting = true;
                    st[i].rx.reset();
                    let ch = FmChannelConfig { seed: seed * 100_000 + tx_count, ..channel.clone() };
                    tx_count += 1;
                    let rx = transmit(&ch, &audio);
                    tx_end[i] = Some(t + audio.len() as u64);
                    incoming[1 - i].push((t, rx));
                }
            }
        }
        for i in 0..2 {
            let mut chunk = vec![0f32; STEP];
            let (a, b) = (t, t + STEP as u64);
            for (s, audio) in &incoming[i] {
                let end = s + audio.len() as u64;
                if end <= a || *s >= b {
                    continue;
                }
                for k in a.max(*s)..b.min(end) {
                    chunk[(k - a) as usize] += audio[(k - s) as usize];
                }
            }
            incoming[i].retain(|(s, audio)| s + audio.len() as u64 > b);
            if st[i].transmitting {
                continue;
            }
            for burst in st[i].rx.push(&chunk) {
                let seen = RxBurst {
                    snr_db: burst.snr_db + sc.snr_offset_db + sc.snr_jitter_db * gauss.next(),
                    ..burst.clone()
                };
                st[i].node.on_burst(now, &seen);
                if let Some(c) = connected_at.filter(|_| i == 1 && burst.header.ncw > 0) {
                    bursts.push(RxRec {
                        t: now - c,
                        level: burst.header.level,
                        ncw: burst.header.ncw,
                        failed: burst.payload.iter().filter(|p| p.is_none()).count() as u8,
                        snr_db: burst.snr_db,
                    });
                }
            }
        }
        for (i, s) in st.iter_mut().enumerate() {
            for e in s.node.link.take_events() {
                match e {
                    LinkEvent::Connected { .. } if i == 0 && connected_at.is_none() => connected_at = Some(now),
                    LinkEvent::Data(d) if i == 1 && connected_at.is_some() => delivered += d.len(),
                    _ => {}
                }
            }
        }
        t += STEP as u64;
    }
    RunResult {
        mode: mode.to_string(),
        scenario: sc.name,
        cnr_db,
        bursts,
        retries: st[0].node.link.stats.retries,
        delivered,
        seconds: connected_at.map_or(0.0, |_| seconds),
    }
}

fn summarize(r: &RunResult) -> String {
    let b = &r.bursts;
    let n = b.len().max(1) as f64;
    let switches = b.windows(2).filter(|w| w[0].level != w[1].level).count();
    let ups = b.windows(2).filter(|w| w[1].level > w[0].level).count();
    // A-B-A: back where it was two bursts ago after a change.
    let flips = b.windows(3).filter(|w| w[0].level == w[2].level && w[0].level != w[1].level).count();
    let failed_bursts = b.iter().filter(|x| x.failed > 0).count();
    let cw: usize = b.iter().map(|x| x.ncw as usize).sum();
    let cw_failed: usize = b.iter().map(|x| x.failed as usize).sum();
    // Failure right after stepping up.
    let up_then_fail = b.windows(2).filter(|w| w[1].level > w[0].level && w[1].failed > 0).count();
    let mut hist = [0usize; 7];
    for x in b {
        hist[x.level as usize] += 1;
    }
    let pct = |h: usize| if h == 0 { ".".into() } else { format!("{:.0}", 100.0 * h as f64 / n) };
    let hist: Vec<String> = hist.iter().map(|&h| pct(h)).collect();
    let snr = if b.is_empty() { f32::NAN } else { b.iter().map(|x| x.snr_db).sum::<f32>() / b.len() as f32 };
    format!(
        "{:<6} {:<10} {:>5.1} {:>5} {:>6.2} {:>6.2} {:>6.2} {:>6.3} {:>6.3} {:>5} {:>4} {:>7.0}  {:>5.1}  {}",
        r.mode,
        r.scenario,
        r.cnr_db,
        b.len(),
        switches as f64 / n,
        ups as f64 / n,
        flips as f64 / n,
        failed_bursts as f64 / n,
        cw_failed as f64 / cw.max(1) as f64,
        up_then_fail,
        r.retries,
        if r.seconds > 0.0 { r.delivered as f64 * 8.0 / r.seconds } else { 0.0 },
        snr,
        hist.join("/"),
    )
}

#[test]
#[ignore]
fn level_dynamics() {
    let modes = env_list("LD_MODES").unwrap_or(vec!["narrow".into(), "wide".into()]);
    let cnrs: Vec<f32> = env_list("LD_CNRS")
        .map(|v| v.iter().map(|s| s.parse().unwrap()).collect())
        .unwrap_or((0..14).map(|i| 8.0 + 2.0 * i as f32).collect());
    let names = env_list("LD_SCENARIOS");
    let scenarios: Vec<Scenario> =
        SCENARIOS.iter().copied().filter(|s| names.as_ref().is_none_or(|n| n.iter().any(|x| x == s.name))).collect();
    let seconds = env_f64("LD_SECONDS", 90.0);
    let seed = env_f64("LD_SEED", 1.0) as u64;
    let trace = std::env::var("LD_TRACE").ok();

    let mut jobs = Vec::new();
    for m in &modes {
        for sc in &scenarios {
            for &c in &cnrs {
                jobs.push((m.clone(), *sc, c));
            }
        }
    }
    let total = jobs.len();
    let jobs = Arc::new(Mutex::new(jobs.into_iter().enumerate().collect::<Vec<_>>()));
    let results = Arc::new(Mutex::new(Vec::new()));
    let threads = thread::available_parallelism().map_or(4, |n| n.get());
    let handles: Vec<_> = (0..threads)
        .map(|_| {
            let (jobs, results) = (jobs.clone(), results.clone());
            thread::spawn(move || loop {
                let Some((k, (m, sc, c))) = jobs.lock().unwrap().pop() else { break };
                let r = run(&m, sc, c, seconds, seed);
                eprintln!("done {}/{}: {}", results.lock().unwrap().len() + 1, total, summarize(&r));
                results.lock().unwrap().push((k, r));
            })
        })
        .collect();
    for h in handles {
        h.join().unwrap();
    }
    let mut results = Arc::try_unwrap(results).ok().unwrap().into_inner().unwrap();
    results.sort_by_key(|(k, _)| *k);
    println!(
        "{:<6} {:<10} {:>5} {:>5} {:>6} {:>6} {:>6} {:>6} {:>6} {:>5} {:>4} {:>7}  {:>5}  level % (0/1/2/3/4/5/6)",
        "mode", "scenario", "CNR", "burst", "sw", "up", "ABA", "fail", "cwfail", "upF", "rtry", "bit/s", "SNR"
    );
    for (_, r) in &results {
        println!("{}", summarize(r));
        if let Some(dir) = &trace {
            std::fs::create_dir_all(dir).unwrap();
            let path = format!("{dir}/{}_{}_{:.1}.csv", r.mode, r.scenario, r.cnr_db);
            let mut s = String::from("t,level,ncw,failed,snr_db\n");
            for x in &r.bursts {
                s += &format!("{:.2},{},{},{},{:.2}\n", x.t, x.level, x.ncw, x.failed, x.snr_db);
            }
            std::fs::write(path, s).unwrap();
        }
    }
}
