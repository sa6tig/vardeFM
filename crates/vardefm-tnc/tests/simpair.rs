//! End to end: two host applications talk over the VARA TCP interface through
//! `sim-pair` (two VardeFM modems and a simulated FM link).
//!
//! Set VARDEFM_SIM_VERBOSE=1 to print every transmission.

use std::io::{Read, Write};
use std::net::{SocketAddr, TcpStream};
use std::sync::mpsc::{channel, Receiver, RecvTimeoutError};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use vardefm_dsp::prng::Prng;
use vardefm_sim::Fading;
use vardefm_tnc::simpair::{SimPair, SimPairConfig};

const TIMEOUT: Duration = Duration::from_secs(240);

/// A minimal host application.
struct Host {
    name: &'static str,
    cmd: TcpStream,
    data: TcpStream,
    lines: Receiver<String>,
    seen: Vec<String>,
    rx: Arc<Mutex<Vec<u8>>>,
}

impl Host {
    fn connect(name: &'static str, (cmd_addr, data_addr): (SocketAddr, SocketAddr)) -> Self {
        let cmd = TcpStream::connect(cmd_addr).unwrap();
        let data = TcpStream::connect(data_addr).unwrap();
        let (tx, lines) = channel();
        let mut r = cmd.try_clone().unwrap();
        thread::spawn(move || {
            let mut buf = [0u8; 1024];
            let mut line = Vec::new();
            while let Ok(n) = r.read(&mut buf) {
                if n == 0 {
                    break;
                }
                for &b in &buf[..n] {
                    if b == b'\r' {
                        let _ = tx.send(String::from_utf8_lossy(&line).into_owned());
                        line.clear();
                    } else {
                        line.push(b);
                    }
                }
            }
        });
        let rx = Arc::new(Mutex::new(Vec::new()));
        let (rx2, mut d) = (rx.clone(), data.try_clone().unwrap());
        thread::spawn(move || {
            let mut buf = [0u8; 4096];
            while let Ok(n) = d.read(&mut buf) {
                if n == 0 {
                    break;
                }
                rx2.lock().unwrap().extend_from_slice(&buf[..n]);
            }
        });
        // Give the modem's acceptor threads a moment to register both sockets.
        thread::sleep(Duration::from_millis(100));
        Self { name, cmd, data, lines, seen: Vec::new(), rx }
    }

    fn send(&mut self, line: &str) {
        self.cmd.write_all(format!("{line}\r").as_bytes()).unwrap();
    }

    /// Wait for a line starting with `prefix`, skipping (but recording) others.
    fn expect(&mut self, prefix: &str) -> String {
        let deadline = Instant::now() + TIMEOUT;
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            match self.lines.recv_timeout(left) {
                Ok(l) => {
                    self.seen.push(l.clone());
                    if l.starts_with(prefix) {
                        return l;
                    }
                }
                Err(RecvTimeoutError::Timeout) | Err(RecvTimeoutError::Disconnected) => {
                    panic!("{}: no '{prefix}' line; saw {:?}", self.name, self.seen)
                }
            }
        }
    }

    /// The modem's reply to the last command (OK or WRONG), skipping
    /// asynchronous lines such as IAMALIVE, BUSY and BUFFER.
    fn reply(&mut self) -> String {
        loop {
            let l = self.expect("");
            if l == "OK" || l == "WRONG" {
                return l;
            }
        }
    }

    fn command(&mut self, line: &str) {
        self.send(line);
        assert_eq!(self.reply(), "OK", "{}: {line}", self.name);
    }

    fn write(&mut self, bytes: &[u8]) {
        self.data.write_all(bytes).unwrap();
    }

    fn wait_rx(&self, n: usize) -> Vec<u8> {
        let deadline = Instant::now() + TIMEOUT;
        loop {
            let got = self.rx.lock().unwrap().clone();
            if got.len() >= n {
                return got;
            }
            assert!(Instant::now() < deadline, "{}: received only {} of {n} bytes", self.name, got.len());
            thread::sleep(Duration::from_millis(20));
        }
    }
}

fn payload(n: usize, seed: u32) -> Vec<u8> {
    // All 256 byte values (CR and 0x00 included), then pseudo-random bytes.
    let mut rng = Prng::new(seed);
    (0..n).map(|i| if i < 256 { i as u8 } else { rng.next_u32() as u8 }).collect()
}

/// Simulated time runs 20x faster than real time: fast enough for CI, slow
/// enough that host reaction times (milliseconds) stay realistic.
fn sim(mode: &str, cnr: Option<f32>, ppm: f32, seed: u64) -> SimPair {
    sim_fading(mode, cnr, ppm, seed, None)
}

fn sim_fading(mode: &str, cnr: Option<f32>, ppm: f32, seed: u64, fading: Option<Fading>) -> SimPair {
    SimPair::start(SimPairConfig {
        mode: mode.into(),
        cnr_db: cnr,
        fading,
        ppm,
        seed,
        speed: 20.0,
        a_port: 0,
        b_port: 0,
        verbose: std::env::var("VARDEFM_SIM_VERBOSE").is_ok(),
        ..SimPairConfig::default()
    })
    .unwrap()
}

/// Full session: connect A->B, 10 kB each way, graceful disconnect.
fn session(mode: &str, cnr: Option<f32>, ppm: f32, seed: u64) {
    session_fading(mode, cnr, ppm, seed, None);
}

fn session_fading(mode: &str, cnr: Option<f32>, ppm: f32, seed: u64, fading: Option<Fading>) {
    let sp = sim_fading(mode, cnr, ppm, seed, fading);
    let mut a = Host::connect("A", sp.addrs[0]);
    let mut b = Host::connect("B", sp.addrs[1]);

    b.command("MYCALL LA2BBB");
    b.command("LISTEN ON");
    a.command("PUBLIC ON"); // Pat sends these at start-up
    a.command("COMPRESSION TEXT");
    a.command("MYCALL LA1AAA");
    a.command("LISTEN OFF");
    a.send("VERSION");
    assert_eq!(a.reply(), "OK");
    assert!(a.expect("VERSION ").starts_with("VERSION VardeFM"));

    a.command("CONNECT LA1AAA LA2BBB");
    let bw = if mode == "wide" { "WIDE" } else { "NARROW" };
    assert_eq!(a.expect("CONNECTED"), format!("CONNECTED LA1AAA LA2BBB {bw}"));
    b.expect("PENDING");
    assert_eq!(b.expect("CONNECTED"), format!("CONNECTED LA1AAA LA2BBB {bw}"));
    let t0 = sp.stats();

    // Like a BBS: the server (B) sends first, the client answers.
    let (to_a, to_b) = (payload(10_000, seed as u32), payload(10_000, seed as u32 + 1));
    b.write(&to_a);
    a.wait_rx(1);
    a.write(&to_b);
    assert_eq!(a.wait_rx(to_a.len()), to_a, "A received corrupted data");
    assert_eq!(b.wait_rx(to_b.len()), to_b, "B received corrupted data");
    a.expect("BUFFER 0");
    let t1 = sp.stats();

    a.command("DISCONNECT");
    a.expect("DISCONNECTED");
    b.expect("DISCONNECTED");
    let t2 = sp.stats();

    let secs = t1.now - t0.now;
    let air = (t1.airtime[0] - t0.airtime[0], t1.airtime[1] - t0.airtime[1]);
    println!(
        "{mode} cnr={cnr:?} ppm={ppm} fading={fading:?}: 2 x 10 kB in {secs:.1} s ({:.0} bit/s total), airtime A {:.1} s / B {:.1} s, \
         bursts {}, disconnect {:.1} s",
        20_000.0 * 8.0 / secs,
        air.0,
        air.1,
        t1.bursts[0] + t1.bursts[1] - t0.bursts[0] - t0.bursts[1],
        t2.now - t1.now
    );
    for h in [&a, &b] {
        assert!(!h.seen.iter().any(|l| l == "WRONG"), "{}: got WRONG", h.name);
        assert!(h.seen.iter().any(|l| l == "PTT ON") && h.seen.iter().any(|l| l == "PTT OFF"));
    }
    sp.stop();
}

#[test]
fn narrow_clean_channel() {
    session("narrow", Some(30.0), 0.0, 1);
}

#[test]
fn narrow_weak_signal_with_clock_error() {
    session("narrow", Some(12.0), 80.0, 2);
}

#[test]
fn wide_moderate_signal() {
    session("wide", Some(24.0), -50.0, 3);
}

#[test]
fn narrow_mobile_fading() {
    // A mobile path: every burst fades (Rayleigh, 20 Hz Doppler) at 30 dB mean CNR.
    // Some bursts are lost or partly decoded; ARQ and rate adaptation must still
    // deliver every byte intact.
    session_fading("narrow", Some(30.0), 0.0, 5, Some(Fading::rayleigh(20.0)));
}

#[test]
fn connect_to_nobody_fails_in_bounded_time() {
    let sp = sim("narrow", Some(30.0), 0.0, 4);
    let mut a = Host::connect("A", sp.addrs[0]);
    a.command("MYCALL LA1AAA");
    let t0 = sp.stats().now;
    a.command("CONNECT LA1AAA LA9ZZZ");
    a.expect("DISCONNECTED");
    let t = sp.stats().now - t0;
    assert!(t < 30.0, "gave up after {t:.1} s");
    assert!(!a.seen.iter().any(|l| l.starts_with("CONNECTED")));
    sp.stop();
}

/// HamBBS regression: DISCONNECT sent right after a data write (no wait for
/// BUFFER 0) must still deliver every byte written before it. Simulation runs as
/// fast as possible, the worst case for the race between the two sockets.
/// Between sessions, DISCONNECT and ABORT for the ended link must be ignored.
#[test]
fn disconnect_right_after_write_delivers_everything() {
    let sp = SimPair::start(SimPairConfig {
        mode: "wide".into(),
        cnr_db: Some(35.0),
        seed: 6,
        speed: 0.0,
        a_port: 0,
        b_port: 0,
        verbose: std::env::var("VARDEFM_SIM_VERBOSE").is_ok(),
        ..SimPairConfig::default()
    })
    .unwrap();
    let mut a = Host::connect("A", sp.addrs[0]);
    let mut b = Host::connect("B", sp.addrs[1]);
    b.command("MYCALL LA2BBB");
    b.command("LISTEN ON");
    a.command("MYCALL LA1AAA");
    let mut expected = Vec::new();
    for (round, size) in [1usize, 700, 3000, 5000, 64].into_iter().enumerate() {
        a.command("CONNECT LA1AAA LA2BBB");
        a.expect("CONNECTED");
        b.expect("CONNECTED");
        let data = payload(size, 100 + round as u32);
        expected.extend_from_slice(&data);
        a.write(&data);
        a.send("DISCONNECT"); // immediately, no BUFFER wait
        assert_eq!(a.reply(), "OK");
        a.expect("DISCONNECTED");
        b.expect("DISCONNECTED");
        let got = b.wait_rx(expected.len());
        assert_eq!(got, expected, "round {round}: B is missing data");

        // Stale commands for the ended link: OK, and nothing else happens.
        let mark = a.seen.len();
        a.command("DISCONNECT");
        a.command("ABORT");
        thread::sleep(Duration::from_millis(50));
        while let Ok(l) = a.lines.try_recv() {
            a.seen.push(l);
        }
        assert!(
            !a.seen[mark..].iter().any(|l| l == "DISCONNECTED" || l == "WRONG"),
            "round {round}: stale DISCONNECT/ABORT had an effect: {:?}",
            &a.seen[mark..]
        );
    }
    sp.stop();
}

/// Every write to the data port is followed by a BUFFER report, so a host can
/// wait for a report newer than its last write before disconnecting.
#[test]
fn buffer_report_follows_every_write() {
    let sp = sim("narrow", Some(30.0), 0.0, 7);
    let mut a = Host::connect("A", sp.addrs[0]);
    let mut b = Host::connect("B", sp.addrs[1]);
    b.command("MYCALL LA2BBB");
    b.command("LISTEN ON");
    a.command("MYCALL LA1AAA");
    a.command("CONNECT LA1AAA LA2BBB");
    a.expect("CONNECTED");
    for i in 0..5 {
        a.write(&payload(10, i));
        let l = a.expect("BUFFER ");
        let n: usize = l[7..].parse().unwrap();
        assert!(n > 0, "write {i}: report after the write still says {l}");
        // Wait for the acknowledgement's report before the next write, so that no
        // report is in flight when it goes out and the one we see is its own. (A
        // fixed drain delay raced the ack on slow CI runners.)
        a.expect("BUFFER 0");
    }
    sp.stop();
}

#[test]
fn wrong_commands_and_abort() {
    let sp = sim("narrow", Some(30.0), 0.0, 5);
    let mut a = Host::connect("A", sp.addrs[0]);
    let mut b = Host::connect("B", sp.addrs[1]);
    a.send("FOO BAR");
    assert_eq!(a.reply(), "WRONG");
    a.send("CONNECT LA1AAA LA2BBB VIA LA3CCC");
    assert_eq!(a.reply(), "WRONG");
    b.command("MYCALL LA2BBB");
    b.command("LISTEN ON");
    a.command("MYCALL LA1AAA");
    a.command("CONNECT LA1AAA LA2BBB");
    a.expect("CONNECTED");
    b.expect("CONNECTED");
    a.command("ABORT");
    a.expect("DISCONNECTED");
    b.expect("DISCONNECTED");
    sp.stop();
}
