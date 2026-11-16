//! `sim-pair`: two VardeFM stations (A and B) in one process, each with its own
//! VARA TCP ports, joined by the FM channel simulator. Optionally up to two
//! digipeaters sit between them in a chain (A–D1–D2–B). Each station then hears
//! only its neighbours, so A and B can reach each other only through the digipeaters.
//!
//! Every transmission goes through [`vardefm_sim::transmit`] (key-up delay,
//! squelch tail, noise, clock offset). Stations are half-duplex: a station hears
//! nothing while it transmits, and overlapping transmissions add up (collide).

use crate::host::valid_call;
use crate::server::{HostIn, HostPort};
use crate::station::{HostOut, Station};
use std::io;
use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};
use vardefm_link::LinkConfig;
use vardefm_modem::{Modem, OfdmParams, LEVELS, MAX_DIGIS};
use vardefm_sim::{transmit, Fading, FmChannelConfig};

/// Audio step: 10 ms.
const STEP: usize = 480;

#[derive(Clone, Debug)]
pub struct SimPairConfig {
    /// "narrow" or "wide".
    pub mode: String,
    /// Carrier-to-noise ratio in dB; None = noiseless.
    pub cnr_db: Option<f32>,
    /// Flat fading (None = static). Each transmission gets its own independent fading
    /// process, seeded like its noise.
    pub fading: Option<Fading>,
    /// Receiver clock error in ppm (both directions).
    pub ppm: f32,
    /// Radio key-up delay in seconds.
    pub keyup_s: f32,
    /// Leader before each burst (must exceed `keyup_s`).
    pub txdelay_s: f32,
    pub seed: u64,
    /// Simulated seconds per real second. 0 or less = as fast as possible.
    pub speed: f64,
    /// Address to listen on.
    pub host: String,
    /// Command ports of A and B (data = command + 1). 0 = pick free ports.
    pub a_port: u16,
    pub b_port: u16,
    /// Save every transmission as WAV files in this folder.
    pub record: Option<PathBuf>,
    /// Print one line per transmission and reception.
    pub verbose: bool,
    /// Call signs of digipeaters between A and B, in order from A (at most two).
    pub digis: Vec<String>,
    /// Extra wait per digipeater hop on A's and B's routed sessions.
    pub hop_margin_s: f64,
}

impl Default for SimPairConfig {
    fn default() -> Self {
        Self {
            mode: "narrow".into(),
            cnr_db: Some(20.0),
            fading: None,
            ppm: 0.0,
            keyup_s: 0.05,
            txdelay_s: 0.1,
            seed: 1,
            speed: 1.0,
            host: "127.0.0.1".into(),
            a_port: 8300,
            b_port: 8310,
            record: None,
            verbose: true,
            digis: Vec::new(),
            hop_margin_s: LinkConfig::default().hop_margin_s,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct SimPairStats {
    /// Simulated time, seconds.
    pub now: f64,
    /// Seconds each station has transmitted, in chain order: A, the digipeaters, B.
    pub airtime: Vec<f64>,
    pub bursts: Vec<u64>,
    pub log: Vec<String>,
}

pub struct SimPair {
    /// (command, data) addresses of A and B.
    pub addrs: [(SocketAddr, SocketAddr); 2],
    stats: Arc<Mutex<SimPairStats>>,
    stop: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl SimPair {
    pub fn start(cfg: SimPairConfig) -> io::Result<Self> {
        let params = OfdmParams::by_name(&cfg.mode)
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "mode must be narrow or wide"))?;
        let invalid = |what: &str| Err(io::Error::new(io::ErrorKind::InvalidInput, what));
        if cfg.digis.len() > MAX_DIGIS {
            return invalid("at most two digipeaters");
        }
        // No spaces or other padding gets past this, so upper case is all that can
        // differ between two spellings of one call sign.
        if let Some(c) = cfg.digis.iter().find(|c| !valid_call(c)) {
            return invalid(&format!("digipeater {c:?}: not a valid call sign"));
        }
        // One station repeats for all its positions at once, so a second station with
        // the same call sign would never see a burst it should repeat.
        let calls: Vec<String> = cfg.digis.iter().map(|c| c.to_ascii_uppercase()).collect();
        if calls.iter().enumerate().any(|(i, c)| calls[..i].contains(c)) {
            return invalid("each digipeater needs its own call sign");
        }
        if !vardefm_link::valid_hop_margin(cfg.hop_margin_s) {
            return invalid("hop margin must be from 0 to 10 seconds");
        }
        let channel = FmChannelConfig::by_name(&cfg.mode).unwrap();
        let data_port = |p: u16| if p == 0 { 0 } else { p + 1 };
        let ports = [
            HostPort::bind(&cfg.host, cfg.a_port, data_port(cfg.a_port))?,
            HostPort::bind(&cfg.host, cfg.b_port, data_port(cfg.b_port))?,
        ];
        let addrs = [(ports[0].cmd_addr, ports[0].data_addr), (ports[1].cmd_addr, ports[1].data_addr)];
        if let Some(dir) = &cfg.record {
            std::fs::create_dir_all(dir)?;
        }
        let modem = Arc::new(Modem::new(params, cfg.txdelay_s));
        let link = LinkConfig { hop_margin_s: cfg.hop_margin_s, ..LinkConfig::default() };
        let count = cfg.digis.len() + 2;
        let seed = cfg.seed as u32 * 2;
        let mut stations = vec![Station::new("A", modem.clone(), link.clone(), seed + 1)];
        for (i, call) in cfg.digis.iter().enumerate() {
            let mut d = Station::new(&format!("D{}", i + 1), modem.clone(), link.clone(), seed + 3 + i as u32);
            d.set_digi(std::slice::from_ref(call));
            stations.push(d);
        }
        stations.push(Station::new("B", modem.clone(), link, seed + 2));
        let stats = Arc::new(Mutex::new(SimPairStats {
            airtime: vec![0.0; count],
            bursts: vec![0; count],
            ..SimPairStats::default()
        }));
        let stop = Arc::new(AtomicBool::new(false));
        let [pa, pb] = ports;
        let mut host_ports: Vec<Option<HostPort>> = (0..count).map(|_| None).collect();
        host_ports[0] = Some(pa);
        host_ports[count - 1] = Some(pb);
        let engine = Engine {
            stations,
            ports: host_ports,
            channel: FmChannelConfig {
                cnr_db: cfg.cnr_db,
                fading: cfg.fading,
                clock_offset_ppm: cfg.ppm,
                keyup_delay_s: cfg.keyup_s,
                lead_silence_s: 0.0,
                ..channel
            },
            incoming: vec![Vec::new(); count],
            tx_end: vec![None; count],
            t: 0,
            tx_count: 0,
            stats: stats.clone(),
            stop: stop.clone(),
            cfg,
        };
        let handle = thread::spawn(move || engine.run());
        Ok(Self { addrs, stats, stop, handle: Some(handle) })
    }

    pub fn stats(&self) -> SimPairStats {
        self.stats.lock().unwrap().clone()
    }

    /// Stop the simulation and close the ports.
    pub fn stop(mut self) {
        self.shutdown();
    }

    fn shutdown(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }

    /// Block until the simulation stops (it runs until the process ends).
    pub fn wait(mut self) {
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

impl Drop for SimPair {
    fn drop(&mut self) {
        self.shutdown();
    }
}

struct Engine {
    /// A, the digipeaters in order, B.
    stations: Vec<Station>,
    /// Host ports of A and B; digipeaters have none.
    ports: Vec<Option<HostPort>>,
    channel: FmChannelConfig,
    /// Audio on its way to each station: (start sample, samples).
    incoming: Vec<Vec<(u64, Vec<f32>)>>,
    /// Sample index at which each station's transmission ends.
    tx_end: Vec<Option<u64>>,
    /// Current sample index.
    t: u64,
    tx_count: u64,
    stats: Arc<Mutex<SimPairStats>>,
    stop: Arc<AtomicBool>,
    cfg: SimPairConfig,
}

impl Engine {
    fn fs(&self) -> f64 {
        self.stations[0].modem().fs() as f64
    }

    /// Stations hear their neighbours along the chain.
    fn hears(to: usize, from: usize) -> bool {
        to.abs_diff(from) == 1
    }

    fn log(&self, line: String) {
        if self.cfg.verbose {
            println!("{line}");
        }
        let mut st = self.stats.lock().unwrap();
        st.log.push(line);
    }

    fn run(mut self) {
        let start = Instant::now();
        let fs = self.fs();
        let n = self.stations.len();
        while !self.stop.load(Ordering::Relaxed) {
            let now = self.t as f64 / fs;

            // Host input.
            for i in 0..n {
                if let Some(port) = &self.ports[i] {
                    while let Ok(msg) = port.inbox.try_recv() {
                        match msg {
                            HostIn::Line(l) => self.stations[i].host_line(now, &l),
                            HostIn::Data(d) => self.stations[i].host_data(&d),
                            HostIn::CmdClosed => self.stations[i].host_closed(now),
                        }
                    }
                }
                self.stations[i].tick(now);
            }

            // Transmissions that end now.
            for i in 0..n {
                if self.tx_end[i].is_some_and(|e| e <= self.t) {
                    self.tx_end[i] = None;
                    self.stations[i].tx_done(now);
                }
            }

            // New transmissions.
            for i in 0..n {
                if self.tx_end[i].is_none() {
                    if let Some(tx) = self.stations[i].poll_tx(now) {
                        self.start_tx(i, now, tx);
                    }
                }
            }

            // Receive this step's audio.
            for i in 0..n {
                let mut chunk = vec![0f32; STEP];
                let (a, b) = (self.t, self.t + STEP as u64);
                for (s, audio) in &self.incoming[i] {
                    let end = s + audio.len() as u64;
                    if end <= a || *s >= b {
                        continue;
                    }
                    for k in a.max(*s)..b.min(end) {
                        chunk[(k - a) as usize] += audio[(k - s) as usize];
                    }
                }
                self.incoming[i].retain(|(s, audio)| s + audio.len() as u64 > b);
                for burst in self.stations[i].rx_audio(now, &chunk) {
                    let h = burst.header;
                    let ok = burst.payload.iter().filter(|p| p.is_some()).count();
                    self.log(format!(
                        "{now:8.2}s {} rx {:<10} cw {ok:>2}/{:<2} SNR {:5.1} dB",
                        self.stations[i].name,
                        format!("{:?}", h.kind),
                        h.ncw,
                        burst.snr_db
                    ));
                }
            }

            // Host output (digipeaters have no host; their lines are dropped).
            for i in 0..n {
                let out = self.stations[i].take_output();
                if let Some(port) = &self.ports[i] {
                    for o in out {
                        match o {
                            HostOut::Line(l) => port.send_line(&l),
                            HostOut::Data(d) => port.send_data(&d),
                        }
                    }
                }
            }

            self.t += STEP as u64;
            self.stats.lock().unwrap().now = self.t as f64 / fs;

            if self.cfg.speed > 0.0 {
                let target = start + Duration::from_secs_f64(self.t as f64 / fs / self.cfg.speed);
                let real = Instant::now();
                if target > real {
                    thread::sleep(target - real);
                }
            }
        }
    }

    fn start_tx(&mut self, i: usize, now: f64, tx: crate::station::TxStart) {
        let name = self.stations[i].name.clone();
        let to: Vec<usize> = (0..self.stations.len()).filter(|&j| Self::hears(j, i)).collect();
        let h = tx.header;
        let level = if h.ncw > 0 { LEVELS[h.level as usize].name } else { "-" };
        let targets: Vec<&str> = to.iter().map(|&j| self.stations[j].name.as_str()).collect();
        let line = if tx.repeat {
            format!(
                "{now:8.2}s {name}->{} {:<10} {:<9} cw {:>2} repeat hop {} air {:.2} s",
                targets.join(","),
                format!("{:?}", h.kind),
                level,
                h.ncw,
                h.hop(),
                tx.seconds
            )
        } else {
            format!(
                "{now:8.2}s {name}->{} {:<10} {:<9} cw {:>2} {:>5} B (resent {:>4}) retry {} air {:.2} s",
                targets.join(","),
                format!("{:?}", h.kind),
                level,
                h.ncw,
                tx.info.bytes,
                tx.info.resent_bytes,
                tx.info.retry,
                tx.seconds
            )
        };
        self.log(line);
        let pair = self.stations.len() == 2;
        for &j in &to {
            // Each receiver gets its own noise (a plain pair keeps its old seeds).
            let base = self.cfg.seed * 100_000 + self.tx_count;
            let seed = if pair { base } else { base * 8 + j as u64 };
            let channel = FmChannelConfig { seed, ..self.channel.clone() };
            let rx_audio = transmit(&channel, &tx.audio);
            if let Some(dir) = &self.cfg.record {
                let fs = self.fs() as u32;
                let rx_name = &self.stations[j].name;
                let file = if pair {
                    format!("{:04}_{name}_rx.wav", self.tx_count)
                } else {
                    format!("{:04}_{name}_rx_{rx_name}.wav", self.tx_count)
                };
                let _ = write_wav(&dir.join(file), fs, &rx_audio);
            }
            self.incoming[j].push((self.t, rx_audio));
        }
        if let Some(dir) = &self.cfg.record {
            let fs = self.fs() as u32;
            let _ = write_wav(&dir.join(format!("{:04}_{name}_tx.wav", self.tx_count)), fs, &tx.audio);
        }
        self.tx_count += 1;
        self.tx_end[i] = Some(self.t + tx.audio.len() as u64);
        let mut st = self.stats.lock().unwrap();
        st.airtime[i] += tx.seconds as f64;
        st.bursts[i] += 1;
    }
}

fn write_wav(path: &std::path::Path, fs: u32, samples: &[f32]) -> Result<(), hound::Error> {
    let spec =
        hound::WavSpec { channels: 1, sample_rate: fs, bits_per_sample: 16, sample_format: hound::SampleFormat::Int };
    let mut w = hound::WavWriter::create(path, spec)?;
    for &v in samples {
        w.write_sample((v.clamp(-1.0, 1.0) * 32767.0) as i16)?;
    }
    w.finalize()
}
