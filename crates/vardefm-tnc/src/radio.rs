//! A VardeFM station on a real radio: sound card + PTT + VARA TCP ports.
//!
//! Time is the input sound card's sample clock. While transmitting, received
//! audio is ignored (half duplex). PTT is keyed before a burst's audio is
//! queued, and released only after the sound card has taken the last sample
//! plus `txtail_s`. A watchdog unkeys the radio if a transmission runs far
//! longer than expected.
//!
//! [`RadioEngine`] holds that logic with the sound card output and PTT behind
//! traits, so it can be tested without hardware; [`run`] wires it to real devices.
//!
//! With `--api`, [`run`] also serves the read-only monitor API (see [`crate::monitor`]):
//! the engine lists what happened in [`RadioEngine::events`], timed on the receive clock.

use crate::host::valid_call;
use crate::level::{meter_line, LevelMeter, Verdict};
use crate::monitor::Monitor;
use crate::server::{HostIn, HostPort};
use crate::station::{HostOut, Station};
use std::io;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use vardefm_api::wire::{ApiEvent, VfmBurstRx, VfmBurstTx, WireTime};
pub use vardefm_api::LoopbackAddr;
use vardefm_audio::{open_input, open_output, AudioOut, Channel, RATE};
use vardefm_link::{valid_hop_margin, LinkConfig, MAX_HOP_MARGIN_S};
use vardefm_modem::{AudioLevel, Header, Modem, OfdmParams, LEVELS};
use vardefm_ptt::{Ptt, PttSpec};

#[derive(Clone, Debug)]
pub struct RadioConfig {
    pub mode: String,
    pub host: String,
    /// Command port (data = +1).
    pub port: u16,
    pub audio_in: Option<String>,
    pub audio_out: Option<String>,
    pub in_channel: Channel,
    pub out_channel: Channel,
    pub ptt: PttSpec,
    /// Leader before each burst (radio key-up time plus margin).
    pub txdelay_s: f32,
    /// PTT held after the last sample leaves the sound card.
    pub txtail_s: f32,
    /// Output level multiplier (sets FM deviation together with the sound card mixer).
    pub tx_gain: f32,
    /// Save every transmitted burst as a WAV file in this folder.
    pub record: Option<PathBuf>,
    /// Highest speed level to send or request (limits the decoding load on slow computers).
    pub max_level: u8,
    pub verbose: bool,
    /// Print the receive level every [`LEVEL_WINDOW_S`] seconds while idle.
    pub show_levels: bool,
    /// Act as a digipeater for these call signs (empty: off).
    pub digi_calls: Vec<String>,
    /// Extra wait per digipeater hop on our own routed sessions.
    pub hop_margin_s: f64,
    /// Serve the monitor API on this address (none: off).
    pub api: Option<LoopbackAddr>,
}

impl RadioConfig {
    /// Refuses values the engine cannot run with. The command line checks its own
    /// arguments; this is for a config built any other way.
    pub fn check(&self) -> io::Result<()> {
        let invalid = |what: String| Err(io::Error::new(io::ErrorKind::InvalidInput, what));
        for (name, s) in [("txdelay", self.txdelay_s), ("txtail", self.txtail_s), ("tx gain", self.tx_gain)] {
            if !(s.is_finite() && s >= 0.0) {
                return invalid(format!("{name} must be 0 or more"));
            }
        }
        if self.max_level as usize >= LEVELS.len() {
            return invalid(format!("max level must be from 0 to {}", LEVELS.len() - 1));
        }
        if !valid_hop_margin(self.hop_margin_s) {
            return invalid(format!("hop margin must be from 0 to {MAX_HOP_MARGIN_S} seconds"));
        }
        if let Some(c) = self.digi_calls.iter().find(|c| !valid_call(c)) {
            return invalid(format!("digipeater {c:?}: not a valid call sign"));
        }
        Ok(())
    }
}

/// Seconds per line of `--levels` output.
pub const LEVEL_WINDOW_S: f32 = 2.0;
/// Shortest time between two level warnings.
const LEVEL_WARN_EVERY_S: f64 = 30.0;

impl Default for RadioConfig {
    fn default() -> Self {
        Self {
            mode: "narrow".into(),
            host: "127.0.0.1".into(),
            port: 8300,
            audio_in: None,
            audio_out: None,
            in_channel: Channel::Left,
            out_channel: Channel::Both,
            ptt: PttSpec::None,
            txdelay_s: 0.15,
            txtail_s: 0.05,
            tx_gain: 1.0,
            record: None,
            max_level: LEVELS.len() as u8 - 1,
            verbose: true,
            show_levels: false,
            digi_calls: Vec::new(),
            hop_margin_s: LinkConfig::default().hop_margin_s,
            api: None,
        }
    }
}

/// Where transmit audio goes.
pub trait AudioSink {
    /// Queue samples; returns the `consumed()` value at which they have all been taken.
    fn play(&mut self, samples: &[f32]) -> u64;
    /// Samples taken by the sound card so far.
    fn consumed(&self) -> u64;
    /// Drop queued samples.
    fn flush(&mut self);
}

impl AudioSink for AudioOut {
    fn play(&mut self, samples: &[f32]) -> u64 {
        AudioOut::play(self, samples)
    }
    fn consumed(&self) -> u64 {
        AudioOut::consumed(self)
    }
    fn flush(&mut self) {
        AudioOut::flush(self)
    }
}

/// A transmission in progress.
struct Tx {
    /// `consumed()` value at which the burst has left the queue.
    end: u64,
    /// When that happened (input clock), once it has.
    drained_at: Option<f64>,
    /// Watchdog: unkey no matter what after this time.
    deadline: f64,
}

/// Station + PTT + audio output, stepped once per received audio chunk.
pub struct RadioEngine<S: AudioSink> {
    pub station: Station,
    ptt: Box<dyn Ptt>,
    out: S,
    /// Transmit level multiplier (default 1).
    pub tx_gain: f32,
    /// Seconds PTT stays on after the last sample has left the queue.
    tail_s: f64,
    rx_pos: u64,
    tx: Option<Tx>,
    tx_count: u64,
    /// Save every transmitted burst as a WAV file in this folder.
    pub record: Option<PathBuf>,
    /// Log lines (one per burst sent or received), drained by the caller.
    pub log: Vec<String>,
    /// Lines to show even when the log is quiet (level meter and warnings).
    pub notices: Vec<String>,
    /// Put a receive level line in `notices` every [`LEVEL_WINDOW_S`] seconds while idle.
    pub show_levels: bool,
    /// What happened, for the monitor API, drained by the caller. Times are on the receive
    /// clock ([`rx_samples`](Self::rx_samples)).
    pub events: Vec<ApiEvent>,
    meter: LevelMeter,
    /// When the last level warning went out.
    last_level_warning: Option<f64>,
    /// The busy state last put in `events`.
    busy: bool,
    /// The receiver's position at the last gap, and how far the receive clock was ahead of it
    /// before the gap: a burst that ended there is timed by that.
    before_gap: Option<(u64, u64)>,
}

impl<S: AudioSink> RadioEngine<S> {
    pub fn new(modem: Arc<Modem>, link: LinkConfig, ptt: Box<dyn Ptt>, out: S, txtail_s: f32, seed: u32) -> Self {
        Self {
            station: Station::new("local", modem, link, seed),
            ptt,
            out,
            tx_gain: 1.0,
            tail_s: txtail_s as f64,
            rx_pos: 0,
            tx: None,
            tx_count: 0,
            record: None,
            log: Vec::new(),
            notices: Vec::new(),
            show_levels: false,
            events: Vec::new(),
            meter: LevelMeter::new((LEVEL_WINDOW_S * RATE as f32) as usize),
            last_level_warning: None,
            busy: false,
            before_gap: None,
        }
    }

    /// An engine set up as `cfg` asks: the link's limits and hop margin, gain, logging
    /// and the digipeater's call signs. Fails if `cfg` does not pass [`RadioConfig::check`].
    pub fn from_config(cfg: &RadioConfig, modem: Arc<Modem>, ptt: Box<dyn Ptt>, out: S, seed: u32) -> io::Result<Self> {
        cfg.check()?;
        let link = LinkConfig {
            max_level: cfg.max_level,
            start_level: cfg.max_level.min(1),
            hop_margin_s: cfg.hop_margin_s,
            ..LinkConfig::default()
        };
        let mut engine = Self::new(modem, link, ptt, out, cfg.txtail_s, seed);
        engine.tx_gain = cfg.tx_gain;
        engine.show_levels = cfg.show_levels;
        engine.record = cfg.record.clone();
        engine.station.set_digi(&cfg.digi_calls);
        Ok(engine)
    }

    /// Warn about a bad receive level, at most every [`LEVEL_WARN_EVERY_S`] seconds.
    fn level_warning(&mut self, now: f64, what: &str, level: AudioLevel) {
        let verdict = Verdict::of(level);
        if !matches!(verdict, Verdict::Clipping | Verdict::Loud | Verdict::Quiet) {
            return;
        }
        if self.last_level_warning.is_some_and(|t| now - t < LEVEL_WARN_EVERY_S) {
            return;
        }
        self.last_level_warning = Some(now);
        let text = format!("{what} peaks at {:.1} dBFS: {} (aim for about -6 dBFS)", level.peak_dbfs, verdict.advice());
        self.notices.push(format!("{now:8.2}s {text}"));
        self.warning("rx_level", text);
    }

    fn warning(&mut self, code: &str, text: String) {
        self.events.push(ApiEvent::Warning { code: code.into(), text });
    }

    /// A log line that is also a warning for the monitor API.
    fn log_warning(&mut self, now: f64, code: &str, text: String) {
        self.log.push(format!("{now:8.2}s {text}"));
        self.warning(code, text);
    }

    pub fn now(&self) -> f64 {
        self.rx_pos as f64 / RATE as f64
    }

    /// Samples received so far: the receive clock. It counts the audio heard while
    /// transmitting and the audio lost (see [`audio_lost`](Self::audio_lost)) too.
    pub fn rx_samples(&self) -> u64 {
        self.rx_pos
    }

    /// `samples` of received audio were lost before the next chunk: the receive clock
    /// skips them, and `events` gets a `Gap`.
    pub fn audio_lost(&mut self, samples: u64) {
        if samples == 0 {
            return;
        }
        let heard = self.station.rx_position();
        self.before_gap = Some((heard, self.rx_pos - heard));
        self.rx_pos += samples;
        let now = self.now();
        self.log.push(format!(
            "{now:8.2}s lost {:.0} ms of received audio: the computer fell behind",
            samples as f64 * 1000.0 / RATE as f64
        ));
        self.events.push(ApiEvent::Gap { t: WireTime(self.rx_pos) });
    }

    pub fn transmitting(&self) -> bool {
        self.tx.is_some()
    }

    /// Advance with one chunk of received audio (or `None` if the sound card had
    /// nothing) and the host input since the last step. Returns host output.
    pub fn step(&mut self, chunk: Option<&[f32]>, host_in: Vec<HostIn>) -> Vec<HostOut> {
        if let Some(c) = chunk {
            self.rx_pos += c.len() as u64;
        }
        let now = self.now();
        for msg in host_in {
            match msg {
                HostIn::Line(l) => self.station.host_line(now, &l),
                HostIn::Data(d) => self.station.host_data(&d),
                HostIn::CmdClosed => self.station.host_closed(now),
            }
        }
        self.station.tick(now);

        if let Some(tx) = &mut self.tx {
            if tx.drained_at.is_none() && self.out.consumed() >= tx.end {
                tx.drained_at = Some(now);
            }
            let done = tx.drained_at.is_some_and(|t| now >= t + self.tail_s);
            if done || now > tx.deadline {
                if !done {
                    self.log_warning(now, "tx_watchdog", "watchdog: transmission overran, unkeying".into());
                    self.out.flush();
                }
                if let Err(e) = self.ptt.set(false) {
                    self.log_warning(now, "ptt", format!("PTT off failed: {e}"));
                }
                self.tx = None;
                self.station.tx_done(now);
            }
        } else {
            if let Some(c) = chunk {
                let bursts = self.station.rx_audio(now, c);
                // The receiver counts only the audio it was given; since the last time it was
                // not given some (while transmitting, or lost), the receive clock is a fixed
                // offset ahead. A burst is decoded a little after it ends, so one that ended just
                // before a gap may come out after it, and takes the offset from before.
                let (now_ahead, before_gap) = (self.rx_pos - self.station.rx_position(), self.before_gap);
                let offset = |end: u64| match before_gap {
                    Some((at, before)) if end <= at => before,
                    _ => now_ahead,
                };
                for b in bursts {
                    let ok = b.payload.iter().filter(|p| p.is_some()).count();
                    let h = b.header;
                    let end = b.end_sample + offset(b.end_sample);
                    self.events.push(ApiEvent::VfmBurstRx(VfmBurstRx {
                        t: WireTime(end.saturating_sub(body_samples(self.station.modem(), &h))),
                        t_end: WireTime(end),
                        kind: format!("{:?}", h.kind),
                        level: (h.ncw > 0).then_some(h.level),
                        ncw: h.ncw,
                        cw_ok: ok as u8,
                        snr_db: b.snr_db,
                        peak_dbfs: b.level.peak_dbfs,
                    }));
                    self.log.push(format!(
                        "{now:8.2}s rx {:<10} cw {ok:>2}/{:<2} SNR {:5.1} dB  peak {:5.1} dBFS",
                        format!("{:?}", b.header.kind),
                        b.header.ncw,
                        b.snr_db,
                        b.level.peak_dbfs
                    ));
                    self.level_warning(now, "received burst", b.level);
                }
                let busy = self.station.busy();
                if busy != self.busy {
                    self.busy = busy;
                    self.events.push(ApiEvent::VfmBusy { t: WireTime(self.rx_pos), busy });
                }
                if let Some(level) = self.meter.push(c) {
                    if self.show_levels {
                        self.notices.push(format!("{now:8.2}s {}", meter_line(level)));
                    } else if Verdict::of(level) == Verdict::Clipping {
                        // Noise or voice that clips will clip our bursts too.
                        self.level_warning(now, "receive audio", level);
                    }
                }
            }
            if let Some(t) = self.station.poll_tx(now) {
                let h = t.header;
                self.log.push(format!(
                    "{now:8.2}s {} {:<10} {:<9} cw {:>2} {:>5} B (resent {:>4}) retry {} air {:.2} s",
                    if t.repeat { "rpt" } else { "tx " },
                    format!("{:?}", h.kind),
                    if h.ncw > 0 { LEVELS[h.level as usize].name } else { "-" },
                    h.ncw,
                    t.info.bytes,
                    t.info.resent_bytes,
                    t.info.retry,
                    t.seconds
                ));
                self.events.push(ApiEvent::VfmBurstTx(VfmBurstTx {
                    t: WireTime(self.rx_pos),
                    kind: format!("{:?}", h.kind),
                    level: (h.ncw > 0).then_some(h.level),
                    ncw: h.ncw,
                    bytes: u32::try_from(t.info.bytes).unwrap_or(u32::MAX),
                    resent_bytes: u32::try_from(t.info.resent_bytes).unwrap_or(u32::MAX),
                    retry: t.info.retry,
                    air_s: t.seconds,
                    repeat: t.repeat,
                }));
                self.meter.reset();
                // Key first, then queue the audio: the leader covers the radio's key-up time.
                if let Err(e) = self.ptt.set(true) {
                    self.log_warning(now, "ptt", format!("PTT on failed: {e}"));
                }
                let audio: Vec<f32> = t.audio.iter().map(|x| x * self.tx_gain).collect();
                if let Some(d) = &self.record {
                    let _ = write_wav(&d.join(format!("{:04}_tx.wav", self.tx_count)), &audio);
                }
                self.tx_count += 1;
                let end = self.out.play(&audio);
                self.tx = Some(Tx { end, drained_at: None, deadline: now + t.seconds as f64 + 3.0 });
            }
        }
        self.station.take_output()
    }

    /// Release PTT (on shutdown).
    pub fn unkey(&mut self) {
        self.out.flush();
        let _ = self.ptt.set(false);
    }
}

/// Samples of a received burst after the sender's leader, which the receiver does not see the
/// start of: the two sync symbols, the header and the payload.
fn body_samples(modem: &Modem, h: &Header) -> u64 {
    let payload = if h.ncw > 0 { h.ncw as usize * modem.symbols_per_codeword(h.level) } else { 0 };
    ((2 + modem.header_symbols() + payload) * modem.ofdm.p.symbol_len()) as u64
}

/// Run until `stop` is set. Returns an error if a device cannot be opened.
pub fn run(cfg: RadioConfig, stop: Arc<AtomicBool>) -> io::Result<()> {
    // Before any device is opened.
    cfg.check()?;
    let params = OfdmParams::by_name(&cfg.mode).ok_or_else(|| io::Error::other("mode must be narrow or wide"))?;
    let modem = Arc::new(Modem::new(params, cfg.txdelay_s));
    let ptt = vardefm_ptt::open(&cfg.ptt)?;
    let input = open_input(cfg.audio_in.as_deref(), cfg.in_channel)?;
    let output = open_output(cfg.audio_out.as_deref(), cfg.out_channel)?;
    let port = HostPort::bind(&cfg.host, cfg.port, cfg.port + 1)?;
    if let Some(d) = &cfg.record {
        std::fs::create_dir_all(d)?;
    }
    println!("VardeFM {} mode", cfg.mode);
    println!("  audio in : {} ({} Hz{})", input.name, input.device_rate, resample_note(input.device_rate));
    println!("  audio out: {} ({} Hz{})", output.name, output.device_rate, resample_note(output.device_rate));
    println!("  PTT      : {}", ptt.describe());
    if (cfg.max_level as usize) < LEVELS.len() - 1 {
        println!("  max level: {} ({})", cfg.max_level, LEVELS[cfg.max_level as usize].name);
    }
    println!("  host     : command {}, data {}", port.cmd_addr, port.data_addr);
    if !cfg.digi_calls.is_empty() {
        println!("  digi     : repeating for {}", cfg.digi_calls.join(", "));
    }

    if cfg.show_levels {
        println!("  levels   : every {LEVEL_WINDOW_S} s while idle; set the radio volume so bursts peak near -6 dBFS");
    }
    let mut monitor = cfg.api.map(|addr| Monitor::start(addr, &cfg.mode)).transpose()?;
    if let Some(m) = &monitor {
        let a = m.local_addr();
        println!("  api      : http://{a}/v1/status, ws://{a}/v1/stream");
    }

    let mut engine = RadioEngine::from_config(&cfg, modem, ptt, output, link_seed())?;
    while !stop.load(Ordering::Relaxed) {
        let chunk = input.rx.recv_timeout(Duration::from_millis(50)).ok();
        if let Some(c) = &chunk {
            engine.audio_lost(c.lost_before);
            if let Some(m) = &mut monitor {
                m.audio(engine.rx_samples(), c.lost_before > 0, &c.samples);
            }
        }
        let host_in: Vec<HostIn> = port.inbox.try_iter().collect();
        let started = Instant::now();
        let out = engine.step(chunk.as_ref().map(|c| &c.samples[..]), host_in);
        match &mut monitor {
            Some(m) => {
                m.modem_step(started.elapsed());
                m.publish(engine.events.drain(..));
                if m.status_due() {
                    m.publish_status(
                        engine.rx_samples(),
                        &engine.station.link_view(),
                        input.overruns.load(Ordering::Acquire),
                        input.lost_samples.load(Ordering::Relaxed),
                    );
                }
            }
            None => engine.events.clear(),
        }
        for o in out {
            match o {
                HostOut::Line(l) => port.send_line(&l),
                HostOut::Data(d) => port.send_data(&d),
            }
        }
        for l in engine.log.drain(..) {
            if cfg.verbose || l.contains("failed") || l.contains("watchdog") {
                println!("{l}");
            }
        }
        for l in engine.notices.drain(..) {
            println!("{l}");
        }
    }
    engine.unkey();
    println!("Stopped; PTT released.");
    Ok(())
}

/// Seed for the link's session ids and retry delays. Two stations with the same seed draw
/// the same ids, and the process id alone repeats between stations started from one image.
fn link_seed() -> u32 {
    use std::hash::{BuildHasher, Hasher};
    // RandomState is keyed from the operating system's randomness.
    let mut h = std::collections::hash_map::RandomState::new().build_hasher();
    h.write_u32(std::process::id());
    if let Ok(t) = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH) {
        h.write_u128(t.as_nanos());
    }
    let x = h.finish();
    (x ^ (x >> 32)) as u32
}

fn resample_note(rate: u32) -> &'static str {
    if rate == RATE {
        ""
    } else {
        ", resampled to 48000"
    }
}

fn write_wav(path: &std::path::Path, samples: &[f32]) -> Result<(), hound::Error> {
    let spec =
        hound::WavSpec { channels: 1, sample_rate: RATE, bits_per_sample: 16, sample_format: hound::SampleFormat::Int };
    let mut w = hound::WavWriter::create(path, spec)?;
    for &v in samples {
        w.write_sample((v.clamp(-1.0, 1.0) * 32767.0) as i16)?;
    }
    w.finalize()
}

#[cfg(test)]
mod tests;
