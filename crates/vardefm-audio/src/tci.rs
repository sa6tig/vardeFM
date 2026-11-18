//! Receive audio over TCI (Transceiver Control Interface), as served by wfview or ExpertSDR.
//!
//! - TCI is a WebSocket (`ws://HOST:PORT/`, no TLS). Text commands end in `;`.
//!   Binary frames carry audio: a 64-byte little-endian header of 32-bit words, then samples.
//! - Only RX audio is used here: 48 kHz float32 stereo, receiver 0. Other stream types
//!   (wfview sends a TX_CHRONO packet before every RX audio packet) and receivers are skipped.
//! - wfview always sends 48 kHz float32 stereo and ignores the format commands, and it
//!   leaves the header's `channels` and `count` words unset. So only the receiver, rate,
//!   format, length and type words are read.
//! - wfview streams audio only when its Audio System is TCI, the radio was connected after
//!   that setting changed, and the rig sends its audio over the network (not USB).
//! - A dropped connection is retried with a backoff. The time lost is filled with zeros, so
//!   the sample count (the modem's clock) stays close to wall time.
//!
//! The TCI port has no authentication: anyone who can reach it can key the radio.

use crate::{err, Channel, RATE};
use std::io;
use std::net::{Shutdown, TcpStream, ToSocketAddrs};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{self, sync_channel, Receiver, RecvTimeoutError, Sender, SyncSender, TryRecvError, TrySendError};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};
use tungstenite::{Message, WebSocket};

/// wfview's default TCI address.
pub const DEFAULT_ADDR: &str = "127.0.0.1:50001";

const HEADER_BYTES: usize = 64;
const TYPE_RX_AUDIO: u32 = 1;
const FORMAT_FLOAT32: u32 = 3;
const MIN_BACKOFF: Duration = Duration::from_millis(100);
const MAX_BACKOFF: Duration = Duration::from_secs(1);
/// Longest gap filled with zeros after a reconnect.
const MAX_GAP_FRAMES: usize = 10 * RATE as usize;
/// Frames per chunk of gap zeros (one wfview packet).
const GAP_CHUNK: usize = 2048;

type Ws = WebSocket<TcpStream>;

/// The TCI address in an `--audio-in` value: `tci` or `tci:HOST:PORT` (any case).
pub fn addr_from_name(name: &str) -> Option<&str> {
    if name.eq_ignore_ascii_case("tci") {
        return Some(DEFAULT_ADDR);
    }
    let rest = name.get(..4).filter(|p| p.eq_ignore_ascii_case("tci:")).map(|_| &name[4..])?;
    Some(if rest.is_empty() { DEFAULT_ADDR } else { rest })
}

/// Time limits; tests shorten them.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Timing {
    /// TCP connect, and each read during the WebSocket handshake.
    pub connect: Duration,
    /// Wait for the first audio packet after opening.
    pub first: Duration,
    /// Silence after which a connection counts as lost.
    pub read: Duration,
}

impl Default for Timing {
    fn default() -> Self {
        Self { connect: Duration::from_secs(2), first: Duration::from_secs(5), read: Duration::from_secs(5) }
    }
}

/// One binary frame, as seen by the receive path.
#[derive(Debug, PartialEq)]
enum Packet {
    /// Another stream type or receiver.
    Skip,
    /// Mono samples from the chosen channel; `nonfinite` if any were NaN or infinite (now 0).
    Audio { mono: Vec<f32>, nonfinite: bool },
    /// RX audio that cannot be used, standing for `frames` frames at 48 kHz.
    Bad { frames: usize, why: String },
}

fn word(b: &[u8], i: usize) -> u32 {
    u32::from_le_bytes(b[4 * i..4 * i + 4].try_into().unwrap())
}

fn format_name(f: u32) -> String {
    match f {
        0 => "int16".into(),
        1 => "int24".into(),
        2 => "int32".into(),
        3 => "float32".into(),
        _ => format!("sample format {f}"),
    }
}

fn parse_packet(b: &[u8], channel: Channel) -> Packet {
    if b.len() < HEADER_BYTES {
        return Packet::Bad { frames: 0, why: format!("a {}-byte packet, shorter than the header", b.len()) };
    }
    let (receiver, rate, format, length, kind) = (word(b, 0), word(b, 1), word(b, 2), word(b, 5), word(b, 6));
    if kind != TYPE_RX_AUDIO || receiver != 0 {
        return Packet::Skip;
    }
    let bytes_per_sample = match format {
        0 => 2,
        1 => 3,
        _ => 4,
    };
    let body = &b[HEADER_BYTES..];
    // `length` counts samples (wfview: floats, not frames). Trust only the bytes present.
    let samples = (length as usize).min(body.len() / bytes_per_sample);
    let frames = samples / 2;
    if rate != RATE || format != FORMAT_FLOAT32 {
        let frames = if rate == 0 { frames } else { (frames as u64 * RATE as u64 / rate as u64) as usize };
        return Packet::Bad { frames, why: format!("{rate} Hz {}", format_name(format)) };
    }
    let mut nonfinite = false;
    let mut sample = |i: usize| {
        let x = f32::from_le_bytes(body[4 * i..4 * i + 4].try_into().unwrap());
        if x.is_finite() {
            x
        } else {
            nonfinite = true;
            0.0
        }
    };
    let mono = (0..frames)
        .map(|f| match channel {
            Channel::Left => sample(2 * f),
            Channel::Right => sample(2 * f + 1),
            Channel::Both => 0.5 * (sample(2 * f) + sample(2 * f + 1)),
        })
        .collect();
    Packet::Audio { mono, nonfinite }
}

fn is_timeout(e: &tungstenite::Error) -> bool {
    matches!(e, tungstenite::Error::Io(e) if matches!(e.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut))
}

/// Connect, handshake and ask for RX audio. Reads time out after `timing.read`.
fn connect(addr: &str, timing: Timing) -> io::Result<Ws> {
    let sa = addr
        .to_socket_addrs()
        .map_err(|e| err(format!("TCI address {addr}: {e}")))?
        .next()
        .ok_or_else(|| err(format!("TCI address {addr} resolves to nothing")))?;
    let tcp = TcpStream::connect_timeout(&sa, timing.connect)
        .map_err(|e| err(format!("cannot reach the TCI server at {addr}: {e}")))?;
    tcp.set_nodelay(true)?;
    tcp.set_read_timeout(Some(timing.connect))?;
    tcp.set_write_timeout(Some(timing.connect))?;
    let (mut ws, _) = tungstenite::client(format!("ws://{addr}/"), tcp)
        .map_err(|e| err(format!("TCI handshake with {addr}: {e}")))?;
    ws.get_ref().set_read_timeout(Some(timing.read))?;
    // wfview ignores the first three (it always sends 48 kHz float32 stereo); ExpertSDR honours them.
    for cmd in
        ["audio_samplerate:48000;", "audio_stream_sample_type:float32;", "audio_stream_channels:2;", "audio_start:0;"]
    {
        ws.send(Message::text(cmd)).map_err(|e| err(format!("TCI {addr}: {e}")))?;
    }
    Ok(ws)
}

fn no_audio(addr: &str, wait: Duration) -> io::Error {
    err(format!(
        "no audio from the TCI server at {addr} within {:.0} s. In wfview, set the Audio System to TCI, \
         then connect the radio again. The radio must send its audio over the network (Icom LAN or Wi-Fi, \
         not USB).",
        wait.as_secs_f32()
    ))
}

/// Wait for the first RX audio packet.
fn first_packet(ws: &mut Ws, addr: &str, channel: Channel, wait: Duration) -> io::Result<(Vec<f32>, bool)> {
    let deadline = Instant::now() + wait;
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        if left.is_zero() {
            return Err(no_audio(addr, wait));
        }
        ws.get_ref().set_read_timeout(Some(left))?;
        match ws.read() {
            Ok(Message::Binary(b)) => match parse_packet(&b, channel) {
                Packet::Skip => {}
                Packet::Audio { mono, nonfinite } => return Ok((mono, nonfinite)),
                Packet::Bad { why, .. } => {
                    return Err(err(format!("the TCI server at {addr} sends {why}; VardeFM needs 48000 Hz float32")))
                }
            },
            Ok(Message::Close(_)) => return Err(err(format!("the TCI server at {addr} closed the connection"))),
            Ok(_) => {}
            Err(e) if is_timeout(&e) => return Err(no_audio(addr, wait)),
            Err(e) => return Err(err(format!("TCI {addr}: {e}"))),
        }
    }
}

/// The live socket, shared so that [`Reader`]'s drop can unblock the worker's read.
struct Shared {
    sock: Mutex<Option<TcpStream>>,
}

/// Owns the reader thread. Dropping it stops the thread and closes the connection.
pub(crate) struct Reader {
    /// Dropped first on shutdown: the worker sees the disconnect and stops.
    stop: Option<Sender<()>>,
    shared: Arc<Shared>,
    thread: Option<JoinHandle<()>>,
}

impl Drop for Reader {
    fn drop(&mut self) {
        self.stop.take();
        if let Some(s) = self.shared.sock.lock().unwrap().take() {
            let _ = s.shutdown(Shutdown::Both);
        }
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

fn stopped(stop: &mpsc::Receiver<()>) -> bool {
    matches!(stop.try_recv(), Err(TryRecvError::Disconnected))
}

/// Keep a handle to `ws`'s socket for shutdown. False if the reader is being dropped.
fn store(shared: &Shared, stop: &mpsc::Receiver<()>, ws: &Ws) -> bool {
    let mut g = shared.sock.lock().unwrap();
    // Checked under the lock, so no socket is stored after drop has taken the old one.
    if stopped(stop) {
        return false;
    }
    *g = ws.get_ref().try_clone().ok();
    true
}

/// Opened TCI input: the reader and its outputs, for [`crate::AudioIn`].
pub(crate) struct Opened {
    pub reader: Reader,
    pub rx: Receiver<Vec<f32>>,
    pub overruns: Arc<AtomicU64>,
}

/// Connect, wait for the first audio packet, then stream on a thread.
pub(crate) fn open(addr: &str, channel: Channel, timing: Timing) -> io::Result<Opened> {
    let mut ws = connect(addr, timing)?;
    let (first, nonfinite) = first_packet(&mut ws, addr, channel, timing.first)?;
    ws.get_ref().set_read_timeout(Some(timing.read))?;
    let (tx, rx) = sync_channel(256);
    let overruns = Arc::new(AtomicU64::new(nonfinite as u64));
    let _ = tx.try_send(first);
    let (stop_tx, stop_rx) = mpsc::channel();
    let shared = Arc::new(Shared { sock: Mutex::new(ws.get_ref().try_clone().ok()) });
    let worker = Worker {
        addr: addr.to_string(),
        channel,
        timing,
        tx,
        overruns: overruns.clone(),
        shared: shared.clone(),
        stop: stop_rx,
    };
    let thread = std::thread::Builder::new().name("tci-audio".into()).spawn(move || worker.run(ws))?;
    Ok(Opened { reader: Reader { stop: Some(stop_tx), shared, thread: Some(thread) }, rx, overruns })
}

struct Worker {
    addr: String,
    channel: Channel,
    timing: Timing,
    tx: SyncSender<Vec<f32>>,
    overruns: Arc<AtomicU64>,
    shared: Arc<Shared>,
    stop: mpsc::Receiver<()>,
}

impl Worker {
    /// Queue a chunk. False if `AudioIn` is gone.
    fn send(&self, chunk: Vec<f32>) -> bool {
        match self.tx.try_send(chunk) {
            Ok(()) => true,
            Err(TrySendError::Full(_)) => {
                self.overruns.fetch_add(1, Ordering::Relaxed);
                true
            }
            Err(TrySendError::Disconnected(_)) => false,
        }
    }

    fn run(self, ws: Ws) {
        let mut ws = Some(ws);
        // Arrival of the latest packet; the first one arrived just before this thread started.
        let mut last = Instant::now();
        let mut lost = false;
        let mut backoff = MIN_BACKOFF;
        loop {
            let Some(w) = ws.as_mut() else {
                match self.stop.recv_timeout(backoff) {
                    Err(RecvTimeoutError::Timeout) => {}
                    _ => return,
                }
                match connect(&self.addr, self.timing) {
                    Ok(w) => {
                        if !store(&self.shared, &self.stop, &w) {
                            return;
                        }
                        ws = Some(w);
                        backoff = MIN_BACKOFF;
                    }
                    Err(_) => backoff = (backoff * 2).min(MAX_BACKOFF),
                }
                continue;
            };
            let chunk = match w.read() {
                Ok(Message::Binary(b)) => match parse_packet(&b, self.channel) {
                    Packet::Skip => continue,
                    Packet::Audio { mono, nonfinite } => {
                        if nonfinite {
                            self.overruns.fetch_add(1, Ordering::Relaxed);
                        }
                        mono
                    }
                    Packet::Bad { frames, .. } => {
                        self.overruns.fetch_add(1, Ordering::Relaxed);
                        vec![0.0; frames]
                    }
                },
                Ok(Message::Close(_)) | Err(_) => {
                    if !lost {
                        eprintln!("TCI audio from {} lost; reconnecting", self.addr);
                    }
                    lost = true;
                    ws = None;
                    self.shared.sock.lock().unwrap().take();
                    continue;
                }
                Ok(_) => continue,
            };
            let now = Instant::now();
            if lost {
                // Count the gap from arrival times, only across a reconnect: within one
                // connection, network jitter would show up as false gaps.
                lost = false;
                let span = (now - last).as_secs_f64() * RATE as f64;
                let gap = (span as i64 - chunk.len() as i64).clamp(1, MAX_GAP_FRAMES as i64) as usize;
                eprintln!("TCI audio from {} back; {} ms filled with silence", self.addr, gap * 1000 / RATE as usize);
                for n in (0..gap).step_by(GAP_CHUNK) {
                    if !self.send(vec![0.0; GAP_CHUNK.min(gap - n)]) {
                        return;
                    }
                }
            }
            last = now;
            if !chunk.is_empty() && !self.send(chunk) {
                return;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpListener;

    /// A wfview-style packet: junk in `channels` and `count`, `length` in samples.
    fn packet(kind: u32, receiver: u32, rate: u32, format: u32, frames: &[(f32, f32)]) -> Vec<u8> {
        let mut words = [0u32; 16];
        words[0] = receiver;
        words[1] = rate;
        words[2] = format;
        words[5] = 2 * frames.len() as u32;
        words[6] = kind;
        words[7] = 0xdead_beef;
        words[8] = 0x1234_5678;
        let mut b: Vec<u8> = words.iter().flat_map(|w| w.to_le_bytes()).collect();
        for &(l, r) in frames {
            b.extend(l.to_le_bytes());
            b.extend(r.to_le_bytes());
        }
        b
    }

    fn rx_audio(frames: &[(f32, f32)]) -> Vec<u8> {
        packet(TYPE_RX_AUDIO, 0, RATE, FORMAT_FLOAT32, frames)
    }

    /// Send RX audio the way wfview does: a TX_CHRONO packet first.
    fn send_audio(ws: &mut Ws, frames: &[(f32, f32)]) {
        ws.send(Message::binary(packet(3, 0, RATE, FORMAT_FLOAT32, &[(9.0, 9.0); 4]))).unwrap();
        ws.send(Message::binary(rx_audio(frames))).unwrap();
    }

    /// Read text commands up to and including `audio_start`.
    fn commands(ws: &mut Ws) -> Vec<String> {
        let mut got = Vec::new();
        while !got.last().is_some_and(|c: &String| c.starts_with("audio_start")) {
            if let Message::Text(t) = ws.read().unwrap() {
                got.push(t.to_string());
            }
        }
        got
    }

    /// Block until the client goes away.
    fn hold(ws: &mut Ws) {
        while ws.read().is_ok() {}
    }

    /// A fake TCI server running `script` on each accepted connection, in turn.
    fn server(script: impl Fn(usize, &mut Ws) + Send + 'static) -> String {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        std::thread::spawn(move || {
            for (n, s) in listener.incoming().enumerate() {
                let Ok(mut ws) = tungstenite::accept(s.unwrap()) else { continue };
                script(n, &mut ws);
            }
        });
        addr
    }

    fn fast() -> Timing {
        Timing {
            connect: Duration::from_millis(500),
            first: Duration::from_millis(300),
            read: Duration::from_millis(300),
        }
    }

    fn open_err(addr: &str, timing: Timing) -> String {
        match open(addr, Channel::Left, timing) {
            Ok(_) => panic!("open succeeded"),
            Err(e) => e.to_string(),
        }
    }

    /// Chunks until `n` samples have arrived.
    fn take(rx: &Receiver<Vec<f32>>, n: usize) -> Vec<Vec<f32>> {
        let mut out = Vec::new();
        let mut got = 0;
        while got < n {
            let c = rx.recv_timeout(Duration::from_secs(5)).expect("audio");
            got += c.len();
            out.push(c);
        }
        out
    }

    #[test]
    fn names() {
        assert_eq!(addr_from_name("tci"), Some(DEFAULT_ADDR));
        assert_eq!(addr_from_name("TCI:"), Some(DEFAULT_ADDR));
        assert_eq!(addr_from_name("Tci:10.0.0.5:50001"), Some("10.0.0.5:50001"));
        assert_eq!(addr_from_name("USB Audio"), None);
        assert_eq!(addr_from_name("tcix"), None);
        assert_eq!(addr_from_name("tc"), None);
    }

    #[test]
    fn parse_bounds_length_and_skips() {
        let mut p = rx_audio(&[(0.1, 0.2), (0.3, 0.4)]);
        p[20..24].copy_from_slice(&100u32.to_le_bytes()); // length beyond the bytes present
        assert_eq!(parse_packet(&p, Channel::Left), Packet::Audio { mono: vec![0.1, 0.3], nonfinite: false });
        p.truncate(p.len() - 4); // half a frame: rounded down
        assert_eq!(parse_packet(&p, Channel::Right), Packet::Audio { mono: vec![0.2], nonfinite: false });
        assert_eq!(parse_packet(&packet(3, 0, RATE, 3, &[(0.0, 0.0)]), Channel::Left), Packet::Skip);
        assert_eq!(parse_packet(&packet(1, 1, RATE, 3, &[(0.0, 0.0)]), Channel::Left), Packet::Skip);
        assert!(matches!(parse_packet(&[0; 10], Channel::Left), Packet::Bad { frames: 0, .. }));
    }

    #[test]
    fn commands_channels_and_skipped_packets() {
        let (cmd_tx, cmd_rx) = mpsc::channel();
        let addr = server(move |_, ws| {
            cmd_tx.send(commands(ws)).unwrap();
            ws.send(Message::binary(packet(1, 1, RATE, 3, &[(7.0, 7.0)]))).unwrap(); // other receiver
            send_audio(ws, &[(0.1, 0.3), (0.2, 0.4)]);
            hold(ws);
        });
        for (ch, want) in
            [(Channel::Left, vec![0.1, 0.2]), (Channel::Right, vec![0.3, 0.4]), (Channel::Both, vec![0.2, 0.3])]
        {
            let o = open(&addr, ch, fast()).unwrap();
            let got = o.rx.recv_timeout(Duration::from_secs(1)).unwrap();
            assert!(got.iter().zip(&want).all(|(a, b)| (a - b).abs() < 1e-6), "{ch:?}: {got:?}");
            assert_eq!(
                cmd_rx.recv().unwrap(),
                [
                    "audio_samplerate:48000;",
                    "audio_stream_sample_type:float32;",
                    "audio_stream_channels:2;",
                    "audio_start:0;"
                ]
            );
        }
    }

    #[test]
    fn silent_server_fails_with_a_hint() {
        // TX_CHRONO only, as wfview sends when its Audio System is not TCI.
        let addr = server(|_, ws| {
            commands(ws);
            ws.send(Message::binary(packet(3, 0, RATE, 3, &[(0.0, 0.0)]))).unwrap();
            hold(ws);
        });
        let e = open_err(&addr, fast());
        assert!(e.contains("Audio System"), "{e}");
    }

    #[test]
    fn wrong_format_and_refused_connect() {
        let addr = server(|_, ws| {
            commands(ws);
            ws.send(Message::binary(packet(1, 0, 44_100, 3, &[(0.0, 0.0)]))).unwrap();
            hold(ws);
        });
        let e = open_err(&addr, fast());
        assert!(e.contains("44100 Hz float32"), "{e}");

        let free = TcpListener::bind("127.0.0.1:0").unwrap().local_addr().unwrap().to_string();
        let e = open_err(&free, fast());
        assert!(e.contains("cannot reach"), "{e}");
    }

    #[test]
    fn bad_packets_mid_stream() {
        let addr = server(|_, ws| {
            commands(ws);
            send_audio(ws, &[(0.5, 0.5); 10]);
            ws.send(Message::binary(packet(1, 0, RATE, 0, &[(0.0, 0.0); 8]))).unwrap(); // int16: 16 samples
            send_audio(ws, &[(f32::NAN, 0.0), (0.25, 0.0), (f32::INFINITY, 0.0)]);
            hold(ws);
        });
        // A long read timeout: a reconnect would replay the script and count its errors again.
        let o = open(&addr, Channel::Left, Timing { read: Duration::from_secs(5), ..fast() }).unwrap();
        let chunks = take(&o.rx, 8 + 10 + 3);
        assert_eq!(chunks[0], vec![0.5; 10]);
        // The int16 packet's `length` says 16 samples: a gap of 8 frames.
        assert_eq!(chunks[1], vec![0.0; 8]);
        assert_eq!(chunks[2], vec![0.0, 0.25, 0.0]);
        assert_eq!(o.overruns.load(Ordering::Relaxed), 2);
    }

    #[test]
    fn reconnect_fills_the_gap() {
        let addr = server(|n, ws| {
            commands(ws);
            if n == 0 {
                send_audio(ws, &[(0.5, 0.5); 2048]);
                send_audio(ws, &[(0.5, 0.5); 2048]);
                // Close: the client reconnects after its backoff.
            } else {
                std::thread::sleep(Duration::from_millis(200));
                send_audio(ws, &[(0.25, 0.25); 2048]);
                hold(ws);
            }
        });
        let start = Instant::now();
        let o = open(&addr, Channel::Left, fast()).unwrap();
        let mut all = Vec::new();
        while !all.contains(&0.25) {
            all.extend(o.rx.recv_timeout(Duration::from_secs(5)).expect("audio after reconnect"));
        }
        let elapsed = start.elapsed().as_secs_f64();
        assert!(all[..4096].iter().all(|&x| x == 0.5));
        let zeros = all[4096..].iter().take_while(|&&x| x == 0.0).count();
        assert_eq!(all.len(), 4096 + zeros + 2048, "one gap, then the new packet");
        // About backoff (100 ms) + server delay (200 ms), never more than the whole run.
        let ms = zeros as f64 * 1000.0 / RATE as f64;
        assert!(ms > 150.0 && ms < elapsed * 1000.0, "gap {ms:.0} ms, run {:.0} ms", elapsed * 1000.0);
    }

    #[test]
    fn drop_while_server_is_silent_returns_promptly() {
        let addr = server(|_, ws| {
            commands(ws);
            send_audio(ws, &[(0.1, 0.1)]);
            hold(ws);
        });
        // Real timing: the worker sits in a 5 s read when dropped.
        let o = open(&addr, Channel::Left, Timing::default()).unwrap();
        std::thread::sleep(Duration::from_millis(100));
        let t = Instant::now();
        drop(o);
        assert!(t.elapsed() < Duration::from_secs(1), "{:?}", t.elapsed());
    }

    #[test]
    fn modem_burst_decodes_through_tci() {
        use vardefm_modem::{Burst, Header, Kind, Modem, OfdmParams, Receiver as ModemRx};
        let modem = Arc::new(Modem::new(OfdmParams::by_name("narrow").unwrap(), 0.1));
        let n = Modem::payload_bytes(1);
        let burst = Burst {
            header: Header {
                kind: Kind::Data,
                session: 0x42,
                level: 1,
                ncw: 2,
                req_level: 1,
                burst_id: 3,
                ack_burst: 0,
                ack_offset: 0,
                bitmap: 0,
                flags: 0,
            },
            payload: (0..2).map(|c| (0..n).map(|i| (i * 7 + c) as u8).collect()).collect(),
        };
        let mut audio = vec![0.0; RATE as usize / 4];
        audio.extend(modem.modulate(&burst));
        audio.extend(vec![0.0; RATE as usize]);
        let total = audio.len();
        let addr = server(move |_, ws| {
            commands(ws);
            for c in audio.chunks(2048) {
                // The burst on the right channel; noise-free junk on the left.
                let frames: Vec<(f32, f32)> = c.iter().map(|&x| (0.9, x)).collect();
                send_audio(ws, &frames);
            }
            hold(ws);
        });
        let o = open(&addr, Channel::Right, fast()).unwrap();
        let mut rx = ModemRx::new(modem);
        let mut got = Vec::new();
        for c in take(&o.rx, total) {
            got.extend(rx.push(&c));
        }
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].header, burst.header);
        assert_eq!(got[0].payload, burst.payload.iter().cloned().map(Some).collect::<Vec<_>>());
    }
}
