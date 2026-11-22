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
//! - A connection that is closed, or brings no RX audio for 5 s (even if other packets
//!   still arrive), is dropped and retried with a backoff, without limit, until the input
//!   is dropped. While the audio is missing, zeros are sent at wall-clock pace (at least
//!   every 250 ms on an open connection), so the sample count (the modem's clock, which also
//!   times PTT release and the transmit watchdog) keeps moving. The clock still stands still
//!   for up to 5 s before the audio counts as missing, and for up to 3 s during each connect
//!   attempt. The zeros then catch up. The address is resolved once, when opening.
//! - On Windows a read that times out can make the next read fail (os error 997); here that
//!   only costs a reconnect while the audio is already missing.
//!
//! The TCI port has no authentication: anyone who can reach it can key the radio.

use crate::{err, Channel, RATE};
use std::io;
use std::net::{Shutdown, SocketAddr, TcpStream, ToSocketAddrs};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{self, sync_channel, Receiver, RecvTimeoutError, Sender, SyncSender, TryRecvError, TrySendError};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};
use tungstenite::handshake::HandshakeError;
use tungstenite::{Message, WebSocket};

/// wfview's default TCI address.
pub const DEFAULT_ADDR: &str = "127.0.0.1:50001";

const HEADER_BYTES: usize = 64;
const TYPE_RX_AUDIO: u32 = 1;
const FORMAT_FLOAT32: u32 = 3;
const MIN_BACKOFF: Duration = Duration::from_millis(100);
/// Longest wait between zero fills while the audio is missing on an open connection.
const FILL_TICK: Duration = Duration::from_millis(250);
/// How often a connection that stays down is reported.
const DOWN_NOTE_EVERY: Duration = Duration::from_secs(30);
/// Longest time one bad packet may stand for (a wfview packet is 43 ms).
const MAX_BAD_FRAMES: usize = RATE as usize;
/// Frames per chunk of filler zeros. Large, so catching up after a 5 s silence takes
/// about 10 chunks of the queue's 256.
const FILL_CHUNK: usize = RATE as usize / 2;

type Ws = WebSocket<TcpStream>;

/// The TCI address in an `--audio-in` value: `tci` (the default address) or `tci:HOST:PORT`
/// (any case). `tci:` alone gives an empty address, which [`open`] refuses.
pub fn addr_from_name(name: &str) -> Option<&str> {
    if name.eq_ignore_ascii_case("tci") {
        return Some(DEFAULT_ADDR);
    }
    name.get(..4).filter(|p| p.eq_ignore_ascii_case("tci:")).map(|_| &name[4..])
}

/// Time limits; tests shorten them.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Timing {
    /// One connect attempt: TCP connect, WebSocket handshake and the start commands.
    pub connect: Duration,
    /// Wait for the first audio packet after opening.
    pub first: Duration,
    /// Time without RX audio after which a connection counts as lost.
    pub read: Duration,
    /// Longest wait between reconnects (the backoff starts at 100 ms and doubles).
    pub max_backoff: Duration,
}

impl Default for Timing {
    fn default() -> Self {
        Self {
            connect: Duration::from_secs(3),
            first: Duration::from_secs(5),
            read: Duration::from_secs(5),
            max_backoff: Duration::from_secs(1),
        }
    }
}

/// One binary frame, as seen by the receive path.
#[derive(Debug, PartialEq)]
enum Packet {
    /// Another stream type or receiver.
    Skip,
    /// A frame shorter than the header.
    Short,
    /// Mono samples from the chosen channel. `damaged` if any were NaN or infinite (now 0),
    /// if the packet holds fewer samples than its `length` says (the rest are zeros), or if
    /// `length` is odd, so not whole stereo frames.
    Audio { mono: Vec<f32>, damaged: bool },
    /// RX audio that cannot be used, standing for `frames` frames at 48 kHz (at most 1 s).
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
        return Packet::Short;
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
    // `length` counts samples (wfview: floats, not frames). Stereo is assumed: the client asks
    // for 2 channels, and wfview always sends 2 but leaves the `channels` word unset, so it
    // cannot be checked. An odd `length` shows at least that it is not whole stereo frames.
    // Trust only the bytes present.
    let samples = (length as usize).min(body.len() / bytes_per_sample);
    let frames = samples / 2;
    // Frames the header promises but the packet lacks, as zeros (bounded like a bad packet).
    let missing = ((length as usize / 2).saturating_sub(frames)).min(MAX_BAD_FRAMES);
    if rate != RATE || format != FORMAT_FLOAT32 {
        let frames = if rate == 0 { frames } else { (frames as u64 * RATE as u64 / rate as u64) as usize };
        return Packet::Bad { frames: frames.min(MAX_BAD_FRAMES), why: format!("{rate} Hz {}", format_name(format)) };
    }
    let mut damaged = missing > 0 || length % 2 == 1;
    let mut sample = |i: usize| {
        let x = f32::from_le_bytes(body[4 * i..4 * i + 4].try_into().unwrap());
        if x.is_finite() {
            x
        } else {
            damaged = true;
            0.0
        }
    };
    let mut mono: Vec<f32> = (0..frames)
        .map(|f| match channel {
            Channel::Left => sample(2 * f),
            Channel::Right => sample(2 * f + 1),
            Channel::Both => 0.5 * (sample(2 * f) + sample(2 * f + 1)),
        })
        .collect();
    mono.resize(frames + missing, 0.0);
    Packet::Audio { mono, damaged }
}

fn is_timeout(e: &tungstenite::Error) -> bool {
    matches!(e, tungstenite::Error::Io(e) if matches!(e.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut))
}

/// Resolve a TCI address once, so that reconnects do not block on name lookups.
fn resolve(addr: &str) -> io::Result<Vec<SocketAddr>> {
    let addrs: Vec<SocketAddr> = addr.to_socket_addrs().map_err(|e| err(format!("TCI address {addr}: {e}")))?.collect();
    if addrs.is_empty() {
        return Err(err(format!("TCI address {addr} resolves to nothing")));
    }
    Ok(addrs)
}

/// Connect, handshake and ask for RX audio, all within `timing.connect`. Reads then time
/// out after `timing.read`. `on_tcp` sees the socket before the handshake, so it can be
/// shut down from outside.
fn connect(
    addr: &str,
    addrs: &[SocketAddr],
    timing: Timing,
    on_tcp: impl FnOnce(&TcpStream) -> io::Result<()>,
) -> io::Result<Ws> {
    let deadline = Instant::now() + timing.connect;
    let left = || {
        let d = deadline.saturating_duration_since(Instant::now());
        if d.is_zero() {
            Err(err(format!("TCI {addr}: no answer within {:.0} s", timing.connect.as_secs_f32())))
        } else {
            Ok(d)
        }
    };
    // Try every address: `localhost` may give `::1` first while the server listens on IPv4 only.
    let mut last = err(format!("TCI address {addr} resolves to nothing"));
    let mut tcp = None;
    for sa in addrs {
        match TcpStream::connect_timeout(sa, left()?) {
            Ok(s) => {
                tcp = Some(s);
                break;
            }
            Err(e) => last = err(format!("cannot reach the TCI server at {addr}: {e}")),
        }
    }
    let tcp = tcp.ok_or(last)?;
    on_tcp(&tcp)?;
    tcp.set_nodelay(true)?;
    tcp.set_read_timeout(Some(left()?))?;
    tcp.set_write_timeout(Some(left()?))?;
    // A read that times out interrupts the handshake; resume it with the time left.
    let mut hs = tungstenite::client(format!("ws://{addr}/"), tcp);
    let mut ws = loop {
        match hs {
            Ok((ws, _)) => break ws,
            Err(HandshakeError::Interrupted(mid)) => {
                mid.get_ref().get_ref().set_read_timeout(Some(left()?))?;
                hs = mid.handshake();
            }
            Err(HandshakeError::Failure(e)) => return Err(err(format!("TCI handshake with {addr}: {e}"))),
        }
    };
    // wfview ignores the first three (it always sends 48 kHz float32 stereo); ExpertSDR honours them.
    for cmd in
        ["audio_samplerate:48000;", "audio_stream_sample_type:float32;", "audio_stream_channels:2;", "audio_start:0;"]
    {
        ws.get_ref().set_write_timeout(Some(left()?))?;
        ws.send(Message::text(cmd)).map_err(|e| err(format!("TCI {addr}: {e}")))?;
    }
    ws.get_ref().set_read_timeout(Some(timing.read))?;
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
                Packet::Skip | Packet::Short => {}
                Packet::Audio { mono, damaged } => return Ok((mono, damaged)),
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
/// Dropping during a reconnect's TCP connect can take up to `Timing::connect` (3 s).
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

/// Opened TCI input: the reader and its outputs, for [`crate::AudioIn`].
pub(crate) struct Opened {
    pub reader: Reader,
    pub rx: Receiver<Vec<f32>>,
    pub overruns: Arc<AtomicU64>,
}

/// Connect, wait for the first audio packet, then stream on a thread.
pub(crate) fn open(addr: &str, channel: Channel, timing: Timing) -> io::Result<Opened> {
    if addr.is_empty() {
        return Err(err("no address after tci: (use tci alone for 127.0.0.1:50001, or tci:HOST:PORT)"));
    }
    let addrs = resolve(addr)?;
    let mut ws = connect(addr, &addrs, timing, |_| Ok(()))?;
    let (first, damaged) = first_packet(&mut ws, addr, channel, timing.first)?;
    ws.get_ref().set_read_timeout(Some(timing.read))?;
    let (tx, rx) = sync_channel(256);
    let overruns = Arc::new(AtomicU64::new(damaged as u64));
    let _ = tx.try_send(first);
    let (stop_tx, stop_rx) = mpsc::channel();
    let shared = Arc::new(Shared { sock: Mutex::new(Some(ws.get_ref().try_clone()?)) });
    let worker = Worker {
        addr: addr.to_string(),
        addrs,
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
    addrs: Vec<SocketAddr>,
    channel: Channel,
    timing: Timing,
    tx: SyncSender<Vec<f32>>,
    overruns: Arc<AtomicU64>,
    shared: Arc<Shared>,
    stop: mpsc::Receiver<()>,
}

/// What one read brought.
enum Step {
    /// Samples to queue (real audio, or zeros standing for a bad packet).
    Audio(Vec<f32>),
    /// A frame without RX audio.
    Nothing,
    /// The connection closed, failed or timed out.
    Down,
}

impl Worker {
    /// Queue a chunk. `Some(true)` if queued, `Some(false)` if the queue was full (an
    /// overrun), `None` if `AudioIn` is gone.
    fn send(&self, chunk: Vec<f32>) -> Option<bool> {
        match self.tx.try_send(chunk) {
            Ok(()) => Some(true),
            Err(TrySendError::Full(_)) => {
                self.overruns.fetch_add(1, Ordering::Relaxed);
                Some(false)
            }
            Err(TrySendError::Disconnected(_)) => None,
        }
    }

    /// Queue `n` zeros. Returns how many were queued, or `None` if `AudioIn` is gone.
    fn zeros(&self, n: usize) -> Option<usize> {
        let mut queued = 0;
        for i in (0..n).step_by(FILL_CHUNK) {
            let k = FILL_CHUNK.min(n - i);
            if self.send(vec![0.0; k])? {
                queued += k;
            }
        }
        Some(queued)
    }

    /// Queue zeros until `filled` covers the wall time `since` the last audio packet.
    /// Zeros dropped on a full queue are not counted, so they are sent again later.
    fn fill(&self, since: Duration, filled: &mut usize) -> Option<()> {
        let due = (since.as_secs_f64() * RATE as f64) as usize;
        *filled += self.zeros(due.saturating_sub(*filled))?;
        Some(())
    }

    /// Keep a new socket for [`Reader`]'s drop to shut down. Fails if the reader is being dropped.
    fn register(&self, tcp: &TcpStream) -> io::Result<()> {
        let mut g = self.shared.sock.lock().unwrap();
        // Checked under the lock, so no socket is stored after drop has taken the old one.
        if stopped(&self.stop) {
            return Err(err("TCI reader stopping"));
        }
        *g = Some(tcp.try_clone()?);
        Ok(())
    }

    fn read(&self, w: &mut Ws) -> Step {
        match w.read() {
            Ok(Message::Binary(b)) => match parse_packet(&b, self.channel) {
                Packet::Skip => Step::Nothing,
                Packet::Short => {
                    self.overruns.fetch_add(1, Ordering::Relaxed);
                    Step::Nothing
                }
                Packet::Audio { mono, damaged } => {
                    if damaged {
                        self.overruns.fetch_add(1, Ordering::Relaxed);
                    }
                    Step::Audio(mono)
                }
                Packet::Bad { frames, .. } => {
                    self.overruns.fetch_add(1, Ordering::Relaxed);
                    Step::Audio(vec![0.0; frames])
                }
            },
            // A read timeout only means the wait set by `run_inner` is over.
            Err(e) if is_timeout(&e) => Step::Nothing,
            Ok(Message::Close(_)) | Err(_) => Step::Down,
            Ok(_) => Step::Nothing,
        }
    }

    fn run(self, ws: Ws) {
        let _ = self.run_inner(ws);
    }

    /// Returns `None` when stopped or when `AudioIn` is gone.
    fn run_inner(&self, ws: Ws) -> Option<()> {
        let mut ws = Some(ws);
        // Arrival of the latest RX audio packet; the first one arrived just before this thread started.
        let mut last = Instant::now();
        // RX audio must arrive by then on the current connection, or it counts as lost.
        let mut audio_due = last + self.timing.read;
        // Zeros queued since `last`, while the audio was missing.
        let mut filled = 0usize;
        let mut lost = false;
        let mut backoff = MIN_BACKOFF;
        let mut last_err = String::new();
        // When to report next that the audio is still missing.
        let mut next_note = Instant::now();
        loop {
            if lost && Instant::now() >= next_note {
                eprintln!("TCI audio from {} still missing after {:.0} s", self.addr, last.elapsed().as_secs_f64());
                next_note += DOWN_NOTE_EVERY;
            }
            let Some(w) = ws.as_mut() else {
                self.fill(last.elapsed(), &mut filled)?;
                match self.stop.recv_timeout(backoff) {
                    Err(RecvTimeoutError::Timeout) => {}
                    _ => return None,
                }
                // Reset only by RX audio: a server that accepts and then sends none backs off too.
                backoff = (backoff * 2).min(self.timing.max_backoff);
                self.fill(last.elapsed(), &mut filled)?;
                match connect(&self.addr, &self.addrs, self.timing, |tcp| self.register(tcp)) {
                    Ok(w) => {
                        ws = Some(w);
                        audio_due = Instant::now() + self.timing.read;
                        last_err.clear();
                    }
                    Err(e) => {
                        if stopped(&self.stop) {
                            return None;
                        }
                        let msg = e.to_string();
                        if msg != last_err {
                            eprintln!("TCI reconnect failed: {msg}");
                            last_err = msg;
                        }
                    }
                }
                continue;
            };
            // Wait for the next frame no longer than until the audio is due, and while it is
            // missing, no longer than until the next zero fill. Other packets (TX_CHRONO,
            // text) keep the socket busy without bringing audio, so the deadline is ours.
            let left = audio_due.saturating_duration_since(Instant::now());
            let step = if left.is_zero() {
                Step::Down
            } else {
                let wait = if lost { left.min(FILL_TICK) } else { left };
                match w.get_ref().set_read_timeout(Some(wait)) {
                    Ok(()) => self.read(w),
                    Err(_) => Step::Down,
                }
            };
            let chunk = match step {
                Step::Audio(c) => c,
                Step::Nothing => {
                    if lost {
                        self.fill(last.elapsed(), &mut filled)?;
                    }
                    continue;
                }
                Step::Down => {
                    if stopped(&self.stop) {
                        return None;
                    }
                    if !lost {
                        eprintln!("TCI audio from {} lost; reconnecting", self.addr);
                        next_note = Instant::now() + DOWN_NOTE_EVERY;
                    }
                    lost = true;
                    ws = None;
                    self.shared.sock.lock().unwrap().take();
                    continue;
                }
            };
            let now = Instant::now();
            if lost {
                // Top up the zeros to the full gap, from arrival times. Only across a loss:
                // within a working connection, network jitter would show up as false gaps.
                // Top-up zeros dropped on a full queue are not sent again.
                lost = false;
                backoff = MIN_BACKOFF;
                let span = ((now - last).as_secs_f64() * RATE as f64) as usize;
                self.zeros(span.saturating_sub(chunk.len() + filled))?;
                eprintln!("TCI audio from {} back after {:.1} s", self.addr, (now - last).as_secs_f64());
            }
            last = now;
            audio_due = now + self.timing.read;
            filled = 0;
            if !chunk.is_empty() {
                self.send(chunk)?;
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
            max_backoff: Duration::from_millis(200),
        }
    }

    /// The modem's clock (all samples so far) against wall time since the first packet,
    /// `t0`, after `secs`: never ahead, and behind by at most one backoff plus a connect.
    fn assert_wall_pace(rx: &Receiver<Vec<f32>>, t0: Instant, secs: f64) {
        let mut got = 0;
        while t0.elapsed().as_secs_f64() < secs {
            if let Ok(c) = rx.recv_timeout(Duration::from_millis(20)) {
                got += c.len();
            }
        }
        got += rx.try_iter().map(|c| c.len()).sum::<usize>();
        let s = got as f64 / RATE as f64;
        assert!(s > secs - 0.35 && s < secs + 0.1, "{s:.2} s of audio after {secs} s");
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
        assert_eq!(addr_from_name("TCI:"), Some(""));
        assert!(open_err("", fast()).contains("no address"));
        assert_eq!(addr_from_name("Tci:10.0.0.5:50001"), Some("10.0.0.5:50001"));
        assert_eq!(addr_from_name("USB Audio"), None);
        assert_eq!(addr_from_name("tcix"), None);
        assert_eq!(addr_from_name("tc"), None);
    }

    #[test]
    fn parse_bounds_length_and_skips() {
        let mut p = rx_audio(&[(0.1, 0.2), (0.3, 0.4)]);
        assert_eq!(parse_packet(&p, Channel::Left), Packet::Audio { mono: vec![0.1, 0.3], damaged: false });
        // Truncated: `length` promises 4 frames, 2 are present. The rest become zeros, as an error.
        p[20..24].copy_from_slice(&8u32.to_le_bytes());
        assert_eq!(parse_packet(&p, Channel::Left), Packet::Audio { mono: vec![0.1, 0.3, 0.0, 0.0], damaged: true });
        // Half a frame present: rounded down, the rest zeros.
        p.truncate(p.len() - 4);
        assert_eq!(parse_packet(&p, Channel::Right), Packet::Audio { mono: vec![0.2, 0.0, 0.0, 0.0], damaged: true });
        // An odd `length` is not whole stereo frames.
        let mut p = rx_audio(&[(0.1, 0.2), (0.3, 0.4)]);
        p[20..24].copy_from_slice(&3u32.to_le_bytes());
        assert_eq!(parse_packet(&p, Channel::Left), Packet::Audio { mono: vec![0.1], damaged: true });
        // A huge `length` stands for at most 1 s.
        p[20..24].copy_from_slice(&u32::MAX.to_le_bytes());
        assert!(
            matches!(parse_packet(&p, Channel::Left), Packet::Audio { mono, damaged: true } if mono.len() == RATE as usize + 2)
        );
        assert_eq!(parse_packet(&packet(3, 0, RATE, 3, &[(0.0, 0.0)]), Channel::Left), Packet::Skip);
        assert_eq!(parse_packet(&packet(1, 1, RATE, 3, &[(0.0, 0.0)]), Channel::Left), Packet::Skip);
        assert_eq!(parse_packet(&[0; 10], Channel::Left), Packet::Short);
        // A bad rate stands for at most 1 s, however small the rate.
        assert!(matches!(
            parse_packet(&packet(1, 0, 1, 3, &[(0.0, 0.0); 4]), Channel::Left),
            Packet::Bad { frames, .. } if frames == RATE as usize
        ));
    }

    #[test]
    fn short_frames_are_skipped_on_open() {
        let addr = server(|_, ws| {
            commands(ws);
            ws.send(Message::binary(vec![0u8; 10])).unwrap();
            send_audio(ws, &[(0.5, 0.5)]);
            hold(ws);
        });
        let o = open(&addr, Channel::Left, fast()).unwrap();
        assert_eq!(o.rx.recv_timeout(Duration::from_secs(1)).unwrap(), vec![0.5]);
    }

    /// Samples received in `secs` of wall time.
    fn samples_in(rx: &Receiver<Vec<f32>>, secs: f64) -> usize {
        let start = Instant::now();
        let mut got = 0;
        while start.elapsed().as_secs_f64() < secs {
            if let Ok(c) = rx.recv_timeout(Duration::from_millis(50)) {
                got += c.len();
            }
        }
        got
    }

    #[test]
    fn clock_keeps_moving_while_lost_and_backs_off() {
        // One packet, then every connection closes at once: the server never comes back.
        let conns = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let c = conns.clone();
        let addr = server(move |n, ws| {
            c.store(n + 1, Ordering::Relaxed);
            commands(ws);
            if n == 0 {
                send_audio(ws, &[(0.5, 0.5)]);
            }
        });
        // The real maximum backoff, to see it grow.
        let o = open(&addr, Channel::Left, Timing { max_backoff: Duration::from_secs(1), ..fast() }).unwrap();
        let s = samples_in(&o.rx, 1.5) as f64 / RATE as f64;
        // Behind by at most one backoff (up to 800 ms in this window), never ahead.
        assert!(s > 0.6 && s < 1.6, "{s:.2} s of audio in 1.5 s");
        // Backoff 100, 200, 400, 800 ms: not a reconnect every 100 ms, though each handshake succeeds.
        let n = conns.load(Ordering::Relaxed);
        assert!(n <= 6, "{n} connections in 1.5 s");
    }

    #[test]
    fn steady_zeros_while_lost() {
        // Every connection after the first closes at once.
        let addr = server(|n, ws| {
            commands(ws);
            if n == 0 {
                send_audio(ws, &[(0.5, 0.5)]);
            }
        });
        let o = open(&addr, Channel::Left, fast()).unwrap();
        assert_wall_pace(&o.rx, Instant::now(), 1.5);
    }

    #[test]
    fn other_packets_do_not_hide_missing_audio() {
        // Audio once, then only TX_CHRONO packets and text, as fast as wfview sends packets.
        let addr = server(|_, ws| {
            commands(ws);
            send_audio(ws, &[(0.5, 0.5)]);
            loop {
                let chrono = Message::binary(packet(3, 0, RATE, FORMAT_FLOAT32, &[(0.0, 0.0); 4]));
                if ws.send(chrono).is_err() || ws.send(Message::text("vfo:0,0,145500000;")).is_err() {
                    return;
                }
                std::thread::sleep(Duration::from_millis(40));
            }
        });
        let o = open(&addr, Channel::Left, fast()).unwrap();
        assert_wall_pace(&o.rx, Instant::now(), 1.5);
    }

    #[test]
    fn audio_timeout_counts_from_the_audio_and_silent_reconnects_still_fill() {
        // Audio, one TX_CHRONO packet 0.8 s later, then silence. Later connections stay silent.
        let addr = server(|n, ws| {
            commands(ws);
            if n == 0 {
                send_audio(ws, &[(0.5, 0.5)]);
                std::thread::sleep(Duration::from_millis(800));
                let _ = ws.send(Message::binary(packet(3, 0, RATE, FORMAT_FLOAT32, &[(0.0, 0.0); 4])));
            }
            hold(ws);
        });
        let o = open(&addr, Channel::Left, Timing { read: Duration::from_secs(1), ..fast() }).unwrap();
        let t0 = Instant::now();
        assert_eq!(o.rx.recv_timeout(Duration::from_secs(1)).unwrap(), vec![0.5]);
        // Lost 1 s after the audio, not 1 s after the TX_CHRONO packet (1.8 s).
        let zeros = o.rx.recv_timeout(Duration::from_secs(3)).unwrap();
        assert!(t0.elapsed() < Duration::from_millis(1300), "first zeros after {:?}", t0.elapsed());
        // The new connections stay open and silent: zeros at least every 250 ms, not only
        // after each 1 s read timeout. (The fill from t0 covers the first 1 s pause.)
        let mut got = 1 + zeros.len();
        while t0.elapsed() < Duration::from_secs(3) {
            if let Ok(c) = o.rx.recv_timeout(Duration::from_millis(20)) {
                got += c.len();
            }
        }
        got += o.rx.try_iter().map(|c| c.len()).sum::<usize>();
        let s = got as f64 / RATE as f64;
        assert!(s > 3.0 - 0.35 && s < 3.0 + 0.1, "{s:.2} s of audio after 3 s");
    }

    #[test]
    fn drop_during_reconnect_handshake_returns_promptly() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        std::thread::spawn(move || {
            let mut ws = tungstenite::accept(listener.accept().unwrap().0).unwrap();
            commands(&mut ws);
            send_audio(&mut ws, &[(0.5, 0.5)]);
            drop(ws);
            // Later connections: accept TCP but never answer the WebSocket handshake.
            let mut held = Vec::new();
            for s in listener.incoming() {
                held.push(s);
            }
        });
        // Real timing: the stalled handshake would wait out the 3 s connect deadline.
        let o = open(&addr, Channel::Left, Timing::default()).unwrap();
        std::thread::sleep(Duration::from_millis(400)); // past the 100 ms backoff, into the handshake
        let t = Instant::now();
        drop(o);
        assert!(t.elapsed() < Duration::from_millis(500), "{:?}", t.elapsed());
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
                cmd_rx.recv_timeout(Duration::from_secs(5)).unwrap(),
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
        // TX_CHRONO only: a server that streams other packets but no RX audio. (Whether wfview
        // sends TX_CHRONO without RX audio, e.g. when its Audio System is not TCI, is unverified.)
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

        // Port 1 is privileged and normally closed. A port that was free a moment ago could be
        // taken by another test's server.
        let e = open_err("127.0.0.1:1", fast());
        assert!(e.contains("cannot reach") || e.contains("no answer"), "{e}");
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
    fn bad_packet_right_after_reconnect() {
        let addr = server(|n, ws| {
            commands(ws);
            if n == 0 {
                send_audio(ws, &[(0.5, 0.5); 2048]);
            } else {
                std::thread::sleep(Duration::from_millis(200));
                // int16: an error, standing for its 2048 frames.
                ws.send(Message::binary(packet(1, 0, RATE, 0, &[(0.0, 0.0); 2048]))).unwrap();
                send_audio(ws, &[(0.25, 0.25); 2048]);
                hold(ws);
            }
        });
        let start = Instant::now();
        let o = open(&addr, Channel::Left, Timing { read: Duration::from_secs(5), ..fast() }).unwrap();
        let mut all = Vec::new();
        while !all.contains(&0.25) {
            all.extend(o.rx.recv_timeout(Duration::from_secs(5)).expect("audio after reconnect"));
        }
        let elapsed = start.elapsed().as_secs_f64();
        all.extend(take(&o.rx, 2048 - all.iter().filter(|&&x| x == 0.25).count()).concat());
        assert!(all[..2048].iter().all(|&x| x == 0.5));
        let zeros = all[2048..].iter().take_while(|&&x| x == 0.0).count();
        assert_eq!(all.len(), 2048 + zeros + 2048, "one run of zeros, then the good packet");
        // The bad packet's 2048 frames, plus the gap of about backoff (100 ms) + server delay (200 ms).
        let ms = zeros as f64 * 1000.0 / RATE as f64;
        // The zeros (gap plus bad packet) span the wall time between the last good packet
        // and the bad one: never more than the whole run.
        assert!(ms > 2048.0 * 1000.0 / RATE as f64 + 150.0 && ms < elapsed * 1000.0, "{ms:.0} ms of zeros");
        assert_eq!(o.overruns.load(Ordering::Relaxed), 1);
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
        // A long read timeout: a stall would otherwise reconnect and replay the burst.
        let o = open(&addr, Channel::Right, Timing { read: Duration::from_secs(5), ..fast() }).unwrap();
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
