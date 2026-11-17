//! Where `vardefm watch` gets its messages: a modem's monitor API, or a capture replayed.
//!
//! Each source runs on its own thread and sends [`Feed`] items down a channel, so the screen
//! never waits on the network or the disk. Both stop when `stop` is set or the receiver is gone.

use std::fs::{File, OpenOptions};
use std::io::{self, BufReader, BufWriter};
use std::net::{SocketAddr, TcpStream};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Sender;
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use super::capture::{read_all, CaptureWriter, Message, Record};

/// Wait between attempts to reach the modem.
pub const RETRY: Duration = Duration::from_secs(1);
/// How long a read waits before checking `stop`.
const POLL: Duration = Duration::from_millis(200);
/// How long the modem may take to answer the WebSocket handshake.
const HANDSHAKE: Duration = Duration::from_secs(5);
/// Slowest replay.
pub const MIN_SPEED: f64 = 0.01;
/// Fastest replay.
pub const MAX_SPEED: f64 = 1000.0;

/// What a source tells the screen.
#[derive(Debug)]
pub enum Feed {
    Connected,
    Lost(String),
    Msg(Message),
    /// Saving the capture failed; the stream goes on unsaved.
    SaveFailed(String),
    /// A replay has nothing more.
    Ended,
}

/// Follows the monitor API at `addr`, reconnecting every [`RETRY`] after a failure. With
/// `save`, every message is also written to that capture, its time counted from the start; an
/// existing file is refused, not overwritten.
pub fn live(
    addr: SocketAddr,
    save: Option<PathBuf>,
    tx: Sender<Feed>,
    stop: Arc<AtomicBool>,
) -> io::Result<JoinHandle<()>> {
    let mut writer = match save {
        Some(path) => {
            let file = OpenOptions::new().write(true).create_new(true).open(path)?;
            Some(CaptureWriter::new(BufWriter::new(file)))
        }
        None => None,
    };
    Ok(thread::spawn(move || {
        let start = Instant::now();
        while !stop.load(Ordering::Relaxed) {
            let reason = match follow(addr, &tx, &stop, &mut |msg: &Message| {
                if let Some(w) = writer.as_mut() {
                    let record = Record { ms: start.elapsed().as_millis() as u64, msg: msg.clone() };
                    if let Err(e) = w.write(&record) {
                        let _ = tx.send(Feed::SaveFailed(e.to_string()));
                        writer = None;
                    }
                }
            }) {
                Ok(()) => return, // stopped, or nobody is listening
                Err(reason) => reason,
            };
            if tx.send(Feed::Lost(reason)).is_err() {
                return;
            }
            let until = Instant::now() + RETRY;
            while Instant::now() < until {
                if stop.load(Ordering::Relaxed) {
                    return;
                }
                thread::sleep(Duration::from_millis(50));
            }
        }
    }))
}

/// One connection: `Ok` when we should stop, `Err` with the reason when it failed.
fn follow(
    addr: SocketAddr,
    tx: &Sender<Feed>,
    stop: &AtomicBool,
    save: &mut impl FnMut(&Message),
) -> Result<(), String> {
    let tcp = TcpStream::connect_timeout(&addr, RETRY).map_err(|e| format!("cannot connect to {addr}: {e}"))?;
    // A read timeout during the handshake fails it outright, so the handshake gets a longer one
    // than the poll: a modem slow to answer must still connect.
    tcp.set_read_timeout(Some(HANDSHAKE)).map_err(|e| e.to_string())?;
    let (mut ws, _) = tungstenite::client(format!("ws://{addr}/v1/stream"), tcp)
        .map_err(|e| format!("WebSocket handshake with {addr} failed: {e}"))?;
    ws.get_ref().set_read_timeout(Some(POLL)).map_err(|e| e.to_string())?;
    if tx.send(Feed::Connected).is_err() {
        return Ok(());
    }
    loop {
        if stop.load(Ordering::Relaxed) {
            let _ = ws.close(None);
            return Ok(());
        }
        let msg = match ws.read() {
            Ok(tungstenite::Message::Text(text)) => Message::Text(text.as_str().to_owned()),
            Ok(tungstenite::Message::Binary(bytes)) => Message::Binary(bytes.to_vec()),
            Ok(tungstenite::Message::Close(frame)) => {
                return Err(match frame {
                    Some(f) => format!("the modem closed the stream: {} {}", f.code, f.reason),
                    None => "the modem closed the stream".into(),
                })
            }
            Ok(_) => continue, // ping and pong: tungstenite answers pings itself
            Err(tungstenite::Error::Io(e))
                if matches!(e.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut) =>
            {
                continue
            }
            Err(e) => return Err(format!("stream lost: {e}")),
        };
        save(&msg);
        if tx.send(Feed::Msg(msg)).is_err() {
            return Ok(());
        }
    }
}

/// Replays the capture at `path`, `speed` times as fast as it was recorded. Reads the whole
/// capture first, so a bad line is reported before anything is shown. `speed` is kept within
/// [`MIN_SPEED`] and [`MAX_SPEED`].
pub fn replay(path: PathBuf, speed: f64, tx: Sender<Feed>, stop: Arc<AtomicBool>) -> Result<JoinHandle<()>, String> {
    let file = File::open(&path).map_err(|e| format!("cannot open {}: {e}", path.display()))?;
    let records = read_all(BufReader::new(file)).map_err(|e| format!("{}: {e}", path.display()))?;
    let speed = if speed.is_finite() { speed.clamp(MIN_SPEED, MAX_SPEED) } else { 1.0 };
    Ok(thread::spawn(move || {
        let _ = tx.send(Feed::Connected);
        let start = Instant::now();
        let first = records.first().map_or(0, |r| r.ms);
        for record in records {
            let secs = record.ms.saturating_sub(first) as f64 / 1000.0 / speed;
            let due = Duration::try_from_secs_f64(secs).unwrap_or(Duration::MAX);
            while start.elapsed() < due {
                if stop.load(Ordering::Relaxed) {
                    return;
                }
                thread::sleep((due - start.elapsed()).min(Duration::from_millis(50)));
            }
            if tx.send(Feed::Msg(record.msg)).is_err() {
                return;
            }
        }
        let _ = tx.send(Feed::Ended);
    }))
}
