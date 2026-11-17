//! `vardefm watch`: a terminal view of a modem's monitor API (`vardefm modem --api ADDR`).
//!
//! It shows the burst log and the modem's status on top, and the waterfall with the latest
//! burst's constellation below. It only reads: the API has no way to change the modem. A
//! session can be saved to a capture and replayed later, without a modem.

pub mod capture;
pub mod model;
pub mod source;
pub mod view;

#[cfg(test)]
mod tests;

use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::sync::Arc;
use std::time::{Duration, Instant};

use ratatui::crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use ratatui::DefaultTerminal;

use model::{Conn, Model};
use source::Feed;
use source::{MAX_SPEED, MIN_SPEED};
use view::{ViewOpts, MAX_SPAN_DB, MIN_SPAN_DB};

/// What to watch.
#[derive(Clone, Debug)]
pub enum Watch {
    /// A modem's monitor API, optionally saving what it sends.
    Live { addr: SocketAddr, save: Option<PathBuf> },
    /// A capture, `speed` times as fast as it was recorded.
    Replay { path: PathBuf, speed: f64 },
}

/// Redraws per second, at most.
const FPS: u64 = 20;

/// Runs the view until the user quits.
pub fn run(watch: Watch) -> Result<(), String> {
    let (tx, rx) = mpsc::channel();
    let stop = Arc::new(AtomicBool::new(false));
    let (source_name, handle) = match watch {
        Watch::Live { addr, save } => {
            let name = format!("ws://{addr}/v1/stream");
            let handle = source::live(addr, save.clone(), tx, stop.clone())
                .map_err(|e| format!("cannot create {}: {e}", save.unwrap_or_default().display()))?;
            (name, handle)
        }
        Watch::Replay { path, speed } => {
            if !(MIN_SPEED..=MAX_SPEED).contains(&speed) {
                return Err(format!("--speed must be from {MIN_SPEED} to {MAX_SPEED}"));
            }
            let name = format!("{} (x{speed})", path.display());
            (name, source::replay(path, speed, tx, stop.clone())?)
        }
    };
    let mut model = Model::new(source_name);
    let result = match ratatui::try_init() {
        Ok(terminal) => {
            let result = ui(terminal, &mut model, &rx);
            ratatui::restore();
            result
        }
        Err(e) => Err(format!("vardefm watch needs a terminal: {e}")),
    };
    stop.store(true, Ordering::Relaxed);
    drop(rx);
    let _ = handle.join();
    result
}

/// Applies everything waiting on `rx`. A source gone without saying it ended (its thread
/// failed) shows as [`Conn::Gone`].
pub fn drain(model: &mut Model, rx: &Receiver<Feed>) {
    loop {
        match rx.try_recv() {
            Ok(Feed::Connected) => model.conn = Conn::Connected,
            Ok(Feed::Lost(why)) => model.conn = Conn::Lost(why),
            Ok(Feed::Ended) => model.conn = Conn::Ended,
            Ok(Feed::SaveFailed(why)) => model.save_error = Some(why),
            Ok(Feed::Msg(msg)) => model.apply(&msg),
            Err(TryRecvError::Empty) => return,
            Err(TryRecvError::Disconnected) => {
                if model.conn != Conn::Ended {
                    model.conn = Conn::Gone;
                }
                return;
            }
        }
    }
}

/// What a key does to the view; `None` to quit.
pub fn key(opts: ViewOpts, code: KeyCode, modifiers: KeyModifiers) -> Option<ViewOpts> {
    let mut opts = opts;
    match code {
        KeyCode::Char('q') | KeyCode::Esc => return None,
        KeyCode::Char('c') if modifiers.contains(KeyModifiers::CONTROL) => return None,
        KeyCode::Up => opts.floor_offset_db += 3.0,
        KeyCode::Down => opts.floor_offset_db -= 3.0,
        KeyCode::Char('+') | KeyCode::Char('=') => opts.span_db = (opts.span_db + 10.0).min(MAX_SPAN_DB),
        KeyCode::Char('-') => opts.span_db = (opts.span_db - 10.0).max(MIN_SPAN_DB),
        KeyCode::Char('a') => opts = ViewOpts::default(),
        _ => {}
    }
    Some(opts)
}

fn ui(mut terminal: DefaultTerminal, model: &mut Model, rx: &Receiver<Feed>) -> Result<(), String> {
    let mut opts = ViewOpts::default();
    let frame_time = Duration::from_millis(1000 / FPS);
    loop {
        let started = Instant::now();
        drain(model, rx);
        terminal.draw(|f| view::draw(f, model, &opts)).map_err(|e| e.to_string())?;
        // Wait for a key until the next frame is due.
        while let Some(left) = frame_time.checked_sub(started.elapsed()) {
            if !event::poll(left).map_err(|e| e.to_string())? {
                break;
            }
            if let Event::Key(k) = event::read().map_err(|e| e.to_string())? {
                if k.kind == KeyEventKind::Press {
                    match key(opts, k.code, k.modifiers) {
                        Some(o) => opts = o,
                        None => return Ok(()),
                    }
                }
            }
        }
    }
}
