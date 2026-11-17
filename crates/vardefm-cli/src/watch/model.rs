//! What `vardefm watch` knows about the modem: everything the stream said, kept for drawing.
//!
//! The model reads messages and keeps state; it draws nothing, so it can be tested without a
//! terminal.

use std::collections::VecDeque;

use vardefm_api::constellation::{ConstellationFrame, STREAM_VFM_CONSTELLATION};
use vardefm_api::wire::{ApiEvent, Envelope, Hello, Status, StreamHeader, VfmBurstRx, VfmBurstTx, STREAM_WATERFALL};

use super::capture::Message;

/// Waterfall rows kept: 60 s at 10 rows a second, more than a wide terminal shows.
pub const HISTORY_ROWS: usize = 600;
/// Rows the noise floor is taken over: 5 s.
pub const FLOOR_ROWS: usize = 50;
/// Which quantile of the recent bins counts as the noise floor.
pub const FLOOR_QUANTILE: f32 = 0.10;
/// Entries kept in the burst log.
pub const MAX_LOG: usize = 500;
/// Warnings kept.
pub const MAX_WARNINGS: usize = 8;
/// Busy changes kept.
pub const MAX_BUSY: usize = 200;
/// Samples per second on the modem's clock.
pub const RATE: f64 = 48_000.0;

/// A full time from the low 32 bits a binary frame carries, taking the one nearest `reference`,
/// the latest full time the stream gave. 2^32 samples are almost 25 hours, so the nearest is
/// the right one for any frame sent within 12 hours of the reference.
pub fn unwrap_t(low: u32, reference: u64) -> u64 {
    const SPAN: u64 = 1 << 32;
    let hi = reference >> 32;
    [hi.wrapping_sub(1), hi, hi + 1]
        .into_iter()
        .filter(|&h| h < SPAN) // hi - 1 wrapped below zero
        .map(|h| (h << 32) | u64::from(low))
        .min_by_key(|&t| t.abs_diff(reference))
        .unwrap_or(u64::from(low))
}

/// A waterfall row, its time unwrapped.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Row {
    /// First sample of the row.
    pub t: u64,
    /// One byte per bin, `db_floor + db_step * b` dBFS.
    pub bins: Vec<u8>,
}

/// One line of the burst log.
#[derive(Clone, Debug, PartialEq)]
pub enum Entry {
    Rx(VfmBurstRx),
    Tx(VfmBurstTx),
    /// Received audio was lost before sample `t`.
    Gap(u64),
    /// A new `Hello`: the modem restarted or we reconnected, and times start over.
    Restart {
        mode: String,
    },
}

/// A burst log line and the stream session it came in, so a burst from before a reconnect is
/// never drawn against the waterfall of the next one.
#[derive(Clone, Debug, PartialEq)]
pub struct LogLine {
    pub session: u32,
    pub entry: Entry,
}

/// The constellation on show.
#[derive(Clone, Debug, PartialEq)]
pub struct Shown {
    /// The burst's start, as in its `VfmBurstRx`.
    pub t: u64,
    pub frame: ConstellationFrame,
    /// The burst it belongs to, if its `VfmBurstRx` came first, as the server sends them.
    pub burst: Option<VfmBurstRx>,
}

/// The noise floor: a quantile of the bins of the last [`FLOOR_ROWS`] rows.
#[derive(Clone, Debug)]
pub struct Floor {
    hist: [u32; 256],
    rows: VecDeque<Vec<u8>>,
}

impl Default for Floor {
    fn default() -> Self {
        Self { hist: [0; 256], rows: VecDeque::new() }
    }
}

impl Floor {
    pub fn push(&mut self, bins: &[u8]) {
        for &b in bins {
            self.hist[usize::from(b)] += 1;
        }
        self.rows.push_back(bins.to_vec());
        if self.rows.len() > FLOOR_ROWS {
            for b in self.rows.pop_front().unwrap_or_default() {
                self.hist[usize::from(b)] -= 1;
            }
        }
    }

    /// The floor as a bin value, or `None` before the first row.
    pub fn value(&self) -> Option<u8> {
        let total: u64 = self.hist.iter().map(|&n| u64::from(n)).sum();
        if total == 0 {
            return None;
        }
        let want = ((total as f32 * FLOOR_QUANTILE).ceil() as u64).max(1);
        let mut seen = 0;
        for (v, &n) in self.hist.iter().enumerate() {
            seen += u64::from(n);
            if seen >= want {
                return Some(v as u8);
            }
        }
        Some(255)
    }

    pub fn clear(&mut self) {
        *self = Self::default();
    }
}

/// The state of the connection to the modem, or of the replay.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Conn {
    /// Trying to reach the modem.
    Connecting,
    /// Streaming.
    Connected,
    /// The connection failed or ended; retrying.
    Lost(String),
    /// A replay ran to its end.
    Ended,
}

/// Everything the stream has said.
#[derive(Debug)]
pub struct Model {
    pub conn: Conn,
    /// What is being watched: the address, or the capture file.
    pub source: String,
    pub hello: Option<Hello>,
    pub status: Option<Status>,
    /// Stream sessions so far: one per `Hello`.
    pub session: u32,
    pub log: VecDeque<LogLine>,
    pub rows: VecDeque<Row>,
    pub floor: Floor,
    pub constellation: Option<Shown>,
    /// Busy changes of this session: (time, busy now).
    pub busy: VecDeque<(u64, bool)>,
    /// Latest warnings: (time, code, text).
    pub warnings: VecDeque<(u64, String, String)>,
    /// Latest full time the stream gave, for unwrapping frame times.
    pub latest_t: u64,
    /// Messages that could not be read, such as a newer server's.
    pub unknown: u64,
}

impl Model {
    pub fn new(source: String) -> Self {
        Self {
            conn: Conn::Connecting,
            source,
            hello: None,
            status: None,
            session: 0,
            log: VecDeque::new(),
            rows: VecDeque::new(),
            floor: Floor::default(),
            constellation: None,
            busy: VecDeque::new(),
            warnings: VecDeque::new(),
            latest_t: 0,
            unknown: 0,
        }
    }

    pub fn apply(&mut self, msg: &Message) {
        match msg {
            Message::Text(text) => match serde_json::from_str::<Envelope>(text) {
                Ok(envelope) => self.event(envelope.event),
                Err(_) => self.unknown += 1,
            },
            Message::Binary(frame) => self.frame(frame),
        }
    }

    fn see(&mut self, t: u64) {
        self.latest_t = self.latest_t.max(t);
    }

    fn push_log(&mut self, entry: Entry) {
        self.log.push_back(LogLine { session: self.session, entry });
        while self.log.len() > MAX_LOG {
            self.log.pop_front();
        }
    }

    fn event(&mut self, event: ApiEvent) {
        match event {
            ApiEvent::Hello(hello) => {
                if self.session > 0 {
                    self.push_log(Entry::Restart { mode: hello.mode.clone() });
                }
                self.session += 1;
                self.rows.clear();
                self.floor.clear();
                self.constellation = None;
                self.busy.clear();
                self.latest_t = 0;
                self.status = None;
                self.hello = Some(hello);
            }
            ApiEvent::Status(status) => {
                self.see(status.t.0);
                self.status = Some(status);
            }
            ApiEvent::Gap { t } => {
                self.see(t.0);
                self.push_log(Entry::Gap(t.0));
            }
            ApiEvent::Warning { code, text } => {
                self.warnings.push_back((self.latest_t, code, text));
                while self.warnings.len() > MAX_WARNINGS {
                    self.warnings.pop_front();
                }
            }
            ApiEvent::VfmBurstRx(rx) => {
                self.see(rx.t_end.0.max(rx.t.0));
                self.push_log(Entry::Rx(rx));
            }
            ApiEvent::VfmBurstTx(tx) => {
                self.see(tx.t.0);
                self.push_log(Entry::Tx(tx));
            }
            ApiEvent::VfmBusy { t, busy } => {
                self.see(t.0);
                self.busy.push_back((t.0, busy));
                while self.busy.len() > MAX_BUSY {
                    self.busy.pop_front();
                }
            }
        }
    }

    fn frame(&mut self, frame: &[u8]) {
        let Some((header, payload)) = StreamHeader::decode(frame) else {
            self.unknown += 1;
            return;
        };
        let t = unwrap_t(header.t_low, self.latest_t);
        match header.stream {
            STREAM_WATERFALL => {
                let wanted = self.hello.as_ref().map(|h| usize::from(h.waterfall.bins));
                if wanted.is_some_and(|n| n != payload.len()) {
                    self.unknown += 1;
                    return;
                }
                self.see(t);
                self.floor.push(payload);
                self.rows.push_back(Row { t, bins: payload.to_vec() });
                while self.rows.len() > HISTORY_ROWS {
                    self.rows.pop_front();
                }
            }
            STREAM_VFM_CONSTELLATION => match ConstellationFrame::decode(payload) {
                Some(frame) => {
                    let burst = self.log.iter().rev().find_map(|line| match &line.entry {
                        Entry::Rx(rx) if line.session == self.session && rx.t.0 == t => Some(rx.clone()),
                        _ => None,
                    });
                    self.constellation = Some(Shown { t, frame, burst });
                }
                None => self.unknown += 1,
            },
            // A stream this tool does not know: skip it, as the API asks.
            _ => {}
        }
    }

    /// Whether the channel was busy at `t`, from this session's busy changes.
    pub fn busy_at(&self, t: u64) -> bool {
        self.busy.iter().take_while(|&&(at, _)| at <= t).last().is_some_and(|&(_, busy)| busy)
    }
}
