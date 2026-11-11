//! VardeFM ARQ link layer.
//!
//! A pure state machine: the caller passes in the current time, received bursts,
//! host commands and whether the channel is busy. It asks [`Link::poll`] what to
//! transmit, and reads host notifications from [`Link::take_events`].
//!
//! # Protocol summary
//! - **Connect:** the caller sends `ConnectReq` (session id + call signs) and the
//!   called station answers `ConnectAck`. Requests are retried
//!   `connect_attempts` times.
//! - **Data:** a byte stream in each direction, numbered by byte offset. Each
//!   payload codeword carries one segment: `offset (24 bits) | length (8 bits) | data`.
//! - **Acknowledgements:** every burst carries `ack_offset`, the next byte expected
//!   from the peer, plus a bitmap of which codewords of the peer's last burst
//!   decoded. A sender resends everything not covered by either.
//! - **Turns:** after receiving a burst that carried data (or a POLL), a station
//!   answers after `turnaround_s`. Its answer carries acks and any data of its
//!   own. If neither side has anything to send, the link goes quiet. A station
//!   that then gets data from its host starts a burst once the channel is clear.
//!   After `ConnectAck`, the called station may send first; the caller waits one
//!   response period for it.
//! - **Rate adaptation:** each burst names the speed level the peer should use
//!   next. It is chosen from the measured SNR (normalized for the per-level
//!   transmit drive) and stepped down after codeword failures or timeouts.
//! - **Disconnect:** after the host's DISCONNECT, once all sent data is
//!   acknowledged, the station sends `Disc` and the peer answers `DiscAck`. Both
//!   then report DISCONNECTED.

use std::collections::{BTreeMap, VecDeque};
use std::sync::Arc;
use vardefm_dsp::prng::Prng;
use vardefm_modem::{Burst, Header, Kind, Modem, RxBurst, FLAG_POLL, LEVELS};

/// Estimated SNR (dB, as reported by the receiver) on a burst sent at each level, at
/// which that level decodes reliably in the FM channel simulator (see the
/// `level_table` benchmark). Each level has its own transmit drive, so a burst
/// measured at one level is normalized to the drive of another before comparing.
///
/// Each value is the highest SNR of any burst at that level with a failed codeword,
/// or the lowest SNR from which every burst decoded fully if higher, rounded up to
/// 0.5 dB, in the worse of the two modes. So with the margin, a level is only
/// requested at least 1 dB above every failure seen. Below about 10 dB the SNR
/// estimate falls fast with CNR (FM threshold), so the low levels' values are small
/// or negative; level 0's is never used.
///
/// Checked against Rayleigh fading at 5, 20 and 80 Hz Doppler (the `fading_table`
/// benchmark): a fade lowers the reported SNR of the burst it hits, so every failed
/// burst still reported less than its level's threshold. No fading margin is needed.
pub const SNR_THRESHOLDS_DB: [f32; 7] = [-2.0, 1.5, 4.5, 7.0, 11.0, 15.0, 18.0];
const SNR_MARGIN_DB: f32 = 1.0;

/// Highest level whose threshold (plus margin) the SNR would reach, given `snr_db`
/// measured on a burst sent at `rx_level`. Each level has its own drive and clipping
/// floor, so the SNR is first predicted for that level ([`Modem::predict_snr_db`]).
pub fn level_by_snr(modem: &Modem, snr_db: f32, rx_level: u8) -> u8 {
    SNR_THRESHOLDS_DB
        .iter()
        .enumerate()
        .rposition(|(l, &t)| modem.predict_snr_db(snr_db, rx_level, l as u8) >= t + SNR_MARGIN_DB)
        .unwrap_or(0) as u8
}

const SEGMENT_HEADER: usize = 4;
const OFFSET_MOD: u64 = 1 << 24;

#[derive(Clone, Debug)]
pub struct LinkConfig {
    /// Gap between receiving a burst and answering it.
    pub turnaround_s: f64,
    /// How long to wait for the peer to start answering before retrying.
    /// The wait is extended while the channel is busy.
    pub response_timeout_s: f64,
    pub connect_attempts: u32,
    /// Consecutive unanswered bursts before the link is declared lost.
    pub max_retries: u32,
    /// Delay between DISCONNECT and closing, so data written just before it
    /// (on the separate data socket) is still sent.
    pub disconnect_grace_s: f64,
    /// Probe a silent link after this long.
    pub keepalive_s: f64,
    /// Level used until the peer asks for another.
    pub start_level: u8,
    pub max_level: u8,
}

impl Default for LinkConfig {
    fn default() -> Self {
        Self {
            turnaround_s: 0.2,
            response_timeout_s: 1.2,
            connect_attempts: 5,
            max_retries: 8,
            disconnect_grace_s: 0.3,
            keepalive_s: 30.0,
            start_level: 1,
            max_level: LEVELS.len() as u8 - 1,
        }
    }
}

/// Notifications for the host application.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LinkEvent {
    Pending,
    CancelPending,
    /// `source` is the calling station.
    Connected {
        source: String,
        destination: String,
    },
    Disconnected,
    /// Bytes queued for transmission and not yet acknowledged.
    Buffer(usize),
    Data(Vec<u8>),
}

/// What the last transmitted burst contained (for logging).
#[derive(Clone, Debug, Default)]
pub struct TxInfo {
    pub kind: Kind,
    pub level: u8,
    pub ncw: usize,
    /// Payload bytes in this burst, including resent bytes.
    pub bytes: usize,
    pub resent_bytes: usize,
    /// Consecutive retry number (0 = first attempt).
    pub retry: u32,
}

#[derive(Clone, Debug, Default)]
pub struct LinkStats {
    pub bursts_sent: u64,
    pub retries: u64,
    pub payload_bytes_sent: u64,
    pub bytes_resent: u64,
    pub bytes_delivered: u64,
    pub codewords_received: u64,
    pub codewords_failed: u64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Phase {
    /// Nothing to do until something happens.
    Idle,
    /// Transmit a burst of `kind` at time `at`. `initiated`: we start the exchange
    /// ourselves (not answering the peer), so give way if the channel turns busy.
    Respond { at: f64, kind: Kind, initiated: bool },
    /// Our burst is on the air.
    Sending { kind: Kind, retry: bool },
    /// Waiting for the peer. `retry`: resend if it does not answer.
    Wait { deadline: f64, retry: bool },
}

struct Session {
    id: u8,
    local: String,
    remote: String,
    caller: bool,
    // Transmit side: tx_buf holds every byte from tx_base that is not yet
    // cumulatively acknowledged. `delivered` lists ranges the peer has received
    // out of order; `in_flight` the ranges sent in our last burst, by codeword.
    tx_base: u64,
    tx_buf: VecDeque<u8>,
    delivered: Vec<(u64, u64)>,
    in_flight: Vec<(u64, u64)>,
    /// Highest byte offset ever sent (anything below it is a resend).
    sent_high: u64,
    burst_id: u8,
    // Receive side.
    rx_next: u64,
    rx_ooo: BTreeMap<u64, Vec<u8>>,
    ack_burst: u8,
    ack_bitmap: u32,
    /// Level the peer asked us to use.
    peer_level: u8,
    /// Level we ask the peer to use.
    req_level: u8,
    phase: Phase,
    retries: u32,
    closing: Option<f64>,
    disc_tries: u32,
    last_heard: f64,
}

enum State {
    Idle,
    Connecting { session: u8, mycall: String, target: String, attempts: u32, next_tx: f64, sending: bool },
    Connected(Box<Session>),
}

pub struct Link {
    modem: Arc<Modem>,
    cfg: LinkConfig,
    mycalls: Vec<String>,
    listen: bool,
    state: State,
    events: Vec<LinkEvent>,
    last_buffer: usize,
    /// Report BUFFER at the next `take_events` even if unchanged (after host data).
    force_buffer: bool,
    rng: Prng,
    /// A burst to send once even though we are disconnected (Disc after ABORT, DiscAck re-sends).
    oneshot: Option<Burst>,
    /// Recently closed session, so a repeated Disc can still be answered.
    closed: Option<(u8, String, f64)>,
    last_tx: TxInfo,
    pub stats: LinkStats,
}

fn norm_call(c: &str) -> String {
    c.trim().to_ascii_uppercase()
}

impl Link {
    pub fn new(modem: Arc<Modem>, cfg: LinkConfig, seed: u32) -> Self {
        Self {
            modem,
            cfg,
            mycalls: Vec::new(),
            listen: false,
            state: State::Idle,
            events: Vec::new(),
            last_buffer: 0,
            force_buffer: false,
            rng: Prng::new(seed),
            oneshot: None,
            closed: None,
            last_tx: TxInfo::default(),
            stats: LinkStats::default(),
        }
    }

    pub fn config(&self) -> &LinkConfig {
        &self.cfg
    }

    pub fn set_mycalls(&mut self, calls: Vec<String>) {
        self.mycalls = calls;
    }

    pub fn set_listen(&mut self, on: bool) {
        self.listen = on;
    }

    pub fn is_connected(&self) -> bool {
        matches!(self.state, State::Connected(_))
    }

    pub fn is_idle(&self) -> bool {
        matches!(self.state, State::Idle)
    }

    pub fn last_tx(&self) -> &TxInfo {
        &self.last_tx
    }

    pub fn take_events(&mut self) -> Vec<LinkEvent> {
        let buf = match &self.state {
            State::Connected(s) => s.tx_buf.len(),
            _ => 0,
        };
        if buf != self.last_buffer || std::mem::take(&mut self.force_buffer) {
            self.last_buffer = buf;
            self.events.push(LinkEvent::Buffer(buf));
        }
        std::mem::take(&mut self.events)
    }

    fn rand_s(&mut self, max: f64) -> f64 {
        (self.rng.next_u32() as f64 / u32::MAX as f64) * max
    }

    // ------------------------------------------------------------ host commands

    /// Start a connection. Fails if the link is not idle.
    #[allow(clippy::result_unit_err)]
    pub fn connect(&mut self, now: f64, mycall: &str, target: &str) -> Result<(), ()> {
        if !self.is_idle() {
            return Err(());
        }
        let session = (self.rng.next_u32() % 255 + 1) as u8;
        self.state = State::Connecting {
            session,
            mycall: norm_call(mycall),
            target: norm_call(target),
            attempts: 0,
            next_tx: now,
            sending: false,
        };
        Ok(())
    }

    /// Graceful close: send everything still queued, then close the session.
    pub fn disconnect(&mut self, now: f64) {
        match &mut self.state {
            State::Idle => {}
            State::Connecting { .. } => self.close(),
            State::Connected(s) => {
                s.closing.get_or_insert(now);
            }
        }
    }

    /// Hard stop. Sends one `Disc` so the peer does not wait for a timeout.
    pub fn abort(&mut self, _now: f64) {
        if let State::Connected(s) = &self.state {
            self.oneshot = Some(Burst {
                header: Header { kind: Kind::Disc, session: s.id, ..Header::default() },
                payload: Vec::new(),
            });
        }
        if !self.is_idle() {
            self.close();
        }
    }

    /// Bytes from the host's data port. Ignored when not connected. Like VARA,
    /// data written after DISCONNECT is still sent (until our Disc goes out).
    /// Every accepted write is followed by a BUFFER report, so a host can tell
    /// that a report is newer than its write.
    pub fn send_data(&mut self, data: &[u8]) {
        if let State::Connected(s) = &mut self.state {
            if s.disc_tries == 0 {
                s.tx_buf.extend(data);
                self.force_buffer = true;
            }
        }
    }

    fn close(&mut self) {
        if let State::Connected(s) = &self.state {
            self.closed = Some((s.id, s.local.clone(), 0.0));
        }
        self.state = State::Idle;
        self.events.push(LinkEvent::Disconnected);
    }

    // ------------------------------------------------------------ receive

    fn level_for(&self, rx: &RxBurst) -> u8 {
        let by_snr = level_by_snr(&self.modem, rx.snr_db, rx.header.level);
        let failed = rx.payload.iter().any(|p| p.is_none());
        let cap = if failed { rx.header.level.saturating_sub(1) } else { u8::MAX };
        by_snr.min(cap).min(self.cfg.max_level)
    }

    /// Handle a decoded burst.
    pub fn on_burst(&mut self, now: f64, rx: &RxBurst) {
        let h = rx.header;
        self.stats.codewords_received += rx.payload.len() as u64;
        self.stats.codewords_failed += rx.payload.iter().filter(|p| p.is_none()).count() as u64;
        let req = self.level_for(rx);
        let turnaround = self.cfg.turnaround_s;

        match &mut self.state {
            State::Idle => {
                if h.kind == Kind::ConnectReq {
                    self.on_connect_req(now, rx, req);
                } else if h.kind == Kind::Disc {
                    // Peer did not hear our DiscAck: answer again.
                    if let Some((id, _, _)) = &self.closed {
                        if *id == h.session {
                            self.oneshot = Some(Burst {
                                header: Header { kind: Kind::DiscAck, session: h.session, ..Header::default() },
                                payload: Vec::new(),
                            });
                        }
                    }
                }
            }
            State::Connecting { session, mycall, target, .. } => {
                if h.session != *session {
                    return;
                }
                let accepted = match h.kind {
                    Kind::ConnectAck => parse_calls(rx).is_some_and(|(s, d)| s == *mycall && d == *target),
                    Kind::Data => true,
                    _ => false,
                };
                if !accepted {
                    return;
                }
                let (id, local, remote) = (*session, mycall.clone(), target.clone());
                self.events.push(LinkEvent::Connected { source: local.clone(), destination: remote.clone() });
                let mut s = self.new_session(id, local, remote, true, now, req, h.req_level);
                // Let the called station speak first.
                s.phase = Phase::Wait { deadline: now + self.cfg.response_timeout_s, retry: false };
                self.state = State::Connected(Box::new(s));
                if h.kind == Kind::Data {
                    self.on_session_burst(now, rx, req);
                }
            }
            State::Connected(s) => {
                if h.session != s.id {
                    return;
                }
                match h.kind {
                    Kind::ConnectReq if !s.caller => {
                        // Our ConnectAck was lost; send it again.
                        s.last_heard = now;
                        s.phase = Phase::Respond { at: now + turnaround, kind: Kind::ConnectAck, initiated: false };
                    }
                    Kind::ConnectReq | Kind::ConnectAck => {}
                    _ => self.on_session_burst(now, rx, req),
                }
            }
        }
    }

    fn on_connect_req(&mut self, now: f64, rx: &RxBurst, req: u8) {
        let Some((src, dst)) = parse_calls(rx) else { return };
        if !self.listen {
            return;
        }
        let Some(local) = self.mycalls.iter().find(|c| norm_call(c) == dst).cloned() else { return };
        self.events.push(LinkEvent::Pending);
        self.events.push(LinkEvent::Connected { source: src.clone(), destination: local.clone() });
        // The caller has not heard us yet, so its requested level is just a default.
        // Our own measurement of its request is a better guess (same radio path).
        let mut s = self.new_session(rx.header.session, local, src, false, now, req, req);
        s.phase = Phase::Respond { at: now + self.cfg.turnaround_s, kind: Kind::ConnectAck, initiated: false };
        self.state = State::Connected(Box::new(s));
    }

    #[allow(clippy::too_many_arguments)]
    fn new_session(
        &self,
        id: u8,
        local: String,
        remote: String,
        caller: bool,
        now: f64,
        req: u8,
        peer_req: u8,
    ) -> Session {
        Session {
            id,
            local,
            remote,
            caller,
            tx_base: 0,
            tx_buf: VecDeque::new(),
            delivered: Vec::new(),
            in_flight: Vec::new(),
            sent_high: 0,
            burst_id: 0,
            rx_next: 0,
            rx_ooo: BTreeMap::new(),
            ack_burst: 0,
            ack_bitmap: 0,
            peer_level: peer_req.min(self.cfg.max_level),
            req_level: req,
            phase: Phase::Idle,
            retries: 0,
            closing: None,
            disc_tries: 0,
            last_heard: now,
        }
    }

    fn on_session_burst(&mut self, now: f64, rx: &RxBurst, req: u8) {
        let h = rx.header;
        let turnaround = self.cfg.turnaround_s;
        let State::Connected(s) = &mut self.state else { return };
        s.last_heard = now;
        s.retries = 0;
        s.peer_level = h.req_level.min(self.cfg.max_level);
        s.req_level = req;

        // Acknowledgements for our data.
        if h.ack_burst == s.burst_id {
            for (i, r) in s.in_flight.iter().enumerate() {
                if h.bitmap & (1 << i) != 0 {
                    s.delivered.push(*r);
                }
            }
            s.in_flight.clear();
            s.delivered = merge(std::mem::take(&mut s.delivered));
        }
        let acked = expand(h.ack_offset, s.tx_base);
        if acked > s.tx_base && acked <= s.tx_base + s.tx_buf.len() as u64 {
            s.tx_buf.drain(..(acked - s.tx_base) as usize);
            s.tx_base = acked;
        }
        s.delivered.retain(|&(_, end)| end > s.tx_base);

        // Data from the peer.
        let mut bitmap = 0u32;
        let mut delivered = Vec::new();
        for (i, cw) in rx.payload.iter().enumerate() {
            let Some(cw) = cw else { continue };
            bitmap |= 1 << i;
            let off = u32::from_be_bytes([0, cw[0], cw[1], cw[2]]);
            let len = (cw[3] as usize).min(cw.len() - SEGMENT_HEADER);
            let start = expand(off, s.rx_next);
            let data = &cw[SEGMENT_HEADER..SEGMENT_HEADER + len];
            if start + len as u64 > s.rx_next && len > 0 {
                s.rx_ooo.entry(start).or_insert_with(|| data.to_vec());
            }
        }
        while let Some((&start, _)) = s.rx_ooo.iter().next() {
            if start > s.rx_next {
                break;
            }
            let data = s.rx_ooo.remove(&start).unwrap();
            let end = start + data.len() as u64;
            if end > s.rx_next {
                delivered.extend_from_slice(&data[(s.rx_next - start) as usize..]);
                s.rx_next = end;
            }
        }
        s.ack_burst = h.burst_id;
        s.ack_bitmap = bitmap;

        match h.kind {
            Kind::Disc => {
                s.phase = Phase::Respond { at: now + turnaround, kind: Kind::DiscAck, initiated: false };
            }
            Kind::DiscAck => {
                if s.closing.is_some() {
                    self.stats.bytes_delivered += delivered.len() as u64;
                    if !delivered.is_empty() {
                        self.events.push(LinkEvent::Data(delivered));
                    }
                    self.close();
                    return;
                }
            }
            _ => {
                let has_data = s.has_data();
                s.phase = if h.ncw > 0 || h.flags & FLAG_POLL != 0 || has_data {
                    Phase::Respond { at: now + turnaround, kind: Kind::Data, initiated: false }
                } else {
                    Phase::Idle
                };
            }
        }
        self.stats.bytes_delivered += delivered.len() as u64;
        if !delivered.is_empty() {
            self.events.push(LinkEvent::Data(delivered));
        }
    }

    // ------------------------------------------------------------ transmit

    /// Called regularly while not transmitting. Returns a burst to transmit now.
    pub fn poll(&mut self, now: f64, busy: bool) -> Option<Burst> {
        if let Some(b) = self.oneshot.take() {
            self.last_tx = TxInfo { kind: b.header.kind, ..TxInfo::default() };
            self.stats.bursts_sent += 1;
            return Some(b);
        }
        let cfg = self.cfg.clone();
        let backoff = self.rand_s(0.4);
        match &mut self.state {
            State::Idle => None,
            State::Connecting { session, mycall, target, attempts, next_tx, sending } => {
                if *sending {
                    return None;
                }
                if busy {
                    *next_tx = next_tx.max(now + 0.3);
                    return None;
                }
                if now < *next_tx {
                    return None;
                }
                if *attempts >= cfg.connect_attempts {
                    self.close();
                    return None;
                }
                *attempts += 1;
                *sending = true;
                let burst = Burst {
                    header: Header {
                        kind: Kind::ConnectReq,
                        session: *session,
                        level: 0,
                        ncw: 1,
                        req_level: cfg.start_level,
                        ..Header::default()
                    },
                    payload: vec![calls_payload(mycall, target)],
                };
                self.last_tx = TxInfo { kind: Kind::ConnectReq, ncw: 1, retry: *attempts - 1, ..TxInfo::default() };
                self.stats.bursts_sent += 1;
                Some(burst)
            }
            State::Connected(s) => {
                // Advance timers.
                match s.phase {
                    Phase::Sending { .. } => return None,
                    Phase::Wait { deadline, retry } => {
                        if busy {
                            s.phase = Phase::Wait { deadline: deadline.max(now + 0.3), retry };
                            return None;
                        }
                        if now < deadline {
                            return None;
                        }
                        if retry {
                            s.retries += 1;
                            self.stats.retries += 1;
                            if s.retries > cfg.max_retries || (s.closing.is_some() && s.disc_tries >= 4) {
                                self.close();
                                return None;
                            }
                            // Unanswered: the peer may not have decoded us. Step down.
                            s.peer_level = s.peer_level.saturating_sub(1);
                            let kind = if s.closing.is_some() && s.disc_tries > 0 { Kind::Disc } else { Kind::Data };
                            s.phase = Phase::Respond { at: now + backoff, kind, initiated: false };
                        } else {
                            s.phase = Phase::Idle;
                        }
                        return None;
                    }
                    Phase::Respond { at, .. } if now < at => return None,
                    Phase::Respond { .. } => {}
                    Phase::Idle => {
                        if busy {
                            return None;
                        }
                        let ready_to_close =
                            s.closing.is_some_and(|t| now >= t + cfg.disconnect_grace_s) && s.tx_buf.is_empty();
                        if ready_to_close {
                            s.phase = Phase::Respond { at: now, kind: Kind::Disc, initiated: true };
                        } else if s.has_data()
                            || now - s.last_heard > cfg.keepalive_s * if s.caller { 1.0 } else { 1.5 }
                        {
                            // Start a burst on a quiet channel, after a short random wait.
                            s.phase =
                                Phase::Respond { at: now + 0.05 + backoff * 0.5, kind: Kind::Data, initiated: true };
                        }
                        return None;
                    }
                }
                let Phase::Respond { kind, initiated, .. } = s.phase else { return None };
                if busy && initiated {
                    // Someone else started talking; go back to listening.
                    s.phase = Phase::Idle;
                    return None;
                }
                let retry = s.retries;
                let (burst, info) = s.build(kind, &self.modem, now, cfg.keepalive_s);
                let needs_reply = matches!(kind, Kind::Disc)
                    || burst.header.ncw > 0 && kind == Kind::Data
                    || burst.header.flags & FLAG_POLL != 0;
                if kind == Kind::Disc {
                    s.disc_tries += 1;
                }
                s.phase = Phase::Sending { kind, retry: needs_reply };
                self.stats.bursts_sent += 1;
                self.stats.payload_bytes_sent += info.bytes as u64;
                self.stats.bytes_resent += info.resent_bytes as u64;
                self.last_tx = TxInfo { retry, ..info };
                Some(burst)
            }
        }
    }

    /// Called when the burst returned by `poll` has finished transmitting.
    pub fn tx_done(&mut self, now: f64) {
        let timeout = self.cfg.response_timeout_s;
        match &mut self.state {
            State::Connecting { next_tx, sending, .. } => {
                *sending = false;
                *next_tx = now + timeout + self.rng.next_u32() as f64 / u32::MAX as f64 * 0.5;
            }
            State::Connected(s) => {
                if let Phase::Sending { kind, retry } = s.phase {
                    match kind {
                        Kind::DiscAck => {
                            self.close();
                        }
                        Kind::ConnectAck => {
                            // We may speak first: go straight to idle.
                            s.phase = Phase::Idle;
                        }
                        _ => s.phase = Phase::Wait { deadline: now + timeout, retry },
                    }
                }
            }
            State::Idle => {}
        }
    }
}

impl Session {
    /// True if there are bytes the peer has not received.
    fn has_data(&self) -> bool {
        let covered: u64 = self.delivered.iter().map(|(a, b)| b - a.max(&self.tx_base)).sum();
        (self.tx_buf.len() as u64) > covered
    }

    /// Byte ranges to send, in order, skipping ranges the peer already has.
    fn pending_ranges(&self) -> Vec<(u64, u64)> {
        let mut out = Vec::new();
        let mut pos = self.tx_base;
        let end = self.tx_base + self.tx_buf.len() as u64;
        let mut d = self.delivered.clone();
        d.sort_unstable();
        for (a, b) in d {
            if a > pos {
                out.push((pos, a.min(end)));
            }
            pos = pos.max(b);
        }
        if pos < end {
            out.push((pos, end));
        }
        out
    }

    fn build(&mut self, kind: Kind, modem: &Modem, now: f64, keepalive: f64) -> (Burst, TxInfo) {
        self.burst_id = (self.burst_id + 1) % 8;
        let level = if kind == Kind::ConnectAck { 0 } else { self.peer_level };
        let mut payload = Vec::new();
        if kind == Kind::ConnectAck {
            payload.push(calls_payload(&self.remote, &self.local));
        }
        let mut info = TxInfo { kind, level, ..TxInfo::default() };
        self.in_flight.clear();
        if kind == Kind::Data {
            let cw_bytes = Modem::payload_bytes(level);
            let cap = cw_bytes - SEGMENT_HEADER;
            let max = modem.max_ncw(level);
            'outer: for (a, b) in self.pending_ranges() {
                let mut pos = a;
                while pos < b {
                    if payload.len() >= max {
                        break 'outer;
                    }
                    let len = ((b - pos) as usize).min(cap);
                    let off = (pos % OFFSET_MOD) as u32;
                    let mut cw = Vec::with_capacity(cw_bytes);
                    cw.extend_from_slice(&off.to_be_bytes()[1..]);
                    cw.push(len as u8);
                    let i = (pos - self.tx_base) as usize;
                    cw.extend(self.tx_buf.range(i..i + len));
                    cw.resize(cw_bytes, 0);
                    payload.push(cw);
                    self.in_flight.push((pos, pos + len as u64));
                    info.bytes += len;
                    info.resent_bytes += (self.sent_high.min(pos + len as u64)).saturating_sub(pos) as usize;
                    self.sent_high = self.sent_high.max(pos + len as u64);
                    pos += len as u64;
                }
            }
        }
        info.ncw = payload.len();
        let poll = kind == Kind::Data && payload.is_empty() && now - self.last_heard > keepalive;
        let header = Header {
            kind,
            session: self.id,
            level: if payload.is_empty() { 0 } else { level },
            ncw: payload.len() as u8,
            req_level: self.req_level,
            burst_id: self.burst_id,
            ack_burst: self.ack_burst,
            ack_offset: (self.rx_next % OFFSET_MOD) as u32,
            bitmap: self.ack_bitmap,
            flags: if poll { FLAG_POLL } else { 0 },
        };
        (Burst { header, payload }, info)
    }
}

/// Sort and merge overlapping or touching ranges.
fn merge(mut r: Vec<(u64, u64)>) -> Vec<(u64, u64)> {
    r.sort_unstable();
    let mut out: Vec<(u64, u64)> = Vec::with_capacity(r.len());
    for (a, b) in r {
        match out.last_mut() {
            Some(last) if a <= last.1 => last.1 = last.1.max(b),
            _ => out.push((a, b)),
        }
    }
    out
}

/// Expand a 24-bit wrapped offset to the 64-bit value closest to `near`.
fn expand(off: u32, near: u64) -> u64 {
    let base = near - near % OFFSET_MOD;
    let cands = [base.wrapping_sub(OFFSET_MOD), base, base + OFFSET_MOD];
    cands.into_iter().map(|c| c.wrapping_add(off as u64)).min_by_key(|&c| c.abs_diff(near)).unwrap()
}

/// ConnectReq/ConnectAck payload: "VFM1" | len | source | len | destination, zero padded.
fn calls_payload(src: &str, dst: &str) -> Vec<u8> {
    let mut p = b"VFM1".to_vec();
    for c in [src, dst] {
        p.push(c.len() as u8);
        p.extend_from_slice(c.as_bytes());
    }
    p.resize(Modem::payload_bytes(0), 0);
    p
}

fn parse_calls(rx: &RxBurst) -> Option<(String, String)> {
    let p = rx.payload.first()?.as_ref()?;
    if !p.starts_with(b"VFM1") {
        return None;
    }
    let mut i = 4;
    let mut calls = Vec::new();
    for _ in 0..2 {
        let n = *p.get(i)? as usize;
        let s = std::str::from_utf8(p.get(i + 1..i + 1 + n)?).ok()?;
        calls.push(norm_call(s));
        i += 1 + n;
    }
    Some((calls[0].clone(), calls[1].clone()))
}

#[cfg(test)]
mod tests;
