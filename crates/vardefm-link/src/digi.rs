//! Digipeater: repeats the bursts of sessions routed through one of its call signs.
//!
//! A pure state machine like [`crate::Link`]: feed it every decoded burst, and ask
//! [`Digipeater::poll`] for repeats that are due.
//!
//! The route is learned from the `VFM2` ConnectReq/ConnectAck payload, which lists the
//! digipeaters from the calling station. Later bursts carry only the session id, a
//! direction flag and the route field, so the digipeater keeps a table of the sessions
//! it relays. Every burst of a routed session is marked routed; a direct session's
//! bursts are not, and are never repeated. The digipeater at position `i` (1 = next
//! to the caller) of `n` repeats a burst towards the called station when it has been
//! repeated `i - 1` times, and one towards the caller when `n - i` times, adding one
//! repeat. The ARQ stays end to
//! end: a codeword the digipeater could not decode goes out as a void segment, which
//! the far end counts as failed, so the sender resends it.

use super::{norm_call, parse_calls_payload, void_codeword};
use std::collections::HashMap;
use vardefm_modem::{Burst, Kind, RxBurst};

// The three lifetimes below must outlast the end stations' timers at the largest hop
// margin (`MAX_HOP_MARGIN_S`); `lifetimes_outlast_the_end_stations_timers` checks them
// against the link's own waits.

/// A session is forgotten after this long without a burst: longer than a quiet
/// session goes before the called station probes it.
const IDLE_EXPIRY_S: f64 = 600.0;
/// After the closing DiscAck, kept this long after the last Disc or DiscAck we repeat,
/// so a resent Disc and its answer still get through.
const CLOSE_EXPIRY_S: f64 = 60.0;
/// Life of a session not yet answered (no ConnectAck or burst from the called station
/// repeated). A stale entry would repeat the bursts of a later routed session that
/// reuses its id if we missed that session's connect bursts, so it must not wait
/// for `IDLE_EXPIRY_S`. Every ConnectReq naming us renews it, so it has to outlast
/// two of the caller's attempts: one request we fail to decode must not drop a
/// session still connecting.
const UNCONFIRMED_EXPIRY_S: f64 = 90.0;

#[derive(Clone, Debug)]
struct Entry {
    /// Our positions on the path, 1-based from the calling station. A path may name
    /// us more than once (the same call twice, or two of our calls).
    pos: Vec<u8>,
    /// Number of digipeaters on the path.
    hops: u8,
    /// We have repeated the called station's answer (its ConnectAck or any burst with
    /// UP set). Until then the entry lives only `UNCONFIRMED_EXPIRY_S`.
    confirmed: bool,
    /// We have repeated a DiscAck: the session is over. Only a resent Disc and its
    /// DiscAck can follow, and the entry lives only `CLOSE_EXPIRY_S`.
    closing: bool,
    expires: f64,
}

impl Entry {
    fn life_s(&self) -> f64 {
        if self.closing {
            CLOSE_EXPIRY_S
        } else if self.confirmed {
            IDLE_EXPIRY_S
        } else {
            UNCONFIRMED_EXPIRY_S
        }
    }
}

pub struct Digipeater {
    calls: Vec<String>,
    turnaround_s: f64,
    table: HashMap<u8, Entry>,
    /// Repeats waiting for their time, in order.
    queue: Vec<(f64, Burst)>,
    repeated: u64,
}

impl Digipeater {
    /// A digipeater for `calls`, repeating `turnaround_s` after each burst ends.
    pub fn new(calls: &[String], turnaround_s: f64) -> Self {
        Self {
            calls: calls.iter().map(|c| norm_call(c)).collect(),
            turnaround_s,
            table: HashMap::new(),
            queue: Vec::new(),
            repeated: 0,
        }
    }

    /// Bursts repeated so far.
    pub fn repeated(&self) -> u64 {
        self.repeated
    }

    /// Sessions relayed at `now`.
    #[cfg(test)]
    fn sessions(&self, now: f64) -> usize {
        self.table.values().filter(|e| e.expires > now).count()
    }

    /// Handle a decoded burst; queue its repeat if it is ours to repeat.
    pub fn on_burst(&mut self, now: f64, rx: &RxBurst) {
        self.table.retain(|_, e| e.expires > now);
        let h = rx.header;
        // A direct session's bursts are unrouted and never ours, whatever their id.
        if !h.routed() {
            return;
        }
        if matches!(h.kind, Kind::ConnectReq | Kind::ConnectAck) {
            // The route is in the payload. Without it we cannot repeat the burst.
            let Some(Some(p)) = rx.payload.first() else { return };
            let Some((_, _, digis)) = parse_calls_payload(p) else { return };
            let pos: Vec<u8> =
                digis.iter().enumerate().filter(|(_, d)| self.calls.contains(d)).map(|(i, _)| i as u8 + 1).collect();
            if pos.is_empty() {
                // The id now belongs to a routed session through other digipeaters. An
                // entry left from an earlier session with this id could repeat its bursts
                // where the repeat counts happen to line up.
                self.table.remove(&h.session);
                return;
            }
            let hops = digis.len() as u8;
            match self.table.get_mut(&h.session) {
                // The same route again (a resent request or ack): keep what we know. On a
                // closed session it is a new session with the same id, so start afresh.
                Some(e) if e.pos == pos && e.hops == hops && !e.closing => e.expires = e.expires.max(now + e.life_s()),
                _ => {
                    let e = Entry { pos, hops, confirmed: false, closing: false, expires: now + UNCONFIRMED_EXPIRY_S };
                    self.table.insert(h.session, e);
                }
            }
        }
        let Some(e) = self.table.get_mut(&h.session) else { return };
        // Our positions counted in the burst's direction of travel. The burst is ours if
        // the next position is one of them. We never hear our own repeat, so when the
        // path names us at several positions in a row we pass all of them at once.
        let along = |p: u8| if h.up() { e.hops + 1 - p } else { p };
        let mine = |p: u8| e.pos.iter().any(|&q| along(q) == p);
        let mut hop = h.hop();
        while hop < e.hops && mine(hop + 1) {
            hop += 1;
        }
        if hop == h.hop() {
            return;
        }
        // Only the called station sends ConnectAck or sets UP: the session was answered.
        let answer = h.kind == Kind::ConnectAck || h.up();
        // Until the answer has come through us the caller sends only requests. Anything
        // else from its side is another routed session using the id whose connect bursts
        // we did not decode: repeating it would also keep this entry alive.
        if !e.confirmed && !answer && h.kind != Kind::ConnectReq {
            return;
        }
        // After the DiscAck the session is over. A resent Disc must not bring back the
        // idle lifetime, and anything else is another session using the id.
        if e.closing && !matches!(h.kind, Kind::Disc | Kind::DiscAck) {
            return;
        }
        e.confirmed |= answer;
        e.closing |= h.kind == Kind::DiscAck;
        e.expires = now + e.life_s();
        let mut header = h;
        header.set_hop(hop);
        let payload = rx.payload.iter().map(|cw| cw.clone().unwrap_or_else(|| void_codeword(h.level))).collect();
        let burst = Burst { header, payload };
        // The same burst heard twice (it cannot normally be) is repeated once.
        if self.queue.iter().any(|(_, b)| *b == burst) {
            return;
        }
        self.queue.push((now + self.turnaround_s, burst));
    }

    /// The next repeat that is due. Repeats answer a burst just heard, so they go out
    /// whether or not the channel is busy, like the link's answers.
    pub fn poll(&mut self, now: f64) -> Option<Burst> {
        let i = self.queue.iter().position(|(at, _)| *at <= now)?;
        self.repeated += 1;
        Some(self.queue.remove(i).1)
    }

    /// True if a repeat is waiting.
    pub fn pending(&self) -> bool {
        !self.queue.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{calls_payload, Link, LinkConfig, CONNECT_JITTER_S, MAX_HOP_MARGIN_S, RETRY_BACKOFF_S};
    use std::sync::Arc;
    use vardefm_modem::{AudioLevel, Header, Modem, OfdmParams, FLAG_UP, MAX_DIGIS};

    /// A burst of a direct session: route field 0, no UP, as released versions send.
    fn direct(kind: Kind, session: u8, payload: Vec<Option<Vec<u8>>>) -> RxBurst {
        let header = Header { kind, session, ncw: payload.len() as u8, ..Header::default() };
        RxBurst { header, payload, snr_db: 20.0, end_sample: 0, level: AudioLevel::default() }
    }

    /// A burst of a routed session, repeated `hop` times so far.
    fn rx(kind: Kind, session: u8, up: bool, hop: u8, payload: Vec<Option<Vec<u8>>>) -> RxBurst {
        let mut b = direct(kind, session, payload);
        b.header.flags = if up { FLAG_UP } else { 0 };
        b.header.set_hop(hop);
        b
    }

    fn path(digis: &[&str]) -> Vec<String> {
        digis.iter().map(|s| s.to_string()).collect()
    }

    fn connect_req(session: u8, digis: &[&str], hop: u8) -> RxBurst {
        rx(Kind::ConnectReq, session, false, hop, vec![Some(calls_payload("LA1AAA", "LA2BBB", &path(digis)))])
    }

    /// A direct (VFM1) request or acknowledgement.
    fn direct_connect(kind: Kind, session: u8) -> RxBurst {
        direct(kind, session, vec![Some(calls_payload("LA5X", "LA6Y", &[]))])
    }

    /// Repeats `b` and returns the repeated header, if any.
    fn repeat(d: &mut Digipeater, now: f64, b: &RxBurst) -> Option<Header> {
        d.on_burst(now, b);
        d.poll(now + 1.0).map(|b| b.header)
    }

    #[test]
    fn position_rule_both_directions() {
        for (n, digis) in [(1u8, &["LA3D1"][..]), (2, &["LA3D1", "LA3D2"])] {
            for pos in 1..=n {
                let me = digis[pos as usize - 1];
                let mut d = Digipeater::new(&[me.to_string()], 0.2);
                let h = repeat(&mut d, 0.0, &connect_req(7, digis, pos - 1)).expect("repeats its ConnectReq");
                assert_eq!((h.hop(), h.up()), (pos, false));
                let a = repeat(&mut d, 0.5, &connect_ack(7, digis, n - pos)).expect("repeats its ConnectAck");
                assert_eq!((a.hop(), a.up()), (n - pos + 1, true));
                // Direct Data with the session's id is never ours, at any position.
                assert!(repeat(&mut d, 1.0, &direct(Kind::Data, 7, vec![])).is_none(), "n {n} pos {pos} direct");
                for hop in 0..=MAX_DIGIS as u8 {
                    let down = repeat(&mut d, 1.0, &rx(Kind::Data, 7, false, hop, vec![]));
                    assert_eq!(down.is_some(), hop + 1 == pos, "n {n} pos {pos} down hop {hop}");
                    let up = repeat(&mut d, 2.0, &rx(Kind::Data, 7, true, hop, vec![]));
                    assert_eq!(up.is_some(), hop == n - pos, "n {n} pos {pos} up hop {hop}");
                    if let Some(up) = up {
                        assert_eq!((up.hop(), up.up()), (hop + 1, true));
                    }
                }
            }
        }
    }

    #[test]
    fn ignores_sessions_not_routed_through_it() {
        let mut d = Digipeater::new(&["LA9XYZ".to_string()], 0.2);
        assert!(repeat(&mut d, 0.0, &connect_req(7, &["LA3D1"], 0)).is_none());
        assert!(repeat(&mut d, 0.0, &rx(Kind::Data, 7, false, 0, vec![])).is_none());
        // A direct (VFM1) request names no digipeater.
        assert!(repeat(&mut d, 0.0, &direct_connect(Kind::ConnectReq, 8)).is_none());
        assert_eq!(d.sessions(0.0), 0);
    }

    #[test]
    fn failed_codewords_become_voids() {
        let mut d = Digipeater::new(&["LA3D1".to_string()], 0.2);
        repeat(&mut d, 0.0, &connect_req(7, &["LA3D1"], 0)).unwrap();
        repeat(&mut d, 0.5, &connect_ack(7, &["LA3D1"], 0)).unwrap();
        let len = Modem::payload_bytes(0);
        let good = vec![1u8; len];
        d.on_burst(1.0, &rx(Kind::Data, 7, false, 0, vec![Some(good.clone()), None]));
        assert!(d.poll(1.1).is_none(), "waits for the turnaround");
        let b = d.poll(1.2).unwrap();
        assert_eq!(b.payload[0], good);
        assert_eq!(b.payload[1], void_codeword(0));
        // A ConnectReq whose codeword failed cannot be repeated, and tells us nothing
        // about the session: the entry stays.
        assert!(repeat(&mut d, 2.0, &rx(Kind::ConnectReq, 7, false, 0, vec![None])).is_none());
        assert_eq!(d.sessions(2.0), 1);
    }

    #[test]
    fn sessions_expire() {
        let mut d = Digipeater::new(&["LA3D1".to_string()], 0.2);
        repeat(&mut d, 0.0, &connect_req(7, &["LA3D1"], 0)).unwrap();
        // Answered: an unanswered request would expire after UNCONFIRMED_EXPIRY_S.
        repeat(&mut d, 5.0, &connect_ack(7, &["LA3D1"], 0)).unwrap();
        assert!(repeat(&mut d, 100.0, &rx(Kind::Data, 7, false, 0, vec![])).is_some());
        assert!(repeat(&mut d, 100.0 + IDLE_EXPIRY_S - 1.0, &rx(Kind::Data, 7, true, 0, vec![])).is_some());
        let late = 100.0 + 2.0 * IDLE_EXPIRY_S;
        assert!(repeat(&mut d, late, &rx(Kind::Data, 7, false, 0, vec![])).is_none(), "idle too long");

        let t = late + 10.0;
        repeat(&mut d, t, &connect_req(9, &["LA3D1"], 0)).unwrap();
        assert!(repeat(&mut d, t + 1.0, &rx(Kind::DiscAck, 9, true, 0, vec![])).is_some());
        assert!(repeat(&mut d, t + 1.0 + CLOSE_EXPIRY_S + 1.0, &rx(Kind::Disc, 9, false, 0, vec![])).is_none());
    }

    #[test]
    fn path_naming_us_twice() {
        // `VIA X X`: one repeat passes both positions.
        let mut d = Digipeater::new(&["LA3D1".to_string()], 0.2);
        let h = repeat(&mut d, 0.0, &connect_req(7, &["LA3D1", "LA3D1"], 0)).unwrap();
        assert_eq!(h.hop(), 2);
        assert_eq!(repeat(&mut d, 1.0, &rx(Kind::Data, 7, true, 0, vec![])).unwrap().hop(), 2);
        assert!(repeat(&mut d, 2.0, &rx(Kind::Data, 7, false, 2, vec![])).is_none(), "already past us");

        // Two calls of one station in a row: `VIA X-1 X-2`, also passed in one repeat.
        let mut d = Digipeater::new(&["LA3X-1".to_string(), "LA3X-2".to_string()], 0.2);
        let path = ["LA3X-1", "LA3X-2"];
        assert_eq!(repeat(&mut d, 0.0, &connect_req(8, &path, 0)).unwrap().hop(), 2);
        assert_eq!(repeat(&mut d, 1.0, &rx(Kind::Data, 8, true, 0, vec![])).unwrap().hop(), 2);
        assert!(repeat(&mut d, 2.0, &rx(Kind::Data, 8, false, 2, vec![])).is_none(), "already past us");
        assert!(repeat(&mut d, 3.0, &rx(Kind::Data, 8, true, 2, vec![])).is_none(), "already past us");

        // Only our second call on the path, after another digipeater: `VIA Y X-2`.
        let path = ["LA3Y", "LA3X-2"];
        assert!(repeat(&mut d, 10.0, &connect_req(9, &path, 0)).is_none(), "Y's turn");
        assert_eq!(repeat(&mut d, 11.0, &connect_req(9, &path, 1)).unwrap().hop(), 2);
        assert_eq!(repeat(&mut d, 12.0, &rx(Kind::Data, 9, true, 0, vec![])).unwrap().hop(), 1);
        assert!(repeat(&mut d, 13.0, &rx(Kind::Data, 9, true, 1, vec![])).is_none(), "Y's turn");
    }

    fn connect_ack(session: u8, digis: &[&str], hop: u8) -> RxBurst {
        rx(Kind::ConnectAck, session, true, hop, vec![Some(calls_payload("LA1AAA", "LA2BBB", &path(digis)))])
    }

    /// A routed ConnectReq for session 7 through us that nobody answers.
    fn stale_entry() -> Digipeater {
        let mut d = Digipeater::new(&["LA3D1".to_string()], 0.2);
        repeat(&mut d, 0.0, &connect_req(7, &["LA3D1"], 0)).expect("repeats the routed request");
        d
    }

    /// Direct sessions send an unrouted header both ways: never ours to repeat.
    fn direct_data_not_repeated(d: &mut Digipeater, t: f64) {
        for i in 1..=3 {
            let at = t + i as f64;
            assert!(repeat(d, at, &direct(Kind::Data, 7, vec![])).is_none(), "direct Data at {at} repeated");
        }
    }

    #[test]
    fn direct_request_leaves_entry() {
        // A direct session picks the same id while ours is still connecting. Its header
        // says it is direct, so it cannot be mistaken for ours and tells us nothing.
        let mut d = stale_entry();
        assert!(repeat(&mut d, 3.0, &direct_connect(Kind::ConnectReq, 7)).is_none());
        assert_eq!(d.sessions(3.0), 1);
        direct_data_not_repeated(&mut d, 3.0);
        // Our caller's resent request is still repeated.
        assert_eq!(repeat(&mut d, 10.0, &connect_req(7, &["LA3D1"], 0)).unwrap().hop(), 1);
    }

    #[test]
    fn direct_ack_leaves_entry() {
        // Only the called station's ConnectAck of the direct session is heard.
        let mut d = stale_entry();
        assert!(repeat(&mut d, 3.0, &direct_connect(Kind::ConnectAck, 7)).is_none());
        assert_eq!(d.sessions(3.0), 1);
        direct_data_not_repeated(&mut d, 3.0);
    }

    #[test]
    fn live_session_survives_direct_connect() {
        // A confirmed session through us, then a direct session takes the same id.
        let mut d = stale_entry();
        assert_eq!(repeat(&mut d, 5.0, &connect_ack(7, &["LA3D1"], 0)).unwrap().hop(), 1);
        assert!(repeat(&mut d, 10.0, &direct_connect(Kind::ConnectReq, 7)).is_none());
        assert!(repeat(&mut d, 12.0, &direct_connect(Kind::ConnectAck, 7)).is_none());
        direct_data_not_repeated(&mut d, 12.0);
        assert_eq!(d.sessions(15.0), 1);
        // Ours keeps being repeated both ways.
        let down = repeat(&mut d, 20.0, &rx(Kind::Data, 7, false, 0, vec![])).expect("repeats the caller");
        assert_eq!((down.hop(), down.up(), down.routed()), (1, false, true));
        let up = repeat(&mut d, 21.0, &rx(Kind::Data, 7, true, 0, vec![])).expect("repeats the called station");
        assert_eq!((up.hop(), up.up(), up.routed()), (1, true, true));
    }

    #[test]
    fn request_through_others_drops_stale_entry() {
        let mut d = stale_entry();
        // A new routed session through other digipeaters, heard as the request and its repeat.
        assert!(repeat(&mut d, 3.0, &connect_req(7, &["LA3D8", "LA3D9"], 0)).is_none());
        assert_eq!(d.sessions(3.0), 0);
        assert!(repeat(&mut d, 4.0, &connect_req(7, &["LA3D8", "LA3D9"], 1)).is_none());
        // Its acknowledgement as well: a fresh stale entry, then only the ConnectAck heard.
        let mut d = stale_entry();
        assert!(repeat(&mut d, 3.0, &connect_ack(7, &["LA3D8", "LA3D9"], 0)).is_none());
        assert_eq!(d.sessions(3.0), 0);
        assert!(repeat(&mut d, 5.0, &rx(Kind::Data, 7, false, 0, vec![])).is_none());
    }

    #[test]
    fn same_id_new_path_through_us_replaces_entry() {
        // Session 7 through us at position 1, then a new session 7 with us at position 2.
        let mut d = stale_entry();
        assert_eq!(repeat(&mut d, 300.0, &connect_req(7, &["LA3D8", "LA3D1"], 1)).unwrap().hop(), 2);
        assert!(repeat(&mut d, 301.0, &rx(Kind::Data, 7, false, 0, vec![])).is_none(), "LA3D8's turn");
        assert_eq!(repeat(&mut d, 302.0, &rx(Kind::Data, 7, true, 0, vec![])).unwrap().hop(), 1);
    }

    #[test]
    fn unanswered_request_expires_early() {
        let mut d = stale_entry();
        // The caller's resent requests, 32 s apart (2 digipeaters, 10 s hop margin), are
        // still repeated, and the entry outlasts one of them we fail to decode.
        for t in [32.0, 64.0, 96.0, 128.0] {
            assert!(repeat(&mut d, t, &connect_req(7, &["LA3D1"], 0)).is_some(), "resent request at {t}");
        }
        assert!(repeat(&mut d, 160.0, &rx(Kind::ConnectReq, 7, false, 0, vec![None])).is_none());
        assert_eq!(d.sessions(160.0), 1, "one undecoded request");
        assert!(repeat(&mut d, 192.0, &connect_req(7, &["LA3D1"], 0)).is_some());
        // Once the caller gives up, the entry does not wait for IDLE_EXPIRY_S.
        let t = 192.0 + UNCONFIRMED_EXPIRY_S + 1.0;
        assert!(repeat(&mut d, t, &rx(Kind::Data, 7, false, 0, vec![])).is_none());
        assert_eq!(d.sessions(t), 0);
    }

    #[test]
    fn unanswered_entry_repeats_only_requests() {
        // Another routed session reuses id 7 and we decode neither its ConnectReq nor its
        // ConnectAck. Its caller's unrepeated bursts look like ours, but our caller has
        // had no answer.
        let mut d = stale_entry();
        for (t, kind) in [(10.0, Kind::Data), (40.0, Kind::Data), (70.0, Kind::Disc), (80.0, Kind::DiscAck)] {
            assert!(repeat(&mut d, t, &rx(kind, 7, false, 0, vec![])).is_none(), "{kind:?} at {t} repeated");
        }
        assert_eq!(d.sessions(80.0), 1);
        // They do not keep the entry alive either. It stops counting when it expires,
        // not only when the next burst clears it out.
        assert_eq!(d.sessions(UNCONFIRMED_EXPIRY_S + 1.0), 0);
        d.on_burst(UNCONFIRMED_EXPIRY_S + 1.0, &rx(Kind::Data, 7, false, 0, vec![]));
        assert_eq!(d.sessions(UNCONFIRMED_EXPIRY_S + 1.0), 0);
    }

    #[test]
    fn confirmed_session_lives_for_idle_expiry() {
        let mut d = stale_entry();
        // Answered: the ConnectAck comes back through us.
        assert_eq!(repeat(&mut d, 5.0, &connect_ack(7, &["LA3D1"], 0)).unwrap().hop(), 1);
        // A resent request (the caller missed our repeat of the ack) keeps it confirmed.
        assert!(repeat(&mut d, 15.0, &connect_req(7, &["LA3D1"], 0)).is_some());
        let t = 15.0 + IDLE_EXPIRY_S - 1.0;
        assert!(repeat(&mut d, t, &rx(Kind::Data, 7, false, 0, vec![])).is_some(), "quiet but confirmed");
        assert!(repeat(&mut d, t + IDLE_EXPIRY_S + 1.0, &rx(Kind::Data, 7, false, 0, vec![])).is_none());
    }

    #[test]
    fn closed_session_keeps_the_close_window() {
        let mut d = stale_entry();
        assert!(repeat(&mut d, 5.0, &connect_ack(7, &["LA3D1"], 0)).is_some());
        assert!(repeat(&mut d, 100.0, &rx(Kind::Disc, 7, false, 0, vec![])).is_some());
        assert!(repeat(&mut d, 103.0, &rx(Kind::DiscAck, 7, true, 0, vec![])).is_some());
        // The caller missed the DiscAck and resends its Disc. We repeat it, but the
        // answer to it never reaches us: the entry must still go after the close window.
        assert!(repeat(&mut d, 115.0, &rx(Kind::Disc, 7, false, 0, vec![])).is_some());
        // Meanwhile only the close is repeated, from either end.
        assert!(repeat(&mut d, 120.0, &rx(Kind::Data, 7, false, 0, vec![])).is_none(), "Data after the close");
        assert!(repeat(&mut d, 121.0, &rx(Kind::Data, 7, true, 0, vec![])).is_none(), "Data after the close");
        assert_eq!(d.sessions(121.0), 1);
        let gone = 115.0 + CLOSE_EXPIRY_S + 1.0;
        assert!(repeat(&mut d, gone, &rx(Kind::Disc, 7, false, 0, vec![])).is_none(), "kept past the close window");
        assert_eq!(d.sessions(gone), 0);
    }

    #[test]
    fn new_session_on_a_closed_entry_starts_afresh() {
        let mut d = stale_entry();
        assert!(repeat(&mut d, 5.0, &connect_ack(7, &["LA3D1"], 0)).is_some());
        assert!(repeat(&mut d, 100.0, &rx(Kind::DiscAck, 7, true, 0, vec![])).is_some());
        // The same id on the same path again, within the close window.
        assert!(repeat(&mut d, 110.0, &connect_req(7, &["LA3D1"], 0)).is_some());
        assert!(repeat(&mut d, 112.0, &rx(Kind::Data, 7, false, 0, vec![])).is_none(), "not answered yet");
        assert!(repeat(&mut d, 115.0, &connect_ack(7, &["LA3D1"], 0)).is_some());
        let t = 115.0 + CLOSE_EXPIRY_S + 60.0;
        assert!(repeat(&mut d, t, &rx(Kind::Data, 7, false, 0, vec![])).is_some(), "the new session is live");
    }

    #[test]
    fn lifetimes_outlast_the_end_stations_timers() {
        for params in [OfdmParams::narrow(), OfdmParams::wide()] {
            let modem = Arc::new(Modem::new(params, 0.1));
            let cfg = LinkConfig { hop_margin_s: MAX_HOP_MARGIN_S, ..LinkConfig::default() };
            let link = Link::new(modem.clone(), cfg.clone(), 1);
            let (connect_s, disc_s) = (modem.burst_seconds(0, 1) as f64, modem.burst_seconds(0, 0) as f64);
            for n in 1..=MAX_DIGIS as u8 {
                let what = format!("{}, {n} digipeaters", modem.ofdm.p.name);
                // Connecting: two of the caller's requests in a row, start to start. We
                // hear each after the same delay, so that is also what we see.
                let attempt =
                    connect_s + cfg.response_timeout_s + link.relay_s(n, connect_s, connect_s) + CONNECT_JITTER_S;
                assert!(UNCONFIRMED_EXPIRY_S > 2.0 * attempt, "{what}: {attempt:.1} s per attempt");
                // Quiet: the called station probes last, 1.5 keepalives after it last
                // heard the caller, whose burst passed us up to a path's length earlier.
                let path = n as f64 * (link.max_burst_s + cfg.turnaround_s + cfg.hop_margin_s);
                let quiet = 1.5 * link.keepalive_s(n) + RETRY_BACKOFF_S + path;
                assert!(IDLE_EXPIRY_S > quiet, "{what}: quiet for {quiet:.1} s");
                // Closing: the DiscAck is lost after us, and the Disc comes again. At the
                // earliest we repeated the DiscAck right after the first Disc.
                let resent = disc_s + link.response_wait(n, disc_s) + RETRY_BACKOFF_S * Link::retry_spread(n);
                assert!(CLOSE_EXPIRY_S > resent, "{what}: Disc again after {resent:.1} s");
                println!("{what}: attempt {attempt:.1} s, quiet {quiet:.1} s, Disc again {resent:.1} s");
            }
        }
    }

    #[test]
    fn answer_heard_as_data_confirms() {
        // We missed the ConnectAck (its codeword failed), but the called station's Data
        // comes up through us: the session was answered.
        let mut d = stale_entry();
        assert!(repeat(&mut d, 5.0, &rx(Kind::ConnectAck, 7, true, 0, vec![None])).is_none());
        assert!(repeat(&mut d, 8.0, &rx(Kind::Data, 7, true, 0, vec![])).is_some());
        let t = 8.0 + IDLE_EXPIRY_S - 1.0;
        assert!(repeat(&mut d, t, &rx(Kind::Data, 7, false, 0, vec![])).is_some());
    }

    #[test]
    fn duplicate_is_repeated_once() {
        let mut d = Digipeater::new(&["LA3D1".to_string()], 0.2);
        repeat(&mut d, 0.0, &connect_req(7, &["LA3D1"], 0)).unwrap();
        repeat(&mut d, 0.5, &connect_ack(7, &["LA3D1"], 0)).unwrap();
        let b = rx(Kind::Data, 7, false, 0, vec![]);
        d.on_burst(1.0, &b);
        d.on_burst(1.05, &b);
        assert!(d.poll(2.0).is_some());
        assert!(d.poll(2.0).is_none());
    }
}
