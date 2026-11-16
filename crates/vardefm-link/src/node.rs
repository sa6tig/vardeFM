//! A station's link and its digipeater, sharing one radio.
//!
//! The radio's side of a [`Link`] (bursts heard, bursts to send) can only be reached
//! through a [`Node`], so a station cannot drive its link past the digipeater.

use crate::{Digipeater, Link};
use vardefm_modem::{Burst, RxBurst};

pub struct Node {
    /// The host's side: commands, data and events.
    pub link: Link,
    digi: Option<Digipeater>,
    /// The burst on the air is a repeat, not the link's own.
    repeating: bool,
}

impl Node {
    pub fn new(link: Link) -> Self {
        Self { link, digi: None, repeating: false }
    }

    /// Repeat the bursts of sessions routed through any of `calls` (none: off).
    /// Works alongside the link's own sessions.
    pub fn set_digi(&mut self, calls: &[String]) {
        let turnaround_s = self.link.config().turnaround_s;
        self.digi = (!calls.is_empty()).then(|| Digipeater::new(calls, turnaround_s));
    }

    /// Bursts repeated so far.
    pub fn repeats(&self) -> u64 {
        self.digi.as_ref().map_or(0, |d| d.repeated())
    }

    /// Handle a decoded burst.
    pub fn on_burst(&mut self, now: f64, rx: &RxBurst) {
        self.link.on_burst(now, rx);
        if let Some(d) = &mut self.digi {
            d.on_burst(now, rx);
        }
    }

    /// Called regularly while not transmitting. Returns a burst to transmit now, and
    /// whether it is a repeat.
    pub fn poll(&mut self, now: f64, busy: bool) -> Option<(Burst, bool)> {
        if let Some(d) = &mut self.digi {
            // Repeats answer a burst just heard, so they go first.
            if let Some(b) = d.poll(now) {
                self.repeating = true;
                return Some((b, true));
            }
            // A repeat is waiting out its turnaround. A burst of our own started now
            // would hold it back until that burst ends, past what the end stations
            // wait for.
            if d.pending() {
                return None;
            }
        }
        Some((self.link.poll(now, busy)?, false))
    }

    /// Called when the burst returned by `poll` has finished transmitting.
    pub fn tx_done(&mut self, now: f64) {
        // A repeat is not the link's burst: its timers must not restart.
        if !std::mem::take(&mut self.repeating) {
            self.link.tx_done(now);
        }
    }
}

#[cfg(test)]
impl Node {
    /// Replace the digipeater, e.g. with a slow one.
    pub(crate) fn set_digipeater(&mut self, d: Digipeater) {
        self.digi = Some(d);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{calls_payload, LinkConfig};
    use std::sync::Arc;
    use vardefm_modem::{Header, Kind, Modem, OfdmParams};

    const DIGI: &str = "LA3DDD";

    /// A digipeater for `DIGI` whose own link is calling LA5X directly.
    fn calling_digi(seed: u32) -> Node {
        let modem = Arc::new(Modem::new(OfdmParams::narrow(), 0.1));
        let mut node = Node::new(Link::new(modem, LinkConfig::default(), seed));
        node.set_digi(&[DIGI.to_string()]);
        node.link.connect(0.0, "LA3DDD", "LA5X", &[]).unwrap();
        node
    }

    /// Another station's routed request through `DIGI`, as its caller sends it.
    fn routed_request() -> RxBurst {
        let mut header = Header { kind: Kind::ConnectReq, session: 7, ncw: 1, ..Header::default() };
        header.set_hop(0);
        let payload = vec![Some(calls_payload("LA1AAA", "LA2BBB", &[DIGI.to_string()]))];
        RxBurst { header, payload, snr_db: 30.0, end_sample: 0, level: Default::default() }
    }

    #[test]
    fn repeat_goes_before_our_own_burst() {
        let mut node = calling_digi(1);
        node.on_burst(0.0, &routed_request());
        // The repeat is due 0.2 s after the burst. Our own request is ready at once,
        // but sending it now would hold the repeat back for the whole burst.
        assert!(node.poll(0.1, false).is_none(), "own burst sent while a repeat was waiting");
        let (burst, repeat) = node.poll(0.2, false).expect("the repeat is due");
        assert!(repeat);
        assert_eq!((burst.header.session, burst.header.hop()), (7, 1));
        node.tx_done(2.0);
        let (burst, repeat) = node.poll(2.0, false).expect("then our own request");
        assert!(!repeat);
        assert_eq!(burst.header.kind, Kind::ConnectReq);
        assert_eq!(node.repeats(), 1);
    }

    /// Steps `node` from `from` until its link sends a burst; returns the time.
    fn next_own_burst(node: &mut Node, from: f64) -> f64 {
        let mut t = from;
        loop {
            assert!(t < from + 60.0, "no burst");
            if let Some((_, repeat)) = node.poll(t, false) {
                assert!(!repeat);
                return t;
            }
            t += 0.01;
        }
    }

    #[test]
    fn repeat_does_not_restart_the_links_timers() {
        // Two stations with the same seed send their first request together. One then
        // repeats a burst while it waits for the answer.
        let (mut quiet, mut busy) = (calling_digi(2), calling_digi(2));
        for node in [&mut quiet, &mut busy] {
            assert!(node.poll(0.0, false).is_some_and(|(_, repeat)| !repeat));
            node.tx_done(1.9);
        }
        busy.on_burst(2.0, &routed_request());
        assert!(busy.poll(2.2, false).is_some_and(|(_, repeat)| repeat));
        busy.tx_done(2.9);
        // Its own retry still goes out when it would have without the repeat.
        let (a, b) = (next_own_burst(&mut quiet, 2.9), next_own_burst(&mut busy, 2.9));
        assert!(a > 2.9 + 0.1, "retry at {a}: not waiting for an answer");
        assert_eq!(a, b);
    }
}
