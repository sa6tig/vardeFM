//! Link tests over a fake channel: bursts take their real airtime, collide when
//! two stations a receiver hears transmit at once (or the receiver itself does),
//! and can lose whole bursts or single codewords. Stations may hear only some of
//! the others, and may run a digipeater.

use super::*;
use vardefm_modem::OfdmParams;

const DT: f64 = 0.01;

struct Node {
    name: String,
    link: Link,
    digi: Option<Digipeater>,
    /// (start, end, burst, from the digipeater) while transmitting.
    tx: Option<(f64, f64, Burst, bool)>,
    host_rx: Vec<u8>,
    events: Vec<LinkEvent>,
    /// Stop receiving and transmitting (station switched off).
    dead: bool,
}

struct Net {
    nodes: Vec<Node>,
    /// `hears[to][from]`.
    hears: Vec<Vec<bool>>,
    modem: Arc<Modem>,
    now: f64,
    /// Bursts on their way: (deliver_at, to, burst, start, end, from).
    air: Vec<(f64, usize, Burst, f64, f64, usize)>,
    /// Every transmission still relevant for collisions and busy: (from, start, end).
    history: Vec<(usize, f64, f64)>,
    rng: Prng,
    burst_loss: f64,
    cw_loss: f64,
    airtime: f64,
}

const A: usize = 0;

impl Net {
    /// Two stations that hear each other: A (LA1AAA) and B (LA2BBB-1, listening).
    fn new(seed: u32, burst_loss: f64, cw_loss: f64) -> Self {
        Self::build(seed, burst_loss, cw_loss, &[], LinkConfig::default())
    }

    /// A, the digipeaters `digis` in order, then B. Each station hears only its
    /// neighbours along the chain.
    fn chain(seed: u32, cw_loss: f64, digis: &[&str], cfg: LinkConfig) -> Self {
        Self::build(seed, 0.0, cw_loss, digis, cfg)
    }

    fn build(seed: u32, burst_loss: f64, cw_loss: f64, digis: &[&str], cfg: LinkConfig) -> Self {
        let modem = Arc::new(Modem::new(OfdmParams::narrow(), 0.1));
        let count = digis.len() + 2;
        let mut nodes: Vec<Node> = (0..count)
            .map(|i| Node {
                name: String::new(),
                link: Link::new(modem.clone(), cfg.clone(), seed + i as u32),
                digi: None,
                tx: None,
                host_rx: Vec::new(),
                events: Vec::new(),
                dead: false,
            })
            .collect();
        nodes[A].name = "A".into();
        nodes[A].link.set_mycalls(vec!["LA1AAA".into()]);
        for (i, d) in digis.iter().enumerate() {
            nodes[i + 1].name = format!("D{}", i + 1);
            nodes[i + 1].digi = Some(Digipeater::new(&[d.to_string()], cfg.turnaround_s));
        }
        let b = count - 1;
        nodes[b].name = "B".into();
        nodes[b].link.set_mycalls(vec!["LA2BBB-1".into()]);
        nodes[b].link.set_listen(true);
        let hears = (0..count).map(|to| (0..count).map(|from| to.abs_diff(from) == 1).collect()).collect();
        Self {
            nodes,
            hears,
            modem,
            now: 0.0,
            air: Vec::new(),
            history: Vec::new(),
            rng: Prng::new(seed + 99),
            burst_loss,
            cw_loss,
            airtime: 0.0,
        }
    }

    fn b(&self) -> usize {
        self.nodes.len() - 1
    }

    fn chance(&mut self, p: f64) -> bool {
        (self.rng.next_u32() as f64 / u32::MAX as f64) < p
    }

    /// True if `to` cannot receive a burst from `from` over `[a, b]`: it transmitted
    /// itself, or heard another station at the same time.
    fn collided(&self, to: usize, from: usize, a: f64, b: f64) -> bool {
        self.history.iter().any(|&(g, s, e)| g != from && (g == to || self.hears[to][g]) && s < b && e > a)
    }

    fn busy(&self, i: usize) -> bool {
        // Hear a station from key-up until the squelch tail ends.
        self.history.iter().any(|&(g, s, e)| self.hears[i][g] && self.now >= s + 0.1 && self.now < e + 0.25)
    }

    fn trace(&self) -> bool {
        std::env::var("LINK_TRACE").is_ok()
    }

    fn step(&mut self) {
        self.now += DT;
        let now = self.now;
        // Finish transmissions.
        for i in 0..self.nodes.len() {
            if self.nodes[i].tx.as_ref().is_some_and(|t| now >= t.1) {
                let (start, end, _, digi) = self.nodes[i].tx.take().unwrap();
                if !digi {
                    self.nodes[i].link.tx_done(now);
                }
                self.airtime += end - start;
            }
        }
        // Deliver.
        let mut arrived = Vec::new();
        self.air.retain(|a| {
            if a.0 <= now {
                arrived.push(a.clone());
                false
            } else {
                true
            }
        });
        for (_, to, burst, start, end, from) in arrived {
            if self.nodes[to].dead || self.collided(to, from, start, end) || self.chance(self.burst_loss) {
                if self.trace() {
                    eprintln!("{now:7.2} lost burst {}->{}", self.nodes[from].name, self.nodes[to].name);
                }
                continue;
            }
            let payload = burst
                .payload
                .iter()
                .map(|cw| if self.chance(self.cw_loss) { None } else { Some(cw.clone()) })
                .collect();
            let rx = RxBurst { header: burst.header, payload, snr_db: 30.0, end_sample: 0, level: Default::default() };
            let node = &mut self.nodes[to];
            node.link.on_burst(now, &rx);
            if let Some(d) = &mut node.digi {
                d.on_burst(now, &rx);
            }
        }
        self.history.retain(|&(_, _, e)| e + 30.0 > now);
        // Poll for new transmissions: digipeater repeats first.
        for i in 0..self.nodes.len() {
            if self.nodes[i].dead || self.nodes[i].tx.is_some() {
                continue;
            }
            let busy = self.busy(i);
            let node = &mut self.nodes[i];
            let (b, digi) = match node.digi.as_mut().and_then(|d| d.poll(now)) {
                Some(b) => (b, true),
                None => match node.link.poll(now, busy) {
                    Some(b) => (b, false),
                    None => continue,
                },
            };
            let dur = self.modem.burst_seconds(b.header.level, b.header.ncw as usize) as f64;
            if self.trace() {
                eprintln!(
                    "{now:7.2} {} TX {:?} ncw={} lvl={} id={} ack={}/{} bm={:b} up={} hop={} {:.2}s",
                    self.nodes[i].name,
                    b.header.kind,
                    b.header.ncw,
                    b.header.level,
                    b.header.burst_id,
                    b.header.ack_burst,
                    b.header.ack_offset,
                    b.header.bitmap,
                    b.header.up(),
                    b.header.hop(),
                    dur
                );
            }
            let end = now + dur;
            for to in 0..self.nodes.len() {
                if self.hears[to][i] {
                    self.air.push((end + 0.02, to, b.clone(), now, end, i));
                }
            }
            self.history.push((i, now, end));
            self.nodes[i].tx = Some((now, end, b, digi));
        }
        for n in &mut self.nodes {
            for e in n.link.take_events() {
                if let LinkEvent::Data(d) = &e {
                    n.host_rx.extend_from_slice(d);
                }
                n.events.push(e);
            }
        }
    }

    /// Run until `done` or `limit` seconds pass. Returns the time taken.
    fn run_until(&mut self, limit: f64, mut done: impl FnMut(&Net) -> bool) -> f64 {
        let start = self.now;
        while !done(self) {
            assert!(self.now - start < limit, "timed out after {limit} s at t={:.1}", self.now);
            self.step();
        }
        self.now - start
    }

    fn has(&self, i: usize, e: &LinkEvent) -> bool {
        self.nodes[i].events.contains(e)
    }

    fn both_connected(&self) -> bool {
        self.nodes[A].link.is_connected() && self.nodes[self.b()].link.is_connected()
    }

    fn both_idle(&self) -> bool {
        self.nodes[A].link.is_idle() && self.nodes[self.b()].link.is_idle()
    }
}

fn data(n: usize, seed: u32) -> Vec<u8> {
    // Every byte value, including CR and 0x00, in pseudo-random order.
    let mut rng = Prng::new(seed);
    (0..n).map(|i| if i < 256 { i as u8 } else { rng.next_u32() as u8 }).collect()
}

fn session(burst_loss: f64, cw_loss: f64, seed: u32) {
    let mut net = Net::new(seed, burst_loss, cw_loss);
    net.nodes[0].link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    net.run_until(60.0, |n| n.nodes[0].link.is_connected() && n.nodes[1].link.is_connected());
    assert!(net.has(0, &LinkEvent::Connected { source: "LA1AAA".into(), destination: "LA2BBB-1".into() }));
    assert!(net.has(1, &LinkEvent::Connected { source: "LA1AAA".into(), destination: "LA2BBB-1".into() }));
    assert!(net.has(1, &LinkEvent::Pending));

    let (a, b) = (data(10_000, seed), data(10_000, seed + 7));
    net.nodes[1].link.send_data(&b[..300]); // banner first
    let t = net.run_until(600.0, |n| n.nodes[0].host_rx.len() >= 300);
    assert!(t < 60.0);
    net.nodes[1].link.send_data(&b[300..]);
    net.nodes[0].link.send_data(&a);
    let t = net.run_until(900.0, |n| n.nodes[0].host_rx.len() >= b.len() && n.nodes[1].host_rx.len() >= a.len());
    assert_eq!(net.nodes[0].host_rx, b);
    assert_eq!(net.nodes[1].host_rx, a);
    net.run_until(60.0, |n| n.has(0, &LinkEvent::Buffer(0)) && n.has(1, &LinkEvent::Buffer(0)));

    net.nodes[0].link.disconnect(net.now);
    net.run_until(60.0, |n| n.nodes[0].link.is_idle() && n.nodes[1].link.is_idle());
    for node in &net.nodes[..2] {
        assert_eq!(node.events.iter().filter(|e| **e == LinkEvent::Disconnected).count(), 1);
    }
    println!(
        "loss {burst_loss}/{cw_loss}: 20 kB in {t:.0} s, airtime {:.0} s, retries {}/{}, resent {}/{} bytes",
        net.airtime,
        net.nodes[0].link.stats.retries,
        net.nodes[1].link.stats.retries,
        net.nodes[0].link.stats.bytes_resent,
        net.nodes[1].link.stats.bytes_resent
    );
}

#[test]
fn clean_session() {
    session(0.0, 0.0, 1);
}

#[test]
fn lossy_session() {
    session(0.15, 0.15, 2);
    session(0.3, 0.1, 3);
}

#[test]
fn connect_to_absent_station_gives_up() {
    let mut net = Net::new(4, 0.0, 0.0);
    net.nodes[1].dead = true;
    net.nodes[0].link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    let t = net.run_until(60.0, |n| n.nodes[0].link.is_idle());
    assert!(t < 25.0, "took {t} s");
    assert_eq!(net.nodes[0].events, vec![LinkEvent::Disconnected]);
}

#[test]
fn not_listening_or_wrong_call_is_ignored() {
    let mut net = Net::new(5, 0.0, 0.0);
    net.nodes[1].link.set_listen(false);
    net.nodes[0].link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    net.run_until(60.0, |n| n.nodes[0].link.is_idle());
    assert!(net.nodes[1].events.is_empty());

    let mut net = Net::new(6, 0.0, 0.0);
    net.nodes[0].link.connect(0.0, "LA1AAA", "LA9ZZZ", &[]).unwrap();
    net.run_until(60.0, |n| n.nodes[0].link.is_idle());
    assert!(net.nodes[1].events.is_empty());
}

#[test]
fn abort_tells_the_peer() {
    let mut net = Net::new(7, 0.0, 0.0);
    net.nodes[0].link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    net.run_until(60.0, |n| n.nodes[0].link.is_connected() && n.nodes[1].link.is_connected());
    net.nodes[0].link.abort(net.now);
    assert!(net.nodes[0].link.is_idle());
    let t = net.run_until(10.0, |n| n.nodes[1].link.is_idle());
    assert!(t < 5.0);
}

#[test]
fn dead_peer_is_detected() {
    let mut net = Net::new(8, 0.0, 0.0);
    net.nodes[0].link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    net.run_until(60.0, |n| n.nodes[0].link.is_connected() && n.nodes[1].link.is_connected());
    net.nodes[1].dead = true;
    net.nodes[0].link.send_data(b"hello?");
    let t = net.run_until(120.0, |n| n.nodes[0].link.is_idle());
    assert!(t < 60.0, "took {t} s");
    assert!(net.has(0, &LinkEvent::Disconnected));
}

#[test]
fn idle_link_is_kept_alive_then_dropped_when_peer_dies() {
    let mut net = Net::new(9, 0.0, 0.0);
    net.nodes[0].link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    net.run_until(60.0, |n| n.nodes[0].link.is_connected() && n.nodes[1].link.is_connected());
    // Quiet for two minutes: keepalives keep both ends connected.
    net.run_until(130.0, |n| n.now > 120.0);
    assert!(net.nodes[0].link.is_connected() && net.nodes[1].link.is_connected());
    // Then the caller disappears without a word; the called side notices.
    net.nodes[0].dead = true;
    net.run_until(200.0, |n| n.nodes[1].link.is_idle());
}

fn via(digis: &[&str]) -> Vec<String> {
    digis.iter().map(|s| s.to_string()).collect()
}

/// Connect A to B through `net`'s digipeaters, move 10 kB each way, disconnect.
fn routed_session(mut net: Net, digis: &[&str], seed: u32) -> f64 {
    let b = net.b();
    net.nodes[A].link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(digis)).unwrap();
    net.run_until(120.0, |n| n.both_connected());
    let connected = LinkEvent::Connected { source: "LA1AAA".into(), destination: "LA2BBB-1".into() };
    assert!(net.has(A, &connected) && net.has(b, &connected));

    let (da, db) = (data(10_000, seed), data(10_000, seed + 7));
    net.nodes[b].link.send_data(&db);
    net.nodes[A].link.send_data(&da);
    let t = net.run_until(3000.0, |n| n.nodes[A].host_rx.len() >= db.len() && n.nodes[b].host_rx.len() >= da.len());
    assert_eq!(net.nodes[A].host_rx, db);
    assert_eq!(net.nodes[b].host_rx, da);

    net.run_until(300.0, |n| n.has(A, &LinkEvent::Buffer(0)) && n.has(b, &LinkEvent::Buffer(0)));
    net.nodes[A].link.disconnect(net.now);
    net.run_until(300.0, |n| n.both_idle());
    for i in [A, b] {
        assert_eq!(net.nodes[i].events.iter().filter(|e| **e == LinkEvent::Disconnected).count(), 1);
    }
    let repeats: Vec<u64> = net.nodes.iter().filter_map(|n| n.digi.as_ref().map(|d| d.repeated)).collect();
    println!(
        "via {digis:?}: 20 kB in {t:.0} s, airtime {:.0} s, retries {}/{}, repeats {repeats:?}",
        net.airtime, net.nodes[A].link.stats.retries, net.nodes[b].link.stats.retries
    );
    assert!(repeats.iter().all(|&r| r > 0), "every digipeater relayed");
    t
}

#[test]
fn session_via_one_and_two_digipeaters() {
    let mut times = Vec::new();
    for (i, digis) in [&["LA3DDD"][..], &["LA3DDD", "LA4EEE-2"]].iter().enumerate() {
        let seed = 20 + i as u32;
        times.push(routed_session(Net::chain(seed, 0.0, digis, LinkConfig::default()), digis, seed));
    }
    // Each hop costs airtime, so longer chains are slower.
    assert!(times[0] < times[1], "{times:?}");
}

#[test]
fn lossy_routed_session_resends_voided_codewords() {
    let digis = ["LA3DDD", "LA4EEE-2"];
    routed_session(Net::chain(30, 0.1, &digis, LinkConfig::default()), &digis, 30);
}

#[test]
fn endpoints_that_hear_each_other_ignore_the_direct_copy() {
    // Everyone hears everyone. A routed session must still deliver each byte once,
    // although A and B also decode each other's bursts directly.
    let digis = ["LA3DDD"];
    let mut net = Net::chain(31, 0.0, &digis, LinkConfig::default());
    for (to, row) in net.hears.iter_mut().enumerate() {
        for (from, h) in row.iter_mut().enumerate() {
            *h = to != from;
        }
    }
    routed_session(net, &digis, 31);
}

/// A Data burst asking for an answer, with UP and the raw 2-bit route field `route`
/// (0 = direct, else 1 + digipeater repeats).
fn probe(session: u8, up: bool, route: u8) -> RxBurst {
    let flags = FLAG_POLL | if up { FLAG_UP } else { 0 } | route << 2;
    let header = Header { kind: Kind::Data, session, flags, ..Header::default() };
    RxBurst { header, payload: Vec::new(), snr_db: 30.0, end_sample: 0, level: Default::default() }
}

/// Every (UP, route field) a burst can carry, except `own`.
fn all_but(own: (bool, u8)) -> Vec<(bool, u8)> {
    [false, true].into_iter().flat_map(|up| (0..4).map(move |r| (up, r))).filter(|&c| c != own).collect()
}

/// Once `net` has gone quiet, feeds each end every other (UP, route field) and then
/// its peer's own: only the peer's own is answered.
fn only_own_answered(net: &mut Net, own_a: (bool, u8), own_b: (bool, u8)) {
    let settled = net.now + 20.0;
    net.run_until(30.0, |n| n.now >= settled);
    let now = net.now;
    for (i, own) in [(A, own_a), (net.b(), own_b)] {
        let link = &mut net.nodes[i].link;
        let State::Connected(s) = &link.state else { panic!("not connected") };
        let id = s.id;
        assert!(link.poll(now + 1.0, false).is_none(), "node {i} not quiet");
        for (up, route) in all_but(own) {
            link.on_burst(now, &probe(id, up, route));
            assert!(link.poll(now + 1.0, false).is_none(), "node {i} answered up {up} route {route}");
        }
        link.on_burst(now, &probe(id, own.0, own.1));
        assert!(link.poll(now + 1.0, false).is_some(), "node {i} answers the peer's own burst");
    }
}

#[test]
fn direct_session_ignores_bursts_a_digipeater_repeated() {
    // A direct session's own bursts are unrouted without UP, as every version sends
    // them. Anything else with its id is not the peer's: most of all a routed caller's
    // unrepeated burst (route 1, UP 0) heard directly, and copies a digipeater repeated.
    let mut net = Net::new(33, 0.0, 0.0);
    net.nodes[A].link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    net.run_until(60.0, |n| n.both_connected());
    only_own_answered(&mut net, (false, 0), (false, 0));

    // A caller still connecting takes any Data with its session id as the answer.
    let mut net = Net::new(34, 0.0, 0.0);
    let link = &mut net.nodes[A].link;
    link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
    let State::Connecting { session, .. } = &link.state else { panic!("not connecting") };
    let id = *session;
    for (up, route) in all_but((false, 0)) {
        link.on_burst(0.5, &probe(id, up, route));
        assert!(!link.is_connected(), "connected by up {up} route {route}");
    }
    link.on_burst(0.5, &probe(id, false, 0));
    assert!(link.is_connected());
}

#[test]
fn routed_session_ignores_direct_bursts() {
    // Through one digipeater each end takes only the other's burst after one repeat
    // (route field 2), with UP from the called station. A direct burst with the same id
    // (route 0) is another session's.
    let digis = ["LA3DDD"];
    let mut net = Net::chain(38, 0.0, &digis, LinkConfig::default());
    net.nodes[A].link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(&digis)).unwrap();
    net.run_until(120.0, |n| n.both_connected());
    only_own_answered(&mut net, (true, 2), (false, 2));

    // A routed caller still connecting is not answered by direct Data either.
    let mut net = Net::new(39, 0.0, 0.0);
    let link = &mut net.nodes[A].link;
    link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(&digis)).unwrap();
    let State::Connecting { session, .. } = &link.state else { panic!("not connecting") };
    let id = *session;
    for (up, route) in all_but((true, 2)) {
        link.on_burst(0.5, &probe(id, up, route));
        assert!(!link.is_connected(), "connected by up {up} route {route}");
    }
    link.on_burst(0.5, &probe(id, true, 2));
    assert!(link.is_connected());
}

#[test]
fn bursts_carry_the_route_field() {
    // Direct: route field 0 and no UP from either end, exactly as released versions.
    // Routed: route field 1 (not yet repeated) from both ends, UP from the called station.
    for (digis, a_flags, b_flags) in [(&[][..], 0, 0), (&["LA3DDD"][..], FLAG_ROUTED, FLAG_ROUTED | FLAG_UP)] {
        let mut net = Net::chain(40, 0.0, digis, LinkConfig::default());
        let b = net.b();
        let mut sent: Vec<(usize, Kind, u8)> = Vec::new();
        // Steps until `done`, noting each end's bursts as they start.
        let run = |net: &mut Net, sent: &mut Vec<(usize, Kind, u8)>, done: &dyn Fn(&Net) -> bool| {
            while !done(net) {
                assert!(net.now < 600.0, "via {digis:?}: timed out");
                net.step();
                for i in [A, b] {
                    if let Some((start, _, burst, false)) = &net.nodes[i].tx {
                        if (*start - net.now).abs() < DT / 2.0 {
                            sent.push((i, burst.header.kind, burst.header.flags & !FLAG_POLL));
                        }
                    }
                }
            }
        };
        net.nodes[A].link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(digis)).unwrap();
        run(&mut net, &mut sent, &|n| n.both_connected());
        net.nodes[A].link.send_data(b"hello");
        net.nodes[b].link.send_data(b"there");
        run(&mut net, &mut sent, &|n| n.nodes[A].host_rx.len() >= 5 && n.nodes[n.b()].host_rx.len() >= 5);
        net.nodes[A].link.disconnect(net.now);
        run(&mut net, &mut sent, &|n| n.both_idle());
        for kind in [Kind::ConnectReq, Kind::ConnectAck, Kind::Data, Kind::Disc, Kind::DiscAck] {
            assert!(sent.iter().any(|&(_, k, _)| k == kind), "via {digis:?}: no {kind:?} sent");
        }
        for (i, kind, flags) in sent {
            let want = if i == A { a_flags } else { b_flags };
            assert_eq!(flags, want, "via {digis:?}: node {i} {kind:?}");
        }
    }
}

#[test]
fn digipeater_switched_off_drops_the_link_on_both_ends() {
    let mut net = Net::chain(32, 0.0, &["LA3DDD"], LinkConfig::default());
    let b = net.b();
    net.nodes[A].link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(&["LA3DDD"])).unwrap();
    net.run_until(120.0, |n| n.both_connected());
    net.nodes[1].dead = true;
    net.nodes[A].link.send_data(b"hello?");
    net.nodes[b].link.send_data(b"anyone?");
    net.run_until(600.0, |n| n.both_idle());
    assert!(net.has(A, &LinkEvent::Disconnected) && net.has(b, &LinkEvent::Disconnected));
}

#[test]
fn connect_via_absent_digipeater_gives_up() {
    // The station between A and B is not a digipeater for LA9ZZZ.
    let mut net = Net::chain(33, 0.0, &["LA3DDD"], LinkConfig::default());
    net.nodes[A].link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(&["LA9ZZZ"])).unwrap();
    let t = net.run_until(120.0, |n| n.nodes[A].link.is_idle());
    assert!(t < 60.0, "took {t} s");
    assert_eq!(net.nodes[A].events, vec![LinkEvent::Disconnected]);
    assert!(net.nodes[net.b()].events.is_empty());
}

#[test]
fn more_than_two_digipeaters_is_refused() {
    let mut net = Net::new(34, 0.0, 0.0);
    for path in [&["LA3A", "LA3B", "LA3C"][..], &["LA3A", "LA3B", "LA3C", "LA3D"]] {
        assert!(net.nodes[A].link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(path)).is_err(), "via {path:?}");
        assert!(net.nodes[A].link.is_idle());
    }
    // Nor is a request through three answered.
    let mut b = Link::new(net.modem.clone(), LinkConfig::default(), 1);
    b.set_mycalls(vec!["LA2BBB-1".into()]);
    b.set_listen(true);
    let mut header = Header { kind: Kind::ConnectReq, session: 9, ncw: 1, ..Header::default() };
    header.set_hop(2);
    let payload = vec![Some(calls_payload("LA1AAA", "LA2BBB-1", &via(&["LA3A", "LA3B", "LA3C"])))];
    b.on_burst(1.0, &RxBurst { header, payload, snr_db: 30.0, end_sample: 0, level: Default::default() });
    assert!(b.is_idle() && b.take_events().is_empty());
}

#[test]
fn retry_wait_grows_with_hops_and_margin() {
    let modem = Arc::new(Modem::new(OfdmParams::narrow(), 0.1));
    let link = Link::new(modem.clone(), LinkConfig::default(), 1);
    let wide_margin = Link::new(modem, LinkConfig { hop_margin_s: 3.0, ..LinkConfig::default() }, 1);
    let cfg = LinkConfig::default();
    assert_eq!(link.response_wait(0, 4.0), cfg.response_timeout_s);
    let w: Vec<f64> = (0..=MAX_DIGIS as u8).map(|n| link.response_wait(n, 4.0)).collect();
    assert!(w.windows(2).all(|p| p[1] > p[0] + 8.0), "{w:?}");
    assert!((wide_margin.response_wait(2, 4.0) - link.response_wait(2, 4.0) - 2.0 * 2.5).abs() < 1e-9);
    // Keepalive never fires while an answer may still be coming.
    for n in 1..=MAX_DIGIS as u8 {
        assert!(link.keepalive_s(n) >= 2.0 * link.response_wait(n, link.max_burst_s));
    }
    assert_eq!(link.keepalive_s(0), cfg.keepalive_s);
    for params in [OfdmParams::narrow(), OfdmParams::wide()] {
        let l = Link::new(Arc::new(Modem::new(params, 0.1)), LinkConfig::default(), 1);
        let w: Vec<String> =
            (1..=MAX_DIGIS as u8).map(|n| format!("{:.1}", l.response_wait(n, l.max_burst_s))).collect();
        println!("{}: longest burst {:.2} s, wait after a full burst {w:?} s", l.modem.ofdm.p.name, l.max_burst_s);
    }
}

#[test]
fn slow_digipeater_needs_a_larger_hop_margin() {
    // The digipeater answers 1 s after each burst instead of 0.2 s. B cannot hear A,
    // so only the hop margin covers the delay: with it raised, the session runs.
    let digis = ["LA3DDD"];
    let cfg = LinkConfig { hop_margin_s: 2.0, ..LinkConfig::default() };
    let mut net = Net::chain(35, 0.0, &digis, cfg);
    net.nodes[1].digi = Some(Digipeater::new(&["LA3DDD".to_string()], 1.0));
    routed_session(net, &digis, 35);
}

#[test]
fn called_station_banner_does_not_collide_with_connect_ack_repeats() {
    // A BBS queues its banner as soon as it reports CONNECTED, which happens when the
    // ConnectReq arrives, before its ConnectAck has gone out.
    for digis in [&["LA3DDD"][..], &["LA3DDD", "LA4EEE-2"]] {
        let mut net = Net::chain(36, 0.0, digis, LinkConfig::default());
        let b = net.b();
        net.nodes[A].link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(digis)).unwrap();
        net.run_until(120.0, |n| n.nodes[n.b()].link.is_connected());
        let banner = data(500, 36);
        net.nodes[b].link.send_data(&banner);
        net.run_until(300.0, |n| n.nodes[A].host_rx.len() >= banner.len());
        assert_eq!(net.nodes[A].host_rx, banner);
        let (ra, rb) = (net.nodes[A].link.stats.retries, net.nodes[b].link.stats.retries);
        assert_eq!((ra, rb), (0, 0), "via {digis:?}: retries A {ra} B {rb}");
    }
}

#[test]
fn path_naming_one_digipeater_twice() {
    let digis = ["LA3DDD"];
    let mut net = Net::chain(37, 0.0, &digis, LinkConfig::default());
    net.nodes[A].link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(&["LA3DDD", "LA3DDD"])).unwrap();
    net.run_until(120.0, |n| n.both_connected());
    net.nodes[A].link.send_data(b"twice");
    net.run_until(120.0, |n| n.nodes[n.b()].host_rx.len() >= 5);
    assert_eq!(net.nodes[net.b()].host_rx, b"twice");
}

/// Both ends of a quiet routed link queue `size` bytes, B `offset` s after A, so their
/// bursts meet at a digipeater and are both lost. Returns the time to deliver both ways
/// and each end's retries, or what went wrong.
fn simultaneous_start(digis: &[&str], seed: u32, size: usize, offset: f64) -> Result<(f64, u64, u64), String> {
    let mut net = Net::chain(seed, 0.0, digis, LinkConfig::default());
    let b = net.b();
    net.nodes[A].link.connect(0.0, "LA1AAA", "LA2BBB-1", &via(digis)).unwrap();
    net.run_until(120.0, |n| n.both_connected());
    let quiet = net.now + 25.0;
    net.run_until(30.0, |n| n.now >= quiet);
    let (da, db) = (data(size, seed), data(size, seed + 7));
    net.nodes[A].link.send_data(&da);
    let at = net.now + offset;
    net.run_until(5.0, |n| n.now >= at);
    net.nodes[b].link.send_data(&db);
    let lost = |n: &Net| n.has(A, &LinkEvent::Disconnected) || n.has(n.b(), &LinkEvent::Disconnected);
    let t = net.run_until(1500.0, |n| {
        lost(n) || n.nodes[A].host_rx.len() >= db.len() && n.nodes[n.b()].host_rx.len() >= da.len()
    });
    let (ra, rb) = (net.nodes[A].link.stats.retries, net.nodes[b].link.stats.retries);
    let what = format!("via {digis:?} seed {seed} {size} B offset {offset:.1} s");
    println!("{what}: {t:.0} s, retries {ra}/{rb}");
    if lost(&net) {
        return Err(format!("{what}: link dropped after {t:.0} s, retries {ra}/{rb}"));
    }
    assert_eq!(net.nodes[A].host_rx, db, "{what}");
    assert_eq!(net.nodes[b].host_rx, da, "{what}");
    Ok((t, ra, rb))
}

#[test]
fn routed_ends_starting_together_do_not_collide_forever() {
    // The two ends do not hear each other, so neither can tell that the other is
    // sending. With equal retry timers their retries met at a digipeater every time.
    // The longest path again, on more seeds.
    let chains = [&["LA3DDD"][..], &["LA3DDD", "LA4EEE-2"], &["LA3DDD", "LA4EEE-2"]];
    let mut runs = Vec::new();
    for (i, digis) in chains.iter().enumerate() {
        for k in 0..4u32 {
            let size = if k == 3 { 300 } else { 40 };
            runs.push(simultaneous_start(digis, 60 + 10 * i as u32 + k, size, 0.2 * k as f64));
        }
    }
    runs.push(simultaneous_start(chains[0], 59, 6000, 0.3));
    let failed: Vec<&String> = runs.iter().filter_map(|r| r.as_ref().err()).collect();
    assert!(failed.is_empty(), "{} of {} runs failed: {failed:#?}", failed.len(), runs.len());
    // The first bursts did collide: every run needed a retry.
    assert!(runs.iter().flatten().all(|&(_, ra, rb)| ra + rb > 0), "a run had no collision");
}

#[test]
fn called_station_outwaits_the_callers_retry() {
    for params in [OfdmParams::narrow(), OfdmParams::wide()] {
        let modem = Arc::new(Modem::new(params, 0.1));
        for cfg in [LinkConfig::default(), LinkConfig { hop_margin_s: 3.0, ..LinkConfig::default() }] {
            let link = Link::new(modem.clone(), cfg.clone(), 1);
            let max = link.max_burst_s;
            let lengths = [modem.burst_seconds(0, 0) as f64, 1.0, max];
            // Direct sessions and the caller keep the plain response wait.
            assert_eq!(link.retry_wait(0, false, 1.0), link.response_wait(0, 1.0));
            let mut waits = Vec::new();
            for n in 1..=MAX_DIGIS as u8 {
                for own in lengths {
                    assert_eq!(link.retry_wait(n, true, own), link.response_wait(n, own));
                }
                // The caller's longest retry period, start to start, and the time its
                // retry takes to reach the called station.
                let caller = (0..=20).map(|i| max * i as f64 / 20.0).map(|t| t + link.response_wait(n, t));
                let caller = caller.fold(0.0, f64::max) + RETRY_BACKOFF_S * Link::retry_spread(n);
                let reach = n as f64 * (max + cfg.turnaround_s + cfg.hop_margin_s);
                for own in lengths {
                    let called = own + link.retry_wait(n, false, own);
                    let what = format!("{n} hops, {own:.2} s burst: {called:.1} < {caller:.1} + {reach:.1}");
                    assert!(called >= caller + reach - 1e-9, "{what}");
                }
                // Keepalive never fires while the called station's retry may be coming.
                assert!(link.keepalive_s(n) >= 2.0 * link.retry_wait(n, false, 0.0));
                let (short, full) = (link.retry_wait(n, false, lengths[0]), link.retry_wait(n, false, max));
                waits.push(format!("{short:.1}/{full:.1}"));
            }
            let name = modem.ofdm.p.name;
            let margin = cfg.hop_margin_s;
            println!("{name}, hop margin {margin}: called station's retry wait after a short/full burst {waits:?} s");
        }
    }
}

#[test]
fn routed_session_with_lost_bursts_completes() {
    let digis = ["LA3DDD", "LA4EEE-2"];
    for seed in 70..82 {
        println!("seed {seed}");
        routed_session(Net::build(seed, 0.1, 0.0, &digis, LinkConfig::default()), &digis, seed);
    }
}

#[test]
fn rate_choice_accounts_for_drive_per_level() {
    for params in [OfdmParams::narrow(), OfdmParams::wide()] {
        let modem = Modem::new(params, 0.1);
        let name = modem.ofdm.p.name;
        let top = LEVELS.len() as u8 - 1;
        for l in 0..=top {
            // Just above the threshold, measured on a burst at that level.
            let at = SNR_THRESHOLDS_DB[l as usize] + SNR_MARGIN_DB + 0.05;
            assert!(level_by_snr(&modem, at, l) >= l, "{name} level {l}");
            // The same channel measured on a louder level-0 burst (the model is invertible
            // below the ceilings).
            let at0 = modem.predict_snr_db(at, l, 0);
            assert!(level_by_snr(&modem, at0, 0) >= l, "{name} level {l}: {at:.2} dB -> {at0:.2} dB at level 0");
            // A level never predicts above its own ceiling, nor above the measurement
            // plus the drive difference.
            for to in 0..=top {
                let pred = modem.predict_snr_db(at, l, to);
                assert!(pred <= modem.ofdm.p.level_snr_ceiling_db[to as usize] + 0.01);
                assert!(pred <= at + modem.level_gain_db(l, to).max(0.0) + 0.01, "{name} {l}->{to}: {pred}");
            }
        }
        // Well below the ceilings, a loud burst's SNR overstates what a softer, faster
        // level would get by about the drive difference.
        let gain = modem.level_gain_db(top, 0);
        assert!(gain > 0.0);
        let pred = modem.predict_snr_db(5.0, 0, top);
        assert!((pred - (5.0 - gain)).abs() < 0.5, "{name}: {pred} vs {}", 5.0 - gain);
        // At the ceiling the measurement says nothing about the channel.
        let ceiling0 = modem.ofdm.p.level_snr_ceiling_db[0];
        assert_eq!(level_by_snr(&modem, ceiling0 + 1.0, 0), top);
        assert_eq!(level_by_snr(&modem, -10.0, top), 0);
        assert_eq!(level_by_snr(&modem, 60.0, 0), top);
    }
}

#[test]
fn predicted_snr_matches_the_simulator() {
    // Measured with the `level_table` benchmark (wide mode): at 24 dB CNR a level-4
    // burst reports 18.0 dB and a level-5 burst 16.9 dB; at 12 dB CNR a level-0 burst
    // reports 8.3 dB and a level-5 burst 4.7 dB.
    let modem = Modem::new(OfdmParams::wide(), 0.1);
    let p = modem.predict_snr_db(18.0, 4, 5);
    assert!((p - 16.9).abs() < 0.7, "L4 -> L5 at 24 dB CNR: {p:.1}");
    let p = modem.predict_snr_db(8.3, 0, 5);
    assert!((p - 4.7).abs() < 1.0, "L0 -> L5 at 12 dB CNR: {p:.1}");
    // Narrow: at 14 dB CNR a level-4 burst reports 15.9 dB and a level-5 burst 15.3 dB.
    let modem = Modem::new(OfdmParams::narrow(), 0.1);
    let p = modem.predict_snr_db(15.9, 4, 5);
    assert!((p - 15.3).abs() < 1.0, "narrow L4 -> L5 at 14 dB CNR: {p:.1}");
}

#[test]
fn direct_ends_retrying_together_recover() {
    // Two stations that hear each other, no digipeater. Once connected and quiet, both get
    // 2 kB at the same instant, so both start a burst of their own and lose it to the other.
    // Their retries must not keep colliding until max_retries drops the session.
    let mut dropped = Vec::new();
    for seed in 0..40 {
        let mut net = Net::new(seed, 0.0, 0.0);
        net.nodes[A].link.connect(0.0, "LA1AAA", "LA2BBB-1", &[]).unwrap();
        net.run_until(60.0, |n| n.both_connected());
        net.run_until(60.0, |n| n.now > 15.0);
        let (a, b) = (data(2000, seed), data(2000, seed + 1));
        net.nodes[A].link.send_data(&a);
        net.nodes[1].link.send_data(&b);
        let lost = |n: &Net| n.has(A, &LinkEvent::Disconnected) || n.has(1, &LinkEvent::Disconnected);
        net.run_until(300.0, |n| lost(n) || (n.nodes[A].host_rx == b && n.nodes[1].host_rx == a));
        if lost(&net) {
            dropped.push(seed);
        }
    }
    assert!(dropped.is_empty(), "sessions dropped for seeds {dropped:?}");
}
