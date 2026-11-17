//! RadioEngine with fake sound cards and PTT lines: two stations, a full
//! session, and checks that audio only ever leaves while PTT is keyed.

use super::*;
use std::collections::VecDeque;
use std::sync::Mutex;

const CHUNK: usize = 480; // 10 ms
const KEYUP_STEPS: usize = 5; // radio needs 50 ms after PTT before it transmits

#[derive(Default)]
struct Card {
    queue: VecDeque<f32>,
    consumed: u64,
    queued: u64,
    ptt: bool,
    /// Steps since PTT went on.
    keyed_for: usize,
    /// Non-silent samples played while PTT was off.
    leaked: usize,
    /// Steps between the last audio sample and PTT release, per transmission.
    tails: Vec<usize>,
    since_audio: usize,
    ptt_events: usize,
    /// Set to stop the card from consuming (stuck driver).
    stuck: bool,
}

type Shared = Arc<Mutex<Card>>;

struct FakeOut(Shared);

impl AudioSink for FakeOut {
    fn play(&mut self, samples: &[f32]) -> u64 {
        let mut c = self.0.lock().unwrap();
        c.queue.extend(samples);
        c.queued += samples.len() as u64;
        c.queued
    }
    fn consumed(&self) -> u64 {
        self.0.lock().unwrap().consumed
    }
    fn flush(&mut self) {
        let mut c = self.0.lock().unwrap();
        c.consumed += c.queue.len() as u64;
        c.queue.clear();
    }
}

struct FakePtt(Shared);

impl Ptt for FakePtt {
    fn set(&mut self, on: bool) -> io::Result<()> {
        let mut c = self.0.lock().unwrap();
        if c.ptt && !on {
            let t = c.since_audio;
            c.tails.push(t);
        }
        if on != c.ptt {
            c.ptt_events += 1;
            c.keyed_for = 0;
        }
        c.ptt = on;
        Ok(())
    }
    fn describe(&self) -> String {
        "fake".into()
    }
}

impl Card {
    /// One 10 ms tick of the sound card. Returns what the radio puts on the air.
    fn tick(&mut self) -> Vec<f32> {
        let mut out = vec![0.0; CHUNK];
        if !self.stuck {
            for o in out.iter_mut() {
                let Some(x) = self.queue.pop_front() else { break };
                self.consumed += 1;
                *o = x;
            }
        }
        let audible = out.iter().any(|x| *x != 0.0);
        if audible {
            self.since_audio = 0;
            if !self.ptt {
                self.leaked += out.iter().filter(|x| **x != 0.0).count();
            }
        } else {
            self.since_audio += 1;
        }
        if self.ptt {
            self.keyed_for += 1;
        }
        // The transmitter only radiates once keyed long enough.
        if self.ptt && self.keyed_for > KEYUP_STEPS {
            out
        } else {
            vec![0.0; CHUNK]
        }
    }
}

fn engine(card: &Shared, seed: u32) -> RadioEngine<FakeOut> {
    let modem = Arc::new(Modem::new(OfdmParams::narrow(), 0.15));
    RadioEngine::new(modem, LinkConfig::default(), Box::new(FakePtt(card.clone())), FakeOut(card.clone()), 0.05, seed)
}

#[derive(Default)]
struct HostSide {
    lines: Vec<String>,
    data: Vec<u8>,
}

impl HostSide {
    fn take(&mut self, out: Vec<HostOut>) {
        for o in out {
            match o {
                HostOut::Line(l) => self.lines.push(l),
                HostOut::Data(d) => self.data.extend(d),
            }
        }
    }
    fn has(&self, l: &str) -> bool {
        self.lines.iter().any(|x| x == l)
    }
}

#[test]
fn session_over_fake_sound_cards() {
    session(0.0);
}

/// With the squelch open each radio hears hiss (-27 dBFS rms, as measured on a
/// handheld) whenever the other is not transmitting. The hiss must not count as
/// a busy channel, or no one would ever transmit.
#[test]
fn session_with_open_squelch() {
    session(0.045);
}

/// A full session; `hiss` is the RMS of the noise heard while the other side is silent.
fn session(hiss: f32) {
    let mut seed = 0x1234_5678u32;
    let mut hiss_chunk = move || -> Vec<f32> {
        (0..CHUNK)
            .map(|_| {
                seed ^= seed << 13;
                seed ^= seed >> 17;
                seed ^= seed << 5;
                (seed as f32 / u32::MAX as f32 * 2.0 - 1.0) * hiss * 3f32.sqrt()
            })
            .collect()
    };
    let cards: [Shared; 2] = [Arc::default(), Arc::default()];
    let mut eng = [engine(&cards[0], 1), engine(&cards[1], 2)];
    let mut host = [HostSide::default(), HostSide::default()];
    let line = |s: &str| HostIn::Line(s.into());
    let mut inbox: [Vec<HostIn>; 2] =
        [vec![line("MYCALL LA1AAA"), line("CONNECT LA1AAA LA2BBB")], vec![line("MYCALL LA2BBB"), line("LISTEN ON")]];
    let to_a: Vec<u8> = (0..2000u32).map(|i| (i * 7) as u8).collect();
    let to_b: Vec<u8> = (0..1500u32).map(|i| (i * 13 + 5) as u8).collect();
    let mut air = [vec![0.0f32; CHUNK], vec![0.0f32; CHUNK]];
    let (mut sent_b, mut sent_a, mut disc) = (false, false, false);

    for step in 0..60_000 {
        // Each station hears what the other radiated in the previous tick.
        for i in 0..2 {
            let other = &air[1 - i];
            let heard = if hiss > 0.0 && other.iter().all(|x| *x == 0.0) { hiss_chunk() } else { other.clone() };
            let out = eng[i].step(Some(&heard), std::mem::take(&mut inbox[i]));
            host[i].take(out);
        }
        for i in 0..2 {
            air[i] = cards[i].lock().unwrap().tick();
        }
        // Host behaviour: B sends a banner once connected, A answers, then A disconnects.
        if !sent_b && host[1].lines.iter().any(|l| l.starts_with("CONNECTED")) {
            inbox[1].push(HostIn::Data(to_a.clone()));
            sent_b = true;
        }
        if !sent_a && host[0].data.len() >= to_a.len() {
            inbox[0].push(HostIn::Data(to_b.clone()));
            sent_a = true;
        }
        if !disc && host[1].data.len() >= to_b.len() && host[0].lines.last().is_some_and(|l| l == "BUFFER 0") {
            inbox[0].push(line("DISCONNECT"));
            disc = true;
        }
        if host[0].has("DISCONNECTED") && host[1].has("DISCONNECTED") {
            println!("session done after {:.1} s", step as f64 * 0.01);
            break;
        }
    }
    assert_eq!(host[0].data, to_a, "A's received data");
    assert_eq!(host[1].data, to_b, "B's received data");
    assert!(host[0].has("DISCONNECTED") && host[1].has("DISCONNECTED"), "A: {:?}", host[0].lines);
    for (i, c) in cards.iter().enumerate() {
        let c = c.lock().unwrap();
        assert_eq!(c.leaked, 0, "station {i}: audio played with PTT off");
        assert!(!c.ptt, "station {i}: left keyed");
        assert!(c.ptt_events >= 4, "station {i}: only {} PTT changes", c.ptt_events);
        // Tail: released 50 ms (5 ticks) after the last sample, give or take a tick; never early.
        for &t in &c.tails {
            assert!((5..=7).contains(&t), "station {i}: PTT released {t} ticks after the last sample");
        }
    }
    for i in 0..2 {
        assert!(!eng[i].log.iter().any(|l| l.contains("watchdog")), "{:?}", eng[i].log);
        let ptt_lines = host[i].lines.iter().filter(|l| l.starts_with("PTT")).count();
        assert_eq!(ptt_lines, cards[i].lock().unwrap().ptt_events, "PTT lines to host match the real PTT");
    }
}

#[test]
fn watchdog_unkeys_a_stuck_sound_card() {
    let card: Shared = Arc::default();
    card.lock().unwrap().stuck = true;
    let mut eng = engine(&card, 3);
    let mut out =
        eng.step(None, vec![HostIn::Line("MYCALL LA1AAA".into()), HostIn::Line("CONNECT LA1AAA LA9ZZZ".into())]);
    let silence = vec![0.0f32; CHUNK];
    let mut keyed_at = None;
    for step in 0..2000 {
        out.extend(eng.step(Some(&silence), Vec::new()));
        card.lock().unwrap().tick();
        let ptt = card.lock().unwrap().ptt;
        if ptt && keyed_at.is_none() {
            keyed_at = Some(step);
        }
        if let (Some(k), false) = (keyed_at, ptt) {
            let held = (step - k) as f64 * 0.01;
            assert!(held < 6.0, "held PTT for {held} s");
            assert!(eng.log.iter().any(|l| l.contains("watchdog")));
            assert!(warned(&eng, "tx_watchdog"), "{:?}", eng.events);
            return;
        }
    }
    panic!("never unkeyed");
}

/// Station A calls; B hears A's audio scaled by `gain` and clipped at full scale.
fn hear_a_call(gain: f32) -> RadioEngine<FakeOut> {
    let cards: [Shared; 2] = [Arc::default(), Arc::default()];
    let mut a = engine(&cards[0], 4);
    let mut b = engine(&cards[1], 5);
    let mut a_in = vec![HostIn::Line("MYCALL LA1AAA".into()), HostIn::Line("CONNECT LA1AAA LA2BBB".into())];
    let mut air = vec![0.0f32; CHUNK];
    for _ in 0..300 {
        a.step(Some(&vec![0.0; CHUNK]), std::mem::take(&mut a_in));
        let heard: Vec<f32> = air.iter().map(|x| (x * gain).clamp(-1.0, 1.0)).collect();
        b.step(Some(&heard), Vec::new());
        air = cards[0].lock().unwrap().tick();
        if b.log.iter().any(|l| l.contains(" rx ")) {
            break;
        }
    }
    b
}

#[test]
fn clipped_bursts_are_reported() {
    let b = hear_a_call(4.0);
    let rx = b.log.iter().find(|l| l.contains(" rx ")).expect("B heard nothing");
    assert!(rx.contains("dBFS"), "{rx}");
    assert!(b.notices.iter().any(|l| l.contains("clipping")), "{:?}", b.notices);
    assert!(warned(&b, "rx_level"), "{:?}", b.events);
}

#[test]
fn good_bursts_are_not_reported() {
    // Our own bursts peak at full scale; 6 dB down is the target.
    let b = hear_a_call(0.5);
    assert!(b.log.iter().any(|l| l.contains(" rx ")), "B heard nothing");
    assert!(b.notices.is_empty(), "{:?}", b.notices);
    assert!(!b.events.iter().any(|e| matches!(e, ApiEvent::Warning { .. })), "{:?}", b.events);
}

#[test]
fn level_lines_only_when_asked() {
    let card: Shared = Arc::default();
    let noise: Vec<f32> = (0..CHUNK).map(|i| 0.05 * ((i * 7919) % 200) as f32 / 100.0 - 0.05).collect();
    for show in [false, true] {
        let mut eng = engine(&card, 6);
        eng.show_levels = show;
        for _ in 0..500 {
            eng.step(Some(&noise), Vec::new());
        }
        // 5 s of audio, one line per 2 s.
        let lines = eng.notices.iter().filter(|l| l.contains("dBFS rms")).count();
        assert_eq!(lines, if show { 2 } else { 0 }, "{:?}", eng.notices);
    }
}

/// A, a digipeater and B on fake sound cards, set up from a `RadioConfig` as `run`
/// does. A and B hear only the digipeater.
#[test]
fn session_through_a_digipeater_over_fake_sound_cards() {
    let cards: [Shared; 3] = [Arc::default(), Arc::default(), Arc::default()];
    let (a, d, b) = (0, 1, 2);
    let mut eng: Vec<RadioEngine<FakeOut>> = (0..3)
        .map(|i| {
            let cfg = RadioConfig {
                // Not the default, so the check below shows it reaches the link.
                hop_margin_s: 1.5,
                digi_calls: if i == d { vec!["LA3DDD".into()] } else { Vec::new() },
                ..RadioConfig::default()
            };
            let modem = Arc::new(Modem::new(OfdmParams::narrow(), cfg.txdelay_s));
            let (ptt, out) = (Box::new(FakePtt(cards[i].clone())), FakeOut(cards[i].clone()));
            RadioEngine::from_config(&cfg, modem, ptt, out, i as u32 + 1).unwrap()
        })
        .collect();
    for e in &eng {
        assert_eq!(e.station.link_config().hop_margin_s, 1.5);
    }
    let mut host = [HostSide::default(), HostSide::default(), HostSide::default()];
    let line = |s: &str| HostIn::Line(s.into());
    let mut inbox: [Vec<HostIn>; 3] = [
        vec![line("MYCALL LA1AAA"), line("CONNECT LA1AAA LA2BBB VIA LA3DDD")],
        Vec::new(),
        vec![line("MYCALL LA2BBB"), line("LISTEN ON")],
    ];
    let to_a: Vec<u8> = (0..600u32).map(|i| (i * 7) as u8).collect();
    let to_b: Vec<u8> = (0..400u32).map(|i| (i * 13 + 5) as u8).collect();
    let mut air = vec![vec![0.0f32; CHUNK]; 3];
    let (mut sent_b, mut sent_a, mut disc) = (false, false, false);
    let mut done_at = None;

    for step in 0..60_000 {
        // Each station hears what its neighbours radiated in the previous tick.
        let both: Vec<f32> = air[a].iter().zip(&air[b]).map(|(x, y)| x + y).collect();
        for i in 0..3 {
            let heard = if i == d { &both } else { &air[d] };
            let out = eng[i].step(Some(heard), std::mem::take(&mut inbox[i]));
            host[i].take(out);
        }
        for i in 0..3 {
            air[i] = cards[i].lock().unwrap().tick();
        }
        if !sent_b && host[b].lines.iter().any(|l| l.starts_with("CONNECTED")) {
            inbox[b].push(HostIn::Data(to_a.clone()));
            sent_b = true;
        }
        if !sent_a && host[a].data.len() >= to_a.len() {
            inbox[a].push(HostIn::Data(to_b.clone()));
            sent_a = true;
        }
        if !disc && host[b].data.len() >= to_b.len() && host[a].lines.last().is_some_and(|l| l == "BUFFER 0") {
            inbox[a].push(line("DISCONNECT"));
            disc = true;
        }
        if done_at.is_none() && host[a].has("DISCONNECTED") && host[b].has("DISCONNECTED") {
            println!("session done after {:.1} s", step as f64 * 0.01);
            done_at = Some(step);
        }
        // Let the digipeater finish the tail of its last repeat.
        if done_at.is_some_and(|t| step > t + 50) {
            break;
        }
    }
    assert_eq!(host[a].data, to_a, "A's received data");
    assert_eq!(host[b].data, to_b, "B's received data");
    assert!(host[a].has("DISCONNECTED") && host[b].has("DISCONNECTED"), "A: {:?}", host[a].lines);
    assert!(host[a].has("CONNECTED LA1AAA LA2BBB NARROW") && host[b].has("CONNECTED LA1AAA LA2BBB NARROW"));
    // The digipeater repeated, logged each repeat as one, and told its own host nothing
    // about a session.
    let repeats = eng[d].log.iter().filter(|l| l.contains(" rpt ")).count();
    assert!(repeats >= 6, "only {repeats} repeats: {:?}", eng[d].log);
    assert!(!eng[d].log.iter().any(|l| l.contains(" tx ")), "{:?}", eng[d].log);
    assert!(!host[d].lines.iter().any(|l| l.contains("CONNECTED") || l == "PENDING"), "{:?}", host[d].lines);
    for (i, c) in cards.iter().enumerate() {
        let c = c.lock().unwrap();
        assert_eq!(c.leaked, 0, "station {i}: audio played with PTT off");
        assert!(!c.ptt, "station {i}: left keyed");
    }
    for e in &eng {
        assert!(!e.log.iter().any(|l| l.contains("watchdog")), "{:?}", e.log);
    }
}

/// An engine built from `cfg` on a fake sound card.
fn engine_from(cfg: &RadioConfig) -> std::io::Result<RadioEngine<FakeOut>> {
    let card = Shared::default();
    let modem = Arc::new(Modem::new(OfdmParams::narrow(), 0.1));
    RadioEngine::from_config(cfg, modem, Box::new(FakePtt(card.clone())), FakeOut(card), 1)
}

#[test]
fn default_config_gives_the_links_defaults() {
    let eng = engine_from(&RadioConfig::default()).unwrap();
    let (got, want) = (eng.station.link_config(), LinkConfig::default());
    assert_eq!((got.hop_margin_s, got.max_level), (want.hop_margin_s, want.max_level));
}

#[test]
fn symbols_are_kept_only_for_the_monitor_api() {
    assert!(!engine_from(&RadioConfig::default()).unwrap().station.keeps_symbols());
    let api = RadioConfig { api: Some("127.0.0.1:8770".parse().unwrap()), ..RadioConfig::default() };
    assert!(engine_from(&api).unwrap().station.keeps_symbols());
}

#[test]
fn bad_config_is_refused() {
    let ok = RadioConfig::default;
    let bad = [
        (RadioConfig { hop_margin_s: -0.1, ..ok() }, "hop margin must be from 0 to 10 seconds"),
        (RadioConfig { hop_margin_s: 10.5, ..ok() }, "hop margin must be from 0 to 10 seconds"),
        (RadioConfig { hop_margin_s: f64::NAN, ..ok() }, "hop margin must be from 0 to 10 seconds"),
        (RadioConfig { max_level: 7, ..ok() }, "max level must be from 0 to 6"),
        (RadioConfig { txdelay_s: f32::NAN, ..ok() }, "txdelay must be 0 or more"),
        (RadioConfig { txtail_s: -0.1, ..ok() }, "txtail must be 0 or more"),
        (RadioConfig { tx_gain: f32::INFINITY, ..ok() }, "tx gain must be 0 or more"),
        (
            RadioConfig { digi_calls: vec!["LA3DDD".into(), " LA4EEE".into()], ..ok() },
            "digipeater \" LA4EEE\": not a valid call sign",
        ),
        (RadioConfig { digi_calls: vec!["".into()], ..ok() }, "digipeater \"\": not a valid call sign"),
    ];
    for (cfg, why) in bad {
        assert_eq!(cfg.check().unwrap_err().to_string(), why);
        assert_eq!(engine_from(&cfg).err().map(|e| e.to_string()).as_deref(), Some(why));
    }
    let edge = RadioConfig { hop_margin_s: 10.0, max_level: 6, digi_calls: vec!["la3ddd-7".into()], ..ok() };
    assert!(engine_from(&edge).is_ok());
}

fn warned(eng: &RadioEngine<FakeOut>, code: &str) -> bool {
    eng.events.iter().any(|e| matches!(e, ApiEvent::Warning { code: c, .. } if c == code))
}

/// A's first burst as A sent it and B heard it.
struct Call {
    /// Where the burst's audio starts on B's receive clock.
    start: u64,
    tx: VfmBurstTx,
    rx: VfmBurstRx,
    /// B's events.
    events: Vec<ApiEvent>,
    /// B's constellation frames.
    constellations: Vec<Vec<u8>>,
    /// The step in which B decoded the burst.
    decoded_in: usize,
    b: RadioEngine<FakeOut>,
}

/// A calls B until B has heard A's first burst and the channel is free again. For each
/// `(step, n)` in `gaps`, B loses `n` samples of audio before its step `step`. B keeps the
/// symbols it receives if `keep`.
fn a_calls_b(gaps: &[(usize, u64)], keep: bool) -> Call {
    let cards: [Shared; 2] = [Arc::default(), Arc::default()];
    let mut a = engine(&cards[0], 4);
    let mut b = engine(&cards[1], 5);
    b.station.keep_symbols(keep);
    let mut constellations = Vec::new();
    let mut a_in = vec![HostIn::Line("MYCALL LA1AAA".into()), HostIn::Line("CONNECT LA1AAA LA2BBB".into())];
    let mut air = vec![0.0f32; CHUNK];
    let (mut sent, mut decoded_in) = (None, None);
    let mut events = Vec::new();
    for step in 0..1000 {
        a.step(Some(&vec![0.0; CHUNK]), std::mem::take(&mut a_in));
        for &(_, lost) in gaps.iter().filter(|g| g.0 == step) {
            b.audio_lost(lost);
        }
        b.step(Some(&air), Vec::new());
        if decoded_in.is_none() && b.events.iter().any(|e| matches!(e, ApiEvent::VfmBurstRx(_))) {
            decoded_in = Some(step);
        }
        events.append(&mut b.events);
        constellations.append(&mut b.constellations);
        // A's card starts playing the burst in this tick; B hears that in its next step.
        if sent.is_none() {
            if let Some(ApiEvent::VfmBurstTx(t)) = a.events.first() {
                assert_eq!(t.t, WireTime(a.rx_samples()), "keyed now");
                sent = Some((b.rx_samples(), t.clone()));
            }
        }
        air = cards[0].lock().unwrap().tick();
        let free = events.iter().rev().find_map(|e| match e {
            ApiEvent::VfmBusy { busy, .. } => Some(!busy),
            _ => None,
        });
        if decoded_in.is_some() && free == Some(true) {
            break;
        }
    }
    let (start, tx) = sent.expect("A never keyed");
    let rx = events.iter().find_map(|e| match e {
        ApiEvent::VfmBurstRx(r) => Some(r.clone()),
        _ => None,
    });
    let rx = rx.unwrap_or_else(|| panic!("B heard nothing: {events:?}"));
    Call { start, tx, rx, events, constellations, decoded_in: decoded_in.unwrap(), b }
}

/// A calls B, which lost audio before it heard anything. B's events give the burst's true
/// place on its receive clock, which counts the lost audio although the receiver never saw it.
#[test]
fn burst_events_are_timed_on_the_receive_clock() {
    const LOST: u64 = 12_345;
    let mut b = engine(&Arc::default(), 5);
    b.audio_lost(0);
    assert!(b.events.is_empty());
    b.audio_lost(LOST);
    assert_eq!((b.rx_samples(), &b.events[..]), (LOST, &[ApiEvent::Gap { t: WireTime(LOST) }][..]));

    let Call { start, tx, rx, events, b, .. } = a_calls_b(&[(0, LOST)], false);
    assert_eq!(events[0], ApiEvent::Gap { t: WireTime(LOST) });
    assert_eq!((tx.kind.as_str(), tx.level, tx.ncw, tx.retry, tx.repeat), ("ConnectReq", Some(0), 1, 0, false));
    assert_eq!((rx.kind.as_str(), rx.level, rx.ncw, rx.cw_ok), ("ConnectReq", Some(0), 1, 1));
    // The receiver places the burst after the leader to within half a cyclic prefix.
    let p = &b.station.modem().ofdm.p;
    let symbol = p.symbol_len() as i64;
    let leader = (0.15 / p.symbol_duration()).ceil() as i64 * symbol;
    let near = p.ncp as i64 / 2;
    let end = start as i64 + (tx.air_s * RATE as f32).round() as i64;
    let (t, t_end) = (rx.t.0 as i64, rx.t_end.0 as i64);
    assert!((t - (start as i64 + leader)).abs() < near, "starts {} samples off", t - start as i64 - leader);
    assert!((t_end - end).abs() < near, "ends {} samples off", t_end - end);

    // The channel went busy during the burst and free after it.
    let busy: Vec<_> = events
        .iter()
        .filter_map(|e| match e {
            ApiEvent::VfmBusy { t, busy } => Some((t.0 as i64, *busy)),
            _ => None,
        })
        .collect();
    // (The detector may let go for a moment within it, as the host's BUSY lines do.)
    assert!(busy.iter().enumerate().all(|(i, &(_, on))| on == (i % 2 == 0)), "{busy:?}");
    assert!(busy.windows(2).all(|w| w[0].0 < w[1].0), "{busy:?}");
    assert!((start as i64..end).contains(&busy[0].0), "{busy:?}, burst {start}..{end}");
    assert!(busy.last().unwrap().0 > end, "{busy:?}, burst {start}..{end}");
}

/// The receiver decodes a burst a little after it ends: more than a chunk after, as the
/// demodulator reads a symbol past the burst. Audio lost in between, in one gap or two, comes
/// after the burst and does not move it.
#[test]
fn gaps_after_a_burst_do_not_move_it() {
    let clean = a_calls_b(&[], false);
    let n = clean.decoded_in;
    for gaps in [&[(n, 777)][..], &[(n - 1, 500), (n, 777)]] {
        let gap = a_calls_b(gaps, false);
        assert_eq!(gap.decoded_in, n);
        assert_eq!((gap.rx.t, gap.rx.t_end), (clean.rx.t, clean.rx.t_end), "gaps {gaps:?}");
        let at: Vec<_> = gap
            .events
            .iter()
            .filter_map(|e| match e {
                ApiEvent::Gap { t } => Some(*t),
                _ => None,
            })
            .collect();
        assert_eq!(at.len(), gaps.len());
        assert!(at.iter().all(|&t| t > gap.rx.t_end), "gaps at {at:?}, burst ends at {:?}", gap.rx.t_end);
    }
}

/// With symbols kept, B sends the constellation of A's burst, timed as its `VfmBurstRx`, in
/// spite of audio lost before it; without, none.
#[test]
fn a_received_burst_has_its_constellation() {
    use vardefm_api::constellation::{ConstellationFrame, MAX_POINTS, SCALE, STREAM_VFM_CONSTELLATION};
    use vardefm_api::wire::StreamHeader;

    let call = a_calls_b(&[(0, 4321)], true);
    assert_eq!(call.constellations.len(), 1);
    let (header, payload) = StreamHeader::decode(&call.constellations[0]).unwrap();
    assert_eq!((header.stream, header.seq, header.t_low), (STREAM_VFM_CONSTELLATION, 0, call.rx.t.0 as u32));
    let f = ConstellationFrame::decode(payload).unwrap();
    let modem = call.b.station.modem();
    let carriers = modem.ofdm.data_carriers();
    assert_eq!((f.level, f.ncw, f.bits_per_symbol, usize::from(f.data_carriers)), (0, 1, 1, carriers));
    // A level-0 codeword has more points than a frame holds: every other symbol is kept.
    let symbols = modem.symbols_per_codeword(0);
    assert!(symbols * carriers > MAX_POINTS && symbols * carriers <= 2 * MAX_POINTS);
    assert_eq!(f.points.len(), symbols.div_ceil(2) * carriers);
    let carrier_ids: Vec<u8> = modem.ofdm.data_idx.iter().map(|&c| c as u8).collect();
    for (n, p) in f.points.iter().enumerate() {
        assert_eq!(p.carrier, carrier_ids[n % carriers]);
        // BPSK, heard clean: on the in-phase axis at +-1.
        let s = SCALE as i8;
        assert!((p.i.abs() - s).abs() <= 4 && p.q.abs() <= 4, "point {n}: {p:?}");
    }

    let off = a_calls_b(&[], false);
    assert!(off.constellations.is_empty());
}
