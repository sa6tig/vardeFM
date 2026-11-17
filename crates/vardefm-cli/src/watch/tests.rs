use std::io::Cursor;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::sync::Arc;
use std::time::{Duration, Instant};

use num_complex::Complex32;
use ratatui::backend::TestBackend;
use ratatui::crossterm::event::{KeyCode, KeyModifiers};
use ratatui::Terminal;
use vardefm_api::constellation::Constellation;
use vardefm_api::waterfall::{Waterfall, WaterfallRow};
use vardefm_api::wire::{ApiEvent, Envelope, Hello, Status, StreamHeader, VfmBurstRx, VfmBurstTx, WireTime};
use vardefm_api::{ApiServer, LoopbackAddr};

use super::capture::{read_all, CaptureWriter, Message, Record};
use super::model::{unwrap_t, Entry, Floor, Model, FLOOR_ROWS};
use super::source::{self, Feed};
use super::view::{self, bands, colour, ideal_points, ViewOpts};
use super::{drain, key};

const NARROW_BINS: usize = 257;

fn text(event: ApiEvent) -> Message {
    Message::Text(serde_json::to_string(&Envelope::new(event)).unwrap())
}

fn hello() -> Hello {
    Hello::new("narrow", &Waterfall::new(NARROW_BINS))
}

fn status(t: u64) -> ApiEvent {
    ApiEvent::Status(Status { t: WireTime(t), mode: "vardefm-narrow".into(), ..Status::default() })
}

fn burst_rx(t: u64, ncw: u8, cw_ok: u8) -> VfmBurstRx {
    VfmBurstRx {
        t: WireTime(t),
        t_end: WireTime(t + 24_000),
        kind: "Data".into(),
        level: Some(3),
        ncw,
        cw_ok,
        snr_db: 18.5,
        peak_dbfs: -6.0,
    }
}

fn row(seq: u64, t: u64, value: u8) -> Message {
    let mut frame = StreamHeader::waterfall(seq, WireTime(t)).encode().to_vec();
    frame.extend(std::iter::repeat_n(value, NARROW_BINS));
    Message::Binary(frame)
}

fn constellation_frame(t: u64) -> Vec<u8> {
    let carriers = [3usize, 5, 7];
    let symbols = vec![vec![Complex32::new(0.7, -0.7), Complex32::new(-0.7, 0.7), Complex32::new(0.7, 0.7)]; 2];
    Constellation { t: WireTime(t), level: 3, ncw: 2, bits_per_symbol: 2, carriers: &carriers, symbols: &symbols }
        .encode(0)
}

// ---- capture ----

#[test]
fn capture_lines_round_trip() {
    let records = vec![
        Record { ms: 0, msg: text(ApiEvent::Gap { t: WireTime(5) }) },
        Record { ms: 1234, msg: Message::Binary(vec![0, 1, 2, 255, 128]) },
        Record { ms: 1235, msg: Message::Text("{\"v\":1,\"type\":\"FromTheFuture\",\"x\":\"a\\nb\"}".into()) },
    ];
    let mut out = Vec::new();
    let mut w = CaptureWriter::new(&mut out);
    for r in &records {
        w.write(r).unwrap();
    }
    let text = String::from_utf8(out).unwrap();
    assert_eq!(text.lines().count(), 3, "one line per record, even with a newline in the text");
    assert!(text.lines().nth(1).unwrap().contains("\"bin\":\"AAEC/4A=\""));
    assert_eq!(read_all(Cursor::new(text)).unwrap(), records);
}

#[test]
fn capture_errors_name_the_line_and_blank_lines_are_skipped() {
    assert_eq!(read_all(Cursor::new("\n{\"ms\":1,\"text\":\"x\"}\n\n")).unwrap().len(), 1);
    for (bad, why) in [
        ("{\"ms\":1}", "text"),
        ("{\"ms\":1,\"text\":\"a\",\"bin\":\"AA==\"}", "text"),
        ("{\"text\":\"a\"}", "ms"),
        ("{\"ms\":1,\"bin\":\"not base64!\"}", ""),
        ("{\"ms\":1,\"text\":5}", "text"),
        ("not json", ""),
    ] {
        let err = read_all(Cursor::new(format!("{{\"ms\":0,\"text\":\"ok\"}}\n{bad}\n"))).unwrap_err();
        assert!(err.starts_with("line 2: ") && err.contains(why), "{bad}: {err}");
    }
}

// ---- time ----

#[test]
fn frame_times_unwrap_to_the_nearest_full_time() {
    const S: u64 = 1 << 32;
    assert_eq!(unwrap_t(100, 0), 100);
    assert_eq!(unwrap_t(4800, 4000), 4800);
    // Just past a wrap: the reference is below 2^32, the frame's low bits small.
    assert_eq!(unwrap_t(10, S - 50), S + 10);
    // A frame from just before a wrap, the reference already past it.
    assert_eq!(unwrap_t((S - 10) as u32, S + 20), S - 10);
    assert_eq!(unwrap_t(7, 5 * S + 3), 5 * S + 7);
    // A low time near the top of the range with reference 0 must not wrap below zero.
    assert_eq!(unwrap_t(u32::MAX, 0), u64::from(u32::MAX));
}

// ---- noise floor ----

#[test]
fn floor_is_the_tenth_percentile_of_the_last_rows() {
    let mut floor = Floor::default();
    assert_eq!(floor.value(), None);
    // 10 bins: one at 10, nine at 200. The 10 % quantile is the lowest.
    let mut bins = vec![200u8; 10];
    bins[0] = 10;
    floor.push(&bins);
    assert_eq!(floor.value(), Some(10));
    // A second low bin: the quantile is still the lowest one of ten.
    bins[1] = 20;
    floor.clear();
    floor.push(&bins);
    assert_eq!(floor.value(), Some(10));
    // Old rows fall out of the window.
    for _ in 0..FLOOR_ROWS {
        floor.push(&[150u8; 10]);
    }
    assert_eq!(floor.value(), Some(150));
}

// ---- drawing helpers ----

#[test]
fn bands_put_the_highest_frequencies_first_and_take_the_largest_bin() {
    let bins: Vec<u8> = (0..8).collect();
    assert_eq!(bands(&bins, 4), vec![7, 5, 3, 1]);
    assert_eq!(bands(&bins, 1), vec![7]);
    // More bands than bins: each band still shows a bin, none beyond the end.
    assert_eq!(bands(&[1, 9], 4), vec![9, 9, 1, 1]);
    assert_eq!(bands(&[], 3), vec![0, 0, 0]);
    // Narrow: 257 bins in 30 bands; the top band holds bin 256, the bottom one bin 0.
    let narrow: Vec<u8> = (0..=255).chain([255]).collect::<Vec<u8>>();
    let b = bands(&narrow, 30);
    assert_eq!((b.len(), b[0], b[29]), (30, 255, 7));
}

#[test]
fn colours_run_from_black_to_white_and_ignore_nan() {
    use ratatui::style::Color::Rgb;
    assert_eq!(colour(0.0), Rgb(0, 0, 0));
    assert_eq!(colour(-3.0), Rgb(0, 0, 0));
    assert_eq!(colour(f32::NAN), Rgb(0, 0, 0));
    assert_eq!(colour(1.0), Rgb(255, 255, 255));
    assert_eq!(colour(7.0), Rgb(255, 255, 255));
}

#[test]
fn ideal_points_have_unit_mean_energy() {
    for (bps, n) in [(1, 2), (2, 4), (4, 16), (6, 64)] {
        let pts = ideal_points(bps);
        assert_eq!(pts.len(), n, "{bps} bits");
        let energy = pts.iter().map(|(i, q)| i * i + q * q).sum::<f64>() / n as f64;
        assert!((energy - 1.0).abs() < 1e-9, "{bps} bits: {energy}");
    }
    let outer = ideal_points(6).iter().map(|p| p.0).fold(0.0, f64::max);
    assert!((outer - 7.0 / 42f64.sqrt()).abs() < 1e-9, "64QAM's outer points sit at 1.08");
    assert!(ideal_points(3).is_empty() && ideal_points(0).is_empty());
}

#[test]
fn keys_move_the_floor_and_span_and_quit() {
    let none = KeyModifiers::NONE;
    let o = ViewOpts::default();
    assert_eq!(key(o, KeyCode::Up, none).unwrap().floor_offset_db, 3.0);
    assert_eq!(key(o, KeyCode::Down, none).unwrap().floor_offset_db, -3.0);
    let mut wide = o;
    for _ in 0..20 {
        wide = key(wide, KeyCode::Char('+'), none).unwrap();
    }
    assert_eq!(wide.span_db, view::MAX_SPAN_DB);
    let mut narrow = o;
    for _ in 0..20 {
        narrow = key(narrow, KeyCode::Char('-'), none).unwrap();
    }
    assert_eq!(narrow.span_db, view::MIN_SPAN_DB);
    assert_eq!(key(wide, KeyCode::Char('a'), none), Some(ViewOpts::default()));
    assert_eq!(key(o, KeyCode::Char('q'), none), None);
    assert_eq!(key(o, KeyCode::Esc, none), None);
    assert_eq!(key(o, KeyCode::Char('c'), KeyModifiers::CONTROL), None);
    assert_eq!(key(o, KeyCode::Char('c'), none), Some(o));
}

// ---- model ----

/// A model that has seen a narrow `Hello`, a status, a received burst, its constellation, a
/// sent burst, busy changes, a gap, a warning and three seconds of rows.
fn busy_model() -> Model {
    let mut m = Model::new("test".into());
    m.apply(&text(ApiEvent::Hello(hello())));
    m.apply(&text(status(48_000)));
    for i in 0..30u64 {
        m.apply(&row(i, 48_000 + i * 4800, if (5..10).contains(&i) { 200 } else { 40 }));
    }
    m.apply(&text(ApiEvent::VfmBurstRx(burst_rx(48_000 + 24_000, 2, 1))));
    m.apply(&Message::Binary(constellation_frame(48_000 + 24_000)));
    m.apply(&text(ApiEvent::VfmBurstTx(VfmBurstTx {
        t: WireTime(48_000 + 72_000),
        kind: "Ack".into(),
        ncw: 1,
        bytes: 40,
        resent_bytes: 10,
        air_s: 0.3,
        ..VfmBurstTx::default()
    })));
    m.apply(&text(ApiEvent::VfmBusy { t: WireTime(48_000 + 100_000), busy: true }));
    m.apply(&text(ApiEvent::Gap { t: WireTime(48_000 + 130_000) }));
    m.apply(&text(ApiEvent::Warning { code: "rx_level".into(), text: "too loud".into() }));
    m
}

#[test]
fn model_keeps_what_the_stream_said() {
    let m = busy_model();
    assert_eq!(m.session, 1);
    assert_eq!(m.rows.len(), 30);
    assert_eq!(m.rows.back().unwrap().t, 48_000 + 29 * 4800);
    assert_eq!(m.log.len(), 3, "rx, tx and gap");
    assert!(matches!(m.log[2].entry, Entry::Gap(178_000)));
    let shown = m.constellation.as_ref().unwrap();
    assert_eq!(shown.t, 72_000);
    assert_eq!(shown.frame.points.len(), 6);
    assert_eq!(shown.burst.as_ref().map(|b| b.cw_ok), Some(1), "matched to its VfmBurstRx");
    assert!(!m.busy_at(148_000 - 1) && m.busy_at(148_000));
    assert_eq!(m.warnings.back().unwrap().1, "rx_level");
    assert_eq!(m.unknown, 0);
}

#[test]
fn model_skips_what_it_cannot_read() {
    let mut m = busy_model();
    m.apply(&Message::Text("{\"v\":1,\"type\":\"FrameRx\"}".into()));
    m.apply(&Message::Text("not json".into()));
    m.apply(&Message::Binary(vec![1, 2, 3]));
    // A row of the wrong width.
    let mut short = StreamHeader::waterfall(99, WireTime(500_000)).encode().to_vec();
    short.extend([0u8; 10]);
    m.apply(&Message::Binary(short));
    assert_eq!(m.unknown, 4);
    // A stream id nobody knows is skipped without complaint.
    m.apply(&Message::Binary(vec![9, 1, 0, 0, 0, 0, 0, 0, 42]));
    assert_eq!(m.unknown, 4);
    assert_eq!(m.rows.len(), 30);
}

#[test]
fn a_new_hello_starts_over_but_keeps_the_log() {
    let mut m = busy_model();
    m.apply(&text(ApiEvent::Hello(Hello::new("wide", &Waterfall::new(641)))));
    assert_eq!(m.session, 2);
    assert!(m.rows.is_empty() && m.constellation.is_none() && m.busy.is_empty() && m.status.is_none());
    assert_eq!(m.floor.value(), None);
    assert_eq!(m.latest_t, 0);
    assert!(matches!(&m.log.back().unwrap().entry, Entry::Restart { mode } if mode == "vardefm-wide"));
    assert_eq!(m.log.len(), 4);
    // The old session's burst is not matched to a new constellation at the same time.
    m.apply(&Message::Binary(constellation_frame(72_000)));
    assert_eq!(m.constellation.unwrap().burst, None);
}

#[test]
fn draws_at_any_size_without_panicking() {
    let full = busy_model();
    let empty = Model::new("nothing yet".into());
    let opts = ViewOpts { floor_offset_db: -200.0, span_db: view::MIN_SPAN_DB };
    for (w, h) in [(1, 1), (5, 3), (20, 6), (40, 12), (80, 24), (250, 70)] {
        for m in [&full, &empty] {
            let mut terminal = Terminal::new(TestBackend::new(w, h)).unwrap();
            terminal.draw(|f| view::draw(f, m, &ViewOpts::default())).unwrap();
            terminal.draw(|f| view::draw(f, m, &opts)).unwrap();
        }
    }
}

#[test]
fn the_screen_shows_the_burst_and_the_waterfall() {
    let m = busy_model();
    let mut terminal = Terminal::new(TestBackend::new(120, 40)).unwrap();
    terminal.draw(|f| view::draw(f, &m, &ViewOpts::default())).unwrap();
    let buf = terminal.backend().buffer();
    let screen: String = buf.content().iter().map(|c| c.symbol()).collect();
    for want in ["Data", "1/2", "18.5", "Ack", "40 (10)", "GAP", "rx_level", "channel busy", "Waterfall 0-3.0 kHz"] {
        assert!(screen.contains(want), "missing {want:?}");
    }
    // The loud rows 5 to 9 are drawn brighter than the quiet ones around them.
    let half_blocks: Vec<_> = buf.content().iter().filter(|c| c.symbol() == "\u{2580}").collect();
    assert!(!half_blocks.is_empty());
    let distinct: std::collections::HashSet<_> = half_blocks.iter().map(|c| format!("{:?}", c.fg)).collect();
    assert!(distinct.len() >= 2, "quiet and loud rows look alike");
}

// ---- sources ----

fn wait_for(rx: &mpsc::Receiver<Feed>, model: &mut Model, done: impl Fn(&Model) -> bool) {
    let until = Instant::now() + Duration::from_secs(10);
    while !done(model) {
        assert!(Instant::now() < until, "timed out; model: {model:?}");
        drain(model, rx);
        std::thread::sleep(Duration::from_millis(10));
    }
}

/// Watch a real server while saving, then replay the capture: the replay shows the same.
#[test]
fn a_saved_capture_replays_what_was_watched() {
    let addr: LoopbackAddr = "127.0.0.1:0".parse().unwrap();
    let server = ApiServer::start(addr, hello()).unwrap();
    let path = std::env::temp_dir().join(format!("vardefm-watch-{}.jsonl", std::process::id()));
    let (tx, rx) = mpsc::channel();
    let stop = Arc::new(AtomicBool::new(false));
    let handle = source::live(server.local_addr(), Some(path.clone()), tx, stop.clone()).unwrap();

    let mut live = Model::new("live".into());
    wait_for(&rx, &mut live, |m| m.hello.is_some() && m.status.is_some());
    server.publish_status(Status { t: WireTime(96_000), ..Status::default() });
    for i in 0..5u64 {
        server.publish_row(&WaterfallRow { seq: i, t: 96_000 + i * 4800, bins: vec![i as u8 * 50; NARROW_BINS] });
    }
    server.publish_event(ApiEvent::VfmBurstRx(burst_rx(100_000, 2, 2)));
    server.publish_frame(constellation_frame(100_000));
    wait_for(&rx, &mut live, |m| m.constellation.is_some() && m.rows.len() == 5);
    stop.store(true, Ordering::Relaxed);
    handle.join().unwrap();
    drop(server);

    let (tx, rx) = mpsc::channel();
    let handle = source::replay(path.clone(), 1000.0, tx, Arc::new(AtomicBool::new(false))).unwrap();
    let mut replayed = Model::new("replay".into());
    wait_for(&rx, &mut replayed, |m| m.conn == super::model::Conn::Ended);
    handle.join().unwrap();
    std::fs::remove_file(&path).unwrap();

    assert_eq!(replayed.hello, live.hello);
    assert_eq!(replayed.rows, live.rows);
    assert_eq!(replayed.log, live.log);
    assert_eq!(replayed.constellation, live.constellation);
    assert_eq!(replayed.constellation.unwrap().burst.unwrap().cw_ok, 2);
}

#[test]
fn live_source_reports_a_modem_that_is_not_there_and_stops() {
    // Bind and drop a listener, so nothing listens on its port.
    let port = std::net::TcpListener::bind("127.0.0.1:0").unwrap().local_addr().unwrap();
    let (tx, rx) = mpsc::channel();
    let stop = Arc::new(AtomicBool::new(false));
    let handle = source::live(port, None, tx, stop.clone()).unwrap();
    let mut m = Model::new("x".into());
    wait_for(&rx, &mut m, |m| matches!(m.conn, super::model::Conn::Lost(_)));
    stop.store(true, Ordering::Relaxed);
    let t = Instant::now();
    handle.join().unwrap();
    assert!(t.elapsed() < Duration::from_secs(2), "stop took {:?}", t.elapsed());
}

#[test]
fn replay_refuses_a_bad_capture_before_starting() {
    let path = std::env::temp_dir().join(format!("vardefm-watch-bad-{}.jsonl", std::process::id()));
    std::fs::write(&path, "{\"ms\":0,\"text\":\"x\"}\nbroken\n").unwrap();
    let (tx, _rx) = mpsc::channel();
    let err = source::replay(path.clone(), 1.0, tx, Arc::new(AtomicBool::new(false))).unwrap_err();
    std::fs::remove_file(&path).unwrap();
    assert!(err.contains("line 2"), "{err}");
}

#[test]
fn warnings_wrap_under_their_heading() {
    use super::view::hanging;
    assert_eq!(
        hanging("receive level high: peaks at -1 dBFS", 21, 2),
        vec!["  receive level high:", "  peaks at -1 dBFS"]
    );
    assert_eq!(hanging("abcdefgh ij", 6, 2), vec!["  abcd", "  efgh", "  ij"]);
    assert_eq!(hanging("", 10, 2), Vec::<String>::new());
    assert_eq!(hanging("x", 0, 2), vec!["  x"]);
}
