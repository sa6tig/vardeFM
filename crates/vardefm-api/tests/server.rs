//! The server over real sockets on 127.0.0.1.

use std::io::{Read, Write};
use std::net::{SocketAddr, TcpStream};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use tungstenite::protocol::frame::coding::CloseCode;
use tungstenite::{Message, WebSocket};
use vardefm_api::server::{EVENT_ROOM, MAX_CLIENTS, RECENT};
use vardefm_api::waterfall::{Waterfall, WaterfallRow};
use vardefm_api::wire::{
    ApiEvent, Envelope, Hello, Status, StatusSnapshot, StreamHeader, WireTime, FORMAT_WATERFALL_U8, STREAM_WATERFALL,
};
use vardefm_api::{ApiServer, LoopbackAddr};

const TIMEOUT: Duration = Duration::from_secs(10);
const STATUS_SCHEMA: &str = include_str!("../../../docs/api/status.schema.json");

fn server() -> ApiServer {
    let addr: LoopbackAddr = "127.0.0.1:0".parse().unwrap();
    ApiServer::start(addr, Hello::new("narrow", &Waterfall::new(257))).unwrap()
}

/// Status line, then the whole answer, of one request with `extra` header lines.
fn request(addr: SocketAddr, head: &str) -> (String, String) {
    let mut tcp = TcpStream::connect(addr).unwrap();
    tcp.set_read_timeout(Some(TIMEOUT)).unwrap();
    tcp.write_all(head.as_bytes()).unwrap();
    let mut text = String::new();
    tcp.read_to_string(&mut text).unwrap();
    let (head, body) = text.split_once("\r\n\r\n").unwrap();
    (head.lines().next().unwrap().to_owned(), body.to_owned())
}

fn get(addr: SocketAddr, path: &str, extra: &str) -> (String, String) {
    request(addr, &format!("GET {path} HTTP/1.1\r\nHost: {addr}\r\n{extra}Connection: close\r\n\r\n"))
}

fn connect(addr: SocketAddr) -> WebSocket<TcpStream> {
    let tcp = TcpStream::connect(addr).unwrap();
    tcp.set_read_timeout(Some(TIMEOUT)).unwrap();
    tungstenite::client(format!("ws://{addr}/v1/stream"), tcp).unwrap().0
}

fn next_event(ws: &mut WebSocket<TcpStream>) -> ApiEvent {
    loop {
        if let Message::Text(text) = ws.read().unwrap() {
            let envelope: Envelope = serde_json::from_str(text.as_str()).unwrap();
            assert_eq!(envelope.v, 1);
            return envelope.event;
        }
    }
}

/// A stream past its `Hello` and first `Status`.
fn opened(addr: SocketAddr) -> WebSocket<TcpStream> {
    let mut ws = connect(addr);
    assert!(matches!(next_event(&mut ws), ApiEvent::Hello(_)));
    assert!(matches!(next_event(&mut ws), ApiEvent::Status(_)));
    ws
}

fn gap(t: u64) -> ApiEvent {
    ApiEvent::Gap { t: WireTime(t) }
}

fn busy(t: u64) -> ApiEvent {
    ApiEvent::VfmBusy { t: WireTime(t), busy: true }
}

fn row(seq: u64, bins: usize) -> WaterfallRow {
    WaterfallRow { seq, t: seq * 4800, bins: vec![seq as u8; bins] }
}

/// Waits until `done`, failing after [`TIMEOUT`].
fn wait_for(what: &str, mut done: impl FnMut() -> bool) {
    let started = Instant::now();
    while !done() {
        assert!(started.elapsed() < TIMEOUT, "timed out waiting for {what}");
        thread::sleep(Duration::from_millis(5));
    }
}

#[test]
fn listens_on_the_loopback_address_it_was_given() {
    let s = server();
    assert!(s.local_addr().ip().is_loopback());
    assert_ne!(s.local_addr().port(), 0);
}

#[test]
fn a_taken_port_is_an_error_at_start() {
    let s = server();
    let addr = LoopbackAddr::try_from(s.local_addr()).unwrap();
    let err = ApiServer::start(addr, Hello::default()).err().unwrap();
    assert!(err.to_string().contains("cannot listen on"), "{err}");
}

#[test]
fn status_answers_with_the_latest_counters_and_recent_events() {
    let s = server();
    s.publish_status(Status { gaps: 3, ..Status::default() });
    for t in 0..70 {
        s.publish_event(gap(t));
        s.publish_event(busy(t));
    }
    let (line, body) = get(s.local_addr(), "/v1/status", "");
    assert!(line.contains("200"), "{line}");
    let json: serde_json::Value = serde_json::from_str(&body).unwrap();
    let schema: serde_json::Value = serde_json::from_str(STATUS_SCHEMA).unwrap();
    assert!(jsonschema::validator_for(&schema).unwrap().is_valid(&json), "{json}");

    let snapshot: StatusSnapshot = serde_json::from_value(json).unwrap();
    assert_eq!(snapshot.v, 1);
    assert_eq!(snapshot.status.gaps, 3);
    // Only the last 64 of each kind are kept, oldest first.
    assert_eq!(snapshot.recent.len(), RECENT);
    assert_eq!((snapshot.recent[0].clone(), snapshot.recent[63].clone()), (gap(6), gap(69)));
    assert_eq!(snapshot.vfm_recent.len(), RECENT);
    assert_eq!((snapshot.vfm_recent[0].clone(), snapshot.vfm_recent[63].clone()), (busy(6), busy(69)));
}

#[test]
fn nothing_can_be_written_and_unknown_paths_are_not_found() {
    let s = server();
    let addr = s.local_addr();
    let post = format!("POST /v1/status HTTP/1.1\r\nHost: {addr}\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
    assert!(request(addr, &post).0.contains("405"));
    assert!(get(addr, "/v1/command", "").0.contains("404"));
    assert!(get(addr, "/v1/stream", "").0.contains("426"));
}

#[test]
fn a_web_page_elsewhere_is_refused_on_both_routes() {
    let s = server();
    let addr = s.local_addr();
    for path in ["/v1/status", "/v1/stream"] {
        let (line, _) = get(addr, path, "Origin: https://evil.example\r\n");
        assert!(line.contains("403"), "{path}: {line}");
    }
    assert!(get(addr, "/v1/status", "Origin: http://localhost:3000\r\n").0.contains("200"));
    let rebound = "GET /v1/status HTTP/1.1\r\nHost: evil.example\r\nConnection: close\r\n\r\n";
    assert!(request(addr, rebound).0.contains("403"));
    let no_host = "GET /v1/status HTTP/1.1\r\nConnection: close\r\n\r\n";
    assert!(request(addr, no_host).0.contains("403"));
}

#[test]
fn an_oversized_request_head_is_refused() {
    let s = server();
    let filler = "X-Filler: ".to_owned() + &"a".repeat(5000) + "\r\n";
    let (line, _) = get(s.local_addr(), "/v1/status", &filler);
    assert!(line.contains("431"), "{line}");
}

#[test]
fn an_answer_is_not_lost_to_request_bytes_left_unread() {
    // The server answers before reading the body; closing with it unread would reset the
    // connection and could take the answer with it.
    let s = server();
    let addr = s.local_addr();
    let body = "b".repeat(32 * 1024);
    let post = format!(
        "POST /v1/status HTTP/1.1\r\nHost: {addr}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{body}",
        body.len()
    );
    for _ in 0..20 {
        assert!(request(addr, &post).0.contains("405"));
    }
}

#[test]
fn a_request_head_sent_a_byte_at_a_time_does_not_hold_the_connection() {
    let s = server();
    let mut tcp = TcpStream::connect(s.local_addr()).unwrap();
    tcp.set_read_timeout(Some(TIMEOUT)).unwrap();
    let mut writer = tcp.try_clone().unwrap();
    let started = Instant::now();
    // Well inside the 2 s head timeout per byte, but the head never ends.
    let drip = thread::spawn(move || {
        for b in b"GET /v1/status HTTP/1.1\r\nX: ".iter().chain(std::iter::repeat(&b'a')) {
            if writer.write_all(&[*b]).is_err() {
                return;
            }
            thread::sleep(Duration::from_millis(100));
        }
    });
    let mut answer = Vec::new();
    let _ = tcp.read_to_end(&mut answer);
    let held = started.elapsed();
    assert!(answer.is_empty(), "an unfinished head got an answer");
    assert!(held < Duration::from_secs(4), "held for {held:?}");
    drop(tcp);
    drip.join().unwrap();
}

#[test]
fn a_stream_opens_with_hello_and_status_then_carries_events_status_and_rows() {
    let s = server();
    let mut ws = connect(s.local_addr());
    let ApiEvent::Hello(hello) = next_event(&mut ws) else { panic!("no hello first") };
    assert_eq!((hello.mode.as_str(), hello.waterfall.bins), ("vardefm-narrow", 257));
    let ApiEvent::Status(status) = next_event(&mut ws) else { panic!("no status second") };
    assert_eq!(status.mode, "vardefm-narrow");

    s.publish_event(gap(99));
    assert_eq!(next_event(&mut ws), gap(99));
    s.publish_event(busy(100));
    assert_eq!(next_event(&mut ws), busy(100));
    s.publish_status(Status { gaps: 8, ..Status::default() });
    assert!(matches!(next_event(&mut ws), ApiEvent::Status(st) if st.gaps == 8));

    let t = (1u64 << 32) + 7;
    s.publish_row(&WaterfallRow { seq: 0x1_0005, t, bins: vec![1, 2, 3] });
    let frame = loop {
        if let Message::Binary(frame) = ws.read().unwrap() {
            break frame;
        }
    };
    let (header, payload) = StreamHeader::decode(&frame).unwrap();
    assert_eq!(header, StreamHeader { stream: STREAM_WATERFALL, format: FORMAT_WATERFALL_U8, seq: 5, t_low: 7 });
    assert_eq!(payload, [1, 2, 3]);
}

#[test]
fn a_client_that_sends_anything_is_closed_as_read_only() {
    let s = server();
    let mut ws = opened(s.local_addr());
    // A second message right after the first is still unread when the server closes.
    ws.send(Message::text(r#"{"type":"Connect","call":"LA2BBB-1"}"#)).unwrap();
    ws.send(Message::text(r#"{"type":"Disconnect"}"#)).unwrap();
    loop {
        match ws.read() {
            Ok(Message::Close(Some(close))) => {
                assert_eq!(close.code, CloseCode::Policy);
                assert_eq!(close.reason.as_str(), "this API is read-only");
                break;
            }
            Ok(Message::Close(None)) | Err(_) => panic!("closed without the policy code"),
            Ok(_) => {}
        }
    }
}

#[test]
fn an_oversized_message_drops_the_connection_and_the_server_goes_on() {
    let s = server();
    let mut ws = opened(s.local_addr());
    ws.send(Message::binary(vec![0u8; 2048])).unwrap();
    loop {
        match ws.read() {
            Ok(Message::Close(Some(close))) if close.code == CloseCode::Policy => {
                panic!("an oversized message is not read, so it gets no policy close");
            }
            Err(tungstenite::Error::Io(e))
                if matches!(e.kind(), std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut) =>
            {
                panic!("the connection stayed open: {e}");
            }
            Ok(Message::Close(_)) | Err(_) => break,
            Ok(_) => {}
        }
    }
    assert!(get(s.local_addr(), "/v1/status", "").0.contains("200"));
}

#[test]
fn a_client_over_the_cap_is_turned_away() {
    let s = server();
    let streams: Vec<_> = (0..MAX_CLIENTS).map(|_| opened(s.local_addr())).collect();
    let tcp = TcpStream::connect(s.local_addr()).unwrap();
    tcp.set_read_timeout(Some(TIMEOUT)).unwrap();
    let err = tungstenite::client(format!("ws://{}/v1/stream", s.local_addr()), tcp).err().unwrap();
    let tungstenite::HandshakeError::Failure(tungstenite::Error::Http(answer)) = err else {
        panic!("not an HTTP refusal: {err:?}")
    };
    assert_eq!(answer.status(), 503);
    drop(streams);
    // Their threads notice the close within a poll; then there is room again.
    wait_for("the clients to leave", || s.clients() == 0);
    opened(s.local_addr());
}

#[test]
fn a_client_that_never_reads_loses_rows_counted_and_publishing_does_not_wait() {
    let s = server();
    let _ws = opened(s.local_addr());
    // Far more than the socket buffers hold, with nothing being read.
    let started = Instant::now();
    let sent = 20_000;
    for seq in 0..sent {
        s.publish_row(&row(seq, 2049));
    }
    assert!(started.elapsed() < TIMEOUT, "publishing waited on the client");
    assert!(s.client_rows_dropped() > 0);
    assert!(s.client_rows_dropped() < sent);
    // The server still answers, and reports the count as it is now, not as last published.
    let (_, body) = get(s.local_addr(), "/v1/status", "");
    let snapshot: StatusSnapshot = serde_json::from_str(&body).unwrap();
    assert_eq!(snapshot.status.client_rows_dropped, s.client_rows_dropped());
}

#[test]
fn a_fast_client_loses_no_rows_while_a_slow_one_does() {
    let s = server();
    let _slow = opened(s.local_addr());
    let mut fast = opened(s.local_addr());
    let received = Arc::new(AtomicU64::new(0));
    let reader = {
        let received = Arc::clone(&received);
        thread::spawn(move || {
            let mut seqs = Vec::new();
            loop {
                match fast.read() {
                    Ok(Message::Binary(frame)) => {
                        let (header, _) = StreamHeader::decode(&frame).unwrap();
                        seqs.push(header.seq);
                        received.fetch_add(1, Ordering::Release);
                    }
                    Ok(_) => {}
                    Err(_) => return seqs,
                }
            }
        })
    };
    // Rows go out only as fast as the fast client takes them, until the slow one has lost some.
    let started = Instant::now();
    let mut sent = 0u64;
    while s.client_rows_dropped() == 0 {
        assert!(started.elapsed() < 6 * TIMEOUT, "the slow client lost nothing");
        if sent - received.load(Ordering::Acquire) < 16 {
            s.publish_row(&row(sent, 2049));
            sent += 1;
        } else {
            thread::yield_now();
        }
    }
    wait_for("the fast client to catch up", || received.load(Ordering::Acquire) == sent);
    drop(s);
    let seqs = reader.join().unwrap();
    let expected: Vec<u16> = (0..sent).map(|seq| seq as u16).collect();
    assert_eq!(seqs, expected);
}

#[test]
fn a_client_that_falls_behind_on_events_is_told_how_many_it_lost() {
    let s = server();
    let mut ws = opened(s.local_addr());
    // Big events, so that the socket fills and the outbox with it while nothing is read: 64 MiB,
    // far beyond what kernel buffers, the outbox and one batch in flight can hold.
    let text = "x".repeat(16 * 1024);
    let sent = 16 * EVENT_ROOM as u64;
    for t in 0..sent {
        s.publish_event(ApiEvent::Warning { code: format!("{t}"), text: text.clone() });
    }
    let mut next = 0u64;
    let mut warnings = 0;
    while next < sent {
        match next_event(&mut ws) {
            ApiEvent::Warning { code, text } if code == "events_lagged" => {
                let lost: u64 =
                    text.strip_prefix("this client lost ").unwrap().strip_suffix(" events").unwrap().parse().unwrap();
                assert!(lost > 0);
                next += lost;
                warnings += 1;
            }
            ApiEvent::Warning { code, .. } => {
                assert_eq!(code, next.to_string(), "events out of order");
                next += 1;
            }
            other => panic!("unexpected {other:?}"),
        }
    }
    assert_eq!(next, sent, "every event arrived or was counted lost");
    assert!(warnings > 0, "nothing was lost");
}

#[test]
fn dropping_the_server_with_a_client_that_reads_nothing_returns_promptly() {
    let s = server();
    let ws = opened(s.local_addr());
    // Enough to fill the socket, then time for the client's thread to be stuck sending.
    for seq in 0..5_000 {
        s.publish_row(&row(seq, 2049));
    }
    thread::sleep(Duration::from_millis(200));
    let started = Instant::now();
    drop(s);
    // The server waits at most 0.5 s for its threads; the rest is margin for a loaded machine,
    // and still far below the 5 s a stuck send would take.
    assert!(started.elapsed() < Duration::from_secs(2), "took {:?}", started.elapsed());
    drop(ws);
}
