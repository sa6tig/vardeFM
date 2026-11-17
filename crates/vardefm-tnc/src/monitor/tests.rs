use super::*;
use std::io::{Read, Write};
use std::net::TcpStream;

fn window(samples: u64, modem_ms: u64, tap_ms: u64, longest_ms: u64) -> Window {
    Window {
        samples,
        modem: Duration::from_millis(modem_ms),
        tap: Duration::from_millis(tap_ms),
        longest: Duration::from_millis(longest_ms),
    }
}

#[test]
fn status_measures_its_window() {
    let link = LinkView { state: Link::Connected, peer: Some("LA2BBB".into()), level: Some(3), tx_queued: 1200 };
    // Two seconds of audio, half a second in the modem.
    let s = status("vardefm-narrow", 96_000, &window(96_000, 500, 20, 31), 2, &link, 5, 2400);
    assert_eq!((s.t, s.mode.as_str()), (WireTime(96_000), "vardefm-narrow"));
    assert!((s.cpu - 25.0).abs() < 1e-3, "{}", s.cpu);
    assert!((s.tap_cpu - 1.0).abs() < 1e-3, "{}", s.tap_cpu);
    assert_eq!((s.longest_push_ms, s.overruns, s.dropped_frames, s.gaps), (31, 5, 2400, 2));
    let l = s.link.unwrap();
    assert_eq!(
        (l.state, l.peer.as_deref(), l.level, l.tx_queued, l.rx_queued),
        (LinkState::Connected, Some("LA2BBB"), Some(3), 1200, 0)
    );
    // Not measured here.
    assert_eq!((s.resample_cpu, s.device_xruns, s.xrun_frames, s.device_errors), (0.0, 0, 0, 0));
    assert_eq!((s.events_dropped, s.rows_dropped), (0, 0));
}

#[test]
fn no_audio_is_no_load() {
    let s = status("vardefm-wide", 0, &window(0, 7, 3, 7), 0, &LinkView::default(), 0, 0);
    assert_eq!((s.cpu, s.tap_cpu, s.longest_push_ms), (0.0, 0.0, 7));
}

#[test]
fn every_link_state_maps() {
    for (from, to) in [
        (Link::Idle, LinkState::Idle),
        (Link::Listening, LinkState::Listening),
        (Link::Connecting, LinkState::Connecting),
        (Link::Connected, LinkState::Connected),
        (Link::Disconnecting, LinkState::Disconnecting),
    ] {
        assert_eq!(link_status(&LinkView { state: from, ..LinkView::default() }).state, to);
    }
}

fn get_status(addr: SocketAddr) -> serde_json::Value {
    let mut s = TcpStream::connect(addr).unwrap();
    write!(s, "GET /v1/status HTTP/1.1\r\nHost: {addr}\r\nConnection: close\r\n\r\n").unwrap();
    let mut answer = String::new();
    s.read_to_string(&mut answer).unwrap();
    let (head, body) = answer.split_once("\r\n\r\n").unwrap();
    assert!(head.starts_with("HTTP/1.1 200"), "{head}");
    serde_json::from_str(body).unwrap()
}

#[test]
fn a_monitor_serves_what_the_radio_gives_it() {
    let mut m = Monitor::start("127.0.0.1:0".parse().unwrap(), "narrow").unwrap();
    m.audio(0, false, &vec![0.0; 4800]);
    m.modem_step(Duration::from_millis(4));
    m.publish([
        ApiEvent::VfmBusy { t: WireTime(4800), busy: true },
        ApiEvent::Gap { t: WireTime(9600) },
        ApiEvent::Warning { code: "rx_level".into(), text: "loud".into() },
    ]);
    m.audio(9600, true, &vec![0.0; 4800]);
    let link = LinkView { state: Link::Listening, ..LinkView::default() };
    m.publish_status(14_400, &link, 1, 4800);

    let v = get_status(m.local_addr());
    let s = &v["status"];
    assert_eq!(s["mode"], "vardefm-narrow");
    assert_eq!((s["t"].as_u64(), s["gaps"].as_u64(), s["overruns"].as_u64()), (Some(14_400), Some(1), Some(1)));
    assert_eq!((s["dropped_frames"].as_u64(), s["longest_push_ms"].as_u64()), (Some(4800), Some(4)));
    assert_eq!(s["link"]["state"], "listening");
    let types =
        |k: &str| v[k].as_array().unwrap().iter().map(|e| e["type"].as_str().unwrap().to_owned()).collect::<Vec<_>>();
    assert_eq!(types("recent"), ["Gap", "Warning"]);
    assert_eq!(types("vfm_recent"), ["VfmBusy"]);
}

#[test]
fn an_unknown_mode_is_refused() {
    let e = Monitor::start("127.0.0.1:0".parse().unwrap(), "medium").err().unwrap();
    assert_eq!(e.to_string(), "mode must be narrow or wide");
}
