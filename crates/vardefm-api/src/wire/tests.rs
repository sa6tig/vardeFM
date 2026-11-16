use super::*;
use serde_json::{json, Value};

const SHARED_ENVELOPE: &str = include_str!("../../../../docs/api/envelope.schema.json");
const SHARED_STATUS: &str = include_str!("../../../../docs/api/status.schema.json");
const VFM_EVENTS: &str = include_str!("../../../../docs/api/vardefm-events.schema.json");

fn validator(schema: &str) -> jsonschema::Validator {
    let schema: Value = serde_json::from_str(schema).unwrap();
    jsonschema::validator_for(&schema).unwrap()
}

/// One of every shared message.
fn shared_events() -> Vec<ApiEvent> {
    vec![
        ApiEvent::Hello(Hello::new("narrow", 257)),
        ApiEvent::Hello(Hello::new("wide", 641)),
        ApiEvent::Status(Status {
            t: WireTime(u64::from(u32::MAX) + 5),
            mode: mode_name("wide"),
            cpu: 3.5,
            tap_cpu: 0.25,
            resample_cpu: 0.125,
            longest_push_ms: 53,
            overruns: 1,
            dropped_frames: 2,
            device_xruns: 3,
            xrun_frames: 4,
            device_errors: 5,
            gaps: 6,
            events_dropped: 7,
            rows_dropped: 8,
            client_rows_dropped: 9,
            capture_priority: CallbackPriority::Unknown,
            link: None,
        }),
        ApiEvent::Status(Status {
            mode: mode_name("narrow"),
            link: Some(LinkStatus {
                state: LinkState::Disconnecting,
                peer: Some("LA2BBB-1".to_owned()),
                level: Some(5),
                tx_queued: 1200,
                rx_queued: 0,
            }),
            ..Status::default()
        }),
        ApiEvent::Gap { t: WireTime(7) },
        ApiEvent::Warning { code: "rx_level".to_owned(), text: "received burst peaks at -0.1 dBFS".to_owned() },
    ]
}

/// One of every message only vardeFM sends, with and without the optional fields.
fn vfm_events() -> Vec<ApiEvent> {
    vec![
        ApiEvent::VfmBurstRx(VfmBurstRx {
            t: WireTime(48_000),
            t_end: WireTime(96_000),
            kind: "Data".to_owned(),
            level: Some(4),
            ncw: 12,
            cw_ok: 11,
            snr_db: 17.5,
            peak_dbfs: -6.25,
        }),
        ApiEvent::VfmBurstRx(VfmBurstRx { kind: "ConnectReq".to_owned(), level: None, ..VfmBurstRx::default() }),
        ApiEvent::VfmBurstTx(VfmBurstTx {
            t: WireTime(200_000),
            kind: "Data".to_owned(),
            level: Some(7),
            ncw: 20,
            bytes: 2400,
            resent_bytes: 120,
            retry: 1,
            air_s: 1.75,
            repeat: false,
        }),
        ApiEvent::VfmBurstTx(VfmBurstTx { kind: "Disc".to_owned(), repeat: true, ..VfmBurstTx::default() }),
        ApiEvent::VfmBusy { t: WireTime(5), busy: true },
    ]
}

fn wire(event: ApiEvent) -> Value {
    serde_json::to_value(Envelope::new(event)).unwrap()
}

#[test]
fn shared_messages_fit_the_shared_schema_and_round_trip() {
    let shared = validator(SHARED_ENVELOPE);
    for event in shared_events() {
        assert!(!event.is_vfm());
        let json = wire(event.clone());
        assert_eq!(json["v"], 1);
        assert!(shared.is_valid(&json), "{json}");
        let back: Envelope = serde_json::from_value(json).unwrap();
        assert_eq!(back.event, event);
    }
}

#[test]
fn vfm_messages_fit_their_own_schema_and_round_trip() {
    let vfm = validator(VFM_EVENTS);
    for event in vfm_events() {
        assert!(event.is_vfm());
        let json = wire(event.clone());
        assert!(json["type"].as_str().unwrap().starts_with("Vfm"));
        assert!(vfm.is_valid(&json), "{json}");
        let back: Envelope = serde_json::from_value(json).unwrap();
        assert_eq!(back.event, event);
    }
}

#[test]
fn a_client_of_the_shared_messages_cannot_mistake_a_vfm_message_for_one() {
    // Such a client skips any message its types refuse; the shared schema stands for them.
    let shared = validator(SHARED_ENVELOPE);
    for event in vfm_events() {
        let json = wire(event);
        assert!(!shared.is_valid(&json), "{json}");
    }
}

#[test]
fn the_vfm_schema_refuses_what_is_not_one_of_its_messages() {
    let vfm = validator(VFM_EVENTS);
    for event in shared_events() {
        assert!(!vfm.is_valid(&wire(event)));
    }
    let good = json!({"v": 1, "type": "VfmBusy", "t": 5, "busy": true});
    assert!(vfm.is_valid(&good));
    for bad in [
        json!({"type": "VfmBusy", "t": 5, "busy": true}),
        json!({"v": 1, "type": "VfmBusy", "busy": true}),
        json!({"v": 1, "type": "VfmBusy", "t": -1, "busy": true}),
        json!({"v": 1, "type": "VfmBurstRx", "t": 0, "t_end": 0, "kind": "Data", "ncw": 1, "cw_ok": 1,
               "snr_db": null, "peak_dbfs": 0.0}),
        json!({"v": 1, "type": "VfmBurstTx", "t": 0, "kind": "Data", "level": 300, "ncw": 1, "bytes": 0,
               "resent_bytes": 0, "retry": 0, "air_s": 0.0, "repeat": false}),
    ] {
        assert!(!vfm.is_valid(&bad), "{bad}");
    }
}

#[test]
fn the_status_answer_fits_the_shared_schema_with_vfm_events_kept_apart() {
    let sent = StatusSnapshot {
        v: API_VERSION,
        status: Status { mode: mode_name("narrow"), ..Status::default() },
        recent: shared_events()
            .into_iter()
            .filter(|e| matches!(e, ApiEvent::Gap { .. } | ApiEvent::Warning { .. }))
            .collect(),
        vfm_recent: vfm_events(),
    };
    let json = serde_json::to_value(&sent).unwrap();
    assert!(validator(SHARED_STATUS).is_valid(&json), "{json}");
    let back: StatusSnapshot = serde_json::from_value(json.clone()).unwrap();
    assert_eq!(back, sent);

    // A Vfm event in `recent` would make a client of the shared messages refuse the whole answer.
    let mut mixed = json;
    mixed["recent"].as_array_mut().unwrap().push(wire(vfm_events().remove(0)));
    assert!(!validator(SHARED_STATUS).is_valid(&mixed));
}

#[test]
fn floats_that_are_not_finite_go_out_as_0() {
    let rx = VfmBurstRx { snr_db: f32::NAN, peak_dbfs: f32::NEG_INFINITY, ..VfmBurstRx::default() };
    let json = wire(ApiEvent::VfmBurstRx(rx));
    assert_eq!((json["snr_db"].as_f64(), json["peak_dbfs"].as_f64()), (Some(0.0), Some(0.0)));
    assert!(validator(VFM_EVENTS).is_valid(&json));

    let status = Status { cpu: f32::INFINITY, tap_cpu: f32::NAN, ..Status::default() };
    let json = wire(ApiEvent::Status(status));
    assert_eq!(json["cpu"], 0.0);
    assert!(validator(SHARED_ENVELOPE).is_valid(&json), "{json}");
}

#[test]
fn hello_names_the_server_and_the_mode_and_describes_the_rows() {
    let json = wire(ApiEvent::Hello(Hello::new("wide", 641)));
    assert_eq!(json["type"], "Hello");
    assert_eq!(json["server"], format!("vardefm {}", env!("CARGO_PKG_VERSION")));
    assert_eq!(json["mode"], "vardefm-wide");
    assert_eq!(
        json["waterfall"],
        json!({"stream": 1, "format": 1, "bins": 641, "bin_hz": 11.71875, "db_floor": -127.5, "db_step": 0.5,
               "rows_per_s": 10.0})
    );
}

#[test]
fn stream_header_has_the_documented_bytes() {
    let header =
        StreamHeader { stream: STREAM_WATERFALL, format: FORMAT_WATERFALL_U8, seq: 0x0102, t_low: 0x0304_0506 };
    assert_eq!(header.encode(), [1, 1, 0x02, 0x01, 0x06, 0x05, 0x04, 0x03]);
    let row = StreamHeader::waterfall(0x1_0002, WireTime(0x7_0000_0003));
    assert_eq!(row.encode(), [1, 1, 0x02, 0x00, 0x03, 0x00, 0x00, 0x00]);
}

#[test]
fn stream_header_splits_a_frame_and_refuses_a_short_one() {
    let header = StreamHeader { stream: 9, format: 8, seq: 65_535, t_low: u32::MAX };
    let mut frame = header.encode().to_vec();
    frame.extend([10, 20]);
    assert_eq!(StreamHeader::decode(&frame), Some((header, &[10, 20][..])));
    assert_eq!(StreamHeader::decode(&frame[..8]), Some((header, &[][..])));
    assert_eq!(StreamHeader::decode(&frame[..7]), None);
}
