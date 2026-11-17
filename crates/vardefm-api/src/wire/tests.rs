use super::*;
use crate::waterfall::{bins_for_mode, HOP};
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
        ApiEvent::Hello(hello("narrow")),
        ApiEvent::Hello(hello("wide")),
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
            capture_priority: CallbackPriority::Realtime,
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

fn hello(mode: &str) -> Hello {
    Hello::new(mode, &Waterfall::new(bins_for_mode(mode).unwrap().into()))
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
    let sent = StatusSnapshot::new(Status { mode: mode_name("narrow"), ..Status::default() }, every_event());
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
    let json = wire(ApiEvent::Hello(hello("wide")));
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

/// Shared and Vfm messages interleaved, as the server sees them.
fn every_event() -> Vec<ApiEvent> {
    let (shared, vfm) = (shared_events(), vfm_events());
    let mut all = Vec::new();
    for i in 0..shared.len().max(vfm.len()) {
        all.extend(shared.get(i).cloned());
        all.extend(vfm.get(i).cloned());
    }
    all
}

#[test]
fn the_status_answer_sorts_events_into_their_lists_in_order() {
    let snapshot = StatusSnapshot::new(Status::default(), every_event());
    assert_eq!(snapshot.v, API_VERSION);
    let shared_past: Vec<_> =
        shared_events().into_iter().filter(|e| !matches!(e, ApiEvent::Hello(_) | ApiEvent::Status(_))).collect();
    assert_eq!(snapshot.recent, shared_past);
    assert_eq!(snapshot.vfm_recent, vfm_events());
}

#[test]
fn hello_announces_as_many_bins_as_a_row_has() {
    for mode in ["narrow", "wide"] {
        let mut waterfall = Waterfall::new(bins_for_mode(mode).unwrap().into());
        let hello = Hello::new(mode, &waterfall);
        let mut rows = Vec::new();
        waterfall.push(0, &[0.0; HOP], &mut |r| rows.push(r));
        assert_eq!(usize::from(hello.waterfall.bins), rows[0].bins.len(), "{mode}");
    }
}

/// Paths of the keys in `value` that `schema` does not name, so a misspelt or renamed key cannot
/// pass unnoticed: none of the schemas forbid unknown keys, so validation alone would not see it.
fn unknown_keys(value: &Value, schema: &str) -> Vec<String> {
    let root: Value = serde_json::from_str(schema).unwrap();
    let mut out = Vec::new();
    walk(value, &root, &root, "", &mut out);
    out
}

/// The `properties` a schema node names itself or through its `$ref`.
fn properties(node: &Value, root: &Value) -> serde_json::Map<String, Value> {
    let mut props = node.get("properties").and_then(Value::as_object).cloned().unwrap_or_default();
    if let Some(target) = node.get("$ref").and_then(Value::as_str) {
        let name = target.strip_prefix("#/$defs/").unwrap();
        props.extend(properties(&root["$defs"][name], root));
    }
    props
}

fn walk(value: &Value, node: &Value, root: &Value, path: &str, out: &mut Vec<String>) {
    let node = match node.get("$ref").and_then(Value::as_str) {
        Some(target) if node.get("properties").is_none() => &root["$defs"][target.strip_prefix("#/$defs/").unwrap()],
        _ => node,
    };
    match value {
        Value::Object(map) => {
            let mut props = properties(node, root);
            let branches = node.get("oneOf").or(node.get("anyOf")).and_then(Value::as_array);
            // Of the branches, the one that names the most of the keys is the one the value is.
            if let Some(best) = branches.and_then(|b| {
                b.iter().map(|b| properties(b, root)).max_by_key(|p| map.keys().filter(|k| p.contains_key(*k)).count())
            }) {
                props.extend(best);
            }
            for (key, child) in map {
                match props.get(key) {
                    Some(sub) => walk(child, sub, root, &format!("{path}/{key}"), out),
                    None => out.push(format!("{path}/{key}")),
                }
            }
        }
        Value::Array(items) => {
            for (i, item) in items.iter().enumerate() {
                walk(item, node.get("items").unwrap_or(&Value::Null), root, &format!("{path}/{i}"), out);
            }
        }
        _ => {}
    }
}

/// The schema of one of the Vfm schema's definitions, such as `VfmConstellationFormat`.
fn vfm_def(name: &str) -> String {
    let mut root: Value = serde_json::from_str(VFM_EVENTS).unwrap();
    let root = root.as_object_mut().unwrap();
    root.remove("oneOf");
    root.insert("$ref".to_owned(), json!(format!("#/$defs/{name}")));
    serde_json::to_string(&root).unwrap()
}

/// `Hello` without vardeFM's own key, and that key.
fn split_hello(mut json: Value) -> (Value, Value) {
    let own = json.as_object_mut().unwrap().remove("vfm_constellation").expect("vfm_constellation");
    (json, own)
}

#[test]
fn hello_carries_the_constellation_format_the_vfm_schema_describes() {
    let format = vfm_def("VfmConstellationFormat");
    for mode in ["narrow", "wide"] {
        let (shared, own) = split_hello(wire(ApiEvent::Hello(hello(mode))));
        assert_eq!(own, json!({"stream": 2, "format": 1, "scale": 48, "max_points": 2048}));
        assert!(validator(&format).is_valid(&own), "{own}");
        assert_eq!(unknown_keys(&own, &format), Vec::<String>::new());
        assert!(validator(SHARED_ENVELOPE).is_valid(&shared));
    }
    assert!(!validator(&format).is_valid(&json!({"stream": 2, "format": 1, "scale": 48})));
    // An older Hello, without the key, still reads.
    let old = json!({"v": 1, "type": "Hello", "server": "vardefm 0.3.0", "mode": "vardefm-narrow",
                     "waterfall": wire(ApiEvent::Hello(hello("narrow")))["waterfall"]});
    let back: Envelope = serde_json::from_value(old).unwrap();
    assert!(matches!(back.event, ApiEvent::Hello(Hello { vfm_constellation: None, .. })));
}

#[test]
fn every_key_sent_is_one_the_schemas_name() {
    for event in shared_events() {
        let json = match event {
            // vardeFM's own key is checked against the Vfm schema above.
            ApiEvent::Hello(_) => split_hello(wire(event)).0,
            _ => wire(event),
        };
        assert_eq!(unknown_keys(&json, SHARED_ENVELOPE), Vec::<String>::new(), "{json}");
    }
    for event in vfm_events() {
        let json = wire(event);
        assert_eq!(unknown_keys(&json, VFM_EVENTS), Vec::<String>::new(), "{json}");
    }
    let snapshot = serde_json::to_value(StatusSnapshot::new(shared_status_with_link(), shared_events())).unwrap();
    assert_eq!(unknown_keys(&snapshot, SHARED_STATUS), Vec::<String>::new(), "{snapshot}");

    // The check sees a misspelt key, here and nested.
    let mut bad = wire(ApiEvent::Status(shared_status_with_link()));
    bad["capture_prio"] = json!("unknown");
    bad["link"]["peer_call"] = json!("LA2BBB-1");
    assert_eq!(unknown_keys(&bad, SHARED_ENVELOPE), ["/capture_prio", "/link/peer_call"]);
    assert!(validator(SHARED_ENVELOPE).is_valid(&bad), "validation alone lets it through");
}

fn shared_status_with_link() -> Status {
    shared_events()
        .into_iter()
        .find_map(|e| match e {
            ApiEvent::Status(s) if s.link.is_some() => Some(s),
            _ => None,
        })
        .unwrap()
}
