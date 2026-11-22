//! The messages of the API, wire version 1.
//!
//! The shared messages ([`Hello`], [`Status`], `Gap`, `Warning`), the [`Envelope`], the
//! [`StatusSnapshot`] and the [`StreamHeader`] are the API's shared messages, described by
//! `docs/api/envelope.schema.json` and `docs/api/status.schema.json`.
//!
//! vardeFM's own messages are named `Vfm...` and described by `docs/api/vardefm-events.schema.json`.
//! A client that does not know them skips them.
//!
//! Every float is written finite: NaN and infinity go out as 0, because JSON has no such numbers
//! and serde_json would write `null`, which the schemas refuse.

use crate::waterfall::{self, Waterfall};
use serde::{Deserialize, Serialize, Serializer};

/// Version of the messages. It goes up only on a breaking change; a new field
/// or message does not change it.
pub const API_VERSION: u16 = 1;

/// Stream id of waterfall rows in a [`StreamHeader`].
pub const STREAM_WATERFALL: u8 = 1;

/// Payload format of a waterfall row in a [`StreamHeader`]: one `u8` per bin, as a
/// [`WaterfallFormat`] describes.
pub const FORMAT_WATERFALL_U8: u8 = 1;

/// Server name and version, as sent in [`Hello`].
pub const SERVER: &str = concat!("vardefm ", env!("CARGO_PKG_VERSION"));

/// `x`, or 0 if it is NaN or infinite.
pub fn finite(x: f32) -> f32 {
    if x.is_finite() {
        x
    } else {
        0.0
    }
}

fn write_finite<S: Serializer>(x: &f32, s: S) -> Result<S::Ok, S::Error> {
    s.serialize_f32(finite(*x))
}

/// The API's name for a vardeFM mode (`narrow` or `wide`), as sent in [`Hello`] and [`Status`].
pub fn mode_name(mode: &str) -> String {
    format!("vardefm-{mode}")
}

/// A time on the 48 kHz receive clock, in samples since the radio started.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct WireTime(pub u64);

/// What a client needs to read the binary waterfall frames; sent in [`Hello`].
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct WaterfallFormat {
    /// Stream id the frames carry, see [`STREAM_WATERFALL`].
    pub stream: u8,
    /// Payload format the frames carry, see [`FORMAT_WATERFALL_U8`].
    pub format: u8,
    /// Bins per row; bin `k` is at `k * bin_hz`.
    pub bins: u16,
    /// Bin spacing, in Hz.
    #[serde(serialize_with = "write_finite")]
    pub bin_hz: f32,
    /// Level of value 0, in dBFS; a value `v` is `db_floor + v * db_step`.
    #[serde(serialize_with = "write_finite")]
    pub db_floor: f32,
    /// Level per value step, in dB.
    #[serde(serialize_with = "write_finite")]
    pub db_step: f32,
    /// Rows per second the server makes.
    #[serde(serialize_with = "write_finite")]
    pub rows_per_s: f32,
}

impl WaterfallFormat {
    /// The format of the rows `waterfall` makes.
    pub fn of(waterfall: &Waterfall) -> Self {
        Self {
            stream: STREAM_WATERFALL,
            format: FORMAT_WATERFALL_U8,
            // At most `waterfall::MAX_BINS`, which fits.
            bins: waterfall.bins() as u16,
            bin_hz: waterfall::BIN_HZ,
            db_floor: waterfall::DB_FLOOR,
            db_step: waterfall::DB_STEP,
            rows_per_s: waterfall::ROWS_PER_S,
        }
    }
}

/// First message on a stream.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Hello {
    /// Server name and version.
    pub server: String,
    /// Modem mode the server runs: `vardefm-narrow` or `vardefm-wide`.
    pub mode: String,
    /// Format of the binary waterfall frames.
    pub waterfall: WaterfallFormat,
}

impl Hello {
    /// The `Hello` of this server for `mode` (`narrow` or `wide`), describing the rows of
    /// `waterfall`.
    pub fn new(mode: &str, waterfall: &Waterfall) -> Self {
        Self { server: SERVER.to_owned(), mode: mode_name(mode), waterfall: WaterfallFormat::of(waterfall) }
    }
}

/// Monitor counters. Those that count since the start only ever grow.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Status {
    /// Stream time reached.
    pub t: WireTime,
    /// Modem mode the server runs.
    pub mode: String,
    /// Time in the modem, as a percentage of the audio's duration.
    #[serde(serialize_with = "write_finite")]
    pub cpu: f32,
    /// Time making waterfall rows, as a percentage of the audio's duration.
    #[serde(serialize_with = "write_finite")]
    pub tap_cpu: f32,
    /// Time in the resampler, as a percentage of the audio's duration; 0 for a 48 kHz device.
    #[serde(serialize_with = "write_finite")]
    pub resample_cpu: f32,
    /// Longest single step of the modem, in ms.
    pub longest_push_ms: u32,
    /// Audio chunks dropped because the modem fell behind.
    pub overruns: u64,
    /// Samples in those chunks.
    pub dropped_frames: u64,
    /// Overruns the audio backend reported.
    pub device_xruns: u64,
    /// Frames those overruns lost, estimated.
    pub xrun_frames: u64,
    /// Other audio backend errors.
    pub device_errors: u64,
    /// Discontinuities in the audio.
    pub gaps: u64,
    /// Events dropped between the radio loop and the server.
    pub events_dropped: u64,
    /// Waterfall rows dropped between the radio loop and the server.
    pub rows_dropped: u64,
    /// Waterfall rows dropped because a client was slow, over all clients.
    pub client_rows_dropped: u64,
    /// Scheduling of the audio backend's capture thread.
    #[serde(default)]
    pub capture_priority: CallbackPriority,
    /// The ARQ session.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub link: Option<LinkStatus>,
}

/// Where an ARQ session stands.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LinkState {
    /// Not listening and not connected.
    #[default]
    Idle,
    /// Answers connect requests.
    Listening,
    /// Calling a peer.
    Connecting,
    /// Connected to a peer.
    Connected,
    /// Ending the connection.
    Disconnecting,
}

/// A snapshot of an ARQ session.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct LinkStatus {
    /// Where the session stands.
    pub state: LinkState,
    /// The peer's call sign, once known.
    pub peer: Option<String>,
    /// Level of our transmissions, while connected.
    pub level: Option<u8>,
    /// Bytes from the host not yet acknowledged by the peer.
    pub tx_queued: u64,
    /// Bytes from the peer not yet read by the host. vardeFM hands received data to the host
    /// as it arrives, so this is 0.
    pub rx_queued: u64,
}

/// Scheduling of an audio backend's callback thread.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CallbackPriority {
    /// Not seen yet, or not measured. vardeFM does not measure it.
    #[default]
    Unknown,
    /// Real-time.
    Realtime,
    /// Promotion was refused.
    Denied,
    /// A normal time-sharing thread.
    Normal,
}

/// A burst the receiver decoded the header of (`VfmBurstRx`).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct VfmBurstRx {
    /// First sample of the burst.
    pub t: WireTime,
    /// End of the burst.
    pub t_end: WireTime,
    /// Header kind, such as `Data` or `ConnectReq`.
    pub kind: String,
    /// Speed level of the payload; absent if the burst has no codewords.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub level: Option<u8>,
    /// Codewords in the burst.
    pub ncw: u8,
    /// Codewords that decoded and passed their CRC.
    pub cw_ok: u8,
    /// The receiver's in-band SNR per symbol (Es/N0), in dB.
    #[serde(serialize_with = "write_finite")]
    pub snr_db: f32,
    /// Peak audio level over the burst, in dBFS.
    #[serde(serialize_with = "write_finite")]
    pub peak_dbfs: f32,
}

/// A burst keyed up (`VfmBurstTx`).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct VfmBurstTx {
    /// Key-up time.
    pub t: WireTime,
    /// Header kind, such as `Data` or `ConnectReq`.
    pub kind: String,
    /// Speed level of the payload; absent if the burst has no codewords.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub level: Option<u8>,
    /// Codewords in the burst.
    pub ncw: u8,
    /// Payload bytes, resent bytes included.
    pub bytes: u32,
    /// Payload bytes sent before.
    pub resent_bytes: u32,
    /// Consecutive retry number, 0 for a first attempt.
    pub retry: u32,
    /// Airtime, in seconds.
    #[serde(serialize_with = "write_finite")]
    pub air_s: f32,
    /// A digipeater repeat of another station's burst.
    pub repeat: bool,
}

/// A message of the server; JSON text on a stream.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum ApiEvent {
    /// First message on a stream.
    Hello(Hello),
    /// Counters, about once per second.
    Status(Status),
    /// The audio had a discontinuity.
    Gap {
        /// First sample after it.
        t: WireTime,
    },
    /// Something a person should know.
    Warning {
        /// Stable identifier, such as `rx_level`.
        code: String,
        /// Description, for people.
        text: String,
    },
    /// A burst received.
    VfmBurstRx(VfmBurstRx),
    /// A burst sent.
    VfmBurstTx(VfmBurstTx),
    /// The channel turned busy or free.
    VfmBusy {
        /// When the receiver noticed.
        t: WireTime,
        /// Busy now.
        busy: bool,
    },
}

impl ApiEvent {
    /// True for vardeFM's own messages, which other servers do not send.
    pub fn is_vfm(&self) -> bool {
        matches!(self, Self::VfmBurstRx(_) | Self::VfmBurstTx(_) | Self::VfmBusy { .. })
    }
}

/// The wire form of an [`ApiEvent`]: `{"v":1,"type":...}`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Envelope {
    /// [`API_VERSION`] of the sender.
    pub v: u16,
    /// The message.
    #[serde(flatten)]
    pub event: ApiEvent,
}

impl Envelope {
    /// Wraps `event` at the current [`API_VERSION`].
    pub const fn new(event: ApiEvent) -> Self {
        Self { v: API_VERSION, event }
    }
}

/// Answer to `GET /v1/status`.
///
/// The status schema types every entry of `recent`, so a `Vfm...` message there would make a
/// client that knows only the shared messages refuse the whole answer. vardeFM's own messages go
/// in `vfm_recent` instead, a field such a client does not read.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct StatusSnapshot {
    /// [`API_VERSION`] of the sender.
    pub v: u16,
    /// Latest counters.
    pub status: Status,
    /// Latest gap and warning events, oldest first.
    pub recent: Vec<ApiEvent>,
    /// Latest `Vfm...` events, oldest first.
    #[serde(default)]
    pub vfm_recent: Vec<ApiEvent>,
}

impl StatusSnapshot {
    /// The answer for `status` and the recent `events`, oldest first. Each event goes in the list
    /// it belongs in: `Vfm...` events in `vfm_recent`, gaps and warnings in `recent`. `Hello` and
    /// `Status` are not events of the past and are left out.
    pub fn new(status: Status, events: impl IntoIterator<Item = ApiEvent>) -> Self {
        let (mut recent, mut vfm_recent) = (Vec::new(), Vec::new());
        for event in events {
            match event {
                ApiEvent::Hello(_) | ApiEvent::Status(_) => {}
                e if e.is_vfm() => vfm_recent.push(e),
                e => recent.push(e),
            }
        }
        Self { v: API_VERSION, status, recent, vfm_recent }
    }
}

/// Header of a binary stream frame: 8 bytes, then the payload.
///
/// | Offset | Type | Field |
/// |---|---|---|
/// | 0 | `u8` | `stream` |
/// | 1 | `u8` | `format` |
/// | 2 | `u16` LE | `seq` |
/// | 4 | `u32` LE | `t_low` |
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StreamHeader {
    /// Stream id, such as [`STREAM_WATERFALL`].
    pub stream: u8,
    /// Payload format, such as [`FORMAT_WATERFALL_U8`].
    pub format: u8,
    /// Sequence number, wrapping; a jump shows frames lost.
    pub seq: u16,
    /// Low 32 bits of the [`WireTime`] of the frame's first sample (wraps after 24.85 h);
    /// [`Status::t`] gives the rest.
    pub t_low: u32,
}

impl StreamHeader {
    /// Length of the encoded header, in bytes.
    pub const LEN: usize = 8;

    /// The header of a waterfall row.
    pub fn waterfall(seq: u64, t: WireTime) -> Self {
        Self { stream: STREAM_WATERFALL, format: FORMAT_WATERFALL_U8, seq: seq as u16, t_low: t.0 as u32 }
    }

    /// The header's bytes.
    pub fn encode(&self) -> [u8; Self::LEN] {
        let [s0, s1] = self.seq.to_le_bytes();
        let [t0, t1, t2, t3] = self.t_low.to_le_bytes();
        [self.stream, self.format, s0, s1, t0, t1, t2, t3]
    }

    /// Splits a binary frame into its header and payload; `None` if it is too short.
    pub fn decode(frame: &[u8]) -> Option<(Self, &[u8])> {
        let (&[stream, format, s0, s1, t0, t1, t2, t3], payload) = frame.split_first_chunk()?;
        let header =
            Self { stream, format, seq: u16::from_le_bytes([s0, s1]), t_low: u32::from_le_bytes([t0, t1, t2, t3]) };
        Some((header, payload))
    }
}

#[cfg(test)]
mod tests;
