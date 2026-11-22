//! VardeFM's read-only monitor API: the parts that need no network.
//!
//! The wire format is version 1 of a monitor API shared with other modems: its shared
//! messages are described by `docs/api/envelope.schema.json` and `status.schema.json`, so a
//! client for that API can show vardeFM's waterfall and status unchanged. vardeFM adds
//! messages of its own, all named `Vfm...` (`docs/api/vardefm-events.schema.json`), which a
//! client that does not know them skips.
//!
//! - [`loopback`]: the only addresses the API may listen on.
//! - [`wire`]: the messages, their JSON form and the binary row header.
//! - [`waterfall`]: rows of the received spectrum, ten a second.

pub mod loopback;
pub mod waterfall;
pub mod wire;

pub use loopback::{LoopbackAddr, NotLoopback};
