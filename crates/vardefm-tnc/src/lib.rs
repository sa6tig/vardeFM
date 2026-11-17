//! VardeFM TNC: everything between the modem and a host application.
//!
//! - [`host`]: the VARA-compatible command protocol.
//! - [`server`]: TCP command and data ports.
//! - [`station`]: one modem (link + modulator + receiver) with its host notifications.
//! - [`simpair`]: two stations connected by a simulated FM radio link.
//! - [`radio`]: one station on a real radio (sound card + PTT).
//! - [`level`]: receive level meter and volume advice.
//! - [`monitor`]: the radio's side of the read-only monitor API.

pub mod host;
pub mod level;
pub mod monitor;
pub mod radio;
pub mod server;
pub mod simpair;
pub mod station;

pub const VERSION: &str = concat!("VardeFM ", env!("CARGO_PKG_VERSION"));
