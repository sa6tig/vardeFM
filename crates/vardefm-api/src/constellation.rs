//! Constellation frames: the equalized payload symbols of a received burst, one binary frame per
//! burst, on vardeFM's own stream [`STREAM_VFM_CONSTELLATION`].
//!
//! After the 8-byte [`StreamHeader`] (whose `t_low` is the burst's `t`, as in its `VfmBurstRx`)
//! comes an 8-byte payload header and then 3 bytes per point:
//!
//! | Offset | Type | Field |
//! |---|---|---|
//! | 0 | `u8` | speed level |
//! | 1 | `u8` | codewords in the burst |
//! | 2 | `u8` | scale: the value of 1.0 |
//! | 3 | `u8` | bits per symbol of the level's modulation (1, 2, 4 or 6) |
//! | 4 | `u16` LE | points |
//! | 6 | `u8` | data carriers per OFDM symbol |
//! | 7 | `u8` | 0 |
//! | 8 + 3n | `i8`, `i8`, `u8` | point `n`: in-phase, quadrature, carrier |
//!
//! The points are whole OFDM symbols, in order, each carrier by carrier. A long burst is
//! thinned to at most [`MAX_POINTS`] by keeping every k-th OFDM symbol, so every carrier stays
//! in view. The carrier is the data carrier's index among all the mode's carriers, pilots
//! included, counted from the lowest. A carrier whose index does not fit a `u8`, or any past the
//! 255th, is left out (no mode has that many), so the counts always fit their fields.

use crate::wire::{finite, StreamHeader, WireTime};
use rustfft::num_complex::Complex32;
use serde::{Deserialize, Serialize};

/// Stream id of constellation frames in a [`StreamHeader`].
pub const STREAM_VFM_CONSTELLATION: u8 = 2;

/// Payload format of a constellation frame, as described in this module.
pub const FORMAT_VFM_CONSTELLATION: u8 = 1;

/// Most points in a frame.
pub const MAX_POINTS: usize = 2048;

/// The value of 1.0. A constellation has unit mean energy, so its outer points of 64QAM sit at
/// 1.08, and an `i8` reaches 2.65: room for noisy points.
pub const SCALE: u8 = 48;

/// Length of the payload header.
pub const PAYLOAD_HEADER_LEN: usize = 8;

/// What a client needs to read constellation frames; sent in `Hello` as `vfm_constellation`.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct VfmConstellationFormat {
    /// Stream id the frames carry, see [`STREAM_VFM_CONSTELLATION`].
    pub stream: u8,
    /// Payload format the frames carry, see [`FORMAT_VFM_CONSTELLATION`].
    pub format: u8,
    /// The value of 1.0 in a point.
    pub scale: u8,
    /// Most points in a frame.
    pub max_points: u16,
}

impl VfmConstellationFormat {
    /// The format of the frames this crate makes.
    pub fn current() -> Self {
        Self {
            stream: STREAM_VFM_CONSTELLATION,
            format: FORMAT_VFM_CONSTELLATION,
            scale: SCALE,
            max_points: MAX_POINTS as u16,
        }
    }
}

/// A received burst's payload symbols.
pub struct Constellation<'a> {
    /// The burst's `t`, as in its `VfmBurstRx`.
    pub t: WireTime,
    pub level: u8,
    pub ncw: u8,
    /// Bits per symbol of the level's modulation.
    pub bits_per_symbol: u8,
    /// For each data carrier, its index among all carriers.
    pub carriers: &'a [usize],
    /// Equalized symbols, one Vec per OFDM symbol, one entry per data carrier.
    pub symbols: &'a [Vec<Complex32>],
}

impl Constellation<'_> {
    /// The binary frame, with sequence number `seq`.
    pub fn encode(&self, seq: u64) -> Vec<u8> {
        // (position in a symbol, carrier index): at most 255, so a symbol is at most 255 points.
        let columns: Vec<(usize, u8)> = self
            .carriers
            .iter()
            .enumerate()
            .filter_map(|(j, &c)| u8::try_from(c).ok().map(|c| (j, c)))
            .take(usize::from(u8::MAX))
            .collect();
        let symbols_kept = MAX_POINTS / columns.len().max(1);
        let every = self.symbols.len().div_ceil(symbols_kept).max(1);
        let mut points = Vec::new();
        for symbol in self.symbols.iter().step_by(every) {
            for &(j, carrier) in &columns {
                if let Some(y) = symbol.get(j) {
                    points.extend_from_slice(&[axis(y.re) as u8, axis(y.im) as u8, carrier]);
                }
            }
        }
        // At most MAX_POINTS: at most `symbols_kept` symbols of `columns.len()` points.
        let [p0, p1] = ((points.len() / 3) as u16).to_le_bytes();

        let mut frame = Vec::with_capacity(StreamHeader::LEN + PAYLOAD_HEADER_LEN + points.len());
        frame.extend_from_slice(&StreamHeader::vfm_constellation(seq, self.t).encode());
        let carriers = columns.len() as u8;
        frame.extend_from_slice(&[self.level, self.ncw, SCALE, self.bits_per_symbol, p0, p1, carriers, 0]);
        frame.extend_from_slice(&points);
        frame
    }
}

/// One axis of a point: scaled, rounded and kept within an `i8`; NaN goes out as 0.
fn axis(x: f32) -> i8 {
    (finite(x) * f32::from(SCALE)).round().clamp(-128.0, 127.0) as i8
}

/// A point of a frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Point {
    pub i: i8,
    pub q: i8,
    pub carrier: u8,
}

/// The payload of a constellation frame, read back.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ConstellationFrame {
    pub level: u8,
    pub ncw: u8,
    pub scale: u8,
    pub bits_per_symbol: u8,
    pub data_carriers: u8,
    pub points: Vec<Point>,
}

impl ConstellationFrame {
    /// Reads the payload of a frame (after its [`StreamHeader`]); `None` if it is short or its
    /// point count does not match its length.
    pub fn decode(payload: &[u8]) -> Option<Self> {
        let (&[level, ncw, scale, bits_per_symbol, p0, p1, data_carriers, _], rest) = payload.split_first_chunk()?;
        let n = usize::from(u16::from_le_bytes([p0, p1]));
        if rest.len() != 3 * n {
            return None;
        }
        let points =
            rest.as_chunks::<3>().0.iter().map(|&[i, q, carrier]| Point { i: i as i8, q: q as i8, carrier }).collect();
        Some(Self { level, ncw, scale, bits_per_symbol, data_carriers, points })
    }
}

#[cfg(test)]
mod tests;
