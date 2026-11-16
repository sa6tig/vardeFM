use vardefm_fec::crc::crc16;

/// Burst type.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Kind {
    /// Connection request; payload: one level-0 codeword with the call signs.
    ConnectReq = 0,
    /// Connection accepted; payload: one level-0 codeword with the call signs.
    ConnectAck = 1,
    /// Data and/or acknowledgements.
    #[default]
    Data = 2,
    /// Close the session (sent once all our data is acknowledged).
    Disc = 3,
    /// Session closed.
    DiscAck = 4,
}

impl Kind {
    fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::ConnectReq,
            1 => Self::ConnectAck,
            2 => Self::Data,
            3 => Self::Disc,
            4 => Self::DiscAck,
            _ => return None,
        })
    }
}

/// Burst header: 80 bits of fields, MSB first, followed by a CRC-16.
///
/// | bits | field |
/// |---|---|
/// | 3 | kind |
/// | 8 | session id |
/// | 3 | payload speed level |
/// | 5 | ncw: payload codewords (0-24) |
/// | 3 | req_level: level the sender asks the peer to use next |
/// | 3 | burst_id: sender's burst counter (mod 8) |
/// | 3 | ack_burst: burst_id of the last peer burst this one acknowledges |
/// | 24 | ack_offset: next byte offset expected from the peer (mod 2^24) |
/// | 24 | bitmap: bit i (LSB = codeword 0) set if codeword i of burst ack_burst decoded |
/// | 4 | flags: bit 0 = POLL (peer must answer even with nothing to send), bit 1 = UP (called → caller), bits 2-3 = route field |
///
/// The route field is 0 on a direct session (all that versions before digipeating
/// send). On a routed session it is 1 as an end station sends it, plus one for each
/// digipeater repeat, so every burst of a routed session is marked as such.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Header {
    pub kind: Kind,
    pub session: u8,
    pub level: u8,
    pub ncw: u8,
    pub req_level: u8,
    pub burst_id: u8,
    pub ack_burst: u8,
    pub ack_offset: u32,
    pub bitmap: u32,
    pub flags: u8,
}

pub const FLAG_POLL: u8 = 1;
/// Direction on a routed (digipeated) session: set on bursts from the called station.
pub const FLAG_UP: u8 = 2;
/// Most digipeaters on one path: the 2-bit route field holds 1 + repeats, and 0 means direct.
pub const MAX_DIGIS: usize = 2;
const ROUTE_SHIFT: u8 = 2;
/// Route field 1: a burst of a routed session as an end station sends it.
pub const FLAG_ROUTED: u8 = 1 << ROUTE_SHIFT;

const FIELDS: [usize; 10] = [3, 8, 3, 5, 3, 3, 3, 24, 24, 4];

impl Header {
    fn route_field(&self) -> u8 {
        (self.flags >> ROUTE_SHIFT) & 3
    }

    /// True if the burst belongs to a routed (digipeated) session.
    pub fn routed(&self) -> bool {
        self.route_field() != 0
    }

    /// How many times a digipeater has repeated this burst (0 on a direct one).
    pub fn hop(&self) -> u8 {
        self.route_field().saturating_sub(1)
    }

    /// Mark the burst routed, repeated `hop` times (at most [`MAX_DIGIS`]).
    pub fn set_hop(&mut self, hop: u8) {
        debug_assert!(hop as usize <= MAX_DIGIS);
        let field = hop.min(MAX_DIGIS as u8) + 1;
        self.flags = (self.flags & !(3 << ROUTE_SHIFT)) | (field << ROUTE_SHIFT);
    }

    /// True if the burst comes from the called station (routed sessions only).
    pub fn up(&self) -> bool {
        self.flags & FLAG_UP != 0
    }

    fn values(&self) -> [u64; 10] {
        [
            self.kind as u64,
            self.session as u64,
            self.level as u64,
            self.ncw as u64,
            self.req_level as u64,
            self.burst_id as u64,
            self.ack_burst as u64,
            self.ack_offset as u64,
            self.bitmap as u64,
            self.flags as u64,
        ]
    }

    /// 12 bytes: 10 bytes of fields + CRC-16 (big-endian).
    pub fn pack(&self) -> [u8; 12] {
        let mut v: u128 = 0;
        for (val, bits) in self.values().into_iter().zip(FIELDS) {
            v = (v << bits) | (val as u128 & ((1u128 << bits) - 1));
        }
        let mut out = [0u8; 12];
        out[..10].copy_from_slice(&v.to_be_bytes()[6..16]);
        let c = crc16(&out[..10]);
        out[10..].copy_from_slice(&c.to_be_bytes());
        out
    }

    pub fn unpack(bytes: &[u8]) -> Option<Self> {
        if bytes.len() != 12 || crc16(&bytes[..10]).to_be_bytes() != bytes[10..12] {
            return None;
        }
        let mut buf = [0u8; 16];
        buf[6..].copy_from_slice(&bytes[..10]);
        let mut v = u128::from_be_bytes(buf);
        let mut vals = [0u64; 10];
        for i in (0..10).rev() {
            vals[i] = (v & ((1u128 << FIELDS[i]) - 1)) as u64;
            v >>= FIELDS[i];
        }
        let h = Self {
            kind: Kind::from_u8(vals[0] as u8)?,
            session: vals[1] as u8,
            level: vals[2] as u8,
            ncw: vals[3] as u8,
            req_level: vals[4] as u8,
            burst_id: vals[5] as u8,
            ack_burst: vals[6] as u8,
            ack_offset: vals[7] as u32,
            bitmap: vals[8] as u32,
            flags: vals[9] as u8,
        };
        let max_level = crate::LEVELS.len() as u8;
        (h.level < max_level && h.req_level < max_level && h.ncw as usize <= crate::MAX_NCW).then_some(h)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pack_roundtrip() {
        let h = Header {
            kind: Kind::Data,
            session: 0xa7,
            level: 5,
            ncw: 17,
            req_level: 3,
            burst_id: 6,
            ack_burst: 1,
            ack_offset: 0x00ab_cdef,
            bitmap: 0x0012_3456,
            flags: FLAG_POLL,
        };
        let p = h.pack();
        assert_eq!(Header::unpack(&p), Some(h));
        let mut bad = p;
        bad[3] ^= 1;
        assert_eq!(Header::unpack(&bad), None);
    }

    #[test]
    fn hop_and_direction_flags() {
        // Route field 0: a direct burst, as every released version sends it.
        let direct = Header::unpack(&Header { flags: FLAG_POLL, ..Header::default() }.pack()).unwrap();
        assert!(!direct.routed());
        assert_eq!(direct.hop(), 0);
        assert!(!direct.up());

        let mut h = Header { flags: FLAG_POLL | FLAG_UP, ..Header::default() };
        for hop in 0..=MAX_DIGIS as u8 {
            h.set_hop(hop);
            let back = Header::unpack(&h.pack()).unwrap();
            assert!(back.routed());
            assert_eq!(back.hop(), hop);
            assert_eq!((back.flags >> ROUTE_SHIFT) & 3, hop + 1);
            assert!(back.up());
            assert_eq!(back.flags & FLAG_POLL, FLAG_POLL);
        }
        let mut sent = Header::default();
        sent.set_hop(0);
        assert_eq!(sent.flags, FLAG_ROUTED);
    }
}
