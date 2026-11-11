//! VardeFM bursts.
//!
//! A burst is one transmission: `[leader][preamble A][preamble B][header][payload]`.
//! - The header is 96 bits (80 bits of fields + CRC-16), LDPC-coded to 400 bits and
//!   sent as BPSK. That is 10 OFDM symbols in narrow mode and 8 in wide mode.
//! - The payload is `ncw` codewords of 2400 bits at the speed level named in the
//!   header. Each codeword carries `k/8 - 4` data bytes plus a CRC-32.

mod busy;
mod header;
mod receiver;

pub use header::{Header, Kind, FLAG_POLL};
pub use receiver::{AudioLevel, Receiver, RxBurst};
pub use vardefm_dsp::OfdmParams;

use vardefm_dsp::{Complex32, Frame, Modulation, Ofdm};
use vardefm_fec::{bytes_to_bits, crc, permutation, scramble_bits, Ldpc};

/// Bits per payload codeword.
pub const CW_BITS: usize = 2400;
/// Header bits before coding (80 bits of fields + CRC-16).
pub const HDR_INFO_BITS: usize = 96;
/// Header bits after coding.
pub const HDR_CODE_BITS: usize = 400;
/// Most codewords in one burst (the ack bitmap has 24 bits).
pub const MAX_NCW: usize = 24;
/// Payload airtime limit per burst. Keeps clock drift inside the cyclic-prefix slack.
pub const MAX_PAYLOAD_SECONDS: f32 = 4.0;
const DECODE_ITERS: usize = 50;

const SEED_CODE: u32 = 0x1d9c_0000;
const SEED_HDR_CODE: u32 = 0x1d9c_1000;
const SEED_SCRAMBLE: u32 = 0x5c4a_0000;
const SEED_HDR_SCRAMBLE: u32 = 0x5c4a_ffff;
const SEED_INTERLEAVE: u32 = 0x17e4_0001;
const SEED_HDR_INTERLEAVE: u32 = 0x17e4_0002;

/// One speed level: constellation and LDPC information length (codeword = 2400 bits).
#[derive(Clone, Copy, Debug)]
pub struct Level {
    pub modulation: Modulation,
    pub k: usize,
    pub name: &'static str,
}

pub const LEVELS: [Level; 7] = [
    Level { modulation: Modulation::Bpsk, k: 1200, name: "BPSK 1/2" },
    Level { modulation: Modulation::Qpsk, k: 1200, name: "QPSK 1/2" },
    Level { modulation: Modulation::Qpsk, k: 1800, name: "QPSK 3/4" },
    Level { modulation: Modulation::Qam16, k: 1200, name: "16QAM 1/2" },
    Level { modulation: Modulation::Qam16, k: 1800, name: "16QAM 3/4" },
    Level { modulation: Modulation::Qam64, k: 1600, name: "64QAM 2/3" },
    Level { modulation: Modulation::Qam64, k: 2000, name: "64QAM 5/6" },
];

/// A burst to transmit.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Burst {
    pub header: Header,
    /// `header.ncw` codewords, each exactly `payload_bytes(header.level)` long.
    pub payload: Vec<Vec<u8>>,
}

/// Modulator/coder for one mode. Shared (read-only) by the transmitter and receiver.
pub struct Modem {
    pub ofdm: Ofdm,
    /// Leader length in seconds (must exceed the radio's key-up time).
    pub txdelay: f32,
    /// Threads used to decode a burst's codewords (default: one per CPU core).
    pub decode_threads: usize,
    hdr_code: Ldpc,
    codes: Vec<Ldpc>,
    hdr_perm: Vec<usize>,
    hdr_syms: usize,
}

impl Modem {
    pub fn new(params: OfdmParams, txdelay: f32) -> Self {
        let ofdm = Ofdm::new(params);
        let nd = ofdm.data_carriers();
        assert_eq!(HDR_CODE_BITS % nd, 0, "header must fill whole symbols");
        for l in &LEVELS {
            assert_eq!(CW_BITS % (nd * l.modulation.bits_per_symbol()), 0);
        }
        assert_eq!(ofdm.p.level_tx_rms.len(), LEVELS.len());
        assert_eq!(ofdm.p.level_snr_ceiling_db.len(), LEVELS.len());
        // Levels share codes (k = 1200 three times, 1800 twice): build each once.
        let mut codes: Vec<Ldpc> = Vec::with_capacity(LEVELS.len());
        for l in &LEVELS {
            let code = match codes.iter().find(|c| c.k == l.k) {
                Some(c) => c.clone(),
                None => Ldpc::new(CW_BITS, l.k, SEED_CODE + l.k as u32),
            };
            codes.push(code);
        }
        Self {
            hdr_syms: HDR_CODE_BITS / nd,
            hdr_code: Ldpc::new(HDR_CODE_BITS, HDR_INFO_BITS, SEED_HDR_CODE),
            codes,
            hdr_perm: permutation(SEED_HDR_INTERLEAVE, HDR_CODE_BITS),
            ofdm,
            txdelay,
            decode_threads: std::thread::available_parallelism().map_or(1, |n| n.get()),
        }
    }

    pub fn fs(&self) -> f32 {
        self.ofdm.p.fs
    }

    /// Output RMS level of a burst at `level` (1.0 = full scale = peak deviation).
    pub fn tx_rms(&self, level: u8) -> f32 {
        self.ofdm.p.level_tx_rms[level as usize]
    }

    /// How much louder (dB) a burst at `to_level` goes out than one at `from_level`.
    /// The receiver's SNR scales with it while the channel, not clipping, limits it.
    pub fn level_gain_db(&self, from_level: u8, to_level: u8) -> f32 {
        20.0 * (self.tx_rms(to_level) / self.tx_rms(from_level)).log10()
    }

    /// The SNR (dB) the receiver would report on a burst at `to_level`, given `snr_db`
    /// measured on a burst at `from_level` over the same channel.
    ///
    /// Model: `1/snr = 1/(channel · drive²) + 1/ceiling`. The channel part scales with
    /// the drive; the ceiling (`OfdmParams::level_snr_ceiling_db`) is the clipping
    /// floor of each level's drive. A measurement at or above its own ceiling says
    /// nothing about the channel, so it predicts the other level's ceiling.
    pub fn predict_snr_db(&self, snr_db: f32, from_level: u8, to_level: u8) -> f32 {
        let lin = |db: f32| 10f32.powf(db / 10.0);
        let ceiling = &self.ofdm.p.level_snr_ceiling_db;
        let inv_channel = (1.0 / lin(snr_db) - 1.0 / lin(ceiling[from_level as usize])).max(0.0);
        let gain = lin(self.level_gain_db(from_level, to_level));
        10.0 * (1.0 / (inv_channel / gain + 1.0 / lin(ceiling[to_level as usize]))).log10()
    }

    /// Data bytes per codeword at `level`.
    pub fn payload_bytes(level: u8) -> usize {
        LEVELS[level as usize].k / 8 - 4
    }

    pub fn header_symbols(&self) -> usize {
        self.hdr_syms
    }

    pub fn symbols_per_codeword(&self, level: u8) -> usize {
        CW_BITS / (self.ofdm.data_carriers() * LEVELS[level as usize].modulation.bits_per_symbol())
    }

    /// Most codewords per burst at `level`.
    pub fn max_ncw(&self, level: u8) -> usize {
        let cw_s = self.symbols_per_codeword(level) as f32 * self.ofdm.p.symbol_duration();
        ((MAX_PAYLOAD_SECONDS / cw_s) as usize).clamp(1, MAX_NCW)
    }

    fn leader_symbols(&self) -> usize {
        (self.txdelay / self.ofdm.p.symbol_duration()).ceil() as usize
    }

    /// Length in samples of a burst.
    pub fn burst_samples(&self, level: u8, ncw: usize) -> usize {
        let syms = self.leader_symbols() + 2 + self.hdr_syms + ncw * self.symbols_per_codeword(level);
        syms * self.ofdm.p.symbol_len()
    }

    pub fn burst_seconds(&self, level: u8, ncw: usize) -> f32 {
        self.burst_samples(level, ncw) as f32 / self.fs()
    }

    /// Net payload rate of a full burst at `level`, bits/s (including all overhead).
    pub fn net_rate(&self, level: u8) -> f32 {
        let ncw = self.max_ncw(level);
        (ncw * Self::payload_bytes(level) * 8) as f32 / self.burst_seconds(level, ncw)
    }

    fn header_bits(&self, h: &Header) -> Vec<u8> {
        let mut bits = bytes_to_bits(&h.pack());
        for (b, s) in bits.iter_mut().zip(scramble_bits(SEED_HDR_SCRAMBLE, HDR_INFO_BITS)) {
            *b ^= s;
        }
        let cw = self.hdr_code.encode(&bits);
        let mut slots = vec![0u8; HDR_CODE_BITS];
        for (j, &b) in cw.iter().enumerate() {
            slots[self.hdr_perm[j]] = b;
        }
        slots
    }

    fn payload_bits(&self, level: u8, payload: &[Vec<u8>]) -> Vec<u8> {
        let code = &self.codes[level as usize];
        let nbytes = Self::payload_bytes(level);
        let mut all = Vec::with_capacity(payload.len() * CW_BITS);
        for (j, data) in payload.iter().enumerate() {
            assert_eq!(data.len(), nbytes, "codeword {j} has the wrong length");
            let mut bytes = data.clone();
            bytes.extend_from_slice(&crc::crc32(data).to_be_bytes());
            let mut bits = bytes_to_bits(&bytes);
            for (b, s) in bits.iter_mut().zip(scramble_bits(SEED_SCRAMBLE + j as u32, code.k)) {
                *b ^= s;
            }
            all.extend(code.encode(&bits));
        }
        let perm = permutation(SEED_INTERLEAVE, all.len());
        let mut slots = vec![0u8; all.len()];
        for (j, &b) in all.iter().enumerate() {
            slots[perm[j]] = b;
        }
        slots
    }

    /// Audio for one burst (starts with the leader; ends with the last symbol).
    pub fn modulate(&self, burst: &Burst) -> Vec<f32> {
        self.ofdm.modulate_with_leader_rms(&self.symbols(burst), self.txdelay, self.tx_rms(burst.header.level))
    }

    /// The data-carrier values of a burst's header and payload symbols, as
    /// [`Modem::modulate`] sends them.
    fn symbols(&self, burst: &Burst) -> Vec<Vec<Complex32>> {
        let h = &burst.header;
        assert_eq!(h.ncw as usize, burst.payload.len());
        let nd = self.ofdm.data_carriers();
        let mut symbols: Vec<Vec<Complex32>> = Vec::new();
        let bpsk = Modulation::Bpsk;
        for chunk in self.header_bits(h).chunks(nd) {
            symbols.push(chunk.iter().map(|&b| bpsk.map(&[b])).collect());
        }
        if h.ncw > 0 {
            let m = LEVELS[h.level as usize].modulation;
            let bits = self.payload_bits(h.level, &burst.payload);
            for chunk in bits.chunks(nd * m.bits_per_symbol()) {
                symbols.push(m.map_all(chunk));
            }
        }
        symbols
    }

    /// The header's data-carrier values, one Vec per OFDM symbol, as transmitted.
    /// Once the header has decoded, the receiver uses them as channel training.
    fn header_carriers(&self, h: &Header) -> Vec<Vec<Complex32>> {
        let bpsk = Modulation::Bpsk;
        let nd = self.ofdm.data_carriers();
        self.header_bits(h).chunks(nd).map(|c| c.iter().map(|&b| bpsk.map(&[b])).collect()).collect()
    }

    /// LLRs for every bit slot of symbols `from..` of `f`. The noise variance of
    /// symbol `i`, carrier `j` is `f.noise_var[j] * f.symbol_noise_scale[i]`.
    fn llrs(m: Modulation, f: &Frame, from: usize) -> Vec<f32> {
        let symbols = &f.symbols[from..];
        let mut out = Vec::with_capacity(symbols.len() * f.noise_var.len() * m.bits_per_symbol());
        for (i, sym) in symbols.iter().enumerate() {
            for (j, y) in sym.iter().enumerate() {
                m.demap_llr(*y, f.noise_at(from + i, j), &mut out);
            }
        }
        out
    }

    /// Decode the header from the first `header_symbols()` symbols of `f`.
    fn decode_header(&self, f: &Frame) -> Option<Header> {
        let slots = Self::llrs(Modulation::Bpsk, f, 0);
        let llr: Vec<f32> = (0..HDR_CODE_BITS).map(|j| slots[self.hdr_perm[j]]).collect();
        let mut bits = self.hdr_code.decode(&llr, DECODE_ITERS)?;
        for (b, s) in bits.iter_mut().zip(scramble_bits(SEED_HDR_SCRAMBLE, HDR_INFO_BITS)) {
            *b ^= s;
        }
        Header::unpack(&vardefm_fec::bits_to_bytes(&bits))
    }

    /// Decode the payload codewords of `f` (a whole frame, header symbols included).
    fn decode_payload(&self, h: &Header, f: &Frame) -> Vec<Option<Vec<u8>>> {
        let level = LEVELS[h.level as usize];
        let code = &self.codes[h.level as usize];
        let slots = Self::llrs(level.modulation, f, self.hdr_syms);
        let total = h.ncw as usize * CW_BITS;
        let perm = permutation(SEED_INTERLEAVE, total);
        let decode = |j: usize| -> Option<Vec<u8>> {
            let llr: Vec<f32> = (0..CW_BITS).map(|i| slots[perm[j * CW_BITS + i]]).collect();
            let mut bits = code.decode(&llr, DECODE_ITERS)?;
            for (b, s) in bits.iter_mut().zip(scramble_bits(SEED_SCRAMBLE + j as u32, code.k)) {
                *b ^= s;
            }
            let bytes = vardefm_fec::bits_to_bytes(&bits);
            let (data, tail) = bytes.split_at(bytes.len() - 4);
            (crc::crc32(data).to_be_bytes() == tail).then(|| data.to_vec())
        };
        let ncw = h.ncw as usize;
        let threads = self.decode_threads.clamp(1, ncw.max(1));
        if threads == 1 {
            return (0..ncw).map(decode).collect();
        }
        // The whole burst is decoded after it ends, and the reply waits for it:
        // spread the codewords over the CPU cores (thread t takes t, t+threads, ...).
        let decode = &decode;
        let mut out = vec![None; ncw];
        std::thread::scope(|scope| {
            let workers: Vec<_> = (0..threads)
                .map(|t| scope.spawn(move || (t..ncw).step_by(threads).map(|j| (j, decode(j))).collect::<Vec<_>>()))
                .collect();
            for w in workers {
                for (j, r) in w.join().expect("decoder thread panicked") {
                    out[j] = r;
                }
            }
        });
        out
    }
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod busy_eval;

#[cfg(test)]
mod detect_eval;

#[cfg(test)]
mod level_eval;
