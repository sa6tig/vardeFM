//! Helpers shared by the `vardefm` tool and the integration tests:
//! known test payloads, frame generation, error counting and WAV I/O.

pub mod watch;

use num_complex::Complex32;
use std::path::Path;
use vardefm_dsp::prng::Prng;
use vardefm_dsp::{Frame, Modulation, Ofdm};

/// Known test payload for frame number `f`.
pub fn test_bits(ofdm: &Ofdm, m: Modulation, nsym: usize, f: usize) -> Vec<u8> {
    Prng::new(1000 + f as u32).bits(nsym * ofdm.data_carriers() * m.bits_per_symbol())
}

/// Modulate `bits` into one frame of audio, with a leader of `txdelay` seconds.
pub fn make_frame(ofdm: &Ofdm, m: Modulation, bits: &[u8], txdelay: f32) -> Vec<f32> {
    let syms = m.map_all(bits);
    let data: Vec<Vec<Complex32>> = syms.chunks(ofdm.data_carriers()).map(|c| c.to_vec()).collect();
    ofdm.modulate_with_leader(&data, txdelay)
}

/// Hard-decision bit errors between a demodulated frame and the bits that were sent.
pub fn count_errors(m: Modulation, frame: &Frame, bits: &[u8]) -> usize {
    let mut out = Vec::with_capacity(bits.len());
    for s in frame.symbols.iter().flatten() {
        m.hard_bits(*s, &mut out);
    }
    out.iter().zip(bits).filter(|(a, b)| a != b).count()
}

/// Detect and demodulate every frame in a recording. Returns (start sample, frame).
pub fn decode_all(ofdm: &Ofdm, rx: &[f32], nsym: usize) -> Vec<(usize, Frame)> {
    let mut pos = 0;
    let mut frames = Vec::new();
    while let Some(b) = ofdm.detect(rx, pos) {
        let Some(frame) = ofdm.demodulate(rx, b, nsym) else { break };
        pos = frame.end;
        frames.push((b, frame));
    }
    frames
}

/// Write mono audio as a 16-bit WAV file.
pub fn write_wav(path: impl AsRef<Path>, fs: u32, samples: &[f32]) -> Result<(), hound::Error> {
    let spec =
        hound::WavSpec { channels: 1, sample_rate: fs, bits_per_sample: 16, sample_format: hound::SampleFormat::Int };
    let mut w = hound::WavWriter::create(path, spec)?;
    for &v in samples {
        w.write_sample((v.clamp(-1.0, 1.0) * 32767.0) as i16)?;
    }
    w.finalize()
}

/// Read a WAV file (any bit depth, int or float). Returns (sample rate, first channel).
pub fn read_wav(path: impl AsRef<Path>) -> Result<(u32, Vec<f32>), hound::Error> {
    let mut r = hound::WavReader::open(path)?;
    let spec = r.spec();
    let samples: Vec<f32> = match spec.sample_format {
        hound::SampleFormat::Int => {
            let max = (1i64 << (spec.bits_per_sample - 1)) as f32;
            r.samples::<i32>().map(|s| s.map(|v| v as f32 / max)).collect::<Result<_, _>>()?
        }
        hound::SampleFormat::Float => r.samples::<f32>().collect::<Result<_, _>>()?,
    };
    let ch = spec.channels as usize;
    Ok((spec.sample_rate, samples.into_iter().step_by(ch).collect()))
}
