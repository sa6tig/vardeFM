//! Sound card input and output at the modem rate (48 kHz mono), using cpal.
//!
//! - Devices are chosen by (part of) their name; `None` means the system default.
//! - Any sample format is converted to f32. One channel is used for input; output
//!   goes to the left, right or both channels.
//! - If a device cannot run at 48 kHz, audio is resampled in software (cubic).
//!   Most USB radio interfaces support 48 kHz natively, which is better.

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{FromSample, SampleFormat, SizedSample, Stream, StreamConfig};
use std::collections::VecDeque;
use std::io;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{sync_channel, Receiver, SyncSender};
use std::sync::{Arc, Mutex};

pub const RATE: u32 = 48_000;

/// Which channel(s) of a stereo device to use.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Channel {
    Left,
    Right,
    /// Output only: the same signal on every channel.
    Both,
}

impl std::str::FromStr for Channel {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, String> {
        match s.to_ascii_lowercase().as_str() {
            "left" | "l" | "0" => Ok(Self::Left),
            "right" | "r" | "1" => Ok(Self::Right),
            "both" => Ok(Self::Both),
            _ => Err(format!("channel must be left, right or both, not '{s}'")),
        }
    }
}

fn err(e: impl std::fmt::Display) -> io::Error {
    io::Error::other(e.to_string())
}

/// Names of input and output devices.
pub fn list_devices() -> io::Result<(Vec<String>, Vec<String>)> {
    let host = cpal::default_host();
    let names = |it: Result<Box<dyn Iterator<Item = cpal::Device>>, _>| -> Vec<String> {
        it.map(|d| d.map(|d| d.to_string()).collect()).unwrap_or_default()
    };
    let ins = names(host.input_devices().map(|d| Box::new(d) as Box<dyn Iterator<Item = _>>));
    let outs = names(host.output_devices().map(|d| Box::new(d) as Box<dyn Iterator<Item = _>>));
    Ok((ins, outs))
}

fn pick(
    devices: impl Iterator<Item = cpal::Device>,
    default: Option<cpal::Device>,
    name: Option<&str>,
    what: &str,
) -> io::Result<cpal::Device> {
    match name {
        None => default.ok_or_else(|| err(format!("no default {what} device"))),
        Some(want) => {
            let want_l = want.to_lowercase();
            let all: Vec<cpal::Device> = devices.collect();
            if let Some(d) = all.iter().find(|d| d.to_string().to_lowercase() == want_l) {
                return Ok(d.clone());
            }
            let matches: Vec<&cpal::Device> =
                all.iter().filter(|d| d.to_string().to_lowercase().contains(&want_l)).collect();
            match matches.len() {
                1 => Ok(matches[0].clone()),
                0 => Err(err(format!("no {what} device matches '{want}' (see `vardefm devices`)"))),
                _ => Err(err(format!(
                    "'{want}' matches several {what} devices: {}",
                    matches.iter().map(|d| d.to_string()).collect::<Vec<_>>().join(", ")
                ))),
            }
        }
    }
}

/// Choose a stream config: 48 kHz if possible, preferring f32 and i16 formats.
fn choose_config(
    ranges: Vec<cpal::SupportedStreamConfigRange>,
    default: Option<cpal::SupportedStreamConfig>,
) -> io::Result<cpal::SupportedStreamConfig> {
    let rank = |f: SampleFormat| match f {
        SampleFormat::F32 => 0,
        SampleFormat::I16 => 1,
        SampleFormat::I32 => 2,
        SampleFormat::U16 => 3,
        _ => 9,
    };
    let mut at_rate: Vec<_> = ranges
        .iter()
        .filter(|r| rank(r.sample_format()) < 9 && r.min_sample_rate() <= RATE && r.max_sample_rate() >= RATE)
        .collect();
    at_rate.sort_by_key(|r| (rank(r.sample_format()), r.channels()));
    if let Some(r) = at_rate.first() {
        return Ok((*(*r)).with_sample_rate(RATE));
    }
    match default {
        Some(d) if rank(d.sample_format()) < 9 => Ok(d),
        _ => Err(err("device offers no usable sample format")),
    }
}

/// Streaming cubic (Hermite) resampler.
struct Resampler {
    step: f64,
    pos: f64,
    hist: [f32; 4],
}

impl Resampler {
    fn new(from: u32, to: u32) -> Self {
        Self { step: from as f64 / to as f64, pos: 0.0, hist: [0.0; 4] }
    }

    fn push(&mut self, x: f32, out: &mut Vec<f32>) {
        self.hist = [self.hist[1], self.hist[2], self.hist[3], x];
        // Interpolate between hist[1] and hist[2].
        while self.pos < 1.0 {
            let f = self.pos as f32;
            let [y0, y1, y2, y3] = self.hist;
            let c1 = 0.5 * (y2 - y0);
            let c2 = y0 - 2.5 * y1 + 2.0 * y2 - 0.5 * y3;
            let c3 = 0.5 * (y3 - y0) + 1.5 * (y1 - y2);
            out.push(((c3 * f + c2) * f + c1) * f + y1);
            self.pos += self.step;
        }
        self.pos -= 1.0;
    }
}

/// Capture: 48 kHz mono chunks arrive on `rx`.
pub struct AudioIn {
    _stream: Stream,
    pub rx: Receiver<Vec<f32>>,
    pub name: String,
    /// Device rate (48000 unless resampling).
    pub device_rate: u32,
    /// Chunks dropped because the receiver fell behind.
    pub overruns: Arc<AtomicU64>,
    /// Samples (at 48 kHz) in those dropped chunks. Updated before `overruns`, so
    /// after reading `overruns` with `Acquire` this covers at least those chunks.
    pub lost_samples: Arc<AtomicU64>,
}

pub fn open_input(name: Option<&str>, channel: Channel) -> io::Result<AudioIn> {
    let host = cpal::default_host();
    let dev = pick(host.input_devices().map_err(err)?, host.default_input_device(), name, "input")?;
    let cfg = choose_config(dev.supported_input_configs().map_err(err)?.collect(), dev.default_input_config().ok())?;
    let channels = cfg.channels() as usize;
    let device_rate = cfg.sample_rate();
    let ch = match channel {
        Channel::Right if channels > 1 => 1,
        _ => 0,
    };
    let (sink, rx) = ChunkSink::new(256);
    let (overruns, lost_samples) = (sink.overruns.clone(), sink.lost_samples.clone());
    let stream_cfg: StreamConfig = cfg.config();
    let stream = match cfg.sample_format() {
        SampleFormat::F32 => build_in::<f32>(&dev, &stream_cfg, channels, ch, device_rate, sink),
        SampleFormat::I16 => build_in::<i16>(&dev, &stream_cfg, channels, ch, device_rate, sink),
        SampleFormat::I32 => build_in::<i32>(&dev, &stream_cfg, channels, ch, device_rate, sink),
        SampleFormat::U16 => build_in::<u16>(&dev, &stream_cfg, channels, ch, device_rate, sink),
        f => return Err(err(format!("unsupported sample format {f:?}"))),
    }?;
    stream.play().map_err(err)?;
    Ok(AudioIn { _stream: stream, rx, name: dev.to_string(), device_rate, overruns, lost_samples })
}

/// The capture callback's end of the chunk channel, counting what it has to drop.
struct ChunkSink {
    tx: SyncSender<Vec<f32>>,
    overruns: Arc<AtomicU64>,
    lost_samples: Arc<AtomicU64>,
}

impl ChunkSink {
    fn new(room: usize) -> (Self, Receiver<Vec<f32>>) {
        let (tx, rx) = sync_channel(room);
        let sink = Self { tx, overruns: Arc::default(), lost_samples: Arc::default() };
        (sink, rx)
    }

    /// Hand over a 48 kHz chunk without waiting; if the reader has fallen behind, drop
    /// it and count it.
    fn deliver(&self, chunk: Vec<f32>) {
        let n = chunk.len() as u64;
        if self.tx.try_send(chunk).is_err() {
            // Samples first: a reader that sees the overrun count grow then sees them all.
            self.lost_samples.fetch_add(n, Ordering::Relaxed);
            self.overruns.fetch_add(1, Ordering::Release);
        }
    }
}

fn build_in<T>(
    dev: &cpal::Device,
    cfg: &StreamConfig,
    channels: usize,
    ch: usize,
    rate: u32,
    sink: ChunkSink,
) -> io::Result<Stream>
where
    T: SizedSample,
    f32: FromSample<T>,
{
    let mut rs = (rate != RATE).then(|| Resampler::new(rate, RATE));
    dev.build_input_stream(
        *cfg,
        move |data: &[T], _| {
            let mut out = Vec::with_capacity(data.len() / channels + 8);
            for frame in data.chunks(channels) {
                let x: f32 = frame[ch.min(frame.len() - 1)].to_sample();
                match &mut rs {
                    Some(r) => r.push(x, &mut out),
                    None => out.push(x),
                }
            }
            sink.deliver(out);
        },
        |e| eprintln!("audio input error: {e}"),
        None,
    )
    .map_err(err)
}

/// Playback: queue 48 kHz mono samples with [`AudioOut::play`].
pub struct AudioOut {
    _stream: Stream,
    queue: Arc<Mutex<VecDeque<f32>>>,
    /// Queued samples the sound card has taken so far.
    consumed: Arc<AtomicU64>,
    /// Queued samples so far.
    queued_total: u64,
    pub name: String,
    pub device_rate: u32,
}

impl AudioOut {
    /// Queue samples (full scale = 1.0). Returns the `consumed()` value at which
    /// the last of them has been taken by the sound card.
    pub fn play(&mut self, samples: &[f32]) -> u64 {
        self.queue.lock().unwrap().extend(samples.iter().copied());
        self.queued_total += samples.len() as u64;
        self.queued_total
    }

    pub fn consumed(&self) -> u64 {
        self.consumed.load(Ordering::Acquire)
    }

    /// Drop anything not yet played.
    pub fn flush(&mut self) {
        let mut q = self.queue.lock().unwrap();
        let dropped = q.len() as u64;
        q.clear();
        self.consumed.fetch_add(dropped, Ordering::AcqRel);
    }
}

pub fn open_output(name: Option<&str>, channel: Channel) -> io::Result<AudioOut> {
    let host = cpal::default_host();
    let dev = pick(host.output_devices().map_err(err)?, host.default_output_device(), name, "output")?;
    let cfg = choose_config(dev.supported_output_configs().map_err(err)?.collect(), dev.default_output_config().ok())?;
    let channels = cfg.channels() as usize;
    let device_rate = cfg.sample_rate();
    let queue = Arc::new(Mutex::new(VecDeque::new()));
    let consumed = Arc::new(AtomicU64::new(0));
    let stream_cfg: StreamConfig = cfg.config();
    let (q, c) = (queue.clone(), consumed.clone());
    let stream = match cfg.sample_format() {
        SampleFormat::F32 => build_out::<f32>(&dev, &stream_cfg, channels, channel, device_rate, q, c),
        SampleFormat::I16 => build_out::<i16>(&dev, &stream_cfg, channels, channel, device_rate, q, c),
        SampleFormat::I32 => build_out::<i32>(&dev, &stream_cfg, channels, channel, device_rate, q, c),
        SampleFormat::U16 => build_out::<u16>(&dev, &stream_cfg, channels, channel, device_rate, q, c),
        f => return Err(err(format!("unsupported sample format {f:?}"))),
    }?;
    stream.play().map_err(err)?;
    Ok(AudioOut { _stream: stream, queue, consumed, queued_total: 0, name: dev.to_string(), device_rate })
}

fn build_out<T>(
    dev: &cpal::Device,
    cfg: &StreamConfig,
    channels: usize,
    channel: Channel,
    rate: u32,
    queue: Arc<Mutex<VecDeque<f32>>>,
    consumed: Arc<AtomicU64>,
) -> io::Result<Stream>
where
    T: SizedSample + FromSample<f32>,
{
    // Resampling 48 kHz -> device rate: pull modem samples as needed.
    let mut rs = (rate != RATE).then(|| Resampler::new(RATE, rate));
    let mut pending: VecDeque<f32> = VecDeque::new();
    dev.build_output_stream(
        *cfg,
        move |data: &mut [T], _| {
            let frames = data.len() / channels;
            let mut taken = 0u64;
            {
                let mut q = queue.lock().unwrap();
                while pending.len() < frames {
                    let Some(x) = q.pop_front() else { break };
                    taken += 1;
                    match &mut rs {
                        Some(r) => {
                            let mut v = Vec::new();
                            r.push(x, &mut v);
                            pending.extend(v);
                        }
                        None => pending.push_back(x),
                    }
                }
            }
            consumed.fetch_add(taken, Ordering::AcqRel);
            for frame in data.chunks_mut(channels) {
                let x = pending.pop_front().unwrap_or(0.0).clamp(-1.0, 1.0);
                for (i, s) in frame.iter_mut().enumerate() {
                    let on = match channel {
                        Channel::Both => true,
                        Channel::Left => i == 0,
                        Channel::Right => i == 1 || channels == 1,
                    };
                    *s = T::from_sample(if on { x } else { 0.0 });
                }
            }
        },
        |e| eprintln!("audio output error: {e}"),
        None,
    )
    .map_err(err)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dropped_chunks_count_their_samples() {
        let (sink, rx) = ChunkSink::new(1);
        sink.deliver(vec![0.0; 100]);
        sink.deliver(vec![0.0; 30]);
        sink.deliver(vec![0.0; 7]);
        assert_eq!(sink.overruns.load(Ordering::Acquire), 2);
        assert_eq!(sink.lost_samples.load(Ordering::Relaxed), 37);
        assert_eq!(rx.try_recv().unwrap().len(), 100);
        sink.deliver(vec![0.0; 5]);
        assert_eq!(rx.try_recv().unwrap().len(), 5);
        assert_eq!(sink.overruns.load(Ordering::Acquire), 2);
    }

    #[test]
    fn resampler_keeps_a_tone() {
        // 1 kHz at 44.1 kHz -> 48 kHz; compare against an ideal 1 kHz at 48 kHz.
        let mut r = Resampler::new(44_100, 48_000);
        let mut out = Vec::new();
        for i in 0..44_100 {
            r.push((2.0 * std::f32::consts::PI * 1000.0 * i as f32 / 44_100.0).sin(), &mut out);
        }
        assert!((out.len() as i64 - 48_000).abs() < 5, "{}", out.len());
        // Output sample k sits at input time (k * step) - 2 samples (interpolation delay).
        let mut err = 0f32;
        for (k, y) in out.iter().enumerate().skip(100).take(40_000) {
            let t = (k as f64 * 44_100.0 / 48_000.0 - 2.0) / 44_100.0;
            let ideal = (2.0 * std::f64::consts::PI * 1000.0 * t).sin() as f32;
            err = err.max((y - ideal).abs());
        }
        assert!(err < 0.01, "max error {err}");
    }
}
