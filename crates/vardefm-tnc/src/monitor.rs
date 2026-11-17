//! The radio's side of the read-only monitor API: the server, the waterfall and the counters.
//!
//! [`crate::radio::run`] gives a [`Monitor`] every chunk of received audio before the engine
//! steps, the time each step took, the engine's events, and once a second the counters. It
//! publishes them on the radio's own thread: publishing only queues messages, so it never waits
//! on a client, and nothing is dropped between the radio loop and the server.

use std::io;
use std::net::SocketAddr;
use std::time::{Duration, Instant};
use vardefm_api::waterfall::{bins_for_mode, Waterfall};
use vardefm_api::wire::{mode_name, ApiEvent, Hello, LinkState, LinkStatus, Status, WireTime};
use vardefm_api::{ApiServer, LoopbackAddr};
use vardefm_audio::RATE;
use vardefm_link::{LinkState as Link, LinkView};

/// Time between two `Status` messages.
pub const STATUS_EVERY: Duration = Duration::from_secs(1);

pub struct Monitor {
    server: ApiServer,
    waterfall: Waterfall,
    mode: String,
    gaps: u64,
    last_status: Instant,
    /// Since the last `Status`.
    window: Window,
}

/// What a `Status` measures over the time since the last one.
#[derive(Default)]
struct Window {
    /// Received audio.
    samples: u64,
    /// Time stepping the engine.
    modem: Duration,
    /// Time making waterfall rows.
    tap: Duration,
    /// Longest single step.
    longest: Duration,
}

impl Monitor {
    /// Starts the server on `addr` for `mode` (`narrow` or `wide`).
    pub fn start(addr: LoopbackAddr, mode: &str) -> io::Result<Self> {
        let bins = bins_for_mode(mode).ok_or_else(|| io::Error::other("mode must be narrow or wide"))?;
        let waterfall = Waterfall::new(bins.into());
        let server = ApiServer::start(addr, Hello::new(mode, &waterfall))?;
        Ok(Self {
            server,
            waterfall,
            mode: mode_name(mode),
            gaps: 0,
            last_status: Instant::now(),
            window: Window::default(),
        })
    }

    /// The address the server listens on.
    pub fn local_addr(&self) -> SocketAddr {
        self.server.local_addr()
    }

    /// Received audio starting at `t0` on the receive clock; `after_gap` if audio was lost just
    /// before it. Makes and publishes the waterfall rows it completes.
    pub fn audio(&mut self, t0: u64, after_gap: bool, samples: &[f32]) {
        let started = Instant::now();
        if after_gap {
            self.waterfall.gap();
        }
        let server = &self.server;
        self.waterfall.push(t0, samples, &mut |row| server.publish_row(&row));
        self.window.samples += samples.len() as u64;
        self.window.tap += started.elapsed();
    }

    /// One engine step took `took`.
    pub fn modem_step(&mut self, took: Duration) {
        self.window.modem += took;
        self.window.longest = self.window.longest.max(took);
    }

    /// Publishes the engine's events.
    pub fn publish(&mut self, events: impl IntoIterator<Item = ApiEvent>) {
        for e in events {
            if matches!(e, ApiEvent::Gap { .. }) {
                self.gaps += 1;
            }
            self.server.publish_event(e);
        }
    }

    /// Publishes binary frames, such as the engine's constellations.
    pub fn publish_frames(&mut self, frames: impl IntoIterator<Item = Vec<u8>>) {
        for f in frames {
            self.server.publish_frame(f);
        }
    }

    /// True once [`STATUS_EVERY`] has passed since the last `Status`.
    pub fn status_due(&self) -> bool {
        self.last_status.elapsed() >= STATUS_EVERY
    }

    /// Publishes the counters: the receive clock is at `t`, the input has dropped `overruns`
    /// chunks holding `dropped_frames` samples.
    pub fn publish_status(&mut self, t: u64, link: &LinkView, overruns: u64, dropped_frames: u64) {
        self.last_status = Instant::now();
        let window = std::mem::take(&mut self.window);
        self.server.publish_status(status(&self.mode, t, &window, self.gaps, link, overruns, dropped_frames));
    }
}

/// The `Status` for one window. The fields nothing here measures stay 0 (see
/// `docs/api/README.md`); `client_rows_dropped` is the server's to fill in.
fn status(
    mode: &str,
    t: u64,
    window: &Window,
    gaps: u64,
    link: &LinkView,
    overruns: u64,
    dropped_frames: u64,
) -> Status {
    let audio_s = window.samples as f64 / RATE as f64;
    let percent = |d: Duration| if audio_s > 0.0 { (d.as_secs_f64() / audio_s * 100.0) as f32 } else { 0.0 };
    Status {
        t: WireTime(t),
        mode: mode.to_owned(),
        cpu: percent(window.modem),
        tap_cpu: percent(window.tap),
        longest_push_ms: u32::try_from(window.longest.as_millis()).unwrap_or(u32::MAX),
        overruns,
        dropped_frames,
        gaps,
        link: Some(link_status(link)),
        ..Status::default()
    }
}

fn link_status(view: &LinkView) -> LinkStatus {
    LinkStatus {
        state: match view.state {
            Link::Idle => LinkState::Idle,
            Link::Listening => LinkState::Listening,
            Link::Connecting => LinkState::Connecting,
            Link::Connected => LinkState::Connected,
            Link::Disconnecting => LinkState::Disconnecting,
        },
        peer: view.peer.clone(),
        level: view.level,
        tx_queued: view.tx_queued,
        rx_queued: 0,
    }
}

#[cfg(test)]
mod tests;
