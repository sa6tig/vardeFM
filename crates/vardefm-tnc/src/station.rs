//! One VardeFM station: link + modulator + receiver, speaking the VARA host protocol.
//!
//! The station is driven from outside: feed it host input and received audio, ask
//! it what to transmit, and collect its host output. It has no threads or clock.

use crate::host::{parse, Command};
use std::sync::Arc;
use vardefm_link::{Link, LinkConfig, LinkEvent, LinkView, Node, TxInfo};
use vardefm_modem::{Header, Modem, Receiver, RxBurst};

/// Output for the host application.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum HostOut {
    /// A command-port line (without the CR).
    Line(String),
    /// Bytes for the data port.
    Data(Vec<u8>),
}

/// A burst that has just been keyed up.
pub struct TxStart {
    pub audio: Vec<f32>,
    pub header: Header,
    pub info: TxInfo,
    pub seconds: f32,
    /// A digipeater repeat of another station's burst.
    pub repeat: bool,
}

const IAMALIVE_S: f64 = 60.0;

pub struct Station {
    pub name: String,
    modem: Arc<Modem>,
    /// The link and, if set, the digipeater.
    node: Node,
    rx: Receiver,
    out: Vec<HostOut>,
    transmitting: bool,
    busy_reported: bool,
    last_alive: f64,
    bandwidth: &'static str,
}

impl Station {
    pub fn new(name: &str, modem: Arc<Modem>, cfg: LinkConfig, seed: u32) -> Self {
        let bandwidth = if modem.ofdm.p.name == "wide" { "WIDE" } else { "NARROW" };
        Self {
            name: name.to_string(),
            node: Node::new(Link::new(modem.clone(), cfg, seed)),
            rx: Receiver::new(modem.clone()),
            modem,
            out: Vec::new(),
            transmitting: false,
            busy_reported: false,
            last_alive: 0.0,
            bandwidth,
        }
    }

    pub fn modem(&self) -> &Arc<Modem> {
        &self.modem
    }

    pub fn link_config(&self) -> &LinkConfig {
        self.node.link.config()
    }

    /// Repeat the bursts of sessions routed through any of `calls` (none: off).
    /// Works without a host and alongside our own sessions.
    pub fn set_digi(&mut self, calls: &[String]) {
        self.node.set_digi(calls);
    }

    fn line(&mut self, s: impl Into<String>) {
        self.out.push(HostOut::Line(s.into()));
    }

    /// One line from the command port.
    pub fn host_line(&mut self, now: f64, line: &str) {
        let Ok(cmd) = parse(line) else {
            self.line("WRONG");
            return;
        };
        match cmd {
            Command::MyCall(calls) => {
                self.node.link.set_mycalls(calls);
                self.line("OK");
            }
            Command::Listen(on) => {
                self.node.link.set_listen(on);
                self.line("OK");
            }
            Command::Connect { source, target, via } => match self.node.link.connect(now, &source, &target, &via) {
                Ok(()) => self.line("OK"),
                Err(()) => self.line("WRONG"),
            },
            Command::Disconnect => {
                self.line("OK");
                self.node.link.disconnect(now);
            }
            Command::Abort => {
                self.line("OK");
                self.node.link.abort(now);
            }
            Command::Version => {
                self.line("OK");
                self.line(format!("VERSION {}", crate::VERSION));
            }
            Command::Ignored => self.line("OK"),
        }
    }

    /// Bytes from the data port (dropped when not connected).
    pub fn host_data(&mut self, data: &[u8]) {
        self.node.link.send_data(data);
    }

    /// The host closed the command port: drop any session.
    pub fn host_closed(&mut self, now: f64) {
        self.node.link.abort(now);
    }

    /// Periodic housekeeping (IAMALIVE).
    pub fn tick(&mut self, now: f64) {
        if now - self.last_alive >= IAMALIVE_S {
            self.last_alive = now;
            self.line("IAMALIVE");
        }
    }

    /// If the link wants to transmit, key up and return the audio.
    /// Call only while not transmitting.
    pub fn poll_tx(&mut self, now: f64) -> Option<TxStart> {
        debug_assert!(!self.transmitting);
        let (burst, repeat) = self.node.poll(now, self.rx.busy())?;
        let audio = self.modem.modulate(&burst);
        self.transmitting = true;
        self.rx.reset();
        self.line("PTT ON");
        Some(TxStart {
            seconds: audio.len() as f32 / self.modem.fs(),
            audio,
            info: if repeat {
                TxInfo {
                    kind: burst.header.kind,
                    level: burst.header.level,
                    ncw: burst.payload.len(),
                    ..TxInfo::default()
                }
            } else {
                self.node.link.last_tx().clone()
            },
            header: burst.header,
            repeat,
        })
    }

    /// The burst returned by `poll_tx` has been fully transmitted.
    pub fn tx_done(&mut self, now: f64) {
        self.transmitting = false;
        self.line("PTT OFF");
        self.node.tx_done(now);
    }

    pub fn transmitting(&self) -> bool {
        self.transmitting
    }

    /// The busy state last reported to the host (`BUSY ON` / `BUSY OFF`).
    pub fn busy(&self) -> bool {
        self.busy_reported
    }

    /// Samples the receiver has been given so far. [`RxBurst::end_sample`] counts in
    /// these, which skip the audio heard while transmitting.
    pub fn rx_position(&self) -> u64 {
        self.rx.position()
    }

    /// Where the link stands, for monitoring.
    pub fn link_view(&self) -> LinkView {
        self.node.link.view()
    }

    /// Received audio. Returns any bursts decoded (for logging).
    pub fn rx_audio(&mut self, now: f64, samples: &[f32]) -> Vec<RxBurst> {
        if self.transmitting {
            return Vec::new();
        }
        let bursts = self.rx.push(samples);
        for b in &bursts {
            self.node.on_burst(now, b);
        }
        let busy = self.rx.busy();
        if busy != self.busy_reported {
            self.busy_reported = busy;
            self.line(if busy { "BUSY ON" } else { "BUSY OFF" });
        }
        bursts
    }

    /// Everything for the host since the last call, in order.
    pub fn take_output(&mut self) -> Vec<HostOut> {
        for e in self.node.link.take_events() {
            let o = match e {
                LinkEvent::Pending => HostOut::Line("PENDING".into()),
                LinkEvent::CancelPending => HostOut::Line("CANCELPENDING".into()),
                LinkEvent::Connected { source, destination } => {
                    HostOut::Line(format!("CONNECTED {source} {destination} {}", self.bandwidth))
                }
                LinkEvent::Disconnected => HostOut::Line("DISCONNECTED".into()),
                LinkEvent::Buffer(n) => HostOut::Line(format!("BUFFER {n}")),
                LinkEvent::Data(d) => HostOut::Data(d),
            };
            self.out.push(o);
        }
        std::mem::take(&mut self.out)
    }
}
