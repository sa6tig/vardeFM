//! `vardefm` — test and measurement tool for the VardeFM modem.

use clap::{Parser, Subcommand};

mod bench;
mod radio_cmds;

use vardefm_audio::Channel;
use vardefm_cli::{count_errors, decode_all, make_frame, read_wav, test_bits, write_wav};
use vardefm_dsp::{Detector, Modulation, Ofdm, OfdmParams};
use vardefm_sim::{transmit, FmChannelConfig};
use vardefm_tnc::simpair::{SimPair, SimPairConfig};

#[derive(Parser)]
#[command(name = "vardefm", version, about = "VardeFM: open FM data modem with a VARA-compatible host interface")]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(clap::Args, Clone)]
struct ModemArgs {
    /// narrow (mic/speaker) or wide (9600 data port)
    #[arg(long, default_value = "narrow")]
    mode: String,
    /// bpsk, qpsk, 16qam or 64qam
    #[arg(long = "mod", default_value = "qpsk")]
    modulation: String,
    /// Data symbols per frame
    #[arg(long, default_value_t = 40, value_parser = at_least_one)]
    symbols: usize,
    /// Modem spectral tilt in dB/octave at both ends, which the receiver expects
    /// (negative tilts down the high carriers); see --tx-tilt for a radio mismatch
    #[arg(long)]
    tilt: Option<f32>,
    /// TX RMS level, 0..1
    #[arg(long)]
    rms: Option<f32>,
    /// Leader before each frame, seconds (must exceed the radio's key-up time)
    #[arg(long, default_value_t = 0.1)]
    txdelay: f32,
}

#[derive(clap::Args, Clone)]
struct DetectorArg {
    /// How the receiver finds bursts (sc = Schmidl-Cox, mf = matched filter)
    #[arg(long, default_value_t = Detector::default(), value_parser = detector_parser())]
    detector: Detector,
}

/// Accepts exactly the names in `Detector::ALL`, which `--help` lists.
fn detector_parser() -> impl clap::builder::TypedValueParser<Value = Detector> {
    use clap::builder::TypedValueParser;
    clap::builder::PossibleValuesParser::new(Detector::ALL.map(Detector::name))
        .map(|s| s.parse::<Detector>().expect("every possible value names a detector"))
}

impl ModemArgs {
    /// Like [`ModemArgs::build`], with the receiver finding frames with `det`.
    fn build_rx(&self, det: &DetectorArg) -> (Ofdm, Modulation) {
        let (mut ofdm, m) = self.build();
        ofdm.detector = det.detector;
        (ofdm, m)
    }

    fn build(&self) -> (Ofdm, Modulation) {
        let mut p = OfdmParams::by_name(&self.mode).unwrap_or_else(|| die("unknown mode"));
        if let Some(t) = self.tilt {
            p.tilt_db_per_octave = t;
        }
        if let Some(r) = self.rms {
            p.tx_rms = r;
        }
        let m = Modulation::from_name(&self.modulation).unwrap_or_else(|| die("unknown modulation"));
        (Ofdm::new(p), m)
    }
}

#[derive(Subcommand)]
enum Cmd {
    /// Send frames through the FM channel simulator and measure bit errors
    Sim {
        #[command(flatten)]
        modem: ModemArgs,
        #[command(flatten)]
        det: DetectorArg,
        /// Carrier-to-noise ratio in dB (omit for no noise)
        #[arg(long)]
        cnr: Option<f32>,
        #[arg(long, default_value_t = 10)]
        frames: usize,
        /// Receiver sound-card clock error in ppm
        #[arg(long, default_value_t = 0.0)]
        ppm: f32,
        /// Simulated radio key-up time, seconds
        #[arg(long, default_value_t = 0.05)]
        keyup: f32,
        /// Sending radio's audio response tilt in dB/octave, -6 to 6, at the audio's own
        /// level (a radio mismatch; --tilt changes the modem at both ends instead)
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true, value_parser = radio_tilt)]
        tx_tilt: f32,
    },
    /// BER table over CNR for every modulation
    Sweep {
        #[command(flatten)]
        modem: ModemArgs,
        #[command(flatten)]
        det: DetectorArg,
        #[arg(long, default_value_t = 5)]
        frames: usize,
        #[arg(long, default_value_t = 0.0)]
        ppm: f32,
        /// Simulated radio key-up time, seconds
        #[arg(long, default_value_t = 0.05)]
        keyup: f32,
        /// Sending radio's audio response tilt in dB/octave, -6 to 6, at the audio's own
        /// level (a radio mismatch; --tilt changes the modem at both ends instead)
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true, value_parser = radio_tilt)]
        tx_tilt: f32,
    },
    /// Write test frames to a WAV file (play it into a radio)
    TxWav {
        #[command(flatten)]
        modem: ModemArgs,
        #[arg(long, default_value_t = 5)]
        frames: usize,
        #[arg(long)]
        out: String,
    },
    /// Decode test frames from a WAV file (a recording of a radio's output)
    RxWav {
        #[command(flatten)]
        modem: ModemArgs,
        #[command(flatten)]
        det: DetectorArg,
        #[arg(long = "in")]
        input: String,
    },
    /// Run the modem on a real radio, with VARA-compatible TCP ports
    Modem {
        /// narrow (mic/speaker) or wide (9600 data port)
        #[arg(long, default_value = "narrow")]
        mode: String,
        /// Input sound card (any unique part of its name; default: system default)
        #[arg(long)]
        audio_in: Option<String>,
        /// Output sound card (default: system default)
        #[arg(long)]
        audio_out: Option<String>,
        /// Input channel: left or right
        #[arg(long, default_value = "left")]
        in_channel: Channel,
        /// Output channel: left, right or both
        #[arg(long, default_value = "both")]
        out_channel: Channel,
        /// PTT: none, vox, cm108[:<gpio>][@<path>], rts:<port>, dtr:<port>, -rts:<port>, rigctld[:<host>:<port>], gpio:<pin>
        #[arg(long, default_value = "none")]
        ptt: String,
        /// Leader before each burst, seconds (radio key-up time plus margin)
        #[arg(long, default_value_t = 0.15)]
        txdelay: f32,
        /// Seconds PTT stays on after the last sample
        #[arg(long, default_value_t = 0.05)]
        txtail: f32,
        /// Transmit level multiplier (also see the `tone` command)
        #[arg(long, default_value_t = 1.0)]
        tx_gain: f32,
        /// Address to listen on
        #[arg(long, default_value = "127.0.0.1")]
        host: String,
        /// Command port (data port = +1)
        #[arg(long, default_value_t = 8300)]
        port: u16,
        /// Save every transmitted burst as a WAV file in this folder
        #[arg(long)]
        record: Option<String>,
        /// Highest speed level (0-6) to send or ask the other station for; lower it on a slow computer (see `bench`)
        #[arg(long, default_value_t = 6, value_parser = clap::value_parser!(u8).range(0..=6))]
        max_level: u8,
        /// Do not print a line per burst
        #[arg(long)]
        quiet: bool,
        /// Print the receive level every 2 s while idle, to set the radio's volume
        #[arg(long)]
        levels: bool,
        #[command(flatten)]
        det: DetectorArg,
    },
    /// Measure whether this computer decodes fast enough for each mode and speed level
    ///
    /// Bursts are timed with --detector; the idle load is shown for every detector.
    Bench {
        /// narrow, wide or both
        #[arg(long, default_value = "both")]
        mode: String,
        /// Decoder threads (default: one per CPU core)
        #[arg(long)]
        threads: Option<usize>,
        #[command(flatten)]
        det: DetectorArg,
    },
    /// List sound cards, CM108/CM119-type PTT devices and serial ports
    Devices,
    /// Key the transmitter briefly to check the PTT wiring
    PttTest {
        /// PTT method (as for `modem --ptt`)
        #[arg(long)]
        ptt: String,
        #[arg(long, default_value_t = 2.0)]
        seconds: f32,
    },
    /// Transmit a test tone to set the transmit level (FM deviation)
    Tone {
        #[arg(long)]
        audio_out: Option<String>,
        #[arg(long, default_value = "both")]
        out_channel: Channel,
        #[arg(long, default_value = "none")]
        ptt: String,
        #[arg(long, default_value_t = 1000.0)]
        freq: f32,
        /// Amplitude, 0..1 (1.0 = full scale = the modem's peak level)
        #[arg(long, default_value_t = 1.0)]
        level: f32,
        #[arg(long, default_value_t = 10.0)]
        seconds: f32,
    },
    /// Show the receive level and decode any VardeFM bursts heard
    Monitor {
        #[arg(long)]
        audio_in: Option<String>,
        #[arg(long, default_value = "left")]
        in_channel: Channel,
        #[arg(long, default_value = "narrow")]
        mode: String,
        #[command(flatten)]
        det: DetectorArg,
    },
    /// Decode VardeFM bursts from a 48 kHz WAV recording
    Decode {
        #[arg(long, default_value = "narrow")]
        mode: String,
        #[arg(long = "in")]
        input: String,
        #[command(flatten)]
        det: DetectorArg,
    },
    /// Two VardeFM modems (A and B) joined by a simulated FM link, each with its
    /// own VARA-compatible TCP ports. Runs until stopped with Ctrl-C.
    SimPair {
        /// narrow (mic/speaker) or wide (9600 data port)
        #[arg(long, default_value = "narrow")]
        mode: String,
        /// Carrier-to-noise ratio in dB (omit for no noise)
        #[arg(long)]
        cnr: Option<f32>,
        /// Receiver sound-card clock error in ppm (both directions)
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        ppm: f32,
        /// Radio key-up time, seconds
        #[arg(long, default_value_t = 0.05)]
        keyup: f32,
        /// Sending radio's audio response tilt in dB/octave, -6 to 6, at the audio's own
        /// level (both directions)
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true, value_parser = radio_tilt)]
        tx_tilt: f32,
        /// Leader before each burst, seconds (must exceed --keyup)
        #[arg(long, default_value_t = 0.1)]
        txdelay: f32,
        /// Random seed for the channel noise
        #[arg(long, default_value_t = 1)]
        seed: u64,
        /// Simulated seconds per real second (0 = as fast as possible)
        #[arg(long, default_value_t = 1.0)]
        speed: f64,
        /// Address to listen on
        #[arg(long, default_value = "127.0.0.1")]
        host: String,
        /// Command port of station A (data port = +1)
        #[arg(long, default_value_t = 8300)]
        a_port: u16,
        /// Command port of station B (data port = +1)
        #[arg(long, default_value_t = 8310)]
        b_port: u16,
        /// Save every transmission as NNNN_<A|B>_tx.wav / _rx.wav in this folder
        #[arg(long)]
        record: Option<String>,
        #[command(flatten)]
        det: DetectorArg,
    },
}

fn die(msg: &str) -> ! {
    eprintln!("error: {msg}");
    std::process::exit(2)
}

struct Stats {
    bits: usize,
    errors: usize,
    lost: usize,
    snr_sum: f32,
    snr_n: usize,
}

fn run_sim(ofdm: &Ofdm, m: Modulation, nsym: usize, txdelay: f32, cfg: &FmChannelConfig, frames: usize) -> Stats {
    let mut st = Stats { bits: 0, errors: 0, lost: 0, snr_sum: 0.0, snr_n: 0 };
    for f in 0..frames {
        let bits = test_bits(ofdm, m, nsym, f);
        let tx = make_frame(ofdm, m, &bits, txdelay);
        let rx = transmit(&FmChannelConfig { seed: f as u64 + 1, ..cfg.clone() }, &tx);
        st.bits += bits.len();
        match ofdm.detect(&rx, 0).and_then(|b| ofdm.demodulate(&rx, b, nsym)) {
            Some(frame) => {
                if std::env::var("VARDEFM_DUMP").is_ok() {
                    let nv: Vec<String> = frame.noise_var.iter().map(|v| format!("{:.0}", -10.0 * v.log10())).collect();
                    eprintln!("frame {f}: per-carrier SNR dB: {}", nv.join(" "));
                }
                st.errors += count_errors(m, &frame, &bits);
                st.snr_sum += frame.snr_db;
                st.snr_n += 1;
            }
            None => {
                st.lost += 1;
                st.errors += bits.len() / 2;
            }
        }
    }
    st
}

fn raw_rate(ofdm: &Ofdm, m: Modulation, nsym: usize) -> f32 {
    let bits = (nsym * ofdm.data_carriers() * m.bits_per_symbol()) as f32;
    bits / (ofdm.frame_len(nsym) as f32 / ofdm.p.fs)
}

fn main() {
    let cli = Cli::parse();
    match cli.cmd {
        Cmd::Sim { modem, det, cnr, frames, ppm, keyup, tx_tilt } => {
            let (ofdm, m) = modem.build_rx(&det);
            let cfg = sim_channel(&modem.mode, cnr, ppm, keyup, tx_tilt);
            let st = run_sim(&ofdm, m, modem.symbols, modem.txdelay, &cfg, frames);
            println!(
                "mode={} mod={} cnr={} ppm={} tx_tilt={}  raw={:.0} bit/s",
                modem.mode,
                m.name(),
                cnr.map_or("none".into(), |c| format!("{c} dB")),
                ppm,
                tx_tilt,
                raw_rate(&ofdm, m, modem.symbols)
            );
            println!(
                "frames lost {}/{}  BER {:.2e}  avg SNR {:.1} dB",
                st.lost,
                frames,
                st.errors as f64 / st.bits as f64,
                st.snr_sum / st.snr_n.max(1) as f32
            );
        }
        Cmd::Sweep { modem, det, frames, ppm, keyup, tx_tilt } => {
            let (ofdm, _) = modem.build_rx(&det);
            let cnrs = [6.0, 8.0, 10.0, 12.0, 15.0, 20.0, 25.0, 30.0, 40.0];
            println!(
                "mode={} symbols/frame={} ppm={} tx_tilt={} frames/point={}",
                modem.mode, modem.symbols, ppm, tx_tilt, frames
            );
            print!("{:>8} {:>8}", "mod", "raw b/s");
            for c in cnrs {
                print!(" {:>8}", format!("{c}dB"));
            }
            println!();
            let mut snr_row = Vec::new();
            for m in Modulation::ALL {
                print!("{:>8} {:>8.0}", m.name(), raw_rate(&ofdm, m, modem.symbols));
                for (ci, &c) in cnrs.iter().enumerate() {
                    let cfg = sim_channel(&modem.mode, Some(c), ppm, keyup, tx_tilt);
                    let st = run_sim(&ofdm, m, modem.symbols, modem.txdelay, &cfg, frames);
                    print!(" {:>8.1e}", st.errors as f64 / st.bits as f64);
                    if m == Modulation::Qpsk {
                        snr_row.push((ci, st.snr_sum / st.snr_n.max(1) as f32));
                    }
                }
                println!();
            }
            print!("{:>8} {:>8}", "SNR dB", "");
            for (_, s) in snr_row {
                print!(" {:>8.1}", s);
            }
            println!();
        }
        Cmd::TxWav { modem, frames, out } => {
            let (ofdm, m) = modem.build();
            let gap = vec![0f32; (ofdm.p.fs * 0.5) as usize];
            let mut audio = gap.clone();
            for f in 0..frames {
                audio.extend(make_frame(&ofdm, m, &test_bits(&ofdm, m, modem.symbols, f), modem.txdelay));
                audio.extend_from_slice(&gap);
            }
            write_wav(&out, ofdm.p.fs as u32, &audio).unwrap_or_else(|e| die(&e.to_string()));
            println!("wrote {frames} frames to {out}");
        }
        Cmd::Modem {
            mode,
            audio_in,
            audio_out,
            in_channel,
            out_channel,
            ptt,
            txdelay,
            txtail,
            tx_gain,
            host,
            port,
            record,
            max_level,
            quiet,
            levels,
            det,
        } => {
            let ptt = ptt.parse().unwrap_or_else(|m: String| die(&m));
            let cfg = vardefm_tnc::radio::RadioConfig {
                mode,
                host,
                port,
                audio_in,
                audio_out,
                in_channel,
                out_channel,
                ptt,
                txdelay_s: txdelay,
                txtail_s: txtail,
                tx_gain,
                record: record.map(Into::into),
                max_level,
                detector: det.detector,
                verbose: !quiet,
                show_levels: levels,
            };
            let stop = radio_cmds::stop_flag();
            if let Err(err) = vardefm_tnc::radio::run(cfg, stop) {
                die(&err.to_string());
            }
        }
        Cmd::Bench { mode, threads, det } => {
            let modes: &[&str] = match mode.as_str() {
                "both" => &["narrow", "wide"],
                "narrow" => &["narrow"],
                "wide" => &["wide"],
                _ => die("--mode must be narrow, wide or both"),
            };
            bench::bench(modes, threads, det.detector).unwrap_or_else(|m| die(&m));
        }
        Cmd::Devices => radio_cmds::devices().unwrap_or_else(|m| die(&m)),
        Cmd::PttTest { ptt, seconds } => radio_cmds::ptt_test(&ptt, seconds).unwrap_or_else(|m| die(&m)),
        Cmd::Tone { audio_out, out_channel, ptt, freq, level, seconds } => {
            radio_cmds::tone(audio_out.as_deref(), out_channel, &ptt, freq, level, seconds).unwrap_or_else(|m| die(&m))
        }
        Cmd::Monitor { audio_in, in_channel, mode, det } => {
            radio_cmds::monitor(audio_in.as_deref(), in_channel, &mode, det.detector).unwrap_or_else(|m| die(&m))
        }
        Cmd::Decode { mode, input, det } => radio_cmds::decode(&input, &mode, det.detector).unwrap_or_else(|m| die(&m)),
        Cmd::SimPair { mode, cnr, ppm, keyup, tx_tilt, txdelay, seed, speed, host, a_port, b_port, record, det } => {
            if keyup >= txdelay {
                die("--txdelay must be longer than --keyup");
            }
            let cfg = SimPairConfig {
                mode,
                cnr_db: cnr,
                ppm,
                keyup_s: keyup,
                tx_tilt,
                txdelay_s: txdelay,
                seed,
                speed,
                host,
                a_port,
                b_port,
                record: record.map(Into::into),
                detector: det.detector,
                verbose: true,
            };
            let sp = SimPair::start(cfg.clone()).unwrap_or_else(|e| die(&e.to_string()));
            println!(
                "sim-pair: {} mode, CNR {}, {} ppm, key-up {} s, speed {}x, detector {}",
                cfg.mode,
                cnr.map_or("none".into(), |c| format!("{c} dB")),
                ppm,
                keyup,
                speed,
                cfg.detector
            );
            for (name, (c, d)) in ["A", "B"].iter().zip(sp.addrs) {
                println!("  station {name}: command {c}, data {d}");
            }
            println!("Ctrl-C to stop.");
            sp.wait();
        }
        Cmd::RxWav { modem, det, input } => {
            let (ofdm, m) = modem.build_rx(&det);
            let (fs, rx) = read_wav(&input).unwrap_or_else(|e| die(&e.to_string()));
            if fs != ofdm.p.fs as u32 {
                die("WAV must be 48 kHz");
            }
            let frames = decode_all(&ofdm, &rx, modem.symbols);
            for (f, (b, frame)) in frames.iter().enumerate() {
                let bits = test_bits(&ofdm, m, modem.symbols, f);
                let e = count_errors(m, frame, &bits);
                println!(
                    "frame {f} at {:.2}s: BER {:.2e}  SNR {:.1} dB",
                    *b as f32 / ofdm.p.fs,
                    e as f64 / bits.len() as f64,
                    frame.snr_db
                );
            }
            println!("{} frames decoded", frames.len());
        }
    }
}

/// Parses a count that must be at least 1.
/// The simulated FM channel of `mode` for the sim and sweep commands.
fn sim_channel(mode: &str, cnr: Option<f32>, ppm: f32, keyup: f32, tx_tilt: f32) -> FmChannelConfig {
    FmChannelConfig {
        cnr_db: cnr,
        clock_offset_ppm: ppm,
        keyup_delay_s: keyup,
        tx_tilt_db_per_octave: tx_tilt,
        ..FmChannelConfig::by_name(mode).unwrap_or_else(|| die("unknown mode"))
    }
}

/// A radio tilt in dB/octave: the simulator models -6 to 6.
fn radio_tilt(s: &str) -> Result<f32, String> {
    match s.parse::<f32>() {
        Ok(t) if t.is_finite() && t.abs() <= 6.0 => Ok(t),
        Ok(_) => Err("must be within -6..6 dB/octave".into()),
        Err(e) => Err(e.to_string()),
    }
}

fn at_least_one(s: &str) -> Result<usize, String> {
    match s.parse::<usize>() {
        Ok(n) if n >= 1 => Ok(n),
        Ok(_) => Err("must be at least 1".into()),
        Err(e) => Err(e.to_string()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tx_tilt_reaches_the_simulated_channel() {
        let sim = Cli::try_parse_from(["vardefm", "sim", "--tx-tilt", "-3"]).unwrap();
        let Cmd::Sim { modem, tx_tilt, .. } = sim.cmd else { panic!("not sim") };
        assert_eq!(sim_channel(&modem.mode, None, 0.0, 0.05, tx_tilt).tx_tilt_db_per_octave, -3.0);
        let sweep = Cli::try_parse_from(["vardefm", "sweep", "--tx-tilt", "4.5"]).unwrap();
        let Cmd::Sweep { modem, tx_tilt, .. } = sweep.cmd else { panic!("not sweep") };
        assert_eq!(sim_channel(&modem.mode, Some(10.0), 0.0, 0.05, tx_tilt).tx_tilt_db_per_octave, 4.5);
        let pair = Cli::try_parse_from(["vardefm", "sim-pair", "--tx-tilt", "6"]).unwrap();
        assert!(matches!(pair.cmd, Cmd::SimPair { tx_tilt, .. } if tx_tilt == 6.0));
    }

    #[test]
    fn tx_tilt_outside_the_modelled_range_is_refused() {
        for t in ["7", "-6.5", "nan", "inf"] {
            assert!(Cli::try_parse_from(["vardefm", "sim", "--tx-tilt", t]).is_err(), "{t}");
        }
    }
}
