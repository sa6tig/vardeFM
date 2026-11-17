//! Commands for real radios: device listing, PTT test, tone, monitor, decode, modem.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use vardefm_audio::{open_input, open_output, Channel, RATE};
use vardefm_modem::{Modem, OfdmParams, Receiver, RxBurst, LEVELS};
use vardefm_ptt::PttSpec;
use vardefm_tnc::level::{meter_line, LevelMeter, Verdict};

type Res = Result<(), String>;

fn e(x: impl std::fmt::Display) -> String {
    x.to_string()
}

/// Ctrl-C sets the returned flag, so loops can stop and release PTT.
pub fn stop_flag() -> Arc<AtomicBool> {
    let stop = Arc::new(AtomicBool::new(false));
    let s = stop.clone();
    let _ = ctrlc::set_handler(move || s.store(true, Ordering::Relaxed));
    stop
}

pub fn devices() -> Res {
    let (ins, outs) = vardefm_audio::list_devices().map_err(e)?;
    println!("Audio input devices (--audio-in, any unique part of the name):");
    for d in ins {
        println!("  {d}");
    }
    println!("Audio output devices (--audio-out):");
    for d in outs {
        println!("  {d}");
    }
    println!("CM108/CM119-type PTT devices (--ptt cm108[:<gpio>][@<path>]):");
    match vardefm_ptt::find_cm108() {
        Ok(v) if v.is_empty() => println!("  none found"),
        Ok(v) => {
            for d in v {
                println!("  {:04x}:{:04x} {} {}  path: {}", d.vid, d.pid, d.manufacturer, d.product, d.path);
            }
        }
        Err(err) => println!("  cannot list HID devices: {err}"),
    }
    if cfg!(target_os = "linux") {
        println!("GPIO pins: --ptt gpio:<n> keys BCM pin GPIO<n> (e.g. gpio:17 = Raspberry Pi header pin 11)");
    }
    println!("Serial ports (--ptt rts:<port> or dtr:<port>):");
    let ports = vardefm_ptt::serial_ports();
    if ports.is_empty() {
        println!("  none found");
    }
    for p in ports {
        println!("  {p}");
    }
    Ok(())
}

pub fn ptt_test(spec: &str, seconds: f32) -> Res {
    let spec: PttSpec = spec.parse()?;
    if spec == PttSpec::None {
        return Err("with --ptt vox nothing is keyed by VardeFM: the interface keys the radio when it hears \
                    audio. Test it with `vardefm tone --audio-out <device> --seconds 2`."
            .into());
    }
    let stop = stop_flag();
    let mut ptt = vardefm_ptt::open(&spec).map_err(e)?;
    println!("Keying {} for {seconds} s (Ctrl-C to stop early)...", ptt.describe());
    ptt.set(true).map_err(e)?;
    let start = Instant::now();
    while start.elapsed().as_secs_f32() < seconds && !stop.load(Ordering::Relaxed) {
        std::thread::sleep(Duration::from_millis(20));
    }
    ptt.set(false).map_err(e)?;
    println!("Released.");
    Ok(())
}

pub fn tone(audio_out: Option<&str>, channel: Channel, spec: &str, freq: f32, level: f32, seconds: f32) -> Res {
    let spec: PttSpec = spec.parse()?;
    let stop = stop_flag();
    let mut ptt = vardefm_ptt::open(&spec).map_err(e)?;
    let mut out = open_output(audio_out, channel).map_err(e)?;
    println!(
        "Tone {freq} Hz at {:.1} dBFS on '{}' for {seconds} s, PTT {}",
        20.0 * level.log10(),
        out.name,
        ptt.describe()
    );
    println!("Set the sound card or radio level so this tone gives full FM deviation:");
    println!("about 2.5 kHz on the mic input (narrow) or 3 kHz on the data port (wide).");
    let n = (seconds * RATE as f32) as usize;
    let audio: Vec<f32> =
        (0..n).map(|i| level * (2.0 * std::f32::consts::PI * freq * i as f32 / RATE as f32).sin()).collect();
    ptt.set(true).map_err(e)?;
    let end = out.play(&audio);
    while out.consumed() < end && !stop.load(Ordering::Relaxed) {
        std::thread::sleep(Duration::from_millis(20));
    }
    out.flush();
    std::thread::sleep(Duration::from_millis(50));
    ptt.set(false).map_err(e)?;
    println!("Done.");
    Ok(())
}

fn print_burst(t: f64, b: &RxBurst) {
    let h = b.header;
    let ok = b.payload.iter().filter(|p| p.is_some()).count();
    let level = if h.ncw > 0 { LEVELS[h.level as usize].name } else { "-" };
    println!(
        "{t:8.2}s burst {:<10} session {:02x} {:<9} cw {ok}/{}  SNR {:.1} dB  peak {:.1} dBFS ({})",
        format!("{:?}", h.kind),
        h.session,
        level,
        h.ncw,
        b.snr_db,
        b.level.peak_dbfs,
        Verdict::of(b.level).advice()
    );
}

pub fn monitor(audio_in: Option<&str>, channel: Channel, mode: &str) -> Res {
    let params = OfdmParams::by_name(mode).ok_or("mode must be narrow or wide")?;
    let modem = Arc::new(Modem::new(params, 0.1));
    let mut rx = Receiver::new(modem);
    let input = open_input(audio_in, channel).map_err(e)?;
    let stop = stop_flag();
    println!("Listening on '{}' ({} Hz). Ctrl-C to stop.", input.name, input.device_rate);
    println!("Aim for received modem audio peaks around -6 dBFS, never at 0 dBFS (clipping).");
    let mut meter = LevelMeter::new(RATE as usize / 2);
    let mut pos = 0u64;
    while !stop.load(Ordering::Relaxed) {
        let Ok(chunk) = input.rx.recv_timeout(Duration::from_millis(200)) else { continue };
        let chunk = chunk.samples;
        pos += chunk.len() as u64;
        for b in rx.push(&chunk) {
            print_burst(pos as f64 / RATE as f64, &b);
        }
        if let Some(level) = meter.push(&chunk) {
            println!("{:8.2}s {}", pos as f64 / RATE as f64, meter_line(level));
        }
    }
    Ok(())
}

pub fn decode(path: &str, mode: &str) -> Res {
    let params = OfdmParams::by_name(mode).ok_or("mode must be narrow or wide")?;
    let (fs, samples) = vardefm_cli::read_wav(path).map_err(e)?;
    if fs != RATE {
        return Err(format!("{path} is {fs} Hz; it must be 48000 Hz"));
    }
    let mut rx = Receiver::new(Arc::new(Modem::new(params, 0.1)));
    let mut count = 0;
    for chunk in samples.chunks(480).chain(std::iter::once(&vec![0.0; RATE as usize][..])) {
        for b in rx.push(chunk) {
            print_burst(b.end_sample as f64 / RATE as f64, &b);
            count += 1;
        }
    }
    println!("{count} bursts decoded");
    Ok(())
}
