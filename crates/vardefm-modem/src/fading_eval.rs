//! Fading diagnostics.
//!
//! `fading_diag` compares, per burst, the genie audio SNR (received audio minus the
//! same burst through a clean channel; time-domain mean and median per symbol) with
//! the receiver's reported SNR. It prints only. The detection side of fading (bursts
//! lost because preamble A sits in a fade) is measured paired, detector against
//! detector, with `FADING=rayleigh:20 ... detector_eval` in `detect_eval`.
//!
//! `cargo test --release -p vardefm-modem fading_diag -- --ignored --nocapture`
//!
//! Environment: `MODES` (narrow,wide), `LEVEL` (6).

use super::*;
use crate::detect_eval::{env_modes, env_or};
use crate::tests::{burst, modem, receive};
use vardefm_sim::{transmit, Fading, FmChannelConfig};

#[test]
#[ignore]
fn fading_diag() {
    let modes = env_modes("narrow,wide");
    let level: u8 = env_or("LEVEL", 6);
    assert!((level as usize) < LEVELS.len(), "LEVEL {level}: levels are 0 to {}", LEVELS.len() - 1);
    for mode in modes.iter().map(String::as_str) {
        let m = modem(mode);
        for dop in [5.0f32, 20.0, 80.0] {
            for cnr in [24.0f32, 60.0] {
                let mut line = format!("{mode} dop {dop} cnr {cnr}:");
                for t in 0..4u32 {
                    let b = burst(level, m.max_ncw(level).min(4), 100 + t);
                    let audio = m.modulate(&b);
                    let base = FmChannelConfig { seed: t as u64 + 1, ..FmChannelConfig::by_name(mode).unwrap() };
                    let clean = transmit(&FmChannelConfig { cnr_db: None, ..base.clone() }, &audio);
                    let rx = transmit(
                        &FmChannelConfig { cnr_db: Some(cnr), fading: Some(Fading::rayleigh(dop)), ..base.clone() },
                        &audio,
                    );
                    let l = m.ofdm.p.symbol_len();
                    let lead = (base.lead_silence_s * m.fs()) as usize + (base.keyup_delay_s * m.fs()) as usize + l;
                    let end = (base.lead_silence_s * m.fs()) as usize + audio.len();
                    let mut ratios: Vec<f32> = clean[lead..end]
                        .chunks_exact(l)
                        .zip(rx[lead..end].chunks_exact(l))
                        .filter_map(|(c, r)| {
                            let s: f32 = c.iter().map(|x| x * x).sum();
                            let e: f32 = c.iter().zip(r).map(|(a, b)| (a - b).powi(2)).sum();
                            (s > 1e-9).then_some(e / s)
                        })
                        .collect();
                    assert!(!ratios.is_empty(), "{mode}: no whole symbol to compare");
                    let mean = ratios.iter().sum::<f32>() / ratios.len() as f32;
                    ratios.sort_by(f32::total_cmp);
                    let med = ratios[ratios.len() / 2];
                    let got = receive(&mut Receiver::new(m.clone()), &rx);
                    let (rep, ok) = got
                        .first()
                        .map_or((f32::NAN, 0), |g| (g.snr_db, g.payload.iter().filter(|p| p.is_some()).count()));
                    line += &format!(
                        " [genie mean {:.1} med {:.1} rep {:.1} ok {}/{}]",
                        -10.0 * mean.log10(),
                        -10.0 * med.log10(),
                        rep,
                        ok,
                        b.payload.len()
                    );
                }
                println!("{line}");
            }
        }
    }
}

/// Noise-model calibration: per quarter of the band, the measured noise of the
/// equalized payload cells (payload known) over the noise the receiver predicts for
/// them (`noise_var[j] * symbol_noise_scale[i]`), summed over bursts. 1 is calibrated;
/// below 1 the receiver overpredicts. Symbols with scale 1 ("clean") and above 1
/// ("click") are counted apart.
///
/// `cargo test --release -p vardefm-modem noise_calib -- --ignored --nocapture`
///
/// Environment: `MODES` (narrow,wide), `LEVEL` (6), `CNR` (60), `BURSTS` (12), `PER`
/// (set: also print the clean ratio and the measured noise per data carrier).
#[test]
#[ignore]
fn noise_calib() {
    let modes = env_modes("narrow,wide");
    let level: u8 = env_or("LEVEL", 6);
    let cnr: f32 = env_or("CNR", 60.0);
    let bursts: u32 = env_or("BURSTS", 12);
    for mode in modes.iter().map(String::as_str) {
        let m = modem(mode);
        for fading in [None, Some(Fading::rayleigh(20.0)), Some(Fading::rayleigh(80.0))] {
            let nd = m.ofdm.data_carriers();
            // [clean, click][quarter] = (measured, predicted, cells)
            let mut acc = [[(0f64, 0f64, 0usize); 4]; 2];
            let mut per = vec![(0f64, 0f64); nd];
            let mut nclick = 0;
            let mut nsyms = 0;
            for t in 0..bursts {
                let b = burst(level, m.max_ncw(level).min(4), 100 + t);
                let audio = m.modulate(&b);
                let cfg = FmChannelConfig {
                    cnr_db: Some(cnr),
                    fading,
                    seed: t as u64 + 1,
                    ..FmChannelConfig::by_name(mode).unwrap()
                };
                let rx = transmit(&cfg, &audio);
                let Some(start) = m.ofdm.detect(&rx, 0) else { continue };
                let sent = m.symbols(&b);
                let hs = m.header_symbols();
                let Some(f) = m.ofdm.demodulate_known(&rx, start, sent.len(), &sent[..hs]) else { continue };
                for ((y, x), &scale) in f.symbols.iter().zip(&sent).zip(&f.symbol_noise_scale).skip(hs) {
                    let c = usize::from(scale > 1.0);
                    nclick += c;
                    nsyms += 1;
                    for j in 0..nd {
                        let err = (y[j] - x[j]).norm_sqr() as f64;
                        let q = &mut acc[c][j * 4 / nd];
                        q.0 += err;
                        q.1 += (f.noise_var[j] * scale) as f64;
                        q.2 += 1;
                        if c == 0 {
                            per[j].0 += err;
                            per[j].1 += f.noise_var[j] as f64;
                        }
                    }
                }
            }
            let fname = fading.map_or("static".into(), |f| f.to_string());
            let fmt = |a: &[(f64, f64, usize); 4]| {
                a.iter()
                    .map(|q| if q.2 > 0 { format!("{:.2}", q.0 / q.1) } else { "-".into() })
                    .collect::<Vec<_>>()
                    .join(" / ")
            };
            if crate::detect_eval::env_flag("PER") {
                let n0 = per.iter().map(|p| p.0).sum::<f64>() / nd as f64;
                println!(
                    "  per-carrier clean ratio: {}",
                    per.iter().map(|p| format!("{:.2}", p.0 / p.1)).collect::<Vec<_>>().join(" ")
                );
                println!(
                    "  measured rel. mean:      {}",
                    per.iter().map(|p| format!("{:.2}", p.0 / n0)).collect::<Vec<_>>().join(" ")
                );
            }
            println!(
                "{mode} {fname} L{level} {cnr} dB: clean {} | click ({nclick}/{nsyms} syms) {}",
                fmt(&acc[0]),
                fmt(&acc[1])
            );
        }
    }
}

/// Codeword errors on fading channels, decoded from the true burst position with the
/// sent header as training (as `level_eval` does on the static channel). Trial `t`
/// uses the same payload and channel seed in every run, so two runs of different
/// receivers are paired trial by trial. With `OUT=file` each point's per-trial error
/// counts are written there, one line per point.
///
/// `cargo test --release -p vardefm-modem fading_cer -- --ignored --nocapture`
///
/// Environment: `MODES` (narrow,wide), `FADINGS` (rayleigh:5,rayleigh:20,rayleigh:80),
/// `LEVELS` (0,1,..,6), `CNRS` (dB, mean CNR: 12,14,..,44), `TRIALS` (bursts per point,
/// 100), `OUT`. The defaults are the sweep of #19 (about 2 minutes on 20 cores).
#[test]
#[ignore]
fn fading_cer() {
    use crate::detect_eval::{env_list, on_every_core, payload_seed, through_channel_with, true_position};
    use std::io::Write;
    let modes = env_modes("narrow,wide");
    let fadings: Vec<Fading> = env_list("FADINGS", "rayleigh:5,rayleigh:20,rayleigh:80");
    let levels: Vec<u8> = env_list("LEVELS", "0,1,2,3,4,5,6");
    let cnrs: Vec<f32> = env_list("CNRS", "12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44");
    let trials: u32 = env_or("TRIALS", 100);
    let mut out = std::env::var("OUT").ok().map(|p| std::fs::File::create(p).unwrap());
    for mode in modes.iter().map(String::as_str) {
        let m = crate::detect_eval::modem(mode);
        for &f in &fadings {
            for &level in &levels {
                let ncw = m.max_ncw(level).min(4);
                let truth = true_position(&m, mode, level, ncw);
                let mut line = format!("{mode} {f} L{level}:");
                for &cnr in &cnrs {
                    let errs = on_every_core(trials, |t| {
                        let sent = crate::detect_eval::burst(level, ncw, payload_seed(t));
                        let rx = through_channel_with(&m, mode, cnr, Some(f), &sent, t);
                        let nsym = m.header_symbols() + ncw * m.symbols_per_codeword(level);
                        let training = m.header_carriers(&sent.header);
                        let Some(frame) = m.ofdm.demodulate_known(&rx, truth, nsym, &training) else {
                            return ncw as u32;
                        };
                        let got = m.decode_payload(&sent.header, &frame);
                        got.iter().zip(&sent.payload).filter(|(g, w)| g.as_ref() != Some(*w)).count() as u32
                    });
                    let total: u32 = errs.iter().sum();
                    line += &format!(" {cnr}:{total}/{}", trials as usize * ncw);
                    if let Some(o) = out.as_mut() {
                        let v: Vec<String> = errs.iter().map(u32::to_string).collect();
                        writeln!(o, "{mode} {f} L{level} {cnr} {}", v.join(",")).unwrap();
                    }
                }
                println!("{line}");
            }
        }
    }
}
