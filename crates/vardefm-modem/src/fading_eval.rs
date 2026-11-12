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
use crate::detect_eval::{env_list, env_modes, env_or};
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

/// Paired measurement of the per-carrier noise profile (`Frame::carrier_noise_scale`)
/// on fading channels: each burst decoded with the profile and with it cleared.
/// Prints codewords decoded each way and bursts helped / hurt.
///
/// `cargo test --release -p vardefm-modem click_profile_gain -- --ignored --nocapture`
///
/// Environment: `MODES` (narrow,wide), `LEVEL` (6), `TRIALS` bursts per cell (48),
/// `FADINGS` (rayleigh:20,rayleigh:80), `CNRS` (dB list, 33,40,60).
#[test]
#[ignore]
fn click_profile_gain() {
    let modes = env_modes("narrow,wide");
    let level: u8 = env_or("LEVEL", 6);
    assert!((level as usize) < LEVELS.len(), "LEVEL {level}: levels are 0 to {}", LEVELS.len() - 1);
    let trials: u32 = env_or("TRIALS", 48);
    let fadings: Vec<Fading> = env_list("FADINGS", "rayleigh:20,rayleigh:80");
    let cnrs: Vec<f32> = env_list("CNRS", "33,40,60");
    let ncw = 4;
    for mode in modes.iter().map(String::as_str) {
        let m = modem(mode);
        let nsym = m.header_symbols() + ncw * m.symbols_per_codeword(level);
        for &f in &fadings {
            for &cnr in &cnrs {
                let (mut with, mut without, mut helped, mut hurt, mut total) = (0, 0, 0, 0, 0);
                for t in 0..trials {
                    let b = burst(level, ncw, 100 + t);
                    let cfg = FmChannelConfig {
                        cnr_db: Some(cnr),
                        fading: Some(f),
                        seed: t as u64 + 1,
                        ..FmChannelConfig::by_name(mode).unwrap()
                    };
                    let audio = transmit(&cfg, &m.modulate(&b));
                    let Some(start) = m.ofdm.detect(&audio, 0) else { continue };
                    let known = m.header_carriers(&b.header);
                    let Some(mut frame) = m.ofdm.demodulate_known(&audio, start, nsym, &known) else { continue };
                    let ok = |frame: &Frame| -> usize {
                        m.decode_payload(&b.header, frame)
                            .iter()
                            .zip(&b.payload)
                            .filter(|(got, want)| got.as_ref() == Some(*want))
                            .count()
                    };
                    let a = ok(&frame);
                    frame.carrier_noise_scale.iter_mut().for_each(Vec::clear);
                    let c = ok(&frame);
                    with += a;
                    without += c;
                    helped += (a > c) as u32;
                    hurt += (a < c) as u32;
                    total += ncw;
                }
                println!(
                    "{mode} L{level} {f} {cnr} dB: codewords {with}/{total} with the profile, {without}/{total} without; bursts helped {helped}, hurt {hurt}"
                );
            }
        }
    }
}
