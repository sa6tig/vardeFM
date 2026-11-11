use super::*;
use std::sync::Arc;
use vardefm_dsp::prng::Prng;
use vardefm_sim::{transmit, Fading, FmChannelConfig};

fn modem(mode: &str) -> Arc<Modem> {
    Arc::new(Modem::new(OfdmParams::by_name(mode).unwrap(), 0.1))
}

fn burst(level: u8, ncw: usize, seed: u32) -> Burst {
    let mut rng = Prng::new(seed);
    let n = Modem::payload_bytes(level);
    Burst {
        header: Header {
            kind: Kind::Data,
            session: 0x42,
            level,
            ncw: ncw as u8,
            req_level: 2,
            burst_id: 3,
            ack_burst: 5,
            ack_offset: 12345,
            bitmap: 0b1011,
            flags: 0,
        },
        payload: (0..ncw).map(|_| (0..n).map(|_| rng.next_u32() as u8).collect()).collect(),
    }
}

/// Push audio in 10 ms chunks, like a sound card would deliver it.
fn receive(rx: &mut Receiver, audio: &[f32]) -> Vec<RxBurst> {
    let mut out = Vec::new();
    for chunk in audio.chunks(480) {
        out.extend(rx.push(chunk));
    }
    out.extend(rx.push(&vec![0.0; 48_000]));
    out
}

#[test]
fn every_level_roundtrips_clean() {
    for mode in ["narrow", "wide"] {
        let m = modem(mode);
        for level in 0..LEVELS.len() as u8 {
            let b = burst(level, 2, level as u32 + 1);
            let mut audio = vec![0.0; 5000];
            audio.extend(m.modulate(&b));
            assert_eq!(audio.len() - 5000, m.burst_samples(level, 2));
            let got = receive(&mut Receiver::new(m.clone()), &audio);
            assert_eq!(got.len(), 1, "{mode} level {level}");
            assert_eq!(got[0].header, b.header);
            let payload: Vec<Vec<u8>> = got[0].payload.iter().map(|p| p.clone().expect("codeword failed")).collect();
            assert_eq!(payload, b.payload, "{mode} level {level}");
        }
    }
}

#[test]
fn header_only_and_back_to_back_bursts() {
    let m = modem("narrow");
    let mut ack = burst(0, 0, 1);
    ack.header.kind = Kind::DiscAck;
    let data = burst(3, 3, 2);
    let mut audio = vec![0.0; 2000];
    audio.extend(m.modulate(&ack));
    audio.extend(vec![0.0; 7000]);
    audio.extend(m.modulate(&data));
    let got = receive(&mut Receiver::new(m.clone()), &audio);
    assert_eq!(got.len(), 2);
    assert_eq!(got[0].header, ack.header);
    assert!(got[0].payload.is_empty());
    assert_eq!(got[1].header, data.header);
    assert!(got[1].payload.iter().all(|p| p.is_some()));
}

#[test]
fn decodes_through_noisy_radio() {
    // (mode, level, CNR): all codewords must decode.
    for (mode, level, cnr) in [("narrow", 1, 12.0), ("narrow", 4, 22.0), ("wide", 1, 18.0), ("wide", 4, 30.0)] {
        let m = modem(mode);
        let b = burst(level, m.max_ncw(level).min(6), 7);
        let cfg =
            FmChannelConfig { cnr_db: Some(cnr), clock_offset_ppm: 70.0, ..FmChannelConfig::by_name(mode).unwrap() };
        let rx_audio = transmit(&cfg, &m.modulate(&b));
        let got = receive(&mut Receiver::new(m.clone()), &rx_audio);
        assert_eq!(got.len(), 1, "{mode} level {level}");
        let ok = got[0].payload.iter().filter(|p| p.is_some()).count();
        assert_eq!(ok, b.payload.len(), "{mode} level {level} @ {cnr} dB, SNR {:.1}", got[0].snr_db);
    }
}

/// Prints, per level, the codeword success rate and estimated SNR across CNR.
/// Used to set the rate-adaptation thresholds in vardefm-link.
/// `cargo test --release -p vardefm-modem level_table -- --ignored --nocapture`
#[test]
#[ignore]
fn level_table() {
    for mode in ["narrow", "wide"] {
        print_level_table(mode, None, &[6.0, 8.0, 10.0, 12.0, 14.0, 17.0, 20.0, 24.0, 28.0, 33.0, 40.0], 4);
    }
}

/// The level table on Rayleigh fading channels (mobile or portable "picket fencing"),
/// CNR being the mean CNR. Compare with `level_table`.
/// `cargo test --release -p vardefm-modem fading_table -- --ignored --nocapture`
/// `VARDEFM_FADING=rician:20:6` runs one fading spec instead of Rayleigh at 5, 20 and 80 Hz.
#[test]
#[ignore]
fn fading_table() {
    let specs: Vec<Fading> = match std::env::var("VARDEFM_FADING") {
        Ok(s) => vec![s.parse().unwrap()],
        Err(_) => [5.0, 20.0, 80.0].map(Fading::rayleigh).to_vec(),
    };
    let cnrs = [10.0, 14.0, 17.0, 20.0, 24.0, 28.0, 33.0, 40.0, 50.0, 60.0];
    for f in specs {
        for mode in ["narrow", "wide"] {
            print_level_table(mode, Some(f), &cnrs, 8);
        }
    }
}

/// One row per level: success rate / mean SNR per CNR, then the lowest CNR from which
/// every burst decoded fully, and the highest reported SNR of any burst with a failed
/// codeword (what `SNR_THRESHOLDS_DB` in vardefm-link must stay above).
fn print_level_table(mode: &str, fading: Option<Fading>, cnrs: &[f32], trials: u32) {
    let m = modem(mode);
    let fname = fading.map_or("static".into(), |f| f.to_string());
    println!("{mode} {fname}: net rates {:?}", (0..7).map(|l| m.net_rate(l) as u32).collect::<Vec<_>>());
    let rows: Vec<String> = std::thread::scope(|scope| {
        let handles: Vec<_> = (0..LEVELS.len() as u8)
            .map(|level| {
                let m = m.clone();
                scope.spawn(move || {
                    let mut line = format!("{mode} L{level} {:>10}:", LEVELS[level as usize].name);
                    let mut full_from = None;
                    let mut worst_fail = f32::NEG_INFINITY;
                    for &cnr in cnrs {
                        let (mut ok, mut total, mut snr, mut nf) = (0, 0, 0.0, 0);
                        for t in 0..trials {
                            let b = burst(level, m.max_ncw(level).min(4), 100 + t);
                            let cfg = FmChannelConfig {
                                cnr_db: Some(cnr),
                                fading,
                                seed: t as u64 + 1,
                                ..FmChannelConfig::by_name(mode).unwrap()
                            };
                            let got = receive(&mut Receiver::new(m.clone()), &transmit(&cfg, &m.modulate(&b)));
                            total += b.payload.len();
                            if let Some(g) = got.first() {
                                let good = g.payload.iter().filter(|p| p.is_some()).count();
                                ok += good;
                                snr += g.snr_db;
                                nf += 1;
                                if good < g.payload.len() {
                                    worst_fail = worst_fail.max(g.snr_db);
                                }
                            }
                        }
                        if ok < total {
                            full_from = None;
                        } else if full_from.is_none() {
                            full_from = Some(cnr);
                        }
                        line +=
                            &format!(" {cnr}:{:.0}%/{:.1}", 100.0 * ok as f32 / total as f32, snr / nf.max(1) as f32);
                    }
                    line += &format!(
                        " | full from {} | worst failed SNR {}",
                        full_from.map_or("-".into(), |c| format!("{c} dB")),
                        if worst_fail.is_finite() { format!("{worst_fail:.1} dB") } else { "-".into() }
                    );
                    line
                })
            })
            .collect();
        handles.into_iter().map(|h| h.join().unwrap()).collect()
    });
    for line in rows {
        println!("{line}");
    }
}

#[test]
fn threaded_decoding_matches_single_thread() {
    // A noisy link where some codewords fail: the result must not depend on the thread count.
    let params = OfdmParams::by_name("narrow").unwrap();
    let b = burst(6, 9, 11);
    let cfg = FmChannelConfig { cnr_db: Some(16.0), seed: 5, ..FmChannelConfig::by_name("narrow").unwrap() };
    let audio = transmit(&cfg, &Modem::new(params.clone(), 0.1).modulate(&b));
    let mut results = Vec::new();
    for threads in [1, 2, 4, 16] {
        let mut m = Modem::new(params.clone(), 0.1);
        m.decode_threads = threads;
        let got = receive(&mut Receiver::new(Arc::new(m)), &audio);
        assert_eq!(got.len(), 1);
        results.push(got[0].payload.clone());
    }
    let ok = results[0].iter().filter(|p| p.is_some()).count();
    assert!(ok > 0 && ok < 9, "want a mix of good and failed codewords, got {ok}/9");
    assert!(results.iter().all(|r| *r == results[0]));
}

/// Received audio of `b` through the simulator, and where its preamble B starts.
fn received(m: &Modem, mode: &str, b: &Burst, cnr: f32) -> (Vec<f32>, usize) {
    let cfg = FmChannelConfig { cnr_db: Some(cnr), ..FmChannelConfig::by_name(mode).unwrap() };
    let audio = transmit(&cfg, &m.modulate(b));
    let start = m.ofdm.detect(&audio, 0).expect("frame not detected");
    (audio, start)
}

#[test]
fn header_training_raises_snr() {
    for mode in ["narrow", "wide"] {
        let m = modem(mode);
        let b = burst(4, 2, 21);
        let (audio, start) = received(&m, mode, &b, 30.0);
        let nsym = m.header_symbols() + 2 * m.symbols_per_codeword(4);
        let plain = m.ofdm.demodulate(&audio, start, nsym).unwrap();
        let known = m.header_carriers(&b.header);
        let trained = m.ofdm.demodulate_known(&audio, start, nsym, &known).unwrap();
        assert!(
            trained.snr_db > plain.snr_db + 0.5,
            "{mode}: SNR {:.1} dB with header training, {:.1} dB without",
            trained.snr_db,
            plain.snr_db
        );
        assert!(m.decode_payload(&b.header, &trained).iter().all(|p| p.is_some()), "{mode}");
    }
}

#[test]
fn survives_a_click() {
    // An FM threshold click or a squelch pop wipes out one whole OFDM symbol. Its
    // per-symbol noise scale must keep its LLRs from swamping the decoder.
    let (mode, level, ncw) = ("narrow", 6, 4);
    let m = modem(mode);
    let b = burst(level, ncw, 31);
    let (mut audio, start) = received(&m, mode, &b, 24.0);
    let p = &m.ofdm.p;
    let l = p.symbol_len();
    let nsym = m.header_symbols() + ncw * m.symbols_per_codeword(level);
    // Data symbol i, cyclic prefix included, starts at start + (i+1)*l - ncp.
    let hit = m.header_symbols() + ncw * m.symbols_per_codeword(level) / 2;
    let from = start + (hit + 1) * l - p.ncp;
    let sig = &audio[start..start + nsym * l];
    let rms = (sig.iter().map(|x| x * x).sum::<f32>() / sig.len() as f32).sqrt();
    // Uniform noise in ±a has RMS a/sqrt(3): make it 3x the signal.
    let a = 3.0 * rms * 3f32.sqrt();
    let mut rng = Prng::new(77);
    for x in &mut audio[from..from + l] {
        *x = a * (rng.next_u32() as f32 / u32::MAX as f32 * 2.0 - 1.0);
    }

    let known = m.header_carriers(&b.header);
    let mut frame = m.ofdm.demodulate_known(&audio, start, nsym, &known).unwrap();
    let scale = frame.symbol_noise_scale.clone();
    assert!(scale[hit] > 10.0, "scales {scale:?}");
    assert!(scale.iter().enumerate().all(|(i, &s)| i == hit || s < scale[hit] / 10.0), "scales {scale:?}");
    let ok = m.decode_payload(&b.header, &frame).iter().filter(|p| p.is_some()).count();
    assert_eq!(ok, ncw);
    // Without the scale the click symbol's LLRs are far too confident.
    frame.symbol_noise_scale.iter_mut().for_each(|s| *s = 1.0);
    frame.carrier_noise_scale.iter_mut().for_each(Vec::clear);
    let ok_flat = m.decode_payload(&b.header, &frame).iter().filter(|p| p.is_some()).count();
    assert!(ok_flat < ncw, "{ok_flat} codewords decode even without the per-symbol scale");

    // And the same through the streaming receiver.
    let got = receive(&mut Receiver::new(m.clone()), &audio);
    assert_eq!(got.len(), 1);
    assert!(got[0].payload.iter().all(|p| p.is_some()));
}

#[test]
fn header_only_burst_reports_the_same_snr() {
    // Rate adaptation uses the SNR of every burst, acks included.
    for (mode, cnr) in [("narrow", 15.0), ("wide", 22.0)] {
        let m = modem(mode);
        let mut snr = Vec::new();
        for ncw in [0, 3] {
            let mut sum = 0.0;
            for seed in 1..=4 {
                let b = burst(3, ncw, seed);
                let cfg =
                    FmChannelConfig { cnr_db: Some(cnr), seed: seed as u64, ..FmChannelConfig::by_name(mode).unwrap() };
                let got = receive(&mut Receiver::new(m.clone()), &transmit(&cfg, &m.modulate(&b)));
                assert_eq!(got.len(), 1, "{mode} ncw {ncw} seed {seed}");
                sum += got[0].snr_db;
            }
            snr.push(sum / 4.0);
        }
        eprintln!("{mode}: header-only {:.2} dB, data {:.2} dB", snr[0], snr[1]);
        // A data burst's SNR includes the clipping noise of its payload symbols (the
        // per-symbol noise scale picks them up), which a header-only burst cannot see:
        // about 0.8 dB in narrow mode at the 0.30 drive, 0.3 dB at 0.20.
        assert!((snr[0] - snr[1]).abs() < 1.0, "{mode}: header-only {:.1} dB, data {:.1} dB", snr[0], snr[1]);
    }
}

/// Error vector magnitude (dB, relative to the symbol power) of the equalized header
/// symbols of `f` against the transmitted values, skipping symbol `skip`.
fn header_evm_db(f: &Frame, known: &[Vec<Complex32>], skip: usize) -> f32 {
    let (mut err, mut sig) = (0.0, 0.0);
    for (i, k) in known.iter().enumerate().filter(|(i, _)| *i != skip) {
        for (y, x) in f.symbols[i].iter().zip(k) {
            err += (y - x).norm_sqr();
            sig += x.norm_sqr();
        }
    }
    10.0 * (err / sig).log10()
}

#[test]
fn survives_a_click_in_the_header() {
    // A click on a header symbol: the header still decodes (its code is rate 0.24),
    // so the symbol is offered as channel training. Averaged in at full weight it
    // would corrupt the estimate for every carrier of the burst, which no later
    // per-symbol scale can undo. The training must leave it out.
    let (mode, level, ncw) = ("narrow", 6, 4);
    let m = modem(mode);
    let b = burst(level, ncw, 31);
    let (clean, start) = received(&m, mode, &b, 24.0);
    let p = &m.ofdm.p;
    let l = p.symbol_len();
    let nsym = m.header_symbols() + ncw * m.symbols_per_codeword(level);
    let hit = m.header_symbols() / 2;
    let from = start + (hit + 1) * l - p.ncp;
    let sig = &clean[start..start + nsym * l];
    let rms = (sig.iter().map(|x| x * x).sum::<f32>() / sig.len() as f32).sqrt();
    let a = 3.0 * rms * 3f32.sqrt();
    let mut audio = clean.clone();
    let mut rng = Prng::new(78);
    for x in &mut audio[from..from + l] {
        *x = a * (rng.next_u32() as f32 / u32::MAX as f32 * 2.0 - 1.0);
    }

    let known = m.header_carriers(&b.header);
    let reference = m.ofdm.demodulate_known(&clean, start, nsym, &known).unwrap();
    let frame = m.ofdm.demodulate_known(&audio, start, nsym, &known).unwrap();
    assert!(frame.symbol_noise_scale[hit] > 10.0, "scales {:?}", frame.symbol_noise_scale);
    let (evm_ref, evm) = (header_evm_db(&reference, &known, hit), header_evm_db(&frame, &known, hit));
    assert!(evm < evm_ref + 1.0, "header EVM {evm:.1} dB with the click, {evm_ref:.1} dB without");
    let ok = m.decode_payload(&b.header, &frame).iter().filter(|p| p.is_some()).count();
    assert_eq!(ok, ncw);

    // And through the streaming receiver, where the header is decoded first.
    let got = receive(&mut Receiver::new(m.clone()), &audio);
    assert_eq!(got.len(), 1);
    assert_eq!(got[0].header, b.header);
    assert!(got[0].payload.iter().all(|p| p.is_some()));
}

#[test]
fn burst_level_ignores_a_squelch_pop_after_the_burst() {
    let m = modem("narrow");
    let mut ack = burst(0, 0, 7);
    ack.header.kind = Kind::DiscAck;
    for b in [burst(2, 2, 6), ack] {
        // The burst at -6 dB, then a full-scale pop right after its last symbol.
        let tx: Vec<f32> = m.modulate(&b).iter().map(|x| x * 0.5).collect();
        let expected = AudioLevel::of(&tx);
        let mut audio = vec![0.0; 3000];
        audio.extend(&tx);
        audio.extend(vec![0.0; 100]);
        audio.extend(vec![1.0; 200]);
        let got = receive(&mut Receiver::new(m.clone()), &audio);
        assert_eq!(got.len(), 1);
        let level = got[0].level;
        assert!(level.peak_dbfs < -5.0, "pop counted: {level:?}");
        // The whole burst is measured (preamble A included): its peak is the burst's own.
        assert!((level.peak_dbfs - expected.peak_dbfs).abs() < 0.5, "{level:?} vs {expected:?}");
    }
}

/// `rms` sine at `f` Hz, `n` samples starting at `phase`.
fn sine(f: f32, rms: f32, n: usize, phase: f32) -> Vec<f32> {
    let w = 2.0 * std::f32::consts::PI * f / 48_000.0;
    (0..n).map(|i| rms * 2f32.sqrt() * (i as f32 * w + phase).sin()).collect()
}

/// A steady CTCSS tone (a keyed repeater) heard before and after our own transmission.
/// The receiver is fed nothing while we transmit. After it, the radio is muted until
/// it is back on receive (tens of ms for a handheld, a VOX interface's hang time of a
/// few hundred ms), and then the tone comes back abruptly, at any phase.
#[test]
fn ctcss_tone_after_our_own_transmission_is_not_busy() {
    let m = modem("wide");
    let mut seed = 1u32;
    for rms in [0.177, 0.354] {
        for muted_ms in [0, 5, 30, 100, 300, 600] {
            for jump in [0.5, 1.0, 1.5] {
                for floor in [0.0, 3e-4] {
                    let mut rx = Receiver::new(m.clone());
                    for chunk in sine(254.1, rms, 3 * 48_000, 0.0).chunks(480) {
                        rx.push(chunk);
                    }
                    assert!(!rx.busy());
                    rx.reset();
                    // The muted radio, with or without a faint noise floor from the sound card.
                    let mut audio: Vec<f32> = (0..muted_ms * 48)
                        .map(|_| {
                            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                            floor * (seed as f32 / u32::MAX as f32 * 2.0 - 1.0)
                        })
                        .collect();
                    audio.extend(sine(254.1, rms, 24_000, jump * std::f32::consts::PI));
                    for (i, chunk) in audio.chunks(480).enumerate() {
                        rx.push(chunk);
                        assert!(
                            !rx.busy(),
                            "{rms} RMS, muted {muted_ms} ms, jump {jump} pi, floor {floor}: busy after {} ms",
                            10 * (i + 1)
                        );
                    }
                }
            }
        }
    }
}

/// Busy holds for a while after the last loud audio, but not across our own
/// transmission: the channel is not busy when reception resumes just because it was
/// busy shortly before we keyed up.
#[test]
fn a_busy_hold_does_not_outlast_our_own_transmission() {
    let m = modem("narrow");
    let mut rx = Receiver::new(m.clone());
    rx.push(&vec![0.0; 48_000]);
    for chunk in sine(993.0, 0.1, 4800, 0.0).chunks(480) {
        rx.push(chunk);
    }
    rx.push(&vec![0.0; 4800]);
    assert!(rx.busy(), "the hold runs 100 ms after the signal");
    rx.reset();
    rx.push(&[0.0; 48]);
    assert!(!rx.busy(), "busy after our transmission");
}

/// A NaN, an Inf or a sample far beyond full scale from the sound card in a steady CTCSS tone, at its peak, up to
/// 1500 Hz deviation on the wide port.
#[test]
fn a_non_finite_sample_in_a_ctcss_tone_is_not_busy() {
    let m = modem("wide");
    for rms in [0.177, 0.354] {
        for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, f32::MAX, -1e30, 5.0] {
            let mut audio = sine(254.1, rms, 4 * 48_000, 0.0);
            let peak = (3 * 48_000..3 * 48_000 + 200).max_by(|&a, &b| audio[a].total_cmp(&audio[b])).unwrap();
            audio[peak] = bad;
            let mut rx = Receiver::new(m.clone());
            for (i, chunk) in audio.chunks(480).enumerate() {
                rx.push(chunk);
                assert!(!rx.busy(), "{rms} RMS, {bad}: busy after {} ms", 10 * (i + 1));
            }
        }
    }
}

/// A NaN, an Inf or a sample far beyond full scale from the sound card before or inside a burst.
#[test]
fn a_non_finite_sample_does_not_hide_a_burst() {
    for mode in ["narrow", "wide"] {
        let m = modem(mode);
        let b = burst(2, 2, 9);
        for bad in [f32::NAN, f32::INFINITY, f32::MAX] {
            // From 1000 samples before the burst's leader to well into its payload.
            for at in (19_000..31_000).step_by(1000) {
                let mut audio = vec![0.0; 20_000];
                audio.extend(m.modulate(&b));
                audio[at] = bad;
                let got = receive(&mut Receiver::new(m.clone()), &audio);
                assert_eq!(got.len(), 1, "{mode}, {bad} at {at}");
                assert!(got[0].payload.iter().all(|p| p.is_some()), "{mode}, {bad} at {at}");
            }
        }
    }
}

/// Diagnostic: how much of the fading impairment is in the audio itself (genie SNR:
/// received audio minus the same burst through a clean channel), and how impulsive.
#[test]
#[ignore]
fn fading_diag() {
    for mode in ["narrow", "wide"] {
        let m = modem(mode);
        let level = 6;
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
                        .chunks(l)
                        .zip(rx[lead..end].chunks(l))
                        .map(|(c, r)| {
                            let s: f32 = c.iter().map(|x| x * x).sum();
                            let e: f32 = c.iter().zip(r).map(|(a, b)| (a - b).powi(2)).sum();
                            e / s
                        })
                        .collect();
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

/// Diagnostic: for bursts lost on fading channels, which stage failed: detection
/// (Schmidl-Cox plateau), fine timing (preamble B) or the header.
#[test]
#[ignore]
fn fading_loss_stages() {
    let trials: u32 = std::env::var("TRIALS").ok().and_then(|s| s.parse().ok()).unwrap_or(64);
    let only_mode = std::env::var("MODE").ok();
    for mode in ["narrow", "wide"] {
        if only_mode.as_deref().is_some_and(|o| o != mode) {
            continue;
        }
        let m = modem(mode);
        let hdr = m.header_symbols();
        for dop in [5.0f32, 20.0] {
            for cnr in [14.0f32, 17.0, 20.0, 24.0] {
                let (mut lost, mut no_detect, mut bad_timing, mut no_header, mut late) = (0, 0, 0, 0, 0);
                let mut genie_ok = 0;
                for t in 0..trials {
                    let b = burst(0, 1, 100 + t);
                    let base = FmChannelConfig { seed: t as u64 + 1, ..FmChannelConfig::by_name(mode).unwrap() };
                    let audio = m.modulate(&b);
                    let truth =
                        m.ofdm.detect(&transmit(&FmChannelConfig { cnr_db: None, ..base.clone() }, &audio), 0).unwrap();
                    let rx = transmit(
                        &FmChannelConfig { cnr_db: Some(cnr), fading: Some(Fading::rayleigh(dop)), ..base.clone() },
                        &audio,
                    );
                    let got = receive(&mut Receiver::new(m.clone()), &rx);
                    if !got.is_empty() {
                        continue;
                    }
                    lost += 1;
                    // Which stage? Walk the detector like the receiver does.
                    let mut from = 0;
                    let mut found = false;
                    let mut stage = 0;
                    while let Some(d) = m.ofdm.detect(&rx, from) {
                        if (d as isize - truth as isize).abs() <= 2 {
                            found = true;
                            let f = m.ofdm.demodulate(&rx, d, hdr).unwrap();
                            stage = if m.decode_header(&f).is_some() { 3 } else { 2 };
                            break;
                        }
                        if d > truth + m.ofdm.p.symbol_len() {
                            stage = 4;
                            break;
                        }
                        from = d;
                    }
                    if !found {
                        if stage == 4 {
                            late += 1;
                        } else {
                            no_detect += 1;
                            let p = &m.ofdm.p;
                            let g = Fading::rayleigh(dop).gains(m.fs(), rx.len(), t as u64 + 1);
                            let pw = |a: usize, b: usize| {
                                10.0 * (g[a..b].iter().map(|x| x.norm_sqr()).sum::<f32>() / (b - a) as f32).log10()
                            };
                            let a0 = truth - p.ncp - p.symbol_len();
                            let minp = g[a0..truth + p.nfft].iter().map(|x| x.norm_sqr()).fold(f32::MAX, f32::min);
                            let gh = m.ofdm.demodulate(&rx, truth, hdr).and_then(|f| m.decode_header(&f));
                            if gh.is_some() {
                                genie_ok += 1;
                            }
                            println!(
                                "   lost t{t} truth {truth} genie header {}: fade A {:.1} dB, B {:.1} dB, min {:.1} dB",
                                gh.is_some(),
                                pw(a0, truth - p.ncp),
                                pw(truth - p.ncp, truth + p.nfft),
                                10.0 * minp.log10()
                            );
                        }
                    } else if stage == 2 {
                        no_header += 1;
                    } else {
                        bad_timing += 1;
                    }
                }
                println!(
                    "{mode} rayleigh:{dop} cnr {cnr}: lost {lost}/{trials}: not detected {no_detect} (genie-timed header ok {genie_ok}), detected elsewhere/late {late}, header failed {no_header}, other {bad_timing}"
                );
            }
        }
    }
}
