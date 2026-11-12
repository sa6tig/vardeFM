use super::*;
use std::sync::Arc;
use vardefm_dsp::prng::Prng;
use vardefm_sim::{transmit, Fading, FmChannelConfig};

pub(crate) fn modem(mode: &str) -> Arc<Modem> {
    Arc::new(Modem::new(OfdmParams::by_name(mode).unwrap(), 0.1))
}

pub(crate) fn burst(level: u8, ncw: usize, seed: u32) -> Burst {
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
pub(crate) fn receive(rx: &mut Receiver, audio: &[f32]) -> Vec<RxBurst> {
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

/// Adds noise to data symbol `hit` of the burst at `start`, on the carriers `band` only
/// (indices into all carriers), at `g` times the signal power per carrier.
fn coloured_click(m: &Modem, audio: &mut [f32], start: usize, hit: usize, band: std::ops::Range<usize>, g: f32) {
    let p = &m.ofdm.p;
    let l = p.symbol_len();
    let from = start + (hit + 1) * l - p.ncp;
    let sig = &audio[start..start + 4 * l];
    let rms = (sig.iter().map(|x| x * x).sum::<f32>() / sig.len() as f32).sqrt();
    // Gaussian noise on each carrier: a complex Gaussian amplitude (Box-Muller) per
    // carrier and symbol, so a pilot's deviation scatters as real noise does.
    let sigma = (g * rms * rms / p.num_carriers as f32).sqrt();
    let mut rng = Prng::new(55 + hit as u32);
    let mut uniform = || (rng.next_u32() as f32 + 1.0) / (u32::MAX as f32 + 2.0);
    for k in band {
        let f = p.carrier_freq(k);
        let (u1, u2) = (uniform(), uniform());
        // Rayleigh amplitude with mean square 2 sigma²: a carrier power of sigma².
        let amp = sigma * (-2.0 * u1.ln()).sqrt();
        let phase = u2 * std::f32::consts::TAU;
        for (i, x) in audio[from..from + l].iter_mut().enumerate() {
            *x += amp * (std::f32::consts::TAU * f * i as f32 / p.fs + phase).cos();
        }
    }
}

/// Bursts of `level` 6, 4 codewords, at `cnr` dB, 12 seeds, every fourth payload symbol
/// hit by Gaussian noise on the carriers `band(nc)` at `g` times the signal power per
/// carrier. Returns codewords decoded with and without `Frame::carrier_noise_scale`,
/// out of 48, how many hit symbols got a profile and how many were hit.
fn click_trials(
    mode: &str,
    cnr: f32,
    g: f32,
    band: fn(usize) -> std::ops::Range<usize>,
) -> (usize, usize, usize, usize) {
    let m = modem(mode);
    let (level, ncw) = (6, 4);
    let nc = m.ofdm.p.num_carriers;
    let hdr = m.header_symbols();
    let nsym = hdr + ncw * m.symbols_per_codeword(level);
    let (mut with, mut without, mut profiled, mut total_hits) = (0, 0, 0, 0);
    for seed in 0..12u32 {
        let b = burst(level, ncw, 40 + seed);
        let (mut audio, start) = received(&m, mode, &b, cnr);
        let hits: Vec<usize> = (hdr + seed as usize % 4..nsym).step_by(4).collect();
        for &h in &hits {
            coloured_click(&m, &mut audio, start, h, band(nc), g);
        }
        let known = m.header_carriers(&b.header);
        let mut frame = m.ofdm.demodulate_known(&audio, start, nsym, &known).unwrap();
        total_hits += hits.len();
        profiled += hits.iter().filter(|&&h| !frame.carrier_noise_scale[h].is_empty()).count();
        with += m.decode_payload(&b.header, &frame).iter().filter(|p| p.is_some()).count();
        // `noise_at` takes the profile over the symbol scale, so clear the profile.
        frame.carrier_noise_scale.iter_mut().for_each(Vec::clear);
        without += m.decode_payload(&b.header, &frame).iter().filter(|p| p.is_some()).count();
    }
    (with, without, profiled, total_hits)
}

#[test]
fn coloured_clicks_need_the_per_carrier_profile() {
    // Noise on the upper half of the band only, a few hundred times the normal noise
    // there and nothing extra below. One noise scale per symbol, the average over its
    // pilots, overstates the noise on the clean carriers and understates it on the hit
    // ones; the per-carrier profile follows the colour. At the time of writing,
    // codewords decoded out of 48 with / without the profile: narrow 38 / 11, wide
    // 23 / 12; a profile on 115 of 120 and 91 of 96 hit symbols.
    for (mode, cnr, g, min_gain) in [("narrow", 24.0, 0.21, 15), ("wide", 33.0, 0.4, 6)] {
        let (with, without, profiled, hits) = click_trials(mode, cnr, g, |nc| nc / 2..nc);
        assert!(profiled * 10 >= 9 * hits, "{mode}: a profile on {profiled} of {hits} hit symbols");
        assert!(with >= without + min_gain, "{mode}: {with}/48 codewords with the profile, {without}/48 without");
    }
}

#[test]
fn flat_clicks_lose_little_to_the_per_carrier_profile() {
    // The same noise over the whole band: one scale per symbol is right, and a profile
    // estimated from single pilot samples would only add scatter (it cost 15 of 48
    // codewords before profiles were limited to uneven noise). At the time of writing,
    // with / without: narrow 36 / 37, wide 35 / 37.
    for (mode, cnr, g) in [("narrow", 24.0, 0.09), ("wide", 33.0, 0.08)] {
        let (with, without, _, _) = click_trials(mode, cnr, g, |nc| 0..nc);
        assert!(without >= 24, "{mode}: only {without}/48 codewords decode: the case says nothing");
        assert!(with + 4 >= without, "{mode}: {with}/48 codewords with the profile, {without}/48 without");
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
