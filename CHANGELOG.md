# Changelog

All notable changes to VardeFM. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
Until 1.0 a minor version bump (0.x) means the on-air protocol changed: both stations must
run the same minor version.

## [Unreleased]

### Added
- **Detector settings and per-carrier noise in vardefm-dsp.** `Ofdm::detect_with` runs
  burst detection with an `ScConfig` (`ScConfig::CLASSIC` is the detector before this
  release). `Frame::noise_at` gives the noise variance of one cell, and the new public
  field `Frame::carrier_noise_scale` holds the per-carrier scales; code that builds a
  `Frame` with a struct literal must add it.
- **Performance gates.** `level_gate` and `fer_gate` are ignored tests, run by hand in
  release mode, that fail when decoding performance moves. Per mode and level, the CNR
  at which 10 % of codewords fail, and per mode and drive level the CNR at which 10 % of
  headers fail (over random header fields), are recorded in
  `crates/vardefm-modem/perf/levels.toml`; per LDPC code, the Eb/N0 of 1 % FER in
  `crates/vardefm-fec/perf/ldpc.toml`. `level_sweep` and `fer_curve` with `WRITE=1`
  record them again. The figures in `docs/PROTOCOL.md` §2.2 and §2.4 now refer to these
  files.
- **Fading in the FM channel simulator.** `FmChannelConfig::fading` applies Rayleigh or
  Rician flat fading (sum-of-sinusoids, Clarke spectrum, seeded like the noise) to the
  carrier before the IF filter. `vardefm sim`, `sweep` and `sim-pair` take
  `--fading rayleigh:<Doppler Hz>` or `--fading rician:<Doppler Hz>:<K dB>`, and the
  `fading_table` benchmark runs the level table at 5, 20 and 80 Hz Doppler.

### Changed
- **Burst detection holds up when noise hits preamble A.** The receiver only; on-air
  compatible. The Schmidl-Cox metric is normalized by the quieter half of its window
  (P²/min(R1, R2)², the energy ratio counted at most 3 times, threshold 0.4) instead of
  the second half (P²/R2², 0.3). Windows whose halves differ by more than 10 dB are
  gated out. Fine timing searches the whole plateau, up to 4 cyclic prefixes either
  side. On the simulator (`detector_eval`, 100 paired bursts) narrow mode at 5 dB CNR
  decodes 84 bursts instead of 59, and Rayleigh 20 Hz at 14 dB mean CNR 88 (narrow)
  and 77 (wide) instead of 73 and 72. Detections on a weak carrier (5 dB CNR) are 0-2
  per minute against the old 1-10, none with a false header.
- **Per-carrier noise for some click-hit symbols.** A symbol whose noise scale is above
  1, and whose pilots show its noise to be uneven across the band (one half more than 4
  times the other), gets a per-carrier scale from its own pilots. That helps where
  such noise hits many symbols (a band-limited interferer); on the simulated fading
  channels the gain is a few codewords in a hundred or none.
- **Busy detection measures only the carrier band** (422–2672 Hz narrow, 400–6000 Hz wide).
  A CTCSS tone, mains hum, a DC step or hiss above the band no longer reads as a busy
  channel, and no longer hides a signal inside the band. A keyed carrier with only a CTCSS
  tone on it is not busy, which lets the modem work through a repeater's tail. Hosts see
  fewer `BUSY ON` lines. (#7, thanks @mustig)
- After its own transmission the modem measures nothing for busy detection until received
  audio comes back (a handheld getting back on receive, a VOX interface's hang time, at
  most 1 s), nor for 20 ms after that. On a repeater that stays keyed with a CTCSS tone,
  the tone came back abruptly and the band filter read that as busy for 150 ms after
  every burst, which held back the modem's next transmission. A busy hold from before a
  transmission no longer carries over it. (#10)

### Fixed
- A NaN, an Inf or a sample far beyond full scale from the sound card no longer loses a
  burst it falls in or shortly before, and cannot leave busy detection deaf; the last
  good sample takes its place. (#10)

## [0.3.1] - 2026-11-09

On-air compatible with 0.3.0.

### Added
- **Receive level advice.** `vardefm modem --levels` prints the receive level every 2 s
  while idle, with advice for the radio's volume knob. The modem always shows each
  received burst's peak level and warns, at most every 30 s, when bursts are too loud or
  too quiet or when received audio clips. `vardefm monitor` gives the same advice.
  `RxBurst` carries the burst's audio level (`AudioLevel`).

### Changed
- `vardefm ptt-test --ptt vox` now says that VardeFM keys nothing with VOX and points to
  `vardefm tone`, instead of reporting a 2-second key-up that never happened.
- `docs/RADIO.md` covers USB sound dongles that key the radio by VOX, and how the VOX
  hang time eats into the other station's leader.

### Fixed
- **The modem never transmitted with the radio's squelch open.** The steady hiss
  counted as a busy channel (anything above −40 dBFS did), so a call waited for a free
  channel until it gave up. The busy detector now compares the audio with the
  background level of the last 2 s and needs 6 dB more, so a steady hiss is not busy;
  with the squelch closed it behaves as before. It also ignores a DC offset from the
  sound card. `docs/RADIO.md` now recommends a squelch that closes when nobody
  transmits, and explains what an open squelch costs.
- The idle level meter (`modem --levels`, `monitor`) no longer says "turn the radio
  volume up" when the squelch is simply closed.
- **DISCONNECT could overtake data** that the host wrote just before it, on a busy
  computer. The data port reader timed its "socket empty" mark after the read had timed
  out, so a pause of the reader thread could make data that arrived meanwhile count as
  already delivered. The mark now uses the time the read began. DISCONNECT also waits until
  the data socket is still empty 50 ms after the command, which covers data that has not
  yet left the host when the host writes more than the socket buffers hold. Under heavy
  CPU load the server test failed 3 times in 30 before and 0 times in 150 after.

## [0.3.0] - 2026-11-03

**Not on-air compatible with 0.2.x.** The LDPC codes, the per-level transmit drive and the
rate-adaptation thresholds all changed. Both stations must upgrade.

### Changed
- **LDPC codes** are now built with progressive edge growth (PEG) and an irregular
  information-degree profile, and decoded with layered offset min-sum (offset 0.5) instead
  of normalized min-sum. At 1 % frame error rate the rate-1/2 code needs 1.5 dB Eb/N0
  instead of 3.1 dB, the header code 2.2 dB instead of 3.7 dB; the other rates gain
  0.3–0.5 dB. The normative construction in `docs/PROTOCOL.md` §2.4 is rewritten.
- **Transmit drive is set per speed level.** BPSK and QPSK bursts go out louder than
  64QAM bursts (wide: RMS 0.50 / 0.40 / 0.30 for levels 0–2 / 3–4 / 5–6; narrow: 0.30 /
  0.30 / 0.25). Peaks are limited by two passes of zero-phase clip-and-filter, so
  out-of-band splatter stays more than 30 dB down. Connect requests and acks are now the
  loudest bursts: calibrate deviation while the modem sends a connect request (see
  `docs/RADIO.md`).
- **Receiver channel estimate** is trained on the decoded header symbols as well as the
  preamble, each weighted by the inverse of its noise so a click on a header symbol does
  not corrupt the estimate. A per-symbol noise scale down-weights payload symbols hit by
  an FM click or squelch pop so one bad symbol no longer sinks a codeword.
- **Rate adaptation** thresholds were re-measured on a 0.25 dB grid. Another level's SNR
  is predicted from the measured burst with a drive-and-ceiling model
  (`1/snr = 1/(channel · drive²) + 1/ceiling`), so the link no longer stalls below the
  fastest level on a strong signal.
- Speed level 6 (64QAM 5/6) decodes error-free from about 16.5 dB carrier-to-noise in
  narrow mode and 24.5 dB in wide mode (was 20 dB and 28 dB). In the simulator a 2 × 10 kB
  transfer at 80 ppm clock error takes 25–40 % less time than 0.2.1 at moderate signal.
- The modem builds each distinct LDPC code once at start-up.

### Known limitations
- Decoding the highest levels costs up to about 45 % more CPU than 0.2.1. Desktop PCs have
  ample headroom; on a Raspberry Pi 3 check `vardefm bench` and use `--max-level` if it
  reports a level as too slow.

## [0.2.1] - 2026-10-17

### Added
- macOS support: release archives for Apple Silicon and Intel, CI tests on macOS.

## [0.2.0] - 2026-10-17

### Added
- Raspberry Pi: GPIO PTT, the `bench` command, multi-core decoding and `--max-level`.
- The user guide ships as `README.md` inside the release archives.

## [0.1.0] - 2026-10-17

First release: real-valued OFDM modem for FM radios (narrow and wide modes), LDPC error
correction with seven speed levels, selective-repeat ARQ link layer, VARA-compatible TCP
host interface, `sim-pair` virtual radio pair, sound card I/O with serial, CM108 and
GPIO PTT, and release builds for Windows, Linux and Raspberry Pi.

[Unreleased]: https://github.com/notepid/vardeFM/compare/v0.3.1...HEAD
[0.3.1]: https://github.com/notepid/vardeFM/compare/v0.3.0...v0.3.1
[0.3.0]: https://github.com/notepid/vardeFM/compare/v0.2.1...v0.3.0
[0.2.1]: https://github.com/notepid/vardeFM/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/notepid/vardeFM/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/notepid/vardeFM/releases/tag/v0.1.0
