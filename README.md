# VardeFM

[![CI](https://github.com/notepid/vardeFM/actions/workflows/ci.yml/badge.svg)](https://github.com/notepid/vardeFM/actions/workflows/ci.yml)

An open-source, cross-platform high-speed data modem for FM radios. It aims to be a
drop-in alternative to VARA FM for Winlink and similar apps.

The name comes from *varde*, Norwegian for the cairns and beacon fires that once relayed
warnings from mountaintop to mountaintop.

- **Open:** the waveform and protocol are documented in [docs/PROTOCOL.md](docs/PROTOCOL.md).
- **Cross-platform:** written in Rust. Runs on Windows, Linux, macOS and Raspberry Pi.
- **App compatible:** VardeFM offers VARA's TCP command/data interface, so VARA clients
  such as Pat and Winlink Express can talk to it. See "Host interface" in the protocol
  document for the differences from VARA.
- **Not on-air compatible with VARA:** both stations need VardeFM.

## Download and use

Get the archive for your system from the [releases page](https://github.com/notepid/vardeFM/releases):

| Archive | For |
|---|---|
| `windows_amd64.zip` | Windows 10/11, 64-bit |
| `linux_amd64.tar.gz` | Linux PC, 64-bit (Debian 12 / Ubuntu 22.04 or newer) |
| `linux_arm64.tar.gz` | Raspberry Pi 3/4/5/Zero 2 W, 64-bit Raspberry Pi OS (Bookworm or newer) |
| `linux_armv7.tar.gz` | Raspberry Pi 2/3/4/Zero 2 W, 32-bit Raspberry Pi OS |
| `darwin_arm64.tar.gz` | Mac with Apple Silicon (M1 or newer), macOS 11 or newer |
| `darwin_amd64.tar.gz` | Intel Mac, macOS 11 or newer |

Each archive includes the user guide ([packaging/README.md](packaging/README.md)). It covers
installation, connecting a radio, PTT, and setting up Pat, Winlink Express and HamBBS.
[docs/RADIO.md](docs/RADIO.md) goes deeper into the radio setup.

## Modes

| Mode | Radio connection | Audio band | Net rate (7 speed levels) |
|---|---|---|---|
| narrow | mic/speaker, any FM radio | ~420–2670 Hz | 0.7 – 7.4 kbit/s |
| wide | 9600 data port (flat audio) | ~280–6000 Hz | 1.9 – 18.7 kbit/s |

Net rate includes error correction and burst overhead. The link picks the speed level
from the measured signal quality.

## Development

### Building

Install Rust (https://rustup.rs), plus `libasound2-dev` and `pkg-config` on Linux. Then:

```sh
cargo build --release      # binary in target/release/vardefm
cargo test                 # all tests, about 20 s
```

CI also requires `cargo fmt --all --check` and `cargo clippy --workspace --all-targets -- -D warnings` to pass.

### Workspace

```
crates/vardefm-dsp     OFDM modulator/demodulator, constellations, filters
crates/vardefm-fec     LDPC codes, CRCs, scrambling, interleaving
crates/vardefm-modem   bursts: header, speed levels, coded payload, streaming receiver
crates/vardefm-link    ARQ link layer (connect, reliable byte stream, disconnect)
crates/vardefm-sim     FM radio channel simulator
crates/vardefm-tnc     VARA-compatible TCP interface, station logic, sim-pair, radio engine
crates/vardefm-audio   sound card input/output (cpal)
crates/vardefm-ptt     PTT: CM108/CM119/AIOC GPIO (after Dire Wolf), serial RTS/DTR, Hamlib rigctld, Linux GPIO
crates/vardefm-cli     `vardefm` command-line tool
packaging/             user guide and udev rules shipped in the release archives
```

### Virtual radio pair (`sim-pair`)

Two VardeFM modems in one process, joined by the FM channel simulator. Each has its own
VARA TCP ports, so two host applications can connect to each other with no radios.

```sh
# Station A on 8300/8301, station B on 8310/8311, 18 dB CNR, 40 ppm clock error
vardefm sim-pair --cnr 18 --ppm -40 --record wavs
```

| Option | Meaning |
|---|---|
| `--mode narrow\|wide` | mic/speaker audio or 9600 data port |
| `--cnr <dB>` | carrier-to-noise ratio (omit for no noise) |
| `--ppm <n>` | sound-card clock error |
| `--keyup <s>`, `--txdelay <s>` | radio key-up time and the modem's leader (must be longer) |
| `--a-port`, `--b-port` | command ports (data = +1); defaults 8300 and 8310 |
| `--speed <x>` | run x times faster than real time (0 = as fast as possible) |
| `--record <dir>` | save each transmission as `NNNN_<A\|B>_tx.wav` and `_rx.wav` |
| `--seed <n>` | noise seed |

It prints one line per transmission and per reception:
```
   20.97s B->A Data       64QAM 2/3 cw  7  1338 B (resent    0) retry 0 air 2.09 s
   23.08s A rx Data       cw  7/7  SNR  17.6 dB
```

### Measuring the modem

```sh
# Frames through a simulated FM link at 12 dB carrier-to-noise, 80 ppm clock error
vardefm sim --mode narrow --mod qpsk --cnr 12 --ppm 80

# Bit-error table over signal level for every modulation (uncoded)
vardefm sweep --mode wide

# Decoding speed of this computer for every mode and level (Raspberry Pi check)
vardefm bench

# Over real radios: play test.wav into one radio, record the other at 48 kHz, decode
vardefm tx-wav --mode narrow --mod qpsk --out test.wav
vardefm rx-wav --mode narrow --mod qpsk --in recording.wav
```

Useful options: `--txdelay` (leader length; it must exceed the radio's key-up time),
`--tilt=<dB/oct>`, `--rms <level>`. Set `VARDEFM_DUMP=1` to print per-carrier SNR.

### Tests

- `crates/vardefm-cli/tests/audio.rs`: generates modem audio and decodes it: direct
  loopback, level and polarity changes, WAV round trips, false alarms, noisy simulated
  links, clock offset, key-up time, many frames in one recording.
- `crates/vardefm-link/src/tests.rs`: the ARQ link over a fake lossy channel.
- `crates/vardefm-tnc/src/radio/tests.rs`: the real-radio engine with fake sound cards
  and PTT. It checks that audio only plays while keyed, the PTT tail, and the watchdog.
- `crates/vardefm-tnc/tests/simpair.rs`: end to end. Two host clients talk over the VARA
  TCP interface through `sim-pair` and transfer 10 kB each way at several signal levels.
  Set `VARDEFM_SIM_VERBOSE=1` to print every transmission.

To keep the audio from the `audio` tests as WAV files (the folder is created in the repo root):

```powershell
# Windows PowerShell
$env:VARDEFM_TEST_WAV_DIR = "wavs"; cargo test --test audio; Remove-Item Env:VARDEFM_TEST_WAV_DIR
```

```sh
# Linux / macOS / Git Bash
VARDEFM_TEST_WAV_DIR=wavs cargo test --test audio
```

Benchmarks (ignored by default; run in release mode):
```sh
cargo test --release -p vardefm-modem level_table -- --ignored --nocapture   # level vs CNR
cargo test --release -p vardefm-fec fer_curve -- --ignored --nocapture       # LDPC curves
```

Performance gates (ignored by default, run by hand in release mode). They fail when a
level's 10 % codeword error point (`crates/vardefm-modem/perf/levels.toml`) or a code's
1 % FER point (`crates/vardefm-fec/perf/ldpc.toml`) has moved:
```sh
cargo test --release -p vardefm-modem level_gate -- --ignored --nocapture   # about 1 min
cargo test --release -p vardefm-fec fer_gate -- --ignored --nocapture       # a few s
```
A deliberate change records new points in the same commit. Commit the code first, so
the files name the commit measured, then run:
```sh
WRITE=1 cargo test --release -p vardefm-modem level_sweep -- --ignored --nocapture
WRITE=1 TRIALS=1000 cargo test --release -p vardefm-fec fer_curve -- --ignored --nocapture
```

### Releases

GitHub Actions runs the checks and tests on Windows, Linux x86-64, Linux ARM and macOS for every
push and pull request, and builds all release archives (kept 7 days as workflow artifacts).
To publish a release:

1. Set `version` in the root `Cargo.toml` (e.g. `0.3.0`), run `cargo build` to update
   `Cargo.lock`, add the release to [CHANGELOG.md](CHANGELOG.md), and commit. Bump the
   minor version whenever the on-air protocol changes.
2. Tag and push: `git tag v0.3.0; git push origin master v0.3.0`.

The release workflow checks that the tag matches `Cargo.toml`, runs the tests, builds the
six archives plus `checksums.txt`, and publishes the release with notes generated from the
commits. A tag with a suffix such as `v0.3.0-rc1` becomes a pre-release.

## License

GPL-3.0-or-later; see [LICENSE](LICENSE). The CM108 PTT support follows Dire Wolf by WB2OSZ.
