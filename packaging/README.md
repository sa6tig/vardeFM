# VardeFM

VardeFM is a free, open-source data modem for FM radios. It turns a radio and a
sound card interface into a fast data link for Winlink and BBS programs.

Programs talk to VardeFM the same way they talk to VARA FM (TCP ports 8300/8301),
so a program with VARA FM support can use VardeFM instead.

> **Both stations need VardeFM.** VardeFM does not talk on the air to stations
> running the original VARA FM.

Project page, source code and issues: https://github.com/notepid/vardeFM

## What's in this archive

| File | What it is |
|---|---|
| `vardefm` / `vardefm.exe` | The program |
| `README.md` | This guide |
| `docs/RADIO.md` | Detailed radio setup guide |
| `docs/PROTOCOL.md` | Technical description of the waveform and protocol |
| `99-vardefm-cm108.rules` | (Linux only) lets normal users key CM108/AIOC PTT |
| `LICENSE` | GNU GPL version 3 |

## 1. Install

### Windows

1. Unzip the archive to a folder, e.g. `C:\VardeFM`.
2. Open PowerShell in that folder and check that it runs:
   ```powershell
   .\vardefm.exe --version
   ```
   If Windows warns about an unknown publisher, choose "More info" → "Run anyway".
   The program is not code-signed.

### Linux PC and Raspberry Pi

Pick the right archive for the computer:

| Computer | `uname -m` says | Archive |
|---|---|---|
| Linux PC, 64-bit | `x86_64` | `linux_amd64` |
| Raspberry Pi with 64-bit Raspberry Pi OS (Pi 3, 4, 5, Zero 2 W) | `aarch64` | `linux_arm64` |
| Raspberry Pi with 32-bit Raspberry Pi OS (Pi 2, 3, 4, Zero 2 W) | `armv7l` | `linux_armv7` |

The original Pi Zero and Pi 1 are not supported. Raspberry Pi OS Bookworm (Debian 12)
or newer is required; the same goes for other distributions (Ubuntu 22.04 or newer).

```sh
tar xzf vardefm_*_linux_*.tar.gz
cd vardefm_*_linux_*/
sudo cp vardefm /usr/local/bin/
vardefm --version
```

If it says `libasound.so.2: cannot open shared object file`, install the ALSA library:
`sudo apt install libasound2` (`libasound2t64` on Debian 13 / Ubuntu 24.04 and newer).

On a Raspberry Pi, check that it is fast enough. This takes a few seconds and
transmits nothing:

```sh
vardefm bench
```

It measures each speed level and tells you if you should limit the speed with
`--max-level` (see step 4). A Pi 4 or 5 is expected to handle every level.

For a CM108/CM119/AIOC interface, allow your user to key the PTT without root.
Then unplug and re-plug the interface:

```sh
sudo cp 99-vardefm-cm108.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo usermod -aG audio $USER     # then log out and in again
```

### macOS

Pick `darwin_arm64` for a Mac with Apple Silicon (M1 or newer; `uname -m` says
`arm64`) or `darwin_amd64` for an Intel Mac (`x86_64`). macOS 11 Big Sur or newer
is required. In Terminal:

```sh
tar xzf vardefm_*_darwin_*.tar.gz
cd vardefm_*_darwin_*/
xattr -d com.apple.quarantine vardefm   # only if you downloaded with a browser
sudo mkdir -p /usr/local/bin
sudo cp vardefm /usr/local/bin/
vardefm --version
```

The program is not notarized by Apple. Without the `xattr` line, macOS says it
"cannot verify" `vardefm` and refuses to run it. You can also allow it in System
Settings → Privacy & Security → "Open Anyway".

The first time VardeFM listens to a sound card, macOS asks whether Terminal may use
the microphone. Allow it, or VardeFM hears only silence. CM108/AIOC PTT needs no setup.

## 2. Try it without a radio (optional)

`sim-pair` runs two VardeFM stations in one program, joined by a simulated FM radio
link. Point two programs at it to see a connection work before touching a radio:

```sh
vardefm sim-pair --cnr 20
```

Station A listens on port 8300 and station B on port 8310. `--cnr` sets the
simulated signal quality in dB: 30 is excellent, 10 is poor. Stop it with Ctrl-C.

To try a digipeater, add `--digi LA3DDD`: A and B can then reach each other only
through it, with `CONNECT <A's call> <B's call> VIA LA3DDD`.

## 3. Connect the radio

You need a sound card interface between the computer and the radio, e.g. a
CM108-based USB interface, AIOC, Digirig or SignaLink. Then:

1. **Find the devices.** List sound cards, CM108-type PTT devices and serial ports:
   ```sh
   vardefm devices
   ```
   You choose a sound card with any unique part of its name, e.g. `--audio-in "USB"`.

2. **Check the PTT.** The radio should transmit for 2 seconds:
   ```sh
   vardefm ptt-test --ptt cm108
   ```

   | Your interface | `--ptt` |
   |---|---|
   | CM108/CM119 USB interface (PTT on GPIO 3), AIOC | `cm108` |
   | ...with PTT on another GPIO pin | `cm108:<pin>`, e.g. `cm108:4` |
   | Serial port RTS or DTR (Digirig and others) | `rts:COM5`, `dtr:COM5`, Linux: `rts:/dev/ttyUSB0`, macOS: `rts:/dev/cu.usbserial-…` |
   | ...with inverted logic | `-rts:COM5` |
   | Radio keyed by Hamlib over CAT (start `rigctld` first) | `rigctld` or `rigctld:<host>:<port>` |
   | Raspberry Pi header pin (radio HATs, DIY transistor PTT) | `gpio:<pin>` with the BCM number, e.g. `gpio:17` (header pin 11); `-gpio:17` inverts |
   | VOX (SignaLink and similar) | `none` |

3. **Set the transmit level.** Send a 1 kHz tone on a quiet frequency, or into a dummy load:
   ```sh
   vardefm tone --audio-out "USB" --ptt cm108 --seconds 10
   ```
   Adjust the sound card's playback volume (Windows sound settings, `alsamixer` on
   Linux, Audio MIDI Setup on macOS) until the radio deviates about 2.5 kHz. Too much audio is worse than too
   little. Without a deviation meter, start low and raise it until another radio
   running `vardefm monitor` sees a clean, steady signal.

   Then check it with real modem audio. A connect request is the loudest
   transmission VardeFM makes: slow, robust transmissions go out louder than fast
   ones. Start the modem (section 4) and connect from your program to a callsign
   that will not answer; VardeFM sends the request 5 times. The peaks should
   reach about 2.5 kHz and not more. If your radio has a mic compressor or AGC,
   set the level on the connect request rather than on the tone.

4. **Set the receive level.** While another station transmits:
   ```sh
   vardefm monitor --audio-in "USB"
   ```
   Adjust the recording level so peaks stay around −6 dB and never reach 0 dB. The
   monitor also decodes VardeFM bursts it hears and shows their signal-to-noise
   ratio (SNR). Set the radio's squelch low or open. VardeFM finds its own signals.

## 4. Run the modem

```sh
vardefm modem --audio-in "USB" --audio-out "USB" --ptt cm108
```

On Windows, start it with `.\vardefm.exe` in place of `vardefm`. Leave the modem
running while you use your Winlink or BBS program. It prints one line per
transmission and reception. Stop it with Ctrl-C; this also releases the PTT.

Important options:

| Option | Default | Meaning |
|---|---|---|
| `--mode narrow\|wide` | `narrow` | `narrow` for the mic/speaker connection (any FM radio). `wide` only with a flat 9600-baud data port, and both stations must use it. |
| `--txdelay <seconds>` | `0.15` | Lead-in before each transmission. Must be longer than the radio's key-up time; many handhelds need 0.2–0.3. Raise it if the other station misses the start of your transmissions. |
| `--txtail <seconds>` | `0.05` | How long PTT stays on after the audio ends. |
| `--tx-gain <x>` | `1` | Transmit level multiplier, if the sound card volume alone is not enough. |
| `--in-channel left\|right` | `left` | Which input channel the radio is on. |
| `--out-channel left\|right\|both` | `both` | Which output channel(s) to send on. |
| `--max-level <0-6>` | `6` | Highest speed level to use. Lower it if `vardefm bench` says the computer is too slow. |
| `--port <n>` | `8300` | Command port for the program; the data port is this plus one. |
| `--host <address>` | `127.0.0.1` | Use `0.0.0.0` to accept programs on other computers. Anyone who can reach the port can transmit, so only do this on a trusted network. |
| `--quiet` | | Do not print a line per transmission. |
| `--record <folder>` | | Save every transmission as a WAV file. |
| `--digi <callsign>` | | Also act as a digipeater for this call sign (repeat for more). Works with no program connected. |
| `--hop-margin <seconds>` | `0.5` | Extra wait per digipeater on sessions through digipeaters, whether you call or are called. Raise it, on both stations, if a slow digipeater makes transfers retry (at most 10). |

`vardefm modem --help` lists every option.

## 5. Use it from your program

Configure the program for **VARA FM** at **127.0.0.1, port 8300**, and start
`vardefm modem` before connecting. Let VardeFM key the radio: turn off PTT control
in the program.

- **HamBBS:** tested with VardeFM.
- **Pat:** in Pat's configuration, set `"varafm": { "addr": "localhost:8300", "ptt_ctrl": false }`,
  then connect with `pat connect varafm:///<CALLSIGN>`.
- **Winlink Express:** use a VARA FM session with the TNC address 127.0.0.1 and port 8300.

Pat and Winlink Express use the same interface as HamBBS, but they have not been
tested with VardeFM yet. Please report what you find.

The program sets your callsign (`MYCALL`) and connects or listens. VardeFM does
not need its own callsign setting.

To connect through digipeaters, the program sends `CONNECT <mycall> <target> VIA <digi1>
[<digi2>]` (at most two). The digipeaters must be VardeFM stations started with `--digi`
(see docs/RADIO.md).

## 6. Start automatically on a Raspberry Pi (optional)

Create `/etc/systemd/system/vardefm.service`, with your user name and devices filled in:

```ini
[Unit]
Description=VardeFM modem
After=sound.target network.target

[Service]
User=YOUR_USER
SupplementaryGroups=audio gpio
ExecStart=/usr/local/bin/vardefm modem --audio-in "USB" --audio-out "USB" --ptt cm108 --quiet
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

Then:

```sh
sudo systemctl daemon-reload
sudo systemctl enable --now vardefm
journalctl -u vardefm -f        # watch the log
```

## 7. Troubleshooting

| Problem | What to try |
|---|---|
| `devices` lists no CM108 device (Linux) | Install the udev rules from step 1 and re-plug the interface. Check with `ls -l /dev/hidraw*`. |
| Several interfaces connected | Pick one with `--ptt cm108@<path>`, using a path from `vardefm devices`. |
| The radio does not transmit | Run `ptt-test`. Try another GPIO pin (`cm108:4`), RTS vs DTR, or inverted logic (`-rts:`, `-gpio:`). |
| macOS: "vardefm" cannot be opened or verified | Run `xattr -d com.apple.quarantine vardefm`, or allow it in System Settings → Privacy & Security. |
| macOS: `monitor` shows no level at all | Allow Terminal under System Settings → Privacy & Security → Microphone, then restart Terminal. |
| macOS: serial PTT hangs when opening the port | Use the `/dev/cu.…` name of the port, not `/dev/tty.…`. |
| `gpio:` PTT says permission denied | Add your user to the `gpio` group: `sudo usermod -aG gpio $USER`, then log in again. |
| Transfers on a Pi stall or retry a lot at high speed | Run `vardefm bench` and use the `--max-level` it suggests. |
| The other station never decodes you | Check the transmit level with `tone`, and raise `--txdelay`. |
| You never decode the other station | Watch `monitor`: is the level moving? Check `--audio-in` and `--in-channel`, and open the squelch. |
| The connection starts but runs slowly | The signal is weak or distorted. Check both audio levels, and keep the transmit level moderate. |
| The modem fails to start with "address already in use" or "only one usage of each socket address" | Another modem (VARA or VardeFM) already uses port 8300. Stop it or use `--port`. |
| Both stations hear each other but nothing connects | Both must run VardeFM, in the same `--mode`. |

More detail is in `docs/RADIO.md`.

## License

VardeFM is free software under the GNU General Public License, version 3 or later
(see `LICENSE`). The CM108 PTT support follows Dire Wolf by WB2OSZ.
