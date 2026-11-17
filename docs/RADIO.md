# Running VardeFM on a radio

VardeFM needs a sound card connected to the radio, and a way to key the
transmitter (PTT). `vardefm modem` then offers the VARA-compatible TCP ports
(default 8300/8301) to Pat, Winlink Express, HamBBS or any other VARA client.

## 1. Pick the PTT method

| Interface | `--ptt` |
|---|---|
| CM108/CM119-based USB audio with PTT on a GPIO pin (many DIY and commercial interfaces) | `cm108` (GPIO 3), or `cm108:<gpio>` |
| AIOC (All-In-One Cable) | `cm108` |
| Interface with a USB serial port keying on RTS or DTR (e.g. Digirig) | `rts:COM5` / `dtr:COM5` (`/dev/ttyUSB0` on Linux, `/dev/cu.usbserial-…` on macOS); prefix `-` to invert |
| Radio keyed over CAT by Hamlib | `rigctld` or `rigctld:<host>:<port>` (start `rigctld` first) |
| Raspberry Pi (or other Linux board) with PTT wired to a header pin, e.g. a radio HAT | `gpio:<pin>` (BCM number: `gpio:17` = header pin 11), `gpio:<chip>:<line>`; prefix `-` to invert |
| VOX: the interface keys the radio when it hears audio (SignaLink, and cheap USB sound dongles sold as radio interfaces that have no COM port, such as GeneralPlus `1b3f:2008`) | `vox` (same as `none`) |
| The host application keys the radio | `none` |

CM108-family devices are recognised by USB ID: C-Media `0d8c:0008`–`000f`,
`0012`, `0013`, `0139`, `013a`, `013c`; SSS1621/1623 `0c76:1605/1607/160b`; AIOC
`1209:7388`. With more than one connected, choose one with
`--ptt cm108@<path>` (paths come from `vardefm devices`). The method follows
Dire Wolf's `cm108.c` by WB2OSZ.

**Linux:** HID devices are root-only by default. Allow your user (in group
`audio`) to use them, then unplug and re-plug the interface. The release archives
include this file as `99-vardefm-cm108.rules`; copy it to `/etc/udev/rules.d/` and
run `sudo udevadm control --reload-rules`:

```
# /etc/udev/rules.d/99-vardefm-cm108.rules
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0d8c", GROUP="audio", MODE="0660"
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0c76", GROUP="audio", MODE="0660"
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="7388", GROUP="audio", MODE="0660"
```

**macOS:** CM108 devices and serial ports work without setup. For serial PTT use
the `/dev/cu.…` name of the port; opening `/dev/tty.…` can hang. macOS asks once
whether Terminal may use the microphone; allow it, or the receiver hears silence.

**GPIO on Linux:** the pin is used through `/dev/gpiochip*`. On Raspberry Pi OS
your user needs to be in group `gpio` (the default user already is):
`sudo usermod -aG gpio $USER`, then log in again. `gpio:<n>` finds the pin named
`GPIO<n>` on any chip, so it works on the Pi 5 too.

## 2. Set up step by step (Windows PowerShell shown)

1. **Find the devices.**
   ```powershell
   .\vardefm devices
   ```
   This lists sound cards, CM108-type PTT devices and serial ports. Sound cards
   are chosen with `--audio-in` / `--audio-out` and any unique part of the name,
   e.g. `--audio-in "USB Audio"`.

2. **Check PTT.** The radio should transmit for 2 seconds.
   ```powershell
   .\vardefm ptt-test --ptt cm108
   ```

3. **Set the transmit level.** Send a full-scale 1 kHz tone into a dummy load or
   a quiet frequency. Adjust the sound card's output level (Windows sound
   settings / `alsamixer`), the radio's data/mic gain or `--tx-gain`, until the
   deviation is about 2.5 kHz (narrow, mic input) or 3 kHz (wide, data port).
   Without a deviation meter, start low and raise it until a second radio's
   `monitor` shows a clean, steady level.
   ```powershell
   .\vardefm tone --audio-out "USB Audio" --ptt cm108 --seconds 10
   ```
   Then check the level with real modem audio. Bursts at the robust speed levels
   go out louder than the fast ones, and a connect request is the loudest burst
   VardeFM sends (RMS 0.30 narrow, 0.50 wide; peaks at full scale). Start the
   modem and let your program call a station that is not there: VardeFM sends
   the request 5 times. Its peaks should reach the deviation above and not
   go past it. If the radio has a mic compressor or AGC, the tone and the modem
   audio can deviate differently; then set the level on the connect request.

4. **Set the receive level.** Let another station transmit (or play
   `tx-wav` audio from a second radio). Adjust the input level so peaks sit
   around −6 dBFS and never reach 0 dBFS. `monitor` also decodes VardeFM bursts
   and shows their SNR and peak level, with advice (`ok`, `loud: turn the
   radio volume down a little`, `quiet: turn the radio volume up`, ...).
   ```powershell
   .\vardefm monitor --audio-in "USB Audio"
   ```
   On radios where the sound card takes the speaker audio, the volume knob sets
   this level, so leave it alone once it is right. `vardefm modem --levels` shows
   the same meter every 2 s while the modem is idle, and the modem always shows
   each received burst's peak level. With or without `--levels`, the modem warns
   (at most every 30 s) when received bursts are too loud or too quiet, or when
   any received audio clips. Set the volume while a station is transmitting: with
   the squelch closed the idle meter reads quiet, and that is fine.

   **Squelch.** Set it just high enough to close when nobody is transmitting
   (on many handhelds a low setting such as 1–3). VardeFM waits for a free
   channel before it transmits, and with the squelch closed any signal counts,
   including someone talking.
   An open squelch also works: VardeFM learns the steady hiss as background
   within about 2 s and does not count it as busy, and it finds its own frames
   in the noise. But a voice or other signal that is not clearly louder than the
   hiss is then not noticed. Squelch tail noise is fine.

5. **Run the modem.**
   ```powershell
   .\vardefm modem --audio-in "USB Audio" --audio-out "USB Audio" --ptt cm108
   ```
   Then point Pat / HamBBS / Winlink Express at VARA FM on `127.0.0.1:8300`.
   Use `--mode wide` only with the radio's flat 9600-baud data port, on both
   stations. Stop with Ctrl-C; PTT is released on exit.

## 3. Timing options

| Option | Default | Meaning |
|---|---|---|
| `--txdelay` | 0.15 s | Leader before each burst. Must be longer than the radio's key-up time (PTT to full power and receiver unsquelched). Many handhelds need 0.2–0.3 s. |
| `--txtail` | 0.05 s | PTT held after the last sample has left the sound card. |

If the other station's log shows frames missing their start, raise `--txdelay`.

**VOX.** With `--ptt vox` the leader is audio, so it also triggers the VOX. The
problem is the end of a burst: the interface keeps the radio keyed for its hang
time after the audio stops, and VardeFM cannot see this. The other station starts
its answer 0.2 s after our burst ends, after its own `--txdelay` leader, so with
the defaults the first 0.35 s are leader. A VOX hang shorter than about 0.3 s is
fine. With a longer hang we miss the start of every answer: turn the hang time
down if the interface has a trimmer, or ask the other station to raise its
`--txdelay` above our hang time. `vardefm ptt-test` does nothing with VOX; use
`vardefm tone` to check that the radio keys.
A watchdog unkeys the radio if a transmission runs 3 s past its expected length.

## 4. Slow computers (Raspberry Pi)

The receiver decodes each burst after it ends, and the reply waits for that.
`vardefm bench` measures how long this takes for each speed level, on all CPU
cores, and says whether this computer keeps up:

```
vardefm bench
```

If it recommends a limit, start the modem with it, e.g. `--max-level 4`. The
modem then never asks the other station for a faster level, and never sends one.

## 5. Running a digipeater

A VardeFM station can repeat other stations' sessions, so two stations out of each
other's range can connect through it. Start the modem with the call sign it repeats for:

```
vardefm modem --audio-in "USB" --audio-out "USB" --ptt vox --digi LA3DDD-9
```

It needs no host program, and it can still carry its own sessions on the same port.
While it repeats a busy session, though, its own session is slow and may be dropped.
A host then connects with `CONNECT <mycall> <target> VIA LA3DDD-9` (up to two
digipeaters, in order). The digipeater prints an `rpt` line for each burst it repeats.

Every burst goes on the air once per digipeater, so a session through one digipeater
runs at about half speed, and a lost burst takes longer to recover. The stations at the
ends wait for the repeats before they retry. If the digipeater is slow, for example a
Raspberry Pi that takes long to decode, raise `--hop-margin` (default 0.5 s per
digipeater, at most 10 s) on the two end stations, to the same value on both. An answer
passes the digipeater twice, so the margin has to be about twice the digipeater's extra
delay. A long VOX hang on the digipeater is a different problem, and the hop margin does
not help with it: its neighbours answer 0.2 s after its repeat ends, while it is still
keyed. Shorten the hang or raise the neighbours' `--txdelay`, as in section 3.
Digipeating only works between VardeFM stations; VARA FM digipeaters cannot repeat
VardeFM.

## 6. Watching the modem

`--api 127.0.0.1:8770` serves a read-only monitor API on that address: counters once a
second, every burst sent and heard, busy changes, warnings, the constellation of each
received burst and a waterfall of the received audio. It listens on the local computer only. [api/README.md](api/README.md) describes it.

## 7. What is not done yet

- Sound card and CM108 PTT matching is manual when several adapters are
  connected. Dire Wolf matches them automatically on Linux.
- The rate-adaptation thresholds come from the channel simulator; on-air
  measurements will refine them.
