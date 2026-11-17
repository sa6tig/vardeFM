# Monitor API

`vardefm modem --api 127.0.0.1:8770` serves a read-only view of the modem: its counters, what it
sends and hears, and a waterfall of the received audio. It listens on 127.0.0.1 or [::1] only,
and answers only requests whose `Host` (and `Origin`, if any) is a loopback name, so a web page
cannot reach it through the browser.

- `GET /v1/status`: the latest `Status`, the last 64 shared events (`recent`) and the last 64
  vardeFM events (`vfm_recent`), as JSON ([status.schema.json](status.schema.json)).
- `/v1/stream` (WebSocket): `Hello`, then the latest `Status`, then events and a `Status` every
  second as JSON text, and waterfall rows and constellations as binary frames. Each text message is one envelope
  ([envelope.schema.json](envelope.schema.json)). The stream is read-only: a message from the
  client closes it. At most 8 clients.

## Watching in a terminal

`vardefm watch [ADDR]` (default `127.0.0.1:8770`) shows the stream in the terminal: the burst
log and the modem's status on top, the waterfall below with the latest burst's constellation
beside it. Time runs left to right in the waterfall, one column per row (0.1 s), the highest
frequency at the top; a strip under it marks received bursts (green, red if a codeword failed),
sent bursts (yellow) and busy time (grey). The colour scale starts at the noise floor, the
lowest tenth of the last 5 s of bins, and spans 60 dB: the up and down keys move it, `+` and
`-` change the span, `a` resets both, `q` quits. Times are seconds on the modem's receive clock.
If the modem is not there or goes away, it retries every second; a new `Hello` clears the
waterfall and the constellation and marks the burst log.

`--save FILE` also writes everything the modem sends to a capture, one JSON object per line:
`{"ms":...,"text":...}` for a message and `{"ms":...,"bin":...}` (base64) for a binary frame,
`ms` counted from the start. It refuses a file that already exists. If writing fails later,
the status pane says so and the view goes on unsaved. `vardefm watch --replay FILE
[--speed N]` plays a capture back without a modem, `N` from 0.01 to 1000 times as fast as it
was recorded.

## Messages

Shared with other modems that serve this API:

- `Hello`: server, mode (`vardefm-narrow` or `vardefm-wide`) and the waterfall's format.
- `Status`: counters, once a second. See below for those vardeFM does not measure.
- `Gap`: received audio was lost (the computer fell behind) just before sample `t`.
- `Warning`: something to fix. vardeFM sends `rx_level` (receive level too high or too low,
  at most every 30 s), `tx_watchdog` (a transmission ran far too long and was cut off) and `ptt`
  (keying or unkeying failed).

vardeFM's own ([vardefm-events.schema.json](vardefm-events.schema.json)), which a client that does
not know them skips:

- `VfmBurstRx`: a burst decoded. `t` is the first sample after the sender's leader (the start of
  the sync symbols), `t_end` just past its last symbol. A burst that spans a `Gap` rarely decodes;
  if one does, it is timed from after the gap, and its `t` falls in the lost audio.
- `VfmBurstTx`: a burst keyed up, at `t`.
- `VfmBusy`: the channel turned busy or free, as the host's `BUSY ON` / `BUSY OFF`.

Every time is a sample count on the 48 kHz receive clock, which starts at 0 with the modem. It
also counts the audio heard while transmitting, which the receiver ignores, and the audio lost
before a `Gap`, so times of bursts on either side of a transmission or a gap compare directly.

## Waterfall

Ten rows a second. Each row is the spectrum of 4096 samples (Hann window), one byte per bin of
11.72 Hz from 0 Hz: 257 bins (up to 3 kHz) in narrow mode, 641 (up to 7.5 kHz) in wide. A byte
`b` is `-127.5 + 0.5 * b` dBFS. A binary frame is an 8-byte header (stream 1, format 1, the
row's sequence number mod 65536, the low 32 bits of its first sample, little-endian) followed by
the bins. After a `Gap` the next row starts at the first sample after it.

## Constellations

Each received burst with a payload is followed by a binary frame on stream 2 holding its
equalized payload symbols, so a client can draw the burst's constellation. `Hello` describes
the frames in `vfm_constellation` (stream, format, scale, most points per frame), a key a
client of the shared messages ignores. A client skips binary frames whose stream id it does not
know.

The frame starts with the same 8-byte header as a waterfall row; its `t_low` is the burst's `t`,
as in its `VfmBurstRx`. Then come 8 bytes: speed level, codewords, scale (the value of 1.0,
48), bits per symbol (1 BPSK, 2 QPSK, 4 16QAM, 6 64QAM), point count (`u16` little-endian),
data carriers per OFDM symbol, and 0. Then 3 bytes per point: in-phase and quadrature as `i8`
(divide by the scale; a constellation has unit mean energy, so 64QAM's outer points sit at
1.08) and the carrier, counted from the lowest of the mode's carriers, pilots included.

The points are whole OFDM symbols in order. A burst with more than 2048 points keeps every
k-th symbol, so every carrier stays in view. The symbols are copied out of the receiver only
while `--api` is on.

## Status fields vardeFM does not fill

These are part of the shared `Status`, and vardeFM always sends them as 0 (or `unknown`):

| Field | Why |
|---|---|
| `resample_cpu` | the resampler runs in the sound card's callback, which is not timed |
| `device_xruns`, `xrun_frames`, `device_errors` | the sound card's own overruns and other errors are printed (`audio input error: ...`), not counted |
| `events_dropped`, `rows_dropped` | events and rows go to the server on the radio's own thread, with no queue in between to drop from |
| `capture_priority` | the capture thread's scheduling is not measured (`unknown`) |
| `link.rx_queued` | received data goes to the host as it arrives |

`cpu` is the time spent stepping the modem, and `tap_cpu` the time spent making waterfall rows and
queueing them for the clients, each as a percentage of the audio received in the same second.
`longest_push_ms` is the longest single modem step in that second. A step takes in the host's
commands and data, runs the receiver and the link, and, when a burst starts, scales its audio and
with `--record` writes it to disk, so with `--record` a long step can be disk time. `overruns` and
`dropped_frames` count the audio chunks, and their samples, dropped because the modem fell
behind; `gaps` counts the `Gap`s sent. `client_rows_dropped` counts every binary frame a slow
client missed, constellations included: they share the waterfall's queue of 32 frames.
