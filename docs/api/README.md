# Monitor API

`vardefm modem --api 127.0.0.1:8770` serves a read-only view of the modem: its counters, what it
sends and hears, and a waterfall of the received audio. It listens on 127.0.0.1 or [::1] only,
and answers only requests whose `Host` (and `Origin`, if any) is a loopback name, so a web page
cannot reach it through the browser.

- `GET /v1/status`: the latest `Status`, the last 64 shared events (`recent`) and the last 64
  vardeFM events (`vfm_recent`), as JSON ([status.schema.json](status.schema.json)).
- `/v1/stream` (WebSocket): `Hello`, then the latest `Status`, then events and a `Status` every
  second as JSON text, and waterfall rows as binary frames. Each text message is one envelope
  ([envelope.schema.json](envelope.schema.json)). The stream is read-only: a message from the
  client closes it. At most 8 clients.

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
  the sync symbols), `t_end` just past its last symbol.
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

## Status fields vardeFM does not fill

These are part of the shared `Status`, and vardeFM always sends them as 0 (or `unknown`):

| Field | Why |
|---|---|
| `resample_cpu` | the resampler runs in the sound card's callback, which is not timed |
| `device_xruns`, `xrun_frames`, `device_errors` | the sound card's own overruns and other errors are printed (`audio input error: ...`), not counted |
| `events_dropped`, `rows_dropped` | events and rows go to the server on the radio's own thread, with no queue in between to drop from |
| `capture_priority` | the capture thread's scheduling is not measured (`unknown`) |
| `link.rx_queued` | received data goes to the host as it arrives |

`cpu` is the time spent stepping the modem (receiver, link and transmitter) and `tap_cpu` the time
spent making waterfall rows, each as a percentage of the audio received in the same second;
`longest_push_ms` is the longest single modem step in that second. `overruns` and
`dropped_frames` count the audio chunks, and their samples, dropped because the modem fell
behind; `gaps` counts the `Gap`s sent.
