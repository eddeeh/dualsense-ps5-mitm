# What the wire actually says

The relay passes everything between the pad and the console byte for byte, so in
principle it never had to understand any of it. In practice almost every part of
it exists to get past something specific the console does, and working those out
meant reading the traffic. This is what the captures showed.

Everything here is checked against real dumps, and the report layouts are
cross-referenced with the kernel's `hid-playstation.c`, which the DualSense
matches where noted. The constants live in [`src/dualsense/DualSense.h`](src/dualsense/DualSense.h);
the parsing is in [`src/dualsense/Transforms.h`](src/dualsense/Transforms.h),
and [`replay_capture`](tests/replay_capture.cpp) walks a whole capture back
through both.

## The reports

Feature reports are a request-and-answer channel over the control interface. The
console asks (`GET`) or writes (`SET`); the pad answers.

| Report | Dir | What it is |
|---|---|---|
| `0x01` | in | the cut-down input report, sent until the host asks for calibration or firmware |
| `0x31` | in/out | the full input report, and every output report |
| `0x05` | GET | IMU factory calibration |
| `0x09` | GET | `[pad MAC][08 25 00][host MAC]` — the pad's address and its current host |
| `0x0b` | GET | pad MAC + the host table, four ranked entries |
| `0x20` | GET | firmware / build date |
| `0x0a` | SET | the console's Bluetooth address and a 16-byte link key |
| `0x08` | SET | transport: `0x02` unpair, `0x01`/`0x11` pair (“come to me over Bluetooth”) |
| `0x80`/`0x81` | SET/GET | a command channel with multi-part answers |
| `0xf0`/`0xf1`/`0xf2` | SET/GET | the authentication handshake |

The three bytes `08 25 00` inside `0x09` and `0x0b` are the class of device, the
same value the adapter is handed by `write_class_of_device`.

The pad starts a fresh link sending the cut-down `0x01` report — sticks and
buttons, no sensors — and only switches to the full `0x31` once the host has
asked for calibration (`0x05`) or firmware (`0x20`). A host that has asked for
neither does not expect `0x31` and drops it, which is why the USB registration
side has to serve those before anything else.

## Two transports, two framings

The same report looks different depending on which wire it is on, and the relay
translates between them.

**Over Bluetooth** a report is always its full declared size and the last four
bytes are a CRC32. The seed is not zero: it is the HIDP transaction header byte,
`0xa1` for input, `0xa2` for output, `0xa3` for a feature answer, `0x53` for a
feature the console sets. So the checksum is `crc32(header || report)`, which is
what `crc32_le()` in the kernel does with `PS_INPUT_CRC32_SEED` and friends.
Verified: `crc32(0xa3 || report)` of the pad's own `0x09` answer reproduces the
four bytes it sent.

**Over USB** the console sends short transfers and no checksum. Relaying a
Bluetooth report verbatim onto USB therefore earns `ERR_INVALID_PARAMETER` — the
sizes are wrong and the trailing CRC is unexpected. The gadget rebuilds each
report in the USB framing instead.

One consequence caught the registration side out early: the USB HID report
descriptor has to declare **every** report the console will request, because a
report absent from the descriptor cannot be asked for at all. The real
descriptor is 289 bytes; an earlier 273-byte version was missing feature reports
`0x0b` and `0x0c`, and the console stopped reading exactly where the descriptor
ran out.

## The input report, nailed down

The full input report body is 63 bytes. Over Bluetooth it is framed
`[0x31][seq_tag][body 63][9 bytes of pad][crc32]` — the nine bytes before the
CRC are zero in every one of thousands of checksum-verified reports, so nothing
lives there. Sticks, buttons and triggers are at the front; the IMU, touchpad
and a status byte follow.

The layout is not taken on faith. Feature report `0x05` is the factory
calibration — 41 bytes, exactly `DS_FEATURE_REPORT_CALIBRATION_SIZE` — and
working it through the kernel's arithmetic gives a real calibration: full scale
±2001, ±1999 and ±1997 °/s on the three gyro axes, agreeing to 0.2%, and ±4 g on
the accelerometer. Applying those numbers to 18 950 input reports from the same
session, keeping only the ones where the pad was near still:

    |acceleration| = 1.0029 g

2.9 mg off exactly one gravity. That one number checks the calibration parse, the
accelerometer offsets inside the input report, and the framing all at once.

Two clocks are embedded, and both are useful.

- The pad stamps each report at `body[27..30]` with a **3 MHz** counter (28 ppm
  off, zero backward steps over 13 856 reports once the opaque frames below are
  filtered out). Its steps are quantised to a 1.6 kHz grid, so the stamp is the
  time of an IMU sample, not of transmission.
- The console stamps each output report with a **1 MHz** counter, whose steps
  come in blocks of 512 samples at 48 kHz — one 10.67 ms audio frame per report.

Because neither clock has anything to do with capture timing, a pad-side and a
console-side capture taken together measure exactly what the relay adds, end to
end, with no instrumentation inside the relay. That is the ruler
[`PERFORMANCE.md`](PERFORMANCE.md) uses.

## A second stream hides in the input channel

About one “input report” in six is not one. Mixed in with controller state, at
the same report id `0x31` and with a valid CRC32, the pad sends 79-byte frames
whose body is incompressible:

    a1 31 <seq_tag> <counter> d4 <70 bytes of high-entropy data> <crc32>

A byte-wide sequence counter, a fixed `0xd4` marker, payload entropy of
5.87 bits/byte — on a controller with a microphone, this is a coded stream, not
controller state. Two facts pin it down:

- **The console turns it on.** It is exactly 0% of the pad's traffic before the
  console owns the controller, and a steady 17-18% afterwards, switching on
  inside the window where the play link is encrypted.
- **Its rate is fixed.** 100.0 frames per second with no variation at all —
  500 per five-second sample, every sample — running beside the ~600/s state
  reports rather than as a fraction of them.

This matters to anything that reads the input stream field by field: a sixth of
it is not what it looks like. The battery byte, for one, read as garbage about
7% of the time until this was understood — those were not corrupt reports, they
were correct frames of something else.

The `0xd4` marker is *not* a clean way to tell them apart: in a real report that
byte is the right stick's Y axis, and `0xd4` is an ordinary value for it. The
clean discriminator is the flags byte between the report id and the body — bit 0
is HasHID, set on a state frame and clear on an opaque one.
`transforms::carries_controller_state()` reads it. Measured with `replay_capture`
over 13.7 million CRC-verified reports, the flag and the marker disagree 43 times
in 13.7 million, and every time it is the marker throwing away real input whose
right stick sits at `0xd4`. The flag never does.

The relay drops these frames toward the console rather than spend the input
stream's flow-control credits on them — losing four frames in five would leave a
codec nothing anyway, and the credits are better spent on fresh stick positions.
See `PERFORMANCE.md` for why that is safe.

## The console's output stream

The console's output reports are relayed to the pad blind, so it is worth knowing
what is in them. Once the link is up there are two:

| report | ACL size | what it is |
|---|---|---|
| `0x36` | 403 | the workhorse, ~90 Hz |
| `0x39` | 552 | less often, in short bursts |

Both embed one audio block that begins `91 06 7f 3d 1f 50 28`, at a fixed offset,
carrying a counter that is **shared between the two report types** — interleaving
them by arrival time gives a single clean sequence, which two independent
counters would not. The block is an array of 200-byte frames with data at frame
offset +10; `0x36` carries two frames, `0x39` three.

In a long menu session, every one of 153 250 `0x36` reports carried the two-byte
value `f4 ff` — that is `-12` as a little-endian int16, i.e. silence. So a
35 KB/s wall of 403-byte packets can be entirely padding, and the way to tell a
live stream from an idle one is the sample bytes, not the packet size. The relay
cannot ask the console to slow this down and must relay it faithfully: a
controller that ignores the console's output is not one the console will keep
taking input from. This was learned the expensive way.

`0x39` is haptic audio — the DualSense drives its two voice-coil actuators and
its speaker from the same 48 kHz stream. It correlates strongly with the holes in
the input stream (see `PERFORMANCE.md`), which is the one place this matters to a
player rather than to a parser.

## The host table, and a warning

Report `0x0b` is the pad's list of hosts it trusts:

    a3 0b | pad MAC (6) | class of device (3) | 4 × [rank][host MAC (6)] | crc32

The leading byte of each entry is a **rank**, not a flag: across 114 verified
copies the non-empty entries always hold a permutation of {0, 1, 2}, and rank 0
is the most recently used host. The table holds three.

This has a consequence worth stating before it is diagnosed the hard way. The
console-facing radio claims the pad's own address, so a fresh bond to it takes a
*new* slot rather than updating the console's existing entry — two bonds to one
MAC coexist because they carry different link keys. After enough relay runs the
console is pushed out of the table entirely. When that happens, taking the
controller back to the PS5 directly needs a re-pair, and from the outside it
looks exactly like a controller that has stopped working. It has not.

## The command channel

`0x80`/`0x81` is a request channel the console uses, relayed blind. The shape is
`53 80 <cmd> ...` to ask and `a3 81 <cmd> <status> ...` to answer, command echoed
back. Two commands appear: `01 13` reads the serial number in one answer, and
`70 11` starts something that takes nine seconds, polled once a second with
status `03` (more to come) until `02` ends it. These are the pad's firmware and
manufacturing commands; psdevwiki documents the set - Get Firmware Info, Get/Set
BT Address, NVS lock/unlock, Set DFU Mode and dozens more - at
[DualSense HID Commands](https://www.psdevwiki.com/ps5/DualSense_HID_Commands).
For a relay the point is only that every part has to be forwarded or the
console is left polling a sequence that never finishes — which is why the control
channel is never flushed and only input reports are marked flushable.

## The authentication handshake

Reports `0xf0`/`0xf1`/`0xf2` are a challenge-response attestation the pad performs
for the console, relayed but not understood. Its shape is fully readable, and
reading it explains why a Bluetooth relay completes the handshake where a USB
gadget proxy cannot.

All three share `[HIDP][reportID][op][session][page][payload][CRC32]`, with the
same CRC32 the input reports use. One round:

```
console -> pad : SET 0xf0 op=01 pages 0..3   upload a ~180-byte nonce
   status       : 0xf2 state 0x12            response ready
pad -> console : GET 0xf1 pages 0..3         certificate + signature over the nonce
console -> pad : SET 0xf0 op=02              finalize, 16 fresh bytes
   status       : 0xf2 state 0x40            authenticated
```

The status bytes have names. The community mapped them against Sony's own PSVR2
kernel sources, which run the same handshake: `0x01` initial, `0x11` executing,
`0x12` response ready, `0x20` waiting on the host's response, `0x40`
authenticated, `0x51`/`0x52` for the periodic re-auth, `0x80` error. And the
check is **bidirectional** - the pad also challenges the console, which is why
the handshake can only be relayed, never faked (GIMX #672; see the README's
Credits). Reports `0xf4`/`0xf5`, which share the `0xf` prefix, are not part of
this - they are the firmware-update (DFU) channel.

The `0xf1` response is dominated by a 128-byte block that is byte-identical in
every one of 48 handshakes — a per-device certificate, with a constant key id
`0002000103247FCA` that names the key rather than hiding it. Only about 32 bytes
change with the nonce: the signature proving the pad holds the matching private
key. (The response tail even leaks uninitialised SRAM — little-endian pointers
into the `0x2004xxxx` Cortex-M region — where the pad's buffer was not cleared.)

The nonce is fresh every time: across 51 handshakes, all 51 page-0 challenges are
distinct, so a captured response is worthless on the next connection. And the
console re-runs the whole thing every few minutes for the life of the link — the
session counter in byte [3] climbs 01 → 02 → 03 over about seven minutes — with a
one-page, 16-byte re-challenge (`op=03`) in place of the heavy initial one. A
relay has to keep passing it through the whole time, which this one does: in one
session all 9 challenge chunks and all 6 response chunks crossed byte for byte.

This is the exact point where the USB gadget route fails and the Bluetooth one
does not. A USB proxy reads `0xf2`, sees “not ready” forever, and the console
never asks for `0xf1`. Over Bluetooth the same poll returns an advancing state
and the round completes. Whatever the authentication IC expects over USB, the
Bluetooth path does not trip it.

## What a relay can and cannot reach

Every input report is signed with an AES-CMAC tag keyed by a session key that is
established through the handshake above and never appears on the wire, in the
kernel driver, or in any console key dump. (That the tag is AES-CMAC is
corroborated by a leaked Sony tool, `Dualsense_checker`, reported in GIMX #672;
over USB the same tag covers only the report's first 16 bytes, and blocking the
`0xf0` challenge makes the console accept tampered reports for about two minutes -
direct evidence the tag is tied to the session key the handshake sets up.) That is why the relay passes
everything through untouched: editing a report breaks the tag, replaying an old
one fails the console's freshness check, and the legacy unsigned `0x01` format is
refused mid-game. The relay reaches everything short of that key — the report
layouts, both checksums, the whole attestation handshake, and the console's exact
behaviour — and passes it all faithfully, which is all a logger needs and all a
relay can do.
