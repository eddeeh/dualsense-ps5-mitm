# Tests

The pure functions only: byte transforms, checksums, the packet cursor, the
ACL credit accounting, and the bytes the USB gadget hands FunctionFS
(`src/usb/HidGadget.h`). No sockets, no UDC, no radio — so they run anywhere
the project builds, with no pad and no console attached.

    cmake --build build --target tests && ./build/tests

Built with `-fsanitize=address,undefined`. That is deliberate: the class of bug
these are aimed at is reading past the end of a packet, which assertions do not
notice and ASan does.

The reverse happens too, so the checks are not redundant with the sanitizer.
`HeldFrame::store()` writes into an array with a length field right behind it;
take its bounds check away and the overflow lands inside the same object, which
plain ASan does not flag. `an_oversized_report_is_refused_rather_than_truncated`
does - verified by removing the guard and watching it fail.

## Fixtures are real reports, not invented ones

`fixtures.h` is generated, and it holds actual DualSense reports lifted out of a
btsnoop capture:

    python3 tools/extract_test_fixtures.py logs/capture-YYYYMMDD-HHMMSS.dump 4

This matters more than it looks. `src/dualsense/Transforms.h` and `DualSense.h`
encode *a reading* of the DualSense report layout, worked out from captures. A
test written from that same reading would agree with itself no matter how wrong
the reading was. Checking against bytes the pad signed itself is the only way
the tests can disagree with the code.

The extractor verifies each report's CRC32 with an independent implementation
(Python's zlib) before writing it out, so a capture that does not parse cannot
quietly become a fixture. It collects both kinds of frame — controller state and
the opaque stream that shares report id 0x31 — because a test that only ever
sees one cannot show that the two are told apart.

Pick a capture from a session where the console was actually playing if you want
opaque frames; they only appear once the console takes the play link.

## Replaying a whole capture

The unit tests carry eight reports. The captures under `logs/` carry millions,
and some questions only they can answer:

    cmake --build build --target replay_capture
    ./build/replay_capture logs/capture-YYYYMMDD-HHMMSS.dump

It walks every record through the same `src/bluetooth/PacketContents.h`,
`src/dualsense/Transforms.h` and `DualSense.h` the relay uses, and checks what
must always hold: every checksum, `feature_report_size()` against the sizes the
pad actually used, and - the reason it exists - whether the bounds checks ever
reject a packet a real controller sent.

Not part of `tests`, because captures are hundreds of megabytes and are not in
the repository.

Measured over the twelve largest captures, 13.7 million input reports:

| | |
| --- | --- |
| checksums verified | 13744664 of 13744669 |
| HCI events parsed, none short of their structure | 2821089 |
| L2CAP payloads declaring more than arrived | 0 |

The five checksum failures are the air, not the arithmetic: one in 2.7 million,
scattered across two captures, in reports that are otherwise perfectly formed.
The pad keeps its own count of exactly this in `BtCrcFailCount`. So the tool
judges checksums as a rate rather than a count - a wrong seed or offset fails
every report, noise fails a handful, and the two are nowhere near each other.

### The ACL accounting, replayed against a real session

btsnoop records the direction of every packet, so a console-side capture carries
the exact sequence of sends and `Number of Completed Packets` events the relay
saw. `replay_capture` feeds that through the real `AclFlowControl` and checks it
reconciles - across three sessions, 616989 packets sent and 616989 buffers
freed, nothing outstanding at the end, no completion for a packet never counted,
and a peak in flight of 5 to 7 against a pool of 8.

It also measures the completion interval, which is the number the depth-2 choice
rests on: the 8.0 ms recorded earlier holds, and the 250 ms figure quoted
elsewhere is the depth-1 behaviour rather than something the controller does
regardless.

## What is not covered

- Anything needing a radio, a gadget or a console: the whole of
  `BluetoothHandler`'s state machine, `USBHandler`'s configfs work, the L2CAP
  and HCI conversation.
- `AclFlowControl`'s three-second stall write-off, which is wall-clock based.
- `has_room()` reads `DS_ACL_DEPTH` once into a function-local static, so the
  budget tests assume the default of 2 and cannot vary it in-process.
