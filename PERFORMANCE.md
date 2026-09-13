# Where the time goes

Two separate questions, and they have very different answers. The CPU is not a
problem and has not been one since the profiling round below. The holes in the
input stream are real, they are felt, and this document is mostly about finding
out what causes them.

Everything here is measured on a live session - a controller connected, the
console playing through it - not on synthetic load. The session behind the stall
numbers is 2.4 hours long and its capture is `logs/capture-ps5-*.dump`.

## The CPU is not the problem

    608 pad reports/s in, 88 console output reports/s back
    0.90% of one core
    14.8 us per pad report (11.5 user, 3.3 system)

Three fixes got it there, in descending order of how much they mattered: the
read loop drains the socket with non-blocking `recv` after each wakeup instead
of awaiting one `async_read_some` per packet; `send_input` is a plain function
over a stack buffer instead of a `vector` handed to `co_spawn` (measured in
isolation, 0.39 us per report against 0.003); and `acl_has_room()` reads a
running counter instead of summing a `std::map`. Together they took 26.5 us per
report down to 14.8 and cut system time by two thirds.

Nothing here is near a limit. What follows is not about throughput.

These figures are from that profiling round and have not been re-taken since;
the code paths they name are unchanged. The stall numbers below, by contrast,
are all from the 2.4 hour session named above and are reproducible from its
capture.

# The input path

The CPU cost above is what the relay spends. This is what it *adds*, which is a
different number and had never been measured. Both radios write a capture, one
kernel timestamps both, and the relay forwards a full `0x31` report byte for
byte - so every send can be matched to the arrival that produced it:

    python3 tools/analyse_input_path.py logs/capture-<stamp>.dump \
                                        logs/capture-ps5-<stamp>.dump

Matching is on the pad's own 32-bit counter, not on the first body bytes: those
are sticks and buttons, identical for long stretches while the pad is still. Over
the 2.4 hour session it matches 1849737 of 1850051 sends, or 100.0%.

## The relay adds 65 microseconds, most of the time

    arrival on the pad radio -> write to the console radio
      p50    0.065 ms
      p75    3.525 ms
      p90    4.569 ms
      p99    6.596 ms
      mean   1.856 ms

Bimodal, and the two halves are two different things. Half the reports go out
in under 0.5 ms: a credit was free when they landed, and the software cost of
forwarding one is 65 microseconds. The rest are the ones held because the budget
was full, and their 3-5 ms is the console's polling interval, not ours.

The held slot is doing its job as well as it can be done. 36.2% of sends were
already superseded by a fresher report at the moment they went out, which sounds
bad and is not: the freshness given up averages **0.0036 ms**, because the
superseding reports arrived in the same burst, microseconds apart. There is
nothing left on the table here and the code should be left alone.

## The pad-facing dongle holds input for a fixed 8 ms

This is the one real finding, and it is larger than everything the relay does.

Input does not arrive from the pad as a stream. It arrives in bursts of five,
spread 2 microseconds apart inside a burst, one burst every 8 ms:

    arrivals sharing a timestamp with the one before   37.3%
    bursts of 5 reports (p90 6), spread 2 us inside
    burst period: median 7.9970 ms, p25 7.9790, p75 8.0000

Nothing is lost - 16.8% of arrivals are frames of the opaque stream, and across
the rest the pad's counter advances by exactly one 100.00% of the time. The
reports are all there. They are just late, together.

**It is a timer, not the data.** Burst starts are locked to an absolute 8.000 ms
grid in the host's clock with a Rayleigh concentration of R=167.6, where chance
is about 1. A burst released as its packets land would sit at no fixed phase. So
the freshest report in each burst waits a mean of 4 ms before the relay can see
it at all - twice what the relay itself adds, and invisible to every measurement
taken inside the process.

**It is the transport, not the air.** The control is in the same capture, at the
same instant, on the same kernel: the console-facing radio is the Pi's onboard
part on a UART, and it receives with no batching whatever.

| | pad radio (USB dongle) | console radio (onboard UART) |
| --- | --- | --- |
| arrivals sharing a timestamp | 37.3% | **0.0%** |
| burst size | 5 (p90 6) | 1 |
| locked to a grid | R=167.6 | R=0.7 |

Completion events tell the same story from another angle: they come from the
same dongle over the interrupt endpoint rather than bulk, and they are spread
uniformly across the 8 ms cycle - a flat histogram in 32 bins. So it is not the
USB link as a whole being serviced on a timer, and not the driver's wakeups. It
is specific to how bulk IN data from this part reaches the host.

What this does not identify is which layer holds it - the dongle's firmware, its
bulk endpoint's servicing, or something in btusb. An HCI capture cannot see
below itself, and the three look identical from here.

### It is not the dongle - tested

The prediction above was that the 8 ms grid might be a property of the Realtek
part, and that moving the pad to the Broadcom would remove it. It does not.

`tools/bench_radio_delivery.py` settles it without a pad, a console, or any
pairing: it drives one adapter with an L2CAP stream from another at the pad's
own rate, captures the receiving side, and reports how the packets are handed
over. Three adapters, ~390 packets/s inbound on each, same sender, run back to
back:

| receiving adapter | sharing a timestamp | burst | period | grid |
| --- | --- | --- | --- | --- |
| Broadcom BCM20702A0, USB | 30.2% | 3 (p90 4) | 7.999 ms | R=70.5 |
| Realtek RTL8761B, USB | 23.5% | 3 (p90 4) | 7.998 ms | R=75.5 |
| **onboard, UART** | **0.0%** | **1** | **2.500 ms** | R=4239 |

The two USB parts are indistinguishable, so the chip is not it. The onboard
radio hands over every packet on its own, and its 2.500 ms cadence is the air
schedule itself - four slots - which is what a transport that adds nothing looks
like.

So the 8 ms belongs to the host's USB path for these full-speed adapters, not to
either dongle. Which layer inside it - the parts' shared handling of bulk IN,
xHCI's servicing of a full-speed endpoint, or btusb - this does not say, and
`btusb` exposes no parameter that touches it.

### The fix is to move the pad off USB

The onboard radio is the only one here that does not batch, and the input stream
is what needs it. The console side does not: what it receives from the console
is rumble and haptic audio at 88/s, and 8 ms of delay on those is not felt.

The transmit direction is unaffected, which the same bench shows in passing: in
the run above the sender was a USB dongle and the UART receiver saw a clean
2.500 ms cadence, so nothing holds packets on the way out. Only bulk IN is
batched.

That makes the swap a straight win, and the hardware for it is already here -
`/lib/firmware/brcm/BCM20702A1-0b05-17cb.hcd` is installed and the BT400 comes up
as `BCM20702A1 (001.002.014) build 1467`, so it can take the address change the
console side needs. In `/etc/dualsense_mitm.adapters`:

    PAD=D8:3A:DD:9E:8B:F9      # was the USB dongle; now the onboard radio
    PS5=5C:F3:70:AA:61:3F      # was the onboard radio; now the BT400

Before the first run, check that the onboard radio still answers to the address
above. It may not: whichever adapter last served the console side keeps the pad's
address, written by a vendor command into the controller's RAM, and that survives
the relay exiting and an HCI reset. `run.sh` pins adapters by address and will
report the pad adapter as "not present" while the leftover is in place. README's
troubleshooting section has the command to put it back.

**It costs a re-pairing.** The pad stores the address of the host it belongs to,
and that address changes, so the first run after the swap needs
`DS_FRESH_PAIRING=1`. The console's own registration is tied to the pad's
spoofed address rather than to the adapter, so it should survive - but that is
reasoning, not a measurement, and the swap has not been run end to end.

### Measured after the swap

Run on 12 Sep at 15:39, pad on the onboard radio, console on the BT400. Fifteen
minutes, 570528 pad reports, 298073 sends matched to their arrival.

| | pad on USB | pad on the onboard radio |
| --- | --- | --- |
| arrivals sharing a timestamp | 37.3% | **0.0%** |
| delivery grid | 8.000 ms, R=167.6 | 2.544 ms, R=15.7 |
| what the grid is | a transport holding packets | the air, and barely a grid at all |
| wait before the relay sees a report | ~4.0 ms | none beyond the air |
| relay latency p50 | 0.065 ms | **0.046 ms** |
| relay latency p90 | 4.569 ms | **1.843 ms** |
| relay latency mean | 1.856 ms | **0.566 ms** |
| sends already superseded | 36.2% | **7.5%** |

The 8 ms is gone. What is left is the link exchanging on slot pairs, with every
packet handed over on its own - and the concentration has collapsed from R=167
to R=15.7, so it is not even a sharp grid any more, just the cadence of the air.

Throughput to the console moved too: 214.5 sends/s to **330.2/s**, median gap
4.16 ms to 3.05 ms, p99.9 134.42 ms to 25.27 ms. None of that is attributable:
the swap moved both radios at once, so the console link is now the BT400 rather
than the onboard part and its share is unmeasured. Running the pad on the
onboard radio with the console back on a Realtek dongle would separate them, at
the cost of another re-pairing.

### What the swap did to the CPU: the same total, a different shape

Measured live on the running relay, 45 s, counters read from `/proc`:

| | pad on USB | pad on the onboard radio |
| --- | --- | --- |
| pad reports | 608/s | 738/s |
| of one core | 0.90% | 1.02% |
| per report | 14.8 us | **13.85 us** |
| of which user | 11.5 us | **4.82 us** |
| of which system | 3.3 us | **9.03 us** |

The total barely moved. The split inverted, and the reason is the batching that
was removed: 931 voluntary context switches a second now against roughly 250
before. The 8 ms grid that cost 4 ms of freshness was also amortising five
reports over one wakeup; one report per wakeup is fresher and three times the
syscalls. System time per report tripled to pay for it.

At 1% of a core that is an easy trade, and worth stating as a trade rather than
a free win. The user-time figures are not a clean comparison - the 11.5 us came
from the earlier profiling round on older code - but the system-time story
stands on the wakeup count, which was measured directly.

**That comparison is not valid, and the reason was found later.** The 13.85 us
was taken on a relay built with no optimisation at all: an editor had
reconfigured `build/` as a Debug build, and `run.sh` built whatever `build/` was.
The 14.8 us it is set against came from the earlier round, which profiled a
`RelWithDebInfo` build - `-O2`. So "the same total" compares -O0 with -O2 and says
nothing. The system-time half of the argument - three times the wakeups, three
times the syscalls - is kernel work that optimisation barely touches, and stands.
The user time is certainly lower than 4.82 us at -O2. `run.sh` now builds `-O2`
in its own `build-release/`; the figures in this section need re-taking on it.

### The path back to the pad is instant; its problem is loss, not delay

The same join in the other direction - every output report the console sent,
matched to the write that forwarded it to the pad by hashing the whole payload
and taking the nearest earlier arrival. 231639 of 231781 forwards matched, 99.9%:

    console radio in -> pad radio out
      p50    0.019 ms
      p90    0.033 ms
      p99    0.078 ms
      mean   0.023 ms

Nineteen microseconds, and 99.9% of forwards inside half a millisecond. Rumble,
trigger effects and haptic audio are passed straight through in the handler that
receives them - there is no held slot and no waiting for a credit, because when
the budget is full the report is dropped instead. So this path has no latency
question at all. It has the 3.9% loss described above, and nothing else.

### Time to first input is almost entirely the human

Milestones from both captures, relative to the first record:

| | 09:55 session | 16:31 session |
| --- | --- | --- |
| pad link up | +71.78 s | +1.20 s |
| first report from the pad | +71.94 s | +1.63 s |
| we call the console | +73.96 s | +3.63 s |
| console link up | +78.08 s | +6.47 s |
| **first input delivered** | **+81.33 s** | **+9.86 s** |
| page attempts to the pad | 17 | 1 |

The eighty seconds and the ten are the same relay. The difference is entirely
whether the pad was awake: 17 pages that timed out against one that answered,
which is the wait for someone to press PS. Once the pad answers, the relay takes
**8.2 s** in the new arrangement and 9.4 s in the old to get input flowing, and
most of that is not ours either - the console registers over USB at its own
pace, and one second of it is the deliberate delay before the synthetic PS
press.

There is no obvious waste here to remove. Worth knowing mainly so that a slow
start is not mistaken for a regression: check the page-attempt count first.

### The swap costs 4% of the output stream, and why

The input path got better; the path back to the pad got worse, and the counters
say so plainly:

| | pad on USB | pad on the onboard radio |
| --- | --- | --- |
| output reports delivered to the pad | 761038 | 196462 |
| dropped because the budget was full | 1002 | 7967 |
| **share dropped** | **0.13%** | **3.90%** |

Thirty times more rumble, trigger and haptic frames thrown away. The throughput
is not the cause - 88.3 writes/s before, 84.9 after - the completion *shape* is:

| | pad on USB | pad on the onboard radio |
| --- | --- | --- |
| completion events | 88.2/s | 42.3/s |
| packets credited per event | **always 1** (761473 of 761473) | **always 2** (100021 of 100026) |
| interval between them | p50 10.00 ms | p50 20.04 ms |

The same credits come back at half the rate in pairs instead of one at a time.
With a budget of two that doubles the stretch in which `has_room()` is false,
and `relay_console_output_to_pad` drops rather than queues - deliberately, since
an effect delivered late is worth nothing. So twice the blocked window is thirty
times the drops.

What this exposes is that `constexpr int depth = 2` in `AclFlowControl.h` is one
constant serving two links whose dynamics have nothing in common. On the console
link we are the slave and the number is about not queueing stale input behind a
console that polls when it likes. On the pad link **we are the master**: we page
the pad, we transmit when we choose, and the controller is holding 8 buffers of
which we use 2. Nothing about the console's argument applies there.

#### What was done

The depth is per link now: `AclFlowControl::set_depth()`, left at two and raised
to three for the pad-facing side in `BluetoothHandler`'s constructor.

Three rather than four, and the number was measured rather than picked. Counting
the console's output into 20 ms windows across a 50 minute session gives what
each depth would cost:

| windows holding | count | depth | packets it would drop |
| --- | --- | --- | --- |
| 1 report | 28136 | 2 | **3.68%** |
| 2 | 105435 | 3 | **0.25%** |
| 3 | 8598 | 4 | 0.04% |
| 4 | 469 | 5 | 0.00% |
| 5 or more | 97 | | |

The model says 3.68% at the current depth and the counters say 3.90%, which is
close enough to trust the rest of the table. Three removes 93% of the loss for
at most one extra window of queueing; four removes 99% for two, and a haptic
frame delivered late is worth little. If the remaining 0.25% ever matters, four
is one character away.

The stats line was wrong about this too and is fixed in the same place: it
printed `acl 2/8`, the in-flight count against the controller's whole pool, so a
link pinned at its cap read as having six buffers spare. It prints the budget
now, and `acl 2/2` says what is actually happening.

Not yet verified on hardware - the run that measured the problem is still going
on the old binary. The `out ... held` counters answer it on the next start, with
no capture needed.

### The channel map is not it either, measured

The relay now reads the AFH channel map from the console link every five
seconds, alongside link quality and RSSI. It looked like the answer at first:
link quality 255, RSSI a few dB, and 48 of 79 channels struck out - a radio that
is perfect on the channels it may use and has too few of them.

It is not the answer.

**The router's 2.4 GHz band made no difference.** With it switched off the map
still sat between 21 and 51 of 79. And the excluded channels do not look like a
Wi-Fi network: a fixed access point strikes out one steady block, while these
wander across the whole band from one five-second snapshot to the next - low end
out, then high end, then the middle - and twice recovered to 79/79 on their own
with our traffic unchanged.

**Our own two radios are not it.** In the 21:21 session both links ran at full
load - about 730 packets a second on the pad link, 320 on the console link - from
+15 s to +45 s, and the map stayed at 79/79 throughout. The collapse came at
+100 s with the traffic exactly as it was.

**And the map does not predict the holes.** Across 65 five-second windows from
two sessions, counting stalls against the channel count read at the end of each:

| AFH channels | windows | stalls per window | 0x39 packets per window |
| --- | --- | --- | --- |
| 70-79 | 12 | 0.92 | 4.6 |
| 55-69 | 10 | 0.00 | 2.0 |
| 40-54 | 24 | 0.75 | 5.9 |
| 0-39 | 19 | 2.37 | 16.4 |

Not monotonic, and the one bucket with many stalls is the one with the most
`0x39`. Stalls against channel count correlate at r = -0.37; stalls against
`0x39` packets at **r = +0.89**. The apparent effect of the map is the audio
stream showing through it. Why a narrow map and a busy stream coincide is not
settled here; that they do, and that the stream is the stronger of the two by a
wide margin, is.

### The holes survive the swap, measured

The 15:39 session carried no haptic audio at all and so could not test them. The
one started at 16:31 could: 11 minutes, 5.1% of it audio, and the holes came
straight back.

| | pad on USB, console onboard | pad onboard, console on BT400 |
| --- | --- | --- |
| session | 2.4 h | 11 min |
| stalls | 0.442/s | 0.267/s |
| duration median | 134.3 ms | 138.3 ms |
| audio, share of session | 5.8% | 5.1% |
| **stall rate with audio** | **6.01/s** | **3.59/s** |
| **stall rate without audio** | **0.10/s** | **0.09/s** |
| console traffic inside a stall | 19% of baseline | 16% |
| burst span, median | 417 ms | 391 ms |
| dead inside a burst | 97.6% | 99.5% |
| packets between back-to-back stalls | 2 (87.6%) | 2 (80.6%) |

Every structural number reproduces: the fixed ~135 ms, the bursts, the two
packets that squeeze through, the console falling silent, and above all the
forty-fold difference between audio running and audio idle. Moving both radios
changed none of it, which is what the cause being the console's own scheduler
predicts.

The absolute rate is lower - 0.267/s against 0.442/s - and that is **not** a
result. Eleven minutes of one game against 2.4 hours of another is not a
comparison, and the audio share differs too. The rate to compare is the one
against audio, and 3.59 against 6.01 rests on 33 seconds of audio in the new
session.

So the swap is a win on the input path and does nothing for the holes, exactly
as the measurement predicted. What is left for them is still the console-side
setting.

### One new stall, and it is ours

At 1138 s into that session the relay sent nothing for **1021 ms**. It looked at
first like the console side misbehaving on its new adapter. It is not. Traced
event by event on the console radio:

      -5.0 ms  Number of Completed Packets, 2
      -5.0 ms  we send
      -3.8 ms  we send                        in flight 2, budget spent
      -0.0 ms  Number of Completed Packets, 2 credits back, in flight 0
      +0.0 ms  we send one packet - and stop
    +246.0 ms  Number of Completed Packets, 1 that one completes, in flight 0
                                             ... and still nothing
   +1021.3 ms  we send
   +1021.4 ms  we send, and the normal rhythm resumes at once

The budget was free for the whole second, so this was never flow control. And
the decisive measurement is on the other radio: **we wrote nothing to the pad
either** - 0 forwards of the console's output against the 91 the rate would put
there - while the pad kept arriving at 117% of its normal rate, 737 reports at a
1.26 ms median.

Both radios were delivering into the kernel at full speed and the relay wrote to
neither for a second. That is not a radio refusing to transmit. That is the
event loop not running.

### What stopped it, probably

Not established, but there is an obvious candidate and it is worth writing down
before someone blames the radio again.

`scripts/run.sh` sends the relay's stdout straight to a file:
`ps5padlog ... > "$APP_LOG" 2>&1`, in the same directory where two `hcidump`
processes are writing captures at about 10 MB a minute between them. Every
`std::cout` in the relay is therefore a blocking `write()` to the SD card, on the
io_context thread - the same thread that owns both HCI sockets. The filesystem
during this session was at **99% full with 636 MB left**, which is where ext4's
block allocator starts working hard and writes start taking a long time.

The relay already knows this class of bug from the read side: the loop drains
sockets with non-blocking `recv` precisely because a blocking read here stops
everything. The logging path never got the same treatment.

Two things were done about it. The disk was freed - 8.4 GB of captures older
than 10 Sep, which took the filesystem from 99% to 85% - and `run.sh` now sends
the relay's stdout through a pipe instead of opening the file itself, so the
write that can block happens in `cat`, in its own process, while the relay's
lands in a 64 KB kernel buffer it will never fill at a few hundred bytes a
minute. The relay is unchanged.

That swap has a trap in it worth knowing, because the obvious version of the
change is worse than the problem: on a pipe, a writer that dies takes the relay
with it via SIGPIPE. Measured both ways with a stand-in binary - without
`trap '' PIPE` the relay is killed, with it the write fails with EPIPE and the
session continues, losing a log line. The trap is set in a subshell that then
`exec`s the relay, which keeps both the ignored disposition and the PID, so
`$!` is still the relay and cleanup's `kill -INT` still reaches it.

One event proves none of this is the cause. What it does establish is where not
to look: the console-facing adapter had nothing to do with it.

It has not come back. In the 16:31 session - freed disk, logging through the
pipe - there are 175 stalls and **not one** of them has the signature: 174 sit
below 30% of the console's baseline traffic, one between 30 and 60%, none above,
and the longest is 567 ms. That is consistent with the explanation and proves
nothing on eleven minutes, but the alternative - a second-long hole appearing
once every 23 minutes - would have shown by now.

# The holes are still untested

This session had **no haptic audio**: 337 `0x39` packets, 0.4% of the console's
output against 2.3% before, and a single episode lasting 0.06 s. Stalls need
audio, so the one stall in fifteen minutes says nothing about whether the swap
helps them. It is what the old data predicts for a session with the audio idle.

The tools now say so rather than dividing by it - a stall rate computed against
0.06 s of audio is noise wearing a number's clothes.

### One new stall, of a kind that never happened before

At 1141 s into that session the relay sent nothing for **1021 ms**, and the
console kept talking throughout: 96 output reports arrived inside the hole
against 95 the baseline rate would put there. One completion event in the whole
second. That is the opposite of the 135 ms stalls, where the console's own
traffic drops to a fifth.

It is one event, so it proves nothing on its own. What makes it worth recording
is that the signature is new. Across all 3811 stalls of the 2.4 hour session on
the old arrangement:

| console traffic inside the stall | stalls |
| --- | --- |
| under 30% of baseline | 3746 (98.3%) |
| 30-60% | 65 (1.7%) |
| over 60% | **0** |

and of the two stalls longer than 500 ms, neither had traffic above 80%. So this
did not happen once in 3811 tries on the onboard radio, and happened within 23
minutes on the BT400. The link was demonstrably alive and our two packets sat in
the controller for a second anyway.

If it recurs at any rate worth caring about, the console side is the suspect and
the swap needs rethinking - and the options are narrow, because the pad wants the
onboard radio and the Realtek dongles are not documented to accept the address
change the console side needs. That would leave a straight trade: 4 ms on every
input report against a rare hole of about a second. Worth watching the next few
sessions for before deciding.

# The holes

The relay's own counters print every 5 seconds, which cannot see a 135 ms hole.
Every number below therefore comes from the console-side capture, which
`scripts/run.sh --capture` records - captures are off by default:

    python3 tools/analyse_stalls.py logs/capture-ps5-YYYYMMDD-HHMMSS.dump

A **stall** is a gap longer than 100 ms between two consecutive ACL packets we
wrote to the play link. In a 2.4 hour session there are 3811 of them, 0.442 per
second, covering 6.0% of the time.

## The stall is not 135 ms, it is half a second

The duration is sharply quantised - median 134.3 ms, 98% of them inside the
100-150 ms band, and the histogram is a bell about 4 ms wide rather than the
long tail that contention produces. That much was known.

What was missed is that they arrive in runs:

| | |
| --- | --- |
| runs of two or more stalls | 996, holding 85% of all stalls |
| stalls per run | median 3, max 6 |
| span of a run | median 417 ms, p90 696 ms, max 920 ms |
| share of a run spent stalled | median 97.6% |
| packets between back-to-back stalls | median 2 - and 87.6% of the time, exactly 2 |

So the event is not a 134 ms hole every 3.8 seconds. It is a blackout of a third
to nearly a full second, punctuated by two packets, roughly every nine seconds.
That is a much better match for what it feels like than the single-stall figure.

## The console does not go away

The earlier reading was that the console's radio leaves and the link is silent
in both directions. It is not. **Every stall, 3811 of 3811, has console traffic
inside it.** And that traffic is not spread out:

    position of a console packet within the stall (0 = start, 1 = end)
      0.00-0.05   1110  ###############
      0.45-0.55   3507  ################################
      0.95-1.00   3768  ####################################

Start, exact middle, end. A stall is two ticks of something with a period of
about 67 ms, not an absence.

## What is on the air

Splitting those packets by report id finds the cause immediately. `0x39` is the
console's haptic-audio output, 552 bytes against `0x36`'s 403:

| report | share of console traffic | share of packets the stall **ends** on |
| --- | --- | --- |
| `0x36` | 97.6% | 1% |
| `0x39` | **2.3%** | **99%** |

And the gap that contains each kind:

| | median gap spanning it | that gap exceeds 100 ms |
| --- | --- | --- |
| a `0x36` arrival | 4.6 ms | 0.1% of the time |
| a `0x39` arrival | **68.9 ms** | **43.9% of the time** |

`0x39`'s own inter-arrival time has a sharp mode at 65-70 ms, which is the 67 ms
period the phase histogram found.

Grouping `0x39` into episodes - a gap over 150 ms starts a new one - closes it:

    1493 episodes, 496 s = 5.8% of the session
    episode length: median 0.29 s, p90 0.65 s, max 0.92 s

    stall rate with audio     6.01/s
    stall rate without audio  0.10/s

**Sixty times the rate.** And the episode length is the burst span measured
above, to three figures: 0.29 / 0.65 / 0.92 against 0.417 / 0.696 / 0.920.
They are the same event seen from two sides.

## Why two packets

`AclFlowControl` allows `depth = 2` packets in flight. The capture shows what
that means on the air:

- Number of Completed Packets credits **exactly 2 packets, 925080 times out of
  925103**. Ones are 23 events in 2.4 hours.
- Exactly one completion arrives inside each stall - the one that ends it -
  against about 14 that the baseline rate would put there.
- That completion lands a median of **0.06 ms** after a packet from the console.

So the cycle is: write two, budget gone, wait; the console sends something, the
controller reports both buffers free, write two more. The accounting closes
exactly: 925103 completion events credited 1850183 packets against 1850184 we
wrote. When audio is running the console's
openings come every 67 ms instead of every 8.75 ms, and two packets is all that
goes into each one.

## What this does not establish

Air time alone does not explain it, and `0x36` is the proof. It carries 403
bytes at 85.5/s - about 34 kB/s, thirty times the audio stream's 1.1 kB/s - and
the gap spanning one has a median of 4.6 ms and exceeds 100 ms in 0.1% of cases.
The console pushes far more data at us without stalling anything, so `0x39` is
not consuming a resource. It is putting the console into a different mode.

**Why** it does that is not visible from this side, and nothing here says. The
evidence is a 60x rate difference and a matching burst structure - strong, and
still correlation. The one-minute test below is what turns it into cause.

Note also that these are not two kinds of packet the console chooses between per
report: `0x36` and `0x39` are both `0xa2` HID output on the interrupt channel,
399 and 548 bytes of payload, and they share a field layout - `0x39`'s body
begins with the block that sits 68 bytes into `0x36`. Reading them as two output
formats of the same stream, one carrying an audio block, fits the bytes better
than reading them as two streams.

There is also no switch to find, at least not in the output stream. Comparing
the first 48 body bytes of the 19225 `0x36` reports that arrive in the 200 ms
before an episode against the 231339 that arrive more than two seconds from any
episode turns up nothing that reads as an enable: one counter-like byte drifts,
and two others take *fewer* distinct values before an episode than during quiet,
which is the wrong shape for a gate. Whatever puts the console into this mode,
it does not announce it to the controller.

## Corrections to the earlier analysis

Three conclusions from the previous round do not survive this capture. They are
recorded because each was reasoned from real data, and the way each went wrong
is worth keeping.

1. **"The console stops talking too - the link goes silent in both directions."**
   It does not. The drop is to 19% of baseline, not to zero, and no stall is
   silent. The earlier round measured `0x39` landing "a median of 64 ms into the
   stall" and read it as the console's radio returning; 64 ms is the *middle*
   cluster in the phase histogram above, and a `0x39` also sits right at the
   start of the stall, before it.

2. **"`0x39` is the console flushing buffered haptics the moment its radio comes
   back."** It precedes the stall (55% of stalls have one within 5 ms before the
   first missed send), marks its midpoint, and ends it. It tracks the whole
   thing. Treating it as an after-effect is what kept the audio stream from
   being suspected.

3. **"More than two buys nothing: the console polls at about 125 Hz and takes
   roughly one packet per poll, so at two we are already at its rate."** Right
   answer, wrong reason - worth separating, because the reason is what a later
   reader would build on. Every completion returns the full budget of 2, so a
   measurement taken at a budget of 2 cannot tell the console's appetite from
   our own cap, and the "one packet per poll" claim has no support either way.
   More than two does buy nothing, but because the stall ends on the console's
   next opening and no queue of ours moves that.

The general conclusion the old document reached - that the 133 ms is the
console's scheduler and nothing host-side reaches it - was right about the
scheduler and wrong about there being no lever. There is one, and it is not on
this machine.

## What to try, in order

**1. Turn the controller's haptics and speaker down on the console.** One
setting, one minute, and the prediction is unambiguous: if haptic audio is the
trigger, the stall rate should fall from 0.442/s toward the 0.10/s measured
while audio is idle, and the bursts should disappear entirely. Every lever the
earlier round tried was on the Pi; this one is not, which is why it was never
reached. Nothing in the relay changes either way.

**2. Do not raise `depth`.** An earlier draft of this document proposed it as a
falsifiable experiment. The capture already answers it, and the answer is no.
The stall ends when the controller reports buffers free - 3810 of 3811 stalls
end on that event - so its length is the time until the console's next opening,
and nothing about our queue moves that. A deeper queue would put four reports
into the opening instead of two; the freshest one in that batch is the same
report either way, so the player feels nothing, and the three stale ones behind
it are exactly what the single held slot exists to avoid. The old document's
objection to a deeper queue was right.

**3. Fix what the stats line prints.** `acl 2/8` reads as "2 of 8 used, plenty
spare". The 8 is `max_packets_`, the controller's whole pool from Read Buffer
Size; the budget is `min(depth, max_packets_ - 1)` = 2. Half of all samples in a
session sit at the cap while the line suggests headroom, which has already
misled one reading of this data.

## Reproducing

`tools/analyse_stalls.py` needs only numpy and prints every figure above in the
order this document uses them; a 568 MB capture takes about 4 seconds. It finds
the play link by packet volume rather than by a hardcoded handle, so it works on
any session's console-side capture.

The relay's own log is too coarse for any of this, and worth saying why: the
`USB 5s` line has no timestamps and no gap counter, so a 135 ms hole is invisible
in it. Correlating the 5-second counters can show that a dip is bidirectional and
that the pad is unaffected - which is a useful first check - but not what causes
one.
