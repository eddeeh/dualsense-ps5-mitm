#pragma once

//
// BluetoothHandlerInternal.h - ps5padlog
//
// The constants more than one of BluetoothHandler's source files uses. Private
// to src/bluetooth: a constant only one file needs is kept in that file.
//
// Still in an unnamed namespace, as they were in BluetoothHandler.cpp, so each
// file that includes this gets its own internal copy exactly as before.
//

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>

#include <chrono>
#include <cstdint>

namespace {

// Scan enable, as HCI_Write_Scan_Enable takes it.
constexpr uint8_t SCAN_NONE      = 0x00;
constexpr uint8_t SCAN_PAGE_ONLY = 0x02;

// Page scan activity, in 0.625 ms slots. The eager pair is a 180 ms window
// every 640 ms: the console's call-back from its play radio is a single page
// and missing it costs the whole run. Once that call-back has arrived the same
// duty cycle is a quarter of the radio spent listening for nothing, so we drop
// back to the usual 11.25 ms every 1.28 s - see quiet_the_radio().
constexpr uint16_t PAGE_SCAN_INTERVAL_EAGER  = 0x0400;  // 640 ms
constexpr uint16_t PAGE_SCAN_WINDOW_EAGER    = 0x0120;  // 180 ms
constexpr uint16_t PAGE_SCAN_INTERVAL_NORMAL = 0x0800;  // 1.28 s
constexpr uint16_t PAGE_SCAN_WINDOW_NORMAL   = 0x0012;  // 11.25 ms

// Our own paging has to leave gaps in that listening: the controller cannot
// answer a page while one of ours is outstanding.
constexpr uint16_t PAGE_TIMEOUT_SLOTS = 0x0800;  // 1.28 s, not the default 5.12 s

// The console-facing side's page while it calls the console straight after the
// handover. The console becomes reachable about 3.5 s after the pair command
// however early we start - 3.57, 3.63 and 3.47 s in run-20260911-142535, -142753
// and -143048, the last paging from the first millisecond - so what counts is
// being in the air at that moment rather than sitting in a pause. Only inside
// HANDOVER_PAGE_WINDOW: outside it the adapter has to stay reachable, which is
// what the short timeout above is for.
constexpr uint16_t HANDOVER_PAGE_TIMEOUT_SLOTS = 0x1000;  // 2.56 s

// How long after the handover a page timeout is answered with the next page at
// once instead of a pause. The pause was there to hand the console the
// initiative, but across the runs of 8-11 September it never once called us on
// its registration radio, and all 28 missed handover pages were PAGE TIMEOUT: a
// miss cost 3 s for nothing, 32 s in run-20260909-095040 and 128 s in
// run-20260908-215225. Past the window the old backoff applies again.
constexpr auto HANDOVER_PAGE_WINDOW = std::chrono::seconds(30);

// 0x13, "connection terminated by the remote user" - what we send when we
// close a link on purpose, and what the peer sends us when it does.
constexpr uint8_t DISCONNECT_REMOTE_USER = HCI_OE_USER_ENDED_CONNECTION;

}  // namespace
