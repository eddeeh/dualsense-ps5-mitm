#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

// The controller side of the relay as the USB gadget (USBHandler) sees it, so
// the gadget does not depend on BluetoothHandler, which implements it. A second
// implementation reaching the pad over hidraw (UsbPadBackend, the pass-through
// experiment) was removed with the rest of the USB-only work.
//
// These are exactly the calls USBHandler makes back toward the pad; the pad
// backend calls USBHandler::send_input / on_feature_report the other way.
class IPadBackend {
public:
    virtual ~IPadBackend() = default;

    // An output report the console sent (rumble, lightbar, trigger effects,
    // haptic audio), in USB framing, to be delivered to the pad.
    virtual void l2cap_send_output_report(const uint8_t *usb_report, size_t size) = 0;

    // The console asked to read a feature report; fetch it from the pad. The
    // answer comes back asynchronously via USBHandler::on_feature_report.
    virtual void l2cap_get_report(uint8_t report_id) = 0;

    // A feature report the console wrote (0x0a pairing, 0x08 transport, 0x80
    // command, ...), to be delivered to the pad.
    virtual void l2cap_send_feature_set(const uint8_t *report, size_t size) = 0;

    // One-line status for the periodic USB stats print.
    virtual std::string relay_status() const = 0;
};
