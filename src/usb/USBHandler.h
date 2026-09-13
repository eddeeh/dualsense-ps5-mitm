#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <map>
#include <optional>
#include <span>
#include <vector>
#include <boost/asio.hpp>

#include "usb/IPadBackend.h"
#include "usb/LinuxAio.h"

// From <linux/usb/functionfs.h>, which stays in the .cpp: the kernel's USB
// headers would otherwise reach everything BluetoothHandler.h is included in.
struct usb_functionfs_event;
struct usb_ctrlrequest;

// The USB side: a configfs gadget with the three UAC1 audio interfaces a real
// DualSense has, and its HID interface served from userspace through
// FunctionFS. ep0 carries the console's control requests - descriptors, idle,
// GET_REPORT and SET_REPORT - and ep1/ep2 are the interrupt IN and OUT pair.
class USBHandler {
public:
    USBHandler(IPadBackend *pad, boost::asio::io_context *ios);
    ~USBHandler();

    // Creates the configfs gadget, mounts FunctionFS for its HID function and
    // writes the descriptors, which is what makes the function bindable.
    // Removes a leftover gadget from an earlier run first, so a second start
    // in the same boot is not fatal.
    bool init_gadget();
    bool enable_gadget();
    void disable_gadget();
    void remove_gadget();

    // Not a coroutine, and deliberately so. It hands one report to the kernel
    // and returns; there is nothing inside it to await. As an awaitable it
    // cost a coroutine frame, an executor copy, a handler wrapper and a trip
    // through the scheduler queue on every report the pad sent - which the
    // profile showed as most of the process's user time.
    void send_input(std::span<const uint8_t> report);

    // Whether there is anywhere for a report to go. The gadget detaches as soon
    // as the console hands over the pairing data, and after that every input
    // report the pad sends - five hundred a second - was still being reformatted
    // for a transport that is gone.
    bool transport_open() const { return bound_; }

    // The host has selected the configuration: it has seen the cable and
    // enumerated the controller, whether or not it has asked for input yet.
    bool endpoints_up() const { return endpoints_enabled_; }

    // When the first input report was taken by the console. The rule for the PS
    // press is anchored to exactly this moment - a press that lands
    // before it is never seen. Default-constructed until it happens.
    std::chrono::steady_clock::time_point input_flowing_since() const {
        return input_flowing_since_;
    }

    // ep0: FunctionFS's events (bind, enable, disable, ...) and the console's
    // control requests, in the order they arrive.
    boost::asio::awaitable<void> control_events();

    // The interrupt endpoints: completions of input reports the console took
    // and of output reports it sent - rumble, lightbar, trigger effects,
    // haptic audio - which go on to the pad.
    boost::asio::awaitable<void> endpoint_events();

    // Prints what actually crossed the USB endpoints. An input report the host
    // is not polling for never completes, which is indistinguishable from
    // silence without a counter.
    boost::asio::awaitable<void> report_stats();

    // Called by BluetoothHandler with a feature report the DualSense returned
    // (report ID first, CRC included), in answer to a GET_REPORT we forwarded.
    void on_feature_report(const uint8_t *report, size_t size);

private:
    // Same idiom as BluetoothHandler::sleep_for.
    boost::asio::awaitable<void> sleep_for(std::chrono::milliseconds d);

    bool open_function();
    void close_function();

    void handle_event(const usb_functionfs_event &event);
    void handle_setup(const usb_ctrlrequest &setup);
    void on_endpoints_enabled();
    void on_endpoints_disabled(const char *why);

    // ep0 data and status stages. FunctionFS completes each in the calling
    // thread, so each is bounded; see Ep0Deadline in the .cpp.
    bool ep0_reply(std::span<const uint8_t> data);
    void ep0_ack();
    void ep0_stall(uint8_t request_type);
    void ep0_stage_failed(const char *stage, int err, uint8_t direction);

    void begin_get_report(uint8_t report_id, uint16_t length);
    void finish_get_report(std::span<const uint8_t> report);
    void take_set_report(uint16_t value, uint16_t length);

    void submit_output_read();
    void on_input_done(int64_t result);
    void on_output_done(int64_t result);

    // USB traffic counters, printed periodically by report_stats().
    uint64_t in_ok_ = 0;        // input reports the host took
    uint64_t in_eagain_ = 0;    // dropped: previous write still in flight
    uint64_t in_shutdown_ = 0;  // dropped: endpoint down
    uint64_t in_error_ = 0;
    uint64_t out_count_ = 0;    // output reports from the console
    uint64_t ctrl_count_ = 0;   // GET_REPORT / SET_REPORT from the console
    std::chrono::steady_clock::time_point input_flowing_since_{};

    IPadBackend *pad_;
    boost::asio::io_context *ios_;
    bool gadget_created_ = false;
    bool mounted_ = false;

    // ep0 stays open for as long as the function is meant to exist: closing it
    // is what tells FunctionFS the function is gone, and it unbinds the gadget.
    int ep0_fd_ = -1;
    int ep_in_fd_ = -1;
    int ep_out_fd_ = -1;

    bool bound_ = false;              // between enable_gadget and disable_gadget
    bool endpoints_enabled_ = false;  // the host has selected the configuration

    // Readiness of ep0, and of the AIO eventfd. Dups, because Asio closes what
    // it is given and the originals carry the actual reads and writes. Held
    // here so disable_gadget() and the destructor can cancel a parked wait.
    std::optional<boost::asio::posix::stream_descriptor> ep0_ready_;
    std::optional<boost::asio::posix::stream_descriptor> aio_ready_;

    LinuxAio aio_;

    // One request in flight per endpoint. Buffers belong to the kernel from
    // submission until the completion is reaped. hidgadget::REPORT_LENGTH,
    // which the .cpp checks this against.
    static constexpr size_t REPORT_BUFFER = 64;
    std::array<uint8_t, REPORT_BUFFER> in_buf_{};
    std::array<uint8_t, REPORT_BUFFER> out_buf_{};
    bool in_busy_ = false;
    bool out_busy_ = false;

    // The GET_REPORT waiting on a Bluetooth round trip. While it is set ep0
    // belongs to it: FunctionFS stalls a pending IN request on any read of ep0.
    struct PendingGet {
        uint8_t report_id;
        uint16_t length;
    };
    std::optional<PendingGet> pending_get_;
    boost::asio::steady_timer get_report_timer_;

    // The latest answer to every feature report the pad gave, report ID first.
    // Fixed pad properties are served from here straight away; everything else
    // only when the pad does not answer in time, the way f_hid fell back to the
    // last report userspace had written.
    std::map<uint8_t, std::vector<uint8_t>> answers_;

    // SET_IDLE's duration, read back by GET_IDLE. 1 is f_hid's default.
    uint8_t idle_ = 1;
};
