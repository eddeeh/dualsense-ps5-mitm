#include <cstdio>
#include "usb/USBHandler.h"
#include "dualsense/DualSense.h"
#include "usb/HidGadget.h"

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <ctime>
#include <endian.h>
#include <fcntl.h>
#include <filesystem>
#include <thread>
#include <fstream>
#include <iostream>
#include <memory>
#include <sys/mount.h>
#include <unistd.h>

#include <usbg/usbg.h>

using boost::asio::use_awaitable;
static constexpr auto use_nothrow_awaitable =
    boost::asio::as_tuple(boost::asio::use_awaitable);

namespace {

// The FunctionFS instance. configfs calls the function ffs.<instance>, and the
// instance name is what gets mounted.
constexpr const char *FFS_INSTANCE = "dualsense";
constexpr const char *FFS_FUNCTION = "ffs.dualsense";
constexpr const char *FFS_MOUNT = "/dev/ffs-dualsense";

// What each AIO completion belongs to.
constexpr uint64_t TAG_INPUT = 1;
constexpr uint64_t TAG_OUTPUT = 2;

// f_hid's GET_REPORT_TIMEOUT_MS, for its reason: a console gives up on an
// unanswered control transfer long before the 5 s most hosts allow, and a
// Bluetooth round trip to the pad fits in a second with room to spare.
constexpr std::chrono::milliseconds GET_REPORT_TIMEOUT{1000};

// How long one ep0 data or status stage may hold the thread; see Ep0Deadline.
constexpr std::chrono::milliseconds EP0_STAGE_DEADLINE{200};

// The signal Ep0Deadline interrupts a stage with. Not a constant: glibc
// reserves the first real-time signals for itself and SIGRTMIN is a call.
int ep0_deadline_signal() { return SIGRTMIN + 1; }

}  // namespace

// ============================================================================
// Construction
// ============================================================================

USBHandler::USBHandler(IPadBackend *pad, boost::asio::io_context *ios)
    : pad_(pad), ios_(ios), get_report_timer_(*ios)
{
    // The header keeps the kernel's USB headers out, so it cannot name this.
    static_assert(REPORT_BUFFER == hidgadget::REPORT_LENGTH);
}

USBHandler::~USBHandler()
{
    remove_gadget();
}

// ============================================================================
// Composite gadget: three UAC1 audio interfaces plus HID
//
// A real DualSense is not a bare HID device. lsusb on the hardware shows four
// interfaces: 0 = Audio Control, 1 = Audio Streaming (speaker, 4ch/48k/16-bit
// OUT), 2 = Audio Streaming (microphone, 2ch/48k/16-bit IN) and 3 = HID with
// its interrupt pair on 0x84/0x03. libusbgx has no UAC1 binding - only UAC2,
// and the controller reports bcdADC 1.00 - so the audio function is built
// straight through configfs. HID is a FunctionFS function answered from this
// process.
// ============================================================================

static const char *GADGET_PATH = "/sys/kernel/config/usb_gadget/g0";
static const std::string UDC_PATH = std::string(GADGET_PATH) + "/UDC";

// configfs unbinds a gadget when an empty *line* is written to UDC. Streaming a
// zero-length string writes nothing at all and leaves the gadget attached,
// which is silent and looks exactly like success - and then every removal below
// fails with EBUSY, and the next run meets USBG_ERROR_EXIST with no explanation.
// teardown_gadget_tree() had that bug; disable_gadget() had the fix and the
// comment. One function now, so there is one place to be right.
static bool unbind_udc(const char *who)
{
    std::ofstream ofs(UDC_PATH);
    if (!ofs) {
        std::cerr << who << ": cannot open " << UDC_PATH << std::endl;
        return false;
    }
    ofs << "\n";
    ofs.close();

    std::ifstream check(UDC_PATH);
    std::string bound;
    std::getline(check, bound);
    if (!bound.empty()) {
        std::cerr << who << ": gadget still bound to " << bound << std::endl;
        return false;
    }
    return true;
}

// configfs attributes vary between kernel versions; writing one that is not
// there is not an error worth aborting for, so this reports rather than fails.
static bool write_gadget_attr(const std::string &path, const std::string &value)
{
    if (!std::filesystem::exists(path))
        return false;
    std::ofstream ofs(path);
    if (!ofs)
        return false;
    ofs << value;
    ofs.close();
    return !ofs.fail();
}

// Removes the gadget without going through libusbgx, which cannot parse a
// function type it has no binding for and would fail on the whole tree. Used
// both for a leftover from an earlier run and for our own at exit: libusbgx's
// recursive removal stops at the config, which still holds the uac1 link it
// does not know about.
static void teardown_gadget_tree()
{
    std::error_code ec;
    if (!std::filesystem::exists(GADGET_PATH))
        return;

    // Must succeed before anything below: configfs refuses to remove the
    // pieces of a gadget that is still bound to a UDC.
    unbind_udc("teardown");

    // A FunctionFS instance cannot go while it is mounted. Nothing is left
    // holding it by now - its ep0 closes with the process that opened it - so
    // a lazy unmount is only there not to fail on a shell sitting inside.
    umount2(FFS_MOUNT, MNT_DETACH);

    const std::string configs = std::string(GADGET_PATH) + "/configs";
    for (const auto &cfg : std::filesystem::directory_iterator(configs, ec)) {
        if (!cfg.is_directory()) continue;
        for (const auto &e : std::filesystem::directory_iterator(cfg.path(), ec)) {
            if (std::filesystem::is_symlink(e.symlink_status()))
                std::filesystem::remove(e.path(), ec);
        }
        const auto strings = cfg.path() / "strings";
        for (const auto &sd : std::filesystem::directory_iterator(strings, ec))
            std::filesystem::remove(sd.path(), ec);
        std::filesystem::remove(cfg.path(), ec);
    }

    const std::string functions = std::string(GADGET_PATH) + "/functions";
    for (const auto &fn : std::filesystem::directory_iterator(functions, ec))
        std::filesystem::remove(fn.path(), ec);

    const std::string strings = std::string(GADGET_PATH) + "/strings";
    for (const auto &sd : std::filesystem::directory_iterator(strings, ec))
        std::filesystem::remove(sd.path(), ec);

    std::filesystem::remove(GADGET_PATH, ec);
}

// Creates uac1.usb0 and links it into the config. Must run before the HID
// function is linked: interface numbers are handed out in link order.
static bool add_audio_function()
{
    // The config directory is named by libusbgx; find it rather than assume.
    std::error_code find_ec;
    std::string config_dir;
    for (const auto &e : std::filesystem::directory_iterator(
             std::string(GADGET_PATH) + "/configs", find_ec)) {
        if (e.is_directory()) { config_dir = e.path().string(); break; }
    }
    if (config_dir.empty()) {
        std::cerr << "uac1: no config directory under " << GADGET_PATH << "/configs"
                  << std::endl;
        return false;
    }

    const std::string fn = std::string(GADGET_PATH) + "/functions/uac1.usb0";

    std::error_code ec;
    std::filesystem::create_directory(fn, ec);
    if (!std::filesystem::exists(fn)) {
        std::cerr << "uac1: cannot create " << fn
                  << " - is usb_f_uac1 present? (modprobe usb_f_uac1)" << std::endl;
        return false;
    }

    // Matching the hardware: speaker is 4 channels (front L/R, surround L/R,
    // wChannelConfig 0x0033) and the microphone is stereo, both 48 kHz 16-bit.
    // "c_" is the gadget's capture side, which is the OUT endpoint the console
    // plays into; "p_" is playback, the IN endpoint carrying the microphone.
    const std::pair<const char *, const char *> attrs[] = {
        // Capture is what the console plays *into* the pad: four channels, as
        // the hardware reports (0x33 is channels 1, 2, 5 and 6). Playback is the
        // microphone, two channels. Both 48 kHz, 16-bit, which is what a real
        // DualSense declares.
        {"c_chmask", "0x33"}, {"c_srate", "48000"}, {"c_ssize", "2"},
        {"p_chmask", "0x3"},  {"p_srate", "48000"}, {"p_ssize", "2"},
    };
    for (const auto &[attr, val] : attrs) {
        if (!write_gadget_attr(fn + "/" + attr, val))
            std::cerr << "uac1: attribute " << attr << " not settable, using the default"
                      << std::endl;
    }

    std::filesystem::create_directory_symlink(fn, config_dir + "/uac1.usb0", ec);
    if (ec) {
        std::cerr << "uac1: cannot link into the config: " << ec.message() << std::endl;
        return false;
    }

    std::cout << "Audio interfaces added (uac1: interfaces 0-2, HID becomes 3)" << std::endl;
    return true;
}

namespace {

// What the console reads out of the device descriptor to decide what we are.
// Both appear again in the log line at the end of init_gadget(), which is the
// only reason they are named rather than written twice.
constexpr uint16_t SONY_VENDOR_ID     = 0x054c;
constexpr uint16_t DUALSENSE_PRODUCT_ID = 0x0ce6;

// Every libusbgx call reports the same way, and repeating the check five times
// buried the sequence in error handling.
bool usbg_ok(int ret, const char *what)
{
    if (ret == USBG_SUCCESS)
        return true;
    std::cerr << what << " failed: " << usbg_strerror((usbg_error)ret) << std::endl;
    return false;
}

// What the console reads out of the device descriptor. The strings matter as
// much as the ids: the accessory list names the controller from iProduct, and
// ours was showing up a word short of what the hardware reports.
usbg_gadget_attrs device_attrs()
{
    return {
        .bcdUSB = 0x0200,
        .bDeviceClass = 0x00,
        .bDeviceSubClass = 0x00,
        .bDeviceProtocol = 0x00,
        .bMaxPacketSize0 = 64,
        .idVendor = SONY_VENDOR_ID,
        .idProduct = DUALSENSE_PRODUCT_ID,
        .bcdDevice = 0x0100,
    };
}

usbg_gadget_strs device_strings()
{
    return {
        .manufacturer = (char *)"Sony Interactive Entertainment",
        .product = (char *)"DualSense Wireless Controller",
        .serial = (char *)"",
    };
}

}  // namespace

bool USBHandler::init_gadget()
{
    usbg_gadget_attrs g_attrs = device_attrs();
    usbg_gadget_strs g_strs = device_strings();

    usbg_config_attrs c_attrs = {
        .bmAttributes = 0xc0, // Self-powered
        .bMaxPower = 250,     // 500 mA
    };
    usbg_config_strs c_strs = {
        .configuration = (char *)"config",
    };

    // A leftover means the last run did not get to tear its gadget down. The
    // pause after removing it gives the console a disconnect it can notice
    // rather than an unbind and a rebind inside the same millisecond. It was
    // once credited with fixing registration hangs; it was not the cause of
    // them - see reset_controller() in HciCommands.cpp - but it is cheap, runs
    // only after a hard kill, and the console seeing a clean disconnect does
    // no harm.
    const bool leftover = std::filesystem::exists(GADGET_PATH);
    if (leftover)
        std::cout << "Removing leftover gadget g0 from a previous run - the last one "
                     "was killed rather than stopped" << std::endl;
    teardown_gadget_tree();
    if (leftover) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "  -> waited for the console to see the disconnect. If it still "
                     "never polls, power the console down fully; rest mode will not do"
                  << std::endl;
    }

    usbg_state *raw_state = nullptr;
    if (!usbg_ok(usbg_init("/sys/kernel/config", &raw_state), "usbg_init")) {
        std::cerr << "  -> is configfs mounted and libcomposite loaded?" << std::endl;
        return false;
    }
    // libusbgx only builds the tree; nothing after this function uses it, and
    // removal goes through teardown_gadget_tree(). It used to be a global
    // freed by whichever USBHandler was destroyed first - the PS5 side's,
    // which never created a gadget, and which left the other a dangling one.
    const std::unique_ptr<usbg_state, decltype(&usbg_cleanup)> state(raw_state, &usbg_cleanup);

    usbg_gadget *g = nullptr;
    if (!usbg_ok(usbg_create_gadget(state.get(), "g0", &g_attrs, &g_strs, &g),
                 "usbg_create_gadget"))
        return false;
    gadget_created_ = true;

    // The HID interface itself - descriptors, report descriptor, endpoints -
    // is not configured here at all: FunctionFS takes it from what
    // open_function() writes to ep0.
    usbg_function *f = nullptr;
    if (!usbg_ok(usbg_create_function(g, USBG_F_FFS, FFS_INSTANCE, nullptr, &f),
                 "usbg_create_function")) {
        std::cerr << "  -> is usb_f_fs available? (modprobe usb_f_fs)" << std::endl;
        return false;
    }

    usbg_config *c = nullptr;
    if (!usbg_ok(usbg_create_config(g, 1, "config", &c_attrs, &c_strs, &c),
                 "usbg_create_config"))
        return false;

    // Audio first: interface numbers follow the order functions are linked, and
    // HID has to end up as interface 3, the way a real pad presents it.
    add_audio_function();

    if (!usbg_ok(usbg_add_config_function(c, FFS_FUNCTION, f), "usbg_add_config_function"))
        return false;

    if (!open_function())
        return false;

    printf("USB gadget created (VID=%04x PID=%04x)\n",
           SONY_VENDOR_ID, DUALSENSE_PRODUCT_ID);
    return true;
}

// ============================================================================
// FunctionFS
//
// Mounting the instance gives an ep0 file. The descriptors and strings written
// to it are the interface; once both are accepted the function is ready to
// bind and the endpoint files appear, ep1 for interrupt IN and ep2 for OUT.
// From then on ep0 reads as a stream of events - bind, enable, disable, and
// every control request aimed at the interface, which userspace answers.
// ============================================================================

bool USBHandler::open_function()
{
    std::error_code ec;
    std::filesystem::create_directories(FFS_MOUNT, ec);
    if (mount(FFS_INSTANCE, FFS_MOUNT, "functionfs", 0, nullptr) != 0) {
        std::cerr << "FunctionFS: cannot mount " << FFS_INSTANCE << " on " << FFS_MOUNT
                  << ": " << strerror(errno) << std::endl;
        return false;
    }
    mounted_ = true;

    const std::string dir = FFS_MOUNT;
    ep0_fd_ = open((dir + "/ep0").c_str(), O_RDWR | O_CLOEXEC);
    if (ep0_fd_ < 0) {
        std::cerr << "FunctionFS: cannot open ep0: " << strerror(errno) << std::endl;
        return false;
    }

    // A descriptor set FunctionFS does not like is refused here, with EINVAL
    // and nothing more. tests/test_hid_gadget.cpp walks the same bytes so that
    // is not the first place a mistake shows up.
    const auto descriptors = hidgadget::ffs_descriptors();
    if (write(ep0_fd_, descriptors.data(), descriptors.size())
        != static_cast<ssize_t>(descriptors.size())) {
        std::cerr << "FunctionFS: descriptors refused: " << strerror(errno) << std::endl;
        return false;
    }
    const auto strings = hidgadget::ffs_strings();
    if (write(ep0_fd_, strings.data(), strings.size())
        != static_cast<ssize_t>(strings.size())) {
        std::cerr << "FunctionFS: strings refused: " << strerror(errno) << std::endl;
        return false;
    }

    // Non-blocking so that an ep0 read with no event waiting returns rather
    // than parking the io_context. The endpoints need it more: without it a
    // request made before the host has selected the configuration waits for
    // that, inside io_submit.
    fcntl(ep0_fd_, F_SETFL, fcntl(ep0_fd_, F_GETFL) | O_NONBLOCK);
    ep_in_fd_ = open((dir + "/ep1").c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    ep_out_fd_ = open((dir + "/ep2").c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (ep_in_fd_ < 0 || ep_out_fd_ < 0) {
        std::cerr << "FunctionFS: cannot open the endpoint files: " << strerror(errno)
                  << std::endl;
        return false;
    }

    // One request per endpoint is ever in flight; the rest is headroom.
    if (!aio_.open(8)) {
        std::cerr << "FunctionFS: io_setup failed: " << strerror(errno)
                  << " - is CONFIG_AIO enabled?" << std::endl;
        return false;
    }

    // Ep0Deadline's signal. Its default action would end the process, and the
    // handler has nothing to do: interrupting the sleep is the whole point.
    struct sigaction sa = {};
    sa.sa_handler = [](int) {};
    sigemptyset(&sa.sa_mask);
    sigaction(ep0_deadline_signal(), &sa, nullptr);

    ep0_ready_.emplace(*ios_, ::dup(ep0_fd_));
    aio_ready_.emplace(*ios_, ::dup(aio_.eventfd()));

    std::cout << "FunctionFS: HID function ready at " << FFS_MOUNT << std::endl;
    return true;
}

void USBHandler::close_function()
{
    pending_get_.reset();
    get_report_timer_.cancel();
    ep0_ready_.reset();
    aio_ready_.reset();

    // Cancels and waits for whatever is still in flight, so the buffers it was
    // given are released before anything else goes.
    aio_.close();
    in_busy_ = out_busy_ = false;
    endpoints_enabled_ = false;

    // Endpoint files before ep0: FunctionFS tears the function down when the
    // last of them closes, and ep0 is the one that says so.
    for (int *fd : {&ep_in_fd_, &ep_out_fd_, &ep0_fd_}) {
        if (*fd >= 0) {
            close(*fd);
            *fd = -1;
        }
    }

    if (mounted_) {
        umount2(FFS_MOUNT, MNT_DETACH);
        mounted_ = false;
    }
}

bool USBHandler::enable_gadget()
{
    if (ep0_fd_ < 0) {
        std::cerr << "USB gadget was never set up, nothing to enable" << std::endl;
        return false;
    }

    std::string udc_name;
    if (std::filesystem::exists("/sys/class/udc/1000480000.usb")) {
        udc_name = "1000480000.usb";
    } else {
        for (const auto &entry : std::filesystem::directory_iterator("/sys/class/udc/")) {
            udc_name = entry.path().filename();
            break;
        }
    }
    if (udc_name.empty()) {
        std::cerr << "No UDC found in /sys/class/udc/" << std::endl;
        return false;
    }

    // The bind is where a function that is not ready fails, so its result is
    // worth reading.
    std::ofstream ofs(UDC_PATH);
    if (ofs)
        ofs << udc_name;
    ofs.close();
    if (!ofs) {
        std::cerr << "Cannot bind the gadget to " << udc_name << ": " << strerror(errno)
                  << std::endl;
        return false;
    }
    std::cout << "Gadget enabled on UDC: " << udc_name << std::endl;

    bound_ = true;
    return true;
}

void USBHandler::disable_gadget()
{
    if (unbind_udc("disable_gadget"))
        std::cout << "USB gadget detached from the UDC" << std::endl;

    bound_ = false;
    endpoints_enabled_ = false;

    // Nobody left to answer. The request itself is cancelled by the unbind,
    // and control_events() goes back to reading ep0.
    if (pending_get_) {
        pending_get_.reset();
        get_report_timer_.cancel();
    }
}

void USBHandler::remove_gadget()
{
    close_function();
    if (gadget_created_) {
        teardown_gadget_tree();
        gadget_created_ = false;
    }
    bound_ = false;
}

// ============================================================================
// Input reports: DualSense -> PS5 (interrupt IN)
// ============================================================================

void USBHandler::send_input(std::span<const uint8_t> report)
{
    if (!endpoints_enabled_) {
        in_shutdown_++;
        return;
    }
    // The same policy f_hid applied to a non-blocking write: while the host has
    // not taken the previous report, the new one is dropped, not queued. A
    // queue would only fill with reports that are stale by the time the host
    // polls again.
    if (in_busy_) {
        in_eagain_++;
        return;
    }

    const size_t len = std::min(report.size(), in_buf_.size());
    std::copy_n(report.begin(), len, in_buf_.begin());
    if (!aio_.submit_write(ep_in_fd_, in_buf_.data(), len, TAG_INPUT)) {
        in_error_++;
        static int errors = 0;
        if (errors++ < 3)
            std::cerr << "USB send_input: io_submit failed: " << strerror(errno) << std::endl;
        return;
    }
    in_busy_ = true;
}

void USBHandler::on_input_done(int64_t result)
{
    in_busy_ = false;

    if (result >= 0) {
        in_ok_++;
        if (input_flowing_since_ == std::chrono::steady_clock::time_point{}) {
            std::cout << "USB input reports now flowing to PS5!" << std::endl;
            // The relay presses PS itself from a second after this.
            input_flowing_since_ = std::chrono::steady_clock::now();
        }
        return;
    }

    const int err = static_cast<int>(-result);
    // ESHUTDOWN is the endpoint disabled under the request, ECONNRESET the
    // request cancelled, EAGAIN a submit that raced the disable.
    if (err == ESHUTDOWN || err == ECONNRESET || err == EAGAIN || err == ENODEV) {
        in_shutdown_++;
    } else {
        in_error_++;
        static int errors = 0;
        if (errors++ < 3)
            std::cerr << "USB input report failed: " << strerror(err) << std::endl;
    }
}

// ============================================================================
// Output reports: PS5 -> DualSense (interrupt OUT)
// ============================================================================

// One read is kept queued on the OUT endpoint whenever the host has it
// enabled. REPORT_LENGTH is the endpoint's packet size, so each completion is
// exactly one transfer, which for an interrupt endpoint is exactly one report.
void USBHandler::submit_output_read()
{
    if (out_busy_ || !endpoints_enabled_ || ep_out_fd_ < 0)
        return;
    if (!aio_.submit_read(ep_out_fd_, out_buf_.data(), out_buf_.size(), TAG_OUTPUT)) {
        std::cerr << "USB output endpoint: io_submit failed: " << strerror(errno) << std::endl;
        return;
    }
    out_busy_ = true;
}

void USBHandler::on_output_done(int64_t result)
{
    out_busy_ = false;

    if (result > 0) {
        out_count_++;
        if (pad_)
            pad_->l2cap_send_output_report(out_buf_.data(), static_cast<size_t>(result));
    } else if (result < 0) {
        // Not requeued: the endpoint is down, and an error that repeats would
        // otherwise spin here. The next enable starts reading again.
        const int err = static_cast<int>(-result);
        if (err != ESHUTDOWN && err != ECONNRESET && err != EAGAIN && err != ENODEV) {
            std::cerr << "USB output endpoint read failed: " << strerror(err)
                      << " - not reading it again until the host re-enables it" << std::endl;
        }
        return;
    }

    submit_output_read();
}

boost::asio::awaitable<void> USBHandler::sleep_for(std::chrono::milliseconds d)
{
    boost::asio::steady_timer t(*ios_, d);
    co_await t.async_wait(use_awaitable);
}

boost::asio::awaitable<void> USBHandler::endpoint_events()
{
    for (;;) {
        if (!aio_ready_)
            co_return;

        auto [ec] = co_await aio_ready_->async_wait(
            boost::asio::posix::stream_descriptor::wait_read, use_nothrow_awaitable);
        if (ec == boost::asio::error::operation_aborted)
            co_return;          // the function is being closed
        if (ec) {
            std::cerr << "USB endpoint events: " << ec.message() << std::endl;
            co_await sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        aio_.reap([this](uint64_t tag, int64_t result) {
            if (tag == TAG_INPUT)
                on_input_done(result);
            else if (tag == TAG_OUTPUT)
                on_output_done(result);
        });
    }
}

// ============================================================================
// Control endpoint: GET_REPORT / SET_REPORT relayed over Bluetooth
// ============================================================================

boost::asio::awaitable<void> USBHandler::report_stats()
{
    uint64_t last_ok = 0, last_eagain = 0, last_out = 0, last_ctrl = 0;

    for (;;) {
        co_await sleep_for(std::chrono::seconds(5));

        const uint64_t d_ok = in_ok_ - last_ok;
        const uint64_t d_eagain = in_eagain_ - last_eagain;
        const uint64_t d_out = out_count_ - last_out;
        const uint64_t d_ctrl = ctrl_count_ - last_ctrl;
        last_ok = in_ok_; last_eagain = in_eagain_;
        last_out = out_count_; last_ctrl = ctrl_count_;

        printf("USB 5s: in ok=%llu eagain=%llu ctrl=%llu | %s\n",
               (unsigned long long)d_ok, (unsigned long long)d_eagain,
               (unsigned long long)d_ctrl,
               pad_ ? pad_->relay_status().c_str() : "no bluetooth");
        (void)d_out;
    }
}

boost::asio::awaitable<void> USBHandler::control_events()
{
    for (;;) {
        if (ep0_fd_ < 0 || !ep0_ready_)
            co_return;

        // A GET_REPORT waiting on the pad owns ep0 until it is answered. A read
        // now would stall it, and a readiness wait would return at once:
        // FunctionFS reports ep0 readable for as long as a request is pending.
        // So the wait is on the answer, or on the timeout.
        if (pending_get_) {
            auto [ec] = co_await get_report_timer_.async_wait(use_nothrow_awaitable);
            if (!ec && pending_get_) {
                const uint8_t report_id = pending_get_->report_id;
                const auto last = answers_.find(report_id);
                printf("GET_REPORT 0x%02x -> no answer from the DualSense in %lld ms, "
                       "sending %s\n", report_id,
                       static_cast<long long>(GET_REPORT_TIMEOUT.count()),
                       last != answers_.end() ? "its last one" : "zeros");
                // What f_hid did when userspace missed its window.
                finish_get_report(last != answers_.end()
                                      ? std::span<const uint8_t>(last->second)
                                      : std::span<const uint8_t>());
            }
            continue;
        }

        usb_functionfs_event events[4];
        const ssize_t n = read(ep0_fd_, events, sizeof(events));
        if (n > 0) {
            for (size_t i = 0; i < static_cast<size_t>(n) / sizeof(events[0]); i++)
                handle_event(events[i]);
            continue;
        }

        const int err = n < 0 ? errno : EAGAIN;
        if (err == EAGAIN) {
            auto [ec] = co_await ep0_ready_->async_wait(
                boost::asio::posix::stream_descriptor::wait_read, use_nothrow_awaitable);
            if (ec == boost::asio::error::operation_aborted)
                co_return;      // the function is being closed
            if (ec) {
                std::cerr << "FunctionFS ep0: " << ec.message() << std::endl;
                co_await sleep_for(std::chrono::milliseconds(10));
            }
            continue;
        }
        // A request the host abandoned for a newer one, which is the next read.
        if (err == EIDRM)
            continue;
        // The function is no longer active - the instance was unmounted or its
        // files closed - and nothing will come from ep0 again.
        if (err == EBADFD) {
            std::cerr << "FunctionFS ep0 closed under us" << std::endl;
            co_return;
        }
        std::cerr << "FunctionFS ep0 read: " << strerror(err) << std::endl;
        co_await sleep_for(std::chrono::milliseconds(100));
    }
}

void USBHandler::handle_event(const usb_functionfs_event &event)
{
    switch (event.type) {
    case FUNCTIONFS_BIND:
        std::cout << "FunctionFS: bound" << std::endl;
        break;
    case FUNCTIONFS_UNBIND:
        on_endpoints_disabled("unbound");
        break;
    case FUNCTIONFS_ENABLE:
        on_endpoints_enabled();
        break;
    case FUNCTIONFS_DISABLE:
        on_endpoints_disabled("the host reset or deconfigured the device");
        break;
    case FUNCTIONFS_SETUP:
        handle_setup(event.u.setup);
        break;
    case FUNCTIONFS_SUSPEND:
    case FUNCTIONFS_RESUME:
        break;
    }
}

void USBHandler::on_endpoints_enabled()
{
    if (endpoints_enabled_)
        return;
    endpoints_enabled_ = true;
    std::cout << "USB configured by the host, HID endpoints up" << std::endl;
    submit_output_read();
}

void USBHandler::on_endpoints_disabled(const char *why)
{
    // Requests still in flight complete on their own, with ESHUTDOWN.
    if (!endpoints_enabled_)
        return;
    endpoints_enabled_ = false;
    std::cout << "USB HID endpoints down: " << why
              << " - waiting for the PS5 to configure it again" << std::endl;
}

void USBHandler::handle_setup(const usb_ctrlrequest &setup)
{
    using hidgadget::Request;
    const uint16_t value = le16toh(setup.wValue);
    const uint16_t length = le16toh(setup.wLength);

    // Descriptors and single bytes go back whole; FunctionFS trims a data
    // stage to wLength itself.
    switch (hidgadget::classify(setup.bRequestType, setup.bRequest, value)) {
    case Request::get_report:
        begin_get_report(static_cast<uint8_t>(value & 0xff), length);
        return;
    case Request::set_report:
        take_set_report(value, length);
        return;
    case Request::get_report_descriptor:
        ep0_reply(hidgadget::REPORT_DESCRIPTOR);
        return;
    case Request::get_hid_descriptor:
        ep0_reply(hidgadget::HID_DESCRIPTOR);
        return;
    case Request::get_idle:
        ep0_reply(std::span<const uint8_t>(&idle_, 1));
        return;
    case Request::get_protocol: {
        constexpr uint8_t report_protocol = 1;
        ep0_reply(std::span<const uint8_t>(&report_protocol, 1));
        return;
    }
    case Request::set_idle:
        idle_ = static_cast<uint8_t>(value >> 8);
        ep0_ack();
        return;
    case Request::set_protocol:
        // Only a boot-subclass interface may change protocol, and this is not
        // one; f_hid stalled it too.
    case Request::unsupported:
        break;
    }
    ep0_stall(setup.bRequestType);
}

// ============================================================================
// Bounded ep0 stages
//
// FunctionFS queues a data or status stage on ep0 and sleeps in the caller
// until the UDC completes it, O_NONBLOCK or not. The host is normally waiting
// for exactly that stage and takes it within a millisecond. But a console that
// has given up on a request never collects it, and the DWC2 drops a queued ep0
// request only on a bus reset - so this thread, which is also the Bluetooth
// relay's thread, could sleep there for as long as the console likes. The sleep
// is interruptible and FunctionFS dequeues the request when interrupted, so a
// one-shot timer signal aimed at this thread puts a bound on it.
// ============================================================================

namespace {

class Ep0Deadline {
public:
    explicit Ep0Deadline(std::chrono::milliseconds d)
    {
        sigevent sev = {};
        sev.sigev_notify = SIGEV_THREAD_ID;
        sev.sigev_signo = ep0_deadline_signal();
        sev._sigev_un._tid = gettid();
        if (timer_create(CLOCK_MONOTONIC, &sev, &timer_) != 0)
            return;
        created_ = true;

        itimerspec its = {};
        its.it_value.tv_sec = d.count() / 1000;
        its.it_value.tv_nsec = (d.count() % 1000) * 1000000L;
        timer_settime(timer_, 0, &its, nullptr);
    }
    ~Ep0Deadline()
    {
        if (created_)
            timer_delete(timer_);
    }
    Ep0Deadline(const Ep0Deadline &) = delete;
    Ep0Deadline &operator=(const Ep0Deadline &) = delete;

private:
    timer_t timer_{};
    bool created_ = false;
};

}  // namespace

// A stage that did not complete. EIDRM needs nothing more: FunctionFS has
// already let the request go for the one that replaced it. An interrupted
// stage is dequeued but the request is still pending, and the next read of ep0
// would take it for that request's data stage - so it is stalled here, which
// is also the honest answer to a request nobody collected.
void USBHandler::ep0_stage_failed(const char *stage, int err, uint8_t direction)
{
    const char *why = err == EIDRM ? "the host abandoned the request"
                    : err == EINTR ? "the host did not collect it in time"
                    : strerror(err);
    std::cerr << "FunctionFS ep0 " << stage << " failed: " << why << std::endl;
    if (err == EINTR)
        ep0_stall(direction);
}

// errno is taken inside the deadline's scope: timer_delete runs on the way out
// of it and may overwrite errno.
bool USBHandler::ep0_reply(std::span<const uint8_t> data)
{
    ssize_t n;
    int err = 0;
    {
        Ep0Deadline deadline(EP0_STAGE_DEADLINE);
        n = write(ep0_fd_, data.data(), data.size());
        err = errno;
    }
    if (n < 0) {
        ep0_stage_failed("data stage", err, USB_DIR_IN);
        return false;
    }
    return true;
}

// The status stage of a request with no data: an empty read, because the
// request's direction is OUT.
void USBHandler::ep0_ack()
{
    ssize_t n;
    int err = 0;
    {
        Ep0Deadline deadline(EP0_STAGE_DEADLINE);
        n = read(ep0_fd_, nullptr, 0);
        err = errno;
    }
    if (n < 0)
        ep0_stage_failed("status stage", err, USB_DIR_OUT);
}

// FunctionFS stalls a pending request when ep0 is used against its direction:
// read for one that expects data from us, write for one that brings some.
void USBHandler::ep0_stall(uint8_t request_type)
{
    uint8_t unused = 0;
    const ssize_t n = (request_type & USB_DIR_IN) ? read(ep0_fd_, &unused, 0)
                                                  : write(ep0_fd_, &unused, 0);
    (void)n;   // EL2HLT is the stall having happened
}

void USBHandler::begin_get_report(uint8_t report_id, uint16_t length)
{
    ctrl_count_++;

    if (dualsense::is_cacheable_feature(report_id)) {
        if (const auto cached = answers_.find(report_id); cached != answers_.end()) {
            printf("GET_REPORT 0x%02x -> served from cache (%zu bytes)\n",
                   report_id, cached->second.size());
            ep0_reply(hidgadget::get_report_answer(cached->second, length).bytes());
            return;
        }
    }

    printf("GET_REPORT 0x%02x -> forwarding to DualSense\n", report_id);
    pending_get_ = PendingGet{report_id, length};
    get_report_timer_.expires_after(GET_REPORT_TIMEOUT);
    if (pad_)
        pad_->l2cap_get_report(report_id);
}

void USBHandler::finish_get_report(std::span<const uint8_t> report)
{
    const PendingGet get = *pending_get_;
    pending_get_.reset();
    get_report_timer_.cancel();
    ep0_reply(hidgadget::get_report_answer(report, get.length).bytes());
}

void USBHandler::take_set_report(uint16_t value, uint16_t length)
{
    ctrl_count_++;

    // The data stage has to be taken whatever happens to it next, or the
    // console's transfer never completes.
    std::vector<uint8_t> data(length);
    ssize_t n;
    int err = 0;
    {
        Ep0Deadline deadline(EP0_STAGE_DEADLINE);
        n = read(ep0_fd_, data.data(), data.size());
        err = errno;
    }
    if (n < 0) {
        ep0_stage_failed("SET_REPORT data stage", err, USB_DIR_OUT);
        return;
    }

    const uint8_t report_id = static_cast<uint8_t>(value & 0xff);
    const uint8_t report_type = static_cast<uint8_t>(value >> 8);
    const size_t len = std::min(static_cast<size_t>(n), hidgadget::REPORT_LENGTH);
    if (len == 0 || !pad_)
        return;

    printf("SET_REPORT 0x%02x type=%u len=%zu -> DualSense\n", report_id, report_type, len);
    if (report_type == hidgadget::REPORT_TYPE_OUTPUT)
        pad_->l2cap_send_output_report(data.data(), len);
    else
        pad_->l2cap_send_feature_set(data.data(), len);
}

void USBHandler::on_feature_report(const uint8_t *report, size_t size)
{
    if (size < 1)
        return;

    const uint8_t report_id = report[0];

    auto &slot = answers_[report_id];
    slot.assign(report, report + std::min(size, hidgadget::REPORT_LENGTH));
    if (dualsense::is_cacheable_feature(report_id))
        std::cout << "Cached feature report 0x" << std::hex << (int)report_id
                  << std::dec << " (" << slot.size() << " bytes)" << std::endl;

    // data[0] is the report ID; HID requires it as the first byte of the
    // GET_REPORT response, so the whole report goes across verbatim.
    if (pending_get_ && pending_get_->report_id == report_id)
        finish_get_report(slot);
}
