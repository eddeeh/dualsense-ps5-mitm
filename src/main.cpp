//
// main.cpp — ps5padlog entry point
//
// Architecture: DualSense --BT--> RPi5 --BT--> PS5
//
// Two radios: one holds the pad, the other claims the pad's address and takes
// the console's play link. The USB gadget on the RPi5's USB-C port (which also
// powers it from the PS5) only registers the controller - the console reads its
// feature reports and writes its pairing data over the cable, then asks it to
// come over Bluetooth, and the gadget detaches.
//
// Usage:
//   First run:  put DualSense in pairing mode (hold PS + Create ~3s), then run
//   Later runs: press PS on the DualSense once; the relay makes the second
//               press itself, and the saved link key is reused
//
// Prerequisites:
//   sudo systemctl stop bluetooth
//   sudo rfkill unblock all
//

#include "bluetooth/BluetoothHandler.h"
#include "common/BdAddr.h"
#include "common/Switches.h"

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstdio>
#include <memory>
#include <print>
#include <bluetooth/bluetooth.h>

// ============================================================================
// Configuration — these are defaults, and the arguments override them:
//
//   ps5padlog [hci_index] [dualsense_mac] [ps5_hci_index]
//
// The dongles' adapter indices move around between boots, so scripts/run.sh
// detects them and passes them in rather than making you edit this file.
// ============================================================================

// DualSense BT MAC address
static const char* DUALSENSE_MAC = "88:03:4C:FE:79:42";

// HCI device index for the ASUS USB dongle (connects to DualSense)
// Check with: hciconfig — look for the USB adapter
static const int HCI_DUALSENSE = 1;

// File to persist link key between runs
static const char* LINK_KEY_FILE = "/etc/dualsense_mitm.key";

// The console's own address and the link key it wrote over USB. It hands the
// key out only the first time it meets a given controller address, so losing
// this file means the PS5-side link can never authenticate again.
static const char* CONSOLE_KEY_FILE = "/etc/dualsense_mitm_console.key";

// ============================================================================
// Link key persistence
// ============================================================================

template<typename T>
static bool read_pod(std::ifstream &f, T &out) {
    return static_cast<bool>(f.read(reinterpret_cast<char*>(&out), sizeof(T)));
}
template<typename T>
static void write_pod(std::ofstream &f, const T &v) {
    f.write(reinterpret_cast<const char*>(&v), sizeof(T));
}

struct SavedState {
    std::array<uint8_t, 16> link_key;
    bdaddr_t ds_addr;
};

static bool load_state(SavedState &state) {
    std::ifstream f(LINK_KEY_FILE, std::ios::binary);
    if (!f) return false;

    read_pod(f, state.link_key);
    read_pod(f, state.ds_addr);

    if (!f) return false;

    std::println("Loaded saved link key for {}", state.ds_addr);
    return true;
}

static void save_state(const std::array<uint8_t, 16> &key, const bdaddr_t &addr) {
    std::ofstream f(LINK_KEY_FILE, std::ios::binary | std::ios::trunc);
    if (!f) {
        std::cerr << "Failed to save link key to " << LINK_KEY_FILE << std::endl;
        return;
    }
    write_pod(f, key);
    write_pod(f, addr);

    std::println("Link key saved for {}", addr);
}

// The console issues one key per controller address. Storing it without
// recording which controller it belongs to meant that swapping pads made us
// authenticate with a stranger's key, which the console refuses.
static bool load_console_pairing(bdaddr_t &addr, std::array<uint8_t, 16> &key,
                                 const bdaddr_t &controller) {
    std::ifstream f(CONSOLE_KEY_FILE, std::ios::binary);
    if (!f) return false;

    bdaddr_t saved_controller{};
    read_pod(f, key);
    read_pod(f, addr);
    const bool has_controller = read_pod(f, saved_controller);

    if (!has_controller) {
        std::cout << "Console pairing file predates the controller field, ignoring it"
                  << std::endl;
        return false;
    }
    if (saved_controller != controller) {
        std::println("Console pairing is for controller {}, not this one - ignoring it",
                     saved_controller);
        return false;
    }

    std::println("Loaded console pairing for {}", addr);
    return true;
}

static void save_console_pairing(const bdaddr_t &addr, const std::array<uint8_t, 16> &key,
                                 const bdaddr_t &controller) {
    std::ofstream f(CONSOLE_KEY_FILE, std::ios::binary | std::ios::trunc);
    if (!f) {
        std::cerr << "Failed to save console pairing to " << CONSOLE_KEY_FILE << std::endl;
        return;
    }
    write_pod(f, key);
    write_pod(f, addr);
    write_pod(f, controller);

    std::println("Console pairing saved for {}", addr);
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char **argv) {
    setbuf(stdout, nullptr);

    int hci_index = HCI_DUALSENSE;
    const char *ds_mac = DUALSENSE_MAC;
    int ps5_hci_index = -1;
    if (argc > 1) hci_index = atoi(argv[1]);
    if (argc > 2) ds_mac = argv[2];
    if (argc > 3) ps5_hci_index = atoi(argv[3]);

    std::cout << "=== ps5padlog ===" << std::endl;
    std::cout << "Architecture: DualSense --BT--> RPi5 --BT--> PS5 (USB for registration)"
              << std::endl;
    std::cout << std::endl;

    // The DS_* switches are gone; one set out of habit would otherwise look as
    // if this run had used it. See Switches.h.
    for (const auto &entry : unknown_switches())
        std::println(stderr, "warning: {} is ignored - this build has no DS_* switches "
                             "(see src/common/Switches.h)", entry);

    boost::asio::io_context ios;

    bdaddr_t ds_addr{};
    std::optional<std::array<uint8_t, 16>> link_key;

    SavedState saved{};
    if (load_state(saved)) {
        ds_addr = saved.ds_addr;
        link_key = saved.link_key;
        std::cout << "Reconnect mode — press PS button on DualSense" << std::endl;
    } else {
        str2ba(ds_mac, &ds_addr);
        std::cout << "First run — put DualSense in PAIRING MODE (hold PS + Create ~3s)" << std::endl;
    }

    std::println("DualSense target: {}", ds_addr);
    std::cout << "Using hci" << hci_index << " for BT connection" << std::endl;
    std::cout << std::endl;

    BluetoothHandler handler(ios, hci_index,
                             BluetoothHandler::EConnectionSide::to_dualsense, link_key);
    if (!handler.ok())
        return 1;

    handler.bt_addr_ = ds_addr;

    // The console issues one pairing per controller, and both sides want it:
    // the PS5-facing side to accept the console's call, this side to know the
    // link key is already held. Read once - it used to be read twice, which
    // also logged the same line twice.
    bdaddr_t console_addr{};
    std::array<uint8_t, 16> console_key{};
    const bool have_console_pairing =
        load_console_pairing(console_addr, console_key, ds_addr);

    // Second adapter, the one that will face the PS5. Without it the console
    // is told the pad's own address and waits for a connection nothing can
    // make, which is what leaves it stuck after the pairing writes.
    std::unique_ptr<BluetoothHandler> ps5_side;
    if (ps5_hci_index >= 0 &&
        handler.set_ps5_side_adapter(static_cast<uint16_t>(ps5_hci_index))) {

        ps5_side = std::make_unique<BluetoothHandler>(
            ios, static_cast<uint32_t>(ps5_hci_index),
            BluetoothHandler::EConnectionSide::to_playstation);

        if (ps5_side->ok()) {
            // With a spoofable adapter the console-facing side claims the
            // pad's own address, so the console meets a controller it already
            // trusts rather than an unknown dongle. The report rewrite then
            // has nothing to change: what we report is what we are.
            ps5_side->set_spoof_addr(ds_addr);
            handler.ps5_side_addr_ = ds_addr;
            std::println("PS5 side will claim the pad's address {}", ds_addr);

            // The console's address has to be known before it or anything else
            // calls in: that side accepts connections from the console only.
            if (have_console_pairing)
                ps5_side->set_console_pairing(console_addr, console_key);

            ps5_side->set_on_console_pairing([&ds_addr](const bdaddr_t &a,
        const std::array<uint8_t, 16> &k) { save_console_pairing(a, k, ds_addr); });

            handler.set_other(ps5_side.get());
            ps5_side->set_other(&handler);
            boost::asio::co_spawn(ios, ps5_side->handle(), boost::asio::detached);
        } else {
            std::cerr << "PS5-side adapter could not be claimed - continuing without it"
                      << std::endl;
            ps5_side.reset();
        }
    } else {
        std::cout << "No PS5-side adapter given (argument 3) - "
                     "the console will not be able to reach us over Bluetooth" << std::endl;
    }

    // Always armed: a key can also be renegotiated mid-run, after the pad has
    // rejected the saved one and we fall back to pairing.
    handler.set_on_link_key_saved([&ds_addr](const std::array<uint8_t, 16> &key) {
        save_state(key, ds_addr);
    });
    if (have_console_pairing)
        handler.set_console_pairing(console_addr, console_key);
    handler.set_on_console_pairing([&ds_addr](const bdaddr_t &a,
        const std::array<uint8_t, 16> &k) { save_console_pairing(a, k, ds_addr); });

    handler.set_on_link_key_rejected([]() {
        if (::remove(LINK_KEY_FILE) == 0)
            std::cout << "Removed stale " << LINK_KEY_FILE << std::endl;
    });

    // Ctrl+C has to close the links, not just kill the process: a peer left
    // holding a half-dead connection ignores the next run until its own
    // supervision timeout runs out.
    // SIGHUP too: closing the terminal killed us outright, and because a
    // HCI_CHANNEL_USER session owns the controller, the kernel does not reset
    // it on close - the links stay live in the radio's firmware. The next run
    // then binds and inherits a link it never opened, with the console still
    // streaming into it and believing the controller is connected.
    boost::asio::signal_set signals(ios, SIGINT, SIGTERM, SIGHUP);
    signals.async_wait([&](const boost::system::error_code &, int) {
        std::cout << "\nShutting down" << std::endl;
        handler.disconnect_link();
        if (ps5_side)
            ps5_side->disconnect_link();

        auto grace = std::make_shared<boost::asio::steady_timer>(
            ios, std::chrono::milliseconds(300));
        grace->async_wait([&ios, grace](const boost::system::error_code &) {
            ios.stop();
        });
    });

    boost::asio::co_spawn(ios, handler.handle(), boost::asio::detached);
    ios.run();

    return 0;
}