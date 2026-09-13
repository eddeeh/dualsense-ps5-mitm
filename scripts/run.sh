#!/bin/bash
#
# One command to build, prepare the adapters and run the relay.
#
#   sudo ./scripts/run.sh                   play: optimised build, nothing extra
#   sudo ./scripts/run.sh --fresh-pairing   pair the pad from scratch
#   sudo ./scripts/run.sh --capture         also record both radio links
#   sudo ./scripts/run.sh --diag            the panel's link and radio numbers
#   sudo ./scripts/run.sh --debug           build without optimisation, with -g
#
# The terminal shows the pad's input, redrawn in place; the relay's own log goes
# to logs/run-*.log and every press and release to logs/inputs-*.log. Ctrl+C
# stops everything together.
#
set -u

usage() {
    cat <<'USAGE'
usage: sudo ./scripts/run.sh [--fresh-pairing] [--capture] [--diag] [--debug]

  --fresh-pairing  forget the pad's link key and pair from scratch
                   (DS_FRESH_PAIRING=1 still does the same)
  --capture        record both radio links with hcidump into logs/, for
                   tools/analyse_stalls.py and tools/analyse_input_path.py.
                   About 10 MB a minute between them - off unless asked for.
  --diag           show the rates, gaps, effects loss and radio on the panel
  --debug          build in build-debug with -O0 -g instead of build-release
USAGE
}

CAPTURE_LINKS=0
DIAG=0
DEBUG_BUILD=0
FRESH_PAIRING="${DS_FRESH_PAIRING:-0}"
for arg in "$@"; do
    case "$arg" in
        --capture)       CAPTURE_LINKS=1 ;;
        --diag)          DIAG=1 ;;
        --debug)         DEBUG_BUILD=1 ;;
        --fresh-pairing) FRESH_PAIRING=1 ;;
        -h|--help)       usage; exit 0 ;;
        *)               echo "unknown option: $arg" >&2; usage >&2; exit 2 ;;
    esac
done

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOGDIR="$ROOT/logs"
STAMP="$(date +%Y%m%d-%H%M%S)"
APP_LOG="$LOGDIR/run-$STAMP.log"
CAPTURE="$LOGDIR/capture-$STAMP.dump"
CAPTURE_PS5="$LOGDIR/capture-ps5-$STAMP.dump"

if [ "$(id -u)" -ne 0 ]; then
    echo "must run as root: sudo $0" >&2
    exit 1
fi

mkdir -p "$LOGDIR"

# ---------------------------------------------------------------------------
# 1. Build (as the invoking user, so the tree does not fill up with
#    root-owned objects that a later unprivileged build cannot overwrite)
# ---------------------------------------------------------------------------
# Its own build directory, never build/. That one belongs to whoever opens the
# tree in an editor: VS Code's CMake Tools reconfigured it as a Debug build, and
# because this script only ever ran `cmake --build build`, the relay quietly ran
# unoptimised - no -O at all - with every measurement taken on it. A directory
# this script configures itself cannot be changed underneath it.
BUILD_USER="${SUDO_USER:-}"
as_user() {
    if [ -n "$BUILD_USER" ] && command -v runuser >/dev/null; then
        runuser -u "$BUILD_USER" -- "$@"
    else
        "$@"
    fi
}
if [ "$DEBUG_BUILD" = "1" ]; then
    BUILD_DIR="$ROOT/build-debug"; BUILD_TYPE=Debug
else
    BUILD_DIR="$ROOT/build-release"; BUILD_TYPE=Release
fi

echo "== building ($BUILD_TYPE, $BUILD_DIR) =="
[ -n "$BUILD_USER" ] || echo "  (no SUDO_USER, building as root)"
if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    as_user cmake -S "$ROOT" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" -Wno-dev >/dev/null || exit 1
fi
as_user cmake --build "$BUILD_DIR" --target dualsense-ps5-mitm padlog -j "$(nproc)" || exit 1

# ---------------------------------------------------------------------------
# 2. Free the Bluetooth adapters, and load the stock gadget functions: the
#    HID interface is FunctionFS served by the relay, the audio is uac1
# ---------------------------------------------------------------------------
echo "== preparing bluetooth =="
systemctl stop bluetooth 2>/dev/null
systemctl mask bluetooth 2>/dev/null >/dev/null
rfkill unblock all
modprobe libcomposite
for m in usb_f_fs usb_f_uac1; do
    modprobe "$m" || { echo "gadget function $m is not available in $(uname -r)" >&2; exit 1; }
done

# ---------------------------------------------------------------------------
# 3. Find the USB dongle. The onboard RPi5 radio sits on a UART, the dongle
#    that talks to the DualSense is the one on the USB bus. Its index moves
#    between boots, which is why this is detected instead of hardcoded.
# ---------------------------------------------------------------------------
# Adapter indices are handed out in whatever order the kernel enumerated them
# and they swap between boots. Both bonds are tied to a BD address - the pad
# remembers which adapter it paired with, and the console registers the
# controller by the address we report to it - so a swap silently invalidates
# both. The assignment is pinned by address instead, in a file written on the
# first run.
ADAPTERS_FILE="/etc/dualsense_mitm.adapters"

addr_of() { hciconfig "$1" 2>/dev/null | awk '/BD Address/ { print $3; exit }'; }
dev_with_addr() {
    for d in $(hciconfig 2>/dev/null | awk '/^hci[0-9]+:/ { sub(":","",$1); print $1 }'); do
        [ "$(addr_of "$d")" = "$1" ] && { echo "$d"; return 0; }
    done
    return 1
}

USB_DEVS="$(hciconfig -a 2>/dev/null |
    awk '/^hci[0-9]+:/ { sub(":","",$1); dev=$1 } /Bus: USB/ { print dev }')"

if [ ! -f "$ADAPTERS_FILE" ]; then
    PAD_ADDR="$(addr_of "$(echo "$USB_DEVS" | sed -n 1p)")"
    PS5_ADDR="$(addr_of "$(echo "$USB_DEVS" | sed -n 2p)")"
    if [ -z "$PAD_ADDR" ]; then
        echo "No USB Bluetooth adapter found. Adapters present:" >&2
        hciconfig -a >&2
        exit 1
    fi
    printf 'PAD=%s\nPS5=%s\n' "$PAD_ADDR" "$PS5_ADDR" > "$ADAPTERS_FILE"
    echo "Pinned adapters in $ADAPTERS_FILE (delete it to re-assign):"
    cat "$ADAPTERS_FILE"
fi

PAD_ADDR="$(awk -F= '/^PAD=/ { print $2 }' "$ADAPTERS_FILE")"
PS5_ADDR="$(awk -F= '/^PS5=/ { print $2 }' "$ADAPTERS_FILE")"

HCI_DEV="$(dev_with_addr "$PAD_ADDR")"
if [ -z "$HCI_DEV" ]; then
    echo "Pad adapter $PAD_ADDR is not present. Adapters:" >&2
    hciconfig >&2
    exit 1
fi
HCI_INDEX="${HCI_DEV#hci}"
echo "Using $HCI_DEV ($PAD_ADDR) for the DualSense link"

PS5_DEV="$(dev_with_addr "$PS5_ADDR")"
if [ -n "$PS5_DEV" ]; then
    PS5_INDEX="${PS5_DEV#hci}"
    echo "Using $PS5_DEV ($PS5_ADDR) for the PS5 link"
else
    PS5_INDEX=""
    echo "PS5 adapter $PS5_ADDR is not present - the console side will be unavailable" >&2
fi

# HCI_CHANNEL_USER needs the adapters down and nothing else holding them.
for d in $(hciconfig 2>/dev/null | awk '/^hci[0-9]+:/ { sub(":","",$1); print $1 }'); do
    hciconfig "$d" down 2>/dev/null
done

# ---------------------------------------------------------------------------
# 4. Capture, then run
# ---------------------------------------------------------------------------
cleanup() {
    # The relay runs in the background. It closes its links on SIGINT,
    # which is what keeps the next run from inheriting them, so it gets that
    # rather than a kill, and time to finish before the captures stop.
    if [ -n "${RELAY_PID:-}" ] && kill -0 "$RELAY_PID" 2>/dev/null; then
        kill -INT "$RELAY_PID" 2>/dev/null
        wait "$RELAY_PID" 2>/dev/null
    fi
    for pid in "${DUMP_PID:-}" "${DUMP_PS5_PID:-}"; do
        [ -n "$pid" ] || continue
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null
            wait "$pid" 2>/dev/null
        fi
    done
    echo
    echo "log:         $APP_LOG"
    if [ "$CAPTURE_LINKS" = "1" ]; then
        echo "capture pad: $CAPTURE"
        [ -n "${PS5_INDEX:-}" ] && echo "capture ps5: $CAPTURE_PS5"
    fi
    [ -n "${INPUT_LOG:-}" ] && echo "inputs:      $INPUT_LOG"
}
# HUP too, which is what closing the terminal sends: without it cleanup never
# ran and the relay was killed mid-run. (This comment used to credit HUP with
# fixing registration hangs; the real cause turned out to be controllers that
# were never reset - see reset_controller() in HciCommands.cpp.)
trap cleanup EXIT INT TERM HUP

# Captures only on request. They are how every finding in PERFORMANCE.md was
# made, and they are two processes writing about 10 MB a minute to the SD card
# for as long as the relay runs - which filled the card to 99% once, and put the
# relay's own log writes behind them in the same queue.
if [ "$CAPTURE_LINKS" = "1" ]; then
    echo "== capturing pad link to $CAPTURE =="
    hcidump -i "$HCI_DEV" -l 4096 -w "$CAPTURE" &
    DUMP_PID=$!

    # The console-facing link is where the interesting failures happen, and it
    # is a different adapter, so it needs its own capture.
    if [ -n "$PS5_INDEX" ]; then
        echo "== capturing PS5 link to $CAPTURE_PS5 =="
        hcidump -i "$PS5_DEV" -l 4096 -w "$CAPTURE_PS5" &
        DUMP_PS5_PID=$!
    fi
    sleep 1
fi

# The app now notices a link key the pad has stopped honouring, drops it and
# asks for pairing mode on its own, so reusing the saved key is the default.
# --fresh-pairing (or DS_FRESH_PAIRING=1) forces a pairing from scratch.
if [ "$FRESH_PAIRING" = "1" ]; then
    rm -f /etc/dualsense_mitm.key
    echo "== put the DualSense in pairing mode: hold PS + Create for ~3s =="
else
    echo "== press the PS button on the DualSense =="
fi

# The relay's log goes to its file only, and the terminal shows the pad's input
# instead, redrawn in place, with every press and release written to
# inputs-*.log. `tail -f` the run log from another terminal to watch the relay.
INPUT_LOG="$LOGDIR/inputs-$STAMP.log"
PAD_MAC="88:03:4C:FE:79:42"
echo "== running, relay log in $APP_LOG =="

# Through a pipe rather than straight into the file, and the reason is measured.
# The relay logs from the same thread that owns both HCI sockets, and a flushed
# line - every std::endl, every cerr - was a write() to the SD card on it. With
# the card busy under two hcidump processes and the filesystem near full, one of
# those writes took about a second, and the relay wrote to neither radio for
# that second: a 1021 ms hole in the input stream, traced packet by packet in
# PERFORMANCE.md. A pipe gives the same write a 64 KB kernel buffer to land in -
# the relay logs a few hundred bytes a minute, so it never fills - and `cat`
# does the part that can block, in its own process.
#
# SIGPIPE ignored, so that a writer which dies costs a log line instead of the
# session; the relay's write fails with EPIPE and it carries on. The disposition
# survives the exec, and the exec keeps the PID, so $! is still the relay and
# cleanup's kill -INT reaches it.
: > "$APP_LOG"
( trap '' PIPE
  exec "$BUILD_DIR/dualsense-ps5-mitm" "$HCI_INDEX" "$PAD_MAC" ${PS5_INDEX:+"$PS5_INDEX"} \
) > >(exec cat >> "$APP_LOG") 2>&1 &
RELAY_PID=$!
VIEW_ARGS=(--log "$INPUT_LOG" --relay-log "$APP_LOG" --relay-pid "$RELAY_PID")
[ "$DIAG" = "1" ] && VIEW_ARGS+=(--diag)
"$BUILD_DIR/padlog" "${VIEW_ARGS[@]}"
