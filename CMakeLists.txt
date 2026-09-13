cmake_minimum_required(VERSION 3.25)
project(ps5padlog)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optimised unless asked otherwise. With no build type CMake adds no -O at all,
# and that is how the relay ended up running at -O0 - with every CPU figure of
# the day taken on it - once an editor reconfigured the build directory.
# scripts/run.sh builds its own build-release; this covers a bare `cmake -B`.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Debug, Release, RelWithDebInfo or MinSizeRel" FORCE)
endif()
# -O2 rather than CMake's -O3 for Release: the relay is a few hundred small
# events a second, not a loop -O3's vectoriser helps, and -O2 is what every
# profile in PERFORMANCE.md is meant to be compared against. NDEBUG stays.
string(REPLACE "-O3" "-O2" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")

include(FindPkgConfig)

if (LIBBLUETOOTH_INCLUDE_DIR AND LIBBLUETOOTH_LIBRARIES)

    # in cache already
    set(LIBBLUETOOTH_FOUND TRUE)

else (LIBBLUETOOTH_INCLUDE_DIR AND LIBBLUETOOTH_LIBRARIES)
    IF (NOT WIN32)
        # use pkg-config to get the directories and then use these values
        # in the FIND_PATH() and FIND_LIBRARY() calls
        find_package(PkgConfig)
        pkg_check_modules(PC_LIBBLUETOOTH bluez)
    ENDIF (NOT WIN32)

    FIND_PATH(LIBBLUETOOTH_INCLUDE_DIR hci.h
            PATHS ${PC_LIBBLUETOOTH_INCLUDEDIR} ${PC_LIBBLUETOOTH_INCLUDE_DIRS}
            PATHS /usr/include /usr/local/include /usr/include/bluetooth
            /usr/local/include/bluetooth)

    FIND_LIBRARY(LIBBLUETOOTH_LIBRARIES NAMES bluetooth
            PATHS ${PC_LIBBLUETOOTH_LIBDIR} ${PC_LIBBLUETOOTH_LIBRARY_DIRS})

    include(FindPackageHandleStandardArgs)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(LIBBLUETOOTH DEFAULT_MSG LIBBLUETOOTH_LIBRARIES LIBBLUETOOTH_INCLUDE_DIR)

    MARK_AS_ADVANCED(LIBBLUETOOTH_INCLUDE_DIR LIBBLUETOOTH_LIBRARIES)

endif (LIBBLUETOOTH_INCLUDE_DIR AND LIBBLUETOOTH_LIBRARIES)

#include(FetchContent)
#set(FETCHCONTENT_QUIET FALSE)
#FetchContent_Declare(
#        Boost
#        URL https://boostorg.jfrog.io/artifactory/main/release/1.83.0/source/boost_1_83_0.tar.gz
#        URL_HASH SHA256=c0685b68dd44cc46574cce86c4e17c0f611b15e195be9848dfd0769a0a207628
#        GIT_TAG "tag"
#        GIT_PROGRESS TRUE
#)
#FetchContent_MakeAvailable(Boost)
#FetchContent_GetProperties(Boost SOURCE_DIR Boost_INCLUDE_DIR)
#find_package(Boost 1.83 REQUIRED)
find_package(Boost 1.83 REQUIRED)
find_package(ZLIB REQUIRED)

add_compile_definitions(BOOST_ASIO_HAS_CO_AWAIT)

add_executable(ps5padlog src/main.cpp
        src/common/BdAddr.h
        src/common/Switches.h
        src/dualsense/DualSense.h
        src/dualsense/Transforms.h
        src/bluetooth/BluetoothHandler.cpp
        src/bluetooth/HciCommands.cpp
        src/bluetooth/HciEvents.cpp
        src/bluetooth/L2cap.cpp
        src/bluetooth/Sdp.cpp
        src/bluetooth/HidRelay.cpp
        src/bluetooth/Handover.cpp
        src/bluetooth/BluetoothHandler.h
        src/bluetooth/BluetoothHandlerInternal.h
        src/bluetooth/AclFlowControl.h
        src/bluetooth/HeldFrame.h
        src/bluetooth/SendGaps.h
        src/bluetooth/HciCommandQueue.h
        src/bluetooth/PacketContents.h
        src/usb/USBHandler.cpp
        src/usb/USBHandler.h
        src/usb/HidGadget.h
        src/usb/LinuxAio.cpp
        src/usb/LinuxAio.h
        src/usb/IPadBackend.h
        src/input/InputFeed.h)
target_include_directories(ps5padlog PRIVATE src)

# Watches and logs the pad's input from the relay's /dev/shm feed. Built with the
# relay so scripts/run.sh has it; needs nothing but the header.
add_executable(ps5padlog-view src/input/InputView.cpp src/input/InputFeed.h)
target_include_directories(ps5padlog-view PRIVATE src)

# ============================================================================
# Tests
#
# The pure functions only - byte transforms, checksums, the packet cursor, the
# ACL credit accounting and the gadget's FunctionFS descriptors. No sockets, no
# UDC, no radio, so `make tests &&
# ./tests` runs anywhere the project builds. Fixtures under tests/fixtures.h are
# real reports lifted out of a capture by tools/extract_test_fixtures.py.
#
# Built with the sanitizers on, because the class of bug these are aimed at -
# reading past the end of a packet - is one ASan reports and assertions do not.
# ============================================================================
add_executable(tests EXCLUDE_FROM_ALL
        tests/main.cpp
        tests/test_packet_contents.cpp
        tests/test_transforms.cpp
        tests/test_dualsense.cpp
        tests/test_acl_flow_control.cpp
        tests/test_held_frame.cpp
        tests/test_send_gaps.cpp
        tests/test_hci_command_queue.cpp
        tests/test_switches.cpp
        tests/test_input_feed.cpp
        tests/test_hid_gadget.cpp)
target_include_directories(tests PRIVATE src)
target_link_libraries(tests ${ZLIB_LIBRARIES})
target_compile_options(tests PRIVATE -fsanitize=address,undefined -g -Wall -Wextra)
target_link_options(tests PRIVATE -fsanitize=address,undefined)

# Walks a whole hcidump capture through the real parsing code. Not part of
# `tests`: it needs a capture, and those are hundreds of megabytes and are not
# in the repository. See tests/README.md.
add_executable(replay_capture EXCLUDE_FROM_ALL tests/replay_capture.cpp)
target_include_directories(replay_capture PRIVATE src)
target_link_libraries(replay_capture ${ZLIB_LIBRARIES} bluetooth)
target_compile_options(replay_capture PRIVATE -O2 -Wall -Wextra)

target_include_directories(${PROJECT_NAME} PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${GLIB_LIBRARIES} bluetooth ${Boost_LIBRARIES} usbgx ${ZLIB_LIBRARIES})