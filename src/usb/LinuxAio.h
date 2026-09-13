#pragma once

// Kernel AIO (io_setup / io_submit / io_getevents), which is how FunctionFS
// endpoints are meant to be driven without a thread.
//
// A FunctionFS endpoint file has no poll, and its synchronous read and write
// ignore O_NONBLOCK: a write to the interrupt IN endpoint sleeps until the host
// polls it. On the io_context thread that freezes the relay - the earlier
// FunctionFS experiment found exactly that and moved the writes to a thread.
// Submitted through AIO the same write returns at once and its completion is
// counted on an eventfd, which Asio can wait on like any other descriptor.
//
// Raw syscalls rather than libaio: four of them, and libaio's headers are not
// installed here. CONFIG_AIO is on in the kernel this runs on. The kernel's
// aio_abi.h stays in LinuxAio.cpp, because it drags <linux/fs.h> along into
// everything that includes this.

#include <cstddef>
#include <cstdint>

class LinuxAio {
public:
    // One finished request: the tag it was submitted with, and the byte count
    // or a negative errno.
    struct Completion {
        uint64_t tag;
        int64_t result;
    };

    LinuxAio() = default;
    LinuxAio(const LinuxAio &) = delete;
    LinuxAio &operator=(const LinuxAio &) = delete;
    ~LinuxAio() { close(); }

    // Room for `depth` requests in flight at once. False with errno set.
    bool open(unsigned depth);

    // Cancels what is still in flight and waits for it; the kernel does both
    // inside io_destroy. Buffers handed to a submit must outlive this.
    void close();

    bool is_open() const { return ctx_ != 0; }

    // Readable whenever a completion is waiting to be reaped.
    int eventfd() const { return eventfd_; }

    // Queues one write or read of `buf`, which must stay valid until its
    // completion is reaped. `tag` comes back with it. A request the file
    // refuses outright - an endpoint not yet enabled answers EAGAIN - still
    // completes, with the error as its result, so every outcome arrives the
    // same way. False only if the submission itself failed, with errno set.
    bool submit_write(int fd, const uint8_t *buf, size_t len, uint64_t tag);
    bool submit_read(int fd, uint8_t *buf, size_t len, uint64_t tag);

    // Hands every waiting completion to fn(tag, result). Never blocks.
    template<typename Fn>
    void reap(Fn &&fn)
    {
        drain_eventfd();
        Completion done[8];
        for (;;) {
            const size_t n = take_completions(done, sizeof(done) / sizeof(done[0]));
            if (n == 0)
                return;
            for (size_t i = 0; i < n; i++)
                fn(done[i].tag, done[i].result);
        }
    }

private:
    void drain_eventfd();
    size_t take_completions(Completion *out, size_t max);
    bool submit(int fd, uint16_t op, uint64_t buf, size_t len, uint64_t tag);

    unsigned long ctx_ = 0;     // aio_context_t
    int eventfd_ = -1;
};
