#include "usb/LinuxAio.h"

#include <cerrno>
#include <cstring>
#include <ctime>

#include <linux/aio_abi.h>
#include <sys/eventfd.h>
#include <sys/syscall.h>
#include <unistd.h>

static_assert(sizeof(aio_context_t) == sizeof(unsigned long));

bool LinuxAio::open(unsigned depth)
{
    if (ctx_)
        return true;
    eventfd_ = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (eventfd_ < 0)
        return false;
    if (::syscall(SYS_io_setup, long(depth), &ctx_) < 0) {
        const int saved = errno;
        ::close(eventfd_);
        eventfd_ = -1;
        ctx_ = 0;
        errno = saved;
        return false;
    }
    return true;
}

void LinuxAio::close()
{
    if (ctx_) {
        ::syscall(SYS_io_destroy, ctx_);
        ctx_ = 0;
    }
    if (eventfd_ >= 0) {
        ::close(eventfd_);
        eventfd_ = -1;
    }
}

bool LinuxAio::submit_write(int fd, const uint8_t *buf, size_t len, uint64_t tag)
{
    return submit(fd, IOCB_CMD_PWRITE, reinterpret_cast<uint64_t>(buf), len, tag);
}

bool LinuxAio::submit_read(int fd, uint8_t *buf, size_t len, uint64_t tag)
{
    return submit(fd, IOCB_CMD_PREAD, reinterpret_cast<uint64_t>(buf), len, tag);
}

bool LinuxAio::submit(int fd, uint16_t op, uint64_t buf, size_t len, uint64_t tag)
{
    iocb cb;
    std::memset(&cb, 0, sizeof(cb));
    cb.aio_fildes = static_cast<uint32_t>(fd);
    cb.aio_lio_opcode = op;
    cb.aio_buf = buf;
    cb.aio_nbytes = len;
    cb.aio_offset = 0;
    cb.aio_flags = IOCB_FLAG_RESFD;
    cb.aio_resfd = static_cast<uint32_t>(eventfd_);
    cb.aio_data = tag;

    iocb *list[1] = {&cb};
    return ::syscall(SYS_io_submit, ctx_, 1L, list) == 1;
}

// The counter only says that something finished; io_getevents says what. It is
// emptied first, so a completion landing after this point raises it again and
// wakes the next wait rather than being missed.
void LinuxAio::drain_eventfd()
{
    uint64_t count;
    while (::read(eventfd_, &count, sizeof(count)) == sizeof(count)) {}
}

size_t LinuxAio::take_completions(Completion *out, size_t max)
{
    io_event events[8];
    if (max > sizeof(events) / sizeof(events[0]))
        max = sizeof(events) / sizeof(events[0]);

    timespec zero{};
    const long n = ::syscall(SYS_io_getevents, ctx_, 0L, long(max), events, &zero);
    if (n <= 0)
        return 0;
    for (long i = 0; i < n; i++)
        out[i] = {uint64_t(events[i].data), int64_t(events[i].res)};
    return size_t(n);
}
