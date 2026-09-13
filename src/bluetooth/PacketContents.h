//
// Created by ed on 10.08.2023.
//

#ifndef TEST2_PACKETREADER_H
#define TEST2_PACKETREADER_H

#include <cstdint>
#include <span>

// A cursor over one packet as it came off the HCI socket.
//
// Every accessor is bounds-checked, and that is the whole point of the class
// rather than a nicety. These bytes arrive from the controller and, through it,
// from the console and the pad; the lengths inside them - an L2CAP payload
// length above all - are attacker- and bug-controlled, not ours. The previous
// version called span::subspan(m_offset) with no check at all, which is
// undefined behaviour the moment the offset passes the end, and handed back a
// T* that the caller then read sizeof(T) bytes through.
//
// The condition is not hypothetical: L2CAP fragments are not reassembled here
// (see the note on sdp_client_channel's MTU), so a fragmented payload really
// does declare a length longer than the bytes that arrived.
//
// Short reads return nullptr and leave the cursor where it was. Callers are
// expected to check and drop the packet - there is nothing sensible to do with
// half a structure.
class PacketContents {
public:
    PacketContents(uint8_t *ptr, size_t size) : m_data(ptr, size) {}

    size_t remaining() const {
        return m_offset < m_data.size() ? m_data.size() - m_offset : 0;
    }

    // Structure at the cursor, or nullptr if the packet does not hold one.
    template<typename T>
    T *get() {
        if (remaining() < sizeof(T))
            return nullptr;
        T *result = reinterpret_cast<T *>(m_data.data() + m_offset);
        m_offset += sizeof(T);
        return result;
    }

    // Typed view of `size` bytes at the cursor, or nullptr if they are not all
    // there. Saves callers casting get()'s void* to uint8_t* by hand.
    const uint8_t *bytes(size_t size) {
        if (remaining() < size)
            return nullptr;
        const uint8_t *result = m_data.data() + m_offset;
        m_offset += size;
        return result;
    }

    void *get(size_t size) {
        if (remaining() < size)
            return nullptr;
        void *result = m_data.data() + m_offset;
        m_offset += size;
        return result;
    }

private:
    std::span<uint8_t> m_data;
    size_t m_offset = 0;
};
#endif //TEST2_PACKETREADER_H
