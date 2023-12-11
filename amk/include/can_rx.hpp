#pragma once

#include <unistd.h>
#include <string.h>
#include <cstring>
#include <stdexcept>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "can_common.hpp"

namespace PUTM_CAN
{
    static constexpr time_t NO_TIMEOUT = 0;

    template <typename T>
    inline T convert(can_frame frame);

    class CanRx
    {
    public:
        inline CanRx(const char *const ifname, const time_t rx_timeout_in_s);
        inline ~CanRx();
        inline CanRx(const CanRx &) = delete;
        inline CanRx &operator=(const CanRx &) = delete;
        inline can_frame receive() const;

    private:
        int file_descriptor;
    };
}

namespace PUTM_CAN
{
    inline CanRx::CanRx(const char *const interface_name, const time_t rx_timeout_in_seconds)
        : file_descriptor(INVALID_FILE_DESCRIPTOR)
    {
        ifreq ifr;
        sockaddr_can addr;
        file_descriptor = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (file_descriptor == INVALID_FILE_DESCRIPTOR)
        {
            throw std::runtime_error("socket() failed");
        }
        (void)std::strncpy(ifr.ifr_name, interface_name, sizeof(ifr.ifr_name));
        if (ioctl(file_descriptor, SIOCGIFINDEX, &ifr) == -1)
        {
            throw std::runtime_error("ioctl() failed");
        }
        timeval tv = {rx_timeout_in_seconds, 0};
        if (setsockopt(file_descriptor, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv)) != 0)
        {
            throw std::runtime_error("setsockopt() set timeout failed");
        }
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(file_descriptor, (sockaddr *)&addr, sizeof(addr)) == -1)
        {
            throw std::runtime_error("bind() failed");
        }
    }

    inline CanRx::~CanRx()
    {
        if (file_descriptor != INVALID_FILE_DESCRIPTOR)
        {
            close(file_descriptor);
        }
    }

    inline can_frame CanRx::receive() const
    {
        can_frame frame;
        if (read(file_descriptor, &frame, sizeof(frame)) < (ssize_t)sizeof(frame))
        {
            throw std::runtime_error("read() failed");
        }
        return frame;
    }

    template <typename T>
    inline T convert(can_frame frame)
    {
        T rx_frame;
        // if ((frame.can_id != can_id<T>) || (frame.can_dlc != sizeof(T)))
        // {
        //     throw std::runtime_error("Invalid conversion");
        // }
        (void)std::memcpy(&rx_frame, frame.data, sizeof(T));
        return rx_frame;
    }
}