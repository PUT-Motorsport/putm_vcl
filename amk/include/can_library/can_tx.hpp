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
    class CanTx
    {
    public:
        inline CanTx(const char *const ifname);
        inline ~CanTx();

        inline CanTx(const CanTx &) = delete;
        inline CanTx &operator=(const CanTx &) = delete;

        template <typename T>
        inline void transmit(const T &tx_frame) const;

        template <typename T>
        inline void transmit_rtr() const;

    private:
        int file_descriptor;
    };

}

namespace PUTM_CAN
{

    inline CanTx::CanTx(const char *const interface_name)
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
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(file_descriptor, (sockaddr *)&addr, sizeof(addr)) == -1)
        {
            throw std::runtime_error("bind() failed");
        }
    }

    inline CanTx::~CanTx()
    {
        if (file_descriptor != INVALID_FILE_DESCRIPTOR)
        {
            close(file_descriptor);
        }
    }

    template <typename T>
    inline void CanTx::transmit(const T &tx_frame) const
    {
        static_assert(can_id<T> != INVALID_CAN_ID, "Can id has not been specified");
        can_frame frame;
        frame.can_id = can_id<T>;
        frame.can_dlc = sizeof(T);
        (void)std::memcpy(frame.data, &tx_frame, sizeof(T));
        if (write(file_descriptor, &frame, sizeof(frame)) < 0)
        {
            throw std::runtime_error("write() failed");
        }
    }

    template <typename T>
    inline void CanTx::transmit_rtr() const
    {
        can_frame frame;
        frame.can_id = can_id<T> | CAN_RTR_FLAG;
        frame.can_dlc = sizeof(T);
        if (write(file_descriptor, &frame, sizeof(can_frame)) < 0)
        {
            throw std::runtime_error("write() failed");
        }
    }
}