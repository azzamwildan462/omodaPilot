#ifndef CANABLE2_SOCKET_CAN_H
#define CANABLE2_SOCKET_CAN_H

#include <hardware/CANBUS_HAL.h>

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>

// Ensure CAN FD support is available
#ifndef CAN_RAW_FD_FRAMES
#define CAN_RAW_FD_FRAMES 5
#endif

#ifndef CANFD_BRS
#define CANFD_BRS 0x01
#endif

#ifndef CANFD_ESI
#define CANFD_ESI 0x02
#endif

// CAN FD frame structure (if not defined by system)
#ifndef CANFD_MTU
#define CANFD_MTU (sizeof(struct canfd_frame))

struct canfd_frame
{
    canid_t can_id; /* 32 bit CAN_ID + EFF/RTR/ERR flags */
    __u8 len;       /* frame payload length in byte */
    __u8 flags;     /* additional flags for CAN FD */
    __u8 __res0;    /* reserved / padding */
    __u8 __res1;    /* reserved / padding */
    __u8 data[64] __attribute__((aligned(8)));
};
#endif

class CANable2_SOCKET_CAN : public CANBUS_HAL
{
public:
    CANable2_SOCKET_CAN(HelpLogger *logger_ptr)
        : CANBUS_HAL(logger_ptr)
    {
        sockfd = -1;
        use_canfd = false;
    };

    int init(std::string device_name, int baudRate, int fd_baudrate = 2000000) override;

    int send_msg(can_frame_t *can_msg) override;
    can_frame_t recv_msg() override;
    int send_msgs(const std::vector<can_frame_t> &can_msgs) override;
    std::vector<can_frame_t> recv_msgs() override;

    int update(uint64_t tick) override;
    int init_update_as_new_thread() override;

    void shutdown() override;

private:
    int sockfd = -1;
    bool use_canfd = false;
    int setup_can_interface(const std::string &interface_name, int bitrate, int fd_bitrate);
    int parse_incoming_data(can_frame_t *frame);
    int dlc_to_len(uint8_t dlc);
    uint8_t len_to_dlc(int len);
};

#endif // CANABLE2_SOCKET_CAN_H