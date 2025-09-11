#include "hardware/canable2_socket_can.h"

int CANable2_SOCKET_CAN::init(std::string device_name, int baudRate, int fd_baudrate)
{
    (void)device_name;
    (void)baudRate;
    (void)fd_baudrate;
    logger->info("CANable2_SOCKET_CAN initialized (dummy)");
    return 0;
}

int CANable2_SOCKET_CAN::send_msg(can_frame_t *can_msg)
{
    (void)can_msg;
    return 0;
}

can_frame_t CANable2_SOCKET_CAN::recv_msg()
{
    can_frame_t frame;
    return frame;
}

int CANable2_SOCKET_CAN::send_msgs(const std::vector<can_frame_t> &can_msgs)
{
    (void)can_msgs;
    return 0;
}

std::vector<can_frame_t> CANable2_SOCKET_CAN::recv_msgs()
{
    std::vector<can_frame_t> frames;
    return frames;
}

int CANable2_SOCKET_CAN::update()
{
    return 0;
}

int CANable2_SOCKET_CAN::init_update_as_new_thread()
{
    return 0;
}

void CANable2_SOCKET_CAN::shutdown()
{
}
