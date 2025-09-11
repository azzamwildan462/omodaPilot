#include "hardware/canable2_slcan.h"

#include <iostream>
#include <string>
#include <vector>
#include <cerrno>
#include <cstring>

#include <fcntl.h>     // open
#include <termios.h>   // termios, tcgetattr, tcsetattr, cfsetspeed
#include <unistd.h>    // read, write, close
#include <sys/ioctl.h> // tcdrain

#define SLCAN_MAX_DLC 15

int CANable2_SLCAN::init(std::string device_name, int baudRate, int fd_baudrate)
{
    (void)device_name;
    (void)baudRate;
    (void)fd_baudrate;
    logger->info("CANable2_SLCAN initialized on %s with baudrate %d %d", device_name.c_str(), baudRate, fd_baudrate);
    return 0;
}

int CANable2_SLCAN::send_msg(can_frame_t *can_msg)
{
    (void)can_msg;
    return 0;
}

can_frame_t CANable2_SLCAN::recv_msg()
{
    can_frame_t frame;
    return frame;
}

int CANable2_SLCAN::send_msgs(const std::vector<can_frame_t> &can_msgs)
{
    (void)can_msgs;
    return 0;
}

std::vector<can_frame_t> CANable2_SLCAN::recv_msgs()
{
    std::vector<can_frame_t> frames;
    return frames;
}

int CANable2_SLCAN::update()
{
    return 0;
}

int CANable2_SLCAN::init_update_as_new_thread()
{
    return 0;
}

void CANable2_SLCAN::shutdown()
{
}
