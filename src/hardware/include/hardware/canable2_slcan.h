#ifndef CANABLE2_SLCAN_H
#define CANABLE2_SLCAN_H

#include <hardware/CANBUS_HAL.h>

#include <iostream>
#include <string>
#include <vector>
#include <cerrno>
#include <cstring>

#include <fcntl.h>     // open
#include <termios.h>   // termios, tcgetattr, tcsetattr, cfsetspeed
#include <unistd.h>    // read, write, close
#include <sys/ioctl.h> // tcdrain

class CANable2_SLCAN : public CANBUS_HAL
{
public:
    CANable2_SLCAN(HelpLogger *logger_ptr)
        : CANBUS_HAL(logger_ptr) {

          };

    int init(std::string device_name, int baudRate, int fd_baudrate = 2000000) override;

    int send_msg(can_frame_t *can_msg) override;
    can_frame_t recv_msg() override;
    int send_msgs(const std::vector<can_frame_t> &can_msgs) override;
    std::vector<can_frame_t> recv_msgs() override;

    int update() override;
    int init_update_as_new_thread() override;

    void shutdown() override;

private:
    bool set_serial(int fd, speed_t speed);
    int init_slcan(int fd, const char *can_bitrate, const char *fdcan_bitrate);
    std::vector<can_frame_t> parse_can_msg(char *buf, size_t len);
    int build_can_msg(can_frame_t *frame, char *ret_buf);
    int parse_incoming_data(can_frame_t *frame);

    uint8_t char_hex2byte(char c)
    {
        if (c >= '0' && c <= '9')
            return c - '0';
        if (c >= 'A' && c <= 'F')
            return c - 'A' + 10;
        if (c >= 'a' && c <= 'f')
            return c - 'a' + 10;
        return 0; // Invalid character
    }
};

#endif // CANABLE2_SLCAN_H