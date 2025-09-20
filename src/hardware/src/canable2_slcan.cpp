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

int CANable2_SLCAN::parse_incoming_data(can_frame_t *frame)
{
    if (frame->id == CHERY_CANFD_STEER_ANGLE_SENSOR_FRAME_ID)
    {
        bzero(&angle_sensor, sizeof(angle_sensor));
        chery_canfd_steer_angle_sensor_unpack(&angle_sensor, frame->data, frame->dlc);
        this->fb_steering_angle = chery_canfd_steer_angle_sensor_steer_angle_decode(angle_sensor.steer_angle) * 3.141592653589793 / 180.0;
    }
    else if (frame->id == CHERY_CANFD_WHEEL_SPEED_REAR_FRAME_ID)
    {
        bzero(&wheel_speed_rear, sizeof(wheel_speed_rear));
        chery_canfd_wheel_speed_rear_unpack(&wheel_speed_rear, frame->data, frame->dlc);
        this->wheel_speed_rl = chery_canfd_wheel_speed_rear_wheel_speed_rl_decode(wheel_speed_rear.wheel_speed_rl);
        this->wheel_speed_rr = chery_canfd_wheel_speed_rear_wheel_speed_rr_decode(wheel_speed_rear.wheel_speed_rr);
    }
    else if (frame->id == CHERY_CANFD_WHEEL_SPEED_FRNT_FRAME_ID)
    {
        bzero(&wheel_speed_front, sizeof(wheel_speed_front));
        chery_canfd_wheel_speed_frnt_unpack(&wheel_speed_front, frame->data, frame->dlc);
        this->wheel_speed_fl = chery_canfd_wheel_speed_frnt_wheel_speed_fl_decode(wheel_speed_front.wheel_speed_fl);
        this->wheel_speed_fr = chery_canfd_wheel_speed_frnt_wheel_speed_fr_decode(wheel_speed_front.wheel_speed_fr);
    }

    return 0;
}

bool CANable2_SLCAN::set_serial(int fd, speed_t speed)
{
    termios tty{};
    if (tcgetattr(fd, &tty) != 0)
    {
        std::cerr << "tcgetattr: " << strerror(errno) << "\n";
        return false;
    }

    // Raw mode
    cfmakeraw(&tty);

    switch (speed)
    {
    case 9600:
        cfsetispeed(&tty, B9600);
        cfsetospeed(&tty, B9600);
        break;
    case 19200:
        cfsetispeed(&tty, B19200);
        cfsetospeed(&tty, B19200);
        break;
    case 38400:
        cfsetispeed(&tty, B38400);
        cfsetospeed(&tty, B38400);
        break;
    case 57600:
        cfsetispeed(&tty, B57600);
        cfsetospeed(&tty, B57600);
        break;
    case 115200:
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);
        break;
    case 1000000:
        cfsetispeed(&tty, B1000000);
        cfsetospeed(&tty, B1000000);
        break;
    }

    // 8N1, no flow control
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= (CLOCAL | CREAD);

    // Set BN1
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // no SW flow control
    // tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // Noncanonical read with timeout:
    // VMIN=0, VTIME=10 => read() returns immediately with available bytes,
    // or waits up to 1.0s (10 * 100ms) for at least 1 byte.
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;

    // Apply settings now
    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        std::cerr << "tcsetattr: " << strerror(errno) << "\n";
        return false;
    }

    // Optional: flush I/O
    tcflush(fd, TCIOFLUSH);
    return true;
}
int CANable2_SLCAN::init_slcan(int fd, const char *can_bitrate, const char *fdcan_bitrate)
{
    {
        ssize_t w = write(fd, can_bitrate, std::strlen(can_bitrate));
        if (w < 0)
        {
            std::cerr << "write: " << strerror(errno) << "\n";
            return 1;
        }
    }

    usleep(300000);
    if (tcdrain(fd) != 0)
    {
        std::cerr << "tcdrain: " << strerror(errno) << "\n";
        return 1;
    }

    {
        ssize_t w = write(fd, fdcan_bitrate, std::strlen(fdcan_bitrate));
        if (w < 0)
        {
            std::cerr << "write: " << strerror(errno) << "\n";
            return 1;
        }
    }

    usleep(300000);
    if (tcdrain(fd) != 0)
    {
        std::cerr << "tcdrain: " << strerror(errno) << "\n";
        return 1;
    }

    {
        const char *cmd = "O\r\n";
        ssize_t w = write(fd, cmd, std::strlen(cmd));
        if (w < 0)
        {
            std::cerr << "write: " << strerror(errno) << "\n";
            close(fd);
            return 1;
        }
    }

    usleep(300000);
    if (tcdrain(fd) != 0)
    {
        std::cerr << "tcdrain: " << strerror(errno) << "\n";
        return 1;
    }

    return 0;
}
std::vector<can_frame_t> CANable2_SLCAN::parse_can_msg(char *buf, size_t len)
{
    std::vector<can_frame_t> frames;

    for (size_t i = 0; i < len; i++)
    {
        if (buf[i] != 'b')
        {
            continue; // Not a CAN FD frame
        }

        if (i + 5 >= len)
        {
            perror("Incomplete CAN FD frame");
            break; // Incomplete frame
        }

        uint8_t id_i = char_hex2byte(buf[i + 1]);
        uint8_t id_ii = char_hex2byte(buf[i + 2]);
        uint8_t id_iii = char_hex2byte(buf[i + 3]);

        can_frame_t frame;
        frame.id = id_i * 16 * 16 + id_ii * 16 + id_iii;

        frame.dlc = buf[i + 4] - '0';
        if (frame.dlc > 8)
        {
            switch (frame.dlc)
            {
            case 9:
                frame.dlc = 12;
                break;
            case 10:
                frame.dlc = 16;
                break;
            case 11:
                frame.dlc = 20;
                break;
            case 12:
                frame.dlc = 24;
                break;
            case 13:
                frame.dlc = 32;
                break;
            case 14:
                frame.dlc = 48;
                break;
            case 15:
                frame.dlc = 64;
                break;
            default:
                frame.dlc = 0; // Fallback to 8
                perror("Invalid DLC");
                break;
            }
        }

        frame.flags = 0;  // No special flags for now
        frame.flags |= 4; // FD frame

        if (frame.dlc * 2 + 5 + i > len)
        {
            perror("Incomplete CAN FD frame data");
            break; // Incomplete frame data
        }

        if (frame.dlc > (CAN_MAX_DATA_FRAME - 5))
        {
            perror("DLC exceeds maximum data frame size");
            break; // DLC exceeds maximum data frame size
        }

        // Data bytes
        for (size_t j = 0; j < frame.dlc; j++)
        {
            frame.data[j] = char_hex2byte(buf[i + 5 + j * 2]) * 16 + char_hex2byte(buf[i + 6 + j * 2]);
        }

        parse_incoming_data(&frame);

        frames.push_back(frame);

        i += frame.dlc * 2 + 4; // Move index to the end of this frame
    }

    return frames;
}
int CANable2_SLCAN::build_can_msg(can_frame_t *frame, char *ret_buf)
{
    char msg_send[CAN_MAX_DATA_FRAME];
    memset(msg_send, 0, CAN_MAX_DATA_FRAME);

    msg_send[0] = 'b';
    snprintf(&msg_send[1], 4, "%03X", frame->id);
    if (frame->dlc <= 8)
    {
        msg_send[4] = '0' + frame->dlc;
    }
    else
    {
        switch (frame->dlc)
        {
        case 12:
            msg_send[4] = '9';
            break;
        case 16:
            msg_send[4] = 'A';
            break;
        case 20:
            msg_send[4] = 'B';
            break;
        case 24:
            msg_send[4] = 'C';
            break;
        case 32:
            msg_send[4] = 'D';
            break;
        case 48:
            msg_send[4] = 'E';
            break;
        case 64:
            msg_send[4] = 'F';
            break;
        default:
            msg_send[4] = '8'; // Fallback to 8
            perror("Invalid DLC");
            break;
        }
    }
    for (size_t j = 0; j < frame->dlc; j++)
    {
        snprintf(&msg_send[5 + j * 2], 3, "%02X", frame->data[j]);
    }
    // msg_send[5 + frame->dlc * 2] = '\r';
    // msg_send[5 + frame->dlc * 2 + 1] = '\0';

    memcpy(ret_buf, msg_send, strlen(msg_send));

    return 0;
}

int CANable2_SLCAN::init(std::string device_name, int baudRate, int fd_baudrate)
{
    (void)device_name;
    (void)baudRate;
    (void)fd_baudrate;
    logger->info("CANable2_SLCAN initialized on %s with baudrate %d %d", device_name.c_str(), baudRate, fd_baudrate);

    int fd = open(device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
        logger->error("Error opening serial port %s: %s", device_name.c_str(), strerror(errno));
        return -1;
    }

    if (!set_serial(fd, 1000000))
    {
        logger->error("Error setting serial port parameters: %s", strerror(errno));
        close(fd);
        return -1;
    }

    if (init_slcan(fd, "S6\r", "Y2\r") != 0)
    {
        logger->error("Error initializing SLCAN interface");
        close(fd);
        return -1;
    }

    this->fd = fd;

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

    char buf[CAN_MTU];
    bzero(buf, CAN_MTU);
    ssize_t n = read(this->fd, buf, CAN_MTU);
    if (n < 0)
    {
        if (errno = EINTR)
        {
            // Interrupted, try again
            // std::cerr << "read: " << strerror(errno) << "\n";
            logger->error("Interrupted read from serial port: %s", strerror(errno));
            return frames;
        }
        else if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            logger->error("No data available on serial port: %s", strerror(errno));
            return frames;
        }
        else
        {
            logger->error("Error reading from serial port: %s", strerror(errno));
            return frames;
        }
    }
    else if (n == 0)
    {
        // EOF
        logger->error("EOF on serial port");
        return frames;
    }

    frames = parse_can_msg(buf, n);

    return frames;
}

int CANable2_SLCAN::update()
{
    // Hitung speed
    float raw_kmh_longitudinal = (this->wheel_speed_rl + this->wheel_speed_rr) / 2.0f;
    this->fb_current_velocity = raw_kmh_longitudinal / 3.6f;

    return 0;
}

int CANable2_SLCAN::init_update_as_new_thread()
{
    return 0;
}

void CANable2_SLCAN::shutdown()
{
}
