// serial_posix.cpp
#include <iostream>
#include <string>
#include <vector>
#include <cerrno>
#include <cstring>

#include <fcntl.h>     // open
#include <termios.h>   // termios, tcgetattr, tcsetattr, cfsetspeed
#include <unistd.h>    // read, write, close
#include <sys/ioctl.h> // tcdrain

#include "chery_canfd.h"

#define CAN_MAX_DATA_FRAME 128
#define CAN_MTU 1024

typedef struct canfd_frame_t
{
    uint16_t can_id;                      // 32 bit CAN_ID + EFF/RTR/ERR flags
    uint8_t dlc;                          // frame payload length in byte (0 .. CANFD_MAX_DLEN)
    uint8_t flags;                        // additional flags for CAN FD
    uint8_t data[CAN_MAX_DATA_FRAME - 5]; // CAN frame payload (up to 64 byte)
    uint8_t has_proceeded;                // internal use
} canfd_frame_t;

chery_canfd_lkas_cam_cmd_345_t lkas_cam_cmd;
chery_canfd_steer_angle_sensor_t angle_sensor;

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

std::vector<canfd_frame_t> parse_can_msg(char *buf, size_t len)
{
    std::vector<canfd_frame_t> frames;

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

        canfd_frame_t frame;
        frame.can_id = id_i * 16 * 16 + id_ii * 16 + id_iii;

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

        frame.flags = 0; // No special flags for now
        frame.has_proceeded = 0;

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
            // frame.data[j] = (buf[i + 5 + j * 2] - '0') * 16 + (buf[i + 6 + j * 2] - '0');
            frame.data[j] = char_hex2byte(buf[i + 5 + j * 2]) * 16 + char_hex2byte(buf[i + 6 + j * 2]);
        }

        if (frame.can_id == 0x1d3)
        {
            // bzero(&lkas_cam_cmd, sizeof(lkas_cam_cmd));
            // chery_canfd_lkas_cam_cmd_345_unpack(&lkas_cam_cmd, frame.data, frame.dlc);

            // double steer = chery_canfd_steer_angle_sensor_steer_angle_decode(lkas_cam_cmd.cmd);

            // printf("LKAS: %d %d %d %.2f\n", lkas_cam_cmd.cmd, lkas_cam_cmd.lka_active, lkas_cam_cmd.set_x0, steer);

            bzero(&angle_sensor, sizeof(angle_sensor));
            chery_canfd_steer_angle_sensor_unpack(&angle_sensor, frame.data, frame.dlc);
            printf("Angle: %d %.2f\n", angle_sensor.steer_angle, chery_canfd_steer_angle_sensor_steer_angle_decode(angle_sensor.steer_angle));
        }

        i += frame.dlc * 2 + 4; // Move index to the end of the current frame

        frames.push_back(frame);
    }

    return frames;
}

int build_can_msg(canfd_frame_t *frame, char *ret_buf)
{
    char msg_send[CAN_MAX_DATA_FRAME];
    memset(msg_send, 0, CAN_MAX_DATA_FRAME);

    msg_send[0] = 'b';
    snprintf(&msg_send[1], 4, "%03X", frame->can_id);
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

uint8_t calculate_crc(const uint8_t *data, size_t length, uint8_t poly, uint8_t xor_output)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ poly;
            }
            else
            {
                crc <<= 1;
            }
            crc &= 0xFF; // Ensure crc remains within 8 bits
        }
    }
    return (crc ^ xor_output);
}

int main()
{
    const char *smpl_can_data_terima = "b3458F966000000000000\rasd123asdb123401020304";
    char smpl_can_data_kirim[CAN_MTU];
    size_t idx_pos_kirim = 0;

    std::vector<canfd_frame_t> can_terima = parse_can_msg(const_cast<char *>(smpl_can_data_terima), std::strlen(smpl_can_data_terima));
    for (const auto &frame : can_terima)
    {
        printf("CAN ID: %X %d | %d\n", frame.can_id, frame.can_id, frame.dlc);
        printf("Data: ");
        for (size_t j = 0; j < frame.dlc; j++)
        {
            printf("%02X ", frame.data[j]);
        }

        printf("\n");

        char rts_data[CAN_MAX_DATA_FRAME];
        memset(rts_data, 0, CAN_MAX_DATA_FRAME);
        build_can_msg(const_cast<canfd_frame_t *>(&frame), rts_data);
        printf("rts %s\n", rts_data);

        memcpy(smpl_can_data_kirim + idx_pos_kirim, rts_data, strlen(rts_data));
        memset(smpl_can_data_kirim + idx_pos_kirim + 1, 49, 1); // Tambah '\r'
        idx_pos_kirim = strlen(rts_data) + 1;

        printf("==========================\n\n");
    }

    printf("semua: %s\n", smpl_can_data_kirim);

    printf("==========================\n\n");

    // data hex kirim b3457A9C000CBEFE2ABA,
    // pack data
    canfd_frame_t frame_kirim;
    frame_kirim.can_id = 0x345;
    frame_kirim.dlc = 8;
    frame_kirim.data[0] = 0x7A;
    frame_kirim.data[1] = 0x9C;
    frame_kirim.data[2] = 0x00;
    frame_kirim.data[3] = 0x0C;
    frame_kirim.data[4] = 0xBE;
    frame_kirim.data[5] = 0xFE;
    frame_kirim.data[6] = 0x2A;
    frame_kirim.data[7] = 0xBA;

    uint8_t crc = calculate_crc(frame_kirim.data, 7, 0x1D, 0xA);
    // print data and crc
    printf("CAN ID: %X %d | %d\n", frame_kirim.can_id, frame_kirim.can_id, frame_kirim.dlc);
    printf("Data: ");
    for (size_t j = 0; j < frame_kirim.dlc; j++)
    {
        printf("%02X ", frame_kirim.data[j]);
    }
    printf("\n");
    printf("CRC: %02X\n", crc);

    frame_kirim.data[0] = 0x7B;
    frame_kirim.data[1] = 0x34;
    frame_kirim.data[2] = 0x00;
    frame_kirim.data[3] = 0xE3;
    frame_kirim.data[4] = 0x06;
    frame_kirim.data[5] = 0x00;
    frame_kirim.data[6] = 0x13;
    frame_kirim.data[7] = 0xB0;

    crc = calculate_crc(frame_kirim.data, 7, 0x1D, 0xA);
    printf("CAN ID: %X %d | %d\n", frame_kirim.can_id, frame_kirim.can_id, frame_kirim.dlc);
    printf("Data: ");
    for (size_t j = 0; j < frame_kirim.dlc; j++)
    {
        printf("%02X ", frame_kirim.data[j]);
    }
    printf("\n");
    printf("CRC: %02X\n", crc);

    return 0;
}