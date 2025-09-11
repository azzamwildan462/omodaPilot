#ifndef CANBUS_HAL_H
#define CANBUS_HAL_H

#include <stdint.h>
#include <vector>
#include <string>

#include "ros2_utils/help_logger.hpp"

#define CAN_MAX_DATA_FRAME 128
#define CAN_MTU 1024

typedef struct can_frame_t
{
    uint16_t id;
    uint8_t dlc;
    uint8_t data[CAN_MAX_DATA_FRAME];
    uint64_t timestamp;
} can_frame_t;

class CANBUS_HAL
{
    /**
     * Data data untuk ros pubsub
     */
public:
    float target_steering_angle; // rad
    float target_velocity;       // m/s
    float target_brake;
    uint32_t acc_setting;
    uint32_t hud_alert;

    float fb_steering_angle;   // rad
    float fb_current_velocity; // m/s
    float fb_brake;
    float fb_throttle;
    uint32_t fb_buttons_bus1;
    uint32_t fb_buttons_bus2;

    HelpLogger *logger;

    /**
     * Data data internal untuk canbus hal
     */
public:
    int fd; // File descriptor for the CAN bus interface
    std::string device_name;

    CANBUS_HAL(HelpLogger *logger_ptr)
    {
        logger = logger_ptr;
        fd = -1;
        device_name = "can0";
    }

    virtual int init(std::string device_name, int baudRate, int fd_baudrate = 2000000) = 0;

    virtual int send_msg(can_frame_t *can_msg) = 0;
    virtual can_frame_t recv_msg() = 0;
    virtual int send_msgs(const std::vector<can_frame_t> &can_msgs) = 0;
    virtual std::vector<can_frame_t> recv_msgs() = 0;

    virtual int update() = 0;
    virtual int init_update_as_new_thread() = 0;

    virtual void shutdown() = 0;
};

#endif // CANBUS_HAL_H