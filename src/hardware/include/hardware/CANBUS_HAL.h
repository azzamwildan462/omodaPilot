#ifndef CANBUS_HAL_H
#define CANBUS_HAL_H

#include <stdint.h>
#include <vector>
#include <string>

#include "ros2_utils/help_logger.hpp"
#include "hardware/chery_canfd.h"

#define CAN_MAX_DATA_FRAME 128
#define CAN_MTU 1024

typedef struct can_frame_t
{
    uint16_t id;
    uint8_t dlc;
    uint8_t data[CAN_MAX_DATA_FRAME];
    uint64_t timestamp;
    uint8_t flags; // 0: normal, 1: extended, 2: remote, 4: FD
} can_frame_t;

class CANBUS_HAL
{
    /**
     * Data data untuk ros pubsub
     */
public:
    float target_steering_angle; // rad

    float cmd_target_steering_angle = 0;
    float cmd_target_velocity = 0;
    uint8_t cmd_hw_flag = 0;

    float fb_steering_angle;   // rad
    float fb_current_velocity; // m/s

    /**
     * Data data interface can bus hal
     */
    chery_canfd_lkas_cam_cmd_345_t lkas_cam_cmd; // ke mobil
    chery_canfd_lkas_state_t lkas_state;         // ke mobil
    chery_canfd_acc_cmd_t acc_cam_cmd;           // ke mobil
    chery_canfd_steer_button_t button_cmd;       // ke adas

    chery_canfd_setting_t setting_cmd_from_adas;

    chery_canfd_acc_t acc_data;

    chery_canfd_steer_angle_sensor_t angle_sensor;

    chery_canfd_steer_sensor_2_t steer_sensor_2;
    float fb_steering_torq_drv;

    chery_canfd_wheel_speed_rear_t wheel_speed_rear;
    chery_canfd_wheel_speed_frnt_t wheel_speed_front;

    float wheel_speed_rl; // km/h
    float wheel_speed_rr; // km/h
    float wheel_speed_fl; // km/h
    float wheel_speed_fr; // km/h

    float speed_cc;
    uint8_t acc_avail;

    HelpLogger *logger;

    chery_canfd_engine_data_t engine_data;
    double engine_gas_pos;
    double engine_gas;
    uint8_t engine_gear;
    uint8_t engine_gear_button;
    uint8_t engine_brake_press;
    uint8_t engine_switch_to_p;

    std::string gear_status;
    std::string gear_button_status;

    chery_canfd_steer_button_t steer_button;
    uint8_t btn_acc;
    uint8_t btn_cc;
    uint8_t btn_res_plus;
    uint8_t btn_res_minus;
    uint8_t btn_gap_adjust_up;
    uint8_t btn_gap_adjust_down;

    chery_canfd_brake_data_t brake_data;
    int16_t data_brake_pos;

    chery_canfd_bcm_signal_1_t bcm_signal_1;
    uint8_t bcm_door_lock;
    uint8_t bcm_rl_door_open;
    uint8_t bcm_rr_door_open;
    uint8_t bcm_fl_door_open;
    uint8_t bcm_fr_door_open;
    uint8_t bcm_sign_signal;

    chery_canfd_bcm_signal_2_t bcm_signal_2;
    uint8_t bcm_left_sign;
    uint8_t bcm_right_sign;
    uint8_t bcm_door_lock_open;
    uint8_t bcm_high_beam;
    uint8_t bcm_left_sign_pressed;
    uint8_t bcm_right_sign_pressed;
    uint8_t bcm_wiper_button;

    chery_canfd_bsm_right_t bsm_right;
    uint8_t bsm_right_detect;

    chery_canfd_bsm_left_t bsm_left;
    uint8_t bsm_left_detect;

    uint8_t is_can_to_adas = 0;

    std::vector<uint16_t> intercepted_can_ids;

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

/**
 *
 *
 * 7b = 01111011
 * 34 = 00110100 01111011
 *
 * 00110100 01111011
 *
 * 11001101000000
 *
 * 10011010000000
 */