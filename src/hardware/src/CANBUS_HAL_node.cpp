/**
 * Menggunakan factory pattern untuk memilih hardware CANBUS_HAL yang sesuai
 * canbus1 adalah canbus mobil
 * canbus2 adalah canbus ADAS
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int8.hpp"
#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/global_definitions.hpp"

#include "hardware/CANBUS_HAL.h"
#include "hardware/canable2_slcan.h"
#include "hardware/canable2_socket_can.h"
#include "hardware/chery_canfd.h"

#include <cstddef>
#include <cstdint>

#include <thread>

#define CMD_STEER_ACTIVE 0b01
#define CMD_GAS_ACTIVE 0b10
#define CMD_GAS_FULL_STOP 0b100
#define CMD_GAS_ACCEL_ON 0b1000
#define CMD_ACC_BTN_PRESS 0b10000

class CANBUS_HAL_node : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_fb_steering_angle;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_fb_current_velocity;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_throttle_position;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_brake_position;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_gear_status;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_steer_torque;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_target_steering_angle;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_target_velocity;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_hw_flag;

    std::thread thread_can1_routine;
    std::thread thread_can2_routine;
    std::thread thread_routine_all;

    // Configs
    int can1_type = 0; // 0: CANable2_SLCAN, 1: CANable2_SOCKET_CAN
    int can2_type = 0; // 0: CANable2_SLCAN, 1: CANable2_SOCKET_CAN
    std::string device1_name = "can0";
    std::string device2_name = "can1";
    int baudrate = 500000;
    int fd_baudrate = 2000000;
    int fd;
    int publish_period_ms = 20;
    float MAX_ACCEL = 2.0;  // m/s^2
    float MIN_ACCEL = -3.5; // m/s^2
    float MAX_GAS = 511.0;
    float MIN_GAS = -511.0;

    HelpLogger logger;
    std::unique_ptr<CANBUS_HAL> canbus1_hal; // Pointer to base class
    std::unique_ptr<CANBUS_HAL> canbus2_hal; // Pointer to base class
    rclcpp::Time time_now;
    rclcpp::Time last_time_publish;
    rclcpp::Time time_Start_program;

    // Data kirim untuk can bus
    chery_canfd_lkas_cam_cmd_345_t msg_steer_cmd;
    chery_canfd_lkas_state_t msg_lkas_state_cmd;
    chery_canfd_acc_cmd_t msg_acc_cmd;
    chery_canfd_steer_button_t msg_steer_button_cmd;

    chery_canfd_setting_t msg_setting_dari_adas_cmd;

    float cmd_target_steering_angle = 0;
    float cmd_target_velocity = 0;
    uint8_t cmd_hw_flag = 0;

    uint64_t can1_internal_tick = 0;

    uint8_t is_mobil_initialized = 0;

    uint8_t flag_reset = 0;
    uint8_t counter_berapa_kali_kirim = 0;

    std::vector<uint16_t> intercepted_id;

    bool intercept_steer = false;
    bool intercept_steer_btn = false;
    bool intercept_gas = false;
    bool intercept_lkas_state = false;
    bool intercept_cc_speed = false;

    CANBUS_HAL_node()
        : Node("canbus_hal_node")
    {
        this->declare_parameter("can1_type", can1_type);
        this->get_parameter("can1_type", can1_type);

        this->declare_parameter("can2_type", can2_type);
        this->get_parameter("can2_type", can2_type);

        this->declare_parameter("device1_name", device1_name);
        this->get_parameter("device1_name", device1_name);

        this->declare_parameter("device2_name", device2_name);
        this->get_parameter("device2_name", device2_name);

        this->declare_parameter("baudrate", baudrate);
        this->get_parameter("baudrate", baudrate);

        this->declare_parameter("fd_baudrate", fd_baudrate);
        this->get_parameter("fd_baudrate", fd_baudrate);

        this->declare_parameter("publish_period_ms", publish_period_ms);
        this->get_parameter("publish_period_ms", publish_period_ms);

        std::vector<int64_t> temp_intercepted_ids;
        this->declare_parameter<std::vector<int64_t>>("intercepted_ids_can", temp_intercepted_ids);
        this->get_parameter("intercepted_ids_can", temp_intercepted_ids);

        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        if (device1_name != "")
        {
            if (can1_type == 0)
                canbus1_hal = std::make_unique<CANable2_SLCAN>(&logger);
            else if (can1_type == 1)
                canbus1_hal = std::make_unique<CANable2_SOCKET_CAN>(&logger);

            if (canbus1_hal->init(device1_name, baudrate) != 0)
            {
                logger.error("Failed to initialize CANBUS_HAL can1");
                rclcpp::shutdown();
            }
        }

        if (device2_name != "")
        {
            if (can2_type == 0)
                canbus2_hal = std::make_unique<CANable2_SLCAN>(&logger);
            else if (can2_type == 1)
                canbus2_hal = std::make_unique<CANable2_SOCKET_CAN>(&logger);

            if (canbus2_hal->init(device2_name, baudrate) != 0)
            {
                logger.error("Failed to initialize CANBUS_HAL can2");
                rclcpp::shutdown();
            }
        }

        // Memastikan semua variabel sudah diinisialisasi
        bzero(&msg_steer_cmd, sizeof(msg_steer_cmd));
        bzero(&msg_lkas_state_cmd, sizeof(msg_lkas_state_cmd));
        bzero(&msg_acc_cmd, sizeof(msg_acc_cmd));
        bzero(&msg_steer_button_cmd, sizeof(msg_steer_button_cmd));
        bzero(&msg_setting_dari_adas_cmd, sizeof(msg_setting_dari_adas_cmd));

        time_now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        last_time_publish = time_now;

        pub_fb_steering_angle = this->create_publisher<std_msgs::msg::Float32>("fb_steering_angle", 1);
        pub_fb_current_velocity = this->create_publisher<std_msgs::msg::Float32>("fb_current_velocity", 1);
        pub_throttle_position = this->create_publisher<std_msgs::msg::Float32>("fb_throttle_position", 1);
        pub_brake_position = this->create_publisher<std_msgs::msg::Int16>("fb_brake_position", 1);
        pub_gear_status = this->create_publisher<std_msgs::msg::UInt8>("fb_gear_status", 1);
        pub_steer_torque = this->create_publisher<std_msgs::msg::Int8>("fb_steer_torque", 1);

        sub_target_steering_angle = this->create_subscription<std_msgs::msg::Float32>(
            "cmd_target_steering_angle", 1, std::bind(&CANBUS_HAL_node::callback_sub_target_steering_angle, this, std::placeholders::_1));
        sub_target_velocity = this->create_subscription<std_msgs::msg::Float32>(
            "cmd_target_velocity", 1, std::bind(&CANBUS_HAL_node::callback_sub_target_velocity, this, std::placeholders::_1));
        sub_hw_flag = this->create_subscription<std_msgs::msg::UInt8>(
            "cmd_hw_flag", 1, std::bind(&CANBUS_HAL_node::callback_sub_hw_flag, this, std::placeholders::_1));

        time_Start_program = rclcpp::Clock(RCL_SYSTEM_TIME).now();

        if (device1_name != "" && device2_name != "")
        {
            thread_routine_all = std::thread(std::bind(&CANBUS_HAL_node::callback_routine_all, this), this);
        }

        // Convert ke uint16_t dan store untuk nanti digunakan
        intercepted_id.clear();
        for (auto id : temp_intercepted_ids)
        {
            intercepted_id.push_back(static_cast<uint16_t>(id));
            logger.info("Parameter intercepted ID: 0x%03X", static_cast<uint16_t>(id));
        }

        logger.info("CANBUS_HAL_node node initialized");
    }

    ~CANBUS_HAL_node()
    {
    }

    void callback_sub_target_steering_angle(const std_msgs::msg::Float32::SharedPtr msg)
    {
        cmd_target_steering_angle = msg->data;
    }

    void callback_sub_target_velocity(const std_msgs::msg::Float32::SharedPtr msg)
    {
        cmd_target_velocity = msg->data;
    }

    void callback_sub_hw_flag(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        static uint8_t prev_cmd_hw_flag = 0;
        cmd_hw_flag = msg->data;

        if (((prev_cmd_hw_flag & CMD_ACC_BTN_PRESS) == 0) && ((cmd_hw_flag & CMD_ACC_BTN_PRESS) == CMD_ACC_BTN_PRESS))
        {
            flag_reset = 0;
            counter_berapa_kali_kirim = 0;
        }

        prev_cmd_hw_flag = cmd_hw_flag;
    }

    // =========================================================================================

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
                crc &= 0xFF;
            }
        }
        return (crc ^ xor_output);
    }

    uint8_t crc_str_btn(const uint8_t *data, size_t length)
    {
        const uint8_t poly = 0x1D;
        uint8_t crc = 0xFF;
        for (size_t i = 0; i < length; ++i)
        {
            crc ^= data[i];
            for (int b = 0; b < 8; ++b)
            {
                if (crc & 0x80)
                    crc = (uint8_t)((crc << 1) ^ poly);
                else
                    crc = (uint8_t)(crc << 1);
            }
        }
        crc ^= 0xFF;
        return crc;
    }

    void send_steer_cmd(std::unique_ptr<CANBUS_HAL> &canbus_hal_from_adas, std::unique_ptr<CANBUS_HAL> &canbus_hal_to_send)
    {
        // Copy dari bus adas ke bus mobil
        memcpy(&msg_steer_cmd, &canbus_hal_from_adas->lkas_cam_cmd, sizeof(msg_steer_cmd));

        // Mengisi sesuai target
        msg_steer_cmd.cmd = chery_canfd_lkas_cam_cmd_345_cmd_encode(cmd_target_steering_angle * 180 / 3.141592653589793);
        msg_steer_cmd.lka_active = ((cmd_hw_flag & CMD_STEER_ACTIVE) >> 0x00);
        msg_steer_cmd.set_x0 = 0;

        // Packing can
        can_frame_t frame_steer_cmd;
        bzero(&frame_steer_cmd, sizeof(frame_steer_cmd));
        chery_canfd_lkas_cam_cmd_345_pack(frame_steer_cmd.data, &msg_steer_cmd, sizeof(frame_steer_cmd.data));

        // Menimpa CRC lalu packing lagi
        uint8_t crc = calculate_crc(frame_steer_cmd.data, CHERY_CANFD_LKAS_CAM_CMD_345_LENGTH - 1, 0x1D, 0xA);
        msg_steer_cmd.checksum = crc;
        bzero(&frame_steer_cmd, sizeof(frame_steer_cmd));
        chery_canfd_lkas_cam_cmd_345_pack(frame_steer_cmd.data, &msg_steer_cmd, sizeof(frame_steer_cmd.data));

        // Mengirim ke bus mobil
        frame_steer_cmd.id = CHERY_CANFD_LKAS_CAM_CMD_345_FRAME_ID;
        frame_steer_cmd.dlc = CHERY_CANFD_LKAS_CAM_CMD_345_LENGTH;
        canbus_hal_to_send->send_msg(&frame_steer_cmd);
    }

    void send_cc_speed_cmd(std::unique_ptr<CANBUS_HAL> &canbus_hal_from_adas, std::unique_ptr<CANBUS_HAL> &canbus_hal_to_send)
    {
        // Copy dari bus adas ke bus mobil
        memcpy(&msg_setting_dari_adas_cmd, &canbus_hal_from_adas->setting_cmd_from_adas, sizeof(msg_setting_dari_adas_cmd));

        // Mengisi sesuai target
        msg_setting_dari_adas_cmd.cc_speed = chery_canfd_setting_cc_speed_encode(10.0);

        // Packing can
        can_frame_t frame_setting_dari_adas_cmd;
        bzero(&frame_setting_dari_adas_cmd, sizeof(frame_setting_dari_adas_cmd));
        chery_canfd_setting_pack(frame_setting_dari_adas_cmd.data, &msg_setting_dari_adas_cmd, sizeof(frame_setting_dari_adas_cmd.data));

        // Menimpa CRC lalu packing lagi
        uint8_t crc = calculate_crc(frame_setting_dari_adas_cmd.data, CHERY_CANFD_SETTING_LENGTH - 1, 0x1D, 0xA);
        msg_setting_dari_adas_cmd.checksum = crc;
        bzero(&frame_setting_dari_adas_cmd, sizeof(frame_setting_dari_adas_cmd));
        chery_canfd_setting_pack(frame_setting_dari_adas_cmd.data, &msg_setting_dari_adas_cmd, sizeof(frame_setting_dari_adas_cmd.data));

        // Mengirim ke bus mobil
        frame_setting_dari_adas_cmd.id = CHERY_CANFD_SETTING_FRAME_ID;
        frame_setting_dari_adas_cmd.dlc = CHERY_CANFD_SETTING_LENGTH;
        canbus_hal_to_send->send_msg(&frame_setting_dari_adas_cmd);
    }

    void send_gas_cmd(std::unique_ptr<CANBUS_HAL> &canbus_hal_from_adas, std::unique_ptr<CANBUS_HAL> &canbus_hal_to_send)
    {
        // Clipping
        if (cmd_target_velocity > MAX_ACCEL)
            cmd_target_velocity = MAX_ACCEL;
        if (cmd_target_velocity < MIN_ACCEL)
            cmd_target_velocity = MIN_ACCEL;

        // Normalisasi
        float target_gas = -24;
        if (fabsf(cmd_target_velocity - __FLT_EPSILON__) < 0)
        {
            target_gas = -24;
        }
        else if (cmd_target_velocity > 0)
        {
            float norm_vel = cmd_target_velocity / MAX_ACCEL;
            target_gas = norm_vel * MAX_GAS;
        }
        else if (cmd_target_velocity < 0)
        {
            float norm_vel = cmd_target_velocity / MIN_ACCEL;
            target_gas = norm_vel * MIN_GAS;
        }

        // Clipping
        if (target_gas > MAX_GAS)
            target_gas = MAX_GAS;
        if (target_gas < MIN_GAS)
            target_gas = MIN_GAS;

        // Copy dari bus adas ke bus mobil
        memcpy(&msg_acc_cmd, &canbus_hal_from_adas->acc_cam_cmd, sizeof(msg_acc_cmd));

        // Mengisi sesuai target
        int16_t throttle = -24;
        if ((cmd_hw_flag & CMD_GAS_ACTIVE) == CMD_GAS_ACTIVE)
            throttle = (int16_t)(target_gas);

        uint8_t acc_state = canbus_hal_from_adas->acc_cam_cmd.acc_state;

        if (can1_internal_tick > 2000)
        {
            acc_state = 2;
        }

        if ((cmd_hw_flag & CMD_GAS_FULL_STOP) == CMD_GAS_FULL_STOP)
            acc_state = 2;
        else if ((cmd_hw_flag & CMD_GAS_ACTIVE) == CMD_GAS_ACTIVE)
            acc_state = 3;

        msg_acc_cmd.cmd = (int16_t)throttle;
        if ((cmd_hw_flag & CMD_GAS_FULL_STOP) == CMD_GAS_FULL_STOP)
            msg_acc_cmd.cmd = 400;

        msg_acc_cmd.accel_on = 0;
        if ((cmd_hw_flag & CMD_GAS_ACCEL_ON) == CMD_GAS_ACCEL_ON)
            msg_acc_cmd.accel_on = 1;

        msg_acc_cmd.acc_state = acc_state;

        msg_acc_cmd.stopped = canbus_hal_from_adas->acc_cam_cmd.stopped;
        if ((cmd_hw_flag & CMD_GAS_FULL_STOP) == CMD_GAS_FULL_STOP)
            msg_acc_cmd.stopped = 1;
        else if ((cmd_hw_flag & CMD_GAS_ACTIVE) == CMD_GAS_ACTIVE)
            msg_acc_cmd.stopped = 0;

        msg_acc_cmd.gas_pressed = 0;
        if (target_gas > 0 && ((cmd_hw_flag & CMD_GAS_FULL_STOP) == 0))
            msg_acc_cmd.gas_pressed = 1;

        msg_acc_cmd.counter = (uint8_t)(can1_internal_tick % 0x0f);

        logger.info("Target gas: %.2f, Throttle: %d, Acc state: %d, Stopped: %d, Gas pressed: %d %d -> %d",
                    target_gas, msg_acc_cmd.cmd, msg_acc_cmd.acc_state, msg_acc_cmd.stopped, msg_acc_cmd.gas_pressed, msg_acc_cmd.accel_on, canbus1_hal->steer_sensor.torque_driver);

        // Packing can
        can_frame_t frame_acc_cmd;
        bzero(&frame_acc_cmd, sizeof(frame_acc_cmd));
        chery_canfd_acc_cmd_pack(frame_acc_cmd.data, &msg_acc_cmd, sizeof(frame_acc_cmd.data));

        // Menimpa CRC lalu packing lagi
        uint8_t crc = calculate_crc(frame_acc_cmd.data, CHERY_CANFD_ACC_CMD_LENGTH - 1, 0x1D, 0xA);
        msg_acc_cmd.checksum = crc;
        bzero(&frame_acc_cmd, sizeof(frame_acc_cmd));
        chery_canfd_acc_cmd_pack(frame_acc_cmd.data, &msg_acc_cmd, sizeof(frame_acc_cmd.data));

        // Mengirim ke bus mobil
        frame_acc_cmd.id = CHERY_CANFD_ACC_CMD_FRAME_ID;
        frame_acc_cmd.dlc = CHERY_CANFD_ACC_CMD_LENGTH;
        canbus_hal_to_send->send_msg(&frame_acc_cmd);
    }

    // ini tpi ke adas. buat trigger kah??
    void send_steer_button_cmd(std::unique_ptr<CANBUS_HAL> &canbus_hal_from_adas, std::unique_ptr<CANBUS_HAL> &canbus_hal_to_send)
    {
        // Copy dari bus mobil ke bus afas
        memcpy(&msg_steer_button_cmd, &canbus_hal_to_send->steer_button, sizeof(msg_steer_button_cmd));

        // Mengisi sesuai target

        if (counter_berapa_kali_kirim >= 4)
        {
            msg_steer_button_cmd.acc = 0;
            flag_reset = 1;
            counter_berapa_kali_kirim++;
            if (counter_berapa_kali_kirim > 50)
            {
                counter_berapa_kali_kirim = 50;
            }
        }

        if (flag_reset == 0)
        {
            msg_steer_button_cmd.acc = ((cmd_hw_flag & CMD_ACC_BTN_PRESS) >> 0x04);
            counter_berapa_kali_kirim++;
        }

        // Packing can
        can_frame_t frame_steer_button_cmd;
        bzero(&frame_steer_button_cmd, sizeof(frame_steer_button_cmd));
        chery_canfd_steer_button_pack(frame_steer_button_cmd.data, &msg_steer_button_cmd, sizeof(frame_steer_button_cmd.data));

        // Menimpa CRC lalu packing lagi
        crc_str_btn(&frame_steer_button_cmd.data[1], CHERY_CANFD_STEER_BUTTON_LENGTH - 1);
        bzero(&frame_steer_button_cmd, sizeof(frame_steer_button_cmd));
        chery_canfd_steer_button_pack(frame_steer_button_cmd.data, &msg_steer_button_cmd, sizeof(frame_steer_button_cmd.data));

        // Mengirim ke bus adas
        frame_steer_button_cmd.id = CHERY_CANFD_STEER_BUTTON_FRAME_ID;
        frame_steer_button_cmd.dlc = CHERY_CANFD_STEER_BUTTON_LENGTH;

        canbus_hal_from_adas->send_msg(&frame_steer_button_cmd);
    }

    void send_lkas_state_cmd(std::unique_ptr<CANBUS_HAL> &canbus_hal_from_adas, std::unique_ptr<CANBUS_HAL> &canbus_hal_to_send) // 20 Hz
    {
        // Copy dari bus adas ke bus mobil
        memcpy(&msg_lkas_state_cmd, &canbus_hal_from_adas->lkas_state, sizeof(msg_lkas_state_cmd));

        // Mengisi sesuai target
        msg_lkas_state_cmd.new_signal_1 = 1;
        msg_lkas_state_cmd.new_signal_2 = ((cmd_hw_flag & CMD_STEER_ACTIVE) >> 0x00) ? 2 : 0;
        msg_lkas_state_cmd.new_signal_3 = ((cmd_hw_flag & CMD_STEER_ACTIVE) >> 0x00) ? 2 : 0;
        msg_lkas_state_cmd.new_signal_4 = 1;
        msg_lkas_state_cmd.state = ((cmd_hw_flag & CMD_STEER_ACTIVE) >> 0x00) ? 0 : 1;
        msg_lkas_state_cmd.lka_active = ((cmd_hw_flag & CMD_STEER_ACTIVE) >> 0x00) ? 1 : 0;
        msg_lkas_state_cmd.counter = (uint8_t)(can1_internal_tick % 0x0f);

        // Packing can
        can_frame_t frame_lkas_state_cmd;
        bzero(&frame_lkas_state_cmd, sizeof(frame_lkas_state_cmd));
        chery_canfd_lkas_state_pack(frame_lkas_state_cmd.data, &msg_lkas_state_cmd, sizeof(frame_lkas_state_cmd.data));

        // Menimpa CRC lalu packing lagi
        uint8_t crc = calculate_crc(frame_lkas_state_cmd.data, CHERY_CANFD_LKAS_STATE_LENGTH - 1, 0x1D, 0xA);
        msg_lkas_state_cmd.checksum = crc;
        bzero(&frame_lkas_state_cmd, sizeof(frame_lkas_state_cmd));
        chery_canfd_lkas_state_pack(frame_lkas_state_cmd.data, &msg_lkas_state_cmd, sizeof(frame_lkas_state_cmd.data));

        // Mengirim ke bus mobil
        frame_lkas_state_cmd.id = CHERY_CANFD_LKAS_STATE_FRAME_ID;
        frame_lkas_state_cmd.dlc = CHERY_CANFD_LKAS_STATE_LENGTH;
        canbus_hal_to_send->send_msg(&frame_lkas_state_cmd);
    }

    // =========================================================================================

    void callback_routine_all_routine()
    {
        time_now = rclcpp::Clock(RCL_SYSTEM_TIME).now();

        std::vector<can_frame_t> canbus1_frames = canbus1_hal->recv_msgs();
        canbus1_hal->update();
        canbus2_hal->send_msgs(canbus1_frames);

        std::vector<can_frame_t> canbus2_frames = canbus2_hal->recv_msgs();
        canbus2_hal->update();
        canbus1_hal->send_msgs(canbus2_frames);

        // time_now - time_Start_program > rclcpp::Duration(2, 0)
        if (can1_internal_tick > 5000 && !is_mobil_initialized)
        {
            is_mobil_initialized = 1;

            // Gunakan parameter intercepted_ids_can jika tersedia, kalau tidak pakai hardcoded
            if (!intercepted_id.empty())
            {
                // Gunakan dari parameter
                for (auto id : intercepted_id)
                {
                    canbus1_hal->intercepted_can_ids.push_back(id);
                    canbus2_hal->intercepted_can_ids.push_back(id);
                    logger.info("Added intercepted ID from parameter: 0x%03X", id);

                    if (id == CHERY_CANFD_LKAS_CAM_CMD_345_FRAME_ID)
                        intercept_steer = true;

                    if (id == CHERY_CANFD_LKAS_STATE_FRAME_ID)
                        intercept_lkas_state = true;

                    if (id == CHERY_CANFD_STEER_BUTTON_FRAME_ID)
                        intercept_steer_btn = true;

                    if (id == CHERY_CANFD_ACC_CMD_FRAME_ID)
                        intercept_gas = true;

                    if (id == CHERY_CANFD_SETTING_FRAME_ID)
                        intercept_cc_speed = true;
                }
            }
            logger.info("Mobil sudah diinisialisasi dengan %zu intercepted IDs",
                        canbus1_hal->intercepted_can_ids.size());
        }

        if (is_mobil_initialized)
        {
            static uint16_t divider_50_hz = 0;

            if (divider_50_hz++ >= 20)
            {
                divider_50_hz = 0;
                if (intercept_steer)
                    send_steer_cmd(canbus2_hal, canbus1_hal);

                if (intercept_steer_btn)
                    send_steer_button_cmd(canbus2_hal, canbus1_hal);

                if (intercept_gas)
                    send_gas_cmd(canbus2_hal, canbus1_hal);
            }

            static uint16_t divider_20_hz = 0;
            if (divider_20_hz++ >= 50)
            {
                divider_20_hz = 0;
                if (intercept_lkas_state)
                    send_lkas_state_cmd(canbus2_hal, canbus1_hal);

                if (intercept_cc_speed)
                    send_cc_speed_cmd(canbus2_hal, canbus1_hal);
            }
        }

        can1_internal_tick++;

        // Memastikan publish sesuai dengan periode yang diinginkan
        rclcpp::Duration dt_publish = time_now - last_time_publish;
        if (dt_publish.seconds() * 1000 < publish_period_ms && publish_period_ms != -1)
            return;
        last_time_publish = time_now;

        std_msgs::msg::Float32 msg_steering_angle;
        msg_steering_angle.data = canbus1_hal->fb_steering_angle;
        pub_fb_steering_angle->publish(msg_steering_angle);

        std_msgs::msg::Float32 msg_current_velocity;
        msg_current_velocity.data = canbus1_hal->fb_current_velocity;
        pub_fb_current_velocity->publish(msg_current_velocity);

        std_msgs::msg::Float32 msg_throttle_position;
        msg_throttle_position.data = (float)canbus1_hal->engine_gas;
        pub_throttle_position->publish(msg_throttle_position);

        std_msgs::msg::Int16 msg_brake_position;
        msg_brake_position.data = canbus1_hal->data_brake_pos;
        pub_brake_position->publish(msg_brake_position);

        std_msgs::msg::UInt8 msg_gear_status;
        msg_gear_status.data = canbus1_hal->engine_gear;
        pub_gear_status->publish(msg_gear_status);

        std_msgs::msg::Int8 msg_steer_torque;
        msg_steer_torque.data = canbus1_hal->steer_sensor.torque_driver;
        pub_steer_torque->publish(msg_steer_torque);
    }

    void
    callback_routine_all()
    {
        while (rclcpp::ok())
        {
            callback_routine_all_routine();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void callback_can2()
    {
        while (rclcpp::ok())
        {
            callback_can2_routine();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void callback_can1()
    {
        while (rclcpp::ok())
        {
            callback_can1_routine();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void callback_can2_routine()
    {
        std::vector<can_frame_t> canbus2_frames = canbus2_hal->recv_msgs();
        canbus2_hal->update();

        canbus1_hal->send_msgs(canbus2_frames);
    }

    void callback_can1_routine()
    {
        time_now = rclcpp::Clock(RCL_SYSTEM_TIME).now();

        logger.info("%.2f %.2f %d", cmd_target_steering_angle, cmd_target_velocity, cmd_hw_flag);

        std::vector<can_frame_t> canbus1_frames = canbus1_hal->recv_msgs();
        canbus1_hal->update();

        canbus2_hal->send_msgs(canbus1_frames);
        // send 20 ms
        if (can1_internal_tick % 2 == 0)
        {
            send_steer_cmd(canbus2_hal, canbus1_hal);
            send_gas_cmd(canbus2_hal, canbus1_hal);
        }

        // send 50 ms
        if (can1_internal_tick % 5 == 0)
        {
            send_lkas_state_cmd(canbus2_hal, canbus1_hal);
        }

        can1_internal_tick++; // Increment internal tick every 10 ms

        // Memastikan publish sesuai dengan periode yang diinginkan
        rclcpp::Duration dt_publish = time_now - last_time_publish;
        if (dt_publish.seconds() * 1000 < publish_period_ms && publish_period_ms != -1)
            return;
        last_time_publish = time_now;

        std_msgs::msg::Float32 msg_steering_angle;
        msg_steering_angle.data = canbus1_hal->fb_steering_angle;
        pub_fb_steering_angle->publish(msg_steering_angle);

        std_msgs::msg::Float32 msg_current_velocity;
        msg_current_velocity.data = canbus1_hal->fb_current_velocity;
        pub_fb_current_velocity->publish(msg_current_velocity);

        std_msgs::msg::Float32 msg_throttle_position;
        msg_throttle_position.data = (float)canbus1_hal->engine_gas;
        pub_throttle_position->publish(msg_throttle_position);

        std_msgs::msg::Int16 msg_brake_position;
        msg_brake_position.data = canbus1_hal->data_brake_pos;
        pub_brake_position->publish(msg_brake_position);

        std_msgs::msg::UInt8 msg_gear_status;
        msg_gear_status.data = canbus1_hal->engine_gear;
        pub_gear_status->publish(msg_gear_status);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_canbus_hal_node = std::make_shared<CANBUS_HAL_node>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_canbus_hal_node);
    executor.spin();

    return 0;
}