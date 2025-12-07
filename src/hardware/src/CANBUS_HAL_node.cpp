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
#include "ros2_utils/pid.hpp"

#include "hardware/CANBUS_HAL.h"
#include "hardware/canable2_slcan.h"
#include "hardware/canable2_socket_can.h"
#include "hardware/chery_canfd.h"

#include <cstddef>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <thread>

#define CMD_STEER_ACTIVE 0b01
#define CMD_GAS_ACTIVE 0b10
#define CMD_GAS_FULL_STOP 0b100
#define CMD_GAS_ACCEL_ON 0b1000
#define CMD_ACC_BTN_PRESS 0b10000

#define FLAG_OVERRIDE_STEER 0b01
#define FLAG_OVERRIDE_GAS 0b10
#define FLAG_OVERRIDE_BRAKE 0b100

#define FSM_GLOBAL_INIT 0
#define FSM_GLOBAL_PREOP 1
#define FSM_GLOBAL_SAFEOP 2
#define FSM_GLOBAL_OP_3 3
#define FSM_GLOBAL_OP_4 4
#define FSM_GLOBAL_OP_5 5
#define FSM_GLOBAL_OP_2 6
#define FSM_GLOBAL_RECORD_ROUTE 7
#define FSM_GLOBAL_MAPPING 8

#define FSM_LOCAL_PRE_FOLLOW_LANE 0
#define FSM_LOCAL_FOLLOW_LANE 1
#define FSM_LOCAL_MENUNGGU_STATION_1 2
#define FSM_LOCAL_MENUNGGU_STATION_2 3
#define FSM_LOCAL_MENUNGGU_STOP 4

class CANBUS_HAL_node : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_fb_steering_angle;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_fb_current_velocity;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_throttle_position;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_brake_position;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_gear_status;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_steer_torque_driver;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_flag_override_ctrl;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_target_steering_angle;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_target_velocity;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_global_fsm;

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

    uint64_t can1_internal_tick = 0;
    uint8_t is_mobil_initialized = 0;

    uint8_t flag_reset = 0;
    uint8_t counter_berapa_kali_kirim = 0;
    uint8_t acc_button_press_count = 0;
    uint64_t acc_button_last_press_tick = 0;
    bool acc_button_waiting_for_interval = false;
    static constexpr uint64_t ACC_BUTTON_INTERVAL_TICKS = 50;

    int16_t global_fsm_value = 0;

    uint8_t flag_override_status = 0;

    // variable untuk steering override detection
    int steering_pressed_counter = 0;
    int steering_unpressed_counter = 0;
    bool steerDisableTemp = false;

    // Constants
    static constexpr float DT_CTRL = 0.02; // 50Hz = 20ms
    float STEER_TORQUE_THRESHOLD = 50.0;   // Nm
    float STEER_ANGLE_MAX = 300.0;         // degrees

    // Constants untuk steering
    static constexpr float STEER_ANGLE_SCALE = 10.0;
    static constexpr float STEER_ANGLE_OFFSET = -392.0;
    static constexpr int STEER_ANGLE_MIN_ACTIVE = 2;

    // Constants untuk longitudinal control
    static constexpr int GAS_INACTIVE_VALUE = -24;

    std::vector<uint16_t> intercepted_id;

    bool intercept_steer = false;
    bool intercept_steer_btn = false;
    bool intercept_gas = false;
    bool intercept_lkas_state = false;
    bool intercept_cc_speed = false;

    std::vector<double> pid_terms;

    PID pid_vx;

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

        this->declare_parameter("steer_torque_threshold", STEER_TORQUE_THRESHOLD);
        this->get_parameter("steer_torque_threshold", STEER_TORQUE_THRESHOLD);

        this->declare_parameter("steer_angle_max", STEER_ANGLE_MAX);
        this->get_parameter("steer_angle_max", STEER_ANGLE_MAX);

        std::vector<int64_t> temp_intercepted_ids;
        this->declare_parameter<std::vector<int64_t>>("intercepted_ids_can", temp_intercepted_ids);
        this->get_parameter("intercepted_ids_can", temp_intercepted_ids);

        this->declare_parameter<std::vector<double>>("pid_terms", {0.0070, 0.000000, 0, 0.02, -0.04, 0.4, -0.0005, 0.0005});
        this->get_parameter("pid_terms", pid_terms);

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

        pid_vx.init(pid_terms[0], pid_terms[1], pid_terms[2], pid_terms[3], pid_terms[4], pid_terms[5], pid_terms[6], pid_terms[7]);

        time_now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        last_time_publish = time_now;

        pub_fb_steering_angle = this->create_publisher<std_msgs::msg::Float32>("fb_steering_angle", 1);
        pub_fb_current_velocity = this->create_publisher<std_msgs::msg::Float32>("fb_current_velocity", 1);
        pub_throttle_position = this->create_publisher<std_msgs::msg::Float32>("fb_throttle_position", 1);
        pub_brake_position = this->create_publisher<std_msgs::msg::Int16>("fb_brake_position", 1);
        pub_gear_status = this->create_publisher<std_msgs::msg::UInt8>("fb_gear_status", 1);
        pub_steer_torque_driver = this->create_publisher<std_msgs::msg::Float32>("fb_steer_torque_driver", 1);
        pub_flag_override_ctrl = this->create_publisher<std_msgs::msg::UInt8>("fb_flag_override_ctrl", 1);

        sub_target_steering_angle = this->create_subscription<std_msgs::msg::Float32>(
            "cmd_target_steering_angle", 1, std::bind(&CANBUS_HAL_node::callback_sub_target_steering_angle, this, std::placeholders::_1));
        sub_target_velocity = this->create_subscription<std_msgs::msg::Float32>(
            "cmd_target_velocity", 1, std::bind(&CANBUS_HAL_node::callback_sub_target_velocity, this, std::placeholders::_1));
        // sub_hw_flag = this->create_subscription<std_msgs::msg::UInt8>(
        //     "cmd_hw_flag", 1, std::bind(&CANBUS_HAL_node::callback_sub_hw_flag, this, std::placeholders::_1));
        sub_global_fsm = this->create_subscription<std_msgs::msg::Int16>(
            "global_fsm", 1, std::bind(&CANBUS_HAL_node::callback_sub_global_fsm, this, std::placeholders::_1));

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
        if (canbus2_hal)
        {
            global_fsm_value = FSM_GLOBAL_SAFEOP;
            canbus2_hal->cmd_hw_flag &= ~(CMD_STEER_ACTIVE | CMD_GAS_ACTIVE);
            canbus2_hal->cmd_target_steering_angle = 0.0;
            canbus2_hal->cmd_target_velocity = 0.0;
            reset_acc_button_state();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            canbus2_hal->shutdown();
        }

        if (thread_routine_all.joinable())
        {
            thread_routine_all.join();
        }

        if (canbus1_hal)
        {
            canbus1_hal->shutdown();
        }
    }

    void callback_sub_target_steering_angle(const std_msgs::msg::Float32::SharedPtr msg)
    {
        canbus2_hal->cmd_target_steering_angle = msg->data;
    }

    void callback_sub_target_velocity(const std_msgs::msg::Float32::SharedPtr msg)
    {
        canbus2_hal->cmd_target_velocity = msg->data;
    }

    void callback_sub_global_fsm(const std_msgs::msg::Int16::SharedPtr msg)
    {
        int16_t prev_fsm = global_fsm_value;
        global_fsm_value = msg->data;

        switch (global_fsm_value)
        {
        case FSM_GLOBAL_INIT:
            canbus2_hal->cmd_hw_flag = 0;
            reset_acc_button_state();
            break;

        case FSM_GLOBAL_PREOP:
        case FSM_GLOBAL_SAFEOP:
            canbus2_hal->cmd_hw_flag &= ~(CMD_STEER_ACTIVE | CMD_GAS_ACTIVE);
            reset_acc_button_state();
            break;

        case FSM_GLOBAL_OP_3: // auto acc & steer
            canbus2_hal->cmd_hw_flag |= (CMD_STEER_ACTIVE | CMD_GAS_ACTIVE);
            if (prev_fsm != FSM_GLOBAL_OP_3)
            {
                trigger_acc_button_sequence();
            }
            break;

        case FSM_GLOBAL_OP_4: // auto acc only
            canbus2_hal->cmd_hw_flag &= ~CMD_STEER_ACTIVE;
            canbus2_hal->cmd_hw_flag |= CMD_GAS_ACTIVE;
            if (prev_fsm != FSM_GLOBAL_OP_4)
            {
                trigger_acc_button_sequence();
            }
            break;

        case FSM_GLOBAL_OP_5: // auto steer only
            canbus2_hal->cmd_hw_flag |= CMD_STEER_ACTIVE;
            canbus2_hal->cmd_hw_flag &= ~CMD_GAS_ACTIVE;
            reset_acc_button_state();
            break;
        }
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

    void send_steer_button_cmd(std::unique_ptr<CANBUS_HAL> &canbus_hal_from_adas, std::unique_ptr<CANBUS_HAL> &canbus_hal_to_send)
    {
        // Copy dari bus mobil ke bus afas
        memcpy(&msg_steer_button_cmd, &canbus_hal_to_send->steer_button, sizeof(msg_steer_button_cmd));

        // ACC button press logic
        if (canbus2_hal->cmd_hw_flag & CMD_ACC_BTN_PRESS)
        {
            if (flag_reset == 0)
            {
                msg_steer_button_cmd.acc = 1;
                counter_berapa_kali_kirim++;
                if (counter_berapa_kali_kirim >= 4)
                {
                    flag_reset = 1;
                    acc_button_last_press_tick = can1_internal_tick;
                }
            }
            else
            {
                msg_steer_button_cmd.acc = 0;
                counter_berapa_kali_kirim++;
                if (counter_berapa_kali_kirim >= 8)
                {
                    canbus2_hal->cmd_hw_flag &= ~CMD_ACC_BTN_PRESS;
                    if (acc_button_press_count < 2)
                    {
                        acc_button_waiting_for_interval = true;
                    }
                    else
                    {
                        reset_acc_button_state();
                    }
                }
            }
        }
        else
        {
            msg_steer_button_cmd.acc = 0;
        }

        // Packing can
        can_frame_t frame_steer_button_cmd;
        bzero(&frame_steer_button_cmd, sizeof(frame_steer_button_cmd));
        chery_canfd_steer_button_pack(frame_steer_button_cmd.data, &msg_steer_button_cmd, sizeof(frame_steer_button_cmd.data));

        // Menimpa CRC lalu packing lagi
        // crc_str_btn(&frame_steer_button_cmd.data[1], CHERY_CANFD_STEER_BUTTON_LENGTH - 1);
        uint8_t crc_ = crc_str_btn(&frame_steer_button_cmd.data[1], CHERY_CANFD_STEER_BUTTON_LENGTH - 1);
        msg_steer_button_cmd.checksum = crc_;
        bzero(&frame_steer_button_cmd, sizeof(frame_steer_button_cmd));
        chery_canfd_steer_button_pack(frame_steer_button_cmd.data, &msg_steer_button_cmd, sizeof(frame_steer_button_cmd.data));

        // Mengirim ke bus adas
        frame_steer_button_cmd.id = CHERY_CANFD_STEER_BUTTON_FRAME_ID;
        frame_steer_button_cmd.dlc = CHERY_CANFD_STEER_BUTTON_LENGTH;

        canbus_hal_from_adas->send_msg(&frame_steer_button_cmd);
    }

    void send_steer_cmd(std::unique_ptr<CANBUS_HAL> &canbus_hal_from_adas, std::unique_ptr<CANBUS_HAL> &canbus_hal_to_send)
    {
        // Copy dari bus adas ke bus mobil
        memcpy(&msg_steer_cmd, &canbus_hal_from_adas->lkas_cam_cmd, sizeof(msg_steer_cmd));

        // Get current steering feedback
        float current_steering_torque = canbus1_hal->fb_steering_torq_drv;

        if (fabs(current_steering_torque) >= STEER_TORQUE_THRESHOLD)
        {
            steering_pressed_counter++;
            steering_unpressed_counter = 0;
        }
        else
        {
            steering_pressed_counter = 0;
            steering_unpressed_counter++;
        }

        if (steering_pressed_counter * DT_CTRL > 1.0)
        { // > 1 second
            steerDisableTemp = true;

            if (global_fsm_value == FSM_GLOBAL_OP_3 || global_fsm_value == FSM_GLOBAL_OP_5)
            {
                flag_override_status |= FLAG_OVERRIDE_STEER;
                logger.warn("LKA temporary disabled - driver override! Torque: %.2f", current_steering_torque);
            }
        }
        else
        {
            if (steering_unpressed_counter * DT_CTRL > 1.0)
            { // > 1 second tanpa override
                steerDisableTemp = false;
                if (global_fsm_value == FSM_GLOBAL_OP_3 || global_fsm_value == FSM_GLOBAL_OP_5)
                    flag_override_status &= ~FLAG_OVERRIDE_STEER;
            }
        }
        float apply_steer = canbus2_hal->cmd_target_steering_angle * 180.0 / M_PI; // Convert to degrees

        if (std::isnan(apply_steer) || std::isinf(apply_steer))
        {
            apply_steer = canbus1_hal->fb_steering_angle * 180.0 / M_PI;
            steerDisableTemp = true;
        }

        if (fabs(apply_steer) > STEER_ANGLE_MAX)
        {
            apply_steer = std::clamp(apply_steer, -STEER_ANGLE_MAX, STEER_ANGLE_MAX);
        }

        // steerDisableTemp = false;

        bool lkas_enable = ((canbus2_hal->cmd_hw_flag & CMD_STEER_ACTIVE) != 0) && !steerDisableTemp;

        if (lkas_enable)
        {
            float raw_steer = (apply_steer * STEER_ANGLE_SCALE) + STEER_ANGLE_OFFSET;
            if (raw_steer >= 0 && raw_steer <= STEER_ANGLE_MIN_ACTIVE)
                raw_steer = STEER_ANGLE_MIN_ACTIVE;
            msg_steer_cmd.cmd = chery_canfd_lkas_cam_cmd_345_cmd_encode(raw_steer);

            msg_steer_cmd.new_signal_3 = (raw_steer > 1.0) ? 1 : 0;

            msg_steer_cmd.lka_active = 1;
        }
        else
        {
            msg_steer_cmd.lka_active = 0;
        }

        // logger.info("Steering Command - Target: %.2f deg, Applied: %.2f deg, LKA Enable: %d",
        //             apply_steer, chery_canfd_lkas_cam_cmd_345_cmd_decode(msg_steer_cmd.cmd), lkas_enable ? 1 : 0);

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

    void send_gas_cmd(std::unique_ptr<CANBUS_HAL> &canbus_hal_from_adas, std::unique_ptr<CANBUS_HAL> &canbus_hal_to_send)
    {
        // Hitung target acceleration menggunakan PID
        float target_acc = pid_vx.calculate(canbus2_hal->cmd_target_velocity - canbus1_hal->fb_current_velocity);

        if (target_acc > MAX_ACCEL)
            target_acc = MAX_ACCEL;
        if (target_acc < MIN_ACCEL)
            target_acc = MIN_ACCEL;

        int gas;
        if (target_acc > 0)
        {
            float norm_acc = target_acc / MAX_ACCEL;
            gas = (int)(norm_acc * MAX_GAS);
        }
        else if (target_acc < 0)
        {
            float norm_acc = target_acc / MIN_ACCEL;
            gas = (int)(norm_acc * MIN_GAS);
        }
        else
        {
            gas = GAS_INACTIVE_VALUE;
        }

        if (gas < MIN_GAS || gas > MAX_GAS)
            gas = std::clamp(gas, (int)MIN_GAS, (int)MAX_GAS);

        // Copy dari bus adas ke bus mobil
        memcpy(&msg_acc_cmd, &canbus_hal_from_adas->acc_cam_cmd, sizeof(msg_acc_cmd));

        bool long_active = (canbus2_hal->cmd_hw_flag & CMD_GAS_ACTIVE) != 0;
        // bool standstill = canbus1_hal->fb_current_velocity < 1e-3;
        bool full_stop = long_active && (canbus2_hal->cmd_target_velocity < 0);
        bool resume = (canbus2_hal->cmd_hw_flag & CMD_ACC_BTN_PRESS) != 0;

        if (gas > 0 && resume)
            full_stop = false;
        // Set thrott global_fsm_value = FSM_GLOBAL_SAFEOP;le value
        int throttle = long_active ? gas : GAS_INACTIVE_VALUE;
        // Set ACC_STATE
        uint8_t acc_state;
        if (full_stop)
            acc_state = 2; // available
        else if (long_active)
            acc_state = 3; // active
        else
            acc_state = canbus_hal_from_adas->acc_cam_cmd.acc_state;

        // CMD: 400 for full_stop, otherwise throttle
        msg_acc_cmd.cmd = full_stop ? 400 : throttle;
        msg_acc_cmd.accel_on = (throttle >= 0) ? 1 : 0;
        msg_acc_cmd.acc_state = acc_state;
        if (full_stop)
            msg_acc_cmd.stopped = 1;
        else if (long_active)
            msg_acc_cmd.stopped = 0;

        msg_acc_cmd.gas_pressed = resume ? 1 : 0;

        if (resume)
            flag_override_status |= FLAG_OVERRIDE_GAS;
        else
            flag_override_status &= ~FLAG_OVERRIDE_GAS;

        msg_acc_cmd.counter = (uint8_t)(can1_internal_tick % 0x0F);

        // log all
        logger.info("throttle: %d, target_acc: %.2f, cmd_target_velocity: %.2f, fb_current_velocity: %.2f, long_active: %d, full_stop: %d, resume: %d, acc_state: %d",
                    throttle, target_acc, canbus2_hal->cmd_target_velocity, canbus1_hal->fb_current_velocity,
                    long_active ? 1 : 0, full_stop ? 1 : 0, resume ? 1 : 0, acc_state);

        // === CAN PACKING & SENDING ===
        can_frame_t frame_acc_cmd;
        bzero(&frame_acc_cmd, sizeof(frame_acc_cmd));
        chery_canfd_acc_cmd_pack(frame_acc_cmd.data, &msg_acc_cmd, sizeof(frame_acc_cmd.data));

        // Calculate and set CRC (mirip Python)
        uint8_t crc = calculate_crc(frame_acc_cmd.data, CHERY_CANFD_ACC_CMD_LENGTH - 1, 0x1D, 0xA);
        msg_acc_cmd.checksum = crc;

        // Re-pack with correct CRC
        bzero(&frame_acc_cmd, sizeof(frame_acc_cmd));
        chery_canfd_acc_cmd_pack(frame_acc_cmd.data, &msg_acc_cmd, sizeof(frame_acc_cmd.data));

        // Send to car
        frame_acc_cmd.id = CHERY_CANFD_ACC_CMD_FRAME_ID;
        frame_acc_cmd.dlc = CHERY_CANFD_ACC_CMD_LENGTH;
        canbus_hal_to_send->send_msg(&frame_acc_cmd);
    }

    void send_lkas_state_cmd(std::unique_ptr<CANBUS_HAL> &canbus_hal_from_adas, std::unique_ptr<CANBUS_HAL> &canbus_hal_to_send) // 20 Hz
    {
        // Copy dari bus adas ke bus mobil
        memcpy(&msg_lkas_state_cmd, &canbus_hal_from_adas->lkas_state, sizeof(msg_lkas_state_cmd));

        // Mengisi sesuai target
        msg_lkas_state_cmd.new_signal_1 = 1;
        msg_lkas_state_cmd.new_signal_2 = ((canbus2_hal->cmd_hw_flag & CMD_STEER_ACTIVE) >> 0x00) ? 2 : 0;
        msg_lkas_state_cmd.new_signal_3 = ((canbus2_hal->cmd_hw_flag & CMD_STEER_ACTIVE) >> 0x00) ? 2 : 0;
        msg_lkas_state_cmd.new_signal_4 = 1;
        msg_lkas_state_cmd.state = ((canbus2_hal->cmd_hw_flag & CMD_STEER_ACTIVE) >> 0x00) ? 0 : 1;
        msg_lkas_state_cmd.lka_active = ((canbus2_hal->cmd_hw_flag & CMD_STEER_ACTIVE) >> 0x00) ? 1 : 0;
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

    void send_cc_speed_cmd(std::unique_ptr<CANBUS_HAL> &canbus_hal_from_adas, std::unique_ptr<CANBUS_HAL> &canbus_hal_to_send)
    {
        // Copy dari bus adas ke bus mobil
        memcpy(&msg_setting_dari_adas_cmd, &canbus_hal_from_adas->setting_cmd_from_adas, sizeof(msg_setting_dari_adas_cmd));

        // Mengisi sesuai target
        msg_setting_dari_adas_cmd.cc_speed = chery_canfd_setting_cc_speed_encode((uint16_t)(canbus2_hal->cmd_target_velocity * 3.6f)); // m/s to km/h

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

        std_msgs::msg::Float32 msg_steer_torque_driver;
        msg_steer_torque_driver.data = canbus1_hal->fb_steering_torq_drv;
        pub_steer_torque_driver->publish(msg_steer_torque_driver);

        std_msgs::msg::UInt8 msg_flag_override_ctrl;
        msg_flag_override_ctrl.data = flag_override_status;
        pub_flag_override_ctrl->publish(msg_flag_override_ctrl);
    }

    void callback_routine_all()
    {
        while (rclcpp::ok())
        {
            callback_routine_all_routine();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void reset_acc_button_state()
    {
        flag_reset = 1;
        counter_berapa_kali_kirim = 0;
        acc_button_press_count = 0;
        acc_button_last_press_tick = 0;
        acc_button_waiting_for_interval = false;
        canbus2_hal->cmd_hw_flag &= ~CMD_ACC_BTN_PRESS;
    }

    void trigger_acc_button_sequence()
    {
        acc_button_press_count = 0;
        acc_button_last_press_tick = 0;
        acc_button_waiting_for_interval = false;
        start_acc_button_press();
    }

    void start_acc_button_press()
    {
        canbus2_hal->cmd_hw_flag |= CMD_ACC_BTN_PRESS;
        flag_reset = 0;
        counter_berapa_kali_kirim = 0;
        acc_button_press_count++;
    }

    void update_acc_button_sequence()
    {
        if (acc_button_waiting_for_interval)
        {
            uint64_t ticks_since_last_press = can1_internal_tick - acc_button_last_press_tick;
            if (ticks_since_last_press >= ACC_BUTTON_INTERVAL_TICKS)
            {
                acc_button_waiting_for_interval = false;
                if (acc_button_press_count < 2)
                {
                    start_acc_button_press();
                }
            }
        }
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