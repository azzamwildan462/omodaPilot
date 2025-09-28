/**
 * Menggunakan factory pattern untuk memilih hardware CANBUS_HAL yang sesuai
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int16.hpp"
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

class CANBUS_HAL_node : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_fb_steering_angle;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_fb_current_velocity;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_throttle_position;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_brake_position;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_gear_status;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_target_steering_angle;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_target_velocity;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_hw_flag;

    std::thread thread_can1_routine;
    std::thread thread_can2_routine;

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
    float MAX_GAS = 51.0;
    float MIN_GAS = -51.0;

    HelpLogger logger;
    std::unique_ptr<CANBUS_HAL> canbus1_hal; // Pointer to base class
    std::unique_ptr<CANBUS_HAL> canbus2_hal; // Pointer to base class
    rclcpp::Time time_now;
    rclcpp::Time last_time_publish;

    // Data kirim untuk can bus
    chery_canfd_lkas_cam_cmd_345_t msg_steer_cmd;
    chery_canfd_acc_cmd_t msg_acc_cmd;

    float cmd_target_steering_angle = 0;
    float cmd_target_velocity = 0;
    uint8_t cmd_hw_flag = 0;

    uint64_t can1_internal_tick = 0;

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
        bzero(&msg_acc_cmd, sizeof(msg_acc_cmd));

        time_now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        last_time_publish = time_now;

        pub_fb_steering_angle = this->create_publisher<std_msgs::msg::Float32>("fb_steering_angle", 1);
        pub_fb_current_velocity = this->create_publisher<std_msgs::msg::Float32>("fb_current_velocity", 1);
        pub_throttle_position = this->create_publisher<std_msgs::msg::Float32>("fb_throttle_position", 1);
        pub_brake_position = this->create_publisher<std_msgs::msg::Int16>("fb_brake_position", 1);
        pub_gear_status = this->create_publisher<std_msgs::msg::UInt8>("fb_gear_status", 1);

        sub_target_steering_angle = this->create_subscription<std_msgs::msg::Float32>(
            "cmd_target_steering_angle", 1, std::bind(&CANBUS_HAL_node::callback_sub_target_steering_angle, this, std::placeholders::_1));
        sub_target_velocity = this->create_subscription<std_msgs::msg::Float32>(
            "cmd_target_velocity", 1, std::bind(&CANBUS_HAL_node::callback_sub_target_velocity, this, std::placeholders::_1));
        sub_hw_flag = this->create_subscription<std_msgs::msg::UInt8>(
            "cmd_hw_flag", 1, std::bind(&CANBUS_HAL_node::callback_sub_hw_flag, this, std::placeholders::_1));

        if (device1_name != "")
            thread_can1_routine = std::thread(std::bind(&CANBUS_HAL_node::callback_can1, this), this);

        if (device2_name != "")
            thread_can2_routine = std::thread(std::bind(&CANBUS_HAL_node::callback_can2, this), this);

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
        cmd_hw_flag = msg->data;
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
                crc &= 0xFF;
            }
        }
        return (crc ^ xor_output);
    }

    // =========================================================================================

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

    void send_gas_cmd(std::unique_ptr<CANBUS_HAL> &canbus_hal_from_adas, std::unique_ptr<CANBUS_HAL> &canbus_hal_to_send)
    {
        // Clipping
        if (cmd_target_velocity > MAX_ACCEL)
            cmd_target_velocity = MAX_ACCEL;
        if (cmd_target_velocity < MIN_ACCEL)
            cmd_target_velocity = MIN_ACCEL;

        // Normalisasi
        float target_gas = 0;
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
        if ((cmd_hw_flag & CMD_GAS_FULL_STOP) == CMD_GAS_FULL_STOP)
            acc_state = 2;
        else if ((cmd_hw_flag & CMD_GAS_ACTIVE) == CMD_GAS_ACTIVE)
            acc_state = 3;

        msg_acc_cmd.cmd = throttle;
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

        logger.info("Target gas: %.2f, Throttle: %d, Acc state: %d, Stopped: %d, Gas pressed: %d %d",
                    target_gas, msg_acc_cmd.cmd, msg_acc_cmd.acc_state, msg_acc_cmd.stopped, msg_acc_cmd.gas_pressed, msg_acc_cmd.accel_on);

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

    //

    void callback_can2()
    {
        while (rclcpp::ok())
        {
            callback_can2_routine();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    void callback_can1()
    {
        while (rclcpp::ok())
        {
            callback_can1_routine();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    void callback_can2_routine()
    {
        canbus2_hal->recv_msgs();
        canbus2_hal->update();
    }

    void callback_can1_routine()
    {
        time_now = rclcpp::Clock(RCL_SYSTEM_TIME).now();

        logger.info("%.2f %.2f %d", cmd_target_steering_angle, cmd_target_velocity, cmd_hw_flag);

        // canbus1_hal->recv_msgs();
        // canbus1_hal->update();
        send_steer_cmd(canbus2_hal, canbus1_hal);
        send_gas_cmd(canbus2_hal, canbus1_hal);

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