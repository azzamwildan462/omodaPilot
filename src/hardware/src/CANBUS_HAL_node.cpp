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

class CANBUS_HAL_node : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_fb_steering_angle;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_fb_current_velocity;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_throttle_position;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_brake_position;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_gear_status;

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

    HelpLogger logger;
    std::unique_ptr<CANBUS_HAL> canbus1_hal; // Pointer to base class
    std::unique_ptr<CANBUS_HAL> canbus2_hal; // Pointer to base class
    rclcpp::Time time_now;
    rclcpp::Time last_time_publish;

    // Data kirim untuk can bus
    chery_canfd_lkas_cam_cmd_345_t msg_steer_cmd;

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

        time_now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        last_time_publish = time_now;

        pub_fb_steering_angle = this->create_publisher<std_msgs::msg::Float32>("fb_steering_angle", 1);
        pub_fb_current_velocity = this->create_publisher<std_msgs::msg::Float32>("fb_current_velocity", 1);
        pub_throttle_position = this->create_publisher<std_msgs::msg::Float32>("fb_throttle_position", 1);
        pub_brake_position = this->create_publisher<std_msgs::msg::Int16>("fb_brake_position", 1);
        pub_gear_status = this->create_publisher<std_msgs::msg::UInt8>("fb_gear_status", 1);

        if (device1_name != "")
            thread_can1_routine = std::thread(std::bind(&CANBUS_HAL_node::callback_can1, this), this);

        if (device2_name != "")
            thread_can2_routine = std::thread(std::bind(&CANBUS_HAL_node::callback_can2, this), this);

        logger.info("CANBUS_HAL_node node initialized");
    }

    ~CANBUS_HAL_node()
    {
    }

    uint8_t calculate_crc(const uint8_t *data, std::size_t len,
                          uint8_t poly, uint8_t xor_out)
    {
        uint8_t crc = 0x00;
        for (std::size_t idx = 0; idx < len - 1; ++idx)
        {
            crc ^= data[idx];
            for (int i = 0; i < 8; ++i)
            {
                if (crc & 0x80)
                {
                    crc = static_cast<uint8_t>((crc << 1) ^ poly);
                }
                else
                {
                    crc = static_cast<uint8_t>(crc << 1);
                }
                crc &= 0xFF; // mirrors the Python masking
            }
        }
        return static_cast<uint8_t>(crc ^ xor_out);
    }

    // =========================================================================================

    void send_steer_cmd(std::unique_ptr<CANBUS_HAL> &canbus_hal_from_adas, std::unique_ptr<CANBUS_HAL> &canbus_hal_to_send)
    {
        float target_steering_angel = 0.5;

        // Copy dari bus adas ke bus mobil
        memcpy(&canbus_hal_from_adas->lkas_cam_cmd, &msg_steer_cmd, sizeof(msg_steer_cmd));

        msg_steer_cmd.cmd = chery_canfd_lkas_cam_cmd_345_cmd_encode(target_steering_angel * 180 / 3.141592653589793);
        msg_steer_cmd.lka_active = 1;
        msg_steer_cmd.set_x0 = 0;

        can_frame_t frame_steer_cmd;
        bzero(&frame_steer_cmd, sizeof(frame_steer_cmd));
        chery_canfd_lkas_cam_cmd_345_pack(frame_steer_cmd.data, &msg_steer_cmd, sizeof(frame_steer_cmd.data));

        uint8_t crc = calculate_crc(frame_steer_cmd.data, sizeof(frame_steer_cmd.data) - 1, 0x1D, 0xA);
        msg_steer_cmd.checksum = crc;
        bzero(&frame_steer_cmd, sizeof(frame_steer_cmd));
        chery_canfd_lkas_cam_cmd_345_pack(frame_steer_cmd.data, &msg_steer_cmd, sizeof(frame_steer_cmd.data));

        frame_steer_cmd.id = CHERY_CANFD_LKAS_CAM_CMD_345_FRAME_ID;
        frame_steer_cmd.dlc = CHERY_CANFD_LKAS_CAM_CMD_345_LENGTH;
        canbus_hal_to_send->send_msg(&frame_steer_cmd);
    }

    void send_steer_cmd_hardcode(std::unique_ptr<CANBUS_HAL> &canbus_hal_to_send)
    {
        static uint8_t status_cmd_naik = 0;
        static float cmd_steer_angle = 0.0;
        static uint64_t counter = 0;
        static const float cmd_steer_angle_max = 1.57; // rad
        static const float cmd_steer_angle_min = -1.57;
        static const float cmd_steer_angle_step = 0.02;

        counter++;
        cmd_steer_angle += status_cmd_naik ? cmd_steer_angle_step : -cmd_steer_angle_step;
        if (cmd_steer_angle >= cmd_steer_angle_max)
            status_cmd_naik = 0;
        else if (cmd_steer_angle <= cmd_steer_angle_min)
            status_cmd_naik = 1;

        msg_steer_cmd.cmd = chery_canfd_lkas_cam_cmd_345_cmd_encode(cmd_steer_angle * 180.0 / 3.141592653589793); // rad to deg
        msg_steer_cmd.lka_active = 1;
        msg_steer_cmd.set_x0 = 0;

        can_frame_t frame_steer_cmd;
        bzero(&frame_steer_cmd, sizeof(frame_steer_cmd));
        chery_canfd_lkas_cam_cmd_345_pack(frame_steer_cmd.data, &msg_steer_cmd, sizeof(frame_steer_cmd.data));
        frame_steer_cmd.id = CHERY_CANFD_LKAS_CAM_CMD_345_FRAME_ID;
        frame_steer_cmd.dlc = CHERY_CANFD_LKAS_CAM_CMD_345_LENGTH;
        canbus_hal_to_send->send_msg(&frame_steer_cmd);
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
        // send_steer_cmd_hardcode(canbus2_hal);
    }

    void callback_can1_routine()
    {
        time_now = rclcpp::Clock(RCL_SYSTEM_TIME).now();

        // canbus1_hal->recv_msgs();
        // canbus1_hal->update();
        // send_steer_cmd_hardcode(canbus1_hal);
        send_steer_cmd(canbus2_hal, canbus1_hal);

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