/**
 * Menggunakan factory pattern untuk memilih hardware CANBUS_HAL yang sesuai
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/global_definitions.hpp"

#include "hardware/CANBUS_HAL.h"
#include "hardware/canable2_slcan.h"
#include "hardware/canable2_socket_can.h"

class CANBUS_HAL_node : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_fb_steering_angle;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_fb_current_velocity;

    // Configs
    int can_type = 0; // 0: CANable2_SLCAN, 1: CANable2_SOCKET_CAN
    std::string device_name = "can0";
    int baudrate = 500000;
    int fd_baudrate = 2000000;
    int fd;

    HelpLogger logger;
    std::unique_ptr<CANBUS_HAL> canbus_hal; // Pointer to base class

    CANBUS_HAL_node()
        : Node("canbus_hal_node")
    {
        this->declare_parameter("can_type", can_type);
        this->get_parameter("can_type", can_type);

        this->declare_parameter("device_name", device_name);
        this->get_parameter("device_name", device_name);

        this->declare_parameter("baudrate", baudrate);
        this->get_parameter("baudrate", baudrate);

        this->declare_parameter("fd_baudrate", fd_baudrate);
        this->get_parameter("fd_baudrate", fd_baudrate);

        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        if (can_type == 0)
            canbus_hal = std::make_unique<CANable2_SLCAN>(&logger);
        else if (can_type == 1)
            canbus_hal = std::make_unique<CANable2_SOCKET_CAN>(&logger);

        if (canbus_hal->init(device_name, baudrate) != 0)
        {
            logger.error("Failed to initialize CANBUS_HAL");
            rclcpp::shutdown();
        }

        pub_fb_steering_angle = this->create_publisher<std_msgs::msg::Float32>("fb_steering_angle", 1);
        pub_fb_current_velocity = this->create_publisher<std_msgs::msg::Float32>("fb_current_velocity", 1);

        tim_routine = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&CANBUS_HAL_node::callback_routine, this));

        logger.info("CANBUS_HAL_node node initialized");
    }

    ~CANBUS_HAL_node()
    {
    }

    void callback_routine()
    {
        canbus_hal->recv_msgs();
        canbus_hal->update();

        std_msgs::msg::Float32 msg_steering_angle;
        msg_steering_angle.data = canbus_hal->fb_steering_angle;
        pub_fb_steering_angle->publish(msg_steering_angle);

        std_msgs::msg::Float32 msg_current_velocity;
        msg_current_velocity.data = canbus_hal->fb_current_velocity;
        pub_fb_current_velocity->publish(msg_current_velocity);
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