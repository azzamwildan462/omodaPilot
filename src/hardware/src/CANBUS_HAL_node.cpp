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
#include "hardware/chery_canfd.h"

#include <thread>

class CANBUS_HAL_node : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_fb_steering_angle;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_fb_current_velocity;

    std::thread thread_routine;

    // Configs
    int can_type = 0; // 0: CANable2_SLCAN, 1: CANable2_SOCKET_CAN
    std::string device_name = "can0";
    int baudrate = 500000;
    int fd_baudrate = 2000000;
    int fd;
    int publish_period_ms = 20;

    HelpLogger logger;
    std::unique_ptr<CANBUS_HAL> canbus_hal; // Pointer to base class
    time_t time_now;
    time_t last_time_publish;

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

        this->declare_parameter("publish_period_ms", publish_period_ms);
        this->get_parameter("publish_period_ms", publish_period_ms);

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

        time_now = rclcpp::Clock().now().seconds();
        last_time_publish = time_now;

        pub_fb_steering_angle = this->create_publisher<std_msgs::msg::Float32>("fb_steering_angle", 1);
        pub_fb_current_velocity = this->create_publisher<std_msgs::msg::Float32>("fb_current_velocity", 1);

        // tim_routine = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&CANBUS_HAL_node::callback_routine, this));
        thread_routine = std::thread(std::bind(&CANBUS_HAL_node::callback_routine_multi_thread, this), this);

        logger.info("CANBUS_HAL_node node initialized");
    }

    ~CANBUS_HAL_node()
    {
    }

    void callback_routine_multi_thread()
    {
        while (rclcpp::ok())
        {
            callback_routine();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void callback_routine()
    {
        time_now = rclcpp::Clock().now().seconds();

        canbus_hal->recv_msgs();
        canbus_hal->update();

        // Memastikan publish sesuai dengan periode yang diinginkan
        time_t dt_publish = time_now - last_time_publish;
        if (dt_publish * 1000 < publish_period_ms)
            return;
        last_time_publish = time_now;

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