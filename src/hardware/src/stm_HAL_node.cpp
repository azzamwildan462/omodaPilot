#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/global_definitions.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/int8.hpp"

class STM_HAL_node : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_fb_steering_ackerman_angle; // rad
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_fb_current_velocity;        // m/s
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_throttle_position;          // 0 - 100 % hasil normalisasi
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_stm_remote;        // 0 - 100 % hasil normalisasi

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_cmd_steering_ackerman_angle; // rad
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_cmd_target_velocity;         // m/s
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_cmd_transmission;               // -1: R, 0: N, 1: D

    HelpLogger logger;

    // Configs
    float encoder_to_meter = 1; // Konversi dari nilai encoder ke meter

    // Vars

    STM_HAL_node()
        : Node("stm_HAL_node")
    {
        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        this->declare_parameter("encoder_to_meter", encoder_to_meter);
        this->get_parameter("encoder_to_meter", encoder_to_meter);

        pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("imu_raw", 1);
        pub_fb_steering_ackerman_angle = this->create_publisher<std_msgs::msg::Float32>("fb_steering_ackerman_angle", 1);
        pub_fb_current_velocity = this->create_publisher<std_msgs::msg::Float32>("fb_current_velocity", 1);
        pub_throttle_position = this->create_publisher<std_msgs::msg::Float32>("fb_throttle_position", 1);
        pub_stm_remote = this->create_publisher<std_msgs::msg::UInt16MultiArray>("fb_stm_remote", 1);

        sub_cmd_steering_ackerman_angle = this->create_subscription<std_msgs::msg::Float32>("cmd_steering_ackerman_angle", 1,
                                                                                            std::bind(&STM_HAL_node::sub_cmd_steering_ackerman_angle_callback, this, std::placeholders::_1));
        sub_cmd_target_velocity = this->create_subscription<std_msgs::msg::Float32>("cmd_target_velocity", 1,
                                                                                    std::bind(&STM_HAL_node::sub_cmd_target_velocity_callback, this, std::placeholders::_1));
        sub_cmd_transmission = this->create_subscription<std_msgs::msg::Int8>("cmd_transmission", 1,
                                                                              std::bind(&STM_HAL_node::sub_cmd_transmission_callback, this, std::placeholders::_1));

        tim_routine = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&STM_HAL_node::callback_routine, this));

        logger.info("STM_HAL_node node initialized");
    }

    ~STM_HAL_node()
    {
    }

    void sub_cmd_steering_ackerman_angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        (void)msg;
    }

    void sub_cmd_target_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        (void)msg;
    }

    void sub_cmd_transmission_callback(const std_msgs::msg::Int8::SharedPtr msg)
    {
        (void)msg;
    }

    void callback_routine()
    {
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_stm_HAL_node = std::make_shared<STM_HAL_node>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_stm_HAL_node);
    executor.spin();

    return 0;
}