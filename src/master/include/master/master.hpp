#ifndef MASTER_HPP
#define MASTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "ros2_utils/help_logger.hpp"

#define CMD_STEER_ACTIVE 0b01
#define CMD_GAS_ACTIVE 0b10
#define CMD_GAS_FULL_STOP 0b100
#define CMD_GAS_ACCEL_ON 0b1000

class Master : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_target_steering_angle;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_target_velocity;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_hw_flag;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_key_pressed;

    HelpLogger logger;

    int16_t current_key_pressed = 0;
    int16_t prev_key_pressed = 0;

    float cmd_target_steering_angle = 0;
    float cmd_target_velocity = 0;
    uint8_t cmd_hw_flag = 0;

    Master();
    ~Master();

    void callback_routine();
    void callback_sub_key_pressed(const std_msgs::msg::Int16::SharedPtr msg);

    void process_transmitter();
};

#endif // MASTER_HPP