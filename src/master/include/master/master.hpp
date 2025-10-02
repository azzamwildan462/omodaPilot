#ifndef MASTER_HPP
#define MASTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int8.hpp>
#include "ros2_utils/help_logger.hpp"

#define CMD_STEER_ACTIVE 0b01
#define CMD_GAS_ACTIVE 0b10
#define CMD_GAS_FULL_STOP 0b100
#define CMD_GAS_ACCEL_ON 0b1000
#define CMD_ACC_BTN_PRESS 0b10000

class Master : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_target_steering_angle;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_target_velocity;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_hw_flag;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_key_pressed;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_fb_steering_angle;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_fb_current_velocity;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_throttle_position;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_brake_position;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_gear_status;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_steer_torque;

    HelpLogger logger;

    float MAX_STEERING_ANGLE = 5.0;
    float MIN_STEERING_ANGLE = -5.0;

    int16_t current_key_pressed = 0;
    int16_t prev_key_pressed = 0;

    float cmd_target_steering_angle = 0;
    float cmd_target_velocity = 0;
    uint8_t cmd_hw_flag = 0;

    float fb_steering_angle = 0;
    float fb_current_velocity = 0;
    float throttle_position = 0;
    int16_t brake_position = 0;
    uint8_t gear_status = 0;
    int8_t steer_torque = 0;

    rclcpp::Time last_time_steer_button_press;
    rclcpp::Time time_now;

    Master();
    ~Master();

    void callback_routine();
    void callback_sub_fb_steering_angle(const std_msgs::msg::Float32::SharedPtr msg);
    void callback_sub_fb_current_velocity(const std_msgs::msg::Float32::SharedPtr msg);
    void callback_sub_throttle_position(const std_msgs::msg::Float32::SharedPtr msg);
    void callback_sub_brake_position(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_gear_status(const std_msgs::msg::UInt8::SharedPtr msg);
    void callback_sub_steer_torque(const std_msgs::msg::Int8::SharedPtr msg);
    void callback_sub_key_pressed(const std_msgs::msg::Int16::SharedPtr msg);

    void process_transmitter();
};

#endif // MASTER_HPP