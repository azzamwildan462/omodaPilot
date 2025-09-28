#include "master/master.hpp"

void Master::process_transmitter()
{
    std_msgs::msg::Float32 msg_steering_angle;
    msg_steering_angle.data = cmd_target_steering_angle;
    pub_target_steering_angle->publish(msg_steering_angle);

    std_msgs::msg::Float32 msg_target_velocity;
    msg_target_velocity.data = cmd_target_velocity;
    pub_target_velocity->publish(msg_target_velocity);

    std_msgs::msg::UInt8 msg_hw_flag;
    msg_hw_flag.data = cmd_hw_flag;
    pub_hw_flag->publish(msg_hw_flag);
}