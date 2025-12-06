#include "master/master.hpp"

void Master::transmit_all()
{
    std_msgs::msg::Float32 msg_steering_angle;
    msg_steering_angle.data = cmd_target_steering_angle;
    pub_target_steering_angle->publish(msg_steering_angle);

    std_msgs::msg::Float32 msg_target_velocity;
    msg_target_velocity.data = cmd_target_velocity;
    pub_target_velocity->publish(msg_target_velocity);

    // std_msgs::msg::UInt8 msg_hw_flag;
    // msg_hw_flag.data = cmd_hw_flag;
    // pub_hw_flag->publish(msg_hw_flag);

    std_msgs::msg::Int16 msg_global_fsm;
    msg_global_fsm.data = global_fsm.value;
    pub_global_fsm->publish(msg_global_fsm);
}