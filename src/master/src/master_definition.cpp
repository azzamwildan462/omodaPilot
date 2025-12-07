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

    curr_traj_optimized.header.stamp = this->now();
    curr_traj_optimized.header.frame_id = "base_link";
    curr_traj_raw.header.stamp = this->now();
    curr_traj_raw.header.frame_id = "base_link";
    prev_traj.header.stamp = this->now();
    prev_traj.header.frame_id = "base_link";
    prev_prev_traj.header.stamp = this->now();
    prev_prev_traj.header.frame_id = "base_link";
    pub_dbg_curr_traj_optimized->publish(curr_traj_optimized);
    pub_dbg_curr_traj_raw->publish(curr_traj_raw);
    pub_dbg_prev_traj->publish(prev_traj);
    pub_dbg_prev_prev_traj->publish(prev_prev_traj);
}

void Master::requestPathBaseLink(double x, double y, double theta_rad)
{
    if (!nav2_client->wait_for_action_server(std::chrono::seconds(0)))
    {
        RCLCPP_WARN(this->get_logger(), "ComputePathToPose server belum ready");
        return;
    }

    static rclcpp::Time last_time_request = this->now();
    rclcpp::Duration time_since_last_request = this->now() - last_time_request;
    if (time_since_last_request.seconds() < 1.0)
    {
        logger.warn("Request path terlalu sering, tunggu 1 detik");
        return;
    }
    last_time_request = this->now();

    ComputePathToPose::Goal goal_msg;

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "base_link";
    goal.header.stamp = this->now();

    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0.0;

    goal.pose.orientation.z = std::sin(theta_rad / 2.0);
    goal.pose.orientation.w = std::cos(theta_rad / 2.0);

    goal_msg.goal = goal;
    goal_msg.use_start = false;         // start = pose robot terbaru (TF)
    goal_msg.planner_id = "SmacHybrid"; // sesuaikan dengan YAML Nav2

    auto options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();

    options.goal_response_callback =
        [this](const GoalHandle::SharedPtr &gh)
    {
        if (!gh)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal ditolak Nav2");
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "Goal diterima Nav2");
        }
    };

    options.result_callback =
        [this](const GoalHandle::WrappedResult &result)
    {
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_WARN(this->get_logger(), "Path planning gagal (code=%d)",
                        static_cast<int>(result.code));
            return;
        }

        const auto &path = result.result->path;
        RCLCPP_INFO(this->get_logger(),
                    "Path OK: frame=%s, poses=%zu",
                    path.header.frame_id.c_str(),
                    path.poses.size());

        curr_traj_raw = path;
    };

    nav2_client->async_send_goal(goal_msg, options);
}