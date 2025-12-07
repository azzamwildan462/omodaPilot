#include "master/master.hpp"

Master::Master()
    : Node("master")
{
    this->declare_parameter("profile_max_acceleration", profile_max_acceleration);
    this->get_parameter("profile_max_acceleration", profile_max_acceleration);

    this->declare_parameter("profile_max_decceleration", profile_max_decceleration);
    this->get_parameter("profile_max_decceleration", profile_max_decceleration);

    this->declare_parameter("profile_max_velocity", profile_max_velocity);
    this->get_parameter("profile_max_velocity", profile_max_velocity);

    this->declare_parameter("profile_max_accelerate_jerk", profile_max_accelerate_jerk);
    this->get_parameter("profile_max_accelerate_jerk", profile_max_accelerate_jerk);

    this->declare_parameter("profile_max_decelerate_jerk", profile_max_decelerate_jerk);
    this->get_parameter("profile_max_decelerate_jerk", profile_max_decelerate_jerk);

    this->declare_parameter("profile_max_braking", profile_max_braking);
    this->get_parameter("profile_max_braking", profile_max_braking);

    this->declare_parameter("profile_max_braking_acceleration", profile_max_braking_acceleration);
    this->get_parameter("profile_max_braking_acceleration", profile_max_braking_acceleration);

    this->declare_parameter("profile_max_braking_jerk", profile_max_braking_jerk);
    this->get_parameter("profile_max_braking_jerk", profile_max_braking_jerk);

    this->declare_parameter<std::vector<double>>("pid_terms", {0.0070, 0.000000, 0, 0.02, -0.04, 0.4, -0.0005, 0.0005});
    this->get_parameter("pid_terms", pid_terms);

    this->declare_parameter("offset_sudut_steering", offset_sudut_steering);
    this->get_parameter("offset_sudut_steering", offset_sudut_steering);

    this->declare_parameter("roda2steering_ratio", roda2steering_ratio);
    this->get_parameter("roda2steering_ratio", roda2steering_ratio);

    this->declare_parameter("lookahead_distance_global", lookahead_distance_global);
    this->get_parameter("lookahead_distance_global", lookahead_distance_global);

    this->declare_parameter("lookahead_distance_local", lookahead_distance_local);
    this->get_parameter("lookahead_distance_local", lookahead_distance_local);

    this->declare_parameter("disable_nav2", disable_nav2);
    this->get_parameter("disable_nav2", disable_nav2);

    if (!logger.init())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
        rclcpp::shutdown();
    }

    pub_target_steering_angle = this->create_publisher<std_msgs::msg::Float32>("cmd_target_steering_angle", 1);
    pub_target_velocity = this->create_publisher<std_msgs::msg::Float32>("cmd_target_velocity", 1);
    pub_hw_flag = this->create_publisher<std_msgs::msg::UInt8>("cmd_hw_flag", 1);
    pub_global_fsm = this->create_publisher<std_msgs::msg::Int16>("global_fsm", 1);
    pub_dbg_curr_traj_optimized = this->create_publisher<nav_msgs::msg::Path>("dbg_curr_traj_optimized", 1);
    pub_dbg_curr_traj_raw = this->create_publisher<nav_msgs::msg::Path>("dbg_curr_traj_raw", 1);
    pub_dbg_prev_traj = this->create_publisher<nav_msgs::msg::Path>("dbg_prev_traj", 1);
    pub_dbg_prev_prev_traj = this->create_publisher<nav_msgs::msg::Path>("dbg_prev_prev_traj", 1);

    sub_fb_steering_angle = this->create_subscription<std_msgs::msg::Float32>(
        "fb_steering_angle", 1, std::bind(&Master::callback_sub_fb_steering_angle, this, std::placeholders::_1));
    sub_fb_current_velocity = this->create_subscription<std_msgs::msg::Float32>(
        "fb_current_velocity", 1, std::bind(&Master::callback_sub_fb_current_velocity, this, std::placeholders::_1));
    sub_throttle_position = this->create_subscription<std_msgs::msg::Float32>(
        "fb_throttle_position", 1, std::bind(&Master::callback_sub_throttle_position, this, std::placeholders::_1));
    sub_brake_position = this->create_subscription<std_msgs::msg::Int16>(
        "fb_brake_position", 1, std::bind(&Master::callback_sub_brake_position, this, std::placeholders::_1));
    sub_gear_status = this->create_subscription<std_msgs::msg::UInt8>(
        "fb_gear_status", 1, std::bind(&Master::callback_sub_gear_status, this, std::placeholders::_1));
    sub_steer_torque = this->create_subscription<std_msgs::msg::Int8>(
        "fb_steer_torque", 1, std::bind(&Master::callback_sub_steer_torque, this, std::placeholders::_1));
    sub_key_pressed = this->create_subscription<std_msgs::msg::Int16>(
        "/key_pressed", 1, std::bind(&Master::callback_sub_key_pressed, this, std::placeholders::_1));
    sub_global_traj = this->create_subscription<nav_msgs::msg::Path>(
        "waypoint_router/path", 1, std::bind(&Master::callback_sub_global_traj, this, std::placeholders::_1));

    nav2_client = rclcpp_action::create_client<ComputePathToPose>(this, "compute_path_to_pose");

    tim_routine = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&Master::callback_routine, this));

    logger.info("Master node initialized");
}

Master::~Master()
{
}
void Master::callback_sub_global_traj(const nav_msgs::msg::Path::SharedPtr msg)
{
    third_party_global_traj = *msg;
}

void Master::callback_sub_fb_steering_angle(const std_msgs::msg::Float32::SharedPtr msg)
{
    fb_steering_angle = msg->data;
}

void Master::callback_sub_fb_current_velocity(const std_msgs::msg::Float32::SharedPtr msg)
{
    fb_current_velocity = msg->data;
}

void Master::callback_sub_throttle_position(const std_msgs::msg::Float32::SharedPtr msg)
{
    throttle_position = msg->data;
}

void Master::callback_sub_brake_position(const std_msgs::msg::Int16::SharedPtr msg)
{
    brake_position = msg->data;
}

void Master::callback_sub_gear_status(const std_msgs::msg::UInt8::SharedPtr msg)
{
    gear_status = msg->data;
}

void Master::callback_sub_steer_torque(const std_msgs::msg::Int8::SharedPtr msg)
{
    steer_torque = msg->data;
}

void Master::callback_sub_key_pressed(const std_msgs::msg::Int16::SharedPtr msg)
{
    current_key_pressed = msg->data;

    switch (current_key_pressed)
    {

    case 'q':
        cmd_target_steering_angle = 4.8;
        break;
    case 'e':
        cmd_target_steering_angle = -4.8;
        break;

    case '9':
        cmd_target_steering_angle += 0.01;
        break;
    case '0':
        cmd_target_steering_angle -= 0.01;
        break;

    case 'w':
        target_vel_x_dummy = 1.0;
        break;
    case 's':
        target_vel_x_dummy = -1.0;
        break;
    case 'a':
        target_pos_theta_dummy = 0.5;
        break;
    case 'd':
        target_pos_theta_dummy = -0.5;
        break;

    case 'z':
        cmd_target_velocity = -1.0;
        break;

    case '1':
        global_fsm.value = FSM_GLOBAL_PREOP;
        break;

    case '2':
        global_fsm.value = FSM_GLOBAL_SAFEOP;
        break;

    case '3':
        global_fsm.value = FSM_GLOBAL_OP_3;
        break;

    case '4':
        global_fsm.value = FSM_GLOBAL_OP_4;
        break;

    case '5':
        global_fsm.value = FSM_GLOBAL_OP_5;
        break;

    case 'l':
        cmd_hw_flag |= CMD_STEER_ACTIVE;
        break;
    case 'k':
        cmd_hw_flag |= CMD_GAS_ACTIVE;
        break;
    case 'j':
        cmd_hw_flag |= CMD_GAS_FULL_STOP;
        break;
    case 'h':
        cmd_hw_flag |= CMD_GAS_ACCEL_ON;
        break;
    case ',':
        cmd_hw_flag &= ~CMD_STEER_ACTIVE;
        break;
    case 'm':
        cmd_hw_flag &= ~CMD_GAS_ACTIVE;
        break;
    case 'n':
        cmd_hw_flag &= ~CMD_GAS_FULL_STOP;
        break;
    case 'b':
        cmd_hw_flag &= ~CMD_GAS_ACCEL_ON;
        break;

    case 't':
        cmd_hw_flag |= CMD_ACC_BTN_PRESS;
        last_time_steer_button_press = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        break;

    case ' ':
        cmd_hw_flag = 0;
        cmd_target_steering_angle = 0;
        cmd_target_velocity = 0;
        break;
    }

    if (cmd_target_steering_angle > MAX_STEERING_ANGLE)
        cmd_target_steering_angle = MAX_STEERING_ANGLE;
    else if (cmd_target_steering_angle < MIN_STEERING_ANGLE)
        cmd_target_steering_angle = MIN_STEERING_ANGLE;
}

void Master::callback_routine()
{
    time_now = rclcpp::Clock(RCL_SYSTEM_TIME).now();

    // last_time_steer_button_press - time_now > 2 second
    if ((cmd_hw_flag & CMD_ACC_BTN_PRESS) != 0 && (time_now - last_time_steer_button_press).seconds() > 1.0)
    {
        cmd_hw_flag &= ~CMD_ACC_BTN_PRESS;
    }

    switch (global_fsm.value)
    {
    case FSM_GLOBAL_INIT:
        break;

    case FSM_GLOBAL_PREOP:
        break;

    case FSM_GLOBAL_SAFEOP:
        break;

    case FSM_GLOBAL_OP_3:
        full_control_mode();
        break;

    case FSM_GLOBAL_OP_4:
        steer_manual_control();
        break;

    case FSM_GLOBAL_OP_5:
        gas_manual_control();
        break;
    }

    transmit_all();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_master = std::make_shared<Master>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_master);
    executor.spin();

    return 0;
}