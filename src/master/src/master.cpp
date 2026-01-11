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

    this->declare_parameter("wheelbase", wheelbase);
    this->get_parameter("wheelbase", wheelbase);

    this->declare_parameter("waypoint_file_path", "/home/robot/waypoints.yaml");
    this->get_parameter("waypoint_file_path", waypoint_file_path);

    this->declare_parameter("terminal_file_path", "/home/robot/terminal.yaml");
    this->get_parameter("terminal_file_path", terminal_file_path);

    this->declare_parameter("global_nav_strategy", global_nav_strategy);
    this->get_parameter("global_nav_strategy", global_nav_strategy);

    this->declare_parameter("local_nav_strategy", local_nav_strategy);
    this->get_parameter("local_nav_strategy", local_nav_strategy);

    this->declare_parameter("use_terminal_as_a_constraint", use_terminal_as_a_constraint);
    this->get_parameter("use_terminal_as_a_constraint", use_terminal_as_a_constraint);

    this->declare_parameter("bypass_wait", bypass_wait);
    this->get_parameter("bypass_wait", bypass_wait);

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
    pub_waypoints = this->create_publisher<sensor_msgs::msg::PointCloud>("waypoints", 1);
    pub_icr = this->create_publisher<sensor_msgs::msg::PointCloud>("icr", 1);
    pub_terminals = this->create_publisher<ros2_interface::msg::TerminalArray>("terminals", 1);

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
    sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 1, std::bind(&Master::callback_sub_scan, this, std::placeholders::_1));
    // sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
    //     "/slam/odometry/filtered", 1, std::bind(&Master::callback_sub_odometry, this, std::placeholders::_1));

    sub_joy = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 1, std::bind(&Master::callback_sub_joy, this, std::placeholders::_1));
    sub_flag_override_ctrl = this->create_subscription<std_msgs::msg::UInt8>(
        "fb_flag_override_ctrl", 1, std::bind(&Master::callback_sub_flag_override_ctrl, this, std::placeholders::_1));

    tf_base2_map_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_base2_map_listener = std::make_unique<tf2_ros::TransformListener>(*tf_base2_map_buffer, this);

    is_tf_initialized = false;
    while (!is_tf_initialized && !bypass_wait)
    {
        rclcpp::sleep_for(1s);
        logger.warn("Master Waiting for transforms to be available...");
        try
        {
            tf_base2_map = tf_base2_map_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
            is_tf_initialized = true;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
            is_tf_initialized = false;
        }
    }

    nav2_client_to_pose = rclcpp_action::create_client<ComputePathToPose>(this, "/compute_path_to_pose");
    nav2_client_through_pose = rclcpp_action::create_client<ComputePathThroughPoses>(this, "/compute_path_through_poses");

    global_fsm.value = FSM_GLOBAL_INIT;

    tim_routine = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&Master::callback_routine, this));

    logger.info("Master node initialized");
}

Master::~Master()
{
}

void Master::callback_sub_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    mutex_scan.lock();
    curr_scan = *msg;
    mutex_scan.unlock();
}

void Master::callback_sub_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    fb_final_pose_xyo[0] = msg->pose.pose.position.x;
    fb_final_pose_xyo[1] = msg->pose.pose.position.y;

    // Convert quaternion to yaw
    double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    fb_final_pose_xyo[2] = std::atan2(siny_cosp, cosy_cosp);
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

        // case 'q':
        //     cmd_target_steering_angle = 4.8;
        //     break;
        // case 'e':
        //     cmd_target_steering_angle = -4.8;
        //     break;

        // case '9':
        //     cmd_target_steering_angle += 0.01;
        //     break;
        // case '0':
        //     cmd_target_steering_angle -= 0.01;
        //     break;

    case 'w':
        target_vel_x_dummy += 1.0;
        break;
    case 's':
        target_vel_x_dummy += -1.0;
        break;
    case 'a':
        target_pos_theta_dummy += 0.5;
        break;
    case 'd':
        target_pos_theta_dummy += -0.5;
        break;

    case 'z':
        cmd_target_velocity = -1.0;
        break;

    case '0':
        terminals.terminals.clear();
        global_fsm.value = FSM_GLOBAL_INIT;
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

    case '7':
        global_fsm.value = FSM_GLOBAL_RECORD_ROUTE;
        break;

    case '8':
        global_fsm.value = FSM_GLOBAL_MAPPING;
        break;

    case ' ':
        terminals.terminals.clear();
        global_fsm.value = FSM_GLOBAL_INIT;
        cmd_target_steering_angle = 0.0;
        cmd_target_velocity = -1.0;
        target_pos_theta_dummy = 0.0;
        target_vel_x_dummy = 0.0;
        break;
    }
}

void Master::callback_sub_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    joy_axes = msg->axes;
    joy_buttons = msg->buttons;
    last_time_joy_msg = rclcpp::Clock(RCL_SYSTEM_TIME).now();
}

void Master::callback_sub_flag_override_ctrl(const std_msgs::msg::UInt8::SharedPtr msg)
{
    flag_override_status = msg->data;
    steer_override_active = (flag_override_status & FLAG_OVERRIDE_STEER) != 0;
}

void Master::process_joy_input()
{
    rclcpp::Time time_now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    double time_diff = (time_now - last_time_joy_msg).seconds();
    if (time_diff > 1.0)
    {
        // No recent joy message, do not process
        return;
    }

    // X (Button 0)
    if (joy_buttons[0])
    {
        global_fsm.value = FSM_GLOBAL_OP_5;
        logger.info("Joy: FSM switched to OP_5 (X pressed)");
    }

    // Circle (Button 1)
    if (joy_buttons[1])
    {
        global_fsm.value = FSM_GLOBAL_OP_4;
        logger.info("Joy: FSM switched to OP_4 (Circle pressed)");
    }

    // Square (Button 2)
    if (joy_buttons[4])
    {
        global_fsm.value = FSM_GLOBAL_OP_3;
        logger.info("Joy: FSM switched to OP_3 (Square pressed)");
    }

    // Triangle (Button 3)
    if (joy_buttons[3])
    {
        global_fsm.value = FSM_GLOBAL_SAFEOP;
        logger.info("Joy: FSM switched to SAFEOP (Triangle pressed)");
    }

    // ========== STEERING CONTROL ==========

    float raw_steering = joy_axes[joy_steering_axis];

    if (!steer_override_active)
    {
        target_pos_theta_dummy = raw_steering * raw_steering * raw_steering * joy_max_steering;
        // if (raw_steering < 0)
        //     target_pos_theta_dummy *= -1;
    }
    else
    {
        target_pos_theta_dummy = fb_steering_angle;
    }

    if (target_pos_theta_dummy > MAX_STEERING_ANGLE)
    {
        target_pos_theta_dummy = MAX_STEERING_ANGLE;
    }
    else if (target_pos_theta_dummy < MIN_STEERING_ANGLE)
    {
        target_pos_theta_dummy = MIN_STEERING_ANGLE;
    }

    target_pos_theta_dummy /= roda2steering_ratio;

    // ========== VELOCITY CONTROL ==========

    float throttle_raw = (1.0 - joy_axes[joy_throttle_axis]) / 2.0;
    float brake_raw = (1.0 - joy_axes[joy_brake_axis]) / 2.0;

    float throttle = (throttle_raw > joy_deadzone) ? throttle_raw : 0.0;
    float brake = (brake_raw > joy_deadzone) ? brake_raw : 0.0;

    if (brake > joy_deadzone)
    {
        target_vel_x_dummy = -1.0;
    }
    else if (throttle > joy_deadzone)
    {
        target_vel_x_dummy = throttle * joy_max_velocity;
    }
    else
    {
        target_vel_x_dummy = -1.0;
    }
}

void Master::callback_routine()
{
    current_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();

    if (is_tf_initialized)
    {
        tf_base2_map = tf_base2_map_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
        fb_final_pose_xyo[0] = tf_base2_map.transform.translation.x;
        fb_final_pose_xyo[1] = tf_base2_map.transform.translation.y;

        // Convert quaternion to yaw
        double siny_cosp = 2 * (tf_base2_map.transform.rotation.w * tf_base2_map.transform.rotation.z + tf_base2_map.transform.rotation.x * tf_base2_map.transform.rotation.y);
        double cosy_cosp = 1 - 2 * (tf_base2_map.transform.rotation.y * tf_base2_map.transform.rotation.y + tf_base2_map.transform.rotation.z * tf_base2_map.transform.rotation.z);
        fb_final_pose_xyo[2] = std::atan2(siny_cosp, cosy_cosp);

        // request_path_nav2(20.0, 0, 0, "base_link");
    }

    process_joy_input();

    // logger.info("GLOBAL FSM: %d", global_fsm.value);

    switch (global_fsm.value)
    {
    case FSM_GLOBAL_INIT:
        if (global_fsm.prev_value == FSM_GLOBAL_RECORD_ROUTE)
        {
            process_save_waypoints();
        }
        else
        {
            process_load_waypoints();
        }
        process_load_terminals();

        local_fsm.value = 0;

        cvt_waypoint_t_2path_msgs(&ist_alg_global_traj);
        global_fsm.value = FSM_GLOBAL_PREOP;
        break;

    case FSM_GLOBAL_PREOP:
    {
        float res = get_emergency_laser_scan(-0.5, 6.0, -1.5, 1.5, 4);
        logger.info("Tes obs val: %.3f", res);
    }
        local_fsm.value = 0;
        break;

    case FSM_GLOBAL_SAFEOP:
        // request_path_nav2(10.0, 0.0, 0.0, "base_link");

        local_fsm.value = 0;
        break;

    case FSM_GLOBAL_OP_3:
        process_local_fsm();
        // full_control_mode();
        break;

    case FSM_GLOBAL_OP_4:
        // steer_manual_control();
        manual_motion(target_vel_x_dummy, 0, 0);
        break;

    case FSM_GLOBAL_OP_5:
        // gas_manual_control();
        manual_motion(0, 0, target_pos_theta_dummy);
        break;

    case FSM_GLOBAL_RECORD_ROUTE:

        if (global_fsm.prev_value != FSM_GLOBAL_RECORD_ROUTE)
        {
            waypoints.clear();
            logger.info("Start recording route...");
        }
        process_record_route();
        break;

    case FSM_GLOBAL_MAPPING:
        break;
    }

    transmit_all();

    global_fsm.prev_value = global_fsm.value;
}

void Master::process_local_fsm()
{
    local_fsm.reentry(999, 1);

    // logger.info("LOCAL FSM: %d", local_fsm.value);

    static float stop_time_s = 20.0;
    switch (local_fsm.value)
    {
    case 999:
        local_fsm.resetUptimeTimeout();
        local_fsm.value = FSM_LOCAL_PRE_FOLLOW_LANE;
        break;

    case FSM_LOCAL_PRE_FOLLOW_LANE:
        manual_motion(-1, 0, 0);
        time_start_follow_lane = current_time;
        local_fsm.value = FSM_LOCAL_FOLLOW_LANE;
        break;

    case FSM_LOCAL_FOLLOW_LANE:
        full_control_mode();

        // Mencari terminal stop
        if ((current_time - time_start_follow_lane).seconds() > 10)
        {
            for (size_t i = 0; i < terminals.terminals.size(); i++)
            {
                float error_arah_hadap = terminals.terminals[i].target_pose_theta - fb_final_pose_xyo[2];
                while (error_arah_hadap > M_PI)
                    error_arah_hadap -= 2 * M_PI;
                while (error_arah_hadap < -M_PI)
                    error_arah_hadap += 2 * M_PI;

                if (fabs(error_arah_hadap) < 0.87)
                {
                    float dx = terminals.terminals[i].target_pose_x - fb_final_pose_xyo[0];
                    float dy = terminals.terminals[i].target_pose_y - fb_final_pose_xyo[1];
                    float jarak_robot_terminal = sqrtf(dx * dx + dy * dy);
                    // logger.warn("JARAK ROBOT TERMINAL: %.2f RADIUS AREA: %.2f", jarak_robot_terminal, terminals.terminals[i].radius_area);
                    if (jarak_robot_terminal < terminals.terminals[i].radius_area)
                    {
                        status_klik_terminal_terakhir = terminals.terminals[i].id;
                        logger.warn("STATUS KLIK TERMINAL TERAKHIR: %d", status_klik_terminal_terakhir);
                        if (terminals.terminals[i].type == TERMINAL_TYPE_STOP1)
                        {
                            local_fsm.resetUptimeTimeout();
                            local_fsm.value = FSM_LOCAL_MENUNGGU_STATION_1;
                            stop_time_s = terminals.terminals[i].stop_time_s;
                        }
                        else if (terminals.terminals[i].type == TERMINAL_TYPE_STOP2)
                        {
                            local_fsm.resetUptimeTimeout();
                            local_fsm.value = FSM_LOCAL_MENUNGGU_STATION_2;
                            stop_time_s = terminals.terminals[i].stop_time_s;
                        }
                        else if (terminals.terminals[i].type == TERMINAL_TYPE_STOP)
                        {
                            local_fsm.resetUptimeTimeout();
                            local_fsm.value = FSM_LOCAL_MENUNGGU_STOP;
                            stop_time_s = terminals.terminals[i].stop_time_s;
                        }
                        break;
                    }
                }
            }
        }
        break;

    case FSM_LOCAL_MENUNGGU_STOP:
    {
        logger.warn("MENUNGGU DI TERMINAL (%.2f)", stop_time_s);
        float target_velocity = 0;
        float target_steering_angle = 0;

        global_navigation_motion();
        local_trajectory_optimization();
        local_traj2velocity_steering(&target_velocity, &target_steering_angle);
        (void)target_velocity;

        manual_motion(-1, 0, target_steering_angle);

        local_fsm.timeout(FSM_LOCAL_PRE_FOLLOW_LANE, stop_time_s);
        break;
    }
    }
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
