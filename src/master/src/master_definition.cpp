#include "master/master.hpp"

void Master::transmit_all()
{
    std_msgs::msg::Float32 msg_steering_angle;
    msg_steering_angle.data = cmd_target_steering_angle;
    pub_target_steering_angle->publish(msg_steering_angle);

    std_msgs::msg::Float32 msg_target_velocity;
    msg_target_velocity.data = cmd_target_velocity;
    pub_target_velocity->publish(msg_target_velocity);

    // logger.info("TX CMD: %.2f %.2f", cmd_target_velocity, cmd_target_steering_angle);

    // std_msgs::msg::UInt8 msg_hw_flag;
    // msg_hw_flag.data = cmd_hw_flag;
    // pub_hw_flag->publish(msg_hw_flag);

    std_msgs::msg::Int16 msg_global_fsm;
    msg_global_fsm.data = global_fsm.value;
    pub_global_fsm->publish(msg_global_fsm);

    curr_traj_optimized.header.stamp = this->now();
    curr_traj_optimized.header.frame_id = "map";
    curr_traj_raw.header.stamp = this->now();
    curr_traj_raw.header.frame_id = "map";
    prev_traj.header.stamp = this->now();
    prev_traj.header.frame_id = "map";
    prev_prev_traj.header.stamp = this->now();
    prev_prev_traj.header.frame_id = "map";
    pub_dbg_curr_traj_optimized->publish(curr_traj_optimized);
    pub_dbg_curr_traj_raw->publish(curr_traj_raw);
    pub_dbg_prev_traj->publish(prev_traj);
    pub_dbg_prev_prev_traj->publish(prev_prev_traj);

    static uint16_t divider_waypoint_pub_counter = 0;
    if (divider_waypoint_pub_counter++ >= 25)
    {
        divider_waypoint_pub_counter = 0;

        sensor_msgs::msg::PointCloud msg_waypoints;
        msg_waypoints.header.stamp = this->now();
        msg_waypoints.header.frame_id = "map";
        for (auto i : waypoints)
        {
            geometry_msgs::msg::Point32 p;
            p.x = i.x;
            p.y = i.y;
            p.z = 0;
            msg_waypoints.points.push_back(p);
        }
        pub_waypoints->publish(msg_waypoints);
        pub_terminals->publish(terminals);
    }

    pub_icr->publish(draw_icr_ackermann(cmd_target_velocity, cmd_target_steering_angle, this->wheelbase, "base_link"));
}

void Master::process_record_route()
{
    static float prev_x = fb_final_pose_xyo[0];
    static float prev_y = fb_final_pose_xyo[1];

    float dx = fb_final_pose_xyo[0] - prev_x;
    float dy = fb_final_pose_xyo[1] - prev_y;
    float d = sqrt(dx * dx + dy * dy);

    if (d > 0.2)
    {
        waypoint_t wp;
        wp.x = fb_final_pose_xyo[0];
        wp.y = fb_final_pose_xyo[1];
        wp.fb_velocity = fb_current_velocity;
        wp.fb_steering = fb_steering_angle;
        waypoints.push_back(wp);
        prev_x = wp.x;
        prev_y = wp.y;
        // logger.info("Recorded waypoint %.2f %.2f %.2f %.2f", wp.x, wp.y, wp.fb_velocity, wp.fb_steering);
    }
}

void Master::process_load_terminals()
{
    std::ifstream fin;
    std::string line;
    std::vector<std::string> tokens;
    bool is_terminal_loaded_normally = false;
    try
    {
        fin.open(terminal_file_path, std::ios::in);
        if (fin.is_open())
        {
            is_terminal_loaded_normally = true;
            while (std::getline(fin, line))
            {
                if (line.find("type") != std::string::npos)
                {
                    continue;
                }
                boost::split(tokens, line, boost::is_any_of(","));

                for (auto &token : tokens)
                {
                    boost::trim(token);
                }

                ros2_interface::msg::Terminal terminal;
                terminal.type = std::stoi(tokens[0]);
                terminal.id = std::stoi(tokens[1]);
                terminal.target_pose_x = std::stof(tokens[2]);
                terminal.target_pose_y = std::stof(tokens[3]);
                terminal.target_pose_theta = std::stof(tokens[4]);
                terminal.target_max_velocity_x = std::stof(tokens[5]);
                terminal.target_max_velocity_y = std::stof(tokens[6]);
                terminal.target_max_velocity_theta = std::stof(tokens[7]);
                terminal.radius_area = std::stof(tokens[8]);
                terminal.target_lookahead_distance = std::stof(tokens[9]);
                terminal.obs_scan_r = std::stof(tokens[10]);
                terminal.stop_time_s = std::stof(tokens[11]);
                terminal.scan_min_x = std::stof(tokens[12]);
                terminal.scan_max_x = std::stof(tokens[13]);
                terminal.scan_min_y = std::stof(tokens[14]);
                terminal.scan_max_y = std::stof(tokens[15]);
                terminal.obs_threshold = std::stof(tokens[16]);

                logger.info("term: %d || %.2f", terminal.id, terminal.obs_threshold);

                // if (transform_map2odom)
                // {
                //     tf2::Transform tf_terminal_pose;
                //     tf_terminal_pose.setOrigin(tf2::Vector3(terminal.target_pose_x, terminal.target_pose_y, 0));
                //     tf2::Quaternion q;
                //     q.setRPY(0, 0, terminal.target_pose_theta);
                //     tf_terminal_pose.setRotation(q);

                //     tf2::Transform tf_transformed = manual_map2odom_tf * tf_terminal_pose;

                //     terminal.target_pose_x = tf_transformed.getOrigin().getX();
                //     terminal.target_pose_y = tf_transformed.getOrigin().getY();

                //     tf2::Matrix3x3 m(tf_transformed.getRotation());
                //     double roll, pitch, yaw;
                //     m.getRPY(roll, pitch, yaw);
                //     terminal.target_pose_theta = yaw;
                // }

                terminals.terminals.push_back(terminal);

                tokens.clear();
            }
            fin.close();

            logger.info("Terminal file loaded");
        }

        if (fin.fail() && !fin.is_open() && !is_terminal_loaded_normally)
        {
            logger.warn("Failed to load terminal file, Recreate the terminal file");
            process_save_terminals();
        }
    }
    catch (const std::exception &e)
    {
        logger.error("Failed to load terminal file: %s, Recreate the terminal file", e.what());
        process_save_terminals();
    }
}

void Master::process_save_terminals()
{
    std::ofstream fout;

    try
    {
        fout.open(terminal_file_path, std::ios::out);
        if (fout.is_open())
        {
            fout << "type, id, x, y, theta, max_vx, max_vy, max_vtheta, radius_area, lookahead_distance, obs_scan_r, stop_time_s, scan_min_x, scan_max_x, scan_min_y, scan_max_y, obs_threshold" << std::endl;
            for (auto terminal : terminals.terminals)
            {
                int terminal_type_integer = terminal.type;
                int terminal_id_integer = terminal.id;
                fout << terminal_type_integer << ", " << terminal_id_integer << ", " << terminal.target_pose_x << ", " << terminal.target_pose_y << ", " << terminal.target_pose_theta << ", " << terminal.target_max_velocity_x << ", " << terminal.target_max_velocity_y << ", " << terminal.target_max_velocity_theta << ", " << terminal.radius_area << ", " << terminal.target_lookahead_distance << ", " << terminal.obs_scan_r << ", " << terminal.stop_time_s << ", " << terminal.scan_min_x << ", " << terminal.scan_max_x << ", " << terminal.scan_min_y << ", " << terminal.scan_max_y << ", " << terminal.obs_threshold << std::endl;
            }
            fout.close();

            logger.info("Terminal file saved");
        }
    }
    catch (const std::exception &e)
    {
        logger.warn("Failed to open terminal file, Recreate the terminal file");
    }
}

void Master::process_load_waypoints()
{
    static float prev_wp_x = 0;
    static float prev_wp_y = 0;

    waypoints.clear();

    if (!boost::filesystem::exists(waypoint_file_path))
    {
        logger.error("File %s does not exist. Create a new file.", waypoint_file_path.c_str());
        std::ofstream file(waypoint_file_path);
        file << "x,y,fb_velocity,fb_steering" << std::endl;
        file.close();
    }

    std::ifstream file;

    file.open(waypoint_file_path);
    if (file.is_open())
    {
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        while (file.good())
        {
            if (file.peek() == EOF)
            {
                break;
            }
            waypoint_t wp;
            file >> wp.x;
            file.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file >> wp.y;
            file.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file >> wp.fb_velocity;
            file.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file >> wp.fb_steering;
            file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

            float dx = wp.x - prev_wp_x;
            float dy = wp.y - prev_wp_y;
            wp.arah = atan2f(dy, dx);

            prev_wp_x = wp.x;
            prev_wp_y = wp.y;

            waypoints.push_back(wp);
        }
        file.close();
        logger.info("Read %d waypoints from file %s.", waypoints.size(), waypoint_file_path.c_str());
    }
}

void Master::process_save_waypoints()
{
    std::ofstream file;
    file.open(waypoint_file_path);

    if (file.is_open())
    {
        file << "x,y,fb_velocity,fb_steering" << std::endl;
        for (auto i : waypoints)
        {
            file << i.x << "," << i.y << "," << i.fb_velocity << "," << i.fb_steering << std::endl;
        }
        file.close();
        logger.info("Saved %d waypoints to file %s.", waypoints.size(), waypoint_file_path.c_str());
    }
    else
    {
        logger.error("Failed to save waypoints to file %s.", waypoint_file_path.c_str());
    }
}

void Master::cvt_waypoint_t_2path_msgs(nav_msgs::msg::Path *ppath)
{
    ppath->header.stamp = this->now();
    ppath->header.frame_id = "map";
    ppath->poses.clear();

    for (auto i : waypoints)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->now();
        pose.header.frame_id = "map";
        pose.pose.position.x = i.x;
        pose.pose.position.y = i.y;
        pose.pose.position.z = 0.0;

        pose.pose.orientation.z = i.arah; // asdasdasda malas convert ke quarternioin

        ppath->poses.push_back(pose);
    }
}

sensor_msgs::msg::PointCloud Master::draw_icr_ackermann(
    double target_velocity,       // m/s
    double target_steering_angle, // rad (steering depan)
    double wheelbase_,            // m (rear → front axle)
    std::string frame_id)
{
    sensor_msgs::msg::PointCloud cloud;
    cloud.header.frame_id = frame_id;
    cloud.header.stamp = this->now();

    // ===== kondisi lurus / diam =====
    if (std::abs(target_steering_angle) < 1e-4 ||
        std::abs(target_velocity) < 1e-4)
    {
        return cloud;
    }

    // ===== hitung radius ICR =====
    double R = wheelbase_ / std::tan(target_steering_angle);

    // ===== titik ICR =====
    geometry_msgs::msg::Point32 icr;
    icr.x = 0.0;
    icr.y = R;
    icr.z = 0.0;
    cloud.points.push_back(icr);

    // ===== gambar lintasan rear axle =====
    constexpr int NUM_POINTS = 80;
    constexpr double ARC = M_PI / 2.0; // 90 deg ke depan

    for (int i = 0; i < NUM_POINTS; ++i)
    {
        double a = -ARC / 2.0 + ARC * i / (NUM_POINTS - 1);

        geometry_msgs::msg::Point32 p;
        p.x = R * std::sin(a);
        p.y = R - R * std::cos(a);
        p.z = 0.0;

        cloud.points.push_back(p);
    }

    return cloud;
}

void Master::request_path_through_poses_nav2(const nav_msgs::msg::Path &input_path, std::string frame_id, std::string planner_id)
{
    status_nav2_through_poses_valid = false;

    (void)frame_id;
    rclcpp::Time t_now = this->now();
    if (!nav2_client_through_pose->wait_for_action_server(std::chrono::seconds(0)))
    {
        RCLCPP_WARN(this->get_logger(), "ComputePathThroughPoses server belum ready");
        return;
    }

    static const double min_time_between_request = 0.2; // detik
    static rclcpp::Time last_time_request = t_now;
    rclcpp::Duration time_since_last_request = t_now - last_time_request;
    if (time_since_last_request.seconds() < min_time_between_request)
    {
        // logger.warn("Request path terlalu sering, tunggu %.2lf detik", min_time_between_request);
        return;
    }
    last_time_request = t_now;

    ComputePathThroughPoses::Goal goal_msg;

    goal_msg.goals = input_path.poses;

    // Mengisi frame id per goal
    for (auto &pose_stamped : goal_msg.goals)
    {
        pose_stamped.header.frame_id = frame_id;
        pose_stamped.header.stamp = t_now;
    }

    goal_msg.planner_id = planner_id;
    goal_msg.use_start = false; // start = pose robot terbaru (TF)

    auto options = rclcpp_action::Client<ComputePathThroughPoses>::SendGoalOptions();

    options.goal_response_callback =
        [this](const GoalHandleThroughPoses::SharedPtr &gh)
    {
        if (!gh)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal ditolak Nav2");
            status_nav2_through_poses_valid = false;

            if (global_nav_strategy == GLOBAL_NAV_STRATEGY_IST || global_nav_strategy == GLOBAL_NAV_STRATEGY_IST_NAV2)
            {
                nav_msgs::msg::Path new_ist_alg_global_traj;
                set_index_now_2_global(&ist_alg_global_traj, &new_ist_alg_global_traj, 6.2, 1.57, 0);

                mutex_curr_traj_raw.lock();
                curr_traj_raw = new_ist_alg_global_traj; // ga pakte mutex soalnya inline
                mutex_curr_traj_raw.unlock();
            }
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "Goal diterima Nav2");
        }
    };

    options.result_callback =
        [this](const GoalHandleThroughPoses::WrappedResult &result)
    {
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_WARN(this->get_logger(), "Path planning gagal (code=%d)",
                        static_cast<int>(result.code));
            status_nav2_through_poses_valid = false;

            if (global_nav_strategy == GLOBAL_NAV_STRATEGY_IST || global_nav_strategy == GLOBAL_NAV_STRATEGY_IST_NAV2)
            {
                nav_msgs::msg::Path new_ist_alg_global_traj;
                set_index_now_2_global(&ist_alg_global_traj, &new_ist_alg_global_traj, 6.2, 1.57, 0);

                mutex_curr_traj_raw.lock();
                curr_traj_raw = new_ist_alg_global_traj; // ga pakte mutex soalnya inline
                mutex_curr_traj_raw.unlock();
            }

            return;
        }

        status_nav2_through_poses_valid = true;

        const auto &path = result.result->path;
        RCLCPP_INFO(this->get_logger(),
                    "Path OK: frame=%s, poses=%zu",
                    path.header.frame_id.c_str(),
                    path.poses.size());

        mutex_curr_traj_raw.lock();
        curr_traj_raw = path;
        mutex_curr_traj_raw.unlock();
    };

    nav2_client_through_pose->async_send_goal(goal_msg, options);
}

void Master::request_path_nav2(double x, double y, double theta_rad, std::string frame_id, std::string planner_id)
{
    if (!nav2_client_to_pose->wait_for_action_server(std::chrono::seconds(0)))
    {
        RCLCPP_WARN(this->get_logger(), "ComputePathToPose server belum ready");
        return;
    }

    static const double min_time_between_request = 0.2; // detik
    static rclcpp::Time last_time_request = this->now();
    rclcpp::Duration time_since_last_request = this->now() - last_time_request;
    if (time_since_last_request.seconds() < min_time_between_request)
    {
        // logger.warn("Request path terlalu sering, tunggu %.2lf detik", min_time_between_request);
        return;
    }
    last_time_request = this->now();

    ComputePathToPose::Goal goal_msg;

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = frame_id;
    goal.header.stamp = this->now();

    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0.0;

    goal.pose.orientation.z = std::sin(theta_rad / 2.0);
    goal.pose.orientation.w = std::cos(theta_rad / 2.0);

    goal_msg.goal = goal;
    goal_msg.use_start = false; // start = pose robot terbaru (TF)
    goal_msg.planner_id = planner_id;

    auto options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();

    options.goal_response_callback =
        [this](const GoalHandleToPose::SharedPtr &gh)
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
        [this](const GoalHandleToPose::WrappedResult &result)
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

        mutex_curr_traj_raw.lock();
        curr_traj_raw = path;
        mutex_curr_traj_raw.unlock();
    };

    nav2_client_to_pose->async_send_goal(goal_msg, options);
}

void Master::clip_sudut_180_min180(float *pangle, bool is_rad)
{
    if (!is_rad)
    {
        while (*pangle > 180.0)
        {
            *pangle -= 360.0;
        }
        while (*pangle < -180.0)
        {
            *pangle += 360.0;
        }
        return;
    }

    while (*pangle > M_PI)
    {
        *pangle -= 2 * M_PI;
    }
    while (*pangle < -M_PI)
    {
        *pangle += 2 * M_PI;
    }
}
