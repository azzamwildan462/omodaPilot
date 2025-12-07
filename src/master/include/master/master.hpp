#ifndef MASTER_HPP
#define MASTER_HPP

#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int8.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/simple_fsm.hpp"
#include "ros2_utils/pid.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#define CMD_STEER_ACTIVE 0b01
#define CMD_GAS_ACTIVE 0b10
#define CMD_GAS_FULL_STOP 0b100
#define CMD_GAS_ACCEL_ON 0b1000
#define CMD_ACC_BTN_PRESS 0b10000

#define FSM_GLOBAL_INIT 0
#define FSM_GLOBAL_PREOP 1
#define FSM_GLOBAL_SAFEOP 2
#define FSM_GLOBAL_OP_3 3
#define FSM_GLOBAL_OP_4 4
#define FSM_GLOBAL_OP_5 5
#define FSM_GLOBAL_OP_2 6
#define FSM_GLOBAL_RECORD_ROUTE 7
#define FSM_GLOBAL_MAPPING 8

#define FSM_LOCAL_PRE_FOLLOW_LANE 0
#define FSM_LOCAL_FOLLOW_LANE 1
#define FSM_LOCAL_MENUNGGU_STATION_1 2
#define FSM_LOCAL_MENUNGGU_STATION_2 3
#define FSM_LOCAL_MENUNGGU_STOP 4

class Master : public rclcpp::Node
{
public:
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

    rclcpp::TimerBase::SharedPtr tim_routine;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_target_steering_angle;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_target_velocity;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_hw_flag;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_global_fsm;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_key_pressed;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_fb_steering_angle;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_fb_current_velocity;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_throttle_position;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_brake_position;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_gear_status;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_steer_torque;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_global_traj;
    rclcpp_action::Client<ComputePathToPose>::SharedPtr nav2_client;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_curr_traj_raw;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_curr_traj_optimized;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_prev_traj;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_prev_prev_traj;

    HelpLogger logger;

    // COnfigs
    bool disable_nav2 = false;
    float MAX_STEERING_ANGLE = 5.0;
    float MIN_STEERING_ANGLE = -5.0;
    float profile_max_acceleration = 1;
    float profile_max_decceleration = 2;
    float profile_max_velocity = 1.5; // m/s (1 m/s == 3.6 km/h)
    float profile_max_accelerate_jerk = 10;
    float profile_max_decelerate_jerk = 100;
    float profile_max_braking = 3;
    float profile_max_braking_acceleration = 2000;
    float profile_max_braking_jerk = 3000;
    float wheelbase = 2.63;
    std::vector<double> pid_terms;
    float roda2steering_ratio = 17.5; // Rasio roda ke setir

    float lookahead_distance_global = 10.0; // meter
    float lookahead_distance_local = 6.0;

    // Sesuatu yang berguna
    float dt = 0.02;
    int16_t current_key_pressed = 0;
    int16_t prev_key_pressed = 0;

    // Motion lama
    float cmd_target_steering_angle = 0;
    float cmd_target_velocity = 0;
    uint8_t cmd_hw_flag = 0;
    float fb_steering_angle = 0;
    float fb_current_velocity = 0;
    float throttle_position = 0;
    int16_t brake_position = 0;
    uint8_t gear_status = 0;
    int8_t steer_torque = 0;

    // Time
    rclcpp::Time last_time_steer_button_press;
    rclcpp::Time time_now;

    // FSM
    MachineState global_fsm;

    // MOtion baru
    float actuation_ax = 0;
    float actuation_ay = 0;
    float actuation_az = 0;
    float actuation_vx = 0;
    float actuation_vy = 0;
    float actuation_wz = 0; // Ini posisi
    PID pid_vx;
    uint8_t status_hardware_ready = 0;
    float offset_sudut_steering = 0.0;

    // Kontrol
    nav_msgs::msg::Path third_party_global_traj;
    nav_msgs::msg::Path curr_traj_raw;
    nav_msgs::msg::Path curr_traj_optimized;
    nav_msgs::msg::Path prev_traj;
    nav_msgs::msg::Path prev_prev_traj;

    float target_vel_x_dummy = 0;
    float target_vel_y_dummy = 0;
    float target_pos_theta_dummy = 0;

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
    void callback_sub_global_traj(const nav_msgs::msg::Path::SharedPtr msg);

    // Misc
    // ===============================================================================================
    void transmit_all();
    void send_goal_pose(float x, float y, float yaw);
    void requestPathBaseLink(double x, double y, double theta_rad);

    // Motion
    // ===============================================================================================
    void manual_motion(float vx, float vy, float wz);
    void global_navigation_motion();
    void local_trajectory_optimization();
    void local_traj2velocity_steering(float *pvelocity, float *psteering_angle);
    void gas_manual_control();
    void steer_manual_control();
    void full_control_mode();
};

#endif // MASTER_HPP