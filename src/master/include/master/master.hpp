#ifndef MASTER_HPP
#define MASTER_HPP

#include <memory>
#include <string>
#include <functional>
#include "boost/filesystem.hpp"
#include "boost/thread/mutex.hpp"
#include "fstream"
#include <boost/algorithm/string.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "ros2_interface/msg/terminal_array.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int8.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/simple_fsm.hpp"
#include "ros2_utils/pid.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

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

#define GLOBAL_NAV_STRATEGY_HERNANDA 0
#define GLOBAL_NAV_STRATEGY_IST 1
#define GLOBAL_NAV_NO_NAV2 2
#define GLOBAL_NAV_STRATEGY_IST_NAV2 3

#define LOCAL_NAV_STRATEGY_NO_FILTER 0
#define LOCAL_NAV_STRATEGY_ALPHA_BETA 1
#define LOCAL_NAV_STRATEGY_ALPHA_BETA_GAMMA 2

#define FLAG_OVERRIDE_STEER 0b01
#define FLAG_OVERRIDE_GAS 0b10
#define FLAG_OVERRIDE_BRAKE 0b100

#define TERMINAL_TYPE_STOP 0x01
#define TERMINAL_TYPE_BELOKAN 0x02
#define TERMINAL_TYPE_STOP1 0x04
#define TERMINAL_TYPE_STOP2 0x08
#define TERMINAL_TYPE_STOP3 0x0C
#define TERMINAL_TYPE_STOP4 0x10
#define TERMINAL_TYPE_LURUS 0x20

using namespace std::chrono_literals;

typedef struct
{
    float x;
    float y;
    float theta;
    float fb_velocity;
    float fb_steering;
    float arah;
} waypoint_t;

class Master : public rclcpp::Node
{
public:
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    using GoalHandleToPose = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
    using ComputePathThroughPoses = nav2_msgs::action::ComputePathThroughPoses;
    using GoalHandleThroughPoses = rclcpp_action::ClientGoalHandle<ComputePathThroughPoses>;

    rclcpp::TimerBase::SharedPtr tim_routine;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_target_steering_angle;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_target_velocity;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_hw_flag;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_global_fsm;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_waypoints;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_icr;
    rclcpp::Publisher<ros2_interface::msg::TerminalArray>::SharedPtr pub_terminals;

    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_key_pressed;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_fb_steering_angle;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_fb_current_velocity;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_throttle_position;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_brake_position;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_gear_status;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_steer_torque;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_global_traj;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_flag_override_ctrl;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;

    rclcpp_action::Client<ComputePathToPose>::SharedPtr nav2_client_to_pose;
    rclcpp_action::Client<ComputePathThroughPoses>::SharedPtr nav2_client_through_pose;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_curr_traj_raw;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_curr_traj_optimized;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_prev_traj;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dbg_prev_prev_traj;

    HelpLogger logger;

    // COnfigs
    double timeout_menunggu_nav2_s = 0.8;
    float MAX_STEERING_ANGLE = 7.7;
    float MIN_STEERING_ANGLE = -7.7;
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
    std::string waypoint_file_path;
    std::string terminal_file_path;
    int global_nav_strategy = 0; // 0 = dari hernanda; 1 = kayak IST
    int local_nav_strategy = 0;  // 0 = no filter; 1 = 1st alpha beta filter; 2 = alpha beta gamma filter
    bool use_terminal_as_a_constraint = true;
    bool bypass_wait = false;

    float lookahead_distance_global = 10.0; // meter
    float lookahead_distance_local = 6.0;

    // Sesuatu yang berguna
    float dt = 0.02;
    int16_t current_key_pressed = 0;
    int16_t prev_key_pressed = 0;

    // TF
    bool is_tf_initialized = false;
    geometry_msgs::msg::TransformStamped tf_base2_map;
    std::unique_ptr<tf2_ros::Buffer> tf_base2_map_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_base2_map_listener;

    // Motion lama
    float cmd_target_steering_angle_velocity = 0;
    float cmd_target_steering_angle = 0;
    float cmd_target_velocity = 0;
    uint8_t cmd_hw_flag = 0;
    float fb_steering_angle = 0;
    float fb_current_velocity = 0;
    float throttle_position = 0;
    int16_t brake_position = 0;
    uint8_t gear_status = 0;
    int8_t steer_torque = 0;
    float fb_final_pose_xyo[3];

    // Time
    rclcpp::Time last_time_steer_button_press;
    rclcpp::Time current_time;
    rclcpp::Time time_start_follow_lane;

    // FSM
    MachineState global_fsm;
    MachineState local_fsm;

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
    nav_msgs::msg::Path ist_alg_global_traj;
    nav_msgs::msg::Path curr_traj_raw;
    nav_msgs::msg::Path curr_traj_optimized;
    nav_msgs::msg::Path prev_traj;
    nav_msgs::msg::Path prev_prev_traj;
    bool status_nav2_through_poses_valid = false;

    float target_vel_x_dummy = 0;
    float target_vel_y_dummy = 0;
    float target_pos_theta_dummy = 0;

    std::vector<waypoint_t> waypoints;
    ros2_interface::msg::TerminalArray terminals;

    int16_t master_status_emergency = 0;
    int16_t status_klik_terminal_terakhir = -1;

    sensor_msgs::msg::LaserScan curr_scan;

    // Mutex mutex mutexxxx
    boost::mutex mutex_curr_traj_raw;
    boost::mutex mutex_scan;

    // Joy configuration parameters
    int joy_steering_axis = 0; // Left analog horizontal
    int joy_throttle_axis = 4; // R2 trigger
    int joy_brake_axis = 5;    // L2 trigger
    float joy_deadzone = 0.1;
    float joy_max_steering = 7.7; // rad
    float joy_max_velocity = 3.0; // m/s

    std::vector<float> joy_axes;
    std::vector<int32_t> joy_buttons;
    rclcpp::Time last_time_joy_msg;

    // Override safety variables
    uint8_t flag_override_status = 0;
    bool steer_override_active = false;

    Master();
    ~Master();

    void process_joy_input();
    void callback_routine();
    void callback_sub_fb_steering_angle(const std_msgs::msg::Float32::SharedPtr msg);
    void callback_sub_fb_current_velocity(const std_msgs::msg::Float32::SharedPtr msg);
    void callback_sub_throttle_position(const std_msgs::msg::Float32::SharedPtr msg);
    void callback_sub_brake_position(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_gear_status(const std_msgs::msg::UInt8::SharedPtr msg);
    void callback_sub_steer_torque(const std_msgs::msg::Int8::SharedPtr msg);
    void callback_sub_key_pressed(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_global_traj(const nav_msgs::msg::Path::SharedPtr msg);
    void callback_sub_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void callback_sub_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    void callback_sub_joy(const sensor_msgs::msg::Joy::SharedPtr msg);
    void callback_sub_flag_override_ctrl(const std_msgs::msg::UInt8::SharedPtr msg);

    void process_local_fsm();

    // Misc
    // ===============================================================================================
    void transmit_all();
    void request_path_nav2(double x, double y, double theta_rad, std::string frame_id = "map", std::string planner_id = "GridBased");
    void request_path_through_poses_nav2(const nav_msgs::msg::Path &input_path, std::string frame_id = "map", std::string planner_id = "GridBased");
    void process_record_route();
    void process_load_waypoints();
    void process_save_waypoints();
    void process_load_terminals();
    void process_save_terminals();
    void cvt_waypoint_t_2path_msgs(nav_msgs::msg::Path *ppath);
    sensor_msgs::msg::PointCloud draw_icr_ackermann(double target_velocity, double target_steering_angle, double wheelbase_, std::string frame_id = "base_link");

    // Motion
    // ===============================================================================================
    float get_emergency_laser_scan(float min_scan_x, float max_scan_x, float min_scan_y, float max_scan_y, float gain = 4.0);
    void clip_sudut_180_min180(float *pangle, bool is_rad = true);
    float cari_rata_rata_kelengkungan_jalan(nav_msgs::msg::Path *ppath, int resolusi);
    void manual_motion(float vx, float vy, float wz);
    void global_navigation_motion();                                             // Ini menghasilkan curr_traj_raw
    void local_trajectory_optimization();                                        // Ini menghasilkan curr_traj_optimized
    void local_traj2velocity_steering(float *pvelocity, float *psteering_angle); // Ini menghasilkan kecepatan dan sudut stir
    void gas_manual_control();
    void steer_manual_control();
    void full_control_mode();
    void set_index_now_2_global(nav_msgs::msg::Path *input, nav_msgs::msg::Path *output, float jarak_minimum, float arah_minimum, size_t berapa_wp_kedepan); // Ini mencari dari titik saya
};

#endif // MASTER_HPP
