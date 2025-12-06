#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

class PoseEstimator : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_error_code;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_encoder_meter;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_encoder;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_current_velocity;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_gyro;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose_offset;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_fb_transmission;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_CAN_eps_encoder;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_filtered;

    //----TransformBroadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_pose_filtered;

    HelpLogger logger;

    // Configs (static)
    // =======================================================
    float encoder_to_meter = 1.0;
    int timer_period = 20; // ms
    bool use_encoder_pulse = true;
    float wheelbase = 1.00;
    int gyro_type = 0;
    float offset_sudut_steering = 0;

    uint16_t encoder[2] = {0, 0};
    uint16_t prev_encoder[2] = {0, 0};
    float gyro = 0;
    float prev_gyro = 0;

    float final_pose_xyo[3] = {0, 0, 0};
    float final_vel_dxdydo[3] = {0, 0, 0};
    float vehicle_speed_ms = 0.0;

    int32_t sensor_left_encoder = 0;
    int32_t sensor_right_encoder = 0;

    rclcpp::Time last_time_gyro_update;
    rclcpp::Time current_time;

    uint8_t fb_transmission = 0;

    float steering_position = 0;
    float d_gyro_steering = 0;

    float pose_filtered[3] = {0, 0, 0};

    float imu_z_velocity = 0;

    float delta_theta_filtered = 0;  // error theta sekarang terhadap filtered
    float delta_vTheta_filtered = 0; // error theta sekarang terhadap filtered

    bool is_gyro_recvd = false;

    //=======================================================
    // Vars
    // =========================================================
    int16_t error_code = 0;

    PoseEstimator()
        : Node("PoseEstimator")
    {
        this->declare_parameter("encoder_to_meter", 1.0);
        this->get_parameter("encoder_to_meter", encoder_to_meter);

        this->declare_parameter("offset_sudut_steering", 0.0);
        this->get_parameter("offset_sudut_steering", offset_sudut_steering);

        this->declare_parameter("gyro_type", 0);
        this->get_parameter("gyro_type", gyro_type);

        this->declare_parameter("wheelbase", 1.0);
        this->get_parameter("wheelbase", wheelbase);

        this->declare_parameter("timer_period", 20);
        this->get_parameter("timer_period", timer_period);

        this->declare_parameter("use_encoder_pulse", true);
        this->get_parameter("use_encoder_pulse", use_encoder_pulse);

        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_broadcaster_pose_filtered = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        //----Timer
        tim_routine = this->create_wall_timer(std::chrono::milliseconds(timer_period), std::bind(&PoseEstimator::callback_routine, this));

        //----Publisher
        pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        pub_error_code = this->create_publisher<std_msgs::msg::Int16>("error_code", 10);
        pub_encoder_meter = this->create_publisher<std_msgs::msg::Float32>("encoder_meter", 1);

        if (use_encoder_pulse)
        {
            sub_encoder = this->create_subscription<std_msgs::msg::Int32>(
                "/can/encoder", 1, std::bind(&PoseEstimator::callback_sub_encoder, this, std::placeholders::_1));
        }
        else
        {
            sub_current_velocity = this->create_subscription<std_msgs::msg::Float32>(
                "/hardware/fb_current_velocity", 1, std::bind(&PoseEstimator::callback_sub_current_velocity, this, std::placeholders::_1));
        }

        sub_gyro = this->create_subscription<sensor_msgs::msg::Imu>(
            "/hardware/imu", 1, std::bind(&PoseEstimator::callback_sub_gyro, this, std::placeholders::_1));
        sub_pose_offset = this->create_subscription<nav_msgs::msg::Odometry>(
            "/master/pose_offset", 1, std::bind(&PoseEstimator::callback_sub_pose_offset, this, std::placeholders::_1));
        sub_fb_transmission = this->create_subscription<std_msgs::msg::UInt8>(
            "/can/fb_transmission", 1, std::bind(&PoseEstimator::callback_sub_fb_transmission, this, std::placeholders::_1));
        sub_odometry_filtered = this->create_subscription<nav_msgs::msg::Odometry>(
            "/slam/odometry/filtered", 1, std::bind(&PoseEstimator::callback_sub_odom_filtered, this, std::placeholders::_1));

        logger.info("PoseEstimator initialized");
    }

    void callback_sub_current_velocity(const std_msgs::msg::Float32::SharedPtr msg)
    {
        vehicle_speed_ms = msg->data;
    }

    void callback_sub_odom_filtered(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        pose_filtered[0] = msg->pose.pose.position.x;
        pose_filtered[1] = msg->pose.pose.position.y;

        // get_angle from quartenion
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        pose_filtered[2] = yaw;
        delta_theta_filtered = pose_filtered[2] - final_pose_xyo[2];

        while (delta_theta_filtered > M_PI)
            delta_theta_filtered -= 2 * M_PI;
        while (delta_theta_filtered < -M_PI)
            delta_theta_filtered += 2 * M_PI;

        float v_yaw = msg->twist.twist.angular.z;
        delta_vTheta_filtered = v_yaw - final_vel_dxdydo[2];

        while (delta_vTheta_filtered > M_PI)
            delta_vTheta_filtered -= 2 * M_PI;
        while (delta_vTheta_filtered < -M_PI)
            delta_vTheta_filtered += 2 * M_PI;
    }

    void callback_sub_CAN_eps_encoder(const std_msgs::msg::Float32::SharedPtr msg)
    {
        steering_position = msg->data + offset_sudut_steering;
    }

    void callback_sub_fb_transmission(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        fb_transmission = msg->data;
    }

    void callback_sub_pose_offset(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        final_pose_xyo[0] = msg->pose.pose.position.x;
        final_pose_xyo[1] = msg->pose.pose.position.y;

        // get_angle from quartenion
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        final_pose_xyo[2] = yaw;
    }

    void callback_sub_encoder(const std_msgs::msg::Int32::SharedPtr msg)
    {
        sensor_left_encoder = msg->data * 0.5;
        sensor_right_encoder = msg->data * 0.5;

        std_msgs::msg::Float32 msg_encoder_meter;

        if (fb_transmission == 5)
            msg_encoder_meter.data = (sensor_left_encoder + sensor_right_encoder) / 2.0 * encoder_to_meter * 25 * -1;
        else
            msg_encoder_meter.data = (sensor_left_encoder + sensor_right_encoder) / 2.0 * encoder_to_meter * 25;

        pub_encoder_meter->publish(msg_encoder_meter);
    }

    void callback_sub_gyro(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // get_angle from quartenion
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        imu_z_velocity = msg->angular_velocity.z;

        gyro = yaw;
        last_time_gyro_update = rclcpp::Clock(RCL_SYSTEM_TIME).now();

        static uint16_t cntr_awal_recvd = 0;
        if (cntr_awal_recvd++ > 50)
        {
            is_gyro_recvd = true;
            cntr_awal_recvd = 50;
        }
    }

    void callback_routine()
    {
        static rclcpp::Time time_old = this->now();
        static rclcpp::Time time_now = this->now();
        time_old = time_now;
        time_now = this->now();

        static const float dt = (float)(1 / (float)timer_period);

        current_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();

        /* Not used */
        // int16_t d_left_encoder = encoder[0] - prev_encoder[0];
        // int16_t d_right_encoder = encoder[1] - prev_encoder[1];
        //======================================================

        float d_gyro = gyro - prev_gyro;

        rclcpp::Duration dt_gyro = current_time - last_time_gyro_update;
        static rclcpp::Duration prev_dt_gyro = dt_gyro;
        if ((prev_dt_gyro.seconds() > 0.5 && dt_gyro.seconds() <= 0.5) || !is_gyro_recvd)
        {
            logger.warn("Gyro restarted");
            d_gyro = 0;
        }
        prev_dt_gyro = dt_gyro;

        memcpy(prev_encoder, encoder, sizeof(prev_encoder));
        prev_gyro = gyro;

        while (d_gyro > M_PI)
            d_gyro -= 2 * M_PI;
        while (d_gyro < -M_PI)
            d_gyro += 2 * M_PI;

        // logger.info("%.2f %.2f %.2f -> %.2f %.2f %f", final_pose_xyo[0], final_pose_xyo[1], final_pose_xyo[2], vehicle_speed_ms, encoder_to_meter, dt);

        if (use_encoder_pulse)
        {
            d_gyro_steering = (sensor_left_encoder + sensor_right_encoder) / 2.0 * encoder_to_meter * tanf(steering_position) / wheelbase;
            final_vel_dxdydo[0] = (sensor_left_encoder + sensor_right_encoder) / 2.0 * cosf(final_pose_xyo[2]) * encoder_to_meter * dt;
            final_vel_dxdydo[1] = (sensor_left_encoder + sensor_right_encoder) / 2.0 * sinf(final_pose_xyo[2]) * encoder_to_meter * dt;
        }
        else
        {
            d_gyro_steering = vehicle_speed_ms * encoder_to_meter * tanf(steering_position) / wheelbase;
            final_vel_dxdydo[0] = vehicle_speed_ms * cosf(final_pose_xyo[2]) * encoder_to_meter * dt;
            final_vel_dxdydo[1] = vehicle_speed_ms * sinf(final_pose_xyo[2]) * encoder_to_meter * dt;
        }

        if (gyro_type == 0)
            final_vel_dxdydo[2] = d_gyro;
        else if (gyro_type == 1)
            final_vel_dxdydo[2] = d_gyro_steering;

        while (final_vel_dxdydo[2] > M_PI)
            final_vel_dxdydo[2] -= 2 * M_PI;
        while (final_vel_dxdydo[2] < -M_PI)
            final_vel_dxdydo[2] += 2 * M_PI;

        final_pose_xyo[0] += final_vel_dxdydo[0];
        final_pose_xyo[1] += final_vel_dxdydo[1];
        final_pose_xyo[2] += final_vel_dxdydo[2];

        while (final_pose_xyo[2] > M_PI)
            final_pose_xyo[2] -= 2 * M_PI;
        while (final_pose_xyo[2] < -M_PI)
            final_pose_xyo[2] += 2 * M_PI;

        logger.info("POse: %.2f %.2f %.2f || %.2f", final_pose_xyo[0], final_pose_xyo[1], final_pose_xyo[2], gyro);

        tf2::Quaternion q;
        q.setRPY(0, 0, final_pose_xyo[2]);

        nav_msgs::msg::Odometry msg_odom;
        msg_odom.header.stamp = time_now;
        msg_odom.header.frame_id = "odom";
        msg_odom.child_frame_id = "base_link";
        msg_odom.pose.pose.position.x = final_pose_xyo[0];
        msg_odom.pose.pose.position.y = final_pose_xyo[1];
        msg_odom.pose.pose.orientation.x = q.x();
        msg_odom.pose.pose.orientation.y = q.y();
        msg_odom.pose.pose.orientation.z = q.z();
        msg_odom.pose.pose.orientation.w = q.w();
        msg_odom.pose.covariance[0] = 1e-2;
        msg_odom.pose.covariance[7] = 1e-2;
        msg_odom.pose.covariance[14] = 1e6;
        msg_odom.pose.covariance[21] = 1e6;
        msg_odom.pose.covariance[28] = 1e6;
        msg_odom.pose.covariance[35] = 1e-2;
        msg_odom.twist.twist.linear.x = final_vel_dxdydo[0] / dt;
        msg_odom.twist.twist.linear.y = final_vel_dxdydo[1] / dt;
        msg_odom.twist.twist.angular.z = final_vel_dxdydo[2] / dt;
        msg_odom.twist.covariance[0] = 1e-2;
        msg_odom.twist.covariance[7] = 1e-2;
        msg_odom.twist.covariance[14] = 1e6;
        msg_odom.twist.covariance[21] = 1e6;
        msg_odom.twist.covariance[28] = 1e6;
        msg_odom.twist.covariance[35] = 1e-2;
        pub_odom->publish(msg_odom);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = time_now;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = final_pose_xyo[0];
        tf.transform.translation.y = final_pose_xyo[1];
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        tf_broadcaster->sendTransform(tf);

        std_msgs::msg::Int16 msg_error_code;
        msg_error_code.data = error_code;
        pub_error_code->publish(msg_error_code);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_pose_estimator = std::make_shared<PoseEstimator>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_pose_estimator);
    executor.spin();

    return 0;
}