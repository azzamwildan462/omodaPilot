// asd
/**
 *
 * Idenya nge transform scan box daripada transform semua lidar point ke frame scan box
 */

#include "nav_msgs/msg/odometry.hpp"
#include "pcl/filters/crop_box.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include "geometry_msgs/msg/point_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

using namespace std::chrono_literals;

class AllObstacleFilter : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_scan_box;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_kanan_points;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_kiri_points;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_tengah_points;

    rclcpp::CallbackGroup::SharedPtr cb_lidar_kanan_points;
    rclcpp::CallbackGroup::SharedPtr cb_lidar_kiri_points;
    rclcpp::CallbackGroup::SharedPtr cb_lidar_tengah_points;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_result_all_obstacle;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_all_pcl2laserscan;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_pcl2laser;

    // Configs
    float scan_box_x_min = 0.0;
    float scan_box_y_min = -0.5;
    float scan_box_x_max = 2.5;
    float scan_box_y_max = 0.5;
    float scan_box_z_min = -1.0;
    float scan_box_z_max = 1.0;
    float pcl2laser_obs_x_min = 0.0;
    float pcl2laser_obs_y_min = -0.5;
    float pcl2laser_obs_x_max = 2.5;
    float pcl2laser_obs_y_max = 0.5;
    float pcl2laser_obs_z_min = -1.0;
    float pcl2laser_obs_z_max = 1.0;
    float exclude_x_min = 0.0;
    float exclude_y_min = -0.5;
    float exclude_x_max = 2.5;
    float exclude_y_max = 0.5;
    float exclude_z_min = -1.0;
    float exclude_z_max = 1.0;
    std::string lidar_kanan_topic = "/lidar_kanan_points";
    std::string lidar_kiri_topic = "/lidar_kiri_points";
    std::string lidar_tengah_topic = "/lidar_tengah_points";
    std::string lidar_kanan_frame_id = "lidar_kanan_link";
    std::string lidar_kiri_frame_id = "lidar_kiri_link";
    std::string lidar_tengah_frame_id = "lidar_tengah_link";

    //----Variables
    float result_lidar_kanan = 0.0;
    float result_lidar_kiri = 0.0;
    float result_lidar_tengah = 0.0;
    pcl::PointCloud<pcl::PointXYZ> pcl2laser_obs_lidar_kiri;
    pcl::PointCloud<pcl::PointXYZ> pcl2laser_obs_lidar_kanan;
    pcl::PointCloud<pcl::PointXYZ> pcl2laser_obs_lidar_tengah;

    geometry_msgs::msg::PointStamped scan_box_kiri_belakang_lidar_kanan;
    geometry_msgs::msg::PointStamped scan_box_kanan_depan_lidar_kanan;
    geometry_msgs::msg::PointStamped scan_box_kiri_belakang_lidar_kiri;
    geometry_msgs::msg::PointStamped scan_box_kanan_depan_lidar_kiri;
    geometry_msgs::msg::PointStamped scan_box_kiri_belakang_lidar_tengah;
    geometry_msgs::msg::PointStamped scan_box_kanan_depan_lidar_tengah;

    geometry_msgs::msg::PointStamped pcl2laser_obs_kiri_belakang_lidar_kanan;
    geometry_msgs::msg::PointStamped pcl2laser_obs_kanan_depan_lidar_kanan;
    geometry_msgs::msg::PointStamped pcl2laser_obs_kiri_belakang_lidar_kiri;
    geometry_msgs::msg::PointStamped pcl2laser_obs_kanan_depan_lidar_kiri;
    geometry_msgs::msg::PointStamped pcl2laser_obs_kiri_belakang_lidar_tengah;
    geometry_msgs::msg::PointStamped pcl2laser_obs_kanan_depan_lidar_tengah;

    geometry_msgs::msg::PointStamped exclude_kiri_belakang_lidar_kanan;
    geometry_msgs::msg::PointStamped exclude_kanan_depan_lidar_kanan;
    geometry_msgs::msg::PointStamped exclude_kiri_belakang_lidar_kiri;
    geometry_msgs::msg::PointStamped exclude_kanan_depan_lidar_kiri;
    geometry_msgs::msg::PointStamped exclude_kiri_belakang_lidar_tengah;
    geometry_msgs::msg::PointStamped exclude_kanan_depan_lidar_tengah;

    // Transform
    // ---------
    std::unique_ptr<tf2_ros::Buffer> tf_base2_lidar_kanan_buffer;
    std::unique_ptr<tf2_ros::Buffer> tf_base2_lidar_kiri_buffer;
    std::unique_ptr<tf2_ros::Buffer> tf_base2_lidar_tengah_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_base2_lidar_kanan_listener;
    std::unique_ptr<tf2_ros::TransformListener> tf_base2_lidar_kiri_listener;
    std::unique_ptr<tf2_ros::TransformListener> tf_base2_lidar_tengah_listener;
    geometry_msgs::msg::TransformStamped tf_base2_lidar_kanan;
    geometry_msgs::msg::TransformStamped tf_base2_lidar_kiri;
    geometry_msgs::msg::TransformStamped tf_base2_lidar_tengah;

    Eigen::Matrix4f T_lidar_kiri2_base_link_4f;
    Eigen::Matrix4f T_lidar_kanan2_base_link_4f;
    Eigen::Matrix4f T_lidar_tengah2_base_link_4f;

    // Last time update
    int timer_routine_period_ms = 40;
    uint64_t wall_time_ms = 0;
    uint64_t last_time_lidar_kiri_update_ms = 0;
    uint64_t last_time_lidar_kanan_update_ms = 0;
    uint64_t last_time_lidar_tengah_update_ms = 0;

    bool is_tf_initialized = false;

    HelpLogger logger;

    AllObstacleFilter()
        : Node("obstacle_filter")
    {
        this->declare_parameter("scan_box_x_min", 0.0);
        this->get_parameter("scan_box_x_min", scan_box_x_min);

        this->declare_parameter("scan_box_x_max", 5.5);
        this->get_parameter("scan_box_x_max", scan_box_x_max);

        this->declare_parameter("scan_box_y_min", -4.5);
        this->get_parameter("scan_box_y_min", scan_box_y_min);

        this->declare_parameter("scan_box_y_max", 4.5);
        this->get_parameter("scan_box_y_max", scan_box_y_max);

        this->declare_parameter("scan_box_z_min", 0.1);
        this->get_parameter("scan_box_z_min", scan_box_z_min);

        this->declare_parameter("scan_box_z_max", 2.0);
        this->get_parameter("scan_box_z_max", scan_box_z_max);

        this->declare_parameter("pcl2laser_obs_x_min", -10.0);
        this->get_parameter("pcl2laser_obs_x_min", pcl2laser_obs_x_min);

        this->declare_parameter("pcl2laser_obs_x_max", 10.0);
        this->get_parameter("pcl2laser_obs_x_max", pcl2laser_obs_x_max);

        this->declare_parameter("pcl2laser_obs_y_min", -5.5);
        this->get_parameter("pcl2laser_obs_y_min", pcl2laser_obs_y_min);

        this->declare_parameter("pcl2laser_obs_y_max", 5.5);
        this->get_parameter("pcl2laser_obs_y_max", pcl2laser_obs_y_max);

        this->declare_parameter("pcl2laser_obs_z_min", 0.1);
        this->get_parameter("pcl2laser_obs_z_min", pcl2laser_obs_z_min);

        this->declare_parameter("pcl2laser_obs_z_max", 2.0);
        this->get_parameter("pcl2laser_obs_z_max", pcl2laser_obs_z_max);

        this->declare_parameter("exclude_x_min", -2.0);
        this->get_parameter("exclude_x_min", exclude_x_min);

        this->declare_parameter("exclude_x_max", 2.0);
        this->get_parameter("exclude_x_max", exclude_x_max);

        this->declare_parameter("exclude_y_min", -1.0);
        this->get_parameter("exclude_y_min", exclude_y_min);

        this->declare_parameter("exclude_y_max", 1.0);
        this->get_parameter("exclude_y_max", exclude_y_max);

        this->declare_parameter("exclude_z_min", 0.1);
        this->get_parameter("exclude_z_min", exclude_z_min);

        this->declare_parameter("exclude_z_max", 2.0);
        this->get_parameter("exclude_z_max", exclude_z_max);

        this->declare_parameter("lidar_kanan_topic", "/lidar_kanan_points");
        this->get_parameter("lidar_kanan_topic", lidar_kanan_topic);

        this->declare_parameter("lidar_kiri_topic", "/lidar_kiri_points");
        this->get_parameter("lidar_kiri_topic", lidar_kiri_topic);

        this->declare_parameter("lidar_tengah_topic", "/camera/rs2_cam_main/depth/color/points");
        this->get_parameter("lidar_tengah_topic", lidar_tengah_topic);

        this->declare_parameter("lidar_kanan_frame_id", "lidar_kanan_link");
        this->get_parameter("lidar_kanan_frame_id", lidar_kanan_frame_id);

        this->declare_parameter("lidar_kiri_frame_id", "lidar_kiri_link");
        this->get_parameter("lidar_kiri_frame_id", lidar_kiri_frame_id);

        this->declare_parameter("lidar_tengah_frame_id", "camera_depth_optical_frame");
        this->get_parameter("lidar_tengah_frame_id", lidar_tengah_frame_id);

        //----Logger
        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        //----Transform listener
        tf_base2_lidar_kanan_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_base2_lidar_kiri_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_base2_lidar_tengah_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_base2_lidar_kanan_listener = std::make_unique<tf2_ros::TransformListener>(*tf_base2_lidar_kanan_buffer, this);
        tf_base2_lidar_kiri_listener = std::make_unique<tf2_ros::TransformListener>(*tf_base2_lidar_kiri_buffer, this);
        tf_base2_lidar_tengah_listener = std::make_unique<tf2_ros::TransformListener>(*tf_base2_lidar_tengah_buffer, this);

        while (!is_tf_initialized)
        {
            rclcpp::sleep_for(1s);
            logger.warn("Waiting for transforms to be available...");
            try
            {
                tf_base2_lidar_kanan = tf_base2_lidar_kanan_buffer->lookupTransform(lidar_kanan_frame_id, "base_link", tf2::TimePointZero);
                tf_base2_lidar_kiri = tf_base2_lidar_kiri_buffer->lookupTransform(lidar_kiri_frame_id, "base_link", tf2::TimePointZero);
                tf_base2_lidar_tengah = tf_base2_lidar_tengah_buffer->lookupTransform(lidar_tengah_frame_id, "base_link", tf2::TimePointZero);
                is_tf_initialized = true;
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
                is_tf_initialized = false;
            }
        }

        // Compute transformation matrices
        T_lidar_kanan2_base_link_4f = tf2::transformToEigen(tf_base2_lidar_kanan).matrix().inverse().cast<float>();
        T_lidar_kiri2_base_link_4f = tf2::transformToEigen(tf_base2_lidar_kiri).matrix().inverse().cast<float>();
        T_lidar_tengah2_base_link_4f = tf2::transformToEigen(tf_base2_lidar_tengah).matrix().inverse().cast<float>();

        //----Callback groups
        cb_lidar_kanan_points = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_lidar_kiri_points = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_lidar_tengah_points = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // HEHE
        geometry_msgs::msg::PointStamped point_exclude_kiri_belakang;
        geometry_msgs::msg::PointStamped point_exclude_kanan_depan;

        point_exclude_kiri_belakang.point.x = exclude_x_min;
        point_exclude_kiri_belakang.point.y = exclude_y_max;
        point_exclude_kiri_belakang.point.z = exclude_z_min;
        point_exclude_kiri_belakang.header.frame_id = "base_link";
        point_exclude_kiri_belakang.header.stamp = this->now();

        point_exclude_kanan_depan.point.x = exclude_x_max;
        point_exclude_kanan_depan.point.y = exclude_y_min;
        point_exclude_kanan_depan.point.z = exclude_z_max;
        point_exclude_kanan_depan.header.frame_id = "base_link";
        point_exclude_kanan_depan.header.stamp = this->now();

        try
        {
            tf2::doTransform(point_exclude_kiri_belakang, exclude_kiri_belakang_lidar_kanan, tf_base2_lidar_kanan);
            tf2::doTransform(point_exclude_kanan_depan, exclude_kanan_depan_lidar_kanan, tf_base2_lidar_kanan);
            tf2::doTransform(point_exclude_kiri_belakang, exclude_kiri_belakang_lidar_kiri, tf_base2_lidar_kiri);
            tf2::doTransform(point_exclude_kanan_depan, exclude_kanan_depan_lidar_kiri, tf_base2_lidar_kiri);
            tf2::doTransform(point_exclude_kiri_belakang, exclude_kiri_belakang_lidar_tengah, tf_base2_lidar_tengah);
            tf2::doTransform(point_exclude_kanan_depan, exclude_kanan_depan_lidar_tengah, tf_base2_lidar_tengah);
        }
        catch (const tf2::TransformException &ex)
        {
            logger.error("Transform PCL2Lasser failed: %s", ex.what());
        }

        auto sub_lidar_kanan_options = rclcpp::SubscriptionOptions();
        sub_lidar_kanan_options.callback_group = cb_lidar_kanan_points;
        auto sub_lidar_kiri_options = rclcpp::SubscriptionOptions();
        sub_lidar_kiri_options.callback_group = cb_lidar_kiri_points;
        auto sub_lidar_tengah_options = rclcpp::SubscriptionOptions();
        sub_lidar_tengah_options.callback_group = cb_lidar_tengah_points;

        //----Subscriber
        sub_lidar_kanan_points = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_kanan_topic, 1, std::bind(&AllObstacleFilter::callback_sub_lidar_kanan_points, this, std::placeholders::_1), sub_lidar_kanan_options);
        sub_lidar_kiri_points = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_kiri_topic, 1, std::bind(&AllObstacleFilter::callback_sub_lidar_kiri_points, this, std::placeholders::_1), sub_lidar_kiri_options);
        sub_lidar_tengah_points = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_tengah_topic, 1, std::bind(&AllObstacleFilter::callback_sub_lidar_tengah_points, this, std::placeholders::_1), sub_lidar_tengah_options);
        sub_scan_box = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/master/obstacle_scan_box", 1, std::bind(&AllObstacleFilter::callback_sub_scan_box, this, std::placeholders::_1));

        //----Publisher
        pub_result_all_obstacle = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/all_obstacle_filter/result_all_obstacle", 1);
        pub_all_pcl2laserscan = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/all_obstacle_filter/all_pcl2laserscan", 1);
        pub_debug_pcl2laser = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/all_obstacle_filter/debug_pcl2laser", 1);

        //----Timer
        tim_routine = this->create_wall_timer(std::chrono::milliseconds(timer_routine_period_ms), std::bind(&AllObstacleFilter::callback_tim_routine, this));

        logger.info("AllObstacleFilter init success");
    }

    // ================================================================================================

    sensor_msgs::msg::LaserScan calc_all_pcl2laserscan(std::string target_frame, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud)
    {
        sensor_msgs::msg::LaserScan laser_scan_msg;

        laser_scan_msg.header.stamp = this->now();
        laser_scan_msg.header.frame_id = target_frame;
        laser_scan_msg.angle_min = -3.14;              // -90 degrees
        laser_scan_msg.angle_max = 3.14;               // 90 degrees
        laser_scan_msg.angle_increment = 0.0174532925; // 1 degree
        laser_scan_msg.range_min = 0.05;
        laser_scan_msg.range_max = 15.0;

        // Hitung jumlah range
        uint32_t num_ranges = static_cast<uint32_t>((laser_scan_msg.angle_max - laser_scan_msg.angle_min) / laser_scan_msg.angle_increment) + 1;

        // Convert pcl ke range + angle
        std::vector<std::vector<float>> angle_ranges;
        angle_ranges.resize(num_ranges);

        // Convert pcl ke range + angle
        for (const auto &point : pcl_cloud.points)
        {
            float range = std::sqrt(point.x * point.x + point.y * point.y);
            if (range < laser_scan_msg.range_min || range > laser_scan_msg.range_max)
            {
                continue; // Skip points outside valid range
            }

            float angle = std::atan2(point.y, point.x);
            if (angle < laser_scan_msg.angle_min || angle > laser_scan_msg.angle_max)
            {
                continue; // Skip points outside valid angle
            }

            uint32_t index = static_cast<uint32_t>((angle - laser_scan_msg.angle_min) / laser_scan_msg.angle_increment);
            if (index >= num_ranges)
            {
                continue; // Safety check
            }
            angle_ranges[index].push_back(range);
        }

        // Iterasi untuk setiap angle, dicari range minimum
        laser_scan_msg.ranges.resize(num_ranges, std::numeric_limits<float>::infinity());
        for (uint32_t i = 0; i < num_ranges; i++)
        {
            if (!angle_ranges[i].empty())
            {
                float min_range = *std::min_element(angle_ranges[i].begin(), angle_ranges[i].end());
                laser_scan_msg.ranges[i] = min_range;
            }
        }

        return laser_scan_msg;
    }

    float hitung_last_update(uint64_t last_time_update_ms)
    {
        float ret = 0.0;

        uint64_t delta_time_ms = wall_time_ms - last_time_update_ms;
        if (delta_time_ms > 2000)
        {
            ret = 0.0;
        }
        else if (delta_time_ms > 1500)
        {
            ret = 0.75;
        }
        else if (delta_time_ms > 1000)
        {
            ret = 0.25;
        }
        else
        {
            ret = 1.0;
        }

        return ret;
    }

    // ================================================================================================

    void callback_sub_scan_box(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 4)
        {
            scan_box_x_min = msg->data[0];
            scan_box_y_min = msg->data[1];
            scan_box_x_max = msg->data[2];
            scan_box_y_max = msg->data[3];
        }
    }

    void callback_sub_lidar_kanan_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!is_tf_initialized)
        {
            logger.warn("Waiting for transforms to be initialized...");
            return;
        }

        static uint16_t counter_pembagi = 0;
        if (counter_pembagi++ <= -1)
        {
            (void)msg;
            return;
        }
        counter_pembagi = 0;

        last_time_lidar_kanan_update_ms = wall_time_ms;

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        float exclude_min_x = fminf(exclude_kiri_belakang_lidar_kanan.point.x, exclude_kanan_depan_lidar_kanan.point.x);
        float exclude_max_x = fmaxf(exclude_kiri_belakang_lidar_kanan.point.x, exclude_kanan_depan_lidar_kanan.point.x);
        float exclude_min_y = fminf(exclude_kiri_belakang_lidar_kanan.point.y, exclude_kanan_depan_lidar_kanan.point.y);
        float exclude_max_y = fmaxf(exclude_kiri_belakang_lidar_kanan.point.y, exclude_kanan_depan_lidar_kanan.point.y);
        float exclude_min_z = fminf(exclude_kiri_belakang_lidar_kanan.point.z, exclude_kanan_depan_lidar_kanan.point.z);
        float exclude_max_z = fmaxf(exclude_kiri_belakang_lidar_kanan.point.z, exclude_kanan_depan_lidar_kanan.point.z);

        pcl::CropBox<pcl::PointXYZ> exclude_crop_box_filter;
        pcl::PointCloud<pcl::PointXYZ> exclude_points_cropped;
        exclude_crop_box_filter.setInputCloud(cloud.makeShared());
        exclude_crop_box_filter.setMin(Eigen::Vector4f(exclude_min_x, exclude_min_y, exclude_min_z, 1));
        exclude_crop_box_filter.setMax(Eigen::Vector4f(exclude_max_x, exclude_max_y, exclude_max_z, 1));
        exclude_crop_box_filter.setNegative(true);
        exclude_crop_box_filter.filter(exclude_points_cropped);

        float pcl2laser_obs_min_x = fminf(pcl2laser_obs_kiri_belakang_lidar_kanan.point.x, pcl2laser_obs_kanan_depan_lidar_kanan.point.x);
        float pcl2laser_obs_max_x = fmaxf(pcl2laser_obs_kiri_belakang_lidar_kanan.point.x, pcl2laser_obs_kanan_depan_lidar_kanan.point.x);
        float pcl2laser_obs_min_y = fminf(pcl2laser_obs_kiri_belakang_lidar_kanan.point.y, pcl2laser_obs_kanan_depan_lidar_kanan.point.y);
        float pcl2laser_obs_max_y = fmaxf(pcl2laser_obs_kiri_belakang_lidar_kanan.point.y, pcl2laser_obs_kanan_depan_lidar_kanan.point.y);
        float pcl2laser_obs_min_z = fminf(pcl2laser_obs_kiri_belakang_lidar_kanan.point.z, pcl2laser_obs_kanan_depan_lidar_kanan.point.z);
        float pcl2laser_obs_max_z = fmaxf(pcl2laser_obs_kiri_belakang_lidar_kanan.point.z, pcl2laser_obs_kanan_depan_lidar_kanan.point.z);

        pcl::CropBox<pcl::PointXYZ> pcl2laser_obs_crop_box_filter;
        pcl2laser_obs_crop_box_filter.setInputCloud(exclude_points_cropped.makeShared());
        pcl2laser_obs_crop_box_filter.setMin(Eigen::Vector4f(pcl2laser_obs_min_x, pcl2laser_obs_min_y, pcl2laser_obs_min_z, 1));
        pcl2laser_obs_crop_box_filter.setMax(Eigen::Vector4f(pcl2laser_obs_max_x, pcl2laser_obs_max_y, pcl2laser_obs_max_z, 1));
        pcl2laser_obs_crop_box_filter.setNegative(false); // Keep points inside the box
        pcl2laser_obs_crop_box_filter.filter(pcl2laser_obs_lidar_kanan);

        float scan_box_min_x = fminf(scan_box_kiri_belakang_lidar_kanan.point.x, scan_box_kanan_depan_lidar_kanan.point.x);
        float scan_box_max_x = fmaxf(scan_box_kiri_belakang_lidar_kanan.point.x, scan_box_kanan_depan_lidar_kanan.point.x);
        float scan_box_min_y = fminf(scan_box_kiri_belakang_lidar_kanan.point.y, scan_box_kanan_depan_lidar_kanan.point.y);
        float scan_box_max_y = fmaxf(scan_box_kiri_belakang_lidar_kanan.point.y, scan_box_kanan_depan_lidar_kanan.point.y);
        float scan_box_min_z = fminf(scan_box_kiri_belakang_lidar_kanan.point.z, scan_box_kanan_depan_lidar_kanan.point.z);
        float scan_box_max_z = fmaxf(scan_box_kiri_belakang_lidar_kanan.point.z, scan_box_kanan_depan_lidar_kanan.point.z);

        pcl::CropBox<pcl::PointXYZ> scan_box_crop_box_filter;
        pcl::PointCloud<pcl::PointXYZ> scan_box_points_cropped;
        scan_box_crop_box_filter.setInputCloud(pcl2laser_obs_lidar_kanan.makeShared());
        scan_box_crop_box_filter.setMin(Eigen::Vector4f(scan_box_min_x, scan_box_min_y, scan_box_min_z, 1));
        scan_box_crop_box_filter.setMax(Eigen::Vector4f(scan_box_max_x, scan_box_max_y, scan_box_max_z, 1));
        scan_box_crop_box_filter.setNegative(false); // Keep points inside the box
        scan_box_crop_box_filter.filter(scan_box_points_cropped);

        result_lidar_kanan = scan_box_points_cropped.size();
    }
    void callback_sub_lidar_kiri_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!is_tf_initialized)
        {
            logger.warn("Waiting for transforms to be initialized...");
            return;
        }

        static uint16_t counter_pembagi = 0;
        if (counter_pembagi++ <= -1)
        {
            (void)msg;
            return;
        }
        counter_pembagi = 0;

        last_time_lidar_kiri_update_ms = wall_time_ms;

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        float exclude_min_x = fminf(exclude_kiri_belakang_lidar_kiri.point.x, exclude_kanan_depan_lidar_kiri.point.x);
        float exclude_max_x = fmaxf(exclude_kiri_belakang_lidar_kiri.point.x, exclude_kanan_depan_lidar_kiri.point.x);
        float exclude_min_y = fminf(exclude_kiri_belakang_lidar_kiri.point.y, exclude_kanan_depan_lidar_kiri.point.y);
        float exclude_max_y = fmaxf(exclude_kiri_belakang_lidar_kiri.point.y, exclude_kanan_depan_lidar_kiri.point.y);
        float exclude_min_z = fminf(exclude_kiri_belakang_lidar_kiri.point.z, exclude_kanan_depan_lidar_kiri.point.z);
        float exclude_max_z = fmaxf(exclude_kiri_belakang_lidar_kiri.point.z, exclude_kanan_depan_lidar_kiri.point.z);

        pcl::CropBox<pcl::PointXYZ> exclude_crop_box_filter;
        pcl::PointCloud<pcl::PointXYZ> exclude_points_cropped;
        exclude_crop_box_filter.setInputCloud(cloud.makeShared());
        exclude_crop_box_filter.setMin(Eigen::Vector4f(exclude_min_x, exclude_min_y, exclude_min_z, 1));
        exclude_crop_box_filter.setMax(Eigen::Vector4f(exclude_max_x, exclude_max_y, exclude_max_z, 1));
        exclude_crop_box_filter.setNegative(true);
        exclude_crop_box_filter.filter(exclude_points_cropped);

        float pcl2laser_obs_min_x = fminf(pcl2laser_obs_kiri_belakang_lidar_kiri.point.x, pcl2laser_obs_kanan_depan_lidar_kiri.point.x);
        float pcl2laser_obs_max_x = fmaxf(pcl2laser_obs_kiri_belakang_lidar_kiri.point.x, pcl2laser_obs_kanan_depan_lidar_kiri.point.x);
        float pcl2laser_obs_min_y = fminf(pcl2laser_obs_kiri_belakang_lidar_kiri.point.y, pcl2laser_obs_kanan_depan_lidar_kiri.point.y);
        float pcl2laser_obs_max_y = fmaxf(pcl2laser_obs_kiri_belakang_lidar_kiri.point.y, pcl2laser_obs_kanan_depan_lidar_kiri.point.y);
        float pcl2laser_obs_min_z = fminf(pcl2laser_obs_kiri_belakang_lidar_kiri.point.z, pcl2laser_obs_kanan_depan_lidar_kiri.point.z);
        float pcl2laser_obs_max_z = fmaxf(pcl2laser_obs_kiri_belakang_lidar_kiri.point.z, pcl2laser_obs_kanan_depan_lidar_kiri.point.z);

        pcl::CropBox<pcl::PointXYZ> pcl2laser_obs_crop_box_filter;
        pcl2laser_obs_crop_box_filter.setInputCloud(exclude_points_cropped.makeShared());
        pcl2laser_obs_crop_box_filter.setMin(Eigen::Vector4f(pcl2laser_obs_min_x, pcl2laser_obs_min_y, pcl2laser_obs_min_z, 1));
        pcl2laser_obs_crop_box_filter.setMax(Eigen::Vector4f(pcl2laser_obs_max_x, pcl2laser_obs_max_y, pcl2laser_obs_max_z, 1));
        pcl2laser_obs_crop_box_filter.setNegative(false); // Keep points inside the box
        pcl2laser_obs_crop_box_filter.filter(pcl2laser_obs_lidar_kiri);

        float scan_box_min_x = fminf(scan_box_kiri_belakang_lidar_kiri.point.x, scan_box_kanan_depan_lidar_kiri.point.x);
        float scan_box_max_x = fmaxf(scan_box_kiri_belakang_lidar_kiri.point.x, scan_box_kanan_depan_lidar_kiri.point.x);
        float scan_box_min_y = fminf(scan_box_kiri_belakang_lidar_kiri.point.y, scan_box_kanan_depan_lidar_kiri.point.y);
        float scan_box_max_y = fmaxf(scan_box_kiri_belakang_lidar_kiri.point.y, scan_box_kanan_depan_lidar_kiri.point.y);
        float scan_box_min_z = fminf(scan_box_kiri_belakang_lidar_kiri.point.z, scan_box_kanan_depan_lidar_kiri.point.z);
        float scan_box_max_z = fmaxf(scan_box_kiri_belakang_lidar_kiri.point.z, scan_box_kanan_depan_lidar_kiri.point.z);

        pcl::CropBox<pcl::PointXYZ> scan_box_crop_box_filter;
        pcl::PointCloud<pcl::PointXYZ> scan_box_points_cropped;
        scan_box_crop_box_filter.setInputCloud(pcl2laser_obs_lidar_kiri.makeShared());
        scan_box_crop_box_filter.setMin(Eigen::Vector4f(scan_box_min_x, scan_box_min_y, scan_box_min_z, 1));
        scan_box_crop_box_filter.setMax(Eigen::Vector4f(scan_box_max_x, scan_box_max_y, scan_box_max_z, 1));
        scan_box_crop_box_filter.setNegative(false); // Keep points inside the box
        scan_box_crop_box_filter.filter(scan_box_points_cropped);

        result_lidar_kiri = scan_box_points_cropped.size();
    }
    void callback_sub_lidar_tengah_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!is_tf_initialized)
        {
            logger.warn("Waiting for transforms to be initialized...");
            return;
        }

        static uint16_t counter_pembagi = 0;
        if (counter_pembagi++ <= -1)
        {
            (void)msg;
            return;
        }
        counter_pembagi = 0;

        last_time_lidar_tengah_update_ms = wall_time_ms;

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        float exclude_min_x = fminf(exclude_kiri_belakang_lidar_tengah.point.x, exclude_kanan_depan_lidar_tengah.point.x);
        float exclude_max_x = fmaxf(exclude_kiri_belakang_lidar_tengah.point.x, exclude_kanan_depan_lidar_tengah.point.x);
        float exclude_min_y = fminf(exclude_kiri_belakang_lidar_tengah.point.y, exclude_kanan_depan_lidar_tengah.point.y);
        float exclude_max_y = fmaxf(exclude_kiri_belakang_lidar_tengah.point.y, exclude_kanan_depan_lidar_tengah.point.y);
        float exclude_min_z = fminf(exclude_kiri_belakang_lidar_tengah.point.z, exclude_kanan_depan_lidar_tengah.point.z);
        float exclude_max_z = fmaxf(exclude_kiri_belakang_lidar_tengah.point.z, exclude_kanan_depan_lidar_tengah.point.z);

        pcl::CropBox<pcl::PointXYZ> exclude_crop_box_filter;
        pcl::PointCloud<pcl::PointXYZ> exclude_points_cropped;
        exclude_crop_box_filter.setInputCloud(cloud.makeShared());
        exclude_crop_box_filter.setMin(Eigen::Vector4f(exclude_min_x, exclude_min_y, exclude_min_z, 1));
        exclude_crop_box_filter.setMax(Eigen::Vector4f(exclude_max_x, exclude_max_y, exclude_max_z, 1));
        exclude_crop_box_filter.setNegative(true);
        exclude_crop_box_filter.filter(exclude_points_cropped);

        float pcl2laser_obs_min_x = fminf(pcl2laser_obs_kiri_belakang_lidar_tengah.point.x, pcl2laser_obs_kanan_depan_lidar_tengah.point.x);
        float pcl2laser_obs_max_x = fmaxf(pcl2laser_obs_kiri_belakang_lidar_tengah.point.x, pcl2laser_obs_kanan_depan_lidar_tengah.point.x);
        float pcl2laser_obs_min_y = fminf(pcl2laser_obs_kiri_belakang_lidar_tengah.point.y, pcl2laser_obs_kanan_depan_lidar_tengah.point.y);
        float pcl2laser_obs_max_y = fmaxf(pcl2laser_obs_kiri_belakang_lidar_tengah.point.y, pcl2laser_obs_kanan_depan_lidar_tengah.point.y);
        float pcl2laser_obs_min_z = fminf(pcl2laser_obs_kiri_belakang_lidar_tengah.point.z, pcl2laser_obs_kanan_depan_lidar_tengah.point.z);
        float pcl2laser_obs_max_z = fmaxf(pcl2laser_obs_kiri_belakang_lidar_tengah.point.z, pcl2laser_obs_kanan_depan_lidar_tengah.point.z);

        pcl::CropBox<pcl::PointXYZ> pcl2laser_obs_crop_box_filter;
        pcl2laser_obs_crop_box_filter.setInputCloud(exclude_points_cropped.makeShared());
        pcl2laser_obs_crop_box_filter.setMin(Eigen::Vector4f(pcl2laser_obs_min_x, pcl2laser_obs_min_y, pcl2laser_obs_min_z, 1));
        pcl2laser_obs_crop_box_filter.setMax(Eigen::Vector4f(pcl2laser_obs_max_x, pcl2laser_obs_max_y, pcl2laser_obs_max_z, 1));
        pcl2laser_obs_crop_box_filter.setNegative(false); // Keep points inside the box
        pcl2laser_obs_crop_box_filter.filter(pcl2laser_obs_lidar_tengah);

        float scan_box_min_x = fminf(scan_box_kiri_belakang_lidar_tengah.point.x, scan_box_kanan_depan_lidar_tengah.point.x);
        float scan_box_max_x = fmaxf(scan_box_kiri_belakang_lidar_tengah.point.x, scan_box_kanan_depan_lidar_tengah.point.x);
        float scan_box_min_y = fminf(scan_box_kiri_belakang_lidar_tengah.point.y, scan_box_kanan_depan_lidar_tengah.point.y);
        float scan_box_max_y = fmaxf(scan_box_kiri_belakang_lidar_tengah.point.y, scan_box_kanan_depan_lidar_tengah.point.y);
        float scan_box_min_z = fminf(scan_box_kiri_belakang_lidar_tengah.point.z, scan_box_kanan_depan_lidar_tengah.point.z);
        float scan_box_max_z = fmaxf(scan_box_kiri_belakang_lidar_tengah.point.z, scan_box_kanan_depan_lidar_tengah.point.z);

        pcl::CropBox<pcl::PointXYZ> scan_box_crop_box_filter;
        pcl::PointCloud<pcl::PointXYZ> scan_box_points_cropped;
        scan_box_crop_box_filter.setInputCloud(pcl2laser_obs_lidar_tengah.makeShared());
        scan_box_crop_box_filter.setMin(Eigen::Vector4f(scan_box_min_x, scan_box_min_y, scan_box_min_z, 1));
        scan_box_crop_box_filter.setMax(Eigen::Vector4f(scan_box_max_x, scan_box_max_y, scan_box_max_z, 1));
        scan_box_crop_box_filter.setNegative(false); // Keep points inside the box
        scan_box_crop_box_filter.filter(scan_box_points_cropped);

        result_lidar_tengah = scan_box_points_cropped.size();
    }

    void callback_tim_routine()
    {
        if (!is_tf_initialized)
        {
            logger.warn("Waiting for transforms to be initialized...");
            return;
        }

        // TF scan box ke lidar links
        // ================================================

        geometry_msgs::msg::PointStamped point_batas_kiri_belakang;
        geometry_msgs::msg::PointStamped point_batas_kanan_depan;

        point_batas_kiri_belakang.point.x = scan_box_x_min;
        point_batas_kiri_belakang.point.y = scan_box_y_max;
        point_batas_kiri_belakang.point.z = scan_box_z_min;
        point_batas_kiri_belakang.header.frame_id = "base_link";
        point_batas_kiri_belakang.header.stamp = this->now();

        point_batas_kanan_depan.point.x = scan_box_x_max;
        point_batas_kanan_depan.point.y = scan_box_y_min;
        point_batas_kanan_depan.point.z = scan_box_z_max;
        point_batas_kanan_depan.header.frame_id = "base_link";
        point_batas_kanan_depan.header.stamp = this->now();

        try
        {
            tf2::doTransform(point_batas_kiri_belakang, scan_box_kiri_belakang_lidar_kanan, tf_base2_lidar_kanan);
            tf2::doTransform(point_batas_kanan_depan, scan_box_kanan_depan_lidar_kanan, tf_base2_lidar_kanan);
            tf2::doTransform(point_batas_kiri_belakang, scan_box_kiri_belakang_lidar_kiri, tf_base2_lidar_kiri);
            tf2::doTransform(point_batas_kanan_depan, scan_box_kanan_depan_lidar_kiri, tf_base2_lidar_kiri);
            tf2::doTransform(point_batas_kiri_belakang, scan_box_kiri_belakang_lidar_tengah, tf_base2_lidar_tengah);
            tf2::doTransform(point_batas_kanan_depan, scan_box_kanan_depan_lidar_tengah, tf_base2_lidar_tengah);
        }
        catch (const tf2::TransformException &ex)
        {
            logger.error("Transform SCANBOX failed: %s", ex.what());
        }

        // TF pcl2laser obs ke lidar links
        // ===============================================

        geometry_msgs::msg::PointStamped point_pcl2laser_obs_kiri_belakang;
        geometry_msgs::msg::PointStamped point_pcl2laser_obs_kanan_depan;

        point_pcl2laser_obs_kiri_belakang.point.x = pcl2laser_obs_x_min;
        point_pcl2laser_obs_kiri_belakang.point.y = pcl2laser_obs_y_max;
        point_pcl2laser_obs_kiri_belakang.point.z = -200.0;
        point_pcl2laser_obs_kiri_belakang.header.frame_id = "base_link";
        point_pcl2laser_obs_kiri_belakang.header.stamp = this->now();

        point_pcl2laser_obs_kanan_depan.point.x = pcl2laser_obs_x_max;
        point_pcl2laser_obs_kanan_depan.point.y = pcl2laser_obs_y_min;
        point_pcl2laser_obs_kanan_depan.point.z = 200.0;
        point_pcl2laser_obs_kanan_depan.header.frame_id = "base_link";
        point_pcl2laser_obs_kanan_depan.header.stamp = this->now();

        try
        {
            tf2::doTransform(point_pcl2laser_obs_kiri_belakang, pcl2laser_obs_kiri_belakang_lidar_kanan, tf_base2_lidar_kanan);
            tf2::doTransform(point_pcl2laser_obs_kanan_depan, pcl2laser_obs_kanan_depan_lidar_kanan, tf_base2_lidar_kanan);
            tf2::doTransform(point_pcl2laser_obs_kiri_belakang, pcl2laser_obs_kiri_belakang_lidar_kiri, tf_base2_lidar_kiri);
            tf2::doTransform(point_pcl2laser_obs_kanan_depan, pcl2laser_obs_kanan_depan_lidar_kiri, tf_base2_lidar_kiri);
            tf2::doTransform(point_pcl2laser_obs_kiri_belakang, pcl2laser_obs_kiri_belakang_lidar_tengah, tf_base2_lidar_tengah);
            tf2::doTransform(point_pcl2laser_obs_kanan_depan, pcl2laser_obs_kanan_depan_lidar_tengah, tf_base2_lidar_tengah);
        }
        catch (const tf2::TransformException &ex)
        {
            logger.error("Transform PCL2Lasser failed: %s", ex.what());
        }

        float last_update_lidar_kiri = hitung_last_update(last_time_lidar_kiri_update_ms);
        float last_update_lidar_kanan = hitung_last_update(last_time_lidar_kanan_update_ms);
        float last_update_lidar_tengah = hitung_last_update(last_time_lidar_tengah_update_ms);
        wall_time_ms += timer_routine_period_ms;

        // Safety ketika lidar gak konek
        // ==============================================

        if (last_update_lidar_kiri < 0.1)
        {
            result_lidar_kiri = 0.0;
            pcl2laser_obs_lidar_kiri.clear();
        }
        if (last_update_lidar_kanan < 0.1)
        {
            result_lidar_kanan = 0.0;
            pcl2laser_obs_lidar_kanan.clear();
        }
        if (last_update_lidar_tengah < 0.1)
        {
            result_lidar_tengah = 0.0;
            pcl2laser_obs_lidar_tengah.clear();
        }

        // Menggabungkan semua lidar dan juga tf ke base_link
        // ===============================================
        sensor_msgs::msg::LaserScan all_obstacle_laserscan;
        pcl::PointCloud<pcl::PointXYZ> all_obstacle_points;
        pcl::PointCloud<pcl::PointXYZ> pcl2laser_obs_lidar_kiri_in_base_link;
        pcl::PointCloud<pcl::PointXYZ> pcl2laser_obs_lidar_kanan_in_base_link;
        pcl::PointCloud<pcl::PointXYZ> pcl2laser_obs_lidar_tengah_in_base_link;

        if (pcl2laser_obs_lidar_kiri.size() > 0)
        {
            pcl::transformPointCloud(pcl2laser_obs_lidar_kiri, pcl2laser_obs_lidar_kiri_in_base_link, T_lidar_kiri2_base_link_4f);
            all_obstacle_points += pcl2laser_obs_lidar_kiri_in_base_link;
        }
        if (pcl2laser_obs_lidar_kanan.size() > 0)
        {
            pcl::transformPointCloud(pcl2laser_obs_lidar_kanan, pcl2laser_obs_lidar_kanan_in_base_link, T_lidar_kanan2_base_link_4f);
            all_obstacle_points += pcl2laser_obs_lidar_kanan_in_base_link;
        }
        if (pcl2laser_obs_lidar_tengah.size() > 0)
        {
            pcl::transformPointCloud(pcl2laser_obs_lidar_tengah, pcl2laser_obs_lidar_tengah_in_base_link, T_lidar_tengah2_base_link_4f);
            all_obstacle_points += pcl2laser_obs_lidar_tengah_in_base_link;
        }

        // Filter Z disini pakai cropbox
        pcl::CropBox<pcl::PointXYZ> crop_box_filter_final;
        pcl::PointCloud<pcl::PointXYZ> final_obstacle_points_cropped;
        crop_box_filter_final.setInputCloud(all_obstacle_points.makeShared());
        crop_box_filter_final.setMin(Eigen::Vector4f(-200.0, -200.0, pcl2laser_obs_z_min, 1));
        crop_box_filter_final.setMax(Eigen::Vector4f(200.0, 200.0, pcl2laser_obs_z_max, 1));
        crop_box_filter_final.setNegative(false);
        crop_box_filter_final.filter(final_obstacle_points_cropped);

        sensor_msgs::msg::PointCloud2 debug_pcl2laser_msg;
        pcl::toROSMsg(final_obstacle_points_cropped, debug_pcl2laser_msg);
        debug_pcl2laser_msg.header.stamp = this->now();
        debug_pcl2laser_msg.header.frame_id = "base_link";
        pub_debug_pcl2laser->publish(debug_pcl2laser_msg);

        if (final_obstacle_points_cropped.size() > 0)
        {
            all_obstacle_laserscan = calc_all_pcl2laserscan("base_link", final_obstacle_points_cropped);
        }

        // Publish result
        // ===============================================

        pub_all_pcl2laserscan->publish(all_obstacle_laserscan);

        std_msgs::msg::Float32MultiArray msg_result_all_obstacle;
        msg_result_all_obstacle.data.push_back(last_update_lidar_kiri);
        msg_result_all_obstacle.data.push_back(last_update_lidar_kanan);
        msg_result_all_obstacle.data.push_back(last_update_lidar_tengah);
        msg_result_all_obstacle.data.push_back(result_lidar_kiri);
        msg_result_all_obstacle.data.push_back(result_lidar_kanan);
        msg_result_all_obstacle.data.push_back(result_lidar_tengah);
        pub_result_all_obstacle->publish(msg_result_all_obstacle);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_all_obstacle_filter = std::make_shared<AllObstacleFilter>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_all_obstacle_filter);
    executor.spin();

    return 0;
}
