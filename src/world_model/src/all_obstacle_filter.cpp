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
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
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
#include <future>
#include <boost/thread/mutex.hpp>
#include <opencv2/opencv.hpp>

#include <nanoflann.hpp>
#ifdef _OPENMP
#include <omp.h>
#endif
#include "world_model/fast_normals_nanoflann.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using KDTree = FastNormalEstimator::KDTree;

class AllObstacleFilter : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_kanan_points;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_kiri_points;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_tengah_points;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_caminfo_kamera_dalam;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_caminfo_kamera_tengah;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_caminfo_kamera_kiri;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_caminfo_kamera_kanan;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_roadseg_mask;

    rclcpp::CallbackGroup::SharedPtr cb_lidar_kanan_points;
    rclcpp::CallbackGroup::SharedPtr cb_lidar_kiri_points;
    rclcpp::CallbackGroup::SharedPtr cb_lidar_tengah_points;
    rclcpp::CallbackGroup::SharedPtr cb_sub_roadseg;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_result_all_obstacle;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_all_pcl2laserscan;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_pcl2laser;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_pcl2cam_dalam;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_pcl2cam_tengah;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_pcl2cam_kiri;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_pcl2cam_kanan;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_pcl2cam_dalam_depth_color;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_pcl2cam_tengah_depth_color;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_pcl2cam_kiri_depth_color;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_pcl2cam_kanan_depth_color;

    // Configs
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
    std::string camera_dalam_topic = "";
    std::string camera_tengah_topic = "";
    std::string camera_kiri_topic = "";
    std::string camera_kanan_topic = "";
    std::string roadseg_topic = "";
    std::string lidar_kanan_frame_id = "lidar_kanan_link";
    std::string lidar_kiri_frame_id = "lidar_kiri_link";
    std::string lidar_tengah_frame_id = "lidar_tengah_link";
    std::string camera_dalam_frame_id = "";
    std::string camera_tengah_frame_id = "";
    std::string camera_kiri_frame_id = "";
    std::string camera_kanan_frame_id = "";
    bool depth_cam_pub_color_dbg = true;
    bool using_normal_to_laserscan = false;
    bool hitung_roadseg2laserscan = false;
    float threshold_delta_z = 0.3;
    float max_range_gap = 0.9;

    //----Variables
    float result_lidar_kanan = 0.0;
    float result_lidar_kiri = 0.0;
    float result_lidar_tengah = 0.0;
    pcl::PointCloud<pcl::PointXYZ> pcl2laser_obs_lidar_kiri;
    pcl::PointCloud<pcl::PointXYZ> pcl2laser_obs_lidar_kanan;
    pcl::PointCloud<pcl::PointXYZ> pcl2laser_obs_lidar_tengah;
    pcl::PointCloud<pcl::PointXYZ> pcl_lidar_kiri_raw;
    pcl::PointCloud<pcl::PointXYZ> pcl_lidar_kanan_raw;
    pcl::PointCloud<pcl::PointXYZ> pcl_lidar_tengah_raw;

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
    std::unique_ptr<tf2_ros::Buffer> tf_base2_camera_dalam_buffer;
    std::unique_ptr<tf2_ros::Buffer> tf_base2_camera_tengah_buffer;
    std::unique_ptr<tf2_ros::Buffer> tf_base2_camera_kiri_buffer;
    std::unique_ptr<tf2_ros::Buffer> tf_base2_camera_kanan_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_base2_lidar_kanan_listener;
    std::unique_ptr<tf2_ros::TransformListener> tf_base2_lidar_kiri_listener;
    std::unique_ptr<tf2_ros::TransformListener> tf_base2_lidar_tengah_listener;
    std::unique_ptr<tf2_ros::TransformListener> tf_base2_camera_dalam_listener;
    std::unique_ptr<tf2_ros::TransformListener> tf_base2_camera_tengah_listener;
    std::unique_ptr<tf2_ros::TransformListener> tf_base2_camera_kiri_listener;
    std::unique_ptr<tf2_ros::TransformListener> tf_base2_camera_kanan_listener;
    geometry_msgs::msg::TransformStamped tf_base2_lidar_kanan;
    geometry_msgs::msg::TransformStamped tf_base2_lidar_kiri;
    geometry_msgs::msg::TransformStamped tf_base2_lidar_tengah;
    geometry_msgs::msg::TransformStamped tf_base2_camera_dalam;
    geometry_msgs::msg::TransformStamped tf_base2_camera_tengah;
    geometry_msgs::msg::TransformStamped tf_base2_camera_kiri;
    geometry_msgs::msg::TransformStamped tf_base2_camera_kanan;

    Eigen::Matrix4f T_lidar_kiri2_base_link_4f;
    Eigen::Matrix4f T_lidar_kanan2_base_link_4f;
    Eigen::Matrix4f T_lidar_tengah2_base_link_4f;
    Eigen::Matrix4f T_base_link2_camera_dalam_4f;
    Eigen::Matrix4f T_base_link2_camera_tengah_4f;
    Eigen::Matrix4f T_base_link2_camera_kiri_4f;
    Eigen::Matrix4f T_base_link2_camera_kanan_4f;

    // Last time update
    int timer_routine_period_ms = 40;
    uint64_t wall_time_ms = 0;
    uint64_t last_time_lidar_kiri_update_ms = 0;
    uint64_t last_time_lidar_kanan_update_ms = 0;
    uint64_t last_time_lidar_tengah_update_ms = 0;

    bool is_tf_initialized = false;

    HelpLogger logger;

    boost::mutex mtx_lidar_kanan;
    boost::mutex mtx_lidar_kiri;
    boost::mutex mtx_lidar_tengah;

    rclcpp::Time last_stamp_lidar_tengah;

    // Tambahan
    // ---------
    sensor_msgs::msg::CameraInfo::SharedPtr caminfo_kamera_dalam;
    sensor_msgs::msg::CameraInfo::SharedPtr caminfo_kamera_tengah;
    sensor_msgs::msg::CameraInfo::SharedPtr caminfo_kamera_kiri;
    sensor_msgs::msg::CameraInfo::SharedPtr caminfo_kamera_kanan;

    AllObstacleFilter()
        : Node("obstacle_filter")
    {
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

        this->declare_parameter("camera_dalam_topic", "");
        this->get_parameter("camera_dalam_topic", camera_dalam_topic);

        this->declare_parameter("camera_tengah_topic", "");
        this->get_parameter("camera_tengah_topic", camera_tengah_topic);

        this->declare_parameter("camera_kiri_topic", "");
        this->get_parameter("camera_kiri_topic", camera_kiri_topic);

        this->declare_parameter("camera_kanan_topic", "");
        this->get_parameter("camera_kanan_topic", camera_kanan_topic);

        this->declare_parameter("lidar_kanan_frame_id", "lidar_kanan_link");
        this->get_parameter("lidar_kanan_frame_id", lidar_kanan_frame_id);

        this->declare_parameter("lidar_kiri_frame_id", "lidar_kiri_link");
        this->get_parameter("lidar_kiri_frame_id", lidar_kiri_frame_id);

        this->declare_parameter("lidar_tengah_frame_id", "camera_depth_optical_frame");
        this->get_parameter("lidar_tengah_frame_id", lidar_tengah_frame_id);

        this->declare_parameter("camera_dalam_frame_id", "");
        this->get_parameter("camera_dalam_frame_id", camera_dalam_frame_id);

        this->declare_parameter("camera_tengah_frame_id", "");
        this->get_parameter("camera_tengah_frame_id", camera_tengah_frame_id);

        this->declare_parameter("camera_kiri_frame_id", "");
        this->get_parameter("camera_kiri_frame_id", camera_kiri_frame_id);

        this->declare_parameter("camera_kanan_frame_id", "");
        this->get_parameter("camera_kanan_frame_id", camera_kanan_frame_id);

        this->declare_parameter("depth_cam_pub_color_dbg", true);
        this->get_parameter("depth_cam_pub_color_dbg", depth_cam_pub_color_dbg);

        this->declare_parameter("using_normal_to_laserscan", false);
        this->get_parameter("using_normal_to_laserscan", using_normal_to_laserscan);

        this->declare_parameter("hitung_roadseg2laserscan", false);
        this->get_parameter("hitung_roadseg2laserscan", hitung_roadseg2laserscan);

        this->declare_parameter("roadseg_topic", "");
        this->get_parameter("roadseg_topic", roadseg_topic);

        this->declare_parameter("threshold_delta_z", 0.3);
        this->get_parameter("threshold_delta_z", threshold_delta_z);

        this->declare_parameter("max_range_gap", 0.9);
        this->get_parameter("max_range_gap", max_range_gap);

        //----Logger
        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

#ifdef _OPENMP
        logger.info("OpenMP enabled, threads=%d", omp_get_max_threads());
#else
        logger.warn("OpenMP not enabled");
#endif

        init_sub_cam_info();

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

        init_tf_cam_info();

        // Compute transformation matrices
        T_lidar_kanan2_base_link_4f = tf2::transformToEigen(tf_base2_lidar_kanan).matrix().inverse().cast<float>();
        T_lidar_kiri2_base_link_4f = tf2::transformToEigen(tf_base2_lidar_kiri).matrix().inverse().cast<float>();
        T_lidar_tengah2_base_link_4f = tf2::transformToEigen(tf_base2_lidar_tengah).matrix().inverse().cast<float>();

        //----Callback groups
        cb_lidar_kanan_points = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_lidar_kiri_points = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_lidar_tengah_points = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_sub_roadseg = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // HEHE
        geometry_msgs::msg::PointStamped point_exclude_kiri_belakang;
        geometry_msgs::msg::PointStamped point_exclude_kanan_depan;

        auto sub_lidar_kanan_options = rclcpp::SubscriptionOptions();
        sub_lidar_kanan_options.callback_group = cb_lidar_kanan_points;
        auto sub_lidar_kiri_options = rclcpp::SubscriptionOptions();
        sub_lidar_kiri_options.callback_group = cb_lidar_kiri_points;
        auto sub_lidar_tengah_options = rclcpp::SubscriptionOptions();
        sub_lidar_tengah_options.callback_group = cb_lidar_tengah_points;
        auto sub_roadseg_options = rclcpp::SubscriptionOptions();
        sub_roadseg_options.callback_group = cb_sub_roadseg;

        //----Subscriber
        sub_lidar_kanan_points = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_kanan_topic, 1, std::bind(&AllObstacleFilter::callback_sub_lidar_kanan_points, this, std::placeholders::_1), sub_lidar_kanan_options);
        sub_lidar_kiri_points = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_kiri_topic, 1, std::bind(&AllObstacleFilter::callback_sub_lidar_kiri_points, this, std::placeholders::_1), sub_lidar_kiri_options);
        sub_lidar_tengah_points = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_tengah_topic, 1, std::bind(&AllObstacleFilter::callback_sub_lidar_tengah_points, this, std::placeholders::_1), sub_lidar_tengah_options);

        if (hitung_roadseg2laserscan)
        {
            sub_roadseg_mask = this->create_subscription<sensor_msgs::msg::Image>(
                roadseg_topic, 1, std::bind(&AllObstacleFilter::callback_sub_roadseg_mask, this, std::placeholders::_1), sub_roadseg_options);
        }

        //----Publisher
        pub_result_all_obstacle = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/all_obstacle_filter/result_all_obstacle", 1);
        pub_all_pcl2laserscan = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/all_obstacle_filter/all_pcl2laserscan", 1);
        pub_debug_pcl2laser = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/all_obstacle_filter/debug_pcl2laser", 1);

        init_pub_cam_info();

        //----Timer
        tim_routine = this->create_wall_timer(std::chrono::milliseconds(timer_routine_period_ms), std::bind(&AllObstacleFilter::callback_tim_routine, this));

        logger.info("AllObstacleFilter init success");
    }

    void init_pub_cam_info()
    {
        if (camera_dalam_topic != "")
        {
            pub_pcl2cam_dalam = this->create_publisher<sensor_msgs::msg::Image>(
                "/all_obstacle_filter/pcl2cam_dalam", 1);
            pub_pcl2cam_dalam_depth_color = this->create_publisher<sensor_msgs::msg::Image>(
                "/all_obstacle_filter/pcl2cam_dalam_depth_color", 1);
        }
        if (camera_tengah_topic != "")
        {
            pub_pcl2cam_tengah = this->create_publisher<sensor_msgs::msg::Image>(
                "/all_obstacle_filter/pcl2cam_tengah", 1);
            pub_pcl2cam_tengah_depth_color = this->create_publisher<sensor_msgs::msg::Image>(
                "/all_obstacle_filter/pcl2cam_tengah_depth_color", 1);
        }
        if (camera_kiri_topic != "")
        {
            pub_pcl2cam_kiri = this->create_publisher<sensor_msgs::msg::Image>(
                "/all_obstacle_filter/pcl2cam_kiri", 1);
            pub_pcl2cam_kiri_depth_color = this->create_publisher<sensor_msgs::msg::Image>(
                "/all_obstacle_filter/pcl2cam_kiri_depth_color", 1);
        }
        if (camera_kanan_topic != "")
        {
            pub_pcl2cam_kanan = this->create_publisher<sensor_msgs::msg::Image>(
                "/all_obstacle_filter/pcl2cam_kanan", 1);
            pub_pcl2cam_kanan_depth_color = this->create_publisher<sensor_msgs::msg::Image>(
                "/all_obstacle_filter/pcl2cam_kanan_depth_color", 1);
        }
    }

    void init_tf_cam_info()
    {
        bool tf_cam_info_initialized = false;

        if (camera_dalam_frame_id != "")
        {
            tf_base2_camera_dalam_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_base2_camera_dalam_listener = std::make_unique<tf2_ros::TransformListener>(*tf_base2_camera_dalam_buffer, this);
        }
        if (camera_tengah_frame_id != "")
        {
            tf_base2_camera_tengah_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_base2_camera_tengah_listener = std::make_unique<tf2_ros::TransformListener>(*tf_base2_camera_tengah_buffer, this);
        }
        if (camera_kiri_frame_id != "")
        {
            tf_base2_camera_kiri_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_base2_camera_kiri_listener = std::make_unique<tf2_ros::TransformListener>(*tf_base2_camera_kiri_buffer, this);
        }
        if (camera_kanan_frame_id != "")
        {
            tf_base2_camera_kanan_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_base2_camera_kanan_listener = std::make_unique<tf2_ros::TransformListener>(*tf_base2_camera_kanan_buffer, this);
        }

        while (!tf_cam_info_initialized)
        {
            rclcpp::sleep_for(1s);
            logger.warn("Waiting for camera transforms to be available...");
            try
            {
                if (camera_dalam_frame_id != "")
                {
                    tf_base2_camera_dalam = tf_base2_camera_dalam_buffer->lookupTransform(camera_dalam_frame_id, "base_link", tf2::TimePointZero);
                }
                if (camera_tengah_frame_id != "")
                {
                    tf_base2_camera_tengah = tf_base2_camera_tengah_buffer->lookupTransform(camera_tengah_frame_id, "base_link", tf2::TimePointZero);
                }
                if (camera_kiri_frame_id != "")
                {
                    tf_base2_camera_kiri = tf_base2_camera_kiri_buffer->lookupTransform(camera_kiri_frame_id, "base_link", tf2::TimePointZero);
                }
                if (camera_kanan_frame_id != "")
                {
                    tf_base2_camera_kanan = tf_base2_camera_kanan_buffer->lookupTransform(camera_kanan_frame_id, "base_link", tf2::TimePointZero);
                }
                tf_cam_info_initialized = true;
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to get camera transform: %s", ex.what());
                tf_cam_info_initialized = false;
            }
        }

        // Compute transformation matrices
        T_base_link2_camera_dalam_4f = tf2::transformToEigen(tf_base2_camera_dalam).matrix().cast<float>();
        T_base_link2_camera_tengah_4f = tf2::transformToEigen(tf_base2_camera_tengah).matrix().cast<float>();
        T_base_link2_camera_kiri_4f = tf2::transformToEigen(tf_base2_camera_kiri).matrix().cast<float>();
        T_base_link2_camera_kanan_4f = tf2::transformToEigen(tf_base2_camera_kanan).matrix().cast<float>();
    }

    void init_sub_cam_info()
    {
        if (camera_dalam_topic != "")
        {
            sub_caminfo_kamera_dalam = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                camera_dalam_topic, 1, [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg)
                {
                    static bool is_first = true;
                    if (is_first){
                        caminfo_kamera_dalam = msg;
                        logger.info("Received first CameraInfo from camera_dalam_topic");
                        is_first = false;

                        if (camera_dalam_frame_id == ""){
                            camera_dalam_frame_id = caminfo_kamera_dalam->header.frame_id;
                        }
                    } });
        }
        if (camera_tengah_topic != "")
        {
            sub_caminfo_kamera_tengah = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                camera_tengah_topic, 1, [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg)
                {
                    static bool is_first = true;
                    if (is_first){
                        caminfo_kamera_tengah = msg;
                        logger.info("Received first CameraInfo from camera_tengah_topic");
                        is_first = false;

                        if (camera_tengah_frame_id == ""){
                            camera_tengah_frame_id = caminfo_kamera_tengah->header.frame_id;
                        }
                    } });
        }
        if (camera_kiri_topic != "")
        {
            sub_caminfo_kamera_kiri = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                camera_kiri_topic, 1, [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg)
                {
                    static bool is_first = true;
                    if (is_first){
                        caminfo_kamera_kiri = msg;
                        logger.info("Received first CameraInfo from camera_kiri_topic");
                        is_first = false;

                        if (camera_kiri_frame_id == ""){
                            camera_kiri_frame_id = caminfo_kamera_kiri->header.frame_id;
                        }
                    } });
        }
        if (camera_kanan_topic != "")
        {
            sub_caminfo_kamera_kanan = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                camera_kanan_topic, 1, [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg)
                {
                    static bool is_first = true;
                    if (is_first){
                        caminfo_kamera_kanan = msg;
                        logger.info("Received first CameraInfo from camera_kanan_topic");
                        is_first = false;

                        if (camera_kanan_frame_id == ""){
                            camera_kanan_frame_id = caminfo_kamera_kanan->header.frame_id;
                        }
                    } });
        }

        uint8_t status_valid = 0;
        while (status_valid != 0b1111)
        {
            rclcpp::sleep_for(1s);
            logger.warn("Waiting for CameraInfo topics to be available... %02X", status_valid);

            // Jika sudah dapat dari callback, skip
            if (camera_dalam_topic != "" && camera_dalam_frame_id != "")
            {
                status_valid |= 0b01;
            }
            else if (camera_dalam_topic == "")
            {
                status_valid |= 0b01;
            }

            if (camera_tengah_topic != "" && camera_tengah_frame_id != "")
            {
                status_valid |= 0b10;
            }
            else if (camera_tengah_topic == "")
            {
                status_valid |= 0b10;
            }

            if (camera_kiri_topic != "" && camera_kiri_frame_id != "")
            {
                status_valid |= 0b100;
            }
            else if (camera_kiri_topic == "")
            {
                status_valid |= 0b100;
            }

            if (camera_kanan_topic != "" && camera_kanan_frame_id != "")
            {
                status_valid |= 0b1000;
            }
            else if (camera_kanan_topic == "")
            {
                status_valid |= 0b1000;
            }
        }
    }

    void process_depth_images(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_lidar)
    {
        rclcpp::Time stamp = last_stamp_lidar_tengah;
        // Persetan dengan mutex
        rclcpp::Time time_camera_dalam = stamp;
        rclcpp::Time time_camera_tengah = stamp;
        rclcpp::Time time_camera_kiri = stamp;
        rclcpp::Time time_camera_kanan = stamp;

        if (camera_dalam_topic != "" && caminfo_kamera_dalam)
        {
            std::async(
                std::launch::async,
                [this,
                 cloud_lidar,
                 caminfo = caminfo_kamera_dalam,
                 T = T_base_link2_camera_dalam_4f,
                 pub = pub_pcl2cam_dalam,
                 pub_color = pub_pcl2cam_dalam_depth_color,
                 stamp = time_camera_dalam]()
                {
                    publish_pcl2cam(
                        cloud_lidar,
                        caminfo,
                        T,
                        pub,
                        pub_color,
                        stamp);
                });
        }
        if (camera_tengah_topic != "" && caminfo_kamera_tengah)
        {
            std::async(
                std::launch::async,
                [this,
                 cloud_lidar,
                 caminfo = caminfo_kamera_tengah,
                 T = T_base_link2_camera_tengah_4f,
                 pub = pub_pcl2cam_tengah,
                 pub_color = pub_pcl2cam_tengah_depth_color,
                 stamp = time_camera_tengah]()
                {
                    publish_pcl2cam(
                        cloud_lidar,
                        caminfo,
                        T,
                        pub,
                        pub_color,
                        stamp);
                });
        }
        if (camera_kiri_topic != "" && caminfo_kamera_kiri)
        {
            std::async(
                std::launch::async,
                [this,
                 cloud_lidar,
                 caminfo = caminfo_kamera_kiri,
                 T = T_base_link2_camera_kiri_4f,
                 pub = pub_pcl2cam_kiri,
                 pub_color = pub_pcl2cam_kiri_depth_color,
                 stamp = time_camera_kiri]()
                {
                    publish_pcl2cam(
                        cloud_lidar,
                        caminfo,
                        T,
                        pub,
                        pub_color,
                        stamp);
                });
        }
        if (camera_kanan_topic != "" && caminfo_kamera_kanan)
        {
            std::async(
                std::launch::async,
                [this,
                 cloud_lidar,
                 caminfo = caminfo_kamera_kanan,
                 T = T_base_link2_camera_kanan_4f,
                 pub = pub_pcl2cam_kanan,
                 pub_color = pub_pcl2cam_kanan_depth_color,
                 stamp = time_camera_kanan]()
                {
                    publish_pcl2cam(
                        cloud_lidar,
                        caminfo,
                        T,
                        pub,
                        pub_color,
                        stamp);
                });
        }
    }

    void colormap_jet(float t, uint8_t &r, uint8_t &g, uint8_t &b)
    {
        t = std::clamp(t, 0.0f, 1.0f);

        float r_f = 0.0f, g_f = 0.0f, b_f = 0.0f;

        // Biru -> cyan -> hijau -> kuning -> merah
        if (t < 0.25f)
        {
            // Biru (1,0,0.5) ke cyan (1,1,0)
            float k = t / 0.25f;
            r_f = 0.0f;
            g_f = k;
            b_f = 1.0f;
        }
        else if (t < 0.5f)
        {
            // Cyan ke hijau
            float k = (t - 0.25f) / 0.25f;
            r_f = 0.0f;
            g_f = 1.0f;
            b_f = 1.0f - k;
        }
        else if (t < 0.75f)
        {
            // Hijau ke kuning
            float k = (t - 0.5f) / 0.25f;
            r_f = k;
            g_f = 1.0f;
            b_f = 0.0f;
        }
        else
        {
            // Kuning ke merah
            float k = (t - 0.75f) / 0.25f;
            r_f = 1.0f;
            g_f = 1.0f - k;
            b_f = 0.0f;
        }

        r = static_cast<uint8_t>(std::round(r_f * 255.0f));
        g = static_cast<uint8_t>(std::round(g_f * 255.0f));
        b = static_cast<uint8_t>(std::round(b_f * 255.0f));
    }

    void depth32fc1_to_rgb8(
        const std::vector<float> &depth,
        int width,
        int height,
        float z_min_vis,
        float z_max_vis,
        const std::string &frame_id,
        const rclcpp::Time &stamp,
        sensor_msgs::msg::Image &out_rgb)
    {
        if (width <= 0 || height <= 0)
        {
            return;
        }
        if (static_cast<int>(depth.size()) != width * height)
        {
            return;
        }

        // Siapkan msg
        out_rgb.header.frame_id = frame_id;
        out_rgb.header.stamp = stamp;
        out_rgb.height = height;
        out_rgb.width = width;
        out_rgb.encoding = "rgb8";
        out_rgb.is_bigendian = false;
        out_rgb.step = width * 3;
        out_rgb.data.resize(static_cast<size_t>(width * height * 3));

        const float eps = 1e-6f;
        float range = std::max(z_max_vis - z_min_vis, eps);

        auto idx = [width](int u, int v)
        {
            return v * width + u;
        };

        for (int v = 0; v < height; ++v)
        {
            for (int u = 0; u < width; ++u)
            {
                int i = idx(u, v);
                float d = depth[i];

                uint8_t r = 0, g = 0, b = 0;

                if (d > 0.0f)
                {
                    // clamp ke [z_min_vis, z_max_vis]
                    float dd = std::clamp(d, z_min_vis, z_max_vis);
                    // 0 (dekat) -> 1 (jauh)
                    float t = (dd - z_min_vis) / range;
                    colormap_jet(t, r, g, b);
                }
                else
                {
                    // d == 0.0 -> no data, biarkan hitam
                    r = g = b = 0;
                }

                size_t base = static_cast<size_t>(i) * 3;
                out_rgb.data[base + 0] = r;
                out_rgb.data[base + 1] = g;
                out_rgb.data[base + 2] = b;
            }
        }
    }

    void postfilter_depth_outdoor(
        std::vector<float> &depth,
        int width,
        int height,
        int min_neighbors_keep = 1, // 1 saja biar jarang dibuang
        int fill_iters = 3,         // 3x iterasi 3x3
        int max_radius = 1          // diabaikan, tetap 3x3
    )
    {
        if (width <= 0 || height <= 0)
            return;

        auto idx = [width](int u, int v)
        { return v * width + u; };

        // ---------- 1) Remove pixel yang benar-benar sendirian (3x3) ----------
        if (min_neighbors_keep > 0)
        {
            std::vector<float> tmp = depth;
            for (int v = 1; v < height - 1; ++v)
            {
                for (int u = 1; u < width - 1; ++u)
                {
                    float d = tmp[idx(u, v)];
                    if (d <= 0.0f)
                        continue;

                    int count = 0;
                    for (int dv = -1; dv <= 1; ++dv)
                    {
                        for (int du = -1; du <= 1; ++du)
                        {
                            if (du == 0 && dv == 0)
                                continue;
                            float dn = tmp[idx(u + du, v + dv)];
                            if (dn > 0.0f)
                                ++count;
                        }
                    }

                    if (count < min_neighbors_keep)
                    {
                        depth[idx(u, v)] = 0.0f;
                    }
                }
            }
        }

        // ---------- 2) Multi-iterative hole filling (FAST 3x3, tanpa vector) ----------
        if (fill_iters > 0)
        {
            std::vector<float> cur = depth;
            std::vector<float> nxt = depth;

            for (int it = 0; it < fill_iters; ++it)
            {
                nxt = cur;

                for (int v = 1; v < height - 1; ++v)
                {
                    for (int u = 1; u < width - 1; ++u)
                    {
                        float d = cur[idx(u, v)];
                        if (d > 0.0f)
                            continue; // cuma isi hole (0)

                        float sum = 0.0f;
                        int count = 0;

                        for (int dv = -1; dv <= 1; ++dv)
                        {
                            for (int du = -1; du <= 1; ++du)
                            {
                                if (du == 0 && dv == 0)
                                    continue;
                                float dn = cur[idx(u + du, v + dv)];
                                if (dn > 0.0f)
                                {
                                    sum += dn;
                                    ++count;
                                }
                            }
                        }

                        if (count >= 3)
                        {
                            float avg = sum / static_cast<float>(count);
                            nxt[idx(u, v)] = avg;
                        }
                    }
                }

                cur.swap(nxt);
            }

            depth.swap(cur);
        }

        // ---------- 3) Smoothing ringan (median 3x3) ----------
        {
            std::vector<float> tmp = depth;
            float window[9];

            for (int v = 1; v < height - 1; ++v)
            {
                for (int u = 1; u < width - 1; ++u)
                {
                    float center = tmp[idx(u, v)];
                    if (center <= 0.0f)
                        continue; // biarkan hole yang masih kosong

                    int n = 0;
                    for (int dv = -1; dv <= 1; ++dv)
                    {
                        for (int du = -1; du <= 1; ++du)
                        {
                            float dn = tmp[idx(u + du, v + dv)];
                            if (dn > 0.0f)
                            {
                                window[n++] = dn;
                            }
                        }
                    }

                    if (n < 3)
                        continue;

                    // median di array kecil (<=9) pakai sort sederhana
                    std::sort(window, window + n);
                    float median = window[n / 2];

                    depth[idx(u, v)] = median;
                }
            }
        }
    }

    // ================================================================================================

    bool publish_pcl2cam(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_lidar,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr &caminfo,
        const Eigen::Matrix4f &T_base_to_cam,
        const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub,
        const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub_depth_color,
        rclcpp::Time override_stamp = rclcpp::Time(0),
        float z_min = 0.1f,
        float z_max = 100.0f,
        float z_min_vis = 0.5f, // range visualisasi
        float z_max_vis = 30.0f)
    {
        if (!cloud_lidar || !caminfo || !pub)
        {
            return false;
        }

        const int width = static_cast<int>(caminfo->width);
        const int height = static_cast<int>(caminfo->height);
        if (width <= 0 || height <= 0)
        {
            return false;
        }

        const double fx = caminfo->k[0];
        const double fy = caminfo->k[4];
        const double cx = caminfo->k[2];
        const double cy = caminfo->k[5];
        if (fx == 0.0 || fy == 0.0)
        {
            return false;
        }

        // 1) Transform cloud ke frame kamera
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cam(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*cloud_lidar, *cloud_cam, T_base_to_cam);

        // 2) Rasterisasi ke depth image (32FC1)
        std::vector<float> depth(width * height,
                                 std::numeric_limits<float>::infinity());

        auto idx = [width](int u, int v)
        {
            return v * width + u;
        };

        // "Splatting" supaya point lebih tebal di image space
        int radius_pix = 1; // 0 = 1 pixel, 1 = 3x3, 2 = 5x5

        for (const auto &pt : cloud_cam->points)
        {
            float x = pt.x;
            float y = pt.y;
            float z = pt.z;

            if (!std::isfinite(z) || z <= z_min || z >= z_max)
            {
                continue;
            }

            float u_f = static_cast<float>(fx) * (x / z) +
                        static_cast<float>(cx);
            float v_f = static_cast<float>(fy) * (y / z) +
                        static_cast<float>(cy);

            int u_center = static_cast<int>(std::round(u_f));
            int v_center = static_cast<int>(std::round(v_f));

            if (u_center < 0 || u_center >= width || v_center < 0 || v_center >= height)
            {
                continue;
            }

            // Splat ke patch kecil di sekitar (u_center, v_center)
            for (int dv = -radius_pix; dv <= radius_pix; ++dv)
            {
                int v = v_center + dv;
                if (v < 0 || v >= height)
                    continue;

                for (int du = -radius_pix; du <= radius_pix; ++du)
                {
                    int u = u_center + du;
                    if (u < 0 || u >= width)
                        continue;

                    std::size_t id = static_cast<std::size_t>(idx(u, v));
                    float old = depth[id];

                    // isi kalau masih inf atau z lebih dekat
                    if (z < old)
                    {
                        depth[id] = z;
                    }
                }
            }
        }

        // 3) Ganti inf → 0 sebelum post-filter
        for (auto &d : depth)
        {
            if (!std::isfinite(d))
            {
                d = 0.0f;
            }
        }

        // 4) Post-filter khusus outdoor (pakai signature BARU)
        //    sesuaikan param di sini kalau mau lebih/kurang agresif
        postfilter_depth_outdoor(
            depth,
            width,
            height,
            /*min_neighbors_keep=*/1,
            /*fill_iters=*/3,
            /*max_radius=*/3);

        // 5) Build & publish Image (depth 32FC1)
        sensor_msgs::msg::Image msg;
        msg.header = caminfo->header; // kalau mau, stamp bisa di-override di luar
        msg.header.stamp = override_stamp;
        msg.height = height;
        msg.width = width;
        msg.encoding = "32FC1";
        msg.is_bigendian = false;
        msg.step = width * sizeof(float);
        msg.data.resize(depth.size() * sizeof(float));
        std::memcpy(msg.data.data(),
                    depth.data(),
                    depth.size() * sizeof(float));

        pub->publish(msg);

        // 6) Optional: publish versi colorized
        if (depth_cam_pub_color_dbg)
        {
            if (pub_depth_color)
            {
                sensor_msgs::msg::Image color_msg;
                depth32fc1_to_rgb8(
                    depth,
                    width,
                    height,
                    z_min_vis,
                    z_max_vis,
                    caminfo->header.frame_id,
                    msg.header.stamp,
                    color_msg);

                pub_depth_color->publish(color_msg);
            }
        }
        return true;
    }

    void pre_calc_all_pcl2laserscan(pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out)
    {
        if (cloud_in.size() <= 0)
        {
            return;
        }

        static const float density_radius_ = 0.25;
        static const float min_neighbors_ = 7;

        /* Filter Z */
        std::vector<Point3D> roi_pts;
        roi_pts.reserve(cloud_in.size());

        for (const auto &p : cloud_in.points)
        {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                continue;

            // x depan, y kiri, z atas
            if (p.x < pcl2laser_obs_x_min || p.x > pcl2laser_obs_x_max)
                continue;

            if (p.y < pcl2laser_obs_y_min || p.y > pcl2laser_obs_y_max)
                continue;

            // range tinggi trotoar (relatif ground)
            if (p.z < pcl2laser_obs_z_min || p.z > pcl2laser_obs_z_max)
                continue;

            roi_pts.push_back({p.x, p.y, p.z});
        }

        const size_t N = roi_pts.size();
        if (N == 0)
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "No ROI points for sidewalk");
            return;
        }

        /* Buat kdtree pakaei nanoflann */
        PointCloudAdaptor adaptor(roi_pts);
        KDTree index(3, adaptor,
                     nanoflann::KDTreeSingleIndexAdaptorParams(10));
        index.buildIndex();
        const float radius_sq = static_cast<float>(density_radius_ * density_radius_);
        std::vector<uint8_t> is_trotoar(N, 0);

        /* Radius search by nanoflann */
#ifdef _OPENMP
#pragma omp parallel for
#endif
        for (int i = 0; i < static_cast<int>(N); ++i)
        {
            const Point3D &p = roi_pts[i];
            const float query_pt[3] = {p.x, p.y, p.z};

            using Result = nanoflann::ResultItem<unsigned int, float>;
            std::vector<Result> matches;
            nanoflann::SearchParameters params;
            params.sorted = false;

            size_t found = index.radiusSearch(query_pt, radius_sq, matches, params);
            if (static_cast<int>(found) >= min_neighbors_)
            {
                is_trotoar[i] = 1;
            }
        }

        /* Masukkan ke output ptr */
        cloud_out.reserve(N);
        for (size_t i = 0; i < N; ++i)
        {
            if (!is_trotoar[i])
                continue;
            const auto &pt = roi_pts[i];
            cloud_out.emplace_back(pt.x, pt.y, pt.z);
        }
    }

    sensor_msgs::msg::LaserScan calc_all_pcl2laserscan(const std::string &target_frame, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud)
    {
        sensor_msgs::msg::LaserScan laser_scan_msg;

        if (pcl_cloud.empty())
        {
            return laser_scan_msg;
        }

        // --- parameter scan (bisa dijadikan param ROS) ---
        const float angle_min = -3.14f;
        const float angle_max = 3.14f;
        const float angle_inc = 0.0174532925f; // 1 derajat
        const float range_min = 0.05f;
        const float range_max = 30.0f;

        laser_scan_msg.header.stamp = this->now();
        laser_scan_msg.header.frame_id = target_frame;
        laser_scan_msg.angle_min = angle_min;
        laser_scan_msg.angle_max = angle_max;
        laser_scan_msg.angle_increment = angle_inc;
        laser_scan_msg.range_min = range_min;
        laser_scan_msg.range_max = range_max;

        const uint32_t num_ranges =
            static_cast<uint32_t>((angle_max - angle_min) / angle_inc) + 1;

        // ----------------------------------------------------------
        // 1. Binning titik ke dalam sudut (beam)
        // ----------------------------------------------------------
        struct BeamPoint
        {
            float r;
            float z;
            float x;
            float y;
        };

        std::vector<std::vector<BeamPoint>> beams(num_ranges);

        for (const auto &point : pcl_cloud.points)
        {
            const float x = point.x;
            const float y = point.y;
            const float z = point.z;

            const float r = std::sqrt(x * x + y * y);
            if (!std::isfinite(r) || r < range_min || r > range_max)
                continue;

            const float angle = std::atan2(y, x);
            if (angle < angle_min || angle > angle_max)
                continue;

            const uint32_t index = static_cast<uint32_t>(
                (angle - angle_min) / angle_inc);
            if (index >= num_ranges)
                continue;

            beams[index].push_back({r, z, x, y});
        }

        // ----------------------------------------------------------
        // 2. Deteksi step trotoar di tiap beam (PARALEL dengan OpenMP)
        // ----------------------------------------------------------
        laser_scan_msg.ranges.assign(
            num_ranges, std::numeric_limits<float>::infinity());

        const int num_bins = static_cast<int>(num_ranges);

#ifdef _OPENMP
#pragma omp parallel for
#endif
        for (int i = 0; i < num_bins; ++i)
        {
            auto &vec = beams[i];
            if (vec.size() < 2)
            {
                continue;
            }

            // sort dari dekat ke jauh
            std::sort(vec.begin(), vec.end(),
                      [](const BeamPoint &a, const BeamPoint &b)
                      {
                          return a.r < b.r;
                      });

            // kandidat trotoar: pilih yang r paling kecil
            float best_step_range = std::numeric_limits<float>::infinity();

            float prev_r = vec[0].r;
            float prev_z = vec[0].z;

            for (size_t j = 1; j < vec.size(); ++j)
            {
                const float curr_r = vec[j].r;
                const float curr_z = vec[j].z;

                // kalau jarak beda jauh, anggap permukaan beda, reset referensi
                if ((curr_r - prev_r) > max_range_gap)
                {
                    prev_r = curr_r;
                    prev_z = curr_z;
                    continue;
                }

                const float delta_z = curr_z - prev_z;
                prev_r = curr_r;
                prev_z = curr_z;

                // cek "perbedaan h besar"
                if (delta_z >= threshold_delta_z)
                {
                    // jadikan kandidat, tapi jangan langsung break
                    if (curr_r < best_step_range)
                    {
                        best_step_range = curr_r;
                    }
                }
            }

            if (std::isfinite(best_step_range))
            {
                laser_scan_msg.ranges[i] = best_step_range;
            }
            // kalau tidak ada kandidat, biarkan tetap infinity
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

    void callback_sub_roadseg_mask(const sensor_msgs::msg::Image::SharedPtr msg)
    {
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

        mtx_lidar_kanan.lock();
        pcl_lidar_kanan_raw = cloud;
        mtx_lidar_kanan.unlock();
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

        mtx_lidar_kiri.lock();
        pcl_lidar_kiri_raw = cloud;
        mtx_lidar_kiri.unlock();
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

        last_stamp_lidar_tengah = msg->header.stamp;

        last_time_lidar_tengah_update_ms = wall_time_ms;

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        mtx_lidar_tengah.lock();
        pcl_lidar_tengah_raw = cloud;
        mtx_lidar_tengah.unlock();
    }

    void callback_tim_routine()
    {
        if (!is_tf_initialized)
        {
            logger.warn("Waiting for transforms to be initialized...");
            return;
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

        mtx_lidar_kiri.lock();
        if (pcl_lidar_kiri_raw.size() > 0)
        {
            pcl::transformPointCloud(pcl_lidar_kiri_raw, pcl2laser_obs_lidar_kiri_in_base_link, T_lidar_kiri2_base_link_4f);
            all_obstacle_points += pcl2laser_obs_lidar_kiri_in_base_link;
        }
        mtx_lidar_kiri.unlock();

        mtx_lidar_kanan.lock();
        if (pcl_lidar_kanan_raw.size() > 0)
        {
            pcl::transformPointCloud(pcl_lidar_kanan_raw, pcl2laser_obs_lidar_kanan_in_base_link, T_lidar_kanan2_base_link_4f);
            all_obstacle_points += pcl2laser_obs_lidar_kanan_in_base_link;
        }
        mtx_lidar_kanan.unlock();

        mtx_lidar_tengah.lock();
        if (pcl_lidar_tengah_raw.size() > 0)
        {
            pcl::transformPointCloud(pcl_lidar_tengah_raw, pcl2laser_obs_lidar_tengah_in_base_link, T_lidar_tengah2_base_link_4f);
            all_obstacle_points += pcl2laser_obs_lidar_tengah_in_base_link;
        }
        mtx_lidar_tengah.unlock();

        pcl::CropBox<pcl::PointXYZ> crop_box_filter_near;
        pcl::PointCloud<pcl::PointXYZ> all_obstacle_point_near;
        crop_box_filter_near.setInputCloud(all_obstacle_points.makeShared());
        crop_box_filter_near.setMin(Eigen::Vector4f(-30.0, -30.0, -30.0, 1));
        crop_box_filter_near.setMax(Eigen::Vector4f(30.0, 30.0, 30.0, 1));
        crop_box_filter_near.setNegative(false);
        crop_box_filter_near.filter(all_obstacle_point_near);

        // Membuat depth image
        if (all_obstacle_point_near.size() > 0)
        {
            process_depth_images(all_obstacle_point_near.makeShared());
        }

        pcl::CropBox<pcl::PointXYZ> crop_box_filter_exclude;
        pcl::PointCloud<pcl::PointXYZ> all_obstacle_points_exclude;
        crop_box_filter_exclude.setInputCloud(all_obstacle_points.makeShared());
        crop_box_filter_exclude.setMin(Eigen::Vector4f(exclude_x_min, exclude_y_min, exclude_z_min, 1));
        crop_box_filter_exclude.setMax(Eigen::Vector4f(exclude_x_max, exclude_y_max, exclude_z_max, 1));
        crop_box_filter_exclude.setNegative(true);
        crop_box_filter_exclude.filter(all_obstacle_points_exclude);

        // Filter  disini pakai cropbox
        pcl::CropBox<pcl::PointXYZ> crop_box_filter_final;
        pcl::PointCloud<pcl::PointXYZ> final_obstacle_points_cropped;
        crop_box_filter_final.setInputCloud(all_obstacle_points_exclude.makeShared());
        crop_box_filter_final.setMin(Eigen::Vector4f(pcl2laser_obs_x_min, pcl2laser_obs_y_min, pcl2laser_obs_z_min, 1));
        crop_box_filter_final.setMax(Eigen::Vector4f(pcl2laser_obs_x_max, pcl2laser_obs_y_max, pcl2laser_obs_z_max, 1));
        crop_box_filter_final.setNegative(false);
        crop_box_filter_final.filter(final_obstacle_points_cropped);

        // Publish debug pcl2laser
        sensor_msgs::msg::PointCloud2 debug_pcl2laser_msg;
        pcl::toROSMsg(final_obstacle_points_cropped, debug_pcl2laser_msg);
        debug_pcl2laser_msg.header.stamp = last_stamp_lidar_tengah;
        debug_pcl2laser_msg.header.frame_id = "base_link";
        pub_debug_pcl2laser->publish(debug_pcl2laser_msg);

        // Membuat laser scan
        if (using_normal_to_laserscan)
        {
            pcl::PointCloud<pcl::PointXYZ> pcl_trotoar;
            pre_calc_all_pcl2laserscan(all_obstacle_points_exclude, pcl_trotoar);
            all_obstacle_laserscan = calc_all_pcl2laserscan("base_link", pcl_trotoar);
        }
        else
        {
            all_obstacle_laserscan = calc_all_pcl2laserscan("base_link", final_obstacle_points_cropped);
        }
        all_obstacle_laserscan.header.stamp = last_stamp_lidar_tengah;

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
