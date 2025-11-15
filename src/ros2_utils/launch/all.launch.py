import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

from ament_index_python.packages import get_package_share_directory

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

path_config_buffer = os.getenv('AMENT_PREFIX_PATH', '')
path_config_buffer_split = path_config_buffer.split(":")
ws_path = path_config_buffer_split[0] + "/../../"
path_config = ws_path + "src/ros2_utils/configs/"
utils_config = ws_path + "src/ros2_utils/"

CHERY_CANFD_LKAS_CAM_CMD_345_FRAME_ID = 0x345
CHERY_CANFD_LKAS_STATE_FRAME_ID = 0x307
CHERY_CANFD_SETTING_FRAME_ID = 0x387
CHERY_CANFD_ACC_CMD_FRAME_ID = 0x3a2
CHERY_CANFD_STEER_BUTTON_FRAME_ID = 0x360

def generate_launch_description():
    
    SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_cyclonedds_cpp'),
    SetEnvironmentVariable(name='CYCLONEDDS_URI', value='file://' + path_config + 'cyclonedds.xml'),

    # =========================== Robot Description ===========================

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": Command(["xacro ", os.path.join(utils_config, "description", "omoda.urdf")])}
        ],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        arguments=[os.path.join(utils_config, "description", "omoda.urdf")],
    )

    tf_map_empty = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_map_empty",
        # fmt: off
        arguments=["0.00","0.00","0.00","0.00","0.00","0.00","map","odom",
            "--ros-args","--log-level","error",],
        # fmt: on
        respawn=True,
    )

    # =========================== Bawaan ROS ===========================

    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        respawn=True,
    )

    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi_node',
        output='screen',
        respawn=True,
    )

    web_video_server = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        output="screen",
        respawn=True,
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # fmt: off
        arguments=["-d",os.path.join(path_config,"robot.rviz"),
                   "--ros-args","--log-level","error",]
        # fmt: on
    )

    rs2_cam_main = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="rs2_cam_main",
        parameters=[
            {
                "camera_name": "camera",
                "camera_namespace": "",
                "enable_accel": False,
                "enable_gyro": False,
                "enable_depth": True,
                "enable_color": True,
                "enable_sync": True,
                "enable_infra1": False,
                "enable_infra2": False,
                "unite_imu_method": 2,
                "align_depth.enable": True,
                "pointcloud.enable": True,
                "initial_reset": False,
                "rgb_camera.color_profile": "640x360x15",
            }
        ],
        arguments=["--ros-args", "--log-level", "error"],
        respawn=True,
        output="screen",
    )

    rtabmap_slam = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace="slam",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "frame_id": "base_link",
            "map_frame_id": "map",
            "odom_frame_id": "odom",

            "subscribe_cloud": False,
            "subscribe_scan_cloud": True,
            "subscribe_scan": False,
            "subscribe_imu": False,
            "subscribe_odom": False,
            "subscribe_stereo": False,
            "subscribe_rgb": True,
            "subscribe_rgbd": False,
            "subscribe_depth": False,
            "qos_scan": 1,
            "wait_for_transform": 2.0,

            "publish_tf": False,
            "approx_sync": True,
            "sync_queue_size": 30,
            "topic_queue_size": 30,
            'qos': 1,
            "wait_for_transform": 2.0,
            "icp_odometry": True, # Percobaan odom pake ICP gak dari hardware

            "Rtabmap/DetectionRate": "5.0",  # Added by Azzam
            "Rtabmap/CreateIntermediateNodes": "True",
            "Rtabmap/LoopThr": "0.11",  # Routine period untuk cek loop closure

            "Mem/STMSize": "20",  # Short-term memory size
            "Mem/IncrementalMemory": "True",  #
            "Mem/InitWMWithAllNodes": "True",  #
            "Mem/RehearsalSimilaritys": "0.9",  #
            "Mem/UseOdomFeatures": "False",  #
            "Mem/NotLinkedNodesKept": "True",  # Keep unlinked nodes

            "Kp/MaxFeatures": "2000",
            "Kp/DetectorStrategy": "2", # Full ORB saja

            "RGBD/Enabled": "True", #
            "RGBD/OptimizeFromGraphEnd": "False",  # True agar robot tidak lompat
            "RGBD/NeighborLinkRefining": "True",  # Added from documentation, ikut odom saja (false)
            "RGBD/AngularUpdate": "0.3",  # Added from documentation
            "RGBD/LinearUpdate": "0.3",  # Added from documentation
            "RGBD/OptimizeMaxError": "4.0",  # Added from documentation
            "RGBD/InvertedReg": "False",  # Added from documentation
            "RGBD/ProximityPathMaxNeighbors": "5",
            "RGBD/ProximityMaxGraphDepth": "20",
            "RGBD/ProximityByTime": "False",
            "RGBD/ProximityBySpace": "True",

            "Optimizer/Strategy": "2",  # Added by Azzam
            "Optimizer/Iterations": "10",  # Added by Azzam
            "Optimizer/Epsilon": "0.00001",  # Added by Azzam
            "Optimizer/Robust": "False",  # Added by Azzam
            "Optimizer/GravitySigma": "0.3",
            "Optimizer/VarianceIgnored": "False",
            "Optimizer/LandmarksIgnored": "False",  # Added by Azzam
            "Optimizer/PriorsIgnored": "True",  # Added by Azzam
            "GTSAM/Incremental": "False",  # Added by Azzam
            "Bayes/PredictionMargin": "0",  # Added by Azzam
            "Bayes/FullPredictionUpdate": "False",  # Added by Azzam
            "Bayes/PredictionLC": "0.1 0.36 0.30 0.16 0.062 0.0151 0.00255 0.000324 2.5e-05 1.3e-06 4.8e-08 1.2e-09 1.9e-11 2.2e-13 1.7e-15 8.5e-18 2.9e-20 6.9e-23",  # Added by Azzam
            "Odom/Strategy": "0",  # Added by Azzam
            "Odom/ResetCountdown": "0",  # Added by Azzam
            "Odom/Holonomic": "False",  # Added by Azzam
            "Odom/ScanKeyFrameThr": "0.9",  # Added by Azzam, semakin kecil semakin sering lidar update
            "Odom/AlignWithGround": "True",  # Added by Azzam
            "Odom/FilteringStrategy": "1",

            "Reg/Strategy": "1",  # Added by Azzam
            "Reg/Force3DoF": "True",  # Added by Azzam
            "Icp/Strategy": "1",  # Added by Azzam
            "Icp/MaxTranslation": "1.5",  # Added by Azzam
            "Icp/MaxRotation": "0.7",  # Added by Azzam
            "Icp/RangeMin": "0.0",  # Added by Azzam
            "Icp/RangeMax": "25.0",  # Added by Azzam
            "Icp/MaxCorrespondenceDistance": "1.0",  # Added by Azzam
            "Icp/Iterations": "30",  # Added by Azzam
            "Icp/PointToPlane": "True",  # Added by Azzam
            "Icp/VoxelSize": "0.05",  # Added by Azzam
            "Icp/PointToPlaneMinComplexity": "0.03",  # to be more robust to long corridors with low geometry
            "Icp/PointToPlaneLowComplexityStrategy": "1",  # to be more robust to long corridors with low geometry
            "Vis/MaxDepth": "20.0",
            "Vis/MinInliers": "15",
            "Vis/FeatureType": "2",
            "Vis/EpipolarGeometryVar": "0.5",
            "Grid/Sensor": "2",  # Added to suppress warning
            "Grid/RangeMin": "0.0",  # Added by Azzam
            "Grid/RangeMax": "150.0",  # Added by Azzam
            "Grid/NormalsSegmentation": 'false', # Use passthrough filter to detect obstacles
            "Grid/UpdateRate": "1.0",  # Update map every 1 second (default is often higher)
            "Grid/CellSize": "0.3",  # Increase cell size to reduce map density
            "Grid/FromDepth": "False",  # Added from documentation
            "Grid/IncrementalMapping": "True",  # Added from documentation
            "Grid/Scan2dUnknownSpaceFilled": "False",  # Added by Azzam
            "GridGlobal/UpdateError": "0.04",  # Added by Azzam
            "Grid/RayTracing": "False",  # Added by Azzam
            'Grid/MaxObstacleHeight':'3.5',  # All points over 1 meter are ignored

            # Depth dari lidar 
            "gen_depth": True,
            "gen_depth_decimation": 4,
            "gen_depth_fill_holes_size": 2,
            "gen_depth_fill_iterations": 1,
            "gen_depth_fill_holes_error": 0.3,

        }],
        remappings=[
            ("scan_cloud", "/lidartengah/lidar_points"),
            ("rgb/image", "/camera/rs2_cam_main/color/image_raw"),
            ("rgb/camera_info", "/camera/rs2_cam_main/color/camera_info"),
            # ("depth/image", "/camera/rs2_cam_main/aligned_depth_to_color/image_raw"),
        ],
        arguments=["--ros-args", "--log-level", "info"],
        respawn=True,
    )

    rtabmap_icp_odom = Node(
        package="rtabmap_odom",
        executable="icp_odometry",
        name="icp_odometry",
        output="screen",
        namespace="icp_odom",
        parameters=[{
            "use_sim_time": False,
            "frame_id": "base_link",

            "subscribe_cloud": False,
            "subscribe_scan_cloud": True,
            "subscribe_scan": False,
            "subscribe_imu": True,
            "subscribe_odom": False,
            "subscribe_stereo": False,
            "subscribe_rgb": False,
            "subscribe_rgbd": False,
            "subscribe_depth": False,

            "publish_tf": False,
            "approx_sync": True,
            "sync_queue_size": 30,
            "topic_queue_size": 10,
            'qos': 1,
            "wait_for_transform": 2.0,

            'Mem/IncrementalMemory':'False',
            'Mem/InitWMWithAllNodes':'True',

            "Reg/Force3DoF": "False",

            "Odom/Strategy": "1",
            "Odom/ResetCountdown": "0",
            "Odom/ScanKeyFrameThr": "0.9",  # Added by Azzam, semakin kecil semakin sering lidar update
            "Odom/GuessMotion": "True",  

            "Icp/Strategy": "1",  # Added by Azzam
            "Icp/MaxTranslation": "2.0", # Added by Azzam
            "Icp/MaxRotation": "0.7", # Added by Azzam
            "Icp/RangeMin": "0.0", # Added by Azzam
            "Icp/RangeMax": "100.0", # Added by Azzam
            "Icp/MaxCorrespondenceDistance": "1.2", # Added by Azzam
            "Icp/Iterations": "30", # Added by Azzam
            "Icp/PointToPlane": "True", # Added by Azzam
            "Icp/VoxelSize": "0.10", # Added by Azzam
            'Icp/PointToPlaneMinComplexity':'0.01254', 
            'Icp/PointToPlaneLowComplexityStrategy':'1', 
        }],
        remappings=[
            ("scan_cloud", "/lidartengah/lidar_points"),
            ("imu", "/lidartengah/lidar_imu"),
            # ("scan", "/disale_wtf_scan"),
            # ("odom", "vo")
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        respawn=True,
    )

    ekf_icp_odom = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        namespace="icp_odom",
        parameters=[{
            "frequency": 50.0,
            "two_d_mode": True,
            "sensor_timeout": 0.2,
            "map_frame": "map",
            "odom_frame": "odom",
            "base_link_frame": "base_link",
            "world_frame": "odom",
            "publish_tf": True,

            "odom0": "/icp_odom/odom",
            "odom0_config": [True, True, False,
                             False, False, True,
                             True,  True,  False,
                             False, False, True,
                             False, False, False],
            "odom0_differential": False,
            "odom0_relative": False,

            "imu0": "/lidartengah/lidar_imu",
            "imu0_config": [False, False, False,
                            False,  False,  True,
                            False, False, False,
                            False,  False,  True,
                            False, False, False],
            "imu0_differential": False,
            "imu0_relative": False,
        }],
        arguments=["--ros-args", "--log-level", "warn"],
        respawn=True,
    )

    ekf_final_pose = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_final_pose",
        namespace="slam",
        parameters=[{
            "frequency": 50.0,
            "two_d_mode": True,
            "sensor_timeout": 0.2,
            "map_frame": "map",
            "odom_frame": "odom",
            "base_link_frame": "base_link",
            "world_frame": "odom",
            "publish_tf": False,

            "odom0": "/icp_odom/odometry/filtered",
            "odom0_config": [True, True, False,
                             False, False, True,
                             False,  False,  False,
                             False, False, False,
                             False, False, False],
            "odom0_differential": True,
            "odom0_relative": True,

            "imu0": "localization_pose",
            "imu0_config": [True, True, False,
                            False,  False,  True,
                            False, False, False,
                            False,  False,  False,
                            False, False, False],
            "imu0_differential": False,
            "imu0_relative": False,
        }],
        arguments=["--ros-args", "--log-level", "warn"],
        respawn=True,
    )

    # =========================== Communication ===========================

    wifi_control = Node(
        package="communication",
        executable="wifi_control",
        name="wifi_control",
        parameters=[
            {
                "hotspot_ssid": "gh_template",
                "hotspot_password": "gh_template",
            },
        ],
        output="screen",
        respawn=True,
    )

    telemetry = Node(
        package="communication",
        executable="telemetry.py",
        name="telemetry",
        parameters=[{
            "INFLUXDB_URL": "http://127.0.0.1:8086",
            "INFLUXDB_USERNAME": "its",
            "INFLUXDB_PASSWORD": "itssurabaya",
            "INFLUXDB_ORG": "its",
            "INFLUXDB_BUCKET": "robotics",
            "ROBOT_NAME": "omoda", 
        }],
        output="screen",
        respawn=True,
    )

    # =========================== Hardware ===========================

    keyboard_input = Node(
        package='hardware',
        executable='keyboard_input',
        name='keyboard_input',
        output='screen',
        respawn=True,
        prefix=['xterm -e'],
    )

    CANBUS_HAL_node = Node(
        package='hardware',
        executable='CANBUS_HAL_node',
        name='CANBUS_HAL_node',
        namespace='hardware',
        parameters=[{
            "can1_type": 0,
            "can2_type": 0,
            "device1_name": "/dev/ttyACM1",
            "device2_name": "/dev/ttyACM0", # String kosong untuk nonaktifkan
            "intercepted_ids_can": [CHERY_CANFD_LKAS_CAM_CMD_345_FRAME_ID, 
                                    CHERY_CANFD_LKAS_STATE_FRAME_ID, 
                                    # CHERY_CANFD_SETTING_FRAME_ID, 
                                    CHERY_CANFD_ACC_CMD_FRAME_ID, 
                                    CHERY_CANFD_STEER_BUTTON_FRAME_ID
                                    ],
            "baudrate": 500000,
            "fd_baudrate": 2000000,
            "pid_terms": [0.4, 0.002, 0, 0.02, -3.5, 2.0, -0.1, 0.02],
            "publish_period_ms": 20,
        }],
        remappings=[
            ("cmd_target_steering_angle", "/master/cmd_target_steering_angle"),
            ("cmd_target_velocity", "/master/cmd_target_velocity"),
            ("cmd_hw_flag", "/master/cmd_hw_flag"),
        ],
        output='screen',
        respawn=True,
    )

    multilidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(utils_config, "launch"), "/multi_vlp16.launch.py"]
        )
    )

    multicamera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(utils_config, "launch"), "/multicamera.launch.py"]
        )
    )

    hesai_lidar = Node(
        package='hesai_ros_driver', 
        executable='hesai_ros_driver_node', 
        namespace='lidartengah', 
        parameters=[{
            "timestamp_type": "ros",
            "use_lidar_time": False,
            "use_gps_ts": False,
        }]
        # output='screen'
    )

    gps = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(utils_config, "launch"), "/navsat.launch.py"]
        )
    )

    serial_imu = Node(
        package="hardware",
        executable="serial_imu",
        name="serial_imu",
        output="screen",
        parameters=[
            {
                "port": "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0",
                "use_boost": False,
                "is_riontech": True,
                "baudrate": 115200,
            }
        ],
        respawn=True,
    )

    # =========================== Master ===========================

    master = Node(
        package='master',
        executable='master',
        name='master',
        namespace='master',
        output='screen',
        remappings=[
            ("fb_steering_angle", "/hardware/fb_steering_angle"),
            ("fb_current_velocity", "/hardware/fb_current_velocity"),
            ("fb_throttle_position", "/hardware/fb_throttle_position"),
            ("fb_brake_position", "/hardware/fb_brake_position"),
            ("fb_gear_status", "/hardware/fb_gear_status"),
            ("fb_steer_torque", "/hardware/fb_steer_torque"),
        ],
        respawn=True,
        prefix='nice -n -10',
    )

    # =========================== Vision ===========================

    dummy_video2ros = Node(
        package="vision",
        executable="dummy_video2ros.py",
        name="dummy_video2ros",
        parameters=[{
            "source": os.path.join(ws_path, "src/vision/scripts/2_rgb_output.mp4"),
            "topic": "/camera/rs2_cam_main/color/image_raw",
        }],
        output="screen",
        respawn=True,
    )

    camera_driver_node2 = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera_node",
        namespace="cameratengah",
        output="both",
        parameters=[os.path.join(utils_config, "configs", "camera_driver.yaml")],
    )

    road_segmentation = Node(
        package="vision",
        executable="road_segmentation.py",
        name="road_segmentation",
        parameters=[{
            "use_cuda": True,
            "weights": os.path.join(ws_path, "src/vision/models/200_nn.pth"),
            "thr": 0.09,
            "use_wb": True,
            "use_clahe": True,
            "use_adaptive_gamma": True,
            "touch_bottom": False,
            "min_area": 100,
            "alpha_lp": 0.7, # INi low pass kayak biasanya (per waktu)
            "crop_upper": 175,
            "crop_lower": 100, # 100 untuk realsense dalam mobil
            # "crop_lower": 20, # 20 untuk kamera atas
            "reset_on_scene_cut": False,
            "scene_cut_thresh": 0.35,

            "image_topic": "/camera/rs2_cam_main/color/image_raw",
            # "image_topic": "/cameratengah/image_raw",
            "mask_topic": "/road_seg/mask",
            "publish_period": 0.001,
            "publish_overlay": True, # Buat debug
        }],
        output="screen",
        respawn=True,
    )

    coco_object_detection = Node(
        package="vision",
        executable="coco_object_detection.py",
        name="coco_object_detection",
        parameters=[{
            # "rgb_topic_sub": "/camera/rs2_cam_main/color/image_raw",
            "rgb_topic_sub": "/cameratengah/image_raw",
            "rgb_topic_pub": "/coco_object_detection/annotated_image",
            "model_path": os.path.join(ws_path, "src/vision/models/yolov8m.pt"),
            "conf_thresh": 0.4,
            "imgsz": 640,
        }],
        output="screen",
        respawn=True,
    )

    # =========================== Web UI ===========================

    ui_server = Node(
        package="web_ui",
        executable="ui_server.py",
        name="ui_server",
        parameters=[
            {
                "ui_root_path": os.path.join(ws_path,"src/web_ui/src")
            },
        ],
        output="screen",
        respawn=True,
    )

    # =========================== World Model ===========================

    pose_estimator = Node(
        package="world_model",
        executable="pose_estimator",
        name="pose_estimator",
        output="screen",
        namespace='world_model',
        parameters=[
            {
                "encoder_to_meter": 1.0,  
                "offset_sudut_steering": -0.04,
                "gyro_type": 0,
                "timer_period": 20, # 50 hz
                "use_encoder_pulse": False
            }
        ],
        respawn=True,
        # remappings=[('/hardware/imu', '/can/imu')],
        ##        prefix='nice -n -9 chrt -f 60'
    )


    all_obstacle_filter = Node(
        package="world_model",
        executable="all_obstacle_filter",
        name="all_obstacle_filter",
        output="screen",
        namespace='world_model',
        parameters=[
            {
                "lidar_kanan_topic": "/velodynekanan/velodyne_points",  
                "lidar_kiri_topic": "/velodynekiri/velodyne_points",
                "lidar_tengah_topic": "/lidartengah/lidar_points",
                "lidar_kanan_frame_id": "velodynekanan",
                "lidar_kiri_frame_id": "velodynekiri",
                "lidar_tengah_frame_id": "lidartengah",
                "exclude_x_min": -2.5,
                "exclude_x_max": 2.5,
                "exclude_y_min": -0.3,
                "exclude_y_max": 0.3,
                "exclude_z_min": -200.0,
                "exclude_z_max": 200.0,
                "pcl2laser_obs_x_min": -10.0,
                "pcl2laser_obs_x_max": 10.0,
                "pcl2laser_obs_y_min": -3.0,
                "pcl2laser_obs_y_max": 3.0,
                "pcl2laser_obs_z_min": 0.1,
                "pcl2laser_obs_z_max": 1.0,
            }
        ],
        respawn=True,
        # remappings=[('/hardware/imu', '/can/imu')],
        ##        prefix='nice -n -9 chrt -f 60'
    )


    return LaunchDescription(
        [
            # dummy_video2ros,

            tf_map_empty,
            # pose_estimator,
            joint_state_publisher_node,
            robot_state_publisher_node,

            # multilidar,
            # # multicamera,
            # hesai_lidar,
            # # gps,

            # all_obstacle_filter,

            rs2_cam_main,
            camera_driver_node2,
            road_segmentation,
            coco_object_detection,

            # serial_imu,

            # rtabmap_icp_odom,
            # ekf_icp_odom,

            # rtabmap_slam,
            # ekf_final_pose,

            rosapi_node,
            ui_server,
            rosbridge_server, 
            web_video_server,

            # telemetry,

            # master,

            # keyboard_input,

            # wifi_control,
            # CANBUS_HAL_node,

            # rviz2,
        ]
    )


# b34587e5a000000000042