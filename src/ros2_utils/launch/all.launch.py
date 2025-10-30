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

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # fmt: off
        arguments=["-d",os.path.join(path_config,"robot.rviz"),
                   "--ros-args","--log-level","error",]
        # fmt: on
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
        # output='screen'
    )

    gps = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(utils_config, "launch"), "/navsat.launch.py"]
        )
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


    return LaunchDescription(
        [
            joint_state_publisher_node,
            robot_state_publisher_node,

            multilidar,
            # multicamera,
            hesai_lidar,
            gps,

            # rosapi_node,
            # ui_server,
            # rosbridge_server, 

            # telemetry,

            # master,

            # keyboard_input,

            # wifi_control,
            # CANBUS_HAL_node,

            rviz2,
        ]
    )


# b34587e5a000000000042