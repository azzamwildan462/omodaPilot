"""Launch the all camera."""

import os

import ament_index_python.packages
import launch
import launch_ros.actions
import yaml


def generate_launch_description():
    driver_share_dir = ament_index_python.packages.get_package_share_directory(
        "ros2_utils"
    )
    driver_params_file = os.path.join(driver_share_dir, "config", "camera_driver.yaml")

    camera_driver_node1 = launch_ros.actions.Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera_node",
        namespace="camerakiri",
        output="both",
        parameters=[driver_params_file],
    )

    camera_driver_node2 = launch_ros.actions.Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera_node",
        namespace="cameratengah",
        output="both",
        parameters=[driver_params_file],
    )

    camera_driver_node3 = launch_ros.actions.Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera_node",
        namespace="camerakanan",
        output="both",
        parameters=[driver_params_file],
    )

    ## Undistord Image
    camera_rectify_node1 = launch_ros.actions.Node(
        package="image_proc",
        executable="rectify_node",
        name="rectify_node",
        namespace="camerakanan",
        output="both",
        remappings=[
                ('image', 'image_raw')
            ]
    )

    camera_rectify_node2 = launch_ros.actions.Node(
        package="image_proc",
        executable="rectify_node",
        name="rectify_node",
        namespace="cameratengah",
        output="both",
        remappings=[
                ('image', 'image_raw')
            ],
    )

    camera_rectify_node3 = launch_ros.actions.Node(
        package="image_proc",
        executable="rectify_node",
        name="rectify_node",
        namespace="camerakiri",
        output="both",
        remappings=[
                ('image', 'image_raw')
            ],
    )

    # image_merger = launch_ros.actions.Node(
    #     package="ros2_panorama",
    #     executable="image_merger",
    #     output='both',
    #     # remappings=[
    #     #     ("panorama", "/panorama"),
    #     #     ("camera_left", "/camerakiri/image_rect/compressed"),
    #     #     ("camera_centre", "/cameratengah/image_rect/compressed"),
    #     #     ("camera_right", "/camerakanan/image_rect/compressed"),
    #     # ],
    #      remappings=[
    #         ("panorama", "/panorama"),
    #         ("camera_left", "/camerakiri/image_rect"),
    #         ("camera_centre", "/cameratengah/image_rect"),
    #         ("camera_right", "/camerakanan/image_rect"),
    #     ],
    # )

    # panorama_converter_node = launch_ros.actions.Node(
    #     package="image_to_base64",
    #     executable="image_to_base64",
    #     name="bird_eye_converter",
    #     arguments=[
    #         "--sub_topic",
    #         "/panorama",
    #         "--pub_topic",
    #         "panorama/converted",
    #     ],
    # )

    # bird_eye_converter_node = launch_ros.actions.Node(
    #     package="image_to_base64",
    #     executable="image_to_base64",
    #     name="bird_eye_converter",
    #     arguments=[
    #         "--sub_topic",
    #         "/warp",
    #         "--pub_topic",
    #         "warp/converted",
    #     ],
    # )


    return launch.LaunchDescription(
        [
            camera_driver_node1,
            camera_driver_node2,
            camera_driver_node3,

            #camera_rectify_node1,
            #camera_rectify_node2,
            #camera_rectify_node3,
            # image_merger,

            # panorama_converter_node,
            # bird_eye_converter_node,
        ]
    )
