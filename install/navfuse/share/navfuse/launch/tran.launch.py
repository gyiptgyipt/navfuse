
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        # Static Transform Publisher Node
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='bl_gps',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'gps_link']
        ),

        # NavSat Transform Node
        launch_ros.actions.Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[
                {"magnetic_declination_radians": 0.0},  # Set your magnetic declination
                {"zero_altitude": False},
                {"publish_filtered_gps": True},
                {"broadcast_utm_transform": True},
                {"wait_for_datum": False},
            ],
            remappings=[
                ("/gps/fix", "/gps/micro"),
                ("/imu/data", "/imu/micro"),
            ]
        ),

        # EKF Localization Node
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_localization_node',
            name='ekf_odom',
            output='screen',
            clear_params='true',
            parameters={
                "imu0": "/imu/micro",
                "frequency": 30,
                "sensor_timeout": 2,
                "two_d_mode": True,
                "map_frame": "map",
                "odom_frame": "odom",
                "base_link_frame": "base_link",
                "world_frame": "odom",
                "imu0_config": [False, False, False, False, False, True, False, False, False, False, False, True, True, False, False],
                "imu0_differential": False,
                "imu0_remove_gravitational_acceleration": True,
                "print_diagnostics": True,
                "debug": False,
                "debug_out_file": "$(env HOME)/adroit_files/debug_ekf_localization.txt"
            }
        )
    ])