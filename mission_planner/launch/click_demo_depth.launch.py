import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    super_config_name = 'click_smooth_ros2_depth.yaml'

    ld = LaunchDescription()

    SUPER = Node(
        package='super_planner',
        executable='fsm_node',
        output='screen',
        parameters=[{
            'config_name': super_config_name,
        }]
    )

    control = Node(
        package='super_planner',
        executable='uav_controller_node',
        output='screen',
        remappings=[
            ('setpoint', '/planning/pos_cmd'),
            ('odom', '/quadrotor1/odometry/local'),
            ('cmd_vel', '/quadrotor1/cmd_vel'),
        ],
        parameters=[{
            'kp': 0.5,
            'kd': 0.0,
            'z_scale': 0.7,
            'max_cmd_lin_': 2.0,
            'kp_yaw': 0.1,
            'kd_yaw': 0.0,
            'max_yaw_cmd': 0.1,
            'odom_in_body_frame': True,
            'control_rate': 20.0,
        }]
    )

    ld.add_action(SUPER)
    ld.add_action(control)

    return ld
