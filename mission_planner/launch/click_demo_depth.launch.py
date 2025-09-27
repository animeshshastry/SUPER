import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    super_config_name = 'click_smooth_ros2_depth.yaml'

    package_path = get_package_share_directory('perfect_drone_sim')
    default_rviz_config_path = os.path.join(package_path, 'rviz2', 'unity_sim.rviz')

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
            'kff': 1.0,
            'kp': 0.5,
            'kd': 0.5,
            'z_scale': 2.0,
            'max_pos_err': 10.0,
            'tolerance': 0.0,
            'kp_yaw': -1.0,
            'kff_yaw': -1.0,
            'max_yaw_cmd': 1.0,
            'odom_in_body_frame': True,
            'control_rate': 30.0,
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path],
        output='screen'
    )

    ld.add_action(SUPER)
    ld.add_action(control)
    ld.add_action(rviz_node)

    return ld
