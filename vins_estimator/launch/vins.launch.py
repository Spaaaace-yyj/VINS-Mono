import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('vins_estimator')

    config_file = LaunchConfiguration('config_file')
    vins_folder = LaunchConfiguration('vins_folder')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(pkg_dir, 'config', 'euroc', 'euroc_config.yaml'),
            description='Absolute path to the VINS config YAML file'),
        DeclareLaunchArgument(
            'vins_folder',
            default_value=pkg_dir + '/',
            description='VINS root folder used to locate support files (e.g. fisheye mask)'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 for visualization'),

        Node(
            package='feature_tracker',
            executable='feature_tracker',
            name='feature_tracker',
            output='screen',
            parameters=[{
                'config_file': config_file,
                'vins_folder': vins_folder,
            }],
        ),

        Node(
            package='vins_estimator',
            executable='vins_estimator',
            name='vins_estimator',
            output='screen',
            parameters=[{
                'config_file': config_file,
            }],
        ),

        Node(
            package='pose_graph',
            executable='pose_graph',
            name='pose_graph',
            output='screen',
            parameters=[{
                'config_file': config_file,
                'visualization_shift_x': 0,
                'visualization_shift_y': 0,
                'skip_cnt': 0,
                'skip_dis': 0.0,
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'vins_rviz2_config.rviz')],
            condition=IfCondition(use_rviz),
        ),
    ])
