"""
construction_robot.launch.py - starts the whole system on the lab PC.

  ros2 launch ecte477_cr_final construction_robot.launch.py

Starts: Nav2 + AMCL + RViz (turtlebot3_navigation2, with our map and tuned params),
        sheet_detector, zone_verifier, and destination_selector in its own xterm window
        (a launched node has no keyboard, so the menu needs its own terminal).
Options:
  start_selector:=false    run the menu yourself: ros2 run ecte477_cr_final destination_selector
  start_nav:=false         if Nav2 is already running in another terminal
  camera_topic:=/image_raw prefer_compressed:=true|false
The robot side (bringup + camera) runs on the Raspberry Pi over SSH.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('ecte477_cr_final')
    tb3_nav = get_package_share_directory('turtlebot3_navigation2')

    args = [
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'tb3_project_map.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_nav', default_value='false'),
        DeclareLaunchArgument('start_selector', default_value='false'),
        DeclareLaunchArgument('selector_prefix', default_value='xterm -fa Monospace -fs 11 -geometry 110x35 -e'),
        DeclareLaunchArgument('camera_topic', default_value='/image_raw'),
        DeclareLaunchArgument('prefer_compressed', default_value='true'),
    ]
    cam = {'camera_topic': LaunchConfiguration('camera_topic'),
           'prefer_compressed': LaunchConfiguration('prefer_compressed')}

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_nav, 'launch', 'navigation2.launch.py')),
        launch_arguments={'map': LaunchConfiguration('map'),
                          'params_file': LaunchConfiguration('params_file'),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        condition=IfCondition(LaunchConfiguration('start_nav')))

    sheet = Node(package='ecte477_cr_final', executable='sheet_detector', name='sheet_detector',
                 output='screen', parameters=[cam])
    verifier = Node(package='ecte477_cr_final', executable='zone_verifier', name='zone_verifier',
                    output='screen', parameters=[cam])
    selector = Node(package='ecte477_cr_final', executable='destination_selector',
                    name='destination_selector', output='screen',
                    prefix=LaunchConfiguration('selector_prefix'),
                    condition=IfCondition(LaunchConfiguration('start_selector')))

    return LaunchDescription(args + [nav2, sheet, verifier, selector])
