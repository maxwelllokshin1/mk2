#!/usr/bin/env python3
"""
Everything the car runs, in one launch (one process each, nothing extra):

    joycon_teleop  --/drive_cmd-->  gpio_driver  --> servo + ESC pins
         ^                              |
         | /drive (autonomy)            +--> /ego_racecar/odom (dead reckoned)
    reactive_node <-- /scan <-- urg_node2 (Hokuyo)

Args:
    lidar:=false       skip the Hokuyo (RC only, or no LiDAR plugged in)
    autonomy:=false    skip reactive_node (R then does nothing useful)
    dashboard:=false   skip the web dashboard (http://<pi-ip>:8080)
    params_file:=...   override pi_drive.yaml
"""
import os

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pi_dir = get_package_share_directory('pi_drive')
    gap_dir = get_package_share_directory('follow_the_gap')
    urg_dir = get_package_share_directory('urg_node2')

    params_file = LaunchConfiguration('params_file')

    gpio_driver = Node(
        package='pi_drive', executable='gpio_driver_node', name='gpio_driver',
        output='screen', emulate_tty=True, parameters=[params_file])

    teleop = Node(
        package='pi_drive', executable='joycon_teleop_node', name='joycon_teleop',
        output='screen', emulate_tty=True, parameters=[params_file])

    reactive = Node(
        package='follow_the_gap', executable='reactive_node', name='reactive_node',
        output='screen', emulate_tty=True,
        parameters=[os.path.join(gap_dir, 'config', 'gap_params.yaml')],
        condition=IfCondition(LaunchConfiguration('autonomy')))

    dashboard = Node(
        package='follow_the_gap', executable='car_dashboard', name='car_dashboard',
        output='screen', condition=IfCondition(LaunchConfiguration('dashboard')))

    # Hokuyo is a lifecycle node: it publishes nothing until configured + activated.
    # The default config in the image is +-12 deg, which is far too narrow for
    # follow_the_gap (it uses +-70 deg), so widen to +-120 deg like the old autonomy.launch.
    lidar = LifecycleNode(
        package='urg_node2', executable='urg_node2_node', name='urg_node2',
        namespace='', output='screen',
        parameters=[os.path.join(urg_dir, 'config', 'ust10lx.yaml'),
                    {'angle_min': -2.0944, 'angle_max': 2.0944}],
        remappings=[('scan', '/scan')],
        condition=IfCondition(LaunchConfiguration('lidar')))

    # Try to configure a few times: on a slow Pi the sensor may not be up at 4 s.
    # (A configure on an already-configured node just logs an error, harmless.)
    def configure_at(period):
        return TimerAction(period=period, actions=[EmitEvent(
            event=ChangeState(lifecycle_node_matcher=launch.events.matches_action(lidar),
                              transition_id=Transition.TRANSITION_CONFIGURE),
            condition=IfCondition(LaunchConfiguration('lidar')))])
    configure_attempts = [configure_at(t) for t in (4.0, 15.0, 30.0)]
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=lidar, goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(lidar),
            transition_id=Transition.TRANSITION_ACTIVATE))]))

    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('autonomy', default_value='true'),
        DeclareLaunchArgument('dashboard', default_value='true'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(pi_dir, 'config', 'pi_drive.yaml')),
        gpio_driver, teleop, reactive, dashboard, lidar, activate, *configure_attempts,
    ])
