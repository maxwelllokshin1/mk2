import os

import launch
from launch import LaunchDescription # required for launch
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction # the actions that launch file uses
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node # required to state nodes
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ///////// DIRECTORIES TO LOOK AT \\\\\\\\\
    pi_dir = get_package_share_directory('pi_drive')
    gap_dir = get_package_share_directory('follow_the_gap')
    urg_dir = get_package_share_directory('urg_node2')


    # PARAMETERS
    params_file = LaunchConfiguration('params_file')
    
    
    # PI Node
    gpio_driver = Node(
        package='pi_drive', 
        executable='gpio_driver_node', 
        name='gpio_driver', 
        output='screen', 
        emulate_tty=True, 
        parameters=[params_file]
        )

    # joycon startup!!
    teleop = Node(
        package='pi_drive', 
        executable='joycon_teleop_node', 
        name='joycon_teleop',
        output='screen', 
        emulate_tty=True, 
        parameters=[params_file]
        )

    # movement node
    reactive = Node(
        package='follow_the_gap', 
        executable='reactive_node', 
        name='reactive_node',
        output='screen', 
        emulate_tty=True,
        parameters=[os.path.join(gap_dir, 'config', 'gap_params.yaml')],
        condition=IfCondition(LaunchConfiguration('autonomy')) # deadman switch 
        )

    # dashboard
    dashboard = Node(
        package='follow_the_gap', 
        executable='car_dashboard', 
        name='car_dashboard',
        output='screen', 
        condition=IfCondition(LaunchConfiguration('dashboard'))
        )

    # for the eyes
    oled = Node(
        package='pi_drive', 
        executable='oled_display_node', 
        name='oled_display',
        output='screen', 
        parameters=[params_file],
        condition=IfCondition(LaunchConfiguration('oled'))
        )

    # detect people using .onyx file
    # low priority so doesnt starve the memory
    person = Node(
        package='pi_drive', 
        executable='person_node', 
        name='person_detector',
        output='screen', 
        parameters=[params_file], 
        prefix='nice -n 10',
        condition=IfCondition(LaunchConfiguration('person'))
        )

    # LIDAR node (+-120 degs)
    lidar = LifecycleNode(
        package='urg_node2', 
        executable='urg_node2_node', 
        name='urg_node2',
        namespace='', 
        output='screen',
        parameters=[
            os.path.join(urg_dir, 'config', 'ust10lx.yaml'),
            {'angle_min': -2.0944, 'angle_max': 2.0944}
            ],
        remappings=[('scan', '/scan')],
        condition=IfCondition(LaunchConfiguration('lidar'))
        )

    # used to configure multiple times since pi might be slow
    def configure_at(period):
        return TimerAction(period=period, 
                           actions=[EmitEvent(
                               event=ChangeState(lifecycle_node_matcher=launch.events.matches_action(lidar),
                                                 transition_id=Transition.TRANSITION_CONFIGURE
                                                 ),
                               condition=IfCondition(LaunchConfiguration('lidar'))
                               )]
                           )
    
    configure_attempts = [configure_at(t) for t in (4.0, 15.0, 30.0)] # run for these intervals (4seconds, 15seconds, 30 seconds)
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=lidar, 
        goal_state='inactive',
        entities=[
            EmitEvent(
                event=ChangeState(
                lifecycle_node_matcher=launch.events.matches_action(lidar),
                transition_id=Transition.TRANSITION_ACTIVATE))])
                                    )

    return LaunchDescription([
        # PARAM
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('autonomy', default_value='true'),
        DeclareLaunchArgument('dashboard', default_value='true'),
        DeclareLaunchArgument('oled', default_value='true'),
        DeclareLaunchArgument('person', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pi_dir, 'config', 'pi_drive.yaml')),
        # STARTUP NODES
        gpio_driver, 
        teleop, 
        reactive, 
        dashboard,
        oled, 
        person, 
        lidar, 
        activate, 
        *configure_attempts,
    ])
