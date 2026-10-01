#!/usr/bin/env python3
"""
real_gap.launch.py — follow_the_gap on the REAL car: Hokuyo lidar + bridge +
reactive_node. Untested; build it after steering_test.launch.py works.

    ros2 launch mcu_bridge real_gap.launch.py backend:=serial
    ros2 launch mcu_bridge real_gap.launch.py backend:=serial steering_only:=false   # wheels MOVE

===========================================================================
WHY IT'S NEEDED
    follow_the_gap's own launch file starts ONLY reactive_node, because in the
    simulator gym_bridge supplies the lidar, the odometry and the car. On the
    real car those three come from somewhere else, so you have to start them.

WHAT THIS ONE MUST START
    1. the Hokuyo lidar driver (urg_node2)  -> publishes /scan
    2. bridge_node                          -> takes /drive, publishes /odom
    3. reactive_node                        -> /scan in, /drive out (unchanged code)

THE LIDAR: A LIFECYCLE NODE (this is the fiddly part)
    urg_node2 is a "lifecycle" node. It starts UNCONFIGURED and publishes
    nothing until something tells it to `configure` (connect to the sensor) and
    then `activate` (start publishing). No one does that automatically, so
    this file does it with timers that run ROS CLI commands:
        LifecycleNode(package='urg_node2', executable='urg_node2_node',
                      name='urg_node2_node', namespace='', output='screen',
                      parameters=[{...}], remappings=[('scan', '/scan')])
        TimerAction(period=4.0, actions=[ExecuteProcess(
            cmd=['ros2','lifecycle','set','/urg_node2_node','configure'], output='screen')])
        TimerAction(period=7.0, actions=[ExecuteProcess(
            cmd=['ros2','lifecycle','set','/urg_node2_node','activate'], output='screen')])
    Why the delays: the lidar takes a few seconds to come up over Ethernet, and
    `configure` has to happen after that. The 4 s / 7 s figures come from your old
    autonomy.launch.py (see git history) and were tuned for the Jetson. A Pi 3 is
    slower, so if `configure` fails ("could not connect"), lengthen them.
    Lidar parameters from that same file:
        'ip_address': '192.168.0.10'   'ip_port': 10940   'frame_id': 'laser'
        'angle_min': -2.0944   'angle_max': 2.0944       (radians, about +-120 deg)
    reactive_node keeps only +-fov_deg (70) around straight ahead, so the scan
    must at least cover that. Angle 0 must be the FRONT of the car; if the lidar
    is mounted rotated, everything steers wrong.
    The Pi's Ethernet port needs an address on the lidar's subnet (192.168.0.x).

THE BRIDGE: same Node(...) as in steering_test.launch.py, except:
    - steering_only comes from a launch argument that DEFAULTS TO 'true', so the
      default run cannot spin the wheels. You have to opt in with steering_only:=false.
    - A LaunchConfiguration is a string, and a boolean parameter needs a real
      bool, so wrap it:  ParameterValue(LaunchConfiguration('steering_only'), value_type=bool)
      (from launch_ros.parameter_descriptions import ParameterValue)

reactive_node: three things to get right
    - Its params live in ANOTHER package's share dir:
        get_package_share_directory('follow_the_gap') + '/config/gap_params.yaml'
    - Node name must be 'reactive_node' (that's the yaml's top-level key).
    - reactive_node subscribes to the SIM's odom topic, hardcoded. Redirect it,
      without editing its code, with a remapping:
        remappings=[('/ego_racecar/odom', '/odom')]
      Now what it thinks is the sim's odom is really the bridge's /odom.
      (Also pass parameters=[gap_params], output='screen', emulate_tty=True.)
===========================================================================
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # TODO 1 — parameter files: bridge_params.yaml (mcu_bridge share dir) and
    #          gap_params.yaml (follow_the_gap share dir). See steering_test.launch.py.
    
    mcu_bridge_dir = get_package_share_directory("mcu_bridge")
    bridge_config = os.path.join(mcu_bridge_dir, "config", "bridge_params.yaml")

    # TODO 2 — launch arguments:
    #   backend        default 'serial'
    #   serial_port    default '/dev/ttyACM0'
    #   steering_only  default 'true'    (a STRING; see ParameterValue note above)
    backend_node = Node(
        package="mcu_bridge",
        executable="backend",
        name="backend",
        parameters=[bridge_config]
    )
    

    # TODO 3 — the lidar: LifecycleNode + the two TimerActions (configure, activate).

    # TODO 4 — the bridge Node (parameters=[bridge_params, {overrides}]).

    # TODO 5 — reactive_node with the odom remapping.

    # TODO 6 — return LaunchDescription([...]) with the arguments, then the lidar
    #          and its two timers, then the bridge and reactive_node.
    return LaunchDescription([
        backend_node
    ])
