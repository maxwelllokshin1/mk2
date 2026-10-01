#!/usr/bin/env python3
"""
steering_test.launch.py — start everything needed for the steering-only test
with ONE command:

    ros2 launch mcu_bridge steering_test.launch.py                       # dummy: just logs
    ros2 launch mcu_bridge steering_test.launch.py backend:=serial serial_port:=/dev/ttyACM0
    ros2 launch mcu_bridge steering_test.launch.py backend:=gpio

===========================================================================
WHAT A LAUNCH FILE IS
    A Python file ROS runs to start several nodes at once, each with the
    parameters and settings you specify. Instead of opening two terminals and
    typing `ros2 run ...` twice, then remembering the right --ros-args every
    time, you describe it here once. `ros2 launch <pkg> <file>` imports this
    file, calls generate_launch_description(), and executes the actions in the
    LaunchDescription it returns.
    IMPORTANT: everything you list starts AT THE SAME TIME. The order in the list
    is not a startup order. (real_gap.launch.py uses TimerActions when order matters.)

WHAT THIS ONE MUST START
    1. bridge_node    from mcu_bridge      -- with steering_only forced True
    2. steering_sweep from mcu_bridge      -- the fake reactive_node

HOW IT INSTALLS (why editing this file sometimes "does nothing")
    setup.py copies launch/*.launch.py and config/*.yaml into
    install/mcu_bridge/share/mcu_bridge/. `ros2 launch` runs the INSTALLED copy,
    so after editing this file or the yaml you must re-run
    `colcon build --packages-select mcu_bridge` (or build once with
    --symlink-install, which links to your source instead of copying).
===========================================================================
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # TODO 1 — find the parameter file.
    #   share = get_package_share_directory('mcu_bridge')
    #   params = os.path.join(share, 'config', 'bridge_params.yaml')
    #   (That returns install/mcu_bridge/share/mcu_bridge, where setup.py put the yaml.)

    # TODO 2 — launch arguments, so `backend:=serial` works on the command line.
    #   DeclareLaunchArgument('backend', default_value='dummy')
    #   DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0')
    #   LaunchConfiguration('backend') is a PLACEHOLDER resolved when the launch
    #   actually runs, not a Python string you can print or compare now. You pass
    #   it straight into a Node's parameters and ROS substitutes it later.

    # TODO 3 — the bridge node:
    #   Node(
    #       package='mcu_bridge',
    #       executable='bridge_node',    # the key in setup.py's console_scripts
    #       name='mcu_bridge',           # MUST equal the top-level key in bridge_params.yaml,
    #                                    # or ROS silently ignores the whole yaml
    #       output='screen',             # print its logs in this terminal
    #       emulate_tty=True,            # flush logs line by line instead of in bursts
    #       parameters=[params, {...}],
    #   )
    #   `parameters` is a LIST; later entries override earlier ones. So: the
    #   yaml first (your defaults), then a dict of overrides from the command line:
    #       {'backend': LaunchConfiguration('backend'),
    #        'serial_port': LaunchConfiguration('serial_port'),
    #        'steering_only': True}       # forced on: this launch must never move the wheels

    # TODO 4 — the fake driver:
    #   Node(package='mcu_bridge', executable='steering_sweep',
    #        name='steering_sweep', output='screen', emulate_tty=True)

    # TODO 5 — return everything, arguments included:
    #   LaunchDescription([<the two DeclareLaunchArguments>, bridge, sweep])
    return LaunchDescription([])
