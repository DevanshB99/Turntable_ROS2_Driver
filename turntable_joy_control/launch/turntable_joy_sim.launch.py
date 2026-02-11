"""Standalone launch for turntable jog control in simulation.

Run the turntable sim launch first:
  ros2 launch turntable_description turntable_sim.launch.py

Then run this (keyboard mode by default):
  ros2 launch turntable_joy_control turntable_joy_sim.launch.py

Or for joy mode:
  ros2 launch turntable_joy_control turntable_joy_sim.launch.py input_mode:=joy
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('turntable_joy_control')

    input_mode_arg = DeclareLaunchArgument(
        'input_mode',
        default_value='keyboard',
        description='Input mode: "keyboard" or "joy"',
    )

    input_mode = LaunchConfiguration('input_mode')

    keyboard_config = PathJoinSubstitution([
        pkg_share, 'config', 'turntable_keyboard.yaml',
    ])

    joy_config = PathJoinSubstitution([
        pkg_share, 'config', 'turntable_joy.yaml',
    ])

    is_keyboard = PythonExpression(["'", input_mode, "' == 'keyboard'"])

    # Keyboard mode node - launched in xterm so it can capture key input
    keyboard_node = Node(
        package='turntable_joy_control',
        executable='turntable_keyboard_node',
        name='turntable_keyboard_node',
        parameters=[keyboard_config],
        output='screen',
        prefix='xterm -e',
        condition=IfCondition(is_keyboard),
    )

    # Joy mode node - no special terminal needed
    joy_node = Node(
        package='turntable_joy_control',
        executable='turntable_joy_node',
        name='turntable_joy_node',
        parameters=[joy_config],
        output='screen',
        condition=UnlessCondition(is_keyboard),
    )

    return LaunchDescription([
        input_mode_arg,
        keyboard_node,
        joy_node,
    ])
