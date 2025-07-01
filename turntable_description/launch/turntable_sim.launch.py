from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    description_package = "turntable_description"
    description_file = "turntable.urdf.xacro"
    moveit_package = "turntable_moveit_config"

    use_fake_hardware = "true"
    headless_mode = "false"
    initial_positions_file = PathJoinSubstitution([
        FindPackageShare(description_package),
        "config", "turntable", "initial_positions.yaml"
    ])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare(description_package),
            "urdf", description_file
        ]),
        " ",
        "name:=turntable",
        " ",
        f"use_fake_hardware:={use_fake_hardware}",
        " ",
        f"headless_mode:={headless_mode}",
        " ",
        "initial_positions_file:=", initial_positions_file,
    ])

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    rviz_config = PathJoinSubstitution([
        FindPackageShare(description_package), "rviz", "view_robot.rviz"
    ])

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare(description_package),
        "config", "turntable", "turntable_controllers.yaml"
    ])

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(moveit_package),
                "launch",
                "move_group.launch.py"
            ])
        ]),
        launch_arguments={
            'planning_pipeline': 'ompl',
            'sensor_manager': 'false',
            'octomap_manager': 'false',
            'use_octomap': 'false',
            'use_fake_hardware': use_fake_hardware,
            'capabilities': '',
        }.items()
    )   

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen"
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[controllers_yaml],
            output="screen",
            remappings=[("/controller_manager/robot_description", robot_description)]
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen"
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["turntable_trajectory_controller", "--controller-manager", "/controller_manager"],
            output="screen"
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            name="spawner_turntable_forward_position_controller",
            arguments=[
                "turntable_forward_position_controller",
                "--controller-manager", "/controller_manager",
                "--inactive"  # Loads but does not activate
            ],
            output="screen"
        ),              
        move_group_launch,
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen"
        )        
    ])
