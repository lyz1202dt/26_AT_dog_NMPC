import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    package_description = LaunchConfiguration("pkg_description").perform(context)

    pkg_path = get_package_share_directory(package_description)
    xacro_file = os.path.join(pkg_path, "xacro", "robot.xacro")
    mujoco_model_path = os.path.join(pkg_path, "model", "scene.xml")
    controller_config_file=os.path.join(pkg_path, "config", "mujoco_controller_test.yaml")

    robot_description = xacro.process_file(
        xacro_file,
        mappings={"SIMULATE": "true"},
    ).toxml()

    mujoco_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        output="screen",
        parameters=[
            {"robot_description":robot_description},
            controller_config_file,
            {"simulation_frequency": 500.0},
            {"realtime_factor": 1.0},
            {"robot_model_path": mujoco_model_path},
            {"show_gui": True},
        ],
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ]
    )


    leg_pd_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "leg_pd_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    actions = [
        mujoco_node,
        leg_pd_controller,
    ]

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pkg_description",
                default_value="at_dog_description",
                description="Package that provides the robot xacro/model/config files.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Whether to open RViz2 with the robot model display.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
