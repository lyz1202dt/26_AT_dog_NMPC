import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    package_description = LaunchConfiguration("pkg_description").perform(context)

    pkg_path = get_package_share_directory(package_description)
    xacro_file = os.path.join(pkg_path, "xacro", "robot.xacro")
    mujoco_model_path = os.path.join(pkg_path, "model", "scene.xml")
    controller_config_file = os.path.join(pkg_path, "config", "mujoco_controller_test.yaml")

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={"SIMULATE": "true"})
    robot_description = {'robot_description': doc.toxml()}

    node_mujoco_ros2_control = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        output="screen",
        parameters=[
            robot_description,
            controller_config_file,
            {'mujoco_model_path': mujoco_model_path}
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_imu_sensor_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'imu_sensor_broadcaster'],
        output='screen'
    )

    load_leg_pd_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'leg_pd_controller'],
        output='screen'
    )

    return [
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=node_mujoco_ros2_control,
                on_start=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_imu_sensor_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_imu_sensor_broadcaster,
                on_exit=[load_leg_pd_controller],
            )
        ),
        node_mujoco_ros2_control,
        node_robot_state_publisher
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pkg_description",
                default_value="at_dog_description",
                description="Package that provides the robot xacro/model/config files.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
