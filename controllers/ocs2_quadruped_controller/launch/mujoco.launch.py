import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

package_controller = "ocs2_quadruped_controller"


def launch_setup(context, *args, **kwargs):
    package_description = LaunchConfiguration("pkg_description").perform(context)

    pkg_path = get_package_share_directory(package_description)
    xacro_file = os.path.join(pkg_path, "xacro", "robot.xacro")
    mujoco_model_path = os.path.join(pkg_path, "model", "scene.xml")
    controller_config_file = os.path.join(pkg_path, "config", "robot_mujoco_sim.yaml")
    rviz_config_file = os.path.join(get_package_share_directory(package_controller), "config", "visualize_ocs2.rviz")

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

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_ocs2',
        output='screen',
        arguments=["-d", rviz_config_file]
    )

    load_joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
            "--service-call-timeout", "120"
        ],
        output="screen"
    )

    load_imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
            "--service-call-timeout", "120"
        ],
        output="screen"
    )

    # 级联控制器架构：ocs2(上层) -> leg_pd(底层) -> mujoco
    # 先加载下层leg_pd_controller导出reference interfaces，再加载上层ocs2控制器
    load_leg_pd_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "leg_pd_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
            "--service-call-timeout", "120"
        ],
        output="screen"
    )

    load_ocs2_quadruped_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ocs2_quadruped_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
            "--service-call-timeout", "120",
            "--switch-timeout", "120"
        ],
        output="screen"
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
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_leg_pd_controller,
                on_exit=[load_ocs2_quadruped_controller],
            )
        ),
        node_mujoco_ros2_control,
        node_robot_state_publisher,
        node_rviz
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
