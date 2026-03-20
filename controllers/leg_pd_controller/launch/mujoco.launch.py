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
    use_rviz = LaunchConfiguration("rviz").perform(context).lower()

    pkg_path = get_package_share_directory(package_description)
    xacro_file = os.path.join(pkg_path, "xacro", "robot.xacro")
    mujoco_model_path = os.path.join(pkg_path, "model", "dog.xml")
    rviz_config_file = os.path.join(pkg_path, "config", "visualize_urdf.rviz")

    robot_description = xacro.process_file(
        xacro_file,
        mappings={"SIMULATE": "true"},
    ).toxml()

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_description),
            "config",
            "mujoco_controller_test.yaml",
        ]
    )

    mujoco_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        name="mujoco_ros2_control_node",
        output="screen",
        parameters=[
            robot_controllers,
            {
                "robot_description": robot_description,
                "mujoco_model_path": mujoco_model_path,
                "use_sim_time": True,
            },
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "publish_frequency": 100.0,
                "use_tf_static": True,
                "ignore_timestamp": True,
                "use_sim_time": True,
                "robot_description": robot_description,
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
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
        robot_state_publisher,
        joint_state_broadcaster,
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[imu_sensor_broadcaster],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=imu_sensor_broadcaster,
                on_exit=[leg_pd_controller],
            )
        ),
    ]

    if use_rviz in ("true", "1", "yes"):
        actions.insert(2, rviz)

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
