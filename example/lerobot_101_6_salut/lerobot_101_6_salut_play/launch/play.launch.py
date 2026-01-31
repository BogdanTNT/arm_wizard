from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path

def _spawner(name: str) -> Node:
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[name],
        output="screen",
    )

def generate_launch_description():
    root = Path(get_package_share_directory("lerobot_101_6_salut_play"))
    overlay = root
    base = Path(get_package_share_directory("lerobot_101_6_salut_moveit_config"))
    desc = Path(get_package_share_directory("lerobot_101_6_salut_description"))

    urdf_xacro = desc / "urdf" / "lerobot_101_6_salut.xacro"
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    moveit_config = (
        MoveItConfigsBuilder("lerobot_101_6_salut", package_name="lerobot_101_6_salut_moveit_config")
        .robot_description(
            file_path=str(urdf_xacro),
            mappings={
                "use_mock_hardware": use_mock_hardware,
                # add any other xacro args your description expects
            },
        )
        .robot_description_semantic(
            file_path=str(base / "config/lerobot_101_6_salut.srdf")
        )
        .robot_description_kinematics(
            file_path=str(base / "config/kinematics.yaml")
        )
        .joint_limits(file_path=str(overlay / "moveit_overlay/config/joint_limits.yaml"))
        .trajectory_execution(file_path=str(overlay / "moveit_overlay/config/moveit_controllers.yaml"))
        .pilz_cartesian_limits(
            file_path=str(base / "config/pilz_cartesian_limits.yaml")
        )
        .to_moveit_configs()
    )

    # robot_state_publisher publishes /robot_description and /tf
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control_node: base yaml + overlay yaml (overlay wins)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            str(base / "config/ros2_controllers.yaml")
        ],
        output="screen",
    )

    # Spawn controllers so MoveIt can execute
    jsb_spawner = _spawner("joint_state_broadcaster")
    arm_spawner = _spawner("arm_controller")
    gr_spawner = _spawner("gr_controller")

    # move_group + rviz + rsp (use moveit_config dict)
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # You can reuse the generated rviz config:
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", str(base / "config/moveit.rviz")],
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock_hardware", default_value="true"),
            rsp,
            ros2_control_node,
            jsb_spawner,
            arm_spawner,
            gr_spawner,
            move_group,
            rviz,
        ]
    )
