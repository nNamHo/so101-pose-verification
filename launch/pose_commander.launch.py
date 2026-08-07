"""
Launch: hardware bringup + our pose_commander node.

Modelled on the repo's so101_bringup/launch/moveit_py_test.launch.py, which is a
proven-working MoveItPy setup for this robot. Deviations from it are marked OURS.

Usage:
  ros2 launch so101_pose_milestone pose_commander.launch.py hardware_type:=mock
  ros2 launch so101_pose_milestone pose_commander.launch.py            # real hardware
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

THIS_PACKAGE = "so101_pose_milestone"


def generate_launch_description():
    hardware_type = LaunchConfiguration("hardware_type")
    namespace = LaunchConfiguration("namespace")
    joint_config_file = LaunchConfiguration("joint_config_file")
    usb_port = LaunchConfiguration("usb_port")

    use_sim_time = PythonExpression(["'", hardware_type, "' == 'mujoco'"])

    xacro_path = os.path.join(
        get_package_share_directory("so101_description"),
        "urdf",
        "so101_arm.urdf.xacro",
    )

    moveit_config = (
        MoveItConfigsBuilder("so101_arm", package_name="so101_moveit_config")
        # variant=follower selects the right arm; use_ros2_control=false because
        # ros2_control is stood up separately by follower_split.launch.py below --
        # including it here would instantiate the hardware interface twice.
        .robot_description(
            file_path=xacro_path,
            mappings={"variant": "follower", "use_ros2_control": "false"},
        )
        .robot_description_semantic()
        .robot_description_kinematics()
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .joint_limits()
        # REQUIRED for execute() to know which controller receives the trajectory.
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        # OURS: our own MoveItPy params (conservative velocity caps for first runs).
        # Swap to so101_moveit_config/config/moveit_py_config.yaml to use the repo's.
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory(THIS_PACKAGE),
                "config",
                "moveit_py_params.yaml",
            )
        )
        .to_moveit_configs()
    )

    # 1) Hardware bringup: ros2_control + robot_state_publisher + controller spawners
    follower_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("so101_bringup"),
                "launch",
                "follower_split.launch.py",
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "hardware_type": hardware_type,
            "joint_config_file": joint_config_file,
            "usb_port": usb_port,
            "enable_static_cam": "false",   # OURS: no cameras needed for this milestone
            "enable_wrist_cam": "false",
            "use_rviz": "false",
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # 2) Static TF: world -> base_link, required by MoveIt's virtual joint (SRDF).
    #    Planning fails confusingly without this.
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # 3) OURS: the pose commander node.
    #    NOTE: MoveItPy spawns its own internal C++ node that does NOT inherit
    #    launch-level remappings. The joint_states topic is therefore set via
    #    planning_scene_monitor_options.joint_state_topic in our YAML. If the
    #    planning scene still reports no state, see how the repo's
    #    so101_moveit_config/scripts/so101_moveit_test.py passes remappings=
    #    to the MoveItPy constructor and mirror that.
    pose_commander = Node(
        package=THIS_PACKAGE,
        executable="pose_commander",
        name="pose_commander",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_type", default_value="real"),
            DeclareLaunchArgument("namespace", default_value="follower"),
            DeclareLaunchArgument("joint_config_file", default_value=""),
            # Repo default is /dev/so101_follower (a udev symlink). Use the raw
            # udev symlink (99-so101.rules). Override with usb_port:=/dev/ttyACM0 if absent.
            DeclareLaunchArgument("usb_port", default_value="/dev/so101_follower"),
            # Repo default is /dev/so101_follower (a udev symlink). Set to the
            # raw device unless you create that rule. Override at runtime with
            follower_bringup,
            static_tf,
            # Delay the commander so ros2_control has time to activate its
            # controllers and publish /joint_states first. Without this the
            # commander can start planning against an empty planning scene.
            TimerAction(period=10.0, actions=[pose_commander]),
        ]
    )