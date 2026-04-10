import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile


def controller_spawner(name, active=True):
    args = [name, "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30"]
    if not active:
        args.append("--inactive")
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=args,
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )


def launch_setup(context, *args, **kwargs):
    ur_type           = LaunchConfiguration("ur_type")
    safety_limits     = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    prefix            = LaunchConfiguration("prefix")
    use_sim_time      = LaunchConfiguration("use_sim_time")
    launch_rviz       = LaunchConfiguration("launch_rviz")
    isaac_joint_commands = LaunchConfiguration("isaac_joint_commands")
    isaac_joint_states   = LaunchConfiguration("isaac_joint_states")

    ur_description_package    = "ur_description"
    description_package       = "ur3_suction_description"
    description_file          = "ur3_suction.urdf.xacro"

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "visual_parameters.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " joint_limit_params:=", joint_limit_params,
            " kinematics_params:=",  kinematics_params,
            " physical_params:=",    physical_params,
            " visual_params:=",      visual_params,
            " safety_limits:=",      safety_limits,
            " safety_pos_margin:=",  safety_pos_margin,
            " safety_k_position:=",  safety_k_position,
            " name:=ur",
            " ur_type:=",            ur_type,
            " isaac_joint_commands:=", isaac_joint_commands,
            " isaac_joint_states:=",   isaac_joint_states,
            " tf_prefix:=",          prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    controllers_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "ur3_suction_controllers_isaac.yaml"]
    )

    # ros2_control node: loads hw interface (TopicBasedSystem) + controller manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ParameterFile(controllers_file, allow_substs=True),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    # Publishes TF from URDF joint states
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(launch_rviz),
        arguments=["--ros-args", "--log-level", "error"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return [
        control_node,
        robot_state_publisher,
        rviz_node,
        controller_spawner("joint_state_broadcaster"),
        controller_spawner("joint_trajectory_controller"),
        controller_spawner("forward_position_controller", active=False),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type", default_value="ur3",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e"],
        ),
        DeclareLaunchArgument("safety_limits",       default_value="true"),
        DeclareLaunchArgument("safety_pos_margin",   default_value="0.15"),
        DeclareLaunchArgument("safety_k_position",   default_value="20"),
        DeclareLaunchArgument("prefix",              default_value=""),
        DeclareLaunchArgument("use_sim_time",        default_value="true"),
        DeclareLaunchArgument("launch_rviz",         default_value="false"),
        DeclareLaunchArgument(
            "isaac_joint_commands", default_value="/joint_command",
            description="Topic Isaac Sim subscribes to for commands.",
        ),
        DeclareLaunchArgument(
            "isaac_joint_states", default_value="/joint_states",
            description="Topic Isaac Sim publishes joint states on.",
        ),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
