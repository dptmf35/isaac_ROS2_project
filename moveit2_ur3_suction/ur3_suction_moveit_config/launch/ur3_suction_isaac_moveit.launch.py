import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def launch_setup(context, *args, **kwargs):
    ur_type             = LaunchConfiguration("ur_type")
    safety_limits       = LaunchConfiguration("safety_limits")
    safety_pos_margin   = LaunchConfiguration("safety_pos_margin")
    safety_k_position   = LaunchConfiguration("safety_k_position")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    prefix              = LaunchConfiguration("prefix")
    use_sim_time        = LaunchConfiguration("use_sim_time")
    launch_rviz         = LaunchConfiguration("launch_rviz")
    isaac_joint_commands = LaunchConfiguration("isaac_joint_commands")
    isaac_joint_states   = LaunchConfiguration("isaac_joint_states")

    ur_description_package  = "ur_description"
    description_package     = "ur3_suction_description"
    description_file        = "ur3_suction.urdf.xacro"
    moveit_config_package   = "ur3_suction_moveit_config"
    moveit_config_file      = "ur3_suction.srdf.xacro"

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

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(moveit_config_package), "srdf", moveit_config_file]),
            " name:=ur",
            " prefix:=", prefix,
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(moveit_config_package, "config/kinematics.yaml")
    }

    # --- Planning pipeline (Humble MoveIt2 2.5.x format) ---
    # "move_group" key nests plugin config so it resolves to move_group.planning_plugin
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(moveit_config_package, "config/ompl_planning.yaml")
    if ompl_planning_yaml:
        ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    controllers_yaml = load_yaml(moveit_config_package, "config/moveit_controllers_isaac.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "rviz", "view_robot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "error"],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    return [move_group_node, rviz_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type", default_value="ur3",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e"],
        ),
        DeclareLaunchArgument("safety_limits",       default_value="true"),
        DeclareLaunchArgument("safety_pos_margin",   default_value="0.15"),
        DeclareLaunchArgument("safety_k_position",   default_value="20"),
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
        ),
        DeclareLaunchArgument("use_sim_time",  default_value="true"),
        DeclareLaunchArgument("prefix",        default_value=""),
        DeclareLaunchArgument("launch_rviz",   default_value="true"),
        DeclareLaunchArgument(
            "isaac_joint_commands", default_value="/joint_command",
        ),
        DeclareLaunchArgument(
            "isaac_joint_states", default_value="/joint_states",
        ),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
