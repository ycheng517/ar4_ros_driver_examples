import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_name):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_name)
    with open(absolute_file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    # Common arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    include_gripper = LaunchConfiguration("include_gripper")
    ar_model_config = LaunchConfiguration("ar_model")
    tf_prefix = LaunchConfiguration("tf_prefix")

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Use simulation time",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "include_gripper",
            default_value="True",
            choices=["True", "False"],
            description="Whether to include the gripper",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ar_model",
            default_value="mk3",
            choices=["mk1", "mk2", "mk3"],
            description="Model for left arm",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="Prefix for AR4 tf_tree",
        ))

    # Shared MoveIt configuration
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "annin_ar4_moveit_config",
            os.path.join("config", "kinematics.yaml")
        )
    }

    joint_limits = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare("annin_ar4_moveit_config"),
            "config",
            "joint_limits.yaml"
        ]),
        allow_substs=True,
    )

    ompl_planning_yaml = load_yaml("annin_ar4_moveit_config", "config/ompl_planning.yaml")
    pilz_planning_yaml = load_yaml("annin_ar4_moveit_config", "config/pilz_planning.yaml")
    planning_pipeline_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl", "pilz"],
        "ompl": ompl_planning_yaml,
        "pilz": pilz_planning_yaml,
    }

    moveit_controller_manager = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"
    }

    moveit_controllers = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare("annin_ar4_moveit_config"),
            "config",
            "controllers.yaml"
        ]),
        allow_substs=True,
    )

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
        "publish_robot_description_semantic": True,
    }

    move_group_capabilities = {
        "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService"
    }

    rviz_config_file = PathJoinSubstitution([
                FindPackageShare("annin_ar4_moveit_config"),
                "rviz",
                "moveit.rviz"
            ])

    # Create nodes for each arm in a loop
    nodes = []
    for arm in ["left", "right"]:
        # Robot description using xacro
        robot_description_content = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("annin_ar4_description"),
                "urdf",
                "ar.urdf.xacro"
            ]),
            " ",
            "ar_model:=",
            ar_model_config,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "include_gripper:=",
            include_gripper,
        ])
        robot_description = {"robot_description": robot_description_content}

        # Semantic robot description using xacro
        robot_description_semantic_content = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("annin_ar4_moveit_config"),
                "srdf",
                "ar.srdf.xacro"
            ]),
            " ",
            "name:=",
            ar_model_config,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "include_gripper:=",
            include_gripper,
        ])
        robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

        # Move group node for the arm
        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            namespace=arm,
            name=f"move_group_{arm}",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                joint_limits,
                planning_pipeline_config,
                trajectory_execution,
                moveit_controller_manager,
                moveit_controllers,
                planning_scene_monitor_parameters,
                move_group_capabilities,
                {"use_sim_time": use_sim_time},
            ],
        )
        nodes.append(move_group_node)

        # RViz node for the arm
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            namespace=arm,
            name=f"rviz2_moveit_{arm}",
            output="log",
            arguments=["-d", rviz_config_file],
            parameters=[
                robot_description,
                robot_description_semantic,
                planning_pipeline_config,
                robot_description_kinematics,
                {"use_sim_time": use_sim_time},
            ],
        )
        nodes.append(rviz_node)

    return LaunchDescription(declared_arguments + nodes)
