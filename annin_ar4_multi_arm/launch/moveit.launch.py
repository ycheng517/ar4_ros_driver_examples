import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def load_yaml(package_name, file_name):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_name)
    with open(absolute_file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    ar_model_config = LaunchConfiguration("ar_model")

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",  # Use simulation time!
            description="Make MoveIt use simulation time. This is needed " +
            "for trajectory planing in simulation.",
        ))

    rviz_config_file_default = PathJoinSubstitution(
        [FindPackageShare("annin_ar4_moveit_config"), "rviz", "moveit.rviz"])
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=rviz_config_file_default,
            description="Full path to the RViz configuration file to use",
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "ar_model",
            default_value="mk3",
            choices=["mk1", "mk2", "mk3"],
            description="Model of AR4",
        ))

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("annin_ar4_description"),
            "urdf",
            "dual_ar4.urdf.xacro"  # Use the dual arm URDF
        ]),
        " ",
        "ar_model:=",
        ar_model_config,
        " ",
        "simulation_controllers:=",
        PathJoinSubstitution([
            FindPackageShare("annin_ar4_driver"), "config", "controllers.yaml"
        ])
    ])
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("annin_ar4_moveit_config"),
            "srdf",
            "ar.srdf.xacro"  #  Use the existing SRDF, it will be adapted.
        ]),
        " ",
        "name:=",
        ar_model_config,
    ])
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    robot_description_kinematics = {
        "robot_description_kinematics":
        load_yaml(
            "annin_ar4_moveit_config",
            os.path.join("config", "kinematics.yaml"),
        )
    }

    # Adding prefixes
    kinematics_config = load_yaml("annin_ar4_moveit_config",
                                  "config/kinematics.yaml")
    kinematics_config["left_ar4"] = dict(kinematics_config["ar_manipulator"])
    kinematics_config["left_ar4"][
        'kinematics_solver'] = 'trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin'
    kinematics_config["left_ar4"][
        'kinematics_solver_base_frame'] = 'left_base_link'
    kinematics_config["right_ar4"] = dict(kinematics_config["ar_manipulator"])
    kinematics_config["right_ar4"][
        'kinematics_solver'] = 'trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin'
    kinematics_config["right_ar4"][
        'kinematics_solver_base_frame'] = 'right_base_link'
    del kinematics_config["ar_manipulator"]
    robot_description_kinematics[
        "robot_description_kinematics"] = kinematics_config

    joint_limits = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare("annin_ar4_moveit_config"),
            "config/joint_limits.yaml"
        ]),
        allow_substs=True,
    )

    # Planning Configuration
    ompl_planning_yaml = load_yaml("annin_ar4_moveit_config",
                                   "config/ompl_planning.yaml")
    pilz_planning_yaml = load_yaml("annin_ar4_moveit_config",
                                   "config/pilz_planning.yaml")

    # Create a dictionary for the planning pipelines
    planning_pipeline_config = {
        'planning_pipelines': ['ompl', 'pilz'],  # Note the change here
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_planning_yaml,
        'pilz': pilz_planning_yaml
    }

    # Update OMPL and PILZ configurations to support two arms (left_ar4 and right_ar4)
    ompl_planning_yaml["left_ar4"] = ompl_planning_yaml["ar_manipulator"]
    ompl_planning_yaml["right_ar4"] = ompl_planning_yaml["ar_manipulator"]
    del ompl_planning_yaml["ar_manipulator"]

    pilz_planning_yaml["left_ar4"] = pilz_planning_yaml["ar_manipulator"]
    pilz_planning_yaml["right_ar4"] = pilz_planning_yaml["ar_manipulator"]
    del pilz_planning_yaml["ar_manipulator"]

    planning_pipeline_config["ompl"] = ompl_planning_yaml
    planning_pipeline_config["pilz"] = pilz_planning_yaml

    # Trajectory Execution Configuration
    moveit_controller_manager = {
        "moveit_controller_manager":
        "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    moveit_controllers = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare("annin_ar4_moveit_config"),
            "config/controllers.yaml"
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
        # Added due to https://github.com/moveit/moveit2_tutorials/issues/528
        "publish_robot_description_semantic": True,
    }

    # Starts Pilz Industrial Motion Planner MoveGroupSequenceAction and MoveGroupSequenceService servers
    move_group_capabilities = {
        "capabilities":
        "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService"
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
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
            {
                "use_sim_time": use_sim_time
            },
        ],
    )

    # rviz with moveit configuration
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            planning_pipeline_config,
            robot_description_kinematics,
            {
                "use_sim_time": use_sim_time
            },
        ],
    )

    nodes_to_start = [move_group_node, rviz_node]
    return LaunchDescription(declared_arguments + nodes_to_start)
