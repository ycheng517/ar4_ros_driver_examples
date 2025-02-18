import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitution import Substitution
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


class ControllerConfigSubstitution(Substitution):
    """Substitution that fills out tf_prefix in controllers.yaml."""

    def __init__(self,
                 file_path: Substitution,
                 tf_prefix: Substitution | str = ""):
        super().__init__()
        self._file_path = file_path
        self._tf_prefix = tf_prefix

    def perform(self, context):
        # Evaluate the file path and namespace substitutions
        file_path_val = self._file_path.perform(context)
        if isinstance(self._tf_prefix, str):
            tf_prefix_val = self._tf_prefix
        else:
            tf_prefix_val = self._tf_prefix.perform(context)

        with open(file_path_val, "r") as f:
            content = f.read()

        content = content.replace('$(var tf_prefix)', tf_prefix_val)

        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".yaml")
        temp_file.write(content.encode("utf-8"))
        temp_file.close()
        return temp_file.name


def generate_launch_description():
    ar_model_arg = DeclareLaunchArgument("ar_model",
                                         default_value="mk3",
                                         choices=["mk1", "mk2", "mk3"],
                                         description="Model of AR4")
    ar_model_config = LaunchConfiguration("ar_model")

    # Gazebo nodes
    world = os.path.join(get_package_share_directory('annin_ar4_gazebo'),
                         'worlds', 'empty.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch", "/gz_sim.launch.py"]),
        launch_arguments={
            'gz_args':
            f'-r -v 4 {world}',
            'on_exit_shutdown': 'True'
        }.items())

    stuff_to_spawn = []
    for namespace in ["/left", "/right"]:
        initial_joint_controllers = ControllerConfigSubstitution(
            PathJoinSubstitution([
                FindPackageShare("annin_ar4_driver"), "config",
                "controllers.yaml"
            ]))

        robot_description_content = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("annin_ar4_description"),
                "urdf",
                "ar_gazebo.urdf.xacro",
            ]),
            " ",
            "ar_model:=",
            ar_model_config,
            " ",
            f"namespace:={namespace}",
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
        ])
        robot_description = {"robot_description": robot_description_content}

        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
            namespace=namespace,
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        )

        joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster", "-c",
                f"{namespace}/controller_manager"
            ],
            namespace=namespace,
        )

        # There may be other controllers of the joints, but this is the initially-started one
        initial_joint_controller_spawner_started = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_trajectory_controller",
                "--param-file",
                initial_joint_controllers,
                "-c",
                f"{namespace}/controller_manager",
            ],
        )

        gripper_joint_controller_spawner_started = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "gripper_controller",
                "--param-file",
                initial_joint_controllers,
                "-c",
                f"{namespace}/controller_manager",
            ],
        )

        x_pos = "0.5" if namespace == "/left" else "-0.5"
        gazebo_spawn_robot = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", f"{namespace}_arm", "-topic",
                f"{namespace}/robot_description", "-x", x_pos, "-y", "0", "-z",
                "0"
            ],
            output="screen",
            namespace=namespace,
        )

        gazebo_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
            output='screen',
            namespace=namespace,
        )

        stuff_to_spawn.extend([
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            initial_joint_controller_spawner_started,
            gripper_joint_controller_spawner_started,
            gazebo_spawn_robot,
            gazebo_bridge,
        ])

    return LaunchDescription([
        ar_model_arg,
        gazebo,
        *stuff_to_spawn,
    ])
