#!/usr/bin/env python3
from functools import partial

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Point, Pose, Quaternion, TwistStamped
from moveit_msgs.srv import ServoCommandType
from pymoveit2 import MoveIt2
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool


class JoystickServo(Node):
    def __init__(self):
        super().__init__("joystick_servo")

        # Constants
        self.gripper_open_position = -0.014
        self.input_to_velocity_linear_scaling = 0.04
        self.input_to_velocity_angular_scaling = 0.3
        self.reset_pose = Pose(
            position=Point(x=0.0, y=-0.35, z=0.30),
            orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
        )

        # State variables
        self._last_joy_msg: Joy | None = None
        self._command_enabled: bool = False
        self._resetting = False

        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.servo_pub = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10
        )

        self.switch_command_type_client = self.create_client(
            ServoCommandType,
            "/servo_node/switch_command_type",
        )
        self.pause_client = self.create_client(
            SetBool,
            "/servo_node/pause_servo",
        )

        self.gripper_action_client = ActionClient(
            self, GripperCommand, "/gripper_controller/gripper_cmd"
        )

        while not self.switch_command_type_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Switch command type service not available, waiting again..."
            )

        while not self.pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Pause servo service not available, waiting again..."
            )

        while not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(
                "Gripper action server not available, waiting again..."
            )

        self.arm_joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=self.arm_joint_names,
            base_link_name="base_link",
            end_effector_name="link_6",
            group_name="ar_manipulator",
            callback_group=ReentrantCallbackGroup(),
        )
        self.moveit2.planner_id = "RRTConnectkConfigDefault"

        self.timer = self.create_timer(
            0.05, self.timer_callback, callback_group=MutuallyExclusiveCallbackGroup()
        )

        # Switch to twist command type
        future = self.switch_command_type_client.call_async(
            ServoCommandType.Request(command_type=ServoCommandType.Request.TWIST)
        )
        future.add_done_callback(
            partial(
                self.service_callback,
                service_name=self.switch_command_type_client.srv_name,
            )
        )

        self._command_enabled = True
        self.get_logger().info("Joystick Servo Node has been started.")

    def joy_callback(self, msg):
        self._last_joy_msg = msg

    def reset_arm(self):
        self._resetting = True
        self._logger.info("Disabling servo control")
        response = self.pause_client.call(SetBool.Request(data=True))
        if not response.success:
            self.get_logger().error("Disable servo failed")
            self._resetting = False
            return

        self._logger.info("Resetting the arm")
        self.moveit2.move_to_pose(self.reset_pose)
        self.moveit2.wait_until_executed()
        self.send_gripper_command(self.gripper_open_position)

        self._logger.info("Enabling servo control")
        response = self.pause_client.call(SetBool.Request(data=False))
        if not response.success:
            self.get_logger().error("Enable servo failed")
            self._resetting = False
            return

        self._resetting = False

    def timer_callback(self):
        if self._last_joy_msg is None:
            return

        if (
            not self._resetting and self._last_joy_msg.buttons[7] == 1
        ):  # Press Start to reset the arm
            self.reset_arm()

        # Press B to disable the command
        if self._command_enabled and self._last_joy_msg.buttons[1] == 1:
            self._logger.info("Disabling servo control")
            future = self.pause_client.call_async(SetBool.Request(data=True))
            future.add_done_callback(
                partial(self.service_callback, service_name=self.pause_client.srv_name)
            )
            self._command_enabled = False
        # Press A to enable the command
        elif not self._command_enabled and self._last_joy_msg.buttons[0] == 1:
            self._logger.info("Enabling servo control")

            # Unpause the servo
            future = self.pause_client.call_async(SetBool.Request(data=False))
            future.add_done_callback(
                partial(self.service_callback, service_name=self.pause_client.srv_name)
            )
            self._command_enabled = True

        if not self._command_enabled:
            return

        if self._last_joy_msg.buttons[3] == 1:  # Press Y to open the gripper
            self.send_gripper_command(self.gripper_open_position)
        elif self._last_joy_msg.buttons[2] == 1:  # Press X to close the gripper
            self.send_gripper_command(0.0)

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "base_link"

        ##########################################################
        # Adjust the mapping of the joystick axes to your liking #
        ##########################################################
        s = self.input_to_velocity_linear_scaling

        twist.twist.linear.x = -s * self._last_joy_msg.axes[0]
        twist.twist.linear.y = s * self._last_joy_msg.axes[1]

        # multiply by 0.5 to map the range [-1, 1] to [0, +/-1]
        if self._last_joy_msg.axes[2] < 1:
            twist.twist.linear.z = 0.5 * s * (self._last_joy_msg.axes[2] - 1)
        elif self._last_joy_msg.axes[5] < 1:
            twist.twist.linear.z = -0.5 * s * (self._last_joy_msg.axes[5] - 1)

        s = self.input_to_velocity_angular_scaling
        twist.twist.angular.x = -s * self._last_joy_msg.axes[4]
        twist.twist.angular.y = -s * self._last_joy_msg.axes[3]
        if self._last_joy_msg.buttons[4] > 0:
            twist.twist.angular.z = s
        elif self._last_joy_msg.buttons[5] > 0:
            twist.twist.angular.z = -s

        self.servo_pub.publish(twist)

    def send_gripper_command(self, position):
        goal_msg = GripperCommand.Goal()

        goal_msg.command.position = position

        self.gripper_action_client.wait_for_server()

        future = self.gripper_action_client.send_goal_async(goal_msg)

        future.add_done_callback(self.gripper_response_callback)

    def gripper_response_callback(self, future: Future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Gripper command rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.gripper_result_callback)

    def gripper_result_callback(self, future: Future):
        result: GripperCommand.Result = future.result().result

        if result.stalled:
            self.get_logger().error("Gripper command stalled")

    def service_callback(self, future, service_name: str = ""):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"{service_name} service call succeeded")
            else:
                self.get_logger().error(f"{service_name} service call failed")
        except Exception as e:
            self.get_logger().error(f"{service_name} service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = JoystickServo()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
