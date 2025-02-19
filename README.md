# ar4_ros_driver_examples

This repo demonstrates various usages of the Annin Robotics AR4 robotic arm.

## Teleoperation using Xbox controller

The [annin_ar4_joystick_servo](./annin_ar4_joystick_servo) package highlights how to teleoperate the AR4.
It uses [moveit_servo](https://index.ros.org/p/moveit_servo/github-ros-planning-moveit2/) to send
continuous real-time velocity trajectories to the robot.

[![Xbox Controller Teleop Demo](https://img.youtube.com/vi/QZEn53Pvfms/0.jpg)](https://youtube.com/shorts/QZEn53Pvfms)

## Multi-Arm Control in Simulation

The [annin_ar4_multi_arm](./annin_ar4_multi_arm) package demonstrates how to
control a dual-arm setup in the Gazebo simulation environment. It achieves this by
running separate nodes for the left and right arms, each within its own namespace.

[![Multi-Arm Control Demo](https://img.youtube.com/vi/USs1s4_NjBI/hqdefault.jpg)](https://youtu.be/USs1s4_NjBI)

## Setup

Clone code, install dependencies, and build packages.

```bash
mkdir -p ~/ar4_ws/src
cd ~/ar4_ws/src
git clone https://github.com/ycheng517/ar4_ros_driver_examples.git
git clone https://github.com/ycheng517/ar4_ros_driver.git
rosdep install --from-paths . --ignore-src -r -y

colcon build
```

### Running the Teleoperation using Xbox controller Demo

```bash
# Launch the robot. Adjust the command based on your robot's version
ros2 launch annin_ar4_driver driver.launch.py ar_model:=mk1 calibrate:=True include_gripper:=True

# Launch Everything else.
ros2 launch annin_ar4_joystick_servo joystick_servo.launch.py ar_model:=mk1 include_gripper:=True
```

### Running the Multi-Arm Control in Simulation Demo

```bash
# Launch Gazebo
ros2 launch annin_ar4_multi_arm gazebo.launch.py

# Launch MoveIt and RViz
ros2 launch annin_ar4_multi_arm moveit.launch.py
```
