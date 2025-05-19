Aim:

- specialise in path planning section

- Application:
  -- go to the marker
  -- follow the marker
  -- replace marker with human

Tools:
-- ros2
-- gazebo
-- <b> nav2 </b>

- load a 2d robot with controller manager
- naviagte in a plane
- load a terrain with enven surface
- follow and plan a path to the marker

# Setup

- ROS2 humble
- install packages -
- `rosdep install -i --from-path src --rosdistro humble -y`
- `sudo apt install ros-humble-xacro ros-humble-teleop-twist-keyboard ros-humble-ros2-control ros-humble-controller-manager ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs ros-humble-twist-mux`

# Build

- colcon build

# Run

- ros2 launch golf_cart robot_state_publisher.launch.py
- test all joints with gazebo: ros2 run joint_state_publisher_gui joint_state_publisher_gui --ros-args -p robot_description:=/robot_description
- ros2 launch golf_cart robot_control.launch.py world:=/home/sanjana/Desktop/golf_cart_ws/src/golf_cart/worlds/golf_field.world
- To control the robot: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robotnik_base_control/cmd_vel_unstamped

---

# DETECT HUMAN

- pip3 install torch torchvision ultralytics opencv-python seaborn
- sudo apt install ros-humble-cv-bridge ros-humble-vision-msgs

- Run: ros2 launch golf_cart detect_human.launch.py
