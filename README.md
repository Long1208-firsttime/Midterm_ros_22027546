# Robot Arm Controller

ROS-based controller for a robotic arm and mobile base in Gazebo.

## Structure
- `Zero_control`: Robot control.
- `Ten`: Robot URDF model (Ackerman-steering Robot with two revolution Joint).

## Prerequisites
- ROS Noetic, Gazebo, Python 3
- Install: `sudo apt install ros-noetic-gazebo-ros ros-noetic-rviz ros-noetic-robot-state-publisher`

## Features
- Base: W/S (forward/back), A/D (left/right), E/R (speed up/down).
- Arm: 4/6 (armone left/right), 8/2 (armtwo up/down), 7/9 (grip close/open).
- Arm holds position until new command.

## Setup
1. Clone: `git clone https://github.com/Long1208-firsttime/Midterm_ros_22027546.git`
2. Build: `cd ~/catkin_ws && catkin_make && source devel/setup.bash`
3. Run:
   - Gazebo: `roslaunch ten gazebo.launch`
   - RViz: `roslaunch ten display.launch`
   - Controller: `rosrun zero_control arm_control.py`

## Usage
- Control base (WASD), speed (ER), arm (4,6,8,2,7,9). Exit with Q.
- Arm starts at home pose (0.13, 0.6, 0.0).

## Notes
- Update joint names/limits in script if URDF differs.
