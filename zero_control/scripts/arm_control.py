#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import sys
import tty
import termios

def get_key():
    """Hàm đọc phím nhấn từ terminal mà không cần Enter"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def send_home_pose(pub_arm):
    """Gửi lệnh đưa tay máy về home pose (0.13, 0.6, 0.0)"""
    traj = JointTrajectory()
    traj.header = Header()
    traj.header.frame_id = "base_link"
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = ['joint_armone', 'joint_armtwo', 'joint_grip']
    
    point = JointTrajectoryPoint()
    point.positions = [0.13, 0.6, 0.0]  # Home pose
    point.velocities = [0.1, 0.1, 0.1]
    point.time_from_start = rospy.Duration(2.0)
    traj.points.append(point)
    
    rospy.loginfo("Moving arm to home pose (0.13, 0.6, 0.0)...")
    pub_arm.publish(traj)
    rospy.sleep(2)

def move_robot():
    # Khởi tạo node ROS
    rospy.init_node('robot_controller_node', anonymous=True)
    
    # Publisher
    pub_arm = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Đợi Gazebo sẵn sàng
    rospy.loginfo("Waiting for Gazebo to be ready...")
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    rospy.sleep(2)
    
    # Gửi tay máy về home pose khi khởi động
    send_home_pose(pub_arm)
    
    # Giá trị góc hiện tại
    armone_pos = 0.13
    armtwo_pos = 0.6
    grip_pos = 0.0
    step = 0.1

    # Giới hạn góc
    arm_min_limit = -1.57
    arm_max_limit = 1.57
    grip_min_limit = -0.2
    grip_max_limit = 0.2

    # Tốc độ di chuyển của robot (4 bánh)
    linear_speed = 1.0
    angular_speed = 1.0

    # Hướng dẫn
    print("Điều khiển robot:")
    print("  W: Tiến lên")
    print("  S: Lùi lại")
    print("  A: Quay trái")
    print("  D: Quay phải")
    print("  E: Tăng tốc độ (+1 m/s)")
    print("  R: Giảm tốc độ (-1 m/s)")
    print("Điều khiển tay máy:")
    print("  4: Quay joint_armone sang trái")
    print("  6: Quay joint_armone sang phải")
    print("  8: Quay joint_armtwo lên")
    print("  2: Quay joint_armtwo xuống")
    print("  7: Đóng joint_grip")
    print("  9: Mở joint_grip")
    print("  Q: Thoát")

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        key = get_key().lower()
        
        # Tạo message Twist cho 4 bánh
        twist = Twist()
        
        # Điều khiển 4 bánh
        if key == 'w':
            twist.linear.x = linear_speed
            twist.angular.z = 0.0
            rospy.loginfo(f"Moving forward at {linear_speed} m/s")
        elif key == 's':
            twist.linear.x = -linear_speed
            twist.angular.z = 0.0
            rospy.loginfo(f"Moving backward at {linear_speed} m/s")
        elif key == 'a':
            twist.linear.x = 0.0
            twist.angular.z = angular_speed
            rospy.loginfo("Turning left")
        elif key == 'd':
            twist.linear.x = 0.0
            twist.angular.z = -angular_speed
            rospy.loginfo("Turning right")
        elif key == 'e':
            linear_speed += 1.0
            rospy.loginfo(f"Speed increased to {linear_speed} m/s")
        elif key == 'r':
            linear_speed = max(0.0, linear_speed - 1.0)
            rospy.loginfo(f"Speed decreased to {linear_speed} m/s")
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        # Điều khiển tay máy
        arm_moved = False
        if key == '4':
            armone_pos = max(arm_min_limit, armone_pos - step)
            arm_moved = True
        elif key == '6':
            armone_pos = min(arm_max_limit, armone_pos + step)
            arm_moved = True
        elif key == '8':
            armtwo_pos = min(arm_max_limit, armtwo_pos + step)
            arm_moved = True
        elif key == '2':
            armtwo_pos = max(arm_min_limit, armtwo_pos - step)
            arm_moved = True
        elif key == '7':
            grip_pos = max(grip_min_limit, grip_pos - step)
            arm_moved = True
        elif key == '9':
            grip_pos = min(grip_max_limit, grip_pos + step)
            arm_moved = True
        elif key == 'q':
            rospy.loginfo("Exiting...")
            break
        
        # Chỉ gửi lệnh cho tay máy nếu có thay đổi vị trí
        if arm_moved:
            traj = JointTrajectory()
            traj.header = Header()
            traj.header.frame_id = "base_link"
            traj.header.stamp = rospy.Time.now()
            traj.joint_names = ['joint_armone', 'joint_armtwo', 'joint_grip']
            
            point = JointTrajectoryPoint()
            point.positions = [armone_pos, armtwo_pos, grip_pos]
            point.velocities = [0.1, 0.1, 0.1]
            point.time_from_start = rospy.Duration(0.1)
            traj.points.append(point)
            
            rospy.loginfo(f"Sending arm command: joint_armone={armone_pos:.2f}, joint_armtwo={armtwo_pos:.2f}, joint_grip={grip_pos:.2f}")
            pub_arm.publish(traj)
        
        # Gửi lệnh cho 4 bánh
        pub_vel.publish(twist)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
