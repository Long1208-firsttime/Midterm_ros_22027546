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
    """Gửi lệnh đưa tay máy về home pose (0.13, 0.6, 0.0) và joint_rr, joint_rl về 0"""
    traj = JointTrajectory()
    traj.header = Header()
    traj.header.frame_id = "base_link"
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = ['joint_armone', 'joint_armtwo', 'joint_grip', 'joint_rr', 'joint_rl']
    
    point = JointTrajectoryPoint()
    point.positions = [0.13, 0.6, 0.0, 0.0, 0.0]  # Home pose + joint_rr, joint_rl = 0
    point.velocities = [0.1, 0.1, 0.1, 0.1, 0.1]
    point.time_from_start = rospy.Duration(2.0)
    traj.points.append(point)
    
    rospy.loginfo("Moving arm to home pose (0.13, 0.6, 0.0) and joint_rr, joint_rl to 0...")
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
    rr_pos = 0.0  # Vị trí của joint_rr
    rl_pos = 0.0  # Vị trí của joint_rl
    step = 0.1

    # Giới hạn góc
    arm_min_limit = -1.57
    arm_max_limit = 1.57
    grip_min_limit = -0.2
    grip_max_limit = 0.2
    steer_min_limit = -0.3  # Giới hạn của joint_rr và joint_rl từ URDF
    steer_max_limit = 0.3

    # Tốc độ di chuyển của robot (4 bánh)
    linear_speed = 1.0
    angular_speed = 1.0

    # Biến kiểm soát trạng thái trước đó
    last_key = None

    # Hướng dẫn
    print("Điều khiển robot:")
    print("  W: Tiến lên (joint_rr, joint_rl về 0 nếu lần đầu)")
    print("  S: Dừng robot")
    print("  A: Quay trái (điều chỉnh joint_rr, joint_rl)")
    print("  D: Quay phải (điều chỉnh joint_rr, joint_rl)")
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
    print(" ")
    print(" ")
    print(" ")
    print(" ")
    print(" ")
    print("Cach fix loi tay may: bam 7/9 den khi nao tay may dung yen thi dieu khien duoc")
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        key = get_key().lower()
        
        # Tạo message Twist cho 4 bánh
        twist = Twist()
        
        # Biến kiểm soát việc gửi lệnh tay máy
        arm_moved = False
        
        # Điều khiển 4 bánh và joint_rr, joint_rl
        if key == 'w':
            twist.linear.x = linear_speed
            twist.angular.z = 0.0
            if last_key != 'w':  # Chỉ đặt về 0 khi lần đầu nhấn 'w'
                rr_pos = 0.0
                rl_pos = 0.0
                arm_moved = True
                rospy.loginfo(f"Moving forward at {linear_speed/2:.2f} m/s, joint_rr and joint_rl set to 0")
            else:
                rospy.loginfo(f"Moving forward at {linear_speed/2:.2f} m/s")
        elif key == 's':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            rospy.loginfo("Robot stopped")
        elif key == 'a':
            twist.linear.x = 0.0
            twist.angular.z = angular_speed
            rr_pos = min(steer_max_limit, rr_pos + step)  # Quay phải joint_rr
            rl_pos = max(steer_min_limit, rl_pos - step)  # Quay trái joint_rl
            arm_moved = True
            rospy.loginfo(f"Turning left at {angular_speed/2:.2f} rad/s")
        elif key == 'd':
            twist.linear.x = 0.0
            twist.angular.z = -angular_speed
            rr_pos = max(steer_min_limit, rr_pos - step)  # Quay trái joint_rr
            rl_pos = min(steer_max_limit, rl_pos + step)  # Quay phải joint_rl
            arm_moved = True
            rospy.loginfo(f"Turning right at {angular_speed/2:.2f} rad/s")
        elif key == 'e':
            linear_speed += 1.0
            rospy.loginfo(f"Speed increased to {linear_speed/2:.2f} m/s")
        elif key == 'r':
            linear_speed = max(0.0, linear_speed - 1.0)
            rospy.loginfo(f"Speed decreased to {linear_speed/2:.2f} m/s")
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        # Điều khiển tay máy
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
        
        # Chỉ gửi lệnh cho tay máy và các joint nếu có thay đổi
        if arm_moved:
            traj = JointTrajectory()
            traj.header = Header()
            traj.header.frame_id = "base_link"
            traj.header.stamp = rospy.Time.now()
            traj.joint_names = ['joint_armone', 'joint_armtwo', 'joint_grip', 'joint_rr', 'joint_rl']
            
            point = JointTrajectoryPoint()
            point.positions = [armone_pos, armtwo_pos, grip_pos, rr_pos, rl_pos]
            point.velocities = [0.1, 0.1, 0.1, 0.1, 0.1]
            point.time_from_start = rospy.Duration(0.1)
            traj.points.append(point)
            
            rospy.loginfo(f"Sending command: joint_armone={armone_pos:.2f}, joint_armtwo={armtwo_pos:.2f}, joint_grip={grip_pos:.2f}, joint_rr={rr_pos:.2f}, joint_rl={rl_pos:.2f}")
            pub_arm.publish(traj)
        
        # Gửi lệnh cho 4 bánh
        pub_vel.publish(twist)
        
        # Cập nhật phím cuối cùng
        last_key = key
        
        rate.sleep()

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
