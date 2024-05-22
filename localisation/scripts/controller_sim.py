#!/usr/bin/env python3
import rospy
import math
import tf
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class Controller():
    def init(self):
        self.goal = PoseStamped()
        self.goalSub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.cbGoal)
        
        self.odom = Odometry()
        self.odomSub = rospy.Subscriber('/puzzlebot_1/base_pose_ground_truth', Odometry, self.cbOdom)

        self.kp_linear = 3.5
        self.ki_linear = 2.0

        self.kp_angular = 1.2
        self.ki_angular = 0.7

        self.goalPub = rospy.Publisher("/goal", PoseStamped, queue_size=1)
        self.goal.header.frame_id = "world"
        self.goal.pose.position.x = 2.0
        self.goal.pose.position.y = 2.0

    def cbGoal(self,goal_msg):
        self.goal = goal_msg
    
    def cbOdom(self,odom_msg):
        self.odom = odom_msg
    
    def wrapToPi(self,theta):
        return (theta + np.pi) % (2 * np.pi) - np.pi
    
    def stop(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel_msg)
    
    def run(self):
        rate = rospy.Rate(50)

        # Create a publisher for the /cmd_vel topic
        # self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/puzzlebot_1/base_controller/cmd_vel", Twist, queue_size=1)
        
        while not rospy.is_shutdown():
            print("rtunning")

            self.goalPub.publish(self.goal)
            theta = tf.transformations.euler_from_quaternion([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])[2]

            # Calculate the error
            error_x = self.goal.pose.position.x - self.odom.pose.pose.position.x
            error_y = self.goal.pose.position.y - self.odom.pose.pose.position.y
            error_linear = math.sqrt(error_x**2 + error_y**2)   

            if (error_linear < 0.1):
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd_vel_msg)
                continue

            error_Theta = math.atan2(error_y,error_x) - theta
            if error_Theta > np.pi:
                error_Theta = error_Theta - 2*np.pi
            elif error_Theta < -np.pi:
                error_Theta = error_Theta + 2*np.pi

            # Calculate the control input
            linear = self.kp_linear * error_linear + self.ki_linear * error_linear
            angular = self.kp_angular * error_Theta + self.ki_angular * error_Theta            

            # Publish the control input
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = linear
            cmd_vel_msg.angular.z = angular

            if cmd_vel_msg.linear.x < 0.01 and cmd_vel_msg.angular.z < 0.01:
                continue

            self.cmd_vel_pub.publish(cmd_vel_msg)
            # self.puzzle_cmd_vel_pub.publish(cmd_vel_msg)

            rate.sleep()

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('control', anonymous=True)

    controller = Controller()

    rospy.on_shutdown(controller.stop)

    controller.init()
    controller.run()
