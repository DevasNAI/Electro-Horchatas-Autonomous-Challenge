#!/usr/bin/env python

# Imports
import numpy as np
import tf
import time 
import rospy 
import math
import tf
from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry

# Init class
class Controller:
    def __init__(self):
        # Initialize class variables
        # Control variables
        self.kp_lin = 0.5 # Izel values
        self.kp_ang = 7.5 # Izel values
        self.ki_lin = 0.0 # Not in use
        self.ki_ang = 0.0
        self.kd_lin = 0.0 # Not in use
        self.kd_ang = 0.0 # Not is use

        # Time variables
        self.lastTime = 0.0
        self.currentTime = 0.0
        self.dt = 0.0

        # Errors and other variables
        self.theta = 0.0
        self.theta_desired = 0.0
        self.theta_err = 0.0
        self.dist_err = 0.0
        self.x_err = 0.0
        self.y_err = 0.0
        self.int_err_lin = 0.0
        self.int_err_ang = 0.0
        self.pre_err_lin = 0.0
        self.pre_err_ang = 0.0
        self.der_ang = 0.0
        self.der_lin = 0.0

        # No se para q es esto -_-
        self.theta_set = False
        
        # Simple variables
        self.quat_update = 0.0
        self.pos = 0.0
        self.goal_pos = 0.0
        self.maxSpeed = 10
        
        # Initialize the ROS node
        rospy.init_node('controller_node', anonymous=True)
        rospy.on_shutdown(self.stop)
        self.rate = rospy.Rate(50)

        # Create a publisher for the /cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.goal = rospy.Publisher('/goal', PoseStamped, queue_size=1)

        # Define and start ROS variables
        self.goal_msg = PoseStamped()
        self.goal_msg.header.frame_id = "odom"
        self.cmd_vel = Twist()
        self.target = PoseStamped()

        self.odom = Odometry()

    def wrapToPi(self, theta):
        return (theta + np.pi) % (2 * np.pi) - np.pi
    
    def odom_cb(self, odom_msg):
        self.odom = odom_msg

    def target_cb(self, target_msg):
        self.target = target_msg

    def stop(self):
        # print("Shutting down")
        empty = Twist()
        empty.linear.x = 0.0
        empty.angular.z = 0.0
        self.cmd_vel_pub.publish(empty)

    def main(self):
        # Hardcode the target
        self.target.pose.position.x = 1
        self.target.pose.position.y = 0
        self.target.pose.position.z = 0
        self.target.pose.orientation.x = 0
        self.target.pose.orientation.y = 0
        self.target.pose.orientation.z = 0
        self.target.pose.orientation.w = 1

        # Initialize subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        # rospy.Subscriber('/target', PoseStamped, self.target_cb)

        # Save last time
        self.lastTime = rospy.Time.now().to_sec()

        # Sigo sin saber q es esto -_-
        self.theta_set = False

        # Initialize loop
        while not rospy.is_shutdown():
            # Calculate delta time
            self.currentTime = rospy.Time.now().to_sec()
            self.dt = self.currentTime - self.lastTime
            
            # Movement warning
            # print("Moving to: ", self.target.pose.position.x, self.target.pose.position.y)
            self.goal_msg.pose.position = self.target.pose.position

            # Calculate and print important information
            # print("Current position: ", self.odom.pose.pose.position.x, self.odom.pose.pose.position.y)
           
            self.theta = self.wrapToPi(tf.transformations.euler_from_quaternion([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])[2] )
            # print("Current theta: ", self.theta)

            self.theta_desired = math.atan2(self.target.pose.position.y, self.target.pose.position.x) - self.theta
            # print("Desired theta: ", self.theta_desired)

            self.theta_err = self.wrapToPi(math.atan2(self.target.pose.position.y - self.odom.pose.pose.position.y, self.target.pose.position.x - self.odom.pose.pose.position.x))
            # print("Error theta: ", self.theta_err)

            # euler(x=0, y=0, z=atan2(y wanted - y now, x wanted - x now)) --> quaternion
            self.quat_update = tf.transformations.quaternion_from_euler(0, 0, math.atan2(self.target.pose.orientation.y - self.odom.pose.pose.orientation.y, self.target.pose.orientation.x - self.odom.pose.pose.orientation.x))

            while(True):
                # Simplify odom position and quaternion
                self.pos = self.odom.pose.pose.position
                self.goal_pos = self.target.pose.position
                self.theta = self.wrapToPi(tf.transformations.euler_from_quaternion([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])[2] )

                # Calculating the hypotenuse
                if (self.pos.x - self.goal_pos.x)**2 + (self.pos.y - self.goal_pos.y)**2 < 0.001:
                    break

                # Calculate error x & y
                self.x_err = self.goal_pos.x - self.pos.x
                self.y_err = self.goal_pos.y - self.pos.y

                # If need to turn start with the angular controller
                if self.theta_set == False:
                    # Angular controller
                    self.theta_err = abs(math.atan2(self.y_err, self.x_err) - self.theta)
                    if self.theta_err < 0.1:
                        self.theta_set = True
                    if (self.kp_ang * self.theta_err) > self.maxSpeed:
                        self.theta_err = self.maxSpeed
                    self.int_err_ang += self.theta_err * self.dt
                    self.der_ang = (self.theta_err - self.pre_err_ang) / self.dt
                    self.cmd_vel.angular.z =  self.kp_ang * self.theta_err + self.ki_ang * self.int_err_ang + self.kd_ang * self.der_ang
                
                else:
                    # Lineal controller
                    self.dist_err = math.sqrt(self.x_err**2 + self.y_err**2)
                    if (self.kp_lin * self.dist_err) > self.maxSpeed:
                        self.dist_err = self.maxSpeed
                    self.int_err_lin += self.dist_err * self.dt
                    self.der_lin = (self.dist_err - self.pre_err_lin) / self.dt
                    self.cmd_vel.linear.x =  self.kp_lin * self.dist_err + self.ki_lin * self.int_err_lin + self.kd_lin * self.der_lin

                self.cmd_vel_pub.publish(self.cmd_vel)
                self.goal.publish(self.goal_msg)
                self.pre_err_lin = self.dist_err
                self.pre_err_ang = self.theta_err
                self.lastTime = self.currentTime
                self.rate.sleep()
            
            self.theta_set = False
            self.rate.sleep()



# Main Code
if __name__ == "__main__":
    try:
        # print("Control Node Running")
        estim = Controller()
        estim.main()
    except rospy.ROSInterruptException:
        pass