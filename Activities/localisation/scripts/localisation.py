#!/usr/bin/env python3

#    A01245418 Andres Sarellano
#   Puzzlebot sym

import rospy
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState




class localization:
    def __init__(self):

        #   Constant
        self.wheel_radius = 0.05 #  meters
        self.l = 0.19            #  Wheelbase in meters
        self.x = 0
        self.cmdVel = Twist()
        self.wl = 0.0
        self.wr = 0.0
        self.linearX = 0.0
        self.angularZ = 0.0

        #   Node Subscriptions
        rospy.Subscriber("/wl", Float32, self.wl_cb)
        rospy.Subscriber("/wr", Float32, self.wr_cb)
        #   Node Publishers
        self.odometry_pub = rospy.Publisher("/odom", Float32, queue_size=1)
    
    def wl_cb(self, msg):
        self.wl = msg
    def wr_cb(self, msg):
        self.wr = msg

    def diffferentialDriveModel(self):
        
        #   Assigns linear and angular velocity components of x and z respectively to local vars
        self.linearX = self.cmdVel.linear.x
        self.angularZ = self.cmdVel.angular.z

        #   Check equations on notebook
        self.wr = (self.linearX / self.wheel_radius) + ((self.l*self.angularZ) / (2 * self.wheel_radius) )
        self.wl = (self.linearX / self.wheel_radius) - ((self.l*self.angularZ) / (2 * self.wheel_radius) )

        self.wr_pub.publish(self.wr)
        self.wl_pub.publish(self.wl)

    def differentialDriveModel2(self):
        #   Assigns linear and angular velocity components of x and z respectively to local vars
        self.linearX = self.cmdVel.linear.x
        self.angularZ = self.cmdVel.angular.z

        #   Check equations on notebook
        self.wr = (self.linearX + self.angularZ) / self.wheel_radius
        self.wl = (self.linearX - self.angularZ) / self.wheel_radius

        self.wr_pub.publish(self.wr)
        self.wl_pub.publish(self.wl)



    def equationTransform(self):
        self.x = 1

    def equationSolver(self):
        self.x = 1






if __name__=='__main__':
    try:
        #   Node initialization
        rospy.init_node("puzzebot_kinematic_model")
        rate = rospy.Rate(10)
        puzz = puzzlebot_sim()
        
        while not rospy.is_shutdown():
            puzz.diffferentialDriveModel()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass#Initialise and setup node
