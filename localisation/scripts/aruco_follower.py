#!/usr/bin/env python3
import cv2
import numpy as np
from time import sleep
import sys
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, TwistStamped, Point, Pose
from std_srvs.srv import Empty
from sensor_msgs.msg import Image
from localisation.msg import Num
from cv_bridge import CvBridge, CvBridgeError


class ArucoFollower:
  
    def __init__(self):
    
        self.curve = 0
        self.fSpeed = .1
        self.maxSpeed=0.125
        self.minSpeed=0.05

        self.p=0.05
        self.d=0.005
        self.prev_error=0
        self.desired_pos=0
        self.prev_error=0
        self.last_time=0
        self.avgError = []

        self.publisher = rospy.Publisher("/puzzlebot_1/base_controller/cmd_vel", Twist, queue_size=1)
        self.aruco_sub = rospy.Subscriber("/aruco_pose",Num, self.aruco_cb)

        self.msg = Twist()

    def aruco_cb(self, aruco_msg):
        self.aruco_pose = aruco_msg
        self.sendCommands(self.aruco_pose)
        print("got", self.aruco_pose)


    def sendCommands(self,aruco_pose):
  
        error=self.desired_pos - aruco_pose.x
        print("x_error:", error)
        avg=0
        current_time=rospy.get_time()
        dt=current_time-self.last_time
        
        pd=self.p*error + self.d*(error-self.prev_error)/dt

        self.prev_error=error
        self.last_time=current_time

        speed=round(float(pd),2)        

        if len(self.avgError) >= 10:
            self.avgError.pop()
        self.avgError.append(error)
        
        for error in self.avgError:
            avg += error
        
        avg = avg / len(self.avgError)

        vel=abs(self.maxSpeed-(abs(float(avg))/1000))
        line=round(vel,3)

        if aruco_pose.dist < 3:
            line = 0
        
        self.move(line,0,0,0,0,speed)

      
    def move(self, x=0, y=0, z=0, wx=0, wy=0, wz=0):
        self.msg.linear.x = x
        self.msg.linear.y = y
        self.msg.linear.z = z

        self.msg.angular.x = wx
        self.msg.angular.y = wy
        self.msg.angular.z = wz

        self.publisher.publish(self.msg)


if __name__=='__main__':
    try:
        #   Node initialization
        rospy.init_node("aruco_follower")
        rate = rospy.Rate(10)
        puzz = ArucoFollower()
        
        while not rospy.is_shutdown():
            puzz.aruco_cb
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


