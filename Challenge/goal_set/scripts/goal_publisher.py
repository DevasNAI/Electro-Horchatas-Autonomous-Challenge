#!/usr/bin/env python3
#   Andres Sarellano
import rospy
import numpy as np

from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Twist, PoseStamped

class Goal:
    def __init__(self):
        #   Define local message objects
        self.goal = PoseStamped()
        self.goalstr = String()
        #   Goal state
        self.goal_changer = ["cube", "base_a", "base_b", "base_c"]

        #   Subscribes to Goal instruction from controller
        rospy.Subscriber("/goal_ack", String, self.ackstr_cb)
        self.goal_pub = rospy.Publisher("/goal", PoseStamped, queue_size=1)
        
    def ackstr_cb(self, msg):
        self.goalstr = msg.data
    
    def pub_goal(self):

        """! Changes the goal coordinates depending on the message"""
        #   Depending on the goal, change the position 
        #   Go to Cube
        if(self.goalstr == self.goal_changer[0]):
            self.goal.header.frame_id = "odom"
            self.goal.pose.position.x = rospy.get_param("/cube/x", 0.0)
            self.goal.pose.position.y = rospy.get_param("/cube/y", 0.0)
        #   Go to Base A
        elif self.goalstr == self.goal_changer[1]:
            self.goal.header.frame_id = "odom"
            self.goal.pose.position.x = rospy.get_param("/base_a/x", 0.0)
            self.goal.pose.position.y = rospy.get_param("/base_a/y", 0.0)
        #   Go to Base B
        elif self.goalstr == self.goal_changer[2]:
            self.goal.header.frame_id = "odom"
            self.goal.pose.position.x = rospy.get_param("/base_b/x", 0.0)
            self.goal.pose.position.y = rospy.get_param("/base_b/y", 0.0)
        #   Go to Base C
        elif self.goalstr == self.goal_changer[3]:
            self.goal.header.frame_id = "odom"
            self.goal.pose.position.x = rospy.get_param("/base_c/x", 0.0)
            self.goal.pose.position.y = rospy.get_param("/base_c/y", 0.0)
        else:
            #   Default go to Origin
            self.goal.header.frame_id = "odom"
            self.goal.pose.position.x = 0.0
            self.goal.pose.position.y = 0.0 

        #   Publishes goal
        self.goal_pub.publish(self.goal)  
        
if __name__=='__main__':
    try:
        #   Initialise and Setup node
        rospy.init_node("goal_publisher")
        rate = rospy.Rate(10)

        goaling = Goal()
        #print("The Controller is Running")

        while not rospy.is_shutdown():
            goaling.pub_goal()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass