#!/usr/bin/env python3

#    A01245418 Andres Sarellano
#   Puzzlebot sym

import rospy
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Time
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from tf.transformations import quaternion_from_euler




class puzzlebot_sim:
    def __init__(self):

        #   Constant
        self.wheel_radius = 0.05 #  meters
        self.l = 0.19            #  Wheelbase in meters
        self.x = 0
        self.cmdVel = Twist()
        self.poses = Pose()
        self.wl = 0.0
        self.wr = 0.0
        self.linearX = 0.0
        self.angularZ = 0.0
        self.prevTime = rospy.Time()
        self.currentTime = 0.0
        self.quaternions = Quaternion()
        self.points = Point()
        self.headerPose = Header()
        self.odomPose = PoseStamped()
        self.phi = 0.0
        self.inertial_velocities = {"x":0.0, "y":0.0, "phi":0.0}

        #   Node Subscriptions
        rospy.Subscriber("/cmd_vel", Twist, self.cmdvel_cb)
        #   Node Publishers
        self.wl_pub = rospy.Publisher("/wl", Float32, queue_size=1)
        self.wr_pub = rospy.Publisher("/wr", Float32, queue_size=1)
        self.pose_pub = rospy.Publisher("/pose", PoseStamped, queue_size=1)
    
    def cmdvel_cb(self, msg):
        self.cmdVel = msg

    def diffferentialDriveModel(self):
        
        #   Assigns linear and angular velocity components of x and z respectively to local vars
        self.linearX = self.cmdVel.linear.x
        self.angularZ = self.cmdVel.angular.z

        #   Check equations on notebook
        self.wr = (self.linearX / self.wheel_radius) + ((self.l*self.angularZ) / (2 * self.wheel_radius) )
        self.wl = (self.linearX / self.wheel_radius) - ((self.l*self.angularZ) / (2 * self.wheel_radius) )

        self.wr_pub.publish(self.wr)
        self.wl_pub.publish(self.wl)

    def diffferentialDriveModel2(self):
        #   Assigns linear and angular velocity components of x and z respectively to local vars
        self.linearX = self.cmdVel.linear.x
        self.angularZ = self.cmdVel.angular.z

        #   Check equations on notebook
        self.wr = (self.linearX + self.angularZ) / self.wheel_radius
        self.wl = (self.linearX - self.angularZ) / self.wheel_radius

        self.wr_pub.publish(self.wr)
        self.wl_pub.publish(self.wl)

    #   wrap to pi function (COnverts position into angular position)
    def wrap_to_Pi(self, theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if(result < 0):
            result += 2 * np.pi
        return result - np.pi


    def equationPose(self):
        #   Obtain current time for integration
        self.currentTime = rospy.Time.now()
        dt = (self.currentTime - self.prevTime).to_sec()

        
        self.phi = self.angularZ
        #   Obtain Jacobian
        jacobian = [((self.wr*self.wheel_radius*np.cos(self.phi))/2) + ((self.wl*self.wheel_radius*np.cos(self.phi))/2),
                    ((self.wr*self.wheel_radius*np.sin(self.phi))/2) + ((self.wl*self.wheel_radius*np.sin(self.phi))/2),
                    ((self.wr*self.wheel_radius)/self.l) + ((self.wl*self.wheel_radius)/self.l)]

        #   Calculate inertial frame velocities
        self.inertial_velocities = {"x":jacobian[0], "y":jacobian[1], "phi":jacobian[2]}
        print(jacobian[0])
        #   Integrate inertial velocities
        self.points.x += self.inertial_velocities["x"] * dt
        self.points.y += self.inertial_velocities["y"] * dt
        self.points.z += self.inertial_velocities["phi"]* dt
        

        #   Get Quaternion message
        self.euler2quater = quaternion_from_euler(self.points.x, self.points.y, self.points.z)
        self.quaternions.x = self.euler2quater[0]
        self.quaternions.y = self.euler2quater[1]
        self.quaternions.z = self.euler2quater[2]
        self.quaternions.w = self.euler2quater[3]

        #   Assign elements to message Point 
        self.poses.position = self.points
        self.poses.orientation = self.quaternions
        #   PoseStamp Odometry message
        self.odomPose.pose = self.poses

#        self.odomPose.header.frame_id = "base_link"
        self.odomPose.header = rospy.Time.now()

        #   Publish Pose
        self.pose_pub.publish(self.odomPose)
        

        #   Update past time
        self.prevTime = self.currentTime




        
    def equationSolver(self):
        self.x = 1






if __name__=='__main__':
    try:
        #   Node initialization
        rospy.init_node("puzzebot_kinematic_model")
        rate = rospy.Rate(10)
        puzz = puzzlebot_sim()
        print("Node initialized")
        
        while not rospy.is_shutdown():
            puzz.diffferentialDriveModel()
            puzz.equationPose()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass#Initialise and setup node
