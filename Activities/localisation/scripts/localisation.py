#!/usr/bin/env python3

#    A01245418 Andres Sarellano
#   Localization node for Dead Reckoning

import rospy
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Time
from std_msgs.msg import Header
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler




class localization:
    def __init__(self):

        #   Constant
        self.wheel_radius = 0.05 #  meters
        self.l = 0.19            #  Wheelbase in meters
        self.x = 0.0
        self.y = 0.0
        
        self.cmdVel = Twist()
        self.wl = 0.0
        self.wr = 0.0
        self.linearX = 0.0
        self.angularZ = 0.0
        self.theta = 0.0
        self.thetaPast = 0.0
        self.odometry = Odometry()
        self.quaternions = Quaternion()
        self.points = Point()
        self.poses = Pose()
        self.headerPose = Header()
        self.arreglo64 = Float64()
        self.odomPose = PoseStamped()
        self.covariancePose = PoseWithCovariance()
        self.covarianceTwist = TwistWithCovariance()
        self.covariance64 = Float64MultiArray()
        #   Covariance constats
        self.kr = 1.0
        self.kl = 1.0
        self.SigmakPast = np.matrix([[0, 0, 0], [0,0,0], [0,0,0]])
        self.covarianceMatrix = np.matrix([[0, 0, 0, 0, 0, 0],
                                           [0, 0, 0, 0, 0, 0],
                                           [0, 0, 0, 0, 0, 0],
                                           [0, 0, 0, 0, 0, 0],
                                           [0, 0, 0, 0, 0, 0],
                                           [0, 0, 0, 0, 0, 0]])
        self.covLayout = MultiArrayLayout()
        self.matrixH = 6
        self.matrixW = 6
        self.prevTime = rospy.Time.now()

        self.covarianceInit()
        #   Node Subscriptions
        rospy.Subscriber("/wl", Float32, self.wl_cb)
        rospy.Subscriber("/wr", Float32, self.wr_cb)
        #   Node Publishers
        self.odometry_pub = rospy.Publisher("/odom", Float32, queue_size=1)
    
    def wl_cb(self, msg):
        self.wl = msg
    def wr_cb(self, msg):
        self.wr = msg

    def covarianceInit(self):
        self.covLayout.dim.append(MultiArrayDimension("height", self.matrixH, self.matrixH * self.matrixW))
        self.covLayout.dim.append(MultiArrayDimension("width", self.matrixW, self.matrixW))
        self.covLayout.data_offset = 0
        self.covariance64.layout = self.covLayout
        #self.covariance64.layout.dim[0].label = "height"
        #self.covariance64.layout.dim[1].label = "width"
        #self.covariance64.layout.dim[0].size = self.matrixH
        #self.covariance64.layout.dim[1].size = self.matrixW
        #self.covariance64.layout.dim[0].stride = self.matrixH * self.matrixW
        #self.covariance64.layout.dim[1].stride = self.matrixW
        #self.covariance64.layout.data_offset = 0
        self.arreglo64.data = [[0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0] ]
        self.covariance64.data = self.arreglo64#[[0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0] ]

    def calculate_cmdVel(self):
        #   Assigns linear and angular velocity 
        self.linearX = self.wheel_radius * ((self.wr+self.wl)/2)
        self.angularZ = self.wheel_radius * ((self.wr-self.wl)/self.l)

    def covarianceCalculation(self, dt):
        """
            @brief The Covariance matrix should be as follows:
            [ sigma_xx        sigma_xy        sigma_xtheta     ]
            [ sigma_yx        sigma_yy        sigma_ytheta     ]
            [ sigma_theta_x   sigma_thetay    sigma_thetatheta ]

            @dt Time Differential

        
        """
        Hk = np.matrix([[1, 0, -dt*self.linearX*np.sin(self.thetaPast)],
                       [0, 0, dt*self.linearX*np.cos(self.thetaPast)],
                       [0, 0, 1]])
        Sigmadeltak = np.matrix([[self.kr * abs(self.wr), 0], [0, self.kl * abs(self.wl)]])
        GradientOmegak = 1/2 * self.wheel_radius * dt *  np.matrix([[np.cos(self.thetaPast), np.cos(self.thetaPast)],
                                                                   [np.sin(self.thetaPast), np.sin(self.thetaPast)],
                                                                   [2/self.l, -2/self.l]])
        Qk = GradientOmegak * Sigmadeltak * np.transpose(GradientOmegak)

        Sigmak = Hk * self.SigmakPast * np.transpose(Hk) + Qk

        self.covarianceMatrix[0, 0] = Sigmak[0,0]   # xx
        self.covarianceMatrix[1, 0] = Sigmak[1,0]   # yx
        self.covarianceMatrix[5, 0] = Sigmak[2,0]   # thetax
        self.covarianceMatrix[0, 1] = Sigmak[0,1]   # xy
        self.covarianceMatrix[1, 1] = Sigmak[1,1]   # yy
        self.covarianceMatrix[5, 1] = Sigmak[2,1]   # thetay
        self.covarianceMatrix[0, 5] = Sigmak[0,2]   # xtheta
        self.covarianceMatrix[1, 5] = Sigmak[1,2]   # ytheta
        self.covarianceMatrix[5, 5] = Sigmak[2,2]   # thetatheta

        #   Inits Covariance Matrix FLoat64MultiArray message
        #   Maybe I should make it a return function so its easier to access and
        #   less variables are here.
        self.covariance64.data = self.covarianceMatrix.tolist()
        print(type(self.covariance64.data))

        

    def getOdometry(self):

        #   Obtain current time for integration
        self.currentTime = rospy.Time.now()
        dt = (self.currentTime - self.prevTime).to_sec()

        self.calculate_cmdVel()
        #   Obtain Theta from angular velocity
        self.theta_dot = self.angularZ
        self.theta += self.theta_dot * dt
        #   Bound to pi
        self.theta = self.theta % (2*np.pi) 

        #   Obtain linear speeds using theta
        self.x_dot = self.linearX * np.cos(self.theta)
        self.y_dot = self.linearX * np.sin(self.theta)

        #   Obtain linear position from velocity
        self.x += self.x_dot * dt
        self.y += self.y_dot * dt

        #   Assign linear elements
        self.points.x = self.x
        self.points.y = self.y
        self.points.z = self.theta

        #   Revisar lo del twist para asignar a cmd_vel
        self.cmdVel.angular = self.angularZ
        self.cmdVel.linear = self.linearX

        #   Get Quaternion message
        self.euler2quater = quaternion_from_euler(self.points.x, self.points.y, self.points.z)
        self.quaternions.x = self.euler2quater[0]
        self.quaternions.y = self.euler2quater[1]
        self.quaternions.z = self.euler2quater[2]
        self.quaternions.w = self.euler2quater[3]

        #   Assign elements to message Point 
        self.poses.position = self.points
        self.poses.orientation = self.quaternions
        
        #   Covariance Pose assignment
        self.covariancePose.pose = self.poses
        self.covarianceTwist.twist = self.cmdVel

        #   Runs the calculations and updates covariance matrix
        self.covarianceCalculation(dt)

        #   Assigns nav_msgs/Odometry message elements
        self.odometry.pose = self.covariancePose
        self.odometry.twist = self.covarianceTwist
        self.odometry.pose.covariance = self.covariance64
        self.odometry.header.stamp = rospy.Time.now()
        self.odometry.header.frame_id = "base_link"
        self.odometry.child_frame_id = "odom"
        
        

        #   Publish Pose
        self.odometry_pub.publish(self.odometry)
        

        #   Update past time
        self.prevTime = self.currentTime
        #   Update previous position
        self.thetaPast = self.theta





if __name__=='__main__':
    try:
        #   Node initialization
        rospy.init_node("localisation")
        rate = rospy.Rate(10)
        puzz = localization()
        
        while not rospy.is_shutdown():
            puzz.getOdometry()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass#Initialise and setup node
