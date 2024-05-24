#!/usr/bin/env python3

#    A01245418 Andres Sarellano
#   Localization node for Dead Reckoning IT NOW WORKS :D SHOULD CALCULATE KL AND KR


import rospy
import numpy as np
from std_msgs.msg import Float32, Time, Header, Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from geometry_msgs.msg import Twist, PoseWithCovariance, TwistWithCovariance, PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
import tf
import tf.transformations
import tf_conversions

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
        self.odomPose = PoseStamped()
        self.covariancePose = PoseWithCovariance()
        self.covarianceTwist = TwistWithCovariance()
        self.covariance64 = Float64MultiArray()
        #   Covariance constats
        #TODO   CALIBRAR LOS PARAMETROS CON MINIMOS CUADRADOS
        self.kr = 0.15
        self.kl = 0.30913602108470944 
        self.SigmakPast = 0.0

        self.prevTime = rospy.Time.now()

        #   Node Subscriptions
        rospy.Subscriber("/wl", Float32, self.wl_cb)
        rospy.Subscriber("/wr", Float32, self.wr_cb)
        #   Node Publishers
        self.odometry_pub = rospy.Publisher("odom", Odometry, queue_size=1)

        self.tf_broadcaster = tf.TransformBroadcaster()
        
        #print("Node Init successfull")
    
    def wl_cb(self, msg):
        self.wl = msg.data
    def wr_cb(self, msg):
        self.wr = msg.data

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
            The full covariance matrix goes as follows:
            [ sigma_xx        sigma_xy        sigma_xtheta      sigma_xtheta        sigma_xtheta        sigma_xtheta     ]
            [ sigma_yx        sigma_yy        sigma_ytheta      sigma_xtheta        sigma_xtheta        sigma_ytheta     ]
            [ sigma_theta_x   sigma_thetay    sigma_thetatheta  sigma_xtheta        sigma_xtheta        sigma_xtheta     ]
            [ sigma_xx        sigma_xy        sigma_xtheta      sigma_xtheta        sigma_xtheta        sigma_xtheta     ]
            [ sigma_yx        sigma_yy        sigma_ytheta      sigma_xtheta        sigma_xtheta        sigma_xtheta     ]
            [ sigma_theta_x   sigma_thetay    sigma_thetatheta  sigma_xtheta        sigma_xtheta        sigma_thetaheta  ]

            @param dt Time Differential current time - previous time

        
        """
 
        #   H Matrix with the puzzlebot's kinematic model 
        Hk = np.matrix([[1.0, 0.0, -dt*self.linearX*np.sin(self.thetaPast)],[0.0, 1.0, dt*self.linearX*np.cos(self.thetaPast)],[0.0, 0.0, 1.0]])
        
        #   Sigma calculation for the covariance matrix with covariance constants
        Sigmadeltak = np.matrix([[self.kr * abs(self.wr), 0.0], [0.0, self.kl * abs(self.wl)]])
        
        scalarComputing = (0.5) * self.wheel_radius * dt
        #   Gradient Velcity calculation with 3 x 3 covariance matrix
        GradientOmegak = np.multiply(scalarComputing, np.matrix([[np.cos(self.thetaPast), np.cos(self.thetaPast)],[np.sin(self.thetaPast), np.sin(self.thetaPast)], [2/self.l, -2/self.l]]))
        
        #   Qk Gaussian Matrix 
        Qk = GradientOmegak * Sigmadeltak * np.transpose(GradientOmegak)
        #   Covariance matrix
        Sigmak = np.dot(np.dot(Hk, self.SigmakPast), np.transpose(Hk)) + Qk
        #   Sigma result k - 1 update
        self.SigmakPast = Sigmak

        #   Assigns covariance values to covariance matrix of the Odometry message
        self.odometry.pose.covariance[0] = Sigmak[0, 0]
        self.odometry.pose.covariance[1] = Sigmak[0, 1]
        self.odometry.pose.covariance[5] = Sigmak[0, 2]
        self.odometry.pose.covariance[6] = Sigmak[1, 0]
        self.odometry.pose.covariance[7] = Sigmak[1, 1]
        self.odometry.pose.covariance[11] = Sigmak[1, 2]
        self.odometry.pose.covariance[30] = Sigmak[2, 0]
        self.odometry.pose.covariance[31] = Sigmak[2, 1]
        self.odometry.pose.covariance[35] = Sigmak[2, 2]


    def getOdometry(self):
        """
            @brief Assigns Odometry values to /odom message
        
        """

        #   Obtain current time for integration
        self.currentTime = rospy.Time.now()
        dt = float((self.currentTime - self.prevTime).to_sec())

        self.calculate_cmdVel()
        #   Obtain Theta from angular velocity
        self.theta_dot = self.angularZ
        self.theta += self.theta_dot * dt

        #   Obtain linear speeds using theta
        self.x_dot = self.linearX * np.cos(self.theta)
        self.y_dot = self.linearX * np.sin(self.theta)

        #   Obtain linear position from velocity
        self.x += self.x_dot * dt
        self.y += self.y_dot * dt

        #   Assign linear elements
        self.points.x = self.x
        self.points.y = self.y
        self.points.z = 0.0

        #   Initialize odometry message
        #self.odometry.twist.covariance = self.covariance.da
        self.odometry.header.stamp = rospy.Time.now()
        self.odometry.header.frame_id = 'odom'
        self.odometry.child_frame_id = 'base_link'
        self.odometry.pose.pose.position.x = self.x
        self.odometry.pose.pose.position.y = self.y
        self.odometry.pose.pose.position.z = 0.0

        #   Get Quaternion Transform for pose orientation 
        self.euler2quater = tf_conversions.transformations.quaternion_from_euler(0, 0, self.theta)
        self.odometry.pose.pose.orientation.x = self.euler2quater[0]
        self.odometry.pose.pose.orientation.y = self.euler2quater[1]
        self.odometry.pose.pose.orientation.z = self.euler2quater[2]
        self.odometry.pose.pose.orientation.w = self.euler2quater[3]

        #   Assigns cmd_vel parameters to Odometry message
        self.odometry.twist.twist.linear.x = self.linearX
        self.odometry.twist.twist.angular.z = self.angularZ
        
        #   Runs the calculations and updates covariance matrix
        self.covarianceCalculation(dt)




        #   Publish Pose
        self.odometry_pub.publish(self.odometry)  

        #   Update past time
        self.prevTime = self.currentTime
        #   Update previous position
        self.thetaPast = self.theta
        #print("passed odometry test")





if __name__=='__main__':
    try:
        #   Node initialization
        rospy.init_node("localisation_node")
        rate = rospy.Rate(10)
        puzz = localization()
        
        
        while not rospy.is_shutdown():
            puzz.getOdometry()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass#Initialise and setup node
