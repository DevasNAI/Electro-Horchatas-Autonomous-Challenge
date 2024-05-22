#! /usr/bin/env python2

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
        self.state = []

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
    def wrapToPi(self, theta):
        return (theta + np.pi) % (2 * np.pi) - np.pi

    def calculate_cmdVel(self):
        #   Assigns linear and angular velocity 
        self.linearX = self.wheel_radius * ((self.wr+self.wl)/2)
        self.angularZ = self.wheel_radius * ((self.wr-self.wl)/self.l)


    def get_marker_position(self, marker_id): 
        ####################### Posibles rotationes por la camara

        #marker : Marker
        #for marker in self.markers.markers:
        #if marker.id == marker_id:
        # x = marker.pose.pose.position.z +.1
        # y = -marker.pose.pose.position.x
        # pose : PoseStamped
        # pose = self.transform_pose(marker.pose.pose, "rviz_puzzlebot/chassis","rviz_puzzlebot/base_link")

        return np.array([[0.9], [-1*(-0.9)]])

   

    def update_s_vector(self, delta_u, s):

        theta = s[2][0] # If you don't add the [0] it will be a 2d array and you will
        # get an error. s[2].item() also works.
        delta_d = delta_u[0]
        delta_theta = delta_u[1]
        s = np.array([[delta_d*np.cos(theta) + s[0][0]],
        [delta_d*np.sin(theta) + s[1][0]],
        [delta_theta + theta]])


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
        self.state = np.array([[self.linearX*np.cos(self.theta) + noise_x],
                                [self.linearX*np.sin(self.theta) + noise_y],
                                [self.angularZ + self.theta]])
        #   H Matrix with the puzzlebot's kinematic model 
        Hk = np.matrix([[1.0, 0.0, -dt*self.linearX*np.sin(self.thetaPast)],
                        [0.0, 1.0, dt*self.linearX*np.cos(self.thetaPast)],
                        [0.0, 0.0, 1.0]])
        
        #   Sigma calculation for the covariance matrix with covariance constants
        # noise 
        Sigmadeltak = np.matrix([[self.kr * abs(self.wr), 0.0], 
                                 [0.0, self.kl * abs(self.wl)]])
        
        scalarComputing = (0.5) * self.wheel_radius * dt
        #   Gradient Velcity calculation with 3 x 3 covariance matrix
        GradientOmegak = np.multiply(scalarComputing, np.matrix([[np.cos(self.thetaPast), np.cos(self.thetaPast)],[np.sin(self.thetaPast), np.sin(self.thetaPast)], [2/self.l, -2/self.l]]))
        
        #   Qk Gaussian Matrix 
        Qk = GradientOmegak * Sigmadeltak * np.transpose(GradientOmegak)
        #   Covariance matrix
        Sigmak = np.dot(np.dot(Hk, self.SigmakPast), np.transpose(Hk)) + Qk
        #   Sigma result k - 1 update
        self.SigmakPast = Sigmak
        
        # Save state vector 
        #print(Sigmadeltak)
       
        noise_x = Sigmadeltak[0, 0]
        noise_y = Sigmadeltak[0, 1]
       
        

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
    

    def post_covarianceCalculation(self, dt):
 
        marker_position = [0.9, 0.9] # Estimated
        real_marker_position = [1, 1] # Real
        s = self.state # get the state vector
        
        delta_x = self.x - marker_position[0] # calculate the change in x
        delta_y = self.y - marker_position[1] # calculate the change in y
        p = delta_x**2 + delta_y**2 # calculate the change in distance
        
        Sigmadeltak = np.matrix([[self.kr * abs(self.wr), 0.0], 
                                 [0.0, self.kl * abs(self.wl)]])
        
        Hk = np.matrix([[1.0, 0.0, -dt*self.linearX*np.sin(self.thetaPast)],
                        [0.0, 1.0, dt*self.linearX*np.cos(self.thetaPast)],
                        [0.0, 0.0, 1.0]])
        
        scalarComputing = (0.5) * self.wheel_radius * dt
        #   Gradient Velcity calculation with 3 x 3 covariance matrix
        GradientOmegak = np.multiply(scalarComputing, np.matrix([[np.cos(self.thetaPast), np.cos(self.thetaPast)],
                                                                 [np.sin(self.thetaPast), np.sin(self.thetaPast)], 
                                                                 [2/self.l, -2/self.l]]))
        
        #   Qk Gaussian Matrix 
        Qk = GradientOmegak * Sigmadeltak * np.transpose(GradientOmegak)
        
        Sigmak = np.dot(np.dot(Hk, self.SigmakPast), np.transpose(Hk)) + Qk
        
        # calculate the observation matrix:
        z_obsvr = np.array([[np.sqrt(p)],
                            [np.arctan(delta_y, delta_x)]])
        
        # calculate the linearised observation matrix:
        G = np.array([  [-delta_x/np.sqrt(p), -delta_y/np.sqrt(p), 0],
                        [delta_y/p, -delta_x/p, -1]])
        
        # calculate the Z matrix:                 # Ruidito indefinido
        Z = np.dot(np.dot(G, Sigmak), G.T )+ np.array([[0.1, 0],[0, 0.02]])
        
        # Kalman Gain matrix 
        K = np.dot(np.dot(Sigmak, G.T), np.linalg.inv(Z)) # calculate the K matrix
        
        delta_real = np.array([ [real_marker_position[0][0]] - s[0],
                                [real_marker_position[1][0]] - s[1]])
        
        p2 = delta_real[0][0]**2 + delta_real[1][0]**2

        error_angle = np.arctan2(self.wrapToPi(np.arctan2(delta_y, delta_x) - self.theta))

        z_real = np.array([[np.sqrt(p2)],
                        [error_angle]])
        
        #rospy.loginfo(f"\nreal marker : {z2.T},\ndetected marker : {z.T},
        # \nrobot_position : {s.T},\nmarker_position : {real_marker_position.T}")
        s = s + np.dot(K, (z_obsvr-z_real)) # calculate the state vector
        
        sigma = np.dot((np.identity(3) - np.dot(K, G)), sigma) # calculate the covariance matrix
        

        
        
        self.state = s # save the state vector

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
