#!/usr/bin/env python2

import rospy
import numpy as np
import math
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
        
        self.wl = 0.0
        self.wr = 0.0
        self.linearX = 0.0
        self.angularZ = 0.0
        
        self.theta = 0.0
        self.thetaPast = 0.0
        
        self.cmdVel = Twist()
        self.odometry = Odometry()
        self.quaternions = Quaternion()
        self.points = Point()
        self.poses = Pose()
        self.headerPose = Header()
        self.odomPose = PoseStamped()
        self.goal = PoseStamped()
        self.covariancePose = PoseWithCovariance()
        self.covarianceTwist = TwistWithCovariance()
        self.covariance64 = Float64MultiArray()
        #   Covariance constats
        #TODO   CALIBRAR LOS PARAMETROS CON MINIMOS CUADRADOS
        self.kr = 0.15
        self.kl = 0.30913602108470944 
        self.SigmakPast = 0.0

        self.prevTime = rospy.Time.now()
        self.new_value = 2
        
        self.markers = {"aruco_0" : [2.5, -0.5],
                        "aruco_1" : [-0.5, -0.5],
                        "aruco_2" : [-0.5, 2.5],
                        "aruco_3" : [2.5, 2.5]}

        #   Node Subscriptions
        rospy.Subscriber("/puzzlebot_1/w1", Float32, self.wl_cb)
        rospy.Subscriber("/puzzlebot_1/wr", Float32, self.wr_cb)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.cbGoal)
        self.goalPub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        
        #   Node Publishers
        self.odometry_pub = rospy.Publisher("odom", Odometry, queue_size=1)

        self.tf_broadcaster = tf.TransformBroadcaster()
        
    def wl_cb(self, msg):
        self.wl = msg.data
    def wr_cb(self, msg):
        self.wr = msg.data
    
    def cbGoal(self, goal_msg):
        self.goal = goal_msg
        
    def wrapToPi(self, theta):
        if theta > np.pi:
                theta = theta - 2*np.pi
        elif theta < -np.pi:
            theta = theta + 2*np.pi
        return theta

    def calculate_cmdVel(self):
        #   Assigns linear and angular velocity 
        self.linearX = self.wheel_radius * ((self.wr+self.wl)/2)
        self.angularZ = self.wheel_radius * ((self.wr-self.wl)/self.l)
        
    def get_real_marker_position(self, marker_id : int):
        for arucos in self.markers:
            if arucos == marker_id:
                return np.array([[self.markers[arucos][0]], 
                                 [self.markers[arucos][1]]])

    def get_marker_position(self, marker_id): 

        return np.array([[0.9], [-1*(-0.9)]])


    def covarianceCalculation(self, dt):
        #   H Matrix with the puzzlebot's kinematic model 
        Hk = np.matrix([[1.0, 0.0, -dt*self.linearX*np.sin(self.thetaPast)],
                        [0.0, 1.0, dt*self.linearX*np.cos(self.thetaPast)],
                        [0.0, 0.0, 1.0]])
        
        #   Sigma calculation for the covariance matrix with covariance constants and noise 
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
        
        # Save state vector 𝑠𝑥,𝑘 
        self.state = np.array([ [self.linearX*np.cos(self.theta) + Sigmadeltak[0, 0]],
                                [self.linearX*np.sin(self.theta) + Sigmadeltak[1, 0]],
                                [self.angularZ + self.theta]])
                
        return Sigmak
    

    def post_covarianceCalculation(self, sigma, dt):
        # Change to the ones that are detected by the camera
        marker_position = [self.goal.pose.position.x, self.goal.pose.position.y]          # Estimated
        real_marker_position = [self.goal.pose.position.x, self.goal.pose.position.y]     # Real
        
        
        s = self.state # get the state vector
        
        delta_x = marker_position[0] - self.x # calculate the change in x
        delta_y = marker_position[1] - self.y # calculate the change in y
        p = delta_x**2 + delta_y**2 # calculate the change in distance
        
        # calculate the observation matrix:
        z_obsvr = np.array([[np.sqrt(p)],
                            [self.wrapToPi(np.arctan2(delta_y, delta_x)-self.theta)]])
        
       # calculate the linearised observation matrix:
        G = np.array([  [-delta_x/np.sqrt(p), -delta_y/np.sqrt(p), 0],
                        [delta_y/p, -delta_x/p, -1]])
        
        # calculate the Z matrix:                           Plus noise
        Z = np.dot(np.dot(G, sigma), G.T) + np.array([[0.1, 0],[0, 0.02]])
        
        # Kalman Gain matrix 
        K = np.dot(np.dot(sigma, G.T), np.linalg.inv(Z)) # calculate the K matrix o la ganancia de Kalman
        
        # Distance among coordinates from the real position
        delta_real = np.array([ [real_marker_position[0]] - s[0],
                                [real_marker_position[1]] - s[1]])
        
        # the p of the Jacobian Matrix
        p2 = delta_real[0][0]**2 + delta_real[1][0]**2

        # error angle
        error_angle = self.wrapToPi(np.arctan2(delta_real[0][0], delta_real[1][0]) - self.theta)

        # Real observer position of the marker
        z_real = np.array(  [[np.sqrt(p2)],
                            [error_angle]])
        rospy.loginfo(f"\nreal observer : {z_real.T},\n estimate observer : {z_obsvr.T}")

        s = s + np.dot(K, (z_obsvr-z_real)) # calculate the state vector
        
        sigma = np.dot((np.identity(3) - np.dot(K, G)), sigma) # calculate the covariance matrix
        
        self.state = s # save the state vector
        
        return sigma

        
    def getOdometry(self):
  
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
        
        goal = PoseStamped()
#
        #if self.linearX  < 0.01 and self.angularZ < 0.01:
        #    self.new_value = 2
        #
#
        goal.header.frame_id = "world"
        goal.pose.position.x = 2
        goal.pose.position.y = 2
        self.goalPub.publish(goal)
        
        #   Runs the calculations and updates covariance matrix
        sigma_past = self.covarianceCalculation(dt)
        sigma = self.post_covarianceCalculation(sigma_past, dt)
        
        self.odometry.pose.covariance[0] = sigma[0, 0]
        self.odometry.pose.covariance[1] = sigma[0, 1]
        self.odometry.pose.covariance[5] = sigma[0, 2]
        self.odometry.pose.covariance[6] = sigma[1, 0]
        self.odometry.pose.covariance[7] = sigma[1, 1]
        self.odometry.pose.covariance[11] = sigma[1, 2]
        self.odometry.pose.covariance[30] = sigma[2, 0]
        self.odometry.pose.covariance[31] = sigma[2, 1]
        self.odometry.pose.covariance[35] = sigma[2, 2]
 
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
        pass
