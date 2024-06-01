#!/usr/bin/env python2

import rospy
import numpy as np
from std_msgs.msg import Float32, Header, Float64MultiArray
from geometry_msgs.msg import Twist, PoseWithCovariance, TwistWithCovariance, PoseStamped, Pose, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from ros_deep_learning.msg import ArucosDetected
import tf
import tf_conversions

class Localisation:
    def __init__(self):
        self.wheel_radius = 0.05
        self.l = 0.17
        self.wl = 0.0
        self.wr = 0.0
        self.linearX = 0.0
        self.angularZ = 0.0
        self.cmdVel = Twist()
        self.odometry = Odometry()
        self.quaternions = Quaternion()
        self.poses = Pose()
        self.headerPose = Header()
        self.odomPose = PoseStamped()
        self.goal = PoseStamped()
        self.covariancePose = PoseWithCovariance()
        self.covarianceTwist = TwistWithCovariance()
        self.covariance64 = Float64MultiArray()
        self.aruco_poses = []
        self.aruco = None
	#self.kr = 0.01505
	#self.kl = 0.0303
        self.kr = 0.0001505
        self.kl = 0.000303
        self.SigmakPast = np.zeros([3,3])
        self.state = np.array([0.0, 0.0, 0.0])
        self.prevTime = rospy.Time.now()
        self.markers = {
            1: [2, 3.03],
            2: [3.14, 2.4965],
            3: [3.14, 2.01],
            4: [3.14,0],
            5: [3,-0.364],
            7: [1.506,-0.359],
            8: [0,0.385],
            9: [-0.309,0],
            10: [-0.322,1.57],
            11: [-0.282,3.0215],
            12: [0,3.3225],
        }
        self.deadReck = [0,0,0]
        rospy.Subscriber("/wl", Float32, self.wl_cb)
        rospy.Subscriber("/wr", Float32, self.wr_cb)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.cbGoal)
        rospy.Subscriber("/aruco_detected", ArucosDetected, self.aruco_cb)
        self.goalPub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.odometry_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
        self.tf_broadcaster = tf.TransformBroadcaster()

    def aruco_cb(self, msg):
        self.aruco_poses = msg.arucos_detected
        
    def wl_cb(self, msg):
        self.wl = msg.data
    
    def wr_cb(self, msg):
        self.wr = msg.data
    
    def cbGoal(self, goal_msg):
        self.goal = goal_msg

    def calculate_cmdVel(self):
        self.linearX = self.wheel_radius * ((self.wr + self.wl) / 2)
        self.angularZ = self.wheel_radius * ((self.wr - self.wl) / self.l)

    def get_real_landmark_pos(self, marker_id):
        return self.markers[marker_id]

    def covarianceCalculation(self, dt):
        Hk = np.array([
            [1.0, 0.0, -dt * self.linearX * np.sin(self.state[2])],
            [0.0, 1.0, dt * self.linearX * np.cos(self.state[2])],
            [0.0, 0.0, 1.0]
        ])
        Sigmadeltak = np.diag([self.kr * abs(self.wr), self.kl * abs(self.wl)])
        scalarComputing = 0.5 * self.wheel_radius * dt
        GradientOmegak = scalarComputing * np.array([
            [np.cos(self.state[2]), np.cos(self.state[2])],
            [np.sin(self.state[2]), np.sin(self.state[2])],
            [2 / self.l, -2 / self.l]
        ])
        Qk = np.dot(GradientOmegak, np.dot(Sigmadeltak, GradientOmegak.T))
        Sigmak = np.dot(Hk, np.dot(self.SigmakPast, Hk.T)) + Qk
        self.state = self.state + np.array([
            dt * self.linearX * np.cos(self.state[2]),
            dt * self.linearX * np.sin(self.state[2]),
            dt * self.angularZ
        ])
        self.deadReck = self.deadReck + np.array([
            dt * self.linearX * np.cos(self.deadReck[2]),
            dt * self.linearX * np.sin(self.deadReck[2]),
            dt * self.angularZ
        ])
        self.SigmakPast = Sigmak
        return Sigmak

    def post_covarianceCalculation(self, sigma):
        if len(self.aruco_poses) == 0:
            return sigma
        aruco = self.aruco_poses[0]
        if aruco.id == 6:
            if len(self.aruco_poses) > 1:
                aruco = self.aruco_poses[1]
            else:
                return sigma
        marker_position = np.array([aruco.pose.position.x, aruco.pose.position.y])
        real_position = self.get_real_landmark_pos(aruco.id)
        deltax = real_position[0] - self.state[0]
        deltay = real_position[1] - self.state[1]
        distance = deltax ** 2 + deltay ** 2
        z_observed = np.array([
            [np.sqrt(distance)],
            [np.arctan2(deltay, deltax) - self.state[2]]
        ])
        G = np.array([
            [-deltax / np.sqrt(distance), -deltay / np.sqrt(distance), 0.0],
            [deltay / distance, -deltax / distance, -1.0]
        ])
	#.25 y 0.125
        Z = np.dot(G, np.dot(sigma, G.T)) + np.diag([0.25, 0.125])
        K = np.dot(sigma, np.dot(G.T, np.linalg.inv(Z)))
        delta_real = marker_position.copy()
        delta_real[0] = marker_position[0] + 0.085
        distance_real = delta_real[0] ** 2 + delta_real[1] ** 2
        error_angle = np.arctan2(delta_real[1], delta_real[0])
        z_real = np.array([
            [np.sqrt(distance_real)],
            [error_angle]
        ])
        rospy.loginfo("z real: %s", z_real)
        rospy.loginfo("z_obs: %s", z_observed)
        self.state = self.state + np.dot(K, (z_real - z_observed).ravel())
        sigma = np.dot((np.eye(3) - np.dot(K, G)), sigma)
        return sigma

    def getOdometry(self):
        self.currentTime = rospy.Time.now()
        dt = (self.currentTime - self.prevTime).to_sec()
        if dt <= 0:
            return
        self.calculate_cmdVel()
        self.odometry.header.stamp = rospy.Time.now()
        self.odometry.header.frame_id = 'odom'
        self.odometry.child_frame_id = 'base_link'
        self.odometry.pose.pose.position.x = self.state[0]
        self.odometry.pose.pose.position.y = self.state[1]
        self.odometry.pose.pose.position.z = 0.0
        quaternion = quaternion_from_euler(0, 0, self.state[2])
        self.odometry.pose.pose.orientation = Quaternion(*quaternion)
        self.odometry.twist.twist.linear.x = self.linearX
        self.odometry.twist.twist.angular.z = self.angularZ
        sigma_past = self.covarianceCalculation(dt)
        sigma = self.post_covarianceCalculation(sigma_past)
        self.odometry.pose.covariance[0] = sigma[0, 0]
        self.odometry.pose.covariance[1] = sigma[0, 1]
        self.odometry.pose.covariance[5] = sigma[0, 2]
        self.odometry.pose.covariance[6] = sigma[1, 0]
        self.odometry.pose.covariance[7] = sigma[1, 1]
        self.odometry.pose.covariance[11] = sigma[1, 2]
        self.odometry.pose.covariance[30] = sigma[2, 0]
        self.odometry.pose.covariance[31] = sigma[2, 1]
        self.odometry.pose.covariance[35] = sigma[2, 2]
        self.odometry_pub.publish(self.odometry)
        self.prevTime = self.currentTime

if __name__ == '__main__':
    try:
        rospy.init_node("localisation_node")
        rate = rospy.Rate(10)
        puzz = Localisation()
        while not rospy.is_shutdown():
            puzz.getOdometry()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
