#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32, Time, Header
from geometry_msgs.msg import Twist, PoseWithCovariance, TwistWithCovariance, PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry


from flask import Flask, render_template
from flask import request, jsonify
import requests
import json
import schedule
import time
import threading



app = Flask(__name__)

class OdometryApi():
    def __init__(self):
        rospy.init_node("localisation_node")
        self.x = 0
        self.odometry = Odometry()
        self.robot_pose = Pose()
        self.robox = 0.0
        self.roboty = 0.0
        self.robotz = 0.0
        self.orientationx = 0.0
        self.orientationy = 0.0
        self.orientationz = 0.0
        self.orientationw = 0.0
        
        
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        
    def odom_cb(self, msg):
        self.odometry = msg.data
        self.robotx = self.odometry.pose.pose.position.x
        self.roboty = self.odometry.pose.pose.position.x
        self.robotz = self.odometry.pose.pose.position.x
        self.orientationx = self.odometry.pose.pose.orientation.x
        self.orientationy = self.odometry.pose.pose.orientation.y
        self.orientationz = self.odometry.pose.pose.orientation.z
        self.orientationw = self.odometry.pose.pose.orientation.w

    def get_pose(self):
        self.robot_pose = self.odometry.pose.pose
    def get_poseX(self):
        return self.robotx
    def get_poseY(self):
        return self.roboty
    def get_poseZ(self):
        return self.robotz
    def get_orientationX(self):
        return self.orientationx
    def get_orientationX(self):
        return self.orientationy
    def get_orientationX(self):
        return self.orientationz
    def get_orientationW(self):
        return self.orientationw
    
    def print_pose_cartesian(self):
        return {"X: ": str(self.robotx), 'Y: ': str(self.roboty), 'Z': str(self.robotz)}
    def print_pose_orientation(self):
        return {"X: ": str(self.orientationx), 'Y: ': str(self.orientationy), 'Z': str(self.orientationz), 'Z': str(self.orientationw)}
    

    

#   Muestra la pagina principal
@app.route("/")
def hello():
    return "Successfull conection"

@app.route('/pose/x',methods=['GET'])
def getTemp():
    try:
        return jsonify( {"X": 0} )
    except (IOError, TypeError) as e:
        return jsonify({"error": e})





if __name__=='__main__':
    try:
        odometry = OdometryApi()
         #   Define la salida 
        app.run(host="127.0.0.1:8042", port=8002) # 10.242.43.39
        #   Node initialization
        
        rate = rospy.Rate(10)
        
        
        while not rospy.is_shutdown():
            odometry.getOdometry()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass#Initialise and setup node





            


