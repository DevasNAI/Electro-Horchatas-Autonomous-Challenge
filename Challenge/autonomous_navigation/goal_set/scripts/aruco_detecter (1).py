#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from ros_deep_learning.msg import ArucosDetected, ArucoDetected
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import Odometry
import yaml

import argparse

# Define command line arguments
ap = argparse.ArgumentParser()
ap.add_argument("-vs", "--video_source", type=str, default="/jetson_camera/image_raw",
                help="Video source ROS topic")
ap.add_argument("-ci", "--camera_info", type=str, default="/camera_info",
                help="Camera info ROS topic")
ap.add_argument("-t", "--type", type=str, default="DICT_5X5_50",
                help="Type of ArUCo tag to detect")

# Use parse_known_args to ignore unknown args
args, unknown = ap.parse_known_args()
args = vars(args)

# Verify and get the ArUco dictionary
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

if args["type"] not in ARUCO_DICT:
    rospy.logerr("ArUCo tag type '{}' is not supported".format(args["type"]))
    sys.exit(0)

aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
aruco_params = cv2.aruco.DetectorParameters_create()

# Define the ArucoDetector class
class ArucoDetector:
    def __init__(self):
        rospy.init_node('aruco_detector')
        self.rate = rospy.Rate(100)  # Define the rate at which to operate the loop
        self.bridge = CvBridge()  # Initialize CvBridge to convert ROS images to OpenCV format
        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_size = 0.143  # Marker size in meters
        self.pose = Pose()
        self.cv_image = None
        # Define Publishers
        self.aruco_pub = rospy.Publisher('aruco_detected', ArucosDetected, queue_size=10)

        # Define Subscriber
        self.current_pose = Pose()
        self.current_pose.position.x = 0
        self.current_pose.position.y = 0
        self.current_pose.position.z = 0
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.wroteFile = False
        self.goals = {
            "cube": Point(0,0,0),
            "base_a": Point(0,0,0),
            "base_b": Point(0,0,0),
            "base_c": Point(0,0,0),
        }
        self.goalIDs = {
                "cube": 6,
                "base_a": 1,
                "base_b": 2,
                "base_c": 3,
            }

        self.id = 0

    # Callback for camera information
    def info_cb(self, data):
        self.camera_matrix = np.array(data.K).reshape(3, 3)
        self.dist_coeffs = np.array(data.D)

    # Callback for image processing
    def image_cb(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        self.corners, self.ids, self.rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        if self.ids is not None:
            self.process_arucos()

    def odom_callback(self, msg):
        # Update the current pose based on odometry data
        self.current_pose = msg.pose.pose

    # Process detected ArUco markers
    def process_arucos(self):
        detected_arucos = []
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(self.corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
        for i, corner in enumerate(self.corners):
            center = corner.reshape((4,2)).mean(axis=0)
            self.pose.position.x = tvecs[i][0][2]
            self.pose.position.y = tvecs[i][0][0]
            self.pose.position.z = tvecs[i][0][1]
            detected_arucos.append(ArucoDetected(id=int(self.ids[i][0]), pose=self.pose))
            if self.ids[i][0] == self.goalIDs["cube"]:
                self.goals["cube"].x = self.pose.position.z + self.current_pose.position.x
                self.goals["cube"].y = self.pose.position.x + self.current_pose.position.y
                self.goals["cube"].z = self.pose.position.y + self.current_pose.position.z
            elif self.ids[i][0] == self.goalIDs["base_a"]:
                self.goals["base_a"].x = self.pose.position.z + self.current_pose.position.x
                self.goals["base_a"].y = self.pose.position.x + self.current_pose.position.y
                self.goals["base_a"].z = self.pose.position.y + self.current_pose.position.z
            elif self.ids[i][0] == self.goalIDs["base_b"]:
                self.goals["base_b"].x = self.pose.position.z + self.current_pose.position.x
                self.goals["base_b"].y = self.pose.position.x + self.current_pose.position.y
                self.goals["base_b"].z = self.pose.position.y + self.current_pose.position.z
            elif self.ids[i][0] == self.goalIDs["base_c"]:
                self.goals["base_c"].x = self.pose.position.z + self.current_pose.position.x
                self.goals["base_c"].y = self.pose.position.x + self.current_pose.position.y
                self.goals["base_c"].z = self.pose.position.y + self.current_pose.position.z

        self.aruco_pub.publish(ArucosDetected(arucos_detected=detected_arucos))

    def none_in_dict(self, d):
        for _, value in d.iteritems():
            if value is None:
               return True
        return False

    # Main function to set up subscribers and maintain node operation
    def main(self):
        self.image_sub = rospy.Subscriber(args["video_source"], Image, self.image_cb)
        self.info_sub = rospy.Subscriber(args["camera_info"], CameraInfo, self.info_cb)
        while not rospy.is_shutdown():
            if self.cv_image is not None:
                if self.wroteFile == False and not self.none_in_dict(self.goals):
                    print("Writting File")
                    with open("goals.yaml", "w") as file:
                        yaml.dump(self.goals, file)
                        self.wroteFile = True

            self.rate.sleep()

if __name__ == '__main__':
    try:
        print("Aruco Detection Node Running or ADNR jeje...")
        AD = ArucoDetector()
        AD.main()
    except rospy.ROSInterruptException:
        pass

