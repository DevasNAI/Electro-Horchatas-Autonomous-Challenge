#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from aruco_detector.msg import ArucosDetected, ArucoDetected

import argparse

# Define command line arguments
ap = argparse.ArgumentParser()
ap.add_argument("-vs", "--video_source", type=str, default="/jetson_camera/image_raw",
                help="Video source ROS topic")
ap.add_argument("-ci", "--camera_info", type=str, default="/camera_info",
                help="Camera info ROS topic")
ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL",
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
        self.marker_size = 0.05  # Marker size in meters

        # Define Publishers
        self.aruco_pub = rospy.Publisher('aruco_detected', ArucosDetected, queue_size=10)
    
    # Callback for camera information
    def info_cb(self, data):
        self.camera_matrix = np.array(data.K).reshape(3, 3)
        self.dist_coeffs = np.array(data.D)

    # Callback for image processing
    def image_cb(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        self.corners, self.ids, self.rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        if self.ids is not None:
            self.process_arucos()

    # Process detected ArUco markers
    def process_arucos(self):
        detected_arucos = []
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(self.corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
        for i, corner in enumerate(self.corners):
            center = corner.reshape((4,2)).mean(axis=0)
            pose = [tvecs[i][0][0], tvecs[i][0][1], tvecs[i][0][2]]  # Position in 3D space
            detected_arucos.append(ArucoDetected(id=int(self.ids[i][0]), pose=pose))
        self.aruco_pub.publish(ArucosDetected(arucos_detected=detected_arucos))

    # Main function to set up subscribers and maintain node operation
    def main(self):
        self.image_sub = rospy.Subscriber(args["video_source"], Image, self.image_cb)
        self.info_sub = rospy.Subscriber(args["camera_info"], CameraInfo, self.info_cb)
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        print("Aruco Detection Node Running or ADNR jeje...")
        AD = ArucoDetector()
        AD.main()
    except rospy.ROSInterruptException:
        pass

