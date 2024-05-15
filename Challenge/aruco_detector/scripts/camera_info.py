#!/usr/bin/env python3
import rospy
import yaml
import os
from sensor_msgs.msg import CameraInfo

def load_camera_info(yaml_file):
    """Load camera info from a YAML file."""
    with open(yaml_file, "r") as file_handle:
        calib_data = yaml.load(file_handle, Loader=yaml.SafeLoader)

    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]

    return camera_info_msg

def camera_info_publisher(yaml_file):
    rospy.init_node("camera_info_publisher", anonymous=True)
    pub = rospy.Publisher("/camera_info", CameraInfo, queue_size=10)

    camera_info_msg = load_camera_info(yaml_file)

    rate = rospy.Rate(10)  # Publicar a 10 Hz
    while not rospy.is_shutdown():
        pub.publish(camera_info_msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        # Proporciona la ruta a tu archivo .yaml
        yaml_file = yaml_file = os.path.expanduser("/ws/src/aruco_detector/yaml/camera_info.yaml")
        camera_info_publisher(yaml_file)
    except rospy.ROSInterruptException:
        pass

