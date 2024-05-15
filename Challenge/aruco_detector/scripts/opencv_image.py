#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, "
        "framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def publish_camera_image():
    rospy.init_node("jetson_camera_publisher", anonymous=True)
    image_pub = rospy.Publisher("/jetson_camera/image_raw", Image, queue_size=10)
    bridge = CvBridge()

    cap = cv2.VideoCapture(
        gstreamer_pipeline(
            sensor_id=0,
            capture_width=1280,
            capture_height=720,
            display_width=1280,
            display_height=720,
            framerate=30,
            flip_method=0,
        ),
        cv2.CAP_GSTREAMER,
    )

    if not cap.isOpened():
        rospy.logerr("Failed to open camera")
        return

    rospy.loginfo("Camera opened successfully")
    rate = rospy.Rate(10)  # Publicar a 10 Hz

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            try:
                image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
                image_pub.publish(image_msg)
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {}".format(e))
        rate.sleep()

    cap.release()

if __name__ == "__main__":
    try:
        publish_camera_image()
    except rospy.ROSInterruptException:
        pass

