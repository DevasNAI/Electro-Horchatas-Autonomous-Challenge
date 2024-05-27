
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import rospy
#   This node SHOULD RUN ON THE PUZZLEBOT
if __name__ =='__main__':
    rospy.init_node("image_sender", anonymous=True)
    objCVBridge = CvBridge()
    pub  = rospy.Publisher("/image_topic", Image, queue_size=10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        #   Reads Map. THIS MAP SHOULD BE LOCATED WHERE THE PUZZLEBOT HAS THE FILE
        img = cv2.imread("PistaMap.pgm", cv2.IMREAD_COLOR)
        height, width = img.shape[:2]
        centerX, centerY = (width // 2, height // 2)
        #   Fixes image rotation
        M = cv2.getRotationMatrix2D((centerX, centerY), 10, 1.0)
        rotated = cv2.warpAffine(img, M, (width, height))
        #   Upscales image
        img = cv2.resize(rotated, (640, 480))

        rospy.loginfo('publishing image')
        print(type(img), img.shape)
        #   Publish image
        if img is not None:
            pub.publish(objCVBridge.cv2_to_imgmsg(img, "bgr8"))

        rate.sleep()
cv2.destroyAllWindows()

#4:11