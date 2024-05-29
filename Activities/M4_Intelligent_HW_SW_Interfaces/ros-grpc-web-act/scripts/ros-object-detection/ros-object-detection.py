#!/usr/bin/env python3
import numpy as np
import cv2
from time import time
#   C++ library management dependency
import ctypes
#   ROS Dependencies
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image

#   Image source: https://isorepublic.com/photo/red-green-apples-2/

#   C++ library path
so_file = "/home/robotics/work/src/cpp_lib_demo.so"

if __name__ =='__main__':
    #   C++ Library object
    mylib = ctypes.CDLL(so_file)
    #   C++ Library input array pointers
    val_in = np.array([0.0, 0.0])
    val_out = np.array([0.0, 0.0])

    #   ROS OpenCV hander
    objCVBridge = CvBridge()

    #   En C++ habiamos dicho que ibamos a meter un argumento de arreglo que se iba a referir con un puntero
    #   Node initialization
    rospy.init_node("object detection", anonymous=True)
    #   Publishers
    pub  = rospy.Publisher("/object_position", Float64MultiArray, queue_size=10)
    pubImage = rospy.Publisher("/image_result", Image, queue_size=10)
    rate = rospy.Rate(100)

    #   C++ Library output array pointers
    lower_green = np.array([40, 50, 50])
    upper_green = np.array([80, 255, 255])

    while not rospy.is_shutdown():
        #   Reads image and resizes it
        img = cv2.imread("isorepublic-red-green-apples-1.jpg", cv2.IMREAD_COLOR)
        img = cv2.resize(img, (640, 480))

        #   Adds noise to image
        noise = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.randn(noise, 0, 60)
        img = cv2.add(img, noise)
        #   Adds filter to image
        hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask_image = cv2.inRange(hsv_image, lower_green, upper_green)
        #   Applies kernel to image to find green color
        kernel = np.ones((5, 5), np.uint8)
        mask_image = cv2.morphologyEx(mask_image, cv2.MORPH_OPEN, kernel)   # Erosion and dilatation
        mask_image = cv2.morphologyEx(mask_image, cv2.MORPH_CLOSE, kernel)  # Close Holes
        #   Finds countours of image
        contours, hierarchy = cv2.findContours(mask_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        max_area = 0
        max_contour = None

        for contour in contours:
            area = cv2.contourArea(contour)
            #   Finds max contour area and reassigns it.
            if(area > max_area):
                max_area = area
                max_contour = contour
        if max_contour is not None:
            #   Gets bounding rectangle's centroid
            x, y, w, h = cv2.boundingRect(max_contour)
            center_x = x + w / 2
            center_y = y + h / 2
            print("Center point: ({}, {})".format(center_x, center_y))
            cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
            #   Assigns centroid values to pointer array
            val_in = np.array([center_x, center_y])
        #   Shows image
        cv2.imshow('img', img)
        cv2.waitKey(1)

        #   Calls C++ library and returns updated values on pointer arrays
        #   Python must manage memory from the cpp_lib function
        # """Python tiene que manejar la memoria de la funcion del cpp_lib"""
        res = mylib.Mult100(val_in.ctypes, val_in.shape[0], val_out.ctypes, val_out.shape[0])
        print("Multiplied Center point:" + str(val_out))

        #   Assigns data to message
        msg = Float64MultiArray()
        msg.data = [val_out[0], val_out[1], time()]
        
        #   Publishes Image and bounding box array
        pub.publish(msg)
        pubImage.publish(objCVBridge.cv2_to_imgmsg(img, "bgr8"))

        rate.sleep()
cv2.destroyAllWindows()
    