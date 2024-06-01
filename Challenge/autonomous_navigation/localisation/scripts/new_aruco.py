#!/usr/bin/env python2.7
import cv2
import cv2.aruco as aruco
import numpy as np
import math
import random
import rospy
import rospkg
import sys
from cv_bridge import CvBridge, CvBridgeError
from localisation.msg import Num
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
#from aruco_detector.msg import ArucosDetected, ArucoDetected

#WORKS

class Aruco:
    def __init__(self):
        # Initialize video capture
        self.bridge = CvBridge()
        rospack = rospkg.RosPack()
        self.pub_msg = Num()
        package_path = rospack.get_path('localisation')
        rospy.init_node("aruco_node")
        self.pub = rospy.Publisher("/servo", Float32, queue_size=1)
        self.image_sub = rospy.Subscriber("/jetson_camera/image_raw", Image, self.callback)
        self.aruco_pose = rospy.Publisher("/aruco_pose", Num, queue_size=10)
        self.img = None
        self.videocap = True

        self.state = 0
        self.last_val = 0
        self.prev_error=0
        self.last_time=0

        self.Kp = 6
        self.Kd = 6
        self.avg_val=[]
        self.maxAngle = 0

        # Load calibration data
        self.calib_data_path = package_path + "/calib_data/MultiMatrix.npz"
        self.calib_data = np.load(self.calib_data_path)
        self.cam_mat = self.calib_data["camMatrix"]
        self.dist_coef = self.calib_data["distCoef"]
        #self.cam_mat = [ 802.18001,    0.     ,  600.47803,
        #    0.     ,  804.52738,  394.93228,
        #    0.     ,    0.     ,    1.     ]
        #self.dist_coef = [-0.332049, 0.103269, -0.000880, 0.001494, 0.000000]
        self.r_vectors = self.calib_data["rVector"]
        self.t_vectors = self.calib_data["tVector"]

        # ArUco marker settings
        self.MARKER_SIZE = 14.3  # centimeters
        self.CUBE_SIZE = 5  # centimeters
        self.marker_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
        self.param_markers = aruco.DetectorParameters_create()

    def aruco_cb(self, aruco_msg):
        self.aruco_pose = aruco_msg

    def callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            #print("GOT DATA")
            #self.cv_image = cv2.flip(self.cv_image, 0)
            #self.cv_image = cv2.flip(self.cv_image, 1)
            #self.img = cv2.resize(self.cv_image, (400, 300))

            #newcameramatrix, _ = cv2.getOptimalCameraMatrix(self.cam_mat, self.dist_coeffs, (400, 300), 1, (400, 300))


            #undistorted_image = cv2.undistort(self.img, self.cam_mat, self.dist_coef, None, newcameramatrix)
            #cv2.imshow("undistorted", undistorted_image)
            #print("got data")

        except CvBridgeError as e:
            rospy.logerr(e)

    def convert_to_float(self, tVec):
        data = tVec.flatten().astype(np.float32).tolist()
        return data
    
    def convert_to_int(self, id):
        data = id.flatten().astype(np.int32).tolist()
        return data
    
    def distanceEstimation(self,frame, marker_size=4, total_markers=50, draw=True):
        #print("RUNNING ESTIMATION")
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, self.marker_dict, parameters=self.param_markers)
        
        


        #if draw:
        #    aruco.drawDetectedMarkers(frame,marker_corners,marker_IDs)
    
        if marker_corners:
            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                marker_corners, self.MARKER_SIZE, self.cam_mat, self.dist_coef
            )
            )
            total_markers = range(0, marker_IDs.size)
            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                cv2.polylines(
                    frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
                )
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()
                # Calculating the distance
                distance = np.sqrt(
                    tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                )
            # Draw the pose of the marker
            # point = cv2.drawFrameAxes(frame, self.cam_mat, self.dist_coef, rVec[i], tVec[i], 4, 4)
            cv2.putText(
                frame,
                #f"id: {ids[0]} Dist: {round(distance, 2)}",
                "id: {0} Dist: {1}".format(ids[0],round(distance,2)),
                tuple(top_right),
                cv2.FONT_HERSHEY_PLAIN,
                1.3,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                #f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
                "x{0} y:{1}".format(round(tVec[i][0][0],1),round(tVec[i][0][1]),1),
                tuple(bottom_right),
                cv2.FONT_HERSHEY_PLAIN,
                1.0,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            control = self.distanceDifference(marker_IDs,tVec)
            tVec_msg = self.convert_to_float(tVec)
            id_msg = self.convert_to_int(marker_IDs)
            print(tVec_msg[2], distance)
            #print(type(id_msg), id_msg)
            self.pub_msg.id = id_msg[0]
            self.pub_msg.x = tVec_msg[2]  # x = zpuzzle
            self.pub_msg.y =  tVec_msg[0] # y = x_puzzle
            self.pub_msg.dist =  tVec_msg[1] # z = y puzzle

            self.aruco_pose.publish(self.pub_msg)
     
       
    
    def distanceDifference(self,res,vec):
        ids_arr = res
        pose_arr = vec
        #------------------------------------
         # vec syntax for 3 arucos= [1,5,6] -> [[0],[1],[2]]
         # vec syntax for 3 arucos= [1,5,6] -> [[0],[1],[2]] = 0 ex. [x,y,dist]
        #------------------------------------
        print(res)
        val = 0
        if ids_arr is not None:
            if pose_arr is not None:
                #print("id: ",ids_arr.size, "& pose: ", pose_arr.size)
                if ids_arr.size == 1:
                    print("1")
                    if ids_arr == 6 and pose_arr[0,0,2] <8 and self.last_pose < 8:
                        val = 115
                        print("hola")
                    else:
                        val = 0
                    self.last_pose = pose_arr[0,0,2]

                    
                elif ids_arr.size ==2:
                    print("2")
                    print("FIRST",pose_arr[0,0,0])
                    print("SECOND",pose_arr[1,0,0])
                    #print("ABS_DIFF= ",abs(pose_arr[0,0,0] - pose_arr[1,0,0]))
                    if ids_arr[0] == 1 and ids_arr[1] == 5:
                        
                        local_distance = abs(pose_arr[1,0,0] - pose_arr[0,0,0])
                        val = 0
                        #val = self.gripperState(ids_arr,local_distance)

                    elif ids_arr[0] == 1 and ids_arr[1] == 6 and pose_arr[1,0,2] <8:
                        local_distance = abs(pose_arr[1,0,0] - pose_arr[0,0,0])
                        #val = self.gripperState(ids_arr,local_distance)
                        val= 115

                    elif ids_arr[0] == 5 and ids_arr[1] == 6 and pose_arr[1,0,2] <8:
                        local_distance = abs(pose_arr[1,0,0] - pose_arr[0,0,0])
                        #val = self.gripperState(ids_arr,local_distance)
                        val= 115
                    #print ("Euclidian = ",abs(math.dist(pose_arr[0,0],pose_arr[1,0]))) #y's
                    #marker1_pos = pose_arr[0][0]
                    #marker2_pos = pose_arr[1][0]
                elif ids_arr.size == 3 :
                    #print("3")
                    #print("FIRST",pose_arr[0])
                    #print("SECOND",pose_arr[1])
                    #print("THIRD",pose_arr[2])
                    if ids_arr[2] == 6 and pose_arr[2,0,2] < 8:
                        local_distance = abs(pose_arr[2,0,0] - pose_arr[0,0,0])
                        #val = self.gripperState(ids_arr,local_distance)
                        val = 115
                    else:
                        val = 0
                    
                self.pub.publish(val)
           
        else:
            val =0
        print(val)
 

    def main(self):
        while not rospy.is_shutdown():
            # Find ArUco markers in the frame
            if self.img is not None:
                self.distanceEstimation(self.img)

                #self.distanceDifference(ids,tVec)
                #self.distanceDifference(ids,tVec,self.img)
                #if bbox is not None and self.res is not None:
                cv2.imshow("aruco", self.img)

            # Exit loop if 'q' is pressed
            if cv2.waitKey(1) == 113:
                break

        cv2.destroyAllWindows()


if __name__ == '__main__':
    aruco_proc = Aruco()
    aruco_proc.main()

