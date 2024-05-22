import cv2
import cv2.aruco as aruco
import numpy as np
import math
import random
import rospy
import sys
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from sensor_msgs.msg import Image

#WORKS

class Aruco:
    def __init__(self):
        # Initialize video capture
        self.bridge = CvBridge()
        rospy.init_node('aruco_node', anonymous=True)
        self.pub = rospy.Publisher("/servo", Float32, queue_size=1)
        self.image_sub = rospy.Subscriber("/puzzlebot_1/camera/image_raw", Image, self.callback)
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
        self.calib_data_path = "../calib_data/MultiMatrix.npz"
        self.calib_data = np.load(self.calib_data_path)
        self.cam_mat = self.calib_data["camMatrix"]
        self.dist_coef = self.calib_data["distCoef"]
        self.r_vectors = self.calib_data["rVector"]
        self.t_vectors = self.calib_data["tVector"]

        # ArUco marker settings
        self.MARKER_SIZE = 2  # centimeters
        self.CUBE_SIZE = 5  # centimeters
        self.marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.param_markers = aruco.DetectorParameters_create()


    def callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            print("GOT DATA")
            #self.cv_image = cv2.flip(self.cv_image, 0)
            #self.cv_image = cv2.flip(self.cv_image, 1)
            #self.img = self.cv_image
            self.img = cv2.resize(self.cv_image, (400, 300))
            #print("got data")

        except CvBridgeError as e:
            rospy.logerr(e)

    def gripperState(self,ids,distance):
        print("ARUCO DETECTION STATE")
        current_time=rospy.get_time()
        dt=current_time-self.last_time
        error  = distance
        avg = 0
        
        pd=self.Kp*error + self.Kd*(error-self.prev_error)/dt

        if len(self.avg_val) >= 10:
            self.avg_val.pop(0)
        self.avg_val.append(pd)
        
        avg = sum(self.avg_val)/len(self.avg_val)

        print("error: ", error)
        self.prev_error=error
        self.last_time=current_time

        val=abs(self.maxAngle-abs(avg))
        print("val_avg_max: ",avg)
    
        return val


    def distanceEstimation(self,frame, marker_size=4, total_markers=50, draw=True):
        #print("RUNNING ESTIMATION")
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, self.marker_dict, parameters=self.param_markers
        )
        if draw:
            aruco.drawDetectedMarkers(frame,marker_corners,marker_IDs)
    
        if marker_corners:
            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                marker_corners, self.MARKER_SIZE, self.cam_mat, self.dist_coef
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
            point = cv2.drawFrameAxes(frame, self.cam_mat, self.dist_coef, rVec[i], tVec[i], 4, 4)
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
            #control = self.distanceDifference(marker_IDs,tVec)
            print(tVec)
       
    
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
                    
                elif ids_arr.size ==2:
                    print("2")
                    print("FIRST",pose_arr[0,0,0])
                    print("SECOND",pose_arr[1,0,0])
                    #print("ABS_DIFF= ",abs(pose_arr[0,0,0] - pose_arr[1,0,0]))
                    if ids_arr[0] == 1 and ids_arr[1] == 5:
                        
                        local_distance = abs(pose_arr[1,0,0] - pose_arr[0,0,0])
                        val = self.gripperState(ids_arr,local_distance)

                    elif ids_arr[0] == 1 and ids_arr[1] == 6:
                        local_distance = abs(pose_arr[1,0,0] - pose_arr[0,0,0])
                        val = self.gripperState(ids_arr,local_distance)

                    #print ("Euclidian = ",abs(math.dist(pose_arr[0,0],pose_arr[1,0]))) #y's
                    #marker1_pos = pose_arr[0][0]
                    #marker2_pos = pose_arr[1][0]
                elif ids_arr.size == 3 :
                    #print("3")
                    #print("FIRST",pose_arr[0])
                    #print("SECOND",pose_arr[1])
                    #print("THIRD",pose_arr[2])
                    if ids_arr[2] == 6 and pose_arr[2,0,2] < 3.75:
                        local_distance = abs(pose_arr[2,0,0] - pose_arr[0,0,0])
                        val = self.gripperState(ids_arr,local_distance)
                    
                else:
                    print("IGNORE")

                self.pub.publish(val)
            else:
                print("wait")
        else:
            print("Normal Exec")
 

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


