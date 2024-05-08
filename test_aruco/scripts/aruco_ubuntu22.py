import cv2
import cv2.aruco as aruco
import numpy as np
import math

class Aruco:
    def __init__(self):
        # Initialize video capture
        self.video_cap = True
        self.cap = cv2.VideoCapture(0)

        # Load calibration data
        self.calib_data_path = "../calib_data/MultiMatrix.npz"
        self.calib_data = np.load(self.calib_data_path)
        self.cam_mat = self.calib_data["camMatrix"]
        self.dist_coef = self.calib_data["distCoef"]
        self.r_vectors = self.calib_data["rVector"]
        self.t_vectors = self.calib_data["tVector"]

        # ArUco marker settings
        self.MARKER_SIZE = 2  # centimeters
        self.CUBE_SIZE = 5 #centimeters 2
        self.marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.param_markers = aruco.DetectorParameters_create()

    def findAruco(self, img, marker_size=4, total_markers=50, draw=True):
        # Convert image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Get ArUco dictionary
        key = getattr(aruco, f'DICT_{marker_size}X{marker_size}_{total_markers}')
        arucoDict = aruco.getPredefinedDictionary(key)
        
        # Detect markers
        #bbox, ids, _ = self.detector.detectMarkers(gray)
        bbox, ids, _= aruco.detectMarkers(
            gray, self.marker_dict, parameters=self.param_markers
        )
        
        # Draw markers if required
        if draw:
            aruco.drawDetectedMarkers(img, bbox, ids)
        
        return bbox, ids


    def distanceEstimation(self,frame):
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, self.marker_dict, parameters=self.param_markers
        )
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
                    f"id: {ids[0]} Dist: {round(distance, 2)}",
                    top_right,
                    cv2.FONT_HERSHEY_PLAIN,
                    1.3,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    frame,
                    f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
                    bottom_right,
                    cv2.FONT_HERSHEY_PLAIN,
                    1.0,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
            return tVec
        
    def distance_between_markers(self,marker1_pos, marker2_pos):
        return np.linalg.norm(marker1_pos - marker2_pos)

    def distance_to_center(self,marker1_pos, marker2_pos):
        return np.linalg.norm((marker1_pos + marker2_pos) / 2)


    def distanceDifference(self,res,vec,img):
        ids_arr = res
        pose_arr = vec
        #print(pose_arr)
        if ids_arr is not None:
            if pose_arr is not None:
                #print("id: ",ids_arr.size, "& pose: ", pose_arr.size)
                if ids_arr.size == 1 and pose_arr.size == 3:
                    print("1")
                elif ids_arr.size == 2 and pose_arr.size == 6:
                    print("2")
                    print("FIRST",pose_arr[0,0,0])
                    print("SECOND",pose_arr[1,0,0])
                    print("ABS_DIFF= ",abs(pose_arr[0,0,0] - pose_arr[1,0,0]))

                    print ("Euclidian = ",abs(math.dist(pose_arr[0,0],pose_arr[1,0]))) #y's
                    marker1_pos = pose_arr[0][0]
                    marker2_pos = pose_arr[1][0]
                    distance_markers = self.distance_between_markers(marker1_pos[:2], marker2_pos[:2])
                    distance_center = self.distance_to_center(marker1_pos[:2], marker2_pos[:2])
                    print("Distance between markers:", distance_markers)
                    print("Distance to center:", distance_center)
                
                elif ids_arr.size == 3 and pose_arr.size ==9:
                    print("3")
                else:
                    print("IGNORE")
            else:
                print("wait")
        else:
            print("Normal Exec")
 

    def Aruco(self):
        while True:
            # Read frame from video capture
            if self.video_cap:
                _, self.img = self.cap.read()
            else: 
                self.img = cv2.imread("/home/sebas/electro_horchatas/scripts/2.png")
                self.img = cv2.resize(self.img, (0,0), fx=1, fy=1)
            
            # Find ArUco markers in the frame
            _, self.res = self.findAruco(self.img)
            self.vec = self.distanceEstimation(self.img)
            self.distanceDifference(self.res,self.vec,self.img)
            # Display the frame with detected markers
            cv2.imshow("img", self.img)

            # Exit loop if 'q' is pressed
            if cv2.waitKey(1) == 113:
                break

# Create an instance of the Aruco class and call the Aruco() methods
aruco_instance = Aruco()
aruco_instance.Aruco()

