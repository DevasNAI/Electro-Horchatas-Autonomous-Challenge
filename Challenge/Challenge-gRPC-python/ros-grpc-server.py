
import signal
import cv2
import numpy as np
import base64
import threading
from concurrent import futures

import rospy
import grpc
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

import puzzlebot_pb2
import puzzlebot_pb2_grpc

#    python -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. puzzlebot.proto
#   python -m grpc_tools.protoc
#   Define object 
object_positionROS = [0, 0, 0]
grpc_object_position = [0, 0 ,0]

class puzzlebotOdom(puzzlebot_pb2_grpc.PuzzlebotOdometryServicer):
    def __init__(self):
        self.odom = Odometry()
        self.dataDict = {"x": 0.0, "y": 0.0, "z": 0.0, "xq": 0.0, "yq": 0.0, "zq": 0.0, "w": 0.0, "linX_Vel": 0.0, "angTheta_Vel": 0.0}
        rospy.Subscriber("/odom", Odometry, self.UpdateData)
        print("Initialized gRPC Server")
        
    def UpdateData(self, data):
        self.odom = data
        self.dataDict["x"] = self.odom.pose.pose.position.x
        self.dataDict["y"] = self.odom.pose.pose.position.y
        self.dataDict["z"] = self.odom.pose.pose.orientation.z
        #self.dataDict["zq"] = self.odom.pose.pose.orientation.z #aqui mandar yaw
        #self.dataDict["w"] = self.odom.pose.pose.orientation.w
        #print(rospy.get_caller_id() + " / Got data: " + str(self.data))
    
    def GetOdometry(self, request, context):
        print("Got call 2: " + context.peer())
        results = puzzlebot_pb2.Odometry()
        results.poseX = self.dataDict["x"]
        results.poseY = self.dataDict["y"]
        results.poseZ = self.dataDict["z"]
        #results.orientationW = self.dataDict["w"]
        return results
    
    def GetImageResult(self, request, context):
        #   Reads Map format
        img = cv2.imread("PistaMap.pgm", cv2.IMREAD_COLOR)
        height, width = img.shape[:2]
        centerX, centerY = (width // 2, height // 2)
        M = cv2.getRotationMatrix2D((centerX, centerY), 10, 1.0)
        #   Rotates (if  map is turned over)
        rotated = cv2.warpAffine(img, M, (width, height))
        img = cv2.resize(rotated, (640, 480))
        #   Compresses image to send
        self.img_compressed = np.array(cv2.imencode('.jpg', img)[1]).tobytes()
        #   Sends through grpc
        results = puzzlebot_pb2.ImageFloat()
        results.b64img = img
        results.width = width
        results.height = height
        return results
        

terminate = threading.Event()
def terminate_server(signum, frame):
    """
        Detecta interrupciones cuando hago ctrl C, la libreria signal permite hacer esto
    """
    print("Got signal {}, {}".format(signum, frame))
    rospy.signal_shutdown("Ending ROS Node")
    terminate.set()


if __name__ =='__main__':
    print("------ROS-gRPC-Wrapper------")
    signal.signal(signal.SIGINT, terminate_server)

    print("Starting ROS Node")
    rospy.init_node("odom_wrapper", anonymous=True)

    print("Starting gRPC Server")
    server_addr = "[::]:7042"
    service = puzzlebotOdom()
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    puzzlebot_pb2_grpc.add_PuzzlebotOdometryServicer_to_server(service, server)
    server.add_insecure_port(server_addr)
    server.start()
    print("gRPC Server listening on: " + server_addr)
    
    print("Running ROS Node")
    rospy.spin()

    terminate.wait()
    print("Stopping gRPC Server")
    server.stop(1).wait()
    print("Exited")

# python -m grpc_tools.protoc -I./protos --python_out=./protos --grpc_python_out=./protos ./proto




