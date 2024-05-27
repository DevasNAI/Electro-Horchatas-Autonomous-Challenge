
import signal
import cv2
import numpy as np
import base64
from cv_bridge import CvBridge
import threading
from concurrent import futures

import rospy
import grpc
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image   

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
        self.br = CvBridge()
        self.dataDict = {"x": 0.0, "y": 0.0, "z": 0.0, "xq": 0.0, "yq": 0.0, "zq": 0.0, "w": 0.0, "linX_Vel": 0.0, "angTheta_Vel": 0.0}
        rospy.Subscriber("/odom", Odometry, self.UpdateData)
        rospy.Subscriber("/image_topic", Image, self.UpdateData)
        print("Initialized gRPC Server")
        
    def UpdateData(self, data):
        self.odom = data
        self.dataDict["x"] = self.odom.pose.pose.position.x
        self.dataDict["y"] = self.odom.pose.pose.position.y
        self.dataDict["z"] = self.odom.pose.pose.orientation.z
        #self.dataDict["zq"] = self.odom.pose.pose.orientation.z #aqui mandar yaw
        #self.dataDict["w"] = self.odom.pose.pose.orientation.w
        #print(rospy.get_caller_id() + " / Got data: " + str(self.data))
    
    def UpdateImage(self, data):
        image = self.br.imgmsg_to_cv2(data, "bgr8")
        #bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
       # print(self.image[0])
        self.shape = image.shape
        self.img_compressed = np.array(cv2.imencode('.jpg', image)[1]).tobytes()

    def GetOdometry(self, request, context):
        print("Got call 2: " + context.peer())
        results = puzzlebot_pb2.Odometry()
        results.poseX = self.dataDict["x"]
        results.poseY = self.dataDict["y"]
        results.poseZ = self.dataDict["z"]
        #results.orientationW = self.dataDict["w"]
        return results
    
    def GetImageResult(self, request, context):
        #   Sends through grpc
        results = puzzlebot_pb2.ImageFloat()
        results.b64img = self.img_compressed
        results.width = self.shape[1]
        results.height = self.shape[0]
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



