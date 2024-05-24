
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
        self.dataDict["z"] = self.odom.pose.pose.position.z
        self.dataDict["xq"] = self.odom.pose.pose.orientation.x
        self.dataDict["yq"] = self.odom.pose.pose.orientation.y
        self.dataDict["zq"] = self.odom.pose.pose.orientation.z
        self.dataDict["w"] = self.odom.pose.pose.orientation.w
        self.dataDict["linX_Vel"] = self.odom.twist.twist.linear.x
        self.dataDict["angTheta_Vel"] = self.odom.twist.twist.angular.z
        #print(rospy.get_caller_id() + " / Got data: " + str(self.data))
    
    #def GetOdometry(self, request, context):
    #    print("Got call: " + context.peer())
    #    results = puzzlebot_pb2.Odometry()
    #    results.values.append(self.dataDict["x"])
    #    results.values.append(self.dataDict["y"])
    #    results.values.append(self.dataDict["z"])
    #    results.values.append(self.dataDict["xq"])
    #    results.values.append(self.dataDict["yq"])
    #    results.values.append(self.dataDict["zq"])
    #    results.values.append(self.dataDict["w"])
    #    results.values.append(self.dataDict["linX_Vel"])
    #    results.values.append(self.dataDict["angTheta_Vel"])
    #    return results
    def GetOdometry(self, request, context):
        print("Got call 2: " + context.peer())
        results = puzzlebot_pb2.odometryM()
        results.poseX = self.dataDict["x"]
        results.poseY = self.dataDict["y"]
        results.poseZ = self.dataDict["z"]
        results.orientationX = self.dataDict["xq"]
        results.orientationY = self.dataDict["yq"]
        results.orientationZ = self.dataDict["zq"]
        results.orientationW = self.dataDict["w"]
        results.LinXSpeed = self.dataDict["linX_Vel"]
        results.AngThetaSpeed = self.dataDict["angTheta_Vel"]
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




