#!/usr/bin/env python3
import cv2
import numpy as np
#   ROS Dependencies
import rospy
import grpc
from cv_bridge import CvBridge
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image

#   gRPC dependencies
import signal
import sys
sys.path.insert(1, "./protos")
import base64
import threading
from concurrent import futures
import image_ceron_pb2
import image_ceron_pb2_grpc

#    python -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. image_ceron.proto
#   python -m grpc_tools.protoc


class RPCDemoImpl(image_ceron_pb2_grpc.RPCDemoServicer):
    def __init__(self):
        """! RPCDemoImpl ROS-gRPC wrapper
        """
        #   Object position handler
        self.data = [0, 0, 0]
        #   Image Handlers
        self.br = CvBridge()
        self.img_compressed = None
        #   ROS Subscribers
        rospy.Subscriber("/object_position", Float64MultiArray, self.UpdateData)
        rospy.Subscriber("/image_result", Image, self.UpdateImage)
        print("Initialized gRPC Server")
        
    def UpdateData(self, data):
        """! Object position callback.

            @param  data Float data.   
        """
        self.data[0] = data.data[0]
        self.data[1] = data.data[1]
        self.data[2] = data.data[2]
        #print(rospy.get_caller_id() + " / Got data: " + str(self.data))
    
    def GetMultCoords(self, request, context):
        """! Gets Mult Coords from ROS data into gRPC Message
            @return An array of image data containing Image position
        """
        
        print("Got call: " + context.peer())
        #   Creates MultCoords gRPC message and adds local data and returns
        results = image_ceron_pb2.MultCoords()
        results.values.append(self.data[0])
        results.values.append(self.data[1])
        results.values.append(self.data[2])
        return results
    
    def GetImageResult(self, request, context):
        """! Gets Image from ROS
            @return An array of image data containing Image position
        """
        print("     GetImageResult Got Call: " + context.peer())
        #   Creates ImageResult gRPC message and adds Image data to the array message
        results = image_ceron_pb2.ImageResult()
        results.b64img = self.img_compressed
        results.width= self.shape[1]
        results.height = self.shape[0]
        return results

    def UpdateImage(self, data):
        """! Image callback compresses image to jpg and changes format to bytes"""
        #   Converts image to cv2 format
        image = self.br.imgmsg_to_cv2(data)
        self.shape = image.shape
        #   Converts image to jpg and returns a bytes array
        #   Convierte imagen en jpg, imencode regresa un arrego de bytes
        self.img_compressed = np.array(cv2.imencode('.jpg', image)[1]).tobytes()
        #   Podemos multiplicar el tamaño de la imagen por cuanto mide en bytes cada casilla
        print("Memory size of original image in bytes:", image.size* image.itemsize)
        print("Memory size of compressed image in bytes:", len(self.img_compressed))
        print("Memory size of base64 image in bytes", len(base64.b64encode(self.img_compressed)))

        #   If we want to stream the puzzlebot's image on the PC, it would be ok to compress it to .jpg
        #   Si queremos hacer streaming de la imagen del puzzlebot a la compu, estaria bien comprimirla con .jpg
        #self.img_compressed = base64.b64encode(self.img)


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
    rospy.init_node("object_position_wrapper", anonymous=True)

    print("Starting gRPC Server")
    server_addr = "[::]:7042"
    service = RPCDemoImpl()
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    image_ceron_pb2_grpc.add_RPCDemoServicer_to_server(service, server)
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




