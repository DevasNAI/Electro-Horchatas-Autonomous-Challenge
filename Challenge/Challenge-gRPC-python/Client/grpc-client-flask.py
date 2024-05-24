#   A01245418 Andrés Sarellano Acevedo
#   Crea un wrapper de grpc que lea alguna imagen publicada por un nodo de ROS, esa imagen se la va a servir a un cliente de GRPC, el cliente de GRPC debe desplegar en la pantalla
import signal
import cv2
import numpy as np
import base64
import threading
from concurrent import futures

from flask import Flask, render_template
import requests
import json
import schedule
import time


import grpc
import puzzlebot_pb2
import puzzlebot_pb2_grpc

app = Flask(__name__)
result = None

channel = grpc.insecure_channel("localhost:7042")






stub = puzzlebot_pb2_grpc.PuzzlebotOdometryStub(channel)


def call_api():
    global result
    url = 'http://127.0.0.1:8042/restgatewaydemo/getmultcoords'
    data = {}
    headers = {'Content-type': 'application/json'}
    odometry = puzzlebot_pb2.odometryM()
    result = stub.GetOdometry(odometry)

def schedule_api_call():
    while True:
        schedule.run_pending()
        time.sleep(1)


if __name__ == '__main__':
    call_api()
    schedule.every(0.5).seconds.do(call_api)

    t = threading.Thread(target=schedule_api_call)
    t.start()

    


print(response)
img = cv2.imread(response)
cv2.imshow('img', img)
cv2.waitKey(1)




#    python -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. wrapper-image.proto
#   python -m grpc_tools.protoc
#   Define object 


