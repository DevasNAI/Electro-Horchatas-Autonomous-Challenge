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

import base64

import grpc
import puzzlebot_pb2
import puzzlebot_pb2_grpc

app = Flask(__name__)
result = None
result_img = None

channel = grpc.insecure_channel("localhost:7042")






stub = puzzlebot_pb2_grpc.PuzzlebotOdometryStub(channel)


def call_api():
    global result
    url = 'http://127.0.0.1:8042/puzzlebotMap/odom'
    data = {}
    headers = {'Content-type': 'application/json'}
    odometry = puzzlebot_pb2.Odometry()
    result = stub.GetOdometry(odometry)

def call_image_api():
    global result_img
    url = 'http://127.0.0.1:8042/puzzlebotMap/map'
    data = {}
    headers = {'Content-type': 'application/json'}
    image_g = puzzlebot_pb2.ImageFloat()
    result_img = stub.GetImage(image_g)

    image = np.asarray(bytearray(result_img.b64img), dtype="uint8")
    imagen = cv2.imdecode(image, cv2.IMREAD_COLOR)
    
    buffer = cv2.imencode('.jpg', imagen)[1]
    bas64 = base64.b64encode(buffer)
    # basenc = bas64.decode()
    # rere = {"b64img": basenc}
    result_img = bas64
    # result_img = json.dumps(rere)
    print(type(result_img))
   # with open("result_img.json", "w") as f:
    #    json.dump(result_img, f)
    


def schedule_api_call():
    while True:
        schedule.run_pending()
        time.sleep(1)


if __name__ == '__main__':
    call_api()
    call_image_api()
    schedule.every(0.5).seconds.do(call_api)
    schedule.every(0.5).seconds.do(call_image_api)

    t = threading.Thread(target=schedule_api_call)
    t.start()

    @app.route('/api/result')
    def get_result():
        odom2json = {"poseX": result.poseX, "poseY": result.poseY, "poseZ": result.poseZ, "orientationX": result.orientationX, "orientationY": result.orientationY, "orientationZ": result.orientationZ, "orientationW": result.orientationW, "LinXSpeed": result.LinXSpeed, "AngThetaSpeed": result.AngThetaSpeed }
        odomJson = json.dumps(odom2json)
        return str(odomJson)
    
    @app.route('/api/image')
    def get_image():
        #   b64 received image
        #image = np.asarray(bytearray(result_img.b64img), dtype="uint8")
        #print(result_img.b64img)
      #  newIM = result_img.decode('utf-8')
        #newIM = str(result_img.b64img)
        #print(type(newIM))
        #   Decoded image
        #imagen = cv2.imdecode(image, cv2.IMREAD_COLOR)
        #imgjson = {"b64img": newIM}
        #img2json = json.dumps(imgjson)
        #   Generates an image and reads that image
        #cv2.imwrite("result.png", imagen)  
        #imgpng = cv2.imread("result.png")

        #print(result_img)
        return  "data:image/jpg;base64," + str(result_img).replace("b'", "").replace("'", "")
    
    #@app.route('/api/image')
    #def get_image():
    #    if result_img is not None:
    #        return "data:image/jpg;base64," + result_img["b64img"].replace('"', '')
    #    else:
    #        return "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAUAAAAFCAYAAACNbyb1AAAAHE1EQVQI12P4//8/w38GIAXDIBKE0DHxgljNBAA09TXL0Y40HwAAAABJRU5ErkJggg=="
        

    
    @app.route('/')
    def show_result():
        return render_template('index.html')
    

    app.run(debug=True, port=8002)





#    python -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. wrapper-image.proto
#   python -m grpc_tools.protoc
#   Define object 

