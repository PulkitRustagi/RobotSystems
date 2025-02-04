#!/usr/bin/env python3

from picamera2 import Picamera2
import libcamera
import cv2
from flask import Flask, Response, render_template

picam2 = Picamera2()
preview_config = picam2.preview_configuration
preview_config.size = (640, 480)
preview_config.format = 'RGB888'
preview_config.colour_space = libcamera.ColorSpace.Sycc()
preview_config.buffer_count = 4
preview_config.queue = True
preview_config.controls.FrameRate = 30

try:
    picam2.start()
except Exception as e:
    print(f"\033[38;5;1mError:\033[0m\n{e}")
    print("\nPlease check whether the camera is connected well" +\
        "You can use the \"libcamea-hello\" command to test the camera"
        )
    exit(1)

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(process(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def process():
    while True:
        img = picam2.capture_array()  
        frame = cv2.imencode('.jpg', img)[1].tobytes()

        yield (b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

try:
    app.run(host='0.0.0.0', debug=False, threaded=True)
except Exception as e:
    print(e)