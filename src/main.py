import calibration

import numpy as np
import cv2, os
from cv2 import aruco

import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd

from picamera.array import PiRGBArray
from picamera import PiCamera

import time

w = 640
h = 480
frame_rate = 32

if __name__ == '__main__':
    #calbrate = calibration.calibrate()
    #ret, mtx, dist, rvecs, tvecs = calbrate.get()

    #initail rap camera
    p_camera = calibration.camera(w=w,h=h,framerate=frame_rate)

    image_process = calibration.process()


    '''
    camera = PiCamera()
    camera.resolution = (w, h)
    camera.framerate = frame_rate
    rawCapture = PiRGBArray(camera, size=(w, h))
    # allow the camera to warmup
    time.sleep(0.1)
    # capture frames from the camera
    '''
    print("Start")
    #for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_0port=True):
    for frame in p_camera.video_stream():

        image_process.update(frame)
        image_process.draw()
        #cv2.imshow("Frame", image)
        key = cv2.waitKey(1) & 0xFF
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
             break

        # clear the stream in preparation for the next frame
        p_camera.rawCapture.truncate(0)




