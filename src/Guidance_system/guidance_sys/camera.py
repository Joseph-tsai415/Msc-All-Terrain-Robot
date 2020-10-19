
import numpy as np
import cv2, os
from cv2 import aruco


from picamera.array import PiRGBArray
from picamera import PiCamera

import time


class camera():
    """
    The class called camera is initialised with constants appropriate
    for the given target Calibration
    """
    def __init__(self,w=640,h=480,framerate=32,frame=None):
        """
        This is a initialisation function for setting up the pi camera on Rasperry pi.
        Parameters
        ----------
        w: int, video width
        h: int, video height
        framerate: int, fps
        frame: array-like, shape (w, h)
        """

        self.w = w
        self.h = h
        self.framerate = framerate
        self.setup_cam()
        #self.get_frame(frame=frame)
    def video_stream(self):
        '''
        continuous capturing
        '''
        return self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True)


    def setup_cam(self):
        '''
        Setup a Picamera and wait 0.1s for warm
        '''
        self.camera = PiCamera()
        self.camera.resolution = (self.w, self.h)
        self.camera.framerate = self.framerate
        self.rawCapture = PiRGBArray(self.camera, size = (self.w, self.h) )
        # allow the camera to warmup
        time.sleep(0.1)

    def get_frame(self,frame=None):
        '''
        To get a frame
        '''
        if frame == None:

            # grab an image from the camera
            self.camera.capture(self.rawCapture, format="bgr")
            self.frame = self.rawCapture.array
        else:
            self.frame=frame

