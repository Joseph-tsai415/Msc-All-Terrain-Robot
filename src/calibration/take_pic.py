"""
Press Q to capture photo and excit
"""

import time
import picamera
import numpy as np
import cv2
from picamera.array import PiRGBArray


with picamera.PiCamera() as camera:
    camera.resolution = (640, 480)
    camera.framerate = 24
    rawCapture = PiRGBArray(camera, size=(640, 480))
    
    
    time.sleep(2)
    print(__doc__)
    #image = np.empty((640 * 480 * 3,), dtype=np.uint8)
    
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
        start= time.time()
        image = frame.array
        cv2.imshow("img", image)
        rawCapture.truncate(0)
        #cv2.waitKey(1) & 0xFF
        if(cv2.waitKey(1) == ord('q')):
            cv2.imwrite("photo_5.png", image);
            exit(0)
