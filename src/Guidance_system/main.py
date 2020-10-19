
import guidance_sys

import numpy as np
import cv2, os
from cv2 import aruco

import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd

from picamera.array import PiRGBArray
from picamera import PiCamera

import time

import serial
ternimal_UART= False
try:
    os.system("sudo chmod a+rw /dev/serial0")
    ser = serial.Serial('/dev/serial0', 9600, timeout=1)
    ser.flush()
    print("open the Serial0")
    ternimal_UART =True
except:
    print("Can't open Serial dev pls add bash command: sudo chmod a+rw /dev/serial0")


import RPi.GPIO as GPIO
from time import sleep

io_pin=[17,27,22,5,23,24,25,6]
GPIO.setmode(GPIO.BCM)
for pin in io_pin:
    GPIO.setup(pin, GPIO.OUT)



w = 640
h = 480
frame_rate = 32


def open_UART_port():


    try:
        os.system("sudo chmod a+rw /dev/serial0")
        ser = serial.Serial('/dev/serial0', 9600, timeout=1)
        ser.flush()
        print("open the Serial0")
        ternimal_UART =True
    except:
        print("Can't open Serial dev pls add bash command: sudo chmod a+rw /dev/serial0")


if __name__ == '__main__':
    #calbrate = guidance_sys.calibrate()
    #ret, mtx, dist, rvecs, tvecs = calbrate.get()


    open_UART_port()
    #initail raspberry pi camera
    p_camera = guidance_sys.camera(w=w,h=h,framerate=frame_rate)

    image_process = guidance_sys.process()

    print("Start")
    #for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_0port=True):
    for frame in p_camera.video_stream():
        start= time.time()
        time_used = time.time()

        image_process.update(frame)

        #image_process.draw()

        for io,pin  in zip(image_process.IO_Mapping(),io_pin):
            GPIO.output(pin,int(io))

        time_used=time.time() -start
        image_process.string_out = f"Time use: {round(time_used,3)}\n\n"
        #image_process.string_out = f"{round(time_used,4)}"
        image_process.sendmessage()



        if ternimal_UART:
            ser.write(image_process.sendmessage())
        else:
            image_process.sendmessage()

            try:
                os.system("sudo chmod a+rw /dev/serial0")
                ser = serial.Serial('/dev/serial0', 9600, timeout=1)
                ser.flush()
                print("open the Serial0")
                ternimal_UART =True
            except:
                print("Can't open Serial dev pls add bash command: sudo chmod a+rw /dev/serial0")


        print(image_process.terminal_UART)

        #cv2.imshow("Frame", image)
        key = cv2.waitKey(1) & 0xFF
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
             break

        # clear the stream in preparation for the next frame
        p_camera.rawCapture.truncate(0)







