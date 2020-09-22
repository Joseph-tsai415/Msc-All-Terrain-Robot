import numpy as np
import cv2, os
from cv2 import aruco

from picamera.array import PiRGBArray
from picamera import PiCamera

import time

from .calibrate import calibrate

from .pid import PID

class process():
    """
    The class called Calibrate is initialised with constants appropriate
    for the given target Calibration
    """

    def __init__(self,w=640,h=480,framerate=32,frame=None,size_of_marker = 0.05, mtx =None, dist =None,kP=0.8, kI=0.01, kD=0.5, target =None, sample_time=0.05):
        self.w = w
        self.h = h
        self.framerate = framerate
        if frame != None:
            self.frame=frame
            #self.update()
        else:
            self.frame=frame

        #This part is aruco marker parameter
        #=========================================
        self.size_of_marker=size_of_marker

        if mtx != None:
            self.mtx=mtx
            self.dist=dist
        else:
            calbrate = calibrate()
            self.mtx=calbrate.mtx
            self.dist=calbrate.dist

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters =  aruco.DetectorParameters_create()
        #=========================================


        self.pid_controller=PID(P=kP, I=kI, D=kD)
        if target == None:
            self.pid_controller.SetPoint = w/2.0
        else:
            self.pid_controller.SetPoint = target
        self.pid_controller.sample_time = sample_time #depend on the performance of compution

        self.track = False




    def update(self,frame, target =None):
        self.frame=frame
        image = frame.array
        #image coretion to save sorouce can close
        #image = cv2.undistort(image,mtx,dist,None)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #blur
        #blur = cv2.GaussianBlur(gray, (5, 5), 0)
        #self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        #self.parameters =  aruco.DetectorParameters_create()
        #parameters.adaptiveThreshWinSizeMax =23
        #print("adaptiveThreshWinSizeMin:",parameters.adaptiveThreshWinSizeMin," adaptiveThreshWinSizeMax:",parameters.adaptiveThreshWinSizeMax," adaptiveThreshWinSizeStep:",parameters.adaptiveThreshWinSizeStep)
        self.corners, self.ids, self.rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        # SUB PIXEL DETECTION improve cornor accurate
        corners_corection =None
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
        for corner in self.corners:
            cv2.cornerSubPix(gray, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)

        #frame_markers = image.copy()
        #frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)

        #estimate and caculate distance
        #self.size_of_marker =  0.05 # side lenght of the marker in meter
        self.rvecs,self.tvecs,_objPoints = aruco.estimatePoseSingleMarkers(self.corners, self.size_of_marker , self.mtx, self.dist)


        self.track = False

        #distance caculate and sort
        if self.tvecs is not None:
            #draw Aix into image
            self.distance=[]
            self.sort_index=[]
            for i in range(len(self.tvecs)):
                sum =0
                for p in range(3):
                    sum+= (self.tvecs[i][0][p]*180/np.pi)**2
                sum =  round(np.sqrt(sum),3)
                self.distance.append(sum)
                self.sort_index.append(i)
            #self.sort_index=sorted(range(len(self.distance)), key=lambda k: self.distance[k])
            #self.sort_index=np.argsort(self.distance)

            #sort index
            sort_dis = sorted(self.distance)
            i=0
            ii=0
            for i in range(len(self.tvecs)):
                for ii in range(len(self.tvecs)):
                    if self.distance[i] == sort_dis[ii]:
                        self.sort_index[i]=ii

            #print("sort index:",self.sort_index,"distance",self.distance)
            index=0
            counter=0
            pos_1=0
            pos_2=0
            pos_3=0
            pos_4=0
            pos_5=0
            pos_6=0
            self.track_index=[]
            for id in self.sort_index:
                if id == 0:
                    pos_1=index
                    counter += 1
                if id == 1:
                    pos_2=index
                    counter += 1
                if id == 2:
                    pos_3=index
                    counter += 1
                if id == 3:
                    pos_4=index
                    counter += 1
                if id == 4:
                    pos_5=index
                    counter += 1
                if id == 5:
                    pos_6=index
                    counter += 1
                index += 1

            if counter==6:
                self.track_index=[pos_1,pos_2,pos_3,pos_4,pos_5,pos_6]
            elif counter==5:
                self.track_index=[pos_1,pos_2,pos_3,pos_4]
            elif counter==4:
                self.track_index=[pos_1,pos_2,pos_3,pos_4]
            elif counter==3:
                self.track_index=[pos_1,pos_2]
            elif counter==2:
                self.track_index=[pos_1,pos_2]
            elif counter==1:
                self.track_index=[pos_1]
            #print(self.track_index)

            self.track_side_pos=np.zeros((len(self.track_index),1,2))
            self.track_side_pos_id=np.zeros((len(self.track_index))
            self.track_side_pos_id=self.ids[self.track_index]

            self.track_center_pos_2d=np.zeros((int(len(self.track_index)/2),1,2))
            self.track_center_pos_3d=np.zeros((int(len(self.track_index)/2),1,3))




            if counter%2 ==0 and counter >0:
                self.track =True


                for i in range(len(self.track_index)):
                    self.track_side_pos[i,0,0] = int(abs(self.corners[self.track_index[i]][0][0][0]+self.corners[self.track_index[i]][0][2][0])/2)
                    self.track_side_pos[i,0,1] = int(abs(self.corners[self.track_index[i]][0][0][1]+self.corners[self.track_index[i]][0][2][1])/2)

                track_side_pos_t = self.track_side_pos.reshape(int(len(self.track_index)/2),2,2) #reshape into pair(n,2,2)
                track_index_t = np.reshape(self.track_index, (int(len(self.track_index)/2),2,1))

                #self.track_index.reshape(int(len(self.track_index)/2),2,1)
                self.distance_track = []
                for i in range(len(track_side_pos_t)):
                    self.track_center_pos_2d[i,0,0] = int((track_side_pos_t[i][1][0]+track_side_pos_t[i][0][0])/2)
                    self.track_center_pos_2d[i,0,1] = int((track_side_pos_t[i][1][1]+track_side_pos_t[i][0][1])/2)

                    for p in range(3):
                        #print(track_index_t)
                        self.track_center_pos_3d[i,0,p]=int((self.tvecs[track_index_t[i,1],0,p]*180/np.pi+self.tvecs[track_index_t[i,0],0,p]*180/np.pi)/2.0)
                    sum =0
                    for p in range(3):
                        sum+= self.track_center_pos_3d[i,0,p]**2
                    sum =  round(np.sqrt(sum),2)
                    self.distance_track.append(sum)

                #track_transpose.reshape(len(self.track_index)/2,2,2)
                #(corners[index][0][1][0]-corners[index][0][0][0])/2,(corners[index][0][1][1]-corners[index][0][2][1])/2
                #int(abs(corners[self.track_index[0]][0][0][0]+corners[self.track_index[0]][0][2][0])/2),int(abs(corners[self.track_index[0]][0][0][1]+corners[self.track_index[0]][0][2][1])/2)
                #self.tvecs[i][0][p]*180/np.pi
                #print("Center",self.track_center_pos_2d,"side",self.track_side_pos,"dis",self.distance_track)

        #output
        if self.track == True:
            if target != None:
                self.pid_controller.SetPoint = target

            feedback_value = self.track_center_pos_2d[0,0,0]
            self.pid_controller.update(feedback_value)
            self.output = self.pid_controller.output
            print(self.output)
    def task_manager(self):
        '''
        manage the target, in different sutuiattio
        when the target is lost mybe the detaction is failure the task will keep going in the whitin time


        '''
        move =False


        if self.track == True:
            move =True
            self.frist = True
            if self.frist == True:
                self.target_id = self.track_side_pos_id[0:2]
                self.distance = self.distance_track

                    feedback_value = self.track_center_pos_2d[0,0,0]]
            current_time = time.time()
        else:
            if time.time()
            current_time








    def IO_Mapping(self):
        '''
        the I/O output array
        '''

    def draw(self,frame=None, target =None):
        #draw frist two marker and distance
        if frame != None:
            image = frame.array
        else:
            image = self.frame
            image = image.array
        imaxis = image.copy()
        length_of_axis = 0.04
        if self.ids is not None:
            #draw the markers and distance
            index=0
            #print(corners[index][0][:],ids[index])
            #imaxis = aruco.drawDetectedMarkers(imaxis.copy(), corners, self.ids)
            font = cv2.FONT_HERSHEY_SIMPLEX
            line = cv2.LINE_AA
            for id in self.sort_index:
                #org=(corners[index][0][2][0],corners[index][0][2][1])
                if id<6:
                    imaxis=cv2.putText(imaxis, text=str(self.distance[index]),
                                   org=(self.corners[index][0][1][0],self.corners[index][0][1][1]),
                                   fontFace=font, fontScale=0.4, color=(255,154,36),thickness=1, lineType=line)
                    imaxis = aruco.drawAxis(imaxis, self.mtx, self.dist, self.rvecs[index], self.tvecs[index], length_of_axis)
                #print ("ID:",ids[index]," rotation:",rvecs[index]*180/np.pi,"Transpose:",tvecs[index]*180/np.pi,"distance: ", distance[index])
                #print(int((corners[index][0][1][0]-corners[index][0][0][0])/2),int((corners[index][0][1][1]-corners[index][0][2][1])/2))
                index+=1
            if self.track == True:
                for i in range(len(self.track_side_pos)):
                    imaxis=cv2.circle(imaxis,center=(int(self.track_side_pos[i,0,0]), int(self.track_side_pos[i,0,1])), radius=3, color=(255, 0, 255), thickness=1)

                for i in range(len(self.track_center_pos_2d)):
                    imaxis=cv2.putText(imaxis, text=str(self.distance_track[i]),
                                   org=(int(self.track_center_pos_2d[i,0,0]), int(self.track_center_pos_2d[i,0,1])),
                                   fontFace=font, fontScale=0.4, color=(255,154,36),thickness=1, lineType=line)
                    imaxis=cv2.circle(imaxis,center=(int(self.track_center_pos_2d[i,0,0]), int(self.track_center_pos_2d[i,0,1])), radius=5, color=(255, 255, 55), thickness=2)
        self.frame_out=imaxis
        cv2.imshow("Frame", imaxis)


