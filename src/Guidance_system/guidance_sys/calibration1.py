
import numpy as np
import cv2, os
from cv2 import aruco

import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd

from picamera.array import PiRGBArray
from picamera import PiCamera

import time

import pid as PID
#%matplotlib nbagg
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
board = aruco.CharucoBoard_create(7, 5, 1, .8, aruco_dict)

def read_chessboards(images):
    """
    Charuco base pose estimation.
    """
    print("POSE ESTIMATION STARTS:")
    allCorners = []
    allIds = []
    decimator = 0
    # SUB PIXEL CORNER DETECTION CRITERION
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)

    for im in images:
        print("=> Processing image {0}".format(im))
        frame = cv2.imread(im)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)

        if len(corners)>0:
            # SUB PIXEL DETECTION
            for corner in corners:
                cv2.cornerSubPix(gray, corner,
                                 winSize = (3,3),
                                 zeroZone = (-1,-1),
                                 criteria = criteria)
            res2 = cv2.aruco.interpolateCornersCharuco(corners,ids,gray,board)
            if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%1==0:
                allCorners.append(res2[1])
                allIds.append(res2[2])

        decimator+=1

    imsize = gray.shape
    return allCorners,allIds,imsize

def calibrate_camera(allCorners,allIds,imsize):
    """
    Calibrates the camera using the dected corners.
    """
    print("CAMERA CALIBRATION")

    cameraMatrixInit = np.array([[ 1000.,    0., imsize[0]/2.],
                                 [    0., 1000., imsize[1]/2.],
                                 [    0.,    0.,           1.]])

    distCoeffsInit = np.zeros((5,1))
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
    #flags = (cv2.CALIB_RATIONAL_MODEL)
    (ret, camera_matrix, distortion_coefficients0,
     rotation_vectors, translation_vectors,
     stdDeviationsIntrinsics, stdDeviationsExtrinsics,
     perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
                      charucoCorners=allCorners,
                      charucoIds=allIds,
                      board=board,
                      imageSize=imsize,
                      cameraMatrix=cameraMatrixInit,
                      distCoeffs=distCoeffsInit,
                      flags=flags,
                      criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))

    return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors



if __name__ == '__main__':
    #import image
    datadir = "photo/"
    images = np.array([datadir + f for f in os.listdir(datadir) if f.endswith(".png") ])
    order = np.argsort([int(p.split(".")[-2].split("_")[-1]) for p in images])
    images = images[order]
    print(images)
    #image calibration
    allCorners,allIds,imsize=read_chessboards(images)
    ret, mtx, dist, rvecs, tvecs = calibrate_camera(allCorners,allIds,imsize)
    '''
    for i in range(5):
        #i=1 # select image id
        plt.figure()
        frame = cv2.imread(images[i])
        img_undist = cv2.undistort(frame,mtx,dist,None)
        plt.subplot(1,2,1)
        plt.imshow(frame)
        plt.title("Raw image")
        plt.axis("off")
        plt.subplot(1,2,2)
        plt.imshow(img_undist)
        plt.title("Corrected image")
        plt.axis("off")
        plt.show()
    '''
    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))
    # allow the camera to warmup
    time.sleep(0.1)
    # capture frames from the camera
    print("Start")
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        start= time.time()
        image = frame.array
        #image coretion to save sorouce can close
        #image = cv2.undistort(image,mtx,dist,None)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #blur
        #blur = cv2.GaussianBlur(gray, (5, 5), 0)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters =  aruco.DetectorParameters_create()
        #parameters.adaptiveThreshWinSizeMax =23
        #print("adaptiveThreshWinSizeMin:",parameters.adaptiveThreshWinSizeMin," adaptiveThreshWinSizeMax:",parameters.adaptiveThreshWinSizeMax," adaptiveThreshWinSizeStep:",parameters.adaptiveThreshWinSizeStep)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # SUB PIXEL DETECTION improve cornor accurate
        corners_corection =None
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
        for corner in corners:
            cv2.cornerSubPix(gray, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)




        #frame_markers = image.copy()
        #frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)

        #estimate and caculate distance
        size_of_marker =  0.05 # side lenght of the marker in meter
        rvecs,tvecs,_objPoints = aruco.estimatePoseSingleMarkers(corners, size_of_marker , mtx, dist)

        #distance caculate and sort
        if tvecs is not None:
            #draw Aix into image
            distance=[]
            sort_index=[]
            for i in range(len(tvecs)):
                sum =0
                for p in range(3):
                    sum+= (tvecs[i][0][p]*180/np.pi)**2
                sum =  round(np.sqrt(sum),3)
                distance.append(sum)
                sort_index.append(i)
            #sort_index=sorted(range(len(distance)), key=lambda k: distance[k])
            #sort_index=np.argsort(distance)

            #sort index
            sort_dis = sorted(distance)
            i=0
            ii=0
            for i in range(len(tvecs)):
                for ii in range(len(tvecs)):
                    if distance[i] == sort_dis[ii]:
                        sort_index[i]=ii

            #print("sort index:",sort_index,"distance",distance)
            index=0
            counter=0
            pos_1=0
            pos_2=0
            pos_3=0
            pos_4=0
            track_index=[]
            for id in sort_index:
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
                index += 1
            if counter==4:
                track_index=[pos_1,pos_2,pos_3,pos_4]
            elif counter==3:
                track_index=[pos_1,pos_2,pos_3]
            elif counter==2:
                track_index=[pos_1,pos_2]
            elif counter==1:
                track_index=[pos_1]
            #print(track_index)

            track = False
            if counter%2 ==0 or counter >=2:
                track =True
                track_side_pos=np.zeros((len(track_index),1,2))
                track_center_pos_2d=np.zeros((int(len(track_index)/2),1,2))

                track_center_pos_3d=np.zeros((int(len(track_index)/2),1,3))

                for i in range(len(track_index)):
                    track_side_pos[i,0,0] = int(abs(corners[track_index[i]][0][0][0]+corners[track_index[i]][0][2][0])/2)
                    track_side_pos[i,0,1] =int(abs(corners[track_index[i]][0][0][1]+corners[track_index[i]][0][2][1])/2)

                track_side_pos_t=track_side_pos.reshape(int(len(track_index)/2),2,2) #reshape into pair(n,2,2)
                track_index_t=np.reshape(track_index, (int(len(track_index)/2),2,1))

                #track_index.reshape(int(len(track_index)/2),2,1)
                distance_track=[]
                for i in range(len(track_side_pos_t)):
                    track_center_pos_2d[i,0,0]=int((track_side_pos_t[i][1][0]+track_side_pos_t[i][0][0])/2)
                    track_center_pos_2d[i,0,1]=int((track_side_pos_t[i][1][1]+track_side_pos_t[i][0][1])/2)

                    for p in range(3):
                        #print(track_index_t)
                        track_center_pos_3d[i,0,p]=int((tvecs[track_index_t[i,1],0,p]*180/np.pi+tvecs[track_index_t[i,0],0,p]*180/np.pi)/2.0)
                    sum =0
                    for p in range(3):
                        sum+= track_center_pos_3d[i,0,p]**2
                    sum =  round(np.sqrt(sum),2)
                    distance_track.append(sum)

                #track_transpose.reshape(len(track_index)/2,2,2)
                #(corners[index][0][1][0]-corners[index][0][0][0])/2,(corners[index][0][1][1]-corners[index][0][2][1])/2
                #int(abs(corners[track_index[0]][0][0][0]+corners[track_index[0]][0][2][0])/2),int(abs(corners[track_index[0]][0][0][1]+corners[track_index[0]][0][2][1])/2)
                tvecs[i][0][p]*180/np.pi
                print("Center",track_center_pos_2d,"side",track_side_pos,"dis",distance_track)



        #draw frist two marker and distance
        imaxis = image.copy()
        length_of_axis = 0.04
        if ids is not None:
            #draw the markers and distance
            index=0
            #print(corners[index][0][:],ids[index])q
            #imaxis = aruco.drawDetectedMarkers(imaxis.copy(), corners, ids)
            font = cv2.FONT_HERSHEY_SIMPLEX
            line = cv2.LINE_AA
            for id in sort_index:
                #org=(corners[index][0][2][0],corners[index][0][2][1])
                if id<4:
                    imaxis=cv2.putText(imaxis, text=str(distance[index]),
                                   org=(corners[index][0][1][0],corners[index][0][1][1]),
                                   fontFace=font, fontScale=0.4, color=(255,154,36),thickness=1, lineType=line)
                    imaxis = aruco.drawAxis(imaxis, mtx, dist, rvecs[index], tvecs[index], length_of_axis)
                #print ("ID:",ids[index]," rotation:",rvecs[index]*180/np.pi,"Transpose:",tvecs[index]*180/np.pi,"distance: ", distance[index])
                #print(int((corners[index][0][1][0]-corners[index][0][0][0])/2),int((corners[index][0][1][1]-corners[index][0][2][1])/2))
                index+=1
            if track == True:
                for i in range(len(track_side_pos)):
                    imaxis=cv2.circle(imaxis,center=(int(track_side_pos[i,0,0]), int(track_side_pos[i,0,1])), radius=3, color=(255, 0, 255), thickness=1)

                for i in range(len(track_center_pos_2d)):
                    imaxis=cv2.putText(imaxis, text=str(distance_track[i]),
                                   org=(int(track_center_pos_2d[i,0,0]), int(track_center_pos_2d[i,0,1])),
                                   fontFace=font, fontScale=0.4, color=(255,154,36),thickness=1, lineType=line)
                    imaxis=cv2.circle(imaxis,center=(int(track_center_pos_2d[i,0,0]), int(track_center_pos_2d[i,0,1])), radius=5, color=(255, 255, 55), thickness=2)
        '''
#finds marker id 47
        if ids is not None:
            index=0
            for id in ids:

                if id == 47:
                    #print(corners[index][0][:],ids[index])
                    #frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
                    imaxis = aruco.drawAxis(frame_markers, mtx, dist, rvecs[index], tvecs[index], length_of_axis)
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    line = cv2.LINE_AA


                    imaxis=cv2.putText(imaxis, text=str(int(distance[index])), org=(corners[index][0][0][0],corners[index][0][0][1]),fontFace=font, fontScale=0.8, color=(0, 0, 255),thickness=1, lineType=line)
                    #print ("ID:47 rotation:",rvecs[index]*180/np.pi,"Transpose:",tvecs[index]*180/np.pi,"distance: ", distance[index])
                    break
                index+=1
        '''

        # show the frame
        #cv2.imshow("Frame", frame_markers)
        cv2.imshow("Frame", imaxis)
        key = cv2.waitKey(1) & 0xFF
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
             break

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        #num detact
        num=0
        if ids is not None:
            for i in range(len(ids)):
                num=num+1

        #fps

        time_used=time.time() -start


        #fps limit
        fps=1.0/time_used
        if fps>30.0:
            sleeptime=1.0/30-(time.time() -start)
            if sleeptime >0:
                time.sleep(sleeptime)

        time_used=time.time() -start


        fps=1.0/time_used

        #print("FPS:",fps,"time(ms)",time_used*1000,"Num detact: ",num)


