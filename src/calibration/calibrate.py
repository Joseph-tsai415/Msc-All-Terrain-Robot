import numpy as np
import cv2, os
from cv2 import aruco


class calibrate():
    """
    The class called Calibrate is initialised with constants appropriate
    for the given target Calibration
    """

    # if __name__ == '__main__':
    def __init__(self):
        #%matplotlib nbagg
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.board = aruco.CharucoBoard_create(7, 5, 1, .8, self.aruco_dict)
        #import image
        dir_path=os.path.dirname(os.path.realpath(__file__))
        datadir = os.path.join(dir_path,"./photo/")
        images = np.array([datadir + f for f in os.listdir(datadir) if f.endswith(".png") ])
        order = np.argsort([int(p.split(".")[-2].split("_")[-1]) for p in images])
        images = images[order]
        print(images)
        #image calibration
        allCorners,allIds,imsize=self.read_chessboards(images)
        self.ret, self.mtx, self.dist, self.rvecs, self.tvecs = self.calibrate_camera(allCorners,allIds,imsize)

    def get(self):
        '''
        Return the fix camera matrix
        '''
        return self.ret, self.mtx, self.dist, self.rvecs, self.tvecs

    def read_chessboards(self,images):
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
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, self.aruco_dict)

            if len(corners)>0:
                # SUB PIXEL DETECTION
                for corner in corners:
                    cv2.cornerSubPix(gray, corner,
                                    winSize = (3,3),
                                    zeroZone = (-1,-1),
                                    criteria = criteria)
                res2 = cv2.aruco.interpolateCornersCharuco(corners,ids,gray,self.board)
                if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%1==0:
                    allCorners.append(res2[1])
                    allIds.append(res2[2])

            decimator+=1

        imsize = gray.shape
        return allCorners,allIds,imsize

    def calibrate_camera(self,allCorners,allIds,imsize):
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
                        board=self.board,
                        imageSize=imsize,
                        cameraMatrix=cameraMatrixInit,
                        distCoeffs=distCoeffsInit,
                        flags=flags,
                        criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))

        return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors


if __name__ == '__main__':

    calbrate = calibrate()
    ret, mtx, dist, rvecs, tvecs = calbrate.get()