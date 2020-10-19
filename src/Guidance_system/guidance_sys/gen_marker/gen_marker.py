import cv2 as cv
import numpy as np

# Load the predefined dictionary

dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
# Generate the marker

markerImage = np.zeros((400, 400), dtype=np.uint8)
for i in range(50):
    markerImage = cv.aruco.drawMarker(dictionary, i, 400, markerImage, 1);
    cv.imwrite("marker4X4_%i.png"%+(i+1), markerImage);

