import socket
import threading
import time
from time import sleep
import sys
import numpy as np
from queue import SimpleQueue
from queue import LifoQueue

import os
import numpy as np
import cv2
import cv2.aruco as aruco

# Save data #################################################################
aruco_data_file_name = 'arucoData.txt'
index = 0
ids = 0
tvec = [[[0, 0, 0]]]
rvec = [[[0, 0, 0]]]   # Control input for left/right
INTERVAL = 0.05  # update rate for state information
start_time = time.time()
dataQ = SimpleQueue()
stateQ = LifoQueue() # have top element available for reading present state by control loop

def writeFileHeader(dataFileName):
    fileout = open(dataFileName,'w')
    #write out parameters in format which can be imported to Excel
    today = time.localtime()
    date = str(today.tm_year)+'/'+str(today.tm_mon)+'/'+str(today.tm_mday)+'  '
    date = date + str(today.tm_hour) +':' + str(today.tm_min)+':'+str(today.tm_sec)
    fileout.write('"Data file recorded ' + date + '"\n')
    # header information
    fileout.write('  time,  ids,   tvec_x,    tvec_y,    tvec_z,    rvec_1,   rvec_2,    rvec_3\n\r')
    #fileout.write('  index,   time,   ids\n\r')

    fileout.close()

def writeDataFile(dataQ):
    fileout = open(aruco_data_file_name, 'w')  # append
    header = 'time,  ids,   tvec_x,    tvec_y,    tvec_z,    rvec_1,   rvec_2,    rvec_3'
    if fileout:

        print('writing data to file')
        telemdata = np.array(dataQ)#dataQ.pop()
        print(telemdata)
        print(type(telemdata))
        print(telemdata.shape)
        #np.savetxt(fileout, telemdata, fmt='%7.3f', delimiter = ',', newline = '\n')  # need to make telemdata a list
        np.savetxt(fileout, telemdata, fmt='%7.3f', delimiter = ',', header = header)
        #np.savetxt(fileout , [telemdata],fmt='%7.3f', delimiter = ',')
        fileout.close()
    else:
        print('file not open')

def report(index):
    telemdata=[]
    telemdata.append(index)
    telemdata.append(time.time()-start_time)
    telemdata.append(ids)
    telemdata.append(tvec[0][0][0])
    telemdata.append(tvec[0][0][1])
    telemdata.append(tvec[0][0][2])
    telemdata.append(rvec[0][0][0])
    telemdata.append(rvec[0][0][1])
    telemdata.append(rvec[0][0][2])
    dataQ.put(telemdata)
    stateQ.put(telemdata)
    if (index %100) == 0:
        print(index, end=',')

#writeFileHeader(aruco_data_file_name)

# Calibrate ##################################################################
def calibrate():

    cap= cv2.VideoCapture("udp://@0.0.0.0:11111")
    # "handheld_tello_vid.mp4"
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    # checkerboard of size (9 x 7) is used
    objp = np.zeros((7*9,3), np.float32)
    objp[:,:2] = np.mgrid[0:9,0:7].T.reshape(-1,2)

    # arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        # resizing for faster detection
        frame = cv2.resize(frame, (640, 480))
        # using a greyscale picture, also for faster detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, (9,7), None)

        # If found, add object points, image points
        if ret == True:
            objpoints.append(objp)
            imgpoints.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(frame, (9,7), corners, ret)
            #write_name = 'corners_found'+str(idx)+'.jpg'

        # Display the resulting frame
        cv2.imshow('Calibration',frame)
        if cv2.waitKey(100) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    cv2.waitKey(10)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

    #create a file to store data
    from lxml import etree
    from lxml.builder import E

    global fname
    with open(fname, "w") as f:
        f.write("{'ret':"+str(ret)+", 'mtx':"+str(list(mtx))+', "dist":'+str(list(dist))+'}')
        f.close()


#test whether already calibrated or not
path = os.path.abspath('..')
fname = path + "/res/calibration_parameters.txt"
print(fname)
try:
    f = open(fname, "r")
    f.read()
    f.close()
except:
    calibrate()

# Find aruco tags ############################################################
cap = cv2.VideoCapture("udp://@0.0.0.0:11111")

#importing aruco dictionary
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

#calibration parameters
f = open(fname, "r")
ff = [i for i in f.readlines()]
f.close()
from numpy import array
parameters = eval(''.join(ff))
mtx = array(parameters['mtx'])
dist = array(parameters['dist'])

# Create absolute path from this module
file_abspath = os.path.join(os.path.dirname(__file__), 'Samples/box.obj')

tvec = [[[0, 0, 0]]]
rvec = [[[0, 0, 0]]]

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250 )
markerLength = 0.25   # Here, our measurement unit is centimetre.
parameters = cv2.aruco.DetectorParameters_create()
parameters.adaptiveThreshConstant = 10
telemdata = np.empty((8,))

telemdataQ = []

while True:

    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    font = cv2.FONT_HERSHEY_SIMPLEX
    telemdata[0] = time.time() - start_time
    if np.all(ids != None):
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
        #print(ids)
        #print(corners)
        #print(rvec)

        aruco.drawAxis(frame, mtx, dist, rvec[0], tvec[0], 0.1)
        # show translation vector on the corner
        font = cv2.FONT_HERSHEY_SIMPLEX
        text = str([round(i,5) for i in tvec[0][0]])
        position = tuple(corners[0][0][0])
        cv2.putText(frame, text, position, font, 0.4, (0, 0, 0), 1, cv2.LINE_AA)
        #get tvec, rvec of each id
        #print(ids[i], tvec[i][0], rvec[i][0])
        telemdata[1] = ids[0]
        telemdata[2:5] = tvec[0][0]
        telemdata[5:] = rvec[0][0]

        #print(telemdata)
        #dataQ.put(telemdata)

        aruco.drawDetectedMarkers(frame, corners)
    else:
        telemdata[1:] = np.NaN

    telemdataQ.append(np.copy(telemdata))
    cv2.imshow('frame',frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        #print(telemdataQ)
        writeDataFile(telemdataQ)
        break

# writeDataFile(aruco_data_file_name)
cap.release()
cv2.destroyAllWindows()
