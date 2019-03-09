# -*- coding: utf-8 -*-
"""
Created on Sun Jul 05 15:01:58 2015

@author: ACSECKIN
"""
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
sys.path.append(ros_path)


import vrep
import numpy as np
from ransacPlane import ransacPlane

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


def convertXYZ(auxPackets):
    # Convert XYZ data into list of points with x,y,z data
    numXPoints = auxPackets[1].pop(0) # corresnponds to num of cols in image
    numYPoints = auxPackets[1].pop(0) # corresponds to num of rows in image
    numPoints = int(numXPoints * numYPoints)
    pointList = []
    for i in range(numPoints):
        xValue = auxPackets[1].pop(0)
        yValue = auxPackets[1].pop(0)
        zValue = auxPackets[1].pop(0)
        distance = auxPackets[1].pop(0)
        # to remove points where nothing was detected
        if distance >= 3.5:
           continue
        pointList.append([xValue, yValue, zValue])
    return pointList

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP


if clientID!=-1:
    print ('Connected to remote API server')
    
else:
    vrep.simxFinish(clientID)
    sys.exit("Failed connecting to remote API server")
    

print ('Vision Sensor object handling')
errorCode, rgbCamHandle = vrep.simxGetObjectHandle(clientID,
                                                   'kinect_depth', vrep.simx_opmode_oneshot_wait)
errorCode, xyzCamHandle = vrep.simxGetObjectHandle(clientID,
                                                   'kinect_pointcloud', vrep.simx_opmode_oneshot_wait)

print ('Setting up First Calls for Reading Images and Sensor Values')
# Note that the image takes a while to display so cannot load first image but
# have to set up a buffer
errorRGB, resolution, rgbImage = vrep.simxGetVisionSensorImage(
        clientID, rgbCamHandle,0, vrep.simx_opmode_streaming)

# api return value for the coordinate extraction filter in camera is as follows:
# (1) point count along x, (2) point count along Y, (3) point x1, (4) point y1,
# (5) point z1, (6) distance point1, (7) point x2, etc.

# auxPacket[1] holds xyz data in the format described above
errorXYZ, detectionState, auxPackets = vrep.simxReadVisionSensor(
        clientID, xyzCamHandle,vrep.simx_opmode_streaming)

# loop while connected to simulation
while (vrep.simxGetConnectionId(clientID) != -1):
    
    # Read in camera information (Note operating mode changed to buffer)
    errorRGB, resolution, rgbImage = vrep.simxGetVisionSensorImage(
            clientID, rgbCamHandle, 0, vrep.simx_opmode_buffer)
    errorXYZ, detectionState, auxPackets = vrep.simxReadVisionSensor(
        clientID, xyzCamHandle,vrep.simx_opmode_buffer)
    
    errorCodes = [errorRGB, errorXYZ]
    
    
    # Check if have both image and xyz coordinate data available
    if (all(errors == vrep.simx_return_ok for errors in errorCodes)):
        print ("image OK!!!")
        
        # resize image data into typical RGB image format
        img = np.array(rgbImage,dtype=np.uint8)
        img.resize([resolution[1],resolution[0],3])
        
        cv2.imshow('image',img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    elif (all(errors == vrep.simx_return_novalue_flag for errors in errorCodes)):
        print ("no image yet")
        pass
    else:
      print (errorCodes)
      
cv2.destroyAllWindows()

pointList = convertXYZ(auxPackets)
planeParameters = ransacPlane(pointList)

## plotting
pointList = np.asarray(pointList)
xVals = pointList[:,0]
yVals = pointList[:,1]
zVals = pointList[:,2]

# Transform to coordinates with z as vertical from ground plane
x1 = zVals
y1 = xVals
z1 = yVals

#zPlane = [0]*len(xVals)
#for i in range(len(xVals)):
#    zPlane[i] = (-planeParameters[0][0]*xVals[i]-planeParameters[0][1]*yVals[i] -\
#          planeParameters[1])/(planeParameters[0][2])
#
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#ax.scatter(xVals, yVals, zVals, c='r', marker='o') # original coordinates
ax.scatter(x1, y1, z1, c='r', marker='o') # new coordinates

#ax.scatter(xVals, yVals, zPlane, c='b', marker='v')
plt.show()


#mylab.points3d(x, y, z, value)
#mylab.show()













