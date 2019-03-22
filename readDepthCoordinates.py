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

# Connect to V-REP Server
def checkConnectivity():
    # Check Connectivity
    print ('Program started')
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
    
    if clientID!=-1:
        print ('Connected to remote API server')
    else:
        vrep.simxFinish(clientID)
        sys.exit("Failed connecting to remote API server")
    return clientID


def convertXYZ(auxPackets):
    # Convert XYZ data into list of points with x,y,z data
    numXPoints = auxPackets[1].pop(0) # corresnponds to num of cols in image
    numYPoints = auxPackets[1].pop(0) # corresponds to num of rows in image
    numPoints = int(numXPoints * numYPoints)
    pointList = []
    for i in range(numPoints):
        xValue = auxPackets[1].pop(0)
        zValue = auxPackets[1].pop(0) # Note switching y and z
        yValue = auxPackets[1].pop(0)
        distance = auxPackets[1].pop(0)
        # to remove points where nothing was detected
        if distance >= 3.5:
        	continue
        
        pointList.append([xValue, yValue, zValue])
        # distances.append(distance)
    return pointList

def main():
    clientID = checkConnectivity()      # Connect to V-Rep

    
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
    # and auxPacket[1] holds  data in the format described above
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
    
    # read point cloud data as an array
    pointList = convertXYZ(auxPackets)
    pointList = np.asarray(pointList)
    
    # extract coordinates
    xVals = np.round(pointList[:,0],1)
    yVals = np.round(pointList[:,1],2) # switch z and y
    zVals = np.round(pointList[:,2],2)
    
   
    # perform ransac on point cloud to predict ground plane equation
    planeParameters = ransacPlane(pointList)
    
    # Use ground plane equation to overlay points over original points
    zPlane = [0]*len(xVals)
    print("length of zPlane: " + str(len(zPlane)))
    for i in range(len(xVals)):
        # Note B and C are switched
        zPlane[i] = (-planeParameters[0][0]*xVals[i]-planeParameters[0][1]*
              yVals[i] -planeParameters[1])/(planeParameters[0][2])

    # Plotting
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(xVals, yVals, zVals, c='r', marker='o') # original coordinates
    ax.scatter(xVals, yVals, zPlane, c='b', marker='v') # Calculated Ground Plane
    
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title('Ground Plane Detection')
    
    plt.show()
    
        
if __name__ == '__main__':
    main()












