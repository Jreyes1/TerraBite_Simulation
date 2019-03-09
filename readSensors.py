#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar  3 14:48:33 2019

@author: jorge
"""

import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
sys.path.append(ros_path)


import vrep
import numpy as np
import math

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
    
def gpsRobot(clientID):
#    print('Getting Robot GPS Location')
    
    # Get Robot GPS Handle
    errorCode, gpsHandle = vrep.simxGetObjectHandle(clientID,
                                                       'GPS', vrep.simx_opmode_oneshot_wait)
    # Get GPS location
    positionError, position = vrep.simxGetObjectPosition(
            clientID,gpsHandle,-1,vrep.simx_opmode_streaming)
    
    # loop while connected to simulation
    while (vrep.simxGetConnectionId(clientID) != -1):
        
        # Read in camera information (Note operating mode changed to buffer)
        positionError, position = vrep.simxGetObjectPosition(
            clientID,gpsHandle,-1,vrep.simx_opmode_buffer)
        
        errorCodes = [positionError]
        # Check if have both image and xyz coordinate data available
        if (all(errors == vrep.simx_return_ok for errors in errorCodes)):
#            print ("Pose OK!!!")
            break
        elif (all(errors == vrep.simx_return_novalue_flag for errors in errorCodes)):
#            print ("Pose Not Available Yet")
            pass
        else:
          print (errorCodes)
    
    position = np.asarray(position)
#    print("position")
#    print(position)
    position = position[:2]
#    print(position)
    return position

def compassRobot(clientID):
#    print('Getting Robot GPS Location')
    
    # Get Robot GPS Handle
    errorCode, gpsHandle = vrep.simxGetObjectHandle(clientID,
                                                       'GPS', vrep.simx_opmode_oneshot_wait)
   
    orientationError, orientation = vrep.simxGetObjectOrientation(
            clientID,gpsHandle,-1,vrep.simx_opmode_streaming)
    
    # loop while connected to simulation
    while (vrep.simxGetConnectionId(clientID) != -1):
        
        # Read in camera information (Note operating mode changed to buffer)
        orientationError, orientation = vrep.simxGetObjectOrientation(
            clientID,gpsHandle,-1,vrep.simx_opmode_buffer)  
        orientation = math.degrees(orientation[2])
        if orientation < 0:
            orientation += 360
        errorCodes = [ orientationError]
        
        
        # Check if have both image and xyz coordinate data available
        if (all(errors == vrep.simx_return_ok for errors in errorCodes)):
#            print ("Pose OK!!!")
            break
        elif (all(errors == vrep.simx_return_novalue_flag for errors in errorCodes)):
#            print ("Pose Not Available Yet")
            pass
        else:
          print (errorCodes)
          
    return orientation

# Get Drone Imagery
def getDroneImage(clientID, height):
    position = np.asarray([0,0,height])
    
    # Get Drone Handle
    errorCode, droneHandle = vrep.simxGetObjectHandle(
            clientID,'drone', vrep.simx_opmode_oneshot_wait)
    
    # Set Drone Height (in meters)
    vrep.simxSetObjectPosition(clientID,
                               droneHandle,-1,position,vrep.simx_opmode_oneshot_wait)

    # Get Drone Camera Image (First Call)
    errorDrone, resolution, droneImage = vrep.simxGetVisionSensorImage(
        clientID, droneHandle,0, vrep.simx_opmode_streaming)
    count = 0
    while (vrep.simxGetConnectionId(clientID) != -1):
    
        # Read in drone camera image (Note operating mode changed to buffer)
        errorDrone, resolution, droneImage = vrep.simxGetVisionSensorImage(
                clientID, droneHandle, 0, vrep.simx_opmode_buffer)
        count+=1
        
        # quit after 5 images read (first images may be empty)
        if (errorDrone == vrep.simx_return_ok and count >= 1):
            droneImage = np.array(droneImage,dtype=np.uint8)
            droneImage.resize([resolution[1],resolution[0],3])
            break
    return resolution,droneImage
    
def main():
    position,orientation = gpsRobot()
    #orientation = 
    print("orientation relative to north:")
    print(math.degrees(orientation[2]))
