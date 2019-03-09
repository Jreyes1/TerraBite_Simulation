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



print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP


if clientID!=-1:
    print ('Connected to remote API server')
    
else:
    vrep.simxFinish(clientID)
    sys.exit("Failed connecting to remote API server")
    
def poseRobot():
    print('Getting Robot GPS Location')
    
    # Get Robot GPS Handle
    errorCode, gpsHandle = vrep.simxGetObjectHandle(clientID,
                                                       'GPS', vrep.simx_opmode_oneshot_wait)
    # Get GPS location
    positionError, position = vrep.simxGetObjectPosition(
            clientID,gpsHandle,-1,vrep.simx_opmode_streaming)
    orientationError, orientation = vrep.simxGetObjectOrientation(
            clientID,gpsHandle,-1,vrep.simx_opmode_streaming)
    
    # loop while connected to simulation
    while (vrep.simxGetConnectionId(clientID) != -1):
        
        # Read in camera information (Note operating mode changed to buffer)
        positionError, position = vrep.simxGetObjectPosition(
            clientID,gpsHandle,-1,vrep.simx_opmode_buffer)
        orientationError, orientation = vrep.simxGetObjectOrientation(
            clientID,gpsHandle,-1,vrep.simx_opmode_buffer)  
        orientation = math.degrees(orientation[2])
        if orientation < 0:
            orientation += 360
        errorCodes = [positionError, orientationError]
        
        
        # Check if have both image and xyz coordinate data available
        if (all(errors == vrep.simx_return_ok for errors in errorCodes)):
            print ("Pose OK!!!")
            break
        elif (all(errors == vrep.simx_return_novalue_flag for errors in errorCodes)):
            print ("Pose Not Available Yet")
            pass
        else:
          print (errorCodes)
  
    return position,orientation
    
def main():
    position,orientation = poseRobot()
    #orientation = 
    print("orientation relative to north:")
    print(math.degrees(orientation[2]))
