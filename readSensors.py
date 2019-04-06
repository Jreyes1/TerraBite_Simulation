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



def getDepthImage(clientID):
    imageCount = 0 # to keep track of how many images have been processed
    processLimit = 5
    
#    print ('Vision Sensor object handling')
    errorCode, rgbCamHandle = vrep.simxGetObjectHandle(clientID,
                                                       'kinect_rgb', vrep.simx_opmode_oneshot_wait)
    errorCode, xyzCamHandle = vrep.simxGetObjectHandle(clientID,
                                                       'kinect_pointcloud', vrep.simx_opmode_oneshot_wait)
    
#    print ('Setting up First Calls for Reading Images and Sensor Values')
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
            # print ("image OK!!!")
            imageCount += 1
            
        elif (all(errors == vrep.simx_return_novalue_flag for errors in errorCodes)):
            # print ("no image yet")
            pass
        else:
           # print (errorCodes)
            pass
         
        # exit if processed more than certain amount of images
        if imageCount > processLimit:
            break
        
    return auxPackets,resolution,rgbImage

# Get Navigation Camera Orientation
def cameraOrientation(clientID):
    # Vision Sensor Object Handling
    errorCode, cameraHandle = vrep.simxGetObjectHandle(
            clientID,'kinect_depth', vrep.simx_opmode_oneshot_wait)
    
    # Setting Up First Call
    error, orientation = vrep.simxGetObjectOrientation(
            clientID,cameraHandle,-1,vrep.simx_opmode_streaming)
    
    # Loop while connected
    processLimit = 5
    count = 0
    while (vrep.simxGetConnectionId(clientID) != -1):
        
        error, orientation = vrep.simxGetObjectOrientation(
            clientID,cameraHandle,-1,vrep.simx_opmode_buffer) 
        if (error == vrep.simx_return_ok ):
            count +=1 
        
        if count > processLimit:
            break
        
    # Assume only pitch possible in camera
    pitch = math.degrees(orientation[1])-90
    return pitch


   
def ROI(image,show):
    # list of reference points
    global roiPts
    roiPts = []
      
    # copy original image
    clone = image.copy()
    
  
    cropping = False
    
    def click_and_crop(event,x,y,flags,param):
        # grab references to the global variables
        global roiPts, cropping
        
        # if the left mouse button was clicked, record the starting
        # coordinates and indicate that cropping is being performed
        if event == cv2.EVENT_LBUTTONDOWN:
            roiPts = [(x,y)]
            cropping = True
        
        # Check to see if left mouse button was released
        elif event == cv2.EVENT_LBUTTONUP:
            ## record the ending (x,y) coordinates and 
            # indicate that cropping is finished
            roiPts.append((x,y))
            cropping = False
            
            # draw a rectangle around the region of interest
            cv2.rectangle(image,roiPts[0],roiPts[1],(0,255,0),2)
            cv2.imshow("Select Farm Region",image)
    
    # set up callback function
    cv2.namedWindow("Select Farm Region")
    cv2.setMouseCallback("Select Farm Region",click_and_crop)
    
    # keep looping unti 'q' is pressed
    while True:
        # display the image and wait for key press
        cv2.imshow("Select Farm Region",image)
        key = cv2.waitKey(1) & 0xFF
        
        # if the 'r' key is pressed, reset the cropping region
        if key == ord("r"):
            image = clone.copy()
        
        # if the 'q' key is pressed, break from the loop
        elif key == ord("q"):
            break
        
    # if there are two reference points, then crop the region of interest
    # from the image and display it
#    print(len(roiPts))
    if len(roiPts) == 2:
        
        roi = clone[roiPts[0][1]:roiPts[1][1],roiPts[0][0]:roiPts[1][0]]
        if show:
            cv2.imshow("ROI",roi)
            cv2.waitKey(0)
        cv2.destroyAllWindows()
        return roi
    else:
        print("Not enough pts")
        cv2.destroyAllWindows()
        
        
    
def main():
#    getDroneImage(1,hei)
    position,orientation = gpsRobot()
    #orientation = 
    print("orientation relative to north:")
    print(math.degrees(orientation[2]))
#if __name__ == "__main__":
#    main()

