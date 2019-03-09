#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Main Code
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
from readSensors import poseRobot


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
        if (errorDrone == vrep.simx_return_ok and count >= 5):
            droneImage = np.array(droneImage,dtype=np.uint8)
            droneImage.resize([resolution[1],resolution[0],3])
            break
    return resolution,droneImage

# mouse callback function for robot to draw circles at clicked points
def draw_circle(event,x,y,flags,param):
    global mouseXRobot, mouseYRobot
    global mouseXTarget, mouseYTarget
    global robotClicked, centerX, centerY
    
    # first left double click is to mark robot
    if event == cv2.EVENT_LBUTTONDBLCLK and not robotClicked:
        cv2.circle(droneImage,(x,y),5,(255,0,0),-1)
        mouseXRobot, mouseYRobot = x,y
        robotClicked = True
        print("clicked robot")
    # following double clicks mark target
    elif event == cv2.EVENT_LBUTTONDBLCLK and robotClicked:
        cv2.circle(droneImage,(x,y),5,(0,255,0),-1)
        mouseXTarget, mouseYTarget = x,y
        # Draw a diagonal blue line with thickness of 5 px
        cv2.line(droneImage,(mouseXRobot,mouseYRobot),
                 (mouseXTarget,mouseYTarget),(0,0,255),5)
        print("clicked target")
#        positionRobot = np.array([[mouseXRobot],
#                          [mouseYRobot]])
#        positionTarget = np.array([[mouseXTarget],
#                           [mouseYTarget]])
#        [deltaX, deltaY] = (positionTarget-positionRobot)
#
#        angle = -math.atan2(deltaY,deltaX)*180/math.pi
#        if angle < 0:
#            angle += 360
#        print("angle: " + str(angle))
#        
        

clientID = checkConnectivity()
droneHeight = 10                    # Set Drone Height
droneFOV = math.radians(85)         # degrees
resolution, droneImage = getDroneImage(clientID,droneHeight) # get drone image
droneImage = cv2.flip(droneImage,0) # flip image to match how its seen

# Moving Origin from top left to center of image
centerX = resolution[0]/2
centerY = resolution[1]/2

# Calcualte ratio between pixels to distance in image
pixDistRatio = resolution[0]/(2*droneHeight*math.tan(droneFOV/2))

cv2.namedWindow('Drone Imagery')
robotClicked = False
cv2.setMouseCallback('Drone Imagery',draw_circle,[centerX,centerY])
while(1):
    cv2.imshow('Drone Imagery',droneImage)
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break

# Convert Image Coordinates to Real World Distance Coordinates
positionRobot = np.array([[mouseXRobot-centerX],
                          [centerY-mouseYRobot]])/pixDistRatio
positionTarget = np.array([[mouseXTarget-centerX],
                           [centerY-mouseYTarget]])/pixDistRatio
# Calculate Angle to Target (Using positive angles only)
[deltaX, deltaY] = positionTarget-positionRobot
angleTarget = math.atan2(deltaY,deltaX)*180/math.pi
if angleTarget < 0:
    angleTarget += 360


# Get Robot Angle and Angle from robot to target
position, robotAngle = poseRobot() # Note real robot position would have GPS noise


print("Robot and Target Position")
print(positionRobot) # Calculated using camera, not position above
print(positionTarget)

print("Robot azimuth angle")
print(robotAngle)

print("Target Angle: " + str(angleTarget))


print("Robot Angle Offset")
difference = angleTarget-robotAngle 
print((difference + 180) % 360 - 180)


input("Press Enter to continue...")
cv2.destroyAllWindows()




    
