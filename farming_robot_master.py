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
from readSensors import gpsRobot,compassRobot,getDroneImage
from simple_pid import PID

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
#        print("clicked robot")
    # following double clicks mark target
    elif event == cv2.EVENT_LBUTTONDBLCLK and robotClicked:
        cv2.circle(droneImage,(x,y),5,(0,255,0),-1)
        mouseXTarget, mouseYTarget = x,y
        # Draw a diagonal blue line with thickness of 5 px
        cv2.line(droneImage,(mouseXRobot,mouseYRobot),
                 (mouseXTarget,mouseYTarget),(0,0,255),5)
#        print("clicked target")
#        
#        
        

clientID = checkConnectivity()      # Connect to V-Rep
droneHeight = 10                    # Set Drone Height
droneFOV = math.radians(85)         # degrees
resolution, droneImage = getDroneImage(clientID,droneHeight) # get drone image
droneImage = cv2.flip(droneImage,0) # flip image to match how its seen

# Moving Origin from top left to center of image
centerX = resolution[0]/2
centerY = resolution[1]/2

# Calcualte ratio between pixels to distance in image
pixDistRatio = resolution[0]/(2*droneHeight*math.tan(droneFOV/2))

# Display Picture and click to draw circles and extract coordinate
cv2.namedWindow('Drone Imagery')
robotClicked = False
cv2.setMouseCallback('Drone Imagery',draw_circle,[centerX,centerY])
while(1):
    cv2.imshow('Drone Imagery',droneImage)
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break

# Position of target attained using GPS
positionTarget = np.array([(mouseXTarget-centerX),
                           (centerY-mouseYTarget)])/pixDistRatio

# Calculate the angle from the robot to the target point    
def robot2Target(positionTarget,clientID):
    # Get position of robot
    robotPosition = gpsRobot(clientID)
    positionTarget = np.asarray(positionTarget)
  
    [deltaX, deltaY] = positionTarget-robotPosition
    angleTarget = math.atan2(deltaY,deltaX)*180/math.pi
    if angleTarget < 0:
        angleTarget += 360
    
    # Calculate Distance to Target
    dist2Target = math.sqrt((robotPosition[0]-positionTarget[0])**2+(robotPosition[1]-
            positionTarget[1])**2)
    
    return dist2Target, angleTarget

# Angle from heading angle of robot to target angle
def angleDifference(currentAngle,targetAngle):
    difference = targetAngle-currentAngle 
    angleDifference = ((difference + 180) % 360 - 180)
    return angleDifference

def powerMotors(omega, forwardVelocity,clientID):
    wheelBase = .267 # m
    wheelRadius = .19 #m
    leftMotorVel = (forwardVelocity-omega*wheelBase/2)/wheelRadius
    rightMotorVel = (forwardVelocity+omega*wheelBase/2)/wheelRadius
    # Get Motor Handles    
    leftErrorCode, leftMotorHandle = vrep.simxGetObjectHandle(
            clientID,"Pioneer_p3dx_leftMotor",vrep.simx_opmode_blocking )
    rightErrorCode, rightMotorHandle = vrep.simxGetObjectHandle(
            clientID,"Pioneer_p3dx_rightMotor",vrep.simx_opmode_blocking )

    # Set Motor Velocity
    vrep.simxSetJointTargetVelocity(
            clientID,leftMotorHandle,leftMotorVel,vrep.simx_opmode_streaming) 
    vrep.simxSetJointTargetVelocity(
            clientID,rightMotorHandle,rightMotorVel,vrep.simx_opmode_streaming)
    

#print("Angle to Target")
dist2Target, angleTarget = robot2Target(positionTarget, clientID)
robotAngle = compassRobot(clientID)
print("Robot Angle Offset")
difference = angleDifference(robotAngle, angleTarget)
print(difference)

tolDist = 1

pid = PID(1, 1, 1, setpoint=0)
pid.sample_time = 0.1
omega = math.radians(difference) 

omegaTrack = []
while(dist2Target > tolDist):
    omegaTrack.append(omega)
    control = pid(omega)    # apply PID to control input (omega)
    if (abs(omega) > math.radians(10)):
        forwardVelocity = 0
        print("turning")
    else:
        forwardVelocity = 1
        print("moving forward")
        print(omega)
        
    powerMotors(omega,forwardVelocity,clientID) # send control input to motors
    
    # Recalculate Values
    dist2Target, angleTarget = robot2Target(positionTarget, clientID)
    robotAngle = compassRobot(clientID)
    omega = math.radians(angleDifference(robotAngle, angleTarget))
    
    resolution, droneImage = getDroneImage(clientID,droneHeight) # get drone image
    droneImage = cv2.flip(droneImage,0) # flip image to match how its seen
    cv2.imshow('Drone Imagery',droneImage)
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break
    
    



input("Press Enter to continue...")
cv2.destroyAllWindows()




    
