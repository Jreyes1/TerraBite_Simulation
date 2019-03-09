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
from multiprocessing import Process

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
def draw_circle(event,x,y,flags,droneImage):
    global mouseXTargetList, mouseYTargetList
#    global mouseXTarget, mouseYTarget
#    global robotClicked, centerX, centerY
    
    # first left double click is to draw line to robot
    if event == cv2.EVENT_LBUTTONDBLCLK:
        # Draw circle at clicked point
        cv2.circle(droneImage,(x,y),5,(255,0,0),-1)
        targetX, targetY = x,y
        
        # Add point to point list
        mouseXTargetList.append(targetX)
        mouseYTargetList.append(targetY)
        
        # Only draw line between target and previous point if more than 1 point
        numTargets = len(mouseXTargetList)
        if(numTargets>1):
            cv2.line(droneImage,(targetX,targetY),
                 (mouseXTargetList[numTargets-2],mouseYTargetList[numTargets-2]),
                 (0,0,255),5)
            
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

#    while(1):
#        print("Video Stream")
#         # Show video Stream
#        print(clientID,droneHeight)
#        resolution, droneImage = getDroneImage(clientID,droneHeight) # get drone image
#        print("passed")
#        droneImage = cv2.flip(droneImage,0) # flip image to match how its seen
#        cv2.imshow('Drone Imagery',droneImage)
#    #        for i in range(numTargets):
#    #            cv2.circle(droneImage,(mouseXTargetList[i],mouseYTargetList[i]),5,(255,0,0),-1)
#        if cv2.waitKey(20) & 0xFF == ord('q'):  
#            break
#    

            
def main():
    clientID = checkConnectivity()      # Connect to V-Rep
    droneHeight = 10                  # Set Drone Height
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
    
    # List of points with 
    global mouseXTargetList, mouseYTargetList
    mouseXTargetList = []
    mouseYTargetList = []
    
    # Open window to select target locations
    cv2.setMouseCallback('Drone Imagery',draw_circle,droneImage)
    while(1):
        cv2.imshow('Drone Imagery',droneImage)
        if cv2.waitKey(20) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
        
    # Check how many targets were selected
    numTargets = len(mouseXTargetList)
    if (numTargets==0):
        cv2.destroyAllWindows()
        sys.exit("No Target Points Selected")
        
    # Position of target attained using GPS
    targetIndex = 0
    currentTargetX = mouseXTargetList[targetIndex]
    currentTargetY = mouseYTargetList[targetIndex]    
    positionTarget = np.asarray([currentTargetX-centerX,
                                 centerY-currentTargetY])/pixDistRatio
    
    
    #print("Angle to Target")
    dist2Target, angleTarget = robot2Target(positionTarget, clientID)
    robotAngle = compassRobot(clientID)
    #print("Robot Angle Offset")
    difference = angleDifference(robotAngle, angleTarget)
    #print(difference)
    
    # Target point reached when robot within this distance 
    tolDist = .2
    
    pid = PID(5, 0, 1, setpoint=0)
#    pid.sample_time = 0.05
    omega = math.radians(difference) 
    
    # Drone Feed on seperate parallel process
    #droneFeed = threading.Thread(target=droneVideoStream,args=(clientID,droneHeight))
    #droneFeed.start()
    #while(dist2Target > tolDist):
    while(targetIndex < numTargets):
        
           # apply PID to control input (omega)
        control = pid(-omega)    
        print("Omega:" + str(omega))
        print("control: " + str(control))
        
        # Spin in place if angle difference is large
        if (abs(control) > math.radians(150)):
            forwardVelocity = 0
    #        print("turning")
        else:
            forwardVelocity = 1
    
        # send control input to motors
        powerMotors(control,forwardVelocity,clientID) 
        
        # Find Position of current target
        currentTargetX = mouseXTargetList[targetIndex]
        currentTargetY = mouseYTargetList[targetIndex]    
        positionTarget = np.asarray([currentTargetX-centerX,
                                 centerY-currentTargetY])/pixDistRatio
        # Recalculate Values
        dist2Target, angleTarget = robot2Target(positionTarget, clientID)
        robotAngle = compassRobot(clientID)
        omega = math.radians(angleDifference(robotAngle, angleTarget))
        
        
    #    # Show video Stream
    #    resolution, droneImage = getDroneImage(clientID,droneHeight) # get drone image
    #    droneImage = cv2.flip(droneImage,0) # flip image to match how its seen
    #    cv2.imshow('Drone Imagery',droneImage)
    #    if cv2.waitKey(20) & 0xFF == ord('q'):  
    #        break
        
         # If reached target point, move to next target point
        if (dist2Target < tolDist):
            targetIndex += 1
        
        # Display Current Target
        print("Moving to Target: " + str(targetIndex))
            
    # If arrived to last checkpoint, power off motors
    powerMotors(0,0,clientID) 
    print("Arrived to Last Checkpoint: " + str(targetIndex))
    #droneFeed.join()
    print("finished")
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main()




    
