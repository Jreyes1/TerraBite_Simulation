#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This program contains the set of functions to drive the robot
and PID controll algorithms
"""

import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
sys.path.append(ros_path)
import vrep
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

def powerMotors(leftMotorVel, rightMotorVel):
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
#def rotate2Angle():
#    # do stuff
    
global clientID
clientID = checkConnectivity()
powerMotors(0,0)

