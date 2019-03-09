# Make sure to have the server side running in V-REP: 
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

import sys

try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP

if clientID!=-1:
    print ('Connected to remote API server')
    
else:
    vrep.simxFinish(clientID)
    sys.exit("Failed connecting to remote API server")
    
# Get Motor Handles    
errorCode, leftMotorHandle = vrep.simxGetObjectHandle(clientID,"Pioneer_p3dx_leftMotor",vrep.simx_opmode_blocking )
errorCode, rightMotorHandle = vrep.simxGetObjectHandle(clientID,"Pioneer_p3dx_rightMotor",vrep.simx_opmode_blocking )

# Set Motor Velocity
vrep.simxSetJointTargetVelocity(clientID,leftMotorHandle,1,vrep.simx_opmode_streaming) 

# Get camera Handle
errorCode, cameraHandle = vrep.simxGetObjectHandle(clientID,"cam1",vrep.simx_opmode_blocking )

# Display images
errorCode, resolution, image = vrep.simxGetVisionSensorImage(clientID, cameraHandle, 0,vrep.simx_opmode_buffer)



print ('Program ended')