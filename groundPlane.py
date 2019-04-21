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
from readSensors import getDepthImage,cameraOrientation,compassRobot
import Hough
import math

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from sklearn import preprocessing
from PIL import Image

# Connect to V-REP Server
def checkConnectivity():
    # Check Connectivity
#    print ('Program started')
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
    
    if clientID!=-1:
        print ('Connected to remote API server')
    else:
        vrep.simxFinish(clientID)
        sys.exit("Failed connecting to remote API server")
    
    print(" ")
    return clientID


# Convert XYZ data into list of points with x,y,z data
def convertXYZ(auxPackets):
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
        
        pointList.append([xValue, yValue, zValue,distance])
        # distances.append(distance)
    return pointList

# for plotting the ground plane
def plotGroundPlane(xVals,yVals,zVals,zPlane):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(xVals, yVals, zVals, c='r', marker='o') # original coordinates
    ax.scatter(xVals, yVals, zPlane, c='b', marker='v') # Calculated Ground Plane
    
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title('Ground Plane Detection')
    
    plt.show()

# calculate axis angle values for vectors v1 and v2
def axisAngle(v1,v2):
#    rollAxis = np.cross(v1,v2)/np.linalg.norm(np.cross(v1,v2))
    axis = np.cross(v2,v1)
    axis = axis/np.linalg.norm(axis)
    
    # account for switching normal vector (positive of negative due to ransac)
    if axis[0] < 0:
        axis = -axis
    
    # calculate roll anlge
    angle = (math.acos(v1.dot(v2)/(np.linalg.norm(v1)*np.linalg.norm(v2))))
    
    
    # account of opposite direction of normal vector
    # which occurs to random ransac sampling
    angleD = math.degrees(angle)
    if angleD > 90:
        angleD = 180 % angleD
        angle = math.radians(angleD)
        
    return axis,angle

# rotate using axis angle representation
def rotateAxisAngle(axis,angle,point):
    angle = -angle
    x,y,z = axis[0],axis[1],axis[2] 
    
    c = math.cos(angle)
    s = math.sin(angle)
    C = 1- c
    R11 = x*x*C+c
    R12 = x*y*C-z*s
    R13 = x*z*C+y*s
    
    R21 = y*x*C+z*s
    R22 = y*y*C+c
    R23 = y*z*C-x*s
    
    R31 = z*x*C-y*s
    R32 = z*y*C+x*s
    R33 = z*z*C+c
    
    R = np.array([[R11, R12, R13],[R21, R22, R23],[R31, R32, R33]])

    return (R.dot(point))
    
#def main():
# Connect to V-Rep
clientID = checkConnectivity()  
auxPackets,resolution,rgbImage = getDepthImage(clientID)    

showGroundPlane = False
showBinary = True


# Camera Parameters
focalLength = 700 # pixels
centerX = resolution[0]/2
centerY = resolution[1]/2# Camera Calibration Matrix
pxSize = 4e-6

# read point cloud data as an array
pointList = convertXYZ(auxPackets)
pointList = np.asarray(pointList)

# extract coordinates
xVals = np.round(pointList[:,0],2)
yVals = np.round(pointList[:,1],2) # switch z and y
zVals = np.round(pointList[:,2],2)
distVals = np.round(pointList[:,3],2)

# remove distance values form point list
pointList = pointList[:,:3]

# perform ransac on point cloud to predict ground plane equation
planeParameters = ransacPlane(pointList)

# Use ground plane equation to overlay points over original points
zPlane = [0]*len(xVals)
for i in range(len(xVals)):
    # Note B and C are switched
    zPlane[i] = (-planeParameters[0][0]*xVals[i]-planeParameters[0][1]*
          yVals[i] -planeParameters[1])/(planeParameters[0][2])

## find camera height (0,0,0)
cameraHeight = planeParameters[1]/np.linalg.norm(planeParameters[0])
#print("Camera Height: " + str(cameraHeight))


# Want ground plane to be on the actual ground in plot, rotate ground plane
groundDesired = np.array([0,0,1])   # desired ground plane normal
groundPlane =  planeParameters[0]/np.linalg.norm(planeParameters[0])

# calculate axis and angle
axis,angle = axisAngle(groundDesired,groundPlane)
#print(np.sum(np.power(axis,2)))
#print(math.degrees(angle))
#print("roll axis")
#print(axis)

# transform all points into desired location
zPlane1 = [0]*len(xVals)
xVals1 = [0]*len(xVals)
yVals1 = [0]*len(xVals)
zVals1 = [0]*len(xVals)
for i in range(len(xVals)):
    realPoint = np.transpose(np.array([xVals[i],yVals[i],zVals[i]]))
    planePoint = np.transpose(np.array([xVals[i],yVals[i],zPlane[i]]))
   
    
    [xVals1[i],yVals1[i],zVals1[i]] = rotateAxisAngle(axis,angle,realPoint)
    [xVals1[i],yVals1[i],zPlane1[i]] = rotateAxisAngle(axis,angle,planePoint)  

# shift ground plane to be at z = 0
groundPlaneHeight = -np.mean(zPlane1)
zVals1 += groundPlaneHeight
zPlane1 += groundPlaneHeight




# Collapse Feature Map to Image
scale = 30
xMin = int(math.floor(min(xVals1)*scale))
xMax = int(math.ceil(max(xVals1)*scale))
yMin = int(math.floor(min(yVals1)*scale))
yMax = int(math.ceil(max(yVals1)*scale))
xRange = xMax-xMin
yRange = yMax-yMin

# find pixel to distance scaling
xMinReal = min(xVals1)
xMaxReal = max(xVals1)
yMinReal = min(yVals1)
yMaxReal = max(yVals1)
xRangeReal = xMaxReal-xMinReal
yRangeReal = yMaxReal-yMinReal

xPx2Dst = xRange/xRangeReal
yPx2Dst = yRange/yRangeReal
px2Dst = (xPx2Dst+yPx2Dst)/2
#print(xRangeReal)

zMax = max(zVals1)
zThresh = zMax/10  # Keep heights greater than 10 % of max


cropImage = np.zeros((yRange+1,xRange+1)).astype('uint8')
for i in range(len(xVals)):
    heightVal = zVals1[i]
    if (heightVal > zThresh):
        heightVal = 255
    else:
        heightVal = 0  
    xCoor = int(xVals1[i]*scale)-xMin
    yCoor = int(yVals1[i]*scale)-yMin
    cropImage[yCoor,xCoor] = heightVal

# create opencv compatible image
pil_image = Image.fromarray(cropImage)
pil_image = pil_image.convert('RGB')
opencvImage = np.asarray(cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2GRAY))  


# blur image
filterSize = 3
kernel = np.ones((filterSize,filterSize),np.float32)/filterSize**2
opencvImage = cv2.filter2D(opencvImage,-1,kernel)

# plot the ground plane
if showGroundPlane:
    plotGroundPlane(xVals,yVals,zVals,zPlane)
    plotGroundPlane(xVals1,yVals1,zVals1,zPlane1)


H,rhos,thetas = Hough.hough_lines_acc(opencvImage)

Hough.plot_hough_acc(H)
#peaks = Hough.hough_simple_peaks(H,6)   


peaks1,H1 = Hough.hough_peaks(H,6,nhood_size=scale,)    
peaks1 = Hough.correctLines(peaks1,px2Dst)
Hough.hough_lines_draw(opencvImage,peaks1,rhos,thetas)


# center of image


    





if showBinary:
    cv2.namedWindow('image',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('image', 600,600)
    cv2.imshow('image',opencvImage)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cv2.imwrite('../reportImages/rowFeatureMapHoughNMS_Blur_Filtered.png',opencvImage)
    
    
    
    
    



    
#if __name__ == '__main__':
#    main()
#











