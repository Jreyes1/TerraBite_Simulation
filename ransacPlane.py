#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar  1 14:20:11 2019

@author: jorge
"""

import random
import numpy as np
from scipy.linalg import lstsq

def planeEquation(point1, point2, point3):
    # plane equation is Ax + By + Cz = D
    # where [A,B,C]' = normal direction to plane
    point1 = np.asarray(point1)
    point2 = np.asarray(point2)
    point3 = np.asarray(point3)
    
    
    
    r1 = point1 - point2 # line on plane
    r2 = point3 - point2 # line on plane
    
    N = np.cross(r1,r2) # normal vector to plane (gives )
    D = -N.dot(point1)
    
    planeParameters= [N,D] # N has A,B,X
    return (planeParameters)

def calcInliers(pointList, planeParameters):
    numPoints = len(pointList)
    N = planeParameters[0]
    D = planeParameters[1]
    
    numInliers = 0
    inlierPoints = []
    tol = 0.01 # 1 centimeter
    for i in range(numPoints):
        distance = abs((N.dot(pointList[i]) + D)/ np.linalg.norm(N))
        if distance < tol:
            numInliers += 1
            inlierPoints.append(pointList[i])
    
    return (numInliers,inlierPoints)

# refine parameters using conjugate gradient descent
def refinePlane(inlierPointList, planeParameters):
    
    # number of points in list
    numPoints = len(inlierPointList)
    
    # have to convert parameters from list to array form
    inlierPointList = np.asarray(inlierPointList)
    planeParameters = np.asarray(planeParameters)
    
    A = inlierPointList[:,[0,1]]
    A1 = np.ones((numPoints,1))
    A = np.append(A,A1, axis=1)
    
    B = inlierPointList[:,[2]]
    
    
    
#    planeParameters = conjugateGradient(A,B,planeParameters)
    print("Initial Parameters from Ransac")
    print(np.asarray(planeParameters))

    planeParameters = np.linalg.lstsq(A,B,rcond=None)[0]
    print("Refining using Least Squares Solution")
    print(planeParameters)


    return planeParameters
    

def ransacPlane(pointList):
    N = 50 # num of ransac iterations
    numPoints = len(pointList)
    
    # variables to keep track of best parameters
    bestNumInliers = 0
    
    bestPlane = [0]
    
    for iteration in range(N):
        
        # select 3 random points
        pointIndex = random.sample(range(numPoints-1),3)
        point1 = pointList[pointIndex[0]]
        point2 = pointList[pointIndex[1]]
        point3 = pointList[pointIndex[2]]

        # calculate plane equation
        planeParameters = planeEquation(point1, point2, point3)
        
        # calculate inliers
        numInliers, inlierPoints = calcInliers(pointList,planeParameters)
        
        if (numInliers > bestNumInliers):
            bestNumInliers = numInliers # track number of inliers
            bestInliers = inlierPoints # track the inlier points
            bestPlane = np.array(planeParameters) # track best plane equation
    print("Ransac Residual")
    residual = 0
    for i in range(len(bestInliers)):
        
        
        points = np.transpose(np.array(bestInliers[i]))     # transpose points
        residual+= bestPlane[0].dot(points)+bestPlane[1]    # ax+by+cz+d = residual
    print(residual)
#        A = np.block([[points],[1]])
#        residual += bestPlane.dot(point)
#        print(residual)
#        point = np.block([[np.transpose(bestInliers[i])],[1]])
#        residual += bestPlane.dot(point)
#        print(residual)
        
        
        
    
    # refine plane equation using inliers
#    bestPlane = refinePlane(bestInliers, bestPlane)
            
    # return refined plane parameters
    return bestPlane
            

        
        
    
    