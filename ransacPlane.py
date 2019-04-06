#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar  1 14:20:11 2019

@author: jorge
"""

import random
import numpy as np
import math

# Calculate Plane Equation using 3 points
def planeEquation(point1, point2, point3):
    
    # plane equation is Ax + By + Cz = D
    # where [A,B,C]' = normal direction to plane
    point1 = np.asarray(point1)
    point2 = np.asarray(point2)
    point3 = np.asarray(point3)
    
    # Construct two lines on plane using set of points
    r1 = point1 - point2 
    r2 = point3 - point2
    
    # Normal vector to plane
    N = np.cross(r1,r2) 
    
    # Find the D parameter of the plane equation
    D = -N.dot(point1)
    
    planeParameters= [N,D] # N has A,B,C
    return planeParameters

# Find the number of inliers for given plane equation
def calcInliers(pointList, planeParameters):
    numPoints = len(pointList)
    N = planeParameters[0]
    D = planeParameters[1]
    
    numInliers = 0
    inlierPoints = []
    tol = 0.001 # in meters
    for i in range(numPoints):
        # find normal distance to plane
        distance = abs((N.dot(pointList[i]) + D)/ np.linalg.norm(N))
        
        # only count as inlier if within tolerance
        if distance < tol:
            numInliers += 1
            inlierPoints.append(pointList[i])
    
    return (numInliers,inlierPoints)

    
# Perform Adaptive RANSAC
def ransacPlane(pointList):
    N =  math.inf # num of ransac iterations
    sampleCount = 0
    inlierRatio = 0
    numPoints = len(pointList)
    p = 0.95
    s = 3 # number of points needed to fit model
    
    # variables to keep track of best parameters
    bestNumInliers = 0
    bestPlane = [0]
   
#    for iteration in range(N):
    while(N > sampleCount):
        
        # select 3 random points
        pointIndex = random.sample(range(numPoints-1),s)
        point1 = pointList[pointIndex[0]]
        point2 = pointList[pointIndex[1]]
        point3 = pointList[pointIndex[2]]

        # calculate plane equation
        planeParameters = planeEquation(point1, point2, point3)
        
        # calculate inliers
        numInliers, inlierPoints = calcInliers(pointList,planeParameters)
        
        # Keep the plane parameters that produce the most inliers
        if (numInliers > bestNumInliers):
            bestNumInliers = numInliers # track number of inliers
            bestInliers = inlierPoints 	# track the inlier points
            bestPlane = np.array(planeParameters) # track best plane equation
            
            # Update N using updated inlier ratio
            inlierRatio = bestNumInliers/numPoints
            e = 1-inlierRatio
            N = math.log(1-p)/math.log(1-(1-e)**s)
        
        # increment sample count
        sampleCount += 1
            
    return bestPlane


    
 
    
            

        
        
    
    