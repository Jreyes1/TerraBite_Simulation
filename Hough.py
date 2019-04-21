#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr  6 14:22:07 2019

@author: jorge
"""

import numpy as np
import math

import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
import matplotlib.pyplot as plt

# function for creating a hough accumulator for lines in an image
def hough_lines_acc(img,rho_resolution=1,theta_resolution=1):
    height,width = img.shape
    
    img_diagonal = np.ceil(np.ceil(np.sqrt(height**2+width**2)))
    rhos = np.arange(-img_diagonal,img_diagonal+1,rho_resolution)
    thetas = np.deg2rad(np.arange(-90,90, theta_resolution))

    
    # create the empty Hough Accumulator with dimmensions
    # equal to the size of rhos and thetas
    H = np.zeros((len(rhos),len(thetas)), dtype=np.uint64)
    y_idxs,x_idxs = np.nonzero(img) # find all nonzero pixel indicies
    
    # cycle through nonzero pixels and vote in hough accumulator
    for i in range(len(x_idxs)):
        x = x_idxs[i]
        y = y_idxs[i]
        
        for j in range(len(thetas)):
            
            # calculate rho, add diag for positive index
            rho = int(((x*np.cos(thetas[j]) + 
                       y*np.sin(thetas[j])) + img_diagonal)/rho_resolution)
            H[rho][j] += 1
            
    return H,rhos,thetas


    

# function that returns indicies of the number of maximum values
# equal to numpeaks
def hough_simple_peaks(H,num_peaks):
    indices = np.argpartition(H.flatten(), -2)[-num_peaks:]
    return np.vstack(np.unravel_index(indices, H.shape)).T

# find peaks incorporating non-maximum supression
def hough_peaks(H,num_peaks,nhood_size=3):
    # loop through number of peaks to identify
    indicies = []
    H1 = np.copy(H)
    for i in range(num_peaks):
        idx = np.argmax(H1) # find argmax in flattened array
        H1_idx = np.unravel_index(idx, H1.shape) # linear index to x, y form
        indicies.append(H1_idx)

        # surpess indicies in neighborhood
        idx_y, idx_x = H1_idx # first separate x, y indexes from argmax(H)
        # if idx_x is too close to the edges choose appropriate values
        if (idx_x - (nhood_size/2)) < 0: min_x = 0
        else: min_x = int(idx_x - nhood_size/2)
        if ((idx_x + nhood_size/2 + 1) > H.shape[1]): max_x = H.shape[1]
        else: max_x = int(idx_x + nhood_size/2 + 1)

        # if idx_y is too close to the edges choose appropriate values
        if (idx_y - (nhood_size/2)) < 0: min_y = 0
        else: min_y = int(idx_y - (nhood_size/2))
        if ((idx_y + (nhood_size/2) + 1) > H.shape[0]): max_y = H.shape[0]
        else: max_y = int(idx_y + nhood_size/2 + 1)

        # bound each index by the neighborhood size and set all values to 0
        for x in range(min_x, max_x):
            for y in range(min_y, max_y):
                # remove neighborhoods in H1
                H1[y][x] = 0

                # highlight peaks in original H
                if (x == min_x or x == (max_x - 1)):
                    H[y][x] = 255
                if (y == min_y or y == (max_y - 1)):
                    H[y][x] = 255

    # return the indicies and the original Hough space with selected points
    return indicies, H1

def correctLines(peaks,px2Dst):
    
    # extract rho and angle, convert rho to real world dimmensions    
    peaks = np.asarray(peaks)
    spacings = peaks[:,0]
    angles = peaks[:,1]
    
    # Find angle the minimizes MSE
    error = math.inf
    N = len(angles)
    for i in range(N):
        
        sumTerm = 0
        num = N-1
        for j in range(N):
            if i==j:
                pass
            else:
                sumTerm += (angles[i]-angles[j])**2
        MSE = 1/(num)*sumTerm
        if (MSE < error):
            error = MSE
            bestAngle = angles[i]
    print(bestAngle)
    
    # Calculate an Average 
    correctAngles = []
    newRhos = []
    for i in range(N): 
        if (abs(bestAngle - angles[i]) < 15):
            correctAngles.append(angles[i])
            newRhos.append(spacings[i])
    
    # find the angle and row spacing
    meanAngle = np.mean(np.asarray(correctAngles))
    rowSpacing = np.mean(np.diff(np.asarray(newRhos)))
    print(meanAngle,rowSpacing)
    
    newPeaks = []
    for i in range(len(correctAngles)):
        newPeaks.append([newRhos[i],int(round(meanAngle))])

  
    return newPeaks
    
    
    
# function to visualize Hough Accumulator
def plot_hough_acc(H,plot_title='Hough Accumulator Plot'):
    fig = plt.figure(figsize=(10, 10))
    fig.canvas.set_window_title(plot_title)
    	
    plt.imshow(H, cmap='jet')

    plt.xlabel('Theta Direction'), plt.ylabel('Rho Direction')
    plt.tight_layout()
    plt.show()

    
def hough_lines_draw(img, indicies, rhos, thetas):
    ''' A function that takes indicies a rhos table and thetas table and draws
        lines on the input images that correspond to these values. '''
    for i in range(len(indicies)):
        # reverse engineer lines from rhos and thetas
        rho = rhos[indicies[i][0]]
        theta = thetas[indicies[i][1]]
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        # these are then scaled so that the lines go off the edges of the image
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))

        cv2.line(img, (x1, y1), (x2, y2), (255, 255, 255), 2)
