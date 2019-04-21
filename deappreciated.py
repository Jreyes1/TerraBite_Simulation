#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr  7 13:13:14 2019

@author: jorge
"""
# DEAPPRECIATED FUNCTION (USING DEPTH DATA INSTEAD of Bird's Eye View)
def cameraNavigate():
    # Find angle and axis of rotation between ground plane normal and camera axis
    groundPlaneNormal = planeParameters[0]
   
    # Get camera orientation
    cameraPitch = cameraOrientation(clientID)
    print("camera Pitch: " + str(cameraPitch))
    
    cameraPitch = math.radians(cameraPitch)
    # Construct camera direction vector
    cameraVector = np.array([0,math.cos(cameraPitch),math.sin(cameraPitch)])
    rollAxis = np.linalg.norm(np.cross(cameraVector,groundPlaneNormal))
    rollAngle = (math.acos(cameraVector.dot(groundPlaneNormal)/(
            np.linalg.norm(groundPlaneNormal)*np.linalg.norm(cameraVector))))
    print("angle from camera to ground plane normal")
    print(math.degrees(rollAngle))
    
    def nothing(x):
        pass
    
    cv2.namedWindow('Colorbars')
    cv2.createTrackbar('Angle','Colorbars',10,90,nothing)
    cv2.createTrackbar('Depth','Colorbars',10,5000,nothing)
    cv2.createTrackbar('FocalLength','Colorbars',700,1000,nothing)
    cv2.createTrackbar('Scale','Colorbars',100,100,nothing)
    cv2.createTrackbar('xScale','Colorbars',500,1000,nothing)
    cv2.createTrackbar('yScale','Colorbars',500,1000,nothing)

    # get original image
    img = np.array(rgbImage,dtype=np.uint8)
    img.resize([resolution[1],resolution[0],3])
    img = cv2.flip(img,0)
    cv2.imshow('Original',img)
    cv2.waitKey(1)

    while(True):
        # Create trackbars to be able to manually adjust parameters
        angle = cv2.getTrackbarPos('Angle','Colorbars')
        depth = cv2.getTrackbarPos('Depth','Colorbars')
        focalLength =  cv2.getTrackbarPos('FocalLength','Colorbars')+1
        scale = cv2.getTrackbarPos('Scale','Colorbars')/100
        dx  = cv2.getTrackbarPos('xScale','Colorbars')-500
        dy  = cv2.getTrackbarPos('yScale','Colorbars')-500
    
        rollAngle = math.radians(angle)
        
        # Camera Intrinsic and Rotation Matricies
        K =  np.array([[focalLength, 0, centerX+dx],[0,focalLength,centerY+dy],[0,0,1]])
        R = np.array([[1,0,0],[0,math.cos(rollAngle),-math.sin(rollAngle)],[
                0,math.sin(rollAngle),math.cos(rollAngle)]])
        H = K.dot(R).dot(np.linalg.inv(K))
        
        # find projected location of projected center
        projectCenter = H.dot(np.array([[centerX],[centerY],[1]]))
        [projectCenterX,projectCenterY, tmp ] = projectCenter

        # Scaling Picture to fit window
        scaleM = np.array([[scale,0,0],[0,scale,0],[0,0,1]])
        H = scaleM.dot(H)
        
        # resize image into typical RGB image format
        img = np.array(rgbImage,dtype=np.uint8)
        img.resize([resolution[1],resolution[0],3])
        img = cv2.flip(img,0)
        
        # transform image
        groundImg = cv2.warpPerspective(img,H,(resolution[0],resolution[1]))
    
        cv2.imshow('transformed',groundImg)
        
        key = cv2.waitKey(1)
        if key == 27:
            break
    cv2.destroyAllWindows()

#def hough_pattern_acc(img,px2Dst,theta_resolution=1,spacing=1.5,spacing_resolution=0.1,):
#    
#    thetas = np.deg2rad(np.arange(-90,90, theta_resolution))    # theta
#    s = np.arange(-spacing,spacing,spacing_resolution)*px2Dst   # row spacing
#    # create the empty Hough Accumulator
#    
#    # row spacing, origin offset, theta
#    lenO = len(s)
#    H = np.zeros((len(s),lenO,len(thetas)), dtype=np.uint64)
#    y_idxs,x_idxs = np.nonzero(img) # find all nonzero pixel indicies
#    print(len(y_idxs))
#    
#    # iterate through non-zero pixels
#    for i in range(len(x_idxs)):
#        x = x_idxs[i]
#        y = y_idxs[i]
#
#        # iterate through thetas
#        for j in range(len(thetas)):
#            
##            print(i,j)
#            # iterate through row spacings
#            for k in range(len(s)):
#                rho = x*np.cos(thetas[j]) + y*np.sin(thetas[j])
#                minRangeO = rho/s[k]-1
#                maxRangeO = rho/s[k]
##                print(s[k])
##                print(minRangeO,maxRangeO)
#                n = int(np.ceil((maxRangeO+minRangeO)/2))
##                print(n)
#                o = int(round(x*np.cos(thetas[j]) + y*np.sin(thetas[j]) -n*s[k],1))
#                
##                print("o: " + str(o))
##                print("")
#                
#                rowSpacing = int((s[k]/px2Dst+spacing)/spacing_resolution)
#                rowOffset = int((o/px2Dst+spacing)/spacing_resolution)
##                print(rowSpacing,rowOffset)
##                print(angle)
#                H[rowSpacing][rowOffset][j] +=1
#    print("finished")
#    return H
