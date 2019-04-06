#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 30 22:05:19 2019

@author: jorge
"""
import math
import numpy as np
#from  import Matrix


print("input")
X0 = np.array([[5],[4],[2]])
print(X0)
H = np.array([[2,2,2],[3,3,3],[4,4,4]])

output1 = H.dot(X0)
#output1[2] = 1
print("output 1")
print(output1)

print("error")
error = X0-output1
print(error)

T = np.array([[1,0,0,error[0]],[0,1,0,error[1]],[0,0,1,error[2]],[0,0,0,1]])
#T = Matrix.Translations((error[0],error[1],0))
output2 = T.dot(np.block([[output1],[1]]))



print("output 2")
print(output2)

zeroV = np.zeros((3,1))
horiz = np.array([0,0,0,1])
H = np.block([[H,zeroV],[horiz]])
X0 = np.block([[X0],[1]])
print("output 3")
print(T.dot(H))