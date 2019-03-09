# -*- coding: utf-8 -*-
"""
Created on Mon Feb 25 13:05:57 2019

@author: jr31
"""
import math
import matplotlib.pyplot as plt
import numpy as np

tol = 0.001

def Jacobi(A,b,x):
    print("Running Jacobi Alogorithm")
    print("Using Guess: ")
    print( str(x))
    
    # seperate into diagonal and triangulatr matrix
    D =  np.diag(A.diagonal()) 
    L = np.tril(A,-1)
    U = np.triu(A,1)
    P = -(np.linalg.inv(D)).dot(L+U)
    c = np.linalg.inv(D).dot(b)
    
    iterations = 0
    while (abs(np.linalg.norm(A.dot(x)-b)) > tol):
        x = P.dot(x)+c
        iterations += 1
        
    print("After " + str(iterations) + " iterations")
    print("Root Located at: ")
    print(str(x) + "\n")
    return x

def conjugateGradient(A,b,x):
    print("Running Conjugate Gradient Alogorithm")
    print("Using Guess: ")
    print( str(x))
    
    d = b - A.dot(x)
    r = d
    
    iterations = 0
    while (abs(np.linalg.norm(A.dot(x)-b)) > tol):
        alpha = (np.transpose(r).dot(r))/(np.transpose(d).dot(A).dot(d))
        x = x + alpha*d 
        rnew = r - (alpha*A).dot(d)
        beta = (np.transpose(rnew).dot(rnew))/(np.transpose(r).dot(r))
        d = rnew + beta*d
        
        iterations += 1
        r = rnew
        
    print("After " + str(iterations) + " iterations")
    print("Root Located at: ")
    print(str(x) + "\n")
    return x
    
 
def steepestDescent(A,b,x):
    print("Running Steepest Descent Alogorithm")
    print("Using Guess: ")
    print( str(x))

    iterations = 0
    error = math.inf
    while (error > tol):
        r = b - A.dot(x)
#        if (iterations == 0):
#            print("Direction of Line Search")
#            print(r)
        r_T = np.transpose(r)
        alpha = np.dot(r_T,r)/(r_T.dot(A).dot(r))
        x_new = x + alpha*r

        error = np.linalg.norm(x_new - x)
        x = x_new
        iterations += 1

    print("After " + str(iterations) + " iterations")
    print("Root Located at: ")
    print(str(x) + "\n")
    return x

def problem2():
    A1 = np.array([[4,-1,0],[-1,4,-1],[0,-1,4]])
    A2 = np.array([[-1,0,0],[0,-1,0],[0,0,-1]])
    A3 = np.zeros((3, 3))
    
#    row1 = np.concatenate((A1,A2,A3))
#    row2 = np.concatenate((A2,A1,A2))
#    row3 = np.concatenate((A3,A2,A1))
#    A = np.concatenate((row1,row2,row3),axis=1)
    A = np.block([[A1,A2,A3],[A2,A1,A2],[A3,A2,A1]])
    b = np.ones((9,1))
    
    # initial guess
    x0 = np.zeros((9,1))
    steepestDescent(A,b,x0)
    Jacobi(A,b,x0)
    conjugateGradient(A,b,x0)
    
   

    
#problem2()




