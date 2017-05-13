# -*- coding: utf-8 -*-
"""
Created on Sat Feb 18 01:16:56 2017

@author: Leon
"""
from osgeo import gdal
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import spatial
import cv2

im = cv2.imread('fill.jpg')
ntu = cv2.imread('DSCF2098_1471837627895.jpg')
imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
ret,thresh = cv2.threshold(imgray,127,255,0)
__,contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
print("there are " + str(len(contours)) + " contours")

#size
[h,w,_] = im.shape
im_final = np.zeros((h,w))


cnt = contours[0]
print("there are " + str(len(cnt)) + " points in contours[0]")
approx = cv2.approxPolyDP(cnt,30,True)
print("after approx, there are " + str(len(approx)) + " points")
print(approx)
cv2.drawContours(im,[approx],0,(255,0,0),-1)
 
contours.sort(key=len,reverse = True)
cnt = contours[0]
print("there are " + str(len(cnt)) + " points in contours[1]")
approx = cv2.approxPolyDP(cnt,50,True)
print("after approx, there are " + str(len(approx)) + " points")
print(approx)
cv2.drawContours(im,[approx],0,(0,255,0),-1)
cv2.drawContours(ntu,[approx],-1,(255,0,0),3)
cv2.drawContours(im_final,[approx],-1,(255,255,255),-1)

cv2.imwrite('contour.jpg',im)
cv2.imwrite('contour_ntu.jpg',ntu)
cv2.imwrite('final_building.jpg',im_final)

