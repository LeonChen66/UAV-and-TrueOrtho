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
from math import *


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

def UAV2ortho(PC_data,approx,affine_par,h_max,h_min,*EX_par):
    [A,D,B,E,C,F] = np.loadtxt(affine_par)
    
    point_cloud = pd.read_csv(PC_data,names=['X', 'Y', 'Z', 'R','G','B'],delim_whitespace=True)
    
    X = point_cloud.X.copy().values
    Y = point_cloud.Y.copy().values      # Inverse the Y-axis
    Z = point_cloud.Z.copy().values
    
    del point_cloud
    #Dsm_arr = np.zeros((h, w))

    loc = np.where((Z<h_max)&(Z>h_min))
    #EX_par
    XL = EX_par[0];
    YL = EX_par[1];
    ZL = EX_par[2];
    omega = np.deg2rad(EX_par[3]);
    phi = np.deg2rad(EX_par[4]);
    kappa = np.deg2rad(EX_par[5]);
    f = EX_par[6];
    x0 = EX_par[7];
    y0 = EX_par[8];
    # Colinear Equation
    m11 = cos(phi)*cos(kappa);
    m12 = sin(omega)*sin(phi)*cos(kappa)+cos(omega)*sin(kappa);
    m13 = -cos(omega)*sin(phi)*cos(kappa)+sin(omega)*sin(kappa);
    m21 = -cos(phi)*sin(kappa);
    m22 = -sin(omega)*sin(phi)*sin(kappa)+cos(omega)*cos(kappa);
    m23 = cos(omega)*sin(phi)*sin(kappa)+sin(omega)*cos(kappa);
    m31 = sin(phi);
    m32 = -sin(omega)*cos(phi);
    m33 = cos(omega)*cos(phi);
    
    XA = X[loc]
    YA = Y[loc]
    ZA = Z[loc]    

    xa = x0 - f*((m11*(XA-XL)+m12*(YA-YL)+m13*(ZA-ZL))/(m31*(XA-XL)+m32*(YA-YL)+m33*(ZA-ZL)));
    ya = y0 + f*((m21*(XA-XL)+m22*(YA-YL)+m23*(ZA-ZL))/(m31*(XA-XL)+m32*(YA-YL)+m33*(ZA-ZL)));
    
    # cKDtree
    tree = spatial.cKDTree(list(zip(xa, ya)))
    pts = np.squeeze(approx)
    print(pts)
    dis, loc = tree.query(pts, k=1)
    print(dis)
    x = XA[loc]
    y = YA[loc]
    ortho_edgeP = np.matrix([[A,B],[D,E]]).I*np.squeeze(np.dstack((x-C,y-F))).T 
    return ortho_edgeP
    
def draw_Point(ortho_edgeP,h,w):
    ortho = np.zeros((h,w))
    ortho_edgeP = ortho_edgeP.T.astype(int)
    print(ortho_edgeP)
    cv2.drawContours(ortho,[ortho_edgeP],-1,(255,255,255),-1)
    cv2.imwrite('ortho_building_footprint.jpg',ortho)
    img = cv2.imread('ntu_pix4d_0212_transparent_mosaic_group1.tif')
    cv2.drawContours(img,[ortho_edgeP],-1,(0,0,255),3)
    cv2.imwrite('test.jpg',img)    
    
if __name__=="__main__":
    XL = -44.565271;
    YL = 17.775985;
    ZL = 14.067157;
    omega = -0.985902;
    phi = 0.030544;
    kappa = -85.832167;
    f = 3803.28260713083182054106;
    x0 = 2471.84341749838540636119;
    y0 = 1653.25150608682383790438;
    
    EX_par = [XL,YL,ZL,omega,phi,kappa,f,x0,y0]
    ortho_edgeP = UAV2ortho('ntu_pix4d_0212_group1_densified_point_cloud.xyz',approx
              ,'ntu_pix4d_0212_dsm.tfw',-220,-240,*EX_par)
    
    draw_Point(ortho_edgeP,7118,4186)