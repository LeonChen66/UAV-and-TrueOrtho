# -*- coding: utf-8 -*-
"""
Created on Sun May 14 08:13:34 2017

@author: Leon
"""

import numpy as np
import cv2
import pandas as pd
from osgeo import gdal
from math import *

def dsm_angle(dsm_name,affine_par,EX_par):
    # load Dsm
    ds = gdal.Open(dsm_name)
    band = ds.GetRasterBand(1)
    dsm_arr = band.ReadAsArray()
    
    [h,w] = dsm_arr.shape
    
    [A,D,B,E,C,F] = affine_par
    [XL,YL,ZL,omega,phi,kappa] = EX_par
    #calculate nadir
    nadir_matrix = np.array(np.matrix([[A,B],[D,E]]).I*np.matrix([[XL-C],[YL-F]]))
    nadir_col = nadir_matrix[0]
    nadir_row = nadir_matrix[1]
    print(nadir_col,nadir_row)
    X_nadir = np.around(nadir_col)*A+np.around(nadir_row)*B+C;
    Y_nadir = np.around(nadir_col)*D+np.around(nadir_row)*E+F;
    Z_nadir = dsm_arr[np.around(nadir_row).astype(int),np.around(nadir_col).astype(int)];
    X_nadir = X_nadir - XL
    Y_nadir = Y_nadir - YL
    Z_nadir = Z_nadir - ZL                  
    
    # Calculate each cell's Angle
    [y,x] = np.mgrid[0:h,0:w]
    XA = (A*x)+(B*y)+C;
    YA = (D*x)+(E*y)+F;
    ZA = dsm_arr;
    vector_X = XA-XL;
    vector_Y = YA-YL;
    vector_Z = ZA-ZL;   
    del x,y,XA,YA,ZA
    #vector_A = np.squeeze(np.dstack((vector_X,vector_Y,vector_Z)))
     
    
    vector_dot = (vector_X*X_nadir+vector_Y*Y_nadir+vector_Z*Z_nadir) \
            /(np.sqrt((vector_X)**2+(vector_Y)**2+(vector_Z)**2) \
             *np.sqrt((X_nadir)**2+(Y_nadir)**2+(Z_nadir)**2))
    dsm_angle = np.arccos(vector_dot)
    dsm_angle = np.rad2deg(dsm_angle)
    
    return dsm_angle,int(nadir_row[0]),int(nadir_col[0])
   
def sort_list(z_list):
    temp = -10000        #temp = current max value
    for i in range(len(z_list)):
        if temp<z_list[i]:
            temp = z_list[i]
        else:
            z_list[i] = -10000
    
    return z_list
    
def occluded(dsm_name,dsm_angle,nadir_row,nadir_col):
    ds = gdal.Open(dsm_name)
    band = ds.GetRasterBand(1)
    dsm_arr = band.ReadAsArray()
    [h,w] = dsm_angle.shape
    j = 0
    for i in range(w):
        if abs(nadir_col-i) >= abs(nadir_row-j):
            x = np.linspace(nadir_col,i,abs(i-nadir_col)+1).astype(int)
            y = np.linspace(nadir_row,j,abs(i-nadir_col)+1).astype(int)
            z = dsm_angle[y,x]
            z = sort_list(z)
            dsm_angle[y,x] = z
        else:
            x = np.linspace(nadir_col,i,abs(j-nadir_row)+1).astype(int)
            y = np.linspace(nadir_row,j,abs(j-nadir_row)+1).astype(int)
            z = dsm_angle[y,x]
            z = sort_list(z)
            dsm_angle[y,x] = z

    j = h-1
    for i in range(w):
        if abs(nadir_col-i) >= abs(nadir_row-j):
            x = np.linspace(nadir_col,i,abs(i-nadir_col)+1).astype(int)
            y = np.linspace(nadir_row,j,abs(i-nadir_col)+1).astype(int)
            z = dsm_angle[y,x]
            z = sort_list(z)
            dsm_angle[y,x] = z
        else:
            x = np.linspace(nadir_col,i,abs(j-nadir_row)+1).astype(int)
            y = np.linspace(nadir_row,j,abs(j-nadir_row)+1).astype(int)
            z = dsm_angle[y,x]
            z = sort_list(z)
            dsm_angle[y,x] = z

    j = w-1
    for i in range(h):
        if abs(nadir_col-j) <= abs(nadir_row-i):
            x = np.linspace(nadir_col,j,abs(i-nadir_row)+1).astype(int)
            y = np.linspace(nadir_row,i,abs(i-nadir_row)+1).astype(int)
            z = dsm_angle[y,x]
            z = sort_list(z)
            dsm_angle[y,x] = z
        else:
            x = np.linspace(nadir_col,j,abs(j-nadir_col)+1).astype(int)
            y = np.linspace(nadir_row,i,abs(j-nadir_col)+1).astype(int)
            z = dsm_angle[y,x]
            z = sort_list(z)
            dsm_angle[y,x] = z

    j = 0
    for i in range(h):
        if abs(nadir_col-j) <= abs(nadir_row-i):
            x = np.linspace(nadir_col,j,abs(i-nadir_row)+1).astype(int)
            y = np.linspace(nadir_row,i,abs(i-nadir_row)+1).astype(int)
            z = dsm_angle[y,x]
            z = sort_list(z)
            dsm_angle[y,x] = z
        else:
            x = np.linspace(nadir_col,j,abs(j-nadir_col)+1).astype(int)
            y = np.linspace(nadir_row,i,abs(j-nadir_col)+1).astype(int)
            z = dsm_angle[y,x]
            z = sort_list(z)
            dsm_angle[y,x] = z

    loc = np.where(dsm_angle == -10000)
    dsm_arr[loc] = -10000
    return dsm_arr
    

def ortho_gen(img_name,dsm_arr_occlude,f,x0,y0,affine_par,EX_par):
    # load Dsm and image
    dsm_arr = dsm_arr_occlude
    img = cv2.imread(img_name)
    #shape of Dsm and image
    [h,w] = dsm_arr.shape
    [img_h,img_w,_] = img.shape
    [A,D,B,E,C,F] = affine_par
    [XL,YL,ZL,omega,phi,kappa] = EX_par
    omega = np.deg2rad(omega)
    phi = np.deg2rad(phi)
    kappa = np.deg2rad(kappa)
    #colinear equation
    m11 = cos(phi)*cos(kappa);
    m12 = sin(omega)*sin(phi)*cos(kappa)+cos(omega)*sin(kappa);
    m13 = -cos(omega)*sin(phi)*cos(kappa)+sin(omega)*sin(kappa);
    m21 = -cos(phi)*sin(kappa);
    m22 = -sin(omega)*sin(phi)*sin(kappa)+cos(omega)*cos(kappa);
    m23 = cos(omega)*sin(phi)*sin(kappa)+sin(omega)*cos(kappa);
    m31 = sin(phi);
    m32 = -sin(omega)*cos(phi);
    m33 = cos(omega)*cos(phi);
    [y,x] = np.mgrid[0:h,0:w]
    XA = (A*x)+(B*y)+C;
    YA = (D*x)+(E*y)+F;
    ZA = dsm_arr
    xa = x0 - f*((m11*(XA-XL)+m12*(YA-YL)+m13*(ZA-ZL))/(m31*(XA-XL)+m32*(YA-YL)+m33*(ZA-ZL)));
    ya = y0 + f*((m21*(XA-XL)+m22*(YA-YL)+m23*(ZA-ZL))/(m31*(XA-XL)+m32*(YA-YL)+m33*(ZA-ZL)));
    xa = np.around(xa).astype(int)
    ya = np.around(ya).astype(int)
    print(xa.shape)
    #True Ortho Generation
    loc = np.where((xa<0)|(xa>=img_w)|(ya<0)|(ya>=img_h)|(dsm_arr==-10000))
    xa[loc] = 0
    ya[loc] = 0
    true_ortho = np.zeros((h,w,3))
    true_ortho[:,:,0] = img[ya,xa,0]
    true_ortho[:,:,1] = img[ya,xa,1]
    true_ortho[:,:,2] = img[ya,xa,2]
    
    true_ortho[loc[0],loc[1],0] = 0
    true_ortho[loc[0],loc[1],1] = 0
    true_ortho[loc[0],loc[1],2] = 0
    return true_ortho
    
def write_trueortho(true_ortho,img_name):
    path = 'True_ortho/'+ 'TrueOrtho_'+ img_name
    cv2.imwrite(path,true_ortho)
    
"""
Read Pix4d calibrated EX camera parameters
imageName X Y Z Omega Phi Kappa 
X_sigma Y_sigma Z_sigma Omega_sigma Phi_sigma Kappa_sigma
"""

#interio
if __name__ == "__main__":
    f = 3803.28260713083182054106;
    x0 = 2471.84341749838540636119;
    y0 = 1653.25150608682383790438;
    
    df = pd.read_csv('ntu_pix4d_0212_calibrated_external_camera_parameters.txt',sep=' ')
    img_name = df['imageName'].values
    EX = np.array([df['X'].values,df['Y'].values,df['Z'].values,
          df['Omega'].values,df['Phi'].values,df['Kappa'].values]).T
                   
    affine_par = np.loadtxt('ntu_pix4d_0212_dsm.tfw')
    
    for i in range(len(img_name)):
        EX_par = EX[i]
        img_N = img_name[i]
        angle_matrix,nadir_row,nadir_col = dsm_angle('distortion_correct_dsm.tif',affine_par,EX_par)
        
        dsm_arr_occlude = occluded('distortion_correct_dsm.tif',angle_matrix,nadir_row,nadir_col)
        
        true_ortho = ortho_gen(img_N,dsm_arr_occlude,f,x0,y0,affine_par,EX_par)
        
        write_trueortho(true_ortho,img_N)