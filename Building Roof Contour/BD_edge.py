# -*- coding: utf-8 -*-
"""
Created on Mon May  8 15:00:07 2017

@author: Leon
"""
from osgeo import gdal
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import spatial
import cv2
from math import *
from scipy import ndimage

def PC2UAV(PC_data,h_max,h_min,h,w,*EX_par):  #EX_par = [XL,YL,ZL,omega,phi,kappa,f,x0,y0]
    point_cloud = pd.read_csv(PC_data,names=['X', 'Y', 'Z', 'R','G','B'],delim_whitespace=True)
    
    X = point_cloud.X.copy().values
    Y = point_cloud.Y.copy().values      # Inverse the Y-axis
    Z = point_cloud.Z.copy().values
    
    del point_cloud
    
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
    
    xa = np.around(xa)
    ya = np.around(ya)
    loc = np.where((xa<w)&(xa>0)&(ya<h)&(ya>0))
    
    xa = xa[loc]
    ya = ya[loc]
    xa = xa.astype(int)
    ya = ya.astype(int)
    PC_image = np.zeros((h,w))
    
    PC_image[ya,xa] = int(255)
    PC_image = np.uint8(PC_image)
    return PC_image
    
def PC_p_edge(edge_image,PC_image):
    # edge_image = cv2.imread(edge_image)
    # edge_image = cv2.cvtColor(edge_image, cv2.COLOR_BGR2GRAY)
    loc = np.where((edge_image>127)&(PC_image>130))
    print(len(loc[0]))
    contour_img = np.zeros(edge_image.shape, np.uint8)
    contour_img[loc] = 255
    #edge_image[loc] = 0
    return contour_img
    #return edge_image
    
def show_image(img):
    plt.imshow(img,cmap = 'gray')
    
def img_gaussian(img,mask):
    img = ndimage.filters.gaussian_filter(img,mask, mode='nearest')
    
    return img
    
def dilation(img,kernel_size,i):
    kernel = np.ones((kernel_size,kernel_size),np.uint8)
    dilation_img = cv2.dilate(img,kernel,iterations = i)
    return dilation_img
    
def erosion(img,kernel_size,i):
    kernel = np.ones((kernel_size,kernel_size),np.uint8)
    erosion_img = cv2.erode(img,kernel,iterations = i)
    return erosion_img

def closing(img,kernel_size):
    kernel = np.ones((kernel_size,kernel_size),np.uint8)
    closing = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    return closing

def opening(img,kernel_size):
    kernel = np.ones((kernel_size,kernel_size),np.uint8)
    opening = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    return opening

def img_fill(im_in,n):   # n = binary image threshold
    th, im_th = cv2.threshold(im_in, n, 255, cv2.THRESH_BINARY);
     
    # Copy the thresholded image.
    im_floodfill = im_th.copy()
     
    # Mask used to flood filling.
    # Notice the size needs to be 2 pixels than the image.
    h, w = im_th.shape[:2]
    mask = np.zeros((h+2, w+2), np.uint8)
     
    # Floodfill from point (0, 0)
    cv2.floodFill(im_floodfill, mask, (0,0), 255);
     
    # Invert floodfilled image
    im_floodfill_inv = cv2.bitwise_not(im_floodfill)
     
    # Combine the two images to get the foreground.
    fill_image = im_th | im_floodfill_inv
    
    return fill_image

def canny(img_name):
    img = cv2.imread(img_name,0)
    img = cv2.GaussianBlur(img, (5, 5), 0)
    canny = cv2.Canny(img, 1, 3)
    dst = cv2.bitwise_and(img, img, mask=canny)
    cv2.imwrite('canny.jpg',dst)
    return canny
    
if __name__=="__main__":
    #I.O.
    f = 3803.28260713083182054106;
    x0 = 2471.84341749838540636119;
    y0 = 1653.25150608682383790438;

    df = pd.read_csv('ntu_pix4d_0212_calibrated_external_camera_parameters.txt', sep=' ')
    img_name = df['imageName'].values
    EX = np.array([df['X'].values, df['Y'].values, df['Z'].values,
                   df['Omega'].values, df['Phi'].values, df['Kappa'].values]).T

    img = 'DSCF2098_1471837627895.JPG'

    index_image = np.where(img_name==img)             #get the index of the image

    EX_par = np.squeeze(EX[index_image])     #which photos EX
    EX_par = np.append(EX_par,[f,x0,y0])
    print(EX_par)
    PC_image = PC2UAV('ntu_pix4d_0212_group1_densified_point_cloud.xyz',-227,-233,3264,4896,*EX_par)
    #Gaussian blur
    PC = img_gaussian(PC_image,3)
    
    loc = np.where(PC>15)
    PC_image[loc] = 255
    PC_image = opening(PC_image,8)
    cv2.imwrite('PC.jpg',PC_image)
    #Canny
    canny(img)
    edge_img = cv2.imread('canny.jpg')
    edge_img = cv2.cvtColor(edge_img, cv2.COLOR_BGR2GRAY)
    contour_image = PC_p_edge(edge_img,PC_image)
    cv2.imwrite('contour.jpg',contour_image)
    
    contour_image = closing(contour_image,10)
    contour_image = dilation(contour_image,5,5)
    cv2.imwrite('dilation.jpg',contour_image)
    fill_img = img_fill(contour_image,127)
    fill_img = erosion(fill_img,5,3)
    #show_image(contour_image)
    cv2.imwrite('fill.jpg',fill_img)