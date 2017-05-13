# -*- coding: utf-8 -*-
"""
Created on Tue Apr 11 23:33:54 2017

@author: Leon
"""

import numpy as np
import pandas as pd
import cv2
from math import *

def PC_error_Eli(Dsm_name,img_name,height_max,height_min,*EX_par):
    point_cloud = pd.read_csv(Dsm_name,names=['X', 'Y', 'Z', 'R','G','B'],
            delim_whitespace=True)
    #img_ortho = cv2.imread('final_building_ortho.jpg',0)
    img = cv2.imread(img_name,0)
    [h,w] = img.shape

    #building height
    #Point Cloud
    XA = point_cloud.X.copy()
    YA = point_cloud.Y.copy()      
    ZA = point_cloud.Z.copy()
    R = point_cloud.R.copy()
    G = point_cloud.G.copy()
    B = point_cloud.B.copy()
    #colinear equation
    XL = EX_par[0];
    YL = EX_par[1];
    ZL = EX_par[2];
    omega = np.deg2rad(EX_par[3]);
    phi = np.deg2rad(EX_par[4]);
    kappa = np.deg2rad(EX_par[5]);
    f = EX_par[6];
    x0 = EX_par[7];
    y0 = EX_par[8];
    
    m11 = cos(phi)*cos(kappa);
    m12 = sin(omega)*sin(phi)*cos(kappa)+cos(omega)*sin(kappa);
    m13 = -cos(omega)*sin(phi)*cos(kappa)+sin(omega)*sin(kappa);
    m21 = -cos(phi)*sin(kappa);
    m22 = -sin(omega)*sin(phi)*sin(kappa)+cos(omega)*cos(kappa);
    m23 = cos(omega)*sin(phi)*sin(kappa)+sin(omega)*cos(kappa);
    m31 = sin(phi);
    m32 = -sin(omega)*cos(phi);
    m33 = cos(omega)*cos(phi);
    #Criteria
    Z = ZA.values
    xa = x0 - f*((m11*(XA-XL)+m12*(YA-YL)+m13*(ZA-ZL))/(m31*(XA-XL)+m32*(YA-YL)+m33*(ZA-ZL)));
    ya = y0 + f*((m21*(XA-XL)+m22*(YA-YL)+m23*(ZA-ZL))/(m31*(XA-XL)+m32*(YA-YL)+m33*(ZA-ZL)));
    xa = np.around(xa).values.astype(int)
    ya = np.around(ya).values.astype(int)
    pts = np.squeeze(np.dstack((xa,ya)))
    #first time sort if in UAV image
    loc_1 = np.where((pts[:,1]>0) & (pts[:,1]<h) &(pts[:,0]>0) &(pts[:,0]<w))
    pts = pts[loc_1]
    Z = ZA.values[loc_1]
    #backward projection
    loc_2 = np.where((img[pts[:,1],pts[:,0]]!=255)&(Z>height_min)&(Z<height_max))
    
    #mask = np.where(R.values[loc_1[loc_2]])

    mask = tuple([list(loc_1)[0][loc_2]])
    X = np.delete(XA.values,mask)
    Y = np.delete(YA.values,mask)
    Z = np.delete(ZA.values,mask)
    R = np.delete(R.values,mask)
    G = np.delete(G.values,mask)
    B = np.delete(B.values,mask)
    """
    X = XA.values[loc_1][loc_2]
    Y = YA.values[loc_1][loc_2]
    Z = ZA.values[loc_1][loc_2]
    R = R.values[loc_1][loc_2]
    G = G.values[loc_1][loc_2]
    B = B.values[loc_1][loc_2]
    """
    PC_data = np.squeeze(np.dstack((X,Y,Z,R,G,B)))
    
    return PC_data
    
def PC_error_mod2ground(Dsm_name,img_name,height_max,height_min,ground_height,*EX_par):
    point_cloud = pd.read_csv(Dsm_name,names=['X', 'Y', 'Z', 'R','G','B'],
            delim_whitespace=True)
    #img_ortho = cv2.imread('final_building_ortho.jpg',0)
    img = cv2.imread(img_name,0)
    [h,w] = img.shape

    #building height
    #Point Cloud
    XA = point_cloud.X.copy()
    YA = point_cloud.Y.copy()      
    ZA = point_cloud.Z.copy()
    R = point_cloud.R.copy()
    G = point_cloud.G.copy()
    B = point_cloud.B.copy()
    #colinear equation
    XL = EX_par[0];
    YL = EX_par[1];
    ZL = EX_par[2];
    omega = np.deg2rad(EX_par[3]);
    phi = np.deg2rad(EX_par[4]);
    kappa = np.deg2rad(EX_par[5]);
    f = EX_par[6];
    x0 = EX_par[7];
    y0 = EX_par[8];
    
    m11 = cos(phi)*cos(kappa);
    m12 = sin(omega)*sin(phi)*cos(kappa)+cos(omega)*sin(kappa);
    m13 = -cos(omega)*sin(phi)*cos(kappa)+sin(omega)*sin(kappa);
    m21 = -cos(phi)*sin(kappa);
    m22 = -sin(omega)*sin(phi)*sin(kappa)+cos(omega)*cos(kappa);
    m23 = cos(omega)*sin(phi)*sin(kappa)+sin(omega)*cos(kappa);
    m31 = sin(phi);
    m32 = -sin(omega)*cos(phi);
    m33 = cos(omega)*cos(phi);
    #Criteria
    Z = ZA.values
    xa = x0 - f*((m11*(XA-XL)+m12*(YA-YL)+m13*(ZA-ZL))/(m31*(XA-XL)+m32*(YA-YL)+m33*(ZA-ZL)));
    ya = y0 + f*((m21*(XA-XL)+m22*(YA-YL)+m23*(ZA-ZL))/(m31*(XA-XL)+m32*(YA-YL)+m33*(ZA-ZL)));
    xa = np.around(xa).values.astype(int)
    ya = np.around(ya).values.astype(int)
    pts = np.squeeze(np.dstack((xa,ya)))
    #first time sort if in UAV image
    loc_1 = np.where((pts[:,1]>0) & (pts[:,1]<h) &(pts[:,0]>0) &(pts[:,0]<w))
    pts = pts[loc_1]
    Z = ZA.values[loc_1]
    #backward projection
    loc_2 = np.where((img[pts[:,1],pts[:,0]]!=255)&(Z>height_min)&(Z<height_max))
    
    #mask = np.where(R.values[loc_1[loc_2]])

    mask = tuple([list(loc_1)[0][loc_2]])
    X = XA.values
    Y = YA.values
    Z = ZA.values
    R = R.values
    G = G.values
    B = B.values
    Z[mask] = ground_height

    PC_data = np.squeeze(np.dstack((X,Y,Z,R,G,B)))
    
    return PC_data
    
def PC_Er_ground(Dsm_name,img_name,height_max,height_min,*EX_par):
    point_cloud = pd.read_csv(Dsm_name,names=['X', 'Y', 'Z', 'R','G','B'],
            delim_whitespace=True)
    #img_ortho = cv2.imread('final_building_ortho.jpg',0)
    img = cv2.imread(img_name,0)
    [h,w] = img.shape
    #[h_ortho,w_ortho] = img_ortho.shape
    #building height
    
    
    #Point Cloud
    XA = point_cloud.X.copy()
    YA = point_cloud.Y.copy()      
    ZA = point_cloud.Z.copy()
    R = point_cloud.R.copy()
    G = point_cloud.G.copy()
    B = point_cloud.B.copy()
    #colinear equation
    XL = EX_par[0];
    YL = EX_par[1];
    ZL = EX_par[2];
    omega = np.deg2rad(EX_par[3]);
    phi = np.deg2rad(EX_par[4]);
    kappa = np.deg2rad(EX_par[5]);
    f = EX_par[6];
    x0 = EX_par[7];
    y0 = EX_par[8];
    
    m11 = cos(phi)*cos(kappa);
    m12 = sin(omega)*sin(phi)*cos(kappa)+cos(omega)*sin(kappa);
    m13 = -cos(omega)*sin(phi)*cos(kappa)+sin(omega)*sin(kappa);
    m21 = -cos(phi)*sin(kappa);
    m22 = -sin(omega)*sin(phi)*sin(kappa)+cos(omega)*cos(kappa);
    m23 = cos(omega)*sin(phi)*sin(kappa)+sin(omega)*cos(kappa);
    m31 = sin(phi);
    m32 = -sin(omega)*cos(phi);
    m33 = cos(omega)*cos(phi);
    #Criteria
    Z = ZA.values
    xa = x0 - f*((m11*(XA-XL)+m12*(YA-YL)+m13*(ZA-ZL))/(m31*(XA-XL)+m32*(YA-YL)+m33*(ZA-ZL)));
    ya = y0 + f*((m21*(XA-XL)+m22*(YA-YL)+m23*(ZA-ZL))/(m31*(XA-XL)+m32*(YA-YL)+m33*(ZA-ZL)));
    xa = np.around(xa).values.astype(int)
    ya = np.around(ya).values.astype(int)
    pts = np.squeeze(np.dstack((xa,ya)))
    #first time sort if in UAV image
    loc_1 = np.where((pts[:,1]>0) & (pts[:,1]<h) &(pts[:,0]>0) &(pts[:,0]<w))
    pts = pts[loc_1]
    Z = ZA.values[loc_1]
    #backward projection
    loc_2 = np.where((img[pts[:,1],pts[:,0]]==255)&(Z>height_min)&(Z<height_max))
    loc_3 = np.where((img[pts[:,1],pts[:,0]]!=255)&(Z>height_min)&(Z<height_max))
    #mask = np.where(R.values[loc_1[loc_2]])
    
    
    mask_building = tuple([list(loc_1)[0][loc_2]])
    mask_error = tuple([list(loc_1)[0][loc_3]])
    mask = tuple(np.hstack((mask_building,mask_error)))
    X = np.delete(XA.values,mask)
    Y = np.delete(YA.values,mask)
    Z = np.delete(ZA.values,mask)
    R = np.delete(R.values,mask)
    G = np.delete(G.values,mask)
    B = np.delete(B.values,mask)
    """
    X = XA.values[loc_1][loc_2]
    Y = YA.values[loc_1][loc_2]
    Z = ZA.values[loc_1][loc_2]
    R = R.values[loc_1][loc_2]
    G = G.values[loc_1][loc_2]
    B = B.values[loc_1][loc_2]
    """
    PC_ground = np.squeeze(np.dstack((X,Y,Z,R,G,B)))
    
    return PC_ground
    
def PC_Er_building(Dsm_name,img_name,height_max,height_min,*EX_par):
    point_cloud = pd.read_csv(Dsm_name,names=['X', 'Y', 'Z', 'R','G','B'],
            delim_whitespace=True)
    #img_ortho = cv2.imread('final_building_ortho.jpg',0)
    img = cv2.imread(img_name,0)
    [h,w] = img.shape
    #[h_ortho,w_ortho] = img_ortho.shape
    #building height
    
    
    #Point Cloud
    XA = point_cloud.X.copy()
    YA = point_cloud.Y.copy()      
    ZA = point_cloud.Z.copy()
    R = point_cloud.R.copy()
    G = point_cloud.G.copy()
    B = point_cloud.B.copy()
    #colinear equation
    XL = EX_par[0];
    YL = EX_par[1];
    ZL = EX_par[2];
    omega = np.deg2rad(EX_par[3]);
    phi = np.deg2rad(EX_par[4]);
    kappa = np.deg2rad(EX_par[5]);
    f = EX_par[6];
    x0 = EX_par[7];
    y0 = EX_par[8];
    
    m11 = cos(phi)*cos(kappa);
    m12 = sin(omega)*sin(phi)*cos(kappa)+cos(omega)*sin(kappa);
    m13 = -cos(omega)*sin(phi)*cos(kappa)+sin(omega)*sin(kappa);
    m21 = -cos(phi)*sin(kappa);
    m22 = -sin(omega)*sin(phi)*sin(kappa)+cos(omega)*cos(kappa);
    m23 = cos(omega)*sin(phi)*sin(kappa)+sin(omega)*cos(kappa);
    m31 = sin(phi);
    m32 = -sin(omega)*cos(phi);
    m33 = cos(omega)*cos(phi);
    #Criteria
    Z = ZA.values
    xa = x0 - f*((m11*(XA-XL)+m12*(YA-YL)+m13*(ZA-ZL))/(m31*(XA-XL)+m32*(YA-YL)+m33*(ZA-ZL)));
    ya = y0 + f*((m21*(XA-XL)+m22*(YA-YL)+m23*(ZA-ZL))/(m31*(XA-XL)+m32*(YA-YL)+m33*(ZA-ZL)));
    xa = np.around(xa).values.astype(int)
    ya = np.around(ya).values.astype(int)
    pts = np.squeeze(np.dstack((xa,ya)))
    #first time sort if in UAV image
    loc_1 = np.where((pts[:,1]>0) & (pts[:,1]<h) &(pts[:,0]>0) &(pts[:,0]<w))
    pts = pts[loc_1]
    Z = ZA.values[loc_1]
    #backward projection
    loc_2 = np.where((img[pts[:,1],pts[:,0]]==255)&(Z>height_min)&(Z<height_max))
    
    #mask = np.where(R.values[loc_1[loc_2]])
    """
    mask = tuple([list(loc_1)[0][loc_2]])
    X = np.delete(XA.values,mask)
    Y = np.delete(YA.values,mask)
    Z = np.delete(ZA.values,mask)
    R = np.delete(R.values,mask)
    G = np.delete(G.values,mask)
    B = np.delete(B.values,mask)
    """
    X = XA.values[loc_1][loc_2]
    Y = YA.values[loc_1][loc_2]
    Z = ZA.values[loc_1][loc_2]
    R = R.values[loc_1][loc_2]
    G = G.values[loc_1][loc_2]
    B = B.values[loc_1][loc_2]
   
    PC_building = np.squeeze(np.dstack((X,Y,Z,R,G,B)))
    
    return PC_building

def Sep_building(PC_data,height_max,height_min):
    loc = np.where((PC_data[:,2]<height_max)&(PC_data[:,2]>height_min))
    PC_building = PC_data[loc]

    return PC_building
    
def PC_2xyz(PC_data,xyz_name):
    np.savetxt(xyz_name,PC_data,delimiter=' ',
               fmt='%.2f %.2f %.2f %u %u %u')
    
if __name__ == "__main__":
    height_max = -222
    height_min = -234
    ground_height = -294
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
    """
    PC_data = PC_error_Eli('ntu_pix4d_0212_group1_densified_point_cloud.xyz'
                 ,'final_building.jpg',height_max,height_min,*EX_par)
    PC_2xyz(PC_data,'test_2.xyz')
    #PC_building = Sep_building(PC_data,height_max,height_min)
    PC_building = PC_Er_building('ntu_pix4d_0212_group1_densified_point_cloud.xyz'
                 ,'final_building.jpg',height_max,height_min,*EX_par)
    PC_2xyz(PC_building,'PC_building.xyz')
    PC_ground = PC_Er_ground('ntu_pix4d_0212_group1_densified_point_cloud.xyz'
                 ,'final_building.jpg',height_max,height_min,*EX_par)
    PC_2xyz(PC_ground,'PC_ground.xyz')
    """
    PC_data = PC_error_mod2ground('ntu_pix4d_0212_group1_densified_point_cloud.xyz'
                 ,'final_building.jpg',height_max,height_min,ground_height,*EX_par)
    
    PC_2xyz(PC_data,'Error_modification_PC.xyz')