#!/usr/bin/env python
# -*- coding: utf-8 -*-

from osgeo import gdal
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import spatial
import cv2
from scipy import ndimage

def genDSM(PC_data,h,w,n,ground_height,affine_par,upper_bd_par):
    [A,D,B,E,C,F] = np.loadtxt(affine_par)
    
    point_cloud = pd.read_csv(PC_data,names=['X', 'Y', 'Z', 'R','G','B'],delim_whitespace=True)
    
    X = point_cloud.X.copy()
    Y = point_cloud.Y.copy()      # Inverse the Y-axis
    Z = point_cloud.Z.copy()
    
    del point_cloud
    #Dsm_arr = np.zeros((h, w))
    # cKDtree
    tree = spatial.cKDTree(list(zip(Y, X)))
    #Affine Trans
    xv, yv = np.meshgrid(np.linspace(1,w,w),np.linspace(1,h,h))
    XA = A*xv+B*yv+C
    YA = D*xv+E*yv+F
    pts = np.dstack((YA,XA))
    
    dis, loc = tree.query(pts, k=n ,distance_upper_bound=upper_bd_par)
   #mask = np.where(np.sum(np.isinf(dis),axis=2)!=n)
    del xv,yv
    
    #Dsm_arr = Z[loc[:,:,0].ravel()].reshape(h,w)
    #Dsm array operation
    Dsm_arr = Z[loc[:,:,:].ravel()].values.reshape(-1, n).max(axis=1).reshape(h,w)
    Dsm_arr[np.isnan(Dsm_arr)] = ground_height  #ground height
    Dsm_arr=np.float32(Dsm_arr)
    
    return Dsm_arr
    
def genDSM_building(BD_footprint,PC_data,h,w,n,ground_height,affine_par,upper_bd_par):
    
    [A,D,B,E,C,F] = np.loadtxt(affine_par)
    BD_footprint = cv2.imread(BD_footprint,0)
    point_cloud = pd.read_csv(PC_data,names=['X', 'Y', 'Z', 'R','G','B'],delim_whitespace=True)
        
    X = point_cloud.X.copy()
    Y = point_cloud.Y.copy()      # Inverse the Y-axis
    Z = point_cloud.Z.copy()
        
    del point_cloud
    Dsm_arr = np.zeros((h, w))
        # cKDtree
    tree = spatial.cKDTree(list(zip(Y, X)))
        #Affine Trans
    yv, xv = np.where(BD_footprint>127)
    XA = A*xv+B*yv+C
    YA = D*xv+E*yv+F
    pts = np.dstack((YA,XA))
    dis, loc = tree.query(pts, k=n ,distance_upper_bound=upper_bd_par)
    #building use max
    Dsm_arr[yv,xv] = Z[loc[:,:,:].ravel()].values.reshape(-1, n).max(axis=1)
    Dsm_arr=np.float32(Dsm_arr)
    
    return Dsm_arr
    
def genDSM_ground(BD_footprint,PC_data,h,w,n,ground_height,affine_par,upper_bd_par):
    
    [A,D,B,E,C,F] = np.loadtxt(affine_par)
    BD_footprint = cv2.imread(BD_footprint,0)
    point_cloud = pd.read_csv(PC_data,names=['X', 'Y', 'Z', 'R','G','B'],delim_whitespace=True)
        
    X = point_cloud.X.copy()
    Y = point_cloud.Y.copy()      # Inverse the Y-axis
    Z = point_cloud.Z.copy()
        
    del point_cloud
    Dsm_arr = np.zeros((h, w))
        # cKDtree
    tree = spatial.cKDTree(list(zip(Y, X)))
        #Affine Trans
    #yv, xv = np.where(BD_footprint==0)
    yv, xv = np.where(BD_footprint<=127)
    XA = A*xv+B*yv+C
    YA = D*xv+E*yv+F
    pts = np.dstack((YA,XA))
    
    dis, loc = tree.query(pts, k=n ,distance_upper_bound=upper_bd_par)
    #ground use min or max
    Dsm_arr[yv,xv] = Z[loc[:,:,:].ravel()].values.reshape(-1, n).max(axis=1)
    Dsm_arr=np.float32(Dsm_arr)
    
    return Dsm_arr

def Dsm_median(Dsm_arr,mask):
    Dsm_arr = ndimage.filters.median_filter(Dsm_arr,mask)
    
    return Dsm_arr

def Dsm_gaussian(Dsm_arr,mask):
    Dsm_arr = ndimage.filters.gaussian_filter(Dsm_arr,mask, mode='nearest')
    
    return Dsm_arr
    
def Dsm_bilatera(Dsm_arr,mask,n):    
    Dsm_arr = cv2.bilateralFilter(Dsm_arr,mask,n,n)
    
    return Dsm_arr
    
#show Dsm

def Dsm_show(DSM_arr):
    plt.imshow(DSM_arr,cmap = 'gray')
"""ds = gdal.Open( "ntu_pix4d_0212_dsm.tif" )
pix4d_dsm = np.array(ds.GetRasterBand(1).ReadAsArray())
pix4d_dsm[np.where(pix4d_dsm==-10000)] = ground_height"""
                   
          
def fill_empty(Dsm_arr,n):               #Dsm_arr & set up Empty cell' height
    loc = np.isnan(Dsm_arr)
    Dsm_arr[loc] = n
    return Dsm_arr
#save to Geotiff
def arr2Raster(data_name,Dsm_arr):
    [h,w] = Dsm_arr.shape
    driver = gdal.GetDriverByName('GTiff')
    dataset = driver.Create(
            data_name, w, h, 1, gdal.GDT_Float32)
    dataset.GetRasterBand(1).WriteArray(Dsm_arr[:, :])
    dataset.FlushCache()

def combine_layer(Dsm_building,Dsm_ground):
    loc = np.where(Dsm_building == 0)
    Dsm_building[loc] = Dsm_ground[loc]
    return Dsm_building
    
def main():
    """
    Dsm_arr = genDSM('test.xyz',7118,4186,
                     3,-300,'ntu_pix4d_0212_dsm.tfw',1)
    Dsm_arr = Dsm_median(Dsm_arr,13)
    #Dsm_arr = Dsm_gaussian(Dsm_arr,3)
    Dsm_arr = Dsm_bilatera(Dsm_arr,19,400)
    DSM_show(Dsm_arr)
    arr2Raster('selfmade_dsm.tif',Dsm_arr)
    """
    Dsm_building = genDSM_building('final_building_ortho.jpg','PC_building.xyz',7118,4186,
                     3,-300,'ntu_pix4d_0212_dsm.tfw',3)
    Dsm_building = Dsm_median(Dsm_building,3)
    #Dsm_building = Dsm_bilatera(Dsm_building,13,250)
    #Dsm_show(Dsm_building)
    Dsm_ground = genDSM_ground('final_building_ortho.jpg','PC_ground.xyz',7118,4186,
                     3,-300,'ntu_pix4d_0212_dsm.tfw',3)
    #
    Dsm_ground = Dsm_median(Dsm_ground,3)
    #Dsm_ground = Dsm_bilatera(Dsm_ground,13,400)
    #Dsm_show(Dsm_ground)
    
    
    final_Dsm = combine_layer(Dsm_building,Dsm_ground)
    final_Dsm = fill_empty(final_Dsm,-300)
    final_Dsm = Dsm_bilatera(final_Dsm,13,200)
    Dsm_show(final_Dsm)
    
    #arr2Raster('distortion_correct_dsm.tif',final_Dsm)
    
if __name__ == "__main__":
    main()