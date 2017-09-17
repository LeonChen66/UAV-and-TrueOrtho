#!/usr/bin/env python
# -*- coding: utf-8 -*-
# from osgeo import gdal
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import spatial
import cv2
from osgeo import gdal

def get_space(data):

    tree = spatial.cKDTree(data[['X', 'Y', 'Z']])

    # The first column is the current point itself
    # and the second one is the nearest point
    dis, loc = tree.query(data[['X', 'Y', 'Z']], k=2)

    return np.mean(dis[:, 1])


def GEM_Dsm(data,h,w,n,upper_bd_par,affine_par):
    # Read basic point cloud data
    X = data.X.copy()
    Y = data.Y.copy()
    Z = data.Z.copy()
    C_R = data.C_R.copy()
    C_G = data.C_G.copy()
    C_B = data.C_B.copy()

    # Affine Par
    [A, D, B, E, C, F] = affine_par
    xv, yv = np.meshgrid(np.linspace(1, w, w), np.linspace(1, h, h))
    XA = A * xv + B * yv + C
    YA = D * xv + E * yv + F
    pts = np.dstack((YA, XA))

    # cKDtree
    tree = spatial.cKDTree(list(zip(Y, X)))
    dis, loc = tree.query(pts, k=n, distance_upper_bound=upper_bd_par)
    # Get the max Z
    idx = (Z[loc[:, :, :].ravel()].values.reshape(-1, n)).argmax(axis=1)
    num = np.arange(h*w)
    final_idx = tuple((num,idx))
    # Trans max Z index to color
    loc2 = loc.reshape(h*w, n)[final_idx]
    # Ortho
    Ortho = np.zeros((h, w, 3))
    # Ortho[:,:,2] = C_R[loc[:, :, 0].ravel()].values.reshape(h, w)
    # Ortho[:, :, 1] = C_G[loc[:, :, 0].ravel()].values.reshape(h, w)
    # Ortho[:, :, 0] = C_B[loc[:, :, 0].ravel()].values.reshape(h, w)
    Ortho[:,:,2] = C_R[loc2.ravel()].values.reshape(h, w)
    Ortho[:, :, 1] = C_G[loc2.ravel()].values.reshape(h, w)
    Ortho[:, :, 0] = C_B[loc2.ravel()].values.reshape(h, w)
    return Ortho


def array2Raster(result_arr,affine_par,dstFilename):
    # save arr to geotiff
    driver = gdal.GetDriverByName('GTiff')

    [A, D, B, E, C, F] = affine_par

    if result_arr.ndim == 2:
        [height,width] = result_arr.shape
        dataset = driver.Create(dstFilename, width, height, numBand, gdal.GDT_Float32)
        dataset.SetGeoTransform((C, A, B, F, D, E))
        dataset.GetRasterBand(1).WriteArray(array[:, :])
        #dataset.GetRasterBand(1).SetNoDataValue(0)

    elif result_arr.ndim == 3:
        [height,width,numBand] = result_arr.shape
        dataset = driver.Create(dstFilename, width, height, numBand, gdal.GDT_Float32)
        dataset.SetGeoTransform((C, A, B, F, D, E))
        for i in range(numBand):
            dataset.GetRasterBand(i + 1).WriteArray(result_arr[:, :, i])
            #dataset.GetRasterBand(i + 1).SetNoDataValue(-999.0)

    dataset.FlushCache()        # Write to disk



def main():
    data = pd.read_csv(
        'Leon_group1_densified_point_cloud.xyz',
        names=['X', 'Y', 'Z', 'C_R','C_G','C_B'],
        delim_whitespace=True)
    # Calculate Geotiff information
    Auto = True

    # If it is auto
    if Auto == True:
        # spacing could be changed
        spacing = 1.6*get_space(data)

        w = int((data.X.max() - data.X.min()) / spacing)
        h = int((data.Y.max() - data.Y.min()) / spacing)
        affine_par = [spacing,0,0,-spacing,data.X.min(),data.Y.max()]

    else:
        affine_name = ''
        affine_par = np.loadtxt(affine_name)   # input the affine name
        h = 1792
        w = 1053

    print(affine_par)
    print(h,w)
    # Generate DEM
    ortho = GEM_Dsm(data, h, w, 3, 0.15,affine_par)
    # save to tif
    ortho = ortho.astype(np.uint8)
    # ortho = cv2.medianBlur(ortho, 3)
    cv2.imwrite('ortho.tif',ortho)

    array2Raster(ortho,affine_par,'test.tif')

if __name__=="__main__":
    main()