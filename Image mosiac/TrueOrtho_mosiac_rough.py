# -*- coding: utf-8 -*-
"""
Spyder Editor

Moasic of TrueOrtho 
"""

import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
import glob
tt = glob.glob(' *.JPG ')


img_name_all = os.listdir(".")

img_list = []

img_list = [s for s in img_name_all if ".JPG" in s]

T_Ortho = cv2.imread(img_list[4])

for img_name in img_list:
    img = cv2.imread(img_name)
    gray_image = cv2.cvtColor(T_Ortho, cv2.COLOR_BGR2GRAY)
#    loc = np.where((T_Ortho[:,:,0]<10)&(T_Ortho[:,:,1]<10)&(T_Ortho[:,:,2]<10))
    loc = np.where(gray_image<15)
    T_Ortho[loc] = img[loc]

T_Ortho = cv2.medianBlur(T_Ortho,3)
cv2.imwrite('T_Ortho.tif',T_Ortho)