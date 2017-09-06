# UAV-and-TrueOrtho
Master's Thesis

* Application of UAV photos generates True Ortho Image

* Modify the distortion of DSM result in distortion of builing edge in True Ortho image


1. Building Roof Contour

-- Recognize building roof contour and building roof by something like DSM but not so dense. With limited and appropriate upper bound.

2. Dsm to UAV_BD Roof Contour
- Recognize building roof contour and building roof by point cloud.

3. Edge Color
- Balance the color of Ortho photo.

4. Elimate PC Error
- By the building roof layer, modify the error point cloud to ground height.

5. PC to DSM
- Point cloud interpolates to DSM. 1) Original 2)divide into building and ground then interpolate with constrains.

6. T_ortho_py
- Generate True ortho photos in Python

7. True Ortho
- Generate True ortho photos in Matlab

8. UAV2TrueOrtho layer
- Transform the building roof layer on the UAV photos to the True Ortho (Dsm) layer to constrain the interpolation.
