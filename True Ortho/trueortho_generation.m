%interio
f = 3803.28260713083182054106;
x0 = 2471.84341749838540636119;
y0 = 1653.25150608682383790438;
global occlude_area
%read data
dsm_name = 'distortion_correct_dsm.tif';
[img_id,X,Y,Z,omega_array,phi_array,kappa_array,X_sigma,Y_sigma,Z_sigma,omega_sigma,phi_sigma,kappa_sigma] = textread('ntu_pix4d_0212_calibrated_external_camera_parameters.txt','%s%f%f%f%f%f%f%f%f%f%f%f%f','headerlines',1);
affine_trans=textread('ntu_pix4d_0212_dsm.tfw');  %A D B E C F
A = affine_trans(1);
D = affine_trans(2);
B = affine_trans(3);
E = affine_trans(4);
C = affine_trans(5);
F = affine_trans(6);


%for i = 15:length(img_id)
    i = 4;
    img_name = img_id(i);
    XL = X(i);
    YL = Y(i);
    ZL = Z(i);
    omega = deg2rad(omega_array(i));
    phi = deg2rad(phi_array(i));
    kappa = deg2rad(kappa_array(i));
    
    
    
    % Run data
    Dsm_array = dsm_angle(dsm_name,img_name,XL,YL,ZL,omega,phi,kappa,A,B,C,D,E,F,f,x0,y0);
    True_ortho_test(Dsm_array,img_name,XL,YL,ZL,omega,phi,kappa,A,B,C,D,E,F,f,x0,y0)
%end
