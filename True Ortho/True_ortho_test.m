function [] = True_ortho_test(Dsm_array,img_name,XL,YL,ZL,omega,phi,kappa,A,B,C,D,E,F,f,x0,y0)

%[X,map] = imread(pixel_Angle{1});
img = imread(img_name{1});
%Dsm_array = pixel_Angle;
%dsm to geotiff
%{
A = 0.08146;
B = 0;
D = 0;
E = -0.08146;
C = -225.44996;
F = 303.80724;
%}
%Exterios

%XL = -44.565271;
%YL = 17.775985;
%ZL = 14.067157;
%omega = deg2rad(omega);
%phi = deg2rad(phi);
%kappa = deg2rad(kappa);
%f = 3803.28260713083182054106;
%x0 = 2471.84341749838540636119;
%y0 = 1653.25150608682383790438;

m11 = cos(phi)*cos(kappa);
m12 = sin(omega)*sin(phi)*cos(kappa)+cos(omega)*sin(kappa);
m13 = -cos(omega)*sin(phi)*cos(kappa)+sin(omega)*sin(kappa);
m21 = -cos(phi)*sin(kappa);
m22 = -sin(omega)*sin(phi)*sin(kappa)+cos(omega)*cos(kappa);
m23 = cos(omega)*sin(phi)*sin(kappa)+sin(omega)*cos(kappa);
m31 = sin(phi);
m32 = -sin(omega)*cos(phi);
m33 = cos(omega)*cos(phi);

[h,w] = size(Dsm_array);
true_ortho = zeros(h,w,3);
[img_h,img_w,rgb] = size(img);

[x,y] = meshgrid(1:w,1:h);
%True Ortho Generation
XA = (A*x)+(B*y)+C;
YA = (D*x)+(E*y)+F;
        ZA = Dsm_array;
        xa = x0 - f.*((m11.*(XA-XL)+m12.*(YA-YL)+m13.*(ZA-ZL))./(m31.*(XA-XL)+m32.*(YA-YL)+m33.*(ZA-ZL)));
        ya = y0 + f.*((m21.*(XA-XL)+m22.*(YA-YL)+m23.*(ZA-ZL))./(m31.*(XA-XL)+m32.*(YA-YL)+m33.*(ZA-ZL)));
        %ya = x0 -f*((m21*(XA-XL)+m22*(YA-YL)+m23*(ZA-ZL))/(m31*(XA-XL)+m32*(YA-YL)+m33*(ZA-ZL)));
        %xa = y0 -f*((m11*(XA-XL)+m12*(YA-YL)+m13*(ZA-ZL))/(m31*(XA-XL)+m32*(YA-YL)+m33*(ZA-ZL)));
        %xa = -(xa-3264/2);
        %ya = ya + 4896/2;
        %ya = -ya;
xa = round(xa);
ya = round(ya);
num = find(xa>0 & xa<=img_w & ya>0 & ya<=img_h & Dsm_array~=-10000);

true_ortho_R = zeros(h,w);
true_ortho_G = zeros(h,w);
true_ortho_B = zeros(h,w);

R = img(1:img_h,1:img_w,1);
G = img(1:img_h,1:img_w,2);
B = img(1:img_h,1:img_w,3);

true_ortho_R(num) = R(sub2ind(size(img),ya(num),xa(num)));
true_ortho_G(num) = G(sub2ind(size(img),ya(num),xa(num)));
true_ortho_B(num) = B(sub2ind(size(img),ya(num),xa(num)));

true_ortho(1:h,1:w,1) = true_ortho_R;
true_ortho(1:h,1:w,2) = true_ortho_G;
true_ortho(1:h,1:w,3) = true_ortho_B;


%name1 = 'True_ortho_test/';

name_last = sprintf('True_ortho_test/ortho_test_%s',img_name{1});

true_ortho = uint8(true_ortho);
imwrite(true_ortho,name_last);

end