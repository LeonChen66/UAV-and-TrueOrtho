function Dsm_array = dsm_angle(dsm_name,img_name,XL,YL,ZL,omega,phi,kappa,A,B,C,D,E,F,f,x0,y0)
% Read DSM
[X,map] = imread(dsm_name);
img = imread(img_name{1});
Dsm_array = X;

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

[h,w] = size(Dsm_array);
[img_h,img_w] = size(img);

m11 = cos(phi)*cos(kappa);
m12 = sin(omega)*sin(phi)*cos(kappa)+cos(omega)*sin(kappa);
m13 = -cos(omega)*sin(phi)*cos(kappa)+sin(omega)*sin(kappa);
m21 = -cos(phi)*sin(kappa);
m22 = -sin(omega)*sin(phi)*sin(kappa)+cos(omega)*cos(kappa);
m23 = cos(omega)*sin(phi)*sin(kappa)+sin(omega)*cos(kappa);
m31 = sin(phi);
m32 = -sin(omega)*cos(phi);
m33 = cos(omega)*cos(phi);

%dsm coordinate to photo coordinate in order to find nidir 
dsm2photo_ya = zeros(h,w);
dsm2photo_xa = zeros(h,w);
dsm2photonadir = zeros(h,w);
%{
for i = 1:h
    for j = 1:w
        XA = A*j+B*i+C;
        YA = D*j+E*i+F;
        ZA = Dsm_array(i,j); 
        xa = x0 - f*((m11*(XA-XL)+m12*(YA-YL)+m13*(ZA-ZL))/(m31*(XA-XL)+m32*(YA-YL)+m33*(ZA-ZL)));
        ya = y0 + f*((m21*(XA-XL)+m22*(YA-YL)+m23*(ZA-ZL))/(m31*(XA-XL)+m32*(YA-YL)+m33*(ZA-ZL)));
        if xa>0 && xa<=img_w && ya>0 && ya<=img_h
             dsm2photonadir(i,j) = (ya-y0)^2+(xa-x0)^2;
        else
            dsm2photonadir(i,j) = 10000000;
        end
    end
end
%}
nadir = inv([A B;D E])*[XL-C;YL-F]
%minMatrix = min(dsm2photonadir(:));
%[nadir_row,nadir_col] = find(dsm2photonadir==minMatrix);
nadir_row = round(nadir(2));
nadir_col = round(nadir(1));

%True Ortho Angle 

pixel_Angle = zeros(h,w);
X_nadir = round(nadir_col)*A+round(nadir_row)*B+C;
Y_nadir = round(nadir_col)*D+round(nadir_row)*E+F;
Z_nadir = Dsm_array(round(nadir_row),round(nadir_col));
vector_B = [X_nadir-XL Y_nadir-YL Z_nadir-ZL]';

%Calculate each cell's Angle
[x,y] = meshgrid(1:w,1:h);
XA = (A*x)+(B*y)+C;
YA = (D*x)+(E*y)+F;
ZA = Dsm_array;
vector_X = XA-XL;
vector_Y = YA-YL;
vector_Z = ZA-ZL;
%vector_A = cat(3,vector_X,vector_Y,vector_Z);

vector_dot = (vector_X*vector_B(1)+vector_Y*vector_B(2)+vector_Z*vector_B(3))./(sqrt((vector_X).^2+(vector_Y).^2+(vector_Z).^2)*norm(vector_B));
pixel_Angle = acosd(vector_dot);

pixel_Angle(Dsm_array==-10000) = -10000;  %Check if pixel_Angle on DSM is -10000
pixel_Angle = real(pixel_Angle);

% occluded detection

global occlude_area
occlude_area = zeros(h,w);
theta = linspace(0,2*pi,28000);
r = sqrt((h)^2+(w)^2);
x = r*cos(theta)+nadir_col;
x(x>w) = w;
x(x<1) = 1;
y = r*sin(theta)+nadir_row;
y(y>h)=h;
y(y<1)=1;
%{
for i = 1:28000
    if x(i) == w 
        test_x = linspace(nadir_col,x(i),x(i)-nadir_col+1);
        test_y = linspace(nadir_row,y(i),x(i)-nadir_col+1);
        test_zip = [round(test_y'),round(test_x')];
        sort_array(test_zip,pixel_Angle,test_x,test_y);
    elseif x(i)==1
        test_x = linspace(nadir_col,x(i),abs(x(i)-nadir_col)+1);
        test_y = linspace(nadir_row,y(i),abs(x(i)-nadir_col)+1);
        test_zip = [round(test_y'),round(test_x')];
        sort_array(test_zip,pixel_Angle,test_x,test_y);
    elseif y(i)==1
        test_x = linspace(nadir_col,x(i),abs(y(i)-nadir_row)+1);
        test_y = linspace(nadir_row,y(i),abs(y(i)-nadir_row)+1);
        test_zip = [round(test_y'),round(test_x')];
        sort_array(test_zip,pixel_Angle,test_x,test_y);
    elseif y(i)==h 
        test_x = linspace(nadir_col,x(i),y(i)-nadir_row+1);
        test_y = linspace(nadir_row,y(i),y(i)-nadir_row+1);
        test_zip = [round(test_y'),round(test_x')];
        sort_array(test_zip,pixel_Angle,test_x,test_y);
    end
end
%}
j = 1;
for i = 1:w
    if abs(nadir_col-i) >= abs(nadir_row-j)
        test_x = linspace(nadir_col,i,abs(i-nadir_col)+1);
        test_y = linspace(nadir_row,j,abs(i-nadir_col)+1);
        test_zip = [round(test_y'),round(test_x')];
        sort_array(test_zip,pixel_Angle,test_x,test_y);
    else
        test_x = linspace(nadir_col,i,abs(j-nadir_row)+1);
        test_y = linspace(nadir_row,j,abs(j-nadir_row)+1);
        test_zip = [round(test_y'),round(test_x')];
        sort_array(test_zip,pixel_Angle,test_x,test_y);
    end
end

j = h;
for i = 1:w
    if abs(nadir_col-i) >= abs(nadir_row-j)
        test_x = linspace(nadir_col,i,abs(i-nadir_col)+1);
        test_y = linspace(nadir_row,j,abs(i-nadir_col)+1);
        test_zip = [round(test_y'),round(test_x')];
        sort_array(test_zip,pixel_Angle,test_x,test_y);
    else
        test_x = linspace(nadir_col,i,abs(j-nadir_row)+1);
        test_y = linspace(nadir_row,j,abs(j-nadir_row)+1);
        test_zip = [round(test_y'),round(test_x')];
        sort_array(test_zip,pixel_Angle,test_x,test_y);
    end
end

j = w;
for i = 1:h
    if abs(nadir_col-j) <= abs(nadir_row-i)
        test_x = linspace(nadir_col,j,abs(i-nadir_row)+1);
        test_y = linspace(nadir_row,i,abs(i-nadir_row)+1);
        test_zip = [round(test_y'),round(test_x')];
        sort_array(test_zip,pixel_Angle,test_x,test_y);
    else
        test_x = linspace(nadir_col,j,abs(j-nadir_col)+1);
        test_y = linspace(nadir_row,i,abs(j-nadir_col)+1);
        test_zip = [round(test_y'),round(test_x')];
        sort_array(test_zip,pixel_Angle,test_x,test_y);
    end
end

j = 1;
for i = 1:h
    if abs(nadir_col-j) <= abs(nadir_row-i)
        test_x = linspace(nadir_col,j,abs(i-nadir_row)+1);
        test_y = linspace(nadir_row,i,abs(i-nadir_row)+1);
        test_zip = [round(test_y'),round(test_x')];
        sort_array(test_zip,pixel_Angle,test_x,test_y);
    else
        test_x = linspace(nadir_col,j,abs(j-nadir_col)+1);
        test_y = linspace(nadir_row,i,abs(j-nadir_col)+1);
        test_zip = [round(test_y'),round(test_x')];
        sort_array(test_zip,pixel_Angle,test_x,test_y);
    end
end

Dsm_array(occlude_area==-10000)=-10000;
