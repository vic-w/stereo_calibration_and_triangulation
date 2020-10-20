function [XL,XR,ERR] = stereo_triangulation(xL,xR,om,T,fc_left,cc_left,kc_left,alpha_c_left,fc_right,cc_right,kc_right,alpha_c_right)

% [XL,XR] = stereo_triangulation(xL,xR,om,T,fc_left,cc_left,kc_left,alpha_c_left,fc_right,cc_right,kc_right,alpha_c_right),
%
% Function that computes the position of a set on N points given the left and right image projections.
% The cameras are assumed to be calibrated, intrinsically, and extrinsically.
%
% Input:
%           xL: 2xN matrix of pixel coordinates in the left image
%           xR: 2xN matrix of pixel coordinates in the right image
%           om,T: rotation vector and translation vector between right and left cameras (output of stereo calibration)
%           fc_left,cc_left,...: intrinsic parameters of the left camera  (output of stereo calibration)
%           fc_right,cc_right,...: intrinsic parameters of the right camera (output of stereo calibration)
%
% Output:
%
%           XL: 3xN matrix of coordinates of the points in the left camera reference frame
%           XR: 3xN matrix of coordinates of the points in the right camera reference frame
%
% Note: XR and XL are related to each other through the rigid motion equation: XR = R * XL + T, where R = rodrigues(om)
% For more information, visit http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/example5.html
%
%
% (c) Jean-Yves Bouguet - Intel Corporation - April 9th, 2003



%--- Normalize the image projection according to the intrinsic parameters of the left and right cameras
xt = normalize_pixel(xL,fc_left,cc_left,kc_left,alpha_c_left);
xtt = normalize_pixel(xR,fc_right,cc_right,kc_right,alpha_c_right);


%--- Extend the normalized projections in homogeneous coordinates
xt = [xt;ones(1,size(xt,2))];
size(xt)
xtt = [xtt;ones(1,size(xtt,2))];

disp(["xt size: ",size(xt)])
disp(["xtt size: ",size(xtt)])

%--- Number of points:
N = size(xt,2);
disp(["N = ",N])

%--- Rotation matrix corresponding to the rigid motion between left and right cameras:
R = rodrigues(om)
%R=om
disp(["R size: ",size(R)])


%--- Triangulation of the rays in 3D space:
%这里我改成点乘先看看
u = R * xt;

n_xt2 = dot(xt,xt);
n_xtt2 = dot(xtt,xtt);
disp(["n_xt2 size: ",size(n_xt2)])
disp(["n_xtt2 size: ",size(n_xtt2)])

T_vect = repmat(T, [1 N])

DD = n_xt2 .* n_xtt2 - dot(u,xtt).^2;
disp(["T_vect size: ",size(T_vect)])
disp(["DD size: ",size(DD)])


dot_uT = dot(u,T_vect);
dot_xttT = dot(xtt,T_vect);
dot_xttu = dot(u,xtt);

disp(["dot_uT size: ",size(dot_uT)])
disp(["dot_xttT size: ",size(dot_xttT)])
disp(["dot_xttu size: ",size(dot_xttu)])


NN1 = dot_xttu.*dot_xttT - n_xtt2 .* dot_uT;
NN2 = n_xt2.*dot_xttT - dot_uT.*dot_xttu;
disp(["NN1 size: ",size(NN1)])
disp(["NN2 size: ",size(NN2)])


Zt = NN1./DD;
Ztt = NN2./DD;
disp(["Zt size: ",size(Zt)])
disp(["Ztt size: ",size(Ztt)])


X1 = xt .* repmat(Zt,[3 1]);
X2 = R'*(xtt.*repmat(Ztt,[3,1])  - T_vect);
disp(["X1 size: ",size(X1)])
disp(["X2 size: ",size(X2)])


%--- Left coordinates:
XL = 1/2 * (X1 + X2);

%--- Right coordinates:
%这里我改成点乘先看看
XR = R*XL + T_vect;

ERR = (X1-X2)

disp(["XL size: ",size(XL)])
XL
disp(["XR size: ",size(XR)])
XR
end
