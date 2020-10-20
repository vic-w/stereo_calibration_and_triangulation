clc()
%xL = [imagePoints(:,1,1,1),imagePoints(:,2,1,1)]';
xL = [[imagePoints(1,1,1,1)]; [imagePoints(1,2,1,1)]];
xL
size(xL)
%xR = [imagePoints(:,1,1,2),imagePoints(:,2,1,2)]';
xR = [[imagePoints(1,1,1,2)]; [imagePoints(1,2,1,2)]];
xR
size(xL)

om = -rodrigues(stereoParams.RotationOfCamera2)
T = stereoParams.TranslationOfCamera2'
fc_left = stereoParams.CameraParameters1.FocalLength
cc_left = stereoParams.CameraParameters1.PrincipalPoint
%disp(stereoParams.CameraParameters1.RadialDistortion)
kc_left = [0.160352473905523;-0.573232847348061;-0.001486627485630;-5.926624737329299e-04;0.738723845547142]
alpha_c_left = 0
fc_right = stereoParams.CameraParameters2.FocalLength
cc_right = stereoParams.CameraParameters2.PrincipalPoint
kc_right = [-0.452374045717689;0.299920948247580;3.343441749612970e-04;-4.344534758797806e-04;-0.141821065338657]
alpha_c_right = 0


disp(["xL size: ",size(xL)])
disp(["xL size: ",size(xR)])
[XL,XR,ERROR] = stereo_triangulation(xL,xR,om,T,fc_left,cc_left,kc_left,alpha_c_left,fc_right,cc_right,kc_right,alpha_c_right)

XL
ERROR
%XL(:,1:69)-XL(:,2:70)