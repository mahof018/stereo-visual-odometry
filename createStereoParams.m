function [stereoParams, cp1, cp2] =createStereoParams(K1,K2,k1,k2,w,h,T12)
%% convert intrinsics and stereo pose to matlab stereoParams
%
%   input
%       Ki =[]                  camera intrinsic matrix
%                                       |fx 0  cu|
%                                   K = |0  fy cv|
%                                       |0  0  1 |
%
%       ki =[r1,r2,p1,p2,r3]    radial (ri) and tangential (pi) distortion
%       w,h                     width, height of image
%       T12                     pose of cam2 wrt cam1
%
%

%% cameraParameters (1:left)
intrinsicMatrix =K1'; % matlab notation 
radialDistortion =k1([1,2,5]);
tangentialDistortion =k1([3,4]);

imageSize =[h, w];
    
cp1 = cameraParameters(...
        'ImageSize', imageSize, ...
        'IntrinsicMatrix',intrinsicMatrix,...
        'RadialDistortion',radialDistortion, ...
        'TangentialDistortion',tangentialDistortion); 

%% cameraParameters (2:right)
intrinsicMatrix =K2'; % matlab notation 
radialDistortion =k2([1,2,5]);
tangentialDistortion =k2([3,4]);

imageSize =[h, w];
    
cp2 = cameraParameters(...
        'ImageSize', imageSize, ...
        'IntrinsicMatrix',intrinsicMatrix,...
        'RadialDistortion',radialDistortion, ...
        'TangentialDistortion',tangentialDistortion); 

%% stereo
T21 =inv(T12);
[R, t] =tr2rt(T21); % tx <0
rotationOfCamera2 =R';  % transpose (different notation in MATLAB)
translationOfCamera2 =t' *1000;   % row vector, [m]->[mm]

% other direction...
% R = rotationOfCamera2';           % transpose (diffrent notation in MATLAB)
% t = translationOfCamera2' * 1e-3; % column vector, [mm]->[m]
% T21 =transl(t) * r2t(R);          % 1. translation 2. rotation
% T12 =inv(T21);

%% create parameter object
stereoParams = stereoParameters(...
    cp1, cp2,...
    rotationOfCamera2,translationOfCamera2);
