function K= calc_intrinsic_camera_matrix(cam_conf)
%Calculation of intrinsic camer matrix
%   
% input
%   cam_conf: Configuration for camera (struct)

K = [cam_conf.fx      0                 cam_conf.cx;...
        0             cam_conf.fy       cam_conf.cy;...
        0             0                 1];
end

