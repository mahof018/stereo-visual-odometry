function T12 = calc_transformation_K12(stereo_info,cam_res)
%   Calcualtion of transformation matrix T12 between left and right camera
% 
%   convert rodrigues rotation to roll,pitch,yaw angles
%   
%   !!!Matlab calcualtes in ZYX Frame, but the given Camera is XYZ
%   aligned!!!
%   -roll in Z ist the RX Rotation
%   -pitch in Y is the CV Rotation
%   -yaw in X is the RZ Rotation
%   -rotation vector /2 leads to the correct rotation , compare with Python openCV: cv2.Rodrigues([CV,RZ,RX])
%
%   input
%       stereo_info:    translation and rotation between left and right
%                       cam (struct)
%       cam_res:        used resolution (string)

%% Calculate transformation matrix
% Ectract Rotation with specific resolution
r = [stereo_info.("RX_" + cam_res), stereo_info.("CV_" + cam_res), stereo_info.("RZ_" + cam_res)] / 2;
% convert rodrigues to roll,pitch,yaw
[roll,pitch,yaw] = rod2angle(r, 'ZYX');
% combine translation and rotation
T21 =transl(-stereo_info.Baseline/1000, 0, 0)*trotz(roll)*trotx(pitch)*troty(yaw);
T12 = inv(T21);
end

