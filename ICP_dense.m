% This script calculates the trajectory of the left camera from the rosbag
% "testdrive_2022-10-25-09-01-50.bag" with dense stereo-visual odometry using the
% density data as a starting point.
%
% Algorithm: Triangulation and Iterative Closest Point
%
% You can adjust the init values, which function as filter values, to get a
% different output.
%
% Output: result/dense_icp_matlab.mat file
%               - T_complete_icp:   Calculated transformation matrices between left camera frame
%                                   i and i-1
%               - not_calculated:   Indices of uncalculated transformations
%                                   matrices due to warnings in algorithm
clear all;
close all;
clc;
%% init values
dist_thresh = 4; % used thresh  for the farthest allowed distance between left camera and calculated point
gridSize = 0.1; % Value used to downsample the point cloud before using ICP.
max_iter = 200; % max iterations for ICP

%% load rosbag
% bag = rosbag("testdrive_2022-10-25-09-01-50.bag");
% bagInfo = rosbag("info","testdrive_2022-10-25-09-01-50.bag");
bag = rosbag("shrink_version_testdrive_2022-10-25-09-01-50.bag");
bagInfo = rosbag("info","shrink_version_testdrive_2022-10-25-09-01-50.bag");

%bag data for dense stereo-visual odometry
bag_img_disparity = select(bag,"Topic","/zedm/zed_node/disparity/disparity_image");
%% Camera Parameters for triangulation
b = -0.0629661;
%calculated with given point_cloud and disparity from rosbag
f = 353.0741;
u0 = 321.8689;
v0 = 178.9860;
%% calculate trajectory
not_calc = [];
for i = 1:bag_img_disparity.NumMessages
        %Read disparity message
        dispMsg = readMessages(bag_img_disparity , i);
        disparityData = readImage(dispMsg{1}.Image);

        % calculate Point Cloud via triangulation
        [col,row] = find(ones(360,640));
        disp_flat = reshape(disparityData, 1, []);
        valid_indices= ~isnan(disp_flat)&~isinf(disp_flat);
        P_cur = [];
        P_cur(1,:) = abs((b*f)./disp_flat);
        P_cur(2,:) = -b* (row'-u0) ./ disp_flat ;
        P_cur(3,:)= -b* (col'-v0) ./ disp_flat ;
        P_cur(:,~valid_indices)=[];


        %remove Points which are to far away
        distsToRemove = abs(P_cur(1,:)) > dist_thresh;
        P_cur(:, distsToRemove) = [];
        %remove z points which are smaller than -0.2
        distsToRemove = P_cur(3,:) < -0.2;
        P_cur(:, distsToRemove) = [];
        if i>=2
            try
                %convert to matlab Point Cloud
                P_cur_cloud = pointCloud(P_cur');
                P_prev_cloud = pointCloud(P_prev');
                
                %denoise point cloud
                P_cur_cloud = pcdenoise(P_cur_cloud,PreserveStructure=true);
                P_prev_cloud = pcdenoise(P_prev_cloud,PreserveStructure=true);

                %downsample point cloud
                P_cur_cloud = pcdownsample(P_cur_cloud,gridAverage=gridSize);
                P_prev_cloud = pcdownsample(P_prev_cloud,gridAverage=gridSize);
                
                %Iterative closest point algorithm
                T = pcregistericp(P_prev_cloud ,P_cur_cloud,Metric="pointToPoint",Maxiterations = max_iter);
                
                %save current transformation matrix
                T_complete_icp{i}=T;
            catch
                % If there is a warning, just save the transformation matrix
                % from the previous frame
                not_calc(j)=i
                T_complete_icp{i}=T;
                j=j+1
            end
        end
        %save current Point cloud as previous Point cloud for the next
        %iteration
        P_prev = P_cur;
end 

%% Save transformation matrices
save('results/dense_icp_matlab.mat', 'T_complete_icp', 'not_calc');