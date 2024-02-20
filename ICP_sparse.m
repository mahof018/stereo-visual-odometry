clear all;
close all;
clc;

%% init values
ransac_thresh  = 1e-2; %used thresh  for als ransac operations
vertical_thresh  = 0.75; %used thresh  for feature location after rectification in vertical direction
dist_thresh = 4; %used thresh  for the farthest allowed distance between left camera and calculated point
border_thresh = 30; % threshold for removing points which are near the image border
%% load rosbag
bag = rosbag("testdrive_2022-10-25-09-01-50.bag");
bagInfo = rosbag("info","testdrive_2022-10-25-09-01-50.bag");

%Groundtruth Pose
bag_imu = select(bag,"Topic",['/zedm/zed_node/pose']);
%Raw Stereo Images
bag_images = select(bag,"Topic","/zedm/zed_node/stereo_raw/image_raw_color");


%% load camera configurations
% Read the cam.conf file data
cam_conf_struct = load_camera_config("SN10028708.conf");
%save relevant ZED-Camera configurations for the given images
left_cam_conf = cam_conf_struct.LEFT_CAM_HD;
right_cam_conf = cam_conf_struct.RIGHT_CAM_HD;
stereo_info_conf = cam_conf_struct.STEREO;
%% load params for rectification and removing distortion
%load intrinsic camera data K1=left cam, K2 = right cam
K1 = calc_intrinsic_camera_matrix(left_cam_conf);
K2 = calc_intrinsic_camera_matrix(right_cam_conf);

width_img = 1280;
height_img = 720;

% Transformation Matrix T12 between left and rigth camera
T12 = calc_transformation_K12(stereo_info_conf,"HD");

%distortion parameters left and right cam
distortion_params_K1 = [left_cam_conf.k1,left_cam_conf.k2,left_cam_conf.p1,left_cam_conf.p2,left_cam_conf.k3];
distortion_params_K2 = [right_cam_conf.k1,right_cam_conf.k2,right_cam_conf.p1,right_cam_conf.p2,right_cam_conf.k3];

%create stereo_params
stereo_params = createStereoParams(K1,K2,distortion_params_K1,distortion_params_K2,width_img,height_img,T12);
imgMsg = readMessages(bag_images, 1);

% Convert ROS image message to MATLAB image
img = readImage(imgMsg{1});
% split img into left and right image
left_img = img(:,1:1280,:);
right_img = img(:,1281:end,:);

%get reprojection matrix
[left,right,reprojection_matrix] = rectifyStereoImages(left_img,right_img,stereo_params);

%get image size after rectification
[height_rect, width_rect,~] = size(left)
%save parameters of reprojection matrix for XYZ-calcualtion
[b, u0, v0, f]  = calc_triangulation_parameters(reprojection_matrix);
%% calculate trajectory
j=1;
not_calc = [];
for i = 1:bag_images.NumMessages
        % Read the image message of the current image
        imgMsg = readMessages(bag_images, i);
        % Convert ROS image message to MATLAB image
        img = readImage(imgMsg{1});
        % split img into left and rigth image
        left_img = img(:,1:1280,:);
        right_img = img(:,1281:end,:);
        
        %Rectification and undistortion
        [L,R,~] = rectifyStereoImages(left_img,right_img,stereo_params);
        
        %convert to gray image, otherwise the feature detection won't
        %work.
        L_gray = rgb2gray(L);
        R_gray = rgb2gray(R);
        % detect current Features
        % Feature Detection with MATLAB Surf Features
        points_left_current = detectSURFFeatures(L_gray);
        points_right_current = detectSURFFeatures(R_gray);

        %remove points which are near the image border

        % Filter points that are too close to the image border
        points_left_current = remove_points_near_the_border(points_left_current, border_thresh,height_rect,width_rect);
        points_right_current = remove_points_near_the_border(points_right_current, border_thresh,height_rect,width_rect);
        
        % Feature Extraction
        [features_left_current, validpoints_left_current] = extractFeatures(L_gray, points_left_current);
        [features_right_current, validpoints_right_current] = extractFeatures(R_gray, points_right_current);
        % Feature Matching
        indexPairs = matchFeatures(features_left_current, features_right_current);
        % Matched Feature Points
        matched_points_left_current = remove_unvalid_features(validpoints_left_current,indexPairs(:, 1));
        matched_points_right_current = remove_unvalid_features(validpoints_right_current,indexPairs(:, 2));
        
        % RANSAC for Fundamental Matrix Estimation
        try
            [F, inliers, residuals] = ransac(@fmatrix, [matched_points_left_current.Location'; matched_points_right_current.Location'], ransac_thresh , 'verbose');
            [inlier_points_left_current,inlier_points_right_current] = remove_unvalid_features(matched_points_left_current,matched_points_right_current,inliers);
        catch %if ransac is not working just use the matched points
            warning('ransac not useable');
            inlier_points_left_current = matched_points_left_current;
            inlier_points_right_current = matched_points_right_current;
        end
        
        %Point Cloud calculation from frame 2, in the first frame there is
        %no previous captured frame
        if i >=2 
            %matching between previous and current features in left images
            [features_left_cur_compare,validpoints_left_cur_compare] = extractFeatures(L_gray,inlier_points_left_current);
            [features_left_prev_compare,validpoints_left_prev_compare] = extractFeatures(L_gray_prev,inlier_points_left_prev);
            
            indexPairs_cur_prev_compare = matchFeatures(features_left_cur_compare,features_left_prev_compare);

            [matched_points_left_current_compare,matched_points_right_current_compare] = ...
                remove_unvalid_features(validpoints_left_cur_compare,inlier_points_right_current,indexPairs_cur_prev_compare(:,1));

            [matched_points_left_prev_compare,matched_points_right_prev_compare] = ...
                remove_unvalid_features(validpoints_left_prev_compare,inlier_points_right_prev,indexPairs_cur_prev_compare(:,2));

            % compute new point cloud and transformation matrix if no warnings or
            % errors occur, otherwise use the transformation matrix from the
            % frame before
            try
                %ransace between left image features between prevoius and
                %current
                [F,in,r] = ransac(@fmatrix, [matched_points_left_current_compare.Location'; matched_points_left_prev_compare.Location'], ransac_thresh, 'verbose');
                %Save surf features that are in the left and right of the previous and current image.
                [points_left_cur_surf_compare,points_left_prev_surf_compare,points_right_cur_surf_compare,points_right_prev_surf_compare] = ...
                    remove_unvalid_features(matched_points_left_current_compare,matched_points_left_prev_compare,...
                    matched_points_right_current_compare,matched_points_right_prev_compare,in);

                
                % Filter based on vertical distance between feature points,
                % apply on all four images
                vertical_diff_prev = abs(points_left_cur_surf_compare.Location(:,2)-points_right_cur_surf_compare.Location(:,2));
                valid_matches_cur = vertical_diff_prev <= vertical_thresh ;
       
                vertical_diff_cur = abs(points_left_prev_surf_compare.Location(:,2)-points_right_prev_surf_compare.Location(:,2));
                valid_matches_prev = vertical_diff_cur <= vertical_thresh ;
                
                %combine and convert to indices
                valid_matches = valid_matches_cur & valid_matches_prev;
                valid_indices = find(valid_matches);

                [points_left_cur_surf_compare,points_right_cur_surf_compare,points_left_prev_surf_compare,points_right_prev_surf_compare] = ...
                    remove_unvalid_features(points_left_cur_surf_compare,points_right_cur_surf_compare,points_left_prev_surf_compare,points_right_prev_surf_compare,valid_indices);

                %Only the location of features without featurespace
                points_left_cur = points_left_cur_surf_compare.Location';
                points_right_cur = points_right_cur_surf_compare.Location';
                points_left_prev = points_left_prev_surf_compare.Location';
                points_right_prev = points_right_prev_surf_compare.Location';

                %Compute the point cloud for cur
                disparity_cur = points_left_cur(1,:)-points_right_cur(1,:); %disparity just horizontal
                P_cur = zeros(3,length(disparity_cur));
                %Triangulation (wrt left cam)
                P_cur(2,:) = -b* (points_left_cur(1, :)-u0) ./ disparity_cur;
                P_cur(3,:) = -b* (points_left_cur(2, :)-v0) ./ disparity_cur;
                P_cur(1,:) = abs((b*f)./disparity_cur);
               
                %Compute the point cloud for prev
                disparity_prev = points_left_prev(1,:)-points_right_prev(1,:); %disparity just horizontal
                P_prev = zeros(3,length(disparity_prev));
                %Triangulation (wrt left cam)
                P_prev(2,:) = -b* (points_left_prev(1, :)-u0) ./ disparity_prev;
                P_prev(3,:) = -b* (points_left_prev(2, :)-v0) ./ disparity_prev;
                P_prev(1,:) = abs((b*f)./disparity_prev);
   
                %remove distances which are too far away from camera
                distsToRemove_prev = P_prev(1,:) > dist_thresh;
                P_prev(:, distsToRemove_prev) = [];
                P_cur(:, distsToRemove_prev) = [];
    
                distsToRemove_cur = P_cur(1,:) >dist_thresh;
                P_prev(:, distsToRemove_cur) = [];
                P_cur(:, distsToRemove_cur) = [];
    
                %save point clouds
                Point_cloud_prev{i} = P_prev;
                Point_cloud_cur{i} = P_cur;
                
                %convert to matlab point cloud
                P_cur_cloud = pointCloud(P_cur');
                P_prev_cloud = pointCloud(P_prev');

                %denoising recommended by matlab, but has no postive effect
                %P_cur_cloud = pcdenoise(P_cur_cloud,PreserveStructure=true);
                %P_prev_cloud = pcdenoise(P_prev_cloud,PreserveStructure=true);

                %downsample recommended by matlab, but has no postive effect
                %P_cur_cloud = pcdownsample(P_cur_cloud,gridAverage=gridSize);

                %Iterative closest Point algorithm with icp from matlab
                T_icp = pcregistericp(P_prev_cloud ,P_cur_cloud,Metric="pointToPoint",Maxiterations = 100, Tolerance= [0.01 0.5]);
                
                %save the translation matrices
                T_complete_icp{i}=T_icp;

                % Extract left imu postion in world frame
                imuMsg = readMessages(bag_imu, i);
                
            catch
                %save indices for not calculated transoformation matrices
                not_calc(j)=i
                warning('ransac not useable');
                T_complete_icp{i}=T_icp;
                j=j+1
            end
        end
    
        % save current feautures and images as previous for the next
        % iteration
        inlier_points_left_prev = inlier_points_left_current;
        inlier_points_right_prev = inlier_points_right_current;
        L_gray_prev = L_gray;
        R_gray_prev = R_gray;
end 

%%
save('results/sparse_icp_matlab.mat', 'T_complete_icp', 'not_calc');