clear all;
close all;
clc;

%
%abs_t_threshold = 0.3; %for spar
abs_t_threshold = 0.8; %for dense
%% load rosbag
bag = rosbag("testdrive_2022-10-25-09-01-50.bag");
bagInfo = rosbag("info","testdrive_2022-10-25-09-01-50.bag");

%Groundtruth Pose
bag_imu = select(bag,"Topic",['/zedm/zed_node/pose']);
%Transforamtion left camera to imu
bag_imu_transform = select(bag,"Topic","/zedm/zed_node/left_cam_imu_transform");
transform_left_cam = readMessages(bag_imu_transform,1);
%Raw Stereo Images
bag_images = select(bag,"Topic","/zedm/zed_node/stereo_raw/image_raw_color");

%% load calculated transformations
% Transformation matrices and non-calculated transformations
%load('results_icp_matlab.mat')
%load('results_icp_matlab_no_denois.mat')
% load('results_icp_matlab_no_downsample.mat')
%load('results_icp_matlab_no_denois_no_downsample.mat')
load('results_just_icp_matlab.mat')
load('results_just_icp_matlab_2.mat')
load('results_just_icp_matlab_3.mat')
% load('results_just_icp_matlab_4.mat')
% load('results_just_icp_matlab_5.mat') #best
load('results_just_icp_matlab_6.mat')
% load('results_dense_icp_matlab.mat')
% load('results_dense_icp_matlab_2.mat')
% load('results_dense_icp_matlab_3.mat')
%load('results_dense_icp_matlab_4.mat')
%load('results_dense_icp_matlab_5.mat')

load('icp_thresh/results_icp_1.mat')
%load('icp_thresh/results_icp_2.mat')
%load('icp_thresh/results_icp_3.mat')
%load('icp_thresh/results_icp_4.mat')
%load('icp_thresh/results_icp_5.mat')
load('icp_thresh/results_icp_no_filter.mat')
load('icp_thresh/results_icp_high_ransac.mat')
%load('icp_thresh/results_icp_final.mat')
load('results/sparse_icp_matlab.mat')
abs_t_threshold = 0.5; %for dense
%% Get IMU Messages belonging to image timestamps
%load timestamps
Time_imu = bag_imu.MessageList.Time;
Time_img = bag_images.MessageList.Time;
%find closest timestamps and save indices
idx_imu_closest = knnsearch(Time_imu, Time_img, 'K', 1);

%get the translation between imu and left cam
T_imu_left_cam = transl(transform_left_cam{1}.Translation.X,transform_left_cam{1}.Translation.Y,transform_left_cam{1}.Translation.Z);
rotation_left_cam = UnitQuaternion([transform_left_cam{1}.Rotation.W, transform_left_cam{1}.Rotation.X, transform_left_cam{1}.Rotation.Y, transform_left_cam{1}.Rotation.Z]).R;
T_imu_left_cam(1:3, 1:3) = rotation_left_cam;
T_imu_left_cam = inv(T_imu_left_cam);

%save imus which are closest to image times
for i=1:length(idx_imu_closest)
    tmp = readMessages(bag_imu,idx_imu_closest(i));
    imu{i}=readMessages(bag_imu,idx_imu_closest(i));
    T_world_imu = transl(tmp{1}.Pose.Position.X,tmp{1}.Pose.Position.Y,tmp{1}.Pose.Position.Z);
    rotation_world_imu = UnitQuaternion([tmp{1}.Pose.Orientation.W, tmp{1}.Pose.Orientation.X, tmp{1}.Pose.Orientation.Y, tmp{1}.Pose.Orientation.Z]).R;
    
    T_world_imu(1:3, 1:3) = rotation_world_imu;

    %calcualte cam pose in respect to world frame
    T_world_cam{i} = T_world_imu*T_imu_left_cam;
   
    x_pos(i) =T_world_cam{i}(1,4);
    y_pos(i) =T_world_cam{i}(2,4);
    z_pos(i) =T_world_cam{i}(3,4);

end

%% Plot calculated Poses with ground truth trajectory
clear not_calculated
%Initial Pose
T_world_left_cam = T_world_cam{1};

x_calc(1) = T_world_left_cam(1,4);
y_calc(1) = T_world_left_cam(2,4);
z_calc(1) = T_world_left_cam(3,4);

figure;
hold on;
j = 1;
not_calculated= [];
for i = 2:length(T_complete_icp)
    T_cur = eye(4,4) ;
    T_cur(1:3,4) = T_complete_icp{i}.Translation;
    T_cur(1:3,1:3) = T_complete_icp{i}.R;  
    if abs(T_cur(1,4)) + abs(T_cur(2,4)) + abs(T_cur(3,4)) < abs_t_threshold 
        % T_complete_good is updated only when the condition is met
        T_complete_good = inv(T_cur);
    else
        not_calculated(j)=i;
        j=j+1;
    end
  

    % Calculate T_calc
    T_calc = T_world_left_cam * T_complete_good;
    x_calc(i) = T_calc(1,4);
    y_calc(i) = T_calc(2,4);
    z_calc(i) = T_calc(3,4);

    T_world_left_cam = T_calc;
    
end
% Plot the entire point cloud
plot3(x_pos, y_pos, z_pos, 'g-');
% Plot the entire point cloud
plot3(x_calc,y_calc,z_calc, 'r-');
plot3(x_calc(not_calculated),y_calc(not_calculated),z_calc(not_calculated), 'b.');
plot3(x_calc(not_calc),y_calc(not_calc),z_calc(not_calc), 'b*');
xlabel("x in m")
ylabel("y in m")

%% Plot calculated Poses with ground truth trajetory, refresh every 10 Points
clear not_calculated
% Initial Pose
%Initial Pose
T_world_left_cam = T_world_cam{1};

x_calc(1) = T_world_left_cam(1,4);
y_calc(1) = T_world_left_cam(2,4);
z_calc(1) = T_world_left_cam(3,4);
figure;
hold on;
j = 1;
not_calculated= [];
for i = 2:length(T_complete_icp)
    T_cur = eye(4,4) ;
    T_cur(1:3,4) = T_complete_icp{i}.Translation;
    T_cur(1:3,1:3) = T_complete_icp{i}.R;   
    if abs(T_cur(1,4)) + abs(T_cur(2,4)) + abs(T_cur(3,4)) < abs_t_threshold 
        % T_complete_good is updated only when the condition is met
        T_complete_good = inv(T_cur);
    else
        not_calculated(j)=i;
        j=j+1;
    end

    % Calculate T_calc
    T_calc = T_world_left_cam * T_complete_good;
    x_calc(i) = T_calc(1,4);
    y_calc(i) = T_calc(2,4);
    z_calc(i) = T_calc(3,4);

    T_world_left_cam = T_calc;
    %trplot(T_world_left_cam,'length', 0.1)
    if mod(i, 10) == 0
        if i<300

        end
        T_world_left_cam  =  T_world_cam{i};
        T_calc = T_world_left_cam;
        x_calc(i) = T_calc(1,4);
        y_calc(i) = T_calc(2,4);
        z_calc(i) = T_calc(3,4);
        
        plot3(T_world_left_cam(1,4), T_world_left_cam(2,4), T_world_left_cam(3,4), "g*")
    end   
end
% Plot the entire true trajetory
plot3(x_pos, y_pos, z_pos, 'g-');
% Plot the entire calculated trajetory
%plot3(x_calc,y_calc,z_calc, 'r.');
plot3(x_calc(not_calculated),y_calc(not_calculated),z_calc(not_calculated), 'b.');
plot3(x_calc(not_calc),y_calc(not_calc),z_calc(not_calc), 'b*');
%plt 
xlabel("x in m")
ylabel("y in m")

for i = 0:10:length(x_calc)-10
    if i==0
        indices = 1:i+9;
    else
        indices = i:i+9;
    end
    plot3(x_calc(indices), y_calc(indices), z_calc(indices), 'r-', 'LineWidth', 1);
end

