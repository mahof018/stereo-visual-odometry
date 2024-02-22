% This script plots the results of ICP_dense or ICP_sparse with a transformation
% into the world coordinate system.
%
% You can adjust the init value abs_t_threshold to get another output
%
% Figure 1: Plot of calculated Poses with ground truth trajectory,
%           initialized once in 2D
% Figure 2: Plot of calculated Poses with ground truth trajectory,
%           initialized once in 3D
% Figure 3: Plot of calculated Poses with ground truth trajectory,
%           initialized every 10 frames in 2D
clear all;
close all;
clc;
%% load calculated transformations
% Transformation matrices and not-calculated transformations indices
%filename_results = 'results/dense_icp_matlab';% results from ICP_dense
filename_results = 'results/sparse_icp_matlab';% results from ICP_sparse
load(filename_results+".mat")
abs_t_threshold = 0.25; %threshold for maximale translation between two frames,
                        % otherwise the transformation matrix of the previous frame is used for the plot for dense 0.8
%% load rosbag
% bag = rosbag("testdrive_2022-10-25-09-01-50.bag");
% bagInfo = rosbag("info","testdrive_2022-10-25-09-01-50.bag");
bag = rosbag("shrink_version_testdrive_2022-10-25-09-01-50.bag");
bagInfo = rosbag("info","shrink_version_testdrive_2022-10-25-09-01-50.bag");

%Groundtruth Pose
bag_imu = select(bag,"Topic",['/zedm/zed_node/pose']);
%Transforamtion left camera to imu
bag_imu_transform = select(bag,"Topic","/zedm/zed_node/left_cam_imu_transform");
transform_left_cam = readMessages(bag_imu_transform,1);
%Raw Stereo Images
bag_images = select(bag,"Topic","/zedm/zed_node/stereo_raw/image_raw_color");


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
    %read bag pose
    tmp = readMessages(bag_imu,idx_imu_closest(i));
    
    %Extract pose into homogeneous transformation matrix
    T_world_imu = transl(tmp{1}.Pose.Position.X,tmp{1}.Pose.Position.Y,tmp{1}.Pose.Position.Z);
    rotation_world_imu = UnitQuaternion([tmp{1}.Pose.Orientation.W, tmp{1}.Pose.Orientation.X, tmp{1}.Pose.Orientation.Y, tmp{1}.Pose.Orientation.Z]).R;
    T_world_imu(1:3, 1:3) = rotation_world_imu;

    %calculate cam pose in respect to world frame
    T_world_cam{i} = T_world_imu*T_imu_left_cam;
   
    %save x,y,z coordinates
    x_pos(i) =T_world_cam{i}(1,4);
    y_pos(i) =T_world_cam{i}(2,4);
    z_pos(i) =T_world_cam{i}(3,4);

end

%% Plot calculated Poses with ground truth trajectory in 2D
clear not_calculated
%Initial Pose
T_world_left_cam = T_world_cam{1};

x_calc(1) = T_world_left_cam(1,4);
y_calc(1) = T_world_left_cam(2,4);
z_calc(1) = T_world_left_cam(3,4);

f = figure;
hold on;
j = 1;
not_calculated= [];
for i = 2:length(T_complete_icp)
    %extract current translation calculated by ICP
    T_cur = eye(4,4) ;
    T_cur(1:3,4) = T_complete_icp{i}.Translation;
    T_cur(1:3,1:3) = T_complete_icp{i}.R;  
    if abs(T_cur(1,4)) + abs(T_cur(2,4)) + abs(T_cur(3,4)) < abs_t_threshold  % T_complete_good is updated only when the condition is met
        T_complete_good = inv(T_cur);
    else
        not_calculated(j)=i;
        j=j+1;
    end
  
    % Calculate T_calc
    T_calc = T_world_left_cam * T_complete_good;

    %save x,y,z positions
    x_calc(i) = T_calc(1,4);
    y_calc(i) = T_calc(2,4);
    z_calc(i) = T_calc(3,4);

    T_world_left_cam = T_calc;
    
end
% Plot the entire GT trajectory
plot3(x_pos, y_pos, z_pos, 'g-');
% Plot the entire calculated trajectory
plot3(x_calc,y_calc,z_calc, 'r-');
% mark the not calculated points
plot3(x_calc(not_calculated),y_calc(not_calculated),z_calc(not_calculated), 'b.');
plot3(x_calc(not_calc),y_calc(not_calc),z_calc(not_calc), 'b*');
xlabel("x in m")
ylabel("y in m")

%save plot
exportgraphics(f,filename_results+"_once_initialized_2d.png",'Resolution',300)
%%  Plot calculated Poses with ground truth trajectory in 3D
f = figure;
plot3(x_calc,y_calc,z_calc, 'r-')
hold on
plot3(x_pos, y_pos, z_pos, 'g-');
xlabel("x in m")
ylabel("y in m")
zlabel("z in m")
legend("ICP","GT")

%save plot
exportgraphics(f,filename_results+"_once_initialized_3d.png",'Resolution',300)
%% Plot calculated Poses with ground truth trajetory, refresh every 10 Pointsin 2D
clear not_calculated
% Initial Pose
T_world_left_cam = T_world_cam{1};

x_calc(1) = T_world_left_cam(1,4);
y_calc(1) = T_world_left_cam(2,4);
z_calc(1) = T_world_left_cam(3,4);
figure;
hold on;
j = 1;
not_calculated= [];
for i = 2:length(T_complete_icp)
    %extract current translation calculated by ICP
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
    %update intitial Pose every 10 frames
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
% mark the not calculated points
plot3(x_calc(not_calculated),y_calc(not_calculated),z_calc(not_calculated), 'b.');
plot3(x_calc(not_calc),y_calc(not_calc),z_calc(not_calc), 'b*');
%plt 
xlabel("x in m")
ylabel("y in m")

%combine the calcualted trajectory
for i = 0:10:length(x_calc)-10
    if i==0
        indices = 1:i+9;
    else
        indices = i:i+9;
    end
    plot3(x_calc(indices), y_calc(indices), z_calc(indices), 'r-', 'LineWidth', 1);
end

%save plot
exportgraphics(f,filename_results+"_multiple_initializations_2d.png",'Resolution',300)