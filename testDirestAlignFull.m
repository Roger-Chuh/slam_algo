function testDirestAlignFull(k2c_comp, LR, Depth, intrMat)
global point_id  block  point_id_offset imgs depths Tcws abs last_point_id est_affine est_depth Depth_change use_half inverse_comp  Depth_meas ...
    depth_noise switch_jac use_weight compensate_ab pose_noise pyr_level do_log;





if 0
    Depth_change = [];
    last_point_id = 0;
    point_id = 0;
end

est_depth = 1;1;0;1;
est_affine = 1;0;1;
% load('pba_white.mat');
% load('pba_brick.mat'); 
%   load('pba_large.mat');
 load('pba_large_scene_2.mat');
%  load('pba_large_scene.mat');
% load('pba_large2.mat');
Pose = [];
deltaPoseMat = [];
for i = 0 : length(LR)-1
    if 0
        imwrite(rgb2gray(LR{i+1,1}), sprintf('./simulation_large_scene_2/img_%05d.png', i));
        imwrite(uint16(10.*Depth{i+1,1}),sprintf('./simulation_large_scene_2/depth_%05d.png',i),'png','bitdepth',16);
    end
    pose = inv(k2c_comp{i+1});
    Pose = [Pose; rodrigues(pose(1:3,1:3))' pose(1:3,4)'];
     deltaPoseMat = [deltaPoseMat; [reshape(pose(1:3,1:3),1,9) pose(1:3,4)']];
end



% close all

num_levels = 3; 1; 3; 2; 4; 2; 3; 2; 4; 3; 4; 2; 3; 1; 2; 4;  6; 4; 2;
areaRatio = 0.08; 0.001; 0.08; 0.25; 0.08; 0.005;
initial_sigma = 5;
default_dof = 5;
para = 5;
isVerbose = 0;
alpha0 = 0;
beta0 = 0;

alpha1 = 0;%-0.0847; 0;  -0.0623;
beta1 = 0;10.5487; 0; 94.0413;



if 1
    %% test jacobian 
    X_host = [10 20 30]';
    Tth = [rodrigues([0.001 0.002 0.001]) [10 10 0]'; 0 0 0 1];
    [X_target_scaled, d_X_target_scaled_d_X_host] = D_X_target_scaled_D_X_host(X_host, Tth);
    
    noise = [0.01 -0.1 0.1]';
    X_host_err = X_host + noise;
    [X_target_scaled_err, d_X_target_scaled_d_X_host_err] = D_X_target_scaled_D_X_host(X_host_err, Tth);
    
    X_target_scaled_err_true = X_target_scaled_err - X_target_scaled;
    X_target_scaled_err_comp = d_X_target_scaled_d_X_host * noise;
    
    difference = X_target_scaled_err_comp - X_target_scaled_err_true;
end


img_num = length(k2c_comp); 10; 5; 30; 8; 5; 2;
skip_step = 3;

k2c_comp = k2c_comp(1:skip_step:img_num,:);
LR = LR(1:skip_step:img_num,:);
Depth = Depth(1:skip_step:img_num,:);

Tcws = k2c_comp;



%% 
pyr_level = 3; 2; 3; 1; 3; 1; 3; 1; %3;
Depth_meas_ = cell(length(Tcws)-0, length(Tcws));
Depth_meas = cell(pyr_level,1);
for q = 1 : pyr_level
   Depth_meas{q,1} = Depth_meas_ ;
end
    
    
depth_noise = -100; -500; 100; -100;-100; 0;
pose_noise = true;
inverse_comp =  true; false; true; false; true;
switch_jac = false;true; false;
est_depth =  1;0;1;0;1; 0;1;
est_affine = 1; 0;1;1;0;1;
use_weight = 0;
compensate_ab = 0;1; 0; 1; 0;

do_log = 1;

%%


abs = zeros(size(LR,1),2);

imgs = LR(:,1);
depths = Depth(:,1);
OptimizeFlowFull(intrMat);

dlt = 1; 2;
if 0
    for i = 1 : length(k2c_comp)-dlt
        relPose_check =  k2c_comp{i+dlt} * inv(k2c_comp{i});
        relPose = inv([rodrigues(Pose(i+dlt,1:3)) Pose(i+dlt,4:6)';0 0 0 1]) * [rodrigues(Pose(i,1:3)) Pose(i,4:6)';0 0 0 1];
        pose_init_error = relPose * [rodrigues([0.01 -0.01 0.01 ]) 20.*(rand(3,1)-0.5); 0 0 0 1];
        pose_init_error = relPose * [rodrigues([0.005 -0.005 0.005 ]) 10.*(rand(3,1)-0.5); 0 0 0 1];
        [pose_rel, score, warped_image1, valid_mask, weightMap, warped_z, validMask] = estimateVisualOdometry(double(rgb2gray(LR{i+dlt,1})), double(rgb2gray(LR{i,1})), Depth{i+dlt,1}, Depth{i,1}, intrMat, num_levels, pose_init_error, isVerbose,initial_sigma,default_dof,para);
        %     OptimizeFlow(double(rgb2gray(LR{i,1})),Depth{i,1}, double(rgb2gray(LR{i+1,1})), pose_init_error, intrMat, alpha0, beta0, alpha1, beta1);
        OptimizeFlow(double(rgb2gray(LR{i,1})),Depth{i,1}, double(rgb2gray(LR{i+dlt,1})), pose_init_error, relPose, intrMat, alpha0, beta0, alpha1, beta1);
    end
end




end
function OptimizeFlowFull(intrMat)
global point_id  block point_id_offset Depth_meas imgs depths Tcws abs last_point_id est_affine est_depth Depth_change use_half iter inverse_comp depth_noise ab_stack ...
    switch_jac use_weight compensate_ab pose_noise pyr_level do_log;
% Depth_meas = cell(length(imgs)-1, length(imgs));

Depth_meas_check = Depth_meas;
Tcws_gt = Tcws;

depths_gt = depths;
ab_noise = [1.2 -10; 1.5 -30; 2.0 -100; 0.7 100; 0.8 50; 0.8 50; 0.8 50; 0.8 50; 0.8 50; 0.8 50; 0.8 50; 0.8 50; 0.8 50; ...
    1.2 -10; 1.5 -30; 2.0 -100; 0.7 100; 0.8 50; 0.8 50; 0.8 50; 0.8 50; 0.8 50; 0.8 50; 0.8 50; 0.8 50; 0.8 50;
    1.2 -10; 1.5 -30; 2.0 -100; 0.7 100; 0.8 50; 0.8 50; 0.8 50; 0.8 50; 0.8 50; 0.8 50; 0.8 50; 0.8 50; 0.8 50;];
ab_noise(:,1)=0;
ab_noise(:,2)=0;
ab_noise=[[6.94776699491607e-16,-1.82028969547464e-13;-0.5503948664055963,-50.81418778479883;-0.231905487458427,-14.8806510125994;-0.308112085687490,-22.5944368338627;-0.604365032733208,-70.0347667148901]];
ab_noise =[ab_noise;ab_noise;ab_noise;ab_noise;ab_noise;ab_noise;ab_noise;ab_noise;ab_noise];
err_pose={};
if 0
     err_pose{2,1}= [rodrigues([-0.001 0.002 0.001]) [6 8 0]'; 0 0 0 1];
    err_pose{3,1}= [rodrigues([0.002 -0.001 0.002]) [2 3 -2]'; 0 0 0 1];
    err_pose{4,1}= [rodrigues([0.002 0.002 -0.001]) [2 2 -3]'; 0 0 0 1];
    err_pose{5,1}= [rodrigues([-0.002 0.002 0.001]) [-2 -2 -3]'; 0 0 0 1];
    
    % err_pose{2,1}= [rodrigues([-0.001 0.002 0.001]) [6 8 0]'; 0 0 0 1];
    % err_pose{3,1}= [rodrigues([0.002 -0.002 0.003]) [5 3 -5]'; 0 0 0 1];
    % err_pose{4,1}= [rodrigues([0.003 0.002 -0.001]) [5 5 -5]'; 0 0 0 1];
    % err_pose{5,1}= [rodrigues([-0.002 0.002 0.002]) [-5 -2 -6]'; 0 0 0 1];
    
    err_pose{6,1}= [rodrigues([0.002 0.002 0.001]) [5 2 3]'; 0 0 0 1];
    err_pose{7,1}= [rodrigues([0.001 -0.002 0.002]) [2 5 -3]'; 0 0 0 1];
    err_pose{8,1}= [rodrigues([-0.001 0.002 -0.002]) [5 2 -3]'; 0 0 0 1];
    err_pose{9,1}= [rodrigues([0.002 -0.002 0.002]) [3 -5 -2]'; 0 0 0 1];
    err_pose{10,1}= [rodrigues([0.001 0.003 0.001]) [1 5 3]'; 0 0 0 1];
    err_pose{11,1}= [rodrigues([0.001 -0.002 0.002]) [2 3 -5]'; 0 0 0 1];
    err_pose{12,1}= [rodrigues([0.001 0.001 -0.002]) [5 2 -3]'; 0 0 0 1];
    err_pose{13,1}= [rodrigues([0.002 -0.001 0.001]) [5 -5 -2]'; 0 0 0 1];
    err_pose{14,1}= [rodrigues([0.001 0.001 0.001]) [1 1 5]'; 0 0 0 1];
    err_pose{15,1}= [rodrigues([0.001 -0.001 0.002]) [2 3 -2]'; 0 0 0 1];
    err_pose{16,1}= [rodrigues([0.001 0.001 -0.001]) [2 2 -2]'; 0 0 0 1];
    err_pose{17,1}= [rodrigues([0.001 -0.001 0.001]) [1 -2 -2]'; 0 0 0 1];
    err_pose{18,1}= [rodrigues([0.001 0.001 0.001]) [1 2 0]'; 0 0 0 1];
    err_pose{19,1}= [rodrigues([0.001 -0.001 0.001]) [2 3 -2]'; 0 0 0 1];
    err_pose{20,1}= [rodrigues([0.001 0.001 -0.001]) [2 1 -2]'; 0 0 0 1];
    err_pose{21,1}= [rodrigues([0.001 -0.001 0.001]) [1 -2 -2]'; 0 0 0 1];
    err_pose{22,1}= [rodrigues([-0.001 0.001 0.001]) [1 2 0]'; 0 0 0 1];
    err_pose{23,1}= [rodrigues([0.001 -0.001 -0.001]) [-1 3 -2]'; 0 0 0 1];
    err_pose{24,1}= [rodrigues([10.001 0.001 -0.001]) [-1 5 -2]'; 0 0 0 1];
    err_pose{25,1}= [rodrigues([0.001 -0.001 0.001]) [2 -2 2]'; 0 0 0 1];
    err_pose{26,1}= [rodrigues([0.001 -0.001 0.001]) [1 -2 0]'; 0 0 0 1];
    err_pose{27,1}= [rodrigues([-0.001 -0.001 0.001]) [-2 3 -2]'; 0 0 0 1];
    err_pose{28,1}= [rodrigues([0.001 0.001 -0.001]) [2 -1 -5]'; 0 0 0 1];
    err_pose{29,1}= [rodrigues([0.001 -0.001 0.001]) [-1 -2 -2]'; 0 0 0 1];
    err_pose{30,1}= [rodrigues([0.001 -0.001 0.001]) [-1 2 2]'; 0 0 0 1];
elseif 0
    err_pose{2,1}= [rodrigues([-0.001 0.002 0.001]) [6 8 0]'; 0 0 0 1];
    err_pose{3,1}= [rodrigues([0.002 -0.001 0.004]) [2 5 -2]'; 0 0 0 1];
    err_pose{4,1}= [rodrigues([0.002 0.004 -0.001]) [2 2 -5]'; 0 0 0 1];
    err_pose{5,1}= [rodrigues([-0.002 0.004 0.001]) [-2 -2 -3]'; 0 0 0 1];
    
    % err_pose{2,1}= [rodrigues([-0.001 0.002 0.001]) [6 8 0]'; 0 0 0 1];
    % err_pose{3,1}= [rodrigues([0.002 -0.002 0.003]) [5 3 -5]'; 0 0 0 1];
    % err_pose{4,1}= [rodrigues([0.003 0.002 -0.001]) [5 5 -5]'; 0 0 0 1];
    % err_pose{5,1}= [rodrigues([-0.002 0.002 0.002]) [-5 -2 -6]'; 0 0 0 1];
    
    err_pose{6,1}= [rodrigues([0.004 0.002 0.001]) [5 2 5]'; 0 0 0 1];
    err_pose{7,1}= [rodrigues([0.001 -0.002 0.004]) [2 5 -5]'; 0 0 0 1];
    err_pose{8,1}= [rodrigues([-0.001 0.004 -0.002]) [5 2 -5]'; 0 0 0 1];
    err_pose{9,1}= [rodrigues([0.004 -0.002 0.002]) [5 -5 -2]'; 0 0 0 1];
    err_pose{10,1}= [rodrigues([0.001 0.005 0.001]) [1 5 5]'; 0 0 0 1];
    err_pose{11,1}= [rodrigues([0.001 -0.001 0.001]) [2 5 -5]'; 0 0 0 1];
    err_pose{12,1}= [rodrigues([0.001 0.001 -0.001]) [5 2 -5]'; 0 0 0 1];
    err_pose{13,1}= [rodrigues([0.001 -0.001 0.001]) [5 -5 -2]'; 0 0 0 1];
    err_pose{14,1}= [rodrigues([0.001 0.001 0.001]) [1 1 5]'; 0 0 0 1];
    err_pose{15,1}= [rodrigues([0.001 -0.001 0.002]) [2 3 -2]'; 0 0 0 1];
    err_pose{16,1}= [rodrigues([0.001 0.001 -0.001]) [2 2 -2]'; 0 0 0 1];
    err_pose{17,1}= [rodrigues([0.001 -0.001 0.001]) [1 -2 -2]'; 0 0 0 1];
    err_pose{18,1}= [rodrigues([0.001 0.001 0.001]) [1 2 0]'; 0 0 0 1];
    err_pose{19,1}= [rodrigues([0.001 -0.001 0.001]) [2 3 -2]'; 0 0 0 1];
    err_pose{20,1}= [rodrigues([0.001 0.001 -0.001]) [2 1 -2]'; 0 0 0 1];
    err_pose{21,1}= [rodrigues([0.001 -0.001 0.001]) [1 -2 -2]'; 0 0 0 1];
    err_pose{22,1}= [rodrigues([-0.001 0.001 0.001]) [1 2 0]'; 0 0 0 1];
    err_pose{23,1}= [rodrigues([0.001 -0.001 -0.001]) [-1 3 -2]'; 0 0 0 1];
    err_pose{24,1}= [rodrigues([10.001 0.001 -0.001]) [-1 5 -2]'; 0 0 0 1];
    err_pose{25,1}= [rodrigues([0.001 -0.001 0.001]) [2 -2 2]'; 0 0 0 1];
    err_pose{26,1}= [rodrigues([0.001 -0.001 0.001]) [1 -2 0]'; 0 0 0 1];
    err_pose{27,1}= [rodrigues([-0.001 -0.001 0.001]) [-2 3 -2]'; 0 0 0 1];
    err_pose{28,1}= [rodrigues([0.001 0.001 -0.001]) [2 -1 -5]'; 0 0 0 1];
    err_pose{29,1}= [rodrigues([0.001 -0.001 0.001]) [-1 -2 -2]'; 0 0 0 1];
    err_pose{30,1}= [rodrigues([0.001 -0.001 0.001]) [-1 2 2]'; 0 0 0 1];
else
    err_pose{2,1}= [rodrigues([-0.001 0.002 0.001]) [6 8 0]'; 0 0 0 1];
    err_pose{3,1}= [rodrigues([0.005 -0.005 0.003]) [5 3 -5]'; 0 0 0 1];
    err_pose{4,1}= [rodrigues([0.03 0.005 -0.006]) [5 5 -5]'; 0 0 0 1];
    err_pose{5,1}= [rodrigues([-0.01 0.002 0.005]) [-5 -2 -6]'; 0 0 0 1];
    
    % err_pose{2,1}= [rodrigues([-0.001 0.002 0.001]) [6 8 0]'; 0 0 0 1];
    % err_pose{3,1}= [rodrigues([0.002 -0.002 0.003]) [5 3 -5]'; 0 0 0 1];
    % err_pose{4,1}= [rodrigues([0.003 0.002 -0.001]) [5 5 -5]'; 0 0 0 1];
    % err_pose{5,1}= [rodrigues([-0.002 0.002 0.002]) [-5 -2 -6]'; 0 0 0 1];
    
    err_pose{6,1}= [rodrigues([0.02 0.002 0.001]) [6 8 0]'; 0 0 0 1];
    err_pose{7,1}= [rodrigues([0.002 -0.02 0.002]) [5 3 -10]'; 0 0 0 1];
    err_pose{8,1}= [rodrigues([0.003 0.002 -0.001]) [5 5 -5]'; 0 0 0 1];
    err_pose{9,1}= [rodrigues([0.001 -0.02 0.003]) [5 -2 -6]'; 0 0 0 1];
    err_pose{10,1}= [rodrigues([0.002 0.002 0.001]) [6 8 0]'; 0 0 0 1];
    err_pose{11,1}= [rodrigues([0.002 -0.002 0.02]) [5 3 -10]'; 0 0 0 1];
    err_pose{12,1}= [rodrigues([0.003 0.002 -0.001]) [5 5 -5]'; 0 0 0 1];
    err_pose{13,1}= [rodrigues([0.01 -0.002 0.003]) [5 -2 -6]'; 0 0 0 1];
    err_pose{14,1}= [rodrigues([0.002 0.002 0.001]) [6 8 0]'; 0 0 0 1];
    err_pose{15,1}= [rodrigues([0.002 -0.02 0.002]) [5 3 -10]'; 0 0 0 1];
    err_pose{16,1}= [rodrigues([0.003 0.002 -0.001]) [5 5 -5]'; 0 0 0 1];
    err_pose{17,1}= [rodrigues([0.001 -0.02 0.003]) [5 -2 -6]'; 0 0 0 1];
    err_pose{18,1}= [rodrigues([0.002 0.002 0.001]) [6 8 0]'; 0 0 0 1];
    err_pose{19,1}= [rodrigues([0.002 -0.02 0.002]) [5 3 -10]'; 0 0 0 1];
    err_pose{20,1}= [rodrigues([0.003 0.002 -0.001]) [5 5 -5]'; 0 0 0 1];
    err_pose{21,1}= [rodrigues([0.01 -0.002 0.003]) [5 -2 -6]'; 0 0 0 1];
    err_pose{22,1}= [rodrigues([0.002 0.002 0.001]) [6 8 0]'; 0 0 0 1];
    err_pose{23,1}= [rodrigues([0.02 -0.002 0.002]) [5 3 -10]'; 0 0 0 1];
    err_pose{24,1}= [rodrigues([0.003 0.002 -0.01]) [5 5 -5]'; 0 0 0 1];
    err_pose{25,1}= [rodrigues([0.001 -0.002 0.003]) [5 -2 -6]'; 0 0 0 1];
    err_pose{26,1}= [rodrigues([0.002 0.002 0.01]) [6 8 0]'; 0 0 0 1];
    err_pose{27,1}= [rodrigues([0.002 -0.002 0.002]) [5 3 -10]'; 0 0 0 1];
    err_pose{28,1}= [rodrigues([0.003 0.002 -0.001]) [5 5 -5]'; 0 0 0 1];
    err_pose{29,1}= [rodrigues([0.001 -0.02 0.003]) [5 -2 -6]'; 0 0 0 1];
    err_pose{30,1}= [rodrigues([0.01 -0.002 0.003]) [5 -2 -6]'; 0 0 0 1];
end
range = [0:255];
%range_ = log(range+1);
range2 = 45.9859 * log(range + 1);
%scal = range./range_;
% scale = inf;
% for(i = 1:length(Tcws))
% %     imgs{i,1} = ab_noise(i,1) * imgs{i,1} + ab_noise(i,2);
%     im = round(double(imgaussfilt(imgs{i,1},0.6)));
%     im(im>255) = 255;
%     im(im<0) = 0;
%     img_(:,:,1) = 45.9859*log(im(:,:,1)+1);
%     img_(:,:,2) = 45.9859*log(im(:,:,2)+1);
%     img_(:,:,3) = 45.9859*log(im(:,:,3)+1);
%     %scale = min([scale, max(255/img_(:))]); 
%     
% end
scale =45.9859/1.2;% scale/2;
for(i = 1:length(Tcws))
    %     imgs{i,1} = ab_noise(i,1) * imgs{i,1} + ab_noise(i,2);
    
    imgs{i,1} = imgaussfilt(imgs{i,1},0.6);
    
    im = double(imgs{i,1});
    im(im>255) = 255;
    im(im<0) = 0;
    if do_log
        img_(:,:,1) = log(im(:,:,1)+1);
        img_(:,:,2) = log(im(:,:,2)+1);
        img_(:,:,3) = log(im(:,:,3)+1);
        img_ = scale.*img_;
    else
        img_ = im;
    end
   
    
    imgs{i,1} = uint8(exp(ab_noise(i,1)) *( img_ - ab_noise(i,2)));
    
     if 0
        figure,imshow([rgb2gray(imgaussfilt(imgs{i,1},0.6)) fliplr(rgb2gray(imgs{i,1}))]);
    end
%     imgs{i,1} = ab_noise(i,1) *( imgs{i,1}) + ab_noise(i,2);
    imgs{i,1}(imgs{i,1} > 255) = 255;
    imgs{i,1}(imgs{i,1} < 0) = 0;
    if est_depth
        depths{i,1} = depths{i,1}  +    depth_noise.*(rand(size(depths{i,1}))-0.5); depth_noise.*ones(size(depths{i,1}));
    end
    if pose_noise
        if length(Tcws) >= 5
            if i >= 3
                Tcws{i,1} = Tcws{i,1} * err_pose{i,1};[rodrigues(0.005.*(rand(3,1)-0.5)) 20.*(rand(3,1)-0.5); 0 0 0 1];
            end
        end
        if length(Tcws) == 2
            if i >= 2
                Tcws{i,1} = Tcws{i,1} * err_pose{i,1};[rodrigues(0.005.*(rand(3,1)-0.5)) 20.*(rand(3,1)-0.5); 0 0 0 1];
            end
        end
        if i == -2
            Tcws{i,1} = Tcws{i,1} * [rodrigues(0.05.*(rand(3,1)-0.5)) [ 0 0 0]'; 0 0 0 1];
        end
    end
end


Depth_change = cell(pyr_level,1);
imgs_pyr = cell(length(Tcws),pyr_level);
depths_pyr = cell(length(Tcws),pyr_level);
depths_gt_pyr = cell(length(Tcws),pyr_level);



for imgId = 1 : length(Tcws)
    img_curr_pyr = constructPyramid(imgs{imgId,1}, pyr_level);
    depth_curr_pyr = constructPyramid(depths{imgId,1}, pyr_level);
    depth_curr_pyr_gt = constructPyramid(depths_gt{imgId,1}, pyr_level);
   for lvl = 1 : pyr_level
        imgs_pyr{imgId, lvl} = img_curr_pyr{end-lvl+1,1};
        depths_pyr{imgId, lvl} = depth_curr_pyr{end-lvl+1,1};
        depths_gt_pyr{imgId, lvl} = depth_curr_pyr_gt{end-lvl+1,1};
    end 
end


total_cost_ = cell(2,1);
Reproj_error_=cell(2,1);
ang_trans_err_stack_ = cell(2,1);
abs_stack_ = cell(2,1);

total_cost = cell(pyr_level,1);
Reproj_error=cell(pyr_level,1);
ang_trans_err_stack = cell(pyr_level,1);
abs_stack = cell(pyr_level,1);
K = cell(3,1);

img_width = size(imgs{imgId,1},2);
img_height = size(imgs{imgId,1},1);
for (lvl = 1 : pyr_level)
    width = img_width/(2^(pyr_level-lvl));
    height = img_height/(2^(pyr_level-lvl));
    K{lvl,1} = intrMat;
    K{lvl,1}(1:2,:) = K{lvl,1}(1:2,:)/(2^(pyr_level-lvl));
     K{lvl,1}(1,3) = (width + 1)/2;
     K{lvl,1}(2,3) = (height+1)/2;
     
%      total_cost{lvl,1} = total_cost_;
%      Reproj_error{lvl,1} = Reproj_error_;
     ang_trans_err_stack{lvl,1} = ang_trans_err_stack_;
     abs_stack{lvl,1} = abs_stack_;
end

figure_num = [100 101 102 103 104 105 106];

point_id = zeros(1, pyr_level);
last_point_id = zeros(1, pyr_level);

iter_num = [50 30 15 15 15 15 15];
iter_num = iter_num(1:pyr_level);
iter_num = fliplr(iter_num);

for pyr_lvl = 1 : pyr_level
    
    
    for iter = 0 : iter_num(pyr_lvl)
        
        %         point_id = zeros(1, pyr_level);
        %         last_point_id = zeros(1, pyr_level);
        
        
        % 不同次迭代之间都是独立的，所有buffer以及统计的id可以清空或置零。
        
        point_id(pyr_lvl) = 0;
        last_point_id(pyr_lvl) = 0;
        
        %     for pyr_lvl = 1 : pyr_level
        
        
        
        
        block.Hpp = [];
        block.Hpm = [];
        block.Hmm = [];
        block.Hmm_inv = [];
        block.bp = [];
        block.bm = [];
        block.xp = [];
        block.xm = [];
        block.c = 0;
        % Depth_meas = {};
        point_id_offset = (length(imgs)-1) * length(imgs) /2 *8;
        
        
        if(size(Depth_meas{1,1},1) < size(Depth_meas{1,1},2))
            use_half = true;
            loop1 = -1;
        else
            use_half = false;
            loop1 = 0;
        end
        
        
        
        reproj_error_=[];
        for i = 1 : length(imgs) + loop1
            host_img = double(imgs_pyr{i,pyr_lvl});
            host_depth = depths_pyr{i,pyr_lvl};
            host_depth_gt = depths_gt_pyr{i,pyr_lvl};
            Twh = inv(Tcws{i,1});
            Twh_gt = inv(Tcws_gt{i,1});
            %         Twh = Twh * [rodrigues(0.0005.*(rand(3,1)-0.5)) 10.*(rand(3,1)-0.5); 0 0 0 1];
            alpha_h = abs(i,1);
            beta_h = abs(i,2);
            if use_half
                loop2 = i+1:length(imgs);
            else
                loop2 =  1:length(imgs);
            end
            for j = loop2
                if i == j
                    continue;
                end
                target_img = double(imgs_pyr{j,pyr_lvl});
                target_depth = depths_pyr{j,pyr_lvl};
                target_depth_gt = depths_gt_pyr{j,pyr_lvl};
                Twt = inv(Tcws{j,1});
                Twt_gt = inv(Tcws_gt{j,1});
                %             Twt = Twt * [rodrigues(0.0001.*(rand(3,1)-0.5)) 10.*(rand(3,1)-0.5); 0 0 0 1];
                alpha_t = abs(j,1);
                beta_t = abs(j,2);
                if ((i == 1 && j == 2) || (i == 2 && j == 1))
                    Twh = Twh_gt;
                    Twt = Twt_gt;
                end
                [ depth_meas, reproj_error] = Adjust(pyr_lvl, i, j , host_img, host_depth, host_depth_gt, target_img, target_depth, target_depth_gt, Twh, Twt,Twh_gt, Twt_gt, alpha_h, beta_h, alpha_t, beta_t, K{pyr_lvl});
                reproj_error_=[reproj_error_;reproj_error];
                if iter >=0;
                    Depth_meas{pyr_lvl,1}{i,j} = depth_meas; %%  另外 全局变量block已经在函数内部更新，depth_meas是为了记录depth优化迭代过程
                end
                
            end
        end
        
        if iter > 0
            [~, reproj_error_norm] = NormalizeVector(reproj_error_);
            if ~isempty(Reproj_error{pyr_lvl})
                if length(reproj_error_norm) < size(Reproj_error{pyr_lvl},1)
                    reproj_error_norm = [reproj_error_norm; nan( size(Reproj_error{pyr_lvl},1) - length(reproj_error_norm), 1)];
                    
                end
                if length(reproj_error_norm) > size(Reproj_error{pyr_lvl},1)
                    reproj_error_norm = reproj_error_norm(1:size(Reproj_error{pyr_lvl},1),:);
                    
                end
            end
            
            Reproj_error{pyr_lvl} = [Reproj_error{pyr_lvl} reproj_error_norm ];
            Solve();
            Update(pyr_lvl);
            total_cost{pyr_lvl} = [total_cost{pyr_lvl};block.c];
            if iter > 10
                if(total_cost{pyr_lvl}(end) > total_cost{pyr_lvl}(end-1))
                    break;
                end
            end
            
            
            ang_trans_err = [];
            for(img_num = 1 : length(imgs))
                
                dlt_pose = inv(Tcws{img_num}) * Tcws_gt{img_num};
                d_ang = rad2deg(norm(rodrigues(dlt_pose(1:3,1:3))));
                d_trans = norm(dlt_pose(1:3,4));
                ang_trans_err = [ang_trans_err; [d_ang d_trans]];
            end
            ang_trans_err_stack{pyr_lvl,1}{1,1} = [ang_trans_err_stack{pyr_lvl,1}{1,1} ang_trans_err(:,1)];
            ang_trans_err_stack{pyr_lvl,1}{2,1} = [ang_trans_err_stack{pyr_lvl,1}{2,1} ang_trans_err(:,2)];
            
            abs_stack{pyr_lvl,1}{1,1} = [abs_stack{pyr_lvl,1}{1,1} abs(:,1)];
            abs_stack{pyr_lvl,1}{2,1} = [abs_stack{pyr_lvl,1}{2,1} abs(:,2)];
            
%             figure(100),
            figure(figure_num(pyr_lvl)),
            if 0
                subplot(4,2,[1]);hist(Depth_change{pyr_lvl},20);grid on;title(sprintf('median depth err: %0.4f mm, depth noise: %0.4f mm',median((Depth_change{pyr_lvl}(~isnan(Depth_change{pyr_lvl}(:,end)),end))), depth_noise));
                subplot(4,2,2);hist(Reproj_error{pyr_lvl},50);grid on;title(sprintf('mean reproj err: %0.4f pixel',mean(Reproj_error{pyr_lvl}(~isnan(Reproj_error{pyr_lvl}(:,end)),end))));
            else
                subplot(4,2,[1]);plot(median(Depth_change{pyr_lvl}));grid on;title(sprintf('median depth err: %0.4f mm, depth noise: %0.4f mm',median((Depth_change{pyr_lvl}(~isnan(Depth_change{pyr_lvl}(:,end)),end))), depth_noise));
                subplot(4,2,2);plot(mean(immultiply(Reproj_error{pyr_lvl},~isnan(Reproj_error{pyr_lvl}))));grid on;title(sprintf('mean reproj err: %0.4f pixel',mean(Reproj_error{pyr_lvl}(~isnan(Reproj_error{pyr_lvl}(:,end)),end))));
            end
            nan_depth_num = sum(sum(isnan(Reproj_error{pyr_lvl})));
            subplot(4,2,3);plot(total_cost{pyr_lvl});grid on;title(sprintf('total loss, inverse comp: %d, use half: %d\nswitch jac: %d, use weight: %d, compensate ab: %d',inverse_comp, use_half, switch_jac, use_weight, compensate_ab));
            subplot(4,2,4);plot(diff(total_cost{pyr_lvl}));grid on;title('loss diff');
            subplot(4,2,5); plot(ang_trans_err_stack{pyr_lvl}{1,1}');title(sprintf('angle err: %0.4f (deg)', max(ang_trans_err_stack{pyr_lvl}{1,1}(:,end))));grid on;
            subplot(4,2,6); plot(ang_trans_err_stack{pyr_lvl}{2,1}');title(sprintf('trans err: %0.4f (mm)', max(ang_trans_err_stack{pyr_lvl}{2,1}(:,end))));grid on;
            subplot(4,2,7); plot(abs_stack{pyr_lvl}{1,1}');title('alpha');grid on;
            subplot(4,2,8); plot(abs_stack{pyr_lvl}{2,1}');title('beta');grid on;drawnow;
            %     FillUpdatedDepthsInDepthMap();
        end
        
        Depth_meas_check = Depth_meas{pyr_lvl,1};
        if iter == 0
            last_point_id_offset = 0;
            for id1 = 1 : size(Depth_meas_check,1)
                if (size(Depth_meas_check,1) < size(Depth_meas_check,2))
                    pointInd = Depth_meas_check{id1,id1+1}(:,5);
                    for id2 = id1 + 1 : size(Depth_meas_check,2)
                        pointInd = intersect(pointInd, Depth_meas_check{id1,id2}(:,5));
                    end
                    for id2 = id1 + 1 : size(Depth_meas_check,2)
                        comm_id = ismember(Depth_meas_check{id1, id2}(:,5), pointInd);
                        Depth_meas{pyr_lvl,1}{id1, id2} = Depth_meas_check{id1, id2}(comm_id,:);
                        Depth_meas{pyr_lvl,1}{id1, id2}(:,5) = last_point_id_offset + [1:length(pointInd)]';
                    end
                    last_point_id_offset = Depth_meas{pyr_lvl,1}{id1, id1+1}(end,5);
                    %             last_point_id_offset = Depth_meas{id1, id1+1}(end,5);
                else
                    if id1 == 1
                        pointInd = Depth_meas_check{id1,2}(:,5);
                        for id2 = 2 : size(Depth_meas_check,2)
                            pointInd = intersect(pointInd, Depth_meas_check{id1,id2}(:,5));
                        end
                        for id2 = 2 : size(Depth_meas_check,2)
                            comm_id = ismember(Depth_meas_check{id1, id2}(:,5), pointInd);
                            Depth_meas{pyr_lvl,1}{id1, id2} = Depth_meas_check{id1, id2}(comm_id,:);
                            Depth_meas{pyr_lvl,1}{id1, id2}(:,5) = last_point_id_offset + [1:length(pointInd)]';
                        end
                        last_point_id_offset = Depth_meas{pyr_lvl,1}{id1, 2}(end,5);
                    else
                        pointInd = Depth_meas_check{id1,1}(:,5);
                        for id2 = 1 : size(Depth_meas_check,2)
                            if (id1 ~= id2)
                                pointInd = intersect(pointInd, Depth_meas_check{id1,id2}(:,5));
                            end
                        end
                        for id2 = 1 : size(Depth_meas_check,2)
                            if id1 ~= id2
                                comm_id = ismember(Depth_meas_check{id1, id2}(:,5), pointInd);
                                Depth_meas{pyr_lvl,1}{id1, id2} = Depth_meas_check{id1, id2}(comm_id,:);
                                Depth_meas{pyr_lvl,1}{id1, id2}(:,5) = last_point_id_offset + [1:length(pointInd)]';
                            end
                        end
                        last_point_id_offset = Depth_meas{pyr_lvl,1}{id1, 1}(end,5);
                    end
                    
                    
                end
            end
        end
        
        
    end
    
    asgjh = 1;
    
end

end
function Solve()
global block est_affine est_depth Tcws;
blk = block;
% blk.Hmm = diag(blk.Hmm);
% blk.Hmm_inv = inv(blk.Hmm);
% assert(isempty(find(blk.Hmm == 0)));
% blk.Hmm(find(blk.Hmm == 0)) = 0.00001;
blk.Hmm_inv = diag(1./blk.Hmm);

if 0
    figure,imshow(abs(blk.Hpp),[]);
end

blk.Hpp_upper = blk.Hpp;

if 1
    upper_ = triu(blk.Hpp);
    lower_ = tril(blk.Hpp);
    mask__ = blk.Hpp ~= 0;
    
    mask_ = (lower_~=0);
    mask_(logical(eye(size(mask_)))) = 0;
    blk.Hpp(mask_) = 0;
    blk.Hpp = blk.Hpp + immultiply(mask_, blk.Hpp');
end
lower = triu(blk.Hpp)';
mask = blk.Hpp==0;
lower1 = immultiply(lower, mask);
%% 把上三角对折下来
blk.Hpp = blk.Hpp + lower1;

block = blk;
if est_depth
    schur = MargPointsAll();
else
    schur.Hpp = blk.Hpp;
    schur.bp = blk.bp;
end
large = 5e14;
medium =  1e5; 1; 1e5;1e6;1e5;1e4; 1e6; 1e8;100; 10000; 1e4;1;100; 1;1e4; 10;1e2;1;1e2;1e2;1e4;
small =  100; 1; 100; 1e4;

% large = 1e5;
% small = 1e2;
% 固定第一帧pose
df = 6 + 2*est_affine;
schur.Hpp_bak = schur.Hpp;
schur.Hpp = FillMatrix(schur.Hpp_bak, 6, 6, 1, 1,large.*eye(6));
% 固定第二帧trans
if length(Tcws) > 2
    schur.Hpp = FillMatrix(schur.Hpp, 3, 3, df+3+1, df+3+1,large.*eye(3));
end
num_frames = length(schur.bp)/df; 8;
if est_affine
    for  i = 1 : num_frames
        ai = (i-1) * df + 6+1;
        if i == 1
            schur.Hpp = FillMatrix(schur.Hpp, 2, 2, ai, ai,large.*eye(2));
            adafd = 1;
            %        schur.Hpp = FillMatrix(schur.Hpp, 2, 2, 1, 1,large.*eye(2));
        else
            if 0
                schur.Hpp = FillMatrix(schur.Hpp, 2, 2, ai, ai,small.*eye(2));
            else
                schur.Hpp = FillMatrix(schur.Hpp, 1, 1, ai, ai,medium);
                schur.Hpp = FillMatrix(schur.Hpp, 1, 1, ai+1, ai+1,small);
            end
        end
    end
end

yp = inv(schur.Hpp)*schur.bp; % yp对应block_.xp: 即pose 和affine部分的增量
fprintf(sprintf('schur.Hpp rank: %f\n',rank(schur.Hpp)));
inc = reshape(yp,df,[])
cost = block.c;
block.xp = yp;
blk2 = block;
if est_depth
    blk2.xm = blk2.Hmm_inv * (blk2.bm - blk2.Hpm' * blk2.xp);
else
    blk2.xm = zeros(length(blk2.bm),1);
end
block = blk2;

end
function Update(pyr_lvl)
global block Depth_meas imgs depths abs Tcws point_id est_affine Depth_change use_half;
df = 6 + 2*est_affine; 8;
for i = 1 : length(imgs)
    kf_id = df*(i-1)+1;
    dx = block.xp(kf_id : df*i);
    Twc1 = inv(Tcws{i});
    Twc = Twc1 * [rodrigues(dx(1:3)) dx(4:6); 0 0 0 1];
    Tcws{i} = inv(Twc);
    if est_affine
        abs(i,1) = abs(i,1) + dx(7);
        abs(i,2) = abs(i,2) + dx(8);
    end
end

% abs

depth_update_scale = 0.75; 1; 0.75; 1; 0.75; 1; 0.75; 1;0.85; 0.5; 0.85;1; 0.75; 0.1; 0.25; 1;0.2; 0.85; 0.75; 0.2; 0.8; 0.1; 0.75;1;0.85; 0.25; 0.1; 0.25; 0.002; 0.85;0.1; 0.85; 0.5; 0.05;

% assert(point_id == length(block.xm));
if 1
%     depth_change = [;
for i = 1 : size(Depth_meas{pyr_lvl,1},1)
    if use_half
        depth_meas = Depth_meas{pyr_lvl,1}{i,i+1};
    else
        if i ~=1
            depth_meas = Depth_meas{pyr_lvl,1}{i,1};
        else
            depth_meas = Depth_meas{pyr_lvl,1}{i,2};
        end
    end
    for pt = 1 : size(depth_meas,1)
        pt_id = depth_meas(pt,5);
        depth_ind = depth_meas(pt,3);
        idepth_old = 1/depth_meas(pt,1);
        depth_gt = depth_meas(pt,4);
        if(block.xm(pt_id) == 0)
            safkgj = 1;
        end
        idepth_new = max([0 idepth_old + depth_update_scale*block.xm(pt_id)]);
        if(idepth_new == 0)
            idepth_new = idepth_old;
        end
        %         diff = [1/idepth_new-1/idepth_old];
        if 1/idepth_new > depth_gt
            diff = [1/idepth_new-depth_gt];
        else
            diff = -[1/idepth_new-depth_gt];
        end
        depth_change(pt_id,1) = diff;
        depths{i}(depth_ind) = 1/idepth_new;
    end
end

if length(depth_change) < size(Depth_change{pyr_lvl},1)
    depth_change = [depth_change; nan(size(Depth_change{pyr_lvl},1) - length(depth_change), 1)];
end
Depth_change{pyr_lvl} = [Depth_change{pyr_lvl} depth_change];

if 0

    figure(100),hist(Depth_change{pyr_lvl},20);grid on;
    
end

end



end
function schur = MargPointsAll()
global block;

blk = block;
Hsc = blk.Hpp;
bsc = blk.bp;

schur.Hsc = Hsc - blk.Hpm * blk.Hmm_inv * blk.Hpm';
% schur.Hsc = Hsc - blk.Hpm * diag(blk.Hmm) * blk.Hpm';



schur.Hpp = schur.Hsc;
% FillUpperTriangular()

Hsc_new =MakeSymmetric(schur.Hsc);


schur.bsc = bsc - blk.Hpm * blk.Hmm_inv *blk.bm;
% schur.bsc = bsc - blk.Hpm * diag(blk.Hmm) *blk.bm;
schur.bp = schur.bsc;
block = blk;



end
function FillUpdatedDepthsInDepthMap()

end

function [depth_meas, reproj_error] = Adjust(pyr_lvl, frame_i, frame_j , host_img, host_depth,host_depth_gt, target_img, target_depth, target_depth_gt,Twh, Twt,Twh_gt, Twt_gt, alpha_h, beta_h, alpha_t, beta_t, intrMat)
global point_id block point_id_offset Depth_meas last_point_id est_affine est_depth use_half iter inverse_comp switch_jac ...
    use_weight compensate_ab pyr_level;

% inverse_comp = true; false; true;  false; true; true; false;


if (inverse_comp)
%     switch_jac = true;
else 
    switch_jac = false;
end


new_inverse_comp = false; true; false; true;
if new_inverse_comp
   inverse_comp = 1; 
end
Tth = inv(Twt) * Twh;
% Tth = Tth *  [rodrigues([0.01 -0.01 0.01 ]) 20.*(rand(3,1)-0.5); 0 0 0 1];
Tth_gt = inv(Twt_gt) * Twh_gt;
fx = intrMat(1,1);
fy = intrMat(2,2);
cx = intrMat(1,3);
cy = intrMat(2,3);
width = size(host_img, 2);
height = size(host_img, 1);
% point_host = detectFASTFeatures(uint8(host_img),'MinQuality',0.4,'MinContrast',0.45);
point_host = detectFASTFeatures(uint8(host_img),'MinQuality',0.5,'MinContrast',0.5);
thr = 0.5;
thr_step = 0.01;
num_thr = [90 50 30 20 20 20];

num_thr = num_thr(1:pyr_level);
num_thr = fliplr(num_thr);

while (size(point_host.Location,1) < num_thr(pyr_lvl)) && thr > 0.01
    point_host = detectFASTFeatures(uint8(host_img),'MinQuality',thr,'MinContrast',thr);
    thr = thr - thr_step;
end
pt_host = double(point_host.Location);
valid_mask = ~isinf(host_depth);
SE = strel('cube',20);
valid_mask_use = imerode(valid_mask,SE);
ind_all = sub2ind(size(host_img), pt_host(:,2), pt_host(:,1));
ind_valid = find(valid_mask_use);
ind_use = intersect(ind_all, ind_valid);

pt_host = pt_host(ismember(ind_all, ind_use),:);


[xMat, yMat] = meshgrid(1: width, 1: height);
pix = [xMat(:) yMat(:)];
pix_norm = pflat(inv(intrMat) * (pextend(pix')))';

pix_norm_mat = reshape(pix_norm, height, width, 3);
pix_norm_mat_x = pix_norm_mat(:,:,1);
pix_norm_mat_y = pix_norm_mat(:,:,2);

border = [15 7 7 7 7 7 7 ];

border1 = [50 25 12 12 12 12 12 ];

border = border(1:pyr_level);
border1 = border1(1:pyr_level);
border = fliplr(border);
border1 = fliplr(border1);


pt_host = pt_host(pt_host(:,1) > border(pyr_lvl) & pt_host(:,1)< width-border(pyr_lvl) & pt_host(:,2) > border(pyr_lvl) & pt_host(:,2)< height-border(pyr_lvl),: );

if 0
    figure,imshow(host_img, []);hold on;plot(pt_host(:,1), pt_host(:,2),'or','MarkerSize',5,'LineWidth',5);
end

gx = zeros(size(host_img));
gy = gx;

gx(:,2 : width-1) = 0.5 * (host_img(:,3:width) - host_img(:, 1:width-2));
gy(2 : height-1, :) = 0.5 * (host_img(3:height,:) - host_img(1:height-2,:));

gx1 = zeros(size(target_img));
gy1 = gx1;

gx1(:,2 : width-1) = 0.5 * (target_img(:,3:width) - target_img(:, 1:width-2));
gy1(2 : height-1, :) = 0.5 * (target_img(3:height,:) - target_img(1:height-2,:));

% offset = [0 0; 0 1; 1 0; 0 -1; -1 0];
offset = [0 0; 0 2; 2 0; 0 -2; -2 0;-1 -1;1 -1;-1 1;1 1];

% est_affine = 0;1;

% Twc = inv(T_th);
cost_change = [];
delta_pose = [];
mean_reproj_err = [];
Match = {};

if frame_i == 1
    check_id = 2;
else
    check_id = 1;
end
    
if ~use_half
    if  size(Depth_meas{pyr_lvl,1}{frame_i,check_id},1) == 0
        % if  size(Depth_meas{frame_i,frame_i+1},1) == 0
        first_round = true;
        point_id(pyr_lvl) = last_point_id(pyr_lvl);
    end
else
    if  size(Depth_meas{pyr_lvl,1}{frame_i,frame_i+1},1) == 0
        first_round = true;
        point_id(pyr_lvl) = last_point_id(pyr_lvl);
    end
end
FrameHessian1.H = zeros(10, 10);
FrameHessian1.b = zeros(10,1);
FrameHessian1.Hii = zeros(10,10);
FrameHessian1.Hij = zeros(10,10);
FrameHessian1.Hjj = zeros(10,10);
FrameHessian1.bi = zeros(10,1);
FrameHessian1.bj = zeros(10,1);
FrameHessian1.c = 0;
depth_meas = [];
reproj_error=[];
for i = 1 : size(pt_host,1)
    pt0 = pt_host(i,:);
    pt0s = pt0 + offset;
    ind = sub2ind(size(host_img), pt0s(:,2), pt0s(:,1));
    pt3d = inv(intrMat)*[pt0 1]'* host_depth(ind(1));
    pt3d_gt = inv(intrMat)*[pt0 1]'* host_depth_gt(ind(1));
    pt3ds = (inv(intrMat)*[pt0s ones(size(offset,1),1)]')'.* host_depth(ind(1));
    
    X = pt3d(1);
    Y = pt3d(2);
    Z = pt3d(3);
    
    pt1 = pflat(intrMat * (Tth(1:3,1:3)*pt3d + Tth(1:3,4)));
    
    pt1_gt = pflat(intrMat * (Tth_gt(1:3,1:3)*pt3d_gt + Tth_gt(1:3,4)));
    pt1_gt = pt1_gt(1:2)';
    
    pt1_3d = (Tth(1:3,1:3)*pt3d + Tth(1:3,4));
    
    pt1s = pflat(intrMat * (Tth(1:3,1:3)*pt3ds' + repmat(Tth(1:3,4),1,size(offset,1))));
    pt1 = pt1(1:2)';
    pt1s = pt1s(1:2,:)';
    
    in_image = false;
    second_round = false;
    project_in_image = false;
    if pt1(1) > border1(pyr_lvl) && pt1(1)< width-border1(pyr_lvl) && pt1(2) > border1(pyr_lvl) && pt1(2)< height-border1(pyr_lvl)
        in_image = true;
        project_in_image = true;
    end
    pt_id = -10;
    if ~use_half
        if  size(Depth_meas{pyr_lvl,1}{frame_i,check_id},1) > 3
            
            %     if  size(Depth_meas{frame_i,frame_i+1},1) > 10
            second_round = true;
            if(ismember(ind(1), Depth_meas{pyr_lvl,1}{frame_i,check_id}(:,3)))
                %         if(ismember(ind(1), Depth_meas{frame_i,frame_i+1}(:,3)))
                if 0
                    id = find(ismember(Depth_meas{frame_i,frame_i+1}(:,3), ind(1)));
                    pt_id = Depth_meas{frame_i,frame_i+1}(id,5);
                else
                    id = find(ismember(Depth_meas{pyr_lvl,1}{frame_i,check_id}(:,3), ind(1)));
                    pt_id = Depth_meas{pyr_lvl,1}{frame_i,check_id}(id,5);
                end
                in_image = true;
            else
                in_image = false;
            end
        else
            first_round = true;
        end
    else
        %         if  size(Depth_meas{frame_i,check_id},1) > 10
        
        if  size(Depth_meas{pyr_lvl,1}{frame_i,frame_i+1},1) > 10
            second_round = true;
            %             if(ismember(ind(1), Depth_meas{frame_i,check_id}(:,3)))
            if(ismember(ind(1), Depth_meas{pyr_lvl,1}{frame_i,frame_i+1}(:,3)))
                if 1
                    id = find(ismember(Depth_meas{pyr_lvl,1}{frame_i,frame_i+1}(:,3), ind(1)));
                    pt_id = Depth_meas{pyr_lvl,1}{frame_i,frame_i+1}(id,5);
                else
                    id = find(ismember(Depth_meas{frame_i,check_id}(:,3), ind(1)));
                    pt_id = Depth_meas{frame_i,check_id}(id,5);
                end
                in_image = true;
            else
                in_image = false;
            end
        else
            first_round = true;
        end
    end
    if (~project_in_image && in_image)
        dsfkjg = 1;
    end
    
    %     if in_image  % && project_in_image
    if iter == 0
        judge = in_image && project_in_image;
    else 
        judge = in_image; % && project_in_image
    end
    if judge
%         project_in_image==false && second_round == true
        if(~second_round)
            point_id(pyr_lvl) = point_id(pyr_lvl) + 1;
        else
            assert(pt_id > 0);
            point_id(pyr_lvl) = pt_id;
        end
        depth_meas = [depth_meas;  [host_depth(ind(1)) frame_i ind(1) host_depth_gt(ind(1)) point_id(pyr_lvl)]];
        reproj_error = [reproj_error;[pt1_gt-pt1]];
        warped_intensities = interp2(double(target_img), pt1s(:, 1), pt1s(:, 2), 'linear', 0);
        if (sum(warped_intensities) <= 1)
           sfsfkj = 1; 
        end
        if ~inverse_comp
            warped_gx = interp2(double(gx1), pt1s(:, 1), pt1s(:, 2), 'linear', 0);
            warped_gy = interp2(double(gy1), pt1s(:, 1), pt1s(:, 2), 'linear', 0);
        end
        if 0
            figure, subplot(1,2,1);imshow(host_img, []);hold on;plot(pt0s(:,1),pt0s(:,2),'.r');  subplot(1,2,2);imshow(target_img, []);hold on;plot(pt1s(:,1),pt1s(:,2),'.r');
        end
        PatchInfo0.pt0s = pt0s;
        PatchInfo0.intensity = host_img(ind);
        if inverse_comp
            PatchInfo0.gx = gx(ind);
            PatchInfo0.gy = gy(ind);
        else
            PatchInfo0.gx = warped_gx;
            PatchInfo0.gy = warped_gy;
        end
        PatchInfo0.idepth = 1/host_depth(ind(1));
        PatchInfo0.pt_norm = [pix_norm_mat_x(ind) pix_norm_mat_y(ind)];
        
        d_uv_host_d_pt3d_host = [fx/Z 0 -fx*X/Z/Z; 0 fy/Z -fy*Y/Z/Z];
        d_pt3d_target_d_Tth = [-Tth(1:3,1:3)*SkewSymMat(pt3d) Tth(1:3,1:3)];
        
        
        d_uv_host_d_pt3d_host = d_uv_host_d_pt3d_host;
        
        d_uv_target_d_pt3d_target_comp = d_uv_host_d_pt3d_host * Tth(1:3,1:3)';
        
        d_uv_target_d_Tth = d_uv_target_d_pt3d_target_comp * d_pt3d_target_d_Tth;
        
        PatchInfo0.JacGeo = d_uv_target_d_Tth; %d_uv_d_pt3d * d_pt3d_d_host;
        PatchInfo0.JacPhoto = [gx(ind) gy(ind)];
        %             PatchInfo0.d_photo_d_host = PatchInfo.JacPhoto * PatchInfo0.JacGeo;
        %             PatchInfo0;
        % TODO host相对target的相对光照系数 calcRelative
        if ~switch_jac
            e_a1ma0 = exp(alpha_t) / exp(alpha_h);
            % TODO 图像0下的根据当前ab补偿后的灰度值
            patch0_comp = e_a1ma0 * (host_img(ind) - beta_h);
            rs = ((warped_intensities - beta_t) - patch0_comp);
        else
            e_a0ma1 = exp(alpha_h) / exp(alpha_t);
            % TODO 图像1下的根据当前ab补偿后的灰度值
            patch2_comp = e_a0ma1 * (warped_intensities - beta_t);
            rs = (host_img(ind) - beta_h) - patch2_comp;
        end
        rs2 = rs.^2;
        %%
        if ~switch_jac
            A0t(1, :) = patch0_comp';
            A0t(2, :) = (e_a1ma0);
            A1t(1, :) = -patch0_comp';
            A1t(2, :) = (-1);
        else
            A0t(1, :) = -patch2_comp';
            A0t(2, :) = -1;
            A1t(1, :) = patch2_comp';
            A1t(2, :) = (e_a0ma1);
        end
        if inverse_comp
            It = [gx(ind) gy(ind)]';
            if compensate_ab
                if switch_jac
                    It = 1./  e_a0ma1 * It;
                else
                    It =  e_a1ma0 * It;
                end
            end
        else
            It = [warped_gx warped_gy]';
            if compensate_ab
                It = 1./  e_a1ma0 * It;
            end
        end
        
        [~,gs] = NormalizeVector(It');
        gs2 = gs.^2;
        w = CalcWeight(rs2, gs2);
        if ~use_weight
            w = ones(size(w));
        end
        % PatchHessian1的生命周期只存在与单个点的处理中，跨点了就销毁老的PatchHessian1，重新新建一个新的PatchHessian1
        PatchHessian1 =  SetI(It, rs, w);
        
        
        %%
        PatchHessian1 = SetA(PatchHessian1, It, A0t, A1t, rs, w);
        
        %             dn_dx = PatchInfo0.JacGeo;
        
        d_uv_host_d_X_host = d_uv_host_d_pt3d_host;
        pt1_scaled = Tth(1:3,1:3)*(pt3d./pt3d(3)) + 1/pt3d(3)*Tth(1:3,4);
        X_target_scaled = pt1_scaled(1);
        Y_target_scaled = pt1_scaled(2);
        Z_target_scaled = pt1_scaled(3);
        d_uv_target_d_X_target = [fx/Z_target_scaled            0               -fx*X_target_scaled/Z_target_scaled/Z_target_scaled; ...
            0            fy/Z_target_scaled       -fy*Y_target_scaled/Z_target_scaled/Z_target_scaled];
        
        %%
        if ~new_inverse_comp
            if 0
                if inverse_comp
                    d_uv_target_d_X_target_scaled = d_uv_host_d_X_host * Tth(1:3,1:3)';
                else
                    d_uv_target_d_X_target_scaled = d_uv_target_d_X_target;
                end
                
                d_X_target_d_R_target = SkewSymMat(pt1_3d);
                d_X_target_d_t_target = -eye(3);
                d_X_target_d_pose_target = [d_X_target_d_R_target d_X_target_d_t_target];
                
                d_X_target_d_R_host = -Tth(1:3,1:3)*SkewSymMat(pt3d);
                d_X_target_d_t_host = Tth(1:3,1:3);
                d_X_target_d_pose_host = [d_X_target_d_R_host d_X_target_d_t_host];
                
                d_X_target_d_idepth = -Tth(1:3,1:3)*pt3d/host_depth(ind(1));
                
            else
                
                d_idepth_d_X_host = [0 0 -1/Z/Z];
                
                d_X_target_scaled_d_X_host = (Tth(1:3,1:3)/pt3d(3) + pt1_3d * d_idepth_d_X_host);
                
                pt3d_norm = pt3d./pt3d(3);
                d_uv_host_d_X_host_norm =  [fx/pt3d_norm(3) 0 -fx*pt3d_norm(1)/pt3d_norm(3)/pt3d_norm(3); ...
                    0 fy/pt3d_norm(3) -fy*pt3d_norm(2)/pt3d_norm(3)/pt3d_norm(3)];
                
                d_X_target_scaled_d_X_host_norm = Tth(1:3,1:3)/pt3d(3);
                d_X_target_scaled_d_X_host_norm_2 = Tth(1:3,1:3);
                [~, d_X_target_scaled_d_X_host_] = D_X_target_scaled_D_X_host(pt3d, Tth);
                %             d_X_target_scaled_d_X_host = Tth(1:3,1:3)/pt3d(3);
                d_X_target_scaled_d_X_host_check =  [d_idepth_d_X_host* pt1_3d(1);d_idepth_d_X_host* pt1_3d(2);d_idepth_d_X_host* pt1_3d(3)] + Tth(1:3,1:3)/pt3d(3);
                d_X_target_scaled_d_X_host_check_simple = Tth(1:3,4) * d_idepth_d_X_host;
                %                 d_uv_target_d_X_target_scaled = d_uv_host_d_X_host * inv(1/pt3d(3)*Tth(1:3,1:3));
                if inverse_comp
                    d_uv_target_d_X_target_scaled_check = d_uv_host_d_X_host * inv(d_X_target_scaled_d_X_host_norm); % 或需要乘以 (-1)
                    d_uv_target_d_X_target_scaled = d_uv_host_d_X_host_norm * inv(d_X_target_scaled_d_X_host_norm_2); % 或需要乘以 (-1)
                    if switch_jac
                        d_uv_target_d_X_target_scaled = -d_uv_target_d_X_target_scaled;
                    end
                    %                 d_uv_target_d_X_target_scaled = d_uv_host_d_X_host * inv(d_X_target_scaled_d_X_host_check_simple); % 或需要乘以 (-1)
                    
                else
                    d_uv_target_d_X_target_scaled = d_uv_target_d_X_target;
                end
                
                d_X_target_d_R_target = 1/pt3d(3)*SkewSymMat(pt1_3d);
                d_X_target_d_t_target = -1/pt3d(3)*eye(3);
                d_X_target_d_pose_target = [d_X_target_d_R_target d_X_target_d_t_target];
                
                d_X_target_d_R_host = -Tth(1:3,1:3)*SkewSymMat(pt3d./pt3d(3));
                d_X_target_d_t_host = 1/pt3d(3)*Tth(1:3,1:3);
                d_X_target_d_pose_host = [d_X_target_d_R_host d_X_target_d_t_host];
                
                d_X_target_d_idepth = Tth(1:3,4);
            end
            
            if ~est_depth
                d_X_target_d_idepth = zeros(3,1);
            end
            
            
            d_uv_target_d_pose_host = d_uv_target_d_X_target_scaled * d_X_target_d_pose_host;
            d_uv_target_d_pose_target = d_uv_target_d_X_target_scaled * d_X_target_d_pose_target;
            d_uv_target_d_idepth = d_uv_target_d_X_target_scaled * d_X_target_d_idepth;
            
            asksa = 1;
        else
            
            d_X_target_d_R_target = 1/pt3d(3)*SkewSymMat(pt1_3d);
            d_X_target_d_t_target = -1/pt3d(3)*eye(3);
            d_X_target_d_pose_target = [d_X_target_d_R_target d_X_target_d_t_target];
            
            d_X_target_d_R_host = -Tth(1:3,1:3)*SkewSymMat(pt3d./pt3d(3));
            d_X_target_d_t_host = 1/pt3d(3)*Tth(1:3,1:3);
            d_X_target_d_pose_host = [d_X_target_d_R_host d_X_target_d_t_host];
            
            d_X_target_d_idepth = Tth(1:3,4);
            if ~est_depth
                d_X_target_d_idepth = zeros(3,1);
            end
            
            P_ts = Tth(1:3,1:3)*(pt3d./Z) + (1/Z)* Tth(1:3,4);
            d_P_ts_d_bearing_h = Tth(1:3,1:3);
            if 0
                d_bearing_ts_d_P_ts = [1/P_ts(3)     0      -P_ts(1)/P_ts(3)/P_ts(3);
                    0     1/P_ts(3)  -P_ts(2)/P_ts(3)/P_ts(3)];
                
                d_bearing_ts_d_bearing_h = d_bearing_ts_d_P_ts * d_P_ts_d_bearing_h;
                d_bearing_ts_d_bearing_h = [d_bearing_ts_d_bearing_h;0 0 1];
            end
            P_ts_norm = P_ts./P_ts(3);
            pt3d_norm = pt3d./pt3d(3);
            d_uv_h_d_bearing_h = [1/pt3d_norm(3)     0      -pt3d_norm(1)/pt3d_norm(3)/pt3d_norm(3);
                                       0     1/pt3d_norm(3)  -pt3d_norm(2)/pt3d_norm(3)/pt3d_norm(3)];
            
%             pt_host_norm = pt3d./pt3d(3);
%             d_uv_h_d_bearing_h = []
            
            
            d_uv_t_d_P_ts = [d_uv_h_d_bearing_h] * inv(d_P_ts_d_bearing_h);
            
            d_uv_target_d_pose_host = d_uv_t_d_P_ts * d_X_target_d_pose_host;
            d_uv_target_d_pose_target = d_uv_t_d_P_ts * d_X_target_d_pose_target;
            d_uv_target_d_idepth = d_uv_t_d_P_ts * d_X_target_d_idepth;
        end
        %%
        FrameHessian1 = AddPatchHessLocal(FrameHessian1, PatchHessian1, d_uv_target_d_pose_host, d_uv_target_d_pose_target , est_affine);
        AddPatchHessGlobal(PatchHessian1, d_uv_target_d_pose_host, d_uv_target_d_pose_target ,d_uv_target_d_idepth, [frame_i frame_j point_id(pyr_lvl)], est_affine);
    end
    
end

AddFrameHess(FrameHessian1, [frame_i frame_j], depth_meas);

if 0
    figure,imshow(abs(block.Hpp),[]);
end

if ~second_round
    last_point_id(pyr_lvl) = point_id(pyr_lvl);
end

end
function a = CalcWeight(r2, g2)
wi =1;
c2 = 2;
dof = 4;
wg = c2 ./ (c2 + g2);
a = wg * (wi * (dof + 1)) ./ (dof + r2 .* wg);
end
function OptimizeFlow(host,depth_host, target, T_th, T_th_gt, intrMat, alpha0, beta0, alpha1, beta1)

target0 = target;
% target = 0.5*target0 + 50;
if 1
    target = 1.5*target0 - 100;
    target(target>255) = 255;
end

fx = intrMat(1,1);
fy = intrMat(2,2);
cx = intrMat(1,3);
cy = intrMat(2,3);


width = size(host, 2);
height = size(host, 1);

point_host = detectFASTFeatures(uint8(host),'MinQuality',0.4,'MinContrast',0.4);

%  [pix,imggggg] = detectEdge(uint8(host),size(host),vec);
if 1
    pt_host = double(point_host.Location);
else
    [pt_host,imggggg] = detectEdge(uint8(host),ones(size(host)),[0.1 0.2 ]);
end

if 0
    figure,imshow(host, []);hold on;plot(pt_host(:,1), pt_host(:,2),'.r')
end

[xMat, yMat] = meshgrid(1: width, 1: height);
pix = [xMat(:) yMat(:)];
pix_norm = pflat(inv(intrMat) * (pextend(pix')))';

pix_norm_mat = reshape(pix_norm, height, width, 3);
pix_norm_mat_x = pix_norm_mat(:,:,1);
pix_norm_mat_y = pix_norm_mat(:,:,2);


pt_host = pt_host(pt_host(:,1) > 10 & pt_host(:,1)< 620 & pt_host(:,2) > 10 & pt_host(:,2)< 460,: );

gx = zeros(size(host));
gy = gx;

gx(:,2 : width-1) = 0.5 * (host(:,3:width) - host(:, 1:width-2));
gy(2 : height-1, :) = 0.5 * (host(3:height,:) - host(1:height-2,:));


offset = [0 0; 0 1; 1 0; 0 -1; -1 0];


est_affine = 1;

Twc = inv(T_th);
cost_change = [];
delta_pose = [];
mean_reproj_err = [];
Match = {};
for iter = 1 : 100
    
    PatchInfo0 = {};
    
    
    if iter < 5
        est_affine = 0;
    else 
        est_affine = 1;
    end
    Match{iter,1} = [];
    % FrameHessian1 只会维持一次迭代，solve后hessian要重置，
    % dsol中T_w_cur，a， b在跨层时才会更新，在同一层的多次迭代中，它们都不会更新，关于affine的雅可比全不变
    % 同一层不不同迭代中pose当然要更新，不然target的投影点会一直不变了
    cnt = 1;
    T_th = inv(Twc);
    FrameHessian1.H = zeros(8,8);
    FrameHessian1.b = zeros(8,1);
    FrameHessian1.c = 0;
    for i = 1 : size(pt_host,1)
        
        pt0 = pt_host(i,:);
        pt0s = pt0 + offset;
        ind = sub2ind(size(host), pt0s(:,2), pt0s(:,1));
        pt3d = inv(intrMat)*[pt0 1]'* depth_host(ind(1));
        pt3ds = (inv(intrMat)*[pt0s ones(5,1)]')'.* depth_host(ind(1));
        
        X = pt3d(1);
        Y = pt3d(2);
        Z = pt3d(3);
        
        pt1 = pflat(intrMat * (T_th(1:3,1:3)*pt3d + T_th(1:3,4)));
        
        pt1_gt = pflat(intrMat * (T_th_gt(1:3,1:3)*pt3d + T_th_gt(1:3,4)));
        pt1_gt = pt1_gt(1:2)';
        
        pt1_3d = (T_th(1:3,1:3)*pt3d + T_th(1:3,4));
        
        pt1s = pflat(intrMat * (T_th(1:3,1:3)*pt3ds' + repmat(T_th(1:3,4),1,size(offset,1))));
        pt1 = pt1(1:2)';
        pt1s = pt1s(1:2,:)';
        
        %    FrameHessian1.H = zeros(8,8);
        %    FrameHessian1.b = zeros(8,1);
        %     FrameHessian1.c = 0;
        if pt1(1) > 10 && pt1(1)< width-10 && pt1(2) > 10 && pt1(2)< height-10
            
            
             Match{iter,1} = [ Match{iter,1}; [pt1 pt1_gt]];
            
            warped_intensities = interp2(double(target), pt1s(:, 1), pt1s(:, 2), 'linear', 0);
            
            if 0
                figure, subplot(1,2,1);imshow(host, []);hold on;plot(pt0s(:,1),pt0s(:,2),'.r');  subplot(1,2,2);imshow(target, []);hold on;plot(pt1s(:,1),pt1s(:,2),'.r');
            end
            PatchInfo0{cnt,1}.pt0s = pt0s;
            PatchInfo0{cnt,1}.intensity = host(ind);
            PatchInfo0{cnt,1}.gx = gx(ind);
            PatchInfo0{cnt,1}.gy = gy(ind);
            PatchInfo0{cnt,1}.idepth = 1/depth_host(ind(1));
            PatchInfo0{cnt,1}.pt_norm = [pix_norm_mat_x(ind) pix_norm_mat_y(ind)];
            
            d_uv_d_pt3d = [fx/Z 0 -fx*X/Z/Z; 0 fy/Z -fy*Y/Z/Z];
            d_pt3d_target_d_Tth = [-T_th(1:3,1:3)*SkewSymMat(pt3d) T_th(1:3,1:3)];
            
            
            d_uv_host_d_pt3d_host = d_uv_d_pt3d;
            
            d_uv_target_d_pt3d_target_comp = d_uv_host_d_pt3d_host * T_th(1:3,1:3)';
                        
            d_uv_target_d_Tth = d_uv_target_d_pt3d_target_comp * d_pt3d_target_d_Tth;
            
            PatchInfo0{cnt,1}.JacGeo = d_uv_target_d_Tth; %d_uv_d_pt3d * d_pt3d_d_host;
            PatchInfo0{cnt,1}.JacPhoto = [gx(ind) gy(ind)];
            PatchInfo0{cnt,1}.d_photo_d_host = PatchInfo0{cnt,1}.JacPhoto * PatchInfo0{cnt,1}.JacGeo;
            PatchInfo0{cnt,1};
            % TODO 1相对0的相对光照系数 calcRelative
            e_a0ma1 = exp(alpha0) / exp(alpha1);
            % TODO 图像1下的根据当前ab补偿后的灰度值
            patch2_comp = e_a0ma1 * (warped_intensities - beta1);
            rs = (host(ind) - beta0) - patch2_comp;
            It = [gx(ind) gy(ind)]';
            PatchHessian1 =  SetI(It, rs, 1);
            
            At(1, :) = patch2_comp';
            At(2, :) = (e_a0ma1);
            
            PatchHessian1 = SetA(PatchHessian1, It, At, rs, 1);
            
            dn_dx = PatchInfo0{cnt,1}.JacGeo;
            
            FrameHessian1 = AddPatchHess(FrameHessian1, PatchHessian1, dn_dx, est_affine);
            cnt = cnt + 1;
        end
    end
    
    [warped_image, valid_mask, warped_z, validMask] = warpImage(e_a0ma1 * (target - beta1), depth_host, inv(Twc), intrMat, intrMat, host-beta0);
    
    if 0
        figure(10),hold on;imshowpair(warped_image, host);drawnow;
        saveas(gcf,  sprintf('./dump/align_iter_%05d.png', iter));
    end
    
    if est_affine
        dx = inv(FrameHessian1.H) * FrameHessian1.b;
        
        
       
        
        Twc = Twc * [rodrigues(dx(1:3)) dx(4:6); 0 0 0 1];
        alpha1 = alpha1 + dx(7);
        beta1 = beta1 + dx(8);
    else
        dx = inv(FrameHessian1.H(1:6,1:6)) * FrameHessian1.b(1:6);
        Twc = Twc * [rodrigues(dx(1:3)) dx(4:6); 0 0 0 1];
    end
    cost_change = [cost_change; FrameHessian1.c];
    
    if (iter > 10)
        if(cost_change(end) > cost_change(end-1))
            break;
        end
    end
    
%     [T_th_gt inv(Twc) Twc * T_th_gt]
  [~,reproj_err] = NormalizeVector(Match{iter,1}(:,1:2) - Match{iter,1}(:,3:4));
    dlt_pose = Twc * T_th_gt;
    mean_reproj_err = [mean_reproj_err; mean(reproj_err)];
    delta_pose = [delta_pose; rad2deg(norm(rodrigues(dlt_pose(1:3,1:3)))) norm(dlt_pose(1:3,4))];
end

if 0
    figure,imshow([target e_a0ma1 * (target - beta1) host-beta0],[])
    
    
    figure,subplot(3,2,[1 2]);plot(cost_change);title('total loss (intensity^2)');grid on;
    subplot(3,2,3);plot(delta_pose(:,1));title('rot err (deg)');grid on;
    subplot(3,2,4);plot(delta_pose(:,2));title('trans err (mm)');grid on;
    subplot(3,2,[5 6]);plot(mean_reproj_err);grid on;title('reproj err (pixel)');
    
end

end

function AddFrameHess(FrameHessian1, ij, depth_meas)
global block point_id est_affine;
blk = block;

% hid = dd*(ijh(3)-1)+1;

df = 6 + 2*est_affine;

ii = df*(ij(1)-1)+1;  
jj = df*(ij(2)-1)+1;  

blk.Hpp = FillMatrix(blk.Hpp, df, df, ii, ii, FrameHessian1.Hii(1:df,1:df));
blk.Hpp = FillMatrix(blk.Hpp, df, df, jj, jj, FrameHessian1.Hjj(1:df,1:df));

if(ii < jj)
    blk.Hpp = FillMatrix(blk.Hpp, df, df, ii, jj, FrameHessian1.Hij(1:df,1:df));
%     blk.Hpp = FillMatrix(blk.Hpp, df, df, jj, ii, FrameHessian1.Hij(1:df,1:df)');
else
    blk.Hpp = FillMatrix(blk.Hpp, df, df, jj, ii, FrameHessian1.Hij(1:df,1:df)');
%     blk.Hpp = FillMatrix(blk.Hpp, df, df, ii, jj, FrameHessian1.Hij(1:df,1:df));
end

blk.bp = FillMatrix(blk.bp, df, 1, ii, 1, FrameHessian1.bi(1:df));
blk.bp = FillMatrix(blk.bp, df, 1, jj, 1, FrameHessian1.bj(1:df));

blk.c = blk.c+FrameHessian1.c;
block = blk;
end
function AddPatchHessGlobal( PatchHessian1, G0, G1 ,Gd, ijh, est_affine)
global block;



dp = 6;
dd = 1;
da = 2;
df = 6 + 2*est_affine;
% if0 = dp*(ijh(1)-1)+1;  
% if1 = dp*(ijh(2)-1)+1;  
% hid = dd*(ijh(3)-1)+1; 

if0 = df*(ijh(1)-1)+1;  
if1 = df*(ijh(2)-1)+1;  
hid = dd*(ijh(3)-1)+1; 

ph = PatchHessian1;
blk = block;
blk.Hpm = FillMatrix(blk.Hpm, dp, dd, if0, hid, G0' * ph.ItI * Gd);
%Hpm.block<dp, dd>(if1, hid).noalias() += G1.transpose() * ph.ItI * Gd;
blk.Hpm = FillMatrix(blk.Hpm, dp, dd, if1, hid, G1' * ph.ItI * Gd);

ss =  Gd' * ph.ItI * Gd;
if ss == 0
   dashk = 1;
end
aaa = Gd' * ph.ItI * Gd;
if((aaa) == 0)
   sdfgkhj = 1;  
end
if (hid == 202)

    asfkgh = 1;
end
blk.Hmm = FillMatrix(blk.Hmm, dd, dd, hid, 1, Gd' * ph.ItI * Gd);
% bm[hid] -= Gd.transpose() * ph.Itr;
blk.bm = FillMatrix(blk.bm, dd, dd, hid, 1, -Gd' * ph.Itr);

if est_affine
    affine_offset = 0; %2;
    cam_ind = 0;
    
   ia0 = if0 + dp;  % // aff_offset = 0, since it's always left
   ia1 = if1 + dp + affine_offset;
    
   blk.Hpm = FillMatrix(blk.Hpm, da, dd, ia0, hid, ph.ItA0' * Gd);
%Hpm.block<dp, dd>(if1, hid).noalias() += G1.transpose() * ph.ItI * Gd;
if 1
    blk.Hpm = FillMatrix(blk.Hpm, da, dd, ia1, hid, ph.ItA1' * Gd);
end

end
block = blk;
end
function Mat2 = FillMatrix(Mat1, rows, cols, start_row, start_col, data)


Mat1 = GrowMat(Mat1, rows, cols, start_row, start_col);
Mat1(start_row:start_row+rows-1, start_col:start_col+cols-1) = Mat1(start_row:start_row+rows-1, start_col:start_col+cols-1) + data;

Mat2 = Mat1;
end
function Mat2 = GrowMat(Mat1, rows, cols, start_row, start_col)
if size(Mat1,1) < start_row + rows -1
    pad_row = start_row + rows -1 - size(Mat1,1);
    Mat1 = [Mat1; zeros(pad_row, size(Mat1,2))];
%     Mat1 = [Mat1 zeros(size(Mat1,1), pad_row)];
end

if size(Mat1,2) < start_col + cols -1
    pad_col = start_col + cols -1 - size(Mat1,2);
    Mat1 = [Mat1 zeros(size(Mat1,1), pad_col)];
%     Mat1 = [Mat1; zeros(pad_col, size(Mat1,2))];
end
Mat2 = Mat1;

end
function FrameHessian2 = AddPatchHessLocal(FrameHessian1, ph, G0,G1, est_affine)
% 构造hession 和 b
pose_offset = 6;
dp = 6;
da = 2;

FrameHessian1.Hii(1:pose_offset, 1:pose_offset) = FrameHessian1.Hii(1:pose_offset, 1:pose_offset) + G0' * ph.ItI * G0;
FrameHessian1.Hij(1:pose_offset, 1:pose_offset) = FrameHessian1.Hij(1:pose_offset, 1:pose_offset) + G0' * ph.ItI * G1;
FrameHessian1.Hjj(1:pose_offset, 1:pose_offset) = FrameHessian1.Hjj(1:pose_offset, 1:pose_offset) + G1' * ph.ItI * G1;

FrameHessian1.bi(1:pose_offset) = FrameHessian1.bi(1:pose_offset) - G0' * ph.Itr;
FrameHessian1.bj(1:pose_offset) = FrameHessian1.bj(1:pose_offset) - G1' * ph.Itr;

FrameHessian1.c = FrameHessian1.c + ph.r2;

if est_affine
    %% TODO  We only fill the upper-triangular part for now and make it symmetric later before solving
    
    if 0
        affine_offset = 2;
        cam_ind = 0;1;0;%1;
        FrameHessian1.Hii(pose_offset+1: pose_offset+affine_offset,1:pose_offset) =  FrameHessian1.Hii(pose_offset+1: pose_offset+affine_offset,1:pose_offset) + ph.ItA0' * G0;
        FrameHessian1.Hii(1:pose_offset, pose_offset+1: pose_offset+affine_offset) = FrameHessian1.Hii(pose_offset+1: pose_offset+affine_offset,1:pose_offset)';
        FrameHessian1.Hii(pose_offset+1: pose_offset+affine_offset,pose_offset+1: pose_offset+affine_offset) = FrameHessian1.Hii(pose_offset+1: pose_offset+affine_offset,pose_offset+1: pose_offset+affine_offset) +  ph.A0tA0;
        %% This is already an off-diagonal block so we need it to be full
        FrameHessian1.Hij(1:pose_offset, pose_offset+cam_ind*affine_offset+1: pose_offset+(cam_ind+1)*affine_offset) = FrameHessian1.Hij(1:pose_offset, pose_offset+cam_ind*affine_offset+1: pose_offset+(cam_ind+1)*affine_offset) + G0' * ph.ItA1;
        FrameHessian1.Hij(pose_offset+1: pose_offset+affine_offset,1:pose_offset) =  FrameHessian1.Hij(pose_offset+1: pose_offset+affine_offset,1:pose_offset) + ph.ItA0' * G1;
        FrameHessian1.Hij(pose_offset+1: pose_offset+affine_offset,pose_offset+cam_ind*affine_offset+1: pose_offset+(cam_ind+1)*affine_offset) = FrameHessian1.Hij(pose_offset+1: pose_offset+affine_offset,pose_offset+cam_ind*affine_offset+1: pose_offset+(cam_ind+1)*affine_offset) +  ph.A0tA1;
        %%
        FrameHessian1.Hjj(pose_offset+cam_ind*affine_offset+1: pose_offset+(cam_ind+1)*affine_offset,1:pose_offset) =  FrameHessian1.Hjj(pose_offset+cam_ind*affine_offset+1: pose_offset+(cam_ind+1)*affine_offset,1:pose_offset) + ph.ItA1' * G1;
        FrameHessian1.Hjj(1:pose_offset, pose_offset+cam_ind*affine_offset+1: pose_offset+(cam_ind+1)*affine_offset) = FrameHessian1.Hjj(1:pose_offset, pose_offset+cam_ind*affine_offset+1: pose_offset+(cam_ind+1)*affine_offset) + G1' * ph.ItA1;
        FrameHessian1.Hjj(pose_offset+cam_ind*affine_offset+1: pose_offset+(cam_ind+1)*affine_offset,pose_offset+cam_ind*affine_offset+1: pose_offset+(cam_ind+1)*affine_offset) = FrameHessian1.Hjj(pose_offset+cam_ind*affine_offset+1: pose_offset+(cam_ind+1)*affine_offset,pose_offset+cam_ind*affine_offset+1: pose_offset+(cam_ind+1)*affine_offset) +  ph.A1tA1;
        
        
        FrameHessian1.bi(pose_offset+1: pose_offset+affine_offset) = FrameHessian1.bi(pose_offset+1: pose_offset+affine_offset)-ph.A0tr;
        FrameHessian1.bj(pose_offset+cam_ind*affine_offset+1: pose_offset+(cam_ind+1)*affine_offset) = FrameHessian1.bj(pose_offset+cam_ind*affine_offset+1: pose_offset+(cam_ind+1)*affine_offset)-ph.A1tr;
    else
         affine_offset = 0;2;0;2;%0;
         ia0 = dp+1;
         ia1 = dp + affine_offset+1;
%          FrameHessian1.Hii = FillMatrix(FrameHessian1.Hii, da, dp, ia0, 1, ph.ItA0' * G0);
         FrameHessian1.Hii = FillMatrix(FrameHessian1.Hii, dp, da, 1, ia0, G0' * ph.ItA0);
         FrameHessian1.Hii = FillMatrix(FrameHessian1.Hii, da, da, ia0, ia0, ph.A0tA0);
         
         FrameHessian1.Hij = FillMatrix(FrameHessian1.Hij, dp, da, 1, ia1,  G0' * ph.ItA1);
         FrameHessian1.Hij = FillMatrix(FrameHessian1.Hij, da, dp, ia0, 1,ph.ItA0' * G1);
         FrameHessian1.Hij = FillMatrix(FrameHessian1.Hij, da, da, ia0, ia1, ph.A0tA1);
         
%          FrameHessian1.Hjj = FillMatrix(FrameHessian1.Hjj, da, dp, ia1, 1, ph.ItA1' * G1);
         FrameHessian1.Hjj = FillMatrix(FrameHessian1.Hjj, dp, da, 1, ia1, G1' * ph.ItA1);
         FrameHessian1.Hjj = FillMatrix(FrameHessian1.Hjj, da, da, ia1, ia1, ph.A1tA1);
         
         FrameHessian1.bi = FillMatrix(FrameHessian1.bi, da, 1, ia0, 1, -ph.A0tr);
         FrameHessian1.bj = FillMatrix(FrameHessian1.bj, da, 1, ia1, 1, -ph.A1tr);
    end
end
FrameHessian2 = FrameHessian1;
end

function A = SetI(It, r, w)
wr = w .* r;
A.ItI = It * diag(w) * It';
A.Itr = It * wr;
A.r2 = sum(r .* r);
A.wr2 = sum(wr .* r);
end
function B = SetA(A,It,A0t,A1t, r, w)

wA0 = diag(w) * A0t';
wA1 = diag(w) * A1t';

A.ItA0 = It * wA0;
A.ItA1 = It * wA1;

A.A0tA0 = A0t * wA0;
A.A0tA1 = A0t * wA1;
A.A1tA1 = A1t * wA1;

wr = w.*r;
A.A0tr = A0t * (wr);
A.A1tr = A1t * (wr);

B = A;
end

function UpdateHess()


end
function [X_target_scaled, d_X_target_scaled_d_X_host] = D_X_target_scaled_D_X_host(X_host, Tth)

X_target_scaled = 1/X_host(3)*(Tth(1:3,1:3)*X_host + Tth(1:3,4));
X_target = Tth(1:3,1:3)*X_host + Tth(1:3,4);
d_idepth_d_X_host = [0 0 -1/X_host(3)/X_host(3)];
                
d_X_target_scaled_d_X_host = (Tth(1:3,1:3)/X_host(3) + X_target * d_idepth_d_X_host);

 
end
function Hpp1 = MakeSymmetric(Hpp0)

Hpp = Hpp0;

    upper_ = triu(Hpp);
    lower_ = tril(Hpp);
    Hpp = tril(Hpp);
    Hpp = Hpp';
    
    mask = Hpp ~= 0;
    
    Hpp1 = Hpp + immultiply(~mask, Hpp');
    
    if 0
        mask1 = zeros(size(Hpp));
        tril
        mask__ = Hpp ~= 0;
        
        mask_ = (lower_~=0);
        mask_(logical(eye(size(mask_)))) = 0;
        Hpp(mask_) = 0;
        Hpp = Hpp + immultiply(mask_, Hpp');
        
        lower = triu(Hpp)';
        mask = Hpp==0;
        lower1 = immultiply(lower, mask);
        %% 把上三角对折下来
        Hpp1 = Hpp + lower1;
    end
end
% function [bearing, d] = UnprojectPinhole(pix, fx, fy,cx, cy)
% 
% 
% 
%  c0(0) = (norm_inv + 2 * mx * mx * d_norm_inv_d_r2) / fx;
%     c0(1) = (2 * my * mx * d_norm_inv_d_r2) / fx;
%     c0(2) = 2 * mx * d_norm_inv_d_r2 / fx;
% 
%     c1(0) = (2 * my * mx * d_norm_inv_d_r2) / fy;
%     c1(1) = (norm_inv + 2 * my * my * d_norm_inv_d_r2) / fy;
%     c1(2) = 2 * my * d_norm_inv_d_r2 / fy;
% 
% end