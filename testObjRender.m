function testObjRender()

global textureFile

num_levels = 3; 1; 3; 2; 4; 2; 3; 2; 4; 3; 4; 2; 3; 1; 2; 4;  6; 4; 2;
areaRatio = 0.08; 0.001; 0.08; 0.25; 0.08; 0.005;
initial_sigma = 5;
default_dof = 5;
para = 5;
isVerbose = 0;

intrMat = [554.256258 0 320.5; 0 554.256258 240.5; 0 0 1];
% intrMat = [300 0 320.5; 0 300 240.5; 0 0 1];

[xMat, yMat] = meshgrid(1 : 640, 1 : 480);

pixAll = [xMat(:), yMat(:)];

pixAll_un = inv(intrMat)*pextend(pixAll');

b2c = [eye(3) [10;45;-170.2];0 0 0 1];

% b2c = eye(4);
if 0
    textureFile = 'C:\Users\rongjiezhu\Desktop\bg6.jpg';
    textureFile = 'C:\Users\rongjiezhu\Desktop\ros1.jpg';
    textureFile = 'C:\Users\rongjiezhu\Desktop\bg3.jpg';
    textureFile = 'C:\Users\rongjiezhu\Desktop\bg9.jpg';
    textureFile = 'C:\Users\rongjiezhu\Desktop\bg16.jpg';
    textureFile = 'C:\Users\rongjiezhu\Desktop\bg17.jpg';
    textureFile = 'C:\Users\rongjiezhu\Desktop\bg.jpg';
    textureFile = 'C:\Users\rongjiezhu\Desktop\bg2.jpg';
    textureFile = 'C:\Users\rongjiezhu\Desktop\bg3.jpg';
    % textureFile = 'C:\Users\rongjiezhu\Desktop\11.jpg';
    textureFile = 'C:\Users\rongjiezhu\Desktop\bg13.jpg';
    
    textureFile = 'C:\Users\rongjiezhu\Desktop\bg2.jpg';
end
textureFile = 'E:\daily\E\bk_20180627\SLAM\slam_algo\bg3.jpg';
textureFile = 'E:\daily\E\bk_20180627\SLAM\slam_algo\bg7.jpg';
textureFile = 'E:\daily\E\bk_20180627\SLAM\slam_algo\bg8.jpg';
% % textureFile = 'E:\daily\E\bk_20180627\SLAM\slam_algo\bg15.jpg';
textureFile = 'G:\matlab\LK\slam_algo\bg16.jpg';
% textureFile = 'G:\matlab\LK\slam_algo\bg18.jpg';



genVid = false;

% obj_test = MR_obj_read('C:\Users\rongjiezhu\Documents\WXWork\1688852647983342\Cache\File\2020-05\÷Ï\01.obj');
% obj_test = MR_obj_read('C:\Users\rongjiezhu\Desktop\÷Ï\07.obj');

% obj_test = MR_obj_read('C:\Users\rongjiezhu\Desktop\÷Ï\01.obj');

obj_test = MR_obj_read('G:\matlab\LK\slam_algo\÷Ï\01.obj');
% obj_test.V = 3.*obj_test.V./7.*5;
obj_test.V = 1.*obj_test.V./7.*5;
depthThr = 60;%200; 2000; 1500;
obj_test.V = (rotx(90)*obj_test.V')';


% L2R = [eye(3) [-20;0;0];0 0 0 1];
L2R = [eye(3) [-100;0;0];0 0 0 1];
L2R = [eye(3) [-500;0;0];0 0 0 1];

% L2R = [eye(3) [-100;0;0];0 0 0 1];
L2R = [eye(3) [-20;0;0];0 0 0 1];

t = [-166;60;0];

T0 = [eye(3) t; 0 0 0 1];

% aviobj = VideoWriter(fullfile(pwd,'demo.avi'), 'Indexed AVI');
if genVid
    aviobj = VideoWriter(fullfile(pwd,'demo.avi'));
    aviobj.FrameRate = 5;
    open(aviobj);
end

if 0
    angList = 0 : -2.6 : -900;
else
    angList = 0 : 2*atand(intrMat(1,3)/intrMat(1,1)) : 2*atand(intrMat(1,3)/intrMat(1,1));  % -900;
    angList = 0 : 55 : 55;
    angStep = -(2*atand(intrMat(1,3)/intrMat(1,1))-10);
%     angStep = 20;
    angStep = 10; 2; %20;%5;
    angList = 0 : angStep : 30*angStep;  % -900;
    T0 = [roty(270) t; 0 0 0 1];
    b2c = [eye(3) [2;45;-100];0 0 0 1];
    
    b2c = [eye(3) [2;800;-33];0 0 0 1];
    if 0
        b2c = [rodrigues([0.1 0.2 -0.1]) [0.6;800;-33];0 0 0 1];
    else
        b2c = [rodrigues([0.1 0.2 -0.1]) [0.6;1500;-33];0 0 0 1];
    end
% %     b2c = eye(4);
    intrMatNew = intrMat;
    scale = 1; 0.5;
    ppOfst = [1500 300];
    
    intrMatNew(1,1) = scale*intrMat(1,1);
    intrMatNew(2,2) = scale*intrMat(1,1);
    
    intrMatNew(1,3) = intrMat(1,3) + ppOfst(1);
    intrMatNew(2,3) = intrMat(2,3) + ppOfst(2);
    
end

% k2c_comp{1,1} = eye(4);

trans = zeros(length(angList),3);
trans_list = 1:20:50*length(angList);
if 0
    trans(:,3) = trans_list;
elseif 0
    trans(:,1) = trans_list;
else
    segment = round(length(trans_list)/3);
    segment = 5;
    trans0  = trans(1:segment,:);
    trans_ = trans0;
    trans_(1:segment,1) = trans_list(1:segment);
    trans_1 = trans_(1:segment,:);
    trans_2 = -trans_(1:segment,:);
    trans_ = trans0;
    trans_(1:segment,2) = trans_list(1:segment);
    trans_3 = trans_(1:segment,:);
    trans_4 = -trans_(1:segment,:);
    trans_ = trans0;
    trans_(1:segment,3) = trans_list(1:segment);
    trans_5 = trans_(1:segment,:);
    trans_6 = -trans_(1:segment,:);
%     trans_1 = trans(1:segment,1);
%     trans_2 = -trans(1:segment,1);
%     trans(segment+1:2*segment,2) = trans_list(segment+1:2*segment);
%     trans(2*segment+1:end,3) = trans_list(2*segment+1:end);
trans = [trans_1; trans_2; trans_3;trans_4;trans_5;trans_6];
end



for i = 1 : size(trans,1)
%     for i = 1 : length(angList)
    
    
    if 0
        RRR = roty(angList(i));
        T_delt = [RRR [0;0;0]; 0 0 0 1];
    else
        T_delt = [rodrigues(0.05.*(rand(3,1)-0.5)) trans(i,:)'; 0 0 0 1];
    end
    k2c_comp{i,1} = b2c * T_delt * inv(b2c);
    if i == 1
        deltaPoseMat = [reshape(eye(3), 1, 9) 0 0 0];
        T1 = b2c*T_delt*T0;
        T1_0 = T1;
    else
        %         T1_0 = T1;
        T1 = b2c*T_delt*T0;
        
        deltaPose = inv(T1_0)*T1;
        deltaPoseMat = [deltaPoseMat; [reshape(deltaPose(1:3,1:3),1,9) deltaPose(1:3,4)']];
    end
    
    %     continue;
    T1R = L2R*T1;
    
    [obj_test2, inlierId] = procObj(obj_test, depthThr, T1(1:3,1:3), T1(1:3,4));
    [xyzOrig22, imgCur022, depthMap22, Vcam22] = render_fcn(obj_test2, T1(1:3,1:3), T1(1:3,4));
    if 0
        pt = detectFASTFeatures(rgb2gray(imgCur022),'MinQuality',0.01,'MinContrast',0.01);
        figure,imshow(imgCur022);hold on;plot(pt.Location(:,1), pt.Location(:,2), '.r')
    end
    
    [obj_test2R, inlierIdR] = procObj(obj_test, depthThr, T1R(1:3,1:3), T1R(1:3,4));
    [xyzOrig22R, imgCur022R, depthMap22R, Vcam22R] = render_fcn(obj_test2R, T1R(1:3,1:3), T1R(1:3,4));
    
    Inlier{i,1} = inlierId;Inlier{i,2} = inlierIdR;
    
    LR{i,1} = imgCur022; LR{i,2} = imgCur022R;
    Depth{i,1} = depthMap22; Depth{i,2} = depthMap22R;
    VCam{i,1} = Vcam22; VCam{i,2} = Vcam22R;
    
    asgjkb = 1;
    if 0
        img = uint8(255.*imgCur022);
        figure;subplot(1,2,1);imshow(imgCur022);subplot(1,2,2);imshow(depthMap22, []);
        drawnow;
    end
    
    if genVid
        depth = cat(3, uint8(depthMap22./25),uint8(depthMap22./25),uint8(depthMap22./25));
        writeVideo(aviobj, [(img) depth]);
    end
    
    
dispMat = disparity(rgb2gray(imgCur022), rgb2gray(imgCur022R),'DisparityRange',[0 64]);
dispMat(dispMat < 0.1) = nan;
dispMat(dispMat > 63) = nan;
depthMat = intrMat(1,1)*abs(L2R(1,4))./dispMat;
depthList = depthMat(:);

xyz = repmat(depthList,1,3).*pixAll_un';
aa = pflat(intrMat*xyz');
err = (aa(1:2,:) - round(aa(1:2,:)))';


if 0
    figure,plot(err);
    figure,imshow(dispMat, []);
    
    
    figure,subplot(1,2,1),pcshow(xyz(xyz(:,3) < 10000,:)); subplot(1,2,2);hist(xyz(:,3),1000);
end
if(i == -2)
    pose_init = inv(k2c_comp{1,1}) * k2c_comp{2,1};
    pose_init_error = pose_init * [rodrigues([0.02 -0.01 0.03]) 2.*(rand(3,1)-0.5); 0 0 0 1];
    [pose_rel, score, warped_image1, valid_mask, weightMap, warped_z, validMask] = estimateVisualOdometry(double(rgb2gray(LR{2,1})), double(rgb2gray(LR{1,1})), Depth{2,1}, Depth{1,1}, intrMat, num_levels, pose_init_error, isVerbose,initial_sigma,default_dof,para);
    dPose = inv(pose_init) *  pose_rel;
    err_ratio = [norm(dPose(1:3,4))/norm(pose_init(1:3,4)) norm(rodrigues(dPose(1:3,1:3)))/norm(rodrigues(pose_init(1:3,1:3)))];  
    figure,subplot(1,2,1);imshowpair(warped_image1, double(rgb2gray(LR{1,1})));
    align_depth_error = abs(Depth{2,1} - warped_z);
    align_depth_error(align_depth_error > 20) = 0;
    align_depth_error_ratio = align_depth_error./Depth{2,1};
    subplot(1,2,2);imshow([align_depth_error align_depth_error_ratio],[]);
end

if (length(k2c_comp) >=  size(trans,1)) % 5)
    save('pba_adjust.mat','k2c_comp', 'LR', 'Depth', 'intrMat');
    testDirestAlign(k2c_comp, LR, Depth, intrMat);
end

end

% if (length(k2c_comp) >=3)
% testDirestAlign(k2c_comp, LR, Depth, intrMat);
% end
% img_curr, img_prev, dep_curr, dep_prev, K, num_levels, pose_init, isVerbose,initial_sigma,default_dof,para
% pose_init = inv(k2c_comp{1,1}) * k2c_comp{2,1};
% [pose_rel, score, warped_image1, valid_mask, weightMap, warped_z, validMask] = estimateVisualOdometry(double(rgb2gray(LR{2,1})), double(rgb2gray(LR{1,1})), Depth{2,1}, Depth{1,1}, intrMat, num_levels, pose_init, isVerbose,initial_sigma,default_dof,para);
% dPose = inv(pose_init) *  pose_rel;
% err_ratio = [norm(dPose(1:3,4))/norm(pose_init(1:3,4)) norm(rodrigues(dPose(1:3,1:3)))/norm(rodrigues(pose_init(1:3,1:3)))];



%     imgK_ = ImgTransform(LR{end, 1}, intrMat, k2c_comp{end, 1}(1:3,1:3)');
%     figure,imshowpair(LR{1,1}, imgK_);
rotVec = rodrigues( k2c_comp{end, 1}(1:3,1:3))./2;


Third2SecondRotMat = inv(k2c_comp{2,1})*k2c_comp{2+1,1};
rotVec223 = rodrigues(Third2SecondRotMat(1:3,1:3))./2;

img_2 = ImgTransform(LR{2, 1}, intrMat, rodrigues(rotVec223), intrMatNew);
img_3 = ImgTransform(LR{3, 1}, intrMat, rodrigues(rotVec223)', intrMatNew);


region2 = rgb2gray(img_2) ~=0;
region3 = rgb2gray(img_3) ~=0;
region23 = region2+region3 == 2;
[y23, x23] = ind2sub(size(region23), find(region23(:) == 1));

% figure,imshow(imgK_1); figure,imshow(imgC_1)
% figure,imshow([imgK_1 imgC_1]);
figure,imshowpair(img_2, img_3);

Second2FirstRotMat = inv(k2c_comp{1,1})*k2c_comp{1+1,1};
Second2FirstRotMatNew = Second2FirstRotMat(1:3,1:3)*rodrigues(rotVec223);
img_1 = ImgTransform(LR{1, 1}, intrMat, (Second2FirstRotMatNew), intrMatNew);
figure,imshowpair(img_1, img_2);
region1 = rgb2gray(img_1) ~=0;
region12 = region2+region1 == 2;
[y12, x12] = ind2sub(size(region12), find(region12(:) == 1));


Third2FourthRotMat = inv(k2c_comp{4,1})*k2c_comp{3,1};
Third2FourthRotMatNew = Third2FourthRotMat(1:3,1:3)*rodrigues(-rotVec223);
img_4 = ImgTransform(LR{4, 1}, intrMat, (Third2FourthRotMatNew), intrMatNew);
figure,imshowpair(img_3, img_4);
region4 = rgb2gray(img_4) ~=0;
region34 = region3+region4 == 2;
[y34, x34] = ind2sub(size(region34), find(region34(:) == 1));



index = [max(x12)-1 max(x23)-1 max(x34)-2];

index = [max(x12)-4 max(x23)-4 max(x34)-4];
imgStich = uint8(zeros(size(img_4)));

imgStich(:,1:index(1),:) = img_1(:,1:index(1),:);
imgStich(:,index(1)+1:index(2),:) = img_2(:,index(1)+1:index(2),:);
imgStich(:,index(2)+1:index(3),:) = img_3(:,index(2)+1:index(3),:);
imgStich(:,index(3)+1:end,:) = img_4(:,index(3)+1:end,:);
figure,imshow(imgStich);

try
    cylinderProj(imgStich, 10000);
catch
    sgfhjk = 1;
end



save('dump.mat','LR','Depth','VCam','angList','b2c','-v7.3');

num = 14;
inlier1 = Inlier{1,1}; vcam1 = VCam{1,1};
inlier2 = Inlier{num,1}; vcam2 = VCam{num,1};

delT = b2c * [roty(angList(num+0)) [0 0 0]';0 0 0 1] * inv(b2c);
[inlier12, id1, id2] = intersect(inlier1, inlier2);
err = ((delT)*pextend(vcam1(id1,:)')) -  pextend(vcam2(id2,:)');



inlier1(id1) - inlier2(id2);

SmartRejection = 2;

[R_, t_, ER, maxD] = icp(vcam1(id1,:)',  vcam2(id2,:)', 200, 'Matching','kDtree','SmartRejection',SmartRejection);



if genVid
    close(aviobj);
end
end