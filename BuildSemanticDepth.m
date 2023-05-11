function [poseOpt] = BuildSemanticDepth(img, K, pt3dline, pt3d2, pt2dline, pt2dz, pose, PoleLane)

reBaseFirst = true;

usePole = PoleLane(1);
useLane = PoleLane(2);

if reBaseFirst
    poseMat = [rodrigues(pose(1:3)) pose(4:6);0 0 0 1];
    poseMatBase = eye(4);
else
    poseMatBase = [rodrigues(pose(1:3)) pose(4:6);0 0 0 1];
    poseMat = eye(4);
end
pt3dline = (poseMat(1:3,1:3)*pt3dline' + repmat(poseMat(1:3,4),1,size(pt3dline,1)))';
pt3d2 = (poseMat(1:3,1:3)*pt3d2' + repmat(poseMat(1:3,4),1,size(pt3d2,1)))';

prvDepth = zeros(size(img(:,:,1)));
prvImg = zeros(size(img(:,:,1)));
curImg = zeros(size(img(:,:,1)));

if usePole
    pole_hi = pt3dline(2:2:end,:);
    pole_lo = pt3dline(1:2:end,:);
    
    for i = 1 :size(pole_hi,1)
        
        line =  [pole_hi(i,:);pole_lo(i,:)];
        lineDir = line(1,:) - line(2,:);
        lineLen = norm(lineDir);
        lineDir = lineDir./norm(lineDir);
        tempPt = [];
        list = 0:0.001:lineLen;
        for j = 1:length(list)
            tempPt(j,:) = line(2,:) + list(j)*lineDir;
        end
        pt2dTemp = pflat(K*tempPt')';
        pt2dTempR = round(pt2dTemp);
        in = find(pt2dTempR(:,1) > 2 & pt2dTempR(:,1) < size(img, 2)-2 & pt2dTempR(:,2) >2 & pt2dTempR(:,2) < size(img, 1)-2);
        ind = sub2ind(size(prvDepth), pt2dTempR(in,2), pt2dTempR(in,1));
        prvDepth(ind) = tempPt(in,3);
        prvImg(ind) = 255;
        if 0
            figure,fig_3D3_pair(tempPt',[]);plot3(line(:,1),line(:,2),line(:,3),'og')
        end
        
    end
end
if useLane
    lane_near = pt3d2(1:2:end,:);
    lane_far = pt3d2(2:2:end,:);
    
    for i = 1 :size(lane_near,1)
        
        line =  [lane_near(i,:);lane_far(i,:)];
        lineDir = line(1,:) - line(2,:);
        lineLen = norm(lineDir);
        lineDir = lineDir./norm(lineDir);
        tempPt = [];
        list = 0:0.001:lineLen;
        for j = 1:length(list)
            tempPt(j,:) = line(2,:) + list(j)*lineDir;
        end
        pt2dTemp = pflat(K*tempPt')';
        pt2dTempR = round(pt2dTemp);
        in = find(pt2dTempR(:,1) > 2 & pt2dTempR(:,1) < size(img, 2)-2 & pt2dTempR(:,2) >2 & pt2dTempR(:,2) < size(img, 1)-2);
        ind = sub2ind(size(prvDepth), pt2dTempR(in,2), pt2dTempR(in,1));
        %    ind = sub2ind(size(prvDepth), pt2dTempR(:,2), pt2dTempR(:,1));
        prvDepth(ind) = tempPt(in,3);
        prvImg(ind) = 255;
        if 0
            figure,fig_3D3_pair(tempPt',[]);plot3(line(:,1),line(:,2),line(:,3),'og')
        end
        
    end
    
end


if usePole
    pole_hi = pt2dline(2:2:end,:);
    pole_lo = pt2dline(1:2:end,:);
    
    for i = 1 :size(pole_hi,1)
        
        line =  [pole_hi(i,:);pole_lo(i,:)];
        lineDir = line(1,:) - line(2,:);
        lineLen = norm(lineDir);
        lineDir = lineDir./norm(lineDir);
        tempPt = [];
        list = 0:0.01:lineLen;
        for j = 1:length(list)
            tempPt(j,:) = line(2,:) + list(j)*lineDir;
        end
        pt2dTemp = tempPt;
        pt2dTempR = round(pt2dTemp);
        in = find(pt2dTempR(:,1) > 2 & pt2dTempR(:,1) < size(img, 2)-2 & pt2dTempR(:,2) >2 & pt2dTempR(:,2) < size(img, 1)-2);
        ind = sub2ind(size(prvDepth), pt2dTempR(in,2), pt2dTempR(in,1));
        %    ind = sub2ind(size(prvDepth), pt2dTempR(:,2), pt2dTempR(:,1));
        curImg(ind) = 255;
        if 0
            figure,imshow(img);hold on;plot(tempPt(:,1), tempPt(:,2),'.');hold on;plot(line(:,1),line(:,2),'og')
        end
        
    end
end
if useLane
    
    lane_near = pt2dz(1:2:end,:);
    lane_far = pt2dz(2:2:end,:);
    
    for i = 1 :size(lane_near,1)
        
        line =  [lane_near(i,:);lane_far(i,:)];
        lineDir = line(1,:) - line(2,:);
        lineLen = norm(lineDir);
        lineDir = lineDir./norm(lineDir);
        tempPt = [];
        list = 0:0.1:lineLen;
        for j = 1:length(list)
            tempPt(j,:) = line(2,:) + list(j)*lineDir;
        end
        pt2dTemp = tempPt;
        pt2dTempR = round(pt2dTemp);
        in = find(pt2dTempR(:,1) > 2 & pt2dTempR(:,1) < size(img, 2)-2 & pt2dTempR(:,2) >2 & pt2dTempR(:,2) < size(img, 1)-2);
        ind = sub2ind(size(prvDepth), pt2dTempR(in,2), pt2dTempR(in,1));
        %    ind = sub2ind(size(prvDepth), pt2dTempR(:,2), pt2dTempR(:,1));
        curImg(ind) = 255;
        if 0
            figure,imshow(img);hold on;plot(tempPt(:,1), tempPt(:,2),'.');hold on;plot(line(:,1),line(:,2),'og')
        end
        
    end
end
se = strel('square',[5]);
se2 = strel('square',[8]);

se = strel('square',[15]);
se2 = strel('square',[15]);
% 
% se = strel('square',[4]);
% se2 = strel('square',[4]);

curImg = imdilate(curImg,se2);
prvImg = imdilate(prvImg,se);
prvDepth = imdilate(prvDepth,se);
% % prvImg(800:end,:) = 0;
% % prvDepth(800:end,:) = 0;
% % figure,imshow(curImg,[]);figure,imshow(prvImg,[]);figure,imshow(prvDepth,[]);




% [pose_rel, score, warped_image, valid_mask, weightMap,  warped_z, warped_z_mask] = estimateVisualOdometry(img_curr, img_prev, depthCur_temp, depthPrv_temp, intrMat, num_levels,initPose, 0);


[pose_rel, score, warped_image, valid_mask, weightMap,  warped_z, warped_z_mask] = estimateVisualOdometry_(curImg, prvImg, prvDepth, prvDepth, K, 3, poseMatBase, 0);
[warped_image1, valid_mask1, warped_z1, warped_z_mask1] = warpImage_(curImg, prvDepth, poseMatBase, K);
score1 = mean((warped_image1(valid_mask1) - prvImg(valid_mask1)).^2);

figure,subplot(2,2,1),imshow(warped_image - prvImg, []);title('opt');subplot(2,2,2),imshow(warped_image1- prvImg, []);title('init'); subplot(2,2,3),imshow(abs(warped_image - prvImg).*valid_mask, []);title(num2str(score));subplot(2,2,4),imshow(abs(warped_image1 - prvImg).*valid_mask1);title(num2str(score1));

figure,imshow(warped_image)
figure,imshow(prvImg)
figure,imshow(warped_image1)

pose_rel1 = pose_rel*poseMat;
poseOpt = [rodrigues(pose_rel1(1:3,1:3));pose_rel1(1:3,4)];
% poseOpt = poseOpt1*poseMat;
end