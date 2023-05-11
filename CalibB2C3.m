function CalibB2C3(inputDir)
close all



intrMat = [554.256258 0 320.5; 0 554.256258 240.5; 0 0 1];

[xGrid_all, yGrid_all] = meshgrid(1:640, 1:480);

dirInfo = dir(fullfile(inputDir, '*.mat'));
load(fullfile(inputDir, dirInfo(1).name))
[xGrid, yGrid] = meshgrid(0:13, 0:8);
cbSize = 60;
xyz = [cbSize.*[xGrid(:) yGrid(:)] zeros(size(xGrid,1)*size(xGrid,2),1)];
err = [];
imgSize = size(depthGT(:,:,1));
pix = [xMat(:,1), yMat(:,1)];
zMap0 = depthGT(:,:,1);
ind0 = sub2ind(imgSize, round(yMat(:,1)), round(xMat(:,1)));
zList0 = zMap0(ind0);
[XYZ] = GetXYZFromDepth(intrMat, pix,zList0);

ptCloud0 = pointCloud(XYZ);

SmartRejection = 2;

cnt = 1;
thetaList = thetaList - thetaList(1);
for i = 2 : length(thetaList)
    
    [rt, idxOutliers] = posest([xMat(:,i), yMat(:,i)], xyz, 0.95, intrMat, 'repr_err');
    %     [rt, idxOutliers] = posest([xMat(:,i), yMat(:,i)], xyz, 0.99, intrMat);
    
    zMap = depthGT(:,:,i);
    ind = sub2ind(imgSize, round(yMat(:,i)), round(xMat(:,i)));
    zList = zMap(ind);
    
    
    [XYZ_all] = GetXYZFromDepth(intrMat, [xGrid_all(:), yGrid_all(:)],zMap(:));
    [XYZ_cur] = GetXYZFromDepth(intrMat, [xMat(:,i), yMat(:,i)],zList);
    ptCloud = pointCloud(XYZ_cur);
    tform = pcregrigid(ptCloud0,ptCloud,'Extrapolate',true);
     t_ = (tform.T');
     [R, t, ER, maxD] = icp(XYZ_cur', XYZ',20, 'Matching','kDtree','SmartRejection',SmartRejection);
    
     XYZ_cur_2 = [R t; 0 0 0 1]*pextend(XYZ');
     
    [rt, idxOutliers] = posest([xMat(:,i), yMat(:,i)], XYZ, 0.95, intrMat, 'repr_err');
    RT_ = [rodrigues(rt(1:3)) rt(4:6); 0 0 0 1];
    RT = [R rt(4:6);0 0 0 1 ];
    
    agsjhb = 1;
    if 0
        point3D = ones([imgSize 3]);
        point3D(:,:,1) = reshape(XYZ_all(:,1), imgSize);
        point3D(:,:,2) = reshape(XYZ_all(:,2), imgSize);
        point3D(:,:,3) = reshape(XYZ_all(:,3), imgSize);
        img = cat(3, Img(:,:,i), Img(:,:,i), Img(:,:,i));
        figure; hold on;
        pcshow(point3D, img, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
        xlabel('X (mm)');
        ylabel('Y (mm)');
        zlabel('Z (mm)');
        set(gca,  'CameraUpVector',[0 -1 0],'DataAspectRatio',[1 1 1]);
        title('Reconstructed 3-D Scene');
    end
    
    
    ang(cnt,1) = norm(rt(1:3));
    
    ptIcs = TransformAndProject(XYZ, intrMat, RT(1:3,1:3), RT(1:3,4));
    
    
    camPoseC2K(:,:,cnt) = inv(RT);
    bodyPoseK2C(:,:,cnt) = [roty(rad2deg(double(norm(thetaList(i))))) [0;0;0];0 0 0 1];
    err = [err; [ptIcs - [xMat(:,i), yMat(:,i)]]];
    figure,subplot(1,2,1);imshow(Img(:,:,i));hold on;plot(xMat(:,i), yMat(:,i),'-r');plot(xMat(1,i), yMat(1,i),'ob');
    subplot(1,2,2);plot(xMat(:,i) - ptIcs(:,1), yMat(:,i) - ptIcs(:,2), '+r');axis equal;
    cnt = cnt + 1;
end

figure,subplot(1,2,1);imshow(Img(:,:,end));hold on;plot(xMat' , yMat','-g');plot(xMat(:,end), yMat(:,end),'or');
subplot(1,2,2);plot(err(:,1), err(:,2), '+r');axis equal

[b2c_solve, error] = TSAIleastSquareCalibration(camPoseC2K, bodyPoseK2C);
b2c_solve0 = b2c_solve;
rotVec = rodrigues(b2c_solve(1:3,1:3));
b2c_solve(1:3,1:3) = rodrigues([rotVec(1);0;0]);

Rb2c = b2c_solve(1:3, 1:3);
err2 = [];
for i = 2 : length(thetaList)
    
    [rt, idxOutliers] = posest([xMat(:,i), yMat(:,i)], XYZ, 0.95, intrMat, 'repr_err');
    %     [rt, idxOutliers] = posest([xMat(:,i), yMat(:,i)], xyz, 0.99, intrMat);
    RT0 = [rodrigues(rt(1:3)) rt(4:6); 0 0 0 1];
    
    RT = [Rb2c*roty(rad2deg(double(norm(thetaList(i)))))*Rb2c' RT0(1:3,4); 0 0 0 1];
    ptIcs = TransformAndProject(XYZ, intrMat, RT(1:3,1:3), RT(1:3,4));
    
    
    
    err2 = [err2; [ptIcs - [xMat(:,i), yMat(:,i)]]];
    figure,subplot(1,2,1);imshow(Img(:,:,i));hold on;plot(xMat(:,i), yMat(:,i),'-r');plot(xMat(1,i), yMat(1,i),'ob');
    subplot(1,2,2);plot(xMat(:,i) - ptIcs(:,1), yMat(:,i) - ptIcs(:,2), '+r');axis equal;
end

figure,plot(err2(:,1), err2(:,2), '+r');axis equal

ang1 = rad2deg(ang - ang(1));
thetaList1 = rad2deg(thetaList - thetaList(1));
end
function [XYZ] = GetXYZFromDepth(intrMat, Pix,depthList)
metricPrevPtCcsGT = intrMat\HomoCoord(Pix',1);
metricPrevPtCcsGT = normc(metricPrevPtCcsGT);
if 1
    if 0
        if 0
            scaleAllGT = depthListGT(inlierId)./metricPrevPtCcsGT(3,:)';
        else
            scaleAllGT = depthListGT(:)./metricPrevPtCcsGT(3,:)';
        end
    else
        scaleAllGT = depthList./metricPrevPtCcsGT(3,:)';
    end
else
    dispGTComp = dispList(inlierId) - disparityErrorRound;
    depthListGTComp = intrMat(1,1).*norm(obj.camModel.transVec1To2)./(dispGTComp + (princpPtR(1) - princpPtL(1)));
    scaleAllGT = depthListGTComp./metricPrevPtCcsGT(3,:)';
end
XYZ = [repmat(scaleAllGT',3,1).*metricPrevPtCcsGT]';




end