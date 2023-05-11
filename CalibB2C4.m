function CalibB2C4(inputDir, startt, endd)

close all

% intrMat = [1108.5125168 0 640.5; 0 1108.5125168 360.5; 0 0 1];
intrMat = [554.256258 0 320.5; 0 554.256258 240.5; 0 0 1];
% intrMat = [554.256258/2 0 160.5; 0 554.256258/2 120.5; 0 0 1];
b2c = [eye(3) [10 45 -170.2]';0 0 0 1];



angPnpEqual = false; true;

forceFixesAxis = true;



dirInfo = dir(fullfile(inputDir, '*.mat'));

load(fullfile(inputDir, dirInfo(startt).name));
data1 = data;
load(fullfile(inputDir, dirInfo(endd).name));
data2 = data;
clear 'data';

theta = data2{6} - data1{6};

depthGTPrv = data1{5};
depthGTCur = data2{5};
imgPrvL = data1{1};
imgCurL = data2{1};
cnt = 1;

if 1
    for i = startt : endd
        load(fullfile(inputDir, dirInfo(i).name));
        
        
        traceX(data{4}(:,1),cnt) = data{4}(:,2);
        traceY(data{4}(:,1),cnt) = data{4}(:,3);
        thetaList(cnt,1) = data{6};
        depthGT(:,:,cnt) = data{5};
        imgL = data{1};
        Img(:,:,cnt) = rgb2gray(imgL);
        cnt = cnt + 1;
    end
else
    load('E:\bk_20180627\SLAM\slam_algo\00_01_R_CCW_1280x720_00\pre\data.mat');
    %     load('\\192.168.50.172\nextvpu\3.CTO°ì¹«ÊÒ\Roger\SlamDataset\01_00_R_CCW_640x480_01\data.mat')
end
imgSize = size(depthGT(:,:,1));
[xGrid_all, yGrid_all] = meshgrid(1:imgSize(2), 1:imgSize(1));


minFrameNum = 15; 25; 15; 25;15; 5; 25;
pixThr = 0.5; 0.8;
numThr = 100;
draw = 0;

options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','off','MaxFunEvals',10000,'TolX',1e-8);%,'MaxFunEvals',10000);%, 'MaxIterations',5);

thetaList = thetaList - thetaList(1);
Err = {}; error = []; b2c_vec = []; camPoseC2K = {}; bodyPoseK2C = {};
XErrMat = []; YErrMat = []; cnt = 1; validCnt = [];
for i = 1 : size(traceX,2) - (minFrameNum-1)
    
    thetaListTemp = thetaList(i : i+minFrameNum-1);
    
    thetaListTemp = thetaListTemp - thetaListTemp(1);
    
    nearestKeyX = traceX(:, i: i + minFrameNum-1);
    nearestKeyY = traceY(:, i: i + minFrameNum-1);
    featIdKey = find(min(nearestKeyX') > 0)';
    
    xMat = nearestKeyX(featIdKey,:);
    yMat = nearestKeyY(featIdKey,:);
    
    depthKey = depthGT(:,:,i);
    
    pix = [xMat(:,1) yMat(:,1)];
    indKey = sub2ind(size(depthKey), round(pix(:,2)), round(pix(:,1)));
    depthKList = depthKey(indKey);
    try
        [xyzKey] = GetXYZFromDepth(intrMat, pix,depthKList);
    catch
        continue;
    end
    
    DiffDiffAng = rad2deg(diff(diff(thetaListTemp)));
    if 0 %max(abs(DiffDiffAng)) > 0.015
        featIdKey = [];
    end
    if length(featIdKey) > numThr
        
        if 1 %~forceFixesAxis
            err = {};IdxInliers = []; RtTemp = [];
            for j = 1 : minFrameNum - 1
                pixCur = [xMat(:,j+1), yMat(:,j+1)];
                
                [rtTemp, ~] = posest(double(pixCur), double(xyzKey), 0.9, intrMat, 'repr_err');
                
                RtTemp = [RtTemp rtTemp(4:6)];
                
                RTTemp = [rodrigues(rtTemp(1:3)) rtTemp(4:6); 0 0 0 1];
                ptIcsTemp = TransformAndProject(xyzKey, intrMat, RTTemp(1:3,1:3), RTTemp(1:3,4));
                [~, errTemp] = NormalizeVector(ptIcsTemp - pixCur);
                idxOutliers = find(errTemp > pixThr);
                inliersTemp = find(errTemp <= pixThr);
                [rt, ~] = posest(double(pixCur(inliersTemp,:)), double(xyzKey(inliersTemp,:)), 0.9, intrMat, 'repr_err');
                
                if angPnpEqual
                    angScale = thetaListTemp(j+1)/norm(rt(1:3));
                    rt(1:3) = angScale.*rt(1:3);
                end
                
                
                inliers = setdiff([1:size(pix,1)]', idxOutliers);
                RT = [rodrigues(rt(1:3)) rt(4:6); 0 0 0 1];
                
                RT_body = b2c*  [roty(rad2deg(double(thetaListTemp(j+1)))) [0 0 0]';0 0 0 1] * inv(b2c);
%                                     RT_body = b2c*  [roty(rad2deg(double(norm(rt(1:3))))) [0 0 0]';0 0 0 1] * inv(b2c);
                
                
                ptIcs_body = TransformAndProject(xyzKey, intrMat, RT_body(1:3,1:3), RT_body(1:3,4));
                ptIcs = TransformAndProject(xyzKey, intrMat, RT(1:3,1:3), RT(1:3,4));
                err = [err; [{ptIcs - pixCur}, {inliers} {mean((ptIcs(inliers,:) - pixCur(inliers,:)),1)} {ptIcs(inliers,:) - pixCur(inliers,:)}]];
                
                XErrMat(cnt,j) = mean(ptIcs(inliers,1) - pixCur(inliers,1));
                YErrMat(cnt,j) = mean(ptIcs(inliers,2) - pixCur(inliers,2));
                XErrMat_body(cnt,j) = mean(ptIcs_body(inliers,1) - pixCur(inliers,1));
                YErrMat_body(cnt,j) = mean(ptIcs_body(inliers,2) - pixCur(inliers,2));
                
                
                angErrMat(cnt,j) = rad2deg(norm(rt(1:3)) - thetaListTemp(j+1));
                
                IdxInliers = [IdxInliers; inliers];
                camPoseC2K{cnt,1}(:,:,j) = inv(RT);
                camPoseK2C{cnt,1}(:,:,j) = (RT);
                %                     bodyPoseK2C{i,1}(:,:,j) = [roty(rad2deg(double(thetaListTemp(j+1)))) [0;0;0];0 0 0 1];
                bodyPoseK2C{cnt,1}(:,:,j) = [roty(rad2deg(double(norm(rt(1:3))))) [0;0;0];0 0 0 1];
                
            end
            [b2c_solve, error(:,cnt)] = TSAIleastSquareCalibration(camPoseC2K{cnt,1}, bodyPoseK2C{cnt,1});
            b2c_vec(:,cnt) = [rodrigues(b2c_solve(1:3,1:3)); b2c_solve(1:3,4)];
            Err{cnt,1} = err;
            errCell = cell2mat(err(:,4));
            
            % % %     xErrMat = []; yErrMat = [];
            % % %     for k = 1 : size(err,1)
            % % %         xErrMat = [xErrMat; mean(err{k,4}(:,1))];
            % % %         yErrMat = [yErrMat; mean(err{k,4}(:,2))];
            % % %     end
            % % %     XErrMat = [XErrMat xErrMat];
            % % %     YErrMat = [YErrMat yErrMat];
            if draw
                figure(cnt),subplot(1,2,1);plot(errCell(:,1),errCell(:,2) ,'+r');title(num2str(mean(errCell)));axis equal;
            end
            reprojErr(cnt,1:2) = mean(errCell);
            
            if 0
                inId = [1:size(xMat,1)]';
                axisVec0 = [pi/2;pi/2];
                err0 = FixAxisPnp(intrMat, thetaList, xyzKey, xMat, yMat, minFrameNum, RtTemp,b2c,inId, axisVec0);
                %         options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','off','MaxFunEvals',10000,'TolX',1e-8);%,'MaxFunEvals',10000);%, 'MaxIterations',5);
                lb = [(pi/2 - rad2deg(0.05)); 0];
                ub = [(pi/2 + rad2deg(0.05)); 2*pi];
                
                if 0
                    [vec,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) FixAxisPnp(intrMat, thetaList, xyzKey, xMat, yMat, minFrameNum, RtTemp,b2c,inId, X),[axisVec0],lb,ub,options);%,data_1,obs_1)
                else
                    [vec,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) FixAxisPnp(intrMat, thetaList, xyzKey, xMat, yMat, minFrameNum, RtTemp,b2c, inId,X),[axisVec0],[],[],options);%,data_1,obs_1)
                end
                
                [err1, camPoseC2K_, camPoseK2C_, bodyPoseK2C_] = FixAxisPnp(intrMat, thetaList, xyzKey, xMat, yMat, minFrameNum, RtTemp,b2c,inId, vec);
                
                [b2c_solve_2, error_2(:,cnt)] = TSAIleastSquareCalibration(camPoseC2K_, bodyPoseK2C_);
                
                rotAxisStack(:,cnt) = [sin(vec(1))*cos(vec(2)); sin(vec(1))*sin(vec(2)); cos(vec(1))];
                
                b2c_solve__stack_2(:,cnt) = rodrigues(b2c_solve_2(1:3,1:3));
                
                if 0
                    figure,plot([err0 err1]);title(num2str(mean([err0 err1])));
                    
                    [b2c_solve__stack_norm_2] = NormalizeVector(b2c_solve__stack_2');
                    
                    ang2 = CalcDegree2(repmat(rotAxisStack(:,1)',size(rotAxisStack,2),1),rotAxisStack');
                    ang22 = CalcDegree2(repmat(b2c_solve__stack_norm_2(1,:),size(rotAxisStack,2),1),b2c_solve__stack_norm_2);
                    figure,hist(real(ang2), 100)
                    figure,hist(real(ang22), 100)
                    figure,plot(rotAxisStack')
                    figure,plot(b2c_solve__stack_norm_2)
                    figure,plot(b2c_solve__stack_2')
                end
                
                
                
                
                err00 = reshape(err0, size(pix,1), []);
                meanErr0 = mean(err00);
                diffMeanErr0 = diff(meanErr0);
            end
%             if sum(err0 > 5)/sum(err0 > 0) < 0.9/(minFrameNum - 1)
            if  1 %  max(abs(diffMeanErr0)) < 1
             
                validCnt = [validCnt; i];
                cnt = cnt + 1;
            else
                
                sdfk = 1;
            end
        else
            % % %             axisVec0 = [pi/2;pi/2];
            % % %             err0 = FixAxisPnp(intrMat, thetaList, depthKey, xMat, yMat, minFrameNum, axisVec0);
            % % %             %         options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','off','MaxFunEvals',10000,'TolX',1e-8);%,'MaxFunEvals',10000);%, 'MaxIterations',5);
            % % %             lb = [(pi/2 - rad2deg(0.05)); 0];
            % % %             ub = [(pi/2 + rad2deg(0.05)); 2*pi];
            % % %             [vec,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) FixAxisPnp(intrMat, thetaList, depthKey, xMat, yMat, minFrameNum, X),[axisVec0],lb,ub,options);%,data_1,obs_1)
            % % %             [err1, camPoseC2K_, camPoseK2C_, bodyPoseK2C_] = FixAxisPnp(intrMat, thetaList, depthKey, xMat, yMat, minFrameNum, vec);
            % % %
            % % %             [b2c_solve_2, error_2(:,cnt)] = TSAIleastSquareCalibration(camPoseC2K_, bodyPoseK2C_);
            % % %
            % % %             rotAxisStack(:,cnt) = [sin(vec(1))*cos(vec(2)); sin(vec(1))*sin(vec(2)); cos(vec(1))];
            % % %
            % % %             b2c_solve__stack_2(:,cnt) = rodrigues(b2c_solve_2(1:3,1:3));
            % % %
            % % %             if 0
            % % %                 figure,plot([err0 err1]);
            % % %
            % % %                 [b2c_solve__stack_norm_2] = NormalizeVector(b2c_solve__stack_2');
            % % %
            % % %                 ang2 = CalcDegree2(repmat(rotAxisStack(:,1)',size(rotAxisStack,2),1),rotAxisStack');
            % % %                 ang22 = CalcDegree2(repmat(b2c_solve__stack_norm_2(1,:),size(rotAxisStack,2),1),b2c_solve__stack_norm_2);
            % % %                 figure,hist(real(ang2), 100)
            % % %                 figure,hist(real(ang22), 100)
            % % %             end
            % % %
            
            validCnt = [validCnt; i];
            cnt = cnt + 1;
        end
    end
end

goodId = find(max(abs(angErrMat)') < 110.1);
goodId = find(max(abs(angErrMat)') < 0.1);

imgId = 50;
zMap = depthGT(:,:,imgId);
[XYZ_all] = GetXYZFromDepth(intrMat, [xGrid_all(:), yGrid_all(:)],zMap(:));
point3D = ones([imgSize 3]);
point3D(:,:,1) = reshape(XYZ_all(:,1), imgSize);
point3D(:,:,2) = reshape(XYZ_all(:,2), imgSize);
point3D(:,:,3) = reshape(XYZ_all(:,3), imgSize);
img = cat(3, Img(:,:,imgId), Img(:,:,imgId), Img(:,:,imgId));
figure;subplot(2,2,1); hold on;
pcshow(point3D, img, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
set(gca,  'CameraUpVector',[0 -1 0],'DataAspectRatio',[1 1 1]);
title('Reconstructed 3-D Scene');
subplot(2,2,2);imshow(point3D(:,:,1), []);title('x coord');
subplot(2,2,3);imshow(point3D(:,:,2), []);title('y coord');
subplot(2,2,4);imshow(point3D(:,:,3), []);title('z coord');




error0 = error;
b2c_vec0 = b2c_vec;
b2c_vec = b2c_vec(:,goodId);

[b2c_vec_norm, vAng] = NormalizeVector(b2c_vec(1:3,:)');
figure,subplot(1,2,1); plot(b2c_vec([1 2 3],:)'); subplot(1,2,2),plotQuiver(b2c_vec_norm, [1 0 0], 2, 1);

ang2 = CalcDegree2(repmat(b2c_vec_norm(1,:),size(b2c_vec_norm,1),1),b2c_vec_norm);
subplot(1,2,2),hist(ang2, 100);

Err0 = Err;

if 0
    aa = norm(b2c_vec(1:3,156));
    bb = b2c_vec_norm(156,:);
    bb(2) = 0;
    bb  = bb./norm(bb);
    b2cVec0 = [aa acos(bb(1))]';
    errVar0 = B2CErr(camPoseK2C, bodyPoseK2C,b2cVec0);
    options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter','MaxFunEvals',10000,'TolX',1e-8);%,'MaxFunEvals',10000);%, 'MaxIterations',5);
    [vec,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) B2CErr(camPoseK2C, bodyPoseK2C,X),[b2cVec0],[],[],options);%,data_1,obs_1)
    errVar1 = B2CErr(camPoseK2C, bodyPoseK2C,vec);
    figure,plot([errVar0 errVar1]);
end

if 0
    b2cAng = vec(1);
    b2cAxis = [cos(vec(2));0;sin(vec(2))];
    b2c_vec(1:3,1)= b2cAng.*b2cAxis;
end

MEANX = [];
MEANY = [];
for kkk = 1 : length(goodId)
    
    Err = {}; error = [];  camPoseC2K = {}; bodyPoseK2C = {};
    XErrMat0 = XErrMat; YErrMat0 = YErrMat;
    XErrMat = []; YErrMat = [];cnt = 1;
    
    
    b2cId = kkk;
    
    
    
    
    
    for i = 1 : size(traceX,2) - (minFrameNum-1)
        if ismember(i, validCnt(goodId))
            thetaListTemp = thetaList(i : i+minFrameNum-1);
            
            
            thetaListTemp = thetaListTemp - thetaListTemp(1);
            
            nearestKeyX = traceX(:, i: i + minFrameNum-1);
            nearestKeyY = traceY(:, i: i + minFrameNum-1);
            featIdKey = find(min(nearestKeyX') > 0)';
            
            xMat = nearestKeyX(featIdKey,:);
            yMat = nearestKeyY(featIdKey,:);
            
            depthKey = depthGT(:,:,i);
            
            pix = [xMat(:,1) yMat(:,1)];
            indKey = sub2ind(size(depthKey), round(pix(:,2)), round(pix(:,1)));
            depthKList = depthKey(indKey);
            try
                [xyzKey] = GetXYZFromDepth(intrMat, pix,depthKList);
            catch
                continue;
            end
            if length(featIdKey) > numThr
                if 1 %~exist('Rb2c','var')
                    
                    %                 Rb2c = rodrigues(b2c_vec(1:3,cnt));
                    Rb2c = rodrigues(b2c_vec(1:3,b2cId));
                    
                    %         Rb2c = eye(3);
                end
                err = {};IdxInliers = [];
                for j = 1 : minFrameNum - 1
                    pixCur = [xMat(:,j+1), yMat(:,j+1)];
                    %             [rt, idxOutliers] = posest(double(pixCur), double(xyzKey), 0.95, intrMat, 'repr_err');
                    [rtTemp, ~] = posest(double(pixCur), double(xyzKey), 0.9, intrMat, 'repr_err');
                    RTTemp = [rodrigues(rtTemp(1:3)) rtTemp(4:6); 0 0 0 1];
                    ptIcsTemp = TransformAndProject(xyzKey, intrMat, RTTemp(1:3,1:3), RTTemp(1:3,4));
                    [~, errTemp] = NormalizeVector(ptIcsTemp - pixCur);
                    idxOutliers = find(errTemp > pixThr);
                    inliersTemp = find(errTemp <= pixThr);
                    [rt, ~] = posest(double(pixCur(inliersTemp,:)), double(xyzKey(inliersTemp,:)), 0.9, intrMat, 'repr_err');
                    
                    if angPnpEqual
                        angScale = thetaListTemp(j+1)/norm(rt(1:3));
                        rt(1:3) = angScale.*rt(1:3);
                    end
                    
                    
                    
                    inliers = setdiff([1:size(pix,1)]', idxOutliers);
                    RT_ = [rodrigues(rt(1:3)) rt(4:6); 0 0 0 1];
                    RT =RT_;
                     
%                                                 Rbody = roty(rad2deg(double(thetaListTemp(j+1))));
                    Rbody = roty(rad2deg(double(norm(rt(1:3)))));
                    
                    RT(1:3,1:3) = Rb2c*Rbody*Rb2c';
                    ptIcs = TransformAndProject(xyzKey, intrMat, RT(1:3,1:3), RT(1:3,4));
                    err = [err; [{ptIcs - pixCur}, {inliers} {mean((ptIcs(inliers,:) - pixCur(inliers,:)),1)} {ptIcs(inliers,:) - pixCur(inliers,:)}]];
                    
                    XErrMat(cnt,j) = mean(ptIcs(inliers,1) - pixCur(inliers,1));
                    YErrMat(cnt,j) = mean(ptIcs(inliers,2) - pixCur(inliers,2));
                    
                    IdxInliers = [IdxInliers; inliers];
                    %         camPoseC2K{i,1}(:,:,j) = inv(RT);
                    %         bodyPoseK2C{i,1}(:,:,j) = [roty(rad2deg(double(thetaListTemp(j+1)))) [0;0;0];0 0 0 1];
                end
                %     [b2c_solve_, error_(:,i)] = TSAIleastSquareCalibration(camPoseC2K{i,1}, bodyPoseK2C{i,1});
                %     b2c_vec_(:,i) = [rodrigues(b2c_solve_(1:3,1:3)); b2c_solve_(1:3,4)];
                
                errCell = cell2mat(err(:,4));
                
                
                
                % % %     xErrMat = []; yErrMat = [];
                % % %     for k = 1 : size(err,1)
                % % %         xErrMat = [xErrMat; mean(err{k,4}(:,1))];
                % % %         yErrMat = [yErrMat; mean(err{k,4}(:,2))];
                % % %     end
                % % %     XErrMat = [XErrMat xErrMat];
                % % %     YErrMat = [YErrMat yErrMat];
                
                if draw
                    figure(cnt),subplot(1,2,2);plot(errCell(:,1),errCell(:,2) ,'+r');title(num2str(mean(errCell)));axis equal;
                end
                reprojErr(cnt,3:4) = mean(errCell);
                cnt = cnt + 1;
            end
        end
    end
    
    
    
    id = find(abs(XErrMat(:,end)) < 10.1);
    
    if draw
        figure,subplot(2,4,1);plot(-(XErrMat(id,:))');title('x');
        subplot(2,4,2),plot(-(YErrMat(id,:))');title('y');
        subplot(2,4,5);plot(-median(XErrMat(id,:)));title('x');
        subplot(2,4,6),plot(-median(YErrMat(id,:)));title('y');
        
        
        subplot(2,4,3);plot(-(XErrMat_body(goodId,:))');title('x');
        subplot(2,4,4),plot(-(YErrMat_body(goodId,:))');title('y');
        subplot(2,4,7);plot(-median(XErrMat_body(goodId,:)));title('x');
        subplot(2,4,8),plot(-median(YErrMat_body(goodId,:)));title('y');
        
        drawnow;
    end
    
    MEANX = [MEANX; -median(XErrMat(id,:))];
    MEANY = [MEANY; -median(YErrMat(id,:))];
    % figure,plot(b2c_vec([1 3],id)');
end
for ij = 1 :size(MEANX,1)
    in = ij; figure,plot([MEANX(in,:);MEANY(in,:)]');title(num2str(in))
end

return;

imgSize = size(depthGTPrv);
margin = 5;

if theta < 0
    [xGrid, yGrid] = meshgrid([imgSize(2)/2: imgSize(2) - margin], [margin : imgSize(1)/2]);
    [xGrid, yGrid] = meshgrid([600: imgSize(2) - margin], [margin : imgSize(1)/2]);
    
else
    [xGrid, yGrid] = meshgrid([margin: 10], [margin : imgSize(1)/2]);
end

pix = [xGrid(:) yGrid(:)];

indPrv = sub2ind(size(depthGTPrv), pix(:,2), pix(:,1));
depthListGT = depthGTPrv(indPrv);
[pixGT, transXYZ] = GetGtTrace2(b2c, rad2deg(theta), pix,depthListGT,intrMat);



% figure,subplot(1,2,1);imshow(imgPrvL);hold on;plot(pix(:,1), pix(:,2),'.r');title('prv');subplot(1,2,2);imshow(imgCurL);hold on;plot(pixGT(:,1), pixGT(:,2),'.r');title('cur');

valid = find(pixGT(:,1) > 1 & pixGT(:,2) > 1 & pixGT(:,1) < imgSize(2)-1 & pixGT(:,2) < imgSize(1)-1);


figure,subplot(1,2,1);imshow(imgPrvL);hold on;plot(pix(valid,1), pix(valid,2),'.r');title('prv');subplot(1,2,2);imshow(imgCurL);hold on;plot(pixGT(valid,1), pixGT(valid,2),'.r');title('cur');

indCur = sub2ind(size(depthGTCur), round(pixGT(valid,2)), round(pixGT(valid,1)));

depthListCur = depthGTCur(indCur);

err = transXYZ(valid,3) - depthListCur;


b2cVec0 = [rodrigues(b2c(1:3,1:3)); b2c(1:3,4)];
b2cVec0 = b2cVec0(1:3);

errVar0 = CostFunc(b2cVec0, theta, pix, depthListGT, intrMat, depthGTCur,imgSize);
options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','off','MaxFunEvals',10000,'TolX',1e-8);%,'MaxFunEvals',10000);%, 'MaxIterations',5);
[vec,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) CostFunc(X, theta, pix, depthListGT, intrMat, depthGTCur,imgSize),[b2cVec0],[],[],options);%,data_1,obs_1)
errVar1 = CostFunc(vec, theta, pix, depthListGT, intrMat, depthGTCur,imgSize);
end
function [pixGT, transXYZ] = GetGtTrace2(b2cPmat, k2cRef0, Pix,depthListGT,intrMat)

% if 0
%     k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(thetaRng(idM)),zeros(3,1));
% else
%     k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cRef0),zeros(3,1));
% end
k2cBodyPmat = [roty(double(k2cRef0)) [0;0;0];0 0 0 1];
k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;



metricPrevPtCcsGT = intrMat\HomoCoord(Pix',1);
metricPrevPtCcsGT = normc(metricPrevPtCcsGT);
if 1
    scaleAllGT = depthListGT./metricPrevPtCcsGT(3,:)';
else
    dispGTComp = dispList(inlierId) - disparityErrorRound;
    depthListGTComp = intrMat(1,1).*norm(obj.camModel.transVec1To2)./(dispGTComp + (princpPtR(1) - princpPtL(1)));
    scaleAllGT = depthListGTComp./metricPrevPtCcsGT(3,:)';
end
XYZ = [repmat(scaleAllGT',3,1).*metricPrevPtCcsGT];


k2cT = -b2cPmat(1:3,1:3)*k2cBodyPmat(1:3,1:3)*b2cPmat(1:3,1:3)'*b2cPmat(1:3,4) + b2cPmat(1:3,4);
k2cCam = [k2cCamPmat(1:3,1:3) k2cT;0 0 0 1];
homocurrCcsXYZ = k2cCam*HomoCoord(XYZ,1); %pix0_GT_ = pflat(intrMat*homocurrCcsXYZALL_(1:3,:))';
pixGT = pflat(intrMat*homocurrCcsXYZ(1:3,:))';
pixGT = pixGT(:,1:2);

transXYZ = homocurrCcsXYZ(1:3,:)';

end

function errVar = CostFunc(b2cVec, theta, pix, depthListGT, intrMat, depthGTCur,imgSize)
b2cVec(1:3);
if length(b2cVec) > 3
    b2c = [rodrigues(b2cVec(1:3)) b2cVec(4:6);0 0 0 1];
else
    b2c = [rodrigues(b2cVec(1:3)) [10;45;-170.2];0 0 0 1];
end
[pixGT, transXYZ] = GetGtTrace2(b2c, rad2deg(theta), pix,depthListGT,intrMat);


valid = find(pixGT(:,1) > 1 & pixGT(:,2) > 1 & pixGT(:,1) < imgSize(2)-1 & pixGT(:,2) < imgSize(1)-1);

indCur = sub2ind(size(depthGTCur), round(pixGT(valid,2)), round(pixGT(valid,1)));

depthListCur = depthGTCur(indCur);

err = transXYZ(valid,3) - depthListCur;
errVar = mean(abs(err).^2);
end
function cbcL = detectCnr(imgL1)
[cbcX1, cbcY1, corner1] = DetectCbCorner(rgb2gray(imgL1));
[initCbcX1, initCbcY1] = CbCoordSys(cbcX1, cbcY1);
cbcL = cornerfinder([initCbcX1(:),initCbcY1(:)]',double(rgb2gray(imgL1)),5,5);
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


function err = B2CErr(camPoseK2C, bodyPoseK2C, rotB2C)


b2cAng = rotB2C(1);
b2cAxis = [cos(rotB2C(2));0;sin(rotB2C(2))];

Rb2c = rodrigues(b2cAng.*b2cAxis);
tb2c = [10; 45;-170.2];

Tb2c = [Rb2c tb2c; 0 0 0 1];
Tb2c2 = repmat(Tb2c,1,1,size((bodyPoseK2C{1,1}),3));
Tb2c2_inv = repmat(inv(Tb2c),1,1,size((bodyPoseK2C{1,1}),3));

for i = 1:size(bodyPoseK2C,1)
    body = bodyPoseK2C{i,1}(1:3,1:3,:);
    for j = 1 : size(body,3)
        body_ = Tb2c2(1:3,1:3,1 )*body(:,:,j)*Tb2c2_inv(1:3,1:3,1);
        cam = camPoseK2C{i,1}(1:3,1:3,j);
        dlt(i,j) = norm(rodrigues(body_'*cam));
    end
end

err = 1000.*dlt(:);
end
