function [XErrMat_body, YErrMat_body, thetaList, traceInfoMat,XErrMat_body_stack, YErrMat_body_stack, Img, validCnt,xErrMat,yErrMat,xErrMatFull,yErrMatFull] = CalibB2C6(inputDir, startt, endd)
% function [XErrMat_body2, YErrMat_body2, thetaList, traceInfoMat,XErrMat_body_stack2, YErrMat_body_stack2, Img, validCnt] = CalibB2C6(inputDir, startt, endd)
global PrvTrackOnGT
% % % close all

if 0
    titleId = find(inputDir == '\' | inputDir == '/');
    for o = 1 : length(titleId)
        if strcmp(inputDir(titleId(o)+1:titleId(o)+4),'prob')
            titleStr = inputDir(titleId(o)+23:titleId(o)+85);
            break;
        end
    end
else
    
    titleId = find(inputDir == '=');
    titleIdDiff = diff(titleId);
    idid = find(titleIdDiff > 1);
    titleStr = inputDir(titleId(idid)-18:titleId(idid+1));
end


ShiftInterval = ReadConfig(fullfile(inputDir,'lkConfig.txt'));
% intrMat = [1108.5125168 0 640.5; 0 1108.5125168 360.5; 0 0 1];
intrMat = [554.256258 0 320.5; 0 554.256258 240.5; 0 0 1];
% intrMat = [550+0 0 320.5+0; 0 550+0 240.5+0; 0 0 1];
% intrMat = [554.256258/2 0 160.5; 0 554.256258/2 120.5; 0 0 1];
b2c = [eye(3) [10 45 -170.2]';0 0 0 1];
global angPnpEqual

% PrvTrackOnGT = true;

angPnpEqual = false; true;
forceFixesAxis = true;

iterNum = 100;
featBatchNum = 30;


dirInfo = dir(fullfile(inputDir, 'Replay*.mat'));
load(fullfile(inputDir, 'judgement.mat'));

endd = min(endd, (size(dirInfo,1) - 1));

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
        if 1
            depthGT(:,:,cnt) = data{5};
        else
            depthGT(:,:,cnt) = data{3};
        end
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


minFrameNum = 15; 14; 25; 14; 15; 25; 15; 25; 15; 25; 15; 25;15; 5; 25;
pixThr = 100; 1; 0.5;  1; 0.5; 0.8;
numThr = 10; 50; 100; 50; 100;
draw = 1;

options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','off','MaxFunEvals',10000,'TolX',1e-8);%,'MaxFunEvals',10000);%, 'MaxIterations',5);

%  thetaList = traceInfoMat{1,6}(1:end-1);
thetaList = thetaList - thetaList(1);
Err = {}; error = []; b2c_vec = []; camPoseC2K = {}; bodyPoseK2C = {};
XErrMat = []; YErrMat = []; cnt = 1; validCnt = []; MetricErr = {};
for i = 1 : endd - 20 % 45 % size(traceInfoMat,1) - 2
    
    if ~isempty(traceInfoMat{i,1})
        frameRng = traceInfoMat{i,3}(1)-1:traceInfoMat{i,3}(end);
        minFrameNum = length(frameRng);
        try
            thetaListTemp = thetaList(frameRng);
        catch
            fjh = 1;
        end
        thetaListTemp = thetaListTemp - thetaListTemp(1);
        if 0
            nearestKeyX = traceX(:, i: i + minFrameNum-1);
            nearestKeyY = traceY(:, i: i + minFrameNum-1);
            featIdKey = find(min(nearestKeyX') > 0)';
        end
        
        featIdKey = traceInfoMat{i,2};
        try
            xMat = traceX(featIdKey, frameRng);
        catch
            break;
            asgdhl = 1;
        end
        try
            yMat = traceY(featIdKey, frameRng);
        catch
            dfsghkj = 1;
        end
        depthKey = depthGT(:,:,i);
        
        pix = [xMat(:,1) yMat(:,1)];
        indKey = sub2ind(size(depthKey), round(pix(:,2)), round(pix(:,1)));
        depthKList = depthKey(indKey);
        try
            [xyzKey] = GetXYZFromDepth(intrMat, pix,depthKList);
        catch
            continue;
        end
        XYZKey{cnt,1} = xyzKey;
        DiffDiffAng = rad2deg(diff(diff(thetaListTemp)));
        if max(abs(DiffDiffAng)) > 10000   %0.2 % 0.015
            featIdKey = [];
        end
        if length(featIdKey) > numThr
            
            if 1 %~forceFixesAxis
                err = {};IdxInliers = [];
                %             RtTemp = [];
                for j = 1 : minFrameNum - 1
                    pixCur = [xMat(:,j+1), yMat(:,j+1)];
                    try
                        indCur = sub2ind(size(depthKey), round(pixCur(:,2)), round(pixCur(:,1)));
                    catch
                        hgjgs = 1;
                    end
                    depthCur = depthGT(:,:,i+j);
                    [xyzCur] = GetXYZFromDepth(intrMat, pixCur, depthCur(indCur));
                    if 0
                        try
                            [rtTemp, ~] = posest(double(pixCur), double(xyzKey), 0.8, intrMat, 'repr_err');
                        catch
                            skh = 1;
                        end
                        RTTemp = [rodrigues(rtTemp(1:3)) rtTemp(4:6); 0 0 0 1];
                    end
                    RTTemp_comp = b2c * [roty(rad2deg(double(thetaListTemp(j+1)))) [0 0 0]';0 0 0 1] * inv(b2c);
                    xyzCur2 = RTTemp_comp*pextend(xyzKey');
                    xyzCur2 = xyzCur2(1:3,:)';
                    ptIcsTemp_gt = TransformAndProject(xyzKey, intrMat, RTTemp_comp(1:3,1:3), RTTemp_comp(1:3,4));
                    try
                        indCur_gt = sub2ind(size(depthKey), round(ptIcsTemp_gt(:,2)), round(ptIcsTemp_gt(:,1)));
                    catch
                        asfkhj = 1;
                    end
                    try
                        [xyzCur_gt] = GetXYZFromDepth(intrMat, ptIcsTemp_gt, depthCur(indCur_gt));
                        [~, xyzErrGT] = NormalizeVector(xyzCur2 - xyzCur_gt);
                        [~, xyzErrLK] = NormalizeVector(xyzCur2 - xyzCur);
                    catch
                        sfkja = 1;
                    end
                    
%                     [~, xyzErrGT] = NormalizeVector(xyzCur2 - xyzCur_gt);
%                     [~, xyzErrLK] = NormalizeVector(xyzCur2 - xyzCur);
                    if 0
                        MetricErr{cnt,1}(:,j) = xyzErrLK - xyzErrGT;
                    end
                    if 0
                        RTTemp = [rodrigues(rtTemp(1:3)) rtTemp(4:6); 0 0 0 1];
                        ptIcsTemp = TransformAndProject(xyzKey, intrMat, RTTemp(1:3,1:3), RTTemp(1:3,4));
                        [~, errTemp] = NormalizeVector(ptIcsTemp - pixCur);
                        idxOutliers = find(errTemp > pixThr);
                    end
                    if 1
                        idxOutliers = [];
                    end
                    if 0
                        inliersTemp = find(errTemp <= pixThr);
                    end
                    if 0
                        try
                            [rt, ~] = posest(double(pixCur(inliersTemp,:)), double(xyzKey(inliersTemp,:)), 0.5, intrMat, 'repr_err');
                        catch
                            if 0
                                continue;
                            end
                            asgkj = 1;
                        end
                    end
                    
                    if 0
                        RtTemp{cnt,1}(:,j) = [ rt(4:6)];
                        RtAng{cnt,1}(j,1) = norm(rt(1:3));
                    end
                    
                    inliers = setdiff([1:size(pix,1)]', idxOutliers);
                    if j == 1
                        inlierIdComm = inliers;
                    else
                        inlierIdComm = intersect(inlierIdComm, inliers);
                    end
                    
                    Inliers_{cnt,j} = inliers;
                    if 0
                        RT = [rodrigues(rt(1:3)) rt(4:6); 0 0 0 1];
                    end
                    RT_body = b2c*  [roty(rad2deg(double(thetaListTemp(j+1)))) [0 0 0]';0 0 0 1] * inv(b2c);
                    %                     RT_body = b2c*  [roty(rad2deg(double(norm(rt(1:3))))) [0 0 0]';0 0 0 1] * inv(b2c);
                    
                    
                    ptIcs_body = TransformAndProject(xyzKey, intrMat, RT_body(1:3,1:3), RT_body(1:3,4));
                    
                    if 0
                        ptIcs = TransformAndProject(xyzKey, intrMat, RT(1:3,1:3), RT(1:3,4));
                        
                        err = [err; [{ptIcs - pixCur}, {inliers} {mean((ptIcs(inliers,:) - pixCur(inliers,:)),1)} {ptIcs(inliers,:) - pixCur(inliers,:)}]];
                        
                        XErrMat(cnt,j) = mean(ptIcs(inliers,1) - pixCur(inliers,1));
                        YErrMat(cnt,j) = mean(ptIcs(inliers,2) - pixCur(inliers,2));
                    end
                    XErrMat_body(cnt,j+1) = -mean(ptIcs_body(inliers,1) - pixCur(inliers,1));
                    YErrMat_body(cnt,j+1) = -mean(ptIcs_body(inliers,2) - pixCur(inliers,2));
                    
                    XErrMat_body2(i,j+1) = -mean(ptIcs_body(inliers,1) - pixCur(inliers,1));
                    YErrMat_body2(i,j+1) = -mean(ptIcs_body(inliers,2) - pixCur(inliers,2));
                    if j == 1
                        XErrMat_body_stack{cnt, j} = pix(:,1:2);
                        YErrMat_body_stack{cnt, j} = pix(:,1:2);
                        XErrMat_body_stack2{i, j} = pix(:,1:2);
                        YErrMat_body_stack1{i, j} = pix(:,1:2);
                    end
                    XErrMat_body_stack{cnt, j+1} = -(ptIcs_body(inliers,1) - pixCur(inliers,1));
                    YErrMat_body_stack{cnt, j+1} = -(ptIcs_body(inliers,2) - pixCur(inliers,2));
                    
                    XErrMat_body_stack2{i, j+1} = -(ptIcs_body(inliers,1) - pixCur(inliers,1));
                    YErrMat_body_stack2{i, j+1} = -(ptIcs_body(inliers,2) - pixCur(inliers,2));
                    
                    djdrst = 1;
                    
                    
                    if 0
                        angErrMat(cnt,j) = rad2deg(norm(rt(1:3)) - thetaListTemp(j+1));
                        
                        IdxInliers = [IdxInliers; inliers];
                        camPoseC2K{cnt,1}(:,:,j) = inv(RT);
                        camPoseK2C{cnt,1}(:,:,j) = (RT);
                        %                     bodyPoseK2C{i,1}(:,:,j) = [roty(rad2deg(double(thetaListTemp(j+1)))) [0;0;0];0 0 0 1];
                        bodyPoseK2C{cnt,1}(:,:,j) = [roty(rad2deg(double(norm(rt(1:3))))) [0;0;0];0 0 0 1];
                    end
                end
                
                ckgkhsg = 1;
                if 0
                    %             inlierIdComm = unique(cell2mat(Inliers_(cnt,:)'));
                    Inliers{cnt,1} = inlierIdComm;
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
                    if 0 % draw
                        figure(cnt),subplot(1,2,1);plot(errCell(:,1),errCell(:,2) ,'+r');title(num2str(mean(errCell)));axis equal;
                    end
                    reprojErr(cnt,1:2) = mean(errCell);
                    
                    
                    
                    axisVec0 = [pi/2;pi/2];
                    err0 = FixAxisPnp(intrMat, thetaList, xyzKey, xMat, yMat, minFrameNum, RtTemp{cnt,1}, b2c, inlierIdComm, RtAng{cnt,1}, axisVec0);
                    %         options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','off','MaxFunEvals',10000,'TolX',1e-8);%,'MaxFunEvals',10000);%, 'MaxIterations',5);
                    lb = [(pi/2 - rad2deg(0.05)); 0];
                    ub = [(pi/2 + rad2deg(0.05)); 2*pi];
                    
                    if 0
                        [vec,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) FixAxisPnp(intrMat, thetaList, xyzKey, xMat, yMat, minFrameNum, RtTemp{cnt,1}, b2c,inlierIdComm,RtAng{cnt,1}, X),[axisVec0],lb,ub,options);%,data_1,obs_1)
                    else
                        [vec,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) FixAxisPnp(intrMat, thetaList, xyzKey, xMat, yMat, minFrameNum, RtTemp{cnt,1}, b2c,inlierIdComm,RtAng{cnt,1}, X),[axisVec0],[],[],options);%,data_1,obs_1)
                    end
                    
                    [err1, camPoseC2K_, camPoseK2C_, bodyPoseK2C_] = FixAxisPnp(intrMat, thetaList, xyzKey, xMat, yMat, minFrameNum, RtTemp{cnt,1}, b2c, inlierIdComm,RtAng{cnt,1},vec);
                    
                    
                    camPoseC2K_2{cnt,1} = camPoseC2K_;
                    camPoseK2C_2{cnt,1} = camPoseK2C_;
                    %                     bodyPoseK2C{i,1}(:,:,j) = [roty(rad2deg(double(thetaListTemp(j+1)))) [0;0;0];0 0 0 1];
                    bodyPoseK2C_2{cnt,1} = bodyPoseK2C_;
                    
                    
                    [b2c_solve_2, error_2(:,cnt)] = TSAIleastSquareCalibration(camPoseC2K_, bodyPoseK2C_);
                    Vec(:,cnt) = vec;
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
                    
                    
                    
                    
                    err00 = reshape(err0, length(inlierIdComm), []);
                    meanErr0 = mean(err00);
                    diffMeanErr0 = diff(meanErr0);
                end
                %             if sum(err0 > 5)/sum(err0 > 0) < 0.9/(minFrameNum - 1)
                if   1% max(abs(diffMeanErr0)) < 1
                    try
                        asd = ~isempty(XErrMat_body_stack{cnt, end});
                    catch
                        sdalkh = 1;
                    end
                    if ~isempty(XErrMat_body_stack{cnt, end})
                        validCnt = [validCnt; i];
                        FeatIdKey{cnt,1} = featIdKey;
                        cnt = cnt + 1;
                    else
                        XErrMat_body = XErrMat_body(1:cnt-1,:);  %-mean(ptIcs_body(inliers,1) - pixCur(inliers,1));
                        YErrMat_body = YErrMat_body(1:cnt-1,:);
                        
                        XErrMat_body_stack = XErrMat_body_stack(1:cnt-1,:);
                        YErrMat_body_stack = YErrMat_body_stack(1:cnt-1,:);
% %                         XErrMat_body_stack(cnt,:) = XErrMat_body_stack(1:cnt-1,:);
% %                         YErrMat_body_stack(cnt,:) = YErrMat_body_stack(1:cnt-1,:);
                        asgk = 1;
                    end
                    skjzs = 1;
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
end


x_p2c = diff(XErrMat_body')'; x_p2c(XErrMat_body(:,2:end) == 0) =  0;
y_p2c = diff(YErrMat_body')';

if 0 % PrvTrackOnGT
    x_p2c = XErrMat_body(:,2:end);
    y_p2c = YErrMat_body(:,2:end);
end

for i = 1 : size(x_p2c, 1) % size(traceInfoMat,1) - 2
    
    
%     frameRng1 = traceInfoMat{i,3} - 1;
%     xErrMat(i,frameRng1) = x_p2c(i,1:length(frameRng1));
%     yErrMat(i,frameRng1) = y_p2c(i,1:length(frameRng1));
    
    frameRng1 = traceInfoMat{validCnt(i),3} - 1;    
    xErrMat(i,frameRng1) = x_p2c(i,1:length(frameRng1));
    yErrMat(i,frameRng1) = y_p2c(i,1:length(frameRng1));
end


validAngOpt2 = abs(xErrMat) > 0;
validAngOptSum2 = sum(validAngOpt2);
xErrMatAccum = (sum(xErrMat)./validAngOptSum2)';




XErrMat_body_stack;
for i = 1 : size(x_p2c, 1) % size(traceInfoMat,1) - 2
    
    
%     frameRng1 = traceInfoMat{i,3} - 1;
%     xErrMat(i,frameRng1) = x_p2c(i,1:length(frameRng1));
%     yErrMat(i,frameRng1) = y_p2c(i,1:length(frameRng1));
    tempFeatId = FeatIdKey{i,1};
    frameRng1 = traceInfoMat{validCnt(i),3} - 1;
    try
        tempXErr = cell2mat(XErrMat_body_stack(i,2:end));
    catch
        erhlk = 1;
    end
    tempYErr = cell2mat(YErrMat_body_stack(i,2:end));
    xErrMatFull(tempFeatId,frameRng1) = tempXErr; % x_p2c(i,1:length(frameRng1));
    yErrMatFull(tempFeatId,frameRng1) = tempYErr; % y_p2c(i,1:length(frameRng1));
end

validAngOptFull2 = abs(xErrMatFull) > 0;
validAngOptSumFull2 = sum(validAngOptFull2);
xErrMatAccumFull = (sum(xErrMatFull)./validAngOptSumFull2)';

figure,plot(cumsum([xErrMatAccum xErrMatAccumFull]));
goodId = find(max(abs(x_p2c)') < 0.1);


weightMatNorm = CalcWeight(abs(xErrMat), 1, 1, 1);
weightMatNormTmp = CalcWeight(abs(xErrMat), 0, 1, 1);


x_head = cumsum(sum(weightMatNorm.*xErrMat)');
x_tail = cumsum(sum(weightMatNormTmp.*xErrMat)');

figure,subplot(2,3,1);plot([x_head x_tail]);legend('head','tail');grid on;
subplot(2,3,2);plot((x_p2c(goodId,:)'));title('p2c x');grid on;
subplot(2,3,3);plot((y_p2c(goodId,:)'));title('p2c y');grid on;
subplot(2,3,5);plot(mean(x_p2c(goodId,:)));title('p2c x');grid on;
subplot(2,3,6);plot(mean(y_p2c(goodId,:)));title('p2c y');grid on;


x_p2c1 = x_p2c(goodId,:);
y_p2c1 = y_p2c(goodId,:);

try
    for i = 1 : iterNum
        
        id_perm = randperm(size(x_p2c1,1)) ;
        %    id_perm = randperm(length(goodId)) ;
        temp_x_p2c = x_p2c1(id_perm(1:featBatchNum),:);
        temp_y_p2c = y_p2c1(id_perm(1:featBatchNum),:);
        mean_x(i,:) = mean(temp_x_p2c, 1);
        mean_y(i,:) = mean(temp_y_p2c, 1);
        
    end
    
    figure,subplot(1,2,1);plot(mean_x');title('x');subplot(1,2,2);plot(mean_y');title('y');
catch
    fgkjaf = 1;
end

evenErr1 = EvenWeightErr(xErrMat);


traceNum1 = size(XErrMat_body,2);
[x_err1, x_err1_upper, x_err1_lower] = CalcEnvelope(xErrMat, traceNum1);
if 0
    figure,subplot(2,1,1);plot(rad2deg(thetaList(1:size(x_err1,1))), x_err1);hold on;plot(rad2deg(thetaList(1:size(x_err1,1))),[x_err1_upper x_err1_lower],'-xb');plot(rad2deg(thetaList(1:size(x_err1,1))),[evenErr1],'-xr');title(sprintf('Shift Interval: %d',ShiftInterval));
    subplot(2,1,2);plot(rad2deg(thetaList(1:size(x_err1,1))), [evenErr1],'-xr'); % plot(rad2deg(thetaList2(1:size(x_err2,1))),x_err2);hold on;plot(rad2deg(thetaList2(1:size(x_err2,1))),[x_err2_upper x_err2_lower],'-xb');plot(rad2deg(thetaList2(1:size(evenErr2,1))),[evenErr2],'-xr');
else
    figure,subplot(2,1,1);plot(rad2deg(thetaList(1:size(x_err1,1))), x_err1);hold on;plot(rad2deg(thetaList(1:size(x_err1,1))),[x_err1_upper x_err1_lower],'-xb');plot(rad2deg(thetaList(1:size(x_err1,1))),[mean(x_err1')],'-xr');title(sprintf('Shift Interval: %d',ShiftInterval));
    subplot(2,1,2);plot(rad2deg(thetaList(1:size(x_err1,1))), [mean(x_err1')],'-xr'); title(titleStr);% plot(rad2deg(thetaList2(1:size(x_err2,1))),x_err2);hold on;plot(rad2deg(thetaList2(1:size(x_err2,1))),[x_err2_upper x_err2_lower],'-xb');plot(rad2deg(thetaList2(1:size(evenErr2,1))),[evenErr2],'-xr');
    
end
% hold on;plot(rad2deg(thetaList), 10.*rad2deg(traceInfoMat{1,6}(1:end-1) - thetaList));
try
%     hold on;plot(rad2deg(thetaList), 10.*rad2deg(traceInfoMat{1,6}(1:end-1) - thetaList));
%     hold on;plot(rad2deg(thetaList), 10.*rad2deg(traceInfoMat{1,7}(1:end-1,:) - thetaList));
%     hold on;plot(rad2deg(thetaList), 10.*rad2deg(traceInfoMat{1,7}(1:end-1,end) - thetaList));
    hold on;plot(rad2deg(thetaList), 10.*rad2deg(traceInfoMat{1,7}(1:end-1,2) - thetaList));
catch
    try
        hold on;plot(rad2deg(thetaList), 10.*rad2deg(traceInfoMat{1,6}(1:end) - thetaList));
    catch
        sghjk = 1;
    end
end
try
    figure,hold on;plot(rad2deg(thetaList(1:size(x_err1,1))),x_err1(:,2:end));plot(rad2deg(thetaList), 10.*rad2deg(traceInfoMat{1,7}(1:end-1,:) - thetaList),'-or');
catch
    sgbjk = 1;
end

for i = 1 : size(XErrMat_body, 1) % size(traceInfoMat,1) - 2
    
    
%     frameRng1 = traceInfoMat{i,3} - 1;
%     xErrMat(i,frameRng1) = x_p2c(i,1:length(frameRng1));
%     yErrMat(i,frameRng1) = y_p2c(i,1:length(frameRng1));
    
    frameRng1 = [traceInfoMat{validCnt(i),3}(1) - 1 : traceInfoMat{validCnt(i),3}(end)];    
    dispErrMatDetailed_gt_bak(i,frameRng1) = [100 XErrMat_body(i,2:length(frameRng1))];
%     yErrMat(i,frameRng1) = y_p2c(i,1:length(frameRng1));
end

try
    objj.accumP2CRef = thetaList;
    figure,clf;hold on
    for jj = [14: 15]
        [~, ~, ~, tempP2CCumGT2] = CalcBound(objj, dispErrMatDetailed_gt_bak, dispErrMatDetailed_gt_bak, dispErrMatDetailed_gt_bak,jj);
        %     [~, ~, ~, tempP2CCumRef2] = CalcBound(obj, dispErrMatDetailed_gt_bak, dispErrMatDetailed_gt_bak, dispErrMatDetailed_gt_bak,jj);
        %     plot(rad2deg(tempP2CCumGT2 - tempP2CCumRef2));
        plot(rad2deg(thetaList(1:length(tempP2CCumGT2))),(tempP2CCumGT2));
        plot(rad2deg(thetaList([1:jj-1:length(tempP2CCumGT2)])), (tempP2CCumGT2([1:jj-1:length(tempP2CCumGT2)]) ), 'or');
        pause(0.5);
        drawnow;
    end
catch
    sdkfjg = 1;
end


return;


ss = cell2mat(MetricErr);
figure,hist(ss(:),10000)


goodId = find(max(abs(angErrMat)') < 110.1);


imgId = 2;
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

if 1
    b2c_vec(1:3,:) = b2c_solve__stack_2;
end


error0 = error;
b2c_vec0 = b2c_vec;
b2c_vec = b2c_vec(:,goodId);

[b2c_vec_norm, vAng] = NormalizeVector(b2c_vec(1:3,:)');

% ang2 = CalcDegree2(repmat(b2c_vec_norm(1,:),size(b2c_vec_norm,1),1),b2c_vec_norm);
% subplot(1,3,3),hist(real(ang2), 100);
if 1
    for ji = 1 : size(b2c_vec_norm,1)
        Ang2 = CalcDegree2(repmat(b2c_vec_norm(ji,:),size(b2c_vec_norm,1),1),b2c_vec_norm)';
        [sd(ji,:),fg(ji,:)] = hist(real(Ang2),50);
        %         figure,hist(real(Ang2),100);
        %         [~,idmax] = max(sd);title(num2str(ji));
        %         bestVec(ji,1) = fg(idmax);
    end
    [idmax] = (max(sd'));
    idMax = find(idmax == max(idmax));
    
    for jii = 1 : length(idMax)
        ffg = fg(idMax(jii),:);
        ssd = sd(idMax(jii),:);
        id12 = find(ssd == max(idmax));
        minVal(jii,1) = ffg(id12);
    end
    [~,minId12] = min(minVal);
    %     [~,bestId] = min(abs(bestVec));
    bestId = idMax(minId12);
end

figure,subplot(1,3,1); plot(b2c_vec([1 2 3],:)'); subplot(1,3,2),plotQuiver(b2c_vec_norm, [1 0 0], 2, 1);
ang2 = CalcDegree2(repmat(b2c_vec_norm(bestId,:),size(b2c_vec_norm,1),1),b2c_vec_norm);
subplot(1,3,3),cla;hist(real(ang2), 100);

b2c_vec_use = b2c_vec(:,1);
vecUse = Vec(:,bestId);
vecUse = [pi/2; pi/2];
[initErr,ppp] = CostFunc(b2c, traceX, traceY, intrMat, minFrameNum, cnt-1, validCnt, goodId, thetaList, XYZKey, numThr, RtTemp, Inliers,RtAng, vecUse);

if 0
    [vecOpt,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) CostFunc(b2c, traceX, traceY, intrMat, minFrameNum, cnt-1, validCnt, goodId, thetaList, XYZKey, numThr, RtTemp, Inliers,RtAng,X),[vecUse],[],[],options);%,data_1,obs_1)
    
    [optErr,ooo] = CostFunc(b2c, traceX, traceY, intrMat, minFrameNum, cnt-1, validCnt, goodId, thetaList, XYZKey, numThr, RtTemp, Inliers,RtAng, vecOpt);
    
    figure,plot([initErr optErr]);
    
    Err0 = Err;
end

asedkb = 1;

if 0
    chosenId = 1;
    aa = norm(b2c_vec(1:3,chosenId));
    bb = b2c_vec_norm(chosenId,:);
    bb(2) = 0;
    bb  = bb./norm(bb);
    b2cVec0 = [aa acos(bb(1))]';
    errVar0 = B2CErr(camPoseK2C_2, bodyPoseK2C_2,b2cVec0);
    options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter','MaxFunEvals',10000,'TolX',1e-8);%,'MaxFunEvals',10000);%, 'MaxIterations',5);
    [vec,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) B2CErr(camPoseK2C_2, bodyPoseK2C_2,X),[b2cVec0],[],[],options);%,data_1,obs_1)
    errVar1 = B2CErr(camPoseK2C_2, bodyPoseK2C_2,vec);
    figure,plot([errVar0 errVar1]);
    b2cAng = vec(1);
    b2cAxis = [cos(vec(2));0;sin(vec(2))];
    if 1
        b2c_vec(1:3,1)= b2cAng.*b2cAxis;
    end
    
    
end


end
function varargout = CostFunc(b2c, traceX, traceY, intrMat, minFrameNum, cnt0, validCnt, goodId, thetaList, XYZKey, numThr, RtTemp, Inliers,RtAng, rotAxisVec)
r = 1;
theta = rotAxisVec(1);
phi = rotAxisVec(2);
rotAxis = [r*sin(theta)*cos(phi); r*sin(theta)*sin(phi); r*cos(theta)];

global angPnpEqual


MEANX = [];
MEANY = [];
% b2c_vec_use = b2c_vec(:,1);
% cnt0 = cnt-1;
reproj_err_x = zeros(cnt0, minFrameNum - 1);
reproj_err_y = zeros(cnt0, minFrameNum - 1);

reproj_err = [];
for kkk = 1 : 1 % length(goodId)
    
    
    XErrMat = []; YErrMat = [];cnt = 1;
    
    
    
    % %     Rb2c = rodrigues(b2c_vec_use(1:3));
    
    
    
    for i = 1 : size(traceX,2) - (minFrameNum-1)
        if ismember(i, validCnt(goodId))
            thetaListTemp = thetaList(i : i+minFrameNum-1);
            
            
            thetaListTemp = thetaListTemp - thetaListTemp(1);
            
            nearestKeyX = traceX(:, i: i + minFrameNum-1);
            nearestKeyY = traceY(:, i: i + minFrameNum-1);
            featIdKey = find(min(nearestKeyX') > 0)';
            
            xMat = nearestKeyX(featIdKey,:);
            yMat = nearestKeyY(featIdKey,:);
            
            pix = [xMat(:,1) yMat(:,1)];
            
            xyzKey = XYZKey{cnt,1};
            
            if length(featIdKey) > numThr
                if (nargout > 1)
                    for j = 1 : minFrameNum - 1
                        pixCur = [xMat(:,j+1), yMat(:,j+1)];
                        %             [rt, idxOutliers] = posest(double(pixCur), double(xyzKey), 0.95, intrMat, 'repr_err');
                        
                        
                        
                        rt(4:6,1) =  RtTemp{cnt,1}(:,j);
                        
                        
                        
                        inliers = Inliers{cnt,1};
                        
                        
                        Rbody = roty(rad2deg(double(thetaListTemp(j+1))));
                        
                        
                        % %                         Rcam = Rb2c*Rbody*Rb2c';
                        
                        
                        angNorm = thetaListTemp(j+1);
                        
                        if angPnpEqual
                            angNorm = RtAng{cnt,1}(j);
                        end
                        Rcam = rodrigues(angNorm*rotAxis);
                        
                        Tbody = [roty(rad2deg(double(angNorm))) [0;0;0];0 0 0 1];
                        Tcam = b2c*Tbody*inv(b2c);
                        
                        if 0
                            ptIcs = TransformAndProject(xyzKey, intrMat, Rcam(1:3,1:3), rt(4:6));
                        else
                            ptIcs = TransformAndProject(xyzKey, intrMat, Rcam(1:3,1:3), Tcam(1:3,4));
                        end
                        
                        
                        reproj_err_x(cnt,j) = mean((ptIcs(inliers,1) - pixCur(inliers,1)).^2);
                        reproj_err_y(cnt,j) = mean((ptIcs(inliers,2) - pixCur(inliers,2)).^2);
                        
                        
                        
                        if (nargout > 1)
                            RT_body = b2c*  [roty(rad2deg(double(thetaListTemp(j+1)))) [0 0 0]';0 0 0 1] * inv(b2c);
                            ptIcs_body = TransformAndProject(xyzKey, intrMat, RT_body(1:3,1:3), RT_body(1:3,4));
                            
                            XErrMat(cnt,j) = mean(ptIcs(inliers,1) - pixCur(inliers,1));
                            YErrMat(cnt,j) = mean(ptIcs(inliers,2) - pixCur(inliers,2));
                            XErrMat_body(cnt,j) = mean(ptIcs_body(inliers,1) - pixCur(inliers,1));
                            YErrMat_body(cnt,j) = mean(ptIcs_body(inliers,2) - pixCur(inliers,2));
                            
                        end
                        
                    end
                end
                error = FixAxisPnp(intrMat, thetaList, xyzKey, xMat, yMat, minFrameNum, RtTemp{cnt,1}, b2c, Inliers{cnt,1}, RtAng{cnt,1},rotAxisVec);
                
                reproj_err = [reproj_err;error];
                cnt = cnt + 1;
            end
        end
    end
    
    %     [~, reproj_err] = NormalizeVector([reproj_err_x(:) reproj_err_y(:)]);
    
    varargout{1} = reproj_err;
    
    if (nargout > 1)
        varargout{2} = rotAxis; %XErrMat;
        %         varargout{3} = YErrMat;
        %         varargout{4} = XErrMat_body;
        %         varargout{5} = YErrMat_body;
        
        figure,subplot(2,4,1);plot(-(XErrMat(:,:))');title('x');
        subplot(2,4,2),plot(-(YErrMat(:,:))');title('y');
        subplot(2,4,5);plot(-mean(XErrMat(:,:)));title('k2c x');
        subplot(2,4,6),plot(-mean(YErrMat(:,:)));title('k2c y');
        
        
        subplot(2,4,3);plot(-(XErrMat_body(:,:))');title('x');
        subplot(2,4,4),plot(-(YErrMat_body(:,:))');title('y');
        
        subplot(2,4,7);plot(-mean( diff([ zeros(size(XErrMat_body,1 ), 1) XErrMat_body(:,:)]')' ));title('p2c x');
        subplot(2,4,8),plot(-mean(diff([zeros(size(XErrMat_body,1 ), 1)  YErrMat_body(:,:)]')'));title('p2c y');
        
    else
        asgk = 1;
    end
    
    
    % figure,plot(b2c_vec([1 3],id)');
end

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

function errVar = CostFunc2(b2cVec, theta, pix, depthListGT, intrMat, depthGTCur,imgSize)
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
function [x_err1, x_err1_upper, x_err1_lower] = CalcEnvelope(xErrMat2, traceNum2)
for u = 1 : traceNum2
    weightMatNorm_temp = CalcWeight(abs(xErrMat2), 1, 1, 1,u);
    x_err1(:,u) = cumsum(sum(weightMatNorm_temp.*xErrMat2)');
    
    
end
x_err1_upper = max(x_err1')';
x_err1_lower = min(x_err1')';
end
function evenErr = EvenWeightErr(xErrMatFull)

validAngOptFull2 = abs(xErrMatFull) > 0;
validAngOptSumFull2 = sum(validAngOptFull2);
xErrMatAccumFull = (sum(xErrMatFull)./validAngOptSumFull2)';

evenErr = cumsum(xErrMatAccumFull);

end
function ShiftInterval = ReadConfig(accFilePath)

ShiftInterval = [];
[configFileFid, errMsg] = fopen(accFilePath);
if (configFileFid < 0)
    error('Cannot open %s for read: %s', accFilePath, errMsg);
end
% gVecBuf = [];
% inReadgVec = false;
% flagg=0;


while ~feof(configFileFid)
    lineBuf = strtrim(fgetl(configFileFid));
    if 1 %length(lineBuf) > 10
        
        
        if strcmp(lineBuf(1:13), 'ShiftInterval')
            ShiftInterval = str2double((lineBuf(16:end)));
        end
        
        
        if strcmp(lineBuf(1:6), 'is_yuv')
            cfg.is_yuv = str2double((lineBuf(9:end)));
        end
        if strcmp(lineBuf(1:6), 'calib_')
            cfg.calib_stereo = str2double((lineBuf(15:end)));
        end
        if strcmp(lineBuf(1:6), 'wide_a')            
            cfg.wide_angle = str2double((lineBuf(13:end)));
        end
        if strcmp(lineBuf(1:6), 'cb_row')            
            cfg.cb_row = str2double((lineBuf(9:end)));
        end
        if strcmp(lineBuf(1:6), 'cb_col')            
            cfg.cb_col = str2double((lineBuf(9:end)));
        end
        if strcmp(lineBuf(1:6), 'cb_siz')            
            cfg.cb_size = str2double((lineBuf(10:end)));
        end
        if strcmp(lineBuf(1:6), 'show_f')            
            cfg.show_fig = str2double((lineBuf(11:end)));
        end
        if strcmp(lineBuf(1:6), 'img_pr')            
            cfg.img_prefix = ((lineBuf(13:end)));
        end
        if strcmp(lineBuf(1:6), 'img_su')            
            cfg.img_suffix = ((lineBuf(13:end)));
        end
        if strcmp(lineBuf(1:6), 'img_su')            
            cfg.img_suffix = ((lineBuf(13:end)));
        end
        if strcmp(lineBuf(1:6), 'img_wi')            
            cfg.img_width = str2double((lineBuf(12:end)));
        end
        if strcmp(lineBuf(1:6), 'img_he')            
            cfg.img_height = str2double((lineBuf(13:end)));
        end
        if strcmp(lineBuf(1:6), 'img_sc')            
            cfg.img_scale = str2double((lineBuf(12:end)));
        end
        if strcmp(lineBuf(1:6), 'switch')            
            cfg.switch_lr = str2double((lineBuf(12:end)));
        end
        if strcmp(lineBuf(1:6), 'est_k3')            
            cfg.est_k3 = str2double((lineBuf(9:end)));
        end
    end
   
    
end
% G=1;
% g=gVec;
fclose(configFileFid);


end