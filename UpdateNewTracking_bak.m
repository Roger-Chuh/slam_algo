function [thetaProbUse, thetaProbUse_, thetaProbUse__, curPredPtIcs, weightRaw, candX, candY,candX_reshape, candY_reshape, weightRaw_reshape,idff,errPair,PixReprojExpXY,objPervPtCcsZGolden1,p2cErrList] = ...
    UpdateNewTracking_bak(obj, thetaSamp, thetaProb, prevFeatPtList0, ptCcsZ0, intrMat, AngleModalOrg, DepthProbility, DepthId, inlierId, DispRng,DispRefineUse__, angOptP2C,DispKeyInPrvRng,angOpt,thetaRng, thetaProbP2C, k2cRef,k2cRef0,thetaRngP2C,thetaSampP2C,thetaProbP2CBeforeNewPlatform,thetaProbBeforeNewPlatform,thetaRngP2CBeforeNewPlatform,thetaRngBeforeNewPlatform,thetaP2C,thetaP2C0,...
    objPervPtCcsZGolden,ThetaNoise,CamModel,CoordSysAligner,LocalTrace,imgCur,imgPrvL,REFAngList,REFAngList4,prvDepthVisual,prvDepthGT,p2cErrList,KeyFrameFlagList,ConfigParam,accumP2CTemp)
global NewOnlyP2 roundingPrecision probPath EXPZERO UseGoldenThetaP2C doRoundingTheta FrmNumWithGloden FigBase

UseStandAloneFunc = true; 


roundingPrecision1  = 0.00000000000000001;
useK2C = true; false; true; false; true; false; true;false; true; false; true; false; true; false;true; false;true;
keepShift = true;  false; true; false; true; false; true; false; true; false; true; false; true; false; true;
if size(LocalTrace.ptIcsX, 2) <= FrmNumWithGloden + 1  %40  %90
    keepShift = true;
    useK2C = true;
else
    keepShift = false;
    if 0
        useK2C = false;
    else
        useK2C = true;
    end
end

%  k2cRef = deg2rad(round(rad2deg(k2cRef0 + obj.thetaNoise)./roundingPrecision).*roundingPrecision);


if isempty(thetaProbP2C)
    if ~EXPZERO
        angleMiddle = deg2rad(round(rad2deg(angOpt)./roundingPrecision1).*roundingPrecision1);
    else
        angleMiddle = k2cRef0;
    end
    
    if UseGoldenThetaP2C
        angleMiddle =  k2cRef0;k2cRef;
    end
    
    
    
else
    if ~useK2C
        if ~EXPZERO
            angleMiddle = deg2rad(round(rad2deg(angOptP2C)./roundingPrecision1).*roundingPrecision1);
        else
            angleMiddle =  k2cRef0 - REFAngList(end);
        end
        
        if UseGoldenThetaP2C
            angleMiddle =  k2cRef0 - REFAngList(end);thetaP2C;
        end
        
        
    else
        if 0
            if ~EXPZERO
                angleMiddle = deg2rad(round(rad2deg(angOpt)./roundingPrecision1).*roundingPrecision1);
            else
                angleMiddle = k2cRef0;
            end
            if UseGoldenThetaP2C
                angleMiddle = k2cRef0;
            end
        else
            if ~EXPZERO
                angleMiddle = deg2rad(round(rad2deg(angOpt - REFAngList4(end))./roundingPrecision1).*roundingPrecision1);
            else
                angleMiddle = k2cRef0 - REFAngList(end);
            end
            if UseGoldenThetaP2C
                angleMiddle = k2cRef0 - REFAngList(end);thetaP2C;
            end
        end
% %         if UseGoldenThetaP2C
% %             angleMiddle = k2cRef0;
% %         end
    end
end

if isempty(thetaProbP2C)
    goldenP2C = k2cRef0;
    rounding_VslP2C = deg2rad(round(rad2deg(angOpt)./roundingPrecision).*roundingPrecision);
    roundingGtK2C_minus_VslK2P = k2cRef;
    roundingGtK2C_minus_VslK2P_round = k2cRef;
else
    goldenP2C = k2cRef0 - REFAngList(end);
    rounding_VslP2C = deg2rad(round(rad2deg(angOptP2C)./roundingPrecision).*roundingPrecision);
    roundingGtK2C_minus_VslK2P = k2cRef - REFAngList4(end);
    thetaP2C0;
    thetaP2C;
    roundingGtK2C_minus_VslK2P_round = thetaP2C;
end


% angleMiddle = angleMiddle + 0.05*(rand(1)-0.5);
% angleMiddle = angleMiddle + 0.01*;
% angleMiddle = angleMiddle + deg2rad(0.05*(-1)^(size(LocalTrace.ptIcsX, 2)));
if 0
    if ~useK2C
        p2cErr = rad2deg([angleMiddle rounding_VslP2C roundingGtK2C_minus_VslK2P roundingGtK2C_minus_VslK2P_round] - goldenP2C);
    else
        %     p2cErr = rad2deg([angleMiddle rounding_VslP2C roundingGtK2C_minus_VslK2P roundingGtK2C_minus_VslK2P_round] - [k2cRef0 goldenP2C goldenP2C goldenP2C]);
        p2cErr = rad2deg([angOpt rounding_VslP2C roundingGtK2C_minus_VslK2P roundingGtK2C_minus_VslK2P_round] - [k2cRef0 goldenP2C goldenP2C goldenP2C]);
    end
else
    if 1 % ~useK2C
        p2cErr = rad2deg([angleMiddle rounding_VslP2C roundingGtK2C_minus_VslK2P roundingGtK2C_minus_VslK2P_round] - goldenP2C);
    else
        %     p2cErr = rad2deg([angleMiddle rounding_VslP2C roundingGtK2C_minus_VslK2P roundingGtK2C_minus_VslK2P_round] - [k2cRef0 goldenP2C goldenP2C goldenP2C]);
        p2cErr = rad2deg([angOpt rounding_VslP2C roundingGtK2C_minus_VslK2P roundingGtK2C_minus_VslK2P_round] - [k2cRef0 goldenP2C goldenP2C goldenP2C]);
    end
end



figure(FigBase + 2);
if size(LocalTrace.ptIcsX, 2) == 2
    clf;
    p2cErrList = [];
end


p2cErrList = [p2cErrList; p2cErr];


figure(FigBase + 2);clf;plot(p2cErrList(:,1),'-*r');hold on;plot(p2cErrList(:,2),'-og');plot(p2cErrList(:,3),'-xb');plot(p2cErrList(:,4),'-sk');legend('VslP2C','rounding VslP2C','rounding GtK2C minus VslK2P','round(round(k2c) - vsl(t-1))');title('angle - gt');
saveas(gcf,fullfile(probPath,sprintf(strcat('p2cErr_',probPath(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('p2cErr_',probPath(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__*.png'))))+1 + FigBase)));
% figure,

%                                         thetaProbUse = thetaProb(ismember(round(rad2deg(thetaSamp),3),round(obj.setting.configParam.angleRange,3)));
if 0
    %     thetaProbUse = thetaProb(ismember(round(rad2deg(thetaSamp),3),round(obj.setting.configParam.angleRange,3)));
    thetaProbUse = thetaProb(ismember(round(rad2deg(thetaRng),3),round(obj.setting.configParam.angleRange + rad2deg(angleMiddle),3)));
else
    if 0
        thetaProbUse = interp1(round(rad2deg(thetaSamp),3),thetaProb,round(obj.setting.configParam.angleRange,3));
        thetaProbUse(isnan(thetaProbUse)) = min(thetaProb);
        if 0
            figure, plot(rad2deg(thetaSamp), thetaProb);hold on;plot(obj.setting.configParam.angleRange, thetaProbUse,'-x');legend('orig','interp')
        end
    elseif 1
        if isempty(thetaProbP2C)
            if keepShift
                thetaProb = thetaProbBeforeNewPlatform;
            end
            %             thetaProbUse = interp1(round(rad2deg(thetaRng),3),thetaProb,round(obj.setting.configParam.angleRange + rad2deg(angleMiddle),3));
            %             thetaProbUse = interp1(round(rad2deg(thetaRng),3),thetaProb,round(obj.setting.configParam.angleRange + rad2deg(mean(thetaRngBeforeNewPlatform)),3));
            if keepShift
                thetaProbUse = interp1(round(rad2deg(thetaRng - k2cRef),3),thetaProb,round(obj.setting.configParam.angleRange + 0,3));
            else
                thetaProbUse = interp1(round(rad2deg(thetaRng),3),thetaProb,round(obj.setting.configParam.angleRange + rad2deg(angleMiddle),3));
            end
        else
            if keepShift
                if   ~useK2C
                    thetaProb = thetaProbP2CBeforeNewPlatform;
                else
                    thetaProb = thetaProbBeforeNewPlatform;
                end
                
            else
                if   ~useK2C%0%
                    thetaProb = thetaProbP2C;
                else
                    thetaProb = thetaProb;
                end
            end
            if  ~useK2C %0%
                if keepShift
                    thetaProbUse = interp1(round(rad2deg(thetaRngP2C - thetaP2C),3),thetaProb,round(obj.setting.configParam.angleRange + 0,3));
                else
                    thetaProbUse = interp1(round(rad2deg(thetaRngP2C),3),thetaProb,round(obj.setting.configParam.angleRange + rad2deg(angleMiddle),3));
                end
            else
                %                 thetaProbUse = interp1(round(rad2deg(thetaRng),3),thetaProb,round(obj.setting.configParam.angleRange + rad2deg(angleMiddle),3));
                %                 thetaProbUse = interp1(round(rad2deg(thetaRng),3),thetaProb,round(obj.setting.configParam.angleRange + rad2deg(angOpt),3));
                %                 thetaProbUse = interp1(round(rad2deg(thetaRng),3),thetaProb,round(obj.setting.configParam.angleRange + rad2deg(mean(thetaRngBeforeNewPlatform)),3));
                if keepShift
                    thetaProbUse = interp1(round(rad2deg(thetaRng - k2cRef),3),thetaProb,round(obj.setting.configParam.angleRange + 0,3));
                else
                    thetaProbUse = interp1(round(rad2deg(thetaRng),3),thetaProb,round(obj.setting.configParam.angleRange + rad2deg(angleMiddle),3));
                end
                
            end
            
        end
        thetaProbUse(isnan(thetaProbUse)) = min(thetaProb);
        midId1 = (length(thetaRng) + 1)/2;
        midId2 = (length(obj.setting.configParam.angleRange) + 1)/2;
        if 0
            figure, plot(rad2deg(thetaSamp), thetaProb);hold on;plot(obj.setting.configParam.angleRange, thetaProbUse,'-x');legend('orig','interp')
            if isempty(thetaProbP2C)
                figure,plot(rad2deg(thetaRng),thetaProb,'-');hold on;plot(obj.setting.configParam.angleRange + rad2deg(angleMiddle),thetaProbUse,'-' );plot(rad2deg(thetaRng(midId1)),thetaProb(midId1),'-s');plot(obj.setting.configParam.angleRange(midId2) + rad2deg(angleMiddle),thetaProbUse(midId2),'-x' );
            else
                if ~useK2C
                    figure,plot(rad2deg(thetaRngP2C),thetaProb,'-');hold on;plot(obj.setting.configParam.angleRange + rad2deg(angleMiddle),thetaProbUse,'-' );plot(rad2deg(thetaRngP2C(midId1)),thetaProb(midId1),'-s');plot(obj.setting.configParam.angleRange(midId2) + rad2deg(angleMiddle),thetaProbUse(midId2),'-x' );
                elseif 0
                    figure,plot(rad2deg(thetaRng),thetaProb,'-');hold on;plot(obj.setting.configParam.angleRange + rad2deg(angleMiddle),thetaProbUse,'-' );plot(rad2deg(thetaRng(midId1)),thetaProb(midId1),'-s');plot(obj.setting.configParam.angleRange(midId2) + rad2deg(angleMiddle),thetaProbUse(midId2),'-x' );
                    figure,plot(rad2deg(thetaRng),thetaProb,'-');hold on;plot(obj.setting.configParam.angleRange + rad2deg(angOpt),thetaProbUse,'-' );plot(rad2deg(thetaRng(midId1)),thetaProb(midId1),'-s');plot(obj.setting.configParam.angleRange(midId2) + rad2deg(angOpt),thetaProbUse(midId2),'-x' );
                    figure,plot(rad2deg(thetaRngP2C),thetaProb,'-');hold on;plot(obj.setting.configParam.angleRange + rad2deg(angleMiddle),thetaProbUse,'-' );plot(rad2deg(thetaRngP2C(midId1)),thetaProb(midId1),'-s');plot(obj.setting.configParam.angleRange(midId2) + rad2deg(angOpt),thetaProbUse(midId2),'-x' );
                end
            end
            figure,plot(rad2deg(thetaSamp),thetaProb,'-o');hold on;plot(obj.setting.configParam.angleRange + rad2deg(angleMiddle) - rad2deg(k2cRef),thetaProbUse,'-x' );
        end
    else
        thetaProbUse = interp1(round(rad2deg(thetaSamp),3),thetaProb,round(obj.setting.configParam.angleRange + rad2deg(angleMiddle) - rad2deg(k2cRef),3));
        thetaProbUse(isnan(thetaProbUse)) = min(thetaProb);
        if 0
            figure, plot(rad2deg(thetaSamp), thetaProb);hold on;plot(obj.setting.configParam.angleRange, thetaProbUse,'-x');legend('orig','interp');
            figure,plot(rad2deg(thetaRng),thetaProb,'-o');hold on;plot(obj.setting.configParam.angleRange + rad2deg(angleMiddle),thetaProbUse,'-x' );
            figure,plot(rad2deg(thetaSamp),thetaProb,'-o');hold on;plot(obj.setting.configParam.angleRange + rad2deg(angleMiddle) - rad2deg(k2cRef),thetaProbUse,'-x' );
        end
    end
    
end




thetaProbUse = thetaProbUse./max(thetaProbUse);
thetaProbUse_ = thetaProbUse;
if sum(thetaProbUse <= ConfigParam.only_p2_prob_cutoff_threshold) > 0 && sum(thetaProbUse <= ConfigParam.only_p2_prob_cutoff_threshold) ~= length(thetaProbUse)
    thetaProbUse_(thetaProbUse > ConfigParam.only_p2_prob_cutoff_threshold) = 1; % max(thetaProbUse(thetaProbUse <= ConfigParam.only_p2_prob_cutoff_threshold));
elseif sum(thetaProbUse <= ConfigParam.only_p2_prob_cutoff_threshold) == length(thetaProbUse)
    
    thetaProbUse_ = thetaProbUse;
else
    thetaProbUse_ = ones(length(thetaProbUse_),1);
end


thetaProbUse__ = thetaProbUse_./sum(thetaProbUse_);

if isempty(angOptP2C)
    angOpt = angleMiddle;
    thetaRngUse = thetaRng;
else
    angOptP2C = angleMiddle;
    thetaRngUse = thetaRngP2C;
end
% angleMiddle = angleMiddle + deg2rad(0.05*(-1)^(size(LocalTrace.ptIcsX, 2)));


if 1 %~UseGoldenThetaP2C
    %     thetaExp = dot(angleMiddle + deg2rad(obj.setting.configParam.angleRange ), thetaProbUse./sum(thetaProbUse));
    thetaExp = dot(angleMiddle + deg2rad(obj.setting.configParam.angleRange ), thetaProbUse./sum(thetaProbUse));
    if 1
        thetaExp = angleMiddle;
        if keepShift
            if isempty(thetaP2C)
                thetaExp = mean(thetaRngBeforeNewPlatform); angleMiddle;
            else
                thetaExp = mean(thetaRngP2CBeforeNewPlatform); thetaP2C;
            end
        end
    end
elseif 0
    if isempty(thetaP2C)
        thetaExp = accumP2CTemp(end);
    else
        thetaExp = thetaP2C;
    end
    % dispExp
else
    if ~isempty(thetaP2C)
        thetaExp = goldenP2C;  % + deg2rad(0.01*(rand(1)-0.5));    %deg2rad(0.02*(-1)^(size(LocalTrace.ptIcsX, 2)));
    else
        thetaExp = k2cRef0;
    end
end

if isempty(angOptP2C)
    angOpt = thetaExp; % angleMiddle;
    thetaRngUse = thetaRng;
else
    angOptP2C = thetaExp; % angleMiddle;
    thetaRngUse = thetaRngP2C;
end


if 1 % ~useK2C
    if ~NewOnlyP2
        if ~UseStandAloneFunc
            [curPredPtIcs,inTrackFlag,curPredPtIcs0,inTrackFlag0,weightRaww,candXX,candYY,errPair, objPervPtCcsZGolden1] = VisualLocalizer.NewTracking(obj, size(LocalTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs, angOpt, DispKeyInPrvRng, angOptP2C, inlierId,DepthProbility, DepthId, thetaProbUse__,1,DispRefineUse__,DispRng,objPervPtCcsZGolden,ThetaNoise,CamModel,CoordSysAligner,LocalTrace,imgCur,imgPrvL,REFAngList,REFAngList4,prvDepthVisual,prvDepthGT);
        else
            [curPredPtIcs,inTrackFlag,curPredPtIcs0,inTrackFlag0,weightRaww,candXX,candYY,errPair,objPervPtCcsZGolden1] = NewTracking(obj, size(LocalTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, k2cRef0, angOpt, DispKeyInPrvRng, angOptP2C, inlierId,DepthProbility, DepthId, thetaProbUse__,1,DispRefineUse__,DispRng,objPervPtCcsZGolden,ThetaNoise,CamModel,CoordSysAligner,LocalTrace,imgCur,imgPrvL,REFAngList,REFAngList4,prvDepthVisual,prvDepthGT);
        end
        %         [curPredPtIcs,inTrackFlag,curPredPtIcs0,inTrackFlag0,weightRaww,candXX,candYY,errPair] = VisualLocalizer.NewTracking(obj, size(LocalTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs, angOpt, DispKeyInPrvRng, angleMiddle, inlierId,DepthProbility, DepthId, thetaProbUse__,1,DispRefineUse__,DispRng);
        %                                             [~,~,~,~,weightRaw,candX,candY] = NewTracking(obj, size(LocalTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs, AngleModalOrg, obj.keyFrameDepth, obj.prvDepthVisual, obj.depthVisual,DepthProbility, DepthId, thetaProbUse__,1);
    else
        [curPredPtIcs,inTrackFlag,curPredPtIcs0,inTrackFlag0] = VisualLocalizer.NewTracking_new(obj, size(LocalTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs, angOpt, DispKeyInPrvRng, angOptP2C, inlierId ,DepthProbility, DepthId, thetaProbUse__,1,DispRefineUse__);
        %         [curPredPtIcs,inTrackFlag,curPredPtIcs0,inTrackFlag0] = VisualLocalizer.NewTracking_new(obj, size(LocalTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs, angOpt, DispKeyInPrvRng, angleMiddle, inlierId ,DepthProbility, DepthId, thetaProbUse__,1,DispRefineUse__);
        
    end
else
    
    %     CalcCandXYWeight(obj, size(LocalTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs, AngleModalOrg, obj.keyFrameDepth, obj.prvDepthVisual, inlierId,DepthProbility, DepthId, thetaProbUse__,1, DispRng)
    %     [curPredPtIcs,inTrackFlag,curPredPtIcs0,inTrackFlag0,weightRaww,candXX,candYY] = VisualLocalizer.NewTrackingTemp(obj, size(LocalTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs, angOpt, DispKeyInPrvRng, angOptPrv, inlierId,DepthProbility, DepthId, thetaProbUse__,1, DispRng,DispRefineUse__);
    [curPredPtIcs,inTrackFlag,curPredPtIcs0,inTrackFlag0,weightRaww,candXX,candYY,errPair] = VisualLocalizer.NewTrackingTemp(obj, size(LocalTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs, angOpt, DispKeyInPrvRng, angOptP2C, inlierId,DepthProbility, DepthId, thetaProbUse__,1, DispRng,DispRefineUse__);
end

idI = find(LocalTrace.ptIcsX(:,end-1) ~= -1 & LocalTrace.ptIcsY(:,end-1) ~= -1);
tempPt = -1.*ones(size(LocalTrace.ptIcsX,1),2);
tempPt(idI,:) = curPredPtIcs;

iff = (tempPt(:,1) ~= -1);
idff = find(ismember(find(iff),inlierId));

iff(setdiff(1:size(tempPt,1),inlierId)) = false;
tempPt2 = tempPt(iff,:);

candX = candXX(idff,:);
candY = candYY(idff,:);
weightRaw = weightRaww(idff,:);


candXT = candX';
candX_reshape = reshape(candXT(:),obj.setting.configParam.angleBin, obj.setting.configParam.disparityBin, size(candX,1) );

candYT = candY';
candY_reshape = reshape(candYT(:),obj.setting.configParam.angleBin, obj.setting.configParam.disparityBin, size(candX,1) );

weightRawT = weightRaw';
weightRaw_reshape = reshape(weightRawT(:),obj.setting.configParam.angleBin, obj.setting.configParam.disparityBin, size(candX,1) );
%                                             candX_reshape = permute(candX_reshape,[2 3 1]);



PixReprojExpX = candX_reshape((size(candX_reshape,1)+1)/2, (size(candX_reshape,2)+1)/2, :);
PixReprojExpY = candY_reshape((size(candY_reshape,1)+1)/2, (size(candY_reshape,2)+1)/2, :);
PixReprojExpXY = [PixReprojExpX(:) PixReprojExpY(:)];

if 1
    if 0
        checkGT = 10;
        tempCandX = reshape(candX(idff(checkGT),:),obj.setting.configParam.angleBin,obj.setting.configParam.disparityBin);
        tempCandY = reshape(candY(idff(checkGT),:),obj.setting.configParam.angleBin,obj.setting.configParam.disparityBin);
        tempWeight = reshape(weightRaw(idff(checkGT),:),obj.setting.configParam.angleBin,obj.setting.configParam.disparityBin)*1000;tempWeight = tempWeight./max(tempWeight(:));
        %                                                 figure,imshow(tempWeight, [])
        idMaxWeight = find(tempWeight(:) > 0.8);
        %                                                 [idMaxWeight] = find(tempWeight(:) == max(tempWeight(:)))
        [weightY,weightX] = ind2sub(size(tempWeight),idMaxWeight)
        figure,subplot(1,2,1);imshow(imresize(tempWeight,2),[]);subplot(1,2,2);imshow(imgCur);hold on;plot(tempCandX(idMaxWeight),tempCandY(idMaxWeight),'.r');plot(pixGT(checkGT,1),pixGT(checkGT,2),'.g');legend('candidate','gt');
    end
    
    
    asgarvn = 1;
    
    
    if 0
        
        try
            [~, errP2] = NormalizeVector(tempPt((inlierId),:) - pt2dCur);
            
            [~, errP2_old] = NormalizeVector(pt2dCur - pixGT);
            [~, errP2_new] = NormalizeVector(tempPt((inlierId),:) - pixGT);
            
            figure(FigBase + 1),clf;hist([errP2_old errP2_new],100);legend('old','new')
            saveas(gcf,fullfile(probPath,sprintf(strcat('refinedTracking_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('refinedTracking_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__*.png'))))+1 + FigBase)));
            
        catch
            sdgahl = 1;
        end
        
        
        
        figure,imshow(imgCur);hold on;plot(pt2dCur(:,1),pt2dCur(:,2),'.r');plot(pixGT(:,1),pixGT(:,2),'.b');plot(tempPt((inlierId),1),tempPt((inlierId),2),'.g');legend('old','gt','new');
        figure,hist([errP2_old errP2_new],500);legend('old','new')
        figure,subplot(1,2,1);hold on;plot(tempPt((inlierId),1), tempPt((inlierId),2),'.g');plot(pt2dCur(:,1),pt2dCur(:,2),'.r');axis equal;subplot(1,2,2);plot(errP2);
    end
    if 0
        pt2dCur = tempPt((inlierId),:);
        
        LocalTrace.ptIcsX(inlierId,end) = tempPt((inlierId),1);
        LocalTrace.ptIcsY(inlierId,end) = tempPt((inlierId),2);
    end
end

end