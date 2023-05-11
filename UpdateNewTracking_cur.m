function [thetaProbUse, thetaProbUse_, thetaProbUse__, curPredPtIcs, weightRaw, candX, candY,candX_reshape, candY_reshape, weightRaw_reshape,idff] = UpdateNewTracking(obj, thetaSamp, thetaProb, prevFeatPtList0, ptCcsZ0, intrMat, AngleModalOrg, DepthProbility, DepthId, inlierId, DispRng)
global NewOnlyP2


%                                         thetaProbUse = thetaProb(ismember(round(rad2deg(thetaSamp),3),round(obj.setting.configParam.angleRange,3)));
if 0
    thetaProbUse = thetaProb(ismember(round(rad2deg(thetaSamp),3),round(obj.setting.configParam.angleRange,3)));
else
    thetaProbUse = interp1(round(rad2deg(thetaSamp),3),thetaProb,round(obj.setting.configParam.angleRange,3));
    thetaProbUse(isnan(thetaProbUse)) = min(thetaProb);
    if 0
        figure, plot(rad2deg(thetaSamp), thetaProb);hold on;plot(obj.setting.configParam.angleRange, thetaProbUse,'-x');legend('orig','interp')
    end
    
    
end
thetaProbUse = thetaProbUse./max(thetaProbUse);
thetaProbUse_ = thetaProbUse;
if sum(thetaProbUse <= obj.configParam.only_p2_prob_cutoff_threshold) > 0 && sum(thetaProbUse <= obj.configParam.only_p2_prob_cutoff_threshold) ~= length(thetaProbUse)
    thetaProbUse_(thetaProbUse > obj.configParam.only_p2_prob_cutoff_threshold) = 1; % max(thetaProbUse(thetaProbUse <= obj.configParam.only_p2_prob_cutoff_threshold));
elseif sum(thetaProbUse <= obj.configParam.only_p2_prob_cutoff_threshold) == length(thetaProbUse)
    
    thetaProbUse_ = thetaProbUse;
else
    thetaProbUse_ = ones(length(thetaProbUse_),1);
end


thetaProbUse__ = thetaProbUse_./sum(thetaProbUse_);

if 1
    if ~NewOnlyP2
        [curPredPtIcs,inTrackFlag,curPredPtIcs0,inTrackFlag0,weightRaww,candXX,candYY] = VisualLocalizer.NewTracking(obj, size(obj.featPtManager.localTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs, AngleModalOrg, obj.keyFrameDepth, obj.prvDepthVisual, inlierId,DepthProbility, DepthId, thetaProbUse__,1);
        %                                             [~,~,~,~,weightRaw,candX,candY] = NewTracking(obj, size(obj.featPtManager.localTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs, AngleModalOrg, obj.keyFrameDepth, obj.prvDepthVisual, obj.depthVisual,DepthProbility, DepthId, thetaProbUse__,1);
    else
        [curPredPtIcs,inTrackFlag,curPredPtIcs0,inTrackFlag0] = VisualLocalizer.NewTracking_new(obj, size(obj.featPtManager.localTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs, AngleModalOrg, obj.keyFrameDepth, obj.prvDepthVisual, inlierId ,DepthProbility, DepthId, thetaProbUse__,1);
        
    end
else
    
%     CalcCandXYWeight(obj, size(obj.featPtManager.localTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs, AngleModalOrg, obj.keyFrameDepth, obj.prvDepthVisual, inlierId,DepthProbility, DepthId, thetaProbUse__,1, DispRng)
    [curPredPtIcs,inTrackFlag,curPredPtIcs0,inTrackFlag0,weightRaww,candXX,candYY] = VisualLocalizer.NewTrackingTemp(obj, size(obj.featPtManager.localTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs, AngleModalOrg, obj.keyFrameDepth, obj.prvDepthVisual, inlierId,DepthProbility, DepthId, thetaProbUse__,1, DispRng);
end

idI = find(obj.featPtManager.localTrace.ptIcsX(:,end-1) ~= -1 & obj.featPtManager.localTrace.ptIcsY(:,end-1) ~= -1);
tempPt = -1.*ones(size(obj.featPtManager.localTrace.ptIcsX,1),2);
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
            
            figure(1),clf;hist([errP2_old errP2_new],100);legend('old','new')
            saveas(gcf,fullfile(probPath,sprintf(strcat('refinedTracking_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('refinedTracking_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
            
        catch
            sdgahl = 1;
        end
        
        
        
        figure,imshow(imgCur);hold on;plot(pt2dCur(:,1),pt2dCur(:,2),'.r');plot(pixGT(:,1),pixGT(:,2),'.b');plot(tempPt((inlierId),1),tempPt((inlierId),2),'.g');legend('old','gt','new');
        figure,hist([errP2_old errP2_new],500);legend('old','new')
        figure,subplot(1,2,1);hold on;plot(tempPt((inlierId),1), tempPt((inlierId),2),'.g');plot(pt2dCur(:,1),pt2dCur(:,2),'.r');axis equal;subplot(1,2,2);plot(errP2);
    end
    if 0
        pt2dCur = tempPt((inlierId),:);
        
        obj.featPtManager.localTrace.ptIcsX(inlierId,end) = tempPt((inlierId),1);
        obj.featPtManager.localTrace.ptIcsY(inlierId,end) = tempPt((inlierId),2);
    end
end

end