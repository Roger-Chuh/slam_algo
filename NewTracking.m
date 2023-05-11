function [curPredPtIcs,inTraceFlag,curPredPtIcs0,inTraceFlag0,weightRaw,candX,candY,errPair,objPervPtCcsZGolden1] = NewTracking(obj,keyFeatNum,prevFeatPtList,ptCcsZ,intrMat,k2cRef,angleModalOrg, keyDepthV, prevDepthV, currDepthV,DepthProbility, DepthId, AngleProbility,reCalc,DispRefineUse__,DispRng,objPervPtCcsZGolden,ThetaNoise,CamModel,CoordSysAligner,LocalTrace,imgCur,imgPrvL,REFAngList,REFAngList4,prvDepthVisual,prvDepthGT)
global doRoundingTheta UsePrvDepth EvenDepthProb InheriDispRng roundingPrecision NewPrvDispResamp

inlierId = currDepthV;
angOptP2C = prevDepthV;
DispKeyInPrvRng = keyDepthV;
angOpt = angleModalOrg;
if EvenDepthProb
    
    if ~isempty(DepthProbility)
        DepthProbility = ones(size(DepthProbility));
    end
    
end

if ~exist('ThetaNoise', 'var')
    ThetaNoise = 0;
end


k2cRef0 = k2cRef;
if doRoundingTheta
    %                 k2cRef = deg2rad(round(rad2deg(k2cRef),1));
    k2cRef = deg2rad(round(rad2deg(k2cRef0 + ThetaNoise)./roundingPrecision).*roundingPrecision);
    
end

OnlyP2 = false; true;


f = intrMat(1); baseline = norm(CamModel.transVec1To2);
fbConst = f*baseline;
[princpPtL, princpPtR] = Get(CamModel, 'PinholePrincpPt', obj.scaleLvl);

T_B2C = CoordSysAligner.pctBody2Cam(1, 1).transformMat;
invTb2c = inv(T_B2C);
featuresNumber = keyFeatNum;
activeFeat =  find(LocalTrace.ptIcsX(:,end - reCalc) ~= -1 & LocalTrace.ptIcsY(:,end - reCalc) ~=-1);
ptIcX_pre = LocalTrace.ptIcsX(:,1);
ptIcY_pre = LocalTrace.ptIcsY(:,1);
%             pt1 = Points.Point(ptIcX_pre,ptIcY_pre,intrMat);
pt0 = Points.Point(ptIcX_pre,ptIcY_pre,intrMat);
ptCcsZGolden = LocalTrace.ptCcsZ(:,1);
commonId = find(ismember(DepthId, inlierId));
if  size(LocalTrace.ptIcsX,2) > 2
    if 0
    ptCcsZGolden(DepthId) = (intrMat(1,1).*norm(CamModel.transVec1To2))./(DispRng(:,(size(DispRng,2)+1)/2) + (princpPtR(1) - princpPtL(1)));
    else
        ptCcsZGolden = (intrMat(1,1).*norm(CamModel.transVec1To2))./(DispRng(commonId,(size(DispRng,2)+1)/2) + (princpPtR(1) - princpPtL(1)));
        
    end
end

objPervPtCcsZGolden = objPervPtCcsZGolden(commonId,:);



%             m_ = Modals.Modal(obj.setting.configParam, featuresNumber);
%             m_ = obj.modals;
%             angleModalOrg = m_.angleModal;

if 0 %~isempty(DispRefineUse__)
    
    if 0
        figure, plot((intrMat(1,1).*norm(CamModel.transVec1To2))./(DispRefineUse__ + (princpPtR(1) - princpPtL(1))) - ptCcsZGolden(DepthId));
    end
    
    
    ptCcsZGolden(DepthId) = (intrMat(1,1).*norm(CamModel.transVec1To2))./(DispRefineUse__ + (princpPtR(1) - princpPtL(1)));
end
%             existPtDisparityG = (intrMat(1,1).*norm(CamModel.transVec1To2)./existPtCcsZG) - (princpPtR(1) - princpPtL(1));


%             depthModalOrg = m_.depthModal;
if size(imgCur,3) > 1
    nextImg = rgb2gray(imgCur);
    %             prevImg = obj.keyFrameImgL;
    prevImg = rgb2gray(imgPrvL);
else
    nextImg = (imgCur);
    prevImg = (imgPrvL);
end
imgSize = size(nextImg); imgSize = imgSize(1:2);

%             if isempty(REFAngList)

if ~reCalc
    numThr = 1;
else
    numThr = 2;
end


if size(LocalTrace.ptIcsX,2) == numThr
    angleOrgin = 0;
else
    %                 angleOrgin = rad2deg(REFAngList3(end));
    angleOrgin = rad2deg(REFAngList4(end));
end
[pt0.matchingPtX,pt0.matchingPtY,pervPtCcsZGolden] = Points.GenerateMatchingPt(pt0,angleOrgin,T_B2C,invTb2c,ptCcsZGolden,featuresNumber,intrMat) ;

%             obj.PervPtCcsZGolden = pervPtCcsZGolden;

angleOrgin = rad2deg(k2cRef);
[pt00.matchingPtX,pt00.matchingPtY,pervPtCcsZGolden00] = Points.GenerateMatchingPt(pt0,angleOrgin,T_B2C,invTb2c,ptCcsZGolden,featuresNumber,intrMat) ;

ptIcX_pre = -1 * ones(featuresNumber,1);
ptIcY_pre = -1 * ones(featuresNumber,1);



if size(LocalTrace.ptIcsX,2) == numThr
    
    ptIcX_pre = LocalTrace.ptIcsX(:,1);
    ptIcY_pre = LocalTrace.ptIcsY(:,1);
else
    
    existIdx0 = find(LocalTrace.ptIcsX(:,end - reCalc)-obj.setting.wx > 1 & LocalTrace.ptIcsY(:,end - reCalc)-obj.setting.wy> 1 ...
        & LocalTrace.ptIcsX(:,end - reCalc)+obj.setting.wx < imgSize(2) & LocalTrace.ptIcsY(:,end - reCalc)+obj.setting.wy< imgSize(1) ...
        & LocalTrace.ptIcsX(:,end-1 - reCalc)-obj.setting.wx > 1 & LocalTrace.ptIcsY(:,end-1 - reCalc)-obj.setting.wy> 1 ...
        & LocalTrace.ptIcsX(:,end-1-reCalc)+obj.setting.wx < imgSize(2) & LocalTrace.ptIcsY(:,end-1 - reCalc)+obj.setting.wy< imgSize(1) ...
        &  objPervPtCcsZGolden~=-1);
    ptIcX_pre(existIdx0) = LocalTrace.ptIcsX(existIdx0,end - reCalc);
    ptIcY_pre(existIdx0) = LocalTrace.ptIcsY(existIdx0,end - reCalc);
    
    
end
%%
if 1
    pt1 = Points.Point(ptIcX_pre,ptIcY_pre,intrMat);
else
    pt1 = Points.Point(pt0.matchingPtX,pt0.matchingPtY,intrMat);
    
end
ptCcsZGolden = pervPtCcsZGolden';
%             if isempty(REFAngList)







if size(LocalTrace.ptIcsX,2) == numThr
    if 0
        angleOrgin = rad2deg(k2cRef);
    else
        angleOrgin = rad2deg(angOpt);
    end
    %                 errPair =  [rad2deg(angOpt - k2cRef0) rad2deg(k2cRef - k2cRef0)];
    errPair =  [rad2deg(k2cRef - k2cRef0) rad2deg(angOpt - k2cRef0)];
else
    %                 angleOrgin = rad2deg(k2cRef - REFAngList3(end));
    if isempty(angOptP2C)
        if 0
            angleOrgin = rad2deg(k2cRef - REFAngList4(end));
        else
            angleOrgin = rad2deg(angOpt);
        end
    else
        angleOrgin = rad2deg(angOptP2C);
    end
    errGTP2C1 = -(rad2deg(k2cRef0 - REFAngList(end)) - rad2deg(angOptP2C));
    errGTP2C2 =  -(rad2deg(k2cRef0 - REFAngList(end)) - (rad2deg(k2cRef - REFAngList4(end))));
    errPair = [errGTP2C2 errGTP2C1];
    
end
%             try
%                 errGTP2C1 = -(rad2deg(k2cRef0 - REFAngList(end)) - rad2deg(angOptP2C));
%                 errGTP2C2 =  -(rad2deg(k2cRef0 - REFAngList(end)) - (rad2deg(k2cRef - REFAngList4(end))));
%                 errPair = [errGTP2C2 errGTP2C1];
%             catch
%                 errPair =  [rad2deg(k2cRef - k2cRef0) rad2deg(k2cRef - k2cRef0)];
%             end
[pt1.matchingPtX,pt1.matchingPtY,curPtCcsZGolden] = Points.GenerateMatchingPt(pt1,angleOrgin,T_B2C,invTb2c,ptCcsZGolden,featuresNumber,intrMat) ;

obj.setting.angleOrginRange = angleOrgin + obj.setting.angleRange;
obj.setting.configParam.angleOrginRange = obj.setting.angleOrginRange;


%              validInd = sub2ind(size(obj.depthVisual), round(pt1.y), round(pt1.x));
%               ptCcsZGolden = obj.keyFrameDepthGT(validInd);

if 0
    [pt1.matchingPtX,pt1.matchingPtY] = Points.GenerateMatchingPt(pt1,angleOrgin,T_B2C,invTb2c,ptCcsZGolden,featuresNumber,intrMat) ;
    
    
    existIdx = find(pt1.matchingPtX-obj.setting.configParam.wx >= 1 & pt1.matchingPtY-obj.setting.configParam.wy>= 1 ...
        & pt1.matchingPtX+obj.setting.configParam.wx <= imgSize(2) & pt1.matchingPtY+obj.setting.configParam.wy<= imgSize(1) ...
        &  ptCcsZ~=-1 & ptCcsZGolden ~= -1);
elseif 0
    [pt1.matchingPtX,pt1.matchingPtY] = Points.GenerateMatchingPt(pt1,angleOrgin,T_B2C,invTb2c,ptCcsZ,featuresNumber,intrMat) ;
    
    
    existIdx = find(pt1.matchingPtX-obj.setting.configParam.wx >= 1 & pt1.matchingPtY-obj.setting.configParam.wy>= 1 ...
        & pt1.matchingPtX+obj.setting.configParam.wx <= imgSize(2) & pt1.matchingPtY+obj.setting.configParam.wy<= imgSize(1) ...
        &  ptCcsZ~=-1); % & ptCcsZGolden ~= -1);
else
    [pt1.matchingPtX,pt1.matchingPtY] = Points.GenerateMatchingPt(pt1,angleOrgin,T_B2C,invTb2c,ptCcsZ,featuresNumber,intrMat) ;
    
    if 0
        existIdx = find(pt1.matchingPtX-obj.setting.configParam.wx >= 1 & pt1.matchingPtY-obj.setting.configParam.wy>= 1 ...
            & pt1.matchingPtX+obj.setting.configParam.wx <= imgSize(2) & pt1.matchingPtY+obj.setting.configParam.wy<= imgSize(1) ...
            & ptCcsZGolden ~= -1); % & ptCcsZ~=-1
    else
        % %                     existIdx = find(pt1.matchingPtX-obj.setting.configParam.wx >= 1 & pt1.matchingPtY-obj.setting.configParam.wy>= 1 ...
        % %                         & pt1.matchingPtX+obj.setting.configParam.wx <= imgSize(2) & pt1.matchingPtY+obj.setting.configParam.wy<= 126 ...
        % %                         & ptCcsZGolden ~= -1);
        
        
        if ~reCalc
            existIdx = find(pt1.matchingPtX-obj.setting.configParam.wx > 1 & pt1.matchingPtY-obj.setting.configParam.wy> 1 ...
                & pt1.matchingPtX+obj.setting.configParam.wx < imgSize(2) & pt1.matchingPtY+obj.setting.configParam.wy< 126*(240/imgSize(1)) ...
                & pt1.x-obj.setting.configParam.wx > 1 & pt1.y-obj.setting.configParam.wy> 1 ...
                & pt1.x+obj.setting.configParam.wx < imgSize(2) & pt1.y+obj.setting.configParam.wy< 126*(240/imgSize(1)) ...
                &  ptCcsZGolden~=-1);
        else
            if 0
                existIdx = inlierId;
            else
                existIdx = [1:length(inlierId)]';
            end
        end
        
    end
    
end

objPervPtCcsZGolden1 = ptCcsZGolden;
existFeaturesNumber = size(existIdx,1);
existPt1 = Points.Point(pt1.x(existIdx),pt1.y(existIdx),intrMat);
existPt2 = Points.Point(pt1.matchingPtX(existIdx),pt1.matchingPtY(existIdx),intrMat);

existPtCcsZG = ptCcsZGolden(existIdx);
existPtDisparityG1 = fbConst./existPtCcsZG;
existPtDisparityG = (intrMat(1,1).*norm(CamModel.transVec1To2)./existPtCcsZG) - (princpPtR(1) - princpPtL(1));


if ~UsePrvDepth
    if 0
        obj.setting.configParam.disparityOrginRange   = existPtDisparityG + obj.setting.configParam.disparityRange;
        
    elseif 1
        if  size(LocalTrace.ptIcsX,2) > 2
            dltDisp = DispRng - repmat(DispRng(:,(size(DispRng,2)+1)/2), 1, size(DispRng,2));
            dltDisp = dltDisp(ismember(DepthId, inlierId),:);
            if ~NewPrvDispResamp
                obj.setting.configParam.disparityOrginRange   = repmat(existPtDisparityG,1,size(DispRng,2)) + dltDisp;
            else
                obj.setting.configParam.disparityOrginRange = DispKeyInPrvRng;
            end
            
        else
            obj.setting.configParam.disparityOrginRange   = existPtDisparityG + obj.setting.configParam.disparityRange;
        end
    else
        obj.setting.configParam.disparityOrginRange   = DispRng;
    end
    
else
    if obj.switchDepth
        prvDepth = prvDepthGT;
    else
        prvDepth = prvDepthVisual;
        
    end
    if  size(LocalTrace.ptIcsX,2) > 2
        prvFeat = [existPt1.x existPt1.y];
        if 0
            prvFeatInd = sub2ind(size(prvDepth), round(prvFeat(:,2)), round(prvFeat(:,1)));
            prvFeatDepth = prvDepth(prvFeatInd);
            prvFeatDisp = (intrMat(1,1).*norm(CamModel.transVec1To2)./prvFeatDepth) - (princpPtR(1) - princpPtL(1));
        else
            if obj.switchDepth
                prvFeatDepth = LocalTrace.ptCcsZGT(:,end-1);
                prvFeatDisp = LocalTrace.dispListGT(:,end-1);
            else
                prvFeatDepth = LocalTrace.ptCcsZ(:,end-1);
                prvFeatDisp = LocalTrace.dispList(:,end-1);
            end
        end
        
        dltDisp = DispRng - repmat(DispRng(:,(size(DispRng,2)+1)/2), 1, size(DispRng,2));
        dltDisp = dltDisp(ismember(DepthId, inlierId),:);
        if InheriDispRng
            %                         obj.setting.configParam.disparityOrginRange   = prvFeatDisp + obj.setting.configParam.disparityRange;
            obj.setting.configParam.disparityOrginRange   = repmat(prvFeatDisp,1,size(DispRng,2)) + dltDisp;
        else
            obj.setting.configParam.disparityOrginRange   = prvFeatDisp + obj.setting.configParam.disparityRange;
        end
        errCheckDisp1 = prvFeatDisp - DispRng(ismember(DepthId,inlierId),(size(DispRng,2)+1)/2);
        errCheckDisp2 = prvFeatDisp - existPtDisparityG;
        errCheckDisp12 = [errCheckDisp1 errCheckDisp2];
    else
        
        obj.setting.configParam.disparityOrginRange   = existPtDisparityG + obj.setting.configParam.disparityRange;
    end
    
end
%             table = LookUpTables.LookUpTable(obj.lookUpTables,existFeaturesNumber,obj.setting.configParam.tableSize); %initialize
table = LookUpTables.LookUpTable(existFeaturesNumber,obj.setting.configParam.tableSize); %initialize
table.winSize = obj.lookUpTables.winSize;
table.angSize = obj.lookUpTables.angSize;
table.angMove = obj.lookUpTables.angMove;
if 1 %~OnlyP2
    table = LookUpTables.GenerateTable(table,obj.setting.configParam,existPt1,existFeaturesNumber, imgSize,T_B2C,fbConst,intrMat,DepthProbility, DepthId, inlierId, AngleProbility,DispRefineUse__) ;
else
    table = LookUpTables.GenerateTableOnlyP2(table,obj.setting.configParam,existPt1,existPt2,existFeaturesNumber, imgSize,T_B2C,invTb2c,fbConst,intrMat, depthProbility) ;
end

weightRaw = table.candidatesWeightRaw;
candX = table.candidatesPtX;
candY = table.candidatesPtY;

% % % % % % % % % % %             if reCalc
% % % % % % % % % % %
% % % % % % % % % % %                 curPredPtIcs = [];inTraceFlag = [];curPredPtIcs0 = [];inTraceFlag0 = [];
% % % % % % % % % % %
% % % % % % % % % % %                 return;
% % % % % % % % % % %             end

% %             m_.angleModal = angleModalOrg;
%             if ~OnlyP2
%                 % -- update modal angle -- %
%                 m_.angleModal = Modals.UpdateModalAngle(m_,obj.setting.configParam,table,existIdx);
%                 % -- update modal depth -- %
%                 m_.depthModal = Modals.UpdateModalDepth(m_,obj.setting.configParam,table,existIdx,existFeaturesNumber);
%             else
%                 m_.angleModal = Modals.UpdateModalAngleOnlyP2(m_,obj.setting.configParam,table,existIdx);
%
%             end
% % %             if ~OnlyP2
% % %                 table.candidatesWeight = LookUpTables.GenerateTableWeight(table,obj.setting.configParam,m_,existIdx);
% % %             else
% % %                 table.candidatesWeight = LookUpTables.GenerateTableWeightOnlyP2(table,obj.setting.configParam,m_,existIdx);
% % %             end
% ------get tracing pts with weights------ %
tracePt = TraceWithWeights.TraceWithWeight(existFeaturesNumber);%initialize
if ~reCalc
    tracePt = TraceWithWeights.Tracing(tracePt,obj.setting.configParam,table,prevImg,nextImg,existPt1,existFeaturesNumber,imgSize); % new tracing Pts
else
    tracePt.x = existPt1.x;
    tracePt.y = existPt1.y;
end
err_x1 = pt00.matchingPtX(existIdx) - tracePt.x;
err_y1 = pt00.matchingPtY(existIdx) - tracePt.y;


% -----re-calculate guassian---- %
% % % %             if ~OnlyP2
% % % %                 table = LookUpTables.RecalculateTableGauss(table,obj.setting.configParam,tracePt,existFeaturesNumber);
% % % %             else
% % % %                 table = LookUpTables.RecalculateTableGaussOnlyP2(table,obj.setting.configParam,tracePt,existFeaturesNumber);
% % % %             end
% -----re-calculate modal angle---- %
% % % % % % % %             m_.angleModal = angleModalOrg;
% % % % % % % %
% % % % % % % %             if ~OnlyP2
% % % % % % % %                 m_.angleModal = Modals.UpdateModalAngle(m_,obj.setting.configParam,table,existIdx);
% % % % % % % %                 % -----re-calculate modal depth---- %
% % % % % % % %                 m_.depthModal = depthModalOrg;
% % % % % % % %                 m_.depthModal = Modals.UpdateModalDepth(m_,obj.setting.configParam,table,existIdx,existFeaturesNumber);
% % % % % % % %             else
% % % % % % % %                 m_.angleModal = Modals.UpdateModalAngleOnlyP2(m_,obj.setting.configParam,table,existIdx);
% % % % % % % %             end
%             err = [ptIcX_cur(existIdx) - tracePt.x ptIcY_cur(existIdx) - tracePt.y];[~,errr]=NormalizeVector(err);
%         figure(9),clf;plot(err(errr<1,1),err(errr<1,2),'+r')
if 0
    figure,imshow(prevImg);hold on;plot(existPt1.x,existPt1.y,'.r')
    figure,imshow(nextImg);hold on;plot(tracePt.x,tracePt.y,'.r')
end

if ~reCalc
    topMargin = obj.featPtTracker.configParam.top_margin;
    leftMargin = obj.featPtTracker.configParam.left_margin;
    bottomMargin = obj.featPtTracker.configParam.bottom_margin;
    rightMargin = obj.featPtTracker.configParam.right_margin;
else
    topMargin = 1; obj.featPtTracker.configParam.top_margin;
    leftMargin = 1;obj.featPtTracker.configParam.left_margin;
    bottomMargin = 1;obj.featPtTracker.configParam.bottom_margin;
    rightMargin = 1;
    
    
end
inBndFlag = tracePt.x(:, 1) >= leftMargin + 1 & ...
    tracePt.x(:, 1) <= Cols(nextImg) - rightMargin & ...
    tracePt.y(:, 1) >= topMargin + 1 & ...
    tracePt.y(:, 1) <= Rows(nextImg) - bottomMargin;
inTrackFlag =  inBndFlag;
%             validNewPt = tracePt.x;

existIdx(~inTrackFlag) = [];
inTraceFlag = false(keyFeatNum,1);
inTraceFlag(existIdx) = true;

curPredPtIcs = -1.*ones(keyFeatNum,2);
curPredPtIcs(existIdx,:) = [tracePt.x(inTrackFlag) tracePt.y(inTrackFlag)];

curPredPtIcs0 = curPredPtIcs;
inTraceFlag0 = inTraceFlag;

curPredPtIcs = curPredPtIcs(activeFeat,:);
inTraceFlag = inTraceFlag(activeFeat,:);
%             obj.modals = m_;
end