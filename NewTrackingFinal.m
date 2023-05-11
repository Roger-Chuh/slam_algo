function [curPredPtIcs,inTraceFlag,curPredPtIcs0,inTraceFlag0, topMargin] = NewTrackingFinal(obj,prevImg,nextImg, keyFeatNum,prevFeatPtList,ptCcsZ,intrMat,k2cRef,angleModalOrg)

OnlyP2 = false; true;


f = intrMat(1); baseline = norm(obj.camModel.transVec1To2);
fbConst = f*baseline;
[princpPtL, princpPtR] = Get(obj.camModel, 'PinholePrincpPt', obj.scaleLvl);

T_B2C = obj.coordSysAligner.pctBody2Cam(1, 1).transformMat;
invTb2c = inv(T_B2C);
featuresNumber = keyFeatNum;
sys = SettingSystem(T_B2C,intrMat,baseline);



if 0
    activeFeat =  find(obj.featPtManager.localTrace.ptIcsX(:,end) ~= -1 & obj.featPtManager.localTrace.ptIcsY(:,end) ~=-1);
    ptIcX_pre = obj.featPtManager.localTrace.ptIcsX(:,1);
    ptIcY_pre = obj.featPtManager.localTrace.ptIcsY(:,1);
else
    activeFeat = [1:keyFeatNum]';
    ptIcX_pre = prevFeatPtList(:,1);
    ptIcY_pre = prevFeatPtList(:,2);
end
featuresNumber = size(ptIcX_pre,1);
ptCcsZStereo = ptCcsZ;
angleRound = rad2deg(k2cRef);

obj.setting2.angleOrginRange = angleRound + obj.setting2.angleRange;
obj.setting2.configParam.angleOrginRange = obj.setting2.angleOrginRange;

disparityBoundary = obj.setting2.disparityBoundary;
disparityIntervel = obj.setting2.disparityIntervel;
angleBoundary = obj.setting2.angleBoundary;
angleIntervel = obj.setting2.angleIntervel;
wx = obj.setting2.wx;
wy = obj.setting2.wy;
pyramidLevel = 1;
settingTracking = SettingTracking(disparityBoundary,disparityIntervel,angleBoundary,angleIntervel,wx,wy,pyramidLevel);
angleBoundary2  = 179;
settingModal = SettingModal(disparityBoundary,disparityIntervel,angleBoundary2,angleIntervel);

imgSize = size(nextImg); imgSize = imgSize(1:2);
pt1 = Points.Point_final(ptIcX_pre,ptIcY_pre,intrMat);
[pt1.matchingPtX,pt1.matchingPtY,curPtCcsZGolden] = Points.GenerateMatchingPt_final(pt1,angleRound,sys.T_B2C,ptCcsZStereo,featuresNumber,sys.intrMat) ;

existIdx = find(pt1.matchingPtX-obj.setting2.configParam.wx >= 1 & pt1.matchingPtY-obj.setting2.configParam.wy>= 1 ...
    & pt1.matchingPtX+obj.setting2.configParam.wx <= imgSize(2) & pt1.matchingPtY+obj.setting2.configParam.wy<= imgSize(1) ...
    &  ptCcsZ~=-1); % & ptCcsZGolden ~= -1);



existFeaturesNumber = size(existIdx,1);
existPt1 = Points2.Point2(pt1.x(existIdx),pt1.y(existIdx),intrMat);
% existPt2 = Points2.Point2(pt1.matchingPtX(existIdx),pt1.matchingPtY(existIdx),intrMat);
settingTracking.angleOrginRange = angleRound + settingTracking.angleRange;



existPtCcsZG = ptCcsZStereo(existIdx);
existPtDisparityG = sys.fbConst./existPtCcsZG;
settingTracking.disparityOrginRange  = existPtDisparityG + settingTracking.disparityRange;

table = LookUpTables.LookUpTable_final(existFeaturesNumber,settingTracking.tableSize); %initialize
try
    table.winSize = obj.lookUpTables2.winSize;
    table.angSize = obj.lookUpTables2.angSize;
    table.angMove = obj.lookUpTables2.angMove;
catch
    %% 20200408
    rsegthk = 1;
end
table = LookUpTables.GenerateTableCandidates2_final(table,settingTracking,existPt1,existFeaturesNumber, imgSize,sys.T_B2C,sys.fbConst,sys.intrMat,existIdx);
tracePt = TraceByLK2(existFeaturesNumber);%initialize

tic
[tracePtA,saveIdx] = TracingLK2withPyramid(tracePt,settingTracking,table,prevImg,nextImg,existPt1,existFeaturesNumber,sys);
toc
tracePt.x =tracePtA.x;
tracePt.y =tracePtA.y;
existIdx = existIdx(saveIdx);


existPt1 = Points.Point_final(ptIcX_pre(existIdx),ptIcY_pre(existIdx),intrMat);
existPt2 = Points.Point_final(tracePt.x,tracePt.y,intrMat);
if 0
    [existPt1,existPt2,existIdx,existFeaturesNumber] = Points.DeOutliers(existPt1,existPt2,sys.intrMat,sys.T_B2C,existIdx);
end
[expectValue2, m] = OnlyP2_final(obj, [existPt1.x existPt1.y], [existPt2.x existPt2.y], intrMat);

if 0
    figure,subplot(1,2,1);imshow(prevImg);hold on;plot(existPt1.x, existPt1.y, '.r');subplot(1,2,2);imshow(nextImg);hold on;plot(existPt2.x, existPt2.y, '.r');
    OnlyP2_final(obj, [existPt1.x existPt1.y], [existPt2.x existPt2.y], intrMat)
end

topMargin = obj.featPtTracker.configParam.top_margin;
leftMargin = obj.featPtTracker.configParam.left_margin;
bottomMargin = obj.featPtTracker.configParam.bottom_margin;
rightMargin = obj.featPtTracker.configParam.right_margin;

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
% obj.modals2 = m;

end