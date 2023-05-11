
function DTD2(obj, imgInd, imgPrv, imgCur, depthPrv, depthCur, thetaList)
global ReplaceOnlyP2 ReplaceOnlyP2Z thetaListGT ShiftByCoord probPath mean_x_err...
    err_x UseDoubleShift rou Rou UseEx Distri

AngleModalOrg = obj.angleModalOrg;
% NewTracker = true; false; true;

obj.accumP2CRef = thetaList;
% PrvTrackOnGT = false; true;

[marg, ~, ~, ~] = GetPredictMargin(obj.featPtTracker);
intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,obj.scaleLvl);
[princpPtL, princpPtR] = Get(obj.camModel, 'PinholePrincpPt', obj.scaleLvl);
b2cPmat = GetPctBody2Cam(obj.coordSysAligner, 1);
b2c = b2cPmat.transformMat;
pnp_ang_est_max_margin = [deg2rad([-1 1]) 0.5];
T_B2C = b2c;


r_cam = sqrt(T_B2C(1,4)^2+T_B2C(2,4)^2+T_B2C(3,4)^2);
tx = T_B2C(1,4)/r_cam;
ty = T_B2C(2,4)/r_cam;
tz = T_B2C(3,4)/r_cam;
baseline = obj.camModel.transVec1To2;
thetaP2C = thetaList(end) - thetaList(end-1);
p2cRef = thetaList(end) - thetaList(end-1);
k2cRef = p2cRef;

if isempty(obj.traceManager.X)
    prvFrmNum = 1;
    [pixPrv, zList] = DetectValidFeature(imgPrv, depthPrv, intrMat, 0.02, marg);
    [predPtList, inTrackFlag, ~] = TrackFeaturePoints(obj.featPtTracker, imgPrv, imgCur, pixPrv, [], intrMat, b2cPmat);
    
    [k2cBodyRotAng_2,Err_2,inId, reprojErrList] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin,intrMat,pixPrv(inTrackFlag,:),predPtList(inTrackFlag,:),zList(inTrackFlag), find(inTrackFlag),true(sum(inTrackFlag),1),b2c,double(thetaP2C));
    inId0 = inId;
    vldPtCur = FindValidDepth(predPtList(inId0,:), depthCur, marg);
    
    inId = inId0(vldPtCur > 0,:);
    vldPtCurValid = vldPtCur(ismember(inId0, inId),:);
    
    pixPrvValid = pixPrv(inId,:);
    pixCurValid = predPtList(inId,:);
    zListValid = zList(inId,:);
    [pixCurReproj] = VisualLocalizer.GetGtTrace2(b2cPmat, k2cBodyRotAng_2, [pixPrvValid],zListValid,intrMat);
    
    if 0
        figure,subplot(1,2,1); showMatchedFeatures(imgPrv, imgCur, pixPrv(inTrackFlag,:), predPtList(inTrackFlag,:));subplot(1,2,2);plot(pixCurReproj(:,1) - pixCurValid(:,1), pixCurReproj(:,2) - pixCurValid(:,2), '+r'); axis equal;
    end
    
    
    
    
    unValidId = setdiff([1: max(inId)]', inId);
    obj.traceManager.X(inId, 1) = pixPrvValid(:,1);
    obj.traceManager.Y(inId, 1) = pixPrvValid(:,2);
    obj.traceManager.Z(inId, 1) = zListValid(:,1);
    
    %     obj.traceManager.XLK(inId, 1) = pixPrvValid(:,1);
    %     obj.traceManager.YLK(inId, 1) = pixPrvValid(:,2);
    %     obj.traceManager.ZLK(inId, 1) = zListValid(:,1);
    
    obj.traceManager.X(inId, 2) = pixCurValid(:,1);
    obj.traceManager.Y(inId, 2) = pixCurValid(:,2);
    obj.traceManager.Z(inId, 2) = vldPtCurValid(:,1);
    
    %     obj.traceManager.XLK(inId, 2) = pixCurValid(:,1);
    %     obj.traceManager.YLK(inId, 2) = pixCurValid(:,2);
    %     obj.traceManager.ZLK(inId, 2) = vldPtCurValid(:,1);
    
    obj.traceManager.X(unValidId, 1) = -1;
    obj.traceManager.Y(unValidId, 1) = -1;
    obj.traceManager.Z(unValidId, 1) = -1;
    
    
    obj.traceManager.X(unValidId, 2) = -1;
    obj.traceManager.Y(unValidId, 2) = -1;
    obj.traceManager.Z(unValidId, 2) = -1;
    
    obj.traceManager.ZGT = obj.traceManager.Z;
    vdhlas = 1;
else
    prvFrmNum = size(obj.traceManager.X, 2);
    
    curValidPt = find(obj.traceManager.X(:,prvFrmNum) > 0);
    prevFeatPtList_22 = [obj.traceManager.X(curValidPt,prvFrmNum) obj.traceManager.Y(curValidPt,prvFrmNum)];
    prevFeatPtList_22_z = obj.traceManager.Z(curValidPt,prvFrmNum);
    
    depthMapPrvGT = depthPrv;
    depthMapCurGT = depthCur;
    
    
    
    sakgj = 1;
    
    
    
    [predPtList000, inTrackFlag000, marg] = TrackFeaturePoints(obj.featPtTracker, imgPrv, imgCur, prevFeatPtList_22, [], intrMat, GetPctBody2Cam(obj.coordSysAligner));
    
    
    
    vldPtCur = nan(size(predPtList000,1),1);
    vldPrTrace = find(predPtList000(:,1) > marg & predPtList000(:,2) > marg & predPtList000(:,1) < size(depthMapCurGT,2) - marg & predPtList000(:,2) < size(depthMapCurGT,1) - marg);
    
    vldPtCur_ = sub2ind(size(depthMapCurGT), round(predPtList000(vldPrTrace,2)), round(predPtList000(vldPrTrace,1)));
    vldPtCur_depth = depthMapCurGT(vldPtCur_);
    
    vldPtCur(vldPrTrace) = vldPtCur_depth;
    
    % %     marg = 5;
    vldPtPrv = FindValidDepth(prevFeatPtList_22, depthMapPrvGT, marg);
    vldPtCur11 = FindValidDepth(predPtList000, depthMapCurGT, marg);
    
    
    depthMapPrvTmp = depthPrv; %depthMapKey_2; % keyFrameDepth;
    depthMapPrvTmp(depthMapPrvTmp == -1) = nan;
    
    depthMapCurTmp = depthCur;
    depthMapCurTmp(depthMapCurTmp == -1) = nan;
    
    vldPtPrv_tmp = FindValidDepth(prevFeatPtList_22, depthMapPrvTmp, marg);
    vldPtCur11_tmp = FindValidDepth(predPtList000, depthMapCurTmp, marg);
    
    if 0
        figure,plot(vldPtCur11(~isnan(vldPtCur11)) - vldPtCur(~isnan(vldPtCur)))
    end
    
    
    
    inTrackFlag000(isnan(vldPtPrv) | isnan(vldPtCur11) | isnan(vldPtPrv_tmp) | isnan(vldPtCur11_tmp)) = false;
    
    
    featPtList___1 = VisualLocalizer.DetectFast(imgPrv);
    
    vldPtPrvNew = sub2ind(size(depthMapPrvGT), round(featPtList___1(:,2)), round(featPtList___1(:,1)));
    depth_vldPtPrvNew = depthMapPrvGT(vldPtPrvNew);
    if 1
        featPtList___ = featPtList___1(~isnan(depth_vldPtPrvNew),:);
    else
        vldPtPrv = FindValidDepth(prevFeatPtList_22, depthMapPrvGT, marg);
        vldPtCur11 = FindValidDepth(predPtList000, depthMapCurGT, marg);
        
        featPtList___ = featPtList___1(~isnan(depth_vldPtPrvNew),:);
        
    end
    sdagkb = 1;
    
    if 1
        scal = size(depthMapCurTmp,1)/240;
        
        pixMarg = 40*scal; % 80*scal; % 40*scal;
        minNum = 100; 100000; 100; 200; 100; 60; 10; 50; 50;
        
        if p2cRef > 0
            validROI = find(featPtList___(:,1) < min(prevFeatPtList_22(:,1) + pixMarg));
            %             if ~isempty(validROI)
            if length(validROI) > minNum*scal
                featPtList = featPtList___(validROI,:);
            else
                % featPtList = featPtList___;
                
                
                featPtList___1 = VisualLocalizer.DetectFast(imgPrv, 0.02); % 0.01 0.02
                
                vldPtPrvNew = sub2ind(size(depthMapPrvGT), round(featPtList___1(:,2)), round(featPtList___1(:,1)));
                depth_vldPtPrvNew = depthMapPrvGT(vldPtPrvNew);
                
                featPtList___ = featPtList___1(~isnan(depth_vldPtPrvNew),:);
                
                validROI = find(featPtList___(:,1) < min(prevFeatPtList_22(:,1) + pixMarg));
                featPtList = featPtList___(validROI,:);
                
                
                
            end
            if 0
                figure,imshow(zeros(size(depthMapCurTmp)));hold on;plot(featPtList(:,1),featPtList(:,2),'.r');plot(prevFeatPtList_22(:,1),prevFeatPtList_22(:,2),'.g');
            end
            sdfgkjh = 1;
        else
            validROI = find(featPtList___(:,1) > max(prevFeatPtList_22(:,1) - pixMarg));
            %             if ~isempty(validROI)
            if length(validROI) > minNum*scal
                featPtList = featPtList___(validROI,:);
            else
                % featPtList = featPtList___;
                
                
                
                featPtList___1 = VisualLocalizer.DetectFast(imgPrv, 0.02); % 0.01 0.02
                
                vldPtPrvNew = sub2ind(size(depthMapPrvGT), round(featPtList___1(:,2)), round(featPtList___1(:,1)));
                depth_vldPtPrvNew = depthMapPrvGT(vldPtPrvNew);
                
                featPtList___ = featPtList___1(~isnan(depth_vldPtPrvNew),:);
                
                %                 validROI = find(featPtList___(:,1) < min(prevFeatPtList_22(:,1) + pixMarg));
                validROI = find(featPtList___(:,1) > max(prevFeatPtList_22(:,1) - pixMarg));
                featPtList = featPtList___(validROI,:);
                
                
                
                
            end
            if 0
                figure,imshow(zeros(size(depthMapCurTmp)));hold on;plot(prevFeatPtList_22(:,1),prevFeatPtList_22(:,2),'.g');plot(featPtList(:,1),featPtList(:,2),'.r');
            end
            
            
            if 0 % length(obj.keyFrameFlagList) - obj.addPt(end) >= addPtInterval
                featPtList = [];
            end
            
            sagkbj = 1;
        end
        
    else
        
        featPtList = featPtList___;
    end
    
    
    
    [predPtList111, inTrackFlag111] = TrackFeaturePoints(obj.featPtTracker, imgPrv, imgCur, featPtList, [], intrMat, GetPctBody2Cam(obj.coordSysAligner));
    
    
    
    if  0 % length(obj.keyFrameFlagList) - obj.addPt(end) < addPtInterval
        inTrackFlag111 = false(length(inTrackFlag111), 1);
    else
        obj.addPt = [obj.addPt; length(obj.keyFrameFlagList)];
    end
    
    
    depthMapCurTmp2 = depthCur;
    depthMapCurTmp2(depthMapCurTmp2 == -1) = nan;
    
    vldPtCur_newAdd = FindValidDepth(predPtList111, depthMapCurGT, marg);
    vldPtCur_newAdd2 = FindValidDepth(predPtList111, depthMapCurTmp2, marg);
    
    inTrackFlag111(isnan(vldPtCur_newAdd) | isnan(vldPtCur_newAdd2)) = false;
    
    
    
    pt0 = predPtList000(inTrackFlag000,:);
    pt1 = predPtList111(inTrackFlag111,:);
    pt1Init = featPtList(inTrackFlag111,:);
    
    nearestIdx1 = knnsearch(pt0, pt1, 'NSMethod', 'kdtree');
    try
        samePtFlag1 = VecNorm(pt1 - pt0(nearestIdx1, :), 2) <= 0.2.*scal; 0.4; % 0.2; %obj.featPtManager.configParam.radius_thresh_for_combine; %
    catch
        sghkbj = 1;
    end
    nearestIdx2 = knnsearch(pt1, pt0, 'NSMethod', 'kdtree');
    try
        samePtFlag2 = VecNorm(pt0 - pt1(nearestIdx2, :), 2) <= 0.2.*scal; 0.4;  % 0.2;% obj.featPtManager.configParam.radius_thresh_for_combine; %
    catch
        dfth = 1;
    end
    newAddedPtInit = pt1Init(~samePtFlag1, :);
    newAddedPt = pt1(~samePtFlag1, :);
    if 0
        figure,imshow(imgCur);hold on;plot(pt1(:,1), pt1(:,2), 'xr');plot(pt0(:,1), pt0(:,2),'.g');
    end
    
    if 0
        figure(7),clf; imshow(imgCur);hold on;plot(pt1(samePtFlag1,1), pt1(samePtFlag1,2), 'xr');plot(pt0(samePtFlag2,1), pt0(samePtFlag2,2),'.g');plot(newAddedPt(:,1), newAddedPt(:,2),'sb');drawnow;
    end
    
    
    if ~obj.switchDepth
        depthMapPrv = depthPrv; %depthMapKey_2; % keyFrameDepth;
        depthMapCur = depthPrv;
    else
        depthMapPrv = depthPrv; %depthMapKeyGT_2;
        depthMapCur = depthPrv;
    end
    
    %     depthMapPrvGT = obj.prvDepthGT;
    %     depthMapCurGT = obj.depthGT;
    
    
    %     inValid___ = sub2ind(size(depthMapPrv), round(newAddedPt(:,2)), round(newAddedPt(:,1)));
    inValid___ = sub2ind(size(depthMapPrv), round(newAddedPtInit(:,2)), round(newAddedPtInit(:,1)));
    %     inValid___ = inValid___(depthMapPrv(inValid___)>0);
    inValid___Cur = sub2ind(size(depthMapCur), round(newAddedPt(:,2)), round(newAddedPt(:,1)));
    %     inValid___Cur = inValid___Cur(depthMapCur(inValid___Cur)>0);
    inValid___Cur0 = sub2ind(size(depthMapCur), round(pt0(:,2)), round(pt0(:,1)));
    %     inValid___Cur0 = inValid___Cur0(depthMapCur(inValid___Cur0)>0);
    
    newAddedZ = depthMapPrv(inValid___);
    newAddedZ_Cur = depthMapCur(inValid___Cur);
    newAddedZ_Cur0 = depthMapCur(inValid___Cur0);
    
    newAddedZGT = depthMapPrvGT(inValid___);
    newAddedZGT_Cur = depthMapCurGT(inValid___Cur);
    newAddedZGT_Cur0 = depthMapCurGT(inValid___Cur0);
    
    obj.traceManager.X = [obj.traceManager.X; -1.*ones(length(newAddedZ), prvFrmNum)];
    obj.traceManager.Y = [obj.traceManager.Y; -1.*ones(length(newAddedZ), prvFrmNum)];
    obj.traceManager.Z = [obj.traceManager.Z; -1.*ones(length(newAddedZ), prvFrmNum)];
    try
        obj.traceManager.ZGT = [obj.traceManager.ZGT; -1.*ones(length(newAddedZGT), prvFrmNum)];
    catch
        sgbk = 1;
    end
    newPtNumAll = size(obj.traceManager.X, 1);
    addedPtNum = length(newAddedZ);
    obj.traceManager.X(1 : newPtNumAll - addedPtNum, prvFrmNum + 1) = -1;
    obj.traceManager.Y(1 : newPtNumAll - addedPtNum, prvFrmNum + 1) = -1;
    obj.traceManager.Z(1 : newPtNumAll - addedPtNum, prvFrmNum + 1) = -1;
    obj.traceManager.ZGT(1 : newPtNumAll - addedPtNum, prvFrmNum + 1) = -1;
    
    obj.traceManager.X(curValidPt(inTrackFlag000), prvFrmNum + 1) = pt0(:,1);
    obj.traceManager.Y(curValidPt(inTrackFlag000), prvFrmNum + 1) = pt0(:,2);
    %     obj.traceManager.Z(curValidPt(inTrackFlag000), prvFrmNum + 1) = obj.traceManager.Z(curValidPt(inTrackFlag000), prvFrmNum );
    obj.traceManager.Z(curValidPt(inTrackFlag000), prvFrmNum + 1) = newAddedZ_Cur0; % obj.traceManager.Z(curValidPt(inTrackFlag000), prvFrmNum );
    obj.traceManager.ZGT(curValidPt(inTrackFlag000), prvFrmNum + 1) = newAddedZGT_Cur0; % obj.traceManager.Z(curValidPt(inTrackFlag000), prvFrmNum );
    
    obj.traceManager.X(newPtNumAll - addedPtNum + 1 : newPtNumAll, prvFrmNum ) = newAddedPtInit(:,1);
    obj.traceManager.Y(newPtNumAll - addedPtNum + 1 : newPtNumAll, prvFrmNum ) = newAddedPtInit(:,2);
    obj.traceManager.Z(newPtNumAll - addedPtNum + 1 : newPtNumAll, prvFrmNum ) = newAddedZ;
    obj.traceManager.ZGT(newPtNumAll - addedPtNum + 1 : newPtNumAll, prvFrmNum ) = newAddedZGT;
    
    obj.traceManager.X(newPtNumAll - addedPtNum + 1 : newPtNumAll, prvFrmNum + 1) = newAddedPt(:,1);
    obj.traceManager.Y(newPtNumAll - addedPtNum + 1 : newPtNumAll, prvFrmNum + 1) = newAddedPt(:,2);
    obj.traceManager.Z(newPtNumAll - addedPtNum + 1 : newPtNumAll, prvFrmNum + 1) = newAddedZ_Cur;
    obj.traceManager.ZGT(newPtNumAll - addedPtNum + 1 : newPtNumAll, prvFrmNum + 1) = newAddedZGT_Cur;
    kvjk = 1;
    
    
    
    
    
    sklav = 1;
end

depthMapPrv = depthPrv;
depthMapCur = depthCur;
depthMapPrvGT = depthPrv;

keyFrmLen = 2;

id_0 = find(obj.traceManager.X(:,prvFrmNum + 0)  > 0 & obj.traceManager.Y(:,prvFrmNum + 0)  > 0 & obj.traceManager.Z(:,prvFrmNum + 0)  > 0);
id = find(obj.traceManager.X(:,prvFrmNum + 1)  > 0 & obj.traceManager.Y(:,prvFrmNum + 1)  > 0 & obj.traceManager.Z(:,prvFrmNum + 1)  > 0);
prvPt = [obj.traceManager.X(id,prvFrmNum)  obj.traceManager.Y(id,prvFrmNum)];
prvPt_0 = [obj.traceManager.X(id_0,prvFrmNum)  obj.traceManager.Y(id_0,prvFrmNum)];
curPt = [obj.traceManager.X(id,prvFrmNum+1)   obj.traceManager.Y(id,prvFrmNum+1)];

valid___ = sub2ind(size(depthMapPrv), round(prvPt(:,2)), round(prvPt(:,1)));
% valid___ = valid___(depthMapPrv(valid___)>0);

valid___0 = sub2ind(size(depthMapPrv), round(prvPt_0(:,2)), round(prvPt_0(:,1)));
% valid___0 = valid___0(depthMapPrv(valid___0)>0);
zList = depthMapPrv(valid___);
zList_0 = depthMapPrv(valid___0);

zListGT = depthMapPrvGT(valid___);
zListGT_0 = depthMapPrvGT(valid___0);

err = (zList_0 - obj.traceManager.Z(id_0,prvFrmNum));
errGT = (zListGT_0 - obj.traceManager.ZGT(id_0,prvFrmNum));

if 1
    % %     pnp_ang_est_max_margin = [deg2rad([-1 1]) obj.pureRotationAngPredictor.configParam.pure_rot_reproject_outlier_threshold];
    pnp_ang_est_max_margin = [deg2rad([-1 1]) 0.5];
else
    pnp_ang_est_max_margin = [deg2rad([-1 1]) 10];
end
if 0
    [p2cBodyRotAng_2,Err_2,inId] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin,intrMat,prvPt,curPt,zList, id,true(length(id),1),b2c,double(p2cRef));
else
    %% 20200415
    roundingPrecision = 0.00000001;
    p2cRef__ = deg2rad(round(rad2deg(p2cRef)./roundingPrecision).*roundingPrecision);
    [~,~,inId] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin,intrMat,prvPt,curPt,zList, id,true(length(id),1),b2c,double(p2cRef));
    pnp_ang_est_max_margin_2 = [deg2rad([-0.2 0.2]) 0.5];
    
    if 1 % ShiftLK && newShift
        pnp_ang_est_max_margin_2 = [deg2rad([-0.05 0.05]) 0.5];
    end
    if 1 % ShiftLK && newShift
        if obj.switchDepth
            [p2cBodyRotAng_2,Err_2,inId___2, reProjErr___2] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin_2,intrMat,prvPt,curPt,zListGT, id,true(length(id),1),b2c,double(p2cRef__),0.00001/2);
        else
            [p2cBodyRotAng_2,Err_2,inId___2, reProjErr___2] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin_2,intrMat,prvPt,curPt,zList, id,true(length(id),1),b2c,double(p2cRef__),0.00001/2);
        end
    else
        if obj.switchDepth
            [p2cBodyRotAng_2,Err_2,inId___2, reProjErr___2] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin_2,intrMat,prvPt,curPt,zListGT, id,true(length(id),1),b2c,double(p2cRef__));
        else
            [p2cBodyRotAng_2,Err_2,inId___2, reProjErr___2] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin_2,intrMat,prvPt,curPt,zList, id,true(length(id),1),b2c,double(p2cRef__));
        end
    end
end

if isempty(inId)
    pnp_ang_est_max_margin = [deg2rad([-0.5 0.5]) 100];
    if obj.switchDepth
        [p2cBodyRotAng_2,Err_2,inId] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin,intrMat,prvPt,curPt,zListGT, id,true(length(id),1),b2c,double(p2cRef));
    else
        [p2cBodyRotAng_2,Err_2,inId] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin,intrMat,prvPt,curPt,zList, id,true(length(id),1),b2c,double(p2cRef));
    end
end

if 1
    ptPrv = [obj.traceManager.X(inId, end-1) obj.traceManager.Y(inId, end-1)];
    ptCur = [obj.traceManager.X(inId, end) obj.traceManager.Y(inId, end)];
    
    ptPrv_norm = inv(intrMat)*pextend(ptPrv');
    ptCur_norm = inv(intrMat)*pextend(ptCur');
    
    if ~obj.switchDepth
        depthMapPrv__onlyP2 = depthPrv;
    else
        depthMapPrv__onlyP2 = depthPrv;
    end
    ind_prv_pt_onlyP2 = sub2ind(size(depthMapPrv__onlyP2), round(ptPrv(:,2)), round(ptPrv(:,1)));
    prevFeatPtList_zz_onlyP2 = depthMapPrv__onlyP2(ind_prv_pt_onlyP2);
    [XYZ_onlyP2_] = GetXYZFromDepth(intrMat, ptPrv,prevFeatPtList_zz_onlyP2);
    
    [angle_resultP2C,depthCurInPrv] = VisualLocalizer.AnglePredict_Prob(ptPrv_norm,ptCur_norm,r_cam,tx, ty, tz);
    [angle_resultP2C1,~] = VisualLocalizer.AnglePredict(ptPrv_norm,ptCur_norm, tx, ty, tz);
    % [angle_result1to2,depth2In1] = VisualLocalizer.AnglePredict_Prob(metricCcs1,PtIn2_norm,r_cam,tx, ty, tz);
    angDiffer = angle_resultP2C - angle_resultP2C1';
    if 1
        validAng = find(abs(angDiffer) < 0.01);
    else
        validAng = find(abs(prevFeatPtList_zz_onlyP2 - depthCurInPrv) < 5000 & abs(depthCurInPrv) > 0);
    end
    if length(thetaList) == 2
        validAng = find(abs(angDiffer) < 0.01);
        
    end
    inId0 = inId;
    if ReplaceOnlyP2
        %         inId0 = inId;
        inId = inId(validAng);
        depthCurInPrvUse = depthCurInPrv(validAng);
        pix_z = [inId ptPrv(validAng,:) ptCur(validAng,:) angle_resultP2C(validAng) depthCurInPrvUse]; %featId  prv cur ang prvZ
        p2cBodyRotAng_2 = deg2rad(mean(angle_resultP2C(validAng)));
    else
        validAng = [1:length(inId0)]';
        pix_z = [];
    end
end
% [XYZ] = GetXYZFromDepth(intrMat, Pix,depthCurInPrv);

metricPrevPtCcs = intrMat\HomoCoord(ptPrv',1);
metricPrevPtCcs = normc(metricPrevPtCcs);
scaleAll = depthCurInPrv./metricPrevPtCcs(3,:)';
XYZ_onlyP2 = [repmat(scaleAll',3,1).*metricPrevPtCcs];

if 0
    
    %     if ~obj.switchDepth
    %         depthMapPrv__onlyP2 = obj.prvDepthVisual;
    %     else
    %         depthMapPrv__onlyP2 = obj.prvDepthGT;
    %     end
    %     ind_prv_pt_onlyP2 = sub2ind(size(depthMapPrv__onlyP2), round(ptPrv(:,2)), round(ptPrv(:,1)));
    %     prevFeatPtList_zz_onlyP2 = depthMapPrv__onlyP2(ind_prv_pt_onlyP2);
    %     [XYZ_onlyP2_] = GetXYZFromDepth(intrMat, ptPrv,prevFeatPtList_zz_onlyP2);
    
    figure,plot([depthCurInPrv prevFeatPtList_zz_onlyP2]);
    for t = 1 : length(validAng)
        pixGT_(t,:) = VisualLocalizer.GetGtTrace2(b2cPmat, deg2rad(angle_resultP2C(validAng(t))), ptPrv(validAng(t),:),depthCurInPrv(validAng(t)),intrMat);
        
    end
    er_onlyP2 = pixGT_ - ptCur(validAng,:);
    figure,plot(er_onlyP2);
end


% % if 0
% %     [p2cBodyRotAng_2,~,~] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin,intrMat,ptPrv,ptCur,depthCurInPrv, inId,true(length(inId),1),b2c,double(p2cRef));
% % end

[tt, pp] = hist(angle_resultP2C,2000);
[~,idMax] = max(tt);
angMax = pp(idMax);
idGoodFeat = find(angle_resultP2C > angMax - 0.025 & angle_resultP2C < angMax + 0.025);
if 0
    [p2cBodyRotAng_2,~,~] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin,intrMat,ptPrv(idGoodFeat,:),ptCur(idGoodFeat,:),depthCurInPrv(idGoodFeat,:), inId(idGoodFeat),true(length(idGoodFeat),1),b2c,double(p2cRef));
end
if 0
    figure,plot(depthCurInPrv(idGoodFeat,:))
end
% obj.accumP2CPNP2 = [obj.accumP2CPNP2; obj.accumP2CPNP2(end) + robotPoseWcs(3) - obj.poseWcsList(end,3)];
obj.accumP2CPNP2 = [obj.accumP2CPNP2; obj.accumP2CPNP2(end) + p2cBodyRotAng_2];


diffVec = setdiff([1:size(obj.traceManager.X,1)]', inId);

obj.traceManager.X(diffVec, prvFrmNum + 1) = -1;
obj.traceManager.Y(diffVec, prvFrmNum + 1) = -1;
obj.traceManager.Z(diffVec, prvFrmNum + 1) = -1;
obj.traceManager.ZGT(diffVec, prvFrmNum + 1) = -1;


inFlag = ismember(id, inId);

if 0
    figure,subplot(2,2,1), showMatchedFeatures(imgPrv, imgCur, prvPt, curPt);subplot(2,2,2), showMatchedFeatures(imgPrv, imgCur, prvPt(inFlag,:), curPt(inFlag,:));
    subplot(2,2,3);imshow(imgPrv);hold on;plot(prvPt(inFlag,1),prvPt(inFlag,2),'.r');plot(prvPt(~inFlag,1),prvPt(~inFlag,2),'.g');
    subplot(2,2,4);imshow(imgCur);hold on;plot(curPt(inFlag,1),curPt(inFlag,2),'.r');plot(curPt(~inFlag,1),curPt(~inFlag,2),'.g');
end








nearestKeyX = obj.traceManager.X(:, prvFrmNum + 1 - keyFrmLen + 1:end);
nearestKeyY = obj.traceManager.Y(:, prvFrmNum + 1 - keyFrmLen + 1:end);
nearestKeyZ = obj.traceManager.Z(:, prvFrmNum + 1 - keyFrmLen + 1:end);
featIdKey = find(nearestKeyX(:,end) > 0 & nearestKeyY(:,end) > 0 & nearestKeyZ(:,end) > 0 & nearestKeyX(:,1) > 0 & nearestKeyY(:,1) > 0 & nearestKeyZ(:,1) > 0);


depthMapKeyGT = depthPrv;
inValid___KeyGT = sub2ind(size(depthMapKeyGT), round(nearestKeyY(featIdKey,1)), round(nearestKeyX(featIdKey,1)));
% inValid___KeyGT = inValid___KeyGT(depthMapKeyGT(inValid___KeyGT)>0);
nearestKeyZGT = depthMapKeyGT(inValid___KeyGT);

b2cPmat = GetPctBody2Cam(obj.coordSysAligner, 1);
if ~isempty(featIdKey)
    %     b2cPmat = GetPctBody2Cam(obj.coordSysAligner, 1);
    [pixGT] = VisualLocalizer.GetGtTrace2(b2cPmat, k2cRef, [nearestKeyX(featIdKey, 1) nearestKeyY(featIdKey, 1)],nearestKeyZGT(:),intrMat);
    
    
    pixelKey = [nearestKeyX(featIdKey, 1) nearestKeyY(featIdKey, 1)];
    
    if p2cRef > 0
        idNewPixel = find(pixelKey(:,1) < 11111160);
    else
        idNewPixel = find(pixelKey(:,1) > -260);
    end
    
    [~, er] = NormalizeVector(pixelKey - round(pixelKey));
    
    idNewPixel = find(er == 0);
    
    
    [pixGT2] = VisualLocalizer.GetGtTrace2(b2cPmat, p2cRef, [prvPt(inFlag,:)],zListGT(inFlag,:),intrMat);
    
    errTracking = [nearestKeyX(featIdKey, end) nearestKeyY(featIdKey, end)] - pixGT;
    
    pt2dKey_check = [[nearestKeyX(featIdKey, 1) nearestKeyY(featIdKey, 1)],nearestKeyZGT(:)];
    pt2dCur_check = [nearestKeyX(featIdKey, end) nearestKeyY(featIdKey, end)];
    pt2dGT_check = pixGT;
    pt2d_check = [pt2dKey_check pt2dCur_check pt2dGT_check];
    
    errTracking2 = curPt(inFlag,:) - pixGT2;
    
    %     obj.p2cTrackingErrWholeStack = [obj.p2cTrackingErrWholeStack; mean(errTracking2,1)];
    
    [~, er2] = NormalizeVector(prvPt(inFlag,:) - round(prvPt(inFlag,:)));
    
    idNewPixel2 = find(er2 == 0);
    
    
    
    
    
    
    depthStereo = zList(inFlag);
    dispStereo = (intrMat(1,1).*norm(baseline)./depthStereo(:,1)) - (princpPtR(1) - princpPtL(1));
    depthGt = zListGT(inFlag);
    dispGt  = (intrMat(1,1).*norm(baseline)./depthGt(:,1)) - (princpPtR(1) - princpPtL(1));
    [dispa,dispb] = hist(dispGt - dispStereo, 1000);
    dispErrExp = dot(dispb, (dispa./sum(dispa)));
    
    
    
    if 0
        if keyFrmLen == 2
            [~,er] = NormalizeVector(errTracking - errTracking2);
            figure,plot(er);
        end
    end
    
    jfhvkgh = 1;
    
    
    if keyFrmLen == 2
        obj.trackingErrAccum2 = [];
        if 0
            figure(6);plot(errTracking2(:,1), errTracking2(:,2),'+'); % title('lk tracking - gt (p2c)');
            axis equal;title(sprintf('frame: %d',imgInd));drawnow;
        end
    end
    
    
    
    
    % subplot(1,2,1);cla;imshow(obj.currImgL);hold on;plot(pt1(samePtFlag1,1), pt1(samePtFlag1,2), 'xr');plot(pt0(samePtFlag2,1), pt0(samePtFlag2,2),'.g');plot(newAddedPt(:,1), newAddedPt(:,2),'sb');drawnow;
%     subplot(2,2,1);hold on;plot(errTracking(:,1), errTracking(:,2),'+');title('lk tracking - gt (k2c)');axis equal;
    % subplot(1,2,2);cla;hold on;plot(mean(errTracking(:,1)), mean(errTracking(:,2)),'o');title('lk tracking - gt'); axis equal;
    %     subplot(2,2,2);cla;hold on;plot((obj.trackingErrAccum2(:,1)), (obj.trackingErrAccum2(:,2)),'-o');title('lk tracking - gt (k2c)');% axis equal;
    %     for j = 1 : size(obj.trackingErrAccum2, 1)
    %         text(double(obj.trackingErrAccum2(j,1)),double(obj.trackingErrAccum2(j,2)),num2str(j), 'Color',[1 0 0],'FontSize',15);  %,'FontWeight','bold');
    %     end
%     subplot(2,2,3);hold on;plot(errTracking2(:,1), errTracking2(:,2),'+');title('lk tracking - gt (p2c)');axis equal;
    % subplot(1,2,2);cla;hold on;plot(mean(errTracking(:,1)), mean(errTracking(:,2)),'o');title('lk tracking - gt'); axis equal;
    %     subplot(2,2,4);cla;hold on;plot((obj.trackingErrAccum2(:,3)), (obj.trackingErrAccum2(:,4)),'-o');title('lk tracking - gt (p2c)');% axis equal;
    %     for j = 1 : size(obj.trackingErrAccum2, 1)
    %         text(double(obj.trackingErrAccum2(j,3)),double(obj.trackingErrAccum2(j,4)),num2str(j), 'Color',[1 0 0],'FontSize',15);  %,'FontWeight','bold');
    %     end
    
    
    
else
    pt2d_check = [];
    dispErrExp = [];
    ljkblkjb = 1;
end

% errTracking2 = curPt(inFlag,:) - pixGT2;
cur_lk_gt = [ curPt(inFlag,:) pixGT2;];


obj.prvDepthGT = depthPrv;
obj.prvDepthVisual = depthPrv;
obj.depthVisual = depthCur;
obj.depthGT = depthCur;

[LocalTraceList, unValidFeatId, validFeatId, traceLenList, dataTmp, dispErrMat, cur_id_lk_gt2, pixGT_temp_stack2] = PlotMultiTrace(obj, 8, imgCur, obj.traceManager.X, obj.traceManager.Y, obj.traceManager.Z, obj.traceManager.ZGT, intrMat, princpPtL,princpPtR, baseline, obj.configParam, obj.colorMat, b2cPmat, thetaList, cur_lk_gt, pix_z);

obj.traceManager.X(unValidFeatId,end) = -1;
obj.traceManager.Y(unValidFeatId,end) = -1;
obj.traceManager.Z(unValidFeatId,end) = -1;
obj.traceManager.ZGT(unValidFeatId,end) = -1;

if length(thetaList) > 2
    obj.traceManager.XLK(unValidFeatId,end) = -1;
    obj.traceManager.YLK(unValidFeatId,end) = -1;
    obj.traceManager.ZLK(unValidFeatId,end) = -1;
    obj.traceManager.ZGTLK(unValidFeatId,end) = -1;
else
    obj.traceManager.XLK(unValidFeatId,end) = -1;
    obj.traceManager.YLK(unValidFeatId,end) = -1;
    obj.traceManager.ZLK(unValidFeatId,end) = -1;
    obj.traceManager.ZGTLK(unValidFeatId,end) = -1;
end

obj.colorMat = [obj.colorMat(2:end,:); obj.colorMat(1,:)];

if isempty(obj.featBatch)
    obj.featBatch = [obj.featBatch; [1:size(obj.traceManager.X,1)]'];
    
else
    lastFeatBatch = cell2mat(obj.featBatch);
    obj.featBatch = [obj.featBatch; setdiff([1:size(obj.traceManager.X,1)]',lastFeatBatch)];
    
end




traceBatchList = 0;


for k = 1 : length(LocalTraceList)
    
    tempId = LocalTraceList{k}.featId;
    for hb = 1 : length(obj.featBatch)
        
        if sum(ismember(tempId, obj.featBatch{hb})) > 0
            traceBatchList(k,1) = hb;
            break;
        end
        
    end
end

for kk = 1 : length(traceBatchList)
    obj.traceInfoMat{traceBatchList(kk),1} = traceBatchList(kk);
    obj.traceInfoMat{traceBatchList(kk),2} = LocalTraceList{kk}.featId;
    
    try (obj.traceInfoMat{traceBatchList(kk),3});
        obj.traceInfoMat{traceBatchList(kk),3} = [obj.traceInfoMat{traceBatchList(kk),3}; length(obj.accumP2CRef)];
        
    catch
        %          obj.traceInfoMat{traceBatchList(kk),3} = [];
        obj.traceInfoMat{traceBatchList(kk),3} = [length(obj.accumP2CRef)];
        
    end
end
if length(obj.accumP2CRef) == 2
    TraceBatchListStackBefore = [];
else
    TraceBatchListStackBefore = obj.TraceBatchListStack{end};
end

FinishedTrace = setdiff(TraceBatchListStackBefore, traceBatchList);
if ~isempty(FinishedTrace)
    for oo = 1 : length(FinishedTrace)
        obj.dispErrExpStack3{FinishedTrace(oo), 1} = FinishedTrace(oo);
        obj.dispErrExpStack3{FinishedTrace(oo), 8} = 1;
    end
end
obj.TraceBatchListStack = [obj.TraceBatchListStack; traceBatchList];


obj.traceInfoMat{1,4} = cur_id_lk_gt2;
obj.traceInfoMat{1,5} = pixGT_temp_stack2;
obj.traceInfoMat{1,6} = obj.accumP2CPNP2;
obj.traceInfoMat{1,7} = obj.accumP2CPNP2_2;



traceInfoMat1 = obj.traceInfoMat;
traceManager1 = obj.traceManager;
traceManager1.X(traceManager1.X == -1) = 0;
traceManager1.Y(traceManager1.Y == -1) = 0;
try
    shiftX = traceManager1.X - traceManager1.XLK;
catch
    rgihu = 1;
end
try
    traceNum1 = 15;
    
    xErrMat = ArrangeData(traceManager1, traceInfoMat1);
    
%     gsjkh;
%     [x_err1, x_err1_upper, x_err1_lower] = CalcEnvelope(traceManager1.XShift, traceNum1);
    [x_err1, x_err1_upper, x_err1_lower] = CalcEnvelope(xErrMat, traceNum1);
    mean_x_err = [[0;mean(x_err1,2)] [mean_x_err(:,2);0]];
    if 0
        figure,plot(rad2deg(thetaList(1:end-1)), x_err1);hold on;plot(rad2deg(thetaList(1:end)), 2.*mean_x_err(:,2),'-g');
    end
    
catch
    
    asgfjb = 1;
end

if imgInd >= length(thetaListGT) - 2
    CalibB2C(thetaList, obj.traceInfoMat, obj.traceManager, 1);
    try
        save(fullfile(probPath,'Distri.mat'), 'Distri');
    catch
        srgkuh = 1;
    end
%     er = CalibB2C(thetaList, obj.traceInfoMat, obj.traceManager, 0);subplot(2,1,1);hold on;plot(rad2deg(thetaList(1:end)), 2.*mean_x_err(:,2),'-g');subplot(2,1,2);hold on;plot(rad2deg(thetaList(1:end)), 2.*mean_x_err(:,2),'-g');
% er = CalibB2C(thetaList, obj.traceInfoMat, obj.traceManager, 0);subplot(2,1,1);hold on;plot(rad2deg(thetaList(1:end)), 2.*mean_x_err(:,2),'-g');subplot(2,1,2);cla;hold on;plot(rad2deg(thetaList(1:end-1)), mean(x_err1,2),'-xr');plot(rad2deg(thetaList(1:end)), 2.*mean_x_err(:,2),'-g');plot(rad2deg(thetaList(1:end-1)),10.*er(:,1:2),'-');plot(rad2deg(thetaList(1:end-1)), err_x(2:end), '-c');

er = CalibB2C(thetaList, obj.traceInfoMat, obj.traceManager, 0);subplot(2,1,1);hold on;plot(rad2deg(thetaList(1:end)), 2.*mean_x_err(:,2),'-g');subplot(2,1,2);cla;hold on;plot(rad2deg(thetaList(1:end-1)), mean(x_err1,2),'-xr');plot(rad2deg(thetaList(1:end)), 2.*mean_x_err(:,2),'-g');plot(rad2deg(thetaList(1:end-1)),10.*er(:,1:2),'-');plot(rad2deg(thetaList(1:end-1)), err_x(2:end,1), '-c');
figure,plot(rad2deg(obj.accumP2CPNP2_2(:,[1 4]) - thetaListGT(1:imgInd+1)));
err = [0 0; er];alpha = 7;rou;figure,subplot(1,2,1);plot(diff(err_x(:,1)));grid on;hold on;plot(diff(alpha.*err(:,2)));subplot(1,2,2);plot(diff(err_x(:,1)) - diff(alpha.*err(:,2)));grid on;title(num2str(mean(diff(err_x(:,1)) - diff(alpha.*err(:,2)))));
err = [0 0; er];
xerr = [0;mean(x_err1(1:end,:),2)];err = [0 0; er];alpha = 7;rou;figure,subplot(1,2,1);plot(diff(xerr(:,1)));grid on;hold on;plot(diff(alpha.*err(:,2)));title(sprintf('p2cX       %d*p2cTheta',alpha));subplot(1,2,2);plot(diff(xerr(:,1)) - diff(alpha.*err(:,2)));grid on;title(sprintf('p2cX - %d*p2cTheta = %04f\nRou: %d\nalpha: %d\nx~~: %d\nuseEx %d',alpha, mean(diff(xerr(:,1)) - diff(alpha.*err(:,2))), Rou, rou,UseDoubleShift,UseEx));

figure,subplot(1,2,1);plot(err_x(3:end,2));hold on;plot(diff(mean(x_err1,2)));subplot(1,2,2);plot(diff(err_x(2:end,1)));hold on;plot(diff(mean(x_err1,2)))
if 0
    traceInfoMat1 = obj.traceInfoMat;
    traceManager1 = obj.traceManager;
    %     combData
    %     saveas(gcf,fullfile(probPath,sprintf(strcat('err_',probPath(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('err_',probPath(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1 + FigBase)));
    tempInfo = dir(fullfile(probPath, 'combData*.mat'));
    save(fullfile(probPath,sprintf('combData_%03d_%02d_%04d.mat',length(tempInfo) + 1, double(ShiftByCoord), imgInd)), 'thetaList','traceInfoMat1', 'traceManager1','-v7.3');
end
end

jhghk = 1;

end
function [pixKey, zList] = DetectValidFeature(imgPrv, depthMap, intrMat, thr, marg)

if ndims(imgPrv) == 3
    imgPrv = rgb2gray(imgPrv);
end
ptIcsKey = detectFASTFeatures(imgPrv, 'MinQuality', thr, 'MinContrast', thr);
pixKey0 = ptIcsKey.Location;
mask = ~isinf(depthMap) & depthMap > 0 & ~isnan(depthMap);
se = strel('square',[7]);
mask_ = imerode(mask,se);
ind0 = find(mask_ > 0);
[y, x] = ind2sub(size(depthMap), ind0);
[xyzKey_all] = GetXYZFromDepth(intrMat, [x y],depthMap(ind0));


ind1 = sub2ind(size(depthMap), round(pixKey0(:,2)), round(pixKey0(:,1)));
ind = intersect(ind0, ind1);
pixKey = pixKey0(ismember(ind1, ind), :);

inBndFlag = pixKey(:, 1) >= marg + 1 & ...
    pixKey(:, 1) <= Cols(imgPrv) - marg & ...
    pixKey(:, 2) >= marg + 1 & ...
    pixKey(:, 2) <= Rows(imgPrv) - marg;

pixKey = pixKey(inBndFlag,:);
ind_check = sub2ind(size(depthMap), round(pixKey(:,2)), round(pixKey(:,1)));
zList = depthMap(ind_check);
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
function vldPtCur = FindValidDepth(predPtList000, depthMapCurGT, marg)

vldPtCur = nan(size(predPtList000,1),1);
vldPrTrace = find(predPtList000(:,1) > marg & predPtList000(:,2) > marg & predPtList000(:,1) < size(depthMapCurGT,2) - marg & predPtList000(:,2) < size(depthMapCurGT,1) - marg);

vldPtCur_ = sub2ind(size(depthMapCurGT), round(predPtList000(vldPrTrace,2)), round(predPtList000(vldPrTrace,1)));
vldPtCur_depth = depthMapCurGT(vldPtCur_);

vldPtCur(vldPrTrace) = vldPtCur_depth;


end
function [LocalTraceList, unValidFeatId, validFeatId, traceLenList, data, dispErrMat, cur_id_lk_gt2, pixGT_temp_stack2] = PlotMultiTrace(obj, figNum, img, localX, localY, localZ, localZGT, intrMat, princpPtL,princpPtR, baseline, ConfigParam, colorMat, b2cPmat, accumP2CRef, cur_lk_gt, pix_z)
global newRender ShiftLK newShift ReplaceOnlyP2 ReplaceOnlyP2Z ShiftByCoord doubleShift mean_x_err...
    rou shiftPixThr doubleShift2 err_x UseDoubleShift Rou UseEx Distri thetaListGT

% newShift = true;

draw = 0;

if 0
    intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,obj.scaleLvl);
    [princpPtL, princpPtR] = Get(obj.camModel, 'PinholePrincpPt', obj.scaleLvl);
    baseline = obj.camModel.transVec1To2;
end


angP2CWhole = obj.accumP2CPNP2(end) - obj.accumP2CPNP2(end-1);

thetaListGTTemp = thetaListGT(1:size(obj.accumP2CPNP2,1));

if 1 % obj.switchDepth
    depthMapPrvGT = obj.prvDepthGT;
    depthMapPrv = obj.prvDepthVisual;
    depthMapCur = obj.depthVisual;
    depthMapCurGT = obj.depthGT;
else
    depthMapPrvGT = obj.prvDepthVisual;
    depthMapCur = obj.depthVisual;
    depthMapCurGT = obj.depthVisual;
    
end
b2cPmat = GetPctBody2Cam(obj.coordSysAligner, 1);
b2c = b2cPmat.transformMat;
T_B2C = b2c;


r_cam = sqrt(T_B2C(1,4)^2+T_B2C(2,4)^2+T_B2C(3,4)^2);
tx = T_B2C(1,4)/r_cam;
ty = T_B2C(2,4)/r_cam;
tz = T_B2C(3,4)/r_cam;
% ConfigParam = obj.configParam;
disparityRng = [-ConfigParam.disparity_error : ConfigParam.disparity_sample_step : ConfigParam.disparity_error];
if draw
    figure(figNum); imshow(img);hold on;
end
% localX = obj.traceManager.X; localY = obj.traceManager.Y;


featId = find(localX(:,end) > 0 & localY(:,end) > 0);

assert(length(featId) - size(cur_lk_gt,1) == 0);



wholeErrCur = (cur_lk_gt(:,1:2) - cur_lk_gt(:,3:4));
wholeErrCurMean = mean(cur_lk_gt(:,1:2) - cur_lk_gt(:,3:4));

inTraceMat = localX(featId,:) > 0;

traceNum = sum(inTraceMat');
traceNumUniq = sort(unique(traceNum),'descend');
frmNum = size(localX, 2);
if 0
    LocalTraceList = cell(length(traceNumUniq),1);
end
cnt = 1;
unValidFeatId = []; validFeatId = []; traceLenList = [];

pnp_ang_est_max_margin = [deg2rad([-1 1]) 0.7];
pnp_ang_est_max_margin_large = [deg2rad([-6 6]) 0.7];
dispMat = [];
dispMatGT = [];
featSize = [];
dispErrMat = [];
numThr0 = 15;
% if abs(rad2deg(obj.accumP2CRef(2))) < 1.4
if newRender
    numThr = 15 + 1; % 18+1; % 17+1; 17*3+1; 14*3 + 1;
else
    numThr = 16*3+1;  %17*3+1;
end

pixGT_temp_stack = [];
pixGT_temp_stack2 = {};
batchAng = [];
for i = 1 : length(traceNumUniq)
    if traceNumUniq(i) < 2 % || traceNumUniq(i) > 8
        %     if traceNumUniq(i) < 2 || traceNumUniq(i) >= 3
        continue;
    else
        featId22 = find(traceNum == traceNumUniq(i));
        tempZ = localZ(featId(featId22),1:end-1);
        validZ = sum(tempZ' > 0,1)';
        featId2 = featId22(validZ == traceNumUniq(i)-1);
        if length(featId2) < 30  || traceNumUniq(i) > numThr %  19 %15  % 17  % 15 %15 % 200  %130   % 14 %80 %80; % 80;  %50 % 80 %50 %130  %15 % 40
            unValidFeatId = [unValidFeatId;featId(featId2)];
            continue;
        end
        
        %         validFeatId = [validFeatId; featId(featId2)];
        
        localTraceX0 = localX(featId(featId2), (frmNum - traceNumUniq(i) + 1) : end);
        localTraceY0 = localY(featId(featId2), (frmNum - traceNumUniq(i) + 1) : end);
        localTraceZ0 = localZ(featId(featId2), (frmNum - traceNumUniq(i) + 1) : end);
        localTraceZGT0 = localZGT(featId(featId2), (frmNum - traceNumUniq(i) + 1) : end);
        
        keyFrameLen = traceNumUniq(i);
        gtangle = [accumP2CRef(end-(keyFrameLen-1):end) - accumP2CRef(end-(keyFrameLen-1))];
        
        
        inTrackFlag00 = true(size(localTraceX0, 1), 1);
        [pixGT_temp] = VisualLocalizer.GetGtTrace2(b2cPmat, double(gtangle(end)), [localTraceX0(inTrackFlag00,1) localTraceY0(inTrackFlag00,1)],localTraceZGT0(inTrackFlag00,1),intrMat);
        pixGT_temp_stack = [pixGT_temp_stack; pixGT_temp];
        
         
        [k2cBodyRotAng_2,Err_2,inId] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin,intrMat,[localTraceX0(inTrackFlag00,1) localTraceY0(inTrackFlag00,1)],[localTraceX0(inTrackFlag00,end) localTraceY0(inTrackFlag00,end)],localTraceZ0(inTrackFlag00,1), find(inTrackFlag00),true(sum(inTrackFlag00),1),b2c,double(gtangle(end)));
        if 0
            if length(inId) < 30
                [k2cBodyRotAng_2,Err_2,inId] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin_large,intrMat,[localTraceX0(inTrackFlag00,1) localTraceY0(inTrackFlag00,1)],[localTraceX0(inTrackFlag00,end) localTraceY0(inTrackFlag00,end)],localTraceZ0(inTrackFlag00,1), find(inTrackFlag00),true(sum(inTrackFlag00),1),b2c,double(gtangle(end)));
                
            end
        else
            pnp_ang_est_max_margin_large_temp = pnp_ang_est_max_margin_large;
            ct = 1;
            while (length(inId) < 25)
                pnp_ang_est_max_margin_large_temp(1:2) = pnp_ang_est_max_margin_large_temp(1:2) + 5.*[deg2rad([-1 1])];
                [k2cBodyRotAng_2,Err_2,inId] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin_large_temp,intrMat,[localTraceX0(inTrackFlag00,1) localTraceY0(inTrackFlag00,1)],[localTraceX0(inTrackFlag00,end) localTraceY0(inTrackFlag00,end)],localTraceZ0(inTrackFlag00,1), find(inTrackFlag00),true(sum(inTrackFlag00),1),b2c,double(gtangle(end)));
                ct = ct + 1; % 
                if ct > 20
                    break;
                end
            end
            
            
        end
        if 1
            [p2cBodyRotAng_233,~,~] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin,intrMat,[localTraceX0(inTrackFlag00,end-1) localTraceY0(inTrackFlag00,end-1)],[localTraceX0(inTrackFlag00,end) localTraceY0(inTrackFlag00,end)],localTraceZ0(inTrackFlag00,end-1), find(inTrackFlag00),true(sum(inTrackFlag00),1),b2c,double(angP2CWhole(end)));
        else
            p2cBodyRotAng_233 = OnlyP2_ang(intrMat, [localTraceX0(inTrackFlag00,end-1) localTraceY0(inTrackFlag00,end-1)], [localTraceX0(inTrackFlag00,end) localTraceY0(inTrackFlag00,end)], r_cam,tx, ty, tz);
        end
        batchAng = [batchAng; p2cBodyRotAng_233];
        
        outId = setdiff(find(inTrackFlag00), inId);
        
        validFeatId = [validFeatId; featId(featId2(inId))];
        unValidFeatId = [unValidFeatId;featId(featId2(outId))];
        
        
        if 0
            figure,plot(Err_2);hold on;plot(find(ismember(featId(featId2),4709)), Err_2(find(ismember(featId(featId2),4709))),'or');
        end
        
        
        localTraceX = localTraceX0(inId,:);  %localX(featId(featId2), (frmNum - traceNumUniq(i) + 1) : end);
        localTraceY = localTraceY0(inId,:);  %localY(featId(featId2), (frmNum - traceNumUniq(i) + 1) : end);
        localTraceZ = localTraceZ0(inId,:); % localZ(featId(featId2), (frmNum - traceNumUniq(i) + 1) : end);
        localTraceZGT = localTraceZGT0(inId,:);  %localZGT(featId(featId2), (frmNum - traceNumUniq(i) + 1) : end);
        
        [pixGT_temp2] = VisualLocalizer.GetGtTrace2(b2cPmat, double(gtangle(end)), [localTraceX0(inId,1) localTraceY0(inId,1)],localTraceZGT0(inId,1),intrMat);
        pixGT_temp_stack2{cnt,1} = featId(featId2(inId));
        if obj.switchDepth
            pixGT_temp_stack2{cnt,2} = [[[localTraceX0(inId, end) localTraceY0(inId,end)] pixGT_temp2 [localTraceX0(inId, end-1) localTraceY0(inId,end-1)] localTraceZGT0(inId,end-1) [localTraceX0(inId, 1) localTraceY0(inId,1)] localTraceZGT0(inId,1)]];
        else
            pixGT_temp_stack2{cnt,2} = [[[localTraceX0(inId, end) localTraceY0(inId,end)] pixGT_temp2 [localTraceX0(inId, end-1) localTraceY0(inId,end-1)] localTraceZ0(inId,end-1) [localTraceX0(inId, 1) localTraceY0(inId,1)] localTraceZ0(inId,1)]];
        end
        if draw
            plot(localTraceX', localTraceY', '-', 'Color', colorMat(i,:));
        end
        
        %     end
        
        dispList = (intrMat(1,1).*norm(baseline)./localTraceZ(:,1)) - (princpPtR(1) - princpPtL(1));
        dispListAll = (intrMat(1,1).*norm(baseline)./localTraceZ) - (princpPtR(1) - princpPtL(1));
        dispListAllGT = (intrMat(1,1).*norm(baseline)./localTraceZGT) - (princpPtR(1) - princpPtL(1));
        DispRng = dispList + disparityRng;
        ZZVec1 = [];ProbZ = [];
        % %         for jk = 1 : length(featId2)
        for jk = 1 : length(inId)
            [ProbZ(jk,:), ZZVec1(jk,:)] = probDensity(dispList((jk),:), ConfigParam.disparity_sigma,ConfigParam.disparity_beta,ConfigParam.reproj_beta, DispRng(jk,:),ConfigParam.disparity_sample_interval, 'disparity');
        end
        
        ProbZ = ProbZ./repmat(max(ProbZ')',1,size(ProbZ,2));
        
        
        keyFrameLen = size(localTraceX, 2);
        
        k2cRef = accumP2CRef(end) - accumP2CRef(length(accumP2CRef) - keyFrameLen + 0 + 1);
        
        %     b2cPmat = GetPctBody2Cam(obj.coordSysAligner, 1);
        [pixGT] = VisualLocalizer.GetGtTrace2(b2cPmat, k2cRef, [localTraceX(:, 1) localTraceY(:, 1)],localTraceZGT(:, 1),intrMat);
        errTracking = [localTraceX(:, end) localTraceY(:, end)] - pixGT;
        
        
        LocalTrace.ptIcsX = localTraceX;
        LocalTrace.ptIcsY = localTraceY;
        LocalTrace.ptCcsZ = localTraceZ;
        LocalTrace.ptCcsZGT = localTraceZGT;
        LocalTrace.dispList = dispListAll;
        LocalTrace.dispListGT = dispListAllGT;
        LocalTrace.featId = featId(featId2(inId));
        LocalTrace.probZ = ProbZ;
        LocalTrace.DispRng = DispRng;
        LocalTrace.sampleZ = ZZVec1;
        LocalTrace.probP2 = {};
        LocalTrace.transDisp = {};
        LocalTraceList{cnt,1} = LocalTrace;
        traceLenList(cnt,1) = traceNumUniq(i);
        
        featSize = [featSize; size(localTraceX,1)];
        
        dispErrMat = [dispErrMat; dispListAll(:) dispListAllGT(:) (dispListAll(:) - dispListAllGT(:))]; % [stereo gt (stereo - gt)]
        
        cnt = cnt + 1;
    end
    
    
end


if 0
    figure,subplot(1,2,1);plot(pixGT_temp_stack(:,1) - cur_lk_gt(:,3), pixGT_temp_stack(:,2) - cur_lk_gt(:,4), '+r');axis equal;title('gt - gt'); subplot(1,2,2),plot(pixGT_temp_stack(:,1) - cur_lk_gt(:,1), pixGT_temp_stack(:,2) - cur_lk_gt(:,2), '+r');axis equal;title('gt - lk');
end

assert(sum(abs(validFeatId - cell2mat(pixGT_temp_stack2(:,1)))) == 0);

cur_lk_gt2 = cell2mat(pixGT_temp_stack2(:,2)); % curLKold curGT  prvLK prvGTZ keyLK keyGTZ
featIdAll = cell2mat(pixGT_temp_stack2(:,1));

if ReplaceOnlyP2
    pix_z_use = pix_z(ismember(pix_z(:,1), featIdAll), :); %featId  prv cur ang prvZ
else
    pix_z_use = [];
end

pnp_ang_est_max_margin_2 = [deg2rad([-0.05 0.05]) 0.5];
roundingPrecision = 0.00000001;
p2cRef__ = deg2rad(round(rad2deg(accumP2CRef(end) - accumP2CRef(end-1))./roundingPrecision).*roundingPrecision);

[p2cBodyRotAng_2,Err_2,inId___2, reProjErr___2] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin_2,intrMat,cur_lk_gt2(:,5:6),cur_lk_gt2(:,1:2),cur_lk_gt2(:,7), [1:size(cur_lk_gt2,1)]',true(size(cur_lk_gt2,1),1),b2c,double(p2cRef__),0.00001/2);
if ReplaceOnlyP2
    if 0
        ptPrv_norm = inv(intrMat)*pextend(cur_lk_gt2(:,5:6)');
        ptCur_norm = inv(intrMat)*pextend(cur_lk_gt2(:,1:2)');
        [angle_resultP2C,depthCurInPrv] = VisualLocalizer.AnglePredict_Prob(ptPrv_norm,ptCur_norm,r_cam,tx, ty, tz);
        [angle_resultP2C1,~] = VisualLocalizer.AnglePredict(ptPrv_norm,ptCur_norm, tx, ty, tz);
        angDiffer = angle_resultP2C - angle_resultP2C1';
        validAng = find(abs(angDiffer) < 0.01);
    else
        p2cBodyRotAng_2_onlyP2 = OnlyP2_ang(intrMat, cur_lk_gt2(:,5:6), cur_lk_gt2(:,1:2), r_cam,tx, ty, tz);
    end
else
    p2cBodyRotAng_2_onlyP2 = p2cBodyRotAng_2;
end

wholeErrCur2 = (cur_lk_gt2(:,1:2) - cur_lk_gt2(:,3:4));
wholeErrCurMean2 = mean(cur_lk_gt2(:,1:2) - cur_lk_gt2(:,3:4));

LocalTraceList_bak = LocalTraceList;

shiftId = find(mean_x_err(:,2) ~= 0);
if isempty(shiftId)
    pixErr = mean_x_err(end,1);
else
    if shiftId(end) + 1 == size(mean_x_err,1)
         pixErr = mean_x_err(end,1);
    else
        pixErr = mean_x_err(end,1) - mean_x_err(shiftId(end)+ 1,1);
    end
end


  
if ShiftLK
    Er = []; TempCurPt_2 = []; XYZShift = [];angDiff = 0;
    for j = 1 : length(LocalTraceList)
        
        tempTrace = LocalTraceList{j, 1};
        tempTrace_bak = tempTrace;
        tempFeatId = pixGT_temp_stack2{j,1};
        temp_cur_lk_gt = pixGT_temp_stack2{j, 2};
        tempCurPt = [tempTrace.ptIcsX(:,end) tempTrace.ptIcsY(:,end)];
        if 0
            er = temp_cur_lk_gt(:,1:2) - tempCurPt;
        end
        p2cBodyRotAng = 0;
        
%         if doubleShift
% %            angBias = 1.08*(-angP2CWhole + batchAng(j,1));
%            angBias = 0.9*(-angP2CWhole + batchAng(j,1)); 
%         else
%             angBias = 0;
%         end
% % % % %         shiftId = find(mean_x_err(:,2) ~= 0);
% % % % %         if isempty(shiftId)
% % % % %             pixErr = mean_x_err(end,1);
% % % % %         else
% % % % %             pixErr = mean_x_err(end,1) - mean_x_err(shiftId(end)+ 1,1);
% % % % %         end
        
%         if abs(pixErr) > shiftPixThr
        if abs(mean_x_err(end,1)) > shiftPixThr
            if 1
                angBias = -deg2rad(mean_x_err(end,1)/rou);
                %             mean_x_err(end,2) = 1;
                mean_x_err(end,2) = rou*rad2deg(angBias);
            else
                 angBias = -deg2rad(pixErr/rou);
                %             mean_x_err(end,2) = 1;
                mean_x_err(end,2) = rou*rad2deg(angBias);
                
            end
            
        end
        
        if doubleShift
            %            angBias = 1.08*(-angP2CWhole + batchAng(j,1));
            angBias = 0.9*(-angP2CWhole + batchAng(j,1));
        else
            angBias = 0;
        end
        
        if ~newShift
            tempTrace.ptIcsX(:,end) = temp_cur_lk_gt(:,3) + wholeErrCurMean2(:,1);
            tempTrace.ptIcsY(:,end) = temp_cur_lk_gt(:,4) + wholeErrCurMean2(:,2);
        else
%             p2cBodyRotAng = 0;
            if 1
                if ~ReplaceOnlyP2 || length(accumP2CRef) == 2
                    [tempCurPt_2_newShift] = VisualLocalizer.GetGtTrace2(b2cPmat, double(angP2CWhole + angBias), [temp_cur_lk_gt(:,5:6)],temp_cur_lk_gt(:,7),intrMat);
                else
                    if ReplaceOnlyP2Z
                        if ~ShiftByCoord
                            [tempCurPt_2_newShift] = VisualLocalizer.GetGtTrace2(b2cPmat, double(angP2CWhole + angBias), [temp_cur_lk_gt(:,5:6)],pix_z_use(ismember(pix_z_use(:,1),tempFeatId),7),intrMat);
                        else
                            [tempCurPt_2_newShift_old] = VisualLocalizer.GetGtTrace2(b2cPmat, double(angP2CWhole + angBias), [temp_cur_lk_gt(:,5:6)],pix_z_use(ismember(pix_z_use(:,1),tempFeatId),7),intrMat);
                            [tempCurPt_2_newShift, tempCurPt_2_newShift_temp, p2cBodyRotAng] = OptFlow(intrMat, b2cPmat, angP2CWhole + angBias, [temp_cur_lk_gt(:,5:6)], temp_cur_lk_gt(:,1:2),temp_cur_lk_gt(:,7),b2c);
                            e1 = tempCurPt_2_newShift_old - tempCurPt_2_newShift_temp;
                        end
                    else
                        if ~ShiftByCoord
                            [tempCurPt_2_newShift] = VisualLocalizer.GetGtTrace2(b2cPmat, double(angP2CWhole + angBias), [temp_cur_lk_gt(:,5:6)],temp_cur_lk_gt(:,7),intrMat);
                        else
                            [tempCurPt_2_newShift_old] = VisualLocalizer.GetGtTrace2(b2cPmat, double(angP2CWhole + angBias), [temp_cur_lk_gt(:,5:6)],temp_cur_lk_gt(:,7),intrMat);
                            [tempCurPt_2_newShift, tempCurPt_2_newShift_temp, p2cBodyRotAng] = OptFlow(intrMat, b2cPmat, angP2CWhole + angBias, [temp_cur_lk_gt(:,5:6)], temp_cur_lk_gt(:,1:2),temp_cur_lk_gt(:,7),b2c);
                            e1 = tempCurPt_2_newShift_old - tempCurPt_2_newShift_temp;
                        end
                        [tempCurPt_2_newShift_onlyp2] = VisualLocalizer.GetGtTrace2(b2cPmat, double(angP2CWhole + angBias), [temp_cur_lk_gt(:,5:6)],pix_z_use(ismember(pix_z_use(:,1),tempFeatId),7),intrMat);
                        
                        if 0
                            figure,plot(tempCurPt_2_newShift_onlyp2(:,1) - tempCurPt_2_newShift(:,1), tempCurPt_2_newShift_onlyp2(:,2) - tempCurPt_2_newShift(:,2), '+r');axis equal;
                        end
                        ghk = 1;
                    end
                end
                tempTrace.ptIcsX(:,end) = tempCurPt_2_newShift(:,1); % temp_cur_lk_gt(:,3) + wholeErrCurMean2(:,1);
                tempTrace.ptIcsY(:,end) = tempCurPt_2_newShift(:,2);  % temp_cur_lk_gt(:,4) + wholeErrCurMean2(:,2);
            else
                keyFrameLenTemp = size(tempTrace.ptIcsX, 2);
                angK2CWhole = [obj.accumP2CPNP2(end-(keyFrameLenTemp-1):end) - obj.accumP2CPNP2(end-(keyFrameLenTemp-1))];
                
                if 1 % ~ReplaceOnlyP2
                    [tempCurPt_2_newShift] = VisualLocalizer.GetGtTrace2(b2cPmat, double(angK2CWhole(end)), [temp_cur_lk_gt(:,8:9)],temp_cur_lk_gt(:,10),intrMat);
                else
                    [tempCurPt_2_newShift] = VisualLocalizer.GetGtTrace2(b2cPmat, double(angK2CWhole(end)), [temp_cur_lk_gt(:,8:9)],temp_cur_lk_gt(:,10),intrMat);
                end
                tempTrace.ptIcsX(:,end) = tempCurPt_2_newShift(:,1); % temp_cur_lk_gt(:,3) + wholeErrCurMean2(:,1);
                tempTrace.ptIcsY(:,end) = tempCurPt_2_newShift(:,2);
                
                
            end
        end
        if 0
            figure,plot(tempTrace_bak.ptIcsX(:,end) - tempTrace.ptIcsX(:,end), tempTrace_bak.ptIcsY(:,end) - tempTrace.ptIcsY(:,end), '+r');axis equal;
        end
        
        angDiff(j,1) = rad2deg(p2cBodyRotAng - angP2CWhole);
        
        tempCurPt_2 = [tempTrace.ptIcsX(:,end) tempTrace.ptIcsY(:,end)];
        TempCurPt_2 = [TempCurPt_2; tempCurPt_2];
        
        XYShift = [(tempCurPt_2 - [tempTrace_bak.ptIcsX(:,end) tempTrace_bak.ptIcsY(:,end)])  ]; % x~ - x
        ZShift = (pix_z_use(ismember(pix_z_use(:,1),tempFeatId),7) - temp_cur_lk_gt(:,7)); % z onlyp2 p - z stereo p
        try
            indCur = sub2ind(size(depthMapCurGT), round(tempCurPt_2(:,2)), round(tempCurPt_2(:,1)));
        catch
            sdgkjb = 1;
        end
        if 1
            
            id34 = sub2ind(size(depthMapPrvGT), round(temp_cur_lk_gt(:,6)), round(temp_cur_lk_gt(:,5)));
            if obj.switchDepth
                zGT34 = depthMapPrvGT(id34);
            else
                zGT34 = depthMapPrv(id34);
            end
            zErr34 = single(zGT34) - temp_cur_lk_gt(:,7);
            ZErr34(j,1) = max(abs(zErr34));
        end
        
        
        LocalTraceList{j,1}.ptIcsXOrig = LocalTraceList{j,1}.ptIcsX;
        LocalTraceList{j,1}.ptIcsYOrig = LocalTraceList{j,1}.ptIcsY;
        
        LocalTraceList{j,1}.ptCcsZOrig = LocalTraceList{j,1}.ptCcsZ;
        LocalTraceList{j,1}.ptCcsZGTOrig = LocalTraceList{j,1}.ptCcsZGT;
        
        
        
        LocalTraceList{j,1}.ptIcsX(:,end) = tempCurPt_2(:,1);
        LocalTraceList{j,1}.ptIcsY(:,end) = tempCurPt_2(:,2);
        
        
        if ~obj.switchDepth
            LocalTraceList{j,1}.ptCcsZ(:,end) = depthMapCur(indCur);
        else
            LocalTraceList{j,1}.ptCcsZ(:,end) = depthMapCurGT(indCur);
        end
        LocalTraceList{j,1}.ptCcsZGT(:,end) = depthMapCurGT(indCur);
        
        try
            Er = [Er; mean(tempCurPt_2 - temp_cur_lk_gt(:,3:4))];
        catch
            fhjkgfd = 1;
        end
        jhktyfjh = 1;
        if 0
            figure,plot(tempCurPt_2(:,1) - temp_cur_lk_gt(:,3), tempCurPt_2(:,2) - temp_cur_lk_gt(:,4),'+r');
        end
        
        obj.traceManager.X(tempTrace.featId,end) = tempCurPt_2(:,1);
        obj.traceManager.Y(tempTrace.featId,end) = tempCurPt_2(:,2);
        if ~obj.switchDepth
            obj.traceManager.Z(tempTrace.featId,end) = depthMapCur(indCur);
        else
            obj.traceManager.Z(tempTrace.featId,end) = depthMapCurGT(indCur);
        end
        obj.traceManager.ZGT(tempTrace.featId,end) = depthMapCurGT(indCur);
        
        
        if size(LocalTraceList{j,1}.ptIcsXOrig, 2) == 2
            obj.traceManager.XLK(tempTrace.featId, length(obj.accumP2CRef) - 1) = LocalTraceList{j,1}.ptIcsXOrig(:,1);
            obj.traceManager.YLK(tempTrace.featId, length(obj.accumP2CRef) - 1) = LocalTraceList{j,1}.ptIcsYOrig(:,1);
            obj.traceManager.ZLK(tempTrace.featId, length(obj.accumP2CRef) - 1) = LocalTraceList{j,1}.ptCcsZOrig(:,1);
            obj.traceManager.ZGTLK(tempTrace.featId, length(obj.accumP2CRef) - 1) = LocalTraceList{j,1}.ptCcsZGTOrig(:,1);
            
            obj.traceManager.XShift(tempTrace.featId, length(obj.accumP2CRef)-1) = zeros(size(XYShift,1),1);
            obj.traceManager.YShift(tempTrace.featId, length(obj.accumP2CRef)-1) = zeros(size(XYShift,1),1);
            obj.traceManager.ZShift(tempTrace.featId, length(obj.accumP2CRef)-1) = zeros(size(XYShift,1),1);
        end
        
        obj.traceManager.XLK(tempTrace.featId, length(obj.accumP2CRef)) = LocalTraceList{j,1}.ptIcsXOrig(:,end);
        obj.traceManager.YLK(tempTrace.featId, length(obj.accumP2CRef)) = LocalTraceList{j,1}.ptIcsYOrig(:,end);
        obj.traceManager.ZLK(tempTrace.featId, length(obj.accumP2CRef)) = LocalTraceList{j,1}.ptCcsZOrig(:,end);
        obj.traceManager.ZGTLK(tempTrace.featId, length(obj.accumP2CRef)) = LocalTraceList{j,1}.ptCcsZGTOrig(:,end);
        
        
        obj.traceManager.XShift(tempTrace.featId, length(obj.accumP2CRef)) = XYShift(:,1);
        obj.traceManager.YShift(tempTrace.featId, length(obj.accumP2CRef)) = XYShift(:,2);
        obj.traceManager.ZShift(tempTrace.featId, length(obj.accumP2CRef)) = ZShift(:,1);
        
    end
    
    assert(max(ZErr34) == 0);
    angDiff;
    er_ = Er - wholeErrCurMean2;
    
    %     pnp_ang_est_max_margin_2 = [deg2rad([-0.05 0.05]) 0.5];
    %     roundingPrecision = 0.00000001;
    %     p2cRef__ = deg2rad(round(rad2deg(accumP2CRef(end) - accumP2CRef(end-1))./roundingPrecision).*roundingPrecision);
    %     [p2cBodyRotAng_2,Err_2,inId___2, reProjErr___2] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin_2,intrMat,cur_lk_gt2(:,5:6),cur_lk_gt2(:,1:2),cur_lk_gt2(:,7), [1:size(cur_lk_gt2,1)]',true(size(cur_lk_gt2,1),1),b2c,double(p2cRef__),0.00001/2);
    [p2cBodyRotAng_22,Err_22,inId___22, reProjErr___22] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin_2,intrMat,cur_lk_gt2(:,5:6),TempCurPt_2(:,1:2),cur_lk_gt2(:,7), [1:size(cur_lk_gt2,1)]',true(size(cur_lk_gt2,1),1),b2c,double(p2cRef__),0.00001/2);
    
    
    if ReplaceOnlyP2
        if ~doubleShift2
            p2cBodyRotAng_22_onlyP2 = OnlyP2_ang(intrMat, cur_lk_gt2(:,5:6), TempCurPt_2, r_cam,tx, ty, tz);
        else
            p2cBodyRotAng_22_onlyP2 = OnlyP2_ang(intrMat, cur_lk_gt2(:,5:6), TempCurPt_2, r_cam,tx, ty, tz);
            p2cBodyRotAng_22_onlyP2_bak = p2cBodyRotAng_22_onlyP2;
            
            
            X_shift_all = TempCurPt_2 - cur_lk_gt2(:,1:2);
            Distri = [Distri; [mean(X_shift_all(:,1)) rad2deg(p2cBodyRotAng_22_onlyP2_bak - (thetaListGTTemp(end) - thetaListGTTemp(end-1)))]];
            
%             err_x = [err_x; err_x(end) + mean(X_shift_all(:,1))];
            

            eeee = p2cBodyRotAng_22_onlyP2_bak - angP2CWhole;
            featIdAll;
            if ~UseEx
                angBias_ = -deg2rad(err_x(end,2)/rou);
            else
                angBias_ = -deg2rad(err_x(end,1)/rou);
            end
%             angBias_ = -deg2rad(mean_x_err(end,1)/rou);
            
            %             [tempCurPt_2_newShift] = VisualLocalizer.GetGtTrace2(b2cPmat, double(angP2CWhole + angBias), [cur_lk_gt2(:,5:6)],cur_lk_gt2(:,7),intrMat);
            [tempCurPt_2_newShift_2] = VisualLocalizer.GetGtTrace2(b2cPmat, double(p2cBodyRotAng_2_onlyP2 + angBias_), [cur_lk_gt2(:,5:6)],cur_lk_gt2(:,7),intrMat);
            
            tempCurPt_2_newShift_2(tempCurPt_2_newShift_2(:,1) > size(depthMapCurGT,2),1) = size(depthMapCurGT,2);
            tempCurPt_2_newShift_2(tempCurPt_2_newShift_2(:,2) > size(depthMapCurGT,1),2) = size(depthMapCurGT,1);
            tempCurPt_2_newShift_2(tempCurPt_2_newShift_2(:,1) < 1, 1) = 1;
            tempCurPt_2_newShift_2(tempCurPt_2_newShift_2(:,2) < 1, 2) = 1;
            
            if 0 % ShiftByCoord
                [tempCurPt_2_newShift_2_old] = VisualLocalizer.GetGtTrace2(b2cPmat, double(p2cBodyRotAng_2_onlyP2 + angBias_), [cur_lk_gt2(:,5:6)],cur_lk_gt2(:,7),intrMat);
                [tempCurPt_2_newShift_2, tempCurPt_2_newShift_temp, p2cBodyRotAng] = OptFlow(intrMat, b2cPmat, p2cBodyRotAng_2_onlyP2 + angBias_, [cur_lk_gt2(:,5:6)], cur_lk_gt2(:,1:2),cur_lk_gt2(:,7),b2c);
                e11 = tempCurPt_2_newShift_2 - tempCurPt_2_newShift_2_old;
            end
            
            
            
            
            
            
            X_shift_all2 = tempCurPt_2_newShift_2 - cur_lk_gt2(:,1:2);
            if UseDoubleShift
                err_x = [err_x; [Rou.*err_x(end,1) + mean(X_shift_all2(:,1)) mean(X_shift_all2(:,1))]];
            else
                err_x = [err_x; [Rou.*err_x(end,1) + mean(X_shift_all(:,1)) mean(X_shift_all(:,1))]];
            end
            [err_x(:,1) cumsum(err_x(:,2))];
            if 0
                p2cBodyRotAng_22_onlyP2 = OnlyP2_ang(intrMat, cur_lk_gt2(:,5:6), tempCurPt_2_newShift_2, r_cam,tx, ty, tz);
            else
                if 0 %~UseEx
                    p2cBodyRotAng_22_onlyP2 = p2cBodyRotAng_22_onlyP2_bak - deg2rad((err_x(end,2))./rou);     % OnlyP2_ang(intrMat, cur_lk_gt2(:,5:6), tempCurPt_2_newShift_2, r_cam,tx, ty, tz);
                else
%                     p2cBodyRotAng_22_onlyP2 = p2cBodyRotAng_22_onlyP2_bak - deg2rad((err_x(end,1))./rou);     % OnlyP2_ang(intrMat, cur_lk_gt2(:,5:6), tempCurPt_2_newShift_2, r_cam,tx, ty, tz);
                    p2cBodyRotAng_22_onlyP2 = p2cBodyRotAng_22_onlyP2_bak - deg2rad((err_x(end-1,1))./rou);
                end
                
            end
            
            if UseDoubleShift
                indCur2 = sub2ind(size(depthMapCurGT), round(tempCurPt_2_newShift_2(:,2)), round(tempCurPt_2_newShift_2(:,1)));
                
                
                obj.traceManager.X(featIdAll, length(obj.accumP2CRef)) = tempCurPt_2_newShift_2(:,1);
                obj.traceManager.Y(featIdAll, length(obj.accumP2CRef)) = tempCurPt_2_newShift_2(:,2);
                
                if ~obj.switchDepth
                    obj.traceManager.Z(featIdAll,length(obj.accumP2CRef)) = depthMapCur(indCur2);
                else
                    obj.traceManager.Z(featIdAll,length(obj.accumP2CRef)) = depthMapCurGT(indCur2);
                end
                obj.traceManager.ZGT(featIdAll, length(obj.accumP2CRef)) = depthMapCurGT(indCur2);
                
                
                
                %             obj.traceManager.XShift(featIdAll, length(obj.accumP2CRef)) = tempCurPt_2_newShift_2(:,1) - cur_lk_gt2(:,5);
                %             obj.traceManager.YShift(featIdAll, length(obj.accumP2CRef)) = tempCurPt_2_newShift_2(:,2) - cur_lk_gt2(:,6);
                obj.traceManager.XShift(featIdAll, length(obj.accumP2CRef)) = tempCurPt_2_newShift_2(:,1) - cur_lk_gt2(:,1);
                obj.traceManager.YShift(featIdAll, length(obj.accumP2CRef)) = tempCurPt_2_newShift_2(:,2) - cur_lk_gt2(:,2);
                %             p2cBodyRotAng_22_onlyP2 = p2cBodyRotAng_2_onlyP2 - deg2rad(mean(X_shift_all(:,1))/rou);
            end
        end
    else
        p2cBodyRotAng_22_onlyP2 = p2cBodyRotAng_22;
    end
    
else
    p2cBodyRotAng_22 = p2cBodyRotAng_2;
    p2cBodyRotAng_22_onlyP2 = p2cBodyRotAng_2;
end
try
    er_456 = rad2deg(p2cBodyRotAng_22_onlyP2 - angP2CWhole);
catch
    dfgkhj = 1;
end
if size(obj.accumP2CPNP2_2,1) == 1
    
    obj.accumP2CPNP2_2 = [0 0 0 0; [p2cBodyRotAng_2 p2cBodyRotAng_22 p2cBodyRotAng_2_onlyP2 p2cBodyRotAng_22_onlyP2]];
else
    
    obj.accumP2CPNP2_2 = [obj.accumP2CPNP2_2; [(obj.accumP2CPNP2_2(end,1)+p2cBodyRotAng_2) (obj.accumP2CPNP2_2(end,2)+p2cBodyRotAng_22) (obj.accumP2CPNP2_2(end,3)+p2cBodyRotAng_2_onlyP2) (obj.accumP2CPNP2_2(end,4)+p2cBodyRotAng_22_onlyP2)]];
end

cur_id_lk_gt2 = [cell2mat(pixGT_temp_stack2(:,1)) cur_lk_gt2];

if ShiftLK
    if length(accumP2CRef) == 2
        
        %     er3 = [LocalTraceList{j,1}.ptIcsX(:,end) LocalTraceList{j,1}.ptIcsY(:,end)] - tempCurPt_2(:,1:2);
        er3 = [tempTrace_bak.ptIcsX(:,end) tempTrace_bak.ptIcsY(:,end)] - tempCurPt_2(:,1:2);
        
        er33 = mean(er3);
        
    end
end


unValidFeatId1 = setdiff(featId, validFeatId);

if 1
    er11 = unValidFeatId1 - unValidFeatId;
    assert(sum(abs(er11)) == 0);
end


zz = localZ(validFeatId, :);
zzGT = localZGT(validFeatId, :);

dd = (intrMat(1,1).*norm(baseline)./zz) - (princpPtR(1) - princpPtL(1));
dd(zz == -1) = -1;
ddGT = (intrMat(1,1).*norm(baseline)./zzGT) - (princpPtR(1) - princpPtL(1));
ddGT(zzGT == -1) = -1;


data.featSize = featSize;
data.dd = dd;
data.ddGT = ddGT;
data.disparityRng = disparityRng;

dsfagk = 1;
if draw
    figure(figNum);
    
    if 0
        plot(localX(featId, end), localY(featId, end), 'ro');
        title(sprintf('%d tracked feature points',length(featId)))
    else
        plot(localX(validFeatId, end), localY(validFeatId, end), 'ro');
        title(sprintf('%d tracked feature points',length(validFeatId)))
    end
    
    drawnow;
    hold off;
end

end
function ang = OnlyP2_ang(intrMat, ptPrv, ptCur, r_cam,tx, ty, tz)
ptPrv_norm = inv(intrMat)*pextend(ptPrv');
ptCur_norm = inv(intrMat)*pextend(ptCur');
[angle_resultP2C,depthCurInPrv] = VisualLocalizer.AnglePredict_Prob(ptPrv_norm,ptCur_norm,r_cam,tx, ty, tz);
[angle_resultP2C1,~] = VisualLocalizer.AnglePredict(ptPrv_norm,ptCur_norm, tx, ty, tz);
angDiffer = angle_resultP2C - angle_resultP2C1';
validAng = find(abs(angDiffer) < 0.01);
ang = deg2rad(mean(angle_resultP2C(validAng)));
end


function er = CalibB2C(thetaList, traceInfoMat,traceManager, doDiff)
global titleStr thetaListGT ShiftInterval

traceX = traceManager.X;
traceY = traceManager.Y;
traceX_ref = traceManager.XLK;
traceY_ref = traceManager.YLK;
xTraceShift = traceManager.XShift;
yTraceShift = traceManager.YShift;
zTraceShift = traceManager.ZShift;





thetaList = thetaList - thetaList(1);
Err = {}; error = []; b2c_vec = []; camPoseC2K = {}; bodyPoseK2C = {};
XErrMat = []; YErrMat = []; cnt = 1; validCnt = []; MetricErr = {};
numThr = 30;
for i = 1 : size(traceInfoMat,1) - 16 % 45 % size(traceInfoMat,1) - 2
    
    if ~isempty(traceInfoMat{i,1})
        frameRng = traceInfoMat{i,3}(1)-1:traceInfoMat{i,3}(end);
        minFrameNum = length(frameRng);
        try
            thetaListTemp = thetaList(frameRng);
        catch
            fjh = 1;
        end
        thetaListTemp = thetaListTemp - thetaListTemp(1);
     
        featIdKey = traceInfoMat{i,2};
        try
            xMat = traceX(featIdKey, frameRng);
            xMat_ref = traceX_ref(featIdKey, frameRng);
            xShift = xTraceShift(featIdKey, frameRng);
            zShift = zTraceShift(featIdKey, frameRng);
        catch
            break;
            asgdhl = 1;
        end
        try
            yMat = traceY(featIdKey, frameRng);
            yMat_ref = traceY_ref(featIdKey, frameRng);
        catch
            dfsghkj = 1;
        end
%         depthKey = depthGT(:,:,i);
        
        pix = [xMat(:,1) yMat(:,1)];
        pix_ref = [xMat_ref(:,1) yMat_ref(:,1)];
        pix_shiftZ = xShift(:,1);
        pix_shiftZ = zShift(:,1);
%         indKey = sub2ind(size(depthKey), round(pix(:,2)), round(pix(:,1)));
%         depthKList = depthKey(indKey);
%         try
%             [xyzKey] = GetXYZFromDepth(intrMat, pix,depthKList);
%         catch
%             continue;
%         end
%         XYZKey{cnt,1} = xyzKey;
        DiffDiffAng = rad2deg(diff(diff(thetaListTemp)));
        if max(abs(DiffDiffAng)) > 10000   %0.2 % 0.015
            featIdKey = [];
        end
        if length(featIdKey) > numThr
            
                err = {};IdxInliers = [];
                %             RtTemp = [];
                for j = 1 : minFrameNum - 1
                    pixCur = [xMat(:,j+1), yMat(:,j+1)];
                    pixCur_ref = [xMat_ref(:,j+1), yMat_ref(:,j+1)];
                    pixCur_shiftX = xShift(:,j+1);
                    pixCur_shiftZ = zShift(:,j+1);

                        idxOutliers = [];
                  
                    
                    inliers = setdiff([1:size(pix,1)]', idxOutliers);

                    XErrMat_body(cnt,j+1) = -mean(pixCur_ref(inliers,1) - pixCur(inliers,1));
                    YErrMat_body(cnt,j+1) = -mean(pixCur_ref(inliers,2) - pixCur(inliers,2));
                    
                    ZErrMat_body_shift(cnt,j+1) = median(pixCur_shiftZ);
                    
                    
                    
                    XErrMat_body2(i,j+1) = -mean(pixCur_ref(inliers,1) - pixCur(inliers,1));
                    YErrMat_body2(i,j+1) = -mean(pixCur_ref(inliers,2) - pixCur(inliers,2));
                    
                    if j == 1
                        XErrMat_body_stack{cnt, j} = pix(:,1:2);
                        YErrMat_body_stack{cnt, j} = pix(:,1:2);
                        XErrMat_body_stack2{i, j} = pix(:,1:2);
                        YErrMat_body_stack1{i, j} = pix(:,1:2);
                    end
                    
                    XErrMat_body_stack{cnt, j+1} = -(pixCur_ref(inliers,1) - pixCur(inliers,1));
                    YErrMat_body_stack{cnt, j+1} = -(pixCur_ref(inliers,2) - pixCur(inliers,2));
                    
                    XErrMat_body_stack2{i, j+1} = -(pixCur_ref(inliers,1) - pixCur(inliers,1));
                    YErrMat_body_stack2{i, j+1} = -(pixCur_ref(inliers,2) - pixCur(inliers,2));
                    

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
                    
                    asgk = 1;
                end
                
                
        end
    end
end


if ~doDiff % PrvTrackOnGT
    x_p2c = XErrMat_body(:,2:end);
    y_p2c = YErrMat_body(:,2:end);
    z_p2c = ZErrMat_body_shift(:,2:end);
else
    x_p2c = diff(XErrMat_body')';
    x_p2c(XErrMat_body(:,2:end) == 0) =  0;
    y_p2c = diff(YErrMat_body')';
    z_p2c = diff(ZErrMat_body_shift')';
    z_p2c(ZErrMat_body_shift(:,2:end) == 0) =  0;

end
for i = 1 : size(x_p2c, 1) % size(traceInfoMat,1) - 2
    
    
%     frameRng1 = traceInfoMat{i,3} - 1;
%     xErrMat(i,frameRng1) = x_p2c(i,1:length(frameRng1));
%     yErrMat(i,frameRng1) = y_p2c(i,1:length(frameRng1));
    
    frameRng1 = traceInfoMat{validCnt(i),3} - 1;    
    xErrMat(i,frameRng1) = x_p2c(i,1:length(frameRng1));
    yErrMat(i,frameRng1) = y_p2c(i,1:length(frameRng1));
    zErrMat(i,frameRng1) = z_p2c(i,1:length(frameRng1));
end

thetaList = thetaListGT(1:length(thetaList));
traceNum1 = size(XErrMat_body,2);
[x_err1, x_err1_upper, x_err1_lower] = CalcEnvelope(xErrMat, traceNum1);
[z_err1, z_err1_upper, z_err1_lower] = CalcEnvelope(zErrMat, traceNum1);

   figure,subplot(2,1,1);plot(rad2deg(thetaList(1:size(x_err1,1))), x_err1);hold on;plot(rad2deg(thetaList(1:size(x_err1,1))),[x_err1_upper x_err1_lower],'-xb');plot(rad2deg(thetaList(1:size(x_err1,1))),[mean(x_err1')],'-xr');title(sprintf('doDiff: %d', doDiff)); %title(sprintf('Shift Interval: %d',ShiftInterval));
    subplot(2,1,2);plot(rad2deg(thetaList(1:size(x_err1,1))), [mean(x_err1')],'-xr');title(titleStr) % plot(rad2deg(thetaList2(1:size(x_err2,1))),x_err2);hold on;plot(rad2deg(thetaList2(1:size(x_err2,1))),[x_err2_upper x_err2_lower],'-xb');plot(rad2deg(thetaList2(1:size(evenErr2,1))),[evenErr2],'-xr');
    if ~doDiff
%         hold on;plot(rad2deg(thetaList(1:end-1)), 16.*rad2deg(traceInfoMat{1,6}(2:end) - thetaList(2:end)));
        hold on;plot(rad2deg(thetaList(1:end-1)), 10.*rad2deg(traceInfoMat{1,7}(2:end,3:4) - thetaList(2:end)));grid on;
    else
%         hold on;plot(rad2deg(thetaList(1:end-1)), 1.*rad2deg(traceInfoMat{1,6}(2:end) - thetaList(2:end)));
        hold on;plot(rad2deg(thetaList(1:end-1)), 1.*rad2deg(traceInfoMat{1,7}(2:end,3:4) - thetaList(2:end)));grid on;
    end
    er = 1.*rad2deg(traceInfoMat{1,7}(2:end,3:4) - thetaList(2:end));
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
function [pixCurNew, tempCurPt_2_newShift, p2cBodyRotAng_22] = OptFlow(intrMat, b2cPmat, angP2CWhole, pixPrv, pixCur,zPrv,b2c)

pnp_ang_est_max_margin_2 = [deg2rad([-0.005 0.005]) 0.7];
step = 0.00001/10;
options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','off','MaxFunEvals',10000,'TolX',1e-8);%,'MaxFunEvals',10000);%, 'MaxIterations',5);


[tempCurPt_2_newShift] = VisualLocalizer.GetGtTrace2(b2cPmat, double(angP2CWhole),pixPrv,zPrv,intrMat);

xyShift0 = double(mean(tempCurPt_2_newShift - pixCur));

 [vec,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(U) CostFunc(pnp_ang_est_max_margin_2, intrMat, pixPrv, pixCur, zPrv, b2c, angP2CWhole, step, U),[xyShift0],[],[],options);%,data_1,obs_1)


[p2cBodyRotAng_22,Err_22,inId___22, reProjErr___22] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin_2,intrMat,pixPrv,pixCur + vec, zPrv, [1:size(pixPrv,1)]',true(size(pixPrv,1),1),b2c,double(angP2CWhole),step);

differ = rad2deg(p2cBodyRotAng_22 - angP2CWhole);
pixCurNew = pixCur + vec;

end
function err = CostFunc(pnp_ang_est_max_margin_2, intrMat, pixPrv, pixCur, zPrv, b2c, p2cRef__,step, xyShift)

% pnp_ang_est_max_margin_2 = [deg2rad([-0.15 0.15]) 0.7];


[p2cBodyRotAng_22,Err_22,inId___22, reProjErr___22] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin_2,intrMat,pixPrv,pixCur + xyShift, zPrv, [1:size(pixPrv,1)]',true(size(pixPrv,1),1),b2c,double(p2cRef__),step);

err = 10000000.*double(rad2deg(abs(p2cBodyRotAng_22 - p2cRef__)));

end
function xErrMat = ArrangeData(traceManager, traceInfoMat)

xErrMat = [];
for i = 1 : size(traceInfoMat,1)
    tempFeatId = traceInfoMat{i,2};
    if ~isempty(tempFeatId)
        frameRng = traceInfoMat{i,3}(1)-1:traceInfoMat{i,3}(end);
        frameRng1 = traceInfoMat{(i),3} - 1;
        
        xMat = traceManager.X(tempFeatId, frameRng);
        xLKMat = traceManager.XLK(tempFeatId, frameRng);
        xShiftMat = traceManager.XShift(tempFeatId, frameRng);
        xShiftMat_check = xMat - xLKMat;
        
        xShiftMat_check_err = abs(xShiftMat_check - xShiftMat);
        
        assert(max(xShiftMat_check_err(:)) == 0);
        
        xErrMat(i, frameRng1) = mean(xShiftMat(:,2:end));

        
        
        
    end
    
end


end