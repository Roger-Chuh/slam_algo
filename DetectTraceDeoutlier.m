function [LocalTraceList, validFeatId, traceLenList, pt2d_check, dataTmp, dispErrMat, dispErrExp, traceBatchList] = DetectTraceDeoutlier(obj, b2c, k2cRef,p2cRef, r_cam, tx, ty, tz)
global probPath FigBase addPtInterval newLK PrvTrackOnGT newRender ShiftLK newShift ReplaceOnlyP2 NewTracker ReplaceOnlyP2Z NewTracker2

AngleModalOrg = obj.angleModalOrg;
% NewTracker = true; false; true;


% PrvTrackOnGT = false; true;

[marg, ~, ~, ~] = GetPredictMargin(obj.featPtTracker);
intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,obj.scaleLvl);
[princpPtL, princpPtR] = Get(obj.camModel, 'PinholePrincpPt', obj.scaleLvl);
b2cPmat = GetPctBody2Cam(obj.coordSysAligner, 1);
baseline = obj.camModel.transVec1To2;
if isempty(obj.traceManager.X)
    prvFrmNum = 1;  size(obj.traceManager.X, 2);
    
    depthMapPrvGT = obj.keyFrameDepthGT;
    depthMapCurGT = obj.depthGT;
    xMat = obj.featPtManager.localTrace.ptIcsX;
    yMat = obj.featPtManager.localTrace.ptIcsY;
    idCur = find(xMat(:,end) > 0 & yMat(:,end) > 0);
    validIndPrv = sub2ind(size(depthMapPrvGT), round(yMat(:,1)), round(xMat(:,1)));
    depthGtPrv = depthMapPrvGT(validIndPrv);
    
    idCur_ = zeros(size(xMat,1),1);
    validIndCur_ = sub2ind(size(depthMapCurGT), round(yMat(idCur,2)), round(xMat(idCur,2)));
    idCur_(idCur) = validIndCur_;
    depthGtCur = depthMapCurGT(validIndCur_);
    depthGtCur_all = nan(size(xMat,1),1);
    depthGtCur_all(idCur) = depthGtCur;
    
    obj.traceManager.X = obj.featPtManager.localTrace.ptIcsX;
    obj.traceManager.Y = obj.featPtManager.localTrace.ptIcsY;
    if 1
        id__ = find(obj.traceManager.X(:,end)  <= 0 );
        id = unique([id__; find(isnan(depthGtPrv) | isnan(depthGtCur_all))]);
        id_1 = setdiff([1 : size(xMat,1)]',id);
    else
        id = find(obj.traceManager.X(:,end)  <= 0 );
        id_1 = find(obj.traceManager.X(:,end)  > 0);
    end
    if ~obj.switchDepth
        depthMapPrv = obj.keyFrameDepth; %depthMapKey_2; % keyFrameDepth;
        depthMapCur = obj.depthVisual;
    else
        depthMapPrv = obj.keyFrameDepthGT; %depthMapKeyGT_2;
        depthMapCur = obj.depthGT;
    end
    
    if 0
        depthMapPrvGT = obj.keyFrameDepthGT;
        depthMapCurGT = obj.depthGT;
    end
    
    inValid___ = sub2ind(size(depthMapPrv), round(obj.traceManager.Y(:,1)), round(obj.traceManager.X(:,1)));
    %     inValid___ = inValid___(depthMapPrv(inValid___)>0);
    inValid___Cur = sub2ind(size(depthMapCur), round(obj.traceManager.Y(id_1,2)), round(obj.traceManager.X(id_1,2)));
    %     inValid___Cur = inValid___Cur(depthMapCur(inValid___Cur)>0);
    %     obj.traceManager.Z = [depthMapPrv(inValid___) depthMapPrv(inValid___)];
    % %     id = find(obj.traceManager.X(:,end)  <= 0 );
    % %     id_1 = find(obj.traceManager.X(:,end)  > 0);
    
    obj.traceManager.Z(:,1) = [depthMapPrv(inValid___)];
    obj.traceManager.Z(id_1,2) = [depthMapCur(inValid___Cur)];
    obj.traceManager.Z(id,2) = -1;
    
    obj.traceManager.ZGT(:,1) = [depthMapPrvGT(inValid___)];
    obj.traceManager.ZGT(id_1,2) = [depthMapCurGT(inValid___Cur)];
    obj.traceManager.ZGT(id,2) = -1;
    
    obj.addPt = [obj.addPt; length(obj.keyFrameFlagList)];
    
    asdgfh = 1;
    
else
%     if length(obj.keyFrameFlagList) - obj.addPt(end) >= addPtInterval
    
    prvFrmNum = size(obj.traceManager.X, 2);
    
    curValidPt = find(obj.traceManager.X(:,prvFrmNum) > 0);
    prevFeatPtList_22 = [obj.traceManager.X(curValidPt,prvFrmNum) obj.traceManager.Y(curValidPt,prvFrmNum)];
    prevFeatPtList_22_z = obj.traceManager.Z(curValidPt,prvFrmNum);
    
    depthMapPrvGT = obj.prvDepthGT;
    depthMapCurGT = obj.depthGT;
    
    if 0
        
        angP2CWhole_ = obj.accumP2CPNP2(end) - obj.accumP2CPNP2(end-1);
        
        tempPrvTrace = obj.traceInfoMat{1,5};
        meanErrPev = mean(obj.traceInfoMat{1,4}(:,2:3) - obj.traceInfoMat{1,4}(:,4:5));
        
        tempPrvTraceMeanStack = [];err_cur_pt = [];
        for f = 1 : size(tempPrvTrace, 1)
            tempPrvTraceFeatId = tempPrvTrace{f,1};
            tempPrvTraceFeatList = tempPrvTrace{f,2};
            tempPrvTraceId = find(ismember(obj.traceInfoMat{1,4}(:,1),tempPrvTraceFeatId,'rows'));
            %            mean(prevFeatPtList_22(:,1:2) - obj.traceInfoMat{1,4}(:,4:5))
            %             mean(tempPrvTraceFeatList(:,1:2) - tempPrvTraceFeatList(:,3:4));
            if newShift
                prv_pt = [obj.traceManager.X(tempPrvTraceFeatId,end-1) obj.traceManager.Y(tempPrvTraceFeatId,end-1)];
                if obj.switchDepth
                    prv_z = obj.traceManager.ZGT(tempPrvTraceFeatId,end-1);
                else
                    prv_z = obj.traceManager.Z(tempPrvTraceFeatId,end-1);
                end
                if 1
                    clear KeyFrameLen_1
                    for kl = 1 : size(obj.traceInfoMat,1)
                       if ~isempty(obj.traceInfoMat{kl,1}) 
                           comId = find(ismember(obj.traceInfoMat{kl,2}(:,1),tempPrvTraceFeatId,'rows'));
                           if length(comId) > 0
                              KeyFrameLen_1 = length(obj.traceInfoMat{kl,3}) + 1;
                               break;
                           else
                               continue;
                           end
                        
                       end
                    end
                    
                    key_pt = [obj.traceManager.X(tempPrvTraceFeatId,end - KeyFrameLen_1 + 1) obj.traceManager.Y(tempPrvTraceFeatId,end - KeyFrameLen_1 + 1)];
                    if obj.switchDepth
                        key_z = obj.traceManager.ZGT(tempPrvTraceFeatId,end - KeyFrameLen_1 + 1);
                    else
                        key_z = obj.traceManager.Z(tempPrvTraceFeatId,end - KeyFrameLen_1 + 1);
                    end
                    angK2CWhole_ = [obj.accumP2CPNP2(end-(KeyFrameLen_1-1):end) - obj.accumP2CPNP2(end-(KeyFrameLen_1-1))];

                    [cur_pt__1] = VisualLocalizer.GetGtTrace2(b2cPmat, double(angK2CWhole_(end)), key_pt,key_z,intrMat); 
                end
                cur_pt_ = [obj.traceManager.X(tempPrvTraceFeatId,end) obj.traceManager.Y(tempPrvTraceFeatId,end)];
                cur_pt = tempPrvTraceFeatList(:,1:2);
               [cur_pt__] = VisualLocalizer.GetGtTrace2(b2cPmat, double(angP2CWhole_), prv_pt,prv_z,intrMat); 
               err_cur_pt = [err_cur_pt; [(cur_pt_ - cur_pt__) (cur_pt_ - cur_pt__1)]];
            end

            tempPrvTraceMeanBatch(f,:) = [(mean(tempPrvTraceFeatList(:,1:2) - tempPrvTraceFeatList(:,3:4))) (mean(prevFeatPtList_22(tempPrvTraceId,:) - tempPrvTraceFeatList(:,3:4)))];
            tempPrvTraceMeanStack = [tempPrvTraceMeanStack; ];
       end
       meanErrPrv = mean(prevFeatPtList_22 - obj.traceInfoMat{1,4}(:,4:5));

        figure,subplot(1,2,1);plot(obj.traceInfoMat{1,4}(:,2) - obj.traceInfoMat{1,4}(:,4), obj.traceInfoMat{1,4}(:,3) - obj.traceInfoMat{1,4}(:,5), '+r');axis equal;title(sprintf('mean err: [%0.5f %0.5f]',meanErrPev));
         subplot(1,2,2);plot(prevFeatPtList_22(:,1) - obj.traceInfoMat{1,4}(:,4), prevFeatPtList_22(:,2) - obj.traceInfoMat{1,4}(:,5), '+r');axis equal;title(sprintf('mean err: [%0.5f %0.5f]',meanErrPrv));
    end
    
    
    
    
    sakgj = 1;
    
    if ~PrvTrackOnGT
        if ~NewTracker
            if ~newLK
                [predPtList000, inTrackFlag000, marg] = TrackFeaturePoints(obj.featPtTracker, obj.prevImgL, obj.currImgL, prevFeatPtList_22, [], intrMat, GetPctBody2Cam(obj.coordSysAligner));
            else
                [predPtList000, inTrackFlag000] = LKTracking(obj.prevImgL, obj.currImgL, prevFeatPtList_22, [],marg);
            end
        else
            if ~NewTracker2
                [predPtList000, inTrackFlag000, ~, ~, marg] = NewTrackingOld2(obj,  obj.prevImgL, obj.currImgL,size(prevFeatPtList_22, 1), prevFeatPtList_22, prevFeatPtList_22_z, intrMat, double(p2cRef), AngleModalOrg);
            else
                [predPtList000, inTrackFlag000, ~, ~, marg] =  NewTrackingFinal(obj,  obj.prevImgL, obj.currImgL,size(prevFeatPtList_22, 1), prevFeatPtList_22, prevFeatPtList_22_z, intrMat, double(p2cRef), AngleModalOrg);
            end
        end
    else
        
%         b2cPmat = GetPctBody2Cam(obj.coordSysAligner, 1);
        if 0
            PrvPtList = [];
            FeatIdLast = [];
            for ji = 1 : size(obj.dispErrExpStack3,1)
                tempData = obj.dispErrExpStack3(ji,:);
                if tempData{8} == 0
                    LocalTrace_last = tempData{1, 7}.LocalTrace;
                    ptSize = size(LocalTrace_last.ptIcsX,1);
                    
                    featIdLast = LocalTrace_last.featId;
                    FeatIdLast = [FeatIdLast; featIdLast];
                    pix = [LocalTrace_last.ptIcsX(:,1) LocalTrace_last.ptIcsY(:,1)];
                    zGT = LocalTrace_last.ptCcsZGT(:,1);
                    gtThetaPrv = tempData{1,5}(end,4);
                    [pixGTPrv] = VisualLocalizer.GetGtTrace2(b2cPmat, gtThetaPrv, [pix],zGT,intrMat);
                    if 0
                        prvPixInd = sub2ind(size(depthMapPrvGT), round(pixGTPrv(:,2)), round(pixGTPrv(:,1)))
                        
                        [XYZ] = GetXYZFromDepth(intrMat, pixGTPrv,depthList)
                    end
                    PrvPtList = [PrvPtList; pixGTPrv];
                    
                    
                    if 0
                        CalcGoldenTracking(obj, intrMat, obj.dispErrExpStack3);
                    end
                    
                    
                end
            end
            
            FeatIdLast2 = intersect(curValidPt,FeatIdLast );
            
            assert(sum(abs([FeatIdLast2 - curValidPt])) == 0);
            
            prevFeatPtList_22_LK = prevFeatPtList_22;
            if ~newLK
                [predPtList000_LK, inTrackFlag000_LK, ~] = TrackFeaturePoints(obj.featPtTracker, obj.prevImgL, obj.currImgL, prevFeatPtList_22_LK, [], intrMat, GetPctBody2Cam(obj.coordSysAligner));
                
                [predPtList000, inTrackFlag000, marg] = TrackFeaturePoints(obj.featPtTracker, obj.prevImgL, obj.currImgL, PrvPtList, [], intrMat, GetPctBody2Cam(obj.coordSysAligner));
            else
                [predPtList000_LK, inTrackFlag000_LK] = LKTracking(obj.prevImgL, obj.currImgL, prevFeatPtList_22_LK, [],marg);
                [predPtList000, inTrackFlag000] = LKTracking(obj.prevImgL, obj.currImgL, PrvPtList, [],marg);
                
            end
            in_flag = inTrackFlag000_LK & inTrackFlag000;
            if 0
                figure, subplot(1,2,1); plot(PrvPtList(:,1) - prevFeatPtList_22_LK(:,1), PrvPtList(:,2) - prevFeatPtList_22_LK(:,2), '+r');title('prv');axis equal;
                subplot(1,2,2); plot(predPtList000(in_flag,1) - predPtList000_LK(in_flag,1), predPtList000(in_flag,2) - predPtList000_LK(in_flag,2), '+r');title('cur');axis equal;
            end
            
            prevFeatPtList_22 = PrvPtList;
            prevFeatPtList_22 = prevFeatPtList_22_LK;
            
        else
            pixRemapErr = []; replacePrvPt = [];
            for p = 1 : size(obj.traceInfoMat,1)
                
                tempInfo = obj.traceInfoMat(p,:);
                angId = tempInfo{1,3};
                if isempty(angId)
                    continue;
                end
                angId = [angId(1) - 1; angId];
                featIdTemp = tempInfo{1,2};
                
                if angId(end) == length(obj.accumP2CRef) - 1
                    keyFrameIndTemp = angId(1);
                    prvFrameIndTemp = angId(end);
                    
                    pixKeyTemp = [obj.traceManager.X(featIdTemp, keyFrameIndTemp) obj.traceManager.Y(featIdTemp, keyFrameIndTemp)];
                    pixPrvTemp = [obj.traceManager.X(featIdTemp, prvFrameIndTemp) obj.traceManager.Y(featIdTemp, prvFrameIndTemp)];
                    
                    zKeyGTTemp = [obj.traceManager.ZGT(featIdTemp, keyFrameIndTemp)];
                    zKeyTemp = [obj.traceManager.Z(featIdTemp, keyFrameIndTemp)];
                    
                    zPrvGTTemp = [obj.traceManager.ZGT(featIdTemp, prvFrameIndTemp)];
                    zPrvTemp = [obj.traceManager.Z(featIdTemp, prvFrameIndTemp)];
                    
                    tempAngRefList = obj.accumP2CRef(angId);
                    tempAngRefList = tempAngRefList - tempAngRefList(1);
                    
                    [pixGTPrv_Temp] = VisualLocalizer.GetGtTrace2(b2cPmat, tempAngRefList(end), [pixKeyTemp],zKeyGTTemp,intrMat);
                    
                    prvPixInd = sub2ind(size(depthMapPrvGT), round(pixGTPrv_Temp(:,2)), round(pixGTPrv_Temp(:,1)));
                    
%                     [XYZ_PrvGT] = GetXYZFromDepth(intrMat, pixGTPrv_Temp, depthMapPrvGT(prvPixInd));
                    
                    [pixGTKey_Temp] = VisualLocalizer.GetGtTrace2(b2cPmat, -tempAngRefList(end), [pixGTPrv_Temp],depthMapPrvGT(prvPixInd),intrMat);
                    
                    if 0 %  length(angId) == 2
                        pixGTKey_Temp = pixKeyTemp;
                    end
                    
                    pixRemapErr = [pixRemapErr; [featIdTemp pixKeyTemp pixGTKey_Temp]];
                    
                    
                    prvTrackingErr = pixPrvTemp - pixGTPrv_Temp;
                    replacePrvPt = [replacePrvPt; [featIdTemp pixGTPrv_Temp pixPrvTemp]];
                    
                    keyFrameLenTemp = 1;
                    
                    
                end
                
                
            end
            assert(sum(abs([replacePrvPt(:,1) - curValidPt])) == 0);
            prvPtMisalignment = prevFeatPtList_22 - replacePrvPt(:,4:5);  
            if 0
                figure,subplot(1,2,1);plot([pixRemapErr(:,2) - pixRemapErr(:,4) pixRemapErr(:,3) - pixRemapErr(:,5)]);legend('x err','y err');title('remap prv pt to key');
                subplot(1,2,2);plot(-replacePrvPt(:,4) + replacePrvPt(:,2), -replacePrvPt(:,5) + replacePrvPt(:,3), '+r');axis equal;title('prv - prv gt');
            end
            shhghk = 1;
            
            prevFeatPtList_22_LK = prevFeatPtList_22;
            prevFeatPtList_22_gt = replacePrvPt(:,2:3);
            if 0
                if ~NewTracker
                    if ~newLK
                        [predPtList000, inTrackFlag000, marg] = TrackFeaturePoints(obj.featPtTracker, obj.prevImgL, obj.currImgL, prevFeatPtList_22_gt, [], intrMat, GetPctBody2Cam(obj.coordSysAligner));
                    else
                        [predPtList000, inTrackFlag000] = LKTracking(obj.prevImgL, obj.currImgL, prevFeatPtList_22_gt, [],marg);
                    end
                else
%                     if ~NewTrackingOld2
                    if ~NewTracker2
                        [predPtList000, inTrackFlag000, ~, ~, marg] = NewTrackingOld2(obj,  obj.prevImgL, obj.currImgL,size(prevFeatPtList_22_gt, 1), prevFeatPtList_22_gt, prevFeatPtList_22_z, intrMat, double(p2cRef), AngleModalOrg);
                    else
%                         [predPtList000, inTrackFlag000, ~, ~, marg] = NewTrackingOld2(obj,  obj.prevImgL, obj.currImgL,size(prevFeatPtList_22_gt, 1), prevFeatPtList_22_gt, prevFeatPtList_22_z, intrMat, double(p2cRef), AngleModalOrg);
                        [predPtList000, inTrackFlag000, ~, ~, marg] =  NewTrackingFinal(obj,  obj.prevImgL, obj.currImgL,size(prevFeatPtList_22_gt, 1), prevFeatPtList_22_gt, prevFeatPtList_22_z, intrMat, double(p2cRef), AngleModalOrg);
                    end
                end
            else
                if ~newLK
                    [predPtList000_LK, inTrackFlag000_LK, ~] = TrackFeaturePoints(obj.featPtTracker, obj.prevImgL, obj.currImgL, prevFeatPtList_22_LK, [], intrMat, GetPctBody2Cam(obj.coordSysAligner));
                    
                    [predPtList000, inTrackFlag000, marg] = TrackFeaturePoints(obj.featPtTracker, obj.prevImgL, obj.currImgL, prevFeatPtList_22_gt, [], intrMat, GetPctBody2Cam(obj.coordSysAligner));
                else
                    [predPtList000_LK, inTrackFlag000_LK] = LKTracking(obj.prevImgL, obj.currImgL, prevFeatPtList_22_LK, [],marg);
                    [predPtList000, inTrackFlag000] = LKTracking(obj.prevImgL, obj.currImgL, prevFeatPtList_22_gt, [],marg);
                    
                end
                in_flag = inTrackFlag000_LK & inTrackFlag000;
                if 0
                    figure, subplot(1,2,1); plot(prevFeatPtList_22_gt(:,1) - prevFeatPtList_22_LK(:,1), prevFeatPtList_22_gt(:,2) - prevFeatPtList_22_LK(:,2), '+r');title('prv');axis equal;
                    subplot(1,2,2); plot(predPtList000(in_flag,1) - predPtList000_LK(in_flag,1), predPtList000(in_flag,2) - predPtList000_LK(in_flag,2), '+r');title('cur');axis equal;
                end
                
                prevFeatPtList_22 = prevFeatPtList_22_gt;
                prevFeatPtList_22 = prevFeatPtList_22_LK;
            end
            
            
        end
        
    end
%     depthMapPrvGT = obj.prvDepthGT;
%     depthMapCurGT = obj.depthGT;
    
    vldPtCur = nan(size(predPtList000,1),1);
    vldPrTrace = find(predPtList000(:,1) > marg & predPtList000(:,2) > marg & predPtList000(:,1) < size(depthMapCurGT,2) - marg & predPtList000(:,2) < size(depthMapCurGT,1) - marg);
    
    vldPtCur_ = sub2ind(size(depthMapCurGT), round(predPtList000(vldPrTrace,2)), round(predPtList000(vldPrTrace,1)));
    vldPtCur_depth = depthMapCurGT(vldPtCur_);
    
    vldPtCur(vldPrTrace) = vldPtCur_depth;
    
    % %     marg = 5;
    vldPtPrv = FindValidDepth(prevFeatPtList_22, depthMapPrvGT, marg);
    vldPtCur11 = FindValidDepth(predPtList000, depthMapCurGT, marg);
    
    
    depthMapPrvTmp = obj.prvDepthVisual; %depthMapKey_2; % keyFrameDepth;
    depthMapPrvTmp(depthMapPrvTmp == -1) = nan;
    
    depthMapCurTmp = obj.depthVisual;
    depthMapCurTmp(depthMapCurTmp == -1) = nan;
    
    vldPtPrv_tmp = FindValidDepth(prevFeatPtList_22, depthMapPrvTmp, marg);
    vldPtCur11_tmp = FindValidDepth(predPtList000, depthMapCurTmp, marg);
    
    if 0
        figure,plot(vldPtCur11(~isnan(vldPtCur11)) - vldPtCur(~isnan(vldPtCur)))
    end
    
    
    
    inTrackFlag000(isnan(vldPtPrv) | isnan(vldPtCur11) | isnan(vldPtPrv_tmp) | isnan(vldPtCur11_tmp)) = false;
    
    
    featPtList___1 = VisualLocalizer.DetectFast(obj.prevImgL);
    
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
        if newLK
            pixMarg = 80*scal; % 80*scal; % 40*scal;
            minNum = 100000; 100; 200; 100; 60; 10; 50; 50; 
        else
            pixMarg = 40*scal; % 80*scal; % 40*scal;
            minNum = 100; 100000; 100; 200; 100; 60; 10; 50; 50;
        end
        if p2cRef > 0
            validROI = find(featPtList___(:,1) < min(prevFeatPtList_22(:,1) + pixMarg));
%             if ~isempty(validROI)
            if length(validROI) > minNum*scal
                featPtList = featPtList___(validROI,:);
            else
                % featPtList = featPtList___;
                
                
                featPtList___1 = VisualLocalizer.DetectFast(obj.prevImgL, 0.02); % 0.01 0.02
                
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
                
                
                
                featPtList___1 = VisualLocalizer.DetectFast(obj.prevImgL, 0.02); % 0.01 0.02
                
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
    
    
    if 1 % ~PrvTrackOnGT
        if ~NewTracker
            if ~newLK
                [predPtList111, inTrackFlag111] = TrackFeaturePoints(obj.featPtTracker, obj.prevImgL, obj.currImgL, featPtList, [], intrMat, GetPctBody2Cam(obj.coordSysAligner));
            else
                [predPtList111, inTrackFlag111] = LKTracking(obj.prevImgL, obj.currImgL, featPtList, [],marg);
            end
        else
            
            if ~obj.switchDepth
                depthMapPrv__ = obj.prvDepthVisual;
            else
                depthMapPrv__ = obj.prvDepthGT;
            end
            ind_prv_pt = sub2ind(size(depthMapPrv__), round(featPtList(:,2)), round(featPtList(:,1)));
            prevFeatPtList_zz = depthMapPrv__(ind_prv_pt);
            if ~NewTracker2
                [predPtList111, inTrackFlag111, ~, ~, marg] = NewTrackingOld2(obj,  obj.prevImgL, obj.currImgL,size(featPtList, 1), featPtList, prevFeatPtList_zz, intrMat, double(p2cRef), AngleModalOrg);
            else
%                 [predPtList111, inTrackFlag111, ~, ~, marg] = NewTrackingOld2(obj,  obj.prevImgL, obj.currImgL,size(featPtList, 1), featPtList, prevFeatPtList_zz, intrMat, double(p2cRef), AngleModalOrg);
                [predPtList111, inTrackFlag111, ~, ~, marg] =  NewTrackingFinal(obj,  obj.prevImgL, obj.currImgL,size(featPtList, 1), featPtList, prevFeatPtList_zz, intrMat, double(p2cRef), AngleModalOrg);
            end
            
        end
    else 
        [predPtList111, inTrackFlag111] = TrackFeaturePoints(obj.featPtTracker, obj.prevImgL, obj.currImgL, featPtList, [], intrMat, GetPctBody2Cam(obj.coordSysAligner));
    end
    
    if  length(obj.keyFrameFlagList) - obj.addPt(end) < addPtInterval 
        inTrackFlag111 = false(length(inTrackFlag111), 1);
    else
        obj.addPt = [obj.addPt; length(obj.keyFrameFlagList)];
    end
    
    
    depthMapCurTmp2 = obj.depthVisual;
    depthMapCurTmp2(depthMapCurTmp2 == -1) = nan;
    
    vldPtCur_newAdd = FindValidDepth(predPtList111, depthMapCurGT, marg);
    vldPtCur_newAdd2 = FindValidDepth(predPtList111, depthMapCurTmp2, marg);
    
    inTrackFlag111(isnan(vldPtCur_newAdd) | isnan(vldPtCur_newAdd2)) = false;
    
    
    
    pt0 = predPtList000(inTrackFlag000,:);
    pt1 = predPtList111(inTrackFlag111,:);
    pt1Init = featPtList(inTrackFlag111,:);
    
    nearestIdx1 = knnsearch(pt0, pt1, 'NSMethod', 'kdtree');
    samePtFlag1 = VecNorm(pt1 - pt0(nearestIdx1, :), 2) <= 0.2.*scal; 0.4; % 0.2; %obj.featPtManager.configParam.radius_thresh_for_combine; %
    nearestIdx2 = knnsearch(pt1, pt0, 'NSMethod', 'kdtree');
    try
        samePtFlag2 = VecNorm(pt0 - pt1(nearestIdx2, :), 2) <= 0.2.*scal; 0.4;  % 0.2;% obj.featPtManager.configParam.radius_thresh_for_combine; %
    catch
        dfth = 1;
    end
    newAddedPtInit = pt1Init(~samePtFlag1, :);
    newAddedPt = pt1(~samePtFlag1, :);
    if 0
        figure,imshow(obj.currImgL);hold on;plot(pt1(:,1), pt1(:,2), 'xr');plot(pt0(:,1), pt0(:,2),'.g');
    end
    
    if 0
        figure(7),clf; imshow(obj.currImgL);hold on;plot(pt1(samePtFlag1,1), pt1(samePtFlag1,2), 'xr');plot(pt0(samePtFlag2,1), pt0(samePtFlag2,2),'.g');plot(newAddedPt(:,1), newAddedPt(:,2),'sb');drawnow;
    end
    
    
    if ~obj.switchDepth
        depthMapPrv = obj.prvDepthVisual; %depthMapKey_2; % keyFrameDepth;
        depthMapCur = obj.depthVisual;
    else
        depthMapPrv = obj.prvDepthGT; %depthMapKeyGT_2;
        depthMapCur = obj.depthGT;
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
    obj.traceManager.ZGT = [obj.traceManager.ZGT; -1.*ones(length(newAddedZGT), prvFrmNum)];
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
    
    
end




keyFrmLen = size(obj.featPtManager.localTrace.ptIcsX, 2);

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
        depthMapPrv__onlyP2 = obj.prvDepthVisual;
    else
        depthMapPrv__onlyP2 = obj.prvDepthGT;
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
    if length(obj.accumP2CRef) == 2
        validAng = find(abs(angDiffer) < 0.01);
        
    end
    inId0 = inId;
    if ReplaceOnlyP2
%         inId0 = inId;
        inId = inId(validAng);
        depthCurInPrvUse = depthCurInPrv(validAng);
        pix_z = [inId ptPrv(validAng,:) ptCur(validAng,:) angle_resultP2C(validAng) depthCurInPrvUse]; %featId  prv cur ang z
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
    figure,subplot(2,2,1), showMatchedFeatures(obj.prevImgL, obj.currImgL, prvPt, curPt);subplot(2,2,2), showMatchedFeatures(obj.prevImgL, obj.currImgL, prvPt(inFlag,:), curPt(inFlag,:));
    subplot(2,2,3);imshow(obj.prevImgL);hold on;plot(prvPt(inFlag,1),prvPt(inFlag,2),'.r');plot(prvPt(~inFlag,1),prvPt(~inFlag,2),'.g');
    subplot(2,2,4);imshow(obj.currImgL);hold on;plot(curPt(inFlag,1),curPt(inFlag,2),'.r');plot(curPt(~inFlag,1),curPt(~inFlag,2),'.g');
end








nearestKeyX = obj.traceManager.X(:, prvFrmNum + 1 - keyFrmLen + 1:end);
nearestKeyY = obj.traceManager.Y(:, prvFrmNum + 1 - keyFrmLen + 1:end);
nearestKeyZ = obj.traceManager.Z(:, prvFrmNum + 1 - keyFrmLen + 1:end);
featIdKey = find(nearestKeyX(:,end) > 0 & nearestKeyY(:,end) > 0 & nearestKeyZ(:,end) > 0 & nearestKeyX(:,1) > 0 & nearestKeyY(:,1) > 0 & nearestKeyZ(:,1) > 0);


depthMapKeyGT = obj.keyFrameDepthGT;
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
    
    obj.p2cTrackingErrWholeStack = [obj.p2cTrackingErrWholeStack; mean(errTracking2,1)];
    
    [~, er2] = NormalizeVector(prvPt(inFlag,:) - round(prvPt(inFlag,:)));
    
    idNewPixel2 = find(er2 == 0);
    
    
    
    
    
    
    depthStereo = zList(inFlag);
    dispStereo = (intrMat(1,1).*norm(baseline)./depthStereo(:,1)) - (princpPtR(1) - princpPtL(1));
    depthGt = zListGT(inFlag);
    dispGt  = (intrMat(1,1).*norm(baseline)./depthGt(:,1)) - (princpPtR(1) - princpPtL(1));
    [dispa,dispb] = hist(dispGt - dispStereo, 1000);
    dispErrExp = dot(dispb, (dispa./sum(dispa)));
    
    if isempty(obj.dispErrExpStack)
        obj.dispErrExpStack = [obj.dispErrExpStack; dispErrExp; dispErrExp];
    else
        obj.dispErrExpStack = [obj.dispErrExpStack; dispErrExp];
    end
    
    
    if 0
        if keyFrmLen == 2
            [~,er] = NormalizeVector(errTracking - errTracking2);
            figure,plot(er);
        end
    end
    
    jfhvkgh = 1;
    
    if 0
        if isempty(obj.trackingErrAccum2)
            figure(6);clf;
        end
        
        obj.trackingErrAccum2 = [obj.trackingErrAccum2; [[mean(errTracking(:,1)) mean(errTracking(:,2))] [mean(errTracking2(:,1)) mean(errTracking2(:,2))]]];
    else
        
        if keyFrmLen == 2
            obj.trackingErrAccum2 = [];
            figure(6);clf;
        end
    end
    if 0
        obj.trackingErrAccum2 = [obj.trackingErrAccum2; [[mean(errTracking(:,1)) mean(errTracking(:,2))] [mean(errTracking2(:,1)) mean(errTracking2(:,2))]]];
    else
        obj.trackingErrAccum2 = [obj.trackingErrAccum2; [[mean(errTracking(idNewPixel,1)) mean(errTracking(idNewPixel,2))] [mean(errTracking2(idNewPixel2, 1)) mean(errTracking2(idNewPixel2, 2))]]];
    end
    
    figure(6),
    
    % subplot(1,2,1);cla;imshow(obj.currImgL);hold on;plot(pt1(samePtFlag1,1), pt1(samePtFlag1,2), 'xr');plot(pt0(samePtFlag2,1), pt0(samePtFlag2,2),'.g');plot(newAddedPt(:,1), newAddedPt(:,2),'sb');drawnow;
    subplot(2,2,1);hold on;plot(errTracking(:,1), errTracking(:,2),'+');title('lk tracking - gt (k2c)');axis equal;
    % subplot(1,2,2);cla;hold on;plot(mean(errTracking(:,1)), mean(errTracking(:,2)),'o');title('lk tracking - gt'); axis equal;
    subplot(2,2,2);cla;hold on;plot((obj.trackingErrAccum2(:,1)), (obj.trackingErrAccum2(:,2)),'-o');title('lk tracking - gt (k2c)');% axis equal;
    for j = 1 : size(obj.trackingErrAccum2, 1)
        text(double(obj.trackingErrAccum2(j,1)),double(obj.trackingErrAccum2(j,2)),num2str(j), 'Color',[1 0 0],'FontSize',15);  %,'FontWeight','bold');
    end
    subplot(2,2,3);hold on;plot(errTracking2(:,1), errTracking2(:,2),'+');title('lk tracking - gt (p2c)');axis equal;
    % subplot(1,2,2);cla;hold on;plot(mean(errTracking(:,1)), mean(errTracking(:,2)),'o');title('lk tracking - gt'); axis equal;
    subplot(2,2,4);cla;hold on;plot((obj.trackingErrAccum2(:,3)), (obj.trackingErrAccum2(:,4)),'-o');title('lk tracking - gt (p2c)');% axis equal;
    for j = 1 : size(obj.trackingErrAccum2, 1)
        text(double(obj.trackingErrAccum2(j,3)),double(obj.trackingErrAccum2(j,4)),num2str(j), 'Color',[1 0 0],'FontSize',15);  %,'FontWeight','bold');
    end
    
    if 0 % 20200227 disable saving image
        saveas(gcf,fullfile(probPath,sprintf(strcat('err_',probPath(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('err_',probPath(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1 + FigBase)));
    end
    
else
    pt2d_check = [];
    dispErrExp = [];
    ljkblkjb = 1;
end

% errTracking2 = curPt(inFlag,:) - pixGT2;
cur_lk_gt = [ curPt(inFlag,:) pixGT2;];

try
    [LocalTraceList, unValidFeatId, validFeatId, traceLenList, dataTmp, dispErrMat, cur_id_lk_gt2, pixGT_temp_stack2] = PlotMultiTrace(obj, 8, obj.currImgL, obj.traceManager.X, obj.traceManager.Y, obj.traceManager.Z, obj.traceManager.ZGT, intrMat, princpPtL,princpPtR, baseline, obj.configParam, obj.colorMat, b2cPmat,obj.accumP2CRef, cur_lk_gt, pix_z);
catch
    hjdfuy = 1;
end

obj.traceManager.X(unValidFeatId,end) = -1;
obj.traceManager.Y(unValidFeatId,end) = -1;
obj.traceManager.Z(unValidFeatId,end) = -1;
obj.traceManager.ZGT(unValidFeatId,end) = -1;


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
 obj.traceInfoMat{1,4} = cur_id_lk_gt2;
 obj.traceInfoMat{1,5} = pixGT_temp_stack2;
 obj.traceInfoMat{1,6} = obj.accumP2CPNP2;
 obj.traceInfoMat{1,7} = obj.accumP2CPNP2_2;
 obj.traceInfoMat{1,8} = obj.traceManager;
 obj.traceInfoMat{1,9} = b2cPmat;
 
jhghk = 1;










if 0
    
    [prevFeatPtList0_2, ptCcsZ0_2] = GetActiveFeatures2(obj.featPtManager,1);
    
    
    featPtListKey = [obj.featPtManager.localTrace2.ptIcsX(:,1) obj.featPtManager.localTrace2.ptIcsY(:,1)];
    featPtList = [obj.featPtManager.localTrace2.ptIcsX(:,end) obj.featPtManager.localTrace2.ptIcsY(:,end)];
    %                                                     validPt2 = (featPtList(:,1) > 0 & featPtList(:,2) > 0);
    
    [prevFeatPtList_22, ptCcsZ_22] = GetActiveFeatures2(obj.featPtManager);
    if ~obj.switchDepth
        depthMapKey2 = obj.trace2.keyFrameDepth; %depthMapKey_2; % keyFrameDepth;
    else
        depthMapKey2 = obj.trace2.keyFrameDepthGT; %depthMapKeyGT_2;
    end
    validPt1 = (featPtListKey(:,1) > 0 & featPtListKey(:,2) > 0);
    inValid___ = sub2ind(size(depthMapKey2), round(featPtListKey(validPt1,2)), round(featPtListKey(validPt1,1)));
    
    inValid__ = ones(size(featPtListKey,1),1);
    inValid__(validPt1) = inValid___;
    depthList__ = depthMapKey2(inValid__);
    depthList__(~validPt1) = -1;
    % validPt2 = (featPtList(:,1) > 0 & featPtList(:,2) > 0 & depthList__ > 0);
    
    
    pnp_ang_est_max_margin = [deg2rad([-1 1]) obj.pureRotationAngPredictor.configParam.pure_rot_reproject_outlier_threshold];
    
    if ~NewTracker
        [predPtList000, inTrackFlag000] = TrackFeaturePoints(obj.featPtTracker, imgPrvL_2, imgCur_2, prevFeatPtList_22, [], intrMat, GetPctBody2Cam(obj.coordSysAligner));
    else
        [predPtList000, inTrackFlag000, ~, ~, marg] = NewTrackingOld2(obj, size(prevFeatPtList_22, 1), prevFeatPtList_22, prevFeatPtList_22_z, intrMat, double(p2cRef), AngleModalOrg);
    end
    
    
    ExtendLocalTrace2(obj.featPtManager, predPtList000, inTrackFlag000);
    
    
    id_ = find(ismember(featPtList, prevFeatPtList_22, 'rows'));
    inTrackFlag00 = false(size(featPtList,1),1);
    inTrackFlag00(id_(inTrackFlag000)) = true;
    
    predPtList = -1.*ones(size(featPtList,1),2);
    predPtList(id_(inTrackFlag000),:) = predPtList000(inTrackFlag000,:);
    
    
    
    [k2cBodyRotAng_2,Err_2,inId] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin,intrMat,featPtListKey(inTrackFlag00,:),predPtList(inTrackFlag00,:),depthList__(inTrackFlag00,:), find(inTrackFlag00),true(sum(inTrackFlag00),1),b2c,double(k2cRef_2));
    
    diffVec = setdiff([1:size(obj.featPtManager.localTrace2.ptIcsX,1)]', inId);
    obj.featPtManager.localTrace2.ptIcsX(diffVec,end) = -1;
    obj.featPtManager.localTrace2.ptIcsY(diffVec,end) = -1;
end
end
function [LocalTraceList, unValidFeatId, validFeatId, traceLenList, data, dispErrMat, cur_id_lk_gt2, pixGT_temp_stack2] = PlotMultiTrace(obj, figNum, img, localX, localY, localZ, localZGT, intrMat, princpPtL,princpPtR, baseline, ConfigParam, colorMat, b2cPmat, accumP2CRef, cur_lk_gt, pix_z)
global newRender ShiftLK newShift ReplaceOnlyP2 ReplaceOnlyP2Z

% newShift = true;



if 0
    intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,obj.scaleLvl);
    [princpPtL, princpPtR] = Get(obj.camModel, 'PinholePrincpPt', obj.scaleLvl);
    baseline = obj.camModel.transVec1To2;
end


angP2CWhole = obj.accumP2CPNP2(end) - obj.accumP2CPNP2(end-1);
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
figure(figNum); imshow(img);hold on;

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
    numThr = 16 + 1; % 18+1; % 17+1; 17*3+1; 14*3 + 1;
else
    numThr = 16*3+1;  %17*3+1;
end

pixGT_temp_stack = [];
pixGT_temp_stack2 = {};

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
        
        if length(inId) < 30 
            [k2cBodyRotAng_2,Err_2,inId] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin_large,intrMat,[localTraceX0(inTrackFlag00,1) localTraceY0(inTrackFlag00,1)],[localTraceX0(inTrackFlag00,end) localTraceY0(inTrackFlag00,end)],localTraceZ0(inTrackFlag00,1), find(inTrackFlag00),true(sum(inTrackFlag00),1),b2c,double(gtangle(end)));
            
        end
        
        
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
        
        plot(localTraceX', localTraceY', '-', 'Color', colorMat(i,:));
        
        
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
    pix_z_use = pix_z(ismember(pix_z(:,1), featIdAll), :); %featId  prv cur ang z
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

if ShiftLK
    Er = []; TempCurPt_2 = [];
    for j = 1 : length(LocalTraceList)
        
        tempTrace = LocalTraceList{j, 1};
        tempTrace_bak = tempTrace;
        tempFeatId = pixGT_temp_stack2{j,1};
        temp_cur_lk_gt = pixGT_temp_stack2{j, 2};
        tempCurPt = [tempTrace.ptIcsX(:,end) tempTrace.ptIcsY(:,end)];
        if 0
            er = temp_cur_lk_gt(:,1:2) - tempCurPt;
        end
        
        if ~newShift
            tempTrace.ptIcsX(:,end) = temp_cur_lk_gt(:,3) + wholeErrCurMean2(:,1);
            tempTrace.ptIcsY(:,end) = temp_cur_lk_gt(:,4) + wholeErrCurMean2(:,2);
        else
            
            if 1
                if ~ReplaceOnlyP2 || length(obj.accumP2CRef) == 2
                    [tempCurPt_2_newShift] = VisualLocalizer.GetGtTrace2(b2cPmat, double(angP2CWhole), [temp_cur_lk_gt(:,5:6)],temp_cur_lk_gt(:,7),intrMat);
                else
                    if ReplaceOnlyP2Z
                        [tempCurPt_2_newShift] = VisualLocalizer.GetGtTrace2(b2cPmat, double(angP2CWhole), [temp_cur_lk_gt(:,5:6)],pix_z_use(ismember(pix_z_use(:,1),tempFeatId),7),intrMat);
                    else
                        [tempCurPt_2_newShift] = VisualLocalizer.GetGtTrace2(b2cPmat, double(angP2CWhole), [temp_cur_lk_gt(:,5:6)],temp_cur_lk_gt(:,7),intrMat);
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
        
        tempCurPt_2 = [tempTrace.ptIcsX(:,end) tempTrace.ptIcsY(:,end)];
        TempCurPt_2 = [TempCurPt_2; tempCurPt_2];
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
         end
        
        obj.traceManager.XLK(tempTrace.featId, length(obj.accumP2CRef)) = LocalTraceList{j,1}.ptIcsXOrig(:,end);
        obj.traceManager.YLK(tempTrace.featId, length(obj.accumP2CRef)) = LocalTraceList{j,1}.ptIcsYOrig(:,end);
        obj.traceManager.ZLK(tempTrace.featId, length(obj.accumP2CRef)) = LocalTraceList{j,1}.ptCcsZOrig(:,end);
        obj.traceManager.ZGTLK(tempTrace.featId, length(obj.accumP2CRef)) = LocalTraceList{j,1}.ptCcsZGTOrig(:,end);
        
        
    end
    
    assert(max(ZErr34) == 0);
    
    er_ = Er - wholeErrCurMean2;
    
    %     pnp_ang_est_max_margin_2 = [deg2rad([-0.05 0.05]) 0.5];
    %     roundingPrecision = 0.00000001;
    %     p2cRef__ = deg2rad(round(rad2deg(accumP2CRef(end) - accumP2CRef(end-1))./roundingPrecision).*roundingPrecision);
    %     [p2cBodyRotAng_2,Err_2,inId___2, reProjErr___2] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin_2,intrMat,cur_lk_gt2(:,5:6),cur_lk_gt2(:,1:2),cur_lk_gt2(:,7), [1:size(cur_lk_gt2,1)]',true(size(cur_lk_gt2,1),1),b2c,double(p2cRef__),0.00001/2);
    [p2cBodyRotAng_22,Err_22,inId___22, reProjErr___22] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin_2,intrMat,cur_lk_gt2(:,5:6),TempCurPt_2(:,1:2),cur_lk_gt2(:,7), [1:size(cur_lk_gt2,1)]',true(size(cur_lk_gt2,1),1),b2c,double(p2cRef__),0.00001/2);
    
    
    if ReplaceOnlyP2
        p2cBodyRotAng_22_onlyP2 = OnlyP2_ang(intrMat, cur_lk_gt2(:,5:6), TempCurPt_2, r_cam,tx, ty, tz);
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

if 0
    featSizeSum = cumsum(featSize);
    % sum_c2c_All =
    figure(300);clf;
    for k = 1 : length(featSize)
        
        if k == 1
            dispTemp = dd(1:featSizeSum(1),:);
            dispGTTemp = ddGT(1:featSizeSum(1),:);
        else
            dispTemp = dd(featSizeSum(k-1)+1:featSizeSum(k),:);
            dispGTTemp = ddGT(featSizeSum(k-1)+1:featSizeSum(k),:);
        end
        
        startId = find(dispTemp(1,:) ~= -1);
        
        
        if k == 1
            figRow = length(featSize); figCol = length(startId);
            ofst  = startId(1)-1;
            sum_c2c_All = zeros(length(disparityRng)*length(startId), length(disparityRng)*length(featSize));
        end
        
        
        dispTemp = dispTemp(:,startId);
        dispGTTemp = dispGTTemp(:,startId);
        sum_c2c_all = [];
        for kk = 1 : size(dispTemp,2)
            
            dispCurList_ = dispTemp(:,kk);
            dispCurList_gt = dispGTTemp(:,kk);
            
            DispRngC2C = dispCurList_(:) + disparityRng;
            
            DispRngStepC2C = mean(diff(DispRngC2C'))';
            [~, depthGTIndAll_cur] = RoundingDispErr2(DispRngC2C(:,(size(DispRngC2C,2)+1)/2),dispCurList_gt, DispRngStepC2C,DispRngC2C);
            try
                depthGTIndAll_cur(isnan(depthGTIndAll_cur)) = 1;
            catch
                asdfgk = 1;
            end
            ProbZTmp_update_norm_sum_tmp_c2c = zeros(size(DispRngC2C));
            %                     ProbZTmp_update_norm_sum_tmp(depthGTInd11) = ProbZTmp_update_norm_sum(depthGTInd11);
            ProbZTmp_update_norm_sum_tmp_c2c(depthGTIndAll_cur) = 1;
            sum_c2c = sum(ProbZTmp_update_norm_sum_tmp_c2c);
            
            subplot(figRow, figCol, (k-1)*figCol + startId(kk) - ofst);plot(disparityRng, sum_c2c);grid on;
            if kk == 1
               title() 
            end
            sum_c2c_ = repmat(sum_c2c,length(sum_c2c), 1);
            
            sum_c2c_all = [sum_c2c_all; sum_c2c_];
        end
        try
            sum_c2c_All(end - size(sum_c2c_all,1)+1 : end, length(disparityRng)*(k-1)+1 : length(disparityRng)*(k)) = sum_c2c_all;
        catch
            asdgfk = 1;
        end
    end
    
end
data.featSize = featSize;
data.dd = dd;
data.ddGT = ddGT;
data.disparityRng = disparityRng;

dsfagk = 1;
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

function vldPtCur = FindValidDepth(predPtList000, depthMapCurGT, marg)

vldPtCur = nan(size(predPtList000,1),1);
vldPrTrace = find(predPtList000(:,1) > marg & predPtList000(:,2) > marg & predPtList000(:,1) < size(depthMapCurGT,2) - marg & predPtList000(:,2) < size(depthMapCurGT,1) - marg);

vldPtCur_ = sub2ind(size(depthMapCurGT), round(predPtList000(vldPrTrace,2)), round(predPtList000(vldPrTrace,1)));
vldPtCur_depth = depthMapCurGT(vldPtCur_);

vldPtCur(vldPrTrace) = vldPtCur_depth;


end
function [depthGTInd11, depthGTIndAll] = RoundingDispErr2(DispRngCenter,disp2round, DispRngStep,DispRng)
disparityError = DispRngCenter - disp2round;

disparityErrorRound = round(disparityError./DispRngStep).*DispRngStep;


depthGTOfst11 = round(-(disparityErrorRound)./DispRngStep);
depthGTInd11 = depthGTOfst11 + (size(DispRng,2)+1)/2;
depthGTInd11(depthGTInd11 < 1) = 1;
depthGTInd11(depthGTInd11 > size(DispRng,2)) = size(DispRng,2);

depthGTIndAll = sub2ind(size(DispRng),[1:size(DispRng,1)]', depthGTInd11);


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
function ang = OnlyP2_ang(intrMat, ptPrv, ptCur, r_cam,tx, ty, tz)
ptPrv_norm = inv(intrMat)*pextend(ptPrv');
ptCur_norm = inv(intrMat)*pextend(ptCur');
[angle_resultP2C,depthCurInPrv] = VisualLocalizer.AnglePredict_Prob(ptPrv_norm,ptCur_norm,r_cam,tx, ty, tz);
[angle_resultP2C1,~] = VisualLocalizer.AnglePredict(ptPrv_norm,ptCur_norm, tx, ty, tz);
angDiffer = angle_resultP2C - angle_resultP2C1';
validAng = find(abs(angDiffer) < 0.01);
ang = deg2rad(mean(angle_resultP2C(validAng)));
end