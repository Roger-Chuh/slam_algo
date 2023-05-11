function [angOpt, inlierId, angRng, PixUse, pt2dCurUse, DispRefineUse_, angOpt3, inFlag,thetaRngOut,thetaProbOut,angOptP2C, angOptK2C_, p2cOnlyP2_111,...
    keyFrameLen ,...
    disparity_resample_prob_threshold,...
    ThetaNoise ,...
    KeyFrameFlagList,...
    ConfigParam ,...
    imgCur ,...
    imgCurR,...
    imgKeyL,...
    imgKeyR,...
    imgPrvL,...
    imgPrvR,...
    LocalTrace,...
    KeyProbZ,...
    thetaRngMat,...
    thetaProbMat,...
    CamModel ,...
    CoordSysAligner,...
    REFAngList ,...
    REFAngList3,...
    REFAngList4,...
    accumP2CTemp,...
    twoOnlyP2List,...
    thetaPlatform,...
    thetaPlatformDistribution,...
    poseWcsList,...
    ANGOpt,...
    ANGRng,...
    meanErrAndStd,...
    p2cErrList,...        
    objPervPtCcsZGolden1] = RefineZ_bak_before_multiFrame_data(k2cRef_00, obj, k2cRef,k2cPnp, inlierId0, inlierId2, theta, DepthProbility,  DepthId,  AngleModalOrg,  AngleProbility,prevFeatPtList0,ptCcsZ0,DispRefineUse__,NewDispRng,newDispStep,ProbZTmpReSamp,k2cBody,...
    keyFrameLen ,...
    disparity_resample_prob_threshold,...
    ThetaNoise ,...
    KeyFrameFlagList,...
    ConfigParam ,...
    imgCur ,...
    imgCurR,...
    imgKeyL,...
    imgKeyR,...
    imgPrvL,...
    imgPrvR,...
    LocalTrace,...
    KeyProbZ,...
    thetaRngMat,...
    thetaProbMat,...
    CamModel ,...
    CoordSysAligner,...
    REFAngList ,...
    REFAngList3,...
    REFAngList4,...
    accumP2CTemp,...
    twoOnlyP2List,...
    thetaPlatform,...
    thetaPlatformDistribution,...
    poseWcsList,...
    ANGOpt,...
    ANGRng,...
    meanErrAndStd,...
    p2cErrList,...
    objPervPtCcsZGolden)  %, XYZ)

global SHOWPOLYGON DRAWPOLYGON ANGLEONLY USEGOLDENDISP UPDATEDEPTH DEPTHITERNUM probPath ...
    USELEFT USELEFTPLUSRIGHT USELEFTXRIGHT USERIGHT WITHINEPILINE BOTHLR HARDCUT QUANTCUT FUSIONCUT ...
    FORCEEVENPLATFORM CUTTHETAANDFORCEPLATFORM DIFFTHETATHR CUTTHETAANDFORCEPLATFORM_NODIFF FORCEEXPEVEN ...
    EXPZERO GTTRACKING SOFTP2 Rati Rati1 Rati2 Sc USEthetaProb0 ONLYP2 ONLYP2_2 UPDATEZ_2 UPDATETracking_2 ...
    Sc1 Sc2 FORCETHETAPLATFORM1 FORCETHETAPLATFORM2 ONLYP2_1_Max NewOnlyP2 UPDATETracking_2_2 UpdateP2_3 ...
    doRoundingTheta doRoundingRefAngList3 UsePrvDepth InheriDispRng EvenDepthProb roundingPrecision IterThetaNum ...
    NewPrvDispResamp Rati_onlyP2 NewFlow UseNewP2Only doRoundingTheta2 roundingPrecision2 UseNewP2CRef ...
    ReplaceK2CRefInKey UseGoldenThetaP2C FrmNumWithGloden ForceThetaExpZero ResampZThr1 ResampZThr2 ...
    ProbZCutOffThr FigBase

inlierId0 = LocalTrace.featId; %[1:size(LocalTrace.ptIcsX,1)]';
inlierId = inlierId0;
if keyFrameLen <= FrmNumWithGloden + 1
    doRoundingTheta = false; true; false;
    UseGoldenThetaP2C = true;
    Rati1 = 0.8; 0; 0.8; 0; 0.5; 0.8; 0; 0.5; 0.8; 0.8; 0.8; 0; 0.4; 0.3; 0.5; 0; 0.8; 0.0001; 0.95;
    Rati2 = 0.8;
    UsePrvDepth = false; false; true; false; true; false; true;
    NewPrvDispResamp = false;
    
    EvenDepthProb = false; false;true; false; true;
    InheriDispRng = true;
    
    
    ConfigParam.disparity_resample_prob_threshold = ResampZThr1;
    
    %                 UpdateP2_3 = false;
    
    if 0
        ForceThetaExpZero = true;
    else
        ForceThetaExpZero = false;
    end
    
else
    if 0
        doRoundingTheta = true;
        UseGoldenThetaP2C = false;
        Rati1 = 0; 0.8; 0; 0.8; 0; 0.5; 0.8; 0; 0.5; 0.8; 0.8; 0.8; 0; 0.4; 0.3; 0.5; 0; 0.8; 0.0001; 0.95;
        Rati2 = 0; 0.8;
        UsePrvDepth = true; false; true; false; true; false; true;
        NewPrvDispResamp = false;
        
        EvenDepthProb = true;true; false; false;true; false; true;
        InheriDispRng =  false; true;
    else
        doRoundingTheta = true;
        UseGoldenThetaP2C = false;
        
        Rati1 = 0.8; 0; 0.8; 0; 0.5; 0.8; 0; 0.5; 0.8; 0.8; 0.8; 0; 0.4; 0.3; 0.5; 0; 0.8; 0.0001; 0.95;
        Rati2 = 0.8;
        UsePrvDepth = false; false; true; false; true; false; true;
        NewPrvDispResamp = false;
        
        EvenDepthProb = false; false;true; false; true;
        InheriDispRng = true;
        
        
    end
    ConfigParam.disparity_resample_prob_threshold = ResampZThr2;
    
    %                 UpdateP2_3 = true;
    
    ForceThetaExpZero = false;
    
    
    
end


k2cRef0 = k2cRef;
if doRoundingTheta
    %                 k2cRef = deg2rad(round(rad2deg(k2cRef),1));
    k2cRef = deg2rad(round(rad2deg(k2cRef0 + ThetaNoise)./roundingPrecision).*roundingPrecision);
    
    
    %                 k2cRef = k2cRef0 + deg2rad(0.1*(-1)^(keyFrameLen)) + deg2rad(0.01*(rand(1)-0.5));
    
    if 0
        if ismember(keyFrameLen, [2 5 8 11 14 17] - 0)
            k2cRef = k2cRef0 +  0 + deg2rad(0.01*(rand(1)-0.5));
        elseif ismember(keyFrameLen, [3 6 9 12 15 18])
            k2cRef = k2cRef0  -  deg2rad(0.15*(1)^(keyFrameLen)) + deg2rad(0.01*(rand(1)-0.5));
        elseif ismember(keyFrameLen, [4 7 10 13 16 19])
            k2cRef = k2cRef0  +  deg2rad(0.15*(1)^(keyFrameLen)) + deg2rad(0.01*(rand(1)-0.5));
        elseif 0 % ismember(keyFrameLen, [5 9 13 17 21])
            k2cRef = k2cRef0  +  deg2rad(0.1*(1)^(keyFrameLen)) + deg2rad(0.01*(rand(1)-0.5));
        end
        
        
    elseif 0
        
        %                     k2cRef = k2cRef0  -  deg2rad(0.15*(-1)^(keyFrameLen)) + deg2rad(0.01*(rand(1)-0.5));
        %                     k2cRef = k2cRef0  -  deg2rad(0.15*(-1)^(size(KeyFrameFlagList, 1))) + deg2rad(0.01*(rand(1)-0.5));
        k2cRef = k2cRef0  -  deg2rad(0.15*(-1)^(sum(KeyFrameFlagList) + keyFrameLen)) + deg2rad(0.01*(rand(1)-0.5));
        
    end
    
    
    
    
end


if ReplaceK2CRefInKey
    if UseGoldenThetaP2C
        if  keyFrameLen == 2
            k2cRef = k2cRef0;
        end
        
        
    end
end




k2cRefBak = k2cRef;

%             [k2cRef, ~, ~] = AngleSpace.EpiPolarThreeView(obj, k2cRef, k2cRef, [deg2rad(-0.2) : ConfigParam.theta_sample_step : deg2rad(0.2)], Pix, Pix, pt2dCur, intrMat, b2c);







if keyFrameLen > 2
    ConfigParam.reproj_sigma = ConfigParam.reproj_sigma0 * ConfigParam.reproj_sigma_scale;
    ConfigParam.reproj_sigma_right = ConfigParam.reproj_sigma_right0 * ConfigParam.reproj_sigma_scale_right;
else
    ConfigParam.reproj_sigma = ConfigParam.reproj_sigma0; %  * ConfigParam.reproj_sigma_scale;
    ConfigParam.reproj_sigma_right = ConfigParam.reproj_sigma_right0; % * ConfigParam.reproj_sigma_scale_right;
end




[princpPtL, princpPtR] = Get(CamModel, 'PinholePrincpPt', obj.scaleLvl);
L2R = [rodrigues(CamModel.rotVec1To2) CamModel.transVec1To2; 0 0 0 1];

intrMat = Get(CamModel, 'PinholeIntrMat', 1,obj.scaleLvl);

if 0
    depthMapCurVisual; %  = depthVisual;
    depthMapCurGT; %  = depthGT;
    
    if USEGOLDENDISP
        depthMapCur = depthMapCurGT;
    else
        depthMapCur = depthMapCurVisual;
    end
    
    dispMapCur = (intrMat(1,1).*norm(CamModel.transVec1To2)./depthMapCur) - (princpPtR(1) - princpPtL(1));
    
    dispMapCurGT = (intrMat(1,1).*norm(CamModel.transVec1To2)./depthMapCurGT) - (princpPtR(1) - princpPtL(1));
    
    dispMapCur(dispMapCur < 0) = nan;
    depthMapCur(depthMapCur < 0) = nan;
end
if 0
    imgCur; % = imgCur;
    imgCurR; % = currImgR;
    
    
    imgKeyL; % = keyFrameImgL;
    imgKeyR; % = keyFrameImgR;
    imgPrvL; % = imgPrvL;
    imgPrvR; % = prevImgR;
    
    
    
    [depthMapKeyVisual, dispMapKey] = GetDepthMap(obj, imgKeyL, imgKeyR);
    
    depthMapKey; %  = keyFrameDepth;
end
% % % % % try
% % % % %     inlierIdPrv = keyProbZ{end,2};
% % % % % catch
% % % % %     asg = 1;
% % % % % end
if keyFrameLen > 2
    
    inlierIdPrv = KeyProbZ{end,2}; %keyProbZ{end,2};
%     inlierIdPrv = 1;
    if 0
        pixKey = [LocalTrace.ptIcsX(inlierIdPrv,1) LocalTrace.ptIcsY(inlierIdPrv,1)];
    else
        pixKey = [LocalTrace.ptIcsX(:,1) LocalTrace.ptIcsY(:,1)];
    end
    if 0
        inlierIdPrvInd = sub2ind(size(dispMapKey), round(pixKey(:,2)), round(pixKey(:,1)));
        dispMapKey(inlierIdPrvInd) = NewDispRng(:,(size(NewDispRng,2)+1)/2);
        depthMapKeyVisual(inlierIdPrvInd) = (intrMat(1,1).*norm(CamModel.transVec1To2))./(dispMapKey(inlierIdPrvInd) + (princpPtR(1) - princpPtL(1)));
        depthMapKey(inlierIdPrvInd) = (intrMat(1,1).*norm(CamModel.transVec1To2))./(dispMapKey(inlierIdPrvInd) + (princpPtR(1) - princpPtL(1)));
        %     keyFrameDepth(inlierIdPrvInd) = (intrMat(1,1).*norm(CamModel.transVec1To2))./(dispMapKey(inlierIdPrvInd) + (princpPtR(1) - princpPtL(1)));
        % %     %                 keyFrameDepthGT(inlierIdPrvInd) = (intrMat(1,1).*norm(CamModel.transVec1To2))./(dispMapKey(inlierIdPrvInd) + (princpPtR(1) - princpPtL(1)));
        LocalTrace.ptCcsZ(inlierIdPrv) = (intrMat(1,1).*norm(CamModel.transVec1To2))./(dispMapKey(inlierIdPrvInd) + (princpPtR(1) - princpPtL(1)));
    else
        commonId = find(ismember(inlierIdPrv, inlierId));
        dispCen = NewDispRng(:,(size(NewDispRng,2)+1)/2);
        LocalTrace.ptCcsZ(:,1) = (intrMat(1,1).*norm(CamModel.transVec1To2))./(dispCen(commonId) + (princpPtR(1) - princpPtL(1)));
        
    end
    sdgaf = 1;
end


if keyFrameLen == 2
    thetaRngMat = [];
    thetaProbMat = [];
end

if 0
    thetaSamp = [ConfigParam.theta_range(1) : ConfigParam.theta_sample_step : ConfigParam.theta_range(2)];
    thetaSamp0 = thetaSamp;
    thetaRange = k2cRef + [ConfigParam.theta_range(1) : ConfigParam.theta_sample_step : ConfigParam.theta_range(2)];
    thetaRng = thetaRange;
end
%             [probTheta, ~] = probDensity(k2cRef, ConfigParam.theta_sigma, thetaRng,ConfigParam.theta_sample_step, 'theta');

disparityRng = [-ConfigParam.disparity_error : ConfigParam.disparity_sample_step : ConfigParam.disparity_error];

if 0
    [thetaGrid, disparityGrid] = meshgrid(thetaRange, disparityRng);
    ThetaDisp = [thetaGrid(:) disparityGrid(:)];
end






if 0
    depthMapKey(dispMapKey < 0) = -1;
    dispMapKey(dispMapKey < 0) = nan;
    
    
    
    depthErrMap = zeros(size(dispMapKey));
    
    validInd = sub2ind(size(dispMapKey), round(LocalTrace.ptIcsY(:,1)), round(LocalTrace.ptIcsX(:,1)));
    validInd_curr = sub2ind(size(dispMapKey), round(LocalTrace.ptIcsY(:,end)), round(LocalTrace.ptIcsX(:,end)));
end
if 0
    dispList = dispMapKey(validInd);
else
    dispList = LocalTrace.dispList(:,1);
end

if 0
    dispMapCurGTList0 = dispMapCurGT(validInd);
end

if 0
    depListVisual = depthMapKeyVisual(validInd);
else
    depListVisual = LocalTrace.dispList(:,1);
end
if 0
    depListGT = depthMapKey(validInd);
    depthMapKeyGT;%   = keyFrameDepthGT;
    depListGT = depthMapKeyGT(validInd);
    dispMatGT = (intrMat(1,1).*norm(CamModel.transVec1To2)./depthMapKeyGT) - (princpPtR(1) - princpPtL(1));
    dispMatGT0 = dispMatGT;
    
    dispListGT = dispMatGT0(validInd);
    depthListGT = depthMapKeyGT(validInd);
else
    depListGT = LocalTrace.ptCcsZGT(:,1);
    dispListGT = LocalTrace.dispListGT(:,1);
    depthListGT = depListGT;
end
%             if USEGOLDENDISP
%                 dispList = dispListGT;
%             end
if 0
    dispCurList_ = dispMapCur(validInd_curr);
else
    if USEGOLDENDISP
        dispCurList_ = LocalTrace.dispListGT(:,end);
    else
        dispCurList_ = LocalTrace.dispList(:,end);
    end
end
if keyFrameLen > 2
    if 0
        validIndPrv = zeros(length(depListVisual),1);
        validPrv = find(LocalTrace.ptIcsX(:,end-1) > 0 & LocalTrace.ptIcsY(:,end-1) > 0);
        validIndPrvValid = sub2ind(size(dispMapKey), round(LocalTrace.ptIcsY(validPrv,end-1)), round(LocalTrace.ptIcsX(validPrv,end-1)));
        
        if obj.switchDepth
            depthMapPrv = prvDepthGT;
        else
            depthMapPrv = prvDepthVisual;
        end
        vldPrvId = find(depthMapPrv(validIndPrvValid) ~= -1);
        validPrv = validPrv(vldPrvId);
        %                 validIndPrvValid = validIndPrvValid(intersect(validPrv, find(depthMapPrv(validIndPrvValid) ~= -1)));
        dispMapPrv = (intrMat(1,1).*norm(CamModel.transVec1To2)./depthMapPrv) - (princpPtR(1) - princpPtL(1));
        %                 dispPrvList_ = dispMapPrv(validIndPrvValid);
        dispPrvList_ = nan(length(dispCurList_),1); depthPrvList_ = nan(length(dispPrvList_),1);
        dispPrvList_(validPrv) = dispMapPrv(validIndPrvValid(vldPrvId));
        depthPrvList_(validPrv) = intrMat(1,1).*norm(CamModel.transVec1To2)./( dispPrvList_(validPrv) + (princpPtR(1) - princpPtL(1)));
    end
    
    if obj.switchDepth
        dispPrvList_ = LocalTrace.dispListGT(:,end-1);
        depthPrvList_ = LocalTrace.ptCcsZGT(:,end-1);
    else
        dispPrvList_ = LocalTrace.dispList(:,end-1);
        depthPrvList_ = LocalTrace.ptCcsZ(:,end-1);
    end
else
    dispPrvList_ = dispCurList_;
end

if 0
    dispMatGT(dispMatGT > max(dispMapKey(validInd)) + 3) = nan;
    %             dispMatGT(dispMatGT < 0.9) = 0.9;
    dispMatGT(dispMatGT < 0.0001) = 0.0001;
end
if USEGOLDENDISP
    if 0
        dispList = dispMatGT(validInd);
    else
        dispList = LocalTrace.dispListGT(:,1);
    end
else
    %                 if 1 %keyFrameLen == 2
    % %                     figure(FigBase + FigBase + 38),clf;plot(dispList - dispMatGT(validInd));title('stereoDisp - gtDisp');
    %                    dispGTTmp = dispMatGT(validInd);
    %                    figure(FigBase + FigBase + 38),clf;plot(dispList(inlierId0) - dispGTTmp(inlierId0));title('stereoDisp - gtDisp');
    %                 end
    if keyFrameLen == 2
        %                     figure(FigBase + FigBase + 38),clf;plot(dispList - dispMatGT(validInd));title('stereoDisp - gtDisp');
        probDir = probPath;  % fullfile(pwd, 'prob');
        if 0
            dispGTTmp = dispMatGT(validInd);
        else
            dispGTTmp = LocalTrace.dispListGT(:,1);
            
        end
        figure(FigBase + FigBase + 38),clf;subplot(1,3,1);hold on;plot(dispList(:) - dispGTTmp(:));title('stereoDisp - gtDisp');subplot(1,3,2);imshow(imgCur);hold on;
        %                     saveas(gcf,fullfile(probDir,sprintf('disp_%05d.png',length(dir(fullfile(probDir,'disp_*.png')))+2)));
        %                         saveas(gcf,fullfile(probDir,sprintf('hist_%05d.fig',length(dir(fullfile(probDir,'hist_*.fig')))+1)));
        
    end
end

if 0
    dispErr = dispMapKey - dispMatGT;
    %                     dispErr(isnan(dispErr)) = 0;
    dispErrList = dispErr(validInd);
end
%
zList = LocalTrace.ptCcsZ(:,1);


baseline = norm(CamModel.transVec1To2);
b2c = CoordSysAligner.pctBody2Cam(1, 1).transformMat;

T_B2C = b2c;


r_cam = sqrt(T_B2C(1,4)^2+T_B2C(2,4)^2+T_B2C(3,4)^2);
tx = T_B2C(1,4)/r_cam;
ty = T_B2C(2,4)/r_cam;
tz = T_B2C(3,4)/r_cam;





reprojError = [];cntt = 0;PixCur = [];PixReproj = [];
validId = []; validXYZAll = []; Pix0 = []; vldReproj = [];
% keyLength = size(LocalTrace.ptIcsX,2);
if 0
    dispGTTmp = dispMatGT(validInd);
else
    dispGTTmp = LocalTrace.dispListGT(:,1);
end

if 0 % 20191216
    idUnValid = find(isnan(zList) | isinf(zList) | isnan(dispList) | isinf(dispList) | isnan(dispCurList_) | isnan(dispGTTmp)) ; % | isnan(dispGTTmp) | isinf(dispGTTmp));
elseif 0
    idUnValid = find(isnan(zList) | isinf(zList) | isnan(dispList) | isinf(dispList) | isnan(dispCurList_) | isnan(dispGTTmp) | isnan(dispPrvList_) ); % | isnan(dispGTTmp) | isinf(dispGTTmp));
else
    jkzvswa = 1;
end
if 0
    inlierId = setdiff(inlierId0, idUnValid);
end
if 0
    if ~isempty(LocalTrace.probZ)
        as = find(max(LocalTrace.probZ' > 0.000001)');
        inlierId = intersect(inlierId,as);
    end
end
if 0 % 20191216
    if keyFrameLen > 2
        inlierId = intersect(inlierId, inlierIdPrv);
    end
end
if keyFrameLen == 2
    curDispStepList = repmat(ConfigParam.disparity_sample_step,length(inlierId),1);
end







dispMapCurGTList = LocalTrace.dispListGT(:,end); %dispMapCurGTList0(inlierId,:);
%             pixGT = pixGT0(ismember(inlierId0, inlierId),1:2);




reprojError = [];cntt = 0;PixCur = [];PixReproj = [];
validId = []; validXYZAll = []; Pix0 = []; vldReproj = [];
keyLength = size(LocalTrace.ptIcsX,2);







if DRAWPOLYGON
    if ~isempty(inlierId)
        %                     if SHOWPOLYGON
        %                         figure(FigBase + FigBase + 110),clf;imshow(imgCurL);hold on;
        %                     end
        
        
        
        %                     pixGTR = [pixGT(:,1) - dispMapCurGTList pixGT(:,2)];
        
        
        
        %                                     figure(FigBase + FigBase + 232);hold on;axis equal;
        if 1
            if keyFrameLen == 2 %isempty(LocalTrace.probZ)
                if 0
                    DispRng = repmat(dispList(inlierId,:),1,length(disparityRng)) + repmat(disparityRng,length(inlierId),1);
                else
                    DispRng = LocalTrace.DispRng;
                end
                DispRng(DispRng < 0) = 0.0001;
                disparityError = dispList(:) - dispGTTmp(:);
                
                disparityErrorRound = round(disparityError./ConfigParam.disparity_sample_step).*ConfigParam.disparity_sample_step;
                
                
                depthGTOfst11 = round(-(disparityErrorRound)./ConfigParam.disparity_sample_step);
                depthGTInd11 = depthGTOfst11 + (size(DispRng,2)+1)/2;
                depthGTInd11(depthGTInd11 < 1) = 1;
                depthGTInd11(depthGTInd11 > size(DispRng,2)) = size(DispRng,2);
                
                %                             DepthGTInd(jkl,:) = depthGTInd11';
                
                depthGTIndAll11 = sub2ind(size(DispRng),[1:size(DispRng,1)]', depthGTInd11);
                
                checkDispRounding = DispRng(depthGTIndAll11) - dispGTTmp(:);
                if 0
                    for jk = 1 : length(inlierId)
                        [ProbZ(jk,:), ZZVec1(jk,:)] = probDensity(dispList(inlierId(jk),:), ConfigParam.disparity_sigma,ConfigParam.disparity_beta,ConfigParam.reproj_beta, DispRng(jk,:),ConfigParam.disparity_sample_interval, 'disparity');
                    end
                else
                    ProbZ = LocalTrace.probZ;
                    ZZVec1 = LocalTrace.sampleZ;
                end
                ProbZ(isnan(ProbZ)) = 0;
                if 0
                    LocalTrace.probZ = nan(length(LocalTrace.ptCcsZ), size(ProbZ,2));
                    LocalTrace.sampleZ = nan(length(LocalTrace.ptCcsZ), size(ZZVec1,2));
                    LocalTrace.probZ(inlierId,:) = ProbZ;
                    LocalTrace.sampleZ(inlierId,:) = ZZVec1;
                end
                outId = [];
            else
                if 0
                    DispRng = repmat(dispList(inlierId,:),1,length(disparityRng)) + repmat(disparityRng,length(inlierId),1);
                else
                    DispRng1 = repmat(dispList(:),1,length(disparityRng)) + repmat(disparityRng,length(inlierId),1);
                    DispRng = NewDispRng(ismember(inlierIdPrv, inlierId),:);
                    DispRng0 = DispRng;
                    if 0 % to check
                        figure,plot(DispRng1(:,11) - DispRng(:,11));
                    end
                    
                end
                
                try
                    [DispRng,ProbZTmpReSamp2,DispKeyInPrvRng] = IntersectCone(obj, inlierId, inlierIdPrv, imgKeyL, imgPrvL,b2c, DispRng,dispPrvList_, DepthProbility,DepthId,ConfigParam,KeyFrameFlagList,thetaRngMat,thetaProbMat,LocalTrace);
                catch
                    svkbj = 1;
                end
                if NewPrvDispResamp
                    NewDispRng(ismember(inlierIdPrv,inlierId),:) = DispRng;
                    newDispStep(ismember(inlierIdPrv,inlierId),:) = mean(diff(DispRng'))';
                    ProbZTmpReSamp(ismember(inlierIdPrv,inlierId),:) = ProbZTmpReSamp2;
                else
                    DispRng = DispRng0;
                end
                
                
                
                
                
                DispRng(DispRng < 0) = 0.0001;
                
                if 0
                    ProbZ = LocalTrace.probZ(inlierId,:);
                else
                    %                                 DispRng=DispRng;
                    if 0 %keyFrameLen > FrmNumWithGloden + 1
                        ProbZ = ProbZTmpReSamp2;
                    else
                        if 0
                            ProbZ = LocalTrace.probZ(inlierId,:);
                        else
                            ProbZ = LocalTrace.probZ;
                        end
                    end
                    
                    
                    dispList(:) = DispRng(:,(size(DispRng,2)+1)/2);
                    LocalTrace.ptCcsZ(:,1) = (intrMat(1,1).*norm(CamModel.transVec1To2))./(dispList(:) + (princpPtR(1) - princpPtL(1)));
                    zList(:) = LocalTrace.ptCcsZ(:,1);
                    dispPrvList_(:) = DispKeyInPrvRng(:,(size(DispRng,2)+1)/2);
                    depthPrvList_(:) = (intrMat(1,1).*norm(CamModel.transVec1To2))./(dispPrvList_(:) + (princpPtR(1) - princpPtL(1)));
                end
                ProbZ_out = KeyProbZ{end,21}(:,2); %  keyProbZ{end,21}(:,2);
                idLast = KeyProbZ{end,2};
                outId = find(~ismember(idLast, inlierId));
                %                             ZZVec1 = LocalTrace.sampleZ(inlierId,:);
                
                
                for jk = 1 : length(inlierId)
                    [~, ZZVec1(jk,:)] = probDensity(dispList((jk),:), ConfigParam.disparity_sigma,ConfigParam.disparity_beta,ConfigParam.reproj_beta, DispRng(jk,:),ConfigParam.disparity_sample_interval, 'disparity');
                end
                
                
                
                
                asdk = 1;
            end
            
            
            
            if 1 % ~USEGOLDENDISP %keyFrameLen == 2
                %                     figure(FigBase + FigBase + 38),clf;plot(dispList - dispMatGT(validInd));title('stereoDisp - gtDisp');
                %                 dispGTTmp = dispMatGT(validInd);
                figure(FigBase + 38),clf;subplot(1,3,1);hold on;plot(dispList(:) - dispGTTmp(:));title('stereoDisp - gtDisp');subplot(1,3,2);imshow(imgCur);hold on; %subplot(1,3,3);plot(dispErrCur);title('stereoDispCur - gtDispCur');
                disparityError = dispList(:) - dispGTTmp(:);
                
                if keyFrameLen == 2
                    disparityErrorRound = round(disparityError./ConfigParam.disparity_sample_step).*ConfigParam.disparity_sample_step;
                else
                    %                                 if NewPrvDispResamp
                    %
                    %                                     newDispStep(ismember(inlierIdPrv, inlierId)) = mean(diff(DispRng'))';
                    %
                    %                                 end
                    
                    newDispStepUse = newDispStep(ismember(inlierIdPrv, inlierId));
                    disparityErrorRound = round(disparityError./newDispStepUse).*newDispStepUse;
                    
                    
                    sabab = 1;
                end
                
                
                
                if 1 % GTTRACKING
                    
                    b2cPmat = GetPctBody2Cam(CoordSysAligner, 1);
                    if 0
                        k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(thetaRng(idM)),zeros(3,1));
                    else
%                         k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cRef0),zeros(3,1));
                        k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cRef_00),zeros(3,1));
                    end
                    k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
                    
                    Pix = [LocalTrace.ptIcsX(:,1) LocalTrace.ptIcsY(:,1)];
                    
                    metricPrevPtCcsGT = intrMat\HomoCoord(Pix',1);
                    metricPrevPtCcsGT = normc(metricPrevPtCcsGT);
                    if 0
                        scaleAllGT = depthListGT(inlierId)./metricPrevPtCcsGT(3,:)';
                    else
                        dispGTComp = dispList(:) - disparityErrorRound;
                        depthListGTComp = intrMat(1,1).*norm(CamModel.transVec1To2)./(dispGTComp + (princpPtR(1) - princpPtL(1)));
                        scaleAllGT = depthListGTComp./metricPrevPtCcsGT(3,:)';
                    end
                    XYZ = [repmat(scaleAllGT',3,1).*metricPrevPtCcsGT];
                    
                    
                    k2cT = -b2cPmat.transformMat(1:3,1:3)*k2cBodyPmat.transformMat(1:3,1:3)*b2cPmat.transformMat(1:3,1:3)'*b2cPmat.transformMat(1:3,4) + b2cPmat.transformMat(1:3,4);
                    k2cCam = [k2cCamPmat.transformMat(1:3,1:3) k2cT;0 0 0 1];
                    homocurrCcsXYZ = k2cCam*HomoCoord(XYZ,1); %pix0_GT_ = pflat(intrMat*homocurrCcsXYZALL_(1:3,:))';
                    pixGT = pflat(intrMat*homocurrCcsXYZ(1:3,:))';
                    pixGT = pixGT(:,1:2);
                    pixGTR = [pixGT(:,1) - dispMapCurGTList pixGT(:,2)];
                    
                    pixGT__ = pixGT;
                    pixGTR__ = pixGTR;
                    
                    if keyFrameLen > 2
                        [pixGTPrv, pixGTRPrv] = GetGtTrace(CamModel,CoordSysAligner,LocalTrace,REFAngList(end),inlierId,dispList,disparityErrorRound,intrMat,princpPtL,princpPtR,dispMapCurGTList);
                        
                        
                    end
                    
                end
            end
            
            
            if 0
                maxFeatZ = max(ProbZ')';
                maxFeatZ = repmat(maxFeatZ, 1, size(ProbZ,2));
                ProbZTmp_norm = ProbZ./maxFeatZ;
                
                % %                         ProbZ = ProbZTmp_norm;
                
                depthGTOfst = round(-(disparityErrorRound)./ConfigParam.disparity_sample_step);
                depthGTInd = depthGTOfst + (size(ProbZ,2)+1)/2;
                depthGTInd(depthGTInd < 1) = 1;
                depthGTInd(depthGTInd > size(ProbZ,2)) = size(ProbZ,2);
                depthGTIndAll = sub2ind(size(ProbZ),[1:size(ProbZ,1)]', depthGTInd);
                figure(FigBase + 43),
                %                         if jkl == 1
                if keyFrameLen == 2
                    probDir = probPath;  %fullfile(pwd, 'prob');
                    saveas(gcf,fullfile(probDir,sprintf('hist_%05d.png',length(dir(fullfile(probDir,'hist_*.png')))+1)));
                    saveas(gcf,fullfile(probDir,sprintf('hist_%05d.fig',length(dir(fullfile(probDir,'hist_*.fig')))+1)));
                    
                end
                %                         end
                ProbZTmpTmp_norm = ProbZTmp_norm(depthGTIndAll);
                figure(FigBase + 43),clf;hist(ProbZTmpTmp_norm(~isnan(ProbZTmpTmp_norm)),50); title(num2str(sum(~isnan(ProbZTmpTmp_norm))));
            end
            
            Pix = [LocalTrace.ptIcsX(:,1) LocalTrace.ptIcsY(:,1)];
            
            ZTrue = zList(:);
            
            
            metricPrevPtCcs = intrMat\HomoCoord(Pix',1);
            metricPrevPtCcs = normc(metricPrevPtCcs);
            scaleAll = ZTrue./metricPrevPtCcs(3,:)';
            %                                   scale0 = zTrue./metricPrevPtCcs(3,:)';
            keyCcsXYZAll = [repmat(scaleAll',3,1).*metricPrevPtCcs];
            
            
            
            thetaRange123 = k2cRef + [ConfigParam.theta_range(1) : ConfigParam.theta_sample_step : ConfigParam.theta_range(2)];
            
            
            Prob = zeros(length(thetaRange123),length(inlierId),length(disparityRng));
            
            if ~GTTRACKING
                pt2dCur = [LocalTrace.ptIcsX(:,keyLength) LocalTrace.ptIcsY(:,keyLength)];
                try
                    pt2dPrv = [LocalTrace.ptIcsX(:,end-1) LocalTrace.ptIcsY(:,end-1)];
                catch
                    pt2dPrv = Pix; % pt2dCur;
                end
            else
                if 1
                    pt2dCur =  pixGT;
                    try
                        pt2dPrv = pixGTPrv;
                    catch
                        asvk = 1;
                    end
                else
                    pt2dCur = [LocalTrace.ptIcsX(inlierId,keyLength) LocalTrace.ptIcsY(inlierId,keyLength)];
                    pt2dCur(:,2) =  pixGT(:,2);
                end
            end
            pt2dCur0 = pt2dCur;
            pt2dCurHomo = (inv(intrMat)*pextend(pt2dCur'))';
            
            dispCurList = dispCurList_(:);
            
            if ~GTTRACKING
                pt2dCurR = [pt2dCur(:,1) - dispCurList pt2dCur(:,2)];
            else
                %                         pt2dCurR = pixGTR;
                if 1
                    %                             pt2dCurR(:,2) = pixGTR(:,2);
                    pt2dCurR = pixGTR;
                    try
                        pt2dPrvR = pixGTRPrv;
                    catch
                        asdgkjb = 1;
                    end
                else
                    pt2dCur_0 = [LocalTrace.ptIcsX(:,keyLength) LocalTrace.ptIcsY(:,keyLength)];
                    pt2dCurR = [pt2dCur_0(:,1) - dispCurList pt2dCur_0(:,2)];
                    pt2dCurR(:,2) = pixGTR(:,2);
                end
            end
            
            pt2dCurRHomo = (inv(intrMat)*pextend(pt2dCurR'))';
            PixHomo = (inv(intrMat)*pextend(Pix'))';
            if 0
                validIndCur = sub2ind(size(dispMapKey), round(pt2dCur(:,2)), round(pt2dCur(:,1)));
            end
            dispErrCur = dispCurList  - dispMapCurGTList;
            
            % %                         try
            % %                             [sampledDispRngPrvInKey,ProbZTmpReSamp,DispKeyInPrvRng] = IntersectCone(obj, inlierId, inlierIdPrv, keyFrameImgL, imgPrvL,b2c, DispRng,dispPrvList_, DepthProbility,DepthId);
            % %                         catch
            % %                             svkbj = 1;
            % %                         end
            
            for jkl = 1 : DEPTHITERNUM
                
                
                
                [maxFeatZ1,idMax] = max(ProbZ');
                maxFeatZ = maxFeatZ1';
                maxFeatZ = repmat(maxFeatZ, 1, size(ProbZ,2));
                ProbZTmp_norm = ProbZ./maxFeatZ;
                
                % %                         ProbZ = ProbZTmp_norm;
                if keyFrameLen == 2
                    depthGTOfst = round(-(disparityErrorRound)./ConfigParam.disparity_sample_step);
                else
                    depthGTOfst1 = round(-(disparityErrorRound)./ConfigParam.disparity_sample_step);
                    depthGTOfst = round(-(disparityErrorRound)./newDispStep(ismember(inlierIdPrv,inlierId)));
                    
                    
                end
                depthGTInd = depthGTOfst + (size(ProbZ,2)+1)/2;
                depthGTInd(depthGTInd < 1) = 1;
                depthGTInd(depthGTInd > size(ProbZ,2)) = size(ProbZ,2);
                
                DepthGTInd(jkl,:) = depthGTInd';
                
                depthGTIndAll = sub2ind(size(ProbZ),[1:size(ProbZ,1)]', depthGTInd);
                figure(FigBase + 43),
                if jkl == 1 %DEPTHITERNUM
                    if keyFrameLen == 2
                        probDir = probPath;  % fullfile(pwd, 'prob');
                        %                                     saveas(gcf,fullfile(probDir,sprintf('hist_%05d.png',length(dir(fullfile(probDir,'hist_*.png')))+1)));
                        %                                     saveas(gcf,fullfile(probDir,sprintf('hist_%05d.fig',length(dir(fullfile(probDir,'hist_*.fig')))+1)));
                        
                    end
                end
                ProbZTmpTmp_norm = ProbZTmp_norm(depthGTIndAll);
                ProbZTmpTmp_norm0 = ProbZTmpTmp_norm;
                %                             [depthGTInd, ProbZTmpTmp_norm, ProbZTmpNorm, idMax, depthGTIndAll, depthUpdateIndAll] = VisualLocalizer.GetDepthHist(obj, ProbZ,disparityErrorRound,ConfigParam.disparity_sample_step);
                
                
                %                             figure(FigBase + 43),clf;hist(ProbZTmpTmp_norm(~isnan(ProbZTmpTmp_norm)),50); title(num2str(sum(~isnan(ProbZTmpTmp_norm))));
                figure(FigBase + 43),clf;subplot(2,2,1);hist(ProbZTmpTmp_norm(~isnan(ProbZTmpTmp_norm)),50); title(sprintf('%d\nsigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f',sum(~isnan(ProbZTmpTmp_norm)),ConfigParam.reproj_sigma,ConfigParam.reproj_beta,ConfigParam.reproj_sigma_right,ConfigParam.reproj_beta_right));
                subplot(2,2,3);imshow(imgCur); hold on;plot(pt2dCur(ProbZTmpTmp_norm>ConfigParam.depth_hist_ratio,1), pt2dCur(ProbZTmpTmp_norm>ConfigParam.depth_hist_ratio,2), '.r');plot(pt2dCur(ProbZTmpTmp_norm<=ConfigParam.depth_hist_ratio,1), pt2dCur(ProbZTmpTmp_norm<=ConfigParam.depth_hist_ratio,2), '.g');legend('good','bad','Location','southwest');title(sprintf('cur probZ > %0.3f',ConfigParam.depth_hist_ratio));
                if ~isempty(outId)
                    subplot(2,2,2);hist(ProbZ_out(outId),50); title(sprintf('%d',length(outId)));
                    subplot(2,2,4);imshow(imgPrvL); hold on;plot(KeyProbZ{end,6}(outId,1), KeyProbZ{end,6}(outId,2), 'xy');plot(KeyProbZ{end,6}(ProbZ_out>ConfigParam.depth_hist_ratio,1), KeyProbZ{end,6}(ProbZ_out>ConfigParam.depth_hist_ratio,2), '.r');plot(KeyProbZ{end,6}(ProbZ_out<=ConfigParam.depth_hist_ratio,1), KeyProbZ{end,6}(ProbZ_out<=ConfigParam.depth_hist_ratio,2), '.g');legend('lost','good','bad','Location','southwest'); title(sprintf('prv probZ > %0.3f',ConfigParam.depth_hist_ratio));
                end
                probDir = probPath;
                %                             saveas(gcf,fullfile(probDir,sprintf(strcat('hist_',probDir(end-14:end),num2str(sum(KeyFrameFlagList)),'_%05d.png'),length(dir(fullfile(probDir,'hist_*.png')))+1)));
                saveas(gcf,fullfile(probDir,sprintf(strcat('hist_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probDir,strcat('hist_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__*.png'))))+1 + FigBase)));
                %                             saveas(gcf,fullfile(probDir,sprintf(strcat('hist_',probDir(end-14:end),'_%05d.png'),length(dir(fullfile(probDir,'hist_*.png')))+1)));
                
                
                if keyFrameLen == 2
                    
                    if 0
                        featPtList = VisualLocalizer.DetectFast(imgKeyL);
                        depthMapKey__ = depthMapKey; % keyFrameDepth;
                        inValid__ = sub2ind(size(depthMapKey__), round(featPtList(:,2)), round(featPtList(:,1)));
                        depthList__ = depthMapKey__(inValid__);
                        %                                 inId_1 = find(depthList__ > 0);
                        featPtList = featPtList(depthList__ > 0, :);
                        depthList__ = depthList__(depthList__ > 0);
                    else
                        featPtList = [LocalTrace.ptIcsX(:,1) LocalTrace.ptIcsY(:,1)];
                        depthList__ = LocalTrace.ptCcsZ(:,1);
                    end
                    
                    
                    
                    inTrackFlag = true(size(featPtList,1),1);
                    
                    pnp_ang_est_max_margin = [deg2rad([-1 1]) 2]; % obj.configParam.pure_rot_reproject_outlier_threshold];
                    
                    
                    
                    [predPtList, inTrackFlag00] = TrackFeaturePoints2( imgKeyL, imgCur, featPtList,inTrackFlag);
                    
                    
                    [p2cBodyRotAng,Err,inId] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin,intrMat,featPtList(inTrackFlag00,:),predPtList(inTrackFlag00,:),depthList__(inTrackFlag00,:), find(inTrackFlag00),true(sum(inTrackFlag00),1),b2c,k2cRef0);
                    
                    
                    if 0
                        figure,subplot(1,2,1),showMatchedFeatures(imgKeyL, imgCur, featPtList(inTrackFlag00,:),predPtList(inTrackFlag00,:));subplot(1,2,2);showMatchedFeatures(imgKeyL, imgCur, featPtList(inId,:),predPtList(inId,:))
                    end
                    
                    
                    thetaRange = k2cRef + [ConfigParam.theta_range(1) : ConfigParam.theta_sample_step : ConfigParam.theta_range(2)];
                    
                    if 0
                        [~, angOptK2C_11111, ~, ~] = New_OnlyP2_new(obj, featPtList(inId,:),predPtList(inId,:), intrMat,[],T_B2C, k2cRef, tx, ty, tz, thetaRng,ConfigParam);
                    elseif 0
                        [~, angOptK2C_11111, ~, ~] = New_OnlyP2_new(obj, Pix, pt2dCur, intrMat,[],T_B2C, k2cRef, tx, ty, tz, thetaRng,ConfigParam);
                    else
                        % %                                    angOptP2C__1 = k2cRef - REFAngList(end);
                        % %
                        % %
                        % %                                     pixprv = pt2dPrv;
                        
                        [angOptK2C_11111, p2cOpt, k2pOpt] = AngleSpace.EpiPolarThreeView(obj, k2cRef0, k2cRef0, [deg2rad(-0.2) : ConfigParam.theta_sample_step : deg2rad(0.2)], Pix, Pix, pt2dCur, intrMat, b2c);
                    end
                    p2cOnlyP2_111 = angOptK2C_11111;
                    
                    
                    if ~UseGoldenThetaP2C
                        p2cOnlyP2_111 = angOptK2C_11111;
                    else
                        angOptK2C_11111 = k2cRef0;
                        p2cOnlyP2_111 = k2cRef0;
                    end
                    
                    
                    
                    if ReplaceK2CRefInKey
                        
                        k2cRef = angOptK2C_11111;
                        
                    end
                    thetaSamp = [ConfigParam.theta_range(1) : ConfigParam.theta_sample_step : ConfigParam.theta_range(2)];
                    thetaSamp0 = thetaSamp;
                    thetaRange = k2cRef + [ConfigParam.theta_range(1) : ConfigParam.theta_sample_step : ConfigParam.theta_range(2)];
                    thetaRng = thetaRange;
                    
                    [thetaGrid, disparityGrid] = meshgrid(thetaRange, disparityRng);
                    ThetaDisp = [thetaGrid(:) disparityGrid(:)];
                    
                else
                    
                    thetaSamp = [ConfigParam.theta_range(1) : ConfigParam.theta_sample_step : ConfigParam.theta_range(2)];
                    thetaSamp0 = thetaSamp;
                    thetaRange = k2cRef + [ConfigParam.theta_range(1) : ConfigParam.theta_sample_step : ConfigParam.theta_range(2)];
                    thetaRng = thetaRange;
                    
                    [thetaGrid, disparityGrid] = meshgrid(thetaRange, disparityRng);
                    ThetaDisp = [thetaGrid(:) disparityGrid(:)];
                    
                    
                    if 0
                        featPtList = VisualLocalizer.DetectFast(imgPrvL);
                        depthMapPrv__ = prvDepthVisual; % prvDepthVisual;
                        inValid__ = sub2ind(size(depthMapPrv__), round(featPtList(:,2)), round(featPtList(:,1)));
                        depthList__ = depthMapPrv__(inValid__);
                        %                                 inId_1 = find(depthList__ > 0);
                        featPtList = featPtList(depthList__ > 0, :);
                        depthList__ = depthList__(depthList__ > 0);
                    else
%                         featPtList = featPtList(depthList__ > 0, :);
%                         depthList__ = depthList__(depthList__ > 0);
                        featPtList = [LocalTrace.ptIcsX(:,end-1) LocalTrace.ptIcsY(:,end-1)];
                        depthList__ = LocalTrace.ptCcsZ(:,end-1);
                        
                    end
                    
                    
                    
                    inTrackFlag = true(size(featPtList,1),1);
                    
                    pnp_ang_est_max_margin = [deg2rad([-1 1]) 2]; %obj.configParam.pure_rot_reproject_outlier_threshold];
                    [predPtList, inTrackFlag00] = TrackFeaturePoints2( imgPrvL, imgCur, featPtList,inTrackFlag);
                    
                    
                    [p2cBodyRotAng,Err, inId] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin,intrMat,featPtList(inTrackFlag00,:),predPtList(inTrackFlag00,:),depthList__(inTrackFlag00), find(inTrackFlag00),true(sum(inTrackFlag00),1),b2c,k2cRef0 - REFAngList(end));
                    
                    if 0
                        figure,subplot(1,2,1),showMatchedFeatures(imgPrvL, imgCur, featPtList(inTrackFlag00,:),predPtList(inTrackFlag00,:));subplot(1,2,2);showMatchedFeatures(imgKeyL, imgCur, featPtList(inId,:),predPtList(inId,:))
                    end
                    
                    
                    
                    
                    
                    thetaP2C_111 = k2cRef - REFAngList4(end);
                    thetaRngP2CTemp_111 = thetaP2C_111 + thetaSamp;
                    if 0
                        [~, thetaP2COpt_111, ~,~] = New_OnlyP2_new(obj, featPtList(inId,:),predPtList(inId,:), intrMat,[],T_B2C, thetaP2C_111, tx, ty, tz, thetaRngP2CTemp_111,ConfigParam);
                    elseif 0
                        [~, thetaP2COpt_111, ~,~] = New_OnlyP2_new(obj, pt2dPrv, pt2dCur, intrMat,[],T_B2C, thetaP2C_111, tx, ty, tz, thetaRngP2CTemp_111,ConfigParam);
                    else
                        angOptP2C_1_1 = k2cRef0 - REFAngList(end);
                        
                        
                        pixprv = pt2dPrv;
                        
                        [k2cOpt, thetaP2COpt_111, k2pOpt] = AngleSpace.EpiPolarThreeView(obj, k2cRef0, angOptP2C_1_1, [deg2rad(-0.2) : ConfigParam.theta_sample_step : deg2rad(0.2)], Pix, pixprv, pt2dCur, intrMat, b2c);
                        
                    end
                    if ~UseGoldenThetaP2C
                        p2cOnlyP2_111 = thetaP2COpt_111;
                    else
                        p2cOnlyP2_111 = k2cRef0 - REFAngList(end);
                    end
                end
                
                if keyFrameLen == 2
                    
                    %
                    accumP2CTemp = [];
                    accumP2CTemp = p2cOnlyP2_111;
                else
                    
                    accumP2CTemp = [accumP2CTemp; accumP2CTemp(end) + p2cOnlyP2_111];
                    
                end
                
                
                try
                    [~, angOptK2C_22222, ~, ~] = New_OnlyP2_new(obj, Pix, pt2dCur, intrMat,[],T_B2C, k2cRef, tx, ty, tz, thetaRng,ConfigParam);
                catch
                    asdbjk = 1;
                end
                if keyFrameLen > 2
                    thetaP2C = k2cRef - REFAngList4(end);
                    
                    thetaP2C1 = accumP2CTemp(end) - REFAngList4(end);
                    
                    thetaP2C2 = accumP2CTemp(end) - accumP2CTemp(end-1);
                    
                    thetaP2C3 = k2cBody - accumP2CTemp(end-1);
                    
                    % % %                                 angOptP2C__1 = k2cRef - REFAngList(end);
                    % % %
                    % % %
                    % % %                                 pixprv = pt2dPrv;
                    % % %
                    % % %                                 [k2cOpt, p2cOpt, k2pOpt] = AngleSpace.EpiPolarThreeView(obj, k2cRef, angOptP2C__1, [deg2rad(-0.2) : ConfigParam.theta_sample_step : deg2rad(0.2)], Pix, pixprv, pt2dCur, intrMat, b2c);
                    % % %
                    if ~UseGoldenThetaP2C
                        if 0
                            thetaP2C = mean([thetaP2C1 thetaP2C1]);
                        elseif 1
                            thetaP2C = mean([thetaP2C2 thetaP2C2]);
                        elseif 0
                            thetaP2C = mean([thetaP2C3 thetaP2C3]);
                        elseif 0
                            thetaP2C = p2cOpt;
                        else
                            thetaP2C = k2cRef - REFAngList4(end);
                        end
                    else
                        %                           thetaP2C = k2cRef0 - REFAngList(end);
                        thetaP2C = k2cRef - REFAngList(end);
                    end
                    %                                 thetaP2C = angOptK2C_22222 - REFAngList4(end);
                    
                    if 0 % keyFrameLen > FrmNumWithGloden + 1
                        thetaP2C = k2cRef - REFAngList4(end);
                    end
                    
                    
                    
                    
                    thetaP2C0 = thetaP2C;
                    
                    thetaRngP2CTemp = thetaP2C + thetaSamp;
                    [~, thetaP2COpt, ~,~] = New_OnlyP2_new(obj, pt2dPrv, pt2dCur, intrMat,[],T_B2C, thetaP2C, tx, ty, tz, thetaRngP2CTemp,ConfigParam);
                    
                    if UseNewP2CRef
                        
                        thetaP2C = thetaP2COpt;
                    end
                    
                    p2cOnlyP2 = thetaP2COpt;
                    
                    
                    
                    
                    if ~UseNewP2CRef
                        if doRoundingTheta2
                            thetaP2C = deg2rad(round(rad2deg(thetaP2C0 + ThetaNoise)./roundingPrecision2).*roundingPrecision2);
                        end
                    end
                    
                    if 0 % UseGoldenThetaP2C
                        thetaP2C = k2cRef0 - REFAngList(end);
                    end
                    
                    
                    % %                                 thetaP2C = thetaP2C - deg2rad(0.02);
                    % % % % % % % % %                                 if ismember(keyFrameLen, [2 5 8 11 14 17])
                    % % % % % % % % %                                     thetaP2C = k2cRef0 - REFAngList(end)  +  deg2rad(0.05*(1)^(keyFrameLen)) + deg2rad(0.01*(rand(1)-0.5));
                    % % % % % % % % %                                 else
                    % % % % % % % % %                                     thetaP2C = k2cRef0 - REFAngList(end)  -  deg2rad(0.025*(1)^(keyFrameLen)) + deg2rad(0.01*(rand(1)-0.5));
                    % % % % % % % % %                                 end
                    
                    % %                                 thetaP2C = k2cRef0 - REFAngList(end);
                    
                    
                    thetaRngP2C = thetaP2C + thetaSamp;
                    thetaSampP2C = thetaRngP2C - thetaP2C;
                    
                    % % % %                                 pt2dPrv = [LocalTrace.ptIcsX(inlierId,end-1) LocalTrace.ptIcsY(inlierId,end-1)];
                    
                    if InheriDispRng
                        if ~NewPrvDispResamp
                            DispRngP2C = repmat(dispPrvList_(:),1,size(DispRng,2)) + (DispRng - repmat(DispRng(:,(size(DispRng,2)+1)/2), 1, size(DispRng,2)));
                        else
                            DispRngP2C = DispKeyInPrvRng;
                        end
                    else
                        DispRngP2C = dispPrvList_(:) + disparityRng;
                    end
                    
                    pt2dPrvR = [pt2dPrv(:,1) - dispPrvList_(:) pt2dPrv(:,2)];
                    
                    
                    if GTTRACKING
                        pt2dPrv = pixGTPrv;
                        pt2dPrvR = pixGTRPrv;
                    end
                    
                    
                    
                    if keyFrameLen <= 3
                        sc = Sc1;
                    else
                        sc = Sc2;
                    end
                    
                    
                    
                    
                    if EvenDepthProb
                        ProbZP2C = ones(size(ProbZ));
                    else
                        if UsePrvDepth
                            ProbZP2C = ones(size(ProbZ));
                        else
                            ProbZP2C = ((ProbZ));
                        end
                    end
                    ProbZP2C_norm = ProbZP2C./repmat(max(ProbZP2C')',1,size(ProbZP2C,2));
                    ZZVecPrv1 = [];
                    for jkj = 1 : length(inlierId)
                        [~, ZZVecPrv1(jkj,:)] = probDensity(dispPrvList_((jkj),:), ConfigParam.disparity_sigma,ConfigParam.disparity_beta,ConfigParam.reproj_beta, DispRngP2C(jkj,:),ConfigParam.disparity_sample_interval, 'disparity');
                    end
                    
                    %                             Pix = [LocalTrace.ptIcsX(inlierId,1) LocalTrace.ptIcsY(inlierId,1)];
                    
                    ZTruePrv = depthPrvList_(:);
                    
                    
                    metricPrv = intrMat\HomoCoord(pt2dPrv',1);
                    metricPrv = normc(metricPrv);
                    scaleAllPrv = ZTruePrv./metricPrv(3,:)';
                    %                                   scale0 = zTrue./metricPrevPtCcs(3,:)';
                    prvCcsXYZAll = [repmat(scaleAllPrv',3,1).*metricPrv];
                    
                    if 1
                        [thetaProbP2C,idMP2C,angOptP2C, ~, ~, ~, ~,~,~,~,~,ProbReprojVecAllP2C, ProbReprojVecRAllP2C, ~, ~, ~,~,~, ~, ~,~,~,~,~,~,~,thetaExp_tmpP2C] = ReplayTheta(obj, thetaRngP2C, b2c, prvCcsXYZAll, intrMat, pt2dPrv, disparityRng, baseline, ZZVecPrv1, L2R, pt2dCur, pt2dCurR, ProbZP2C, ProbZP2C_norm, imgCur, imgCurR,thetaSampP2C,r_cam,tx, ty, tz);
                        
                        % % % %                                     angOptP2C = angOptP2C - deg2rad(0.01);
                        
                        angOptP2C__1 = angOptP2C;
                        % %                                     angOptP2C = k2cRef0 - REFAngList(end);
                        %                                     [thetaProbP2C_, angOptP2C, thetaExp_tmpP2C_, gtP2C] = New_OnlyP2(obj, pt2dPrv, pt2dCur, intrMat,thetaP2C,T_B2C, k2cRef0);
                        if UseNewP2Only
                            
                            if 0
                                inFlag1 = find(LocalTrace.ptIcsX(:,end) > 0 & LocalTrace.ptIcsY(:,end) > 0);
                                pixprv = [LocalTrace.ptIcsX(inFlag1,end-1)  LocalTrace.ptIcsY(inFlag1,end-1)];
                                pixcur = [LocalTrace.ptIcsX(inFlag1,end)  LocalTrace.ptIcsY(inFlag1,end)];
                                [thetaProbK2C_, angOptP2C, thetaExp_tmpP2C, ~,thetaRngP2C_use] = New_OnlyP2(obj, pixprv, pixcur, intrMat,angOptP2C,T_B2C, thetaP2C);
                            else
                                if 1
                                    [thetaProbP2C, angOptP2C, thetaExp_tmpP2C,thetaRngP2C_use] = New_OnlyP2_new(obj, pt2dPrv, pt2dCur, intrMat,[],T_B2C, thetaP2C, tx, ty, tz, thetaRngP2C,ConfigParam);
                                else
                                    [thetaProbP2C, angOptP2C, thetaExp_tmpP2C,thetaRngP2C_use] = New_OnlyP2_new_old(obj, pt2dPrv, pt2dCur, intrMat,thetaP2C,T_B2C, k2cRef, tx, ty, tz, thetaRngP2C,DispRng,inlierId);
                                end
                            end
                            
                            
                            
                        end
                    else
                        if 0
                            existPt1 = Points.Point(pt2dPrv(:,1),pt2dPrv(:,2),intrMat);
                            existPt2 = Points.Point(pt2dCur(:,1),pt2dCur(:,2),intrMat);
                            thetaRngP2C_ = [deg2rad(-179) : ConfigParam.theta_sample_step : deg2rad(179)] + (thetaP2C);
                            thetaSampP2C_ = thetaRngP2C_ - thetaP2C;
                            [thetaProbP2C_, angOptP2C_, thetaExp_tmpP2C] = UpdateModalAngleOnlyP2NewVersion(rad2deg(thetaRngP2C_),existPt1,existPt2,ConfigParam.reproj_sigma,intrMat,T_B2C);
                            
                            gtP2C = k2cRef0 - REFAngList(end);
                            angOptP2C_ = deg2rad(dot(rad2deg(thetaRngP2C), thetaProbP2C_));   % 0.5 0.8 0.0
                            thetaExp_tmpP2C_ = deg2rad(dot(rad2deg(thetaSampP2C), thetaProbP2C_));
                        else
                            [thetaProbP2C, angOptP2C, thetaExp_tmpP2C, gtP2C] = New_OnlyP2(obj, pt2dPrv, pt2dCur, intrMat,thetaP2C,T_B2C, k2cRef);
                        end
                    end
                    
                    
                    %                                     [reProjErr2,err2] = AngleSpace.ProjectErrorUnderTransform2(k2cCam(1:3,1:3), k2cCam(1:3,4), keyPtIcs, curPtIcs, intrMat);
                    
                    
                    
                    thetaProbP2CBeforeNewPlatform = thetaProbP2C;
                    thetaRngP2CBeforeNewPlatform = thetaRngP2C;  % - deg2rad(0.01);
                    thetaSampP2CBeforeNewPlatform = thetaSampP2C;
                    
                    
                    %%
                    %                                 angOptP2C = thetaP2C;
                    %                                 angOptP2C = k2cRef0 - REFAngList(end);
                    
                    
                    [thetaRngP2C, thetaSampP2C] = ArrangeThetaRng(obj,rad2deg(thetaExp_tmpP2C),thetaP2C,sc,thetaSampP2C);
                    %
                    thetaProbP2C = interp1(thetaRngP2CBeforeNewPlatform, thetaProbP2CBeforeNewPlatform, thetaRngP2C);
                    thetaProbP2C(isnan(thetaProbP2C)) = min(thetaProbP2CBeforeNewPlatform);
                    if 0
                        figure,plot(rad2deg(thetaRngP2CBeforeNewPlatform), thetaProbP2CBeforeNewPlatform);hold on;plot(rad2deg(thetaRngP2C), thetaProbP2C);
                    end
                    
                    
                    
                    %                                 k2cRef = REFAngList4(end) + angOptP2C;
                    k2cRefErr = rad2deg(k2cRef - k2cRefBak);
                    
                    p2cRefErrGT = [rad2deg(angOptP2C - (k2cRef0 - REFAngList(end)))];
                    k2cRefErrGT = [rad2deg(k2cRef - k2cRef0) rad2deg(k2cRefBak - k2cRef0)  p2cRefErrGT];
                    akgdsaj = 1;
                else
                    angOptP2C = []; thetaProbP2C = [];thetaRngP2C = [];thetaSampP2C = []; thetaProbP2CBeforeNewPlatform = []; thetaRngP2CBeforeNewPlatform = []; thetaP2C = [];thetaP2C0 = [];% thetaRngP2C - thetaP2C;
                end
                thetaRange = k2cRef + [ConfigParam.theta_range(1) : ConfigParam.theta_sample_step : ConfigParam.theta_range(2)];
                thetaRng = thetaRange;
                [thetaGrid, disparityGrid] = meshgrid(thetaRange, disparityRng);
                ThetaDisp = [thetaGrid(:) disparityGrid(:)];
                
                
                
                
                ReprojErrVecTmp = {}; ReprojErrVecRTmp = {}; ReprojErrVecTmpMatT = {}; ReprojErrVecRTmpMatT = {};
                ValidFeatFlag = []; ValidFeatFlag0 = []; ValidFeatFlagFusion = [];ValidFeatFlagQuant = [];
                thetaRng0 = thetaRng;
                if 0
                    gukj = 1;
                else
                    [thetaProb,idM,angOpt, ReprojErrVecTmp, ReprojErrVecRTmp, ReprojErrVecTmpMatT, ReprojErrVecRTmpMatT,ValidFeatFlag,ValidFeatFlag0,ValidFeatFlagFusion,ValidFeatFlagQuant,ProbReprojVecAll,ProbReprojVecRAll, PixKeyVecTmp, PixKeyVecRTmp, Probb,validFeatFlagFusion,validFeatFlag, ProbReprojVec, ProbReprojVecR,bb,b,pixKeyVecTmpXCoordMatStack,pixKeyVecTmpYCoordMatStack,pixKeyVecRTmpXCoordMatStack,pixKeyVecRTmpYCoordMatStack, ...
                        thetaExp_tmp0, errEpiline, angOptEpiInit, depthC2KInd_ind]       =   ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, pt2dCur, pt2dCurR,ProbZ, ProbZTmp_norm, imgCur, imgCurR,thetaSamp,r_cam,tx, ty, tz);
                end
                
                k2cRef0;
                angOptEpi = angOptEpiInit;
                
                angOptInit = angOpt;
                thetaRngBefore00 = thetaRng;
                thetaSampBefore00 = thetaSamp;
                
                if keyFrameLen <= 3
                    sc = Sc1;
                else
                    sc = Sc2;
                end
                %                             if IterThetaNum > 1
                %                                 [thetaRng, thetaSamp] = ArrangeThetaRng(obj,rad2deg(thetaExp_tmp0),k2cRef,sc,thetaSamp0);
                %                             end
                angOptList0 = [];
                for ft = 1 : IterThetaNum-1
                    
                    
                    if 1 % 20191123 use ExpCoord_OnlyP2 to update angOpt
                        [thetaProb,idM,angOpt, ReprojErrVecTmp, ReprojErrVecRTmp, ReprojErrVecTmpMatT, ReprojErrVecRTmpMatT,ValidFeatFlag,ValidFeatFlag0,ValidFeatFlagFusion,ValidFeatFlagQuant,ProbReprojVecAll, ProbReprojVecRAll, PixKeyVecTmp, PixKeyVecRTmp, Probb, ...
                            validFeatFlagFusion,validFeatFlag, ProbReprojVec, ProbReprojVecR,bb,b,pixKeyVecTmpXCoordMatStack,pixKeyVecTmpYCoordMatStack,pixKeyVecRTmpXCoordMatStack,pixKeyVecRTmpYCoordMatStack, thetaExp_tmp, errEpiline, angOptEpi,depthC2KInd_ind]...
                            = ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, pt2dCur, pt2dCurR,ProbZ./max(ProbZ(:)), ProbZTmp_norm, imgCur, imgCurR,thetaSamp,r_cam,tx, ty, tz);
                    else
                        [thetaProb,idM,angOpt, ReprojErrVecTmp, ReprojErrVecRTmp, ReprojErrVecTmpMatT, ReprojErrVecRTmpMatT,ValidFeatFlag,ValidFeatFlag0,ValidFeatFlagFusion,ValidFeatFlagQuant,ProbReprojVecAll, ProbReprojVecRAll, PixKeyVecTmp, PixKeyVecRTmp, Probb, ...
                            validFeatFlagFusion,validFeatFlag, ProbReprojVec, ProbReprojVecR,bb,b,pixKeyVecTmpXCoordMatStack,pixKeyVecTmpYCoordMatStack,pixKeyVecRTmpXCoordMatStack,pixKeyVecRTmpYCoordMatStack,thetaExp_tmp,errEpiline, angOptEpi,depthC2KInd_ind]...
                            = ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, ExpCoord_OnlyP2, pt2dCurR,ProbZ./max(ProbZ(:)), ProbZTmp_norm, imgCur, imgCurR,thetaSamp,r_cam,tx, ty, tz);
                        
                    end
                    %                                         [thetaRngNew, thetaSampNew] = ArrangeThetaRng(obj,rad2deg(angOpt - k2cRef),k2cRef,sc,thetaSamp0);
                    %                                 [thetaRng, thetaSamp] = ArrangeThetaRng(obj,rad2deg(thetaExp_tmp),k2cRef,sc,thetaSamp0);
                    angOptList0(ft,:) = angOpt; %rad2deg(angOpt - k2cRef0);
                    
                end
                angOptList0 = [angOptInit; angOptList0];
                if 0
                    if doRoundingTheta
                        [thetaProbK2C_, angOptK2C_, thetaExp_tmpK2C, ~,thetaRngP2C_use] = New_OnlyP2(obj, Pix, pt2dCur, intrMat,angOpt,T_B2C, k2cRef);
                        %                                 inFlag = find(LocalTrace.ptIcsX(:,end) > 0 & LocalTrace.ptIcsY(:,end) > 0);
                        %                                 pixkey = [LocalTrace.ptIcsX(inFlag,1)  LocalTrace.ptIcsY(inFlag,1)];
                        %                                 pixcur = [LocalTrace.ptIcsX(inFlag,end)  LocalTrace.ptIcsY(inFlag,end)];
                        %                                 [thetaProbK2C_, angOptK2C_, thetaExp_tmpK2C, ~,thetaRngP2C_use] = New_OnlyP2(obj, pixkey, pixcur, intrMat,angOpt,T_B2C, k2cRef);
                    else
                        [thetaProbK2C_, angOptK2C_, thetaExp_tmpK2C, ~,thetaRngP2C_use] = New_OnlyP2(obj, Pix, pt2dCur, intrMat,k2cRef0,T_B2C, k2cRef);
                        %                                 inFlag = find(LocalTrace.ptIcsX(:,end) > 0 & LocalTrace.ptIcsY(:,end) > 0);
                        %                                 pixkey = [LocalTrace.ptIcsX(inFlag,1) LocalTrace.ptIcsY(inFlag,1)];
                        %                                 pixcur = [LocalTrace.ptIcsX(inFlag,end)  LocalTrace.ptIcsY(inFlag,end)];
                        %                                 [thetaProbK2C_, angOptK2C_, thetaExp_tmpK2C, ~,thetaRngP2C_use] = New_OnlyP2(obj, pixkey, pixcur, intrMat,k2cRef0,T_B2C, k2cRef);
                        
                    end
                    
                else
                    %                                 [thetaProbK2C_, angOptK2C_, thetaExp_tmpK2C, ~,thetaRngP2C_use] = New_OnlyP2_new(obj, Pix, pt2dCur, intrMat,angOpt,T_B2C, k2cRef);
                    if 1
                        [thetaProbK2C_, angOptK2C_, thetaExp_tmpK2C, thetaRngK2C_use] = New_OnlyP2_new(obj, Pix, pt2dCur, intrMat,[],T_B2C, k2cRef, tx, ty, tz, thetaRng,ConfigParam);
                    else
                        [thetaProbK2C_, angOptK2C_, thetaExp_tmpK2C,thetaRngK2C_use] = New_OnlyP2_new_old(obj, Pix, pt2dCur, intrMat,k2cRef,T_B2C, k2cRef, tx, ty, tz, thetaRng,DispRng,inlierId);
                    end
                end
                figure(FigBase + 18),hold on;
                if keyFrameLen == 2
                    clf;
                end
                figure(FigBase + 18);plot(rad2deg(thetaRngK2C_use) - rad2deg(k2cRef0), thetaProbK2C_);hold on;plot(rad2deg(angOptK2C_ - k2cRef0), max(thetaProbK2C_),'or');grid on;
                saveas(gcf,fullfile(probPath,sprintf(strcat('newOnlyP2_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('newOnlyP2_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__*.png'))))+1 + FigBase)));
                
                % % % %                             if keyFrameLen == 2
                % % % %
                % % % % %
                % % % %                                 accumP2CTemp = [];
                % % % %
                % % % %                                 p2cOnlyP2 = angOptK2C_;
                % % % %                                 accumP2CTemp = p2cOnlyP2;
                % % % %                             else
                % % % %
                % % % %                                 accumP2CTemp = [accumP2CTemp; accumP2CTemp(end) + p2cOnlyP2];
                % % % %
                % % % %                             end
                %                             accumP2CTemp = [accumP2CTemp; accumP2CTemp(end) + p2cOnlyP2];
                
                
                
                
                
                
                
                if IterThetaNum > 1
                    if ~UseNewP2Only
                        [thetaRng, thetaSamp] = ArrangeThetaRng(obj,rad2deg(thetaExp_tmp0),k2cRef,sc,thetaSamp0);
                    else
                        if keyFrameLen == 2
                            angOpt = angOptK2C_;
                            [thetaRng, thetaSamp] = ArrangeThetaRng(obj,rad2deg(thetaExp_tmpK2C),k2cRef,sc,thetaSamp0);
                        else
                            [thetaRng, thetaSamp] = ArrangeThetaRng(obj,rad2deg(thetaExp_tmp0),k2cRef,sc,thetaSamp0);
                            
                        end
                    end
                end
                if keyFrameLen == 2
                    if UseNewP2Only
                        angOpt = angOptK2C_;
                        thetaProb = thetaProbK2C_;
                        thetaExp_tmpK2C;
                        angOpt3 = angOpt;
                        angOptList0 = angOpt;
                    end
                end
                
                s = Setting([]);
                s.configParam.disparityBoundary = ConfigParam.disparity_error;
                s.configParam.disparityIntervel = ConfigParam.disparity_sample_step;
                s.configParam.disparityRange = -ConfigParam.disparity_error : ConfigParam.disparity_sample_step : ConfigParam.disparity_error;
                s.configParam.disparityBin = length(s.configParam.disparityRange);
                
                s.configParam.disparityOrginRange = DispRng;
                s.configParam.angleIntervel = rad2deg(ConfigParam.theta_sample_step);
                s.configParam.angleBoundary = rad2deg(ConfigParam.theta_range(2));
                s.configParam.angleRange  = -s.configParam.angleBoundary : s.configParam.angleIntervel : s.configParam.angleBoundary;
                s.configParam.angleBin = length(s.configParam.angleRange);
                
                s.configParam.angleOrginRange = rad2deg(k2cRef) + s.configParam.angleRange;
                s.configParam.tableSize = s.configParam.angleBin*s.configParam.disparityBin;
                s.configParam.disparitySigma = obj.setting.configParam.disparitySigma;
                s.configParam.disparityBeta = obj.setting.configParam.disparityBeta;
                s.configParam.gaussSigma = obj.setting.configParam.gaussSigma;
                s.configParam.gaussBeta = obj.setting.configParam.gaussBeta;
                s.configParam.wx = obj.setting.configParam.wx;
                s.configParam.wy = obj.setting.configParam.wy;
                %
                angOpt3 = angOpt;
                
                if ONLYP2
                    akja = 1;
                    
                else
                    figure(FigBase + 20)
                    if keyFrameLen == 2
                        probDir = probPath;
                        clf;
                    end
                    
                    figure(FigBase + 20),subplot(2,1,1);hold on;plot(rad2deg(thetaSamp+ (k2cRef - k2cRef0)), thetaProb);grid on; title('1st theta from only p2');
                    plot(rad2deg(angOpt3 - k2cRef0), max(thetaProb),'or');
                    
                    
                end
                
                if isempty(angOptP2C)
                    angOptP2C__ = angOpt3;
                    pixprv = Pix;
                    
                else
                    angOptP2C__ = angOptP2C;
                    pixprv = pt2dPrv;
                end
                
                
                if 0
                    [k2cOpt, p2cOpt, k2pOpt] = AngleSpace.EpiPolarThreeView(obj, angOpt3, angOptP2C__, [deg2rad(-0.2) : ConfigParam.theta_sample_step : deg2rad(0.2)], Pix, pixprv, pt2dCur, intrMat, b2c);
                end
                
                
                thetaProb0 = thetaProb;
                if 0
                    sbjk = 1;
                end
                
                if FORCEEXPEVEN
                    if  1 %~ONLYP2
                        expandNum = ConfigParam.expectation_theta_expand_num;
                        
                        if exist('pltfrm_1','var')
                            pltfrm_1 = [pltfrm_1(1):ConfigParam.theta_sample_step2:pltfrm_1(end)];
                        else
                            
                            if 1
                                if keyFrameLen <= 3
                                    sc = Sc1;
                                else
                                    sc = Sc2;
                                end
                            else
                                
                                sc = Sc;
                            end
                            
                            thetaProb = thetaProb./sum(thetaProb);
                            
                            rati = Rati1;
                            thetaProb1 = thetaProb./max(thetaProb);
                            thetaProb2 = thetaProb1(thetaProb1 >= rati);
                            thetaProb2 = thetaProb2./sum(thetaProb2);
                            
                            if ~ONLYP2_1_Max
                                if 1
                                    if ~UseNewP2Only
                                        thetaExp_2 = dot(rad2deg(thetaSamp(thetaProb1 >= rati)), thetaProb2);
                                    else
                                        if keyFrameLen == 2
                                            thetaExp_2 = rad2deg(thetaExp_tmpK2C);
                                        else
                                            thetaExp_2 = dot(rad2deg(thetaSamp(thetaProb1 >= rati)), thetaProb2);
                                        end
                                    end
                                else
                                    thetaExp_2 = rad2deg(angOpt - k2cRef);
                                end
                                
                            else
                                thetaExp_2 = rad2deg(angOpt3 - k2cRef);
                            end
                            
                            
                            if EXPZERO
                                thetaExp_2 = 0;
                                
                            end
                            
                            if ForceThetaExpZero
                                thetaExp_2 = 0;
                            end
                            
                            
                            if ~SOFTP2
                                pltfrm_ = [thetaExp_2 - min(110.2,rad2deg(sc*ConfigParam.theta_sample_step)) thetaExp_2 + min(110.2,rad2deg(sc*ConfigParam.theta_sample_step))];
                            else
                                pltfrm_ = [thetaExp_2 - min(0.2,rad2deg(sc*ConfigParam.theta_sample_step)) thetaExp_2 + min(0.2,rad2deg(sc*ConfigParam.theta_sample_step))];
                            end
                            pltfrm_1 = [deg2rad(pltfrm_(1)) : ConfigParam.theta_sample_step : deg2rad(pltfrm_(end))] + k2cRef;
                            
                            
                            pltfrm_1 = [pltfrm_1(1):ConfigParam.theta_sample_step2:pltfrm_1(end)];
                        end
                        
                        
                        %                                     expandNum = (length(thetaProbUse__) - length(pltfrm_1))/2;
                        expandNum = (length(thetaSamp0) - length(pltfrm_1))/2;
                        
                        thetaProbBeforeNewPlatform = thetaProb;
                        thetaRngBeforeNewPlatform = thetaRng;
                        thetaSampBeforeNewPlatform = thetaSamp;
                        
                        thetaRng = [(-ConfigParam.theta_sample_step2.*[expandNum:-1:1] + pltfrm_1(1)) pltfrm_1 (pltfrm_1(end) + ConfigParam.theta_sample_step2.*[1 : expandNum])];
                        
                        if 0
                            thetaProb = interp1(thetaRngBeforeNewPlatform, thetaProbBeforeNewPlatform, thetaRng);
                            thetaProb = interp1(thetaRngBeforeNewPlatform-k2cRef, thetaProbBeforeNewPlatform, thetaRng-k2cRef);
                        else
                            thetaProb = interp1(round(rad2deg(thetaSamp0),3),thetaProbBeforeNewPlatform,round(rad2deg(thetaRng - k2cRef) ,3));
                        end
                        
                        thetaProb(isnan(thetaProb)) = min(thetaProbBeforeNewPlatform);
                        if 0
                            figure,plot(rad2deg(thetaRngBeforeNewPlatform), thetaProbBeforeNewPlatform);hold on;plot(rad2deg(thetaRng), thetaProb);
                        end
                        
                        doHalfPlatform = 0;
                        
                        
                        if doHalfPlatform
                            thetaRng = thetaRng(1:(length(thetaRng) + 1)/2 + 1);
                        end
                        
                        thetaSamp = thetaRng - k2cRef;
                        
                        
                        if doHalfPlatform
                            thetaSamp = thetaSamp(1:(length(thetaSamp) + 1)/2 + 1);
                        end
                    end
                    
                    
                    if 1 % ~ONLYP2
                        if 1
                            
                            
                            
                            
                            if  1 % ONLYP2
                                
                                if 0 %~ONLYP2_2
                                    [thetaProb,idM,angOpt, ReprojErrVecTmp, ReprojErrVecRTmp, ReprojErrVecTmpMatT, ReprojErrVecRTmpMatT,ValidFeatFlag,ValidFeatFlag0,ValidFeatFlagFusion,ValidFeatFlagQuant,ProbReprojVecAll, ProbReprojVecRAll, PixKeyVecTmp, PixKeyVecRTmp, Probb, ...
                                        validFeatFlagFusion,validFeatFlag, ProbReprojVec, ProbReprojVecR,bb,b,pixKeyVecTmpXCoordMatStack,pixKeyVecTmpYCoordMatStack,pixKeyVecRTmpXCoordMatStack,pixKeyVecRTmpYCoordMatStack]...
                                        = ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, pt2dCur, pt2dCurR,ProbZ, ProbZTmp_norm, imgCur, imgCurR,thetaSamp,r_cam,tx, ty, tz);
                                else
                                    
                                    
                                    ConfigParam.reproj_sigma = ConfigParam.reproj_sigma0 ;
                                    ConfigParam.reproj_sigma_right = ConfigParam.reproj_sigma_right0;
                                    
                                    
                                    
                                    
                                    [thetaProb__,idM__,angOpt__, ReprojErrVecTmp, ReprojErrVecRTmp, ReprojErrVecTmpMatT, ReprojErrVecRTmpMatT,ValidFeatFlag,ValidFeatFlag0,ValidFeatFlagFusion,ValidFeatFlagQuant,ProbReprojVecAll, ProbReprojVecRAll, PixKeyVecTmp, PixKeyVecRTmp, Probb, ...
                                        validFeatFlagFusion,validFeatFlag, ProbReprojVec, ProbReprojVecR,bb,b,pixKeyVecTmpXCoordMatStack,pixKeyVecTmpYCoordMatStack,pixKeyVecRTmpXCoordMatStack,pixKeyVecRTmpYCoordMatStack,~, errEpiline, angOptEpi,depthC2KInd_ind]...
                                        = ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, pt2dCur, pt2dCurR,ProbZ, ProbZTmp_norm, imgCur, imgCurR,thetaSamp,r_cam,tx, ty, tz);
                                    
                                    ProbReprojVecAll00 = ProbReprojVecAll;
                                    ProbReprojVecAll0 = ProbReprojVecAll;
                                    
                                    
                                    
                                    if 0  % 20191108  change update p2
                                        dbkjk = 1;
                                    elseif 1
                                        if ~exist('DispKeyInPrvRng','var')
                                            DispKeyInPrvRng = [];
                                        end
                                        doUNT = 1;
                                        updateP2;
                                        
                                    end
                                    
                                    wqgeqjvbh = 1;
                                    
                                    if ~exist('thetaProbOnlyP2','var')
                                        thetaProbOnlyP2 = thetaProb;
                                    end
                                    
                                    if 1
                                        
                                        
                                        if 0
                                            thetaProb = thetaProb(ismember(round(rad2deg(thetaSamp0),3),round(rad2deg(thetaSamp) ,3)));
                                        elseif 0
                                            thetaSampReplay = thetaSamp;
                                            thetaProb = interp1(round(rad2deg(thetaSamp0),3),thetaProbOnlyP2,round(rad2deg(thetaSamp) ,3));
                                            thetaProb(isnan(thetaProb)) = min(thetaProbOnlyP2);
                                            
                                            if 0
                                                %                                                 figure, plot(rad2deg(thetaSamp0), thetaProb0);hold on;plot(rad2deg(thetaSamp), thetaProb,'-x');legend('orig','interp')
                                                figure, plot(rad2deg(thetaSamp0), thetaProbOnlyP2);hold on;plot(rad2deg(thetaSamp), thetaProb,'-x');legend('orig','interp')
                                            end
                                        else
                                            thetaSampReplay = thetaSamp;
                                            svkh = 1;
                                        end
                                    else
                                        thetaSampReplay = thetaSamp;
                                        thetaProb = thetaProb__;
                                        idM = idM__;
                                        angOpt = angOpt__;
                                        angOpt3 = angOpt;
                                        if 0
                                            figure,plot(thetaSamp,thetaProb__);hold on;plot(thetaSamp0, thetaProb0)
                                        end
                                    end
                                end
                            end
                            
                            ProbZOld = repmat(ProbZ(:),length(thetaProb),1);
                            ProbZOld = reshape(ProbZOld, size(ProbZ,1), size(ProbZ,2), length(thetaProb));
                            ProbZOld = permute(ProbZOld, [3 1 2]);
                            
                            
                            pt2dCurXMat = repmat(pt2dCur(:,1),length(thetaSamp)*length(disparityRng),1);
                            pt2dCurXMat = reshape(pt2dCurXMat,size(pt2dCur,1), length(thetaSamp), length(disparityRng));
                            pt2dCurXMat = permute(pt2dCurXMat,[2 1 3]);
                            
                            pt2dCurYMat = repmat(pt2dCur(:,2),length(thetaSamp)*length(disparityRng),1);
                            pt2dCurYMat = reshape(pt2dCurYMat,size(pt2dCur,1), length(thetaSamp), length(disparityRng));
                            pt2dCurYMat = permute(pt2dCurYMat,[2 1 3]);
                            
                            
                            
                            
                            pt2dCurXMatR = repmat(pt2dCurR(:,1),length(thetaSamp)*length(disparityRng),1);
                            pt2dCurXMatR = reshape(pt2dCurXMatR,size(pt2dCurR,1), length(thetaSamp), length(disparityRng));
                            pt2dCurXMatR = permute(pt2dCurXMatR,[2 1 3]);
                            
                            pt2dCurYMatR = repmat(pt2dCurR(:,2),length(thetaSamp)*length(disparityRng),1);
                            pt2dCurYMatR = reshape(pt2dCurYMatR,size(pt2dCurR,1), length(thetaSamp), length(disparityRng));
                            pt2dCurYMatR = permute(pt2dCurYMatR,[2 1 3]);
                            
                            modulate_curve_quant_size = ConfigParam.modulate_curve_quant_size;
                            
                            
                            
                            
                            
                        else
                            [thetaProb,idM,angOpt, ReprojErrVecTmp, ReprojErrVecRTmp, ReprojErrVecTmpMatT, ReprojErrVecRTmpMatT,ValidFeatFlag,ValidFeatFlag0,ValidFeatFlagFusion,ValidFeatFlagQuant,ProbReprojVecAll, ProbReprojVecRAll, ~, ~, ~, ...
                                ~,~, ~, ~,~,~]...
                                = ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, pt2dCur, pt2dCurR,ProbZ, ProbZTmp_norm, imgCur, imgCurR,thetaSamp,r_cam,tx, ty, tz);
                        end
                        
                        
                        
                        
                        thetaRange = thetaRng;
                        [thetaGrid, disparityGrid] = meshgrid(thetaRange, disparityRng);
                        ThetaDisp = [thetaGrid(:) disparityGrid(:)];
                        
                        Prob = zeros(length(thetaRng),length(inlierId),length(disparityRng));
                        
                        thetaProb0 = thetaProb;
                        idM_Vec00 = find(ismember(thetaRng, pltfrm_1));
                        idM_Vec00Diff = idM_Vec00;
                        
                    else
                        asvag = 1;
                        
                    end
                    
                    
                    xx1 = (round(pixKeyVecTmpXCoordMatStack./modulate_curve_quant_size).*modulate_curve_quant_size); yy1 = (round(pixKeyVecTmpYCoordMatStack./modulate_curve_quant_size).*modulate_curve_quant_size);
                    xx1 = permute(xx1,[1 3 2])./modulate_curve_quant_size; yy1 = permute(yy1,[1 3 2])./modulate_curve_quant_size;
                    zz1 = permute(ProbZOld,[1 3 2]);
                    tempTable0 = zeros((1./ConfigParam.modulate_curve_quant_size)*size(rgb2gray(imgCur)));
                    errList = zeros(length(inlierId),2);
                    imgSize = size(rgb2gray(imgCur));
                    if 0
                        figure(FigBase + 2);clf,hist(errList(errList(:,1)>0,1) - errList(errList(:,1)>0,2),100);title('abs(lk-peak) - abs(gt-peak)')
                        % % %                                     saveas(gcf,fullfile(probPath,sprintf(strcat('moduCurve_',probDir(end-14:end),'____',sprintf('%04d',sum(KeyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('moduCurve_',probDir(end-14:end),'____',sprintf('%04d',(sum(KeyFrameFlagList))),'__*.png'))))+1 + FigBase)));
                    end
                    asdv = 1;
                    
                    %                                 xx1 = pixKeyVecTmpXCoordMatStack(idM_Vec00,:,:);yy1 = pixKeyVecTmpYCoordMatStack(idM_Vec00,:,:);
                    %                                 xx1 = round(xx1./modulate_curve_quant_size).*modulate_curve_quant_size; yy1 = round(yy1./modulate_curve_quant_size).*modulate_curve_quant_size;
                    %                                 xx1 = permute(xx1, [2 1 3]); yy1 = permute(yy1, [2 1 3]);
                    %
                    %                                 zz1 = ProbZOld(idM_Vec00,:,:); zz1 = permute(zz1, [2 1 3]);
                    %
                    %                                 tempTable1 = zeros((1./ConfigParam.modulate_curve_quant_size).^2*size(rgb2gray(imgCur),1)*size(rgb2gray(imgCur),2)*length(inlierId),1);
                    %                                 tempTable1 = reshape(tempTable1, size(rgb2gray(imgCur),2),size(rgb2gray(imgCur),1),[]);
                    %                                 tempTable1 = cat(length(inlierId),tempTable1);
                    
                    if 0
                        aekja = 1;
                    end
                    
                    figure(FigBase + 19);
                    if keyFrameLen == 2
                        probDir = probPath;  % fullfile(pwd, 'prob');
                        %                                     saveas(gcf,fullfile(probDir,sprintf('prob_%05d.png',length(dir(fullfile(probDir,'prob_*.png')))+1)));
                        %                                     saveas(gcf,fullfile(probDir,sprintf('prob_%05d.fig',length(dir(fullfile(probDir,'prob_*.fig')))+1)));
                        clf;
                    end
                    
                    
                    
                    figure(FigBase + 19),%clf,%subplot(1,2,1);
                    %                                 plot(rad2deg(thetaSamp), thetaProb);hold on;plot(rad2deg(thetaSamp(find(thetaProb1 > 0.3))), thetaProb(find(thetaProb1 > 0.3)), '.b');grid on;
                    %                                 plot(rad2deg(thetaSamp + (k2cRef - k2cRef0) ), thetaProb);hold on;plot(rad2deg(thetaSamp(idM_Vec00)+ (k2cRef - k2cRef0)), thetaProb(idM_Vec00), '.r');
                    if ~NewFlow
                        plot(rad2deg(thetaRng + ( - k2cRef0) ), thetaProb);hold on;plot(rad2deg(thetaRng(idM_Vec00)+ ( - k2cRef0)), thetaProb(idM_Vec00), '.r');
                        plot(rad2deg(angOpt3 - k2cRef0), max(thetaProb), 'ob');
                    else
                        plot(rad2deg(thetaRngBeforeNewPlatform1 + ( - k2cRef0) ), thetaProbBeforeNewPlatform1);hold on;plot(rad2deg(thetaRngBeforeNewPlatform1(idM_Vec00)+ ( - k2cRef0)), thetaProbBeforeNewPlatform1(idM_Vec00), '.r');
                        plot(rad2deg(angOpt3 - k2cRef0), max(thetaProbBeforeNewPlatform1), 'ob');
                    end
                    grid on;
                    %                                 subplot(1,2,2);plot(rad2deg(thetaSamp), thetaProb);hold on;plot(rad2deg(thetaSamp(idM_Vec11)), thetaProb(idM_Vec11), '.r');grid ongrid on;
                    if 1 %keyFrameLen <= 3
                        saveas(gcf,fullfile(probPath,sprintf(strcat('cutThetaDiff_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('cutThetaDiff_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__*.png'))))+1 + FigBase)));
                    end
                    
                    thetaProb1 = thetaProb;
                    
                    
                    
                    if FORCETHETAPLATFORM1
                        thetaProb(idM_Vec00) = max(thetaProb);
                        
                    end
                    thetaProb = thetaProb./sum(thetaProb);
                    
                    
                    
                    
                end
                
                
                figure(FigBase + 31);
                if jkl == 1  %DEPTHITERNUM
                    if keyFrameLen == 2
                        probDir = probPath;  % fullfile(pwd, 'prob');
                        %                                     saveas(gcf,fullfile(probDir,sprintf('prob_%05d.png',length(dir(fullfile(probDir,'prob_*.png')))+1)));
                        %                                     saveas(gcf,fullfile(probDir,sprintf('prob_%05d.fig',length(dir(fullfile(probDir,'prob_*.fig')))+1)));
                        clf;
                    end
                end
                %                             figure(FigBase + 31),hold on;plot(rad2deg(thetaSamp), thetaProb);plot(rad2deg(thetaSamp(idM)), thetaProb(idM),'*r'); % title(sprintf('k2cRef: %0.5f\nk2cPnP: %0.5f\nk2cProb: %0.5f\nsigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f',rad2deg(k2cRef), rad2deg(k2cPnp), rad2deg(thetaRng(idM)),ConfigParam.reproj_sigma,ConfigParam.reproj_beta,ConfigParam.reproj_sigma_right,ConfigParam.reproj_beta_right));grid on;
                %                             title(sprintf('sigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f',ConfigParam.reproj_sigma,ConfigParam.reproj_beta,ConfigParam.reproj_sigma_right,ConfigParam.reproj_beta_right));
                
                
                b2cPmat = GetPctBody2Cam(CoordSysAligner, 1);
                if 0
                    k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(thetaRng(idM)),zeros(3,1));
                else
%                     k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cRef0),zeros(3,1));
                    k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cRef_00),zeros(3,1));
                end
                k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
                
                metricPrevPtCcsGT = intrMat\HomoCoord(Pix',1);
                metricPrevPtCcsGT = normc(metricPrevPtCcsGT);
                if 0
                    scaleAllGT = depthListGT(inlierId)./metricPrevPtCcsGT(3,:)';
                else
                    dispGTComp = dispList(:) - disparityErrorRound;
                    depthListGTComp = intrMat(1,1).*norm(CamModel.transVec1To2)./(dispGTComp + (princpPtR(1) - princpPtL(1)));
                    scaleAllGT = depthListGTComp./metricPrevPtCcsGT(3,:)';
                end
                XYZ = [repmat(scaleAllGT',3,1).*metricPrevPtCcsGT];
                
                
                k2cT = -b2cPmat.transformMat(1:3,1:3)*k2cBodyPmat.transformMat(1:3,1:3)*b2cPmat.transformMat(1:3,1:3)'*b2cPmat.transformMat(1:3,4) + b2cPmat.transformMat(1:3,4);
                k2cCam = [k2cCamPmat.transformMat(1:3,1:3) k2cT;0 0 0 1];
                homocurrCcsXYZ = k2cCam*HomoCoord(XYZ,1); %pix0_GT_ = pflat(intrMat*homocurrCcsXYZALL_(1:3,:))';
                pixGT = pflat(intrMat*homocurrCcsXYZ(1:3,:))';
                pixGT = pixGT(:,1:2);
                pixGTR = [pixGT(:,1) - dispMapCurGTList pixGT(:,2)];
                
                
                
                if 0
                    tmpProb22 = permute(Probb(idM,:,:),[2 3 1]);
                    maxx2 = (max(tmpProb22'));
                    maxx2(isnan(maxx2) | isinf(maxx2)) = [];
                    maxxsum2 = sum((maxx2));
                    %                            figure(FigBase + 33),clf;subplot(2,1,1);plot(ZZVec1(randId,:),ProbZ(randId,:));hold on;plot(ZZVec1(randId,:),probReprojVec(randId,:));
                    %                                           subplot(2,1,2);plot(ZZVec1(randId,:),tmpProb(:)');
                    
                    
                    figure(FigBase + 36),clf;subplot(4,1,1);plot(ProbZ');title(num2str(rad2deg(thetaRng(idM)))); subplot(4,1,2);plot(ProbReprojVec{idM}');subplot(4,1,3);plot(ProbReprojVecR{idM}'); subplot(4,1,4);plot(tmpProb22');title(num2str(maxxsum2));
                    
                    
                    %                             figure,plot([ProbReprojVecR{idM}(1,:) ; ProbReprojVec{idM}(1,:); (ProbReprojVecR{idM}(1,:) + ProbReprojVec{idM}(1,:)); (ProbReprojVecR{idM}(1,:) .* ProbReprojVec{idM}(1,:)).*1000]')
                    
                    
                    figure(FigBase + 37),clf;subplot(1,2,1);imshow(imgCur);hold on;plot(PixKeyVecTmp{idM}(:,1), PixKeyVecTmp{idM}(:,2),'.g');plot(pt2dCur(:,1), pt2dCur(:,2),'.r');title(num2str(rad2deg(thetaRng(idM))));
                    subplot(1,2,2);imshow(imgCurR);hold on;plot(PixKeyVecRTmp{idM}(:,1), PixKeyVecRTmp{idM}(:,2),'.g');plot(pt2dCurR(:,1), pt2dCurR(:,2),'.r');title('cur right');
                end
                %                     figure(FigBase + 31),hold on;plot( thetaProb);title(sprintf('k2cRef: %0.5f\nk2cPnP: %0.5f\nk2cProb: %0.5f',rad2deg(k2cRef), rad2deg(k2cPnp), rad2deg(thetaRng(idM))));
                %                     drawnow;
                
                
                ProbZAll = repmat(ProbZ(:),length(thetaProb),1);
                ProbZAll = reshape(ProbZAll, size(ProbZ,1), size(ProbZ,2), length(thetaProb));
                ProbZAll = permute(ProbZAll, [3 1 2]);
                ProbZAll0 = ProbZAll;
                
                
                if ~USEthetaProb0
                    thetaProbAll = repmat(thetaProb', size(ProbZ,1)*size(ProbZ,2),1);
                else
                    thetaProbAll = repmat(thetaProb0', size(ProbZ,1)*size(ProbZ,2),1);
                end
                thetaProbAll = reshape(thetaProbAll, length(thetaProb), size(ProbZ,1), size(ProbZ,2));
                
                %                 errTheta = thetaProbAll(:,1,2) - thetaProbAll(:,34,56);
                %                 errZ = ProbZAll(1,:,:) - ProbZAll(10,:,:); errZ = errZ(:);
                try
                    errTheta = thetaProbAll(:,1,2) - thetaProbAll(:,7,3);
                    errZ = ProbZAll(1,:,:) - ProbZAll(6,:,:); errZ = errZ(:);
                catch
                    askjbv = 1;
                end
                if 0 %BOTHLR
                    %                                 ValidFeatFlag((length(thetaRng)+1)/2,:) = 1;
                    ValidFeatFlag(idM,:) = 1;
                end
                ValidFeatFlag_bak = ValidFeatFlag;
                idM_Vec00 = find(thetaProb./max(thetaProb) > ConfigParam.theta_prob_cutoff_threshold) ;
                
                if 0
                    ValidFeatFlagFusionWithin = ValidFeatFlagFusion.*ValidFeatFlagQuant;
                elseif 1
                    idM_Vec00 = find(thetaProb./max(thetaProb) > ConfigParam.theta_prob_cutoff_threshold) ;
                    ValidFeatFlagTmpCut = zeros(size(ValidFeatFlag));
                    if 1
                        ValidFeatFlagTmpCut(idM_Vec00,:) = 1;
                        ValidFeatFlagFusionWithin = ValidFeatFlagFusion.*ValidFeatFlagTmpCut;
                    else
                        idM_Vec000  = find(thetaSamp >= -ConfigParam.ref_theta_trust_margin & thetaSamp <= ConfigParam.ref_theta_trust_margin);
                        ValidFeatFlagTmpCut(idM_Vec000,:) = 1;
                        ValidFeatFlagFusionWithin = ValidFeatFlagFusion.*ValidFeatFlagTmpCut;
                    end
                else % 20190925
                    ValidFeatFlagFusionWithin = ValidFeatFlagFusion;
                end
                [asd,weg] = max(ValidFeatFlagFusionWithin);
                asdMat = repmat(asd, size(ValidFeatFlagFusionWithin,1),1);
                BB_ = ValidFeatFlagFusionWithin./asdMat;
                BB0 = BB_./repmat(sum(BB_),size(BB_,1),1);
                if 1
                    BB = BB_;
                else
                    BB = BB0;
                end
                try
                    errrrr = (BB(:,10) - ValidFeatFlagFusionWithin(:,10)./max(ValidFeatFlagFusionWithin(:,10)));
                catch
                    asbvkj = 1;
                end
                %                             wegCoord = [[1:size(bb,2)]' weg'];
                %                             ind = sub2ind(size(bb), wegCoord);
                
                
                if ~CUTTHETAANDFORCEPLATFORM
                    if 1 %~HARDCUT
                        if FUSIONCUT
                            ValidFeatFlag = immultiply(BB,BB > ConfigParam.probZ_ratio_threshold);
                            %                                     ValidFeatFlag = BB;
                        end
                    end
                else
                    
                    % % %                                 diffTheta = abs(diff(thetaProb./max(thetaProb)));
                    % % %                                 [minv,mini1]=findpeaks(diffTheta,'SortStr','descend');
                    % % %                                 mini1 = sort(mini1(1:2), 'ascend');
                    % % %                                 idThetaDiff  = find(diffTheta <= ConfigParam.theta_diff_margin);
                    % % %                                 idThetaDiff = intersect(idThetaDiff, mini1(1):mini1(2));
                    % % %                                 idThetaDiffExtend = [idThetaDiff(1) : idThetaDiff(end) + 5];
                    % % %                                 [~, distThetaDiff] = NormalizeVector(repmat(diffTheta(idThetaDiffExtend(1)),length(idThetaDiffExtend),1) - diffTheta(idThetaDiffExtend)');
                    % % %                                 [closestVal] = min(distThetaDiff - diffTheta(idThetaDiffExtend(1)));
                    % % %                                 closestId = max(find(distThetaDiff == closestVal));
                    % % %
                    % % %                                 cutId = [idThetaDiff(1) idThetaDiffExtend(closestId) + 1];
                    % % %
                    % % %                                 idM_Vec00 = [cutId(1) : cutId(2)];
                    %                                 ValidFeatFlag = immultiply(BB,BB > ConfigParam.probZ_ratio_threshold);
                    ValidFeatFlag = zeros(size(BB));
                    ValidFeatFlag(idM_Vec00,:) = 1;
                    %                                     ValidFeatFlag = BB;
                end
                
                
                ValidFeatFlag_backup = ValidFeatFlag;
                
                
                if 0 % 20190927 try ref_theta_margin
                    idM_Vec0 = find(thetaProb./max(thetaProb) > ConfigParam.theta_prob_cutoff_threshold) ;
                    
                    ValidFeatFlagTmp0 = zeros(size(ValidFeatFlag));
                    ValidFeatFlagTmp0(idM_Vec0,:) = 1;
                    ValidFeatFlag = ValidFeatFlagTmp0;
                    
                elseif FORCEEVENPLATFORM % 20190927 try ref_theta_margin
                    
                    ValidFeatFlagTmpCut000 = zeros(size(ValidFeatFlag));
                    idM_Vec00  = find(thetaSamp >= -ConfigParam.ref_theta_trust_margin - 0 & thetaSamp <= ConfigParam.ref_theta_trust_margin);
                    thetaProb(idM_Vec00) = max(thetaProb);
                    thetaProb = thetaProb./sum(thetaProb);
                    ValidFeatFlagTmpCut000(idM_Vec00,:) = 1;
                    ValidFeatFlag = ValidFeatFlagTmpCut000;
                    
                    if ~USEthetaProb0
                        thetaProbAll = repmat(thetaProb', size(ProbZ,1)*size(ProbZ,2),1);
                    else
                        thetaProbAll = repmat(thetaProb0', size(ProbZ,1)*size(ProbZ,2),1);
                    end
                    
                    thetaProbAll = reshape(thetaProbAll, length(thetaProb), size(ProbZ,1), size(ProbZ,2));
                    
                else
                    asvdj = 1;
                end
                
                if DIFFTHETATHR
                    ValidFeatFlag = zeros(size(BB));
                    ValidFeatFlag(idM_Vec00Diff,:) = 1;
                    
                end
                
                if 0
                    figure(FigBase + 3),clf;imshow(ValidFeatFlag, [])
                    saveas(gcf,fullfile(probPath,sprintf(strcat('cutTheta_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('cutTheta_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__*.png'))))+1 + FigBase)));
                end
                
                
                
                ValidFeatFlagNew = ProbZTmp_norm > ConfigParam.probZ_ratio_threshold;
                ValidFeatFlagAll0 = repmat(ValidFeatFlag(:),size(ProbZ,2),1);
                ValidFeatFlagAll0 = reshape(ValidFeatFlagAll0, length(thetaRng),length(inlierId),size(ProbZ,2));
                
                ValidFeatFlagAll00 = repmat(ValidFeatFlagNew(:),length(thetaRng),1);
                ValidFeatFlagAll00 = reshape(ValidFeatFlagAll00, length(inlierId),size(ProbZ,2),length(thetaRng));
                ValidFeatFlagAll00 = permute(ValidFeatFlagAll00, [3 1 2]);
                
                if 1
                    if WITHINEPILINE
                        ValidFeatFlagAll = ValidFeatFlagAll0;
                    else
                        ValidFeatFlagAll = ones(size(ValidFeatFlagAll0));
                    end
                else
                    if WITHINEPILINE
                        %                                     ValidFeatFlagAll = ValidFeatFlagAll00.*ValidFeatFlagAll0;
                        ValidFeatFlagAll = ValidFeatFlagAll00;
                    else
                        ValidFeatFlagAll = ones(size(ValidFeatFlagAll00));
                    end
                end
                ValidFeatFlagAll000 = ValidFeatFlagAll;
                
                
                ValidFeatFlagAll0000 = ValidFeatFlagAll;
                errValidFeatAll = abs(ValidFeatFlagAll(:,:,1) - ValidFeatFlagAll(:,:,end));
                errValidFeat = abs(ValidFeatFlagAll0(:,:,1) - ValidFeatFlagAll0(:,:,end));
                errValidFeat2 = abs(ValidFeatFlagAll00(1,:,:) - ValidFeatFlagAll00(end,:,:));
                errValidFeat2 = permute(errValidFeat2, [2 3 1]);
                
                
                ProbReprojVecAll_New = ProbReprojVecAll.*ValidFeatFlagAll;
                ProbReprojVecRAll_New = ProbReprojVecRAll.*ValidFeatFlagAll;
                ProbReprojVecLRAll_New = ProbReprojVecAll.*ProbReprojVecRAll.*ValidFeatFlagAll;
                
                thetaProbAll_New = thetaProbAll.*ValidFeatFlagAll;
                thetaProbAll_New0 = thetaProbAll_New;
                
                
                
                if ~FORCETHETAPLATFORM1
                    if 0
                        ValidFeatFlagAll = ones(size(ValidFeatFlagAll));
                    else
                        ValidFeatFlag__ = zeros(size(BB));
                        ValidFeatFlag__(idM_Vec00Diff,:) = 1;
                        ValidFeatFlagAll0__ = repmat(ValidFeatFlag__(:),size(ProbZ,2),1);
                        ValidFeatFlagAll0__ = reshape(ValidFeatFlagAll0__, length(thetaRng),length(inlierId),size(ProbZ,2));
                        
                        ValidFeatFlagAll = ValidFeatFlagAll0__;
                    end
                end
                
                if 0
                    thetaProbTmp = thetaProbAll(:,1,1);
                    %                                 figure,plot(rad2deg(thetaRng), thetaProbTmp);hold on;plot(rad2deg(thetaRng(idM_Vec00)), thetaProbTmp(idM_Vec00),'or');plot(rad2deg(angOpt), max(thetaProbTmp),'*g');
                    figure,plot(rad2deg(thetaRng), thetaProbTmp);hold on;plot(rad2deg(thetaRng(idM_Vec00Diff)), thetaProbTmp(idM_Vec00Diff),'or');plot(rad2deg(angOpt), max(thetaProbTmp),'*g');
                    figure,imshow(ValidFeatFlagAll(:,:,1))
                end
                
                
                
                
                if USELEFTXRIGHT
                    updatedProbZ = ProbZAll.*ProbReprojVecAll.*thetaProbAll.*ProbReprojVecRAll.*ValidFeatFlagAll;
                    %                                 updatedProbZ = ProbZAll.*ProbReprojVecAll.*ProbReprojVecRAll;
                    %                                 updatedProbZ = ProbZAll.*thetaProbAll.*(ProbReprojVecAll + ProbReprojVecRAll);
                end
                
                if USELEFTPLUSRIGHT
                    %                                 updatedProbZ = ProbZAll.*ProbReprojVecAll.*thetaProbAll.*ProbReprojVecRAll;
                    %                                 updatedProbZ = ProbZAll.*ProbReprojVecRAll;
                    updatedProbZ = ProbZAll.*thetaProbAll.*(ProbReprojVecAll + ProbReprojVecRAll).*ValidFeatFlagAll;
                end
                
                if USERIGHT
                    %                                 updatedProbZ = ProbZAll.*ProbReprojVecAll.*thetaProbAll.*ProbReprojVecRAll;
                    %                                 updatedProbZ = ProbZAll.*ProbReprojVecAll.*ProbReprojVecRAll;
                    updatedProbZ = ProbZAll.*thetaProbAll.*ProbReprojVecRAll.*ValidFeatFlagAll;
                end
                
                if USELEFT
                    updatedProbZ = ProbZAll.*ProbReprojVecAll.*thetaProbAll.*ValidFeatFlagAll;
                end
                
                updatedProbZ0 = updatedProbZ;
                
                
                updatedProbZSum = sum(updatedProbZ);
                updatedProbZSum = permute(updatedProbZSum,  [2 3 1]);
                updatedProbZSum = updatedProbZSum./repmat(max(updatedProbZSum')',1,size(updatedProbZSum,2));
                
                updatedProbZSum00_0 = updatedProbZSum;
                if keyFrameLen <= FrmNumWithGloden + 1
                    
                    dltDispMat = DispRng - DispRng(:,(size(DispRng,2)+1)/2);
                    
                    %                                 boundDispId = find(abs(round(disparityRng,3)) > 0.5);
                    
                    updatedProbZSum(updatedProbZSum < ProbZCutOffThr & round(abs(dltDispMat),3) > 0.6) = 0;
                    updatedProbZSum = updatedProbZSum./repmat(max(updatedProbZSum')',1,size(updatedProbZSum,2));
                end
                
                updatedProbZSum0 = updatedProbZSum;
                
                if 1  % ONLYP2_2
                    if 0
                        sbjvh = 1;
                    else
                        
                        
                        ConfigParam.reproj_sigma = ConfigParam.reproj_sigma0 * ConfigParam.reproj_sigma_scale;
                        ConfigParam.reproj_sigma_right = ConfigParam.reproj_sigma_right0 * ConfigParam.reproj_sigma_scale_right;
                        
                        thetaRngBefore = thetaRng;
                        thetaSampBefore = thetaSamp;
                        if ~NewFlow
                            
                            for ft = 1 : IterThetaNum
                                
                                
                                
                                if 1 % 20191123 use ExpCoord_OnlyP2 to update angOpt
                                    [thetaProb,idM,angOpt, ReprojErrVecTmp, ReprojErrVecRTmp, ReprojErrVecTmpMatT, ReprojErrVecRTmpMatT,ValidFeatFlag,ValidFeatFlag0,ValidFeatFlagFusion,ValidFeatFlagQuant,ProbReprojVecAll, ProbReprojVecRAll, PixKeyVecTmp, PixKeyVecRTmp, Probb, ...
                                        validFeatFlagFusion,validFeatFlag, ProbReprojVec, ProbReprojVecR,bb,b,pixKeyVecTmpXCoordMatStack,pixKeyVecTmpYCoordMatStack,pixKeyVecRTmpXCoordMatStack,pixKeyVecRTmpYCoordMatStack, thetaExp_tmp,errEpiline, angOptEpi,depthC2KInd_ind]...
                                        = ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, pt2dCur, pt2dCurR,updatedProbZSum0./max(updatedProbZSum0(:)), ProbZTmp_norm, imgCur, imgCurR,thetaSamp,r_cam,tx, ty, tz);
                                else
                                    [thetaProb,idM,angOpt, ReprojErrVecTmp, ReprojErrVecRTmp, ReprojErrVecTmpMatT, ReprojErrVecRTmpMatT,ValidFeatFlag,ValidFeatFlag0,ValidFeatFlagFusion,ValidFeatFlagQuant,ProbReprojVecAll, ProbReprojVecRAll, PixKeyVecTmp, PixKeyVecRTmp, Probb, ...
                                        validFeatFlagFusion,validFeatFlag, ProbReprojVec, ProbReprojVecR,bb,b,pixKeyVecTmpXCoordMatStack,pixKeyVecTmpYCoordMatStack,pixKeyVecRTmpXCoordMatStack,pixKeyVecRTmpYCoordMatStack,thetaExp_tmp,errEpiline, angOptEpi,depthC2KInd_ind]...
                                        = ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, ExpCoord_OnlyP2, pt2dCurR,updatedProbZSum0./max(updatedProbZSum0(:)), ProbZTmp_norm, imgCur, imgCurR,thetaSamp,r_cam,tx, ty, tz);
                                    
                                end
                                %                                         [thetaRngNew, thetaSampNew] = ArrangeThetaRng(obj,rad2deg(angOpt - k2cRef),k2cRef,sc,thetaSamp0);
                                
                                if 0 % ft == IterThetaNum - 1
                                    thetaExp_tmp = mean([angOpt angOpt3]) - k2cRef;
                                end
                                [thetaRng, thetaSamp] = ArrangeThetaRng(obj,rad2deg(thetaExp_tmp),k2cRef,sc,thetaSamp0);
                                angOptList(ft,:) = angOpt; %rad2deg(angOpt - k2cRef0);
                                
                            end
                            
                        else
                            
                            angOptList = angOpt;
                        end
                        
                        
                        
                        
                        
                        %%
                        if 0
                            angOpt = angOpt3;
                        end
                        
                        
                        
                        
                        
                        
                        thetaRng = thetaRngBefore;
                        thetaSamp = thetaSampBefore;
                        
                        idM_Vec00OnlyP2_2 = find(ismember(thetaRng, pltfrm_1));
                        idM_Vec00 = idM_Vec00OnlyP2_2;
                        ProbReprojVecAll0 = ProbReprojVecAll;
                        %                                     angOpt = deg2rad(dot(thetaProb./sum(thetaProb),rad2deg(thetaRng)));
                    end
                    angOpt4 = angOpt;
                    
                    if 1
                        figure(FigBase + 20)
                        if keyFrameLen == 2
                            probDir = probPath;
                            %                                         clf;
                        end
                        if 0
                            figure(FigBase + 20),subplot(2,1,2);hold on;plot(rad2deg(thetaSamp+ (k2cRef - k2cRef0)), thetaProbInterp);grid on;
                            plot(rad2deg(angOpt4 - k2cRef0), max(thetaProbInterp),'or');
                        else
                            figure(FigBase + 20),subplot(2,1,2);hold on;plot(rad2deg(thetaSamp+ (k2cRef - k2cRef0)), thetaProb);grid on;title('2nd theta from p2 and z');
                            plot(rad2deg(angOpt4 - k2cRef0), max(thetaProb),'or');
                            thetaProbOut = thetaProb;
                            thetaRngOut = thetaRng;
                            
                        end  
                        saveas(gcf,fullfile(probPath,sprintf(strcat('cutThetaDiff2_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('cutThetaDiff2_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__*.png'))))+1 + FigBase)));
                        %                                     saveas(gcf,fullfile(probPath,sprintf(strcat('cutThetaDiff2_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__%05d.fig'),length(dir(fullfile(probPath,strcat('cutThetaDiff2_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__*.fig'))))+1)));
                    end
                    
                    figure(FigBase + 15)
                    
                    if 1
                        probDir = probPath;
                        clf;
                    end
                    
                    
                    if keyFrameLen == 2
%                         probDir = probPath;
%                         clf;
                        twoOnlyP2List = []; % zeros(1,1 + 2*IterThetaNum);
                    end
                    %                                 twoOnlyP2List = [twoOnlyP2List; [rad2deg([angOptList0' angOptList' k2cRef] - [k2cRef0.*ones(1,1 + 2*IterThetaNum)])] ];
                    %                                 twoOnlyP2List = [twoOnlyP2List; [rad2deg([angOptList0' angOptList' angOptEpi k2cRef] - [k2cRef0.*ones(1,2 + 2*IterThetaNum)])] ];
%                     twoOnlyP2List = [twoOnlyP2List; [rad2deg([angOptList0' angOptList' angOptEpiInit k2cRef accumP2CTemp(end)] - [k2cRef0.*ones(1,3 + 2*IterThetaNum)])] ];
                    
%                     twoOnlyP2List = [twoOnlyP2List; [rad2deg([angOptList0' angOptList' angOptEpiInit k2cRef accumP2CTemp(end)] - [k2cRef0.*ones(1,3 + 2*IterThetaNum)])] ];

                    twoOnlyP2List = [twoOnlyP2List; [rad2deg([angOptList0' angOptList' angOptEpiInit k2cRef accumP2CTemp(end)] - [k2cRef_00.*ones(1,3 + 2*IterThetaNum)])] ];
                    
                    figure(FigBase + 15);hold on;
                    plot(twoOnlyP2List(:,end-1),'-sg');
                    colorCell0 = {};
                    colorCell0{1} = '-rx';
                    colorCell0{2} = '-mx';
                    
                    for uy = 1:IterThetaNum
                        plot(twoOnlyP2List(:,uy),colorCell0{uy});
                    end
                    
                    colorCell = {};
                    colorCell{1} = '-bo';
                    colorCell{2} = '-co';
                    
                    for uy = 1:IterThetaNum
                        plot(twoOnlyP2List(:,IterThetaNum+uy),colorCell{uy});
                    end
                    plot(twoOnlyP2List(:,end-2),'-*k');
                    plot(twoOnlyP2List(:,end),'-om');
                    legend('k2cRef - k2cRef0','1st theta from p2 and z','2nd theta', 'epipolar line', 'accum p2c','Location','southwest');title('ang - ref (deg)');
                    saveas(gcf,fullfile(probPath,sprintf(strcat('twoOnlyP2List_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('twoOnlyP2List_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__*.png'))))+1 + FigBase)));
                    
                    
                    gfdgbs = 1;
                    
                    
                    %                                 [thetaProbUse, thetaProbUse_, thetaProbUse__, curPredPtIcs, weightRaw, candX, candY,candX_reshape, candY_reshape, weightRaw_reshape, idff] = UpdateNewTracking(obj, thetaSamp, thetaProb, prevFeatPtList0, ptCcsZ0, intrMat, AngleModalOrg, DepthProbility, DepthId, inlierId,DispRefineUse__);
                    
                    
                    
                    
                    
                    
                    if 0  % 20191108  change update p2
                        asvbkj = 1;
                        
                    elseif 0
                        doUNT = 1;
                        updateP2;
                        
                        
                    end
                    
                    
                    
                    
                    
                    if ~NewFlow
                        if UPDATEZ_2
                            
                            if 0
                                thetaProbUse2 = thetaProb(ismember(round(rad2deg(thetaSamp),3),round(s.configParam.angleRange ,3)));
                            else
                                %                                         thetaProbUse2 = interp1(round(rad2deg(thetaSamp),3), thetaProbInterp,round(s.configParam.angleRange ,3));
                                if 0
                                    thetaProbUse2 = interp1(round(rad2deg(thetaSamp),3), thetaProbInterp,round(rad2deg(thetaSampReplay) ,3));
                                    
                                    thetaProbUse2(isnan(thetaProbUse2)) = min(thetaProbInterp);
                                else
                                    if 0
                                        thetaProbUse2 = interp1(round(rad2deg(thetaSamp),3), thetaProb,round(rad2deg(thetaSampReplay) ,3));
                                    else
                                        thetaProbUse2 = thetaProb;
                                        
                                    end
                                    checkThetaSamp = sum(abs(thetaSampReplay - thetaSamp));
                                    
                                    
                                    thetaProbUse2(isnan(thetaProbUse2)) = min(thetaProb);
                                    
                                end
                                if 0
                                    %                                         figure, plot(rad2deg(thetaSamp), thetaProbInterp);hold on;plot(s.configParam.angleRange, thetaProbUse2,'-x');legend('orig','interp')
                                    figure, plot(rad2deg(thetaSamp), thetaProbInterp);hold on;plot(rad2deg(thetaSampReplay), thetaProbUse2,'-x');legend('orig','interp')
                                end
                                
                            end
                            thetaProbUse2 = thetaProbUse2./max(thetaProbUse2);
                            thetaProbUse2_ = thetaProbUse2;
                            if 0
                                thetaProbUse2_(thetaProbUse2 > ConfigParam.only_p2_prob_cutoff_threshold_2) = max(thetaProbUse2(thetaProbUse2 < ConfigParam.only_p2_prob_cutoff_threshold_2));
                            else
                                if FORCETHETAPLATFORM2
                                    thetaProbUse2_(idM_Vec00OnlyP2_2) = max(thetaProbUse2_);
                                end
                            end
                            thetaProbUse2__ = thetaProbUse2_./sum(thetaProbUse2_);
                            
                            thetaProb = thetaProbUse2__;
                            if 1
                                ValidFeatFlag = zeros(size(BB));
                                %                                             ValidFeatFlag(idM_Vec00Diff,:) = 1;
                                ValidFeatFlag(idM_Vec00OnlyP2_2,:) = 1;
                                
                            else
                                ValidFeatFlag = ones(size(BB));
                            end
                            
                            
                            
                            ValidFeatFlagAll_0 = repmat(ValidFeatFlag(:),size(ProbZ,2),1);
                            ValidFeatFlagAll = reshape(ValidFeatFlagAll_0, length(thetaRng),length(inlierId),size(ProbZ,2));
                            
                            
                            
                            
                            thetaProbAll = repmat(thetaProbUse2__', size(ProbZ,1)*size(ProbZ,2),1);
                            thetaProbAll = reshape(thetaProbAll, length(thetaProb), size(ProbZ,1), size(ProbZ,2));
                            
                            updatedProbZSum__ = updatedProbZSum./max(updatedProbZSum(:));
                            
                            if 1
                                ProbZAll = repmat(updatedProbZSum__(:),length(thetaProb),1);
                            else
                                ProbZAll = repmat(ProbZ(:),length(thetaProb),1);
                            end
                            ProbZAll = reshape(ProbZAll, size(ProbZ,1), size(ProbZ,2), length(thetaProb));
                            ProbZAll = permute(ProbZAll, [3 1 2]);
                            
                            if ~FORCETHETAPLATFORM2
                                %                                 ValidFeatFlagAll = ones(size(ValidFeatFlagAll));
                                
                                %                                 ValidFeatFlagFusion;
                                
                                [asd,weg] = max(ValidFeatFlagFusion);
                                asdMat = repmat(asd, size(ValidFeatFlagFusion,1),1);
                                BB_ = ValidFeatFlagFusion./asdMat;
                                BB0 = BB_./repmat(sum(BB_),size(BB_,1),1);
                                
                                
                                
                                ValidFeatFlagAll_0 = repmat(BB_(:),size(ProbZ,2),1);
                                ValidFeatFlagAll = reshape(ValidFeatFlagAll_0, length(thetaRng),length(inlierId),size(ProbZ,2));
                                
                                
                                
                            end
                            
                            
                            
                            
                            if 0
                                figure,plot(thetaProbAll(:,1,1))
                                figure,imshow(ValidFeatFlagAll(:,:,1), [])
                            end
                            
                            if USELEFTXRIGHT
                                updatedProbZ = ProbZAll.*ProbReprojVecAll.*thetaProbAll.*ProbReprojVecRAll.*ValidFeatFlagAll;
                                %                                 updatedProbZ = ProbZAll.*ProbReprojVecAll.*ProbReprojVecRAll;
                                %                                 updatedProbZ = ProbZAll.*thetaProbAll.*(ProbReprojVecAll + ProbReprojVecRAll);
                            end
                            
                            if USELEFTPLUSRIGHT
                                %                                 updatedProbZ = ProbZAll.*ProbReprojVecAll.*thetaProbAll.*ProbReprojVecRAll;
                                %                                 updatedProbZ = ProbZAll.*ProbReprojVecRAll;
                                updatedProbZ = ProbZAll.*thetaProbAll.*(ProbReprojVecAll + ProbReprojVecRAll).*ValidFeatFlagAll;
                            end
                            
                            if USERIGHT
                                %                                 updatedProbZ = ProbZAll.*ProbReprojVecAll.*thetaProbAll.*ProbReprojVecRAll;
                                %                                 updatedProbZ = ProbZAll.*ProbReprojVecAll.*ProbReprojVecRAll;
                                updatedProbZ = ProbZAll.*thetaProbAll.*ProbReprojVecRAll.*ValidFeatFlagAll;
                            end
                            
                            if USELEFT
                                updatedProbZ = ProbZAll.*ProbReprojVecAll.*thetaProbAll.*ValidFeatFlagAll;
                            end
                            
                            
                            updatedProbZSum = sum(updatedProbZ);
                            updatedProbZSum = permute(updatedProbZSum,  [2 3 1]);
                            
                            angOptFinalP2 = angOpt4;
                            if 0
                                qwa = 13;figure(FigBase + 13),clf;plot([ProbZ(qwa,:)./max(ProbZ(qwa,:)); updatedProbZSum0(qwa,:)./max(updatedProbZSum0(qwa,:)); updatedProbZSum(qwa,:)./max(updatedProbZSum(qwa,:))]');legend('orig Z','1st upadte','2nd update')
                            end
                        else
                            angOptFinalP2 = angOpt4;
                        end
                        
                    end
                else
                    angOptFinalP2 = angOpt3;
                    
                end
                
                
                % %                             else
                % %                                 angOptFinalP2 = angOpt;
                % %                                 angOpt4 = angOpt;
                % %                         end
                
                
                
                
                
                
                
                
                
                
                
                try
                    errZSum = sum(updatedProbZ(:,2,3)) - updatedProbZSum(2,3);
                catch
                    sadvkj = 1;
                end
                if 1
                    figure(FigBase + 40),clf;subplot(2,1,1),plot(disparityRng, ProbZ'./max(ProbZ(:)));title('orig ProbZ');
                    subplot(2,1,2),plot(disparityRng, updatedProbZSum'./max(updatedProbZSum(:)));%title('updated ProbZ');
                    title(sprintf('updated ProbZ\nsigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f',ConfigParam.reproj_sigma,ConfigParam.reproj_beta,ConfigParam.reproj_sigma_right,ConfigParam.reproj_beta_right));
                    saveas(gcf,fullfile(probPath,sprintf(strcat('zProb_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('zProb_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__*.png'))))+1 + FigBase)));
                end
                
                
                
                area1 = trapz(rad2deg(thetaSamp(1:idM)), thetaProb(1:idM));
                area2 = trapz(rad2deg(thetaSamp(idM:end)), thetaProb(idM:end));
                percentage = ConfigParam.theta_percentage;
                
                for uu = 2 : idM
                    area11 = trapz(rad2deg(thetaSamp(1:uu)), thetaProb(1:uu));
                    if (area11)/area1 > 1-(percentage)
                        break;
                    end
                    
                end
                ind1 = uu;
                
                for vv = idM+1 : length(thetaSamp)
                    area22 = trapz(rad2deg(thetaSamp(idM:vv)), thetaProb(idM:vv));
                    if area22/area2 > percentage
                        break;
                    end
                    
                end
                ind2 = vv;
                
                if isempty(ind1)
                    ind1 = 1;
                end
                if isempty(ind2)
                    ind2 = length(thetaRng);
                end
                
                angRng = [thetaRng(ind1)  thetaRng(ind2);];
                
                %                 pt1 = [rad2deg(thetaSamp([ind1; ind2]))', thetaProb([ind1; ind2])']';
                %                 pt2 = [rad2deg(thetaSamp([ind1; ind2]))', [0 0]']';
                %                 figure(FigBase + 31),hold on;line(pt1(:,1), pt1(:,2),'Color',[0 0 1]);
                figure(FigBase + 31),
                hold on;plot(rad2deg(thetaSamp+ (k2cRef - k2cRef0)), thetaProb);plot(rad2deg(thetaSamp(idM)+ (k2cRef - k2cRef0)), thetaProb(idM),'*r');
                hold on;plot(rad2deg(thetaSamp([ind1; ind2])+ (k2cRef - k2cRef0)), thetaProb([ind1; ind2]),'*b');
                
                plot(rad2deg(thetaSamp([idM_Vec00(1); idM_Vec00(end)])+ (k2cRef - k2cRef0)), thetaProb([idM_Vec00(1); idM_Vec00(end)]),'*g');
                if ~FORCEEXPEVEN
                    title(sprintf('p(theta) cutoff threshold: %0.5f\nmean theta ind: %0.2f / %0.2f\nk2cRef: %0.5f\nk2cPnP: %0.5f\nk2cProb: %0.5f\nsigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f',ConfigParam.theta_prob_cutoff_threshold,mean(idM_Vec00),(length(thetaSamp)+1)/2,rad2deg(k2cRef0), rad2deg(k2cPnp), rad2deg(thetaRng(idM)),ConfigParam.reproj_sigma,ConfigParam.reproj_beta,ConfigParam.reproj_sigma_right,ConfigParam.reproj_beta_right));grid on;
                else
                    title(sprintf('p(theta) cutoff threshold: %0.5f\nmean theta ind: %0.2f / %0.2f\nk2cRef: %0.5f\nk2cPnP: %0.5f\nk2cProb: %0.5f\nsigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f',ConfigParam.theta_prob_cutoff_threshold,mean(idM_Vec00),(length(thetaSamp)+1)/2,rad2deg(k2cRef0), rad2deg(k2cPnp), rad2deg(angOpt),ConfigParam.reproj_sigma,ConfigParam.reproj_beta,ConfigParam.reproj_sigma_right,ConfigParam.reproj_beta_right));grid on;
                end
                saveas(gcf,fullfile(probPath,sprintf(strcat('prob_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('prob_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__*.png'))))+1 + FigBase)));
                %                             title(sprintf('sigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f',ConfigParam.reproj_sigma,ConfigParam.reproj_beta,ConfigParam.reproj_sigma_right,ConfigParam.reproj_beta_right));
                
                if ~exist('thetaProb0','var')
                    thetaProb0 = thetaProb;
                end
                
                if DIFFTHETATHR % 0
                    %                                 ValidFeatFlag = zeros(size(BB));
                    %                                 ValidFeatFlag(idM_Vec00Diff,:) = 1;
                    idM_Vec00 = idM_Vec00Diff;
                end
                
                if 1
                    if ~FORCEEXPEVEN
                        thetaPlatformDistribution_ = [[-1 + (0 - rad2deg(thetaSamp(idM_Vec00(1)))).*2./(rad2deg(thetaSamp(idM_Vec00(end)) - thetaSamp(idM_Vec00(1)))) ] ...
                            [-1 + (dot(rad2deg(thetaSamp), thetaProb0) - rad2deg(thetaSamp(idM_Vec00(1)))).*2./(rad2deg(thetaSamp(idM_Vec00(end)) - thetaSamp(idM_Vec00(1))))]];
                    else
                        %                                     thetaPlatformDistribution_ = [[-1 + (0 - rad2deg(thetaSamp(idM_Vec00(1)))).*2./(rad2deg(thetaSamp(idM_Vec00(end)) - thetaSamp(idM_Vec00(1)))) ] ...
                        %                                                               [-1 + (thetaExp_ - rad2deg(thetaSamp(idM_Vec00(1)))).*2./(rad2deg(thetaSamp(idM_Vec00(end)) - thetaSamp(idM_Vec00(1))))]];
                        
                        thetaPlatformDistribution_ = [[-1 + (0 - rad2deg(thetaSamp(idM_Vec00(1)))).*2./(rad2deg(thetaSamp(idM_Vec00(end)) - thetaSamp(idM_Vec00(1)))) ] ...
                            [-1 + (thetaExp_2 - rad2deg(thetaSamp(idM_Vec00(1)))).*2./(rad2deg(thetaSamp(idM_Vec00(end)) - thetaSamp(idM_Vec00(1))))]];
                    end
                else
                    thetaExp = dot(rad2deg(thetaSamp), thetaProb0);
                    radius = max(abs(thetaExp - rad2deg(thetaSamp(idM_Vec00(1)))), abs(thetaExp - rad2deg(thetaSamp(idM_Vec00(end)))));
                    thetaPlatformDistribution_ = [[-radius + (0 - rad2deg(thetaSamp(idM_Vec00(1)))).*(2.*radius)./(rad2deg(thetaSamp(idM_Vec00(end)) - thetaSamp(idM_Vec00(1)))) ]...
                        [-radius + (thetaExp - rad2deg(thetaSamp(idM_Vec00(1)))).*(2.*radius)./(rad2deg(thetaSamp(idM_Vec00(end)) - thetaSamp(idM_Vec00(1))))]];
                    
                    %                                 thetaPlatformDistribution_ = [[-1 + (0 - rad2deg(thetaSamp(idM_Vec00(1)))).*(2.*radius)./(rad2deg(thetaSamp(idM_Vec00(end)) - thetaSamp(idM_Vec00(1)))) ]...
                    %                                                               [-1 + (thetaExp - rad2deg(thetaSamp(idM_Vec00(1)))).*(2.*radius)./(rad2deg(thetaSamp(idM_Vec00(end)) - thetaSamp(idM_Vec00(1))))]];
                end
                
                if 0
                    figure(FigBase + 41);
                    if jkl == 1  %DEPTHITERNUM
                        if keyFrameLen == 2
                            %                          rngDir = fullfile(pwd, 'rng');
                            %                                     saveas(gcf,fullfile(probDir,sprintf('rng_%05d.png',length(dir(fullfile(probDir,'rng_*.png')))+1)));
                            %                                     saveas(gcf,fullfile(probDir,sprintf('rng_%05d.fig',length(dir(fullfile(probDir,'rng_*.fig')))+1)));
                            clf;
                            %                                     REFAngList = [];
                        end
                    end
                end
                if jkl == 1
                    if keyFrameLen == 2
                        REFAngList = [];
                        thetaPlatform = [];
                        thetaPlatformDistribution = [];
                    end
                end
                  
                if jkl == 1
                    REFAngList = [REFAngList; (k2cRef0)];
%                     REFAngList = [REFAngList; (k2cRef_00)];
                    if doRoundingRefAngList3
                        REFAngList3 = [REFAngList3; (k2cRef)];
                    else
                        REFAngList3 = [REFAngList3; (k2cRef0)];
                    end
                    thetaPlatform = [thetaPlatform; mean(idM_Vec00)./((length(thetaSamp)+1)/2)];
                    thetaPlatformDistribution = [thetaPlatformDistribution; thetaPlatformDistribution_];
                end
                
                if 0
                    figure(FigBase + 30),clf;subplot(1,2,1);plot(thetaPlatform,'-xb');title('platform midpoint ratio');
                    subplot(1,2,2);plot(thetaPlatformDistribution(:,1),'-xb');hold on;plot(thetaPlatformDistribution(:,2),'-xr');title('ref theta distribution');legend('zero')
                    if 0 % keyFrameLen <= 3
                        saveas(gcf,fullfile(probPath,sprintf(strcat('platform_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('platform_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__*.png'))))+1 + FigBase)));
                    end
                end
                
                ind = GetLastKeyFrameInd(obj);
                %% 20191230 useless code
                if 0
                    vslAng = poseWcsList(ind:end,3) - poseWcsList(ind,3);
                end
                if length(ANGOpt) >= length(REFAngList)
                    %                         optAng = ANGOpt(end-length(REFAngList)+2:end,:) - ANGOpt(end-length(REFAngList)+2,:);
                    baseId = length(ANGOpt) - length(REFAngList);
                    optAng0 = ANGOpt(baseId+1:end);
                    optAng = optAng0 - optAng0(1);
                    if 0
                        optAngRng = ANGRng(baseId+1:end,:);
                        optAngRng = [optAngRng(:,1) - optAngRng(1,1)  optAngRng(:,2) - optAngRng(1,2)];
                    else
                        %                             optAng0 = ANGOpt(baseId+1:end);
                        optAngRng = ANGRng(baseId+1:end,:);
                        optAngRng = [optAngRng(:,1) - optAng0(1,1)  optAngRng(:,2) - optAng0(1,1)];
                    end
                else
                    optAng = ANGOpt;
                    optAngRng = ANGRng;
                end
                if 0
                    figure(FigBase + 41);hold on;%plot(rad2deg(REFAngList),'-xr');
                    % %                     plot(rad2deg(angRngList(:,1)) - rad2deg(REFAngList),'-xg');
                    %                     plot(rad2deg([vslAng(2:end);thetaRng(idM)]) - rad2deg(REFAngList),'-xk');
                    if length(ANGOpt) >= length(REFAngList)
                        plot(rad2deg([optAngRng(2:end,1); thetaRng(ind1)]) - rad2deg(REFAngList),'-xg');
                        %                                 plot(rad2deg([optAng(2:end);thetaRng(idM)]) - rad2deg(REFAngList),'-xk');
                        plot(rad2deg([optAng(2:end);angOpt]) - rad2deg(REFAngList),'-xk');
                        plot(rad2deg([optAngRng(2:end,2); thetaRng(ind2)]) - rad2deg(REFAngList),'-xb');
                    else
                        if ~isempty(optAngRng)
                            plot(rad2deg([optAngRng(1:end,1); thetaRng(ind1)]) - rad2deg(REFAngList),'-xg');
                            %                                     plot(rad2deg([optAng(1:end);thetaRng(idM)]) - rad2deg(REFAngList),'-xk');
                            plot(rad2deg([optAng(1:end);angOpt]) - rad2deg(REFAngList),'-xk');
                            plot(rad2deg([optAngRng(1:end,2); thetaRng(ind2)]) - rad2deg(REFAngList),'-xb');
                        else
                            
                            plot(rad2deg([thetaRng(ind1)]) - rad2deg(REFAngList),'-xg');
                            %                                     plot(rad2deg([optAng(1:end);thetaRng(idM)]) - rad2deg(REFAngList),'-xk');
                            plot(rad2deg([optAng(1:end);angOpt]) - rad2deg(REFAngList),'-xk');
                            plot(rad2deg([thetaRng(ind2)]) - rad2deg(REFAngList),'-xb');
                        end
                    end
                    %                     plot(rad2deg(angRngList(:,2)) - rad2deg(REFAngList),'-xb');
                    legend('k2cUpper - k2cRef','k2cOpt - k2cRef','k2cLower - k2cRef');
                    title(sprintf('sigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f',ConfigParam.reproj_sigma,ConfigParam.reproj_beta,ConfigParam.reproj_sigma_right,ConfigParam.reproj_beta_right));
                    %                                        legend('k2cRef','k2cUpper','k2cOpt','k2cLower');
                    
                    % %                 for kl = 1 : length(thetaProb)
                    % %
                    % %
                    % %
                    % %
                    % %
                    % %                 end
                    % % % % %                             if UPDATEDEPTH
                    % % % % %                                 updatedProbZSum_ = (max(ProbZ(:))/max(updatedProbZSum(:))).*updatedProbZSum;
                    % % % % %                                 LocalTrace.probZ(inlierId,:) = updatedProbZSum_;
                    % % % % %                                 %                 obj.keyProbZ = [obj.keyProbZ; [{sum(KeyFrameFlagList)} {inlierId} {ProbZ} {updatedProbZSum_} {2.^size(LocalTrace.ptIcsX,2)}]];
                    % % % % %                                 obj.keyProbZ = [obj.keyProbZ; [{sum(KeyFrameFlagList)} {inlierId} {ProbZ} {updatedProbZSum_} {(max(ProbZ(:))/max(updatedProbZSum(:)))}]];
                    % % % % %                             end
                    % % % % %                             drawnow;
                    
                    drawnow;
                    
                    saveas(gcf,fullfile(probPath,sprintf(strcat('rng_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('rng_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__*.png'))))+1 + FigBase)));
                end
                updatedProbZSum_ = (max(ProbZ(:))/max(updatedProbZSum(:))).*updatedProbZSum;
                
                if 0 %% 20191115
                    updatedProbZSum_(isnan(updatedProbZSum_)) = 0;
                else
                    updatedProbZSum_(isnan(updatedProbZSum_)) = 1;
                end
                
                if keyFrameLen == 2
                    NewDispStep__ = [];
                else
                    NewDispStep__ = newDispStep(ismember(inlierIdPrv,inlierId));
                end
                curDispStepList = [];
                updatedProbZSum_ = updatedProbZSum_./repmat(max(updatedProbZSum_')',1,size(updatedProbZSum_,2));
                
                [depthGTInd_update, ProbZTmpTmp_update_norm, ProbZTmp_update_norm, idMax_update, depthGTIndAll_update, depthUpdateIndAll_update, ProbZNormSum, NewDispRng_, newDispStep_, ProbZTmpReSamp_] = GetDepthHist(obj,curDispStepList, updatedProbZSum_,disparityErrorRound,ConfigParam.disparity_sample_step, DispRng,NewDispStep__, depthC2KInd_ind,ConfigParam,keyFrameLen);
                %                             obj.curDispStepList = newDispStep;
                
                platformRatio = sum((ProbZTmp_update_norm > 0.99)')./sum((ProbZTmp_norm > 0.99)');
                if 0
                    DispRngUse = repmat(dispList(inlierId,:),1,length(disparityRng)) + repmat(disparityRng,length(inlierId),1);
                else
                    DispRngUse = DispRng;
                end
                
                DispRefine_ = sum(DispRngUse'.*ProbZNormSum')';
                DispRefine = DispRngUse(depthUpdateIndAll_update);
                
                [~, idMax_update_check] = ind2sub(size(DispRngUse), depthUpdateIndAll_update);
                zPeakHeight_update = updatedProbZSum_(depthUpdateIndAll_update);
                
                if 0
                    ProbZTmp_update_norm_sum = ProbZTmp_update_norm ./ (repmat(sum(ProbZTmp_update_norm')',1,size(ProbZTmp_update_norm,2)));
                    ProbZTmp_update_norm_sum_tmp = zeros(size(ProbZTmp_update_norm_sum));
                    
                else
                    if 0
                        DispRngInit = DispRng(:,(size(DispRng,2)+1)/2) + disparityRng;
                        ProbZNew = [];
                        for hu = 1 : size(DispRng,1)
                            ProbZNew(hu,:) = interp1(DispRng(hu,:),ProbZTmp_update_norm(hu,:),DispRngInit(hu,:));
                        end
                        ProbZNew(isnan(ProbZNew)) = 0;
                        ProbZTmp_update_norm_sum = ProbZNew ./ (repmat(sum(ProbZNew')',1,size(ProbZNew,2)));
                    else
                        ProbZTmp_update_norm_sum = ProbZTmp_update_norm ./ (repmat(sum(ProbZTmp_update_norm')',1,size(ProbZTmp_update_norm,2)));
                        tempDispRngMat = diff(DispRng')';
                        dispStepTemp = mean(tempDispRngMat(:));
                        dispStepBound = dispStepTemp*(length(disparityRng)-1)/2;
                        dispStepRngTemp = [-dispStepBound :dispStepTemp: dispStepBound];
                    end
                    ProbZTmp_update_norm_sum_tmp = zeros(size(ProbZTmp_update_norm_sum));
                end
                
                
                if 0
                    ProbZTmp_update_norm_sum_tmp(depthUpdateIndAll_update) = ProbZTmp_update_norm_sum(depthUpdateIndAll_update);
                    ProbZTmp_update_norm_sum_tmp(depthUpdateIndAll_update) = 1;
                else
                    ProbZTmp_update_norm_sum_tmp(depthGTIndAll_update) = ProbZTmp_update_norm_sum(depthGTIndAll_update);
                    ProbZTmp_update_norm_sum_tmp(depthGTIndAll_update) = 1;
                end
                if 1
                    if 0
                        figure(FigBase + 28),clf;plot(disparityRng, sum(ProbZTmp_update_norm_sum));hold on;plot(disparityRng,sum(ProbZTmp_update_norm_sum_tmp));legend('refined disp','gt disp');
                        saveas(gcf,fullfile(probPath,sprintf(strcat('depthDistri_',probDir(end-14:end),'____',sprintf('%04d',sum(KeyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('depthDistri_',probDir(end-14:end),'____',sprintf('%04d',(sum(KeyFrameFlagList))),'__*.png'))))+1 + FigBase)));
                    else
                        figure(FigBase + 28),clf;plot(dispStepRngTemp, sum(ProbZTmp_update_norm_sum));hold on;plot(dispStepRngTemp,sum(ProbZTmp_update_norm_sum_tmp));legend('refined disp','gt disp');
                        saveas(gcf,fullfile(probPath,sprintf(strcat('depthDistri_',probDir(end-14:end),'____',sprintf('%04d',sum(KeyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('depthDistri_',probDir(end-14:end),'____',sprintf('%04d',(sum(KeyFrameFlagList))),'__*.png'))))+1 + FigBase)));
                        
                    end
                    
                    
                end
                
                [~,goodGtId] = sort(ProbZTmpTmp_update_norm,'descend');
                [~,goodOptId] = sort(zPeakHeight_update,'descend');
                nN = length(goodOptId);
                pt2dCurUse = pt2dCur(goodOptId(1:nN),:);
                PixUse = Pix(goodOptId(1:nN),:);
                if 0
                    DispRefineUse = DispRefine(goodOptId(1:nN));
                else
                    DispRefineUse_ = DispRefine_(goodOptId(1:nN));
                end
                if 0
                    DispGT = dispGTTmp(inlierId); %DispRefine(goodOptId(1:nN));
                    DispInit = dispList(inlierId);
                end
                if 0
                    figure(FigBase + 25);clf;plot(ProbZTmpTmp_update_norm(goodOptId(1:nN)));
                    saveas(gcf,fullfile(probPath,sprintf(strcat('peakHeight_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('peakHeight_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__*.png'))))+1 + FigBase)));
                    figure(FigBase + 24);clf;hist([DispRefine DispGT DispInit], 50);legend('refined disp','gt disp','stereo disp');
                    saveas(gcf,fullfile(probPath,sprintf(strcat('dispDistri_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('dispDistri_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__*.png'))))+1 + FigBase)));
                end
                
                try
                    [minVal,idididMin] = sort(ProbZTmpTmp_update_norm,'ascend');
                    [maxVal,idididMax] = sort(ProbZTmpTmp_update_norm,'descend');
                    
                    checkId1111 = ([idididMin(1:15); idididMax(1:15)]);
                    
                    
                    qwa = checkId1111(1);figure(FigBase + 17),clf;subplot(2,1,1);plot(disparityRng, [ProbZ(qwa,:)./max(ProbZ(qwa,:)); updatedProbZSum0(qwa,:)./max(updatedProbZSum0(qwa,:)); updatedProbZSum(qwa,:)./max(updatedProbZSum(qwa,:))]');hold on;plot(disparityRng(depthGTInd_update(qwa,1)),updatedProbZSum(qwa,depthGTInd_update(qwa,1))./max(updatedProbZSum(qwa,:)),'*r');legend('orig Z','1st upadte','2nd update','gt');
                    qwa = checkId1111(end);             subplot(2,1,2);plot(disparityRng, [ProbZ(qwa,:)./max(ProbZ(qwa,:)); updatedProbZSum0(qwa,:)./max(updatedProbZSum0(qwa,:)); updatedProbZSum(qwa,:)./max(updatedProbZSum(qwa,:))]');hold on;plot(disparityRng(depthGTInd_update(qwa,1)),updatedProbZSum(qwa,depthGTInd_update(qwa,1))./max(updatedProbZSum(qwa,:)),'*r');legend('orig Z','1st upadte','2nd update','gt');
                    saveas(gcf,fullfile(probPath,sprintf(strcat('zUpdate_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('zUpdate_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__*.png'))))+1 + FigBase)));
                    
                    
                    
                    %                                 checkId = inlierId([idididMin(1:5); idididMax(1:5)]);
                    checkId = ([idididMin(1:15); idididMax(1:15)]);
                    ratioMat = BB.*repmat(thetaProb'./max(thetaProb), 1,size(BB0,2));
                    ratioVec = sum(ratioMat);
                    if 0
                        figure(FigBase + 29),clf;subplot(1,2,1);plot(repmat(rad2deg(thetaSamp)',1,length(checkId)/2), BB0(:,checkId(1:length(checkId)/2)),'-g');hold on;plot(rad2deg(thetaSamp), thetaProb./max(1),'-b','LineWIdth',5);title('bad');
                        subplot(1,2,2);plot(repmat(rad2deg(thetaSamp)',1,length(checkId)/2), BB0(:,checkId(length(checkId)/2+1:end)),'-r');hold on;plot(rad2deg(thetaSamp), thetaProb./max(1),'-b','LineWIdth',5);title('good');
                    end
                    
                    %                                 checkId = setdiff(find(ProbZTmpTmp_norm > ConfigParam.depth_hist_ratio), find(ProbZTmpTmp_update_norm > ConfigParam.depth_hist_ratio));
                    checkId1 = find(0.9.*ProbZTmpTmp_norm > ProbZTmpTmp_update_norm);
                    
                    
                    try
                        %                                     checkId = checkId(1:10);
                        checkId1 = checkId1(1:2*floor(length(checkId1)/2));
                        figure(FigBase + 38);subplot(1,3,1);hold on;plot(checkId1(1:length(checkId1)/2), dispList(inlierId(checkId1(1:length(checkId1)/2))) - dispGTTmp(inlierId(checkId1(1:length(checkId1)/2))),'og');plot(checkId1(length(checkId1)/2+1:end), dispList(inlierId(checkId1(length(checkId1)/2+1:end))) - dispGTTmp(inlierId(checkId1(length(checkId1)/2+1:end))),'or');legend('','bad','good')
                        subplot(1,3,2);plot(pt2dCur(checkId1(1:length(checkId1)/2),1),pt2dCur(checkId1(1:length(checkId1)/2),2),'og');plot(pt2dCur(checkId1(length(checkId1)/2+1:end),1),pt2dCur(checkId1(length(checkId1)/2+1:end),2),'or');
                        subplot(1,3,3);plot(dispErrCur);title('stereoDispCur - gtDispCur');
                    catch
                        sdafb = 1;
                    end
                    figure(FigBase + 38);
                    saveas(gcf,fullfile(probPath,sprintf(strcat('disp_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('disp_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__*.png'))))+1 + FigBase)));
                    
                    ArrangeCheckId;
                    
                    tracebackId = inlierId(checkId);
                    if ~isempty(obj.feat2check)
                        tracebackId - obj.feat2check;
                    end
                    %                                 checkId = ([idididMin(1:15); idididMax(1:15)]);
                    doProb = 0; idM_Vec = 0; debugProb;
                catch
                    svkjhj = 1;
                end
                ProbZ000 = updatedProbZSum_;
                
                if sum(isnan(ProbZ000(:))) > 0
                    sdkj = 1;
                end
                
                
            end
            [~, trackingError] = NormalizeVector(pt2dCur - pixGT);
            
            oldZArea = sum(ProbZ')';
            newZArea = sum(updatedProbZSum')';
            
            [maxFeatZ1_update,idMax1_update] = max(updatedProbZSum');
            
            scaleRatio = 1;  (max(ProbZ(:))/max(updatedProbZSum(:)));
            depthGTIndAll;         maxFeatZ1;         idMax;
            depthGTIndAll_update;  maxFeatZ1_update;  idMax_update;
            
            if 0
                figure,plot([ProbZ(1,:);scaleRatio.*updatedProbZSum(1,:)]')
            end
            
            oldDepthMaxIndAll = sub2ind(size(ProbZ),[1:size(ProbZ,1)]', idMax');
            if 0
                figure,plot(ProbZ(oldDepthMaxIndAll) - maxFeatZ1');
            end
            
            
            
            if 0
                areaInfo = scaleRatio.*[newZArea./oldZArea maxFeatZ1_update'./maxFeatZ1'];
            else
                areaInfo = scaleRatio.*[newZArea./oldZArea updatedProbZSum(oldDepthMaxIndAll)./ProbZ(oldDepthMaxIndAll)];
            end
            
            %                         figure(FigBase + 77),clf,subplot(1,2,1);plot(newZArea./oldZArea);title('oldFeatArea / newFeatArea');subplot(1,2,2);plot(maxFeatZ1_update./maxFeatZ1);title('oldMaxFeat / newMaxFeat');
            %                         figure(FigBase + 77),clf,subplot(1,2,1);plot([newZArea./oldZArea maxFeatZ1_update'./maxFeatZ1']);legend('area','peak','Location','southwest');title('oldFeatArea / newFeatArea');subplot(1,2,2);plot(maxFeatZ1_update./maxFeatZ1);title('oldMaxFeat / newMaxFeat');
            %                         figure(FigBase + 77),clf,subplot(1,2,1);plot([areaInfo]);legend('area','peak','Location','southwest');title('oldFeatArea / newFeatArea');subplot(1,2,2);plot(areaInfo(:,2));title('oldMaxFeat / newMaxFeat');
            
            [~,id123] = sort(areaInfo(:,2),'ascend');
            if 0
                figure(FigBase + 77),clf,subplot(3,2,1);hist([areaInfo],50);legend('area','peak','Location','southwest');title('newFeatArea / oldFeatArea');subplot(3,2,2);hist(areaInfo(:,2),50);title('newMaxFeat / oldMaxFeat');
                subplot(3,2,3);plot(trackingError(id123,end));title('norm(trackingXY - gtXY)'); subplot(3,2,4);plot(abs(pt2dCur(id123,1) -pixGT(id123,1)));title('norm(trackingX - gtX)');
                subplot(3,2,5);plot(abs(dispList(inlierId(id123)) - dispGTTmp(inlierId(id123))));title('norm(stereoDisp - gtDisp)');subplot(3,2,6);plot(abs(pt2dCur(id123,2) -pixGT(id123,2)));title('norm(trackingY - gtY)');
                
                saveas(gcf,fullfile(probPath,sprintf(strcat('areaDrop_',probDir(end-14:end),'____',sprintf('%04d',sum(KeyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('areaDrop_',probDir(end-14:end),'____',sprintf('%04d',(sum(KeyFrameFlagList))),'__*.png'))))+1 + FigBase)));
            end
            
            if UPDATEDEPTH
                %                                 updatedProbZSum_ = (max(ProbZ(:))/max(updatedProbZSum(:))).*updatedProbZSum;
                
                if keyFrameLen <= 2
                    outThr = 0.5;
                else
                    outThr = 0.45;
                end
                outThr = -1;
                if 1  %keyFrameLen <= 3
                    
                    updatedProbZSum_ = updatedProbZSum_./max(updatedProbZSum_(:));
                    updatedProbZSum_(platformRatio < outThr,:) = min(min(updatedProbZSum_(:)),0.000001);
                    inFlag = inlierId(find(platformRatio >= 0.7)');
                else
                    inFlag = inlierId;  [1:length(platformRatio)]';
                end
                if 0
                    LocalTrace.probZ(inlierId,:) = updatedProbZSum_;
                    %                 KeyProbZ = [KeyProbZ; [{sum(KeyFrameFlagList)} {inlierId} {ProbZ} {updatedProbZSum_} {2.^size(LocalTrace.ptIcsX,2)}]];
                    KeyProbZ = [KeyProbZ; [{sum(KeyFrameFlagList)} {inlierId} {ProbZ./max(ProbZ(:))} {updatedProbZSum_./max(updatedProbZSum_(:))} {(max(ProbZ(:))/max(updatedProbZSum(:)))} {pt2dCur} {pixGT} {trackingError} {Pix} {zList} {dispList} {dispGTTmp} {dispCurList} {pt2dCurR} {dispMapCurGTList} {pixGTR} {depthListGT} {XYZ} {k2cRef0} {k2cPnp} {[ProbZTmpTmp_norm ProbZTmpTmp_update_norm]} {thetaPlatformDistribution_} {areaInfo} {ProbZTmp_update_norm} {DispRefineUse_} {NewDispRng} {newDispStep} {ProbZTmpReSamp}]];
                elseif 0
                    LocalTrace.probZ(inlierId,:) = ProbZTmpReSamp_./max(ProbZTmpReSamp_(:));
                    %                 KeyProbZ = [KeyProbZ; [{sum(KeyFrameFlagList)} {inlierId} {ProbZ} {updatedProbZSum_} {2.^size(LocalTrace.ptIcsX,2)}]];
                    KeyProbZ = [KeyProbZ; [{sum(KeyFrameFlagList)} {inlierId} {ProbZ./max(ProbZ(:))} {ProbZTmpReSamp_./max(ProbZTmpReSamp_(:))} {(max(ProbZ(:))/max(updatedProbZSum(:)))} {pt2dCur} {pixGT} {trackingError} {Pix} {zList} {dispList} {dispGTTmp} {dispCurList} {pt2dCurR} {dispMapCurGTList} {pixGTR} {depthListGT} {XYZ} {k2cRef0} {k2cPnp} {[ProbZTmpTmp_norm ProbZTmpTmp_update_norm]} {thetaPlatformDistribution_} {areaInfo} {ProbZTmp_update_norm} {DispRefineUse_} {NewDispRng_} {newDispStep_} {ProbZTmpReSamp_}]];
                else
                    idGoodInlier = find(ProbZTmpReSamp_(depthC2KInd_ind) > -0.995);
                    inlierIdNew = inlierId(idGoodInlier,:);
                    
                    
                    % % % % %                                    if keyFrameLen > 3
                    % % % % %
                    % % % % %                                        if sign(obj.meanErrAndStd(end-1,7)) == sign(obj.meanErrAndStd(end,7))
                    %                                    ProbZ2Nect =
                    
                    if 0
                        ProbZTmpReSamp_ = ProbZTmpReSamp_ + ProbZ;
                        ProbZTmpReSamp_ = ProbZTmpReSamp_./repmat(max(ProbZTmpReSamp_')',1,size(ProbZTmpReSamp_,2));
                    end
                    if 0
                        ProbZTmpReSamp_ = ones(size(ProbZTmpReSamp_));
                    end
                    if 0
                        LocalTrace.probZ(inlierIdNew,:) = ProbZTmpReSamp_(idGoodInlier,:); % ProbZTmpReSamp_./max(ProbZTmpReSamp_(:));
                    else
                        LocalTrace.probZ = ProbZTmpReSamp_(idGoodInlier,:);
                    end
                    %                 KeyProbZ = [KeyProbZ; [{sum(KeyFrameFlagList)} {inlierId} {ProbZ} {updatedProbZSum_} {2.^size(LocalTrace.ptIcsX,2)}]];
                    KeyProbZ = [KeyProbZ; [{sum(KeyFrameFlagList)} {inlierIdNew} {ProbZ(idGoodInlier,:)} {ProbZTmpReSamp_(idGoodInlier,:)} {(max(ProbZ(:))/max(updatedProbZSum(:)))} {pt2dCur(idGoodInlier,:)} {pixGT(idGoodInlier,:)} {trackingError(idGoodInlier,:)} {Pix(idGoodInlier,:)} {zList} {dispList} {dispGTTmp} {dispCurList(idGoodInlier,:)} {pt2dCurR(idGoodInlier,:)} {dispMapCurGTList(idGoodInlier,:)} {pixGTR(idGoodInlier,:)} {depthListGT} {XYZ(:,idGoodInlier)} {k2cRef0} {k2cPnp} {[ProbZTmpTmp_norm(idGoodInlier,:) ProbZTmpTmp_update_norm(idGoodInlier,:)]} {thetaPlatformDistribution_} {areaInfo(idGoodInlier,:)} {ProbZTmp_update_norm(idGoodInlier,:)} {DispRefineUse_(idGoodInlier,:)} {NewDispRng_(idGoodInlier,:)} {newDispStep_(idGoodInlier,:)} {ProbZTmpReSamp_(idGoodInlier,:)}]];
                end
                if 0
                    LocalTrace.ptIcsX(inlierId,end) = ExpCoord_OnlyP2(:,1);
                    LocalTrace.ptIcsY(inlierId,end) = ExpCoord_OnlyP2(:,2);
                elseif 0
                    LocalTrace.ptIcsX(inlierId,end) = pixGT(:,1);
                    LocalTrace.ptIcsY(inlierId,end) = pixGT(:,2);
                else
                    
                    asvkb = 1;
                    
                end
                
                %                             checkId = ([idididMin(1:15); idididMax(1:15)]);
                if 0
                    try
                        doShift = 1; use1 = 1; debugPeakShift;
                        
                    catch
                        asvdbkj = 1;
                    end
                end
            end
            
            
            idTheta = find(thetaProb > 0.00001);
            projErrList = cell2mat(ReprojErrVecTmp(idTheta));
            if 0
                [ac,dc] = probDensity(0, 8 ,ConfigParam.disparity_beta,ConfigParam.reproj_beta, [-10:0.01:10],ConfigParam.reproj_sample_interval, 'reproj');
                figure,plot(dc,ac)
                figure,plot(projErrList)
            end
            
            if 0
                try
                    %                                 wg;
                    ArrangeCheckId
                    tracebackId = inlierId(checkId);
                    
                catch
                    checkId = ([idididMin(1:15); idididMax(1:15)]);
                    tracebackId = inlierId(checkId);
                    
                end
                %                             doProb = 1; idM_Vec = -20:20; debugProb;
                try
                    drawPorb1 = 0;doProb = 1; idM_Vec = -20:20; debugProb;
                catch
                    asdgk = 1;
                end
            end
            
        end
    end
end

end

function [pixGT, pixGTR] = GetGtTrace(CamModel,CoordSysAligner, LocalTrace, k2cRef0,inlierId,dispList,disparityErrorRound,intrMat,princpPtL,princpPtR,dispMapCurGTList)
b2cPmat = GetPctBody2Cam(CoordSysAligner, 1);
if 0
    k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(thetaRng(idM)),zeros(3,1));
else
    k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cRef0),zeros(3,1));
end
k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;

Pix = [LocalTrace.ptIcsX(:,1) LocalTrace.ptIcsY(:,1)];

metricPrevPtCcsGT = intrMat\HomoCoord(Pix',1);
metricPrevPtCcsGT = normc(metricPrevPtCcsGT);
if 0
    scaleAllGT = depthListGT(inlierId)./metricPrevPtCcsGT(3,:)';
else
    dispGTComp = dispList(:) - disparityErrorRound;
    depthListGTComp = intrMat(1,1).*norm(CamModel.transVec1To2)./(dispGTComp + (princpPtR(1) - princpPtL(1)));
    scaleAllGT = depthListGTComp./metricPrevPtCcsGT(3,:)';
end
XYZ = [repmat(scaleAllGT',3,1).*metricPrevPtCcsGT];


k2cT = -b2cPmat.transformMat(1:3,1:3)*k2cBodyPmat.transformMat(1:3,1:3)*b2cPmat.transformMat(1:3,1:3)'*b2cPmat.transformMat(1:3,4) + b2cPmat.transformMat(1:3,4);
k2cCam = [k2cCamPmat.transformMat(1:3,1:3) k2cT;0 0 0 1];
homocurrCcsXYZ = k2cCam*HomoCoord(XYZ,1); %pix0_GT_ = pflat(intrMat*homocurrCcsXYZALL_(1:3,:))';
pixGT = pflat(intrMat*homocurrCcsXYZ(1:3,:))';
pixGT = pixGT(:,1:2);
pixGTR = [pixGT(:,1) - dispMapCurGTList pixGT(:,2)];

pixGT__ = pixGT;
pixGTR__ = pixGTR;
end

function [depthGTInd, ProbZTmpTmpNormUse, ProbZTmpNorm, peakIdTmp, depthGTIndAll, depthUpdateIndAllUse, ProbZNormSum, NewDispRng, newDispStep, ProbZTmpReSamp] = GetDepthHist(obj, curDispStepList,ProbZ,disparityErrorRound,disparity_sample_step,DispRng,NewDispStep_,depthC2KInd_ind,ConfigParam,keyFrameLen)

if ~exist('ConfigParam','var')
    ConfigParam = obj.configParam;
end

[maxFeatZ, idMax] = max(ProbZ');

ProbZSumMat = repmat(sum(ProbZ')', 1, size(ProbZ, 2));
ProbZNormSum = ProbZ./ProbZSumMat;



%             curDispStepList = curDispStepList;


maxFeatZ = maxFeatZ';
maxFeatZ = repmat(maxFeatZ, 1, size(ProbZ,2));
idMaxFeat = ProbZ == maxFeatZ;

[xGrid,~ ] = meshgrid(1:size(ProbZ,2), 1:size(ProbZ,1));
xGridMean = xGrid.*idMaxFeat;

if 0
    idMaxFeatSum = round(mean(xGridMean'));
else
    idMaxFeatSum = round(sum(xGridMean')./sum(idMaxFeat'));
end

idMax = idMaxFeatSum;

ProbZTmpNorm = ProbZ./maxFeatZ;

platformRegion = ProbZTmpNorm > 0.9;
platformRegion_ind = find(platformRegion' > 0);
[platformRegionX, platformRegionY] = ind2sub(size(ProbZ'), platformRegion_ind);


highProbRng = [];
for df = 1 : size(ProbZ,1)
    idTemp = find(platformRegion(df,:) > 0);
    highProbRng(df,:) = [min(idTemp) max(idTemp)];
end
[~, platformRegionX] = ind2sub(size(ProbZ), depthC2KInd_ind);
intersectRatio = (platformRegionX - highProbRng(:,1))./diff(highProbRng')';


highProbRegion = ProbZTmpNorm >= ConfigParam.disparity_resample_prob_threshold | isnan(ProbZTmpNorm);
highProbRegionId = find(highProbRegion(:) > 0);
[yCoord, xCoord] = ind2sub(size(highProbRegion), highProbRegionId);
% %                         ProbZ = ProbZTmp_norm;
[~,idY] = sort(yCoord,'ascend');
xCoord_ = xCoord(idY);
yCoord_ = yCoord(idY);
highProbRegionFilled = nan(size(highProbRegion));
highProbRegionFilled(highProbRegionId) = xCoord;

if 1
    newDepthCenter = repmat((size(ProbZ,2)+1)/2, 1, size(ProbZ,1));
    
    highProbRng0 = [min(highProbRegionFilled');max(highProbRegionFilled')]';
    dlt1 = abs(highProbRng0(:,1) - newDepthCenter');
    dlt2 = abs(highProbRng0(:,2) - newDepthCenter');
    radius = max([dlt1 dlt2]')';
    highProbRng = [(newDepthCenter' - radius) (newDepthCenter' + radius)];
else
    highProbRng = [min(highProbRegionFilled');max(highProbRegionFilled')]';
    
end
highProbRngMinCoord = [highProbRng(:,1) [1:size(DispRng,1)]'];
highProbRngMinCoordInd = sub2ind(size(DispRng), highProbRngMinCoord(:,2), highProbRngMinCoord(:,1));
highProbRngMaxCoord = [highProbRng(:,2) [1:size(DispRng,1)]'];
highProbRngMaxCoordInd = sub2ind(size(DispRng), highProbRngMaxCoord(:,2), highProbRngMaxCoord(:,1));

if 0
    newDepthCenter = ceil(mean(highProbRng'));
end

%             newDepthCenter = repmat((size(ProbZ,2)+1)/2, 1, size(ProbZ,1));





newDepthCenterCoord = [newDepthCenter' [1:size(DispRng,1)]'];

newDisp = DispRng(sub2ind(size(DispRng), newDepthCenterCoord(:,2),newDepthCenterCoord(:,1))) ;
%             newDispStep = (DispRng(:,end) - DispRng(:,1))./(size(DispRng,2)-1);
newDispStep = (DispRng(highProbRngMaxCoordInd) - DispRng(highProbRngMinCoordInd))./(size(DispRng,2)-1);
%             newDispStep(newDispStep <= 0) = 0.001;
newDispStep(newDispStep <= 0.05) = 0.05;
SampMat = repmat(-(size(DispRng,2)-1)/2 : (size(DispRng,2)-1)/2,size(DispRng,1),1);
SampMatVal = SampMat.*repmat(newDispStep,1,size(DispRng,2));
NewDispRng = SampMatVal + repmat(newDisp,1,size(DispRng,2));

if 0
    DispRngT = DispRng';
    ProbZTmpNormT = ProbZTmpNorm';
    NewDispRngT = NewDispRng';
    ProbZTmpNormReSampTVec = interp1(DispRngT(:)', ProbZTmpNormT(:)', NewDispRngT(:)');
end
if ConfigParam.disparity_resample_prob_threshold > 0
    ProbZTmpReSamp = [];
    for tp = 1 : size(DispRng,1)
        %                 ProbZTmpNormReSamp(tp,:) = interp1(DispRng(tp,:), ProbZTmpNorm(tp,:), NewDispRng(tp,:));
        ProbZTmpReSamp(tp,:) = interp1(DispRng(tp,:), ProbZ(tp,:), NewDispRng(tp,:));
    end
else
    
    ProbZTmpReSamp = ProbZ;
    NewDispRng = DispRng;
    newDispStep = ConfigParam.disparity_sample_step.*ones(size(DispRng,1),1);
end
nanInd = find(isnan(ProbZTmpReSamp(:)));
[nanY, nanX] = ind2sub(size(DispRng),nanInd);
idXFirst = find(nanX == 1);
idXLast = find(nanX == size(DispRng,2));

if ~isempty(idXFirst)
    nonNanIndFirst = sub2ind(size(DispRng), nanY(idXFirst), nanX(idXFirst) + 1);
    ProbZTmpReSamp(nanInd(idXFirst)) = ProbZTmpReSamp(nonNanIndFirst);
end
if ~isempty(idXLast)
    nonNanIndLast = sub2ind(size(DispRng), nanY(idXLast), nanX(idXLast) - 1);
    ProbZTmpReSamp(nanInd(idXLast)) = ProbZTmpReSamp(nonNanIndLast);
end

ProbZTmpReSamp = ProbZTmpReSamp./repmat(max(ProbZTmpReSamp')',1,size(ProbZTmpReSamp,2));


%             ProbZTmpReSamp(nanInd) = ProbZTmpReSamp(nonNanInd);


if 0
    figure,plot(DispRng(highProbRngMinCoordInd) - NewDispRng(:,1))
    figure,plot(DispRng(highProbRngMaxCoordInd) - NewDispRng(:,end))
    figure,plot((DispRng(highProbRngMaxCoordInd) - DispRng(highProbRngMinCoordInd)) - (NewDispRng(:,end) - NewDispRng(:,1)))
    figure,plot((DispRng(highProbRngMaxCoordInd) - DispRng(highProbRngMinCoordInd)))
end


if keyFrameLen == 2
    depthGTOfst = round(-(disparityErrorRound)./disparity_sample_step);
else
    depthGTOfst = round(-(disparityErrorRound)./NewDispStep_);
end
depthGTInd = depthGTOfst + (size(ProbZ,2)+1)/2;
depthGTInd(depthGTInd < 1) = 1;
depthGTInd(depthGTInd > size(ProbZ,2)) = size(ProbZ,2);

%                             DepthGTInd(jkl,:) = depthGTInd';

depthGTIndAll = sub2ind(size(ProbZ),[1:size(ProbZ,1)]', depthGTInd);
depthUpdateIndAll = sub2ind(size(ProbZ),[1:size(ProbZ,1)]', idMax');

ProbZTmpTmpNorm = ProbZTmpNorm(depthGTIndAll);




if 1
    ProbZTmpTmpNormUse = ProbZTmpTmpNorm;
    depthUpdateIndAllUse = depthUpdateIndAll;
    peakIdTmp = idMax;
    
else
    
    %             ProbZTmp = ProbZ;
    %             ProbZTmp(depthUpdateIndAll) = 0;
    %
    %             [maxFeatZTmp, idMaxTmp] = max(ProbZTmp');
    %             maxFeatZTmp = maxFeatZTmp';
    %             maxFeatZTmp = repmat(maxFeatZTmp, 1, size(ProbZTmp,2));
    ProbZTmpNormTmp = ProbZTmpNorm; %ProbZTmp./maxFeatZTmp;
    ProbZTmpNormTmp(depthUpdateIndAll) = 0;
    [~, idMaxTmp] = max(ProbZTmpNormTmp');
    % %                         ProbZ = ProbZTmp_norm;
    
    depthGTOfstTmp = round(-(disparityErrorRound)./disparity_sample_step);
    depthGTIndTmp = depthGTOfstTmp + (size(ProbZTmpNorm,2)+1)/2;
    depthGTIndTmp(depthGTIndTmp < 1) = 1;
    depthGTIndTmp(depthGTIndTmp > size(ProbZTmpNorm,2)) = size(ProbZTmpNorm,2);
    
    %                             DepthGTInd(jkl,:) = depthGTInd';
    
    depthGTIndAllTmp = sub2ind(size(ProbZTmpNorm),[1:size(ProbZTmpNorm,1)]', depthGTIndTmp);
    depthUpdateIndAllTmp = sub2ind(size(ProbZTmpNorm),[1:size(ProbZTmpNorm,1)]', idMaxTmp');
    
    ProbZTmpTmpNormTmp = ProbZTmpNormTmp(depthGTIndAllTmp);
    
    
    idTwoPeak = (ProbZTmpNorm(depthUpdateIndAll) - ProbZTmpNormTmp(depthUpdateIndAllTmp) <= 0.005) & (abs(idMax - idMaxTmp)' > 1);
    %             depthUpdateIndAllUse = depthUpdateIndAll;
    %             peakIdTmp(~idTwoPeak,:) = idMax(~idTwoPeak');
    
    if 0
        peakIdTmp = idMax;
    else
        peakIdTmp = idMax';
        
        peakIdTmp(idTwoPeak,:) = round(mean([idMax(idTwoPeak); idMaxTmp(idTwoPeak)]))';
        
        depthUpdateIndAllUse = sub2ind(size(ProbZ),[1:size(ProbZ,1)]', peakIdTmp);
    end
    ProbZTmpTmpNormUse = ProbZTmpTmpNorm;
    ProbZTmpTmpNormUse(idTwoPeak,:) = mean([ProbZTmpNorm(depthUpdateIndAll(idTwoPeak,:)) ProbZTmpNorm(depthUpdateIndAllTmp(idTwoPeak,:))]')';
    % %             ProbZUse = ProbZ;
    % %             ProbZUse
end
end

function  [thetaProbUse, angOpt, thetaExp_tmp,thetaRngP2C_use] = New_OnlyP2_new_old(obj, pt2dPrv, pt2dCur, intrMat,thetaP2C,T_B2C, k2cRef, tx, ty, tz, thetaRng,DispRng, inlierId)



s = Setting([]);
s.configParam.disparityBoundary = ConfigParam.disparity_error;
s.configParam.disparityIntervel = ConfigParam.disparity_sample_step;
s.configParam.disparityRange = -ConfigParam.disparity_error : ConfigParam.disparity_sample_step : ConfigParam.disparity_error;
s.configParam.disparityBin = length(s.configParam.disparityRange);

s.configParam.disparityOrginRange = DispRng;
s.configParam.angleIntervel = rad2deg(ConfigParam.theta_sample_step);
s.configParam.angleBoundary = rad2deg(ConfigParam.theta_range(2));
s.configParam.angleRange  = -s.configParam.angleBoundary : s.configParam.angleIntervel : s.configParam.angleBoundary;
s.configParam.angleBin = length(s.configParam.angleRange);

s.configParam.angleOrginRange = rad2deg(thetaP2C) + s.configParam.angleRange;
s.configParam.tableSize = s.configParam.angleBin*s.configParam.disparityBin;
s.configParam.disparitySigma = obj.setting.configParam.disparitySigma;
s.configParam.disparityBeta = obj.setting.configParam.disparityBeta;
s.configParam.gaussSigma = obj.setting.configParam.gaussSigma;
s.configParam.gaussBeta = obj.setting.configParam.gaussBeta;
s.configParam.wx = obj.setting.configParam.wx;
s.configParam.wy = obj.setting.configParam.wy;


pt2dPrvHomo = (inv(intrMat)*pextend(pt2dPrv'))';
pt2dCurHomo = (inv(intrMat)*pextend(pt2dCur'))';

[angleResult ] = VisualLocalizer.AnglePredict_old(pt2dPrvHomo',pt2dCurHomo',tx, ty, tz);
angleOrgin = rad2deg(thetaP2C);
angleResultDiff     = angleResult - angleOrgin;
angleResultRangeMin = angleResult - angleOrgin+ (-0.1);
angleResultRangeMax = angleResult - angleOrgin+   0.1;

co = cosd(angleResult) ;
so = sind(angleResult) ;
%                                     depthResult = (r_cam .*(existPt2.ptHomo(1,:).*(-tz.*co+tx.*so+tz) - (-tz.*so-tx.*co+tx))) ...
%                                         ./ (existPt1.ptHomo(1,:).*co+so-existPt2.ptHomo(1,:).*(-existPt1.ptHomo(1,:).*so+co));
mm = Modals.Modal(s.configParam,length(inlierId));
%                                   mm.angleModal = obj.angleModalOrg;
if 1 %~NewOnlyP2
    mm.angleModal = Modals.UpdateModalAngle1(mm, s.configParam,[],[],angleResultRangeMin,angleResultRangeMax);
else
    mm.angleModal = Modals.UpdateModalAngleOnlyP2(mm, s.configParam,[],[],[],[],angleResultDiff);
end
thetaProb = mm.angleModal./sum(mm.angleModal);
thetaProbUse = thetaProb;
angOpt = deg2rad(dot(s.configParam.angleOrginRange, thetaProb));
thetaExp_tmp = angOpt - k2cRef;
thetaRngP2C_use = deg2rad(s.configParam.angleOrginRange);

end















function [thetaProbUse, angOpt, thetaExp_tmp,thetaRngP2C_use] = New_OnlyP2_new(obj, pt2dPrv, pt2dCur, intrMat,thetaP2C,T_B2C, k2cRef, tx, ty, tz, thetaRng,ConfigParam)
global Rati_onlyP2
areaThreshold = -1; %  1-Rati_onlyP2;
radius = 3;
existPt1 = Points.Point(pt2dPrv(:,1),pt2dPrv(:,2),intrMat);
existPt2 = Points.Point(pt2dCur(:,1),pt2dCur(:,2),intrMat);
thetaRngP2C = [deg2rad(-179) : ConfigParam.theta_sample_step : deg2rad(179)];  % + (thetaP2C);
% thetaSampP2C_ = thetaRngP2C_ - thetaP2C;



angleIntervel = rad2deg(ConfigParam.theta_sample_step);


[angleResult ] = VisualLocalizer.AnglePredict(existPt1.ptHomo,existPt2.ptHomo,tx, ty, tz);
ambiguifyRange = 1.0./angleIntervel;
thetaProb = VisualLocalizer.UpdateModalAngleOnlyP2NewVersion2(angleIntervel,angleResult,existPt1,existPt2,ambiguifyRange);
%         m.angleModal = angleModal;




rati = Rati_onlyP2; 0.95; 0.8;0.5;0.8; 0.5; 0.8; 0.9; 0;0.8;

thetaProb1 = thetaProb./max(thetaProb);
thetaProb22 = thetaProb1(thetaProb1 > rati);
thetaProb22 = thetaProb22./sum(thetaProb22);

thetaRngP2C22 = thetaRngP2C(thetaProb1 > rati);

if 0
    diffTheta = (diff(thetaRngP2C22));
    
    idx = find(round(rad2deg(diffTheta),4) == round(angleIntervel,4));
    [idxCell,idx] = splitIndex2(idx');
    for i = 1 : length(idxCell)
        len(i,1) = length(idxCell{i});
    end
    [~,idMax] = max(len);
    id_ = idxCell{idMax};
    thetaRngP2C2 = thetaRngP2C22(id_(1):id_(end)+1);
    thetaProb2 = thetaProb22(id_(1):id_(end)+1);
    
    thetaProb2 = thetaProb2./sum(thetaProb2);
    
    angOpt = deg2rad(dot(rad2deg(thetaRngP2C2), thetaProb2));   % 0.5 0.8 0.0
    %  thetaExp = deg2rad(dot(rad2deg(thetaSamp(thetaProb1 > rati)), thetaProb2));
    thetaExp_tmp = angOpt - k2cRef;
else
    
    
    [max_v, max_idx] = max(thetaProb);
    
    maxIds = find(thetaProb == max_v);
    max_idx = round(mean(maxIds));
    area =thetaProb(max_idx);
    
    for idx_i = 1:length(thetaProb)/2
        area = area + thetaProb(max_idx - idx_i);
        area = area + thetaProb(max_idx + idx_i);
        if area >areaThreshold
            minBD = max_idx - idx_i;
            maxBD = max_idx + idx_i;
            break;
        end
    end
    thetaRngP2C2 = thetaRngP2C(minBD:maxBD);
    thetaProb2 = thetaProb(minBD:maxBD);thetaProb2 = thetaProb2./sum(thetaProb2);
    
    
    angOpt = deg2rad(dot(rad2deg(thetaRngP2C2), thetaProb2));   % 0.5 0.8 0.0
    %  thetaExp = deg2rad(dot(rad2deg(thetaSamp(thetaProb1 > rati)), thetaProb2));
    thetaExp_tmp = angOpt - k2cRef;
    
end




thetaProbUse = interp1(thetaRngP2C2, thetaProb2, (thetaRng));
thetaProbUse(isnan(thetaProbUse)) = 0;
%             [ac,dc] = probDensity(angOpt, 0.5 ,4,4, rad2deg(thetaRng),mean(diff(thetaRng)), 'reproj');

[ac,dc] = probDensity(rad2deg(angOpt), 0.2 ,2,2, rad2deg(thetaRng),mean(diff(thetaRng)), 'reproj');
thetaProbUse = ac;
if 0
    figure, plot(rad2deg(thetaRngP2C2), thetaProb2);hold on;plot(rad2deg(thetaRng),thetaProbUse);
    
end
thetaProbUse = thetaProbUse./sum(thetaProbUse);


thetaRngP2C_use = thetaRng;



end

function [curPredPtIcs,inTraceFlag,curPredPtIcs0,inTraceFlag0,weightRaw,candX,candY,errPair,objPervPtCcsZGolden1] = NewTracking0(obj,keyFeatNum,prevFeatPtList,ptCcsZ,intrMat,k2cRef,angleModalOrg, keyDepthV, prevDepthV, currDepthV,DepthProbility, DepthId, AngleProbility,reCalc,DispRefineUse__,DispRng,objPervPtCcsZGolden)
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
ptCcsZGolden = LocalTrace.ptCcsZ;
if  size(LocalTrace.ptIcsX,2) > 2
    ptCcsZGolden(DepthId) = (intrMat(1,1).*norm(CamModel.transVec1To2))./(DispRng(:,(size(DispRng,2)+1)/2) + (princpPtR(1) - princpPtL(1)));
end





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
nextImg = rgb2gray(imgCur);
%             prevImg = keyFrameImgL;
prevImg = rgb2gray(imgPrvL);
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

%             objPervPtCcsZGolden = pervPtCcsZGolden;

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
            existIdx = inlierId;
            
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
            dltDisp = dltDisp(ismember(DepthId, existIdx),:);
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
        prvDepth = obj.prvDepthGT;
    else
        prvDepth = obj.prvDepthVisual;
        
    end
    if  size(LocalTrace.ptIcsX,2) > 2
        prvFeat = [existPt1.x existPt1.y];
        
        prvFeatInd = sub2ind(size(prvDepth), round(prvFeat(:,2)), round(prvFeat(:,1)));
        prvFeatDepth = prvDepth(prvFeatInd);
        prvFeatDisp = (intrMat(1,1).*norm(CamModel.transVec1To2)./prvFeatDepth) - (princpPtR(1) - princpPtL(1));
        
        
        dltDisp = DispRng - repmat(DispRng(:,(size(DispRng,2)+1)/2), 1, size(DispRng,2));
        dltDisp = dltDisp(ismember(DepthId, existIdx),:);
        if InheriDispRng
            %                         obj.setting.configParam.disparityOrginRange   = prvFeatDisp + obj.setting.configParam.disparityRange;
            obj.setting.configParam.disparityOrginRange   = repmat(prvFeatDisp,1,size(DispRng,2)) + dltDisp;
        else
            obj.setting.configParam.disparityOrginRange   = prvFeatDisp + obj.setting.configParam.disparityRange;
        end
        errCheckDisp1 = prvFeatDisp - DispRng(ismember(DepthId,existIdx),(size(DispRng,2)+1)/2);
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
    table = LookUpTables.GenerateTable(table,obj.setting.configParam,existPt1,existFeaturesNumber, imgSize,T_B2C,fbConst,intrMat,DepthProbility, DepthId, existIdx, AngleProbility,DispRefineUse__) ;
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
%         figure(FigBase + 9),clf;plot(err(errr<1,1),err(errr<1,2),'+r')
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




