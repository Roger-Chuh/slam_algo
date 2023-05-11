function [projDispMatAll, thetaProb,idM,angOpt, ReprojErrVecTmp, ReprojErrVecRTmp, ReprojErrVecTmpMatT, ReprojErrVecRTmpMatT,ValidFeatFlag,ValidFeatFlag0,ValidFeatFlagFusion,ValidFeatFlagQuant,ProbReprojVecAll,ProbReprojVecRAll, PixKeyVecTmp, PixKeyVecRTmp, Probb,validFeatFlagFusion,validFeatFlag, ProbReprojVec, ProbReprojVecR,bb,b,pixKeyVecTmpXCoordMatStack,pixKeyVecTmpYCoordMatStack,pixKeyVecRTmpXCoordMatStack,pixKeyVecRTmpYCoordMatStack, thetaExp, errEpiline, angOptEpi,depthC2KInd_ind, bbbOpt]...
         = ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, pt2dCur, pt2dCurR,ProbZ, ProbZTmp_norm, imgCur, imgCurR,thetaSamp,r_cam,tx, ty, tz)
global SHOWPOLYGON DRAWPOLYGON ANGLEONLY USEGOLDENDISP UPDATEDEPTH DEPTHITERNUM probPath ...
    USELEFT USELEFTPLUSRIGHT USELEFTXRIGHT USERIGHT WITHINEPILINE BOTHLR HARDCUT QUANTCUT FUSIONCUT ...
    FORCEEVENPLATFORM CUTTHETAANDFORCEPLATFORM DIFFTHETATHR CUTTHETAANDFORCEPLATFORM_NODIFF FORCEEXPEVEN SOFTP2 Rati Sc USEthetaProb0 ...
    Rati2 FORCETHETAPLATFORM1


fastVersion = true; false; true; 


[princpPtL, princpPtR] = Get(obj.camModel, 'PinholePrincpPt', obj.scaleLvl);

if obj.switchDepth 
    depthMapCur = obj.depthGT;
else
    depthMapCur = obj.depthVisual;
end

dispMapCur = (intrMat(1,1).*norm(obj.camModel.transVec1To2))./(depthMapCur + (princpPtR(1) - princpPtL(1)));
dispMapCur(dispMapCur < 0) = nan;


if ~exist('r_cam','var')
    r_cam = [];
    tx = [];
    ty = [];
    tz = [];
end

if ~exist('ConfigParam','var')
    ConfigParam = obj.configParam;
end
ReprojErrVecTmp = {}; ReprojErrVecRTmp = {}; ReprojErrVecTmpMatT = {}; ReprojErrVecRTmpMatT = {};
ValidFeatFlag = []; ValidFeatFlag0 = []; ValidFeatFlagFusion = [];ValidFeatFlagQuant = [];

sigma = ConfigParam.disparity_sigma_cur;
nDisp = ConfigParam.disparity_beta_cur;


metricPrevPtCcsKey = intrMat\HomoCoord(Pix',1);
metricPrevPtCcsKey = normc(metricPrevPtCcsKey);
metricPrevPtCcsKey = repmat(metricPrevPtCcsKey, 1, length(disparityRng));
zListTmp = intrMat(1)*baseline*ZZVec1(:)';
Scale00 = zListTmp./metricPrevPtCcsKey(3,:);
KeyCcsXYZVec = [ Scale00.*metricPrevPtCcsKey];



R_stack = zeros(3*length(thetaRng), 3*length(thetaRng));
t_stack = zeros(3* length(thetaRng), 1);
intrMat_stack = zeros(3*length(thetaRng), 3*length(thetaRng));
for ij = 1 : length(thetaRng)
    k2cBodyTmp = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(thetaRng(ij)),zeros(3,1));
    k2cCam = b2c*k2cBodyTmp.transformMat/b2c;
    R_stack(3*ij-2 : 3*ij,3*ij-2 : 3*ij) = k2cCam(1:3,1:3);
    intrMat_stack(3*ij-2 : 3*ij,3*ij-2 : 3*ij) = intrMat;
    
    t_stack(3*ij - 2:3*ij,1) = k2cCam(1:3,4);
end
if 0
    transR = R_stack*repmat(repmat(keyCcsXYZAll, length(thetaRng), 1), 1, length(disparityRng));
else
    transR = R_stack*(repmat(KeyCcsXYZVec, length(thetaRng), 1));
end
transRt = transR + repmat(t_stack, 1, size(transR, 2));

transRt_R = transRt + repmat(L2R(1:3,4), length(thetaRng), size(transR, 2));



reprojKey = intrMat_stack*transRt;

reprojKeyR = intrMat_stack*transRt_R;

div = reprojKey(3:3:end,:); divMat = repmat(div,1,1,3);divMat = permute(divMat, [3 1 2]); divMat = reshape(divMat, size(reprojKey));
divR = reprojKeyR(3:3:end,:); divMatR = repmat(divR,1,1,3);divMatR = permute(divMatR, [3 1 2]); divMatR = reshape(divMatR, size(reprojKey));
reprojKey = reprojKey./divMat;
reprojKeyR = reprojKeyR./divMatR;
reprojKey(3:3:end,:) = [];
reprojKeyR(3:3:end,:) = [];

for jj = 1 : length(thetaRng)
    
    if 0
        k2cBodyTmp = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(thetaRng(jj)),zeros(3,1));
        k2cCam = b2c*k2cBodyTmp.transformMat/b2c;
        homocurrCcsXYZALL = k2cCam*HomoCoord(keyCcsXYZAll,1);
        pixKey = pflat(intrMat*homocurrCcsXYZALL(1:3,:));
        pixKeyTmp = pixKey(1:2,:)';
    end
    % % % % % % % % % % % % %                         [~, reprojErrTmp] = NormalizeVector(pixKeyTmp - pt2dCur);
    % % % % % % % % % % % % %                         [probReproj, ~] = probDensity(0, ConfigParam.disparity_sigma, thetaRng,ConfigParam.theta_sample_step, 'theta');
    
    % %                         DispRng = repmat(dispList(inlierId,:),1,length(disparityRng)) + repmat(disparityRng,length(inlierId),1);
    % %                         for jk = 1 : length(inlierId)
    % %                              [ProbZ(jk,:), ZZVec1(jk,:)] = probDensity(dispList(inlierId(jk),:), ConfigParam.disparity_sigma, DispRng(jk,:),ConfigParam.disparity_sample_interval, 'disparity');
    % %                         end
    
    if ~fastVersion
        [reProjErr2(jj,1),err2(:,jj)] = AngleSpace.ProjectErrorUnderTransform2(ConfigParam.reproj_sigma, k2cCam(1:3,1:3), k2cCam(1:3,4), Pix, pt2dCur, intrMat,r_cam,tx, ty, tz);
    end
    
    if 0
        metricPrevPtCcsKey = intrMat\HomoCoord(Pix',1);
        metricPrevPtCcsKey = normc(metricPrevPtCcsKey);
        metricPrevPtCcsKey = repmat(metricPrevPtCcsKey, 1, length(disparityRng));
        zListTmp = intrMat(1)*baseline*ZZVec1(:)';
        Scale00 = zListTmp./metricPrevPtCcsKey(3,:);
        KeyCcsXYZVec = [ Scale00.*metricPrevPtCcsKey];
    end
    
    
    if 0
        homocurrCcsXYZALLVec = k2cCam*HomoCoord(KeyCcsXYZVec,1); homocurrCcsXYZALLVecR = L2R*homocurrCcsXYZALLVec;
        pixKeyVec = pflat(intrMat*homocurrCcsXYZALLVec(1:3,:)); pixKeyVecR = pflat(intrMat*homocurrCcsXYZALLVecR(1:3,:));
        pixKeyVecTmp = pixKeyVec(1:2,:)'; pixKeyVecRTmp = pixKeyVecR(1:2,:)';
        
        
        
        
        transRt(3*jj-2:3*jj, :) - homocurrCcsXYZALLVec(1:3,:);
        transRt_R(3*jj-2:3*jj, :) - homocurrCcsXYZALLVecR(1:3,:);
        
        
        
        pixKeyVecTmp - reprojKey(2*jj-1:2*jj,:)';
        pixKeyVecRTmp - reprojKeyR(2*jj-1:2*jj,:)';
        
        
        
        
    else
        homocurrCcsXYZALLVec = pextend(transRt(3*jj-2:3*jj, :));
        homocurrCcsXYZALLVecR = pextend(transRt_R(3*jj-2:3*jj, :));
        pixKeyVecTmp = reprojKey(2*jj-1:2*jj,:)';
        pixKeyVecRTmp = reprojKeyR(2*jj-1:2*jj,:)';
    end
    
    PixKeyVecTmp{jj,1} = pixKeyVecTmp; PixKeyVecRTmp{jj,1} = pixKeyVecRTmp;
    
    
    % %                            figure,imshow(imgCur);hold on;plot(pixKeyVecTmp(:,1), pixKeyVecTmp(:,2),'.g');plot(pt2dCur(:,1), pt2dCur(:,2),'.r');
    %                            figure,imshow(imgCurR);hold on;plot(pixKeyVecRTmp(:,1), pixKeyVecRTmp(:,2),'.g');plot(pt2dCurR(:,1), pt2dCurR(:,2),'.r');
    %%
    if 1
        [errVec, reprojErrVecTmp] = NormalizeVector(pixKeyVecTmp - repmat(pt2dCur,length(disparityRng) ,1)); [errVecR, reprojErrVecRTmp] = NormalizeVector(pixKeyVecRTmp - repmat(pt2dCurR,length(disparityRng) ,1));
    else
        [errVec, reprojErrVecTmp] = NormalizeVector(pixKeyVecTmp(:,1) - repmat(pt2dCur(:,1),length(disparityRng) ,1)); [errVecR, reprojErrVecRTmp] = NormalizeVector(pixKeyVecRTmp(:,1) - repmat(pt2dCurR(:,1),length(disparityRng) ,1));
    end
    
    idValid = find(pixKeyVecTmp(:,1) > 0 & pixKeyVecTmp(:,2) > 0 & pixKeyVecTmp(:,1) < size(dispMapCur,2) & pixKeyVecTmp(:,2) < size(dispMapCur,1));
    
    reProjPixInd = sub2ind(size(dispMapCur), round(pixKeyVecTmp(idValid,2)), round(pixKeyVecTmp(idValid,1)));
    
    dispCandList = nan(size(pixKeyVecTmp,1),1);
    dispCandList(idValid,1) = dispMapCur(reProjPixInd);
    
    dispK2C = (intrMat(1,1).*norm(obj.camModel.transVec1To2))./(homocurrCcsXYZALLVec(3,:)' + (princpPtR(1) - princpPtL(1)));   
    
    dispK2CMat = reshape(dispK2C, size(ProbZ));
    
%     sigma = ConfigParam.disparity_sigma_cur; 
%     nDisp = ConfigParam.disparity_beta_cur;
    projDisp = 1/(sqrt(2*pi)*sigma)* exp(-(abs(dispK2C - dispCandList)).^nDisp./2.*sigma^(-nDisp));
    projDisp(isnan(projDisp)) = 0;
    
    
    
    
    projDispMat = reshape(projDisp, size(ProbZ));
    projDispMatAll(jj,:,:) = projDispMat;
    
    sdbhj = 1;
% % % %     ZZVec1_proj = [];ProbZ_proj = [];
% % % %     for jk = 1 : length(dispK2C)
% % % %         %             [ProbZ_proj(jk,:), ZZVec1_proj(jk,:)] = probDensity(dispK2C((jk),:), ConfigParam.disparity_sigma_cur,ConfigParam.disparity_beta_cur,ConfigParam.reproj_beta, dispK2CMat(jk,:),ConfigParam.disparity_sample_interval, 'disparity');
% % % %         [ProbZ_proj(jk,:), ZZVec1_proj(jk,:)] = probDensity(dispK2C((jk),:), ConfigParam.disparity_sigma_cur,ConfigParam.disparity_beta_cur,ConfigParam.reproj_beta, dispK2C(jk,:),ConfigParam.disparity_sample_interval, 'disparity');
% % % %     end
    
    
    errVec = pixKeyVecTmp - repmat(pt2dCur,length(disparityRng) ,1);
    errVecR = pixKeyVecRTmp - repmat(pt2dCurR,length(disparityRng) ,1);
    ReprojErrVecTmp{jj,1} = reprojErrVecTmp;
    ReprojErrVecRTmp{jj,1} = reprojErrVecRTmp;
    
    pixKeyVecTmpXCoord = pixKeyVecTmp(:,1);
    pixKeyVecTmpYCoord = pixKeyVecTmp(:,2);
    pixKeyVecTmpXCoordMat = reshape(pixKeyVecTmpXCoord, size(ProbZ));
    pixKeyVecTmpYCoordMat = reshape(pixKeyVecTmpYCoord, size(ProbZ));
    pixKeyVecTmpXCoordMatT = pixKeyVecTmpXCoordMat';
    pixKeyVecTmpYCoordMatT = pixKeyVecTmpYCoordMat';
    
    
    pixKeyVecTmpXCoordMatStack(jj,:,:) = pixKeyVecTmpXCoordMat;
    pixKeyVecTmpYCoordMatStack(jj,:,:) = pixKeyVecTmpYCoordMat;
    
    
    
    pixKeyVecRTmpXCoord = pixKeyVecRTmp(:,1);
    pixKeyVecRTmpYCoord = pixKeyVecRTmp(:,2);
    pixKeyVecRTmpXCoordMat = reshape(pixKeyVecRTmpXCoord, size(ProbZ));
    pixKeyVecRTmpYCoordMat = reshape(pixKeyVecRTmpYCoord, size(ProbZ));
    pixKeyVecRTmpXCoordMatT = pixKeyVecRTmpXCoordMat';
    pixKeyVecRTmpYCoordMatT = pixKeyVecRTmpYCoordMat';
    
    pixKeyVecRTmpXCoordMatStack(jj,:,:) = pixKeyVecRTmpXCoordMat;
    pixKeyVecRTmpYCoordMatStack(jj,:,:) = pixKeyVecRTmpYCoordMat;
    
    
    
    
    ProbZTmp_normT = ProbZTmp_norm';
    
    %                            reprojErrVecTmpMat = reshape(reprojErrVecTmp, size(DispRng));
    reprojErrVecTmpMat = reshape(reprojErrVecTmp, size(ProbZ)); reprojErrVecRTmpMat = reshape(reprojErrVecRTmp, size(ProbZ));
    reprojErrVecTmpMatX = reshape(errVec(:,1), size(ProbZ)); reprojErrVecRTmpMatX = reshape(errVecR(:,1), size(ProbZ));
    
    reprojErrVecTmpMatT = reprojErrVecTmpMat'; reprojErrVecRTmpMatT = reprojErrVecRTmpMat';
    reprojErrVecTmpMatTX = reprojErrVecTmpMatX'; reprojErrVecRTmpMatTX = reprojErrVecRTmpMatX';
    
    
    %                                 wheelNew4 = interp1(pixKeyVecTmpXCoordMat,ProbZTmp_norm,repmat(pt2dCur(:,1), 1,size(pixKeyVecTmpXCoordMat,2)));
    %                                 wheelNew4 = interp1(pixKeyVecTmpXCoordMatT(:),ProbZTmp_normT(:),repmat(pt2dCur(:,1), 1,1)');
    
    if 0
        for uio = 1 : size(pixKeyVecTmpXCoordMat,1)
            interpLXProbZ(:,uio) = interp1(pixKeyVecTmpXCoordMat(uio,:),ProbZTmp_norm(uio,:),repmat(pt2dCur(uio,1), 1,1));
            interpRXProbZ(:,uio) =  interp1(pixKeyVecRTmpXCoordMat(uio,:),ProbZTmp_norm(uio,:),repmat(pt2dCurR(uio,1), 1,1));
        end
    else
        %                                     vldLXProbZ = pixKeyVecTmpXCoordMat > repmat(pt2dCur(:,1), 1,size(pixKeyVecTmpXCoordMat,2)) & pixKeyVecTmpXCoordMat < repmat(pt2dCur(:,1), 1,size(pixKeyVecTmpXCoordMat,2));
        vldLXProbZ = abs(pixKeyVecTmpXCoordMat - repmat(pt2dCur(:,1), 1,size(pixKeyVecTmpXCoordMat,2)));
        [~,vldLXProbZMinId] = min(vldLXProbZ');
        vldLXProbZMinIdCoord = [vldLXProbZMinId' [1:size(ProbZ,1)]'];
        vldLXProbZMinIdCoordInd = sub2ind(size(ProbZ), vldLXProbZMinIdCoord(:,2), vldLXProbZMinIdCoord(:,1));
        interpLXProbZ = ProbZTmp_norm(vldLXProbZMinIdCoordInd)';
        interpLXProbZ(vldLXProbZMinId == 1 | vldLXProbZMinId == size(pixKeyVecTmpXCoordMat,2)) = 0;
        
        vldLXProbZR = abs(pixKeyVecRTmpXCoordMat - repmat(pt2dCurR(:,1), 1,size(pixKeyVecRTmpXCoordMat,2)));
        [~,vldLXProbZMinIdR] = min(vldLXProbZR');
        vldLXProbZMinIdCoordR = [vldLXProbZMinIdR' [1:size(ProbZ,1)]'];
        vldLXProbZMinIdCoordIndR = sub2ind(size(ProbZ), vldLXProbZMinIdCoordR(:,2), vldLXProbZMinIdCoordR(:,1));
        interpRXProbZ = ProbZTmp_norm(vldLXProbZMinIdCoordIndR)';
        interpRXProbZ(vldLXProbZMinIdR == 1 | vldLXProbZMinIdR == size(pixKeyVecTmpXCoordMat,2)) = 0;
    end
    
    epiLine_margin = ConfigParam.epiLine_margin;
    flagL0 = sign(reprojErrVecTmpMatTX(1,:)).*sign(reprojErrVecTmpMatTX(end,:)) < 0;
    flagR0 = sign(reprojErrVecRTmpMatTX(1,:)).*sign(reprojErrVecRTmpMatTX(end,:)) < 0;
    
    if BOTHLR
        validFeatFlag0 = double((flagL0 & abs(reprojErrVecTmpMatTX(1,:)) > epiLine_margin & abs(reprojErrVecTmpMatTX(end,:)) > epiLine_margin) & (flagR0 & abs(reprojErrVecRTmpMatTX(1,:)) > epiLine_margin & abs(reprojErrVecRTmpMatTX(end,:)) > epiLine_margin));
    else
        %                                     validFeatFlag = double(flagL | flagR);
        validFeatFlag0 = double((flagL0 & abs(reprojErrVecTmpMatTX(1,:)) > epiLine_margin & abs(reprojErrVecTmpMatTX(end,:)) > epiLine_margin) | (flagR0 & abs(reprojErrVecRTmpMatTX(1,:)) > epiLine_margin & abs(reprojErrVecRTmpMatTX(end,:)) > epiLine_margin));
    end
    ValidFeatFlag0 = [ValidFeatFlag0; validFeatFlag0];
    
    flagLHardCut = sign(reprojErrVecTmpMatTX(1,:)).*sign(reprojErrVecTmpMatTX(end,:)) < 0;
    flagRHardCut = sign(reprojErrVecRTmpMatTX(1,:)).*sign(reprojErrVecRTmpMatTX(end,:)) < 0;
    
    if HARDCUT
        flagL = sign(reprojErrVecTmpMatTX(1,:)).*sign(reprojErrVecTmpMatTX(end,:)) < 0;
        flagR = sign(reprojErrVecRTmpMatTX(1,:)).*sign(reprojErrVecRTmpMatTX(end,:)) < 0;
    else
        if QUANTCUT
            flagL = interpLXProbZ > ConfigParam.probZ_ratio_threshold;
            flagR = interpRXProbZ > ConfigParam.probZ_ratio_threshold;
        else
            flagL = interpLXProbZ; % > ConfigParam.probZ_ratio_threshold;
            flagR = interpRXProbZ; % > ConfigParam.probZ_ratio_threshold;
        end
        
    end
    
    if BOTHLR
        validFeatFlag = double((flagL & abs(reprojErrVecTmpMatTX(1,:)) > epiLine_margin & abs(reprojErrVecTmpMatTX(end,:)) > epiLine_margin) & (flagR & abs(reprojErrVecRTmpMatTX(1,:)) > epiLine_margin & abs(reprojErrVecRTmpMatTX(end,:)) > epiLine_margin));
        validFeatFlagQuant = double((flagLHardCut & abs(reprojErrVecTmpMatTX(1,:)) > epiLine_margin & abs(reprojErrVecTmpMatTX(end,:)) > epiLine_margin) & (flagRHardCut & abs(reprojErrVecRTmpMatTX(1,:)) > epiLine_margin & abs(reprojErrVecRTmpMatTX(end,:)) > epiLine_margin));
    else
        %                                     validFeatFlag = double(flagL | flagR);
        try
            validFeatFlag = double((flagL & abs(reprojErrVecTmpMatTX(1,:)) > epiLine_margin & abs(reprojErrVecTmpMatTX(end,:)) > epiLine_margin) | (flagR & abs(reprojErrVecRTmpMatTX(1,:)) > epiLine_margin & abs(reprojErrVecRTmpMatTX(end,:)) > epiLine_margin));
        catch
            dfnjk = 1;
        end
        validFeatFlagQuant = double((flagLHardCut & abs(reprojErrVecTmpMatTX(1,:)) > epiLine_margin & abs(reprojErrVecTmpMatTX(end,:)) > epiLine_margin) | (flagRHardCut & abs(reprojErrVecRTmpMatTX(1,:)) > epiLine_margin & abs(reprojErrVecRTmpMatTX(end,:)) > epiLine_margin));
        
    end
    
    if ~HARDCUT
        if ~QUANTCUT
            validFeatFlag = flagL;
            
        end
    end
    % %                                 validFeatFlag = double((flagL & abs(reprojErrVecTmpMatTX(1,:)) > epiLine_margin & abs(reprojErrVecTmpMatTX(end,:)) > epiLine_margin));
    
    ValidFeatFlag = [ValidFeatFlag; validFeatFlag];
    ValidFeatFlagQuant = [ValidFeatFlagQuant; validFeatFlagQuant];
    
    
    ReprojErrVecTmpMatT{jj,1} = reprojErrVecTmpMatT;
    ReprojErrVecRTmpMatT{jj,1} = reprojErrVecRTmpMatT;
    if ~SOFTP2
        [probReprojVec22, ~] = probDensity(0, ConfigParam.reproj_sigma,ConfigParam.disparity_beta,ConfigParam.reproj_beta, reprojErrVecTmpMatT(:)',ConfigParam.reproj_sample_interval, 'reproj');
        [probReprojVecR22, ~] = probDensity(0, ConfigParam.reproj_sigma_right,ConfigParam.disparity_beta,ConfigParam.reproj_beta_right, reprojErrVecRTmpMatT(:)',ConfigParam.reproj_sample_interval, 'reproj');
    else
        [probReprojVec22, ~] = probDensity(0, min(5,max(ConfigParam.reproj_sigma, size(LocalTrace.ptIcsX, 2)*0.5)),ConfigParam.disparity_beta,ConfigParam.reproj_beta, reprojErrVecTmpMatT(:)',ConfigParam.reproj_sample_interval, 'reproj');
        [probReprojVecR22, ~] = probDensity(0, min(5,max(ConfigParam.reproj_sigma_right,size(LocalTrace.ptIcsX, 2)*0.5)),ConfigParam.disparity_beta,ConfigParam.reproj_beta_right, reprojErrVecRTmpMatT(:)',ConfigParam.reproj_sample_interval, 'reproj');
    end
    probReprojVec = reshape(probReprojVec22, size(reprojErrVecTmpMat,2),size(reprojErrVecTmpMat,1))'; probReprojVecR = reshape(probReprojVecR22, size(reprojErrVecRTmpMat,2),size(reprojErrVecRTmpMat,1))';
    ProbReprojVec{jj,1} = probReprojVec;  ProbReprojVecR{jj,1} = probReprojVecR;
    ProbReprojVecAll(jj,:,:) = probReprojVec; ProbReprojVecRAll(jj,:,:) = probReprojVecR;
    %                            probReprojVec = [];
    %                            for jk = 1 : length(inlierId)
    %                                %                              [ProbZ(jk,:), ZZVec1(jk,:)] = probDensity(dispList(inlierId(jk),:), ConfigParam.disparity_sigma, DispRng(jk,:),ConfigParam.disparity_sample_interval, 'disparity');
    %                                [probReprojVec(jk,:), ~] = probDensity(0, ConfigParam.reproj_sigma, reprojErrVecTmpMat(jk,:),ConfigParam.reproj_sample_interval, 'reproj');
    %                                %
    %                            end
    
    
    validFeatFlagFusionMat = ((ProbZ.*(probReprojVec.*probReprojVecR)));
    %                                 validFeatFlagFusionMat = ((ProbZ.*(probReprojVec)));
    if BOTHLR
        inFusionFlag = abs(reprojErrVecTmpMatT) < ConfigParam.reproj_sigma & abs(reprojErrVecRTmpMatT) < ConfigParam.reproj_sigma_right;
    else
        inFusionFlag = abs(reprojErrVecTmpMatT) < ConfigParam.reproj_sigma | abs(reprojErrVecRTmpMatT) < ConfigParam.reproj_sigma_right;
    end
    inFusionFlag = abs(reprojErrVecTmpMatT) < ConfigParam.reproj_sigma;
    inFusionFlag = double(inFusionFlag);
    inFusionFlag(find(inFusionFlag == 0)) = nan;
    if 1
        validFeatFlagFusion = max((validFeatFlagFusionMat.*inFusionFlag')');
    elseif 0
        validFeatFlagFusionMatIn = ((validFeatFlagFusionMat.*inFusionFlag'))';
        validFeatFlagFusionMatIn_ = immultiply(validFeatFlagFusionMatIn, ~isnan(validFeatFlagFusionMatIn));
        validFeatFlagFusionMatInSum = sum(validFeatFlagFusionMatIn_);
        validFeatFlagFusionMatInSumCount = sum(~isnan(validFeatFlagFusionMatIn));
        validFeatFlagFusion = validFeatFlagFusionMatInSum./validFeatFlagFusionMatInSumCount;
    else
        validFeatFlagFusion = max((validFeatFlagFusionMat)');
    end
    validFeatFlagFusion(isnan(validFeatFlagFusion)) = 0;
    ValidFeatFlagFusion = [ValidFeatFlagFusion; validFeatFlagFusion];
    if USERIGHT
        %                                     Probb(jj,:,:) = ProbZ.*probReprojVec.*probReprojVecR;
        Probb(jj,:,:) = ProbZ.*probReprojVecR;
    end
    % % % % % % %                                     Probb(jj,:,:) = ProbZ.*(probReprojVec + probReprojVecR);
    if USELEFT
        Probb(jj,:,:) = ProbZ.*probReprojVec;
    end
    
    if USELEFTPLUSRIGHT
        Probb(jj,:,:) = ProbZ.*(probReprojVec + probReprojVecR);
    end
    
    if USELEFTXRIGHT
        Probb(jj,:,:) = ProbZ.*(probReprojVec.*probReprojVecR);
    end
    
    tempProb = permute(Probb(jj,:,:),[2 3 1]);
    tempProbMax = repmat(max(tempProb')',1,size(tempProb,2));
    tempProb_ = tempProb./tempProbMax;
    tempProb_(isnan(tempProb_)|isinf(tempProb_)) = 0;
    tempProb(tempProb_ > 0.8) = tempProbMax(tempProb_ > 0.8);
    %     Probb(jj,:,:) = tempProb;
    %                            if jj <= (length(thetaRng)+1)/2 && jj >= (length(thetaRng)+1)/2 - 8
    
    
    randId = 1;
    tmpProb = Probb(jj,randId,:);
    tmpProb2 = permute(Probb(jj,:,:),[2 3 1]);
    maxx = (max(tmpProb2'));
    maxx(isnan(maxx) | isinf(maxx)) = [];
    maxxsum = sum((maxx));
    
    if jj == (length(thetaRng)+1)/2 % && jj >= (length(thetaRng)+1)/2 - 8
        
        %         randId = 10;
        %         tmpProb = Probb(jj,randId,:);
        %         tmpProb2 = permute(Probb(jj,:,:),[2 3 1]);
        %         maxx = (max(tmpProb2'));
        %         maxx(isnan(maxx) | isinf(maxx)) = [];
        %         maxxsum = sum((maxx));
        %                            figure(33),clf;subplot(2,1,1);plot(ZZVec1(randId,:),ProbZ(randId,:));hold on;plot(ZZVec1(randId,:),probReprojVec(randId,:));
        %                                           subplot(2,1,2);plot(ZZVec1(randId,:),tmpProb(:)');
        if 0
            figure(33),clf;subplot(4,1,1);plot(ProbZ');title(num2str(rad2deg(thetaRng(jj))));  subplot(4,1,2);plot(probReprojVec'); subplot(4,1,3);plot(probReprojVecR'); subplot(4,1,4);plot(tmpProb2');title(num2str(maxxsum));
            figure(35),clf;subplot(1,2,1);imshow(imgCur);hold on;plot(pixKeyVecTmp(:,1), pixKeyVecTmp(:,2),'.g');plot(pt2dCur(:,1), pt2dCur(:,2),'.r');title(num2str(rad2deg(thetaRng(jj))));
            subplot(1,2,2);imshow(imgCurR);hold on;plot(pixKeyVecRTmp(:,1), pixKeyVecRTmp(:,2),'.g');plot(pt2dCurR(:,1), pt2dCurR(:,2),'.r'); title('cur right');
        end
    end
    %                         for j = 1 : length(inlierId)
    for j = 1 : 0
        pixCur = ([LocalTrace.ptIcsX(inlierId(j),keyLength) LocalTrace.ptIcsY(inlierId(j),keyLength)]);
        %                                         plot(pixCur(1),pixCur(2),'.r');
        pix = ([LocalTrace.ptIcsX(inlierId(j),1) LocalTrace.ptIcsY(inlierId(j),1)]);
        zTrue = zList(inlierId(j));
        PixCur = [PixCur;pixCur];
        dispTrue = dispList(inlierId(j));
        
        if (zTrue) < 0 || isnan(dispTrue)
            continue;
        else
            cntt = cntt + 1;
        end
        
        try
            if 1
                zRng = [intrMat(1)*baseline/(dispTrue + ConfigParam.disparity_error) intrMat(1)*baseline/(max(0.1, dispTrue - ConfigParam.disparity_error))];
                dispRng = dispTrue + disparityRng;
            else
                zRng = ConfigParam.depth_uncertainty_range + zTrue;
            end
        catch
            if 1
                zRng = [intrMat(1)*baseline/(dispTrue + 3.5) intrMat(1)*baseline/(dispTrue - 3.5)];
            else
                zRng = [-200 300] + zTrue;
            end
        end
        [probZ, ZVec1] = probDensity(dispTrue, ConfigParam.disparity_sigma,ConfigParam.disparity_beta,ConfigParam.reproj_beta, dispRng,ConfigParam.disparity_sample_interval, 'disparity');
        
        
        
        % % %                             metricPrevPtCcs = intrMat\HomoCoord(pix',1);
        % % %                             metricPrevPtCcs = normc(metricPrevPtCcs);
        % % %                             scale1 = zRng./metricPrevPtCcs(3,:)';
        % % %                             scale0 = zTrue./metricPrevPtCcs(3,:)';
        % % %                             keyCcsXYZ = [scale1(1).*metricPrevPtCcs scale1(2).*metricPrevPtCcs scale0.*metricPrevPtCcs];
        % % %                             %                         try
        % % %                             %                             thetaRng = ConfigParam.theta_range + theta;
        % % %                             %                             thetaRng = ConfigParam.theta_range + theta;
        % % %                             %                         catch
        % % %                             %                             thetaRng = deg2rad([-0.5 0.5]) + theta;
        % % %                             %                         end
        % % %
        % % %                             k2cBodyPmat0 = [roty(rad2deg(theta)) [0;0;0];0 0 0 1];
        % % %                             k2cCamPmat0 = b2c*k2cBodyPmat0/b2c;
        % % %                             homocurrCcsXYZ0 = k2cCamPmat0*HomoCoord(keyCcsXYZ(:,3),1);
        % % %                             pix0 = pflat(intrMat*homocurrCcsXYZ0(1:3));
        
        
        
        metricPrevPtCcs = intrMat\HomoCoord(pix',1);
        metricPrevPtCcs = normc(metricPrevPtCcs);
        zListTmp = intrMat(1)*baseline*ZVec1;
        scale00 = zListTmp./metricPrevPtCcs(3,:);
        keyCcsXYZVec = [ scale00.*repmat(metricPrevPtCcs,1,length(scale00))];
        
        k2cBodyPmat0 = k2cBodyTmp.transformMat; %[roty(rad2deg(theta)) [0;0;0];0 0 0 1];
        k2cCamPmat0 = b2c*k2cBodyPmat0/b2c;
        homocurrCcsXYZ00 = k2cCamPmat0*HomoCoord(keyCcsXYZVec,1);
        pix00 = pflat(intrMat*homocurrCcsXYZ00(1:3,:));
        %                             pix00 = pflat(intrMat*pixKey);
        %                             pix00 = (pixKey);
        [~, reprojTmp] = NormalizeVector(pix00(1:2,:)' - repmat(pixCur,length(scale00),1));
        %                             figure,imshow(zeros(240,320));hold on;plot(pix00(1,:),pix00(2,:),'.r');hold on;plot(pixCur(1), pixCur(2),'xg')
        
        %                                         Pix0 = [Pix0; pix0(1:2)'];
        % % %                             reprojError(cntt,:) = norm([pix0(1:2)'] - [pixCur]);
        probReproj = [];
        %                             for kk = 1 : length(reprojTmp)
        %                                 [probReproj(kk,1), ~] = probDensity(0, ConfigParam.reproj_sigma, reprojTmp(kk),ConfigParam.reproj_sample_interval, 'theta');
        %                             end
        [probReproj, ~] = probDensity(0, ConfigParam.reproj_sigma, reprojTmp',ConfigParam.reproj_sample_interval, 'reproj');
        %                             Prob1(jj,j,:) = max(probZ.*probReproj);
        Prob(jj,j,:) = (probZ.*probReproj);
        % %                             PixReproj = [PixReproj;[pix0(1) pix0(2)]];
        
        %                                         if inlierId(j) == 1222
        %                                             figure(232),plot([pix0(1)'] - [pixCur(1)],[pix0(2)'] - [pixCur(2)],'+');
        %                                         end
        vldReproj = [vldReproj; j];
        
        if j == (length(disparityRng)+1)/2 && jj == (length(thetaRng)+1)/2
            figure(32),clf;subplot(2,1,1);plot(ZVec1,probZ);hold on;plot(ZVec1,probReproj);legend('depthProb','reprojProb');subplot(2,1,2);plot(ZVec1,probZ.*probReproj);legend('thetaProb');
            dsvk = 1;
        end
        % % % % %                             k2cBodyPmat1 = [roty(rad2deg(thetaRng(1))) [0;0;0];0 0 0 1];
        % % % % %                             k2cCamPmat1 = b2c*k2cBodyPmat1/b2c;
        % % % % %                             homocurrCcsXYZ1 = k2cCamPmat1*HomoCoord(keyCcsXYZ(:,1),1);
        % % % % %                             pix1 = pflat(intrMat*homocurrCcsXYZ1(1:3));
        % % % % %
        % % % % %
        % % % % %                             k2cBodyPmat2 = [roty(rad2deg(thetaRng(2))) [0;0;0];0 0 0 1];
        % % % % %                             k2cCamPmat2 = b2c*k2cBodyPmat2/b2c;
        % % % % %                             homocurrCcsXYZ2 = k2cCamPmat2*HomoCoord(keyCcsXYZ(:,1),1);
        % % % % %                             pix2 = pflat(intrMat*homocurrCcsXYZ2(1:3));
        % % % % %
        % % % % %                             k2cBodyPmat3 = [roty(rad2deg(thetaRng(1))) [0;0;0];0 0 0 1];
        % % % % %                             k2cCamPmat3 = b2c*k2cBodyPmat3/b2c;
        % % % % %                             homocurrCcsXYZ3 = k2cCamPmat3*HomoCoord(keyCcsXYZ(:,2),1);
        % % % % %                             pix3 = pflat(intrMat*homocurrCcsXYZ3(1:3));
        % % % % %
        % % % % %                             k2cBodyPmat4 = [roty(rad2deg(thetaRng(2))) [0;0;0];0 0 0 1];
        % % % % %                             k2cCamPmat4 = b2c*k2cBodyPmat4/b2c;
        % % % % %                             homocurrCcsXYZ4 = k2cCamPmat4*HomoCoord(keyCcsXYZ(:,2),1);
        % % % % %                             pix4 = pflat(intrMat*homocurrCcsXYZ4(1:3));
        % % % % %
        % % % % %                             square = [pix1(1:2) pix2(1:2) pix4(1:2) pix3(1:2)  pix1(1:2)]';
        %          plot(square(:,1),square(:,2),'-b');
        %                                         plot(square([1 2],1),square([1 2],2),'-c'); plot(square([3 4],1),square([3 4],2),'-c');
        %                                         plot(square([1 4],1),square([1 4],2),'-b'); plot(square([2 3],1),square([2 3],2),'-b');
        %                                         plot(pix0(1),pix0(2),'.g');
        
        
        
        
        % % %                         margLen = ConfigParam.polygon_margin;
        % % %
        % % %                         thetaPt1 = [square([1 4],:) [1 1]']; thetaPt2 = [square([2 3],:) [1 1]'];
        % % %                         depthPt1 = [square([1 2],:) [1 1]']; depthPt2 = [square([3 4],:) [1 1]'];
        % % %
        % % %
        % % %                         thetaLine1 = cross(thetaPt1(1,:), thetaPt1(2,:)); thetaLine2 = cross(thetaPt2(1,:), thetaPt2(2,:));
        % % %                         depthLine1 = cross(depthPt1(1,:), depthPt1(2,:)); depthLine2 = cross(depthPt2(1,:), depthPt2(2,:));
        % % %
        % % %                         depthLine1 = depthLine1./norm(depthLine1(1:2)); depthLine2 = depthLine2./norm(depthLine2(1:2));
        % % %                         %                                         [intersectPt,dist] = CalcPt2Plane2([depthLine1(1:2) 0],[square(1,:) 0],[pixCur 0]);
        % % %                         %                                         dot(intersectPt, depthLine1);
        % % %                         [retVal1, dist1] = Pt2Line(square(1,:), square(2,:), pixCur); [retVal2, dist2] = Pt2Line(pixCur, retVal1, pix0(1:2)');
        % % %                         [retVal3, dist3] = Pt2Line(square(3,:), square(4,:), pixCur); [retVal4, dist4] = Pt2Line(pixCur, retVal3, pix0(1:2)');
        % % %
        % % %                         candi = [retVal2; retVal4];
        % % %                         [~,iderr5] = min([dist2 dist4]);
        % % %                         candi_ = candi(iderr5,:);
        % % %
        % % %
        % % %                         err1 = dot([pixCur 1], thetaLine1); err2 = dot([pixCur 1], thetaLine2);
        % % %                         thetaLineIntersect = cross(thetaLine1, thetaLine2);
        % % %                         thetaLineIntersect = [thetaLineIntersect(1)/thetaLineIntersect(3) thetaLineIntersect(2)/thetaLineIntersect(3)];
        % % %                         square2 = [square(1:4,:); thetaLineIntersect];
        % % % % % %                         if ANGLEONLY
        % % % % % %                             if 0
        % % % % % %                                 dist11 = norm(thetaLineIntersect - thetaPt1(1,1:2));
        % % % % % %                                 dist12 = norm(thetaLineIntersect - thetaPt1(2,1:2));
        % % % % % %                                 %                                             k1 = -thetaLine1(1)/thetaLine1(2);
        % % % % % %                                 dirVec = [(thetaPt1(1,1) - thetaLineIntersect(1)) (thetaPt1(1,2) - thetaLineIntersect(2))];
        % % % % % %                                 dirVec = dirVec./norm(dirVec);
        % % % % % %                                 if dist11 > dist12
        % % % % % %                                     thetaPt1_3 = thetaPt1(1,1:2) + margLen*dirVec;
        % % % % % %                                 else
        % % % % % %                                     thetaPt1_3 = thetaPt1(2,1:2) + margLen*dirVec;
        % % % % % %                                 end
        % % % % % %                                 dist13 = norm(thetaLineIntersect - thetaPt1_3);
        % % % % % %
        % % % % % %                                 dist21 = norm(thetaLineIntersect - thetaPt2(1,1:2));
        % % % % % %                                 dist22 = norm(thetaLineIntersect - thetaPt2(2,1:2));
        % % % % % %                                 %                                             k1 = -thetaLine1(1)/thetaLine1(2);
        % % % % % %                                 dirVec2 = [(thetaPt2(1,1) - thetaLineIntersect(1)) (thetaPt2(1,2) - thetaLineIntersect(2))];
        % % % % % %                                 dirVec2 = dirVec2./norm(dirVec2);
        % % % % % %                                 if dist21 > dist22
        % % % % % %                                     thetaPt2_3 = thetaPt2(1,1:2) + margLen*dirVec2;
        % % % % % %                                 else
        % % % % % %                                     thetaPt2_3 = thetaPt2(2,1:2) + margLen*dirVec2;
        % % % % % %                                 end
        % % % % % %                                 dist23 = norm(thetaLineIntersect - thetaPt2_3);
        % % % % % %                             else
        % % % % % %
        % % % % % %                                 if 0
        % % % % % %                                     num1 = dot(thetaLine1, pix0);
        % % % % % %                                     num2 = dot(thetaLine1, [pixCur 1]);
        % % % % % %                                     num3 = dot(thetaLine2, pix0);
        % % % % % %                                     num4 = dot(thetaLine2, [pixCur 1]);
        % % % % % %                                 else
        % % % % % %                                     num1 = dot(thetaLine1, pix0);
        % % % % % %                                     num2 = dot(thetaLine1, [candi_ 1]);
        % % % % % %                                     num3 = dot(thetaLine2, pix0);
        % % % % % %                                     num4 = dot(thetaLine2, [candi_ 1]);
        % % % % % %                                 end
        % % % % % %                                 %                                                 [intersect,dist] = CalcPt2Plane2(lineN,ptOnline,ptIso);
        % % % % % %
        % % % % % %                                 if (sign(num1) == sign(num2) && sign(num3) == sign(num4))  % || err5 < ConfigParam.polygon_inlier_thresh
        % % % % % %                                     in = true; on = true;
        % % % % % %                                 else
        % % % % % %                                     in = false; on = false;
        % % % % % %                                 end
        % % % % % %                             end
        % % % % % %
        % % % % % %
        % % % % % %
        % % % % % %                             %                                             [in,on] = inpolygon(pixCur(1),pixCur(2),square2(1:5,1),square2(1:5,2));
        % % % % % %                         else
        % % % % % %                             [in,on] = inpolygon(pixCur(1),pixCur(2),square(1:4,1),square(1:4,2));
        % % % % % %                         end
        
        % %                         if 1
        % %                             if in || on
        % %
        % %                                 validId = [validId; inlierId2(j)];
        % %                                 validXYZAll = [validXYZAll;j];
        % %
        % %                                 if inlierId(j) == 1222
        % %                                     Idd = j;
        % %                                 end
        % %                                 %                                                 plot(square2([1 4 5],1),square2([1 4 5],2),'-m'); plot(square2([2 3 5],1),square2([2 3 5],2),'-m');
        % %                                 if SHOWPOLYGON
        % %                                     plot(square([1 2],1),square([1 2],2),'-c'); plot(square([3 4],1),square([3 4],2),'-c');
        % %                                     plot(square([1 4],1),square([1 4],2),'-b'); plot(square([2 3],1),square([2 3],2),'-b');
        % %
        % %                                     plot([pixCur(1) retVal1(1)],[pixCur(2) retVal1(2)], '-xy');plot([pix0(1) retVal2(1)], [pix0(2) retVal2(2)], '-xy');plot([retVal1(1) retVal2(1)], [retVal1(2) retVal2(2)], '-xy');
        % %                                     plot([pixCur(1) retVal3(1)],[pixCur(2) retVal3(2)], '-xk');plot([pix0(1) retVal4(1)], [pix0(2) retVal4(2)], '-xk');plot([retVal3(1) retVal4(1)], [retVal3(2) retVal4(2)], '-xk');
        % %
        % %                                     plot(pixCur(1),pixCur(2),'.r');
        % %                                     plot(pix0(1),pix0(2),'.g');
        % %                                 end
        % %                                 sdvhk = 1;
        % %                             else
        % %                                 %                                                 plot(pixCur(1),pixCur(2),'.r');
        % %                                 %                                                 plot(square2([1 4 5],1),square2([1 4 5],2),'-m'); plot(square2([2 3 5],1),square2([2 3 5],2),'-m');
        % %                                 %                                                 plot(square([1 2],1),square([1 2],2),'-c'); plot(square([3 4],1),square([3 4],2),'-c');
        % %                                 %                                                 plot(square([1 4],1),square([1 4],2),'-b'); plot(square([2 3],1),square([2 3],2),'-b');
        % %                                 %                                                 plot(pix0(1),pix0(2),'.g');
        % %                                 sadb = 1;
        % %                             end
        % %                         else
        % %                             validId = [validId; inlierId2(j)];
        % %                         end
    end
end


b = max(permute(Probb,[3 1 2]));
b = mean(permute(Probb,[3 1 2]));
bb = permute(b,[2 3 1]);

bb(isinf(bb)) = 0;
bb(isnan(bb)) = 0;
thetaProb = sum(bb');

bbb = bb';

rati = Rati2;



rati11 = 0;


bbb1 = bbb./repmat(max(bbb')',1,size(bbb,2));

bbb2 = immultiply(bbb1, (bbb1 > rati11));
bbb2 = bbb2./repmat(sum(bbb2')',1,size(bbb2,2));

thetaRngMat = repmat(thetaRng,size(bbb2,1),1);
 bbbOpt = deg2rad(dot(rad2deg(immultiply(thetaRngMat',bbb1' > rati11)), bbb2'));   % 0.5 0.8 0.0


  

thetaProb = thetaProb./sum(thetaProb);
[~,idM_] = max(thetaProb);

idM = idM_;


% rati = Rati2; 0.95; 0.8;0.5;0.8; 0.5; 0.8; 0.9; 0;0.8;
thetaProb1 = thetaProb./max(thetaProb);
thetaProb2 = thetaProb1(thetaProb1 > rati);
thetaProb2 = thetaProb2./sum(thetaProb2);


if 0
    angOpt = thetaRng(idM);
else
    if FORCETHETAPLATFORM1
        idM = mean([1:length(thetaSamp)]);
    else
        idM = idM_;
    end
    %     angOpt = dot(rad2deg(thetaSamp), thetaProb);
    angOpt = deg2rad(dot(rad2deg(thetaRng(thetaProb1 > rati)), thetaProb2));   % 0.5 0.8 0.0
    thetaExp = deg2rad(dot(rad2deg(thetaSamp(thetaProb1 > rati)), thetaProb2));
end
% errEpiline = sum(err2)';

if ~fastVersion
    errEpiline = sum(err2)';
    
    [~,idM2] = min(errEpiline);
    angOptEpi = thetaRng(idM2);
else
    errEpiline = single(ones(length(thetaRng), 1));
    angOptEpi = angOpt;
    
end
if 0
    angOpt = mean([angOpt angOptEpi]);
    thetaExp = angOpt - mean(thetaRng);
end



k2cBodyTmp_opt = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(angOpt),zeros(3,1));
k2cCam_opt = b2c*k2cBodyTmp_opt.transformMat/b2c;
homocurrCcsXYZALL_opt = k2cCam_opt*HomoCoord(keyCcsXYZAll,1);
pixKey_opt = pflat(intrMat*homocurrCcsXYZALL_opt(1:3,:));
pixKeyTmp_opt = pixKey_opt(1:2,:)';


[meanErr, projErr, PtInKey, distInKey, PtInCur, distInCur, XCurInKey, fundMatKey2Cur, eplInCur] =  AngleSpace.ProjectErrorUnderTransform2(ConfigParam.reproj_sigma,  k2cCam_opt(1:3,1:3), k2cCam_opt(1:3,4), Pix, pt2dCur, intrMat,r_cam,tx, ty, tz);

projKey = pflat(intrMat*XCurInKey')';
[~, projKeyErr] = NormalizeVector(projKey(:,1:2) - Pix);

projKeyCur = pflat(intrMat*(k2cCam_opt(1:3,1:3)*XCurInKey' + repmat(k2cCam_opt(1:3,4),1,size(XCurInKey,1))))';
[~, projKeyCurErr] = NormalizeVector(projKeyCur(:,1:2) - PtInCur);



[princpPtL, princpPtR] = Get(obj.camModel, 'PinholePrincpPt', obj.scaleLvl);

DispRng = 1./ZZVec1;   %(intrMat(1,1).*norm(obj.camModel.transVec1To2)./(1./ZZVec1)) - (princpPtR(1) - princpPtL(1));
DispRngCenter = DispRng(:,(size(DispRng,2)+1)/2);
DispRngStep = mean(diff(DispRng'))';


dispInKey = (intrMat(1,1).*norm(obj.camModel.transVec1To2)./XCurInKey(:,3)) - (princpPtR(1) - princpPtL(1));
depthC2KInd = RoundingDispErr(DispRngCenter,dispInKey(:,1), DispRngStep,DispRng);
depthC2KCoord = [depthC2KInd (1:size(DispRng,1))'];
depthC2KInd_ind = sub2ind(size(DispRng), depthC2KCoord(:,2), depthC2KCoord(:,1));

if 0
    figure, [radi, ptOnLineErr1, ptOnLineErr2] = ShowEpilineCircle1(pt2dCur,Pix, fundMatKey2Cur, eplInCur, imgCur, [PtInCur PtInCur]);
end

end

function depthGTInd11 = RoundingDispErr(DispRngCenter,disp2round, DispRngStep,DispRng)
disparityError = DispRngCenter - disp2round;

disparityErrorRound = round(disparityError./DispRngStep).*DispRngStep;


depthGTOfst11 = round(-(disparityErrorRound)./DispRngStep);
depthGTInd11 = depthGTOfst11 + (size(DispRng,2)+1)/2;
depthGTInd11(depthGTInd11 < 1) = 1;
depthGTInd11(depthGTInd11 > size(DispRng,2)) = size(DispRng,2);
end