function [thetaProb,idM,angOpt, ReprojErrVecTmp, ReprojErrVecRTmp, ReprojErrVecTmpMatT, ReprojErrVecRTmpMatT,ValidFeatFlag,ValidFeatFlag0,ValidFeatFlagFusion,ValidFeatFlagQuant,ProbReprojVecAll,ProbReprojVecRAll, PixKeyVecTmp, PixKeyVecRTmp, Probb,validFeatFlagFusion,validFeatFlag, ProbReprojVec, ProbReprojVecR,bb,b,pixKeyVecTmpXCoordMatStack,pixKeyVecTmpYCoordMatStack,pixKeyVecRTmpXCoordMatStack,pixKeyVecRTmpYCoordMatStack] = ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, pt2dCur, pt2dCurR,ProbZ, ProbZTmp_norm, imgCur, imgCurR,thetaSamp)
global SHOWPOLYGON DRAWPOLYGON ANGLEONLY USEGOLDENDISP UPDATEDEPTH DEPTHITERNUM probPath ...
    USELEFT USELEFTPLUSRIGHT USELEFTXRIGHT USERIGHT WITHINEPILINE BOTHLR HARDCUT QUANTCUT FUSIONCUT ...
    FORCEEVENPLATFORM CUTTHETAANDFORCEPLATFORM DIFFTHETATHR CUTTHETAANDFORCEPLATFORM_NODIFF FORCEEXPEVEN SOFTP2 Rati Sc USEthetaProb0 ...
    Rati2 FORCETHETAPLATFORM1

ReprojErrVecTmp = {}; ReprojErrVecRTmp = {}; ReprojErrVecTmpMatT = {}; ReprojErrVecRTmpMatT = {};
ValidFeatFlag = []; ValidFeatFlag0 = []; ValidFeatFlagFusion = [];ValidFeatFlagQuant = [];
for jj = 1 : length(thetaRng)
    k2cBodyTmp = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(thetaRng(jj)),zeros(3,1));
    k2cCam = b2c*k2cBodyTmp.transformMat/b2c;
    homocurrCcsXYZALL = k2cCam*HomoCoord(keyCcsXYZAll,1);
    pixKey = pflat(intrMat*homocurrCcsXYZALL(1:3,:));
    pixKeyTmp = pixKey(1:2,:)';
    % % % % % % % % % % % % %                         [~, reprojErrTmp] = NormalizeVector(pixKeyTmp - pt2dCur);
    % % % % % % % % % % % % %                         [probReproj, ~] = probDensity(0, obj.configParam.disparity_sigma, thetaRng,obj.configParam.theta_sample_step, 'theta');
    
    % %                         DispRng = repmat(dispList(inlierId,:),1,length(disparityRng)) + repmat(disparityRng,length(inlierId),1);
    % %                         for jk = 1 : length(inlierId)
    % %                              [ProbZ(jk,:), ZZVec1(jk,:)] = probDensity(dispList(inlierId(jk),:), obj.configParam.disparity_sigma, DispRng(jk,:),obj.configParam.disparity_sample_interval, 'disparity');
    % %                         end
    
    
    metricPrevPtCcsKey = intrMat\HomoCoord(Pix',1);
    metricPrevPtCcsKey = normc(metricPrevPtCcsKey);
    metricPrevPtCcsKey = repmat(metricPrevPtCcsKey, 1, length(disparityRng));
    zListTmp = intrMat(1)*baseline*ZZVec1(:)';
    Scale00 = zListTmp./metricPrevPtCcsKey(3,:);
    KeyCcsXYZVec = [ Scale00.*metricPrevPtCcsKey];
    
    homocurrCcsXYZALLVec = k2cCam*HomoCoord(KeyCcsXYZVec,1); homocurrCcsXYZALLVecR = L2R*homocurrCcsXYZALLVec;
    pixKeyVec = pflat(intrMat*homocurrCcsXYZALLVec(1:3,:)); pixKeyVecR = pflat(intrMat*homocurrCcsXYZALLVecR(1:3,:));
    pixKeyVecTmp = pixKeyVec(1:2,:)'; pixKeyVecRTmp = pixKeyVecR(1:2,:)';
    PixKeyVecTmp{jj,1} = pixKeyVecTmp; PixKeyVecRTmp{jj,1} = pixKeyVecRTmp;
    % %                            figure,imshow(imgCur);hold on;plot(pixKeyVecTmp(:,1), pixKeyVecTmp(:,2),'.g');plot(pt2dCur(:,1), pt2dCur(:,2),'.r');
    %                            figure,imshow(imgCurR);hold on;plot(pixKeyVecRTmp(:,1), pixKeyVecRTmp(:,2),'.g');plot(pt2dCurR(:,1), pt2dCurR(:,2),'.r');
    %%
    if 1
        [errVec, reprojErrVecTmp] = NormalizeVector(pixKeyVecTmp - repmat(pt2dCur,length(disparityRng) ,1)); [errVecR, reprojErrVecRTmp] = NormalizeVector(pixKeyVecRTmp - repmat(pt2dCurR,length(disparityRng) ,1));
    else
        [errVec, reprojErrVecTmp] = NormalizeVector(pixKeyVecTmp(:,1) - repmat(pt2dCur(:,1),length(disparityRng) ,1)); [errVecR, reprojErrVecRTmp] = NormalizeVector(pixKeyVecRTmp(:,1) - repmat(pt2dCurR(:,1),length(disparityRng) ,1));
    end
    
    
    
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
    
    epiLine_margin = obj.configParam.epiLine_margin;
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
            flagL = interpLXProbZ > obj.configParam.probZ_ratio_threshold;
            flagR = interpRXProbZ > obj.configParam.probZ_ratio_threshold;
        else
            flagL = interpLXProbZ; % > obj.configParam.probZ_ratio_threshold;
            flagR = interpRXProbZ; % > obj.configParam.probZ_ratio_threshold;
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
        [probReprojVec22, ~] = probDensity(0, obj.configParam.reproj_sigma,obj.configParam.disparity_beta,obj.configParam.reproj_beta, reprojErrVecTmpMatT(:)',obj.configParam.reproj_sample_interval, 'reproj');
        [probReprojVecR22, ~] = probDensity(0, obj.configParam.reproj_sigma_right,obj.configParam.disparity_beta,obj.configParam.reproj_beta_right, reprojErrVecRTmpMatT(:)',obj.configParam.reproj_sample_interval, 'reproj');
    else
        [probReprojVec22, ~] = probDensity(0, min(5,max(obj.configParam.reproj_sigma, size(obj.featPtManager.localTrace.ptIcsX, 2)*0.5)),obj.configParam.disparity_beta,obj.configParam.reproj_beta, reprojErrVecTmpMatT(:)',obj.configParam.reproj_sample_interval, 'reproj');
        [probReprojVecR22, ~] = probDensity(0, min(5,max(obj.configParam.reproj_sigma_right,size(obj.featPtManager.localTrace.ptIcsX, 2)*0.5)),obj.configParam.disparity_beta,obj.configParam.reproj_beta_right, reprojErrVecRTmpMatT(:)',obj.configParam.reproj_sample_interval, 'reproj');
    end
    probReprojVec = reshape(probReprojVec22, size(reprojErrVecTmpMat,2),size(reprojErrVecTmpMat,1))'; probReprojVecR = reshape(probReprojVecR22, size(reprojErrVecRTmpMat,2),size(reprojErrVecRTmpMat,1))';
    ProbReprojVec{jj,1} = probReprojVec;  ProbReprojVecR{jj,1} = probReprojVecR;
    ProbReprojVecAll(jj,:,:) = probReprojVec; ProbReprojVecRAll(jj,:,:) = probReprojVecR;
    %                            probReprojVec = [];
    %                            for jk = 1 : length(inlierId)
    %                                %                              [ProbZ(jk,:), ZZVec1(jk,:)] = probDensity(dispList(inlierId(jk),:), obj.configParam.disparity_sigma, DispRng(jk,:),obj.configParam.disparity_sample_interval, 'disparity');
    %                                [probReprojVec(jk,:), ~] = probDensity(0, obj.configParam.reproj_sigma, reprojErrVecTmpMat(jk,:),obj.configParam.reproj_sample_interval, 'reproj');
    %                                %
    %                            end
    
    
    validFeatFlagFusionMat = ((ProbZ.*(probReprojVec.*probReprojVecR)));
    %                                 validFeatFlagFusionMat = ((ProbZ.*(probReprojVec)));
    if BOTHLR
        inFusionFlag = abs(reprojErrVecTmpMatT) < obj.configParam.reproj_sigma & abs(reprojErrVecRTmpMatT) < obj.configParam.reproj_sigma_right;
    else
        inFusionFlag = abs(reprojErrVecTmpMatT) < obj.configParam.reproj_sigma | abs(reprojErrVecRTmpMatT) < obj.configParam.reproj_sigma_right;
    end
    inFusionFlag = abs(reprojErrVecTmpMatT) < obj.configParam.reproj_sigma;
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
    
    
    randId = 10;
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
        pixCur = ([obj.featPtManager.localTrace.ptIcsX(inlierId(j),keyLength) obj.featPtManager.localTrace.ptIcsY(inlierId(j),keyLength)]);
        %                                         plot(pixCur(1),pixCur(2),'.r');
        pix = ([obj.featPtManager.localTrace.ptIcsX(inlierId(j),1) obj.featPtManager.localTrace.ptIcsY(inlierId(j),1)]);
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
                zRng = [intrMat(1)*baseline/(dispTrue + obj.configParam.disparity_error) intrMat(1)*baseline/(max(0.1, dispTrue - obj.configParam.disparity_error))];
                dispRng = dispTrue + disparityRng;
            else
                zRng = obj.configParam.depth_uncertainty_range + zTrue;
            end
        catch
            if 1
                zRng = [intrMat(1)*baseline/(dispTrue + 3.5) intrMat(1)*baseline/(dispTrue - 3.5)];
            else
                zRng = [-200 300] + zTrue;
            end
        end
        [probZ, ZVec1] = probDensity(dispTrue, obj.configParam.disparity_sigma,obj.configParam.disparity_beta,obj.configParam.reproj_beta, dispRng,obj.configParam.disparity_sample_interval, 'disparity');
        
        
        
        % % %                             metricPrevPtCcs = intrMat\HomoCoord(pix',1);
        % % %                             metricPrevPtCcs = normc(metricPrevPtCcs);
        % % %                             scale1 = zRng./metricPrevPtCcs(3,:)';
        % % %                             scale0 = zTrue./metricPrevPtCcs(3,:)';
        % % %                             keyCcsXYZ = [scale1(1).*metricPrevPtCcs scale1(2).*metricPrevPtCcs scale0.*metricPrevPtCcs];
        % % %                             %                         try
        % % %                             %                             thetaRng = obj.configParam.theta_range + theta;
        % % %                             %                             thetaRng = obj.configParam.theta_range + theta;
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
        %                                 [probReproj(kk,1), ~] = probDensity(0, obj.configParam.reproj_sigma, reprojTmp(kk),obj.configParam.reproj_sample_interval, 'theta');
        %                             end
        [probReproj, ~] = probDensity(0, obj.configParam.reproj_sigma, reprojTmp',obj.configParam.reproj_sample_interval, 'reproj');
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
        
        
        
        
        % % %                         margLen = obj.configParam.polygon_margin;
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
        % % % % % %                                 if (sign(num1) == sign(num2) && sign(num3) == sign(num4))  % || err5 < obj.configParam.polygon_inlier_thresh
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
thetaProb = thetaProb./sum(thetaProb);
[~,idM_] = max(thetaProb);
thetaProb1 = thetaProb./max(thetaProb);
idM = idM_;
rati = Rati2; 0.95; 0.8;0.5;0.8; 0.5; 0.8; 0.9; 0;0.8;
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
    
end

end