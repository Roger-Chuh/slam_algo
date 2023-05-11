function [ANGOptFinal, cfg] = ReplayZOpt(obj, LocalTraceDone_replay, localTrace_gtAng, traceBatch, useGoldenZ, angOpt2, cfg_new)

global FrmNumWithGloden0 FrmNumWithGloden1 Switch4to3 FrmNumWithGloden FigBase probPath Rati2 GTTRACKING ReplaceNoKey




theta_range_bak = obj.configParam.theta_range;
theta_sample_step_bak = obj.configParam.theta_sample_step;
theta_sample_step2_bak = obj.configParam.theta_sample_step2;
disparity_error_bak = obj.configParam.disparity_error;
disparity_sample_step_bak = obj.configParam.disparity_sample_step;

reproj_sigma_bak = obj.configParam.reproj_sigma;
reproj_sigma_right_bak = obj.configParam.reproj_sigma_right;
reproj_sigma0_bak = obj.configParam.reproj_sigma0;
reproj_sigma_update_z_bak = obj.configParam.reproj_sigma_update_z;
reproj_sigma_update_z_right_bak = obj.configParam.reproj_sigma_update_z_right;


disparity_sigma_bak = obj.configParam.disparity_sigma;
disparity_beta_bak = obj.configParam.disparity_beta;



% % % obj.configParam.theta_range = deg2rad([-0.4 0.4]);
% % % obj.configParam.theta_sample_step = deg2rad(0.005);
% % % obj.configParam.theta_sample_step2 = deg2rad(0.005);
% % % obj.configParam.disparity_error = 0.4;
% % % obj.configParam.disparity_sample_step = 0.05;
% % % 
% % % 
if 1
    obj.configParam.reproj_sigma = 0.25; 0.1;  0.2;         0.1; 0.2; 0.5; 0.1;  1; 0.5;
    % % % % obj.configParam.reproj_sigma_right = 0.1; 0.1; 1; 0.5;
    obj.configParam.reproj_beta = 2;         1; 2;
    obj.configParam.reproj_sigma0 = 0.25;  0.1;  0.2;         0.1;0.2;0.5; 1;
    obj.configParam.reproj_sigma_update_z =  0.25; 0.1;  0.2;         0.1; 0.2; 0.5; 1;
end
% % % % % %  
obj.configParam.reproj_sigma_update_z = 1;
obj.configParam.reproj_sigma_update_z_right = 1;

 


obj.configParam.disparity_sigma = 0.001; 15;
obj.configParam.disparity_beta = 2;


 
singlePt =  false; true;
chosenId = 5.*[1;1;1;1];

forceGTTracking =  false; true;


FigBase_bak = FigBase;

Rati2_bak = Rati2;
GTTRACKING_bak = GTTRACKING;


[princpPtL, princpPtR] = Get(obj.camModel, 'PinholePrincpPt', obj.scaleLvl);
L2R = [rodrigues(obj.camModel.rotVec1To2) obj.camModel.transVec1To2; 0 0 0 1];

intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,obj.scaleLvl);

b2cPmat = GetPctBody2Cam(obj.coordSysAligner, 1);

disparityRng = [-obj.configParam.disparity_error : obj.configParam.disparity_sample_step : obj.configParam.disparity_error];

LocalTrace1 = LocalTraceDone_replay.LocalTrace;
LocalTrace10 = LocalTrace1;

CamModel = obj.camModel;
CoordSysAligner = obj.coordSysAligner;
ConfigParam = obj.configParam;

raplaceRng = [1 1200];
curUpdateTransDispRatio = 0.002;  1; 0.2; 1; 0.2; 1; 0.2;

keyDispWeighted = [];

if 0
    for iop = 1 : size(LocalTrace.transDisp,1)
        if iop == 1
            keyDispWeighted = LocalTrace.transDisp{iop,3};
        else
            keyDispWeighted = (1 - curUpdateTransDispRatio).*keyDispWeighted + curUpdateTransDispRatio.*LocalTrace.transDisp{iop,3};
        end
    end
elseif 0
    if size(LocalTrace.transDisp,1) < raplaceRng(1)
        keyDispWeighted = LocalTrace.dispList(:,1)';
        
    else
        for iop = raplaceRng(1) : min(raplaceRng(2), size(LocalTrace.transDisp,1))
            if iop == raplaceRng(1)
                keyDispWeighted = LocalTrace.transDisp{iop,3};
            else
                keyDispWeighted = (1 - curUpdateTransDispRatio).*keyDispWeighted + curUpdateTransDispRatio.*LocalTrace.transDisp{iop,3};
            end
        end
        
    end
else
    keyDispWeighted_all = [];
    for iop = raplaceRng(1) : min(raplaceRng(2), size(LocalTraceDone_replay.LocalTrace.transDisp,1)) - 1
        
        keyDispWeighted_all = [keyDispWeighted_all;  LocalTraceDone_replay.LocalTrace.transDisp{iop,3}];
    end
%         if (size(LocalTraceDone_replay.LocalTrace.transDisp,1)) == 1
    if size(keyDispWeighted_all,1) == 1
        
        keyDispWeighted = keyDispWeighted_all;
    else
        
        keyDispWeighted = mean(keyDispWeighted_all);
        if ReplaceNoKey
            keyDispWeighted = keyDispWeighted_all(1,:);
        end
        % %                     keyDispWeighted = (keyDispWeighted_all(end,:));
    end
    
end


if useGoldenZ
    keyDispWeighted = LocalTraceDone_replay.LocalTrace.dispListGT(:,1)';
end



keyDepthWeighted = (intrMat(1,1).*norm(obj.camModel.transVec1To2))./(keyDispWeighted + (princpPtR(1) - princpPtL(1)));

LocalTrace1.ptCcsZ(:,1) = keyDepthWeighted';

LocalTrace1.dispList(:,1) = keyDispWeighted';
LocalTrace1.DispRng = keyDispWeighted' + disparityRng;

ZZVec1___ = [];ProbZ___ = [];
for jk11 = 1 : length(LocalTrace1.featId)
    [ProbZ___(jk11,:), ZZVec1___(jk11,:)] = probDensity(keyDispWeighted(1, (jk11)), obj.configParam.disparity_sigma,obj.configParam.disparity_beta,obj.configParam.reproj_beta, LocalTrace1.DispRng(jk11,:),obj.configParam.disparity_sample_interval, 'disparity');
end

ProbZ___ = ProbZ___./repmat(max(ProbZ___')',1,size(ProbZ___,2));

LocalTrace1.sampleZ = ZZVec1___;


LocalTraceDone_replay.LocalTrace.transDisp;

if singlePt
    ProbZ___1 = ProbZ___(chosenId,:);
    LocalTrace1.probZ = LocalTrace1.probZ(chosenId,:);
    LocalTrace1.DispRng = LocalTrace1.DispRng(chosenId,:);
    LocalTrace1.sampleZ = LocalTrace1.sampleZ(chosenId,:);
else
    ProbZ___1 = ProbZ___;
end



% % if ~isempty(angOpt2)
% %     cfg.newAngOpt = angOpt2;
% % end



for i = 1 : size(LocalTrace1.ptIcsX,2) - 1
    if ~singlePt
        LocalTrace.ptIcsX = LocalTrace1.ptIcsX(:,1:1+i);
        LocalTrace.ptIcsY = LocalTrace1.ptIcsY(:,1:1+i);
        LocalTrace.ptCcsZ = LocalTrace1.ptCcsZ(:,1:1+i);
        LocalTrace.ptCcsZGT = LocalTrace1.ptCcsZGT(:,1:1+i);
        LocalTrace.dispList = LocalTrace1.dispList(:,1:1+i);
        LocalTrace.dispListGT = LocalTrace1.dispListGT(:,1:1+i);
        LocalTrace.featId = LocalTrace1.featId;
    else
        LocalTrace.ptIcsX = LocalTrace1.ptIcsX(chosenId,1:1+i);
        LocalTrace.ptIcsY = LocalTrace1.ptIcsY(chosenId,1:1+i);
        LocalTrace.ptCcsZ = LocalTrace1.ptCcsZ(chosenId,1:1+i);
        LocalTrace.ptCcsZGT = LocalTrace1.ptCcsZGT(chosenId,1:1+i);
        LocalTrace.dispList = LocalTrace1.dispList(chosenId,1:1+i);
        LocalTrace.dispListGT = LocalTrace1.dispListGT(chosenId,1:1+i);
        LocalTrace.featId = LocalTrace1.featId(chosenId,:);
        
    end
    
    
    
    
    k2cRef_1_00 = localTrace_gtAng(i);
    k2cRef = LocalTraceDone_replay.angOpt(i);
    keyFrameLen = i+1;
    pt2d_check = [];
    k2cBodyRotAng = k2cRef;
    inlierId = [1:size(LocalTrace.ptIcsX,1)]';
    inlierId2 = [1:size(LocalTrace.ptIcsX,1)]';
    theta = k2cRef;
    
    if i == 1
        if 0
            LocalTrace.probZ = ones(size(LocalTrace1.probZ));
        else
            LocalTrace.probZ = ProbZ___1;  % LocalTrace1.probZ;
        end
        LocalTrace.DispRng = LocalTrace1.DispRng;
        LocalTrace.sampleZ = LocalTrace1.sampleZ;
        LocalTrace.probP2 = {};
        LocalTrace.transDisp = {};
        LocalTrace.cfg.compensateFrmId = [];
        LocalTrace.cfg.thetaOfst = [];
        LocalTrace.cfg.dispOfst = [];
        
        
        
        disparity_resample_prob_threshold = obj.configParam.disparity_resample_prob_threshold;
        ThetaNoise = obj.thetaNoise;
        KeyFrameFlagList = obj.keyFrameFlagList;
        ConfigParam = obj.configParam;
        
        if 0
            imgCur = obj.currImgL;
            imgCurR = obj.currImgR;
            imgKeyL = obj.keyFrameImgL;
            imgKeyR = obj.keyFrameImgR;
            imgPrvL = obj.prevImgL;
            imgPrvR = obj.prevImgR;
        else
            imgCur = LocalTraceDone_replay.currImgL(:,:,1);
            imgCurR = LocalTraceDone_replay.currImgR(:,:,1);
            imgKeyL = LocalTraceDone_replay.keyFrameImgL(:,:,1);
            imgKeyR = LocalTraceDone_replay.keyFrameImgR(:,:,1);
            imgPrvL = LocalTraceDone_replay.prevImgL(:,:,1);
            imgPrvR = LocalTraceDone_replay.prevImgR(:,:,1);
        end
        
        if 0
            LocalTrace_0 = obj.featPtManager.localTrace;
        else
            %         LocalTrace = LocalTraceList{traceIndex};
            %     keyFrameLen =  size(LocalTrace.ptIcsX, 2);
            prevFeatPtList0 = [LocalTrace.ptIcsX(:,1) LocalTrace.ptIcsY(:,1)];
            ptCcsZ0 = LocalTrace.ptCcsZ(:,1);
            
        end
        if 1
            KeyProbZ = {};  % obj.tempProb{traceIndex,2};   %obj.keyProbZ;
        end
        
        
% %         cfg.compensateFrmId = [];
% %         cfg.thetaOfst = [];
% %         cfg.dispOfst = [];
        
        thetaRngMat = [];   %obj.thetaRngMat;
        thetaProbMat = [];   %obj.thetaProbMat;
        CamModel = obj.camModel;
        CoordSysAligner = obj.coordSysAligner;
        REFAngList = [];   %obj.refAngList;
        REFAngList3 = [];   %obj.refAngList3;
        REFAngList4 = [];   %obj.refAngList4;
        accumP2CTemp = [];   %obj.accumP2CTemp;
        twoOnlyP2List = [];   %obj.twoOnlyP2List;
        thetaPlatform = [];   %obj.thetaPlatform;
        thetaPlatformDistribution = [];   %obj.thetaPlatformDistribution;
        poseWcsList = obj.poseWcsList;
        ANGOpt = [];    %obj.angOpt;
        ANGRng = [];   %obj.angRng;
        meanErrAndStd = [];  %obj.meanErrAndStd;
        p2cErrList = []; %  obj.p2cErrList;
        objPervPtCcsZGolden = [];    %obj.PervPtCcsZGolden;
        DepthId = []; %KeyProbZ{end,2};
        %         commonId = find(ismember(DepthId, LocalTrace.featId));
        %         LocalTrace.probZ = KeyProbZ{end,28}(commonId,:);
        DepthProbility = [];%KeyProbZ{end,24};
        
        DispRefineUse__ = [];%KeyProbZ{end,25};
        NewDispRng = [];%KeyProbZ{end,26};
        newDispStep = [];%KeyProbZ{end,27};
        ProbZTmpReSamp = []; %KeyProbZ{end,28};
        
        DepthProbility = ProbZTmpReSamp;
        
        
        %         k2cRef_1_00 = obj.accumP2CRef(end) - obj.accumP2CRef(length(obj.accumP2CRef) - keyFrameLen + 0 + 1);  %obj.accumP2CRef(end) - obj.accumP2CRef(1);
        
        AngleModalOrg = obj.angleModalOrg;
        AngleProbility = [];
        cfg = LocalTrace.cfg;
        
        cfg.compensateFrmId = [];
        cfg.thetaOfst = [];
        cfg.dispOfst = [];
        
        cfg.a = 1;
        if ~isempty(angOpt2)
            cfg.newAngOpt = angOpt2;
        end
        
        
        
    else
        DepthId = KeyProbZ{end,2};
        LocalTrace.probZ = KeyProbZ{end,28};
        DepthProbility = KeyProbZ{end,24};
        
        DispRefineUse__ = KeyProbZ{end,25};
        NewDispRng = KeyProbZ{end,26};
        newDispStep = KeyProbZ{end,27};
        ProbZTmpReSamp = KeyProbZ{end,28};
        
        DepthProbility = ProbZTmpReSamp;
        prevFeatPtList0 = [LocalTrace.ptIcsX(:,1) LocalTrace.ptIcsY(:,1)];
        ptCcsZ0 = LocalTrace.ptCcsZ(:,1);
        objPervPtCcsZGolden = objPervPtCcsZGolden1;
        
    end
     cfg.thetaOfst = [cfg.thetaOfst; 0];
    
     if i == size(LocalTrace1.ptIcsX,2) - 1
         cfg.LocalTrace10 = LocalTrace10;
         if 0
             figure,plot(keyDispWeighted_all(1,:) - LocalTrace10.dispList(:,1)')
         end
     end
     FigBase = 100000;
     

     Rati2 = 0;
     
     if forceGTTracking
         GTTRACKING = true;
     end
     [angOpt1, inlierIdNew,  angRng1, pixKey ,pt2dCur, DispRefine, angOpt33, inFlag,thetaRngOut,thetaProbOut,angOptP2C, angOptK2C_, p2cOnlyP2,...
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
        objPervPtCcsZGolden1, cfg] = RefineZ(cfg, pt2d_check, traceBatch, k2cRef_1_00, obj, k2cRef,k2cBodyRotAng, inlierId,inlierId2, theta, DepthProbility,  DepthId,  AngleModalOrg,  AngleProbility, prevFeatPtList0,ptCcsZ0, DispRefineUse__,NewDispRng,newDispStep,ProbZTmpReSamp, k2cBodyRotAng, keyFrameLen , disparity_resample_prob_threshold, ThetaNoise ,KeyFrameFlagList, ConfigParam,  imgCur , imgCurR,  imgKeyL,  imgKeyR,  imgPrvL,  imgPrvR,  LocalTrace,  KeyProbZ,  thetaRngMat, thetaProbMat,   CamModel ,  CoordSysAligner,   REFAngList ,  REFAngList3,  REFAngList4,   accumP2CTemp,  twoOnlyP2List,   thetaPlatform,  thetaPlatformDistribution,   poseWcsList,    ANGOpt,    ANGRng,  meanErrAndStd,   p2cErrList,  objPervPtCcsZGolden);
    
    FigBase = FigBase_bak;
    Rati2 = Rati2_bak;
    GTTRACKING = GTTRACKING_bak;
    
    
    ANGOpt = [ANGOpt; [angOpt1 ]];
    
    
    if Switch4to3
        REFAngList4 = [REFAngList3];
    else
        REFAngList4 = [REFAngList4; angOpt1 ];
    end
    
    
    ANGRng = [ANGRng; angRng1 ];
    
    thetaRngMat = [thetaRngMat; thetaRngOut ];
    
    thetaProbMat = [thetaProbMat; thetaProbOut ];
    
    
    
end

unValid = [];
for ij = 1 : length(cfg.KKKK_unValid)
    unValid = [unValid; cfg.KKKK_unValid{ij}];
end

Valid = setdiff([1:length(LocalTrace.featId)]', unique(unValid));


BB = [];
for ij = 1 : length(cfg.KKKK)
    BB = [BB; cfg.KKKK{ij}(Valid)];
end

% BB = (cell2mat(cfg.KKKK));
AA = diff(BB);
[maxAA,idAA] = max(AA);

cfg.AA = AA;
cfg.Valid = Valid;

obj.configParam.theta_range = theta_range_bak;
obj.configParam.theta_sample_step =  theta_sample_step_bak;
obj.configParam.theta_sample_step2 =  theta_sample_step2_bak;
obj.configParam.disparity_error = disparity_error_bak;
obj.configParam.disparity_sample_step =  disparity_sample_step_bak;

obj.configParam.reproj_sigma = reproj_sigma_bak;
obj.configParam.reproj_sigma_right = reproj_sigma_right_bak;
obj.configParam.reproj_sigma0 = reproj_sigma0_bak;
obj.configParam.reproj_sigma_update_z = reproj_sigma_update_z_bak;
obj.configParam.reproj_sigma_update_z_right = reproj_sigma_update_z_right_bak;
 



obj.configParam.disparity_sigma = disparity_sigma_bak;
obj.configParam.disparity_beta = disparity_beta_bak;


ANGOptFinal = ANGOpt;

angErr = [0;ANGOptFinal] - [0;localTrace_gtAng];

[old_distri, old_Distri_sum] = CalcDispErrDistribution(LocalTraceDone_replay.LocalTrace.transDisp{1,6}', disparityRng);
[new_distri, new_Distri_sum] = CalcDispErrDistribution(LocalTrace.dispList(:,1) - LocalTrace.dispListGT(:,1), disparityRng);

exp_old = dot(disparityRng, old_Distri_sum./sum(old_Distri_sum));
exp_new = dot(disparityRng, new_Distri_sum./sum(new_Distri_sum));

prob_old = interp1(disparityRng, old_Distri_sum, exp_old);
prob_new = interp1(disparityRng, new_Distri_sum, exp_new);
if isempty(angOpt2)
    figure(4),clf;subplot(3,2,[1 2]);plot(disparityRng, [old_Distri_sum; new_Distri_sum]');hold on;plot(exp_old, prob_old,'ob');plot(exp_new, prob_new,'or');
    title(sprintf('traceBatch: %d\nold exp: %0.3f  new exp: %0.3f',traceBatch, exp_old, exp_new));legend('old','new');grid on;
    subplot(3,2,[3 4]);plot(rad2deg([(LocalTraceDone_replay.angOpt - localTrace_gtAng)     (ANGOptFinal - localTrace_gtAng)]));legend('old - gt theta','new - gt theta');
    if 0
        subplot(3,1,3);plot(diff(rad2deg([(LocalTraceDone_replay.angOpt - localTrace_gtAng)     (ANGOptFinal - localTrace_gtAng)])));legend('old','new');title('p2c err');
    else
        if isempty(angOpt2)
            subplot(3,2,5);plot(rad2deg([(LocalTraceDone_replay.angOpt - localTrace_gtAng)   -  (ANGOptFinal - localTrace_gtAng)]));legend('(old - gt theta) - (new - gt theta)');
            subplot(3,2,6);plot(abs(diff(rad2deg([(LocalTraceDone_replay.angOpt - localTrace_gtAng)     (ANGOptFinal - localTrace_gtAng)]))));legend('diff old - gt', 'diff new - gt');
        else
            subplot(3,2,6);plot(abs(diff(rad2deg([(LocalTraceDone_replay.angOpt - ANGOptFinal)     (angOpt2 - ANGOptFinal)]))));legend('diff old - gt', 'diff new - gt');
        end    
    end   
else
      
    figure(4);subplot(3,2,5);cla;plot(rad2deg([(LocalTraceDone_replay.angOpt - ANGOptFinal)     (angOpt2 - ANGOptFinal)]));legend('old - gt disp theta', 'new - gt disp theta');
    
    subplot(3,2,[3]);cla; plot(AA);title('slope');
    subplot(3,2,[4]);cla; plot(rad2deg([(LocalTraceDone_replay.angOpt - localTrace_gtAng)  (angOpt2 - localTrace_gtAng)   (ANGOptFinal - localTrace_gtAng)]));legend('old - gt theta','new - gt theta','gtZ - gt theta');
    
    try
        cfg_gt = cfg;
        AA1 = cell2mat(cfg_new.bbbOpt);
        BB1 = cell2mat(cfg_gt.bbbOpt);
        AA1 = [zeros(1,size(AA1,2)); AA1];
        BB1 = [zeros(1,size(BB1,2)); BB1];
        CC1 = abs(rad2deg(diff(AA1) - diff(BB1)));
        [maxCC, indCC] = max(CC1);
        figure(4),subplot(3,2,5);cla;plot(CC1);title('angle curve');
    catch
        sdafgoj = 1;
    end
    
    figure(4);subplot(3,2,6);cla;
    if 0
        plot(abs(diff(rad2deg([([0;LocalTraceDone_replay.angOpt] - [0;ANGOptFinal])     ([0;angOpt2] - [0;ANGOptFinal])]))));legend('diff old - gt disp theta', 'diff new - gt disp theta');
    else
        plot(LocalTrace.ptCcsZ');title('depth');
    end
end
try
    saveas(gcf,fullfile(probPath,sprintf(strcat('thetaComp_',probPath(end-14:end),'____',sprintf('%04d__%06d',sum(traceBatch), length(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('thetaComp_',probPath(end-14:end),'____',sprintf('%04d',(sum(traceBatch))),'__*.png'))))+1 + FigBase)));
catch
    askjhaz = 1;
end

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

function [ProbZTmp_update_norm_sum_tmp_c2c, ProbZTmp_update_norm_sum_tmp_c2c_sum] = CalcDispErrDistribution(dispCurList_gt, disparityRng)
DispRngC2C = zeros(length(dispCurList_gt),1) + disparityRng;

DispRngStepC2C = mean(diff(DispRngC2C'))';
[~, depthGTIndAll_cur] = RoundingDispErr2(DispRngC2C(:,(size(DispRngC2C,2)+1)/2),-dispCurList_gt, DispRngStepC2C,DispRngC2C);
try
    depthGTIndAll_cur(isnan(depthGTIndAll_cur)) = 1;
catch
    asdfgk = 1;
end
ProbZTmp_update_norm_sum_tmp_c2c = zeros(size(DispRngC2C));
%                     ProbZTmp_update_norm_sum_tmp(depthGTInd11) = ProbZTmp_update_norm_sum(depthGTInd11);
ProbZTmp_update_norm_sum_tmp_c2c(depthGTIndAll_cur) = 1;
ProbZTmp_update_norm_sum_tmp_c2c_sum = sum(ProbZTmp_update_norm_sum_tmp_c2c);
end
