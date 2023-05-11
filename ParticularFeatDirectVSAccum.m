function err = ParticularFeatDirectVSAccum(sc, obj, LocalTrace, featId, refThetaList, angSampRng, curUpdateRatio, depthGTInd_update0, endFrameNum, useGTTracking, useGTZ)

global Rati2


theta_range_bak = obj.configParam.theta_range;
theta_sample_step_bak = obj.configParam.theta_sample_step;
theta_sample_step2_bak = obj.configParam.theta_sample_step2;
disparity_error_bak = obj.configParam.disparity_error;
disparity_sample_step_bak = obj.configParam.disparity_sample_step;

reproj_sigma_bak = obj.configParam.reproj_sigma;
reproj_sigma_right_bak = obj.configParam.reproj_sigma_right;

Rati2_bak = Rati2;

Rati2 = 0;

% % % % % obj.configParam.theta_range = deg2rad([-0.3 0.3]);
% % % % % obj.configParam.theta_sample_step = deg2rad(0.005);
% % % % % obj.configParam.theta_sample_step2 = deg2rad(0.005);
% % % % % obj.configParam.disparity_error = 0.3;
% % % % % obj.configParam.disparity_sample_step = 0.01;
% % % % % % 
% % obj.configParam.reproj_sigma = 0.5; 0.1;  1; 0.5;
% % obj.configParam.reproj_sigma_right = 0.5; 0.1; 1; 0.5;




pnp_ang_est_max_margin = [deg2rad([-1 1]) 2];
disparityRng = [-obj.configParam.disparity_error : obj.configParam.disparity_sample_step : obj.configParam.disparity_error];
thetaSamp = [obj.configParam.theta_range(1) : obj.configParam.theta_sample_step : obj.configParam.theta_range(2)];
thetaSamp0 = thetaSamp;
intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,obj.scaleLvl);
b2c = obj.coordSysAligner.pctBody2Cam(1, 1).transformMat;
T_B2C = b2c;


r_cam = sqrt(T_B2C(1,4)^2+T_B2C(2,4)^2+T_B2C(3,4)^2);
tx = T_B2C(1,4)/r_cam;
ty = T_B2C(2,4)/r_cam;
tz = T_B2C(3,4)/r_cam;

[princpPtL, princpPtR] = Get(obj.camModel, 'PinholePrincpPt', obj.scaleLvl);
L2R = [rodrigues(obj.camModel.rotVec1To2) obj.camModel.transVec1To2; 0 0 0 1];
baseline = norm(obj.camModel.transVec1To2);

localTrace = [LocalTrace.ptIcsX(unique(featId),:)' LocalTrace.ptIcsY(unique(featId),:)'];
if length(featId) > length(unique(featId))
    Pix = localTrace(repmat(1,length(featId),1),:);
else
    %     Pix = localTrace((featId),:);
    localTraceX = [LocalTrace.ptIcsX(unique(featId),:)];
    localTraceY = [ LocalTrace.ptIcsY(unique(featId),:)];
    Pix = [localTraceX(:,1) localTraceY(:,1)];
end
depthGTInd_update = depthGTInd_update0(featId,:);

% pt2dCur = repmat(localTrace(end,:),length(featId),1);

dispList = LocalTrace.dispList(featId,1);
% dispCurList = LocalTrace.dispList(featId,end);
depthList = LocalTrace.ptCcsZ(featId,1);

if useGTZ
    depthList = LocalTrace.ptCcsZGT(featId,1);
    dispList = LocalTrace.dispListGT(featId,1);
end

DispRng = dispList + disparityRng;

% ZZVec1 = LocalTrace.sampleZ;

ZZVec1 = [];ProbZ = [];
k2cList = [];
p2cList = [];
for jk = 1 : size(dispList,1)
    [ProbZ(jk,:), ZZVec1(jk,:)] = probDensity(dispList((jk),:), obj.configParam.disparity_sigma,obj.configParam.disparity_beta,obj.configParam.reproj_beta, DispRng(jk,:),obj.configParam.disparity_sample_interval, 'disparity');
end
ProbZ = ProbZ./repmat(max(ProbZ')',1,size(ProbZ,2));

ZTrue = depthList(:);
depthListGT = LocalTrace.ptCcsZGT(featId,1);

metricPrevPtCcs = intrMat\HomoCoord(Pix',1);
metricPrevPtCcs = normc(metricPrevPtCcs);
scaleAll = ZTrue./metricPrevPtCcs(3,:)';
%                                   scale0 = zTrue./metricPrevPtCcs(3,:)';
keyCcsXYZAll = [repmat(scaleAll',3,1).*metricPrevPtCcs];




dispListStereo = LocalTrace.dispList(featId,:);
dispListGT = LocalTrace.dispListGT(featId,:);

figure(98989),clf;subplot(2,2,1);plot(dispListStereo(1,:) - dispListGT(1,:));title('disparity trace error     stereo - gt')
drawnow;

% pt2dCurR = [pt2dCur(:,1) - dispCurList pt2dCur(:,2)];
probP2 = {};
startId = 1;
if isempty(endFrameNum)
    endFrameNum = size(localTrace,1)-1;
end
pixGT = []; skewnessVec = [];
if 0
    figure(611),clf;
end
figure(98989), subplot(2,2,4); cla; 

KminusPinK_mat = [];

for i = 1 : endFrameNum
    k2cRef = refThetaList(i+1) - refThetaList(1);
    pixGTPrv = pixGT;
    
    
    b2cPmat = GetPctBody2Cam(obj.coordSysAligner, 1);
    [pixGT] = VisualLocalizer.GetGtTrace2(b2cPmat, k2cRef, Pix, depthListGT(:),intrMat);
    pixGT(:,1) = pixGT(:,1) ; + 0.1;
    
    
    
%     figure(98989), subplot(2,2,4); hold on;plot(pt2dCur(:,1) - pixGT(:,1), pt2dCur(:,2) - pixGT(:,2),'+r'); 
    
    thetaRng0 = k2cRef + thetaSamp0;
    
    angSampNum = round(angSampRng./obj.configParam.theta_sample_step2);
    angSampNumMid = (length(thetaSamp0)+1)/2;
    angSampInd = (angSampNumMid - angSampNum) : (angSampNumMid + angSampNum);
    
    if length(featId) > length(unique(featId))
        pt2dCur = repmat(localTrace(i+1,:),length(featId),1);
    else
        %         pt2dCur = localTrace((featId),:);
        pt2dCur = [localTraceX(:,i+1) localTraceY(:,i+1)];
    end
    
    if useGTTracking
        pt2dCur = pixGT;
    end
    
    if useGTTracking
        dispCurList = LocalTrace.dispListGT(featId,i+1);
    else
        dispCurList = LocalTrace.dispList(featId,i+1);
    end
    pt2dCurR = [pt2dCur(:,1) - dispCurList pt2dCur(:,2)];
    
    [angOptK2C_pnp, ~, ~] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin,intrMat,Pix,pt2dCur,ZTrue, [1:size(ZTrue,1)]',true(size(ZTrue,1),1),b2c,k2cRef);
    
    if 0 % exist('updatedProbZSum', 'var')
        ProbZ = updatedProbZSum;
    end
    
    trackingErr = pt2dCur - pixGT;
    figure(98989), subplot(2,2,3); hold on;plot(trackingErr(1,1), trackingErr(1,2),'+');
    text(double(trackingErr(1,1)),double(trackingErr(1,2)),num2str(i), 'Color',[1 0 0],'FontSize',15);  %,'FontWeight','bold');
    
    
   
    
    
    
    [~,thetaProb0,idM,angOpt0, ReprojErrVecTmp, ReprojErrVecRTmp, ReprojErrVecTmpMatT, ReprojErrVecRTmpMatT,ValidFeatFlag,ValidFeatFlag0,ValidFeatFlagFusion,ValidFeatFlagQuant,ProbReprojVecAll,ProbReprojVecRAll, PixKeyVecTmp, PixKeyVecRTmp, Probb,validFeatFlagFusion,validFeatFlag, ProbReprojVec, ProbReprojVecR,bb,b,pixKeyVecTmpXCoordMatStack,pixKeyVecTmpYCoordMatStack,pixKeyVecRTmpXCoordMatStack,pixKeyVecRTmpYCoordMatStack, ...
        thetaExp_tmp0, errEpiline, angOptEpiInit, depthC2KInd_ind]       =   ReplayTheta(obj, thetaRng0, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, pt2dCur, pt2dCurR,ProbZ, ProbZ, zeros(240,320), zeros(240,320),thetaSamp0,r_cam,tx, ty, tz);
    
    
    [thetaRng, thetaSamp] = ArrangeThetaRng(obj,rad2deg(thetaExp_tmp0),k2cRef,sc,thetaSamp0);
    
    [~,thetaProb,idM,angOpt, ReprojErrVecTmp, ReprojErrVecRTmp, ReprojErrVecTmpMatT, ReprojErrVecRTmpMatT,ValidFeatFlag,ValidFeatFlag0,ValidFeatFlagFusion,ValidFeatFlagQuant,ProbReprojVecAllInit,ProbReprojVecRAll, PixKeyVecTmp, PixKeyVecRTmp, Probb,validFeatFlagFusion,validFeatFlag, ProbReprojVec, ProbReprojVecR,bb,b,pixKeyVecTmpXCoordMatStack,pixKeyVecTmpYCoordMatStack,pixKeyVecRTmpXCoordMatStack,pixKeyVecRTmpYCoordMatStack, ...
        thetaExp_tmp0, errEpiline, angOptEpiInit, depthC2KInd_ind]       =   ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, pt2dCur, pt2dCurR,ProbZ, ProbZ, zeros(240,320), zeros(240,320),thetaSamp,r_cam,tx, ty, tz);
    
    if 1
        
        thetaRng0 = thetaRng;
        thetaProb0 = thetaProb;
        angOpt0 = angOpt;
    end
    
    
    
    thetaProbOpt = interp1(thetaRng0, thetaProb0, angOpt0);
    
    id1 = find(thetaRng0 < angOpt0);
    id2 = find(thetaRng0 >= angOpt0);
    %     thetaRangeTemp = sort([thetaRng0 angOpt0]);
    %     idOpt = find(thetaRangeTemp == angOpt0);
    thetaRng0Temp = [thetaRng0(id1) angOpt0 thetaRng0(id2)];
    thetaProb0Temp = [thetaProb0(id1) thetaProbOpt thetaProb0(id2)];
    thetaProb0Temp = thetaProb0Temp./sum(thetaProb0Temp);
    idOpt = find(thetaRng0Temp == angOpt0);
    skewness = sum(thetaProb0Temp(idOpt:end)) / sum(thetaProb0Temp(1:idOpt));
    skewnessVec = [skewnessVec; skewness];
    
    
    if 0
        figure(611),clf;plot(rad2deg(thetaSamp0), [thetaProb0'./max(1)]);hold on;plot(rad2deg(angOpt0 - k2cRef),thetaProbOpt,'or');grid on;
    elseif 0
        thetaProb0Temp = thetaProb0Temp./max(thetaProb0Temp);
        figure(611);plot(rad2deg(thetaRng0Temp - k2cRef), thetaProb0Temp);hold on;plot(rad2deg(angOpt0 - k2cRef),thetaProb0Temp(idOpt),'or');grid on;
    else
        asdgfh = 1;
    end
    if 0
        figure(616);clf;plot(skewnessVec)
        drawnow;
    end
    % %     angOpt = angOpt0;
    
    k2cList = [k2cList; [angOpt angOptK2C_pnp]];
    probP2 = [probP2; {ProbReprojVecAllInit(angSampInd,:,:)}];
    
    
    
    probP2Mat = []; probP2Mat2 = []; probP2Mat3 = [];
    for vn = startId : length(probP2)
        probP2Mat = [probP2Mat; permute(probP2{vn, 1},[2 1 3])];
        probP2Mat2 = [probP2Mat2; probP2{vn, 1}];
        if vn == startId
            probP2Mat3 = probP2{vn, 1};
        else
            probP2Mat3 = (1 - curUpdateRatio).*probP2Mat3 + curUpdateRatio.*probP2{vn, 1};
        end
        
    end
    probP2Mat = permute(probP2Mat, [2 3 1]);
    
    
    if 0
        probP2Mat22 = permute(probP2Mat2, [1 3 2]);
        probP2Mat22Sum = sum(probP2Mat22);
        probP2Mat22Sum = permute(probP2Mat22Sum, [3 2 1]);
    else
        probP2Mat22 = permute(probP2Mat3, [1 3 2]);
        probP2Mat22Sum = sum(probP2Mat22);
        probP2Mat22Sum = permute(probP2Mat22Sum, [3 2 1]);
    end
    
    probP2Mat22Sum = probP2Mat22Sum./repmat(max(probP2Mat22Sum')',1,size(probP2Mat22Sum,2));
    
    probP2Mat22Sum_1 = probP2Mat22Sum./repmat(sum(probP2Mat22Sum')',1,size(probP2Mat22Sum,2));
    
    updatedProbZSum = probP2Mat22Sum;
    
    
    if 0
        ParticularFeatUpdate(startId, probP2, 1, depthGTInd_update,disparityRng, 1, curUpdateRatio, 1)
    end
    
    
    
    if i > 1
        %         pt2dPrv = localTrace(i-0,:);
        if length(featId) > length(unique(featId))
            pt2dPrv = localTrace(repmat(i,length(featId),1),:);
        else
            pt2dPrv = [localTraceX(:,i) localTraceY(:,i)];
        end
        dispListPrv = LocalTrace.dispList(featId,i);
        dispListPrv_gt = LocalTrace.dispListGT(featId,i);
        depthListPrv = LocalTrace.ptCcsZ(featId,i);
        DispRngPrv = dispListPrv + disparityRng;
        
        
        if useGTTracking
            pt2dPrv = pixGTPrv;
        end
        
        
        
        
        if useGTZ
            
            
            k2pBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(refThetaList(i) - refThetaList(1)),zeros(3,1));
            
            k2pCamPmat = b2cPmat*k2pBodyPmat/b2cPmat;
            k2pCam = k2pCamPmat.transformMat;
            homoPrevCcsXYZ = k2pCam*HomoCoord(keyCcsXYZAll,1);
            depthListPrv = homoPrevCcsXYZ(3,:)';
        end
        
        
        
        
        
        k2pB = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cList(end-1)),zeros(3,1));
        
        k2pCam_ = b2cPmat*k2pB/b2cPmat;
        k2pCam_ = k2pCam_.transformMat;
        
        
        [XYZPrv] = GetXYZFromDepth(intrMat, pt2dPrv,depthListPrv);
        
        p2kCam = inv(k2pCam_);
        
        XYZPrv2KeyRng = [p2kCam(1:3,1:3)*XYZPrv' + repmat(p2kCam(1:3,4),1,size(XYZPrv,1))]';
        dispPInK = (intrMat(1,1).*norm(obj.camModel.transVec1To2))./(XYZPrv2KeyRng(:,3) + (princpPtR(1) - princpPtL(1)));

        KminusPinK = dispList - dispPInK;
        
        KminusPinK_mat = [KminusPinK_mat; dispPInK(1)];
        
        ZTruePrv = depthListPrv;
        ZZVecPrv = [];ProbZPrv = [];
        for jk = 1 : size(dispList,1)
            [ProbZPrv(jk,:), ZZVecPrv(jk,:)] = probDensity(dispListPrv((jk),:), obj.configParam.disparity_sigma,obj.configParam.disparity_beta,obj.configParam.reproj_beta, DispRngPrv(jk,:),obj.configParam.disparity_sample_interval, 'disparity');
        end
        ProbZPrv = ProbZPrv./repmat(max(ProbZPrv')',1,size(ProbZPrv,2));
        [angOptP2C_pnp, ~, ~] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin,intrMat,pt2dPrv,pt2dCur,ZTruePrv, [1:size(ZTruePrv,1)]',true(size(ZTruePrv,1),1),b2c,refThetaList(i+1) - refThetaList(i));
        
        
        
        metricPrv = intrMat\HomoCoord(pt2dPrv',1);
        metricPrv = normc(metricPrv);
        scaleAllPrv = ZTruePrv./metricPrv(3,:)';
        %                                   scale0 = zTrue./metricPrevPtCcs(3,:)';
        prvCcsXYZAll = [repmat(scaleAllPrv',3,1).*metricPrv];
        
        
        
        thetaSampP2C = thetaSamp0;
        p2cRef = refThetaList(i+1) - refThetaList(i);
        thetaRngP2C = p2cRef + thetaSampP2C;
        
        [~,thetaProbP2C,idMP2C,angOptP2C, ~, ~, ~, ~,~,~,~,~,ProbReprojVecAllP2C, ProbReprojVecRAllP2C, ~, ~, ~,~,~, ~, ~,~,~,~,~,~,~,thetaExp_tmpP2C] = ReplayTheta(obj, thetaRngP2C, b2c, prvCcsXYZAll, intrMat, pt2dPrv, disparityRng, baseline, ZZVecPrv, L2R, pt2dCur, pt2dCurR, ProbZPrv, ProbZPrv, zeros(240,320), zeros(240,320),thetaSampP2C,r_cam,tx, ty, tz);
        
        p2cList = [p2cList; [angOptP2C angOptP2C_pnp]];
    else
        p2cList = [p2cList; [angOpt angOptK2C_pnp]];
    end
    
end

Rati2 = Rati2_bak;

obj.configParam.theta_range = theta_range_bak;
obj.configParam.theta_sample_step =  theta_sample_step_bak;
obj.configParam.theta_sample_step2 =  theta_sample_step2_bak;
obj.configParam.disparity_error = disparity_error_bak;
obj.configParam.disparity_sample_step =  disparity_sample_step_bak;

obj.configParam.reproj_sigma = reproj_sigma_bak;
obj.configParam.reproj_sigma_right = reproj_sigma_right_bak;

% figure(611),clf;plot(rad2deg(thetaSamp), [thetaProb'./max(thetaProb) thetaProb0'./max(thetaProb0)]);
% % figure(611),clf;plot(rad2deg(thetaSamp0), [thetaProb0'./max(thetaProb0)]);hold on;plot(rad2deg(angOpt0 - k2cRef),1,'or');grid on;



err = rad2deg(k2cList(:,1:2) - [refThetaList(2:size(k2cList,1)+1)]);

figure(98989), subplot(2,2,1);hold on;plot(KminusPinK_mat - dispListGT(1,1)); 
if 0
    figure(98989), subplot(2,2,2); cla; plot(rad2deg([k2cList(:,1) cumsum(p2cList(:,1))] - [refThetaList(2:size(k2cList,1)+1)]));title(num2str(depthGTInd_update(1)));legend('direct','accum')
    figure(98989), subplot(2,2,4); cla; plot(rad2deg([k2cList(:,2) cumsum(p2cList(:,2))] - [refThetaList(2:size(k2cList,1)+1)]));title(sprintf('orig pnp\nuseGTTracking: %d\nuseGTZ: %d', double(useGTTracking), double(useGTZ)));
else
    figure(98989), subplot(2,2,2); cla; plot(rad2deg([k2cList(:,1) ] - [refThetaList(2:size(k2cList,1)+1)]));title(num2str(depthGTInd_update(1)));legend('k2c')
    figure(98989), subplot(2,2,4); cla; plot(rad2deg([k2cList(:,2) ] - [refThetaList(2:size(k2cList,1)+1)]));title(sprintf('orig pnp\nuseGTTracking: %d\nuseGTZ: %d', double(useGTTracking), double(useGTZ)));
end

if 0
    figure(613),clf; subplot(1,2,1);plot(rad2deg(diff([0;k2cList(:,1)]) - diff(refThetaList(1:size(k2cList,1)+1))));title('diff(K2C) - gt, opt');subplot(1,2,2);plot(rad2deg(p2cList(:,1) - diff(refThetaList(1:size(k2cList,1)+1))));title('P2C - gt');
    figure(614),clf,subplot(1,2,1);plot(rad2deg(diff([0;k2cList(:,2)]) - diff(refThetaList(1:size(k2cList,1)+1))));title('diff(K2C) - gt, pnp');subplot(1,2,2);plot(rad2deg(p2cList(:,2) - diff(refThetaList(1:size(k2cList,1)+1))));title('P2C - gt');
end

end
function [XYZ] = GetXYZFromDepth(intrMat, Pix,depthList)
metricPrevPtCcsGT = intrMat\HomoCoord(Pix',1);
metricPrevPtCcsGT = normc(metricPrevPtCcsGT);
if 1
    scaleAllGT = depthList./metricPrevPtCcsGT(3,:)';
else
    dispGTComp = dispList(inlierId) - disparityErrorRound;
    depthListGTComp = intrMat(1,1).*norm(obj.camModel.transVec1To2)./(dispGTComp + (princpPtR(1) - princpPtL(1)));
    scaleAllGT = depthListGTComp./metricPrevPtCcsGT(3,:)';
end
XYZ = [repmat(scaleAllGT',3,1).*metricPrevPtCcsGT]';




end