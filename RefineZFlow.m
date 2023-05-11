validTrace = true;
if 0
    FrmNumWithGloden = FrmNumWithGloden0;
else
    FrmNumWithGloden = FrmNumWithGloden1;
end
prepareRefineZ;


ang_thr = 11111; 0.1;  0.061;
pix_thr = 0.1;
scaleAng = min(keyFrameLen, 4); % 6 3

ProbZNormSum11 = LocalTrace.probZ./(repmat(sum(LocalTrace.probZ')', 1, size(LocalTrace.probZ, 2)));
DispExp_ = sum(LocalTrace.DispRng'.*ProbZNormSum11')';

% [thetaPnP_checkK2C, xOfst_k2cRef,angStep, minInd] = PureRotationAngPredictor.pnpLite_1([],intrMat,[],obj.coordSysAligner,k2cRef,[],[],[],[LocalTrace.ptIcsX(:,1) LocalTrace.ptIcsY(:,1)] ,[LocalTrace.ptIcsX(:,end) LocalTrace.ptIcsY(:,end)], DispExp_, CamModel, obj.scaleLvl );


ind_key = GetLastKeyFrameInd(obj);
keyPoseWcs_key = obj.poseWcsList(ind_key,:);%vsl.configParam.right_feature_point_trace_param
robotPoseWcs_temp = [keyPoseWcs_key(1:2), keyPoseWcs_key(3) + k2cBodyRotAng];
if 0
    accumP2CPNP_temp = [obj.accumP2CPNP; obj.accumP2CPNP(end) + robotPoseWcs_temp(3) - obj.poseWcsList(end,3)];
else
    accumP2CPNP_temp = [obj.accumP2CPNP2]; %obj.accumP2CPNP(end) + robotPoseWcs_temp(3) - obj.poseWcsList(end,3)];
end
k2cRef_pnp = accumP2CPNP_temp(end) - accumP2CPNP_temp(length(accumP2CPNP_temp) - keyFrameLen + 0 + 1);

[thetaPnP_checkK2C, xOfst_k2cRef,angStep, minInd] = PureRotationAngPredictor.pnpLite_1([],intrMat,[],obj.coordSysAligner,k2cRef_pnp,[],[],[],[LocalTrace.ptIcsX(:,1) LocalTrace.ptIcsY(:,1)] ,[LocalTrace.ptIcsX(:,end) LocalTrace.ptIcsY(:,end)], DispExp_, CamModel, obj.scaleLvl );

dltAngK2C1 = thetaPnP_checkK2C - k2cRef_pnp;

dltAngK2C_deg = rad2deg(dltAngK2C1)
dltAngK2C = sign(dltAngK2C1)*deg2rad(0.01);
if length(obj.accumP2CRef) > FrmNumWithGloden0 + 1
    
    if keyFrameLen <= 80  && keyFrameLen > FrmNumWithGloden1 + 1%10
        if k2cRef > 0
            if dltAngK2C_deg > ang_thr
                k2cRef = k2cRef_pnp - scaleAng*dltAngK2C;
                FrmNumWithGloden = 1999;
            end
            if dltAngK2C_deg < -ang_thr
                k2cRef = k2cRef_pnp - scaleAng*dltAngK2C;
                FrmNumWithGloden = 1999;
            end
            
        else
            
            if dltAngK2C_deg > ang_thr
                k2cRef = k2cRef_pnp - scaleAng*dltAngK2C;
                FrmNumWithGloden = 1999;
            end
            if dltAngK2C_deg < -ang_thr
                k2cRef = k2cRef_pnp - scaleAng*dltAngK2C;
                FrmNumWithGloden = 1999;
            end
            
            
        end
    end
end

if traceIndex == 1
    pt2d_check0 = pt2d_check;
else
    pt2d_check = pt2d_check0;
end

if single(k2cRef_1_00) == single(k2cBodySensorRotAng)
    
    checkTrackingErr = true;
else
    checkTrackingErr = false;
    pt2d_check = [];
end




if validTrace
    % if size(LocalTraceList{1, 1}.ptIcsX,2) > 2 % ~isempty(DepthId)
    %
    %     DepthId = obj.keyProbZ{end,2};
    %     commonId = find(ismember(DepthId, LocalTrace.featId));
    %     LocalTrace.probZ = obj.keyProbZ{end,28}(commonId,:);
    %     DepthProbility = obj.keyProbZ{end,24};
    %
    %     DispRefineUse__ = obj.keyProbZ{end,25};
    %     NewDispRng = obj.keyProbZ{end,26};
    %     newDispStep = obj.keyProbZ{end,27};
    %     ProbZTmpReSamp = obj.keyProbZ{end,28};
    %
    %     DepthProbility = ProbZTmpReSamp;
    % end
    
    
    
    
    DirectAccumDiffThr = 0.025;
    if k2cRef > 0
        overcompensate = 0.0175;  -0.0125; -0.01; 0; 0.01; 0.0175;
        overcompensate = 0.0275; % big scene
        overcompensate = 0.05; % big scene
    else
        overcompensate = -0.0125;
    end
    
    overcompensateFrameNumThr = 111111111111;  11;  111111;  11;  11000000;
    skipFramNum = 4;
    minNum = 4;
    
    
    cfg.thetaOfst = [cfg.thetaOfst; 0];
    if keyFrameLen - 1 >= overcompensateFrameNumThr & (keyFrameLen - 1 <= overcompensateFrameNumThr +100)
        if  ~isempty(twoOnlyP2List)
            DirectAccumDiff = min(99990.5, twoOnlyP2List(end, end));
            if k2cRef > 0
                if 0
                    if DirectAccumDiff < -DirectAccumDiffThr
                        cfg.thetaOfst(end) = (abs(DirectAccumDiff) + overcompensate);
                        cfg.compensateFrmId = [cfg.compensateFrmId; keyFrameLen-1];
                    end
                    if DirectAccumDiff > DirectAccumDiffThr
                        cfg.thetaOfst(end) = (-abs(DirectAccumDiff) - overcompensate);
                        cfg.compensateFrmId = [cfg.compensateFrmId; keyFrameLen-1];
                    end
                else
                    dispOfstSign_ = sign(cfg.dispOfst);
                    dispOfstSign = dispOfstSign_(skipFramNum:end);
                    
                    if sum(dispOfstSign) > minNum
                        cfg.thetaOfst(end) = (-abs(DirectAccumDiff) - overcompensate);
                        cfg.compensateFrmId = [cfg.compensateFrmId; keyFrameLen-1];
                    elseif sum(dispOfstSign) < -minNum
                        cfg.thetaOfst(end) = (abs(DirectAccumDiff) + overcompensate);
                        cfg.compensateFrmId = [cfg.compensateFrmId; keyFrameLen-1];
                    end
                    
                end
            else
                if 0
                    if DirectAccumDiff < -DirectAccumDiffThr
                        cfg.thetaOfst(end) = (abs(DirectAccumDiff) + overcompensate);
                        cfg.compensateFrmId = [cfg.compensateFrmId; keyFrameLen-1];
                    end
                    if DirectAccumDiff > DirectAccumDiffThr
                        cfg.thetaOfst(end) = (-abs(DirectAccumDiff) - overcompensate);
                        cfg.compensateFrmId = [cfg.compensateFrmId; keyFrameLen-1];
                    end
                else
                    dispOfstSign_ = sign(cfg.dispOfst);
                    dispOfstSign = dispOfstSign_(skipFramNum:end);
                    
                    if sum(dispOfstSign) > minNum
                        cfg.thetaOfst(end) = (abs(DirectAccumDiff) + overcompensate);
                        cfg.compensateFrmId = [cfg.compensateFrmId; keyFrameLen-1];
                    elseif sum(dispOfstSign) < -minNum
                        cfg.thetaOfst(end) = (-abs(DirectAccumDiff) - overcompensate);
                        cfg.compensateFrmId = [cfg.compensateFrmId; keyFrameLen-1];
                    end
                    
                    
                end
            end
        else
            cfg.thetaOfst(end) = 0;
        end
    else
        cfg.thetaOfst(end) = 0;
    end
    sdfjhk = 1;
    
    LocalTrace_before = LocalTrace;
    
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
    
    
    
if 1    
    obj.traceManager.X(LocalTrace.featIdOut,end) = -1;
    obj.traceManager.Y(LocalTrace.featIdOut,end) = -1;
    obj.traceManager.Z(LocalTrace.featIdOut,end) = -1;
    obj.traceManager.ZGT(LocalTrace.featIdOut,end) = -1;
    
end




    dispCompare = [dispCompare; {cell2mat((LocalTrace.transDisp(:,3)))'}];
    dispCompare2 = [dispCompare2; LocalTrace.transDisp(end,6:8)];
     
    obj.judgement{traceBatch,1} = traceBatch;
    obj.judgement{traceBatch,2} = LocalTrace.transDisp;
    
    gt_angle = [obj.accumP2CRef(end-(keyFrameLen-1):end) - obj.accumP2CRef(end-(keyFrameLen-1))];
    
    if 0
        obj.dispErrExpStack3{traceBatch, 1} = traceBatch;
        obj.dispErrExpStack3{traceBatch, 2} = length(obj.accumP2CRef);
        obj.dispErrExpStack3{traceBatch, 3} = LocalTrace.transDisp(:,6:8);
        obj.dispErrExpStack3{traceBatch, 4} = size(LocalTrace.ptIcsX,2);
        obj.dispErrExpStack3{traceBatch, 5} = [[ANGOpt; angOpt1] [ANGRng; angRng1] gt_angle(2:end)];
        obj.dispErrExpStack3{traceBatch, 6} = LocalTrace.transDisp{1,4}(3,:);
    end
    
    if 0
%         EvalJudgement(obj, 50, [1000 5000]);
        EvalJudgement2(obj, 50, [1000 5000]);
    end
    
    
    dltIdAll = dltIdAll + dltId;
    afterRefineZ;
    
    
%     minOptFrameLen = 5;
    
    TraceBatchListStackBefore;
    
% % %     if ~isempty(TraceBatchListStackBefore)
% % %        if ~ismember(traceBatch, TraceBatchListStackBefore) & keyFrameLen > 2
% % %            
% % %            
% % %        end
% % %         
% % %         
% % %     end
    
    
    
    if size(obj.dispErrExpStack3,1) < traceBatch
        
        
        obj.dispErrExpStack3{traceBatch, 1} = traceBatch;
        obj.dispErrExpStack3{traceBatch, 2} = length(obj.accumP2CRef);
        obj.dispErrExpStack3{traceBatch, 3} = LocalTrace.transDisp(:,6:8);
        obj.dispErrExpStack3{traceBatch, 4} = size(LocalTrace.ptIcsX,2);
        obj.dispErrExpStack3{traceBatch, 5} = [[ANGOpt; angOpt1] [ANGRng; angRng1] gt_angle(2:end)];
        obj.dispErrExpStack3{traceBatch, 6} = LocalTrace.transDisp{1,4}(3,:);
        obj.dispErrExpStack3{traceBatch, 7} = LocalTraceDone;
        obj.dispErrExpStack3{traceBatch, 8} = 0;  % 1 is out
        obj.dispErrExpStack3{traceBatch, 9} = 0;  % 1 is replayed
        
    else
        obj.dispErrExpStack3{traceBatch, 1} = traceBatch;
        obj.dispErrExpStack3{traceBatch, 2} = length(obj.accumP2CRef);
        obj.dispErrExpStack3{traceBatch, 3} = LocalTrace.transDisp(:,6:8);
        obj.dispErrExpStack3{traceBatch, 4} = size(LocalTrace.ptIcsX,2);
        obj.dispErrExpStack3{traceBatch, 5} = [[ANGOpt; angOpt1] [ANGRng; angRng1] gt_angle(2:end)];
        obj.dispErrExpStack3{traceBatch, 6} = LocalTrace.transDisp{1,4}(3,:);
        obj.dispErrExpStack3{traceBatch, 7} = LocalTraceDone;
        
        
%         batches_all = cell2mat(obj.dispErrExpStack3(:,1));
        
        
        
        
        
        obj.dispErrExpStack3{traceBatch, 8} = 0;  % 1 is out
        obj.dispErrExpStack3{traceBatch, 9} = 0;  % 1 is replayed
    end
        
        
    
    
    if zx == length(LocalTraceList)
        GtAng_bak = {};
        
        for uij = 1 : size(obj.dispErrExpStack3,1)
            
            tempDisps3 = obj.dispErrExpStack3{uij, 3};
            if ~isempty(tempDisps3)
                tempLocalTrace3 = obj.dispErrExpStack3{uij, 7};
                frameRng3 = [obj.dispErrExpStack3{uij, 2} - obj.dispErrExpStack3{uij, 4} + 1 : obj.dispErrExpStack3{uij, 2} - obj.dispErrExpStack3{uij, 4} + 0 + size(tempDisps3,1)];
                gtAngList3 = obj.accumP2CRef(frameRng3);
                gtAngList3 = gtAngList3 - gtAngList3(1);
                gtAng_bak = obj.dispErrExpStack3{uij, 5}(:,4);
                GtAng_bak = [GtAng_bak; {gtAng_bak}];
                if 0
                    figure,plot(rad2deg(gtAng_bak - gtAngList3(2:end)));
                end
                obj.dispErrExpStack3{uij, 5}(:,4) =  gtAngList3(2:end);
            end
        end
    end
    
    if 0
        FrmNumWithGloden = FrmNumWithGloden0;
    else
        FrmNumWithGloden = FrmNumWithGloden1;
    end
end