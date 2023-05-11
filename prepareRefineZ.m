
% traceIndex = 1;

freeFrame = find(traceLenList > FrmNumWithGloden1+1);

ForceGoldenFrame = false;


commonId = [];
LocalTrace = LocalTraceList{traceIndex};
keyFrameLen =  size(LocalTrace.ptIcsX, 2);
dltId = 0;
if keyFrameLen > 2
    commonId = []; dltId = -1;
    while isempty(commonId)
        dltId = dltId + 1;
        KeyProbZ = obj.tempProb{traceIndex + dltId,2};   %obj.keyProbZ;
        %    if size(LocalTraceList{1, 1}.ptIcsX,2) > 2 % ~isempty(DepthId)
        
        DepthId = KeyProbZ{end,2};
        commonId = find(ismember(DepthId, LocalTrace.featId));
        
    end
    if dltId > 0
        lossLocalTraceDone = obj.tempProb{traceIndex,1};
        oldAngOpt = lossLocalTraceDone.angOpt;
        
    end
    % if dltId > 0
    for te = 1 : dltId
        if 0
            obj.tempProb{te,1} = {};
            obj.tempProb{te,2} = {};
        end
    end
end
if ~exist('commonId','var')
    commonId = [];
end

if ~isempty(commonId) || keyFrameLen == 2
    
    
    if keyFrameLen == 2
        % keyFrameLen = size(obj.featPtManager.localTrace.ptIcsX, 2);
        disparity_resample_prob_threshold = obj.configParam.disparity_resample_prob_threshold;
        ThetaNoise = obj.thetaNoise;
        KeyFrameFlagList = obj.keyFrameFlagList;
        ConfigParam = obj.configParam;
        
        imgCur = rgb2gray(obj.currImgL);
        imgCurR = rgb2gray(obj.currImgR);
        imgKeyL = rgb2gray(obj.keyFrameImgL);
        imgKeyR = rgb2gray(obj.keyFrameImgR);
        imgPrvL = rgb2gray(obj.prevImgL);
        imgPrvR = rgb2gray(obj.prevImgR);
        
        if 0
            LocalTrace_0 = obj.featPtManager.localTrace;
        else
            %         LocalTrace = LocalTraceList{traceIndex};
            %     keyFrameLen =  size(LocalTrace.ptIcsX, 2);
            prevFeatPtList0 = [LocalTrace.ptIcsX(:,1) LocalTrace.ptIcsY(:,1)];
            ptCcsZ0 = LocalTrace.ptCcsZ(:,1);
            
        end
        if 1
            KeyProbZ = {}; obj.tempProb{traceIndex,2};   %obj.keyProbZ;
        end
        
        
        cfg.compensateFrmId = [];
        cfg.thetaOfst = [];
        cfg.dispOfst = [];
        
        thetaRngMat = [];obj.thetaRngMat;
        thetaProbMat = [];obj.thetaProbMat;
        CamModel = obj.camModel;
        CoordSysAligner = obj.coordSysAligner;
        REFAngList = [];obj.refAngList;
        REFAngList3 = [];obj.refAngList3;
        REFAngList4 = [];obj.refAngList4;
        accumP2CTemp = [];obj.accumP2CTemp;
        twoOnlyP2List = [];obj.twoOnlyP2List;
        thetaPlatform = [];obj.thetaPlatform;
        thetaPlatformDistribution = [];obj.thetaPlatformDistribution;
        poseWcsList = obj.poseWcsList;
        ANGOpt = [];obj.angOpt;
        ANGRng = [];obj.angRng;
        meanErrAndStd = [];obj.meanErrAndStd;
        p2cErrList = [];obj.p2cErrList;
        objPervPtCcsZGolden = [];obj.PervPtCcsZGolden;
        
        
        
        DepthId = []; %KeyProbZ{end,2};
        %         commonId = find(ismember(DepthId, LocalTrace.featId));
        %         LocalTrace.probZ = KeyProbZ{end,28}(commonId,:);
        DepthProbility = [];%KeyProbZ{end,24};
        
        DispRefineUse__ = [];%KeyProbZ{end,25};
        NewDispRng = [];%KeyProbZ{end,26};
        newDispStep = [];%KeyProbZ{end,27};
        ProbZTmpReSamp = []; %KeyProbZ{end,28};
        
        DepthProbility = ProbZTmpReSamp;
        
        traceIndex;
        if 0
            if traceIndex > 1
                if 1
                    olderAngList = obj.tempProb{1,1}.angOpt;
                    k2cRef = olderAngList(end) - olderAngList(traceIndex-1);
                else
                    olderAngList = obj.tempProb{traceIndex - 1,1}.angOpt;
                    %             k2cRef = olderAngList(end) - olderAngList(traceIndex - 1-1);
                    k2cRef = olderAngList(end) - olderAngList(1);
                end
                
            else
                k2cRef = obj.accumP2CRef(end) - obj.accumP2CRef(length(obj.accumP2CRef) - keyFrameLen + dltId + 1);
            end
            
            % k2cRef = obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs;
            
        else
            %             k2cRef = obj.accumP2CRef(end) - obj.accumP2CRef(length(obj.accumP2CRef) - keyFrameLen + dltId + 1);
            
            if length(obj.accumP2CRef) <= FrmNumWithGloden0 + 1
                k2cRef = obj.accumP2CRef(end) - obj.accumP2CRef(length(obj.accumP2CRef) - keyFrameLen + 0 + 1);
                FrmNumWithGloden = FrmNumWithGloden0;
            else
                
                if traceIndex == 1
                     k2cRef = obj.accumP2CRef(end) - obj.accumP2CRef(length(obj.accumP2CRef) - keyFrameLen + 0 + 1);
                     FrmNumWithGloden = -1000;
                 else
                     keyFrameLen;
%                      angOptListLast = tempProbNew{traceIndex-1, 1}.angOpt; %(end) - tempProbNew{traceIndex-1, 1}.angOpt(end - (length(tempProbNew{traceIndex-1, 1}.angOpt) - keyFrameLen));
                     if ForceGoldenFrame
                         FrmNumWithGloden = 1000;
                     else
                         FrmNumWithGloden = FrmNumWithGloden1;
                     end
                     tempRefAng = [];
                     for jn = 1 : size(angOptListLast,1)
                         tempRefAng = [tempRefAng;angOptListLast(jn,end) - angOptListLast(jn,size(angOptListLast,2) - keyFrameLen + 1)];
                     end
                     traceLenListTemp = traceLenList(1 : traceIndex-1);
                     TraceWeight = traceLenListTemp./sum(traceLenListTemp);
                     
                     if 0
                         k2cRef = mean(tempRefAng);
                     elseif 0
                         k2cRef = dot(TraceWeight, tempRefAng);
                     elseif 0
% %                          k2cRef = tempRefAng(1);
                         if ~isempty(freeFrame)
                             tempRefAng_ = tempRefAng(1:min([length(tempRefAng) freeFrame(end)]));
                             k2cRef = mean(tempRefAng_);
                         else
% %                              k2cRef = mean(tempRefAng(1));
                             k2cRef = mean(tempRefAng);
                         end
                     elseif 0
                         k2cRef = tempRefAng(end);
                     else
                         k2cRef = tempRefAng(1);
                     end
                     
                     
%                      k2cRef = angOptListLast(end) - angOptListLast((length(angOptListLast) - keyFrameLen ) +1);
                     fdjdsut = 1;
                 end
                
                fdjdsut = 1;
            end
        end
        
        % k2cRef_1_00 = obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs;
        
        
%         k2cRef_1_00 = obj.accumP2CRef(end) - obj.accumP2CRef(length(obj.accumP2CRef) - keyFrameLen + dltId + 1);  %obj.accumP2CRef(end) - obj.accumP2CRef(1);
        k2cRef_1_00 = obj.accumP2CRef(end) - obj.accumP2CRef(length(obj.accumP2CRef) - keyFrameLen + 0 + 1);  %obj.accumP2CRef(end) - obj.accumP2CRef(1);
        
        AngleModalOrg = obj.angleModalOrg;
        AngleProbility = [];
    else
        
        KeyProbZLastFrm = obj.tempProb{traceIndex + dltId,1};
        LocalTraceDone = obj.tempProb{traceIndex + dltId,1};
        KeyProbZ = obj.tempProb{traceIndex + dltId,2};   %obj.keyProbZ;
        
        
        
        cfg = LocalTraceDone.LocalTrace.cfg;
        %    if size(LocalTraceList{1, 1}.ptIcsX,2) > 2 % ~isempty(DepthId)
        if 1
            DepthId = KeyProbZ{end,2};
            commonId = find(ismember(DepthId, LocalTrace.featId));
            
            if isempty(commonId)
                
                
            end
            
            
            LocalTrace.probZ = KeyProbZ{end,28}(commonId,:);
            %             LocalTrace.probP2 = LocalTraceDone.LocalTrace.probP2;    % [LocalTrace.probP2 ;KeyProbZ{end,29}(:,commonId,:)];
            for hu = 1 : size(LocalTraceDone.LocalTrace.probP2,1) + 1
                %                 commonId_temp = find(ismember(KeyProbZ{hu,2}, LocalTrace.featId));
                commonId_temp = commonId;
                if hu <= size(LocalTraceDone.LocalTrace.probP2,1)
                    LocalTrace.probP2{hu,1} =  LocalTraceDone.LocalTrace.probP2{hu,1}(:, commonId_temp, :);
                    LocalTrace.probP2{hu,2} =  LocalTraceDone.LocalTrace.probP2{hu,2}(:, commonId_temp, :);
                end
                if 1
                    for jyu = 1 : size(LocalTraceDone.LocalTrace.transDisp,2)
                        LocalTrace.transDisp{hu,jyu} =  LocalTraceDone.LocalTrace.transDisp{hu,jyu}(:, commonId_temp);
                    end
                else
                    LocalTrace.transDisp{hu,1} =  LocalTraceDone.LocalTrace.transDisp{hu,1}(:, commonId_temp);
                    LocalTrace.transDisp{hu,2} =  LocalTraceDone.LocalTrace.transDisp{hu,2}(:, commonId_temp);
                    LocalTrace.transDisp{hu,3} =  LocalTraceDone.LocalTrace.transDisp{hu,3}(:, commonId_temp);
                    LocalTrace.transDisp{hu,4} =  LocalTraceDone.LocalTrace.transDisp{hu,4}(:, commonId_temp);
                    LocalTrace.transDisp{hu,5} =  LocalTraceDone.LocalTrace.transDisp{hu,5}(:, commonId_temp);
                end
            end
            LocalTrace.probP2 = [LocalTrace.probP2; {}];
            LocalTrace.transDisp = [LocalTrace.transDisp; {}];
            
            
            
            try
                errCkeckProbZ = LocalTrace.probZ - KeyProbZLastFrm.LocalTrace.probZ(commonId,:);
            catch
                asbvkj = 1;
            end
            
            
            
            DepthProbility = KeyProbZ{end,24};
            
            DispRefineUse__ = KeyProbZ{end,25};
            NewDispRng = KeyProbZ{end,26};
            newDispStep = KeyProbZ{end,27};
            ProbZTmpReSamp = KeyProbZ{end,28};
            
            DepthProbility = ProbZTmpReSamp;
        end
        
        disparity_resample_prob_threshold = LocalTraceDone.configParam.disparity_resample_prob_threshold;
        ThetaNoise = obj.thetaNoise;
        KeyFrameFlagList = LocalTraceDone.keyFrameFlagList;
        ConfigParam = obj.configParam;
        
        imgCur = rgb2gray(obj.currImgL);    %LocalTraceDone.currImgL;
        imgCurR = rgb2gray(obj.currImgR);    %LocalTraceDone.currImgR;
        imgKeyL = rgb2gray(obj.keyFrameImgL);    %LocalTraceDone.keyFrameImgL;
        imgKeyR = rgb2gray(obj.keyFrameImgR);   %LocalTraceDone.keyFrameImgR;
        imgPrvL = rgb2gray(obj.prevImgL);   %LocalTraceDone.prevImgL;
        imgPrvR = rgb2gray(obj.prevImgR);   %LocalTraceDone.prevImgR;
        
        if 0
            LocalTrace_0 = LocalTraceDoneBefore.featPtManager.localTrace;
        else
            %         LocalTrace = LocalTraceList{traceIndex};
            %     keyFrameLen =  size(LocalTrace.ptIcsX, 2);
            prevFeatPtList0 = [LocalTrace.ptIcsX(:,1) LocalTrace.ptIcsY(:,1)];
            ptCcsZ0 = LocalTrace.ptCcsZ(:,1);
            
        end
        %     KeyProbZ = obj.tempProb{traceIndex,2};   %obj.keyProbZ;
        thetaRngMat = LocalTraceDone.thetaRngMat;
        thetaProbMat = LocalTraceDone.thetaProbMat;
        CamModel = obj.camModel;
        CoordSysAligner = obj.coordSysAligner;
        REFAngList = LocalTraceDone.refAngList;
        REFAngList3 = LocalTraceDone.refAngList3;
        REFAngList4 = LocalTraceDone.refAngList4;
        accumP2CTemp = LocalTraceDone.accumP2CTemp;
        twoOnlyP2List = LocalTraceDone.twoOnlyP2List;
        thetaPlatform = LocalTraceDone.thetaPlatform;
        thetaPlatformDistribution = LocalTraceDone.thetaPlatformDistribution;
        if 0
            poseWcsList = LocalTraceDone.poseWcsList;
        else
            poseWcsList = obj.poseWcsList;
        end
        ANGOpt = LocalTraceDone.angOpt;
        ANGRng = LocalTraceDone.angRng;
        meanErrAndStd = LocalTraceDone.meanErrAndStd;
        p2cErrList = LocalTraceDone.p2cErrList;
        objPervPtCcsZGolden = LocalTraceDone.PervPtCcsZGolden;
        
        traceIndex;
        if 0
            if traceIndex > 1
                % %         olderAngList = obj.tempProb{traceIndex-1,1}.angOpt;
                % %         k2cRef = olderAngList(end) - olderAngList(traceIndex-1);
                if 1
                    olderAngList = obj.tempProb{1,1}.angOpt;
                    k2cRef = olderAngList(end) - olderAngList(traceIndex-1);
                else
                    olderAngList = obj.tempProb{traceIndex - 1,1}.angOpt;
                    %             k2cRef = olderAngList(end) - olderAngList(traceIndex - 1-1);
                    k2cRef = olderAngList(end) - olderAngList(1);
                end
                
                
            else
                k2cRef = obj.accumP2CRef(end) - obj.accumP2CRef(length(obj.accumP2CRef) - keyFrameLen + 1);
            end
            % k2cRef = obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs;
            
            
        else
%             k2cRef = obj.accumP2CRef(end) - obj.accumP2CRef(length(obj.accumP2CRef) - keyFrameLen + dltId +  1);
%             k2cRef = obj.accumP2CRef(end) - obj.accumP2CRef(length(obj.accumP2CRef) - keyFrameLen + 0 +  1);
            
             if length(obj.accumP2CRef) <= FrmNumWithGloden0 + 1
                k2cRef = obj.accumP2CRef(end) - obj.accumP2CRef(length(obj.accumP2CRef) - keyFrameLen + 0 + 1);
                FrmNumWithGloden = FrmNumWithGloden0;
             else
                
                 if traceIndex == 1
                     k2cRef = obj.accumP2CRef(end) - obj.accumP2CRef(length(obj.accumP2CRef) - keyFrameLen + 0 + 1);
                     FrmNumWithGloden = -1000;
                 else
                     keyFrameLen;
                     %                      angOptListLast = tempProbNew{traceIndex-1, 1}.angOpt; %(end) - tempProbNew{traceIndex-1, 1}.angOpt(end - (length(tempProbNew{traceIndex-1, 1}.angOpt) - keyFrameLen));
                     
                     
                     if ForceGoldenFrame
                         FrmNumWithGloden = 1000;
                     else
                         FrmNumWithGloden = FrmNumWithGloden1;
                     end
                     tempRefAng = [];
                     for jn = 1 : size(angOptListLast,1)
                         tempRefAng = [tempRefAng;angOptListLast(jn,end) - angOptListLast(jn,size(angOptListLast,2) - keyFrameLen + 1)];
                     end
                      traceLenListTemp = traceLenList(1 : traceIndex-1);
                     TraceWeight = traceLenListTemp./sum(traceLenListTemp);
                     
                     if 0
                         k2cRef = mean(tempRefAng);
                     elseif 0
                         k2cRef = dot(TraceWeight, tempRefAng);
                     elseif 0
                         
                         if ~isempty(freeFrame)
                             tempRefAng_ = tempRefAng(1:min([length(tempRefAng) freeFrame(end)]));
                             k2cRef = mean(tempRefAng_);
                         else
% %                              k2cRef = mean(tempRefAng(1));
                             k2cRef = mean(tempRefAng);
                         end
                     elseif 0
                         k2cRef = tempRefAng(end);
                     else
                         k2cRef = tempRefAng(1);
                     end
%                      k2cRef = angOptListLast(end) - angOptListLast((length(angOptListLast) - keyFrameLen ) +1);
                     fdjdsut = 1;
                 end
            end
        end
        
        
        % k2cRef_1_00 = obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs;
%         k2cRef_1_00 = obj.accumP2CRef(end) - obj.accumP2CRef(length(obj.accumP2CRef) - keyFrameLen + dltId + 1); %obj.accumP2CRef(end) - obj.accumP2CRef(1);
        k2cRef_1_00 = obj.accumP2CRef(end) - obj.accumP2CRef(length(obj.accumP2CRef) - keyFrameLen + 0 + 1); %obj.accumP2CRef(end) - obj.accumP2CRef(1);
        
        AngleModalOrg = obj.angleModalOrg;
        AngleProbility = [];
        
        
        
        
        
        
    end
    
    traceBatch = 0;
    
    for hb = 1 : length(obj.featBatch)
        
        if sum(ismember(LocalTrace.featId, obj.featBatch{hb})) > 0    
            traceBatch = hb;
            break;
        end
        
    end
    
    
    
    kjsad = 1;
    
else
    validTrace = false;
end