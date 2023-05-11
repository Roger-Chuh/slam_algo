


if 0
    
    %%
else
    LocalTraceDone.configParam.disparity_resample_prob_threshold = disparity_resample_prob_threshold;
    %                                             ThetaNoise = LocalTraceDone.thetaNoise;
    LocalTraceDone.keyFrameFlagList = KeyFrameFlagList;
    %                                             ConfigParam = LocalTraceDone.configParam;
    % %     LocalTraceDone.depthVisual = depthMapCurVisual;
    % %     LocalTraceDone.depthGT = depthMapCurGT;
    LocalTraceDone.currImgL = imgCur(:,:,1);
    LocalTraceDone.currImgR = imgCurR(:,:,1);
    LocalTraceDone.keyFrameImgL = imgKeyL(:,:,1);
    LocalTraceDone.keyFrameImgR = imgKeyR(:,:,1);
    LocalTraceDone.prevImgL = imgPrvL(:,:,1);
    LocalTraceDone.prevImgR = imgPrvR(:,:,1);
    % %     LocalTraceDone.keyFrameDepth = depthMapKey;
    % %     LocalTraceDone.keyFrameDepthGT = depthMapKeyGT;
    if 0
        LocalTraceDone.featPtManager.localTrace = LocalTrace;
    end
    %     LocalTraceDone.keyProbZ = KeyProbZ;
    if 0
        obj.tempProb{traceIndex + dltIdAll,2} = KeyProbZ;
    else
        tempProbNew{traceIndex,2} = KeyProbZ;
    end
    
    trackingErr12345 = KeyProbZ{end,6} - KeyProbZ{end,7}; % pt2dCur - pixGT;
    if 0
        obj.trackingErrDistribution{size(KeyProbZ,1),1} = [obj.trackingErrDistribution{size(KeyProbZ,1),1}; [trackingErr12345]];
    else
        obj.trackingErrDistribution{size(KeyProbZ,1),1} = [obj.trackingErrDistribution{size(KeyProbZ,1),1}; [{trackingErr12345} {traceBatch} {LocalTrace.featId}] ];
    end
    
    [~, refTrackingErr] = NormalizeVector(KeyProbZ{end,6} - KeyProbZ{end,7});
    checkTrackingErr = refTrackingErr - KeyProbZ{end,8};
    
    LocalTrace.cfg = cfg;
    LocalTraceDone.LocalTrace = LocalTrace;
    
    
    
    LocalTraceDone.thetaRngMat = thetaRngMat;
    LocalTraceDone.thetaProbMat = thetaProbMat;
    % %     LocalTraceDone.prvDepthGT = prvDepthGT;
    % %     LocalTraceDone.prvDepthVisual = prvDepthVisual;
    %                                             CamModel = LocalTraceDone.camModel;
    %                                             CoordSysAligner = LocalTraceDone.coordSysAligner;
    LocalTraceDone.refAngList = REFAngList;
    LocalTraceDone.refAngList3 = REFAngList3;
    LocalTraceDone.refAngList4 = REFAngList4;
    LocalTraceDone.accumP2CTemp = accumP2CTemp;
    LocalTraceDone.twoOnlyP2List = twoOnlyP2List;
    LocalTraceDone.thetaPlatform = thetaPlatform;
    LocalTraceDone.thetaPlatformDistribution = thetaPlatformDistribution;
    LocalTraceDone.poseWcsList = poseWcsList;
    LocalTraceDone.angOpt = ANGOpt;
    LocalTraceDone.angRng = ANGRng;
    LocalTraceDone.meanErrAndStd = meanErrAndStd;
    LocalTraceDone.p2cErrList = p2cErrList;
    LocalTraceDone.PervPtCcsZGolden = objPervPtCcsZGolden1;
    
    thetaPnP = pnpLite(obj.pureRotationAngPredictor,intrMat,obj.featPtManager,obj.coordSysAligner,k2cRef,obj.keyFrameFlagList,obj.frmStampList,obj.goldenPose,pixKey ,pt2dCur, DispRefine, obj.camModel, obj.scaleLvl);
    
    
    
    if traceIndex == 1
        obj.refAngList = REFAngList;
        obj.refAngList3 = REFAngList3;
        obj.refAngList4 = REFAngList4;
        obj.accumP2CTemp = accumP2CTemp;
        obj.twoOnlyP2List = twoOnlyP2List;
        obj.thetaPlatform = thetaPlatform;
        obj.thetaPlatformDistribution = thetaPlatformDistribution;
        obj.poseWcsList = poseWcsList;
        obj.angOpt = ANGOpt;
        obj.angRng = ANGRng;
        obj.meanErrAndStd = meanErrAndStd;
        obj.p2cErrList = p2cErrList;
        obj.PervPtCcsZGolden = objPervPtCcsZGolden1;
        if 0
            obj.angOptP2CList = ANGOptP2CList;
        end
    end
end




LocalTraceDone.angOpt = [LocalTraceDone.angOpt; [angOpt1 ]];
try
    LocalTraceDone.angOptCheck = [LocalTraceDone.angOptCheck; angOptK2C_ ];
catch
    LocalTraceDone.angOptCheck = [[]; angOptK2C_ ];
end
try
    LocalTraceDone.angOpt3 = [LocalTraceDone.angOpt3; [angOpt33 ]];
    
    LocalTraceDone.accumP2C = [LocalTraceDone.accumP2C; LocalTraceDone.accumP2CTemp(end)];
catch
    LocalTraceDone.angOpt3 = [ [angOpt33 ]];
    
    LocalTraceDone.accumP2C = [ LocalTraceDone.accumP2CTemp(end)];
end

if size(LocalTrace.ptIcsX, 2) == 2%size(obj.traceManager.X, 2) == 2
    LocalTraceDone.refAngList4 = [];
end

try
    LocalTraceDone.angOptP2CList;
catch
    LocalTraceDone.angOptP2CList = [];
end

if isempty(LocalTraceDone.angOptP2CList)
    LocalTraceDone.angOptP2CList = [LocalTraceDone.angOptP2CList; angOpt1 ];
else
    if isempty(LocalTraceDone.refAngList4)
        LocalTraceDone.angOptP2CList = [LocalTraceDone.angOptP2CList; angOpt1 ];
    else
        LocalTraceDone.angOptP2CList = [LocalTraceDone.angOptP2CList; LocalTraceDone.refAngList4(end) + angOptP2C ];
    end
end


if Switch4to3
    LocalTraceDone.refAngList4 = [LocalTraceDone.refAngList3];
else
    LocalTraceDone.refAngList4 = [LocalTraceDone.refAngList4; angOpt1 ];
end

try
    LocalTraceDone.angOptPnP = [LocalTraceDone.angOptPnP; [thetaPnP ]];
catch
    LocalTraceDone.angOptPnP = [[thetaPnP ]];
end
LocalTraceDone.angRng = [LocalTraceDone.angRng; angRng1 ];

LocalTraceDone.thetaRngMat = [LocalTraceDone.thetaRngMat; thetaRngOut ];

LocalTraceDone.thetaProbMat = [LocalTraceDone.thetaProbMat; thetaProbOut ];
if 0
    obj.tempProb{traceIndex + dltIdAll,1} = LocalTraceDone;
else
    tempProbNew{traceIndex,1} = LocalTraceDone;
end

if length(obj.accumP2CRef) <= FrmNumWithGloden0 + 1
    abbjk = 1;
else
    if traceIndex == 1
        angOptListLast(traceIndex,:) = [0;LocalTraceDone.angOpt]';
    else
        angOptListLast(traceIndex,size(angOptListLast,2) - keyFrameLen +1:end) = [0;LocalTraceDone.angOpt]';
    end
end


asbaasb = 1;



if 0
    LocalTraceDone.LocalTraceDone = [LocalTraceDone.LocalTraceDone; LocalTraceDone];
end


if 0
    keyId = find(LocalTraceDone.keyFrameFlagList);
    keyIdList = LocalTraceDone.keyFrameFlagList(keyId(1)+0:end);
    KeyAng = [keyIdList [LocalTraceDone.angOpt;0]];
    
    
    accumTrace2 = [];
    KeyFrmCnt = 0;
    for ui = 1 : size(KeyAng,1) - 1
        if KeyAng(ui,1) == 1
            KeyFrmCnt = KeyFrmCnt + 1;
        end
        if KeyFrmCnt == 1
            accumTrace2 = [accumTrace2; KeyAng(ui,2)];
        else
            idKeyTemp = find(KeyAng(1:ui,1) == 1);
            baseAng = accumTrace2(idKeyTemp(end)-1);
            accumTrace2 = [accumTrace2; baseAng + KeyAng(ui,2)];
            
        end
        
    end
    
    errTrace2 = rad2deg([0;accumTrace2] - [0;cumsum(LocalTraceDone.refAngAll)]);
    KeyAngCat = [zeros(length(LocalTraceDone.keyFrameFlagList) - size(KeyAng,1),2);[KeyAng(:,1) deg2rad(errTrace2)]];
    LocalTraceDone.KeyAngCat = KeyAngCat;
end
%                                         pulse = repmat(min(rad2deg(KeyAngCat(:,2))),size(KeyAngCat,1),1);
%                                         pulse(idPulse) = max(errTrace2);
%                                         figure(333),clf;plot(rad2deg(KeyAngCat(:,2)),'-b');hold on;plot(find(KeyAngCat(:,1)),rad2deg(KeyAngCat(find(KeyAngCat(:,1)),2)),'*g'); % plot(pulse,'k');
%                                         drawnow;title('accum P2C')
%                                         saveas(gcf,fullfile(probPath,sprintf('final_error_%05d.fig',0+3)));





