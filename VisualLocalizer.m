classdef VisualLocalizer < BaseLocalizer
     
    properties (Constant)
        INSERT_KEYFRAME_NO = 0;
        INSERT_KEYFRAME_CURRENT = 1;
        INSERT_KEYFRAME_PREVIOUS = 2;  
        DEBUG_FLAG = false;
    end
    
    properties
        camModel;
        coordSysAligner;
        scaleLvl;
        stereoDepthMap;  
        featPtExtractor;    
        featPtTracker;
        featPtManager;
        rotAngPredictor;
        robotPosePredictor;
        angleSpace;
        pointCloudManager;
        pureRotationAngPredictor;
        lookUpTables;
        modals;
        points;
        setting;
        traceWithWeights;
        
        lookUpTables2;
        modals2;
        points2;
        setting2;
        traceWithWeights2;
        
        
        prevImgL;
        prevImgR;
        prevRefRobotPoseWcs;
        prevRefRobotRotAngWcs;
        currImgL;
        currImgR;
        currRefRobotPoseWcs;
        currRefRobotRotAngWcs;
        currMotionState;
        keyFrameFlagList;
        keyRefRobotPoseWcs;
        keyRefRobotRotAngWcs;
        refRobotPosVec;
        delta;
        deltaAngle;
        frmStamp;
        goldenPose;
        frmStampList;
        depthGT;
        depthVisual;
        switchDepth;
        prvDepthGT;
        prvDepthVisual;
        keyFrameImgL;
        keyFrameImgR;
        keyFrameDone;
        dispMap;
        keyFrameDepth;
        keyFrameDepthGT;
        traceErr;
        MeanErr;
        MeanErrGT;
        MeanErrOld;
        noIntersection;
        angOpt;
        keyProbZ;
        angRng;
        refAngList;
        feat2check;
        angOpt2;
        refAngList2;
        angRng2;
        keyProbZ2;
        thetaPlatform;
        thetaPlatformDistribution;
        angOptPnP;
        m;
        angleModalOrg;
        PervPtCcsZGolden;
        refAngList3;
        angOpt3;
        twoOnlyP2List;
        meanErrAndStd;
        refAngList4;
        curDispStepList;
        thetaRngMat;
        thetaProbMat;
        angOptP2CList;
        thetaNoise;
        p2cErrList;
        angOptCheck;
        accumP2C;
        accumP2CTemp;
        startTrace2;
        trace2;
        traceAng1;
        trace2KeyRefAng;
        trace1KeyRefAng;
        useGoldenK2C;
        useGoldenK2C1;
        useGoldenK2C2;
        trackingErrAccum;
        trackingErrAccum2;
        traceManager;
        colorMat;
        accumP2CRef;
        accumP2CPNP;
        accumP2CPNP2;
        accumP2CPNP2_2;
        accumP2CPNP3;
        LocalTraceDone;
        angOptNew;
        tempProb;
        angOptManager;
        validFeatIdStack;
        featBatch;
        stackGTDisp;
        trackingErrDistribution
        p2cErrComp;
        judgement;
        addPt;
        dispErrExpStack;
        dispErrExpStack2;
        dispErrExpStack3;
        TraceBatchListStack;
        imuSample;
        goldenPoseStackImu;
        goldenPoseStackWheel;
        p2cTrackingErrWholeStack;
        traceInfoMat;
    end
    
    methods
        % Constructor
        function obj = VisualLocalizer(camModel, coordSysAligner, cfgParam)
            obj@BaseLocalizer(cfgParam);
            obj.scaleLvl = obj.configParam.scale_level;
            obj.camModel = camModel;
            obj.coordSysAligner = coordSysAligner;
            obj.stereoDepthMap = StereoDepthMapSgbm(obj.configParam.sgbm_stereo_matching_param);
            obj.featPtExtractor = FeaturePointExtractorOrb(obj.configParam.orb_feature_detector_param);
            InitPointTracker(obj, obj.configParam.feature_tracking_param.param_list);
            obj.featPtManager = FeaturePointManager(obj.configParam.left_feature_point_trace_param);
            obj.rotAngPredictor = RotateAnglePredictor([]);
            obj.robotPosePredictor = RobotPosePredictor([]);
            obj.angleSpace = AngleSpace([]);
            obj.pointCloudManager = PointCloudManager([]);
            obj.pureRotationAngPredictor = PureRotationAngPredictor([]);
            obj.setting = Setting([]);
            obj.lookUpTables = LookUpTables([]);
            obj.modals = Modals([]);
            obj.points = Points([]);
            
            obj.traceWithWeights = TraceWithWeights([]);
            
            
            obj.setting2 = Setting2([]);
            obj.lookUpTables2 = LookUpTables2([]);
            obj.modals2 = Modals2([]);
            obj.points2 = Points2([]);
            
            obj.traceWithWeights2 = TraceWithWeights2([]);
            
            
            
            obj.prevImgL = [];
            obj.prevImgR = [];
            obj.prevRefRobotPoseWcs = PointCoordTransformer;
            obj.currImgL = [];
            obj.currImgR = [];
            obj.currRefRobotPoseWcs = [];
            obj.keyFrameFlagList = [];
            obj.deltaAngle = [];
            obj.delta = [];
            obj.frmStamp = [];
            obj.goldenPose = [];
            obj.frmStampList = [];
            obj.depthGT = [];
            obj.switchDepth = false;
            obj.prvDepthGT = [];
            obj.depthVisual = [];
            obj.prvDepthVisual = [];
            obj.keyFrameImgL = [];
            obj.keyFrameImgR = [];
            obj.keyFrameDone = false;
            obj.dispMap = [];
            obj.keyFrameDepth = [];
            obj.keyFrameDepthGT = [];
            obj.traceErr = {};
            obj.MeanErr = [0 0];
            obj.MeanErrGT = [0 0];
            obj.MeanErrOld = [0 0];
            obj.noIntersection = 0;
            obj.angOpt = [];
            obj.keyProbZ = {};
            obj.angRng = [];
            obj.refAngList = [];
            obj.feat2check = [];
            obj.angOpt2 = [];
            obj.refAngList2 = [];
            obj.angRng2 = [];
            obj.keyProbZ2 = {};
            obj.thetaPlatform = [];
            obj.thetaPlatformDistribution = [];
            obj.angOptPnP = [];
            obj.m = [];
            obj.PervPtCcsZGolden = [];
            obj.refAngList3 = [];
            obj.angOpt3 = [];
            obj.twoOnlyP2List = [];
            obj.meanErrAndStd = [];
            obj.refAngList4 = [];
            obj.curDispStepList = [];
            obj.thetaRngMat = [];
            obj.thetaProbMat = [];
            obj.angOptP2CList = [];
            obj.thetaNoise = 0;
            obj.p2cErrList = [];
            obj.angOptCheck = [];
            
            obj.accumP2C = [];
            obj.accumP2CTemp = [];
            
            obj.startTrace2 = false;
            obj.traceAng1 = [];
            obj.trace2KeyRefAng = [];
            obj.trace1KeyRefAng = [];
            
            
            obj.trace2.keyRefRobotPoseWcs = [];
            obj.trace2.keyRefRobotRotAngWcs = [];
            obj.trace2.prevImgL = [];
            obj.trace2.prevImgR = [];
            obj.trace2.prevRefRobotPoseWcs = PointCoordTransformer;
            obj.trace2.currImgL = [];
            obj.trace2.currImgR = [];
            obj.trace2.currRefRobotPoseWcs = [];
            obj.trace2.keyFrameFlagList = [];
            obj.trace2.deltaAngle = [];
            obj.trace2.delta = [];
            obj.trace2.frmStamp = [];
            obj.trace2.goldenPose = [];
            obj.trace2.frmStampList = [];
            obj.trace2.depthGT = [];
            obj.trace2.switchDepth = false;
            obj.trace2.prvDepthGT = [];
            obj.trace2.depthVisual = [];
            obj.trace2.prvDepthVisual = [];
            obj.trace2.keyFrameImgL = [];
            obj.trace2.keyFrameImgR = [];
            obj.trace2.keyFrameDone = false;
            obj.trace2.dispMap = [];
            obj.trace2.keyFrameDepth = [];
            obj.trace2.keyFrameDepthGT = [];
            obj.trace2.traceErr = {};
            obj.trace2.MeanErr = [0 0];
            obj.trace2.MeanErrGT = [0 0];
            obj.trace2.MeanErrOld = [0 0];
            obj.trace2.noIntersection = 0;
            obj.trace2.angOpt = [];
            obj.trace2.keyProbZ = {};
            obj.trace2.angRng = [];
            obj.trace2.refAngList = [];
            obj.trace2.feat2check = [];
            obj.trace2.angOpt2 = [];
            obj.trace2.refAngList2 = [];
            obj.trace2.angRng2 = [];
            obj.trace2.keyProbZ2 = {};
            obj.trace2.thetaPlatform = [];
            obj.trace2.thetaPlatformDistribution = [];
            obj.trace2.angOptPnP = [];
            obj.trace2.m = [];
            obj.trace2.PervPtCcsZGolden = [];
            obj.trace2.refAngList3 = [];
            obj.trace2.angOpt3 = [];
            obj.trace2.twoOnlyP2List = [];
            obj.trace2.meanErrAndStd = [];
            obj.trace2.refAngList4 = [];
            obj.trace2.curDispStepList = [];
            obj.trace2.thetaRngMat = [];
            obj.trace2.thetaProbMat = [];
            obj.trace2.angOptP2CList = [];
            obj.trace2.thetaNoise = 0;
            obj.trace2.p2cErrList = [];
            obj.trace2.angOptCheck = [];
            
            obj.trace2.poseWcsList = [];
            
            obj.trace2.accumP2C = [];
            obj.trace2.accumP2CTemp = [];
            obj.trace2.refAngAll = [];
            obj.trace2.KeyAngCat = [];
            
            
            
            obj.useGoldenK2C = 0;
            obj.useGoldenK2C1 = [];
            obj.useGoldenK2C2 = [];
            obj.trackingErrAccum = [];
            obj.trackingErrAccum2 = [];
            
            
            obj.traceManager.X = [];
            obj.traceManager.Y = [];
            obj.traceManager.Z = [];
            obj.traceManager.ZGT = [];
            
            obj.accumP2CRef = 0;
            obj.accumP2CPNP = 0;
            obj.accumP2CPNP2 = 0;
            obj.accumP2CPNP2_2 = 0;
            obj.accumP2CPNP3 = 0;
            obj.LocalTraceDone = {};
            obj.angOptNew = 0;
            obj.tempProb = {};
            
            obj.angOptManager.featIdCell = {};
            obj.angOptManager.angOptMat = [];
            obj.angOptManager.angOptUpperMat = [];
            obj.angOptManager.angOptLowerMat = [];
            obj.angOptManager.angOptUpperMatUniq = [];
            obj.angOptManager.angOptLowerMatUniq = [];
            obj.angOptManager.angOptMatUniq = [];
            obj.angOptManager.angRefMat = [];
            obj.angOptManager.angErrMat = [];
            
            obj.validFeatIdStack = [];
            obj.featBatch = {};
            obj.stackGTDisp = [];
            obj.trackingErrDistribution = {};
            obj.colorMat = rand(1000,3);
            obj.p2cErrComp = [];
            obj.judgement = {};
            obj.addPt = [];
            obj.dispErrExpStack = [];
            obj.dispErrExpStack2 = [];
            obj.dispErrExpStack3 = {};
            obj.TraceBatchListStack = {};
            obj.imuSample = [];
            obj.goldenPoseStackImu = [];
            obj.goldenPoseStackWheel = [];
            obj.p2cTrackingErrWholeStack = [];
            obj.traceInfoMat = {};
        end
        
        % Main entry
        function Localize(obj, imgL, imgR, refMotionState, refRobotPoseWcs, frameInd)
            if (obj.scaleLvl > 0)
                imgL = imresize(imgL, 1/2^obj.scaleLvl);
                imgR = imresize(imgR, 1/2^obj.scaleLvl);
            end
            
            if 0
                if size(imgL, 1) == 240
                    obj.featPtTracker.configParam.win_width = 9;
                    obj.featPtTracker.configParam.win_height = 9;
                    obj.featPtTracker.configParam.max_level = 3;
                    
                else
                    obj.featPtTracker.configParam.win_width = 11;
                    obj.featPtTracker.configParam.win_height = 11;
                    obj.featPtTracker.configParam.max_level = 5;
                end
            end
            
            
            obj.currImgL = imgL;
            obj.currImgR = imgR;
            obj.currRefRobotPoseWcs = WcsPoseVecToPct(obj, refRobotPoseWcs);
            obj.currRefRobotRotAngWcs = refRobotPoseWcs(3) +  0;  %deg2rad(0.01*rand(1));
            obj.refRobotPosVec = refRobotPoseWcs; 
            doInsert = NeedInsertKeyFrame(obj, refMotionState, false);
            if 0 %size(obj.poseWcsList,1) == 96; 129; 47; 35;  129; 104; 33;  250; 97; 32; 81; 257
                doInsert = 2;
            end
            
% %             if 1  %size(obj.poseWcsList,1) == 97; 32; 81; 257
% %                 doInsert = 0;
% %             end
            
            
            obj.currMotionState = refMotionState;
            debug.debug_904x_FindDebugCase(frameInd == -1,doInsert == -1,refMotionState == -1);
            obj.depthVisual = GetDepthMap(obj, imgL, imgR);
            if ~obj.switchDepth
                [depthMap, dispMap] = GetDepthMap(obj, imgL, imgR);
            else
                if ~isempty(obj.depthGT)
                    depthMap = obj.depthGT;
                else
                    [depthMap, dispMap] = GetDepthMap(obj, imgL, imgR);
                end
            end
            if 0 % 20191009
                StoreImage(obj.pointCloudManager,imgL);
                StoreDepthMap(obj.pointCloudManager,depthMap);
            end
            if refMotionState == obj.MOTIONSTATE_STILL
                if IsEmpty(obj)
                    %  record motion before the first frame
                    SetMotion(obj, obj.MOTIONSTATE_STILL, refRobotPoseWcs);
                    %                     StoreTrace(obj.pointCloudManager, obj.poseWcsList, 'blue')
                else
                    SetMotion(obj, obj.MOTIONSTATE_STILL, GetPrevPoseWcs(obj));
                    %                     StoreTrace(obj.pointCloudManager, obj.poseWcsList, 'blue')
                    %                     SetMotion(obj, obj.MOTIONSTATE_STILL, refRobotPoseWcs);
                end
                
                if doInsert == obj.INSERT_KEYFRAME_NO
                    obj.keyFrameFlagList = [obj.keyFrameFlagList; false];
                    obj.prevImgL = obj.currImgL;
                    obj.prevImgR = obj.currImgR;
                    obj.prvDepthGT = obj.depthGT;
                    obj.prvDepthVisual = obj.depthVisual;
                    obj.prevRefRobotPoseWcs = obj.currRefRobotPoseWcs;
                    obj.prevRefRobotRotAngWcs = obj.currRefRobotRotAngWcs;
                    %                     PlotSense(obj.pointCloudManager)
                    CheckDataConsistency(obj);
                    return;
                elseif (doInsert == obj.INSERT_KEYFRAME_CURRENT)
                    obj.keyRefRobotPoseWcs = obj.currRefRobotPoseWcs;
                    obj.keyRefRobotRotAngWcs = obj.currRefRobotRotAngWcs;
                    obj.keyFrameFlagList =  [obj.keyFrameFlagList; true];
                    obj.keyFrameImgL = obj.currImgL;
                    obj.keyFrameImgR = obj.currImgR;
                    obj.keyFrameDepth = depthMap;
                    obj.keyFrameDepthGT = obj.depthGT;
                    InsertKeyFrame(obj, obj.currImgL, depthMap);
                    %                     MergePointMap(obj);
                    obj.prevImgL = obj.currImgL;
                    obj.prevImgR = obj.currImgR;
                    obj.prvDepthGT = obj.depthGT;
                    obj.prvDepthVisual = obj.depthVisual;
                    obj.prevRefRobotPoseWcs = obj.currRefRobotPoseWcs; 
                    obj.prevRefRobotRotAngWcs = obj.currRefRobotRotAngWcs;
                    %                     PlotSense(obj.pointCloudManager)
                    CheckDataConsistency(obj);
                    debug.debug_910x_ShowKeyFrameLocation(obj,doInsert)
                    return;
                elseif (doInsert == obj.INSERT_KEYFRAME_PREVIOUS)
                    obj.keyRefRobotPoseWcs = obj.prevRefRobotPoseWcs;
                    obj.keyRefRobotRotAngWcs = obj.prevRefRobotRotAngWcs;
                    if ~ obj.switchDepth
                        [depthMap, dispMap] = GetDepthMap(obj, obj.prevImgL, obj.prevImgR);
                    else
                        if ~isempty(obj.prvDepthGT)
                            depthMap = obj.prvDepthGT;
                        else
                            [depthMap, dispMap] = GetDepthMap(obj, obj.prevImgL, obj.prevImgR);
                        end
                    end
                    obj.keyFrameImgL = obj.prevImgL;
                    obj.keyFrameImgR = obj.prevImgR;
                    obj.keyFrameDepth = depthMap;
                    obj.keyFrameDepthGT = obj.prvDepthGT;
                    InsertKeyFrame(obj, obj.prevImgL, depthMap);
                    obj.keyFrameFlagList(end) = true;
                    %                     MergePointMap(obj)
                    %                     StoreImage(obj.pointCloudManager,obj.prevImgL)
                    %                     ExtendPointCould(obj);
                    obj.keyFrameFlagList = [obj.keyFrameFlagList; false];
                    obj.prevImgL = obj.currImgL;
                    obj.prevImgR = obj.currImgR;
                    obj.prvDepthGT = obj.depthGT;
                    obj.prvDepthVisual = obj.depthVisual;
                    
                    
                    obj.prevRefRobotPoseWcs = obj.currRefRobotPoseWcs;
                    obj.prevRefRobotRotAngWcs = obj.currRefRobotRotAngWcs;
                    %                     PlotSense(obj.pointCloudManager)
                    CheckDataConsistency(obj);
                    debug.debug_910x_ShowKeyFrameLocation(obj,doInsert)
                    return;
                end
            elseif IsEmpty(obj)
                % record motion before the first frame
                SetMotion(obj, refMotionState, refRobotPoseWcs);
                %                 StoreTrace(obj.pointCloudManager, obj.poseWcsList, 'blue')
                obj.keyRefRobotPoseWcs = obj.currRefRobotPoseWcs;
                obj.keyRefRobotRotAngWcs = obj.currRefRobotRotAngWcs;
                obj.keyFrameFlagList =  [obj.keyFrameFlagList; true];
                InsertKeyFrame(obj, obj.currImgL, depthMap);
                obj.keyFrameImgL = obj.currImgL;
                obj.keyFrameImgR = obj.currImgR;
                obj.keyFrameDepth = depthMap;
                obj.keyFrameDepthGT = obj.depthGT;
                %                     MergePointMap(obj);
                obj.prevImgL = obj.currImgL;
                obj.prevImgR = obj.currImgR;
                obj.prvDepthGT = obj.depthGT;
                obj.prvDepthVisual = obj.depthVisual;
                
                obj.prevRefRobotPoseWcs = obj.currRefRobotPoseWcs;
                obj.prevRefRobotRotAngWcs = obj.currRefRobotRotAngWcs;
                %                 PlotSense(obj.pointCloudManager)
                CheckDataConsistency(obj);
                debug.debug_910x_ShowKeyFrameLocation(obj,doInsert)
                return;
            end
            
            if (doInsert == obj.INSERT_KEYFRAME_PREVIOUS)
                
                
                      
                
                
                
                obj.keyRefRobotPoseWcs = obj.prevRefRobotPoseWcs;
                obj.keyRefRobotRotAngWcs = obj.prevRefRobotRotAngWcs;
                %                 depthMap = GetDepthMap(obj, obj.prevImgL, obj.prevImgR);
                if ~obj.switchDepth
                    [depthMap, dispMap] = GetDepthMap(obj, obj.prevImgL, obj.prevImgR);
                else
                    if ~isempty(obj.prvDepthGT)
                        depthMap = obj.prvDepthGT;
                    else
                        [depthMap, dispMap] = GetDepthMap(obj, obj.prevImgL, obj.prevImgR);
                    end
                end
                InsertKeyFrame(obj, obj.prevImgL, depthMap);
                
                [obj.prvDepthVisual, ~] = GetDepthMap(obj, obj.prevImgL, obj.prevImgR);
                
                obj.keyFrameImgL = obj.prevImgL;
                obj.keyFrameImgR = obj.prevImgR;
                obj.keyFrameDepth = depthMap;
                obj.keyFrameDepthGT = obj.prvDepthGT;
                obj.keyFrameFlagList(end) = true;
                if refMotionState == obj.MOTIONSTATE_ROTATE
                    %                     StoreImage(obj.pointCloudManager,obj.prevImgL)
                    %                     ExtendPointCould(obj);
                else
                    %                     StoreImage(obj.pointCloudManager,obj.prevImgL)
                    %                     ExtendPointCould(obj);
                end
                obj.keyFrameFlagList = [obj.keyFrameFlagList; false];
                if refMotionState == obj.MOTIONSTATE_ROTATE
                    [robotPoseWcs] = PureRotationTrackAndLocalize(obj);
                else
                    [robotPoseWcs] = TrackAndLocalize(obj);
                end
                SetMotion(obj, refMotionState, robotPoseWcs);
                %                 StoreTrace(obj.pointCloudManager, obj.poseWcsList, 'blue')
            elseif (doInsert == obj.INSERT_KEYFRAME_CURRENT)
                obj.keyRefRobotPoseWcs = obj.currRefRobotPoseWcs;
                obj.keyRefRobotRotAngWcs = obj.currRefRobotRotAngWcs;
                if refMotionState == obj.MOTIONSTATE_ROTATE
                    [robotPoseWcs] = PureRotationTrackAndLocalize(obj);
                else
                    [robotPoseWcs] = TrackAndLocalize(obj);
                end
                SetMotion(obj, refMotionState, robotPoseWcs);
                %                 StoreTrace(obj.pointCloudManager, obj.poseWcsList, 'blue')
                obj.keyFrameFlagList =  [obj.keyFrameFlagList; true];
                obj.keyFrameImgL = obj.currImgL;
                obj.keyFrameImgR = obj.currImgR;
                obj.keyFrameDepth = depthMap;
                obj.keyFrameDepthGT = obj.depthGT;
                InsertKeyFrame(obj, obj.currImgL, depthMap);
                if refMotionState == obj.MOTIONSTATE_ROTATE
                    %                     StoreImage(obj.pointCloudManager,obj.prevImgL)
                    %                     ExtendPointCould(obj);
                else
                    %                     StoreImage(obj.pointCloudManager,obj.prevImgL)
                    %                     ExtendPointCould(obj);
                end
                
            elseif (doInsert == obj.INSERT_KEYFRAME_NO)
                obj.keyFrameFlagList = [obj.keyFrameFlagList; false];
                if refMotionState == obj.MOTIONSTATE_ROTATE
                    [robotPoseWcs] = PureRotationTrackAndLocalize(obj);
                else
                    [robotPoseWcs] = TrackAndLocalize(obj);
                end
                SetMotion(obj, refMotionState, robotPoseWcs);
                %                 StoreTrace(obj.pointCloudManager, obj.poseWcsList, 'blue')
            else
                error('Unknown keyframe insertion mode %d', doInsert);
            end
            
            %             if ~isempty(POINT_TRACE_KEY_SEQ_FIG)
            figure(1122), UpdateLocalTracePlot(obj.featPtManager, obj.currImgL);title(sprintf('Frame %d', frameInd));drawnow;
            %             end
            
            %             global PATH_FIG
            %             if ~isempty(PATH_FIG)
            %                 figure(PATH_FIG), UpdateTrajectoryPlot(obj);
            %             end
            
            debug.debug_911x_CheckRotationAngle(obj,frameInd)
            %             ExtendPointCloud_OpticalFlow1D(obj);
            
            obj.prevImgL = obj.currImgL;
            obj.prevImgR = obj.currImgR;
            obj.prvDepthGT = obj.depthGT;
            obj.prvDepthVisual = obj.depthVisual;
            obj.prevRefRobotPoseWcs = obj.currRefRobotPoseWcs;
            obj.prevRefRobotRotAngWcs = obj.currRefRobotRotAngWcs;
            %             PlotSense(obj.pointCloudManager)
            
            CheckDataConsistency(obj);
            debug.debug_910x_ShowKeyFrameLocation(obj,doInsert);
            DoInsert = NeedInsertKeyFrame(obj, refMotionState, true);
            [FeatureNumDropRate(obj.featPtManager)*100 NumActiveFeatures(obj.featPtManager)];
        end
        
        % Member functions
        function b = NeedInsertKeyFrame(obj, refMotionState, keyFlag)
            
            
            
            
            
            
            if (GetPrevMotionState(obj) == obj.MOTIONSTATE_UNKNOWN)
                if (refMotionState ~= obj.MOTIONSTATE_STILL)
                    b = obj.INSERT_KEYFRAME_CURRENT;
                    if keyFlag
                        obj.keyFrameDone = true;
                    end
                else
                    b = obj.INSERT_KEYFRAME_NO;
                    if keyFlag
                        obj.keyFrameDone = false;
                    end
                end
                return;
            end
            
            assert(~isempty(obj.keyFrameFlagList), 'For non very beginning frame, the keyFrameFlagList should not be empty');
            if (GetPrevMotionState(obj) ~= refMotionState)
                if (refMotionState ~= obj.MOTIONSTATE_STILL)
                    if ~obj.keyFrameFlagList(end)
                        b = obj.INSERT_KEYFRAME_PREVIOUS;
                        if keyFlag
                            if length(obj.keyFrameFlagList) > 1
                                obj.keyFrameDone = true;
                            end
                        end
                    else
                        b = obj.INSERT_KEYFRAME_NO;
                        if keyFlag
                            obj.keyFrameDone = false;
                        end
                    end
                else
                    b = obj.INSERT_KEYFRAME_CURRENT;
                    if keyFlag
                        obj.keyFrameDone = true;
                    end
                end
                return;
            end
            
            %             disp(FeatureNumDropRate(obj.featPtManager))
            if (FeatureNumDropRate(obj.featPtManager) > obj.configParam.feature_num_drop_rate_thresh)
                if ~obj.keyFrameFlagList(end)
                    b = obj.INSERT_KEYFRAME_PREVIOUS;
                    if keyFlag
                        obj.keyFrameDone = true;
                    end
                else
                    b = obj.INSERT_KEYFRAME_NO;
                    if keyFlag
                        obj.keyFrameDone = false;
                    end
                end
                return;
            end
            
            if (FeatureNumDropRate(obj.featPtManager) ~= 0 && NumActiveFeatures(obj.featPtManager) < obj.configParam.min_feature_point_number)
                if ~obj.keyFrameFlagList(end)
                    b = obj.INSERT_KEYFRAME_PREVIOUS;
                    if keyFlag
                        obj.keyFrameDone = true;
                    end
                else
                    b = obj.INSERT_KEYFRAME_NO;
                    if keyFlag
                        obj.keyFrameDone = false;
                    end
                end
                return;
            end
            
            %             if (FeatureNumDropRate(obj.featPtManager) > obj.configParam.feature_num_drop_rate_thresh)
            %                 if ~obj.keyFrameFlagList(end)
            %                     b = obj.INSERT_KEYFRAME_PREVIOUS;
            %                 else
            %                     b = obj.INSERT_KEYFRAME_NO;
            %                 end
            %                 return;
            %             end
            
            
            b = obj.INSERT_KEYFRAME_NO;
            if keyFlag
                obj.keyFrameDone = false;
            end
        end
        
        function InsertKeyFrame(obj, imgL, depthMap)
            
            % refresh Body error
            obj.robotPosePredictor.prvBodyOrigErrRng = [-0,0];
            
            % Calculate depth map
            %             depthMap = GetDepthMap(obj, imgL, imgR);
            
            
            %             StoreDepthMap(obj.pointCloudManager,depthMap);
            
            % Extract feature points
            [topMargin, leftMargin, bottomMargin, rightMargin] = GetPredictMargin(obj.featPtTracker);
            predReginMask = zeros(size(depthMap));
            predReginMask(topMargin + 1 : Rows(depthMap) - bottomMargin, leftMargin + 1 : Cols(depthMap) - rightMargin) = 1;
            ptIcs = ExtractFeaturePoints(obj.featPtExtractor, imgL, predReginMask);
            
            % Get depth value for each feature point
            if (LocalTraceLength(obj.featPtManager) < 2)
                pctCcs = [];
            else
                pctBcs = GetDeltaPose(obj, -1, 0, 'PointCoordTransformer');
                rotAng = RotAxang(pctBcs);
                transVec = T(pctBcs);
                if (abs(rotAng) < obj.configParam.zero_rotation_angle_thresh && norm(transVec) < obj.configParam.zero_translation_thresh)
                    pctCcs = [];
                else
                    pctCcs = BodyMoveCcs(obj.coordSysAligner, pctBcs, 1);
                end
                
                
% %                 c2wPBody = obj.currRefRobotPoseWcs;
% %                 p2wPBody = obj.prevRefRobotPoseWcs;
% %                 b2cP = GetPctBody2Cam(obj.coordSysAligner, 1);
% %                 p2cPBody = c2wPBody \ p2wPBody;
% %                 
% %                 pctCcs = (b2cP*p2cPBody)*Inv(b2cP);
            end
            pixInd = sub2ind(size(depthMap), round(ptIcs(:,2)), round(ptIcs(:,1)));
            ptCcsZ = depthMap(pixInd);
            intrMat1 = Get(obj.camModel, 'PinholeIntrMat', 1,obj.scaleLvl);
            intrMat2 = Get(obj.camModel, 'PinholeIntrMat', 2,obj.scaleLvl);
            if ~isempty(ptIcs)||~isempty(ptCcsZ)||~isempty(pctCcs)
                ResetLocalTrace(obj.featPtManager, ptIcs, ptCcsZ, pctCcs, intrMat1, intrMat2);
            else
                ptIcs = [-1, -1];
                ptCcsZ = -1;
                ResetLocalTrace(obj.featPtManager, ptIcs, ptCcsZ, [], intrMat1, intrMat2);
            end
        end
        
        
        function InsertKeyFrame2(obj, imgL, depthMap)
            
            % refresh Body error
%             obj.robotPosePredictor.prvBodyOrigErrRng = [-0,0];
            
            % Calculate depth map
            %             depthMap = GetDepthMap(obj, imgL, imgR);
            
            
            %             StoreDepthMap(obj.pointCloudManager,depthMap);
            
            % Extract feature points
            [topMargin, leftMargin, bottomMargin, rightMargin] = GetPredictMargin(obj.featPtTracker);
            predReginMask = zeros(size(depthMap));
            predReginMask(topMargin + 1 : Rows(depthMap) - bottomMargin, leftMargin + 1 : Cols(depthMap) - rightMargin) = 1;
            ptIcs = ExtractFeaturePoints(obj.featPtExtractor, imgL, predReginMask);
            
            % Get depth value for each feature point
            if (LocalTraceLength2(obj.featPtManager) < 2)
                pctCcs = [];
            else
                pctBcs = GetDeltaPose(obj, -1, 0, 'PointCoordTransformer');
                rotAng = RotAxang(pctBcs);
                transVec = T(pctBcs);
                if (abs(rotAng) < obj.configParam.zero_rotation_angle_thresh && norm(transVec) < obj.configParam.zero_translation_thresh)
                    pctCcs = [];
                else
                    pctCcs = BodyMoveCcs(obj.coordSysAligner, pctBcs, 1);
                end
            end
            pixInd = sub2ind(size(depthMap), round(ptIcs(:,2)), round(ptIcs(:,1)));
            ptCcsZ = depthMap(pixInd);
            intrMat1 = Get(obj.camModel, 'PinholeIntrMat', 1,obj.scaleLvl);
            intrMat2 = Get(obj.camModel, 'PinholeIntrMat', 2,obj.scaleLvl);
            if ~isempty(ptIcs)||~isempty(ptCcsZ)||~isempty(pctCcs)
                ResetLocalTrace2(obj.featPtManager, ptIcs, ptCcsZ, pctCcs, intrMat1, intrMat2);
            else
                ptIcs = [-1, -1];
                ptCcsZ = -1;
                ResetLocalTrace2(obj.featPtManager, ptIcs, ptCcsZ, [], intrMat1, intrMat2);
            end
        end
        
        
        
        
        
        function ExtendPointCould(obj)
            intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,1);
            numKeyframeStored = size(obj.pointCloudManager.keyFrameDepthMaps,3);
            keyIds = GetLastNKeyFrameInd(obj,numKeyframeStored);
            poseVec = obj.poseWcsList(keyIds,:);
            b2cP = GetPctBody2Cam(obj.coordSysAligner, 1);
            [lastKeyCcsXYZ, validFlag] = MergePointMap_AverageInverseDepthMethod(obj.pointCloudManager, intrMat, poseVec, obj.coordSysAligner);
            lastKeyInd = GetLastKeyFrameInd(obj);
            keyPose = obj.poseWcsList(lastKeyInd,:);
            k2wP = WcsPoseVecToPct(obj, keyPose);
            k2wP.transformMat(1:3,4) = 1000*k2wP.transformMat(1:3,4);
            mask = GetPointCloudMask(obj.pointCloudManager, lastKeyCcsXYZ);
            if ~all(mask(:))
                densePointCouldXYZ = k2wP*Inv(b2cP)*lastKeyCcsXYZ';
                densePointCouldXYZ = densePointCouldXYZ';
                StorePointCloud(obj.pointCloudManager, densePointCouldXYZ, mask)
            end
        end
        
        function ExtendPointCloud_OpticalFlow1D(obj)
            if Rows(obj.poseWcsList) >=2
                intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,1);
                b2cP = GetPctBody2Cam(obj.coordSysAligner, 1);
                c2wPBody = WcsPoseVecToPct(obj, obj.poseWcsList(end,:));
                p2wPBody = WcsPoseVecToPct(obj, obj.poseWcsList(end - 1,:));
                p2wPBody.transformMat(1:3,4) = 1000*p2wPBody.transformMat(1:3,4);
                c2wPBody.transformMat(1:3,4) = 1000*c2wPBody.transformMat(1:3,4);
                p2cPBody = c2wPBody \ p2wPBody;
                p2cPCam = (b2cP*p2cPBody)*Inv(b2cP);
                
                MergePointMap_OpticalFlow1D(obj.pointCloudManager, obj.prevImgL, obj.currImgL,...
                    intrMat, p2cPCam, b2cP, p2wPBody, c2wPBody);
            end
        end
        
        function [robotPoseWcs] = TrackAndLocalize(obj)
            % Track active feature points
            intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,1);
            [prevFeatPtList, ptCcsZ] = GetActiveFeatures(obj.featPtManager);
            if ~isempty(prevFeatPtList)
                [predPtIcs, inTrackFlag] = TrackFeaturePoints(obj.featPtTracker, obj.prevImgL, obj.currImgL, prevFeatPtList, [], intrMat, GetPctBody2Cam(obj.coordSysAligner));
                ExtendLocalTrace(obj.featPtManager, predPtIcs, inTrackFlag);
                %                 figure(1122), UpdateLocalTracePlot(obj.featPtManager, obj.currImgL);
                
                %                 [~, ~, keyPtCcsZ] = GetFeaturePairs(obj.featPtManager, 'key', 'last');
                %                 if sum(inTrackFlag) > obj.configParam.min_num_pt_for_calculate &&  (Rows(prevFeatPtList) - sum(inTrackFlag))/Rows(prevFeatPtList) < 0.2
                if 1 %sum(inTrackFlag) > obj.configParam.min_num_pt_for_calculate
                    %                     figure(1122), UpdateLocalTracePlot(obj.featPtManager, obj.currImgL);
                    
                    p2cBodySensorRotAng = obj.currRefRobotRotAngWcs - obj.prevRefRobotRotAngWcs;
                    
                    try
                        p2cVisionRotAng = PredictRotateAngle(obj,p2cBodySensorRotAng);
                        deltaVision2BodySensor = abs(rad2deg(p2cVisionRotAng-p2cBodySensorRotAng));
                        debug.debug_900x_PredictRotateAngle(obj,intrMat,prevFeatPtList,predPtIcs, obj.DEBUG_FLAG & false)
                        debug.debug_905x_CheckVisionAndBodyP2CRotAng(deltaVision2BodySensor)
                        %                     obj.deltaAngle(end + 1) = rad2deg(p2cVisionRotAng-p2cBodySensorRotAng);
                        %                     disp('p2c Ang is')
                        %                     disp(deltaVision2BodySensor)
                        %                     figure(773377), plot(obj.deltaAngle)
                        if deltaVision2BodySensor > 10.5
                            theta = p2cBodySensorRotAng;
                        else
                            theta = p2cVisionRotAng;
                        end
                        %% theta -> cur - prv
                        [k2cRotAngGrids, k2cCamTransDirAngGrids, inPolygonFlag, curBodyOrigUncertPolygonBcsKey] = PredictRobotPose(obj, theta, norm(T(obj.currRefRobotPoseWcs \ obj.prevRefRobotPoseWcs)));
                        [k2cRotAngCandidates, k2cCamTransDirAngCandidates, projScore] = EstimateInAngleSpace(obj, k2cRotAngGrids, k2cCamTransDirAngGrids, inPolygonFlag);
                        debug.debug_912x_CheckAlphaThetaAndCandidateAngle(k2cRotAngGrids, k2cCamTransDirAngGrids, inPolygonFlag, k2cRotAngCandidates, k2cCamTransDirAngCandidates, obj.DEBUG_FLAG & true)
                        %                     [k2cRotAng, k2cCamTransDirAng, k2cScaleFactor, validFlag] = EstimateScaleFactor(obj, k2cRotAngCandidates, k2cCamTransDirAngCandidates, 1, projScore);
                        validFlag = (projScore>0.9)';
                        k2cRotAng = k2cRotAngCandidates(1);
                        k2cCamTransDirAng = k2cCamTransDirAngCandidates(1);
                        k2cScaleFactor = -1;
                        [~, ~, activeFeatInd] = GetActiveFeatures(obj.featPtManager, 'last');
                        % SaveInLier(obj.featPtManager, 'last', activeFeatInd, validFlag);
                        inFlag = obj.featPtManager.localTrace.ptIcsX(:,end) ~= -1;
                        cur = [obj.featPtManager.localTrace.ptIcsX(inFlag,end) obj.featPtManager.localTrace.ptIcsY(inFlag,end)];
                        prv = [obj.featPtManager.localTrace.ptIcsX(inFlag,end-1) obj.featPtManager.localTrace.ptIcsY(inFlag,end-1)];
                        key = [obj.featPtManager.localTrace.ptIcsX(inFlag,1) obj.featPtManager.localTrace.ptIcsY(inFlag,1)];
                        debug.debug_906x_CheckVisionAndBodyK2CRotAng(obj,k2cRotAng,p2cBodySensorRotAng, obj.DEBUG_FLAG & true);
                        debug.debug_907x_CheckVisionAndBodyK2CCamTransDirAng(obj,k2cCamTransDirAng, obj.DEBUG_FLAG & true);
                        robotPoseWcs = EstimateLocation(obj, double(k2cRotAng), double(k2cCamTransDirAng), k2cScaleFactor, curBodyOrigUncertPolygonBcsKey);
                        debug.debug_909x_keyFrameBCS_PolygonAndBodyTraceInteractgion(obj,curBodyOrigUncertPolygonBcsKey,robotPoseWcs, obj.DEBUG_FLAG & true)
                        if 0
                            %                     reTrackFlag = setdiff(find(obj.featPtManager.localTrace.ptIcsX(:,end-1) ~= -1),activeFeatInd(validFlag));
                            reTrackFlag = setdiff(find(obj.featPtManager.localTrace.ptIcsX(:,end-1) ~= -1),activeFeatInd);
                            ptPrv = [obj.featPtManager.localTrace.ptIcsX(:,end-1) obj.featPtManager.localTrace.ptIcsY(:,end-1)];
                            [curPt, flagCur,fundMat,fundMatKey] = ReTraceEpiLine(obj, robotPoseWcs,ptPrv,obj.featPtManager.localTrace.ptIcsX(:,end-1) ~= -1);
                            %                         [~,~,errL,errR] = ShowEpipoleLine3(fundMat,obj.prevImgL,obj.currImgL,prv,cur,0);
                            [~,~,errL,errR] = ShowEpipoleLine3(fundMatKey,obj.prevImgL,obj.currImgL,key,cur,0);
                            validFlag = validFlag & max(abs([errL' errR'])')' < 1.5;
                            reTrackId = intersect(reTrackFlag,find(flagCur));
                            [curPoint, ~, ~] = GetActiveFeatures(obj.featPtManager, 'last');
                            if obj.keyFrameFlagList(end-1) == 1
                                figure(135),clf,imshow(obj.currImgL);hold on;plot(curPoint(validFlag,1),curPoint(validFlag,2),'or');plot(curPoint(~validFlag,1),curPoint(~validFlag,2),'xy');
                            end
                            
                            inFlag = obj.featPtManager.localTrace.ptIcsX(:,end) ~= -1;
                            key = [obj.featPtManager.localTrace.ptIcsX(inFlag,1) obj.featPtManager.localTrace.ptIcsY(inFlag,1)];
                            cur = [obj.featPtManager.localTrace.ptIcsX(inFlag,end) obj.featPtManager.localTrace.ptIcsY(inFlag,end)];
                            prv = [obj.featPtManager.localTrace.ptIcsX(inFlag,end-1) obj.featPtManager.localTrace.ptIcsY(inFlag,end-1)];
                            %                         figure(136),clf;[~,~,errL,errR] = ShowEpipoleLine3(fundMatKey,obj.prevImgL,obj.currImgL,key(validFlag,:),cur(validFlag,:));figure(137),clf,plot(abs([errL' errR']));
                            figure(136),clf;[~,~,errL,errR] = ShowEpipoleLine3(fundMatKey,obj.prevImgL,obj.currImgL,key,cur,0);figure(137),clf,plot(abs([errL' errR']));
                            %                         figure(138),clf;[~,~,errL,errR] = ShowEpipoleLine3(fundMat,obj.prevImgL,obj.currImgL,prv(validFlag,:),cur(validFlag,:));figure(139),clf,plot(abs([errL' errR']));
                            errFlag = max(abs([errL' errR'])')' >= 1.5; % & validFlag & ;
                            curr = cur(validFlag,:);
                            figure(134),clf,imshow(obj.currImgL);hold on;plot(curPoint(validFlag,1),curPoint(validFlag,2),'or');plot(curPoint(~validFlag,1),curPoint(~validFlag,2),'xy');plot(cur(errFlag,1),cur(errFlag,2),'+g');drawnow;
                            obj.featPtManager.localTrace.ptIcsX(activeFeatInd(~validFlag),end) = -1;
                            obj.featPtManager.localTrace.ptIcsY(activeFeatInd(~validFlag),end) = -1;
                            
                            fprintf(sprintf('### retracked points: %d/%d, AngleSpace inlier ratio: %d/%d ###\n\n\n',length(reTrackId),sum(obj.featPtManager.localTrace.ptIcsX(:,end) ~= -1), sum(validFlag),length(validFlag)));
                            obj.featPtManager.localTrace.ptIcsX(reTrackId,end) = curPt(reTrackId,1);
                            obj.featPtManager.localTrace.ptIcsY(reTrackId,end) = curPt(reTrackId,2);
                            SaveInLier(obj.featPtManager, 'last', find(obj.featPtManager.localTrace.ptIcsX(:,end) ~= -1), true(length(find(obj.featPtManager.localTrace.ptIcsX(:,end) ~= -1)),1));
                        else
                            
                            
                        end
                    catch
                        
                        c2wTrans = obj.currRefRobotPoseWcs;
                        p2wTrans = obj.prevRefRobotPoseWcs;
                        c2pTrans = p2wTrans\c2wTrans;
                        p2wTransV = WcsPoseVecToPct(obj, obj.poseWcsList(end,:));
                        c2wTransV = p2wTransV*c2pTrans;
                        curloc = T(c2wTransV);
                        deltaAngle1 = obj.currRefRobotRotAngWcs - obj.prevRefRobotRotAngWcs;
                        robotPoseWcs = [curloc(1),curloc(3),obj.poseWcsList(end,3)+deltaAngle1];
                        
                    end
                    if sum(inTrackFlag) > obj.configParam.min_num_pt_for_calculate2
                        fprintf(sprintf('### AngleSpace inlier ratio: %d/%d ###\n\n\n', sum(validFlag),length(validFlag)));
                        SaveInLier(obj.featPtManager, 'last', activeFeatInd, validFlag);
                    else
                        [~, ~, activeFeatInd] = GetActiveFeatures(obj.featPtManager, 'last');
                        inLierFlag = false(Rows(activeFeatInd),1);
                        SaveInLier(obj.featPtManager, 'last', activeFeatInd, inLierFlag);
                    end
                    %                     if rad2deg(abs(obj.currRefRobotRotAngWcs - robotPoseWcs(3))) > obj.robotPosePredictor.configParam.prev2curr_rotate_angle_err_range
                    %                         c2wTrans = obj.currRefRobotPoseWcs;
                    %                         p2wTrans = obj.prevRefRobotPoseWcs;
                    %                         c2pTrans = p2wTrans\c2wTrans;
                    %                         p2wTransV = WcsPoseVecToPct(obj, obj.poseWcsList(end,:));
                    %                         c2wTransV = p2wTransV*c2pTrans;
                    %                         curloc = T(c2wTransV);
                    %                         deltaAngle = obj.currRefRobotRotAngWcs - obj.prevRefRobotRotAngWcs;
                    %                         robotPoseWcs = [curloc(1),curloc(3),obj.poseWcsList(end,3)+deltaAngle];
                    %                         [~, ~, activeFeatInd] = GetActiveFeatures(obj.featPtManager, 'last');
                    %                         inLierFlag = false(Rows(activeFeatInd),1);
                    %                         SaveInLier(obj.featPtManager, 'last', activeFeatInd, inLierFlag);
                    %                     end
                else
                    %                     c2wTrans = obj.currRefRobotPoseWcs;
                    %                     p2wTrans = obj.prevRefRobotPoseWcs;
                    %                     c2pTrans = p2wTrans\c2wTrans;
                    %                     p2wTransV = WcsPoseVecToPct(obj, obj.poseWcsList(end,:));
                    %                     c2wTransV = p2wTransV*c2pTrans;
                    %                     curloc = T(c2wTransV);
                    %                     deltaAngle = obj.currRefRobotRotAngWcs - obj.prevRefRobotRotAngWcs;
                    %                     robotPoseWcs = [curloc(1),curloc(3),obj.poseWcsList(end,3)+deltaAngle];
                    %                     [~, ~, activeFeatInd] = GetActiveFeatures(obj.featPtManager, 'last');
                    %                     inLierFlag = false(Rows(activeFeatInd),1);
                    %                     SaveInLier(obj.featPtManager, 'last', activeFeatInd, inLierFlag);
                end
            else
                c2wTrans = obj.currRefRobotPoseWcs;
                p2wTrans = obj.prevRefRobotPoseWcs;
                c2pTrans = p2wTrans\c2wTrans;
                p2wTransV = WcsPoseVecToPct(obj, obj.poseWcsList(end,:));
                c2wTransV = p2wTransV*c2pTrans;
                curloc = T(c2wTransV);
                deltaAngle1 = obj.currRefRobotRotAngWcs - obj.prevRefRobotRotAngWcs;
                robotPoseWcs = [curloc(1),curloc(3),obj.poseWcsList(end,3)+deltaAngle1];
                [~, ~, activeFeatInd] = GetActiveFeatures(obj.featPtManager, 'last');
                inLierFlag = false(Rows(activeFeatInd),1);
                SaveInLier(obj.featPtManager, 'last', activeFeatInd, inLierFlag);
            end
            if sum(inTrackFlag) <= obj.configParam.min_num_pt_for_calculate2
                c2wTrans = obj.currRefRobotPoseWcs;
                p2wTrans = obj.prevRefRobotPoseWcs;
                c2pTrans = p2wTrans\c2wTrans;
                p2wTransV = WcsPoseVecToPct(obj, obj.poseWcsList(end,:));
                c2wTransV = p2wTransV*c2pTrans;
                curloc = T(c2wTransV);
                deltaAngle1 = obj.currRefRobotRotAngWcs - obj.prevRefRobotRotAngWcs;
                robotPoseWcs = [curloc(1),curloc(3),obj.poseWcsList(end,3)+deltaAngle1];
                [~, ~, activeFeatInd] = GetActiveFeatures(obj.featPtManager, 'last');
                inLierFlag = false(Rows(activeFeatInd),1);
                SaveInLier(obj.featPtManager, 'last', activeFeatInd, inLierFlag);
            end
        end
        
        function [robotPoseWcs] = PureRotationTrackAndLocalize(obj)
            global  DRAWPOLYGON ANGLEONLY SHOWPOLYGON USEGOLDENDISP NEWTRACKER NewOnlyP2 NewTrackerWithProb Switch4to3 ...
                FigBase FrmNumWithGloden FrmNumWithGloden0 ForceGoldenK2cRef probPath FrmNumWithGloden1  featInd addPtInterval ...
                EnableDump
            %             intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,1);
%             EnableDump = true;
            forceWrong = EnableDump;
            
            intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,obj.scaleLvl);
            [prevFeatPtList, ptCcsZ] = GetActiveFeatures(obj.featPtManager);
            [prevFeatPtList0, ptCcsZ0] = GetActiveFeatures(obj.featPtManager,1);
            
            
            if size(obj.featPtManager.localTrace.ptIcsX,2) == 1
                %% 20200408
                obj.setting = Setting([]);
                obj.lookUpTables = LookUpTables([]);
                
                obj.setting2 = Setting2([]);
                obj.lookUpTables2 = LookUpTables2([]);
                try
                    obj.lookUpTables.winSize = obj.setting.winSize;
                    obj.lookUpTables.angSize = obj.setting.angSize;
                    obj.lookUpTables.angMove = obj.setting.angMove;
                catch
                    %% 20200408
                    saghk = 1;
                end
                obj.lookUpTables.configParam.winSize = obj.setting.winSize;
                obj.lookUpTables.configParam.angSize = obj.setting.angSize;
                obj.lookUpTables.configParam.angMove = obj.setting.angMove;
                
                obj.modals = Modals([]);
                obj.points = Points([]);
                obj.traceWithWeights = TraceWithWeights([]);
                obj.angleModalOrg = zeros(1,obj.setting.angleBin);
                obj.modals = Modals.Modal(obj.setting.configParam, size(obj.featPtManager.localTrace.ptIcsX,1));
                
                
                
                
                
                obj.modals2 = Modals2([]);
                obj.points2 = Points2([]);
                obj.traceWithWeights2 = TraceWithWeights2([]);
                obj.angleModalOrg = zeros(1,obj.setting.angleBin);
                obj.modals2 = Modals2.Modal(obj.setting2.configParam, size(obj.featPtManager.localTrace.ptIcsX,1));
                
                DepthProbility = [];
                DepthId = [];
                DispRefineUse__ = [];
                NewDispRng = [];
                newDispStep = [];
                ProbZTmpReSamp = [];
                
                
%                 DepthProbility_2 = [];
%                 DepthId_2 = [];
%                 DispRefineUse___2 = [];
%                 NewDispRng_2 = [];
%                 newDispStep_2 = [];
%                 ProbZTmpReSamp_2 = [];
                
            else
                %                 DepthProbility = obj.keyProbZ{end,4};
                try
                    DepthProbility = obj.keyProbZ{end,24};
                    DepthId = obj.keyProbZ{end,2};
                    DispRefineUse__ = obj.keyProbZ{end,25};
                    NewDispRng = obj.keyProbZ{end,26};
                    newDispStep = obj.keyProbZ{end,27};
                    ProbZTmpReSamp = obj.keyProbZ{end,28};
                    
                    DepthProbility = ProbZTmpReSamp;
                    
                    
                    
%                     DepthProbility_2 =  obj.trace2.keyProbZ{end,24};
%                     DepthId_2 = obj.trace2.keyProbZ{end,2};
%                     DispRefineUse___2 = obj.trace2.keyProbZ{end,25};
%                     NewDispRng_2 = obj.trace2.keyProbZ{end,26};
%                     newDispStep_2 = obj.trace2.keyProbZ{end,27};
%                     ProbZTmpReSamp_2 = obj.trace2.keyProbZ{end,28};
%                     
%                     
%                     DepthProbility_2 = ProbZTmpReSamp_2;
                    
                    
                catch
                    dvwnjk = 1;
                end
                %                 ptCcsZ0 = ptCcsZ;
            end
            
            
            
% %             if size(obj.featPtManager.localTrace2.ptIcsX,2) == 1
% % %                 obj.setting = Setting([]);
% % %                 obj.lookUpTables = LookUpTables([]);
% % %                 obj.lookUpTables.winSize = obj.setting.winSize;
% % %                 obj.lookUpTables.angSize = obj.setting.angSize;
% % %                 obj.lookUpTables.angMove = obj.setting.angMove;
% % %                 obj.lookUpTables.configParam.winSize = obj.setting.winSize;
% % %                 obj.lookUpTables.configParam.angSize = obj.setting.angSize;
% % %                 obj.lookUpTables.configParam.angMove = obj.setting.angMove;
% % %                 obj.modals = Modals([]);
% % %                 obj.points = Points([]);
% % %                 obj.traceWithWeights = TraceWithWeights([]);
% % %                 obj.angleModalOrg = zeros(1,obj.setting.angleBin);
% % %                 obj.modals = Modals.Modal(obj.setting.configParam, size(obj.featPtManager.localTrace.ptIcsX,1));
% %                 DepthProbility = [];
% %                 DepthId = [];
% %                 DispRefineUse__ = [];
% %                 NewDispRng = [];
% %                 newDispStep = [];
% %                 ProbZTmpReSamp = [];
% %                 
% %                 
% %                 DepthProbility_2 = [];
% %                 DepthId_2 = [];
% %                 DispRefineUse___2 = [];
% %                 NewDispRng_2 = [];
% %                 newDispStep_2 = [];
% %                 ProbZTmpReSamp_2 = [];
% %                 
% %             else
% %                 %                 DepthProbility = obj.keyProbZ{end,4};
% %                 try
% %                     DepthProbility = obj.keyProbZ{end,24};
% %                     DepthId = obj.keyProbZ{end,2};
% %                     DispRefineUse__ = obj.keyProbZ{end,25};
% %                     NewDispRng = obj.keyProbZ{end,26};
% %                     newDispStep = obj.keyProbZ{end,27};
% %                     ProbZTmpReSamp = obj.keyProbZ{end,28};
% %                     
% %                     DepthProbility = ProbZTmpReSamp;
% %                     
% %                     
% %                     
% %                     DepthProbility_2 =  obj.trace2.keyProbZ{end,24};
% %                     DepthId_2 = obj.trace2.keyProbZ{end,2};
% %                     DispRefineUse___2 = obj.trace2.keyProbZ{end,25};
% %                     NewDispRng_2 = obj.trace2.keyProbZ{end,26};
% %                     newDispStep_2 = obj.trace2.keyProbZ{end,27};
% %                     ProbZTmpReSamp_2 = obj.trace2.keyProbZ{end,28};
% %                     
% %                     
% %                     DepthProbility_2 = ProbZTmpReSamp_2;
% %                     
% %                     
% %                 catch
% %                     dvwnjk = 1;
% %                 end
% %                 %                 ptCcsZ0 = ptCcsZ;
% %             end
            

win_width_bak = obj.featPtTracker.configParam.win_width;
win_height_bak = obj.featPtTracker.configParam.win_height;
max_level_bak = obj.featPtTracker.configParam.max_level;
 

if 0 % ~exist(fullfile(probPath,'lkConfig.txt'))
    fid1 = fopen(fullfile(probPath, 'lkConfig.txt'),'w');
    fprintf(fid1, 'width: %d, height: %d, level: %d', win_width_bak, win_height_bak, max_level_bak);
    fclose(fid1);
end
            
            if ~isempty(prevFeatPtList) && ~all(ptCcsZ<0)
                if ~NEWTRACKER
                    
                    AngleModalOrg = obj.angleModalOrg;
                    AngleProbility = [];
                    if 1
                        [curPredPtIcs, inTrackFlag] = TrackFeaturePoints(obj.featPtTracker, obj.prevImgL, obj.currImgL, prevFeatPtList, [], intrMat, GetPctBody2Cam(obj.coordSysAligner));
                        if sum(inTrackFlag) < 50
                            
%                             win_width_bak = obj.featPtTracker.configParam.win_width;
%                             win_height_bak = obj.featPtTracker.configParam.win_height;
%                             max_level_bak = obj.featPtTracker.configParam.max_level;
                            
                            
                            obj.featPtTracker.configParam.win_width = 11;
                            obj.featPtTracker.configParam.win_height = 11;
                            obj.featPtTracker.configParam.max_level = 5;
                            
                            [curPredPtIcs, inTrackFlag] = TrackFeaturePoints(obj.featPtTracker, obj.prevImgL, obj.currImgL, prevFeatPtList, [], intrMat, GetPctBody2Cam(obj.coordSysAligner));
                            
                            
                            obj.featPtTracker.configParam.win_width = win_width_bak;
                            obj.featPtTracker.configParam.win_height = win_height_bak;
                            obj.featPtTracker.configParam.max_level = max_level_bak;
                        end
                    else
                        [curPredPtIcs,inTrackFlag] = NewTrackingOld(obj, size(obj.featPtManager.localTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs, AngleModalOrg);
                    end
% %                      AngleModalOrg = obj.angleModalOrg;
% %                     AngleProbility = [];
                else
                    [curPredPtIcs1, inTrackFlag1] = TrackFeaturePoints(obj.featPtTracker, obj.prevImgL, obj.currImgL, prevFeatPtList, [], intrMat, GetPctBody2Cam(obj.coordSysAligner));
                    AngleModalOrg = obj.angleModalOrg;
                    AngleProbility = [];
                    
                    
                    
                     
                    
                    
                    
                    if ~NewOnlyP2 
                        if NewTrackerWithProb
                            [curPredPtIcs,inTrackFlag,curPredPtIcs0,inTrackFlag0, weightRaw] = NewTracking(obj, size(obj.featPtManager.localTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs, AngleModalOrg, obj.keyFrameDepth, obj.prvDepthVisual, obj.depthVisual,DepthProbility, DepthId, AngleProbility,0);
                        else
                            [curPredPtIcs,inTrackFlag,curPredPtIcs0,inTrackFlag0, weightRaw] = NewTracking(obj, size(obj.featPtManager.localTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs, AngleModalOrg, obj.keyFrameDepth, obj.prvDepthVisual, obj.depthVisual,[], [], [],0);
                        end
                    else
                        if NewTrackerWithProb
                            [curPredPtIcs,inTrackFlag,curPredPtIcs0,inTrackFlag0] = NewTracking_new(obj, size(obj.featPtManager.localTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs, AngleModalOrg, obj.keyFrameDepth, obj.prvDepthVisual, obj.depthVisual,DepthProbility, DepthId, AngleProbility,0);
                        else
                            [curPredPtIcs,inTrackFlag,curPredPtIcs0,inTrackFlag0] = NewTracking_new(obj, size(obj.featPtManager.localTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs, AngleModalOrg, obj.keyFrameDepth, obj.prvDepthVisual, obj.depthVisual,[], [], [],0);
                        end
                    end
                    if 0
                        figure,showMatchedFeatures(obj.prevImgL,obj.currImgL,prevFeatPtList(inTrackFlag,:),curPredPtIcs(inTrackFlag,:))
                        figure,showMatchedFeatures(obj.prevImgL,obj.currImgL,prevFeatPtList(inTrackFlag1,:),curPredPtIcs1(inTrackFlag1,:))
                    end
                end
%                 curPredPtIcs(:,1) = curPredPtIcs(:,1) + 0.01;
                
                ExtendLocalTrace(obj.featPtManager, curPredPtIcs, inTrackFlag);
                if size(obj.featPtManager.localTrace.ptIcsX,2) <= 2
                    %                                         figure(109),clf;imshow(imgCurL);hold on;plot(PixCur(:,1),PixCur(:,2),'or');plot(pix0_GT(:,1),pix0_GT(:,2),'.g');plot([pix0_GT(:,1) PixCur(validXYZAll,1)]',[pix0_GT(:,2) PixCur(validXYZAll,2)]','-c');
                    obj.featPtManager.localTrace.xGT = obj.featPtManager.localTrace.ptIcsX(:,1);
                    obj.featPtManager.localTrace.yGT = obj.featPtManager.localTrace.ptIcsY(:,1);
                end
                obj.featPtManager.localTrace.xGT = [obj.featPtManager.localTrace.xGT repmat(-1,size(obj.featPtManager.localTrace.xGT,1),1)];
                obj.featPtManager.localTrace.yGT = [obj.featPtManager.localTrace.yGT repmat(-1,size(obj.featPtManager.localTrace.yGT,1),1)];
                % %                 figure(1122), UpdateLocalTracePlot(obj.featPtManager, obj.currImgL);
                if 1 %sum(inTrackFlag) > obj.configParam.min_num_pt_for_calculate
                    %                     figure(1122), UpdateLocalTracePlot(obj.featPtManager, obj.currImgL);
                    PNP_FLAG = true;
                    
                    [depthMapKeyVisual, dispMapKey] = GetDepthMap(obj, obj.keyFrameImgL, obj.keyFrameImgR);
                    dispMapKey(dispMapKey < 0) = nan;
                    depthMapKey = obj.keyFrameDepth;
                    depthMapKey(dispMapKey < 0) = -1;
                    
                    depthErrMap = zeros(size(dispMapKey));
                    
                    validInd = sub2ind(size(dispMapKey), round(obj.featPtManager.localTrace.ptIcsY(:,1)), round(obj.featPtManager.localTrace.ptIcsX(:,1)));
                    zList0 = depthMapKey(validInd);
                    zList00 = obj.keyFrameDepth(validInd);
                    dispList = dispMapKey(validInd);
                    
                    depListVisual = depthMapKeyVisual(validInd);
                    depListGT = depthMapKey(validInd);
                    depthMapKeyGT = obj.keyFrameDepthGT;
                    depListGT = depthMapKeyGT(validInd);
                    dispMatGT = intrMat(1,1).*norm(obj.camModel.transVec1To2)./depthMapKeyGT;
                    dispMatGT0 = dispMatGT;
                    dispMatGT(dispMatGT > max(dispMapKey(validInd))) = nan;
                    dispMatGT(dispMatGT < 0.5) = 0.5;
                    dispErr = dispMapKey - dispMatGT;
                    %                     dispErr(isnan(dispErr)) = 0;
                    dispErrList = dispErr(validInd);
                    iddisp1 = dispErrList > 0;
                    iddisp2 = dispErrList < 0;
                    vldFlag = (~isnan(depListGT) & ~isnan(dispList));
                    vldFlagGT = (~isnan(depListGT) & ~isnan(dispList))&(~isnan(dispErrList));
                    vldFlagAll = vldFlag & vldFlagGT;
                    
                    
                    depthErrMap(validInd(depListVisual > 0)) = depListVisual(depListVisual > 0) - depListGT(depListVisual > 0);
                    if 0
                        dispList1 = dispList-1;
                        dispList1(dispList1 < 0.5) = 0.5;
                        dispList2 = dispList+1;
                        z1 = intrMat(1,1).*obj.camModel.transVec1To2./dispList1;
                        z2 = intrMat(1,1).*obj.camModel.transVec1To2./dispList2;
                        ovlpFlag = (~(depListGT > z2 & depListGT < z1)) & (~isnan(depListGT) & ~isnan(dispList));
                        goodFlag = ((depListGT > z2 & depListGT < z1)) & (~isnan(depListGT) & ~isnan(dispList));
                        vldFlag = (~isnan(depListGT) & ~isnan(dispList));
                        vldFlagGT = (~isnan(depListGT) & ~isnan(dispList))&(~isnan(dispErrList));
                        vldFlagAll = vldFlag & vldFlagGT;
                        figure,imshow(obj.keyFrameImgL);hold on; plot((obj.featPtManager.localTrace.ptIcsX(ovlpFlag,1)),(obj.featPtManager.localTrace.ptIcsY(ovlpFlag,1)),'.r');
                        figure,imshow(obj.keyFrameImgL);hold on; plot((obj.featPtManager.localTrace.ptIcsX(goodFlag,1)),(obj.featPtManager.localTrace.ptIcsY(goodFlag,1)),'.r');
                        figure,plot(dispErrList(vldFlagGT));title('visualDisparity - gtDisparity');
                        idovfl = (abs(dispErrList) > 1);
                        figure,imshow(obj.keyFrameImgL);hold on; plot((obj.featPtManager.localTrace.ptIcsX(vldFlag & idovfl,1)),(obj.featPtManager.localTrace.ptIcsY(vldFlag & idovfl,1)),'.r');
                        
                    end
                    if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                        if 0
                            figure(99);clf;imshow(depthErrMap,[]);title('stereo - gt');
                        end
                        iddisp1 = dispErrList > 0 & vldFlagAll;
                        iddisp2 = dispErrList < 0 & vldFlagAll;
                        %                         figure(98);clf;imshow(obj.keyFrameImgL);hold on; hold on; plot((obj.featPtManager.localTrace.ptIcsX(iddisp1,1)),(obj.featPtManager.localTrace.ptIcsY(iddisp1,1)),'.r');
                        %                         plot((obj.featPtManager.localTrace.ptIcsX(iddisp2,1)),(obj.featPtManager.localTrace.ptIcsY(iddisp2,1)),'.b');legend('stereoDisp > gtDisp','stereoDisp < gtDisp');
                        %                         inputDir = fullfile(pwd, 'dispErr');
                        %                         saveas(gcf,fullfile(inputDir,sprintf('img_%05d.png',length(dir(fullfile(inputDir,'img_*.png')))+1)));
                        
                        dumpDir = fullfile(pwd, 'dump');
                        stereoPair = [obj.keyFrameImgL obj.keyFrameImgR];
                        %                        disparityGroundTruth = dispMatGT0;
                        disparityGroundTruth = dispMatGT;
                        if 0
                            save(fullfile(dumpDir,sprintf('stereoPair_%05d.mat',length(dir(fullfile(dumpDir,'stereoPair_*.mat')))+1)), 'stereoPair');
                            save(fullfile(dumpDir,sprintf('disparityGroundTruth_%05d.mat',length(dir(fullfile(dumpDir,'disparityGroundTruth_*.mat')))+1)), 'disparityGroundTruth');
                        end
                    end
                      
                    if 0
                        dumpDir = fullfile(pwd, 'dump');
                        stereoPair = [obj.keyFrameImgL obj.keyFrameImgR];
                        %                        disparityGroundTruth = dispMatGT0;
                        disparityGroundTruth = dispMatGT;
                        save(fullfile(dumpDir,sprintf('stereoPair_%05d.mat',length(dir(fullfile(dumpDir,'stereoPair_*.mat')))+1)), 'stereoPair');
                        save(fullfile(dumpDir,sprintf('disparityGroundTruth_%05d.mat',length(dir(fullfile(dumpDir,'disparityGroundTruth_*.mat')))+1)), 'disparityGroundTruth');
                    end
                    
                    if 0
                        dumpDir = fullfile(pwd, 'dump');
                        stereoPair = [obj.currImgL obj.currImgR];
                        %                        disparityGroundTruth = dispMatGT0;
                        dispMatGTCur = intrMat(1,1).*norm(obj.camModel.transVec1To2)./obj.depthGT;
                        disparityGroundTruth = dispMatGTCur;
                        if 0
                            save(fullfile(dumpDir,sprintf('stereoPair_%05d.mat',length(dir(fullfile(dumpDir,'stereoPair_*.mat')))+1)), 'stereoPair');
                            save(fullfile(dumpDir,sprintf('disparityGroundTruth_%05d.mat',length(dir(fullfile(dumpDir,'disparityGroundTruth_*.mat')))+1)), 'disparityGroundTruth');
                        else
                            save(fullfile(probPath,sprintf('stereoPair_%05d.mat',length(dir(fullfile(probPath,'stereoPair_*.mat')))+1)), 'stereoPair');
                            save(fullfile(probPath,sprintf('disparityGroundTruth_%05d.mat',length(dir(fullfile(probPath,'disparityGroundTruth_*.mat')))+1)), 'disparityGroundTruth');
                        end
                        
                    end
                    
                    
                    
                    imgPrvL = obj.prevImgL; imgPrvR = obj.prevImgR;
                    imgCurL = obj.currImgL; imgCurR = obj.currImgR;
                    %     PrvL = CurL;
                    %                     CurL = timeL(i+1);
                    %                     [~,id1] = min(abs(timeMat(:,4)-PrvL));
                    %                     [~,id2] = min(abs(timeMat(:,4)-CurL));
                    %                     inTrackFlag0 = inTrackFlag;
                    %                     PtList = predPtList;
                    %                     [predPtList, inTrackFlag] = TrackFeaturePoints2( imgPrvL, imgCurL, single(PtList),inTrackFlag0);
                    %                     featX(:,i+1) = predPtList(:,1);
                    %                     featY(:,i+1) = predPtList(:,2);
                    
                    
                    
                    
                    
                    if PNP_FLAG
                        k2cBodySensorRotAng = obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs;
                        if 0 %size(obj.poseWcsList,1) == 2
                            k2cBodySensorRotAng = -0.011777263303185;
                        end
                        k2cRef = k2cBodySensorRotAng;
                        ind = GetLastKeyFrameInd(obj);
                         
                        try
%                         if 1
                            %                             key2cVisionRotAng = PredictRotateAngle2(obj,k2cBodySensorRotAng,obj.frmStamp,obj.goldenPose,ind);
                            
                            
                            [k2cBodyRotAng, reprojError, activeFlag2] = RotationAngleEstimate_PNP(obj.configParam.reproj_sigma, obj.pureRotationAngPredictor,intrMat,obj.featPtManager,obj.coordSysAligner,k2cRef,obj.keyFrameFlagList,obj.frmStampList,obj.goldenPose);
                            %                             RefineZ(obj.pureRotationAngPredictor,intrMat,obj.featPtManager,obj.coordSysAligner,k2cRef,obj.keyFrameFlagList,obj.frmStampList,obj.goldenPose);
                            
% %                             if length(obj.accumP2CPNP) == 1
% %                                 obj.accumP2CPNP = [obj.accumP2CPNP;k2cBodyRotAng];
% %                             end
                            
                            
                            
                            deltaVision2BodySensor = abs(rad2deg(k2cBodyRotAng-k2cBodySensorRotAng));
                            if deltaVision2BodySensor > 200 %  0.3 % 200 %0.1
                                theta = k2cBodySensorRotAng;
                            else
                                theta = k2cBodyRotAng;
                            end
                            %             debug.debug_902x_PureRotationTest(theta,b2cPmat,keyCcsXYZ,intrMat,activeFlag,curPtIcs);
                            inLierFlag = DeoutLier_PNP(obj.pureRotationAngPredictor,theta, intrMat,obj.featPtManager,obj.coordSysAligner);
                            inLierFlag = activeFlag2;
                            
                            idd = find(activeFlag2);
                            %                             idd(reprojError <= 0.5) = [];
                            idd(reprojError <= 0) = [];
                            activeFlag3 = false(length(activeFlag2),1);
                            activeFlag3(idd) = true;
                            inLierFlag2 = activeFlag3;
                            
                            
                            [~, ~, activeFeatIndLast] = GetActiveFeatures(obj.featPtManager, 'last');
                            inLkTrace = find(inTrackFlag);
                            inlierId2 = find(inLierFlag2);
                            
                            inlierId = activeFeatIndLast(inLierFlag2);
                            
                            if 0
                                [princpPtL, princpPtR] = Get(obj.camModel, 'PinholePrincpPt', obj.scaleLvl);
                                L2R = [rodrigues(obj.camModel.rotVec1To2) obj.camModel.transVec1To2; 0 0 0 1];
                                
                                %                             intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,obj.scaleLvl);
                                
                                depthMapCurVisual = obj.depthVisual;
                                depthMapCurGT = obj.depthGT;
                                
                                if USEGOLDENDISP
                                    depthMapCur = depthMapCurGT;
                                else
                                    depthMapCur = depthMapCurVisual;
                                end
                                
                                dispMapCur = (intrMat(1,1).*norm(obj.camModel.transVec1To2)./depthMapCur) - (princpPtR(1) - princpPtL(1));
                                
                                dispMapCurGT = (intrMat(1,1).*norm(obj.camModel.transVec1To2)./depthMapCurGT) - (princpPtR(1) - princpPtL(1));
                                
                                dispMapCur(dispMapCur < 0) = nan; dispCurList_ = dispMapCur(validInd);
                                
                                idUnValid = find(isnan(zList) | isinf(zList) | isnan(dispList) | isinf(dispList) | isnan(dispCurList_)); % | isnan(dispGTTmp) | isinf(dispGTTmp));
                                inlierId = setdiff(inlierId0, idUnValid);
                            end
                            
                            
                            zList = obj.featPtManager.localTrace.ptCcsZ;
                            %                             intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,1);
                            intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,obj.scaleLvl);
                            baseline = norm(obj.camModel.transVec1To2);
                            b2c = obj.coordSysAligner.pctBody2Cam(1, 1).transformMat;
                            
                            
                            %                             if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                            %                                 obj.featPtManager.localTrace.probZ = [];
                            %                             end
                            %%
                            % % % %                              if 1
                            % % % %                                  [angOpt1, ~,  angRng1] = RefineZ(obj, k2cRef,k2cBodyRotAng, inlierId,inlierId2, theta);
                            % % % %                              else
                            % % % %                                  angOpt1 = 0.02;
                            % % % %                                  angRng1 = [-0.01 0.03];
                            % % % %                              end
                            % % % %                             obj.angOpt = [obj.angOpt; angOpt1];
                            % % % %                             obj.angRng = [obj.angRng; angRng1];
                            
                            reprojError = [];cntt = 0;PixCur = [];PixReproj = [];
                            validId = []; validXYZAll = []; Pix0 = []; vldReproj = [];
                            keyLength = size(obj.featPtManager.localTrace.ptIcsX,2);
                            if DRAWPOLYGON
                                if ~isempty(inlierId)
                                    if SHOWPOLYGON
                                        figure(110),clf;imshow(imgCurL);hold on;
                                    end
                                    
                                    Pix = [obj.featPtManager.localTrace.ptIcsX(inlierId,1) obj.featPtManager.localTrace.ptIcsY(inlierId,1)];
                                    ZTrue = zList(inlierId);
                                    
                                    
                                    metricPrevPtCcs = intrMat\HomoCoord(Pix',1);
                                    metricPrevPtCcs = normc(metricPrevPtCcs);
                                    scaleAll = ZTrue./metricPrevPtCcs(3,:)';
                                    %                                   scale0 = zTrue./metricPrevPtCcs(3,:)';
                                    keyCcsXYZAll = [repmat(scaleAll',3,1).*metricPrevPtCcs];
                                    
                                    %                                     figure(232);hold on;axis equal;
                                    for j = 1 : length(inlierId)
                                        pixCur = [obj.featPtManager.localTrace.ptIcsX(inlierId(j),keyLength) obj.featPtManager.localTrace.ptIcsY(inlierId(j),keyLength)];
                                        %                                         plot(pixCur(1),pixCur(2),'.r');
                                        pix = [obj.featPtManager.localTrace.ptIcsX(inlierId(j),1) obj.featPtManager.localTrace.ptIcsY(inlierId(j),1)];
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
                                        metricPrevPtCcs = intrMat\HomoCoord(pix',1);
                                        metricPrevPtCcs = normc(metricPrevPtCcs);
                                        scale1 = zRng./metricPrevPtCcs(3,:)';
                                        scale0 = zTrue./metricPrevPtCcs(3,:)';
                                        keyCcsXYZ = [scale1(1).*metricPrevPtCcs scale1(2).*metricPrevPtCcs scale0.*metricPrevPtCcs];
                                        try
                                            thetaRng = obj.configParam.theta_range + theta;
                                        catch
                                            thetaRng = deg2rad([-0.5 0.5]) + theta;
                                        end
                                        
                                        k2cBodyPmat0 = [roty(rad2deg(theta)) [0;0;0];0 0 0 1];
                                        k2cCamPmat0 = b2c*k2cBodyPmat0/b2c;
                                        homocurrCcsXYZ0 = k2cCamPmat0*HomoCoord(keyCcsXYZ(:,3),1);
                                        pix0 = pflat(intrMat*homocurrCcsXYZ0(1:3));
                                        %                                         Pix0 = [Pix0; pix0(1:2)'];
                                        reprojError(cntt,:) = norm([pix0(1:2)'] - [pixCur]);
                                        
                                        PixReproj = [PixReproj;[pix0(1) pix0(2)]];
                                        
                                        %                                         if inlierId(j) == 1222
                                        %                                             figure(232),plot([pix0(1)'] - [pixCur(1)],[pix0(2)'] - [pixCur(2)],'+');
                                        %                                         end
                                        vldReproj = [vldReproj; j];
                                        
                                        
                                        k2cBodyPmat1 = [roty(rad2deg(thetaRng(1))) [0;0;0];0 0 0 1];
                                        k2cCamPmat1 = b2c*k2cBodyPmat1/b2c;
                                        homocurrCcsXYZ1 = k2cCamPmat1*HomoCoord(keyCcsXYZ(:,1),1);
                                        pix1 = pflat(intrMat*homocurrCcsXYZ1(1:3));
                                        
                                        
                                        k2cBodyPmat2 = [roty(rad2deg(thetaRng(2))) [0;0;0];0 0 0 1];
                                        k2cCamPmat2 = b2c*k2cBodyPmat2/b2c;
                                        homocurrCcsXYZ2 = k2cCamPmat2*HomoCoord(keyCcsXYZ(:,1),1);
                                        pix2 = pflat(intrMat*homocurrCcsXYZ2(1:3));
                                        
                                        k2cBodyPmat3 = [roty(rad2deg(thetaRng(1))) [0;0;0];0 0 0 1];
                                        k2cCamPmat3 = b2c*k2cBodyPmat3/b2c;
                                        homocurrCcsXYZ3 = k2cCamPmat3*HomoCoord(keyCcsXYZ(:,2),1);
                                        pix3 = pflat(intrMat*homocurrCcsXYZ3(1:3));
                                        
                                        k2cBodyPmat4 = [roty(rad2deg(thetaRng(2))) [0;0;0];0 0 0 1];
                                        k2cCamPmat4 = b2c*k2cBodyPmat4/b2c;
                                        homocurrCcsXYZ4 = k2cCamPmat4*HomoCoord(keyCcsXYZ(:,2),1);
                                        pix4 = pflat(intrMat*homocurrCcsXYZ4(1:3));
                                        
                                        square = [pix1(1:2) pix2(1:2) pix4(1:2) pix3(1:2)  pix1(1:2)]';
                                        %          plot(square(:,1),square(:,2),'-b');
                                        %                                         plot(square([1 2],1),square([1 2],2),'-c'); plot(square([3 4],1),square([3 4],2),'-c');
                                        %                                         plot(square([1 4],1),square([1 4],2),'-b'); plot(square([2 3],1),square([2 3],2),'-b');
                                        %                                         plot(pix0(1),pix0(2),'.g');
                                        
                                        
                                        
                                        
                                        margLen = obj.configParam.polygon_margin;
                                        
                                        thetaPt1 = [square([1 4],:) [1 1]']; thetaPt2 = [square([2 3],:) [1 1]'];
                                        depthPt1 = [square([1 2],:) [1 1]']; depthPt2 = [square([3 4],:) [1 1]'];
                                        
                                        
                                        thetaLine1 = cross(thetaPt1(1,:), thetaPt1(2,:)); thetaLine2 = cross(thetaPt2(1,:), thetaPt2(2,:));
                                        depthLine1 = cross(depthPt1(1,:), depthPt1(2,:)); depthLine2 = cross(depthPt2(1,:), depthPt2(2,:));
                                        
                                        depthLine1 = depthLine1./norm(depthLine1(1:2)); depthLine2 = depthLine2./norm(depthLine2(1:2));
                                        %                                         [intersectPt,dist] = CalcPt2Plane2([depthLine1(1:2) 0],[square(1,:) 0],[pixCur 0]);
                                        %                                         dot(intersectPt, depthLine1);
                                        [retVal1, dist1] = Pt2Line(square(1,:), square(2,:), pixCur); [retVal2, dist2] = Pt2Line(pixCur, retVal1, pix0(1:2)');
                                        [retVal3, dist3] = Pt2Line(square(3,:), square(4,:), pixCur); [retVal4, dist4] = Pt2Line(pixCur, retVal3, pix0(1:2)');
                                        
                                        candi = [retVal2; retVal4];
                                        [~,iderr5] = min([dist2 dist4]);
                                        candi_ = candi(iderr5,:);
                                        
                                        
                                        err1 = dot([pixCur 1], thetaLine1); err2 = dot([pixCur 1], thetaLine2);
                                        thetaLineIntersect = cross(thetaLine1, thetaLine2);
                                        thetaLineIntersect = [thetaLineIntersect(1)/thetaLineIntersect(3) thetaLineIntersect(2)/thetaLineIntersect(3)];
                                        square2 = [square(1:4,:); thetaLineIntersect];
                                        if ANGLEONLY
                                            if 0
                                                dist11 = norm(thetaLineIntersect - thetaPt1(1,1:2));
                                                dist12 = norm(thetaLineIntersect - thetaPt1(2,1:2));
                                                %                                             k1 = -thetaLine1(1)/thetaLine1(2);
                                                dirVec = [(thetaPt1(1,1) - thetaLineIntersect(1)) (thetaPt1(1,2) - thetaLineIntersect(2))];
                                                dirVec = dirVec./norm(dirVec);
                                                if dist11 > dist12
                                                    thetaPt1_3 = thetaPt1(1,1:2) + margLen*dirVec;
                                                else
                                                    thetaPt1_3 = thetaPt1(2,1:2) + margLen*dirVec;
                                                end
                                                dist13 = norm(thetaLineIntersect - thetaPt1_3);
                                                
                                                dist21 = norm(thetaLineIntersect - thetaPt2(1,1:2));
                                                dist22 = norm(thetaLineIntersect - thetaPt2(2,1:2));
                                                %                                             k1 = -thetaLine1(1)/thetaLine1(2);
                                                dirVec2 = [(thetaPt2(1,1) - thetaLineIntersect(1)) (thetaPt2(1,2) - thetaLineIntersect(2))];
                                                dirVec2 = dirVec2./norm(dirVec2);
                                                if dist21 > dist22
                                                    thetaPt2_3 = thetaPt2(1,1:2) + margLen*dirVec2;
                                                else
                                                    thetaPt2_3 = thetaPt2(2,1:2) + margLen*dirVec2;
                                                end
                                                dist23 = norm(thetaLineIntersect - thetaPt2_3);
                                            else
                                                
                                                if 0
                                                    num1 = dot(thetaLine1, pix0);
                                                    num2 = dot(thetaLine1, [pixCur 1]);
                                                    num3 = dot(thetaLine2, pix0);
                                                    num4 = dot(thetaLine2, [pixCur 1]);
                                                else
                                                    num1 = dot(thetaLine1, pix0);
                                                    num2 = dot(thetaLine1, [candi_ 1]);
                                                    num3 = dot(thetaLine2, pix0);
                                                    num4 = dot(thetaLine2, [candi_ 1]);
                                                end
                                                %                                                 [intersect,dist] = CalcPt2Plane2(lineN,ptOnline,ptIso);
                                                
                                                if (sign(num1) == sign(num2) && sign(num3) == sign(num4))  % || err5 < obj.configParam.polygon_inlier_thresh
                                                    in = true; on = true;
                                                else
                                                    in = false; on = false;
                                                end
                                            end
                                            
                                            
                                            
                                            %                                             [in,on] = inpolygon(pixCur(1),pixCur(2),square2(1:5,1),square2(1:5,2));
                                        else
                                            [in,on] = inpolygon(pixCur(1),pixCur(2),square(1:4,1),square(1:4,2));
                                        end
                                        
                                        if 1
                                            if in || on
                                                
                                                validId = [validId; inlierId2(j)];
                                                validXYZAll = [validXYZAll;j];
                                                
                                                if inlierId(j) == 1222
                                                    Idd = j;
                                                end
                                                %                                                 plot(square2([1 4 5],1),square2([1 4 5],2),'-m'); plot(square2([2 3 5],1),square2([2 3 5],2),'-m');
                                                if SHOWPOLYGON
                                                    plot(square([1 2],1),square([1 2],2),'-c'); plot(square([3 4],1),square([3 4],2),'-c');
                                                    plot(square([1 4],1),square([1 4],2),'-b'); plot(square([2 3],1),square([2 3],2),'-b');
                                                    
                                                    plot([pixCur(1) retVal1(1)],[pixCur(2) retVal1(2)], '-xy');plot([pix0(1) retVal2(1)], [pix0(2) retVal2(2)], '-xy');plot([retVal1(1) retVal2(1)], [retVal1(2) retVal2(2)], '-xy');
                                                    plot([pixCur(1) retVal3(1)],[pixCur(2) retVal3(2)], '-xk');plot([pix0(1) retVal4(1)], [pix0(2) retVal4(2)], '-xk');plot([retVal3(1) retVal4(1)], [retVal3(2) retVal4(2)], '-xk');
                                                    
                                                    plot(pixCur(1),pixCur(2),'.r');
                                                    plot(pix0(1),pix0(2),'.g');
                                                end
                                                sdvhk = 1;
                                            else
                                                %                                                 plot(pixCur(1),pixCur(2),'.r');
                                                %                                                 plot(square2([1 4 5],1),square2([1 4 5],2),'-m'); plot(square2([2 3 5],1),square2([2 3 5],2),'-m');
                                                %                                                 plot(square([1 2],1),square([1 2],2),'-c'); plot(square([3 4],1),square([3 4],2),'-c');
                                                %                                                 plot(square([1 4],1),square([1 4],2),'-b'); plot(square([2 3],1),square([2 3],2),'-b');
                                                %                                                 plot(pix0(1),pix0(2),'.g');
                                                sadb = 1;
                                            end
                                        else
                                            validId = [validId; inlierId2(j)];
                                        end
                                    end
                                    
                                    b2cPmat = GetPctBody2Cam(obj.coordSysAligner, 1);
                                    k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cBodySensorRotAng),zeros(3,1));
                                    
                                    k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
                                    
                                    keyCcsXYZAll_ = keyCcsXYZAll(:,validXYZAll);
                                    %                                     keyCcsXYZAll_1222 = keyCcsXYZAll(:,Idd);
                                    if 1
                                        k2cT = -b2cPmat.transformMat(1:3,1:3)*k2cBodyPmat.transformMat(1:3,1:3)*b2cPmat.transformMat(1:3,1:3)'*b2cPmat.transformMat(1:3,4) + b2cPmat.transformMat(1:3,4);
                                        k2cCam = [k2cCamPmat.transformMat(1:3,1:3) k2cT;0 0 0 1];
                                        homocurrCcsXYZALL = k2cCam*HomoCoord(keyCcsXYZAll_,1);
                                        homocurrCcsXYZALL_ = k2cCam*HomoCoord(keyCcsXYZAll,1);
                                        %                                         homocurrCcsXYZALL_1222 = k2cCam*HomoCoord(keyCcsXYZAll_1222,1);
                                    else
                                        %                     k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
                                        homocurrCcsXYZ = k2cCamPmat.transformMat*HomoCoord(keyCcsXYZAll_',1);
                                    end
                                    %                                     pix0_GT = pflat(intrMat*homocurrCcsXYZALL(1:3,:))';
                                    %                                     pix0_GT_ = pflat(intrMat*homocurrCcsXYZALL_(1:3,:))';
                                    
                                    validInd = sub2ind(size(dispMapKey), round(obj.featPtManager.localTrace.ptIcsY(:,1)), round(obj.featPtManager.localTrace.ptIcsX(:,1)));
                                    depthMapKeyGT = obj.keyFrameDepthGT;
                                    depthListGT0 = depthMapKeyGT(validInd);
                                    depthListGT = depthListGT0(inlierId);
                                    
                                    
                                    metricPrevPtCcsGT = intrMat\HomoCoord(Pix',1);
                                    metricPrevPtCcsGT = normc(metricPrevPtCcsGT);
                                    scaleAllGT = depthListGT./metricPrevPtCcsGT(3,:)';
                                    XYZ = [repmat(scaleAllGT',3,1).*metricPrevPtCcsGT];
                                    
                                    homocurrCcsXYZALLGT = k2cCam*HomoCoord(XYZ(:,validXYZAll),1);
                                    homocurrCcsXYZALLGT_ = k2cCam*HomoCoord(XYZ,1);
                                    
                                    pix0_GT = pflat(intrMat*homocurrCcsXYZALLGT(1:3,:))';
                                    pix0_GT_ = pflat(intrMat*homocurrCcsXYZALLGT_(1:3,:))';
                                    
                                    
                                    
                                    
                                    %                                     k2cT = -b2cPmat.transformMat(1:3,1:3)*k2cBodyPmat.transformMat(1:3,1:3)*b2cPmat.transformMat(1:3,1:3)'*b2cPmat.transformMat(1:3,4) + b2cPmat.transformMat(1:3,4);
                                    %                                     k2cCam = [k2cCamPmat.transformMat(1:3,1:3) k2cT;0 0 0 1];
                                    %                                     homocurrCcsXYZALL = k2cCam*HomoCoord(keyCcsXYZAll_,1);
                                    %                                     homocurrCcsXYZALL_ = k2cCam*HomoCoord(keyCcsXYZAll,1);
                                    
                                    
                                    
                                    
                                    %                                     pix0_GT_1222 = pflat(intrMat*homocurrCcsXYZALL_1222(1:3,:))';
                                    
                                    %                                     figure(233);hold on;plot(pix0_GT_1222(1) - PixCur(Idd,1), pix0_GT_1222(2) - PixCur(Idd,2), '+');axis equal
                                    
                                    obj.accumP2CRef = [obj.accumP2CRef; obj.accumP2CRef(end) + obj.currRefRobotRotAngWcs - obj.prevRefRobotRotAngWcs];
                                    
                                    
%                                     b2c = obj.coordSysAligner.pctBody2Cam(1, 1).transformMat;
                                    
                                    T_B2C = b2c;
                                    
                                    
                                    r_cam = sqrt(T_B2C(1,4)^2+T_B2C(2,4)^2+T_B2C(3,4)^2);
                                    tx = T_B2C(1,4)/r_cam;
                                    ty = T_B2C(2,4)/r_cam;
                                    tz = T_B2C(3,4)/r_cam;
                                    
                                    
                                    try
                                        [LocalTraceList, validFeatId, traceLenList, pt2d_check, dataTmp, dispErrMat, dispErrExp, traceBatchList1111] = DetectTraceDeoutlier(obj, b2c, k2cRef, obj.currRefRobotRotAngWcs - obj.prevRefRobotRotAngWcs, r_cam, tx, ty, tz);
                                        %                                     [LocalTraceList, validFeatId, traceLenList, ~, dataTmp, ~, ~] = DetectTraceDeoutlier(obj, b2c, k2cRef, obj.currRefRobotRotAngWcs - obj.prevRefRobotRotAngWcs, r_cam, tx, ty, tz);
                                    catch
                                        kughgj = 1;
                                    end
%                                     traceIndex = 1;
                                    skjdf = 1;
                                    if 0
                                        prepareRefineZ;
                                        
                                        % % %                                     if size(LocalTraceList{1, 1}.ptIcsX,2) > 2 % ~isempty(DepthId)
                                        % % %
                                        % % %                                         DepthId = obj.keyProbZ{end,2};
                                        % % %                                         commonId = find(ismember(DepthId, LocalTrace.featId));
                                        % % %                                         LocalTrace.probZ = obj.keyProbZ{end,28}(commonId,:);
                                        % % %                                         DepthProbility = obj.keyProbZ{end,24};
                                        % % %
                                        % % %                                         DispRefineUse__ = obj.keyProbZ{end,25};
                                        % % %                                         NewDispRng = obj.keyProbZ{end,26};
                                        % % %                                         newDispStep = obj.keyProbZ{end,27};
                                        % % %                                         ProbZTmpReSamp = obj.keyProbZ{end,28};
                                        % % %
                                        % % %                                         DepthProbility = ProbZTmpReSamp;
                                        % % %                                     end
                                        
                                        
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
                                            objPervPtCcsZGolden1] = RefineZ(k2cRef_1_00, obj, k2cRef,k2cBodyRotAng, inlierId,inlierId2, theta, DepthProbility,  DepthId,  AngleModalOrg,  AngleProbility, prevFeatPtList0,ptCcsZ0, DispRefineUse__,NewDispRng,newDispStep,ProbZTmpReSamp, k2cBodyRotAng, keyFrameLen , disparity_resample_prob_threshold, ThetaNoise ,KeyFrameFlagList, ConfigParam,  imgCur , imgCurR,  imgKeyL,  imgKeyR,  imgPrvL,  imgPrvR,  LocalTrace,  KeyProbZ,  thetaRngMat, thetaProbMat,   CamModel ,  CoordSysAligner,   REFAngList ,  REFAngList3,  REFAngList4,   accumP2CTemp,  twoOnlyP2List,   thetaPlatform,  thetaPlatformDistribution,   poseWcsList,    ANGOpt,    ANGRng,  meanErrAndStd,   p2cErrList,  objPervPtCcsZGolden);
                                        afterRefineZ;
                                    else
                                        
                                        
                                        
                                        
                                        
                                        
                                        
                                        
                                        
                                        if 0  %1
                                            if length(LocalTraceList) >= 2
                                                LocalTraceDump = LocalTraceList{2,1};
                                                if 0
                                                    dumpDir = fullfile(pwd, 'dump');
                                                    stereoPairPrv = [obj.prevImgL obj.prevImgR];
                                                    stereoPair = [obj.currImgL obj.currImgR];
                                                    %                        disparityGroundTruth = dispMatGT0;
                                                    dispMatGTCur = intrMat(1,1).*norm(obj.camModel.transVec1To2)./obj.depthGT;
                                                    disparityGroundTruth = dispMatGTCur;
                                                end
                                                LocalTraceDump.prevImgL = [];
                                                LocalTraceDump.prevImgR = [];
                                                LocalTraceDump.currImgL = obj.currImgL;
                                                LocalTraceDump.currImgR = obj.currImgR;
                                                LocalTraceDump.prevDepthGT = [];
                                                LocalTraceDump.currDepthGT = obj.depthGT;
                                                LocalTraceDump.prevDepthSGBM = [];
                                                LocalTraceDump.currDepthSGBM = obj.depthVisual;
                                                keyFrameLen = size(LocalTraceDump.ptIcsX,2);
                                                LocalTraceDump.angGT = obj.accumP2CRef(end) -  obj.accumP2CRef(length(obj.accumP2CRef) - keyFrameLen + 0 + 1);
                                                
                                                
                                                
                                                
                                                if length(dir(fullfile(probPath,'LocalTraceDump_*.mat'))) == 0
                                                    LocalTraceDump.prevImgL = obj.prevImgL;
                                                    LocalTraceDump.prevImgR = obj.prevImgL;
                                                    LocalTraceDump.prevDepthGT = obj.prvDepthGT;
                                                    LocalTraceDump.prevDepthSGBM = obj.prvDepthVisual;
%                                                     save(fullfile(probPath,sprintf('LocalTraceDump_%05d.mat',length(dir(fullfile(probPath,'LocalTraceDump_*.mat')))+1)), 'LocalTraceDump');
                                                    %                                            save(fullfile(probPath,sprintf('stereoPair_%05d.mat',length(dir(fullfile(probPath,'stereoPair_*.mat')))+0)), 'stereoPair');
                                                end
                                                save(fullfile(probPath,sprintf('LocalTraceDump_%05d.mat',length(dir(fullfile(probPath,'LocalTraceDump_*.mat')))+1)), 'LocalTraceDump');
                                                
                                                %                                         save(fullfile(probPath,sprintf('stereoPair_%05d.mat',length(dir(fullfile(probPath,'stereoPair_*.mat')))+1)), 'stereoPair');
                                                %                                         save(fullfile(probPath,sprintf('disparityGroundTruth_%05d.mat',length(dir(fullfile(probPath,'disparityGroundTruth_*.mat')))+1)), 'disparityGroundTruth');
                                            end
                                        end
                                        
                                        
                                        
                                        
                                        
                                        tempProbBak = obj.tempProb;
                                        validTraceId = [];dltIdMat = []; dltIdAll = 0;
                                        
                                        %                        for zx = length(LocalTraceList) : -1: 1
                                        
                                        TraceBatchList = []; dispCompare = {}; % []; 
                                        dispCompare2 = {};
                                        
                                        if ~isempty(obj.TraceBatchListStack)
                                            TraceBatchListStackBefore = obj.TraceBatchListStack{end};
                                        else
                                            TraceBatchListStackBefore = [];
                                        end
                                        
                                        
                                        if strcmp(probPath(1), 'D') % EnableDump
                                            SaveDumpData(obj, LocalTraceList);
                                        else
                                            if EnableDump
                                                SaveDumpData(obj, LocalTraceList);
                                            end
                                        end
                                        if forceWrong
                                            sgdha;
                                        end
                                        
                                        for zx = 1 : length(LocalTraceList)
                                            traceIndex = zx;
                                            RefineZFlow;
                                            
                                            
                                            if size(LocalTraceDone.LocalTrace.ptIcsX, 2) == 2
                                                obj.accumP2CPNP3 = [obj.accumP2CPNP3; obj.accumP2CPNP3(end) + LocalTraceDone.angOpt(end)];
                                                
                                            end
                                            
                                            dltIdMat = [dltIdMat; dltId];
                                            if validTrace
                                                validTraceId = [validTraceId; zx];
                                            end
                                            TraceBatchList = [TraceBatchList; traceBatch];
                                        end
                                        
% % %                                         if 0
% % %                                             SaveDumpData(obj, LocalTraceList);
% % %                                         end
                                        
                                        minOptFrameLen = 5; 14; -5;  5; 300; 5;
                                          
                                        if ~isempty(TraceBatchListStackBefore)
                                            FinishedTrace = setdiff(TraceBatchListStackBefore, TraceBatchList);
                                            if ~isempty(FinishedTrace)
                                                for oo = 1 : length(FinishedTrace)
                                                    obj.dispErrExpStack3{FinishedTrace(oo), 8} = 1;  % 1 is out
                                                    LocalTraceDone_replay = obj.dispErrExpStack3{FinishedTrace(oo), 7};
                                                    
                                                    LocalTraceDone_replay.angOpt_old = LocalTraceDone_replay.angOpt;
                                                    
                                                    localTrace_gtAng = obj.dispErrExpStack3{FinishedTrace(oo), 5}(:,4);
                                                    
                                                    if size(LocalTraceDone_replay.LocalTrace.ptIcsX, 2) > minOptFrameLen     
                                                        
                                                        if 1
                                                            [ANGOptFinal,cfg_new] = ReplayZOpt(obj, LocalTraceDone_replay, localTrace_gtAng, FinishedTrace(oo), 0, [], []);
                                                            [ANGOptFinal2,cfg_gt] = ReplayZOpt(obj, LocalTraceDone_replay, localTrace_gtAng, FinishedTrace(oo), 1, ANGOptFinal, cfg_new);
                                                        end
                                                        
                                                        if 0
                                                            CalcGoldenTracking(obj, intrMat, obj.dispErrExpStack3);
                                                            
                                                            batch2check = 26;
                                                            LocalTraceDone_replay22 = obj.dispErrExpStack3{batch2check, 7};
                                                            localTrace_gtAng22 = obj.dispErrExpStack3{batch2check, 5}(:,4);
                                                            ReplayZOpt(obj, LocalTraceDone_replay22, localTrace_gtAng22,batch2check, 0, [], []);

                                                            try
                                                                AA = cell2mat(cfg_new.bbbOpt);
                                                                BB = cell2mat(cfg_gt.bbbOpt);
                                                                AA = [zeros(1,size(AA,2)); AA];
                                                                BB = [zeros(1,size(BB,2)); BB];
                                                                CC = abs(rad2deg(diff(AA) - diff(BB)));
                                                                [maxCC, indCC] = max(CC);
                                                                figure(4),subplot(3,2,5);cla;plot(CC);
                                                            catch
                                                                dsgkjh = 1;
                                                            end
                                                        end
                                                        
                                                        
                                                        
                                                        
                                                        LocalTraceDone_replay.ANGOptFinal = ANGOptFinal;
                                                        
                                                        LocalTraceDone_replay.cfg_new = cfg_new;
                                                        LocalTraceDone_replay.cfg_gt = cfg_gt;
                                                        
                                                        
                                                        
                                                        LocalTraceDone_replay.localTrace_gtAng2 = localTrace_gtAng; % obj.dispErrExpStack3{FinishedTrace(oo), 5}(:,4);
                                                              
                                                        if 0
                                                            if 0 % 20200327
                                                                LocalTraceDone_replay.gtTheta = localTrace_gtAng;
                                                            else
                                                                LocalTraceDone_replay.gtTheta = ANGOptFinal2;
                                                            end
                                                        else
                                                             %% 20200327 change gt theta to theta with golden depth
                                                            try
                                                                LocalTraceDone_replay.gtTheta = ANGOptFinal2;
                                                            catch
                                                                LocalTraceDone_replay.gtTheta = localTrace_gtAng;
                                                            end
                                                        end
                                                        obj.dispErrExpStack3{FinishedTrace(oo), 7} = LocalTraceDone_replay;
                                                        obj.dispErrExpStack3{FinishedTrace(oo), 9} = 1;  
                                                        
                                                    else
                                                        obj.dispErrExpStack3{FinishedTrace(oo), 9} = 0;
                                                        
                                                    end
                                               end
                                            end
                                        end
                                        
                                        
                                        
                                        
                                        obj.TraceBatchListStack = [obj.TraceBatchListStack; {TraceBatchList}];
                                        
                                        
                                        if 1
                                            [dispa0,dispb0] = hist(cell2mat((dispCompare2(:,3))')', 1000);
                                            dispErrExp0 = -dot(dispb0, (dispa0./sum(dispa0)));
                                            
                                            
                                            [dispa1,dispb1] = hist(cell2mat((dispCompare2(:,1))')', 1000);
                                            dispErrExp1 = -dot(dispb1, (dispa1./sum(dispa1)));
                                            
                                            [dispa2,dispb2] = hist(cell2mat((dispCompare2(:,2))')', 1000);
                                            dispErrExp2 = -dot(dispb2, (dispa2./sum(dispa2)));
                                        else
                                            [dispa0,dispb0] = hist(cell2mat((dispCompare2(1,3))')', 1000);
                                            dispErrExp0 = -dot(dispb0, (dispa0./sum(dispa0)));
                                            
                                            
                                            [dispa1,dispb1] = hist(cell2mat((dispCompare2(1,1))')', 1000);
                                            dispErrExp1 = -dot(dispb1, (dispa1./sum(dispa1)));
                                            
                                            [dispa2,dispb2] = hist(cell2mat((dispCompare2(1,2))')', 1000);
                                            dispErrExp2 = -dot(dispb2, (dispa2./sum(dispa2)));
                                            
                                            
                                        end
                                        
                                        
                                        if isempty(obj.dispErrExpStack2)
                                            obj.dispErrExpStack2 = [obj.dispErrExpStack2; [dispErrExp0 dispErrExp1 dispErrExp2]; [dispErrExp0 dispErrExp1 dispErrExp2]];
                                        else
                                            obj.dispErrExpStack2 = [obj.dispErrExpStack2; [dispErrExp0 dispErrExp1 dispErrExp2]];
                                        end
%                                         obj.dispErrExpStack3  = [obj.dispErrExpStack3;  ];
                                         
                                         
                                         
                                         
                                         
                                        [obj.tempProb tempProbBak];
                                        
                                        
                                        
%                                         featInd = 3;
                                        
% % %                                         try
% % %                                             VisualLocalizer.DrawPic(probPath, featInd, length(obj.keyFrameFlagList), TraceBatchList, dataTmp.featSize, dataTmp.dd, dataTmp.ddGT, dataTmp.disparityRng)
% % %                                         catch
% % %                                             asbkj = 1;
% % %                                         end
                                        
                                        
                                        yfkkdry = 1;
                                        if 0
                                            for za = 1 :  length(LocalTraceList)
                                                angOptTemp =  [0;obj.tempProb{za,1}.angOpt];
                                                if za == 1 
                                                    angOptStackMat = zeros(length(LocalTraceList), length(angOptTemp) + 0);
                                                    goldenStackMat =  repmat(obj.accumP2CRef',length(LocalTraceList),1);
                                                    angOptStackMat(za,:) = angOptTemp';
                                                else
                                                    angOptStackMat(za,za:end) = angOptTemp';
                                                    goldenStackMat(za,za:end) = goldenStackMat(za,za:end) - goldenStackMat(za,za);
                                                    goldenStackMat(za,1:za) = 0;
                                                end
                                            end
                                            figure(5),clf;plot(rad2deg(angOptStackMat - goldenStackMat)');
                                        end
                                    end
                                    obj.angOptNew = [obj.angOptNew; angOpt1];
                                    ghjgjcj = 1;
                                    if 0
                                        %                                         try
                                        %                                             obj1 = obj;
                                        
%                                         falseNoInsertKey2 = false;
                                        InsertKeyThr0 = obj.configParam.feature_num_drop_rate_thresh;
                                        
                                        id_key_trace1 = find(obj.keyFrameFlagList);
                                        id_key_trace2 = find(obj.trace2.keyFrameFlagList);
                                        if ~obj.startTrace2
                                            doTrace1First = true;
                                            doTrace1Later = ~doTrace1First;
                                            FrmNumWithGloden = FrmNumWithGloden0;
                                        else
                                            if 1
                                                dropRatio_2_ = sum(obj.featPtManager.localTrace2.ptIcsX(:,end) < 0) ./ sum(obj.featPtManager.localTrace2.ptIcsX(:,1) > 0);
                                                keyFlagTemp =  dropRatio_2_ > obj.configParam.feature_num_drop_rate_thresh/1;
                                            else
                                                dropRatio_2_ = sum(obj.featPtManager.localTrace.ptIcsX(:,end) < 0) ./ sum(obj.featPtManager.localTrace.ptIcsX(:,1) > 0);
                                                keyFlagTemp =  dropRatio_2_ > obj.configParam.feature_num_drop_rate_thresh/2;
                                            end
                                            keyFlagTemp_ = obj.trace2.keyFrameFlagList;
                                            keyFlagTemp_(end) = keyFlagTemp;
                                            keyFlagTemp_ = [keyFlagTemp_; false];
                                            id_key_trace2_ = find(keyFlagTemp_); % [id_key_trace2;keyFlagTemp];
                                            
                                            if id_key_trace2_(end) > id_key_trace1(end)
                                                FrmNumWithGloden = max(FrmNumWithGloden, -10000);
                                                doTrace1First = true;
                                                doTrace1Later = ~doTrace1First;
                                            elseif id_key_trace2_(end) < id_key_trace1(end)
                                                FrmNumWithGloden = min(FrmNumWithGloden0, 10000);
                                                doTrace1First = false;
                                                doTrace1Later = ~doTrace1First;
                                                k2cRef_1 = obj.currRefRobotRotAngWcs - obj.trace2.prevRefRobotRotAngWcs;
                                                askbj = 1;
                                            else
%                                                 falseNoInsertKey2 = true;
                                                obj.configParam.feature_num_drop_rate_thresh = 1;
                                                 FrmNumWithGloden = min(FrmNumWithGloden0, 10000);
                                                doTrace1First = false;
                                                doTrace1Later = ~doTrace1First;
                                                k2cRef_1 = obj.currRefRobotRotAngWcs - obj.trace2.prevRefRobotRotAngWcs;
                                                askbj = 1;
                                                
                                            end
                                        end
                                        
                                        
                                        
                                        
                                         disparity_resample_prob_threshold = obj.configParam.disparity_resample_prob_threshold;
                                            ThetaNoise = obj.thetaNoise;
                                        
                                        
                                        if doTrace1First
                                            keyFrameLen = size(obj.featPtManager.localTrace.ptIcsX, 2);
                                            disparity_resample_prob_threshold = obj.configParam.disparity_resample_prob_threshold;
                                            ThetaNoise = obj.thetaNoise;
                                            KeyFrameFlagList = obj.keyFrameFlagList;
                                            ConfigParam = obj.configParam;
                                            depthMapCurVisual = obj.depthVisual;
                                            depthMapCurGT = obj.depthGT;
                                            imgCur = obj.currImgL;
                                            imgCurR = obj.currImgR;
                                            imgKeyL = obj.keyFrameImgL;
                                            imgKeyR = obj.keyFrameImgR;
                                            imgPrvL = obj.prevImgL;
                                            imgPrvR = obj.prevImgR;
                                            depthMapKey = obj.keyFrameDepth;
                                            depthMapKeyGT = obj.keyFrameDepthGT;
                                            LocalTrace = obj.featPtManager.localTrace;
                                            KeyProbZ = obj.keyProbZ;
                                            thetaRngMat = obj.thetaRngMat;
                                            thetaProbMat = obj.thetaProbMat;
                                            prvDepthGT = obj.prvDepthGT;
                                            prvDepthVisual = obj.prvDepthVisual;
                                            CamModel = obj.camModel;
                                            CoordSysAligner = obj.coordSysAligner;
                                            REFAngList = obj.refAngList;
                                            REFAngList3 = obj.refAngList3;
                                            REFAngList4 = obj.refAngList4;
                                            accumP2CTemp = obj.accumP2CTemp;
                                            twoOnlyP2List = obj.twoOnlyP2List;
                                            thetaPlatform = obj.thetaPlatform;
                                            thetaPlatformDistribution = obj.thetaPlatformDistribution;
                                            poseWcsList = obj.poseWcsList;
                                            ANGOpt = obj.angOpt;
                                            ANGRng = obj.angRng;
                                            meanErrAndStd = obj.meanErrAndStd;
                                            p2cErrList = obj.p2cErrList;
                                            objPervPtCcsZGolden = obj.PervPtCcsZGolden;
                                            
                                            k2cRef_1_00 = obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs;
                                            
                                            
                                            if ForceGoldenK2cRef  % FrmNumWithGloden > 20
                                                k2cRef = k2cRef_1_00;
                                            end
                                            
                                            
                                            
                                            if doTrace1First && keyFrameLen <= (FrmNumWithGloden + 1) && obj.startTrace2
                                                obj.useGoldenK2C = obj.useGoldenK2C + 1;
                                                obj.useGoldenK2C1 = [obj.useGoldenK2C1; length(obj.keyFrameFlagList)];
                                                asb = 1;
                                            end
                                            
                                            
                                            FigBase = 0;
                                            RefineZ(k2cRef_1_00, obj, k2cRef,k2cBodyRotAng, inlierId,inlierId2, theta, DepthProbility,  DepthId,  AngleModalOrg,  AngleProbility, prevFeatPtList0,ptCcsZ0, DispRefineUse__,NewDispRng,newDispStep,ProbZTmpReSamp, k2cBodyRotAng, keyFrameLen , disparity_resample_prob_threshold, ThetaNoise ,KeyFrameFlagList, ConfigParam,  depthMapCurVisual, depthMapCurGT, imgCur , imgCurR,  imgKeyL,  imgKeyR,  imgPrvL,  imgPrvR,  depthMapKey,  depthMapKeyGT, LocalTrace,  KeyProbZ,  thetaRngMat, thetaProbMat, prvDepthGT ,  prvDepthVisual,  CamModel ,  CoordSysAligner,   REFAngList ,  REFAngList3,  REFAngList4,   accumP2CTemp,  twoOnlyP2List,   thetaPlatform,  thetaPlatformDistribution,   poseWcsList,    ANGOpt,    ANGRng,  meanErrAndStd,   p2cErrList,  objPervPtCcsZGolden);
                                            %                                             RefineZ(k2cRef_1_00, obj, k2cRef,k2cBodyRotAng, inlierId,inlierId2, theta, DepthProbility,  DepthId,  AngleModalOrg,  AngleProbility, prevFeatPtList0,ptCcsZ0, DispRefineUse__,NewDispRng,newDispStep,ProbZTmpReSamp, k2cBodyRotAng, keyFrameLen , disparity_resample_prob_threshold, ThetaNoise ,KeyFrameFlagList, ConfigParam,  depthMapCurVisual, depthMapCurGT, imgCur , imgCurR,  imgKeyL,  imgKeyR,  imgPrvL,  imgPrvR,  depthMapKey,  depthMapKeyGT, LocalTrace,  KeyProbZ,  thetaRngMat, thetaProbMat, prvDepthGT ,  prvDepthVisual,  CamModel ,  CoordSysAligner,   REFAngList ,  REFAngList3,  REFAngList4,   accumP2CTemp,  twoOnlyP2List,   thetaPlatform,  thetaPlatformDistribution,   poseWcsList,    ANGOpt,    ANGRng,  meanErrAndStd,   p2cErrList,  objPervPtCcsZGolden);
                                            [angOpt1, inlierIdNew,  angRng1, pixKey ,pt2dCur, DispRefine, angOpt33, inFlag,thetaRngOut,thetaProbOut,angOptP2C, angOptK2C_, p2cOnlyP2,...
                                                keyFrameLen ,...
                                                disparity_resample_prob_threshold,...
                                                ThetaNoise ,...
                                                KeyFrameFlagList,...
                                                ConfigParam ,...
                                                depthMapCurVisual,...
                                                depthMapCurGT,...
                                                imgCur ,...
                                                imgCurR,...
                                                imgKeyL,...
                                                imgKeyR,...
                                                imgPrvL,...
                                                imgPrvR,...
                                                depthMapKey,...
                                                depthMapKeyGT,...
                                                LocalTrace,...
                                                KeyProbZ,...
                                                thetaRngMat,...
                                                thetaProbMat,...
                                                prvDepthGT ,...
                                                prvDepthVisual,...
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
                                                objPervPtCcsZGolden1] = RefineZ(k2cRef_1_00, obj, k2cRef,k2cBodyRotAng, inlierId,inlierId2, theta, DepthProbility,  DepthId,  AngleModalOrg,  AngleProbility,prevFeatPtList0,ptCcsZ0,DispRefineUse__,NewDispRng,newDispStep,ProbZTmpReSamp, k2cBodyRotAng,...
                                                keyFrameLen ,...
                                                disparity_resample_prob_threshold,...
                                                ThetaNoise ,...
                                                KeyFrameFlagList,...
                                                ConfigParam ,...
                                                depthMapCurVisual,...
                                                depthMapCurGT,...
                                                imgCur ,...
                                                imgCurR,...
                                                imgKeyL,...
                                                imgKeyR,...
                                                imgPrvL,...
                                                imgPrvR,...
                                                depthMapKey,...
                                                depthMapKeyGT,...
                                                LocalTrace,...
                                                KeyProbZ,...
                                                thetaRngMat,...
                                                thetaProbMat,...
                                                prvDepthGT ,...
                                                prvDepthVisual,...
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
                                                objPervPtCcsZGolden);  %, keyCcsXYZAll);
                                            
                                            
                                            FrmNumWithGloden = FrmNumWithGloden0;
                                            
                                            
                                            
                                            % % % %                                             if isempty(angOptP2C)
                                            % % % %                                                 angOptP2C = angOpt1;
                                            % % % %                                             end
                                            
                                            %                                             keyFrameLen = size(obj.featPtManager.localTrace.ptIcsX, 2);
                                            obj.configParam.disparity_resample_prob_threshold = disparity_resample_prob_threshold;
                                            %                                             ThetaNoise = obj.thetaNoise;
                                            obj.keyFrameFlagList = KeyFrameFlagList;
                                            %                                             ConfigParam = obj.configParam;
                                            obj.depthVisual = depthMapCurVisual;
                                            obj.depthGT = depthMapCurGT;
                                            obj.currImgL = imgCur;
                                            obj.currImgR = imgCurR;
                                            obj.keyFrameImgL = imgKeyL;
                                            obj.keyFrameImgR = imgKeyR;
                                            obj.prevImgL = imgPrvL;
                                            obj.prevImgR = imgPrvR;
                                            obj.keyFrameDepth = depthMapKey;
                                            obj.keyFrameDepthGT = depthMapKeyGT;
                                            obj.featPtManager.localTrace = LocalTrace;
                                            obj.keyProbZ = KeyProbZ;
                                            obj.thetaRngMat = thetaRngMat;
                                            obj.thetaProbMat = thetaProbMat;
                                            obj.prvDepthGT = prvDepthGT;
                                            obj.prvDepthVisual = prvDepthVisual;
                                            %                                             CamModel = obj.camModel;
                                            %                                             CoordSysAligner = obj.coordSysAligner;
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
                                            
                                            thetaPnP = pnpLite(obj.pureRotationAngPredictor,intrMat,obj.featPtManager,obj.coordSysAligner,k2cRef,obj.keyFrameFlagList,obj.frmStampList,obj.goldenPose,pixKey ,pt2dCur, DispRefine, obj.camModel, obj.scaleLvl);
                                        end
                                        
                                        
                                        
                                        % %                                             [depthMapPrv2, dispMap2] = GetDepthMap(obj, obj.prevImgL, obj.prevImgR);
                                        if 0
                                                try
                                                    obj.featPtManager.localTrace2.ptIcsX;
                                                catch
                                                    obj.localTrace2.ptIcsX = [];
                                                    obj.localTrace2.ptIcsY = [];
                                                    obj.localTrace2.ptCcsZ = [];
                                                    obj.localTrace2.xGT = [];
                                                    obj.localTrace2.yGT = [];
                                                    obj.localTrace2.probZ = [];
                                                    obj.localTrace2.sampleZ = [];
                                                end
                                            end
                                            
                                            if 0 % ~obj.startTrace2
                                                if keyFrameLen == 3
                                                    obj.startTrace2 = true;
                                                    insertKey2 = true;
                                                else
                                                    insertKey2 = false;
                                                end
                                            else
                                                if ~obj.startTrace2
                                                    if keyFrameLen > FrmNumWithGloden0 + 1
                                                        obj.startTrace2 = true;
                                                        insertKey2 = true;
                                                        if sum(obj.traceAng1(:,1)) == 1
                                                            obj.trace2KeyRefAng = obj.traceAng1(end-1,2);
                                                        else
                                                            svsasb = 1;
                                                        end
                                                    end
                                                else
                                                    
                                                    dropRatio_2 = sum(obj.featPtManager.localTrace2.ptIcsX(:,end) < 0) ./ sum(obj.featPtManager.localTrace2.ptIcsX(:,1) > 0);
                                                    if dropRatio_2 > obj.configParam.feature_num_drop_rate_thresh/1

% % %                                                     dropRatio_2 = sum(obj.featPtManager.localTrace.ptIcsX(:,end) < 0) ./ sum(obj.featPtManager.localTrace.ptIcsX(:,1) > 0);
% % %                                                     if dropRatio_2 > obj.configParam.feature_num_drop_rate_thresh/2
%                                                         obj.startTrace2 = true;
                                                        insertKey2 = true;
                                                        if sum(obj.traceAng1(:,1)) == 1
                                                            obj.trace2KeyRefAng = obj.traceAng1(end-1,2); 1;
                                                        else
                                                            sdhsdb = 1;
                                                            idf = find(obj.traceAng1(:,1));
                                                            obj.trace2KeyRefAng = obj.traceAng1(end-1,2) - obj.traceAng1(idf(end)-1,2);
                                                            checkErr_____ = obj.refAngList4(end) - obj.trace2KeyRefAng;
                                                        end
                                                    else
                                                        insertKey2 = false;
                                                    end
                                                end
                                            end
                                            obj.configParam.feature_num_drop_rate_thresh = InsertKeyThr0;
                                            
                                            
                                            
                                            
                                            
%                                             keyFrameLen_2 = size(obj.featPtManager.localTrace2.ptIcsX, 2);
%                                             disparity_resample_prob_threshold_2 = obj.configParam.disparity_resample_prob_threshold;
%                                             ThetaNoise_2 = obj.thetaNoise;
%                                             KeyFrameFlagList_2 = obj.trace2.keyFrameFlagList;
%                                             ConfigParam_2 = obj.configParam;
%                                             depthMapCurVisual_2 = obj.trace2.depthVisual;
%                                             depthMapCurGT_2 = obj.trace2.depthGT;
%                                             imgCur_2 = obj.trace2.currImgL;
%                                             imgCurR_2 = obj.trace2.currImgR;
%                                             imgKeyL_2 = obj.trace2.keyFrameImgL;
%                                             imgKeyR_2 = obj.trace2.keyFrameImgR;
%                                             imgPrvL_2 = obj.trace2.prevImgL;
%                                             imgPrvR_2 = obj.trace2.prevImgR;
%                                             depthMapKey_2 = obj.trace2.keyFrameDepth;
%                                             depthMapKeyGT_2 = obj.trace2.keyFrameDepthGT;
%                                             LocalTrace_2 = obj.featPtManager.localTrace2;
%                                             KeyProbZ_2 = obj.trace2.keyProbZ;
%                                             thetaRngMat_2 = obj.trace2.thetaRngMat;
%                                             thetaProbMat_2 = obj.trace2.thetaProbMat;
%                                             prvDepthGT_2 = obj.trace2.prvDepthGT;
%                                             prvDepthVisual_2 = obj.trace2.prvDepthVisual;
%                                             CamModel = obj.camModel;
%                                             CoordSysAligner = obj.coordSysAligner;
%                                             REFAngList_2 = obj.trace2.refAngList;
%                                             REFAngList3_2 = obj.trace2.refAngList3;
%                                             REFAngList4_2 = obj.trace2.refAngList4;
%                                             accumP2CTemp_2 = obj.trace2.accumP2CTemp;
%                                             twoOnlyP2List_2 = obj.trace2.twoOnlyP2List;
%                                             thetaPlatform_2 = obj.trace2.thetaPlatform;
%                                             thetaPlatformDistribution_2 = obj.trace2.thetaPlatformDistribution;
%                                             poseWcsList_2 = obj.poseWcsList;
%                                             ANGOpt_2 = obj.trace2.angOpt;
%                                             ANGRng_2 = obj.trace2.angRng;
%                                             meanErrAndStd_2 = obj.trace2.meanErrAndStd;
%                                             p2cErrList_2 = obj.trace2.p2cErrList;
%                                             objPervPtCcsZGolden_2 = obj.trace2.PervPtCcsZGolden;
                                            
                                            
                                            
  
                                            imgPrvL_2 = obj.prevImgL;
                                            imgPrvR_2 = obj.prevImgR;
                                            
                                            
                                            if ~obj.switchDepth
                                                [depthMap2, dispMap] = GetDepthMap(obj, imgPrvL_2, imgPrvR_2);
                                            else
                                                if ~isempty(obj.prvDepthGT)
                                                    depthMap2 = obj.prvDepthGT;
                                                else
                                                    [depthMap2, dispMap] = GetDepthMap(obj, imgPrvL_2, imgPrvR_2);
                                                end
                                            end
                                            
                                            if isempty(obj.trace2.keyFrameFlagList)
                                                if obj.startTrace2
                                                    obj.trace2.keyFrameFlagList = false(length(obj.keyFrameFlagList)-1,1);
                                                    obj.trace2.poseWcsList = zeros(length(obj.keyFrameFlagList)-1,3);
                                                    obj.trace2.keyFrameImgL = imgPrvL_2;
                                                    obj.trace2.keyFrameImgR = imgPrvR_2;
                                                    obj.trace2.keyFrameDepth = depthMap2;
                                                    obj.trace2.keyFrameDepthGT = obj.prvDepthGT;
%                                                     obj.trace2.depthVisual = depthMapCurVisual_2;
%                                                     obj.trace2.depthGT = depthMapCurGT_2;
                                                    obj.trace2.keyFrameFlagList(end) = true;
                                                    obj.trace2.keyFrameFlagList = [obj.trace2.keyFrameFlagList; false];
                                                    obj.trace2.keyRefRobotPoseWcs = obj.prevRefRobotPoseWcs;
                                                    obj.trace2.keyRefRobotRotAngWcs = obj.prevRefRobotRotAngWcs;%obj.trace2KeyRefAng; %
%                                                     obj.prevRefRobotPoseWcs.transformMat
%                                                     obj.trace2.poseWcsList(end,:) = [obj.prevRefRobotPoseWcs.transformMat(1,4) obj.prevRefRobotPoseWcs.transformMat(3,4) 0];
                                                    InsertKeyFrame2(obj, imgPrvL_2, depthMap2);
                                                end
                                            else
                                                if insertKey2
                                                    obj.trace2.keyFrameImgL = imgPrvL_2;
                                                    obj.trace2.keyFrameImgR = imgPrvR_2;
                                                    obj.trace2.keyFrameDepth = depthMap2;
                                                    obj.trace2.keyFrameDepthGT = obj.prvDepthGT;
%                                                     obj.trace2.depthVisual = depthMapCurVisual_2;
%                                                     obj.trace2.depthGT = depthMapCurGT_2;
                                                    obj.trace2.keyFrameFlagList(end) = true;
                                                    obj.trace2.keyFrameFlagList = [obj.trace2.keyFrameFlagList; false];
                                                    
%                                                    obj.trace2.currImgL = obj.currImgL;
%                                                    obj.trace2.currImgR = obj.currImgR;
                                                    
                                                    obj.trace2.keyRefRobotPoseWcs = obj.prevRefRobotPoseWcs;
                                                    obj.trace2.keyRefRobotRotAngWcs = obj.prevRefRobotRotAngWcs; %obj.trace2KeyRefAng; % 
                                                    
                                                    InsertKeyFrame2(obj, imgPrvL_2, depthMap2);
                                                else
                                                    obj.trace2.keyFrameFlagList = [obj.trace2.keyFrameFlagList; false];
                                                end
                                            end
                                            
                                            
                                            
                                            obj.trace2.currRefRobotPoseWcs = obj.currRefRobotPoseWcs; % = WcsPoseVecToPct(obj, refRobotPoseWcs);
                                            obj.trace2.currRefRobotRotAngWcs = obj.currRefRobotRotAngWcs; %  = refRobotPoseWcs(3) +  0;  %deg2rad(0.01*rand(1));
                                            obj.trace2.refRobotPosVec = obj.refRobotPosVec; %  = refRobotPoseWcs;
                                            
                                            
                                            obj.trace2.currImgL = obj.currImgL;
                                            obj.trace2.currImgR = obj.currImgR;
                                            obj.trace2.depthVisual = obj.depthVisual;
                                            obj.trace2.depthGT = obj.depthGT;
                                            
                                            if ~isempty(obj.trace2.keyFrameFlagList)
                                                if 1 % obj.trace2.keyFrameFlagList(end-1)
                                                    if 1 %~isempty(obj.trace2.refAngAll)
%                                                         obj.trace2.refAngAll = [obj.trace2.refAngAll; ];
                                                        obj.trace2.refAngAll = [obj.trace2.refAngAll; [obj.trace2.currRefRobotRotAngWcs - obj.trace2.prevRefRobotRotAngWcs]];
                                                    else
                                                        obj.trace2.refAngAll = obj.trace2.currRefRobotRotAngWcs - obj.trace2.keyRefRobotRotAngWcs;
                                                    end
                                                    if size(obj.featPtManager.localTrace2.ptIcsX,2) == 1
                                                        
                                                        
                                                        DepthProbility_2 = [];
                                                        DepthId_2 = [];
                                                        DispRefineUse___2 = [];
                                                        NewDispRng_2 = [];
                                                        newDispStep_2 = [];
                                                        ProbZTmpReSamp_2 = [];
                                                        
                                                    else
                                                        %                 DepthProbility = obj.keyProbZ{end,4};
                                                        try
                                                            
                                                            
                                                            
                                                            DepthProbility_2 =  obj.trace2.keyProbZ{end,24};
                                                            DepthId_2 = obj.trace2.keyProbZ{end,2};
                                                            DispRefineUse___2 = obj.trace2.keyProbZ{end,25};
                                                            NewDispRng_2 = obj.trace2.keyProbZ{end,26};
                                                            newDispStep_2 = obj.trace2.keyProbZ{end,27};
                                                            ProbZTmpReSamp_2 = obj.trace2.keyProbZ{end,28};
                                                            
                                                            
                                                            DepthProbility_2 = ProbZTmpReSamp_2;
                                                            
                                                            
                                                        catch
                                                            dvwnjk = 1;
                                                        end
                                                        %                 ptCcsZ0 = ptCcsZ;
                                                    end
                                                    
                                                    
                                                    disparity_resample_prob_threshold_2 = obj.configParam.disparity_resample_prob_threshold;
                                                    ThetaNoise_2 = obj.thetaNoise;
                                                    KeyFrameFlagList_2 = obj.trace2.keyFrameFlagList;
                                                    ConfigParam_2 = obj.configParam;
                                                    depthMapCurVisual_2 = obj.trace2.depthVisual;
                                                    depthMapCurGT_2 = obj.trace2.depthGT;
                                                    imgCur_2 = obj.trace2.currImgL;
                                                    imgCurR_2 = obj.trace2.currImgR;
                                                    imgKeyL_2 = obj.trace2.keyFrameImgL;
                                                    imgKeyR_2 = obj.trace2.keyFrameImgR;
                                                    imgPrvL_2 = obj.trace2.prevImgL;
                                                    imgPrvR_2 = obj.trace2.prevImgR;
                                                    depthMapKey_2 = obj.trace2.keyFrameDepth;
                                                    depthMapKeyGT_2 = obj.trace2.keyFrameDepthGT;
%                                                     LocalTrace_2 = obj.featPtManager.localTrace2;
                                                    KeyProbZ_2 = obj.trace2.keyProbZ;
                                                    thetaRngMat_2 = obj.trace2.thetaRngMat;
                                                    thetaProbMat_2 = obj.trace2.thetaProbMat;
                                                    prvDepthGT_2 = obj.trace2.prvDepthGT;
                                                    prvDepthVisual_2 = obj.trace2.prvDepthVisual;
                                                    CamModel = obj.camModel;
                                                    CoordSysAligner = obj.coordSysAligner;
                                                    REFAngList_2 = obj.trace2.refAngList;
                                                    REFAngList3_2 = obj.trace2.refAngList3;
                                                    REFAngList4_2 = obj.trace2.refAngList4;
                                                    accumP2CTemp_2 = obj.trace2.accumP2CTemp;
                                                    twoOnlyP2List_2 = obj.trace2.twoOnlyP2List;
                                                    thetaPlatform_2 = obj.trace2.thetaPlatform;
                                                    thetaPlatformDistribution_2 = obj.trace2.thetaPlatformDistribution;
                                                    poseWcsList_2 = obj.trace2.poseWcsList;
                                                    ANGOpt_2 = obj.trace2.angOpt;
                                                    ANGRng_2 = obj.trace2.angRng;
                                                    meanErrAndStd_2 = obj.trace2.meanErrAndStd;
                                                    p2cErrList_2 = obj.trace2.p2cErrList;
                                                    objPervPtCcsZGolden_2 = obj.trace2.PervPtCcsZGolden;
                                      
                                                    
                                                    
                                                    
                                                    AngleModalOrg_2 = obj.angleModalOrg;
                                                    AngleProbility_2 = [];
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
                                                    validPt2 = (featPtList(:,1) > 0 & featPtList(:,2) > 0 & depthList__ > 0);
                                                    %                                 inId_1 = find(depthList__ > 0);
% % % % % % % %                                                     featPtList2 = featPtList(validPt2, :);
% % % % % % % %                                                     depthList__ = depthList__(validPt2,:);
                                                    
                                                    
                                                    
                                                    
                                                    inTrackFlag0_0 = validPt2; % true(size(featPtList2,1),1);
                                                    
                                                    pnp_ang_est_max_margin = [deg2rad([-1 1]) obj.pureRotationAngPredictor.configParam.pure_rot_reproject_outlier_threshold];
                                                    
                                                    
                                                    
%                                                     [predPtList, inTrackFlag00] = TrackFeaturePoints3(obj.featPtTracker, imgPrvL_2, imgCur_2, featPtList,inTrackFlag00);
%                                                     [predPtList000, inTrackFlag000] = TrackFeaturePoints3(obj.featPtTracker, imgPrvL_2, imgCur_2, prevFeatPtList_22,true(size(prevFeatPtList_22,1),1));
                                                    [predPtList000, inTrackFlag000] = TrackFeaturePoints(obj.featPtTracker, imgPrvL_2, imgCur_2, prevFeatPtList_22, [], intrMat, GetPctBody2Cam(obj.coordSysAligner));

                                                    
                                                    
                                                    ExtendLocalTrace2(obj.featPtManager, predPtList000, inTrackFlag000);
                                                    
                                                    
                                                    id_ = find(ismember(featPtList, prevFeatPtList_22, 'rows'));
                                                    inTrackFlag00 = false(size(featPtList,1),1);
                                                    inTrackFlag00(id_(inTrackFlag000)) = true;
                                                    
                                                    predPtList = -1.*ones(size(featPtList,1),2);
                                                    predPtList(id_(inTrackFlag000),:) = predPtList000(inTrackFlag000,:);
                                                    
                                                    if 0
                                                        k2cRef_2 = obj.currRefRobotRotAngWcs - obj.trace2.keyRefRobotRotAngWcs;
                                                    else
                                                        if doTrace1First
                                                            if sum(obj.traceAng1(:,1)) == 1
                                                                k2cRef_2 = double(angOpt1 - obj.trace2KeyRefAng);
                                                            else
                                                                k2cRef_2 = double(angOpt1 - obj.trace2KeyRefAng);
                                                                sakbvj = 1;
                                                            end
                                                        else
                                                            FrmNumWithGloden = max(FrmNumWithGloden0, -1000);
                                                            k2cRef_2 = obj.trace2.currRefRobotRotAngWcs - obj.trace2.keyRefRobotRotAngWcs;
                                                        end
                                                    end
                                                    keyFrameLen_2 = size(obj.featPtManager.localTrace2.ptIcsX, 2);
                                                    LocalTrace_2 = obj.featPtManager.localTrace2;
%                                                     disparity_resample_prob_threshold_2 = obj.configParam.disparity_resample_prob_threshold;
%                                                     ThetaNoise_2 = obj.thetaNoise;
%                                                     KeyFrameFlagList_2 = obj.trace2.keyFrameFlagList;
%                                                     ConfigParam_2 = obj.configParam;
%                                                     depthMapCurVisual_2 = obj.trace2.depthVisual;
%                                                     depthMapCurGT_2 = obj.trace2.depthGT;
%                                                     imgCur_2 = obj.trace2.currImgL;
%                                                     imgCurR_2 = obj.trace2.currImgR;
%                                                     imgKeyL_2 = obj.trace2.keyFrameImgL;
%                                                     imgKeyR_2 = obj.trace2.keyFrameImgR;
%                                                     imgPrvL_2 = obj.trace2.prevImgL;
%                                                     imgPrvR_2 = obj.trace2.prevImgR;
%                                                     depthMapKey_2 = obj.trace2.keyFrameDepth;
%                                                     depthMapKeyGT_2 = obj.trace2.keyFrameDepthGT;
%                                                     LocalTrace_2 = obj.featPtManager.localTrace2;
%                                                     KeyProbZ_2 = obj.trace2.keyProbZ;
%                                                     thetaRngMat_2 = obj.trace2.thetaRngMat;
%                                                     thetaProbMat_2 = obj.trace2.thetaProbMat;
%                                                     prvDepthGT_2 = obj.trace2.prvDepthGT;
%                                                     prvDepthVisual_2 = obj.trace2.prvDepthVisual;
%                                                     CamModel = obj.camModel;
%                                                     CoordSysAligner = obj.coordSysAligner;
%                                                     REFAngList_2 = obj.trace2.refAngList;
%                                                     REFAngList3_2 = obj.trace2.refAngList3;
%                                                     REFAngList4_2 = obj.trace2.refAngList4;
%                                                     accumP2CTemp_2 = obj.trace2.accumP2CTemp;
%                                                     twoOnlyP2List_2 = obj.trace2.twoOnlyP2List;
%                                                     thetaPlatform_2 = obj.trace2.thetaPlatform;
%                                                     thetaPlatformDistribution_2 = obj.trace2.thetaPlatformDistribution;
%                                                     poseWcsList_2 = obj.trace2.poseWcsList;
%                                                     ANGOpt_2 = obj.trace2.angOpt;
%                                                     ANGRng_2 = obj.trace2.angRng;
%                                                     meanErrAndStd_2 = obj.trace2.meanErrAndStd;
%                                                     p2cErrList_2 = obj.trace2.p2cErrList;
%                                                     objPervPtCcsZGolden_2 = obj.trace2.PervPtCcsZGolden;
                                                    
                                                    
                                                    
                                                    
                                                    
                                                    
                                                    
                                                    
                                                    
                                                    
                                                    [k2cBodyRotAng_2,Err_2,inId] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin,intrMat,featPtListKey(inTrackFlag00,:),predPtList(inTrackFlag00,:),depthList__(inTrackFlag00,:), find(inTrackFlag00),true(sum(inTrackFlag00),1),b2c,double(k2cRef_2));
                                                    
                                                    %% useless code
                                                    if 0
                                                        idKeyTrace1 = find(KeyFrameFlagList);
                                                        idKeyTrace2 = find(KeyFrameFlagList_2);
                                                    end
                                                    
                                                    k2cRef_1_check = obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs;
                                                    k2cRef_2_check = obj.currRefRobotRotAngWcs - obj.trace2.keyRefRobotRotAngWcs;
                                                    
                                                    k2cRef_1_check_err = k2cRef_1_check - k2cRef;
                                                    k2cRef_2_check_err = k2cRef_2_check - k2cRef_2;
%                                                     checkK2C_2
                                                    
                                                    if 0
                                                        figure,plot(depthList__(id_) - ptCcsZ_22);
                                                        featPtList(inId,:);
                                                        predPtList(inId,:);
                                                        figure,subplot(1,2,1),showMatchedFeatures(imgKeyL, imgCur, featPtList(inTrackFlag00,:),predPtList(inTrackFlag00,:));subplot(1,2,2);showMatchedFeatures(imgKeyL, imgCur, featPtList(inId,:),predPtList(inId,:))
                                                        
                                                    end
                                                    
                                                    if 0
                                                        diffVec = setdiff([1:size(obj.featPtManager.localTrace2.ptIcsX,1)]', inId);
                                                        obj.featPtManager.localTrace2.ptIcsX(diffVec,end) = -1;
                                                        obj.featPtManager.localTrace2.ptIcsY(diffVec,end) = -1;
                                                    else
                                                        diffVec = setdiff([1:size(obj.featPtManager.localTrace2.ptIcsX,1)]', inId);
                                                        obj.featPtManager.localTrace2.ptIcsX(diffVec,end) = -1;
                                                        obj.featPtManager.localTrace2.ptIcsY(diffVec,end) = -1;
                                                    end
                                                    %                                                     SaveInLier2(obj.featPtManager, 'last', find(obj.featPtManager.localTrace2.ptIcsX(:,end) >0), true(length(find(obj.featPtManager.localTrace2.ptIcsX(:,end) >0 -1)),1));
                                                    
                                                    
%                                                     k2cRef_2 = 1;
                                                    
                                                    FigBase = 8800;
                                                    id_key_trace1 = find(obj.keyFrameFlagList);
                                                    id_key_trace2 = find(obj.trace2.keyFrameFlagList);
                                                    if id_key_trace2(end) > id_key_trace1(end)
                                                        FrmNumWithGloden = min(FrmNumWithGloden0, 10000);
                                                    else
                                                        FrmNumWithGloden = max(FrmNumWithGloden0, -1000);
                                                        askbj = 1;
                                                    end
                                                    
                                                    
                                                    k2cRef_2_00 = obj.trace2.currRefRobotRotAngWcs - obj.trace2.keyRefRobotRotAngWcs;
                                                    
                                                    if doTrace1First
                                                        FrmNumWithGloden = min(FrmNumWithGloden0, 1000);
                                                        if ForceGoldenK2cRef
                                                            k2cRef_2 = k2cRef_2_00;
                                                        end
                                                    else
                                                        FrmNumWithGloden = max(FrmNumWithGloden0, -1000);
                                                    end
                                                    
                                                    
                                                    
%                                                     if FrmNumWithGloden
%                                                     end
                                                    if doTrace1Later && keyFrameLen_2 <= (FrmNumWithGloden + 1)
                                                        obj.useGoldenK2C = obj.useGoldenK2C + 1;
                                                        idtrace2 = find(obj.traceAng1(:,1));
%                                                         obj.useGoldenK2C2 = [zeros(idtrace2(1)-1,2); obj.keyFrameFlagList(idtrace2(1):end)];
                                                        obj.useGoldenK2C2 = [obj.useGoldenK2C2; length(obj.keyFrameFlagList)];
                                                        
                                                        asb = 1;
                                                    end
                                                    % RefineZ(k2cRef_2_00, obj, k2cRef_2,k2cBodyRotAng_2, inId,inlierId2, theta, DepthProbility_2,  DepthId_2,  AngleModalOrg_2,  AngleProbility_2,prevFeatPtList0_2,ptCcsZ0_2,DispRefineUse___2,NewDispRng_2,newDispStep_2,ProbZTmpReSamp_2, k2cBodyRotAng_2,keyFrameLen_2 , disparity_resample_prob_threshold_2, ThetaNoise ,KeyFrameFlagList_2, ConfigParam_2,  depthMapCurVisual_2, depthMapCurGT_2, imgCur_2 , imgCurR_2,  imgKeyL_2,  imgKeyR_2,  imgPrvL_2,  imgPrvR_2,  depthMapKey_2,  depthMapKeyGT_2, LocalTrace_2,  KeyProbZ_2,  thetaRngMat_2, thetaProbMat_2, prvDepthGT_2 ,  prvDepthVisual_2,  CamModel ,  CoordSysAligner,   REFAngList_2 ,  REFAngList3_2,  REFAngList4_2,   accumP2CTemp_2,  twoOnlyP2List_2,   thetaPlatform_2,  thetaPlatformDistribution_2,   poseWcsList_2,    ANGOpt_2,    ANGRng_2,  meanErrAndStd_2,   p2cErrList_2,  objPervPtCcsZGolden_2);
                                                    [angOpt1_2, inlierIdNew_2,  angRng1_2, pixKey_2 ,pt2dCur_2, DispRefine_2, angOpt33_2, inFlag_2,thetaRngOut_2,thetaProbOut_2,angOptP2C_2, angOptK2C__2, p2cOnlyP2_2,...
                                                        keyFrameLen_2 ,...
                                                        disparity_resample_prob_threshold,...
                                                        ThetaNoise ,...
                                                KeyFrameFlagList_2,...
                                                ConfigParam_2 ,...
                                                depthMapCurVisual_2,...
                                                depthMapCurGT_2,...
                                                imgCur_2 ,...
                                                imgCurR_2,...
                                                imgKeyL_2,...
                                                imgKeyR_2,...
                                                imgPrvL_2,...
                                                imgPrvR_2,...
                                                depthMapKey_2,...
                                                depthMapKeyGT_2,...
                                                LocalTrace_2,...
                                                KeyProbZ_2,...
                                                thetaRngMat_2,...
                                                thetaProbMat_2,...
                                                prvDepthGT_2 ,...
                                                prvDepthVisual_2,...
                                                CamModel ,...
                                                CoordSysAligner,...
                                                REFAngList_2 ,...
                                                REFAngList3_2,...
                                                REFAngList4_2,...
                                                accumP2CTemp_2,...
                                                twoOnlyP2List_2,...
                                                thetaPlatform_2,...
                                                thetaPlatformDistribution_2,...
                                                poseWcsList_2,...
                                                ANGOpt_2,...
                                                ANGRng_2,...
                                                meanErrAndStd_2,...
                                                p2cErrList_2,...
                                                objPervPtCcsZGolden1_2] = RefineZ(k2cRef_2_00, obj, k2cRef_2,k2cBodyRotAng_2, inId,inlierId2, theta, DepthProbility_2,  DepthId_2,  AngleModalOrg_2,  AngleProbility_2,prevFeatPtList0_2,ptCcsZ0_2,DispRefineUse___2,NewDispRng_2,newDispStep_2,ProbZTmpReSamp_2, k2cBodyRotAng_2,...
                                                keyFrameLen_2 ,...
                                                disparity_resample_prob_threshold_2,...
                                                ThetaNoise ,...
                                                KeyFrameFlagList_2,...
                                                ConfigParam_2 ,...
                                                depthMapCurVisual_2,...
                                                depthMapCurGT_2,...
                                                imgCur_2 ,...
                                                imgCurR_2,...
                                                imgKeyL_2,...
                                                imgKeyR_2,...
                                                imgPrvL_2,...
                                                imgPrvR_2,...
                                                depthMapKey_2,...
                                                depthMapKeyGT_2,...
                                                LocalTrace_2,...
                                                KeyProbZ_2,...
                                                thetaRngMat_2,...
                                                thetaProbMat_2,...
                                                prvDepthGT_2 ,...
                                                prvDepthVisual_2,...
                                                CamModel ,...
                                                CoordSysAligner,...
                                                REFAngList_2 ,...
                                                REFAngList3_2,...
                                                REFAngList4_2,...
                                                accumP2CTemp_2,...
                                                twoOnlyP2List_2,...
                                                thetaPlatform_2,...
                                                thetaPlatformDistribution_2,...
                                                poseWcsList_2,...
                                                ANGOpt_2,...
                                                ANGRng_2,...
                                                meanErrAndStd_2,...
                                                p2cErrList_2,...
                                                objPervPtCcsZGolden_2); 

FrmNumWithGloden = FrmNumWithGloden0;


                                            thetaPnP_2 = pnpLite(obj.pureRotationAngPredictor,intrMat,obj.featPtManager,obj.coordSysAligner,k2cRef_2,obj.trace2.keyFrameFlagList,obj.frmStampList,obj.goldenPose,pixKey_2 ,pt2dCur_2, DispRefine_2, obj.camModel, obj.scaleLvl);
                                            
                                            
                                            
                                            
                                            
                                            
                                            obj.configParam.disparity_resample_prob_threshold = disparity_resample_prob_threshold;
                                            %                                             ThetaNoise = obj.thetaNoise;
                                            obj.trace2.keyFrameFlagList = KeyFrameFlagList_2;
                                            %                                             ConfigParam = obj.configParam;
                                            obj.trace2.depthVisual = depthMapCurVisual_2;
                                            obj.trace2.depthGT = depthMapCurGT_2;
                                            obj.trace2.currImgL = imgCur_2;
                                            obj.trace2.currImgR = imgCurR_2;
                                            obj.trace2.keyFrameImgL = imgKeyL_2;
                                            obj.trace2.keyFrameImgR = imgKeyR_2;
                                            obj.trace2.prevImgL = imgPrvL_2;
                                                    obj.trace2.prevImgR = imgPrvR_2;
                                                    obj.trace2.keyFrameDepth = depthMapKey_2;
                                                    obj.trace2.keyFrameDepthGT = depthMapKeyGT_2;
                                                    %% 
                                                    obj.featPtManager.localTrace2 = LocalTrace_2;
                                                    
                                                    obj.trace2.keyProbZ = KeyProbZ_2;
                                                    obj.trace2.thetaRngMat = thetaRngMat_2;
                                                    obj.trace2.thetaProbMat = thetaProbMat_2;
                                                    obj.trace2.prvDepthGT = prvDepthGT_2;
                                                    obj.trace2.prvDepthVisual = prvDepthVisual_2;
                                                    %                                             CamModel = obj.camModel;
                                                    %                                             CoordSysAligner = obj.coordSysAligner;
                                                    obj.trace2.refAngList = REFAngList_2;
                                                    obj.trace2.refAngList3 = REFAngList3_2;
                                                    obj.trace2.refAngList4 = REFAngList4_2;
                                                    obj.trace2.accumP2CTemp = accumP2CTemp_2;
                                                    obj.trace2.twoOnlyP2List = twoOnlyP2List_2;
                                                    obj.trace2.thetaPlatform = thetaPlatform_2;
                                                    obj.trace2.thetaPlatformDistribution = thetaPlatformDistribution_2;
                                                    obj.trace2.poseWcsList = poseWcsList_2;
                                                    obj.trace2.angOpt = ANGOpt_2;
                                                    obj.trace2.angRng = ANGRng_2;
                                                    obj.trace2.meanErrAndStd = meanErrAndStd_2;
                                                    obj.trace2.p2cErrList = p2cErrList_2;
                                                    obj.trace2.PervPtCcsZGolden = objPervPtCcsZGolden1_2;
                                                    
                                                end
                                            else
                                                %                                                 obj.trace2.refAngAll = obj.refRobotPosVec;
                                            end
                                            
                                            try
                                                obj.trace2.angOpt = [obj.trace2.angOpt; [angOpt1_2]];
                                                obj.trace2.angOptCheck = [obj.trace2.angOptCheck; angOptK2C__2];
                                                
                                                obj.trace2.angOpt3 = [obj.trace2.angOpt3; [angOpt33_2]];
                                                
                                                obj.trace2.accumP2C = [obj.trace2.accumP2C; obj.trace2.accumP2CTemp(end)];
                                                
                                                if size(obj.featPtManager.localTrace2.ptIcsX, 2) == 2
                                                    obj.trace2.refAngList4 = [];
                                                end
                                                
                                                if isempty(obj.trace2.angOptP2CList)
                                                    obj.trace2.angOptP2CList = [obj.trace2.angOptP2CList; angOpt1_2];
                                                else
                                                    if isempty(obj.trace2.refAngList4)
                                                        obj.trace2.angOptP2CList = [obj.trace2.angOptP2CList; angOpt1_2];
                                                    else
                                                        obj.trace2.angOptP2CList = [obj.trace2.angOptP2CList; obj.trace2.refAngList4(end) + angOptP2C_2];
                                                    end
                                                end
                                                
                                                
                                                if Switch4to3
                                                    obj.trace2.refAngList4 = [obj.trace2.refAngList3];
                                                else
                                                    obj.trace2.refAngList4 = [obj.trace2.refAngList4; angOpt1_2];
                                                end
                                                
                                                
                                                obj.trace2.angOptPnP = [obj.trace2.angOptPnP; [thetaPnP_2]];
                                                obj.trace2.angRng = [obj.trace2.angRng; angRng1_2];
                                                
                                                obj.trace2.thetaRngMat = [obj.trace2.thetaRngMat; thetaRngOut_2];
                                                
                                                obj.trace2.thetaProbMat = [obj.trace2.thetaProbMat; thetaProbOut_2];
                                                
                                                
                                                
                                                
                                                keyId = find(obj.trace2.keyFrameFlagList);
                                                keyIdList = obj.trace2.keyFrameFlagList(keyId(1)+0:end);
                                                KeyAng = [keyIdList [obj.trace2.angOpt;0]];
                                                
                                                
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
                                                
                                                errTrace2 = rad2deg([0;accumTrace2] - [0;cumsum(obj.trace2.refAngAll)]);
                                                KeyAngCat = [zeros(length(obj.trace2.keyFrameFlagList) - size(KeyAng,1),2);[KeyAng(:,1) deg2rad(errTrace2)]];
                                                obj.trace2.KeyAngCat = KeyAngCat;
                                                %                                         pulse = repmat(min(rad2deg(KeyAngCat(:,2))),size(KeyAngCat,1),1);
                                                %                                         pulse(idPulse) = max(errTrace2);
                                                %                                         figure(333),clf;plot(rad2deg(KeyAngCat(:,2)),'-b');hold on;plot(find(KeyAngCat(:,1)),rad2deg(KeyAngCat(find(KeyAngCat(:,1)),2)),'*g'); % plot(pulse,'k');
                                                %                                         drawnow;title('accum P2C')
                                                %                                         saveas(gcf,fullfile(probPath,sprintf('final_error_%05d.fig',0+3)));
                                                
                                                
                                                
                                            catch
                                                kjasv = 1;
                                            end
                                            
                                            if doTrace1Later
                                                FrmNumWithGloden = min(FrmNumWithGloden0, 1000);
                                                
                                                if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                                    obj.trace1KeyRefAng = accumTrace2(end-1); % obj.trace2.KeyAngCat(end-1,2);
                                                end
                                                
                                                k2cRef_1 = accumTrace2(end) - obj.trace1KeyRefAng;  %obj.trace2.KeyAngCat(end,2) - obj.trace1KeyRefAng;
                                                
                                                keyFrameLen = size(obj.featPtManager.localTrace.ptIcsX, 2);
                                                disparity_resample_prob_threshold = obj.configParam.disparity_resample_prob_threshold;
                                                ThetaNoise = obj.thetaNoise;
                                                KeyFrameFlagList = obj.keyFrameFlagList;
                                                ConfigParam = obj.configParam;
                                                depthMapCurVisual = obj.depthVisual;
                                                depthMapCurGT = obj.depthGT;
                                                imgCur = obj.currImgL;
                                                imgCurR = obj.currImgR;
                                                imgKeyL = obj.keyFrameImgL;
                                                imgKeyR = obj.keyFrameImgR;
                                                imgPrvL = obj.prevImgL;
                                                imgPrvR = obj.prevImgR;
                                                depthMapKey = obj.keyFrameDepth;
                                                depthMapKeyGT = obj.keyFrameDepthGT;
                                                LocalTrace = obj.featPtManager.localTrace;
                                                KeyProbZ = obj.keyProbZ;
                                                thetaRngMat = obj.thetaRngMat;
                                                thetaProbMat = obj.thetaProbMat;
                                                prvDepthGT = obj.prvDepthGT;
                                                prvDepthVisual = obj.prvDepthVisual;
                                                CamModel = obj.camModel;
                                                CoordSysAligner = obj.coordSysAligner;
                                                REFAngList = obj.refAngList;
                                                REFAngList3 = obj.refAngList3;
                                                REFAngList4 = obj.refAngList4;
                                                accumP2CTemp = obj.accumP2CTemp;
                                                twoOnlyP2List = obj.twoOnlyP2List;
                                                thetaPlatform = obj.thetaPlatform;
                                                thetaPlatformDistribution = obj.thetaPlatformDistribution;
                                                poseWcsList = obj.poseWcsList;
                                                ANGOpt = obj.angOpt;
                                                ANGRng = obj.angRng;
                                                meanErrAndStd = obj.meanErrAndStd;
                                                p2cErrList = obj.p2cErrList;
                                                objPervPtCcsZGolden = obj.PervPtCcsZGolden;
                                                
                                                k2cRef_1_00 = obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs;
                                                
                                                if ForceGoldenK2cRef % FrmNumWithGloden > 20
                                                    k2cRef_1 = k2cRef_1_00;
                                                end
                                                
                                                
                                                FigBase = 0;
                                                %                                             RefineZ(k2cRef_1_00, obj, k2cRef,k2cBodyRotAng, inlierId,inlierId2, theta, DepthProbility,  DepthId,  AngleModalOrg,  AngleProbility, prevFeatPtList0,ptCcsZ0, DispRefineUse__,NewDispRng,newDispStep,ProbZTmpReSamp, k2cBodyRotAng, keyFrameLen , disparity_resample_prob_threshold, ThetaNoise ,KeyFrameFlagList, ConfigParam,  depthMapCurVisual, depthMapCurGT, imgCur , imgCurR,  imgKeyL,  imgKeyR,  imgPrvL,  imgPrvR,  depthMapKey,  depthMapKeyGT, LocalTrace,  KeyProbZ,  thetaRngMat, thetaProbMat, prvDepthGT ,  prvDepthVisual,  CamModel ,  CoordSysAligner,   REFAngList ,  REFAngList3,  REFAngList4,   accumP2CTemp,  twoOnlyP2List,   thetaPlatform,  thetaPlatformDistribution,   poseWcsList,    ANGOpt,    ANGRng,  meanErrAndStd,   p2cErrList,  objPervPtCcsZGolden);
                                                [angOpt1, inlierIdNew,  angRng1, pixKey ,pt2dCur, DispRefine, angOpt33, inFlag,thetaRngOut,thetaProbOut,angOptP2C, angOptK2C_, p2cOnlyP2,...
                                                    keyFrameLen ,...
                                                    disparity_resample_prob_threshold,...
                                                    ThetaNoise ,...
                                                    KeyFrameFlagList,...
                                                    ConfigParam ,...
                                                    depthMapCurVisual,...
                                                    depthMapCurGT,...
                                                    imgCur ,...
                                                    imgCurR,...
                                                    imgKeyL,...
                                                    imgKeyR,...
                                                    imgPrvL,...
                                                    imgPrvR,...
                                                    depthMapKey,...
                                                    depthMapKeyGT,...
                                                    LocalTrace,...
                                                    KeyProbZ,...
                                                    thetaRngMat,...
                                                    thetaProbMat,...
                                                    prvDepthGT ,...
                                                    prvDepthVisual,...
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
                                                    objPervPtCcsZGolden1] = RefineZ(k2cRef_1_00, obj, k2cRef_1,k2cBodyRotAng, inlierId,inlierId2, theta, DepthProbility,  DepthId,  AngleModalOrg,  AngleProbility,prevFeatPtList0,ptCcsZ0,DispRefineUse__,NewDispRng,newDispStep,ProbZTmpReSamp, k2cBodyRotAng,...
                                                    keyFrameLen ,...
                                                    disparity_resample_prob_threshold,...
                                                    ThetaNoise ,...
                                                    KeyFrameFlagList,...
                                                    ConfigParam ,...
                                                    depthMapCurVisual,...
                                                    depthMapCurGT,...
                                                    imgCur ,...
                                                    imgCurR,...
                                                    imgKeyL,...
                                                    imgKeyR,...
                                                    imgPrvL,...
                                                    imgPrvR,...
                                                    depthMapKey,...
                                                    depthMapKeyGT,...
                                                    LocalTrace,...
                                                    KeyProbZ,...
                                                    thetaRngMat,...
                                                    thetaProbMat,...
                                                    prvDepthGT ,...
                                                    prvDepthVisual,...
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
                                                    objPervPtCcsZGolden);  %, keyCcsXYZAll);
                                                
                                                
                                                FrmNumWithGloden = FrmNumWithGloden0;
                                                
                                                
                                                
                                                % % % %                                             if isempty(angOptP2C)
                                                % % % %                                                 angOptP2C = angOpt1;
                                                % % % %                                             end
                                                
                                                %                                             keyFrameLen = size(obj.featPtManager.localTrace.ptIcsX, 2);
                                                obj.configParam.disparity_resample_prob_threshold = disparity_resample_prob_threshold;
                                                %                                             ThetaNoise = obj.thetaNoise;
                                                obj.keyFrameFlagList = KeyFrameFlagList;
                                                %                                             ConfigParam = obj.configParam;
                                                obj.depthVisual = depthMapCurVisual;
                                                obj.depthGT = depthMapCurGT;
                                                obj.currImgL = imgCur;
                                                obj.currImgR = imgCurR;
                                                obj.keyFrameImgL = imgKeyL;
                                                obj.keyFrameImgR = imgKeyR;
                                                obj.prevImgL = imgPrvL;
                                                obj.prevImgR = imgPrvR;
                                                obj.keyFrameDepth = depthMapKey;
                                                obj.keyFrameDepthGT = depthMapKeyGT;
                                                obj.featPtManager.localTrace = LocalTrace;
                                                obj.keyProbZ = KeyProbZ;
                                                obj.thetaRngMat = thetaRngMat;
                                                obj.thetaProbMat = thetaProbMat;
                                                obj.prvDepthGT = prvDepthGT;
                                                obj.prvDepthVisual = prvDepthVisual;
                                                %                                             CamModel = obj.camModel;
                                                %                                             CoordSysAligner = obj.coordSysAligner;
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
                                                
                                                thetaPnP = pnpLite(obj.pureRotationAngPredictor,intrMat,obj.featPtManager,obj.coordSysAligner,k2cRef_1,obj.keyFrameFlagList,obj.frmStampList,obj.goldenPose,pixKey ,pt2dCur, DispRefine, obj.camModel, obj.scaleLvl);
                                                
                                            end
                                            
                                            
                                            
                                            
                                            
                                            
                                            
                                            
                                            
                                            
                                            
                                            
                                            
                                            if obj.startTrace2
                                                errTrace2_ = rad2deg([0;accumTrace2]); % - ([0;cumsum(obj.trace2.refAngAll)]);
                                                KeyAngCat_ = [zeros(length(obj.trace2.keyFrameFlagList) - size(KeyAng,1),2);[KeyAng(:,1) deg2rad(errTrace2_)]];
                                                
                                                
                                                %                                             [obj.traceAng1 obj.trace2.KeyAngCat]
                                                if 0
                                                    compareTrace = [[obj.keyFrameFlagList [obj.traceAng1(1:end-1, 2);angOpt1;0]] KeyAngCat_];
                                                else
                                                    compareTrace = [[obj.keyFrameFlagList [0; obj.traceAng1(1:end-1, 2);angOpt1]] KeyAngCat_];
                                                    id1 = find(compareTrace(:,1) == 1);
                                                    id2 = find(compareTrace(:,3) == 1);
                                                    compareTrace(end,2) = compareTrace(id1(end),2) + angOpt1;
                                                    
                                                    id12 = max([id1(end) id2(end)]);
                                                    id12 = max([id1(1) id2(1)]);
                                                    compareTraceNearestKey = rad2deg([(compareTrace(id12:end,2) - compareTrace(id12,2))  (compareTrace(id12:end,4) - compareTrace(id12,4))]);
                                                end
                                            end
                                            
                                            
                                            obj.trace2.prevImgL = obj.currImgL;
                                            obj.trace2.prevImgR = obj.currImgR;
                                            obj.trace2.prvDepthGT = obj.depthGT;
                                            obj.trace2.prvDepthVisual = obj.depthVisual;
                                            obj.trace2.prevRefRobotPoseWcs = obj.currRefRobotPoseWcs;
                                            obj.trace2.prevRefRobotRotAngWcs = obj.currRefRobotRotAngWcs;
                                            
                                            % % % %                                             thetaPnP = pnpLite(obj.pureRotationAngPredictor,intrMat,obj.featPtManager,obj.coordSysAligner,k2cRef,obj.keyFrameFlagList,obj.frmStampList,obj.goldenPose,pixKey ,pt2dCur, DispRefine, obj.camModel, obj.scaleLvl);
                                            
                                            %                                         catch
                                            wblnk = 1;
%                                         end
                                    elseif 0
                                        angOpt1 = 0.02;
                                        angRng1 = [-0.01 0.03];
                                        thetaPnP = 0.01;
                                        angOpt33 = 0.025;
                                        angOptP2C = 0.002;
                                        thetaRngOut = [];
                                        thetaProbOut = [];
                                        angOptK2C_ = 0.1;
                                        
                                        
                                        
                                         angOpt1_2 = 0.02;
                                        angRng1_2 = [-0.01 0.03];
                                        thetaPnP_2 = 0.01;
                                        angOpt33_2 = 0.025;
                                        angOptP2C_2 = 0.002;
                                        thetaRngOut_2 = [];
                                        thetaProbOut_2 = [];
                                        angOptK2C__2 = 0.1;
                                        obj.accumP2CTemp = 0.1;
                                    else
                                        sjhsds = 1;
                                    end
                                    %                                     thetaPnP = pnpLite(obj.pureRotationAngPredictor,intrMat,obj.featPtManager,obj.coordSysAligner,k2cRef,obj.keyFrameFlagList,obj.frmStampList,obj.goldenPose,pixKey ,pt2dCur, DispRefine, obj.camModel, obj.scaleLvl);
                                    
                                    
                                    if 0
                                        
                                        obj.angOpt = [obj.angOpt; [angOpt1]];
                                        obj.angOptCheck = [obj.angOptCheck; angOptK2C_];
                                        
                                        obj.angOpt3 = [obj.angOpt3; [angOpt33]];
                                        
                                        obj.accumP2C = [obj.accumP2C; obj.accumP2CTemp(end)];
                                        
                                        if size(LocalTraceList{1}.ptIcsX,2) == 2   % size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                            obj.refAngList4 = [];
                                        end
                                        
                                        if isempty(obj.angOptP2CList)
                                            obj.angOptP2CList = [obj.angOptP2CList; angOpt1];
                                        else
                                            if isempty(obj.refAngList4)
                                                obj.angOptP2CList = [obj.angOptP2CList; angOpt1];
                                            else
                                                obj.angOptP2CList = [obj.angOptP2CList; obj.refAngList4(end) + angOptP2C];
                                            end
                                        end
                                        
                                        
                                        if Switch4to3
                                            obj.refAngList4 = [obj.refAngList3];
                                        else
                                            obj.refAngList4 = [obj.refAngList4; angOpt1];
                                        end
                                        
                                        
                                        obj.angOptPnP = [obj.angOptPnP; [thetaPnP]];
                                        obj.angRng = [obj.angRng; angRng1];
                                        
                                        obj.thetaRngMat = [obj.thetaRngMat; thetaRngOut];
                                        
                                        obj.thetaProbMat = [obj.thetaProbMat; thetaProbOut];
                                        
                                        
                                    else
                                        if 0
                                            LocalTraceDoneTemp = obj.tempProb{1,1};
                                        else
                                            LocalTraceDoneTemp = tempProbNew{1,1};
                                        end
                                        
                                        if 0 % length(LocalTraceDoneTemp.angOpt) == length(obj.accumP2CRef) - 1
                                            if 0
                                                if isempty(obj.angOptManager.angOptMat)
                                                    angOptMat = zeros(1);
                                                    
                                                else
                                                    
                                                end
                                            else
                                                for za = 1 :  length(LocalTraceList)
                                                    angOptTemp =  [0;tempProbNew{za,1}.angOpt];
                                                    if za == 1
                                                        angOptStackMat = zeros(length(LocalTraceList), length(angOptTemp) + 0);
                                                        goldenStackMat =  repmat(obj.accumP2CRef',length(LocalTraceList),1);
                                                        angOptStackMat(za,:) = angOptTemp';
                                                    else
                                                        angOptStackMat(za,za:end) = angOptTemp';
                                                        goldenStackMat(za,za:end) = goldenStackMat(za,za:end) - goldenStackMat(za,za);
                                                        goldenStackMat(za,1:za) = 0;
                                                    end
                                                end
                                            end
                                            sdb = 1;
                                        else
                                            
                                            asvhkj = 1;
                                        end
                                        
                                        
                                        if isempty(obj.angOptManager.featIdCell)
                                            for cv = 1 : length(LocalTraceList)
                                                obj.angOptManager.featIdCell{cv,1} = LocalTraceList{cv, 1}.featId;
                                            end
                                        else
                                            newFeatIdCell = {};OldNewMat = [];addNewTraceNum = 1;
                                            for cv = 1 : length(LocalTraceList)
                                                curFeatId = LocalTraceList{cv, 1}.featId;
                                                newFeatIdCellTemp = {};
                                                foundOldTrace = false;
                                                for cf = 1 : length(obj.angOptManager.featIdCell)
                                                   prvFeatId =  obj.angOptManager.featIdCell{cf,1};
                                                   commonId2 = intersect(curFeatId, prvFeatId);
                                                   if ~isempty(commonId2)
                                                       obj.angOptManager.featIdCell{cf, 1} = curFeatId;
                                                       foundOldTrace = true;
                                                       OldNewMat = [OldNewMat; [cf cv]];
                                                       break;
                                                   else
                                                       newFeatIdCellTemp = [newFeatIdCellTemp; {curFeatId}];
                                                   end
                                                end
                                                if ~foundOldTrace
                                                    if ~isempty(newFeatIdCellTemp)
                                                        newFeatIdCell = [newFeatIdCell; newFeatIdCellTemp{1,1}];
                                                    end
                                                    OldNewMat = [OldNewMat; [(length(obj.angOptManager.featIdCell)+addNewTraceNum) cv]];
                                                    addNewTraceNum = addNewTraceNum + 1;
                                                else
%                                                     OldNewMat = [OldNewMat; [0 cv]];
                                                end
                                            end
                                            if ~isempty(newFeatIdCell)
                                                obj.angOptManager.featIdCell = [obj.angOptManager.featIdCell; newFeatIdCell];
                                            end
                                        end
                                        
                                        if 0
                                            obj.angOptManager.angOptMat = angOptStackMat;
                                            obj.angOptManager.angRefMat = goldenStackMat;
                                            obj.angOptManager.angErrMat = angOptStackMat - goldenStackMat;
                                        end
                                        
                                        haqeh = 1;
                                        
                                        if 0
                                            
                                            if isempty(obj.angOptManager.angOptMat)
                                                obj.angOptManager.angOptMat = -1.*ones(size(obj.traceManager.X));
                                            else
                                                obj.angOptManager.angOptMat = [obj.angOptManager.angOptMat -1.*ones(size(obj.angOptManager.angOptMat,1),1)];
                                            end
                                            
                                            
                                            for fg = 1 : size(tempProbNew,1)
                                                % % %                                             if isempty(obj.angOptManager.angOptMat)
                                                % % %                                                 obj.angOptManager.angOptMat = zeros(size(obj.traceManager.X));
                                                % % %                                             else
                                                % % %                                                 obj.angOptManager.angOptMat = [obj.angOptManager.angOptMat zeros(size(obj.angOptManager.angOptMat,1),1)];
                                                % % %                                             end
                                                frmLenTemp = size(tempProbNew{fg,1}.LocalTrace.ptIcsX,2);
                                                angOptTemp_ = abs([100;tempProbNew{fg, 1}.angOpt]');
                                                obj.angOptManager.angOptMat(tempProbNew{fg,1}.LocalTrace.featId,end-frmLenTemp+1:end) = repmat(angOptTemp_, length(tempProbNew{fg,1}.LocalTrace.featId),1);
                                            end
                                            obj.validFeatIdStack = [obj.validFeatIdStack; validFeatId];
                                            
                                            checkManagement = ((obj.angOptManager.angOptMat(obj.validFeatIdStack,:) < 0 - obj.traceManager.X(obj.validFeatIdStack,:) < 0));
                                            %                                         checkManagement = ((obj.angOptManager.angOptMat(obj.validFeatIdStack,:) <= 0 - obj.traceManager.X(obj.validFeatIdStack,:) <= 0));
                                            %                                         checkManagement = (((obj.angOptManager.angOptMat(obj.validFeatIdStack,:) >= 0) - (obj.traceManager.X(obj.validFeatIdStack,:) >= 0)));
                                            
                                            checkManagement = (((obj.angOptManager.angOptMat(obj.validFeatIdStack,:) > 0) - (obj.traceManager.X(obj.validFeatIdStack,:) > 0)));
                                            a1 = obj.angOptManager.angOptMat(obj.validFeatIdStack,:);%  >= 0;
                                            a2 = obj.traceManager.X(obj.validFeatIdStack,:); %  >= 0;
                                            
                                            a11 = obj.angOptManager.angOptMat > 0;
                                            checkManagement = (a1>0) - (a2>0);
                                            checkManagementSum = sum(double(checkManagement(:)));
                                            
                                            angTrace = sum(a11')';
                                            angMatSort = immultiply(a11,obj.angOptManager.angOptMat);
                                            %                                         angTrace1 = sum(obj.angOptManager.angOptMat')';
                                            angTrace = sum(angMatSort')';
                                            %                                         [angTraceUnique1,a31,a41] = unique(angTrace1);
                                            [angTraceUnique,a3,a4] = unique(angTrace);
                                            
                                            if 0
                                                angTrace1 = sum(obj.angOptManager.angOptMat')';
                                                [angTraceUnique1,a31,a41] = unique(angTrace1);
                                                angMatSlim1 = obj.angOptManager.angOptMat(a31,:);
                                            else
                                                angMatSlim = angMatSort(a3,:);
                                            end
                                            
                                            
                                            
                                            startInd = find(angMatSlim == 100);
                                            [startIndY, startIndX] = ind2sub(size(angMatSlim), startInd);
                                            [~, sortIdStart2] = sort(startIndX , 'ascend');
                                            [~, sortIdStart] = sort(startIndY , 'ascend');
                                            startIndX = startIndX(sortIdStart);
                                            startIndY = startIndY(sortIdStart);
                                            endInd = find(angMatSlim ~= 100 & angMatSlim > 0);
                                            [endIndY, endIndX] = ind2sub(size(angMatSlim), endInd);
                                            [~, sortIdEnd] = sort(endIndY, 'ascend');
                                            endIndX = endIndX(sortIdEnd);
                                            endIndY = endIndY(sortIdEnd);
                                            coordAngEnd = [];
                                            for nm = startIndY'
                                                idtp = find(endIndY == nm);
                                                [~, idmaxx] = max(endIndX(idtp));
                                                coordAngEnd = [coordAngEnd; [endIndX(idtp(idmaxx)) nm]];
                                            end
                                            coordAngStart = [startIndX startIndY];
                                            for bn = 1 : size(coordAngStart,1)
                                                angMatUse(coordAngStart(bn,2), coordAngStart(bn,1):coordAngEnd(bn,1)) = angMatSlim(coordAngStart(bn,2), coordAngStart(bn,1):coordAngEnd(bn,1));
                                            end
                                            
                                            angMatUse2 = angMatSlim(sortIdStart2,:);
                                            %                                         angMatUse(angMatUse == 100) = 0;
                                            %                                         angMatUseDiff = diff(angMatUse')';
                                            uniqueStartX = unique(coordAngStart(:,1));
                                            coordStartEnd = [coordAngStart coordAngEnd];
                                            for gf1 = 1:length(uniqueStartX)
                                                idXX = find(coordAngStart(:,1) == uniqueStartX(gf1));
                                                [~, idmaxx] = max(coordAngEnd(idXX,2));
                                                %                                             angMatUse3((gf1), uniqueX(gf1):coordAngEnd(bn,1)) = angMatSlim(coordAngStart(bn,2), coordAngStart(bn,1):coordAngEnd(bn,1));
                                                angMatUseNew((gf1), uniqueStartX(gf1):coordAngEnd(idXX(idmaxx),1)) = angMatSlim(coordAngStart(idXX(idmaxx),2),  uniqueStartX(gf1):coordAngEnd(idXX(idmaxx),1));
                                            end
                                            angMatUseNewBak = angMatUseNew;
                                            
                                            
                                            %                                         obj.angOptManager.angOptMat = angOptStackMat;
                                            obj.angOptManager.angOptMatUniq = angMatUseNewBak;
                                            
                                            if 1
                                                if 1
                                                    figure(4),clf;subplot(1,2,1);imshow(immultiply(angMatUseNewBak , angMatUseNewBak ~= 100), []);subplot(1,2,2);imshow(angMatUseNewBak & angMatUseNewBak ~= 100, [])
                                                else
                                                    figure(4),clf;imshow(angMatUseNewBak & angMatUseNewBak ~= 100, [])
                                                end
                                            end
                                            
                                            angMatUseNew(angMatUseNew == 100) = 0;
                                            if 0
                                                angArrangeErr = angMatUseNew - angOptListLast;
                                            end
                                            
                                            
                                            angMatUseNewDiff = diff(angMatUseNew')';
                                            angMatUseNewDiff(angMatUseNewDiff < 0) = 0;
                                            validAngOpt = angMatUseNewDiff > 0;
                                            validAngOptSum = sum(validAngOpt);
                                            
                                            accumErrWeight = sum(validAngOpt')';
                                            accumErrWeightMat = repmat(accumErrWeight,1,size(validAngOpt,2));
                                            accumErrWeightMatValid = (accumErrWeightMat.*validAngOpt)';
                                            
                                            if 0
                                                accumErrWeightMatValid = 1./accumErrWeightMatValid;
                                                accumErrWeightMatValid(isinf(accumErrWeightMatValid)) = 0;
                                            elseif 0
                                                accumErrWeightMatValid(accumErrWeightMatValid == 0) = nan;
                                                accumErrWeightMatValid = max(accumErrWeightMatValid(:)) - accumErrWeightMatValid+1;
                                                accumErrWeightMatValid(isnan(accumErrWeightMatValid)) = 0;
                                            else
                                                saknab = 1;
                                            end
                                            
                                            accumErrWeightMatValidNorm = (accumErrWeightMatValid./repmat(sum(accumErrWeightMatValid')',1,size(accumErrWeightMatValid,2)))';
                                            if 1
                                                meanAngP2C = (sum(angMatUseNewDiff)./validAngOptSum)';
                                            else
                                                meanAngP2C = sum(accumErrWeightMatValidNorm.*angMatUseNewDiff)';
                                            end
                                            
                                            angErr = rad2deg([0;cumsum(meanAngP2C)] - abs(obj.accumP2CRef));
                                        else
                                            
                                            [angErr, angErrUpper, angErrLower, angErrP2C, flipWeight,frame_k2, KeyBaseErrGtZ] = CalcAngErr(obj, tempProbNew, validFeatId);
                                            
                                            ssdk = 1;
                                        end
                                          
                                        try
                                            sgdjhf;
                                            VisualLocalizer.DrawPic(probPath, featInd, length(obj.keyFrameFlagList), TraceBatchList, dataTmp.featSize, dataTmp.dd, dataTmp.ddGT, dataTmp.disparityRng, dispCompare)
                                        catch
                                            asbkj = 1;
                                        end
                                        
                                        asdtg = 1;
                                        
                                        
                                        %                                         figure(5),clf;plot(rad2deg(angOptStackMat - goldenStackMat)');
                                        if 0
                                            figure(5),clf;plot(angErr,'-b');
                                        end
                                        grktjte = 1;
                                        for gy = 1 : size(obj.colorMat,1)
                                            if gy <= size(tempProbNew,1)
                                                % if ~isempty(tempProbNew{gy,1})
                                                obj.tempProb{gy,1} = tempProbNew{gy,1};
                                                obj.tempProb{gy,2} = tempProbNew{gy,2};
                                            else
                                                obj.tempProb{gy,1} = {};%tempProbNew{gy,1};
                                                obj.tempProb{gy,2} = {};%tempProbNew{gy,2};
                                            end
                                        end
                                            
                                        obj.angOpt = LocalTraceDoneTemp.angOpt; %[obj.angOpt; [angOpt1]];
                                        obj.angOptCheck = LocalTraceDoneTemp.angOptCheck; %[obj.angOptCheck; angOptK2C_];
                                        
                                        obj.angOpt3 = LocalTraceDoneTemp.angOpt3; %[obj.angOpt3; [angOpt33]];
                                        
                                        obj.accumP2C = LocalTraceDoneTemp.accumP2C; %[obj.accumP2C; obj.accumP2CTemp(end)];
                                        
%                                         if size(LocalTraceList{1}.ptIcsX,2) == 2   % size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
%                                             obj.refAngList4 = [];
%                                         end
                                        
% % %                                         if isempty(obj.angOptP2CList)
% % %                                             obj.angOptP2CList = [obj.angOptP2CList; angOpt1];
% % %                                         else
% % %                                             if isempty(obj.refAngList4)
% % %                                                 obj.angOptP2CList = [obj.angOptP2CList; angOpt1];
% % %                                             else
% % %                                                 obj.angOptP2CList = [obj.angOptP2CList; obj.refAngList4(end) + angOptP2C];
% % %                                             end
% % %                                         end
                                         obj.angOptP2CList =  LocalTraceDoneTemp.angOptP2CList;
                                        
% %                                         if Switch4to3
% %                                             obj.refAngList4 = [obj.refAngList3];
% %                                         else
% %                                             obj.refAngList4 = [obj.refAngList4; angOpt1];
% %                                         end
                                        
                                        obj.refAngList4 = LocalTraceDoneTemp.refAngList4;
                                        
                                        
                                        
                                        
                                        
                                        
                                        
                                        
                                        
                                        obj.angOptPnP = LocalTraceDoneTemp.angOptPnP; % [obj.angOptPnP; [thetaPnP]];
                                        obj.angRng = LocalTraceDoneTemp.angRng;  %[obj.angRng; angRng1];
                                        
                                        obj.thetaRngMat = LocalTraceDoneTemp.thetaRngMat; %[obj.thetaRngMat; thetaRngOut];
                                        
                                        obj.thetaProbMat = LocalTraceDoneTemp.thetaProbMat;  %[obj.thetaProbMat; thetaProbOut];
                                        
                                        
                                        
                                    end
                                    
                                    
                                    if 0
                                        try
                                            obj.trace2.angOpt = [obj.trace2.angOpt; [angOpt1_2]];
                                            obj.trace2.angOptCheck = [obj.trace2.angOptCheck; angOptK2C__2];
                                            
                                            obj.trace2.angOpt3 = [obj.trace2.angOpt3; [angOpt33_2]];
                                            
                                            obj.trace2.accumP2C = [obj.trace2.accumP2C; obj.trace2.accumP2CTemp(end)];
                                            
                                            if size(obj.featPtManager.localTrace2.ptIcsX, 2) == 2
                                                obj.trace2.refAngList4 = [];
                                            end
                                            
                                            if isempty(obj.trace2.angOptP2CList)
                                                obj.trace2.angOptP2CList = [obj.trace2.angOptP2CList; angOpt1_2];
                                            else
                                                if isempty(obj.trace2.refAngList4)
                                                    obj.trace2.angOptP2CList = [obj.trace2.angOptP2CList; angOpt1_2];
                                                else
                                                    obj.trace2.angOptP2CList = [obj.trace2.angOptP2CList; obj.trace2.refAngList4(end) + angOptP2C_2];
                                                end
                                            end
                                            
                                            
                                            if Switch4to3
                                                obj.trace2.refAngList4 = [obj.trace2.refAngList3];
                                            else
                                                obj.trace2.refAngList4 = [obj.trace2.refAngList4; angOpt1_2];
                                            end
                                            
                                            
                                            obj.trace2.angOptPnP = [obj.trace2.angOptPnP; [thetaPnP_2]];
                                            obj.trace2.angRng = [obj.trace2.angRng; angRng1_2];
                                            
                                            obj.trace2.thetaRngMat = [obj.trace2.thetaRngMat; thetaRngOut_2];
                                            
                                            obj.trace2.thetaProbMat = [obj.trace2.thetaProbMat; thetaProbOut_2];
                                            
                                            
                                            
                                            
                                            keyId = find(obj.trace2.keyFrameFlagList);
                                            keyIdList = obj.trace2.keyFrameFlagList(keyId(1)+0:end);
                                            KeyAng = [keyIdList [obj.trace2.angOpt;0]];
                                            
                                            
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
                                            
                                            errTrace2 = rad2deg([0;accumTrace2] - [0;cumsum(obj.trace2.refAngAll)]);
                                            KeyAngCat = [zeros(length(obj.trace2.keyFrameFlagList) - size(KeyAng,1),2);[KeyAng(:,1) deg2rad(errTrace2)]];
                                            obj.trace2.KeyAngCat = KeyAngCat;
                                            %                                         pulse = repmat(min(rad2deg(KeyAngCat(:,2))),size(KeyAngCat,1),1);
                                            %                                         pulse(idPulse) = max(errTrace2);
                                            %                                         figure(333),clf;plot(rad2deg(KeyAngCat(:,2)),'-b');hold on;plot(find(KeyAngCat(:,1)),rad2deg(KeyAngCat(find(KeyAngCat(:,1)),2)),'*g'); % plot(pulse,'k');
                                            %                                         drawnow;title('accum P2C')
                                            %                                         saveas(gcf,fullfile(probPath,sprintf('final_error_%05d.fig',0+3)));
                                            
                                            
                                            
                                        catch
                                            kjasv = 1;
                                        end
                                    end
                                    
                                    dat = [pix0_GT_(:,1) - PixCur(:,1), pix0_GT_(:,2) - PixCur(:,2)];
                                    obj.trackingErrAccum = [obj.trackingErrAccum; dat];
% %                                     figure(13),clf;hist3(obj.trackingErrAccum);
% %                                     xlabel('x error'); ylabel('y error');
% %                                     set(get(gca,'child'),'FaceColor','interp','CDataMode','auto');
                                    if 0
                                        n_ = hist3(dat); % default is to 10x10 bins
                                        n1_ = n_';
                                        n1_(size(n_,1) + 1, size(n_,2) + 1) = 0;
                                        
                                        
                                        xb = linspace(min(dat(:,1)),max(dat(:,1)),size(n_,1)+1);
                                        yb = linspace(min(dat(:,2)),max(dat(:,2)),size(n_,1)+1);
                                        
                                        h = pcolor(xb,yb,n1_);
                                        h.ZData = ones(size(n1_)) * -max(max(n_));
                                        colormap(hot) % heat map
                                        title('Seamount:Data Point Density Histogram and Intensity Map');
                                        grid on
                                        view(3);
                                    end
                                    if 0
                                        figure(14),
                                    end
                                    if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                        if 0
                                            clf;
                                        end
                                    end
                                    if 0
                                        hold on;plot(pix0_GT_(:,1) - PixCur(:,1), pix0_GT_(:,2) - PixCur(:,2), '+');title('gt - tracking');axis equal;
                                    end
                                    if 0
                                        figure(16),
                                        if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                            clf;
                                            obj.MeanErr = [0 0];
                                            obj.MeanErrGT = [0 0];
                                            obj.MeanErrOld = [0 0];
                                        end
                                        hold on;plot(mean(pix0_GT_(:,1) - PixCur(:,1)), mean(pix0_GT_(:,2) - PixCur(:,2)), '-+');title('gt - tracking');axis equal;
                                        
                                    else
                                        %                                         figure(16),
                                        if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                            %                                             clf;
                                            obj.MeanErr = [0 0];
                                            obj.MeanErrGT = [0 0];
                                            obj.MeanErrOld = [0 0];
                                        end
                                        %                                         hold on;plot(mean(pix0_GT_(:,1) - PixCur(:,1)), mean(pix0_GT_(:,2) - PixCur(:,2)), '-+');title('gt - tracking');axis equal;
                                        
                                    end
                                    if 0
                                        %                                         meanErr = [mean(pix0_GT_(:,1) - PixCur(:,1)) mean(pix0_GT_(:,2) - PixCur(:,2))];
                                    else
                                        %                                         meanErr = [mean(pix0_GT_(validXYZAll,1) - PixReproj(:,1)) mean(pix0_GT_(validXYZAll,2) - PixReproj(:,2))];
                                        %                                         meanErr = [mean(pix0_GT_(validXYZAll,1) - PixReproj(:,1)) mean(pix0_GT_(validXYZAll,2) - PixReproj(:,2))];
                                        meanErr = [mean(PixReproj(:,1) - PixCur(vldReproj,1)) mean(PixReproj(:,2) - PixCur(vldReproj,2))];
                                        meanErrGT = [mean(PixReproj(:,1) - pix0_GT_(vldReproj,1)) mean(PixReproj(:,2) - pix0_GT_(vldReproj,2))];
                                        meanErrOld = [mean(pix0_GT_(:,1) - PixCur(:,1)) mean(pix0_GT_(:,2) - PixCur(:,2))];
                                    end
                                    obj.MeanErr = [obj.MeanErr; meanErr];
                                    obj.MeanErrGT = [obj.MeanErrGT; meanErrGT];
                                    obj.MeanErrOld = [obj.MeanErrOld; meanErrOld];
                                    
                                    errrrr = pix0_GT(:,1:2)-PixReproj(1:size(pix0_GT,1),:);
                                    
                                    obj.traceErr = [obj.traceErr; [pix0_GT(:,1:2) PixCur(validXYZAll,:)]];
                                    
                                    if 0
                                        figure(111),clf;imshow(imgCurL);hold on;plot(PixCur(:,1),PixCur(:,2),'or');plot(PixReproj(:,1),PixReproj(:,2),'.g');
                                    else
                                        
                                    end
                                    % % % % % % % % % % % % % %                                     if size(obj.featPtManager.localTrace.ptIcsX,2) <= 2
                                    % % % % % % % % % % % % % % %                                         figure(109),clf;imshow(imgCurL);hold on;plot(PixCur(:,1),PixCur(:,2),'or');plot(pix0_GT(:,1),pix0_GT(:,2),'.g');plot([pix0_GT(:,1) PixCur(validXYZAll,1)]',[pix0_GT(:,2) PixCur(validXYZAll,2)]','-c');
                                    % % % % % % % % % % % % % %                                         obj.featPtManager.localTrace.xGT = obj.featPtManager.localTrace.ptIcsX(:,1);
                                    % % % % % % % % % % % % % %                                         obj.featPtManager.localTrace.yGT = obj.featPtManager.localTrace.ptIcsY(:,1);
                                    % % % % % % % % % % % % % %                                     end
                                    % % % % % % % % % % % % % %                                     obj.featPtManager.localTrace.xGT = [obj.featPtManager.localTrace.xGT repmat(-1,size(obj.featPtManager.localTrace.xGT,1),1)];
                                    if 0
                                        obj.featPtManager.localTrace.xGT(inlierId(validXYZAll),end) = pix0_GT(:,1);
                                        %                                     obj.featPtManager.localTrace.xGT(inlierId,end) = pix0_GT_(:,1);
                                        
                                        % % % % % % % % % % % % % %                                     obj.featPtManager.localTrace.yGT = [obj.featPtManager.localTrace.yGT repmat(-1,size(obj.featPtManager.localTrace.yGT,1),1)];
                                        obj.featPtManager.localTrace.yGT(inlierId(validXYZAll),end) = pix0_GT(:,2);
                                    else
                                        if 1 % 20200106 temporary debug
                                        obj.featPtManager.localTrace.xGT(inlierId(validXYZAll),end) = pix0_GT(:,1);
                                        obj.featPtManager.localTrace.yGT(inlierId(validXYZAll),end) = pix0_GT(:,2);
                                        end
                                    end
                                    %                                     obj.featPtManager.localTrace.yGT(inlierId,end) = pix0_GT_(:,2);
                                end
                                %                                 obj.featPtManager.localTrace.ptIcsX(setdiff((1:size(obj.featPtManager.localTrace.ptIcsX,1))',validId),keyLength) = -1;
                                %                                 obj.featPtManager.localTrace.ptIcsY(setdiff((1:size(obj.featPtManager.localTrace.ptIcsX,1))',validId),keyLength) = -1;
                                inLierFlag = false(length(activeFeatIndLast),1);
                                inLierFlag(validId) = true;
                                drawnow;
                            end
                             
                            catch
%                         else 
                            
                            
                            theta = k2cBodySensorRotAng;
                            thetaRngOut = [];
                            thetaProbOut = [];
                            angOptK2C_ = theta;
                            if isempty(obj.angOptP2CList)
                                obj.angOptP2CList = [obj.angOptP2CList; theta];
                            else
%                                 obj.angOptP2CList = [obj.angOptP2CList; obj.refAngList4(end) + angOptP2C];
                                obj.angOptP2CList = [obj.angOptP2CList; theta];
                            end
                            
                            obj.angOpt = [obj.angOpt; [theta]];
                            obj.accumP2C = [obj.accumP2C; [theta]];
                            
                            
                            
                            obj.angOptCheck = [obj.angOptCheck; theta];
                            
                            obj.angOptPnP = [obj.angOptPnP; [theta]];
                            obj.angOpt3 = [obj.angOpt3; [theta]];
%                             obj.angRng = [obj.angRng; obj.angRng(end,:) + theta];
                            obj.angRng = [obj.angRng; theta + deg2rad([-0.2 0.2])];
                            
                            if Switch4to3
                                obj.refAngList4 = [obj.refAngList3];
                            else
                                obj.refAngList4 = [obj.refAngList4; theta];
                            end
                            
                            
                            
                            
                            inTrackFlag = false(length(inTrackFlag),1);
                            
                            inLierFlag = false(sum(inTrackFlag),1);
                            c2wTrans = obj.currRefRobotPoseWcs;
                            p2wTrans = obj.prevRefRobotPoseWcs;
                            c2pTrans = p2wTrans\c2wTrans;
                            p2wTransV = WcsPoseVecToPct(obj, obj.poseWcsList(end,:));
                            c2wTransV = p2wTransV*c2pTrans;
                            curloc = T(c2wTransV);
                            deltaAngle1 = obj.currRefRobotRotAngWcs - obj.prevRefRobotRotAngWcs;
                            robotPoseWcs = [curloc(1),curloc(3),obj.poseWcsList(end,3)+deltaAngle1];
                            
                        end
                    else
                        
                        
                        k2cBodySensorRotAng = obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs;
                        p2cBodySensorRotAng = obj.currRefRobotRotAngWcs - obj.prevRefRobotRotAngWcs;
                        
                        %                          p2cVisionRotAng = PredictRotateAngle2(obj,p2cBodySensorRotAng,obj.frmStamp,obj.goldenPose);
                        
                        p2cVisionRotAng = PredictRotateAngle(obj,p2cBodySensorRotAng);
                        ind = GetLastKeyFrameInd(obj);
                        k2pVisionRotAng = obj.poseWcsList(end,3) - obj.poseWcsList(ind,3);
                        k2cVisionRotAng = p2cVisionRotAng + k2pVisionRotAng;
                        deltaVision2BodySensor = abs(rad2deg(k2cVisionRotAng-k2cBodySensorRotAng));
                        if deltaVision2BodySensor > 200
                            theta = k2cBodySensorRotAng;
                        else
                            theta = k2cVisionRotAng;
                        end
                        [keyPtIcs, curPtIcs] = GetFeaturePairs(obj.featPtManager, 'key', 'last');
                        projErr = AngleSpace.ProjectErrorUnderTransform(theta, 0, keyPtIcs, curPtIcs, obj.coordSysAligner, 1, intrMat);
                        projErrType = obj.angleSpace.configParam.objective_Function_type;
                        if strcmp(projErrType, 'L1')
                            trackNoiseLvl = obj.angleSpace.configParam.track_noise_level_in_pixel;
                        elseif strcmp(projErrType, 'L2')
                            trackNoiseLvl = obj.angleSpace.configParam.track_noise_level_in_pixel^2;
                        else
                            error('Unsupported project error type: %s', projErrType);
                        end
                        transitionLength = obj.angleSpace.configParam.smooth_function_transition_length;
                        score = AngleSpace.SmoothRectFunc(projErr, -trackNoiseLvl, trackNoiseLvl, transitionLength);
                        inLierFlag = score > 0.9;
                        inLierFlag =inLierFlag';
                    end
                    %%  NO need to retrack
                    %%  STILL need to retrack
                    if sum(inTrackFlag) > obj.configParam.min_num_pt_for_calculate2
                        %                     if sum(inLierFlag) > obj.configParam.min_num_pt_for_calculate2
                        [~, ~, activeFeatInd] = GetActiveFeatures(obj.featPtManager, 'last');
                        %  SaveInLier(obj.featPtManager, 'last', activeFeatInd, inLierFlag);
                        ind = GetLastKeyFrameInd(obj);
                        keyPoseWcs = obj.poseWcsList(ind,:);%vsl.configParam.right_feature_point_trace_param
                        robotPoseWcs = [keyPoseWcs(1:2), keyPoseWcs(3)+theta];
                        reTrackFlag = setdiff(find(obj.featPtManager.localTrace.ptIcsX(:,end-1) ~= -1),activeFeatInd);
                        ptPrv = [obj.featPtManager.localTrace.ptIcsX(:,end-1) obj.featPtManager.localTrace.ptIcsY(:,end-1)];
                        try
                            [curPt, flagCur,fundMat] = ReTraceEpiLine(obj, robotPoseWcs,ptPrv,obj.featPtManager.localTrace.ptIcsX(:,end-1) ~= -1);
                            reTrackId = intersect(reTrackFlag,find(flagCur));
                            reTrackId = [];
                            obj.featPtManager.localTrace.ptIcsX(reTrackId,end) = curPt(reTrackId,1);
                            obj.featPtManager.localTrace.ptIcsY(reTrackId,end) = curPt(reTrackId,2);
                        catch
                            qgbqb = 1;
                        end
                    end
                    %                     fprintf(sprintf('### retracked points: %d/%d ###\n\n\n',length(reTrackId),sum(obj.featPtManager.localTrace.ptIcsX(:,end) ~= -1)));
                    %% changed way before reconstruct code
                    % SaveInLier(obj.featPtManager, 'last', activeFeatInd, validFlag);
                    %% 20190713 changed below
                    %                     SaveInLier(obj.featPtManager, 'last', find(obj.featPtManager.localTrace.ptIcsX(:,end) ~= -1), true(length(find(obj.featPtManager.localTrace.ptIcsX(:,end) ~= -1)),1));
                    
                else
                    % %                     c2wTrans = obj.currRefRobotPoseWcs;
                    % %                     p2wTrans = obj.prevRefRobotPoseWcs;
                    % %                     c2pTrans = p2wTrans\c2wTrans;
                    % %                     p2wTransV = WcsPoseVecToPct(obj, obj.poseWcsList(end,:));
                    % %                     c2wTransV = p2wTransV*c2pTrans;
                    % %                     curloc = T(c2wTransV);
                    % %                     deltaAngle = obj.currRefRobotRotAngWcs - obj.prevRefRobotRotAngWcs;
                    % %                     robotPoseWcs = [curloc(1),curloc(3),obj.poseWcsList(end,3)+deltaAngle];
                    % %                     [~, ~, activeFeatInd] = GetActiveFeatures(obj.featPtManager, 'last');
                    % %                     inLierFlag = false(Rows(activeFeatInd),1);
                    % %                     SaveInLier(obj.featPtManager, 'last', activeFeatInd, inLierFlag);
                end
                if sum(inTrackFlag) > obj.configParam.min_num_pt_for_calculate2
                    %                 if sum(inLierFlag) > obj.configParam.min_num_pt_for_calculate2
                    if 0
                        fprintf(sprintf('### retracked points: %d/%d ###\n\n\n',length(reTrackId),sum(obj.featPtManager.localTrace.ptIcsX(:,end) ~= -1)));
                        SaveInLier(obj.featPtManager, 'last', find(obj.featPtManager.localTrace.ptIcsX(:,end) ~= -1), true(length(find(obj.featPtManager.localTrace.ptIcsX(:,end) ~= -1)),1));
                    else
                        %                         fprintf(sprintf('### AngleSpace inlier ratio: %d/%d ###\n\n\n', sum(validFlag),length(validFlag)));
                        %                         SaveInLier(obj.featPtManager, 'last', activeFeatInd, validFlag);
                        fprintf(sprintf('### AngleSpace inlier ratio: %d/%d ###\n\n\n', sum(inLierFlag),length(inLierFlag)));
                        SaveInLier(obj.featPtManager, 'last', activeFeatInd, inLierFlag);
                    end
                    
                else
                    [~, ~, activeFeatInd] = GetActiveFeatures(obj.featPtManager, 'last');
                    inLierFlag = false(Rows(activeFeatInd),1);
                    SaveInLier(obj.featPtManager, 'last', activeFeatInd, inLierFlag);
                end
            else
                c2wTrans = obj.currRefRobotPoseWcs;
                p2wTrans = obj.prevRefRobotPoseWcs;
                c2pTrans = p2wTrans\c2wTrans;
                p2wTransV = WcsPoseVecToPct(obj, obj.poseWcsList(end,:));
                c2wTransV = p2wTransV*c2pTrans;
                curloc = T(c2wTransV);
                deltaAngle1 = obj.currRefRobotRotAngWcs - obj.prevRefRobotRotAngWcs;
                robotPoseWcs = [curloc(1),curloc(3),obj.poseWcsList(end,3)+deltaAngle1];
                [~, ~, activeFeatInd] = GetActiveFeatures(obj.featPtManager, 'last');
                inLierFlag = false(Rows(activeFeatInd),1);
                SaveInLier(obj.featPtManager, 'last', activeFeatInd, inLierFlag);
            end
            
            if sum(inTrackFlag) <= obj.configParam.min_num_pt_for_calculate2
                
                c2wTrans = obj.currRefRobotPoseWcs;
                p2wTrans = obj.prevRefRobotPoseWcs;
                c2pTrans = p2wTrans\c2wTrans;
                p2wTransV = WcsPoseVecToPct(obj, obj.poseWcsList(end,:));
                c2wTransV = p2wTransV*c2pTrans;
                curloc = T(c2wTransV);
                deltaAngle1 = obj.currRefRobotRotAngWcs - obj.prevRefRobotRotAngWcs;
                robotPoseWcs = [curloc(1),curloc(3),obj.poseWcsList(end,3)+deltaAngle1];
                [~, ~, activeFeatInd] = GetActiveFeatures(obj.featPtManager, 'last');
                inLierFlag = false(Rows(activeFeatInd),1);
                SaveInLier(obj.featPtManager, 'last', activeFeatInd, inLierFlag);
            end
            
            %             robotPoseWcs;
            
            obj.accumP2CPNP = [obj.accumP2CPNP; obj.accumP2CPNP(end) + robotPoseWcs(3) - obj.poseWcsList(end,3)];
            if 0
                angErr = rad2deg([0;cumsum(meanAngP2C)] - obj.accumP2CRef);
            else
                %                  angErrPNP = rad2deg([0;cumsum(meanAngP2C)] - obj.accumP2CRef);
                angErrPNP = rad2deg(abs(obj.accumP2CPNP) - abs(obj.accumP2CRef));
                angErrPNP2 = rad2deg(abs(obj.accumP2CPNP2) - abs(obj.accumP2CRef));
%                 angErrPNP3 = rad2deg(abs(obj.accumP2CPNP3) - abs(obj.accumP2CRef));
                if length((obj.accumP2CPNP3)) < length((obj.accumP2CRef))
                    dltLen = length((obj.accumP2CRef)) - length((obj.accumP2CPNP3));
                    obj.accumP2CPNP3 = [obj.accumP2CPNP3; obj.accumP2CPNP3(end) + obj.accumP2CPNP2(end -dltLen + 1 : end) - obj.accumP2CPNP2(end -dltLen)];
                    angErrPNP3 = rad2deg(abs(obj.accumP2CPNP3) - abs(obj.accumP2CRef));
                else
                    angErrPNP3 = rad2deg(abs(obj.accumP2CPNP3) - abs(obj.accumP2CRef));
                end
            end
            
            n__ = floor(abs(rad2deg(obj.poseWcsList(end,3)))/360);
            pulseVec = [0 : 360 : (n__)*360]';
            %             [DD] = pdist2([pulseVec],abs(rad2deg(obj.poseWcsList(:,3))),'euclidean');
            [DD] = pdist2([pulseVec],abs(rad2deg(obj.accumP2CRef)),'euclidean');
            [~,idPulse] = min(DD');
            %             pulse = repmat(min(angErrPNP),size(obj.poseWcsList,1),1);
            if 0
                pulse = repmat(min(angErrPNP),size(obj.accumP2CRef,1),1);
                pulse(idPulse) = max((angErrPNP));
            else
                pulse = repmat(min(0),size(obj.accumP2CRef,1),1);
                pulse(idPulse) = max(abs(angErrPNP));
            end
            
            % %             pulse(idPulse) = max(abs(angErrPNP));
            
            if ~EnableDump
                if k2cRef > 0
                    figure(5);clf;subplot(2,5,[1 2]); hold on;plot(pulse,'-k');
                    if 0
                        plot(angErrPNP,'-r');plot(angErrPNP3,'-m');plot(angErr,'-b');plot([angErrUpper angErrLower],'-g');legend('one circle','ang pnp','ang pnp p2c','ang opt','ang range','Location','northwest');
                    end
                    angErrP2C_ = angErrP2C;
                    if ~isempty(obj.angOptManager.angErr_before_after)
                        
                        temp_angErr_before_after = rad2deg(obj.angOptManager.angErr_before_after);
                        temp_angErr_before_after2 = rad2deg(obj.angOptManager.angErr_before_after2);
                        
                        before_error = rad2deg(obj.angOptManager.meanDispErrMatDetailed_before_Diff - obj.angOptManager.meanDispErrMatDetailed_gt_Diff);
                        after_error = rad2deg(obj.angOptManager.meanDispErrMatDetailed_after_Diff - obj.angOptManager.meanDispErrMatDetailed_gt_Diff);
                        gt_error = rad2deg(obj.angOptManager.meanDispErrMatDetailed_before_Diff - obj.angOptManager.meanDispErrMatDetailed_gt_Diff);
                        
                        before_ = rad2deg(obj.angOptManager.meanDispErrMatDetailed_before_Diff - 0);
                        after_ = rad2deg(obj.angOptManager.meanDispErrMatDetailed_after_Diff - 0);
                        gt_ = rad2deg(obj.angOptManager.meanDispErrMatDetailed_before_Diff - 0);

                        
                        abh = [0;cumsum(obj.angOptManager.keyEndAng(:,4))];
%                         gtErr = rad2deg(abh - abs(obj.accumP2CRef(1:size(abh,1) + 0)));
                        gtErr = rad2deg(abs(obj.angOptManager.gtDispTheta(1:size(abh,1),1)) - abs(obj.accumP2CRef(1:size(abh,1) + 0)));
                        gtErr22 = rad2deg(abs(obj.angOptManager.gtDispTheta(1:size(abh,1),2)) - abs(obj.accumP2CRef(1:size(abh,1) + 0)));
                        
                        angLen = min(size(temp_angErr_before_after, 1), size(gtErr,1));
                        
                         abh1 = [0;cumsum(obj.angOptManager.keyEndAng(:,5))];
                         
                         gt_error = rad2deg(obj.angOptManager.meanDispErrMatDetailed_gt_Diff(1:angLen) - abh1(1:angLen));

                        
                        temp_angErr_before_after_gt = temp_angErr_before_after(1:angLen,:)  + gtErr(1:angLen,:) ;
                         temp_angErr_before_after_gt2 = [before_(1:angLen,:) after_(1:angLen,:)] - rad2deg(obj.accumP2CRef(1:angLen,:));
                        
                        
                        
                        
                        
                        if 0 % flipWeight
                            
                            figure,plot([(after_(1:angLen) - abs(rad2deg(obj.accumP2CRef(1:angLen))))    temp_angErr_before_after(1:angLen,2) + gtErr(1:angLen,1)])
                            
                            
                            figure,plot(temp_angErr_before_after_gt - temp_angErr_before_after_gt2);
                            figure,plot((gtErr - gt_error ) )
                            abh1 = [0;cumsum(obj.angOptManager.keyEndAng(:,5))];
                            figure(9),clf;plot([rad2deg(abh1(2:end) - cumsum(single(diff(obj.accumP2CRef(1:length(abh1))))))  (gtErr(2:end) - gt_error(2:end))])
                        end
                        
                        
                        
                        
                        
                        
                        
                        
                        if 0
                            plot(rad2deg(obj.angOptManager.angErr_before_after(:,1)),'-c');plot(rad2deg(obj.angOptManager.angErr_before_after(:,2)),'Color',[2/3 0 1]);
                        else
                            plot(temp_angErr_before_after(1:angLen,1) + gtErr(1:angLen,1),'-c');plot(temp_angErr_before_after(1:angLen,2) + gtErr(1:angLen,1),'-x','Color',[2/3 0 1]);
                            plot(temp_angErr_before_after2(1:angLen,1) + gtErr(1:angLen,1),'-r');plot(temp_angErr_before_after2(1:angLen,2) + gtErr(1:angLen,1),'-x','Color',[0 0 1]);
%                             plot(temp_angErr_before_after(1:angLen,2),'-g')
                        end
                        
                        plot(gtErr,'-xy');plot(gtErr22,'-oy')
                        title(sprintf('flipWeight: %d', double(flipWeight)));
                  if 0
                        legend('one circle','ang pnp','ang pnp p2c','ang opt','ang range','','old','new','theta - gt','Location','northwest'); 
                  elseif 0
                      legend('one circle','old disp - gt theta','new disp - gt theta','gt disp - gt theta','new disp - gt disp','Location','northwest'); 
                  else
                      legend('one circle','old short - gt theta','new short - gt theta','old long - gt theta','new long - gt theta','gt short - gt theta','gt long - gt theta','Location','northwest'); 
                  end
                    end
                    
                    %                 figure(5);clf;hold on;plot(angErrPNP,'-r');plot(angErrPNP2,'-g');plot(angErr,'-b');plot(pulse,'-k');legend('ang pnp','ang pnp p2c','ang opt','one circle');
                else
                    figure(5);clf;subplot(2,5,[1 2]); hold on;plot(-pulse,'-k');
                    if 0
                        plot(-angErrPNP,'-r');plot(-angErrPNP3,'-m');plot(-angErr,'-b');plot([-angErrUpper -angErrLower],'-g');legend('one circle','ang pnp','ang pnp p2c','ang opt','ang range','Location','northwest');
                    end
                    angErrP2C_ = -angErrP2C;
                    if ~isempty(obj.angOptManager.angErr_before_after)
                        temp_angErr_before_after = -rad2deg(obj.angOptManager.angErr_before_after);
                        temp_angErr_before_after2 = -rad2deg(obj.angOptManager.angErr_before_after2);

                        
                        before_error = -(abs(rad2deg(obj.angOptManager.meanDispErrMatDetailed_before_Diff)) - abs(rad2deg(obj.angOptManager.meanDispErrMatDetailed_gt_Diff)));
                        after_error = -(abs(rad2deg(obj.angOptManager.meanDispErrMatDetailed_after_Diff)) - abs(rad2deg(obj.angOptManager.meanDispErrMatDetailed_gt_Diff)));
                        gt_error = -(abs(rad2deg(obj.angOptManager.meanDispErrMatDetailed_before_Diff)) - abs(rad2deg(obj.angOptManager.meanDispErrMatDetailed_gt_Diff)));
                        
                        before_ = rad2deg(obj.angOptManager.meanDispErrMatDetailed_before_Diff - 0);
                        after_ = rad2deg(obj.angOptManager.meanDispErrMatDetailed_after_Diff - 0);
                        gt_ = rad2deg(obj.angOptManager.meanDispErrMatDetailed_before_Diff - 0);
                        
                        
                        
                        
                        
                        
                        
                        abh = [0;cumsum(obj.angOptManager.keyEndAng(:,4))];
                        %                         gtErr = -rad2deg(abh - abs(obj.accumP2CRef(1:size(abh,1) + 0)));
                        gtErr = -rad2deg(abs(obj.angOptManager.gtDispTheta(1:size(abh,1),1)) - abs(obj.accumP2CRef(1:size(abh,1) + 0)));
                        gtErr22 = -rad2deg(abs(obj.angOptManager.gtDispTheta(1:size(abh,1),2)) - abs(obj.accumP2CRef(1:size(abh,1) + 0)));
                        
                        angLen = min(size(temp_angErr_before_after, 1), size(gtErr,1));
                        
                        
                        if 0 % flipWeight
                            figure,plot([-(after_(1:angLen) - abs(rad2deg(obj.accumP2CRef(1:angLen))))    temp_angErr_before_after(1:angLen,2) + gtErr(1:angLen,1)])
                            
                            abh1 = [0;cumsum(obj.angOptManager.keyEndAng(:,5))];
                            figure(9),clf;plot(abh1(2:end) - cumsum(single(diff(obj.accumP2CRef(1:length(abh1))))))
                        end
                        
                        
                        

                        if 0
                            plot(-rad2deg(obj.angOptManager.angErr_before_after(:,1)),'-c');plot(-rad2deg(obj.angOptManager.angErr_before_after(:,2)),'Color',[2/3 0 1]);
                        else
                            plot(temp_angErr_before_after(1:angLen,1) + gtErr(1:angLen,1),'-c');plot(temp_angErr_before_after(1:angLen,2) + gtErr(1:angLen,1),'-x','Color',[2/3 0 1]);
                            plot(temp_angErr_before_after2(1:angLen,1) + gtErr(1:angLen,1),'-r');plot(temp_angErr_before_after2(1:angLen,2) + gtErr(1:angLen,1),'-x','Color',[0 0 1]);
%                             plot(temp_angErr_before_after(1:angLen,2),'-g')
                        end
                   
                        plot(gtErr,'-xy');plot(gtErr22,'-oy')
                        title(sprintf('flipWeight: %d', double(flipWeight)));
                        if 0
                            legend('one circle','ang pnp','ang pnp p2c','ang opt','ang range','','old','new','theta - gt','Location','northwest');
                        elseif 0
                            legend('one circle','old disp - gt theta','new disp - gt theta','gt disp - gt theta','new disp - gt disp','Location','northwest');
                        else
                            legend('one circle','old short - gt theta','new short - gt theta','old long - gt theta','new long - gt theta','gt short - gt theta','gt long - gt theta','Location','northwest'); 
                            
                        end
                    end
                    %                 figure(5);clf;hold on;plot(-angErrPNP,'-r');plot(-angErrPNP2,'-g');plot(-angErr,'-b');plot(-pulse,'-k');legend('ang pnp','ang pnp p2c','ang opt','one circle');
                end
                [dispErrDistriMat, dispErrExpMat] = PlotDispErr(obj, 5, [1 1000000]);
                if 0
                    subplot(2,1,2);plot(zeros(length(obj.dispErrExpStack),1), 'k'); hold on; plot(obj.dispErrExpStack, 'b');plot(obj.dispErrExpStack2(:,1), 'r');plot(obj.dispErrExpStack2(:,2), 'g');plot(angErrP2C,'-m');legend('gt','gt - cur','cur - key','curAvg - key','p2c err');title('cur(gtDisp - stereoDisp)');
                elseif 0
                    subplot(2,1,2);plot(zeros(length(obj.dispErrExpStack),1), 'k'); hold on; plot(obj.dispErrExpStack2(:,1), 'b');plot(obj.dispErrExpStack2(:,2), 'r');plot(obj.dispErrExpStack2(:,3), 'g');plot(angErrP2C_,'-m');legend('gt','gt - cur','key - cur','key - curAvg','p2c err');title('cur(gtDisp - stereoDisp)');
                elseif 1
                    if 0
                        subplot(2,5,[6 7]);plot(zeros(length(obj.dispErrExpStack),1), 'k'); hold on; plot(obj.dispErrExpStack2(:,1), 'c');plot(dispErrDistriMat(:,1), dispErrDistriMat(:,4), 'b');plot(dispErrDistriMat(:,1), dispErrDistriMat(:,2), 'r');plot(dispErrDistriMat(:,1), dispErrDistriMat(:,3), 'g');plot(angErrP2C_,'-m');legend('gt','gt - cur','gt - key','gt - c in k','gt - cAvg in k', 'p2c err');title('cur(gtDisp - stereoDisp)');
                    elseif 1
                        if 1 % isempty(obj.angOptManager.keyEndAng)
                            subplot(2,5,[6 7]);cla;plot(zeros(1,size(dispErrExpMat,1)),'-k');hold on;plot(dispErrExpMat(:,1),dispErrExpMat(:,2),'-b');plot(dispErrExpMat(:,1),dispErrExpMat(:,3),'-g');legend('zero','k','avg');
                        else
                            
                            %                         angErr_before_after = [obj.angOptManager.keyEndAng(:,1) rad2deg(obj.angOptManager.keyEndAng(:,2) - obj.angOptManager.keyEndAng(:,4)) rad2deg(obj.angOptManager.keyEndAng(:,3) - obj.angOptManager.keyEndAng(:,4))];
                            if k2cRef > 0
                                subplot(2,5,[6 7]);cla;plot(zeros(1,size(dispErrExpMat,1)),'-k');hold on;plot(dispErrExpMat(:,1),dispErrExpMat(:,2),'-b');plot(dispErrExpMat(:,1),dispErrExpMat(:,3),'-g');legend('zero','k','avg');
                            else
                                subplot(2,5,[6 7]);cla;plot(zeros(1,size(dispErrExpMat,1)),'-k');hold on;plot(dispErrExpMat(:,1),dispErrExpMat(:,2),'-b');plot(dispErrExpMat(:,1),dispErrExpMat(:,3),'-g');legend('zero','k','avg');
                            end
                            
                            
                        end
                        if 0 % ~isempty(obj.angOptManager.angErr_before_after)
                            angErr_before_after = [obj.angOptManager.keyEndAng(:,1) rad2deg(obj.angOptManager.keyEndAng(:,2) - obj.angOptManager.keyEndAng(:,4)) rad2deg(obj.angOptManager.keyEndAng(:,3) - obj.angOptManager.keyEndAng(:,4))];
                            
                            if k2cRef > 0
                                plot(rad2deg(diff(obj.angOptManager.angErr_before_after(:,1))),'-b');hold on;plot(rad2deg(diff(obj.angOptManager.angErr_before_after(:,2))),'-g');legend('zero','k','avg','k p2c','avg p2c');
                                %                            subplot(2,5,[6 7]);cla;plot(zeros(1,size(dispErrExpMat,1)),'-k');hold on;plot(dispErrExpMat(:,1),dispErrExpMat(:,2),'-b');plot(dispErrExpMat(:,1),dispErrExpMat(:,3),'-g');legend('zero','k','avg');
                            else
                                plot(-rad2deg(diff(obj.angOptManager.angErr_before_after(:,1))),'-b');hold on;plot(-rad2deg(diff(obj.angOptManager.angErr_before_after(:,2))),'-g');legend('zero','k','avg','k p2c','avg p2c');
                                %                            plot(-rad2deg(diff(obj.angOptManager.angErr_before_after(:,1))),'-b');hold on;plot(-rad2deg(diff(obj.angOptManager.angErr_before_after(:,2))),'-g');legend('zero','k','avg','k p2c','avg p2c');
                            end
                        end
                        if  ~isempty(obj.angOptManager.keyEndAng)
                            angErr_before_after = [obj.angOptManager.keyEndAng(:,1) rad2deg(obj.angOptManager.keyEndAng(:,2) - obj.angOptManager.keyEndAng(:,4)) rad2deg(obj.angOptManager.keyEndAng(:,3) - obj.angOptManager.keyEndAng(:,4))];
                            angErr_before_after2 = [obj.angOptManager.keyEndAng(:,1) rad2deg(cumsum(obj.angOptManager.keyEndAng(:,2)) - cumsum(obj.angOptManager.keyEndAng(:,4))) rad2deg(cumsum(obj.angOptManager.keyEndAng(:,3)) - cumsum(obj.angOptManager.keyEndAng(:,4)))];
                            angErr_before_after__1 = [obj.angOptManager.keyEndAng2(:,1) rad2deg(obj.angOptManager.keyEndAng2(:,2) - obj.angOptManager.keyEndAng2(:,4)) rad2deg(obj.angOptManager.keyEndAng2(:,3) - obj.angOptManager.keyEndAng2(:,4))];
                            if 1 % k2cRef > 0
                                %                            plot(rad2deg(diff(obj.angOptManager.angErr_before_after(:,1))),'-b');hold on;plot(rad2deg(diff(obj.angOptManager.angErr_before_after(:,2))),'-g');legend('zero','k','avg','k p2c','avg p2c');
                                plot(angErr_before_after(:,1),angErr_before_after(:,2),'-c');plot(angErr_before_after(:,1),angErr_before_after(:,3),'-r');
%                                 legend('zero','k','avg','k p2c','avg p2c');
                                plot(angErr_before_after__1(:,1),angErr_before_after__1(:,3),'-k');legend('zero','k','avg','k p2c','avg p2c', 'avg p2c 2');
                            else
                                %                            plot(-rad2deg(diff(obj.angOptManager.angErr_before_after(:,1))),'-b');hold on;plot(-rad2deg(diff(obj.angOptManager.angErr_before_after(:,2))),'-g');legend('zero','k','avg','k p2c','avg p2c');
                                plot(angErr_before_after(:,1),-angErr_before_after(:,2),'-c');plot(angErr_before_after(:,1),-angErr_before_after(:,3),'-r');legend('zero','k','avg','k p2c','avg p2c');
                            end
                            
                            if flipWeight
                                if 0
                                    figure,plot( [0;cumsum(angErr_before_after(:,3))] - temp_angErr_before_after(1:angLen,2));
                                end
                                
                            end
                            
                            
                        end
                    else
                        subplot(2,5,[6 7]);plot(zeros(length(obj.dispErrExpStack),1), 'k'); hold on; plot(obj.dispErrExpStack2(:,1), 'c');plot(dispErrDistriMat(:,1), dispErrDistriMat(:,4), 'b');plot(dispErrDistriMat(:,1), dispErrDistriMat(:,3), 'r');plot(dispErrDistriMat(:,1), dispErrDistriMat(:,3), 'g');plot(angErrP2C_,'-m');legend('gt','gt - cur','gt - key','gt - c in k','gt - cAvg in k', 'p2c err');title('cur(gtDisp - stereoDisp)');
                    end
                    subplot(2,5,8);imshow([obj.angOptManager.angOptErrMatUniq], []); title('p2c(ang - gt)');
                    try
                        subplot(2,5,9);imshow([obj.angOptManager.dispErrMatDetailedUniq], []); title('avg disp err (gt - Avg)');
                    catch
                        aghl = 1;
                    end
                    subplot(2,5,10);imshow([obj.angOptManager.angOptErrK2CMatUniq], []); title('k2c(ang - gt)');
                else
                    subplot(2,1,2);plot(zeros(length(obj.dispErrExpStack),1), 'k'); hold on; plot(obj.dispErrExpStack2(:,1), 'b');plot(obj.dispErrExpStack2(:,3), 'r');plot(obj.dispErrExpStack2(:,3), 'g');plot(angErrP2C_,'-m');legend('gt','gt - cur','cur - key','curAvg - key','p2c err');title('cur(gtDisp - stereoDisp)');
                end
                
                try
                    aaaa = diff(obj.angOptManager.angOptErrK2CMatUniq')';
                    bbbb = obj.angOptManager.angOptErrMatUniq;
                    abErr = aaaa - bbbb(:,2:end);
                catch
                    safbiku = 1;
                end
                if 0
                    
                    figure,hold on;plot(zeros(1,size(dispErrExpMat,1)),'-k');hold on;plot(dispErrExpMat(:,1),dispErrExpMat(:,2),'-b');plot(dispErrExpMat(:,1),dispErrExpMat(:,3),'-g');plot(angErr_before_after(:,1),angErr_before_after(:,2),'-c');plot(angErr_before_after(:,1),angErr_before_after(:,3),'-r');plot(angErr_before_after__1(:,1),angErr_before_after__1(:,3),'-k');legend('zero','k','avg','k p2c','avg p2c', 'avg p2c 2');
                    figure,plot([cumsum(angErr_before_after(:,3)) cumsum(angErr_before_after__1(:,3))]);legend('short','long');
                    figure,hist(angErr_before_after__1(:,3)./angErr_before_after(:,3), 100);                    
                    
                    
                    id123 = find(dispErrExpMat(:,2) > 0);
                    id321 = find(dispErrExpMat(:,2) < 0);
                    
                    common_ = find(ismember(dispErrExpMat(:,1), angErr_before_after(:,1)));
                    dispErrExpMatCom = dispErrExpMat(common_,:);
                    
                    
                    [~,idOld] = sort(dispErrExpMatCom(:,2));
                    [~,idNew] = sort(dispErrExpMatCom(:,3));
% %                     figure,subplot(1,2,1);plot(dispErrExpMatCom(idOld,2));hold on;plot(angErr_before_after(idOld,2));legend('dispErr','angErr');title('old');grid on;
% %                            subplot(1,2,2);plot(dispErrExpMatCom(idNew,3));hold on;plot(angErr_before_after(idNew,3));legend('dispErr','angErr');title('new');grid on;
                    
                    dispThr = -0.01;0; 0.05;
                    % new
                    id12345 = find(dispErrExpMatCom(:,3) > -dispThr);
                    id54321 = find(dispErrExpMatCom(:,3) < dispThr);
                    new_plus = sum(angErr_before_after(id12345,3) > 0)/length(id12345);
                    new_minus = sum(angErr_before_after(id54321,3) < 0)/length(id54321);
                    % old
                    id12345_ = find(dispErrExpMatCom(:,2) > -dispThr);
                    id54321_ = find(dispErrExpMatCom(:,2) < dispThr);
                    old_plus = sum(angErr_before_after(id12345_,2) > 0)/length(id12345_);
                    old_minus = sum(angErr_before_after(id54321_,2) < 0)/length(id54321_);
                    
                    figure,subplot(4,2,[1 2]);plot(zeros(1,size(dispErrExpMat,1)),'-k');hold on;plot(dispErrExpMat(:,1),dispErrExpMat(:,2),'-b');subplot(4,2,[1 2]);plot(dispErrExpMat(:,1),dispErrExpMat(:,3),'-g');legend('zero','k','avg');
                    try
                        plot(angErr_before_after(:,1),angErr_before_after(:,2),'-c');plot(angErr_before_after(:,1),angErr_before_after(:,3),'-r');legend('zero','k','avg','k p2c','avg p2c');
                    catch
                        sghoij = 1;
                    end
                    title(sprintf('old angErr>0 / dispErr>0:   %0.3f || old angErr<0 / dispErr<0:   %0.3f\nnew angErr>0 / dispErr>0:   %0.3f || new angErr<0 / dispErr<0:   %0.3f',old_plus, old_minus, new_plus, new_minus));
                    subplot(4,2,3);plot(id123, dispErrExpMat(id123,2) - dispErrExpMat(id123,3),'-x');subplot(4,2,4);plot(id321, dispErrExpMat(id321,2) - dispErrExpMat(id321,3),'-x');
                    subplot(4,2,5);hist(dispErrExpMat(id123,2) - dispErrExpMat(id123,3));title('old > 0');subplot(4,2,6);hist(dispErrExpMat(id321,2) - dispErrExpMat(id321,3));title('old < 0, old - new')
                    subplot(4,2,7);plot(dispErrExpMatCom(idOld,2));hold on;plot(angErr_before_after(idOld,2));legend('dispErr','angErr');title('old');grid on;
                    subplot(4,2,8);plot(dispErrExpMatCom(idNew,3));hold on;plot(angErr_before_after(idNew,3));legend('dispErr','angErr');title('new');grid on;
                    
                    
                    figure,plot(rad2deg(obj.angOptManager.angErr_before_after(2:2+size(angErr_before_after,1)-1,2)) - cumsum(angErr_before_after(:,3)))
                    
                    figure,plot(rad2deg(obj.angOptManager.angErr_before_after(2:2+size(angErr_before_after,1)-1,2)) - (angErr_before_after2(:,3)))
                    
                    
                    figure,plot(cumsum(angErr_before_after(:,3)) - angErr_before_after2(:,3))
                    
                    
                    abhgg = cumsum(obj.angOptManager.keyEndAng(:,4));
                    if k2cRef > 0
                        figure,plot(rad2deg(abhgg - abs(obj.accumP2CRef(2:size(abhgg,1) + 1))));
                    else  
                        figure,plot(-rad2deg(abhgg - abs(obj.accumP2CRef(2:size(abhgg,1) + 1))));
                    end
                end
                
            end
            if ~EnableDump
                saveas(gcf,fullfile(probPath,sprintf('final_error_%05d.fig',999+double(flipWeight))));
            end
            asdgfklugfkf = 1;
        end
        
        function [depthMap, dispMap] = GetDepthMap(obj, imgL, imgR)
            assert(~isempty(imgL) && ~isempty(imgR), 'Input images should not be empty.');
            [princpPtL, princpPtR] = Get(obj.camModel, 'PinholePrincpPt', obj.scaleLvl);
            [focLenL, focLenR] = Get(obj.camModel, 'PinholeFocLen', obj.scaleLvl);
            baseline = GetBaseline(obj.camModel);
            assert(all(focLenL == focLenR), 'Rectified camera pairs should have the same pinhole focal length');
            if isempty(obj.pointCloudManager.depthMaps)
                prevKeyMinDepth = -1;
            else
                %                 intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,1);
                intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,obj.scaleLvl);
                depthMap = obj.pointCloudManager.depthMaps(:,:,end);
                flag = Filter3DPoint(obj.pointCloudManager, depthMap, intrMat);
                prevKeyMinDepth = mean(depthMap(flag));
            end
            
%             [depthMap, dispMap] = CalcDepthMap(obj.stereoDepthMap, imgL, imgR, focLenL, princpPtL(1), princpPtR(1), baseline, prevKeyMinDepth);
            [depthMap, dispMap] = CalcDepthMap(obj.stereoDepthMap, imgL, imgR, focLenL, princpPtL(1), princpPtR(1), baseline, prevKeyMinDepth);
        end
        
        function p2cPredRotAng = PredictRotateAngle(obj,k2cBodySensorRotAng)
            [prvPtIcs, curPtIcs] = GetFeaturePairs(obj.featPtManager, -1, 0);
            intrMat = Get(obj.camModel, 'PinholeIntrMat', 1, obj.scaleLvl);
            p2cPredRotAng = PredictRotateAngle(obj.rotAngPredictor, prvPtIcs, curPtIcs, intrMat, obj.coordSysAligner, 1, 0, k2cBodySensorRotAng + obj.pureRotationAngPredictor.configParam.init_angle_range);
        end
        function p2cPredRotAng = PredictRotateAngle2(obj,k2cBodySensorRotAng,stamp,goldenPose,ind)
            if isempty(ind)
                [prvPtIcs, curPtIcs] = GetFeaturePairs(obj.featPtManager, -1, 0);
            else
                [prvPtIcs, curPtIcs] = GetFeaturePairs(obj.featPtManager, 'key', 'last');
            end
            intrMat = Get(obj.camModel, 'PinholeIntrMat', 1, obj.scaleLvl);
            p2cPredRotAng = PredictRotateAngle2(obj.rotAngPredictor, prvPtIcs, curPtIcs, intrMat, obj.coordSysAligner, 1, 0, k2cBodySensorRotAng + obj.pureRotationAngPredictor.configParam.init_angle_range,stamp,goldenPose);
        end
        
        function [k2cRotAngs, k2cCamTransDirAngs, inPolygonFlag, curBodyOrigUncertPolygonBcsKey] = PredictRobotPose(obj, prv2curPredRotAng, prv2curPredDisplacement)
            %             firstKeyInd = GetFirstKeyFrameInd(obj);
            %             [w2pRotAng, w2pTransVecBcsKey] = GetDeltaPose(obj, firstKeyInd, 'last', 'AngleT0');
            %             [k2cRotAngs, k2cCamTransDirAngs, inPolygonFlag, curBodyOrigUncertPolygonBcsPrv] = ...
            %                 PredictRobotPose(obj.robotPosePredictor, w2pRotAng, w2pTransVecBcsKey, prv2curPredRotAng, prv2curPredDisplacement, obj.coordSysAligner, 1);
            %             lastKeyInd = GetLastKeyFrameInd(obj);
            %             k2wP = WcsPoseVecToPct(obj, obj.poseWcsList(lastKeyInd,:));
            %             curBodyOrigUncertPolygonBcsKey = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(-w2pRotAng), w2pTransVecBcsKey) * curBodyOrigUncertPolygonBcsPrv;
            %             curBodyOrigUncertPolygonBcsKey = Inv(k2wP)*curBodyOrigUncertPolygonBcsPrv;
            
            lastKeyInd = GetLastKeyFrameInd(obj);
            %             [k2pRotAng, k2pTransVecBcsKey] = GetDeltaPose(obj, lastKeyInd, 'last', 'AngleT0');
            k2wP = WcsPoseVecToPct(obj, obj.poseWcsList(lastKeyInd,:));
            %             Xw = [obj.poseWcsList(end,1);0;obj.poseWcsList(end,2)];
            %             k2pTransVecBcsKey = Inv(k2wP)*Xw;
            
            p2wP = WcsPoseVecToPct(obj, obj.poseWcsList(end,:));
            p2kP = k2wP \ p2wP;
            pInk = T(p2kP);  %Inv(k2wP)*Xw;
            k2pRotAng = obj.poseWcsList(end,3) - obj.poseWcsList(lastKeyInd,3);
            % prv2curPredRotAng < 0 -> turn right
            [k2cRotAngs, k2cCamTransDirAngs, inPolygonFlag, curBodyOrigUncertPolygonBcsPrv] = PredictRobotPose(obj.robotPosePredictor, k2pRotAng, pInk, prv2curPredRotAng, prv2curPredDisplacement, obj.coordSysAligner, 1);
            
            %             k2wP = WcsPoseVecToPct(obj, obj.poseWcsList(lastKeyInd,:));
            curBodyOrigUncertPolygonBcsKey = curBodyOrigUncertPolygonBcsPrv;  %PointCoordTransformer(BaseLocalizer.RotMatFromAngle(-k2pRotAng), pInk) * curBodyOrigUncertPolygonBcsPrv;
            %             curBodyOrigUncertPolygonBcsKey = Inv(k2wP)*curBodyOrigUncertPolygonBcsPrv;
        end
        
        function [k2cRotAng, k2cCamTransDirAng, projScore] = EstimateInAngleSpace(obj, k2cRotAngs, k2cCamTransDirAngs, inPolygonFlag)
            intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,1);
            [keyPtIcs, curPtIcs] = GetFeaturePairs(obj.featPtManager, 'key', 'last');
            [k2cRotAng, k2cCamTransDirAng, projScore] = EvaluatePosterierProbability(obj.angleSpace, keyPtIcs, curPtIcs, k2cRotAngs, k2cCamTransDirAngs, inPolygonFlag, obj.coordSysAligner, 1,intrMat);
        end
        
        function [k2cRotAng, k2cCamTransDirAng, k2cScaleFactor, featInlierFlag] = EstimateScaleFactor(obj, k2cRotAngCandidates, k2cCamTransDirAngCandidates,camInd, projScore)
            [keyPtIcs, curPtIcs, keyPtCcsZ] = GetFeaturePairs(obj.featPtManager, 'key', 'last');
            flag = projScore > 0.9 & keyPtCcsZ' > 0;
            flag = flag';
            if sum(flag(:)) ~= 0
                numberCandiateAng = length(k2cRotAngCandidates);
                intrMat = Get(obj.camModel, 'PinholeIntrMat', 1, obj.scaleLvl);
                k2cScaleFactor_ = zeros(numberCandiateAng,1);
                rsq = zeros(numberCandiateAng,1);
                reProjErr = zeros(numberCandiateAng,1);
                validflag = false(Rows(keyPtIcs),numberCandiateAng);
                for anglePairID = 1: numberCandiateAng
                    rotMatCcs = BodyRotateRotMatCcs(obj.coordSysAligner, k2cRotAngCandidates(anglePairID), camInd);
                    transVecCcs = -R(GetPctBody2Cam(obj.coordSysAligner))'*[cos(k2cCamTransDirAngCandidates(anglePairID)); 0; sin(k2cCamTransDirAngCandidates(anglePairID))];
                    numberPt = Rows(keyPtIcs(flag(:,anglePairID),:));
                    y_ = rotMatCcs / intrMat * HomoCoord(keyPtIcs(flag(:,anglePairID),:)',1);
                    y_ = [y_(1,:).*keyPtCcsZ(flag(:,anglePairID))';y_(2,:).*keyPtCcsZ(flag(:,anglePairID))';y_(3,:).*keyPtCcsZ(flag(:,anglePairID))'];
                    x_ = intrMat \ HomoCoord(curPtIcs(flag(:,anglePairID),:)',1);
                    x = zeros(3*numberPt,numberPt);
                    y = zeros(3*numberPt,1);
                    t_ = zeros(3*numberPt,1);
                    for i = 1:numberPt
                        x(3*i - 2:3*i,i) = x_(:,i);
                        t_(3*i - 2:3*i,1) = -transVecCcs;
                        y(3*i - 2:3*i,1) = y_(:,i);
                    end
                    x = [x,t_];
                    tau = x\y;
                    yCal = x*tau;
                    rsq(anglePairID) = 1 - sum((y - yCal).^2)/sum((y - mean(y)).^2);
                    
                    k2cScaleFactor_(anglePairID) = tau(end);
                    
                    metricKeyPtMetricCcs = intrMat\HomoCoord(keyPtIcs',1) ;
                    metricKeyPtMetricCcs = normc(metricKeyPtMetricCcs);
                    metricKeyPtMetricCcs = metricKeyPtMetricCcs';
                    scale = keyPtCcsZ./metricKeyPtMetricCcs(:,3);
                    keyPtCcsXYZ = scale.*metricKeyPtMetricCcs;
                    
                    outLierThreshold = obj.configParam.reproject_outlier_threshold;
                    transVecCcs = tau(end)*transVecCcs;
                    
                    [reProjErr(anglePairID),dis,validflag(:,anglePairID)] = VisualLocalizer.CalculateReprojectError(intrMat,...
                        keyPtCcsXYZ,curPtIcs,rotMatCcs,transVecCcs,flag(:,anglePairID),outLierThreshold);
                end
                [~, ind] = min(reProjErr);
                k2cRotAng = k2cRotAngCandidates(ind);
                k2cCamTransDirAng = k2cCamTransDirAngCandidates(ind);
                k2cScaleFactor = k2cScaleFactor_(ind);
                featInlierFlag = validflag(:,ind);
                debug.debug_903x_FindThetaAlphaScale(obj,k2cRotAng,k2cCamTransDirAng,k2cScaleFactor,ind,keyPtIcs,intrMat,flag,keyPtCcsZ,curPtIcs)
            else
                k2cRotAng = k2cRotAngCandidates(1);
                k2cCamTransDirAng = k2cCamTransDirAngCandidates(1);
                k2cScaleFactor = -1;
                featInlierFlag = false(Rows(keyPtIcs),1);
            end
        end
        
        function robotPoseWcs = EstimateLocation(obj, k2cRotAngBcs, k2cCamTransDirAngBcs, k2cScaleFactor, curBodyOrigUncertPolygonBcsKey)
            %             global TOPOGRAPHY_FIG
            %             figure(TOPOGRAPHY_FIG)
            %             hold on
            %             plot(k2cCamTransDirAngBcs,k2cRotAngBcs,'*g')
            %             hold off
            rotMatCcs = BodyRotateRotMatCcs(obj.coordSysAligner, k2cRotAngBcs, 1);
            transVecCcs = R(GetPctBody2Cam(obj.coordSysAligner))*[cos(k2cCamTransDirAngBcs); 0; sin(k2cCamTransDirAngBcs)];
            tK2CCcs1 = [rotMatCcs transVecCcs;0 0 0 1];tK2CCcs2 = [rotMatCcs 2*transVecCcs;0 0 0 1];
            tC2KCcs1 = inv(tK2CCcs1);tC2KCcs2 = inv(tK2CCcs2);
            c2b = Inv(GetPctBody2Cam(obj.coordSysAligner, 1));
            c2b = c2b.transformMat;
            tC2KBcs1 = c2b*tC2KCcs1*inv(c2b); tC2KBcs2 = c2b*tC2KCcs2*inv(c2b);
            
            
            camOCInCurBcs = T(Inv(GetPctBody2Cam(obj.coordSysAligner, 1)));
            x0 = camOCInCurBcs(1)/1000;
            z0 = camOCInCurBcs(3)/1000;
            gamma = k2cRotAngBcs + k2cCamTransDirAngBcs;
            c = cos(gamma);
            s = sin(gamma);
            c2kR = [cos(-k2cRotAngBcs),sin(-k2cRotAngBcs);-sin(-k2cRotAngBcs),cos(-k2cRotAngBcs)];
            temp1 = c2kR * [x0;z0];
            x_ = temp1(1);
            z_ = temp1(2);
            lineVec(1) = s;
            lineVec(2) = -c;
            lineVec(3) = (s*x_ - c*z_ + c*z0 - s*x0);
            lineVec0 = lineVec;
            
            
            lineVec = cross([tC2KBcs1([1 3 ],4);1], [tC2KBcs2([1 3 ],4);1])';
            lineVec = lineVec./norm(lineVec([1 2]));
            lineVec(3) = lineVec(3)/1000;
            
            numPolygonCor = Cols(curBodyOrigUncertPolygonBcsKey);
            t = zeros(numPolygonCor,1);
            x = zeros(numPolygonCor,2);
            for i = 1:numPolygonCor
                x1 = [curBodyOrigUncertPolygonBcsKey(1,i),curBodyOrigUncertPolygonBcsKey(3,i)];
                x2 = [curBodyOrigUncertPolygonBcsKey(1,mod(i,numPolygonCor)+1),curBodyOrigUncertPolygonBcsKey(3,mod(i,numPolygonCor)+1)];
                dx = x2 - x1;
                t(i) = - (lineVec(3) + lineVec(1)*x1(1) + lineVec(2)*x1(2)) ...
                    / (lineVec(1)*dx(1) + lineVec(2)*dx(2));
                x(i,:) = x1 + dx*t(i);
            end
            
            len1 = min(curBodyOrigUncertPolygonBcsKey(3,:)) - 0.005;
            len2 = max(curBodyOrigUncertPolygonBcsKey(3,:)) + 0.005;
            figure(11), hold on
            plot(curBodyOrigUncertPolygonBcsKey(1,[1:end 1]),curBodyOrigUncertPolygonBcsKey(3,[1:end 1]),'-xb');axis equal;
            hold on;line([(-lineVec(2)*len1-lineVec(3))/lineVec(1) (-lineVec(2)*len2-lineVec(3))/lineVec(1)]',[len1 len2]','Color',[1 0 0],'LineWidth',1);
            
            
            flag1 = t < 1 & t >=0;
            %             assert(~all(flag == 0),'there is no interaction between line and polygon!')
            if sum(flag1) ~= 2
                obj.noIntersection = obj.noIntersection + 1;
            end
            if sum(flag1) == 0
                debug.debug_908x_CheckPolygonAndLineInteraction(flag1);
                dis = zeros(1,numPolygonCor);
                for i = 1:numPolygonCor
                    p = [curBodyOrigUncertPolygonBcsKey(1,i), curBodyOrigUncertPolygonBcsKey(3,i),1];
                    dis(i) = abs(dot(lineVec,p))/norm(lineVec);
                end
                [mindis,ind] = min(dis);
                if mindis <= 0.002
                    X1 = [curBodyOrigUncertPolygonBcsKey(1,ind), curBodyOrigUncertPolygonBcsKey(3,ind)];
                    X2 = [curBodyOrigUncertPolygonBcsKey(1,ind), curBodyOrigUncertPolygonBcsKey(3,ind)];
                else
                    warning('Too far between line and polygon')
                    X1 = [curBodyOrigUncertPolygonBcsKey(1,ind), curBodyOrigUncertPolygonBcsKey(3,ind)];
                    X2 = [curBodyOrigUncertPolygonBcsKey(1,ind), curBodyOrigUncertPolygonBcsKey(3,ind)];
                end
            elseif sum(flag1) == 2
                temp2 = x(flag1,:);
                X1 = temp2(1,:);
                X2 = temp2(2,:);
            elseif sum(flag1) == 1
                temp2 = x(flag1,:);
                X1 = temp2;
                X2 = temp2;
            end
            
            %             plot(X1(1),X1(2),'or')
            %             plot(X2(1),X2(2),'or')
            ratio = 0.5;
            curPoseBcsKey = [ratio*X1 + (1-ratio)*X2, k2cRotAngBcs];
            % find current error range
            
            slop = tan(k2cRotAngBcs);
            lineVec(1) = slop;
            lineVec(2) = -1;
            lineVec(3) = -slop*curPoseBcsKey(1)+curPoseBcsKey(2);
            
            t = zeros(numPolygonCor,1);
            x = zeros(numPolygonCor,2);
            for i = 1:numPolygonCor
                x1 = [curBodyOrigUncertPolygonBcsKey(1,i),curBodyOrigUncertPolygonBcsKey(3,i)];
                x2 = [curBodyOrigUncertPolygonBcsKey(1,mod(i,numPolygonCor)+1),curBodyOrigUncertPolygonBcsKey(3,mod(i,numPolygonCor)+1)];
                dx = x2 - x1;
                t(i) = - (lineVec(3) + lineVec(1)*x1(1) + lineVec(2)*x1(2)) ...
                    / (lineVec(1)*dx(1) + lineVec(2)*dx(2));
                x(i,:) = x1 + dx*t(i);
            end
            flag = t <= 1 & t >=0;
            if sum(flag) == 0
                warning('error for no interaction to estimate next position')
                X3 = curPoseBcsKey(1:2);
                X4 = curPoseBcsKey(1:2);
            elseif sum(flag) == 2
                temp3 = x(flag,:);
                X3 = temp3(1,:);
                X4 = temp3(2,:);
            elseif (sum(flag) == 1 || sum(flag) >= 3) && (sum(flag1) == 1 || sum(flag1) == 0)
                temp3 = curPoseBcsKey(1:2);
                X3 = temp3;
                X4 = temp3;
            end
            
            %             plot(X3(1),X3(2),'og')
            %             plot(X4(1),X4(2),'ob')
            
            dis1 = norm(X3-curPoseBcsKey(1:2));
            dis2 = norm(X4-curPoseBcsKey(1:2));
            if X3(1) <= X4(1)
                errRng(1) = -dis1;
                errRng(2) = dis2;
            else
                errRng(1) = -dis2;
                errRng(2) = dis1;
            end
            obj.robotPosePredictor.prvBodyOrigErrRng = errRng;
            %             obj.robotPosePredictor.prvBodyOrigErrRng = [0,0];
            
            pk = zeros(3,1);
            pk(1) = ratio*X1(1) + (1-ratio)*X2(1);
            pk(2) = 0;
            pk(3) = ratio*X1(2) + (1-ratio)*X2(2);
            
            %             c2kTrans = WcsPoseVecToPct(obj, curPoseBcsKey);
            ind = GetLastKeyFrameInd(obj);
            keyPoseWcs = obj.poseWcsList(ind,:);
            k2wTrans = WcsPoseVecToPct(obj, keyPoseWcs);
            %
            %             c2wTrans = k2wTrans*c2kTrans;
            %             curloc = T(c2wTrans);
            
            curloc = k2wTrans*pk;
            robotPoseWcs = [curloc(1),curloc(3),k2cRotAngBcs+keyPoseWcs(3)];
            
            
            %             global PATH_FIG
            %             figure(PATH_FIG)
            %             curBodyOrigUncertPolygonWcs = k2wTrans.transformMat*HomoCoord(curBodyOrigUncertPolygonBcsKey,1);
            %             hold on
            %             plot(curBodyOrigUncertPolygonWcs(1,[1:end 1]),curBodyOrigUncertPolygonWcs(3,[1:end 1]),'-xg');
            %             hold off
        end
        
        function UpdateTrajectoryPlot(obj)
            hold on;
            plot(-obj.poseWcsList(:,1), obj.poseWcsList(:,2),'-ob');
            
            %             PlotRobotPose(obj);
            hold off;
        end
        
        % Initializers
        function InitPointTracker(obj, pointTrackerParam)
            if strcmp(Configurable.OneofName(pointTrackerParam), 'pyrlk_feature_tracking_param')
                obj.featPtTracker = FeaturePointTrackerPyrLK(pointTrackerParam.pyrlk_feature_tracking_param);
            else
                error('Unsupported feature tracking method£º %s', Configurable.OneofName(pointTrackerParam));
            end
        end
        
        % Helper functions
        function ind = GetLastKeyFrameInd(obj)
            ind = find(obj.keyFrameFlagList, 1, 'last');
        end
        
        function inds = GetLastNKeyFrameInd(obj, n)
            assert(n<=sum(obj.keyFrameFlagList),'n should be greater than sum of keyframeList')
            inds = find(obj.keyFrameFlagList, n, 'last');
        end
        
        function ind = GetFirstKeyFrameInd(obj)
            ind = find(obj.keyFrameFlagList, 1, 'first');
        end
        %         function [t, x] = lineAndLineSegmentInteraction(lineNorm,x1,x2)
        %             dx = x2 - x1;
        %             t = - (lineNorm(3) + lineNorm(1)*x1(1) + lineNorm(2)*x1(2)) ...
        %                 / (lineNorm(1)*dx(1) + lineNorm(2)*dx(2));
        %             x = x1 + dx*t;
        %         end
        
        
        function CheckDataConsistency(obj)
            CheckDataConsistency(obj.featPtManager);
            assert(Rows(obj.poseWcsList) == length(obj.motionStateList) && Rows(obj.poseWcsList) == length(obj.keyFrameFlagList), 'Inconsistent internal data');
        end
    end
    
    methods (Static)
        function [reProjErr,dis,validFlag,err,errXY] = CalculateReprojectError(intrMat,Point3D,Point2D,R,T,flag,outLierThreshold)
            %             assert(Rows(Point3D)>=Cols(Point3D),'3d point should be input as n*3 Vec')
            %             assert(Rows(Point2D)>=Cols(Point2D),'2d point should be input as n*3 Vec')
            dis = -1*ones(Rows(Point2D),1);
            validFlag = false(Rows(Point2D),1);
            if isempty(R)||isempty(T)
                reProjxyz = Point3D';
            else
                reProjxyz = R*Point3D' + repmat(T,1,Rows(Point3D));
            end
            reprojPix = intrMat*reProjxyz;
            reprojPix = [reprojPix(1,:)./reprojPix(3,:);reprojPix(2,:)./reprojPix(3,:)];
            reprojPix = reprojPix';
            err = sqrt((reprojPix(flag,1)-Point2D(flag,1)).^2 +(reprojPix(flag,2)-Point2D(flag,2)).^2);
            %             dis(flag) = sqrt((reprojPix(flag,1)-Point2D(flag,1)).^2 +(reprojPix(flag,2)-Point2D(flag,2)).^2);
            %             dis(flag) = sqrt((reprojPix(flag,1)-Point2D(flag,1)).^2);
            dis(flag) = ((reprojPix(flag,1)-Point2D(flag,1)));
            errXY = [reprojPix(flag,1) - Point2D(flag,1),reprojPix(flag,2) - Point2D(flag,2)];
            validFlag(flag) = dis(flag) <= outLierThreshold;
            %             reProjErr = sum(dis(flag))/Rows(Point3D(flag));
            reProjErr = abs(mean(dis(flag) )); %sum(dis(flag))/Rows(Point3D(flag));
        end
    end
    methods (Static)
        function [reProjErr,dis,validFlag,err,errXY,meanXOfst] = CalculateReprojectError2(intrMat,Point3D,Point2D,R,T,flag,outLierThreshold)
            %             assert(Rows(Point3D)>=Cols(Point3D),'3d point should be input as n*3 Vec')
            %             assert(Rows(Point2D)>=Cols(Point2D),'2d point should be input as n*3 Vec')
            dis = -1*ones(Rows(Point2D),1);
            validFlag = false(Rows(Point2D),1);
            if isempty(R)||isempty(T)
                reProjxyz = Point3D';
            else
                reProjxyz = R*Point3D' + repmat(T,1,Rows(Point3D));
            end
            reprojPix = intrMat*reProjxyz;
            reprojPix = [reprojPix(1,:)./reprojPix(3,:);reprojPix(2,:)./reprojPix(3,:)];
            reprojPix = reprojPix';
            err = sqrt((reprojPix(flag,1)-Point2D(flag,1)).^2 +(reprojPix(flag,2)-Point2D(flag,2)).^2);
            dis(flag) = sqrt((reprojPix(flag,1)-Point2D(flag,1)).^2 +(reprojPix(flag,2)-Point2D(flag,2)).^2);
            %             dis(flag) = sqrt((reprojPix(flag,1)-Point2D(flag,1)).^2);
            %             dis(flag) = ((reprojPix(flag,1)-Point2D(flag,1)));
            errXY = [reprojPix(flag,1) - Point2D(flag,1),reprojPix(flag,2) - Point2D(flag,2)];
            validFlag(flag) = dis(flag) <= outLierThreshold;
            reProjErr = sum(dis(flag))/Rows(Point3D(flag));
            meanXOfst = mean(errXY(:,1));
            %             reProjErr = abs(mean(dis(flag) )); %sum(dis(flag))/Rows(Point3D(flag));
        end
    end
    
    methods
        function [pt2, flag,FundMat,FundMat00] = ReTraceEpiLine(obj,robotPoseWcs,pt1,flag)
            if sum(flag) == 0
                pt2 = pt1;
            else
                %                 intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,1);
                intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,obj.scaleLvl);
                id = find(obj.keyFrameFlagList == 1);
                id = id(end);
                
                b2cP = GetPctBody2Cam(obj.coordSysAligner, 1);
                c2wPBody = WcsPoseVecToPct(obj, robotPoseWcs);
                p2wPBody = WcsPoseVecToPct(obj, obj.poseWcsList(end ,:));
                k2wPBody = WcsPoseVecToPct(obj, obj.poseWcsList(id ,:));
                p2wPBody.transformMat(1:3,4) = 1000*p2wPBody.transformMat(1:3,4);
                c2wPBody.transformMat(1:3,4) = 1000*c2wPBody.transformMat(1:3,4);
                k2wPBody.transformMat(1:3,4) = 1000*k2wPBody.transformMat(1:3,4);
                p2cPBody = c2wPBody \ p2wPBody;
                k2cPBody = c2wPBody \ k2wPBody;
                p2cPCam = (b2cP*p2cPBody)*Inv(b2cP);
                k2cPCam = (b2cP*k2cPBody)*Inv(b2cP);
                p2c = p2cPCam.transformMat;
                k2c = k2cPCam.transformMat;
                transMat = SkewSymMat(k2c(1:3,4)./norm(k2c(1:3,4)));
                FundMat00 = inv(intrMat)'*transMat*k2c(1:3,1:3)*inv(intrMat);
                FundMat00 = FundMat00./norm(FundMat00);
                
                [pt2,flag,FundMat] = EpilineLK(obj.prevImgL,obj.currImgL,intrMat,p2cPCam.transformMat, pt1,flag);
            end
        end
        
    end
    
    methods
        
%         function [angOpt, inlierId, angRng, PixUse, pt2dCurUse, DispRefineUse, angOptFinalP2] = RefineZ(obj, k2cRef,k2cPnp, inlierId0, inlierId2, theta, DepthProbility,  DepthId,  AngleModalOrg,  AngleProbility,prevFeatPtList0,ptCcsZ0)  %, XYZ)
%           function [angOpt, inlierId, angRng, PixUse, pt2dCurUse, DispRefineUse, angOpt3, inFlag] = RefineZ(obj, k2cRef,k2cPnp, inlierId0, inlierId2, theta, DepthProbility,  DepthId,  AngleModalOrg,  AngleProbility,prevFeatPtList0,ptCcsZ0)  %, XYZ)
%           function [angOpt, inlierId, angRng, PixUse, pt2dCurUse, DispRefineUse_, angOpt3, inFlag,thetaRngOut,thetaProbOut,angOptP2C, angOptK2C_, p2cOnlyP2] = RefineZ(obj, k2cRef,k2cPnp, inlierId0, inlierId2, theta, DepthProbility,  DepthId,  AngleModalOrg,  AngleProbility,prevFeatPtList0,ptCcsZ0,DispRefineUse__,NewDispRng,newDispStep,ProbZTmpReSamp)  %, XYZ)
          function [angOpt, inlierId, angRng, PixUse, pt2dCurUse, DispRefineUse_, angOpt3, inFlag,thetaRngOut,thetaProbOut,angOptP2C, angOptK2C_, p2cOnlyP2_111] = RefineZ0(obj, k2cRef,k2cPnp, inlierId0, inlierId2, theta, DepthProbility,  DepthId,  AngleModalOrg,  AngleProbility,prevFeatPtList0,ptCcsZ0,DispRefineUse__,NewDispRng,newDispStep,ProbZTmpReSamp,k2cBody)  %, XYZ)
              
            global SHOWPOLYGON DRAWPOLYGON ANGLEONLY USEGOLDENDISP UPDATEDEPTH DEPTHITERNUM probPath ...
                USELEFT USELEFTPLUSRIGHT USELEFTXRIGHT USERIGHT WITHINEPILINE BOTHLR HARDCUT QUANTCUT FUSIONCUT ...
                FORCEEVENPLATFORM CUTTHETAANDFORCEPLATFORM DIFFTHETATHR CUTTHETAANDFORCEPLATFORM_NODIFF FORCEEXPEVEN ...
                EXPZERO GTTRACKING SOFTP2 Rati Rati1 Rati2 Sc USEthetaProb0 ONLYP2 ONLYP2_2 UPDATEZ_2 UPDATETracking_2 ...
                Sc1 Sc2 FORCETHETAPLATFORM1 FORCETHETAPLATFORM2 ONLYP2_1_Max NewOnlyP2 UPDATETracking_2_2 UpdateP2_3 ...
                doRoundingTheta doRoundingRefAngList3 UsePrvDepth InheriDispRng EvenDepthProb roundingPrecision IterThetaNum ...
                NewPrvDispResamp Rati_onlyP2 NewFlow UseNewP2Only doRoundingTheta2 roundingPrecision2 UseNewP2CRef ...
                ReplaceK2CRefInKey UseGoldenThetaP2C FrmNumWithGloden ForceThetaExpZero ResampZThr1 ResampZThr2 ...
                ProbZCutOffThr
            
            
            if size(obj.featPtManager.localTrace.ptIcsX, 2) <= FrmNumWithGloden + 1
                doRoundingTheta = false; true; false;
                UseGoldenThetaP2C = true;
                Rati1 = 0.8; 0; 0.8; 0; 0.5; 0.8; 0; 0.5; 0.8; 0.8; 0.8; 0; 0.4; 0.3; 0.5; 0; 0.8; 0.0001; 0.95;
                Rati2 = 0.8;
                UsePrvDepth = false; false; true; false; true; false; true;
                NewPrvDispResamp = false;
                
                EvenDepthProb = false; false;true; false; true;
                InheriDispRng = true;
                
                
                obj.configParam.disparity_resample_prob_threshold = ResampZThr1;
                
%                 UpdateP2_3 = false;
                
                if 0
                    ForceThetaExpZero = true;
                else
                    ForceThetaExpZero = false;
                end
                
            else
                if 1
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
                obj.configParam.disparity_resample_prob_threshold = ResampZThr2;
                
%                 UpdateP2_3 = true;
                
                ForceThetaExpZero = false;
                
                
                
            end
            
            
            k2cRef0 = k2cRef;
            if doRoundingTheta
                %                 k2cRef = deg2rad(round(rad2deg(k2cRef),1));
                k2cRef = deg2rad(round(rad2deg(k2cRef0 + obj.thetaNoise)./roundingPrecision).*roundingPrecision);
                
                
                %                 k2cRef = k2cRef0 + deg2rad(0.1*(-1)^(size(obj.featPtManager.localTrace.ptIcsX, 2))) + deg2rad(0.01*(rand(1)-0.5));
                
                if 0
                    if ismember(size(obj.featPtManager.localTrace.ptIcsX, 2), [2 5 8 11 14 17] - 0)
                        k2cRef = k2cRef0 +  0 + deg2rad(0.01*(rand(1)-0.5));
                    elseif ismember(size(obj.featPtManager.localTrace.ptIcsX, 2), [3 6 9 12 15 18])
                        k2cRef = k2cRef0  -  deg2rad(0.15*(1)^(size(obj.featPtManager.localTrace.ptIcsX, 2))) + deg2rad(0.01*(rand(1)-0.5));
                    elseif ismember(size(obj.featPtManager.localTrace.ptIcsX, 2), [4 7 10 13 16 19])
                        k2cRef = k2cRef0  +  deg2rad(0.15*(1)^(size(obj.featPtManager.localTrace.ptIcsX, 2))) + deg2rad(0.01*(rand(1)-0.5));
                    elseif 0 % ismember(size(obj.featPtManager.localTrace.ptIcsX, 2), [5 9 13 17 21])
                        k2cRef = k2cRef0  +  deg2rad(0.1*(1)^(size(obj.featPtManager.localTrace.ptIcsX, 2))) + deg2rad(0.01*(rand(1)-0.5));
                    end
                    
                    
                elseif 0
                    
%                     k2cRef = k2cRef0  -  deg2rad(0.15*(-1)^(size(obj.featPtManager.localTrace.ptIcsX, 2))) + deg2rad(0.01*(rand(1)-0.5));
%                     k2cRef = k2cRef0  -  deg2rad(0.15*(-1)^(size(obj.keyFrameFlagList, 1))) + deg2rad(0.01*(rand(1)-0.5));
                    k2cRef = k2cRef0  -  deg2rad(0.15*(-1)^(sum(obj.keyFrameFlagList) + size(obj.featPtManager.localTrace.ptIcsX, 2))) + deg2rad(0.01*(rand(1)-0.5));
                    
                end
                
                
                
                
            end
            
            
            if ReplaceK2CRefInKey
                if UseGoldenThetaP2C
                    if  size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                        k2cRef = k2cRef0;
                    end
                    
                    
                end
            end
            
            
            
            
            k2cRefBak = k2cRef;
            
%             [k2cRef, ~, ~] = AngleSpace.EpiPolarThreeView(obj, k2cRef, k2cRef, [deg2rad(-0.2) : obj.configParam.theta_sample_step : deg2rad(0.2)], Pix, Pix, pt2dCur, intrMat, b2c);

            
            
            
            
            
            
            if size(obj.featPtManager.localTrace.ptIcsX, 2) > 2
                obj.configParam.reproj_sigma = obj.configParam.reproj_sigma0 * obj.configParam.reproj_sigma_scale;
                obj.configParam.reproj_sigma_right = obj.configParam.reproj_sigma_right0 * obj.configParam.reproj_sigma_scale_right;
            else
                obj.configParam.reproj_sigma = obj.configParam.reproj_sigma0; %  * obj.configParam.reproj_sigma_scale;
                obj.configParam.reproj_sigma_right = obj.configParam.reproj_sigma_right0; % * obj.configParam.reproj_sigma_scale_right;
            end
            
             
            
            
            [princpPtL, princpPtR] = Get(obj.camModel, 'PinholePrincpPt', obj.scaleLvl);
            L2R = [rodrigues(obj.camModel.rotVec1To2) obj.camModel.transVec1To2; 0 0 0 1];
            
            intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,obj.scaleLvl);
            depthMapCurVisual = obj.depthVisual;
            depthMapCurGT = obj.depthGT;
            
            if USEGOLDENDISP
                depthMapCur = depthMapCurGT;
            else
                depthMapCur = depthMapCurVisual;
            end
            
            dispMapCur = (intrMat(1,1).*norm(obj.camModel.transVec1To2)./depthMapCur) - (princpPtR(1) - princpPtL(1));
            
            dispMapCurGT = (intrMat(1,1).*norm(obj.camModel.transVec1To2)./depthMapCurGT) - (princpPtR(1) - princpPtL(1));
            
            dispMapCur(dispMapCur < 0) = nan;
            depthMapCur(depthMapCur < 0) = nan;
            
            imgCur = obj.currImgL;
            imgCurR = obj.currImgR;
            
            
            imgKeyL = obj.keyFrameImgL; imgKeyR = obj.keyFrameImgR;
            imgPrvL = obj.prevImgL; imgPrvR = obj.prevImgR;
            
            
            
            [depthMapKeyVisual, dispMapKey] = GetDepthMap(obj, obj.keyFrameImgL, obj.keyFrameImgR);
            
            depthMapKey = obj.keyFrameDepth;
            try
                inlierIdPrv = obj.keyProbZ{end,2};
            catch
                asg = 1;
            end
            if size(obj.featPtManager.localTrace.ptIcsX, 2) > 2
                
                inlierIdPrv = obj.keyProbZ{end,2};
                
                pixKey = [obj.featPtManager.localTrace.ptIcsX(inlierIdPrv,1) obj.featPtManager.localTrace.ptIcsY(inlierIdPrv,1)];
                
                inlierIdPrvInd = sub2ind(size(dispMapKey), round(pixKey(:,2)), round(pixKey(:,1)));
                dispMapKey(inlierIdPrvInd) = NewDispRng(:,(size(NewDispRng,2)+1)/2);
                depthMapKeyVisual(inlierIdPrvInd) = (intrMat(1,1).*norm(obj.camModel.transVec1To2))./(dispMapKey(inlierIdPrvInd) + (princpPtR(1) - princpPtL(1)));
                depthMapKey(inlierIdPrvInd) = (intrMat(1,1).*norm(obj.camModel.transVec1To2))./(dispMapKey(inlierIdPrvInd) + (princpPtR(1) - princpPtL(1)));
                obj.keyFrameDepth(inlierIdPrvInd) = (intrMat(1,1).*norm(obj.camModel.transVec1To2))./(dispMapKey(inlierIdPrvInd) + (princpPtR(1) - princpPtL(1)));
%                 obj.keyFrameDepthGT(inlierIdPrvInd) = (intrMat(1,1).*norm(obj.camModel.transVec1To2))./(dispMapKey(inlierIdPrvInd) + (princpPtR(1) - princpPtL(1)));
                obj.featPtManager.localTrace.ptCcsZ(inlierIdPrv) = (intrMat(1,1).*norm(obj.camModel.transVec1To2))./(dispMapKey(inlierIdPrvInd) + (princpPtR(1) - princpPtL(1)));
                sdgaf = 1;
            end
                
                
            if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                obj.thetaRngMat = [];
                obj.thetaProbMat = [];
            end
            
            if 0
                thetaSamp = [obj.configParam.theta_range(1) : obj.configParam.theta_sample_step : obj.configParam.theta_range(2)];
                thetaSamp0 = thetaSamp;
                thetaRange = k2cRef + [obj.configParam.theta_range(1) : obj.configParam.theta_sample_step : obj.configParam.theta_range(2)];
                thetaRng = thetaRange;
            end
            %             [probTheta, ~] = probDensity(k2cRef, obj.configParam.theta_sigma, thetaRng,obj.configParam.theta_sample_step, 'theta');
            
            disparityRng = [-obj.configParam.disparity_error : obj.configParam.disparity_sample_step : obj.configParam.disparity_error];
            
            if 0
                [thetaGrid, disparityGrid] = meshgrid(thetaRange, disparityRng);
                ThetaDisp = [thetaGrid(:) disparityGrid(:)];
            end
            
            
            
%             depthMapKey = obj.keyFrameDepth;
            
            
            
            
            depthMapKey(dispMapKey < 0) = -1;
            dispMapKey(dispMapKey < 0) = nan;
            
            
            
            depthErrMap = zeros(size(dispMapKey));
            
            validInd = sub2ind(size(dispMapKey), round(obj.featPtManager.localTrace.ptIcsY(:,1)), round(obj.featPtManager.localTrace.ptIcsX(:,1)));
            zList0 = depthMapKey(validInd);
            zList00 = obj.keyFrameDepth(validInd);
            dispList = dispMapKey(validInd);
            
            
            dispMapCurGTList0 = dispMapCurGT(validInd);
            
            
            
            depListVisual = depthMapKeyVisual(validInd);
            depListGT = depthMapKey(validInd);
            depthMapKeyGT = obj.keyFrameDepthGT;
            depListGT = depthMapKeyGT(validInd);
            dispMatGT = (intrMat(1,1).*norm(obj.camModel.transVec1To2)./depthMapKeyGT) - (princpPtR(1) - princpPtL(1));
            dispMatGT0 = dispMatGT;
            
            dispListGT = dispMatGT0(validInd);
            depthListGT = depthMapKeyGT(validInd);
            %             if USEGOLDENDISP
            %                 dispList = dispListGT;
            %             end
            dispCurList_ = dispMapCur(validInd);
            
            if size(obj.featPtManager.localTrace.ptIcsX, 2) > 2
                
                validIndPrv = zeros(length(depListVisual),1);
                validPrv = find(obj.featPtManager.localTrace.ptIcsX(:,end-1) > 0 & obj.featPtManager.localTrace.ptIcsY(:,end-1) > 0);
                validIndPrvValid = sub2ind(size(dispMapKey), round(obj.featPtManager.localTrace.ptIcsY(validPrv,end-1)), round(obj.featPtManager.localTrace.ptIcsX(validPrv,end-1)));
                
                if obj.switchDepth
                    depthMapPrv = obj.prvDepthGT;
                else
                    depthMapPrv = obj.prvDepthVisual;
                end
                vldPrvId = find(depthMapPrv(validIndPrvValid) ~= -1);
                validPrv = validPrv(vldPrvId);
%                 validIndPrvValid = validIndPrvValid(intersect(validPrv, find(depthMapPrv(validIndPrvValid) ~= -1)));
                dispMapPrv = (intrMat(1,1).*norm(obj.camModel.transVec1To2)./depthMapPrv) - (princpPtR(1) - princpPtL(1));
%                 dispPrvList_ = dispMapPrv(validIndPrvValid);
                dispPrvList_ = nan(length(dispCurList_),1); depthPrvList_ = nan(length(dispPrvList_),1);
                dispPrvList_(validPrv) = dispMapPrv(validIndPrvValid(vldPrvId));
                depthPrvList_(validPrv) = intrMat(1,1).*norm(obj.camModel.transVec1To2)./( dispPrvList_(validPrv) + (princpPtR(1) - princpPtL(1)));
            else
                dispPrvList_ = dispCurList_;
            end
            
            
            dispMatGT(dispMatGT > max(dispMapKey(validInd)) + 3) = nan;
%             dispMatGT(dispMatGT < 0.9) = 0.9;
            dispMatGT(dispMatGT < 0.0001) = 0.0001;
            if USEGOLDENDISP
                dispList = dispMatGT(validInd);
            else
                %                 if 1 %size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                % %                     figure(38),clf;plot(dispList - dispMatGT(validInd));title('stereoDisp - gtDisp');
                %                    dispGTTmp = dispMatGT(validInd);
                %                    figure(38),clf;plot(dispList(inlierId0) - dispGTTmp(inlierId0));title('stereoDisp - gtDisp');
                %                 end
                if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                    %                     figure(38),clf;plot(dispList - dispMatGT(validInd));title('stereoDisp - gtDisp');
                    probDir = probPath;  % fullfile(pwd, 'prob'); 
                    
                    dispGTTmp = dispMatGT(validInd);
                    figure(38),clf;subplot(1,3,1);hold on;plot(dispList(inlierId0) - dispGTTmp(inlierId0));title('stereoDisp - gtDisp');subplot(1,3,2);imshow(imgCur);hold on;
                    %                     saveas(gcf,fullfile(probDir,sprintf('disp_%05d.png',length(dir(fullfile(probDir,'disp_*.png')))+2)));
                    %                         saveas(gcf,fullfile(probDir,sprintf('hist_%05d.fig',length(dir(fullfile(probDir,'hist_*.fig')))+1)));
                    
                end
            end
            
            dispErr = dispMapKey - dispMatGT;
            %                     dispErr(isnan(dispErr)) = 0;
            dispErrList = dispErr(validInd);
            
            %             if obj.switchDepth
            zList = obj.featPtManager.localTrace.ptCcsZ;
            
            
            baseline = norm(obj.camModel.transVec1To2);
            b2c = obj.coordSysAligner.pctBody2Cam(1, 1).transformMat;
            
            T_B2C = b2c;
            
            
            r_cam = sqrt(T_B2C(1,4)^2+T_B2C(2,4)^2+T_B2C(3,4)^2);
            tx = T_B2C(1,4)/r_cam;
            ty = T_B2C(2,4)/r_cam;
            tz = T_B2C(3,4)/r_cam;
            
            
            
            
            
            reprojError = [];cntt = 0;PixCur = [];PixReproj = [];
            validId = []; validXYZAll = []; Pix0 = []; vldReproj = [];
            keyLength = size(obj.featPtManager.localTrace.ptIcsX,2);
            
            dispGTTmp = dispMatGT(validInd);
            if 0 % 20191216
                idUnValid = find(isnan(zList) | isinf(zList) | isnan(dispList) | isinf(dispList) | isnan(dispCurList_) | isnan(dispGTTmp)) ; % | isnan(dispGTTmp) | isinf(dispGTTmp));
            else
                idUnValid = find(isnan(zList) | isinf(zList) | isnan(dispList) | isinf(dispList) | isnan(dispCurList_) | isnan(dispGTTmp) | isnan(dispPrvList_) ); % | isnan(dispGTTmp) | isinf(dispGTTmp));
            end
            inlierId = setdiff(inlierId0, idUnValid);
            
            if ~isempty(obj.featPtManager.localTrace.probZ)
                as = find(max(obj.featPtManager.localTrace.probZ' > 0.000001)');
                inlierId = intersect(inlierId,as);
            end
            
            if 1 % 20191216
                if size(obj.featPtManager.localTrace.ptIcsX, 2) > 2
                    inlierId = intersect(inlierId, inlierIdPrv);
                end
            end
            if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                curDispStepList = repmat(obj.configParam.disparity_sample_step,length(inlierId),1);
            end
            
            
            
            
            
            
            
            dispMapCurGTList = dispMapCurGTList0(inlierId,:);
            %             pixGT = pixGT0(ismember(inlierId0, inlierId),1:2);
            
           
            
            
            reprojError = [];cntt = 0;PixCur = [];PixReproj = [];
            validId = []; validXYZAll = []; Pix0 = []; vldReproj = [];
            keyLength = size(obj.featPtManager.localTrace.ptIcsX,2);
            
            
   
            
            
            
            
            if DRAWPOLYGON
                if ~isempty(inlierId)
                    %                     if SHOWPOLYGON
                    %                         figure(110),clf;imshow(imgCurL);hold on;
                    %                     end
                    
                    
                    
                    %                     pixGTR = [pixGT(:,1) - dispMapCurGTList pixGT(:,2)];
                    
                    
                    
                    %                                     figure(232);hold on;axis equal;
                    if 1
                        if isempty(obj.featPtManager.localTrace.probZ)
                            DispRng = repmat(dispList(inlierId,:),1,length(disparityRng)) + repmat(disparityRng,length(inlierId),1);
                            DispRng(DispRng < 0) = 0.0001;
                           disparityError = dispList(inlierId) - dispGTTmp(inlierId);

                           disparityErrorRound = round(disparityError./obj.configParam.disparity_sample_step).*obj.configParam.disparity_sample_step;

                            
                            depthGTOfst11 = round(-(disparityErrorRound)./obj.configParam.disparity_sample_step);
                            depthGTInd11 = depthGTOfst11 + (size(DispRng,2)+1)/2;
                            depthGTInd11(depthGTInd11 < 1) = 1;
                            depthGTInd11(depthGTInd11 > size(DispRng,2)) = size(DispRng,2);
                            
%                             DepthGTInd(jkl,:) = depthGTInd11';
                            
                            depthGTIndAll11 = sub2ind(size(DispRng),[1:size(DispRng,1)]', depthGTInd11);
                            
                            checkDispRounding = DispRng(depthGTIndAll11) - dispGTTmp(inlierId);
                            
                            for jk = 1 : length(inlierId)
                                [ProbZ(jk,:), ZZVec1(jk,:)] = probDensity(dispList(inlierId(jk),:), obj.configParam.disparity_sigma,obj.configParam.disparity_beta,obj.configParam.reproj_beta, DispRng(jk,:),obj.configParam.disparity_sample_interval, 'disparity');
                            end
                            ProbZ(isnan(ProbZ)) = 0;
                            obj.featPtManager.localTrace.probZ = nan(length(obj.featPtManager.localTrace.ptCcsZ), size(ProbZ,2));
                            obj.featPtManager.localTrace.sampleZ = nan(length(obj.featPtManager.localTrace.ptCcsZ), size(ZZVec1,2));
                            obj.featPtManager.localTrace.probZ(inlierId,:) = ProbZ;
                            obj.featPtManager.localTrace.sampleZ(inlierId,:) = ZZVec1;
                            outId = [];
                        else
                            if 0
                                DispRng = repmat(dispList(inlierId,:),1,length(disparityRng)) + repmat(disparityRng,length(inlierId),1);
                            else
                                DispRng1 = repmat(dispList(inlierId,:),1,length(disparityRng)) + repmat(disparityRng,length(inlierId),1);
                                DispRng = NewDispRng(ismember(inlierIdPrv, inlierId),:);
                                DispRng0 = DispRng;
                                if 0 % to check
                                    figure,plot(DispRng1(:,11) - DispRng(:,11));
                                end
                                    
                            end
                            
                            try
                                [DispRng,ProbZTmpReSamp2,DispKeyInPrvRng] = IntersectCone(obj, inlierId, inlierIdPrv, obj.keyFrameImgL, obj.prevImgL,b2c, DispRng,dispPrvList_, DepthProbility,DepthId);
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
                                ProbZ = obj.featPtManager.localTrace.probZ(inlierId,:);
                            else
                                %                                 DispRng=DispRng;
                                if 0 %size(obj.featPtManager.localTrace.ptIcsX, 2) > FrmNumWithGloden + 1
                                    ProbZ = ProbZTmpReSamp2;
                                else
                                    ProbZ = obj.featPtManager.localTrace.probZ(inlierId,:);
                                end
                                  
                                
                                dispList(inlierId) = DispRng(:,(size(DispRng,2)+1)/2);
                                obj.featPtManager.localTrace.ptCcsZ(inlierId) = (intrMat(1,1).*norm(obj.camModel.transVec1To2))./(dispList(inlierId) + (princpPtR(1) - princpPtL(1)));
                                zList(inlierId) = obj.featPtManager.localTrace.ptCcsZ(inlierId);
                                dispPrvList_(inlierId) = DispKeyInPrvRng(:,(size(DispRng,2)+1)/2);
                                depthPrvList_(inlierId) = (intrMat(1,1).*norm(obj.camModel.transVec1To2))./(dispPrvList_(inlierId) + (princpPtR(1) - princpPtL(1)));
                            end
                            ProbZ_out = obj.keyProbZ{end,21}(:,2);
                            idLast = obj.keyProbZ{end,2};
                            outId = find(~ismember(idLast, inlierId));
%                             ZZVec1 = obj.featPtManager.localTrace.sampleZ(inlierId,:);
                            
                            
                            for jk = 1 : length(inlierId)
                                [~, ZZVec1(jk,:)] = probDensity(dispList(inlierId(jk),:), obj.configParam.disparity_sigma,obj.configParam.disparity_beta,obj.configParam.reproj_beta, DispRng(jk,:),obj.configParam.disparity_sample_interval, 'disparity');
                            end
                            
                            
                            
                            
                            asdk = 1;
                        end
                        
                        
                        
                        if 1 % ~USEGOLDENDISP %size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                            %                     figure(38),clf;plot(dispList - dispMatGT(validInd));title('stereoDisp - gtDisp');
                            %                 dispGTTmp = dispMatGT(validInd);
                            figure(38),clf;subplot(1,3,1);hold on;plot(dispList(inlierId) - dispGTTmp(inlierId));title('stereoDisp - gtDisp');subplot(1,3,2);imshow(imgCur);hold on; %subplot(1,3,3);plot(dispErrCur);title('stereoDispCur - gtDispCur');
                            disparityError = dispList(inlierId) - dispGTTmp(inlierId);
                            
                            if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                disparityErrorRound = round(disparityError./obj.configParam.disparity_sample_step).*obj.configParam.disparity_sample_step;
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
                                
                                b2cPmat = GetPctBody2Cam(obj.coordSysAligner, 1);
                                if 0
                                    k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(thetaRng(idM)),zeros(3,1));
                                else
                                    k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cRef0),zeros(3,1));
                                end
                                k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
                                
                                Pix = [obj.featPtManager.localTrace.ptIcsX(inlierId,1) obj.featPtManager.localTrace.ptIcsY(inlierId,1)];
                                
                                metricPrevPtCcsGT = intrMat\HomoCoord(Pix',1);
                                metricPrevPtCcsGT = normc(metricPrevPtCcsGT);
                                if 0
                                    scaleAllGT = depthListGT(inlierId)./metricPrevPtCcsGT(3,:)';
                                else
                                    dispGTComp = dispList(inlierId) - disparityErrorRound;
                                    depthListGTComp = intrMat(1,1).*norm(obj.camModel.transVec1To2)./(dispGTComp + (princpPtR(1) - princpPtL(1)));
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
                                
                                if size(obj.featPtManager.localTrace.ptIcsX, 2) > 2
                                    [pixGTPrv, pixGTRPrv] = VisualLocalizer.GetGtTrace(obj, obj.refAngList(end),inlierId,dispList,disparityErrorRound,intrMat,princpPtL,princpPtR,dispMapCurGTList);
                                    
                                    
                                end
                                
                            end
                        end
                        
                        
                        if 0  
                            maxFeatZ = max(ProbZ')';
                            maxFeatZ = repmat(maxFeatZ, 1, size(ProbZ,2));
                            ProbZTmp_norm = ProbZ./maxFeatZ;
                            
                            % %                         ProbZ = ProbZTmp_norm;
                            
                            depthGTOfst = round(-(disparityErrorRound)./obj.configParam.disparity_sample_step);
                            depthGTInd = depthGTOfst + (size(ProbZ,2)+1)/2;
                            depthGTInd(depthGTInd < 1) = 1;
                            depthGTInd(depthGTInd > size(ProbZ,2)) = size(ProbZ,2);
                            depthGTIndAll = sub2ind(size(ProbZ),[1:size(ProbZ,1)]', depthGTInd);
                            figure(43),
                            %                         if jkl == 1
                            if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                probDir = probPath;  %fullfile(pwd, 'prob');
                                saveas(gcf,fullfile(probDir,sprintf('hist_%05d.png',length(dir(fullfile(probDir,'hist_*.png')))+1)));
                                saveas(gcf,fullfile(probDir,sprintf('hist_%05d.fig',length(dir(fullfile(probDir,'hist_*.fig')))+1)));
                                
                            end
                            %                         end
                            ProbZTmpTmp_norm = ProbZTmp_norm(depthGTIndAll);
                            figure(43),clf;hist(ProbZTmpTmp_norm(~isnan(ProbZTmpTmp_norm)),50); title(num2str(sum(~isnan(ProbZTmpTmp_norm))));
                        end
                        
                    Pix = [obj.featPtManager.localTrace.ptIcsX(inlierId,1) obj.featPtManager.localTrace.ptIcsY(inlierId,1)];
                    
                    ZTrue = zList(inlierId);
                    
                    
                    metricPrevPtCcs = intrMat\HomoCoord(Pix',1);
                    metricPrevPtCcs = normc(metricPrevPtCcs);
                    scaleAll = ZTrue./metricPrevPtCcs(3,:)';
                    %                                   scale0 = zTrue./metricPrevPtCcs(3,:)';
                    keyCcsXYZAll = [repmat(scaleAll',3,1).*metricPrevPtCcs];
                    
                    
                                            
                    thetaRange123 = k2cRef + [obj.configParam.theta_range(1) : obj.configParam.theta_sample_step : obj.configParam.theta_range(2)];
                    
                    
                    Prob = zeros(length(thetaRange123),length(inlierId),length(disparityRng));
                    
                    if ~GTTRACKING
                        pt2dCur = [obj.featPtManager.localTrace.ptIcsX(inlierId,keyLength) obj.featPtManager.localTrace.ptIcsY(inlierId,keyLength)];
                        try
                            pt2dPrv = [obj.featPtManager.localTrace.ptIcsX(inlierId,end-1) obj.featPtManager.localTrace.ptIcsY(inlierId,end-1)];
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
                            pt2dCur = [obj.featPtManager.localTrace.ptIcsX(inlierId,keyLength) obj.featPtManager.localTrace.ptIcsY(inlierId,keyLength)];
                            pt2dCur(:,2) =  pixGT(:,2);
                        end
                    end
                    pt2dCur0 = pt2dCur;
                    pt2dCurHomo = (inv(intrMat)*pextend(pt2dCur'))';
                    
                    dispCurList = dispCurList_(inlierId);
                    
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
                            pt2dCur_0 = [obj.featPtManager.localTrace.ptIcsX(inlierId,keyLength) obj.featPtManager.localTrace.ptIcsY(inlierId,keyLength)];
                            pt2dCurR = [pt2dCur_0(:,1) - dispCurList pt2dCur_0(:,2)];
                            pt2dCurR(:,2) = pixGTR(:,2);
                        end
                    end
                    
                    pt2dCurRHomo = (inv(intrMat)*pextend(pt2dCurR'))';
                    PixHomo = (inv(intrMat)*pextend(Pix'))';
                    
                    validIndCur = sub2ind(size(dispMapKey), round(pt2dCur(:,2)), round(pt2dCur(:,1)));
                    dispErrCur = dispCurList  - dispMapCurGTList;
                        
% %                         try
% %                             [sampledDispRngPrvInKey,ProbZTmpReSamp,DispKeyInPrvRng] = IntersectCone(obj, inlierId, inlierIdPrv, obj.keyFrameImgL, obj.prevImgL,b2c, DispRng,dispPrvList_, DepthProbility,DepthId);
% %                         catch
% %                             svkbj = 1;
% %                         end
                        
                        for jkl = 1 : DEPTHITERNUM
                            
                            
                            
                            [maxFeatZ1,idMax] = max(ProbZ');
                            maxFeatZ = maxFeatZ1';
                            maxFeatZ = repmat(maxFeatZ, 1, size(ProbZ,2));
                            ProbZTmp_norm = ProbZ./maxFeatZ;
                            
                            % %                         ProbZ = ProbZTmp_norm;
                            if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                depthGTOfst = round(-(disparityErrorRound)./obj.configParam.disparity_sample_step);
                            else
                                depthGTOfst1 = round(-(disparityErrorRound)./obj.configParam.disparity_sample_step);
                                depthGTOfst = round(-(disparityErrorRound)./newDispStep(ismember(inlierIdPrv,inlierId)));


                            end
                            depthGTInd = depthGTOfst + (size(ProbZ,2)+1)/2;
                            depthGTInd(depthGTInd < 1) = 1;
                            depthGTInd(depthGTInd > size(ProbZ,2)) = size(ProbZ,2);
                            
                            DepthGTInd(jkl,:) = depthGTInd';
                            
                            depthGTIndAll = sub2ind(size(ProbZ),[1:size(ProbZ,1)]', depthGTInd);
                            figure(43),
                            if jkl == 1 %DEPTHITERNUM
                                if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                    probDir = probPath;  % fullfile(pwd, 'prob');
                                    %                                     saveas(gcf,fullfile(probDir,sprintf('hist_%05d.png',length(dir(fullfile(probDir,'hist_*.png')))+1)));
                                    %                                     saveas(gcf,fullfile(probDir,sprintf('hist_%05d.fig',length(dir(fullfile(probDir,'hist_*.fig')))+1)));
                                    
                                end
                            end
                            ProbZTmpTmp_norm = ProbZTmp_norm(depthGTIndAll);
                            ProbZTmpTmp_norm0 = ProbZTmpTmp_norm;
%                             [depthGTInd, ProbZTmpTmp_norm, ProbZTmpNorm, idMax, depthGTIndAll, depthUpdateIndAll] = VisualLocalizer.GetDepthHist(obj, ProbZ,disparityErrorRound,obj.configParam.disparity_sample_step);
                            
                            
                            %                             figure(43),clf;hist(ProbZTmpTmp_norm(~isnan(ProbZTmpTmp_norm)),50); title(num2str(sum(~isnan(ProbZTmpTmp_norm))));
                            figure(43),clf;subplot(2,2,1);hist(ProbZTmpTmp_norm(~isnan(ProbZTmpTmp_norm)),50); title(sprintf('%d\nsigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f',sum(~isnan(ProbZTmpTmp_norm)),obj.configParam.reproj_sigma,obj.configParam.reproj_beta,obj.configParam.reproj_sigma_right,obj.configParam.reproj_beta_right));
                            subplot(2,2,3);imshow(obj.currImgL); hold on;plot(pt2dCur(ProbZTmpTmp_norm>obj.configParam.depth_hist_ratio,1), pt2dCur(ProbZTmpTmp_norm>obj.configParam.depth_hist_ratio,2), '.r');plot(pt2dCur(ProbZTmpTmp_norm<=obj.configParam.depth_hist_ratio,1), pt2dCur(ProbZTmpTmp_norm<=obj.configParam.depth_hist_ratio,2), '.g');legend('good','bad','Location','southwest');title(sprintf('cur probZ > %0.3f',obj.configParam.depth_hist_ratio));
                            if ~isempty(outId)
                                subplot(2,2,2);hist(ProbZ_out(outId),50); title(sprintf('%d',length(outId)));
                                subplot(2,2,4);imshow(obj.prevImgL); hold on;plot(obj.keyProbZ{end,6}(outId,1), obj.keyProbZ{end,6}(outId,2), 'xy');plot(obj.keyProbZ{end,6}(ProbZ_out>obj.configParam.depth_hist_ratio,1), obj.keyProbZ{end,6}(ProbZ_out>obj.configParam.depth_hist_ratio,2), '.r');plot(obj.keyProbZ{end,6}(ProbZ_out<=obj.configParam.depth_hist_ratio,1), obj.keyProbZ{end,6}(ProbZ_out<=obj.configParam.depth_hist_ratio,2), '.g');legend('lost','good','bad','Location','southwest'); title(sprintf('prv probZ > %0.3f',obj.configParam.depth_hist_ratio));
                            end
                            probDir = probPath;
                            %                             saveas(gcf,fullfile(probDir,sprintf(strcat('hist_',probDir(end-14:end),num2str(sum(obj.keyFrameFlagList)),'_%05d.png'),length(dir(fullfile(probDir,'hist_*.png')))+1)));
                            saveas(gcf,fullfile(probDir,sprintf(strcat('hist_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probDir,strcat('hist_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                            %                             saveas(gcf,fullfile(probDir,sprintf(strcat('hist_',probDir(end-14:end),'_%05d.png'),length(dir(fullfile(probDir,'hist_*.png')))+1)));
                            
                            
                            if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                
                                featPtList = VisualLocalizer.DetectFast(imgKeyL);
                                depthMapKey__ = obj.keyFrameDepth;
                                inValid__ = sub2ind(size(depthMapKey__), round(featPtList(:,2)), round(featPtList(:,1)));
                                depthList__ = depthMapKey__(inValid__);
%                                 inId_1 = find(depthList__ > 0);
                                featPtList = featPtList(depthList__ > 0, :);
                                depthList__ = depthList__(depthList__ > 0);
                                
                                
                                
                                
                                inTrackFlag = true(size(featPtList,1),1);
                                
                                pnp_ang_est_max_margin = [deg2rad([-1 1]) obj.configParam.pure_rot_reproject_outlier_threshold];
                                
                                
                                
                               [predPtList, inTrackFlag00] = TrackFeaturePoints2( imgKeyL, imgCur, featPtList,inTrackFlag);    
    
                               
                               [p2cBodyRotAng,Err,inId] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin,intrMat,featPtList(inTrackFlag00,:),predPtList(inTrackFlag00,:),depthList__(inTrackFlag00,:), find(inTrackFlag00),true(sum(inTrackFlag00),1),b2c,k2cRef0);
                               
                               
                                if 0
                                   figure,subplot(1,2,1),showMatchedFeatures(imgKeyL, imgCur, featPtList(inTrackFlag00,:),predPtList(inTrackFlag00,:));subplot(1,2,2);showMatchedFeatures(imgKeyL, imgCur, featPtList(inId,:),predPtList(inId,:))
                               end
                               
                               
                               thetaRange = k2cRef + [obj.configParam.theta_range(1) : obj.configParam.theta_sample_step : obj.configParam.theta_range(2)];
                              
                               if 0
                                   [~, angOptK2C_11111, ~, ~] = VisualLocalizer.New_OnlyP2_new(obj, featPtList(inId,:),predPtList(inId,:), intrMat,[],T_B2C, k2cRef, tx, ty, tz, thetaRng);
                               elseif 0
                                   [~, angOptK2C_11111, ~, ~] = VisualLocalizer.New_OnlyP2_new(obj, Pix, pt2dCur, intrMat,[],T_B2C, k2cRef, tx, ty, tz, thetaRng);
                               else
% %                                    angOptP2C__1 = k2cRef - obj.refAngList(end);
% %                                     
% %                                     
% %                                     pixprv = pt2dPrv;
                                         
                                    [angOptK2C_11111, p2cOpt, k2pOpt] = AngleSpace.EpiPolarThreeView(obj, k2cRef0, k2cRef0, [deg2rad(-0.2) : obj.configParam.theta_sample_step : deg2rad(0.2)], Pix, Pix, pt2dCur, intrMat, b2c);
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
                               thetaSamp = [obj.configParam.theta_range(1) : obj.configParam.theta_sample_step : obj.configParam.theta_range(2)];
                               thetaSamp0 = thetaSamp;
                               thetaRange = k2cRef + [obj.configParam.theta_range(1) : obj.configParam.theta_sample_step : obj.configParam.theta_range(2)];
                               thetaRng = thetaRange;
                               
                               [thetaGrid, disparityGrid] = meshgrid(thetaRange, disparityRng);
                               ThetaDisp = [thetaGrid(:) disparityGrid(:)];
                                   
                            else
                                
                                thetaSamp = [obj.configParam.theta_range(1) : obj.configParam.theta_sample_step : obj.configParam.theta_range(2)];
                                thetaSamp0 = thetaSamp;
                                thetaRange = k2cRef + [obj.configParam.theta_range(1) : obj.configParam.theta_sample_step : obj.configParam.theta_range(2)];
                                thetaRng = thetaRange;
                                
                                [thetaGrid, disparityGrid] = meshgrid(thetaRange, disparityRng);
                                ThetaDisp = [thetaGrid(:) disparityGrid(:)];
                                
                                
                                
                                featPtList = VisualLocalizer.DetectFast(imgPrvL);
                                depthMapPrv__ = obj.prvDepthVisual;
                                inValid__ = sub2ind(size(depthMapPrv__), round(featPtList(:,2)), round(featPtList(:,1)));
                                depthList__ = depthMapPrv__(inValid__);
%                                 inId_1 = find(depthList__ > 0);
                                featPtList = featPtList(depthList__ > 0, :);
                                depthList__ = depthList__(depthList__ > 0);
                                inTrackFlag = true(size(featPtList,1),1);
                                
                                pnp_ang_est_max_margin = [deg2rad([-1 1]) obj.configParam.pure_rot_reproject_outlier_threshold];
                               [predPtList, inTrackFlag00] = TrackFeaturePoints2( imgPrvL, imgCur, featPtList,inTrackFlag);     
                                
                               
                                [p2cBodyRotAng,Err, inId] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin,intrMat,featPtList(inTrackFlag00,:),predPtList(inTrackFlag00,:),depthList__(inTrackFlag00), find(inTrackFlag00),true(sum(inTrackFlag00),1),b2c,k2cRef0 - obj.refAngList(end));
                                
                                if 0
                                   figure,subplot(1,2,1),showMatchedFeatures(imgPrvL, imgCur, featPtList(inTrackFlag00,:),predPtList(inTrackFlag00,:));subplot(1,2,2);showMatchedFeatures(imgKeyL, imgCur, featPtList(inId,:),predPtList(inId,:))
                               end
                                
                               
                               
                                
                                
                                thetaP2C_111 = k2cRef - obj.refAngList4(end);
                                thetaRngP2CTemp_111 = thetaP2C_111 + thetaSamp;  
                                if 0
                                    [~, thetaP2COpt_111, ~,~] = VisualLocalizer.New_OnlyP2_new(obj, featPtList(inId,:),predPtList(inId,:), intrMat,[],T_B2C, thetaP2C_111, tx, ty, tz, thetaRngP2CTemp_111);
                                elseif 0
                                    [~, thetaP2COpt_111, ~,~] = VisualLocalizer.New_OnlyP2_new(obj, pt2dPrv, pt2dCur, intrMat,[],T_B2C, thetaP2C_111, tx, ty, tz, thetaRngP2CTemp_111);
                                else
                                    angOptP2C_1_1 = k2cRef0 - obj.refAngList(end);
                                    
                                    
                                    pixprv = pt2dPrv;
                                    
                                    [k2cOpt, thetaP2COpt_111, k2pOpt] = AngleSpace.EpiPolarThreeView(obj, k2cRef0, angOptP2C_1_1, [deg2rad(-0.2) : obj.configParam.theta_sample_step : deg2rad(0.2)], Pix, pixprv, pt2dCur, intrMat, b2c);
                                    
                                end
                                if ~UseGoldenThetaP2C
                                    p2cOnlyP2_111 = thetaP2COpt_111;
                                else
                                    p2cOnlyP2_111 = k2cRef0 - obj.refAngList(end);
                                end
                            end
                            
                             if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                
%                                 obj.accumP2C = [];
                                obj.accumP2CTemp = [];
                                obj.accumP2CTemp = p2cOnlyP2_111;
                            else
                                
                                obj.accumP2CTemp = [obj.accumP2CTemp; obj.accumP2CTemp(end) + p2cOnlyP2_111];
                                
                            end
                            
                            
                            
                            [~, angOptK2C_22222, ~, ~] = VisualLocalizer.New_OnlyP2_new(obj, Pix, pt2dCur, intrMat,[],T_B2C, k2cRef, tx, ty, tz, thetaRng);
                            
                            if size(obj.featPtManager.localTrace.ptIcsX, 2) > 2
                                thetaP2C = k2cRef - obj.refAngList4(end);
                                
                                thetaP2C1 = obj.accumP2CTemp(end) - obj.refAngList4(end);
                                
                                thetaP2C2 = obj.accumP2CTemp(end) - obj.accumP2CTemp(end-1);
                                
                                thetaP2C3 = k2cBody - obj.accumP2CTemp(end-1);
                                
                                % % %                                 angOptP2C__1 = k2cRef - obj.refAngList(end);
                                % % %
                                % % %
                                % % %                                 pixprv = pt2dPrv;
                                % % %
                                % % %                                 [k2cOpt, p2cOpt, k2pOpt] = AngleSpace.EpiPolarThreeView(obj, k2cRef, angOptP2C__1, [deg2rad(-0.2) : obj.configParam.theta_sample_step : deg2rad(0.2)], Pix, pixprv, pt2dCur, intrMat, b2c);
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
                                        thetaP2C = k2cRef - obj.refAngList4(end);
                                    end
                                else
         %                           thetaP2C = k2cRef0 - obj.refAngList(end);
                                     thetaP2C = k2cRef - obj.refAngList(end);
                                end
                                %                                 thetaP2C = angOptK2C_22222 - obj.refAngList4(end);
                                
                                if 0 % size(obj.featPtManager.localTrace.ptIcsX, 2) > FrmNumWithGloden + 1
                                    thetaP2C = k2cRef - obj.refAngList4(end);
                                end
                                  
                                
                                
                                
                                thetaP2C0 = thetaP2C;
                                
                                thetaRngP2CTemp = thetaP2C + thetaSamp;
                                [~, thetaP2COpt, ~,~] = VisualLocalizer.New_OnlyP2_new(obj, pt2dPrv, pt2dCur, intrMat,[],T_B2C, thetaP2C, tx, ty, tz, thetaRngP2CTemp);
                                
                                if UseNewP2CRef
                                    
                                    thetaP2C = thetaP2COpt;
                                end
                                
                                p2cOnlyP2 = thetaP2COpt;
                                
                                
                                
                                
                                if ~UseNewP2CRef
                                    if doRoundingTheta2
                                        thetaP2C = deg2rad(round(rad2deg(thetaP2C0 + obj.thetaNoise)./roundingPrecision2).*roundingPrecision2);
                                    end
                                end
                                
                                if 0 % UseGoldenThetaP2C
                                    thetaP2C = k2cRef0 - obj.refAngList(end);
                                end
                                
                                
% %                                 thetaP2C = thetaP2C - deg2rad(0.02);
% % % % % % % % %                                 if ismember(size(obj.featPtManager.localTrace.ptIcsX, 2), [2 5 8 11 14 17])
% % % % % % % % %                                     thetaP2C = k2cRef0 - obj.refAngList(end)  +  deg2rad(0.05*(1)^(size(obj.featPtManager.localTrace.ptIcsX, 2))) + deg2rad(0.01*(rand(1)-0.5));
% % % % % % % % %                                 else
% % % % % % % % %                                     thetaP2C = k2cRef0 - obj.refAngList(end)  -  deg2rad(0.025*(1)^(size(obj.featPtManager.localTrace.ptIcsX, 2))) + deg2rad(0.01*(rand(1)-0.5));
% % % % % % % % %                                 end
                                
% %                                 thetaP2C = k2cRef0 - obj.refAngList(end);
                                
                                
                                thetaRngP2C = thetaP2C + thetaSamp;
                                thetaSampP2C = thetaRngP2C - thetaP2C;
                                
% % % %                                 pt2dPrv = [obj.featPtManager.localTrace.ptIcsX(inlierId,end-1) obj.featPtManager.localTrace.ptIcsY(inlierId,end-1)];

                                if InheriDispRng
                                    if ~NewPrvDispResamp
                                    DispRngP2C = repmat(dispPrvList_(inlierId),1,size(DispRng,2)) + (DispRng - repmat(DispRng(:,(size(DispRng,2)+1)/2), 1, size(DispRng,2)));
                                    else
                                        DispRngP2C = DispKeyInPrvRng;
                                    end
                                else
                                    DispRngP2C = dispPrvList_(inlierId) + disparityRng;
                                end
                                
                                pt2dPrvR = [pt2dPrv(:,1) - dispPrvList_(inlierId) pt2dPrv(:,2)];
                                
                                
                                if GTTRACKING
                                    pt2dPrv = pixGTPrv;
                                    pt2dPrvR = pixGTRPrv;
                                end
                                
                                
                                
                                if size(obj.featPtManager.localTrace.ptIcsX, 2) <= 3
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
                                    [~, ZZVecPrv1(jkj,:)] = probDensity(dispPrvList_(inlierId(jkj),:), obj.configParam.disparity_sigma,obj.configParam.disparity_beta,obj.configParam.reproj_beta, DispRngP2C(jkj,:),obj.configParam.disparity_sample_interval, 'disparity');
                                end
                                
                                %                             Pix = [obj.featPtManager.localTrace.ptIcsX(inlierId,1) obj.featPtManager.localTrace.ptIcsY(inlierId,1)];
                                
                                ZTruePrv = depthPrvList_(inlierId);
                                
                                
                                metricPrv = intrMat\HomoCoord(pt2dPrv',1);
                                metricPrv = normc(metricPrv);
                                scaleAllPrv = ZTruePrv./metricPrv(3,:)';
                                %                                   scale0 = zTrue./metricPrevPtCcs(3,:)';
                                prvCcsXYZAll = [repmat(scaleAllPrv',3,1).*metricPrv];
                                
                                if 1
                                    [thetaProbP2C,idMP2C,angOptP2C, ~, ~, ~, ~,~,~,~,~,ProbReprojVecAllP2C, ProbReprojVecRAllP2C, ~, ~, ~,~,~, ~, ~,~,~,~,~,~,~,thetaExp_tmpP2C] = ReplayTheta(obj, thetaRngP2C, b2c, prvCcsXYZAll, intrMat, pt2dPrv, disparityRng, baseline, ZZVecPrv1, L2R, pt2dCur, pt2dCurR, ProbZP2C, ProbZP2C_norm, imgCur, imgCurR,thetaSampP2C,r_cam,tx, ty, tz);
                                    
% % % %                                     angOptP2C = angOptP2C - deg2rad(0.01);
                                    
                                    angOptP2C__1 = angOptP2C;
% %                                     angOptP2C = k2cRef0 - obj.refAngList(end);
                                    %                                     [thetaProbP2C_, angOptP2C, thetaExp_tmpP2C_, gtP2C] = New_OnlyP2(obj, pt2dPrv, pt2dCur, intrMat,thetaP2C,T_B2C, k2cRef0);
                                    if UseNewP2Only
                                        
                                        if 0
                                            inFlag1 = find(obj.featPtManager.localTrace.ptIcsX(:,end) > 0 & obj.featPtManager.localTrace.ptIcsY(:,end) > 0);
                                            pixprv = [obj.featPtManager.localTrace.ptIcsX(inFlag1,end-1)  obj.featPtManager.localTrace.ptIcsY(inFlag1,end-1)];
                                            pixcur = [obj.featPtManager.localTrace.ptIcsX(inFlag1,end)  obj.featPtManager.localTrace.ptIcsY(inFlag1,end)];
                                            [thetaProbK2C_, angOptP2C, thetaExp_tmpP2C, ~,thetaRngP2C_use] = New_OnlyP2(obj, pixprv, pixcur, intrMat,angOptP2C,T_B2C, thetaP2C);
                                        else
                                            if 1
                                                [thetaProbP2C, angOptP2C, thetaExp_tmpP2C,thetaRngP2C_use] = VisualLocalizer.New_OnlyP2_new(obj, pt2dPrv, pt2dCur, intrMat,[],T_B2C, thetaP2C, tx, ty, tz, thetaRngP2C);
                                            else
                                                [thetaProbP2C, angOptP2C, thetaExp_tmpP2C,thetaRngP2C_use] = VisualLocalizer.New_OnlyP2_new_old(obj, pt2dPrv, pt2dCur, intrMat,thetaP2C,T_B2C, k2cRef, tx, ty, tz, thetaRngP2C,DispRng,inlierId);
                                            end
                                        end
                                        
                                        
                                        
                                    end
                                else
                                    if 0
                                        existPt1 = Points.Point(pt2dPrv(:,1),pt2dPrv(:,2),intrMat);
                                        existPt2 = Points.Point(pt2dCur(:,1),pt2dCur(:,2),intrMat);
                                        thetaRngP2C_ = [deg2rad(-179) : obj.configParam.theta_sample_step : deg2rad(179)] + (thetaP2C);
                                        thetaSampP2C_ = thetaRngP2C_ - thetaP2C;
                                        [thetaProbP2C_, angOptP2C_, thetaExp_tmpP2C] = UpdateModalAngleOnlyP2NewVersion(rad2deg(thetaRngP2C_),existPt1,existPt2,obj.configParam.reproj_sigma,intrMat,T_B2C);
                                        
                                        gtP2C = k2cRef0 - obj.refAngList(end);
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
%                                 angOptP2C = k2cRef0 - obj.refAngList(end);
                                
                                
                                [thetaRngP2C, thetaSampP2C] = ArrangeThetaRng(obj,rad2deg(thetaExp_tmpP2C),thetaP2C,sc,thetaSampP2C);
%                                 
                                thetaProbP2C = interp1(thetaRngP2CBeforeNewPlatform, thetaProbP2CBeforeNewPlatform, thetaRngP2C);
                                thetaProbP2C(isnan(thetaProbP2C)) = min(thetaProbP2CBeforeNewPlatform);
                                if 0
                                    figure,plot(rad2deg(thetaRngP2CBeforeNewPlatform), thetaProbP2CBeforeNewPlatform);hold on;plot(rad2deg(thetaRngP2C), thetaProbP2C);
                                end
                                
                                
                                
%                                 k2cRef = obj.refAngList4(end) + angOptP2C;
                                k2cRefErr = rad2deg(k2cRef - k2cRefBak);
                                
                                p2cRefErrGT = [rad2deg(angOptP2C - (k2cRef0 - obj.refAngList(end)))];
                                k2cRefErrGT = [rad2deg(k2cRef - k2cRef0) rad2deg(k2cRefBak - k2cRef0)  p2cRefErrGT];
                                akgdsaj = 1;
                            else
                                angOptP2C = []; thetaProbP2C = [];thetaRngP2C = [];thetaSampP2C = []; thetaProbP2CBeforeNewPlatform = []; thetaRngP2CBeforeNewPlatform = []; thetaP2C = [];thetaP2C0 = [];% thetaRngP2C - thetaP2C;
                            end
                            thetaRange = k2cRef + [obj.configParam.theta_range(1) : obj.configParam.theta_sample_step : obj.configParam.theta_range(2)];
                            thetaRng = thetaRange;
                            [thetaGrid, disparityGrid] = meshgrid(thetaRange, disparityRng);
                            ThetaDisp = [thetaGrid(:) disparityGrid(:)];
                            
                            
                        
                            
                            ReprojErrVecTmp = {}; ReprojErrVecRTmp = {}; ReprojErrVecTmpMatT = {}; ReprojErrVecRTmpMatT = {};
                            ValidFeatFlag = []; ValidFeatFlag0 = []; ValidFeatFlagFusion = [];ValidFeatFlagQuant = [];
                            thetaRng0 = thetaRng;
                            if 0
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
                                    
                                    
                                    
                                    [reProjErr2(jj,1),err2(:,jj)] = AngleSpace.ProjectErrorUnderTransform2(obj.configParam.reproj_sigma, k2cCam(1:3,1:3), k2cCam(1:3,4), Pix, pt2dCur, intrMat);
                                    
                                    
                                    
                                    
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
                                    
                                    [errVec, reprojErrVecTmp] = NormalizeVector(pixKeyVecTmp - repmat(pt2dCur,length(disparityRng) ,1)); [errVecR, reprojErrVecRTmp] = NormalizeVector(pixKeyVecRTmp - repmat(pt2dCurR,length(disparityRng) ,1));
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
                                    %                                 Probb(jj,:,:) = tempProb;
                                    
                                    
                                    %                            if jj <= (length(thetaRng)+1)/2 && jj >= (length(thetaRng)+1)/2 - 8
                                    if jj == (length(thetaRng)+1)/2 % && jj >= (length(thetaRng)+1)/2 - 8
                                        
                                        randId = 10;
                                        tmpProb = Probb(jj,randId,:);
                                        tmpProb2 = permute(Probb(jj,:,:),[2 3 1]);
                                        maxx = (max(tmpProb2'));
                                        maxx(isnan(maxx) | isinf(maxx)) = [];
                                        maxxsum = sum((maxx));
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
                                %                     end
                                %                     a = max(permute(Prob,[3 1 2]));
                                %                     aa = permute(a,[2 3 1]);
                                % % %                             vsl = VisualLocalizer(cam, csa, robotConfig.tracking_config);
                                %                             b = max(permute(Probb,[3 1 2]));
                                b = mean(permute(Probb,[3 1 2]));
                                bb = permute(b,[2 3 1]);
                                
                                bb(isinf(bb)) = 0;
                                bb(isnan(bb)) = 0;
                                thetaProb = sum(bb');
                                thetaProb = thetaProb./sum(thetaProb);
                                [~,idM] = max(thetaProb);
                                angOpt = thetaRng(idM);
                                
                                
                                errEpiline = sum(err2)';
                                [~,idM2] = min(errEpiline);
                                angOptEpi = thetaRng(idM2);
                                
                                
                                rati = Rati1;
                                thetaProb111 = thetaProb./max(thetaProb);
                                thetaProb222 = thetaProb111(thetaProb111 >= rati);
                                thetaProb222 = thetaProb222./sum(thetaProb222);
                                
                                if 1
                                    angOpt = deg2rad(dot(rad2deg(thetaRng(thetaProb111 >= rati)), thetaProb222));
                                    
                                    thetaExp_tmp0 = deg2rad(dot(rad2deg(thetaSamp(thetaProb111 >= rati)), thetaProb222));
                                else
                                    angOpt = mean([angOpt angOptEpi]);
                                    thetaExp_tmp0 = angOpt - k2cRef;
                                end
                                
                                
                                
                                
                                
                                
                            else
                                
                                [thetaProb,idM,angOpt, ReprojErrVecTmp, ReprojErrVecRTmp, ReprojErrVecTmpMatT, ReprojErrVecRTmpMatT,ValidFeatFlag,ValidFeatFlag0,ValidFeatFlagFusion,ValidFeatFlagQuant,ProbReprojVecAll,ProbReprojVecRAll, PixKeyVecTmp, PixKeyVecRTmp, Probb,validFeatFlagFusion,validFeatFlag, ProbReprojVec, ProbReprojVecR,bb,b,pixKeyVecTmpXCoordMatStack,pixKeyVecTmpYCoordMatStack,pixKeyVecRTmpXCoordMatStack,pixKeyVecRTmpYCoordMatStack, ...
                                    thetaExp_tmp0, errEpiline, angOptEpiInit, depthC2KInd_ind]       =   ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, pt2dCur, pt2dCurR,ProbZ, ProbZTmp_norm, imgCur, imgCurR,thetaSamp,r_cam,tx, ty, tz);
                                
                                
                                
                                
                            end
                            
                            k2cRef0;
                            angOptEpi = angOptEpiInit;
                            
                            angOptInit = angOpt;
                            thetaRngBefore00 = thetaRng;
                            thetaSampBefore00 = thetaSamp;
                            
                            if size(obj.featPtManager.localTrace.ptIcsX, 2) <= 3
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
                                    %                                 inFlag = find(obj.featPtManager.localTrace.ptIcsX(:,end) > 0 & obj.featPtManager.localTrace.ptIcsY(:,end) > 0);
                                    %                                 pixkey = [obj.featPtManager.localTrace.ptIcsX(inFlag,1)  obj.featPtManager.localTrace.ptIcsY(inFlag,1)];
                                    %                                 pixcur = [obj.featPtManager.localTrace.ptIcsX(inFlag,end)  obj.featPtManager.localTrace.ptIcsY(inFlag,end)];
                                    %                                 [thetaProbK2C_, angOptK2C_, thetaExp_tmpK2C, ~,thetaRngP2C_use] = New_OnlyP2(obj, pixkey, pixcur, intrMat,angOpt,T_B2C, k2cRef);
                                else
                                    [thetaProbK2C_, angOptK2C_, thetaExp_tmpK2C, ~,thetaRngP2C_use] = New_OnlyP2(obj, Pix, pt2dCur, intrMat,k2cRef0,T_B2C, k2cRef);
                                    %                                 inFlag = find(obj.featPtManager.localTrace.ptIcsX(:,end) > 0 & obj.featPtManager.localTrace.ptIcsY(:,end) > 0);
                                    %                                 pixkey = [obj.featPtManager.localTrace.ptIcsX(inFlag,1) obj.featPtManager.localTrace.ptIcsY(inFlag,1)];
                                    %                                 pixcur = [obj.featPtManager.localTrace.ptIcsX(inFlag,end)  obj.featPtManager.localTrace.ptIcsY(inFlag,end)];
                                    %                                 [thetaProbK2C_, angOptK2C_, thetaExp_tmpK2C, ~,thetaRngP2C_use] = New_OnlyP2(obj, pixkey, pixcur, intrMat,k2cRef0,T_B2C, k2cRef);
                                    
                                end
                                
                            else
                                %                                 [thetaProbK2C_, angOptK2C_, thetaExp_tmpK2C, ~,thetaRngP2C_use] = VisualLocalizer.New_OnlyP2_new(obj, Pix, pt2dCur, intrMat,angOpt,T_B2C, k2cRef);
                                if 1
                                    [thetaProbK2C_, angOptK2C_, thetaExp_tmpK2C, thetaRngK2C_use] = VisualLocalizer.New_OnlyP2_new(obj, Pix, pt2dCur, intrMat,[],T_B2C, k2cRef, tx, ty, tz, thetaRng);
                                else
                                    [thetaProbK2C_, angOptK2C_, thetaExp_tmpK2C,thetaRngK2C_use] = VisualLocalizer.New_OnlyP2_new_old(obj, Pix, pt2dCur, intrMat,k2cRef,T_B2C, k2cRef, tx, ty, tz, thetaRng,DispRng,inlierId);
                                end
                            end
                            figure(18),hold on;
                            if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                clf;
                            end
                            figure(18);plot(rad2deg(thetaRngK2C_use) - rad2deg(k2cRef0), thetaProbK2C_);hold on;plot(rad2deg(angOptK2C_ - k2cRef0), max(thetaProbK2C_),'or');grid on;
                            saveas(gcf,fullfile(probPath,sprintf(strcat('newOnlyP2_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('newOnlyP2_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                            
% % % %                             if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
% % % %                                 
% % % % %                                 obj.accumP2C = [];
% % % %                                 obj.accumP2CTemp = [];
% % % %                                 
% % % %                                 p2cOnlyP2 = angOptK2C_;
% % % %                                 obj.accumP2CTemp = p2cOnlyP2;
% % % %                             else
% % % %                                 
% % % %                                 obj.accumP2CTemp = [obj.accumP2CTemp; obj.accumP2CTemp(end) + p2cOnlyP2];
% % % %                                 
% % % %                             end
%                             obj.accumP2CTemp = [obj.accumP2CTemp; obj.accumP2CTemp(end) + p2cOnlyP2];
                            
                            
                            
                            
                            
                            
                            
                             if IterThetaNum > 1
                                 if ~UseNewP2Only
                                     [thetaRng, thetaSamp] = ArrangeThetaRng(obj,rad2deg(thetaExp_tmp0),k2cRef,sc,thetaSamp0);
                                  else 
                                     if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                         angOpt = angOptK2C_;
                                         [thetaRng, thetaSamp] = ArrangeThetaRng(obj,rad2deg(thetaExp_tmpK2C),k2cRef,sc,thetaSamp0);
                                     else
                                         [thetaRng, thetaSamp] = ArrangeThetaRng(obj,rad2deg(thetaExp_tmp0),k2cRef,sc,thetaSamp0);
                                         
                                     end
                                 end
                            end
                            if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                if UseNewP2Only
                                    angOpt = angOptK2C_;
                                    thetaProb = thetaProbK2C_;
                                    thetaExp_tmpK2C;
                                    angOpt3 = angOpt;
                                    angOptList0 = angOpt;
                                end
                            end
                            
                             s = Setting([]);
                                s.configParam.disparityBoundary = obj.configParam.disparity_error;
                                s.configParam.disparityIntervel = obj.configParam.disparity_sample_step;
                                s.configParam.disparityRange = -obj.configParam.disparity_error : obj.configParam.disparity_sample_step : obj.configParam.disparity_error;
                                s.configParam.disparityBin = length(s.configParam.disparityRange);
                                
s.configParam.disparityOrginRange = DispRng;
                                s.configParam.angleIntervel = rad2deg(obj.configParam.theta_sample_step);
                                s.configParam.angleBoundary = rad2deg(obj.configParam.theta_range(2));
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
                                    
                                    [angleResult ] = VisualLocalizer.AnglePredict_old(PixHomo',pt2dCurHomo',tx, ty, tz);
                                    angleOrgin = rad2deg(k2cRef);
                                    angleResultDiff     = angleResult - angleOrgin;
                                    angleResultRangeMin = angleResult - angleOrgin+ (-0.1);
                                    angleResultRangeMax = angleResult - angleOrgin+   0.1;
                                    
                                    co = cosd(angleResult) ;
                                    so = sind(angleResult) ;
                                    %                                     depthResult = (r_cam .*(existPt2.ptHomo(1,:).*(-tz.*co+tx.*so+tz) - (-tz.*so-tx.*co+tx))) ...
                                    %                                         ./ (existPt1.ptHomo(1,:).*co+so-existPt2.ptHomo(1,:).*(-existPt1.ptHomo(1,:).*so+co));
                                    mm = Modals.Modal(s.configParam,length(inlierId));
                                    %                                   mm.angleModal = obj.angleModalOrg;
                                    if ~NewOnlyP2
                                        mm.angleModal = Modals.UpdateModalAngle1(mm, s.configParam,[],[],angleResultRangeMin,angleResultRangeMax);
                                    else
                                        mm.angleModal = Modals.UpdateModalAngleOnlyP2(mm, s.configParam,[],[],[],[],angleResultDiff);
                                    end
                                    thetaProb = mm.angleModal./sum(mm.angleModal); thetaProbOnlyP2 = thetaProb;
                                    
                                    if 0
                                        angOpt = deg2rad(dot(thetaProb,rad2deg(thetaRng)));
                                    else
                                        rati = Rati1;
                                        thetaProb111 = thetaProb./max(thetaProb);
                                        thetaProb222 = thetaProb111(thetaProb111 >= rati);
                                        thetaProb222 = thetaProb222./sum(thetaProb222);
                                        
                                        
                                        angOpt = deg2rad(dot(rad2deg(thetaRng(thetaProb111 >= rati)), thetaProb222));
                                    end
                                    
                                    
                                    
                                    
                                    [~,idM] = max(thetaProb);
                                    if ONLYP2_1_Max
                                        angOpt = thetaRng(idM);
                                    end
                                    
                                    
                                    if EXPZERO
                                        
                                        idM = (length(thetaRng) + 1)/2;
                                        angOpt = thetaRng(idM);
                                        
                                    end
                                    
                                    
                                    
                                    
                                    angOpt3 = angOpt;
                                    
                                    
                                    
                                    
                                    
                                    
                                    
                                    
                                    if 1
                                        figure(20)
                                        if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                            probDir = probPath;
                                            clf;
                                        end
                                        
                                        figure(20),subplot(2,1,1);hold on;plot(rad2deg(thetaSamp+ (k2cRef - k2cRef0)), thetaProb);grid on; title('1st theta from only p2');
                                        plot(rad2deg(angOpt3 - k2cRef0), max(thetaProb),'or');
                                        %                                         saveas(gcf,fullfile(probPath,sprintf(strcat('cutThetaDiff2_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('cutThetaDiff2_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                                        %                                         saveas(gcf,fullfile(probPath,sprintf(strcat('cutThetaDiff2_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.fig'),length(dir(fullfile(probPath,strcat('cutThetaDiff2_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.fig'))))+1)));
                                    end
                                    
                                    
                                    
                                    if UPDATETracking_2
                                        
                                        if 0
                                            %                                         thetaProbUse = thetaProb(ismember(round(rad2deg(thetaSamp),3),round(obj.setting.configParam.angleRange,3)));
                                            if 0
                                                thetaProbUse = thetaProb(ismember(round(rad2deg(thetaSamp),3),round(obj.setting.configParam.angleRange,3)));
                                            else
                                                thetaProbUse = interp1(round(rad2deg(thetaSamp),3),thetaProb,round(obj.setting.configParam.angleRange,3));
                                                thetaProbUse(isnan(thetaProbUse)) = min(thetaProb);
                                                if 0
                                                    figure, plot(rad2deg(thetaSamp), thetaProb);hold on;plot(obj.setting.configParam.angleRange, thetaProbUse,'-x');legend('orig','interp')
                                                end
                                                
                                                
                                            end
                                            thetaProbUse = thetaProbUse./max(thetaProbUse);
                                            thetaProbUse_ = thetaProbUse;
                                            if sum(thetaProbUse <= obj.configParam.only_p2_prob_cutoff_threshold) > 0 && sum(thetaProbUse <= obj.configParam.only_p2_prob_cutoff_threshold) ~= length(thetaProbUse)
                                                thetaProbUse_(thetaProbUse > obj.configParam.only_p2_prob_cutoff_threshold) = 1; % max(thetaProbUse(thetaProbUse <= obj.configParam.only_p2_prob_cutoff_threshold));
                                            elseif sum(thetaProbUse <= obj.configParam.only_p2_prob_cutoff_threshold) == length(thetaProbUse)
                                                
                                                thetaProbUse_ = thetaProbUse;
                                            else
                                                thetaProbUse_ = ones(length(thetaProbUse_),1);
                                            end
                                            
                                            
                                            thetaProbUse__ = thetaProbUse_./sum(thetaProbUse_);
                                            if ~NewOnlyP2
                                                [curPredPtIcs,inTrackFlag,curPredPtIcs0,inTrackFlag0,weightRaww,candXX,candYY] = NewTracking(obj, size(obj.featPtManager.localTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs, AngleModalOrg, obj.keyFrameDepth, obj.prvDepthVisual, obj.depthVisual,DepthProbility, DepthId, thetaProbUse__,1,DispRefineUse__);
                                                %                                             [~,~,~,~,weightRaw,candX,candY] = NewTracking(obj, size(obj.featPtManager.localTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs, AngleModalOrg, obj.keyFrameDepth, obj.prvDepthVisual, obj.depthVisual,DepthProbility, DepthId, thetaProbUse__,1,DispRefineUse__);
                                            else
                                                [curPredPtIcs,inTrackFlag,curPredPtIcs0,inTrackFlag0] = NewTracking_new(obj, size(obj.featPtManager.localTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs, AngleModalOrg, obj.keyFrameDepth, obj.prvDepthVisual, obj.depthVisual,DepthProbility, DepthId, thetaProbUse__,1,DispRefineUse__);
                                                
                                            end
                                            
                                            
                                            idI = find(obj.featPtManager.localTrace.ptIcsX(:,end-1) ~= -1 & obj.featPtManager.localTrace.ptIcsY(:,end-1) ~= -1);
                                            tempPt = -1.*ones(size(obj.featPtManager.localTrace.ptIcsX,1),2);
                                            tempPt(idI,:) = curPredPtIcs;
                                            
                                            iff = (tempPt(:,1) ~= -1);
                                            idff = find(ismember(find(iff),inlierId));
                                            
                                            iff(setdiff(1:size(tempPt,1),inlierId)) = false;
                                            tempPt2 = tempPt(iff,:);
                                            
                                            candX = candXX(idff,:);
                                            candY = candYY(idff,:);
                                            weightRaw = weightRaww(idff,:);
                                            
                                            
                                            candXT = candX';
                                            candX_reshape = reshape(candXT(:),obj.setting.configParam.angleBin, obj.setting.configParam.disparityBin, size(candX,1) );
                                            
                                            candYT = candY';
                                            candY_reshape = reshape(candYT(:),obj.setting.configParam.angleBin, obj.setting.configParam.disparityBin, size(candX,1) );
                                            
                                            weightRawT = weightRaw';
                                            weightRaw_reshape = reshape(weightRawT(:),obj.setting.configParam.angleBin, obj.setting.configParam.disparityBin, size(candX,1) );
                                            %                                             candX_reshape = permute(candX_reshape,[2 3 1]);
                                            
                                            
                                            
                                            if 1
                                                if 0
                                                    checkGT = 10;
                                                    tempCandX = reshape(candX(idff(checkGT),:),obj.setting.configParam.angleBin,obj.setting.configParam.disparityBin);
                                                    tempCandY = reshape(candY(idff(checkGT),:),obj.setting.configParam.angleBin,obj.setting.configParam.disparityBin);
                                                    tempWeight = reshape(weightRaw(idff(checkGT),:),obj.setting.configParam.angleBin,obj.setting.configParam.disparityBin)*1000;tempWeight = tempWeight./max(tempWeight(:));
                                                    %                                                 figure,imshow(tempWeight, [])
                                                    idMaxWeight = find(tempWeight(:) > 0.8);
                                                    %                                                 [idMaxWeight] = find(tempWeight(:) == max(tempWeight(:)))
                                                    [weightY,weightX] = ind2sub(size(tempWeight),idMaxWeight)
                                                    figure,subplot(1,2,1);imshow(imresize(tempWeight,2),[]);subplot(1,2,2);imshow(imgCur);hold on;plot(tempCandX(idMaxWeight),tempCandY(idMaxWeight),'.r');plot(pixGT(checkGT,1),pixGT(checkGT,2),'.g');legend('candidate','gt');
                                                end
                                                
                                                
                                                asgarvn = 1;
                                                
                                                
                                                if 0
                                                    
                                                    try
                                                        [~, errP2] = NormalizeVector(tempPt((inlierId),:) - pt2dCur);
                                                        
                                                        [~, errP2_old] = NormalizeVector(pt2dCur - pixGT);
                                                        [~, errP2_new] = NormalizeVector(tempPt((inlierId),:) - pixGT);
                                                        
                                                        figure(1),clf;hist([errP2_old errP2_new],100);legend('old','new')
                                                        saveas(gcf,fullfile(probPath,sprintf(strcat('refinedTracking_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('refinedTracking_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                                                        
                                                    catch
                                                        sdgahl = 1;
                                                    end
                                                    
                                                    
                                                    %
                                                    figure,imshow(imgCur);hold on;plot(pt2dCur(:,1),pt2dCur(:,2),'.r');plot(pixGT(:,1),pixGT(:,2),'.b');plot(tempPt((inlierId),1),tempPt((inlierId),2),'.g');legend('old','gt','new');
                                                    figure,hist([errP2_old errP2_new],500);legend('old','new')
                                                    figure,subplot(1,2,1);hold on;plot(tempPt((inlierId),1), tempPt((inlierId),2),'.g');plot(pt2dCur(:,1),pt2dCur(:,2),'.r');axis equal;subplot(1,2,2);plot(errP2);
                                                end
                                                if 0
                                                    pt2dCur = tempPt((inlierId),:);
                                                    
                                                    obj.featPtManager.localTrace.ptIcsX(inlierId,end) = tempPt((inlierId),1);
                                                    obj.featPtManager.localTrace.ptIcsY(inlierId,end) = tempPt((inlierId),2);
                                                end
                                            end
                                            
                                        else
                                            
                                            [thetaProbUse, thetaProbUse_, thetaProbUse__, curPredPtIcs, weightRaw, candX, candY,candX_reshape, candY_reshape, weightRaw_reshape, idff] = UpdateNewTracking(obj, thetaSamp, thetaProb, prevFeatPtList0, ptCcsZ0, intrMat, AngleModalOrg, DepthProbility, DepthId, inlierId,DispRefineUse__);
                                        end
                                    end
                                    
                                    
                                    
                                else
                                    figure(20)
                                    if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                        probDir = probPath;
                                        clf;
                                    end
                                    
                                    figure(20),subplot(2,1,1);hold on;plot(rad2deg(thetaSamp+ (k2cRef - k2cRef0)), thetaProb);grid on; title('1st theta from only p2');
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
                                    [k2cOpt, p2cOpt, k2pOpt] = AngleSpace.EpiPolarThreeView(obj, angOpt3, angOptP2C__, [deg2rad(-0.2) : obj.configParam.theta_sample_step : deg2rad(0.2)], Pix, pixprv, pt2dCur, intrMat, b2c);
                                end
                                
                                
                                thetaProb0 = thetaProb;
                                if 0
                                    if CUTTHETAANDFORCEPLATFORM
                                        idM_Vec00 = find(thetaProb./max(thetaProb) > obj.configParam.theta_prob_cutoff_threshold) ;
                                        diffTheta = abs(diff(thetaProb./max(thetaProb)));
                                        if 1
                                            diffTheta = abs(diff(thetaProb./max(thetaProb)));
                                            %                                 [minv,mini1]=findpeaks(diffTheta,'SortStr','descend');
                                            %                                 mini1 = sort(mini1(1:2), 'ascend');
                                            idf = find(diffTheta < 0.001);
                                            %                                 [idxCell,idx] = splitIndex2(idf');
                                            %                                     mini1 = [idxCell{2}(1) idxCell{2}(end)];
                                            
                                            
                                            % %                                     base_theta_diff_margin =  obj.configParam.theta_diff_margin;
                                            
                                            idThetaDiff  = find(diffTheta <= obj.configParam.theta_diff_margin);
                                            [idxCell,idx] = splitIndex2(idThetaDiff');
                                            marginSize = 1; minStep = max(0.001, obj.configParam.theta_diff_margin/10);
                                            %                                     while length(idxCell) <= 2
                                            while ~exist('edc0','var')
                                                marginSize = marginSize + 1;
                                                %                                         idThetaDiff  = find(diffTheta <= marginSize*minStep*obj.configParam.theta_diff_margin);
                                                idThetaDiff  = find(diffTheta <= marginSize*minStep);
                                                [idxCell,idx] = splitIndex2(idThetaDiff');
                                                if marginSize > 1000
                                                    break;
                                                end
                                                
                                                for edc = 1 : length(idxCell)
                                                    if ismember(idM-1, idxCell{edc})
                                                        edc0 = edc;
                                                        break;
                                                    end
                                                    
                                                end
                                                
                                            end
                                            
                                            
                                            for edc = 1 : length(idxCell)
                                                if ismember(idM-1, idxCell{edc})
                                                    edc0 = edc;
                                                    break;
                                                end
                                            end
                                            if ~exist('edc0', 'var')
                                                %                                         edc0 = 2;
                                                edc0 = length(idxCell);
                                            end
                                            
                                            
                                            if 0
                                                idThetaDiff = intersect(idThetaDiff, mini1(1):mini1(2));
                                            else
                                                idThetaDiff = idxCell{edc0};
                                            end
                                            
                                            if edc0 == 1
                                                [minv,mini12321]=findpeaks(thetaProb,'SortStr','descend');
                                                %                                          figure(914),clf; plot(theta,val,'-b'), hold on;plot(theta(minID),val(minID),'ro');
                                                idThetaDiff = mini12321 -1;
                                            end
                                            
                                            
                                            
                                            %                                 idThetaDiff = [mini1(1) : mini1(end)];
                                            idThetaDiffExtend = [idThetaDiff(1) : min(idThetaDiff(end) + 1,length(diffTheta)-1)];
                                            
                                        else
                                            idThetaDiff = idM_Vec00;
                                            idThetaDiffExtend = [idThetaDiff(1) : idThetaDiff(end) + 1];
                                        end
                                        
                                        try
                                            leftoverMarg = 1;
                                            dltLen = length(idThetaDiffExtend) - length(idThetaDiff) + leftoverMarg;
                                            [~, distThetaDiff] = NormalizeVector(repmat(diffTheta(idThetaDiffExtend(1)),dltLen,1) - diffTheta(idThetaDiffExtend(end - dltLen+1:end))');
                                            absDistThetaDiff = abs(distThetaDiff);
                                            distThetaDiffSign = repmat(diffTheta(idThetaDiffExtend(1)),dltLen,1) - diffTheta(idThetaDiffExtend(end - dltLen+1:end))';
                                            absDistThetaDiffDiff = abs(diff(absDistThetaDiff));
                                            absDistThetaDiffSign = (distThetaDiff);
                                            if min(absDistThetaDiffDiff) < 0.0001
                                                svdk = 1;
                                            end
                                            if 0
                                                [closestVal] = min(absDistThetaDiff(distThetaDiffSign < 0));
                                            else
                                                [closestVal] = min(absDistThetaDiff(distThetaDiffSign < 1111110));
                                            end
                                            closestId = idThetaDiff(end - (leftoverMarg - 1)) + max(find(distThetaDiff == closestVal)) - 1;
                                            
                                            %                                 cutId = [idThetaDiff(1) idThetaDiffExtend(closestId) + 1];
                                            cutId = [idThetaDiff(1) (closestId) + 1];
                                            
                                            idM_Vec00 = [cutId(1) : cutId(2)];
                                            
                                            rati = Rati1; 0.8; 0.5; 0.8;0.5; 0.8;0.9; 0; 0.8;
                                            thetaExp_ = dot(rad2deg(thetaSamp), thetaProb);
                                            thetaProb1 = thetaProb./max(thetaProb);
                                            thetaProb2 = thetaProb1(thetaProb1 > rati);
                                            thetaProb2 = thetaProb2./sum(thetaProb2);
                                            thetaExp_2 = dot(rad2deg(thetaSamp(thetaProb1 > rati)), thetaProb2); % 0.5; 0.8; 0.0
                                            
                                            if EXPZERO
                                                thetaExp_2 = 0;
                                            end
                                            
                                            if ForceThetaExpZero
                                                thetaExp_2 = 0;
                                            end
                                            
                                            %%
                                            if 0 % 20191030
                                                
                                                radius_ = max(max(abs(thetaExp_ - rad2deg(thetaSamp(idM_Vec00(1)))), abs(thetaExp_ - rad2deg(thetaSamp(idM_Vec00(end))))),max(0.1,rad2deg(obj.configParam.theta_sample_step)));
                                                radius_2 = max(max(abs(thetaExp_2 - rad2deg(thetaSamp(idM_Vec00(1)))), abs(thetaExp_2 - rad2deg(thetaSamp(idM_Vec00(end))))),max(0.1,rad2deg(obj.configParam.theta_sample_step)));
                                                pltfrm = [thetaExp_ - radius_ thetaExp_ + radius_];
                                                [DD] = pdist2(double(pltfrm'),double(rad2deg(thetaSamp)'),'euclidean');
                                                [minDD, minDDId] = min(DD');
                                                if 0
                                                    %                                          pltfrm_ = [thetaExp_ - rad2deg(obj.configParam.expectation_theta_radius) thetaExp_ + rad2deg(obj.configParam.expectation_theta_radius)];
                                                    pltfrm_ = [thetaExp_2 - rad2deg(obj.configParam.expectation_theta_radius) thetaExp_2 + rad2deg(obj.configParam.expectation_theta_radius)];
                                                else
                                                    %                                          pltfrm_ = [thetaExp_ - radius_ thetaExp_ + radius_];
                                                    %                                          sc = floor(radius_2/rad2deg(obj.configParam.theta_sample_step ));
                                                    sc = ceil(radius_2/rad2deg(obj.configParam.theta_sample_step ));
                                                    if sc < 1
                                                        sc = 1;
                                                    end
                                                    %% good, reserved
                                                    sc = max([sc 4]); % 0.05
                                                    sc = max([1 4]);
                                                    sc = Sc;
                                                    % % %                                             sc = max([sc 20]); % 0.01
                                                    
                                                    %% try
                                                    %                                             sc = max([sc 6]);
                                                    %                                             sc = max([5 6]);
                                                    if ~SOFTP2
                                                        pltfrm_ = [thetaExp_2 - min(110.2,rad2deg(sc*obj.configParam.theta_sample_step)) thetaExp_2 + min(110.2,rad2deg(sc*obj.configParam.theta_sample_step))];
                                                    else
                                                        pltfrm_ = [thetaExp_2 - min(0.2,rad2deg(sc*obj.configParam.theta_sample_step)) thetaExp_2 + min(0.2,rad2deg(sc*obj.configParam.theta_sample_step))];
                                                    end
                                                end
                                                pltfrm_1 = [deg2rad(pltfrm_(1)) : obj.configParam.theta_sample_step : deg2rad(pltfrm_(end))] + k2cRef;
                                                doHalf = 0;
                                                
                                                if doHalf
                                                    pltfrm_1 = pltfrm_1(1:(length(pltfrm_1) + 1)/2 + 0);
                                                    
                                                end
                                                
                                                %                                     thetaRng = pltfrm_1;
                                                %                                     thetaSamp = thetaRng - k2cRef;
                                                
                                                idM_Vec00 = [minDDId(1) : minDDId(2)];
                                                asba = 1;
                                            else
                                                
                                                idM_Vec00 = (length(thetaProb) + 1 ) / 2;
                                                
                                            end
                                        catch
                                            asgkl = 1;
                                        end
                                        figure(19);
                                        if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                            probDir = probPath;  % fullfile(pwd, 'prob');
                                            %                                     saveas(gcf,fullfile(probDir,sprintf('prob_%05d.png',length(dir(fullfile(probDir,'prob_*.png')))+1)));
                                            %                                     saveas(gcf,fullfile(probDir,sprintf('prob_%05d.fig',length(dir(fullfile(probDir,'prob_*.fig')))+1)));
                                            clf;
                                        end
                                        %                                 idM_Vec11 = find(thetaProb./max(thetaProb) > obj.configParam.theta_prob_cutoff_threshold) ;
                                        
                                        
                                        if CUTTHETAANDFORCEPLATFORM_NODIFF % 1 %~DIFFTHETATHR
                                            idM_Vec00 = find(thetaProb./max(thetaProb) > obj.configParam.theta_prob_cutoff_threshold) ;
                                        end
                                        if 0 %mod(length(idM_Vec00),2) == 0
                                            idM_Vec00 = [idM_Vec00 idM_Vec00(end) + 1];
                                        end
                                        
                                        if ~FORCEEXPEVEN
                                            figure(19),%clf,%subplot(1,2,1);
                                            plot(rad2deg(thetaSamp+ (k2cRef - k2cRef0)), thetaProb);hold on;plot(rad2deg(thetaSamp(idM_Vec00)+ (k2cRef - k2cRef0)), thetaProb(idM_Vec00), '.r');grid on;
                                            %                                 subplot(1,2,2);plot(rad2deg(thetaSamp), thetaProb);hold on;plot(rad2deg(thetaSamp(idM_Vec11)), thetaProb(idM_Vec11), '.r');grid ongrid on;
                                            if 1 %size(obj.featPtManager.localTrace.ptIcsX, 2) <= 3
                                                saveas(gcf,fullfile(probPath,sprintf(strcat('cutThetaDiff_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('cutThetaDiff_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                                            end
                                        end
                                        thetaProb0 = thetaProb;
                                        idM_Vec00Diff = idM_Vec00;
                                        
                                        
                                        
                                        if FORCETHETAPLATFORM1
                                            if 0 %20191120
                                                thetaProb(idM_Vec00) = max(thetaProb);
                                            else
                                                thetaProb(idM_Vec00) = thetaProb(idM_Vec00);
                                            end
                                        end
                                        thetaProb = thetaProb./sum(thetaProb);
                                        
                                        %                             else
                                        %                                 idM_Vec11 = find(thetaProb./max(thetaProb) > obj.configParam.theta_prob_cutoff_threshold) ;
                                        %                                 idM_Vec11 = find(thetaProb./max(thetaProb) > obj.configParam.theta_prob_cutoff_threshold) ;
                                        %                                 figure(19),clf;plot(rad2deg(thetaSamp), thetaProb);hold on;plot(rad2deg(thetaSamp(idM_Vec11)), thetaProb(idM_Vec11), '.r');grid on;
                                    end
                                end
                            
                            if FORCEEXPEVEN
                                if  1 %~ONLYP2
                                    expandNum = obj.configParam.expectation_theta_expand_num;
                                    
                                    if exist('pltfrm_1','var')
                                        pltfrm_1 = [pltfrm_1(1):obj.configParam.theta_sample_step2:pltfrm_1(end)];
                                    else
                                        
                                        if 1
                                            if size(obj.featPtManager.localTrace.ptIcsX, 2) <= 3
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
                                                    if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
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
                                            pltfrm_ = [thetaExp_2 - min(110.2,rad2deg(sc*obj.configParam.theta_sample_step)) thetaExp_2 + min(110.2,rad2deg(sc*obj.configParam.theta_sample_step))];
                                        else
                                            pltfrm_ = [thetaExp_2 - min(0.2,rad2deg(sc*obj.configParam.theta_sample_step)) thetaExp_2 + min(0.2,rad2deg(sc*obj.configParam.theta_sample_step))];
                                        end
                                        pltfrm_1 = [deg2rad(pltfrm_(1)) : obj.configParam.theta_sample_step : deg2rad(pltfrm_(end))] + k2cRef;
                                        
                                        
                                        pltfrm_1 = [pltfrm_1(1):obj.configParam.theta_sample_step2:pltfrm_1(end)];
                                    end
                                    
                                    
%                                     expandNum = (length(thetaProbUse__) - length(pltfrm_1))/2;
                                    expandNum = (length(thetaSamp0) - length(pltfrm_1))/2;
                                    
                                    thetaProbBeforeNewPlatform = thetaProb;
                                    thetaRngBeforeNewPlatform = thetaRng;
                                    thetaSampBeforeNewPlatform = thetaSamp;
                                    
                                    thetaRng = [(-obj.configParam.theta_sample_step2.*[expandNum:-1:1] + pltfrm_1(1)) pltfrm_1 (pltfrm_1(end) + obj.configParam.theta_sample_step2.*[1 : expandNum])];
                                             
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
% % % % %                                 s = Setting([]);
% % % % %                                 s.configParam.disparityBoundary = obj.configParam.disparity_error;
% % % % %                                 s.configParam.disparityIntervel = obj.configParam.disparity_sample_step;
% % % % %                                 s.configParam.disparityRange = -obj.configParam.disparity_error : obj.configParam.disparity_sample_step : obj.configParam.disparity_error;
% % % % %                                 s.configParam.disparityBin = length(s.configParam.disparityRange);
% % % % %                                 s.configParam.disparityOrginRange = DispRng;
% % % % %                                 s.configParam.angleIntervel = rad2deg(obj.configParam.theta_sample_step);
% % % % %                                 s.configParam.angleBoundary = rad2deg(obj.configParam.theta_range(2));
% % % % %                                 s.configParam.angleRange  = -s.configParam.angleBoundary : s.configParam.angleIntervel : s.configParam.angleBoundary;
% % % % %                                 s.configParam.angleBin = length(s.configParam.angleRange);
% % % % %                                 s.configParam.angleOrginRange = rad2deg(k2cRef) + s.configParam.angleRange;
% % % % %                                 s.configParam.tableSize = s.configParam.angleBin*s.configParam.disparityBin;
% % % % %                                 s.configParam.disparitySigma = obj.setting.configParam.disparitySigma;
% % % % %                                 s.configParam.disparityBeta = obj.setting.configParam.disparityBeta;
% % % % %                                 s.configParam.gaussSigma = obj.setting.configParam.gaussSigma;
% % % % %                                 s.configParam.gaussBeta = obj.setting.configParam.gaussBeta;
% % % % %                                 s.configParam.wx = obj.setting.configParam.wx;
% % % % %                                 s.configParam.wy = obj.setting.configParam.wy;
                                
% %                                 if ONLYP2
% %                                     [angleResult ] = VisualLocalizer.AnglePredict(PixHomo',pt2dCurHomo',tx, ty, tz);
% %                                     angleOrgin = rad2deg(k2cRef);
% %                                     angleResultDiff     = angleResult - angleOrgin;
% %                                     angleResultRangeMin = angleResult - angleOrgin+ (-0.1);
% %                                     angleResultRangeMax = angleResult - angleOrgin+   0.1;
% %                                     
% %                                     co = cosd(angleResult) ;
% %                                     so = sind(angleResult) ;
% % %                                     depthResult = (r_cam .*(existPt2.ptHomo(1,:).*(-tz.*co+tx.*so+tz) - (-tz.*so-tx.*co+tx))) ...
% % %                                         ./ (existPt1.ptHomo(1,:).*co+so-existPt2.ptHomo(1,:).*(-existPt1.ptHomo(1,:).*so+co));
% %                                     mm = Modals.Modal(s.configParam,length(inlierId));
% %                                     %                                   mm.angleModal = obj.angleModalOrg;
% %                                     mm.angleModal = Modals.UpdateModalAngle1(mm, s.configParam,[],[],angleResultRangeMin,angleResultRangeMax);
% %                                     if 0
% %                                         figure,plot(s.configParam.angleRange, mm.angleModal);grid on;
% %                                     end
% %                                 end
                                  
                                %                                 ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, pt2dCur, pt2dCurR,ProbZ,ProbZTmp_norm,imgCur,imgCurR, thetaSamp,r_cam,tx, ty, tz);
                                
                                
                                if 1 % ~ONLYP2
                                    if 1
                                        
                                        
                                        
                                      
                                        if  1 % ONLYP2
                                            
                                            if 0 %~ONLYP2_2
                                                [thetaProb,idM,angOpt, ReprojErrVecTmp, ReprojErrVecRTmp, ReprojErrVecTmpMatT, ReprojErrVecRTmpMatT,ValidFeatFlag,ValidFeatFlag0,ValidFeatFlagFusion,ValidFeatFlagQuant,ProbReprojVecAll, ProbReprojVecRAll, PixKeyVecTmp, PixKeyVecRTmp, Probb, ...
                                                    validFeatFlagFusion,validFeatFlag, ProbReprojVec, ProbReprojVecR,bb,b,pixKeyVecTmpXCoordMatStack,pixKeyVecTmpYCoordMatStack,pixKeyVecRTmpXCoordMatStack,pixKeyVecRTmpYCoordMatStack]...
                                                    = ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, pt2dCur, pt2dCurR,ProbZ, ProbZTmp_norm, imgCur, imgCurR,thetaSamp,r_cam,tx, ty, tz);
                                            else
                                                
                                                
                                                obj.configParam.reproj_sigma = obj.configParam.reproj_sigma0 ;
                                                obj.configParam.reproj_sigma_right = obj.configParam.reproj_sigma_right0;
                                                
                                                
                                                
                                                
                                                [thetaProb__,idM__,angOpt__, ReprojErrVecTmp, ReprojErrVecRTmp, ReprojErrVecTmpMatT, ReprojErrVecRTmpMatT,ValidFeatFlag,ValidFeatFlag0,ValidFeatFlagFusion,ValidFeatFlagQuant,ProbReprojVecAll, ProbReprojVecRAll, PixKeyVecTmp, PixKeyVecRTmp, Probb, ...
                                                    validFeatFlagFusion,validFeatFlag, ProbReprojVec, ProbReprojVecR,bb,b,pixKeyVecTmpXCoordMatStack,pixKeyVecTmpYCoordMatStack,pixKeyVecRTmpXCoordMatStack,pixKeyVecRTmpYCoordMatStack,~, errEpiline, angOptEpi,depthC2KInd_ind]...
                                                    = ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, pt2dCur, pt2dCurR,ProbZ, ProbZTmp_norm, imgCur, imgCurR,thetaSamp,r_cam,tx, ty, tz);
                                                
                                                ProbReprojVecAll00 = ProbReprojVecAll;
                                                ProbReprojVecAll0 = ProbReprojVecAll;
                                                
                                                
                                                
                                                if 0  % 20191108  change update p2
                                                    if size(weightRaw_reshape, 1) > size(ProbReprojVecAll0,1)
                                                        compMat = zeros((size(weightRaw_reshape,1) - size(ProbReprojVecAll0,1))/2,size(weightRaw_reshape,2),size(weightRaw_reshape,3));
                                                        
                                                        ProbReprojVecAll0 = [compMat; ProbReprojVecAll0;compMat];
                                                    else
                                                        compMat = [];
                                                    end
                                                    
                                                    if size(weightRaw_reshape, 1) < size(ProbReprojVecAll0,1)
                                                        compMat = zeros((size(ProbReprojVecAll0,1) - size(weightRaw_reshape,1))/2,size(weightRaw_reshape,2),size(weightRaw_reshape,3));
                                                        
                                                        weightRaw_reshape = [compMat; weightRaw_reshape;compMat];
                                                    else
                                                        compMat = [];
                                                    end
                                                    ProbReprojVecAll_reshape = permute(ProbReprojVecAll0, [1 3 2]);
                                                    %                                             weightRaw_reshape = permute(weightRaw_reshape, [1 3 2]);
                                                    
                                                    
                                                    ProbReprojVecAll = ProbReprojVecAll0.*permute(weightRaw_reshape,[1 3 2]);
                                                    
                                                    
                                                    
                                                    [ExpCoord_Reproj, ExpCoord_OnlyP2, errReproj, errOnlyP2, errPt2dCur] =...
                                                        CalcP2Exp(obj, candX, candY, weightRaw, idff,pt2dCur, pixGT, imgCur,candX_reshape,candY_reshape,weightRaw_reshape, ProbReprojVecAll_reshape, compMat);
                                                    %                                            figure(1),clf;hist([errReproj errOnlyP2],100) ;legend('reproj exp','onlyP2 exp');
                                                    %                                            figure(1),clf;hist([errReproj errOnlyP2],100) ;legend('old p2 exp','new p2 exp');
                                                    %                                            figure(1),clf;hist([errPt2dCur errOnlyP2],100) ;legend('init NewTracking','new p2 exp');
                                                    %                                            figure(1),clf;histogram(errPt2dCur,100);hold on;histogram(errOnlyP2, 100);legend('init NewTracking','new p2 exp');
                                                    figure(1),clf;hist([errPt2dCur errOnlyP2],100);legend('init NewTracking','new p2 exp');
                                                    
                                                    %                                             figure(1),clf;hist([errP2_old errP2_new],100);legend('old','new')
                                                    saveas(gcf,fullfile(probPath,sprintf(strcat('refinedTracking_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('refinedTracking_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                                                
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
                                        
                                        modulate_curve_quant_size = obj.configParam.modulate_curve_quant_size;
                                        
                                        
                                        
                                        
                                        
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
                                    [angleResult ] = VisualLocalizer.AnglePredict(PixHomo',pt2dCurHomo',tx, ty, tz);
                                    angleOrgin = rad2deg(k2cRef);
                                    angleResultDiff     = angleResult - angleOrgin;
                                    angleResultRangeMin = angleResult - angleOrgin+ (-0.1);
                                    angleResultRangeMax = angleResult - angleOrgin+   0.1;
                                    
                                    co = cosd(angleResult) ;
                                    so = sind(angleResult) ;
                                    %                                     depthResult = (r_cam .*(existPt2.ptHomo(1,:).*(-tz.*co+tx.*so+tz) - (-tz.*so-tx.*co+tx))) ...
                                    %                                         ./ (existPt1.ptHomo(1,:).*co+so-existPt2.ptHomo(1,:).*(-existPt1.ptHomo(1,:).*so+co));
                                    mm = Modals.Modal(s.configParam,length(inlierId));
                                    %                                   mm.angleModal = obj.angleModalOrg;
                                    if ~NewOnlyP2
                                    mm.angleModal = Modals.UpdateModalAngle1(mm, s.configParam,[],[],angleResultRangeMin,angleResultRangeMax);
                                    else
                                        mm.angleModal = Modals.UpdateModalAngleOnlyP2(mm, s.configParam,[],[],[],[],angleResultDiff);
                                    end
                                    thetaProb = mm.angleModal./sum(mm.angleModal);
                                    angOpt = deg2rad(dot(thetaProb,rad2deg(thetaRng)));
                                    [~,idM] = max(thetaProb);
                                    if 0
                                        figure,plot(s.configParam.angleRange, mm.angleModal);grid on;
                                    end
                                      
                                    
                                end
                                    
                                
                                 xx1 = (round(pixKeyVecTmpXCoordMatStack./modulate_curve_quant_size).*modulate_curve_quant_size); yy1 = (round(pixKeyVecTmpYCoordMatStack./modulate_curve_quant_size).*modulate_curve_quant_size);
                                xx1 = permute(xx1,[1 3 2])./modulate_curve_quant_size; yy1 = permute(yy1,[1 3 2])./modulate_curve_quant_size;
                                zz1 = permute(ProbZOld,[1 3 2]);
                                tempTable0 = zeros((1./obj.configParam.modulate_curve_quant_size)*size(rgb2gray(imgCur)));
                                errList = zeros(length(inlierId),2);
                                imgSize = size(rgb2gray(imgCur));
                                if 0
                                    draw = 0;
                                    for featId = 1 : length(inlierId)
                                        
                                        xx = xx1(idM_Vec00,:,featId);yy = yy1(idM_Vec00,:,featId);
                                        %                                     xx = round(xx./modulate_curve_quant_size).*modulate_curve_quant_size; yy = round(yy./modulate_curve_quant_size).*modulate_curve_quant_size;
                                        %                                     xx = permute(xx, [1 3 2]); yy = permute(yy, [1 3 2]);
                                        baseX = min(xx(:))-1; baseY = min(yy(:))-1;
                                        
                                        zz = zz1(idM_Vec00,:,featId); %zz = permute(zz, [1 3 2]);
                                        if 0
                                            epilineEquivX = xx(:,(size(xx,2)+1)/2);
                                            epilineEquivY = yy(:,(size(yy,2)+1)/2);
                                        else
                                            epilineEquivX = xx((size(xx,1)+1)/2,:)';
                                            epilineEquivY = yy((size(yy,1)+1)/2,:)';
                                            
                                        end
                                        [C, dist] = VisualLocalizer.fitline([epilineEquivX.*modulate_curve_quant_size epilineEquivY.*modulate_curve_quant_size]');
                                        points3 = lineToBorderPoints(C', imgSize);
                                        
                                        
                                        [retVal1, dist1] = Pt2Line(points3(1:2), points3(3:4), modulate_curve_quant_size.*[xx(:) yy(:)]);
                                        
                                        
                                        
                                        coordVal = [retVal1 zz(:)];
                                        coordValX = reshape(coordVal(:,1), size(xx));
                                        coordValY = reshape(coordVal(:,2), size(yy));
                                        coordValZ = reshape(coordVal(:,3), size(zz));
                                        
                                        coordValXRound = round(coordValX./modulate_curve_quant_size).*modulate_curve_quant_size;
                                        coordValYRound = round(coordValY./modulate_curve_quant_size).*modulate_curve_quant_size;
                                        coordValXRoundBig = round(coordValXRound./modulate_curve_quant_size);
                                        coordValYRoundBig = round(coordValYRound./modulate_curve_quant_size);
                                        
                                        baseXVal = min(coordValXRoundBig(:))-1; baseYVal = min(coordValYRoundBig(:))-1;
                                        
                                        
                                        retVal1X = reshape(retVal1(:,1),size(xx));
                                        retVal1Y = reshape(retVal1(:,2),size(yy));
                                        dist1Mat = reshape(dist1, size(xx));
                                        retVal1XQuant = round(retVal1X./modulate_curve_quant_size).*modulate_curve_quant_size;
                                        retVal1YQuant = round(retVal1Y./modulate_curve_quant_size).*modulate_curve_quant_size;
                                        [EE] = pdist2(double([unique(retVal1XQuant(:))]),double([retVal1X(:) ]),'euclidean');
                                        [FF] = pdist2(double([unique(retVal1YQuant(:))]),double([retVal1Y(:) ]),'euclidean');
                                        EEMask = EE < modulate_curve_quant_size./2;
                                        FFMask = FF < modulate_curve_quant_size./2;
                                        zzMat = repmat(zz(:)',size(EE,1),1);
                                        zzMatY = repmat(zz(:)',size(FF,1),1);
                                        %                                     [yyy,xxx] = ind2sub(size(EE), find(EEMask > 0));
                                        %                                     xxxMat = reshape(xxx, size(xx));
                                        %                                     yyyMat = reshape(yyy, size(yy));
                                        
                                        zSum = sum((EEMask.*zzMat)');
                                        zSumY = sum((FFMask.*zzMatY)');
                                        
                                        
                                        if 0
                                            figure,subplot(1,2,1);plot(unique(retVal1XQuant(:)), zSum);title('x dir');subplot(1,2,2);plot(unique(retVal1YQuant(:)), zSumY);title('y dir');
                                            
                                            figure,plot(dot([epilineEquivX.*modulate_curve_quant_size epilineEquivY.*modulate_curve_quant_size ones(length(epilineEquivX),1)]', repmat(C,1,length(epilineEquivX))));title('pt on line');
                                        end
                                        
                                        %                                     tempTable = tempTable0; % zeros((1./obj.configParam.modulate_curve_quant_size)*size(rgb2gray(imgCur)));
                                        tempTable = zeros(round([(max(yy(:)) - min(yy(:)) + 1)]), round([(max(xx(:)) - min(xx(:)) + 1)]));
                                        
                                        tempTableVal = zeros(round([(max(coordValYRoundBig(:)) - min(coordValYRoundBig(:)) + 1)]), round([(max(coordValXRoundBig(:)) - min(coordValXRoundBig(:)) + 1)]));
                                        
                                        
                                        Indtmp = [];
                                        if 0
                                        for yt = 1 : size(xx,1)
                                            tempTableVal1 = zeros(size(tempTableVal));
%                                             coordValXTmp = unique(coordValX(yt,:));
                                            
                                            
                                            wheelNew2 = interp1(coordValX(yt,:),coordValZ(yt,:),coordValXRound(yt,:));
                                            wheelNew2(isnan(wheelNew2)) = coordValZ(yt,isnan(wheelNew2));
                                            roundCoordValXBig = coordValXRoundBig(yt,:);
                                            roundCoordValYBig = coordValYRoundBig(yt,:);
                                            indtmp = sub2ind(size(tempTableVal), roundCoordValYBig' - baseYVal, roundCoordValXBig' - baseXVal);
                                            Indtmp = [Indtmp; indtmp];
                                            tempTableVal1(indtmp) = wheelNew2;  %coordValZ(yt,:);
                                            tempTableVal = tempTableVal + tempTableVal1;
                                        end
                                        subX = coordValXRoundBig(find(tempTableVal) > 0);
                                        subY = coordValYRoundBig(find(tempTableVal) > 0);
                                        [~, sortIId] = sort(subX,'ascend');
                                        tempTableVal_ = tempTableVal(find(tempTableVal > 0));
% % % %                                         figure,plot(subX(sortIId).*modulate_curve_quant_size, tempTableVal_(sortIId));
                                        
                                        end
                                        
                                        
                                        
                                        zzInd = sub2ind(size(tempTable), round(yy(:)-baseY), round(xx(:)-baseX));
                                        tempTable(zzInd) = zz;
                                        ans1 = sum(tempTable,1);
                                        [~,maxzzId] = max(ans1);
                                        
                                        ans2 = sum(tempTable,2);
                                        [~,maxzzId2] = max(ans2);
                                        
                                        if 0
                                            [ansId] = sort(unique(xx(:)));
                                        else
                                            ansId = ([min(xx(:)).*modulate_curve_quant_size : modulate_curve_quant_size : max(xx(:)).*modulate_curve_quant_size])./modulate_curve_quant_size;
                                            ansId2 = ([min(yy(:)).*modulate_curve_quant_size : modulate_curve_quant_size : max(yy(:)).*modulate_curve_quant_size])./modulate_curve_quant_size;
                                        end
                                        errList(featId,:) = [abs(pt2dCur(featId,1) - ansId(maxzzId)*modulate_curve_quant_size) abs(pixGT(featId,1) - ansId(maxzzId)*modulate_curve_quant_size)];
                                        %                                     plot(ansId*modulate_curve_quant_size,ans1)
                                        %                                 figure,plot([ansId]/20,ans1([ansId]))
                                        filtWin = 1; filtWinY = 1;
                                        if draw == 1
                                            figure(2),clf;subplot(2,3,1);imshow(imgCur);hold on;plot(pt2dCur(featId,1), pt2dCur(featId,2),'.r');plot(pixGT(featId,1),pixGT(featId,2),'.b'); plot(xx(:).*modulate_curve_quant_size, yy(:).*modulate_curve_quant_size, '.g');plot(epilineEquivX.*modulate_curve_quant_size,epilineEquivY.*modulate_curve_quant_size,'-y','LineWidth',2);line(points3(:, [1,3])', points3(:, [2,4])','Color','c'); plot(pt2dCur(featId,1), pt2dCur(featId,2),'.r');plot(pixGT(featId,1),pixGT(featId,2),'.b');legend('tracking','gt','epiline'); title(sprintf('tracking: %0.5f %0.5f\ngt:      %0.5f %0.5f\n',pt2dCur(featId,1),pt2dCur(featId,2),pixGT(featId,1),pixGT(featId,2)));
%                                             subplot(2,3,2);plot(ansId*modulate_curve_quant_size,medfilt1(ans1,1));hold on;plot(pt2dCur(featId,1),max(ans1),'or');plot(pixGT(featId,1),max(ans1),'ob');subplot(2,3,3);plot(ansId2*modulate_curve_quant_size,medfilt1(ans2,filtWinY));hold on;plot(pt2dCur(featId,2),max(ans2),'or');plot(pixGT(featId,2),max(ans2),'ob');
%                                             subplot(2,3,4);plot(subX(sortIId).*modulate_curve_quant_size, tempTableVal_(sortIId));hold on;plot(pt2dCur(featId,1),max(zSum),'or');plot(pixGT(featId,1),max(zSum),'ob');title('interp accum');
                                            subplot(2,3,5);plot(unique(retVal1XQuant(:)), medfilt1(zSum,filtWin));hold on;plot(pt2dCur(featId,1),max(zSum),'or');plot(pixGT(featId,1),max(zSum),'ob');title('x dir');subplot(2,3,6);plot(unique(retVal1YQuant(:)), medfilt1(zSumY,filtWinY));hold on;plot(pt2dCur(featId,2),max(zSumY),'or');plot(pixGT(featId,2),max(zSumY),'ob');title('y dir');
                                            drawnow;
                                            saveas(gcf,fullfile(probPath,sprintf(strcat('tempModuCurve_',probDir(end-14:end),'____',sprintf('%04d',sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('tempModuCurve_',probDir(end-14:end),'____',sprintf('%04d',(sum(obj.keyFrameFlagList))),'__*.png'))))+1)));
                                        end
                                        %
                                    end
                                    figure(2);clf,hist(errList(errList(:,1)>0,1) - errList(errList(:,1)>0,2),100);title('abs(lk-peak) - abs(gt-peak)')
% % %                                     saveas(gcf,fullfile(probPath,sprintf(strcat('moduCurve_',probDir(end-14:end),'____',sprintf('%04d',sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('moduCurve_',probDir(end-14:end),'____',sprintf('%04d',(sum(obj.keyFrameFlagList))),'__*.png'))))+1)));
                                end
                                asdv = 1;
                                
%                                 xx1 = pixKeyVecTmpXCoordMatStack(idM_Vec00,:,:);yy1 = pixKeyVecTmpYCoordMatStack(idM_Vec00,:,:);
%                                 xx1 = round(xx1./modulate_curve_quant_size).*modulate_curve_quant_size; yy1 = round(yy1./modulate_curve_quant_size).*modulate_curve_quant_size;
%                                 xx1 = permute(xx1, [2 1 3]); yy1 = permute(yy1, [2 1 3]);
%                                 
%                                 zz1 = ProbZOld(idM_Vec00,:,:); zz1 = permute(zz1, [2 1 3]);
%                                 
%                                 tempTable1 = zeros((1./obj.configParam.modulate_curve_quant_size).^2*size(rgb2gray(imgCur),1)*size(rgb2gray(imgCur),2)*length(inlierId),1);
%                                 tempTable1 = reshape(tempTable1, size(rgb2gray(imgCur),2),size(rgb2gray(imgCur),1),[]);
%                                 tempTable1 = cat(length(inlierId),tempTable1);

                                if 0
                                    errXMat = pixKeyVecTmpXCoordMatStack -  pt2dCurXMat;
                                    errYMat = pixKeyVecTmpYCoordMatStack -  pt2dCurYMat;
                                    [~,errXYMat] = NormalizeVector([errXMat(:) errYMat(:)]);
                                    
                                    checkThetaId = 10;
                                    errXMat1 = errXMat(checkThetaId,:,:);
                                    errXMat1 = permute(errXMat1, [2 3 1]);
                                    errYMat1 = errYMat(checkThetaId,:,:);
                                    errYMat1 = permute(errYMat1, [2 3 1]);
                                    
                                    [~, errCheck] = NormalizeVector([errXMat1(:) errYMat1(:)]);
                                    
                                    errRef = ReprojErrVecTmp{checkThetaId};
                                    
                                    figure,plot(errCheck - errRef)
                                    
                                    
                                    
                                    errXMatR = pixKeyVecRTmpXCoordMatStack -  pt2dCurXMatR;
                                    errYMatR = pixKeyVecRTmpYCoordMatStack -  pt2dCurYMatR;
                                    [~,errXYMatR] = NormalizeVector([errXMatR(:) errYMatR(:)]);
                                    
                                    checkThetaId = 10;
                                    errXMatR1 = errXMatR(checkThetaId,:,:);
                                    errXMatR1 = permute(errXMatR1, [2 3 1]);
                                    errYMatR1 = errYMatR(checkThetaId,:,:);
                                    errYMatR1 = permute(errYMatR1, [2 3 1]);
                                    
                                    [~, errCheckR] = NormalizeVector([errXMatR1(:) errYMatR1(:)]);
                                    
                                    errRefR = ReprojErrVecRTmp{checkThetaId};
                                    
                                    figure,plot(errCheckR - errRefR)
                                    
                                    
                                    
                                end
                                
                                figure(19);
                                if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                    probDir = probPath;  % fullfile(pwd, 'prob');
                                    %                                     saveas(gcf,fullfile(probDir,sprintf('prob_%05d.png',length(dir(fullfile(probDir,'prob_*.png')))+1)));
                                    %                                     saveas(gcf,fullfile(probDir,sprintf('prob_%05d.fig',length(dir(fullfile(probDir,'prob_*.fig')))+1)));
                                    clf;
                                end
                                
                                
                                
                                figure(19),%clf,%subplot(1,2,1);
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
                                if 1 %size(obj.featPtManager.localTrace.ptIcsX, 2) <= 3
                                    saveas(gcf,fullfile(probPath,sprintf(strcat('cutThetaDiff_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('cutThetaDiff_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                                end
                                
                                thetaProb1 = thetaProb;
                                
                                
                                
                                if FORCETHETAPLATFORM1
                                    thetaProb(idM_Vec00) = max(thetaProb);
                                    
                                end
                                thetaProb = thetaProb./sum(thetaProb);
                                
                                
                                
                                
                            end
                            
                            
                            figure(31);
                            if jkl == 1  %DEPTHITERNUM
                                if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                    probDir = probPath;  % fullfile(pwd, 'prob');
                                    %                                     saveas(gcf,fullfile(probDir,sprintf('prob_%05d.png',length(dir(fullfile(probDir,'prob_*.png')))+1)));
                                    %                                     saveas(gcf,fullfile(probDir,sprintf('prob_%05d.fig',length(dir(fullfile(probDir,'prob_*.fig')))+1)));
                                    clf;
                                end
                            end
%                             figure(31),hold on;plot(rad2deg(thetaSamp), thetaProb);plot(rad2deg(thetaSamp(idM)), thetaProb(idM),'*r'); % title(sprintf('k2cRef: %0.5f\nk2cPnP: %0.5f\nk2cProb: %0.5f\nsigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f',rad2deg(k2cRef), rad2deg(k2cPnp), rad2deg(thetaRng(idM)),obj.configParam.reproj_sigma,obj.configParam.reproj_beta,obj.configParam.reproj_sigma_right,obj.configParam.reproj_beta_right));grid on;
                            %                             title(sprintf('sigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f',obj.configParam.reproj_sigma,obj.configParam.reproj_beta,obj.configParam.reproj_sigma_right,obj.configParam.reproj_beta_right));
                            
                            
                            b2cPmat = GetPctBody2Cam(obj.coordSysAligner, 1);
                            if 0
                                k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(thetaRng(idM)),zeros(3,1));
                            else
                                k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cRef0),zeros(3,1));
                            end
                            k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
                            
                            metricPrevPtCcsGT = intrMat\HomoCoord(Pix',1);
                            metricPrevPtCcsGT = normc(metricPrevPtCcsGT);
                            if 0
                                scaleAllGT = depthListGT(inlierId)./metricPrevPtCcsGT(3,:)';
                            else
                                dispGTComp = dispList(inlierId) - disparityErrorRound;
                                depthListGTComp = intrMat(1,1).*norm(obj.camModel.transVec1To2)./(dispGTComp + (princpPtR(1) - princpPtL(1)));
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
                                %                            figure(33),clf;subplot(2,1,1);plot(ZZVec1(randId,:),ProbZ(randId,:));hold on;plot(ZZVec1(randId,:),probReprojVec(randId,:));
                                %                                           subplot(2,1,2);plot(ZZVec1(randId,:),tmpProb(:)');
                                
                                
                                figure(36),clf;subplot(4,1,1);plot(ProbZ');title(num2str(rad2deg(thetaRng(idM)))); subplot(4,1,2);plot(ProbReprojVec{idM}');subplot(4,1,3);plot(ProbReprojVecR{idM}'); subplot(4,1,4);plot(tmpProb22');title(num2str(maxxsum2));
                                
                                
                                %                             figure,plot([ProbReprojVecR{idM}(1,:) ; ProbReprojVec{idM}(1,:); (ProbReprojVecR{idM}(1,:) + ProbReprojVec{idM}(1,:)); (ProbReprojVecR{idM}(1,:) .* ProbReprojVec{idM}(1,:)).*1000]')
                                
                                
                                figure(37),clf;subplot(1,2,1);imshow(imgCur);hold on;plot(PixKeyVecTmp{idM}(:,1), PixKeyVecTmp{idM}(:,2),'.g');plot(pt2dCur(:,1), pt2dCur(:,2),'.r');title(num2str(rad2deg(thetaRng(idM))));
                                subplot(1,2,2);imshow(imgCurR);hold on;plot(PixKeyVecRTmp{idM}(:,1), PixKeyVecRTmp{idM}(:,2),'.g');plot(pt2dCurR(:,1), pt2dCurR(:,2),'.r');title('cur right');
                            end
                            %                     figure(31),hold on;plot( thetaProb);title(sprintf('k2cRef: %0.5f\nk2cPnP: %0.5f\nk2cProb: %0.5f',rad2deg(k2cRef), rad2deg(k2cPnp), rad2deg(thetaRng(idM))));
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
                            errTheta = thetaProbAll(:,1,2) - thetaProbAll(:,7,3);
                            errZ = ProbZAll(1,:,:) - ProbZAll(6,:,:); errZ = errZ(:);
                            
                            if 0 %BOTHLR
                                %                                 ValidFeatFlag((length(thetaRng)+1)/2,:) = 1;
                                ValidFeatFlag(idM,:) = 1;
                            end
                            ValidFeatFlag_bak = ValidFeatFlag;
                            idM_Vec00 = find(thetaProb./max(thetaProb) > obj.configParam.theta_prob_cutoff_threshold) ;

                            if 0
                                ValidFeatFlagFusionWithin = ValidFeatFlagFusion.*ValidFeatFlagQuant;
                            elseif 1
                                idM_Vec00 = find(thetaProb./max(thetaProb) > obj.configParam.theta_prob_cutoff_threshold) ;
                                ValidFeatFlagTmpCut = zeros(size(ValidFeatFlag));
                                if 1
                                    ValidFeatFlagTmpCut(idM_Vec00,:) = 1;
                                    ValidFeatFlagFusionWithin = ValidFeatFlagFusion.*ValidFeatFlagTmpCut;
                                else
                                    idM_Vec000  = find(thetaSamp >= -obj.configParam.ref_theta_trust_margin & thetaSamp <= obj.configParam.ref_theta_trust_margin);
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
                            errrrr = (BB(:,10) - ValidFeatFlagFusionWithin(:,10)./max(ValidFeatFlagFusionWithin(:,10)));
                            %                             wegCoord = [[1:size(bb,2)]' weg'];
                            %                             ind = sub2ind(size(bb), wegCoord);
                            
                            
                            if ~CUTTHETAANDFORCEPLATFORM
                                if 1 %~HARDCUT
                                    if FUSIONCUT
                                        ValidFeatFlag = immultiply(BB,BB > obj.configParam.probZ_ratio_threshold);
                                        %                                     ValidFeatFlag = BB;
                                    end
                                end
                            else
                                
% % %                                 diffTheta = abs(diff(thetaProb./max(thetaProb)));
% % %                                 [minv,mini1]=findpeaks(diffTheta,'SortStr','descend');
% % %                                 mini1 = sort(mini1(1:2), 'ascend');
% % %                                 idThetaDiff  = find(diffTheta <= obj.configParam.theta_diff_margin);
% % %                                 idThetaDiff = intersect(idThetaDiff, mini1(1):mini1(2));
% % %                                 idThetaDiffExtend = [idThetaDiff(1) : idThetaDiff(end) + 5];
% % %                                 [~, distThetaDiff] = NormalizeVector(repmat(diffTheta(idThetaDiffExtend(1)),length(idThetaDiffExtend),1) - diffTheta(idThetaDiffExtend)');
% % %                                 [closestVal] = min(distThetaDiff - diffTheta(idThetaDiffExtend(1)));
% % %                                 closestId = max(find(distThetaDiff == closestVal));
% % %                                 
% % %                                 cutId = [idThetaDiff(1) idThetaDiffExtend(closestId) + 1];
% % %                                 
% % %                                 idM_Vec00 = [cutId(1) : cutId(2)];
%                                 ValidFeatFlag = immultiply(BB,BB > obj.configParam.probZ_ratio_threshold);
                                ValidFeatFlag = zeros(size(BB));
                                ValidFeatFlag(idM_Vec00,:) = 1;
                                %                                     ValidFeatFlag = BB;
                            end
                            
                            
                            ValidFeatFlag_backup = ValidFeatFlag;
                            
                            
                            if 0 % 20190927 try ref_theta_margin
                                idM_Vec0 = find(thetaProb./max(thetaProb) > obj.configParam.theta_prob_cutoff_threshold) ;
                                
                                ValidFeatFlagTmp0 = zeros(size(ValidFeatFlag));
                                ValidFeatFlagTmp0(idM_Vec0,:) = 1;
                                ValidFeatFlag = ValidFeatFlagTmp0;
                                
                            elseif FORCEEVENPLATFORM % 20190927 try ref_theta_margin
                                
                                ValidFeatFlagTmpCut000 = zeros(size(ValidFeatFlag));
                                idM_Vec00  = find(thetaSamp >= -obj.configParam.ref_theta_trust_margin - 0 & thetaSamp <= obj.configParam.ref_theta_trust_margin);
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
                                figure(3),clf;imshow(ValidFeatFlag, [])
                                saveas(gcf,fullfile(probPath,sprintf(strcat('cutTheta_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('cutTheta_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                            end
                            
                            
                            
                            ValidFeatFlagNew = ProbZTmp_norm > obj.configParam.probZ_ratio_threshold;
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
                            if size(obj.featPtManager.localTrace.ptIcsX, 2) <= FrmNumWithGloden + 1
                                
                                dltDispMat = DispRng - DispRng(:,(size(DispRng,2)+1)/2);
                                
%                                 boundDispId = find(abs(round(disparityRng,3)) > 0.5);
                                 
                                updatedProbZSum(updatedProbZSum < ProbZCutOffThr & round(abs(dltDispMat),3) > 0.6) = 0;
                                updatedProbZSum = updatedProbZSum./repmat(max(updatedProbZSum')',1,size(updatedProbZSum,2));
                            end
                            
                            updatedProbZSum0 = updatedProbZSum;
                            
                            if 1  % ONLYP2_2
                                if 0
                                    s = Setting([]);
                                    s.configParam.disparityBoundary = obj.configParam.disparity_error;
                                    s.configParam.disparityIntervel = obj.configParam.disparity_sample_step;
                                    s.configParam.disparityRange = -obj.configParam.disparity_error : obj.configParam.disparity_sample_step : obj.configParam.disparity_error;
                                    s.configParam.disparityBin = length(s.configParam.disparityRange);
                                    
                                    s.configParam.disparityOrginRange = DispRng;
                                    s.configParam.angleIntervel = rad2deg(obj.configParam.theta_sample_step2);
                                    
                                    if 0 %obj.configParam.disparity_sample_step ~= obj.configParam.disparity_sample_step2
                                        s.configParam.angleBoundary = round(rad2deg(thetaSamp(end)),3);
                                    end
                                    s.configParam.angleRange  = -s.configParam.angleBoundary : s.configParam.angleIntervel : s.configParam.angleBoundary;
                                    
                                    angRngTmp = [s.configParam.angleRange(1) : rad2deg(obj.configParam.theta_sample_step2) :s.configParam.angleRange(end)];
                                    
                                    %                                 dl = length(s.configParam.angleRange) - length(thetaRng).*1;
                                    dl = length(angRngTmp) - length(thetaRng).*1;
                                    if dl < 0
                                        s.configParam.angleRange = [angRngTmp s.configParam.angleRange(end)+[1:1:-dl]*rad2deg(obj.configParam.theta_sample_step2)];
                                    else
                                        
                                        s.configParam.angleRange = angRngTmp(1:length(thetaRng));
                                    end
                                    
                                    
                                    s.configParam.angleBin = length(s.configParam.angleRange);
                                    s.configParam.angleOrginRange = rad2deg(k2cRef) + s.configParam.angleRange;
                                    s.configParam.tableSize = s.configParam.angleBin*s.configParam.disparityBin;
                                    s.configParam.disparitySigma = obj.setting.configParam.disparitySigma;
                                    s.configParam.disparityBeta = obj.setting.configParam.disparityBeta;
                                    s.configParam.gaussSigma = obj.setting.configParam.gaussSigma;
                                    s.configParam.gaussBeta = obj.setting.configParam.gaussBeta;
                                    s.configParam.wx = obj.setting.configParam.wx;
                                    s.configParam.wy = obj.setting.configParam.wy;
                                    
                                    
                                    
                                    pt2dCurHomo = (inv(intrMat)*pextend(pt2dCur'))';
                                    [angleResult ] = VisualLocalizer.AnglePredict(PixHomo',pt2dCurHomo',tx, ty, tz);
                                    angleOrgin = rad2deg(k2cRef);
                                    angleResultDiff     = angleResult - angleOrgin;
                                    angleResultRangeMin = angleResult - angleOrgin+ (-0.1);
                                    angleResultRangeMax = angleResult - angleOrgin+   0.1;
                                    
                                    co = cosd(angleResult) ;
                                    so = sind(angleResult) ;
                                    %                                     depthResult = (r_cam .*(existPt2.ptHomo(1,:).*(-tz.*co+tx.*so+tz) - (-tz.*so-tx.*co+tx))) ...
                                    %                                         ./ (existPt1.ptHomo(1,:).*co+so-existPt2.ptHomo(1,:).*(-existPt1.ptHomo(1,:).*so+co));
                                    mm = Modals.Modal(s.configParam,length(inlierId));
                                    %                                   mm.angleModal = obj.angleModalOrg;
                                    if ~NewOnlyP2
                                        mm.angleModal = Modals.UpdateModalAngle1(mm, s.configParam,[],[],angleResultRangeMin,angleResultRangeMax);
                                    else
                                        mm.angleModal = Modals.UpdateModalAngleOnlyP2(mm, s.configParam,[],[],[],[],angleResultDiff);
                                    end
                                    thetaProb = mm.angleModal./sum(mm.angleModal);
                                    thetaProbInterp = interp1(s.configParam.angleRange + rad2deg(k2cRef), thetaProb, rad2deg(thetaRng));
                                    thetaProbInterp(isnan(thetaProbInterp)) = min(thetaProb);
                                    if 0
                                        checkInterp =  thetaSamp + k2cRef - thetaRng;
                                        figure, plot(s.configParam.angleRange + rad2deg(k2cRef), thetaProb);hold on;plot(rad2deg(thetaRng), thetaProbInterp,'-x');legend('orig','interp');
                                        
                                    end
                                    
                                    
                                    
                                    angOpt = deg2rad(dot(thetaProbInterp,rad2deg(thetaRng)));
                                    [~,idM] = max(thetaProb);
                                    
                                else
                                    
                                    
                                    obj.configParam.reproj_sigma = obj.configParam.reproj_sigma0 * obj.configParam.reproj_sigma_scale;
                                    obj.configParam.reproj_sigma_right = obj.configParam.reproj_sigma_right0 * obj.configParam.reproj_sigma_scale_right;
                                    
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
                                    figure(20)
                                    if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                        probDir = probPath;
                                        %                                         clf;
                                    end
                                    if 0
                                        figure(20),subplot(2,1,2);hold on;plot(rad2deg(thetaSamp+ (k2cRef - k2cRef0)), thetaProbInterp);grid on;
                                        plot(rad2deg(angOpt4 - k2cRef0), max(thetaProbInterp),'or');
                                    else
                                        figure(20),subplot(2,1,2);hold on;plot(rad2deg(thetaSamp+ (k2cRef - k2cRef0)), thetaProb);grid on;title('2nd theta from p2 and z');
                                        plot(rad2deg(angOpt4 - k2cRef0), max(thetaProb),'or');
                                        thetaProbOut = thetaProb;
                                        thetaRngOut = thetaRng;
                                        
                                    end
                                    saveas(gcf,fullfile(probPath,sprintf(strcat('cutThetaDiff2_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('cutThetaDiff2_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                                    %                                     saveas(gcf,fullfile(probPath,sprintf(strcat('cutThetaDiff2_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.fig'),length(dir(fullfile(probPath,strcat('cutThetaDiff2_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.fig'))))+1)));
                                end
                                
                                figure(15)
                                if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                    probDir = probPath;
                                    clf;
                                    obj.twoOnlyP2List = []; % zeros(1,1 + 2*IterThetaNum);
                                end
%                                 obj.twoOnlyP2List = [obj.twoOnlyP2List; [rad2deg([angOptList0' angOptList' k2cRef] - [k2cRef0.*ones(1,1 + 2*IterThetaNum)])] ];
%                                 obj.twoOnlyP2List = [obj.twoOnlyP2List; [rad2deg([angOptList0' angOptList' angOptEpi k2cRef] - [k2cRef0.*ones(1,2 + 2*IterThetaNum)])] ];
                                obj.twoOnlyP2List = [obj.twoOnlyP2List; [rad2deg([angOptList0' angOptList' angOptEpiInit k2cRef obj.accumP2CTemp(end)] - [k2cRef0.*ones(1,3 + 2*IterThetaNum)])] ];
                                
                                figure(15);hold on;
                                plot(obj.twoOnlyP2List(:,end-1),'-sg');
                                colorCell0 = {};
                                colorCell0{1} = '-rx';
                                colorCell0{2} = '-mx';
                                
                                for uy = 1:IterThetaNum
                                    plot(obj.twoOnlyP2List(:,uy),colorCell0{uy});
                                end
                                
                                colorCell = {};
                                colorCell{1} = '-bo';
                                colorCell{2} = '-co';
                                
                                for uy = 1:IterThetaNum
                                    plot(obj.twoOnlyP2List(:,IterThetaNum+uy),colorCell{uy});
                                end
                                plot(obj.twoOnlyP2List(:,end-2),'-*k');
                                plot(obj.twoOnlyP2List(:,end),'-om');
                                legend('k2cRef - k2cRef0','1st theta from p2 and z','2nd theta', 'epipolar line', 'accum p2c','Location','southwest');title('ang - ref (deg)');
                                saveas(gcf,fullfile(probPath,sprintf(strcat('twoOnlyP2List_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('twoOnlyP2List_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                                
                                
                                gfdgbs = 1;
                                
                                
%                                 [thetaProbUse, thetaProbUse_, thetaProbUse__, curPredPtIcs, weightRaw, candX, candY,candX_reshape, candY_reshape, weightRaw_reshape, idff] = UpdateNewTracking(obj, thetaSamp, thetaProb, prevFeatPtList0, ptCcsZ0, intrMat, AngleModalOrg, DepthProbility, DepthId, inlierId,DispRefineUse__);
                                
                                
                                
                                
                                
                                
                                if 0  % 20191108  change update p2
                                    [thetaProbUse, thetaProbUse_, thetaProbUse__, curPredPtIcs, weightRaw, candX, candY,candX_reshape, candY_reshape, weightRaw_reshape, idff] = UpdateNewTracking(obj, thetaSamp, thetaProb, prevFeatPtList0, ptCcsZ0, intrMat, AngleModalOrg, DepthProbility, DepthId, inlierId,DispRefineUse__);
                                    if size(weightRaw_reshape, 1) > size(ProbReprojVecAll0,1)
                                        compMat = zeros((size(weightRaw_reshape,1) - size(ProbReprojVecAll0,1))/2,size(weightRaw_reshape,2),size(weightRaw_reshape,3));
                                        
                                        ProbReprojVecAll0 = [compMat; permute(ProbReprojVecAll0,[1 3 2]);compMat];
                                    else
                                        compMat = [];
                                    end
                                    
                                    if size(weightRaw_reshape, 1) < size(ProbReprojVecAll0,1)
                                        compMat = zeros((size(ProbReprojVecAll0,1) - size(weightRaw_reshape,1))/2,size(weightRaw_reshape,2),size(weightRaw_reshape,3));
                                        
                                        weightRaw_reshape = [compMat; weightRaw_reshape;compMat];
                                    else
                                        compMat = [];
                                    end
                                    ProbReprojVecAll_reshape = permute(ProbReprojVecAll0, [1 3 2]);
                                    %                                             weightRaw_reshape = permute(weightRaw_reshape, [1 3 2]);
                                    
                                    
                                    
                                    
                                    if UpdateP2_3
                                    ProbReprojVecAll = ProbReprojVecAll0.*permute(weightRaw_reshape,[1 3 2]);
                                    end
                                    
                                    
                                    [ExpCoord_Reproj, ExpCoord_OnlyP2, errReproj, errOnlyP2, errPt2dCur,interpValGT,interpValCur] =...
                                        CalcP2Exp(obj, candX, candY, weightRaw, idff,pt2dCur, pixGT, imgCur,candX_reshape,candY_reshape,weightRaw_reshape, ProbReprojVecAll_reshape, compMat,depthGTInd,k2cRef0,thetaRng);
                                    %                                            figure(1),clf;hist([errReproj errOnlyP2],100) ;legend('reproj exp','onlyP2 exp');
                                    %                                            figure(1),clf;hist([errReproj errOnlyP2],100) ;legend('old p2 exp','new p2 exp');
                                    %                                            figure(1),clf;hist([errPt2dCur errOnlyP2],100) ;legend('init NewTracking','new p2 exp');
                                    %                                            figure(1),clf;histogram(errPt2dCur,100);hold on;histogram(errOnlyP2, 100);legend('init NewTracking','new p2 exp');
%                                     figure(1),clf;hist([errPt2dCur errOnlyP2],100);legend('init NewTracking','new p2 exp');
                                    figure(1),clf;hist([interpValCur interpValGT],100);legend('old tracking prob','gt prob');
                                    %                                             figure(1),clf;hist([errP2_old errP2_new],100);legend('old','new')
                                    saveas(gcf,fullfile(probPath,sprintf(strcat('refinedTracking_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('refinedTracking_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
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
                                            thetaProbUse2_(thetaProbUse2 > obj.configParam.only_p2_prob_cutoff_threshold_2) = max(thetaProbUse2(thetaProbUse2 < obj.configParam.only_p2_prob_cutoff_threshold_2));
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
                                            qwa = 13;figure(13),clf;plot([ProbZ(qwa,:)./max(ProbZ(qwa,:)); updatedProbZSum0(qwa,:)./max(updatedProbZSum0(qwa,:)); updatedProbZSum(qwa,:)./max(updatedProbZSum(qwa,:))]');legend('orig Z','1st upadte','2nd update')
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
                            
                            
                            
                            
                            
                            
                            
                            
                            
                            
                            
                            
                            errZSum = sum(updatedProbZ(:,2,3)) - updatedProbZSum(2,3);
                            
                            if 1
                                figure(40),clf;subplot(2,1,1),plot(disparityRng, ProbZ'./max(ProbZ(:)));title('orig ProbZ');
                                subplot(2,1,2),plot(disparityRng, updatedProbZSum'./max(updatedProbZSum(:)));%title('updated ProbZ');
                                title(sprintf('updated ProbZ\nsigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f',obj.configParam.reproj_sigma,obj.configParam.reproj_beta,obj.configParam.reproj_sigma_right,obj.configParam.reproj_beta_right));
                                saveas(gcf,fullfile(probPath,sprintf(strcat('zProb_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('zProb_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                            end
                            
                            
                            
                            area1 = trapz(rad2deg(thetaSamp(1:idM)), thetaProb(1:idM));
                            area2 = trapz(rad2deg(thetaSamp(idM:end)), thetaProb(idM:end));
                            percentage = obj.configParam.theta_percentage;
                            
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
                            %                 figure(31),hold on;line(pt1(:,1), pt1(:,2),'Color',[0 0 1]);
                            figure(31),
                            hold on;plot(rad2deg(thetaSamp+ (k2cRef - k2cRef0)), thetaProb);plot(rad2deg(thetaSamp(idM)+ (k2cRef - k2cRef0)), thetaProb(idM),'*r'); 
                            hold on;plot(rad2deg(thetaSamp([ind1; ind2])+ (k2cRef - k2cRef0)), thetaProb([ind1; ind2]),'*b');
                             
                            plot(rad2deg(thetaSamp([idM_Vec00(1); idM_Vec00(end)])+ (k2cRef - k2cRef0)), thetaProb([idM_Vec00(1); idM_Vec00(end)]),'*g');
                            if ~FORCEEXPEVEN    
                                title(sprintf('p(theta) cutoff threshold: %0.5f\nmean theta ind: %0.2f / %0.2f\nk2cRef: %0.5f\nk2cPnP: %0.5f\nk2cProb: %0.5f\nsigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f',obj.configParam.theta_prob_cutoff_threshold,mean(idM_Vec00),(length(thetaSamp)+1)/2,rad2deg(k2cRef0), rad2deg(k2cPnp), rad2deg(thetaRng(idM)),obj.configParam.reproj_sigma,obj.configParam.reproj_beta,obj.configParam.reproj_sigma_right,obj.configParam.reproj_beta_right));grid on;
                            else
                                title(sprintf('p(theta) cutoff threshold: %0.5f\nmean theta ind: %0.2f / %0.2f\nk2cRef: %0.5f\nk2cPnP: %0.5f\nk2cProb: %0.5f\nsigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f',obj.configParam.theta_prob_cutoff_threshold,mean(idM_Vec00),(length(thetaSamp)+1)/2,rad2deg(k2cRef0), rad2deg(k2cPnp), rad2deg(angOpt),obj.configParam.reproj_sigma,obj.configParam.reproj_beta,obj.configParam.reproj_sigma_right,obj.configParam.reproj_beta_right));grid on;
                            end
                            saveas(gcf,fullfile(probPath,sprintf(strcat('prob_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('prob_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                            %                             title(sprintf('sigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f',obj.configParam.reproj_sigma,obj.configParam.reproj_beta,obj.configParam.reproj_sigma_right,obj.configParam.reproj_beta_right));
                            
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
                                figure(41);
                                if jkl == 1  %DEPTHITERNUM
                                    if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                        %                          rngDir = fullfile(pwd, 'rng');
                                        %                                     saveas(gcf,fullfile(probDir,sprintf('rng_%05d.png',length(dir(fullfile(probDir,'rng_*.png')))+1)));
                                        %                                     saveas(gcf,fullfile(probDir,sprintf('rng_%05d.fig',length(dir(fullfile(probDir,'rng_*.fig')))+1)));
                                        clf;
                                        %                                     obj.refAngList = [];
                                    end
                                end
                            end
                            if jkl == 1
                                if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                    obj.refAngList = [];
                                    obj.thetaPlatform = [];
                                    obj.thetaPlatformDistribution = [];
                                end
                            end
                            
                            if jkl == 1
                                obj.refAngList = [obj.refAngList; (k2cRef0)];
                                if doRoundingRefAngList3
                                    obj.refAngList3 = [obj.refAngList3; (k2cRef)];
                                else
                                    obj.refAngList3 = [obj.refAngList3; (k2cRef0)];
                                end
                                obj.thetaPlatform = [obj.thetaPlatform; mean(idM_Vec00)./((length(thetaSamp)+1)/2)];
                                obj.thetaPlatformDistribution = [obj.thetaPlatformDistribution; thetaPlatformDistribution_];
                            end
                            
                            if 0
                                figure(30),clf;subplot(1,2,1);plot(obj.thetaPlatform,'-xb');title('platform midpoint ratio');
                                subplot(1,2,2);plot(obj.thetaPlatformDistribution(:,1),'-xb');hold on;plot(obj.thetaPlatformDistribution(:,2),'-xr');title('ref theta distribution');legend('zero')
                                if 0 % size(obj.featPtManager.localTrace.ptIcsX, 2) <= 3
                                    saveas(gcf,fullfile(probPath,sprintf(strcat('platform_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('platform_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                                end
                            end
                            
                            ind = GetLastKeyFrameInd(obj);
                            %                     angRngList = [obj.angRng(ind-2+1:end,:);angRng];
                            %                     angRngList = [obj.angRng(end-length(obj.refAngList)+2:end,:);angRng];
                            vslAng = obj.poseWcsList(ind:end,3) - obj.poseWcsList(ind,3);
                            if length(obj.angOpt) >= length(obj.refAngList)
                                %                         optAng = obj.angOpt(end-length(obj.refAngList)+2:end,:) - obj.angOpt(end-length(obj.refAngList)+2,:);
                                baseId = length(obj.angOpt) - length(obj.refAngList);
                                optAng0 = obj.angOpt(baseId+1:end);
                                optAng = optAng0 - optAng0(1);
                                if 0
                                    optAngRng = obj.angRng(baseId+1:end,:);
                                    optAngRng = [optAngRng(:,1) - optAngRng(1,1)  optAngRng(:,2) - optAngRng(1,2)];
                                else
                                    %                             optAng0 = obj.angOpt(baseId+1:end);
                                    optAngRng = obj.angRng(baseId+1:end,:);
                                    optAngRng = [optAngRng(:,1) - optAng0(1,1)  optAngRng(:,2) - optAng0(1,1)];
                                end
                            else
                                optAng = obj.angOpt;
                                optAngRng = obj.angRng;
                            end
                            if 0
                                figure(41);hold on;%plot(rad2deg(obj.refAngList),'-xr');
                                % %                     plot(rad2deg(angRngList(:,1)) - rad2deg(obj.refAngList),'-xg');
                                %                     plot(rad2deg([vslAng(2:end);thetaRng(idM)]) - rad2deg(obj.refAngList),'-xk');
                                if length(obj.angOpt) >= length(obj.refAngList)
                                    plot(rad2deg([optAngRng(2:end,1); thetaRng(ind1)]) - rad2deg(obj.refAngList),'-xg');
                                    %                                 plot(rad2deg([optAng(2:end);thetaRng(idM)]) - rad2deg(obj.refAngList),'-xk');
                                    plot(rad2deg([optAng(2:end);angOpt]) - rad2deg(obj.refAngList),'-xk');
                                    plot(rad2deg([optAngRng(2:end,2); thetaRng(ind2)]) - rad2deg(obj.refAngList),'-xb');
                                else
                                    if ~isempty(optAngRng)
                                        plot(rad2deg([optAngRng(1:end,1); thetaRng(ind1)]) - rad2deg(obj.refAngList),'-xg');
                                        %                                     plot(rad2deg([optAng(1:end);thetaRng(idM)]) - rad2deg(obj.refAngList),'-xk');
                                        plot(rad2deg([optAng(1:end);angOpt]) - rad2deg(obj.refAngList),'-xk');
                                        plot(rad2deg([optAngRng(1:end,2); thetaRng(ind2)]) - rad2deg(obj.refAngList),'-xb');
                                    else
                                        
                                        plot(rad2deg([thetaRng(ind1)]) - rad2deg(obj.refAngList),'-xg');
                                        %                                     plot(rad2deg([optAng(1:end);thetaRng(idM)]) - rad2deg(obj.refAngList),'-xk');
                                        plot(rad2deg([optAng(1:end);angOpt]) - rad2deg(obj.refAngList),'-xk');
                                        plot(rad2deg([thetaRng(ind2)]) - rad2deg(obj.refAngList),'-xb');
                                    end
                                end
                                %                     plot(rad2deg(angRngList(:,2)) - rad2deg(obj.refAngList),'-xb');
                                legend('k2cUpper - k2cRef','k2cOpt - k2cRef','k2cLower - k2cRef');
                                title(sprintf('sigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f',obj.configParam.reproj_sigma,obj.configParam.reproj_beta,obj.configParam.reproj_sigma_right,obj.configParam.reproj_beta_right));
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
                                % % % % %                                 obj.featPtManager.localTrace.probZ(inlierId,:) = updatedProbZSum_;
                                % % % % %                                 %                 obj.keyProbZ = [obj.keyProbZ; [{sum(obj.keyFrameFlagList)} {inlierId} {ProbZ} {updatedProbZSum_} {2.^size(obj.featPtManager.localTrace.ptIcsX,2)}]];
                                % % % % %                                 obj.keyProbZ = [obj.keyProbZ; [{sum(obj.keyFrameFlagList)} {inlierId} {ProbZ} {updatedProbZSum_} {(max(ProbZ(:))/max(updatedProbZSum(:)))}]];
                                % % % % %                             end
                                % % % % %                             drawnow;
                                
                                drawnow;
                                
                                saveas(gcf,fullfile(probPath,sprintf(strcat('rng_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('rng_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                            end
                            updatedProbZSum_ = (max(ProbZ(:))/max(updatedProbZSum(:))).*updatedProbZSum;
                            
                            if 0 %% 20191115
                                updatedProbZSum_(isnan(updatedProbZSum_)) = 0;
                            else
                                updatedProbZSum_(isnan(updatedProbZSum_)) = 1;
                            end
                            
                            if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                NewDispStep__ = [];
                            else
                                NewDispStep__ = newDispStep(ismember(inlierIdPrv,inlierId));
                            end
                            curDispStepList = [];
                            updatedProbZSum_ = updatedProbZSum_./repmat(max(updatedProbZSum_')',1,size(updatedProbZSum_,2));
                              
                            [depthGTInd_update, ProbZTmpTmp_update_norm, ProbZTmp_update_norm, idMax_update, depthGTIndAll_update, depthUpdateIndAll_update, ProbZNormSum, NewDispRng_, newDispStep_, ProbZTmpReSamp_] = VisualLocalizer.GetDepthHist(obj,curDispStepList, updatedProbZSum_,disparityErrorRound,obj.configParam.disparity_sample_step, DispRng,NewDispStep__, depthC2KInd_ind);
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
                                figure(28),clf;plot(disparityRng, sum(ProbZTmp_update_norm_sum));hold on;plot(disparityRng,sum(ProbZTmp_update_norm_sum_tmp));legend('refined disp','gt disp');
                                saveas(gcf,fullfile(probPath,sprintf(strcat('depthDistri_',probDir(end-14:end),'____',sprintf('%04d',sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('depthDistri_',probDir(end-14:end),'____',sprintf('%04d',(sum(obj.keyFrameFlagList))),'__*.png'))))+1)));
                                else
                                    figure(28),clf;plot(dispStepRngTemp, sum(ProbZTmp_update_norm_sum));hold on;plot(dispStepRngTemp,sum(ProbZTmp_update_norm_sum_tmp));legend('refined disp','gt disp');
                                saveas(gcf,fullfile(probPath,sprintf(strcat('depthDistri_',probDir(end-14:end),'____',sprintf('%04d',sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('depthDistri_',probDir(end-14:end),'____',sprintf('%04d',(sum(obj.keyFrameFlagList))),'__*.png'))))+1)));
                                    
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
                            DispGT = dispGTTmp(inlierId); %DispRefine(goodOptId(1:nN));
                            DispInit = dispList(inlierId);
                            
                            if 0
                                figure(25);clf;plot(ProbZTmpTmp_update_norm(goodOptId(1:nN)));
                                saveas(gcf,fullfile(probPath,sprintf(strcat('peakHeight_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('peakHeight_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                                figure(24);clf;hist([DispRefine DispGT DispInit], 50);legend('refined disp','gt disp','stereo disp');
                                saveas(gcf,fullfile(probPath,sprintf(strcat('dispDistri_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('dispDistri_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                            end
                            
                            try
                                [minVal,idididMin] = sort(ProbZTmpTmp_update_norm,'ascend');
                                [maxVal,idididMax] = sort(ProbZTmpTmp_update_norm,'descend');
                                
                                checkId1111 = ([idididMin(1:15); idididMax(1:15)]);
                            
                                
                                qwa = checkId1111(1);figure(17),clf;subplot(2,1,1);plot(disparityRng, [ProbZ(qwa,:)./max(ProbZ(qwa,:)); updatedProbZSum0(qwa,:)./max(updatedProbZSum0(qwa,:)); updatedProbZSum(qwa,:)./max(updatedProbZSum(qwa,:))]');hold on;plot(disparityRng(depthGTInd_update(qwa,1)),updatedProbZSum(qwa,depthGTInd_update(qwa,1))./max(updatedProbZSum(qwa,:)),'*r');legend('orig Z','1st upadte','2nd update','gt');
                                qwa = checkId1111(end);             subplot(2,1,2);plot(disparityRng, [ProbZ(qwa,:)./max(ProbZ(qwa,:)); updatedProbZSum0(qwa,:)./max(updatedProbZSum0(qwa,:)); updatedProbZSum(qwa,:)./max(updatedProbZSum(qwa,:))]');hold on;plot(disparityRng(depthGTInd_update(qwa,1)),updatedProbZSum(qwa,depthGTInd_update(qwa,1))./max(updatedProbZSum(qwa,:)),'*r');legend('orig Z','1st upadte','2nd update','gt');
                                saveas(gcf,fullfile(probPath,sprintf(strcat('zUpdate_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('zUpdate_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                                
                                
                                
                                %                                 checkId = inlierId([idididMin(1:5); idididMax(1:5)]);
                                checkId = ([idididMin(1:15); idididMax(1:15)]);
                                ratioMat = BB.*repmat(thetaProb'./max(thetaProb), 1,size(BB0,2));
                                ratioVec = sum(ratioMat);
                                if 0
                                    figure(29),clf;subplot(1,2,1);plot(repmat(rad2deg(thetaSamp)',1,length(checkId)/2), BB0(:,checkId(1:length(checkId)/2)),'-g');hold on;plot(rad2deg(thetaSamp), thetaProb./max(1),'-b','LineWIdth',5);title('bad');
                                    subplot(1,2,2);plot(repmat(rad2deg(thetaSamp)',1,length(checkId)/2), BB0(:,checkId(length(checkId)/2+1:end)),'-r');hold on;plot(rad2deg(thetaSamp), thetaProb./max(1),'-b','LineWIdth',5);title('good');
                                end
                                
                                %                                 checkId = setdiff(find(ProbZTmpTmp_norm > obj.configParam.depth_hist_ratio), find(ProbZTmpTmp_update_norm > obj.configParam.depth_hist_ratio));
                                checkId1 = find(0.9.*ProbZTmpTmp_norm > ProbZTmpTmp_update_norm);
                                
                                
                                try
                                    %                                     checkId = checkId(1:10);
                                    checkId1 = checkId1(1:2*floor(length(checkId1)/2));
                                    figure(38);subplot(1,3,1);hold on;plot(checkId1(1:length(checkId1)/2), dispList(inlierId(checkId1(1:length(checkId1)/2))) - dispGTTmp(inlierId(checkId1(1:length(checkId1)/2))),'og');plot(checkId1(length(checkId1)/2+1:end), dispList(inlierId(checkId1(length(checkId1)/2+1:end))) - dispGTTmp(inlierId(checkId1(length(checkId1)/2+1:end))),'or');legend('','bad','good')
                                    subplot(1,3,2);plot(pt2dCur(checkId1(1:length(checkId1)/2),1),pt2dCur(checkId1(1:length(checkId1)/2),2),'og');plot(pt2dCur(checkId1(length(checkId1)/2+1:end),1),pt2dCur(checkId1(length(checkId1)/2+1:end),2),'or');
                                    subplot(1,3,3);plot(dispErrCur);title('stereoDispCur - gtDispCur');
                                catch
                                    sdafb = 1;
                                end
                                figure(38);
                                saveas(gcf,fullfile(probPath,sprintf(strcat('disp_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('disp_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                                
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
                        
                        %                         figure(77),clf,subplot(1,2,1);plot(newZArea./oldZArea);title('oldFeatArea / newFeatArea');subplot(1,2,2);plot(maxFeatZ1_update./maxFeatZ1);title('oldMaxFeat / newMaxFeat');
                        %                         figure(77),clf,subplot(1,2,1);plot([newZArea./oldZArea maxFeatZ1_update'./maxFeatZ1']);legend('area','peak','Location','southwest');title('oldFeatArea / newFeatArea');subplot(1,2,2);plot(maxFeatZ1_update./maxFeatZ1);title('oldMaxFeat / newMaxFeat');
                        %                         figure(77),clf,subplot(1,2,1);plot([areaInfo]);legend('area','peak','Location','southwest');title('oldFeatArea / newFeatArea');subplot(1,2,2);plot(areaInfo(:,2));title('oldMaxFeat / newMaxFeat');
                        
                        [~,id123] = sort(areaInfo(:,2),'ascend');
                        if 0
                            figure(77),clf,subplot(3,2,1);hist([areaInfo],50);legend('area','peak','Location','southwest');title('newFeatArea / oldFeatArea');subplot(3,2,2);hist(areaInfo(:,2),50);title('newMaxFeat / oldMaxFeat');
                            subplot(3,2,3);plot(trackingError(id123,end));title('norm(trackingXY - gtXY)'); subplot(3,2,4);plot(abs(pt2dCur(id123,1) -pixGT(id123,1)));title('norm(trackingX - gtX)');
                            subplot(3,2,5);plot(abs(dispList(inlierId(id123)) - dispGTTmp(inlierId(id123))));title('norm(stereoDisp - gtDisp)');subplot(3,2,6);plot(abs(pt2dCur(id123,2) -pixGT(id123,2)));title('norm(trackingY - gtY)');

                            saveas(gcf,fullfile(probPath,sprintf(strcat('areaDrop_',probDir(end-14:end),'____',sprintf('%04d',sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('areaDrop_',probDir(end-14:end),'____',sprintf('%04d',(sum(obj.keyFrameFlagList))),'__*.png'))))+1)));
                        end
                        
                        if UPDATEDEPTH
                            %                                 updatedProbZSum_ = (max(ProbZ(:))/max(updatedProbZSum(:))).*updatedProbZSum;
                            
                            if size(obj.featPtManager.localTrace.ptIcsX, 2) <= 2
                                outThr = 0.5;
                            else
                                outThr = 0.45;
                            end
                            outThr = -1;
                            if 1  %size(obj.featPtManager.localTrace.ptIcsX, 2) <= 3
                                
                                updatedProbZSum_ = updatedProbZSum_./max(updatedProbZSum_(:));
                                updatedProbZSum_(platformRatio < outThr,:) = min(min(updatedProbZSum_(:)),0.000001);
                                inFlag = inlierId(find(platformRatio >= 0.7)');
                            else
                                inFlag = inlierId;  [1:length(platformRatio)]';
                            end
                            if 0
                                obj.featPtManager.localTrace.probZ(inlierId,:) = updatedProbZSum_;
                                %                 obj.keyProbZ = [obj.keyProbZ; [{sum(obj.keyFrameFlagList)} {inlierId} {ProbZ} {updatedProbZSum_} {2.^size(obj.featPtManager.localTrace.ptIcsX,2)}]];
                                obj.keyProbZ = [obj.keyProbZ; [{sum(obj.keyFrameFlagList)} {inlierId} {ProbZ./max(ProbZ(:))} {updatedProbZSum_./max(updatedProbZSum_(:))} {(max(ProbZ(:))/max(updatedProbZSum(:)))} {pt2dCur} {pixGT} {trackingError} {Pix} {zList} {dispList} {dispGTTmp} {dispCurList} {pt2dCurR} {dispMapCurGTList} {pixGTR} {depthListGT} {XYZ} {k2cRef0} {k2cPnp} {[ProbZTmpTmp_norm ProbZTmpTmp_update_norm]} {thetaPlatformDistribution_} {areaInfo} {ProbZTmp_update_norm} {DispRefineUse_} {NewDispRng} {newDispStep} {ProbZTmpReSamp}]];
                            elseif 0
                                obj.featPtManager.localTrace.probZ(inlierId,:) = ProbZTmpReSamp_./max(ProbZTmpReSamp_(:));
                                %                 obj.keyProbZ = [obj.keyProbZ; [{sum(obj.keyFrameFlagList)} {inlierId} {ProbZ} {updatedProbZSum_} {2.^size(obj.featPtManager.localTrace.ptIcsX,2)}]];
                                obj.keyProbZ = [obj.keyProbZ; [{sum(obj.keyFrameFlagList)} {inlierId} {ProbZ./max(ProbZ(:))} {ProbZTmpReSamp_./max(ProbZTmpReSamp_(:))} {(max(ProbZ(:))/max(updatedProbZSum(:)))} {pt2dCur} {pixGT} {trackingError} {Pix} {zList} {dispList} {dispGTTmp} {dispCurList} {pt2dCurR} {dispMapCurGTList} {pixGTR} {depthListGT} {XYZ} {k2cRef0} {k2cPnp} {[ProbZTmpTmp_norm ProbZTmpTmp_update_norm]} {thetaPlatformDistribution_} {areaInfo} {ProbZTmp_update_norm} {DispRefineUse_} {NewDispRng_} {newDispStep_} {ProbZTmpReSamp_}]];
                            else
                                idGoodInlier = find(ProbZTmpReSamp_(depthC2KInd_ind) > -0.995);
                                inlierIdNew = inlierId(idGoodInlier,:);
                                
                                
                                % % % % %                                    if size(obj.featPtManager.localTrace.ptIcsX, 2) > 3
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
                                obj.featPtManager.localTrace.probZ(inlierIdNew,:) = ProbZTmpReSamp_(idGoodInlier,:); % ProbZTmpReSamp_./max(ProbZTmpReSamp_(:));
                                %                 obj.keyProbZ = [obj.keyProbZ; [{sum(obj.keyFrameFlagList)} {inlierId} {ProbZ} {updatedProbZSum_} {2.^size(obj.featPtManager.localTrace.ptIcsX,2)}]];
                                obj.keyProbZ = [obj.keyProbZ; [{sum(obj.keyFrameFlagList)} {inlierIdNew} {ProbZ(idGoodInlier,:)} {ProbZTmpReSamp_(idGoodInlier,:)} {(max(ProbZ(:))/max(updatedProbZSum(:)))} {pt2dCur(idGoodInlier,:)} {pixGT(idGoodInlier,:)} {trackingError(idGoodInlier,:)} {Pix(idGoodInlier,:)} {zList} {dispList} {dispGTTmp} {dispCurList(idGoodInlier,:)} {pt2dCurR(idGoodInlier,:)} {dispMapCurGTList(idGoodInlier,:)} {pixGTR(idGoodInlier,:)} {depthListGT} {XYZ(:,idGoodInlier)} {k2cRef0} {k2cPnp} {[ProbZTmpTmp_norm(idGoodInlier,:) ProbZTmpTmp_update_norm(idGoodInlier,:)]} {thetaPlatformDistribution_} {areaInfo(idGoodInlier,:)} {ProbZTmp_update_norm(idGoodInlier,:)} {DispRefineUse_(idGoodInlier,:)} {NewDispRng_(idGoodInlier,:)} {newDispStep_(idGoodInlier,:)} {ProbZTmpReSamp_(idGoodInlier,:)}]];
                            end
                            if 0
                                obj.featPtManager.localTrace.ptIcsX(inlierId,end) = ExpCoord_OnlyP2(:,1);
                                obj.featPtManager.localTrace.ptIcsY(inlierId,end) = ExpCoord_OnlyP2(:,2);
                            elseif 0
                                obj.featPtManager.localTrace.ptIcsX(inlierId,end) = pixGT(:,1);
                                obj.featPtManager.localTrace.ptIcsY(inlierId,end) = pixGT(:,2);
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
                            [ac,dc] = probDensity(0, 8 ,obj.configParam.disparity_beta,obj.configParam.reproj_beta, [-10:0.01:10],obj.configParam.reproj_sample_interval, 'reproj');
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
    end
    
    
    methods (Static)
        function [depthGTInd, ProbZTmpTmpNormUse, ProbZTmpNorm, peakIdTmp, depthGTIndAll, depthUpdateIndAllUse, ProbZNormSum, NewDispRng, newDispStep, ProbZTmpReSamp] = GetDepthHist0(obj, curDispStepList,ProbZ,disparityErrorRound,disparity_sample_step,DispRng,NewDispStep_,depthC2KInd_ind)
            
            
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
            
            
            highProbRegion = ProbZTmpNorm >= obj.configParam.disparity_resample_prob_threshold | isnan(ProbZTmpNorm);
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
            if obj.configParam.disparity_resample_prob_threshold > 0
                ProbZTmpReSamp = [];
                for tp = 1 : size(DispRng,1)
                    %                 ProbZTmpNormReSamp(tp,:) = interp1(DispRng(tp,:), ProbZTmpNorm(tp,:), NewDispRng(tp,:));
                    ProbZTmpReSamp(tp,:) = interp1(DispRng(tp,:), ProbZ(tp,:), NewDispRng(tp,:));
                end
            else
                
                ProbZTmpReSamp = ProbZ;
                NewDispRng = DispRng;
                newDispStep = obj.configParam.disparity_sample_step.*ones(size(DispRng,1),1);
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
            
            
            if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
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
        
        
        
    end
    
    methods(Static)
        
        function [pixGT, pixGTR] = GetGtTrace0(obj, k2cRef0,inlierId,dispList,disparityErrorRound,intrMat,princpPtL,princpPtR,dispMapCurGTList)
            b2cPmat = GetPctBody2Cam(obj.coordSysAligner, 1);
            if 0
                k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(thetaRng(idM)),zeros(3,1));
            else
                k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cRef0),zeros(3,1));
            end
            k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
            
            Pix = [obj.featPtManager.localTrace.ptIcsX(inlierId,1) obj.featPtManager.localTrace.ptIcsY(inlierId,1)];
            
            metricPrevPtCcsGT = intrMat\HomoCoord(Pix',1);
            metricPrevPtCcsGT = normc(metricPrevPtCcsGT);
            if 0
                scaleAllGT = depthListGT(inlierId)./metricPrevPtCcsGT(3,:)';
            else
                dispGTComp = dispList(inlierId) - disparityErrorRound;
                depthListGTComp = intrMat(1,1).*norm(obj.camModel.transVec1To2)./(dispGTComp + (princpPtR(1) - princpPtL(1)));
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
        
        
        
    end
    
    
    methods(Static)
        
        function [pixGT, transXYZ] = GetGtTrace2(b2cPmat, k2cRef0, Pix,depthListGT,intrMat)
%             b2cPmat = GetPctBody2Cam(obj.coordSysAligner, 1);
            if 0
                k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(thetaRng(idM)),zeros(3,1));
            else
                k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cRef0),zeros(3,1));
            end
            k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
            
% %             Pix = [obj.featPtManager.localTrace.ptIcsX(inlierId,1) obj.featPtManager.localTrace.ptIcsY(inlierId,1)];
            
            metricPrevPtCcsGT = intrMat\HomoCoord(Pix',1);
            metricPrevPtCcsGT = normc(metricPrevPtCcsGT);
            if 1
%                 scaleAllGT = depthListGT(inlierId)./metricPrevPtCcsGT(3,:)';
                scaleAllGT = depthListGT./metricPrevPtCcsGT(3,:)';
            else
                dispGTComp = dispList(inlierId) - disparityErrorRound;
                depthListGTComp = intrMat(1,1).*norm(obj.camModel.transVec1To2)./(dispGTComp + (princpPtR(1) - princpPtL(1)));
                scaleAllGT = depthListGTComp./metricPrevPtCcsGT(3,:)';
            end
            XYZ = [repmat(scaleAllGT',3,1).*metricPrevPtCcsGT];
            
            
            k2cT = -b2cPmat.transformMat(1:3,1:3)*k2cBodyPmat.transformMat(1:3,1:3)*b2cPmat.transformMat(1:3,1:3)'*b2cPmat.transformMat(1:3,4) + b2cPmat.transformMat(1:3,4);
            k2cCam = [k2cCamPmat.transformMat(1:3,1:3) k2cT;0 0 0 1];
            homocurrCcsXYZ = k2cCam*HomoCoord(XYZ,1); %pix0_GT_ = pflat(intrMat*homocurrCcsXYZALL_(1:3,:))';
            pixGT = pflat(intrMat*homocurrCcsXYZ(1:3,:))';
            pixGT = pixGT(:,1:2);
            
            transXYZ = homocurrCcsXYZ(1:3,:)';
%             pixGTR = [pixGT(:,1) - dispMapCurGTList pixGT(:,2)];
%             
%             pixGT__ = pixGT;
%             pixGTR__ = pixGTR;
        end
        
        
        
    end
    
    
    methods (Static)
        function [C, dist] = fitline(XY)
            
            [rows,npts] = size(XY);
            
            if npts < 2
                error('Too few points to fit line');
            end
            
            if rows ==2    % Add homogeneous scale coordinate of 1
                XY = [XY; ones(1,npts)];
            end
            
            if npts == 2    % Pad XY with a third column of zeros
                XY = [XY zeros(3,1)];
            end
            
            % Normalise points so that centroid is at origin and mean distance from
            % origin is sqrt(2).  This conditions the equations.
            [XYn, T] = VisualLocalizer.normalise2dpts(XY);
            
            % Set up constraint equations of the form  XYn'*C = 0,
            % where C is a column vector of the line coefficients
            % in the form   c(1)*X + c(2)*Y + c(3) = 0.
            
            [u d v] = svd(XYn',0);   % Singular value decomposition.
            C = v(:,3);              % Solution is last column of v.
            
            % Denormalise the solution
            C = T'*C;
            
            % Rescale coefficients so that line equation corresponds to
            %   sin(theta)*X + (-cos(theta))*Y + rho = 0
            % so that the perpendicular distance from any point (x,y) to the line
            % to be simply calculated as
            %   r = abs(c(1)*X + c(2)*Y + c(3))
            C = C / sqrt(C(1)^2 + C(2)^2);
            
            % If requested, calculate the distances from the fitted line to
            % the supplied data points
            if nargout==2
                dist = abs(C(1)*XY(1,:) + C(2)*XY(2,:) + C(3));
            end
        end
    end
    methods (Static)
        function [newpts, T] = normalise2dpts(pts)
            
            if size(pts,1) ~= 3
                error('pts must be 3xN');
            end
            
            % Find the indices of the points that are not at infinity
            finiteind = find(abs(pts(3,:)) > eps);
            
            if length(finiteind) ~= size(pts,2)
                warning('Some points are at infinity');
            end
            
            % For the finite points ensure homogeneous coords have scale of 1
            pts(1,finiteind) = pts(1,finiteind)./pts(3,finiteind);
            pts(2,finiteind) = pts(2,finiteind)./pts(3,finiteind);
            pts(3,finiteind) = 1;
            
            c = mean(pts(1:2,finiteind)')';            % Centroid of finite points
            newp(1,finiteind) = pts(1,finiteind)-c(1); % Shift origin to centroid.
            newp(2,finiteind) = pts(2,finiteind)-c(2);%   °ÑËùÓÐµãµÄx¡¢y×ø±ê¼õµôx¡¢yµÄÆ½¾ùÖµ
            
            dist = sqrt(newp(1,finiteind).^2 + newp(2,finiteind).^2);
            meandist = mean(dist(:));  % Ensure dist is a column vector for Octave 3.0.1Çó¸÷¸öÐÂµãµ½ÐÂÔ­µãµÄ¾àÀë
            
            scale = sqrt(2)/meandist;%ÇóÆ½¾ù¾àÀëºÍ¸ùºÅ2Ö®¼äµÄ±ÈÀý³ß¶È
            
            T = [scale   0   -scale*c(1)
                0     scale -scale*c(2)
                0       0      1      ];%¹¹½¨Ò»¸ö×ªÒÆ¾ØÕó
            
            newpts = T*pts;%%µÃµ½ÐÂµÄµã
        end
    end
    
    methods (Static)
        function angleModal= UpdateModalAngleOnlyP2NewVersion2(angleIntervel,angleResult,existPt1,existPt2,ambiguifyRange)
            
            
            badRange = 1;
            goodIdx1 = find(abs(existPt1.x - 160)>badRange & abs(existPt2.x - 160)>badRange );
            goodIdx2 = find(abs(existPt1.y - 120)>badRange & abs(existPt2.y - 120)>badRange );%  find(existPt1.y ~= 120 & existPt2.y ~= 120);
            angleResult = angleResult(intersect(goodIdx1,goodIdx2));
            
            
            %figure(n*1000+frameIdx);
            
            % hold on
            if 0
                figure(1111),clf; h = histogram(angleResult,[-179 - angleIntervel/2 : angleIntervel : 179 + angleIntervel/2]);
                tmp = h.Values;
            else
                hh = hist(angleResult,[-179 - angleIntervel/2 : angleIntervel : 179 + angleIntervel/2]);
                tmp = hh(1:end-1);
            end
            angleBin = 179/angleIntervel*2+1;
            
            % ambiguifyRange = 1.0./angleIntervel;
            tmp2 = zeros(1,angleBin);
            for angleIdx = 1:angleBin
                for anbiIdx = -ambiguifyRange:1:ambiguifyRange
                    
                    if angleIdx+anbiIdx < 1 || angleIdx+anbiIdx > angleBin
                        continue;
                    end
                    
                    tmp2(angleIdx+anbiIdx) = tmp2(angleIdx+anbiIdx) + tmp(angleIdx);
                    
                end
            end
            
            
            
            %angleModal = UpdateModalAngleOnlyP2(m,s2,angleResult,existPt1,existPt2);
            angleModal =tmp2./sum(tmp2);% angleModal;
            %                 angleModal = obj.angleModal./sum(obj.angleModal);
            
            
            
        end
    end
    
    methods (Static)
        
        function  [thetaProbUse, angOpt, thetaExp_tmp,thetaRngP2C_use] = New_OnlyP2_new_old0(obj, pt2dPrv, pt2dCur, intrMat,thetaP2C,T_B2C, k2cRef, tx, ty, tz, thetaRng,DispRng, inlierId)
            
            
            
            s = Setting([]);
            s.configParam.disparityBoundary = obj.configParam.disparity_error;
            s.configParam.disparityIntervel = obj.configParam.disparity_sample_step;
            s.configParam.disparityRange = -obj.configParam.disparity_error : obj.configParam.disparity_sample_step : obj.configParam.disparity_error;
            s.configParam.disparityBin = length(s.configParam.disparityRange);
            
            s.configParam.disparityOrginRange = DispRng;
            s.configParam.angleIntervel = rad2deg(obj.configParam.theta_sample_step);
            s.configParam.angleBoundary = rad2deg(obj.configParam.theta_range(2));
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
        
        
        
        
    end
    
    
    
    
    
    
    
    
    
    methods(Static)
        
        function [thetaProbUse, angOpt, thetaExp_tmp,thetaRngP2C_use] = New_OnlyP2_new0(obj, pt2dPrv, pt2dCur, intrMat,thetaP2C,T_B2C, k2cRef, tx, ty, tz, thetaRng)
            global Rati_onlyP2
            areaThreshold = 1-Rati_onlyP2;
            radius = 3;
            existPt1 = Points.Point(pt2dPrv(:,1),pt2dPrv(:,2),intrMat);
            existPt2 = Points.Point(pt2dCur(:,1),pt2dCur(:,2),intrMat);
            thetaRngP2C = [deg2rad(-179) : obj.configParam.theta_sample_step : deg2rad(179)];  % + (thetaP2C);
            % thetaSampP2C_ = thetaRngP2C_ - thetaP2C;
            
            
            
            angleIntervel = rad2deg(obj.configParam.theta_sample_step);
            
            
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

    end

    
    methods (Static)
        function [angle_result,wrongIdx ] = AnglePredict(p1,p2,tx, ty, tz)
            %global
            x = p1(1,:);
            y = p1(2,:);
            
            x_ = p2(1,:);
            y_ = p2(2,:);
            
            %-----------%
            A1 = (x-x_) ;%C
            A2 = (ones(1,size(A1,2))+x.*x_);%S
            A3 = 0;
            A4 = y_.*(-tz);%C
            A5 = y_.*tx;
            A6 = y_.*tz;
            
            B1 = (-y_);%C
            B2 = x.*y_;
            B3 = y;
            B4 = (-x_.*tz + tx);%C
            B5 = (x_.*tx+tz);
            B6 = (x_.*tz -tx);
            
            A = (A1.*A4-B1.*B4) ;
            C = (A2.*A5-B2.*B5);
            F = (A3.*A6-B3.*B6);
            D = (A3.*A5-B3.*B5+A2.*A6-B2.*B6);%S
            
            B = (A2.*A4-B2.*B4+A1.*A5-B1.*B5);% 0
            E = (A3.*A4-B3.*B4+A1.*A6-B1.*B6);
            % A*sol_c1^2+C*sol_s1^2 + B*sol_c1*sol_s1 + D*sol_s1 + E*sol_c1+F
            % D*sol_s1+E*sol_c1+(F+A) = 0
            
            a = (E.^2+D.^2);
            b = 2.*(F+A).*D;
            c = (F+A).^2-E.^2;
            delta = b.^2-4.*a.*c;
            
            modifyIdx = find(delta<0);
            
            if ~isempty(modifyIdx)
                delta(modifyIdx)
            end
            delta(modifyIdx) = 0;
            % sol_s0 = (-b+sqrt(delta))/(2*a)
            %     sol_s1 = (-b-sqrt(delta))./(2.*a);
            % s    sol_s0 = (-b+sqrt(delta))./(2.*a);
            %
            sol_1 = -b./a;
            sol_s0 = 0;
            
            if sol_1>1
                sol_1 = 1;
            end
            if sol_1<-1
                sol_1 = -1;
            end
            angle_s1 = asind(sol_1);
            %angle_c1 = acosd(sol_1);
            
            
            angle_result = angle_s1;
            %     angle_c0 = asind(sol_s0);
            %     if length(find(abs(anangle_c1gle_c1) < 0.001)) >  length(find(abs(angle_c0) < 0.001))
            %         angle_result = angle_c0;
            %     else
            %         angle_result = angle_c1;
            %     end
            %     angleResultDiff = angle_result - angleOrgin;
            %     wrongIdx = find(abs(angleResultDiff) > 90);
            %     if ~isempty(wrongIdx)
            %        % wrongIdx
            %     end
            wrongIdx=[];
            
        end
    end
    
    
    methods(Static)
        function [angle_result ] = AnglePredict_old(p1,p2,tx, ty, tz)
            %global
            x = p1(1,:);
            y = p1(2,:);
            
            x_ = p2(1,:);
            y_ = p2(2,:);
            
            %-----------%
            A1 = (x-x_) ;%C
            A2 = (ones(1,size(A1,2))+x.*x_);%S
            A3 = 0;
            A4 = y_.*(-tz);%C
            A5 = y_.*tx;
            A6 = y_.*tz;
            
            B1 = (-y_);%C
            B2 = x.*y_;
            B3 = y;
            B4 = (-x_.*tz + tx);%C
            B5 = (x_.*tx+tz);
            B6 = (x_.*tz -tx);
            
            A = (A1.*A4-B1.*B4) ;
            C = (A2.*A5-B2.*B5);
            F = (A3.*A6-B3.*B6);
            D = (A3.*A5-B3.*B5+A2.*A6-B2.*B6);%S
            
            B = (A2.*A4-B2.*B4+A1.*A5-B1.*B5);% 0
            E = (A3.*A4-B3.*B4+A1.*A6-B1.*B6);
            % A*sol_c1^2+C*sol_s1^2 + B*sol_c1*sol_s1 + D*sol_s1 + E*sol_c1+F
            % D*sol_s1+E*sol_c1+(F+A) = 0
            
            a = (E.^2+D.^2);
            b = 2.*(F+A).*D;
            c = (F+A).^2-E.^2;
            delta = b.^2-4.*a.*c;
            
            modifyIdx = find(delta<0);
            delta(modifyIdx) = 0;
            
            % sol_s0 = (-b+sqrt(delta))/(2*a)
            sol_s1 = (-b-sqrt(delta))./(2.*a);
            sol_s0 = (-b+sqrt(delta))./(2.*a);
            
            angle_c1 = asind(sol_s1);
            angle_c0 = asind(sol_s0);
            if length(find(abs(angle_c1) < 0.001)) >  length(find(abs(angle_c0) < 0.001))
                angle_result = angle_c0;
            else
                angle_result = angle_c1;
            end
        end
        
    end
    
    
    
    methods (Static)
        
        function [angle_result,depth] = AnglePredict_Prob(p1,p2,r_cam,tx, ty, tz)
            %global
            x = p1(1,:);
            y = p1(2,:);
            
            x_ = p2(1,:);
            y_ = p2(2,:);
            
            
            alpha1 = 2.*(-y.*tx).* (-y.*tz);
            alpha2 = 2.*[(-y.*tx).* (-x.*tz+tx) + (tz+x.*tx).*(-y.*tz)];
            alpha3 = 2.*(tz+x.*tx).* (-x.*tz+tx);
            alpha4 = 2.*[(-y.*tx).*(y.*tx)+(-y.*tz).*(-y.*tz)];
            alpha5 = 2.*[(tz+x.*tx).*(y.*tx) +  (-x.*tz+tx).* (-y.*tz)];
            alpha6 = 2.*(y.*tx).* (-y.*tz);
            
            b = alpha1.*(x_.^2)+alpha2.*(x_).*(y_)+alpha3.*(y_.^2)+alpha4.*(x_)+alpha5.*(y_)+alpha6;
            
            beta1 = (-y.*tx).^2 + (y.*tz).^2;
            beta2 = 2.*[(-y.*tx).* (tz+x.*tx)+ (x.*tz-tx) .*(y.*tz)];
            beta3 = (tz+x.*tx).^2 + (x.*tz-tx).^2;
            beta4 = 0;%2.*[(-y.*tx).*(-y.*tx)+(-y.*tz).*(y.*tz)];
            beta5 = 2.*[(tz+x.*tx).*(-y.*tz) +  (x.*tz-tx).* (-y.*tx)];
            beta6 = (y.*tx).^2 + (y.*tz).^2;
            
            a = beta1.*(x_.^2)+beta2.*(x_).*(y_)+beta3.*(y_.^2)+beta4.*(x_)+beta5.*(y_)+beta6;
            
            sinTheta = -b./a;
            modifyIdx = find(sinTheta>1);
            sinTheta(modifyIdx) = 1.0;
            modifyIdx = find(sinTheta<-1);
            sinTheta(modifyIdx) = -1.0;
            angle_s1 = asind(sinTheta);
            angle_result = angle_s1';
            cosTheta = sqrt(ones(size(sinTheta))-(sinTheta).^2);
            
            
            depth = (r_cam.*[x_.*(-tz.*cosTheta+tx.*sinTheta+tz) - (-tz.*sinTheta-tx.*cosTheta+tx)] ./ (x.*cosTheta+sinTheta-x_.*(-x.*sinTheta+cosTheta)))';%-x.*sinTheta + cosTheta + r_cam.*(-tz.*cosTheta+tx.*sinTheta+tz);
            
            
        end
        
        
        
        
        
        
        
    end
    
    
    
    
    
    
    
    
    
    
    
    methods (Static)
        function featPtList = DetectFast(img, varargin)
            
            
            if (nargin <= 1)
                thr = [];
            elseif (nargin == 2)
                thr = varargin{1};
            else
                error('Too many input arguments');
            end
            
            
            
            
            [a,b,c] = size(img);
            if c > 1
                img = rgb2gray(img);
            end
            if a < 300
                if isempty(thr)
                    ptIcs = detectFASTFeatures(img,'MinQuality',0.01,'MinContrast',0.01);
                else
                    ptIcs = detectFASTFeatures(img,'MinQuality',thr,'MinContrast',thr);
                end
                % % %                 ptIcs = detectFASTFeatures(img,'MinQuality',0.02,'MinContrast',0.02);
            else
                % % % % % % %                 ptIcs = detectFASTFeatures(img,'MinQuality',0.04,'MinContrast',0.04);
                %                 ptIcs = detectFASTFeatures(img,'MinQuality',0.01,'MinContrast',0.01);
                if isempty(thr)
%                     ptIcs = detectFASTFeatures(img,'MinQuality',0.001,'MinContrast',0.001);
                    ptIcs = detectFASTFeatures(img,'MinQuality',0.04,'MinContrast',0.04);
                else
                    ptIcs = detectFASTFeatures(img,'MinQuality',thr,'MinContrast',thr);
                end
                %                 ptIcs = detectFASTFeatures(img,'MinQuality',0.02,'MinContrast',0.02);
            end
%             ptIcs = detectFASTFeatures(img,'MinQuality',0.01,'MinContrast',0.01);
%             ptIcs = detectFASTFeatures(img,'MinQuality',0.04,'MinContrast',0.04);
            %                     ptIcs = detectFASTFeatures(img,'MinQuality',0.01,'MinContrast',0.01);
            featPtList = ptIcs.Location;
            imgSize = size(img);
            if imgSize(1) == 240
                inId = find(featPtList(:,1) > 10 & featPtList(:,2) > 10 & featPtList(:,1) < imgSize(2) - 10 & featPtList(:,2) < 128);
            else
%                 inId = find(featPtList(:,1) > 10 & featPtList(:,2) > 10 & featPtList(:,1) < imgSize(2) - 10 & featPtList(:,2) < imgSize(1)/2 + 0 );
                inId = find(featPtList(:,1) > 10 & featPtList(:,2) > 10 & featPtList(:,1) < imgSize(2) - 10 & featPtList(:,2) < imgSize(1)/1 + 0 );
            end
            featPtList = featPtList(inId,:);
        end
    
    end
    
    methods (Static)
        function [k2cBodyRotAng,Err, inlierId, reProjErr] = RotationAngleEstimate_PNP_(pnp_ang_est_max_margin,intrMat,keyPtIcs,curPtIcs,zList, ind,inTrackFlag,b2c,k2cRef, varargin)
            % [keyPtIcs, curPtIcs, keyPtCcsZ] = GetFeaturePairs(featPtManager, 'key', 'last');
            %             keyPtIcs = [featX(inTrackFlag,1) featY(inTrackFlag,1)];
            %             curPtIcs = [featX(inTrackFlag,ind) featY(inTrackFlag,ind)];
            
            global rot_y
            
            if (nargin <= 9)
                minAngGrid = 0.0001;
            elseif (nargin == 10)
                minAngGrid = varargin{1};
            else
                error('Too many input arguments');
            end
            
            
            
            
            
            keyPtIcs = keyPtIcs(inTrackFlag,:);
            curPtIcs = curPtIcs(inTrackFlag,:);
            keyPtCcsZ = zList(inTrackFlag);
            
            k2cRef = double(k2cRef);
            
            
            
            
            activeFlag = keyPtCcsZ > 0;
            metricPrevPtCcs = intrMat\HomoCoord(keyPtIcs',1);
            metricPrevPtCcs = normc(metricPrevPtCcs);
            scale = keyPtCcsZ./metricPrevPtCcs(3,:)';
            keyCcsXYZ = scale.*metricPrevPtCcs';
            b2cPmat = b2c; %GetPctBody2Cam(coordSysAligner, 1);
            maxk2cRotAng = [k2cRef k2cRef] + [pnp_ang_est_max_margin(1:2)];
            k2cBodyRotAngCand = maxk2cRotAng(1):minAngGrid:maxk2cRotAng(2);
            numRotAng = length(k2cBodyRotAngCand);
            reProjErr = zeros(numRotAng,1);
            
            for i = 1:numRotAng
                k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cBodyRotAngCand(i)),zeros(3,1));
                if rot_y
                    k2cBodyPmat = [roty(rad2deg(k2cBodyRotAngCand(i))) [0;0;0];0 0 0 1];
                else
                    k2cBodyPmat = [rotx(rad2deg(k2cBodyRotAngCand(i))) [0;0;0];0 0 0 1];
                end
                k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
                if 1
                    k2cT = -b2cPmat(1:3,1:3)*k2cBodyPmat(1:3,1:3)*b2cPmat(1:3,1:3)'*b2cPmat(1:3,4) + b2cPmat(1:3,4);
                    k2cCam = [k2cCamPmat(1:3,1:3) k2cT;0 0 0 1];
                    homocurrCcsXYZ = k2cCam*HomoCoord(keyCcsXYZ',1);
                else
                    %                     k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
                    homocurrCcsXYZ = k2cCamPmat.transformMat*HomoCoord(keyCcsXYZ',1);
                end
                currCcsXYZ = homocurrCcsXYZ(1:3,:);
                [reProjErr(i),~,~,err(:,i)] = VisualLocalizer.CalculateReprojectError_(intrMat,currCcsXYZ',curPtIcs,[],[],activeFlag,0);
            end
            [minDis,minInd] = min(reProjErr);
            k2cBodyRotAng = k2cBodyRotAngCand(minInd);
            Err = err(:,minInd);
            inlierId_ = find(Err < pnp_ang_est_max_margin(3));
            tmp = find(activeFlag);
            if 0
                inlierId = ind(inlierId_);
            else
                inlierId = ind(tmp(inlierId_));
            end
            % figure(914);plot(k2cBodyRotAng,0,'*g');legend(num2str(k2cBodyRotAng));drawnow;
        end
        
    end
    methods (Static)
        function [reProjErr,dis,validFlag,err] = CalculateReprojectError_(intrMat,Point3D,Point2D,R,T,flag,outLierThreshold)
            %             assert(Rows(Point3D)>=Cols(Point3D),'3d point should be input as n*3 Vec')
            %             assert(Rows(Point2D)>=Cols(Point2D),'2d point should be input as n*3 Vec')
            dis = -1*ones(Rows(Point2D),1);
            validFlag = false(Rows(Point2D),1);
            if isempty(R)||isempty(T)
                reProjxyz = Point3D';
            else
                reProjxyz = R*Point3D' + repmat(T,1,Rows(Point3D));
            end
            reprojPix = intrMat*reProjxyz;
            reprojPix = [reprojPix(1,:)./reprojPix(3,:);reprojPix(2,:)./reprojPix(3,:)];
            reprojPix = reprojPix';
            err = sqrt((reprojPix(flag,1)-Point2D(flag,1)).^2 +(reprojPix(flag,2)-Point2D(flag,2)).^2);
            dis(flag) = sqrt((reprojPix(flag,1)-Point2D(flag,1)).^2 +(reprojPix(flag,2)-Point2D(flag,2)).^2);
            validFlag(flag) = dis(flag) <= outLierThreshold;
            reProjErr = sum(dis(flag))/Rows(Point3D(flag));
            if 0
                figure,plot(reprojPix(flag,1)-Point2D(flag,1),reprojPix(flag,2)-Point2D(flag,2),'+r')
            end
        end
        
    end
    
    methods (Static)
        function [curPredPtIcs,inTraceFlag,curPredPtIcs0,inTraceFlag0] = NewTracking_new(obj,keyFeatNum,prevFeatPtList,ptCcsZ,intrMat,k2cRef,angleModalOrg, keyDepthV, prevDepthV, currDepthV,DepthProbility, DepthId, AngleProbility,reCalc,DispRefineUse__)
            global roundingPrecision
            k2cRef0 = k2cRef;
%             k2cRef = deg2rad(round(rad2deg(k2cRef),1));
            k2cRef = deg2rad(round(rad2deg(k2cRef0)./roundingPrecision).*roundingPrecision);

            
            OnlyP2 = false; true;
            
            
            f = intrMat(1); baseline = norm(obj.camModel.transVec1To2);
            fbConst = f*baseline;
            [princpPtL, princpPtR] = Get(obj.camModel, 'PinholePrincpPt', obj.scaleLvl);
            
            T_B2C = obj.coordSysAligner.pctBody2Cam(1, 1).transformMat;
            invTb2c = inv(T_B2C);
            featuresNumber = keyFeatNum;
            activeFeat =  find(obj.featPtManager.localTrace.ptIcsX(:,end - reCalc) ~= -1 & obj.featPtManager.localTrace.ptIcsY(:,end - reCalc) ~=-1);
            ptIcX_pre = obj.featPtManager.localTrace.ptIcsX(:,1);
            ptIcY_pre = obj.featPtManager.localTrace.ptIcsY(:,1);
%             pt1 = Points.Point(ptIcX_pre,ptIcY_pre,intrMat);
            pt0 = Points.Point(ptIcX_pre,ptIcY_pre,intrMat);
            ptCcsZGolden = obj.featPtManager.localTrace.ptCcsZ;
            
            ptCcsGolden_disparity = (intrMat(1,1).*norm(obj.camModel.transVec1To2)./ptCcsZGolden) - (princpPtR(1) - princpPtL(1));
            ptCcsGolden_disparity = round(ptCcsGolden_disparity./obj.configParam.disparity_sample_step).*obj.configParam.disparity_sample_step;
            ptCcsZGolden = fbConst./(ptCcsGolden_disparity + (princpPtR(1) - princpPtL(1)));
            ptCcsZGolden(ptCcsZGolden < 0) = -1;
            %             m_ = Modals.Modal(obj.setting.configParam, featuresNumber);
            %             m_ = obj.modals;
            %             angleModalOrg = m_.angleModal;
            
            
            
            
            
            %             depthModalOrg = m_.depthModal;
            nextImg = rgb2gray(obj.currImgL);
            %             prevImg = obj.keyFrameImgL;
            prevImg = rgb2gray(obj.prevImgL);
            imgSize = size(nextImg); imgSize = imgSize(1:2);
            
            %             if isempty(obj.refAngList)
            
            if ~reCalc
                numThr = 1;
            else
                numThr = 2;
            end
            
            
            if size(obj.featPtManager.localTrace.ptIcsX,2) == numThr
                angleOrgin = 0;
            else
%                 angleOrgin = rad2deg(obj.refAngList3(end));
                angleOrgin = rad2deg(obj.refAngList4(end));
            end
            [pt0.matchingPtX,pt0.matchingPtY,pervPtCcsZGolden] = Points.GenerateMatchingPt(pt0,angleOrgin,T_B2C,invTb2c,ptCcsZGolden,featuresNumber,intrMat) ;
            
            %             obj.PervPtCcsZGolden = pervPtCcsZGolden;
            
            angleOrgin = rad2deg(k2cRef);
            [pt00.matchingPtX,pt00.matchingPtY,pervPtCcsZGolden00] = Points.GenerateMatchingPt(pt0,angleOrgin,T_B2C,invTb2c,ptCcsZGolden,featuresNumber,intrMat) ;
            
            ptIcX_pre = -1 * ones(featuresNumber,1);
            ptIcY_pre = -1 * ones(featuresNumber,1);
            
            
            
            if size(obj.featPtManager.localTrace.ptIcsX,2) == numThr
                
                ptIcX_pre = obj.featPtManager.localTrace.ptIcsX(:,1);
                ptIcY_pre = obj.featPtManager.localTrace.ptIcsY(:,1);
            else
                
                existIdx0 = find(obj.featPtManager.localTrace.ptIcsX(:,end - reCalc)-obj.setting.wx > 1 & obj.featPtManager.localTrace.ptIcsY(:,end - reCalc)-obj.setting.wy> 1 ...
                    & obj.featPtManager.localTrace.ptIcsX(:,end - reCalc)+obj.setting.wx < imgSize(2) & obj.featPtManager.localTrace.ptIcsY(:,end - reCalc)+obj.setting.wy< imgSize(1) ...
                    & obj.featPtManager.localTrace.ptIcsX(:,end-1 - reCalc)-obj.setting.wx > 1 & obj.featPtManager.localTrace.ptIcsY(:,end-1 - reCalc)-obj.setting.wy> 1 ...
                    & obj.featPtManager.localTrace.ptIcsX(:,end-1-reCalc)+obj.setting.wx < imgSize(2) & obj.featPtManager.localTrace.ptIcsY(:,end-1 - reCalc)+obj.setting.wy< imgSize(1) ...
                    &  obj.PervPtCcsZGolden~=-1);
                ptIcX_pre(existIdx0) = obj.featPtManager.localTrace.ptIcsX(existIdx0,end - reCalc);
                ptIcY_pre(existIdx0) = obj.featPtManager.localTrace.ptIcsY(existIdx0,end - reCalc);
                
                
            end
            
            
            if 1
                pt1 = Points.Point(ptIcX_pre,ptIcY_pre,intrMat);
            else
                pt1 = Points.Point(pt0.matchingPtX,pt0.matchingPtY,intrMat);
            end
            ptCcsZGolden = pervPtCcsZGolden';
            %             if isempty(obj.refAngList)
            
            
            
            
            
            
            
            if size(obj.featPtManager.localTrace.ptIcsX,2) == numThr
                angleOrgin = rad2deg(k2cRef);
            else
%                 angleOrgin = rad2deg(k2cRef - obj.refAngList3(end));
                angleOrgin = rad2deg(k2cRef - obj.refAngList4(end));
            end
            
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
%                 [pt1.matchingPtX,pt1.matchingPtY] = Points.GenerateMatchingPt(pt1,angleOrgin,T_B2C,invTb2c,ptCcsZ,featuresNumber,intrMat) ;
                       
                if 0
                    existIdx = find(pt1.matchingPtX-obj.setting.configParam.wx >= 1 & pt1.matchingPtY-obj.setting.configParam.wy>= 1 ...
                        & pt1.matchingPtX+obj.setting.configParam.wx <= imgSize(2) & pt1.matchingPtY+obj.setting.configParam.wy<= imgSize(1) ...
                        & ptCcsZGolden ~= -1); % & ptCcsZ~=-1
                else
                    % %                     existIdx = find(pt1.matchingPtX-obj.setting.configParam.wx >= 1 & pt1.matchingPtY-obj.setting.configParam.wy>= 1 ...
                    % %                         & pt1.matchingPtX+obj.setting.configParam.wx <= imgSize(2) & pt1.matchingPtY+obj.setting.configParam.wy<= 126 ...
                    % %                         & ptCcsZGolden ~= -1);
                    
                    existIdx = find(pt1.matchingPtX-obj.setting.configParam.wx > 1 & pt1.matchingPtY-obj.setting.configParam.wy> 1 ...
                        & pt1.matchingPtX+obj.setting.configParam.wx < imgSize(2) & pt1.matchingPtY+obj.setting.configParam.wy< 126*(240/imgSize(1)) ...
                        & pt1.x-obj.setting.configParam.wx > 1 & pt1.y-obj.setting.configParam.wy> 1 ...
                        & pt1.x+obj.setting.configParam.wx < imgSize(2) & pt1.y+obj.setting.configParam.wy< 126*(240/imgSize(1)) ...
                        &  ptCcsZGolden~=-1);
                    
                    
                end
                
            end
            
            obj.PervPtCcsZGolden = ptCcsZGolden;
            existFeaturesNumber = size(existIdx,1);
            existPt1 = Points.Point(pt1.x(existIdx),pt1.y(existIdx),intrMat);
            existPt2 = Points.Point(pt1.matchingPtX(existIdx),pt1.matchingPtY(existIdx),intrMat);
            
            existPtCcsZG = ptCcsZGolden(existIdx);
            existPtDisparityG1 = fbConst./existPtCcsZG;
            existPtDisparityG = (intrMat(1,1).*norm(obj.camModel.transVec1To2)./existPtCcsZG) - (princpPtR(1) - princpPtL(1));
            
            
            
            
            obj.setting.configParam.disparityOrginRange   = existPtDisparityG + obj.setting.configParam.disparityRange;
            
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
            tracePt = TraceWithWeights.Tracing(tracePt,obj.setting.configParam,table,prevImg,nextImg,existPt1,existFeaturesNumber,imgSize); % new tracing Pts
             
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
            %         figure(9),clf;plot(err(errr<1,1),err(errr<1,2),'+r')
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
        
        
        
        
        
    end

    methods (Static)
        function [curPredPtIcs,inTraceFlag,curPredPtIcs0,inTraceFlag0,weightRaw,candX,candY,errPair] = NewTracking0(obj,keyFeatNum,prevFeatPtList,ptCcsZ,intrMat,k2cRef,angleModalOrg, keyDepthV, prevDepthV, currDepthV,DepthProbility, DepthId, AngleProbility,reCalc,DispRefineUse__,DispRng)
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
               k2cRef = deg2rad(round(rad2deg(k2cRef0 + obj.thetaNoise)./roundingPrecision).*roundingPrecision);

            end
            
            OnlyP2 = false; true;
            
            
            f = intrMat(1); baseline = norm(obj.camModel.transVec1To2);
            fbConst = f*baseline;
            [princpPtL, princpPtR] = Get(obj.camModel, 'PinholePrincpPt', obj.scaleLvl);
            
            T_B2C = obj.coordSysAligner.pctBody2Cam(1, 1).transformMat;
            invTb2c = inv(T_B2C);
            featuresNumber = keyFeatNum;
            activeFeat =  find(obj.featPtManager.localTrace.ptIcsX(:,end - reCalc) ~= -1 & obj.featPtManager.localTrace.ptIcsY(:,end - reCalc) ~=-1);
            ptIcX_pre = obj.featPtManager.localTrace.ptIcsX(:,1);
            ptIcY_pre = obj.featPtManager.localTrace.ptIcsY(:,1);
%             pt1 = Points.Point(ptIcX_pre,ptIcY_pre,intrMat);
            pt0 = Points.Point(ptIcX_pre,ptIcY_pre,intrMat);
            ptCcsZGolden = obj.featPtManager.localTrace.ptCcsZ;
            if  size(obj.featPtManager.localTrace.ptIcsX,2) > 2
                ptCcsZGolden(DepthId) = (intrMat(1,1).*norm(obj.camModel.transVec1To2))./(DispRng(:,(size(DispRng,2)+1)/2) + (princpPtR(1) - princpPtL(1)));
            end
            
            
            
            
            
            %             m_ = Modals.Modal(obj.setting.configParam, featuresNumber);
            %             m_ = obj.modals;
            %             angleModalOrg = m_.angleModal;
            
            if 0 %~isempty(DispRefineUse__)
                
                if 0
                    figure, plot((intrMat(1,1).*norm(obj.camModel.transVec1To2))./(DispRefineUse__ + (princpPtR(1) - princpPtL(1))) - ptCcsZGolden(DepthId));
                end
                
                
                ptCcsZGolden(DepthId) = (intrMat(1,1).*norm(obj.camModel.transVec1To2))./(DispRefineUse__ + (princpPtR(1) - princpPtL(1)));
            end
%             existPtDisparityG = (intrMat(1,1).*norm(obj.camModel.transVec1To2)./existPtCcsZG) - (princpPtR(1) - princpPtL(1));
            
            
            %             depthModalOrg = m_.depthModal;
            nextImg = rgb2gray(obj.currImgL);
            %             prevImg = obj.keyFrameImgL;
            prevImg = rgb2gray(obj.prevImgL);
            imgSize = size(nextImg); imgSize = imgSize(1:2);
            
            %             if isempty(obj.refAngList)
            
            if ~reCalc
                numThr = 1;
            else
                numThr = 2;
            end
            
            
            if size(obj.featPtManager.localTrace.ptIcsX,2) == numThr
                angleOrgin = 0;
            else
%                 angleOrgin = rad2deg(obj.refAngList3(end));
                angleOrgin = rad2deg(obj.refAngList4(end));
            end
            [pt0.matchingPtX,pt0.matchingPtY,pervPtCcsZGolden] = Points.GenerateMatchingPt(pt0,angleOrgin,T_B2C,invTb2c,ptCcsZGolden,featuresNumber,intrMat) ;
            
            %             obj.PervPtCcsZGolden = pervPtCcsZGolden;
            
            angleOrgin = rad2deg(k2cRef);
            [pt00.matchingPtX,pt00.matchingPtY,pervPtCcsZGolden00] = Points.GenerateMatchingPt(pt0,angleOrgin,T_B2C,invTb2c,ptCcsZGolden,featuresNumber,intrMat) ;
            
            ptIcX_pre = -1 * ones(featuresNumber,1);
            ptIcY_pre = -1 * ones(featuresNumber,1);
            
            
            
            if size(obj.featPtManager.localTrace.ptIcsX,2) == numThr
                
                ptIcX_pre = obj.featPtManager.localTrace.ptIcsX(:,1);
                ptIcY_pre = obj.featPtManager.localTrace.ptIcsY(:,1);
            else
                
                existIdx0 = find(obj.featPtManager.localTrace.ptIcsX(:,end - reCalc)-obj.setting.wx > 1 & obj.featPtManager.localTrace.ptIcsY(:,end - reCalc)-obj.setting.wy> 1 ...
                    & obj.featPtManager.localTrace.ptIcsX(:,end - reCalc)+obj.setting.wx < imgSize(2) & obj.featPtManager.localTrace.ptIcsY(:,end - reCalc)+obj.setting.wy< imgSize(1) ...
                    & obj.featPtManager.localTrace.ptIcsX(:,end-1 - reCalc)-obj.setting.wx > 1 & obj.featPtManager.localTrace.ptIcsY(:,end-1 - reCalc)-obj.setting.wy> 1 ...
                    & obj.featPtManager.localTrace.ptIcsX(:,end-1-reCalc)+obj.setting.wx < imgSize(2) & obj.featPtManager.localTrace.ptIcsY(:,end-1 - reCalc)+obj.setting.wy< imgSize(1) ...
                    &  obj.PervPtCcsZGolden~=-1);
                ptIcX_pre(existIdx0) = obj.featPtManager.localTrace.ptIcsX(existIdx0,end - reCalc);
                ptIcY_pre(existIdx0) = obj.featPtManager.localTrace.ptIcsY(existIdx0,end - reCalc);
                
                
            end
             %%
            if 1
                pt1 = Points.Point(ptIcX_pre,ptIcY_pre,intrMat);
            else
                pt1 = Points.Point(pt0.matchingPtX,pt0.matchingPtY,intrMat);
                
            end
            ptCcsZGolden = pervPtCcsZGolden';
            %             if isempty(obj.refAngList)
            
            
            
            
            
            
            
            if size(obj.featPtManager.localTrace.ptIcsX,2) == numThr
                if 0
                    angleOrgin = rad2deg(k2cRef);
                else
                    angleOrgin = rad2deg(angOpt);
                end
%                 errPair =  [rad2deg(angOpt - k2cRef0) rad2deg(k2cRef - k2cRef0)];
                errPair =  [rad2deg(k2cRef - k2cRef0) rad2deg(angOpt - k2cRef0)];
            else
                %                 angleOrgin = rad2deg(k2cRef - obj.refAngList3(end));
                if isempty(angOptP2C)
                    if 0
                        angleOrgin = rad2deg(k2cRef - obj.refAngList4(end));
                    else
                        angleOrgin = rad2deg(angOpt);
                    end
                else
                    angleOrgin = rad2deg(angOptP2C); 
                end
                errGTP2C1 = -(rad2deg(k2cRef0 - obj.refAngList(end)) - rad2deg(angOptP2C));
                errGTP2C2 =  -(rad2deg(k2cRef0 - obj.refAngList(end)) - (rad2deg(k2cRef - obj.refAngList4(end))));
                errPair = [errGTP2C2 errGTP2C1];

            end
%             try
%                 errGTP2C1 = -(rad2deg(k2cRef0 - obj.refAngList(end)) - rad2deg(angOptP2C));
%                 errGTP2C2 =  -(rad2deg(k2cRef0 - obj.refAngList(end)) - (rad2deg(k2cRef - obj.refAngList4(end))));
%                 errPair = [errGTP2C2 errGTP2C1];
%             catch
%                 errPair =  [rad2deg(k2cRef - k2cRef0) rad2deg(k2cRef - k2cRef0)];
%             end
            [pt1.matchingPtX,pt1.matchingPtY,~] = Points.GenerateMatchingPt(pt1,angleOrgin,T_B2C,invTb2c,ptCcsZGolden,featuresNumber,intrMat) ;
            
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
            
            obj.PervPtCcsZGolden = ptCcsZGolden;
            existFeaturesNumber = size(existIdx,1);
            existPt1 = Points.Point(pt1.x(existIdx),pt1.y(existIdx),intrMat);
            existPt2 = Points.Point(pt1.matchingPtX(existIdx),pt1.matchingPtY(existIdx),intrMat);
            
            existPtCcsZG = ptCcsZGolden(existIdx);
            existPtDisparityG1 = fbConst./existPtCcsZG;
            existPtDisparityG = (intrMat(1,1).*norm(obj.camModel.transVec1To2)./existPtCcsZG) - (princpPtR(1) - princpPtL(1));
            
            
            if ~UsePrvDepth
                if 0
                    obj.setting.configParam.disparityOrginRange   = existPtDisparityG + obj.setting.configParam.disparityRange;
                    
                elseif 1
                    if  size(obj.featPtManager.localTrace.ptIcsX,2) > 2
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
                if  size(obj.featPtManager.localTrace.ptIcsX,2) > 2
                    prvFeat = [existPt1.x existPt1.y];
                    
                    prvFeatInd = sub2ind(size(prvDepth), round(prvFeat(:,2)), round(prvFeat(:,1)));
                    prvFeatDepth = prvDepth(prvFeatInd);
                    prvFeatDisp = (intrMat(1,1).*norm(obj.camModel.transVec1To2)./prvFeatDepth) - (princpPtR(1) - princpPtL(1));
                    
                    
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
            %         figure(9),clf;plot(err(errr<1,1),err(errr<1,2),'+r')
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
        
        
        
        
        
    end
    
    
    methods (Static)
        function [curPredPtIcs,inTraceFlag,curPredPtIcs0,inTraceFlag0,weightRaw,candX,candY, errPair] = NewTrackingTemp(obj,keyFeatNum,prevFeatPtList,ptCcsZ,intrMat,k2cRef,angleModalOrg, keyDepthV, prevDepthV, currDepthV,DepthProbility, DepthId, AngleProbility,reCalc,DispRng,DispRefineUse__)
            global doRoundingTheta roundingPrecision
            
            inlierId = currDepthV;
            angOptP2C = prevDepthV;
            angOpt = angleModalOrg;
            k2cRef0 = k2cRef;
            if doRoundingTheta
%                 k2cRef = deg2rad(round(rad2deg(k2cRef),1));
                k2cRef = deg2rad(round(rad2deg(k2cRef0 + obj.thetaNoise)./roundingPrecision).*roundingPrecision);

            end
            
            OnlyP2 = false; true;
            
            
            f = intrMat(1); baseline = norm(obj.camModel.transVec1To2);
            fbConst = f*baseline;
            [princpPtL, princpPtR] = Get(obj.camModel, 'PinholePrincpPt', obj.scaleLvl);
            
            T_B2C = obj.coordSysAligner.pctBody2Cam(1, 1).transformMat;
            invTb2c = inv(T_B2C);
            featuresNumber = keyFeatNum;
            activeFeat =  find(obj.featPtManager.localTrace.ptIcsX(:,end - reCalc) ~= -1 & obj.featPtManager.localTrace.ptIcsY(:,end - reCalc) ~=-1);
            ptIcX_pre = obj.featPtManager.localTrace.ptIcsX(:,1);
            ptIcY_pre = obj.featPtManager.localTrace.ptIcsY(:,1);
%             pt1 = Points.Point(ptIcX_pre,ptIcY_pre,intrMat);
            pt0 = Points.Point(ptIcX_pre,ptIcY_pre,intrMat);
            ptCcsZGolden = obj.featPtManager.localTrace.ptCcsZ;
            
            if  size(obj.featPtManager.localTrace.ptIcsX,2) > 2
                ptCcsZGolden(DepthId) = (intrMat(1,1).*norm(obj.camModel.transVec1To2))./(DispRng(:,(size(DispRng,2)+1)/2) + (princpPtR(1) - princpPtL(1)));
            end
            
            %             m_ = Modals.Modal(obj.setting.configParam, featuresNumber);
            %             m_ = obj.modals;
            %             angleModalOrg = m_.angleModal;
            
            
            
            
            
            %             depthModalOrg = m_.depthModal;
            nextImg = rgb2gray(obj.currImgL);
            %             prevImg = obj.keyFrameImgL;
            prevImg = rgb2gray(obj.prevImgL);
            imgSize = size(nextImg); imgSize = imgSize(1:2);
            
            %             if isempty(obj.refAngList)
            
            if ~reCalc
                numThr = 1;
            else
                numThr = 2;
            end
            
            
            if size(obj.featPtManager.localTrace.ptIcsX,2) == numThr
                angleOrgin = 0;
            else
%                 angleOrgin = rad2deg(obj.refAngList3(end));
                angleOrgin = rad2deg(obj.refAngList4(end));
            end
            [pt0.matchingPtX,pt0.matchingPtY,pervPtCcsZGolden] = Points.GenerateMatchingPt(pt0,angleOrgin,T_B2C,invTb2c,ptCcsZGolden,featuresNumber,intrMat) ;
            
            %             obj.PervPtCcsZGolden = pervPtCcsZGolden;
            if 0
                angleOrgin = rad2deg(k2cRef);
            else
                angleOrgin = rad2deg(angOpt);
            end
            
            [pt00.matchingPtX,pt00.matchingPtY,pervPtCcsZGolden00] = Points.GenerateMatchingPt(pt0,angleOrgin,T_B2C,invTb2c,ptCcsZGolden,featuresNumber,intrMat) ;
            
            ptIcX_pre = -1 * ones(featuresNumber,1);
            ptIcY_pre = -1 * ones(featuresNumber,1);
            
               
            
            if size(obj.featPtManager.localTrace.ptIcsX,2) == numThr
                
                ptIcX_pre = obj.featPtManager.localTrace.ptIcsX(:,1);
                ptIcY_pre = obj.featPtManager.localTrace.ptIcsY(:,1);
            else
                
                existIdx0 = find(obj.featPtManager.localTrace.ptIcsX(:,end - reCalc)-obj.setting.wx > 1 & obj.featPtManager.localTrace.ptIcsY(:,end - reCalc)-obj.setting.wy> 1 ...
                    & obj.featPtManager.localTrace.ptIcsX(:,end - reCalc)+obj.setting.wx < imgSize(2) & obj.featPtManager.localTrace.ptIcsY(:,end - reCalc)+obj.setting.wy< imgSize(1) ...
                    & obj.featPtManager.localTrace.ptIcsX(:,end-1 - reCalc)-obj.setting.wx > 1 & obj.featPtManager.localTrace.ptIcsY(:,end-1 - reCalc)-obj.setting.wy> 1 ...
                    & obj.featPtManager.localTrace.ptIcsX(:,end-1-reCalc)+obj.setting.wx < imgSize(2) & obj.featPtManager.localTrace.ptIcsY(:,end-1 - reCalc)+obj.setting.wy< imgSize(1) ...
                    &  obj.PervPtCcsZGolden~=-1);
                ptIcX_pre(existIdx0) = obj.featPtManager.localTrace.ptIcsX(existIdx0,end - reCalc);
                ptIcY_pre(existIdx0) = obj.featPtManager.localTrace.ptIcsY(existIdx0,end - reCalc);
                
                
            end
            if 0
                pt1 = Points.Point(ptIcX_pre,ptIcY_pre,intrMat);
            elseif 0
                pt1 = Points.Point(pt0.matchingPtX,pt0.matchingPtY,intrMat);
            else
                pt1 = Points.Point(pt0.x,pt0.y,intrMat);
            end
            ptCcsZGolden = pervPtCcsZGolden';
            %             if isempty(obj.refAngList)
            
            
            
            
            
             
            if 0
                if 1  % size(obj.featPtManager.localTrace.ptIcsX,2) == numThr
                    if 1
                        angleOrgin = rad2deg(k2cRef);
                    else
                        try
                            angleOrgin = rad2deg(k2cRef) - rad2deg(obj.refAngList3(end)) + rad2deg(obj.refAngList4(end));
                        catch
                            angleOrgin = rad2deg(k2cRef) ;
                        end
                    end
                    
                else
                    %                 angleOrgin = rad2deg(k2cRef - obj.refAngList3(end));
                    angleOrgin = rad2deg(k2cRef - obj.refAngList4(end));
                end
            end
            
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
            
            obj.PervPtCcsZGolden = ptCcsZGolden;
            existFeaturesNumber = size(existIdx,1);
            existPt1 = Points.Point(pt1.x(existIdx),pt1.y(existIdx),intrMat);
            existPt2 = Points.Point(pt1.matchingPtX(existIdx),pt1.matchingPtY(existIdx),intrMat);
            
            existPtCcsZG = ptCcsZGolden(existIdx);
            existPtDisparityG1 = fbConst./existPtCcsZG;
            existPtDisparityG = (intrMat(1,1).*norm(obj.camModel.transVec1To2)./existPtCcsZG) - (princpPtR(1) - princpPtL(1));
            
            
            DepthProbility_ = DispRng;
            
            
            if 0
                if isempty(DepthId)
                    DepthId = inlierId;
                end
                
                
                DepthProbility0 = ones(length(existIdx), obj.setting.configParam.disparityBin);
                if length(DepthId) >= length(existIdx)
                    if 0
                        DepthProbility0 = DepthProbility_(find(ismember(DepthId, existIdx)),:);
                        
                    else
                        %                     [ia,ib,ic] = intersect(DepthId, existIdx);
                        id1 = find(ismember(DepthId, existIdx));
                        id2 = find(ismember(existIdx, DepthId));
                        DepthProbility0 = ones(length(existIdx),size(DepthProbility_,2));
                        DepthProbility0(id2,:) = DepthProbility_(id1,:);
                    end
                else
                    if 0
                        DepthProbility0(find(ismember(existIdx, DepthId)),:) = DepthProbility_;
                    else
                        id1 = find(ismember(DepthId, existIdx));
                        id2 = find(ismember(existIdx, DepthId));
                        DepthProbility0 = ones(length(existIdx),size(DepthProbility_,2));
                        DepthProbility0(id2,:) = DepthProbility_(id1,:);
                    end
                end
                
                
            end
            
            
            
            
            
            
            if 0
                obj.setting.configParam.disparityOrginRange   = existPtDisparityG + obj.setting.configParam.disparityRange;
            else
                if size(obj.featPtManager.localTrace.ptIcsX,2) > 2
                    obj.setting.configParam.disparityOrginRange   = DispRng(ismember(DepthId, existIdx),:); % DepthProbility0;  %existPtDisparityG + obj.setting.configParam.disparityRange;
                else
                    obj.setting.configParam.disparityOrginRange   = existPtDisparityG + obj.setting.configParam.disparityRange;
                end
            end
%             table = LookUpTables.LookUpTable(obj.lookUpTables,existFeaturesNumber,obj.setting.configParam.tableSize); %initialize
            table = LookUpTables.LookUpTable(existFeaturesNumber,obj.setting.configParam.tableSize); %initialize
            table.winSize = obj.lookUpTables.winSize;
            table.angSize = obj.lookUpTables.angSize;
            table.angMove = obj.lookUpTables.angMove;
            errPair = [0 0];
            if 0 %~OnlyP2
                table = LookUpTables.GenerateTable(table,obj.setting.configParam,existPt1,existFeaturesNumber, imgSize,T_B2C,fbConst,intrMat,DepthProbility, DepthId, existIdx, AngleProbility,DispRefineUse__) ;
            elseif 1
                table = LookUpTables.GenerateTableTemp(table,obj.setting.configParam,existPt1,existFeaturesNumber, imgSize,T_B2C,fbConst,intrMat,DepthProbility, DepthId, existIdx, AngleProbility,DispRefineUse__) ;
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
            %         figure(9),clf;plot(err(errr<1,1),err(errr<1,2),'+r')
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
        
        
        
        
        
    end
    
    
    
    
    methods
        function [curPredPtIcs,inTraceFlag,curPredPtIcs0,inTraceFlag0] = NewTrackingOld(obj,keyFeatNum,prevFeatPtList,ptCcsZ,intrMat,k2cRef,angleModalOrg)
            
            
            
            OnlyP2 = false; true;
            
            
            f = intrMat(1); baseline = norm(obj.camModel.transVec1To2);
            fbConst = f*baseline;
            [princpPtL, princpPtR] = Get(obj.camModel, 'PinholePrincpPt', obj.scaleLvl);
            
            T_B2C = obj.coordSysAligner.pctBody2Cam(1, 1).transformMat;
            invTb2c = inv(T_B2C);
            featuresNumber = keyFeatNum;
            activeFeat =  find(obj.featPtManager.localTrace.ptIcsX(:,end) ~= -1 & obj.featPtManager.localTrace.ptIcsY(:,end) ~=-1);
            ptIcX_pre = obj.featPtManager.localTrace.ptIcsX(:,1);
            ptIcY_pre = obj.featPtManager.localTrace.ptIcsY(:,1);
            pt1 = Points.Point(ptIcX_pre,ptIcY_pre,intrMat);
            %             m_ = Modals.Modal(obj.setting.configParam, featuresNumber);
            m_ = obj.modals;
            %             angleModalOrg = m_.angleModal;
            
            
            
            
            
            depthModalOrg = m_.depthModal;
            nextImg = obj.currImgL;
            prevImg = obj.keyFrameImgL;
            imgSize = size(nextImg); imgSize = imgSize(1:2);
            
            angleOrgin = rad2deg(k2cRef);
            obj.setting.angleOrginRange = angleOrgin + obj.setting.angleRange;
            obj.setting.configParam.angleOrginRange = obj.setting.angleOrginRange;
            
            
             validInd = sub2ind(size(obj.depthVisual), round(pt1.y), round(pt1.x));
              ptCcsZGolden = obj.keyFrameDepthGT(validInd);
            
            if 0
            [pt1.matchingPtX,pt1.matchingPtY] = Points.GenerateMatchingPt(pt1,angleOrgin,T_B2C,invTb2c,ptCcsZGolden,featuresNumber,intrMat) ;
            
            
            existIdx = find(pt1.matchingPtX-obj.setting.configParam.wx >= 1 & pt1.matchingPtY-obj.setting.configParam.wy>= 1 ...
                & pt1.matchingPtX+obj.setting.configParam.wx <= imgSize(2) & pt1.matchingPtY+obj.setting.configParam.wy<= imgSize(1) ...
                &  ptCcsZ~=-1 & ptCcsZGolden ~= -1);
            else
                [pt1.matchingPtX,pt1.matchingPtY] = Points.GenerateMatchingPt(pt1,angleOrgin,T_B2C,invTb2c,ptCcsZ,featuresNumber,intrMat) ;
            
            
            existIdx = find(pt1.matchingPtX-obj.setting.configParam.wx >= 1 & pt1.matchingPtY-obj.setting.configParam.wy>= 1 ...
                & pt1.matchingPtX+obj.setting.configParam.wx <= imgSize(2) & pt1.matchingPtY+obj.setting.configParam.wy<= imgSize(1) ...
                &  ptCcsZ~=-1); % & ptCcsZGolden ~= -1);
                
                
                
            end
            existFeaturesNumber = size(existIdx,1);
            existPt1 = Points.Point(pt1.x(existIdx),pt1.y(existIdx),intrMat);
            existPt2 = Points.Point(pt1.matchingPtX(existIdx),pt1.matchingPtY(existIdx),intrMat);
            
            existPtCcsZ = ptCcsZ(existIdx);
            existPtDisparity1 = fbConst./existPtCcsZ;
            existPtDisparity = (intrMat(1,1).*norm(obj.camModel.transVec1To2)./existPtCcsZ) - (princpPtR(1) - princpPtL(1));
            
            
            
            
            obj.setting.configParam.disparityOrginRange   = existPtDisparity + obj.setting.configParam.disparityRange;
            
            %             table = LookUpTables.LookUpTable(obj.lookUpTables,existFeaturesNumber,obj.setting.configParam.tableSize); %initialize
            table = LookUpTables.LookUpTable(existFeaturesNumber,obj.setting.configParam.tableSize); %initialize
            try
                table.winSize = obj.lookUpTables.winSize;
                table.angSize = obj.lookUpTables.angSize;
                table.angMove = obj.lookUpTables.angMove;
            catch
                %% 20200408
                rsegthk = 1;
            end
            if ~OnlyP2
                table = LookUpTables.GenerateTable(table,obj.setting.configParam,existPt1,existPt2,existFeaturesNumber, imgSize,T_B2C,invTb2c,fbConst,intrMat) ;
            else
                table = LookUpTables.GenerateTableOnlyP2(table,obj.setting.configParam,existPt1,existPt2,existFeaturesNumber, imgSize,T_B2C,invTb2c,fbConst,intrMat) ;        
            end
            m_.angleModal = angleModalOrg;
            if ~OnlyP2
                % -- update modal angle -- %
                m_.angleModal = Modals.UpdateModalAngle(m_,obj.setting.configParam,table,existIdx);
                % -- update modal depth -- %
                m_.depthModal = Modals.UpdateModalDepth(m_,obj.setting.configParam,table,existIdx,existFeaturesNumber);
            else
                m_.angleModal = Modals.UpdateModalAngleOnlyP2(m_,obj.setting.configParam,table,existIdx);
                
            end
            if ~OnlyP2
                table.candidatesWeight = LookUpTables.GenerateTableWeight(table,obj.setting.configParam,m_,existIdx);
            else
                table.candidatesWeight = LookUpTables.GenerateTableWeightOnlyP2(table,obj.setting.configParam,m_,existIdx);
            end
            % ------get tracing pts with weights------ %
            tracePt = TraceWithWeights.TraceWithWeight(existFeaturesNumber);%initialize
            tracePt = TraceWithWeights.Tracing(tracePt,obj.setting.configParam,table,prevImg,nextImg,existPt1,existFeaturesNumber,imgSize); % new tracing Pts
            % -----re-calculate guassian---- %
            if ~OnlyP2
                try
                    table = LookUpTables.RecalculateTableGauss(table,obj.setting.configParam,tracePt,existFeaturesNumber);
                catch
                    %% 20200408
                    table = LookUpTables.RecalculateTableGauss(table,obj.setting.configParam,tracePt);
                end
            else
                table = LookUpTables.RecalculateTableGaussOnlyP2(table,obj.setting.configParam,tracePt,existFeaturesNumber);
            end
            % -----re-calculate modal angle---- %
            m_.angleModal = angleModalOrg;
            
            if ~OnlyP2
                m_.angleModal = Modals.UpdateModalAngle(m_,obj.setting.configParam,table,existIdx);
                % -----re-calculate modal depth---- %
                m_.depthModal = depthModalOrg;
                m_.depthModal = Modals.UpdateModalDepth(m_,obj.setting.configParam,table,existIdx,existFeaturesNumber);
            else
                m_.angleModal = Modals.UpdateModalAngleOnlyP2(m_,obj.setting.configParam,table,existIdx);
            end
            %             err = [ptIcX_cur(existIdx) - tracePt.x ptIcY_cur(existIdx) - tracePt.y];[~,errr]=NormalizeVector(err);
            %         figure(9),clf;plot(err(errr<1,1),err(errr<1,2),'+r')
            if 0
                figure,imshow(prevImg);hold on;plot(existPt1.x,existPt1.y,'.r')
                figure,imshow(nextImg);hold on;plot(tracePt.x,tracePt.y,'.r')
            end
            
            topMargin = obj.featPtTracker.configParam.top_margin;
            leftMargin = obj.featPtTracker.configParam.left_margin;
            bottomMargin = obj.featPtTracker.configParam.bottom_margin;
            rightMargin = obj.featPtTracker.configParam.right_margin;
            
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
            obj.modals = m_;
        end
        
        
        
        
        
    end
    
     methods
        function [curPredPtIcs,inTraceFlag,curPredPtIcs0,inTraceFlag0, topMargin] = NewTrackingOld2(obj,prevImg,nextImg, keyFeatNum,prevFeatPtList,ptCcsZ,intrMat,k2cRef,angleModalOrg)
            
            
            
            OnlyP2 = false; true;
            
            
            f = intrMat(1); baseline = norm(obj.camModel.transVec1To2);
            fbConst = f*baseline;
            [princpPtL, princpPtR] = Get(obj.camModel, 'PinholePrincpPt', obj.scaleLvl);
            
            T_B2C = obj.coordSysAligner.pctBody2Cam(1, 1).transformMat;
            invTb2c = inv(T_B2C);
            featuresNumber = keyFeatNum;
            
            if 0
                activeFeat =  find(obj.featPtManager.localTrace.ptIcsX(:,end) ~= -1 & obj.featPtManager.localTrace.ptIcsY(:,end) ~=-1);
                ptIcX_pre = obj.featPtManager.localTrace.ptIcsX(:,1);
                ptIcY_pre = obj.featPtManager.localTrace.ptIcsY(:,1);
            else
                activeFeat = [1:keyFeatNum]';
                ptIcX_pre = prevFeatPtList(:,1);
                ptIcY_pre = prevFeatPtList(:,2);
            end
            pt1 = Points.Point(ptIcX_pre,ptIcY_pre,intrMat);
            m_ = Modals2.Modal(obj.setting2.configParam, featuresNumber);
%             m_ = obj.modals2;
            %             angleModalOrg = m_.angleModal;
            
            
            
            
            
            depthModalOrg = m_.depthModal;
% %             nextImg = obj.currImgL;
% %             prevImg = obj.prevImgL;
            imgSize = size(nextImg); imgSize = imgSize(1:2);
            
            angleOrgin = rad2deg(k2cRef);
            obj.setting2.angleOrginRange = angleOrgin + obj.setting2.angleRange;
            obj.setting2.configParam.angleOrginRange = obj.setting2.angleOrginRange;
            
            
             validInd = sub2ind(size(obj.depthVisual), round(pt1.y), round(pt1.x));
              ptCcsZGolden = obj.keyFrameDepthGT(validInd);
              
              if 0
                  [pt1.matchingPtX,pt1.matchingPtY] = Points.GenerateMatchingPt(pt1,angleOrgin,T_B2C,invTb2c,ptCcsZGolden,featuresNumber,intrMat) ;
                  
                  
                  existIdx = find(pt1.matchingPtX-obj.setting.configParam.wx >= 1 & pt1.matchingPtY-obj.setting.configParam.wy>= 1 ...
                      & pt1.matchingPtX+obj.setting.configParam.wx <= imgSize(2) & pt1.matchingPtY+obj.setting.configParam.wy<= imgSize(1) ...
                      &  ptCcsZ~=-1 & ptCcsZGolden ~= -1);
              else
                  [pt1.matchingPtX,pt1.matchingPtY] = Points2.GenerateMatchingPt(pt1,angleOrgin,T_B2C,invTb2c,ptCcsZ,featuresNumber,intrMat) ;
                  
                  
                  existIdx = find(pt1.matchingPtX-obj.setting2.configParam.wx >= 1 & pt1.matchingPtY-obj.setting2.configParam.wy>= 1 ...
                      & pt1.matchingPtX+obj.setting2.configParam.wx <= imgSize(2) & pt1.matchingPtY+obj.setting2.configParam.wy<= imgSize(1) ...
                      &  ptCcsZ~=-1); % & ptCcsZGolden ~= -1);
                  
                  
                  
              end
            existFeaturesNumber = size(existIdx,1);
            existPt1 = Points2.Point(pt1.x(existIdx),pt1.y(existIdx),intrMat);
            existPt2 = Points2.Point(pt1.matchingPtX(existIdx),pt1.matchingPtY(existIdx),intrMat);
            
            existPtCcsZ = ptCcsZ(existIdx);
            existPtDisparity1 = fbConst./existPtCcsZ;
            existPtDisparity = (intrMat(1,1).*norm(obj.camModel.transVec1To2)./existPtCcsZ) - (princpPtR(1) - princpPtL(1));
            
            
            
            
            obj.setting2.configParam.disparityOrginRange   = existPtDisparity + obj.setting2.configParam.disparityRange;
            
            %             table = LookUpTables.LookUpTable(obj.lookUpTables,existFeaturesNumber,obj.setting.configParam.tableSize); %initialize
            table = LookUpTables2.LookUpTable(existFeaturesNumber,obj.setting2.configParam.tableSize); %initialize
            try
                table.winSize = obj.lookUpTables2.winSize;
                table.angSize = obj.lookUpTables2.angSize;
                table.angMove = obj.lookUpTables2.angMove;
            catch
                %% 20200408
                rsegthk = 1;
            end
            if ~OnlyP2
                table = LookUpTables2.GenerateTable(table,obj.setting2.configParam,existPt1,existPt2,existFeaturesNumber, imgSize,T_B2C,invTb2c,fbConst,intrMat) ;
            else
                table = LookUpTables2.GenerateTableOnlyP2(table,obj.setting2.configParam,existPt1,existPt2,existFeaturesNumber, imgSize,T_B2C,invTb2c,fbConst,intrMat) ;        
            end
            m_.angleModal = angleModalOrg;
            if ~OnlyP2
                % -- update modal angle -- %
                m_.angleModal = Modals2.UpdateModalAngle(m_,obj.setting2.configParam,table,existIdx);
                % -- update modal depth -- %
                m_.depthModal = Modals2.UpdateModalDepth(m_,obj.setting2.configParam,table,existIdx,existFeaturesNumber);
            else
                m_.angleModal = Modals2.UpdateModalAngleOnlyP2(m_,obj.setting2.configParam,table,existIdx);
                
            end
            if ~OnlyP2
                table.candidatesWeight = LookUpTables2.GenerateTableWeight(table,obj.setting2.configParam,m_,existIdx);
            else
                table.candidatesWeight = LookUpTables2.GenerateTableWeightOnlyP2(table,obj.setting2.configParam,m_,existIdx);
            end
            % ------get tracing pts with weights------ %
            tracePt = TraceWithWeights.TraceWithWeight(existFeaturesNumber);%initialize
            tracePt = TraceWithWeights.Tracing(tracePt,obj.setting.configParam,table,prevImg,nextImg,existPt1,existFeaturesNumber,imgSize); % new tracing Pts
            % -----re-calculate guassian---- %
            if ~OnlyP2
                try
                    table = LookUpTables2.RecalculateTableGauss(table,obj.setting2.configParam,tracePt,existFeaturesNumber);
                catch
                    %% 20200408
                    table = LookUpTables2.RecalculateTableGauss(table,obj.setting2.configParam,tracePt);
                end
            else
                table = LookUpTables2.RecalculateTableGaussOnlyP2(table,obj.setting2.configParam,tracePt,existFeaturesNumber);
            end
            % -----re-calculate modal angle---- %
            m_.angleModal = angleModalOrg;
            
            if ~OnlyP2
                m_.angleModal = Modals2.UpdateModalAngle(m_,obj.setting2.configParam,table,existIdx);
                % -----re-calculate modal depth---- %
                m_.depthModal = depthModalOrg;
                m_.depthModal = Modals2.UpdateModalDepth(m_,obj.setting2.configParam,table,existIdx,existFeaturesNumber);
            else
                m_.angleModal = Modals.UpdateModalAngleOnlyP2(m_,obj.setting.configParam,table,existIdx);
            end
            %             err = [ptIcX_cur(existIdx) - tracePt.x ptIcY_cur(existIdx) - tracePt.y];[~,errr]=NormalizeVector(err);
            %         figure(9),clf;plot(err(errr<1,1),err(errr<1,2),'+r')
            if 0
                figure,imshow(prevImg);hold on;plot(existPt1.x,existPt1.y,'.r')
                figure,imshow(nextImg);hold on;plot(tracePt.x,tracePt.y,'.r')
            end
            
            topMargin = obj.featPtTracker.configParam.top_margin;
            leftMargin = obj.featPtTracker.configParam.left_margin;
            bottomMargin = obj.featPtTracker.configParam.bottom_margin;
            rightMargin = obj.featPtTracker.configParam.right_margin;
            
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
            obj.modals2 = m_;
        end
        
        
        
        
        
    end
    
    
    
    methods (Static)
        function DrawPic(probPath, featInd, frameNum, traceBatchList, featSize, dd, ddGT, disparityRng, dispCompare)
%             featInd = 10;
            maxRow = 10;  %  12;
            maxCol = 16;
            featSizeSum = cumsum(featSize);
            rowNum = min(maxRow, length(featSize));
            
            % sum_c2c_All =
            figure(300);clf;
            for k = 1 : rowNum %length(featSize)
                
                if k == 1
                    dispTemp = dd(1:featSizeSum(1),:);
                    dispGTTemp = ddGT(1:featSizeSum(1),:);
%                     dispCompareTemp = dispCompare(1:featSizeSum(1),:);
                else
                    dispTemp = dd(featSizeSum(k-1)+1:featSizeSum(k),:);
                    dispGTTemp = ddGT(featSizeSum(k-1)+1:featSizeSum(k),:);
%                     dispCompareTemp = dispCompare(featSizeSum(k-1)+1:featSizeSum(k),:);
                end
                
                dispCompareTemp = dispCompare{k,1};
                
                startId = find(dispTemp(1,:) ~= -1);
                startId0 = startId;
                
                if length(startId) > maxCol
                    startId = startId(length(startId) - maxCol + 1:end);
                end
                
                
                if k == 1
                    figRow = rowNum;  %   length(featSize);
                    figCol = length(startId) + 2;
                    ofst  = startId(1)-1;
                    sum_c2c_All = zeros(length(disparityRng)*length(startId), length(disparityRng)*length(featSize));
                end
                
                
                dispTemp = dispTemp(:,startId);
                dispGTTemp = dispGTTemp(:,startId);
                if 0
                    dispCompareTemp = dispCompareTemp(:,startId);
                else
                    dispCompareTemp = dispCompareTemp(:,ismember(startId0, startId));
                end
                
                sum_c2c_all = [];
                
                dispErrCurveInd = [];
                for kk = 1 : min(1000000, size(dispTemp,2))
                    
                    dispCurList_ = dispTemp(:,kk);
                    dispCurList_gt = dispGTTemp(:,kk);
                    
                    
                    dispErrFeatInd = dispCurList_gt(featInd) - dispCurList_(featInd);
                    dispErrCurveInd = [dispErrCurveInd; dispErrFeatInd];
                    
                    DispRngC2C = dispCurList_(:) + disparityRng;
                    
                    DispRngStepC2C = mean(diff(DispRngC2C'))';
                    [~, depthGTIndAll_cur] = VisualLocalizer.RoundingDispErr2(DispRngC2C(:,(size(DispRngC2C,2)+1)/2),dispCurList_gt, DispRngStepC2C,DispRngC2C);
                    try
                        depthGTIndAll_cur(isnan(depthGTIndAll_cur)) = 1;
                    catch
                        asdfgk = 1;
                    end
                    ProbZTmp_update_norm_sum_tmp_c2c = zeros(size(DispRngC2C));
                    %                     ProbZTmp_update_norm_sum_tmp(depthGTInd11) = ProbZTmp_update_norm_sum(depthGTInd11);
                    ProbZTmp_update_norm_sum_tmp_c2c(depthGTIndAll_cur) = 1;
                    sum_c2c = sum(ProbZTmp_update_norm_sum_tmp_c2c);
                    
                    pulse2 = disparityRng(1):0.001:disparityRng(end);
                    pulseVal2 = zeros(1,length(pulse2));
                    [~, id_pulse2] = min(abs(dispErrFeatInd - pulse2));
                    pulseVal2(id_pulse2) = max((sum_c2c));
                    
                    
                    subplot(figRow, figCol, (k-1)*figCol + startId(kk) - ofst);plot(disparityRng, sum_c2c);grid on;
                    hold on;plot(pulse2, pulseVal2);
                    if k == 1 && kk == 1
                        title(num2str(frameNum));
                        
                    end
                    if kk == size(dispTemp,2)  % 1
                        title(num2str(traceBatchList(k)))
                    end
                    sum_c2c_ = repmat(sum_c2c,length(sum_c2c), 1);
                    
                    sum_c2c_all = [sum_c2c_all; sum_c2c_];
                    
                    if kk == size(dispTemp,2)
                        subplot(figRow, figCol, (k-1)*figCol + startId(kk) - ofst + 1);plot(dispErrCurveInd);grid on;title('gt - stereo')
                        %                         subplot(figRow, figCol, (k-1)*figCol + startId(kk) - ofst + 2);plot(diff(dispTemp(featInd,:)));grid on;title('gt - stereo')
                        if 0
                            subplot(figRow, figCol, (k-1)*figCol + startId(kk) - ofst + 2);plot(dispTemp(featInd,:) - dispTemp(featInd,1));grid on;title('c - k')
                        else
                            subplot(figRow, figCol, (k-1)*figCol + startId(kk) - ofst + 2);plot(dispCompareTemp(featInd,:) - dispCompareTemp(featInd,1));grid on;title('c - k')
                        end
                    end
                    
                end
                try
                    sum_c2c_All(end - size(sum_c2c_all,1)+1 : end, length(disparityRng)*(k-1)+1 : length(disparityRng)*(k)) = sum_c2c_all;
                catch
                    asdgfk = 1;
                end
            end
            
            drawnow;
            if 0
                saveas(gcf,fullfile(probPath,sprintf(strcat('dispTrace_',probPath(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('dispTrace_',probPath(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1 + FigBase)));
            else
                saveas(gcf,fullfile(probPath,sprintf(strcat('dispTrace_',probPath(end-14:end),'____%06d.fig'),frameNum)));
            end
        end
        
    end
    methods (Static)
        function [depthGTInd11, depthGTIndAll] = RoundingDispErr2(DispRngCenter,disp2round, DispRngStep,DispRng)
            disparityError = DispRngCenter - disp2round;
            
            disparityErrorRound = round(disparityError./DispRngStep).*DispRngStep;
            
            
            depthGTOfst11 = round(-(disparityErrorRound)./DispRngStep);
            depthGTInd11 = depthGTOfst11 + (size(DispRng,2)+1)/2;
            depthGTInd11(depthGTInd11 < 1) = 1;
            depthGTInd11(depthGTInd11 > size(DispRng,2)) = size(DispRng,2);
            
            depthGTIndAll = sub2ind(size(DispRng),[1:size(DispRng,1)]', depthGTInd11);
            
            
        end
        
    end
    
    
    methods
        % Implementation of abstract methods in Configurable base class
        function SetDefaultValue(obj, cfgParam)
            cfgParam = Configurable.SetField(cfgParam, 'min_feature_point_number', 100);
            cfgParam = Configurable.SetField(cfgParam, 'feature_num_drop_rate_thresh', 0.65);  % 0.65 0.6
            cfgParam = Configurable.SetField(cfgParam, 'zero_translation_thresh', 0.01);
            cfgParam = Configurable.SetField(cfgParam, 'zero_rotation_angle_thresh', 0.2);
            cfgParam = Configurable.SetField(cfgParam, 'reproject_outlier_threshold', 300);
            cfgParam = Configurable.SetField(cfgParam, 'min_num_pt_for_calculate', 100);
            cfgParam = Configurable.SetField(cfgParam, 'min_num_pt_for_calculate2', 3);
            cfgParam = Configurable.SetField(cfgParam, 'disparity_error', 2); % 2 1
            cfgParam = Configurable.SetField(cfgParam, 'depth_uncertainty_range', [-100 300]);
            cfgParam = Configurable.SetField(cfgParam, 'theta_range', deg2rad([-1 1]));  % [-0.3 0.3]
            cfgParam = Configurable.SetField(cfgParam, 'polygon_margin', 2);
            cfgParam = Configurable.SetField(cfgParam, 'polygon_inlier_thresh', 0.2);
            cfgParam = Configurable.SetField(cfgParam, 'theta_sample_step', deg2rad(0.05));% 0.05 0.01
            cfgParam = Configurable.SetField(cfgParam, 'theta_sample_step2', deg2rad(0.01));% 0.05 0.01
            cfgParam = Configurable.SetField(cfgParam, 'disparity_sample_step', 0.1); % 0.01 % 0.05 0.1
            cfgParam = Configurable.SetField(cfgParam, 'reproj_sample_step', 0.001);
            
            cfgParam = Configurable.SetField(cfgParam, 'disparity_sample_interval', 0.01);
            cfgParam = Configurable.SetField(cfgParam, 'reproj_sample_interval', 0.01);
            
            cfgParam = Configurable.SetField(cfgParam, 'disparity_sigma',15); %  1/3  3
            cfgParam = Configurable.SetField(cfgParam, 'disparity_beta',100);
            
            cfgParam = Configurable.SetField(cfgParam, 'disparity_sigma_cur',0.4); %  1/3  3
            cfgParam = Configurable.SetField(cfgParam, 'disparity_beta_cur',20);
            
            
            
            
            
            
            
            
            
            
            
            cfgParam = Configurable.SetField(cfgParam, 'reproj_sigma0', 1.5*2);
            cfgParam = Configurable.SetField(cfgParam, 'reproj_sigma_update_z', 1);
            cfgParam = Configurable.SetField(cfgParam, 'reproj_sigma', 1.5*2); % 1/3
            cfgParam = Configurable.SetField(cfgParam, 'reproj_beta', 40);
            
            cfgParam = Configurable.SetField(cfgParam, 'reproj_sigma_right0', 1.5*2);
            cfgParam = Configurable.SetField(cfgParam, 'reproj_sigma_update_z_right', 1);
            cfgParam = Configurable.SetField(cfgParam, 'reproj_sigma_right', 1.5*2); % 1/3
            cfgParam = Configurable.SetField(cfgParam, 'reproj_beta_right', 40);
            
            cfgParam = Configurable.SetField(cfgParam, 'epiLine_margin',0);
            cfgParam = Configurable.SetField(cfgParam, 'probZ_ratio_threshold',0.0004); % 0.05
            
            cfgParam = Configurable.SetField(cfgParam, 'theta_prob_cutoff_threshold',0.99); % 0.001
            cfgParam = Configurable.SetField(cfgParam, 'depth_prob_cutoff_threshold',0.7);
            cfgParam = Configurable.SetField(cfgParam, 'tracking_trust_radius',1.2*2); %2
            cfgParam = Configurable.SetField(cfgParam, 'ref_theta_trust_margin',deg2rad(0.2));  % 0.5
            cfgParam = Configurable.SetField(cfgParam, 'theta_diff_margin',0.005);
            cfgParam = Configurable.SetField(cfgParam, 'expectation_theta_radius', deg2rad(0.2));
            cfgParam = Configurable.SetField(cfgParam, 'expectation_theta_expand_num', 10);
            
            cfgParam = Configurable.SetField(cfgParam, 'only_p2_prob_cutoff_threshold', 0.8);
            cfgParam = Configurable.SetField(cfgParam, 'only_p2_prob_cutoff_threshold_2', 0.95);
            
            cfgParam = Configurable.SetField(cfgParam, 'disparity_resample_prob_threshold', 0.5);
            cfgParam = Configurable.SetField(cfgParam, 'theta_resample_prob_threshold', 0.5);
            
            
            
            cfgParam = Configurable.SetField(cfgParam, 'reproj_sigma_scale', 1);
            cfgParam = Configurable.SetField(cfgParam, 'reproj_sigma_scale_right', 5);
            
            cfgParam = Configurable.SetField(cfgParam, 'modulate_curve_quant_size', 0.1);  % 0.05;
            
            cfgParam = Configurable.SetField(cfgParam, 'theta_percentage', 0.6);
            cfgParam = Configurable.SetField(cfgParam, 'depth_hist_ratio', 0.95);
            
            
            
            cfgParam = Configurable.SetField(cfgParam, 'theta_sigma', 0.01);
            
            obj.configParam = cfgParam;
        end
        
        function CheckConfig(obj) %#ok<MANU>
        end
    end
end