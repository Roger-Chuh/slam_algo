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
            obj.twoOnlyP2List;
            obj.meanErrAndStd = [];
            
        end
        
        % Main entry
        function Localize(obj, imgL, imgR, refMotionState, refRobotPoseWcs, frameInd)
            if (obj.scaleLvl > 0)
                imgL = imresize(imgL, 1/2^obj.scaleLvl);
                imgR = imresize(imgR, 1/2^obj.scaleLvl);
            end
            
            obj.currImgL = imgL;
            obj.currImgR = imgR;
            obj.currRefRobotPoseWcs = WcsPoseVecToPct(obj, refRobotPoseWcs);
            obj.currRefRobotRotAngWcs = refRobotPoseWcs(3) +  0;  %deg2rad(0.01*rand(1));
            obj.refRobotPosVec = refRobotPoseWcs;
            doInsert = NeedInsertKeyFrame(obj, refMotionState, false);
            if size(obj.poseWcsList,1) == 250; 97; 32; 81; 257
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
            global  DRAWPOLYGON ANGLEONLY SHOWPOLYGON USEGOLDENDISP NEWTRACKER NewOnlyP2 NewTrackerWithProb
            %             intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,1);
            intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,obj.scaleLvl);
            [prevFeatPtList, ptCcsZ] = GetActiveFeatures(obj.featPtManager);
            [prevFeatPtList0, ptCcsZ0] = GetActiveFeatures(obj.featPtManager,1);
            
            
            if size(obj.featPtManager.localTrace.ptIcsX,2) == 1
                obj.setting = Setting([]);
                obj.lookUpTables = LookUpTables([]);
                obj.lookUpTables.winSize = obj.setting.winSize;
                obj.lookUpTables.angSize = obj.setting.angSize;
                obj.lookUpTables.angMove = obj.setting.angMove;
                obj.lookUpTables.configParam.winSize = obj.setting.winSize;
                obj.lookUpTables.configParam.angSize = obj.setting.angSize;
                obj.lookUpTables.configParam.angMove = obj.setting.angMove;
                obj.modals = Modals([]);
                obj.points = Points([]);
                obj.traceWithWeights = TraceWithWeights([]);
                obj.angleModalOrg = zeros(1,obj.setting.angleBin);
                obj.modals = Modals.Modal(obj.setting.configParam, size(obj.featPtManager.localTrace.ptIcsX,1));
                DepthProbility = [];
                DepthId = [];
            else
                %                 DepthProbility = obj.keyProbZ{end,4};
                try
                    DepthProbility = obj.keyProbZ{end,24};
                    DepthId = obj.keyProbZ{end,2};
                catch
                    dvwnjk = 1;
                end
                %                 ptCcsZ0 = ptCcsZ;
            end
            
            
            
            if ~isempty(prevFeatPtList) && ~all(ptCcsZ<0)
                if ~NEWTRACKER
                    [curPredPtIcs, inTrackFlag] = TrackFeaturePoints(obj.featPtTracker, obj.prevImgL, obj.currImgL, prevFeatPtList, [], intrMat, GetPctBody2Cam(obj.coordSysAligner));
                     AngleModalOrg = obj.angleModalOrg;
                    AngleProbility = [];
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
                        figure(99);clf;imshow(depthErrMap,[]);title('stereo - gt');
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
                            
                            
                            [k2cBodyRotAng, reprojError, activeFlag2] = RotationAngleEstimate_PNP(obj.pureRotationAngPredictor,intrMat,obj.featPtManager,obj.coordSysAligner,k2cRef,obj.keyFrameFlagList,obj.frmStampList,obj.goldenPose);
                            %                             RefineZ(obj.pureRotationAngPredictor,intrMat,obj.featPtManager,obj.coordSysAligner,k2cRef,obj.keyFrameFlagList,obj.frmStampList,obj.goldenPose);
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
                                    
                                    
                                    
                                    if 1
                                        try
                                            [angOpt1, inlierIdNew,  angRng1, pixKey ,pt2dCur, DispRefine, angOpt33, inFlag] = RefineZ(obj, k2cRef,k2cBodyRotAng, inlierId,inlierId2, theta, DepthProbility,  DepthId,  AngleModalOrg,  AngleProbility,prevFeatPtList0,ptCcsZ0);  %, keyCcsXYZAll);
                                            thetaPnP = pnpLite(obj.pureRotationAngPredictor,intrMat,obj.featPtManager,obj.coordSysAligner,k2cRef,obj.keyFrameFlagList,obj.frmStampList,obj.goldenPose,pixKey ,pt2dCur, DispRefine, obj.camModel, obj.scaleLvl);
                                        catch
                                            wblnk = 1;
                                        end
                                    else
                                        angOpt1 = 0.02;
                                        angRng1 = [-0.01 0.03];
                                        thetaPnP = 0.01;
                                        angOpt33 = 0.025;
                                    end
%                                     thetaPnP = pnpLite(obj.pureRotationAngPredictor,intrMat,obj.featPtManager,obj.coordSysAligner,k2cRef,obj.keyFrameFlagList,obj.frmStampList,obj.goldenPose,pixKey ,pt2dCur, DispRefine, obj.camModel, obj.scaleLvl);
                                    obj.angOpt = [obj.angOpt; [angOpt1]];
                                    obj.angOpt3 = [obj.angOpt3; [angOpt33]];
                                    obj.angOptPnP = [obj.angOptPnP; [thetaPnP]];
                                    obj.angRng = [obj.angRng; angRng1];
                                    
                                    
                                    
                                    
                                    
                                    
                                    
                                    figure(14),
                                    if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                        clf;
                                    end
                                    hold on;plot(pix0_GT_(:,1) - PixCur(:,1), pix0_GT_(:,2) - PixCur(:,2), '+');title('gt - tracking');axis equal;
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
                                    
                                    if 1
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
                                        obj.featPtManager.localTrace.xGT(inlierId(validXYZAll),end) = pix0_GT(:,1);
                                        obj.featPtManager.localTrace.yGT(inlierId(validXYZAll),end) = pix0_GT(:,2);
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
                            
                            
                            
                            obj.angOpt = [obj.angOpt; [theta]];
                            obj.angOptPnP = [obj.angOptPnP; [theta]];
                            obj.angOpt3 = [obj.angOpt3; [theta]];
%                             obj.angRng = [obj.angRng; obj.angRng(end,:) + theta];
                            obj.angRng = [obj.angRng; theta + deg2rad([-0.2 0.2])];
                            
                            
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
        function [reProjErr,dis,validFlag,err,errXY] = CalculateReprojectError2(intrMat,Point3D,Point2D,R,T,flag,outLierThreshold)
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
          function [angOpt, inlierId, angRng, PixUse, pt2dCurUse, DispRefineUse, angOpt3, inFlag] = RefineZ(obj, k2cRef,k2cPnp, inlierId0, inlierId2, theta, DepthProbility,  DepthId,  AngleModalOrg,  AngleProbility,prevFeatPtList0,ptCcsZ0)  %, XYZ)
            global SHOWPOLYGON DRAWPOLYGON ANGLEONLY USEGOLDENDISP UPDATEDEPTH DEPTHITERNUM probPath ...
                USELEFT USELEFTPLUSRIGHT USELEFTXRIGHT USERIGHT WITHINEPILINE BOTHLR HARDCUT QUANTCUT FUSIONCUT ...
                FORCEEVENPLATFORM CUTTHETAANDFORCEPLATFORM DIFFTHETATHR CUTTHETAANDFORCEPLATFORM_NODIFF FORCEEXPEVEN ...
                EXPZERO GTTRACKING SOFTP2 Rati Rati1 Rati2 Sc USEthetaProb0 ONLYP2 ONLYP2_2 UPDATEZ_2 UPDATETracking_2 ...
                Sc1 Sc2 FORCETHETAPLATFORM1 FORCETHETAPLATFORM2 ONLYP2_1_Max NewOnlyP2 UPDATETracking_2_2 UpdateP2_3
            
            k2cRef0 = k2cRef;
            k2cRef = deg2rad(round(rad2deg(k2cRef),1));
             
            
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
            
            [depthMapKeyVisual, dispMapKey] = GetDepthMap(obj, obj.keyFrameImgL, obj.keyFrameImgR);
            
            thetaSamp = [obj.configParam.theta_range(1) : obj.configParam.theta_sample_step : obj.configParam.theta_range(2)];
            thetaSamp0 = thetaSamp;
            thetaRange = k2cRef + [obj.configParam.theta_range(1) : obj.configParam.theta_sample_step : obj.configParam.theta_range(2)];
            thetaRng = thetaRange;
            
            %             [probTheta, ~] = probDensity(k2cRef, obj.configParam.theta_sigma, thetaRng,obj.configParam.theta_sample_step, 'theta');
            
            disparityRng = [-obj.configParam.disparity_error : obj.configParam.disparity_sample_step : obj.configParam.disparity_error];
            [thetaGrid, disparityGrid] = meshgrid(thetaRange, disparityRng);
            ThetaDisp = [thetaGrid(:) disparityGrid(:)];
            
            
            
            
            depthMapKey = obj.keyFrameDepth;
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
            
            idUnValid = find(isnan(zList) | isinf(zList) | isnan(dispList) | isinf(dispList) | isnan(dispCurList_) | isnan(dispGTTmp)) ; % | isnan(dispGTTmp) | isinf(dispGTTmp));
            inlierId = setdiff(inlierId0, idUnValid);
            
            if ~isempty(obj.featPtManager.localTrace.probZ)
                as = find(max(obj.featPtManager.localTrace.probZ' > 0.000001)');
                inlierId = intersect(inlierId,as);
            end
            
            
            dispMapCurGTList = dispMapCurGTList0(inlierId,:);
            %             pixGT = pixGT0(ismember(inlierId0, inlierId),1:2);
            
            if 1 % ~USEGOLDENDISP %size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                %                     figure(38),clf;plot(dispList - dispMatGT(validInd));title('stereoDisp - gtDisp');
                %                 dispGTTmp = dispMatGT(validInd);
                figure(38),clf;subplot(1,3,1);hold on;plot(dispList(inlierId) - dispGTTmp(inlierId));title('stereoDisp - gtDisp');subplot(1,3,2);imshow(imgCur);hold on; %subplot(1,3,3);plot(dispErrCur);title('stereoDispCur - gtDispCur');
                disparityError = dispList(inlierId) - dispGTTmp(inlierId);
                disparityErrorRound = round(disparityError./obj.configParam.disparity_sample_step).*obj.configParam.disparity_sample_step;
            end
            
            
            reprojError = [];cntt = 0;PixCur = [];PixReproj = [];
            validId = []; validXYZAll = []; Pix0 = []; vldReproj = [];
            keyLength = size(obj.featPtManager.localTrace.ptIcsX,2);
            
            
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
            end
            
            
            
            
            
            
            if DRAWPOLYGON
                if ~isempty(inlierId)
                    %                     if SHOWPOLYGON
                    %                         figure(110),clf;imshow(imgCurL);hold on;
                    %                     end
                    
                    Pix = [obj.featPtManager.localTrace.ptIcsX(inlierId,1) obj.featPtManager.localTrace.ptIcsY(inlierId,1)];
                    
                    ZTrue = zList(inlierId);
                    
                    
                    metricPrevPtCcs = intrMat\HomoCoord(Pix',1);
                    metricPrevPtCcs = normc(metricPrevPtCcs);
                    scaleAll = ZTrue./metricPrevPtCcs(3,:)';
                    %                                   scale0 = zTrue./metricPrevPtCcs(3,:)';
                    keyCcsXYZAll = [repmat(scaleAll',3,1).*metricPrevPtCcs];
                    Prob = zeros(length(thetaRng),length(inlierId),length(disparityRng));
                    
                    if ~GTTRACKING
                        pt2dCur = [obj.featPtManager.localTrace.ptIcsX(inlierId,keyLength) obj.featPtManager.localTrace.ptIcsY(inlierId,keyLength)];
                    else
                        if 1
                            pt2dCur =  pixGT;
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
                    
                    
                    %                     pixGTR = [pixGT(:,1) - dispMapCurGTList pixGT(:,2)];
                    
                    
                    
                    %                                     figure(232);hold on;axis equal;
                    if 1
                        if isempty(obj.featPtManager.localTrace.probZ)
                            DispRng = repmat(dispList(inlierId,:),1,length(disparityRng)) + repmat(disparityRng,length(inlierId),1);
                            DispRng(DispRng < 0) = 0.0001;
                            
                            
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
                            
                            DispRng = repmat(dispList(inlierId,:),1,length(disparityRng)) + repmat(disparityRng,length(inlierId),1);
                            DispRng(DispRng < 0) = 0.0001;
                            
                            
                            ProbZ = obj.featPtManager.localTrace.probZ(inlierId,:);
                            ProbZ_out = obj.keyProbZ{end,21}(:,2);
                            idLast = obj.keyProbZ{end,2};
                            outId = find(~ismember(idLast, inlierId));
                            ZZVec1 = obj.featPtManager.localTrace.sampleZ(inlierId,:);
                            asdk = 1;
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
                        
                        for jkl = 1 : DEPTHITERNUM
                            
                            
                            
                            [maxFeatZ1,idMax] = max(ProbZ');
                            maxFeatZ = maxFeatZ1';
                            maxFeatZ = repmat(maxFeatZ, 1, size(ProbZ,2));
                            ProbZTmp_norm = ProbZ./maxFeatZ;
                            
                            % %                         ProbZ = ProbZTmp_norm;
                            
                            depthGTOfst = round(-(disparityErrorRound)./obj.configParam.disparity_sample_step);
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
%                             [depthGTInd, ProbZTmpTmp_norm, ProbZTmpNorm, idMax, depthGTIndAll, depthUpdateIndAll] = VisualLocalizer.GetDepthHist(ProbZ,disparityErrorRound,obj.configParam.disparity_sample_step);
                            
                            
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
                            
                            
                            
                            
                            ReprojErrVecTmp = {}; ReprojErrVecRTmp = {}; ReprojErrVecTmpMatT = {}; ReprojErrVecRTmpMatT = {};
                            ValidFeatFlag = []; ValidFeatFlag0 = []; ValidFeatFlagFusion = [];ValidFeatFlagQuant = [];
                            thetaRng0 = thetaRng;
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
                            
                            rati = Rati1;
                            thetaProb111 = thetaProb./max(thetaProb);
                            thetaProb222 = thetaProb111(thetaProb111 >= rati);
                            thetaProb222 = thetaProb222./sum(thetaProb222);
                            
                            
                            angOpt = deg2rad(dot(rad2deg(thetaRng(thetaProb111 >= rati)), thetaProb222));
                            
                            
                            
                            
                            
                            
                            
                            
                            
                            
                            
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
                                        
                                        figure(20),subplot(2,1,1);hold on;plot(rad2deg(thetaSamp), thetaProb);grid on; title('1st theta from only p2');
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
                                                [curPredPtIcs,inTrackFlag,curPredPtIcs0,inTrackFlag0,weightRaww,candXX,candYY] = NewTracking(obj, size(obj.featPtManager.localTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs, AngleModalOrg, obj.keyFrameDepth, obj.prvDepthVisual, obj.depthVisual,DepthProbility, DepthId, thetaProbUse__,1);
                                                %                                             [~,~,~,~,weightRaw,candX,candY] = NewTracking(obj, size(obj.featPtManager.localTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs, AngleModalOrg, obj.keyFrameDepth, obj.prvDepthVisual, obj.depthVisual,DepthProbility, DepthId, thetaProbUse__,1);
                                            else
                                                [curPredPtIcs,inTrackFlag,curPredPtIcs0,inTrackFlag0] = NewTracking_new(obj, size(obj.featPtManager.localTrace.ptIcsX,1),prevFeatPtList0,ptCcsZ0,intrMat, obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs, AngleModalOrg, obj.keyFrameDepth, obj.prvDepthVisual, obj.depthVisual,DepthProbility, DepthId, thetaProbUse__,1);
                                                
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
                                            
                                            [thetaProbUse, thetaProbUse_, thetaProbUse__, curPredPtIcs, weightRaw, candX, candY,candX_reshape, candY_reshape, weightRaw_reshape, idff] = UpdateNewTracking(obj, thetaSamp, thetaProb, prevFeatPtList0, ptCcsZ0, intrMat, AngleModalOrg, DepthProbility, DepthId, inlierId);
                                        end
                                    end
                                    
                                    
                                    
                                else
                                    figure(20)
                                    if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                        probDir = probPath;
                                        clf;
                                    end
                                    
                                    figure(20),subplot(2,1,1);hold on;plot(rad2deg(thetaSamp), thetaProb);grid on; title('1st theta from only p2');
                                    plot(rad2deg(angOpt3 - k2cRef0), max(thetaProb),'or');
                                    
                                    
                                    
                                    
                                    
                                    
                                    
                                    
                                    
                                end
                                
                                
                                
                                thetaProb0 = thetaProb;
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
                                    plot(rad2deg(thetaSamp), thetaProb);hold on;plot(rad2deg(thetaSamp(idM_Vec00)), thetaProb(idM_Vec00), '.r');grid on;
                                    %                                 subplot(1,2,2);plot(rad2deg(thetaSamp), thetaProb);hold on;plot(rad2deg(thetaSamp(idM_Vec11)), thetaProb(idM_Vec11), '.r');grid ongrid on;
                                    if 1 %size(obj.featPtManager.localTrace.ptIcsX, 2) <= 3
                                        saveas(gcf,fullfile(probPath,sprintf(strcat('cutThetaDiff_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('cutThetaDiff_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                                    end
                                end
                                thetaProb0 = thetaProb;
                                idM_Vec00Diff = idM_Vec00;
                                
                                
                                
                                if FORCETHETAPLATFORM1
                                thetaProb(idM_Vec00) = max(thetaProb);
                                end
                                thetaProb = thetaProb./sum(thetaProb);
                                
%                             else
%                                 idM_Vec11 = find(thetaProb./max(thetaProb) > obj.configParam.theta_prob_cutoff_threshold) ;
%                                 idM_Vec11 = find(thetaProb./max(thetaProb) > obj.configParam.theta_prob_cutoff_threshold) ;
%                                 figure(19),clf;plot(rad2deg(thetaSamp), thetaProb);hold on;plot(rad2deg(thetaSamp(idM_Vec11)), thetaProb(idM_Vec11), '.r');grid on;
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
                                        rati = Rati1;
                                        thetaProb1 = thetaProb./max(thetaProb);
                                        thetaProb2 = thetaProb1(thetaProb1 >= rati);
                                        thetaProb2 = thetaProb2./sum(thetaProb2);
                                        
                                        if ~ONLYP2_1_Max
                                        thetaExp_2 = dot(rad2deg(thetaSamp(thetaProb1 >= rati)), thetaProb2);
                                        else
                                            thetaExp_2 = rad2deg(angOpt3 - k2cRef);
                                        end
                                        
                                         
                                        if EXPZERO
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
                                    
                                    
                                    
                                    thetaRng = [(-obj.configParam.theta_sample_step2.*[expandNum:-1:1] + pltfrm_1(1)) pltfrm_1 (pltfrm_1(end) + obj.configParam.theta_sample_step2.*[1 : expandNum])];
                                    
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
                                  
                                %                                 ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, pt2dCur, pt2dCurR,ProbZ,ProbZTmp_norm,imgCur,imgCurR, thetaSamp);
                                
                                
                                if 1 % ~ONLYP2
                                    if 1
                                        
                                        
                                        
                                      
                                        if  1 % ONLYP2
                                            
                                            if 0 %~ONLYP2_2
                                                [thetaProb,idM,angOpt, ReprojErrVecTmp, ReprojErrVecRTmp, ReprojErrVecTmpMatT, ReprojErrVecRTmpMatT,ValidFeatFlag,ValidFeatFlag0,ValidFeatFlagFusion,ValidFeatFlagQuant,ProbReprojVecAll, ProbReprojVecRAll, PixKeyVecTmp, PixKeyVecRTmp, Probb, ...
                                                    validFeatFlagFusion,validFeatFlag, ProbReprojVec, ProbReprojVecR,bb,b,pixKeyVecTmpXCoordMatStack,pixKeyVecTmpYCoordMatStack,pixKeyVecRTmpXCoordMatStack,pixKeyVecRTmpYCoordMatStack]...
                                                    = ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, pt2dCur, pt2dCurR,ProbZ, ProbZTmp_norm, imgCur, imgCurR,thetaSamp);
                                            else
                                                
                                                
                                                obj.configParam.reproj_sigma = obj.configParam.reproj_sigma0 ;
                                                obj.configParam.reproj_sigma_right = obj.configParam.reproj_sigma_right0;
                                                
                                                
                                                
                                                
                                                [thetaProb__,idM__,angOpt__, ReprojErrVecTmp, ReprojErrVecRTmp, ReprojErrVecTmpMatT, ReprojErrVecRTmpMatT,ValidFeatFlag,ValidFeatFlag0,ValidFeatFlagFusion,ValidFeatFlagQuant,ProbReprojVecAll, ProbReprojVecRAll, PixKeyVecTmp, PixKeyVecRTmp, Probb, ...
                                                    validFeatFlagFusion,validFeatFlag, ProbReprojVec, ProbReprojVecR,bb,b,pixKeyVecTmpXCoordMatStack,pixKeyVecTmpYCoordMatStack,pixKeyVecRTmpXCoordMatStack,pixKeyVecRTmpYCoordMatStack]...
                                                    = ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, pt2dCur, pt2dCurR,ProbZ, ProbZTmp_norm, imgCur, imgCurR,thetaSamp);
                                                
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
                                                    else
                                                        thetaSampReplay = thetaSamp;
                                                        thetaProb = interp1(round(rad2deg(thetaSamp0),3),thetaProbOnlyP2,round(rad2deg(thetaSamp) ,3));
                                                        thetaProb(isnan(thetaProb)) = min(thetaProbOnlyP2);
                                                        
                                                        if 0
                                                            %                                                 figure, plot(rad2deg(thetaSamp0), thetaProb0);hold on;plot(rad2deg(thetaSamp), thetaProb,'-x');legend('orig','interp')
                                                            figure, plot(rad2deg(thetaSamp0), thetaProbOnlyP2);hold on;plot(rad2deg(thetaSamp), thetaProb,'-x');legend('orig','interp')
                                                        end
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
                                            = ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, pt2dCur, pt2dCurR,ProbZ, ProbZTmp_norm, imgCur, imgCurR,thetaSamp);
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
                                
                                
                                
                                
                                
                                figure(19),%clf,%subplot(1,2,1);
%                                 plot(rad2deg(thetaSamp), thetaProb);hold on;plot(rad2deg(thetaSamp(find(thetaProb1 > 0.3))), thetaProb(find(thetaProb1 > 0.3)), '.b');grid on;
                                plot(rad2deg(thetaSamp), thetaProb);hold on;plot(rad2deg(thetaSamp(idM_Vec00)), thetaProb(idM_Vec00), '.r');
                                plot(rad2deg(angOpt3 - k2cRef0), max(thetaProb), 'ob');
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
                                ValidFeatFlagAll = ones(size(ValidFeatFlagAll));
                            end
                            
                            if 0
                                figure,plot(thetaProbAll(:,1,1))
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
                                    
                                    
                                    [thetaProb,idM,angOpt, ReprojErrVecTmp, ReprojErrVecRTmp, ReprojErrVecTmpMatT, ReprojErrVecRTmpMatT,ValidFeatFlag,ValidFeatFlag0,ValidFeatFlagFusion,ValidFeatFlagQuant,ProbReprojVecAll, ProbReprojVecRAll, PixKeyVecTmp, PixKeyVecRTmp, Probb, ...
                                        validFeatFlagFusion,validFeatFlag, ProbReprojVec, ProbReprojVecR,bb,b,pixKeyVecTmpXCoordMatStack,pixKeyVecTmpYCoordMatStack,pixKeyVecRTmpXCoordMatStack,pixKeyVecRTmpYCoordMatStack]...
                                        = ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, pt2dCur, pt2dCurR,updatedProbZSum0./max(updatedProbZSum0(:)), ProbZTmp_norm, imgCur, imgCurR,thetaSamp);
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
                                        figure(20),subplot(2,1,2);hold on;plot(rad2deg(thetaSamp), thetaProbInterp);grid on;
                                        plot(rad2deg(angOpt4 - k2cRef0), max(thetaProbInterp),'or');
                                    else
                                        figure(20),subplot(2,1,2);hold on;plot(rad2deg(thetaSamp), thetaProb);grid on;title('2nd theta from p2 and z');
                                        plot(rad2deg(angOpt4 - k2cRef0), max(thetaProb),'or');
                                        
                                    end
                                    saveas(gcf,fullfile(probPath,sprintf(strcat('cutThetaDiff2_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('cutThetaDiff2_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                                    %                                     saveas(gcf,fullfile(probPath,sprintf(strcat('cutThetaDiff2_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.fig'),length(dir(fullfile(probPath,strcat('cutThetaDiff2_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.fig'))))+1)));
                                end
                                
                                figure(15)
                                if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                    probDir = probPath;
                                    clf;
                                    obj.twoOnlyP2List = [0 0 ];
                                end
                                obj.twoOnlyP2List = [obj.twoOnlyP2List; [rad2deg([angOpt3 angOpt4] - [k2cRef0 k2cRef0])] ];
                                
                                figure(15);hold on;plot(obj.twoOnlyP2List(:,1),'-xr');plot(obj.twoOnlyP2List(:,2),'-xb');legend('1st theta from only p2','2nd theta from p2 and z');title('ang - ref (deg)');
                                saveas(gcf,fullfile(probPath,sprintf(strcat('twoOnlyP2List_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('twoOnlyP2List_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                                
                                
                                
                                
                                
%                                 [thetaProbUse, thetaProbUse_, thetaProbUse__, curPredPtIcs, weightRaw, candX, candY,candX_reshape, candY_reshape, weightRaw_reshape, idff] = UpdateNewTracking(obj, thetaSamp, thetaProb, prevFeatPtList0, ptCcsZ0, intrMat, AngleModalOrg, DepthProbility, DepthId, inlierId);
                                
                                
                                
                                
                                
                                
                                if 0  % 20191108  change update p2
                                    [thetaProbUse, thetaProbUse_, thetaProbUse__, curPredPtIcs, weightRaw, candX, candY,candX_reshape, candY_reshape, weightRaw_reshape, idff] = UpdateNewTracking(obj, thetaSamp, thetaProb, prevFeatPtList0, ptCcsZ0, intrMat, AngleModalOrg, DepthProbility, DepthId, inlierId);
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
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                if UPDATEZ_2
                                    
                                    if 0
                                        thetaProbUse2 = thetaProb(ismember(round(rad2deg(thetaSamp),3),round(s.configParam.angleRange ,3)));
                                    else
                                        %                                         thetaProbUse2 = interp1(round(rad2deg(thetaSamp),3), thetaProbInterp,round(s.configParam.angleRange ,3));
                                        if 0
                                            thetaProbUse2 = interp1(round(rad2deg(thetaSamp),3), thetaProbInterp,round(rad2deg(thetaSampReplay) ,3));
                                            
                                            thetaProbUse2(isnan(thetaProbUse2)) = min(thetaProbInterp);
                                        else
                                            
                                            thetaProbUse2 = interp1(round(rad2deg(thetaSamp),3), thetaProb,round(rad2deg(thetaSampReplay) ,3));
                                            
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
                                    
                                    ProbZAll = repmat(updatedProbZSum__(:),length(thetaProb),1);
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
                            else
                                angOptFinalP2 = angOpt3;
                                
                            end
                            
                            
                            
                            
                            
                            
                            
                            
                            
                            
                            
                            
                            
                            errZSum = sum(updatedProbZ(:,2,3)) - updatedProbZSum(2,3);
                            
                            figure(40),clf;subplot(2,1,1),plot(ProbZ'./max(ProbZ(:)));title('orig ProbZ');
                            subplot(2,1,2),plot(updatedProbZSum'./max(updatedProbZSum(:)));%title('updated ProbZ');
                            title(sprintf('updated ProbZ\nsigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f',obj.configParam.reproj_sigma,obj.configParam.reproj_beta,obj.configParam.reproj_sigma_right,obj.configParam.reproj_beta_right));
                            saveas(gcf,fullfile(probPath,sprintf(strcat('zProb_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('zProb_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                                
                            
                            
                            
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
                            hold on;plot(rad2deg(thetaSamp), thetaProb);plot(rad2deg(thetaSamp(idM)), thetaProb(idM),'*r'); 
                            hold on;plot(rad2deg(thetaSamp([ind1; ind2])), thetaProb([ind1; ind2]),'*b');
                            plot(rad2deg(thetaSamp([idM_Vec00(1); idM_Vec00(end)])), thetaProb([idM_Vec00(1); idM_Vec00(end)]),'*g');
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
                            
                            if jkl == 1
                                if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                                    obj.refAngList = [];
                                    obj.thetaPlatform = [];
                                    obj.thetaPlatformDistribution = [];
                                end
                            end
                            
                            if jkl == 1
                                obj.refAngList = [obj.refAngList; (k2cRef0)];
                                obj.refAngList3 = [obj.refAngList3; (k2cRef0)];
                                obj.thetaPlatform = [obj.thetaPlatform; mean(idM_Vec00)./((length(thetaSamp)+1)/2)];
                                obj.thetaPlatformDistribution = [obj.thetaPlatformDistribution; thetaPlatformDistribution_];
                            end
                            
                            
                            figure(30),clf;subplot(1,2,1);plot(obj.thetaPlatform,'-xb');title('platform midpoint ratio');
                            subplot(1,2,2);plot(obj.thetaPlatformDistribution(:,1),'-xb');hold on;plot(obj.thetaPlatformDistribution(:,2),'-xr');title('ref theta distribution');legend('zero')
                            if 1 % size(obj.featPtManager.localTrace.ptIcsX, 2) <= 3
                                saveas(gcf,fullfile(probPath,sprintf(strcat('platform_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('platform_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
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
                            
                            updatedProbZSum_ = (max(ProbZ(:))/max(updatedProbZSum(:))).*updatedProbZSum;
                            updatedProbZSum_(isnan(updatedProbZSum_)) = 0;
                            
                            
                            
                            [depthGTInd_update, ProbZTmpTmp_update_norm, ProbZTmp_update_norm, idMax_update, depthGTIndAll_update, depthUpdateIndAll_update] = VisualLocalizer.GetDepthHist(updatedProbZSum_,disparityErrorRound,obj.configParam.disparity_sample_step);
                            
                            platformRatio = sum((ProbZTmp_update_norm > 0.99)')./sum((ProbZTmp_norm > 0.99)');
                              
                            DispRngUse = repmat(dispList(inlierId,:),1,length(disparityRng)) + repmat(disparityRng,length(inlierId),1);
                            DispRefine = DispRngUse(depthUpdateIndAll_update);
                            [~, idMax_update_check] = ind2sub(size(DispRngUse), depthUpdateIndAll_update);
                            zPeakHeight_update = updatedProbZSum_(depthUpdateIndAll_update);
                            
                            ProbZTmp_update_norm_sum = ProbZTmp_update_norm ./ (repmat(sum(ProbZTmp_update_norm')',1,size(ProbZTmp_update_norm,2)));
                            ProbZTmp_update_norm_sum_tmp = zeros(size(ProbZTmp_update_norm_sum));
                            if 0
                                ProbZTmp_update_norm_sum_tmp(depthUpdateIndAll_update) = ProbZTmp_update_norm_sum(depthUpdateIndAll_update);
                                ProbZTmp_update_norm_sum_tmp(depthUpdateIndAll_update) = 1;
                            else
                                ProbZTmp_update_norm_sum_tmp(depthGTIndAll_update) = ProbZTmp_update_norm_sum(depthGTIndAll_update);
                                ProbZTmp_update_norm_sum_tmp(depthGTIndAll_update) = 1;
                            end
                            if 1
                                figure(28),clf;plot(disparityRng, sum(ProbZTmp_update_norm_sum));hold on;plot(disparityRng,sum(ProbZTmp_update_norm_sum_tmp));legend('refined disp','gt disp');
                                saveas(gcf,fullfile(probPath,sprintf(strcat('depthDistri_',probDir(end-14:end),'____',sprintf('%04d',sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('depthDistri_',probDir(end-14:end),'____',sprintf('%04d',(sum(obj.keyFrameFlagList))),'__*.png'))))+1)));

                                
                                
                            end
                            
                            [~,goodGtId] = sort(ProbZTmpTmp_update_norm,'descend');
                            [~,goodOptId] = sort(zPeakHeight_update,'descend');
                            nN = length(goodOptId);
                            pt2dCurUse = pt2dCur(goodOptId(1:nN),:);
                            PixUse = Pix(goodOptId(1:nN),:);
                            DispRefineUse = DispRefine(goodOptId(1:nN));
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
                        figure(77),clf,subplot(3,2,1);hist([areaInfo],50);legend('area','peak','Location','southwest');title('newFeatArea / oldFeatArea');subplot(3,2,2);hist(areaInfo(:,2),50);title('newMaxFeat / oldMaxFeat');
                        subplot(3,2,3);plot(trackingError(id123,end));title('norm(trackingXY - gtXY)'); subplot(3,2,4);plot(abs(pt2dCur(id123,1) -pixGT(id123,1)));title('norm(trackingX - gtX)');
                        subplot(3,2,5);plot(abs(dispList(inlierId(id123)) - dispGTTmp(inlierId(id123))));title('norm(stereoDisp - gtDisp)');subplot(3,2,6);plot(abs(pt2dCur(id123,2) -pixGT(id123,2)));title('norm(trackingY - gtY)');
                        
                        
                        
                        saveas(gcf,fullfile(probPath,sprintf(strcat('areaDrop_',probDir(end-14:end),'____',sprintf('%04d',sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('areaDrop_',probDir(end-14:end),'____',sprintf('%04d',(sum(obj.keyFrameFlagList))),'__*.png'))))+1)));

                        
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
                            
                            obj.featPtManager.localTrace.probZ(inlierId,:) = updatedProbZSum_;
                            %                 obj.keyProbZ = [obj.keyProbZ; [{sum(obj.keyFrameFlagList)} {inlierId} {ProbZ} {updatedProbZSum_} {2.^size(obj.featPtManager.localTrace.ptIcsX,2)}]];
                            obj.keyProbZ = [obj.keyProbZ; [{sum(obj.keyFrameFlagList)} {inlierId} {ProbZ./max(ProbZ(:))} {updatedProbZSum_./max(updatedProbZSum_(:))} {(max(ProbZ(:))/max(updatedProbZSum(:)))} {pt2dCur} {pixGT} {trackingError} {Pix} {zList} {dispList} {dispGTTmp} {dispCurList} {pt2dCurR} {dispMapCurGTList} {pixGTR} {depthListGT} {XYZ} {k2cRef0} {k2cPnp} {[ProbZTmpTmp_norm ProbZTmpTmp_update_norm]} {thetaPlatformDistribution_} {areaInfo} {ProbZTmp_update_norm}]];
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
        function [depthGTInd, ProbZTmpTmpNormUse, ProbZTmpNorm, peakIdTmp, depthGTIndAll, depthUpdateIndAllUse] = GetDepthHist(ProbZ,disparityErrorRound,disparity_sample_step)
            
            
            [maxFeatZ, idMax] = max(ProbZ');
            
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
            
            % %                         ProbZ = ProbZTmp_norm;
            
            depthGTOfst = round(-(disparityErrorRound)./disparity_sample_step);
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
    methods(Static)
        function [angle_result ] = AnglePredict(p1,p2,tx, ty, tz)
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
        function [curPredPtIcs,inTraceFlag,curPredPtIcs0,inTraceFlag0] = NewTracking_new(obj,keyFeatNum,prevFeatPtList,ptCcsZ,intrMat,k2cRef,angleModalOrg, keyDepthV, prevDepthV, currDepthV,DepthProbility, DepthId, AngleProbility,reCalc)
            
            k2cRef0 = k2cRef;
            k2cRef = deg2rad(round(rad2deg(k2cRef),1));
            
            
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
                angleOrgin = rad2deg(obj.refAngList3(end));
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
                angleOrgin = rad2deg(k2cRef - obj.refAngList3(end));
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
                table = LookUpTables.GenerateTable(table,obj.setting.configParam,existPt1,existFeaturesNumber, imgSize,T_B2C,fbConst,intrMat,DepthProbility, DepthId, existIdx, AngleProbility) ;
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
        function [curPredPtIcs,inTraceFlag,curPredPtIcs0,inTraceFlag0,weightRaw,candX,candY] = NewTracking(obj,keyFeatNum,prevFeatPtList,ptCcsZ,intrMat,k2cRef,angleModalOrg, keyDepthV, prevDepthV, currDepthV,DepthProbility, DepthId, AngleProbility,reCalc)
            
            inlierId = currDepthV;
            
            
            k2cRef0 = k2cRef;
            k2cRef = deg2rad(round(rad2deg(k2cRef),1));
            
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
                angleOrgin = rad2deg(obj.refAngList3(end));
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
                angleOrgin = rad2deg(k2cRef - obj.refAngList3(end));
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
            
            
            
            
            obj.setting.configParam.disparityOrginRange   = existPtDisparityG + obj.setting.configParam.disparityRange;
            
%             table = LookUpTables.LookUpTable(obj.lookUpTables,existFeaturesNumber,obj.setting.configParam.tableSize); %initialize
            table = LookUpTables.LookUpTable(existFeaturesNumber,obj.setting.configParam.tableSize); %initialize
            table.winSize = obj.lookUpTables.winSize;
            table.angSize = obj.lookUpTables.angSize;
            table.angMove = obj.lookUpTables.angMove;
            if 1 %~OnlyP2
                table = LookUpTables.GenerateTable(table,obj.setting.configParam,existPt1,existFeaturesNumber, imgSize,T_B2C,fbConst,intrMat,DepthProbility, DepthId, existIdx, AngleProbility) ;
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
            table.winSize = obj.lookUpTables.winSize;
            table.angSize = obj.lookUpTables.angSize;
            table.angMove = obj.lookUpTables.angMove;
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
                table = LookUpTables.RecalculateTableGauss(table,obj.setting.configParam,tracePt,existFeaturesNumber);
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
            
            
            
            
            
            
            
            
            
            
            
            
            cfgParam = Configurable.SetField(cfgParam, 'reproj_sigma0', 1.5*2);
            cfgParam = Configurable.SetField(cfgParam, 'reproj_sigma', 1.5*2); % 1/3
            cfgParam = Configurable.SetField(cfgParam, 'reproj_beta', 40);
            
            cfgParam = Configurable.SetField(cfgParam, 'reproj_sigma_right0', 1.5*2);
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