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
            if size(obj.poseWcsList,1) == 257
                doInsert = 2;
            end
            
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
            global  DRAWPOLYGON ANGLEONLY SHOWPOLYGON USEGOLDENDISP
            %             intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,1);
            intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,obj.scaleLvl);
            [prevFeatPtList, ptCcsZ] = GetActiveFeatures(obj.featPtManager);
            if ~isempty(prevFeatPtList) && ~all(ptCcsZ<0)
                [curPredPtIcs, inTrackFlag] = TrackFeaturePoints(obj.featPtTracker, obj.prevImgL, obj.currImgL, prevFeatPtList, [], intrMat, GetPctBody2Cam(obj.coordSysAligner));
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
                        if size(obj.poseWcsList,1) == 2
                            k2cBodySensorRotAng = -0.011777263303185;
                        end
                        k2cRef = k2cBodySensorRotAng;
                        ind = GetLastKeyFrameInd(obj);
                        try
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
                                            [angOpt1, inlierIdNew,  angRng1, pixKey ,pt2dCur, DispRefine] = RefineZ(obj, k2cRef,k2cBodyRotAng, inlierId,inlierId2, theta);  %, keyCcsXYZAll);
                                            thetaPnP = pnpLite(obj.pureRotationAngPredictor,intrMat,obj.featPtManager,obj.coordSysAligner,k2cRef,obj.keyFrameFlagList,obj.frmStampList,obj.goldenPose,pixKey ,pt2dCur, DispRefine, obj.camModel, obj.scaleLvl);
                                        catch
                                            wblnk = 1;
                                        end
                                    else
                                        angOpt1 = 0.02;
                                        angRng1 = [-0.01 0.03];
                                        thetaPnP = 0.01;
                                    end
%                                     thetaPnP = pnpLite(obj.pureRotationAngPredictor,intrMat,obj.featPtManager,obj.coordSysAligner,k2cRef,obj.keyFrameFlagList,obj.frmStampList,obj.goldenPose,pixKey ,pt2dCur, DispRefine, obj.camModel, obj.scaleLvl);
                                    obj.angOpt = [obj.angOpt; [angOpt1]];
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
                                    obj.featPtManager.localTrace.xGT(inlierId(validXYZAll),end) = pix0_GT(:,1);
                                    %                                     obj.featPtManager.localTrace.xGT(inlierId,end) = pix0_GT_(:,1);
                                    
                                    % % % % % % % % % % % % % %                                     obj.featPtManager.localTrace.yGT = [obj.featPtManager.localTrace.yGT repmat(-1,size(obj.featPtManager.localTrace.yGT,1),1)];
                                    obj.featPtManager.localTrace.yGT(inlierId(validXYZAll),end) = pix0_GT(:,2);
                                    %                                     obj.featPtManager.localTrace.yGT(inlierId,end) = pix0_GT_(:,2);
                                end
                                %                                 obj.featPtManager.localTrace.ptIcsX(setdiff((1:size(obj.featPtManager.localTrace.ptIcsX,1))',validId),keyLength) = -1;
                                %                                 obj.featPtManager.localTrace.ptIcsY(setdiff((1:size(obj.featPtManager.localTrace.ptIcsX,1))',validId),keyLength) = -1;
                                inLierFlag = false(length(activeFeatIndLast),1);
                                inLierFlag(validId) = true;
                                drawnow;
                            end
                            
                        catch
                            
                            
                            
                            theta = k2cBodySensorRotAng;
                            
                            
                            
                            obj.angOpt = [obj.angOpt; [theta]];
                            obj.angOptPnP = [obj.angOptPnP; [theta]];
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
        
        function [angOpt, inlierId, angRng, PixUse, pt2dCurUse, DispRefineUse] = RefineZ(obj, k2cRef,k2cPnp, inlierId0, inlierId2, theta)  %, XYZ)
            global SHOWPOLYGON DRAWPOLYGON ANGLEONLY USEGOLDENDISP UPDATEDEPTH DEPTHITERNUM probPath ...
                USELEFT USELEFTPLUSRIGHT USELEFTXRIGHT USERIGHT WITHINEPILINE BOTHLR HARDCUT QUANTCUT FUSIONCUT ...
                FORCEEVENPLATFORM CUTTHETAANDFORCEPLATFORM DIFFTHETATHR CUTTHETAANDFORCEPLATFORM_NODIFF FORCEEXPEVEN ...
                EXPZERO GTTRACKING SOFTP2
            
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
            dispMatGT(dispMatGT < 0.9) = 0.9;
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
            reprojError = [];cntt = 0;PixCur = [];PixReproj = [];
            validId = []; validXYZAll = []; Pix0 = []; vldReproj = [];
            keyLength = size(obj.featPtManager.localTrace.ptIcsX,2);
            
            dispGTTmp = dispMatGT(validInd);
            
            idUnValid = find(isnan(zList) | isinf(zList) | isnan(dispList) | isinf(dispList) | isnan(dispCurList_)); % | isnan(dispGTTmp) | isinf(dispGTTmp));
            inlierId = setdiff(inlierId0, idUnValid);
            
            
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
            
            
            if GTTRACKING
                
                b2cPmat = GetPctBody2Cam(obj.coordSysAligner, 1);
                if 0
                    k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(thetaRng(idM)),zeros(3,1));
                else
                    k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cRef),zeros(3,1));
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
                        pt2dCur =  pixGT;
                    end
                    
                    dispCurList = dispCurList_(inlierId);
                    
                    if ~GTTRACKING
                        pt2dCurR = [pt2dCur(:,1) - dispCurList pt2dCur(:,2)];
                    else
                        pt2dCurR = pixGTR;
                    end
                    validIndCur = sub2ind(size(dispMapKey), round(pt2dCur(:,2)), round(pt2dCur(:,1)));
                    dispErrCur = dispCurList  - dispMapCurGTList;
                    
                    
                    %                     pixGTR = [pixGT(:,1) - dispMapCurGTList pixGT(:,2)];
                    
                    
                    
                    %                                     figure(232);hold on;axis equal;
                    if 1
                        if isempty(obj.featPtManager.localTrace.probZ)
                            DispRng = repmat(dispList(inlierId,:),1,length(disparityRng)) + repmat(disparityRng,length(inlierId),1);
                            
                            
                            
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
                            
                            
                            
                            [maxFeatZ,idMax] = max(ProbZ');
                            maxFeatZ = maxFeatZ';
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
                                
                                
                                pixKeyVecRTmpXCoord = pixKeyVecRTmp(:,1);
                                pixKeyVecRTmpYCoord = pixKeyVecRTmp(:,2);
                                pixKeyVecRTmpXCoordMat = reshape(pixKeyVecRTmpXCoord, size(ProbZ));
                                pixKeyVecRTmpYCoordMat = reshape(pixKeyVecRTmpYCoord, size(ProbZ));
                                pixKeyVecRTmpXCoordMatT = pixKeyVecRTmpXCoordMat';
                                pixKeyVecRTmpYCoordMatT = pixKeyVecRTmpYCoordMat';
                                
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
                                            if ismember(idM, idxCell{edc})
                                                edc0 = edc;
                                                break;
                                            end
                                            
                                        end
                                        
                                    end
                                    
                                    
                                    for edc = 1 : length(idxCell)
                                        if ismember(idM, idxCell{edc})
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
                                    
                                     rati = 0.8;
                                     thetaExp_ = dot(rad2deg(thetaSamp), thetaProb);
                                     thetaProb1 = thetaProb./max(thetaProb);
                                     thetaProb2 = thetaProb1(thetaProb1 > rati);
                                     thetaProb2 = thetaProb2./sum(thetaProb2);
                                     thetaExp_2 = dot(rad2deg(thetaSamp(thetaProb1 > rati)), thetaProb2); % 0.5; 0.8; 0.0
                                     
                                     if EXPZERO
                                         thetaExp_2 = 0;
                                     end
                                     
                                     
                                     
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
                                            sc = max([sc 4]);
                                         if ~SOFTP2
                                             pltfrm_ = [thetaExp_2 - min(110.2,rad2deg(sc*obj.configParam.theta_sample_step)) thetaExp_2 + min(110.2,rad2deg(sc*obj.configParam.theta_sample_step))];
                                         else
                                             pltfrm_ = [thetaExp_2 - min(0.2,rad2deg(sc*obj.configParam.theta_sample_step)) thetaExp_2 + min(0.2,rad2deg(sc*obj.configParam.theta_sample_step))];
                                         end
                                     end
                                     pltfrm_1 = [deg2rad(pltfrm_(1)) : obj.configParam.theta_sample_step : deg2rad(pltfrm_(end))] + k2cRef;
%                                     thetaRng = pltfrm_1;
%                                     thetaSamp = thetaRng - k2cRef;
                                    
                                    idM_Vec00 = [minDDId(1) : minDDId(2)];
                                    asba = 1;
                                    
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
                                
                                
                                
                                
                                thetaProb(idM_Vec00) = max(thetaProb);
                                thetaProb = thetaProb./sum(thetaProb);
                                
%                             else
%                                 idM_Vec11 = find(thetaProb./max(thetaProb) > obj.configParam.theta_prob_cutoff_threshold) ;
%                                 idM_Vec11 = find(thetaProb./max(thetaProb) > obj.configParam.theta_prob_cutoff_threshold) ;
%                                 figure(19),clf;plot(rad2deg(thetaSamp), thetaProb);hold on;plot(rad2deg(thetaSamp(idM_Vec11)), thetaProb(idM_Vec11), '.r');grid on;
                            end
                            
                            
                            if FORCEEXPEVEN
                                expandNum = obj.configParam.expectation_theta_expand_num;
                                thetaRng = [(-obj.configParam.theta_sample_step.*[expandNum:-1:1] + pltfrm_1(1)) pltfrm_1 (pltfrm_1(end) + obj.configParam.theta_sample_step.*[1 : expandNum])];
                                
                                thetaSamp = thetaRng - k2cRef;
                                %                                 ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, pt2dCur, pt2dCurR,ProbZ,ProbZTmp_norm,imgCur,imgCurR, thetaSamp);
                                [thetaProb,idM,angOpt, ReprojErrVecTmp, ReprojErrVecRTmp, ReprojErrVecTmpMatT, ReprojErrVecRTmpMatT,ValidFeatFlag,ValidFeatFlag0,ValidFeatFlagFusion,ValidFeatFlagQuant,ProbReprojVecAll, ProbReprojVecRAll]...
                                    = ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, pt2dCur, pt2dCurR,ProbZ, ProbZTmp_norm, imgCur, imgCurR,thetaSamp);
                                
                                
                                
                                thetaProb0 = thetaProb;
                                idM_Vec00 = find(ismember(thetaRng, pltfrm_1));
                                idM_Vec00Diff = idM_Vec00;
                                figure(19),%clf,%subplot(1,2,1);
%                                 plot(rad2deg(thetaSamp), thetaProb);hold on;plot(rad2deg(thetaSamp(find(thetaProb1 > 0.3))), thetaProb(find(thetaProb1 > 0.3)), '.b');grid on;
                                plot(rad2deg(thetaSamp), thetaProb);hold on;plot(rad2deg(thetaSamp(idM_Vec00)), thetaProb(idM_Vec00), '.r');grid on;
                                %                                 subplot(1,2,2);plot(rad2deg(thetaSamp), thetaProb);hold on;plot(rad2deg(thetaSamp(idM_Vec11)), thetaProb(idM_Vec11), '.r');grid ongrid on;
                                if 1 %size(obj.featPtManager.localTrace.ptIcsX, 2) <= 3
                                    saveas(gcf,fullfile(probPath,sprintf(strcat('cutThetaDiff_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('cutThetaDiff_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                                end
                                
                                
                                
                                
                                
                                
                                thetaProb(idM_Vec00) = max(thetaProb);
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
                            figure(31),hold on;plot(rad2deg(thetaSamp), thetaProb);plot(rad2deg(thetaSamp(idM)), thetaProb(idM),'*r'); % title(sprintf('k2cRef: %0.5f\nk2cPnP: %0.5f\nk2cProb: %0.5f\nsigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f',rad2deg(k2cRef), rad2deg(k2cPnp), rad2deg(thetaRng(idM)),obj.configParam.reproj_sigma,obj.configParam.reproj_beta,obj.configParam.reproj_sigma_right,obj.configParam.reproj_beta_right));grid on;
                            %                             title(sprintf('sigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f',obj.configParam.reproj_sigma,obj.configParam.reproj_beta,obj.configParam.reproj_sigma_right,obj.configParam.reproj_beta_right));
                            
                            
                            b2cPmat = GetPctBody2Cam(obj.coordSysAligner, 1);
                            if 0
                                k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(thetaRng(idM)),zeros(3,1));
                            else
                                k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cRef),zeros(3,1));
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
                            
                            
                            
                            
                            tmpProb22 = permute(Probb(idM,:,:),[2 3 1]);
                            maxx2 = (max(tmpProb22'));
                            maxx2(isnan(maxx2) | isinf(maxx2)) = [];
                            maxxsum2 = sum((maxx2));
                            %                            figure(33),clf;subplot(2,1,1);plot(ZZVec1(randId,:),ProbZ(randId,:));hold on;plot(ZZVec1(randId,:),probReprojVec(randId,:));
                            %                                           subplot(2,1,2);plot(ZZVec1(randId,:),tmpProb(:)');
                            
                            if 0
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
                            thetaProbAll = repmat(thetaProb', size(ProbZ,1)*size(ProbZ,2),1);
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
                                thetaProbAll = repmat(thetaProb', size(ProbZ,1)*size(ProbZ,2),1);
                                thetaProbAll = reshape(thetaProbAll, length(thetaProb), size(ProbZ,1), size(ProbZ,2));
                            
                            else
                                asvdj = 1;
                            end
                            
                            if DIFFTHETATHR
                                ValidFeatFlag = zeros(size(BB));
                                ValidFeatFlag(idM_Vec00Diff,:) = 1;
                            
                            end
                            
                            figure(3),clf;imshow(ValidFeatFlag, [])
                            saveas(gcf,fullfile(probPath,sprintf(strcat('cutTheta_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('cutTheta_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                            
                            
                            
                            
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
                            
                            errValidFeatAll = abs(ValidFeatFlagAll(:,:,1) - ValidFeatFlagAll(:,:,end));
                            errValidFeat = abs(ValidFeatFlagAll0(:,:,1) - ValidFeatFlagAll0(:,:,end));
                            errValidFeat2 = abs(ValidFeatFlagAll00(1,:,:) - ValidFeatFlagAll00(end,:,:));
                            errValidFeat2 = permute(errValidFeat2, [2 3 1]);
                            
                            
                            ProbReprojVecAll_New = ProbReprojVecAll.*ValidFeatFlagAll;
                            ProbReprojVecRAll_New = ProbReprojVecRAll.*ValidFeatFlagAll;
                            ProbReprojVecLRAll_New = ProbReprojVecAll.*ProbReprojVecRAll.*ValidFeatFlagAll;
                            
                            thetaProbAll_New = thetaProbAll.*ValidFeatFlagAll;
                            
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
                            figure(31),hold on;plot(rad2deg(thetaSamp([ind1; ind2])), thetaProb([ind1; ind2]),'*b');
                            plot(rad2deg(thetaSamp([idM_Vec00(1); idM_Vec00(end)])), thetaProb([idM_Vec00(1); idM_Vec00(end)]),'*g');
                            if ~FORCEEXPEVEN
                                title(sprintf('p(theta) cutoff threshold: %0.5f\nmean theta ind: %0.2f / %0.2f\nk2cRef: %0.5f\nk2cPnP: %0.5f\nk2cProb: %0.5f\nsigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f',obj.configParam.theta_prob_cutoff_threshold,mean(idM_Vec00),(length(thetaSamp)+1)/2,rad2deg(k2cRef), rad2deg(k2cPnp), rad2deg(thetaRng(idM)),obj.configParam.reproj_sigma,obj.configParam.reproj_beta,obj.configParam.reproj_sigma_right,obj.configParam.reproj_beta_right));grid on;
                            else
                                title(sprintf('p(theta) cutoff threshold: %0.5f\nmean theta ind: %0.2f / %0.2f\nk2cRef: %0.5f\nk2cPnP: %0.5f\nk2cProb: %0.5f\nsigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f',obj.configParam.theta_prob_cutoff_threshold,mean(idM_Vec00),(length(thetaSamp)+1)/2,rad2deg(k2cRef), rad2deg(k2cPnp), rad2deg(angOpt),obj.configParam.reproj_sigma,obj.configParam.reproj_beta,obj.configParam.reproj_sigma_right,obj.configParam.reproj_beta_right));grid on;
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
                                obj.refAngList = [obj.refAngList; (k2cRef)];
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
                            figure(25);clf;plot(ProbZTmpTmp_update_norm(goodOptId(1:nN)));
                            saveas(gcf,fullfile(probPath,sprintf(strcat('peakHeight_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('peakHeight_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                            figure(24);clf;hist([DispRefine DispGT DispInit], 50);legend('refined disp','gt disp','stereo disp');
                            saveas(gcf,fullfile(probPath,sprintf(strcat('dispDistri_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('dispDistri_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                            
                            try
                                [minVal,idididMin] = sort(ProbZTmpTmp_update_norm,'ascend');
                                [maxVal,idididMax] = sort(ProbZTmpTmp_update_norm,'descend');
                                %                                 checkId = inlierId([idididMin(1:5); idididMax(1:5)]);
                                checkId = ([idididMin(1:15); idididMax(1:15)]);
                                ratioMat = BB.*repmat(thetaProb'./max(thetaProb), 1,size(BB0,2));
                                ratioVec = sum(ratioMat);
                                figure(29),clf;subplot(1,2,1);plot(repmat(rad2deg(thetaSamp)',1,length(checkId)/2), BB0(:,checkId(1:length(checkId)/2)),'-g');hold on;plot(rad2deg(thetaSamp), thetaProb./max(1),'-b','LineWIdth',5);title('bad');
                                subplot(1,2,2);plot(repmat(rad2deg(thetaSamp)',1,length(checkId)/2), BB0(:,checkId(length(checkId)/2+1:end)),'-r');hold on;plot(rad2deg(thetaSamp), thetaProb./max(1),'-b','LineWIdth',5);title('good');
                                
                                
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
                        
                        if UPDATEDEPTH
                            %                                 updatedProbZSum_ = (max(ProbZ(:))/max(updatedProbZSum(:))).*updatedProbZSum;
                            obj.featPtManager.localTrace.probZ(inlierId,:) = updatedProbZSum_;
                            %                 obj.keyProbZ = [obj.keyProbZ; [{sum(obj.keyFrameFlagList)} {inlierId} {ProbZ} {updatedProbZSum_} {2.^size(obj.featPtManager.localTrace.ptIcsX,2)}]];
                            obj.keyProbZ = [obj.keyProbZ; [{sum(obj.keyFrameFlagList)} {inlierId} {ProbZ./max(ProbZ(:))} {updatedProbZSum_./max(updatedProbZSum_(:))} {(max(ProbZ(:))/max(updatedProbZSum(:)))} {pt2dCur} {pixGT} {trackingError} {Pix} {zList} {dispList} {dispGTTmp} {dispCurList} {pt2dCurR} {dispMapCurGTList} {pixGTR} {depthListGT} {XYZ} {k2cRef} {k2cPnp} {[ProbZTmpTmp_norm ProbZTmpTmp_update_norm]} {thetaPlatformDistribution_}]];
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
        function [depthGTInd, ProbZTmpTmpNorm, ProbZTmpNorm, idMax, depthGTIndAll, depthUpdateIndAll] = GetDepthHist(ProbZ,disparityErrorRound,disparity_sample_step)
            
            
            [maxFeatZ, idMax] = max(ProbZ');
            maxFeatZ = maxFeatZ';
            maxFeatZ = repmat(maxFeatZ, 1, size(ProbZ,2));
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
            cfgParam = Configurable.SetField(cfgParam, 'disparity_error', 1); %2
            cfgParam = Configurable.SetField(cfgParam, 'depth_uncertainty_range', [-100 300]);
            cfgParam = Configurable.SetField(cfgParam, 'theta_range', deg2rad([-2 2]));  % [-0.3 0.3]
            cfgParam = Configurable.SetField(cfgParam, 'polygon_margin', 2);
            cfgParam = Configurable.SetField(cfgParam, 'polygon_inlier_thresh', 0.2);
            cfgParam = Configurable.SetField(cfgParam, 'theta_sample_step', deg2rad(0.05));% 0.05 0.01
            cfgParam = Configurable.SetField(cfgParam, 'disparity_sample_step', 0.1); % 0.01 % 0.05 0.1
            cfgParam = Configurable.SetField(cfgParam, 'reproj_sample_step', 0.001);
            
            cfgParam = Configurable.SetField(cfgParam, 'disparity_sample_interval', 0.01);
            cfgParam = Configurable.SetField(cfgParam, 'reproj_sample_interval', 0.01);
            
            cfgParam = Configurable.SetField(cfgParam, 'disparity_sigma',3); %  1/3
            cfgParam = Configurable.SetField(cfgParam, 'disparity_beta',100);
            
            
            
            
            
            
            
            
            
            
            
            
            
            cfgParam = Configurable.SetField(cfgParam, 'reproj_sigma', 1.5); % 1/3
            cfgParam = Configurable.SetField(cfgParam, 'reproj_beta', 10);
            
            cfgParam = Configurable.SetField(cfgParam, 'reproj_sigma_right', 1.5); % 1/3
            cfgParam = Configurable.SetField(cfgParam, 'reproj_beta_right', 10);
            
            cfgParam = Configurable.SetField(cfgParam, 'epiLine_margin',0);
            cfgParam = Configurable.SetField(cfgParam, 'probZ_ratio_threshold',0.05);
            
            cfgParam = Configurable.SetField(cfgParam, 'theta_prob_cutoff_threshold',0.9); % 0.001
            cfgParam = Configurable.SetField(cfgParam, 'depth_prob_cutoff_threshold',0.0);
            cfgParam = Configurable.SetField(cfgParam, 'tracking_trust_radius',2);
            cfgParam = Configurable.SetField(cfgParam, 'ref_theta_trust_margin',deg2rad(0.5));
            cfgParam = Configurable.SetField(cfgParam, 'theta_diff_margin',0.005);
            cfgParam = Configurable.SetField(cfgParam, 'expectation_theta_radius', deg2rad(0.2));
            cfgParam = Configurable.SetField(cfgParam, 'expectation_theta_expand_num', 10);
            
            
            
            
            
            cfgParam = Configurable.SetField(cfgParam, 'theta_percentage', 0.6);
            cfgParam = Configurable.SetField(cfgParam, 'depth_hist_ratio', 0.95);
            
            
            
            cfgParam = Configurable.SetField(cfgParam, 'theta_sigma', 0.01);
            
            obj.configParam = cfgParam;
        end
        
        function CheckConfig(obj) %#ok<MANU>
        end
    end
end