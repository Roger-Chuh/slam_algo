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
            doInsert = NeedInsertKeyFrame(obj, refMotionState);
            obj.currMotionState = refMotionState;
            debug.debug_904x_FindDebugCase(frameInd == -1,doInsert == -1,refMotionState == -1);
            
            depthMap = GetDepthMap(obj, imgL, imgR);
            StoreImage(obj.pointCloudManager,imgL);
            StoreDepthMap(obj.pointCloudManager,depthMap);
            
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
                    obj.prevRefRobotPoseWcs = obj.currRefRobotPoseWcs;
                    obj.prevRefRobotRotAngWcs = obj.currRefRobotRotAngWcs;
%                     PlotSense(obj.pointCloudManager)
                    CheckDataConsistency(obj);
                    return;
                elseif (doInsert == obj.INSERT_KEYFRAME_CURRENT)
                    obj.keyRefRobotPoseWcs = obj.currRefRobotPoseWcs;                    
                    obj.keyRefRobotRotAngWcs = obj.currRefRobotRotAngWcs;
                    obj.keyFrameFlagList =  [obj.keyFrameFlagList; true];
                    InsertKeyFrame(obj, obj.currImgL, depthMap);
%                     MergePointMap(obj);
                    obj.prevImgL = obj.currImgL;
                    obj.prevImgR = obj.currImgR;
                    obj.prevRefRobotPoseWcs = obj.currRefRobotPoseWcs;
                    obj.prevRefRobotRotAngWcs = obj.currRefRobotRotAngWcs;
%                     PlotSense(obj.pointCloudManager)
                    CheckDataConsistency(obj);
                    debug.debug_910x_ShowKeyFrameLocation(obj,doInsert)
                    return;
                elseif (doInsert == obj.INSERT_KEYFRAME_PREVIOUS)
                    obj.keyRefRobotPoseWcs = obj.prevRefRobotPoseWcs;
                    obj.keyRefRobotRotAngWcs = obj.prevRefRobotRotAngWcs;
                    depthMap = GetDepthMap(obj, obj.prevImgL, obj.prevImgR);
                    InsertKeyFrame(obj, obj.prevImgL, depthMap);
                    obj.keyFrameFlagList(end) = true;
%                     MergePointMap(obj)
%                     StoreImage(obj.pointCloudManager,obj.prevImgL)
%                     ExtendPointCould(obj);
                    obj.keyFrameFlagList = [obj.keyFrameFlagList; false];
                    obj.prevImgL = obj.currImgL;
                    obj.prevImgR = obj.currImgR;
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
                %                     MergePointMap(obj);
                obj.prevImgL = obj.currImgL;
                obj.prevImgR = obj.currImgR;
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
                depthMap = GetDepthMap(obj, obj.prevImgL, obj.prevImgR);
                InsertKeyFrame(obj, obj.prevImgL, depthMap);
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
%             figure(1122), UpdateLocalTracePlot(obj.featPtManager, obj.currImgL);
            figure(1122), title(sprintf('Frame %d', frameInd));
            %             end
            
            %             global PATH_FIG
            %             if ~isempty(PATH_FIG)
            %                 figure(PATH_FIG), UpdateTrajectoryPlot(obj);
            %             end
            
            debug.debug_911x_CheckRotationAngle(obj,frameInd)
            ExtendPointCloud_OpticalFlow1D(obj);
            
            obj.prevImgL = obj.currImgL;
            obj.prevImgR = obj.currImgR;
            obj.prevRefRobotPoseWcs = obj.currRefRobotPoseWcs;
            obj.prevRefRobotRotAngWcs = obj.currRefRobotRotAngWcs;
%             PlotSense(obj.pointCloudManager)
            
            CheckDataConsistency(obj);
            debug.debug_910x_ShowKeyFrameLocation(obj,doInsert)
        end
        
        % Member functions
        function b = NeedInsertKeyFrame(obj, refMotionState)
            
            
            
            
            
            
            if (GetPrevMotionState(obj) == obj.MOTIONSTATE_UNKNOWN)
                if (refMotionState ~= obj.MOTIONSTATE_STILL)
                    b = obj.INSERT_KEYFRAME_CURRENT;
                else
                    b = obj.INSERT_KEYFRAME_NO;
                end
                return;
            end
            
            assert(~isempty(obj.keyFrameFlagList), 'For non very beginning frame, the keyFrameFlagList should not be empty');
            if (GetPrevMotionState(obj) ~= refMotionState)
                if (refMotionState ~= obj.MOTIONSTATE_STILL)
                    if ~obj.keyFrameFlagList(end)
                        b = obj.INSERT_KEYFRAME_PREVIOUS;
                    else
                        b = obj.INSERT_KEYFRAME_NO;
                    end
                else
                    b = obj.INSERT_KEYFRAME_CURRENT;
                end
                return;
            end
            
            %             disp(FeatureNumDropRate(obj.featPtManager))
            if (FeatureNumDropRate(obj.featPtManager) > obj.configParam.feature_num_drop_rate_thresh)
                if ~obj.keyFrameFlagList(end)
                    b = obj.INSERT_KEYFRAME_PREVIOUS;
                else
                    b = obj.INSERT_KEYFRAME_NO;
                end
                return;
            end
            
            if (FeatureNumDropRate(obj.featPtManager) ~= 0 && NumActiveFeatures(obj.featPtManager) < obj.configParam.min_feature_point_number)
                if ~obj.keyFrameFlagList(end)
                    b = obj.INSERT_KEYFRAME_PREVIOUS;
                else
                    b = obj.INSERT_KEYFRAME_NO;
                end
                return;
            end
            
            
            b = obj.INSERT_KEYFRAME_NO;
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
            intrMat1 = Get(obj.camModel, 'PinholeIntrMat', 1,1);
            intrMat2 = Get(obj.camModel, 'PinholeIntrMat', 2,1);
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
                figure(1122), UpdateLocalTracePlot(obj.featPtManager, obj.currImgL);

                %                 [~, ~, keyPtCcsZ] = GetFeaturePairs(obj.featPtManager, 'key', 'last');
                %                 if sum(inTrackFlag) > obj.configParam.min_num_pt_for_calculate &&  (Rows(prevFeatPtList) - sum(inTrackFlag))/Rows(prevFeatPtList) < 0.2
                if sum(inTrackFlag) > obj.configParam.min_num_pt_for_calculate
                    
                    
                    p2cBodySensorRotAng = obj.currRefRobotRotAngWcs - obj.prevRefRobotRotAngWcs;
                    p2cVisionRotAng = PredictRotateAngle(obj,p2cBodySensorRotAng);
                    deltaVision2BodySensor = abs(rad2deg(p2cVisionRotAng-p2cBodySensorRotAng));
                    debug.debug_900x_PredictRotateAngle(obj,intrMat,prevFeatPtList,predPtIcs, obj.DEBUG_FLAG & false)
                    debug.debug_905x_CheckVisionAndBodyP2CRotAng(deltaVision2BodySensor)
%                     obj.deltaAngle(end + 1) = rad2deg(p2cVisionRotAng-p2cBodySensorRotAng);
%                     disp('p2c Ang is')
%                     disp(deltaVision2BodySensor)
%                     figure(773377), plot(obj.deltaAngle)
                    if deltaVision2BodySensor > 0.5
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
                        SaveInLier(obj.featPtManager, 'last', activeFeatInd, validFlag);
                        fprintf(sprintf('### AngleSpace inlier ratio: %d/%d ###\n\n\n', sum(validFlag),length(validFlag)));
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
                    c2wTrans = obj.currRefRobotPoseWcs;
                    p2wTrans = obj.prevRefRobotPoseWcs;
                    c2pTrans = p2wTrans\c2wTrans;
                    p2wTransV = WcsPoseVecToPct(obj, obj.poseWcsList(end,:));
                    c2wTransV = p2wTransV*c2pTrans;
                    curloc = T(c2wTransV);
                    deltaAngle = obj.currRefRobotRotAngWcs - obj.prevRefRobotRotAngWcs;
                    robotPoseWcs = [curloc(1),curloc(3),obj.poseWcsList(end,3)+deltaAngle];
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
                deltaAngle = obj.currRefRobotRotAngWcs - obj.prevRefRobotRotAngWcs;
                robotPoseWcs = [curloc(1),curloc(3),obj.poseWcsList(end,3)+deltaAngle];
                [~, ~, activeFeatInd] = GetActiveFeatures(obj.featPtManager, 'last');
                inLierFlag = false(Rows(activeFeatInd),1);
                SaveInLier(obj.featPtManager, 'last', activeFeatInd, inLierFlag);
            end
        end
        
        function [robotPoseWcs] = PureRotationTrackAndLocalize(obj)
            intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,1);
            [prevFeatPtList, ptCcsZ] = GetActiveFeatures(obj.featPtManager);
            if ~isempty(prevFeatPtList) && ~all(ptCcsZ<0)
                [curPredPtIcs, inTrackFlag] = TrackFeaturePoints(obj.featPtTracker, obj.prevImgL, obj.currImgL, prevFeatPtList, [], intrMat, GetPctBody2Cam(obj.coordSysAligner));
                ExtendLocalTrace(obj.featPtManager, curPredPtIcs, inTrackFlag);
                figure(1122), UpdateLocalTracePlot(obj.featPtManager, obj.currImgL);
                if sum(inTrackFlag) > obj.configParam.min_num_pt_for_calculate
                    PNP_FLAG = true;
                    if PNP_FLAG
                        k2cBodySensorRotAng = obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs;
                        k2cRef = k2cBodySensorRotAng;
                        ind = GetLastKeyFrameInd(obj);
                        key2cVisionRotAng = PredictRotateAngle2(obj,k2cBodySensorRotAng,obj.frmStamp,obj.goldenPose,ind);
                        
                        
                        k2cBodyRotAng = RotationAngleEstimate_PNP(obj.pureRotationAngPredictor,intrMat,obj.featPtManager,obj.coordSysAligner,k2cRef);
                        deltaVision2BodySensor = abs(rad2deg(k2cBodyRotAng-k2cBodySensorRotAng));
                        if deltaVision2BodySensor > 20 %0.1
                            theta = k2cBodySensorRotAng;
                        else
                            theta = k2cBodyRotAng;
                        end
                        %             debug.debug_902x_PureRotationTest(theta,b2cPmat,keyCcsXYZ,intrMat,activeFlag,curPtIcs);
                        inLierFlag = DeoutLier_PNP(obj.pureRotationAngPredictor,theta, intrMat,obj.featPtManager,obj.coordSysAligner);
                        
                    else
                       
                        
                        k2cBodySensorRotAng = obj.currRefRobotRotAngWcs - obj.keyRefRobotRotAngWcs;
                        p2cBodySensorRotAng = obj.currRefRobotRotAngWcs - obj.prevRefRobotRotAngWcs;
                        
%                          p2cVisionRotAng = PredictRotateAngle2(obj,p2cBodySensorRotAng,obj.frmStamp,obj.goldenPose);
                         
                         p2cVisionRotAng = PredictRotateAngle(obj,p2cBodySensorRotAng);
                        ind = GetLastKeyFrameInd(obj);
                        k2pVisionRotAng = obj.poseWcsList(end,3) - obj.poseWcsList(ind,3);
                        k2cVisionRotAng = p2cVisionRotAng + k2pVisionRotAng;
                        deltaVision2BodySensor = abs(rad2deg(k2cVisionRotAng-k2cBodySensorRotAng));
                        if deltaVision2BodySensor > 20
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
                    [~, ~, activeFeatInd] = GetActiveFeatures(obj.featPtManager, 'last');
                  %  SaveInLier(obj.featPtManager, 'last', activeFeatInd, inLierFlag);
                  ind = GetLastKeyFrameInd(obj);
                  keyPoseWcs = obj.poseWcsList(ind,:);
                  robotPoseWcs = [keyPoseWcs(1:2), keyPoseWcs(3)+theta];
                  reTrackFlag = setdiff(find(obj.featPtManager.localTrace.ptIcsX(:,end-1) ~= -1),activeFeatInd);
                  ptPrv = [obj.featPtManager.localTrace.ptIcsX(:,end-1) obj.featPtManager.localTrace.ptIcsY(:,end-1)];
                  [curPt, flagCur,fundMat] = ReTraceEpiLine(obj, robotPoseWcs,ptPrv,obj.featPtManager.localTrace.ptIcsX(:,end-1) ~= -1);
                    reTrackId = intersect(reTrackFlag,find(flagCur));
                    
                    obj.featPtManager.localTrace.ptIcsX(reTrackId,end) = curPt(reTrackId,1);
                    obj.featPtManager.localTrace.ptIcsY(reTrackId,end) = curPt(reTrackId,2);
                    fprintf(sprintf('### retracked points: %d/%d ###\n\n\n',length(reTrackId),sum(obj.featPtManager.localTrace.ptIcsX(:,end) ~= -1)));
%                     SaveInLier(obj.featPtManager, 'last', activeFeatInd, validFlag);
                    SaveInLier(obj.featPtManager, 'last', find(obj.featPtManager.localTrace.ptIcsX(:,end) ~= -1), true(length(find(obj.featPtManager.localTrace.ptIcsX(:,end) ~= -1)),1));
          
                else
                    c2wTrans = obj.currRefRobotPoseWcs;
                    p2wTrans = obj.prevRefRobotPoseWcs;
                    c2pTrans = p2wTrans\c2wTrans;
                    p2wTransV = WcsPoseVecToPct(obj, obj.poseWcsList(end,:));
                    c2wTransV = p2wTransV*c2pTrans;
                    curloc = T(c2wTransV);
                    deltaAngle = obj.currRefRobotRotAngWcs - obj.prevRefRobotRotAngWcs;
                    robotPoseWcs = [curloc(1),curloc(3),obj.poseWcsList(end,3)+deltaAngle];
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
                deltaAngle = obj.currRefRobotRotAngWcs - obj.prevRefRobotRotAngWcs;
                robotPoseWcs = [curloc(1),curloc(3),obj.poseWcsList(end,3)+deltaAngle];
                [~, ~, activeFeatInd] = GetActiveFeatures(obj.featPtManager, 'last');
                inLierFlag = false(Rows(activeFeatInd),1);
                SaveInLier(obj.featPtManager, 'last', activeFeatInd, inLierFlag);
            end
        end
        
        function depthMap = GetDepthMap(obj, imgL, imgR)
            assert(~isempty(imgL) && ~isempty(imgR), 'Input images should not be empty.');
            [princpPtL, princpPtR] = Get(obj.camModel, 'PinholePrincpPt', obj.scaleLvl);
            [focLenL, focLenR] = Get(obj.camModel, 'PinholeFocLen', obj.scaleLvl);
            baseline = GetBaseline(obj.camModel);
            assert(all(focLenL == focLenR), 'Rectified camera pairs should have the same pinhole focal length');
            if isempty(obj.pointCloudManager.depthMaps)
                prevKeyMinDepth = -1;
            else
                intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,1);
                depthMap = obj.pointCloudManager.depthMaps(:,:,end);
                flag = Filter3DPoint(obj.pointCloudManager, depthMap, intrMat);
                prevKeyMinDepth = mean(depthMap(flag));
            end
            
            depthMap = CalcDepthMap(obj.stereoDepthMap, imgL, imgR, focLenL, princpPtL(1), princpPtR(1), baseline, prevKeyMinDepth);
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
        function [reProjErr,dis,validFlag] = CalculateReprojectError(intrMat,Point3D,Point2D,R,T,flag,outLierThreshold)
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
            dis(flag) = sqrt((reprojPix(flag,1)-Point2D(flag,1)).^2 +(reprojPix(flag,2)-Point2D(flag,2)).^2);
            validFlag(flag) = dis(flag) <= outLierThreshold;
            reProjErr = sum(dis(flag))/Rows(Point3D(flag));
        end
    end
    methods
        % Implementation of abstract methods in Configurable base class
        function SetDefaultValue(obj, cfgParam)
            cfgParam = Configurable.SetField(cfgParam, 'min_feature_point_number', 80);
            cfgParam = Configurable.SetField(cfgParam, 'feature_num_drop_rate_thresh', 0.75);
            cfgParam = Configurable.SetField(cfgParam, 'zero_translation_thresh', 0.01);
            cfgParam = Configurable.SetField(cfgParam, 'zero_rotation_angle_thresh', 0.2);
            cfgParam = Configurable.SetField(cfgParam, 'reproject_outlier_threshold', 100);
            cfgParam = Configurable.SetField(cfgParam, 'min_num_pt_for_calculate', 80);

            obj.configParam = cfgParam;
        end
        
        function CheckConfig(obj) %#ok<MANU>
        end
    end
 methods
        function [pt2, flag,FundMat,FundMat00] = ReTraceEpiLine(obj,robotPoseWcs,pt1,flag)
            if sum(flag) == 0
                pt2 = pt1;
            else
                intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,1);
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
    
end

