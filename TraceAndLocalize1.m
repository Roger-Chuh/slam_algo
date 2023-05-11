function TraceAndLocalize1()

global OUTPUT_ROOT RECORDED_BIN_ROOT DUMP_ROOT RESULT_ROOT

assert(~isempty(OUTPUT_ROOT) && ~isempty(RECORDED_BIN_ROOT) && ~isempty(DUMP_ROOT), 'The directories are not correctly set');

fclose all;
close all;

csaParamFilePath = fullfile(DUMP_ROOT, 'coordsys_alignment.txt');
csa = CoordSysAligner;
Load(csa, csaParamFilePath);

robotCfgFilePath = fullfile(OUTPUT_ROOT, 'robot_config.prototxt');
robotConfig = ReadRobotConfig(robotCfgFilePath);

camParamFilePath = fullfile(DUMP_ROOT, 'camera_param.txt');
cam = BCStereoCameraModel(camParamFilePath);
CvtToRectifiedModel(cam);
cibfr = CamImuBinFileReader;
SetInputDevice(cam, cibfr);
camImuBinFilePath = fullfile(RECORDED_BIN_ROOT, 'camera_imu.bin');
OpenInput(cam, camImuBinFilePath);

bsbfr = BodySensorBinFileReader;
bodySensorBinFilePath = fullfile(RECORDED_BIN_ROOT, 'bodysensor.bin');
bodySensorRespFilePath = fullfile(RECORDED_BIN_ROOT, 'bodysensor_response.bin');
Open(bsbfr, bodySensorBinFilePath, bodySensorRespFilePath);

bsl = BodySensorLocalizer(GetBodySensorDataDecoder(bsbfr), robotConfig.bodysensor_localizer_param);
vsl = VisualLocalizer(cam, csa, robotConfig.tracking_config);
% AlignBodySensorToCamera(cibfr);
% 
% [imuData, bodySensorData] = GetImuBodySensorSamples(cibfr);
% [bodySensorMotionState, imuMotionState] = GetImuBodySensorMotionState('LK', bodySensorData, imuData, frameTimeStamps, bodySensorLocalizerParam, imuLocalizerParam);
lp = LocalPlanner(robotConfig);

visualizeResult = VisualizeResult();
SAVE_VID = false;
goldenPath = fullfile(RESULT_ROOT, 'alignedData.txt');
LOAD_GOLDEN_DATA =  exist(goldenPath);
LoadGolden(visualizeResult, goldenPath, LOAD_GOLDEN_DATA);

videoFilePath = fullfile(RESULT_ROOT, 'frame.avi');
LoadResultVidFile(visualizeResult, videoFilePath, SAVE_VID)

db = debug();
lp = LocalPlanner(robotConfig);

% figNum = 990000;
motionState = [];

while HasFrame(cam)
    [imgL, imgR, frmTimestamp, frmInd] = GetNextStereoImagePair(cam, true);
    
%     imgL = RGB2BGR(imgL);
%     imgR = RGB2BGR(imgR);
    
    bsSampFrame = GetNextDataFrame(bsbfr, frmTimestamp, 0);
    if isempty(bsSampFrame)
        continue;
%         break;
    end
    
%     bsSampFrame = debug.modifySampData(bsSampFrame,frmInd,0,2);
    Localize(bsl, bsSampFrame, frmInd);
    Localize(vsl, imgL, imgR, GetMotionState(bsl, 'last'), GetPoseWcs(bsl, 'last'), frmInd);
    
%     figure(771177),plot(rad2deg(vsl.poseWcsList(:,3)-bsl.poseWcsList(:,3)));
    
    CompareGoldenTraceAndVisionTrace(visualizeResult,vsl,bsl,77, frmInd, frmTimestamp);
    CompareGoldenAngAndVisionAng(visualizeResult,vsl,bsl,133, frmInd, frmTimestamp);
    [v,w] = PlanLocalRoute(lp, vsl, bsl);
    debug_show_local_planner(db,visualizeResult,vsl, lp, 123)
    motionState = [motionState;GetMotionState(bsl, 'last')];
    debug.debug_915_compare_length_between_vsl_and_bsl(vsl,bsl,motionState, true)
%     id = find(vsl.keyFrameFlagList == 1);
    
%     if ~isempty(id)
%         dataRange =  id(end):size(vsl.keyFrameFlagList,1);
%         if exist('idPrv','var')
%             if id(end) ~= idPrv
%                 figNum = figNum + 1;
%             end
%         end
%         figure(figNum),clf;plot(rad2deg(vsl.poseWcsList(dataRange,3)-vsl.poseWcsList(dataRange(1),3)),'b');hold on; plot(rad2deg(bsl.poseWcsList(dataRange,3)-bsl.poseWcsList(dataRange(1),3)),'r');legend('visual','wheel');
%         idPrv = id(end);
%     end
    
%     PlotMap2D(visualizeResult,vsl,bsl,122, frmInd, frmTimestamp)
    drawnow;
    SaveFrame(visualizeResult, 1122);
%     SaveRawFrame(visualizeResult, imgL,155);
%     SaveStereoFrame(visualizeResult, imgL,imgR, 155);

end

bslPose = bsl.poseWcsList;
vslPose = vsl.poseWcsList;
vslPose(:,1) = -vslPose(:,1);
keyframeList = vsl.keyFrameFlagList;
pt3D = vsl.pointCloudManager.pt3D;
resultFilePath = fullfile(RESULT_ROOT, 'result');
save(resultFilePath,'bslPose','vslPose','keyframeList','pt3D');

CloseInput(cam);
Close(bsbfr);
CloseVid(visualizeResult);

end