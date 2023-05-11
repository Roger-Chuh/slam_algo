function ReplayTrace(folderName,keySeqNum)



replayDir = fullfile(pwd, folderName);
delete(fullfile(pwd,'\temp\*.png'));
delete(fullfile(pwd,'\dispErr\*.png'));
delete(fullfile(pwd,'\prob\*.png'));


delete(fullfile(pwd,'\prob\*.fig'));
delete(fullfile(pwd,'\dump\*.png')); delete(fullfile(pwd,'\dump\*.mat'));
delete(strcat(fullfile(pwd,folderName),'\TraceData*'));

replayDirInfo = dir(fullfile(replayDir,'KeySeq*'));

global OUTPUT_ROOT RECORDED_BIN_ROOT DUMP_ROOT RESULT_ROOT DRAWPOLYGON ANGLEONLY SHOWPOLYGON USEGOLDENDISP USERIGHT UPDATEDEPTH DEPTHITERNUM probPath ...
    USELEFT USELEFTPLUSRIGHT USELEFTXRIGHT USELEFT2 USELEFTPLUSRIGHT2 USELEFTXRIGHT2 USERIGHT2 WITHINEPILINE BOTHLR HARDCUT QUANTCUT FUSIONCUT FORCEEVENPLATFORM ...
    CUTTHETAANDFORCEPLATFORM DIFFTHETATHR GOLDENZ CUTTHETAANDFORCEPLATFORM_NODIFF FORCEEXPEVEN EXPZERO GTTRACKING SOFTP2 Rati Rati1 Rati2 Sc USEthetaProb0 ...
    ONLYP2 NEWTRACKER ONLYP2_2 UPDATEZ_2 UPDATETracking_2 Sc1 Sc2 FORCETHETAPLATFORM1 FORCETHETAPLATFORM2 ONLYP2_1_Max NewOnlyP2 NewTrackerWithProb UPDATETracking_2_2 ...
    UpdateP2_3 doRoundingTheta Switch4to3 doRoundingRefAngList3




t0 = datetime('now','Format','dd-MMM-y HH:mm:ss');t = datestr(t0);t1 = yyyymmdd(t0);t2 = strsplit(t,' ');
tt = strcat(num2str(t1),'_',t2{2});tt(find(tt == ':')) = '';





assert(~isempty(OUTPUT_ROOT) && ~isempty(RECORDED_BIN_ROOT) && ~isempty(DUMP_ROOT), 'The directories are not correctly set');

% fclose all;
% close all;

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
BsSampFrame = [];
PoseWheel = [];
EulerWheel = [];
PoseWheel_All = [];
EulerWheel_All  = [];
curImgTimeStamp = 0;
curFrmTimeStamp = 0;
wheelId = [];
wheelTimeAng = [];

coordSysAligner = vsl.coordSysAligner;
if isempty(vsl.scaleLvl)
    vsl.scaleLvl = 0;
end


% keySeqNum = 11;

if keySeqNum == 1
    needReplace = 0;
else
    needReplace = 1;
end


ScaleLevel = 1; 0;

GOLDENZ = false; false; true;


DRAWPOLYGON = true; false;
ANGLEONLY = true;
SHOWPOLYGON = false;
USEGOLDENDISP = GOLDENZ; true; false; true; false;

USERIGHT = false;true; false; true; false;
USELEFT = true;
USELEFTXRIGHT = false;
USELEFTPLUSRIGHT = false;

USERIGHT2 = false; true; false; true; false;
USELEFT2 = true;
USELEFTXRIGHT2 = false;
USELEFTPLUSRIGHT2 = false;


EXPZERO = false; true; false; true;
GTTRACKING = false; false; true;false;true; false; true;
SOFTP2 = false; true;


Rati = 0;

Rati1 = 0.8; 0; 0.5; 0.8; 0.8; 0.8; 0; 0.4; 0.3; 0.5; 0; 0.8; 0.0001; 0.95;
Rati2 = 0.8; 0;  0.5; 0.8; 0.8; 0.8; 0.5; 0.4; 0.3; 0.5; 0; 0.8; 0.5; 0.8; 0.0001; 0.95;
Sc = 4; 12; 10; 8; 4; 6; 4; 6; 2; 4; 2; 4; 10; 2;4; 2;

Sc1 = 10; 4; 6; 2; 4; 2; 6; 4; 10; 4; 6; 4; 10; 4; 2; 4;4; 2; 4; 2; 20; 8;4;
Sc2 = 10; 4 ;6; 2; 4; 2; 6; 4; 10; 4; 6; 4; 10; 4; 2;4;4; 2; 20; 8; 4;


USEthetaProb0 = false; true;



if 0
    ONLYP2 = true;false;true;
    
    ONLYP2_2 = true;
    UPDATEZ_2 = false; true; false; true; false; true;
    UPDATETracking_2 = true; false; true; false; true;  false;
    UPDATETracking_2_2 = true;
    
    
    
    NEWTRACKER = true; false; true; false;
    NewTrackerWithProb = false;
    
    
    
    FORCETHETAPLATFORM1 = true;
    FORCETHETAPLATFORM2 = true; false;
    ONLYP2_1_Max = false; true; false; true; false; true; false;
    
    NewOnlyP2 = false;
    
    
else
    
    doRoundingRefAngList3 = true; false;true;
    Switch4to3 = true;false; true;
    
    doRoundingTheta = true; false;
    
    
    UpdateP2_3 = true; false; true; false; true;
    
    
    
    EXPZERO = false; true; false; true;
    
    
    ONLYP2 = false; true; false;true;false;true;
    
    
    ONLYP2_2 = false;true;
    UPDATEZ_2 = false; true;  false; true; false; true; false; true; false; true;
    UPDATETracking_2 = false; true; false; true; false; true;  false;
    
    
    
    UPDATETracking_2_2 = true;
    
    
    
    NEWTRACKER = false; false; true; false;
    NewTrackerWithProb = false;
    
    
    
    FORCETHETAPLATFORM1 =  true; false;true;
    FORCETHETAPLATFORM2 =  true; false; true; false;
    ONLYP2_1_Max = false; true; false; true; false; true; false;
    
    NewOnlyP2 = false;
end

if 1
    FORCEEXPEVEN = true; false; true;
    CUTTHETAANDFORCEPLATFORM = true; false; true; false;
    DIFFTHETATHR = true; false;true; true; false; true; false; true;
    CUTTHETAANDFORCEPLATFORM_NODIFF = false;  true;
else
    FORCEEXPEVEN = false; true; false; true;
    CUTTHETAANDFORCEPLATFORM = false;true; false; true; false;
    DIFFTHETATHR = false; true; false;true; true; false; true; false; true;
    CUTTHETAANDFORCEPLATFORM_NODIFF = false;  true;
end
if 0
    FUSIONCUT = false;true;
    QUANTCUT = true;
    HARDCUT = true;  % true;
else
    FUSIONCUT = true;
    QUANTCUT = true;
    HARDCUT = false;  % true;
end

FORCEEVENPLATFORM = ~DIFFTHETATHR; true;false;



WITHINEPILINE = true;true; true; true; false; true;
BOTHLR = false;


UPDATEDEPTH = true; false; true; false; true;

DEPTHITERNUM = 1;


vsl.switchDepth = GOLDENZ; false; true; false;
depthScale = 1; 1.04;1.05; 1;
DltAng = [];
DiffPixTrace = [];
DiffPixTraceGT = [];
splitData = [];

visionTimeAng = [];
visionTimeAngRef = [];
gyroTimeAng = [];
wheelTimeAng = [];
RwheelCur = eye(3);
RgyroCur = eye(3);
EulerGyro = [];
GyroAngAccum = 0;
PoseGyro = [];
useGyro = true; false; true;
frameID = [];
imgBuf = {};
depthGTBuf = {};

saveTrace = false; true;
checkGT = false;

b2c = vsl.coordSysAligner.pctBody2Cam(1, 1).transformMat;
intrMat = Get(vsl.camModel, 'PinholeIntrMat', 1,vsl.scaleLvl);
intrMat0 = Get(vsl.camModel, 'PinholeIntrMat', 1,0);




% vsl.scaleLvl = ScaleLevel;

vsl.feat2check = [1039
    1221
    899
    934
    769
    1102
    426
    886
    1177
    712
    1077
    681
    564
    224
    1072
    77
    111
    113
    127
    134
    135
    149
    170
    235
    266
    279
    291
    299
    302
    307];

% vsl.feat2check = [];

if keySeqNum == 1
    
    probPath = fullfile(pwd, 'prob',strcat(num2str(vsl.configParam.reproj_sigma),'_',num2str(vsl.configParam.reproj_beta),'_',num2str(vsl.configParam.reproj_sigma_right),'_',num2str(vsl.configParam.reproj_beta_right),'____',tt));
    MakeDirIfMissing(probPath);
    
    
    copyfile(fullfile(pwd,'ReplayTrace.m'), probPath);
    copyfile(fullfile(pwd,'VisualLocalizer.m'), probPath);
    copyfile(fullfile(pwd,'ReplayTheta.m'), probPath);
    copyfile(fullfile(pwd,'UpdateNewTracking.m'), probPath);
    copyfile(fullfile(pwd,'CalcP2Exp.m'), probPath);
    copyfile(fullfile(pwd,'updateP2.m'), probPath);
    copyfile(fullfile(pwd,'Tracing\Setting.m'), probPath);
    
    
    
    fid1 = fopen(fullfile(probPath, 'config.txt'),'w');
    
    fprintf(fid1, 'keySeqNum: %d\n', keySeqNum);
    fprintf(fid1, 'ScaleLevel: %d\n', vsl.scaleLvl);
    fprintf(fid1, 'theta_range: [%0.5f %0.5f]\n', rad2deg(vsl.configParam.theta_range));
    fprintf(fid1, 'disparity_error: %0.5f\n',  vsl.configParam.disparity_error);
    fprintf(fid1, 'theta_sample_step: %0.5f\n',rad2deg(vsl.configParam.theta_sample_step));
    fprintf(fid1, 'theta_sample_step2: %0.5f\n',rad2deg(vsl.configParam.theta_sample_step2));
    fprintf(fid1, 'disparity_sample_step: %0.5f\n', vsl.configParam.disparity_sample_step);
    fprintf(fid1, 'reproj_sample_step: %0.5f\n',  vsl.configParam.reproj_sample_step);
    fprintf(fid1, '===================================\n');
    fprintf(fid1, 'disparity_sample_interval: %0.5f\n', vsl.configParam.disparity_sample_interval);
    fprintf(fid1, 'reproj_sample_interval: %0.5f\n',vsl.configParam.reproj_sample_interval);
    fprintf(fid1, '===================================\n');
    fprintf(fid1, 'disparity_sigma: %0.5f\n', vsl.configParam.disparity_sigma);
    fprintf(fid1, 'disparity_beta: %0.5f\n',vsl.configParam.disparity_beta);
    
    fprintf(fid1, 'reproj_sigma0: %0.5f\n', vsl.configParam.reproj_sigma0);
    fprintf(fid1, 'reproj_sigma: %0.5f\n', vsl.configParam.reproj_sigma);
    fprintf(fid1, 'reproj_sigma_scale: %0.5f\n',  vsl.configParam.reproj_sigma_scale);
    fprintf(fid1, 'reproj_beta: %0.5f\n',  vsl.configParam.reproj_beta);
    
    fprintf(fid1, 'reproj_sigma_right0: %0.5f\n', vsl.configParam.reproj_sigma_right0);
    fprintf(fid1, 'reproj_sigma_right: %0.5f\n', vsl.configParam.reproj_sigma_right);
    fprintf(fid1, 'reproj_sigma_scale_right: %0.5f\n',  vsl.configParam.reproj_sigma_scale_right);
    fprintf(fid1, 'reproj_beta_right: %0.5f\n', vsl.configParam.reproj_beta_right);
    
    
    fprintf(fid1, '===================================\n');
    fprintf(fid1, 'feat2check:\n');
    fprintf(fid1, '%d\n', vsl.feat2check');
    fprintf(fid1, '===================================\n');
    fprintf(fid1, 'switchDepth: %d\n', double(vsl.switchDepth));
    fprintf(fid1, 'USEGOLDENDISP: %d\n', double(USEGOLDENDISP));
    fprintf(fid1, '===================================\n');
    fprintf(fid1, 'USERIGHT: %d\n',double(USERIGHT));
    fprintf(fid1, 'USELEFT: %d\n', double(USELEFT));
    fprintf(fid1, 'USELEFTXRIGHT: %d\n', double(USELEFTXRIGHT));
    fprintf(fid1, 'USELEFTPLUSRIGHT: %d\n', double(USELEFTPLUSRIGHT));
    fprintf(fid1, '===================================\n');
    fprintf(fid1, 'USERIGHT2: %d\n',double(USERIGHT2));
    fprintf(fid1, 'USELEFT2: %d\n', double(USELEFT2));
    fprintf(fid1, 'USELEFTXRIGHT2: %d\n', double(USELEFTXRIGHT2));
    fprintf(fid1, 'USELEFTPLUSRIGHT2: %d\n', double(USELEFTPLUSRIGHT2));
    fprintf(fid1, '===================================\n');
    fprintf(fid1, 'FUSIONCUT: %d\n',double(FUSIONCUT));
    fprintf(fid1, 'HARDCUT: %d\n',double(HARDCUT));
    fprintf(fid1, 'QUANTCUT: %d\n',double(QUANTCUT));
    fprintf(fid1, 'WITHINEPILINE: %d\n',double(WITHINEPILINE));
    fprintf(fid1, 'BOTHLR: %d\n',double(BOTHLR));
    fprintf(fid1, 'epiLine_margin: %0.5f\n', vsl.configParam.epiLine_margin);
    fprintf(fid1, 'probZ_ratio_threshold: %0.5f\n', vsl.configParam.probZ_ratio_threshold);
    fprintf(fid1, '===================================\n');
    fprintf(fid1, 'theta_prob_cutoff_threshold: %0.5f\n', vsl.configParam.theta_prob_cutoff_threshold);
    fprintf(fid1, 'depth_prob_cutoff_threshold: %0.5f\n', vsl.configParam.depth_prob_cutoff_threshold);
    fprintf(fid1, 'tracking_trust_radius: %0.5f\n',vsl.configParam.tracking_trust_radius);
    fprintf(fid1, 'ref_theta_trust_margin: %0.5f\n',rad2deg(vsl.configParam.ref_theta_trust_margin));
    fprintf(fid1, '===================================\n');
    fprintf(fid1, 'CUTTHETAANDFORCEPLATFORM: %d\n',double(CUTTHETAANDFORCEPLATFORM));
    fprintf(fid1, 'CUTTHETAANDFORCEPLATFORM_NODIFF: %d\n',double(CUTTHETAANDFORCEPLATFORM_NODIFF));
    fprintf(fid1, 'theta_diff_margin: %0.5f\n', vsl.configParam.theta_diff_margin);
    fprintf(fid1, 'DIFFTHETATHR: %d\n',double(DIFFTHETATHR));
    fprintf(fid1, 'FORCEEVENPLATFORM: %d\n',double(FORCEEVENPLATFORM));
    fprintf(fid1, '===================================\n');
    fprintf(fid1, 'FORCEEXPEVEN: %d\n',double(FORCEEXPEVEN));
    fprintf(fid1, 'expectation_theta_radius: %0.5f\n', rad2deg(vsl.configParam.expectation_theta_radius));
    fprintf(fid1, 'expectation_theta_expand_num: %0.5f\n', vsl.configParam.expectation_theta_expand_num);
    fprintf(fid1, 'EXPZERO: %d\n',double(EXPZERO));
    fprintf(fid1, 'GTTRACKING: %d\n',double(GTTRACKING));
    fprintf(fid1, 'SOFTP2: %d\n',double(SOFTP2));
    fprintf(fid1, '===================================\n');
    fprintf(fid1, 'Rati: %0.5f\n',double(Rati));
    fprintf(fid1, 'Rati1: %0.5f\n',double(Rati1));
    fprintf(fid1, 'Rati2: %0.5f\n',double(Rati2));
    fprintf(fid1, 'Sc: %0.5f\n',double(Sc));
    fprintf(fid1, 'Sc1: %0.5f\n',double(Sc1));
    fprintf(fid1, 'Sc2: %0.5f\n',double(Sc2));
    fprintf(fid1, 'USEthetaProb0: %d\n',double(USEthetaProb0));
    fprintf(fid1, '===================================\n');
    fprintf(fid1, 'modulate_curve_quant_size: %0.5f\n', vsl.configParam.modulate_curve_quant_size);
    fprintf(fid1, '===================================\n');
    fprintf(fid1, 'NEWTRACKER: %d\n',double(NEWTRACKER));
    fprintf(fid1, 'ONLYP2: %d\n',double(ONLYP2));
    fprintf(fid1, 'ONLYP2_2: %d\n',double(ONLYP2_2));
    fprintf(fid1, 'UPDATEZ_2: %d\n',double(UPDATEZ_2));
    fprintf(fid1, 'UPDATETracking_2: %d\n',double(UPDATETracking_2));
    fprintf(fid1, 'UPDATETracking_2_2: %d\n',double(UPDATETracking_2_2));
    fprintf(fid1, '===================================\n');
    fprintf(fid1, 'only_p2_prob_cutoff_threshold: %0.5f\n',vsl.configParam.only_p2_prob_cutoff_threshold);
    fprintf(fid1, 'only_p2_prob_cutoff_threshold_2: %0.5f\n',vsl.configParam.only_p2_prob_cutoff_threshold_2);
    fprintf(fid1, '===================================\n');
    fprintf(fid1, 'FORCETHETAPLATFORM1: %d\n',double(FORCETHETAPLATFORM1));
    fprintf(fid1, 'FORCETHETAPLATFORM2: %d\n',double(FORCETHETAPLATFORM2));
    fprintf(fid1, 'ONLYP2_1_Max: %d\n',double(ONLYP2_1_Max));
    fprintf(fid1, 'NewOnlyP2: %d\n',double(NewOnlyP2));
    fprintf(fid1, 'NewTrackerWithProb: %d\n',double(NewTrackerWithProb));
    fprintf(fid1, 'UpdateP2_3: %d\n',double(UpdateP2_3));
    fprintf(fid1, '===================================\n');
    fprintf(fid1, 'doRoundingTheta: %d\n',double(doRoundingTheta));
    fprintf(fid1, '===================================\n');
    fprintf(fid1, 'Switch4to3: %d\n',double(Switch4to3));
    fprintf(fid1, 'doRoundingRefAngList3: %d\n',double(doRoundingRefAngList3));
    
    fclose(fid1);
    
end


if strcmp(folderName, '20190816_1834_clockwise_5000_small_world_bo')
    load('angErrGT.mat');
else
    if 0
        load('angErrGT.mat');
    elseif 0
        load('angErrGT2.mat');
    elseif 0
        load('angErrGT3.mat');
    else
        finalKeySeqData = load(fullfile(replayDir,replayDirInfo(end).name));
        angErrGT = rad2deg(finalKeySeqData.vsl.poseWcsList(:,3)) - (finalKeySeqData.gyroNew);
    end
    
end

for i = keySeqNum : length(replayDirInfo)  %14 %
    %     RwheelPrv = RwheelCur;
    %     RgyroPrv = RgyroCur;
    curKeySeqData = load(fullfile(replayDir,replayDirInfo(i).name));
    try
        %         wheelTimeAng = curKeySeqData.wheelTimeAng;
        %         gyroTimeAng = curKeySeqData.gyroTimeAng;
        %         visionTimeAng = curKeySeqData.visionTimeAng;
    catch
        asvdhkj = 1;
    end
    for j = 1 : size(curKeySeqData.KeySeq.imgHeader,1)-1
        if needReplace
            if keySeqNum > 1 && j ==1
                
                
                
                prvKeySeqData = load(fullfile(replayDir,replayDirInfo(i-1).name));
                bsl = prvKeySeqData.bsl;
                vsl = prvKeySeqData.vsl;
                
                
                %                 vsl.configParam.feature_num_drop_rate_thresh = 0.85;
                
                vsl.featPtTracker.configParam.left_margin = 8;
                vsl.featPtTracker.configParam.right_margin = 8;
                vsl.featPtTracker.configParam.top_margin = 8;
                vsl.featPtTracker.configParam.bottom_margin = 8;
                vsl.pureRotationAngPredictor.configParam.pure_rot_reproject_outlier_threshold = 2;
                
                %                 vsl.scaleLvl = ScaleLevel;
                %                 imgBuf = [imgBuf; [imresize(vsl.prevImgL,2) imresize(vsl.prevImgR,[480 640])]];
                imgBuf = [imgBuf; [imresize(vsl.prevImgL,[480 640]) imresize(vsl.prevImgR,[480 640])]];
                depthGTBuf = [depthGTBuf; imresize(vsl.prvDepthGT,[480 640])];
                
                vsl.pureRotationAngPredictor.configParam.pnp_ang_est_max_margin = deg2rad([2]);
                vsl.pureRotationAngPredictor.configParam.init_angle_range = deg2rad([-2 2]);
                
                
                vsl.keyFrameDone = false;
                
                
                vsl.coordSysAligner = coordSysAligner;
                
                vsl.switchDepth = GOLDENZ; false;
                %                 vsl.pureRotationAngPredictor.configParam.pure_rot_reproject_outlier_threshold = 2;
                vsl.configParam.disparity_error = 1;0.5; 1; 0.5; 1; 2; 1; 1; 2; 1.5; 1; 2;  1; 2; 1; 2; 1.5; 1; 2; 1.5; 2;   2; 1;
                vsl.configParam.depth_uncertainty_range = [-100 300];
                vsl.configParam.theta_range = deg2rad([-1 1]);
                vsl.configParam.polygon_margin = 2;
                vsl.configParam.polygon_inlier_thresh = 0.2;
                
                vsl.configParam.theta_sample_step =  deg2rad(0.02); % 0.1;
                vsl.configParam.theta_sample_step2 =  deg2rad(0.02); % 0.1;
                vsl.configParam.disparity_sample_step =  0.1;
                vsl.configParam.reproj_sample_step =  0.001;
                
                vsl.configParam.disparity_sample_interval =  0.01;
                vsl.configParam.reproj_sample_interval =  0.01;
                
                vsl.configParam.disparity_sigma = 15;3; 2; 3; 1; %  1/3 1 3
                vsl.configParam.disparity_beta = 100;10; 10; 100; %  1/3 10 100
                
                vsl.configParam.reproj_sigma0 = 1.5; 2; 1; 2; 1; 2; 1.5; 3.6; 1.5; 2; 1.5; 1.5; 2; 3; 1.5; 2; 1; 1.5; % 1/3
                vsl.configParam.reproj_sigma =  1.5; 2; 1; 2; 1;  2; 1.5; 3.6; 1.5; 2; 1.5; 1.5; 2; 3; 1.5; 2; 1; 1.5; % 1/3
                vsl.configParam.reproj_sigma_scale = 1; 1.5; 1.5; 1.5; 2; 3; 1.5; 2; 1; 1.5; % 1/3
                vsl.configParam.reproj_beta =  100; 20; 100;20; 100; 20; 100; % 1/3
                vsl.configParam.reproj_sigma_right0 = 2; 3.6; 2; 1.5; 2; 3 ;1.5; 2; 1; 1.5; % 1/3
                vsl.configParam.reproj_sigma_right = 2; 3.6; 2; 1.5; 2; 3 ;1.5; 2; 1; 1.5; % 1/3
                vsl.configParam.reproj_sigma_scale_right = 5; 1.5; 1.5; 2; 3; 1.5; 2; 1; 1.5; % 1/3
                vsl.configParam.reproj_beta_right = 20 ; % 1/3
                
                vsl.configParam.theta_percentage =  0.6;
                vsl.configParam.depth_hist_ratio = 0.95;
                vsl.configParam.epiLine_margin = 0;
                
                vsl.configParam.probZ_ratio_threshold = 0.0004 ; %0.05;
                
                
                vsl.configParam.theta_prob_cutoff_threshold = 0.99; 0.99;0.5; 0.99; 0.98;  %0.85;
                vsl.configParam.depth_prob_cutoff_threshold = 0.7;
                vsl.configParam.tracking_trust_radius = 1.2;
                vsl.configParam.ref_theta_trust_margin = deg2rad(0.2);
                vsl.configParam.theta_diff_margin = 0.015;  0.005; 0.01; 0.005;  0.005; 0.1; 0.08; 0.3; 0.2;0.003; 0.3; 0.05; 0.003; 0.01; 0.01; 0.005; 0.01; 0.005;
                
                vsl.configParam.expectation_theta_radius = deg2rad(0.2);% 0.5
                vsl.configParam.expectation_theta_expand_num = 0.05*10/rad2deg( vsl.configParam.theta_sample_step); 50; 10; 50; 10; 20;10; 20; 10; 20; 10; 20;  10;30;
                
                
                vsl.configParam.only_p2_prob_cutoff_threshold = 1;     0.8; 0.5; 0.8; 1; 0.4; 0.7; %1; 0.99999999999999; 0; 0.6;
                vsl.configParam.only_p2_prob_cutoff_threshold_2 = 1; 0.99999999999999;0; 0.6; 0.6; 0.95;
                
                
                
                
                
                
                vsl.configParam.modulate_curve_quant_size = 0.1; 0.01; 0.05; 0.05; 0.01;0.1; 0.05;
                
                
                vsl.feat2check = [202
                    547
                    211
                    149
                    616
                    169
                    386
                    646
                    550
                    571
                    573
                    402
                    527
                    196
                    487
                    121
                    257
                    606
                    246
                    337
                    683
                    381
                    654
                    446
                    477
                    478
                    545
                    442
                    258
                    462]; %tracebackId;
                vsl.feat2check = [190
                    553
                    173
                    526
                    104
                    212
                    81
                    111
                    112
                    300
                    351
                    470
                    406
                    356
                    272
                    2
                    4
                    14
                    15
                    19
                    59
                    63
                    67
                    70
                    88
                    89
                    97
                    103
                    130
                    131];
                vsl.feat2check = [161;161];
                
                vsl.configParam.theta_sigma = 0.01;
                
                
                
                
                probPath = fullfile(pwd, 'prob',strcat(num2str(vsl.configParam.reproj_sigma),'_',num2str(vsl.configParam.reproj_beta),'_',num2str(vsl.configParam.reproj_sigma_right),'_',num2str(vsl.configParam.reproj_beta_right),'____',tt));
                MakeDirIfMissing(probPath);
                
                copyfile(fullfile(pwd,'ReplayTrace.m'), probPath);
                copyfile(fullfile(pwd,'VisualLocalizer.m'), probPath);
                copyfile(fullfile(pwd,'ReplayTheta.m'), probPath);
                copyfile(fullfile(pwd,'UpdateNewTracking.m'), probPath);
                copyfile(fullfile(pwd,'CalcP2Exp.m'), probPath);
                copyfile(fullfile(pwd,'updateP2.m'), probPath);
                copyfile(fullfile(pwd,'Tracing\Setting.m'), probPath);
                
                
                
                fid1 = fopen(fullfile(probPath, 'config.txt'),'w');
                fprintf(fid1, 'keySeqNum: %d\n', keySeqNum);
                fprintf(fid1, 'ScaleLevel: %d\n', vsl.scaleLvl);
                fprintf(fid1, 'theta_range: [%0.5f %0.5f]\n', rad2deg(vsl.configParam.theta_range));
                fprintf(fid1, 'disparity_error: %0.5f\n',  vsl.configParam.disparity_error);
                fprintf(fid1, 'theta_sample_step: %0.5f\n',rad2deg(vsl.configParam.theta_sample_step));
                fprintf(fid1, 'theta_sample_step2: %0.5f\n',rad2deg(vsl.configParam.theta_sample_step2));
                fprintf(fid1, 'disparity_sample_step: %0.5f\n', vsl.configParam.disparity_sample_step);
                fprintf(fid1, 'reproj_sample_step: %0.5f\n',  vsl.configParam.reproj_sample_step);
                fprintf(fid1, '===================================\n');
                fprintf(fid1, 'disparity_sample_interval: %0.5f\n', vsl.configParam.disparity_sample_interval);
                fprintf(fid1, 'reproj_sample_interval: %0.5f\n',vsl.configParam.reproj_sample_interval);
                fprintf(fid1, '===================================\n');
                fprintf(fid1, 'disparity_sigma: %0.5f\n', vsl.configParam.disparity_sigma);
                fprintf(fid1, 'disparity_beta: %0.5f\n',vsl.configParam.disparity_beta);
                
                fprintf(fid1, 'reproj_sigma0: %0.5f\n', vsl.configParam.reproj_sigma0);
                fprintf(fid1, 'reproj_sigma: %0.5f\n', vsl.configParam.reproj_sigma);
                fprintf(fid1, 'reproj_sigma_scale: %0.5f\n',  vsl.configParam.reproj_sigma_scale);
                fprintf(fid1, 'reproj_beta: %0.5f\n',  vsl.configParam.reproj_beta);
                
                fprintf(fid1, 'reproj_sigma_right0: %0.5f\n', vsl.configParam.reproj_sigma_right0);
                fprintf(fid1, 'reproj_sigma_right: %0.5f\n', vsl.configParam.reproj_sigma_right);
                fprintf(fid1, 'reproj_sigma_scale_right: %0.5f\n',  vsl.configParam.reproj_sigma_scale_right);
                fprintf(fid1, 'reproj_beta_right: %0.5f\n', vsl.configParam.reproj_beta_right);
                
                fprintf(fid1, '===================================\n');
                fprintf(fid1, 'feat2check:\n');
                fprintf(fid1, '%d\n', vsl.feat2check');
                fprintf(fid1, '===================================\n');
                fprintf(fid1, 'switchDepth: %d\n', double(vsl.switchDepth));
                fprintf(fid1, 'USEGOLDENDISP: %d\n', double(USEGOLDENDISP));
                fprintf(fid1, '===================================\n');
                fprintf(fid1, 'USERIGHT: %d\n',double(USERIGHT));
                fprintf(fid1, 'USELEFT: %d\n', double(USELEFT));
                fprintf(fid1, 'USELEFTXRIGHT: %d\n', double(USELEFTXRIGHT));
                fprintf(fid1, 'USELEFTPLUSRIGHT: %d\n', double(USELEFTPLUSRIGHT));
                fprintf(fid1, '===================================\n');
                fprintf(fid1, 'USERIGHT2: %d\n',double(USERIGHT2));
                fprintf(fid1, 'USELEFT2: %d\n', double(USELEFT2));
                fprintf(fid1, 'USELEFTXRIGHT2: %d\n', double(USELEFTXRIGHT2));
                fprintf(fid1, 'USELEFTPLUSRIGHT2: %d\n', double(USELEFTPLUSRIGHT2));
                fprintf(fid1, '===================================\n');
                fprintf(fid1, 'FUSIONCUT: %d\n',double(FUSIONCUT));
                fprintf(fid1, 'HARDCUT: %d\n',double(HARDCUT));
                fprintf(fid1, 'QUANTCUT: %d\n',double(QUANTCUT));
                fprintf(fid1, 'WITHINEPILINE: %d\n',double(WITHINEPILINE));
                fprintf(fid1, 'BOTHLR: %d\n',double(BOTHLR));
                fprintf(fid1, 'epiLine_margin: %0.5f\n', vsl.configParam.epiLine_margin);
                fprintf(fid1, 'probZ_ratio_threshold: %0.5f\n', vsl.configParam.probZ_ratio_threshold);
                fprintf(fid1, '===================================\n');
                fprintf(fid1, 'theta_prob_cutoff_threshold: %0.5f\n', vsl.configParam.theta_prob_cutoff_threshold);
                fprintf(fid1, 'depth_prob_cutoff_threshold: %0.5f\n', vsl.configParam.depth_prob_cutoff_threshold);
                fprintf(fid1, 'tracking_trust_radius: %0.5f\n',vsl.configParam.tracking_trust_radius);
                fprintf(fid1, 'ref_theta_trust_margin: %0.5f\n',rad2deg(vsl.configParam.ref_theta_trust_margin));
                fprintf(fid1, '===================================\n');
                fprintf(fid1, 'CUTTHETAANDFORCEPLATFORM: %d\n',double(CUTTHETAANDFORCEPLATFORM));
                fprintf(fid1, 'CUTTHETAANDFORCEPLATFORM_NODIFF: %d\n',double(CUTTHETAANDFORCEPLATFORM_NODIFF));
                fprintf(fid1, 'theta_diff_margin: %0.5f\n', vsl.configParam.theta_diff_margin);
                fprintf(fid1, 'DIFFTHETATHR: %d\n',double(DIFFTHETATHR));
                fprintf(fid1, 'FORCEEVENPLATFORM: %d\n',double(FORCEEVENPLATFORM));
                fprintf(fid1, '===================================\n');
                fprintf(fid1, 'FORCEEXPEVEN: %d\n',double(FORCEEXPEVEN));
                fprintf(fid1, 'expectation_theta_radius: %0.5f\n', rad2deg(vsl.configParam.expectation_theta_radius));
                fprintf(fid1, 'expectation_theta_expand_num: %0.5f\n', vsl.configParam.expectation_theta_expand_num);
                fprintf(fid1, 'EXPZERO: %d\n',double(EXPZERO));
                fprintf(fid1, 'GTTRACKING: %d\n',double(GTTRACKING));
                fprintf(fid1, 'SOFTP2: %d\n',double(SOFTP2));
                fprintf(fid1, '===================================\n');
                fprintf(fid1, 'Rati: %0.5f\n',double(Rati));
                fprintf(fid1, 'Rati1: %0.5f\n',double(Rati1));
                fprintf(fid1, 'Rati2: %0.5f\n',double(Rati2));
                fprintf(fid1, 'Sc: %0.5f\n',double(Sc));
                fprintf(fid1, 'Sc1: %0.5f\n',double(Sc1));
                fprintf(fid1, 'Sc2: %0.5f\n',double(Sc2));
                fprintf(fid1, 'USEthetaProb0: %d\n',double(USEthetaProb0));
                fprintf(fid1, '===================================\n');
                fprintf(fid1, 'modulate_curve_quant_size: %0.5f\n', vsl.configParam.modulate_curve_quant_size);
                fprintf(fid1, '===================================\n');
                fprintf(fid1, 'NEWTRACKER: %d\n',double(NEWTRACKER));
                fprintf(fid1, 'ONLYP2: %d\n',double(ONLYP2));
                fprintf(fid1, 'ONLYP2_2: %d\n',double(ONLYP2_2));
                fprintf(fid1, 'UPDATEZ_2: %d\n',double(UPDATEZ_2));
                fprintf(fid1, 'UPDATETracking_2: %d\n',double(UPDATETracking_2));
                fprintf(fid1, 'UPDATETracking_2_2: %d\n',double(UPDATETracking_2_2));
                fprintf(fid1, '===================================\n');
                fprintf(fid1, 'only_p2_prob_cutoff_threshold: %0.5f\n',vsl.configParam.only_p2_prob_cutoff_threshold);
                fprintf(fid1, 'only_p2_prob_cutoff_threshold_2: %0.5f\n',vsl.configParam.only_p2_prob_cutoff_threshold_2);
                fprintf(fid1, '===================================\n');
                fprintf(fid1, 'FORCETHETAPLATFORM1: %d\n',double(FORCETHETAPLATFORM1));
                fprintf(fid1, 'FORCETHETAPLATFORM2: %d\n',double(FORCETHETAPLATFORM2));
                fprintf(fid1, 'ONLYP2_1_Max: %d\n',double(ONLYP2_1_Max));
                fprintf(fid1, 'NewOnlyP2: %d\n',double(NewOnlyP2));
                fprintf(fid1, 'NewTrackerWithProb: %d\n',double(NewTrackerWithProb));
                fprintf(fid1, 'UpdateP2_3: %d\n',double(UpdateP2_3));
                fprintf(fid1, '===================================\n');
                fprintf(fid1, 'doRoundingTheta: %d\n',double(doRoundingTheta));
                fprintf(fid1, '===================================\n');
                fprintf(fid1, 'Switch4to3: %d\n',double(Switch4to3));
                fprintf(fid1, 'doRoundingRefAngList3: %d\n',double(doRoundingRefAngList3));
                
                fclose(fid1);
                %                 vsl.configParam.feature_num_drop_rate_thresh = 0.65;
                %             vsl.prevImgL =
                needReplace = 0;
            end
        end
        
        RwheelPrv = RwheelCur;
        RgyroPrv = RgyroCur;
        
        bsSampFrame = curKeySeqData.KeySeq.bsSampFrame{j,1};
        imgHeader = curKeySeqData.KeySeq.imgHeader{j,1};
        imgL = curKeySeqData.KeySeq.imgL{j,1};
        imgR = curKeySeqData.KeySeq.imgR{j,1};
        
        imgL = Remap(vsl.camModel.monoCamModel1, imgL);
        imgR = Remap(vsl.camModel.monoCamModel2, imgR);
        
        
        
        imuSampFrame = curKeySeqData.KeySeq.imuSampFrame{j,1};
        goldenVal = curKeySeqData.KeySeq.goldenVal{j,1};
        depthGT = curKeySeqData.KeySeq.depthGT{j,1};
        depthGTR = curKeySeqData.KeySeq.depthGTR{j,1};
        depthGT = depthScale*depthGT;
        frameID = [frameID; imgHeader(1)];
        
        if length(frameID) == 1
            imgLOrig = imgL;
            imgROrig = imgR;
            depthGTOrig = depthGT;
            depthGTROrig = depthGTR;
        end
        
        BsSampFrame = [BsSampFrame;[bsSampFrame goldenVal]];
        wheelData = [bsSampFrame(end,2) goldenVal(end,4:end)];
        RwheelCur = (roty(0)*rotx(0)*quatern2rotMat(wheelData([5 2 3 4])));
        %     RwheelCur = [-RwheelCur(2,:); RwheelCur(3,:); -RwheelCur(1,:)];
        
        
        
        
        RwheelDelt = RwheelCur'; %(RwheelCur'*RwheelPrv)';%
        %     eulerWheel = rad2deg(rotMat2euler(RwheelDelt));
        
        eulerWheel = rad2deg(rodrigues(RwheelDelt)');
        %     eulerWheel = sign(eulerWheel(2))*eulerWheel;
        %     eulerWheel = ([-eulerWheel(:,2) -eulerWheel(:,3) eulerWheel(:,1)]);
        eulerWheel = ([-eulerWheel(:,2) -eulerWheel(:,3) eulerWheel(:,1)]);
        rWheelTmp = rodrigues(deg2rad(eulerWheel));
        PoseWheel = [PoseWheel; [rWheelTmp(:)' -goldenVal(end,2) -goldenVal(end,3) goldenVal(end,1)]];
        EulerWheel = [EulerWheel;[wheelData(1) eulerWheel sign(eulerWheel(2))*norm(rad2deg(rodrigues(RwheelDelt)'))]];
        
        [PoseWheel_, EulerWheel_] = CvtBsFrame2Pose(bsSampFrame,goldenVal);
        
        PoseWheel_All = [PoseWheel_All;PoseWheel_];
        EulerWheel_All = [EulerWheel_All; EulerWheel_];
        gyroData = imuSampFrame(end,2:6);
        RgyroCur = (roty(0)*rotx(0)*quatern2rotMat(gyroData([5 2 3 4])));
        %     RgyroCur = [-RgyroCur(2,:); RgyroCur(3,:); -RgyroCur(1,:)];
        
        
        rgyroDelt_ = rodrigues(RgyroCur'*RgyroPrv);
        RgyroDelt_ = rad2deg(norm(rodrigues(RgyroCur'*RgyroPrv)));
        
        if rgyroDelt_(3)<0
            GyroAngAccum = [GyroAngAccum; -RgyroDelt_];
        else
            GyroAngAccum = [GyroAngAccum; RgyroDelt_];
        end
        RgyroDelt = RgyroCur'; %(RgyroCur'*RgyroPrv)'; %RgyroCur; %
        %     eulerGyro = rad2deg(rotMat2euler(RgyroDelt));
        
        eulerGyro = rad2deg(rodrigues(RgyroDelt)');
        %     eulerGyro = sign(eulerGyro(2))*eulerGyro;
        %     eulerGyro = ([-eulerGyro(:,2) -eulerGyro(:,3) eulerGyro(:,1)]);
        eulerGyro = ([-eulerGyro(:,2) -eulerGyro(:,3) eulerGyro(:,1)]);
        if exist('rGyroTmp','var')
            rGyroTmpPrv = rGyroTmp;
        end
        rGyroTmp = rodrigues(deg2rad(eulerGyro));
        PoseGyro = [PoseGyro; [rGyroTmp(:)'  -goldenVal(end,2) -goldenVal(end,3) goldenVal(end,1)]];
        EulerGyro = [EulerGyro;[gyroData(1) eulerGyro sign(eulerGyro(2))*norm(rad2deg(rodrigues(RgyroDelt)'))]];
        
        
        gyroTimeAng = [EulerGyro(:,1) cumsum(GyroAngAccum([1 [3:end]]))];
        
        
        prvImgTimeStamp = curImgTimeStamp;
        curImgTimeStamp = gyroData(1); %bsSampFrame(end,2);  %imgHeader(2);
        if isempty(vsl.frmStampList)
            prvFrmTimeStamp = curFrmTimeStamp;
        else
            prvFrmTimeStamp = vsl.frmStampList(end);
        end
        curFrmTimeStamp = imgHeader(2); %bsSampFrame(end,2);  %imgHeader(2);
        
        if saveTrace
            if vsl.keyFrameDone
                
                ptTrace = vsl.featPtManager.localTrace;
                %                 theta = vsl.poseWcsList(size(vsl.poseWcsList,1) - size(ptTrace.ptIcsX,2) + 1:end,3);
                %                 theta = vsl.poseWcsList(size(bsl.poseWcsList,1) - size(ptTrace.ptIcsX,2) + 1:end,3);
                
                theta = bsl.poseWcsList(size(bsl.poseWcsList,1) - size(ptTrace.ptIcsX,2) + 1:end,3);
                try
                    visionTimeAngTemp = visionTimeAng(size(visionTimeAng,1) - size(ptTrace.ptIcsX,2) + 1:end,:);
                catch
                    %                     visionTimeAngTemp = visionTimeAng(size(visionTimeAng,1) - size(ptTrace.ptIcsX,2) + 0:end,:);
                    visionTimeAngTemp = visionTimeAng;
                end
                try
                    wheelTimeAngTemp = wheelTimeAng(size(wheelTimeAng,1) - size(ptTrace.ptIcsX,2) + 1:end,:);
                catch
                    %                     wheelTimeAngTemp = wheelTimeAng(size(wheelTimeAng,1) - size(ptTrace.ptIcsX,2) + 0:end,:);
                    wheelTimeAngTemp = wheelTimeAng;
                end
                try
                    gyroTimeAngTemp = gyroTimeAng(size(gyroTimeAng,1) - size(ptTrace.ptIcsX,2) + 1:end,:);
                catch
                    %                     gyroTimeAngTemp = gyroTimeAng(size(gyroTimeAng,1) - size(ptTrace.ptIcsX,2) + 0:end,:);
                    gyroTimeAngTemp = gyroTimeAng;
                end
                %             imgBuf = imgBuf(length(imgBuf) - length(theta) + 1 : end);
                save(fullfile(replayDir,sprintf('TraceData_%05d.mat',length(dir(fullfile(replayDir,'TraceData_*.mat')))+1)), 'b2c','theta','ptTrace','intrMat','imgBuf','visionTimeAngTemp','wheelTimeAngTemp','gyroTimeAngTemp','depthGTBuf');
                imgBuf = imgBuf(end,:);
                depthGTBuf = depthGTBuf(end,:);
                
                aawbs = 1;
            end
        end
        imgBuf = [imgBuf; [imgL imgR]];
        depthGTBuf = [depthGTBuf; depthGT];
        if 1
            
            idWheel = find(BsSampFrame(:,2) >= prvFrmTimeStamp & BsSampFrame(:,2) < curFrmTimeStamp);
            idWheel1 = find(BsSampFrame(:,2) > prvFrmTimeStamp,1,'first');
            if isempty(idWheel1)
                idWheel1 = 0;
            end
            idWheel2 = find(BsSampFrame(:,2) > curFrmTimeStamp,1,'first');
            if isempty(idWheel2)
                idWheel2 = size(BsSampFrame,1);
            end
            %         Localize(bsl, bsSampFrame, imgHeader(1));
            if idWheel1 + 1 > idWheel2
                idWheel1 = idWheel2 - 1;
            end
            
            wheelId = [wheelId;[idWheel1+1 idWheel2]];
            if size(wheelId,1) == 1
                Localize(bsl, BsSampFrame([idWheel1+1 : idWheel2],1:6), imgHeader(1));
            elseif 1
                Localize(bsl, BsSampFrame([wheelId(end-1,2)+1 : idWheel2],1:6), imgHeader(1));
            else
                Localize(bsl, BsSampFrame(idWheel,1:6), imgHeader(1));
            end
            
            if useGyro
                try
                    if 1
                        if ~isnan(curKeySeqData.gyroNew(size(bsl.poseWcsList,1)))
                            wheelNew4 = interp1(curKeySeqData.gyroTimeAng(:,1),curKeySeqData.gyroTimeAng(:,2),curKeySeqData.visionTimeAng(:,1));
                            wheelNew4Id = wheelNew4(find(uint32(curKeySeqData.visionTimeAng(:,1)) == uint32(curFrmTimeStamp)));
                            bsl.poseWcsList(1:end,3) = deg2rad(curKeySeqData.gyroNew(1:size(bsl.poseWcsList,1)));
                            try
                                vsl.prevRefRobotRotAngWcs = bsl.poseWcsList(end-1,3);
                                vsl.prevRefRobotPoseWcs.transformMat(1:3,1:3) = roty(rad2deg(-vsl.prevRefRobotRotAngWcs));
                            catch
                                askbj = 1;
                            end
                            
                            %                             bsl.poseWcsList(1:end,3) = deg2rad(wheelNew4Id);
                            skh = 1;
                        end
                    else
                        gyroNew = interp1(gyroTimeAng(:,1),gyroTimeAng(:,2),visionTimeAng(:,1));
                        if ~isnan(curKeySeqData.gyroNew(size(bsl.poseWcsList,1)))
                            bsl.poseWcsList(1:end,3) = deg2rad(curKeySeqData.gyroNew(1:size(bsl.poseWcsList,1)));
                        end
                        
                    end
                    
                catch
                    asgnjk = 1;
                end
            end
            
            try
                if size(BsSampFrame,1) > idWheel(end)
                    
                    wheelTimeAng = [wheelTimeAng; [BsSampFrame(idWheel(end) + 1,2) rad2deg(bsl.poseWcsList(end,3))]];
                    
                else
                    wheelTimeAng = [wheelTimeAng; [BsSampFrame(end,2) rad2deg(bsl.poseWcsList(end,3))]];
                end
            catch
                wheelTimeAng = [wheelTimeAng; [BsSampFrame(end,2) rad2deg(bsl.poseWcsList(end,3))]];
            end
        else
            Localize(bsl, bsSampFrame(:,1:6), imgHeader(1));
            %         bsl.poseWcsList(i,3) = deg2rad(curKeySeqData.gyroNew(j));
            try
                if ~isnan(curKeySeqData.gyroNew(size(bsl.poseWcsList,1)))
                    bsl.poseWcsList(1:end,3) = deg2rad(curKeySeqData.gyroNew(1:size(bsl.poseWcsList,1)));
                end
            catch
                asgnjk = 1;
            end
        end
        
        
        vsl.goldenPose = [vsl.goldenPose; [EulerWheel_All(:,1) PoseWheel_All]];
        vsl.frmStamp = [prvFrmTimeStamp curFrmTimeStamp];
        vsl.frmStampList = [vsl.frmStampList; curFrmTimeStamp];
        
        
        vsl.depthGT = imresize(depthGT, 2.^-vsl.scaleLvl);
        try
            if size(vsl.prevImgL,1) ~= size(vsl.depthGT,1)
                vsl.prevImgL = imresize(vsl.prevImgL, [size(vsl.depthGT) ]);
                vsl.currImgL = imresize(vsl.currImgL, [size(vsl.depthGT) ]);
                vsl.prevImgR = imresize(vsl.prevImgR, [size(vsl.depthGT) ]);
                vsl.currImgR = imresize(vsl.currImgR, [size(vsl.depthGT) ]);
                vsl.keyFrameDepthGT = imresize(vsl.keyFrameDepthGT, [size(vsl.depthGT) ]);
                vsl.prvDepthGT = imresize(vsl.prvDepthGT, [size(vsl.depthGT) ]);
            end
        catch
            ahkj = 1;
        end
        Localize(vsl, imgL, imgR, GetMotionState(bsl, 'last'), GetPoseWcs(bsl, 'last'), imgHeader(1));
        visionTimeAng = [visionTimeAng; [curFrmTimeStamp rad2deg(vsl.poseWcsList(end,3))]];
        visionTimeAngRef = [visionTimeAngRef; [curFrmTimeStamp rad2deg(bsl.poseWcsList(end,3))]];
        if length(vsl.keyFrameFlagList) > 1
            if vsl.keyFrameFlagList(end-1)
                idd = length(vsl.angOpt(1:end-1));
            end
            try
                vsl.angOpt(end) = vsl.angOpt(idd) + vsl.angOpt(end);
                
                vsl.angOpt3(end) = vsl.angOpt3(idd) + vsl.angOpt3(end);
                
                
                vsl.angOptPnP(end) = vsl.angOptPnP(idd) + vsl.angOptPnP(end);
                %                  vsl.angRng(end,:) = vsl.angRng(idd,:) + vsl.angRng(end,:);
                %                  vsl.angRng(end,:) = [vsl.angOpt(end) vsl.angOpt(end)] + vsl.angRng(end,:);
                vsl.angRng(end,:) = [vsl.angOpt(idd) vsl.angOpt(idd)] + vsl.angRng(end,:);
            catch
                asgdhjk = 1;
            end
        end
        try
            if 0
                gyroNew = interp1(gyroTimeAng(:,1),gyroTimeAng(:,2),visionTimeAng(:,1));
                bsl.poseWcsList(:,3) = deg2rad(gyroNew);
            end
        catch
            ahbdb = 1;
        end
        
        %         idPulse = find(abs(rad2deg(vsl.poseWcsList(:,3))) > 0.001);
        n = floor(abs(rad2deg(vsl.poseWcsList(end,3)))/360);
        pulseVec = [0 : 360 : (n)*360]';
        [DD] = pdist2([pulseVec],abs(rad2deg(vsl.poseWcsList(:,3))),'euclidean');
        [~,idPulse] = min(DD');
        %         idPulse = find(abs(rad2deg(vsl.poseWcsList(:,3))));
        %         pulse = repmat(min(rad2deg(vsl.poseWcsList(:,3))-rad2deg(bsl.poseWcsList(:,3))),size(vsl.poseWcsList,1),1);
        %         pulse(idPulse) = max(rad2deg(vsl.poseWcsList(:,3))-rad2deg(bsl.poseWcsList(:,3)));
        
        % abs((rad2deg(vsl.poseWcsList(end,3) - bsl.poseWcsList(end,3))) - (rad2deg(vsl.poseWcsList(end-1,3) - bsl.poseWcsList(end-1,3)))) > 0.1
        %         angErr = rad2deg(vsl.poseWcsList(:,3) - bsl.poseWcsList(:,3));
        idid = find(vsl.keyFrameFlagList);
        
        
        if 1
            if ~isempty(idid)
                ididInit = idid(end);
                if ~exist('ididInit0', 'var')
                    ididInit0 = ididInit(end);
                end
                idid2 = idid(find(idid >= ididInit0));
                bsAng = bsl.poseWcsList(:,3);
                bsAng2 = bsAng;
                bsAng2(idid2(1):end) = bsAng2(idid2(1):end) - bsAng2(idid2(1));
                if keySeqNum == 1
                    bsAng = bsAng2;
                end
                bsAng3 = [bsAng2 bsAng2];
                %                 angErr2 = [[0;0];rad2deg(vsl.angOpt - bsAng(size(bsl.poseWcsList,1) - size(vsl.angOpt,1) + 1:end,1))];
                angErr2 = [[0;0];rad2deg(vsl.angOpt - bsAng2(size(bsl.poseWcsList,1) - size(vsl.angOpt,1) + 1:end,1))];
                angErr22 = [[0;0];rad2deg(vsl.angOptPnP - bsAng2(size(bsl.poseWcsList,1) - size(vsl.angOptPnP,1) + 1:end,1))];
                angErr222 = [[0;0];rad2deg(vsl.angOpt3 - bsAng2(size(bsl.poseWcsList,1) - size(vsl.angOpt3,1) + 1:end,1))];
                
                angErr3 = [[0 0;0 0];rad2deg(vsl.angRng - bsAng3(size(bsl.poseWcsList,1) - size(vsl.angOpt,1) + 1:end,:))];
                angErr = rad2deg(vsl.poseWcsList(:,3) - bsAng);
                %                 angErr_ = angErr(size(angErr,1) - size(angErr2,1) + 1 : end,:); angErr_(1:3,:) = 0;
                %                 angErr2 = [zeros(size(angErr,1) - size(angErr2,1),1); angErr2];
                if ~exist('ididInitErr0', 'var')
                    ididInitErr0 = length(angErr2);
                end
                angErr(ididInit0:end) = angErr(ididInit0:end) - angErr(ididInit0);
                angErr2 = [angErr(1:ididInit0-1,:); angErr2(ididInitErr0-1:end)];
                angErr22 = [angErr(1:ididInit0-1,:); angErr22(ididInitErr0-1:end)];
                angErr222 = [angErr(1:ididInit0-1,:); angErr222(ididInitErr0-1:end)];
                angErr3 = [[angErr(1:ididInit0-1,:) angErr(1:ididInit0-1,:)]; angErr3(ididInitErr0-1 : end,:)];
            else
                angErr2 = [[0;0];rad2deg(vsl.angOpt - bsl.poseWcsList(size(bsl.poseWcsList,1) - size(vsl.angOpt,1) + 1:end,3))];
                angErr22 = [[0;0];rad2deg(vsl.angOptPnP - bsl.poseWcsList(size(bsl.poseWcsList,1) - size(vsl.angOptPnP,1) + 1:end,3))];
                angErr222 = [[0;0];rad2deg(vsl.angOpt3 - bsl.poseWcsList(size(bsl.poseWcsList,1) - size(vsl.angOpt3,1) + 1:end,3))];
                
                angErr = rad2deg(vsl.poseWcsList(:,3) - bsl.poseWcsList(:,3));
                try
                    angErr3 = [[0 0;0 0];rad2deg(vsl.angRng - repmat(bsl.poseWcsList(size(bsl.poseWcsList,1) - size(vsl.angOpt,1) + 1:end,3),1,2))];
                catch
                    angErr3 = [angErr2 angErr2];
                end
            end
            
            
        else
            if ~isempty(idid)
                bsAng = bsl.poseWcsList(:,3);
                bsAng(idid(1):end) = bsAng(idid(1):end) - bsAng(idid(1));
                bsAng3 = [bsAng bsAng];
                angErr2 = [[0;0];rad2deg(vsl.angOpt - bsAng(size(bsl.poseWcsList,1) - size(vsl.angOpt,1) + 1:end,1))];
                angErr3 = [[0 0;0 0];rad2deg(vsl.angRng - bsAng3(size(bsl.poseWcsList,1) - size(vsl.angOpt,1) + 1:end,:))];
                angErr = rad2deg(vsl.poseWcsList(:,3) - bsAng);
            else
                angErr2 = [[0;0];rad2deg(vsl.angOpt - bsl.poseWcsList(size(bsl.poseWcsList,1) - size(vsl.angOpt,1) + 1:end,3))];
                angErr = rad2deg(vsl.poseWcsList(:,3) - bsl.poseWcsList(:,3));
                try
                    angErr3 = [[0 0;0 0];rad2deg(vsl.angRng - repmat(bsl.poseWcsList(size(bsl.poseWcsList,1) - size(vsl.angOpt,1) + 1:end,3),1,2))];
                catch
                    angErr3 = [angErr2 angErr2];
                end
            end
        end
        if 0
            noise = 0.1*((rand(length(angErr),1))-0.5)./5;
            noise = round(noise./0.01).*0.01;
        else
            noise = 0;
        end
        pulse = repmat(min(angErr),size(vsl.poseWcsList,1),1);
        pulse(idPulse) = max(angErr);
        %         figure(12),clf;plot(angErr + noise);title('visual-body');hold on;plot(find(vsl.keyFrameFlagList),rad2deg(vsl.poseWcsList(find(vsl.keyFrameFlagList),3))-rad2deg(bsl.poseWcsList(find(vsl.keyFrameFlagList),3)),'or');plot(pulse,'k');%plot(scaleNum.*rotAxisGyro([3],1:end)','g'); %plot(deg2rad(Euler(:,3)));
        figure(12),clf;plot(angErr + noise);title('visual-body');hold on;plot(find(vsl.keyFrameFlagList),angErr(find(vsl.keyFrameFlagList)),'or');plot(pulse,'k');%plot(scaleNum.*rotAxisGyro([3],1:end)','g'); %plot(deg2rad(Euler(:,3)));
        figure(10),clf;plot(angErr,'r');hold on;plot(angErr2,'g');plot(angErr22,'c');plot(angErr222,'m');plot(angErr3,'b');plot(find(vsl.keyFrameFlagList),angErr222(find(vsl.keyFrameFlagList)),'*k');plot(find(vsl.keyFrameFlagList),angErr2(find(vsl.keyFrameFlagList)),'om');plot(pulse,'k');title(num2str(vsl.configParam.theta_percentage));
        try
            figure(10),plot(angErrGT(1:length(angErr)),'-k');
        catch
            dbdabaa = 1;
        end
        figure(10),legend('pnp err','2nd opt err','opt pnp err','only p2 err','upper err', 'lower err','key frame2','key frame','one cycle','golden err');
        %         saveas(gcf,fullfile(probPath,sprintf('final_error_%05d.fig',length(dir(fullfile(probPath,'final_error_*.fig')))+1)));
        if 1
            saveas(gcf,fullfile(probPath,sprintf('final_error_%05d.fig',0+1)));
        end
        try
            save(fullfile(probPath,'vsl.mat'), 'vsl','bsl','angErr','angErr2','angErr22','angErr222','angErr3');
        catch
            gssagq = 1;
        end
        
        
        
        try
            figure(22),clf;plot(visionTimeAng(:,1),visionTimeAng(:,2) - visionTimeAng(1,2),'-x');hold on;plot(wheelTimeAng(:,1),wheelTimeAng(:,2) - wheelTimeAng(1,2),'-x');plot(gyroTimeAng(:,1),gyroTimeAng(:,2) - gyroTimeAng(1,2),'-x');plot(visionTimeAngRef(:,1),visionTimeAngRef(:,2) - visionTimeAngRef(1,2),'-x');legend('visual','wheel','gyro','ref');
        catch
            svkj = 1;
        end
        
        try
            [sum(vsl.featPtManager.localTrace.xGT(:,end) > 0) sum(vsl.featPtManager.localTrace.ptIcsX(:,end) > 0)]
            %             temp1 = vsl.featPtManager;
            %             temp2 = temp1;
            %             temp2.localTrace.ptIcsX = temp1.localTrace.xGT;
            %             temp2.localTrace.ptIcsY = temp1.localTrace.yGT;
            figure(1133), UpdateLocalTracePlot1(vsl.featPtManager, vsl.currImgL);
            UpdateLocalTracePlot2(vsl.featPtManager, vsl.currImgL);
            
        catch
            sgdnk = 1;
        end
        
        try
            diffBody = diff(rad2deg(bsl.poseWcsList(:,3)));
            diffVisual = diff(rad2deg(vsl.poseWcsList(:,3)));
            diffPixTrace = diff(vsl.MeanErr(:,1));
            diffPixTraceGT = diff(vsl.MeanErrGT(:,1));
            ofst = length(diffVisual) - length(diffPixTrace) + 1;
            dltAng = [(diffVisual( ofst: end) - diffBody(ofst: end))];
            
            DltAng = [DltAng; dltAng(end)];
            DiffPixTrace = [DiffPixTrace; diffPixTrace(end)];
            DiffPixTraceGT = [DiffPixTraceGT; diffPixTraceGT(end)];
            if length(diffPixTrace) == 1
                try
                    splitData = [splitData; [length(DltAng)-1 DltAng(end-1) length(DiffPixTrace)-1 DiffPixTrace(end-1) length(DiffPixTraceGT)-1 DiffPixTraceGT(end-1)]];
                catch
                    kbja = 1;
                end
                
            end
            
            %             figure(7),clf;subplot(3,1,1),plot(DltAng);title('p2cAng(visual) - p2cAng(body)');
            %             subplot(3,1,2);plot([DiffPixTrace]);title('cur(vslX - trackingX) - prv(vslX - trackingX)')
            %             subplot(3,1,3);plot([DiffPixTraceGT]);title('cur(vslX - gtX) - prv(vslX - gtX)')
            if 0 %length(diffPixTrace) == 1
                try
                    figure(7),clf;subplot(3,1,1),plot(DltAng);title('p2cAng(visual) - p2cAng(body)');hold on;plot(splitData(:,1),splitData(:,2),'or');
                    subplot(3,1,2);plot([DiffPixTrace]);title('cur(vslX - trackingX) - prv(vslX - trackingX)');hold on;plot(splitData(:,3),splitData(:,4),'or');
                    subplot(3,1,3);plot([DiffPixTraceGT]);title('cur(vslX - gtX) - prv(vslX - gtX)');hold on;plot(splitData(:,5),splitData(:,6),'or');
                catch
                    figure(7),clf;subplot(3,1,1),plot(DltAng);title('p2cAng(visual) - p2cAng(body)');
                    subplot(3,1,2);plot([DiffPixTrace]);title('cur(vslX - trackingX) - prv(vslX - trackingX)')
                    subplot(3,1,3);plot([DiffPixTraceGT]);title('cur(vslX - gtX) - prv(vslX - gtX)')
                end
            else
                figure(7),clf;subplot(3,1,1),plot(dltAng);title('p2cAng(visual) - p2cAng(body)');
                subplot(3,1,2);plot([diffPixTrace]);title('cur(vslX - trackingX) - prv(vslX - trackingX)')
                subplot(3,1,3);plot([diffPixTraceGT]);title('cur(vslX - gtX) - prv(vslX - gtX)');
            end
            
            %             figure(7),clf;subplot(3,1,1),plot([(diffVisual(length(diffVisual) - length(diffPixTrace) + 1 : end) - diffBody(length(diffVisual) - length(diffPixTrace) + 1 : end))]);title('p2c(visual) - p2c(body)');
            %             subplot(3,1,2);plot([diffPixTrace]);title('cur(vslX - trackingX) - prv(vslX - trackingX)')
            %             subplot(3,1,3);plot([diffPixTrace]);title('cur(vslX - gtX) - prv(vslX - gtX)')
            
        catch
            savj = 1;
        end
        try
            figure(21);clf;plot(vsl.MeanErrOld(:,1), vsl.MeanErrOld(:,2),'-o');title('gt - tracking'); axis equal
            hold on;
            for k = 1 : size(vsl.MeanErrOld,1)
                text(double(vsl.MeanErrOld(k,1)),double(vsl.MeanErrOld(k,2)),num2str(k), 'Color',[1 0 0],'FontSize',15);  %,'FontWeight','bold');
            end
            saveas(gcf,fullfile(probPath,sprintf(strcat('trackingErrP2_',probPath(end-14:end),'____',num2str(sum(vsl.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('trackingErrP2_',probPath(end-14:end),'____',num2str(sum(vsl.keyFrameFlagList)),'__*.png'))))+1)));
            figure(14);
            saveas(gcf,fullfile(probPath,sprintf(strcat('trackingErrP2All_',probPath(end-14:end),'____',num2str(sum(vsl.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('trackingErrP2All_',probPath(end-14:end),'____',num2str(sum(vsl.keyFrameFlagList)),'__*.png'))))+1)));
            
            
            figure(21);clf;subplot(1,3,1);plot(vsl.MeanErrOld(:,1), vsl.MeanErrOld(:,2),'-o');title('gt - tracking'); axis equal
            hold on;
            for k = 1 : size(vsl.MeanErrOld,1)
                text(double(vsl.MeanErrOld(k,1)),double(vsl.MeanErrOld(k,2)),num2str(k), 'Color',[1 0 0],'FontSize',15);  %,'FontWeight','bold');
            end
            subplot(1,3,2);plot(vsl.MeanErrGT(:,1), vsl.MeanErrGT(:,2),'-o');title('reproj - gt'); axis equal
            hold on;
            for k = 1 : size(vsl.MeanErrGT,1)
                text(double(vsl.MeanErrGT(k,1)),double(vsl.MeanErrGT(k,2)),num2str(k), 'Color',[1 0 0],'FontSize',15);  %,'FontWeight','bold');
            end
            subplot(1,3,3);plot(vsl.MeanErr(:,1), vsl.MeanErr(:,2),'-o');title('reproj - tracking'); axis equal
            hold on;
            for k = 1 : size(vsl.MeanErr,1)
                text(double(vsl.MeanErr(k,1)),double(vsl.MeanErr(k,2)),num2str(k), 'Color',[1 0 0],'FontSize',15);  %,'FontWeight','bold');
            end
            sabf = 1;
        catch
            asbjk = 1;
        end
        if checkGT
            ang1 = PureVision(imgLOrig, imgL, imgR, intrMat0, depthGTOrig, depthGT);
            ang2 = PureVision(imgL, imgLOrig, imgROrig, intrMat0, depthGT, depthGTOrig);
        end
    end
    try
        err = vsl.poseWcsList(:,3) - curKeySeqData.vsl.poseWcsList(:,3);
    catch
        sankbj = 1;
    end
    %     rad2deg(vsl.poseWcsList(:,3)) - curKeySeqData.wheelNew
    drawnow;
end

% % figure(10);
% % saveas(gcf,fullfile(probPath,sprintf('final_error_%05d.fig',length(dir(fullfile(probPath,'final_error_*.fig')))+1)));

saknj  =1;
if 0
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
end