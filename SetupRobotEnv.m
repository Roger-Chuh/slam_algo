function SetupRobotEnv(timeStr)

global PROJ_ROOT SRC_ROOT DATA_ROOT PARAM_ROOT OUTPUT_ROOT DUMP_ROOT LOG_ROOT RECORD_ROOT DEBUG_ROOT BIN_ROOT
global RECORDED_BIN_ROOT
global DEBUG_DIR
global IMU_BODYSNESOR_RAW_DATA_FIG ...
       GYRO_BIAS_FIG ...
       FILTERED_BODYSENSOR_DATA_FIG ...
       POINT_TRACE_KEY_SEQ_FIG ...
       POINT_TRACE_ADJ_FRAMES_FIG ...
       PATH_FIG ANGLE_BRIDGE_FIG ROBOT_CAM_POSE_FIG ...
       TOPOGRAPHY_FIG ...
       BODY_UNCERT_POLYGON_CONSTR_FIG ...
       KEY2CUR_BODY_PATH_FIG ...
       SPEED_FIG ...
       KALMAN_FLT_MEASURE_ERR_FIG ...
       DEBUG_FIG
global IN_DEBUG_MODE
global RESULT_ROOT
curMFilePath = strsplit(mfilename('fullpath'), filesep);
PROJ_ROOT = fullfile(curMFilePath{1:end-2});

SRC_ROOT = fullfile(PROJ_ROOT, 'src', 'matlab');
DATA_ROOT = fullfile(PROJ_ROOT, 'data');
BIN_ROOT = fullfile(PROJ_ROOT, 'bin');

PARAM_ROOT = fullfile(DATA_ROOT, 'parameter');
RECORD_ROOT = fullfile(DATA_ROOT, 'recorded');

OUTPUT_ROOT = fullfile(PROJ_ROOT, 'output', timeStr);
if (~exist(OUTPUT_ROOT, 'dir'))
    error('Directory %s doesnot exist', OUTPUT_ROOT);
end
DUMP_ROOT = fullfile(OUTPUT_ROOT, 'dump');
addpath(DUMP_ROOT);
LOG_ROOT = fullfile(OUTPUT_ROOT, 'log');
RECORDED_BIN_ROOT = fullfile(OUTPUT_ROOT, 'record');
DEBUG_ROOT = fullfile(OUTPUT_ROOT, 'matlab_debug');
RESULT_ROOT = fullfile(OUTPUT_ROOT, 'result');

DEBUG_DIR = ''; % set when running

POINT_TRACE_KEY_SEQ_FIG = 1;
POINT_TRACE_ADJ_FRAMES_FIG = 2;
IMU_BODYSNESOR_RAW_DATA_FIG = 3;
GYRO_BIAS_FIG = 4;
FILTERED_BODYSENSOR_DATA_FIG = 5;
PATH_FIG = 6;
ANGLE_BRIDGE_FIG = 7;
ROBOT_CAM_POSE_FIG = 8;
TOPOGRAPHY_FIG = 9;
BODY_UNCERT_POLYGON_CONSTR_FIG = 7;
KEY2CUR_BODY_PATH_FIG = 10;
SPEED_FIG = 11;
KALMAN_FLT_MEASURE_ERR_FIG = 12;
DEBUG_FIG = 20;

IN_DEBUG_MODE = false;

addpath(genpath('./EpilineLK')); % this function is for 1D optical flow using epilineLK method
addpath(genpath('./GlobalPlanner')); 
addpath(genpath('./Tracing')); 
addpath(genpath('./Tracing_Bk_Mod')); 
addpath(genpath('./Renderer'));

addpath(genpath('./rgbd-vo'));
addpath(genpath('./matlab-orb-slam-master'));
addpath(genpath('./Dense-VO-master'));
addpath(genpath('./LSD_SLAM_Matlab-master'));
addpath(genpath('./pose-graph-visual-odometry-master'));
addpath(genpath('./cvo-rgbd-master'));
addpath(genpath('./Depth-Estimation-from-Stereo-and-Video-master'));
addpath(genpath('./nanhuCalib_bak_20200921'));


addpath(genpath('./BundleAdjustmentMatlab-master'));
addpath(genpath('./lmatch'));


addpath(genpath('./Robust'));
addpath(genpath('./ICRA2018_Code'));
% % addpath(genpath('./TracingBak')); 




% Compile ReadRobotConfigMex
% mex -I'C:\Users\lingc\Desktop\nextvpu\nextvpu_program\SLAM'...
%     -I'C:\Users\lingc\Desktop\nextvpu\nextvpu_program\SLAM\protobuf-3.6.1_release_MD_DR\include'...
%     -I'C:\Users\lingc\Desktop\nextvpu\nextvpu_program\SLAM\RobotCofigPb'...
%     -I'C:\opencv3.4x64\build\include\opencv'...
%     -I'C:\opencv3.4x64\build\include'...
%     -L'C:\Users\lingc\Desktop\nextvpu\nextvpu_program\SLAM\ioLib\dllmakerForioLib\x64\Release'...
%     -l'iolib.lib'...
%     -L'C:\Users\lingc\Desktop\nextvpu\nextvpu_program\SLAM\protobuf-3.6.1_release_MD_DR\lib'...    
%     -l'libprotobuf.lib'...
%     -l'libprotobuf-lite.lib'...
%     -l'libprotoc.lib'...
%     -L'C:\opencv3.4x64\build\x64\vc14\lib'...
%     -l'opencv_world340.lib'...
%     -L'C:\Users\lingc\Desktop\nextvpu\nextvpu_program\SLAM\RobotConfigLibs\x64\Release'...
%     -l'RobotConfigLibs.lib'...
%     'RobotCofigReader/ReadRobotConfigMex.cpp'


end