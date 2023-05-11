function ReplicateVSL(inputDir, startt, endd)
global rot_y ShiftLK newShift PrvTrackOnGT ReplaceOnlyP2 NewTracker ReplaceOnlyP2Z NewTracker2 ReplaceGolden newLK newRender...
    OUTPUT_ROOT RECORDED_BIN_ROOT DUMP_ROOT titleStr thetaListGT ShiftByCoord probPath doubleShift mean_x_err rou...
    shiftPixThr doubleShift2 err_x switchMethod rou_temp_list Rou UseDoubleShift...
    UseEx Distri

rot_y = true;
newRender = true;
PrvTrackOnGT = false;  true; false; true;
ShiftLK = true; false; true;
newShift = true; false; true; false; true; false; true;
ShiftInterval = 1; 10; 1; 5; 40; 20; 1; 10; 10;
ReplaceOnlyP2 = true; false; true;
NewTracker = false; true; false; true;
ReplaceOnlyP2Z = false; true;
NewTracker2 = true;
ReplaceGolden = false; true;
Distri = [];

newLK = false; true;


t0 = datetime('now','Format','dd-MMM-y HH:mm:ss');t = datestr(t0);t1 = yyyymmdd(t0);t2 = strsplit(t,' ');
tt = strcat(num2str(t1),'_',t2{2});tt(find(tt == ':')) = '';
probPath = fullfile(inputDir, tt);
MakeDirIfMissing(probPath);


ShiftByCoord = false; true;
doubleShift = false; true; false; true; false;true;
mean_x_err = [0 0];
rou = 10; 7; 10; 7; 10; 7; 10;7; 10; 7; 10;7; 8; 14;9; 14; 9; 7; 12; 10; 7; 10; 7; 6; 10; 7; 10; 7; 5; 8; 9; 10.1;
Rou = 1; 0;  0.5; 0; 1; 0; 1; 1.5;

shiftPixThr = inf; -2; 3; 2;
doubleShift2 = true;
err_x = [0 0];
switchMethod = true; 
rou_temp_list = [];
UseDoubleShift = false; true; false; true; false; true;false; true; false; true;
UseEx = true; false; true; false; true; false; true;


assert(~isempty(OUTPUT_ROOT) && ~isempty(RECORDED_BIN_ROOT) && ~isempty(DUMP_ROOT), 'The directories are not correctly set');
csaParamFilePath = fullfile(DUMP_ROOT, 'coordsys_alignment.txt');
csa = CoordSysAligner;
Load(csa, csaParamFilePath);
robotCfgFilePath = fullfile(OUTPUT_ROOT, 'robot_config.prototxt');
robotConfig = ReadRobotConfig(robotCfgFilePath);

camParamFilePath = fullfile(DUMP_ROOT, 'camera_param.txt');
cam = BCStereoCameraModel(camParamFilePath);
CvtToRectifiedModel(cam);
vsl = VisualLocalizer(cam, csa, robotConfig.tracking_config);
if isempty(vsl.scaleLvl)
    vsl.scaleLvl = 0;
end
vsl_bak = vsl;
try
    load(fullfile(inputDir,'angGT.mat'));
    vsl = vsl_bak;
catch
    sdjfh = 1;
end




titleId = find(inputDir == '=');
titleIdDiff = diff(titleId);
idid = find(titleIdDiff > 1);
titleStr = inputDir(titleId(idid)-18:titleId(idid+1));

dirInfo = dir(fullfile(inputDir, 'Replay*.mat'));

try
    load(fullfile(inputDir, 'judgement.mat'));
catch
    sguk = 1;
end

endd = min(endd, (size(dirInfo,1) - 1));

cnt = 1;

for i = startt : endd
    load(fullfile(inputDir, dirInfo(i).name));
    
    try
        traceX(data{4}(:,1),cnt) = data{4}(:,2);
        traceY(data{4}(:,1),cnt) = data{4}(:,3);
        
        traceX_ref(data{4}(:,1),cnt) = data{4}(:,4);
        traceY_ref(data{4}(:,1),cnt) = data{4}(:,5);
    catch
        asdkhj = 1;
    end
    thetaList(cnt,1) = data{6};
    %     if 0
    depthGT(:,:,cnt) = data{5};
    %     else
    depthVisual(:,:,cnt) = data{3};
    %     end
    imgL = data{1};
    Img(:,:,cnt) = rgb2gray(imgL);
    cnt = cnt + 1;
end
try
    load(fullfile(inputDir,'angWhole0.mat'));
    thetaList00 = deg2rad(angWhole0(1:length(thetaList)));
catch
    sgkj = 1;
end

try
    load(fullfile(inputDir,'angWhole0.mat'));
    traceInfoMat{1,6} = traceInfoMat{1,6}(1:length(thetaList)+1,:);
    traceInfoMat{1,7} = traceInfoMat{1,7}(1:length(thetaList)+1,:);
    thetaList01 = thetaList;
    %     thetaList = AngRef(1:length(thetaList));
catch
    sgkj = 1;
end
thetaList = thetaList - thetaList(1);
try
    thetaListGT = AngRef(1:length(thetaList));
catch
    thetaListGT = thetaList;
end

for i = 1 : size(Img, 3) - 1
   
    if i == 1
        imgPrv = Img(:,:,1);
        imgCur = Img(:,:,2);
        depthPrv = depthVisual(:,:,1);
        depthCur = depthVisual(:,:,2);
        
        vsl.traceManager.X = single([]);
        vsl.traceManager.Y = single([]);
        vsl.traceManager.Z = [];
        vsl.traceManager.ZGT = [];
        
        vsl.traceManager.XLK = single([]);
        vsl.traceManager.YLK = single([]);
        vsl.traceManager.ZLK = [];
        vsl.traceManager.ZGTLK = [];
        
        vsl.traceManager.XShift = single([]);
        vsl.traceManager.YShift = single([]);
        vsl.traceManager.ZShift = [];
        
        
        vsl.accumP2CRef = 0;
        vsl.accumP2CPNP = 0;
        vsl.accumP2CPNP2 = 0;
        vsl.accumP2CPNP2_2 = 0;
        vsl.accumP2CPNP3 = 0;
        vsl.featBatch = {};
        vsl.traceInfoMat = {};
        vsl.TraceBatchListStack = {};
        vsl.dispErrExpStack3 = {};
        
        
        vsl.setting = Setting([]);
        vsl.lookUpTables = LookUpTables([]);
        vsl.modals = Modals([]);
        vsl.points = Points([]);
        
        vsl.traceWithWeights = TraceWithWeights([]);
        
        
        vsl.setting2 = Setting2([]);
        vsl.lookUpTables2 = LookUpTables2([]);
        vsl.modals2 = Modals2([]);
        vsl.points2 = Points2([]);
        
        vsl.traceWithWeights2 = TraceWithWeights2([]);
        
        
    else
        imgPrv = imgCur;
        imgCur = Img(:,:,i + 1);
        depthPrv = depthCur;
        depthCur = depthVisual(:,:,i + 1);
    end
    thetaTemp = thetaList(1:i+1);
    %     doubleShift = false;
    if 0
        DTD(vsl, i, imgPrv, imgCur, depthPrv, depthCur, thetaTemp);
    else
        DTD2(vsl, i, imgPrv, imgCur, depthPrv, depthCur, thetaTemp);
    end
end
% if 0
%         CalibB2C(thetaList, vsl.traceInfoMat,vsl.traceManager.X,vsl.traceManager.Y, vsl.traceManager.XLK, vsl.traceManager.YLK)
% end
end