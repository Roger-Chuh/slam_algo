function VSL6Dof(inputDir, startt, endd, skipNum)
global probPath ReplaceOnlyP2 ShiftLK vslMat vslMatPNP num_levels deOutlierThr areaRatio initial_sigma default_dof para...
    yMatPrv yMatCur yRng zRng xRng2 zRng2 yRng2 ...
    OUTPUT_ROOT RECORDED_BIN_ROOT DUMP_ROOT...
    State Params Debug Map MatchRatio inputDir0 frameList dirInfo


% load('D:\Temp\20200831\rMatNew.mat')

Draw = 0;

ros = false; true;false; true; false; true;
simu = false;true; false; true; false; true;
  
kitti = true; false; true; false; true; 

if 1
    extDepthDir = [];
else
    extDepthDir = 'D:\Work\project1_IM_2018-2019-master\project1_IM_2018-2019-master\src\output_smallest_bundle_00_opt';
    extDepthScale = 1;
    
    extDepthDir = 'D:\Work\project1_IM_2018-2019-master\project1_IM_2018-2019-master\src\output_smallest_bundle_simu1_opt';
    extDepthScale = 0;
    
    extDepthDir = 'D:\Work\project1_IM_2018-2019-master\project1_IM_2018-2019-master\src\output_smallest_bundle_ros_differ_opt';
    
    extDepthScale = 0;
end


if ~isempty(extDepthDir)
   
    extDepthDirInfo = dir(fullfile(extDepthDir, '*.png'));
    
    xRng2 = 1000.*[15 -15];
    zRng2 = 1000.*[50 15];
    yRng2 = 1000.*[15 -15];
    
    
    if ros
        xRng2 = 1000.*[15 -15];
        zRng2 = 1000.*[25 0.5];
        yRng2 = 1000.*[0 -6];
    end
    
end



if kitti
    dInfo = dir(fullfile(inputDir,'image*'));
    leftInfo = dir(fullfile(inputDir,dInfo(1).name, '*.png'));
    rightInfo = dir(fullfile(inputDir,dInfo(2).name, '*.png'));
    gtInfo = dir(fullfile(inputDir,'*.txt'));
    groundtruth0 = load(fullfile(inputDir, gtInfo(1).name));
    groundtruth = [];
    for i = 1 : size(groundtruth0,1)
        rt0 = reshape(groundtruth0(i,:),4,3)';
        groundtruth(i,:) = [reshape(rt0(1:3,1:3),1,9) rt0(1:3,4)'];
    end
end



if ros
    gtD = true; false; true;
    gtDepth = false; true; false; true; false;
else
    gtD = false; true; false; true;
    gtDepth = false;
    
end
inputDir0 = inputDir;
ReplaceOnlyP2 = false;
ShiftLK = false;
vslMat = [reshape(eye(3),1,9) 0 0 0];
vslMatPNP = [reshape(eye(3),1,9) 0 0 0];
MatchRatio = [];
frame_prev = [];


deOutlierThr = 1; 2;  1;
num_levels = 2; 4; 2; 3; 2; 4; 3; 4; 2; 3; 1; 2; 4;  6; 4; 2;
areaRatio = 0.001; 0.08; 0.25; 0.08; 0.005;
initial_sigma = 5;
default_dof = 5;
para = 5;

if ~simu
    if kitti
        num_levels = 3; 2; 4; 2; 3; 2; 4; 3; 4; 2; 3; 1; 2; 4;  6; 4; 2;
        
        if 0
            yRng = 1000.*[2 -4];
            zRng = 1000.*[250 0];
            depthThr = 250*1000; 140*1000; 100*1000; 100*1000;  50*1000;  20*1000; 200*1000; 25*1000; 50*1000;
        else
           %% 20210318 office010
            yRng = 1000.*[20 -40];
            zRng = 1000.*[7 0];
            depthThr = 7*1000; 140*1000; 100*1000; 100*1000;  50*1000;  20*1000; 200*1000; 25*1000; 50*1000;
        
            
        end
    elseif 1
        if ~ros
            num_levels = 2; 3;
            yRng = 1000.*[150 -6];
            zRng = 1000.*[50 0];
        else
            yRng = 1000.*[0 -2.5]; % 1000.*[0 -6];
            zRng = 1000.*[20 0];
            num_levels = 3;
        end
        depthThr = 50*1000; 140*1000; 100*1000; 100*1000;  50*1000;  20*1000; 200*1000; 25*1000; 50*1000;
        
    elseif 	1
        num_levels = 3; 2; 4; 2; 3; 2; 4; 3; 4; 2; 3; 1; 2; 4;  6; 4; 2;
        yRng = 1000.*[15 -3.5];
        zRng = 1000.*[150 0];
        depthThr = 150*1000; 140*1000; 100*1000; 100*1000;  50*1000;  20*1000; 200*1000; 25*1000; 50*1000;
        
    elseif 1 % 083500 case
        num_levels = 3; 2; 4; 2; 3; 2; 4; 3; 4; 2; 3; 1; 2; 4;  6; 4; 2;
        yRng = 1000.*[1.5 -1113.5];
        zRng = 1000.*[150 0];
        depthThr = 150*1000; 140*1000; 100*1000; 100*1000;  50*1000;  20*1000; 200*1000; 25*1000; 50*1000;
        
        
    
    else % simu case
        num_levels = 3; 2; 4; 2; 3; 2; 4; 3; 4; 2; 3; 1; 2; 4;  6; 4; 2;
        yRng = 1000.*[250 -300.5];
        zRng = 1000.*[150 0];
        depthThr = 150*1000; 140*1000; 100*1000; 100*1000;  50*1000;  20*1000; 200*1000; 25*1000; 50*1000;
    end
    
else
    num_levels = 3; 2; 4; 3; 2; 3; 4; 2; 3; 2; 4; 2; 3; 2; 4; 3; 4; 2; 3; 1; 2; 4;  6; 4; 2;
    yRng = 1000.*[250 -3000.5];
    zRng = 1000.*[50 0];
    depthThr = 50*1000; 140*1000; 100*1000; 100*1000;  50*1000;  20*1000; 200*1000; 25*1000; 50*1000;
end
depthThr2 = 0; 30*1000;
xThr = 1500*1000; 150*1000;

if ~simu
    yThr = 120*1000;5000*1000; 5.0*1000; 20000*1000; 2.0*1000;  1.0*1000; 1.30*1000; 200*1000; 2*1000; 2000*1000; 2*1000; 20*1000;  0;
    yThr2 = -600*1000; 0;-20*1000;0;-10*1000;-200000000; 1*1000; -2*1000; -500.5*1000; -80.*1000;-1000000; 
else
    yThr = 1200*1000;5000*1000; 5.0*1000; 20000*1000; 2.0*1000;  1.0*1000; 1.30*1000; 200*1000; 2*1000; 2000*1000; 2*1000; 20*1000;  0;
    yThr2 = -6000*1000; 0;-20*1000;0;-10*1000;-200000000; 1*1000; -2*1000; -500.5*1000; -80.*1000;-1000000;
end


if ros
    yThr = 0; 1200*1000;5000*1000; 5.0*1000; 20000*1000; 2.0*1000;  1.0*1000; 1.30*1000; 200*1000; 2*1000; 2000*1000; 2*1000; 20*1000;  0;
    yThr2 = -6000*1000; 0;-20*1000;0;-10*1000;-200000000; 1*1000; -2*1000; -500.5*1000; -80.*1000;-1000000;
    
end

randPtNum = 1000; 8000; 50*1000;
mergeSize = 20; 50; 150;
mergeStep = 10; 2; 10; 2; 20; 1;

if ~simu
    zShow = depthThr2 + 20*1000;
else
    zShow = depthThr2 + 200*1000;
end

t0 = datetime('now','Format','dd-MMM-y HH:mm:ss');t = datestr(t0);t1 = yyyymmdd(t0);t2 = strsplit(t,' ');
tt = strcat(num2str(t1),'_',t2{2});tt(find(tt == ':')) = '';
probPath = fullfile(inputDir, tt);
% MakeDirIfMissing(probPath);


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



if ~kitti
dirInfo = dir(fullfile(inputDir, 'Replay*.mat'));


[~, ind1] = sort([dirInfo(1:length(dirInfo)).datenum], 'ascend');
% [~, ind2] = sort([dirInfo(length(dirInfo)/2+1:end).datenum], 'ascend');
% ind2 = ind2 + length(dirInfo)/2;
% I = [ind1 ind2];
I = [ind1];

tempInfo = cell(length(I),1);
for fg = 1:length(I)
    tempInfo{fg,1} = dirInfo(I(fg)).name;
end
for fgg = 1:length(I)
    dirInfo(fgg).name = tempInfo{fgg};
end






% intrMat = [329.115520046, 0,             320.0;...
%  0,             329.115520046, 240.0;...
%  0,             0,                 1]; 
% 
% princpPtL = intrMat(1:2, 3)';
% princpPtR = intrMat(1:2, 3)';
% 
% b2c = eye(4);


endd = min(endd, (size(dirInfo,1) - 1));

else
    endd = min(endd, (size(leftInfo,1) - 1));
    dirInfo = leftInfo;
end


if ~isempty(extDepthDir)
    if kitti
        endd = min(endd, (size(extDepthDirInfo,1) - 1));
    end
end

frameList = startt : skipNum : endd;


intrMat = Get(vsl.camModel, 'PinholeIntrMat', 1, vsl.scaleLvl);
intrMat0 = Get(vsl.camModel, 'PinholeIntrMat', 1, 0);


[princpPtL, princpPtR] = Get(vsl.camModel, 'PinholePrincpPt',  vsl.scaleLvl);
L2R = [rodrigues(vsl.camModel.rotVec1To2) vsl.camModel.transVec1To2; 0 0 0 1];


intr = [vsl.camModel.monoCamModel1.focLen(1) 0 vsl.camModel.monoCamModel1.princpPt(1); 0 vsl.camModel.monoCamModel1.focLen(2) vsl.camModel.monoCamModel1.princpPt(2); 0 0 1];
cameraParams = cameraParameters('WorldUnits', 'mm',...
    'IntrinsicMatrix', intr', ...
    'RotationVectors', zeros(1, 3), ...
    'TranslationVectors', [0 0 0], 'RadialDistortion', vsl.camModel.monoCamModel1.distCoeff([1 2 5])','TangentialDistortion', vsl.camModel.monoCamModel1.distCoeff([3 4])');


codewords = load('codewords.mat');
codewords = codewords.codewords;

State.mu = [0;0;0];
State.Sigma = zeros(length(State.mu));

Params.theta = 15; % Number of shared observations a keyframe must have to be considered the same map points
Params.theta_min = 100; % Defines high covisability for spanning tree
Params.keyFramePercentOfBestScoreThreshold = 75; % bag of words returns keyframes that are more than this percentage of the best match
Params.cameraParams = cameraParams; % load_camera_params(calibFile, cameraID);
Params.minMatchesForConnection = 50;
% ADD number of features to say we didn't lose localization
% ADD angle threshold between v and n
% ADD scale invariance region - perhaps set from data set

Params.cullingSkip = 25;
Params.cullingThreshold = 0.9;

Params.kdtree = KDTreeSearcher(codewords);
Params.numCodewords = size(codewords, 1);
Params.numFramesApart = 20;

Params.numViewsToLookBack = 5;
Params.minMatchRatioRatio = 0.7; 0.3; 0.4;

Params.numSkip = 2;1;2;
Params.deletedframes = [];

Debug.displayFeaturesOnImages = false;

Map.covisibilityGraph = ourViewSet();

[xGrid0,yGrid0] = meshgrid(1:ceil(vsl.camModel.imgSize(2)), 1:ceil(vsl.camModel.imgSize(1)));
Pix0 = [xGrid0(:) yGrid0(:)];




[xGrid,yGrid] = meshgrid(1:ceil(vsl.camModel.imgSize(2)*(1/2^vsl.scaleLvl)), 1:ceil(vsl.camModel.imgSize(1)*(1/2^vsl.scaleLvl)));
Pix = [xGrid(:) yGrid(:)];
metricPrevPtCcsGT = intrMat\HomoCoord(Pix',1);
metricPrevPtCcsGT = normc(metricPrevPtCcsGT);
if simu
    scaleMat = reshape(metricPrevPtCcsGT(3,:), vsl.camModel.imgSize(1)*(1/2^vsl.scaleLvl), vsl.camModel.imgSize(2)*(1/2^vsl.scaleLvl));
else
    scaleMat = ones(size(xGrid));
end
if round(intrMat(1)) == 554 || intrMat(1) > 900
    scaleMat = ones(size(scaleMat));
    b2cPmat = GetPctBody2Cam(vsl.coordSysAligner, 1);
    b2c = b2cPmat.transformMat;
    if intrMat(1) > 900
        b2c = eye(4);
    end
    mergeSize = 50; 20; 20;
    if 0 % 20200928
        gtDepth = gtD;
    end
else
     b2c = eye(4);
end

if simu
    mergeStep = 20;
    mergeSize = 200;
else
    mergeSize = 80;
end
if ros
    mergeSize = 50;
else
    mergeSize = 200;
end
poseGT = [];
pt3d = {};
for i = 1 : length(frameList) - 1
    
    if i == 1
        if ~kitti
            load(fullfile(inputDir, dirInfo(frameList(i)).name));

            if ~isempty(extDepthDir)
                imgl = data{1};
                imgr = data{2};
                if 0
                    depth11 = scaleMat.*data{3};
                    disp11 = intrMat(1) * norm(L2R(1:3,4)) ./ depth11 - (princpPtR(1) - princpPtL(1));
                else
                    [depth11, disp11] = GetDepthMap(vsl, data{1}, data{2});
                end
                
                depth110 = depth11;
                dispNew = imresize(single(imread(fullfile(extDepthDir, extDepthDirInfo((i)).name))),2^extDepthScale);
                depth11 = ReplaceDepth(depth110, disp11, dispNew, imgl, intrMat0, intrMat, Pix0, norm(L2R(1:3,4)), princpPtL, princpPtR);
                data{3} = depth11./scaleMat;
            end
   
        else
            data = {};
            imgl = imread(fullfile(inputDir,dInfo(1).name, leftInfo(frameList(i)).name));
            imgr = imread(fullfile(inputDir,dInfo(2).name, rightInfo(frameList(i)).name));
            data{1,1} = cat(3, imgl, imgl, imgl);
            data{1,2} = cat(3, imgr, imgr, imgr);
            data{1,6} = [];
%             depth11 = GetDepthMap(vsl, imresize(data{1},1/2^vsl.scaleLvl), imresize(data{2},1/2^vsl.scaleLvl));
            [depth11, disp11] = GetDepthMap(vsl, data{1}, data{2});
            depth110 = depth11;
            if ~isempty(extDepthDir)
                dispNew = imresize(single(imread(fullfile(extDepthDir, extDepthDirInfo((i)).name))),2^extDepthScale);
                depth11 = ReplaceDepth(depth110, disp11, dispNew, imgl, intrMat0, intrMat, Pix0, norm(L2R(1:3,4)), princpPtL, princpPtR);
            end
            
            
            
            data{3} = depth11;
            try
                data{1,7} = groundtruth(frameList(i),:);
            catch
            end
        end
        if isempty(data{5})
            data{5} = data{3};
        end
        if length(data) < 7
            data{1,7} = [reshape(eye(3),1,9) 0 0 0];
        end
        if size(data{7},1) > 1
            Tcam = b2c * [roty(double(rad2deg(-data{6}))) [0 0 0]'; 0 0 0 1] * inv(b2c);
            data{7} = [reshape(Tcam(1:3,1:3),1,9) Tcam(1:3,4)'];
        end
        if gtDepth
            data{3} = scaleMat.*imresize(data{5},1/2^vsl.scaleLvl);
        else
%             data{3} = scaleMat.*imresize(data{3},1/2^vsl.scaleLvl);
            try 
                try
                    depth1 = GetDepthMap(vsl, ImgTransform(imresize(data{1},1/2^vsl.scaleLvl),intrMat, v2c), ImgTransform(imresize(data{2},1/2^vsl.scaleLvl),intrMat, v2c));
                catch
                    depth1 = GetDepthMap(vsl, imresize(data{1},1/2^vsl.scaleLvl), imresize(data{2},1/2^vsl.scaleLvl));
                end
                data{3} = depth1;
            catch
                data{3} = scaleMat.*imresize(data{3},1/2^vsl.scaleLvl);
            end
        end
        %         if kitti
        %            data{5} = data{3};
        %         end
        
        if ~isempty(extDepthDir)
            data{3} = imresize(depth11, 1/2^vsl.scaleLvl);
        end


        imgPrv = imresize(data{1},1/2^vsl.scaleLvl);
        try
            imgPrv = ImgTransform(imgPrv, intrMat, v2c);
        catch
        end
        
        
        depthPrv = data{3};
        depthPrvGT = scaleMat.*imresize(data{5},1/2^vsl.scaleLvl);
        [XYZ_] = GetXYZFromDepth(intrMat, Pix,depthPrv(:));xMat = reshape(XYZ_(:,1), size(depthPrv));
        yMatPrv = reshape(XYZ_(:,2), size(depthPrv));

        depthPrv(depthPrv > depthThr | depthPrv < depthThr2) = nan;
        depthPrv(abs(xMat) > xThr) = nan;
        depthPrv(yMatPrv > yThr) = nan;
        depthPrv(yMatPrv < yThr2) = nan;
        depthPrv(depthPrv < 0) = nan;
        depthPrvGT(isnan(depthPrv)) = nan;
        
        depthPrv = RemoveDepthHoles(depthPrv);
        depthPrvGT = RemoveDepthHoles(depthPrvGT);
        
        
        
        poseGT = [poseGT; data{7}];
        
        Tt1 = [reshape(poseGT(1,1:9),3,3) poseGT(1,10:12)'; 0 0 0 1];
        Tinit = inv(Tt1);
        T_transformed = Tinit*Tt1;
        poseGT(end,:) = [reshape(T_transformed(1:3,1:3),1,9) T_transformed(1:3,4)'];
        
        if ~kitti
            load(fullfile(inputDir, dirInfo(frameList(i+1)).name));
            
            if ~isempty(extDepthDir)
                imgl = data{1};
                imgr = data{2};
                if 0
                    depth11 = scaleMat.*data{3};
                    disp11 = intrMat(1) * norm(L2R(1:3,4)) ./ depth11 - (princpPtR(1) - princpPtL(1));
                else
                    [depth11, disp11] = GetDepthMap(vsl, data{1}, data{2});
                end
                
                depth110 = depth11;
                dispNew = imresize(single(imread(fullfile(extDepthDir, extDepthDirInfo((i+1)).name))),2^extDepthScale);
                depth11 = ReplaceDepth(depth110, disp11, dispNew, imgl, intrMat0, intrMat, Pix0, norm(L2R(1:3,4)), princpPtL, princpPtR);
                data{3} = depth11./scaleMat;
            end
        else
            data = {};
            imgl = imread(fullfile(inputDir,dInfo(1).name, leftInfo(frameList(i+1)).name));
            imgr = imread(fullfile(inputDir,dInfo(2).name, rightInfo(frameList(i+1)).name));
            data{1,1} = cat(3, imgl, imgl, imgl);
            data{1,2} = cat(3, imgr, imgr, imgr);
            data{1,6} = [];
%             depth11 = GetDepthMap(vsl, imresize(data{1},1/2^vsl.scaleLvl), imresize(data{2},1/2^vsl.scaleLvl));
            [depth11, disp11] = GetDepthMap(vsl, data{1}, data{2});
            
            depth110 = depth11;
            if ~isempty(extDepthDir)
                dispNew = imresize(single(imread(fullfile(extDepthDir, extDepthDirInfo(frameList(i+1)).name))),2^extDepthScale);
                depth11 = ReplaceDepth(depth110, disp11, dispNew, imgl, intrMat0, intrMat, Pix0, norm(L2R(1:3,4)), princpPtL, princpPtR);
            end
            
            
            
            data{3} = depth11;
            try
                data{1,7} = groundtruth(frameList(i+1),:);
            catch
            end
        end
        if isempty(data{5})
            data{5} = data{3};
        end
        if length(data) < 7
            data{1,7} = [reshape(eye(3),1,9) 0 0 0];
        end
        if size(data{7},1) > 1
            Tcam = b2c * [roty(double(rad2deg(-data{6}))) [0 0 0]'; 0 0 0 1] * inv(b2c);
            data{7} = [reshape(Tcam(1:3,1:3),1,9) Tcam(1:3,4)'];
        end
        if gtDepth
            data{3} = scaleMat.*imresize(data{5},1/2^vsl.scaleLvl);
        else
%             data{3} = scaleMat.*imresize(data{3},1/2^vsl.scaleLvl);
            try 
                try
                    depth1 = GetDepthMap(vsl, ImgTransform(imresize(data{1},1/2^vsl.scaleLvl),intrMat, v2c), ImgTransform(imresize(data{2},1/2^vsl.scaleLvl),intrMat, v2c));
                    
                catch
                    depth1 = GetDepthMap(vsl, imresize(data{1},1/2^vsl.scaleLvl), imresize(data{2},1/2^vsl.scaleLvl));
                end
                data{3} = depth1;
            catch
                data{3} = scaleMat.*imresize(data{3},1/2^vsl.scaleLvl);
            end
        end
        
        if ~isempty(extDepthDir)
            data{3} = imresize(depth11, 1/2^vsl.scaleLvl);
        end
        
        
        imgCur = imresize(data{1},1/2^vsl.scaleLvl);
        try
            imgCur = ImgTransform(imgCur, intrMat, v2c);
        catch
        end
        
        
        depthCur = data{3};
        depthCurGT = scaleMat.*imresize(data{5},1/2^vsl.scaleLvl);
        [XYZ_] = GetXYZFromDepth(intrMat, Pix,depthCur(:));xMat = reshape(XYZ_(:,1), size(depthCur));
        yMatCur = reshape(XYZ_(:,2), size(depthCur));

        depthCur(depthCur > depthThr | depthCur < depthThr2) = nan;
        depthCur(abs(xMat) > xThr) = nan;
        depthCur(yMatCur > yThr) = nan;
        depthCur(yMatCur < yThr2) = nan;
        depthCur(depthCur < 0) = nan;
        
        depthCurGT(isnan(depthCur)) = nan;
        
         depthCur = RemoveDepthHoles(depthCur);
        depthCurGT = RemoveDepthHoles(depthCurGT);
        
        
        
        poseGT = [poseGT; data{7}];
        Tt2 = [reshape(poseGT(end,1:9),3,3) poseGT(end,10:12)'; 0 0 0 1];
        T_transformed = Tinit*Tt2;
        poseGT(end,:) = [reshape(T_transformed(1:3,1:3),1,9) T_transformed(1:3,4)'];
        
        
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
        if 0
        [descriptors, points] = extract_features(imgPrv);
        bow = calc_bow_repr(descriptors, Params.kdtree, Params.numCodewords);
        
        Map.covisibilityGraph = addView(Map.covisibilityGraph, 1,...
            descriptors, points, bow, 'Points', points, ...
            'Orientation', eye(3), 'Location', zeros(1, 3));
        else
            surf_slam2(rgb2gray(imgPrv), 1);
            
        end
        
        
    else
        yMatPrv = yMatCur;
        imgPrv = imgCur;
        depthPrv = depthCur;
        depthPrvGT = depthCurGT;
        if ~kitti
            load(fullfile(inputDir, dirInfo(frameList(i+1)).name));
            
            if ~isempty(extDepthDir)
                imgl = data{1};
                imgr = data{2};
                if 0
                    depth11 = scaleMat.*data{3};
                    disp11 = intrMat(1) * norm(L2R(1:3,4)) ./ depth11 - (princpPtR(1) - princpPtL(1));
                else
                    [depth11, disp11] = GetDepthMap(vsl, data{1}, data{2});
                end
                
                depth110 = depth11;
                dispNew = imresize(single(imread(fullfile(extDepthDir, extDepthDirInfo((i+1)).name))),2^extDepthScale);
                depth11 = ReplaceDepth(depth110, disp11, dispNew, imgl, intrMat0, intrMat, Pix0, norm(L2R(1:3,4)), princpPtL, princpPtR);
                data{3} = depth11./scaleMat;
            end
        else
            data = {};
            imgl = imread(fullfile(inputDir,dInfo(1).name, leftInfo(frameList(i+1)).name));
            imgr = imread(fullfile(inputDir,dInfo(2).name, rightInfo(frameList(i+1)).name));
            data{1,1} = cat(3, imgl, imgl, imgl);
            data{1,2} = cat(3, imgr, imgr, imgr);
            data{1,6} = [];
            %             depth11 = GetDepthMap(vsl, imresize(data{1},1/2^vsl.scaleLvl), imresize(data{2},1/2^vsl.scaleLvl));
            [depth11, disp11] = GetDepthMap(vsl, data{1}, data{2});
            
            depth110 = depth11;
            if ~isempty(extDepthDir)
                dispNew = imresize(single(imread(fullfile(extDepthDir, extDepthDirInfo(frameList(i+1)).name))),2^extDepthScale);
                depth11 = ReplaceDepth(depth110, disp11, dispNew, imgl, intrMat0, intrMat, Pix0, norm(L2R(1:3,4)), princpPtL, princpPtR);
            end
            
            
            
            data{3} = depth11;
            try
                data{1,7} = groundtruth(frameList(i+1),:);
            catch
            end
        end
        if isempty(data{5})
            data{5} = data{3};
        end
        if length(data) < 7
            data{1,7} = [reshape(eye(3),1,9) 0 0 0];
        end
        
        if size(data{7},1) > 1
            Tcam = b2c * [roty(double(rad2deg(-data{6}))) [0 0 0]'; 0 0 0 1] * inv(b2c);
            data{7} = [reshape(Tcam(1:3,1:3),1,9) Tcam(1:3,4)'];
        end
        if gtDepth
            data{3} = scaleMat.*imresize(data{5},1/2^vsl.scaleLvl);
        else
            try 
                try
                    depth1 = GetDepthMap(vsl, ImgTransform(imresize(data{1},1/2^vsl.scaleLvl),intrMat, v2c), ImgTransform(imresize(data{2},1/2^vsl.scaleLvl),intrMat, v2c));
                catch
                    depth1 = GetDepthMap(vsl, imresize(data{1},1/2^vsl.scaleLvl), imresize(data{2},1/2^vsl.scaleLvl));
                end
                data{3} = depth1;
            catch
                data{3} = scaleMat.*imresize(data{3},1/2^vsl.scaleLvl);
            end
        end
        
        if ~isempty(extDepthDir)
            data{3} = imresize(depth11, 1/2^vsl.scaleLvl);
        end
        
        
        imgCur = imresize(data{1},1/2^vsl.scaleLvl);
        try
            imgCur = ImgTransform(imgCur, intrMat, v2c);
        catch
        end
        depthCur = data{3};
        
        depthCurGT = scaleMat.*imresize(data{5},1/2^vsl.scaleLvl);
        

        [XYZ_] = GetXYZFromDepth(intrMat, Pix,depthCur(:));xMat = reshape(XYZ_(:,1), size(depthCur));
        yMatCur = reshape(XYZ_(:,2), size(depthCur));

        depthCur(depthCur > depthThr | depthCur < depthThr2) = nan;
        depthCur(abs(xMat) > xThr) = nan;
        depthCur(yMatCur > yThr) = nan;
        depthCur(yMatCur < yThr2) = nan;
        depthCur(depthCur < 0) = nan;
        depthCurGT(isnan(depthCur)) = nan;
        
        depthCur = RemoveDepthHoles(depthCur);
        depthCurGT = RemoveDepthHoles(depthCurGT);

        poseGT = [poseGT; data{7}];
        Tt2 = [reshape(poseGT(end,1:9),3,3) poseGT(end,10:12)'; 0 0 0 1];
        T_transformed = Tinit*Tt2;
        poseGT(end,:) = [reshape(T_transformed(1:3,1:3),1,9) T_transformed(1:3,4)'];
    end
    
    
%     yMat = yMatPrv;
    
    if 0
        DTD(vsl, i, imgPrv, imgCur, depthPrv, depthCur, thetaTemp);
    elseif 0
        DTD2(vsl, i, imgPrv, imgCur, depthPrv, depthCur, thetaTemp);
    elseif 1
        DTD6Dof(vsl, i, imgPrv, imgCur, depthPrv, depthCur, poseGT, depthPrvGT, depthCurGT);
        if ~kitti
            surf_slam2(rgb2gray(imgCur), i+1);
        end
    end
    
%     yMatPrv = yMatCur;
    
    
    L2G = [reshape(poseGT(end,1:9),3,3) poseGT(end,10:12)';0 0 0 1];
    L2G = [reshape(vslMat(end,1:9),3,3) vslMat(end,10:12)';0 0 0 1];
    L2G0 = L2G;
    [UU,SS,VV] = svd(L2G(1:3,1:3));
    L2G(1:3,1:3) = UU*VV';
    
    if i == 1
        [XYZ_key] = GetXYZFromDepth(intrMat, Pix,depthPrv(:));
        randList = randperm(size(Pix,1));
        tempXYZ = XYZ_key(randList(1:randPtNum),:);
        pt3d = [pt3d; {tempXYZ(~isnan(tempXYZ(:,3)),:)}];
%         [XYZ_cur] = GetXYZFromDepth(intrMat, Pix,depthCur(:));
%         [~, XYZ_cur_in_key] = TransformAndProject(XYZ_Prv_all, intrMat, L2G(1:3,1:3), L2G(1:3,4));
%         randList = randperm(size(Pix,1));
%         pt3d = [pt3d; {XYZ_cur_in_key(randList(1:randPtNum),:)}];
    end
    [XYZ_cur] = GetXYZFromDepth(intrMat, Pix,depthCur(:));
    [~, XYZ_cur_in_key] = TransformAndProject(XYZ_cur, intrMat, L2G(1:3,1:3), L2G(1:3,4));
    randList = randperm(size(Pix,1));
    tempXYZ = XYZ_cur_in_key(randList(1:randPtNum),:);
    pt3d = [pt3d; {tempXYZ(~isnan(tempXYZ(:,3)),:)}];
    %     pt3d = [pt3d; {XYZ_cur_in_key(randList(1:randPtNum),:)}];
    rgbMat = [reshape(imgCur(:,:,1),[],1) reshape(imgCur(:,:,2),[],1) reshape(imgCur(:,:,3),[],1)];
    
    ptCloudCur = pointCloud(XYZ_cur(XYZ_cur(:,3) < zShow,:),'Color',rgbMat(XYZ_cur(:,3) < zShow,:));
    ptCloudAligned = pctransform(ptCloudCur, affine3d(L2G'));
    if i == 1
        
        rgbMatKey = [reshape(imgPrv(:,:,1),[],1) reshape(imgPrv(:,:,2),[],1) reshape(imgPrv(:,:,3),[],1)];
        ptCloudScene = pointCloud(XYZ_key(XYZ_key(:,3) < zShow,:),'Color',rgbMatKey(XYZ_key(:,3) < zShow,:));
        figure(33),clf;hold on; grid on;
        hAxes = pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
        title('Updated world scene')
        % Set the axes property for faster rendering
        hAxes.CameraViewAngleMode = 'auto';
        hScatter = hAxes.Children;
    end
    
    ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergeSize);
    ptCloudScene = pcdownsample(ptCloudScene,'gridAverage',mergeSize);
    
    if Draw
        try
            if mod(i, mergeStep) == 0
% %                 ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergeSize);
% %                 ptCloudScene = pcdownsample(ptCloudScene,'gridAverage',mergeSize);
                hScatter.XData = ptCloudScene.Location(:,1);
                hScatter.YData = ptCloudScene.Location(:,2);
                hScatter.ZData = ptCloudScene.Location(:,3);
                hScatter.CData = ptCloudScene.Color;
                figure(33),plot3(vslMat(:,10),vslMat(:,11),vslMat(:,12),'-r','MarkerSize', 3, 'lineWidth', 3);
                drawnow('limitrate');
            end
        catch
            kjfgh = 1;
        end
    end
    
end



drawnow;
Pt3d = cell2mat(pt3d(1:end,:));figure,pcshow(Pt3d);plotPath(vslMat(1:5:end,:));
MakeDirIfMissing(probPath);
saveas(gcf,fullfile(probPath,'1.fig'));
vslMat4 = vslMat;
vslMatPNP4 = vslMatPNP;
frameList4 = frameList;
save(fullfile(probPath,'vslMat4.mat'),'vslMat4','frameList4','vslMatPNP4');
copyfile(fullfile(pwd,'VSL6Dof.m'), fullfile(probPath));
copyfile(fullfile(pwd,'DTD6Dof.m'), fullfile(probPath));
copyfile(fullfile(pwd,'rgbd-vo','estimateVisualOdometry.m'), fullfile(probPath));


poseGT_temp = [];
for ll = 1:length(frameList)
   tempPose = [reshape(groundtruth(frameList(ll),1:9),3,3) groundtruth(frameList(ll),10:12)'; 0 0 0 1];
   if ll == 1 %frameList(1)       
       basePoseGT = inv(tempPose);
   end
   tempPose2 = basePoseGT*tempPose;
    poseGT_temp(ll,:) = [reshape(tempPose2(1:3,1:3),1,9) tempPose2(1:3,4)'];
end

% pgt = poseGT_temp(:,10:12)*roty(2);figure,plot(1.*pgt(:,1),1.*pgt(:,3));axis equal;hold on;plot(vslMat(:,10)./1000,vslMat(:,12)./1000);plot(vslMatPNP(:,10)./1000,vslMatPNP(:,12)./1000);legend('gt','vo','pnp');
pgt = poseGT(:,10:12)*roty(2);figure,plot(1.*pgt(:,1),1.*pgt(:,3));axis equal;hold on;plot(vslMat(:,10)./1000,vslMat(:,12)./1000);plot(vslMatPNP(:,10)./1000,vslMatPNP(:,12)./1000);legend('gt','vo','pnp');



Tf = reprojection(vslMat(:,10:12)'./1000, poseGT(:,10:12)', 1);
xyz_opt = Tf(1:3, 1:3) * vslMat(:,10:12)'./1000 + Tf(1:3, 4);
figure(2)
plot(poseGT(:,10), poseGT(:,12), 'g', ...
xyz_opt(1,:), xyz_opt(3,:), 'k', ...
'linewidth', 2)
axis equal
axis(axis() + [-1, 1, -1, 1] * 10)
legend({'Ground Truth', 'Result'}, 'location', 'best')



PrepareMVS(probPath,inputDir, intrMat, vslMat, [1 size(vslMat, 1)], dirInfo, frameList, scaleMat, kitti)
PrepareMVS(probPath,inputDir, intrMat, vslMat, [1 size(vslMat, 1)], dirInfo, frameList, scaleMat, kitti, dInfo)

figure(33);
saveas(gcf,fullfile(probPath,'2.fig'));



% % fid1 = fopen(fullfile(probPath, 'pose.txt'),'w');
% % 
% % 
% % for kk = 1 : size(poseGT,1)
% %     for jj = 1 : size(poseGT,2)
% %         if jj == 1
% %             fprintf(fid1, '%d ', poseGT(kk,jj));
% %         else
% %             fprintf(fid1, '%06f  ', poseGT(kk,jj));
% %         end
% %     end
% %     fprintf(fid1, '\n');
% % end
% % fclose(fid1);



B = fitplane(vslMat(:,10:12)');
B = B./norm(B(1:3));


Z = {};
Keys = {};
ct = 1;
chosenFrameList = 1 :30: size(vslMat,1)-1;
for g = 1 :length(chosenFrameList)-1
    posePrv = [reshape(vslMat(chosenFrameList(g),1:9),3,3) vslMat(chosenFrameList(g),10:12)';0 0 0 1];
    poseCur = [reshape(vslMat(chosenFrameList(g+1),1:9),3,3) vslMat(chosenFrameList(g+1),10:12)';0 0 0 1];
    Keys{1,ct} = [ct ct+1];
    Z{ct,1} = inv(poseCur \ posePrv);
    ct = ct+1;
end
jointPose = joint_optimization(Z, Keys, length(Z) + 1);

vMat = [];
for gg = 1 : length(jointPose)
    rt = ([jointPose{gg}(1:3,1:3) jointPose{gg}(1:3,4); 0 0 0 1]);
    vMat(gg,:) = [reshape(rt(1:3,1:3),1,9) rt(1:3,4)'];
end




if 1
    fid1 = fopen(fullfile(probPath, 'pose.txt'),'w');
    pose = [];
    for i = 1 : size(vslMat,1)
        rt_out = [reshape(vslMat(i, 1 : 9), 3,3) vslMat(i,10:12)']';
        pose(i,:) = [rt_out(:)'];
        for jj = 1 : size(pose,2)
            if 0%jj == 1
                fprintf(fid1, '%d ', frameList(i)-1);
            else
                fprintf(fid1, '%06f  ', pose(i,jj-0));
            end
        end
        fprintf(fid1, '\n');
    end
    fclose(fid1);

else
    
    fid1 = fopen(fullfile(probPath, 'pose.txt'),'w');
    pose = [];
    for i = 1 : size(vslMat, 1)
        rt_out = [reshape(vslMat(i, 1 : 9), 3,3) vslMat(i,10:12)']';
        pose(i,:) = [frameList(i) rt_out(:)'];
        for jj = 1 : size(pose,2)+1
            if jj == 1
                fprintf(fid1, '%d ', frameList(i)-1);
            else
                fprintf(fid1, '%06f  ', pose(i,jj-1));
            end
        end
        fprintf(fid1, '\n');
    end
    fclose(fid1);
end

gt = load(fullfile('D:\Temp\20200824', '081500-5000-6000-utm.txt'));
gt = gt - gt(1,:);
gt = [[5000:5999]' gt];
startFrame = 5170;
endFrame = 5311;
gt = gt(find(gt(:,1) == startFrame):find(gt(:,1) == endFrame),1:3);
gt(:,2:3) = gt(:,2:3) - gt(1,2:3);
gtt = gt(:,2:3);
gtt = [gtt(:,1) zeros(size(gtt,1),1) gtt(:,2)];
gttt = 1000.*gtt*roty(171)';
figure,plot(gttt(:,1),gttt(:,3));hold on;plot(vslMat(:,10),vslMat(:,12));axis equal
gtttt = [gt(:,1) gttt];
frame2check = [5180 : 10 :5290]';
gts0 = gtttt(ismember(gtttt(:,1), frame2check),:);


uiopen('D:\Auto\data4\20200323_081500_video\1\081500_gps_polygon.fig',1);
vsl2 = vslMat(ismember(gtttt(:,1), frame2check),10:12)./1000*roty(2.125)';
gts(:,2:4) = gts0(:,2:4)./1000*roty(2.125)';
% hold on;plot(gts(:,2), gts(:,4),'sg');
plot(vsl2(:,1), vsl2(:,3),'*g');





gt = load('D:\Auto\data4\20200323_081500_video\081500-0-3000-utm.txt');
gt = gt - gt(1,:);
gt = [[0:2999]' gt];
gtt = gt(:,2:3);
gtt = [gtt(:,1) zeros(size(gtt,1),1) gtt(:,2)];
gttt = 1000.*gtt([111:376 425 1845:2580],:)*roty(180)';gttt = gttt - gttt(1,:);figure,plot(gttt(:,1)./0.82,gttt(:,3)./0.82);hold on;plot(vslMat(:,10),vslMat(:,12));axis equal;legend('gps','vo');



gttt = 1000.*gtt([111:376 425 1845:2580],:)*roty(178)';gttt = gttt - gttt(1,:);figure,plot(gttt(:,1),gttt(:,3));hold on;plot(vslMat(:,10),vslMat(:,12));axis equal;legend('gps','vo');




gttt = 1000.*gtt([111:376 425 1845:2580],:)*roty(180)';gttt = gttt - gttt(1,:);figure,plot(gttt(:,1)./0.82,gttt(:,3)./0.82);hold on;plot(vslMat(:,10),vslMat(:,12));axis equal;legend('gps','vo');

figure,plot(gttt(:,1)./0.82,gttt(:,3)./0.82);hold on;plot(vslMat4(:,10),vslMat4(:,12));axis equal

figure(33);hold on;plot3(gttt(:,1)./0.82,gttt(:,2)./0.82, gttt(:,3)./0.82,'-g');


gt = load('D:\Auto\data7\081500\081500-5000-6000-utm.txt');
baseId = 170;
gt = gt - gt(baseId,:);
gt = [[2000:2999]' gt];
gtt = gt(:,2:3);
gtt = [gtt(:,1) zeros(size(gtt,1),1) gtt(:,2)];
gttt = 1000.*gtt*roty(171)';figure,plot(gttt(baseId:baseId+size(vslMat,1)-1,1),gttt(baseId:baseId+size(vslMat,1)-1,3),'-');axis equal;hold on;plot(vslMat(:,10),vslMat(:,12));legend('gps','vo');






gt = load('D:\Auto\data3\20200323_083500_video\083500-2000-3000-utm.txt');
gt = gt - gt(1,:);
gt = [[2000:2999]' gt];
gtt = gt(:,2:3);
gtt = [gtt(:,1) zeros(size(gtt,1),1) gtt(:,2)];
gttt = 1000.*gtt*roty(135.5)';

figure,plot(gttt(133:end,1),gttt(133:end,3),'-');axis equal;hold on;plot(vslMat(:,10),vslMat(:,12))
figure(13),plot3(gttt(133:end,1),gttt(133:end,2),gttt(133:end,3),'-b');
load('D:\Auto\data3\20200323_083500_video\20200827_065108\vslMat2.mat')
load('D:\Auto\data3\20200323_083500_video\20200827_065108\vslMat1.mat')
figure,plot(gttt(:,1), gttt(:,3),'-r');hold on;plot(vslMat1(:,10), vslMat1(:,12),'-g');plot(vslMat2(:,10), vslMat2(:,12),'-b');axis equal;legend('gps','direct method','point based method');


gt = load('D:\Auto\data3\20200323_083500_video\083500-2000-3000-utm.txt');
baseId = 1;
gt = gt - gt(baseId,:);
gt = [[2000:2999]' gt];
gtt = gt(:,2:3);
gtt = [gtt(:,1) zeros(size(gtt,1),1) gtt(:,2)];
gttt = 1000.*gtt*roty(135)';
figure,plot(gttt(baseId:baseId+size(vslMat,1)-1,1),gttt(baseId:baseId+size(vslMat,1)-1,3),'-');axis equal;hold on;plot(vslMat(:,10),vslMat(:,12));legend('gps','vo');
figure,plot(diff(gttt(baseId:end,3)));hold on;plot(diff(vslMat(:,12)));


figure,plot(diff(gttt(baseId:baseId+size(vslMat,1)-1,3)));hold on;plot(diff(vslMat(:,12)));


figure,hold on;
for p = 1 : 2: size(vslMat,1)
    plot(gttt(baseId:baseId+p-1,1),gttt(baseId:baseId+p-1,3),'-b');axis equal;hold on;plot(vslMat(1:p,10),vslMat(1:p,12),'-r');legend('gps','vo');
    drawnow;
end


diffTrans = diff(vslMat(:,10:12));
diffTransGT = diff(gttt);
[~, diffNorm] = NormalizeVector(diffTrans);
[~, diffNormGT] = NormalizeVector(diffTransGT);

figure(33);hold on;plot3(gttt(:,1)./0.82,gttt(:,2)./0.82, gttt(:,3)./0.82,'-g');


Pt3d = cell2mat(pt3d);
figure,pcshow(Pt3d);

figure,plot(frameList(MatchRatio(:,1)), MatchRatio(:,2));
optimize_graph;

% randList2 = randperm(size(Pt3d,1));
% figure,pcshow(Pt3d(randList2(1:900000),:));

for jj = 1:size(vslMat,1)
    vslMat_temp = inv([reshape(vslMat(jj, 1:9),3,3) vslMat(jj, 10:12)'; 0 0 0 1]);
%     vslMat_temp = [reshape(newMat(1:3,1:3), 1, 9) newMat(1:3, 4)'];
    vslMatGlobal(jj,:) = [reshape(vslMat_temp(1:3,1:3), 1, 9) vslMat_temp(1:3, 4)'];
    
end


traceManager = vsl.traceManager;
intrMat = Get(vsl.camModel, 'PinholeIntrMat', 1, vsl.scaleLvl);
save(fullfile(probPath,'traceManager.mat'), 'traceManager', 'poseGT','intrMat','-v7.3');

for j = 1 : size(poseGT, 1)
    T = [reshape(poseGT(j,1:9),3,3) poseGT(j,10:12)'; 0 0 0 1];

    if j == 1
        T0 = inv(T);
    end
    
    TT = T0 * T;
    
    poseMat(j,:) = [reshape(TT(1:3,1:3), 1, 9) TT(1:3, 4)'];

end


% if 0
%         CalibB2C(thetaList, vsl.traceInfoMat,vsl.traceManager.X,vsl.traceManager.Y, vsl.traceManager.XLK, vsl.traceManager.YLK)
% end
end
function [XYZ] = GetXYZFromDepth(intrMat, Pix,depthList)
metricPrevPtCcsGT = intrMat\HomoCoord(Pix',1);
metricPrevPtCcsGT = normc(metricPrevPtCcsGT);
if 1
    if 0
        if 0
            scaleAllGT = depthListGT(inlierId)./metricPrevPtCcsGT(3,:)';
        else
            scaleAllGT = depthListGT(:)./metricPrevPtCcsGT(3,:)';
        end
    else
        scaleAllGT = depthList./metricPrevPtCcsGT(3,:)';
    end
else
    dispGTComp = dispList(inlierId) - disparityErrorRound;
    depthListGTComp = intrMat(1,1).*norm(obj.camModel.transVec1To2)./(dispGTComp + (princpPtR(1) - princpPtL(1)));
    scaleAllGT = depthListGTComp./metricPrevPtCcsGT(3,:)';
end

XYZ = [repmat(scaleAllGT',3,1).*metricPrevPtCcsGT]';


end
function depthNew = RemoveDepthHoles(disparityMap)
global areaRatio
[L, LL] = bwlabel(disparityMap > 0, 8);

% areaRatio = 0.005; 0.01; 0.01;

for i = 1 : LL
    bwTemp =  L == (i);
    area(i) = sum(sum(bwTemp));
end


validLabel  = find(area > areaRatio*max(area));

validArea = false(size(disparityMap));
for i = 1 : length(validLabel)
    validTemp =  find(L == validLabel(i));
    validArea(validTemp) = true;
end
depthNew = nan(size(disparityMap));

depthNew(validArea) = disparityMap(validArea);
% validMap(~validArea) = false;
% disparityMap(~validArea) = 0;
end
function depthMap = ReplaceDepth(depthOrig, dispOrig, dispNew, img, intrMat,intrMat_s, Pix, baseline, cenX1, cenX2)
global xRng2 yRng2 zRng2



if size(dispNew,2) < size(dispOrig,2)
    dispNew = [dispNew dispOrig(:,end-(size(dispOrig,2) - size(dispNew,2))+1:end)];
end
if size(dispNew,1) < size(dispOrig, 1)
    dispNew = [dispNew; dispOrig(end-(size(dispOrig,1) - size(dispNew,1))+1:end,:)];
end

dispOrig0 = dispOrig;
dispNew0 = dispNew;


[XYZ_] = GetXYZFromDepth(intrMat, Pix, depthOrig(:));
xMat = reshape(XYZ_(:,1), size(depthOrig));
yMat = reshape(XYZ_(:,2), size(depthOrig));

depthOrig(depthOrig > zRng2(1) | depthOrig < zRng2(2)) = nan;
depthOrig(xMat > xRng2(1) | xMat < xRng2(2)) = nan;
depthOrig(yMat > yRng2(1) | yMat < yRng2(2)) = nan;
depthOrig(depthOrig < 0) = nan;
dispOrig(isnan(depthOrig)) = nan;
dispNew(isnan(depthOrig)) = nan;

a = dispNew(:)./dispOrig(:);
a = a(~isnan(a));
scale = median(a);
dispNewScaled = dispNew0./scale;
dispNewScaled(dispNewScaled < 0) = nan;
% dispNewScaled(dispNewScaled > max(dispOrig(:))) = nan;
dispNewScaled(isnan(dispOrig0)) = nan;


depthMap = intrMat_s(1) * baseline ./ (dispNewScaled + (cenX2(1) - cenX1(1)));
depthMap(isinf(depthMap)) = nan;

end