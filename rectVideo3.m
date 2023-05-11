function rectVideo3(inputDir)
global OUTPUT_ROOT RECORDED_BIN_ROOT DUMP_ROOT titleStr thetaListGT ShiftByCoord probPath doubleShift mean_x_err rou...
    shiftPixThr doubleShift2 err_x switchMethod rou_temp_list Rou UseDoubleShift...
    UseEx


if 0
    img1 = (imread('C:\Users\rongjiezhu\Pictures\Camera Roll\zedRect\office015\image_0\000000.png'));
    img2 = (imread('C:\Users\rongjiezhu\Pictures\Camera Roll\zedRect\office015\image_1\000000.png'));
    img11 = img1(55:620,62:1238);
    img22 = img2(55:620,62:1238);
    disparityMap = disparity(img11, img22, 'DisparityRange',[0 144]);
    validMap = disparityMap>0&disparityMap<144;
    figure,imshow(immultiply(disparityMap, validMap),[])
end





ShiftByCoord = false; true;
doubleShift = false; true; false; true; false;true;
mean_x_err = [0 0];
rou = 10; 7; 10; 7; 10; 7; 10;7; 10; 7; 10;7; 8; 14;9; 14; 9; 7; 12; 10; 7; 10; 7; 6; 10; 7; 10; 7; 5; 8; 9; 10.1;
Rou = 0;  0.5; 0; 1; 0; 1; 1.5;

shiftPixThr = inf; -2; 3; 2;
doubleShift2 = true;
err_x = [0 0];
switchMethod = true;
rou_temp_list = [];
UseDoubleShift = true; false; true; false; true;false; true; false; true;
UseEx = false; true; false; true; false; true;

if 0
    load('D:\Temp\zed_calib\result\stereo_param.mat');
    imgSize = [1080 1920]';
else
    load('D:\Temp\zed_calib\720\result\stereo_param.mat');
    imgSize = [720 1280]';
end


assert(~isempty(OUTPUT_ROOT) && ~isempty(RECORDED_BIN_ROOT) && ~isempty(DUMP_ROOT), 'The directories are not correctly set');
csaParamFilePath = fullfile(DUMP_ROOT, 'coordsys_alignment.txt');
csa = CoordSysAligner;
% % Load(csa, csaParamFilePath);
robotCfgFilePath = fullfile(OUTPUT_ROOT, 'robot_config.prototxt');
robotConfig = ReadRobotConfig(robotCfgFilePath);

camParamFilePath = fullfile(DUMP_ROOT, 'camera_param.txt');
cam = BCStereoCameraModel(camParamFilePath);
CvtToRectifiedModel(cam);
vsl = VisualLocalizer(cam, csa, robotConfig.tracking_config);
if isempty(vsl.scaleLvl)
    vsl.scaleLvl = 0;
end
try
    load(fullfile(inputDir,'angGT.mat'));
catch
    sdjfh = 1;
end

rVec = stereoParam.rotVecRef; % [ 1.52973223e+00, 7.39023369e-03, -7.28091807e-04 ]';
tVec = stereoParam.transVecRef; % 10.*[ -6.53009415e-01, 1.49285034e+02, -6.13110685e+00 ]';

R = [ 9.9999067662137064e-01, -4.2691725225514534e-03,6.4871897307110975e-04;...
    4.2691085282925076e-03, 9.9999088231463540e-01, 9.9999853603564991e-05;...
    -6.4913997488288771e-04, -9.7229469566657761e-05, 9.9999978458183847e-01 ];
T = 10.*[ -1.1996676211530170e+01, -5.1620839537964008e-02, -9.2826799911339235e-03 ]';
KL = [ 1.9748364077614247e+03, 0, 9.5465987383796642e+02;...
    0  1.9757374771183775e+03, 5.2313946471934150e+02;...
    0., 0., 1. ];
KR = [ 1.9628183589635823e+03, 0., 9.8119298543983234e+02;...
    0. 1.9638148934135372e+03, 5.6751332830636375e+02;...
    0., 0., 1. ];
kcL = [ -4.7126784623298640e-01, 1.5707134103342354e-01,  8.8282659945944375e-04, 1.5838321454318845e-04, 5.0270413373945999e-01 ]';
kcR = [ -4.6676181774088582e-01, 1.6765245155018399e-01, 2.2689439615351582e-03, 9.9312512176874166e-04,   4.0665398542188858e-01 ]';



R = rodrigues(stereoParam.rotVecRef);
T =  stereoParam.transVecRef;
KL = [stereoParam.focLeft(1) 0 stereoParam.cenLeft(1); 0 stereoParam.focLeft(2) stereoParam.cenLeft(2); 0 0 1];
KR = [stereoParam.focRight(1) 0 stereoParam.cenRight(1); 0 stereoParam.focRight(2) stereoParam.cenRight(2); 0 0 1];
kcL = stereoParam.kcLeft;
kcR = stereoParam.kcRight;
[rotMat1ToCP, rotMat2ToCP, projMat1, projMat2] = OcvStereoRectify(KL, kcL, KR, kcR, imgSize, rodrigues(R), T);


vsl.camModel.monoCamModel1.focLen = [KL(1,1); KL(2,2)];
vsl.camModel.monoCamModel1.princpPt = [KL(1,3); KL(2,3)];
vsl.camModel.monoCamModel1.distCoeff = kcL;
vsl.camModel.monoCamModel2.focLen = [KR(1,1); KR(2,2)];
vsl.camModel.monoCamModel2.princpPt = [KR(1,3); KR(2,3)];
vsl.camModel.monoCamModel2.distCoeff = kcR;
SetPinholeCamParam(vsl.camModel.monoCamModel2, [projMat2(1,1); projMat2(2,2)], projMat2(1:2,3), projMat2(1,2));
SetPinholeCamParam(vsl.camModel.monoCamModel1, [projMat1(1,1); projMat1(2,2)], projMat1(1:2,3), projMat1(1,2));
vsl.camModel.imgSize = imgSize;
[mapX1, mapY1] = OcvInitUndistortRectifyMap(Get(vsl.camModel, 'IntrMat', 1), Get(vsl.camModel, 'DistCoeff', 1), rotMat1ToCP, Get(vsl.camModel, 'PinholeIntrMat', 1), vsl.camModel.imgSize);
[mapX2, mapY2] = OcvInitUndistortRectifyMap(Get(vsl.camModel, 'IntrMat', 2), Get(vsl.camModel, 'DistCoeff', 2), rotMat2ToCP, Get(vsl.camModel, 'PinholeIntrMat', 2), vsl.camModel.imgSize);
SetMap(vsl.camModel.monoCamModel1, mapX1, mapY1);
SetMap(vsl.camModel.monoCamModel2, mapX2, mapY2);
vsl.camModel.rotVec1To2 = rodrigues(R);
vsl.camModel.transVec1To2 = T;

intrMat = Get(vsl.camModel, 'PinholeIntrMat', 1,vsl.scaleLvl);
[princpPtL, princpPtR] = Get(vsl.camModel, 'PinholePrincpPt', vsl.scaleLvl);


dirInfo = dir(fullfile(inputDir, '*.mp4'));


% obj= VideoReader(fullfile(inputDir, dirInfo(1).name));

poseVec = [reshape(eye(3), 1, 9) 0 0 0];





% % dirInfo = dir(fullfile(inputDir, '*.jpg'));
% % 
% % [~, ind1] = sort([dirInfo(1:length(dirInfo)/2).datenum], 'ascend');
% % [~, ind2] = sort([dirInfo(length(dirInfo)/2+1:end).datenum], 'ascend');
% % ind2 = ind2 + length(dirInfo)/2;
% % I = [ind1 ind2];
% % 
% % 
% % tempInfo = cell(length(I),1);
% % for fg = 1:length(I)
% %     tempInfo{fg,1} = dirInfo(I(fg)).name;
% % end
% % for fgg = 1:length(I)
% %     dirInfo(fgg).name = tempInfo{fgg};
% % end
% % 
% % 
% % frameNum = length(dirInfo)/2;
% % 
% % startt = 1; 3171; 5170;  2690; 1; 2133;
% % endd = length(dirInfo)/2; 3313; 5300;  3001;frameNum; 2526;

cnt = 0;

obj= VideoReader(fullfile(inputDir, dirInfo(1).name));

kittiK = intrMat';
kittit = [-norm(T)./1000 0 0];

kittiP1 = kittit * kittiK;
kittiP1 = [kittiK; kittiP1];


kittiP0 = kittiP1;
kittiP0(end,:) = 0;

kittiP = [kittiP0(:) kittiP1(:)];

fid1 = fopen(fullfile(inputDir,'calib.txt'),'w');%????

for j = 1 : size(kittiP, 2)

fprintf(fid1,sprintf('P%d: %0.9f %0.9f %0.9f %0.9f %0.9f %0.9f %0.9f %0.9f %0.9f %0.9f %0.9f %0.9f\n',j-1, kittiP(:,j)));
            
end       
fclose(fid1);

MakeDirIfMissing(fullfile(inputDir,'image_0'));
MakeDirIfMissing(fullfile(inputDir,'image_1'));
poseVec = [reshape(eye(3), 1, 9) 0 0 0];

cnt = 0;
ct = 0; 5968; 0; 40000; 34000; 0; 25885; 20000;  0; 4000; 0; 1700; 0;

skipFrameNum = 1; 3; 1; 3;

while hasFrame(obj)
    I = readFrame(obj);
    timeStamp = round(obj.CurrentTime,3);
    if (timeStamp >= 1)  % && timeStamp < 89.7) || (timeStamp >= 89.9)
        
       
        if mod(cnt,skipFrameNum) == 0
            imgL = imresize(I(:,1:size(I,2)/2,:), imgSize');
            imgR = imresize(I(:,size(I,2)/2 + 1 : end,:), imgSize');
            imgLL = Remap(vsl.camModel.monoCamModel1, imgL);
            imgRR = Remap(vsl.camModel.monoCamModel2, imgR);
            if 0 
                imwrite(rgb2gray(imgLL), fullfile(inputDir,'image_0', sprintf('marker_%d.png',ct)));
            else
                imwrite(rgb2gray(imgLL), fullfile(inputDir,'image_0', sprintf('%06d.png',ct)));
                imwrite(rgb2gray(imgRR), fullfile(inputDir, 'image_1',sprintf('%06d.png',ct)));
            end
           
            ct = ct + 1;
            
            %             depth1 = GetDepthMap(vsl, imgLL, imgRR);
            %
            %             data = {};
            %
            %             %             depth1 = GetDepthMap(vsl, imgLL, imgRR);
            %
            %             data{1, 1} = imgLL; data{1, 2} = imgRR;
            %
            %             data{1, 3} = depth1;
            %             data{1, 5} = depth1;
            %             data{1, 7} = poseVec;
            %             %             save(fullfile(inputDir,sprintf('ReplayData_%05d.mat',length(dir(fullfile(inputDir,'ReplayData_*.mat')))+1)), 'data');
            %             save(fullfile(inputDir,sprintf('ReplayData_%015d.mat',1000*timeStamp)), 'data');
            
        end
        
        cnt = cnt + 1;
        
    end
    %     imgLL = Remap(vsl.camModel.monoCamModel1, imgL);
    %     imgRR = Remap(vsl.camModel.monoCamModel2, imgR);
    
end

figure,imshowpair(imgLL, imgRR);


end