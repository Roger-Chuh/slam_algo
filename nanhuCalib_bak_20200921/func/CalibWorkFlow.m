function [paraDir, cropDir, calibFrameNum, good, stereoParam,config,cbImgLstLLL2check,cbImgLstRRR2check] = CalibWorkFlow(cropPrefix,paraPrefix,calibFuncDir, DX,imgNumUsed,times,N, leastNum,cbImgListLLL, cbImgListRRR, cbcXYLLL, cbcXYRRR,modeE,imgNum,baselineDiff)



    
MakeDirIfMissing((cropPrefix));
MakeDirIfMissing((paraPrefix));
cropDir = (cropPrefix);
paraDir = (paraPrefix);
    


STEREOPARAM = {};
RECTIFYPARAM = {};

config.dX = DX; config.dY = DX;

NNNOrig = imgNum;

goodId = [];


for vrkw = 1:length(cbcXYLLL)
    if ~isempty(cbcXYLLL{vrkw})
        goodId = [goodId; vrkw];
    end
end


cbImgListLLL = cbImgListLLL(goodId);
cbImgListRRR = cbImgListRRR(goodId);

cbcXYLLL = cbcXYLLL(goodId);
cbcXYRRR = cbcXYRRR(goodId);
if length(cbImgListLLL) < leastNum
    
    good = 0;
    stereoParam = [];config = [];calib_dataLLL = []; calib_dataRRR = [];
    cbImgLstLLL2check = []; cbImgLstRRR2check = []; calibFrameNum = [];
    fprintf(strcat('\n\n======= too few valid images to calibrate, need to reCapture =======\n\n'));

    return;
end

if length(cbImgListLLL) < NNNOrig
    fprintf(strcat('\n\n======= some images failed, but still enough to proceed=======\n\n'));
else
    fprintf(strcat('\n\n======= all images are good =======\n\n'));
    
end


id = randperm(length(cbImgListLLL))';
iddx = id(1:imgNumUsed);
iddx_left = setdiff([1:length(cbImgListLLL)]',iddx);

% cbImgListLL = cbImgListLLL(1:imgNumUsed); cbImgListRR = cbImgListRRR(1:imgNumUsed);
% cbcXYLL = cbcXYLLL(1:imgNumUsed); cbcXYRR = cbcXYRRR(1:imgNumUsed);
cbImgListLL = cbImgListLLL(iddx); cbImgListRR = cbImgListRRR(iddx);
cbcXYLL = cbcXYLLL(iddx); cbcXYRR = cbcXYRRR(iddx);

calibFrameNum = length(cbImgListLL);

% % % cbImgLstLLL2check = cbImgListLLL(imgNumUsed+1: end);
% % % cbImgLstRRR2check = cbImgListRRR(imgNumUsed+1: end);
cbImgLstLLL2check = cbImgListLLL(iddx_left);
cbImgLstRRR2check = cbImgListRRR(iddx_left);


good = 222;

STEREOPARAM = cell(times,1);
imgIdx = cell(times,2);
for i = 1 : times
    tic;
    fprintf('\n\n\n');
    fprintf(strcat('======= Loop ======',num2str(i),' ========'));
    fprintf('\n\n\n');

    idx = randperm(length(cbImgListLL));
    idxx = idx(1 : N);

    cbImgListL = cbImgListLL(idxx); cbImgListR = cbImgListRR(idxx);
    cbcXYL = cbcXYLL(idxx); cbcXYR = cbcXYRR(idxx);
    
    if 0
        
       for lk = 1 : length(cbcXYL)
           ptL = cbcXYL{lk}; ptR = cbcXYR{lk};
           figure(lk),subplot(1,2,1);imshow(imread(cbImgListL{lk}));hold on;plot(ptL(1,:),ptL(2,:),'+g');title('left');
                   plot(ptL(1, 1), ptL(2,1), 'o', 'color',[1.000 0.314 0.510],'linewidth',2);
        plot(ptL(1,1:14), ptL(2, 1:14),'-b','linewidth',2);
        plot(ptL(1, 14), ptL(2,14),'oy','linewidth',2);
                      subplot(1,2,2);imshow(imread(cbImgListR{lk}));hold on;plot(ptR(1,:),ptR(2,:),'+g');title('right');
                      plot(ptR(1, 1), ptR(2,1), 'o', 'color',[1.000 0.314 0.510],'linewidth',2);
        plot(ptR(1,1:14), ptR(2, 1:14),'-b','linewidth',2);
        plot(ptR(1, 14), ptR(2,14),'oy','linewidth',2);
       end       
        
    end
    
    if 1
        [camParamL, cbcXYL, cbGrid, config, ~] = CbCalibSingle4(cbImgListL,cbcXYL,calibFuncDir,modeE, config);
        [camParamR, cbcXYR, cbGrid, config, ~] = CbCalibSingle4(cbImgListR,cbcXYR,calibFuncDir,modeE, config);

        
    else
        [camParamL, cbcXYL, cbGrid, config, ~] = CbCalibSingle2(cbImgListL,calibFuncDir, config);
        [camParamR, cbcXYR, cbGrid, config, ~] = CbCalibSingle2(cbImgListR,calibFuncDir, config);
    end
    stereoParamm = CbCalibStereo(camParamL, camParamR, cbcXYL, cbcXYR, cbGrid, config, config);
    

    imgIdx{i,1} = cbImgListL;imgIdx{i,2} = cbImgListR';
    STEREOPARAM{i,1} = stereoParamm;
toc;
end

FocLeft = zeros(2,times);
CenLeft = zeros(2,times);
AlphaLeft = zeros(1,times);
KcLeft = zeros(5,times);
FocRight = zeros(2,times);
CenRight = zeros(2,times);
AlphaRight = zeros(1,times);
KcRight = zeros(5,times);
RotVecRef = zeros(3,times);
TransVecRef = zeros(3,times);
FocLeftError = zeros(2,times);
CenLeftError = zeros(2,times);
AlphaLeftError = zeros(1,times);
KcLeftError = zeros(5,times);
FocRightError = zeros(2,times);
CenRightError = zeros(2,times);
AlphaRightError = zeros(1,times);
KcRightError = zeros(5,times);
RotVecRefError = zeros(3,times);
TransVecRefError = zeros(3,times);

for ii = 1 : times
    temp = STEREOPARAM{ii};
    FocLeft(:,ii) = temp.focLeft;
    CenLeft(:,ii) = temp.cenLeft;
    AlphaLeft(:,ii) = temp.alphaLeft;
    KcLeft(:,ii) = temp.kcLeft;
    FocRight(:,ii) = temp.focRight;
    CenRight(:,ii) = temp.cenRight;
    AlphaRight(:,ii) = temp.alphaRight;
    KcRight(:,ii) = temp.kcRight;
    RotVecRef(:,ii) = temp.rotVecRef;
    TransVecRef(:,ii) = temp.transVecRef;
    FocLeftError(:,ii) = temp.focLeftError;
    CenLeftError(:,ii) = temp.cenLeftError;
    AlphaLeftError(:,ii) = temp.alphaLeftError;
    KcLeftError(:,ii) = temp.kcLeftError;
    FocRightError(:,ii) = temp.focRightError;
    CenRightError(:,ii) = temp.cenRightError;
    AlphaRightError(:,ii) = temp.alphaRightError;
    KcRightError(:,ii) = temp.kcRightError;
    RotVecRefError(:,ii) = temp.rotVecRefError;
    TransVecRefError(:,ii) = temp.transVecRefError;

end

    stereoParam.focLeft = mean(FocLeft,2);
    stereoParam.cenLeft = mean(CenLeft,2);
    stereoParam.alphaLeft = mean(AlphaLeft,2);
    stereoParam.kcLeft = mean(KcLeft,2);
    stereoParam.focRight = mean(FocRight,2);
    stereoParam.cenRight = mean(CenRight,2);
    stereoParam.alphaRight = mean(AlphaRight,2);
    stereoParam.kcRight = mean(KcRight,2);
    stereoParam.rotVecRef = mean(RotVecRef,2);
    stereoParam.transVecRef = mean(TransVecRef,2);
    stereoParam.focLeftError = mean(FocLeftError,2);
    stereoParam.cenLeftError = mean(CenLeftError,2);
    stereoParam.alphaLeftError = mean(AlphaLeftError,2);
    stereoParam.kcLeftError = mean(KcLeftError,2);
    stereoParam.focRightError = mean(FocRightError,2);
    stereoParam.cenRightError = mean(CenRightError,2);
    stereoParam.alphaRightError = mean(AlphaRightError,2);
    stereoParam.kcRightError = mean(KcRightError,2);
    stereoParam.rotVecRefError = mean(RotVecRefError,2);
    stereoParam.transVecRefError = mean(TransVecRefError,2);
    
    [rectParamL, rectParamR] = GetRectifyParam(stereoParam, size(rgb2gray(imread(cbImgListL{1,1}))));
    baseline = norm(stereoParam.transVecRef);
    blDiff = stereoParam.transVecRef - [-65;0;0];
     fprintf('\n\n');
        fprintf(sprintf('========== Baseline is %f, %f, %f ===========',stereoParam.transVecRef(1),stereoParam.transVecRef(2),stereoParam.transVecRef(3)));
        fprintf('\n\n\n');
% %     if norm(blDiff) >= baselineDiff
% %         fprintf('\n\n');
% %         fprintf(sprintf('========== Estimated baseline differs too much from golden, error being %f mm ==========', norm(blDiff)));
% %         fprintf('\n\n\n');
% %         good = 0;
% %         stereoParam = [];config = [];calib_dataLLL = []; calib_dataRRR = [];
% %         cbImgLstLLL2check = []; cbImgLstRRR2check = []; calibFrameNum = [];
% %         return;
% % 
% %         
% %     end
    
    stereoParam_File = strcat(paraDir,'\stereoParam.mat');
    stereo_param_File = strcat(paraDir,'\stereo_param.mat');
    rectify_param_File = strcat(paraDir,'\rectify_param.mat');
    calib_File = strcat(paraDir,'\calib.mat');
    
save(stereoParam_File,'stereoParam');
save(stereo_param_File,'stereoParam');
save(rectify_param_File,'baseline','rectParamL','rectParamR');
save(calib_File,'baseline', 'camParamL', 'camParamR', 'cbcXYL', 'cbcXYR', 'cbGrid', 'cbImgListL', 'cbImgListLL', 'cbImgListLLL', 'cbImgListR', 'cbImgListRR', 'cbImgListRRR', 'cbImgLstLLL2check', 'cbImgLstRRR2check', 'config', 'DX', 'goodId', 'imgIdx', 'rectify_param_File', 'RECTIFYPARAM', 'rectParamL', 'rectParamR', 'stereo_param_File', 'stereoParam', 'STEREOPARAM', 'stereoParam_File', 'calib_File', 'stereoParamm', 'temp');
 
    IMU = [1 1 1 1 1 1];
    
    
    focL = stereoParam.focLeft; cenL = stereoParam.cenLeft; alphaL = stereoParam.alphaLeft; kcL = stereoParam.kcLeft;
    focR = stereoParam.focRight; cenR = stereoParam.cenRight; alphaR = stereoParam.alphaRight; kcR = stereoParam.kcRight;
    rotMat = rodrigues(stereoParam.rotVecRef)'; transVec = stereoParam.transVecRef;
    ImuData = IMU';

    calibRomFid = fopen(strcat(paraDir,'\calib.rom'),'w');
    fwrite(calibRomFid, [focL;cenL;alphaL;kcL;focR;cenR;alphaR;kcR;rotMat(:);transVec;ImuData],'single'); fclose(calibRomFid);

% check rectify results
if 0
    load(strcat(paraDir,'\stereoParam.mat'));
    imgL = imread(strcat(cropDir,'\cap_frame__left29.bmp'));
    imgR = imread(strcat(cropDir,'\cap_frame__right29.bmp'));
    [rectImgL, rectImgR] = RectifyImagePair(stereoParam, imgL, imgR);
end

if norm(blDiff) >= baselineDiff
        fprintf('\n\n');
        fprintf(sprintf('========== Estimated baseline differs too much from golden, error being %f mm ==========', norm(blDiff)));
        fprintf('\n\n\n');
        good = 0;
        stereoParam = [];config = [];calib_dataLLL = []; calib_dataRRR = [];
        cbImgLstLLL2check = []; cbImgLstRRR2check = []; calibFrameNum = [];
        return;

        
end

end
    
    
    
    

