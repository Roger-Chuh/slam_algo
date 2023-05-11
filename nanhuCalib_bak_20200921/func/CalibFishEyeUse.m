function [camParam, cbcXYNew, cbGrid, config, undistImg, cbcXYTmp, cbGridTmp, goodId, cbcXY, origImg] = CalibFishEyeUse(inputDir)
global cfg
workDir = pwd;
draw = cfg.show_fig;
% len = 70;  23.7; 70;23.7; 
calibConfig.inputDir = inputDir;
calibConfig.baseName = cfg.img_prefix; % 'imgR_';'Image__2019-11-01__';'imgR_';'TP-IPC22-4 1_102_20190214160400_';
calibConfig.suffix = cfg.img_suffix; % 'png';'bmp';'png'; 'jpg';
calibConfig.dX = cfg.cb_size; % 18.4
calibConfig.dY = cfg.cb_size; % 18.4
calibConfig.Col = cfg.cb_col; % 14;
calibConfig.Row = cfg.cb_row; % 9;
calibConfig.func = cfg.calibFuncDir; % 'D:\bk_20180627_\nextvpu\algorithm\src\matlibs\computer_vision\camera_calibration\Scaramuzza_OCamCalib_v3.0_win';

set_up_global;
data_calib(calib_data, calibConfig);


click_calib(calib_data,calibConfig)
imgList = getImgList(calibConfig.inputDir);
config.dX = calibConfig.dX;
config.dY = calibConfig.dY;
calibFuncDir = calibConfig.func;
modeE = 2;
if ~exist(fullfile(inputDir,'calib.mat'))
% %     [camParam, cbcXY, cbGrid, config, cb_data, cbcXYTmp, cbGridTmp, goodId] = CbCalibSingle4(imgList, [], calibFuncDir, modeE, config);
    [camParam, cbcXY, cbGrid, config, cb_data, cbcXYTmp, cbGridTmp, goodId] = CbCalibSingleUse4(imgList, [], calibFuncDir, modeE, config);
    save(fullfile(inputDir,'calib.mat'),'camParam', 'cbcXY', 'cbGrid', 'config', 'cbcXYTmp', 'cbGridTmp', 'goodId');
else
    cb_data = [];
    load(fullfile(inputDir,'calib.mat'))
end
if ~exist(fullfile(inputDir,'calib_data.mat'))
    calib_data.L = calib_data.L(goodId);
    calib_data.I = calib_data.I(goodId);
    calib_data.n_ima = length(goodId);
    calib_data.ind_active = 1:length(goodId);
    calib_data.ind_read = 1:length(goodId);
    calib_data.active_images = ones(1,length(goodId));
    % % calib_data.RRfin = calib_data.RRfin(:,:,goodId);
    calib_data.ima_proc = 1:length(goodId);
    calib_data = cbCalib2OcamCalib(calib_data, inputDir);
    
    calib_data.Xp_abs = calib_data.Xp_abs(:,:,1:length(goodId));
    calib_data.Yp_abs = calib_data.Yp_abs(:,:,1:length(goodId));
        
    camParamBak = camParam;
    close all
    %1 : length(calib_data.L);
    % % calib_data = cbCalib2OcamCalib(calib_data, 'D:\Temp\20180724\calibFishEye1');
    % calib_data.ind_active = goodId';
    calib_data = calibration(calib_data);
    if 0
        calib_data = findcenter(calib_data);
        calib_data = optimizefunction(calib_data);
    else
        calib_data = bundleAdjustmentUrban(calib_data, 1);
    end
    save(fullfile(inputDir,'calib_data.mat'),'calib_data');
else
    load(fullfile(inputDir,'calib_data.mat'),'calib_data');
end



analyse_error(calib_data)
create_simulation_points(calib_data);

[calib_data.ocam_model.pol, err, N] = findinvpoly(calib_data.ocam_model.ss, calib_data.ocam_model.height);
oCamModel = calib_data.ocam_model;
if strcmp(inputDir(end-3:end),'Left')
    oCamModelL = oCamModel;
    save(fullfile(inputDir,'oCamModelL.mat'),'oCamModelL');
else
    oCamModelR = oCamModel;
    save(fullfile(inputDir,'oCamModelR.mat'),'oCamModelR');
end


U_same = ocam_undistort_map(oCamModel,'OutputView','same');
U_full = ocam_undistort_map(oCamModel,'OutputView','full');
intrMat_same = U_same.K';
intrMat_full = U_full.K';
% nim_same = ocam_undistort(img,U_same);
% nim_full = ocam_undistort(img,U_full);
M = [calib_data.Xt calib_data.Yt zeros(size(calib_data.Xt,1),1)];
cd (workDir);
[nr,nc,~] = size(calib_data.I{1});
for i = 1 : calib_data.n_ima
    
    img = calib_data.I{i};
    Img(:,:,1) = img;
    Img(:,:,2) = img;
    Img(:,:,3) = img;
    Img = uint8(Img);
    nim_same = ocam_undistort(Img,U_same);
    undistImg{i,1} = nim_same;
    origImg{i,1} = Img;
    
    R=calib_data.RRfin(:,:,i);
    R(:,3)=cross(R(:,1),R(:,2));
    r=rodrigues(R);
    t=calib_data.RRfin(:,3,i);
    
    RR = R;
    RR(1,:) = -R(2,:);
    RR(2,:) = -R(1,:);
    tt = t;
    tt(1) = -t(2);
    tt(2) = -t(1);
    
    
    
    T = ([-RR -tt;0 0 0 1]);
    Mc = T(1:3,1:3)*M' + repmat(T(1:3,4),1,size(M,1));
    pixProj = pflat(intrMat_same*Mc);
    [ptUndist, ptUndist3D] = remapFishEyePix([calib_data.Yp_abs(:,:,i) calib_data.Xp_abs(:,:,i)]', intrMat_same, oCamModel);
    pixDistorted = remapFishEyePixInv(pixProj(1:2,:), intrMat_same, oCamModel);
    
    if draw
        figure,plot(ptUndist'-pixProj(1:2,:)');
        figure,plot(pixDistorted-[calib_data.Yp_abs(:,:,i) calib_data.Xp_abs(:,:,i)]);
        
        figure,subplot(1,2,1);imshow(undistImg{i,1});hold on;plot(ptUndist(1,:),ptUndist(2,:),'or');plot(pixProj(1,:),pixProj(2,:),'.g')
        subplot(1,2,2);imshow(calib_data.I{i},[]);hold on;plot(pixDistorted(:,1),pixDistorted(:,2),'or');plot(calib_data.Yp_abs(:,:,i),calib_data.Xp_abs(:,:,i),'.g')
    end
    cbcXYNew{i,1} = ptUndist(1:2,:);
    camParam.rotVec(:,i) = rodrigues(T(1:3,1:3));
    camParam.tranVec(:,i) = T(1:3,4);
end
camParam.kc = zeros(5,1);
camParam.foc = [intrMat_same(1,1);intrMat_same(2,2)];
camParam.cen = [intrMat_same(1,3);intrMat_same(2,3)];
end