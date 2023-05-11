function CalibrationTool_(inputDir, funcDir, paraDir)
global cfg stepDir inputDir0
inputDir0 = inputDir;
paraDirInfo = dir(fullfile(paraDir, 'config.txt'));
cfg = readConfig(fullfile(paraDir, paraDirInfo(1).name));
cfg.calibFuncDir = funcDir;
stepDir = paraDir;
if cfg.wide_angle
    [origImgL1, origImgR1, undistImgL1, undistImgR1, rectImgL1, rectImgR1] = CalibStereoFishEyeUse(inputDir, []);
    load(fullfile(inputDir, 'calib.mat'));
    if ~cfg.calib_stereo
        stereoParam = camParamL;
    end
%     [rectParamL, rectParamR,rectImgL,rectImgR] = GetRectifyParam_table_use(stereoParam, size(origImgL1),origImgL1,origImgR1,inputDir);
    GetRectifyParam_table_use(stereoParam, size(origImgL1),origImgL1,origImgR1,inputDir);
    
    
else
    [imgL0, imgR0, rectImgL0, rectImgR0] = tempCalibFlowUse(inputDir, num2str(cfg.img_scale), num2str(cfg.is_yuv), num2str(cfg.img_width), num2str(cfg.img_height), cfg.calibFuncDir, num2str(cfg.cb_size), num2str(cfg.switch_lr));
    load(fullfile(inputDir, 'calib.mat'));
    %     [rectParamL, rectParamR,rectImgL,rectImgR] = GetRectifyParam_table_use(stereoParam, size(imgL0),imgL0,imgR0, []);
    if ~cfg.calib_stereo
        try
            stereoParam = camParamL;
        catch
            gkag = 1;
        end
    end
    if cfg.switch_lr
        GetRectifyParam_table_use(stereoParam, size(imgL0),imgR0,imgL0, []);
    else
        GetRectifyParam_table_use(stereoParam, size(imgL0),imgL0,imgR0, []);
    end
end

end

function cfg = readConfig(accFilePath)

[configFileFid, errMsg] = fopen(accFilePath);
if (configFileFid < 0)
    error('Cannot open %s for read: %s', accFilePath, errMsg);
end
% gVecBuf = [];
% inReadgVec = false;
% flagg=0;


while ~feof(configFileFid)
    lineBuf = strtrim(fgetl(configFileFid));
    if 1 %length(lineBuf) > 10
        if strcmp(lineBuf(1:6), 'is_yuv')
            cfg.is_yuv = str2double((lineBuf(9:end)));
        end
        if strcmp(lineBuf(1:6), 'calib_')
            cfg.calib_stereo = str2double((lineBuf(15:end)));
        end
        if strcmp(lineBuf(1:6), 'wide_a')            
            cfg.wide_angle = str2double((lineBuf(13:end)));
        end
        if strcmp(lineBuf(1:6), 'cb_row')            
            cfg.cb_row = str2double((lineBuf(9:end)));
        end
        if strcmp(lineBuf(1:6), 'cb_col')            
            cfg.cb_col = str2double((lineBuf(9:end)));
        end
        if strcmp(lineBuf(1:6), 'cb_siz')            
            cfg.cb_size = str2double((lineBuf(10:end)));
        end
        if strcmp(lineBuf(1:6), 'show_f')            
            cfg.show_fig = str2double((lineBuf(11:end)));
        end
        if strcmp(lineBuf(1:6), 'img_pr')            
            cfg.img_prefix = ((lineBuf(13:end)));
        end
        if strcmp(lineBuf(1:6), 'img_su')            
            cfg.img_suffix = ((lineBuf(13:end)));
        end
        if strcmp(lineBuf(1:6), 'img_su')            
            cfg.img_suffix = ((lineBuf(13:end)));
        end
        if strcmp(lineBuf(1:6), 'img_wi')            
            cfg.img_width = str2double((lineBuf(12:end)));
        end
        if strcmp(lineBuf(1:6), 'img_he')            
            cfg.img_height = str2double((lineBuf(13:end)));
        end
        if strcmp(lineBuf(1:6), 'img_sc')            
            cfg.img_scale = str2double((lineBuf(12:end)));
        end
        if strcmp(lineBuf(1:6), 'switch')            
            cfg.switch_lr = str2double((lineBuf(12:end)));
        end
        if strcmp(lineBuf(1:6), 'est_k3')            
            cfg.est_k3 = str2double((lineBuf(9:end)));
        end
    end
   
    
end
% G=1;
% g=gVec;
fclose(configFileFid);


end