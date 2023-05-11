function rectifyImg(inputDir, outputDir, paraDir)
global cfg
paraDirInfo = dir(fullfile(paraDir, 'config.txt'));
cfg = readConfig(fullfile(paraDir, paraDirInfo(1).name));
MakeDirIfMissing(inputDir);
MakeDirIfMissing(outputDir);
try
    load(fullfile(paraDir, 'calib.mat'));
catch
    tmp1 = outputDir;
    tmp2 = paraDir;
    paraDir = tmp1;
    outputDir = tmp2;
    MakeDirIfMissing(inputDir);
    MakeDirIfMissing(outputDir);
    load(fullfile(paraDir, 'calib.mat'));
end


dirInfo = dir(fullfile(inputDir, '*.png'));
if length(dirInfo) == 0
    dirInfo = dir(fullfile(inputDir, '*.bmp'));
end
if length(dirInfo) == 0
    dirInfo = dir(fullfile(inputDir, '*.jpg'));
end
frameNum = length(dirInfo)/2;
for i = 1 : frameNum
    if ~cfg.switch_lr
        imgL = imread(fullfile(inputDir, dirInfo(i).name));
        imgR = imread(fullfile(inputDir, dirInfo(i + frameNum).name));
    else
        imgR = imread(fullfile(inputDir, dirInfo(i).name));
        imgL = imread(fullfile(inputDir, dirInfo(i + frameNum).name));
    end
   [rectImgL, rectImgR] = RectifyImagePair(stereoParam, imgL, imgR);
   
   if 0
       figure, imshowpair(rectImgL, rectImgR);
   end
   
   imwrite(rectImgL, fullfile(outputDir, sprintf('rectL_%05d.png',i)));
   imwrite(rectImgR, fullfile(outputDir, sprintf('rectR_%05d.png',i)));
    
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
        if strcmp(lineBuf(1:6), 'is_kep')
            cfg.is_kepler = str2double((lineBuf(12:end)));
        end
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
        % % % %         if strcmp(lineBuf(1:6), 'img_pr')
        % % % %             cfg.img_prefix = ((lineBuf(13:end)));
        % % % %         end
        % % % %         if strcmp(lineBuf(1:6), 'img_su')
        % % % %             cfg.img_suffix = ((lineBuf(13:end)));
        % % % %         end
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
        if strcmp(lineBuf(1:6), 'is_rot')
            cfg.isRotate = str2double((lineBuf(12:end)));
        end
        if strcmp(lineBuf(1:6), 'check_')
            cfg.check_depth = str2double((lineBuf(14:end)));
        end
        if strcmp(lineBuf(1:6), 'fxy_ra')
            cfg.fxy_ratio = str2double((lineBuf(12:end)));
        end
        if strcmp(lineBuf(1:6), 'param_')
            cfg.paramDir = ((lineBuf(12:end)));
        end
        if strcmp(lineBuf(1:6), 'ext_fo')
            cfg.ext_focal = str2double((lineBuf(10:end)));
        end
        if strcmp(lineBuf(1:6), 'cvt_ca')
            cfg.cvt_calib = str2double((lineBuf(12:end)));
        end
        if strcmp(lineBuf(1:6), 'cvt_di')
            cfg.cvt_calib_dir = ((lineBuf(10:end)));
        end
    end
    
    
end
% G=1;
% g=gVec;
fclose(configFileFid);


end