function [leftImgLst, rightImgLst, cbcXYL, cbcXYR] = CaptureAndDetect2(inputDir, calibFuncDir,outputDir,DXX,modeE,blbl)
% % % % % % glassId = str2double(glassIdd);
baseDir = pwd;
workDir = pwd;
% % % imgNum = str2double(imgNumm);
Dx = str2double(DXX);
% % nnn = str2double(nn);

% % % inputDir = fullfile(baseDir,inputDirr);
% % % outputDir = fullfile(baseDir,outputDirr);
% % % calibFuncDir = fullfile(baseDir,calibFuncDirr);
tempDir = fullfile(calibFuncDir,'Temp');
MakeDirIfMissing(tempDir);

% % % % glassName = sprintf('%04d',glassId);
outputDirPath = outputDir;
% % % fprintf(outputDirPath);

MakeDirIfMissing(outputDirPath);  % creat corresponding folder to store images used for calibration

dstInfo = dir(fullfile(outputDirPath,'*.bmp'));

% % % % % % % % % while (length(dir(fullfile(inputDir,'*.bmp')))) < imgNum - length(dstInfo)
% % % % % % % % % %     fprintf(strcat('====',num2str(imgNum - length(dstInfo) - length(fullfile(inputDir,'*.bmp'))),'images left to capture===='));
% % % % % % % % %     fprintf(sprintf('==== %d images left to capture ====',(imgNum - length(dstInfo) - length(dir(fullfile(inputDir,'*.bmp'))))));
% % % % % % % % %     fprintf('\n\n');
% % % % % % % % %     return;
% % % % % % % % % end

% % % % fprintf('\n\n');
% % % % fprintf(sprintf('==== frame %d ..');
% % % % fprintf('\n\n');
for erq = 1:5
    pause(0.01);
if length(dir(fullfile(inputDir,'*.bmp'))) == 1
srcInfo = dir(fullfile(inputDir,'*.bmp'));
end
end
% % % % dstFileName = cell(imgNum,1);
% % % % 
% % % % for j = length(dstInfo)+1 : imgNum
% % % %     dstFileName{j,1} = sprintf('cap_frame_%03d.bmp', j); %strcat('cap_frame_',sprintf('%03d',j),'.bmp');
% % % % end



% % % % k = length(dstInfo)+1;
% % % % strL = fullfile(tempDir, 'tempL.bmp');
% % % % strR = fullfile(tempDir, 'tempR.bmp');
% % % % % % % % tic;
cropDir = fullfile(inputDir,'Temp');
MakeDirIfMissing(cropDir);
for fg = 1:5
    pause(0.01);
while length(dir(fullfile(cropDir,'*.bmp'))) ~= 0
delete(fullfile(cropDir,'*.bmp'))
end
end
config.dX = Dx; config.dY = Dx;
for i = 1 : length(srcInfo)
    
   
    
    [cbImgListL, cbImgListR] = PreprocZedImgPair4(inputDir, cropDir);
% %     [isblL, blL] = isBlur(imread(cbImgListL{1}),blbl);
% %     [isblR, blR] = isBlur(imread(cbImgListR{1}),blbl);
    
    [isblL, blL] = isBlur2(imread(cbImgListL{1}),blbl);
    [isblR, blR] = isBlur2(imread(cbImgListR{1}),blbl);
    
    
    if isblL + isblR == 0
    [goodIdL,cbcXYLLL] = CbCheckSingle3(cbImgListL,calibFuncDir, modeE,config);
% % % % % pause;
    [goodIdR,cbcXYRRR] = CbCheckSingle3(cbImgListR, calibFuncDir,modeE,config);
    cd(baseDir);
    infoCrop = dir(fullfile(cropDir,'*.bmp'));
% % %     nameL = infoCrop(1).name;
% % %     nameR = infoCrop(2).name;
    nameL = cbImgListL{1}(length(cropDir)+2:end);
    nameR = cbImgListR{1}(length(cropDir)+2:end);
    if ~isempty(goodIdL) && ~isempty(goodIdR)
        movefile(cbImgListL{1}, outputDirPath); movefile(cbImgListR{1}, outputDirPath);
        leftImgLst = fullfile(outputDirPath,nameL);
        rightImgLst = fullfile(outputDirPath,nameR);
        outInfo = dir(fullfile(outputDirPath,'*.bmp'));
        cbcXYL = cbcXYLLL;
        cbcXYR = cbcXYRRR;
% % % % % % % % %         fprintf(sprintf('\n\n==== %d images captured ====\n\n',length(outInfo)/2));
    else
        leftImgLst = []; rightImgLst = []; cbcXYL = []; cbcXYR = [];
        fprintf('\n\n============================= capture failed, no corner ======\n\n');
    end
    
    else
        
        leftImgLst = []; rightImgLst = []; cbcXYL = []; cbcXYR = [];
        fprintf(sprintf('\n\n============================= capture failed, too blur %f and %f ======\n\n',blL,blR));
    end
    
% % % % % % % % % % %    img = imread(fullfile(inputDir,srcInfo(i).name)); 
% % % % % % % % % % %    imgL = img(:,1:size(img,2)/2,:);
% % % % % % % % % % %    imgR = img(:,(size(img,2)/2 + 1):end,:);
% % % % % % % % % % %    imwrite(imgL, strL);
% % % % % % % % % % %    imwrite(imgR, strR);
% % % % % % % % % % %    try
% % % % % % % % % % %        [callBack, initCbcXL, initCbcYL]  = get_checkerboard_corners3(0,calib_data,strL,calibFuncDir);
% % % % % % % % % % % %        cd(workDir);
% % % % % % % % % % %    catch
% % % % % % % % % % %        [initCbcXL, initCbcYL] = DetectCbCorner(rgb2gray(imgL));
% % % % % % % % % % %    end
% % % % % % % % % % %    try
% % % % % % % % % % %        [callBack, initCbcXR, initCbcYR]  = get_checkerboard_corners3(0,calib_data,strR,calibFuncDir);
% % % % % % % % % % % %        cd(workDir);
% % % % % % % % % % %    catch
% % % % % % % % % % %        [initCbcXR, initCbcYR] = DetectCbCorner(rgb2gray(imgR)); 
% % % % % % % % % % %    end
% % % % % % % % % % %    cd(workDir);
% % % % % % % % % % %    [initCbcXL, initCbcYL] = CbCoordSys(initCbcXL, initCbcYL);
% % % % % % % % % % %    [initCbcXR, initCbcYR] = CbCoordSys(initCbcXR, initCbcYR);
% % % % % % % % % % %    cbcL = cornerfinder([initCbcXL(:),initCbcYL(:)]',double(rgb2gray(imgL)),config.halfWinW,config.halfWinH);
% % % % % % % % % % %    cbcR = cornerfinder([initCbcXR(:),initCbcYR(:)]',double(rgb2gray(imgR)),config.halfWinW,config.halfWinH);
% % % % % % % % % % %  
% % % % % % % % % % %    vertL = cbcL(:,1:calib_data.n_sq_y+1);
% % % % % % % % % % %    vertR = cbcR(:,1:calib_data.n_sq_y+1);
% % % % % % % % % % %    [~,indVertL] = sort(vertL(2,:));
% % % % % % % % % % %    [~,indVertR] = sort(vertR(2,:));
% % % % % % % % % % %     
% % % % % % % % % % %    horiL = cbcL(:,1:calib_data.n_sq_y+1:size(cbcL,2)-calib_data.n_sq_y);
% % % % % % % % % % %    horiR = cbcR(:,1:calib_data.n_sq_y+1:size(cbcR,2)-calib_data.n_sq_y);
% % % % % % % % % % %    [~,indHoriL] = sort(horiL(1,:));
% % % % % % % % % % %    [~,indHoriR] = sort(horiR(1,:));
% % % % % % % % % % %    
% % % % % % % % % % %    
% % % % % % % % % % %    cbcXL_mat = reshape(cbcL(1,:),calib_data.n_sq_y+1,calib_data.n_sq_x+1);  cbcYL_mat = reshape(cbcL(2,:),calib_data.n_sq_y+1,calib_data.n_sq_x+1);
% % % % % % % % % % %    cbcXR_mat = reshape(cbcR(1,:),calib_data.n_sq_y+1,calib_data.n_sq_x+1);  cbcYR_mat = reshape(cbcR(2,:),calib_data.n_sq_y+1,calib_data.n_sq_x+1);
% % % % % % % % % % %    [~,indVertAllL] = sort(cbcYL_mat,1); [~,indVertAllR] = sort(cbcYR_mat,1);
% % % % % % % % % % %    [~,indHoriAllL] = sort(cbcXL_mat,2); [~,indHoriAllR] = sort(cbcXR_mat,2);
% % % % % % % % % % %    
% % % % % % % % % % %    if 0
% % % % % % % % % % %         figure, 
% % % % % % % % % % %         subplot(1,2,1); imshow(imgL); hold on; plot(cbcL(1, :), cbcL(2,:), '+m', cbcL(1, 1), cbcL(2,1), 'og','linewidth',2); plot(cbcL(1, (1:calib_data.n_sq_y+1)), cbcL(2, (1:calib_data.n_sq_y+1)),'-b','linewidth',2);title(sprintf('Left Frame %d',i));
% % % % % % % % % % %         subplot(1,2,2); imshow(imgR); hold on; plot(cbcR(1, :), cbcR(2,:), '+m', cbcR(1, 1), cbcR(2,1), 'og','linewidth',2); plot(cbcR(1, (1:calib_data.n_sq_y+1)), cbcR(2, (1:calib_data.n_sq_y+1)),'-b','linewidth',2);title(sprintf('Right Frame %d',i));
% % % % % % % % % % %         hold off
% % % % % % % % % % %     end
% % % % % % % % % % %    
% % % % % % % % % % %    
% % % % % % % % % % %    if size(cbcL,2) == (calib_data.n_sq_x+1)*(calib_data.n_sq_y+1) && ...
% % % % % % % % % % %       size(cbcR,2) == (calib_data.n_sq_x+1)*(calib_data.n_sq_y+1) && ...
% % % % % % % % % % %       sum(sum(indVertL - [1:(calib_data.n_sq_y+1)])) == 0 && sum(sum(indVertR - [1:(calib_data.n_sq_y+1)])) == 0 && ...
% % % % % % % % % % %       sum(sum(indHoriL - [1:(calib_data.n_sq_x+1)])) == 0 && sum(sum(indHoriR - [1:(calib_data.n_sq_x+1)])) == 0 && ...
% % % % % % % % % % %       sum(sum(indVertAllL - repmat([1:(calib_data.n_sq_y+1)]',1,calib_data.n_sq_x+1))) == 0 && ...
% % % % % % % % % % %       sum(sum(indVertAllR - repmat([1:(calib_data.n_sq_y+1)]',1,calib_data.n_sq_x+1))) == 0 && ...
% % % % % % % % % % %       sum(sum(indHoriAllL - repmat([1:(calib_data.n_sq_x+1)],calib_data.n_sq_y+1,1))) == 0 && ...
% % % % % % % % % % %       sum(sum(indHoriAllR - repmat([1:(calib_data.n_sq_x+1)],calib_data.n_sq_y+1,1))) == 0
% % % % % % % % % % %     
% % % % % % % % % % % % % % % % %        imwrite(img, fullfile(outputDirPath,dstFileName{k,1}));
% % % % % % % % % % %        
% % % % % % % % % % %        cd(inputDir);
% % % % % % % % % % %        eval(['!rename' 32 srcInfo(i).name 32 dstFileName{k,1}]);
% % % % % % % % % % %        cd(workDir);
% % % % % % % % % % %        movefile(fullfile(inputDir,dstFileName{k,1}), outputDirPath);  
% % % % % % % % % % % % % % % % % %        fprintf(fullfile(outputDirPath,dstFileName{k,1}));
% % % % % % % % % % %        k = k+1;
% % % % % % % % % % %    else
% % % % % % % % % % %        continue;
% % % % % % % % % % %    end
%    [abc, errs, avgerr] = fitline_ls(HomoCoord(vertColL, 1), [1:calib_data.n_sq_y+1], 1);
    
end
% % % % % toc;
dstInfoNew = dir(fullfile(outputDirPath,'*.bmp'));

% % % % % % % % if length(dstInfoNew) == imgNum
% % % % % % % %     fprintf('\n\n');
% % % % % % % %     fprintf('==== capture and detection finished ====');
% % % % % % % %     fprintf('\n\n');
% % % % % % % % else
% % % % % % % %     fprintf('\n\n');
% % % % % % % %     fprintf(sprintf('==== %d images failed the detection, need to recapture ====', (imgNum-length(dstInfoNew))));
% % % % % % % %     fprintf('\n\n');
% % % % % % % % end
cd(baseDir);
for sd = 1:5
pause(0.01)
while length(dir(fullfile(inputDir,'*.bmp'))) ~= 0
delete(fullfile(inputDir,'*.bmp')); % delete previously captured images
end
end




end