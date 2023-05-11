function [leftImgLst, rightImgLst] = PreprocZedImgPair1(zedImgFolder, outputFolder,filename,swapLR)


% % imgNamePrefix = 'z';
imgType = filename(end-2:end);
imgNamePrefix = filename(1:end-3);
if isempty(imgNamePrefix);
    imgNamePrefix = '';
end
% % % imgNamePrefix = 'frame_';
% % imgNamePrefix = 'a';

% imgNamePrefix = 'rec_31154518_';
% % imgNamePrefix = 'cbpair';
% imgNamePrefix = 'z';

% % % delete(fullfile(outputFolder,'*.bmp')); % delete previously captured images


leftImgLst = [];
rightImgLst = [];
dirInfo = dir(zedImgFolder);
for iItm = 1 : length(dirInfo)
    fileName = dirInfo(iItm).name;
    if strcmp(imgNamePrefix,'cbpair')
        imgNo = regexp(fileName, sprintf(strcat('%s(\\d{3}).',imgType), imgNamePrefix), 'tokens');
    else
        imgNo = regexp(fileName, sprintf(strcat('%s(\\d{1,5}).',imgType), imgNamePrefix), 'tokens');
    end
    if isempty(imgNo)
        continue;
    end
    
    try
    img = imread(fullfile(zedImgFolder, fileName));
    [~,w,~] = size(img);
    assert(mod(w,2)==0, 'The width of zed stero camera input must be an even number.');
    % % %
    if swapLR == 0
       imgLeft = img(:, 1:w/2, :);
       imgRight = img(:, w/2+1:end, :);
    else
        imgRight = img(:, 1:w/2, :);
        imgLeft = img(:, w/2+1:end, :);
    end
    leftImgPath = fullfile(GetFullPath(outputFolder), sprintf(strcat('%s_left%s.',imgType), imgNamePrefix, imgNo{1}{1}));
    rightImgPath = fullfile(GetFullPath(outputFolder), sprintf(strcat('%s_right%s.',imgType), imgNamePrefix, imgNo{1}{1}));
    imwrite(imgLeft, leftImgPath);
    imwrite(imgRight, rightImgPath);
    leftImgLst = [leftImgLst; {leftImgPath}];
    rightImgLst = [rightImgLst, {rightImgPath}];
    catch
        iItm
        continue;
    end
end

end

