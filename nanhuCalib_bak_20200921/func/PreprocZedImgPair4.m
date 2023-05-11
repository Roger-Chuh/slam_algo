function [leftImgLst, rightImgLst] = PreprocZedImgPair4(zedImgFolder, outputFolder)


% % imgNamePrefix = 'z';
imgNamePrefix = '';
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
    imgNo = regexp(fileName, sprintf('%s(\\d{1,5}).bmp', imgNamePrefix), 'tokens');
    if isempty(imgNo)
        continue;
    end
    
    try
    img = imread(fullfile(zedImgFolder, fileName));
    [~,w,~] = size(img);
    assert(mod(w,2)==0, 'The width of zed stero camera input must be an even number.');
    % % %
    imgLeft = img(:, 1:w/2, :);
    imgRight = img(:, w/2+1:end, :);
    leftImgPath = fullfile(GetFullPath(outputFolder), sprintf('%sleft%s.bmp', imgNamePrefix, imgNo{1}{1}));
    rightImgPath = fullfile(GetFullPath(outputFolder), sprintf('%sright%s.bmp', imgNamePrefix, imgNo{1}{1}));
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

