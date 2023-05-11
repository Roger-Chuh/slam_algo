function imgList = getImgList(inputDir)

dirInfo = dir(fullfile(inputDir, '*.png'));
if length(dirInfo) == 0
    dirInfo = dir(fullfile(inputDir, '*.bmp'));
end
if length(dirInfo) == 0
    dirInfo = dir(fullfile(inputDir, '*.jpg'));
end   
for i = 1 : length(dirInfo)
    imgList{i,1} = fullfile(inputDir, dirInfo(i).name);
end

end