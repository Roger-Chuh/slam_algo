function testQuanThoughts3()
inputDir = 'D:\Temp\20200728\rectify\rectify';
inputDir = 'D:\Temp\20200829';
dirInfo = dir(fullfile(inputDir, '*.jpg'));


for i = 1 : length(dirInfo)
    id_ = find(dirInfo(i).name == '_');
    timeNum1(i,1) = str2double(dirInfo(i).name(7:id_(1)-1));
    
end

[B,I] = sort(timeNum1);

dirInfoNew = dirInfo;
for i = 1 : length(dirInfo)
    dirInfoNew(i).name = dirInfo(I(i)).name;
end
dirInfo = dirInfoNew;


ts = [5300:10:5600];


% for i = 61:length(dirInfo)
for i = 1:length(dirInfo)
    img = imread(fullfile(inputDir, dirInfo(i).name));
    figure(1), imshow(img); hold on
    title(dirInfo(i).name);
%     [x1, y1, button] = ginput(1);

    
end


end