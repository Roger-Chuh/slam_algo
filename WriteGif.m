function WriteGif(inputDir, gifName,delay)
dirInfo = dir(fullfile(inputDir, '*.png'));
nImages = length(dirInfo);
filename = fullfile(inputDir, gifName); % Specify the output file name
[a,b,c] = size(imread(fullfile(inputDir, dirInfo(end).name)));

for idx = 1:nImages
    if 0
        img = imresize(imread(fullfile(inputDir, dirInfo(idx).name)),[360,640]);
    else
        imgg = imread(fullfile(inputDir, dirInfo(idx).name));
%         img = imgg(71:1287, 611:2236,:);
        img = imgg;
    end
% %     img = imresize(img(21:386,143:583,:), 2);
    [A,map] = rgb2ind(img,256);
    if idx == 1
        for j = 1
            imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',delay);
        end
    else
        if 0
            imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',1);
        else
            imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',delay);
        end
    end
end
end