function imgLst = getImgLst(inputDir,suffix)

dirInfo = dir(fullfile(inputDir,strcat('*.',suffix)));
frameNum = length(dirInfo);
for i = 1 : frameNum
    imgLst{i,1} = fullfile(inputDir,dirInfo(i).name);
end




% % 
% % for j = 1 : 23
% %     figure,imshow(imread(imgListRaw{j}));hold on;plot(cbcXYLL{j}(1,:),cbcXYLL{j}(2,:),'xr',cbcXYRR{j}(1,:),cbcXYRR{j}(2,:),'xg')
% % end
    
    



end