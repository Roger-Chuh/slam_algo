function imgLst = getImgLstNew(inputDir,idImg,prefix,suffix)

dirInfo = dir(fullfile(inputDir,strcat(strcat(prefix,'*.'),suffix)));
frameNum = length(dirInfo)/2;
imgLst = cell(2*length(idImg),1);
for i = 1 : length(idImg)
    imgLst{i,1} = fullfile(inputDir,dirInfo(idImg(i)).name);
    imgLst{length(idImg) + i,1} = fullfile(inputDir,dirInfo(frameNum + idImg(i)).name);
end




% % 
% % for j = 1 : 23
% %     figure,imshow(imread(imgListRaw{j}));hold on;plot(cbcXYLL{j}(1,:),cbcXYLL{j}(2,:),'xr',cbcXYRR{j}(1,:),cbcXYRR{j}(2,:),'xg')
% % end
    
    



end