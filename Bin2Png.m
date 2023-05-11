function Bin2Png(inputDir, outputDir)

dirInfo = dir(fullfile(inputDir, 'MapperCurImage*.bin'));
frameNum = length(dirInfo)/2;

for i = 1 : frameNum
    fid3=fopen(fullfile(inputDir,dirInfo(i).name),'r');
    D1 = fread(fid3,'uint8');fclose(fid3);
    imgR = reshape(D1,640,480)';
    
    fid3=fopen(fullfile(inputDir,dirInfo(i + frameNum).name),'r');
    D1 = fread(fid3,'uint8');fclose(fid3);
    imgL = reshape(D1,640,480)';
    
    imwrite(uint8(imgL), fullfile(outputDir, sprintf('rectL_%05d.png',i)));
    imwrite(uint8(imgR), fullfile(outputDir, sprintf('rectR_%05d.png',i)));
    
    
end


end