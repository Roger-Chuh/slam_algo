function LutDec2Hex(inputDir, whichOne)

dirInfo = dir(fullfile(inputDir, 'LutCheck*.txt'));



for i = 1 : length(dirInfo)
    [HexBuf] = readLut(fullfile(inputDir,dirInfo(i).name));
    if strcmp(whichOne,'L')
        %     fid111 = fopen(fullfile(inputDir,strcat('LutHexL',dirInfo(i).name(end-6:end))),'w');
        fid111 = fopen(fullfile(inputDir,'LutHexL.txt'),'w');
    else
        %        fid111 = fopen(fullfile(inputDir,strcat('LutHexR',dirInfo(i).name(end-6:end))),'w');
        fid111 = fopen(fullfile(inputDir,'LutHexR.txt'),'w');
    end
    for j = 1:length(HexBuf)
        fprintf(fid111,'%s\n',HexBuf{j});
    end
    fclose(fid111);
    
end


end
function [hexStr] = readLut(accFilePath)
[configFileFid, errMsg] = fopen(accFilePath);
if (configFileFid < 0)
    error('Cannot open %s for read: %s', accFilePath, errMsg);
end
% gVecBuf = [];
% inReadgVec = false;
% flagg=0;

cnt = 1;
widthFlag = 0;
intrFlag = 0;
extrFlag = 0;
intrMat = [];
extrMat = [];
while ~feof(configFileFid)
    lineBuf = strtrim(fgetl(configFileFid));
    %     binStr = str2double((lineBuf(1:end)));
    binStr = ((lineBuf(1:end)));
    hexStr{cnt,:} = dec2hex(bin2dec(binStr));
    cnt = cnt + 1;
    
end
% G=1;
% g=gVec;
fclose(configFileFid);




end