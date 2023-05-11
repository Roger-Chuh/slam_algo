function LutDec2HexUse(inputDir, whichOne)
if strcmp(whichOne,'L')
    dirInfo = dir(fullfile(inputDir, 'LutCheckL.txt'));
else
    dirInfo = dir(fullfile(inputDir, 'LutCheckR.txt'));
end

for i = 1 : length(dirInfo)
    [HexBuf] = readLut(fullfile(inputDir,dirInfo(i).name), inputDir, whichOne);
    if strcmp(whichOne,'L')
        %     fid111 = fopen(fullfile(inputDir,strcat('LutHexL',dirInfo(i).name(end-6:end))),'w');
        fid111 = fopen(fullfile(inputDir,'LutHexL.txt'),'w');
    else
        %        fid111 = fopen(fullfile(inputDir,strcat('LutHexR',dirInfo(i).name(end-6:end))),'w');
        fid111 = fopen(fullfile(inputDir,'LutHexR.txt'),'w');
    end
    for j = 1:length(HexBuf)
        %         fprintf(fid111,'%s\n',HexBuf{j});
        fprintf(fid111,sprintf('0x%s,\n',HexBuf{j}));
    end
    fclose(fid111);
    
end


end
function [hexStr] = readLut(accFilePath, inputDir, whichOne)
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
header = {};
finalVec = [];
while ~feof(configFileFid)
    lineBuf = strtrim(fgetl(configFileFid));
    %     binStr = str2double((lineBuf(1:end)));
    binStr = ((lineBuf(1:end)));
    
    if cnt == 1  % image column
        header = [header; {binStr}];
    end
    
    if cnt == 2  % image row
        header = [header; {binStr}];
    end
    
    if cnt == 3  % crop top left x y
        header = [header; {binStr(end-10:end)}; {binStr(1:10)}];
    end
    if cnt == 4 % crop bottom right x y
        header = [header; {binStr(end-12:end)}; {binStr(1:12)}];
    end
    if cnt == 5 % forward column
        header = [header; {binStr}];
    end
    
    if cnt == 6 % forward row
        header = [header; {binStr}];
    end
    
    if cnt == 7 % reverse column
        header = [header; {binStr}];
    end
    
    if cnt == 8 % reverse row
        header = [header; {binStr}];
    end
    
    
    if cnt > 8
        finalVec = [finalVec; binStr'];
        
    end
    
    hexStr{cnt,:} = dec2hex(bin2dec(binStr));
    cnt = cnt + 1;
    
end


intLength = floor(length(finalVec)/8);
leftover = finalVec(8*intLength+1:end);
aaa_ = finalVec;
lastCol = leftover';
for t = 1 : (8-length(leftover))
    aaa_ = [aaa_;num2str(0)];
    lastCol = [num2str(0) lastCol];
end
aaa_Mat = reshape(aaa_,8,[]);
% ccc = bin2dec((aaa_Mat'));
aaa_Mat1 = [aaa_Mat(:,1:end-1) lastCol'];

% G=1;
% g=gVec;
fclose(configFileFid);

header1 = header([1 2 10 9 8 7 4 3 5 6]);
header1 = [dec2bin(163);header1];
fid111 = fopen(fullfile(inputDir,strcat('LutTableHex',whichOne,'.txt')),'w');
fid222 = fopen(fullfile(inputDir,strcat('LutTableDec',whichOne,'.txt')),'w');
binMat = [];
% fid333 = fopen(fullfile(inputDir,strcat('LutTableBin',whichOne,'.bin')),'w');
for i = 1 : length(header1) + 3
    if i < length(header1) + 1
        tempHex = dec2hex(bin2dec(header1{i}));
        tempDec = bin2dec(header1{i});
        tempBin = header1{i};
        fprintf(fid111,sprintf('0x%s,\n',tempHex));
        fprintf(fid222,sprintf('%d\n',tempDec));
        binMat = [binMat; sprintf('%032s',tempBin)];
%         fprintf(fid333,sprintf('%032s\n',tempBin));
        
    end
    if i == length(header1) + 1
        tempHex = '5A5A5A5A';
        tempDec = hex2dec(tempHex);
        tempBin = dec2bin(tempDec);
        
        fprintf(fid111,sprintf('0x%s,\n',tempHex));
        fprintf(fid222,sprintf('%d\n',tempDec));
        binMat = [binMat; sprintf('%032s',tempBin)];
%         fprintf(fid333,sprintf('%032s\n',tempBin));
        
        
    end
    if i == length(header1) + 2
        if 0
            tempHex = dec2hex(size(aaa_Mat1,2));
        else
            tempHex = dec2hex(length(hexStr) - 8);
            tempDec = hex2dec(tempHex);
            tempBin = dec2bin(tempDec);
            
        end
        fprintf(fid111,sprintf('0x%s,\n',tempHex));
        fprintf(fid222,sprintf('%d\n',tempDec));
        binMat = [binMat; sprintf('%032s',tempBin)];
%         fprintf(fid333,sprintf('%032s\n',tempBin));
    end
    if i == length(header1) + 3
        %         tempHex = dec2hex(bin2dec(header1{i}));
%         tempHex = 'ABCDEFEA';
        tempHex = 'FFFFFFFF';
        tempDec = hex2dec(tempHex);
        tempBin = dec2bin(tempDec);
        
        fprintf(fid111,sprintf('0x%s,\n',tempHex));
        fprintf(fid222,sprintf('%d\n',tempDec));
        binMat = [binMat; sprintf('%032s',tempBin)];
%         fprintf(fid333,sprintf('%032s\n',tempBin));
    end
    
end

if 0
    for j = 1 : size(aaa_Mat1,2)
        tempHex = dec2hex(bin2dec(aaa_Mat1(:,j)'));
        fprintf(fid111,sprintf('0x%s,\n',tempHex));
    end
else
    
    for j = 9 : length(hexStr)
        %          tempHex = dec2hex(bin2dec(aaa_Mat1(:,j)'));
        %         fprintf(fid111,sprintf('0x%s,\n',tempHex));
        if j < length(hexStr)
            
            tempHex = hexStr{j};
            tempDec = hex2dec(tempHex);
            tempBin = dec2bin(tempDec);
            %             fprintf(fid111,sprintf('0x%s,\n',hexStr{j}));
            fprintf(fid111,sprintf('0x%s,\n',tempHex));
            fprintf(fid222,sprintf('%d\n',tempDec));
            binMat = [binMat; sprintf('%032s',tempBin)];
%             fprintf(fid333,sprintf('%032s\n',tempBin));
        else
            %             fprintf(fid111,sprintf('0x%s,',hexStr{j}));
            
            tempHex = hexStr{j};
            tempDec = hex2dec(tempHex);
            tempBin = dec2bin(tempDec);
            %             fprintf(fid111,sprintf('0x%s,\n',hexStr{j}));
            fprintf(fid111,sprintf('0x%s,',tempHex));
            fprintf(fid222,sprintf('%d',tempDec));
            binMat = [binMat; sprintf('%032s',tempBin)];
%             fprintf(fid333,sprintf('%032s',tempBin));
            
        end
    end
    
end
fclose(fid111);
fclose(fid222);
% fclose(fid333);
BinMat = binMat';
% BinMat = flipud(BinMat);
% BinMat = [BinMat(17:end,:); BinMat(1:16,:)];

BinMat = [BinMat(9:16,:);BinMat(1:8,:); BinMat(25:32,:);BinMat(17:24,:)];
BinMat = flipud(BinMat);
BinMat = [BinMat(17:end,:); BinMat(1:16,:)];
BinMatVec = str2num(BinMat(:));
if 0
    aaa = (load(fullfile(inputDir,strcat('LutTableDec',whichOne,'.txt'))));
    calibRomFid = fopen(fullfile(inputDir,strcat('LutTableBin',whichOne,'.bin')),'wb');
    %     fwrite(calibRomFid, aaa,'ubit1');
    fwrite(calibRomFid, [aaa],'uint32');
    fclose(calibRomFid);
else
    
%     aaa = (load(fullfile(inputDir,strcat('LutTableDec',whichOne,'.txt'))));
    calibRomFid = fopen(fullfile(inputDir,strcat('LutTableBin',whichOne,'.bin')),'wb');
    %     fwrite(calibRomFid, aaa,'ubit1');
    fwrite(calibRomFid, [BinMatVec],'ubit1');
    fclose(calibRomFid);
    
end
calibRomFid = fopen(fullfile(inputDir,strcat('LutTableBin',whichOne,'.bin')),'rb');
qqq = fread(calibRomFid,'uint32');
fclose(calibRomFid);
if 0
    figure,plot(aaa-qqq)
end
end