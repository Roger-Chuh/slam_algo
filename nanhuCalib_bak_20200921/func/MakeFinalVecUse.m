function [finalVec, errLen] = MakeFinalVecUse(whichCam, para, crop1,crop2,LutVecOrig2RectXNum,LutVecRect2OrigXNum,LutVecRect2OrigX,LutVecOrig2RectX,Orig2RectNameMatX,Rect2OrigNameMatX,LutVecOrig2RectY,LutVecRect2OrigY,LutVecOrig2RectYNum)

if ~isempty(crop1)
    crop1x = MakeCrop(crop1(1), 11);
    crop1y = MakeCrop(crop1(2), 10);
    crop2x = MakeCrop(crop2(1), 13);
    crop2y = MakeCrop(crop2(2), 12);
    
    err = [bin2dec(crop1x)-crop1(1) bin2dec(crop1y)-crop1(2) bin2dec(crop2x)-crop2(1) bin2dec(crop2y)-crop2(2)];
    
    LutVecOrig2RectX(37:46) = crop1y;  %crop1 Y(10)
    LutVecOrig2RectX(26:36) = crop1x;  % X(11)%                 LutVecOrig2RectX(60:71) LutVecOrig2RectX(47:59)...   %crop2 Y(12) X(13)
    LutVecOrig2RectX(60:71) = crop2y;  %crop2 Y(12)
    LutVecRect2OrigX(47:59) = crop2x;    %X(13)
end



fid111 = fopen(fullfile(para,strcat('LutCheck',whichCam,'.txt')),'w');

supposedLen = 25+21+25+11+12+LutVecOrig2RectXNum(7)*15+LutVecOrig2RectXNum(8)*14+LutVecRect2OrigXNum(7)*15+LutVecRect2OrigXNum(8)*14+LutVecOrig2RectXNum(7)*LutVecOrig2RectXNum(8)*28+LutVecRect2OrigXNum(7)*LutVecRect2OrigXNum(8)*26;
finalVec = [LutVecRect2OrigX(1:25)...
    LutVecOrig2RectX(37:46) LutVecOrig2RectX(26:36)...   %crop1 Y(10) X(11)%                 LutVecOrig2RectX(60:71) LutVecOrig2RectX(47:59)...   %crop2 Y(12) X(13)
    LutVecOrig2RectX(60:71) LutVecRect2OrigX(47:59)...   %crop2 Y(12) X(13)
    LutVecOrig2RectX(72:82) LutVecRect2OrigX(72:83)...
    LutVecOrig2RectX(sum(cell2mat(Orig2RectNameMatX(1:8,2)))+1:sum(cell2mat(Orig2RectNameMatX(1:9,2)))) LutVecOrig2RectY(sum(cell2mat(Orig2RectNameMatX(1:9,2)))+1:sum(cell2mat(Orig2RectNameMatX(1:10,2))))...
    LutVecRect2OrigX(sum(cell2mat(Rect2OrigNameMatX(1:8,2)))+1:sum(cell2mat(Rect2OrigNameMatX(1:9,2)))) LutVecRect2OrigY(sum(cell2mat(Rect2OrigNameMatX(1:9,2)))+1:sum(cell2mat(Rect2OrigNameMatX(1:10,2))))];

fprintf(fid111,'%s\n',(LutVecRect2OrigX(1:13)));
fprintf(fid111,'%s\n',(LutVecOrig2RectX(14:25)));
fprintf(fid111,'%s\n',([LutVecOrig2RectX(37:46) LutVecOrig2RectX(26:36)]));
fprintf(fid111,'%s\n',([LutVecOrig2RectX(60:71) LutVecRect2OrigX(47:59)]));
fprintf(fid111,'%s\n',(LutVecOrig2RectX(72:76)));
fprintf(fid111,'%s\n',(LutVecOrig2RectX(77:82)));
fprintf(fid111,'%s\n',(LutVecRect2OrigX(72:77)));
fprintf(fid111,'%s\n',(LutVecRect2OrigX(78:83)));
vec1 = [LutVecOrig2RectX(sum(cell2mat(Orig2RectNameMatX(1:8,2)))+1:sum(cell2mat(Orig2RectNameMatX(1:9,2))))];
vec1 = reshape(vec1,15,[]); vec1 = vec1';
for oo = 1 : size(vec1,1)
    fprintf(fid111,'%s\n',((vec1(oo,:))));
end
vec2 = [LutVecOrig2RectY(sum(cell2mat(Orig2RectNameMatX(1:9,2)))+1:sum(cell2mat(Orig2RectNameMatX(1:10,2))))];
vec2 = reshape(vec2,14,[]); vec2 = vec2';
for oo = 1 : size(vec2,1)
    fprintf(fid111,'%s\n',(vec2(oo,:)));
end
vec3 = [LutVecRect2OrigX(sum(cell2mat(Rect2OrigNameMatX(1:8,2)))+1:sum(cell2mat(Rect2OrigNameMatX(1:9,2))))];
vec3 = reshape(vec3,15,[]); vec3 = vec3';
for oo = 1 : size(vec3,1)
    fprintf(fid111,'%s\n',(vec3(oo,:)));
end
vec4 = [LutVecRect2OrigY(sum(cell2mat(Rect2OrigNameMatX(1:9,2)))+1:sum(cell2mat(Rect2OrigNameMatX(1:10,2))))];
vec4 = reshape(vec4,14,[]); vec4 = vec4';
for oo = 1 : size(vec4,1)
    fprintf(fid111,'%s\n',(vec4(oo,:)));
end
% forward
for jj = 1 : LutVecOrig2RectYNum(7) * LutVecOrig2RectYNum(8)
    unitLength1 = Orig2RectNameMatX{11,2}/size(Orig2RectNameMatX{11,1},1);
    startId1 = sum(cell2mat(Orig2RectNameMatX(1:10,2)));
    tmp =  [LutVecOrig2RectY(startId1+(jj-1)*unitLength1+1:startId1+jj*unitLength1) LutVecOrig2RectX(startId1+(jj-1)*unitLength1+1:startId1+jj*unitLength1)];
    fprintf(fid111,'%s\n',(tmp));
    finalVec = [finalVec tmp];
end
% reverse
for kk = 1 : LutVecRect2OrigXNum(7) * LutVecRect2OrigXNum(8)
    unitLength2 = Rect2OrigNameMatX{11,2}/size(Rect2OrigNameMatX{11,1},1);
    startId2 = sum(cell2mat(Rect2OrigNameMatX(1:10,2)));
    tmp =  [LutVecRect2OrigY(startId2+(kk-1)*unitLength2+1:startId2+kk*unitLength2) LutVecRect2OrigX(startId2+(kk-1)*unitLength2+1:startId2+kk*unitLength2)];
    fprintf(fid111,'%s\n',(tmp));
    finalVec = [finalVec tmp];
    
end
fclose(fid111);

errLen = supposedLen - length(finalVec);

fid1 = fopen(fullfile(para,strcat('Lut',whichCam,'.txt')),'w');%????
for i = 1 : size(finalVec,2)
    fprintf(fid1,'%d\n',str2double(finalVec(i)));
end
fclose(fid1);


aaa = load(fullfile(para, strcat('Lut',whichCam,'.txt')));
aaa = aaa(:);


intLength = floor(length(aaa)/8);
leftover = aaa(8*intLength+1:end);
aaa_ = aaa;
for t = 1 : (8-length(leftover))
    aaa_ = [aaa_;0];
end
aaa_Mat = reshape(aaa_,8,[]);
ccc = bin2dec(num2str(aaa_Mat'));
% calibRomFid = fopen(fullfile(para,strcat('lut8',whichCam,'.bin')),'wb');
calibRomFid = fopen(fullfile(para,strcat('lut',whichCam,'.bin')),'wb');
%     fwrite(calibRomFid, aaa,'ubit1');
fwrite(calibRomFid, [ccc],'uint8');
fclose(calibRomFid);



end

function bitVec = MakeCrop(cropCoord, bitLen)

bitVec_ = dec2bin(cropCoord);
bitVec = sprintf(strcat('%0',num2str(bitLen),'s'),bitVec_);
% 
% if length(bitVec_) < bitLen
%     for i = 1 : (bitLen - length(bitVec_))
%         bitVec_ = ['0' bitVec_];
%     end
%     bitVec = bitVec_;
% else
%     bitVec = bitVec_;
% end
% 


end