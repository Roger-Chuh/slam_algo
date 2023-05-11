function [xMat, yMat] = RecoverLut(file,xcoord,ycoord)
[X_Dec, Y_Dec] = readLog(file);
xMat = reshape(X_Dec,length(ycoord),length(xcoord))./2^5;
yMat = reshape(Y_Dec,length(ycoord),length(xcoord))./2^5;
end



function [X_Dec, Y_Dec] = readLog(accFilePath)

[configFileFid, errMsg] = fopen(accFilePath);
if (configFileFid < 0)
    error('Cannot open %s for read: %s', accFilePath, errMsg);
end
% gVecBuf = [];
% inReadgVec = false;
% flagg=0;
Y_Dec = [];
X_Dec = [];
cnt = 1;
while ~feof(configFileFid)
    lineBuf = strtrim(fgetl(configFileFid));
    Y =  lineBuf(1:length(lineBuf)/2);
    X =  lineBuf(length(lineBuf)/2+1:end);
    Y_dec = bin2dec(Y);
    if strcmp(Y(1),'1')
       Y_dec = Y_dec - 2^length(Y);
    end
    X_dec = bin2dec(X);
    if strcmp(X(1),'1')
        X_dec = X_dec - 2^length(X);
    end
    Y_Dec = [Y_Dec; Y_dec];
    X_Dec = [X_Dec; X_dec];
% %     if length(str2double(strsplit(lineBuf(1:end), ','))) == 15
% %         acc(cnt,:) = str2double(strsplit(lineBuf(1:end), ','));
% %         cnt = cnt + 1;
% %         % % %     catch
% %         % % %         sadvkj = 1;
% %     end
    
end
% G=1;
% g=gVec;
fclose(configFileFid);

end