function [camPosePath, timeStamp, dist, logg] = readBeiDump(inputDir)

angThr = 2;

dirInfo = dir(fullfile(inputDir,'*.log'));

try
    log = load(fullfile(inputDir, dirInfo(1).name));
catch
    log = readLog(fullfile(inputDir, dirInfo(1).name));
end

% % figure,imshow(zeros(1080,1920));hold on;
for i = 1 : size(log,1)
    r = rodrigues(log(i,2:4));
    camPath2(i,:) =  [r(:)' 1000.*log(i,5:7) log(i,1)];
    
    
    xList(i,:) = log(i,8:2:end);
    yList(i,:) = log(i,9:2:end);
    % %     if i > 1
    % %         Dist(i-1,:) = NormalizeVector(xList(i,:) - xList(i-1,:));
    % %     end
    
    % %     plot(xList(i,:),yList(i,:),'-');plot(xList(i,1),yList(i,1),'*b');drawnow;
end


Ind = [];
for i = 1 : size(log,1)
    ind = find(log(:,1) == log(i,1));
    if length(ind) == 1
        Ind = [Ind; ind];
    end
end
IndMore = setdiff([1:size(log,1)]',Ind);

if 0
    rotVec = log(Ind,2:4);
    
    [rotVecNorm,~] = NormalizeVector(rotVec);
    
    for e = 2 : size(rotVecNorm,1)
        if norm(rotVecNorm(e,:) - rotVecNorm(1,:)) > 1.5
            rotVecNorm(e,:) = -rotVecNorm(e,:);
        end
    end
end


camPath3 = camPath2(Ind,:);
log3 = log(Ind,:);

[BB3, ~, inliers3] = ransacfitplane(camPath3(:,10:12)', 100);
BB3 = BB3./norm(BB3(1:3)); N = BB3(1:3);
indGood = [];Mid = [];RVec = [];
for ij = 1 : length(IndMore)
    inde = find(log(:,1) == log(IndMore(ij),1));
    deg = [];
    for ji = 1 : length(inde)
        rVec = log(inde(ji),2:4)./norm(log(inde(ji),2:4));
        
        deg(ji,:) = CalcDegree(N,rVec);
    end
    [mid,id] = min(abs(deg - 90));
    Mid = [Mid;mid];
    if mid < angThr
        indGood = [indGood;inde(id)];
    end
end

Ind2 = sort([Ind;indGood]);

if 0
    camPath4 = camPath2(Ind2,:);
    log4 = log(Ind2);
else
    camPath4 = camPath3;
    log4 = log3;
end
[BB, ~, inliers] = ransacfitplane(camPath4(:,10:12)', 50);


camPath = camPath4(inliers,:);
logg = log4(inliers,:);

for z = 1 : size(logg,1)
    xListt(z,:) = logg(z,8:2:end);
    yListt(z,:) = logg(z,9:2:end);
    if z > 1
        [~,Dist(z-1,:)] = NormalizeVector([xListt(z,:) - xListt(z-1,:);yListt(z,:) - yListt(z-1,:)]');
    end
    
end
dist = mean(Dist')';
distF = medfilt1(dist,2*2-1);

camPosePath = camPath(:,1:end - 1);
timeStamp = camPath(:,end)./1000;

figure, plotPath(camPath);
fig_3D3_pair(camPath(:,10:12)',camPath(:,10:12)');



for k = 1 : size(camPath,1)
    rr = reshape(camPath(k,1:9),3,3);
    xAxisList(:,k) = rr(:,1)';
    yAxisiList(:,k) = rr(:,2)';
    zAxisiList(:,k) = rr(:,3)';
    
   rotVecList(:,k) = rodrigues(reshape(camPath(k,1:9),3,3))./norm(rodrigues(reshape(camPath(k,1:9),3,3))); 
    
    
end

figure,plotQuiver(rotVecList',[1 0 0]);

end
function [acc] = readLog(accFilePath)

[configFileFid, errMsg] = fopen(accFilePath);
if (configFileFid < 0)
    error('Cannot open %s for read: %s', accFilePath, errMsg);
end
% gVecBuf = [];
% inReadgVec = false;
% flagg=0;

cnt = 1;
while ~feof(configFileFid)
    lineBuf = strtrim(fgetl(configFileFid));
    if length(str2double(strsplit(lineBuf(1:end), ','))) == 15
        acc(cnt,:) = str2double(strsplit(lineBuf(1:end), ','));
        cnt = cnt + 1;
        % % %     catch
        % % %         sadvkj = 1;
    end
    
end
% G=1;
% g=gVec;
fclose(configFileFid);

end