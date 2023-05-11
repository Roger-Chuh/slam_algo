function [DistAng1, DistAng2,angRangeInd,IndRange2,DistAng] = getDistAngInfo2(accumAng, camPoseMatNew, angThr11,angThr22,window,marginInd,minNum,dist,tss,varargin)
% marginInd = 2;
% % draw = 0;

minNumDist = 12;

if (nargin == 9)
    draw = 1;
elseif (nargin == 10)
    draw = varargin{1};
else
    error('Too many input arguments');
end



marginIndInd = 0;2;5;2;1;
if 0 % accumAng(end) < 0
    accumAng = -accumAng;
end

% % draw = 1;

angThr2 = angThr11 + 0; %1; %0;  %1;
dltDistRoboTempNewAng = abs(diff(accumAng));
dltDistRoboFiltTempNewAng = medfilt1(dltDistRoboTempNewAng,(2*window - 1));

angVelo = diff(accumAng)./diff(tss);


if 1
    dltDistRoboTempNew = dist;
    dltDistRoboFiltTempNew = medfilt1(dltDistRoboTempNew,(2*window - 1));
else
    dltDistRoboFiltTempNew = dist;
end


% % [segInfoRobo, MiniRobo, DistAng] = segPath([repmat(camPoseMatNew,1,4) tss accumAng], 6, 6, 15,10,1,draw);


%     dltDistRobo = medfilt1(dltDistRobo1,(2*10 - 1));
[indexCellRoboNew1111,idxRoboNew] = splitIndex2(find(dltDistRoboFiltTempNew > angThr11));  %  & dltDistRoboFiltTempNew < angThr22));
[indexCellRoboNew2,idxRoboNew2] = splitIndex2(find(dltDistRoboFiltTempNew <= angThr2));
validInd = [];
indexCellRoboNew11112 = {};cttt = 1;plusInd = 0;
for y = 1 : length(indexCellRoboNew1111)
    diffAngDistribution(y,1) = median(dltDistRoboFiltTempNewAng(indexCellRoboNew1111{y}));
    
    if length(indexCellRoboNew1111{y}) > minNumDist
        validInd = [validInd; y];
        indexCellRoboNew11112{cttt,1} = cell2mat(indexCellRoboNew1111(y+plusInd :y,1)');
        cttt = cttt + 1;
    else
        slv = 1;
        if median(dltDistRoboFiltTempNewAng(indexCellRoboNew1111{y})) > 1
            if median(dltDistRoboFiltTempNewAng(indexCellRoboNew1111{y-1})) < 0.5
                plusInd = -1;
            end
            % %             if median(dltDistRoboFiltTempNewAng(indexCellRoboNew1111{y-1})) < 0.5
            % %                 plus = 1;
            % %             end
        end
    end
end
[indexCellDistr,idxDistr] = splitIndex2(find(diffAngDistribution > angThr22));

asdghi = 1;
if 0
    indexCellRoboNew111 = indexCellRoboNew1111(validInd,:);
    rotSeg = [];
    for u = 1 : length(indexCellRoboNew111)
        medAng(u,:) = median(dltDistRoboFiltTempNewAng(indexCellRoboNew111{u}));
        if medAng(u,:) > 1.5
            rotSeg = [rotSeg; u];
        end
    end
    
    indexCellRoboNew11 = indexCellRoboNew111(rotSeg);
else
    for i = 1 : length(indexCellDistr)
        iiid = indexCellDistr{i};
        indexCellRoboNew11{i,1} = cell2mat(indexCellRoboNew1111(iiid,1)');
    end
end


id = [];
for oo = 1 : length(indexCellRoboNew11)
    if length(indexCellRoboNew11{oo}) >= minNum
        id = [id;oo];
    end
end
if 0
    for w = 1 : length(id)
        dlt1 = abs(indexCellRoboNew11{id(w)-1}(end) - indexCellRoboNew11{id(w)}(1));
        dlt2 = abs(indexCellRoboNew11{id(w)+1}(1) - indexCellRoboNew11{id(w)}(end));
        if dlt1 < dlt2
            adlfk = 1;
        end
    end
end


if ~isempty(id)
    indexCellRoboNew = indexCellRoboNew11(id,:);
else
    indexCellRoboNew = indexCellRoboNew1111;
end

try
    for w = 1 : length(indexCellRoboNew) + 1
        if w == 1
            idEval{w,1} = [1 round(mean(indexCellRoboNew{w}))];
        elseif w == length(indexCellRoboNew) + 1
            idEval{w,1} = [idEval{w-1,1}(2) length(accumAng)];
        else
            idEval{w,1} = [idEval{w-1,1}(2) round(mean(indexCellRoboNew{w}))];
        end
    end
    idx = cell2mat(idEval');
    idx = unique(idx);
    idxAll = zeros(1,length(dltDistRoboFiltTempNew) + 1);
    idxAll(idx) = 10;
    idxAll2 = zeros(1,length(dltDistRoboFiltTempNew) + 1);
    idxAll2(idx) = max(abs(accumAng));
    
    
    if draw == 1
        figure(101),clf;subplot(3,1,1);plot(accumAng,'-');title('ceiling');hold on;plot(idxAll2,'r');
        subplot(3,1,2);plot(dltDistRoboTempNew);title(sprintf('before filter, windowsize = %d',window));
        subplot(3,1,3);plot(dltDistRoboFiltTempNew);title(sprintf('after filter, angle threshold = %.3f', angThr11));hold on;plot(idxAll,'r');hold on;plot(idxAll,'r');
    end
catch
    
    fprintf(sprintf('\n### sth wrong in drawing ###\n'));
end


IndRange1 = [];
for q = 1 : length(indexCellRoboNew)
    
    if q >= 45
        askh = 1;
    end
    indRange1 = indexCellRoboNew{q,1}([1 end]);
    % %     indRange1Ang = [indRange1(1) - marginInd min(indRange1(2) + marginInd + marginIndInd, size(camPoseMatNew,1))];% loose
    % %     indRange1Dist = [min(indRange1(1) + marginInd, size(camPoseMatNew,1)) min(indRange1(2) - marginInd - marginIndInd, size(camPoseMatNew,1))];%tight
    
    indRange1Ang = [max(1,indRange1(1) - marginInd - 0) min(indRange1(2) + marginInd + marginIndInd, size(camPoseMatNew,1))];% loose
    if 0
        indRange1Dist = [min(indRange1(1) + marginInd +0, size(camPoseMatNew,1)) min(indRange1(2) - marginInd - marginIndInd, size(camPoseMatNew,1))];%tight
    else
        indRange1Dist = indRange1Ang;
    end
    IndRange1 = [IndRange1; [indRange1Ang indRange1Dist]];
    if q > 2
        if IndRange1(q,1) < IndRange1(q-1,2);
            sldvnk = 1;
        end
    end
    %     DistAng1(q,1) = norm(camPoseMatNew(indRange1(1),[10:12]) - camPoseMatNew(indRange1(2),[10:12]));
    try
        DistAng1(q,1) = norm(camPoseMatNew(indRange1Dist(1),[1:3]) - camPoseMatNew(indRange1Dist(2),[1:3]));
    catch
        askj = 1;
    end
    % % %     DistAng1(q,2) = accumAng(indRange1Ang(2)) - accumAng(indRange1Ang(1));
    
    %     if q > 1
    if q >= 1   % 20190109
        try
            DistAng1(q,2) = accumAng(indRange1Ang(2)) - accumAng(indRange1Ang(1));
        catch
            dghiu = 1;
        end
    else
        d1 = abs(accumAng(indRange1Ang(2)) - accumAng(indRange1Ang(1)) - 90);
        d2 = abs(accumAng(indRange1Ang(2)) + accumAng(indRange1Ang(1)) - 90);
        if d1 < d2
            DistAng1(q,2) = accumAng(indRange1Ang(2)) - accumAng(indRange1Ang(1));
        else
            DistAng1(q,2) = accumAng(indRange1Ang(2)) + accumAng(indRange1Ang(1));
        end
    end
    
end


angRangeInd = IndRange1(:,1:2);
for i = 1 : size(angRangeInd,1)
    range = angRangeInd(i,:);
    dlt = diff(camPoseMatNew(range(1):range(2),1:3));
    [~, dist] = NormalizeVector(dlt);
    cumDist = cumsum(dist);
    %     dltAng = diff(roboPoseInfo(range(1):range(2),14));
    %     [~, distAng] = NormalizeVector(dltAng);
    cumDistAng = accumAng(range(2),1) - accumAng(range(1),1);   %cumsum(distAng);
    DistAng(i,1) = cumDist(end);
    DistAng(i,2) = cumDistAng;
end
% % angRangeInd = IndRange1(:,3:4);



IndRange2 = [];
if 0
    for q = 1 : length(indexCellRoboNew2)
        
        if q == 5
            askh = 1;
        end
        indRange2= indexCellRoboNew2{q,1}([1 end]);
        indRange2 = [indRange2(1) + marginInd indRange2(2) - marginInd];
        if q == 1
            indRange2 = [1 indRange2(2)];
        end
        IndRange2 = [IndRange2; indRange2];
        %     DistAng2(q,1) = norm(camPoseMatNew(indRange2(1),[10:12]) - camPoseMatNew(indRange2(2),[10:12]));
        try
            DistAng2(q,1) = norm(camPoseMatNew(indRange2(1),[1:3]) - camPoseMatNew(indRange2(2),[1:3]));
        catch
            qwk = 1;
        end
        try
            DistAng2(q,2) = accumAng(indRange2(2)) - accumAng(indRange2(1));
        catch
            awkj = 1;
        end
    end
else
    for j = 1 : size(IndRange1,1) + 1
        
        if j == 1
            indRange2Ang = [1 IndRange1(j,1)];
            indRange2Dist = [1 IndRange1(j,3)];
        elseif j == size(IndRange1,1) + 1
            indRange2Ang = [IndRange1(j-1,2) - 0, length(accumAng)];
            indRange2Dist = [IndRange1(j-1,4) - 0, length(accumAng)];
            %             indRange2 = [IndRange1(j-1,2) - marginInd, length(accumAng)];
        else
            indRange2Ang = [IndRange1(j-1,2) - 0 IndRange1(j,1)];
            indRange2Dist = [IndRange1(j-1,4) - 0 IndRange1(j,3)];
            %             indRange2 = [IndRange1(j-1,2) - marginInd IndRange1(j,1)];
        end
        if diff(indRange2Ang) < 0
            asdkb = 1;
        end
        IndRange2 = [IndRange2; [indRange2Ang indRange2Dist]];
        try
            DistAng2(j,1) = norm(camPoseMatNew(indRange2Dist(1),[1:3]) - camPoseMatNew(indRange2Dist(2),[1:3]));
        catch
            qwk = 1;
        end
        try
            DistAng2(j,2) = accumAng(indRange2Ang(2)) - accumAng(indRange2Ang(1));
        catch
            awkj = 1;
        end
        
        
        
    end
    
    indxAng = IndRange2(:,1:2);indxAng = indxAng(:);
    indxAng = unique(indxAng);
    
    indxDist = IndRange2(:,3:4);indxDist = indxDist(:);
    indxDist = unique(indxDist);
    
    % %     idxAll3 = zeros(1,length(dltDistRoboFiltTempNew) + 1);
    % %     idxAll4 = zeros(1,length(dltDistRoboFiltTempNew) + 1);
    % %     idxAll5 = idxAll4;
    % %     idxAll3(indx) = max(abs(accumAng(:,1)));
    % %     idxAll4(indx) = max(abs(camPoseMatNew(:,1)));
    % %     idxAll5(indx) = max(abs(camPoseMatNew(:,3)));
    
    if draw == 1
        try
            figure(101),subplot(3,1,2);hold on;plot(indxAng(1:end-1), dltDistRoboTempNew(indxAng(1:end-1)),'or');plot(indxDist(1:end-1), dltDistRoboTempNew(indxDist(1:end-1)),'*g'); legend('curve','ang','dist');
        catch
            asdvku = 1;
        end
        figure,subplot(3,1,1);plot(accumAng,'-');title('ang');hold on;plot(indxAng, accumAng(indxAng),'or');plot(indxDist, accumAng(indxDist),'*g'); legend('curve','ang','dist');%plot(idxAll3,'r');
        subplot(3,1,2);plot(camPoseMatNew(:,1));title('x');hold on;plot(indxAng, camPoseMatNew(indxAng,1),'or');plot(indxDist, camPoseMatNew(indxDist,1),'*g');  legend('curve','ang','dist');%plot(idxAll4,'r');
        subplot(3,1,3);plot(camPoseMatNew(:,3));title('z');hold on;plot(indxAng, camPoseMatNew(indxAng,3),'or'); plot(indxDist, camPoseMatNew(indxDist,3),'*g');legend('curve','ang','dist');%plot(idxAll5,'r');
        figure,plot(camPoseMatNew(:,1),camPoseMatNew(:,3),'-');hold on;plot(camPoseMatNew(indxAng,1),camPoseMatNew(indxAng,3),'or');plot(camPoseMatNew(indxDist,1),camPoseMatNew(indxDist,3),'*g');axis equal;legend('curve','ang','dist');
    end
    
    if draw == 1;
        figure(31),clf;
    end
    for i = 2 : size(IndRange2,1)
        
        DistAngList = [camPoseMatNew(IndRange2(i,1):IndRange2(i,2),:) accumAng(IndRange2(i,1):IndRange2(i,2))];
        
        if isempty(DistAngList)
            DistAngList = [camPoseMatNew(IndRange2(i,1):-1:IndRange2(i,2),:) accumAng(IndRange2(i,1):-1:IndRange2(i,2))];
        end
        % %         DistAngList(:,end) = DistAngList(:,end) - DistAngList(1,end);
        try
            rtBase = [roty(-DistAngList(1,end)) DistAngList(1,1:3)';0 0 0 1];
        catch
            klasv = 1;
        end
        for ii = 1 : size(DistAngList,1)
            %             coord =  rtBase*inv([roty(DistAngList(ii,end)) DistAngList(ii,1:3)';0 0 0 1]);%*[DistAngList(ii,1:3) 1]';
            %              coord =  [roty(-DistAngList(ii,end)) DistAngList(ii,1:3)';0 0 0 1]*inv(rtBase);   % *inv([roty(-DistAngList(ii,end)) DistAngList(ii,1:3)';0 0 0 1]);
            coord =  inv(rtBase)*[roty(-DistAngList(ii,end)) DistAngList(ii,1:3)';0 0 0 1];
            distInfo{i-1,1}(ii,:) = [coord(1:3,4)' DistAngList(ii,end)];
            
        end
        if i == 7
            asdvgk = 1;
        end
        distInfo{i-1}(:,end) = distInfo{i-1}(:,end) - distInfo{i-1}(1,end);
        distInfo{i-1} =  distInfo{i-1}( distInfo{i-1}(:,end) < 5,:);
        if max(abs(distInfo{i-1}(:,end))) < 3
            if draw == 1
                figure(31),hold on;plot(distInfo{i-1}(:,3),distInfo{i-1}(:,4));
            end
        end
    end
    
    
    
    
end
end

function [segInfo, Mini, DistAng] = segPath(roboPoseInfo, angVeloThr, linearVeloThr, window3,minpeakdistance,marg,draw)

[indexCellRobo, indexRobo] = splitIndex2(find(abs(diff(roboPoseInfo(:,14))./diff(roboPoseInfo(:,13))) < 1)); % 0.2
tmp = medfilt1(abs(diff(roboPoseInfo(:,14))./diff(roboPoseInfo(:,13))),2*window3-1); %diff(roboPoseInfo(:,14))./diff(roboPoseInfo(:,13));
[~, dltvelo] = NormalizeVector(diff(roboPoseInfo(:,10:12)));
tmpVelo = medfilt1(abs(dltvelo./diff(roboPoseInfo(:,13))),2*window3-1); %diff(roboPoseInfo(:,14))./diff(roboPoseInfo(:,13));
[minv,mini11]=findpeaks(-tmp,'minpeakdistance',minpeakdistance);
[minv,mini22]=findpeaks(-tmpVelo,'minpeakdistance',minpeakdistance);
maxTmp = max(tmpVelo,tmp);
[minv,mini33]=findpeaks(-maxTmp,'minpeakdistance',minpeakdistance);
mini1 = unique([mini11;mini22]);
mini1 = mini33;
mini2 = find(tmp(mini1) < angVeloThr & tmpVelo(mini1) < linearVeloThr);
%     mini22 = find(tmp < 5 & tmpVelo < 10);
mini = mini1(mini2) + marg;


Mini = [1;mini;size(roboPoseInfo,1)] + 0;
dltAngRobo = (diff(roboPoseInfo(Mini,end)));
[~, dltDistRobo] = NormalizeVector(diff(roboPoseInfo(Mini,10:12)));
segInfo = [dltDistRobo dltAngRobo];
if draw
    
    figure,plot([tmp tmpVelo maxTmp]);legend('ang','dist','max');hold on;plot(mini1,maxTmp(mini1),'*r');plot(mini,maxTmp(mini),'*g');
    %     plot(mini,tmpVelo(mini),'*g');
    
    figure,plot(roboPoseInfo(:,end-1),roboPoseInfo(:,end));hold on;plot(roboPoseInfo(Mini,end-1),roboPoseInfo(Mini,end),'*g')
    for u = 1 : length(dltAngRobo)
        text(roboPoseInfo(Mini(u),end-1)+0.1,roboPoseInfo(Mini(u),end)+10,num2str(round(dltAngRobo(u),2)), 'Color',[1 0 0],'FontSize',15);  %,'FontWeight','bold');
    end
    
    
    
    figure,subplot(1,2,1);plot(roboPoseInfo(:,end-1),roboPoseInfo(:,10));hold on;plot(roboPoseInfo(Mini,end-1),roboPoseInfo(Mini,10),'*g');title('x');
    if sum(roboPoseInfo(:,12)) == 0
        subplot(1,2,2);plot(roboPoseInfo(:,end-1),roboPoseInfo(:,11));hold on;plot(roboPoseInfo(Mini,end-1),roboPoseInfo(Mini,11),'*g');title('z');
    else
        subplot(1,2,2);plot(roboPoseInfo(:,end-1),roboPoseInfo(:,12));hold on;plot(roboPoseInfo(Mini,end-1),roboPoseInfo(Mini,12),'*g');title('z');
    end
    % %     figure,hold on;plot([dltAngRobo(1);dltAngRobo;dltAngRobo(end)])
    % % figure,
end

for i = 1 : length(Mini)-1
    range = [Mini(i) Mini(i+1)];
    dlt = diff(roboPoseInfo(range(1):range(2),10:12));
    [~, dist] = NormalizeVector(dlt);
    cumDist = cumsum(dist);
    %     dltAng = diff(roboPoseInfo(range(1):range(2),14));
    %     [~, distAng] = NormalizeVector(dltAng);
    cumDistAng = roboPoseInfo(range(2),14) - roboPoseInfo(range(1),14);   %cumsum(distAng);
    DistAng(i,1) = cumDist(end);
    DistAng(i,2) = cumDistAng;
end


end


