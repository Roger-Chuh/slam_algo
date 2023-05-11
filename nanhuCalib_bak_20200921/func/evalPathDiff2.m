function [timeShift,maxTime, maxTime2] = evalPathDiff2(roboData, ceilingData,IndRangeStraight, IndRangeRound,indAlignRange)
% % close all
dltTimeThr = 1; 0.5;1;2;10;10;1;20;1; 0.6; 1; % 1 sec
% % IndRangeRound = indAlignRange;
window = 0;

useSeg = 0;

if iscell(indAlignRange)
    twoSeg = 1;
else
    twoSeg = 0;
end

if 0 %roboData(end,end) < 0
    roboData(:,end) = -roboData(:,end);
end

if 0 %ceilingData(end,end) < 0
    ceilingData(:,end) = -ceilingData(:,end);
end

if 0
    figure,plot(ceilingData(:,4),ceilingData(:,5));hold on;plot(roboData(:,4),roboData(:,5))
end


timeShift0 = [1 0.1];
% % ErrorTmp0 = errFunc(IndRangeStraight,IndRangeRound,roboData,ceilingData,1,1,indAlignRange,timeShift0);
ErrorTmp0 = errFunc([],[],roboData,ceilingData,1,0,indAlignRange,timeShift0);
Error0 = ErrorTmp0(1);
% % % Error10 = reshape(ErrorTmp0(2:end),[],4);
options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','off','MaxFunEvals',10000,'TolX',1e-8);
if useSeg == 1
    if 0
        for i = 1:(size(IndRangeStraight,1) + size(IndRangeRound,1))
            if i <= size(IndRangeStraight,1)
                [timeShift,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(U) errFunc(IndRangeStraight(i,:),[],roboData,ceilingData,1,0, indAlignRange,U),[timeShift0],[],[],options);
            else
                [timeShift,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(U) errFunc([],IndRangeRound(i-size(IndRangeStraight,1),:),roboData,ceilingData,1,0,indAlignRange, U),[timeShift0],[],[],options);
            end
            TimeShift(i,1) = timeShift;
        end
    else
        ci = 1;iNd = [];
        for i = 1:(size(indAlignRange,1) )
            % % %         if i <= size(IndRangeStraight,1)
            % %             [timeShift,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(U) errFunc([],[],roboData(indAlignRange(i,1):indAlignRange(i,2),:),ceilingData,1,0, indAlignRange(i,:),U),[timeShift0],[],[],options);
            try
                [timeShift,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(U) errFunc([],[],roboData,ceilingData,1,0, indAlignRange(i,:),U),[timeShift0],[],[],options);
                % %         else
                % %             [timeShift,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(U) errFunc([],IndRangeRound(i-size(IndRangeStraight,1),:),roboData,ceilingData,1,0,indAlignRange, U),[timeShift0],[],[],options);
                % %         end
                TimeShift(ci,1) = timeShift;
                ci = ci+1;
                iNd = [iNd;i];
            catch
                asjovk = 1;
            end
        end
        indAlignRange = indAlignRange(iNd,:);
    end
elseif 0
    % %     [timeShift,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(U) errFunc(IndRangeStraight,IndRangeRound,roboData,ceilingData,1,0,indAlignRange,U),[timeShift0],[],[],options);
    [timeShift,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(U) errFunc([],indAlignRange,roboData,ceilingData,1,0,indAlignRange,U),[timeShift0],[],[],options);
    TimeShift = timeShift;
elseif 0
    
    [timeShift,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(U) errFunc([],[],roboData,ceilingData,1,0, indAlignRange,U),[timeShift0],[],[],options);
    TimeShift = timeShift;
else
    kbjhg = 1;
    [timeShift,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(U) errFunc([],[],roboData,ceilingData,1,0, indAlignRange,U),[[timeShift0]],[],[],options);
    TimeShift = timeShift;
end

asja = 1;Err = [];
if useSeg == 0
    % % %     sizeN = size(IndRangeStraight,1);
    sizeN = size(indAlignRange,1);
else
    sizeN = size(indAlignRange,1);
end
for i = 1 : sizeN
    errFlag = [1 2];
    errFlag = [1 2 3];
    errFlag = [3];
    if i == 22;
        sakdj = 1;
    end
    if useSeg == 1
        if 0
            [Err1] = evalDiff(errFlag,roboData,ceilingData(IndRangeStraight(i,1):IndRangeStraight(i,2),:),2,0,indAlignRange,TimeShift(i));
        else
            if twoSeg == 0
                [Err1] = evalDiff(errFlag,roboData(indAlignRange(i,1):indAlignRange(i,2),:),ceilingData,2,0,indAlignRange,TimeShift(i));
            else
                [Err1] = evalDiff(errFlag,roboData(indAlignRange{i,1},:),ceilingData,2,0,indAlignRange,TimeShift(i));
            end
        end
    elseif 0
        [Err1] = evalDiff(errFlag,roboData,ceilingData(IndRangeStraight(i,1):IndRangeStraight(i,2),:),2,0,indAlignRange,TimeShift);
    elseif 0
        [Err1] = evalDiff(errFlag,roboData,ceilingData,2,1,indAlignRange,TimeShift);
    else
        if twoSeg == 0
            [Err1] = evalDiff(errFlag,roboData(indAlignRange(i,1):indAlignRange(i,2),:),ceilingData,2,0,indAlignRange,TimeShift);
        else
            try
                [Err1] = evalDiff(errFlag,roboData(indAlignRange{i,1},:),ceilingData,2,0,indAlignRange,TimeShift);
            catch
                esihk = 1;
            end
        end
    end
    Error(i,1) = Err1(1);
    if length(Err1) > 1
        Err = [Err;reshape(Err1(2:end),[],4)];
    end
end
Error = Error(~isnan(Error),:);
Err = Err(~isnan(Err(:,1)),:);

Error1 = Error;
% % % % % % % % % % % % % % % % % % for i = 1 : size(IndRangeRound,1)
% % % % % % % % % % % % % % % % % %     errFlag = 3;
% % % % % % % % % % % % % % % % % % % %     errFlag = [1 2 3];
% % % % % % % % % % % % % % % % % %     if useSeg == 1
% % % % % % % % % % % % % % % % % %         [Err1] = evalDiff(errFlag,roboData,ceilingData(IndRangeRound(i,1):IndRangeRound(i,2),:),2,1,indAlignRange,TimeShift(i + size(IndRangeStraight,1)));
% % % % % % % % % % % % % % % % % %     elseif 0
% % % % % % % % % % % % % % % % % %         [Err1] = evalDiff(errFlag,roboData,ceilingData(IndRangeRound(i,1):IndRangeRound(i,2),:),2,1,indAlignRange,TimeShift);
% % % % % % % % % % % % % % % % % %         fhjk = 1;
% % % % % % % % % % % % % % % % % %     else
% % % % % % % % % % % % % % % % % %         [Err1] = evalDiff(errFlag,roboData,ceilingData,2,1,indAlignRange,TimeShift);
% % % % % % % % % % % % % % % % % %     end
% % % % % % % % % % % % % % % % % %     Error(size(Error1,1) + i,1) = Err1(1);
% % % % % % % % % % % % % % % % % %     if length(Err1) > 1
% % % % % % % % % % % % % % % % % %         Err = [Err;reshape(Err1(2:end),[],4)];
% % % % % % % % % % % % % % % % % %     end
% % % % % % % % % % % % % % % % % % end
% % % % % % % % % % % % % % % % % % Error = Error(~isnan(Error),:);
% % % % % % % % % % % % % % % % % % Err = Err(~isnan(Err(:,1)),:);
% % % % % % % % % % % % % % % % % % wdvhk = 1;
Err1 = Err;
[~,id1] = sort(Err1(:,4),'ascend');
Err = Err1(id1,:);




% % % figure,subplot(1,3,1);plot(Err(:,4),Err(:,1));title('x error');
% % % subplot(1,3,2);plot(Err(:,4),Err(:,2));title('z error');
% % % subplot(1,3,3);plot(Err(:,4),Err(:,3));title('angle error');
maxTime = max(Err(:,4));
% % timeShift = 0;
if useSeg == 1
    timeShift = mean(TimeShift);
    ceilingData(:,end-1) = ceilingData(:,end-1) + repmat(timeShift,size(ceilingData,1),1);
else
    ceilingData(:,end-1) = TimeShift(1).*ceilingData(:,end-1) + repmat(TimeShift(2),size(ceilingData,1),1);
end
% % % timeShift = 0;

cnt = 1;
Cnt = [];
for i = 1 : size(roboData,1)
    
    if i == 570
        wqeoih = 1;
    end
    
    x = roboData(i,1);
    z = roboData(i,3);
    timeStamp = roboData(i,4);
    if timeStamp >= 135.1 %1327.5
        qafiu = 1;
    end
    ang = roboData(i,5);
    largeId = find(ceilingData(:,4) > timeStamp);
    smallId = find(ceilingData(:,4) < timeStamp);
    equalId = find(ceilingData(:,4) == timeStamp);
    if isempty(equalId) % && ~isempty(smallId) && ~isempty(largeId)
        if isempty(equalId)
            try
                range = [smallId(end) largeId(1)];
            catch
                skv = 1;
            end
        end
        if isempty(largeId)
            range = [smallId(end) equalId];
        end
        if isempty(smallId)
            range = [equalId largeId(1)];
        end
    else
        try
            dlt1 = abs(ceilingData(largeId(1),4) - timeStamp);
            dlt2 = abs(ceilingData(smallId(end),4) - timeStamp);
            if dlt1 < dlt2
                range = [equalId largeId(1)];
            else
                range = [smallId(end) equalId];
            end
        catch
            if isempty(largeId)
                range = [smallId(end) equalId];
            end
            if isempty(smallId)
                range = [equalId largeId(1)];
            end
        end
    end
    if length(range) < 2
        continue;
    end
    try
        dltTime = ceilingData(range(2),4) - ceilingData(range(1),4);
    catch
        asgkj = 1;
    end
    if dltTime > dltTimeThr
        continue;
    end
    range = [max(range(1) - window,1) min(range(2) + window,size(ceilingData,1))];
    
    ratio = (timeStamp - ceilingData(range(1),4))/(ceilingData(range(2),4) - ceilingData(range(1),4));
    % %     ratio = 1-ratio;
    if 0  % ratio == 0
        xCeilTemp = ceilingData(range(2),1);
        zCeilTemp = ceilingData(range(2),3);
        angCeilTemp = ceilingData(range(2),5);
    else
        xCeilTemp = (1 - ratio)*ceilingData(range(1),1) + ratio*ceilingData(range(2),1);
        zCeilTemp = (1 - ratio)*ceilingData(range(1),3) + ratio*ceilingData(range(2),3);
        angCeilTemp = (1-ratio)*ceilingData(range(1),5) + (ratio)*ceilingData(range(2),5);
    end
    if i == 217
        asfkdj = 1;
    end
    XCeilTemp(cnt,:) = xCeilTemp;
    ZCeilTemp(cnt,:) = zCeilTemp;
    AngCeilTemp(cnt,:) = angCeilTemp;
    xerr(cnt,:) = ((x - xCeilTemp));  % /xCeilTemp);
    zerr(cnt,:) = ((z - zCeilTemp));  % /xCeilTemp);
    tss(cnt,1) = timeStamp;
    if abs(ang - angCeilTemp) > 40
        sdvkj = 1;
    end
    
    if (ang - angCeilTemp) < -4.5
        sdvkj = 1;
    end
    
    angerr(cnt,:) = ((ang - angCeilTemp));  % /xCeilTemp);
    
    if 0
        [ceilingData(range(1),4) ceilingData(range(2),4)]
        [ceilingData(range(1),4) ceilingData(range(2),4) timeStamp]
        [ceilingData(range(1),5) ceilingData(range(2),5) ang]
    end
    
    
    Cnt = [Cnt; cnt];
    cnt = cnt + 1;
end

if 0
    figure,subplot(1,3,1);plot(tss,xerr); title('x err');% hold on;plot(roboData(:,4),roboData(:,end));
    % % subplot(2,3,4);plot(roboData(Cnt,4),roboData(Cnt,1) - xerr);hold on;plot(ceilingData(:,4),ceilingData(:,1));
    % % legend('wheel - x err','ceiling');
    subplot(1,3,2);plot(tss,zerr); title('z err');% hold on;plot(roboData(:,4),roboData(:,end));
    % % subplot(2,3,5);plot(roboData(Cnt,4),roboData(Cnt,3) - zerr);hold on;plot(ceilingData(:,4),ceilingData(:,3));
    % % legend('wheel - z err','ceiling');
    subplot(1,3,3);plot(tss(Cnt(angerr>-inf),:),angerr(angerr>-inf)); title('ang err');% hold on;plot(roboData(:,4),roboData(:,end));
    % % subplot(2,3,6);plot(roboData(Cnt,4),roboData(Cnt,5) - angerr);hold on;plot(ceilingData(:,4),ceilingData(:,5));
    % % legend('wheel - ang err','ceiling');
    
    % % %
    % % % figure,subplot(2,3,1);plot(roboData(Cnt,4),xerr); title('x err');% hold on;plot(roboData(:,4),roboData(:,end));
    % % % subplot(2,3,4);plot(roboData(Cnt,4),roboData(Cnt,1) - xerr);hold on;plot(ceilingData(:,4),ceilingData(:,1));
    % % % legend('wheel - x err','ceiling');
    % % % subplot(2,3,2);plot(roboData(Cnt,4),zerr); title('z err');% hold on;plot(roboData(:,4),roboData(:,end));
    % % % subplot(2,3,5);plot(roboData(Cnt,4),roboData(Cnt,3) - zerr);hold on;plot(ceilingData(:,4),ceilingData(:,3));
    % % % legend('wheel - z err','ceiling');
    % % % subplot(2,3,3);plot(roboData(Cnt,4),angerr); title('ang err');% hold on;plot(roboData(:,4),roboData(:,end));
    % % % subplot(2,3,6);plot(roboData(Cnt,4),roboData(Cnt,5) - angerr);hold on;plot(ceilingData(:,4),ceilingData(:,5));
    % % % legend('wheel - ang err','ceiling');
end


%%
for gg = 1 : size(IndRangeRound,1)
    segTime(gg,1) = ceilingData(IndRangeRound(gg,2),end-1);
end

dltSegTime = abs(maxTime - segTime);

[id1,id2] = min(dltSegTime);

maxTime2 = segTime(id2) + 0.25;
%%

Err2 = Err(Err(:,end) <= maxTime2,:);

figure,subplot(1,3,1);plot(Err2(:,4),Err2(:,1));title('x error');
subplot(1,3,2);plot(Err2(:,4),Err2(:,2));title('z error');
subplot(1,3,3);plot(Err2(:,4),Err2(:,3));title('angle error');



end

function Error = errFunc(IndRangeStraight,IndRangeRound,roboData,ceilingData,outNum,draw,indAlignRange,timeShift)
if ~isempty(IndRangeStraight)
    for i = 1 : size(IndRangeStraight,1)
        errFlag = [1 2];
        err = evalDiff(errFlag,roboData,ceilingData(IndRangeStraight(i,1):IndRangeStraight(i,2),:),outNum,draw,indAlignRange,timeShift);
        Error(i,1) = err(1);
        
    end
    Error = Error(~isnan(Error),:);
    Error1 = Error;
else
    Error1 = [];
end
if ~isempty(IndRangeRound)
    for i = 1 : size(IndRangeRound,1)
        errFlag = 3;
        err = evalDiff(errFlag,roboData,ceilingData(IndRangeRound(i,1):IndRangeRound(i,2),:),outNum,draw,indAlignRange,timeShift);
        try
            Error(size(Error1,1) + i,1) = err(1);
        catch
            wh = 1;
        end
    end
    Error = Error(~isnan(Error),:);
else
    Error1 = [];
    
end
if isempty(IndRangeStraight) && isempty(IndRangeRound)
    errFlag = [3];
    for i = 1 : size(indAlignRange,1)
        try
            if ~iscell(indAlignRange)
                err = evalDiff(errFlag,roboData(indAlignRange(i,1):indAlignRange(i,2),:),ceilingData,outNum,draw,indAlignRange,timeShift);
            else
                err = evalDiff(errFlag,roboData(indAlignRange{i,1},:),ceilingData,outNum,draw,indAlignRange,timeShift);
            end
        catch
            asdgvijbk = 1;
        end
        try
            Error(size(Error1,1) + i,1) = err(1);
        catch
            wjh = 1;
        end
    end
    Error = Error(~isnan(Error),:);
end

end

function [Error] = evalDiff(errFlag,roboData1,ceilingData,outNum,draw,indAlignRange,timeShift)

N = 1;
window = 0;
thr0 = [20 3];
thr0 = [2 1];
if 0
    if 1
        if length(errFlag) == 2
            [~,dltLen1] = NormalizeVector(ceilingData(2:end,1:3) - ceilingData(1:end-1,1:3));
            thr = thr0(1);
        else
            [~,dltLen1] = NormalizeVector(ceilingData(2:end,end) - ceilingData(1:end-1,end));
            thr = thr0(2);
        end
        dltLen = medfilt1(dltLen1,(2*5 - 1));
        
        indd = find(dltLen > thr);
        [indexCell1,idx1] = splitIndex2(indd);
        
        for y = 1 : length(indexCell1)
            inSize(y,1) = length(indexCell1{y});
            
        end
        [~,idm] = max(inSize);
        inddd = indexCell1(idm);
        % % %     indd = inddd{1};
        try
            indd = [inddd{1} inddd{1}(end) + 1];
        catch
            asvk = 1;
        end
        dlen = min(dltLen(inddd{1}));
        try
            ceilingData = ceilingData([indd(N):indd(end-N + 1)],:);
        catch
            asdgvkh = 1;
        end
    end
    % % draw = 0;
    
    
    timeRange1 = [min(ceilingData(:,4)) max(ceilingData(:,4))];
    timeRange = [max(0,timeRange1(1)-1.5)  min(timeRange1(2)+1.5,roboData1(end,4))];
    % % roboData = roboData1(indd{1},:);
    
    
    
    roboData = roboData1(roboData1(:,4) >= timeRange(1) & roboData1(:,4) <= timeRange(2),:);
    
    roboData2 = roboData;
    inddRobo = findChange(roboData2,thr0,errFlag);
    roboData = roboData2(inddRobo,:);
elseif 0
    indAlig = [];
    for e = 1 : size(indAlignRange,1)
        if size(indAlignRange,2) == 2
            indAlig = [indAlig;[indAlignRange(e,1):indAlignRange(e,2)]'];
        else
            indAlig = [indAlig;[indAlignRange(e,:)]'];
        end
    end
    roboData = roboData1(unique(indAlig),:);
else
    
    timeRange1 = [min(roboData1(:,4)) max(roboData1(:,4))];
    timeRange = [max(0,timeRange1(1) - 1)  min(timeRange1(2) + 1,ceilingData(end,4))];
    ceilingData11 = ceilingData;
    ceilingData = ceilingData(ceilingData(:,4) >= timeRange(1) & ceilingData(:,4) <= timeRange(2),:);
    roboData = roboData1;
    
end

Range = [];


if draw == 1
    
    try
        index = find(roboData(:,4) > ceilingData(1,4)-10 & roboData(:,4) < ceilingData(end,4)+10);
    catch
        ewgoh = 1;
    end
    figure,subplot(3,1,1);plot(roboData(index,4),roboData(index,1),'x-');hold on;plot(ceilingData(:,4),ceilingData(:,1),'.');
    subplot(3,1,2);plot(roboData(index,4),roboData(index,3),'x-');hold on;plot(ceilingData(:,4),ceilingData(:,3),'.');
    subplot(3,1,3);plot(roboData(index,4),roboData(index,5),'x-');hold on;plot(ceilingData(:,4),ceilingData(:,5),'.');
end
ceilingData(:,end-1) = timeShift(1).*ceilingData(:,end-1) + repmat(timeShift(2),size(ceilingData,1),1);
ceilingData1 = ceilingData;

% ceilingData1(:,1) = interp1(ceilingData1(:,4),ceilingData1(:,1),roboData(:,4));
% ceilingData1(:,3) = interp1(ceilingData1(:,4),ceilingData1(:,3),roboData(:,4));
% ceilingData1(:,5) = interp1(ceilingData1(:,4),ceilingData1(:,5),roboData(:,4));


if draw == 1
    subplot(3,1,1);plot(ceilingData(:,4),ceilingData(:,1),'.');
    subplot(3,1,2);plot(ceilingData(:,4),ceilingData(:,3),'.');
    subplot(3,1,3);plot(ceilingData(:,4),ceilingData(:,5),'.');
    svdjk = 1;
end
dltTimeThr = 10;
dltAngThr = 10;20;
dltDispThr = 4000;
cnt = 1;
Cnt = [];
tsStack = [];
for i = 1 : size(roboData,1)
    
    if i == 570
        wqeoih = 1;
    end
    
    x = roboData(i,1);
    z = roboData(i,3);
    timeStamp = roboData(i,4);
    if timeStamp > 1327.5
        qafiu = 1;
    end
    ang = roboData(i,5);
    largeId = find(ceilingData(:,4) > timeStamp);
    smallId = find(ceilingData(:,4) < timeStamp);
    equalId = find(ceilingData(:,4) == timeStamp);
    if isempty(largeId) + isempty(smallId) + isempty(equalId) >=2
        continue;
    else
        if isempty(equalId) % && ~isempty(smallId) && ~isempty(largeId)
            if isempty(equalId)
                try
                    range = [smallId(end) largeId(1)];
                catch
                    skv = 1;
                end
            end
            if isempty(largeId)
                range = [smallId(end) equalId];
            end
            if isempty(smallId)
                range = [equalId largeId(1)];
            end
        else
            try
                dlt1 = abs(ceilingData(largeId(1),4) - timeStamp);
                dlt2 = abs(ceilingData(smallId(end),4) - timeStamp);
                if dlt1 < dlt2
                    range = [equalId largeId(1)];
                else
                    range = [smallId(end) equalId];
                end
            catch
                if isempty(largeId)
                    range = [smallId(end) equalId];
                end
                if isempty(smallId)
                    range = [equalId largeId(1)];
                end
            end
        end
        if length(range) < 2
            continue;
        end
        try
            dltTime = ceilingData(range(2),4) - ceilingData(range(1),4);
            dltAngle =  abs(ceilingData(range(2),5) - ceilingData(range(1),5));
            dltDisp =  norm(ceilingData(range(2),[1 3]) - ceilingData(range(1),[1 3]));
        catch
            asgkj = 1;
        end
        if dltAngle > dltAngThr
            continue;
        end
        % % % % % %         if dltAngle/dltTime > dltAngThr
        % % % % % %             continue;
        % % % % % %         end
        
        % % % % % % % % % % % % % %         if dltDisp/dltTime > dltDispThr
        % % % % % % % % % % % % % %             continue;
        % % % % % % % % % % % % % %         end
        if dltTime > dltTimeThr
            continue;
        end
        
        range = [max(range(1) - window,1) min(range(2) + window,size(ceilingData,1))];
        Range = [Range;range];
        tsStack = [tsStack;timeStamp];
        ratio = (timeStamp - ceilingData(range(1),4))/(ceilingData(range(2),4) - ceilingData(range(1),4));
        % %     ratio = 1-ratio;
        if 0  % ratio == 0
            xCeilTemp = ceilingData(range(2),1);
            zCeilTemp = ceilingData(range(2),3);
            angCeilTemp = ceilingData(range(2),5);
        else
            xCeilTemp = (1 - ratio)*ceilingData(range(1),1) + ratio*ceilingData(range(2),1);
            zCeilTemp = (1 - ratio)*ceilingData(range(1),3) + ratio*ceilingData(range(2),3);
            angCeilTemp = (1-ratio)*ceilingData(range(1),5) + (ratio)*ceilingData(range(2),5);
        end
        if i == 217
            asfkdj = 1;
        end
        XCeilTemp(cnt,:) = xCeilTemp;
        ZCeilTemp(cnt,:) = zCeilTemp;
        AngCeilTemp(cnt,:) = angCeilTemp;
        xerr(cnt,:) = ((x - xCeilTemp));  % /xCeilTemp);
        zerr(cnt,:) = ((z - zCeilTemp));  % /xCeilTemp);
        ts(cnt,:) = timeStamp;
        if abs(ang - angCeilTemp) > 40
            sdvkj = 1;
        end
        
        if (ang - angCeilTemp) < -4.5
            sdvkj = 1;
        end
        if abs(zerr(cnt,:)) > 10
            awefjk = 1;
        end
        
        angerr(cnt,:) = ((ang - angCeilTemp));  % /xCeilTemp);
        
        if cnt == 15
            asfkj = 1;
        end
        if 0
            [ceilingData(range(1),4) ceilingData(range(2),4)]
            [ceilingData(range(1),4) ceilingData(range(2),4) timeStamp]
            [ceilingData(range(1),5) ceilingData(range(2),5) ang]
        end
        
        
        Cnt = [Cnt; cnt];
        cnt = cnt + 1;
    end
end
if 1
    try
        ind = [find(roboData(:,4) == min(tsStack)) find(roboData(:,4) == max(tsStack))];
        ind = find(ismember(roboData(:,4),tsStack));
        err = [xerr zerr angerr];
        Err = err(:,errFlag);
        Error = var(Err);   %mean(abs(Err(:)));
    catch
        Error = nan;
        qvkhj = 1;
    end
    if outNum == 2
        try
            tmp = [err ts];
        catch
            
            tmp = [];
        end
        Error = [Error; tmp(:)];
    end
else
    roboData1 = roboData;
    try
        ind = [find(roboData1(:,4) == min(tsStack)) find(roboData1(:,4) == max(tsStack))];
        % % ind1 = find(roboData1(:,4) >= min(ceilingData1(:,4)));
        
        % % % % catch
        % % % %     awkjv = 1;
        % % % % end
        ceilingData1 = zeros(size(roboData1(ind(1):ind(2),4),1),5);
        ceilingData1(:,4) = roboData1(ind(1):ind(2),4);
        ceilingData1(:,1) = interp1(ceilingData(:,4),ceilingData(:,1),roboData1(ind(1):ind(2),4));
        ceilingData1(:,3) = interp1(ceilingData(:,4),ceilingData(:,3),roboData1(ind(1):ind(2),4));
        ceilingData1(:,5) = interp1(ceilingData(:,4),ceilingData(:,5),roboData1(ind(1):ind(2),4));
        
        % % % % try
        err = roboData1(ind(1):ind(2),[1 3 5]) - ceilingData1(:,[1 3 5]);
        
        
        % % %     err = [xerr zerr angerr];
        Err = err(:,errFlag);
        Error = mean(abs(Err(:)));
    catch
        Error = nan;
        qvkhj = 1;
    end
    if outNum == 2
        try
            tmp = [err ts];
        catch
            
            tmp = [];
        end
        Error = [Error; tmp(:)];
    end
end


% % ceilingData1(min(Range(:)):max(Range(:)),1) = interp1(roboData1(:,4),roboData1(:,1),roboData1(min(Range(:)):max(Range(:)),4));
% % ceilingData1(min(Range(:)):max(Range(:)),3) = interp1(roboData1(:,4),roboData1(:,3),roboData1(min(Range(:)):max(Range(:)),4));
% % ceilingData1(min(Range(:)):max(Range(:)),5) = interp1(roboData1(:,4),roboData1(:,5),roboData1(min(Range(:)):max(Range(:)),4));


if draw == 1
    % %     figure,plot(ceilingData(:,4),ceilingData(:,3));hold on;plot(ceilingData1(:,4),ceilingData1(:,3)); plot(roboData1(ind(1):ind(2),4),roboData1(ind(1):ind(2),3));
    
    
    dltX = abs(ceilingData(1,1) - ceilingData(end,1));
    dltZ = abs(ceilingData(1,3) - ceilingData(end,3));
    dltAng = abs(ceilingData(1,5) - ceilingData(end,5));
    
    if length(errFlag) == 2
        [~,bigId] = max([dltX dltZ dltAng]);
    else
        bigId = 3;
    end
    
    if draw == 1
        figure,
        try
            if bigId == 1
                plot(ceilingData(:,4),ceilingData(:,1),'x-');hold on;plot(roboData(ind,4),XCeilTemp,'x-'); plot(roboData(ind,4),roboData(ind,1),'x-');   legend('ceiling','interpolated ceiling','wheel');title(sprintf('x, %0.3f',dlen));
            elseif bigId == 2
                plot(ceilingData(:,4),ceilingData(:,3),'x-');hold on;plot(roboData(ind,4),ZCeilTemp,'x-'); plot(roboData(ind,4),roboData(ind,3),'x-');   legend('ceiling','interpolated ceiling','wheel');title(sprintf('z, %0.3f',dlen));
            else
                plot(ceilingData(:,4),ceilingData(:,5),'x-');hold on;plot(roboData(ind,4),AngCeilTemp,'x-'); plot(roboData(ind,4),roboData(ind,5),'x-');   legend('ceiling','interpolated ceiling','wheel');%%%% title(sprintf('angle, %0.3f',dlen));
            end
            % %    legend('ceiling','interpolated ceiling','wheel');
            
            figure,subplot(3,1,1);plot(roboData(ind,4),err(:,1));subplot(3,1,2);plot(roboData(ind,4),err(:,2));subplot(3,1,3);plot(roboData(ind,4),err(:,3));
            
            % %             figure,plot([dltLen1 dltLen repmat(thr,length(dltLen),1)]);
            
        catch
            ergk = 1;
        end
    end
end





end

function indd = findChange(ceilingData,thr0,errFlag)
if length(errFlag) == 2
    [~,dltLen1] = NormalizeVector(ceilingData(2:end,1:3) - ceilingData(1:end-1,1:3));
    thr = thr0(1);
else
    [~,dltLen1] = NormalizeVector(ceilingData(2:end,end) - ceilingData(1:end-1,end));
    thr = thr0(2);
end
dltLen = medfilt1(dltLen1,(2*5 - 1));

indd = find(dltLen > thr);
[indexCell1,idx1] = splitIndex2(indd);

for y = 1 : length(indexCell1)
    inSize(y,1) = length(indexCell1{y});
    
end
[~,idm] = max(inSize);
inddd = indexCell1(idm);
indd = [inddd{1} inddd{1}(end) + 1];
end
