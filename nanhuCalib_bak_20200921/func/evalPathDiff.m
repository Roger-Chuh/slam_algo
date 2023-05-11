function evalPathDiff(roboData, ceilingData)
dltTimeThr = 1; 0.5;1;2;10;10;1;20;1; 0.6; 1; % 1 sec
if roboData(end,end) < 0
    roboData(:,end) = -roboData(:,end);
end

if ceilingData(end,end) < 0
    ceilingData(:,end) = -ceilingData(:,end);
end

cnt = 1;
Cnt = [];
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


figure,subplot(1,3,1);plot(roboData(Cnt,4),xerr); title('x err');% hold on;plot(roboData(:,4),roboData(:,end));
% % subplot(2,3,4);plot(roboData(Cnt,4),roboData(Cnt,1) - xerr);hold on;plot(ceilingData(:,4),ceilingData(:,1));
% % legend('wheel - x err','ceiling');
subplot(1,3,2);plot(roboData(Cnt,4),zerr); title('z err');% hold on;plot(roboData(:,4),roboData(:,end));
% % subplot(2,3,5);plot(roboData(Cnt,4),roboData(Cnt,3) - zerr);hold on;plot(ceilingData(:,4),ceilingData(:,3));
% % legend('wheel - z err','ceiling');
subplot(1,3,3);plot(roboData(Cnt(angerr>-inf),4),angerr(angerr>-inf)); title('ang err');% hold on;plot(roboData(:,4),roboData(:,end));
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