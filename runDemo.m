function runDemo(calTagDir, vrDir)


close all
% [ceilPosePoseMatStamp, vrDistAng1, vrDistAng2] = readVidAndPlot(calTagDir, 1, 'D:\Work\caltag-master\caltag-master\GeneratePattern', 'D:\Temp\20180625\calibCalTag1\calib_use', 1);
[ceilPosePoseMatStamp, vrDistAng1, vrDistAng2,posestPoseMapped2,startInd] = readVidAndPlot(calTagDir, 1, 'E:\bk_20180627\pc\Work\caltag-master\caltag-master\GeneratePattern', 'E:\bk_20180627\pc\Temp\20180724\calibFishEye1', 1, 1);
startInd = 1;

% posestPoseMap = PlotRobot2(vrDir,1,calTagDir,ceilPosePoseMatStamp(:,14));
if 0
    posestPoseMap = PlotVR2(vrDir,1,calTagDir);
    vrPosePoseMatStamp = PlotVR3(vrDir,1);
    
    
    invT = PlotRobotVR3(vrDir,calTagDir,1,1);
    posestPoseMap2 = (invT(1:3,1:3)*posestPoseMap' + repmat(invT(1:3,4),1,size(posestPoseMap,1)))';
    % e
    fid1 = fopen(fullfile(vrDir, 'cam_path22.txt'),'w');
    for j = 1 : size(posestPoseMap2,1)
        fprintf(fid1, '%0.2f %0.2f %0.2f', posestPoseMap2(j,1)./1000,posestPoseMap2(j,2)./1000,posestPoseMap2(j,3)./1000);
        fprintf(fid1, '\n');
    end
    fclose(fid1);
    
    
    [T, roboMap, vrMap2, TClient, roboPosePoseMatStamp, slamPosePoseMatStamp] = PlotRobotVR2(vrDir,1,1);
    
    if 1
        vrMap = [vrPosePoseMatStamp(:,10) zeros(size(vrPosePoseMatStamp,1),1) vrPosePoseMatStamp(:,12)];
    else
        vrMap = vrMap2;
    end
    % T = TClient;
    vrMapMapped = (T(1:3,1:3)*vrMap' + repmat(T(1:3,4),1,size(vrMap,1)))';
    posestPoseMapMapped = (T(1:3,1:3)*posestPoseMap2' + repmat(T(1:3,4),1,size(posestPoseMap2,1)))';
else
    posestPoseMap = PlotRobot2(vrDir,1,calTagDir,ceilPosePoseMatStamp(:,14));  %,posestPoseMapped2);
    [T,~,~,roboPosePoseMatStamp] = PlotRobotVR4(vrDir,1,1);
    ceilMap = (T(1:3,1:3)*posestPoseMap' + repmat(T(1:3,end),1,size(posestPoseMap,1)))';
    evalPathDiff([roboPosePoseMatStamp(:,[10:12 14:15]) ], [ceilMap(startInd:end,:) ceilPosePoseMatStamp(:,[14:15])]);
    figure,plot(ceilMap(:,1),ceilMap(:,3),'-b');hold on;plot(roboPosePoseMatStamp(:,10),roboPosePoseMatStamp(:,12),'-r');axis equal;
    [ceilingDistAng1, ceilingDistAng2, ceilingAngRangeInd] = getDistAngInfo(ceilPosePoseMatStamp(:,end), ceilMap(:,1:3),1.5,5,4,3); % ceiling
    [wheelDistAng1, wheelDistAng2,wheelAngRangeInd] = getDistAngInfo(roboPosePoseMatStamp(:,end), roboPosePoseMatStamp(:,10:12), 1,1,1,0); % wheel
    figure,hold on;plot(ceilPosePoseMatStamp(:,14),ceilPosePoseMatStamp(:,end),'-');plot(roboPosePoseMatStamp(:,14),roboPosePoseMatStamp(:,end),'.');title('angle');legend('ceiling','wheel');
    figure,hold on;plot(ceilPosePoseMatStamp(:,14),ceilMap(startInd:end,1),'-');plot(roboPosePoseMatStamp(:,14),roboPosePoseMatStamp(:,10),'.');title('x');legend('ceiling','wheel');
    figure,hold on;plot(ceilPosePoseMatStamp(:,14),ceilMap(startInd:end,3),'-');plot(roboPosePoseMatStamp(:,14),roboPosePoseMatStamp(:,12),'.');title('z');legend('ceiling','wheel');
    
    
end

evalPathDiff([roboPosePoseMatStamp(:,[10:12 14:15]) ], [posestPoseMapMapped ceilPosePoseMatStamp(:,[14:15])]);
evalPathDiff([roboPosePoseMatStamp(:,[10:12 14:15]) ], [vrMapMapped vrPosePoseMatStamp(:,[14:15])]);

evalPathDiff([slamPosePoseMatStamp(:,[10:12 14:15]) ], [posestPoseMapMapped ceilPosePoseMatStamp(:,[14:15])]);
evalPathDiff([slamPosePoseMatStamp(:,[10:12 14:15]) ], [vrMapMapped vrPosePoseMatStamp(:,[14:15])]);



[ceilingDistAng1, ceilingDistAng2, ceilingAngRangeInd] = getDistAngInfo(ceilPosePoseMatStamp(:,end), posestPoseMapMapped(:,1:3),2,5,3); % ceiling
[vrDistAng1, vrDistAng2,vrAngRangeInd] = getDistAngInfo(vrPosePoseMatStamp(:,end), vrMapMapped(:,1:3), 0.6,10,3); % vr
[wheelDistAng1, wheelDistAng2,wheelAngRangeInd] = getDistAngInfo(roboPosePoseMatStamp(:,end), roboPosePoseMatStamp(:,10:12), 1,1,1); % wheel
[visionDistAng1, visionDistAng2,visionAngRangeInd] = getDistAngInfo(slamPosePoseMatStamp(:,end), slamPosePoseMatStamp(:,10:12), 1,2,2); % vision

% % evalAng(ceilPosePoseMatStamp,ceilingAngRangeInd,roboPosePoseMatStamp,wheelAngRangeInd);
% % evalAng(ceilPosePoseMatStamp,ceilingAngRangeInd,slamPosePoseMatStamp,visionAngRangeInd)

% % % figure(71),clf;hold on;
% % % for k = 1 : min(size(ceilingAngRangeInd,1),size(wheelAngRangeInd,1))
% % %     ceilingAng = ceilPosePoseMatStamp(ceilingAngRangeInd(k,1):ceilingAngRangeInd(k,2),end);
% % %     ceilingAng = ceilingAng - ceilingAng(1);
% % %     ceilId = find(ceilingAng > 0);
% % %     ceilingTime = ceilPosePoseMatStamp(ceilingAngRangeInd(k,1):ceilingAngRangeInd(k,2),14);
% % %     ceilingTime = ceilingTime - ceilingTime(1);
% % %     ceilingAng = ceilingAng(ceilId);
% % %     ceilingTime = ceilingTime(ceilId);
% % %     ceilingTime = ceilingTime - ceilingTime(1);
% % %
% % %     wheelAng = roboPosePoseMatStamp(wheelAngRangeInd(k,1):wheelAngRangeInd(k,2),end);
% % %     wheelAng = wheelAng - wheelAng(1);
% % %     wheelId = find(wheelAng > 0);
% % %     wheelTime = roboPosePoseMatStamp(wheelAngRangeInd(k,1):wheelAngRangeInd(k,2),14);
% % %     wheelTime = wheelTime - wheelTime(1);
% % %     wheelAng = wheelAng(wheelId);
% % %     wheelTime = wheelTime(wheelId);
% % %     wheelTime = wheelTime - wheelTime(1);
% % %     figure(71);plot(ceilingTime, ceilingAng,'-r');
% % %     plot(wheelTime,wheelAng ,'-g');
% % % % %    figure(71);plot(ceilingAng,'-r');
% % % % %     plot(wheelAng ,'-g');
% % %
% % % end

% fid = fopen(fullfile(vrDir, sprintf('cam_path_%02d.txt', 1)),'w');
fid = fopen(fullfile(vrDir, 'cam_path.txt'),'w');
for j = 1 : size(posestPoseMapMapped,1)
    fprintf(fid, '%0.2f %0.2f %0.2f', posestPoseMapMapped(j,1)./1000,posestPoseMapMapped(j,2)./1000,posestPoseMapMapped(j,3)./1000);
    fprintf(fid, '\n');
end
fclose(fid);

figure,hold on;plot(ceilPosePoseMatStamp(:,14),ceilPosePoseMatStamp(:,end),'-'); plot(roboPosePoseMatStamp(:,14),roboPosePoseMatStamp(:,end),'-');title('angle');legend('ceiling','wheel');

figure,hold on;plot(ceilPosePoseMatStamp(:,14),ceilPosePoseMatStamp(:,end),'-');plot(vrPosePoseMatStamp(:,14),vrPosePoseMatStamp(:,end),'-');
plot(roboPosePoseMatStamp(:,14),roboPosePoseMatStamp(:,end),'-');plot(slamPosePoseMatStamp(:,14),slamPosePoseMatStamp(:,end),'-');
title('angle');legend('ceiling','vr','wheel','vision');
figure,hold on;plot(ceilPosePoseMatStamp(:,14),posestPoseMapMapped(:,1),'-');plot(vrPosePoseMatStamp(:,14),vrMapMapped(:,1),'-');
plot(roboPosePoseMatStamp(:,14),roboPosePoseMatStamp(:,10),'-');plot(slamPosePoseMatStamp(:,14),slamPosePoseMatStamp(:,10),'-');
title('x');legend('ceiling','vr','wheel','vision');
figure,hold on;plot(ceilPosePoseMatStamp(:,14),posestPoseMapMapped(:,3),'-');plot(vrPosePoseMatStamp(:,14),vrMapMapped(:,3),'-');
plot(roboPosePoseMatStamp(:,14),roboPosePoseMatStamp(:,12),'-');plot(slamPosePoseMatStamp(:,14),slamPosePoseMatStamp(:,12),'-');
title('z');legend('ceiling','vr','wheel','vision');


figure,hold on;plot(posestPoseMapMapped(:,1),posestPoseMapMapped(:,3),'-');plot(vrMapMapped(:,1),vrMapMapped(:,3),'-');
plot(roboPosePoseMatStamp(:,10),roboPosePoseMatStamp(:,12),'-');plot(slamPosePoseMatStamp(:,10),slamPosePoseMatStamp(:,12),'-');
axis equal;legend('ceiling','vr','wheel','vision');

% % figure,plot(roboMap(:,1),roboMap(:,3),'-r');hold on;
% % plot(vrMapMapped(:,1),vrMapMapped(:,3),'.g');
% % plot(posestPoseMapMapped(:,1),posestPoseMapMapped(:,3),'-b');
% % axis equal
% % legend('wheel', 'vr', 'ceiling');

end
function evalAng(ceilPosePoseMatStamp,ceilingAngRangeInd,roboPosePoseMatStamp,wheelAngRangeInd)
figure(71),clf;hold on;
for k = 1 : min(size(ceilingAngRangeInd,1),size(wheelAngRangeInd,1))
    ceilingAng = ceilPosePoseMatStamp(ceilingAngRangeInd(k,1):ceilingAngRangeInd(k,2),end);
    ceilingAng = ceilingAng - ceilingAng(1);
    %     ceilId = find(ceilingAng > 0);
    ceilingTime = ceilPosePoseMatStamp(ceilingAngRangeInd(k,1):ceilingAngRangeInd(k,2),14);
    ceilingTime = ceilingTime - ceilingTime(1);
    %     ceilingAng = ceilingAng(ceilId);
    %     ceilingTime = ceilingTime(ceilId);
    %     ceilingTime = ceilingTime - ceilingTime(1);
    
    wheelAng = roboPosePoseMatStamp(wheelAngRangeInd(k,1):wheelAngRangeInd(k,2),end);
    wheelAng = wheelAng - wheelAng(1);
    %     wheelId = find(wheelAng > 0);
    wheelTime = roboPosePoseMatStamp(wheelAngRangeInd(k,1):wheelAngRangeInd(k,2),14);
    wheelTime = wheelTime - wheelTime(1);
    %     wheelAng = wheelAng(wheelId);
    %     wheelTime = wheelTime(wheelId);
    %     wheelTime = wheelTime - wheelTime(1);
    figure(71);plot(ceilingTime, ceilingAng,'-r');
    plot(wheelTime,wheelAng ,'-g');
    % %    figure(71);plot(ceilingAng,'-r');
    % %     plot(wheelAng ,'-g');
    
end
end
