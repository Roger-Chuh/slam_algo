function readRosBag(inputDir,topics)

% readRosBag('D:\Temp\20180919\rodbag1',{'/odom'})
% readRosBag('D:\Temp\20181016\rosbag3',{'/odom','/mobile_base/sensors/core'});

draw = 0;
dirInfo = dir(fullfile(inputDir,'*.bag'));
sonarDist = 0.35;0.4;0.3;0.25; 0.4;
veloThr = 0.04;
% msgs = bag.readAll(topics);
cnt = 1; cnt2 = 1;
for i = 1:length(dirInfo)
    bag = ros.Bag.load(fullfile(inputDir,dirInfo(i).name));
    [msgs, meta] = bag.readAll(topics);
    accumAng = 0; slamPoseMat = []; poseFid = [];
    for j = 1 : length(msgs)
        
        timeS(j,:) = ([double(msgs{1, j}.header.seq) double(msgs{1, j}.header.stamp.sec) double(msgs{1, j}.header.stamp.nsec) double(msgs{1, j}.header.stamp.time) ]);
        try
            infoMat{i}.pose(cnt,:) = [double(msgs{1, j}.pose.pose.position') double(rodrigues(q2R(msgs{1, j}.pose.pose.orientation([4 1 2 3])))')];
            infoMat{i}.time(cnt,:) = ([double(msgs{1, j}.header.seq) double(msgs{1, j}.header.stamp.sec) double(msgs{1, j}.header.stamp.nsec) double(msgs{1, j}.header.stamp.time) ]);
            % %             infoMat{i}.pose(cnt,:) = infoMat{i}.pose(cnt,[2 3 1 4 6 5]);
            infoMat{i}.pose(cnt,:) = infoMat{i}.pose(cnt,[2 3 1 4 6 5]);
            infoMat{i}.pose(cnt,[1 5]) = -infoMat{i}.pose(cnt,[1 5]);  %infoMat{i}.pose(cnt,[2 3 1 4 6 5]);
            rmatSlam = rodrigues(infoMat{i}.pose(cnt,end-2:end));  % [[slampath(7) 0 -slampath(5)]' [0 1 0]' slampath(5:7)']';
            slamPoseMat = [slamPoseMat;[reshape(rmatSlam,1,9) infoMat{i}.pose(cnt,1:3)]];
            
            poseFid = [poseFid; [infoMat{i}.pose(cnt,1:3) rmatSlam(3,:)]];
            
            if size(infoMat{1, 1}.pose,1) > 1  %j > 1
                dltT =  [rmatSlam infoMat{i}.pose(cnt,1:3)';0 0 0 1]*inv([reshape(slamPoseMat(end-1,1:9),3,3) slamPoseMat(end-1,10:12)';0 0 0 1]);
                dltr = dltT(1:3,1:3);
                dltt = dltT(1:3,end);
                prvPt = [slamPoseMat(end-1,10);0;slamPoseMat(end-1,12)];
                curPt = [slamPoseMat(end,10);0;slamPoseMat(end,12)];
                dltAng = rad2deg(norm(rodrigues(dltr)));
                err = dltr*prvPt + dltt - curPt;
                if norm(rodrigues(roty(dltAng)) - rodrigues(dltr)) < norm(rodrigues(roty(-dltAng)) - rodrigues(dltr))
                    accumAng = [accumAng; accumAng(end) + dltAng];
                else
                    accumAng = [accumAng; accumAng(end) - dltAng];
                end
                
            end
            cnt = cnt + 1;
        catch
            timeS2(cnt2,:) = ([double(msgs{1, j}.header.seq) double(msgs{1, j}.header.stamp.sec) double(msgs{1, j}.header.stamp.nsec) double(msgs{1, j}.header.stamp.time) ]);
            try
                sonar(cnt2,:) = double(msgs{1, j}.distance')./1000;
            catch
                asvknj = 1;
            end
            cnt2 = cnt2 + 1;
            asdgfv = 1;
            
        end
        
        
        
        
    end
    
    timeStamp = infoMat{1}.time(:,end) - infoMat{1}.time(1,end);
    timeS1 = infoMat{1}.time;
    try
        timeMat = [timeS1(1:min(size(timeS1,1),size(timeS2,1)),end)  timeS2(1:min(size(timeS1,1),size(timeS2,1)),end)];
    catch
        timeMat = timeS1;
    end
    
    timeMat = timeMat - repmat(min(timeMat(:)),size(timeMat));
    try
        poseFid = poseFid(1:min(size(timeS1,1),size(timeS2,1)),:);
    catch
        avdkn = 1;
    end
    [~,dis] = NormalizeVector(1000.*slamPoseMat(1:end-1,10:12)-1000.*slamPoseMat(2:end,10:12));
    idd = find(dis > 0.1);
    
    baseNum = idd(1);
    
    timeMat = timeMat(baseNum:end,:);
    timeMat = timeMat - repmat(min(timeMat(:)),size(timeMat));
    
    slamPoseMat2 = [];poseFidd = [];
    rt1675 = [reshape(slamPoseMat(baseNum,1:9),3,3) slamPoseMat(baseNum,10:12)';0 0 0 1];
    for ii = baseNum:size(slamPoseMat,1)
        tmpRT =  [reshape(slamPoseMat(ii,1:9),3,3) slamPoseMat(ii,10:12)';0 0 0 1];
        tmpRt2 = inv(rt1675)*tmpRT;
        slamPoseMat2 = [slamPoseMat2;[reshape(tmpRt2(1:3,1:3),1,9) tmpRt2(1:3,4)']];
        poseFidd = [poseFidd;[tmpRt2(1:3,4)' tmpRt2(3,1:3)]];
    end
    
    
    
    
    %%
    if draw
        figure,plotPath(slamPoseMat)
    end
    
    
    
    Ts2b = inv(inv([roty(-90) [-180/1000;0;-90/1000];0 0 0 1]));
    accumAng1 = 0;
    sonar = sonar(1:size(timeMat,1),:);
    
    sonar(:,7) = mean(sonar(:,5:6),2);
    
    camPoseMat2 = slamPoseMat;
    for h = 1 : size(timeMat,1)  %size(camPoseMat2,1)
        
        
        if h == 194
            asfkj = 1;
        end
        Told = [reshape(camPoseMat2(h,1:9),3,3) camPoseMat2(h,10:12)';0 0 0 1];
        %    dltT = [rt;0 0 0 1];
        % %         Tnew = Ts2b*Told;
        Tnew = Told*Ts2b;
        sonar3d_ = [0 0 0;0 0 0;sonar(h,5:end)];
        sonar3d = Tnew(1:3,1:3)*sonar3d_ + repmat(Tnew(1:3,4),1,size(sonar3d_,2));
        sonar3dMat(h,:) = sonar3d(:)';
        camPoseMatNew(h,:) = [reshape(Tnew(1:3,1:3),1,9) Tnew(1:3,end)'];
        if size(camPoseMatNew,1) > 1
            dltT =  Tnew*inv([reshape(camPoseMatNew(end-1,1:9),3,3) camPoseMatNew(end-1,10:12)';0 0 0 1]);
            dltr = dltT(1:3,1:3);
            dltt = dltT(1:3,end);
            prvPt = [camPoseMatNew(end-1,10);0;camPoseMatNew(end-1,12)];
            curPt = [camPoseMatNew(end,10);0;camPoseMatNew(end,12)];
            prvPt = [camPoseMatNew(end-1,10);0;camPoseMatNew(end-1,12)];
            curPt = [camPoseMatNew(end,10);0;camPoseMatNew(end,12)];
            errSlam = dltr*prvPt + dltt - curPt;
            try
                dltAng = rad2deg(norm(rodrigues(dltr)));
            catch
                ewrgl = 1;
            end
            try
                if norm(rodrigues(roty(dltAng)) - rodrigues(dltr)) < norm(rodrigues(roty(-dltAng)) - rodrigues(dltr))
                    accumAng1 = [accumAng1; accumAng1(end) + dltAng];
                else
                    accumAng1 = [accumAng1; accumAng1(end) - dltAng];
                end
            catch
                weflkh = 1;
            end
            if length(accumAng1) >= 2676
                asdfgj = 1;
            end
        end
        
    end
    
    [~,disp] = NormalizeVector(slamPoseMat(1:end,10:12) - repmat(slamPoseMat(1,10:12),size(slamPoseMat,1),1));
    cdisp = cumsum(disp);
    if draw
        figure,plot(cumsum(disp))
        figure,plot(slamPoseMat(:,10),slamPoseMat(:,12));axis equal
    end
    differ1 = (slamPoseMat(2:end,[10:12])-slamPoseMat(1:end-1,[10:12]));
    [~,disppp1] = NormalizeVector(differ1);
    cdisp1 = cumsum(disppp1);
    if draw
        figure(3),plot(cdisp1);hold on;
    end
    abs(cdisp1(end) - disp(end));
    
    
    
    
    %     differ2 = (slamPoseMat(2:end,10:11)-slamPoseMat(1:end-1,10:11));
    differ2 = (slamPoseMat(2:end,10:12)-slamPoseMat(1:end-1,10:12));
    [~,disppp2] = NormalizeVector(differ2);
    if draw
        figure,plot(disppp2)
        figure(3),plot(cumsum(disppp2));
        figure,plot(disppp2)
    end
    differ2 = diff(slamPoseMat(1:end,10:11));
    differ2 = (slamPoseMat(2:end,[10 12])-slamPoseMat(1:end-1,[10 12]));
    [~,disppp3] = NormalizeVector(differ2);
    if draw
        figure,plot(disppp3)
        figure(3),plot(cumsum(disppp3))
    end
    
    if draw
        figure,plot(slamPoseMat(:,10),slamPoseMat(:,12));hold on;plot(camPoseMatNew(:,10),camPoseMatNew(:,12));axis equal;
        plot(sonar3dMat(sonar(:,5)<sonarDist&sonar(:,6)<sonarDist,1),sonar3dMat(sonar(:,5)<sonarDist & sonar(:,6)<sonarDist,3),'oc');
        plot(sonar3dMat(sonar(:,6)<sonarDist,4),sonar3dMat(sonar(:,6)<sonarDist,6),'om');
        plot(sonar3dMat(sonar(:,7)<sonarDist,7),sonar3dMat(sonar(:,7)<sonarDist,9),'oy');
    end
    slamPoseMat_ = slamPoseMat(1:size(timeMat,1),:);
    [~,dltLen] = NormalizeVector(slamPoseMat_(2:end,10:12) - slamPoseMat_(1:end-1,10:12));
    
    velo = dltLen./diff(timeMat(:,1));
    veloAng = diff(accumAng1)./diff(timeMat(:,1));
    velo = [velo(1);velo];
    if 0
        poseFid2 = [[1:size(timeMat,1)]' repmat(round(1000.*timeMat(:,2)),1,2) poseFid];
        id = find(poseFid2(:,6) == 0);
        
        poseFid2 = poseFid2(id(end):end,:);
    else
        poseFid2 = [[1:size(timeMat,1)]' repmat(round(1000.*timeMat(:,2)),1,2) poseFidd];
    end
    for ji = 1 : size(poseFid2,1)
        SlamBufStack{ji,1} = sprintf('Slam FID %d %d %d %f %f %f %f %f %f',poseFid2(ji,1:3),poseFid2(ji,4:end));
        RobotBufStack{ji,1} = sprintf('Robot FID %d %d %d %f %f %f %f %f %f',poseFid2(ji,1:3),poseFid2(ji,4:end));
    end
    
    
    fid = fopen(fullfile(inputDir, 'comm_client_log.txt'),'w');
    
    for ij = 1: length(RobotBufStack)
        fprintf(fid, strcat(RobotBufStack{ij},'\n'));
        fprintf(fid, strcat(SlamBufStack{ij},'\n'));
    end
    fclose(fid);
    return
    
    inFlag = sonar(:,5)<sonarDist&sonar(:,6)<sonarDist;
    figure,hold on;axis equal;
    for ii = 1 : size(timeMat,1)
        if norm(slamPoseMat(ii,10:12)) > 0
            if 1 %accumAng(ii) - accumAng(ii-1) ~= 0 && velo(ii) > veloThr
                plot(slamPoseMat(1:ii,10),slamPoseMat(1:ii,12),'-b');
                plot(camPoseMatNew(1:ii,10),camPoseMatNew(1:ii,12),'-r');
                if sonar(ii,5) < sonarDist && sonar(ii,6)<sonarDist % && sonar(ii,7)<sonarDist %sonar(ii,7)<sonarDist %
                    ii
                    plot(sonar3dMat(ii,7),sonar3dMat(ii,9),'om');
                end
                drawnow;
            end
        end
    end
    
    
    
    
    
    
    [disp1,accumAng1,ind01,pt1,data1,timeStamp1] = readBeiDumpCalib2('D:\Temp\20180930\rosbag1','D:\Temp\20180930\param','0','0');
    figure,plot(timeStamp, accumAng);hold on;plot( timeStamp1, accumAng1)
    
end





end