function posestPoseMapped = PlotRobot2(inputDir,startt, posestDir,tsCeiling)   %,timeInd)  %,posestPoseMapped)
posestInfo = dir(fullfile(posestDir,'pose*'));
load(fullfile(posestDir, posestInfo(1).name));
dirInfo = dir(fullfile(inputDir,'comm_client*.txt'));
% % close all
useNewHua = 0;

thr = 2;
thrStillVR = 10;
thrStillSlam = 10;
thrStillRobo = 10;
draw = 0; 1; 0;
for  i = startt : length(dirInfo)
    
    [pathFid, errMsg] = fopen(fullfile(inputDir,dirInfo(i).name));
    if (pathFid < 0)
        error('Cannot open %s for read: %s', fullfile(inputDir,dirInfo(i).name), errMsg);
    end
    pathBuf = [];
    inReadPath = false;
    % % flagg=0;
    roboPath = [];
    vrPath = [];
    slamPath = [];
    
    timeStamp2 = [];
    timeStamp3 = [];
    
    
    
    countVR = 0;
    countSlam = 0;
    countRobo = 0;
    cntVR = []; cntRobo = [];
    if draw == 1
        figure,subplot(1,2,1);title('vr');hold on;
        subplot(1,2,2);title('robot');hold on;
    end
    
    count = 1; countS = 1;
    while ~feof(pathFid)
        lineBuf = strtrim(fgetl(pathFid));
        
        if length(lineBuf) < 10
            continue;
        end
        try
            aa = lineBuf(1:6);
        catch
            shkj = 1;
        end
        
        
        
        %         LineBufStack{count,:} = lineBuf;
        %         count = count + 1;
        
        if strcmp(lineBuf(1:2), 'VR')
            flagg=1;
        end
        if strcmp(lineBuf(1:2), 'Sl')
            flagg=333;
        end
        if strcmp(lineBuf(1:8), 'Robot FI')
            flagg=22;
        end
        if strcmp(lineBuf(1:2), 'RT')
            rtList = str2double(strsplit(lineBuf(4:end), ' '));%lineBuf
            rt = reshape(rtList,3,4)';
            TClient = [rt(1:3,1:3) 1000.*rt(4,:)';0 0 0 1];
            % %             vrMatch = [vrMatch; 1001*vrmatch];
            flagg = 5;
        end
        
        
        if flagg == 333     % 1
            % %             vrpath = str2double(strsplit(lineBuf(8:end), ' '));%lineBuf
            slampath = str2double(strsplit(lineBuf(11:end), ' '));
            if length(slampath) == 9
                ts3 = slampath(3);
                slampath = slampath([2 4:end]);
            else
                ts3 = slampath(1);
            end
            
            
            
            SlamBufStack{countS,:} = lineBuf;
            countS = countS + 1;
            try
                % %                 vrpath(2:4) = [-1000 1000 1000].*vrpath(2:4);
                slampath(2:4) = [1000 1000 1000].*slampath(2:4);
                
                if isempty(slamPath)
                    if 0 %draw == 1
                        subplot(1,2,1);plot(vrpath(2),vrpath(4),'sb','LineWidth',2);axis equal;drawnow;
                    end
                end
                
                if norm(slampath(2:4) - slamPath(end,1:3)) > thr
                    if draw == 1
                        subplot(1,2,1);plot(slampath(2),slampath(4),'.r');axis equal;drawnow;
                    end
                end
                countSlam = countSlam + 1;
                if norm(slampath(2:4) - slamPath(end,1:3)) < thrStillSlam
                    cntSlam = [cntSlam;countSlam];
                end
            catch
                evhsergoeq = 1;
            end
            try
                slamPath = [slamPath; slampath(2:end)];
                timeStamp3 = [timeStamp3; ts3];
            catch
                wvhsdfgkj = 1;
            end
            falgg = 0;
        end
        
        
        
        
        if flagg == 22     % 1
            % %             vrpath = str2double(strsplit(lineBuf(8:end), ' '));%lineBuf
            vrpath = str2double(strsplit(lineBuf(11:end), ' '));
            if length(vrpath) == 9
                ts2 = vrpath(3);
                vrpath = vrpath([2 4:end]);
            else
                ts2 = vrpath(1);
            end
            
            
            %%
            if useNewHua == 1
                vrpath(2) = -vrpath(2);
            end

            WheelBufStack{count,:} = lineBuf;
            count = count + 1;
            try
                % %                 vrpath(2:4) = [-1000 1000 1000].*vrpath(2:4);
                vrpath(2:4) = [1000 1000 1000].*vrpath(2:4);
                
                if isempty(vrPath)
                    if draw == 1
                        subplot(1,2,1);plot(vrpath(2),vrpath(4),'sb','LineWidth',2);axis equal;drawnow;
                    end
                end
                
                if norm(vrpath(2:4) - vrPath(end,1:3)) > thr
                    if draw == 1
                        subplot(1,2,1);plot(vrpath(2),vrpath(4),'.r');axis equal;drawnow;
                    end
                end
                countVR = countVR + 1;
                if norm(vrpath(2:4) - vrPath(end,1:3)) < thrStillVR
                    cntVR = [cntVR;countVR];
                end
            catch
                evhoeq = 1;
            end
            try
                vrPath = [vrPath; vrpath(2:end)];
                timeStamp2 = [timeStamp2; ts2];
            catch
                wvhkj = 1;
            end
            falgg = 0;
        end
        if flagg == 2
            robopath = str2double(strsplit(lineBuf(11:end), ' '));%lineBuf
            try
                robopath(2:4) = 1000.*robopath(2:4);
                if isempty(roboPath)
                    if draw == 1
                        subplot(1,2,2);plot(robopath(2),robopath(4),'sb','LineWidth',2);axis equal;drawnow;
                    end
                end
                if norm(robopath(2:4) - roboPath(end,1:3)) > thr
                    %                     subplot(1,2,2);plot(robopath(2),robopath(4),'.r');axis equal;drawnow;
                end
                countRobo = countRobo + 1;
                %                 if norm(robopath(2:4) - roboPath(end,1:3)) < thrStillRobo
                %                     cntRobo = [cntRobo;countRobo];
                %                 end
            catch
                aghij = 1;
            end
            try
                roboPath = [roboPath; robopath(2:end)];
            catch
                wghei = 1;
            end
            falgg = 0;
            
        end
        
        
        
    end
    G=1;
    fclose(pathFid);
    [~,dltDistVR1] = NormalizeVector(vrPath(1:end-1,1:3) - vrPath(2:end,1:3));
    %     [~,dltDistRobo1] = NormalizeVector(roboPath(1:end-1,1:3) - roboPath(2:end,1:3));
    dltDistVR = medfilt1(dltDistVR1,(2*10 - 1));
    %     dltDistRobo = medfilt1(dltDistRobo1,(2*10 - 1));
    
    % [indexCellVR,idxVR] = splitIndex2(cntVR);
    % [indexCellRobo,idxRobo] = splitIndex2(cntRobo);
    [indexCellVR,idxVR] = splitIndex2(find(dltDistVR <thrStillVR));
    %     [indexCellRobo,idxRobo] = splitIndex2(find(dltDistRobo <thrStillRobo));
    
    
    lenVR = [];
    for ii = 1 : length(indexCellVR)-1
        indexPrv = indexCellVR{ii} + 0;
        indexCur = indexCellVR{ii+1} + 1;
        % %     len(ii,:) = [vrPath(round(median(indexPrv)),[1 3]) vrPath(round(median(indexCur)),[1 3]) norm(vrPath(round(median(indexPrv)),[1 3]) - vrPath(round(median(indexCur)),[1 3]))];
        lenVR(ii,:) = [vrPath(round(indexPrv(end)),[1 3]) vrPath(round(indexCur(1)),[1 3]) norm(vrPath(round(indexPrv(end)),[1 3]) - vrPath(round(indexCur(1)),[1 3]))];
    end
    %     lenRobo = [];
    %     for jj = 1 : length(indexCellRobo)
    %
    %         if jj == 1
    %             indexPrv = [1];
    %         else
    %             indexPrv = indexCellRobo{jj-1} - 2;
    %         end
    %         indexCur = indexCellRobo{jj} + 1;
    %         % %     len(ii,:) = [vrPath(round(median(indexPrv)),[1 3]) vrPath(round(median(indexCur)),[1 3]) norm(vrPath(round(median(indexPrv)),[1 3]) - vrPath(round(median(indexCur)),[1 3]))];
    %         if jj == 1
    %             lenRobo(jj,:) = [[0 0] roboPath(round(indexCur(1)),[1 3]) norm([0 0] - roboPath(round(indexCur(1)),[1 3]))];
    %         else
    %             lenRobo(jj,:) = [roboPath(round(indexPrv(end)),[1 3]) roboPath(round(indexCur(1)),[1 3]) norm(roboPath(round(indexPrv(end)),[1 3]) - roboPath(round(indexCur(1)),[1 3]))];
    %         end
    %     end
    if draw == 1
        subplot(1,2,1),
        for iii = 1 : size(lenVR,1)
            text((lenVR(iii,1)+lenVR(iii,3))/2,(lenVR(iii,2)+lenVR(iii,4))/2,num2str(lenVR(iii,5)), 'Color',[1 0 1],'FontSize',10,'FontWeight','bold');
        end
    end
    
    % %     subplot(1,2,2),
    % %     for jjj = 1 : size(lenRobo,1)
    % %         text((lenRobo(jjj,1)+lenRobo(jjj,3))/2,(lenRobo(jjj,2)+lenRobo(jjj,4))/2,num2str(lenRobo(jjj,5)), 'Color',[1 0 1],'FontSize',10,'FontWeight','bold');
    % %     end
    
    if 0
        figure,subplot(2,2,1);plot(dltDistVR1);title('vr before');
        subplot(2,2,2);plot(dltDistRobo1);title('robot before');
        subplot(2,2,3);plot(dltDistVR);title('vr after');
        subplot(2,2,4);plot(dltDistRobo);title('robot after');
    end
    
    WheelBufStack = WheelBufStack(1:end-1,:);
    SlamBufStack = SlamBufStack(1:end-1,:);
    posestPoseMapped = [posestPoseMapped(:,2) zeros(size(posestPoseMapped,1),1) posestPoseMapped(:,1)];
    [dltRobo] = diff(posestPoseMapped(1:end,1:3));
    [~,lenRobo] = NormalizeVector(dltRobo);
    idid = find(lenRobo > 5);
    
    posestPoseMapped = posestPoseMapped;
    for k = 1 : size(posestPoseMapped,1)
        try
            CeilingBufStack{k,:} = sprintf('VR FID %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f\n',tsCeiling(k),posestPoseMapped(k,:)./1000,posestPoseMapped(k,:)./1000);
            % %         CeilingBufStack{k,:} = sprintf('VR FID %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f\n',(k),posestPoseMapped(k,:)./1000,posestPoseMapped(k,:)./1000);
        catch
            asvibjk = 1;
        end
    end
    vrPath(:,1) = -vrPath(:,1);
    if draw == 1
        figure,plot(posestPoseMapped(:,1),posestPoseMapped(:,3),'.');hold on; plot(vrPath(:,1),vrPath(:,3),'.');axis equal
    end
    ebve = 1;
    if 0
        [dltRobo] = diff(posestPoseMapped(1:end,1:3));
        [~,lenRobo] = NormalizeVector(dltRobo);
        [dltVR] = diff(vrPath(1:end,1:3));
        [~,lenVR] = NormalizeVector(dltVR);
        idRobo = find(lenRobo > 10);
        idVR = find(lenVR > 10);
        roboLIST = posestPoseMapped(idRobo,[1 2 3]);
        vrLIST = vrPath(idVR,[1 2 3]);
        
        randId = randperm(size(vrLIST,1));
        
        [ T , Y ] = ICP( [vrLIST(randId(1:size(roboLIST,1)),:) ones(size(roboLIST,1),1)]' ,[roboLIST ones(size(roboLIST,1),1)]');
        
        [R, t]=ICPMatching(roboLIST(:,[1 3])', vrLIST(randId(1:size(roboLIST,1)),[1 3])');
        %         ICP( [posestPoseMapped ones(size(posestPoseMapped,1),1)]', [vrPath(randId(1:size(posestPoseMapped,1)),1:3) ones(size(posestPoseMapped,1),1)]' );
        
        %         tform = pcregrigid(pointCloud(posestPoseMapped),pointCloud(vrPath(randId(1:size(posestPoseMapped,1)),1:3)),'Extrapolate',true,'InlierRatio',0.8);
        tform = pcregrigid(pointCloud(roboLIST),pointCloud(vrLIST(randId(1:size(roboLIST,1)),:)),'Extrapolate',true,'InlierRatio',0.8);
        %         tform = pcregrigid(pointCloud(bodyList(:,1:3)),pointCloud(vrList(:,1:3)),'Extrapolate',true,'InlierRatio',0.8);
        T = inv(tform.T');
        transformed = T(1:3,1:3)*vrPath(:,1:3)' + repmat(T(1:3,end),1,size(vrPath,1));
        figure,plot(transformed(1,:),transformed(3,:));hold on;plot(posestPoseMapped(:,1),posestPoseMapped(:,3));axis equal;
        
        [~, err] = NormalizeVector(transformed' - matchPtBody);
        transformed2 = T(1:3,1:3)*vrPath(:,1:3)' + repmat(T(1:3,end),1,size(vrPath,1));
        transformede3 = R*vrPath(:,[1 3])' + repmat(t,1,size(vrPath,1));
        figure,plot(transformede3(1,:),transformede3(2,:));hold on;plot(posestPoseMapped(:,1),posestPoseMapped(:,3));
        
    end
    
    % % % %     fidd = fopen(fullfile(inputDir, sprintf('cam_path_%02d.txt', i)),'w');
    % % % %     for j = 1 : size(posestPoseMapped,1)
    % % % %         fprintf(fidd, '%0.2f %0.2f %0.2f', posestPoseMapped(j,1),posestPoseMapped(j,2),posestPoseMapped(j,3));
    % % % %         fprintf(fidd, '\n');
    % % % %     end
    % % % %     fclose(fidd);
    
    fid = fopen(fullfile(inputDir, sprintf('comm_path_ceiling_%02d.txt', i)),'w');
    cttVrThr = length(CeilingBufStack);
    cttRoboThr = length(WheelBufStack);
    cttSlamThr = length(SlamBufStack);
    cttVr = 1;
    cttRobo = 1;
    cttSlam = 1;
    % %     while cttVr <= cttVrThr
    while cttRobo <= cttRoboThr
        for xx = 1 : 1
            try
                fprintf(fid, strcat(CeilingBufStack{cttVr},'\n'));
            catch
                % %                 break;
                fprintf(fid, strcat(CeilingBufStack{end},'\n'));
            end
            cttVr = cttVr + 1;
        end
        if cttVr > 20   %150
            try
                fprintf(fid, strcat(WheelBufStack{cttRobo},'\n'));
                cttRobo = cttRobo + 1;
            catch
                svklj = 1;
            end
            try
                fprintf(fid, strcat(SlamBufStack{cttSlam},'\n'));
                cttSlam = cttSlam + 1;
            catch
                svklj = 1;
            end
        end
        
    end
    
    % %     ctt = 1;
    % %     while 1
    % %         try
    % %             fprintf(fid, strcat(RoboBufStack{ctt},'\n'));
    % %             ctt = ctt + 1;
    % %         catch
    % %             break;
    % %         end
    % %     end
    
    % for i = 1 : length(wholeI2CVec)
    %     if i ~= length(wholeI2CVec)
    %         fprintf(fid, '%0.6f ', wholeI2CVec(i));
    %     else
    %         fprintf(fid, '%0.6f', wholeI2CVec(i));
    %     end
    % end
    fclose(fid);
    
    
end

end