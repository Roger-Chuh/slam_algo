function vrPosePoseMatStamp = PlotVR3(inputDir,startt)
% posestInfo = dir(fullfile(posestDir,'pose*'));
% load(fullfile(posestDir, posestInfo(1).name));
dirInfo = dir(fullfile(inputDir,'comm_client*.txt'));
% % close all
thr = 2;
vrStartThr = 0.6;
angThr = 0.6; % degree;
thrStillVR = 10;
thrStillRobo = 10;
draw = 0;
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
    RT = []; camPoseMat = []; accumAng1 = 0;
    countVR = 0;
    countRobo = 0;
    cntVR = []; cntRobo = []; vrStamp = [];
    if draw == 1
        figure,subplot(1,2,1);title('vr');hold on;
        subplot(1,2,2);title('robot');hold on;
    end
    
    count = 1;
    while ~feof(pathFid)
        lineBuf = strtrim(fgetl(pathFid));
        %         LineBufStack{count,:} = lineBuf;
        %         count = count + 1;
        if length(lineBuf) < 10
            continue;
        end
        
        try
            if strcmp(lineBuf(1:6), 'VR FID')
                flagg=1;
            end
        catch
            sdhj = 1;
        end
        if strcmp(lineBuf(1:2), 'Sl')
            flagg=333;
        end
        if strcmp(lineBuf(1:2), 'Ro')
            flagg=22;
        end
        if strcmp(lineBuf(1:2), 'RT')
            rtList = str2double(strsplit(lineBuf(4:end), ' '));%lineBuf
            rt = reshape(rtList,3,4)';
            TClient = [rt(1:3,1:3) 1000.*rt(4,:)';0 0 0 1];
            % %             vrMatch = [vrMatch; 1001*vrmatch];
            flagg = 5;
        end
        if strcmp(lineBuf(1:6), 'VR Fit')
            flagg=555;
        end
        
        
        if flagg == 1
            vrpath = str2double(strsplit(lineBuf(8:end), ' '));%lineBuf
            if length(vrpath) < 6
                continue;
            end
            LineBufStack{count,:} = lineBuf;
            count = count + 1;
            try
                % %                 vrpath(2:4) = [-1000 1000 1000].*vrpath(2:4);
                vrpath(2:4) = [1000 1000 1000].*vrpath(2:4);
                if isnan(vrpath(1))
                    savlk = 1;
                end
                if length(vrpath) ~= 7
                    ahvjk;
                end
                %                 vrStamp = [vrStamp; vrpath(1)];
                t = [1 1 1].*vrpath(2:4);
                R = rotz(90)*rotz(vrpath(7))*roty(vrpath(5))*rotx(vrpath(6));
                camPoseMat = [camPoseMat;[reshape(R,1,9) t]];
                if size(camPoseMat,1) > 1
                    dltT =  [R t';0 0 0 1]*inv([reshape(camPoseMat(end-1,1:9),3,3) camPoseMat(end-1,10:12)';0 0 0 1]);
                    dltr = dltT(1:3,1:3);
                    dltt = dltT(1:3,end);
                    prvPt = [camPoseMat(end-1,10);0;camPoseMat(end-1,12)];
                    curPt = [camPoseMat(end,10);0;camPoseMat(end,12)];
                    dltAng = rad2deg(norm(rodrigues(dltr)));
                    if norm(rodrigues(roty(dltAng)) - rodrigues(dltr)) < norm(rodrigues(roty(-dltAng)) - rodrigues(dltr))
                        accumAng1 = [accumAng1; accumAng1(end) + dltAng];
                    else
                        accumAng1 = [accumAng1; accumAng1(end) - dltAng];
                    end
                    
                end
                
                
                if size(camPoseMat,1) > 500
                    %                     figure(11),clf, plotPath(camPoseMat);
                    %                     drawnow;
                end
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
                vrStamp = [vrStamp; vrpath(1)];
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
    [indexCellVRStart,idxVRStart] = splitIndex2(find(dltDistVR > vrStartThr));
    
    for hj = 1 : length(indexCellVRStart)
        if length(indexCellVRStart{hj}) > 80
            startInd = indexCellVRStart{hj}(1)-0;  % -1;
            break;
        end
    end
    
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
    
    
    %     dltDistVrTemp = abs(diff(accumAng));
    if accumAng1(end) < 0
        accumAng1 = -accumAng1;
    end
    dltDistVrTemp = diff(accumAng1);
    dltDistVRFiltTemp = medfilt1(dltDistVrTemp,(2*10 - 1));
    %     dltDistRobo = medfilt1(dltDistRobo1,(2*10 - 1));
    [indexCellVR,idxVR] = splitIndex2(find(dltDistVRFiltTemp > angThr));
    
    %     [indexCellRobo,idxRobo] = splitIndex2(find(dltDistRoboFiltTemp > angThr));
    
    % idEval = 1;
    for w = 1 : length(indexCellVR) + 1
        if w == 1
            idEval{w,1} = [1 round(mean(indexCellVR{w}))];
        elseif w == length(indexCellVR) + 1
            idEval{w,1} = [idEval{w-1,1}(2) length(accumAng1)];
        else
            idEval{w,1} = [idEval{w-1,1}(2) round(mean(indexCellVR{w}))];
        end
    end
    idx = cell2mat(idEval');
    idx = unique(idx);
    idxAll = zeros(1,length(dltDistVRFiltTemp) + 1);
    idxAll(idx) = 1.5;
    idxAll2 = zeros(1,length(dltDistVRFiltTemp) + 1);
    idxAll2(idx) = max(abs(accumAng1));
    
    for iii = 1 : length(idEval)
        lenVR2(iii,:) = [vrPath(idEval{iii,1}(1),[1 3]) vrPath(idEval{iii,1}(2),[1 3]) norm( vrPath(idEval{iii,1}(1),[1 3]) - vrPath(idEval{iii,1}(2),[1 3]) )];
        
    end
    
    
    
    
    ajs = 1;
    % [indexCellVR,idxVR] = splitIndex2(cntVR);
    % [indexCellRobo,idxRobo] = splitIndex2(cntRobo);
    % % % % % % % %     [indexCellVR,idxVR] = splitIndex2(find(dltDistVR <thrStillVR));
    % % % % % % % %     [indexCellRobo,idxRobo] = splitIndex2(find(dltDistRobo <thrStillRobo));
    
    
    
    if 0
        figure,subplot(1,2,1);plot(dltDistVR1);title('vr before');
        %         subplot(2,2,2);plot(dltDistRobo1);title('robot before');
        subplot(1,2,2);plot(dltDistVR);title('vr after');
        %         subplot(2,2,4);plot(dltDistRobo);title('robot after');
    end
    figure,plot(vrStamp);
    figure(11),clf, plotPath(camPoseMat);
    vrSeconds = unique(vrStamp);
    validSeconds = []; indd = 1;
    for g = 1 : length(vrSeconds);
        ind = find(vrStamp(:) == vrSeconds(g));
        if length(ind) == 20;
            % %               validSeconds = [validSeconds;[(g-1+0.05):0.05:g;(g-1)*20+1:g*20 ]'];
            validSeconds = [validSeconds;[(g-1+0.05) : 0.05 : g; ind']'];
            indd = indd + 1;
        else
            timeStep = 1/length(ind);
            validSeconds = [validSeconds;[(g - 1 + timeStep) : timeStep : g; ind']'];
        end
        
    end
    vrPosePoseMatStamp2 = [camPoseMat(validSeconds(:,2),:) validSeconds(:,2) validSeconds(:,1) accumAng1(validSeconds(:,2))];
    iid = find(vrPosePoseMatStamp2(:,13) == startInd);
    vrPosePoseMatStamp = vrPosePoseMatStamp2(iid:end,:);
    
    
    accumAng = 0;
    dltT0 = eye(4);
    for hh = 1 : length(idEval)
        ToldTmp1 = [reshape(camPoseMat(idEval{hh}(1),1:9),3,3) camPoseMat(idEval{hh}(1),10:12)';0 0 0 1];
        TnewTmp1 = dltT0*ToldTmp1;
        ToldTmp2 = [reshape(camPoseMat(idEval{hh}(2),1:9),3,3) camPoseMat(idEval{hh}(2),10:12)';0 0 0 1];
        TnewTmp2 = dltT0*ToldTmp2;
        
        
        dltTRef =  TnewTmp2*inv(TnewTmp1);
        rotax = rodrigues(dltTRef(1:3,1:3))./norm(rodrigues(dltTRef(1:3,1:3)));
        if hh == 1
            rotAxis(:,hh) = rotax;
        else
            if norm(rotax - rotAxis(:,1)) < 0.5
                rotAxis(:,hh) = rotax;
            else
                rotAxis(:,hh) = -rotax;
            end
        end
    end
    loopcnt = 1;
    for h = 1 : size(vrPosePoseMatStamp,1)
        
        %     if size(vrPosePoseMatStamp,1) > 1
        if h > 1
            % %         dltT =  Tnew*inv([reshape(vrPosePoseMatStamp(end-1,1:9),3,3) vrPosePoseMatStamp(end-1,10:12)';0 0 0 1]);
            dltT =  [reshape(vrPosePoseMatStamp(h,1:9),3,3) vrPosePoseMatStamp(h,10:12)';0 0 0 1]*inv([reshape(vrPosePoseMatStamp(1,1:9),3,3) vrPosePoseMatStamp(1,10:12)';0 0 0 1]);
            dltr = dltT(1:3,1:3);
            dltt = dltT(1:3,end);
            prvPt = [vrPosePoseMatStamp(end-1,10);0;vrPosePoseMatStamp(end-1,12)];
            curPt = [vrPosePoseMatStamp(end,10);0;vrPosePoseMatStamp(end,12)];
            prvPt = [vrPosePoseMatStamp(end-1,10);0;vrPosePoseMatStamp(end-1,12)];
            curPt = [vrPosePoseMatStamp(end,10);0;vrPosePoseMatStamp(end,12)];
            errSlam = dltr*prvPt + dltt - curPt;
            dltAng = rad2deg(norm(rodrigues(dltr)));
            
            % %         if dltAng - accumAng(end) < -50
            % %             dltAng = 180 + dltAng;
            % %         end
            for ee = 1 : length(idEval)
                if ismember(vrPosePoseMatStamp(h,13), [idEval{ee}(1) : idEval{ee}(2)]);
                    rotaxis = rotAxis(:,ee);
                    pos = ee;
                    break;
                end
            end
            if length(accumAng) >= 34
                dsvhj = 1;
            end
            if dltAng > 0.2
                if norm(rodrigues(dltr)./norm(rodrigues(dltr)) - rotaxis) > 1.5 && pos > 1  %find(ismember(h, [idEval{ee}(1) : idEval{ee}(2)]))
                    % %             if 360*loopcnt-dltAng - accumAng(end) < -100
                    % %                 loopcnt = loopcnt + 1;
                    % %             end
                    dltAng  = 360*1-dltAng;  %360*loopcnt-dltAng;
                end
            end
            if dltAng - accumAng(end) < -300
                dltAng = dltAng + abs(round((dltAng - accumAng(end))/360))*360;
            end
            %         if norm(rodrigues(dltr)./norm(rodrigues(dltr)) - rotaxis) > 0.5
            %             dltang  = dltAng;
            %         end
            if h > 165
                qervkj = 1;
            end
            dltang  = dltAng - accumAng(end);
            accumAng = [accumAng; dltAng]; % accumAng(end) + dltang];
            % %         if norm(rodrigues(rotz(dltAng)) - rodrigues(dltr)) < norm(rodrigues(rotz(-dltAng)) - rodrigues(dltr))
            % %         if norm(deg2rad(dltAng).*rotaxis - rodrigues(dltr)) < norm(deg2rad(-dltAng).*rotaxis - rodrigues(dltr))
            % %             accumAng = [accumAng; accumAng(end) + dltAng];
            % %         else
            % %             accumAng = [accumAng; accumAng(end) - dltAng];
            % %         end
            
        end
    end
    
    figure,subplot(3,1,1);plot(accumAng,'-');title('vr');hold on; plot(idxAll2,'r');
    subplot(3,1,2);plot(dltDistVrTemp);title('before filter');
    subplot(3,1,3);plot(dltDistVRFiltTemp);title('after filter');hold on; plot(idxAll,'r');
    
    figure,plot(vrPath(:,1),vrPath(:,3));axis equal;hold on;plot([lenVR2(:,1);lenVR2(:,3)],[lenVR2(:,2);lenVR2(:,4)],'*r');
    for iii = 1 : length(idEval)
        text((lenVR2(iii,1)+lenVR2(iii,3))/2,(lenVR2(iii,2)+lenVR2(iii,4))/2,num2str(lenVR2(iii,5)), 'Color',[1 0 1],'FontSize',10,'FontWeight','bold');
    end
    
    if accumAng(end) < 0
        accumAng = -accumAng;
    end
    
    
    vrPosePoseMatStamp(:,end) = accumAng;
    vrPosePoseMatStamp(:,14) = vrPosePoseMatStamp(:,14) - vrPosePoseMatStamp(1,14);
    vrPosePoseMatStamp(:,14) = vrPosePoseMatStamp(:,14) + 0;  1/20;
    vrPosePoseMatStamp(:,15) = vrPosePoseMatStamp(:,15) - vrPosePoseMatStamp(1,15);
    save(fullfile(inputDir,sprintf('vrPoseStamp_%02d.mat',i)),'vrPosePoseMatStamp');
    
    
    
    %                     drawnow;
    savknj = 1;
    % %     LineBufStack = LineBufStack(1:end-1,:);
    % %     posestPoseMapped = [posestPoseMapped(:,2) zeros(size(posestPoseMapped,1),1) posestPoseMapped(:,1)];
    % %     [dltRobo] = diff(posestPoseMapped(1:end,1:3));
    % %     [~,lenRobo] = NormalizeVector(dltRobo);
    % %     idid = find(lenRobo > 5);
    % %
    % %
    % %     for k = 1 : size(posestPoseMapped,1)
    % %         RoboBufStack{k,:} = sprintf('Robot FID %d %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f\n',k,posestPoseMapped(k,:),posestPoseMapped(k,:));
    % %     end
    % %     vrPath(:,1) = -vrPath(:,1);
    % %     if draw == 1
    % %         figure,plot(posestPoseMapped(:,1),posestPoseMapped(:,3),'.');hold on; plot(vrPath(:,1),vrPath(:,3),'.');axis equal
    % %     end
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
    
    if 0
        fid = fopen(fullfile(inputDir, sprintf('comm_path_%02d.txt', i)),'w');
        cttVrThr = length(LineBufStack);
        cttRoboThr = length(RoboBufStack);
        cttVr = 1;
        cttRobo = 1;
        while cttVr <= cttVrThr
            for xx = 1 : 1
                try
                    fprintf(fid, strcat(LineBufStack{cttVr},'\n'));
                catch
                    break
                end
                cttVr = cttVr + 1;
            end
            if cttVr >1000
                try
                    fprintf(fid, strcat(RoboBufStack{cttRobo},'\n'));
                    cttRobo = cttRobo + 1;
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

end