function [T, roboMap, vrMap, roboPosePoseMatStamp,roboTimeStamp2,slamPosePoseMatStamp,slamTimeStamp2] = PlotRobotVR4(inputDir,startt,figNum)
dirInfo = dir(fullfile(inputDir,'comm_path_ceiling_*.txt'));
% close all
% TGood = 0;
% % figNum = 80;
% % % figure(3),clf;
useNewHua = 0;
draw = 0;1;0;1;0;1;
medFiltSize = 10;
% % flagStart = 0;
thrStart = 200; 20;100; 300;
thr = 5;
thrStillVR = 1; 2; 5; 10;
thrStillRobo = 3; 10;
stillNum =  3; 5; 10;
cnt = 0;
alignRobot = 1;

if alignRobot == 1;
    n2 = 11; n1 = 10;
else
    n1 = 11; n2 = 10;
end
for  i = startt : length(dirInfo)
    
    [pathFid, errMsg] = fopen(fullfile(inputDir,dirInfo(i).name));
    if (pathFid < 0)
        error('Cannot open %s for read: %s', fullfile(inputDir,dirInfo(i).name), errMsg);
    end
    pathBuf = [];
    inReadPath = false;
    
    
    roboTimeStamp2 = [];
    slamTimeStamp2 = [];
    
    
    
    % % flagg=0;
    roboPath = [];
    vrPath = [];
    vrPathTempAligned = [];
    TGood = 0;
    flagStart = 0;
    countVR = 0;
    countRobo = 0;
    stillVRGood = 0;
    stillRoboGood = 0;
    clear robopath
    cntVR = []; cntRobo = [];
    vrMatch = []; roboMatch = [];
    roboStamp = [];
    slamStamp = [];
    slamPoseMat = [];
    roboPoseMat = [];
    accumAngSlam = 0;
    accumAngRobo = 0;
    slamPath = [];
    % % figure,subplot(1,2,1);title('vr');hold on;
    % %        subplot(1,2,2);title('robot');hold on;
    if draw == 1
        figure(figNum),clf;subplot(1,2,2);hold on; plot(0,0,'.r'); plot(0,0,'.b'); title('vr + robot');axis equal;%% legend('vr','robot');
        subplot(1,2,1);hold on; axis equal;title('vr');
    end
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
        if strcmp(lineBuf(1:6), 'VR FID')
            flagg=1;
        end
        if strcmp(lineBuf(1:9), 'Robot FID')
            if alignRobot == 1
                flagg=2;
            else
                flagg=333;
            end
        end
        if strcmp(lineBuf(1:9), 'Robot Fit')
            robomatch = str2double(strsplit(lineBuf(16:end), ' '));%lineBuf
            roboMatch = [roboMatch; 1000.*robomatch];
            flagg = 3;
        end
        if strcmp(lineBuf(1:6), 'VR Fit')
            vrmatch = str2double(strsplit(lineBuf(13:end), ' '));%lineBuf
            vrMatch = [vrMatch; 1000.*vrmatch];
            flagg = 4;
        end
        if strcmp(lineBuf(1:2), 'Sl')
            if alignRobot == 1
                flagg=333;
            else
                flagg=2;
            end
            % %             flagg=2;
            
        end
        if strcmp(lineBuf(1:2), 'RT')
            rtList = str2double(strsplit(lineBuf(4:end), ' '));%lineBuf
            rt = reshape(rtList,3,4)';
            TClient = [rt(1:3,1:3) 1000.*rt(4,:)';0 0 0 1];
            % %             vrMatch = [vrMatch; 1000.*vrmatch];
            flagg = 5;
        end
        if flagg == 333
            slampath = str2double(strsplit(lineBuf(n1:end), ' '));
            
            if length(slampath) == 9
                ts3 = slampath(3);
                slampath = slampath([2 4:end]);
            else
                ts3 = slampath(1);
            end
            
            
            %             slampath(2:4) = [1000 1000 1000].*slampath(2:4);
            slampath(2:4) = 1000.*slampath(2:4);
            slampath(1) = slampath(1)/1000;
            try
                if length(slampath) ~= 7
                    sadvhjk;
                end
                rmatSlam = [[slampath(7) 0 -slampath(5)]' [0 1 0]' slampath(5:7)']';
                slamPoseMat = [slamPoseMat;[reshape(rmatSlam,1,9) slampath(2:4)]];
                
                %             camPoseMat = [camPoseMat;[reshape(R,1,9) t]];
                if size(slamPoseMat,1) > 1
                    dltTSlam =  [rmatSlam slampath(2:4)';0 0 0 1]*inv([reshape(slamPoseMat(end-1,1:9),3,3) slamPoseMat(end-1,10:12)';0 0 0 1]);
                    dltrSlam = dltTSlam(1:3,1:3);
                    dlttSlam = dltTSlam(1:3,end);
                    prvPtSlam = [slamPoseMat(end-1,10);0;slamPoseMat(end-1,12)];
                    curPtSlam = [slamPoseMat(end,10);0;slamPoseMat(end,12)];
                    dltAngSlam = rad2deg(norm(rodrigues(dltrSlam)));
                    errSlam = dltrSlam*prvPtSlam + dlttSlam - curPtSlam;
                    if norm(rodrigues(roty(dltAngSlam)) - rodrigues(dltrSlam)) < norm(rodrigues(roty(-dltAngSlam)) - rodrigues(dltrSlam))
                        accumAngSlam = [accumAngSlam; accumAngSlam(end) + dltAngSlam];
                    else
                        accumAngSlam = [accumAngSlam; accumAngSlam(end) - dltAngSlam];
                    end
                    
                end
                
                
                try
                    slamPath = [slamPath; slampath(2:end)];
                    slamStamp = [slamStamp; slampath(1)];
                    slamTimeStamp2 = [slamTimeStamp2;ts3];
                catch
                    wdlv = 1;
                end
            catch
                cwkj = 1;
            end
            
        end
        
        if flagg == 1
            vrpath = str2double(strsplit(lineBuf(8:end), ' '));%lineBuf
            
            vrpath(3) = 0;
            try
                vrpath(2:4) = [1000 1000 1000].*vrpath(2:4);
                if vrpath(2) < -206
                    wefgbhk = 1;
                end
                
                if isempty(vrPath)
                    if draw == 1
                        figure(figNum),subplot(1,2,1);plot(vrpath(2),vrpath(4),'sb','LineWidth',2);
                    end
                end
                
                if norm(vrpath([2 4]) - vrPath(end,[1 3])) > thr
                    if draw == 1
                        figure(figNum),subplot(1,2,1);plot(vrpath(2),vrpath(4),'.r');
                    end
                    drawVR = 1;
                else
                    drawVR = 0;
                end
                countVR = countVR + 1;
                if norm(vrpath(2:4) - vrPath(end,1:3)) < thrStillVR
                    cntVR = [cntVR;countVR];
                end
            catch
                evhoeq = 1;
                drawVR = 0;
            end
            try
                vrPath = [vrPath; vrpath(2:end)];
            catch
                wvhkj = 1;
            end
            falgg = 0;
        else
            drawVR = 0;
        end
        if flagg == 2
            robopath = str2double(strsplit(lineBuf(n2:end), ' '));%lineBuf
          
  %%
            if useNewHua == 1
                robopath(4) = -robopath(4);
            end
            
            if length(robopath) == 9
                ts2 = robopath(3);
                robopath = robopath([2 4:end]);
            else
                ts2 = robopath(1);
            end
            
            robopath(1) = robopath(1)/1000;
            try
                robopath(2:4) = 1000.*robopath(2:4);
                
                if length(robopath) ~= 7
                    ahvjk;
                end
                
                
                rmatRobo = [[robopath(7) 0 -robopath(5)]' [0 1 0]' robopath(5:7)']';
                roboPoseMat = [roboPoseMat;[reshape(rmatRobo,1,9) robopath(2:4)]];
                
                %             camPoseMat = [camPoseMat;[reshape(R,1,9) t]];
                if size(roboPoseMat,1) > 1
                    dltTRobo =  [rmatRobo robopath(2:4)';0 0 0 1]*inv([reshape(roboPoseMat(end-1,1:9),3,3) roboPoseMat(end-1,10:12)';0 0 0 1]);
                    dltrRobo = dltTRobo(1:3,1:3);
                    dlttRobo = dltTRobo(1:3,end);
                    prvPtRobo = [roboPoseMat(end-1,10);0;roboPoseMat(end-1,12)];
                    curPtRobo = [roboPoseMat(end,10);0;roboPoseMat(end,12)];
                    dltAngRobo = rad2deg(norm(rodrigues(dltrRobo)));
                    errRobo = dltrRobo*prvPtRobo + dlttRobo - curPtRobo;
                    if norm(rodrigues(roty(dltAngRobo)) - rodrigues(dltrRobo)) < norm(rodrigues(roty(-dltAngRobo)) - rodrigues(dltrRobo))
                        accumAngRobo = [accumAngRobo; accumAngRobo(end) + dltAngRobo];
                    else
                        accumAngRobo = [accumAngRobo; accumAngRobo(end) - dltAngRobo];
                    end
                    if size(roboPoseMat,1) ~= length(accumAngRobo)
                        sdvkj = 1;
                    end
                end
                
                
                
                
                if isempty(roboPath)
                    if draw == 1
                        figure(figNum),subplot(1,2,2);plot(robopath(2),robopath(4),'sb','LineWidth',2);
                    end
                end
                if norm(robopath(2:4) - roboPath(end,1:3)) > thr
                    % %                 subplot(1,2,2);plot(robopath(2),robopath(4),'.r');axis equal;drawnow;
                    drawRobo = 1;
                else
                    drawRobo = 0;
                end
                countRobo = countRobo + 1;
                if norm(robopath(2:4) - roboPath(end,1:3)) < thrStillRobo
                    cntRobo = [cntRobo;countRobo];
                end
            catch
                aghij = 1;
                drawRobo = 0;
            end
            try
                roboPath = [roboPath; robopath(2:end)];
                roboStamp = [roboStamp; robopath(1)];
                roboTimeStamp2 = [roboTimeStamp2; ts2];
            catch
                wghei = 1;
            end
            flagg = 0;
            
            % %         if norm(robopath(2:4) - [0 0 0]) > thrStart || flagStart == 1
            % %             T = AlignCoordSys(robopath(2:4)', vrpath(2:4)', vrPath(1,1:3)');
            % %             if flagStart == 0
            % %                 vrPathTemp = T(1:3,1:3)*[vrPath(:,1)';zeros(1,size(vrPath,1));vrPath(:,3)'] + repmat(T(1:3,4),1,size(vrPath,1));
            % %                 roboPathTemp = roboPath;
            % %             else
            % %                 vrPathTemp = T(1:3,1:3)*vrpath(2:4)' + T(1:3,4);
            % %                 roboPathTemp = robopath;
            % %             end
            % %
            % % %         if flagStart == 0
            % %             subplot(1,2,2);plot(vrPathTemp(1,:),vrPathTemp(3,:),'.r');axis equal;drawnow;
            % %                            plot(roboPathTemp(:,1),roboPathTemp(:,3),'.b');axis equal;drawnow;
            % %             flagStart = 1;
            % %         end
        else
            drawRobo = 0;
        end
        if exist('robopath','var')
            if norm(robopath(2:4) - [0 0 0]) > thrStart || flagStart == 1
                
                if TGood == 0;
                    [~,dltDistVR1] = NormalizeVector(vrPath(1:end-1,1:3) - vrPath(2:end,1:3));
                    try
                        [~,dltDistRobo1] = NormalizeVector(roboPath(1:end-1,1:3) - roboPath(2:end,1:3));
                    catch
                        vei = 1;
                    end
                    dltDistVR = medfilt1(dltDistVR1,(2*medFiltSize - 1));
                    dltDistRobo = medfilt1(dltDistRobo1,(2*medFiltSize - 1));
                    stillVR = find(dltDistVR <thrStillVR);
                    stillRobo = find(dltDistRobo <thrStillRobo);
                    if stillVRGood == 0
                        if length(stillVR) >= stillNum
                            % %                             if sum(dltDistVR1(end-stillNum+1:end) < thrStillVR) == stillNum
                            if sum(dltDistVR(end-stillNum+1:end) < thrStillVR) >= stillNum %  && norm(vrPath(1,1:3)-vrPath(end,1:3)) > 100 %% 20181204 
                                stillVRGood = 1;
                                vrpathTemp = vrPath(end-round(stillNum/2),:);
                            else
                                stillVRGood = 0;
                            end
                        else
                            stillVRGood = 0;
                        end
                    end
                    
                    if stillRoboGood == 0
                        if length(stillRobo) >= stillNum
                            if sum(dltDistRobo1(end-stillNum+1:end) < thrStillRobo) >= stillNum
                                stillRoboGood = 1;
                                robopathTemp = roboPath(end-round(stillNum/2),:);
                            end
                        end
                    end
                    [indexCellVRTemp,idxVRTemp] = splitIndex2(stillVR);
                    if length(indexCellVRTemp) == 1
                        indexCellVRTemp{2} = [1];
                    end
                    % %             if (flagStart == 0 || (~(length(indexCellVRTemp{2}) > 10 && length(stillRobo) > 10 && norm(robopath(2:4) - [0 0 0]) > 700))) && TGood == 0
                    if (flagStart == 0 || (~(stillVRGood == 1 && stillRoboGood == 1 && norm(robopath(2:4) - [0 0 0]) > 700))) && TGood == 0
                        %                 if ~(stillVRGood == 1 && stillRoboGood == 1)
                        %                     T = AlignCoordSys(robopath(2:4)', vrpath(2:4)', vrPath(1,1:3)');
                        %                 else
                        %                     T = AlignCoordSys(robopathTemp(1:3)', vrpathTemp(1:3)', vrPath(1,1:3)');
                        %                 end
                        if stillRoboGood == 1
                            roboCur = robopathTemp(1:3)';
                            %                             roboList = roboPath(1:end-round(stillNum/2),:);
                            roboPathh = roboPath(1:find(roboPath(:,1) == roboCur(1) & roboPath(:,3) == roboCur(3)),:);
                            roboInd = find(dltDistRobo1 > thrStillRobo);
                            roboInd = roboInd(roboInd <= size(roboPathh,1)-1);
                            % %                             roboList = roboPathh([roboInd; roboInd(end)+1],:);
                            % %                             roboList = roboPathh([roboInd(1):roboInd(end)+1],:);
                            try
                                %                                 roboList = roboPathh([1:roboInd(end)+1],:); % 20180719
                                roboList = roboPathh([1:roboInd(end)+1],:); % 20180719
                            catch
                                wvli = 1;
                            end
                            
                            roboList = roboList(roboList(:,3) ~= 0,:);
                            roboList = [0 0 0 0 0 1;roboList];
                            
                        else
                            roboCur = robopath(2:4)';
                            roboPathh = roboPath(1:find(roboPath(:,1) == roboCur(1) & roboPath(:,3) == roboCur(3)),:);
                            roboInd = find(dltDistRobo1 > thrStillRobo);
                            %                             roboList = roboPathh([roboInd; roboInd(end)+1],:);
                            try
                                roboList = roboPathh([1:roboInd(end)+1],:);
                            catch
                                wvkg = 1;
                            end
                            try
                                roboList = roboList(abs(roboList(:,3)) > 0.01,:);
                            catch
                                asfibhk = 1;
                            end
                            try
                                roboList = [0 0 0 0 0 1;roboList];
                            catch
                                awfvgoh = 1; %1
                            end
                            %                             roboList = roboPathh(1:end,:);
                            %                             roboList = roboPath(dltDistRobo1 > thrStillRobo,:);
                        end
                        if stillVRGood == 1
                            vrCur = vrpathTemp(1:3)';
                            %                             vrListTemp = vrPath(1:end-round(stillNum/2),:);
                            vrListTemp = vrPath(1:find(vrPath(:,1) == vrCur(1) & vrPath(:,3) == vrCur(3)),:);
                            [~,dltDistVR1Temp] = NormalizeVector(vrListTemp(1:end-1,1:3) - vrListTemp(2:end,1:3));
                            dltDistVRTemp = medfilt1(dltDistVR1Temp,(2*medFiltSize - 1));
                            %                             vrList = vrPath(dltDistVRTemp > thrStillVR,:);
                            vrInd = find(dltDistVRTemp > thrStillVR);
                            %                             vrList = vrListTemp([vrInd; vrInd(end)+1],:);
                            try
                                vrList = vrListTemp([1: vrInd(end)+1],:);
                            catch
                                dafgjl = 1;
                            end
                            ahi = 1;
                        else
                            vrCur = vrpath(2:4)';
                            %                             vrList = vrPath(1:end,:);
                            vrInd = find(dltDistVR > thrStillVR);
                            %                             vrList = vrPath([vrInd;vrInd(end)+1],:);
                            try
                                vrList = vrPath([1:vrInd(end)+1],:);
                            catch
%                                 vrList = vrPath;
                                sdlh = 1;
                            end
                        end
                        if 0
                            T = AlignCoordSys(roboCur, vrCur, vrPath(1,1:3)');
                        elseif 0
                            T = AlignCoordSys2(roboList, vrList,figNum);
                        else
                            [T, transformedVR] = AlignCoordSys3(roboList, vrList,figNum);
                        end
                        if isempty(T)
                            [T, transformedVR] = AlignCoordSys2(roboList, vrList,figNum);
                        end
                        eve = 1;
                        cnt = cnt + 1;
                        if cnt == 440
                            dbiqeb = 1;
                        end
                    else
                        if 0
                            T = AlignCoordSys(robopathTemp(1:3)', vrpathTemp(1:3)', vrPath(1,1:3)');
                        else
                            % %                             roboList = roboPath(1:end-round(stillNum/2),:);
                            % %                             vrList = vrPath(1:end-round(stillNum/2),:);
                            %                             roboList = roboPath(dltDistRobo1 > thrStillRobo,:);
                            %                              vrListTemp = vrPath(1:end-round(stillNum/2),:);
                            %                             [~,dltDistVR1Temp] = NormalizeVector(vrListTemp(1:end-1,1:3) - vrListTemp(2:end,1:3));
                            %                             dltDistVRTemp = medfilt1(dltDistVR1Temp,(2*medFiltSize - 1));
                            %                             vrList = vrPath(dltDistVRTemp > thrStillVR,:);
                            vrList = vrList;
                            roboList = roboList;
                            %                             vrList = vrPath(find(dltDistVR > thrStillVR) - 1,:);
                            
                            if 0
                                TT = AlignCoordSys2(roboList, vrList,figNum);
                            else
                                [TT, transformedVR] = AlignCoordSys3(roboList, vrList,figNum);
                            end
                            if isempty(TT)
                                [TT, transformedVR] = AlignCoordSys2(roboList, vrList,figNum);
                            end
                            regq3 = 1;
                            
                            if 0
                                [~,disVR] = NormalizeVector(vrMatch(1:end,1:3) - repmat(vrMatch(1,1:3),size(vrMatch,1),1));
                                [~,disRobo] = NormalizeVector(roboMatch(1:end,1:3) - repmat(roboMatch(1,1:3),size(roboMatch,1),1));
                                [roboMatch vrMatch];
                            end
                            
                            
                            if 0
                                % %                             t = roboList(end,1:3)' - TT(1:3,1:3)*vrList(end,1:3)';
                                t = robopathTemp(1:3)' - TT(1:3,1:3)*vrpathTemp(end,1:3)';
                                T = [TT(1:3,1:3) t;0 0 0 1];
                            else
                                T = TT;
                            end
                        end
                        
                        if 1
                            vrPathTemp1 = T(1:3,1:3)*vrpathTemp(1:3)' + T(1:3,4);
                            % %                 subplot(1,2,2);plot(robopath(:,2),robopath(:,4),'og','LineWidth',3);
                            if draw == 1
                                figure(figNum),subplot(1,2,2);plot(robopathTemp(1),robopathTemp(3),'og','LineWidth',3);
                                plot(vrPathTemp1(1),vrPathTemp1(3),'oy','LineWidth',3);
                                
                                figure(figNum),subplot(1,2,1);plot(vrpathTemp(1),vrpathTemp(3),'og','LineWidth',3);drawnow;
                            end
                        else
                            vrPathTemp1 = T(1:3,1:3)*vrList(end,1:3)' + T(1:3,4);
                            if vrPathTemp1(1) <  -206
                                aebqhi = 1;
                            end
                            if draw == 1
                                figure(figNum),subplot(1,2,2);plot(roboList(end,1),roboList(end,3),'og','LineWidth',3);
                                plot(vrPathTemp1(1),vrPathTemp1(3),'oy','LineWidth',3);
                                
                                figure(figNum),subplot(1,2,1);plot(vrList(end,1),vrList(end,3),'og','LineWidth',3);drawnow;
                            end
                            startPose = vrList(end,1:3);
                        end
                        TGood = 1;
                    end
                end
                
                if flagStart == 0
                    vrPathTemp = T(1:3,1:3)*[vrPath(:,1)';zeros(1,size(vrPath,1));vrPath(:,3)'] + repmat(T(1:3,4),1,size(vrPath,1));
                    roboPathTemp = roboPath;
                else
                    try
                        vrPathTemp = T(1:3,1:3)*vrpath(2:4)' + T(1:3,4);
                    catch
                        fvek = 1;
                    end
                    if vrPathTemp(1) <  -206
                        aebqhi = 1;
                    end
                    roboPathTemp = robopath;
                end
                
                %         if flagStart == 0
                % %             if (norm(robopath(2:4) - roboPath(end-1,1:3)) > thr) || (norm(vrpath(2:4) - vrPath(end-1,1:3)) > thr)
                
                if TGood == 0 % 0
                    
                    %                     figure(figNum),subplot(1,2,2);plot(vrPathTemp(1,:),vrPathTemp(3,:),'.r');
                    %                     vrPathTempAligned = [vrPathTempAligned vrPathTemp];
                    %                     plot(roboPathTemp(:,2),roboPathTemp(:,4),'.b');drawnow;
                    if draw == 1
                        figure(figNum),subplot(1,2,2);cla;plot(roboList(:,1),roboList(:,3),'.b');hold on;plot(transformedVR(1,:),transformedVR(3,:),'.r'); %% axis equal;
                        plot(roboList(1,1),roboList(1,3),'sb','LineWidth',2);
                        drawnow;
                    end
                    flefbk = 1;
                    
                else
                    
                    
                    if drawVR == 1 || drawRobo == 1
                        if draw == 1
                            figure(figNum),subplot(1,2,2);plot(vrPathTemp(1,:),vrPathTemp(3,:),'.r');
                        end
                        vrPathTempAligned = [vrPathTempAligned vrPathTemp];
                        if draw == 1
                            plot(roboPathTemp(:,2),roboPathTemp(:,4),'.b');drawnow;
                        end
                        
                    end
                end
                flagStart = 1;
            end
        end
        if size(vrPath,1) > 1000
            wdkhve = 1;
        end
        flagg = 0;
    end
    %     slamPosePoseMatStamp = [slamPoseMat [1:size(slamPoseMat,1)]' slamStamp+15 accumAngSlam];
    %     roboPosePoseMatStamp = [roboPoseMat [1:size(roboPoseMat,1)]' roboStamp+15 accumAngRobo];
    
    roboDisp = roboPoseMat(:,10:12);
    [~,roboDis] = NormalizeVector(roboDisp);
    findStartRobo = find(roboDis == 0);
    if isempty(findStartRobo)
        findStartRobo = 1;
    end
    
    
    try
        startTimeRobo = roboStamp(findStartRobo(end));
    catch
        dewkj = 1;
    end
    
    try
        slamDisp = slamPoseMat(:,10:12);
        [~,slamDis] = NormalizeVector(slamDisp);
        findStartSlam = find(slamDis == 0);
    catch
        findStartSlam = [];
    end
    if isempty(findStartSlam)
        findStartSlam = 1;
    end
    
    try
        startTimeSlam = slamStamp(findStartSlam(end));
    catch
        asdkjg = 1;
    end
    
    if 0 % accumAngRobo(end) < 0
        accumAngRobo = -accumAngRobo;
    end
    
    if 0 % accumAngSlam(end) < 0
        accumAngSlam = -accumAngSlam;
    end
    
    roboTimeStamp2 = roboTimeStamp2./1000;
    roboTimeStamp2 = roboTimeStamp2(findStartRobo(end):end,:)-roboTimeStamp2(findStartRobo(end),:);
    try
        slamTimeStamp2 = slamTimeStamp2./1000;
        slamTimeStamp2 = slamTimeStamp2(findStartSlam(end):end,:)-slamTimeStamp2(findStartSlam(end),:);
        
        
        
        
        slamPosePoseMatStamp = [slamPoseMat(findStartSlam(end):end,:) [findStartSlam(end):size(slamPoseMat,1)]' slamStamp(findStartSlam(end):end,:)-slamStamp(findStartSlam(end),:) accumAngSlam(findStartSlam(end):end,:)];
        slamPosePoseMatStamp(:,10:12) = slamPosePoseMatStamp(:,10:12) - repmat(slamPosePoseMatStamp(1,10:12),size(slamPosePoseMatStamp,1),1);
        
    catch
        slamPosePoseMatStamp = [];
        slamTimeStamp2 = [];
    end
    
    roboPosePoseMatStamp = [roboPoseMat(findStartRobo(end):end,:) [findStartRobo(end):size(roboPoseMat,1)]' roboStamp(findStartRobo(end):end,:)-roboStamp(findStartRobo(end),:) accumAngRobo(findStartRobo(end):end,:)];
    roboPosePoseMatStamp(:,10:12) = roboPosePoseMatStamp(:,10:12) - repmat(roboPosePoseMatStamp(1,10:12),size(roboPosePoseMatStamp,1),1);
    
    if useNewHua == 0
        roboPosePoseMatStamp(:,end) = -roboPosePoseMatStamp(:,end);
        slamPosePoseMatStamp(:,end) = -slamPosePoseMatStamp(:,end);
    end
    
    G=1;
    fclose(pathFid);
    
    
    roboMap = roboPath(:,1:3);
    vrMap = vrPath(:,1:3);
    
    
    
    
    [~,dltDistVR1] = NormalizeVector(vrPath(1:end-1,1:3) - vrPath(2:end,1:3));
    [~,dltDistRobo1] = NormalizeVector(roboPath(1:end-1,1:3) - roboPath(2:end,1:3));
    dltDistVR = medfilt1(dltDistVR1,(2*10 - 1));
    dltDistRobo = medfilt1(dltDistRobo1,(2*10 - 1));
    
    
    % [indexCellVR,idxVR] = splitIndex2(cntVR);
    % [indexCellRobo,idxRobo] = splitIndex2(cntRobo);
    [indexCellVR,idxVR] = splitIndex2(find(dltDistVR <thrStillVR));
    [indexCellRobo,idxRobo] = splitIndex2(find(dltDistRobo <thrStillRobo));
    
    
    lenVR = [];
    for ii = 1 : length(indexCellVR)-1
        indexPrv = indexCellVR{ii} + 0;
        indexCur = indexCellVR{ii+1} + 1;
        % %     len(ii,:) = [vrPath(round(median(indexPrv)),[1 3]) vrPath(round(median(indexCur)),[1 3]) norm(vrPath(round(median(indexPrv)),[1 3]) - vrPath(round(median(indexCur)),[1 3]))];
        lenVR(ii,:) = [vrPath(round(indexPrv(end)),[1 3]) vrPath(round(indexCur(1)),[1 3]) norm(vrPath(round(indexPrv(end)),[1 3]) - vrPath(round(indexCur(1)),[1 3]))];
    end
    lenRobo = [];
    for jj = 1 : length(indexCellRobo)
        
        if jj == 1
            indexPrv = [1];
        else
            indexPrv = indexCellRobo{jj-1} - 0;2;
        end
        indexCur = indexCellRobo{jj} + 1;
        % %     len(ii,:) = [vrPath(round(median(indexPrv)),[1 3]) vrPath(round(median(indexCur)),[1 3]) norm(vrPath(round(median(indexPrv)),[1 3]) - vrPath(round(median(indexCur)),[1 3]))];
        if jj == 1
            lenRobo(jj,:) = [[0 0] roboPath(round(indexCur(1)),[1 3]) norm([0 0] - roboPath(round(indexCur(1)),[1 3]))];
        else
            try
                lenRobo(jj,:) = [roboPath(round(indexPrv(end)),[1 3]) roboPath(round(indexCur(1)),[1 3]) norm(roboPath(round(indexPrv(end)),[1 3]) - roboPath(round(indexCur(1)),[1 3]))];
            catch
                evhk = 1;
            end
        end
    end
    if draw == 1
        figure(figNum),subplot(1,2,1),
        for iii = 1 : size(lenVR,1)
            text((lenVR(iii,1)+lenVR(iii,3))/2,(lenVR(iii,2)+lenVR(iii,4))/2,num2str(lenVR(iii,5)), 'Color',[1 0 1],'FontSize',10,'FontWeight','bold');
        end
        
        figure(figNum),subplot(1,2,2),
        for jjj = 1 : size(lenRobo,1)
            text((lenRobo(jjj,1)+lenRobo(jjj,3))/2,(lenRobo(jjj,2)+lenRobo(jjj,4))/2,num2str(lenRobo(jjj,5)), 'Color',[1 0 1],'FontSize',10,'FontWeight','bold');
        end
    end
    
    salbn = 1;
    
    if 0
        figure(figNum),subplot(2,2,1);plot(dltDistVR1);title('vr before');
        subplot(2,2,2);plot(dltDistRobo1);title('robot before');
        subplot(2,2,3);plot(dltDistVR);title('vr after');
        subplot(2,2,4);plot(dltDistRobo);title('robot after');
    end
    vrPathTempAligned = vrPathTempAligned';
    fidd = fopen(fullfile(inputDir,sprintf('aligned_vr_path_%02d.txt',i-1)),'w');
    
    for j = 1 : size(vrPathTempAligned,1)
        fprintf(fidd, '%0.2f %0.2f %0.2f', vrPathTempAligned(j,1),vrPathTempAligned(j,2),vrPathTempAligned(j,3));
        fprintf(fidd, '\n');
    end
    
    
    fclose(fidd);
    
    if ~exist('TClient')
        TClient = T;
    end
end

end