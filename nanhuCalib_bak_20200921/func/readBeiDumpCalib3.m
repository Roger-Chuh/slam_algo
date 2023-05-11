% % function [disp,accumAng,ind0,pt,data] = readBeiDumpCalib3(inputDir,paraDir,ang00, dist00,useLR)
function [camPosePath2,accumAng,disp] = readBeiDumpCalib3(inputDir,paraDir)    %   ,ang00, dist00)

% % ang0 = str2double(ang00);
% % dist0 = str2double(dist00);
forTest = 1;
minNumDist = 12;
minNum = 1;
marginInd = 2;
marginIndInd = 0;


window = 2;3; 2;3; 5; 10; 3; 5; 3;
window2 = 1; 3; 1;
window3 = 1;


scaleee = 0.5; 0.5;  1;0.5;  1;


% % close all
draw = 0;
lenDiff = 10; 30;
varThr = 5;

stright = 1;
MakeDirIfMissing(inputDir);

if forTest == 0
    if ~exist(fullfile(inputDir,'test.mp4'))
        try
            % %         movefile('D:\ForDump\apriltag\cmake-build-debug\test.log', inputDir);
            movefile('D:\ForDump\apriltag\bin\test.log', inputDir);
        catch
            ashkj = 1;
        end
        try
            % %         movefile('D:\ForDump\apriltag\cmake-build-debug\test.mp4', inputDir);
            movefile('D:\ForDump\apriltag\bin\test.mp4', inputDir);
        catch
            asgjokhk = 1;
        end
        try
            movefile('D:\ForDump\CommClient\comm_client_log.txt',inputDir);
        catch
            asdvbki = 1;
        end
        
    end
end

if 0
    [roboPath, roboPoseInfo1, roboPositiveFlag] = PlotRobotOnly(inputDir,1);
    [slamPath, slamPoseInfo1, slamPosetiveFlag] = PlotSlamOnly(inputDir,1);
    if roboPositiveFlag == 0
        roboPoseInfo1(:,end) = -roboPoseInfo1(:,end);
        slamPoseInfo1(:,end) = -slamPoseInfo1(:,end);
    end
    
    id1 = find(roboPoseInfo1(:,14) == 0);
    if isempty(id1)
        id1 = 1;
    end
    roboPoseInfo = roboPoseInfo1(id1(end):end,:);
    
    id2 = find(slamPoseInfo1(:,14) == 0);
    if isempty(id2)
        id2 = 1;
    end
    slamPoseInfo = slamPoseInfo1(id2(end):end,:);
    
end

% xyzCB = 1000.*[0, -0.079000004*2, 0;
%     0.079000004*2, -0.079000004*2, 0;
%     0.079000004*2, 0, 0;
%     0, 0, 0];
height = 165;125;  135;125;135; 160; 135; 0;125;135;  0; 135;125;132;100; 129;
heights = repmat(height,3,1);
% % heightOpt = [160.6092 159.0273 168.7507 169.6341];

try
    load(fullfile(paraDir, 'calib.mat'));
catch
    load(fullfile(inputDir, 'calib.mat'));
    paraDir = inputDir;
end



if forTest == 0
    % %     heightOpt = [ 158.7972 157.5447 167.6330 168.2161];
    heightOpt = [161.8151 160.6105 170.4220 171.3661];
    
else
    try
        load(fullfile(inputDir, 'heightOpt.mat'));
        heightOpt = calcHeight;
        calcHeight = [];
        load(fullfile(inputDir, 'offset.mat'));
        % %         xzOffsetOpt = [3.67836661170452,-1.38003182427598];
        
    catch
        skuj = 1;
    end
end


cbSize = 170.9539; 170.4792;  156.6186;   158; 156.6186; 158;158; 170.4792;
xyzCB = 1000.*[cbSize/2000, cbSize/2000, 0;
    cbSize/2000, -cbSize/2000, 0;
    -cbSize/2000, -cbSize/2000, 0;
    -cbSize/2000, cbSize/2000, 0];
xyzCB(:,end) = 1;
% % load(fullfile(paraDir, 'calib.mat'));
% try
%     load(fullfile(paraDir, 'calib.mat'));
% catch
%     load(fullfile(inputDir, 'calib.mat'));
%     paraDir = inputDir;
% end


intrMatRight = [camParam.foc(1),0,camParam.cen(1); 0,camParam.foc(2),camParam.cen(2); 0,0,1];
k1 = camParam.kc(1);
k2 = camParam.kc(2);
k3 = camParam.kc(5);
p1 = camParam.kc(3);
p2 = camParam.kc(4);
load(fullfile(paraDir,'oCamModel.mat'));
% %     oCamModel = calib_data.ocam_model;
U_same = ocam_undistort_map(oCamModel,'OutputView','same');
U_full = ocam_undistort_map(oCamModel,'OutputView','full');
intrMat_same = U_same.K';
intrMat_full = U_full.K';
intrMatRight = intrMat_same;

try
    [H, camPoseVec, metricList1, ptUndistHomo, intrMatHomo] = calibHomo2('D:\Temp\20180809\calibFishEyeHomo1', 'E:\bk_20180627\pc\Temp\20180724\calibFishEye1');
catch
    % %     [H, camPoseVec, metricList1, ptUndistHomo, intrMatHomo] = calibHomo2(inputDir, 'E:\bk_20180627\pc\Temp\20180724\calibFishEye1');
    [H, camPoseVec, metricList1, ptUndistHomo, intrMatHomo] = calibHomo2(paraDir, 'E:\bk_20180627\pc\Temp\20180724\calibFishEye1');
end
[sol1, H1] = DecomposeHomoGraphyMat(H, eye(3), eye(3));


for e = 1 : 4
    rTmp = sol1(1:3,1:3,e);
    tTmp = sol1(1:3,4,e);
    projErr1(e,1) = ProjectErr3dTo2d(ptUndistHomo', metricList1, intrMatHomo, rTmp, tTmp);
end
[~, idH1] = min(projErr1);
RHomo1 = sol1(1:3,1:3,idH1);
tHomo1 = sol1(1:3,4,idH1);
NHomo1 = sol1(1:3,5,idH1);
T1 = [RHomo1 tHomo1; 0 0 0 1];


Cen2CenDist = norm(tHomo1);


[sol, H2] = DecomposeHomoGraphyMat((inv(H)), eye(3), eye(3));
angThr = 2;


for e = 1 : 4
    rTmp = sol(1:3,1:3,e);
    tTmp2 = sol(1:3,4,e);
    scale = Cen2CenDist./norm(tTmp2);
    tTmp = scale.*tTmp2;
    invT = inv([rTmp tTmp;0 0 0 1]);
    
    projErrrr(e,1) = ProjectErr3dTo2d(ptUndistHomo', metricList1, intrMatHomo, invT(1:3,1:3), invT(1:3,4));
end
projErr = ProjectErr3dTo2d(ptUndistHomo', metricList1, intrMatHomo, rodrigues(camPoseVec(1:3)), camPoseVec(4:6));
[~, idH] = min(projErrrr);
RHomo = sol(1:3,1:3,idH);
NHomo = sol(1:3,5,idH);
ground = [NHomo; -Cen2CenDist./norm(sol(1:3,1:3,idH))];
tHomo = Cen2CenDist./norm(sol(1:3,4,idH)).*sol(1:3,4,idH);

THomo = [RHomo tHomo;0 0 0 1];
THomoInv = inv(THomo);  % gnd2ceiling

Ngnd2c = THomoInv(1:3,1:3)*[0;0;1];
% %         ground = [NHomo; -Cen2CenDist./norm(tHomo)];
ground = [NHomo; -(norm(mean(THomo(1:3,4)./sol(1:3,4,idH))) - height)];
% % % % % %         ground1 = [RHomo(3,:)'; -norm(THomo(3,4))];

dist2Gnd = norm(mean(THomo(1:3,4)./sol(1:3,4,idH)));
err = ground'*[THomoInv(1:3,4);1];






dirInfo = dir(fullfile(inputDir,'*.log'));

try
    log = load(fullfile(inputDir, dirInfo(1).name));
catch
    log = readLog(fullfile(inputDir, dirInfo(1).name));
end
log(:,8:end) = log(:,8:end)./scaleee;


% % figure,imshow(zeros(1080,1920));hold on;
% % if 0
for i = 1 : size(log,1)
    r = rodrigues(log(i,2:4));
    camPath2(i,:) =  [r(:)' 1000.*log(i,5:7) log(i,1)];
    
    
    xList(i,:) = log(i,8:2:end);
    yList(i,:) = log(i,9:2:end);
    if i > 1
        [~,Dist(i-1,:)] = NormalizeVector([xList(i,:) - xList(i-1,:);yList(i,:) - yList(i-1,:)]');
    end
    if 1
        iPt = [xList(i,:); yList(i,:)]';
        [ptUndist, ptUndist3D] = remapFishEyePix(iPt(:,[1 2])', intrMatRight, oCamModel);
        ptUndist3D_ = ptUndist3D';
        ptUndistRight = ptUndist3D(1:2,:);
        
        initIntersectPt = backProjPix2SpacePlane(ptUndist3D',ground);
        
        initIntersectPt = [];
        % %         initIntersectPt(1,:) = backProjPix2SpacePlane(ptUndist3D(:,1)',[ground(1:3);heightOpt(1)]);
        initIntersectPt(1,:) = backProjPix2SpacePlane(ptUndist3D(:,1)',[ground(1:3);-(dist2Gnd - heightOpt(1))]);
        initIntersectPt(2,:) = backProjPix2SpacePlane(ptUndist3D(:,2)',[ground(1:3);-(dist2Gnd - heightOpt(2))]);
        initIntersectPt(3,:) = backProjPix2SpacePlane(ptUndist3D(:,3)',[ground(1:3);-(dist2Gnd - heightOpt(3))]);
        initIntersectPt(4,:) = backProjPix2SpacePlane(ptUndist3D(:,4)',[ground(1:3);-(dist2Gnd - heightOpt(4))]);
        
        
        
        
        
        
        initIntersectPtRnd = (THomo(1:3,1:3)*initIntersectPt' + repmat(THomo(1:3,4),1, size(initIntersectPt,1)))';
        initIntersectPtRnd(:,end) = 1;
        if 1
            len(i,1) = norm(initIntersectPtRnd(1,:) - initIntersectPtRnd(2,:));
            len(i,2) = norm(initIntersectPtRnd(3,:) - initIntersectPtRnd(2,:));
            len(i,3) = norm(initIntersectPtRnd(3,:) - initIntersectPtRnd(4,:));
            len(i,4) = norm(initIntersectPtRnd(1,:) - initIntersectPtRnd(4,:));
        end
        erwbgv = 1;
        
        % %         rt = AlignCoord(initIntersectPtRnd,xyzCB);
        % %         err = rt(1:3,1:3)*xyzCB' + repmat(rt(:,end),1,4) - initIntersectPtRnd';
        % %         [~,err2] = NormalizeVector(err');
        % %         Err(i,:) = err2';
        % %         camPosePath2(i,:) = [reshape(rt(:,1:3),1,9) rt(:,end)'];
        
        
        
        
        
        
        % % %     if i > 10
        % % %         ptUndistRight = normalize_pixel([log(i,8:2:end);log(i,9:2:end)],camParam.foc,camParam.cen,camParam.kc,0);
        % % %         ptUndist = intrMatRight*[ptUndistRight;ones(1,size(ptUndistRight,2))];
        % % %         ptUndist = [ptUndist(1,:)./ptUndist(3,:);ptUndist(2,:)./ptUndist(3,:)];
        % % %         ptUndist3D = [ptUndistRight; ones(1,size(ptUndistRight,2))]';
        % % %
        % % %         mappedPtMetric = H*ptUndist3D';
        % % %         mappedPtMetric = [mappedPtMetric(1,:)./mappedPtMetric(3,:);mappedPtMetric(2,:)./mappedPtMetric(3,:);ones(1,size(mappedPtMetric,2))];
        % % %
        % % %
        % % % % %         [Rstereo,Tstereo,Nstereo,~] = GetTrueRT('stereo',sol, mappedPtMetric', [],[],[]);
        % % %         [Rstereo,Tstereo,Nstereo,~] = GetTrueRT('stereo',sol, camPath2(1:i,10:12), [],[],[]);
        % % %         d = baseline/norm(Tstereo);
        % % %         ground = [Nstereo; -d];
        % % %     end
        
        
        % %     if i > 1
        % %         Dist(i-1,:) = NormalizeVector(xList(i,:) - xList(i-1,:));
        % %     end
        
        % %     plot(xList(i,:),yList(i,:),'-');plot(xList(i,1),yList(i,1),'*b');drawnow;
    end
    
end
dist = mean(Dist')';
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

Indd = (find(abs(mean(len') - cbSize) < lenDiff & var(len') < varThr))';
if 0
    Ind = intersect(Ind,Indd);
else
    Ind = Indd;
end

xListt = xList(Ind,:);
yListt = yList(Ind,:);
pixErr = mean(sqrt(diff(xListt).^2 + diff(yListt).^2)')';
pixErrFilt = medfilt1(pixErr,(2*window - 1));

%%
% % % % % % % dltDistRoboTempNewAng = abs(diff(accumAng));
% % % % % % % dltDistRoboFiltTempNewAng = medfilt1(dltDistRoboTempNewAng,(2*window - 1));
% % % % % % %
% % % % % % % if 1
% % % % % % %     dltDistRoboTempNew = pixErr; % dist;
% % % % % % %     dltDistRoboFiltTempNew = pixErrFilt; % medfilt1(dltDistRoboTempNew,(2*window - 1));
% % % % % % % else
% % % % % % %     dltDistRoboFiltTempNew = dist;
% % % % % % % end
% % % % % % %
% % % % % % % %     dltDistRobo = medfilt1(dltDistRobo1,(2*10 - 1));
% % % % % % % [indexCellRoboNew1111,idxRoboNew] = splitIndex2(find(dltDistRoboFiltTempNew > 0.7));
% % % % % % %
% % % % % % % validInd = [];
% % % % % % % indexCellRoboNew11112 = {};cttt = 1;plusInd = 0;
% % % % % % % for y = 1 : length(indexCellRoboNew1111)
% % % % % % %     diffAngDistribution(y,1) = median(dltDistRoboFiltTempNewAng(indexCellRoboNew1111{y}));
% % % % % % %
% % % % % % %     if length(indexCellRoboNew1111{y}) > minNumDist
% % % % % % %         validInd = [validInd; y];
% % % % % % %         indexCellRoboNew11112{cttt,1} = cell2mat(indexCellRoboNew1111(y+plusInd :y,1)');
% % % % % % %         cttt = cttt + 1;
% % % % % % %     else
% % % % % % %         slv = 1;
% % % % % % %         if median(dltDistRoboFiltTempNewAng(indexCellRoboNew1111{y})) > 1
% % % % % % %             if median(dltDistRoboFiltTempNewAng(indexCellRoboNew1111{y-1})) < 0.5
% % % % % % %                 plusInd = -1;
% % % % % % %             end
% % % % % % %             % %             if median(dltDistRoboFiltTempNewAng(indexCellRoboNew1111{y-1})) < 0.5
% % % % % % %             % %                 plus = 1;
% % % % % % %             % %             end
% % % % % % %         end
% % % % % % %     end
% % % % % % % end

%%




figure,plot(len(Ind,:))

camPath3 = camPath2(Ind,:);
ang = rad2deg(norm(rodrigues(reshape(camPath3(end,1:9),3,3)'*reshape(camPath3(1,1:9),3,3))));


timeStamp = camPath3(:,end)./1000;

log3 = log(Ind,:);

% xzOffsetOpt = [3.67836661170452,-1.38003182427598];
[camPosePath2,Err,len1,accumAng] = costFunc3(xyzCB,log3,xzOffsetOpt,intrMatRight,oCamModel,ground,THomo,heightOpt,dist2Gnd);

[~,disp] = NormalizeVector(camPosePath2(1:end,10:12) - repmat(camPosePath2(1,10:12),size(camPosePath2,1),1));


jahs = 1;

if 0
    %%
    if ang0 == 0
        dltDistRoboTempNewAng = abs(diff(accumAng));
        dltDistRoboFiltTempNewAng = medfilt1(dltDistRoboTempNewAng,(2*window - 1));
        
        if 1
            dltDistRoboTempNew = pixErr; % dist;
            dltDistRoboFiltTempNew = pixErrFilt; % medfilt1(dltDistRoboTempNew,(2*window - 1));
        else
            dltDistRoboFiltTempNew = dist;
        end
        
        %     dltDistRobo = medfilt1(dltDistRobo1,(2*10 - 1));
        [indexCellRoboNew1111,idxRoboNew] = splitIndex2(find(dltDistRoboFiltTempNew > 0.7));
        
        validInd = [];
        indexCellRoboNew11112 = {};cttt = 1;plusInd = 0;
        for y = 1 : length(indexCellRoboNew1111)
            diffAngDistribution(y,1) = median(dltDistRoboFiltTempNewAng(indexCellRoboNew1111{y}));
            if 0
                if length(indexCellRoboNew1111{y}) > minNumDist
                    validInd = [validInd; y];
                    indexCellRoboNew11112{cttt,1} = cell2mat(indexCellRoboNew1111(y+plusInd :y,1)');
                    cttt = cttt + 1;
                else
                    slv = 1;
                    if median(dltDistRoboFiltTempNewAng(indexCellRoboNew1111{y})) > 1
                        try
                            if median(dltDistRoboFiltTempNewAng(indexCellRoboNew1111{y-1})) < 0.5
                                plusInd = -1;
                            end
                        catch
                            aivhk = 1;
                        end
                        
                        % %             if median(dltDistRoboFiltTempNewAng(indexCellRoboNew1111{y-1})) < 0.5
                        % %                 plus = 1;
                        % %             end
                    end
                end
            end
        end
        
        
        [indexCellDistr,idxDistr] = splitIndex2(find(diffAngDistribution > 1));
        
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
        
        
        indexCellRoboNew = indexCellRoboNew11(id,:);
        
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
        
        
        
        IndRange1 = [];
        for q = 1 : length(indexCellRoboNew)
            
            if q >= 45
                askh = 1;
            end
            indRange1 = indexCellRoboNew{q,1}([1 end]);
            % %     indRange1Ang = [indRange1(1) - marginInd min(indRange1(2) + marginInd + marginIndInd, size(camPoseMatNew,1))];% loose
            % %     indRange1Dist = [min(indRange1(1) + marginInd, size(camPoseMatNew,1)) min(indRange1(2) - marginInd - marginIndInd, size(camPoseMatNew,1))];%tight
            
            indRange1Ang = [indRange1(1) - marginInd - 0 min(indRange1(2) + marginInd + marginIndInd, size(camPosePath2,1))];% loose
            if 0
                indRange1Dist = [min(indRange1(1) + marginInd +0, size(camPoseMatNew,1)) min(indRange1(2) - marginInd - marginIndInd, size(camPosePath2,1))];%tight
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
                DistAng1(q,1) = norm(camPosePath2(indRange1Dist(1),[10:12]) - camPosePath2(indRange1Dist(2),[10:12]));
            catch
                askj = 1;
            end
            % % %     DistAng1(q,2) = accumAng(indRange1Ang(2)) - accumAng(indRange1Ang(1));
            if q > 1
                DistAng1(q,2) = accumAng(indRange1Ang(2)) - accumAng(indRange1Ang(1));
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
                    DistAng2(q,1) = norm(camPosePath2(indRange2(1),[10:12]) - camPosePath2(indRange2(2),[10:12]));
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
                    DistAng2(j,1) = norm(camPosePath2(indRange2Dist(1),[10:12]) - camPosePath2(indRange2Dist(2),[10:12]));
                catch
                    qwk = 1;
                end
                try
                    DistAng2(j,2) = accumAng(indRange2Ang(2)) - accumAng(indRange2Ang(1));
                catch
                    awkj = 1;
                end
                
                
                
            end
        end
        indxAng = IndRange2(:,1:2);indxAng = indxAng(:);
        indxAng = unique(indxAng);
        
        indxDist = IndRange2(:,3:4);indxDist = indxDist(:);
        indxDist = unique(indxDist);
        
        figure,subplot(3,1,1);plot(timeStamp,accumAng,'-');title('ang');hold on;plot(timeStamp(indxAng), accumAng(indxAng),'or');plot(timeStamp(indxDist), accumAng(indxDist),'*g'); legend('curve','ang','dist');%plot(idxAll3,'r');
        subplot(3,1,2);plot(timeStamp,camPosePath2(:,10));title('x');hold on;plot(timeStamp(indxAng), camPosePath2(indxAng,10),'or');plot(timeStamp(indxDist), camPosePath2(indxDist,10),'*g');  legend('curve','ang','dist');%plot(idxAll4,'r');
        subplot(3,1,3);plot(timeStamp,camPosePath2(:,11));title('z');hold on;plot(timeStamp(indxAng), camPosePath2(indxAng,11),'or'); plot(timeStamp(indxDist), camPosePath2(indxDist,11),'*g');legend('curve','ang','dist');%plot(idxAll5,'r');
        figure,plot(camPosePath2(:,10),camPosePath2(:,11),'-');hold on;plot(camPosePath2(indxAng,10),camPosePath2(indxAng,11),'or');plot(camPosePath2(indxDist,10),camPosePath2(indxDist,11),'*g');axis equal;legend('curve','ang','dist');
        
        data = DistAng2(1:end-1,:);
    else
        data = [];
    end
    
    %%
    
    
    
    
    
    
    
    % % disp = disp-disp(1);
    % % disp = abs(disp);
    % % disp = [0;disp];
    dltAng = abs(diff(accumAng));
    dltAngFilt = medfilt1(dltAng,(2*2 - 1));
    [indexCell1, index] = splitIndex2(find(pixErrFilt < 0.1));
    
    
    if dist0 == 0
        
        diffAngFilt = medfilt1(abs(diff(accumAng)),2*window-1);
        % % % [indexCell, index] = splitIndex2(find(diffAngFilt < 0.1));
        % % [indexCell, index] = splitIndex2(find(abs(diff(accumAng))./diff(timeStamp) < 0.1));
        [indexCell, index] = splitIndex2(find(medfilt1(diffAngFilt./diff(timeStamp),2*(window + window2)-1) < 1.5));  % 1
        
    end
    
    
    for y = 1 : length(indexCell1)
        % %     diffAngDistribution(y,1) = median(dltAngFilt(indexCell{y}));
        diffAngDistribution(y,1) = median(pixErrFilt(indexCell1{y}));
    end
    
    
    if 0
        [indexCellDistr,idxDistr] = splitIndex2(find(diffAngDistribution < 0.03));
        
        
        for i = 1 : length(indexCellDistr)
            iiid = indexCellDistr{i};
            indexCell{i,1} = cell2mat(indexCell1(iiid,1)');
        end
    else
        indexCell = indexCell1(find(diffAngDistribution < 0.02),1);
    end
    
    
    diffAngFilt = medfilt1(abs(diff(accumAng)),2*window-1);
    [indexCell, index] = splitIndex2(find(diffAngFilt < 0.1));
    % % [indexCell, index] = splitIndex2(find(abs(diff(accumAng))./diff(timeStamp) < 0.1));
    [indexCell, index] = splitIndex2(find(medfilt1(diffAngFilt./diff(timeStamp),2*(window + window2)-1) < 1.5));  % 1
    
    % [indexCellRobo, indexRobo] = splitIndex2(find(diff(roboPoseInfo(:,14)) < 0.015));
    % % [indexCellRobo, indexRobo] = splitIndex2(find(diff(roboPoseInfo(:,14)) < 0.7)); % 0.2
    % % [indexCellRobo, indexRobo] = splitIndex2(find(diff(roboPoseInfo(:,14)) < 1)); % 0.2
    try
        [indexCellRobo, indexRobo] = splitIndex2(find(diff(roboPoseInfo(:,14))./diff(roboPoseInfo(:,13)) < 1)); % 0.2
        tmp = medfilt1(diff(roboPoseInfo(:,14))./diff(roboPoseInfo(:,13)),2*window3-1); %diff(roboPoseInfo(:,14))./diff(roboPoseInfo(:,13));
        [minv,mini1]=findpeaks(-tmp,'minpeakdistance',1);
        mini2 = find(tmp(mini1) < 2);
        mini = mini1(mini2);
    catch
        awuoi = 1;
    end
    % figure,plot(tmp);hold on;plot(mini,tmp(mini),'*g');
    
    % % figure,
    
    try
        for i = 1 : length(indexCell)
            id(i,1) = round(mean(indexCell{i}));
        end
        % %     id  = mini;
        
        for i = 1 : length(indexCellRobo)
            idRobo(i,1) = round(mean(indexCellRobo{i}));
        end
        idRobo = mini;
        ceilingRange1 = [id(1:end-1) id(2:end)];
        roboRange1 = [[1;idRobo(1:end-1)] idRobo];
        
        pulse = zeros(length(pixErrFilt),1);
        if 0
            pulse(id) = max(diffAngFilt);
            figure,plot(diffAngFilt);hold on;plot(pulse);
        else
            pulse(id) = max(pixErrFilt);
            difCeilingAng = medfilt1(diffAngFilt./diff(timeStamp),2*(window+window2)-1); % pixErrFilt;
            if 0
                % %             figure,plot(pixErrFilt);hold on;plot(pulse);
                %             figure,plot(pixErrFilt);hold on;plot(find(pulse)-1,difCeilingAng(find(pulse)-1),'*g');
                figure,plot(difCeilingAng);hold on;plot(find(pulse)-1,difCeilingAng(find(pulse)-1),'*g');
            end
        end
        pulseRobo = zeros(length(diff(roboPoseInfo(:,14))),1);
        pulseRobo(idRobo) = max(diff(roboPoseInfo(:,14)));
        % %     difRoboAng = medfilt1(diffAngFilt./diff(timeStamp),2*(window+window2)-1); %diff(roboPoseInfo(:,14));
        difRoboAng = diff(roboPoseInfo(:,14));
        if 0
            
            % %             figure,plot(diff(roboPoseInfo(:,14)));hold on;plot(pulseRobo);
            
            % % %        figure,plot(diff(roboPoseInfo(:,14)));hold on;plot(find(pulseRobo)-1,difRoboAng(find(pulseRobo)-1),'*g');
            
            figure,plot(tmp);hold on;plot(mini,tmp(mini),'*g');
            
            % %         figure,plot(diff(roboPoseInfo(:,14)));hold on;plot(idRobo,roboPoseInfo(idRobo,14)),'*g';
        end
        % %     figure,plot(roboPoseInfo(1:end-1,13),diff(roboPoseInfo(:,14)));hold on;plot(roboPoseInfo(1:end-1,13),pulseRobo);
        
    catch
        sadfkhj = 1;
    end
    
    
    if 0
        figure,plot(disp,accumAng,'-');
        figure,plot(len1);
    end
    if 0
        figure,plot(timeStamp, accumAng);hold on;
        try
            plot(timeStamp(id), accumAng(id),'*g','MarkerSize',5,'LineWidth',5);
        catch
            askhlj = 1;
        end
        if accumAng(end) > 0
            plot(roboPoseInfo(:,13),roboPoseInfo(:,14),'-x');
        else
            plot(roboPoseInfo(:,13),-roboPoseInfo(:,14),'-x');
        end
        try
            if accumAng(end) > 0
                plot(roboPoseInfo(idRobo,13),roboPoseInfo(idRobo,14),'*b','MarkerSize',5,'LineWidth',5);
            else
                plot(roboPoseInfo(idRobo,13),-roboPoseInfo(idRobo,14),'*b','MarkerSize',5,'LineWidth',5);
            end
        catch
            asflih = 1;
        end
        if accumAng(end) > 0
            plot(slamPoseInfo(:,13),slamPoseInfo(:,14),'-x');
        else
            plot(slamPoseInfo(:,13),-slamPoseInfo(:,14),'-x');
        end
        try
            if accumAng(end) > 0
                plot(slamPoseInfo(idRobo,13),slamPoseInfo(idRobo,14),'*r','MarkerSize',5,'LineWidth',5);
            else
                plot(slamPoseInfo(idRobo,13),-slamPoseInfo(idRobo,14),'*r','MarkerSize',5,'LineWidth',5);
            end
        catch
            asvlkh = 1;
        end
    end
    % % % % figure,plot(camPosePath2(:,10),camPosePath2(:,11));hold on;plot(camPosePath2(1,10),camPosePath2(1,11),'*');axis equal
    if 0
        fig_3D3_pair(camPosePath2(:,10:12)',camPosePath2(:,10:12)');
    end
    if 1
        figure,plot3(camPosePath2(:,10),camPosePath2(:,11),camPosePath2(:,12),'-');hold on;plot3(camPosePath2(1,10),camPosePath2(1,11),camPosePath2(1,12),'*','MarkerSize',5,'LineWidth',4);plot3(camPosePath2(end,10),camPosePath2(end,11),camPosePath2(end,12),'*','MarkerSize',5,'LineWidth',4);plot3(camPosePath2(round(size(camPosePath2,1)/2),10),camPosePath2(round(size(camPosePath2,1)/2),11),camPosePath2(round(size(camPosePath2,1)/2),12),'*','MarkerSize',5,'LineWidth',4);axis equal;xlabel('X (mm)');ylabel('Y (mm)'); zlabel('Z (mm)');
        
        %     pt = [camPosePath2(:,10),camPosePath2(:,11); ;camPosePath2(end,10),camPosePath2(end,11)]
        
    end
    
    
    if 1  %straight == 1
        reportVec = [ang0 roboPoseInfo(end,end) accumAng(end); dist0 norm(roboPoseInfo(end,10:12)) disp(end)];
    else
        reportVec = [roboPoseInfo(end,end) accumAng(end); norm(roboPoseInfo(end,10:12)) disp(end)];
    end
    try
        
        for tt = 1 : size(ceilingRange1,1)
            angMat1(tt,:) = abs(accumAng(ceilingRange1(tt,1))-accumAng(ceilingRange1(tt,2)));
        end
        for tt = 1 : size(roboRange1,1)
            angMat2(tt,:) = abs(roboPoseInfo(roboRange1(tt,1),14)-roboPoseInfo(roboRange1(tt,2),14));
        end
        
        ceilingRange = ceilingRange1(angMat1(:,1)>2,:);
        roboRange = roboRange1(angMat2(:,1)>2,:);
        try
            for t = 1 : min(size(ceilingRange,1),size(roboRange,1))
                angMat(t,:) = [abs(accumAng(ceilingRange(t,1))-accumAng(ceilingRange(t,2))) abs(roboPoseInfo(roboRange(t,1),14)-roboPoseInfo(roboRange(t,2),14)) abs(slamPoseInfo(roboRange(t,1),14)-slamPoseInfo(roboRange(t,2),14))];
                
                
            end
            angMat = [angMat;sum(angMat)];
            if 0
                figure,hist(angMat(1:end-1,1:3),40);title(sprintf( 'expected angle: %d degree\nceiling %f.2\nwheel %f.2\nvisoin %f.2\n',ang0,angMat(end,1),angMat(end,2),angMat(end,3))); legend('ceiling','wheel','vision');%num2str(ang0));
            end
            % % %     saveas(gcf,fullfile(inputDir,strcat('angDistribution.png')));
            
            if 0
                fid2 = fopen(fullfile(inputDir,'report_angle.txt'),'w');%????
                for u = 1 : size(angMat,1)
                    if 1 %u <= size(staightness,1)
                        fprintf(fid2,'%06f  %06f  %06f\n',angMat(u,1),angMat(u,2),angMat(u,3));
                        % % %         elseif u == size(staightness,1) + 1
                        % % %             fprintf(fid1,'#########################\n');
                        % % %         else
                        % % %             fprintf(fid1,'%06f  %06f  %06f  %06f\n',reportVec(u-1,1),reportVec(u-1,2),reportVec(u-1,3),reportVec(u-1,4));
                    end
                    
                end
                fclose(fid2);
            end
        catch
            dsghj = 1;
        end
        
    catch
        asods = 1;
    end
    
    fid1 = fopen(fullfile(inputDir,'report.txt'),'w');%????
    for u = 1 : size(reportVec,1)
        if 1 %u <= size(staightness,1)
            fprintf(fid1,'%06f  %06f  %06f\n',reportVec(u,1),reportVec(u,2),reportVec(u,3));
            % % %         elseif u == size(staightness,1) + 1
            % % %             fprintf(fid1,'#########################\n');
            % % %         else
            % % %             fprintf(fid1,'%06f  %06f  %06f  %06f\n',reportVec(u-1,1),reportVec(u-1,2),reportVec(u-1,3),reportVec(u-1,4));
        end
        
    end
    
    fclose(fid1);
    
    try
        if 0
            figure,plot(angMat(1:end-1,:))
            figure,plot(angMat(1:end-1,3) - angMat(1:end-1,1));
            figure,hist(angMat(1:end-1,3) - angMat(1:end-1,1),50);
        end
    catch
        qeskhjg = 1;
    end
    
    diffAccumAng = abs(medfilt1(abs(diff(accumAng)),2*2-1));
    figure,plot(diffAccumAng);
    [indexCell0,index0] = splitIndex2(find(diffAccumAng < 0.2));
    
    
    try
        
        
        for i = 1 : length(indexCell0)-1
            accumAngMat(i,:) =  abs(accumAng(round(mean(indexCell0{i}))) - accumAng(round(mean(indexCell0{i+1}))));
        end
        mean(accumAngMat)
        figure,plot(accumAngMat);
        
        if useLR == 1
            ind0 = [round(mean(indexCell0{2})) round(mean(indexCell0{((length(indexCell0)+1)/2)/2+1})) round(mean(indexCell0{(length(indexCell0)+1)/2}))  round(mean(indexCell0{(length(indexCell0)+1)/2 + 1})) round(mean(indexCell0{((length(indexCell0)+1)/2)/2+1-2 + (length(indexCell0)+1)/2 + 1}))    round(mean(indexCell0{end}))];
            % % figure,plot(accumAng)
            
            pt = [camPosePath2(ind0(1),10),camPosePath2(ind0(1),11); camPosePath2(ind0(3),10),camPosePath2(ind0(3),11);camPosePath2(ind0(end),10),camPosePath2(ind0(end),11)];
        else
            ind00 = [round(mean(indexCell0{2})) round(mean(indexCell0{end}))];
            ind0 = [ind00(1) ind00(1) ind00(2) ind00(1) ind00(1) ind00(2)];
            pt = [camPosePath2(ind0(1),10),camPosePath2(ind0(1),11); camPosePath2(ind0(3),10),camPosePath2(ind0(3),11);camPosePath2(ind0(end),10),camPosePath2(ind0(end),11)];
        end
        
        figure,plot(disp,accumAng);  % hold on;plot(disp(ind0),accumAng(ind0),'*g');
        figure,plot(timeStamp,accumAng);hold on;plot(timeStamp(ind0),accumAng(ind0),'*g');plot(roboPoseInfo1(:,13),roboPoseInfo1(:,14));plot(slamPoseInfo1(:,13),slamPoseInfo1(:,14));
        
    catch
        ind0 = [];
        pt = [];
        figure,plot(disp,accumAng);  % hold on;plot(disp(ind0),accumAng(ind0),'*g');
        figure,plot(timeStamp,accumAng);   %hold on;plot(timeStamp(ind0),accumAng(ind0),'*g');plot(roboPoseInfo1(:,13),roboPoseInfo1(:,14));plot(slamPoseInfo1(:,13),slamPoseInfo1(:,14));
        
    end
    
end

asdvkj = 1;





% % if forTest == 1
% %     exit(1);
% % end




if 0
    % % ptUndistRight = normalize_pixel([logg(z,8:2:end);logg(z,9:2:end)],camParam.foc,camParam.cen,camParam.kc,0);
    % %         ptUndist = intrMatRight*[ptUndistRight;ones(1,size(ptUndistRight,2))];
    % %         ptUndist = [ptUndist(1,:)./ptUndist(3,:);ptUndist(2,:)./ptUndist(3,:)];
    % %         ptUndist3D = [ptUndistRight; ones(1,size(ptUndistRight,2))]';
    % %
    % %         mappedPtMetric = H*ptUndist3D';
    % %         mappedPtMetric = [mappedPtMetric(1,:)./mappedPtMetric(3,:);mappedPtMetric(2,:)./mappedPtMetric(3,:);ones(1,size(mappedPtMetric,2))];
    
    
    % %         [Rstereo,Tstereo,Nstereo,~] = GetTrueRT('stereo',sol, mappedPtMetric', [],[],[]);
    % % %         [Rstereo,Tstereo,Nstereo,~] = GetTrueRT('stereo',sol, camPath(50:z,10:12), [],[],[]);
    % % %
    % % %         for e = 1 : 4
    % % %             rTmp = sol(1:3,1:3,e);
    % % %             tTmp2 = sol(1:3,4,e);
    % % %             scale = Cen2CenDist./norm(tTmp2);
    % % %             tTmp = scale.*tTmp2;
    % % %             invT = inv([rTmp tTmp;0 0 0 1]);
    % % %
    % % %             projErrrr(e,1) = ProjectErr3dTo2d(ptUndistHomo', metricList1, intrMatHomo, invT(1:3,1:3), invT(1:3,4));
    % % %         end
    % % %         projErr = ProjectErr3dTo2d(ptUndistHomo', metricList1, intrMatHomo, rodrigues(camPoseVec(1:3)), camPoseVec(4:6));
    % % %         [~, idH] = min(projErrrr);
    % % %         RHomo = sol(1:3,1:3,idH);
    % % %         NHomo = sol(1:3,5,idH);
    % % %         ground = [NHomo; -Cen2CenDist./norm(sol(1:3,1:3,idH))];
    % % %         tHomo = Cen2CenDist./norm(sol(1:3,4,idH)).*sol(1:3,4,idH);
    % % %
    % % %         THomo = [RHomo tHomo;0 0 0 1];
    % % %         THomoInv = inv(THomo);  % gnd2ceiling
    % % %
    % % %         Ngnd2c = THomoInv(1:3,1:3)*[0;0;1];
    % % %         % %         ground = [NHomo; -Cen2CenDist./norm(tHomo)];
    % % %         ground = [NHomo; -(norm(mean(THomo(1:3,4)./sol(1:3,4,idH))) - height)];
    % % %         % % % % % %         ground1 = [RHomo(3,:)'; -norm(THomo(3,4))];
    % % %
    % % %
    % % %         err = ground'*[THomoInv(1:3,4);1];
    % %         ground = [NHomo; -norm(THomo(3,4))];
    
    % %          d = norm(camPoseVec(4:6))/norm(Tstereo);
    
    
    
    
    
    
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
        if 0
            if z == 100
                ptUndistRight = normalize_pixel([logg(z,8:2:end);logg(z,9:2:end)],camParam.foc,camParam.cen,camParam.kc,0);
                ptUndist = intrMatRight*[ptUndistRight;ones(1,size(ptUndistRight,2))];
                ptUndist = [ptUndist(1,:)./ptUndist(3,:);ptUndist(2,:)./ptUndist(3,:)];
                ptUndist3D = [ptUndistRight; ones(1,size(ptUndistRight,2))]';
                
                mappedPtMetric = H*ptUndist3D';
                mappedPtMetric = [mappedPtMetric(1,:)./mappedPtMetric(3,:);mappedPtMetric(2,:)./mappedPtMetric(3,:);ones(1,size(mappedPtMetric,2))];
                
                
                % %         [Rstereo,Tstereo,Nstereo,~] = GetTrueRT('stereo',sol, mappedPtMetric', [],[],[]);
                % % %         [Rstereo,Tstereo,Nstereo,~] = GetTrueRT('stereo',sol, camPath(50:z,10:12), [],[],[]);
                
                for e = 1 : 4
                    rTmp = sol(1:3,1:3,e);
                    tTmp2 = sol(1:3,4,e);
                    scale = Cen2CenDist./norm(tTmp2);
                    tTmp = scale.*tTmp2;
                    invT = inv([rTmp tTmp;0 0 0 1]);
                    
                    projErrrr(e,1) = ProjectErr3dTo2d(ptUndistHomo', metricList1, intrMatHomo, invT(1:3,1:3), invT(1:3,4));
                end
                projErr = ProjectErr3dTo2d(ptUndistHomo', metricList1, intrMatHomo, rodrigues(camPoseVec(1:3)), camPoseVec(4:6));
                [~, idH] = min(projErrrr);
                RHomo = sol(1:3,1:3,idH);
                NHomo = sol(1:3,5,idH);
                ground = [NHomo; -Cen2CenDist./norm(sol(1:3,1:3,idH))];
                tHomo = Cen2CenDist./norm(sol(1:3,4,idH)).*sol(1:3,4,idH);
                
                THomo = [RHomo tHomo;0 0 0 1];
                THomoInv = inv(THomo);  % gnd2ceiling
                
                Ngnd2c = THomoInv(1:3,1:3)*[0;0;1];
                % %         ground = [NHomo; -Cen2CenDist./norm(tHomo)];
                ground = [NHomo; -(norm(mean(THomo(1:3,4)./sol(1:3,4,idH))) - height)];
                % % % % % %         ground1 = [RHomo(3,:)'; -norm(THomo(3,4))];
                
                
                err = ground'*[THomoInv(1:3,4);1];
                % %         ground = [NHomo; -norm(THomo(3,4))];
                
                % %          d = norm(camPoseVec(4:6))/norm(Tstereo);
                
            end
        end
        
        
        
        
    end
    dist = mean(Dist')';
    distF = medfilt1(dist,2*2-1);
    
    camPosePath = camPath(:,1:end - 1);
    timeStamp = camPath(:,end)./1000;
    
    % % figure, plotPath(camPath);
    % % fig_3D3_pair(camPath(:,10:12)',camPath(:,10:12)');
    
    
    
    for k = 1 : size(camPath,1)
        rr = reshape(camPath(k,1:9),3,3);
        xAxisList(:,k) = rr(:,1)';
        yAxisiList(:,k) = rr(:,2)';
        zAxisiList(:,k) = rr(:,3)';
        try
            rotVecList(:,k) = rodrigues(reshape(camPath(k,1:9),3,3))./norm(rodrigues(reshape(camPath(k,1:9),3,3)));
        catch
            asdkjb = 1;
            rotVecList(:,k) = [0;0;1];
        end
        
    end
    
    if draw == 1
        figure,plotQuiver(rotVecList',[1 0 0]);
    end
    [~, ~, inliersRot] = ransacfitplane(rotVecList, 0.05);
    if draw == 1
        figure,plotQuiver(rotVecList(:,inliersRot)',[1 0 0]);
    end
    
    logg = logg(inliersRot,:);
    camPath = camPath(inliersRot,:);
    xListt = []; yListt = []; Dist = [];
    for z = 1 : size(logg,1)
        xListt(z,:) = logg(z,8:2:end);
        yListt(z,:) = logg(z,9:2:end);
        if z > 1
            [~,Dist(z-1,:)] = NormalizeVector([xListt(z,:) - xListt(z-1,:);yListt(z,:) - yListt(z-1,:)]');
        end
        % % % % % % % % % % %
        % % % % % % % % % % %     if z == 100
        % % % % % % % % % % %         ptUndistRight = normalize_pixel([logg(z,8:2:end);logg(z,9:2:end)],camParam.foc,camParam.cen,camParam.kc,0);
        % % % % % % % % % % %         ptUndist = intrMatRight*[ptUndistRight;ones(1,size(ptUndistRight,2))];
        % % % % % % % % % % %         ptUndist = [ptUndist(1,:)./ptUndist(3,:);ptUndist(2,:)./ptUndist(3,:)];
        % % % % % % % % % % %         ptUndist3D = [ptUndistRight; ones(1,size(ptUndistRight,2))]';
        % % % % % % % % % % %
        % % % % % % % % % % %         mappedPtMetric = H*ptUndist3D';
        % % % % % % % % % % %         mappedPtMetric = [mappedPtMetric(1,:)./mappedPtMetric(3,:);mappedPtMetric(2,:)./mappedPtMetric(3,:);ones(1,size(mappedPtMetric,2))];
        % % % % % % % % % % %
        % % % % % % % % % % %
        % % % % % % % % % % %         % %         [Rstereo,Tstereo,Nstereo,~] = GetTrueRT('stereo',sol, mappedPtMetric', [],[],[]);
        % % % % % % % % % % %         % % %         [Rstereo,Tstereo,Nstereo,~] = GetTrueRT('stereo',sol, camPath(50:z,10:12), [],[],[]);
        % % % % % % % % % % %
        % % % % % % % % % % %         for e = 1 : 4
        % % % % % % % % % % %             rTmp = sol(1:3,1:3,e);
        % % % % % % % % % % %             tTmp2 = sol(1:3,4,e);
        % % % % % % % % % % %             scale = Cen2CenDist./norm(tTmp2);
        % % % % % % % % % % %             tTmp = scale.*tTmp2;
        % % % % % % % % % % %             invT = inv([rTmp tTmp;0 0 0 1]);
        % % % % % % % % % % %
        % % % % % % % % % % %             projErrrr(e,1) = ProjectErr3dTo2d(ptUndistHomo', metricList1, intrMatHomo, invT(1:3,1:3), invT(1:3,4));
        % % % % % % % % % % %         end
        % % % % % % % % % % %         projErr = ProjectErr3dTo2d(ptUndistHomo', metricList1, intrMatHomo, rodrigues(camPoseVec(1:3)), camPoseVec(4:6));
        % % % % % % % % % % %         [~, idH] = min(projErrrr);
        % % % % % % % % % % %         RHomo = sol(1:3,1:3,idH);
        % % % % % % % % % % %         NHomo = sol(1:3,5,idH);
        % % % % % % % % % % %         ground = [NHomo; -Cen2CenDist./norm(sol(1:3,1:3,idH))];
        % % % % % % % % % % %         tHomo = Cen2CenDist./norm(sol(1:3,4,idH)).*sol(1:3,4,idH);
        % % % % % % % % % % %
        % % % % % % % % % % %         THomo = [RHomo tHomo;0 0 0 1];
        % % % % % % % % % % %         THomoInv = inv(THomo);  % gnd2ceiling
        % % % % % % % % % % %
        % % % % % % % % % % %         Ngnd2c = THomoInv(1:3,1:3)*[0;0;1];
        % % % % % % % % % % % % %         ground = [NHomo; -Cen2CenDist./norm(tHomo)];
        % % % % % % % % % % %         ground = [NHomo; -mean(THomo(1:3,4)./sol(1:3,4,idH))];
        % % % % % % % % % % % % % % % % %         ground1 = [RHomo(3,:)'; -norm(THomo(3,4))];
        % % % % % % % % % % %
        % % % % % % % % % % %
        % % % % % % % % % % %         err = ground'*[THomoInv(1:3,4);1];
        % % % % % % % % % % % % %         ground = [NHomo; -norm(THomo(3,4))];
        % % % % % % % % % % %
        % % % % % % % % % % %         % %          d = norm(camPoseVec(4:6))/norm(Tstereo);
        % % % % % % % % % % %
        % % % % % % % % % % %     end
        
    end
    dist = mean(Dist')';
    distF = medfilt1(dist,2*2-1);
    
    camPosePath = camPath(:,1:end - 1);
    timeStamp = camPath(:,end)./1000;
    
    % figure, plotPath(camPath);
    % fig_3D3_pair(camPath(:,10:12)',camPath(:,10:12)');
    if 0
        
        error0 = lenErr(logg, intrMatRight,oCamModel,ground(1:3),THomo,xyzCB, repmat(ground(4),3,1));
        options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter','MaxFunEvals',10000,'TolX',1e-8);
        [heightOpt,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(U) lenErr(logg, intrMatRight,oCamModel,ground(1:3),THomo,xyzCB, U),[repmat(ground(4),1,1)],[],[],options);
        % % [heightOpt2,resnorm2,residual2,exitflag2,output2,lambda2,jacobian2] = lsqnonlin(@(U) lenErr(logg, intrMatRight,oCamModel,ground(1:3),THomo,xyzCB, U),[repmat(ground(4)+10,3,1)],[],[],options);
        
        % % % %     error1 = lenErr(logg, intrMatRight,oCamModel,ground(1:3),THomo,xyzCB, heightOpt);
        [errorOpt, camPosePath2,heightList] = lenErr2(logg, intrMatRight,oCamModel,ground(1:3),THomo,xyzCB, heightOpt);
        calcHeight = -heightList -mean(THomo(1:3,4)./sol(1:3,4,idH));
        sdakjv = 1;
        
    else
        accumAngSlam = 0;
        for q = 1 : size(logg,1)
            iPt = [xListt(q,:); yListt(q,:)]';
            [ptUndist, ptUndist3D] = remapFishEyePix(iPt(:,[1 2])', intrMatRight, oCamModel);
            ptUndist3D_ = ptUndist3D';
            ptUndistRight = ptUndist3D(1:2,:);
            
            initIntersectPt = backProjPix2SpacePlane(ptUndist3D',ground);
            
            initIntersectPtRnd = (THomo(1:3,1:3)*initIntersectPt' + repmat(THomo(1:3,4),1, size(initIntersectPt,1)))';
            initIntersectPtRnd(:,end) = 1;
            if 1
                len(q,1) = norm(initIntersectPtRnd(1,:) - initIntersectPtRnd(2,:));
                len(q,2) = norm(initIntersectPtRnd(3,:) - initIntersectPtRnd(2,:));
                len(q,3) = norm(initIntersectPtRnd(3,:) - initIntersectPtRnd(4,:));
                len(q,4) = norm(initIntersectPtRnd(1,:) - initIntersectPtRnd(4,:));
            end
            
            InitIntersectPtRnd{q,1} = initIntersectPtRnd;
            rt = AlignCoord(initIntersectPtRnd,xyzCB);
            rmatSlam = rt(1:3,1:3);
            rotVecTmp(q,:) = rodrigues(rt(1:3,1:3));
            slamPoseMat(q,:) = [reshape(rt(1:3,1:3),1,9) rt(:,end)']';
            
            if q > 1
                dltTSlam =  [rmatSlam rt(:,end);0 0 0 1]*inv([reshape(slamPoseMat(end-1,1:9),3,3) slamPoseMat(end-1,10:12)';0 0 0 1]);
                dltrSlam = dltTSlam(1:3,1:3);
                dlttSlam = dltTSlam(1:3,end);
                prvPtSlam = [slamPoseMat(end-1,10);0;slamPoseMat(end-1,12)];
                curPtSlam = [slamPoseMat(end,10);0;slamPoseMat(end,12)];
                dltAngSlam = rad2deg(norm(rodrigues(dltrSlam)));
                errSlam = dltrSlam*prvPtSlam + dlttSlam - curPtSlam;
                if norm(rodrigues(rotz(dltAngSlam)) - rodrigues(dltrSlam)) > norm(rodrigues(rotz(-dltAngSlam)) - rodrigues(dltrSlam))
                    accumAngSlam = [accumAngSlam; accumAngSlam(end) + dltAngSlam];
                else
                    accumAngSlam = [accumAngSlam; accumAngSlam(end) - dltAngSlam];
                end
                
            end
            % %         if rotVecTmp(3) < 0
            % %             rotVecTmp = -rotVecTmp;
            % %         end
            
            
            
            err = rt(1:3,1:3)*xyzCB' + repmat(rt(:,end),1,4) - initIntersectPtRnd';
            [~,err2] = NormalizeVector(err');
            Err(q,:) = err2';
            camPosePath2(q,:) = [reshape(rt(:,1:3),1,9) rt(:,end)'];
            
        end
        
        [DistAng1, DistAng2,angRangeInd] = getDistAngInfo2(accumAngSlam, camPosePath2(:,[10 12 11]), 0.7,10,5,8,0,dist,0);
        roundnexxErr = mean(DistAng1(:,1));
        
    end
    
    
    
    
    if ~exist(fullfile(inputDir, 'offset.mat'))
        options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','off');   %,'MaxFunEvals',10000);  %,'TolX',1e-8);
        [xzOffsetOpt,resnorm3,residual3,exitflag3,output3,lambda3,jacobian3] = lsqnonlin(@(U) costFunc(xyzCB,InitIntersectPtRnd,dist, U),[[0 0]],[],[],options);
        save(fullfile(inputDir, 'offset.mat'),'xzOffsetOpt');
    else
        load(fullfile(inputDir, 'offset.mat'),'xzOffsetOpt');
    end
    [roundnessErr22,camPosePath22] = costFunc2(xyzCB,InitIntersectPtRnd,dist,xzOffsetOpt);
    camPosePath2 = camPosePath22;
end

end



function Error = lenErr(logg, intrMatRight,oCamModel,N,THomo,xyzCB, heights)

heights = repmat(heights(1),3,1);

cbSize = norm(xyzCB(1,:) - xyzCB(2,:));
for q = 1 : size(logg,1)
    xListt(q,:) = logg(q,8:2:end);
    yListt(q,:) = logg(q,9:2:end);
    iPt = [xListt(q,:); yListt(q,:)]';
    [ptUndist, ptUndist3D] = remapFishEyePix(iPt(:,[1 2])', intrMatRight, oCamModel);
    ptUndist3D_ = ptUndist3D';
    ptUndistRight = ptUndist3D(1:2,:);
    
    initIntersectPt = [];
    initIntersectPt(1,:) = backProjPix2SpacePlane(ptUndist3D(:,1)',[N;heights(1)]);
    initIntersectPt(2,:) = backProjPix2SpacePlane(ptUndist3D(:,2)',[N;heights(2)]);
    initIntersectPt(3,:) = backProjPix2SpacePlane(ptUndist3D(:,3)',[N;heights(3)]);
    
    ground4 = fitplane(initIntersectPt');
    ground4 = ground4./norm(ground4(1:3));
    if sign(N(3)) ~= sign(ground4(3))
        ground4 = - ground4;
    end
    
    initIntersectPt(4,:) = backProjPix2SpacePlane(ptUndist3D(:,4)',ground4);
    
    
    initIntersectPtRnd = (THomo(1:3,1:3)*initIntersectPt' + repmat(THomo(1:3,4),1, size(initIntersectPt,1)))';
    initIntersectPtRnd(:,end) = 1;
    if 1
        len(q,1) = norm(initIntersectPtRnd(1,:) - initIntersectPtRnd(2,:));
        len(q,2) = norm(initIntersectPtRnd(3,:) - initIntersectPtRnd(2,:));
        len(q,3) = norm(initIntersectPtRnd(3,:) - initIntersectPtRnd(4,:));
        len(q,4) = norm(initIntersectPtRnd(1,:) - initIntersectPtRnd(4,:));
    end
    
    
    rt = AlignCoord(initIntersectPtRnd,xyzCB);
    err = rt(1:3,1:3)*xyzCB' + repmat(rt(:,end),1,4) - initIntersectPtRnd';
    [~,err2] = NormalizeVector(err');
    Err(q,:) = err2';
    camPosePath2(q,:) = [reshape(rt(:,1:3),1,9) rt(:,end)'];
    
end

Error = abs(mean(len,2) - cbSize);
end

% % % % % function [Error, camPosePath2, heightList] = lenErr2(logg, intrMatRight,oCamModel,N,THomo,xyzCB, heights)
% % % % % heights = repmat(heights(1),3,1);
% % % % %
% % % % % cbSize = norm(xyzCB(1,:) - xyzCB(2,:));
% % % % % for q = 1 : size(logg,1)
% % % % %     xListt(q,:) = logg(q,8:2:end);
% % % % %     yListt(q,:) = logg(q,9:2:end);
% % % % %     iPt = [xListt(q,:); yListt(q,:)]';
% % % % %     [ptUndist, ptUndist3D] = remapFishEyePix(iPt(:,[1 2])', intrMatRight, oCamModel);
% % % % %     ptUndist3D_ = ptUndist3D';
% % % % %     ptUndistRight = ptUndist3D(1:2,:);
% % % % %
% % % % %     initIntersectPt = [];
% % % % %     initIntersectPt(1,:) = backProjPix2SpacePlane(ptUndist3D(:,1)',[N;heights(1)]);
% % % % %     initIntersectPt(2,:) = backProjPix2SpacePlane(ptUndist3D(:,2)',[N;heights(2)]);
% % % % %     initIntersectPt(3,:) = backProjPix2SpacePlane(ptUndist3D(:,3)',[N;heights(3)]);
% % % % %
% % % % %     ground4 = fitplane(initIntersectPt');
% % % % %     ground4 = ground4./norm(ground4(1:3));
% % % % %     if sign(N(3)) ~= sign(ground4(3))
% % % % %         ground4 = - ground4;
% % % % %     end
% % % % %
% % % % %     initIntersectPt(4,:) = backProjPix2SpacePlane(ptUndist3D(:,4)',ground4);
% % % % %
% % % % %
% % % % %     initIntersectPtRnd = (THomo(1:3,1:3)*initIntersectPt' + repmat(THomo(1:3,4),1, size(initIntersectPt,1)))';
% % % % %     initIntersectPtRnd(:,end) = 1;
% % % % %     if 1
% % % % %         len(q,1) = norm(initIntersectPtRnd(1,:) - initIntersectPtRnd(2,:));
% % % % %         len(q,2) = norm(initIntersectPtRnd(3,:) - initIntersectPtRnd(2,:));
% % % % %         len(q,3) = norm(initIntersectPtRnd(3,:) - initIntersectPtRnd(4,:));
% % % % %         len(q,4) = norm(initIntersectPtRnd(1,:) - initIntersectPtRnd(4,:));
% % % % %     end
% % % % %
% % % % %
% % % % %     rt = AlignCoord(initIntersectPtRnd,xyzCB);
% % % % %     err = rt(1:3,1:3)*xyzCB' + repmat(rt(:,end),1,4) - initIntersectPtRnd';
% % % % %     [~,err2] = NormalizeVector(err');
% % % % %     Err(q,:) = err2';
% % % % %     camPosePath2(q,:) = [reshape(rt(:,1:3),1,9) rt(:,end)'];
% % % % %
% % % % %
% % % % %
% % % % %     % % %      options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter','MaxFunEvals',10000,'TolX',1e-8);
% % % % %     % % %     [xzOffsetOpt,resnorm3,residual3,exitflag3,output3,lambda3,jacobian3] = lsqnonlin(@(U) costFunc(xyzCB,InitIntersectPtRnd, U),[[0 0]],[],[],options);
% % % % %     % % %
% % % % %
% % % % %
% % % % %
% % % % % end
% % % % % heightList = [heights;ground4(end)];
% % % % % Error = abs(mean(len,2) - cbSize);
% % % % % end






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


function roundnessErr = costFunc(xyzCB0,InitIntersectPtRnd,dist,xzOffset)
xyzCB = xyzCB0 + repmat([xzOffset 0],size(xyzCB0,1),1);
accumAngSlam = 0;
for q = 1 : length(InitIntersectPtRnd)
    % %         iPt = [xListt(q,:); yListt(q,:)]';
    % %         [ptUndist, ptUndist3D] = remapFishEyePix(iPt(:,[1 2])', intrMatRight, oCamModel);
    % %         ptUndist3D_ = ptUndist3D';
    % %         ptUndistRight = ptUndist3D(1:2,:);
    % %
    % %         initIntersectPt = backProjPix2SpacePlane(ptUndist3D',ground);
    % %
    % %         initIntersectPtRnd = (THomo(1:3,1:3)*initIntersectPt' + repmat(THomo(1:3,4),1, size(initIntersectPt,1)))';
    % %         initIntersectPtRnd(:,end) = 1;
    % %         if 1
    % %             len(q,1) = norm(initIntersectPtRnd(1,:) - initIntersectPtRnd(2,:));
    % %             len(q,2) = norm(initIntersectPtRnd(3,:) - initIntersectPtRnd(2,:));
    % %             len(q,3) = norm(initIntersectPtRnd(3,:) - initIntersectPtRnd(4,:));
    % %             len(q,4) = norm(initIntersectPtRnd(1,:) - initIntersectPtRnd(4,:));
    % %         end
    
    initIntersectPtRnd = InitIntersectPtRnd{q,1};
    rt = AlignCoord(initIntersectPtRnd,xyzCB);
    rmatSlam = rt(1:3,1:3);
    rotVecTmp(q,:) = rodrigues(rt(1:3,1:3));
    slamPoseMat(q,:) = [reshape(rt(1:3,1:3),1,9) rt(:,end)']';
    
    if q > 1
        dltTSlam =  [rmatSlam rt(:,end);0 0 0 1]*inv([reshape(slamPoseMat(end-1,1:9),3,3) slamPoseMat(end-1,10:12)';0 0 0 1]);
        dltrSlam = dltTSlam(1:3,1:3);
        dlttSlam = dltTSlam(1:3,end);
        prvPtSlam = [slamPoseMat(end-1,10);0;slamPoseMat(end-1,12)];
        curPtSlam = [slamPoseMat(end,10);0;slamPoseMat(end,12)];
        dltAngSlam = rad2deg(norm(rodrigues(dltrSlam)));
        errSlam = dltrSlam*prvPtSlam + dlttSlam - curPtSlam;
        if norm(rodrigues(rotz(dltAngSlam)) - rodrigues(dltrSlam)) > norm(rodrigues(rotz(-dltAngSlam)) - rodrigues(dltrSlam))
            accumAngSlam = [accumAngSlam; accumAngSlam(end) + dltAngSlam];
        else
            accumAngSlam = [accumAngSlam; accumAngSlam(end) - dltAngSlam];
        end
        
    end
    % %         if rotVecTmp(3) < 0
    % %             rotVecTmp = -rotVecTmp;
    % %         end
    
    
    
    err = rt(1:3,1:3)*xyzCB' + repmat(rt(:,end),1,4) - initIntersectPtRnd';
    [~,err2] = NormalizeVector(err');
    Err(q,:) = err2';
    camPosePath2(q,:) = [reshape(rt(:,1:3),1,9) rt(:,end)'];
    
end

[DistAng1, DistAng2,angRangeInd] = getDistAngInfo2(accumAngSlam, camPosePath2(:,[10 12 11]), 0.7,10,5,8,1,dist,0);
roundnessErr = mean(DistAng1(:,1));
end

function [roundnessErr,camPosePath2] = costFunc2(xyzCB0,InitIntersectPtRnd,dist,xzOffset)
xyzCB = xyzCB0 + repmat([xzOffset 0],size(xyzCB0,1),1);
accumAngSlam = 0;
for q = 1 : length(InitIntersectPtRnd)
    % %         iPt = [xListt(q,:); yListt(q,:)]';
    % %         [ptUndist, ptUndist3D] = remapFishEyePix(iPt(:,[1 2])', intrMatRight, oCamModel);
    % %         ptUndist3D_ = ptUndist3D';
    % %         ptUndistRight = ptUndist3D(1:2,:);
    % %
    % %         initIntersectPt = backProjPix2SpacePlane(ptUndist3D',ground);
    % %
    % %         initIntersectPtRnd = (THomo(1:3,1:3)*initIntersectPt' + repmat(THomo(1:3,4),1, size(initIntersectPt,1)))';
    % %         initIntersectPtRnd(:,end) = 1;
    % %         if 1
    % %             len(q,1) = norm(initIntersectPtRnd(1,:) - initIntersectPtRnd(2,:));
    % %             len(q,2) = norm(initIntersectPtRnd(3,:) - initIntersectPtRnd(2,:));
    % %             len(q,3) = norm(initIntersectPtRnd(3,:) - initIntersectPtRnd(4,:));
    % %             len(q,4) = norm(initIntersectPtRnd(1,:) - initIntersectPtRnd(4,:));
    % %         end
    
    initIntersectPtRnd = InitIntersectPtRnd{q,1};
    rt = AlignCoord(initIntersectPtRnd,xyzCB);
    rmatSlam = rt(1:3,1:3);
    rotVecTmp(q,:) = rodrigues(rt(1:3,1:3));
    slamPoseMat(q,:) = [reshape(rt(1:3,1:3),1,9) rt(:,end)']';
    
    if q > 1
        dltTSlam =  [rmatSlam rt(:,end);0 0 0 1]*inv([reshape(slamPoseMat(end-1,1:9),3,3) slamPoseMat(end-1,10:12)';0 0 0 1]);
        dltrSlam = dltTSlam(1:3,1:3);
        dlttSlam = dltTSlam(1:3,end);
        prvPtSlam = [slamPoseMat(end-1,10);0;slamPoseMat(end-1,12)];
        curPtSlam = [slamPoseMat(end,10);0;slamPoseMat(end,12)];
        dltAngSlam = rad2deg(norm(rodrigues(dltrSlam)));
        errSlam = dltrSlam*prvPtSlam + dlttSlam - curPtSlam;
        if norm(rodrigues(rotz(dltAngSlam)) - rodrigues(dltrSlam)) > norm(rodrigues(rotz(-dltAngSlam)) - rodrigues(dltrSlam))
            accumAngSlam = [accumAngSlam; accumAngSlam(end) + dltAngSlam];
        else
            accumAngSlam = [accumAngSlam; accumAngSlam(end) - dltAngSlam];
        end
        
    end
    % %         if rotVecTmp(3) < 0
    % %             rotVecTmp = -rotVecTmp;
    % %         end
    
    
    
    err = rt(1:3,1:3)*xyzCB' + repmat(rt(:,end),1,4) - initIntersectPtRnd';
    [~,err2] = NormalizeVector(err');
    Err(q,:) = err2';
    camPosePath2(q,:) = [reshape(rt(:,1:3),1,9) rt(:,end)'];
    
end

[DistAng1, DistAng2,angRangeInd] = getDistAngInfo2(accumAngSlam, camPosePath2(:,[10 12 11]), 0.7,10,5,8,1,dist,0);
roundnessErr = mean(DistAng1(:,1));
end

function [camPosePath2,Err,len,accumAngSlam] = costFunc3(xyzCB0,logg,xzOffset,intrMatRight,oCamModel,ground,THomo, heightOpt,dist2Gnd)
xyzCB = xyzCB0 + repmat([xzOffset 0],size(xyzCB0,1),1);
accumAngSlam = 0;
for q = 1 : size(logg,1)
    xListt(q,:) = logg(q,8:2:end);
    yListt(q,:) = logg(q,9:2:end);
    iPt = [xListt(q,:); yListt(q,:)]';
    [ptUndist, ptUndist3D] = remapFishEyePix(iPt(:,[1 2])', intrMatRight, oCamModel);
    ptUndist3D_ = ptUndist3D';
    ptUndistRight = ptUndist3D(1:2,:);
    
    initIntersectPt = backProjPix2SpacePlane(ptUndist3D',ground);
    initIntersectPt = [];
    initIntersectPt(1,:) = backProjPix2SpacePlane(ptUndist3D(:,1)',[ground(1:3);-(dist2Gnd - heightOpt(1))]);
    initIntersectPt(2,:) = backProjPix2SpacePlane(ptUndist3D(:,2)',[ground(1:3);-(dist2Gnd - heightOpt(2))]);
    initIntersectPt(3,:) = backProjPix2SpacePlane(ptUndist3D(:,3)',[ground(1:3);-(dist2Gnd - heightOpt(3))]);
    initIntersectPt(4,:) = backProjPix2SpacePlane(ptUndist3D(:,4)',[ground(1:3);-(dist2Gnd - heightOpt(4))]);
    
    
    
    % % % % % % % % % % % % % % %     initIntersectPt(1,:) = backProjPix2SpacePlane(ptUndist3D(:,1)',[N;heights(1)]);
    % % % % % % % % % % % % % % %     initIntersectPt(2,:) = backProjPix2SpacePlane(ptUndist3D(:,2)',[N;heights(2)]);
    % % % % % % % % % % % % % % %     initIntersectPt(3,:) = backProjPix2SpacePlane(ptUndist3D(:,3)',[N;heights(3)]);
    % % % % % % % % % % % % % % %
    % % % % % % % % % % % % % % %     if 0
    % % % % % % % % % % % % % % %         ground4 = fitplane(initIntersectPt');
    % % % % % % % % % % % % % % %         ground4 = ground4./norm(ground4(1:3));
    % % % % % % % % % % % % % % %         if sign(N(3)) ~= sign(ground4(3))
    % % % % % % % % % % % % % % %             ground4 = - ground4;
    % % % % % % % % % % % % % % %         end
    % % % % % % % % % % % % % % %
    % % % % % % % % % % % % % % %         initIntersectPt(4,:) = backProjPix2SpacePlane(ptUndist3D(:,4)',ground4);
    % % % % % % % % % % % % % % %     else
    % % % % % % % % % % % % % % %         initIntersectPt(4,:) = backProjPix2SpacePlane(ptUndist3D(:,4)',[N;heights(4)]);
    % % % % % % % % % % % % % % %     end
    
    
    
    
    
    
    
    
    
    initIntersectPtRnd = (THomo(1:3,1:3)*initIntersectPt' + repmat(THomo(1:3,4),1, size(initIntersectPt,1)))';
    initIntersectPtRnd(:,end) = 1;
    if 1
        len(q,1) = norm(initIntersectPtRnd(1,:) - initIntersectPtRnd(2,:));
        len(q,2) = norm(initIntersectPtRnd(3,:) - initIntersectPtRnd(2,:));
        len(q,3) = norm(initIntersectPtRnd(3,:) - initIntersectPtRnd(4,:));
        len(q,4) = norm(initIntersectPtRnd(1,:) - initIntersectPtRnd(4,:));
    end
    
    InitIntersectPtRnd{q,1} = initIntersectPtRnd;
    rt = AlignCoord(initIntersectPtRnd,xyzCB);
    rmatSlam = rt(1:3,1:3);
    rotVecTmp(q,:) = rodrigues(rt(1:3,1:3));
    slamPoseMat(q,:) = [reshape(rt(1:3,1:3),1,9) rt(:,end)']';
    
    if q > 1
        dltTSlam =  [rmatSlam rt(:,end);0 0 0 1]*inv([reshape(slamPoseMat(end-1,1:9),3,3) slamPoseMat(end-1,10:12)';0 0 0 1]);
        dltrSlam = dltTSlam(1:3,1:3);
        dlttSlam = dltTSlam(1:3,end);
        prvPtSlam = [slamPoseMat(end-1,10);0;slamPoseMat(end-1,12)];
        curPtSlam = [slamPoseMat(end,10);0;slamPoseMat(end,12)];
        dltAngSlam = rad2deg(norm(rodrigues(dltrSlam)));
        errSlam = dltrSlam*prvPtSlam + dlttSlam - curPtSlam;
        if norm(rodrigues(rotz(dltAngSlam)) - rodrigues(dltrSlam)) > norm(rodrigues(rotz(-dltAngSlam)) - rodrigues(dltrSlam))
            accumAngSlam = [accumAngSlam; accumAngSlam(end) + dltAngSlam];
        else
            accumAngSlam = [accumAngSlam; accumAngSlam(end) - dltAngSlam];
        end
        
    end
    % %         if rotVecTmp(3) < 0
    % %             rotVecTmp = -rotVecTmp;
    % %         end
    
    
    
    err = rt(1:3,1:3)*xyzCB' + repmat(rt(:,end),1,4) - initIntersectPtRnd';
    [~,err2] = NormalizeVector(err');
    Err(q,:) = err2';
    camPosePath2(q,:) = [reshape(rt(:,1:3),1,9) rt(:,end)'];
    
end

% % [DistAng1, DistAng2,angRangeInd] = getDistAngInfo2(accumAngSlam, camPosePath2(:,[10 12 11]), 0.7,10,5,8,1,dist,0);
% % roundnessErr = mean(DistAng1(:,1));
end


function [Error, camPosePath2,heightList,InitIntersectPtRnd] = lenErr2(logg, intrMatRight,oCamModel,N,THomo,xyzCB, heights)

cbSize = norm(xyzCB(1,:) - xyzCB(2,:));
for q = 1 : size(logg,1)
    xListt(q,:) = logg(q,8:2:end);
    yListt(q,:) = logg(q,9:2:end);
    iPt = [xListt(q,:); yListt(q,:)]';
    [ptUndist, ptUndist3D] = remapFishEyePix(iPt(:,[1 2])', intrMatRight, oCamModel);
    ptUndist3D_ = ptUndist3D';
    ptUndistRight = ptUndist3D(1:2,:);
    
    initIntersectPt = [];
    initIntersectPt(1,:) = backProjPix2SpacePlane(ptUndist3D(:,1)',[N;heights(1)]);
    initIntersectPt(2,:) = backProjPix2SpacePlane(ptUndist3D(:,2)',[N;heights(2)]);
    initIntersectPt(3,:) = backProjPix2SpacePlane(ptUndist3D(:,3)',[N;heights(3)]);
    
    if 0
        ground4 = fitplane(initIntersectPt');
        ground4 = ground4./norm(ground4(1:3));
        if sign(N(3)) ~= sign(ground4(3))
            ground4 = - ground4;
        end
        
        initIntersectPt(4,:) = backProjPix2SpacePlane(ptUndist3D(:,4)',ground4);
    else
        initIntersectPt(4,:) = backProjPix2SpacePlane(ptUndist3D(:,4)',[N;heights(4)]);
    end
    
    initIntersectPtRnd = (THomo(1:3,1:3)*initIntersectPt' + repmat(THomo(1:3,4),1, size(initIntersectPt,1)))';
    initIntersectPtRnd(:,end) = 1;
    if 1
        len(q,1) = abs(norm(initIntersectPtRnd(1,:) - initIntersectPtRnd(2,:)) - cbSize);
        len(q,2) = abs(norm(initIntersectPtRnd(3,:) - initIntersectPtRnd(2,:)) - cbSize);
        len(q,3) = abs(norm(initIntersectPtRnd(3,:) - initIntersectPtRnd(4,:)) - cbSize);
        len(q,4) = abs(norm(initIntersectPtRnd(1,:) - initIntersectPtRnd(4,:)) - cbSize);
        len(q,5) = abs(norm(initIntersectPtRnd(1,:) - initIntersectPtRnd(3,:)) - sqrt(2)*cbSize);
        len(q,6) = abs(norm(initIntersectPtRnd(2,:) - initIntersectPtRnd(4,:)) - sqrt(2)*cbSize);
    end
    
    InitIntersectPtRnd{q,1} = initIntersectPtRnd;
    
    D = pdist(initIntersectPtRnd,'euclidean');
    Vari(q,:) = 10.*var([D([1 3 4 6]) D([2 5])./sqrt(2)]);
    
    rt = AlignCoord(initIntersectPtRnd,xyzCB);
    err = rt(1:3,1:3)*xyzCB' + repmat(rt(:,end),1,4) - initIntersectPtRnd';
    [~,err2] = NormalizeVector(err');
    Err(q,:) = err2';
    camPosePath2(q,:) = [reshape(rt(:,1:3),1,9) rt(:,end)'];
    
end

heightList = [heights];

Error = abs(mean(len,2) - 0);
% % % Error = Vari;
end
