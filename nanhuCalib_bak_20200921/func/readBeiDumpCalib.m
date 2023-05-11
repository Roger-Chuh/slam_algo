function [disp,accumAng] = readBeiDumpCalib(inputDir,paraDir,ang0, dist0)

forTest = 0;
window = 3; 5; 10; 3; 5; 3;
window2 = 1; 3; 1;
window3 = 1;


scaleee = 0.5;


% % close all
draw = 0;
lenDiff = 10; 30;

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

if 1
    [roboPath, roboPoseInfo1] = PlotRobotOnly(inputDir,1);
    [slamPath, slamPoseInfo1] = PlotSlamOnly(inputDir,1);
    
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

if forTest == 0
    % %     heightOpt = [ 158.7972 157.5447 167.6330 168.2161];
    heightOpt = [161.8151 160.6105 170.4220 171.3661];
    
else
    try
        heightOpt = load(fullfile(inputDir, 'patternHeight.txt'));
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
load(fullfile(paraDir, 'calib.mat'));
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
    [H, camPoseVec, metricList1, ptUndistHomo, intrMatHomo] = calibHomo2(inputDir, 'E:\bk_20180627\pc\Temp\20180724\calibFishEye1');
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

Indd = (find(abs(mean(len') - cbSize) < lenDiff & var(len') < 100))';
if 0
    Ind = intersect(Ind,Indd);
else
    Ind = Indd;
end

xListt = xList(Ind,:);
yListt = yList(Ind,:);
pixErr = mean(sqrt(diff(xListt).^2 + diff(yListt).^2)')';
pixErrFilt = medfilt1(pixErr,(2*window - 1));




camPath3 = camPath2(Ind,:);
ang = rad2deg(norm(rodrigues(reshape(camPath3(end,1:9),3,3)'*reshape(camPath3(1,1:9),3,3))));


timeStamp = camPath3(:,end)./1000;

log3 = log(Ind,:);

xzOffsetOpt = [3.67836661170452,-1.38003182427598];
[camPosePath2,Err,len1,accumAng] = costFunc3(xyzCB,log3,xzOffsetOpt,intrMatRight,oCamModel,ground,THomo,heightOpt,dist2Gnd);

[~,disp] = NormalizeVector(camPosePath2(1:end,10:12) - repmat(camPosePath2(1,10:12),size(camPosePath2,1),1));

% % disp = disp-disp(1);
% % disp = abs(disp);
% % disp = [0;disp];
dltAng = abs(diff(accumAng));
dltAngFilt = medfilt1(dltAng,(2*2 - 1));
[indexCell1, index] = splitIndex2(find(pixErrFilt < 0.1));

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
[indexCellRobo, indexRobo] = splitIndex2(find(diff(roboPoseInfo(:,14))./diff(roboPoseInfo(:,13)) < 1)); % 0.2
tmp = medfilt1(diff(roboPoseInfo(:,14))./diff(roboPoseInfo(:,13)),2*window3-1); %diff(roboPoseInfo(:,14))./diff(roboPoseInfo(:,13));
[minv,mini1]=findpeaks(-tmp,'minpeakdistance',1);


% %   mini2 = find(tmp(mini1) < 2);
  mini2 = find(tmp(mini1) < 8);


mini = mini1(mini2);
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
        if 1
            % %             figure,plot(pixErrFilt);hold on;plot(pulse);
            %             figure,plot(pixErrFilt);hold on;plot(find(pulse)-1,difCeilingAng(find(pulse)-1),'*g');
            figure,plot(difCeilingAng);hold on;plot(find(pulse)-1,difCeilingAng(find(pulse)-1),'*g');
        end
    end
    pulseRobo = zeros(length(diff(roboPoseInfo(:,14))),1);
    pulseRobo(idRobo) = max(diff(roboPoseInfo(:,14)));
    % %     difRoboAng = medfilt1(diffAngFilt./diff(timeStamp),2*(window+window2)-1); %diff(roboPoseInfo(:,14));
    difRoboAng = diff(roboPoseInfo(:,14));
    if 1
        
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
% % % % figure,plot(camPosePath2(:,10),camPosePath2(:,11));hold on;plot(camPosePath2(1,10),camPosePath2(1,11),'*');axis equal
if 0
    fig_3D3_pair(camPosePath2(:,10:12)',camPosePath2(:,10:12)');
    figure,plot3(camPosePath2(:,10),camPosePath2(:,11),camPosePath2(:,12),'-');hold on;plot3(camPosePath2(1,10),camPosePath2(1,11),camPosePath2(1,12),'*','MarkerSize',5,'LineWidth',4);plot3(camPosePath2(end,10),camPosePath2(end,11),camPosePath2(end,12),'*','MarkerSize',5,'LineWidth',4);axis equal;xlabel('X (mm)');ylabel('Y (mm)'); zlabel('Z (mm)');
end


if 1  %straight == 1
    reportVec = [ang0 roboPoseInfo(end,end) accumAng(end); dist0 norm(roboPoseInfo(end,10:12)) disp(end)];
else
    reportVec = [roboPoseInfo(end,end) accumAng(end); norm(roboPoseInfo(end,10:12)) disp(end)];
end

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
    figure,hist(angMat(1:end-1,1:3),40);title(sprintf( 'expected angle: %d degree\nceiling %f.2\nwheel %f.2\nvisoin %f.2\n',ang0,angMat(end,1),angMat(end,2),angMat(end,3))); legend('ceiling','wheel','vision');%num2str(ang0));
    % % %     saveas(gcf,fullfile(inputDir,strcat('angDistribution.png')));
    
    
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
catch
    dsghjl = 1;
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

figure,plot(angMat(1:end-1,:))
figure,plot(angMat(1:end-1,3) - angMat(1:end-1,1));
figure,hist(angMat(1:end-1,3) - angMat(1:end-1,1),50);


asdvkj = 1;


if forTest == 1
    exit(1);
end



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
