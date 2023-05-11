function [camPosePath2, timeStamp, dist, logg] = readBeiDump3(inputDir,paraDir,homoDir)
% xyzCB = 1000.*[0, -0.079000004*2, 0;
%     0.079000004*2, -0.079000004*2, 0;
%     0.079000004*2, 0, 0;
%     0, 0, 0];

scaleee = 1; 0.5;  1; 0.5;1;0.5;1;0.5;1;0.5;0.5; 1;  0.5;0.5; 1; 0.5;




lenThr = 50; 5;1.5; 5;15;5; 8;10;40;10;30;20;30;10;30; 20;
varThr = 50; 10;2; 5;20; 40; 12;20;20;10;50;15;5;15; 10;
height = 220; 200; 230; 195; 165;195; 170;199;165;135; 165;132;100; 129;
heights = repmat(height,3,1);
% % % % cbSize = 158;
% % % % xyzCB = 1000.*[0.079000004, 0.079000004, 0;
% % % %     0.079000004, -0.079000004, 0;
% % % %     -0.079000004, -0.079000004, 0;
% % % %     -0.079000004, 0.079000004, 0];
cbSize = 158; 158; 170.3511 + 0.3;  170.9511; 170.9539;    %% 170.4792; 156.6186; 158;156.6186;
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

% % [H, camPoseVec, metricList1, ptUndistHomo, intrMatHomo] = calibHomo2('D:\Temp\20180809\calibFishEyeHomo1', 'E:\bk_20180627\pc\Temp\20180724\calibFishEye1');
% % [H, camPoseVec, metricList1, ptUndistHomo, intrMatHomo] = calibHomo2('D:\Temp\20180828\calibFishEyeHomo1', 'E:\bk_20180627\pc\Temp\20180724\calibFishEye1');
try
    % %     [H, camPoseVec, metricList1, ptUndistHomo, intrMatHomo] = calibHomo2('D:\Temp\20180828\calibFishEyeHomo1', 'E:\bk_20180627\pc\Temp\20180724\calibFishEye1');
    [H, camPoseVec, metricList1, ptUndistHomo, intrMatHomo] = calibHomo2(homoDir, paraDir);
catch
    %     [H, camPoseVec, metricList1, ptUndistHomo, intrMatHomo] = calibHomo2(inputDir, 'E:\bk_20180627\pc\Temp\20180724\calibFishEye1');
    [H, camPoseVec, metricList1, ptUndistHomo, intrMatHomo] = calibHomo2(inputDir, paraDir);
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


err = ground'*[THomoInv(1:3,4);1];






dirInfo = dir(fullfile(inputDir,'test*.log'));

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
    
    if 1
        iPt = [xList(i,:); yList(i,:)]';
        [ptUndist, ptUndist3D] = remapFishEyePix(iPt(:,[1 2])', intrMatRight, oCamModel);
        ptUndist3D_ = ptUndist3D';
        ptUndistRight = ptUndist3D(1:2,:);
        
        initIntersectPt = backProjPix2SpacePlane(ptUndist3D',ground);
        
        initIntersectPtRnd = (THomo(1:3,1:3)*initIntersectPt' + repmat(THomo(1:3,4),1, size(initIntersectPt,1)))';
        initIntersectPtRnd(:,end) = 1;
        if 1
            len(i,1) = norm(initIntersectPtRnd(1,:) - initIntersectPtRnd(2,:));
            len(i,2) = norm(initIntersectPtRnd(3,:) - initIntersectPtRnd(2,:));
            len(i,3) = norm(initIntersectPtRnd(3,:) - initIntersectPtRnd(4,:));
            len(i,4) = norm(initIntersectPtRnd(1,:) - initIntersectPtRnd(4,:));
        end
        
        if 0
            rt = AlignCoord(initIntersectPtRnd,xyzCB);
            err = rt(1:3,1:3)*xyzCB' + repmat(rt(:,end),1,4) - initIntersectPtRnd';
            [~,err2] = NormalizeVector(err');
            Err(i,:) = err2';
            camPosePath2(i,:) = [reshape(rt(:,1:3),1,9) rt(:,end)'];
        end
        
        
        
        
        
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
if 1
    
    % %     Ind = (find(abs(mean(len') - cbSize) < 5))';
    Ind = (find(abs(mean(len') - cbSize) < lenThr & var(len') < varThr))';
% %     figure,plot(len(Ind,:));
    camPath3 = camPath2(Ind,:);
    log3 = log(Ind,:);
    
else
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
    
    
    camPath3 = camPath2(Ind,:);
    log3 = log(Ind,:);
    
    
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
    
end

if 0
    camPath4 = camPath2(Ind2,:);
    log4 = log(Ind2);
else
    camPath4 = camPath3;
    log4 = log3;
end
if 1
    [BB, ~, inliers] = ransacfitplane(camPath4(:,10:12)', 50000);
else
    inliers = [1:size(camPath4,1)];
end

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
    
    rotVecList(:,k) = rodrigues(reshape(camPath(k,1:9),3,3))./norm(rodrigues(reshape(camPath(k,1:9),3,3)));
    
    
end

% % % % % % % figure,plotQuiver(rotVecList',[1 0 0]);
[~, ~, inliersRot] = ransacfitplane(rotVecList, 100000.05);
% % % % % % % figure,plotQuiver(rotVecList(:,inliersRot)',[1 0 0]);


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



if ~exist(fullfile(inputDir, 'heightOpt.mat'))
    error0 = lenErr(logg, intrMatRight,oCamModel,ground(1:3),THomo,xyzCB, repmat(ground(4),4,1));
    options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','off','MaxFunEvals',10000,'TolX',1e-8);
    [heightOpt,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(U) lenErr(logg, intrMatRight,oCamModel,ground(1:3),THomo,xyzCB, U),[repmat(ground(4),4,1)],[],[],options);
    % % [heightOpt2,resnorm2,residual2,exitflag2,output2,lambda2,jacobian2] = lsqnonlin(@(U) lenErr(logg, intrMatRight,oCamModel,ground(1:3),THomo,xyzCB, U),[repmat(ground(4)+10,3,1)],[],[],options);
    
    error1 = lenErr(logg, intrMatRight,oCamModel,ground(1:3),THomo,xyzCB, heightOpt);
    save(fullfile(inputDir, 'heightOpt.mat'),'heightOpt')
else
    load(fullfile(inputDir, 'heightOpt.mat'))
end
[errorOpt, camPosePath2,heightList,InitIntersectPtRnd,lenn] = lenErr2(logg, intrMatRight,oCamModel,ground(1:3),THomo,xyzCB, heightOpt);




% indIn = find(mean(lenn') < 20);
indIn = find(mean(lenn') < 200);

xListtt = xListt(indIn,:);
yListtt = yListt(indIn,:);
dist = mean(sqrt(diff(xListtt).^2 + diff(yListtt).^2)')';
InitIntersectPtRnd = InitIntersectPtRnd(indIn,:);
logg = logg(indIn,:);
timeStamp = timeStamp(indIn,:);
% % figure,plot(lenn);
% % calcHeight = -heightList -mean(THomo(1:3,4)./sol(1:3,4,idH));

calcHeight = abs(-heightList -mean(THomo(1:3,4)./sol(1:3,4,idH)));
save(fullfile(inputDir, 'heightOpt.mat'),'heightOpt','calcHeight')


sdakjv = 1;


if ~exist(fullfile(inputDir, 'offset.mat'))
    options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','off');   %,'MaxFunEvals',10000);  %,'TolX',1e-8);
    [xzOffsetOpt,resnorm3,residual3,exitflag3,output3,lambda3,jacobian3] = lsqnonlin(@(U) costFunc(xyzCB,InitIntersectPtRnd,dist,timeStamp, U),[[0 0]],[],[],options);
    save(fullfile(inputDir, 'offset.mat'),'xzOffsetOpt');
else
    load(fullfile(inputDir, 'offset.mat'),'xzOffsetOpt');
end
[roundnessErr22,camPosePath22] = costFunc2(xyzCB,InitIntersectPtRnd,dist,timeStamp,xzOffsetOpt);
camPosePath2 = camPosePath22;


askj = 1;

 


if 0
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
        
        
        rt = AlignCoord(initIntersectPtRnd,xyzCB);
        err = rt(1:3,1:3)*xyzCB' + repmat(rt(:,end),1,4) - initIntersectPtRnd';
        [~,err2] = NormalizeVector(err');
        Err(q,:) = err2';
        camPosePath2(q,:) = [reshape(rt(:,1:3),1,9) rt(:,end)'];
        
    end
end


end



function Error = lenErr(logg, intrMatRight,oCamModel,N,THomo,xyzCB, heights)

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
        % %         len(q,1) = norm(initIntersectPtRnd(1,:) - initIntersectPtRnd(2,:));
        % %         len(q,2) = norm(initIntersectPtRnd(3,:) - initIntersectPtRnd(2,:));
        % %         len(q,3) = norm(initIntersectPtRnd(3,:) - initIntersectPtRnd(4,:));
        % %         len(q,4) = norm(initIntersectPtRnd(1,:) - initIntersectPtRnd(4,:));
        
        
        len(q,1) = abs(norm(initIntersectPtRnd(1,:) - initIntersectPtRnd(2,:)) - cbSize);
        len(q,2) = abs(norm(initIntersectPtRnd(3,:) - initIntersectPtRnd(2,:)) - cbSize);
        len(q,3) = abs(norm(initIntersectPtRnd(3,:) - initIntersectPtRnd(4,:)) - cbSize);
        len(q,4) = abs(norm(initIntersectPtRnd(1,:) - initIntersectPtRnd(4,:)) - cbSize);
        len(q,5) = abs(norm(initIntersectPtRnd(1,:) - initIntersectPtRnd(3,:)) - sqrt(2)*cbSize);
        len(q,6) = abs(norm(initIntersectPtRnd(2,:) - initIntersectPtRnd(4,:)) - sqrt(2)*cbSize);
        
    end
    
    D = pdist(initIntersectPtRnd,'euclidean');
    
    Vari(q,:) = 10.*var([D([1 3 4 6]) D([2 5])./sqrt(2)]);
    
    
    rt = AlignCoord(initIntersectPtRnd,xyzCB);
    err = rt(1:3,1:3)*xyzCB' + repmat(rt(:,end),1,4) - initIntersectPtRnd';
    [~,err2] = NormalizeVector(err');
    Err(q,:) = err2';
    camPosePath2(q,:) = [reshape(rt(:,1:3),1,9) rt(:,end)'];
    
end

Error = abs(mean(len,2) - 0); % cbSize);
% % % Error = Vari;
end

function [Error, camPosePath2,heightList,InitIntersectPtRnd,len] = lenErr2(logg, intrMatRight,oCamModel,N,THomo,xyzCB, heights)

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


function roundnessErr = costFunc(xyzCB0,InitIntersectPtRnd,dist,timeStamp,xzOffset)
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

[DistAng1, DistAng2,angRangeInd] = getDistAngInfo2(accumAngSlam, camPosePath2(:,[10 12 11]), 0.7,10,5,8,1,dist,timeStamp,0);
roundnessErr = mean(DistAng1(:,1));
end

function [roundnessErr,camPosePath2] = costFunc2(xyzCB0,InitIntersectPtRnd,dist,timeStamp,xzOffset)
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
try
    [DistAng1, DistAng2,angRangeInd] = getDistAngInfo2(accumAngSlam, camPosePath2(:,[10 12 11]), 0.7,10,5,8,1,dist,timeStamp,0);
    roundnessErr = mean(DistAng1(:,1));
catch
    roundnessErr = inf;
end
end