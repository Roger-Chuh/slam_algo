function [roboPosePoseMatStamp, DistAng1, DistAng2,posestPoseMapped2,startInd,Dist,timeStampOrig] = readVidAndPlot(inputDir, vidInd, matDir, paraDir, dataNum, readBei,homoDir)
% H = calibHomo2('D:\Temp\20180703\calibHomo1\calib', 'D:\Temp\20180625\calibCalTag1\calib_use');

n = 2; n2 = 3; n3 = 8;
ratio = 1.5; ratio2 = 1;
ratioSmall = 1.5; 0.05; 1.5;
ratioLarge = 1.5;
skipTimes = 4; 5; 8; 6;
draw = 1;0;
roboStartThr = 15; 10;
drawResult = 0; 1;
initSkipNum = 6; 10;6; 8; 6; 10;8; 5; 10;
angThr = 2; 3; % degree;
WindowSize = 5;2;10; 2;  10;
pixDiff = 1.5;
log = 0;
thr = 1; 15000; 1;5000; 5000;2500;1000; 10000;10000; 15000; 6000; 6000; 12000; 18000;
margin = 50; 20;50; 200; 50;200;
marginSmall = 70;100;50;
marginLarge = 150;250;150; 50; 100;
smallTic = 0; bigTic = 1;

use_fisheye = 0;


marginInd = 3;
% % close all
% H = calibHomo2('D:\Temp\20180703\calibHomo2\calib', 'D:\Temp\20180625\calibCalTag1\calib_use');
% % H = calibHomo2('D:\Temp\20180706\calibHomo1\calib', 'D:\Temp\20180625\calibCalTag1\calib_use');
try
    % %     H = calibHomo2('E:\bk_20180627\pc\Temp\20180725\calibHomo1', 'E:\bk_20180627\pc\Temp\20180724\calibFishEye1');
    % %     H = calibHomo2('D:\Temp\20180809\calibFishEyeHomo1', 'E:\bk_20180627\pc\Temp\20180724\calibFishEye1');
    H = calibHomo2(homoDir, paraDir);
catch
    try
        load(fullfile(inputDir,'homo.mat'),'HH');
        H = HH;
    catch
        load(fullfile(inputDir,'homo.mat'),'H_');
        H = H_;
    end
end

outputDir = fullfile(inputDir, 'calib');
MakeDirIfMissing(outputDir);
dirInfo = dir(fullfile(inputDir, '*.mp4'));
try
    load(fullfile(paraDir, 'calib.mat'));
catch
    load(fullfile(inputDir, 'calib.mat'));
    paraDir = inputDir;
end
k1 = camParam.kc(1);
k2 = camParam.kc(2);
k3 = camParam.kc(5);
p1 = camParam.kc(3);
p2 = camParam.kc(4);
% % img = rgb2gray(imread(imgList{1}));
% % imgSize = size(img);
% % imgH = imgSize(1);
% % imgW = imgSize(2);
% intrMatRight = [camParam.foc(1),0,camParam.cen(1); 0,camParam.foc(2),camParam.cen(2); 0,0,1];

paramVec = [camParam.foc; camParam.cen; camParam.kc];
fid1 = fopen(fullfile(inputDir,'FishEyeParam.txt'),'w');%????
for u = 1 : length(paramVec)
    fprintf(fid1,'%06f\n',paramVec(u));
end

fclose(fid1);




if use_fisheye == 0
    intrMatRight = [camParam.foc(1),0,camParam.cen(1); 0,camParam.foc(2),camParam.cen(2); 0,0,1];
else
    %     load('D:\Temp\20180625\calibCalTag1\calib_use\Omni_Calib_Results.mat');
    % %     load('E:\bk_20180627\pc\Temp\20180625\calibCalTag1\calib_use\oCamModel.mat');
    load(fullfile(paraDir,'oCamModel.mat'));
    % %     oCamModel = calib_data.ocam_model;
    U_same = ocam_undistort_map(oCamModel,'OutputView','same');
    U_full = ocam_undistort_map(oCamModel,'OutputView','full');
    intrMat_same = U_same.K';
    intrMat_full = U_full.K';
    intrMatRight = intrMat_same;
    
    % %         intrMatRight = intrMat_full;
    % %         U_same = U_full;
end


squareSize = 149;
squareSize = 107;
scale = 1;0.5;1;0.5; 1;1;1;0.4;
% matDir = 'D:\Work\caltag-master\caltag-master\GeneratePattern';
if 1 %~exist(fullfile(inputDir,sprintf('data_%02d.mat',dataNum)))
    
    if readBei == 0
        if bigTic == 1
            ticAll = tic;
        end
        for i = 1 : 1 %length(dirInfo)
            obj = VideoReader(fullfile(inputDir, dirInfo(vidInd).name));%输入视频位置
            %                 numFrames = obj.NumberOfFrames;% 帧的总数
            %         figure(i),clf;
            cnt1 = 1; cnt2 = 1;
            iPt_Use = {}; wPt_Use = {}; Flow = [];LargeSkipFlag = []; tm = 1;tm2 = 1;  % accumAng = 0;
            k = 0;
            roboStamp = [];
            while hasFrame(obj)
                %         for k = 1 : numFrames%
                %             if smallTic == 1
                %               tic;
                %             end
                I =  readFrame(obj);
                %             if use_fisheye == 0
                %                 nim = undistortimage(I, [intrMatRight(1,1) intrMatRight(2,2)]', intrMatRight(1,3), intrMatRight(2,3), k1, k2, k3, p1, p2);
                %             else
                %                 nim = ocam_undistort(I,U_same);
                %             end
                
                
                
                roboStamp = [roboStamp; obj.CurrentTime];
                k = k+1;
                %             if isempty(Flow) || length(Flow) <= n2+1
                if ~exist('skipNum')
                    skipNum = initSkipNum;
                end
                % % %             else
                %                 if max(Flow(end-n2:end,1)) > pixDiff  % || length(Flow) == 0
                %                     LargeSkipFlag = [LargeSkipFlag; 0];
                %                     skipNum = initSkipNum;
                %                 else
                %                     skipNum = initSkipNum;
                %                     LargeSkipFlag = [LargeSkipFlag; 1];
                %                 end
                %                 if length(LargeSkipFlag) > 5 && sum(LargeSkipFlag(end-n3:end)) == n3 + 1
                %                     skipNum = skipTimes*initSkipNum;
                %                 end
                
                % %             end
                if mod(k,skipNum) == 0 || k == 1
                    if 0
                        if use_fisheye == 0
                            nim = undistortimage(I, [intrMatRight(1,1) intrMatRight(2,2)]', intrMatRight(1,3), intrMatRight(2,3), k1, k2, k3, p1, p2);
                        else
                            nim = ocam_undistort(I,U_same);
                        end
                    end
                    if smallTic == 1
                        tic1 = tic;
                    end
                    %                 I =  read(obj,k);
                    %         for k = 1 : 5: numFrames% 读取前15帧
                    %             I = read(obj,k);%读取第几帧
                    % imshow(frame);%显示帧
                    if 0
                        if ~exist('IOld');
                            IOld = zeros(size(rgb2gray(I)));
                        end
                        if ndims(IOld) == 3
                            dltI = im2double(rgb2gray(I)) - im2double(rgb2gray(IOld));
                        else
                            dltI = im2double(rgb2gray(I)) - im2double(IOld);
                        end
                        Sum(cnt1,1) = sum(sum(abs(dltI)));
                    else
                        dltI = 10000000;
                    end
                    % %                 if log == 1
                    % %                     fprintf(sprintf('\n================================ image difference: %d\n', round(Sum(end))));
                    % %                 end
                    if smallTic == 1
                        tic;
                    end
                    if cnt1 > 70
                        awvbhi = 1;
                    end
                    if cnt1 == 212
                        wevbhk = 1;
                    end
                    if sum(sum(abs(dltI))) >  thr; 0;
                        
                        if 1
                            try
                                try
                                    if ~exist('boxGood')
                                        [mask, box1] = caltagTmp( imresize(I,scale), fullfile(matDir, 'robotPattern2.mat'), false );
                                    end
                                catch
                                    wvhoi = 1;
                                end
                                if ~exist('boxGood')
                                    box = round(box1./scale); boxO = box;
                                else
                                    box = boxGood;
                                    boxO = boxGood1;
                                end
                                %                             cnt1
                                if cnt1 == 181
                                    aeglkj = 1;
                                end
                                if cnt1 == 437
                                    wvdkh = 1;
                                end
                                imValid = I(box(1,2):box(2,2),box(1,1):box(2,1),:);
                                imValidO = I(boxO(1,2):boxO(2,2),boxO(1,1):boxO(2,1),:);
                                %                             if exist('boxGood')
                                % % % % % %                             try
                                ticCal = tic;
                                try
                                    [wPt1,iPt1] = caltag( imValid, fullfile(matDir, 'robotPattern2.mat'), false );
                                catch
                                    sdj = 1;
                                end
                                if isempty(iPt1)
                                    wjcbh = 1;
                                end
                                timeCal(tm2,1) = toc(ticCal);
                                tm2 = tm2 + 1;
                                iPt_use = iPt1 + repmat([box(1,2)-1 box(1,1)-1],size(iPt1,1),1);
                                iPt_Use =[iPt_Use;iPt_use];
                                wPt_Use = [wPt_Use; wPt1];
                                
                                if length(wPt_Use) > 1
                                    idCur = ismember(wPt_Use{end},wPt_Use{end-1},'rows');
                                    idPrv = ismember(wPt_Use{end-1},wPt_Use{end},'rows');
                                    flow = ratio.*(iPt_Use{end}(idCur,[1 2]) - iPt_Use{end-1}(idPrv, [1 2]));
                                    slk = 1;
                                else
                                    flow = zeros(10,2);
                                end
                                Flow = [Flow; norm(mean(flow))];
                                if max(abs(flow(:))) > 10
                                    elkd = 1;
                                end
                                
                                
                                % %                             if ~(isempty(Flow) || length(Flow) <= n2+1)
                                % %                                 if max(Flow(end-n2:end,1)) > pixDiff  % || length(Flow) == 0
                                % %                                     LargeSkipFlag = [LargeSkipFlag; 0];
                                % %                                     skipNum = initSkipNum;
                                % %                                 else
                                % %                                     skipNum = initSkipNum;
                                % %                                     LargeSkipFlag = [LargeSkipFlag; 1];
                                % %                                 end
                                % %                                 if length(LargeSkipFlag) > 10 && sum(LargeSkipFlag(end-n3:end)) == n3 + 1
                                % %                                     skipNum = skipTimes*initSkipNum;
                                % %                                     ratio = ratioLarge;
                                % %
                                % %                                 end
                                % %                             end
                                % %                             if length(iPt_Use) <= n
                                % %                                 boxGood1 = round([[max(min(iPt_use(:,2))-margin,1) max(min(iPt_use(:,1))-margin,1)];[min(max(iPt_use(:,2))+margin,size(I,2)) min(max(iPt_use(:,1))+margin,size(I,1))]]);
                                % %                                 iPt_useTmp2 = iPt_use + repmat(mean(flow),size(iPt_use,1),1);
                                % %                                 boxGood = round([[max(min(iPt_useTmp2(:,2))-margin,1) max(min(iPt_useTmp2(:,1))-margin,1)];[min(max(iPt_useTmp2(:,2))+margin,size(I,2)) min(max(iPt_useTmp2(:,1))+margin,size(I,1))]]);
                                % %                             else
                                % %                                 iPt_useTmp = cell2mat(iPt_Use(end-n+1:end));
                                % %                                 iPt_useTmp2 = iPt_useTmp + repmat(mean(flow),size(iPt_useTmp,1),1);
                                % %                                 boxGood1 = round([[max(min(iPt_useTmp(:,2))-margin,1) max(min(iPt_useTmp(:,1))-margin,1)];[min(max(iPt_useTmp(:,2))+margin,size(I,2)) min(max(iPt_useTmp(:,1))+margin,size(I,1))]]);
                                % %                                 boxGood = round([[max(min(iPt_useTmp2(:,2))-margin,1) max(min(iPt_useTmp2(:,1))-margin,1)];[min(max(iPt_useTmp2(:,2))+margin,size(I,2)) min(max(iPt_useTmp2(:,1))+margin,size(I,1))]]);
                                % %                             end
                                
                                asdv = 1;
                                %                             boxGood = box;
                                % % % % % % %                             catch
                                % % % % % % %                                 wvhoi = 1;
                                % % % % % % %                             end
                                
                                if size(iPt1,1) ~= 12
                                    werfjlk = 1;
                                end
                                
                                %             [wPt,iPt] = caltag( I, fullfile(matDir, 'mypattern2.mat'), false );
                                iPt = iPt1 + repmat([box(1,2)-1 box(1,1)-1],size(iPt1,1),1);
                                wPt1 = wPt1./min(wPt1(wPt1 ~=0)).*squareSize;
                                
                                wPt1 = wPt1-repmat([squareSize*0 0],size(wPt1,1),1);
                                
                                xyzCB = [wPt1 zeros(size(wPt1,1),1)];
                                
                                % % % % % %                             ptUndistRight = normalize_pixel(iPt(:,[2 1])',camParam.foc,camParam.cen,camParam.kc,0);
                                % % % % % %                             ptUndist = intrMatRight*[ptUndistRight;ones(1,size(ptUndistRight,2))];
                                % % % % % %                             ptUndist = [ptUndist(1,:)./ptUndist(3,:);ptUndist(2,:)./ptUndist(3,:)];
                                % % % % % %
                                if use_fisheye == 0
                                    ptUndistRight = normalize_pixel(iPt(:,[2 1])',camParam.foc,camParam.cen,camParam.kc,0);
                                    % % %                                 ptUndist3D = [ptUndistRight; ones(1,size(ptUndistRight,2))]';
                                    ptUndist = intrMatRight*[ptUndistRight;ones(1,size(ptUndistRight,2))];
                                    ptUndist = [ptUndist(1,:)./ptUndist(3,:);ptUndist(2,:)./ptUndist(3,:)];
                                else
                                    [ptUndist, ptUndist3D] = remapFishEyePix(iPt(:,[2 1])', intrMatRight, oCamModel);
                                    ptUndist3D_ = ptUndist3D';
                                    ptUndistRight = ptUndist3D(1:2,:);
                                end
                                
                                if 0
                                    figure,subplot(1,2,1);imshow(I); hold on; plot(iPt(:,2),iPt(:,1),'.g');title('original image');
                                    subplot(1,2,2);imshow(nim);hold on;plot(ptUndist(1,:),ptUndist(2,:),'.g');title('undistorted image');
                                end
                                
                                %                             camPosVec = posest(ptUndist', xyzCB, 0.95, intrMatRight, 'repr_err');
                                Id = ismember(xyzCB, [0 0 0], 'rows');
                                if sum(Id) ~= 0
                                    afkhj = 1;
                                end
                                camPosVec = posest(ptUndist(:,~Id)', xyzCB(~Id,:), 0.95, intrMatRight, 'repr_err');
                                camPose{cnt1,1} = [rodrigues(camPosVec(1:3)) camPosVec(4:6);0 0 0 1];
                                camPoseMat(cnt1,:) = [reshape(camPose{cnt1,1}(1:3,1:3),1,9) camPosVec(4:6)' k];
                                if size(camPoseMat,1) == 286
                                    agkh = 1;
                                end
                                
                                
                                %                             if size(camPoseMat,1) > 1
                                %                                 dltT =  [R t';0 0 0 1]*inv([reshape(camPoseMat(end-1,1:9),3,3) camPoseMat(end-1,10:12)';0 0 0 1]);
                                %                                 dltr = dltT(1:3,1:3);
                                %                                 dltt = dltT(1:3,end);
                                %                                 prvPt = [camPoseMat(end-1,10);0;camPoseMat(end-1,12)];
                                %                                 curPt = [camPoseMat(end,10);0;camPoseMat(end,12)];
                                %                                 dltAng = rad2deg(norm(rodrigues(dltr)));
                                %                                 if norm(rodrigues(roty(dltAng)) - rodrigues(dltr)) < norm(rodrigues(roty(-dltAng)) - rodrigues(dltr))
                                %                                     accumAng = [accumAng; accumAng(end) + dltAng];
                                %                                 else
                                %                                     accumAng = [accumAng; accumAng(end) - dltAng];
                                %                                 end
                                %
                                %                             end
                                
                                
                                
                                
                                mappedPt = H*[ptUndistRight; ones(1,size(ptUndistRight,2))];
                                mappedPt = [mappedPt(1,:)./mappedPt(3,:);mappedPt(2,:)./mappedPt(3,:)];
                                
                                % %                         if sum(ismember([321 0],wPt1,'rows')) == 1  % sum(ismember([0 0],wPt1,'rows')) == 1
                                if sum(ismember([squareSize*1 0]-[squareSize*0 0],wPt1,'rows')) == 1  % sum(ismember([0 0],wPt1,'rows')) == 1
                                    %                             poseMat(cnt2,:) = [mappedPt(:,1)' cnt1];
                                    %                             poseMat(cnt2,:) = [mappedPt(:,ismember(wPt1,[321 0],'rows'))' cnt1];
                                    poseMat(cnt2,:) = [mappedPt(:,ismember(wPt1,[squareSize*1 0]-[squareSize*0 0],'rows'))' cnt1];
                                    if log == 1
                                        fprintf(sprintf('\n### frame %d, [%0.2f %0.2f]\n', cnt1, poseMat(cnt2,1:2)));
                                    end
                                    cnt2 = cnt2 + 1;
                                end
                                % %                         if cnt1 > 1
                                % %                             try
                                % %                                 dltR = rodrigues(camPosVec(1:3))'*camPose{cnt1-1}(1:3,1:3);
                                % %                             catch
                                % %                                 wvdhkj = 1;
                                % %                             end
                                % %                             dltRotAxis = rodrigues(dltR)./norm(rodrigues(dltR));
                                % %                             vdj = 1;
                                % %                         end
                                if draw == 1
                                    figure(10), clf, subplot(2,2,1);imshow(imValid);hold on; plot(iPt1(:,2),iPt1(:,1),'-r','LineWidth',5,'MarkerSize',5); plot(iPt1(:,2),iPt1(:,1),'*g','LineWidth',5,'MarkerSize',5); plot(iPt1(1,2),iPt1(1,1),'*b','LineWidth',5,'MarkerSize',5);title(sprintf('frame %d',cnt1));
                                    subplot(2,2,2);imshow(I); hold on; plot(iPt(:,2),iPt(:,1),'-r','LineWidth',2,'MarkerSize',2); plot(iPt(:,2),iPt(:,1),'*g','LineWidth',2,'MarkerSize',2); plot(iPt(1,2),iPt(1,1),'*b','LineWidth',2,'MarkerSize',2);title(sprintf('skipNum %d',skipNum));
                                    subplot(2,2,3);imshow(imValidO);
                                    %                                 drawnow;
                                end
                            catch
                                wvohj = 1;
                            end
                            if draw == 1
                                subplot(2,2,4);imshow(I); hold on;  plot(box(:,1),box(:,2),'*r','MarkerSize',5,'LineWidth',5); %  plot(iPt(:,2),iPt(:,1),'-r','LineWidth',2,'MarkerSize',2); plot(iPt(:,2),iPt(:,1),'*g','LineWidth',2,'MarkerSize',2); plot(iPt(1,2),iPt(1,1),'*b','LineWidth',2,'MarkerSize',2);
                                drawnow;
                            end
                            
                            if 0  %draw == 1
                                figure(20),clf;subplot(1,2,1);imshow(I); hold on; plot(iPt(:,2),iPt(:,1),'.g');title('original image');
                                subplot(1,2,2);imshow(nim);hold on;plot(ptUndist(1,:),ptUndist(2,:),'.g');title('undistorted image');
                                drawnow;
                                
                            end
                            
                            
                            
                            
                            %             WPt{k,1} = wPt;
                            %             IPt{k,1} = iPt;
                            % % % % % %                 try
                            % % % % % %                     figure(i), imshow(I); hold on; plot(iPt(:,2),iPt(:,1),'-r','LineWidth',2); plot(iPt(:,2),iPt(:,1),'*g','LineWidth',2,'MarkerSize',10); plot(iPt(1,2),iPt(1,1),'*b','LineWidth',2,'MarkerSize',10);title(sprintf('frame %d',k));
                            % % % % % %                     hold off;
                            % % % % % %                     drawnow;
                            % % % % % %                 catch
                            % % % % % %                     fprintf('\n\n');
                            % % % % % %                     fprintf(sprintf('#### no corner at frame %d ####',k));
                            % % % % % %                     fprintf('\n\n\n');
                            % % % % % %                 end
                            if ~(isempty(Flow) || length(Flow) <= n2+1)
                                if max(Flow(end-n2:end,1)) > pixDiff  % || length(Flow) == 0
                                    LargeSkipFlag = [LargeSkipFlag; 0];
                                    skipNum = initSkipNum;
                                else
                                    skipNum = initSkipNum;
                                    LargeSkipFlag = [LargeSkipFlag; 1];
                                end
                                if length(LargeSkipFlag) > 10 && sum(LargeSkipFlag(end-n3:end)) == n3 + 1
                                    skipNum = skipTimes*initSkipNum;
                                    ratio = ratioLarge;
                                    margin =  marginLarge; marginSmall;
                                else
                                    margin = marginSmall;
                                end
                            end
                            if length(iPt_Use) <= n
                                boxGood1 = round([[max(min(iPt_use(:,2))-margin,1) max(min(iPt_use(:,1))-margin,1)];[min(max(iPt_use(:,2))+margin,size(I,2)) min(max(iPt_use(:,1))+margin,size(I,1))]]);
                                if skipNum == skipTimes*initSkipNum;
                                    iPt_useTmp2 = iPt_use + ratio2.*repmat(mean(flow),size(iPt_use,1),1);
                                else
                                    iPt_useTmp2 = iPt_use + repmat(mean(flow),size(iPt_use,1),1);
                                end
                                boxGood = round([[max(min(iPt_useTmp2(:,2))-margin,1) max(min(iPt_useTmp2(:,1))-margin,1)];[min(max(iPt_useTmp2(:,2))+margin,size(I,2)) min(max(iPt_useTmp2(:,1))+margin,size(I,1))]]);
                            else
                                iPt_useTmp = cell2mat(iPt_Use(end-n+1:end));
                                if skipNum == skipTimes*initSkipNum;
                                    iPt_useTmp2 = iPt_useTmp + ratio2.*repmat(mean(flow),size(iPt_useTmp,1),1);
                                else
                                    try
                                        iPt_useTmp2 = iPt_useTmp + repmat(mean(flow),size(iPt_useTmp,1),1);
                                    catch
                                        gelwhk = 1;
                                        iPt_useTmp2 = iPt_useTmp + repmat(mean(flow',2)',size(iPt_useTmp,1),1);
                                    end
                                end
                                
                                boxGood1 = round([[max(min(iPt_useTmp(:,2))-margin,1) max(min(iPt_useTmp(:,1))-margin,1)];[min(max(iPt_useTmp(:,2))+margin,size(I,2)) min(max(iPt_useTmp(:,1))+margin,size(I,1))]]);
                                boxGood = round([[max(min(iPt_useTmp2(:,2))-margin,1) max(min(iPt_useTmp2(:,1))-margin,1)];[min(max(iPt_useTmp2(:,2))+margin,size(I,2)) min(max(iPt_useTmp2(:,1))+margin,size(I,1))]]);
                            end
                            
                            
                            
                            cnt1 = cnt1 + 1;
                            IOld = I;
                        else
                            % %             outputDir = fullfile(inputDir, 'calib');
                            % %             MakeDirIfMissing(outputDir);
                            k
                            % %             figure(i), imshow(frame);title(sprintf('frame %d',k));drawnow;
                            imwrite(frame, fullfile(outputDir,sprintf('calib_frame_%06d.png', k)));% 保存帧
                            
                        end
                        % %                 toc;
                    else
                        if log == 1
                            fprintf(sprintf('\n### frame %d, no motion\n', cnt1));
                        end
                        cnt1 = cnt1 + 1;
                        IOld = I;
                    end
                    if smallTic == 1
                        time(tm,1) = toc(tic1);
                        tm = tm + 1;
                    end
                end
            end
            %         toc;
            clear obj
            
            %             save(fullfile(inputDir,sprintf('data_%02d.mat',dataNum)));
        end
        if bigTic == 1
            timeAll = toc(ticAll);
        end
    else
        
        if 0
            [camPoseMat, roboStamp, dist, logg] = readBeiDump(inputDir);
        else
            % %             [camPoseMat, roboStamp, dist, logg] = readBeiDump2(inputDir,paraDir);
            [camPoseMat, roboStamp, dist, logg] = readBeiDump3(inputDir,paraDir,homoDir);
        end
        camPoseMat = [camPoseMat [1:size(camPoseMat,1)]'];
        poseMat = [camPoseMat(:,10:12) [1:size(camPoseMat,1)]'];
        
        
        
        
    end
    %     clear readBei
    % %     clear drawResult
    save(fullfile(inputDir,sprintf('data_%02d.mat',dataNum)));
else
    load(fullfile(inputDir,sprintf('data_%02d.mat',dataNum)));
    %     drawResult = 0;
end

if drawResult == 1
    % % % figure, plotPath(camPoseMat);
    figure,subplot(1,2,1);plot3(camPoseMat(camPoseMat(:,10) ~= 0,10),camPoseMat(camPoseMat(:,10) ~= 0,11),camPoseMat(camPoseMat(:,10) ~= 0,12),'-r');axis equal; subplot(1,2,2),plot(poseMat(:,1),poseMat(:,2),'-r');axis equal;axis([min(poseMat(:,1))-10 max(poseMat(:,1))+10 min(poseMat(:,2))-10 max(poseMat(:,2))+10]);
end
if 0
    rad2deg(norm(rodrigues(reshape(camPoseMat(173,1:9),3,3)'*reshape(camPoseMat(209,1:9),3,3))))
end
id = [];
if readBei == 0
    for o = 1 : length(camPose)
        if ~isempty(camPose{o})
            id = [id; o];
        end
    end
else
    id = [1:size(camPoseMat,1)]';
end
posestCamPose = camPoseMat(id,10:12);
poseMat2 = poseMat;
[iddd,id1,id2] = intersect(id,poseMat(:,end));
id1  = find(ismember(id,iddd));
id2  = find(ismember(poseMat(:,end),iddd));
camPoseMatTmp = [camPoseMat(iddd,:) poseMat(id2,1:2)];
idd = find(camPoseMatTmp(:,10) ~= 0);
% idd = 1:length(id1);
Id1 = id1(idd);
Id2 = id2(idd);
if readBei == 0
    poseMat2(:,end) = 0; max(camPoseMat(Id1,end));
else
    poseMat2(:,end) = [];
end
% % posestPose = camPoseMat(Id1,10:end);
posestPose = camPoseMatTmp(idd,10:12);
homoPose = poseMat2(Id2,:);
rt = AlignCoord(homoPose, posestPose);
if drawResult == 1
    figure,subplot(1,2,1);plot3(posestPose(:,1),posestPose(:,2),posestPose(:,3),'.r');axis equal; subplot(1,2,2),plot(homoPose(:,1),homoPose(:,2),'.r');axis equal;axis([min(homoPose(:,1))-10 max(homoPose(:,1))+10 min(homoPose(:,2))-10 max(homoPose(:,2))+10]);
end
% % posestPoseMappedd = (rt(1:3,1:3)*posestPose' + repmat(rt(:,4),1,size(posestPose,1)))';
% % if drawResult == 1
% %     figure,subplot(1,2,1);plot3(posestPoseMappedd(:,1),posestPoseMappedd(:,2),posestPoseMappedd(:,3),'.r');axis equal; subplot(1,2,2),plot(homoPose(:,1),homoPose(:,2),'.r');axis equal;axis([min(homoPose(:,1))-10 max(homoPose(:,1))+10 min(homoPose(:,2))-10 max(homoPose(:,2))+10]);
% %
% %     fig_3D3_pair(posestPoseMappedd',homoPose');
% % end


% % % % % % % % if 0
% % % % % % % %     posestPoseMapped = posestPoseMappedd;
% % % % % % % %     posestPoseMapped = posestPoseMapped - repmat(posestPoseMapped(1,:),size(posestPoseMapped,1),1);
% % % % % % % % else
% % % % % % % %     posestCamPoseMapped = (rt(1:3,1:3)*posestCamPose' + repmat(rt(:,4),1,size(posestCamPose,1)))';
% % % % % % % %     posestPoseMapped2 = posestCamPoseMapped - repmat(posestCamPoseMapped(1,:),size(posestCamPoseMapped,1),1);
% % % % % % % % end

% % [BB, ~, inliers] = ransacfitplane(camPoseMat(:,10:12)', 0.2);
B = fitplane(camPoseMat(:,10:12)'); B = B./norm(B(1:3)); N = B(1:3);
rotAxis1 = cross(N,[0;0;1])./norm(cross(N,[0;0;1]));
rotAng = deg2rad(CalcDegree(N,[0;0;1]));
% % if norm(rodrigues(rotAng*rotAxis1)*N - [0;0;1]) < norm(rodrigues(-rotAng*rotAxis1)*N - [0;0;1])
if CalcDegree(rodrigues(rotAng*rotAxis1)*N, [0;0;1]) < CalcDegree(rodrigues(-rotAng*rotAxis1)*N , [0;0;1])
    rBei = rodrigues(rotAng*rotAxis1);
else
    rBei = rodrigues(-rotAng*rotAxis1);
end
camposeBei = rBei*camPoseMat(:,10:12)';
tBei =  [0;0;-mean(camposeBei(3,:))];[0;0;0];
Tbei = [rBei tBei;0 0 0 1];


camposeBeiTest = rBei*camPoseMat(:,10:12)' + repmat(tBei,1,size(camposeBei,2));
if 0
    tt = camPoseMat(:,10:12)' - repmat([0;0;mean(camPoseMat(:,12))],1,401);
    fig_3D3_pair(tt,tt);
    fig_3D3_pair(camposeBeiTest,camposeBeiTest);
    D = fitplane(rBei*camPoseMat(:,10:12)');
    D./norm(D(1:3))
end

if readBei == 1
    rt = [rBei tBei];
    rt = [eye(3) [0 0 0]'];
end
try
    posestPoseMappedd = (rt(1:3,1:3)*posestPose' + repmat(rt(:,4),1,size(posestPose,1)))';
catch
    sahfkl = 1;
end
if drawResult == 1
    figure,subplot(1,2,1);plot3(posestPoseMappedd(:,1),posestPoseMappedd(:,2),posestPoseMappedd(:,3),'.r');axis equal; subplot(1,2,2),plot(homoPose(:,1),homoPose(:,2),'.r');axis equal;axis([min(homoPose(:,1))-10 max(homoPose(:,1))+10 min(homoPose(:,2))-10 max(homoPose(:,2))+10]);
    
    fig_3D3_pair(posestPoseMappedd',homoPose');
end

if 0
    posestPoseMapped = posestPoseMappedd;
    posestPoseMapped = posestPoseMapped - repmat(posestPoseMapped(1,:),size(posestPoseMapped,1),1);
else
    posestCamPoseMapped = (rt(1:3,1:3)*posestCamPose' + repmat(rt(:,4),1,size(posestCamPose,1)))';
    posestPoseMapped2 = posestCamPoseMapped - repmat(posestCamPoseMapped(1,:),size(posestCamPoseMapped,1),1);
end

accumAng1 = 0;
dltT0 = [rt;0 0 0 1];
camPoseMat2 = camPoseMat(id,:);
if drawResult == 1
    figure, plotPath(camPoseMat2);
end
%  ToldTmp1 = [reshape(camPoseMat2(1,1:9),3,3) camPoseMat2(1,10:12)';0 0 0 1];
%  TnewTmp1 = dltT0*ToldTmp1;
%  ToldTmp2 = [reshape(camPoseMat2(50,1:9),3,3) camPoseMat2(1,10:12)';0 0 0 1];
%  TnewTmp1 = dltT0*ToldTmp1;
for h = 1 : size(camPoseMat2,1)
    
    
    if h == 194
        asfkj = 1;
    end
    Told = [reshape(camPoseMat2(h,1:9),3,3) camPoseMat2(h,10:12)';0 0 0 1];
    %    dltT = [rt;0 0 0 1];
    Tnew = dltT0*Told;
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
            if norm(rodrigues(rotz(dltAng)) - rodrigues(dltr)) < norm(rodrigues(rotz(-dltAng)) - rodrigues(dltr))
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
if 0
    if accumAng1(end) < 0
        accumAng1 = -accumAng1;
    end
end
% % figure,plot(accumAng,'.');title('ceiling');
dltDistRoboTemp = abs(diff(accumAng1));
dltDistRoboFiltTemp = medfilt1(dltDistRoboTemp,(2*WindowSize - 1));
%     dltDistRobo = medfilt1(dltDistRobo1,(2*10 - 1));
angThr = 2.2;

[indexCellRobo,idxRobo] = splitIndex2(find(dltDistRoboFiltTemp > angThr));
[indexCellRobo2,idxRobo2] = splitIndex2(find(dltDistRoboFiltTemp <= angThr));







% idEval = 1;
for w = 1 : length(indexCellRobo) + 1
    if w == 1
        idEval{w,1} = [1 round(mean(indexCellRobo{w}))];
    elseif w == length(indexCellRobo) + 1
        idEval{w,1} = [idEval{w-1,1}(2) length(accumAng1)];
    else
        idEval{w,1} = [idEval{w-1,1}(2) round(mean(indexCellRobo{w}))];
    end
end
try
    idx = cell2mat(idEval');
    idx = unique(idx);
    idxAll = zeros(1,length(dltDistRoboFiltTemp) + 1);
    idxAll(idx) = 10;
    idxAll2 = zeros(1,length(dltDistRoboFiltTemp) + 1);
    idxAll2(idx) = max(abs(accumAng1));
catch
    dsargh = 1;
end
try
    for iii = 1 : length(idEval)
        lenRobo(iii,:) = [posestCamPoseMapped(idEval{iii,1}(1),1:2) posestCamPoseMapped(idEval{iii,1}(2),1:2) norm( posestCamPoseMapped(idEval{iii,1}(1),1:2) - posestCamPoseMapped(idEval{iii,1}(2),1:2) )];
        
    end
catch
    asvkhj = 1;
end
accumAng = 0;

try
    for hh = 1 : length(idEval)
        ToldTmp1 = [reshape(camPoseMat2(idEval{hh}(1),1:9),3,3) camPoseMat2(idEval{hh}(1),10:12)';0 0 0 1];
        TnewTmp1 = dltT0*ToldTmp1;
        ToldTmp2 = [reshape(camPoseMat2(idEval{hh}(2),1:9),3,3) camPoseMat2(idEval{hh}(2),10:12)';0 0 0 1];
        TnewTmp2 = dltT0*ToldTmp2;
        
        
        dltTRef =  TnewTmp2*inv(TnewTmp1);
        rotax = rodrigues(dltTRef(1:3,1:3))./norm(rodrigues(dltTRef(1:3,1:3)));
        if hh == 1
            rotAxis(:,hh) = rotax;
        else
            if norm(rotax - rotAxis(:,1)) < 1 %0.5
                rotAxis(:,hh) = rotax;
            else
                rotAxis(:,hh) = -rotax;
            end
        end
    end
catch
    %% 20181023
    rotAxis = [0;0;-1];
end
pt3d = camPoseMatNew(:,10:12);
pt3d(:,3) = pt3d(:,3) + repmat(1000,size(pt3d,1),1);
[BBB, ~, inlierss] = ransacfitplane(pt3d', 100);
BBB = BBB./norm(BBB(1:3));




if norm(BBB(1:3) - rotAxis(:,1)) < 1
    rotAxis2 = repmat(BBB(1:3),1,size(rotAxis,2));
else
    rotAxis2 = repmat(-BBB(1:3),1,size(rotAxis,2));
end
loopcnt = 1;
if 0
    for h = 1 : size(camPoseMatNew,1)
        
        %     if size(camPoseMatNew,1) > 1
        if h > 1
            % %         dltT =  Tnew*inv([reshape(camPoseMatNew(end-1,1:9),3,3) camPoseMatNew(end-1,10:12)';0 0 0 1]);
            dltT =  [reshape(camPoseMatNew(h,1:9),3,3) camPoseMatNew(h,10:12)';0 0 0 1]*inv([reshape(camPoseMatNew(1,1:9),3,3) camPoseMatNew(1,10:12)';0 0 0 1]);
            dltr = dltT(1:3,1:3);
            dltt = dltT(1:3,end);
            prvPt = [camPoseMatNew(end-1,10);0;camPoseMatNew(end-1,12)];
            curPt = [camPoseMatNew(end,10);0;camPoseMatNew(end,12)];
            prvPt = [camPoseMatNew(end-1,10);0;camPoseMatNew(end-1,12)];
            curPt = [camPoseMatNew(end,10);0;camPoseMatNew(end,12)];
            errSlam = dltr*prvPt + dltt - curPt;
            dltAng = rad2deg(norm(rodrigues(dltr)));
            
            % %         if dltAng - accumAng(end) < -50
            % %             dltAng = 180 + dltAng;
            % %         end
            for ee = 1 : length(idEval)
                if ismember(h, [idEval{ee}(1) : idEval{ee}(2)]);
                    rotaxis = rotAxis2(:,ee);
                    pos = ee;
                    break;
                end
            end
            if length(accumAng) >= 34
                dsvhj = 1;
            end
            if length(accumAng) >= 743
                asid = 1;
            end
            
            if dltAng > 0.2; %0.2 1
                if norm(rodrigues(dltr)./norm(rodrigues(dltr)) - rotaxis) > 1.5 && pos > 1  %find(ismember(h, [idEval{ee}(1) : idEval{ee}(2)]))
                    % %             if 360*loopcnt-dltAng - accumAng(end) < -100
                    % %                 loopcnt = loopcnt + 1;
                    % %             end
                    dltAng  = 360*1-dltAng;  %360*loopcnt-dltAng;
                end
            end
            % % % % % % % % %         if pos == 1
            % % % % % % % % %             if 1
            % % % % % % % % %                 if norm(rodrigues(rotz(dltAng)) - rodrigues(dltr)) < norm(rodrigues(rotz(-dltAng)) - rodrigues(dltr))
            % % % % % % % % %                     accumAng1 = [accumAng1; accumAng1(end) + dltAng];
            % % % % % % % % %                 else
            % % % % % % % % %                     accumAng1 = [accumAng1; accumAng1(end) - dltAng];
            % % % % % % % % %                 end
            % % % % % % % % %
            % % % % % % % % %
            % % % % % % % % %             else
            % % % % % % % % %                 dltAng = -dltAng;
            % % % % % % % % %             end
            % % % % % % % % %         end
            if dltAng - accumAng(end) < -180 %-300
                dltAng = dltAng + abs(round((dltAng - accumAng(end))/360))*360;
            end
            %         if norm(rodrigues(dltr)./norm(rodrigues(dltr)) - rotaxis) > 0.5
            %             dltang  = dltAng;
            %         end
            if h > 743
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
else
    accumAng = accumAng1;
end
camPoseMatNew = [camPoseMatNew camPoseMat2(:,end)];


dltDistRoboTempNew = abs(diff(accumAng));
dltDistRoboFiltTempNew = medfilt1(dltDistRoboTempNew,(2*2 - 1));
%     dltDistRobo = medfilt1(dltDistRobo1,(2*10 - 1));
[indexCellRoboNew,idxRoboNew] = splitIndex2(find(dltDistRoboFiltTempNew > angThr));
[indexCellRoboNew2,idxRoboNew2] = splitIndex2(find(dltDistRoboFiltTempNew <= angThr));

IndRange1 = [];
try
    for q = 1 : length(indexCellRoboNew)
        
        if q == 5
            askh = 1;
        end
        indRange1 = indexCellRoboNew{q,1}([1 end]);
        indRange1 = [indRange1(1) - marginInd indRange1(2) + marginInd];
        IndRange1 = [IndRange1; indRange1];
        DistAng1(q,1) = norm(camPoseMatNew(indRange1(1),[10:12]) - camPoseMatNew(indRange1(2),[10:12]));
        DistAng1(q,2) = accumAng(indRange1(2)) - accumAng(indRange1(1));
    end
    
    IndRange2 = [];
    for q = 1 : length(indexCellRoboNew2)
        
        if q == 5
            askh = 1;
        end
        indRange2= indexCellRoboNew2{q,1}([1 end]);
        indRange2 = [indRange2(1) + marginInd indRange2(2) - marginInd];
        IndRange2 = [IndRange2; indRange2];
        DistAng2(q,1) = norm(camPoseMatNew(indRange2(1),[10:12]) - camPoseMatNew(indRange2(2),[10:12]));
        DistAng2(q,2) = accumAng(indRange2(2)) - accumAng(indRange2(1));
    end
catch
    wvik = 1;
    DistAng1 = [];
    DistAng2 = [];
end
if drawResult == 1
    figure,subplot(3,1,1);plot(accumAng,'-');title('ceiling');hold on;plot(idxAll2,'r');
    subplot(3,1,2);plot(dltDistRoboTemp);title(sprintf('before filter, windowsize = %d',WindowSize));
    subplot(3,1,3);plot(dltDistRoboFiltTemp);title(sprintf('after filter, angle threshold = %.3f', angThr));hold on;plot(idxAll,'r');
    
    
    figure,plot(posestCamPoseMapped(:,1),posestCamPoseMapped(:,2));axis equal;hold on;plot([lenRobo(:,1);lenRobo(:,3)],[lenRobo(:,2);lenRobo(:,4)],'*r');
    for iii = 1 : length(idEval)
        text((lenRobo(iii,1)+lenRobo(iii,3))/2,(lenRobo(iii,2)+lenRobo(iii,4))/2,num2str(lenRobo(iii,5)), 'Color',[1 0 1],'FontSize',10,'FontWeight','bold');
    end
end

if 0
    if accumAng(end) < 0
        accumAng = -accumAng;
    end
end
roboPosePoseMatStamp2 = [camPoseMatNew roboStamp(camPoseMatNew(:,end)) -accumAng];

[~,dltDistRobo1] = NormalizeVector(camPoseMatNew(1:end-1,10:12) - camPoseMatNew(2:end,10:12));
dltDistRobo = medfilt1(dltDistRobo1,(2*10 - 1));
if 0
    [indexCellRoboStart,idxRoboStart] = splitIndex2(find(dltDistRobo >5));%5 1.5 8  % roboStartThr));
else
    [indexCellRoboStart,idxRoboStart] = splitIndex2(find(dist > 0.5));%5 1.5 8  % roboStartThr));
end

for hj = 1 : length(indexCellRoboStart)
    if length(indexCellRoboStart{hj}) > 10
        startInd1 = indexCellRoboStart{hj}(1)-0; % -1;
        break;
    end
end

if 0
    good = 0;startInd1Tmp = startInd1;
    while good == 0
        if dltDistRobo1(startInd1Tmp) - dltDistRobo1(startInd1Tmp-1) < 1
            good = 1;
        else
            startInd1Tmp = startInd1Tmp - 1;
        end
    end
    startInd = startInd1Tmp;
else
    try
        startInd = startInd1 + 1; %0; %1; %2
    catch
        startInd = 1;
    end
end

% % Dist = dist(startInd:end);
log = logg(startInd:end,:);
for z = 1 : size(log,1)
    xListt(z,:) = log(z,8:2:end);
    yListt(z,:) = log(z,9:2:end);
    if z > 1
        [~,dist2(z-1,:)] = NormalizeVector([xListt(z,:) - xListt(z-1,:);yListt(z,:) - yListt(z-1,:)]');
    end
    
end
Dist = mean(dist2')';


% %     iid = find(roboPosePoseMatStamp2(:,13) == startInd);

%     posestPoseMapped = posestPoseMapped2(iid:end,:);
posestPoseMapped = posestPoseMapped2(startInd:end,:);


%     roboPosePoseMatStamp = roboPosePoseMatStamp2(iid:end,:);

%% 20190313
if 1
    roboPosePoseMatStamp2(:,10:12) = roboPosePoseMatStamp2(:,10:12) - repmat(roboPosePoseMatStamp2(1,10:12),size(roboPosePoseMatStamp2,1),1);
end


% %    roboPosePoseMatStamp2(:,10:12) = roboPosePoseMatStamp2(:,10:12);   %  - repmat(roboPosePoseMatStamp2(1,10:12),size(roboPosePoseMatStamp2,1),1);
roboPosePoseMatStamp = roboPosePoseMatStamp2(startInd:end,:);

timeStampOrig = roboPosePoseMatStamp(:,14);
roboPosePoseMatStamp(:,14) = roboPosePoseMatStamp(:,14) - roboPosePoseMatStamp(1,14);
roboPosePoseMatStamp(:,14) = roboPosePoseMatStamp(:,14) + 0;  1/20;
roboPosePoseMatStamp(:,15) = roboPosePoseMatStamp(:,15) - roboPosePoseMatStamp(1,15);



% save(fullfile(inputDir,sprintf('roboPoseStamp_%02d.mat',i)),'roboPosePoseMatStamp');
save(fullfile(inputDir,sprintf('roboPoseStamp_%02d.mat',1)),'roboPosePoseMatStamp');
if 0
    try
        bb = medfilt1(roboPosePoseMatStamp(:,15),3*3,1);
        aa = abs(diff(bb));
        [indexCell__, ~] = splitIndex2(find(aa < 1));
        Ind = [];
        for j = 1 : length(indexCell__)
            Ind(j,1) = round(mean(indexCell__{j}));
        end
        figure,plot(roboPosePoseMatStamp(:,14),roboPosePoseMatStamp(:, 15),'.');hold on;plot(roboPosePoseMatStamp(Ind,14),roboPosePoseMatStamp(Ind, 15),'.g');
        save(fullfile(inputDir,sprintf('roboPoseStamp_%02d_.mat',1)),'roboPosePoseMatStamp','Ind','indexCell__');
    catch
        asvnjk = 1;
    end
end
wsdvkjb = 1;
% % for hh = 2 : size(camPoseMatNew,1)
% %
% %         dltT =  [R t';0 0 0 1]*inv([reshape(camPoseMatNew(end-1,1:9),3,3) camPoseMatNew(end-1,10:12)';0 0 0 1]);
% %     dltr = dltT(1:3,1:3);
% %     dltt = dltT(1:3,end);
% %     prvPt = [camPoseMatNew(end-1,10);0;camPoseMatNew(end-1,12)];
% %     curPt = [camPoseMatNew(end,10);0;camPoseMatNew(end,12)];
% %     dltAng = rad2deg(norm(rodrigues(dltr)));
% %     if norm(rodrigues(roty(dltAng)) - rodrigues(dltr)) < norm(rodrigues(roty(-dltAng)) - rodrigues(dltr))
% %         accumAng = [accumAng; accumAng(end) + dltAng];
% %     else
% %         accumAng = [accumAng; accumAng(end) - dltAng];
% %     end
% %
% %
% %
% %
% % end



% % if size(camPoseMat,1) > 1
% %     dltT =  [R t';0 0 0 1]*inv([reshape(camPoseMat(end-1,1:9),3,3) camPoseMat(end-1,10:12)';0 0 0 1]);
% %     dltr = dltT(1:3,1:3);
% %     dltt = dltT(1:3,end);
% %     prvPt = [camPoseMat(end-1,10);0;camPoseMat(end-1,12)];
% %     curPt = [camPoseMat(end,10);0;camPoseMat(end,12)];
% %     dltAng = rad2deg(norm(rodrigues(dltr)));
% %     if norm(rodrigues(roty(dltAng)) - rodrigues(dltr)) < norm(rodrigues(roty(-dltAng)) - rodrigues(dltr))
% %         accumAng = [accumAng; accumAng(end) + dltAng];
% %     else
% %         accumAng = [accumAng; accumAng(end) - dltAng];
% %     end
% %
% % end


% % % [DistAng1, DistAng2,angRangeInd] = getDistAngInfo2(roboPosePoseMatStamp(:,end), roboPosePoseMatStamp(:,10:12), 0.7,10,5,8,1,Dist);

save(fullfile(inputDir,sprintf('posest_%02d.mat',dataNum)),'posestPoseMapped');
end

function rt = AlignCoord(matchPtBody,matchPtVR)
bodyCtrMass = mean(matchPtBody);
vrCtrMass = mean(matchPtVR);
bodyPt = matchPtBody - repmat(bodyCtrMass,size(matchPtBody,1),1);
vrPt = matchPtVR - repmat(vrCtrMass,size(matchPtVR,1),1);
[U,S,V] = svd(vrPt'*bodyPt);
H = U*S*V';
X = V*U';
% %     [U,S,V] = svd(bodyPt'*vrPt);
% %     H = U*S*V';
% %     X = U'*V;
if abs(det(X) - (-1)) < 0.0001
    %     if X(2,2) < 0
    V(:,3) = -V(:,3);
    R = V*U';
else
    R = X;
    
end
% %     R(3,3) = -R(3,3);
t = bodyCtrMass' - R*vrCtrMass';
rt = [R t];
end