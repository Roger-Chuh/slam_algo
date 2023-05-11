function ReplayTraceLite(folderName)



replayDir = fullfile(pwd, folderName);
delete(fullfile(pwd,'\temp\*.png'));
delete(fullfile(pwd,'\dispErr\*.png'));
delete(fullfile(pwd,'\prob\*.png'));
delete(fullfile(pwd,'\dump\*.png')); delete(fullfile(pwd,'\dump\*.mat'));
delete(strcat(fullfile(pwd,folderName),'\TraceData*'));

replayDirInfo = dir(fullfile(replayDir,'KeySeq*'));

global OUTPUT_ROOT RECORDED_BIN_ROOT DUMP_ROOT RESULT_ROOT DRAWPOLYGON ANGLEONLY SHOWPOLYGON USEGOLDENDISP
 



 



assert(~isempty(OUTPUT_ROOT) && ~isempty(RECORDED_BIN_ROOT) && ~isempty(DUMP_ROOT), 'The directories are not correctly set');

% % fclose all;
% % close all;

csaParamFilePath = fullfile(DUMP_ROOT, 'coordsys_alignment.txt');
csa = CoordSysAligner;
Load(csa, csaParamFilePath);

robotCfgFilePath = fullfile(OUTPUT_ROOT, 'robot_config.prototxt');
robotConfig = ReadRobotConfig(robotCfgFilePath);

camParamFilePath = fullfile(DUMP_ROOT, 'camera_param.txt');
cam = BCStereoCameraModel(camParamFilePath);
CvtToRectifiedModel(cam);
cibfr = CamImuBinFileReader;
SetInputDevice(cam, cibfr);
camImuBinFilePath = fullfile(RECORDED_BIN_ROOT, 'camera_imu.bin');
OpenInput(cam, camImuBinFilePath);

bsbfr = BodySensorBinFileReader;
bodySensorBinFilePath = fullfile(RECORDED_BIN_ROOT, 'bodysensor.bin');
bodySensorRespFilePath = fullfile(RECORDED_BIN_ROOT, 'bodysensor_response.bin');
Open(bsbfr, bodySensorBinFilePath, bodySensorRespFilePath);

bsl = BodySensorLocalizer(GetBodySensorDataDecoder(bsbfr), robotConfig.bodysensor_localizer_param);
vsl = VisualLocalizer(cam, csa, robotConfig.tracking_config);
BsSampFrame = [];
PoseWheel = [];
EulerWheel = [];
PoseWheel_All = [];
EulerWheel_All  = [];
curImgTimeStamp = 0;
curFrmTimeStamp = 0;
wheelId = [];
wheelTimeAng = [];


if isempty(vsl.scaleLvl)
    vsl.scaleLvl = 0;
end


keySeqNum = 1;

if keySeqNum == 1
    needReplace = 0;
else
    needReplace = 1;
end
  
   
  


DRAWPOLYGON = true; false;
ANGLEONLY = true;
SHOWPOLYGON = false;
USEGOLDENDISP = false; true;false;

vsl.switchDepth = false; true; false;
depthScale = 1; 1.04;1.05; 1;
DltAng = [];
DiffPixTrace = [];
DiffPixTraceGT = [];
splitData = [];

visionTimeAng = [];
visionTimeAngRef = [];
gyroTimeAng = [];
wheelTimeAng = [];
RwheelCur = eye(3);
RgyroCur = eye(3);
EulerGyro = [];
GyroAngAccum = 0;
PoseGyro = [];
useGyro = true; false; true;
frameID = [];
imgBuf = {};
depthGTBuf = {};

 
saveTrace = false; false; true;
checkGT = false;

b2c = vsl.coordSysAligner.pctBody2Cam(1, 1).transformMat;
intrMat = Get(vsl.camModel, 'PinholeIntrMat', 1,vsl.scaleLvl);
intrMat0 = Get(vsl.camModel, 'PinholeIntrMat', 1,0);

for i = keySeqNum : length(replayDirInfo)
%     RwheelPrv = RwheelCur;
%     RgyroPrv = RgyroCur;
    curKeySeqData = load(fullfile(replayDir,replayDirInfo(i).name));
    try
%         wheelTimeAng = curKeySeqData.wheelTimeAng;
%         gyroTimeAng = curKeySeqData.gyroTimeAng;
%         visionTimeAng = curKeySeqData.visionTimeAng;
    catch
        asvdhkj = 1;
    end
    for j = 1 : size(curKeySeqData.KeySeq.imgHeader,1)-1
        if needReplace
            if keySeqNum > 1 && j ==1
                prvKeySeqData = load(fullfile(replayDir,replayDirInfo(i-1).name));
                bsl = prvKeySeqData.bsl;
                vsl = prvKeySeqData.vsl;
                
%                 vsl.pureRotationAngPredictor.configParam.pure_rot_reproject_outlier_threshold = 2;
                vsl.configParam.disparity_error = 1;
                vsl.configParam.depth_uncertainty_range = [-100 300];
                vsl.configParam.theta_range = deg2rad([-0.5 0.5]);
                vsl.configParam.polygon_margin = 2;
                vsl.configParam.polygon_inlier_thresh = 0.2;
                
                
                
                
%                 vsl.configParam.feature_num_drop_rate_thresh = 0.65;
                %             vsl.prevImgL =
                needReplace = 0;
            end
        end
        
        RwheelPrv = RwheelCur;
        RgyroPrv = RgyroCur;
        
        bsSampFrame = curKeySeqData.KeySeq.bsSampFrame{j,1};
        imgHeader = curKeySeqData.KeySeq.imgHeader{j,1};
        imgL = curKeySeqData.KeySeq.imgL{j,1};
        imgR = curKeySeqData.KeySeq.imgR{j,1};
        if ndims(imgL) ~= 3
           imgL = cat(3, imgL, imgL, imgL); 
           imgR = cat(3, imgR, imgR, imgR);
        end
        
        imuSampFrame = curKeySeqData.KeySeq.imuSampFrame{j,1};
        goldenVal = curKeySeqData.KeySeq.goldenVal{j,1};
        depthGT = double(curKeySeqData.KeySeq.depthGT{j,1});
        depthGTR = double(curKeySeqData.KeySeq.depthGTR{j,1});
        depthGT = depthScale*depthGT;
        frameID = [frameID; imgHeader(1)];
        
        if length(frameID) == 1
            imgLOrig = imgL;
            imgROrig = imgR;
            depthGTOrig = depthGT;
            depthGTROrig = depthGTR;
        end
        
        BsSampFrame = [BsSampFrame;[bsSampFrame goldenVal]];
        wheelData = [bsSampFrame(end,2) goldenVal(end,4:end)];
        RwheelCur = (roty(0)*rotx(0)*quatern2rotMat(wheelData([5 2 3 4])));
        %     RwheelCur = [-RwheelCur(2,:); RwheelCur(3,:); -RwheelCur(1,:)];
        
        
        
        
        RwheelDelt = RwheelCur'; %(RwheelCur'*RwheelPrv)';%
        %     eulerWheel = rad2deg(rotMat2euler(RwheelDelt));
        
        eulerWheel = rad2deg(rodrigues(RwheelDelt)');
        %     eulerWheel = sign(eulerWheel(2))*eulerWheel;
        %     eulerWheel = ([-eulerWheel(:,2) -eulerWheel(:,3) eulerWheel(:,1)]);
        eulerWheel = ([-eulerWheel(:,2) -eulerWheel(:,3) eulerWheel(:,1)]);
        rWheelTmp = rodrigues(deg2rad(eulerWheel));
        PoseWheel = [PoseWheel; [rWheelTmp(:)' -goldenVal(end,2) -goldenVal(end,3) goldenVal(end,1)]];
        EulerWheel = [EulerWheel;[wheelData(1) eulerWheel sign(eulerWheel(2))*norm(rad2deg(rodrigues(RwheelDelt)'))]];
        
        [PoseWheel_, EulerWheel_] = CvtBsFrame2Pose(bsSampFrame,goldenVal);
        
        PoseWheel_All = [PoseWheel_All;PoseWheel_];
        EulerWheel_All = [EulerWheel_All; EulerWheel_];
        gyroData = imuSampFrame(end,2:6);
        RgyroCur = (roty(0)*rotx(0)*quatern2rotMat(gyroData([5 2 3 4])));
        %     RgyroCur = [-RgyroCur(2,:); RgyroCur(3,:); -RgyroCur(1,:)];
        
        
        rgyroDelt_ = rodrigues(RgyroCur'*RgyroPrv);
        RgyroDelt_ = rad2deg(norm(rodrigues(RgyroCur'*RgyroPrv)));
        
        if rgyroDelt_(3)<0
            GyroAngAccum = [GyroAngAccum; -RgyroDelt_];
        else
            GyroAngAccum = [GyroAngAccum; RgyroDelt_];
        end
        RgyroDelt = RgyroCur'; %(RgyroCur'*RgyroPrv)'; %RgyroCur; %
        %     eulerGyro = rad2deg(rotMat2euler(RgyroDelt));
        
        eulerGyro = rad2deg(rodrigues(RgyroDelt)');
        %     eulerGyro = sign(eulerGyro(2))*eulerGyro;
        %     eulerGyro = ([-eulerGyro(:,2) -eulerGyro(:,3) eulerGyro(:,1)]);
        eulerGyro = ([-eulerGyro(:,2) -eulerGyro(:,3) eulerGyro(:,1)]);
        if exist('rGyroTmp','var')
            rGyroTmpPrv = rGyroTmp;
        end
        rGyroTmp = rodrigues(deg2rad(eulerGyro));
        PoseGyro = [PoseGyro; [rGyroTmp(:)'  -goldenVal(end,2) -goldenVal(end,3) goldenVal(end,1)]];
        EulerGyro = [EulerGyro;[gyroData(1) eulerGyro sign(eulerGyro(2))*norm(rad2deg(rodrigues(RgyroDelt)'))]];
        
        
        gyroTimeAng = [EulerGyro(:,1) cumsum(GyroAngAccum([1 [3:end]]))];
        
        
        prvImgTimeStamp = curImgTimeStamp;
        curImgTimeStamp = gyroData(1); %bsSampFrame(end,2);  %imgHeader(2);
        if isempty(vsl.frmStampList)
            prvFrmTimeStamp = curFrmTimeStamp;
        else
            prvFrmTimeStamp = vsl.frmStampList(end);
        end
        curFrmTimeStamp = imgHeader(2); %bsSampFrame(end,2);  %imgHeader(2);
        
        if saveTrace
            if vsl.keyFrameDone
                
                ptTrace = vsl.featPtManager.localTrace;
%                 theta = vsl.poseWcsList(size(vsl.poseWcsList,1) - size(ptTrace.ptIcsX,2) + 1:end,3);
                theta = vsl.poseWcsList(size(bsl.poseWcsList,1) - size(ptTrace.ptIcsX,2) + 1:end,3);
                visionTimeAngTemp = visionTimeAng(size(visionTimeAng,1) - size(ptTrace.ptIcsX,2) + 1:end,:);
                wheelTimeAngTemp = wheelTimeAng(size(wheelTimeAng,1) - size(ptTrace.ptIcsX,2) + 1:end,:);
                gyroTimeAngTemp = gyroTimeAng(size(gyroTimeAng,1) - size(ptTrace.ptIcsX,2) + 1:end,:);
                %             imgBuf = imgBuf(length(imgBuf) - length(theta) + 1 : end);
                save(fullfile(replayDir,sprintf('TraceData_%05d.mat',length(dir(fullfile(replayDir,'TraceData_*.mat')))+1)), 'b2c','theta','ptTrace','intrMat','imgBuf','visionTimeAngTemp','wheelTimeAngTemp','gyroTimeAngTemp','depthGTBuf');
                imgBuf = imgBuf(end,:);
                depthGTBuf = depthGTBuf(end,:);
                
                aawbs = 1;
            end
        end
        imgBuf = [imgBuf; [imgL imgR]];
        depthGTBuf = [depthGTBuf; depthGT];
        if 0
             
            idWheel = find(BsSampFrame(:,2) >= prvFrmTimeStamp & BsSampFrame(:,2) < curFrmTimeStamp);
            idWheel1 = find(BsSampFrame(:,2) > prvFrmTimeStamp,1,'first');
            if isempty(idWheel1)
                idWheel1 = 0;
            end
            idWheel2 = find(BsSampFrame(:,2) > curFrmTimeStamp,1,'first');
            if isempty(idWheel2)
                idWheel2 = size(BsSampFrame,1);
            end
            %         Localize(bsl, bsSampFrame, imgHeader(1));
            if idWheel1 + 1 > idWheel2
                idWheel1 = idWheel2 - 1;
            end
            
            wheelId = [wheelId;[idWheel1+1 idWheel2]];
            if size(wheelId,1) == 1
                Localize(bsl, BsSampFrame([idWheel1+1 : idWheel2],1:6), imgHeader(1));
            elseif 1
                Localize(bsl, BsSampFrame([wheelId(end-1,2)+1 : idWheel2],1:6), imgHeader(1));
            else
                Localize(bsl, BsSampFrame(idWheel,1:6), imgHeader(1));
            end
            
            if useGyro
                try
                    if 1
                        if ~isnan(curKeySeqData.gyroNew(size(bsl.poseWcsList,1)))
                            bsl.poseWcsList(1:end,3) = deg2rad(curKeySeqData.gyroNew(1:size(bsl.poseWcsList,1)));
                        end
                    else
                        gyroNew = interp1(gyroTimeAng(:,1),gyroTimeAng(:,2),visionTimeAng(:,1));
                        if ~isnan(curKeySeqData.gyroNew(size(bsl.poseWcsList,1)))
                            bsl.poseWcsList(1:end,3) = deg2rad(curKeySeqData.gyroNew(1:size(bsl.poseWcsList,1)));
                        end
                        
                    end
                    
                catch
                    asgnjk = 1;
                end
            end
            
            try
                if size(BsSampFrame,1) > idWheel(end)
                    
                    wheelTimeAng = [wheelTimeAng; [BsSampFrame(idWheel(end) + 1,2) rad2deg(bsl.poseWcsList(end,3))]];
                    
                else
                    wheelTimeAng = [wheelTimeAng; [BsSampFrame(end,2) rad2deg(bsl.poseWcsList(end,3))]];
                end
            catch
                wheelTimeAng = [wheelTimeAng; [BsSampFrame(end,2) rad2deg(bsl.poseWcsList(end,3))]];
            end
        else
            Localize(bsl, bsSampFrame(:,1:6), imgHeader(1));
            %         bsl.poseWcsList(i,3) = deg2rad(curKeySeqData.gyroNew(j));
            try
                if ~isnan(curKeySeqData.gyroNew(size(bsl.poseWcsList,1)))
                    bsl.poseWcsList(1:end,3) = deg2rad(curKeySeqData.gyroNew(1:size(bsl.poseWcsList,1)));
                end
            catch
                asgnjk = 1;
            end
        end
        
        
        vsl.goldenPose = [vsl.goldenPose; [EulerWheel_All(:,1) PoseWheel_All]];
        vsl.frmStamp = [prvFrmTimeStamp curFrmTimeStamp];
        vsl.frmStampList = [vsl.frmStampList; curFrmTimeStamp];
        
        
        vsl.depthGT = imresize(depthGT, 2.^-vsl.scaleLvl);
        
        
        Localize(vsl, imgL, imgR, GetMotionState(bsl, 'last'), GetPoseWcs(bsl, 'last'), imgHeader(1));
         
        visionTimeAng = [visionTimeAng; [curFrmTimeStamp rad2deg(vsl.poseWcsList(end,3))]];
        visionTimeAngRef = [visionTimeAngRef; [curFrmTimeStamp rad2deg(bsl.poseWcsList(end,3))]];
        if length(vsl.keyFrameFlagList) > 1
            if vsl.keyFrameFlagList(end-1)
                idd = length(vsl.angOpt(1:end-1));
            end
            try
                 vsl.angOpt(end) = vsl.angOpt(idd) + vsl.angOpt(end);
            catch
                asgdhjk = 1;
            end
        end
        try
            if 0
                gyroNew = interp1(gyroTimeAng(:,1),gyroTimeAng(:,2),visionTimeAng(:,1));
                bsl.poseWcsList(:,3) = deg2rad(gyroNew);
            end
        catch
            ahbdb = 1;
        end
        
%         idPulse = find(abs(rad2deg(vsl.poseWcsList(:,3))) > 0.001);
        n = floor(abs(rad2deg(vsl.poseWcsList(end,3)))/360);
        pulseVec = [0 : 360 : (n)*360]';
        [DD] = pdist2([pulseVec],abs(rad2deg(vsl.poseWcsList(:,3))),'euclidean');
        [~,idPulse] = min(DD');
%         idPulse = find(abs(rad2deg(vsl.poseWcsList(:,3))));
%         pulse = repmat(min(rad2deg(vsl.poseWcsList(:,3))-rad2deg(bsl.poseWcsList(:,3))),size(vsl.poseWcsList,1),1);
%         pulse(idPulse) = max(rad2deg(vsl.poseWcsList(:,3))-rad2deg(bsl.poseWcsList(:,3)));
        
        % abs((rad2deg(vsl.poseWcsList(end,3) - bsl.poseWcsList(end,3))) - (rad2deg(vsl.poseWcsList(end-1,3) - bsl.poseWcsList(end-1,3)))) > 0.1
        angErr = rad2deg(vsl.poseWcsList(:,3) - bsl.poseWcsList(:,3));
        angErr2 = [0;rad2deg(vsl.angOpt - bsl.poseWcsList(size(bsl.poseWcsList,1) - size(vsl.angOpt,1) + 1:end,3))];
        if 0
            noise = 0.1*((rand(length(angErr),1))-0.5)./5;
            noise = round(noise./0.01).*0.01;
        else
            noise = 0;
        end
        pulse = repmat(min(angErr),size(vsl.poseWcsList,1),1);
        pulse(idPulse) = max(angErr);
%         figure(12),clf;plot(angErr + noise);title('visual-body');hold on;plot(find(vsl.keyFrameFlagList),rad2deg(vsl.poseWcsList(find(vsl.keyFrameFlagList),3))-rad2deg(bsl.poseWcsList(find(vsl.keyFrameFlagList),3)),'or');plot(pulse,'k');%plot(scaleNum.*rotAxisGyro([3],1:end)','g'); %plot(deg2rad(Euler(:,3)));
        figure(12),clf;plot(angErr + noise);title('visual-body');hold on;plot(find(vsl.keyFrameFlagList),angErr(find(vsl.keyFrameFlagList)),'or');plot(pulse,'k');%plot(scaleNum.*rotAxisGyro([3],1:end)','g'); %plot(deg2rad(Euler(:,3)));
        figure(10),clf;plot(angErr);hold on;plot(angErr2);
        try
            figure(22),clf;plot(visionTimeAng(:,1),visionTimeAng(:,2) - visionTimeAng(1,2),'-x');hold on;plot(wheelTimeAng(:,1),wheelTimeAng(:,2) - wheelTimeAng(1,2),'-x');plot(gyroTimeAng(:,1),gyroTimeAng(:,2) - gyroTimeAng(1,2),'-x');plot(visionTimeAngRef(:,1),visionTimeAngRef(:,2) - visionTimeAngRef(1,2),'-x');legend('visual','wheel','gyro','ref');
        catch
            svkj = 1;
        end
        
        try
            [sum(vsl.featPtManager.localTrace.xGT(:,end) > 0) sum(vsl.featPtManager.localTrace.ptIcsX(:,end) > 0)]
            %             temp1 = vsl.featPtManager;
            %             temp2 = temp1;
            %             temp2.localTrace.ptIcsX = temp1.localTrace.xGT;
            %             temp2.localTrace.ptIcsY = temp1.localTrace.yGT;
            figure(1133), UpdateLocalTracePlot1(vsl.featPtManager, vsl.currImgL);
            UpdateLocalTracePlot2(vsl.featPtManager, vsl.currImgL);
            
        catch
            sgdnk = 1;
        end
        
        try
            diffBody = diff(rad2deg(bsl.poseWcsList(:,3)));
            diffVisual = diff(rad2deg(vsl.poseWcsList(:,3)));
            diffPixTrace = diff(vsl.MeanErr(:,1));
            diffPixTraceGT = diff(vsl.MeanErrGT(:,1));
            ofst = length(diffVisual) - length(diffPixTrace) + 1;
            dltAng = [(diffVisual( ofst: end) - diffBody(ofst: end))];
            
            DltAng = [DltAng; dltAng(end)];
            DiffPixTrace = [DiffPixTrace; diffPixTrace(end)];
            DiffPixTraceGT = [DiffPixTraceGT; diffPixTraceGT(end)];
            if length(diffPixTrace) == 1
                try
                    splitData = [splitData; [length(DltAng)-1 DltAng(end-1) length(DiffPixTrace)-1 DiffPixTrace(end-1) length(DiffPixTraceGT)-1 DiffPixTraceGT(end-1)]];
                catch
                    kbja = 1;
                end
                    
            end
            
            %             figure(7),clf;subplot(3,1,1),plot(DltAng);title('p2cAng(visual) - p2cAng(body)');
            %             subplot(3,1,2);plot([DiffPixTrace]);title('cur(vslX - trackingX) - prv(vslX - trackingX)')
            %             subplot(3,1,3);plot([DiffPixTraceGT]);title('cur(vslX - gtX) - prv(vslX - gtX)')
            if 0 %length(diffPixTrace) == 1
                try
                    figure(7),clf;subplot(3,1,1),plot(DltAng);title('p2cAng(visual) - p2cAng(body)');hold on;plot(splitData(:,1),splitData(:,2),'or');
                    subplot(3,1,2);plot([DiffPixTrace]);title('cur(vslX - trackingX) - prv(vslX - trackingX)');hold on;plot(splitData(:,3),splitData(:,4),'or');
                    subplot(3,1,3);plot([DiffPixTraceGT]);title('cur(vslX - gtX) - prv(vslX - gtX)');hold on;plot(splitData(:,5),splitData(:,6),'or');
                catch
                    figure(7),clf;subplot(3,1,1),plot(DltAng);title('p2cAng(visual) - p2cAng(body)');
                    subplot(3,1,2);plot([DiffPixTrace]);title('cur(vslX - trackingX) - prv(vslX - trackingX)')
                    subplot(3,1,3);plot([DiffPixTraceGT]);title('cur(vslX - gtX) - prv(vslX - gtX)')
                end
            else
                figure(7),clf;subplot(3,1,1),plot(dltAng);title('p2cAng(visual) - p2cAng(body)');
                subplot(3,1,2);plot([diffPixTrace]);title('cur(vslX - trackingX) - prv(vslX - trackingX)')
                subplot(3,1,3);plot([diffPixTraceGT]);title('cur(vslX - gtX) - prv(vslX - gtX)');
            end
            
            %             figure(7),clf;subplot(3,1,1),plot([(diffVisual(length(diffVisual) - length(diffPixTrace) + 1 : end) - diffBody(length(diffVisual) - length(diffPixTrace) + 1 : end))]);title('p2c(visual) - p2c(body)');
%             subplot(3,1,2);plot([diffPixTrace]);title('cur(vslX - trackingX) - prv(vslX - trackingX)')
%             subplot(3,1,3);plot([diffPixTrace]);title('cur(vslX - gtX) - prv(vslX - gtX)')
             
        catch
            savj = 1;
        end
        try
            figure(21);clf;subplot(1,3,1);plot(vsl.MeanErrOld(:,1), vsl.MeanErrOld(:,2),'-o');title('gt - tracking'); axis equal
            hold on;
            for k = 1 : size(vsl.MeanErrOld,1)
                text(double(vsl.MeanErrOld(k,1)),double(vsl.MeanErrOld(k,2)),num2str(k), 'Color',[1 0 0],'FontSize',15);  %,'FontWeight','bold');
            end
            subplot(1,3,2);plot(vsl.MeanErrGT(:,1), vsl.MeanErrGT(:,2),'-o');title('reproj - gt'); axis equal
            hold on;
            for k = 1 : size(vsl.MeanErrGT,1)
                text(double(vsl.MeanErrGT(k,1)),double(vsl.MeanErrGT(k,2)),num2str(k), 'Color',[1 0 0],'FontSize',15);  %,'FontWeight','bold');
            end
            subplot(1,3,3);plot(vsl.MeanErr(:,1), vsl.MeanErr(:,2),'-o');title('reproj - tracking'); axis equal
            hold on;
            for k = 1 : size(vsl.MeanErr,1)
                text(double(vsl.MeanErr(k,1)),double(vsl.MeanErr(k,2)),num2str(k), 'Color',[1 0 0],'FontSize',15);  %,'FontWeight','bold');
            end
            sabf = 1;
        catch
            asbjk = 1;
        end
        if checkGT
            ang1 = PureVision(imgLOrig, imgL, imgR, intrMat0, depthGTOrig, depthGT);
            ang2 = PureVision(imgL, imgLOrig, imgROrig, intrMat0, depthGT, depthGTOrig);
        end
    end
    try
        err = vsl.poseWcsList(:,3) - curKeySeqData.vsl.poseWcsList(:,3);
    catch
        sankbj = 1;
    end
    %     rad2deg(vsl.poseWcsList(:,3)) - curKeySeqData.wheelNew
    drawnow;
end



saknj  =1;
if 0
    global OUTPUT_ROOT RECORDED_BIN_ROOT DUMP_ROOT RESULT_ROOT
    
    assert(~isempty(OUTPUT_ROOT) && ~isempty(RECORDED_BIN_ROOT) && ~isempty(DUMP_ROOT), 'The directories are not correctly set');
    
    fclose all;
    close all;
    
    csaParamFilePath = fullfile(DUMP_ROOT, 'coordsys_alignment.txt');
    csa = CoordSysAligner;
    Load(csa, csaParamFilePath);
    
    robotCfgFilePath = fullfile(OUTPUT_ROOT, 'robot_config.prototxt');
    robotConfig = ReadRobotConfig(robotCfgFilePath);
    
    camParamFilePath = fullfile(DUMP_ROOT, 'camera_param.txt');
    cam = BCStereoCameraModel(camParamFilePath);
    CvtToRectifiedModel(cam);
    cibfr = CamImuBinFileReader;
    SetInputDevice(cam, cibfr);
    camImuBinFilePath = fullfile(RECORDED_BIN_ROOT, 'camera_imu.bin');
    OpenInput(cam, camImuBinFilePath);
    
    bsbfr = BodySensorBinFileReader;
    bodySensorBinFilePath = fullfile(RECORDED_BIN_ROOT, 'bodysensor.bin');
    bodySensorRespFilePath = fullfile(RECORDED_BIN_ROOT, 'bodysensor_response.bin');
    Open(bsbfr, bodySensorBinFilePath, bodySensorRespFilePath);
    
    bsl = BodySensorLocalizer(GetBodySensorDataDecoder(bsbfr), robotConfig.bodysensor_localizer_param);
    vsl = VisualLocalizer(cam, csa, robotConfig.tracking_config);
    % AlignBodySensorToCamera(cibfr);
    %
    % [imuData, bodySensorData] = GetImuBodySensorSamples(cibfr);
    % [bodySensorMotionState, imuMotionState] = GetImuBodySensorMotionState('LK', bodySensorData, imuData, frameTimeStamps, bodySensorLocalizerParam, imuLocalizerParam);
    lp = LocalPlanner(robotConfig);
    
    visualizeResult = VisualizeResult();
    SAVE_VID = false;
    goldenPath = fullfile(RESULT_ROOT, 'alignedData.txt');
    LOAD_GOLDEN_DATA =  exist(goldenPath);
    LoadGolden(visualizeResult, goldenPath, LOAD_GOLDEN_DATA);
    
    videoFilePath = fullfile(RESULT_ROOT, 'frame.avi');
    LoadResultVidFile(visualizeResult, videoFilePath, SAVE_VID)
    
    db = debug();
    lp = LocalPlanner(robotConfig);
    
    % figNum = 990000;
    motionState = [];
    
    
    
    
    
    
    
    
    
    while HasFrame(cam)
        [imgL, imgR, frmTimestamp, frmInd] = GetNextStereoImagePair(cam, true);
        
        %     imgL = RGB2BGR(imgL);
        %     imgR = RGB2BGR(imgR);
        
        bsSampFrame = GetNextDataFrame(bsbfr, frmTimestamp, 0);
        if isempty(bsSampFrame)
            continue;
            %         break;
        end
        
        %     bsSampFrame = debug.modifySampData(bsSampFrame,frmInd,0,2);
        Localize(bsl, bsSampFrame, frmInd);
        Localize(vsl, imgL, imgR, GetMotionState(bsl, 'last'), GetPoseWcs(bsl, 'last'), frmInd);
        
        %     figure(771177),plot(rad2deg(vsl.poseWcsList(:,3)-bsl.poseWcsList(:,3)));
        
        CompareGoldenTraceAndVisionTrace(visualizeResult,vsl,bsl,77, frmInd, frmTimestamp);
        CompareGoldenAngAndVisionAng(visualizeResult,vsl,bsl,133, frmInd, frmTimestamp);
        [v,w] = PlanLocalRoute(lp, vsl, bsl);
        debug_show_local_planner(db,visualizeResult,vsl, lp, 123)
        motionState = [motionState;GetMotionState(bsl, 'last')];
        debug.debug_915_compare_length_between_vsl_and_bsl(vsl,bsl,motionState, true)
        %     id = find(vsl.keyFrameFlagList == 1);
        
        %     if ~isempty(id)
        %         dataRange =  id(end):size(vsl.keyFrameFlagList,1);
        %         if exist('idPrv','var')
        %             if id(end) ~= idPrv
        %                 figNum = figNum + 1;
        %             end
        %         end
        %         figure(figNum),clf;plot(rad2deg(vsl.poseWcsList(dataRange,3)-vsl.poseWcsList(dataRange(1),3)),'b');hold on; plot(rad2deg(bsl.poseWcsList(dataRange,3)-bsl.poseWcsList(dataRange(1),3)),'r');legend('visual','wheel');
        %         idPrv = id(end);
        %     end
        
        %     PlotMap2D(visualizeResult,vsl,bsl,122, frmInd, frmTimestamp)
        drawnow;
        SaveFrame(visualizeResult, 1122);
        %     SaveRawFrame(visualizeResult, imgL,155);
        %     SaveStereoFrame(visualizeResult, imgL,imgR, 155);
        
    end
    
    bslPose = bsl.poseWcsList;
    vslPose = vsl.poseWcsList;
    vslPose(:,1) = -vslPose(:,1);
    keyframeList = vsl.keyFrameFlagList;
    pt3D = vsl.pointCloudManager.pt3D;
    resultFilePath = fullfile(RESULT_ROOT, 'result');
    save(resultFilePath,'bslPose','vslPose','keyframeList','pt3D');
    
    CloseInput(cam);
    Close(bsbfr);
    CloseVid(visualizeResult);
end
end