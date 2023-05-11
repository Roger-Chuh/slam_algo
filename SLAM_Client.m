function SLAM_Client(folderName)

replayDir = fullfile(pwd, folderName);
delete('D:\SLAM\slam_algorithm\temp\*.png')


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

bsbfr = BodySensorBinFileReader;

bsl = BodySensorLocalizer(GetBodySensorDataDecoder(bsbfr), robotConfig.bodysensor_localizer_param);
vsl = VisualLocalizer(cam, csa, robotConfig.tracking_config);
visualizeResult = VisualizeResult();

SAVE_VID = true;
% videoFilePath = fullfile(RESULT_ROOT, 'frame.avi');
videoFilePath = './Result/small_baseline.avi';
% videoFilePath2 =  './Result/frame.avi';

LoadResultVidFile(visualizeResult, videoFilePath, SAVE_VID)

rosshutdown
% serverIP = 'http://192.168.50.26:11311';
serverIP = 'http://192.168.20.84:11311';

% clientIP = '192.168.1.113';
clientIP = '192.168.20.86';

% clientIP = '192.168.1.114';

communication = SeverAndClientCommunication(serverIP, clientIP);

lp = LocalPlanner_zigzag(robotConfig);
lp_A2B = LocalPlanner_A2B(robotConfig);


% lp = LocalPlanner2(robotConfig, clockwise);

db = debug();


imuSampRate = 50;

nCounter = 0;
counterMax = inf;
seqNum = [];
goldenData = [];
bslTimeStamp = [];
vslTimeStamp = [];
vel = [];
omega = [];
figNum = 990000;
motionState = [];

zigzagDone = false;
A2BDone = false;
pt = [];
zigzagMainDone = false;
isSetA2BAll = false;
PathPlanningDone = false;
gpDone = false;

bodyWidth = 0.4;
vec = [0 bodyWidth/2];
ptbody = [];
for j = 0 : 90 : 360-90
    ptbody = [ptbody [sind(j).*vec;cosd(j).*vec]];
    
    
end
ptbody = intersect(ptbody',ptbody','rows');
ptbody = [ptbody  zeros(size(ptbody,1),1)];

isContinueZigzag = true;
isContinueA2B = false;
doneFlag = false;
ovlpThr = 200;
storePath = {};
ImuData = [];
imuData = [];
curImgTimeStamp = 0;
curFrmTimeStamp = 0;

wheelGolden = [];
imuGolden = [];
RwheelCur = eye(3);
RgyroCur = eye(3);
EulerWheel = []; PoseWheel = [];
EulerGyro = []; PoseGyro = [];
BsSampFrame = [];PoseImu = [];
startId = 2;
Euler = zeros(startId-1,5);  [0 0 0 0 0;0 0 0 0 0];
Ang = zeros(startId-1,1); [0;0];
I2G = [];
GyroAngAccum = 0;

AngB = []; AngV = [];

PoseWheel_All = [];
EulerWheel_All = [];

scaleNum = 100;

saveFrame = 1;
% vsl.switchDepth = false;
wheelId = [];
gyroId = [];
wheelTimeAng = [];
visionTimeAng = [];
gyroTimeAng = [];

KeySeq.bsSampFrame = {};
KeySeq.imgHeader = {};
KeySeq.imgL = {};
KeySeq.imgR = {};
KeySeq.imuSampFrame = {};
KeySeq.goldenVal = {};
KeySeq.depthGT = {};
while(nCounter < counterMax) && ~doneFlag
    ResumeSystem(communication)
    RwheelPrv = RwheelCur;
    RgyroPrv = RgyroCur;
    [bsSampFrame ,imgHeader, imgL, imgR, imuSampFrame, goldenVal, depthGT] = SubscribeData(communication);
    KeySeq.bsSampFrame = [KeySeq.bsSampFrame; bsSampFrame];
    KeySeq.imgHeader = [KeySeq.imgHeader; imgHeader];
    KeySeq.imgL = [KeySeq.imgL; imgL];
    KeySeq.imgR = [KeySeq.imgR; imgR];
    KeySeq.imuSampFrame = [KeySeq.imuSampFrame; imuSampFrame];
    KeySeq.goldenVal = [KeySeq.goldenVal; goldenVal];
    KeySeq.depthGT = [KeySeq.depthGT; depthGT];
    if vsl.keyFrameDone
        save(fullfile(replayDir,sprintf('KeySeq_%05d.mat',length(dir(fullfile(replayDir,'KeySeq_*.mat')))+1)), 'KeySeq','wheelNew','gyroNew','bsl','vsl');
        KeySeq.bsSampFrame = {};
        KeySeq.imgHeader = {};
        KeySeq.imgL = {};
        KeySeq.imgR = {};
        KeySeq.imuSampFrame = {};
        KeySeq.goldenVal = {};
        KeySeq.depthGT = {};
        
%         vsl.keyFrameDone = false;
    end
    %     imuSampFrame = [imuSampFrame(1:end-1,1:6) imuSampFrame(2:end,7:end)];
    
    % %     bsSampFrame(:,end) = bsSampFrame(:,end) + deg2rad(0.5*rand(size(bsSampFrame,1),1));
    if isempty(bsSampFrame) || isempty(imuSampFrame)
        continue;
    end
    
    if nCounter >= 1
        imuSampFrame_tmp = [ImuData(end,:);imuSampFrame(1,:)];
        imuSampFrame_tmp_ = checkImuSample(imuSampFrame_tmp,imuSampRate);
        ImuData(end,:) = imuSampFrame_tmp_(1,:);
        %         imuData(end,2:4) = [imuSampFrame_tmp_(1,7) -imuSampFrame_tmp_(1,9) -imuSampFrame_tmp_(1,8)];
        imuData(end,2:4) = [-imuSampFrame_tmp_(1,8) -imuSampFrame_tmp_(1,9) imuSampFrame_tmp_(1,7)];
        imuSampFrame = checkImuSample(imuSampFrame,imuSampRate);
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
    wheelGolden = [wheelGolden; ];
    disp(imgHeader(1))
    seqNum(end+1) = imgHeader(1);
    ImuData = [ImuData; imuSampFrame];
    imuData_ = imuSampFrame(:,[2 7:end]);
    %     %     imuData__ = [imuData_(:,1) imuData_(:,3) -imuData_(:,4) -imuData_(:,2) imuData_(:,6) -imuData_(:,7) -imuData_(:,5)];
    %         imuData__ = [imuData_(:,1) -imuData_(:,3) -imuData_(:,4) imuData_(:,2) -imuData_(:,6) -imuData_(:,7) imuData_(:,5)];
    imuData__ = [imuData_(:,1) -imuData_(:,3) -imuData_(:,4) imuData_(:,2) -imuData_(:,6) -imuData_(:,7) imuData_(:,5)];
    %         imuData__ = [imuData_(:,1) imuData_(:,2) -imuData_(:,4) -imuData_(:,3) imuData_(:,5) -imuData_(:,7) -imuData_(:,6)];
    %     imuData__ = [imuData_(:,1) -imuData_(:,4) imuData_(:,3) imuData_(:,2) -imuData_(:,7) imuData_(:,6) imuData_(:,5)];
    %     imuData__ = imuData_;
    imuData = [imuData;imuData__];
    if nCounter == startId-1
        %         meanG = imuData__(end,5:7);
        meanG = [ -0.004482234704458  -9.800009137812605  -0.007887722192715];
        
        startPoseId = find(EulerGyro(:,1) == curImgTimeStamp);
        imul = ImuLocalizer(meanG,1/imuSampRate,reshape(PoseGyro(startPoseId,1:9),3,3));
        %         imul = ImuLocalizer(meanG,1/imuSampRate,eye(3));
    end
    prvImgTimeStamp = curImgTimeStamp;
    curImgTimeStamp = gyroData(1); %bsSampFrame(end,2);  %imgHeader(2);
    prvFrmTimeStamp = curFrmTimeStamp;
    curFrmTimeStamp = imgHeader(2); %bsSampFrame(end,2);  %imgHeader(2);
    
    
    if exist('imul','var')
        if 1
            RPrv = imul.RotIntg;
            [UU,SS,VV] = svd(RPrv);
            RPrv = UU*VV';
            PreintegrateOneFrame(imul, imuData,prvImgTimeStamp,curImgTimeStamp);
            RCur = imul.RotIntg;
            [UU,SS,VV] = svd(RCur);
            RCur = UU*VV';
            PoseImu = [PoseImu; [RCur(:)' imul.transVecI2W(end,:)]];
            
            deltR = reshape(imul.rotMatI2W(end,:),3,3);%RCur'*RPrv;% reshape(imul.rotMatI2W(end,:),3,3);%
            deltRVec = rodrigues(RCur'*RPrv);
            %             euler = rotMat2euler(deltR);
            % %             [rodrigues(rGyroTmp) rodrigues(deltR)]
            err = norm([rodrigues(rGyroTmp)-rodrigues(deltR)]);
            
            euler = rodrigues(deltR)';
            %             euler = sign(euler(2))*euler;
            Euler = [Euler;[gyroData(1) rad2deg(euler) sign(euler(2))*rad2deg(norm(deltRVec))]];
            
            data = debug_integration(ImuData, imuData, imul.id(1),imul.id(end)+1);
            dltt = rodrigues(rGyroTmpPrv'*rGyroTmp);
            
            [(data - [dltt dltt]) (rodrigues(rGyroTmp) - rodrigues(deltR))]
            
            (abs((Euler(end,3)) - (EulerGyro(end,3))))
            %             ang = rad2deg(norm(rodrigues(deltR)));
            %             ang = rad2deg(norm(deltRVec(3)));
            %             ang = rad2deg(norm(deltRVec(1:3)));
            ang = rad2deg(norm(euler(2)));
            if euler(2) < 0
                %             if deltRVec(2) < 0
                %             if deltRVec(2) < 0
                %             if deltRVec(3) > 0
                %             if w <0
                %             if imul.motionState == 3
                ang = -ang;
                Euler(end,end) = -Euler(end,end);
            end
        else
            PreintegrateOneFrame(imul, imuData__, imuData__(1,1), imuData__(end,1));
        end
        %         ang = rad2deg(norm(rodrigues(reshape(imul.rotMatI2W(end,:),3,3))));
        Ang = [Ang;ang];
    end
    if ~isempty(bsSampFrame) % && nCounter > 0
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
        if size(BsSampFrame,1) > idWheel(end)
            wheelTimeAng = [wheelTimeAng; [BsSampFrame(idWheel(end) + 1,2) rad2deg(bsl.poseWcsList(end,3))]];
        else
            wheelTimeAng = [wheelTimeAng; [BsSampFrame(end,2) rad2deg(bsl.poseWcsList(end,3))]];
        end
        
        AngB = [AngB; [bsSampFrame(end,2) bsl.poseWcsList(end,3)]];
        %         vsl.goldenPose = [EulerWheel(:,1) PoseWheel]; 
        vsl.goldenPose = [EulerWheel_All(:,1) PoseWheel_All];
        vsl.frmStamp = [prvFrmTimeStamp curFrmTimeStamp];
        vsl.frmStampList = [vsl.frmStampList; curFrmTimeStamp];
        
%         vsl.depthGT = imresize(depthGT, 2.^-vsl.scaleLvl);
        
        
        Localize(vsl, imgL, imgR, GetMotionState(bsl, 'last'), GetPoseWcs(bsl, 'last'), imgHeader(1));
        visionTimeAng = [visionTimeAng; [curFrmTimeStamp rad2deg(vsl.poseWcsList(end,3))]];
        
        AngV = [AngV; [vsl.frmStamp(2) vsl.poseWcsList(end,3) 1]];
        if 0
            try
                AngV(:,3) = interp1(AngV(:,1),AngV(:,2),AngB(:,1),'linear');
            catch
                avlaf = 1;
            end
        end
        
    end
    
    
    
    if nCounter >=startId
        
        
        %         I2G = [I2G rodrigues(RgyroDelt'*deltR)];
        I2G = [I2G rodrigues(rGyroTmp'*deltR)];
        %         figure(99),plot(I2G');
        
        %         figure(100),clf;plot([rad2deg(vsl.poseWcsList(:,end)) rad2deg(bsl.poseWcsList(:,end)) -(Ang)]);legend('visual','body','gyro');
        if 0
            figure(98),clf;plot((Euler(startId:end,2:4)) - (EulerGyro(startId:end,2:4)));title('imu - imuGT');
            figure(101),clf;plot(Euler(startId:end,1),(Euler(startId:end,2)));hold on;plot(EulerGyro(startId:end,1),(EulerGyro(startId:end,2)));plot(EulerWheel(startId:end,1),(EulerWheel(startId:end,2)));legend('x imu','x imuGT','x wheelGT');
            figure(102),clf;plot(Euler(startId:end,1),(Euler(startId:end,3)));hold on;plot(EulerGyro(startId:end,1),(EulerGyro(startId:end,3)));plot(EulerWheel(startId:end,1),(EulerWheel(startId:end,3)));legend('y imu','y imuGT','y wheelGT');
            figure(103),clf;plot(Euler(startId:end,1),(Euler(startId:end,4)));hold on;plot(EulerGyro(startId:end,1),(EulerGyro(startId:end,4)));plot(EulerWheel(startId:end,1),(EulerWheel(startId:end,4)));legend('z imu','z imuGT','z wheelGT');
        end
        %         figure(104),clf;plot(Euler(startId:end,1),(Euler(startId:end,5)));hold on;plot(EulerGyro(startId:end,1),(EulerGyro(startId:end,5)));plot(EulerWheel(startId:end,1),(EulerWheel(startId:end,5)));legend('z imu','z imuGT','z wheelGT');
        %         figure(105),clf;hold on;plot(EulerGyro(startId:end,1),(EulerGyro(startId:end,5)));plot(EulerWheel(startId:end,1),(EulerWheel(startId:end,5)));plot(Euler(startId:end,1),(Ang(startId:end)));legend('all imuGT','all wheelGT','all imu');
    end
    if nCounter ==  0
        position = vsl.poseWcsList(1,:);
        position(3) = position(3) + pi/2;
        clockwise = true;
        %         SetA2B(lp_A2B, position, [0.6,0.6], pi/4);
        SetLocalPlanner(lp, position, clockwise);
    end
    
    %
    if ~zigzagMainDone
        if isContinueZigzag
            [v, w , isContinueZigzag] = PlanZigzagRoout(lp, vsl, true);
            isContinueZigzag = true;
                        v = 0;
                        w = 0.5;
            if 0
                if nCounter > 2  && nCounter <50
                    if 0 % vsl.keyFrameFlagList(end-1) == 1
                        v = 0;
                        w = 0;
                        
                    else
                        v = 0;
                        w = -0.5;
                    end
                end
                
                if nCounter > 50
                    v = 0;
                    w = 0.5;
                    
                end
            end
            
            
% %                 if nCounter > 5
% %                     v = 0.1;
% %                     w = 0.4;
% %                     
% %                 end
            
            
            
            position = vsl.poseWcsList(end,:);
            position(3) = position(3)+pi/2;
            tmp = rotz(rad2deg(double(position(3))))*ptbody' + repmat([position(1:2) 0]',1,size(ptbody,1));
            zigzagMainDone = ~isContinueZigzag;
            pt = [pt tmp];
            if ~isContinueZigzag
                dfnkj = 12;
            end
            
            %                     figure(1),clf;
            %                     [position,trace,insightFlag] = drawPath(v,w,position,dt,trace,maxDepth,fovx,obstacle,insightFlag);
        end
    else
        if ~PathPlanningDone
            obstacle = vsl.pointCloudManager.map2D;
            
            position = vsl.poseWcsList(end,:);
            position(3) = position(3)+pi/2;
            %             [path, newXY] = GlobalPlanner(bodyWidth,[[min(obstacle(:,1))-0.01 obstacle(1,2)];obstacle], pt, [position(1:2) pi/2]);
            [path, newXY,visitedPath, bodyScale] = GlobalPlanner(bodyWidth,obstacle, pt, [position(1:2) pi/2]);
            storePath = [storePath; {path, newXY,visitedPath, bodyScale,bodyWidth,obstacle, pt, [position(1:2) pi/2]}];
            
            bodyVisitedLastGrid = VisitedGrid(pt, bodyWidth, bodyScale);
            bodyVisitedLast = pt;
            isStop = false;
            
            
            
            figure(1),clf;hold on;
            for k = 1: size(path,1)
                plot(path{k,1}(:,1),path{k,1}(:,2),'-o');
            end
            selectPath = 1;
            loopCnt = size(path{selectPath},1);
            A2BDone = false;
            PathPlanningDone = true;
            isSetA2B = false;
            %             isContinueA2B = true;
        end
    end
    if zigzagMainDone
        %             zigzagDone = true;
        if PathPlanningDone && ~isempty(path)
            for jj = 1 : 1 %loopCnt
                j = 1;
                if size(path{selectPath},1) > 1
                    if ~isContinueA2B && ~isSetA2B
                        position = vsl.poseWcsList(end,:);
                        position(3) = position(3)+pi/2;
                        SetA2B(lp_A2B, position, path{selectPath}(j,1:2),[]);
                        isSetA2B = true;
                        isContinueA2B = true;
                        
                    end
                    if isSetA2B && isContinueA2B
                        [v, w , isContinueA2B] =  RunP2P_2d(lp_A2B, vsl);
                        position = vsl.poseWcsList(end,:);
                        position(3) = position(3)+pi/2;
                        tmp = rotz(rad2deg(double(position(3))))*ptbody' + repmat([position(1:2) 0]',1,size(ptbody,1));
                        pt = [pt tmp];
                        %                                 figure(1),clf
                        
                        if ~isContinueA2B
                            isSetA2B = false;
                            path{selectPath}(1,:) = [];
                        end
                        %                                 [position,trace,insightFlag] = drawPath(v,w,position,dt,trace,maxDepth,fovx,obstacle,insightFlag);
                        %                                 for k = 1: size(path,1)
                        %                                     plot(path{k,1}(:,1),path{k,1}(:,2),'-o');
                        %                                 end
                    end
                else
                    if ~isSetA2BAll
                        if ~isContinueA2B && ~isSetA2B
                            position = vsl.poseWcsList(end,:);
                            position(3) = position(3)+pi/2;
                            SetA2B(lp_A2B, position, path{selectPath}(j,1:2),newXY{selectPath,6});
                            isSetA2B = true;
                            isContinueA2B = true;
                            
                        end
                        if isSetA2B && isContinueA2B
                            [v, w , isContinueA2B] =  RunP2P_2d(lp_A2B, vsl);
                            position = vsl.poseWcsList(end,:);
                            position(3) = position(3)+pi/2;
                            tmp = rotz(rad2deg(double(position(3))))*ptbody' + repmat([position(1:2) 0]',1,size(ptbody,1));
                            pt = [pt tmp];
                            %                                 figure(1),clf;
                            if ~isContinueA2B
                                isSetA2B = false;
                                isSetA2BAll = true;
                                isSetAZigzag = false;
                                %                             isContinueZigzag = true;
                            end
                            %                                 [position,trace,insightFlag] = drawPath(v,w,position,dt,trace,maxDepth,fovx,obstacle,insightFlag);
                            %                                 for k = 1: size(path,1)
                            %                                     plot(path{k,1}(:,1),path{k,1}(:,2),'-o');
                            %                                 end
                        end
                        
                    else
                        
                        if ~isContinueZigzag && ~isSetAZigzag
                            position = vsl.poseWcsList(end,:);
                            position(3) = position(3)+pi/2;
                            SetLocalPlanner(lp, position,newXY{selectPath,7});
                            isSetAZigzag = true;
                            isContinueZigzag = true;
                            
                        end
                        
                        
                        if isContinueZigzag
                            [v, w , isContinueZigzag] = PlanZigzagRoout(lp, vsl, true);
                            position = vsl.poseWcsList(end,:);
                            position(3) = position(3)+pi/2;
                            tmp = rotz(rad2deg(double(position(3))))*ptbody' + repmat([position(1:2) 0]',1,size(ptbody,1));
                            pt = [pt tmp];
                            bodyVisitedNew = (setdiff(pt',bodyVisitedLast','rows'))';
                            bodyVisitedGrid = VisitedGrid(bodyVisitedNew, bodyWidth, bodyScale);
                            ovlp = intersect(bodyVisitedGrid, bodyVisitedLastGrid,'rows');
                            if size(ovlp,1) > ovlpThr
                                %                                 isStop = true;
                                isContinueZigzag = false;
                            end
                            isStop = ~isContinueZigzag;
                            %                                 figure(1),clf;
                            
                            %                                 [position,trace,insightFlag] = drawPath(v,w,position,dt,trace,maxDepth,fovx,obstacle,insightFlag);
                            %                                 for k = 1: size(path,1)
                            %                                     plot(path{k,1}(:,1),path{k,1}(:,2),'-o');
                            %                                 end
                            %                             zigzagMainDone = ~isContinue;
                            
                            %                             if ~isContinueZigzag
                            %                                 doneFlag = true;
                            %                             end
                        end
                    end
                    gpDone = true;
                end
            end
            if isStop
                obstacle = vsl.pointCloudManager.map2D;
                position = vsl.poseWcsList(end,:);
                position(3) = position(3)+pi/2;
                [path, newXY, visitedPath,bodyScale] = GlobalPlanner(bodyWidth,obstacle, pt, [position(1:2) pi/2]);
                
                figure(1),clf;hold on;
                for k = 1: size(path,1)
                    plot(path{k,1}(:,1),path{k,1}(:,2),'-o');
                end
                
                storePath = [storePath; {path, newXY,visitedPath, bodyScale,bodyWidth,obstacle, pt, [position(1:2) pi/2]}];
                
                bodyVisitedLastGrid = VisitedGrid(pt, bodyWidth, bodyScale);
                bodyVisitedLast = pt;
                isStop = false;
                isSetA2BAll = false;
                
                isSetA2B = false;
                isContinueA2B = false;
                if ~isempty(path)
                    selectPath = 1;
                else
                    doneFlag = true;
                end
            end
            zigzagDone = true;
            %         else
            %              doneFlag = true;
        end
    end
    %%
    SetCmdVelocity(communication, v, w);
    %         SetCmdVelocity(communication, 0, 0.4);
    
    %     CompareGoldenTraceAndVisionTrace(visualizeResult,vsl,bsl,77, imgHeader(1), imgHeader(2));
    %     CompareGoldenAngAndVisionAng(visualizeResult,vsl,bsl,133, imgHeader(1), imgHeader(2));
    %     VisualizeLocalPlanner(visualizeResult,vsl,bsl,lp, 123);
    
    
    debug_show_local_planner2(db,visualizeResult,vsl, 123)
    debug.debug_914_compare_goldenData(vsl,bsl,goldenData,bslTimeStamp,vslTimeStamp, GetMotionState(bsl, 'last'), false)
    tmp = cumsum(GyroAngAccum(2:end));
    if 0
        figure(321),clf;subplot(4,1,1);plot([rad2deg([vsl.poseWcsList(:,3) bsl.poseWcsList(:,3)]) cumsum(GyroAngAccum(2:end))]);legend('visual','body','golden');
        subplot(4,1,2);plot(rad2deg(vsl.poseWcsList(:,3))-cumsum(GyroAngAccum(2:end)));title('visual-gyro');hold on;plot(find(vsl.keyFrameFlagList),rad2deg(vsl.poseWcsList(find(vsl.keyFrameFlagList),3))-(tmp(find(vsl.keyFrameFlagList))),'or');
        subplot(4,1,3);plot(rad2deg(bsl.poseWcsList(:,3))-cumsum(GyroAngAccum(2:end)));title('body-gyro');
        subplot(4,1,4);plot(rad2deg(vsl.poseWcsList(:,3))-rad2deg(bsl.poseWcsList(:,3)));title('visual-body');hold on;plot(find(vsl.keyFrameFlagList),rad2deg(vsl.poseWcsList(find(vsl.keyFrameFlagList),3))-rad2deg(bsl.poseWcsList(find(vsl.keyFrameFlagList),3)),'or');
    end
    %     figure(1122),saveas(gcf,sprintf('tmp/temp_%06d.png',size(vsl.poseWcsList,1)));
    
    
    if size(vsl.poseWcsList,1) >= 3
        prvLsat = vsl.frmStampList(end-2);
        curLsat = vsl.frmStampList(end-1);
        wheelNew = interp1(wheelTimeAng(:,1),wheelTimeAng(:,2),visionTimeAng(:,1));
        wheelNew(1) = 0;
        gyroNew = interp1(gyroTimeAng(:,1),gyroTimeAng(:,2),visionTimeAng(:,1));
        gyroNew(1) = 0;
    end
    
    
    
    Untitled5;
    try
    idPulse = find(diff(Euler(:,3)) > 180-10);
    idPulse = idPulse - idPulse(1) + 1;
    pulse = repmat(min(rad2deg(vsl.poseWcsList(:,3))-rad2deg(bsl.poseWcsList(:,3))),size(Euler,1),1);
    pulse(idPulse) = max(rad2deg(vsl.poseWcsList(:,3))-rad2deg(bsl.poseWcsList(:,3)));
    
    
        figure(11),clf; plot(rad2deg(vsl.poseWcsList(:,3))-gyroNew);title('visual-gyro');hold on;plot(find(vsl.keyFrameFlagList),rad2deg(vsl.poseWcsList(find(vsl.keyFrameFlagList),3))-(gyroNew(find(vsl.keyFrameFlagList))),'or');plot(pulse,'k');
    catch
        swkdvjb = 1;
    end
    try
        figure(12),clf; plot(rad2deg(vsl.poseWcsList(:,3))-wheelNew);title('visual-body');hold on;plot(find(vsl.keyFrameFlagList),rad2deg(vsl.poseWcsList(find(vsl.keyFrameFlagList),3))-(wheelNew(find(vsl.keyFrameFlagList),1)),'or');plot(pulse,'k');plot(scaleNum.*rotAxisGyro([3],1:end)','g'); %plot(deg2rad(Euler(:,3)));
    catch
        sdva  = 1;
    end
if saveFrame
    imwrite(imresize(imgL,1),sprintf('tmp/tempL_%06d.png',size(vsl.poseWcsList,1)));
    imwrite(imresize(imgR,1),sprintf('tmp/tempR_%06d.png',size(vsl.poseWcsList,1)));
end
    if 0
        try
            subplot(5,1,5);plot(AngB(:,1),rad2deg(AngV(:,3))-rad2deg(AngB(:,2)));title('visual-body');hold on;plot(AngB(find(vsl.keyFrameFlagList),1),rad2deg(AngV(find(vsl.keyFrameFlagList),3))-rad2deg(AngB(find(vsl.keyFrameFlagList),2)),'or');
        catch
            abk = 1;
        end
    end
    drawnow;
    %     debug.debug_915_compare_length_between_vsl_and_bsl(vsl,bsl,motionState, true)
    
    %     id = find(vsl.keyFrameFlagList == 1);
    %
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
    %     PlotMap2D(visualizeResult,vsl,bsl,122, imgHeader(1), imgHeader(2))
    figure(1122);saveas(gcf,sprintf('trace/trace_%06d.png',size(vsl.poseWcsList,1)));
    SaveFrame(visualizeResult, 123);
    nCounter = nCounter + 1;
end
CloseInput(cam);
Close(bsbfr);
CloseVid(visualizeResult);

end