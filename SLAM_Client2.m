function SLAM_Client()

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
serverIP = 'http://192.168.1.111:11311';

% clientIP = '192.168.1.113';
clientIP = '192.168.1.114';

communication = SeverAndClientCommunication(serverIP, clientIP);

lp = LocalPlanner_zigzag(robotConfig);
lp_A2B = LocalPlanner_A2B(robotConfig);


% lp = LocalPlanner2(robotConfig, clockwise);

db = debug();

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
while(nCounter < counterMax) && ~doneFlag
    ResumeSystem(communication)
    [bsSampFrame ,imgHeader, imgL, imgR, goldenVal] = SubscribeData(communication);
    disp(imgHeader(1))
    seqNum(end+1) = imgHeader(1);
    %     goldenData = [goldenData;goldenVal];
    %     bslTimeStamp = [bslTimeStamp; bsSampFrame];
    %     vslTimeStamp = [vslTimeStamp; imgHeader];
    %     motionState = [motionState;GetMotionState(bsl, 'last')];
    
    %     if length(seqNum) >=2
    %        assert(seqNum(end)-seqNum(end-1) == 1, 'seqNum is not continuous!')
    %     end
    if ~isempty(bsSampFrame)
        Localize(bsl, bsSampFrame, imgHeader(1));
        Localize(vsl, imgL, imgR, GetMotionState(bsl, 'last'), GetPoseWcs(bsl, 'last'), imgHeader(1));
    end
    if nCounter ==  0
        position = vsl.poseWcsList(1,:);
        position(3) = position(3) + pi/2;
        clockwise = false;
        %         SetA2B(lp_A2B, position, [0.6,0.6], pi/4);
        SetLocalPlanner(lp, position, clockwise);
    end
    %     [v,w] = PlanLocalRoute2(lp, vsl);
    %     if plannerState == 1
    %         [v,w, isContinue] = RunP2P_2d(lp_A2B, vsl);
    %         if isContinue == false
    %             clockwise = true;
    %             position = vsl.poseWcsList(end,:);
    %             position(3) = position(3) + pi/2;
    %             SetLocalPlanner(lp, position, clockwise);
    %             plannerState = 2;
    %         end
    %     else
    %         [v,w, isContinue] = PlanZigzagRoout(lp, vsl, true);
    %     end
    
    %%
    
    if ~zigzagMainDone
        if isContinueZigzag
            [v, w , isContinueZigzag] = PlanZigzagRoout(lp, vsl, true);
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
    %     SetCmdVelocity(communication, 0, 0);
    
    %     CompareGoldenTraceAndVisionTrace(visualizeResult,vsl,bsl,77, imgHeader(1), imgHeader(2));
    %     CompareGoldenAngAndVisionAng(visualizeResult,vsl,bsl,133, imgHeader(1), imgHeader(2));
    %     VisualizeLocalPlanner(visualizeResult,vsl,bsl,lp, 123);
    debug_show_local_planner2(db,visualizeResult,vsl, 123)
    debug.debug_914_compare_goldenData(vsl,bsl,goldenData,bslTimeStamp,vslTimeStamp, GetMotionState(bsl, 'last'), true)
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
    drawnow;
    SaveFrame(visualizeResult, 123);
    nCounter = nCounter + 1;
end
CloseInput(cam);
Close(bsbfr);
CloseVid(visualizeResult);

end