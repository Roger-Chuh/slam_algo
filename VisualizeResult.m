classdef VisualizeResult  < handle
   properties
       goldenTrace;
       timeOffset;
       maxDepth;
       fovx;
       
       vid;
       LOAD_GOLDEN_DATA;
       PLOT_BODY_SENSOR_TRACE;
       PLOT_BODY_SENSOR_ANG;
       SAVE_VID;
       
       prevTime;
       currTime;
   end
    
    methods 
        function obj = VisualizeResult()
            obj.goldenTrace = [];
%             obj.timeOffset = 19634;
            obj.timeOffset = 19200;
            
            obj.maxDepth = 0.2;
            obj.fovx = pi/6;

            obj.PLOT_BODY_SENSOR_TRACE = true;
            obj.PLOT_BODY_SENSOR_ANG = true;
        end
        
        function obj = LoadGolden(obj ,goldenPath, LOAD_GOLDEN_DATA)
            obj.LOAD_GOLDEN_DATA = LOAD_GOLDEN_DATA;
            if obj.LOAD_GOLDEN_DATA > 0
                obj.goldenTrace = load(goldenPath);
                obj.goldenTrace(:,3) = obj.goldenTrace(:,3)/1000;
                obj.goldenTrace(:,4) = obj.goldenTrace(:,4)/1000;
            else
                warning('cannot find golden trace.')
            end
        end
        
        function CompareGoldenTraceAndVisionTrace(obj,vsl,bsl,FigNum, frmInd, frmTimestamp)
            figure(FigNum), hold on
            title(sprintf('Frame %d. BS: %s', frmInd, strrep(GetMotionStateStr(bsl), '_', '\_')))
            if obj.PLOT_BODY_SENSOR_TRACE
                if Rows(bsl.poseWcsList) < 2
                    plot(bsl.poseWcsList(end,1), bsl.poseWcsList(end,2),'+r'); axis equal;
                else
                    line([bsl.poseWcsList(end,1),bsl.poseWcsList(end-1,1)],...
                        [bsl.poseWcsList(end,2),bsl.poseWcsList(end-1,2)],'Color',[1 0 0],'LineWidth',1);axis equal;
                    plot(bsl.poseWcsList(end,1), bsl.poseWcsList(end,2),'+r');axis equal;
                end
            end
            
            if Rows(vsl.poseWcsList) < 2
                plot(vsl.poseWcsList(end,1), vsl.poseWcsList(end,2),'ob');axis equal;
            else
                line([vsl.poseWcsList(end,1),vsl.poseWcsList(end-1,1)],...
                    [vsl.poseWcsList(end,2),vsl.poseWcsList(end-1,2)],'Color',[0 0 1],'LineWidth',1);axis equal;
                plot(vsl.poseWcsList(end,1), vsl.poseWcsList(end,2),'ob');axis equal;
            end
            
            if obj.LOAD_GOLDEN_DATA > 0
                %                 currentTime = frmInd*0.08 - obj.timeOffset;
                %                 frmTimestamp
                id = find(obj.goldenTrace(:,1) <= (frmTimestamp - obj.timeOffset)/1000);
                if ~isempty(id)
                    plot(obj.goldenTrace(id,3),obj.goldenTrace(id,4),'-g');axis equal;
                end
            end
            
        end
        
        function CompareGoldenAngAndVisionAng(obj,vsl,bsl,FigNum, frmInd, frmTimestamp)
            figure(FigNum), hold on
            obj.currTime = (frmTimestamp - obj.timeOffset)/1000;
            if obj.PLOT_BODY_SENSOR_ANG
                if Rows(bsl.poseWcsList) < 2
                    plot(obj.currTime, rad2deg(bsl.poseWcsList(end,3)),'+r'); 
                else
                    line([obj.prevTime,obj.currTime],...
                        [rad2deg(bsl.poseWcsList(end-1,3)),rad2deg(bsl.poseWcsList(end,3))],'Color',[1 0 0],'LineWidth',1);
                    plot(obj.currTime, rad2deg(bsl.poseWcsList(end,3)),'+r');
                end
            end
            
            if Rows(vsl.poseWcsList) < 2
                plot(obj.currTime, rad2deg(vsl.poseWcsList(end,3)),'ob');
            else
                line([obj.prevTime,obj.currTime],...
                    [rad2deg(vsl.poseWcsList(end-1,3)),rad2deg(vsl.poseWcsList(end,3))],'Color',[0 0 1],'LineWidth',1);
                plot(obj.currTime, rad2deg(vsl.poseWcsList(end,3)),'ob');
            end
            
            if obj.LOAD_GOLDEN_DATA > 0
                %                 currentTime = frmInd*0.08 - obj.timeOffset;
                %                 frmTimestamp
                id = find(obj.goldenTrace(:,1) <= (frmTimestamp - obj.timeOffset)/1000);
                if ~isempty(id)
                    plot(obj.goldenTrace(id,1),obj.goldenTrace(id,2),'*g');
                end
            end
            
            obj.prevTime = obj.currTime;
        end

        
        function PlotMap2D(obj,vsl,bsl,FigNum, frmInd, frmTimestamp)

            zSize = vsl.pointCloudManager.configParam.map2d_z_range; % unit: meter
            xSize = vsl.pointCloudManager.configParam.map2d_x_range; % unit: meter
            subGridSize = vsl.pointCloudManager.configParam.map2d_subgrid_size; % unit: meter
            gridSize = vsl.pointCloudManager.configParam.map2d_grid_size; % unit: meter
            height = zSize/subGridSize;
            weight = xSize/subGridSize;
            offsetX = vsl.pointCloudManager.configParam.map2d_x_offset; %
            offsetZ = vsl.pointCloudManager.configParam.map2d_z_offset; %
            
            
            if ~isempty(vsl.pointCloudManager.pt3D) && ~isempty(vsl.pointCloudManager.trace)
                subGridNum = gridSize/subGridSize;
                subMap = zeros(height,weight);
                globalMap = zeros(height,weight);
                topDownMap = [zSize - vsl.pointCloudManager.pt3D.Location(:,3)-offsetZ, vsl.pointCloudManager.pt3D.Location(:,1)+offsetX];
                topDownTrace = [zSize - vsl.pointCloudManager.trace.Location(:,3)-offsetZ, vsl.pointCloudManager.trace.Location(:,1)+offsetX];
                subMapID = ceil(topDownMap/subGridSize);
                subTraceID = ceil(topDownTrace/subGridSize);
                mapID = ceil(topDownMap/gridSize);
                traceID = ceil(topDownTrace/gridSize);
                for i = 1:Rows(subMapID)
                    subMap(subMapID(i,1),subMapID(i,2)) = subMap(subMapID(i,1),subMapID(i,2)) + 1;
                end
                
                for i = 1:Rows(mapID)
                    globalMap((mapID(i,1)-1)*subGridNum + 1:(mapID(i,1))*subGridNum,(mapID(i,2)-1)*subGridNum + 1:(mapID(i,2))*subGridNum) = ...
                        globalMap((mapID(i,1)-1)*subGridNum + 1:(mapID(i,1))*subGridNum,(mapID(i,2)-1)*subGridNum + 1:(mapID(i,2))*subGridNum) + 1;
                end
                
                vsl.pointCloudManager.subMap2D = subMap;
                vsl.pointCloudManager.globalMap2D = globalMap;
                height = zSize/subGridSize;
                weight = xSize/subGridSize;
                maxVal = max(max(globalMap));
                newMap = 255 * ones(height, weight, 3, 'uint8');
                newMap(:,:,1) = 255 - uint8(globalMap/maxVal*255);
                newMap(:,:,2) = 255 - uint8(globalMap/maxVal*255);
                newMap(:,:,3) = 255 - uint8(globalMap/maxVal*255);
                
                viewGrid = 2;
                for i = 1:Rows(traceID)
                    tempLocalMap = subMap((traceID(i,1)-1 - viewGrid)*subGridNum + 1:(traceID(i,1) + viewGrid)*subGridNum,...
                        (traceID(i,2)-1 - viewGrid)*subGridNum + 1:(traceID(i,2)+viewGrid)*subGridNum);
                    tempLocalMap = tempLocalMap == 0;
                    newMap((traceID(i,1)-1 - viewGrid)*subGridNum + 1:(traceID(i,1) + viewGrid)*subGridNum,...
                        (traceID(i,2)-1 - viewGrid)*subGridNum + 1:(traceID(i,2) + viewGrid)*subGridNum,1) = uint8(255*tempLocalMap);
                    newMap((traceID(i,1)-1 - viewGrid)*subGridNum + 1:(traceID(i,1) + viewGrid)*subGridNum,...
                        (traceID(i,2)-1 - viewGrid)*subGridNum + 1:(traceID(i,2) + viewGrid)*subGridNum,2) = uint8(255*tempLocalMap);
                    newMap((traceID(i,1)-1 - viewGrid)*subGridNum + 1:(traceID(i,1) + viewGrid)*subGridNum,...
                        (traceID(i,2)-1 - viewGrid)*subGridNum + 1:(traceID(i,2) +viewGrid)*subGridNum,3) = uint8(255);
                end
                figure(FigNum), hold on
                imshow(newMap)
                realTrace = topDownTrace/subGridSize;
                figure(FigNum), hold on
                plot(realTrace(:,2),realTrace(:,1),'.g')
                
                if obj.LOAD_GOLDEN_DATA > 0
                    %                 currentTime = frmInd*0.08 - obj.timeOffset;
                    %                 frmTimestamp
                    id = find(obj.goldenTrace(:,1) <= (frmTimestamp - obj.timeOffset)/1000);
                    vTrace = [obj.goldenTrace(id,3)+offsetX,zSize - obj.goldenTrace(id,4)-offsetZ];
                    vTrace = vTrace/subGridSize;
                    if ~isempty(id)
                        figure(FigNum), hold on
                        plot(vTrace(:,1),vTrace(:,2),'-r');axis equal;
                    end
                end
            else
                newMap = 255 * ones(height, weight, 3, 'uint8');
                figure(FigNum), hold on
                imshow(newMap)
            end
            
            hold off
        end

        
        function LoadResultVidFile(obj, videoFilePath, SAVE_VID)            
            if SAVE_VID
                obj.SAVE_VID = SAVE_VID;
                obj.vid = VideoWriter(videoFilePath);
                obj.vid.FrameRate = 12.5;
                open(obj.vid);
            end
        end
        
        function SaveFrame(obj, figNum)
            if obj.SAVE_VID
                figure(figNum);
                saveas(gcf,'temp.png');
                frameImg = imread('temp.png');
                writeVideo(obj.vid, imresize(frameImg,[1275, 1116]));
            end
        end
        
        function SaveRawFrame(obj, img, figNum)
            figure(figNum);
            imshow(img);
            writeVideo(obj.vid, img);
        end
        
        function SaveStereoFrame(obj, imgL, imgR, figNum)
            figure(figNum);
            img = [imgL,imgR];
            imshow(img);
            writeVideo(obj.vid, img);
        end
        
        function CloseVid(obj)
            close(obj.vid);
        end
               
        function vision3D()
            scene = pcplayer([-3, 3], [-3, 3], [-1, 10], 'VerticalAxis', 'y', ...
                'VerticalAxisDir', 'down');
            offset = 117*0.1;
            currentTime = frmInd*0.1 - offset;
            id = find(goldenBuffer(:,1) <= currentTime);
            if ~isempty(id)
                poseVec = [goldenBuffer(id,3),goldenBuffer(id,4)]/1000;
                traceVec = [poseVec(:,1),zeros(Rows(poseVec),1),poseVec(:,2)];
                goldenTrace = pointCloud(traceVec);
                goldenTrace.Color = uint8([zeros(Rows(poseVec),1), 255*ones(Rows(poseVec),1),zeros(Rows(poseVec),1)]);
                if ~isempty(vsl.pointCloudManager.pt3D) && ~isempty(goldenTrace)
                    tempMap = pcmerge(vsl.pointCloudManager.trace, vsl.pointCloudManager.pt3D, 0.0001);
                    tempMap = pcmerge(tempMap, goldenTrace, 0.0001);
                elseif isempty(vsl.pointCloudManager.pt3D) && ~isempty(goldenTrace)
                    tempMap = pcmerge(vsl.pointCloudManager.trace, goldenTrace, 0.0001);
                elseif ~isempty(vsl.pointCloudManager.pt3D) && isempty(goldenTrace)
                    tempMap = pcmerge(vsl.pointCloudManager.trace, vsl.pointCloudManager.pt3D, 0.0001);
                else
                    tempMap = vsl.pointCloudManager.trace;
                end
                view(scene, tempMap);
            end
        end
        
        function VisualizeLocalPlanner(obj, vsl, bsl, lp, figNum)
            figure(figNum), clf
            if ~isempty(vsl.pointCloudManager.pt3D)
                obstacle = [vsl.pointCloudManager.pt3D.Location(:,1),vsl.pointCloudManager.pt3D.Location(:,3)];
                position = bsl.poseWcsList(end,:);
                position(3) = position(3)+pi/2;
                trace = [bsl.poseWcsList(:,1),bsl.poseWcsList(:,2)];
                
                plot(obstacle(:,1),obstacle(:,2),'.b'); hold on;
                plot(trace(:,1),trace(:,2),'-g'),hold on;
                tri = VisualizeResult.VisionWindow(position, obj.maxDepth, obj.fovx);
                plot(position(1),position(2),'or'),hold on;
                plot([tri(1:end,1);tri(1,1)],[tri(1:end,2);tri(1,2)],'-r');hold on;
                plot(lp.nextRoadMap(1),lp.nextRoadMap(2),'xr');hold on
                axis([-5 5 -1 10])
                
            else
                obstacle = [];
                position = bsl.poseWcsList(end,:);
                position(3) = position(3)+pi/2;
                trace = [bsl.poseWcsList(:,1),bsl.poseWcsList(:,2)];                

                plot(trace(:,1),trace(:,2),'-g'),hold on;
                tri = VisualizeResult.VisionWindow(position, obj.maxDepth, obj.fovx);
                plot(position(1),position(2),'or'),hold on;
                plot([tri(1:end,1);tri(1,1)],[tri(1:end,2);tri(1,2)],'-r');hold on;
                plot(lp.nextRoadMap(1),lp.nextRoadMap(2),'xr');hold on
                axis([-5 5 -1 10])
            end  
        end
        
        
    end
    
    methods (Static)
        function tri = VisionWindow(x, maxDepth, fovx)
            tri = [];
            tri(end + 1,:) = [x(1),x(2)];
            tri(end + 1,:) = [x(1) + maxDepth /cos(fovx)*cos(x(3) - fovx), x(2) + maxDepth/cos(fovx)*sin(x(3) - fovx)];
            tri(end + 1,:) = [x(1) + maxDepth/cos(fovx)*cos(x(3) + fovx), x(2) + maxDepth/cos(fovx)*sin(x(3) + fovx)];
        end
    end
        
end