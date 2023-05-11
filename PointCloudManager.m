classdef PointCloudManager < Configurable
    
    properties
        depthMaps;  
        prevImg;
        scene3D;
        localScene3D;
        pt3D;
        trace;
        globalMap2D;
        subMap2D;
        map2D;
        
        trackCount;
        prevTrackedPt;
        trackedPtFlag;
        depthValidFlag;
        
        validPtFlag;
        mergedXYZ;
    end
    
    methods
        function obj = PointCloudManager(cfgParam)
            obj@Configurable(cfgParam);
            obj.depthMaps = [];
            obj.prevImg = [];
%             obj.scene3D = pcplayer([-3, 3], [-3, 3], [-1, 10], 'VerticalAxis', 'y', ...
%                 'VerticalAxisDir', 'down');
%             obj.localScene3D = pcplayer([-3, 3], [-3, 3], [0, 3], 'VerticalAxis', 'y', ...
%                 'VerticalAxisDir', 'down');
            obj.pt3D = [];
            obj.map2D = [];
            obj.trackCount = obj.configParam.merge_pt_max_tracking_num;
            obj.prevTrackedPt = [];
            obj.validPtFlag = [];
            obj.mergedXYZ = [];
                        
            zSize = obj.configParam.map2d_z_range; % unit: meter
            xSize = obj.configParam.map2d_x_range; % unit: meter
            subGridSize = obj.configParam.map2d_subgrid_size; % unit: meter
            
            height = zSize/subGridSize;
            weight = xSize/subGridSize;
            obj.subMap2D = zeros(height,weight);
            obj.globalMap2D = zeros(height,weight);
        end        
        
        function MergePointMap(obj)
            %             denseFlag = obj.denseDepthMap > 0 & obj.denseDepthMap <= 2000;
            global PATH_FIG
            intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,1);
            lastKeyInd = GetLastKeyFrameInd(obj);
            b2cP = GetPctBody2Cam(obj.coordSysAligner, 1);
            keyPose = obj.poseWcsList(lastKeyInd,:);
            k2wP = WcsPoseVecToPct(obj, keyPose);
            k2wP.transformMat(1:3,4) = 1000*k2wP.transformMat(1:3,4);
            
            gray = rgb2gray(obj.prevImgL);
            BW = edge(gray,'canny');
            ind = find(obj.denseDepthMap(:) > 0 & obj.denseDepthMap(:) <= 9000 & BW(:) == 1);
            if ~isempty(ind)
                [yIcs,xIcs] = ind2sub(size(obj.denseDepthMap),ind);
                xyIcs = [xIcs, yIcs];
                metricPrevPtCcs = intrMat\HomoCoord(xyIcs',1);
                metricPrevPtCcs = normc(metricPrevPtCcs);
                scale = obj.denseDepthMap(ind)./metricPrevPtCcs(3,:)';
                denseCcsXYZ = scale.*metricPrevPtCcs';
                densePointCouldXYZ = k2wP*Inv(b2cP)*denseCcsXYZ';
                densePointCouldXYZ = densePointCouldXYZ';
                denseFlag = densePointCouldXYZ(:,2) < 0 & densePointCouldXYZ(:,2) >= -1000;
                figure(222)
                hold on
                %                 scatter3(densePointCouldXYZ(denseFlag,1),densePointCouldXYZ(denseFlag,2),densePointCouldXYZ(denseFlag,3),'.g'); axis equal;
                pcshow(densePointCouldXYZ(denseFlag,:), [0 0 1], 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');axis equal;
                figure(PATH_FIG),
                hold on;
                plot(densePointCouldXYZ(:,1)/1000,densePointCouldXYZ(:,3)/1000,'.g')
            end
            
            %             kfBasedFrmInd = lastKeyInd - length(obj.keyFrameFlagList);
            %             [keyPtIcs, keyPtCcsZ, ~] = GetActiveFeatures(obj.featPtManager, kfBasedFrmInd);
            %             metricPrevPtCcs = intrMat\HomoCoord(keyPtIcs',1);
            %             metricPrevPtCcs = normc(metricPrevPtCcs);
            %             scale = keyPtCcsZ./metricPrevPtCcs(3,:)';
            %             keyCcsXYZ = scale.*metricPrevPtCcs';
            %             activeFlag = keyCcsXYZ(:,3) > 0 & keyCcsXYZ(:,3) <= 1500;
            %             if ~isempty(keyCcsXYZ(activeFlag,:))
            %                 pointCouldXYZ = k2wP*Inv(b2cP)*keyCcsXYZ(activeFlag,:)';
            %                 pointCouldXYZ = pointCouldXYZ';
            %                 figure(222)
            %                 hold on
            %                 scatter3(pointCouldXYZ(:,1),pointCouldXYZ(:,2),pointCouldXYZ(:,3),'.r'); axis equal;
            %                 figure(PATH_FIG),
            %                 hold on;
            %                 plot(pointCouldXYZ(:,1)/1000,pointCouldXYZ(:,3)/1000,'*r');
            %             end
            %             figure(22)
            %             imshow(obj.currImgL)
            %             hold on
            %             plot(keyPtIcs(activeFlag,1),keyPtIcs(activeFlag,2),'or')
        end
        
        function MergePointMap_OpticalFlow1D(obj, prevImgL, currImgL, intrMat, p2cPCam, b2cP, p2wPBody, c2wPBody)
            maxTrackingCount = obj.configParam.merge_pt_max_tracking_num;
            samePtThr = obj.configParam.same_point_threshold;
            if obj.trackCount >= maxTrackingCount
                obj.trackCount = 0;
                obj.prevTrackedPt = FindEdgePoints(prevImgL);
                obj.validPtFlag = true(Rows(obj.prevTrackedPt),1);
            end
            obj.trackCount = obj.trackCount + 1;
            if sum(obj.validPtFlag) ~= 0 && ~isempty(obj.prevTrackedPt)
                trackerMethod = 'epilineLK';
                switch trackerMethod
                    case 'epilineLK'
                            [currPt, obj.validPtFlag] = EpilineLK(prevImgL,currImgL,intrMat,...
                                p2cPCam.transformMat, obj.prevTrackedPt, obj.validPtFlag);
                    otherwise
                        assert(false,'no such tracker method')
                end
                
                prevDepthMap = obj.depthMaps(:,:,end-1);
                currDepthMap = obj.depthMaps(:,:,end);
                
                
                prevPt = obj.prevTrackedPt;
                obj.prevTrackedPt = currPt;
                prevZ = zeros(Rows(prevPt),1);
                currZ = zeros(Rows(prevPt),1);
                prevZ(obj.validPtFlag) = prevDepthMap(sub2ind(size(prevDepthMap), round(prevPt(obj.validPtFlag,2)), round(prevPt(obj.validPtFlag,1))));
                currZ(obj.validPtFlag) = currDepthMap(sub2ind(size(currDepthMap), round(currPt(obj.validPtFlag,2)), round(currPt(obj.validPtFlag,1))));
                
                prevCCSXYZ = PointCloudManager.Convert2DPtTo3DPt(intrMat,prevPt,prevZ);
                currCCSXYZ = PointCloudManager.Convert2DPtTo3DPt(intrMat,currPt,currZ);
                prevXYZ = (p2wPBody*Inv(b2cP)*prevCCSXYZ')';
                currXYZ = (c2wPBody*Inv(b2cP)*currCCSXYZ')';   
                
                tempDelta = prevXYZ - currXYZ;
                dis = sqrt(tempDelta(:,1).*tempDelta(:,1)+tempDelta(:,2).*tempDelta(:,2)+tempDelta(:,3).*tempDelta(:,3));
                obj.validPtFlag = prevCCSXYZ(:,3)>0 & currCCSXYZ(:,3)>0 & dis < samePtThr & obj.validPtFlag > 0;
                %                 debug.debug_913x_CheckOpticalFlowPointCloud(obj,prevPt,currPt,prevZ, currZ,goodPointID,dis,obj.DEBUG_FLAG&true)
                
                if obj.trackCount >= maxTrackingCount
                    
                    obj.mergedXYZ(obj.validPtFlag,:) = (currXYZ(obj.validPtFlag,:) + obj.mergedXYZ(obj.validPtFlag,:)*obj.trackCount)/(obj.trackCount + 1);
                    goodPointID = find(prevCCSXYZ(:,3)>0 & currCCSXYZ(:,3)>0 & dis <samePtThr & obj.validPtFlag > 0);
%                     colorVec = [];
%                     for i = 1:Rows(goodPointID)
%                         colorVec(end+1,:) = [currImgL(round(currPt(goodPointID(i),2)),round(currPt(goodPointID(i),1)),1),...
%                             currImgL(round(currPt(goodPointID(i),2)),round(currPt(goodPointID(i),1)),2),...
%                             currImgL(round(currPt(goodPointID(i),2)),round(currPt(goodPointID(i),1)),3)];
%                     end
                    flag = FilterPointCloudVec(obj, currCCSXYZ(goodPointID,:));
                    disp('number of remaining points1:')
                    disp(sum(obj.validPtFlag));
%                     StorePointCloudVec(obj, obj.mergedXYZ(obj.validPtFlag,:), uint8(colorVec), flag);
                    Store2DMap(obj, obj.mergedXYZ(obj.validPtFlag,:), flag)
                    disp('number of remaining points2:')
                    disp(sum(flag));
                else
                    if obj.trackCount == 1
                        obj.mergedXYZ = prevXYZ;                        
                    end
                    obj.mergedXYZ(obj.validPtFlag,:) = (currXYZ(obj.validPtFlag,:) + obj.mergedXYZ(obj.validPtFlag,:)*obj.trackCount)/(obj.trackCount + 1);
                    disp('number of remaining points:')
                    disp(sum(obj.validPtFlag));
                end

            end
        end
        
        function [lastKeyCcsXYZ, validFlag] = MergePointMap_AverageInverseDepthMethod(obj,intrMat, poseVec, coordSysAligner)
            %             denseFlag = obj.denseDepthMap > 0 & obj.denseDepthMap <= 2000;
            zMax = obj.configParam.point_cloud_max_z_range; % unit: mm
            zMin = obj.configParam.point_cloud_min_z_range; % unit: mm
            yMin = obj.configParam.point_cloud_min_y_range; % unit: mm
            yMax = obj.configParam.point_cloud_max_y_range; % unit: mm
            samePointThreshold = obj.configParam.same_point_threshold; % unit: mm
            
            numKeyframeStored = size(poseVec,1);
            b2cP = GetPctBody2Cam(coordSysAligner, 1);
            lastKeyDepMap = obj.depthMaps(:,:,end);
            numTotalPt = Rows(lastKeyDepMap)*Cols(lastKeyDepMap);
            
            rotMatToWcs = BaseLocalizer.RotMatFromAngle(poseVec(end,3))';
            transVecToWcs = [poseVec(end,1); 0; poseVec(end,2)];
            k2wP = PointCoordTransformer(inv(rotMatToWcs), transVecToWcs);
            k2wP.transformMat(1:3,4) = 1000*k2wP.transformMat(1:3,4);
            
            [yIcs,xIcs] = ind2sub(size(lastKeyDepMap),1:numTotalPt);
            xyIcs = [xIcs; yIcs];
            lastKeyCcsXYZ = PointCloudManager.Convert2DPtTo3DPt(intrMat,xyIcs',lastKeyDepMap(:));
            
            flag = true(numTotalPt,numKeyframeStored-1);
            
            for i = 1:numKeyframeStored-1
                rotMatToWcs = BaseLocalizer.RotMatFromAngle(poseVec(i,3))';
                transVecToWcs = [poseVec(i,1); 0; poseVec(i,2)];
                k02wP = PointCoordTransformer(inv(rotMatToWcs), transVecToWcs);
                k02wP.transformMat(1:3,4) = 1000*k02wP.transformMat(1:3,4);
                k02kP = k2wP\k02wP;
                prevKeyDepMap = obj.depthMaps(:,:,i);
                prevKeyCcsXYZ = PointCloudManager.Convert2DPtTo3DPt(intrMat,xyIcs',prevKeyDepMap(:));                
                prev2lastKeyCcsXYZ = [k02kP*prevKeyCcsXYZ']';
                projCurKeyIcs = round(PointCloudManager.Convert3DPtTo2DPt(intrMat, prev2lastKeyCcsXYZ));
                flag1 = lastKeyDepMap(:)>zMin & lastKeyDepMap(:)<zMax;
                flag2 = prevKeyDepMap(:)>zMin & prevKeyDepMap(:)<zMax;
                flag3 = projCurKeyIcs(:,1)>=1 & projCurKeyIcs(:,1) <= Cols(prevKeyDepMap) & projCurKeyIcs(:,2)>=1 & projCurKeyIcs(:,2) <= Rows(prevKeyDepMap);
                tempFlag = flag1&flag2&flag3;
                prevInd = find(tempFlag == 1);
                lastInd = [sub2ind(size(lastKeyDepMap),projCurKeyIcs(tempFlag,2), projCurKeyIcs(tempFlag,1))]';
                dis = sqrt(sum((prev2lastKeyCcsXYZ(prevInd,:)-lastKeyCcsXYZ(lastInd,:)).^2,2));
                overLapInd = find(dis <= samePointThreshold & lastKeyCcsXYZ(lastInd,3)>zMin);
                flag(lastInd(overLapInd),i) = false;
            end
            validFlag = sum(flag,2) ==  numKeyframeStored-1;
%             tempFlag = lastKeyCcsXYZ(:,3) > 0
%             xlabel('x');
%             ylabel('y');
%             zlabel('z');
%             pcshow(lastKeyCcsXYZ(tempFlag,:), [0 0 1], 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');axis equal;
        end
        
        function StoreImage(obj,img)
            maxKeyframeNum = obj.configParam.max_num_keyframes;
            if isempty(obj.prevImg)
                obj.prevImg(:,:,:,end) = img;
            elseif size(obj.prevImg,4) < maxKeyframeNum
                obj.prevImg(:,:,:,end + 1) = img;
            else
                obj.prevImg(:,:,:,1) = [];
                obj.prevImg(:,:,:,end + 1) = img;
            end            
        end
        
        function StoreDepthMap(obj,depthMap)
            maxKeyframeNum = obj.configParam.max_num_keyframes;
            if isempty(obj.depthMaps)
                obj.depthMaps(:,:,end) = depthMap;
            elseif size(obj.depthMaps,3) < maxKeyframeNum
                obj.depthMaps(:,:,end + 1) = depthMap;
            else
                obj.depthMaps(:,:,1) = [];
                obj.depthMaps(:,:,end + 1) = depthMap;
            end
        end
        
        function [mask] = GetPointCloudMask(obj, lastKeyCcsXYZ)
            zMax = obj.configParam.point_cloud_max_z_range; % unit: mm
            zMin = obj.configParam.point_cloud_min_z_range; % unit: mm
            yMin = obj.configParam.point_cloud_min_y_range; % unit: mm
            yMax = obj.configParam.point_cloud_max_y_range; % unit: mm
            xMin = obj.configParam.point_cloud_min_x_range; % unit: mm
            xMax = obj.configParam.point_cloud_max_x_range; % unit: mm
            gray = rgb2gray(uint8(obj.prevImg(:,:,:,end)));
            edgeMask = edge(gray,'canny');
            X = [vec2mat(lastKeyCcsXYZ(:,1),Rows(gray))]';
            Y = [vec2mat(lastKeyCcsXYZ(:,2),Rows(gray))]';
            Z = [vec2mat(lastKeyCcsXYZ(:,3),Rows(gray))]';
            rangeMask = X < xMax & X >= xMin & Y < yMax & Y >= yMin & Z < zMax & Z >= zMin;
            mask = edgeMask & rangeMask;
%             mask = rangeMask;
        end
        
        function StorePointCloud(obj, densePointCouldXYZ, mask)
                gridSize = obj.configParam.point_cloud_grid_size; % unit: m
                mask0 = repmat(mask,[1,1,3]);
                lastKeyImg = uint8(obj.prevImg(:,:,:,end));
                piontCould3D(:,:,1) = [vec2mat(densePointCouldXYZ(:,1),Rows(lastKeyImg))]';
                piontCould3D(:,:,2) = [vec2mat(densePointCouldXYZ(:,2),Rows(lastKeyImg))]';
                piontCould3D(:,:,3) = [vec2mat(densePointCouldXYZ(:,3),Rows(lastKeyImg))]';
                piontCould3D = piontCould3D./1000;
                piontCould3D(mask0 == 0) = nan;
                lastKeyImg(mask0 == 0) = nan;
                ptCloud = pointCloud(piontCould3D, 'Color', lastKeyImg);
                if isempty(obj.pt3D)
                    ptCloud = pcmerge(ptCloud, ptCloud, gridSize);
%                     if Rows(ptCloud.Location) > 10
%                         ptCloud = pcdenoise(ptCloud,'NumNeighbors', 4, 'Threshold',0.5);
%                     end
                    if Rows(ptCloud.Location) > 10
                        obj.pt3D = ptCloud;
                    end
%                     if Rows(obj.pt3D.Location) > 10                        
%                         obj.pt3D = pcdenoise(obj.pt3D,'NumNeighbors', 8, 'Threshold',0.5);
%                     end
                else
%                     if Rows(ptCloud.Location) > 10
%                         ptCloud = pcdenoise(ptCloud,'NumNeighbors', 4, 'Threshold',0.5);
%                     end
                    if Rows(ptCloud.Location) > 10
                        obj.pt3D = pcmerge(obj.pt3D, ptCloud, gridSize);
                    end
%                     if Rows(obj.pt3D.Location) > 10                        
%                         obj.pt3D = pcdenoise(obj.pt3D,'NumNeighbors', 8, 'Threshold',0.5);
%                     end
                end
                %                 view(obj.scene3D, obj.pt3D);
%                 obj.pt3D = pcdenoise(obj.pt3D,'NumNeighbors', 5, 'Threshold',0.5);

        end
        
        function flag = FilterPointCloudVec(obj, XYZ)
            zMax = obj.configParam.point_cloud_max_z_range; % unit: mm
            zMin = obj.configParam.point_cloud_min_z_range; % unit: mm
            yMin = obj.configParam.point_cloud_min_y_range; % unit: mm
            yMax = obj.configParam.point_cloud_max_y_range; % unit: mm
            xMin = obj.configParam.point_cloud_min_x_range; % unit: mm
            xMax = obj.configParam.point_cloud_max_x_range; % unit: mm
            
            flag = XYZ(:,1) < xMax & XYZ(:,1) >= xMin & XYZ(:,2) < yMax & XYZ(:,2) >= yMin & XYZ(:,3) < zMax & XYZ(:,3) >= zMin;
        end
        
        function Store2DMap(obj, XYZ, flag)
            tempMap2d = [XYZ(flag,1), XYZ(flag,3)];
            tempMap2d = round(tempMap2d/1000*20)/20;
            obj.map2D = [obj.map2D;tempMap2d];
            obj.map2D = unique(obj.map2D,'rows');
        end
        
        function StorePointCloudVec(obj, XYZ, colorVec, flag)
            gridSize = obj.configParam.point_cloud_grid_size; % unit: m


            ptCloud = pointCloud(XYZ(flag,:)/1000);
            ptCloud.Color = colorVec(flag,:);
            if isempty(obj.pt3D)
%                 ptCloud = pcmerge(ptCloud, ptCloud, gridSize);
%                 if Rows(ptCloud.Location) > 10
%                     ptCloud = pcdenoise(ptCloud,'NumNeighbors', 4, 'Threshold',0.5);
%                 end
                if Rows(ptCloud.Location) > 10

                    obj.pt3D = ptCloud;
                end
                %                     if Rows(obj.pt3D.Location) > 10
                %                         obj.pt3D = pcdenoise(obj.pt3D,'NumNeighbors', 8, 'Threshold',0.5);
                %                     end
            else
%                 if Rows(ptCloud.Location) > 10
%                     ptCloud = pcdenoise(ptCloud,'NumNeighbors', 4, 'Threshold',0.5);
%                 end
%                 if Rows(ptCloud.Location) > 10
                    obj.pt3D = pcmerge(obj.pt3D, ptCloud, gridSize);
%                 end
                %                     if Rows(obj.pt3D.Location) > 10
                %                         obj.pt3D = pcdenoise(obj.pt3D,'NumNeighbors', 8, 'Threshold',0.5);
                %                     end
            end
        end
        
        function StoreTrace(obj, poseVec, color)
            if strcmp(color,'red')
                colorVec = uint8([255*ones(Rows(poseVec),1),zeros(Rows(poseVec),1),zeros(Rows(poseVec),1)]);
            elseif strcmp(color,'green')                
                colorVec = uint8([zeros(Rows(poseVec),1), 255*ones(Rows(poseVec),1),zeros(Rows(poseVec),1)]);
            elseif strcmp(color,'blue')
                colorVec = uint8([zeros(Rows(poseVec),1),zeros(Rows(poseVec),1), 255*ones(Rows(poseVec),1)]);
            end
            
            traceVec = [poseVec(:,1),zeros(Rows(poseVec),1),poseVec(:,2)];
            localtrace = pointCloud(traceVec);
            localtrace.Color = colorVec;
            if isempty(obj.pt3D)
                localtrace = pcmerge(localtrace, localtrace, 0.001);
                obj.trace = localtrace;
            else
                obj.trace = pcmerge(obj.trace, localtrace, 0.001);
            end            
        end 
        
        function PlotSense(obj)
         if ~isempty(obj.pt3D) && ~isempty(obj.trace)
           tempMap = pcmerge(obj.trace, obj.pt3D, 0.015);           
         elseif isempty(obj.pt3D) && ~isempty(obj.trace)
           tempMap = obj.trace;
         elseif ~isempty(obj.pt3D) && isempty(obj.trace)
           tempMap = obj.pt3D;
         else
             assert(false, 'no pt3D and no trace');
         end
         view(obj.scene3D, tempMap);
        end
        
        function flag = Filter3DPoint(obj, depthMap, intrMat)
            zMax = obj.configParam.point_cloud_max_z_range; % unit: mm
            zMin = obj.configParam.point_cloud_min_z_range; % unit: mm
            yMin = obj.configParam.point_cloud_min_y_range; % unit: mm
            yMax = obj.configParam.point_cloud_max_y_range; % unit: mm
            xMin = obj.configParam.point_cloud_min_x_range; % unit: mm
            xMax = obj.configParam.point_cloud_max_x_range; % unit: mm
            numTotalPt = Rows(depthMap)*Cols(depthMap);
            [yIcs,xIcs] = ind2sub(size(depthMap),1:numTotalPt);
            xyIcs = [xIcs; yIcs];
            XYZ = PointCloudManager.Convert2DPtTo3DPt(intrMat,xyIcs',depthMap(:));
            flag = XYZ(:,1) < xMax & XYZ(:,1) >= xMin ...
                & XYZ(:,2) < yMax & XYZ(:,2) >= yMin ...
                & XYZ(:, 3) < zMax & XYZ(:,3) >= zMin;
        end
        
        function Plot2DMap(obj)
            if ~isempty(obj.pt3D) && ~isempty(obj.trace)
                zSize = obj.configParam.map2d_z_range; % unit: meter
                xSize = obj.configParam.map2d_x_range; % unit: meter
                subGridSize = obj.configParam.map2d_subgrid_size; % unit: meter
                gridSize = obj.configParam.map2d_grid_size; % unit: meter

                offsetX = obj.configParam.map2d_x_offset; % 
                offsetZ = obj.configParam.map2d_z_offset; %

                height = zSize/subGridSize;
                weight = xSize/subGridSize;
                subGridNum = gridSize/subGridSize;
                subMap = zeros(height,weight);
                globalMap = zeros(height,weight);
%                 subMap = obj.subMap2D;
%                 globalMap = obj.globalMap2D;
                topDownMap = [zSize - obj.pt3D.Location(:,3)-offsetZ, obj.pt3D.Location(:,1)+offsetX];
                topDownTrace = [zSize - obj.trace.Location(:,3)-offsetZ, obj.trace.Location(:,1)+offsetX];
                subMapID = ceil(topDownMap/subGridSize);
                subTraceID = ceil(topDownTrace/subGridSize);
                mapID = ceil(topDownMap/gridSize);
                traceID = ceil(topDownTrace/gridSize);
                for i = 1:Rows(subMapID)
                    subMap(subMapID(i,1),subMapID(i,2)) = subMap(subMapID(i,1),subMapID(i,2)) + 1;
                end
%                 for i = 1:Rows(subTraceID)
%                     subMap(subTraceID(i,1),subTraceID(i,2)) = 100;
%                 end

                
                for i = 1:Rows(mapID)
                    globalMap((mapID(i,1)-1)*subGridNum + 1:(mapID(i,1))*subGridNum,(mapID(i,2)-1)*subGridNum + 1:(mapID(i,2))*subGridNum) = ...
                        globalMap((mapID(i,1)-1)*subGridNum + 1:(mapID(i,1))*subGridNum,(mapID(i,2)-1)*subGridNum + 1:(mapID(i,2))*subGridNum) + 1;
                end
                
%                 for i = 1:Rows(traceID)
%                     tempLocalMap = subMap((traceID(i,1)-1)*subGridNum + 1:(traceID(i,1))*subGridNum,(traceID(i,2)-1)*subGridNum + 1:(traceID(i,2))*subGridNum);
%                     tempLocalMap = maxVal*(tempLocalMap > 0);
%                     globalMap((traceID(i,1)-1)*subGridNum + 1:(traceID(i,1))*subGridNum,(traceID(i,2)-1)*subGridNum + 1:(traceID(i,2))*subGridNum) = tempLocalMap;
%                 end                
                
                obj.subMap2D = subMap;
                obj.globalMap2D = globalMap;
                height = zSize/subGridSize;
                weight = xSize/subGridSize;
                maxVal = max(max(globalMap));
                newMap = 255 * ones(height, weight, 3, 'uint8');
                % newMap(:,:,1) = uint8(globalMap/maxVal*255)
                newMap(:,:,1) = 255 - uint8(globalMap/maxVal*255);
                newMap(:,:,2) = 255 - uint8(globalMap/maxVal*255);
                newMap(:,:,3) = 255 - uint8(globalMap/maxVal*255);
%                 figure(122),imshow(newMap)
%                 alpha_data = uint8(globalMap/maxVal*255)
%                 set(newMap, 'AlphaData', alpha_data);
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
                
%                 tempMap = max(max(globalMap)) - globalMap;
                figure(111)
%                 colormap(autumn(256))
%                 imagesc(tempMap)
                imshow(newMap)
                hold on
                realTrace = topDownTrace/subGridSize;
                plot(realTrace(:,2),realTrace(:,1),'.g')      
%                 figure(12)
%                 imshow(obj.subMap2D)
%                 figure(14)
%                 imshow(obj.globalMap2D)
            end
        end
    end
    
    methods (Static)
        function Pt3D = Convert2DPtTo3DPt(intrMat,Pt2D,depth)
            assert(Rows(Pt2D) == Rows(depth),'Pt2D should be the same size of depth!')
            metricPrevPtCcs = intrMat\HomoCoord(Pt2D',1);
            metricPrevPtCcs = normc(metricPrevPtCcs);
            scale = depth./metricPrevPtCcs(3,:)';
            Pt3D = scale.*metricPrevPtCcs';
        end
        
        function Pt2D = Convert3DPtTo2DPt(intrMat, Pt3D)
            projIcs = intrMat*Pt3D';
            projIcs = [projIcs(1,:)./projIcs(3,:);projIcs(2,:)./projIcs(3,:)];
            Pt2D = projIcs';
        end
        
        
    end
    
    methods
        % Implementation of abstract methods in Configurable base class
        function SetDefaultValue(obj, cfgParam)
            cfgParam = Configurable.SetField(cfgParam, 'point_cloud_max_z_range', 600);% unit: mm
            cfgParam = Configurable.SetField(cfgParam, 'point_cloud_min_z_range', 0); % unit: mm
            cfgParam = Configurable.SetField(cfgParam, 'point_cloud_max_y_range', 50); % unit: mm
            cfgParam = Configurable.SetField(cfgParam, 'point_cloud_min_y_range', -2000); % unit: mm
            cfgParam = Configurable.SetField(cfgParam, 'point_cloud_max_x_range', +inf); % unit: mm
            cfgParam = Configurable.SetField(cfgParam, 'point_cloud_min_x_range', -inf); % unit: mm
            cfgParam = Configurable.SetField(cfgParam, 'max_num_keyframes', 5); % unit: mm
            cfgParam = Configurable.SetField(cfgParam, 'point_cloud_grid_size', 0.02); % unit: m
            
            
            cfgParam = Configurable.SetField(cfgParam, 'map2d_x_range', 13); 
            cfgParam = Configurable.SetField(cfgParam, 'map2d_z_range', 15); 
            cfgParam = Configurable.SetField(cfgParam, 'map2d_subgrid_size', 0.05); 
            cfgParam = Configurable.SetField(cfgParam, 'map2d_grid_size', 0.2); 
            cfgParam = Configurable.SetField(cfgParam, 'map2d_x_offset', 8);
            cfgParam = Configurable.SetField(cfgParam, 'map2d_z_offset', 5);
            
            cfgParam = Configurable.SetField(cfgParam, 'merge_pt_max_tracking_num', 2); % unit: frame
            cfgParam = Configurable.SetField(cfgParam, 'same_point_threshold', 100); % unit: mm

            obj.configParam = cfgParam;
        end
        
        function CheckConfig(obj) %#ok<MANU>
        end
    end
    
end