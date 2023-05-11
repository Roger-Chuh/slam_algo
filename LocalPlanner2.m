classdef LocalPlanner2 < Configurable
    properties
        minR;
        moveDisMax;
        moveDisMin;
        thetaRange;
        plannerState;
        nextRoadMap;
        scaleForObstacleRange;
        prevPose;
        v;
        w;
        stillFrameNum;
        
        angleThreshold;
        positionThreshold;
        maxStillFrameNum;
        minPtNumToDetect;
        
        FIND_ROAD_MAP = 1;
        MOVE_STRAIGHT = 2;
        PURE_ROTATION = 3;
        CHECK_SURROUNDING = 4;
        OBSERVE_SURROUNDING = 5;
        STILL_FLAG;
        
        CLOCKWISE;
    end
    
    methods
        function obj = LocalPlanner2(cfgParam)
            obj@Configurable(cfgParam);
            obj.maxStillFrameNum = 3;
            obj.stillFrameNum = 0;
            obj.v = 0.4; % unit: m/s
            obj.w = 0.4;% unit: rad/s
            obj.minR = 0.5; % unit: m
            obj.moveDisMax = 0.4; % unit: m
            obj.moveDisMin = 0.1; % unit: m
            
            obj.thetaRange = pi/2 + pi/10; % unit: rad
            obj.angleThreshold = obj.w*0.1*1.1; % unit: rad
            obj.positionThreshold = obj.v*0.1*1; % unit: m
            obj.scaleForObstacleRange = 1.44;
            obj.minPtNumToDetect = 10;
            obj.plannerState = obj.FIND_ROAD_MAP;
            obj.STILL_FLAG = false;
            obj.CLOCKWISE = true;
        end
        
        function SetClockwiseForObstacleAvoidance(obj, clockwise)
            obj.CLOCKWISE = clockwise;
        end

        function nextRoadMap = PredictObservationAngle(obj, obstacle, position)
                nextRoadMap = position;
                if obj.CLOCKWISE
                    nextRoadMap(3) = nextRoadMap(3) + 0.5*pi;
                else
                    nextRoadMap(3) = nextRoadMap(3) - 0.5*pi;
                end
        end
        
        function [v,w] = ObstacleAvoidance(obj, obstacle, position)
            if obj.STILL_FLAG
                if obj.stillFrameNum <= obj.maxStillFrameNum - 1
                    obj.stillFrameNum = obj.stillFrameNum + 1;
                    v = 0;
                    w = 0;
                    return;
                else
                    obj.stillFrameNum = 0;
                    v = 0;
                    w = 0;
                    obj.STILL_FLAG = false;
                    return;
                end                
            else
                if ~isempty(obstacle)&&~isempty(obj.nextRoadMap)
                    dis2 = (obstacle(:,1) - obj.nextRoadMap(:,1)).^2 + (obstacle(:,2) - obj.nextRoadMap(:,2)).^2;
                    minDis2 = min(dis2);
                    if minDis2 <= 0.81 * obj.minR*obj.minR
                        obj.plannerState = obj.FIND_ROAD_MAP;
                    end
                end
                
                if obj.plannerState == obj.CHECK_SURROUNDING
                    if IsNeedToObserve(obj,obstacle, position)
                        obj.nextRoadMap = PredictObservationAngle(obj, obstacle, position);
                        obj.plannerState = obj.OBSERVE_SURROUNDING;
                    else
                        obj.nextRoadMap = FindRoadMap(obj, obstacle, position);
                        obj.plannerState = obj.PURE_ROTATION;
                        obj.prevPose = position;
                    end                        
                elseif obj.plannerState == obj.FIND_ROAD_MAP
                    obj.nextRoadMap = FindRoadMap(obj, obstacle, position);
                    obj.plannerState = obj.PURE_ROTATION;
                    obj.prevPose = position;
                end
                
                if obj.plannerState == obj.PURE_ROTATION
                    [v, w, obj.plannerState] = PureRotationMode(obj, position);
                elseif obj.plannerState == obj.MOVE_STRAIGHT
                    [v, w, obj.plannerState] = MoveStraightMode(obj, position);
                elseif obj.plannerState == obj.OBSERVE_SURROUNDING
%                     if ~IsNeedToObserve(obj,obstacle, position)
%                         obj.nextRoadMap = FindRoadMap(obj, obstacle, position);
%                         obj.plannerState = obj.PURE_ROTATION;
%                         obj.prevPose = position;
%                         obj.STILL_FLAG = true;
%                         v = 0;
%                         w = 0;
%                     else
                        [v, w, obj.plannerState] = ObservationMode(obj, position);
%                     end
                else
                    v = 0;
                    w= 0;
                end
            end
        end       

        function flag = IsNeedToObserve(obj,obstacle, position)
            if ~isempty(obstacle)
                dis2 = (obstacle(:,1) - position(:,1)).^2 + (obstacle(:,2) - position(:,2)).^2;
                obstacleId = find(dis2 < 2.25*obj.minR^2);
                obstacleInSight = obstacle(obstacleId,:);
                [~,id] = min(dis2);
                nearestOb = obstacle(id,:);
            else
                obstacleInSight = [];
            end
            
            if ~isempty(obstacleInSight)
                x1 = position(1);
                y1 = position(2);
                x2 = nearestOb(1);
                y2 = nearestOb(2);
                ptInWay = [];
                ptOutWay = [];
                for i = 1:size(obstacleInSight,1)
                    x = obstacleInSight(i,1);
                    y = obstacleInSight(i,2);
                    n1 = [x2-x1;y2-y1;0];
                    n2 = [x-x2;y-y2;0];
                    n3 = cross(n1,n2);
                    %                     if (y2-y1)*(x-x1)-(x2-x1)*(y-y1) > 0
                    if obj.CLOCKWISE
                        if n3(3) <0
                            ptInWay = [ptInWay; x,y];
                        else
                            ptOutWay = [ptOutWay; x,y];
                        end
                    else
                        
                        if n3(3) >0
                            ptInWay = [ptInWay; x,y];
                        else
                            ptOutWay = [ptOutWay; x,y];
                        end
                    end
                end

                if ~isempty(ptInWay)
                    ptInWayDis2 = (ptInWay(:,1)-x2).^2 + (ptInWay(:,2)-y2).^2;
                    maxPtInWayDis2 = max(ptInWayDis2);
%                     figure, plot(ptInWay(:,1),ptInWay(:,2),'ob')
%                     %                 figure, plot(obstacleInSight(:,1), obstacleInSight(:,2),'og')
%                     hold on
%                     plot(ptOutWay(:,1),ptOutWay(:,2),'og')
%                     plot(position(1), position(2),'*r');
%                     plot(nearestOb(1),nearestOb(2),'xr');
                else
                    maxPtInWayDis2 = 0;
                end
%                 max(ptInWayDis2)
                %                 if min(counterL,counterR) < obj.minPtNumToDetect || min(counterL,counterR)/max(counterL,counterR) < 0.15
                if maxPtInWayDis2 < 0.1^2
                    flag = true;
                else
                    flag = false;
                end
                
            else
               flag = false; 
            end
        end
        
        
        function [v, w, state] = ObservationMode(obj,position)
            if abs(obj.nextRoadMap(3)-position(3)) < obj.angleThreshold
                v = 0; % unit: m/s
                w = 0; % unit: rad/s
                obj.stillFrameNum = 1;
                obj.STILL_FLAG = true;
                state = obj.FIND_ROAD_MAP;
            else
                if obj.nextRoadMap(3)-position(3) > 0
                    v = 0; % unit: m/s
                    w = obj.w; % unit: rad/s
                else
                    v = 0; % unit: m/s
                    w = -obj.w; % unit: rad/s
                end
                state = obj.OBSERVE_SURROUNDING;
            end
        end
        
        function [v, w, state] = PureRotationMode(obj,position)
            if abs(obj.nextRoadMap(3)-position(3)) < obj.angleThreshold
                v = 0; % unit: m/s
                w = 0; % unit: rad/s
                obj.stillFrameNum = 1;
                obj.STILL_FLAG = true;
                state = obj.MOVE_STRAIGHT;
            else
                if obj.nextRoadMap(3)-position(3) > 0
                    v = 0; % unit: m/s
                    w = obj.w; % unit: rad/s
                else
                    v = 0; % unit: m/s
                    w = -obj.w; % unit: rad/s
                end
                state = obj.PURE_ROTATION;
            end            
        end
        
        function [v, w, state] = MoveStraightMode(obj, position)
            if norm(position(1:2)-obj.nextRoadMap(1:2)) < obj.positionThreshold
                v = 0; % unit: m/s
                w = 0; % unit: rad/s
                obj.stillFrameNum = 1;
                obj.STILL_FLAG = true;
                state = obj.CHECK_SURROUNDING;
            else
                v = obj.v; % unit: m/s
                w = 0; % unit: rad/s
                state = obj.MOVE_STRAIGHT;
            end
        end
        
        function [nextRoadMap] = FindRoadMap(obj, obstacle, position)
            if ~isempty(obstacle)
                theta = position(3) - obj.thetaRange: 0.01:position(3) + obj.thetaRange;
                dis = [];
                dis2 = (obstacle(:,1) - position(:,1)).^2 + (obstacle(:,2) - position(:,2)).^2;
                [~,dis2MinId] = min(dis2);
                obstacleId = find(dis2 < obj.scaleForObstacleRange*(obj.moveDisMax+ obj.minR)^2);
                %             validObstacle = obstacleId ~= dis2MinId;
                %             obstacleInSight = obstacle(obstacleId(validObstacle),:);
                obstacleInSight = obstacle(obstacleId,:);
            else
                obstacleInSight = [];
            end
            
            if ~isempty(obstacleInSight)
                scale = (obj.moveDisMax-obj.moveDisMin)/obj.thetaRange;
                
                
                for i = 1:length(theta)
                    moveDis = obj.moveDisMax - abs(position(3) - theta(i))*scale;
                    A = position(1:2)';
                    B = A + moveDis*[cos(theta(i));sin(theta(i))];
                    tempDis = [];
                    tempP2PDis = [];
                    for j = 1:size(obstacleInSight,1)
                        P =  obstacleInSight(j,:)';
                        tempDis(j) =  LocalPlanner2.distP2S(P,A,B);
                        tempP2PDis(j) = norm(P-B);
                    end
                    [tempDisMin, nearID] = min(tempDis);
                    tempP2PDisMin = min(tempP2PDis);
                    P = obstacleInSight(nearID,:)';
                    n1 = [B-A;0];
                    n2 = [P-A;0];
                    n3 = cross(n1,n2);
                    
                    if obj.CLOCKWISE
                        if n3(3) < 0 %% && tempDisMin < obj.minR*0.9
                            dis(i) = -1;
                            p2pDis(i) = -1;
                        else
                            dis(i) = tempDisMin;
                            p2pDis(i) = tempP2PDisMin;
                        end
                    else                        
                        if n3(3) > 0 %% && tempDisMin < obj.minR*0.9
                            dis(i) = -1;
                            p2pDis(i) = -1;
                        else
                            dis(i) = tempDisMin;
                            p2pDis(i) = tempP2PDisMin;
                        end
                    end
                end
                
                if max(dis) < obj.minR
                    
                    [minval, minid] = min(abs(p2pDis-obj.minR));
                    thetaPrime = theta(minid);
                    moveDis = obj.moveDisMax - abs(position(3) - thetaPrime)*scale;
                    nextRoadMap = [A + moveDis*[cos(thetaPrime);sin(thetaPrime)];thetaPrime]';
                    %                     plot(nextRoadMap(1),nextRoadMap(2),'xr');hold on
                    
                else
                    [minval, minid] = min(abs(dis-obj.minR));
                    thetaPrime = theta(minid);
                    moveDis = obj.moveDisMax - abs(position(3) - thetaPrime)*scale;
                    nextRoadMap = [A + moveDis*[cos(thetaPrime);sin(thetaPrime)];thetaPrime]';
                    %                     plot(nextRoadMap(1),nextRoadMap(2),'xr');hold on
                    
                end
                
            else
                thetaPrime = position(3);
                nextRoadMap = [position(1:2)' + obj.moveDisMax*[cos(thetaPrime);sin(thetaPrime)];thetaPrime]';
            end
        end
        
        
    end
    
    % interface
    methods
        function [v,w] = PlanLocalRoute(obj, vsl, bsl)
            if ~isempty(vsl.pointCloudManager.pt3D)
                obstacle = [vsl.pointCloudManager.pt3D.Location(:,1),vsl.pointCloudManager.pt3D.Location(:,3)];
                obstacle = round(obstacle*1000)/1000;
                obstacle = unique(obstacle,'rows');
                position = vsl.poseWcsList(end,:);
                position(3) = position(3)+pi/2;
            else
                obstacle = [];
                position = vsl.poseWcsList(end,:);
                position(3) = position(3)+pi/2;
            end
            [v,w] = ObstacleAvoidance(obj, obstacle, position);
        end
        
        function [v,w] = PlanLocalRoute2(obj, vsl)
            if ~isempty( vsl.pointCloudManager.map2D)
                obstacle = vsl.pointCloudManager.map2D;
                position = vsl.poseWcsList(end,:);
                position(3) = position(3)+pi/2;
            else
                obstacle = [];
                position = vsl.poseWcsList(end,:);
                position(3) = position(3)+pi/2;
            end
            [v,w] = ObstacleAvoidance(obj, obstacle, position);
        end
    end
    
    methods (Static)
        function dis = distP2S(P,A,B)
            AP = P - A;
            AB = B - A;
            BA = A - B;
            BP = P - B;
            
            angleA = acos(dot(AP,AB)/norm(AP)/norm(AB));
            angleB = acos(dot(BA,BP)/norm(BA)/norm(BP));
            
            if angleA >= pi/2
                dis = norm(AP);
                return
            end
            
            if angleB >= pi/2
                dis = norm(BP);
                return
            end
            
            area = abs((A(1)*(B(2)-P(2)) + B(1)*(P(2)-A(2)) + P(1)*(A(2)-B(2)))/2);
            dis = area*2/norm(AB);            
        end        
    end
    
    methods
%         Implementation of abstract methods in Configurable base class
        function SetDefaultValue(obj, cfgParam)
            %             cfgParam = Configurable.SetField(cfgParam, 'min_feature_point_number', 100);
            
            
            obj.configParam = cfgParam;
        end
        
        function CheckConfig(obj) %#ok<MANU>
        end
    end
end