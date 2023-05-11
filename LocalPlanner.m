classdef LocalPlanner  < Configurable
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
        
        angleThreshold;
        positionThreshold;
        
        FIND_ROAD_MAP = 1;
        MOVE_STRAIGHT = 2;
        PURE_ROTATION = 3;
    end
    
    methods
        function obj = LocalPlanner(cfgParam)
            obj@Configurable(cfgParam);
            
            obj.v = 0.4; % unit: m/s
            obj.w = 0.4;% unit: rad/s
            obj.minR = 0.5; % unit: m
            obj.moveDisMax = 0.4; % unit: m
            obj.moveDisMin = 0.1; % unit: m

            obj.thetaRange = pi/2 + pi/10; % unit: rad
            obj.angleThreshold = obj.w*0.1*1.1; % unit: rad
            obj.positionThreshold = obj.v*0.1*2; % unit: m
            obj.scaleForObstacleRange = 4;

            obj.plannerState = obj.FIND_ROAD_MAP;
        end
        
        function [v,w, state] = PureRotationMode(obj,position)
            if abs(obj.nextRoadMap(3)-position(3)) < obj.angleThreshold
                v = 0; % unit: m/s
                w = 0; % unit: rad/s
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
                state = obj.FIND_ROAD_MAP;
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
            obstacleId = find(dis2 < obj.scaleForObstacleRange*(obj.moveDisMax*obj.moveDisMax + obj.minR*obj.minR));
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
                        tempDis(j) =  LocalPlanner.distP2S(P,A,B);
                        tempP2PDis(j) = norm(P-B);
                    end
                    [tempDisMin, nearID] = min(tempDis);
                    tempP2PDisMin = min(tempP2PDis);
                    P = obstacleInSight(nearID,:)';
                    n1 = [B-A;0];
                    n2 = [P-A;0];
                    n3 = cross(n1,n2);
                    
                    if n3(3) > 0 %% && tempDisMin < obj.minR*0.9
                        dis(i) = -1;
                        p2pDis(i) = -1;
                    else
                        dis(i) = tempDisMin;
                        p2pDis(i) = tempP2PDisMin;
                    end
                end
                
                if max(dis) < obj.minR
                    
                    [minval, minid] = min(abs(p2pDis-obj.minR));
                    thetaPrime = theta(minid);
                    moveDis = obj.moveDisMax - abs(position(3) - thetaPrime)*scale;
                    nextRoadMap = [A + moveDis*[cos(thetaPrime);sin(thetaPrime)];thetaPrime]';
                    plot(nextRoadMap(1),nextRoadMap(2),'xr');hold on
                    
                else
                    [minval, minid] = min(abs(dis-obj.minR));
                    thetaPrime = theta(minid);
                    moveDis = obj.moveDisMax - abs(position(3) - thetaPrime)*scale;
                    nextRoadMap = [A + moveDis*[cos(thetaPrime);sin(thetaPrime)];thetaPrime]';
                    plot(nextRoadMap(1),nextRoadMap(2),'xr');hold on
                    
                end
                
            else
                thetaPrime = position(3);
                nextRoadMap = [position(1:2)' + obj.moveDisMax*[cos(thetaPrime);sin(thetaPrime)];thetaPrime]';
            end
        end
        
        function [v,w] = ObstacleAvoidance(obj, obstacle, position)
            if ~isempty(obstacle)
                dis2 = (obstacle(:,1) - obj.nextRoadMap(:,1)).^2 + (obstacle(:,2) - obj.nextRoadMap(:,2)).^2;
                minDis2 = min(dis2);
                %                 dis22 = (obstacle(:,1) - position(:,1)).^2 + (obstacle(:,2) - position(:,2)).^2;
                %                 minDis22 = min(dis22);
                %                 if minDis2 <= 0.36 * obj.minR*obj.minR || minDis22 >= 2.25*obj.minR*obj.minR
                
                if minDis2 <= 0.36 * obj.minR*obj.minR
                    obj.plannerState = obj.FIND_ROAD_MAP;
                end
            end

            
            if obj.plannerState == obj.FIND_ROAD_MAP
                obj.nextRoadMap = FindRoadMap(obj, obstacle, position);
                obj.plannerState = obj.PURE_ROTATION;
                obj.prevPose = position;
            end
            
            if obj.plannerState == obj.PURE_ROTATION
                [v,w, obj.plannerState] = PureRotationMode(obj, position);
            elseif obj.plannerState == obj.MOVE_STRAIGHT
                [v, w, obj.plannerState] = MoveStraightMode(obj, position);
            else
                v = 0;
                w= 0;
            end
        end       
        
        function [v,w] = PlanLocalRoute(obj, vsl, bsl)
            if ~isempty(vsl.pointCloudManager.pt3D)
                obstacle = [vsl.pointCloudManager.pt3D.Location(:,1),vsl.pointCloudManager.pt3D.Location(:,3)];
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
        % Implementation of abstract methods in Configurable base class
        function SetDefaultValue(obj, cfgParam)
            %             cfgParam = Configurable.SetField(cfgParam, 'min_feature_point_number', 100);
            
            
            obj.configParam = cfgParam;
        end
        
        function CheckConfig(obj) %#ok<MANU>
        end
    end
end