classdef LocalPlanner_A2B < Configurable
    properties
        A;
        B;
        orgDir;
        dir;
        motionState;
        
        PURE_ROTATION = 1;
        MOVE_STRAIGHT = 2;
        FIND_DIRECTION = 3;
        STOP = 4;
        
        maxStillFrameNum = 3;
        stillFrameNum;
        STILL_FLAG;
        angleThreshold;
        positionThreshold;
        v = 0.4;
        w = 0.4;
        wmin = 0.1;
        minDis = 0.0000001;

    end
    
    methods
        function obj = LocalPlanner_A2B(cfgParam)

            obj@Configurable(cfgParam);
            obj.angleThreshold = obj.wmin*0.1*1.1; % unit: rad
            obj.positionThreshold = obj.v*0.1*2; % unit: m
            obj.stillFrameNum = 0;
            obj.motionState = obj.PURE_ROTATION;
            obj.dir = [];
            obj.STILL_FLAG = false;
        end
        
        function SetA2B(obj, curPosition, destination, direction)
            obj.A = curPosition(1:2);
            obj.B = destination(1:2);
            obj.orgDir = curPosition(3);
            obj.dir = direction;            
            obj.motionState = obj.PURE_ROTATION;
            obj.STILL_FLAG = false;

        end

        function [v,w, isContinue] = RunA2B(obj, obstacle, position)
            isContinue = 1;            
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
                if obj.motionState == obj.PURE_ROTATION
                    [v, w, obj.motionState] = PureRotationMode(obj,position);
                elseif obj.motionState == obj.MOVE_STRAIGHT
                    [v, w, obj.motionState] = MoveStraightMode(obj, position);
                elseif obj.motionState == obj.FIND_DIRECTION
                    if ~isempty(obj.dir)
                        [v, w, obj.motionState] = FindDirectionMode(obj, position);
                    else
                        v = 0;
                        w = 0;
                        isContinue = 0;
                        obj.motionState = obj.STOP;
                    end
                elseif obj.motionState == obj.STOP
                    v = 0;
                    w = 0;
                    isContinue = 0;
                end
            end
        end
    
        
        function [v, w, state] = PureRotationMode(obj,position)
            n1 = [cos(obj.orgDir);sin(obj.orgDir);0];
            n2 = [obj.B'-obj.A';0];
            n3 = cross(n1,n2);
            theta = acos(dot(n1,n2)/norm(n1)/norm(n2));
            if n3(3) > 0
                theta = obj.orgDir + theta;
            else
                theta = obj.orgDir - theta;
            end
            
            if abs(theta-position(3)) < obj.angleThreshold
                v = 0; % unit: m/s
                w = 0; % unit: rad/s
                obj.stillFrameNum = 1;
                obj.STILL_FLAG = true;
                state = obj.MOVE_STRAIGHT;
            elseif rad2deg(abs(theta-position(3))) > 5
                if theta-position(3) > 0
                    v = 0; % unit: m/s
                    w = obj.w; % unit: rad/s
                else
                    v = 0; % unit: m/s
                    w = -obj.w; % unit: rad/s
                end
                state = obj.PURE_ROTATION;
            else
                if theta-position(3) > 0
                    v = 0; % unit: m/s
                    w = obj.wmin; % unit: rad/s
                else
                    v = 0; % unit: m/s
                    w = -obj.wmin; % unit: rad/s
                end
                state = obj.PURE_ROTATION;
            end
            
        end
        
        function [v, w, state] = MoveStraightMode(obj, position)
            if norm(position(1:2)-obj.B) < obj.positionThreshold
                v = 0; % unit: m/s
                w = 0; % unit: rad/s
                obj.stillFrameNum = 1;
                obj.STILL_FLAG = true;
                state = obj.FIND_DIRECTION;
            else
                v = obj.v; % unit: m/s
                w = 0; % unit: rad/s
                state = obj.MOVE_STRAIGHT;
            end
        end
        
        function [v, w, state] = FindDirectionMode(obj, position)
            n1 = [cos(position(3));sin(position(3));0];
            n2 = [cos(obj.dir);sin(obj.dir);0];
            n3 = cross(n1,n2);
            theta = acos(dot(n1,n2)/norm(n1)/norm(n2));            
            if theta < obj.angleThreshold
                v = 0; % unit: m/s
                w = 0; % unit: rad/s
                obj.stillFrameNum = 1;
                obj.STILL_FLAG = true;
                state = obj.STOP;
            else
                if n3(3) > 0
                    v = 0; % unit: m/s
                    w = obj.wmin; % unit: rad/s
                else
                    v = 0; % unit: m/s
                    w = -obj.wmin; % unit: rad/s
                end
                state = obj.FIND_DIRECTION;
            end
        end
    end
        

  
    
    % interface
    methods
        function [v,w, isContinue] = RunP2P_3d(obj, vsl, bsl)
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
            [v,w, isContinue] = RunA2B(obj, obstacle, position);
        end
        
        function [v,w, isContinue] = RunP2P_2d(obj, vsl)
            if ~isempty( vsl.pointCloudManager.map2D)
                obstacle = vsl.pointCloudManager.map2D;
                position = vsl.poseWcsList(end,:);
                position(3) = position(3)+pi/2;
                dis2 = (obstacle(:,1) - position(:,1)).^2 + (obstacle(:,2) - position(:,2)).^2;
                minDis2 = min(dis2);
                if  minDis2 < obj.minDis
                    v = 0;
                    w = 0;
                    isContinue = -1;
                    return
                end
                
            else
                obstacle = [];
                position = vsl.poseWcsList(end,:);
                position(3) = position(3)+pi/2;
            end
            [v,w, isContinue] = RunA2B(obj, obstacle, position);
        end
    end
    
    methods (Static)
        
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