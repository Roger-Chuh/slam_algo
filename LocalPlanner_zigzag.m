classdef LocalPlanner_zigzag < Configurable
    properties
        
        STILL_FLAG;
        stillFrameNum;
        motionState;

        maxStillFrameNum = 0;  % 3
        v = 0.4;% unit: m/s
        w = 0.4;% unit: rad/s
        wmin = 0.01;
        minBody2Wall = 0.5; % unit: m
        PID_scale = 0.4;
        obstacleAvoidance;
        direction;
        lineEndPt;
        lineStartPt;
        linePt;
        
        marginL2L = 0.4;
        positionThreshold;
        angleThreshold;
        rotationDirection;
        MOVE_STRAIGHT = 1;
        OBSTACLE_AVOIDANCE = 2;
        PURE_ROTATION = 3;
    end
    
    % constructor
    methods
        function  obj = LocalPlanner_zigzag(cfgParam)
            obj@Configurable(cfgParam);
            obj.STILL_FLAG = false;
            obj.stillFrameNum = 0;
            obj.motionState = obj.MOVE_STRAIGHT;
            obj.angleThreshold = obj.wmin*0.1*1.1; % unit: rad            
            obj.positionThreshold = obj.v*0.1*2; % unit: m
            obj.obstacleAvoidance = LocalPlanner2(cfgParam);
            
            obj.direction = pi/2;
            obj.lineEndPt = [0,0];
            obj.lineStartPt = [0,0];
            obj.linePt = [0, 0];
            obj.rotationDirection = (-1)^obj.obstacleAvoidance.CLOCKWISE*pi/2;
        end
        
        function SetLocalPlanner(obj, position, clockwise)
            SetClockwiseForObstacleAvoidance(obj.obstacleAvoidance, clockwise);
            obj.direction = position(3);
            obj.lineEndPt = position(1:2);
            obj.lineStartPt = position(1:2);
            obj.linePt = position(1:2);
            obj.rotationDirection = (-1)^obj.obstacleAvoidance.CLOCKWISE*pi/2;
            obj.STILL_FLAG = false;
            obj.stillFrameNum = 0;
            obj.motionState = obj.MOVE_STRAIGHT;
            obj.obstacleAvoidance.nextRoadMap = [];
            obj.obstacleAvoidance.plannerState = obj.obstacleAvoidance.FIND_ROAD_MAP;
        end
    end
    
    % main function
    methods
        function [v,w, IsContinue] = RunZigzag(obj, obstacle, position, IsStop)
            IsContinue = true;
            if IsStop
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
                    switch obj.motionState
                        case  obj.MOVE_STRAIGHT
                            if IsMoveStraight(obj, obstacle, position)
                                [v,w] = ExecuteMoveStraight(obj, obstacle, position);                                
                                return;
                            else
                                v = 0;
                                w = 0;
                                obj.lineEndPt = position(1:2);
                                obj.motionState = obj.OBSTACLE_AVOIDANCE;
                                obj.STILL_FLAG = true;
                                return;
                            end
                        case  obj.OBSTACLE_AVOIDANCE
                            if IsObstacleAvoidance(obj,obstacle, position)
                                if norm(position(1:2) - obj.lineStartPt) < obj.marginL2L
                                    v = 0;
                                    w = 0;
                                    IsContinue = false;
                                else
                                    [v,w] = ExecuteObstacleAvoidance(obj, obstacle, position);
                                end
                                return;
                            else
                                v = 0;
                                w = 0;
                                obj.motionState = obj.PURE_ROTATION;
                                obj.linePt = position(1:2);
                                obj.STILL_FLAG = true;
                                obj.obstacleAvoidance.nextRoadMap = [];
                                obj.obstacleAvoidance.plannerState = obj.obstacleAvoidance.FIND_ROAD_MAP;
                                return;
                            end
                        case obj.PURE_ROTATION
                            if IsRotation(obj,obstacle, position)
                                [v,w] = ExecutePureRotation(obj, position);
                                return;
                            else
                                v = 0;
                                w = 0;
                                obj.motionState = obj.MOVE_STRAIGHT;
                                obj.obstacleAvoidance.CLOCKWISE = ~obj.obstacleAvoidance.CLOCKWISE;
                                obj.lineStartPt = position(1:2);
                                obj.STILL_FLAG = true;                                
                                return;
                            end                             
                        otherwise
                            assert(false, 'localPlanner_zigzag: no such motion state')
                    end
                end
            else
                v = 0;
                w = 0;
            end
        end       
    end
    
    % member function
    methods 
        function flag = IsMoveStraight(obj, obstacle, position)
            if ~isempty(obstacle)
                dis2 = (obstacle(:,1) - position(1)).^2 + (obstacle(:,2) - position(2)).^2;
                n2 = [obstacle(:,1)-position(1), obstacle(:,2) - position(2)];
                n1 = [cos(position(3)),sin(position(3))];
                theta = acos((n2(:,1)*n1(1) + n2(:,2)*n1(2))/norm(n1)./sqrt(n2(:,1).^2+n2(:,2).^2));
                validPt = dis2 < obj.minBody2Wall^2 & theta < 0.4;
            else
                validPt = 0;
            end
            if sum(validPt) == 0
                flag = true;
            else
                flag = false;
            end
        end
        
        function flag = IsObstacleAvoidance(obj, obstacle, position)
            theta = obj.direction;
            alpha = obj.direction + obj.rotationDirection;
            nx = cos(alpha)*obj.marginL2L;
            ny = sin(alpha)*obj.marginL2L;
            x0 = obj.lineEndPt(1);
            y0 = obj.lineEndPt(2);
            A = sin(theta);
            B = - cos(theta);
            C = -x0*sin(theta) + y0*cos(theta);
            D = -A*nx - B*ny + C;
            dis = abs(A*position(1) + B*position(2) + D)/sqrt(A*A+B*B);
            if dis > obj.positionThreshold
                flag = true;
            else
                flag = false;
            end
        end
        
        function flag = IsRotation(obj, obstacle, position)
            if obj.rotationDirection > 0
                if obj.obstacleAvoidance.CLOCKWISE
                    alpha = obj.direction;
                else
                    alpha = obj.direction - pi;
                end
            else
                if ~obj.obstacleAvoidance.CLOCKWISE
                    alpha = obj.direction;
                else
                    alpha = obj.direction - pi;
                end
            end
            belta = position(3);
            theta = acos(cos(alpha)*cos(belta)+sin(alpha)*sin(belta));
            if theta > obj.angleThreshold
                flag = true;
            else
                flag = false;
            end
        end
        
        function [v,w] = ExecuteMoveStraight(obj, obstacle, position)
%             v = obj.v;
%             w = 0;
            T = obj.linePt';
            if obj.rotationDirection > 0
                if obj.obstacleAvoidance.CLOCKWISE
                    alpha = obj.direction - pi/2;
                else
                    alpha = obj.direction - pi - pi/2;
                end
            else
                if ~obj.obstacleAvoidance.CLOCKWISE
                    alpha = obj.direction - pi/2;
                else
                    alpha = obj.direction - pi - pi/2;
                end
            end
            
            R = [cos(alpha), sin(alpha); -sin(alpha), cos(alpha)];
            c2wP = [R',T; 0, 0, 1];
            ptInLaneDir = c2wP\ [position(1);position(2);1]; 
            
            if ptInLaneDir(1) > 0 
                v = obj.v;
                w = -obj.PID_scale*ptInLaneDir(1);
            else
                v = obj.v;
                w = -obj.PID_scale*ptInLaneDir(1);
            end
        end
        
        function [v,w] = ExecuteObstacleAvoidance(obj, obstacle, position)
            [v,w] = ObstacleAvoidance(obj.obstacleAvoidance, obstacle, position);
        end
        
        function [v,w] = ExecutePureRotation(obj, position)
            if obj.rotationDirection > 0
                if obj.obstacleAvoidance.CLOCKWISE
                    alpha = obj.direction;
                else
                    alpha = obj.direction - pi;
                end
            else
                if ~obj.obstacleAvoidance.CLOCKWISE
                    alpha = obj.direction;
                else
                    alpha = obj.direction - pi;
                end
            end
            belta = position(3);
            theta = acos(cos(alpha)*cos(belta)+sin(alpha)*sin(belta));
            if rad2deg(theta) > 5
                v = 0;
                w = (-1)^obj.obstacleAvoidance.CLOCKWISE*obj.w;
            else
                v = 0;
                w = (-1)^obj.obstacleAvoidance.CLOCKWISE*obj.wmin;
            end
        end
    end
    
    %interfaces
    methods
        function [v,w, IsContinue] = PlanZigzagRoout(obj, vsl, IsStop)
            if ~isempty( vsl.pointCloudManager.map2D)
                obstacle = vsl.pointCloudManager.map2D;
                position = vsl.poseWcsList(end,:);
                position(3) = position(3)+pi/2;
            else
                obstacle = [];
                position = vsl.poseWcsList(end,:);
                position(3) = position(3)+pi/2;
            end
            [v,w, IsContinue] = RunZigzag(obj, obstacle, position, IsStop);
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