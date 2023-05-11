classdef BodySensorLocalizer < BaseLocalizer
    
    properties
        prevTimeStamp;
        inputDecoder;        
        robotPosePlotHdl;
    end
    
    methods
        function obj = BodySensorLocalizer(bodySensorDataDecoder, cfgParam)
            obj@BaseLocalizer(cfgParam);
            
            obj.prevTimeStamp = [];
            
            obj.inputDecoder = bodySensorDataDecoder;
            
            obj.robotPosePlotHdl = [];
        end
        
        function motionState = DetermineMotionState(obj, frameData)
            startInd = 1 + max(0, Rows(frameData) - obj.configParam.win_length);
            meanV = mean(frameData(startInd : end, obj.inputDecoder.V));
            meanW = mean(frameData(startInd : end, obj.inputDecoder.W));

            if (abs(meanV) >= obj.configParam.still_threshold1 || abs(meanW) >= obj.configParam.still_threshold2)
                if (abs(meanV) > obj.configParam.motion_noise_level1 && abs(meanW) <= obj.configParam.motion_noise_level2)
                    motionState = obj.MOTIONSTATE_MOVE_STRAIGHT;
                elseif (abs(meanV) <= obj.configParam.motion_noise_level1 && abs(meanW) > obj.configParam.motion_noise_level2)
                    motionState = obj.MOTIONSTATE_ROTATE;
                elseif (abs(meanV) > obj.configParam.motion_noise_level1 && abs(meanW) > obj.configParam.motion_noise_level2)
                    motionState = obj.MOTIONSTATE_MOVE_GENERAL;
                else
                    motionState = GetPrevMotionState(obj);
                end
            else
                motionState = obj.MOTIONSTATE_STILL;
            end
            
        end
        
        function [deltaRotVec, deltaTransVec] = IntegrateFramePoseDelta(obj, frameData)
            if ~isempty(obj.prevTimeStamp)
                dt = diff([obj.prevTimeStamp; frameData(:, obj.inputDecoder.SYSTEMTIMESTAMP)])/1000; % ms -> s
            else
                dt = [0; diff(frameData(:, obj.inputDecoder.SYSTEMTIMESTAMP))]/1000;  % ms -> s
            end
            v = frameData(:, obj.inputDecoder.V);
            w = frameData(:, obj.inputDecoder.W);
            theta = cumsum([0;w.*dt]);
            xt = cumsum(-v.*sin(theta(1:end-1)).*dt);
            zt = cumsum(v.*cos(theta(1:end-1)).*dt);

            deltaRotVec = [0; theta(end); 0];
            deltaTransVec = [xt(end); 0; zt(end)];
            
            obj.prevTimeStamp = frameData(end, obj.inputDecoder.SYSTEMTIMESTAMP);
        end
        
        function IntegrateWcsPose(obj, deltaRotVec, deltaTransVec)
            prevTransVecWcs = GetTransVecWcs(obj, 'last');
            [prevRotAngWcs, prevRotMatWcs] = GetRotAngWcs(obj, 'last');
            
            rotAngWcs = prevRotAngWcs + deltaRotVec(2);
            transVecWcs = inv(prevRotMatWcs) * deltaTransVec + prevTransVecWcs;
            
            obj.poseWcsList = [obj.poseWcsList; [transVecWcs(1), transVecWcs(3), rotAngWcs]];
        end
        
        function Localize(obj, frameData, frameInd)
%             global PATH_FIG
            
            motionState = DetermineMotionState(obj, frameData);
            [deltaRotVec, deltaTransVec] = IntegrateFramePoseDelta(obj, frameData);
            IntegrateWcsPose(obj, deltaRotVec, deltaTransVec);
            
            obj.motionStateList = [obj.motionStateList; motionState];
            
%             if ~isempty(PATH_FIG)
%                 figure(PATH_FIG), UpdateTrajectoryPlot(obj);
%                 figure(PATH_FIG), title(sprintf('Frame %d. BS: %s', frameInd, strrep(GetMotionStateStr(obj), '_', '\_'))), axis 'equal';
%             end          
            
            
        end
        
        
        function UpdateTrajectoryPlot(obj)
            hold on
            if Rows(obj.poseWcsList) < 2
                plot(obj.poseWcsList(end,obj.POSEWCSLIST_TRANSVEC_X), obj.poseWcsList(end,obj.POSEWCSLIST_TRANSVEC_Z),'+r');
            else
                line([obj.poseWcsList(end,obj.POSEWCSLIST_TRANSVEC_X),obj.poseWcsList(end-1,obj.POSEWCSLIST_TRANSVEC_X)],...
                    [obj.poseWcsList(end,obj.POSEWCSLIST_TRANSVEC_Z),obj.poseWcsList(end-1,obj.POSEWCSLIST_TRANSVEC_Z)],'Color',[1 0 0],'LineWidth',1)
                plot(obj.poseWcsList(end,obj.POSEWCSLIST_TRANSVEC_X), obj.poseWcsList(end,obj.POSEWCSLIST_TRANSVEC_Z),'+r');
            end
            hold off
        end
        
        function PlotRobotPose(obj)
            if ~isempty(obj.robotPosePlotHdl)
                delete(obj.robotPosePlotHdl);
            end 
            rotAngWcs = GetRotAngWcs(obj, 'last');
            orientVec = 0.1*[-sin(rotAngWcs), cos(rotAngWcs)];
            transVecWcs = GetTransVecWcs(obj, 'last');
            obj.robotPosePlotHdl = plot(transVecWcs(1), transVecWcs(3), 'bo', ...
                                        transVecWcs(1) + [0; orientVec(1)], transVecWcs(3) + [0; orientVec(2)], 'r-', ...
                                        transVecWcs(1) + orientVec(1) + [-orientVec(2), orientVec(2)]/2, transVecWcs(3) + orientVec(2) + [orientVec(1), -orientVec(1)]/2, 'r-');
        end
        
    end
    
    methods
        % Implementation of abstract methods in Configurable base class
        function SetDefaultValue(obj, cfgParam)
            obj.configParam = cfgParam;
        end
        
        function CheckConfig(obj) %#ok<MANU>
        end
    end
    
end