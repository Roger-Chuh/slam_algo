classdef BaseLocalizer < Configurable
    properties (Constant)
        POSEWCSLIST_TRANSVEC_X = 1;
        POSEWCSLIST_TRANSVEC_Z = 2;
        POSEWCSLIST_ROTANGLE = 3;
        
        MOTIONSTATE_UNKNOWN = uint8(0);
        MOTIONSTATE_STILL = uint8(1);
        MOTIONSTATE_MOVE_STRAIGHT = uint8(2);
        MOTIONSTATE_ROTATE = uint8(3);
        MOTIONSTATE_MOVE_GENERAL = uint8(4);
    end
    
    properties
        poseWcsList;
        motionStateList;
        
        motionState2Str;
    end
    
    methods (Abstract)
        Localize(obj);
    end
    
    methods
        function obj = BaseLocalizer(cfgParam)
            obj@Configurable(cfgParam);
            
            obj.poseWcsList = [];
            obj.motionStateList = [];
            
            obj.motionState2Str = containers.Map({obj.MOTIONSTATE_UNKNOWN, ...
                                                  obj.MOTIONSTATE_STILL, ...
                                                  obj.MOTIONSTATE_MOVE_STRAIGHT, ...
                                                  obj.MOTIONSTATE_ROTATE, ...
                                                  obj.MOTIONSTATE_MOVE_GENERAL}, ... 
                                                 {'UNKNOWN', 'STILL', 'MOVE_STRAIGHT', 'ROTATE', 'MOVE_GENERAL'});
        end
        
        function poseWcs = GetPoseWcs(obj, frameInd, outputType)
            % Support negtative frameInd. 0 means the last element.
            
            if ~exist('outputType', 'var')
                outputType = 'Raw';
            end
            
            if strcmp(frameInd, 'last')
                frameInd = NumFrames(obj);
            end
            
            if (frameInd > NumFrames(obj) || (~IsEmpty(obj) && frameInd < -NumFrames(obj) + 1))
                error('Specified frame index exceeds the number of frames recorded');
            end
            
            if (frameInd <= 0)
                frameInd = NumFrames(obj) + frameInd;
            end
            
            if strcmp(outputType, 'Raw')
                if ~IsEmpty(obj)
                    poseWcs = obj.poseWcsList(frameInd, :);
                else
                    poseWcs = [0,0,0];
                end
            elseif strcmp(outputType, 'PointCoordTransformer')
                % poseWcs is the transformation converting a point to WCS
                if ~IsEmpty(obj)
                    rotMatToWcs = BaseLocalizer.RotMatFromAngle(obj.poseWcsList(frameInd, obj.POSEWCSLIST_ROTANGLE))';
                    transVecToWcs = [obj.poseWcsList(frameInd, obj.POSEWCSLIST_TRANSVEC_X); 0; obj.poseWcsList(frameInd, obj.POSEWCSLIST_TRANSVEC_Z)];
                    poseWcs = PointCoordTransformer(rotMatToWcs, transVecToWcs);
                else
                    poseWcs = eye(4);
                end
            else
                error('Unsupported outputType %s', outputType);
            end
        end
        
        function transVecWcs = GetTransVecWcs(obj, frameInd)
            poseWcs = GetPoseWcs(obj, frameInd, 'Raw');
            transVecWcs = [poseWcs(obj.POSEWCSLIST_TRANSVEC_X); 0; poseWcs(obj.POSEWCSLIST_TRANSVEC_Z)];
        end
        
        function [rotAngWcs, varargout] = GetRotAngWcs(obj, frameInd)
            poseWcs = GetPoseWcs(obj, frameInd, 'Raw');
            rotAngWcs = poseWcs(obj.POSEWCSLIST_ROTANGLE);
            if (nargout == 2)
                if isempty(obj.poseWcsList)
                    varargout{1} = eye(3);
                else
                    varargout{1} = BaseLocalizer.RotMatFromAngle(rotAngWcs);
                end
            elseif (nargout > 2)
                error('Too many output arguments');
            end
        end
        
        function varargout = GetDeltaPose(obj, frameInd1, frameInd2, outputType)
            if (~exist('outputType', 'var'))
                outputType = 'Raw';
            end
            if strcmp(outputType, 'Raw')
                poseWcs1 = GetPoseWcs(obj, frameInd1, outputType);
                poseWcs2 = GetPoseWcs(obj, frameInd2, outputType);
                assert(nargout == 1, 'Incorrect output parameter number');
                % deltaPose in WCS
                varargout{1} = poseWcs2 - poseWcs1;
            elseif strcmp(outputType, 'PointCoordTransformer')
                poseWcs1 = GetPoseWcs(obj, frameInd1, outputType);
                poseWcs2 = GetPoseWcs(obj, frameInd2, outputType);
                assert(nargout == 1, 'Incorrect output parameter number');
                % Transformation converting a point from BCS1 to BCS2
                varargout{1} = Inv(poseWcs2)*poseWcs1;
            elseif strcmp(outputType, 'AngleT0')
                poseWcs1 = GetPoseWcs(obj, frameInd1, 'Raw');
                poseWcs2 = GetPoseWcs(obj, frameInd2, 'Raw');
                assert(nargout == 2, 'Incorrect output parameter number');
                % rotation angle of robot and translation vector at the
                % frame of frameInd1
                deltaPoseWcs = poseWcs2 - poseWcs1;
                deltaAng = deltaPoseWcs(obj.POSEWCSLIST_ROTANGLE);
                deltaTransVecWcs = [deltaPoseWcs(obj.POSEWCSLIST_TRANSVEC_X); 0; deltaPoseWcs(obj.POSEWCSLIST_TRANSVEC_Z)];
                varargout{1} = deltaAng;
                
                P1 = WcsPoseVecToPct(obj, poseWcs1);
                Xw = [poseWcs2(1);0;poseWcs2(2)];
                varargout{2} = Inv(P1)*Xw;
%                 varargout{2} = BaseLocalizer.RotateVector(deltaAng, deltaTransVecWcs);
%                 varargout{2} = BaseLocalizer.RotateVector(poseWcs1(3), deltaTransVecWcs);
            else
                error('Unsupported outputType %s', outputType);
            end
        end
        
        function poseWcs = GetPrevPoseWcs(obj, outputType)
            if ~exist('outputType', 'var')
                outputType = 'Raw';
            end
            poseWcs = GetPoseWcs(obj, 'last', outputType);
        end
        
        function motionState = GetMotionState(obj, frameInd)
            if strcmp(frameInd, 'last')
                frameInd = NumFrames(obj);
            end
            
            if isempty(obj.motionStateList)
                motionState = obj.MOTIONSTATE_UNKNOWN;
            else
                if (frameInd > NumFrames(obj))
                    error('Specified frame index exceeds the number of frames recorded');
                end
                motionState = obj.motionStateList(frameInd);
            end
        end
        
        function motionState = GetPrevMotionState(obj)
            motionState = GetMotionState(obj, 'last');
        end
        
        function msStr = GetMotionStateStr(obj, frameInd)
            if ~exist('frameInd', 'var')
                frameInd = NumFrames(obj);
            end
            
            motionState = GetMotionState(obj, frameInd);
            msStr = obj.motionState2Str(motionState);
        end
        
        function b = IsEmpty(obj)
            assert(Rows(obj.poseWcsList) == length(obj.motionStateList), 'Inconsistent internal data');
            b = isempty(obj.poseWcsList);
        end
        
        function SetMotion(obj, motionState, newPoseWcs)
            obj.motionStateList = [obj.motionStateList; motionState];
            obj.poseWcsList = [obj.poseWcsList; newPoseWcs];
        end
        
        function n = NumFrames(obj)
            assert(Rows(obj.poseWcsList) == length(obj.motionStateList), 'Inconsistent internal data');
            n = length(obj.motionStateList);
        end
        
        function poseWcs = WcsPoseVecToPct(obj, poseVecWcs)
            assert(isrow(poseVecWcs) && length(poseVecWcs) == 3, 'Wrong poseVecWcs format. It must be a 1 by 3 vector');
            rotMatToWcs = BaseLocalizer.RotMatFromAngle(poseVecWcs(obj.POSEWCSLIST_ROTANGLE))';
            transVecToWcs = [poseVecWcs(obj.POSEWCSLIST_TRANSVEC_X); 0; poseVecWcs(obj.POSEWCSLIST_TRANSVEC_Z)];
            poseWcs = PointCoordTransformer(rotMatToWcs, transVecToWcs);
        end
    end
    
    methods (Static)
        function rotMat = RotMatFromAngle(theta)
            % theta is the angle that the robot body rotates
            % rotMat transforms a point from pre-rotation coordinates to
            % rotated coordinates
            rotAxis = [-0.000000000829335   0.999999999262215  -0.000038413313970];
%             rotAxis = [-0.000000000821355   0.999999999262213  -0.000038413351247];
            rotAxis = [ 0.000000009866276   0.999999999984453   0.000005589195821];
            rotAxis = [-0.000008213751383   0.999962479988871   0.008389491816959];
            rotAxis = [0.000000009724004   0.999999999834510   0.000018192843212];
            if 1
                rotMat = [cos(theta),  0,  sin(theta)
                    0,           1,          0
                    -sin(theta), 0,  cos(theta)];
            else
                rotMat = rodrigues( theta *rotAxis);
                
            end
        end
        
        function vecOut = RotateVector(theta, vecIn)
            assert(Rows(vecIn) == 3, 'Input vectors must in 3 by n format');
            vecOut = BaseLocalizer.RotMatFromAngle(theta) * vecIn;
        end
        
    end
    
end