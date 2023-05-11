classdef StereoCameraInterface < handle
    properties
        monoCamModel1
        monoCamModel2
        
        imgSize;
        swapFlag;
        
        rotVec1To2;
        transVec1To2;
        
        inputDevice
    end
    
    methods (Abstract)
        CvtToRectifiedModel(obj);
    end
    
    methods
        function varargout = Get(obj, whatToGet, varargin)
            if (nargout == 1)
                if (nargin <= 2)
                    error('Which camera is not specified.');
                elseif (nargin == 3)
                    camInd = varargin{1};
                    scaleLevel = 0;
                elseif (nargin == 4)
                    camInd = varargin{1};
                    scaleLevel = varargin{2};
                else
                    error('Too many input arguments');
                end
                if (camInd == 1)
                    camModel = obj.monoCamModel1;
                elseif (camInd == 2)
                    camModel = obj.monoCamModel2;
                else
                    error('camInd can only be 1 or 2. Given value is %d', camInd);
                end
                
                if any(strcmp(whatToGet, {'IntrMat', 'FocLen', 'FocLenVec', 'PrincpPt', 'DistCoeff', 'IntrParam'}))
                    getFuncHdl = str2func(['Get', whatToGet]);
                    varargout{1} = getFuncHdl(camModel);
                elseif any(strcmp(whatToGet, strcat('Pinhole', {'IntrMat', 'FocLen', 'FocLenVec', 'PrincpPt', 'DistCoeff', 'IntrParam'})))
                    getFuncHdl = str2func(['Get', whatToGet]);
                    varargout{1} = getFuncHdl(camModel, scaleLevel);
                else
                    error('Unrecognized property to get: %s', whatToGet);
                end
            elseif (nargout == 2)
                if (nargin == 2)
                    scaleLevel = 0;
                elseif (nargin == 3)
                    scaleLevel = varargin{1};
                else
                    error('Too many input arguments');
                end
                
                if any(strcmp(whatToGet, {'IntrMat', 'FocLen', 'FocLenVec', 'PrincpPt', 'DistCoeff', 'IntrParam'}))
                    getFuncHdl = str2func(['Get', whatToGet]);
                    varargout{1} = getFuncHdl(obj.monoCamModel1);
                    varargout{2} = getFuncHdl(obj.monoCamModel2);
                elseif any(strcmp(whatToGet, strcat('Pinhole', {'IntrMat', 'FocLen', 'FocLenVec', 'PrincpPt', 'DistCoeff', 'IntrParam'})))
                    getFuncHdl = str2func(['Get', whatToGet]);
                    varargout{1} = getFuncHdl(obj.monoCamModel1, scaleLevel);
                    varargout{2} = getFuncHdl(obj.monoCamModel2, scaleLevel);
                else
                    error('Unrecognized property to get: %s', whatToGet);
                end
            else
                error('Too many output arguments.');
            end
        end
        
        function rotMat1To2 = GetExtrRotMat(obj)
            rotMat1To2 = rodrigues(obj.rotVec1To2);
        end
        
        function rotVec1To2 = GetExtrRotVec(obj)
            rotVec1To2 = obj.rotVec1To2;
        end
        
        function transVec1To2 = GetExtrTransVec(obj)
            transVec1To2 = obj.transVec1To2;
        end
        
        function baseline = GetBaseline(obj)
            baseline = norm(GetExtrTransVec(obj));
        end
        
        function [rotVec1To2, transVec1To2] = GetExtrTransformVT(obj)
            rotVec1To2 = GetExtrRotVec(obj);
            transVec1To2 = GetExtrTransVec(obj);
        end
        
        function [rotMat1To2, transVec1To2] = GetExtrTransformRT(obj)
            rotMat1To2 = GetExtrRotMat(obj);
            transVec1To2 = GetExtrTransVec(obj);
        end
        
        function LoadParam(obj, paramFilePath)
            [paramFid, errMsg] = fopen(paramFilePath);
            if (paramFid < 0)
                error('Cannot open %s for read: %s', paramFilePath, errMsg);
            end
            
            params = fscanf(paramFid, '%d %d %d', 3);
            obj.imgSize = params(2:-1:1);
            obj.swapFlag = params(3);
            
            paramFid = LoadParam(obj.monoCamModel1, paramFid);
            paramFid = LoadParam(obj.monoCamModel2, paramFid);
           
            params = fscanf(paramFid, '%f %f %f %f %f %f', 6);
            obj.rotVec1To2 = params(1:3);
            obj.transVec1To2 = params(4:6);
            
            fclose(paramFid);
        end
        
        function SetInputDevice(obj, inputDevice)
            assert(any(strcmp('CameraInputInterface', superclasses(inputDevice))), 'The input object must be an object of class derived from CameraInputDevice');
            obj.inputDevice = inputDevice;
        end
        
        function OpenInput(obj, deviceInfo)
            assert(~isempty(obj.inputDevice), 'Not specified input device');
            Open(obj.inputDevice, deviceInfo);
        end
        
        function CloseInput(obj)
            if ~isempty(obj.inputDevice)
                Close(obj.inputDevice);
            end
        end
        
        function b = HasFrame(obj)
            assert(~isempty(obj.inputDevice), 'Not specified input device');
            b = HasFrame(obj.inputDevice);
        end
        
        function [img1, img2, timestamp, frameInd] = GetNextStereoImagePair(obj, needRemap)
            if ~exist('needRemap', 'var')
                needRemap = false;
            end
            
            [frame, timestamp, frameInd] = GetNextFrame(obj.inputDevice);
            if ~obj.swapFlag
                img1 = frame(:, 1:obj.imgSize(2),     :);
                img2 = frame(:, obj.imgSize(2)+1:end, :);
            else
                img1 = frame(:, obj.imgSize(2)+1:end, :);
                img2 = frame(:, 1:obj.imgSize(2),     :);
            end
            
            if needRemap
                img1 = Remap(obj.monoCamModel1, img1);
                img2 = Remap(obj.monoCamModel2, img2);
            end
        end
    end
end