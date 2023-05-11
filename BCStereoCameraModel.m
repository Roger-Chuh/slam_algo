classdef BCStereoCameraModel < StereoCameraInterface
    
    methods
        function obj = BCStereoCameraModel(varargin)
            obj.monoCamModel1 = BCCameraModel;
            obj.monoCamModel2 = BCCameraModel;
            
            if (nargin == 1)
                if ischar(varargin{1})
                    paramFilePath = varargin{1};
                    LoadParam(obj, paramFilePath);
                else
                    error('The argument should be the parameter file path which has to be a string');
                end
            end
        end
        
        function CvtToRectifiedModel(obj)
            [rotVec, transVec] = GetExtrTransformVT(obj);
            [rotMat1ToCP, rotMat2ToCP, projMat1, projMat2] = OcvStereoRectify(Get(obj, 'IntrMat', 1), Get(obj, 'DistCoeff', 1), Get(obj, 'IntrMat', 2), Get(obj, 'DistCoeff', 2), obj.imgSize, rotVec, transVec);
            SetPinholeCamParam(obj.monoCamModel1, [projMat1(1,1); projMat1(2,2)], projMat1(1:2,3), projMat1(1,2));
            SetPinholeCamParam(obj.monoCamModel2, [projMat2(1,1); projMat2(2,2)], projMat2(1:2,3), projMat2(1,2));
            
            [mapX1, mapY1] = OcvInitUndistortRectifyMap(Get(obj, 'IntrMat', 1), Get(obj, 'DistCoeff', 1), rotMat1ToCP, Get(obj, 'PinholeIntrMat', 1), obj.imgSize);
            SetMap(obj.monoCamModel1, mapX1, mapY1);
            [mapX2, mapY2] = OcvInitUndistortRectifyMap(Get(obj, 'IntrMat', 2), Get(obj, 'DistCoeff', 2), rotMat2ToCP, Get(obj, 'PinholeIntrMat', 2), obj.imgSize);
            SetMap(obj.monoCamModel2, mapX2, mapY2);
        end
    end
    
end