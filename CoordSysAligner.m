classdef CoordSysAligner < handle
    
    properties
        pctBody2Cam;
    end
    
    methods
        function Load(obj, paramFilePath) %#ok<INUSD>
            coordSysAlignParam = ReadCoordSysAlignDataDump;
            obj.pctBody2Cam = [PointCoordTransformer(coordSysAlignParam(1).rotMatB2C, coordSysAlignParam(1).transVecB2C); ...
                               PointCoordTransformer(coordSysAlignParam(2).rotMatB2C, coordSysAlignParam(2).transVecB2C)];
        end
        
        function pctBody2Cam = GetPctBody2Cam(obj, camInd)
            if ~exist('camInd', 'var')
                camInd = 1;
            end
            assert(camInd == 1 || camInd == 2, 'camInd exceeds the number of cameras. The max is 2');
            pctBody2Cam = obj.pctBody2Cam(camInd);
        end
        
        function pctCcs = BodyRotatePctCcs(obj, bodyRotAng, camInd)
            if ~exist('camInd', 'var')
                camInd = 1;
            end
            assert(camInd == 1 || camInd == 2, 'camInd exceeds the number of cameras. The max is 2');
            pctBcs = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(bodyRotAng), zeros(3,1));
            pctCcs = obj.pctBody2Cam(camInd) * pctBcs / obj.pctBody2Cam(camInd);
        end
        
        function rotMatCcs = BodyRotateRotMatCcs(obj, bodyRotAng, camInd)
            if ~exist('camInd', 'var')
                camInd = 1;
            end
            assert(camInd == 1 || camInd == 2, 'camInd exceeds the number of cameras. The max is 2');
            pctBcs = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(bodyRotAng), zeros(3,1));
            rotMatCcs = R(obj.pctBody2Cam(camInd)) * R(pctBcs) * R(obj.pctBody2Cam(camInd))';
        end
        
        function pctCcs = BodyMoveCcs(obj, pctBcs, camInd)
            if ~exist('camInd', 'var')
                camInd = 1;
            end
            assert(camInd == 1 || camInd == 2, 'camInd exceeds the number of cameras. The max is 2');
            pctCcs = obj.pctBody2Cam(camInd) * pctBcs / obj.pctBody2Cam(camInd);
        end
        
        function pctCcs = PctBcs2Ccs(obj, pctBcs, camInd)
            if ~exist('camInd', 'var')
                camInd = 1;
            end
            assert(camInd == 1 || camInd == 2, 'camInd exceeds the number of cameras. The max is 2');
            pctCcs = obj.pctBody2Cam(camInd) * pctBcs;
        end
        
        function pctBcs = PctCcs2Bcs(obj, pctCcs)
            if ~exist('camInd', 'var')
                camInd = 1;
            end
            assert(camInd == 1 || camInd == 2, 'camInd exceeds the number of cameras. The max is 2');
            pctBcs = obj.pctBody2Cam(camInd) \ pctCcs;
        end
    end
    
end