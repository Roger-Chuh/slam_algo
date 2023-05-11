classdef RotateAnglePredictor < Configurable
    properties
        optimMethod;
    end
    
    methods
        
        % Constructor
        function obj = RotateAnglePredictor(cfgParam)
            obj@Configurable(cfgParam);
        end
        
        function predRotAng = PredictRotateAngle(obj, pt1Ics, pt2Ics, intrMat, coordSysAligner, camInd, refRotAng, rotAngRng)
            assert(Rows(pt1Ics) == Rows(pt2Ics), 'ptIcs1 and ptIcs2 must contain the same number of points');
            [predRotAng, optimEigVec, optimVal, exitFlag] = MinimizeSmallestEigenVal(obj, pt1Ics, pt2Ics, intrMat, coordSysAligner, camInd, refRotAng, rotAngRng);
            if 1
                theta = rotAngRng(1):deg2rad(0.01):rotAngRng(2);
                val = [];
                for i = 1:length(theta)
                    [val(i), optimEigVec] = RotateAnglePredictor.SmallestEigenValue(theta(i), pt1Ics, pt2Ics, intrMat, coordSysAligner, camInd);
                end
                [~,minID] = min(val);
                predRotAng = theta(minID);
                figure(913); plot(theta,val,'-b'), hold on; plot(theta(minID),val(minID),'ro'), hold off
            end
        end
        function ang = PredictRotateAngle2(obj, pt1Ics, pt2Ics, intrMat, coordSysAligner, camInd, refRotAng, rotAngRng,stamp,goldenPose)
            assert(Rows(pt1Ics) == Rows(pt2Ics), 'ptIcs1 and ptIcs2 must contain the same number of points');
            
            id = find(goldenPose(:,1) >= stamp(1) & goldenPose(:,1) <= stamp(2));
            b2c = coordSysAligner.pctBody2Cam(1, 1).transformMat;
            if abs(goldenPose(id(1),1)-stamp(1)) < abs(goldenPose(id(1)-1,1)-stamp(1))
                if abs(goldenPose(id(end),1)-stamp(2)) < abs(goldenPose(end,1)-stamp(2))
                    deltRGoldenBody = reshape(goldenPose(id(end),2:10),3,3)'*reshape(goldenPose(id(1),2:10),3,3);
                else
                    deltRGoldenBody = reshape(goldenPose((end),2:10),3,3)'*reshape(goldenPose(id(1),2:10),3,3);
                end
            else
                if abs(goldenPose(id(end),1)-stamp(2)) < abs(goldenPose(end,1)-stamp(2))
                    deltRGoldenBody = reshape(goldenPose(id(end),2:10),3,3)'*reshape(goldenPose(id(1)-1,2:10),3,3);
                else
                    deltRGoldenBody = reshape(goldenPose((end),2:10),3,3)'*reshape(goldenPose(id(1)-1,2:10),3,3);
                end
            end
            deltRGoldenCam = b2c(1:3,1:3)*deltRGoldenBody*b2c(1:3,1:3)';
            rotAxis = rodrigues(deltRGoldenBody)./norm(rodrigues(deltRGoldenBody));
            rotAxis = sign(rotAxis(2)).*rotAxis;
            ang0 = norm(rodrigues(deltRGoldenBody));
            
%             rotMatIcs = deltRGoldenCam / intrMat;  %BodyRotateRotMatCcs(coordSysAligner, rotAng, camInd) / intrMat;
%             pt1HomoCcsRot = rotMatIcs * HomoCoord(pt1Ics',1);
%             pt2HomoCcs = intrMat \ HomoCoord(pt2Ics',1);
% 
%             planeNorm = cross(Normalize(pt1HomoCcsRot), Normalize(pt2HomoCcs));
%             varMat = planeNorm * planeNorm';
%             [eigVec, eigVal] = eig(varMat);
%             eigVal = diag(eigVal)';
%             smallestEigVal = eigVal(1);
%             smallestEigVec = eigVec(:,1);
            
            
            
%             [predRotAng, optimEigVec, optimVal, exitFlag] = MinimizeSmallestEigenVal(obj, pt1Ics, pt2Ics, intrMat, coordSysAligner, camInd, refRotAng, rotAngRng);
%             [predRotAng2, optimEigVec2, optimVal2, exitFlag2] = MinimizeSmallestEigenVal2(obj, pt1Ics, pt2Ics, intrMat, coordSysAligner, camInd, refRotAng, rotAngRng,rotAxis);
            if 1
                theta = rotAngRng(1):0.0001:rotAngRng(2);
                val = [];
                for i = 1:length(theta)
                    [val(i), optimEigVec] = RotateAnglePredictor.SmallestEigenValue2(theta(i), pt1Ics, pt2Ics, intrMat, coordSysAligner, camInd,rotAxis);
                end
                [~,minID] = min(val);
                ang = theta(minID);
%                  [minv,mini1]=findpeaks(val,'minpeakdistance',100);
%                  [minv,mini1]=findpeaks(val,'NPeaks',1,'minpeakdistance',100);
                try 
                    [minv,mini1]=findpeaks(val,'SortStr','descend');
                    figure(914),clf; plot(theta,val,'-b'), hold on;plot(theta(minID),val(minID),'ro');plot(theta(mini1(1)),val(mini1(1)),'rs');title(sprintf('%06f  ------  %06f',theta(minID),theta(mini1(1))));
                catch
                    mini1 = 1;
                end
               
                
            end
            if ang < 0
                ang0 = -ang0;
            end
            
            if 0
                figure(19),hold on;plot(stamp(2),ang0-ang,'*r');
            end
            
        end
        
        function [optimAng, optimEigVec, optimVal, exitFlag] = MinimizeSmallestEigenVal(obj, pt1Ics, pt2Ics, intrMat, coordSysAligner, camInd, refRotAng, rotAngRng)
            switch obj.optimMethod
                case 'fminbnd'
                    [optimAng, optimEigVec, optimVal, exitFlag] = MinimizeFminbnd(obj, pt1Ics, pt2Ics, intrMat, coordSysAligner, camInd, rotAngRng);
                case 'exhaustion'
                    
                otherwise
                    error('Unsupported minimization method %s', cfgParam.MinimizeMethod);
            end
        end
         function [optimAng, optimEigVec, optimVal, exitFlag] = MinimizeSmallestEigenVal2(obj, pt1Ics, pt2Ics, intrMat, coordSysAligner, camInd, refRotAng, rotAngRng,rotAxis)
            switch obj.optimMethod
                case 'fminbnd'
                    [optimAng, optimEigVec, optimVal, exitFlag] = MinimizeFminbnd2(obj, pt1Ics, pt2Ics, intrMat, coordSysAligner, camInd,rotAxis, rotAngRng);
                case 'exhaustion'
                    
                otherwise
                    error('Unsupported minimization method %s', cfgParam.MinimizeMethod);
            end
        end
        
        function [optimAng, optimEigVec, optimVal, exitFlag] = MinimizeFminbnd(obj, pt1Ics, pt2Ics, intrMat, coordSysAligner, camInd, varargin)
            if (nargin == 6)
                rotAngRng = [-pi/2, pi/2];
            elseif (nargin == 7)
                rotAngRng = varargin{1};
                if isempty(rotAngRng)
                    rotAngRng = [-pi/2, pi/2];
                end
                assert(isvector(rotAngRng) && numel(rotAngRng) == 2, 'rotAngRng must be a 2-vector.');
            else
                error('Too many input arguments');
            end
            
            assert(rotAngRng(1) < rotAngRng(2), 'Invalid rotAngRng. A valid one should satisfy rotAngRng(1) < rotAngRng(2)');
            
            [optimAng, ~, exitFlag] = fminbnd(@ObjectiveFunction, rotAngRng(1), rotAngRng(2), optimset('TolX',obj.configParam.oneof_optim_method_param.fminbnd_param.tolerance_var,'Display',obj.configParam.oneof_optim_method_param.fminbnd_param.display));
            [optimVal, optimEigVec] = RotateAnglePredictor.SmallestEigenValue(optimAng, pt1Ics, pt2Ics, intrMat, coordSysAligner, camInd);
            
            
            
            function smallestEigVal = ObjectiveFunction(rotAng)
                smallestEigVal = RotateAnglePredictor.SmallestEigenValue(rotAng, pt1Ics, pt2Ics, intrMat, coordSysAligner, camInd);
            end
        end
        function [optimAng, optimEigVec, optimVal, exitFlag] = MinimizeFminbnd2(obj, pt1Ics, pt2Ics, intrMat, coordSysAligner, camInd,rotAxis, varargin)
            if (nargin == 7)
                rotAngRng = [-pi/2, pi/2];
            elseif (nargin == 8)
                rotAngRng = varargin{1};
                if isempty(rotAngRng)
                    rotAngRng = [-pi/2, pi/2];
                end
                assert(isvector(rotAngRng) && numel(rotAngRng) == 2, 'rotAngRng must be a 2-vector.');
            else
                error('Too many input arguments');
            end
            
            assert(rotAngRng(1) < rotAngRng(2), 'Invalid rotAngRng. A valid one should satisfy rotAngRng(1) < rotAngRng(2)');
            
            [optimAng, ~, exitFlag] = fminbnd(@ObjectiveFunction, rotAngRng(1), rotAngRng(2), optimset('TolX',obj.configParam.oneof_optim_method_param.fminbnd_param.tolerance_var,'Display',obj.configParam.oneof_optim_method_param.fminbnd_param.display));
            [optimVal, optimEigVec] = RotateAnglePredictor.SmallestEigenValue2(optimAng, pt1Ics, pt2Ics, intrMat, coordSysAligner, camInd, rotAxis);
            
            
            
            function smallestEigVal = ObjectiveFunction(rotAng)
                smallestEigVal = RotateAnglePredictor.SmallestEigenValue2(rotAng, pt1Ics, pt2Ics, intrMat, coordSysAligner, camInd,rotAxis);
            end
        end
        
    end
    
    methods (Static)
        function [smallestEigVal, smallestEigVec] = SmallestEigenValue(rotAng, pt1Ics, pt2Ics, intrMat, coordSysAligner, camInd)
            rotMatIcs = BodyRotateRotMatCcs(coordSysAligner, rotAng, camInd) / intrMat;
            pt1HomoCcsRot = rotMatIcs * HomoCoord(pt1Ics',1);
            pt2HomoCcs = intrMat \ HomoCoord(pt2Ics',1);

            planeNorm = cross(Normalize(pt1HomoCcsRot), Normalize(pt2HomoCcs));
            varMat = planeNorm * planeNorm';
            [eigVec, eigVal] = eig(varMat);
            eigVal = diag(eigVal)';
            smallestEigVal = eigVal(1);
            smallestEigVec = eigVec(:,1);
        end
    end
    
    methods (Static)
        function [smallestEigVal, smallestEigVec] = SmallestEigenValue2(rotAng, pt1Ics, pt2Ics, intrMat, coordSysAligner, camInd, rotAxis)
%             rotMatIcs = BodyRotateRotMatCcs(coordSysAligner, rotAng, camInd) / intrMat;
            b2c = coordSysAligner.pctBody2Cam(1, 1).transformMat(1:3,1:3);
            rBody = rodrigues(rotAxis*rotAng);
            rCam = b2c*rBody*b2c';
            rotMatIcs = rCam / intrMat;
            pt1HomoCcsRot = rotMatIcs * HomoCoord(pt1Ics',1);
            pt2HomoCcs = intrMat \ HomoCoord(pt2Ics',1);

            if 0
                planeNorm = cross(Normalize(pt1HomoCcsRot), Normalize(pt2HomoCcs));
            else
                planeNorm = cross(pt1HomoCcsRot, pt2HomoCcs);
                planeNorm = normc(planeNorm);
            end
            
            varMat = planeNorm * planeNorm';
            [eigVec, eigVal] = eig(varMat);
            eigVal = diag(eigVal)';
            smallestEigVal = eigVal(1);
            smallestEigVec = eigVec(:,1);
        end
    end
    methods
        % Implementation of abstract methods in Configurable base class
        function SetDefaultValue(obj, cfgParam)
            if isempty(cfgParam) || ~isfield(cfgParam, 'oneof_optim_method_param')
                obj.optimMethod = 'fminbnd';
                cfgParam.oneof_optim_method_param.fminbnd_param = [];
            end
            if strcmp(Configurable.OneofName(cfgParam.oneof_optim_method_param), 'fminbnd_param')
                obj.optimMethod = 'fminbnd';
            else
                error('Unsupported optim_method_param %s', Configurable.OneofName(cfpParam.oneof_optim_method_param));
            end
            
            switch obj.optimMethod
                case 'fminbnd'
                    cfgParam.oneof_optim_method_param.fminbnd_param = Configurable.SetField(cfgParam.oneof_optim_method_param.fminbnd_param, 'tolerance_var', 0.001);
                    cfgParam.oneof_optim_method_param.fminbnd_param = Configurable.SetField(cfgParam.oneof_optim_method_param.fminbnd_param, 'display', 'off');
                otherwise
                    error('Unsupported minimization method %s', cfgParam.MinimizeMethod);
            end
            
            obj.configParam = cfgParam;
        end
        
        function CheckConfig(obj) %#ok<MANU>
        end
    end
end