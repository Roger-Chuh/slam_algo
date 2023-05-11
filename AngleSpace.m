classdef AngleSpace < Configurable
    properties
        DEBUG_FLAG = true;
    end
    
    
    methods
        % Constructor
        function obj = AngleSpace(cfgParam)
            obj@Configurable(cfgParam);
        end
        
        function [k2cRotAng, k2cCamTransDirAng, projScore] = EvaluatePosterierProbability(obj, ptIcs1, ptIcs2, rotAngs, camTransDirAngs, inPolygonFlag, coordSysAligner, camInd,intrMat)
            %projErrType = obj.configParam.project_error_type;
            projErrType = obj.configParam.objective_Function_type;
            if strcmp(projErrType, 'L1')
                trackNoiseLvl = obj.configParam.track_noise_level_in_pixel;
            elseif strcmp(projErrType, 'L2')
                trackNoiseLvl = obj.configParam.track_noise_level_in_pixel^2;
            else
                error('Unsupported project error type: %s', projErrType);
            end
            transitionLength = obj.configParam.smooth_function_transition_length;
            
            if isempty(inPolygonFlag)
                inPolygonFlag = true(length(rotAngs), length(camTransDirAngs));
            end
            
            if (ischar(inPolygonFlag) && strcmp(inPolygonFlag, 'ValidPair'))
                rotAngList = rotAngs;
                camTransDirAngList = camTransDirAngs;
            elseif islogical(inPolygonFlag)
                [camTransDirAngGrid, rotAngGrid] = meshgrid(camTransDirAngs, rotAngs);
                rotAngList = rotAngGrid(inPolygonFlag);
                camTransDirAngList = camTransDirAngGrid(inPolygonFlag);
            else
                error('Wrong argument type of inPolygonFlag: it must be a string or a logical matrix');
            end
            
            [~, projErr] = ProjectErrorsStandard(obj, ptIcs1, ptIcs2, rotAngList, camTransDirAngList, coordSysAligner, camInd, intrMat);
            
            projErrHist = AngleSpace.SmoothRectFunc(projErr, -trackNoiseLvl, trackNoiseLvl, transitionLength);
            if strcmp(projErrType, 'L1')
                objVal = sum(abs(projErr).*projErrHist,2);
            elseif strcmp(projErrType, 'L2')
                objVal = sum(projErr.*projErr.*projErrHist,2);
            else
                error('Unsupported project error type: %s', projErrType);
            end
            %             [~, Ind] = min(objVal);
            %             k2cRotAng = rotAngList(Ind);
            %             k2cCamTransDirAng = camTransDirAngList(Ind);
            angleNumber = obj.configParam.angle_number;
            if angleNumber >= numel(rotAngList)
                warning('candidate angle number should be less than angle grid numbers')
                angleNumber = numel(rotAngList);
            end
            %             angleNumber = numel(rotAngList);
            try
                temp = sortrows([objVal,rotAngList,camTransDirAngList,projErrHist]);
            catch
                temp = sortrows([objVal,rotAngList',camTransDirAngList',projErrHist]);
            end
            k2cRotAng = temp(1:angleNumber,2);
            k2cCamTransDirAng = temp(1:angleNumber,3);
            projScore = temp(1:angleNumber,4:end);
            figure(17),clf;plot(objVal./size(ptIcs1,1));
            try
                debug.debug_901x_RotationAngleAmbiguity(camTransDirAngGrid,rotAngGrid,objVal,inPolygonFlag,k2cCamTransDirAng,k2cRotAng, obj.DEBUG_FLAG & false)
            catch
                warning('inPolygonFlag is valid')
            end
        end
        
        function inlerFlat = DetermineInliersStandard(obj, ptIcs1, ptIcs2, rotAngList, camTransDirAngList, coordSysAligner, camInd)
            trackNoiseLevel = obj.configParam.track_noise_level_in_pixel;
            transitionLength = obj.configParam.smooth_function_transition_length;
            supportFactor = obj.configParam.min_support_factor;
            
            numTrans = numel(rotAngList);
            numPts = Rows(ptIcs1);
            projErrList = zeros(numTrans, numPts);
            for iTrans = 1:numTrans
                projErrList(iTrans, :) = AngleSpace.ProjectErrorUnderTransform(rotAngList(iTrans), camTransDirAngList(iTrans), ptIcs1, ptIcs2, coordSysAligner, camInd, intrMat);
            end
            projErrHist = sum(AngleSpace.SmoothRectFunc(projErrList, -trackNoiseLevel, trackNoiseLevel, transitionLength), 2);
            maxHist = max(projErrHist);
            candAngPairFlag = projErrHist >= supportFactor * maxHist;
        end
        
        function varargout = ProjectErrorsStandard(obj, ptIcs1, ptIcs2, rotAngList, camTransDirAngList, coordSysAligner, camInd, intrMat)
            objType = obj.configParam.objective_Function_type;
            
            numTrans = numel(rotAngList);
            numPts = Rows(ptIcs1);
            objValList = zeros(numTrans, 1);
            projErrList = zeros(numTrans, numPts);
            for iTrans = 1:numTrans
                projErrList(iTrans, :) = AngleSpace.ProjectErrorUnderTransform(rotAngList(iTrans), camTransDirAngList(iTrans), ptIcs1, ptIcs2, coordSysAligner, camInd, intrMat);
                %                 rotMatCcs = BodyRotateRotMatCcs(coordSysAligner, rotAngList(iTrans), camInd);
                %                 transVecCcs = -R(GetPctBody2Cam(coordSysAligner))'*[cos(camTransDirAngList(iTrans)); 0; sin(camTransDirAngList(iTrans))];
                %                 projErrList(iTrans, :) = ProjectErrorUnderTransform(obj, ptIcs1, ptIcs2, rotMatCcs, transVecCcs, false);
            end
            if strcmp(objType, 'L1')
                objValList = sum(abs(projErrList),2);
            elseif strcmp(objType, 'L2')
                objValList = sum(projErrList.^2, 2);
            else
                error('Unrecognized objective function type: %s', objType);
            end
            varargout{1} = objValList;
            if (nargout > 1)
                varargout{2} = projErrList;
                if (nargout > 2)
                    assert(false);
                end
            end
        end
        
        %         function [projErr, varargout] = ProjectErrorUnderTransform(obj, ptIcs1, ptIcs2, rotMatCcs, transVecCcs, calcGrad)
        %             projErrType = obj.configParam.project_error_type;
        %
        %             if strcmp(projErrType, 'L1')
        %                 if calcGrad
        %                     error('Gradient is only calculated when project_error_type is set to ''L2''');
        %                 end
        %             end
        %
        %             planeNorm = cross(HomoCoord(ptIcs2'), rotMatCcs*HomoCoord(ptIcs1'));
        %             num = transVecCcs'*planeNorm;
        %             den = VecNorm(SkewSymMat([0;0;1]) * planeNorm);
        %             if strcmp(projErrType, 'L1')
        %                 projErr = abs(num./den);
        % %                trackNoise = obj.configParam.track_noise_level_in_pixel;
        %             elseif strcmp(projErrType, 'L2')
        %                 projErr = (num./den).^2;
        % %                trackNoise = obj.configParam.track_noise_level_in_pixel^2;
        %             else
        %                 error('Unsupported project error type: %s', projErrType);
        %             end
        % %            transitionLength = obj.configParam.smooth_function_transition_length;
        %
        %             if (nargout >= 1)
        %                 if (nargout >= 2)
        %                     if calcGrad
        %                         error('TODO');
        %                         varargout{1} = objVal;
        %                         if (nargout >= 3)
        %                         	error('Too many outut arguments');
        %                         end
        %                     end
        %                 end
        %             else
        %                 error('Expect at least one output.');
        %             end
        %         end
        
        function PlotTopologyFig(obj)
        end
    end
    
    methods (Static)
        function [projErr, jacMat] =  ProjectErrorUnderTransform(rotAng, transDirAng, ptIcs1, ptIcs2, coordSysAligner, camInd, intrMat)
            rotMatCcs = BodyRotateRotMatCcs(coordSysAligner, rotAng, camInd);
            transVecCcs = -R(GetPctBody2Cam(coordSysAligner))*[cos(transDirAng); 0; sin(transDirAng)];
            transVecCcs = repmat(transVecCcs,[1,Rows(ptIcs1)]);
            planeNorm = intrMat'\ cross(rotMatCcs/intrMat*HomoCoord(ptIcs1',1), transVecCcs);
            num = dot(HomoCoord(ptIcs2',1),planeNorm);
            den = VecNorm(SkewSymMat([0;0;1]) * planeNorm);
            projErr = num./den;
            
            if (nargout == 2)
                error('TODO');
                varargout{1} = jacMat;
            elseif (nargout > 2)
                error('Too many output arguments');
            end
        end
        
        function y = SmoothRectFunc(x, a, b, c)
            
            y = (tanh((x - a)/c) - tanh((x-b)/c))/2;
            
        end
    end
    
    methods (Static)
        
        function [k2cOpt, p2cOpt, k2pOpt] = EpiPolarThreeView(obj, k2cInit, p2cInit, thetaSamp, PixKey, PixPrv, PixCur, intrMat, b2c)
            k2pInit = k2cInit - p2cInit;
            
            thetaK2CRng = k2cInit + thetaSamp;
            thetaP2CRng = p2cInit + thetaSamp;
            
            cnt = 1;
            if k2cInit ~= p2cInit
                errMat = nan(length(thetaK2CRng), length(thetaP2CRng));
            else
                errMat = nan(length(thetaK2CRng),1);
            end
            for i = 1 : length(thetaK2CRng)
                k2cTemp = thetaK2CRng(i);
                
                k2cBodyTmp = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cTemp),zeros(3,1));
                k2cCam = b2c*k2cBodyTmp.transformMat/b2c;
                [reProjErr1(i,1),err1(:,i)] = AngleSpace.ProjectErrorUnderTransform2(obj.configParam.reproj_sigma, k2cCam(1:3,1:3), k2cCam(1:3,4), PixKey, PixCur, intrMat, [], [], [], []);
                
                if k2cInit ~= p2cInit
                    for j = 1 : length(thetaP2CRng)
                        p2cTemp = thetaP2CRng(j);
                        k2pTemp = k2cTemp - p2cTemp;
                        
                        p2cBodyTmp = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(p2cTemp),zeros(3,1));
                        p2cCam = b2c*p2cBodyTmp.transformMat/b2c;
                        
                        k2pBodyTmp = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2pTemp),zeros(3,1));
                        k2pCam = b2c*k2pBodyTmp.transformMat/b2c;
                         
                        [reProjErr2(cnt,1),err2(:,cnt)] = AngleSpace.ProjectErrorUnderTransform2(obj.configParam.reproj_sigma, k2pCam(1:3,1:3), k2pCam(1:3,4), PixKey, PixPrv, intrMat, [], [], [], []);
                        [reProjErr3(cnt,1),err3(:,cnt)] = AngleSpace.ProjectErrorUnderTransform2(obj.configParam.reproj_sigma, p2cCam(1:3,1:3), p2cCam(1:3,4), PixPrv, PixCur, intrMat, [], [], [], []);
                        
                        
                        errMat(i,j) = reProjErr1(i,1) + reProjErr2(cnt,1) + reProjErr3(cnt,1);
                        cnt = cnt + 1;
                    end
                else
                    
                    errMat(i,:) = reProjErr1(i,1);
                    
                end
            end
            
            [y,x] = ind2sub(size(errMat), find(errMat(:) == min(errMat(:))));
            
            p2cId = x;
            k2cId = y;
            
            k2cOpt = thetaK2CRng(k2cId(1));
            p2cOpt = thetaP2CRng(p2cId(1));
            k2pOpt = k2cOpt - p2cOpt;
            if 0
                figure,surf(errMat);
                
                k2cTemp = thetaK2CRng(k2cId);
                
                k2cBodyTmp = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cTemp),zeros(3,1));
                k2cCam = b2c*k2cBodyTmp.transformMat/b2c;
                [reProjErr11,~] = AngleSpace.ProjectErrorUnderTransform2(obj.configParam.reproj_sigma, k2cCam(1:3,1:3), k2cCam(1:3,4), PixKey, PixCur, intrMat, [], [], [], []);
                
                p2cTemp = thetaP2CRng(p2cId);
                k2pTemp = k2cTemp - p2cTemp;
                
                p2cBodyTmp = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(p2cTemp),zeros(3,1));
                p2cCam = b2c*p2cBodyTmp.transformMat/b2c;
                
                k2pBodyTmp = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2pTemp),zeros(3,1));
                k2pCam = b2c*k2pBodyTmp.transformMat/b2c;
                [reProjErr22,~] = AngleSpace.ProjectErrorUnderTransform2(obj.configParam.reproj_sigma, k2pCam(1:3,1:3), k2pCam(1:3,4), PixKey, PixPrv, intrMat, [], [], [], []);
                [reProjErr33,~] = AngleSpace.ProjectErrorUnderTransform2(obj.configParam.reproj_sigma, p2cCam(1:3,1:3), p2cCam(1:3,4), PixPrv, PixCur, intrMat, [], [], [], []);
                err123 = reProjErr11 + reProjErr22 + reProjErr33;
            end
            
            
        end
        
    end
    
    
    
    methods (Static)
        %         function [projErr, jacMat] =  ProjectErrorUnderTransform2(rotMatCcs, transVecCcs, ptIcs1, ptIcs2, coordSysAligner, camInd, intrMat)
        function [meanErr, projErr, PtIn1, distIn1, PtIn2, distIn2, X2In1, fundMat1to2, eplIn2] =  ProjectErrorUnderTransform2(radius, rotMatCcs1To2, transVecCcs1to2, ptIcs1, ptIcs2, intrMat,r_cam,tx, ty, tz)
            %             rotMatCcs = BodyRotateRotMatCcs(coordSysAligner, rotAng, camInd);
            %             transVecCcs = -R(GetPctBody2Cam(coordSysAligner))*[cos(transDirAng); 0; sin(transDirAng)];
            %             radius = obj.configParam.reproj_sigma;
            
            metricCcs1 = intrMat\HomoCoord(ptIcs1',1);
            
            
            
            T1to2 = [rotMatCcs1To2 transVecCcs1to2 ; 0 0 0 1];
            firstCamOrigInSecondCam = transVecCcs1to2';
            T2to1 = inv(T1to2);
            
            transVecCcs1to2 = transVecCcs1to2./norm(transVecCcs1to2);
            
            transVecCcs1to2 = repmat(transVecCcs1to2,[1,Rows(ptIcs1)]);
            planeNorm = intrMat'\ cross(rotMatCcs1To2/intrMat*HomoCoord(ptIcs1',1), transVecCcs1to2);
            num = dot(HomoCoord(ptIcs2',1),planeNorm);
            den = VecNorm(SkewSymMat([0;0;1]) * planeNorm);
            projErr0 = abs(num./den)';
            meanErr0 = mean(projErr0);
            %             if (nargout == 2)
            %                 error('TODO');
            %                 varargout{1} = jacMat;
            %             elseif (nargout > 2)
            %                 error('Too many output arguments');
            %             end
            
            
            ptIcs2Euclid = intrMat \ HomoCoord(ptIcs2',1);
            planeNormEuclid = normc(cross(rotMatCcs1To2/intrMat*HomoCoord(ptIcs1',1), transVecCcs1to2));
            num1 = dot(planeNormEuclid,ptIcs2Euclid);
            den1 = VecNorm(SkewSymMat([0;0;1]) * planeNormEuclid);
            projErr1 = num1./den1.*intrMat(1,1);
            
            [angZ, angX] = TransVec2Pol(firstCamOrigInSecondCam');
            
            [reprjErrIn1, reprjErrIn2,eplIn1,eplIn2, fundMat1to2] = ReprojectError2(ptIcs1, ptIcs2, intrMat, intrMat, rodrigues(rotMatCcs1To2(1:3,1:3)), real([angZ; angX]), 'extrinsic');
            
            if (nargout > 3)
                [PtIn1, distIn1] = Pt2Line([],[],ptIcs1,eplIn1);
                [PtIn2, distIn2] = Pt2Line([],[],ptIcs2,eplIn2);
            end
            
            
            projErr = max([abs(reprjErrIn1)  abs(reprjErrIn2)]')';  % abs(reprjErrIn1) +  abs(reprjErrIn2);
            
            errCheckConsistency = [(projErr0 - abs(reprjErrIn2))  (projErr0 - abs(projErr1'))];
            
            
            meanErr = mean(projErr);
            
            [pt21, pt22] = IntersectLineCircle(ptIcs2,eplIn2,radius);
            
            
            
            
            pt21_norm = intrMat\HomoCoord(pt21',1);
            pt22_norm = intrMat\HomoCoord(pt22',1);
            
            if (nargout > 3)
                PtIn2_norm = intrMat\HomoCoord(PtIn2',1);
            end
            
            [X1, err2_1,err1_1] = GetBoundaryDepth1(intrMat, T2to1,pt21_norm,metricCcs1, pt21, ptIcs1);
            [X2, err2_2,err1_2] = GetBoundaryDepth1(intrMat, T2to1,pt22_norm,metricCcs1, pt22, ptIcs1);
            
            
            
            
            
            
            if (nargout > 3)
                [X1In2, err1in2,X2In1,err2in1] = GetBoundaryDepth1(intrMat, T2to1,PtIn2_norm,metricCcs1, PtIn2, ptIcs1);
                [angle_result1to2,depth2In1] = VisualLocalizer.AnglePredict_Prob(metricCcs1,PtIn2_norm,r_cam,tx, ty, tz);
                if 0
                    figure,plot(X2In1(1:50,3) - depth2In1(1:50));
                end
            end
            
            
            LineDir1_1 = X1 - repmat(firstCamOrigInSecondCam,size(X1,1),1);
            LineDir1_2 = X2 - repmat(firstCamOrigInSecondCam,size(X2,1),1);
            
            [LineDir1_1_norm, ~] = NormalizeVector(LineDir1_1);
            [LineDir1_2_norm, ~] = NormalizeVector(LineDir1_2);
            errLineDir12 = LineDir1_1_norm + LineDir1_2_norm;
            %      figure(32),clf; [radi, ptOnLineErr1, ptOnLineErr2] = ShowEpilineCircle1(ptIcs2,ptIcs1, fundMat1to2, eplIn2, zeros(240,320), [pt21 pt22]);
            
        end
    end
    
    
    methods
        % Implementation of abstract methods in Configurable base class
        function SetDefaultValue(obj, cfgParam)
            cfgParam = Configurable.SetField(cfgParam, 'objective_Function_type', 'L1');
            cfgParam = Configurable.SetField(cfgParam, 'track_noise_level_in_pixel', 2);
            cfgParam = Configurable.SetField(cfgParam, 'smooth_function_transition_length', 0.01);
            cfgParam = Configurable.SetField(cfgParam, 'min_support_factor', 0.8);
            cfgParam = Configurable.SetField(cfgParam, 'angle_number', 1);
            
            obj.configParam = cfgParam;
        end
        
        function CheckConfig(obj) %#ok<MANU>
        end
    end
    
end