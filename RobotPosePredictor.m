classdef RobotPosePredictor < Configurable
    
    properties
        prvBodyOrigErrRng; % 1x2 vector [leftLimit, rightLimit] in X direction of previous BCS frame relative to origin
    end
    
    methods
        % Constructor
        function obj = RobotPosePredictor(cfgParam)
            obj@Configurable(cfgParam);
            obj.prvBodyOrigErrRng = [0,0]; %[-1 1];
        end
        
        function [k2cRotAngs, k2cCamTransDirAngs, inPolygonFlag, curBodyOrigUncertPolygonBcsPrv] = PredictRobotPose(obj, key2prvRotAng, prvBodyInKeyBody, prv2curPredRotAng, prv2curPredDisplacement, coordSysAligner, camInd)
            if ~exist('camInd', 'var')
                camInd = 1;
            end
            p2k = [roty(double(rad2deg(key2prvRotAng)))' prvBodyInKeyBody;0 0 0 1];
            [k2pBodyTransVecBcsKey, p2cRotAngRng, p2cDisplacementRng] = GetPoseParamRanges(obj, key2prvRotAng, prvBodyInKeyBody, prv2curPredRotAng, prv2curPredDisplacement,p2k);
            
            curBodyOrigUncertPolygonBcsPrv = UncertainPolygonBcsPrv(obj, p2cRotAngRng, p2cDisplacementRng,p2k);
            
            [k2cRotAngs, k2cCamTransDirAngs, inPolygonFlag] = AngleSpaceSampleGrid(obj, key2prvRotAng, k2pBodyTransVecBcsKey, prv2curPredRotAng, p2cDisplacementRng, GetPctBody2Cam(coordSysAligner, camInd));
        end
        
        function [key2prvBodyTransVecBcsKey, prv2curRotAngRng, prv2curDisplacementRng] = GetPoseParamRanges(obj, key2prvRotAng, prvBodyOrigBcsKey, prv2curPredRotAng, prv2curPredDisplacement,p2k)
            c = cos(key2prvRotAng);
            s = sin(key2prvRotAng);
             p2cDispErrRng = obj.configParam.prev2curr_displacement_err_range;
            prv2curDisplacementRng = prv2curPredDisplacement + p2cDispErrRng;
            assert(prv2curDisplacementRng(2) > 0, 'Moving back is not supported yet.');
            prv2curDisplacementRng(1) = max(0, prv2curDisplacementRng(1));
            %            prvBodyOrigErrRng = obj.configParam.prev_body_origin_err_range;
            
            
            leftSide = [obj.prvBodyOrigErrRng(1);0;0]; rightSide = [obj.prvBodyOrigErrRng(2);0;0]; 
            key2prvBodyTransVecBcsKey = [p2k(1:3,1:3)*leftSide + p2k(1:3,4) p2k(1:3,1:3)*rightSide + p2k(1:3,4)];
            
%             curveTop = prv2curDisplacementRng;
%             curveLeft = prv2curDisplacementRng;
            
%             key2prvBodyTransVecBcsKey = [[c * obj.prvBodyOrigErrRng(1) + prvBodyOrigBcsKey(1); 0; s * obj.prvBodyOrigErrRng(1) + prvBodyOrigBcsKey(3)], ...
%                 [c * obj.prvBodyOrigErrRng(2) + prvBodyOrigBcsKey(1); 0; s * obj.prvBodyOrigErrRng(2) + prvBodyOrigBcsKey(3)]];
%             
            p2cRoAngErrRng = obj.configParam.prev2curr_rotate_angle_err_range;
            prv2curRotAngRng = prv2curPredRotAng + p2cRoAngErrRng;
            
            
        end
        
        function polyCnr = UncertainPolygonBcsPrv(obj, prv2curRotAngRng, prv2curDisplacementRange,p2k)
            %            prvBodyOrigErrRng = obj.configParam.prev_body_origin_err_range;
            prvBodyOrigRngBcsPrv = [obj.prvBodyOrigErrRng; zeros(2,2)];
            
            leftSide = [obj.prvBodyOrigErrRng(1);0;0]; rightSide = [obj.prvBodyOrigErrRng(2);0;0]; 
            key2prvBodyTransVecBcsKey1 = [p2k(1:3,1:3)*leftSide + p2k(1:3,4) p2k(1:3,1:3)*rightSide + p2k(1:3,4)];
            
            p2cBodyTransDirAngRngBcsZPrv = prv2curRotAngRng/2;
            if 1
                curveTop = prv2curDisplacementRange(2);
                curveBottom = prv2curDisplacementRange(1);
%                 curveLeft = 1;
%                 curveLeft = 1;
                XZTop = []; XZBottom = [];
                for j = [p2cBodyTransDirAngRngBcsZPrv(1) : 0.0001*5 : p2cBodyTransDirAngRngBcsZPrv(2) p2cBodyTransDirAngRngBcsZPrv(2)]
                    
                    XZTop = [XZTop; [curveTop*cos(pi/2+j) 0 curveTop*sin(pi/2+j)]];
                    XZBottom = [XZBottom; [[curveBottom*cos(pi/2+j) 0 curveBottom*sin(pi/2+j)]]];
                
                end
                topLeft = XZTop(end,:); topRight = XZTop(1,:);
                bottomLeft = XZBottom(end,:); bottomRight = XZBottom(1,:);
                topTop = [mean(XZTop(XZTop(:,3) == max(XZTop(:,3)),1)) 0 max(XZTop(:,3))];
                
                if 0
                    one = [topLeft;topRight;bottomLeft;bottomRight;topTop];
                    %                 one = [topLeft;topTop;topRight;bottomLeft;bottomRight];
                    one1 = one; one1(:,1) = one1(:,1) + obj.prvBodyOrigErrRng(1);
                    one2 = one; one2(:,1) = one2(:,1) + obj.prvBodyOrigErrRng(2);
                    
                    key2prvBodyTransVecBcsKey = [p2k(1:3,1:3)*one1' + repmat(p2k(1:3,4),1,size(one1,1)) p2k(1:3,1:3)*one2' + repmat(p2k(1:3,4),1,size(one2,1))]';
                    key2prvBodyTransVecBcsKey = unique(key2prvBodyTransVecBcsKey,'rows')';
                    polyCnr = FindPolyon(double(key2prvBodyTransVecBcsKey([1 3],:))');
                    polyCnr = [polyCnr(1:end-1,1) zeros(size(polyCnr,1)-1,1) polyCnr(1:end-1,2)]';
                else
                    one = [topLeft;topTop;topRight;bottomLeft;bottomRight];
                    one1 = one; one1(:,1) = one1(:,1) + obj.prvBodyOrigErrRng(1);
                    one2 = one; one2(:,1) = one2(:,1) + obj.prvBodyOrigErrRng(2);
                    key2prvBodyTransVecBcsKey = [p2k(1:3,1:3)*one1' + repmat(p2k(1:3,4),1,size(one1,1)) p2k(1:3,1:3)*one2' + repmat(p2k(1:3,4),1,size(one2,1))]';
                    %                     key2prvBodyTransVecBcsKey = key2prvBodyTransVecBcsKey([1 2 3 6 7 8 10 9 5 4],:);
                    key2prvBodyTransVecBcsKey = key2prvBodyTransVecBcsKey([1 2 8 10 4],:);
                    polyCnr = key2prvBodyTransVecBcsKey';
                end
                
                
                
                
%                 one = [topLeft;topRight;bottomLeft;bottomRight;topTop];
%                 one1 = one; one1(:,1) = one1(:,1) + obj.prvBodyOrigErrRng(1);
%                 one2 = one; one2(:,1) = one2(:,1) + obj.prvBodyOrigErrRng(2);
%                 
%                 key2prvBodyTransVecBcsKey = [p2k(1:3,1:3)*one1' + repmat(p2k(1:3,4),1,size(one1,1)) p2k(1:3,1:3)*one2' + repmat(p2k(1:3,4),1,size(one2,1))]';
%                 key2prvBodyTransVecBcsKey = unique(key2prvBodyTransVecBcsKey,'rows')';
%                 polyCnr = FindPolyon(double(key2prvBodyTransVecBcsKey([1 3],:))');
%                 polyCnr = [polyCnr(1:end-1,1) zeros(size(polyCnr,1)-1,1) polyCnr(1:end-1,2)]';
            
                if (p2cBodyTransDirAngRngBcsZPrv(1) < 0 && p2cBodyTransDirAngRngBcsZPrv(2) > 0)
                    if (abs(p2cBodyTransDirAngRngBcsZPrv(1)) < abs(p2cBodyTransDirAngRngBcsZPrv(2)))
                        uncertPolygon = zeros(3,7);
                        uncertPolygon(:, 1) = prv2curDisplacementRange(1)*[-sin(p2cBodyTransDirAngRngBcsZPrv(2)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(2))] + prvBodyOrigRngBcsPrv(:,1);
                        uncertPolygon(:, 2) = prv2curDisplacementRange(1)*[-sin(p2cBodyTransDirAngRngBcsZPrv(2)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(2))] + prvBodyOrigRngBcsPrv(:,2);
                        uncertPolygon(:, 3) = prv2curDisplacementRange(1)*[-sin(p2cBodyTransDirAngRngBcsZPrv(1)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(1))] + prvBodyOrigRngBcsPrv(:,2);
                        uncertPolygon(:, 4) = prv2curDisplacementRange(2)*[-sin(p2cBodyTransDirAngRngBcsZPrv(1)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(1))] + prvBodyOrigRngBcsPrv(:,2);
                        uncertPolygon(:, 5) = prv2curDisplacementRange(2)*[0; 0; 1] + prvBodyOrigRngBcsPrv(:,2);
                        uncertPolygon(:, 6) = prv2curDisplacementRange(2)*[0; 0; 1] + prvBodyOrigRngBcsPrv(:,1);
                        uncertPolygon(:, 7) = prv2curDisplacementRange(2)*[-sin(p2cBodyTransDirAngRngBcsZPrv(2)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(2))] + prvBodyOrigRngBcsPrv(:,1);
                    elseif (abs(p2cBodyTransDirAngRngBcsZPrv(1)) > abs(p2cBodyTransDirAngRngBcsZPrv(2)))
                        uncertPolygon = zeros(3,7);
                        uncertPolygon(:, 1) = prv2curDisplacementRange(1)*[-sin(p2cBodyTransDirAngRngBcsZPrv(2)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(2))] + prvBodyOrigRngBcsPrv(:,1);
                        uncertPolygon(:, 2) = prv2curDisplacementRange(1)*[-sin(p2cBodyTransDirAngRngBcsZPrv(1)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(1))] + prvBodyOrigRngBcsPrv(:,1);
                        uncertPolygon(:, 3) = prv2curDisplacementRange(1)*[-sin(p2cBodyTransDirAngRngBcsZPrv(1)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(1))] + prvBodyOrigRngBcsPrv(:,2);
                        uncertPolygon(:, 4) = prv2curDisplacementRange(2)*[-sin(p2cBodyTransDirAngRngBcsZPrv(1)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(1))] + prvBodyOrigRngBcsPrv(:,2);
                        uncertPolygon(:, 5) = prv2curDisplacementRange(2)*[0; 0; 1] + prvBodyOrigRngBcsPrv(:,2);
                        uncertPolygon(:, 6) = prv2curDisplacementRange(2)*[0; 0; 1] + prvBodyOrigRngBcsPrv(:,1);
                        uncertPolygon(:, 7) = prv2curDisplacementRange(2)*[-sin(p2cBodyTransDirAngRngBcsZPrv(2)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(2))] + prvBodyOrigRngBcsPrv(:,1);
                    else
                        uncertPolygon = zeros(3,6);
                        uncertPolygon(:, 1) = prv2curDisplacementRange(1)*[-sin(p2cBodyTransDirAngRngBcsZPrv(2)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(2))] + prvBodyOrigRngBcsPrv(:,1);
                        uncertPolygon(:, 2) = prv2curDisplacementRange(1)*[-sin(p2cBodyTransDirAngRngBcsZPrv(1)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(1))] + prvBodyOrigRngBcsPrv(:,2);
                        uncertPolygon(:, 3) = prv2curDisplacementRange(2)*[-sin(p2cBodyTransDirAngRngBcsZPrv(1)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(1))] + prvBodyOrigRngBcsPrv(:,2);
                        uncertPolygon(:, 4) = prv2curDisplacementRange(2)*[0; 0; 1] + prvBodyOrigRngBcsPrv(:,2);
                        uncertPolygon(:, 5) = prv2curDisplacementRange(2)*[0; 0; 1] + prvBodyOrigRngBcsPrv(:,1);
                        uncertPolygon(:, 6) = prv2curDisplacementRange(2)*[-sin(p2cBodyTransDirAngRngBcsZPrv(2)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(2))] + prvBodyOrigRngBcsPrv(:,1);
                    end
                elseif (p2cBodyTransDirAngRngBcsZPrv(1) >= 0)
                    uncertPolygon = zeros(3,6);
                    uncertPolygon(:, 1) = prv2curDisplacementRange(1)*[-sin(p2cBodyTransDirAngRngBcsZPrv(2)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(2))] + prvBodyOrigRngBcsPrv(:,1);
                    uncertPolygon(:, 2) = prv2curDisplacementRange(1)*[-sin(p2cBodyTransDirAngRngBcsZPrv(2)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(2))] + prvBodyOrigRngBcsPrv(:,2);
                    uncertPolygon(:, 3) = prv2curDisplacementRange(1)*[-sin(p2cBodyTransDirAngRngBcsZPrv(1)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(1))] + prvBodyOrigRngBcsPrv(:,2);
                    uncertPolygon(:, 4) = prv2curDisplacementRange(2)*[-sin(p2cBodyTransDirAngRngBcsZPrv(1)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(1))] + prvBodyOrigRngBcsPrv(:,2);
                    uncertPolygon(:, 5) = prv2curDisplacementRange(2)*[-sin(p2cBodyTransDirAngRngBcsZPrv(1)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(1))] + prvBodyOrigRngBcsPrv(:,1);
                    uncertPolygon(:, 6) = prv2curDisplacementRange(2)*[-sin(p2cBodyTransDirAngRngBcsZPrv(2)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(2))] + prvBodyOrigRngBcsPrv(:,1);
                elseif (p2cBodyTransDirAngRngBcsZPrv(2) <= 0)
                    uncertPolygon = zeros(3,6);
                    uncertPolygon(:, 1) = prv2curDisplacementRange(1)*[-sin(p2cBodyTransDirAngRngBcsZPrv(2)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(2))] + prvBodyOrigRngBcsPrv(:,1);
                    uncertPolygon(:, 2) = prv2curDisplacementRange(1)*[-sin(p2cBodyTransDirAngRngBcsZPrv(1)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(1))] + prvBodyOrigRngBcsPrv(:,1);
                    uncertPolygon(:, 3) = prv2curDisplacementRange(1)*[-sin(p2cBodyTransDirAngRngBcsZPrv(1)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(1))] + prvBodyOrigRngBcsPrv(:,2);
                    uncertPolygon(:, 4) = prv2curDisplacementRange(2)*[-sin(p2cBodyTransDirAngRngBcsZPrv(1)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(1))] + prvBodyOrigRngBcsPrv(:,2);
                    uncertPolygon(:, 5) = prv2curDisplacementRange(2)*[-sin(p2cBodyTransDirAngRngBcsZPrv(2)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(2))] + prvBodyOrigRngBcsPrv(:,2);
                    uncertPolygon(:, 6) = prv2curDisplacementRange(2)*[-sin(p2cBodyTransDirAngRngBcsZPrv(2)); 0; cos(p2cBodyTransDirAngRngBcsZPrv(2))] + prvBodyOrigRngBcsPrv(:,1);
                else
                    assert(false);
                end
            end
            
            
% %             figure(167),clf;plot(polyCnr(1,:),polyCnr(3,:),'-x');hold on;plot(uncertPolygon(1,:),uncertPolygon(3,:));
        end
        
        function [k2cRotAngs, k2cCamTransDirAngs, inPolygonFlag] = AngleSpaceSampleGrid(obj, key2prvRotAng, key2prvBodyOrigBndBcsKey, prv2curPredRotAng, prv2curPredDisplacementRng, pctB2C)
            k2cRotAngRng = key2prvRotAng + prv2curPredRotAng + obj.configParam.prev2curr_rotate_angle_err_range;
            rotAngPrec = obj.configParam.prev2curr_rotate_angle_sample_precision;
            camTransDirPrec = obj.configParam.prev2curr_transdir_angle_sample_precision;
            numSmallAngDiv = obj.configParam.prev2curr_num_small_angle_division;
            
            
            if ((k2cRotAngRng(2) - k2cRotAngRng(1)) / rotAngPrec < numSmallAngDiv)
                rotAngStep = rotAngPrec /floor(rotAngPrec/((k2cRotAngRng(2) - k2cRotAngRng(1))/numSmallAngDiv));
            else
                rotAngStep = rotAngPrec;
            end
%             k2cRotAngs = floor(k2cRotAngRng(1) / rotAngStep) * rotAngStep : rotAngStep : ceil(k2cRotAngRng(2) / rotAngStep) * rotAngStep;
            k2cRotAngs = ceil(k2cRotAngRng(1) / rotAngStep) * rotAngStep : rotAngStep : floor(k2cRotAngRng(2) / rotAngStep) * rotAngStep;
            
            alphaVsThetaLimitCurves = zeros(4, numel(k2cRotAngs));
            alphaVsThetaLimitCurves(1,:) = RobotPosePredictor.AlphaVsThetaCurve(k2cRotAngs, prv2curPredDisplacementRng(1), key2prvRotAng, key2prvBodyOrigBndBcsKey(:,1), pctB2C);
            alphaVsThetaLimitCurves(2,:) = RobotPosePredictor.AlphaVsThetaCurve(k2cRotAngs, prv2curPredDisplacementRng(2), key2prvRotAng, key2prvBodyOrigBndBcsKey(:,1), pctB2C);
            alphaVsThetaLimitCurves(3,:) = RobotPosePredictor.AlphaVsThetaCurve(k2cRotAngs, prv2curPredDisplacementRng(1), key2prvRotAng, key2prvBodyOrigBndBcsKey(:,2), pctB2C);
            alphaVsThetaLimitCurves(4,:) = RobotPosePredictor.AlphaVsThetaCurve(k2cRotAngs, prv2curPredDisplacementRng(2), key2prvRotAng, key2prvBodyOrigBndBcsKey(:,2), pctB2C);
            
            p2cCamTransDirAngLowLim = min(alphaVsThetaLimitCurves);
            p2cCamTransDirAngHighLim = max(alphaVsThetaLimitCurves);
            p2cCamTransDirAngRng = [min(p2cCamTransDirAngLowLim), max(p2cCamTransDirAngHighLim)];
            if ((p2cCamTransDirAngRng(2) - p2cCamTransDirAngRng(1)) / camTransDirPrec < numSmallAngDiv)
                camTransDirAngStep = camTransDirPrec / floor(camTransDirPrec / ((p2cCamTransDirAngRng(2) - p2cCamTransDirAngRng(1))/numSmallAngDiv));
            else
                camTransDirAngStep = camTransDirPrec;
            end
%             k2cCamTransDirAngs = floor(p2cCamTransDirAngRng(1) / camTransDirAngStep) * camTransDirAngStep : camTransDirAngStep : ceil(p2cCamTransDirAngRng(2) / camTransDirAngStep) * camTransDirAngStep;
            k2cCamTransDirAngs = ceil(p2cCamTransDirAngRng(1) / camTransDirAngStep) * camTransDirAngStep : camTransDirAngStep : floor(p2cCamTransDirAngRng(2) / camTransDirAngStep) * camTransDirAngStep;
         
            
            [sampleGridAlpha, sampleGridTheta] = meshgrid(k2cCamTransDirAngs, k2cRotAngs);            
%             [sampleGridAlpha, sampleGridTheta] = meshgrid(sort(alphaVsThetaLimitCurves(:)','ascend'), k2cRotAngs);
            margin = obj.configParam.alphaVsTheta_margin;
            inPolygonFlag = bsxfun(@ge, sampleGridAlpha, p2cCamTransDirAngLowLim' + margin*camTransDirAngStep) & bsxfun(@le, sampleGridAlpha,  p2cCamTransDirAngHighLim' - margin*camTransDirAngStep) & ...
                (sampleGridTheta >= k2cRotAngRng(1) + margin*rotAngStep & sampleGridTheta <= k2cRotAngRng(2) - margin*rotAngStep);
%             inPolygonFlag = bsxfun(@ge, sampleGridAlpha, p2cCamTransDirAngLowLim' + 0*camTransDirAngStep) & bsxfun(@le, sampleGridAlpha,  p2cCamTransDirAngHighLim' - 0*camTransDirAngStep) & ...
%                 (sampleGridTheta >= k2cRotAngRng(1) + 0*rotAngStep & sampleGridTheta <= k2cRotAngRng(2) - 0*rotAngStep);
            
            fprintf(sprintf('### inPolygonFlag: %d ###\n\n\n',sum(sum(inPolygonFlag))));
            if all(~inPolygonFlag(:))
                k2cCamTransDirAngs = (p2cCamTransDirAngLowLim + p2cCamTransDirAngHighLim)/2;
                inPolygonFlag = 'ValidPair';
            end
            
%                         RobotPosePredictor.PlotUncertainPolygonInAngleSpace(k2cRotAngs, alphaVsThetaLimitCurves, p2cCamTransDirAngLowLim, p2cCamTransDirAngHighLim, sampleGridAlpha, sampleGridTheta, inPolygonFlag);
        end
        
    end
    
    methods (Static)      
        function key2curTransVecDirAngBcsCur = AlphaVsThetaCurve(key2curRotAng, prv2curBodyDisplacement, key2prvRotAng, key2prvBodyOrigBcsKey, pctB2C)
            rC2B = R(Inv(pctB2C));
            tC2B = T(Inv(pctB2C));
            x0 = tC2B(1)/1000;
            z0 = tC2B(3)/1000;
            theta01 = key2prvRotAng;
            k2pT = [key2prvBodyOrigBcsKey(1);key2prvBodyOrigBcsKey(3)];
            v = prv2curBodyDisplacement;
            for i = 1:length(key2curRotAng)
                theta02 = double(key2curRotAng(i));
                gamma = 0.5*(theta02-theta01);
                phi = 0.5*pi + gamma + theta02;
                k2cR = [cos(theta02),sin(theta02);-sin(theta02),cos(theta02)];
                keyC0 = [x0;z0];
                c2kT = k2pT+v*[cos(phi);sin(phi)];
                curC0 = k2cR*(keyC0 - c2kT);
                curC2 = keyC0;
                vec = curC2 - curC0;
                key2curTransVecDirAngBcsCur(i) = atan(vec(2)/vec(1));
            end
            key2curTransVecDirAngBcsCur(key2curTransVecDirAngBcsCur < 0) = key2curTransVecDirAngBcsCur(key2curTransVecDirAngBcsCur < 0) + pi;
%             Alpha = PredAlpha(theta01, key2curRotAng-theta01, rC2B, tC2B, prv2curBodyDisplacement,x0,z0);
            Alpha = PredAlpha(theta01, key2curRotAng-theta01, rC2B, tC2B, prv2curBodyDisplacement,key2prvBodyOrigBcsKey(1)*1000,key2prvBodyOrigBcsKey(3)*1000);
            key2curTransVecDirAngBcsCur = Alpha'; 
        end
        
        function PlotUncertainPolygonInAngleSpace(k2cRotAngs, alphaVsThetaLimitCurves, p2cCamTransDirAngLowLim, p2cCamTransDirAngHighLim, sampleGridAlpha, sampleGridTheta, inPolygonFlag)
            global TOPOGRAPHY_FIG
            
            if isempty(TOPOGRAPHY_FIG)
                return;
            end
            
            figure(TOPOGRAPHY_FIG)
            if islogical(inPolygonFlag)
                subplot(2,2,3), plot(p2cCamTransDirAngLowLim, k2cRotAngs, '*', ...
                    p2cCamTransDirAngHighLim, k2cRotAngs, 'o', ...
                    alphaVsThetaLimitCurves', k2cRotAngs, ...
                    sampleGridAlpha(:), sampleGridTheta(:), 'b+', ...
                    sampleGridAlpha(inPolygonFlag), sampleGridTheta(inPolygonFlag), 'ro');
            else
                subplot(2,2,3), plot(p2cCamTransDirAngLowLim, k2cRotAngs, '*', ...
                    p2cCamTransDirAngHighLim, k2cRotAngs, 'o', ...
                    alphaVsThetaLimitCurves', k2cRotAngs, ...
                    sampleGridAlpha(:), sampleGridTheta(:), 'b+');
            end
        end
    end
    
    methods
        % Implementation of abstract methods in Configurable base class
        function SetDefaultValue(obj, cfgParam)
            cfgParam = Configurable.SetField(cfgParam, 'prev2curr_rotate_angle_err_range', [-0.01, 0.01] * pi / 180);  % rad
            cfgParam = Configurable.SetField(cfgParam, 'prev2curr_displacement_err_range', [-0.00001, 0.00001]);  % m
            cfgParam = Configurable.SetField(cfgParam, 'prev2curr_rotate_angle_sample_precision', 0.001 * pi / 180);  % rad
            cfgParam = Configurable.SetField(cfgParam, 'prev2curr_transdir_angle_sample_precision', 0.001 * pi / 180); % rad
            cfgParam = Configurable.SetField(cfgParam, 'prev2curr_num_small_angle_division', 5);
            cfgParam = Configurable.SetField(cfgParam, 'alphaVsTheta_margin', -0.0);
            
            obj.configParam = cfgParam;
        end
        
        function CheckConfig(obj) %#ok<MANU>
        end
    end
    
end