classdef FeaturePointTrackerInterface < Configurable
    
    methods (Abstract)
        TrackingImplement(obj);
        GetPredictMargin(obj);
    end
    
    methods
        function obj = FeaturePointTrackerInterface(cfgParam)
            obj@Configurable(cfgParam);
        end
        
        function [predPtList, inTrackFlag, topMargin] = TrackFeaturePoints(obj, prvImg, curImg, prvPtToTrack, predPtList, intrMat, pctB2C)
            global rot_y
            
            if 1
                [predPtList, trackStatus] = TrackingImplement(obj, prvImg, curImg, prvPtToTrack - 1, predPtList);
                trackStatus = true(size(trackStatus, 1), 1);
                predPtList = predPtList + 1;
            else
                [predPtList, trackStatus] = TrackingImplement(obj, prvImg, curImg, prvPtToTrack, predPtList);
            end
            
            if 1
                [topMargin, leftMargin, bottomMargin, rightMargin] = GetPredictMargin(obj);
                inBndFlag = predPtList(:, 1) >= leftMargin + 1 & ...
                    predPtList(:, 1) <= Cols(curImg) - rightMargin & ...
                    predPtList(:, 2) >= topMargin + 1 & ...
                    predPtList(:, 2) <= Rows(curImg) - bottomMargin;
                inTrackFlag = trackStatus & inBndFlag;
            else
                inTrackFlag = trackStatus>0;
            end
            if rot_y %
                if  ~isempty(predPtList(inTrackFlag, :))
                    inlierFlag = false(size(inTrackFlag));
                    inlierFlag(inTrackFlag) = ProximityDeoutlier(obj, prvPtToTrack(inTrackFlag, :), predPtList(inTrackFlag, :), intrMat, pctB2C);
                    inTrackFlag = inTrackFlag & inlierFlag;
                end
            end
        end
        
        function inlierFlag = ProximityDeoutlier(obj, ptIcs1, ptIcs2, intrMat, pctB2C)
            [curveMat, branch] = FeaturePointTrackerInterface.GetFeatureRotationCurveCcs(ptIcs1, intrMat, pctB2C);
            deltaY = FeaturePointTrackerInterface.EvaluateFeatureRotationCurve(ptIcs2(:,1)', curveMat, branch) - ptIcs2(:,2);
            
            inlierFlag = true(Rows(ptIcs1), 1);
            inlierFlag(branch == 1 & (deltaY <= -obj.configParam.point_tracking_noise_level_cam_rot | deltaY >= obj.configParam.point_tracking_max_dist_to_rot_curve)) = false;
            inlierFlag(branch == -1 & (deltaY >= obj.configParam.point_tracking_noise_level_cam_rot | deltaY <= -obj.configParam.point_tracking_max_dist_to_rot_curve)) = false;
            
            deltaX = ptIcs1(:, 1) - ptIcs2(:, 1);
            medDeltaX = median(deltaX(inlierFlag,1));
%             inlierFlag(inlierFlag & abs(medDeltaX) >= obj.configParam.point_tracking_rot_offset_x_thresh & abs(deltaX - medDeltaX) >= obj.configParam.point_tracking_rot_max_delta_x_factor*abs(medDeltaX)) = false;
        end
    end
    
    methods (Static)
        function [curveMat, branch, angPt2Ax] = GetFeatureRotationCurveCcs(ptIcs, intrMat, pctB2C)
            rotAxCcs = RotateAxisDst(pctB2C);
            angPt2Ax = acos(rotAxCcs'*Normalize(HomoCoord(ptIcs, 2) / intrMat', 2)')';
            
            numPts = Rows(ptIcs);
            curveMat = zeros(3, 3, numPts);
            branch = zeros(numPts,1);
            invIntrMat = inv(intrMat);
            err = zeros(numPts,1);
            rotMatB2C = R(pctB2C);
            for iPt = 1:numPts
                conicSurfBcs = [1,0,0; 0, -tan(angPt2Ax(iPt))^2, 0; 0, 0, 1];
                curveMat(:,:,iPt) = invIntrMat'*rotMatB2C * conicSurfBcs * rotMatB2C'*invIntrMat;
                a = curveMat(2,2,iPt);
                b  = 2*curveMat(1,2,iPt)*ptIcs(iPt,1) + 2*curveMat(2,3,iPt);
                c  = curveMat(1,1,iPt)*ptIcs(iPt,1)^2 + 2*curveMat(1,3,iPt)*ptIcs(iPt,1) + curveMat(3,3,iPt); 
                delta = sqrt(b^2 - 4*a*c);
                y1 = (-b + delta)/(a*2) - ptIcs(iPt,2);
                y2 = (-b - delta)/(a*2) - ptIcs(iPt,2);
                if (abs(y1) > abs(y2))
                    branch(iPt) = -1;
                    err(iPt) = abs(y2);
                else
                    branch(iPt) = 1;
                    err(iPt) = abs(y1);
                end
            %    err(iPt) = [pt(iPt,:),1] * curveMat(:,:,iPt) * [pt(iPt,:),1]';
            end
        end
        
        function y = EvaluateFeatureRotationCurve(featPtX, curveMat, branch)
            numCurves = size(curveMat, 3);
            if isscalar(featPtX)
                featPtX = featPtX(ones(numCurves, 1));
            end

            if iscolumn(featPtX)
                % one curve, multiple points to evaluate
                y = zeros(Rows(featPtX), numCurves);
                for iC = 1:numCurves
                    a = curveMat(2,2,iC);
                    b  = 2*curveMat(1,2,iC)*featPtX + 2*curveMat(2,3,iC);
                    c  = curveMat(1,1,iC)*featPtX.^2 + 2*curveMat(1,3,iC)*featPtX + curveMat(3,3,iC);
                    y(:,iC) = (-b + branch(iC)*sqrt(b.^2 - 4*a.*c))./(2*a);
                end

            elseif isrow(featPtX)
                % one curve per point
                assert(numel(featPtX) == numCurves, 'Evaluated in "one point per curve mode. But the point number is not equal to the curve number"');
                y = zeros(numCurves, 1);
                for iC = 1:numCurves
                    a = curveMat(2,2,iC);
                    b  = 2*curveMat(1,2,iC)*featPtX(iC) + 2*curveMat(2,3,iC);
                    c  = curveMat(1,1,iC)*featPtX(iC)^2 + 2*curveMat(1,3,iC)*featPtX(iC) + curveMat(3,3,iC);
                    y(iC) = (-b + branch(iC)*sqrt(b^2 - 4*a*c))/(2*a);
                end
            else
                error('x can not be a matrix');
            end
        end
        
    end
end