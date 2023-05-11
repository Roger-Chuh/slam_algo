classdef FeaturePointManager < Configurable
    
    properties
        localTrace;  % Feature point trace between key frames
        localTrace2;
    end
    
    methods
        % Constructor
        function obj = FeaturePointManager(cfgParam)
            obj@Configurable(cfgParam);
            
            obj.localTrace.ptIcsX = [];
            obj.localTrace.ptIcsY = [];
            obj.localTrace.ptCcsZ = [];
            obj.localTrace.xGT = [];
            obj.localTrace.yGT = [];
            obj.localTrace.probZ = [];
            obj.localTrace.sampleZ = [];
            
            
            obj.localTrace2.ptIcsX = [];
            obj.localTrace2.ptIcsY = [];
            obj.localTrace2.ptCcsZ = [];
            obj.localTrace2.xGT = [];
            obj.localTrace2.yGT = [];
            obj.localTrace2.probZ = [];
            obj.localTrace2.sampleZ = [];
        end
        
        function n = LocalTraceLength(obj)
            n = Cols(obj.localTrace.ptIcsX);
        end
        function n = LocalTraceLength2(obj)
            n = Cols(obj.localTrace2.ptIcsX);
        end
        function b = IsLocalTraceEmpty(obj)
            b = LocalTraceLength(obj) == 0;
        end
        function b = IsLocalTraceEmpty2(obj)
            b = LocalTraceLength2(obj) == 0;
        end
        function [ptIcs, ptCcsZ, activeFeatInd] = GetActiveFeatures(obj, kfBasedFrmInd)
            if IsLocalTraceEmpty(obj)
                ptIcs = [];
                ptCcsZ = [];
                activeFeatInd = [];
                return;
            end
            
            if (~exist('kfBasedFrmInd', 'var') || strcmp(kfBasedFrmInd, 'last'))
                kfBasedFrmInd = LocalTraceLength(obj);
            end
            
            assert(kfBasedFrmInd <= LocalTraceLength(obj) && kfBasedFrmInd >= -LocalTraceLength(obj) + 1, 'frame index exceeds local trace length');
            if (kfBasedFrmInd <= 0)
                kfBasedFrmInd = LocalTraceLength(obj) + kfBasedFrmInd;
            end
            
            activeFeatInd = find(obj.localTrace.ptIcsX(:, kfBasedFrmInd) > 0);
            ptIcs = [obj.localTrace.ptIcsX(activeFeatInd, kfBasedFrmInd), obj.localTrace.ptIcsY(activeFeatInd, kfBasedFrmInd)];
            ptCcsZ = obj.localTrace.ptCcsZ(activeFeatInd);
        end
        function [ptIcs, ptCcsZ, activeFeatInd] = GetActiveFeatures2(obj, kfBasedFrmInd)
            if IsLocalTraceEmpty2(obj)
                ptIcs = [];
                ptCcsZ = [];
                activeFeatInd = [];
                return;
            end
            
            if (~exist('kfBasedFrmInd', 'var') || strcmp(kfBasedFrmInd, 'last'))
                kfBasedFrmInd = LocalTraceLength2(obj);
            end
            
            assert(kfBasedFrmInd <= LocalTraceLength2(obj) && kfBasedFrmInd >= -LocalTraceLength2(obj) + 1, 'frame index exceeds local trace length');
            if (kfBasedFrmInd <= 0)
                kfBasedFrmInd = LocalTraceLength2(obj) + kfBasedFrmInd;
            end
            
            activeFeatInd = find(obj.localTrace2.ptIcsX(:, kfBasedFrmInd) > 0);
            ptIcs = [obj.localTrace2.ptIcsX(activeFeatInd, kfBasedFrmInd), obj.localTrace2.ptIcsY(activeFeatInd, kfBasedFrmInd)];
            ptCcsZ = obj.localTrace2.ptCcsZ(activeFeatInd);
        end
        
        function [ptIcs, ptCcsZ] = GetFeatures(obj, kfBasedFrmInd, featInd)
            if IsLocalTraceEmpty(obj)
                ptIcs = [];
                ptCcsZ = [];
                return;
            end
                
            if strcmp(kfBasedFrmInd, 'last')
                kfBasedFrmInd = LocalTraceLength(obj);
            end
            
            assert(kfBasedFrmInd <= LocalTraceLength(obj) && kfBasedFrmInd >= -LocalTraceLength(obj) + 1, 'frame index exceeds local trace length');
            if (kfBasedFrmInd <= 0)
                kfBasedFrmInd = LocalTraceLength(obj) + kfBasedFrmInd;
            end
            
            if (~exist('featInd', 'var') || isempty(featInd))
                featInd = 1:NumLocalTraceFeatures(obj);
            end
            
            ptIcs = [obj.localTrace.ptIcsX(featInd, kfBasedFrmInd), obj.localTrace.ptIcsY(featInd, kfBasedFrmInd)];
            ptCcsZ = obj.localTrace.ptCcsZ(featInd);
        end
        function [ptIcs, ptCcsZ] = GetFeatures2(obj, kfBasedFrmInd, featInd)
            if IsLocalTraceEmpty2(obj)
                ptIcs = [];
                ptCcsZ = [];
                return;
            end
                
            if strcmp(kfBasedFrmInd, 'last')
                kfBasedFrmInd = LocalTraceLength2(obj);
            end
            
            assert(kfBasedFrmInd <= LocalTraceLength2(obj) && kfBasedFrmInd >= -LocalTraceLength2(obj) + 1, 'frame index exceeds local trace length');
            if (kfBasedFrmInd <= 0)
                kfBasedFrmInd = LocalTraceLength2(obj) + kfBasedFrmInd;
            end
            
            if (~exist('featInd', 'var') || isempty(featInd))
                featInd = 1:NumLocalTraceFeatures2(obj);
            end
            
            ptIcs = [obj.localTrace2.ptIcsX(featInd, kfBasedFrmInd), obj.localTrace2.ptIcsY(featInd, kfBasedFrmInd)];
            ptCcsZ = obj.localTrace2.ptCcsZ(featInd);
        end
        function [pt1Ics, pt2Ics, pt1CcsZ] = GetFeaturePairs(obj, kfBasedFrmInd1, kfBasedFrmInd2)
            % kfBasedFrmInd1: key frame based start frame index of
            % predicting range. It can be zero or negative integer
            % kfBasedFrmInd2: key frame based end frame index of predicting
            % range. It can be zero or negative integer
            % Note the two indices must be both positive or non-positive,
            % and kfBasedFrmInd1 should be smaller than kfBasedFrmInd2 in
            % value.
            if ischar(kfBasedFrmInd1)
                if strcmp(kfBasedFrmInd1, 'key')
                    kfBasedFrmInd1 = 1;
                elseif strcmp(kfBasedFrmInd1, 'prev')
                    kfBasedFrmInd1 = LocalTraceLength(obj) - 1;
                elseif strcmp(kfBasedFrmInd1, 'last')
                    kfBasedFrmInd1 = LocalTraceLength(obj);
                else
                    error('Unrecognized kfBasedFrmInd1:name %s', kfBasedFrmInd1);
                end
            end
            
            if ischar(kfBasedFrmInd2)
                if strcmp(kfBasedFrmInd2, 'key')
                    kfBasedFrmInd2 = 1;
                elseif strcmp(kfBasedFrmInd2, 'prev')
                    kfBasedFrmInd2 = LocalTraceLength(obj) - 1;
                elseif strcmp(kfBasedFrmInd2, 'last')
                    kfBasedFrmInd2 = LocalTraceLength(obj);
                else
                    error('Unrecognized kfBasedFrmInd1:name %s', kfBasedFrmInd1);
                end
            end
            
            assert((kfBasedFrmInd1 > 0 &&  kfBasedFrmInd2 > 0) || (kfBasedFrmInd1 <= 0 &&  kfBasedFrmInd2 <= 0), 'The input frame indices must be both positive or non-positive');
            assert(kfBasedFrmInd1 < kfBasedFrmInd2, 'the frame of index1 must be ahead of the frame of index2 in time');
            
            [pt2Ics, ~, activeFeatInd] = GetActiveFeatures(obj, kfBasedFrmInd2);
            [pt1Ics, pt1CcsZ] = GetFeatures(obj, kfBasedFrmInd1, activeFeatInd);
        end
        function [pt1Ics, pt2Ics, pt1CcsZ] = GetFeaturePairs2(obj, kfBasedFrmInd1, kfBasedFrmInd2)
            % kfBasedFrmInd1: key frame based start frame index of
            % predicting range. It can be zero or negative integer
            % kfBasedFrmInd2: key frame based end frame index of predicting
            % range. It can be zero or negative integer
            % Note the two indices must be both positive or non-positive,
            % and kfBasedFrmInd1 should be smaller than kfBasedFrmInd2 in
            % value.
            if ischar(kfBasedFrmInd1)
                if strcmp(kfBasedFrmInd1, 'key')
                    kfBasedFrmInd1 = 1;
                elseif strcmp(kfBasedFrmInd1, 'prev')
                    kfBasedFrmInd1 = LocalTraceLength2(obj) - 1;
                elseif strcmp(kfBasedFrmInd1, 'last')
                    kfBasedFrmInd1 = LocalTraceLength2(obj);
                else
                    error('Unrecognized kfBasedFrmInd1:name %s', kfBasedFrmInd1);
                end
            end
            
            if ischar(kfBasedFrmInd2)
                if strcmp(kfBasedFrmInd2, 'key')
                    kfBasedFrmInd2 = 1;
                elseif strcmp(kfBasedFrmInd2, 'prev')
                    kfBasedFrmInd2 = LocalTraceLength2(obj) - 1;
                elseif strcmp(kfBasedFrmInd2, 'last')
                    kfBasedFrmInd2 = LocalTraceLength2(obj);
                else
                    error('Unrecognized kfBasedFrmInd1:name %s', kfBasedFrmInd1);
                end
            end
            
            assert((kfBasedFrmInd1 > 0 &&  kfBasedFrmInd2 > 0) || (kfBasedFrmInd1 <= 0 &&  kfBasedFrmInd2 <= 0), 'The input frame indices must be both positive or non-positive');
            assert(kfBasedFrmInd1 < kfBasedFrmInd2, 'the frame of index1 must be ahead of the frame of index2 in time');
            
            [pt2Ics, ~, activeFeatInd] = GetActiveFeatures2(obj, kfBasedFrmInd2);
            [pt1Ics, pt1CcsZ] = GetFeatures2(obj, kfBasedFrmInd1, activeFeatInd);
        end
        function n = NumActiveFeatures(obj, kfBasedFrmInd)
            if ~exist('kfBasedFrmInd', 'var')
                kfBasedFrmInd = LocalTraceLength(obj);
            end
            
            if strcmp(kfBasedFrmInd, 'key')
                n = sum(obj.localTrace.ptIcsX(:, 1) > 0);
                return;
            end
            
            if (IsLocalTraceEmpty(obj))
                n = 0;
                return;
            end
            
            assert((kfBasedFrmInd <= 0 && abs(kfBasedFrmInd) < LocalTraceLength(obj)) || (kfBasedFrmInd > 0 && kfBasedFrmInd <= LocalTraceLength(obj)), 'Frame index exceeds frame number');
            if (kfBasedFrmInd <= 0)
                kfBasedFrmInd = LocalTraceLength(obj) + kfBasedFrmInd;
            end
            
            n = sum(obj.localTrace.ptIcsX(:, kfBasedFrmInd) > 0);
        end
        function n = NumActiveFeatures2(obj, kfBasedFrmInd)
            if ~exist('kfBasedFrmInd', 'var')
                kfBasedFrmInd = LocalTraceLength2(obj);
            end
            
            if strcmp(kfBasedFrmInd, 'key')
                n = sum(obj.localTrace2.ptIcsX(:, 1) > 0);
                return;
            end
            
            if (IsLocalTraceEmpty2(obj))
                n = 0;
                return;
            end
            
            assert((kfBasedFrmInd <= 0 && abs(kfBasedFrmInd) < LocalTraceLength2(obj)) || (kfBasedFrmInd > 0 && kfBasedFrmInd <= LocalTraceLength2(obj)), 'Frame index exceeds frame number');
            if (kfBasedFrmInd <= 0)
                kfBasedFrmInd = LocalTraceLength2(obj) + kfBasedFrmInd;
            end
            
            n = sum(obj.localTrace2.ptIcsX(:, kfBasedFrmInd) > 0);
        end
        
        function n = NumLocalTraceFeatures(obj)
            n = Rows(obj.localTrace.ptIcsX);
        end
        function n = NumLocalTraceFeatures2(obj)
            n = Rows(obj.localTrace2.ptIcsX);
        end
        function dropRate = FeatureNumDropRate(obj)
            if (IsLocalTraceEmpty(obj) || LocalTraceLength(obj) == 1 || NumActiveFeatures(obj, -1) < NumActiveFeatures(obj, 0) || NumActiveFeatures(obj, -1) == 0)
                dropRate = 0;
            else
                
                dropRate = (NumActiveFeatures(obj, 'key') - NumActiveFeatures(obj, 0))/NumActiveFeatures(obj, 'key');
            end
        end
        
        function ResetLocalTrace(obj, ptIcs, ptCcsZ, pctCcsP2C, intrMat1, intrMat2)
            if IsLocalTraceEmpty(obj)
                obj.localTrace.ptIcsX = ptIcs(:, 1);
                obj.localTrace.ptIcsY = ptIcs(:, 2);
                obj.localTrace.ptCcsZ = ptCcsZ;  
            else
                % Combine new feature points with orignal ones
                [origPtIcs, ~, activeFeatInd] = GetActiveFeatures(obj);
                if isempty(origPtIcs)
                    obj.localTrace.ptIcsX = ptIcs(:, 1);
                    obj.localTrace.ptIcsY = ptIcs(:, 2);
                    obj.localTrace.ptCcsZ = ptCcsZ;
                else
                    nearestIdx1 = knnsearch(origPtIcs, ptIcs, 'NSMethod', 'kdtree');
                    samePtFlag1 = VecNorm(ptIcs - origPtIcs(nearestIdx1, :), 2) <= obj.configParam.radius_thresh_for_combine;
                    nearestIdx2 = knnsearch(ptIcs, origPtIcs, 'NSMethod', 'kdtree');
                    samePtFlag2 = VecNorm(origPtIcs - ptIcs(nearestIdx2, :), 2) <= obj.configParam.radius_thresh_for_combine;
                    newPtFlag = true(Rows(ptIcs),1);
                    if (isempty(pctCcsP2C) || LocalTraceLength(obj) < 2)||~all(samePtFlag1)||~all(samePtFlag2)
                        newPtFlag(nearestIdx2(samePtFlag2)) = false;
                    else
                        % Use epipolar geometry to check which should be true
                        % feature point locations, the original feature points
                        % or the new feature points possibly coincide with
                        % original ones.
                        
                        fmat = FundamentalMatrix(pctCcsP2C,intrMat1, intrMat2);
                        ptIcsPreOrig = GetFeatures(obj, -1, activeFeatInd(samePtFlag2));
                        assert(all(ptIcsPreOrig(2,:) > 0), 'The tracked back points of active feature points must be valid');
                        epl = fmat * HomoCoord(ptIcsPreOrig',1);
                        dist2EplCand = dot(HomoCoord(ptIcs(samePtFlag1, :)',1), epl)./VecNorm(epl(1:2, :));
                        dist2EplOrig = dot(HomoCoord(origPtIcs(samePtFlag2, :)',1), epl)./VecNorm(epl(1:2, :));
                        newPtFlag(nearestIdx1(samePtFlag1(dist2EplCand < dist2EplOrig))) = false;
                    end
                    combinedPtIcsXY = ptIcs(~newPtFlag, :);
                    combinedPtCcsZ = ptCcsZ(~newPtFlag);
                    newPtIcs = ptIcs(newPtFlag, :);
                    newPtCcsZ = ptCcsZ(newPtFlag);
                    obj.localTrace.ptIcsX = [combinedPtIcsXY(:, 1); newPtIcs(:, 1)];
                    obj.localTrace.ptIcsY = [combinedPtIcsXY(:, 2); newPtIcs(:, 2)];
                    obj.localTrace.ptCcsZ = [combinedPtCcsZ; newPtCcsZ];
                end
            end
%             inlierId = find(obj.localTrace.ptIcsX(:,end) ~= -1);
%             
%              DispRng = repmat(dispList(inlierId,:),1,length(disparityRng)) + repmat(disparityRng,length(inlierId),1);
%                         for jk = 1 : length(inlierId)
%                              [ProbZ(jk,:), ZZVec1(jk,:)] = probDensity(dispList(inlierId(jk),:), obj.configParam.disparity_sigma, DispRng(jk,:),obj.configParam.disparity_sample_interval, 'disparity');
%                         end
            obj.localTrace.probZ = [];
            obj.localTrace.sampleZ = [];
        end
        
        function ResetLocalTrace2(obj, ptIcs, ptCcsZ, pctCcsP2C, intrMat1, intrMat2)
            if IsLocalTraceEmpty2(obj)
                obj.localTrace2.ptIcsX = ptIcs(:, 1);
                obj.localTrace2.ptIcsY = ptIcs(:, 2);
                obj.localTrace2.ptCcsZ = ptCcsZ;  
            else
                % Combine new feature points with orignal ones
                [origPtIcs, ~, activeFeatInd] = GetActiveFeatures2(obj);
                if isempty(origPtIcs)
                    obj.localTrace2.ptIcsX = ptIcs(:, 1);
                    obj.localTrace2.ptIcsY = ptIcs(:, 2);
                    obj.localTrace2.ptCcsZ = ptCcsZ;
                else
                    nearestIdx1 = knnsearch(origPtIcs, ptIcs, 'NSMethod', 'kdtree');
                    samePtFlag1 = VecNorm(ptIcs - origPtIcs(nearestIdx1, :), 2) <= obj.configParam.radius_thresh_for_combine;
                    nearestIdx2 = knnsearch(ptIcs, origPtIcs, 'NSMethod', 'kdtree');
                    samePtFlag2 = VecNorm(origPtIcs - ptIcs(nearestIdx2, :), 2) <= obj.configParam.radius_thresh_for_combine;
                    newPtFlag = true(Rows(ptIcs),1);
                    if (isempty(pctCcsP2C) || LocalTraceLength2(obj) < 2)||~all(samePtFlag1)||~all(samePtFlag2)
                        newPtFlag(nearestIdx2(samePtFlag2)) = false;
                    else
                        % Use epipolar geometry to check which should be true
                        % feature point locations, the original feature points
                        % or the new feature points possibly coincide with
                        % original ones.
                        
                        fmat = FundamentalMatrix(pctCcsP2C,intrMat1, intrMat2);
                        ptIcsPreOrig = GetFeatures2(obj, -1, activeFeatInd(samePtFlag2));
                        assert(all(ptIcsPreOrig(2,:) > 0), 'The tracked back points of active feature points must be valid');
                        epl = fmat * HomoCoord(ptIcsPreOrig',1);
                        dist2EplCand = dot(HomoCoord(ptIcs(samePtFlag1, :)',1), epl)./VecNorm(epl(1:2, :));
                        dist2EplOrig = dot(HomoCoord(origPtIcs(samePtFlag2, :)',1), epl)./VecNorm(epl(1:2, :));
                        newPtFlag(nearestIdx1(samePtFlag1(dist2EplCand < dist2EplOrig))) = false;
                    end
                    combinedPtIcsXY = ptIcs(~newPtFlag, :);
                    combinedPtCcsZ = ptCcsZ(~newPtFlag);
                    newPtIcs = ptIcs(newPtFlag, :);
                    newPtCcsZ = ptCcsZ(newPtFlag);
                    obj.localTrace2.ptIcsX = [combinedPtIcsXY(:, 1); newPtIcs(:, 1)];
                    obj.localTrace2.ptIcsY = [combinedPtIcsXY(:, 2); newPtIcs(:, 2)];
                    obj.localTrace2.ptCcsZ = [combinedPtCcsZ; newPtCcsZ];
                end
            end
%             inlierId = find(obj.localTrace.ptIcsX(:,end) ~= -1);
%             
%              DispRng = repmat(dispList(inlierId,:),1,length(disparityRng)) + repmat(disparityRng,length(inlierId),1);
%                         for jk = 1 : length(inlierId)
%                              [ProbZ(jk,:), ZZVec1(jk,:)] = probDensity(dispList(inlierId(jk),:), obj.configParam.disparity_sigma, DispRng(jk,:),obj.configParam.disparity_sample_interval, 'disparity');
%                         end
            obj.localTrace2.probZ = [];
            obj.localTrace2.sampleZ = [];
        end
        
        
        
        function ExtendLocalTrace(obj, ptIcs, validFlag, switchFlag)
            assert(Rows(ptIcs) == NumActiveFeatures(obj), 'The number of points to add should be equal to the number of active feature points');
            assert(Rows(ptIcs) == length(validFlag), 'The validFlag vector should have the same size as the point list to add');
            assert(~IsLocalTraceEmpty(obj), 'Logical error. When to add feature points to local trace, the local trace should not be empty');
            activeInd = find(obj.localTrace.ptIcsX(:, end) > 0);
            if ~exist('switchFlag', 'var')
                switchFlag = 'extend';
            end
            if strcmp(switchFlag, 'updateLast')
                obj.localTrace.ptIcsX(activeInd(validFlag), end) = ptIcs(validFlag, 1);
                obj.localTrace.ptIcsY(activeInd(validFlag), end) = ptIcs(validFlag, 2);
                obj.localTrace.ptIcsX(activeInd(~validFlag), end) = -1;
                obj.localTrace.ptIcsY(activeInd(~validFlag), end) = -1;
            elseif strcmp(switchFlag, 'extend')
                obj.localTrace.ptIcsX = [obj.localTrace.ptIcsX, -1*ones(Rows(obj.localTrace.ptIcsX), 1)];
                obj.localTrace.ptIcsX(activeInd(validFlag), end) = ptIcs(validFlag, 1);
                obj.localTrace.ptIcsY = [obj.localTrace.ptIcsY, -1*ones(Rows(obj.localTrace.ptIcsY), 1)];
                obj.localTrace.ptIcsY(activeInd(validFlag), end) = ptIcs(validFlag, 2);
            else
                assert(false,'switchflag is wrong!')
            end
        end
        function ExtendLocalTrace2(obj, ptIcs, validFlag, switchFlag)
            assert(Rows(ptIcs) == NumActiveFeatures2(obj), 'The number of points to add should be equal to the number of active feature points');
            assert(Rows(ptIcs) == length(validFlag), 'The validFlag vector should have the same size as the point list to add');
            assert(~IsLocalTraceEmpty2(obj), 'Logical error. When to add feature points to local trace, the local trace should not be empty');
            activeInd = find(obj.localTrace2.ptIcsX(:, end) > 0);
            if ~exist('switchFlag', 'var')
                switchFlag = 'extend';
            end
            if strcmp(switchFlag, 'updateLast')
                obj.localTrace2.ptIcsX(activeInd(validFlag), end) = ptIcs(validFlag, 1);
                obj.localTrace2.ptIcsY(activeInd(validFlag), end) = ptIcs(validFlag, 2);
                obj.localTrace2.ptIcsX(activeInd(~validFlag), end) = -1;
                obj.localTrace2.ptIcsY(activeInd(~validFlag), end) = -1;
            elseif strcmp(switchFlag, 'extend')
                obj.localTrace2.ptIcsX = [obj.localTrace2.ptIcsX, -1*ones(Rows(obj.localTrace2.ptIcsX), 1)];
                obj.localTrace2.ptIcsX(activeInd(validFlag), end) = ptIcs(validFlag, 1);
                obj.localTrace2.ptIcsY = [obj.localTrace2.ptIcsY, -1*ones(Rows(obj.localTrace2.ptIcsY), 1)];
                obj.localTrace2.ptIcsY(activeInd(validFlag), end) = ptIcs(validFlag, 2);
            else
                assert(false,'switchflag is wrong!')
            end
        end
        
        function SaveInLier(obj, kfBasedFrmInd, activeFeatInd, validFlag)
%             assert(length(validFlag) == NumActiveFeatures(obj), 'The number of points to add should be equal to the number of active feature points');
            assert(~IsLocalTraceEmpty(obj), 'Logical error. When to add feature points to local trace, the local trace should not be empty');
            if ischar(kfBasedFrmInd)
                if strcmp(kfBasedFrmInd, 'key')
                    kfBasedFrmInd = 1;
                elseif strcmp(kfBasedFrmInd, 'prev')
                    kfBasedFrmInd = LocalTraceLength(obj) - 1;
                elseif strcmp(kfBasedFrmInd, 'last')
                    kfBasedFrmInd = LocalTraceLength(obj);
                else
                    error('Unrecognized kfBasedFrmInd1:name %s', kfBasedFrmInd);
                end
            end
            assert(kfBasedFrmInd <= LocalTraceLength(obj) && kfBasedFrmInd >= -LocalTraceLength(obj) + 1, 'frame index exceeds local trace length');
            if (kfBasedFrmInd <= 0)
                kfBasedFrmInd = LocalTraceLength(obj) + kfBasedFrmInd;
            end
            obj.localTrace.ptIcsX(activeFeatInd(~validFlag), kfBasedFrmInd) = -1;
            obj.localTrace.ptIcsY(activeFeatInd(~validFlag), kfBasedFrmInd) = -1;
        end
        function SaveInLier2(obj, kfBasedFrmInd, activeFeatInd, validFlag)
%             assert(length(validFlag) == NumActiveFeatures(obj), 'The number of points to add should be equal to the number of active feature points');
            assert(~IsLocalTraceEmpty2(obj), 'Logical error. When to add feature points to local trace, the local trace should not be empty');
            if ischar(kfBasedFrmInd)
                if strcmp(kfBasedFrmInd, 'key')
                    kfBasedFrmInd = 1;
                elseif strcmp(kfBasedFrmInd, 'prev')
                    kfBasedFrmInd = LocalTraceLength2(obj) - 1;
                elseif strcmp(kfBasedFrmInd, 'last')
                    kfBasedFrmInd = LocalTraceLength2(obj);
                else
                    error('Unrecognized kfBasedFrmInd1:name %s', kfBasedFrmInd);
                end
            end
            assert(kfBasedFrmInd <= LocalTraceLength2(obj) && kfBasedFrmInd >= -LocalTraceLength2(obj) + 1, 'frame index exceeds local trace length');
            if (kfBasedFrmInd <= 0)
                kfBasedFrmInd = LocalTraceLength2(obj) + kfBasedFrmInd;
            end
            obj.localTrace2.ptIcsX(activeFeatInd(~validFlag), kfBasedFrmInd) = -1;
            obj.localTrace2.ptIcsY(activeFeatInd(~validFlag), kfBasedFrmInd) = -1;
        end
        function UpdateLocalTracePlot(obj, img)
            imshow(img);
            if IsLocalTraceEmpty(obj)
                return;
            end
            
            hold on;
            [~, ~, activeInd] = GetActiveFeatures(obj);
            localTraceX = obj.localTrace.ptIcsX(activeInd, :);
            localTraceY = obj.localTrace.ptIcsY(activeInd, :);
%             assert(all(all(localTraceX > 0 & localTraceY > 0)), 'The back trace of active features must be valid');
            if (LocalTraceLength(obj) >= 2)
                plot(localTraceX', localTraceY', 'g-');
                
                loseTrackingFlag = true(NumLocalTraceFeatures(obj), 1);
                loseTrackingFlag(activeInd) = false;
                ptIcsPrev = GetFeatures(obj, -1);
                loseTrackingFlag(ptIcsPrev(:,1) < 0) = false;
                if any(loseTrackingFlag)
                    plot(ptIcsPrev(loseTrackingFlag, 1), ptIcsPrev(loseTrackingFlag, 2), 'yx');
                end
            end
            plot(localTraceX(:, end), localTraceY(:, end), 'ro');
            hold off;
        end
        
%         function UpdateLocalTracePlot2(obj, img)
% %             imshow(img);
%             if IsLocalTraceEmpty(obj)
%                 return;
%             end
%             
%             hold on;
%             [~, ~, activeInd] = GetActiveFeatures(obj);
%             localTraceX = obj.localTrace.ptIcsX(activeInd, :);
%             localTraceY = obj.localTrace.ptIcsY(activeInd, :);
% %             assert(all(all(localTraceX > 0 & localTraceY > 0)), 'The back trace of active features must be valid');
%             if (LocalTraceLength(obj) >= 2)
%                 plot(localTraceX', localTraceY', 'c-');
%                 
%                 loseTrackingFlag = true(NumLocalTraceFeatures(obj), 1);
%                 loseTrackingFlag(activeInd) = false;
%                 ptIcsPrev = GetFeatures(obj, -1);
%                 loseTrackingFlag(ptIcsPrev(:,1) < 0) = false;
%                 if any(loseTrackingFlag)
%                     plot(ptIcsPrev(loseTrackingFlag, 1), ptIcsPrev(loseTrackingFlag, 2), 'kx');
%                 end
%             end
%             plot(localTraceX(:, end), localTraceY(:, end), 'bo');
%             hold off;
%         end
        
        function CheckDataConsistency(obj)
            assert(Rows(obj.localTrace.ptIcsX) == Rows(obj.localTrace.ptIcsY) && Rows(obj.localTrace.ptIcsX) == Rows(obj.localTrace.ptCcsZ), 'Inconsistent internal data');
            if ~isempty(obj.localTrace.ptIcsX)
                assert(all((obj.localTrace.ptIcsX(:, end) == -1) == (obj.localTrace.ptIcsY(:, end) == -1)))
            end
        end
        function CheckDataConsistency2(obj)
            assert(Rows(obj.localTrace2.ptIcsX) == Rows(obj.localTrace2.ptIcsY) && Rows(obj.localTrace2.ptIcsX) == Rows(obj.localTrace2.ptCcsZ), 'Inconsistent internal data');
            if ~isempty(obj.localTrace2.ptIcsX)
                assert(all((obj.localTrace2.ptIcsX(:, end) == -1) == (obj.localTrace2.ptIcsY(:, end) == -1)))
            end
        end
    end
    
    methods
        % Implementation of abstract methods in Configurable base class
        function SetDefaultValue(obj, cfgParam)
            cfgParam = Configurable.SetField(cfgParam, 'radius_thresh_for_combine', 1);
            
            obj.configParam = cfgParam;
        end
        
        function CheckConfig(obj) %#ok<MANU>
        end
    end
    
end