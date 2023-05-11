classdef StereoDepthMapSgbm < StereoDepthMapInterface
    
    methods
        % Constructor
        function obj = StereoDepthMapSgbm(cfgParam)
            obj@StereoDepthMapInterface(cfgParam);
        end
    end
    
    methods
        % Implementation of abstract methods in StereoDepthMapInterface
        function [disparityMap, validMap] = CalcDisparityMap(obj, img1, img2, searchDisparityRef)
            assert(all(size(img1) == size(img2)), 'Input image pair must be of the szme size');
            assert(any(size(img1, 3) == [1,3]),  'Invalid input image format');
            if (size(img1, 3) == 3)
                img1 = rgb2gray(img1);
                img2 = rgb2gray(img2);
            end
            
            %             disparityRange = [searchDisparityRef - obj.configParam.num_disparities/2, searchDisparityRef + obj.configParam.num_disparities/2];
            %             if disparityRange(1) < 0
            %                 disparityRange(2) = obj.configParam.num_disparities;
            %                 disparityRange(1) = 0;
            %             end
            disparityRange(2) = obj.configParam.num_disparities;
            disparityRange(1) = 0;
            
            
            areaV = 0.1;
            
            
            if 1
                areaV = 0.05;
                areaH = 0.04;
            else
                areaV = 0.12;
                areaH = 0.06;
            end
            areaRatio = 0.05; 0.01;
            
            
            % Matlab builtin implementation
            if 1
                if 1
                    disparityMap = disparity(img1, img2, ...
                        'DisparityRange', disparityRange, ...
                        'BlockSize', obj.configParam.block_size, ...
                        'UniquenessThreshold', obj.configParam.uniqueness_ratio, ...
                        'DistanceThreshold', obj.configParam.disp12_max_diff);
                    
                else
                    
                    scale = 4;
                    disparityMap = disparity(imresize(img1,[size(img1,1) scale*size(img1,2)]), imresize(img2,[size(img2,1) scale*size(img2,2)]), ...
                        'DisparityRange', disparityRange.*scale, ...
                        'BlockSize', obj.configParam.block_size, ...
                        'UniquenessThreshold', obj.configParam.uniqueness_ratio, ...
                        'DistanceThreshold', obj.configParam.disp12_max_diff);
                    
                    disparityMap = imresize(disparityMap,size(img1))./scale;
                    
                    if 0
                        disparityMap = disparityMap + 0.4;
                    end
                    
                    dskdsgjk = 1;
                    
                end
                % % %                 disparityMap = disparityMap + 0.1;
                
                if 0
                    validMap = disparityMap ~= -realmax('single');
                else
                    validMap = (disparityMap > 0.1 & disparityMap < disparityRange(2));
%                     validMap = disparityMap >= 0.2 & disparityMap < disparityRange(2);
%                     validMap = disparityMap >= 0.6  & disparityMap < disparityRange(2);
%                     validMap = disparityMap >= 2  & disparityMap < disparityRange(2);
                end
                
                      disparityMap(1:round(areaV.*size(disparityMap,1)),:) = nan;
                      disparityMap(end-round(areaV.*size(disparityMap,1)):end,:) = nan;
                      disparityMap(:,1:round(areaH.*size(disparityMap,2))) = nan;
                      disparityMap(:, end-round(areaH.*size(disparityMap,2)):end) = nan;
                      
                      
                      validMap(1:round(areaV.*size(disparityMap,1)),:) = 0;
                      validMap(end-round(areaV.*size(disparityMap,1)):end,:) = 0;
                      validMap(:,1:round(areaH.*size(disparityMap,2))) = 0;
                      validMap(:, end-round(areaH.*size(disparityMap,2)):end) = 0;
                      
%                       validMap(isnan)
                      
% % %                       [L, LL] = bwlabel(disparityMap > 0, 8);
% % %                       
% % %                       for i = 1 : LL
% % %                          bwTemp =  L == (i);
% % %                          area(i) = sum(sum(bwTemp));
% % %                       end
% % %                       
% % %                       
% % %                       validLabel  = find(area > areaRatio*max(area));
% % %                       
% % %                       validArea = false(size(disparityMap));
% % %                        for i = 1 : length(validLabel)
% % %                          validTemp =  find(L == validLabel(i));
% % %                          validArea(validTemp) = true;
% % %                        end
% % %                       
% % %                       validMap(~validArea) = false;
% % %                       disparityMap(~validArea) = 0;
                      
                saghfjk = 1;
                
            else
                try
%                     [disparityMap, validMap] = nanhu_depth(img1, img2, 0, 8, 7, 11, 21, 41, 2, 6);
                    [disparityMap, validMap] = nanhu_depth_upsample(img1, img2, 0, 8, 7, 11, 21, 41, 2, 6,4);
                    
                catch
                    agnkj = 1;
                end
            end
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