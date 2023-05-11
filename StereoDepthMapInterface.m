classdef StereoDepthMapInterface < Configurable
    
    methods (Abstract)
        CalcDisparityMap(obj);
    end
    
    methods
        % Constructor
        function obj = StereoDepthMapInterface(cfgParam)
            obj@Configurable(cfgParam);
        end
        
        function [depthMap, disparityMap1] = CalcDepthMap(obj, img1, img2, focLen, cenX1, cenX2, baseline, prevKeyMinDepth)
            if prevKeyMinDepth >= 0
                searchDisparityRef = round(focLen * baseline / prevKeyMinDepth);
            else
                searchDisparityRef = obj.configParam.num_disparities/2;
            end
            [disparityMap, validMap] = CalcDisparityMap(obj, img1, img2, searchDisparityRef);
            disparityMap0 = disparityMap;
            disparityMap(isnan(disparityMap)) = 0;
            depthMap = -1*ones(size(disparityMap));
            depthMap(validMap) = focLen * baseline ./ (disparityMap(validMap) + (cenX2 - cenX1));
            depthMap(depthMap < 0 | isinf(depthMap)) = -1;
            disparityMap1 = immultiply(disparityMap0, depthMap > 0);
            disparityMap1(isnan(disparityMap0)) = nan;
        end
    end
    
end