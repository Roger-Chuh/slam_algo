classdef FeaturePointExtractorInterface < Configurable
    
    methods (Abstract)
        ExtractFeaturePoints(obj);
    end
    
    methods
        % Constructor
        function obj = FeaturePointExtractorInterface(cfgParam)
            obj@Configurable(cfgParam);
        end
    end
    
end