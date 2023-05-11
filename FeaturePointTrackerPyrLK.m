classdef FeaturePointTrackerPyrLK < FeaturePointTrackerInterface
    
    methods
        % Constructor
        function obj = FeaturePointTrackerPyrLK(cfgParam)
            obj@FeaturePointTrackerInterface(cfgParam);
        end
    end
    
    methods
        % Implementation of abstract methods in FeaturePointTrackerInterface
        function [predictedPt, status] = TrackingImplement(obj, prvImg, curImg, prvPtToTrack, predictedPt)
            % OpenCV implementation
                [predictedPt, status] = OcvTrackFeaturePointsPyrLK(prvImg, curImg, prvPtToTrack, ...
                [obj.configParam.win_width, obj.configParam.win_height], ...
                obj.configParam.max_level, obj.configParam.criteria_count, obj.configParam.criteria_eps, obj.configParam.min_eigen_threshold);

        end
        
        function [topMargin, leftMargin, bottomMargin, rightMargin] = GetPredictMargin(obj)
            topMargin = obj.configParam.top_margin;
            leftMargin = obj.configParam.left_margin;
            bottomMargin = obj.configParam.bottom_margin;
            rightMargin = obj.configParam.right_margin;
        end
    end
    
    methods
        % Implementation of abstract methods in Configurable base class
        function SetDefaultValue(obj, cfgParam)
            cfgParam = Configurable.SetField(cfgParam, 'point_tracking_noise_level_cam_rot', 5);
            cfgParam = Configurable.SetField(cfgParam, 'point_tracking_max_dist_to_rot_curve', 5);
            cfgParam = Configurable.SetField(cfgParam, 'point_tracking_rot_offset_x_thresh', 5);
            cfgParam = Configurable.SetField(cfgParam, 'point_tracking_rot_max_delta_x_factor', 5);
            
            obj.configParam = cfgParam;
        end
        
        function CheckConfig(obj) %#ok<MANU>
        end
    end
    
    
end