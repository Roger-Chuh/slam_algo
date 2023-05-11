classdef FeaturePointExtractorOrb < FeaturePointExtractorInterface
    
    methods
        % Constructor
        function obj = FeaturePointExtractorOrb(cfgParam)
            obj@FeaturePointExtractorInterface(cfgParam);
        end
    end
    
    methods
        % Implementation of abstract methods in FeaturePointExtractorInterface
        function [featPtList, descList] = ExtractFeaturePoints(obj, img, mask)
            
            if (size(img, 3) == 3)
                img = rgb2gray(img);
            else
                assert(size(img, 3) == 1, 'Wrong image format. The channel number of image must be 1 or 3');
            end
            
            featurePointDetectionMothod = obj.configParam.feat_pt_detection_method;
            switch featurePointDetectionMothod
                case 'ocv_orb'
                    maxNumFeatPts = obj.configParam.max_num_feature_points;
                    scaleFactor = obj.configParam.scale_factor;
                    numScaleLvls = obj.configParam.num_scale_levels;
                    initFastThresh = obj.configParam.initial_fast_threshold;
                    minFastThresh = obj.configParam.min_fast_threshold;
                    [ptIcs, descList] = OcvExtractFeaturePointOrb(img, uint8(mask), maxNumFeatPts, scaleFactor, numScaleLvls, initFastThresh, minFastThresh);
                    featPtList = ptIcs.Location;
                case 'matlab_fast'
                    descList = [];
                    %                     ptIcs = detectFASTFeatures(img,'MinQuality',0.05,'MinContrast',0.04);
                    %                     ptIcs = detectFASTFeatures(img,'MinQuality',0.06,'MinContrast',0.06);
                    ptIcs = detectFASTFeatures(img,'MinQuality',0.02,'MinContrast',0.02);
                    %                     ptIcs = detectFASTFeatures(img,'MinQuality',0.03,'MinContrast',0.03);
                    %                     ptIcs = detectFASTFeatures(img,'MinQuality',0.01,'MinContrast',0.01);
                    featPtList = ptIcs.Location;
                    imgSize = size(img);
                    
                    %                     inId = find(featPtList(:,1) > 10 & featPtList(:,2) > 10 & featPtList(:,1) < imgSize(2) - 10 & featPtList(:,2) < imgSize(1) - 10);
                    if imgSize(1) == 240
                        inId = find(featPtList(:,1) > 10 & featPtList(:,2) > 10 & featPtList(:,1) < imgSize(2) - 10 & featPtList(:,2) < 128);
                    else
                        inId = find(featPtList(:,1) > 10 & featPtList(:,2) > 10 & featPtList(:,1) < imgSize(2) - 10 & featPtList(:,2) < imgSize(1)/2 - 10 );
                    end
                    
                    featPtList = featPtList(inId,:);
                    
                    asdbkn = 1;
                    
                case 'matlab_harris'
                    descList = [];
                    ptIcs = detectHarrisFeatures(img,'MinQuality',0.01,'FilterSize',5);
                    featPtList = ptIcs.Location;
                case 'matlab_SURF'
                    descList = [];
                    ptIcs = detectSURFFeatures(img,'MetricThreshold',1000, 'NumOctaves',3, 'NumScaleLevels',4);
                    featPtList = ptIcs.Location;
                otherwise                    
                    assert(false,'no such feature piont detection method')                    
            end
%             imshow(img); hold on;
%             for i = 1:length(featPtList)
%                 plot(featPtList(i,1),featPtList(i,2),'+r');
%             end
        end
    end
    
    methods
        % Implementation of abstract methods in Configurable base class
        function SetDefaultValue(obj, cfgParam)
            cfgParam = Configurable.SetField(cfgParam, 'feat_pt_detection_method', 'matlab_fast');% unit: mm            
            obj.configParam = cfgParam;
        end
        
        function CheckConfig(obj) %#ok<MANU>
        end
    end
    
end