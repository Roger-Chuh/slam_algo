classdef Setting < Configurable
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    properties
        % Property1
        disparityBoundary        
        disparityIntervel
        disparityRange
        disparityBin
        disparityOrginRange
        %disparityCurrentRange
        
        angleIntervel
        angleBoundary
        angleRange
        angleBin
        angleOrginRange        
        
        tableSize
        
        wx
        wy

        disparitySigma
        disparityBeta
        gaussSigma
        gaussBeta
        
        winSize;
        angSize;
        angMove;
        
    end
    
    properties
        % Property1
       
    end
    
    methods (Static)
        function obj = Setting(cfgParam)
            obj@Configurable(cfgParam);
            
            obj.wx = obj.configParam.wx;
            obj.wy = obj.configParam.wy;
            obj.disparitySigma = obj.configParam.disparitySigma;
            obj.disparityBeta = obj.configParam.disparityBeta;
            obj.gaussSigma = obj.configParam.gaussSigma;
            obj.gaussBeta = obj.configParam.gaussBeta;
            obj.disparityBoundary = obj.configParam.disparityBoundary;
            obj.disparityIntervel = obj.configParam.disparityIntervel;
            obj.disparityRange = -obj.disparityBoundary:  obj.disparityIntervel : obj.disparityBoundary; 
            obj.configParam.disparityRange = obj.disparityRange;
            obj.disparityBin = Setting.getDisparityBin( obj.configParam);
            obj.configParam.disparityBin = obj.disparityBin;
            
            obj.angleBoundary = obj.configParam.angleBoundary;           
            obj.angleIntervel = obj.configParam.angleIntervel;
            obj.angleRange =  -obj.configParam.angleBoundary: obj.configParam.angleIntervel : obj.configParam.angleBoundary;      
            obj.configParam.angleRange = obj.angleRange;
            obj.angleBin = Setting.getAngleBin(obj.configParam);
            obj.configParam.angleBin = obj.angleBin;
            
            obj.disparityOrginRange = [];
            
            obj.tableSize =obj.disparityBin * obj.angleBin;
            obj.configParam.tableSize = obj.tableSize;
            
            
            obj.configParam.disparityOrginRange = obj.disparityOrginRange;
            obj.configParam.angleOrginRange = obj.angleOrginRange;
            
            obj.winSize = obj.configParam.winSize;
            obj.angSize = obj.configParam.angSize;
            obj.angMove = obj.configParam.angMove;
            
           
            
            
        end       
        
        function SettingDisparity(setDisparityBoundary,setDisparityIntervel)           
            obj.disparityBoundary = setDisparityBoundary;
            obj.disparityIntervel = setDisparityIntervel;
            obj.disparityRange = -setDisparityBoundary: setDisparityIntervel : setDisparityBoundary;    
           
        end
        function SettingAngle(setAngleBoundary,setAngleIntervel)            
            obj.angleBoundary = setAngleBoundary;           
            obj.angleIntervel = setAngleIntervel;
            obj.angleRange =  -setAngleBoundary: setAngleIntervel : setAngleBoundary;                        
        end
%     end
%     methods
        function disparityBin = getDisparityBin(obj)           
            disparityBin = size(obj.disparityRange,2);
        end
        function angleBin = getAngleBin(obj)           
            angleBin = size(obj.angleRange,2);
        end
        
      
    end
    
    methods
        function SetDefaultValue(obj, cfgParam)
            
            cfgParam = Configurable.SetField(cfgParam, 'wx', 2);
            cfgParam = Configurable.SetField(cfgParam, 'wy', 2);
            cfgParam = Configurable.SetField(cfgParam, 'disparityBoundary', 0.8);  % 0.8 % 0.4 0.6
            cfgParam = Configurable.SetField(cfgParam, 'disparityIntervel', 0.05); % 0.1 0.2 0.05
            cfgParam = Configurable.SetField(cfgParam, 'angleBoundary', 0.4);  % 0.2 0.4; 0.5   
            cfgParam = Configurable.SetField(cfgParam, 'angleIntervel', 0.02);  % 0.01 0.02; % 0.05
            cfgParam = Configurable.SetField(cfgParam, 'disparitySigma', 1);
            cfgParam = Configurable.SetField(cfgParam, 'disparityBeta', 10);
            cfgParam = Configurable.SetField(cfgParam, 'gaussSigma', 1);
            cfgParam = Configurable.SetField(cfgParam, 'gaussBeta', 10);
            cfgParam = Configurable.SetField(cfgParam, 'winSize',0.05);     
            cfgParam = Configurable.SetField(cfgParam, 'angSize',0.3);
            cfgParam = Configurable.SetField(cfgParam, 'angMove', 0.0);
            obj.configParam = cfgParam;
        end
        
        function CheckConfig(obj) %#ok<MANU>
        end
    end
    
    
end

