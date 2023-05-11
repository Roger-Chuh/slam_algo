classdef MonoCameraInterface < handle
    
    properties
        mapX    % X components of map that mapping from output of one camera model to that of another model 
        mapY    % Y components of map that mapping from output of one camera model to that of another model
    end
    
    methods (Abstract)
        GetPinholeIntrMat(obj)     % Get intrinsic matrix of pinhole camera model
        GetPinholeFocLenVec(obj)   % Get focal length vector ([focLenX; focLenY]) of pinhole camera model
        GetPinholePrincpPt(obj)    % Get principle point ([x; y]) of pinhole camera model
        GetPinholeSkew(obj)        % Get skew factor of pinhole camera model
        
        GetIntrMat(obj)         % Get intrinsic matrix
        GetFocLenVec(obj)       % Get focal length vector ([focLenX; focLenY])
        GetPrincpPt(obj)        % Get principle point ([x; y])
        GetSkew(obj)            % Get skew factor
        GetDistCoeff(obj)       % Get distortion coefficients
        
        LoadParam(obj)          % Load camera parameters from file
    end
    
    methods
        function isoFocLen = GetPinholeFocLen(obj, scaleLevel)
            if ~exist('scaleLevel', 'var')
                scaleLevel = 0;
            end
            
            isoFocLen = mean(GetPinholeFocLenVec(obj, scaleLevel));
        end
        
        function isoFocLen = GetFocLen(obj)
            isoFocLen = mean(GetFocLenVec(obj));
        end
        
        function intrParam = GetPinholeIntrParam(obj, scaleLevel)
            focLenVec = GetPinholeFocLenVec(obj, scaleLevel);
            princpPt = GetPinholePrincpPt(obj, scaleLevel);
            skew = GetPinholeSkew(obj);
            
            intrParam = [focLenVec; princpPt; skew];
        end
        
        function intrParam = GetIntrParam(obj)
            focLenVec = GetFocLenVec(obj);
            princpPt = GetPrincpPt(obj);
            skew = GetSkew(obj);
            distCoeff = GetDistCoeff(obj);
            
            intrParam = [focLenVec; princpPt; skew; distCoeff];
        end
        
        function SetMap(obj, mapX, mapY)
            obj.mapX = mapX;
            obj.mapY = mapY;
        end
        
        function outImg = Remap(obj, inImg)
            if (isempty(obj.mapX) || isempty(obj.mapY))
                outImg = inImg;
                return;
            end
            
            assert(all(size(inImg(:,:,1)) == size(obj.mapX)) && all(size(inImg(:,:,1)) == size(obj.mapY)), 'The size of input image is not the same as the size of precomputed map');
            outImg = OcvRemap(inImg, obj.mapX, obj.mapY);
        end
        
    end
    
end