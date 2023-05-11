classdef PinholeCameraModel < MonoCameraInterface
    properties
        phFocLen    % 2 by 1 vector, focal length [focLenX, focLenY]
        phPrincpPt  % 2 by 1 vector, principle point [x,y]
        phSkew      % scalar, skew factor. set to zero
    end
    
    methods
        function intrMat = GetPinholeIntrMat(obj, scaleLevel)
            if ~exist('scaleLevel', 'var')
                scaleLevel = 0;
            end
            
            focLen = obj.phFocLen / 2^scaleLevel;
            princpPt = (obj.phPrincpPt + 0.5) / 2^scaleLevel - 0.5;
            skew = obj.phSkew;
            intrMat = [...
                       focLen(1), skew,      princpPt(1)
                       0          focLen(2), princpPt(2)
                       0          0          1];
        end
        
        function focLenVec = GetPinholeFocLenVec(obj, scaleLevel)
            if ~exist('scaleLevel', 'var')
                scaleLevel = 0;
            end
            focLenVec = obj.phFocLen / 2^scaleLevel;
        end
        
        function princpPt = GetPinholePrincpPt(obj, scaleLevel)
            if ~exist('scaleLevel', 'var')
                scaleLevel = 0;
            end
            
            princpPt = (obj.phPrincpPt + 0.5) / 2^scaleLevel - 0.5;
        end
        
        function intrMat = GetIntrMat(obj, scaleLevel)
            if ~exist('scaleLevel', 'var')
                scaleLevel = 0;
            end
            intrMat = GetPinholeIntrMat(obj, scaleLevel);
        end
        
        function focLenVec = GetFocLenVec(obj, scaleLevel)
            if ~exist('scaleLevel', 'var')
                scaleLevel = 0;
            end
            focLenVec = GetPinholeFocLenVec(obj, scaleLevel);
        end
        
        function princpPt = GetPrincpPt(obj, scaleLevel)
            if ~exist('scaleLevel', 'var')
                scaleLevel = 0;
            end
            princpPt = GetPinholePrincpPt(obj, scaleLevel);
        end
        
        function phSkew = GetPinholeSkew(obj, scaleLevel) %#ok<INUSD>
            phSkew = obj.phSkew;
        end
        
        function distCoeff = GetDistCoeff(obj) %#ok<MANU>
            distCoeff = [];
        end
        
        function SetPinholeCamParam(obj, phFocLen, phPrincpPt, phSkew)
            obj.phFocLen = phFocLen;
            obj.phPrincpPt = phPrincpPt;
            if ~exist('phSkew', 'var')
                phSkew = 0;
            end
            obj.phSkew = phSkew;
        end
        
        function varargout = LoadParam(obj, paramFileOrFid)
            if ischar(paramFileOrFid)
                assert(nargout == 0, 'No FID is returned if given parameter file path ');
                [paramFid, errMsg] = fopen(paramFileOrFid);
                if (paramFid < 0)
                    error('Cannot open %s for read: %s', paramFileOrFid, errMsg);
                end
                needClose = true;
            else
                paramFid = paramFileOrFid;
                needClose = false;
            end
             
            params = fscanf(paramFid, '%f %f %f %f', 4);
            
            obj.phFocLen = params(1:2);
            obj.phPrincpPt = params(3:4);
            obj.phSkew = 0;
            
            if needClose
                fclose(paramFid);
            else
                if (nargout > 0)
                    varargout{1} = paramFid;
                end
            end
        end
    end
end