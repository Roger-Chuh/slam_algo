classdef BCCameraModel < PinholeCameraModel
    % Brown-Conrady distortion model
    
    properties
        focLen    % 2 by 1 vector, focal length [focLenX; focLenY]
        princpPt  % 2 by 1 vector, principle point [x; y]
        skew      % scalar, skew factor
        
        distCoeff % 5 by 1 vector, distortion coefficients
        
    end
    
    methods
        function intrMat = GetIntrMat(obj)
            intrMat = [...
                       obj.focLen(1), obj.skew,      obj.princpPt(1)
                       0              obj.focLen(2), obj.princpPt(2)
                       0              0              1];
        end
        
        function focLenVec = GetFocLenVec(obj)
            focLenVec = obj.focLen;
        end
        
        function princpPt = GetPrincpPt(obj)
            princpPt = obj.princpPt;
        end
        
        function skew = GetSkew(obj)
            skew = obj.Skew;
        end
        
        function distCoeff = GetDistCoeff(obj)
            distCoeff = obj.distCoeff;
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
             
            params = fscanf(paramFid, '%f %f %f %f %f %f %f %f %f', 9);
            
            obj.focLen = params(1:2);
            obj.princpPt = params(3:4);
            obj.skew = 0;
            obj.distCoeff = params(5:9);
            
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