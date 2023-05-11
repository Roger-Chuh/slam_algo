classdef PointCoordTransformer < handle
    
    % This class represents the transformation of point coordinates to
    % another Euclid coordinate system
    
    properties
        transformMat;  % 4 by 4 Euclid transformation matrix
    end
    
    methods
        % Constructor
        function obj = PointCoordTransformer(varargin)
            if (nargin == 0)
                obj.transformMat = eye(4);
            elseif (nargin == 1)
                % A transformation matrix in SE(3) 
                obj.transformMat = varargin{1};
            elseif (nargin == 2)
                rotParam = varargin{1};
                transVec = varargin{2};
                assert(isvector(transVec) && length(transVec) == 3, 'The second input parameter is translation vector which must be a 3-vector');
                if all(size(rotParam) == [3,3])
                    % rotation matrix
                    rotMat = rotParam;
                elseif (isvector(rotParam) && length(rotParam) == 3)
                    % rotation vector
                    rotMat = rodrigues(rotParam);
                elseif (isvector(rotParam) && length(rotParam) == 4)
                    % quaternion
                    rotMat = quat2rotm(rotParam(:));
                else
                    error('The first input parameter is rotation parameter. It can only be a rotatioin (3x3) orthonomal matrix, a rotation vector (3-vector) or a quaternion (4-vector)');
                end
                
                obj.transformMat = [rotMat, transVec(:); zeros(1,3), 1];
            else
                error('The number of input parameters should be 0, 1 or 2');
            end
        end
        
        function rotMat = R(obj)
            rotMat = obj.transformMat(1:3, 1:3);
        end
        
        function rotVec = Rotv(obj)
            rotVec = rodrigues(R(obj));
        end
        
        function [rotAng, rotAx] = RotAxang(obj)
            axang = rotm2axang(R(obj));
            rotAng = axang(end);
            rotAx = axang(1:3)';
        end
        
        function rotQuat = Rotq(obj)
            rotQuat = rotm2quat(R(obj));
        end
        
        function transVec = T(obj)
            transVec = obj.transformMat(1:3, 4);
        end
        
        function transVec = T0(obj)
            transVec = -R(obj)'*T(obj);
        end
        
        function emat = EssentialMatrix(obj)
            % pt2Ccs' * emat * pt1Ccs = 0
            
            emat = SkewSymMat(T(obj)) * R(obj);
        end
        
        function fmat = FundamentalMatrix(obj, intrMat1, intrMat2)
            % pt2IcsHomo' * fmat * pt1IcsHomo = 0
            
            fmat = intrMat2' \ EssentialMatrix(obj) / intrMat1;
        end
        
        function axDst = RotateAxisDst(obj)
            % Rotation axis in destination coordinate system
            % It is the second column of rotation matrix
            axDst = obj.transformMat(1:3, 2);
        end
        
        function objResult = Inv(obj)
            objResult = PointCoordTransformer(inv(obj.transformMat));
        end
        
        % Overloaded operators
        % *
        function obj3 = mtimes(obj1, obj2)
            if isa(obj2, 'PointCoordTransformer')
                obj3 = PointCoordTransformer(obj1.transformMat * obj2.transformMat);
            elseif isnumeric(obj2)
                if (Rows(obj2) == 3)
                    obj3 = obj1.transformMat * HomoCoord(double(obj2),1);
                    obj3 = obj3(1:3, :);
                else
                    error('If the second operand is of numeric type, it must be a 3xn matrix');
                end
            else
                error('The second operand must be a PointCoordTransformer or a 3xn numeric matrix');
            end
        end
        
        % /
        function obj3 = mrdivide(obj1, obj2)
            if isa(obj2, 'PointCoordTransformer')
                obj3 = PointCoordTransformer(obj1.transformMat / obj2.transformMat);
            else
                error('The second operand must be a PointCoordTransformer');
            end
        end
        
        % \
        function obj3 = mldivide(obj1, obj2)
            if isa(obj2, 'PointCoordTransformer')
                obj3 = PointCoordTransformer(obj1.transformMat \ obj2.transformMat);
            elseif isnumeric(obj2)
                if (Rows(obj2) == 3)
                    obj3 = obj1.transformMat \ HomoCoord(double(obj2));
                    obj3 = obj3(1:3, :);
                end
            else
                error('The second operand must be a PointCoordTransformer or a 3xn numeric matrix');
            end
        end
    end
    
end