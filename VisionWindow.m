classdef VisionWindow
    properties
        
    end
    
    methods (Static)
        function [normal, d, piontInPlane] = GetPlaneFuncUsing3Points(p1,p2,p3) 
            % this function is to get plane function using 3 3D points;
            % also, the function can get 3d point within the 3D points;
            % input:
            % p1, p2, p3 is three 3-D piont
            % output:
            % normal is the norm vector
            % d is the forth coeff of plane functions
            % piontInPlane is the point within the 3D points
            
            xv = [p1(1);p2(1);p3(1)];
            yv = [p1(2);p2(2);p3(2)];
            xmax = max(xv);
            xmin = min(xv);
            ymax = max(yv);
            ymin = min(yv);
            xstep = 0.05;
            ystep = 0.05;
                       
            normal = cross(p1 - p2, p1 - p3);
            
            d = p1(1)*normal(1) + p1(2)*normal(2) + p1(3)*normal(3);
            d = -d;
            x = xmin:xstep:xmax; y = ymin:ystep:ymax;
            [X,Y] = meshgrid(x,y);
            Z = (-d - (normal(1)*X) - (normal(2)*Y))/normal(3);
            pp(:,:,1) = X;
            pp(:,:,2) = Y;
            pp(:,:,3) = Z;            

            validFlag = inpolygon(X(:),Y(:),xv,yv);
            mask = [vec2mat(validFlag,Rows(X))]';
            mask3D = repmat(mask,[1,1,3]);
            pp(mask3D == 0) = nan;
            
            color(:,:,1) = 255*ones(size(X));
            color(:,:,2) = zeros(size(X));
            color(:,:,3) = zeros(size(X));
            color = uint8(color);
            
            piontInPlane = pointCloud(pp);
            piontInPlane.Color = color;
%             scene = pcplayer([-3, 3], [-3, 3], [-1, 10], 'VerticalAxis', 'y', ...
%                 'VerticalAxisDir', 'down');
%             view(scene, visionWindow)
        end
        
        function [Vertex] = GetWindowVertex(intrMat,imgSize, windowDepth)
            p1 = [intrMat\[0;0;1]*windowDepth/1000]';
            p2 = [intrMat\[imgSize(2);0;1]*windowDepth/1000]';
            p3 = [intrMat\[imgSize(2);imgSize(1);1]*windowDepth/1000]';
            p4 = [intrMat\[0;imgSize(1);1]*windowDepth/1000]';
            Vertex = [p1;p2;p3;p4];
        end        
        
    end

            
end