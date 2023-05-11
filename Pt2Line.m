function [retVal, dist] = Pt2Line(startt, endd, iso, varargin)

if nargin == 4
    
    
    lineCoeff = varargin{1};
    
    xList1 = 1.*ones(size(lineCoeff,1),1);  xList2 = 100.*ones(size(lineCoeff,1),1);
    
    yList1 = (-lineCoeff(:,3) - lineCoeff(:,1).*xList1)./lineCoeff(:,2);
    yList2 = (-lineCoeff(:,3) - lineCoeff(:,1).*xList2)./lineCoeff(:,2);
    startt = [xList1 yList1];
    endd = [xList2 yList2];
    
    dx = startt(:,1) - endd(:,1);
    dy = startt(:,2) - endd(:,2);
else
    
    dx = startt(:,1) - endd(:,1);
    dy = startt(:,2) - endd(:,2);
end
% 	if(abs(dx) < 0.00000001 && abs(dy) < 0.00000001 )
% 	{
% 		retVal = begin;
% 		return retVal;
% 	}

u = (iso(:,1) - startt(:,1)).*(startt(:,1) - endd(:,1)) +(iso(:,2) - startt(:,2)).*(startt(:,2) - endd(:,2));
u = u./((dx.*dx)+(dy.*dy));

retVal = [startt(:,1) + u.*dx startt(:,2) + u.*dy];
% dist = norm(retVal - iso);
if 1
    try
        [~, dist] = NormalizeVector(retVal - iso);
    catch
        dist = norm(retVal - iso);
    end
    % 	retVal.x = begin.x + u*dx;
    % 	retVal.y = begin.y + u*dy;
    if 0
        line1 = cross([retVal ones(size(retVal,1),1)], [iso ones(size(iso,1),1)]);
        %         line1 = repmat(sign(line1(:,1)),1,size(line1,2)).*line1;
        [~,linenorm] = NormalizeVector(line1(:,1:2));
        line1 = line1./repmat(linenorm,1,size(line1,2));
        
        if 0
            line2 = cross([startt 1], [endd 1]); line2 = line2./norm(line2(1:2));
            err1 = dot(line1(:,1:2)',repmat(line2(1:2),size(line1,1),1)');
            err2 = dot([retVal ones(size(retVal,1),1)]',repmat(line2(1:3),size(line1,1),1)');
        else
            line2 = lineCoeff; % cross([startt ones(size(retVal,1),1)], [endd ones(size(retVal,1),1)]); line2 = line2./norm(line2(1:2));
            err1 = dot(line1(:,1:2)',line2(:,1:2)');  % perpendicular err
            err2 = dot([retVal ones(size(retVal,1),1)]',line2'); % pt on line err
        end
        
    end
    
else
    dist = norm(retVal - iso);
    % 	retVal.x = begin.x + u*dx;
    % 	retVal.y = begin.y + u*dy;
    if 0
        line1 = cross([retVal 1], [iso 1]); line1 = line1./norm(line1(1:2));
        line2 = cross([startt 1], [endd 1]); line2 = line2./norm(line2(1:2));
        err1 = dot(line1(1:2),line2(1:2));
        err2 = dot([retVal 1],line2);
    end
end
end