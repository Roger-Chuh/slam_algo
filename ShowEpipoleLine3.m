function [change,change2,errL,errR] = ShowEpipoleLine3(FundMat,imgL,imgR,matchedPointsL,matchedPointsR,varargin)


if (nargin == 5)
    draw = 1;

elseif (nargin == 6)
    draw = varargin{1};
    

else
    error('Too many input arguments');
end



matchedPointsL0 = matchedPointsL;
matchedPointsR0 = matchedPointsR;

N = size(matchedPointsL,1);
if N > 15
    idx = randperm(N);
    idxx = idx(1 : 15);
    matchedPointsL = matchedPointsL(idxx,:);
    matchedPointsR = matchedPointsR(idxx,:);
end

if draw == 1
    if ndims(imgL) ~= 3
        imgL(:,:,2) = imgL(:,:,1);
        imgL(:,:,3) = imgL(:,:,1);
        imgR(:,:,2) = imgR(:,:,1);
        imgR(:,:,3) = imgR(:,:,1);
        imgL = uint8(imgL);
        imgR = uint8(imgR);
    end
    subplot(1,2,1),
end
epiLines = epipolarLine(FundMat', matchedPointsR);
epiLines0 = epipolarLine(FundMat', matchedPointsR0);


if draw == 1
    imshow(imgL);hold on;plot(matchedPointsL(:,1),matchedPointsL(:,2),'*r');
end
[~,scale0] = NormalizeVector(epiLines0(:,1:2));
epiLines0 = epiLines0./repmat(scale0,1,3);
% errR = dot(epiLines20',pextend(matchedPointsR0'));
errL = dot(epiLines0',pextend(matchedPointsL0'));

points = lineToBorderPoints(epiLines, size(imgL));
if draw == 1
    line(points(:, [1,3])', points(:, [2,4])'); title('Left');
    
    subplot(1,2,2);
    imshow(imgR);hold on;plot(matchedPointsR(:,1),matchedPointsR(:,2),'*r');
end
epiLines2 = epipolarLine(FundMat, matchedPointsL);
epiLines20 = epipolarLine(FundMat, matchedPointsL0);
[~,scale20] = NormalizeVector(epiLines20(:,1:2));
epiLines20 = epiLines20./repmat(scale20,1,3);
errR = dot(epiLines20',pextend(matchedPointsR0'));


points2 = lineToBorderPoints(epiLines2, size(imgR));
if draw == 1
    line(points2(:, [1,3])', points2(:, [2,4])'); title('right');
end


epiLine = [epiLines(:,1)./epiLines(:,3) epiLines(:,2)./epiLines(:,3) epiLines(:,3)./epiLines(:,3)];

% figure(103),clf;plot([errL' errR']);
epiline = mean(epiLine);
aa = epiLines(:,1)./epiLines(:,2);
% % % change = var(aa);
change = mean(aa);
change2 = var(aa);
if 0
    figure,plot(epiLine(:,1),epiLine(:,2),'r+');axis equal
end

end