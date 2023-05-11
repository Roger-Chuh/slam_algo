function [radi, ptOnLineErr1, ptOnLineErr2] = ShowEpilineCircle1(ptKey,ptPrv, fundMatP2K, eplInKey, imgKey, circlePt)
randId = randperm(size(ptKey,1));
showNum = 20;
randId = randId(1:showNum);
eplInKey_check = epipolarLine(fundMatP2K,ptPrv(randId,:));

[~, scale] = NormalizeVector(eplInKey(:,1:2));
eplInKey = eplInKey./repmat(scale,1,3);

[~, radi] = NormalizeVector(repmat(ptKey,2,1) - real([circlePt(:,1:2);circlePt(:,3:4)]));

ptOnLineErr1 = dot(eplInKey', real([circlePt(:,1:2) ones(size(circlePt,1),1)])');
ptOnLineErr2 = dot(eplInKey', real([circlePt(:,3:4) ones(size(circlePt,1),1)])');

points = lineToBorderPoints(eplInKey(randId,:), size(imgKey));
imshow(imgKey);hold on;line(points(:, [1,3])', points(:, [2,4])'); title('Left');
plot(ptKey(randId,1),ptKey(randId,2),'.r');
plot(real(circlePt(randId,1)),real(circlePt(randId,2)),'.g');
plot(real(circlePt(randId,3)),real(circlePt(randId,4)),'.b');
title('key frame');

end