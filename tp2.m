figure,hold on;axis equal;grid on;
MedianMinMax = [];
validId = [];
goldenPose0 = goldenPose;
for pp = 1 :size(polyMat,1)%  13% 21% 
    try
        plot(polyMat{pp,2}(:,1), polyMat{pp,2}(:,2),'.m');
        pos(pp,:) = median(polyMat{pp,2},1);
        [~,minZId] = min(polyMat{pp,2}(:,2));
        [~,maxZId] = max(polyMat{pp,2}(:,2));
        MedianMinMax(pp,:) = [timeStampList(pp) median(polyMat{pp,2},1) polyMat{pp,2}(minZId,:) polyMat{pp,2}(maxZId,:)];
        validId = [validId; pp];
        plot(goldenPose0(pp,2), goldenPose0(pp,4)+0,'xc');
        ashsdh = 1;
%         plot(pt(:,1),pt(:,2),'ob');
    catch
        dghl = 1;
    end
end

% goldenPose0(:,[4]) = goldenPose0(:,[4]) + pos(1,2);
% plot(goldenPose0(validId,2), goldenPose0(validId,4)+0,'xr');
plot(pt(:,1),pt(:,2),'ob');