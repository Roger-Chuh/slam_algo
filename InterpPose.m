function [vPoseNew, tsNew, ingIndNew] = InterpPose(ingInd, gtPose, gtTimeStamp, vPose, visualTimeStamp)


valid = [];
cnt = 1;
for i = 1 : size(visualTimeStamp, 1)
    
    time11 = visualTimeStamp(i,1);
    id1 = find(gtTimeStamp(:,1) <= time11);
    id2 = find(gtTimeStamp(:,1) > time11);
    
    if isempty(id1) || isempty(id2)
        continue;
    end
    t1 = gtTimeStamp(id1(end),1);
    t2 = gtTimeStamp(id2(1),1);
    q1 = rotMat2quatern(reshape(gtPose(id1(end),1:9), 3,3)); 
    q2 = rotMat2quatern(reshape(gtPose(id2(1),1:9), 3,3));
    scale = [(time11-t1)/(t2-t1) (t2-time11)/(t2-t1) ];
    q11 = quatnormalize(q1);
    q22 = quatnormalize(q2);
    qi = quatinterp(q11,q22,scale(1),'slerp');
    rNew(cnt,:) = reshape(quatern2rotMat(qi),1,9);
    valid = [valid; i];
    cnt = cnt + 1;
end


vXNew = interp1(gtTimeStamp(:,1),gtPose(:,10),visualTimeStamp(:,1));
vYNew = interp1(gtTimeStamp(:,1),gtPose(:,11),visualTimeStamp(:,1));
vZNew = interp1(gtTimeStamp(:,1),gtPose(:,12),visualTimeStamp(:,1));
vPoseNew = [rNew vXNew(valid) vYNew(valid) vZNew(valid)];

tsNew = visualTimeStamp(valid);
ingIndNew = ingInd(valid);
if 0
    figure,plotPath(vPoseNew)
end

end