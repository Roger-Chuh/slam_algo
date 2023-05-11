function slamPoseMat2 = transFormPose(slamPoseMat, rt1675)
%  rt1675 = [reshape(slamPoseMat(baseNum,1:9),3,3) slamPoseMat(baseNum,10:12)';0 0 0 1];
slamPoseMat2 = [];
for ii = 1:size(slamPoseMat,1)
    tmpRT =  [reshape(slamPoseMat(ii,1:9),3,3) slamPoseMat(ii,10:12)';0 0 0 1];
    tmpRt2 = inv(rt1675)*tmpRT;
    slamPoseMat2 = [slamPoseMat2;[reshape(tmpRt2(1:3,1:3),1,9) tmpRt2(1:3,4)']];
end
end