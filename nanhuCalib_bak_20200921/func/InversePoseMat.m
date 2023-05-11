function poseMat2 = InversePoseMat(poseMat1)

for i = 1 : size(poseMat1,1)
    poseNew = inv([reshape(poseMat1(i,1:9),3,3) poseMat1(i,10:12)';0 0 0 1]);
    poseMat2(i,:) = [reshape(poseNew(1:3,1:3),1,9) poseNew(1:3,4)']; 
end




end