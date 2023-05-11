
clear
load('D:\Temp\20190717\vr\matlab.mat')


poseMat1 = [];
for i = 1 : size(data1,1)
    r1 = q2R(data1(i,[8 5 6 7]))';
    tempVec1 = rodrigues(r1)';
    rotVec1(i,:) = tempVec1;
    t1 = data1(i,[2:4]);
    rt1 = ([r1 t1';0 0 0 1]);
    poseMat1 = [poseMat1; [reshape(rt1(1:3,1:3),1,9) rt1(1:3,4)']];
    
    
end


poseMat2 = [];
for i = 1 : size(data2,1)
    r2 = q2R(data2(i,[8 5 6 7]))';
    tempVec2 = rodrigues(r2)';
    rotVec2(i,:) = tempVec2;
    t2 = data2(i,[2:4]);
    rt2 = ([r2 t2';0 0 0 1]);
    poseMat2 = [poseMat2; [reshape(rt2(1:3,1:3),1,9) rt2(1:3,4)']];
    
    
end

vldCnt = [];
data11 = data1(1228:end,:);

for i = 1 : size(data11,1)
    if 0 %data1(i,1) > 0
        vldCnt = [vldCnt;i];
    end
    
    time11 = data11(i,1);
    id1 = find(data2(:,1) <= time11);
    id2 = find(data2(:,1) > time11);
    t1 = data2(id1(end),1);   q1 = data2(id1(end),[8 5 6 7]);
    t2 = data2(id2(1),1);     q2 = data2(id2(1),[8 5 6 7]);
    scale = [(time11-t1)/(t2-t1) (t2-time11)/(t2-t1) ];
    q11 = quatnormalize(q1);
    q22 = quatnormalize(q2);
    qi = quatinterp(q11,q22,scale(1),'slerp');
    if 0
        qi1 = quatinterp(q11,q22,0,'slerp');
        qi1-q11
    end
    qMat(i,:) = qi;
    
    
end
data1New = data11;
data2New = [data4(1228:end,1:4) qMat(:,[2 3 4 1])];
poseMat1 = [];
for i = 1 : size(data1New,1)
    r1 = q2R(data1New(i,[8 5 6 7]))';
    tempVec1 = rodrigues(r1)';
    rotVec1(i,:) = tempVec1;
    t1 = data1New(i,[2:4]);
    rt1 = ([r1 t1';0 0 0 1]);
    poseMat1 = [poseMat1; [reshape(rt1(1:3,1:3),1,9) rt1(1:3,4)']];
    
    
end


poseMat2 = [];
for i = 1 : size(data2New,1)
    r2 = q2R(data2New(i,[8 5 6 7]))';
    tempVec2 = rodrigues(r2)';
    rotVec2(i,:) = tempVec2;
    t2 = data2New(i,[2:4]);
    rt2 = ([r2 t2';0 0 0 1]);
    poseMat2 = [poseMat2; [reshape(rt2(1:3,1:3),1,9) rt2(1:3,4)']];
    
    
end
ceilingPose = poseMat1;
cbPose = poseMat2;
ChangeCoord2(ceilingPose,cbPose)