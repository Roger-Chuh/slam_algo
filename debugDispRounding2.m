k2cBodyTmp = [roty(10) [0 0 0]';0 0 0 1];
b2c = inv([eye(3) [-10.000000 -45.000000 170.200000]';0 0 0 1]);
intrMat = [277 0 0160; 0 277 120;0 0 1]

k2cCam = b2c*k2cBodyTmp/b2c;
pix = [10 130];
metricPrevPtCcsKey = intrMat\HomoCoord(pix',1);
dispRef = 3.0778;
baseline = 20;
metricPrevPtCcsKey = normc(metricPrevPtCcsKey);
zListTmp = intrMat(1)*baseline/dispRef;
Scale00 = zListTmp./metricPrevPtCcsKey(3,:);
KeyCcsXYZVec = [ Scale00.*metricPrevPtCcsKey];
homocurrCcsXYZALLVec = k2cCam*HomoCoord(KeyCcsXYZVec,1);
pixKeyVec = pflat(intrMat*homocurrCcsXYZALLVec(1:3,:));

dispVec = [-0.05:0.0001:0.05]; pixKeyVecTemp = []; ZListTmp1 = [];
for i = 1 : length(dispVec)
    dispTemp = dispRef + dispVec(i);
    metricPrevPtCcsKey = normc(metricPrevPtCcsKey);
    zListTmp1 = intrMat(1)*baseline/dispTemp;
    ZListTmp1(i,1) = zListTmp1;
    Scale00 = zListTmp1./metricPrevPtCcsKey(3,:);
    KeyCcsXYZVec = [ Scale00.*metricPrevPtCcsKey];
    homocurrCcsXYZALLVec = k2cCam*HomoCoord(KeyCcsXYZVec,1);
    pixKeyVecTemp(:,i) = pflat(intrMat*homocurrCcsXYZALLVec(1:3,:));
    
    
end
figure,imshow(zeros(240,320));hold on;plot(pixKeyVecTemp(1,:),pixKeyVecTemp(2,:),'.r');plot(pixKeyVec(1), pixKeyVec(2),'.g')


sadgah = 1;