function debugDispRounding()


b2c = inv([eye(3) [-10.000000 -45.000000 170.200000]';0 0 0 1]);
intrMat = [277 0 0160; 0 277 120;0 0 1];

disp0 = 3.0778;
baseline = 20;
angK2P = 30;
angP2C = 2.5;
% angK2C = angK2P + angP2C + 0.1*(rand(1)-0.5);
angK2C = angK2P + angP2C + 0.03;



thetaSamp = -1:0.02:1; 
dispSamp = -0.1:0.01:0.1;
randIdZ = randperm(length(dispSamp));
gtIdZ = randIdZ(1);
randIdAng = randperm(length(thetaSamp));
gtIdAng = randIdAng(1);



k2cBodyK2P = [roty(angK2P) [0 0 0]';0 0 0 1];
k2cCamK2P = b2c*k2cBodyK2P/b2c;

k2cBodyP2C = [roty(angP2C) [0 0 0]';0 0 0 1];
k2cCamP2C = b2c*k2cBodyP2C/b2c;

k2cBodyK2C = [roty(angK2C) [0 0 0]';0 0 0 1];
k2cCamK2C = b2c*k2cBodyK2C/b2c;



pix = [50 13];

KeyCcsXYZVecKey = GetXYZ(intrMat, pix, disp0, dispSamp,baseline);


homocurrCcsXYZALLVecKey = k2cCamK2P*HomoCoord(KeyCcsXYZVecKey,1);
pixKeyVecKey = pflat(intrMat*homocurrCcsXYZALLVecKey(1:3,:));

homocurrCcsXYZALLVecPrv = homocurrCcsXYZALLVecKey(1:3,(size(pixKeyVecKey,2)+1)/2)';
pixKeyVecPrv = pixKeyVecKey(1:2,(size(pixKeyVecKey,2)+1)/2)';
KeyCcsXYZVecPrv = GetXYZ(intrMat, pixKeyVecPrv, intrMat(1)*baseline/homocurrCcsXYZALLVecPrv(3), dispSamp,baseline);

[ReprojXP2C, ReprojYP2C] = Reproj2Candidate(angP2C, thetaSamp, b2c, KeyCcsXYZVecPrv,intrMat);

% dispVec = [-0.05:0.0001:0.05]; pixKeyVecTemp = []; ZListTmp1 = [];
[ReprojXK2P, ReprojYK2P] = Reproj2Candidate(angK2P, thetaSamp, b2c, KeyCcsXYZVecKey,intrMat);

[ReprojXK2C, ReprojYK2C] = Reproj2Candidate(angK2C, thetaSamp, b2c, KeyCcsXYZVecKey,intrMat);

checkErr = [ReprojXP2C((length(dispSamp)+1)/2,(length(thetaSamp)+1)/2)-ReprojXK2C((length(dispSamp)+1)/2,(length(thetaSamp)+1)/2) ReprojYP2C((length(dispSamp)+1)/2,(length(thetaSamp)+1)/2)-ReprojYK2C((length(dispSamp)+1)/2,(length(thetaSamp)+1)/2)];


checkErr2 = [mean([ReprojXP2C(:) ReprojYP2C(:)]) - mean([ReprojXK2C(:) ReprojYK2C(:)])]

gtPixP2C = [ReprojXP2C(gtIdZ, (length(thetaSamp)+1)/2) ReprojYP2C(gtIdZ, (length(thetaSamp)+1)/2)];
gtPixK2C = [ReprojXK2C(gtIdZ, (length(thetaSamp)+1)/2) ReprojYK2C(gtIdZ, (length(thetaSamp)+1)/2)];


% figure,imshow(zeros(240,320));hold on;plot(pixKeyVecTemp(1,:),pixKeyVecTemp(2,:),'.r');plot(pixKeyVec(1), pixKeyVec(2),'.g')
figure,imshow(rand(240,320),[]);hold on; 
% plot(ReprojXK2P(:), ReprojYK2P(:),'.r');plot(ReprojXK2P(:,1), ReprojYK2P(:,1),'.g');
plot(ReprojXP2C(:), ReprojYP2C(:),'.r');plot(ReprojXP2C(:,1), ReprojYP2C(:,1),'.g');
plot(ReprojXK2C(:), ReprojYK2C(:),'.b');plot(ReprojXK2C(:,1), ReprojYK2C(:,1),'.c');plot(ReprojXK2C(:,2), ReprojYK2C(:,2),'.y');
plot(pix(1),pix(2),'*r');
plot(gtPixP2C(1),gtPixP2C(2),'og');plot(gtPixK2C(1),gtPixK2C(2),'oc');
title(num2str(gtIdZ));



sadgah = 1;
end

function KeyCcsXYZVec = GetXYZ(intrMat, pix, disp0, dispSamp,baseline)
metricPrevPtCcsKey = intrMat\HomoCoord(repmat(pix,length(dispSamp),1)',1);
dispRef = disp0 + dispSamp;

metricPrevPtCcsKey = normc(metricPrevPtCcsKey);
zListTmp = intrMat(1).*baseline./dispRef;
Scale00 = zListTmp./metricPrevPtCcsKey(3,:);
KeyCcsXYZVec = [ Scale00.*metricPrevPtCcsKey];
end

function [ReprojX, ReprojY] = Reproj2Candidate(angK2P, thetaSamp, b2c, KeyCcsXYZVec,intrMat)

ReprojX = []; ReprojY = [];
for i = 1 : length(thetaSamp)
    k2cBodyK2P_tmp = [roty(angK2P + thetaSamp(i)) [0 0 0]';0 0 0 1];
    k2cCamK2P_tmp = b2c*k2cBodyK2P_tmp/b2c;
    
    homocurrCcsXYZALLVec_tmp = k2cCamK2P_tmp*HomoCoord(KeyCcsXYZVec,1);
    pixKeyVec_tmp = pflat(intrMat*homocurrCcsXYZALLVec_tmp(1:3,:));
    ReprojX(:,i) = pixKeyVec_tmp(1,:)';
    ReprojY(:,i) = pixKeyVec_tmp(2,:)';
       
    
end

end