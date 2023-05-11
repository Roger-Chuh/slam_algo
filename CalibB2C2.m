function CalibB2C2(inputDir, startt, endd)

intrMat = [554.256258 0 320.5; 0 554.256258 240.5; 0 0 1];
% intrMat = [554.256258/2 0 160.5; 0 554.256258/2 120.5; 0 0 1];
b2c = [eye(3) [10 45 -170.2]';0 0 0 1];


dirInfo = dir(fullfile(inputDir, '*.mat'));

load(fullfile(inputDir, dirInfo(startt).name));
data1 = data;
load(fullfile(inputDir, dirInfo(endd).name));
data2 = data;
clear 'data';

theta = data2{6} - data1{6};

depthGTPrv = data1{5};
depthGTCur = data2{5};
imgPrvL = data1{1};
imgCurL = data2{1};


xMat = []; yMat = []; cnt = 1;
for i = startt : endd
    load(fullfile(inputDir, dirInfo(i).name));
    thetaList(cnt,1) = data{6};
    imgL = data{1};
    depthGT(:,:,cnt) = data{5};
    Img(:,:,cnt) = imgL(:,:,1);
    cbcL = detectCnr(imgL);
    xMat(:,cnt) = cbcL(1,:)';
    yMat(:,cnt) = cbcL(2,:)';
    cnt = cnt + 1;
end
 
cbcL2 = detectCnr(imgPrvL);
cbcR2 = detectCnr(imgCurL);


imgSize = size(depthGTPrv);
margin = 5;

if theta < 0
   [xGrid, yGrid] = meshgrid([imgSize(2)/2: imgSize(2) - margin], [margin : imgSize(1)/2]);
   [xGrid, yGrid] = meshgrid([600: imgSize(2) - margin], [margin : imgSize(1)/2]);
  
else
    [xGrid, yGrid] = meshgrid([margin: 10], [margin : imgSize(1)/2]);
end

pix = [xGrid(:) yGrid(:)];

indPrv = sub2ind(size(depthGTPrv), pix(:,2), pix(:,1));
depthListGT = depthGTPrv(indPrv);
[pixGT, transXYZ] = GetGtTrace2(b2c, rad2deg(theta), pix,depthListGT,intrMat);



% figure,subplot(1,2,1);imshow(imgPrvL);hold on;plot(pix(:,1), pix(:,2),'.r');title('prv');subplot(1,2,2);imshow(imgCurL);hold on;plot(pixGT(:,1), pixGT(:,2),'.r');title('cur');

valid = find(pixGT(:,1) > 1 & pixGT(:,2) > 1 & pixGT(:,1) < imgSize(2)-1 & pixGT(:,2) < imgSize(1)-1);


figure,subplot(1,2,1);imshow(imgPrvL);hold on;plot(pix(valid,1), pix(valid,2),'.r');title('prv');subplot(1,2,2);imshow(imgCurL);hold on;plot(pixGT(valid,1), pixGT(valid,2),'.r');title('cur');

indCur = sub2ind(size(depthGTCur), round(pixGT(valid,2)), round(pixGT(valid,1)));

depthListCur = depthGTCur(indCur);

err = transXYZ(valid,3) - depthListCur;


b2cVec0 = [rodrigues(b2c(1:3,1:3)); b2c(1:3,4)];
b2cVec0 = b2cVec0(1:3);

errVar0 = CostFunc(b2cVec0, theta, pix, depthListGT, intrMat, depthGTCur,imgSize);
options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','off','MaxFunEvals',10000,'TolX',1e-8);%,'MaxFunEvals',10000);%, 'MaxIterations',5);
[vec,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) CostFunc(X, theta, pix, depthListGT, intrMat, depthGTCur,imgSize),[b2cVec0],[],[],options);%,data_1,obs_1)
errVar1 = CostFunc(vec, theta, pix, depthListGT, intrMat, depthGTCur,imgSize);        
end
function [pixGT, transXYZ] = GetGtTrace2(b2cPmat, k2cRef0, Pix,depthListGT,intrMat)

% if 0
%     k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(thetaRng(idM)),zeros(3,1));
% else
%     k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cRef0),zeros(3,1));
% end
k2cBodyPmat = [roty(double(k2cRef0)) [0;0;0];0 0 0 1];
k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;



metricPrevPtCcsGT = intrMat\HomoCoord(Pix',1);
metricPrevPtCcsGT = normc(metricPrevPtCcsGT);
if 1
    scaleAllGT = depthListGT./metricPrevPtCcsGT(3,:)';
else
    dispGTComp = dispList(inlierId) - disparityErrorRound;
    depthListGTComp = intrMat(1,1).*norm(obj.camModel.transVec1To2)./(dispGTComp + (princpPtR(1) - princpPtL(1)));
    scaleAllGT = depthListGTComp./metricPrevPtCcsGT(3,:)';
end
XYZ = [repmat(scaleAllGT',3,1).*metricPrevPtCcsGT];


k2cT = -b2cPmat(1:3,1:3)*k2cBodyPmat(1:3,1:3)*b2cPmat(1:3,1:3)'*b2cPmat(1:3,4) + b2cPmat(1:3,4);
k2cCam = [k2cCamPmat(1:3,1:3) k2cT;0 0 0 1];
homocurrCcsXYZ = k2cCam*HomoCoord(XYZ,1); %pix0_GT_ = pflat(intrMat*homocurrCcsXYZALL_(1:3,:))';
pixGT = pflat(intrMat*homocurrCcsXYZ(1:3,:))';
pixGT = pixGT(:,1:2);

transXYZ = homocurrCcsXYZ(1:3,:)';

end

function errVar = CostFunc(b2cVec, theta, pix, depthListGT, intrMat, depthGTCur,imgSize)
b2cVec(1:3);
if length(b2cVec) > 3
    b2c = [rodrigues(b2cVec(1:3)) b2cVec(4:6);0 0 0 1];
else
    b2c = [rodrigues(b2cVec(1:3)) [10;45;-170.2];0 0 0 1];
end
[pixGT, transXYZ] = GetGtTrace2(b2c, rad2deg(theta), pix,depthListGT,intrMat);


valid = find(pixGT(:,1) > 1 & pixGT(:,2) > 1 & pixGT(:,1) < imgSize(2)-1 & pixGT(:,2) < imgSize(1)-1);

indCur = sub2ind(size(depthGTCur), round(pixGT(valid,2)), round(pixGT(valid,1)));

depthListCur = depthGTCur(indCur);

err = transXYZ(valid,3) - depthListCur;
errVar = mean(abs(err).^2);
end
function cbcL = detectCnr(imgL1)
[cbcX1, cbcY1, corner1] = DetectCbCorner(rgb2gray(imgL1));
[initCbcX1, initCbcY1] = CbCoordSys(cbcX1, cbcY1);
cbcL = cornerfinder([initCbcX1(:),initCbcY1(:)]',double(rgb2gray(imgL1)),5,5);
end