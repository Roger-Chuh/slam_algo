function TestRogThoughts5()

load('D:\Auto\data3\20200323_083500_video\20200922_071710\vslMatBak2.mat');
load('D:\Auto\data3\20200323_083500_video\20200827_133524\vslMat3.mat')

load('D:\Temp\20200831\rMatNew3.mat')

intrMat = eye(3);
intrMat(1,1) = 2.0003201001638317e+03;
intrMat(1,3) = 9.6889274597167969e+02;
intrMat(2,2) = 2.0003201001638317e+03;
intrMat(2,3) = 5.4882026672363281e+02;


inputDir = 'D:\Auto\data5\081500_all';
dirInfo = dir(fullfile(inputDir, '*.jpg'));
[~, ind1] = sort([dirInfo(1:length(dirInfo)).datenum], 'ascend');
I = [ind1];
tempInfo = cell(length(I),1);
for fg = 1:length(I)
    tempInfo{fg,1} = dirInfo(I(fg)).name;
end
for fgg = 1:length(I)
    dirInfo(fgg).name = tempInfo{fgg};
end

imgInd = 5489; 5870;
img1 = imread(fullfile(inputDir, dirInfo(imgInd).name));
img2 = imread(fullfile(inputDir, dirInfo(imgInd + 1).name));


vslMat = vslMatBak2; vslMat = vslMat(124:320,:);
% vslMat = vslMat3; vslMat = vslMat(250:450,:);



B = fitplane(vslMat(:,10:12)');
B = B./norm(B(1:3));
er = dot(repmat(B,1,size(vslMat,1)), pextend(vslMat(:,10:12)'));




for i = 1 : size(vslMat, 1) - 1
    
    TCur_cam = [reshape(vslMat(i+1,1:9),3,3) vslMat(i+1,10:12)';0 0 0 1];
    TPrv_cam = [reshape(vslMat(i,1:9),3,3) vslMat(i,10:12)';0 0 0 1];
    
    deltaCam(:,:,i) = inv(TCur_cam(1:3,1:3)) * TPrv_cam(1:3,1:3);
    
    
    if deltaCam(1,3,i) < 0
        deltaBody(:,:,i) = roty(-rad2deg(norm(rodrigues(deltaCam(:,:,i)))));
    else
        deltaBody(:,:,i) = roty(rad2deg(norm(rodrigues(deltaCam(:,:,i)))));
%         turnId = [turnId;i];
    end
    
    
%     deltaBody(:,:,cnt) = inv(TCur_body) * TPrv_body;
%     deltaCam(:,:,cnt) = [b2c [0;0;0];0 0 0 1] * deltaBody(:,:,cnt) * [b2c' [0;0;0];0 0 0 1];
    
    
end


b2cVec = [0;0;0];
b2cVec = rodrigues(rMatNew(1:3,1:3));

Err0 = CostFunc(deltaCam, deltaBody, b2cVec);

options1= optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter','MaxFunEvals',10000,'TolX',1e-20, 'OptimalityTolerance', 1e-20,'FunctionTolerance', 1e-20,'StepTolerance',1e-20,'MaxFunEvals',10000, 'MaxIterations',5000000);
[b2cVecOpt,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) CostFunc(deltaCam, deltaBody, X),[b2cVec],[],[],options1);


Err1 = CostFunc(deltaCam, deltaBody, b2cVecOpt);

b2cOpt = rodrigues(b2cVecOpt);
ang = rad2deg(norm(b2cVecOpt));

rMatNew = b2cOpt;

figure,plot([Err0 Err1]);

rMatNew0 = CorrectRot([0; 1; 0], B(1:3));
rMatNew = rotx(180)*rMatNew0;
rMatNew = [rMatNew0(:,3) rMatNew0(:,2) -rMatNew0(:,1)];


imggg = ImgTransform(rgb2gray(img1), intrMat, rMatNew');figure,imshow(imggg);testBirdView2(intrMat,imggg)

end
function Err = CostFunc(deltaCam, deltaBody, b2cVec)
Err = zeros(size(deltaCam, 3), 1);

b2c = rodrigues(b2cVec);
for i = 1 : size(deltaCam, 3)
    tempCam = deltaCam(1:3,1:3,i);
    tempBody = deltaBody(1:3,1:3,i);
    tempCam_comp = b2c(1:3,1:3) * tempBody * b2c(1:3,1:3)';
    Err(i,:) = rad2deg(norm(rodrigues(tempCam_comp' * tempCam)));
end
end
function rMatNew = CorrectRot(axis, gVec)
ang = CalcDegree(axis',gVec');
xVec = cross(axis,gVec);
xVec = xVec./norm(xVec);
zVec = cross(xVec, gVec);
zVec = zVec./norm(zVec);

rMatNew = [xVec'; gVec'; zVec']';
end