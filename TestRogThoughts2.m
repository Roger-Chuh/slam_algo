function TestRogThoughts2()
global usePNP c2b b2v v2c extraVec ctrlPt shortLenScaled  Ptt inCell


c2b = [9.9997740983963013e-01, 4.1571599431335926e-03,  -5.2833566442131996e-03, 0.; -4.3705194257199764e-03,  9.9914819002151489e-01, -4.1034817695617613e-02, -1.4941230773925781e+02; 5.1082675345242023e-03,  4.1056983172893462e-02, 9.9914371967315674e-01, 9.1488652213050694e-15; 0., 0., 0., 1.];
c2b = c2b(1:3,1:3);
b2v = [0.99935734,  -0.035846297,  0. ,  0; 0.035846297,  0.99935734, 0.,  0. ;0. ,0. ,1. ,0. ; 0. , 0. , 0., 1. ];
b2v = b2v(1:3,1:3);

v2c = b2v*c2b;

% close all


shortMatch = false; true; false;


usePNP = true; false; true;

load('D:\Temp\20200911\Ptt.mat')

if shortMatch
    usedId = 5:7;
    for i = 1 : size(Ptt,1)
        
        chosenId = find(ismember(Ptt{i,2}, usedId));
        Ptt{i,1} = Ptt{i,1}(2*chosenId(1)-1:2*chosenId(end),:);
        Ptt{i,2} = 1:length(usedId);
        
    end
end

% close all

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


imgRng = [5867:5899];




intrMat = eye(3);
intrMat(1,1) = 2.0003201001638317e+03;
intrMat(1,3) = 9.6889274597167969e+02;
intrMat(2,2) = 2.0003201001638317e+03;
intrMat(2,3) = 5.4882026672363281e+02;




% % figure(1),
% % for i = 1 : length(imgRng)
% %     img = imread(fullfile(inputDir, dirInfo(imgRng(i)).name));
% %     if 1%i == 1
% %         imshow(img);hold on;plot(Ptt{i,1}(:,1), Ptt{i,1}(:,2), '-or');
% %         drawnow;
% %     end
% % end


shortLen = 2.6597;
ctrlPt = 1.*[0:3:24];
ctrlPtNum = length(ctrlPt);

if usePNP
    scaleList = 0.379135353535353  + linspace(-0.006,0.006,100);
    thetaList = 70.345172727272740  + linspace(-0.1,0.1,100);
    
    scaleList = 0.379171932948603;  + linspace(-0.003,0.003,100);
    thetaList = 70.344627487986202;  + linspace(-0.05,0.05,100);
    
% %     if ~shortMatch
% %         scaleList = 0.379139774630730;  + linspace(-0.003,0.003,100);
% %         thetaList = 70.344957316887488;  + linspace(-0.05,0.05,100);
% %     else
% %         scaleList = 0.378765027155983;  + linspace(-0.00035,0.00035,100);
% %         thetaList = 70.273014828525959;  + linspace(-0.1,0.1,300);
% %     end
    




%     scaleList = 0.379170077661033;  + linspace(-0.003,0.003,100);
%     thetaList = 70.344452266382433;  + linspace(-0.05,0.05,100);
    
else
    scaleList = 0.3741 + linspace(-0.001,0.001,50);
    thetaList = 72.9646  + linspace(-0.12,0.12,50);
    
% %     scaleList = 0.3741 + linspace(-0.2,1,50);
% %     thetaList = 72.9646  + linspace(-2.12,20.12,50);
end


[scaleMat, thetaMat] = meshgrid(scaleList, thetaList);

scaleTheta = [scaleMat(:) thetaMat(:)];

shortLenScaled = 1.*shortLen;
validId = 1 : ctrlPtNum;

if 1
    Error = [];
    for j = 1 : size(scaleTheta,1)
        
        Err = CalcErr(imgRng, intrMat, Ptt, ctrlPt, shortLenScaled, validId, scaleTheta(j,:));
        Error(j,:) = mean(Err);
%         Err = [];
        
        
    end
    
    ErrMat = reshape(Error, size(scaleMat));
    
    [~, minId] = min(Error);
    try
        figure,surf(scaleMat, thetaMat, ErrMat);
    catch
        close;
        sgkbj = 1;
    end
else
    scaleTheta0 = [0.384 70.3202];
    Err0 = CalcErr(imgRng, intrMat, Ptt, ctrlPt, shortLenScaled, validId, scaleTheta0);
    options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter','MaxFunEvals',10000,'TolX',1e-18, 'OptimalityTolerance', 1e-17,'FunctionTolerance', 1e-16,'StepTolerance',1e-15,'MaxFunEvals',10000, 'MaxIterations',5000000);
    [vec,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) CalcErr(imgRng, intrMat, Ptt, ctrlPt, shortLenScaled, validId, X),[scaleTheta0],[],[],options);
    
    Err1 = CalcErr(imgRng, intrMat, Ptt, ctrlPt, shortLenScaled, validId, vec);
    
end
if 1 % usePNP
    scaleThetaOpt = [scaleList(1) thetaList(1)];  [0.3847   70.3354];
else
    scaleThetaOpt = [0.3741 72.9646];
end
[ErrOpt, xyzOpt, inCell] = CalcErr(imgRng, intrMat, Ptt, ctrlPt, shortLenScaled, validId, scaleThetaOpt);



 [ErrOptPNP, rotVecList, a, rotAng, rt] = CalcErr2(inputDir, dirInfo, xyzOpt, Ptt, intrMat, imgRng, 0,[], scaleThetaOpt, []);
 [C, dist] = fitline(a([1 3],:));
xList = linspace(min(a(1,:)), max(a(1,:)), 200);
yList = (-C(1).*xList - C(3))./C(2);



% figure,subplot(1,3,1);plot(a(1,:), a(2,:));hold on;plot(xList,yList);axis equal;grid on;title('cam path (m)');legend('raw path','fit path');subplot(1,3,2),plot(a(3,:));grid on; title('cam height to gnd (m)'); subplot(1,3,3),plot(90 - rad2deg(rotAng));title('cam rot (deg)');grid on; % subplot(2,2,3),plot(a');grid on;subplot(2,2,4),plot(b');grid on;
% figure,subplot(1,3,1);plot(a(1,:), a(3,:));hold on;plot(xList,yList);axis equal;grid on;title('cam path (m)');legend('raw path','fit path');subplot(1,3,2),plot(a(2,:));grid on; title('cam height to gnd (m)'); subplot(1,3,3),plot(90 - rad2deg(rotAng));title('cam rot (deg)');grid on; % subplot(2,2,3),plot(a');grid on;subplot(2,2,4),plot(b');grid on;
for i = 1 : size(rt,3)-1
    TPrv = (rt(:,:,i));
    TCur = (rt(:,:,i+1)); 
    deltaT = TCur \ TPrv;
    rPrv0 = roty(-10)*(rt(1:3,1:3,1));
    deltaT0 = TCur(1:3,1:3) \ rPrv0;
    deltaRotAng(i,1) = rad2deg(norm(rodrigues(deltaT0(1:3,1:3))));
    deltaRotAxis(:,i) = rodrigues(deltaT(1:3,1:3))./norm(rodrigues(deltaT(1:3,1:3)));
end




for i = 1 : size(rt, 3)
        if i == 1
            Tc2w(:,:,i) = eye(3);
            Tc2w_base = inv(rt(1:3,1:3,i));
        else
            Tc2w(:,:,i) = Tc2w_base*rt(1:3,1:3,i);
        end
%     Tc2w(:,:,i) = rt(1:3,1:3,i);
end


c2w0 = Tc2w(:,:,1);
Tb2w = eye(3);
b2w0 = Tb2w;
turnId = [];
for i = 1 : size(Tc2w,3) - 1
    
    delta_c_a2b = inv(Tc2w(:,:,i+1)) * c2w0;
    if delta_c_a2b(1,3) < 0
        delta_b_a2b = roty(-rad2deg(norm(rodrigues(delta_c_a2b))));
    else
        delta_b_a2b = roty(rad2deg(norm(rodrigues(delta_c_a2b))));
        turnId = [turnId;i];
    end
    Tb2w(:,:,i + 1) = b2w0 * inv(delta_b_a2b);
end
for i = 1 : size(Tb2w,3)
%     Tc2w(:,:,i) = inv(Tc2w(:,:,i));
    Tw2b_all(:,:,i) = [(Tb2w(:,:,i))' [0 0 0]';0 0 0 1];
    Tc2w_all(:,:,i) = [(Tc2w(:,:,i)) [0 0 0]';0 0 0 1];
end
[b2c_solve, err] = TSAIleastSquareCalibration(Tc2w_all, Tw2b_all);



for i = 1 : size(rt,3)
   
    TCur = (rt(:,:,i)); 
    rPrv0 = roty(-10)*(rt(1:3,1:3,1));
    deltaT0(:,:,i) = TCur(1:3,1:3) \ rPrv0;
    deltaRotAng(i,1) = rad2deg(norm(rodrigues(deltaT0(1:3,1:3,i))));
end

deltaRotAng = deltaRotAng - deltaRotAng(1);
% camPoseC2K = rt;




baseRotAng = min(rad2deg(rotAng))-0;
if 0
    deltAng = [rad2deg(rotAng) - rad2deg(rotAng(1))];
else
    deltAng0 = [rad2deg(rotAng) - rad2deg(rotAng(1))];
    deltAng = deltaRotAng;
end
for i = 1 : size(rt,3)
    camPoseC2K(:,:,i) = [rt(1:3,1:3,i) [0;0;0]; 0 0 0 1];
%     bodyPoseK2C(:,:,i) = [roty(baseRotAng + deltAng(i))' [0;0;0]; 0 0 0 1];
    bodyPoseK2C(:,:,i) = [roty(rad2deg(rotAng(i)))' [0;0;0]; 0 0 0 1];
end
[b2c_init, b2c_error] = TSAIleastSquareCalibration(camPoseC2K, bodyPoseK2C);


for i = 1 : size(rt,3)
   camK2C(:,:,i) = inv(rt(1:3,1:3,i));
%    camK2C_comp(:,:,i) = b2c_init(1:3,1:3)*roty(baseRotAng +0 + deltAng(i))';
   camK2C_comp(:,:,i) = b2c_init(1:3,1:3)*roty(rad2deg(rotAng(i)))';
   err(i,:) = rad2deg(norm(rodrigues(camK2C(:,:,i)'*camK2C_comp(:,:,i))));
end


b2cVec = [0;0;0];
err = optB2CInit(rt, baseRotAng, deltAng, b2cVec);



for i = 1 : size(rt, 3) - 1
    
    TCur_cam = rt(:,:,i+1);
    TPrv_cam = rt(:,:,i);
    
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

Err0 = CostFunc(deltaCam, deltaBody, b2cVec);

options1= optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter','MaxFunEvals',10000,'TolX',1e-20, 'OptimalityTolerance', 1e-20,'FunctionTolerance', 1e-20,'StepTolerance',1e-20,'MaxFunEvals',10000, 'MaxIterations',5000000);
[b2cVecOpt,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) CostFunc(deltaCam, deltaBody, X),[b2cVec],[],[],options1);


Err1 = CostFunc(deltaCam, deltaBody, b2cVecOpt);




options1= optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter','MaxFunEvals',10000,'TolX',1e-18, 'OptimalityTolerance', 1e-17,'FunctionTolerance', 1e-16,'StepTolerance',1e-15,'MaxFunEvals',10000, 'MaxIterations',5000000);
[b2cVecOpt,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) optB2CInit(rt, baseRotAng, deltAng, X),[b2cVec],[],[],options1);
err1 = optB2CInit(rt, baseRotAng, deltAng, b2cVecOpt);
%  
% err = optB2CInit2(Tw2b_all, Tc2w_all, b2cVec);
% options1= optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter','MaxFunEvals',10000,'TolX',1e-18, 'OptimalityTolerance', 1e-17,'FunctionTolerance', 1e-16,'StepTolerance',1e-15,'MaxFunEvals',10000, 'MaxIterations',5000000);
%  [b2cVecOpt,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) optB2CInit2(Tw2b_all, Tc2w_all, X),[b2cVec],[],[],options1);
% err1 = optB2CInit2(Tw2b_all, Tc2w_all, b2cVecOpt);


[rotAxis, rotAngs] = NormalizeVector(rotVecList');
optVec0 = [mean(rotVecList')'; -C(1)/C(2); -C(3)/C(2); median(a(3,:))];

% optVec0 = optVec0(1:3);



optVec00 = optVec0;

optVec0 = [mean(rotAxis)'; -C(1)/C(2); -C(3)/C(2); median(a(3,:))];%  rotAngs ];

extraVec = []; optVec0(4:6); [];

optVec0 = optVec0(1:end-length(extraVec));

if 1
    meanAx = mean(rotAxis)'./norm(mean(rotAxis)');
    
    fi = acos(meanAx(3));
    theta = asin(meanAx(2)/sin(fi));
    xx = sin(fi)*cos(theta);
    yy = sin(fi)*sin(theta);
    zz = cos(fi);
    meanAx_ = [-abs(xx); abs(yy); -abs(zz)];
    meanAx_ = [abs(xx); abs(yy); abs(zz)];
    
    optVec0 = [scaleThetaOpt'; [fi; theta];rotAngs; a(:)];
    
    optVec0 = [scaleThetaOpt'; b2cVecOpt;baseRotAng + 0 + deltAng; a(:)];
    
end

[ErrOpt0, rtOpt0, aOpt0, rotAngOpt0, rt0] = CalcErr2(inputDir, dirInfo, xyzOpt, Ptt, intrMat, imgRng, 0,rotAngs, scaleThetaOpt, optVec0);

 [COpt0, distOpt0] = fitline(aOpt0(1:2,:));
 yListOpt = (-COpt0(1).*xList - COpt0(3))./COpt0(2);
 
 
 
% figure,subplot(1,3,1);plot(aOpt0(1,:), aOpt0(2,:));hold on;plot(xList,yListOpt);axis equal;grid on;title('cam path (m)');legend('raw path','fit path');subplot(1,3,2),plot(aOpt0(3,:));grid on; title('cam height to gnd (m)'); subplot(1,3,3),plot(90 - rad2deg(rotAngOpt0));title('cam rot (deg)');grid on; % subplot(2,2,3),plot(a');grid on;subplot(2,2,4),plot(b');grid on;

 
 
% figure,subplot(1,2,1);plot(reProjErr);subplot(1,2,2);plotQuiver(NHomo1',[1 0 0 ],1,1)
% figure,plotQuiver(rotAxis',[1 0 0 ],1,1)
% options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter','MaxFunEvals',10000,'TolX',1e-18, 'OptimalityTolerance', 1e-17,'FunctionTolerance', 1e-16,'StepTolerance',1e-15,'MaxFunEvals',10000, 'MaxIterations',5000000);
options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter','MaxFunEvals',3000,'TolX',1e-8, 'OptimalityTolerance', 1e-8,'FunctionTolerance', 1e-8,'StepTolerance',1e-8,'MaxFunEvals',3000, 'MaxIterations',5000000);
 [vec,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) CalcErr2(inputDir, dirInfo, xyzOpt, Ptt, intrMat, imgRng, 0,rotAngs, scaleThetaOpt, X),[optVec0],[],[],options);
    
[ErrOpt1, rtOpt1, aOpt1, rotAngOpt1, rt1] = CalcErr2(inputDir,dirInfo, xyzOpt, Ptt, intrMat, imgRng, 0,rotAngs, scaleThetaOpt, vec);

rMatNew = rodrigues(vec(3:5));






img2 = imread('D:\Temp\20200829\imagel5300_081500_rectify.jpg');
img1 = imread('D:\Auto\data5\081500_all\imagel5300_081500_rectify.jpg');
img11 = ImgTransform(img1, intrMat, rMatNew');
figure,imshow(img11)
figure,imshowpair(img11, img2)

testBirdView2(intrMat,img11)
testBirdView2(intrMat,img2)


xx1 = sin(vec(3))*cos(vec(4));
yy1 = sin(vec(3))*sin(vec(4));
zz1 = cos(vec(3));
meanAx_1 = [-abs(xx1); abs(yy1); -abs(zz1)];


for i = 1 : size(rt1,3)-1
    TPrv1 = (rt1(:,:,i));
    TCur1 = (rt1(:,:,i+1)); 
    deltaT1 = TCur1 \ TPrv1;
    deltaRotAxis1(:,i) = rodrigues(deltaT1(1:3,1:3))./norm(rodrigues(deltaT1(1:3,1:3)));
end


figure,plot([ErrOpt0 ErrOpt1]);

% rMatOpt = rodrigues(vec(1:3));


asfjg = 1;

if 0
    meanG = mean(gNorm',1)'./norm(mean(gNorm',1)');
    rMatNewMean = roty(29)*rotz(180)*CorrectRot([0;0;1], meanG);img1 = ImgTransform(rgb2gray(imgLL), intrMat, rMatNewMean);testBirdView2(intrMat,img1)
    
    
    R = rodrigues(deg2rad(2).*mean(rotAxis',1)'./norm(mean(rotAxis',1)'));
    R2 = roty(1)*[R(:,1) R(:,3)  R(:,2)];
    img1 = ImgTransform(rgb2gray(imgLL), intrMat, R2');testBirdView2(intrMat,img1)
end


end

function varargout = CalcErr(imgRng, intrMat, Ptt, ctrlPt,shortLenScaled, validId, scaleTheta)
global usePNP

scale = scaleTheta(1);
theta = scaleTheta(2 ); %70; 90; 120;

Err = [];

ctrlPtScaled = scale.*ctrlPt;
ctrlPtScaledValid = ctrlPtScaled(validId);
xyz1 = [ctrlPtScaledValid'  zeros(length(validId),1) ones(length(validId), 1)];
xyz2 = [[cosd(theta).*shortLenScaled : (ctrlPtScaledValid(2)-ctrlPtScaledValid(1)) : cosd(theta).*shortLenScaled + (length(validId)-1)*(ctrlPtScaledValid(2)-ctrlPtScaledValid(1))]'...
    sind(theta).*shortLenScaled.*ones(length(validId),1) ones(length(validId), 1)];

xyz1 = [xyz1(:,1) xyz1(:,3) xyz1(:,2)];
xyz2 = [xyz2(:,1) xyz2(:,3) xyz2(:,2)];
% xyz1(:,2) = -xyz1(:,2);
% xyz2(:,2) = -xyz2(:,2);

xyz = {};
for i = 1 :  length(imgRng) % length(validId)  %
    validId0 = Ptt{i,2};
    
    
    
    xyz_ = [reshape([xyz2(validId0,1)'; xyz1(validId0,1)'], [], 1) reshape([xyz2(validId0,2)'; xyz1(validId0,2)'], [], 1) reshape([xyz2(validId0,3)'; xyz1(validId0,3)'], [], 1)];
    
    pt2d = Ptt{i,1};
    idAll = [1:size(pt2d,1)];
    try
        if usePNP
            [rtTemp, outlierId] = posest(pt2d, xyz_, 0.9, intrMat, 'repr_err');
            
            
            inCell{i,1} = setdiff(1:size(pt2d,1), outlierId);
            
            [ptIcsTemp_pnp, pt3d_pnp] = TransformAndProject(xyz_, intrMat, rodrigues(rtTemp(1:3)), rtTemp(4:6));
            [~, err0] = NormalizeVector(ptIcsTemp_pnp - pt2d);
            err = err0(setdiff(idAll, outlierId'));
        else
            outlierId
            pt2dMetric = (inv(intrMat)*pextend(pt2d'))';
            xz2 = xyz_;
            [H,Hnorm,inv_Hnorm] = compute_homography(xz2', pt2dMetric');
            
            
%             mappedPt = inv(H)*xz2';
%             mappedPt = intrMat*mappedPt;
%             ptIcsTemp_gt = [mappedPt(1,:)./mappedPt(3,:); mappedPt(2,:)./mappedPt(3,:)]';
%             mappingErr = ptIcsTemp_gt - pt2d;
            
            
            
            
            [sol1, H1] = DecomposeHomoGraphyMat((H), eye(3),eye(3));
            for e = 1 : 4
                rTmp = sol1(1:3,1:3,e);
                tTmp = sol1(1:3,4,e);
                projErr1(e,1) = ProjectErr3dTo2d(pt2d, xz2, intrMat, rTmp, tTmp);
            end
            [~, idH1] = min(projErr1);
            RHomo1 = sol1(1:3,1:3,idH1);
            tHomo1 = sol1(1:3,4,idH1);
            NHomo1 = sol1(1:3,5,idH1);
            T1 = [RHomo1 tHomo1; 0 0 0 1];
            [ptIcsTemp_gt, pt3d] = TransformAndProject(xz2, intrMat, T1(1:3,1:3), T1(1:3,4));
            [~, err] = NormalizeVector(ptIcsTemp_gt - pt2d);
        end
    catch
        err = 100;
    end
    Err(i,1) = mean(err);
    xyz{i,1} = xyz_;
end


varargout{1} = Err;
if (nargout > 1)
    varargout{2} = xyz;
    varargout{3} = inCell;
end

end
function theta = CalcDegree(vec1, vec2)
if 0
    theta = min([rad2deg(acos(dot(vec1, vec2)/norm(vec1)/norm(vec2))); rad2deg(acos(dot(vec1, -vec2)/norm(vec1)/norm(vec2)))]);
else
    
    [vec1_norm, ~] = NormalizeVector(vec1);
    [vec2_norm, ~] = NormalizeVector(vec2);
    theta = min([rad2deg(acos(dot(vec1_norm', vec2_norm')')) rad2deg(acos(dot(vec1_norm', -vec2_norm')'))]')';
    
end
end
function y = pextend(x)
y = [x; ones(1,size(x,2))];
end
function rMatNew = CorrectRot(axis, gVec)
ang = CalcDegree(axis',gVec');
xVec = cross(axis,gVec);
xVec = xVec./norm(xVec);
zVec = cross(xVec, gVec);
zVec = zVec./norm(zVec);

rMatNew = [xVec'; gVec'; zVec'];
end
function [varargout] = CalcErr2(inputDir,dirInfo, xyzOpt, Ptt, intrMat, imgRng, draw, ang, scaleThetaOpt, optVec)
global v2c extraVec ctrlPt shortLenScaled inCell




inId = {};
ErrStack = [];


 %70; 90; 120;







 if ~isempty(optVec)
     optVec = [optVec; extraVec];
     rMatComm = rodrigues(optVec(1:3));
     
     if length(optVec) <=6
         rAxisComm = (optVec(1:3));
         kb = optVec(4:5);
         height = optVec(6);
         rotAngs = [];
         transMat = [];
     else
         
         scale = optVec(1);
         theta = optVec(2 );
         
         
         kb = [];
         height = [];
         
         if 0
             b2cVec = [];
             fi = optVec(3);
             theta1 = optVec(4);
             xx = sin(fi)*cos(theta1);
             yy = sin(fi)*sin(theta1);
             zz = cos(fi);
             
             rAxisComm = [-abs(xx); abs(yy); -abs(zz)];
             rAxisComm = [abs(xx); abs(yy); abs(zz)];
             
             rAxisComm = [rAxisComm];
             rotAngs = optVec(5:4+length(imgRng));
             transMat = reshape(optVec(5+length(imgRng):end),3, []);
         else
             
             b2cVec = optVec(3:5);
             rAxisComm = [0;1;0];
             rotAngs = optVec(6:5+length(imgRng));
             transMat = reshape(optVec(6+length(imgRng):end),3, []);
         end
     end
     %    ang = optVec(7:end);
     
 else
     scale = scaleThetaOpt(1);
     theta = scaleThetaOpt(2 );
     
 end



validId = 1 : length(ctrlPt);

ctrlPtScaled = scale.*ctrlPt;
ctrlPtScaledValid = ctrlPtScaled(validId);
xyz1 = [ctrlPtScaledValid'  zeros(length(validId),1) ones(length(validId), 1)];
xyz2 = [[cosd(theta).*shortLenScaled : (ctrlPtScaledValid(2)-ctrlPtScaledValid(1)) : cosd(theta).*shortLenScaled + (length(validId)-1)*(ctrlPtScaledValid(2)-ctrlPtScaledValid(1))]'...
    sind(theta).*shortLenScaled.*ones(length(validId),1) ones(length(validId), 1)];

xyz1 = [xyz1(:,1) xyz1(:,3) xyz1(:,2)];
xyz2 = [xyz2(:,1) xyz2(:,3) xyz2(:,2)];

for j = 1 : size(xyzOpt,1)

    
    validId0 = Ptt{j,2};
     xz2 = [reshape([xyz2(validId0,1)'; xyz1(validId0,1)'], [], 1) reshape([xyz2(validId0,2)'; xyz1(validId0,2)'], [], 1) reshape([xyz2(validId0,3)'; xyz1(validId0,3)'], [], 1)];
   
    
    
    pt2d = Ptt{j,1};
    pt2dMetric = (inv(intrMat)*pextend(pt2d'))';
%     xz2 = xyzOpt{j,1};
    
    validIds = Ptt{j, 2};
    inlier = sort([2*validIds 2*validIds-1]);
    [rtTemp, outlierId] = posest(pt2d, xz2, 0.9, intrMat, 'repr_err');
    
    rtTemp0 = rtTemp;
    inId{j,1} = setdiff(inlier', inlier(outlierId));
    
    inliers = setdiff(1:size(pt2d,1), outlierId);
    inliers = inCell{j,1};
    
    
    Rg2c = rodrigues(rtTemp(1:3));
    Tg2c = rtTemp(4:6);
    rt(:,:,j) = inv([Rg2c Tg2c; 0 0 0 1]);
    
    if isempty(optVec)
        
        
        gNorm(:,j) = Rg2c(:,2);
        xNorm(:,j) = Rg2c(:,1);
        zNorm(:,j) = Rg2c(:,3);
        % local 2 world
        rotVecList(:,j) = rodrigues(rt(1:3,1:3,j));
        rotAxis(:,j) = rodrigues(rt(1:3,1:3,j))./norm(rodrigues(rt(1:3,1:3,j)));
        rotAng(j,:) = norm(rodrigues(rt(1:3,1:3,j)));
        
        if draw
            imgLL = imread(fullfile(inputDir, dirInfo(imgRng(j)).name)); rMatNew = rotx(180)*roty(27)*rotz(180)*CorrectRot([0;1;0], -Rg2c(:,3));img1 = ImgTransform(rgb2gray(imgLL), intrMat, rMatNew);testBirdView2(intrMat,img1)
            drawnow;
        end
        
        %     img1 = ImgTransform(rgb2gray(imgLL), intrMat, rMatNew);
        if 0
            imgLL = imread(fullfile(inputDir, dirInfo(imgRng(j)).name));
            rMatNew = roty(22)*rotz(180)*CorrectRot(gNorm(:,j));img1 = ImgTransform(rgb2gray(imgLL), intrMat, rMatNew);testBirdView2(intrMat,img1)
        end
        
    else
        
%         rtTemp = reprojCost(rMatComm, kb, height, intrMat, pt2d(inliers,:), xz2(inliers,:), rt(:,:,j), ang(j));
        rtTemp = reprojCost(rAxisComm, kb, height, intrMat, pt2d(inliers,:), xz2(inliers,:), rt(:,:,j), ang(j), rotAngs(j), transMat(:,j), b2cVec);
        
        if nargout > 1
            Rg2c_ = rodrigues(rtTemp(1:3));
            Tg2c_ = rtTemp(4:6);
            rt_(:,:,j) = inv([Rg2c_ Tg2c_; 0 0 0 1]);
            rotAng_(j,:) = norm(rodrigues(rt_(1:3,1:3,j)));
            
            if draw
                imgLL = imread(fullfile(inputDir, dirInfo(imgRng(j)).name)); rMatNew = rotx(180)*roty(27)*rotz(180)*CorrectRot([0;1;0], -Rg2c_(:,3));img1 = ImgTransform(rgb2gray(imgLL), intrMat, rMatNew);testBirdView2(intrMat,img1)
                drawnow;
            end
        end
    end
    [x_1, ~] = TransformAndProject(xz2, intrMat, rodrigues(rtTemp(1:3)), rtTemp(4:6));

    Hc = (((rodrigues(rtTemp(1:3))+(rtTemp(4:6))*[0;1;0]')));
%     Hc = (((rodrigues(rtTemp(1:3))+(rtTemp(4:6))*[0;0;1]')));
% %     Hc = (((rt(1:3,1:3,j)+(rtTemp(4:6))*[0;0;1]')));
    Hc_inv = inv(Hc);
    x__ = pflat(intrMat*Hc*xz2');
    x_2 = x__(1:2,:)';
    [~, reprojError] = NormalizeVector(x_1 - pt2d);
    
    ErrStack = [ErrStack; (reprojError(inliers))];
%     ErrStack = [ErrStack; reprojError];
    if isempty(optVec)
        if 0
             imgLL = imread(fullfile(inputDir, dirInfo(imgRng(j)).name)); figure,imshow(imgLL);hold on;plot(pt2d(:,1),pt2d(:,2),'or');plot(x_1(:,1),x_1(:,2),'sg');plot(x_2(:,1),x_2(:,2),'*b');
             rMatNew = roty(22)*rotz(180)*CorrectRot(gNorm(:,j));img1 = ImgTransform(rgb2gray(imgLL), intrMat, rMatNew);testBirdView2(intrMat,img1)

        end
        
        try
            dsf
            [H,Hnorm,inv_Hnorm] = compute_homography(xz2', pt2dMetric');
            % [H, inliers2] = ransacfithomography(pt2dMetric', xz2', 0.0001);
        catch
            H = inv(Hc);
        end
        mappedPt = inv(H)*xz2';
        mappedPt = intrMat*mappedPt;
        mappedPt1 = [mappedPt(1,:)./mappedPt(3,:); mappedPt(2,:)./mappedPt(3,:)]';
        mappingErr = mappedPt1 - pt2d;
        %     figure,subplot(1,2,1);imshow(imgLL);hold on;plot( pt2d(:,1), pt2d(:,2), 'or');plot(mappedPt1(:,1), mappedPt1(:,2),'.g');
        
        [sol1, H1] = DecomposeHomoGraphyMat((H), eye(3),eye(3));
        projErr1 = [];
        for e = 1 : 4
            rTmp = sol1(1:3,1:3,e);
            tTmp = sol1(1:3,4,e);
            projErr1(e,1) = ProjectErr3dTo2d(pt2d, xz2, intrMat, rTmp, tTmp);
        end
        [reProjErr(j,1), idH1] = min(projErr1);
        RHomo1 = sol1(1:3,1:3,idH1);
        tHomo1 = sol1(1:3,4,idH1);
        NHomo1(:,j) = sol1(1:3,5,idH1);
        T1 = [RHomo1 tHomo1; 0 0 0 1];
        
        tErr = (rt(:,:,j))*T1;
        
        
        rt2(:,:,j) = inv(T1); % local 2 world
        rotAxis2(:,j) = rodrigues(rt2(1:3,1:3,j))./norm(rodrigues(rt2(1:3,1:3,j)));
        
        
        [ptIcsTemp_gt, pt3d] = TransformAndProject(xz2, intrMat, T1(1:3,1:3), T1(1:3,4));
        %     subplot(1,2,2),imshow(imgLL);hold on;plot(pt2d(:,1), pt2d(:,2), 'or');plot(ptIcsTemp_gt(:,1), ptIcsTemp_gt(:,2), 'xg');title(num2str(min(projErr1)));
        
    end
end

if isempty(optVec)
    a = squeeze(rt(1:3,4,:)); b = squeeze(rt2(1:3,4,:));
    varargout{1} = ErrStack;
    varargout{2} = rotVecList;
    varargout{3} = a;
    varargout{4} = rotAng;
    varargout{5} = rt;
else
    
    if nargout == 1
         varargout{1} = ErrStack;
        
    else
        a_ = squeeze(rt_(1:3,4,:)); %b = squeeze(rt2(1:3,4,:));
        varargout{1} = ErrStack;
        varargout{2} = rt_;
        varargout{3} = a_;
        varargout{4} = rotAng_;
        varargout{5} = rt_;
    end
    
end

% [C, dist] = fitline(a(1:2,:));
% xList = linspace(min(a(1,:)), max(a(1,:)), 200);
% yList = (-C(1).*xList - C(3))./C(2);

% figure,subplot(2,2,1);plot(a(1,:), a(2,:));hold on;plot(xList,yList);axis equal;grid on;subplot(2,2,2),plot(b(1,:), b(2,:));axis equal;grid on;subplot(2,2,3),plot(a');grid on;subplot(2,2,4),plot(b');grid on;
% figure,subplot(1,3,1);plot(a(1,:), a(2,:));hold on;plot(xList,yList);axis equal;grid on;title('cam path (m)');legend('raw path','fit path');subplot(1,3,2),plot(a(3,:));grid on; title('cam height to gnd (m)'); subplot(1,3,3),plot(90 - rad2deg(rotAng));title('cam rot (deg)');grid on; % subplot(2,2,3),plot(a');grid on;subplot(2,2,4),plot(b');grid on;

end
% function poseVec = reprojCost(rMatComm, kb, height, intrMat, pt2d, xz2, rtRef,ang)
function poseVec = reprojCost(rAxisComm, kb, height, intrMat, pt2d, xz2, rtRef,ang, rotAngs, transMat, b2cVec)

xList = linspace(rtRef(1,4) - 0.2, rtRef(1,4) + 0.2, 100);
xList = linspace(rtRef(1,4) - 1, rtRef(1,4) + 1, 100);

if ~isempty(kb)
    yList = kb(1).*xList + kb(2);
end
err = zeros(size(pt2d,1), length(xList));

angList = linspace(ang - deg2rad(0.5), ang + deg2rad(0.5), 100);

if 0
    for i = 1 : length(xList)
        rtTemp =  inv([rMatComm [xList(i) yList(i) height]'; 0 0 0 1]);
        if 1
            [ptIcsTemp_gt, pt3d] = TransformAndProject(xz2, intrMat, rtTemp(1:3,1:3), rtTemp(1:3,4));
        else
            Hc = (((rtTemp(1:3,1:3)+(rtTemp(1:3,4))*[0;0;1]')));
            % %     Hc = (((rt(1:3,1:3,j)+(rtTemp(4:6))*[0;0;1]')));
            Hc_inv = inv(Hc);
            x__ = pflat(intrMat*Hc*xz2');
            x_2 = x__(1:2,:)';
            ptIcsTemp_gt = x_2;
            %         [~, reprojError] = NormalizeVector(x_1 - pt2d);
            
        end
        
        [~, err(:,i)] = NormalizeVector(ptIcsTemp_gt - pt2d);
    end
    
    
    [minErr, bestId] = min(mean(err));
    
    rtBest = inv([rMatComm [xList(bestId) yList(bestId) height]'; 0 0 0 1]);
    
    poseVec = [rodrigues(rtBest(1:3,1:3)); rtBest(1:3,4)];
    
elseif 0
    
    errOpt = zeros(length(angList), 1);
    optimXs = zeros(length(angList), 1);
    for j = 1 : length(angList)
        
        angTemp = angList(j);
        [optimX, ~, exitFlag] = fminbnd(@ObjectiveFunction, xList(1), xList(end), optimset('TolX',0.0000001,'Display','off'));
        
        optimXs(j,1) = optimX;
        errOpt(j,1) = ObjectiveFunction(optimX);
        
        
    end
    
    [minErr, bestId] = min(errOpt);
    rtBest =  inv([rodrigues(angList(bestId).*rAxisComm) [optimXs(bestId) (kb(1)*optimXs(bestId) + kb(2)) height]'; 0 0 0 1]);

    poseVec = [rodrigues(rtBest(1:3,1:3)); rtBest(1:3,4)];
 
elseif 0
    
%     rtTemp =  inv([rodrigues(angTemp.*rAxisComm) [X (kb(1)*X + kb(2)) height]'; 0 0 0 1]);
    
    rtBest =  inv([rodrigues(rotAngs.*rAxisComm) transMat; 0 0 0 1]);
    poseVec = [rodrigues(rtBest(1:3,1:3)); rtBest(1:3,4)];
    
else
    
    rtBest =  inv([rodrigues(deg2rad(rotAngs).*rAxisComm) transMat; 0 0 0 1]);
    rtBest(1:3,1:3) = rodrigues(b2cVec)*rtBest(1:3,1:3);
    poseVec = [rodrigues(rtBest(1:3,1:3)); rtBest(1:3,4)];
    
    afkh = 1;
end


    function err1 = ObjectiveFunction(X)
        
        rtTemp =  inv([rodrigues(angTemp.*rAxisComm) [X (kb(1)*X + kb(2)) height]'; 0 0 0 1]);
        [ptIcsTemp_gt, ~] = TransformAndProject(xz2, intrMat, rtTemp(1:3,1:3), rtTemp(1:3,4));
        
        [~, err2] = NormalizeVector(ptIcsTemp_gt - pt2d);
        err1 = mean(err2);
        
    end

end
function err = optB2CInit(rt, baseRotAng, deltAng, b2cVec)
b2c_init = rodrigues(b2cVec);
    for i = 1 : size(rt,3)
        camK2C(:,:,i) = inv(rt(1:3,1:3,i));
        camK2C_comp(:,:,i) = b2c_init(1:3,1:3)*roty(baseRotAng +0 + deltAng(i))';
        err(i,:) = rad2deg(norm(rodrigues(camK2C(:,:,i)'*camK2C_comp(:,:,i))));
    end

end
function err = optB2CInit2(Tw2b, Tc2w, b2cVec)
b2c_init = rodrigues(b2cVec);

    
a1 = b2c_init*Tw2b(1:3,1:3,1);


for i = 1 : size(Tw2b,3)
    
    id = i;
    err(i,1) = rad2deg(norm(...
        rodrigues(inv(Tc2w(1:3,1:3,id))*(a1(1:3,1:3)) *           inv(  b2c_init(1:3,1:3)*Tw2b(1:3,1:3,id) )    )...
        ));
    
    
end
    
    

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

