function ChangeCoord2(ceilingPose,cbPose)

close all


if 0
    trials = 10;
    
    if 0
        load('D:\Temp\20190306\apriltag1\bak\cb.mat')
        load('D:\Temp\20190306\apriltag1\bak\roboPoseStamp_01_.mat')
        load('D:\Temp\20190306\imgs\use\use\initPt.mat');
        Ind = Ind(1:23);
    elseif 0
        load('D:\Temp\20190307\apriltag1\roboPoseStamp_01_.mat')
        load('D:\Temp\20190307\warped_image\use\initPt.mat')
        load('D:\Temp\20190307\warped_image\use\cb.mat')
    elseif 0
        load('D:\Temp\20190313\b2c\1\roboPoseStamp_01_.mat');
        load('D:\Temp\20190313\rt6\initPt.mat')
        load('D:\Temp\20190313\rt6\cb.mat')
        roboPosePoseMatStamp(:,10:12) = roboPosePoseMatStamp(:,10:12) - repmat(roboPosePoseMatStamp(1,10:12),size(roboPosePoseMatStamp,1),1);
        
    elseif 0
        load('D:\Temp\20190316\rt1\rect\cb.mat')
        load('D:\Temp\20190316\rt1\rect\initPt.mat')
        load('D:\Temp\20190316\b2c_zm002\roboPoseStamp_01_.mat')
    elseif 0
        load('D:\Temp\20190319\b2c\LR1\initPt.mat')
        load('D:\Temp\20190319\b2c\LR1\cb.mat')
        load('D:\Temp\20190319\b2c\roboPoseStamp_01_.mat')
    else
        load('D:\Temp\20190322\b2c\1\1\cb.mat')
        load('D:\Temp\20190322\b2c\1\1\initPt.mat')
        load('D:\Temp\20190322\b2c\apriltag1\roboPoseStamp_01_.mat')
    end
    
    
    N = 30;
    
    
    
    if 0
        load('D:\bk_20180627_\nextvpu\algorithm\output\calibration\cbccalib\results\iEye\zm_001\invT.mat')
        load('D:\Temp\20190305\18-02-2019-06-26-07\dump\imgs\use2\param\intrMat.mat');
    elseif 0
        load('D:\bk_20180627_\nextvpu\algorithm\output\calibration\cbccalib\results\iEye\zm_002\invT.mat')
        load('D:\Temp\20190313\rt6\intrMat.mat')
        invT(1,4) = invT(1,4)-15;
    elseif 0
        load('D:\bk_20180627_\nextvpu\algorithm\output\calibration\cbccalib\results\iEye\zm_001\invT.mat')
        load('D:\Temp\20190319\b2c\intrMat.mat')
    else
        load('D:\Temp\20190322\b2c\1\1\intrMat.mat')
        invT = [eye(3) [-55 ; -125; 170] ;0 0 0 1];
    end
    
    
    xyzCb = [0,0,0,0,0,0,0,0,0,70,70,70,70,70,70,70,70,70,140,140,140,140,140,140,140,140,140,210,210,210,210,210,210,210,210,210,280,280,280,280,280,280,280,280,280,350,350,350,350,350,350,350,350,350,420,420,420,420,420,420,420,420,420,490,490,490,490,490,490,490,490,490,560,560,560,560,560,560,560,560,560,630,630,630,630,630,630,630,630,630,700,700,700,700,700,700,700,700,700,770,770,770,770,770,770,770,770,770,840,840,840,840,840,840,840,840,840,910,910,910,910,910,910,910,910,910;0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560;0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]';
    % % invT = inv(invT);
    
    
    c2b = invT;
    
    invInvT = inv(invT);
    
    ceilingPose_ = roboPosePoseMatStamp(Ind,1:12);
    if 1
        ceilingPose_1 = InversePoseMat(ceilingPose_);ceilingPose_1 = transFormPose(ceilingPose_1, [rotz(-90) [0 0 0]';0 0 0 1]);ceilingPose_1 = InversePoseMat(ceilingPose_1);figure,plotPath(ceilingPose_1);
        ceilingPose_ = ceilingPose_1;
        
    end
    
    ceilingPose_ = rebaseStart(ceilingPose_, 1);
    [ceilingPose_,accumAng1] = reBase_inv(ceilingPose_,eye(4));
    
    
    
    
    
    
    
    cbPose = poseMat;
    % slamPoseMat2 = rebaseStart(poseMat, 1);
    % [roboPosePoseMatStampGnd2,accumAng1] = reBase_(slamPoseMat2(:,1:12),eye(4));
    % [check,accumAng1] = reBase_inv(roboPosePoseMatStampGnd2,eye(4));
    
    % cbPose = rebaseStart(roboPosePoseMatStampGnd2, 1);
    
    cbPose = flipud(cbPose);
    ceilingPose_ = flipud(ceilingPose_);
    
    if 0
        ceilingPose = transFormPose(ceilingPose_, ([reshape(ceilingPose_(1,1:9),3,3) ceilingPose_(1,10:12)';0 0 0 1])*inv(invInvT*[reshape(cbPose(1,1:9),3,3) cbPose(1,10:12)';0 0 0 1]));
    else
        ceilingPose = ceilingPose_;
        %     ceilingPose = transFormPose(ceilingPose_, [roty(-180) [0 0 0]';0 0 0 1]);
    end
    
else
    N = size(ceilingPose,1); 3000;
    initPt = [2;2];
    invT = [eye(3) [0 0 0]';0 0 0 1];
    trials = 1;
    c2b = invT;
end


poseMat1 = [];
for i = 1 : size(cbPose,1)
    
    r1 = reshape(cbPose(i,1:9),3,3);
    tempVec1 = rodrigues(r1)';
    tempVec1 = [tempVec1(1) -tempVec1(2) -tempVec1(3)];
    rotVec1(i,:) = tempVec1;
    t1 = cbPose(i,[10:12]);
    t1 = [t1(1) -t1(2) -t1(3)];
    rt1 = ([r1 t1';0 0 0 1]);
    poseMat1 = [poseMat1; [reshape(rt1(1:3,1:3),1,9) rt1(1:3,4)']];
    
    
end
cbPose = poseMat1;

figure,clf;plotPath(ceilingPose);
figure,clf;plotPath(cbPose);


initPt = flipud(initPt);

if 0
    initPt = initPt(1:end-1,:);
    cbPose = cbPose(1:end-1,:);
    ceilingPose = ceilingPose(1:end-1,:);
end
cbPose0 = cbPose;
initPt0 = initPt;
ceilingPose0 = ceilingPose;
for j = 1 : trials
    
    if trials ~=1
        id = randperm(size(cbPose0,1));
    else
        id = 1 : size(cbPose0,1);
    end
    
    
    
    cbPose = cbPose0(id(1:N),:);
    % %     initPt = initPt0(id(1:N),:);
    ceilingPose = ceilingPose0(id(1:N),:);
    
    sdbhj = 1;
    % vec0 = [0 0 0 invInvT(3,4) invInvT(1,4) invInvT(2,4)]';
    
    if 1
        vec0 = [rodrigues(invT(1:3,1:3));invT(1:3,4)];
    else
        vec0 = [rodrigues(invT(1:3,1:3))];
    end
    
    if 0
        err0 = errFunc(xyzCb, initPt, intrMat,cbPose, invT, ceilingPose, vec0);
        
        options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','off','MaxFunEvals',10000,'TolX',1e-8);%,'MaxFunEvals',10000);%, 'MaxIterations',5);
        [vec,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) errFunc(xyzCb, initPt, intrMat,cbPose, invT, ceilingPose, X),[vec0],[],[],options);%,data_1,obs_1)
        
        
        errOpt = errFunc(xyzCb, initPt, intrMat,cbPose, invT, ceilingPose, vec);
    elseif 0
        err0 = errFunc2(xyzCb, initPt, intrMat,cbPose, invT, ceilingPose, vec0);
        
        options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','off','MaxFunEvals',10000,'TolX',1e-18);%,'MaxFunEvals',10000);%, 'MaxIterations',5);
        [vec,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) errFunc2(xyzCb, initPt, intrMat,cbPose, invT, ceilingPose, X),[vec0],[],[],options);%,data_1,obs_1)
        
        
        errOpt = errFunc2(xyzCb, initPt, intrMat,cbPose, invT, ceilingPose, vec);
    else
        err0 = errFunc3(cbPose, ceilingPose, vec0);
        options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter','TolX',1e-8,'MaxFunEvals',10000, 'MaxIterations',20);
        [vec,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) errFunc3(cbPose, ceilingPose, X),[vec0],[],[],options);%,data_1,obs_1)
        errOpt = errFunc3(cbPose, ceilingPose, vec);
        
    end
    if trials == 1
        figure,plot([err0 errOpt])
    end
    if length(vec) > 3
        c2bOpt = ([rodrigues(vec(1:3,1)) vec(4:6,1);0 0 0 1]);
    else
        c2bOpt = ([rodrigues(vec(1:3,1)) invT(1:3,4);0 0 0 1]);
    end
    
    b2cOpt = inv(c2bOpt);
    b2c = inv(c2b);
    
    
    
    for i = 1 : size(cbPose,1)
        rtNewOpt = c2bOpt*([reshape(cbPose(i,1:9),3,3) cbPose(i,10:12)';0 0 0 1]);
        cbPoseNewOpt(i,:) = [reshape(rtNewOpt(1:3,1:3),1,9) rtNewOpt(1:3,4)'];
    end
    rt0 = AlignCoord(cbPoseNewOpt(:,10:12),ceilingPose(:,10:12));
    rt0 = [rt0;0 0 0 1];
    for i = 1: size(ceilingPose,1)
        rt = [reshape(ceilingPose(i,1:9),3,3) ceilingPose(i,10:12)';0 0 0 1];
        rt1 = rt0*rt;
        eulerCeiling(i,:) = rotMat2euler(rt1(1:3,1:3));
        ceilingPoseNew(i,:) = [reshape(rt1(1:3,1:3),1,9) rt1(1:3,4)'];
        
        
    end
    
    
    for i = 1: size(cbPoseNewOpt,1)
        rtCB = [reshape(cbPoseNewOpt(i,1:9),3,3) cbPoseNewOpt(i,10:12)';0 0 0 1];
        eulerCB(i,:) = rotMat2euler(rtCB(1:3,1:3));

    end
    
    %         meanErr = mean(ceilingPose(10:12)-cbPoseNewOpt(:,10:12));
    %         cbPoseNewOpt(:,10:12) = cbPoseNewOpt(:,10:12) + repmat(meanErr,size(cbPoseNewOpt,1),1);
    figure,plotPath(cbPoseNewOpt);plotPath(ceilingPoseNew)
    eulerCeiling([3323 3324 3328 3511 3513:3548],3) = eulerCeiling([3323 3324 3328 3511 3513:3548],3) - 2*pi;
    eulerCB(2474:end,3) = eulerCB(2474:end,3) + 2*pi;
    figure,plot([eulerCeiling(:,1) eulerCB(:,1)]);title('x')
    figure,plot([eulerCeiling(:,2) -eulerCB(:,2)]);title('y')
    figure,plot([eulerCeiling(:,3) -eulerCB(:,3)]);title('z')
    
    
    
    return;
    for i = 1 : size(cbPose,1)
        rtNewOpt = c2bOpt*inv([reshape(cbPose(i,1:9),3,3) cbPose(i,10:12)';0 0 0 1]);
        cbPoseNewOpt(i,:) = [reshape(rtNewOpt(1:3,1:3),1,9) rtNewOpt(1:3,4)'];
    end
    
    
    for i = 1 : size(cbPose,1)
        rtNew = c2b*inv([reshape(cbPose(i,1:9),3,3) cbPose(i,10:12)';0 0 0 1]);
        cbPoseNew(i,:) = [reshape(rtNew(1:3,1:3),1,9) rtNew(1:3,4)'];
    end
    
    cbPoseNewOpt_inRobotCs = InversePoseMat(cbPoseNewOpt);
    cbPoseNew_inRobotCs = InversePoseMat(cbPoseNew);
    
    figure,plotPath(cbPoseNewOpt_inRobotCs);plotPath(ceilingPose)
    
    
    
    AlignCoord(ceilingPose(:,10:12), cbPoseNewOpt_inRobotCs(:,10:12));
    
    if trials == 1
        figure,plot([cbPoseNew(:,11) cbPoseNewOpt(:,11)])
    end
    yAxisB2COpt = b2cOpt(1:3,1:3)*[0;1;0];
    yAxisB2C = b2c(1:3,1:3)*[0;1;0];
    yAxisDiff = CalcDegree(yAxisB2COpt,yAxisB2C);
    
    
    zAxisB2COpt = b2cOpt(1:3,1:3)*[0;0;1];
    zAxisB2C = b2c(1:3,1:3)*[0;0;1];
    zAxisDiff = CalcDegree(zAxisB2COpt,zAxisB2C);
    
    c2bMat(:,j) = rodrigues(c2bOpt(1:3,1:3));
end
% % invT = c2bOpt;
% % invTL = invT;
figure,plot(c2bMat');
figure,plotQuiver(c2bMat');
end

function err = errFunc(xyzCb, initPt, intrMat,cbPose, invT, ceilingPose, vec)

% vec = reshape(vec,6,2);

draw = 0;
err = [];
% invT = inv(invT);
if length(vec) > 3
    c2b = ([rodrigues(vec(1:3,1)) vec(4:6,1);0 0 0 1]);
else
    c2b = ([rodrigues(vec(1:3,1)) invT(1:3,4);0 0 0 1]);
end
% initBody = [rodrigues(vec(1:3,2)) vec(4:6,2);0 0 0 1];
for i = 1 : size(cbPose,1)
    if 0
        poseCam =  inv([reshape(cbPose(i,1:9),3,3) cbPose(i,10:12)';0 0 0 1]);
    end
    poseBody =  inv(c2b*[reshape(ceilingPose(i,1:9),3,3) ceilingPose(i,10:12)';0 0 0 1]);
    
    ptIcs = TransformAndProject(xyzCb, intrMat, poseBody(1:3,1:3), poseBody(1:3,4));
    cbPt = [initPt{i,1}(:) initPt{i,2}(:)];
    [~, errr] = NormalizeVector(ptIcs - cbPt);
    err = [err;errr];
    if draw
        figure,imshow(zeros(480,640));hold on;plot(cbPt(:,1),cbPt(:,2),'or');plot(ptIcs(:,1),ptIcs(:,2),'.g');
    end
    
end

end

function err = errFunc2(xyzCb, initPt, intrMat,cbPose, invT, ceilingPose, vec)

% vec = reshape(vec,6,2);

draw = 0;
err = [];
errStore = [];

% invT = inv(invT);

if length(vec) > 3
    c2b = ([rodrigues(vec(1:3,1)) vec(4:6,1);0 0 0 1]);
else
    c2b = ([rodrigues(vec(1:3,1)) invT(1:3,4);0 0 0 1]);
end
BaseCb = [reshape(cbPose(1,1:9),3,3) cbPose(1,10:12)';0 0 0 1];
BaseCeiling = [reshape(ceilingPose(1,1:9),3,3) ceilingPose(1,10:12)';0 0 0 1];
for i = 1 : size(cbPose,1) - 1
    if 0
        poseCam =  inv([reshape(cbPose(i,1:9),3,3) cbPose(i,10:12)';0 0 0 1]);
    end
    %     poseBody =  inv(c2b*[reshape(ceilingPose(i,1:9),3,3) ceilingPose(i,10:12)';0 0 0 1]);
    
    CurCb = [reshape(cbPose(i+1,1:9),3,3) cbPose(i+1,10:12)';0 0 0 1];
    CurCeiling = [reshape(ceilingPose(i+1,1:9),3,3) ceilingPose(i+1,10:12)';0 0 0 1];
    
    Cur2BaseCb = inv(BaseCb)*CurCb;
    
    Cur2BaseCeiling = inv(BaseCeiling)*CurCeiling;
    Cur2BaseCbCompose = inv(c2b)*Cur2BaseCeiling*c2b;
    
    CurCb_ = BaseCb*Cur2BaseCb;
    CurCb_Compose = BaseCb*Cur2BaseCbCompose;
    
    Cur2BaseCbAng(i,:) = rad2deg(norm(rodrigues(Cur2BaseCb(1:3,1:3))));
    Cur2BaseCbComposeAng(i,:) = rad2deg(norm(rodrigues(Cur2BaseCbCompose(1:3,1:3))));
    
    CurCb_Compose_inv = inv(CurCb_Compose);
    
    errMat = Cur2BaseCb - Cur2BaseCbCompose;
    errRot = rodrigues(Cur2BaseCb(1:3,1:3)) - rodrigues(Cur2BaseCbCompose(1:3,1:3));
    errTrans = errMat(1:3,4);
    errVec = [errRot; errTrans];
    
    ptIcs = TransformAndProject(xyzCb, intrMat, CurCb_Compose_inv(1:3,1:3), CurCb_Compose_inv(1:3,4));
    cbPt = [initPt{i+1,1}(:) initPt{i+1,2}(:)];
    [~, errr] = NormalizeVector(ptIcs - cbPt);
    err = [err;errr];
    errStore = [errStore; (ptIcs - cbPt)];
    
    if draw
        figure,imshow(zeros(480,640));hold on;plot(cbPt(:,1),cbPt(:,2),'or');plot(ptIcs(:,1),ptIcs(:,2),'.g');
    end
    
end
if draw
    figure,plot([Cur2BaseCbAng - Cur2BaseCbComposeAng],'-x');
    figure,plot(errStore(:,1),errStore(:,2),'+r');axis equal;
end


end


function err = errFunc3(cbPose, ceilingPose, vec)

% vec = reshape(vec,6,2);

draw = 0;
err = [];
errStore = [];

% invT = inv(invT);

if length(vec) > 3
    c2b = ([rodrigues(vec(1:3,1)) vec(4:6,1);0 0 0 1]);
else
    c2b = ([rodrigues(vec(1:3,1)) invT(1:3,4);0 0 0 1]);
end
BaseCb = [reshape(cbPose(1,1:9),3,3) cbPose(1,10:12)';0 0 0 1];
BaseCeiling = [reshape(ceilingPose(1,1:9),3,3) ceilingPose(1,10:12)';0 0 0 1];
for i = 1 : size(cbPose,1) - 1
    if 0
        poseCam =  inv([reshape(cbPose(i,1:9),3,3) cbPose(i,10:12)';0 0 0 1]);
    end
    %     poseBody =  inv(c2b*[reshape(ceilingPose(i,1:9),3,3) ceilingPose(i,10:12)';0 0 0 1]);
    
    CurCb = [reshape(cbPose(i+1,1:9),3,3) cbPose(i+1,10:12)';0 0 0 1];
    CurCeiling = [reshape(ceilingPose(i+1,1:9),3,3) ceilingPose(i+1,10:12)';0 0 0 1];
    
    Cur2BaseCb = inv(BaseCb)*CurCb;
    
    Cur2BaseCeiling = inv(BaseCeiling)*CurCeiling;
    Cur2BaseCbCompose = inv(c2b)*Cur2BaseCeiling*c2b;
    
    CurCb_ = BaseCb*Cur2BaseCb;
    CurCb_Compose = BaseCb*Cur2BaseCbCompose;
    
    Cur2BaseCbAng(i,:) = rad2deg(norm(rodrigues(Cur2BaseCb(1:3,1:3))));
    Cur2BaseCbComposeAng(i,:) = rad2deg(norm(rodrigues(Cur2BaseCbCompose(1:3,1:3))));
    
    CurCb_Compose_inv = inv(CurCb_Compose);
    
    errMat = Cur2BaseCb - Cur2BaseCbCompose;
    errRot = rodrigues(Cur2BaseCb(1:3,1:3)) - rodrigues(Cur2BaseCbCompose(1:3,1:3));
    errTrans = errMat(1:3,4);
    errVec = [errRot; errTrans];
    
    err = [err; norm(errTrans)];
    if 0
        ptIcs = TransformAndProject(xyzCb, intrMat, CurCb_Compose_inv(1:3,1:3), CurCb_Compose_inv(1:3,4));
        cbPt = [initPt{i+1,1}(:) initPt{i+1,2}(:)];
        [~, errr] = NormalizeVector(ptIcs - cbPt);
        err = [err;errr];
        errStore = [errStore; (ptIcs - cbPt)];
        
        if draw
            figure,imshow(zeros(480,640));hold on;plot(cbPt(:,1),cbPt(:,2),'or');plot(ptIcs(:,1),ptIcs(:,2),'.g');
        end
    end
end
if draw
    figure,plot([Cur2BaseCbAng - Cur2BaseCbComposeAng],'-x');
    figure,plot(errStore(:,1),errStore(:,2),'+r');axis equal;
end


end
