function Alpha = PredAlpha(theta01, theta02, rC2B, tC2B, prv2curBodyDisplacement,tx01,tz01)
C2B = [rC2B tC2B;0 0 0 1];
B2C = inv([rC2B tC2B;0 0 0 1]);
theta01 = double(rad2deg(theta01));
theta02 = double(rad2deg(theta02));   [-5:0.01:5];
thetaPrv = double(theta01 + 90);
d = prv2curBodyDisplacement*1000;
cnt = 1;
camInBody = C2B(1:3,1:3)*[0;0;0] + C2B(1:3,end);
bodyPrv = [tx01;0;tz01];
for i = 1:length(theta02)
    betaTmp = theta02(i)/2 + 90;
    thetaPrvTmp = theta02(i);  %2*iBeta;
    thetaKeyTmp = thetaPrvTmp + thetaPrv;
    xPrvTmp = d*cosd(betaTmp);
    zPrvTmp = d*sind(betaTmp);
    
    %     xPrvTmp = d*cosd(iBeta);
    %     zPrvTmp = d*sind(iBeta);
    
    tTmp = [roty(theta02(i))' [xPrvTmp;0;zPrvTmp];0 0 0 1]; % cur body to prv body
    % %     pKeyBodyTmp = roty(thetaPrv - 90)'*[xPrvTmp;0;zPrvTmp] + bodyPrv;
    %     camInBody = C2B(1:3,1:3)*[0;0;0] + C2B(1:3,end);
    curCamInPrvbody = tTmp(1:3,1:3)*camInBody + tTmp(1:3,end);
    curBodyXDirInPrvbody = tTmp(1:3,1:3)*[1;0;0]; % + tTmp(1:3,end);
    curCamInKeybody = roty(thetaPrv - 90)'*curCamInPrvbody + bodyPrv;
    curBodyXDirInKeybody = roty(thetaPrv - 90)'*curBodyXDirInPrvbody; %  + bodyPrv;
    curCamInKeyCam = B2C(1:3,1:3)*curCamInKeybody + B2C(1:3,end);
    curBodyXDirInKeyCam = B2C(1:3,1:3)*curBodyXDirInKeybody; % + B2C(1:3,end);
    keyCamInKeyCam = [0;0;0];  %B2C(1:3,1:3)*[0;0;0] + B2C(1:3,end);
    alphaVecCam1 = -(keyCamInKeyCam - curCamInKeyCam);
    alphaVecCam1 = alphaVecCam1./norm(alphaVecCam1);
    %     alphaVecCam2 =  B2C(1:3,1:3)*(roty(thetaKeyTmp - 90)'*[1;0;0]);
    alphaVecCam2 =  curBodyXDirInKeyCam; %B2C(1:3,1:3)*(roty(thetaKeyTmp - 90)'*[1;0;0]);
    alphaVecBody1 = -((C2B(1:3,1:3)*[0;0;0] + C2B(1:3,end)) - curCamInKeybody);
    alphaVecBody1 = alphaVecBody1./norm(alphaVecBody1);
    alphaVecBody2 = roty(thetaKeyTmp - 90)'*[1;0;0];
    alphaKeyTmp2 = double(rad2deg(acos(dot(alphaVecBody1,alphaVecBody2))));
    
   
    if norm(roty(alphaKeyTmp2)*alphaVecBody1 - alphaVecBody2) < norm(roty(alphaKeyTmp2)*alphaVecBody2 - alphaVecBody1)
        alpha = alphaKeyTmp2 - 0;
    else
%         alpha = 180-alphaKeyTmp2; 
        alpha = 360-alphaKeyTmp2;
    end
    camDirBody =  [cosd(alpha);0;sind(alpha)];
    camDirDir = B2C(1:3,1:3)*camDirBody;
    CamDirDir(:,cnt) = camDirDir;
%     err = min(norm([camDirDir + KeyCam2CurCamDir]),norm([camDirDir - KeyCam2CurCamDir]));
    
    alphaKeyTmp = rad2deg(acos(dot(alphaVecCam1,alphaVecCam2)));
%     AlphaTheta(cnt,:) = [alpha thetaKeyTmp-90];  [alphaKeyTmp thetaKeyTmp-90];
    XZ(cnt,:) = [xPrvTmp zPrvTmp];
    Alpha_(cnt,1) = alpha;
    cnt = cnt + 1;
end

Alpha = deg2rad(Alpha_);
% figure,plot(Alpha, deg2rad(theta02) );hold on;plot(Alpha, repmat(min(deg2rad(theta02)),length(theta02)));plot(Alpha, repmat(max(deg2rad(theta02)),length(theta02)))
end