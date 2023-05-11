function [thetaProbP2C_, angOptP2C_, thetaExp_tmpP2C, gtP2C, thetaRngP2C_use] = New_OnlyP2(obj, pt2dPrv, pt2dCur, intrMat,thetaP2C,T_B2C, k2cRef)
global Rati_onlyP2

radius = 3;
existPt1 = Points.Point(pt2dPrv(:,1),pt2dPrv(:,2),intrMat);
existPt2 = Points.Point(pt2dCur(:,1),pt2dCur(:,2),intrMat);
thetaRngP2C_ = [deg2rad(-179) : obj.configParam.theta_sample_step : deg2rad(179)] + (thetaP2C);
thetaSampP2C_ = thetaRngP2C_ - thetaP2C;
% [thetaProbP2C_, angOptP2C_, ~] = UpdateModalAngleOnlyP2NewVersion_(rad2deg(thetaRngP2C_),existPt1,existPt2,obj.configParam.reproj_sigma,intrMat,T_B2C);
[thetaProbP2C_, angOptP2C_, ~] = UpdateModalAngleOnlyP2NewVersion_(rad2deg(thetaRngP2C_),existPt1,existPt2,radius,intrMat,T_B2C);
thetaRngP2C_use = (thetaRngP2C_);
if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
    gtP2C = k2cRef;
else
    gtP2C = k2cRef - obj.refAngList(end);
    
end
thetaExp_tmpP2C = angOptP2C_ - k2cRef;
% % angOptP2C_ = deg2rad(dot(rad2deg(thetaRngP2C), thetaProbP2C_));   % 0.5 0.8 0.0
% % thetaExp_tmpP2C_ = deg2rad(dot(rad2deg(thetaSampP2C), thetaProbP2C_));

end
function [thetaProb, angOpt, thetaExp] = UpdateModalAngleOnlyP2NewVersion_(angleOrginRange,existPt1,existPt2,ambiguifyRadius,intrMat,T_B2C)
global Rati_onlyP2
tmp = zeros(1,length(angleOrginRange));
for angleIdx = 1 : length(angleOrginRange)
    
    bodyAngle =  angleOrginRange(angleIdx);% body_angle_between_two_imgs + var_angle ;
    cc = cosd(bodyAngle) ;
    ss = sind(bodyAngle) ;
    % p3D = [z.*p1;ones(1,length(z))];
    T_B = ...
        [
        cc , 0 , ss , 0. ;
        0 , 1 , 0 , 0.;
        -ss  , 0 , cc  , 0.;
        0 0 0 1];
    T = T_B2C*T_B*inv(T_B2C);
    
    
    matT = T(1:3,4);
    matSkewT = [   0      -matT(3)   matT(2); ...
        matT(3)     0      -matT(1); ...
        -matT(2)   matT(1)       0;];
    matR = T(1:3,1:3);
    matE = matSkewT*matR;
    matF = inv(intrMat)'*matE*inv(intrMat);
    L = matF*existPt1.pt;
    if 0
        [~,Llen] = NormalizeVector(L(1:2,:)');
        L = L./repmat(Llen',3,1);
        dltLen = abs(dot(existPt2.pt, L));
    end
    existPt2toL = abs(sum(existPt2.pt .* L,1)) ./ vecnorm(L(1:2,:));
    
    if 0
        points = lineToBorderPoints(L', [240 320]);    figure,imshow(zeros(240,320));hold on;line(points(:, [1,3])', points(:, [2,4])');    plot(existPt2.x,existPt2.y,'.r');
    end
    
    
    
    QQidx = find(vecnorm(L(1:2,:)) == 0);
    if size(QQidx,2)~=0
        existPt2.pt(QQidx);
        existPt1.pt(QQidx);
    end
    inlierIdx = find(existPt2toL < ambiguifyRadius);
    if 1
        if ~isempty(inlierIdx)
            intersectPt2toL = 2.*(ambiguifyRadius.^2 - existPt2toL(inlierIdx).^2);
            tmp(angleIdx) = tmp(angleIdx)+sum(intersectPt2toL);
%             tmp(angleIdx) = tmp(angleIdx)+var(existPt2toL);
        end
    else
        tmp(angleIdx) = tmp(angleIdx)+var(intersectPt2toL);
    end
end







thetaProb = tmp./sum(tmp);
rati = Rati_onlyP2; 0.95; 0.8;0.5;0.8; 0.5; 0.8; 0.9; 0;0.8;

thetaProb1 = thetaProb./max(thetaProb);
thetaProb2 = thetaProb1(thetaProb1 > rati);
thetaProb2 = thetaProb2./sum(thetaProb2);

thetaRng = deg2rad(angleOrginRange);

angOpt = deg2rad(dot(rad2deg(thetaRng(thetaProb1 > rati)), thetaProb2));   % 0.5 0.8 0.0
%  thetaExp = deg2rad(dot(rad2deg(thetaSamp(thetaProb1 > rati)), thetaProb2));
thetaExp = 0;
% thetaProbP2C_ = thetaProb;


% angleModal = obj.angleModal;
end