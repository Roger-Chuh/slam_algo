function [expectValue2, m] = OnlyP2_final(obj, ptPrv, ptCur, intrMat)

ambiguifyByAngle = 1;
deOutlierScale = 1;


f = intrMat(1); baseline = norm(obj.camModel.transVec1To2);
fbConst = f*baseline;
[princpPtL, princpPtR] = Get(obj.camModel, 'PinholePrincpPt', obj.scaleLvl);

T_B2C = obj.coordSysAligner.pctBody2Cam(1, 1).transformMat;
invTb2c = inv(T_B2C);
featuresNumber = size(ptPrv,1);
sys = SettingSystem(T_B2C,intrMat,baseline);


disparityBoundary = obj.setting2.disparityBoundary;
disparityIntervel = obj.setting2.disparityIntervel;
angleBoundary = obj.setting2.angleBoundary;
angleIntervel = obj.setting2.angleIntervel;
wx = obj.setting2.wx;
wy = obj.setting2.wy;
pyramidLevel = 1;
% settingTracking = SettingTracking(disparityBoundary,disparityIntervel,angleBoundary,angleIntervel,wx,wy,pyramidLevel);
angleBoundary2  = 179;
settingModal = SettingModal(disparityBoundary,disparityIntervel,angleBoundary2,angleIntervel);


% ptPrv_norm = inv(intrMat)*pextend(ptPrv');
% ptCur_norm = inv(intrMat)*pextend(ptCur');


existPt1 = Points.Point_final(ptPrv(:,1),ptPrv(:,2),intrMat);
existPt2 = Points.Point_final(ptCur(:,1),ptCur(:,2),intrMat);

[angleResult,depth] = AnglePredict2(existPt1.ptHomo, existPt2.ptHomo, sys);

m = AngleModal(settingModal,featuresNumber);%initialize


if ambiguifyByAngle
    angleBD = 0.1;
    ambiguifyRadius =0.5;
    [m.angleModal,tmp,tmp1,tmp2] = UpdateModalAngleOnlyP2NewVersion2(m,settingModal,angleResult,existPt1,existPt2,depth,angleBD,ambiguifyRadius,sys,deOutlierScale);
else
    ambiguifyRange = 0.3./settingModal.angleIntervel;
    [m.angleModal,tmp,tmp1,tmp2] = UpdateModalAngleOnlyP2NewVersion3(m,settingModal,angleResult,ambiguifyRange,deOutlierScale);
end

expectValue2 = dot(m.angleModal,settingModal.angleRange) ./ sum(m.angleModal);

end

function [angle_result,depth] = AnglePredict2(p1,p2,sys)
%simplify AnglePredict
r_cam = sys.r_cam;
tx = sys.tx;
ty = sys.ty;
tz = sys.tz;

%global
x = p1(1,:);
y = p1(2,:);

x_ = p2(1,:);
y_ = p2(2,:);


alpha1 = 2.*(-y.*tx).* (-y.*tz);
alpha2 = 2.*[(-y.*tx).* (-x.*tz+tx) + (tz+x.*tx).*(-y.*tz)];
alpha3 = 2.*(tz+x.*tx).* (-x.*tz+tx);
alpha4 = 2.*[(-y.*tx).*(y.*tx)+(-y.*tz).*(-y.*tz)];
alpha5 = 2.*[(tz+x.*tx).*(y.*tx) +  (-x.*tz+tx).* (-y.*tz)];
alpha6 = 2.*(y.*tx).* (-y.*tz);

b = alpha1.*(x_.^2)+alpha2.*(x_).*(y_)+alpha3.*(y_.^2)+alpha4.*(x_)+alpha5.*(y_)+alpha6;

beta1 = (-y.*tx).^2 + (y.*tz).^2;
beta2 = 2.*[(-y.*tx).* (tz+x.*tx)+ (x.*tz-tx) .*(y.*tz)];
beta3 = (tz+x.*tx).^2 + (x.*tz-tx).^2;
beta4 = 0;%2.*[(-y.*tx).*(-y.*tx)+(-y.*tz).*(y.*tz)];
beta5 = 2.*[(tz+x.*tx).*(-y.*tz) +  (x.*tz-tx).* (-y.*tx)];
beta6 = (y.*tx).^2 + (y.*tz).^2;

a = beta1.*(x_.^2)+beta2.*(x_).*(y_)+beta3.*(y_.^2)+beta4.*(x_)+beta5.*(y_)+beta6;

sinTheta = -b./a;
modifyIdx = find(sinTheta>1);
sinTheta(modifyIdx) = 1;
modifyIdx = find(sinTheta<-1);
sinTheta(modifyIdx) = -1;
angle_s1 = asind(sinTheta);
angle_result = angle_s1;
cosTheta = sqrt(ones(size(sinTheta))-(sinTheta).^2);


depth = r_cam.*[x_.*(-tz.*cosTheta+tx.*sinTheta+tz) - (-tz.*sinTheta-tx.*cosTheta+tx)] ./ (x.*cosTheta+sinTheta-x_.*(-x.*sinTheta+cosTheta));%-x.*sinTheta + cosTheta + r_cam.*(-tz.*cosTheta+tx.*sinTheta+tz);


end