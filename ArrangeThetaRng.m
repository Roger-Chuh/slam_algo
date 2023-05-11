function [thetaRngNew, thetaSampNew, pltfrm_1, idM_Vec00] = ArrangeThetaRng(obj,thetaExp_2,k2cRef,sc,thetaSamp0)
global SOFTP2
% thetaExp_2 = rad2deg(angOpt3 - k2cRef);
if ~SOFTP2
    pltfrm_ = [thetaExp_2 - min(110.2,rad2deg(sc*obj.configParam.theta_sample_step)) thetaExp_2 + min(110.2,rad2deg(sc*obj.configParam.theta_sample_step))];
else
    pltfrm_ = [thetaExp_2 - min(0.2,rad2deg(sc*obj.configParam.theta_sample_step)) thetaExp_2 + min(0.2,rad2deg(sc*obj.configParam.theta_sample_step))];
end
pltfrm_1 = [deg2rad(pltfrm_(1)) : obj.configParam.theta_sample_step : deg2rad(pltfrm_(end))] + k2cRef;


pltfrm_1 = [pltfrm_1(1):obj.configParam.theta_sample_step2:pltfrm_1(end)];
expandNum = (length(thetaSamp0) - length(pltfrm_1))/2;


thetaRngNew = [(-obj.configParam.theta_sample_step2.*[expandNum:-1:1] + pltfrm_1(1)) pltfrm_1 (pltfrm_1(end) + obj.configParam.theta_sample_step2.*[1 : expandNum])];
thetaSampNew = thetaRngNew - k2cRef;

idM_Vec00 = find(ismember(thetaRngNew, pltfrm_1));
end