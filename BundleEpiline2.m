function BundleEpiline2(obj)


options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','off','MaxFunEvals',10000,'TolX',1e-18);%,'MaxFunEvals',10000);%, 'MaxIterations',5);
 [vec2,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) errFunc2(v2, X),[p1],[],[],options);%,data_1,obs_1)



end