function OptB2C(obj, intrMat, dispErrExpStack, newCalc)

options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter','MaxFunEvals',10000,'TolX',1e-18, 'OptimalityTolerance', 1e-17,'FunctionTolerance', 1e-16,'StepTolerance',1e-5);%,'MaxFunEvals',10000);%, 'MaxIterations',5);
b2c0 = [0;0;0; 10;45;-170.2];
b2c0 = b2c0(4:6);
b2c0 = [0;0;0];
error0 = CalcGoldenTrackingFunc(obj, intrMat, dispErrExpStack, newCalc,b2c0);
[vec,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) CalcGoldenTrackingFunc(obj, intrMat, dispErrExpStack, 0, X),[b2c0],[],[],options);
 
 error = CalcGoldenTrackingFunc(obj, intrMat, dispErrExpStack, newCalc, vec);


end