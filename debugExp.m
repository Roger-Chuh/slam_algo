
v1 = [1:41];
zeroV1 = 0;
p1 = 1/(length(v1) - (zeroV1));
p1 = p1.*ones(1,(length(v1) - (zeroV1)));
p1 = [zeros(1,zeroV1/2) p1 zeros(1,zeroV1/2)];
v2 = [];
for k = 1:length(v1)
    step = k;
    if k > 1
        v2 = [v2 v2(end) + step];
    else
        v2 = k;
    end
    
end
dltV2 = diff(v2)./2;
sampleV2 = v2(1:end-1) + [ dltV2];
len = diff(sampleV2);
p2 = p1;
p2(2:end-1) = p1(2:end-1)./len;
% p2 = p1./(v2./v1);
exp1 = dot(v1,p1);
exp22 = dot(v2,p2);


prob1 = 0; 

p2 = [];
for i = 1 : length(v1)
    prob10 = prob1;
    prob1 = prob1 + v1(i)*p1(i);
    p2 = [p2 (prob1 - prob10)/v2(i)];
    
end



 options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','off','MaxFunEvals',10000,'TolX',1e-18);%,'MaxFunEvals',10000);%, 'MaxIterations',5);
 [vec2,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) errFunc2(v2, X),[p1],[],[],options);%,data_1,obs_1)
 exp2 = dot(v2,vec2);
 sadvbi = 1;
 function err = errFunc2(v2, p2)
%  p2([1 2 20 21]) = 0;
 p2 = p2./sum(p2);
 err = abs(dot(v2,p2)) - v2((length(v2)+1)/2);
 
 
 end
