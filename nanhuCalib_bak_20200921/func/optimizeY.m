function [dirVecOpt, theta] = optimizeY(dirVec0, dirVec)


plane = 0;

if plane ~= 1
    theta0 = angularErr(dirVec, dirVec0);
else
    theta0 = dist2PlaneErr2(dirVec',1,dirVec0');
end
if 1
    Err0 = ([rad2deg(acos(dot(dirVec', repmat(dirVec0'./norm(dirVec0),1,size(dirVec,1)))/norm(1)/norm(1)))])';
else
    for i = 1 : size(dirVec,1)
        Err0(i,:) = CalcDegree(dirVec(i,:),dirVec0);
    end
    
end
err0 = sum(sqrt(Err0.^2))/size(Err0,1);      %norm(mean(Err0));


options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','off','MaxFunEvals',10000,'TolX',1e-8);%,'MaxFunEvals',10000);%, 'MaxIterations',5);
% options = optimset('JacobPattern',J,'Algorithm','trust-region-reflective');
if plane ~= 1
    [vec,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(U) angularErr(dirVec, U),[dirVec0],[],[],options);%,data_1,obs_1)
else
    
    [vec,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(U) dist2PlaneErr2(dirVec',1, U),[dirVec0'],[],[],options);
end
if plane ~= 1
    [theta] = angularErr(dirVec, vec);
else
    [theta] = dist2PlaneErr2(dirVec',1,vec');
end
dirVecOpt = vec./norm(vec);
if dirVecOpt(2) < 0
    dirVecOpt = -dirVecOpt;
end

if 1
%     Err = ([rad2deg(acos(dot(dirVec', repmat(vec'./norm(vec),1,size(dirVec,1)))/norm(1)/norm(1)))])';
    Err = ([rad2deg(acos(dot(dirVec', repmat(dirVecOpt'./norm(dirVecOpt),1,size(dirVec,1)))/norm(1)/norm(1)))])';
else
    for i = 1 : size(dirVec,1)
        Err(i,:) = CalcDegree(dirVec(i,:),vec);
    end
end
err = norm(mean(Err));


% % figure,plot(Err0);hold on;plot(Err);legend('Err0','Err')



end



function err = angularErr(dirVec, dirVec0)

if dirVec0(2) < 0
    dirVec0 = -dirVec0;
end
dirVec0 = dirVec0./norm(dirVec0);
theta = ([rad2deg(acos(dot(dirVec', repmat(dirVec0',1,size(dirVec,1)))/norm(1)/norm(1)))])';
% % % % theta = abs(dot(dirVec', repmat(dirVec0',1,size(dirVec,1))) - 1);
% for i = 1 : size(dirVec,1)
%     theta(i,:) = CalcDegree(dirVec(i,:),dirVec0);
% end

err = theta;
% % % % % % % err = norm(mean(theta));
% % err = sqrt(sum(theta.^2))/size(theta,1);

end


function errr = dist2PlaneErr2(xyz,d,g)

% % judge = xyz*g;
% %
% %
% % if ~isempty(find(judge(:)<=0))
% %     g = -g;
% % end

if g(2)<0
    g = -g;
end
g = g./norm(g);
xyz = xyz;
if size(g,1) == 1;
    g = g';
end
plane = [g;-d];
% err = [xyz ones(size(xyz,1),1)]*plane;
den = plane(1)^2+plane(2)^2+plane(3)^2;
err = ( ...
    plane(1)*xyz(1,:) + ...
    plane(2)*xyz(2,:) + ...
    plane(3)*xyz(3,:) + ...
    plane(4)...
    ).^2 / den;
if den > 1
    kvhjqe = 1;
end
err = err';
errr = (sum(err.^2)/length(err));
errr = sqrt(err);
% %     errr = norm(mean(sqrt(err)));
% %     errr = err;
end