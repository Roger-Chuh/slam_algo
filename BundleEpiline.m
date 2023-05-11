function BundleEpiline(obj, angOpt, intrMat, b2c, XYZ)
idKey = find(cell2mat(obj.keyProbZ(:,1)) == max(cell2mat(obj.keyProbZ(:,1))));

visibleIdMat = nan(length(obj.keyProbZ{idKey(1),2}),size(idKey,1));
inlierIdMat = nan(length(obj.keyProbZ{idKey(1),2}),size(idKey,1));
xMat = obj.featPtManager.localTrace.ptIcsX;
yMat = obj.featPtManager.localTrace.ptIcsY;

% xMat(:,1) =
errInDeg = 0;

angList = double([obj.refAngList4; angOpt]);
if errInDeg > 0
    angList = angList+ [0:deg2rad(errInDeg):(length(angList)-1)*deg2rad(errInDeg)]';
end
angListGT = [obj.refAngList];


angList0 = [0;angList];

k2cBodyTmp = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(0),zeros(3,1));
k2cCam = b2c*k2cBodyTmp.transformMat/b2c;



w = zeros(3,1+length(angList));
T = zeros(3,1+length(angList));
w(:,1) = rodrigues(k2cCam(1:3,1:3)); % key 2 cur, global 2 local
T(:,1) = k2cCam(1:3,4);


XXX = ones(4, size(visibleIdMat, 1));
try
    XXX(1:3,:)  =XYZ;
catch
    sdvkgh = 1;
end
x = zeros(3,size(XXX,2),length(angList) + 1);
vis = zeros(size(XXX,2),length(angList) + 1);

K = repmat([intrMat(1,1); intrMat(2,2); intrMat(1,3); intrMat(2,3)], 1, length(angList) + 1);

% 6 7

% % idKey = find(cell2mat(obj.keyProbZ(:,1)) == max(cell2mat(obj.keyProbZ(:,1))));

for i = 1 : length(idKey) % size(obj.keyProbZ,1)
    
    k2cBodyTmp = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(angList(i)),zeros(3,1));
    k2cCam = b2c*k2cBodyTmp.transformMat/b2c;
    w(:,i+1) = rodrigues(k2cCam(1:3,1:3));
    T(:,i+1) = k2cCam(1:3,4);
    
    
    if i == 1
        visibleIdMat(:,i) = [1:size(visibleIdMat,1)]';% obj.keyProbZ{i,2};
        inlierIdMat(:,i) = obj.keyProbZ{idKey(i),2};
        x(:,visibleIdMat(:,1),1) = pextend([xMat(inlierIdMat(:,1), 1) yMat(inlierIdMat(:,1), 1)]');
        x(:,visibleIdMat(:,1),2) = pextend([xMat(inlierIdMat(:,1), 2) yMat(inlierIdMat(:,1), 2)]');
        vis(visibleIdMat(:,1),1) = 1;
        vis(visibleIdMat(:,1),2) = 1;
        
    else
        
        visibleIdMat(ismember(inlierIdMat(:,1), obj.keyProbZ{idKey(i),2}), i) = find(ismember(inlierIdMat(:,1), obj.keyProbZ{idKey(i),2})); % obj.keyProbZ{i,2};
        inlierIdMat(ismember(inlierIdMat(:,1), obj.keyProbZ{idKey(i),2}), i) = obj.keyProbZ{idKey(i),2};
        %         x(:,visibleIdMat(:,i),i+1) = pextend([xMat(inlierIdMat(:,i), i+1) yMat(inlierIdMat(:,i), i+1)]');
        x(:, (visibleIdMat(:,i) > 0), i+1) = pextend([xMat(obj.keyProbZ{idKey(i),2}, i+1) yMat(obj.keyProbZ{idKey(i),2}, i+1)]');
        vis((visibleIdMat(:,i) > 0), i+1) = 1;
    end
end


if 0
    [K_, Te_, w_, Xe_, error] = bundle_euclid_nomex( K, angList0', XXX, x,b2c,'visibility', vis, 'fix_calibration', 'fix_structure','verbose' );
    Te_ = Te_ - Te_(1);
    
else
    options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter','MaxFunEvals',10000,'TolX',1e-18);%,'MaxFunEvals',10000);%, 'MaxIterations',5);
    %     [vec2,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) errFunc2(v2, X),[p1],[],[],options);%,data_1,obs_1)
    
    reProjErr1 = errFunc1(visibleIdMat, inlierIdMat, b2c, intrMat, xMat, yMat, rad2deg(angList));
    [vec2,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) errFunc1(visibleIdMat, inlierIdMat, b2c, intrMat, xMat, yMat, X),[rad2deg(angList)],[],[],options);%,data_1,obs_1)
    reProjErr2 = errFunc1(visibleIdMat, inlierIdMat, b2c, intrMat, xMat, yMat, (vec2));
    figure,subplot(1,2,1);plot([reProjErr1 reProjErr2]);title('error'); legend('before', 'after'); subplot(1,2,2);plot(vec2 - rad2deg(angListGT));title('opt - gt');
    
end


end

function reProjErr1 = errFunc1(visibleIdMat, inlierIdMat, b2c, intrMat, xMat, yMat, angList)
angList0 = [0; angList];
angList0 = deg2rad(angList0);
cnt = 1;
for j = 1 : length(angList0)-1
    
    for i = j+1 : length(angList0)
        k2cTemp = angList0(i) - angList0(j);
        
        inlierId = find(xMat(:, i) > 0 & yMat(:, i) > 0);
        
        PixKey = [xMat(inlierId, j) yMat(inlierId, j)];
        
        PixCur = [xMat(inlierId, i) yMat(inlierId, i)];
        
        k2cBodyTmp = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cTemp),zeros(3,1));
        k2cCam = b2c*k2cBodyTmp.transformMat/b2c;
        [reProjErr1(cnt,1), ~] = AngleSpace.ProjectErrorUnderTransform2(1, k2cCam(1:3,1:3), k2cCam(1:3,4), PixKey, PixCur, intrMat, [], [], [], []);
        cnt = cnt + 1;
    end
    
end
reProjErr1 = double(reProjErr1);
end


function errFunc2(visibleIdMat, inlierIdMat, b2c, intrMat, xMat, yMat, angList)

for i = 1 : length(thetaK2CRng)
    k2cTemp = thetaK2CRng(i);
    
    k2cBodyTmp = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cTemp),zeros(3,1));
    k2cCam = b2c*k2cBodyTmp.transformMat/b2c;
    [reProjErr1(i,1),err1(:,i)] = AngleSpace.ProjectErrorUnderTransform2(obj.configParam.reproj_sigma, k2cCam(1:3,1:3), k2cCam(1:3,4), PixKey, PixCur, intrMat, [], [], [], []);
    
end

end