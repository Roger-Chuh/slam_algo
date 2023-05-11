function [optVec1, optVec2] = OptBilinear(pixOrigAll, pixOrig_2, xuu2,yuu2, xuuAll, rowNew, colNew, nr,nc)
init = zeros(size(pixOrig_2));
initVec = init(:);
% err01 = errFunc1(pixOrigAll, pixOrig_2, xuu2, xuuAll, rowNew, colNew, nr,nc, initVec);
[err01,lutIndMat,under,frac,idWithin,imgInd,dx,dy] = errFunc11(pixOrigAll, pixOrig_2, xuu2,yuu2 ,xuuAll, rowNew, colNew, nr,nc, initVec);
[err011] = errFunc111(pixOrigAll, pixOrig_2, lutIndMat,under,frac,idWithin,xuu2,imgInd,nr,nc,initVec);
% err02 = errFunc2(pixOrigAll, pixOrig_2, xuu2, xuuAll, rowNew, colNew, nr,nc, initVec);
options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter','MaxFunEvals',1000000000,'MaxIter',10,'TolX',1e-8,'TolFun', 1.0000e-8);
% [optVec1,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(U) errFunc1(pixOrigAll, pixOrig_2, xuu2, xuuAll, rowNew, colNew, nr,nc, U),[initVec],[],[],options);
% [optVec1,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(U) errFunc11(pixOrigAll, pixOrig_2, xuu2,yuu2, xuuAll, rowNew, colNew, nr,nc, U),[initVec],[],[],options);


[optVec1,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(U) errFunc111(pixOrigAll, pixOrig_2, lutIndMat,under,frac,idWithin,xuu2,imgInd,nr,nc, U),[initVec],[],[],options);


% [optVec2,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(U) errFunc2(pixOrigAll, pixOrig_2, xuu2, xuuAll, rowNew, colNew, nr,nc, U),[initVec],[],[],options);
optVec2 = optVec1;
% err11 = errFunc11(pixOrigAll, pixOrig_2, xuu2,yuu2, xuuAll, rowNew, colNew, nr,nc, optVec1);

% err12 = errFunc2(pixOrigAll, pixOrig_2, xuu2, xuuAll, rowNew, colNew, nr,nc, optVec2);


end

function err = errFunc1(pixOrigAll, pixOrig_2, xuu2, xuuAll, rowNew, colNew, nr,nc, initVec)
pixOrig_2_ = pixOrig_2 + reshape(initVec, size(pixOrig_2));

xOrigMat = reshape(pixOrig_2_(:,1),size(xuu2));
yOrigMat = reshape(pixOrig_2_(:,2),size(xuu2));

xRectMat = imresize(xOrigMat,size(xuuAll),'Method','bilinear');
yRectMat = imresize(yOrigMat,size(xuuAll),'Method','bilinear');

xRectMatUse = xRectMat((rowNew-nr)/2+1:end-(rowNew-nr)/2,(colNew-nc)/2+1:end-(colNew-nc)/2);
yRectMatUse = yRectMat((rowNew-nr)/2+1:end-(rowNew-nr)/2,(colNew-nc)/2+1:end-(colNew-nc)/2);

idWithin = find(pixOrigAll(:,1) >= 1 & pixOrigAll(:,1) <= nc & pixOrigAll(:,2) >= 1 & pixOrigAll(:,2) <= nr);
 
% [~,err] = NormalizeVector(pixOrigAll(idWithin,:) - [xRectMatUse(idWithin)+0 yRectMatUse(idWithin)+0]);
dx=pixOrigAll(idWithin,1)-xRectMatUse(idWithin);
dy=pixOrigAll(idWithin,2)-yRectMatUse(idWithin);
% err = sum(var(abs(dx)) + var(abs(dy)));
err = sum(sum(abs(dx)) + sum(abs(dy)));
end

function err = errFunc2(pixOrigAll, pixOrig_2, xuu2, xuuAll, rowNew, colNew, nr,nc, initVec)
pixOrig_2_ = pixOrig_2 + reshape(initVec, size(pixOrig_2));

xOrigMat = reshape(pixOrig_2_(:,1),size(xuu2));
yOrigMat = reshape(pixOrig_2_(:,2),size(xuu2));

xRectMat = imresize(xOrigMat,size(xuuAll),'Method','bilinear');
yRectMat = imresize(yOrigMat,size(xuuAll),'Method','bilinear');

xRectMatUse = xRectMat((rowNew-nr)/2+1:end-(rowNew-nr)/2,(colNew-nc)/2+1:end-(colNew-nc)/2);
yRectMatUse = yRectMat((rowNew-nr)/2+1:end-(rowNew-nr)/2,(colNew-nc)/2+1:end-(colNew-nc)/2);

idWithin = find(pixOrigAll(:,1) >= 1 & pixOrigAll(:,1) <= nc & pixOrigAll(:,2) >= 1 & pixOrigAll(:,2) <= nr);
 
% [~,err] = NormalizeVector(pixOrigAll(idWithin,:) - [xRectMatUse(idWithin)+0 yRectMatUse(idWithin)+0]);
dx=pixOrigAll(idWithin,1)-xRectMatUse(idWithin);
dy=pixOrigAll(idWithin,2)-yRectMatUse(idWithin);
err = sum(var(abs(dx)) + var(abs(dy)));
% err = sum(sum(abs(dx)) + sum(abs(dy)));
end
function [err,lutIndMatt,underr,fracc,idWithin,imgIndd,dx,dy] = errFunc11(pixOrigAll, pixOrig_2, xuu2, yuu2,xuuAll, rowNew, colNew, nr,nc, initVec)
pixOrig_2_ = pixOrig_2 + reshape(initVec, size(pixOrig_2));
% 
xOrigMat = reshape(pixOrig_2_(:,1),size(xuu2));
yOrigMat = reshape(pixOrig_2_(:,2),size(xuu2));
% 
% xRectMat = imresize(xOrigMat,size(xuuAll),'Method','bilinear');
% yRectMat = imresize(yOrigMat,size(xuuAll),'Method','bilinear');
% 
% xRectMatUse = xRectMat((rowNew-nr)/2+1:end-(rowNew-nr)/2,(colNew-nc)/2+1:end-(colNew-nc)/2);
% yRectMatUse = yRectMat((rowNew-nr)/2+1:end-(rowNew-nr)/2,(colNew-nc)/2+1:end-(colNew-nc)/2);
% 
idWithin = find(pixOrigAll(:,1) >= 1 & pixOrigAll(:,1) <= nc & pixOrigAll(:,2) >= 1 & pixOrigAll(:,2) <= nr);
[xAll,yAll] = meshgrid([1:nc],[1:nr]);
pixAll = [xAll(:) yAll(:)];
emptyLut = [];
XY = pixAll;
    cnt = 1;
    for j = 1 : size(xuu2,1) - 1
        markY = yuu2(j,1);
        markY_next = yuu2(j+1,1);
        dltY = markY_next - markY;
        for k = 1 : size(xuu2,2) - 1
            markX = xuu2(1,k);
            markX_next = xuu2(1,k+1);
            dltX = markX_next - markX;
            dltXY = dltX*dltY;
            id = find(XY(:,1) >= markX & XY(:,1) < markX_next & XY(:,2) >= markY & XY(:,2) < markY_next);
            storeXY{cnt,1} = XY(id,:);
            
            lutInd = sub2ind(size(xOrigMat), [j;j;j+1;j+1],[k;k+1;k;k+1]);
            storeXY{cnt,2} = [xOrigMat(j,k) xOrigMat(j,k+1) xOrigMat(j+1,k) xOrigMat(j+1,k+1)]./dltXY;
            storeXY{cnt,6} = [yOrigMat(j,k) yOrigMat(j,k+1) yOrigMat(j+1,k) yOrigMat(j+1,k+1)]./dltXY;
            storeXY{cnt,3} = [(markX_next-XY(id,1)).*(markY_next-XY(id,2)) (-markX+XY(id,1)).*(markY_next-XY(id,2)) (markX_next-XY(id,1)).*(-markY+XY(id,2)) (-markX+XY(id,1)).*(-markY+XY(id,2))];
            storeXY{cnt,4} = dot(repmat(storeXY{cnt,2},length(id),1)',storeXY{cnt,3}')';
            storeXY{cnt,7} = dot(repmat(storeXY{cnt,6},length(id),1)',storeXY{cnt,3}')';
            storeXY{cnt,5} = id;
            storeXY{cnt,8} = [repmat(lutInd',length(id),1)];
            storeXY{cnt,9} = repmat(1./dltXY,length(id),1);
            cnt = cnt + 1;
        end
    end
    
    lutIndMat = cell2mat(storeXY(:,8));
    under = cell2mat(storeXY(:,9));
    frac = cell2mat(storeXY(:,3));
    lutMatX = [xOrigMat(lutIndMat(:,1)) xOrigMat(lutIndMat(:,2)) xOrigMat(lutIndMat(:,3)) xOrigMat(lutIndMat(:,4))];
    lutMatY = [yOrigMat(lutIndMat(:,1)) yOrigMat(lutIndMat(:,2)) yOrigMat(lutIndMat(:,3)) yOrigMat(lutIndMat(:,4))];
    xSeed = dot((lutMatX.*repmat(under,1,4))',frac')';
    ySeed = dot((lutMatY.*repmat(under,1,4))',frac')';
    imgInd = cell2mat(storeXY(:,5));
    
    imgIndLogic = ismember(imgInd,idWithin);
    imgIndd = imgInd(imgIndLogic);
    lutIndMatt = lutIndMat(find(imgIndLogic),:);
    underr = under(find(imgIndLogic),:);
    fracc = frac(find(imgIndLogic),:);
    
    b = zeros(nr,nc);
    b(cell2mat(storeXY(:,5))) = cell2mat(storeXY(:,4));
    if 0
        cnt = 1;
        for j = 1 : size(xuu2,1) - 1
            markY = yuu2(j,1);
            markY_next = yuu2(j+1,1);
            dltY = markY_next - markY;
            for k = 1 : size(xuu2,2) - 1
                markX = xuu2(1,k);
                markX_next = xuu2(1,k+1);
                dltX = markX_next - markX;
                dltXY = dltX*dltY;
                id = find(XY(:,1) >= markX & XY(:,1) < markX_next & XY(:,2) >= markY & XY(:,2) < markY_next);
                storeXY2{cnt,1} = XY(id,:);
                storeXY2{cnt,2} = [yOrigMat(j,k) yOrigMat(j,k+1) yOrigMat(j+1,k) yOrigMat(j+1,k+1)]./dltXY;
                storeXY2{cnt,3} = [(markX_next-XY(id,1)).*(markY_next-XY(id,2)) (-markX+XY(id,1)).*(markY_next-XY(id,2)) (markX_next-XY(id,1)).*(-markY+XY(id,2)) (-markX+XY(id,1)).*(-markY+XY(id,2))];
                storeXY2{cnt,4} = dot(repmat(storeXY2{cnt,2},length(id),1)',storeXY2{cnt,3}')';
                storeXY2{cnt,5} = id;
                cnt = cnt + 1;
            end
        end
        figure,plot([cell2mat(storeXY2(:,4)) - cell2mat(storeXY(:,7))])
        figure,plot([cell2mat(storeXY2(:,5)) - cell2mat(storeXY(:,5))])
    end
    
    b1 = zeros(nr,nc);
    b1(cell2mat(storeXY(:,5))) = cell2mat(storeXY(:,7));





% [~,err] = NormalizeVector(pixOrigAll(idWithin,:) - [b(idWithin)+0 b1(idWithin)+0]);
dx=pixOrigAll(idWithin,1)-b(idWithin);
dy=pixOrigAll(idWithin,2)-b1(idWithin);
% err = sum(var(abs(dx)) + var(abs(dy)));
% err = sum(sum(abs(dx)) + sum(abs(dy)));
err = dx.^2 + dy.^2;
end
function [err] = errFunc111(pixOrigAll, pixOrig_2, lutIndMat,under,frac,idWithin,xuu2,imgInd,nr,nc,initVec)
pixOrig_2_ = pixOrig_2 + reshape(initVec, size(pixOrig_2));
%
xOrigMat = reshape(pixOrig_2_(:,1),size(xuu2));
yOrigMat = reshape(pixOrig_2_(:,2),size(xuu2));

lutMatX = [xOrigMat(lutIndMat(:,1)) xOrigMat(lutIndMat(:,2)) xOrigMat(lutIndMat(:,3)) xOrigMat(lutIndMat(:,4))];
lutMatY = [yOrigMat(lutIndMat(:,1)) yOrigMat(lutIndMat(:,2)) yOrigMat(lutIndMat(:,3)) yOrigMat(lutIndMat(:,4))];
xSeed = dot((lutMatX.*repmat(under,1,4))',frac')';
ySeed = dot((lutMatY.*repmat(under,1,4))',frac')';
b = zeros(nr,nc); b1 = b;
b(imgInd) = xSeed;
b1(imgInd) = ySeed;
dx=pixOrigAll(idWithin,1)-b(idWithin);
dy=pixOrigAll(idWithin,2)-b1(idWithin);
% err = sum(var(abs(dx)) + var(abs(dy)));


err = sum(sum(abs(dx)) + sum(abs(dy)));
% err = dx.^2 + dy.^2;
end
