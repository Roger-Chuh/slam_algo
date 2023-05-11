function [XinKey, errKey, XinPrv,errPrv] = GetBoundaryDepth1(intrMat, k2pCam,ptKey1_norm,metricPrevPtCcs, ptKey1, pixPrv)
X1 = intsec2views_midpoint([eye(3) [0;0;0]],k2pCam(1:3,:),ptKey1_norm,metricPrevPtCcs);

projKey = pflat(intrMat*X1(1:3,:));
projPrv = pflat(intrMat*(k2pCam(1:3,1:3)*X1(1:3,:) + repmat(k2pCam(1:3,4),1,size(X1,2))));
[~,errKey] = NormalizeVector(ptKey1 - projKey(1:2,:)');
[~,errPrv] = NormalizeVector(pixPrv - projPrv(1:2,:)');

XinKey = X1(1:3,:)';
XinPrv = (k2pCam(1:3,1:3)*X1(1:3,:) + repmat(k2pCam(1:3,4),1,size(X1,2)))';
end