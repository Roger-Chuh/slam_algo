function testZLimit(LocalTrace, b2c, intrMat, featId, modifyPt, pixOfst)

pnp_ang_est_max_margin = [deg2rad([-2 2]) 2];




pix = [LocalTrace.ptIcsX(featId,1) LocalTrace.ptIcsY(featId,1)];


z = LocalTrace.ptCcsZ(featId,1);
zGT = LocalTrace.ptCcsZGT(featId,1);
if modifyPt(1)
    z = modifyPt(2);
    zGT = modifyPt(3);
end

metricKey = intrMat\HomoCoord(pix',1);
metricKey = normc(metricKey);
scaleAllKey = z./metricKey(3,:)';
scaleAllKeyGT = zGT./metricKey(3,:)';
%                                   scale0 = zTrue./metricPrevPtCcs(3,:)';
keyCcsXYZ = [repmat(scaleAllKey',3,1).*metricKey];
keyCcsXYZGT = [repmat(scaleAllKeyGT',3,1).*metricKey];



thetaRng = 1:1:49;
projList = []; projListGT = [];
% pixOfst = -0.1;
for i = 1 : length(thetaRng)
    k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(deg2rad(thetaRng(i))),zeros(3,1));
    k2cBodyPmat = k2cBodyPmat.transformMat;
    k2cCam = b2c*k2cBodyPmat/b2c;
    homocurrCcsXYZ = k2cCam*HomoCoord(keyCcsXYZ,1);
    proj = pflat(intrMat*homocurrCcsXYZ(1:3,:));
    projList = [projList proj(1:2)];
    homocurrCcsXYZGT = k2cCam*HomoCoord(keyCcsXYZGT,1);
    projGT = pflat(intrMat*homocurrCcsXYZGT(1:3,:));
    projListGT = [projListGT projGT(1:2)];
    
    
    [k2c(i,1), ~, ~] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin,intrMat,pix,projGT(1:2)' + pixOfst, z, [1:size(z,1)]',true(size(z,1),1),b2c,deg2rad(thetaRng(i)));
    [k2cGT(i,1), ~, ~] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin,intrMat,pix,projGT(1:2)' + pixOfst, zGT, [1:size(z,1)]',true(size(z,1),1),b2c,deg2rad(thetaRng(i)));
    
end
projList = [pix' projList];
projListGT = [pix' projListGT];

figure(767),clf;subplot(1,2,1);imshow(ones(240,320));hold on;plot(projList(1,:), projList(2,:),'.r');plot(projListGT(1,:), projListGT(2,:),'.g');plot(projList(1,1),projList(2,1),'ob');title(sprintf('stereo Z: %dmm\ngt Z: %dmm', round(z), round(zGT)));legend('stereoZ','goldenZ');
subplot(1,2,2);plot([(rad2deg(k2c) - thetaRng')  (rad2deg(k2cGT) - thetaRng')],'-x');legend('stereo theta','gt theta');
end