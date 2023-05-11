function ang = PureVision(imgL1, imgL2, imgR2, intrMat, depthGT1, depthGT2)
% [cbcX1, cbcY1, corner1] = DetectCbCorner(rgb2gray(imgL1));
% [initCbcX1, initCbcY1] = CbCoordSys(cbcX1, cbcY1);
% cbcL = cornerfinder([initCbcX1(:),initCbcY1(:)]',double(rgb2gray(imgL1)),5,5);
xyzCB = [0,0,0,0,0,0,0,0,0,70,70,70,70,70,70,70,70,70,140,140,140,140,140,140,140,140,140,210,210,210,210,210,210,210,210,210,280,280,280,280,280,280,280,280,280,350,350,350,350,350,350,350,350,350,420,420,420,420,420,420,420,420,420,490,490,490,490,490,490,490,490,490,560,560,560,560,560,560,560,560,560,630,630,630,630,630,630,630,630,630,700,700,700,700,700,700,700,700,700,770,770,770,770,770,770,770,770,770,840,840,840,840,840,840,840,840,840,910,910,910,910,910,910,910,910,910;0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560,0,70,140,210,280,350,420,490,560;0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]'./70.*60;
baseline = 20;
 c = 0;
    qMat = [1    0       0        -intrMat(1,3);...
        0    1       0        -intrMat(2,3);...
        0    0       0         intrMat(1,1);...
        0    0   (1/baseline)            0/baseline];
  


cbcL2 = detectCnr(imgL2);
cbcR2 = detectCnr(imgR2);

cbcXL_mat = reshape(cbcL2(1,:),9,14);  cbcYL_mat = reshape(cbcL2(2,:),9,14);
cbcXR_mat = reshape(cbcR2(1,:),9,14);  cbcYR_mat = reshape(cbcR2(2,:),9,14);
xl = cbcXL_mat'; yl = cbcYL_mat';
xr = cbcXR_mat'; yr = cbcYR_mat';
ptMatches_mat_calib = [xl(:) yl(:) xr(:) yr(:)];

xxx = (qMat*[ptMatches_mat_calib(:,1:2) -ptMatches_mat_calib(:,3)+ptMatches_mat_calib(:,1) ones((9)*(14),1)]')';
        ptCcs_calib = [xxx(:,1)./xxx(:,end) xxx(:,2)./xxx(:,end) xxx(:,3)./xxx(:,end)];
        xyz = permute(reshape(permute(ptCcs_calib, [1,3,2]), [14,9,3]), [2,1,3]);
         lenX = sqrt(sum(diff(xyz,1,2).^2,3));
        lenY = sqrt(sum(diff(xyz,1,1).^2,3));
        
        
        
    
% %         [xr] = normalize_pixel(cbcXYR{goodId(i)},stereoParam.focRight,stereoParam.cenRight,stereoParam.kcRight,stereoParam.alphaRight);
% %         xrr = pflat((KR)*pextend(xr));
% %         rt = [rodrigues(camParamL.rotVec(:,goodId(i))) camParamL.tranVec(:,goodId(i));0 0 0 1];
% %         cbPt = rt(1:3,1:3)*pextend(cbGridR{1}) + repmat(rt(1:3,4),1,126);
% %         ptIcs = TransformAndProject(cbPt', KR, rodrigues(stereoParam.rotVecRef), stereoParam.transVecRef);
% %         figure,imshow(zeros(nr,nc));hold on;plot(xrr(1,:),xrr(2,:),'or');plot(ptIcs(:,1),ptIcs(:,2),'.g')
% %         err(i,1) = norm(mean(abs(xrr(1:2,:)'-ptIcs)));
         
    
        
        
        cbcL1 = detectCnr(imgL1);
        validInd = sub2ind(size(depthGT1), round(cbcL1(2,:)), round(cbcL1(1,:)));
zList1 = depthGT1(validInd);
metricPrevPtCcs = intrMat\HomoCoord(cbcL1,1);
metricPrevPtCcs1 = normc(metricPrevPtCcs);
scaleAll = zList1./metricPrevPtCcs1(3,:);
keyCcsXYZAll = [repmat(scaleAll,3,1).*metricPrevPtCcs1]';       

xyz2 = permute(reshape(permute(keyCcsXYZAll, [1,3,2]), [14,9,3]), [2,1,3]);
lenX2 = sqrt(sum(diff(xyz2,1,2).^2,3));
lenY2 = sqrt(sum(diff(xyz2,1,1).^2,3));

[omckk,Tckk,Rckk,H,x,ex,JJ] = compute_extrinsic_use(cbcL2,keyCcsXYZAll',[intrMat(1,1);intrMat(2,2)],[intrMat(1,3);intrMat(2,3)],zeros(5,1),0,200,1000000);
R = rodrigues(omckk);

ang = rad2deg(norm(omckk));

xyzReproj = R*keyCcsXYZAll' + repmat(Tckk, 1, size(keyCcsXYZAll,1));
cbcReproj = pflat(intrMat * xyzReproj);

figure,subplot(1,2,1);imshow(imgL2);hold on;plot(cbcL2(1,:), cbcL2(2,:),'or');plot(cbcReproj(1,:), cbcReproj(2,:),'.g');
subplot(1,2,2);plot(cbcL2(1,:) - cbcReproj(1,:), cbcL2(2,:) - cbcReproj(2,:),'+r');axis equal;
end

function cbcL = detectCnr(imgL1)
[cbcX1, cbcY1, corner1] = DetectCbCorner(rgb2gray(imgL1));
[initCbcX1, initCbcY1] = CbCoordSys(cbcX1, cbcY1);
cbcL = cornerfinder([initCbcX1(:),initCbcY1(:)]',double(rgb2gray(imgL1)),5,5);
end