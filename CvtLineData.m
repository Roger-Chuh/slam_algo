function [R, t, poseVec, R_wc, t_wc] = CvtLineData(img, intrMat, line2d, line3d,pt2dz, pt3d2,pose)


R_wc = [];
t_wc = [];

line2d_hi = line2d(2:2:end,:);
line2d_lo = line2d(1:2:end,:);


cnt = 1;
figure,imshow(img);hold on;

for i = 1 : size(line2d_hi, 1)
    cnr = [line2d_hi(i,:); line2d_lo(i,:)];
    [~,~,VV] = svd([cnr';ones(1,size(cnr,1))]');
    initLine = VV(:,end)';
    linek = [initLine(1)/initLine(3) initLine(2)/initLine(3) 1];
    [linek*[cnr';ones(1,size(cnr,1))]];
    linek =  linek./norm(linek(1:2));
    err = dot(repmat(linek,size(cnr,1),1)',[cnr ones(size(cnr,1),1)]')';
    lineCoeff(i,:) = linek;
    points2 = lineToBorderPoints(linek, size(img));
    line(points2(:, [1,3])', points2(:, [2,4])');
end

[~,~,Vv] = svd(lineCoeff);
pt = Vv(:,end);
vertiPt = [pt(1)./pt(end); pt(2)./pt(end); 1];
vertiDir = inv(intrMat)*vertiPt;
plot(vertiPt(1), vertiPt(2), '*r');

% filtLen = 3;
% line00 = lmatch_detect_lines_use(img, 3, filtLen);


line2d_far = pt2dz(2:2:end,:);
line2d_near = pt2dz(1:2:end,:);

lineCoeffZ = [];

for i = 1 : size(line2d_far, 1)
    cnr = [line2d_far(i,:); line2d_near(i,:)];
    [~,~,VV] = svd([cnr';ones(1,size(cnr,1))]');
    initLine = VV(:,end)';
    linek = [initLine(1)/initLine(3) initLine(2)/initLine(3) 1];
    [linek*[cnr';ones(1,size(cnr,1))]];
    linek =  linek./norm(linek(1:2));
    err = dot(repmat(linek,size(cnr,1),1)',[cnr ones(size(cnr,1),1)]')';
    lineCoeffZ(i,:) = linek;
    points2 = lineToBorderPoints(linek, size(img));
    line(points2(:, [1,3])', points2(:, [2,4])');
end

[~,~,Vv] = svd(lineCoeffZ);
pt = Vv(:,end);
vanPt = [pt(1)./pt(end); pt(2)./pt(end); 1];
vanDir = inv(intrMat)*vanPt;
plot(vanPt(1), vanPt(2), '*r');






line3d_hi = line3d(2:2:end,:);
line3d_lo = line3d(1:2:end,:);


for i = 1 : size(line3d_hi,1)
    dir1 = line3d_hi(i,:) - line3d_lo(i,:);
    dir1 = dir1./norm(dir1);
    verD(i,:) = dir1;
end
verDmean = mean(verD,1);
verDmean = verDmean./verDmean(3);

pt3d2_far = pt3d2(2:2:end,:);
pt3d2_near = pt3d2(1:2:end,:);

for i = 1 : size(pt3d2_far,1)
    dir1 = pt3d2_far(i,:) - pt3d2_near(i,:);
    dir1 = dir1./norm(dir1);
    vanD(i,:) = dir1;
end

vanDmean = mean(vanD,1);
vanDmean = vanDmean./vanDmean(3);

estimated_Rij = Van2R(verDmean, vanDmean, vertiDir, vanDir, rodrigues(pose(1:3)));




randId = randperm(size(pt2dz,1));
pt2dz2 = pt2dz(randId,:);
pt3d22 = pt3d2(randId,:);
line2d_far = pt2dz2(2:2:end,:);
line2d_near = pt2dz2(1:2:end,:);

lineCoeffZ2 = [];
figure,imshow(img);hold on;
for i = 1 : size(line2d_far, 1)
    cnr = [line2d_far(i,:); line2d_near(i,:)];
    [~,~,VV] = svd([cnr';ones(1,size(cnr,1))]');
    initLine = VV(:,end)';
    linek = [initLine(1)/initLine(3) initLine(2)/initLine(3) 1];
    [linek*[cnr';ones(1,size(cnr,1))]];
    linek =  linek./norm(linek(1:2));
    err = dot(repmat(linek,size(cnr,1),1)',[cnr ones(size(cnr,1),1)]')';
    lineCoeffZ2(i,:) = linek;
    points2 = lineToBorderPoints(linek, size(img));
    line(points2(:, [1,3])', points2(:, [2,4])');
end
plot(pt2dz2(:,1), pt2dz2(:,2),'-r');

if 0
    pi_ch_s = [pt2dz2 ones(size(pt2dz2,1),1)]';
    
else
%     pt = intrMat*pt3d22';
%     pi_ch_s = [pt2dz2 ones(size(pt2dz2,1),1)]';
    pi_ch_s = pflat(intrMat*pt3d22');
end



figure,imshow(img);hold on;
points2 = lineToBorderPoints(lineCoeffZ2(1,:), size(img));
line(points2(:, [1,3])', points2(:, [2,4])');
plot([reshape(pi_ch_s(1,:),2,[])], [reshape(pi_ch_s(2,:),2,[])],'r');


[uu,dd,vv] = svd([pt3d22 ones(size(pt3d22,1),1)]);
planeNorm = vv(1:3,4)./norm(vv(1:3,4));
tij_estimated = CoplanarR2t(lineCoeffZ2', estimated_Rij, pi_ch_s, planeNorm, intrMat);







index0 = nchoosek([1:size(line2d_hi,1)],2);

dif = index0(:,2) - index0(:,1);

index = index0(dif>1,:)';
index = reshape(index, [],2);
line2d_comb = [line2d_hi(index(1:end,1),:);line2d_hi(index(1:end,2),:)];
line3d_comb = [line3d_hi(index(1:end,1),:);line3d_hi(index(1:end,2),:)];
line2d = [line2d;line2d_comb];
line3d = [line3d;line3d_comb];

if 0
    line2d_s = line2d(1:2:end,:);
    line2d_e = line2d(2:2:end,:);
    
    
    err = line2d_s - line2d_e;
    
    
    id = find(err(:,1) ~=0);
    
    line2d = line2d([2*id-1; 2*id],:);
    line3d = line3d([2*id-1; 2*id],:);
end
% line2d = unique(line2d,'rows');
% line3d = unique(line3d,'rows');
if 0
    
    figure,imshow(img); hold on;
    for i = 1 : size(line2d,1)/2
        plot(line2d( 2*i-1:2*i, 1),line2d(2*i-1:2*i,2),'-');
    end
    
end



line2d_metric = pflat(inv(intrMat)*pextend([line2d]'));
line3d = [line3d];

if 1
    if 1
        [ R_wc,t_wc ] = SRPnL( line2d_metric(1:2,1:2:end), line2d_metric(1:2,2:2:end), line3d(1:2:end,:)', line3d(2:2:end,:)' );
%         [ R_wc,t_wc ] = ASPnL( line2d_metric(1:2,1:2:end), line2d_metric(1:2,2:2:end), line3d(1:2:end,:)', line3d(2:2:end,:)' );
%         [ R_wc,t_wc ] = LPnL_Bar_ENull( line2d_metric(1:2,1:2:end), line2d_metric(1:2,2:2:end), line3d(1:2:end,:)', line3d(2:2:end,:)' );
%         [ R_wc,t_wc ]=RPnL( line2d_metric(1:2,1:2:end), line2d_metric(1:2,2:2:end), line3d(1:2:end,:)', line3d(2:2:end,:)' );
        
        
        error=inf;
        for jjj=1:size(R_wc,3)
            if 0
                rr = R_wc(:,:,jjj)';
                tt = -R_wc(:,:,jjj)'*t_wc(:,jjj);
            else
                rr = R_wc(:,:,jjj);
                tt = t_wc(:,jjj);
            end
            reproj_l =  reprojErrLines(rr, tt, line3d(1:2:end,:)', line3d(2:2:end,:)', line2d_metric(1:2,1:2:end), line2d_metric(1:2,2:2:end));
            errl(jjj,:) = sqrt(sum(sum(reproj_l.*reproj_l))/2/(size(line2d_metric,2)/2));
            
            
        end
        [~,minId] = min(errl);
        R = R_wc(:,:,minId);
        t = t_wc(:,minId); 
        poseVec = [rodrigues(R); t];
        
        
        
        [Rt, poseOpt] = OptScale(estimated_Rij,tij_estimated,line2d_metric(1:2,1:2:end), line2d_metric(1:2,2:2:end),line3d(1:2:end,:)', line3d(2:2:end,:)', norm(t));
        
        R = Rt(1:3,1:3);
        t = Rt(1:3,4);
        poseVec0 = poseVec;
        poseVec = poseOpt;
    else
        [R, t, poseVec] = PNLWrapper(line2d_metric(1:2,1:2:end), line2d_metric(1:2,2:2:end), line3d(1:2:end,:)', line3d(2:2:end,:)', pose);
    end
else
    
    [R, t] = RANSAC(line2d_metric(1:2,1:2:end), line2d_metric(1:2,2:2:end), line3d(1:2:end,:)', line3d(2:2:end,:)', 4);
    %     [R1, t1] = LPnL_DLT(line2d_metric(1:2,1:2:end), line2d_metric(1:2,2:2:end), line3d(1:2:end,:)', line3d(2:2:end,:)');
    %     [UU,SS,VV] = svd(R1);
    %         R = UU*VV';
    poseVec = [rodrigues(R); t];
end



end