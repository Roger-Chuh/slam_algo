function debugReLoc()
close all;

K = eye(3);
K(1,1) = 2.0003201001638317e+03;
K(1,3) = 9.6889274597167969e+02;
K(2,2) = 2.0003201001638317e+03;
K(2,3) = 5.4882026672363281e+02;
pt2d = [1524, 715;
    1478, 86;
    1205, 369;
    1129, 663;
    1122, 470;
    480, 494;
    243, 421;
    243, 713];
zOfst = -366468 - 30;0;-11110;
xOfst = 3458225 - 15;0;11110;
% zOfst = 0;
if 1
        img = imread('C:\Users\rongjiezhu\Desktop\imgL.jpg');
%     img = imread('C:\Users\rongjiezhu\Desktop\imgL2.jpg');
else
    img0 = imread('D:\Auto\data2\imgL_000000000210920.png');
    %     img0 = imread('D:\Auto\data2\imgL_000000000210280.png');
    %     img0 = imread('D:\Auto\data2\imgL_000000000210000.png');
    
    K = [1974.836407761425                   0   954.659873837966;
        0   1975.737477118378   523.139464719341;
        0                   0   1];
    kc = [ -0.471267846232986;
        0.157071341033424;
        0.000882826599459;
        0.000158383214543;
        0.502704133739460];
    
    img = undistortimage(img0, [K(1,1) K(2,2)]', K(1,3), K(2,3), kc(1), kc(2), kc(5), kc(3), kc(4));
    
    
    
    pt2d0 = [1476, 646;1428, 86;1183, 345;1111, 623;1104, 439;483, 470;279, 404;279, 668];
    
    % pt2d0 = [1367, 636;1337, 180;1157, 372;1097, 624;1091, 449;516, 476;341, 420;344, 658];
    %
    % pt2d0 = [1342, 649;1313, 212;1149, 383;1092, 623;1088, 458;524, 480;360, 430;362, 663];
    %
    % pt2d0 = [1302, 648;1278, 247;1135, 393;1086, 623;1082, 460;539, 486;389, 438;393, 661];
    
    ptUndistRight = normalize_pixel(pt2d0',[K(1,1);K(2,2)],[K(1,3);K(2,3)],kc,0);
    ptUndist3D = [ptUndistRight; ones(1,size(ptUndistRight,2))]';
    ptUndist = K*[ptUndistRight;ones(1,size(ptUndistRight,2))];
    ptUndist = [ptUndist(1,:)./ptUndist(3,:);ptUndist(2,:)./ptUndist(3,:)];
    pt2d = ptUndist';
    figure,imshow(img);hold on;plot(pt2d(:,1), pt2d(:,2),'or');
end
pt3d2 =[-8.45216;0;-28.6952
    -12.3214;0;-60.375
    -5.09059;0;-28.7379
    -8.9598;0;-60.4177
    -1.65283;0;-28.7815
    -5.45537;0;-60.4622
    1.84206;0;-28.8259
    -1.98904;0;-60.5062];
pt3d2 = reshape(pt3d2,3,[])';

if 1
    pt3d0 = [366468.91 3458225.43 0;
        366468.68  3458225.24 -8.987;
        
        366464.94  3458194.09 -9.248;
        
        366461.17  3458161.56 0;
        366460.81  3458161.17 -9.224;
        
        366489.51  3458154.74 -9.059;
        
        366493.61  3458186.51 -9.324;
        366493.25  3458186.10 0];
    % pt3d0(:,1:2) = pt3d0(:,1:2)-[366479.61 3458253.82];
    
    pt3d00 = pt3d0;
    pt3d00(:,3) = -pt3d0(:,1);
    pt3d00(:,1) = pt3d0(:,2);
    pt3d00(:,2) = pt3d0(:,3);
    pt3d = pt3d00;
    
    pt3d20 = [366471.16 3458224.96 0;  366467.37 3458193.83 0;
        366474.40 3458224.92 0;  366470.66 3458193.79 0;
        366477.95 3458224.87 0;  366474.16 3458193.75 0;
        366481.50 3458224.83 0;  366477.63 3458193.70 0];
    % pt3d20(:,1:2) = pt3d20(:,1:2) - [366479.61 3458253.82];
    
    
    pt3d200 = pt3d20;
    pt3d200(:,3) = -pt3d20(:,1);
    pt3d200(:,1) = pt3d20(:,2);
    pt3d200(:,2) = pt3d20(:,3);
    pt3d2 = pt3d200;
    
    pt3d(:,3) = pt3d(:,3) - zOfst;
    pt3d2(:,3) = pt3d2(:,3) - zOfst;
    
    pt3d(:,1) = pt3d(:,1) - xOfst;
    pt3d2(:,1) = pt3d2(:,1) - xOfst;
    figure,plot3(pt3d0(:,1),pt3d0(:,2),pt3d0(:,3));axis equal;hold on;plot3(pt3d0(1,1),pt3d0(1,2),pt3d0(1,3),'or');
else
    pt3d = [-13.671906, -0, -40.096401;
        -13.742473, -8.9779997, -40.253265;
        -19.038452, -0, -74.942268;
        -19.354174, -9.0830002, -75.316635;
        -22.766642, -9.0769997, -103.4422;
        13.27886, -0, -81.354393;
        13.482544, -9.184, -81.117516]
    pt2d = [1494, 705;
        1471, 248;
        1311, 670;
        1304, 429;
        1232, 489;
        415, 685;
        412, 458];
    
    pt3d2 = [-1.51946,0,-41.9347
        -3.61626,0,-59.3025
        -5.01162,0,-41.6797
        -7.10744,0,-58.9699
        -8.4563,0,-41.4364
        -10.5607,0,-58.6489
        -11.7768,0,-41.1615
        -13.8902,0,-58.3405
        -15.8214,0,-58.1608
        1.86822,0,-13.6634
        -1.76799,0,-15.003
        -5.37676,0,-16.4317
        -8.96396,0,-17.661
        -16.7644,0,-88.1705];
    pt3d(:,3) = pt3d(:,3) - zOfst;
    pt3d2(:,3) = pt3d2(:,3) - zOfst;
    
    pt3d(:,1) = pt3d(:,1) - xOfst;
    pt3d2(:,1) = pt3d2(:,1) - xOfst;
    
    
    
end
% pt3d0(:,[1 3]) = pt3d0(:,[3 1]);
% figure,plot3(pt3d0(:,1),pt3d0(:,2),pt3d0(:,3));axis equal;hold on;plot3(pt3d0(1,1),pt3d0(1,2),pt3d0(1,3),'or');
if 0
    pt3d = [-10.702579, -0, -28.394226;
        -10.930539, -8.9870005, -28.584896;
        -14.673742, -9.2480001, -59.730812;
        -18.441362, -0, -92.262527;
        -18.799133, -9.224, -92.650444;
        9.8973379, -9.059, -99.077942;
        14.005021, -9.3240004, -67.305847;
        13.643219, -0, -67.723419];
    pt3d(:,1) = -pt3d(:,1);
    pt3d(:,3) = -pt3d(:,3);
    
    pt3d2(:,1) = -pt3d2(:,1);
    pt3d2(:,3) = -pt3d2(:,3);
    
elseif 0
    pt2d = [1524, 715;
        1478, 86;
        1205, 369;
        1129, 663;
        1122, 470;
        480, 494;
        243, 421;
        243, 713];
    pt2d = pt2d();
    
    
    if 0
        pt3d = [7.34017, 0 61.0991;
            7.34017, -9 61.0991;
            7.37715, -9 91.7626
            7.56543, 0 127.722;
            7.56543, -9 127.722;
            -22.2646, -9 121.914;
            -23.1775, -9 93.9024;
            -23.1775, 0 93.9024];
    elseif 0
        pt3d = [7.14592 0 56.8779
            7.14592 -9 56.8779
            7.36643 -9 86.4441
            7.98387 0 121.604
            7.98387 -9 121.604
            -22.6235 -9 125.999
            -23.9466 -9 97.6313
            -23.9466 0 97.6313];
        
    else
        pt3d = [7.48235 0 60.0699
            7.48235 -9.0598 60.0699;
            7.70567 -8.9510 90.1318;
            8.1538 0  128.83;
            8.1538 -9.2624  128.83;
            -21.7316 -9.1469 121.835;
            -23.1484 -9.0091 95.0941;
            -23.1484 0 95.0941];
        
    end
    
    
    
    
    %     pt3d(:,1) = -pt3d(:,1);
    %     pt3d(:,3) = -pt3d(:,3);
    
    
    pt3d2 =[-8.45216;0;-28.6952
        -12.3214;0;-60.375
        -5.09059;0;-28.7379
        -8.9598;0;-60.4177
        -1.65283;0;-28.7815
        -5.45537;0;-60.4622
        1.84206;0;-28.8259
        -1.98904;0;-60.5062];
    pt3d2 = reshape(pt3d2,3,[])';
    pt3d2(:,1) = -pt3d2(:,1);
    pt3d2(:,3) = -pt3d2(:,3);
    
    
    
else
end
% pt3d(:,3) = pt3d(:,3) - zOfst;
% pt3d2(:,3) = pt3d2(:,3) - zOfst;

v = [2 3 5 7];
v = [2 3 5 6 7];
v = [2 3 5 7];
% v = [3 5 7 7];

v = [2 3 5 6 7];
% v = [2 4 5 7];
if 1
    pt2dline = [1521 698; 1478 88; 1219 653; 1206 366; 1130 646; 1122 473; 487 698;480 496; 245 732; 244 422];
    pt2dz = [808 688;35 916;913 684;718 855;1026 691;1383 962; 1118 681; 1757 831];
    
    pt3d1 = [366468.91 3458225.43 0;
        366468.68  3458225.24 -8.987;
        
        366464.94  3458194.09 0;
        366464.94  3458194.09 -9.248;
        
        366461.17  3458161.56 0;
        366460.81  3458161.17 -9.224;
        
        366489.51  3458154.74 0;
        366489.51  3458154.74 -9.059;
        
        366493.25  3458186.10 0
        366493.61  3458186.51 -9.324];
    % pt3d0(:,1:2) = pt3d0(:,1:2)-[366479.61 3458253.82];
    
    pt3dline = pt3d1;
    pt3dline(:,3) = -pt3d1(:,1);
    pt3dline(:,1) = pt3d1(:,2);
    pt3dline(:,2) = pt3d1(:,3);
    %     pt3dline = pt3d00;
    
    pt3d20 = [366471.16 3458224.96 0;  366467.37 3458193.83 0;
        366474.40 3458224.92 0;  366470.66 3458193.79 0;
        366477.95 3458224.87 0;  366474.16 3458193.75 0;
        366481.50 3458224.83 0;  366477.63 3458193.70 0];
    % pt3d20(:,1:2) = pt3d20(:,1:2) - [366479.61 3458253.82];
    
    
    pt3d200 = pt3d20;
    pt3d200(:,3) = -pt3d20(:,1);
    pt3d200(:,1) = pt3d20(:,2);
    pt3d200(:,2) = pt3d20(:,3);
    %     pt3d2 = pt3d200;
    
    
    
    pt3dline(:,3) = pt3dline(:,3) - zOfst;
    %     pt3d2(:,3) = pt3d2(:,3) - zOfst;
    
    pt3dline(:,1) = pt3dline(:,1) - xOfst;
    %     pt3d2(:,1) = pt3d2(:,1) - xOfst;
else
    pt2dline = [pt2d([1 2 3 4 6 7],:)];
    
     pt3d(:,1) = -pt3d(:,1);
    pt3d(:,3) = -pt3d(:,3);
    
    pt3d2(:,1) = -pt3d2(:,1);
    pt3d2(:,3) = -pt3d2(:,3);
    
   pt3dline = pt3d([1 2 3 4 6 7],:);
    
end


v = [2 3 5 7];
% v = [2 4 5 7];
[rtTemp, outlierId] = posest(pt2d(v,:), pt3d(v,:), 0.9, K, 'repr_err');

% % [R,t,Xc,best_solution]=efficient_pnp_planar( pt3d([2 3 5 6 7],:),pt2d([2 3 5 6 7],:),K);
% rtTemp = [rodrigues(R);t];


ptIcsTemp_gt = TransformAndProject(pt3d, K, rodrigues(rtTemp(1:3)), rtTemp(4:6));
ptIcsTemp_gt2 = TransformAndProject(pt3d2, K, rodrigues(rtTemp(1:3)), rtTemp(4:6));
err = ptIcsTemp_gt(v,:) - pt2d(v,:)
figure,imshow(img);hold on;plot(pt2d(v,1), pt2d(v,2), 'or');plot(ptIcsTemp_gt(v,1), ptIcsTemp_gt(v,2), 'xg');plot(ptIcsTemp_gt2(:,1), ptIcsTemp_gt2(:,2), 'xm');

[R, t, poseVec,R_wc,t_wc ] = CvtLineData(img, K, pt2dline, pt3dline,pt2dz,pt3d2, rtTemp);
% poseVec(4:6) = rtTemp(4:6);
ptIcsTemp_gt = TransformAndProject(pt3d, K, rodrigues(poseVec(1:3)), poseVec(4:6));
ptIcsTemp_gt2 = TransformAndProject(pt3d2, K, rodrigues(poseVec(1:3)), poseVec(4:6));
% v = 1:8;
err1 = ptIcsTemp_gt(v,:) - pt2d(v,:)
figure,imshow(img);hold on;plot(pt2d(v,1), pt2d(v,2), 'or');plot(ptIcsTemp_gt(v,1), ptIcsTemp_gt(v,2), 'xg');plot(ptIcsTemp_gt2(:,1), ptIcsTemp_gt2(:,2), 'xm');


for j = 1 : size(R_wc, 3)
    
    ptIcsTemp_gt = TransformAndProject(pt3d, K, R_wc(:,:,j), t_wc(:,j));
    ptIcsTemp_gt2 = TransformAndProject(pt3d2, K, R_wc(:,:,j), t_wc(:,j));
    % v = 1:8;
    err1 = ptIcsTemp_gt(v,:) - pt2d(v,:)
    figure,imshow(img);hold on;plot(pt2d(v,1), pt2d(v,2), 'or');plot(ptIcsTemp_gt(v,1), ptIcsTemp_gt(v,2), 'xg');plot(ptIcsTemp_gt2(:,1), ptIcsTemp_gt2(:,2), 'xm');
    
    
end

return;

v2 = [2 3 5 6 7];

v2 = [3 5 7 7];
pt3dTemp = pt3d(v2,:);
ptUndist3D = (inv(K)*pextend(pt2d(v2,:)'))';
[H,Hnorm,inv_Hnorm] = compute_homography(pt3dTemp', ptUndist3D');
mappedPt = inv(H)*pt3d(v2,:)';
mappedPt = K*mappedPt;
mappedPt1 = [mappedPt(1,:)./mappedPt(3,:); mappedPt(2,:)./mappedPt(3,:)]';
mappingErr = mappedPt1 - pt2d(v2,:);
figure,imshow(img);hold on;plot( pt2d(v2,1), pt2d(v2,2), 'or');plot(mappedPt1(:,1), mappedPt1(:,2),'.g');

figure,fig_3D3_pair([pt3d(:,:); pt3d2]',[]); hold on;

v2 = [1 2 3 4 5];
v2 = [2 3 5 6 7];
v2 = [2 3 5 7];
% v2 = [3 5 7 7];
pt3dTemp = pt3d(v2,:);
ptUndist3D = (inv(K)*pextend(pt2d(v2,:)'))';
[H2, inliers2, projErr1, sol1] = doHomo2(ptUndist3D, pt3dTemp, K, pt3d,pt3d2, pt2d, v2, img);

end

function [H2, inliers2, projErr1, sol1] = doHomo2(ptUndist3D, pt3dTemp, K, pt3d,pt3d2, pt2d, v2, img)
try
    [H2, inliers2] = ransacfithomography(ptUndist3D', pt3dTemp', 0.0001);
catch
    [H2,Hnorm,inv_Hnorm] = compute_homography(pt3dTemp', ptUndist3D');
    %     H2 = inv(H2);
    inliers2 = 1: size(pt3dTemp,1);
end
H2 = H2./H2(end);

mappedPt = inv(H2)*pt3d(v2,:)';
mappedPt = K*mappedPt;
mappedPt2 = [mappedPt(1,:)./mappedPt(3,:); mappedPt(2,:)./mappedPt(3,:)]';
mappingErr2 = mappedPt2 - pt2d(v2,:);
figure,imshow(img);hold on;plot( pt2d(v2,1), pt2d(v2,2), 'or');plot(mappedPt2(:,1), mappedPt2(:,2),'.g');

[sol1, H1] = DecomposeHomoGraphyMat((H2), eye(3),eye(3));
for e = 1 : 4
    rTmp = sol1(1:3,1:3,e);
    tTmp = sol1(1:3,4,e);
    projErr1(e,1) = ProjectErr3dTo2d(pt2d(v2,:), pt3d(v2,:), K, rTmp, tTmp);
end
[~, idH1] = min(projErr1);
RHomo1 = sol1(1:3,1:3,idH1);
tHomo1 = sol1(1:3,4,idH1);
NHomo1 = sol1(1:3,5,idH1);
T1 = [RHomo1 tHomo1; 0 0 0 1];
ptIcsTemp_gt = TransformAndProject(pt3d, K, T1(1:3,1:3), T1(1:3,4));
ptIcsTemp_gt2 = TransformAndProject(pt3d2, K,T1(1:3,1:3), T1(1:3,4));
figure,imshow(img);hold on;plot(pt2d(v2,1), pt2d(v2,2), 'or');plot(ptIcsTemp_gt(v2,1), ptIcsTemp_gt(v2,2), 'xg');plot(ptIcsTemp_gt2(:,1), ptIcsTemp_gt2(:,2), 'xm');


end