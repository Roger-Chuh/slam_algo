function debugReLoc2()
close all;

K = eye(3);
K(1,1) = 2.0003201001638317e+03;
K(1,3) = 9.6889274597167969e+02;
K(2,2) = 2.0003201001638317e+03;
K(2,3) = 5.4882026672363281e+02;

zOfst = 0; -366468 - 30;0;-11110;
xOfst = 0; 3458225 - 15;0;11110;
% zOfst = 0;



xAng = 0; -40;



% pt3d(:,3) = pt3d(:,3) - zOfst;
% pt3d2(:,3) = pt3d2(:,3) - zOfst;

v = [2 3 5 7];
v = [2 3 5 6 7];
v = [2 3 5 7];
% v = [3 5 7 7];

v = [2 3 5 6 7];

  K(1,1) = K(1,1); + 180;
    K(2,2) = K(2,2); + 180;
    K(2,3) = K(2,3); - 0;  80;

% v = [2 4 5 7];
if 1
    img = imread('C:\Users\rongjiezhu\Desktop\imgL.jpg');
   
    
    img = ImgTransform(rgb2gray(img), K, rotx(xAng));
    
    pt2d = [1524, 715;
        1478, 86;
        1205, 369;
        1129, 663;
        1122, 470;
        480, 494;
        243, 421;
        243, 713];
    pt2d = Orig2Rect(pt2d, K, K, rotx(xAng),zeros(5,1));
    
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
    
    pt3d2_0 = pt3d2;
    
    pt2dline = [1521 698; 1478 88; 1219 653; 1206 366; 1130 646; 1122 473; 487 698;480 496; 245 732; 244 422];
    pt2dz = [808 688;35 916;913 684;718 855;1026 691;1383 962; 1118 681; 1757 831];
    %     if 0
    pt2dz = flipud(pt2dz);
    
    pt2dline = Orig2Rect(pt2dline, K, K, rotx(xAng),zeros(5,1));
    pt2dz = Orig2Rect(pt2dz, K, K, rotx(xAng),zeros(5,1));
    
    
    pt3dline = [ 10.7026         0   28.3942;
        10.9305   -8.9870   28.5849;
        14.6737   0   59.7308;
        14.6737   -9.2480   59.7308;
        18.4414         0   92.2625;
        18.7991   -9.2240   92.6504;
        -9.8973   0   99.0779;
        -9.8973   -9.0590   99.0779;
        -13.6432         0   67.7234;
        -14.0050   -9.3240   67.3058
        ];
    
    
    
    pt3dline(:,3) = pt3dline(:,3) - zOfst;
    %     pt3d2(:,3) = pt3d2(:,3) - zOfst;
    
    pt3dline(:,1) = pt3dline(:,1) - xOfst;
    
    v = [2 3 5 7];
    
    %     pt3d2(:,1) = pt3d2(:,1) - xOfst;
else
    img = imread('C:\Users\rongjiezhu\Desktop\imgL2.jpg');
    
    img = ImgTransform(rgb2gray(img), K, rotx(xAng));
    if 1
        pt3d = [-13.671906, -0, -40.096401;
            -13.742473, -8.9779997, -40.253265;
            -19.038452, -0, -74.942268;
            -19.354174, -9.0830002, -75.316635;
            -22.766642, -9.0769997, -103.4422;
            13.27886, -0, -81.354393;
            13.482544, -9.184, -81.117516];
    else
        pt3d = [7.204272109521389 , 0 , -94.90524628086322
            7.204272109521389 , -8.945893800000002 , -94.90524628086322 ;
            8.563598824858808 , 0 , -59.416777022822025 ;
            8.563598824858808 , -8.9014237 , -59.416777022822025 ;
            8.57628573931563 , -8.81596608 , -31.09143727765797 ;
            -24.838848377811765 ,0 , -88.20753416179893 ;
            -24.838848377811765 , -8.82759363 , -88.20753416179893 ;];
        
        pt3d(:,1) = -pt3d(:,1);
        pt3d(:,3) = -pt3d(:,3);
        
        
         pt3dline = [7.204272109521389 , 0 , -94.90524628086322
            7.204272109521389 , -8.945893800000002 , -94.90524628086322 ;
            8.563598824858808 , 0 , -59.416777022822025 ;
            8.563598824858808 , -8.9014237 , -59.416777022822025 ;
            8.57628573931563 , 0 , -31.09143727765797 ;
            8.57628573931563 , -8.81596608 , -31.09143727765797 ;
            -24.838848377811765 ,0 , -88.20753416179893 ;
            -24.838848377811765 , -8.82759363 , -88.20753416179893 ;];
        
    end
    
    
    
    pt2d = [1494, 705;
        1471, 248;
        1311, 670;
        1304, 429;
        1232, 489;
        415, 685;
        412, 458];
    
    pt2d = Orig2Rect(pt2d, K, K, rotx(xAng),zeros(5,1));
    
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
    
    
    pt3d(:,1) = -pt3d(:,1);
    pt3d(:,3) = -pt3d(:,3);
    
    pt3d2(:,1) = -pt3d2(:,1);
    pt3d2(:,3) = -pt3d2(:,3);
    if 1
        pt3d2_0 = pt3d2;
        pt3d2 = [pt3d2_0([13 8 12 6 11 4 10 2],:)];
         pt3dline = [ 13.6719         0   40.0964;
        13.7425   -8.9780   40.2533;
        19.0385         0   74.9423;
        19.3542   -9.0830   75.3166;
        22.7666   0  103.4422;
        22.7666   -9.0770  103.4422;
        -13.2789         0   81.3544;
        -13.4825   -9.1840   81.1175];
    else
        pt3d2_1 = [5.17186382629218 , -0.15710323999999964 , -117.90886816262285 ;
            1.758171283051448 , -0.1706983700000002 , -119.55993288810339 ;
            -1.6530229486130548 , -0.20544159000000128 , -121.41142704589608 ;
            -5.102147297336488 , -0.25802460999999965 , -123.17782134325657 ;
            5.143510092462098 , -0.15638722000000094 , -94.24069954568851 ;
            1.8140420982947294 , -0.16041459000000025 , -94.36627772072667 ;
            -1.6349424032647786 , -0.21602059000000118 , -94.53821964307842 ;
            -5.132459506545651 , -0.28163457000000136 , -94.70425646241424 ;];
        pt3d2 = pt3d2_1;
        pt3d2(1:2:end,:) = pt3d2_1(1:size(pt3d2_1,1)/2,:);
        pt3d2(2:2:end,:) = pt3d2_1(size(pt3d2_1,1)/2 + 1:end,:);
        pt3d2_0 = pt3d2;
        
        
        
        
    end
    
    pt2dline = [1498 703; 1471 248; 1311 670; 1305 426; 1240 655; 1231 484;415 685;412 461];
    pt2dz = [1852 796; 1350 697;1367 773;1324 754;1037 810;1032 672; 277 948;917 687];
    
    pt2dline = Orig2Rect(pt2dline, K, K, rotx(xAng),zeros(5,1));
    pt2dz = Orig2Rect(pt2dz, K, K, rotx(xAng),zeros(5,1));
    
    
    %     if 0
    %     pt2dz = flipud(pt2dz);
    
    
   
    
    
    
    v = [2 4 5 7];
    
end



% v = [2 4 5 7];

xThr = 0; -2;
pt3d(:,1) = pt3d(:,1) + xThr;
pt3dline(:,1) = pt3dline(:,1) + xThr;
pt3d2(:,1) = pt3d2(:,1) + xThr;
pt3d2_0(:,1) = pt3d2_0(:,1) + xThr;


yThr = 0;2;
pt3d(:,2) = pt3d(:,2) + yThr;
pt3dline(:,2) = pt3dline(:,2) + yThr;
pt3d2(:,2) = pt3d2(:,2) + yThr;
pt3d2_0(:,2) = pt3d2_0(:,2) + yThr;

zThr = 150; -25;
pt3d(:,3) = pt3d(:,3) + zThr;
pt3dline(:,3) = pt3dline(:,3) + zThr;
pt3d2(:,3) = pt3d2(:,3) + zThr;
pt3d2_0(:,3) = pt3d2_0(:,3) + zThr;


pt3d = (rotz(0)*roty(-0)*rotx(-0)*pt3d')';
pt3dline = (rotz(0)*roty(-0)*rotx(-0)*pt3dline')';
pt3d2 = (rotz(0)*roty(-0)*rotx(-0)*pt3d2')';

[rtTemp, outlierId] = posest(pt2d(v,:), pt3d(v,:), 0.9, K, 'repr_err');

% % [R,t,Xc,best_solution]=efficient_pnp_planar( pt3d([2 3 5 6 7],:),pt2d([2 3 5 6 7],:),K);
% rtTemp = [rodrigues(R);t];


ptIcsTemp_gt = TransformAndProject(pt3d, K, rodrigues(rtTemp(1:3)), rtTemp(4:6));
ptIcsTemp_gt2 = TransformAndProject(pt3d2, K, rodrigues(rtTemp(1:3)), rtTemp(4:6));
err = ptIcsTemp_gt(v,:) - pt2d(v,:)
figure,imshow(img);hold on;plot(pt2d(v,1), pt2d(v,2), 'or');plot(ptIcsTemp_gt(v,1), ptIcsTemp_gt(v,2), 'xg');plot(ptIcsTemp_gt2(:,1), ptIcsTemp_gt2(:,2), 'xm');

if 1
    if 0
        pt3dline2 = [7.571427801248781 , -8.68580087 , 90.86268516628888 ;
            7.547508926637793 , -8.94490766 , 122.23255487333536 ;
            7.6935138699581564 , -8.8578463 , 155.41669471391378 ;
            -21.567079933926326 , -8.606127290000002 , 158.34426866217856 ;
            -21.83000067983201 , -9.00228437 , 126.33350422237083 ;];
        
        pt3dline22 = [pt3dline2;pt3dline2];
        pt3dline22(2:2:end,:) = pt3dline2;
        pt3dline22(1:2:end,:) = pt3dline2;
        pt3dline22(1:2:end,2) = 0;
        
        pt3d = pt3dline22([1 2 4 5 6 8 10 9],:);
        vv = [2 3 5 6 7];
        [rtTemp1, outlierId] = posest(pt2d(vv,:), pt3d(vv,:), 0.9, K, 'repr_err');
        
        % % [R,t,Xc,best_solution]=efficient_pnp_planar( pt3d([2 3 5 6 7],:),pt2d([2 3 5 6 7],:),K);
        % rtTemp = [rodrigues(R);t];
        
        
        ptIcsTemp_gt = TransformAndProject(pt3d, K, rodrigues(rtTemp1(1:3)), rtTemp1(4:6));
        ptIcsTemp_gt2 = TransformAndProject(pt3d2, K, rodrigues(rtTemp1(1:3)), rtTemp1(4:6));
        err = ptIcsTemp_gt(vv,:) - pt2d(vv,:)
        figure,imshow(img);hold on;plot(pt2d(vv,1), pt2d(vv,2), 'or');plot(ptIcsTemp_gt(vv,1), ptIcsTemp_gt(vv,2), 'xg');plot(ptIcsTemp_gt2(:,1), ptIcsTemp_gt2(:,2), 'xm');
        
        
        
        
        [poseVec1] = BuildSemanticDepth(img, K, pt3dline22, pt3d2, pt2dline, pt2dz, rtTemp1, [1 0]);
        ptIcsTemp_gt = TransformAndProject(pt3dline22, K, rodrigues(poseVec1(1:3)), poseVec1(4:6));
        ptIcsTemp_gt2 = TransformAndProject(pt3d2, K, rodrigues(poseVec1(1:3)), poseVec1(4:6));
        % v = 1:8;
        err1 = ptIcsTemp_gt - pt2dline
        figure,imshow(img);hold on;plot(pt2dline(:,1), pt2dline(:,2), 'or');plot(ptIcsTemp_gt(:,1), ptIcsTemp_gt(:,2), 'xg');plot(ptIcsTemp_gt2(:,1), ptIcsTemp_gt2(:,2), 'xm');
        
    end
    
    
    
    %     [poseVec] = BuildSemanticDepth(img, K, pt3dline([3 4 5 6],:), pt3d2, pt2dline([3 4 5 6],:), pt2dz, rtTemp, [1 1]);
    
%     [poseVec1] = BuildSemanticDepth(img, K, pt3dline([5 6],:), pt3d2([1 2 7 8],:), pt2dline([5 6],:), pt2dz([1 2 7 8],:), rtTemp, [1 1]);
%     [poseVec1] = BuildSemanticDepth(img, K, pt3dline(1:6,:), pt3d2([1 2 7 8],:), pt2dline(1:6,:), pt2dz([1 2 7 8],:), poseVec1, [1 1]);
    
    [poseVec1] = BuildSemanticDepth(img, K, pt3dline([3 4 5 6 7 8],:), pt3d2([1 2 7 8],:), pt2dline([3 4  5 6 7 8],:), pt2dz([1 2 7 8],:), rtTemp, [1 1]);
    
%     [poseVec1] = BuildSemanticDepth(img, K, pt3dline(:,:), pt3d2([1 2 5 6 7 8],:), pt2dline(:,:), pt2dz([1 2 5 6 7 8],:), rtTemp, [1 1]);
    [poseVec] = BuildSemanticDepth(img, K, pt3dline, pt3d2, pt2dline, pt2dz, poseVec1, [1 1]);
    
else
    [R, t, poseVec,R_wc,t_wc ] = CvtLineData(img, K, pt2dline, pt3dline,pt2dz,pt3d2, rtTemp);
end
% poseVec(4:6) = rtTemp(4:6);
ptIcsTemp_gt = TransformAndProject(pt3dline, K, rodrigues(poseVec(1:3)), poseVec(4:6));
try
    ptIcsTemp_gt2 = TransformAndProject(pt3d2_0, K, rodrigues(poseVec(1:3)), poseVec(4:6));
catch
    ptIcsTemp_gt2 = TransformAndProject(pt3d2, K, rodrigues(poseVec(1:3)), poseVec(4:6));
end
% v = 1:8;
err1 = ptIcsTemp_gt - pt2dline
figure,imshow(img);hold on;plot(pt2dline(:,1), pt2dline(:,2), 'or');plot(ptIcsTemp_gt(:,1), ptIcsTemp_gt(:,2), 'xg');plot(ptIcsTemp_gt2(:,1), ptIcsTemp_gt2(:,2), 'xm');

asdgfkj = 1;

if 0
    for j = 1 : size(R_wc, 3)
        
        ptIcsTemp_gt = TransformAndProject(pt3d, K, R_wc(:,:,j), t_wc(:,j));
        ptIcsTemp_gt2 = TransformAndProject(pt3d2, K, R_wc(:,:,j), t_wc(:,j));
        % v = 1:8;
        err1 = ptIcsTemp_gt(v,:) - pt2d(v,:)
        figure,imshow(img);hold on;plot(pt2d(v,1), pt2d(v,2), 'or');plot(ptIcsTemp_gt(v,1), ptIcsTemp_gt(v,2), 'xg');plot(ptIcsTemp_gt2(:,1), ptIcsTemp_gt2(:,2), 'xm');
    end
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