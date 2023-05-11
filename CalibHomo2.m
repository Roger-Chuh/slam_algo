function CalibHomo2()
global OUTPUT_ROOT RECORDED_BIN_ROOT DUMP_ROOT
inputDir = 'D:\Auto\data5\20200323_081500_video';
inputDir = 'D:\Auto\data5\081500_all';

imgInd = 5870; 5804;5726;


dirInfo = dir(fullfile(inputDir, '*.jpg'));


close all

if 0
    assert(~isempty(OUTPUT_ROOT) && ~isempty(RECORDED_BIN_ROOT) && ~isempty(DUMP_ROOT), 'The directories are not correctly set');
    csaParamFilePath = fullfile(DUMP_ROOT, 'coordsys_alignment.txt');
    csa = CoordSysAligner;
    % % Load(csa, csaParamFilePath);
    robotCfgFilePath = fullfile(OUTPUT_ROOT, 'robot_config.prototxt');
    robotConfig = ReadRobotConfig(robotCfgFilePath);
    
    camParamFilePath = fullfile(DUMP_ROOT, 'camera_param.txt');
    cam = BCStereoCameraModel(camParamFilePath);
    CvtToRectifiedModel(cam);
    vsl = VisualLocalizer(cam, csa, robotConfig.tracking_config);
    if isempty(vsl.scaleLvl)
        vsl.scaleLvl = 0;
    end
    try
        load(fullfile(inputDir,'angGT.mat'));
    catch
        sdjfh = 1;
    end
    
    rVec = [ 1.52973223e+00, 7.39023369e-03, -7.28091807e-04 ]';
    tVec = 10.*[ -6.53009415e-01, 1.49285034e+02, -6.13110685e+00 ]';
    
    R = [ 9.9999067662137064e-01, -4.2691725225514534e-03,6.4871897307110975e-04;...
        4.2691085282925076e-03, 9.9999088231463540e-01, 9.9999853603564991e-05;...
        -6.4913997488288771e-04, -9.7229469566657761e-05, 9.9999978458183847e-01 ];
    T = 10.*[ -1.1996676211530170e+01, -5.1620839537964008e-02, -9.2826799911339235e-03 ]';
    KL = [ 1.9748364077614247e+03, 0, 9.5465987383796642e+02;...
        0  1.9757374771183775e+03, 5.2313946471934150e+02;...
        0., 0., 1. ];
    KR = [ 1.9628183589635823e+03, 0., 9.8119298543983234e+02;...
        0. 1.9638148934135372e+03, 5.6751332830636375e+02;...
        0., 0., 1. ];
    kcL = [ -4.7126784623298640e-01, 1.5707134103342354e-01,  8.8282659945944375e-04, 1.5838321454318845e-04, 5.0270413373945999e-01 ]';
    kcR = [ -4.6676181774088582e-01, 1.6765245155018399e-01, 2.2689439615351582e-03, 9.9312512176874166e-04,   4.0665398542188858e-01 ]';
    
    [rotMat1ToCP, rotMat2ToCP, projMat1, projMat2] = OcvStereoRectify(KL, kcL, KR, kcR, [1080;1920], rodrigues(R), T);
    
    vsl.camModel.monoCamModel1.focLen = [KL(1,1); KL(2,2)];
    vsl.camModel.monoCamModel1.princpPt = [KL(1,3); KL(2,3)];
    vsl.camModel.monoCamModel1.distCoeff = kcL;
    vsl.camModel.monoCamModel2.focLen = [KR(1,1); KR(2,2)];
    vsl.camModel.monoCamModel2.princpPt = [KR(1,3); KR(2,3)];
    vsl.camModel.monoCamModel2.distCoeff = kcR;
    SetPinholeCamParam(vsl.camModel.monoCamModel2, [projMat2(1,1); projMat2(2,2)], projMat2(1:2,3), projMat2(1,2));
    SetPinholeCamParam(vsl.camModel.monoCamModel1, [projMat1(1,1); projMat1(2,2)], projMat1(1:2,3), projMat1(1,2));
    vsl.camModel.imgSize = [1080;1920];
    [mapX1, mapY1] = OcvInitUndistortRectifyMap(Get(vsl.camModel, 'IntrMat', 1), Get(vsl.camModel, 'DistCoeff', 1), rotMat1ToCP, Get(vsl.camModel, 'PinholeIntrMat', 1), vsl.camModel.imgSize);
    [mapX2, mapY2] = OcvInitUndistortRectifyMap(Get(vsl.camModel, 'IntrMat', 2), Get(vsl.camModel, 'DistCoeff', 2), rotMat2ToCP, Get(vsl.camModel, 'PinholeIntrMat', 2), vsl.camModel.imgSize);
    SetMap(vsl.camModel.monoCamModel1, mapX1, mapY1);
    SetMap(vsl.camModel.monoCamModel2, mapX2, mapY2);
    vsl.camModel.rotVec1To2 = rodrigues(R);
    vsl.camModel.transVec1To2 = T;
    
    intrMat = Get(vsl.camModel, 'PinholeIntrMat', 1,vsl.scaleLvl);
    [princpPtL, princpPtR] = Get(vsl.camModel, 'PinholePrincpPt', vsl.scaleLvl);
    
else
    
    intrMat = eye(3);
    intrMat(1,1) = 2.0003201001638317e+03;
    intrMat(1,3) = 9.6889274597167969e+02;
    intrMat(2,2) = 2.0003201001638317e+03;
    intrMat(2,3) = 5.4882026672363281e+02;
end

[~, ind1] = sort([dirInfo(1:length(dirInfo)).datenum], 'ascend');
% [~, ind2] = sort([dirInfo(length(dirInfo)/2+1:end).datenum], 'ascend');
% ind2 = ind2 + length(dirInfo)/2;
% I = [ind1 ind2];
I = [ind1];

tempInfo = cell(length(I),1);
for fg = 1:length(I)
    tempInfo{fg,1} = dirInfo(I(fg)).name;
end
for fgg = 1:length(I)
    dirInfo(fgg).name = tempInfo{fgg};
end


% imgR = imread(fullfile(inputDir, dirInfo(imgInd).name));
imgLL = imread(fullfile(inputDir, dirInfo(imgInd).name));
% try
%     imgLL = Remap(vsl.camModel.monoCamModel1, imgL);
%     imgRR = Remap(vsl.camModel.monoCamModel2, imgR);
% catch
%     
% end

pt2d = [410 819; 77 939; 1601 843; 1385 772];
pt2d = [421 887; 48 1059; 1807 932; 1501 831];
pt2d = [214 776; 15 815; 1637 748; 1520 726];
% pt2d = [214 776; 15 815; 1710 746; 1590 724];
% pt2d = [481 817; 97 975; 869 965; 1539 949; 1261 799; 903 808];

figure,imshow(imgLL);hold on;plot(pt2d(:,1), pt2d(:,2), '-or');

pt2dMetric = (inv(intrMat)*pextend(pt2d'))';
xz2 = PrepareMap();
xz2 = 1000.*xz2;
if 1
    xz2 = [xz2(:,1) xz2(:,3) xz2(:,2)];
    %     xz20 = xz2;
    xz2(:,3) = 1;
    
end
xz2 = xz2*rotx(90)';
xz2(:,3) = xz2(:,3) + 1000;

len1 = norm(xz2(1,:) - xz2(end,:));
if 0
    ratioList = 0.2144; [0.23:-0.0001:0.20]; 0.2480;
    thetaList = 1.65; [-0:0.05:3];
else
    ratioList = [0.77:-0.0002:0.73]; 0.2480;0.7502;
    thetaList = 0;
    %     ratioList = 0.8178; 0.8174+ [-0.01:0.0001:0.01];[0.83: -0.0001: 0.81];
    %     thetaList = 0.106; 0.096 + [-0.01:0.0001:0.01]; [1:-0.001:-1];
    ratioList = 0.8002;    0.7524;   0.7524+[-0.005:0.0001:0.005];  [0.77:-0.0002:0.73]; 0.2480;0.7502;
    thetaList = 7.347;   0.623;  0.624 + [-0.02:0.0005:0.02];  [1:-0.001:-1];
end
dir1 = xz2(end,:) - xz2(1,:);
dir1 = dir1./norm(dir1);
cnt = 1;
in = {};
RtTemp1 = [];
Err = [];
ang11 = CalcDegree(xz2(1,:) - xz2(2,:),xz2(3,:) - xz2(2,:));
ang2 = CalcDegree(xz2(1,:) - xz2(2,:),[0 0 1]);
ang1 = ang2; ang11 - ang2;
norm1 = norm(xz2(1,:) - xz2(2,:));

for i = 1:length(ratioList)
    xz2_ = xz2;
    xz2_(4,:) = xz2_(1,:) + len1.*ratioList(i).*dir1;
    xz2_(3,:) = xz2_(2,:) + len1.*ratioList(i).*dir1;
    len11 = norm(xz2_(1,:) - xz2_(end,:));
    len22 = norm(xz2_(2,:) - xz2_(3,:));
    len11/len1;
    xz2_0 = xz2_;
    for k = 1 : length(thetaList)
        xz2_ = xz2_0;
        theta = thetaList(k) + ang1;
        pt1_ = [xz2_(2,1) + norm1*sind(theta) xz2_(2,2) xz2_(2,3) +  norm1*cosd(theta)];
        pt4_ = [xz2_(3,1) + norm1*sind(theta) xz2_(3,2) xz2_(3,3) +  norm1*cosd(theta)];
        xz2_([1 4],:) = [pt1_; pt4_];
        try
            [rtTemp1, outlierId] = posest(pt2d, xz2_, 0.5, intrMat, 'repr_err');
            [ptIcsTemp_pnp, pt3d_pnp] = TransformAndProject(xz2_, intrMat, rodrigues(rtTemp1(1:3)), rtTemp1(4:6));
            RtTemp1 = [RtTemp1 rtTemp1];
            [~, err] = NormalizeVector(ptIcsTemp_pnp - pt2d);
            Err = [Err err];
            if 0
                figure,imshow(imgLL);hold on;plot( pt2d(:,1), pt2d(:,2), 'or');plot(ptIcsTemp_pnp(:,1), ptIcsTemp_pnp(:,2),'.g');
            end
            
            in{cnt,1} = outlierId;
            in{cnt,2} = ratioList(i);
            in{cnt,3} = ptIcsTemp_pnp;
            in{cnt,4} = i;
            in{cnt,5} = xz2_;
            in{cnt,6} = [ratioList(i) thetaList(k)];
            in{cnt,7} = rtTemp1;
            cnt = cnt+1;
            
        catch
            
        end
    end
end
figure,plot(mean(Err))
[~,minId] = min(mean(Err));
a = in(minId,:);
aa = a{end}

ptInterp = [mean(xz2([1 4],:)); mean(xz2([2 3],:))];

figure,plot(xz2(:,1), xz2(:,3));hold on;plot(ptInterp(:,1), ptInterp(:,3),'or');axis equal

for j = 1 :size(in,1)
    figure,imshow(imgLL);hold on;plot( pt2d(:,1), pt2d(:,2), 'or');
    plot(in{j,3}(:,1),in{j,3}(:,2),'.g')
    [ptIcsTemp_gt22, pt3d] = TransformAndProject(ptInterp, intrMat, rodrigues( in{j,7}(1:3)),  in{j,7}(4:6));
    plot(ptIcsTemp_gt22(:,1),ptIcsTemp_gt22(:,2),'.b')
    drawnow;
end


xz2 = in{1,5};

[rtTemp, outlierId] = posest(pt2d, xz2, 0.5, intrMat, 'repr_err');
%
% [ptIcsTemp_pnp, pt3d_pnp] = TransformAndProject(xz2, intrMat, rodrigues(rtTemp1(1:3)), rtTemp1(4:6));
% figure,imshow(imgLL);hold on;plot( pt2d(:,1), pt2d(:,2), 'or');plot(ptIcsTemp_pnp(:,1), ptIcsTemp_pnp(:,2),'.g');



[H,Hnorm,inv_Hnorm] = compute_homography(xz2', pt2dMetric');
% [H, inliers2] = ransacfithomography(pt2dMetric', xz2', 0.0001);

mappedPt = inv(H)*xz2';
mappedPt = intrMat*mappedPt;
mappedPt1 = [mappedPt(1,:)./mappedPt(3,:); mappedPt(2,:)./mappedPt(3,:)]';
mappingErr = mappedPt1 - pt2d;
figure,imshow(imgLL);hold on;plot( pt2d(:,1), pt2d(:,2), 'or');plot(mappedPt1(:,1), mappedPt1(:,2),'.g');

[sol1, H1] = DecomposeHomoGraphyMat((H), eye(3),eye(3));
for e = 1 : 4
    rTmp = sol1(1:3,1:3,e);
    tTmp = sol1(1:3,4,e);
    projErr1(e,1) = ProjectErr3dTo2d(pt2d, xz2, intrMat, rTmp, tTmp);
end
[~, idH1] = min(projErr1);
RHomo1 = sol1(1:3,1:3,idH1);
tHomo1 = sol1(1:3,4,idH1);
NHomo1 = sol1(1:3,5,idH1);
T1 = [RHomo1 tHomo1; 0 0 0 1];
[ptIcsTemp_gt, pt3d] = TransformAndProject(xz2, intrMat, T1(1:3,1:3), T1(1:3,4));


[ptIcsTemp_gt2, pt3d] = TransformAndProject(ptInterp, intrMat, T1(1:3,1:3), T1(1:3,4));
figure,imshow(imgLL);hold on;plot(pt2d(:,1), pt2d(:,2), 'or');plot(ptIcsTemp_gt(:,1), ptIcsTemp_gt(:,2), 'xg');plot(ptIcsTemp_gt2(:,1), ptIcsTemp_gt2(:,2), 'xb');


R0 = roty(2)*rodrigues(rtTemp1(1:3));
% R0(:,3) = -R0(:,3);
R = T1(1:3,1:3);
R1 = roty(2)*R;
% R2 = [R(:,1) R(:,3)  R(:,2)];

% R3 = [R0(1,:); R0(3,:); R0(2,:)];
% R4 = [R0(:,1) R0(:,3)  R0(:,2)];

img1 = ImgTransform(rgb2gray(imgLL), intrMat, R1);
img11 = ImgTransform(rgb2gray(imgLL), intrMat, R1');
testBirdView2(intrMat,img11)



img2 = ImgTransform(rgb2gray(imgLL), intrMat, R0);
img22 = ImgTransform(rgb2gray(imgLL), intrMat, R0');
testBirdView2(intrMat,img22)
% img3 = ImgTransform(rgb2gray(imgLL), intrMat, R3);
% img33 = ImgTransform(rgb2gray(imgLL), intrMat, R3');
% 
% 
% img4 = ImgTransform(rgb2gray(imgLL), intrMat, R4);
% img44 = ImgTransform(rgb2gray(imgLL), intrMat, R4');



img = imread(fullfile(inputDir, dirInfo(5451).name));
imggg0 = ImgTransform(rgb2gray(img), intrMat, R0');
imggg = ImgTransform(rgb2gray(img), intrMat, R1');
testBirdView2(intrMat,imggg0)
testBirdView2(intrMat,imggg)


figure,imshow(img1)
figure,imshow(img2)
figure,imshow(img11)
figure,imshow(img22)
% figure,imshow(img3)
% figure,imshow(img33)
% figure,imshow(img4)
% figure,imshow(img44)

end
function xz2 = PrepareMap()


corner1 = [121.597471509 31.248058897];
corner2 = [121.597474425 31.248100698];
corner3 = [121.59732666 31.24808036];
corner4 = [121.59732180 31.24803710];
Corner = [corner1; corner2; corner3; corner4];


Corner = [121.59741231 31.24766556; 121.597415332 31.247710700; 121.597306184 31.247694470; 121.597302160 31.247650835];

% Corner = [121.597461197 31.248402834; 121.5974669439 31.2484506631; 121.597433206 31.248453283;121.5974022765 31.2484554461;...
%     121.5973963188 31.2484086950; 121.59742736817 31.24840532543];

% Corner = [121.59745338 31.24840423; 121.597459209 31.248451379; 121.597424334 31.248454173; 121.5973948058 31.2484561176;...
%     121.597388973 31.248409334; 121.597417529 31.248406297];


for i = 1 : size(Corner)
    xz(i,:) = GPS2UTM(Corner(i,:));
    
end


yAng = -173.1245  -90 + 180-1.7;
yAng2 =  -9; 0; -10;-20;
xz = xz*roty(yAng)';
xz = xz - xz(1,:);
figure,plot(xz(:,1), xz(:,3),'-x');axis equal;grid on;

len1 = norm(xz(2,:));
dir1 = xz(2,:)./norm(xz(2,:));
if 1
    pt3 = xz(3,:) - len1.*dir1;
    xz0 = xz;
    xz(4,:) = pt3;
end
figure,plot(xz(:,1), xz(:,3),'-x');axis equal;grid on;
err1 = norm(xz(2,:) - xz(3,:)) - norm(xz(4,:));
err2 = norm(xz(2,:) - xz(1,:)) - norm(xz(4,:) - xz(3,:));

ang1 = CalcDegree(xz(2,:) - xz(1,:), xz(2,:) - xz(3,:));
ang11 = CalcDegree(xz(4,:) - xz(3,:), xz(4,:) - xz(1,:));

ang2 = CalcDegree(xz(4,:) - xz(3,:), xz(2,:) - xz(3,:));
ang22 = CalcDegree(xz(4,:) - xz(1,:), xz(2,:) - xz(1,:));


xz2 = xz*roty(yAng2)';
% xz2([1 5 6],1) = xz2([1 5 6],1) + (-0.4);
% xz2([3 6],3) = xz2([3 6],3) - abs(xz2(3,3) - xz2(2,3));
% xz2([4 5],3) = xz2([4 5],3) - abs(xz2(4,3) - xz2(2,3));
figure,plot(xz2(:,1), xz2(:,3),'-x');axis equal;grid on;

end

function xz = GPS2UTM(input)
% lat = 40.0691643333333;
% lon = 116.242161333333;

lat = input(1);
lon = input(2);

kD2R = pi/180;
ZoneNumber = floor((lon - 1.5) / 3.0) + 1;
L0 = ZoneNumber * 3.0;

a = 6378137.0;
F = 298.257223563;
f = 1 / F;
b = a * (1 - f);
ee = (a * a - b * b) / (a * a);
e2 = (a * a - b * b) / (b * b);
n = (a - b) / (a + b); n2 = (n * n); n3 = (n2 * n); n4 = (n2 * n2); n5 = (n4 * n);
al = (a + b) * (1 + n2 / 4 + n4 / 64) / 2.0;
bt = -3 * n / 2 + 9 * n3 / 16 - 3 * n5 / 32.0;
gm = 15 * n2 / 16 - 15 * n4 / 32;
dt = -35 * n3 / 48 + 105 * n5 / 256;
ep = 315 * n4 / 512;
B = lat * kD2R;
L = lon * kD2R;
L0 = L0 * kD2R;
l = L - L0;cl = (cos(B) * l);cl2 = (cl * cl); cl3 = (cl2 * cl); cl4 = (cl2 * cl2); cl5 = (cl4 * cl); cl6 = (cl5 * cl); cl7 = (cl6 * cl); cl8 = (cl4 * cl4);
lB = al * (B + bt * sin(2 * B) + gm * sin(4 * B) + dt * sin(6 * B) + ep * sin(8 * B));
t = tan(B); t2 = (t * t); t4 = (t2 * t2); t6 = (t4 * t2);
Nn = a / sqrt(1 - ee * sin(B) * sin(B));
yt = e2 * cos(B) * cos(B);
N = lB;
N = N + t * Nn * cl2 / 2;
N = N + t * Nn * cl4 * (5 - t2 + 9 * yt + 4 * yt * yt) / 24;
N = N + t * Nn * cl6 * (61 - 58 * t2 + t4 + 270 * yt - 330 * t2 * yt) / 720;
N = N + t * Nn * cl8 * (1385 - 3111 * t2 + 543 * t4 - t6) / 40320;
E = Nn * cl;
E = E + Nn * cl3 * (1 - t2 + yt) / 6;
E = E + Nn * cl5 * (5 - 18 * t2 + t4 + 14 * yt - 58 * t2 * yt) / 120;
E = E + Nn * cl7 * (61 - 479 * t2 + 179 * t4 - t6) / 5040;
E = E + 500000;
N = 0.9996 * N;
E = 0.9996 * (E - 500000.0) + 500000.0;
xz = [E 0 N];
end
function theta = CalcDegree(vec1, vec2)
if 0
    theta = min([rad2deg(acos(dot(vec1, vec2)/norm(vec1)/norm(vec2))); rad2deg(acos(dot(vec1, -vec2)/norm(vec1)/norm(vec2)))]);
else
    
    [vec1_norm, ~] = NormalizeVector(vec1);
    [vec2_norm, ~] = NormalizeVector(vec2);
    theta = min([rad2deg(acos(dot(vec1_norm', vec2_norm')')) rad2deg(acos(dot(vec1_norm', -vec2_norm')'))]')';
    
end
end
function y = pextend(x)
y = [x; ones(1,size(x,2))];
end