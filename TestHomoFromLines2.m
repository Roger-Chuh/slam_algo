function TestHomoFromLines2()



load('D:\Temp\20200923\Hline.mat')

inputDir = 'D:\Auto\data5\081500_all';
dirInfo = dir(fullfile(inputDir, '*.jpg'));
[~, ind1] = sort([dirInfo(1:length(dirInfo)).datenum], 'ascend');
I = [ind1];
tempInfo = cell(length(I),1);
for fg = 1:length(I)
    tempInfo{fg,1} = dirInfo(I(fg)).name;
end
for fgg = 1:length(I)
    dirInfo(fgg).name = tempInfo{fgg};
end

imgInd = 5489; 5870;
img1 = imread(fullfile(inputDir, dirInfo(imgInd).name));
img2 = imread(fullfile(inputDir, dirInfo(imgInd + 1).name));


% line1 = lmatch_detect_lines(img1, 20);
% line2 = lmatch_detect_lines(img2, 20);
% [matchedLine1, matchedLine2, matchIndex12] = MatchHoughLines(line1, [], line2, [], img1, img2, [],'flow');


intrMat = eye(3);
intrMat(1,1) = 2.0003201001638317e+03;
intrMat(1,3) = 9.6889274597167969e+02;
intrMat(2,2) = 2.0003201001638317e+03;
intrMat(2,3) = 5.4882026672363281e+02;




ctrlPt = [189 799; 514 733; 808 676; 1126 669; 1546 754; 1836 802];

p = polyfit(ctrlPt(:,1),ctrlPt(:,2),2);
x = [1:1920];
y = polyval(p,x);

[pix1,mask1] = detectEdge(img1(:,:,1),img1(:,:,1), [0.1 0.3]);

yRef1 = polyval(p,pix1(:,1));

good1 = find(pix1(:,2) > yRef1);
pix11 = pix1(good1,:);


pix1to2 = pflat(inv(Hline)*pextend(pix11'));
pix1to2 = pix1to2(1:2,:)';

mask1to2 = Pix2Img(pix1to2, img1(:,:,1));

mask1_ = Pix2Img(pix11, img1(:,:,1));
upperY = min(pix11(:,2));
mask11 = mask1_(upperY:end,:);
if 0
    figure,imshow(img1);hold on;plot(pix11(:,1),pix11(:,2),'.r');plot(x,y,'-g')
end
[D1,IDX1] = bwdist(mask11);

[pix2,mask2] = detectEdge(img2(:,:,1),img2(:,:,1), [0.1 0.3]);
yRef2 = polyval(p,pix2(:,1));
good2 = find(pix2(:,2) > yRef2);
pix22 = pix2(good2,:);
if 0
    figure,imshow(img2);hold on;plot(pix22(:,1),pix22(:,2),'.r');plot(x,y,'-g')
end
mask2_ = Pix2Img(pix22, img2(:,:,1));
% upperY = min(pix22(:,2));
mask22 = mask2_(upperY:end,:);
[D2,IDX2] = bwdist(mask22);

[ssimval, ssimmap] = ssim(D1,D2);
cor = corr2(D1, D2);


figure,imshow(img2);hold on;plot(pix22(:,1),pix22(:,2),'.r');plot(pix1to2(:,1), pix1to2(:,2),'.g');

Hline = Hline./Hline(end);

HlineVec = Hline(:);
err0 = warppingError(img1, pix11, mask22, D2, upperY, HlineVec(1:8));

options1= optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter','MaxFunEvals',10000,'TolX',1e-20, 'OptimalityTolerance', 1e-20,'FunctionTolerance', 1e-20,'StepTolerance',1e-20,'MaxFunEvals',10000, 'MaxIterations',5000000);
[vec,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) warppingError(img1, pix11, mask22, D2, upperY, X),[HlineVec(1:8)],[],[],options1);

err1 = warppingError(img1, pix11, mask22, D2, upperY,vec);

pt1 = [498 766; 387 801;1561 751;1454 727;1005 744;1018 772];
pt2 = [382 811; 184 871;1713 788;1558 755;1010 780; 1031 823];
[tform,inlierPtsDistorted,inlierPtsOriginal] = estHomo(img1,img2,pt2,pt1,0);
tform.T = Hline';

outputView = imref2d(size(img2));
Ir = imwarp(img2,tform,'OutputView',outputView);
figure,imshowpair(Ir,img1);





[sol1, H1] = DecomposeHomoGraphyMat((Hline), intrMat,intrMat);
[sol2, H2] = DecomposeHomoGraphyMat(inv(Hline), intrMat,intrMat);

if 0
    r1 = rodrigues(deg2rad(1).*sol1(:,end,1));
    r2 = rodrigues(deg2rad(1).*sol1(:,end,2));
    r3 = rodrigues(deg2rad(1).*sol1(:,end,3));
    r4 = rodrigues(deg2rad(1).*sol1(:,end,4));
else
    r1 = CorrectRot([0;1;0], sol1(:,end,1));
    r2 = CorrectRot([0;1;0], sol1(:,end,2));
    r3 = CorrectRot([0;1;0], sol1(:,end,3));
    r4 = CorrectRot([0;1;0], sol2(:,end,2));
end
r3(:,1) = -r3(:,1);r3(:,3) = -r3(:,3);
r4(:,1) = -r4(:,1);r4(:,3) = -r4(:,3);

r44 = r4*roty(30);
rMatNew = r3*roty(30);

img = imread(fullfile(inputDir, dirInfo(5804).name));
imggg = ImgTransform(rgb2gray(img1), intrMat, r3);
imggg = ImgTransform(rgb2gray(img), intrMat, roty(15)*r1');
testBirdView2(intrMat,imggg)

imggg = ImgTransform(rgb2gray(img1), intrMat, r44');figure,imshow(imggg)
return;

line1 = [498 766 387 801; 387 801 1561 751;1561 751 1454 727;1454 727 498 766;1005 744 1018 772; 404 769 270 807];
line2 = [382 811 184 871;184 871 1713 788;1713 788 1558 755; 1558 755 382 811;1010 780 1031 823; 266 817 22 880];
pt1 = [498 766; 387 801;1561 751;1454 727;1005 744;1018 772];
pt2 = [382 811; 184 871;1713 788;1558 755;1010 780; 1031 823];
img1(1:684,:,:) = 0;
img2(1:684,:,:) = 0;pt1 = detectFastCorners((img1), 0.02);
% pt1 = feat1.Location;
pt2 = detectFastCorners((img2), 0.02);

[features1,valid_points1] = extractFeatures(rgb2gray(img1), pt1);
[features2,valid_points2] = extractFeatures(rgb2gray(img2), pt2);
indexPairs = matchFeatures(features1,features2, 'MatchThreshold', 100,'MaxRatio', 1, 'Unique', true, 'Metric', 'SSD');

matchedPoints11 = valid_points1(indexPairs(:,1),:);
matchedPoints22 = valid_points2(indexPairs(:,2),:);

matchedPoints1 = matchedPoints11(matchedPoints11(:,2) > 706 & matchedPoints22(:,2) > 706 & matchedPoints11(:,1) > 708 & matchedPoints22(:,1) > 708, :);
matchedPoints2 = matchedPoints22(matchedPoints11(:,2) > 706 & matchedPoints22(:,2) > 706 & matchedPoints11(:,1) > 708 & matchedPoints22(:,1) > 708, :);

[F,inliersIndex] = estimateFundamentalMatrix(matchedPoints1,matchedPoints2, 'Method', 'RANSAC','NumTrials', 100000, 'DistanceThreshold', 1,'Confidence', 99, 'InlierPercentage', 80);

p1 = matchedPoints1(inliersIndex,:);
p2 = matchedPoints2(inliersIndex,:);
flag = ismember(p1, [1686 748; 1649 794; 1748 817],'rows');

p11 = p1(~flag,:);
p22 = p2(~flag,:);
% pt2 = feat2.Location;
% figure,imshow(img2);hold on;plot(matchedPoints2(:,1), matchedPoints2(:,2),'.r')
% figure,imshow(img1);hold on;plot(matchedPoints1(:,1), matchedPoints1(:,2),'.r')
figure,showMatchedFeatures(img1, img2, p11, p22);


[tform,inlierPtsDistorted,inlierPtsOriginal] = estHomo(img1,img2,p22,p11,1);
Hline = tform.T';

outputView = imref2d(size(img2));
Ir = imwarp(img2,tform,'OutputView',outputView);
figure,imshowpair(Ir,img1);





return;

lineCoeff1 = changeLineFromEndpt2Coeff(line1);
lineCoeff2 = changeLineFromEndpt2Coeff(line2);
[tform,inlierPtsDistorted,inlierPtsOriginal] = estHomo(img1,img2,lineCoeff2(:,1:2),lineCoeff1(:,1:2));
Hline = tform.T;


[sol1, H1] = DecomposeHomoGraphyMat((Hline), intrMat,intrMat);

if 0
    r1 = rodrigues(deg2rad(1).*sol1(:,end,1));
    r2 = rodrigues(deg2rad(1).*sol1(:,end,2));
    r3 = rodrigues(deg2rad(1).*sol1(:,end,3));
    r4 = rodrigues(deg2rad(1).*sol1(:,end,4));
else
    r1 = CorrectRot([0;1;0], sol1(:,end,1));
    r2 = CorrectRot([0;1;0], sol1(:,end,2));
    r3 = CorrectRot([0;1;0], sol1(:,end,3));
    r4 = CorrectRot([0;1;0], sol1(:,end,4));
end
r3(:,1) = -r3(:,1);r3(:,3) = -r3(:,3);
r4(:,1) = -r4(:,1);r4(:,3) = -r4(:,3);

img = imread(fullfile(inputDir, dirInfo(5804).name));
imggg = ImgTransform(rgb2gray(img), intrMat, r4);
imggg = ImgTransform(rgb2gray(img), intrMat, roty(15)*r1');
testBirdView2(intrMat,imggg)

tform.T = tform.T';
outputView = imref2d(size(img2));
Ir = imwarp(img1,tform,'OutputView',outputView);
figure,imshowpair(Ir,img2);


[tform,inlierPtsDistorted,inlierPtsOriginal] = estHomo(img2,img1,pt1,pt2,1);

end
function rMatNew = CorrectRot(axis, gVec)
ang = CalcDegree(axis',gVec');
xVec = cross(axis,gVec);
xVec = xVec./norm(xVec);
zVec = cross(xVec, gVec);
zVec = zVec./norm(zVec);

rMatNew = [xVec'; gVec'; zVec']';
end
function img = Pix2Img(pix,dispMat)

img = zeros(size(dispMat));
pix = round(pix);
pix = pix(pix(:,1)>0&pix(:,1)<size(dispMat,2)&pix(:,2)>0&pix(:,2)<size(dispMat,1),:);
pix(pix(:,1) ==0,1) = 1;
pix(pix(:,2) ==0,2) = 1;
ind = sub2ind(size(dispMat),round(pix(:,2)),round(pix(:,1)));
img(ind) = 1;

end
function [pix,imggggg] = detectEdge(img,mask,vec)



if exist('vec') == 2
    vec = [];
end


mask = ones(size(mask));
if ndims(img) == 3;
    img = rgb2gray(img);
end
if max(img(:))~=1;
    try
        imgg = immultiply(img,uint8(mask));
    catch
        imgg = immultiply(img,(mask));
    end
else
    imgg = immultiply(img,mask);
end
bw = edge(imgg,'canny',vec);


h1 = fspecial('average',[1 1]);
imgfs = imfilter(double(bw),h1);


imgfs(imgfs(:)>0)=1;
imgfs(imgfs(:)<=0)=0;


% leftover = immultiply(img,imgfs);
indd=find(imgfs(:)~=0);
[yy,xx] = ind2sub(size(img),indd);
pix = [xx yy];

pix = pix(pix(:,1)>10&pix(:,1)<size(img,2)-10,:);
pix = pix(pix(:,2)>10&pix(:,2)<size(img,1)-10,:);
imggggg = zeros(size(img));
ing = sub2ind(size(img),pix(:,2),pix(:,1));
imggggg(ing) = 1;


% % figure,imshow(imgfs);


end
function [tform,inlierPtsDistorted,inlierPtsOriginal] = estHomo(original,distorted,ptDistor,ptOrig, draw)

%% transform 14 to 9.
% % % close all
% % % load('D:\nextvpu\algorithm\output\calibration\cbccalib\results\iEye\0012\calib.mat')
% % % original  = imread('D:\nextvpu\algorithm\output\calibration\cbccalib\stereo_pairs\iEye\0012\cbpair_left009.bmp');
% % % distorted = imread('D:\nextvpu\algorithm\output\calibration\cbccalib\stereo_pairs\iEye\0012\cbpair_left014.bmp');
% % figure,imshow(original);
% % title('Base image');
% % % figure; imshow(distorted);
% % % title('Transformed image');
if sum(sum(ptDistor))>10
    [tform,inlierPtsDistorted,inlierPtsOriginal] = estimateGeometricTransform(ptDistor,ptOrig,'projective','Confidence',99,'MaxDistance',1.2); %0.85);
    %% 20200909
    [tform,inlierPtsDistorted,inlierPtsOriginal] = estimateGeometricTransform(ptDistor,ptOrig,'projective','Confidence',99,'MaxDistance',20); %0.85);
    if draw == 1
        figure; showMatchedFeatures(original,distorted,inlierPtsOriginal,inlierPtsDistorted);
        title('Matched inlier points');
        outputView = imref2d(size(original));
        Ir = imwarp(distorted,tform,'OutputView',outputView);
        % % % % % % % % % % % figure; imshow(Ir);
        % % % % % % % % % % % title('Recovered image');
        % % % % % % % % % % % figure,imshow(original);
        % % % % % % % % % % % title('first image');
        % % % % % figure; imshow(distorted);
        % % % % % title('second image');
        figure,imshowpair(Ir,original);title('warpped and oroginal');
    end
    % [tform,inlierPtsDistorted,inlierPtsOriginal] = estimateGeometricTransform(cbcXYL{14}',cbcXYL{9}','projective','Confidence',99,'MaxDistance',1.5);
    % warpped = tform.T'*[cbcXYL{14} ;ones(1,size(cbcXYL{14},2))];
    % warp = [warpped(1,:)./warpped(3,:) ;warpped(2,:)./warpped(3,:)];
    % origin is 9, 14 is meant to be warpped into 9;
    % H'*14 = 9; H'*ptDistir = ptOrig; estimateGeometricTransform(ptDistor, ptOrig)
    % % % % % % % % % warpped = tform.T'*[ptDistor' ;ones(1,size(ptDistor,1))];
    % % % % % % % % % warp = [warpped(1,:)./warpped(3,:) ;warpped(2,:)./warpped(3,:)];
    % % % % % % % % % warp = warp';
    % % % % % % % % % m_err = warp(:,1:2) - ptOrig(:,1:2);
    % % % % % % % % % figure;
    % % % % % % % % % plot(m_err(:,1),m_err(:,2),'r+');
else
    [tform,inlierPtsDistorted,inlierPtsOriginal] = estimateGeometricTransform(ptDistor,ptOrig,'projective','Confidence',99,'MaxDistance',1);
    %% 20200909
    %     [tform,inlierPtsDistorted,inlierPtsOriginal] = estimateGeometricTransform(ptDistor,ptOrig,'projective','Confidence',99,'MaxDistance',20);
end

% % % tform.T'*[256; 185; 1]
% % % [ans(1)/ans(3) ans(2)/ans(3)]




%%  another two alternatives to calc Homography


% %  #1   [Theta, k] = estimate_homography([cbXYR{1,1};cbXYL{1,1}], [1:126]); Hgit = reshape(Theta,3,3);
% %
% %  #2   temp1 =  estimateGeometricTransform(cbXYR{1,1}',cbXYL{1,1}','projective','Confidence',99,'MaxDistance',0.001);
% %       Hmat = temp1.T';
% %
% %  #3   [Hcalib,Hnorm,inv_Hnorm] = compute_homography(cbXYL{1,1},cbXYR{1,1});  % m ~ H*M
% %
% %
% %
% %  #!!  here  'Hgit' equals 'Hmat' equal 'Hcalib';


end
function corners11 = detectFastCorners(imgL1, thresh)

if ndims(imgL1) == 3
    imgL1 = rgb2gray(imgL1);
end

corners1 = detectFASTFeatures(imgL1,'MinQuality',thresh,'MinContrast',thresh);
corners11 = double(corners1.Location);




end
function y = pextend(x)
y = [x; ones(1,size(x,2))];
end
function y = pflat(x)
y = x./repmat(x(end,:),[size(x,1) 1]);

end
function err = warppingError(img1, pix1, mask2, D2, upperY, hVec)
hMat = reshape([hVec;1],3,3);

pix1to2 = pflat(inv(hMat)*pextend(pix1'));
pix1to2 = pix1to2(1:2,:)';
mask1to2 = Pix2Img(pix1to2, img1(:,:,1));

mask1to2_ = mask1to2(upperY:end,:);
mask22 = mask2(upperY:end,:);
[D1to2,IDX1to2] = bwdist(mask1to2_);


id = find(mask1to2_>0);
mask2_bak = mask2;
mask2_bak(id) = 1;

[D2_bak,IDX2_bak] = bwdist(mask2_bak);

if 0
    figure,imshow(D2_bak,[]);
end


[ssimval, ssimmap] = ssim(D1to2,D2);

a = 1-immultiply(ssimmap, ssimmap>0);
a(a == 1) = 0;
if 0
    err = 100*(1-corr2(D1to2, D2));
elseif 0
    err = double(100*(1 - ssimval));
elseif 0
    err = (D1to2(:) - D2(:)).^2;
else
    err = double(sum(abs(a(:))));
end


if 0
    
   figure,imshowpair(mask2, mask1to2_); 
    
end


end