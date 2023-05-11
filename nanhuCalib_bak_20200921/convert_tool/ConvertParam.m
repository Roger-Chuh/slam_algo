function ConvertParam()

% load('D:\facTest2\#01\calib.mat')
load('calib.mat');
inputDir0 = pwd;

transVec = stereoParam.transVecRef;
imgSize = [720 1280];
imgSizeHalf = [720 640];


sampledX1 = [-15.5,16.5,48.5,80.5,96.5,112.5,144.5,176.5,208.5,240.5,272.5,304.5,336.5,368.5,400.5,432.5,464.5,496.5,528.5,544.5,560.5,592.5,624.5,656.5];
sampledY1 = [-7.5,8.5,24.5,40.5,56.5,72.5,88.5,120.5,152.5,168.5,184.5,200.5,216.5,232.5,248.5,280.5,312.5,344.5,376.5,408.5,440.5,472.5,488.5,504.5,520.5,536.5,552.5,568.5,600.5,632.5,648.5,664.5,680.5,696.5,712.5,728.5];
sampledX2 = [-15.5,16.5,48.5,80.5,112.5,144.5,176.5,208.5,240.5,272.5,304.5,336.5,368.5,384.5,400.5,432.5,464.5,496.5,512.5,528.5,560.5,592.5,624.5,656.5,688.5,720.5,752.5,768.5,784.5,816.5,848.5,880.5,896.5,912.5,944.5,976.5,1008.5,1040.5,1072.5,1104.5,1136.5,1168.5,1200.5,1232.5,1264.5,1296.5];
sampledY2 = [-7.5,24.5,40.5,56.5,88.5,104.5,120.5,136.5,152.5,168.5,184.5,200.5,216.5,248.5,280.5,312.5,328.5,344.5,376.5,392.5,408.5,440.5,472.5,504.5,520.5,536.5,552.5,568.5,584.5,600.5,616.5,632.5,664.5,680.5,696.5,728.5];

% [~, ~, rotMatL, rotMatR,  intrMatNewL, intrMatNewR] = GetRectifyParam2(stereoParam, imgSize);
intrMatOldL = [stereoParam.focLeft(1) 0 stereoParam.cenLeft(1); 0 stereoParam.focLeft(2) stereoParam.cenLeft(2); 0 0 1];
intrMatOldR = [stereoParam.focRight(1) 0 stereoParam.cenRight(1); 0 stereoParam.focRight(2) stereoParam.cenRight(2); 0 0 1];
kcL = stereoParam.kcLeft;
kcR = stereoParam.kcRight;

[LutVecOrig2RectX, LutVecOrig2RectY, LutVecOrig2RectXNum, LutVecOrig2RectYNum,Orig2RectNameMatX,Orig2RectNameMatY] = genLutData(imgSizeHalf,intrMatOldL,kcL,intrMatNewL,rotMatL, 1, 'L', sampledX1, sampledY1);
[LutVecRect2OrigX, LutVecRect2OrigY, LutVecRect2OrigXNum, LutVecRect2OrigYNum,Rect2OrigNameMatX,Rect2OrigNameMatY] = genLutData(imgSize,intrMatOldL,kcL,intrMatNewL,rotMatL, 0, 'L', sampledX2, sampledY2);
makeLut('R',inputDir0, LutVecOrig2RectXNum,LutVecRect2OrigXNum,LutVecRect2OrigX,LutVecOrig2RectX,Orig2RectNameMatX,Rect2OrigNameMatX,LutVecOrig2RectY,LutVecRect2OrigY,LutVecOrig2RectYNum);


[LutVecOrig2RectXR, LutVecOrig2RectYR, LutVecOrig2RectXNumR, LutVecOrig2RectYNumR,Orig2RectNameMatXR,Orig2RectNameMatYR] = genLutData(imgSizeHalf,intrMatOldR,kcR,intrMatNewR,rotMatR, 1, 'R', sampledX1, sampledY1);
[LutVecRect2OrigXR, LutVecRect2OrigYR, LutVecRect2OrigXNumR, LutVecRect2OrigYNumR,Rect2OrigNameMatXR,Rect2OrigNameMatYR] = genLutData(imgSize,intrMatOldR,kcR,intrMatNewR,rotMatR, 0, 'R', sampledX2, sampledY2);
makeLut('L',inputDir0, LutVecOrig2RectXNumR,LutVecRect2OrigXNumR,LutVecRect2OrigXR,LutVecOrig2RectXR,Orig2RectNameMatXR,Rect2OrigNameMatXR,LutVecOrig2RectYR,LutVecRect2OrigYR,LutVecOrig2RectYNumR);



fid1 = fopen(fullfile(inputDir0,'cam_param.txt'),'w');%????

r_stereo = eye(3);
r_stereo_t = r_stereo';
r_stereo_t_vec = r_stereo_t(:);

fprintf(fid1,'%d %d %d',imgSize(2), imgSize(1), 0);
fprintf(fid1, '\n');
fprintf(fid1,'pinhole');
fprintf(fid1, '\n');
fprintf(fid1, '%d\n',2);
fprintf(fid1,'%0.6f %0.6f %0.6f %0.6f\n',intrMatNewL(1,1), intrMatNewL(2,2), intrMatNewL(1,3),intrMatNewL(2,3));
fprintf(fid1,'%0.6f %0.6f %0.6f %0.6f\n',intrMatNewR(1,1), intrMatNewR(2,2), intrMatNewR(1,3),intrMatNewR(2,3));
fprintf(fid1,'%06f %06f %06f %06f %06f %06f %06f %06f %06f %06f %06f %06f\n',r_stereo_t_vec,-norm(transVec),0, 0);

fclose(fid1);

end
function [LutVecCharX, LutVecCharY, lutVecX, lutVecY, nameMatX, nameMatY] = genLutData(imgSize,intrMatOld,kc,intrMatNew,rotMat, reverseMapping, whichCam, sampledX, sampledY)
nc = imgSize(2);
nr = imgSize(1);
lutSize = [length(sampledY) length(sampledX)];




[xMatSampled, yMatSampled] = meshgrid(sampledX, sampledY);
pixSampled = [xMatSampled(:) yMatSampled(:)];

if reverseMapping == 0
    pixOrigSampled = remapRect(pixSampled, intrMatOld, intrMatNew ,rotMat, kc);
else
    pixOrigSampled = Orig2Rect(pixSampled, intrMatOld, intrMatNew, rotMat, kc);
end

xOrigMat = reshape(pixOrigSampled(:,1), lutSize);
yOrigMat = reshape(pixOrigSampled(:,2), lutSize);


if reverseMapping == 0
    deltaLutX2 = xMatSampled - xOrigMat; %MakeOfst2(inValidLutX,deltaLutX,ones(size(validMat)));
    deltaLutY2 = yMatSampled - yOrigMat; %MakeOfst2(inValidLutY,deltaLutY,ones(size(validMat)));
    thr = 2^7;
else
    pixOrigSampled_floor = floor(pixOrigSampled);
    pixOrigSampled_ceil = ceil(pixOrigSampled);
    ind1 = find(pixOrigSampled_floor(:,1) >= 1 & pixOrigSampled_floor(:,1) <= nc & pixOrigSampled_floor(:,2) >= 1 & pixOrigSampled_floor(:,2) <= nr);
    ind2 = find(pixOrigSampled_ceil(:,1) >= 1 & pixOrigSampled_ceil(:,1) <= nc & pixOrigSampled_ceil(:,2) >= 1 & pixOrigSampled_ceil(:,2) <= nr);
    
    validMat = zeros(lutSize);
    validMat([ind1; ind2]) = 1;
    validMat([1 end],:) = 0;
    validMat(:,[1 end]) = 0;
    se = strel('square',[3]);
    validMat = imdilate(validMat,se);
    deltaLutX = xMatSampled - xOrigMat;
    deltaLutY = yMatSampled - yOrigMat;
    inValidLutX = deltaLutX .* (~validMat);
    inValidLutY = deltaLutY .* (~validMat);
    deltaLutX2 = MakeOfst2(inValidLutX,deltaLutX,validMat);
    deltaLutY2 = MakeOfst2(inValidLutY,deltaLutY,validMat);
    thr = 2^8;
end


deltaDeltaLutX2 = cumsum([deltaLutX2(1,:);diff(deltaLutX2)]);
deltaDeltaLutX2(deltaDeltaLutX2 > thr) = (thr-1);
deltaDeltaLutX2(deltaDeltaLutX2 < -thr) = (-thr+1);
xOrigMatRecovered = xMatSampled - deltaDeltaLutX2;


deltaDeltaLutY2 = cumsum([deltaLutY2(1,:);diff(deltaLutY2)]);
deltaDeltaLutY2(deltaDeltaLutY2 > thr) = (thr-1);
deltaDeltaLutY2(deltaDeltaLutY2 < -thr) = (-thr+1);
yOrigMatRecovered = yMatSampled - deltaDeltaLutY2;

[LutVecCharX, LutVecCharY,lutVecX,lutVecY,nameMatX, nameMatY] = ...
    BiLinearInterp(reverseMapping, xMatSampled, yMatSampled, xOrigMatRecovered, yOrigMatRecovered,imgSize);

end

function pixDist = remapRect(pixRect, KDistort, KRect, R, distCoeff)

alpha = 0;
pixRectHomo = [pixRect'; ones(1,size(pixRect,1))];
rays = inv(KRect)*pixRectHomo;


% Rotation: (or affine transformation):

rays2 = R'*rays;

x = [rays2(1,:)./rays2(3,:);rays2(2,:)./rays2(3,:)];


% Add distortion:
xd = apply_distortion(x,distCoeff);


% Reconvert in pixels:

px2_ = KDistort(1,1)*(xd(1,:) + alpha*xd(2,:)) + KDistort(1,3);
py2_ = KDistort(2,2)*xd(2,:) + KDistort(2,3);
pixDist = [px2_;py2_]';

end
function [xd,dxddk] = apply_distortion(x,k)


% Complete the distortion vector if you are using the simple distortion model:
length_k = length(k);
if length_k <5 ,
    k = [k ; zeros(5-length_k,1)];
end;


[m,n] = size(x);

% Add distortion:

r2 = x(1,:).^2 + x(2,:).^2;

r4 = r2.^2;

r6 = r2.^3;


% Radial distortion:

cdist = 1 + k(1) * r2 + k(2) * r4 + k(5) * r6;

if nargout > 1,
    dcdistdk = [ r2' r4' zeros(n,2) r6'];
end;


xd1 = x .* (ones(2,1)*cdist);



if nargout > 1,
    dxd1dk = zeros(2*n,5);
    dxd1dk(1:2:end,:) = (x(1,:)'*ones(1,5)) .* dcdistdk;
    dxd1dk(2:2:end,:) = (x(2,:)'*ones(1,5)) .* dcdistdk;
end;


% tangential distortion:

a1 = 2.*x(1,:).*x(2,:);
a2 = r2 + 2*x(1,:).^2;
a3 = r2 + 2*x(2,:).^2;

delta_x = [k(3)*a1 + k(4)*a2 ;
    k(3) * a3 + k(4)*a1];



if nargout > 1,
    ddelta_xdk = zeros(2*n,5);
    ddelta_xdk(1:2:end,3) = a1';
    ddelta_xdk(1:2:end,4) = a2';
    ddelta_xdk(2:2:end,3) = a3';
    ddelta_xdk(2:2:end,4) = a1';
end;

xd = xd1 + delta_x;

if nargout > 1,
    dxddk = dxd1dk + ddelta_xdk ;
    if length_k < 5,
        dxddk = dxddk(:,1:length_k);
    end;
end;


end

function pixRect = Orig2Rect(pix, intrMatOld, intrMatNew, R, kc)

[pixUndist] = normalize_pixel(pix',[intrMatOld(1,1);intrMatOld(2,2)],[intrMatOld(1,3);intrMatOld(2,3)],kc,0);
pixUndistHomo = [pixUndist; ones(1, size(pixUndist,2))];
pixUndistR = R*pixUndistHomo;
pixRect = intrMatNew*pixUndistR;
pixRect = [pixRect(1,:)./pixRect(3,:); pixRect(2,:)./pixRect(3,:)];
pixRect = pixRect(1:2,:)';

end
function [xn] = normalize_pixel(x_kk,fc,cc,kc,alpha_c)

%normalize
%
%[xn] = normalize_pixel(x_kk,fc,cc,kc,alpha_c)
%
%Computes the normalized coordinates xn given the pixel coordinates x_kk
%and the intrinsic camera parameters fc, cc and kc.
%
%INPUT: x_kk: Feature locations on the images
%       fc: Camera focal length
%       cc: Principal point coordinates
%       kc: Distortion coefficients
%       alpha_c: Skew coefficient
%
%OUTPUT: xn: Normalized feature locations on the image plane (a 2XN matrix)
%
%Important functions called within that program:
%
%comp_distortion_oulu: undistort pixel coordinates.

if nargin < 5,
    alpha_c = 0;
    if nargin < 4;
        kc = [0;0;0;0;0];
        if nargin < 3;
            cc = [0;0];
            if nargin < 2,
                fc = [1;1];
            end;
        end;
    end;
end;


% First: Subtract principal point, and divide by the focal length:
x_distort = [(x_kk(1,:) - cc(1))/fc(1);(x_kk(2,:) - cc(2))/fc(2)];

% Second: undo skew
x_distort(1,:) = x_distort(1,:) - alpha_c * x_distort(2,:);

if norm(kc) ~= 0,
    % Third: Compensate for lens distortion:
    xn = comp_distortion_oulu(x_distort,kc);
else
    xn = x_distort;
end;

end

function [x] = comp_distortion_oulu(xd,k);

%comp_distortion_oulu.m
%
%[x] = comp_distortion_oulu(xd,k)
%
%Compensates for radial and tangential distortion. Model From Oulu university.
%For more informatino about the distortion model, check the forward projection mapping function:
%project_points.m
%
%INPUT: xd: distorted (normalized) point coordinates in the image plane (2xN matrix)
%       k: Distortion coefficients (radial and tangential) (4x1 vector)
%
%OUTPUT: x: undistorted (normalized) point coordinates in the image plane (2xN matrix)
%
%Method: Iterative method for compensation.
%
%NOTE: This compensation has to be done after the subtraction
%      of the principal point, and division by the focal length.


if length(k) == 1,
    
    [x] = comp_distortion(xd,k);
    
else
    
    k1 = k(1);
    k2 = k(2);
    k3 = k(5);
    p1 = k(3);
    p2 = k(4);
    
    x = xd; 				% initial guess
    
    for kk=1:20,
        
        r_2 = sum(x.^2);
        k_radial =  1 + k1 * r_2 + k2 * r_2.^2 + k3 * r_2.^3;
        delta_x = [2*p1*x(1,:).*x(2,:) + p2*(r_2 + 2*x(1,:).^2);
            p1 * (r_2 + 2*x(2,:).^2)+2*p2*x(1,:).*x(2,:)];
        x = (xd - delta_x)./(ones(2,1)*k_radial);
        
    end;
    
end;

end

function deltaLutNew = MakeOfst2(inValidY,deltaLut,validMat)

imValidYUp = inValidY(1:round(size(inValidY,1)/2),:);
imValidYDown = inValidY(round(size(inValidY,1)/2)+1:end,:);

for ii = 1 : size(imValidYUp,2)
    id = find(imValidYUp(:,ii) ~= 0);
    if ~isempty(id)
        imValidYUp(id,ii) = imValidYUp(id(end),ii);
    end
end
for ii = 1 : size(imValidYDown,2)
    id = find(imValidYDown(:,ii) ~= 0);
    if ~isempty(id)
        imValidYDown(id,ii) = imValidYDown(id(1),ii);
    end
end
inValidY = [imValidYUp; imValidYDown];
deltaLut(~validMat) = inValidY(inValidY~=0);
deltaLutNew = deltaLut;

end
function [LutVecCharX, LutVecCharY,lutVecX,lutVecY,nameMatX, nameMatY] = BiLinearInterp(reverseMapping,xMatSampled, yMatSampled, xOrigMatRecovered, yOrigMatRecovered,imgSize)


optFrac = [5 12 12 9 15 15];
optFracCell = {[1:2];[3:4];[7:8];[9 10 13 14 15 16];[17 18 19 20];[21];[22]};




intLen = [0 0 0 0 13 12 7 6 7 6 7 6 0 0 0 0 0 0 0 0 13 13] +1;


[LutVecCharX,lutVecX,nameMatX] = bilinear2x2(1,reverseMapping, xMatSampled, yMatSampled, xOrigMatRecovered,imgSize,intLen,optFracCell,optFrac);
[LutVecCharY,lutVecY,nameMatY] = bilinear2x2(2,reverseMapping, xMatSampled, yMatSampled, yOrigMatRecovered,imgSize,intLen,optFracCell,optFrac);


end
function [LutVecChar,LutVec,nameMat] = bilinear2x2(coordType,reverseMapping, xMatSampled, yMatSampled, Lut, imgSize,intLen,optFracCell,optFrac)

nr = imgSize(1);
nc = imgSize(2);


optFrac = [optFrac(1:3) max(optFrac(2:3)) optFrac(4:end)];

wordInfo(:,1) = intLen;
for i = 1 : length(optFracCell)
    wordInfo(optFracCell{i},2) = optFrac(i);
end



Lut = round(2^optFrac(1).*Lut)./2^optFrac(1);



if coordType == 1
    Lut2 = xMatSampled - Lut;
else
    Lut2 = yMatSampled - Lut;
end


cropPt1 = [1 1]';
cropPt2 = [nc nr]';

wordInfo = [13 0 0;12 0 0; 11 0 0; 10 0 0; 13 0 0; 12 0 0];
if reverseMapping == 1
    % forward case
    wordInfo = [wordInfo;[5 0 0;6 0 0; 13 1 1;12 1 1; 8 optFrac(1) 1; 8 optFrac(1) 1]];
else
    % reverse case
    wordInfo = [wordInfo;[6 0 0;6 0 0; 13 1 1;12 1 1; 7 optFrac(1) 1; 7 optFrac(1) 1]];
    
end
wordInfo(:,1) = wordInfo(:,1) + wordInfo(:,3);

LutVec = [nc; nr;cropPt1;cropPt2; size(Lut,2);size(Lut,1);  xMatSampled(1,:)'.*2^wordInfo(9,2); yMatSampled(:,1).*2^wordInfo(10,2);Lut2(:).*2^wordInfo(11,2)];
lengthMat = [wordInfo(1:2,:);wordInfo([3 4],:);wordInfo([5 6],:);wordInfo([7 8],:);repmat(wordInfo([9],:),size(xMatSampled,2),1);repmat(wordInfo([10],:),size(yMatSampled,1),1);repmat(wordInfo([11],:),size(yMatSampled,1)*size(yMatSampled,2),1) ];

nameMat = {['ic' ] [sum(wordInfo(1,1:2))];['ir'] [sum(wordInfo(2,1:2))];['x1'] [sum(wordInfo(3,1:2))];['y1'] [sum(wordInfo(4,1:2))];['x2'] [sum(wordInfo(5,1:2))];['y2'] [sum(wordInfo(6,1:2))];['tc'] [sum(wordInfo(7,1:2))];['tr'] [sum(wordInfo(8,1:2))];[repmat('controlX',size(xMatSampled,2),1)] [size(xMatSampled,2)*sum(wordInfo(9,1:2))];[repmat('controlY',size(yMatSampled,1),1)] [size(yMatSampled,1)*sum(wordInfo(10,1:2))];[repmat('Lut',size(yMatSampled,1)*size(yMatSampled,2),1)] [size(yMatSampled,1)*size(yMatSampled,2)*sum(wordInfo(11,1:2))]};


LutVecChar = [];
for u = 1 : length(LutVec)
    if LutVec(u) >= 0
        bin_x = dec2bin(LutVec(u),lengthMat(u,1)+lengthMat(u,2));
        LutVecChar = [LutVecChar '' bin_x];
    else
        bin_x = dec2bin(2^(lengthMat(u,1)+lengthMat(u,2)) + LutVec(u),lengthMat(u,1)+lengthMat(u,2));
        LutVecChar = [LutVecChar '' bin_x];
        
    end
end



end
function makeLut(whichCam, para,LutVecOrig2RectXNum,LutVecRect2OrigXNum,LutVecRect2OrigX,LutVecOrig2RectX,Orig2RectNameMatX,Rect2OrigNameMatX,LutVecOrig2RectY,LutVecRect2OrigY,LutVecOrig2RectYNum)


supposedLen = 25+21+25+11+12+LutVecOrig2RectXNum(7)*15+LutVecOrig2RectXNum(8)*14+LutVecRect2OrigXNum(7)*15+LutVecRect2OrigXNum(8)*14+LutVecOrig2RectXNum(7)*LutVecOrig2RectXNum(8)*28+LutVecRect2OrigXNum(7)*LutVecRect2OrigXNum(8)*26;
finalVec = [LutVecRect2OrigX(1:25)...
    LutVecOrig2RectX(37:46) LutVecOrig2RectX(26:36)...   %crop1 Y(10) X(11)%                 LutVecOrig2RectX(60:71) LutVecOrig2RectX(47:59)...   %crop2 Y(12) X(13)
    LutVecOrig2RectX(60:71) LutVecRect2OrigX(47:59)...   %crop2 Y(12) X(13)
    LutVecOrig2RectX(72:82) LutVecRect2OrigX(72:83)...
    LutVecOrig2RectX(sum(cell2mat(Orig2RectNameMatX(1:8,2)))+1:sum(cell2mat(Orig2RectNameMatX(1:9,2)))) LutVecOrig2RectY(sum(cell2mat(Orig2RectNameMatX(1:9,2)))+1:sum(cell2mat(Orig2RectNameMatX(1:10,2))))...
    LutVecRect2OrigX(sum(cell2mat(Rect2OrigNameMatX(1:8,2)))+1:sum(cell2mat(Rect2OrigNameMatX(1:9,2)))) LutVecRect2OrigY(sum(cell2mat(Rect2OrigNameMatX(1:9,2)))+1:sum(cell2mat(Rect2OrigNameMatX(1:10,2))))];


data = [{dec2bin(163)};...
    {LutVecRect2OrigX(1:13)};...
    {LutVecOrig2RectX(14:25)};...
    {LutVecRect2OrigX(78:83)};...
    {LutVecRect2OrigX(72:77)};...
    {LutVecOrig2RectX(77:82)};...
    {LutVecOrig2RectX(72:76)};...
    {LutVecOrig2RectX(37:46)};...
    {LutVecOrig2RectX(26:36)};...
    LutVecRect2OrigX(47:59);...
    {LutVecOrig2RectX(60:71)}];



vec1 = [LutVecOrig2RectX(sum(cell2mat(Orig2RectNameMatX(1:8,2)))+1:sum(cell2mat(Orig2RectNameMatX(1:9,2))))];
vec1 = reshape(vec1,15,[]); vec1 = vec1';
for oo = 1 : size(vec1,1)
    data = [data;{vec1(oo,:)}];
    
end
vec2 = [LutVecOrig2RectY(sum(cell2mat(Orig2RectNameMatX(1:9,2)))+1:sum(cell2mat(Orig2RectNameMatX(1:10,2))))];
vec2 = reshape(vec2,14,[]); vec2 = vec2';
for oo = 1 : size(vec2,1)
    data = [data;{vec2(oo,:)}];
    
end
vec3 = [LutVecRect2OrigX(sum(cell2mat(Rect2OrigNameMatX(1:8,2)))+1:sum(cell2mat(Rect2OrigNameMatX(1:9,2))))];
vec3 = reshape(vec3,15,[]); vec3 = vec3';
for oo = 1 : size(vec3,1)
    data = [data;{vec3(oo,:)}];
    
end
vec4 = [LutVecRect2OrigY(sum(cell2mat(Rect2OrigNameMatX(1:9,2)))+1:sum(cell2mat(Rect2OrigNameMatX(1:10,2))))];
vec4 = reshape(vec4,14,[]); vec4 = vec4';
for oo = 1 : size(vec4,1)
    data = [data;{vec4(oo,:)}];
    
end
% forward
for jj = 1 : LutVecOrig2RectYNum(7) * LutVecOrig2RectYNum(8)
    unitLength1 = Orig2RectNameMatX{11,2}/size(Orig2RectNameMatX{11,1},1);
    startId1 = sum(cell2mat(Orig2RectNameMatX(1:10,2)));
    tmp =  [LutVecOrig2RectY(startId1+(jj-1)*unitLength1+1:startId1+jj*unitLength1) LutVecOrig2RectX(startId1+(jj-1)*unitLength1+1:startId1+jj*unitLength1)];
    data = [data;{tmp}];
    
    finalVec = [finalVec tmp];
end
% reverse
for kk = 1 : LutVecRect2OrigXNum(7) * LutVecRect2OrigXNum(8)
    unitLength2 = Rect2OrigNameMatX{11,2}/size(Rect2OrigNameMatX{11,1},1);
    startId2 = sum(cell2mat(Rect2OrigNameMatX(1:10,2)));
    tmp =  [LutVecRect2OrigY(startId2+(kk-1)*unitLength2+1:startId2+kk*unitLength2) LutVecRect2OrigX(startId2+(kk-1)*unitLength2+1:startId2+kk*unitLength2)];
    data = [data;{tmp}];
    
    finalVec = [finalVec tmp];
    
end


errLen = supposedLen - length(finalVec);

assert(errLen == 0);


headerLen = 11;

bodyPart = data(12:end,:);

fid222 = fopen(fullfile(para,strcat('LutDec',whichCam,'_1920_1080.txt')),'w');

for i = 1 : headerLen + 3
    if i < headerLen + 1
        tempDec = bin2dec(data{i});
        fprintf(fid222,sprintf('%d\n',tempDec));
    end
    if i == headerLen + 1
        tempHex = '5A5A5A5A';
        tempDec = hex2dec(tempHex);
        fprintf(fid222,sprintf('%d\n',tempDec));
    end
    if i == headerLen + 2
        tempHex = dec2hex(length(bodyPart));
        tempDec = hex2dec(tempHex);
        fprintf(fid222,sprintf('%d\n',tempDec));
    end
    if i == headerLen + 3
        tempHex = 'FFFFFFFF';
        tempDec = hex2dec(tempHex);
        fprintf(fid222,sprintf('%d\n',tempDec));
    end
    
end



for j = 1 : length(bodyPart)
    
    if j < length(bodyPart)
        
        tempBin = bodyPart{j};
        tempDec = bin2dec(tempBin);
        fprintf(fid222,sprintf('%d\n',tempDec));
        
        
    else
        
        
        tempBin = bodyPart{j};
        tempDec = bin2dec(tempBin);
        fprintf(fid222,sprintf('%d',tempDec));
        
        
        
    end
end

fclose(fid222);

end



