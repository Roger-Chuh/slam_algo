function U = ocam_undistort_map_use(OCAM_MODEL, varargin)
%
% U = ocam_undistort_map(OCAM_MODEL[, UNDIST_RES, Name, Value, ...] )
%
% builds a undistortion map and calculates the perspective camera
% matrix K for a camera modeled with the OCamCalib toolbox
%
% --- input -----
%
% OCAM_MODEL  the calibration camera model as obtained from OCamCalib
%             toolbox
% UNDIST_RES  (optional) the resolution of the undistorted image
%             defaults to standard resolution
% Name, Value (optional) additional parametes
%             'OutputView'    'same' undistorted image is the same size
%                                    as  distorted image
%                             'full' undistorted image contains all
%                                    pixels of distorted image
%
% --- output -----
%
% U         structure with the following fields:
%
%           U.map the undistortion map to be used with ocam_undistort
%           U.K   the intrinsic camera matrix K for a perspective camera,
%                 same convention as matlab IntrinsicMatrix
%           U.res target resolution
%
% 2016 Bernd Pfrommer
%
[target_res, mode] = get_res_and_mode(OCAM_MODEL, varargin);
% % % % % % target_res(1) = target_res(1)*4;
%
% precompute inverse polynomial if necessary
%
if ~isfield(OCAM_MODEL,'pol')
    width  = OCAM_MODEL.width;
    height = OCAM_MODEL.height;
    %The OCAM_MODEL does not contain the inverse polynomial pol
    OCAM_MODEL.pol = findinvpoly(OCAM_MODEL.ss,sqrt((width/2)^2+(height/2)^2));
end
assert(isfield(OCAM_MODEL, 'pol'));
%
% create an intrinsic matrix K that fits the required resolution and
% mode
%
k = make_intrinsic_matrix(OCAM_MODEL, target_res, mode);
%
% set up a rectangular even grid in the undistorted image pixel
% coordinates. This grid corresponds to destination pixels
% of the undistorted, perspective image.
%
[nx, ny] = meshgrid(1:target_res(1), 1:target_res(2));
%
% now shift and invert to go from pixel coordinates to
% homogenous world coordinates
%
n = [nx(:)'-k(1,3); ny(:)'-k(2,3)];
x = (k(1:2,1:2)^-1) * n;
xh = [-x(1,:); -x(2,:); -ones(1, target_res(1) * target_res(2))];

a = inv(k)*pextend([nx(:) ny(:)]');
err = a+xh;
%
% Project the world coordinates back to distorted pixel coordinates to
% get a fast lookup table
%
U.map = world2cam_fast(normc(xh), OCAM_MODEL)';

if 1
    if 0
        [pixRect, cam2world_3d, cam2world_3d_sphere] = Orig2RectFishEye([nx(:) ny(:)], [], k, eye(3),  [] ,OCAM_MODEL);
        [pixUndist1] = remapFishEyePix([nx(:) ny(:)]', k, OCAM_MODEL);
        figure,imshow(zeros(1080, 1920));hold on;plot(pixRect(:,1), pixRect(:,2),'.r');title('orig to rect');
        err1 = pixRect - pixUndist1';
        
        
        %     K1= [500 0 960; 0 500 540; 0 0 1];
        figure(1),clf;imshow(img);hold on;
    end
    
    imgSize = [OCAM_MODEL.height OCAM_MODEL.width];
    foc = 690+175;%150; 1000; 690 + 100;% + 150;
    K = [foc 0 imgSize(2)/2; 0 foc imgSize(1)/2; 0 0 1];
    K1 = K;
    [nxx, nyy] = meshgrid(1:1:OCAM_MODEL.width, 1:1:OCAM_MODEL.height);
    pixAll = [nxx(:) nyy(:)];
    
%     img = rgb2gray(imread('D:\Temp\20190125\sensor210\Left\imgR_0001.png'));
%     img = rgb2gray(imread('D:\Temp\20210118\fisheye\img3.jpg'));
    img = rgb2gray(imread('G:\matlab\LK\slam_algo\nanhuCalib_bak_20200921\Temp\20210118\fisheye\img3.jpg'));
    
    zeroImg = uint8(zeros(imgSize));
    zeroMask = zeros(imgSize);
    
    
    [pix_ind_1, targetImg1, mask1, pixUndist1] = get_mask(img,pixAll, K, OCAM_MODEL, 11, eye(3));
    
    [pix_ind_2, targetImg2, mask2, pixUndist2] = get_mask(img,pixAll, K, OCAM_MODEL, 21, rotx(90));
    [pix_ind_3, targetImg3, mask3, pixUndist3] = get_mask(img,pixAll, K, OCAM_MODEL, 31, rotx(-90));
    [pix_ind_4, targetImg4, mask4, pixUndist4] = get_mask(img,pixAll, K, OCAM_MODEL, 41, roty(90));
    [pix_ind_5, targetImg5, mask5, pixUndist5] = get_mask(img,pixAll, K, OCAM_MODEL, 51, roty(-90));
    
    if 0
        figure,imshow(mask1 + mask2 + mask3 + mask4 + mask5, [])
    end
    
    segMask = mask1 + mask4 + mask5;
    
    centerPart = mask1 > 0;
    leftPart = segMask == 41;
    rightPart = segMask == 51;
    updownPart = segMask == 0;
    
    
    [L, LL] = bwlabel(updownPart > 0, 8);
    
    
    for i = 1 : LL
        bwTemp =  L == (i);
        tempArea(:,:,i) = immultiply(updownPart,bwTemp) > 0;
        [y, x] = ind2sub(size(tempArea(:,:,i)), find(tempArea(:,:,i) > 0));
        area(i) = sum(updownPart(bwTemp));
        yCoord(i,1) = mean(y);
    end
    validArea = find(area > 10000);
    %     yCoord_ = yCoord(area > 10000);
    [~, sortId] = sort(yCoord(validArea),'descend');
    %     [~, minId] = min(yCoord(validArea));
    downPart = tempArea(:,:,validArea(sortId(1)));
    try
        upPart = tempArea(:,:,validArea(sortId(2)));
    catch
        upPart = tempArea(:,:,validArea(sortId(1)));
    end
    
    if 0
        figure,imshow(centerPart + 2.*leftPart + 3.*rightPart + 4.*upPart + 5.*downPart, []);
    end
%     hold on;plot(pixUndist4(pix_ind_4,1), pixUndist4(pix_ind_4,2), '.r');
    
    pixUndist4r = round(pixUndist4(pix_ind_4,:));
    ind4 = sub2ind(imgSize, pixUndist4r(:,2), pixUndist4r(:,1));
    in4 = find(leftPart(ind4) == 0);
    validPix4 = pixAll(pix_ind_4(in4),:);
%     figure,imshow(targetImg4);hold on;plot(validPix4(:,1), validPix4(:,2), '.r');
    
    %     figure(1),plot(pt1(:,1), pt1(:,2),'.');
    
    origPt = [1 1;imgSize(2) 1;1 imgSize(1); imgSize(2) imgSize(1)];
    [cornerPt] = remapFishEyePixInv([origPt]', K, OCAM_MODEL);
    
    
    [optimX2, error2] = optAng(cornerPt([3 4],:), [[origPt(1,1) origPt(1,2)-1];[origPt(2,1) origPt(2,2)-1]], 'x', K, OCAM_MODEL, 10); % down;
    [pix_ind_22, targetImg22, mask22, pixUndist22] = get_mask(img,pixAll, K, OCAM_MODEL, 21, rotx(optimX2));
    
    [optimX3, error3] = optAng(cornerPt([1 2],:), [[origPt(3,1) origPt(3,2)-1];[origPt(4,1) origPt(4,2)-1]], 'x', K, OCAM_MODEL, -10); % up;
    [pix_ind_33, targetImg33, mask33, pixUndist33] = get_mask(img,pixAll, K, OCAM_MODEL, 31, rotx(optimX3));
    
    [optimY4, error4] = optAng(cornerPt([1 3],:), [[origPt(2,1)-1 origPt(2,2)];[origPt(4,1)-1 origPt(4,2)]], 'y', K, OCAM_MODEL, 10); % left;
    [pix_ind_44, targetImg44, mask44, pixUndist44] = get_mask(img,pixAll, K, OCAM_MODEL, 41, roty(optimY4));
    
    [optimY5, error5] = optAng(cornerPt([2 4],:), [[origPt(1,1)-1 origPt(1,2)];[origPt(3,1)-1 origPt(3,2)]], 'y', K, OCAM_MODEL, -10); % right;
    [pix_ind_55, targetImg55, mask55, pixUndist55] = get_mask(img,pixAll, K, OCAM_MODEL, 51, roty(optimY5));
    
    figure,subplot(2,2,1);imshow(img);subplot(2,2,2);imshow([zeroImg targetImg33 zeroImg;targetImg44 targetImg1 targetImg55;zeroImg targetImg22 zeroImg]);subplot(2,2,3);imshow(mask1 + mask22 + mask33 + mask44 + mask55, []);
    
    figure,imshow(img)
    figure,imshow([zeroImg targetImg33 zeroImg;targetImg44 targetImg1 targetImg55;zeroImg targetImg22 zeroImg]);
    
    sdgjhd = 1;
    
    if 0
        
        targetImg1 = uint8(reshape(warped_intensities, 1080, 1920));
        
        
        pixUndist2 = remapRectFishEye([nx(:) ny(:)]', K1, K1,zeros(5,1), rotx(90),OCAM_MODEL);
        warped_intensities = interp2(double(img), pixUndist2(:,1), pixUndist2(:,2), 'linear', 0);
        targetImg2 = uint8(reshape(warped_intensities, 1080, 1920));
        figure(1),plot(pixUndist2(:,1), pixUndist2(:,2),'.');
        
        pixUndist2 = remapRectFishEye([nx(:) ny(:)]', K1, K1,zeros(5,1), rotx(-90),OCAM_MODEL);
        warped_intensities = interp2(double(img), pixUndist2(:,1), pixUndist2(:,2), 'linear', 0);
        targetImg3 = uint8(reshape(warped_intensities, 1080, 1920));
        figure(1),plot(pixUndist2(:,1), pixUndist2(:,2),'.');
        
        pixUndist2 = remapRectFishEye([nx(:) ny(:)]', K, K,zeros(5,1), roty(90),OCAM_MODEL);
        warped_intensities = interp2(double(img), pixUndist2(:,1), pixUndist2(:,2), 'linear', 0);
        targetImg4 = uint8(reshape(warped_intensities, 1080, 1920));
        figure(1),plot(pixUndist2(:,1), pixUndist2(:,2),'.');
        
        pixUndist2 = remapRectFishEye([nx(:) ny(:)]', K, K,zeros(5,1), roty(-90),OCAM_MODEL);
        warped_intensities = interp2(double(img), pixUndist2(:,1), pixUndist2(:,2), 'linear', 0);
        targetImg5 = uint8(reshape(warped_intensities, 1080, 1920));
        figure(1),plot(pixUndist2(:,1), pixUndist2(:,2),'.');
        
    end
end

%     figure,subplot(1,2,1);imshow(img);subplot(1,2,2);imshow([zeroImg targetImg3 zeroImg;targetImg4 targetImg1 targetImg5;zeroImg targetImg2 zeroImg]);



if 0
    
    
    %      [gridX, gridY] = meshgrid([1:w], [1:h]);
    imageCorners___ = [nx(:) ny(:)]';
    xworld___ = cam2world(imageCorners___, OCAM_MODEL);
    err0 = cam2world_3d_sphere - xworld___;
    
    figure,plot3(cam2world_3d_sphere(1,1:1000:end),cam2world_3d_sphere(2,1:1000:end),cam2world_3d_sphere(3,1:1000:end),'.');axis equal
    
    
    [pixUndist] = remapFishEyePixInv([nx(:) ny(:)]', k, OCAM_MODEL);
    pixUndist2 = remapRectFishEye([nx(:) ny(:)]', k, k,zeros(5,1), eye(3),OCAM_MODEL);
    figure,imshow(zeros(1080, 1920));hold on;plot(pixUndist(:,1), pixUndist(:,2),'.r');title('rect to orig');
    err2 = [U.map pixUndist pixUndist2];
end

%
% transpose intrinsic matrix to follow matlab convention
%
U.K = k';
U.res = target_res;
end

function k = make_intrinsic_matrix(ocam_model, target_res, mode)
if mode == 0
    k = make_normal_k(ocam_model, target_res);
else
    if  mode == 1
        k = make_high_k(ocam_model, target_res);
    else
        error('invalid mode!');
    end
end
end

function k = make_normal_k(ocam_model, target_res)
%
% make k such that top left and bottom right corners
% of distorted and undistorted image line up
%
scal = 20;
imageCorners = [[1, 1]'+[target_res(1)/scal;target_res(2)/scal], [ocam_model.width, ocam_model.height]'-[target_res(1)/scal;target_res(2)/scal]];
xworld = cam2world(imageCorners, ocam_model);
X = [xworld(1,:)./xworld(3,:); xworld(2,:)./xworld(3,:); ones(1, size(xworld,2))];
xmin = X(1,1); xmax = X(1,2);
ymin = X(2,1); ymax = X(2,2);
umin = 1; umax = target_res(1);
vmin = 1; vmax = target_res(2);

k = make_k(xmin, xmax, ymin, ymax, umin, umax, vmin, vmax);
end

function k = make_high_k(ocam_model, target_res)
change = 0;
% target_res(1) = target_res(1)*2;
w = ocam_model.width;
h = ocam_model.height;

imageCorners = [[1, h/2]', [w/2, 1]', [w, h/2]', [w/2, h]'];
xworld = cam2world(imageCorners, ocam_model);

if 0
    [gridX, gridY] = meshgrid([1:w], [1:h]);
    imageCorners___ = [gridX(:) gridY(:)]';
    xworld___ = cam2world(imageCorners___, ocam_model);
    figure,plot3(xworld___(1,1:1000:end),xworld___(2,1:1000:end),xworld___(3,1:1000:end),'.');axis equal
    inflag = find(xworld___(3,:) < -0.25);
    xworld_in = xworld___(:,xworld___(3,:)<-0.25);
    outflag = find(xworld___(3,:) >= -0.25);
    xworld_out = xworld___(:,xworld___(3,:)>=-0.25);
    pixList = [gridX(inflag)' gridY(inflag)'];
    corner = [min(pixList); max(pixList)];
    [gridXin, gridYin] = meshgrid(corner(1,1):corner(2,1), corner(1,2):corner(2,2));
    gridXin = gridXin(1:2*floor(size(gridXin,1)/2),1:2*floor(size(gridXin,2)/2));
    gridYin = gridYin(1:2*floor(size(gridYin,1)/2),1:2*floor(size(gridYin,2)/2));
    indIn = sub2ind(size(gridX), gridYin(:), gridXin(:));
    figure,plot3(xworld___(1,1:1000:end),xworld___(2,1:1000:end),xworld___(3,1:1000:end),'.');hold on;plot3(xworld___(1,indIn),xworld___(2,indIn),xworld___(3,indIn),'.');axis equal
    figure,plot3(xworld_in(1,1:1:end),xworld_in(2,1:1:end),xworld_in(3,1:1:end),'.');axis equal;hold on;plot3(xworld_out(1,1:1:end),xworld_out(2,1:1:end),xworld_out(3,1:1:end),'.');
    mask = zeros(h,w);
    mask(inflag) = 1;
    figure,imshow(mask);hold on;plot(corner(:,1), corner(:,2),'*g');
end
skhg = 1;
thr =  -0.25; 0;
if 1
    rowpix = [1:w;h/2.*ones(1,w)];
    colpix = [round(w/2*1).*ones(1,h);1:h];
    %     imageCornersr = [[1, h/2]', [w/2, 1]', [w, h/2]', [w/2, h]'];
    xworld_row = cam2world(rowpix, ocam_model);
    xworld_col = cam2world(colpix, ocam_model);
    % %     xworld_row = xworld_row(:,xworld_row(3,:)<-0.05);
    % %     xworld_col = xworld_col(:,xworld_col(3,:)<-0.05);
    xworld_row = xworld_row(:,xworld_row(3,:)<thr);
    xworld_col = xworld_col(:,xworld_col(3,:)<thr);
    xworld = xworld(:,xworld(3,:)<-0.3);
    X_row = [xworld_row(1,:)./xworld_row(3,:); xworld_row(2,:)./xworld_row(3,:); ones(1, size(xworld_row,2))];
    X_col = [xworld_col(1,:)./xworld_col(3,:); xworld_col(2,:)./xworld_col(3,:); ones(1, size(xworld_col,2))];
    
    xmin = min(X_row(1,:));
    xmax = max(X_row(1,:));
    ymin = min(X_col(2,:));
    ymax = max(X_col(2,:));
    
    
else
    
    
    X = [xworld(1,:)./xworld(3,:); xworld(2,:)./xworld(3,:); ones(1, size(xworld,2))];
    
    imageCorners_ = imageCorners([2 1],:);
    xworld_ = cam2world_(imageCorners_, ocam_model);
    X_ = [xworld_(1,:)./xworld_(3,:);xworld_(2,:)./xworld_(3,:);ones(1,size(xworld_,2))];
    X_ = [-X_(2,:); -X_(1,:); X_(3,:)];
    
    
    if 1
        xmin = X(1,1); xmax = X(1,3);
        if xmin > xmax
            %     xtmp = xmin;
            %     xmin = xmax;
            %     xmax = xtmp;
            % % %         xmin = -xmin;
            % % %         xmax = -xmax;
            xmin = -max(abs(X(1,:)));
            xmax = max(abs(X(1,:)));
        end
        ymin = X(2,2); ymax = X(2,4);
        % if ymin > ymax
        %     ytmp = ymin;
        %     ymin = ymax;
        %     ymax = ytmp;
        % end
    else
        xmin = -max(abs(X(1,:)));
        xmax = max(abs(X(1,:)));
        ymin = -max(abs(X(2,:)));
        ymax = max(abs(X(2,:)));
        
    end
end
umin = 1; umax = target_res(1);
vmin = 1; vmax = target_res(2);
k = make_k(xmin, xmax, ymin, ymax, umin, umax, vmin, vmax);
end

function k = make_k(xmin, xmax, ymin, ymax, umin, umax, vmin, vmax)
zmax = 1;
zmin = 1;
fx = (umax*zmax - umin*zmin) / (xmax-xmin);
fy = (vmax*zmax - vmin*zmin) / (ymax-ymin);
cx = (zmin*umin - fx * xmin) / zmin;
cy = (zmin*vmin - fy * ymin) / zmin;
k=[fx, 0, cx; 0, fy, cy; 0, 0, 1];
end

function [res, mode] = get_res_and_mode(model, varg)
%
% parses input arguments.
%
res = [model.width, model.height];
mode = 0;
if length(varg) <= 0
    return
end

istart = 1;
if isnumeric(varg{istart})
    res = varg{istart};
    istart = istart + 1;
end
for i = istart:length(varg)
    if ~ischar(varg{i})
        error 'expected string parameters'
    end
    if strcmp(varg{i}, 'OutputView')
        if (i >= length(varg))
            error('no value provided for OutputView');
        end
        val = varg{i+1};
        if ~ischar(val)
            error('value for OutputView must be string');
        end
        if strcmp(val, 'full')
            mode = 0;
        else
            if strcmp(val, 'same')
                mode = 1;
            else
                error('invalid value for OutputView');
            end
        end
    end
end
end

%
% ----------------------------------------------------------------------
% copied from OCAMCALIB toolbox and hacked to modify the orientation
% of the axis.
%


%CAM2WORLD Project a give pixel point onto the unit sphere
%   M=CAM2WORLD=(m, ocam_model) returns the 3D coordinates of the vector
%   emanating from the single effective viewpoint on the unit sphere
%
%   m=[rows;cols] is a 2xN matrix containing the pixel coordinates of the image
%   points.
%
%   "ocam_model" contains the model of the calibrated camera.
%
%   M=[X;Y;Z] is a 3xN matrix with the coordinates on the unit sphere:
%   thus, X^2 + Y^2 + Z^2 = 1
%
%   Last update May 2009
%   Copyright (C) 2006 DAVIDE SCARAMUZZA
%   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org

function M=cam2world(m, ocam_model)

n_points = size(m,2);

ss = ocam_model.ss;
xc = ocam_model.xc;
yc = ocam_model.yc;
c = ocam_model.c;
d = ocam_model.d;
e = ocam_model.e;

A = [1, e;
    d, c];
T = [yc;xc]*ones(1,n_points);
m = A^-1*(m-T);
M = getpoint(ss,m);
M = normc(M);

end

function w=getpoint(ss,m)

% Given an image point it returns the 3D coordinates of its correspondent optical
% ray

w = [-m(1,:) ; -m(2,:) ; polyval(ss(end:-1:1),sqrt(m(1,:).^2+m(2,:).^2)) ];

end

function M=cam2world_(m, ocam_model)

n_points = size(m,2);

ss = ocam_model.ss;
xc = ocam_model.xc;
yc = ocam_model.yc;
width = ocam_model.width;
height = ocam_model.height;
c = ocam_model.c;
d = ocam_model.d;
e = ocam_model.e;

A = [c,d;
    e,1];
T = [xc;yc]*ones(1,n_points);

m = A^-1*(m-T);
M = getpoint_(ss,m);
M = normc(M); %normalizes coordinates so that they have unit length (projection onto the unit sphere)
end
function w=getpoint_(ss,m)

% Given an image point it returns the 3D coordinates of its correspondent optical
% ray

w = [m(1,:) ; m(2,:) ; polyval(ss(end:-1:1),sqrt(m(1,:).^2+m(2,:).^2)) ];
end

%FINDINVPOLY finds the inverse polynomial specified in the argument.
%   [POL, ERR, N] = FINDINVPOLY(SS, RADIUS, N) finds an approximation of the inverse polynomial specified in OCAM_MODEL.SS.
%   The returned polynomial POL is used in WORLD2CAM_FAST to compute the reprojected point very efficiently.
%
%   SS is the polynomial which describe the mirrror/lens model.
%   RADIUS is the radius (pixels) of the omnidirectional picture.
%   ERR is the error (pixel) that you commit in using the returned
%   polynomial instead of the inverse SS. N is searched so that
%   that ERR is < 0.01 pixels.
%
%   Copyright (C) 2008 DAVIDE SCARAMUZZA, ETH Zurich
%   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org

function [pol, err, N] = findinvpoly(ss, radius)

if nargin < 3
    maxerr = inf;
    N = 1;
    while maxerr > 0.01 %Repeat until the reprojection error is smaller than 0.01 pixels
        N = N + 1;
        [pol, err] = findinvpoly2(ss, radius, N);
        maxerr = max(err);
    end
else
    [pol, err, N] = findinvpoly2(ss, radius, N)
end

end

function [pol, err, N] = findinvpoly2(ss, radius, N)

theta = [-pi/2:0.01:1.20];
r     = invFUN(ss, theta, radius);
ind   = find(r~=inf);
theta = theta(ind);
r     = r(ind);

pol = polyfit(theta,r,N);
err = abs( r - polyval(pol, theta)); %approximation error in pixels

end

function r=invFUN(ss, theta, radius)

m=tan(theta);

r=[];
poly_coef=ss(end:-1:1);
poly_coef_tmp=poly_coef;
for j=1:length(m)
    poly_coef_tmp(end-1)=poly_coef(end-1)-m(j);
    rhoTmp=roots(poly_coef_tmp);
    res=rhoTmp(find(imag(rhoTmp)==0 & rhoTmp>0 & rhoTmp<radius ));
    if isempty(res) | length(res)>1
        r(j)=inf;
    else
        r(j)=res;
    end
end

end

%WORLD2CAM projects a 3D point on to the image
%   m=WORLD2CAM_FAST(M, ocam_model) projects a 3D point on to the
%   image and returns the pixel coordinates. This function uses an approximation of the inverse
%   polynomial to compute the reprojected point. Therefore it is very fast.
%
%   M is a 3xN matrix containing the coordinates of the 3D points: M=[X;Y;Z]
%   "ocam_model" contains the model of the calibrated camera.
%   m=[rows;cols] is a 2xN matrix containing the returned rows and columns of the points after being
%   reproject onto the image.
%
%   Copyright (C) 2008 DAVIDE SCARAMUZZA, ETH Zurich
%   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org

function m = world2cam_fast(M, ocam_model)

ss = ocam_model.ss;
xc = ocam_model.xc;
yc = ocam_model.yc;
c = ocam_model.c;
d = ocam_model.d;
e = ocam_model.e;
pol = ocam_model.pol;

npoints = size(M, 2);
theta = zeros(1,npoints);

NORM = sqrt(M(1,:).^2 + M(2,:).^2);

ind0 = find( NORM == 0); %these are the scene points which are along the z-axis
NORM(ind0) = eps; %this will avoid division by ZERO later

theta = atan( M(3,:)./NORM );

rho = -polyval( pol , theta ); %Distance in pixel of the reprojected points from the image center

x = M(1,:)./NORM.*rho ;
y = M(2,:)./NORM.*rho ;

%Add center coordinates
m(1,:) = x*1 + y*e  + yc;
m(2,:) = x*d + y*c  + xc;

end
function [pix_ind_1, targetImg1, mask, pixUndist2] = get_mask(img,pixAll, K, OCAM_MODEL, n, rotMat)
mask = zeros(OCAM_MODEL.height, OCAM_MODEL.width);
pixUndist2 = remapRectFishEye(pixAll', K, K,zeros(5,1),rotMat,OCAM_MODEL);
warped_intensities = interp2(double(img), pixUndist2(:,1), pixUndist2(:,2), 'linear', 0);
targetImg1 = uint8(reshape(warped_intensities,OCAM_MODEL.height, OCAM_MODEL.width));
pt = pixUndist2; %unique(round(pixUndist2), 'rows');
inFlag1 = find(pt(:,1) >= 0.5 & pt(:,1) < OCAM_MODEL.width & pt(:,2) >=0.5 & pt(:,2) < OCAM_MODEL.height);
pt = pt(inFlag1, :);
ind1 = sub2ind([OCAM_MODEL.height, OCAM_MODEL.width], round(pt(:,2)), round(pt(:,1)));
% z1 = zeroMask;
mask(ind1) = n;
mask = imfill(mask, 8);
se = strel('square',[3]);
mask = imdilate(mask,se);
mask = imerode(mask,se);

pix_ind_1 = [inFlag1];
end
function [optimX, error] = optAng(centerCnr, otherCnr, rot, K, OCAM_MODEL, theta0)


if theta0 > 0
    rng = [0 180];
    
else
    rng = [-180 0];
    
end

[optimX, ~, exitFlag] = fminbnd(@ObjectiveFunction, rng(1), rng(2), optimset('TolX',0.0000001,'Display','off'));
error = ObjectiveFunction(optimX);

    function Err = ObjectiveFunction(X)
        
        if strcmp(rot ,'x')
            pixUndist2 = remapRectFishEye(otherCnr', K, K,zeros(5,1),rotx(X),OCAM_MODEL);
        else
            pixUndist2 = remapRectFishEye(otherCnr', K, K,zeros(5,1),roty(X),OCAM_MODEL);
        end
        
        [~, err] = NormalizeVector(pixUndist2 - centerCnr);
        Err = mean(err);
    end

end