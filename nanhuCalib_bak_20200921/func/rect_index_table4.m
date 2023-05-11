function [Irec,ind_new,ind_1,ind_2,ind_3,ind_4,a1,a2,a3,a4,imgL1] = rect_index_table4(I,R,f,c,k,alpha,KK_new,imgL)



draw = 1; 0;
reverseMapping = 1;

marg = 10;
skipX = 75; skipY = 75;
skipX = 85; skipY = 85;
dltErr = 0;0.5;
rowNum = size(I,1);
colNum = size(I,2);

endThr = 3;
change = 1;

if change
    scaleSizeY = 120; 120;%40;10;40;60;20;20;60;60;10;20;2;36;2;  %rowNum/yLen;
    scaleSizeX = 160; 160;%40;10;40;80;20;20;80;10;20;2;48;2;  colNum/xLen;
    roundNum = 10;
    yLen = rowNum/scaleSizeY;30;  %192;40;
    xLen = colNum/scaleSizeX;40;  %108;30;
else
    yLen = 27;30;  %192;40;
    xLen = 48;60;  %108;30;
    scaleSizeY = rowNum/yLen;
    scaleSizeX = colNum/xLen;
    
    roundNum = 10;
    
end



% % % % skipX = 10; skipY = 10;
if nargin < 5,
    k = [0;0;0;0;0];
    if nargin < 4,
        c = [0;0];
        if nargin < 3,
            f = [1;1];
            if nargin < 2,
                R = eye(3);
                if nargin < 1,
                    error('ERROR: Need an image to rectify');
                    %break;
                end;
            end;
        end;
    end;
end;


if nargin < 7,
    if nargin < 6,
        KK_new = [f(1) 0 c(1);0 f(2) c(2);0 0 1];
    else
        KK_new = alpha; % the 6th argument is actually KK_new
    end;
    alpha = 0;
end;



% Note: R is the motion of the points in space
% So: X2 = R*X where X: coord in the old reference frame, X2: coord in the new ref frame.


if ~exist('KK_new'),
    KK_new = [f(1) alpha_c*fc(1) c(1);0 f(2) c(2);0 0 1];
end;


[nr,nc] = size(I);

Irec = 255*ones(nr,nc);

[mx,my] = meshgrid(1:nc, 1:nr);
px = reshape(mx',nc*nr,1);
py = reshape(my',nc*nr,1);

if 1
    rays = inv(KK_new)*[(px - 1)';(py - 1)';ones(1,length(px))];
else
    rays = inv(KK_new)*[(px - 0)';(py - 0)';ones(1,length(px))];
end

% Rotation: (or affine transformation):

rays2 = R'*rays;

x = [rays2(1,:)./rays2(3,:);rays2(2,:)./rays2(3,:)];


% Add distortion:
xd = apply_distortion(x,k);


% Reconvert in pixels:

px2_ = f(1)*(xd(1,:)+alpha*xd(2,:))+c(1);
py2_ = f(2)*xd(2,:)+c(2);

px2 = f(1)*(xd(1,:)+alpha*xd(2,:))+c(1);
py2 = f(2)*xd(2,:)+c(2);


% Interpolate between the closest pixels:

px_0 = floor(px2);
py_0 = floor(py2);


% % good_points = find((px_0 >= 0) & (px_0 <= (nc-2)) & (py_0 >= 0) & (py_0 <= (nr-2)));
good_points = find((px_0 > 0) & (px_0 <= (nc-2)) & (py_0 > 0) & (py_0 <= (nr-2)));

px2 = px2(good_points);
py2 = py2(good_points);
px_0 = px_0(good_points);
py_0 = py_0(good_points);

alpha_x = px2 - px_0;
alpha_y = py2 - py_0;

a1 = (1 - alpha_y).*(1 - alpha_x);
a2 = (1 - alpha_y).*alpha_x;
a3 = alpha_y .* (1 - alpha_x);
a4 = alpha_y .* alpha_x;

if 1
    ind_1 = px_0 * nr + py_0 + 1;
    ind_2 = (px_0 + 1) * nr + py_0 + 1;
    ind_3 = px_0 * nr + (py_0 + 1) + 1;
    ind_4 = (px_0 + 1) * nr + (py_0 + 1) + 1;
else
    ind_1 = (px_0-1) * nr + py_0 + 0;
    ind_2 = (px_0 + 0) * nr + py_0 + 0;
    ind_3 = (px_0-1) * nr + (py_0 + 1) + 0;
    ind_4 = (px_0 + 0) * nr + (py_0 + 1) + 0;
end
ind_new = (px(good_points)-1)*nr + py(good_points);


Irec(ind_new) = a1 .* I(ind_1) + a2 .* I(ind_2) + a3 .* I(ind_3) + a4 .* I(ind_4);

[py11,px11] = ind2sub(size(I),ind_1);
[py22,px22] = ind2sub(size(I),ind_2);
[py33,px33] = ind2sub(size(I),ind_3);
[py44,px44] = ind2sub(size(I),ind_4);
[xu,yu] = meshgrid(1:nc, 1:nr);

% % imgL = imread(strcat('D:\bk_20180627_\nextvpu\algorithm\output\calibration\cbccalib\stereo_pairs\iEye\hua_01_01','\cbpair_left001.png'));
% % nim = interp2(xu,yu,double(rgb2gray(imgL)),reshape(px2_,nc,nr)',reshape(py2_,nc,nr)');

% % imgL = imread('D:\bk_20180627_\nextvpu\algorithm\output\calibration\cbccalib\stereo_pairs\iEye\i2c15_6\cbpair_left001.bmp');
% % nim = interp2(xu,yu,double(rgb2gray(imgL)),reshape(px2_,nc,nr)',reshape(py2_,nc,nr)');
% % figure,imshow(ones(480,640));hold on;plot(cbcXYLR{1}(1,:),cbcXYLR{1}(2,:),'or');plot(pixDist{1}(:,1),pixDist{1}(:,2),'.g');plot(pixDist{2}(:,1),pixDist{2}(:,2),'.b');legend('orig','rect-mod','rect-orig');




% xd=apply_distortion(x,kc);
% px2__=f(1)*px2_+c(1);
% py2__=f(2)*xd(2,:)+c(2);
Kinit = [f(1) 0 c(1);0 f(2) c(2);0 0 1];
skipNum = 40;

figure(11),plotUndistortion(nc, nr, Kinit, Kinit,k,eye(3),skipNum);
figure(12),pixNewAll = plotUndistortion(nc, nr, Kinit, KK_new,k,R,skipNum);
% % [xuu,yuu] = meshgrid(1:nc/skipNum:nc, 1:nr/skipNum:nr);


if 0
    [xuu2, yuu2] = meshgrid(1:(colNum-1)/(xLen-1):colNum,1:(rowNum-1)/(yLen-1):rowNum);
else
    
    aTmp = scaleSizeX/2+0.5:(colNum)/(xLen):colNum-1;
    bTmp = scaleSizeY/2+0.5:(rowNum)/(yLen):rowNum-1;
    Nx = 100;
    aTmp1 = nc/2+0.5-scaleSizeX/2 : -(colNum)/(xLen) : nc/2+0.5-scaleSizeX/2-Nx*(colNum)/(xLen);
    aTmp1 = aTmp1(end:-1:1);
    aTmp2 = nc/2+0.5+scaleSizeX/2 : (colNum)/(xLen) : nc/2+0.5+scaleSizeX/2+Nx*(colNum)/(xLen);
    aTmp = [aTmp1 aTmp2];
    cntX = 0;
    goodX = 10;
    while (~(aTmp(1)>0 || aTmp(end)<nc)) || (goodX < 3)
        if (aTmp(1)>0 || aTmp(end)<nc)
            goodX = goodX + 1;
        end
        cntX = cntX + 1;
        Nx = Nx-1;
        aTmp1 = nc/2+0.5-scaleSizeX/2 : -(colNum)/(xLen) : nc/2+0.5-scaleSizeX/2-Nx*(colNum)/(xLen);
        aTmp1 = aTmp1(end:-1:1);
        aTmp2 = nc/2+0.5+scaleSizeX/2 : (colNum)/(xLen) : nc/2+0.5+scaleSizeX/2+Nx*(colNum)/(xLen);
        aTmp = [aTmp1 aTmp2];
        ATmp{cntX,1} = aTmp;
    end
    aTmp_ = ATmp{end-endThr};
    %     aTmp_ = ATmp{end};
    
    Ny = 100;
    bTmp1 = nr/2+0.5-scaleSizeY/2 : -(rowNum)/(yLen) : nr/2+0.5-scaleSizeY/2-Ny*(rowNum)/(yLen);
    bTmp1 = bTmp1(end:-1:1);
    bTmp2 = nr/2+0.5+scaleSizeY/2 : (rowNum)/(yLen) : nr/2+0.5+scaleSizeY/2+Ny*(rowNum)/(yLen);
    bTmp = [bTmp1 bTmp2];
    cntY = 0;
    goodY = 10;
    while (~(bTmp(1)>0 || bTmp(end)<nr)) || goodY < 3
        if (bTmp(1)>0 || bTmp(end)<nr)
            goodY = goodY + 1;
        end
        cntY = cntY + 1;
        Ny = Ny-1;
        % %         bTmp1 = nc/2+0.5-scaleSizeY/2 : -(colNum)/(xLen) : nc/2+0.5-scaleSizeY/2-Nx*(colNum)/(xLen);
        % %         bTmp1 = bTmp1(end:-1:1);
        % %         bTmp2 = nc/2+0.5+scaleSizeY/2 : (colNum)/(xLen) : nc/2+0.5+scaleSizeY/2+Nx*(colNum)/(xLen);
        % %         bTmp = [bTmp1 bTmp2];
        % %
        bTmp1 = nr/2+0.5-scaleSizeY/2 : -(rowNum)/(yLen) : nr/2+0.5-scaleSizeY/2-Ny*(rowNum)/(yLen);
        bTmp1 = bTmp1(end:-1:1);
        bTmp2 = nr/2+0.5+scaleSizeY/2 : (rowNum)/(yLen) : nr/2+0.5+scaleSizeY/2+Ny*(rowNum)/(yLen);
        bTmp = [bTmp1 bTmp2];
        
        BTmp{cntY,1} = bTmp;
    end
    bTmp_ = BTmp{end-endThr};
    %     bTmp_ = BTmp{end};
    [xuu2, yuu2] = meshgrid(aTmp_,bTmp_);
    if 1
        colNew = size(xuu2,2)*scaleSizeX;
        rowNew = size(xuu2,1)*scaleSizeY;
    else
        colNew = nc;
        rowNew = nr;
        
    end
    if 0
        if change
            aTmpp = 0.5:(colNum)/(xLen):0.5+(xLen-1)*scaleSizeX;
            %     aTmpp = aTmpp + (colNum/2+0.5-mean(aTmpp));
            bTmpp = 0.5:(rowNum)/(yLen):0.5+(yLen-1)*scaleSizeY;
            %     bTmpp = bTmpp + (rowNum/2+0.5-mean(bTmpp));
        else
            aTmpp = 0:colNum/(xLen-1):colNum;
            bTmpp = 0:rowNum/(yLen-1):rowNum;
            
        end
        
        aTmpp = round(aTmpp + (colNum/2+0.5-mean(aTmpp)),roundNum);
        bTmpp = round(bTmpp + (rowNum/2+0.5-mean(bTmpp)),roundNum);
        [xuu2, yuu2] = meshgrid(aTmpp, bTmpp);
        
    end
    
    
    [xuuAll,yuuAll] = meshgrid(1-(colNew-nc)/2:1:colNew-(colNew-nc)/2, 1-(rowNew-nr)/2:1:rowNew-(rowNew-nr)/2);
    % %     [xuuAll,yuuAll] = meshgrid(aTmpp(1):mean(diff(aTmpp))/4:aTmpp(end), bTmpp(1):mean(diff(bTmpp))/4:bTmpp(end));
    
    xuuAllUse = xuuAll((rowNew-nr)/2+1:end-(rowNew-nr)/2,(colNew-nc)/2+1:end-(colNew-nc)/2);
    yuuAllUse = yuuAll((rowNew-nr)/2+1:end-(rowNew-nr)/2,(colNew-nc)/2+1:end-(colNew-nc)/2);
    if reverseMapping == 0
        pixOrigAll = remapRect([xuuAllUse(:) yuuAllUse(:)]', KK_new, Kinit,k, R);
    else
        pixOrigAll = Orig2Rect([xuuAllUse(:) yuuAllUse(:)], Kinit, KK_new, R, k);
    end
    xRectMatRef = reshape(pixOrigAll(:,1),size(xuuAllUse));
    yRectMatRef = reshape(pixOrigAll(:,2),size(xuuAllUse));
    
    
    
    if 0
        [xuu2, yuu2] = meshgrid(0.5:(colNum)/(xLen):colNum+1,0.5:(rowNum)/(yLen):rowNum+1);
    end
    if 0
        [xuu2, yuu2] = meshgrid(scaleSizeX+0.5:(colNum)/(xLen):colNum,scaleSizeY+0.5:(rowNum)/(yLen):rowNum+0);
    end
    
    
    
    if 0
        [xuu2, yuu2] = meshgrid(scaleSizeX/2+0.5:(colNum)/(xLen):colNum-1,scaleSizeY/2+0.5:(rowNum)/(yLen):rowNum-1);
    elseif 0
        [xuu2, yuu2] = meshgrid(scaleSizeX/2+0.5-(colNum)/(xLen):(colNum)/(xLen):colNum-1+(colNum)/(xLen),scaleSizeY/2+0.5-(rowNum)/(yLen):(rowNum)/(yLen):rowNum-1+(rowNum)/(yLen));
    else
        skgjc = 1;
    end
    
    
    %     [xuu2, yuu2] = meshgrid(0.5:(colNum)/(xLen):colNum-1,0.5:(rowNum)/(yLen):rowNum-1);
end

if 0
    figure,plot(diff(aTmpp))
    size(xuuAll)./size(xuu2)
    figure,plot(diff(bTmpp))
end




[xuu2_, yuu2_] = meshgrid(1:size(xuu2,1),1:size(xuu2,1));
if reverseMapping == 0
    pixOrig_2 = remapRect([xuu2(:) yuu2(:)]', KK_new, Kinit,k, R);
else
    pixOrig_2 = Orig2Rect([xuu2(:) yuu2(:)], Kinit,KK_new, R, k);
end
xOrigMat = reshape(pixOrig_2(:,1),size(xuu2));
yOrigMat = reshape(pixOrig_2(:,2),size(xuu2));

figure,imshow(zeros(nr,nc));hold on;plot(xuu2(:),yuu2(:),'.r');plot(pixOrig_2(:,1),pixOrig_2(:,2),'.g');

if 1
    xRectMat = imresize(xOrigMat,size(xuuAll),'Method','bicubic');
    yRectMat = imresize(yOrigMat,size(xuuAll),'Method','bicubic');
    %     xRectMat = xRectMat + dltErr;
    %     yRectMat = yRectMat + dltErr;
elseif 0
    %     xRectMat = imresize(xOrigMat,[nr nc],'Method','bilinear');
    %     yRectMat = imresize(yOrigMat,[nr nc],'Method','bilinear');
    xRectMat = imresize(xOrigMat,size(xuuAll),'Method','bilinear');
    yRectMat = imresize(yOrigMat,size(xuuAll),'Method','bilinear');
    
elseif 0
    xRectMat = bicubic8x8FastUse(xOrigMat,scaleSizeX,scaleSizeY);
    yRectMat = bicubic8x8FastUse(yOrigMat,scaleSizeX,scaleSizeY);
elseif 1
    xRectMat = bicubic6x6FastUse(xOrigMat,scaleSizeX,scaleSizeY);
    yRectMat = bicubic6x6FastUse(yOrigMat,scaleSizeX,scaleSizeY);
    
else
    xRectMat = bicubic4x4FastUse(xOrigMat,scaleSizeX,scaleSizeY);
    yRectMat = bicubic4x4FastUse(yOrigMat,scaleSizeX,scaleSizeY);
end
xRectMat = xRectMat + dltErr;
yRectMat = yRectMat + dltErr;
xRectMatUse = xRectMat((rowNew-nr)/2+1:end-(rowNew-nr)/2,(colNew-nc)/2+1:end-(colNew-nc)/2);
yRectMatUse = yRectMat((rowNew-nr)/2+1:end-(rowNew-nr)/2,(colNew-nc)/2+1:end-(colNew-nc)/2);

if 0
    idWithin = find(pixOrigAll(:,1) >= 1 & pixOrigAll(:,1) <= nc & pixOrigAll(:,2) >= 1 & pixOrigAll(:,2) <= nr);
else
    idWithin = [1:nc*nr]';
end
[~,err] = NormalizeVector(pixOrigAll(idWithin,:) - [xRectMatUse(idWithin)+0 yRectMatUse(idWithin)+0]);
figure,subplot(1,2,1);plot(err);subplot(1,2,2);plot(sort(err));
idShow = randperm(size(pixOrigAll(idWithin,:),1));
showNum = 10000;
figure,imshow(zeros(nr,nc));hold on;plot(pixOrigAll(idWithin(idShow(1:showNum)),1),pixOrigAll(idWithin(idShow(1:showNum)),2),'or');plot(xRectMatUse(idWithin(idShow(1:showNum)))+dltErr,yRectMatUse(idWithin(idShow(1:showNum)))+dltErr,'.g')
dx=pixOrigAll(idWithin(idShow(1:showNum)),1)-xRectMatUse(idWithin(idShow(1:showNum)));
dy=pixOrigAll(idWithin(idShow(1:showNum)),2)-yRectMatUse(idWithin(idShow(1:showNum)));
figure,Q=quiver(pixOrigAll(idWithin(idShow(1:showNum)),1)+0,pixOrigAll(idWithin(idShow(1:showNum)),2)+0,dx,dy,1);

% kx= 10;
% ky = 10;%


PixRectTmp = [xuuAllUse(:) yuuAllUse(:)];
imgL1 = uint8(zeros(nr,nc,3));
imgLR = imgL(:,:,1);
imgLG = imgL(:,:,2);
imgLB = imgL(:,:,3);
imgL1R = zeros(nr,nc);
imgL1G = zeros(nr,nc);
imgL1B = zeros(nr,nc);
PixNew3_ = round([xRectMatUse(:) yRectMatUse(:)]);
idIn = PixNew3_(:,1) >= 1 & PixNew3_(:,1) <= nc & PixNew3_(:,2) >= 1 & PixNew3_(:,2) <= nr;
%     idDraw_ = idDraw(idIn);
PixNew3_ = PixNew3_(idIn,:);
PixRectTmp_ = PixRectTmp(idIn,:);
ind1 = sub2ind(size(imgL1),PixNew3_(:,2),PixNew3_(:,1));
ind2 = sub2ind(size(imgL1),PixRectTmp_(:,2),PixRectTmp_(:,1));
imgL1R(ind2) = imgLR(ind1);
imgL1G(ind2) = imgLG(ind1);
imgL1B(ind2) = imgLB(ind1);
imgL1(:,:,1) = uint8(imgL1R);
imgL1(:,:,2) = uint8(imgL1G);
imgL1(:,:,3) = uint8(imgL1B);




%%
if reverseMapping == 0
    pixRect = Orig2Rect(pixOrigAll, Kinit, KK_new, R,k);
else
    pixRect = remapRect(pixOrigAll', KK_new,Kinit,k,R);
end

[~, reMappingErr] = NormalizeVector([xuuAllUse(:) yuuAllUse(:)]-pixRect);

vldMask = ones(nr,nc);
vldMask_in = ones(nr,nc);
windowSizeX = 21; 5; 3; 15; 21;15;
windowSizeY = 21; 7; 7; 11;7;
bufferLen = 10;



[offsetX,offsetY] = meshgrid(-(windowSizeX-1)/2:(windowSizeX-1)/2,-(windowSizeY-1)/2:(windowSizeY-1)/2);
[offsetX,offsetY] = meshgrid(0:(windowSizeX-1),0:(windowSizeY-1));
offsetXY = [offsetX(:) offsetY(:)];

pixRectAll1 = Orig2Rect([xuuAllUse(:) yuuAllUse(:)], Kinit, KK_new, R,k);
[~, RectOrigErr1] = NormalizeVector(pixRectAll1 - [xuuAllUse(:) yuuAllUse(:)]);
[~, RectOrigErrY1] = NormalizeVector(pixRectAll1(:,2) - [yuuAllUse(:)]);

pixRectAll2 = remapRect([xuuAllUse(:) yuuAllUse(:)]', KK_new,Kinit,  k,R);
[~, RectOrigErr2] = NormalizeVector(pixRectAll2 - [xuuAllUse(:) yuuAllUse(:)]);
[~, RectOrigErrY2] = NormalizeVector(pixRectAll2(:,2) - [yuuAllUse(:)]);
if 0
    figure,plot(RectOrigErr1-RectOrigErr2)
    figure,plot(RectOrigErrY1-RectOrigErrY2)
end

xNum = 300;
SearchTimes = [];
SearchY = [];
Err_table = [];
PixRectStack = [];
PixRectRStack = [];
Pix2Stack = [];
Pix2StackIn = [];
step = 1; 3;
yStep = 2;1;2;20; 3;15;3;10; 5; 10;
splitStep = 2; 3; 5;3; 5;
flag = false;
ySampled = [1:yStep:nr]';
if ySampled(end) ~= nr
    ySampled = [ySampled;nr];
end
xAxis = [-100:nc+100];

figure(111),imshow(zeros(nr,nc));hold on;
plotRow1 = 0;
judge = 1;
ErrDir = []; ErrWrong = [];
vldMsk = zeros(nr,nc);
pixStack = [];
figure(444),imshow(zeros(nr,nc));hold on;
figure(333),imshow(ones(nr,nc));hold on;
for i = 1 : length(ySampled) - 1
    if i == 539
%         draw = 1;
    end
    pixRect_1 = Orig2Rect([xuuAllUse(ySampled(i),:); yuuAllUse(ySampled(i),:)]', Kinit, KK_new, R,k);
    pixRect_2 = Orig2Rect([xuuAllUse(ySampled(i+1),:); yuuAllUse(ySampled(i+1),:)]', Kinit, KK_new, R,k);
    %     [pixRect_1,ind1,ind11] = intersect(ceil(pixRect_1),ceil(pixRect_1),'rows');
    if 1
        pixRect_1 = ceil(pixRect_1);
    else
        pixRect_1(:,1) = ceil(pixRect_1(:,1));
        pixRect_1(:,2) = round(pixRect_1(:,2));
    end
    
    [aa1,bb1,cc1] = unique(pixRect_1,'rows');
    cc1 = sort(cc1,'ascend');
    ccd1 = diff(cc1);
    [indexCell1, ~] = splitIndex2(find(ccd1 == 1));
    iccd1 = cell2mat(indexCell1')';
    pixOrig_1 = [xuuAllUse(ySampled(i),:); yuuAllUse(ySampled(i),:)]';
    pixOrig_1 = pixOrig_1(iccd1,:);
    pixRect_1 = pixRect_1(iccd1,:);
    [aa11,bb11,cc11] = unique(pixRect_1(:,1),'rows');
    cc11 = sort(cc11,'ascend');
    ccd11 = diff(cc11);
    [indexCell11, ~] = splitIndex2(find(ccd11 == 1));
    iccd11 = cell2mat(indexCell11')';
    pixOrig_1 = pixOrig_1(iccd11,:);
    pixRect_1 = pixRect_1(iccd11,:);
    
    
    
    pixRect_2(:,1) = ceil(pixRect_2(:,1));
    if 1
        pixRect_2(:,2) = floor(pixRect_2(:,2));
    else
        pixRect_2(:,2) = round(pixRect_2(:,2));
    end
    [aa2,bb2,cc2] = unique(pixRect_2,'rows');
    cc2 = sort(cc2,'ascend');
    ccd2 = diff(cc2);
    [indexCell2, ~] = splitIndex2(find(ccd2 == 1));
    iccd2 = cell2mat(indexCell2')';
    pixOrig_2 = [xuuAllUse(ySampled(i+1),:); yuuAllUse(ySampled(i+1),:)]';
    pixOrig_2 = pixOrig_2(iccd2,:);
    pixRect_2 = pixRect_2(iccd2,:);
    [aa22,bb22,cc22] = unique(pixRect_2(:,1),'rows');
    cc22 = sort(cc22,'ascend');
    ccd22 = diff(cc22);
    [indexCell22, ~] = splitIndex2(find(ccd22 == 1));
    iccd22 = cell2mat(indexCell22')';
    pixOrig_2 = pixOrig_2(iccd22,:);
    pixRect_2 = pixRect_2(iccd22,:);
    
    flag1 = pixRect_1(:,1) >=1 & pixRect_1(:,1)<=nc & pixRect_1(:,2) >=1 & pixRect_1(:,2) <= nr;
    pixRect_1 = pixRect_1(flag1,:);
    pixOrig_1 = pixOrig_1(flag1,:);
    flag2 = pixRect_2(:,1) >=1 & pixRect_2(:,1)<=nc & pixRect_2(:,2) >=1 & pixRect_2(:,2) <= nr;
    pixRect_2 = pixRect_2(flag2,:);
    pixOrig_2 = pixOrig_2(flag2,:);
    %     [~,id1,~] = unique(pixRect_1(:,1));
    % %     pixRect_1 = pixRect_1(id1,:);
    % %     pixOrig_1 = pixOrig_1(id1,:);
    % %     [~,id2,~] = unique(pixRect_2(:,1));
    % %     pixRect_2 = pixRect_2(id2,:);
    % %     pixOrig_2 = pixOrig_2(id2,:);
    %     pixRect_ = Orig2Rect([xAxis; repmat(ySampled(i),1,length(xAxis))]', Kinit, KK_new, R,k);
    startX = min([pixRect_1(:,1); pixRect_2(:,1)]);
    endX = max([pixRect_1(:,1); pixRect_2(:,1)]);
    if min([pixRect_1(:,2); pixRect_2(:,2)]) < 5
        bound = 1;
    else
        bound = nr;
    end
    if pixRect_1(1,1) <= 2 && pixRect_1(end,1) >= nc-2 && pixRect_2(1,1) <= 2 && pixRect_2(end,1) >= nc-2
        judge = 0;
    else
        judge = 1;
    end
    minY1_1 = pixRect_1(1,2);
    if minY1_1 <= 2
        minY1_1 = 1;
    end
    maxY2_1 = pixRect_2(1,2);
    if maxY2_1 >= nr-1
        maxY2_1 = nr;
    end
    
    
    minY1_2 = pixRect_1(end,2);
    if minY1_2 <= 2
        minY1_2 = 1;
    end
    maxY2_2 = pixRect_2(end,2);
    if maxY2_2 >= nr-1
        maxY2_2 = nr;
    end
    if 1
        if pixRect_1(1,1) > pixRect_2(1,1)
            pixOrig_1 = [[repmat(pixOrig_1(1,:),length(startX:pixRect_1(1,1)-1),1)];pixOrig_1];
            pixRect_1 = [[startX:pixRect_1(1,1)-1;repmat(minY1_1,1,length(startX:pixRect_1(1,1)-1))]';pixRect_1];
            
        else
            pixOrig_2 = [[repmat(pixOrig_2(1,:),length(startX:pixRect_2(1,1)-1),1)];pixOrig_2];
            pixRect_2 = [[startX:pixRect_2(1,1)-1;repmat(maxY2_1,1,length(startX:pixRect_2(1,1)-1))]';pixRect_2];
            
        end
        
        %     if judge
        if pixRect_1(end,1) < pixRect_2(end,1)
            pixOrig_1 = [pixOrig_1;[repmat(pixOrig_1(end,:),length(pixRect_1(end,1)+1:endX),1)]];
            pixRect_1 = [pixRect_1;[pixRect_1(end,1)+1:endX; repmat(minY1_2,1,length(pixRect_1(end,1)+1:endX))]'];
            
        else
            pixOrig_2 = [pixOrig_2;[repmat(pixOrig_2(end,:),length(pixRect_2(end,1)+1:endX),1)]];
            pixRect_2 = [pixRect_2;[pixRect_2(end,1)+1:endX; repmat(maxY2_2,1,length(pixRect_2(end,1)+1:endX))]'];
            
        end
    end
    idd1 = diff(pixRect_1(:,2));
    iddd1_1 = find(idd1 > 0);
    iddd1_2 = find(idd1 < 0);
    iddd1_1 = iddd1_1 +1;
    if 0
        iddd1_2 = iddd1_2 +0;
    else
        iddd1_2 = iddd1_2 +1;
    end
    iddd1 = [iddd1_1;iddd1_2];
    if 0
        pixRect_1(iddd1_2,2) = pixRect_1(iddd1_2,2) - 1;
    end
    idd2 = diff(pixRect_2(:,2));
    %     iddd2 = find(idd2 ~= 0);iddd2 = iddd2 +1;
    iddd2_1 = find(idd2 > 0);
    iddd2_2 = find(idd2 < 0);
    iddd2_1 = iddd2_1 +1;
    if 0
        iddd2_2 = iddd2_2 +0;
    else
        iddd2_2 = iddd2_2 +1;
    end
    iddd2 = [iddd2_1;iddd2_2];
    if 0
        pixRect_2(iddd2_2,2) = pixRect_2(iddd2_2,2) - 1;
    end
    
    if draw
         figure(444),plot(pixRect_1(:,1),pixRect_1(:,2),'xr');plot(pixRect_2(:,1),pixRect_2(:,2),'.g');plot(pixRect_1(iddd1,1),pixRect_1(iddd1,2),'or');plot(pixRect_2(iddd2,1),pixRect_2(iddd2,2),'og')
    end
    splitX = [pixRect_1(iddd1,1);pixRect_2(iddd2,1)];
    splitX = unique(splitX);
    splitX = [startX;splitX;endX];
    splitX = unique(splitX);
    splitXId1 = find(ismember(pixRect_1(:,1),splitX));
    splitXId2 = find(ismember(pixRect_2(:,1),splitX));
    for j = 1 : length(splitX) -1
        
        splitXiD1_1 = find(pixRect_1(:,1) == splitX(j));
        splitXiD1_2 = find(pixRect_1(:,1) == splitX(j+1)-1);
        splitXiD2_1 = find(pixRect_2(:,1) == splitX(j));
        splitXiD2_2 = find(pixRect_2(:,1) == splitX(j+1)-1);
        if isempty(splitXiD1_1)
            grid11 = [splitX(j) pixRect_1(find(pixRect_1(:,1) == splitX(j)-1),2)];
            %             Orig11 =
        else
            grid11 = pixRect_1(splitXiD1_1,:);
            %             OrigXStart = pixOrig_1(splitXiD1_1,1);
        end
        if isempty(splitXiD1_2)
            grid12 = [splitX(j+1)-1 pixRect_1(find(pixRect_1(:,1) == splitX(j+1)-2),2)];
            origXEnd1 = [];
        else
            grid12 = pixRect_1(splitXiD1_2,:);
            origXEnd1 = pixOrig_1(splitXiD1_2+1,1);
            %             OrigXEnd = pixOrig_1(splitXiD1_2,1);
        end
        
        if isempty(splitXiD2_1)
            grid21 = [splitX(j) pixRect_2(find(pixRect_2(:,1) == splitX(j)-1),2)];
        else
            grid21 = pixRect_2(splitXiD2_1,:);
            %             Orig21 = pixRect_2(splitXiD2_1,:);
        end
        if isempty(splitXiD2_2)
            grid22 = [splitX(j+1)-1 pixRect_2(find(pixRect_2(:,1) == splitX(j+1)-2),2)];
            origXEnd2 = pixOrig_2(find(pixRect_2(:,1) == splitX(j+1)-0),1);
        else
            grid22 = pixRect_2(splitXiD2_2,:);
            origXEnd2 = pixOrig_2(splitXiD2_2+1,1);
        end
        % %        grid11 = pixRect_1(splitXId1(j),:);
        % %        grid12 = pixRect_1(splitXId1(j+1)-1,:);
        % %        grid21 = pixRect_2(splitXId2(j),:);
        % %        grid22 = pixRect_2(splitXId2(j+1)-1,:);
        gridTmp = [grid11;grid12;grid21;grid22];
        origXEnd = origXEnd2 + 1;
        
        try
            grid = [[min(grid11(1),grid21(1)) grid11(2)];[max(grid12(1),grid22(1)) grid12(2)];[min(grid11(1),grid21(1)) grid21(2)];[max(grid12(1),grid22(1)) grid22(2)]];
            
            [tmpX,tmpY] = meshgrid(min(grid(:,1)):max(grid(:,1)),min(grid(:,2)):max(grid(:,2)));
            pixStack = [pixStack; [tmpX(:) tmpY(:)]];
            
            vldInd = sub2ind([nr,nc], tmpY(:), tmpX(:));
            vldMsk(vldInd) = 1;
            
            if size(pixStack,1) >= 1969607
                asghik = 1;
            end
            
            if sum(sum(vldMsk)) ~= size(pixStack,1)
                eadvkbj = 1;
            end
            
            [pixx,pixy] = meshgrid(1:nc,ySampled(i):ySampled(i+1));
            pixOrigPool = [[pixx(:) pixy(:)];[1:origXEnd; repmat(ySampled(i+1),1,length(1:origXEnd))]'];
            pixOrigEnd = remapRect(grid(end,:)', KK_new,Kinit,k,R);
            errDir = pixOrigPool(end,:) - pixOrigEnd;
            if min(errDir) < 0
                ErrWrong = [ErrWrong; [pixOrigPool(end,:) pixOrigEnd errDir]];
                if draw
                    figure(444),plot(grid(end,1),grid(end,2),'sy');
                end
            end
            if min(errDir(2)) < 0
                asgnlv = 1;
                figure(444),plot(grid(end,1),grid(end,2),'xm');
                fjsaq = 1;
            end
            ErrDir = [ErrDir;errDir];
            
            if sum(ismember(1969607, vldInd)) >= 2
                sgkdfj =1;
            end
            if draw
                figure(333),plot(grid([1 2 4 3 1],1),grid([1 2 4 3 1],2),'r');
            end
        catch
            enkj = 1;
        end
    end
    if draw
        drawnow;
    end
    continue;
    rowSeg{i,1} = intersect(round(pixRect_),round(pixRect_),'rows');
    [~,id1,~] = unique(rowSeg{i,1}(:,1));
    rowSeg{i,1} = rowSeg{i,1}(id1,:);
    rowSeg{i,2} = rowSeg{i,1}(rowSeg{i,1}(:,1) >=1 & rowSeg{i,1}(:,1)<=nc & rowSeg{i,1}(:,2) >=1 & rowSeg{i,1}(:,2) <= nr,:);
    %     rowSeg{i,2} = rowSeg{i,1}(rowSeg{i,1}(:,1) >=1 & rowSeg{i,1}(:,1)<=nc,:);
    %     rowSeg{i,2}(rowSeg{i,2}(:,2) < 0, 2) = 1;
    %     rowSeg{i,2}(rowSeg{i,2}(:,2) > nr, 2) = nr;
    
    if ~flag
        
        
        
        cumYDiff = cumsum(abs(diff(rowSeg{i,2}(:,2))));
        splitId = find(ismember(cumYDiff,[0:splitStep:10000]));
        splitId = [1;splitId;length(cumYDiff) + 1];
        splitId = unique(splitId);
        if splitId(end)-splitId(end-1) < 3
            splitId = [splitId(1:end-2); splitId(end)];
        end
        yDiff = diff(rowSeg{i,2}(:,2));
        diffSpId = diff(splitId);
        id = find(diffSpId > 1);
        
        if i == 15
            kas = 1;
        end
        
        
        try
            idd = find(diff(diffSpId(id))<-5); idd = idd(1);
        catch
            savkbj = 1;
            idd = length(id) + 1;
        end
        id(1:idd-1) = id(1:idd-1) + 2;
        id(idd:end) = id(idd:end) + 1;
        if id(end) >= length(splitId)
            id  = id(id<length(splitId));
            %         id = [1;id;length(splitId)];
            %     else
            %         id = [1;id;length(splitId)];
        end
        id = [1;id;length(splitId)];
        id  = unique(id);
        %     [indexCellRobo, indexRobo] = splitIndex2(splitId);
        figure(111),plot(rowSeg{i,2}(:,1),rowSeg{i,2}(:,2),'.','Color',rand(1,3));
        try
            plot(rowSeg{i,2}(splitId(id),1),rowSeg{i,2}(splitId(id),2),'*g')
        catch
            asvgikj = 1;
        end
        drawnow;
        
        rowSeg{i,3} = [rowSeg{i,2}(splitId(id),1) rowSeg{i,2}(splitId(id),2)];
        rowSeg{i,4} = rowSeg{i,3};
        
        if i >= 2
            % %        Id1 = find(rowSeg{i,3}(:,1) > rowSeg{i-1,3}(1,1) & rowSeg{i,3}(:,1) < rowSeg{i-1,3}(end,1));
            Id1 = find(rowSeg{i,3}(:,1) < rowSeg{i-1,3}(1,1)-10);
            Id2 = find(rowSeg{i,3}(:,1) > rowSeg{i-1,3}(end,1)+10);
            
            if ~isempty(Id1)
                rowSeg{i,4} = [rowSeg{i,3}(1:Id1(end),:); rowSeg{i-1,4}];
            else
                rowSeg{i,4} = [rowSeg{i-1,4}];
            end
            
            if ~isempty(Id2)
                rowSeg{i,4} = [rowSeg{i,4};rowSeg{i,3}(Id2(1):end,:)];
                %        else
                %            rowSeg{i,4} = [rowSeg{i,4}(1:Id1(1)-1,:); rowSeg{i-1,4}; rowSeg{i,4}(Id1(end)+1:end,:)];
            end
            inFlag = ismember(rowSeg{i,2}(:,1), rowSeg{i-1,4}(:,1));
            inFlag__ = false(length(inFlag),1);
            
            if sum(inFlag) ~= size(rowSeg{i-1,4}(:,1),1)
                inFlag_ = ~ismember(rowSeg{i-1,4}(:,1), rowSeg{i,2}(:,1));
                inFlag__ = ismember(rowSeg{i,2}(:,1), rowSeg{i-1,4}(inFlag_,1)-1);
            end
            inFlagMerge = inFlag | inFlag__;
            segTmp = [rowSeg{i,2}(inFlagMerge,:)];
            segTmp(:,1) = rowSeg{i-1,4}(:,1);
            rowSeg{i,5} = rowSeg{i,3};
            %        if isempty(Id1)
            %           rowSeg{i,3} = [rowSeg{i,2}(inFlagMerge,:)];
            %        else
            %            rowSeg{i,3} = [rowSeg{i,3}(1:Id1(1)-1,:);rowSeg{i,2}(inFlagMerge,:);rowSeg{i,3}(Id1(end)+1:end,:)];
            %        end
            rowSeg3Tmp = rowSeg{i,3};
            if ~isempty(Id1)
                rowSeg{i,3} = [rowSeg3Tmp(1:Id1(end),:); segTmp];
            else
                rowSeg{i,3} = [segTmp];
            end
            
            if ~isempty(Id2)
                rowSeg{i,3} = [rowSeg{i,3};rowSeg3Tmp(Id2(1):end,:)];
                %        else
                %            rowSeg{i,4} = [rowSeg{i,4}(1:Id1(1)-1,:); rowSeg{i-1,4}; rowSeg{i,4}(Id1(end)+1:end,:)];
            end
            if rowSeg{i-1,3}(1,1) == 1
                iD = find(rowSeg{i,2}(:,1) == 1);
                if isempty(iD)
                    iD = find(rowSeg{i,2}(:,1) == 2);
                    rowSeg{i,3}(1,:) = [1 rowSeg{i,2}(iD,2)];
                end
            end
            if nc - rowSeg{i-1,3}(end,1) == 0
                iD = find(nc - rowSeg{i,2}(:,1) == 0);
                if isempty(iD)
                    iD = find(nc - rowSeg{i,2}(:,1) == 1);
                    rowSeg{i,3}(end,:) = [nc rowSeg{i,2}(iD,2)];
                end
            end
            
            if rowSeg{i,3}(1,1) <= 5
                iD = find(rowSeg{i,2}(:,1) <= 2);
                if ~isempty(iD)
                    rowSeg{i,3}(1,:) = [1 rowSeg{i,2}(iD(1),2)];
                end
            end
            if nc - rowSeg{i,3}(end,1) >= 0 && nc - rowSeg{i,3}(end,1) <= 5
                iD = find(nc - rowSeg{i,2}(:,1) <= 2);
                if ~isempty(iD)
                    rowSeg{i,3}(end,:) = [nc rowSeg{i,2}(iD(end),2)];
                end
            end
            
            if rowSeg{i,3}(1,1) == 1 && rowSeg{i,3}(end,1) == nc
                flag = true;
            end
            
            if plotRow1 == 0
                plotRow1 = 1;
                plot(rowSeg{1,3}(:,1),rowSeg{1,3}(:,2),'xm','LineWidth',5,'MarkerSize',5);
            end
            
            plot(rowSeg{i,4}(:,1),rowSeg{i,4}(:,2),'or');
            plot(rowSeg{i,3}(:,1),rowSeg{i,3}(:,2),'xm','LineWidth',5,'MarkerSize',5);
            
            
        end
        
    else
        if isempty(find(rowSeg{i,2}(:,1) == 1))
            %             rowSeg{i,2}(rowSeg{i,2}(:,1) == 2,1) = 1;
            rowSeg{i,2}(1,1) = 1;
        end
        if isempty(find(rowSeg{i,2}(:,1) == nc))
            %             rowSeg{i,2}(rowSeg{i,2}(:,1) == nc-1,1) = nc;
            rowSeg{i,2}(end,1) = nc;
        end
        inFlag = ismember(rowSeg{i,2}(:,1), rowSeg{i-1,3}(:,1));
        inFlag__ = false(length(inFlag),1);
        if sum(inFlag) ~= size(rowSeg{i-1,3}(:,1),1)
            inFlag_ = ~ismember(rowSeg{i-1,3}(:,1), rowSeg{i,2}(:,1));
            inFlag__ = ismember(rowSeg{i,2}(:,1), rowSeg{i-1,3}(inFlag_,1)-1);
        end
        %         FF = pdist2(rowSeg{i,2}(:,1),rowSeg{i-1,3}(:,1),'euclidean');
        
        
        inFlagMerge = inFlag | inFlag__;
        segTmp = [rowSeg{i,2}(inFlagMerge,:)];
        
        if segTmp(1,1) ~= 1
            segTmp = [rowSeg{i,2}(1,:); segTmp];
        end
        if segTmp(end,1) ~= nc
            segTmp = [segTmp;rowSeg{i,2}(end,:)];
        end
        segTmp(:,1) = rowSeg{i-1,3}(:,1);
        rowSeg{i,3} = segTmp;
        
        plot(rowSeg{i,2}(:,1),rowSeg{i,2}(:,2),'.','Color',rand(1,3));
        plot(rowSeg{i,3}(:,1),rowSeg{i,3}(:,2),'xm','LineWidth',5,'MarkerSize',5);
        
        drawnow;
        askjhb = 1;
        %        if ~isempty(Id1)
        %
        %        end
        %        if ~isempty(Id2)
        %
        %        end
        
    end
    
    if i == 1
        continue;
    end
    
    dfghkj = 1;
    
    
    if 0
        pixRect_table = [xRectMatUse(i,:); yRectMatUse(i,:)]';
        [~, err_table] = NormalizeVector(pixRect_ - pixRect_table);
        Err_table = [Err_table; err_table'];
        
        pixRect = pixRect_(1:step:end,:);
        % %     pixRect_table = [xRectMatUse(i,:); yRectMatUse(i,:)]';
        
        pixRectR = round(pixRect);
        pixRectRMat = repmat(pixRectR,1,1,windowSizeX*windowSizeY);
        offsetXYMat = repmat(offsetXY,1,1,size(pixRectR,1));
        offsetXYMat = permute(offsetXYMat,[3 2 1]);
        pixRectSample = pixRectRMat + offsetXYMat;
        pixRectSampleX = pixRectSample(:,1,:); pixRectSampleX = reshape(pixRectSampleX(:),size(pixRectR,1),[])';
        pixRectSampleY = pixRectSample(:,2,:); pixRectSampleY = reshape(pixRectSampleY(:),size(pixRectR,1),[])';
        [tmpX, tmpY] = meshgrid(min(pixRectSampleX(:)) : max(pixRectSampleX(:)), min(pixRectSampleY(:)) : max(pixRectSampleY(:)));
        %  tmpX = tmpX - min(pixRectSampleX(:)) + 1;
        %  tmpY = tmpY - min(pixRectSampleY(:)) + 1;
        tmpMsak = zeros(size(tmpX)); tmpMsak2 = zeros(size(tmpX));
        ind = sub2ind(size(tmpX), pixRectSampleY(:)-min(pixRectSampleY(:)) + 1, pixRectSampleX(:)-min(pixRectSampleX(:)) + 1);
        [commInd,id1,id2] = intersect(ind,ind);
        [~,idd1] = sort(id1,'ascend');
        id11 = id1(idd1);
        indd = ind(id11);
        id11_ovlp = setdiff(1:length(ind),id11);
        %     id11_ovlp2 = setdiff(ind,indd);
        %     id11_ovlp3 = find(~ismember(ind,indd));
        if 0
            plot(id11_ovlp-id11_ovlp2)
        end
        
        % %     tmpMsak(ind) = 1;
        % %     tmpMsak2(ind(id1)) = 1;
        
        indMat = reshape(ind,[],size(pixRectR,1)); indMat0 = indMat;
        % indMat(ind(id11_ovlp)) = nan;
        indMat((id11_ovlp)) = nan; indMat00 = indMat;
        if 0
            [sum(sum(~isnan(indMat00))) length(id1)]
        end
        
        OutOfBoundId = find(pixRectSampleX(:) < 1 | pixRectSampleX(:)>nc | pixRectSampleY(:) < 1 | pixRectSampleY(:) > nr);
        indMat(OutOfBoundId) = nan; indMat000 = indMat;
        indMatTmp = indMat;
        indMatTmp(isnan(indMatTmp)) = 0;
        idValidrowPix = find(sum(indMatTmp) > 0);
        iddd = find(~isnan(indMat));
        pix = [pixRectSampleX(iddd) pixRectSampleY(iddd)];
        vldInd = sub2ind([nr nc],pix(:,2),pix(:,1));
        tmpMask = zeros(nr,nc);
        tmpMask(vldInd) = 1;
        mergeMask = immultiply(tmpMask,vldMask);
        pixMerge = Img2Pix(mergeMask,mergeMask);
        [~,Id11,Id22] = intersect(pixMerge, pix,'rows');
        [~,Idd1] = sort(Id22,'ascend');
        Id222 = Id22(Idd1);
        idddd = iddd(Id222);
        OOBId = setdiff(iddd,idddd);
        OOBId2 = iddd(setdiff([1:length(iddd)]',Id222));
        if 0
            figure,plot([OOBId-OOBId2])
        end
        indMat(OOBId) = nan;
        iddd2 = find(~isnan(indMat));
        pix2 = [pixRectSampleX(iddd2) pixRectSampleY(iddd2)];
        if 1 % reverseMapping == 0 lookup table, switch it to approximate lookup table other than exact mapping
            pixOrig_2 = remapRect(pix2', KK_new, Kinit,k, R);
        else
            pixOrig_2 = Orig2Rect(pix2', KK_new, Kinit,k, R);
        end
        
        pixOrig_2_in = pixOrig_2(pixOrig_2(:,1) >=1 & pixOrig_2(:,1)<=nc & pixOrig_2(:,2) >=1 & pixOrig_2(:,2) <= nr,:);
        pix2_in = pix2(pixOrig_2(:,1) >=1 & pixOrig_2(:,1)<=nc & pixOrig_2(:,2) >=1 & pixOrig_2(:,2) <= nr,:);
        if 0
            figure,plot([pixOrig_2(:,2) repmat(i,size(pixOrig_2,1),1)]);
            figure,imshow(zeros(nr,nc));hold on;plot(pixOrig_2(:,1),pixOrig_2(:,2),'or');
            plot(pix2(:,1),pix2(:,2),'ob');plot([1:nc],repmat(i,nc,1),'.g');
            plot(pixOrig_2_in(:,1),pixOrig_2_in(:,2),'.y');
            legend('orig','rect','row','origIn')
        end
        try
            %         searchY(i,:) = [min(pixOrig_2_in(:,2)-i)  max(pixOrig_2_in(:,2)-i)];
            SearchY = [SearchY;[min(pixOrig_2_in(:,2)-i)  max(pixOrig_2_in(:,2)-i)]];
        catch
            continue;
        end
        vldInd2 = sub2ind([nr nc],pix2(:,2),pix2(:,1));
        vldInd2_in = sub2ind([nr nc],pix2_in(:,2),pix2_in(:,1));
        if 0
            pix2 = pix2_in;
            vldInd2 = vldInd2_in;
        end
        
        Pix2Stack = [Pix2Stack;pix2];
        Pix2StackIn = [Pix2StackIn;pix2_in];
        
        
        indMatTmp2 = indMat;
        indMatTmp2(isnan(indMatTmp2)) = 0;
        idValidrowPix2 = find(sum(indMatTmp2) > 0);
        
        SearchTimes = [SearchTimes;length(idValidrowPix2)];
        % indMatTmp = indMat;
        % indMatTmp(isnan(indMatTmp)) = 0;
        % idValidrowPix = find(sum(indMatTmp) > 0);
        % iddd = find(~isnan(indMat));
        % pix = [pixRectSampleX(iddd) pixRectSampleY(iddd)];
        % vldInd = sub2ind([nr nc],pix(:,2),pix(:,1));
        % tmpMask = zeros(nr,nc);
        % tmpMask(vldInd) = 1;
        % mergeMask = immultiply(tmpMask,vldMask);
        % pixMerge = Img2Pix(mergeMask,mergeMask);
        % [~,Id11,Id22] = intersect(pixMerge, pix,'rows');
        % [~,Idd1] = sort(Id22,'ascend');
        % Id222 = Id22(Idd1);
        % idddd = iddd(Id222);
        % OOBId = setdiff(iddd,idddd);
        % indMat(OOBId) = nan;
        
        vldMask(vldInd2) = 0;
        vldMask_in(vldInd2_in) = 0;
        
        PixRectStack = [PixRectStack; pixRect];
        PixRectRStack = [PixRectRStack; pixRectR];
        
        kgjvh = 1;
        if 0
            figure,plot(vldMask(vldInd2));  % should all be 1
        end
        
        if 0 % i >= 621 %500
            % %         figure,imshow(zeros(nr,nc));hold on;plot(pixRectSampleX(:,1),pixRectSampleY(:,1),'og');plot(pixRectSampleX(:,2),pixRectSampleY(:,2),'.b');plot(pixRectR(:,1),pixRectR(:,2),'*r');
            % %         figure,imshow(zeros(nr,nc));hold on;plot(pixRectSampleX(:),pixRectSampleY(:),'.g');plot(pixRectR(:,1),pixRectR(:,2),'.r');
            figure,subplot(1,2,1),imshow(zeros(nr,nc));hold on;plot(pixRectSampleX(~isnan(indMat000)),pixRectSampleY(~isnan(indMat000)),'.g');plot(pix(:,1),pix(:,2),'og');plot(pixRectR(:,1),pixRectR(:,2),'.r');plot(pixRectR(idValidrowPix,1),pixRectR(idValidrowPix,2),'sb');
            subplot(1,2,2),imshow(zeros(nr,nc));hold on;plot(pixRectSampleX(~isnan(indMat)),pixRectSampleY(~isnan(indMat)),'.g');plot(pix(:,1),pix(:,2),'og');plot(pixRectR(:,1),pixRectR(:,2),'.r');plot(pixRectR(idValidrowPix2,1),pixRectR(idValidrowPix2,2),'sb');
            
            figure,subplot(1,2,1);imshow(zeros(nr,nc));hold on;plot(pixRectSampleX(~isnan(indMat000(:,xNum)),xNum),pixRectSampleY(~isnan(indMat000(:,xNum)),xNum),'.g');plot(pixRectSampleX(~isnan(indMat0(:,xNum)),xNum),pixRectSampleY(~isnan(indMat0(:,xNum)),xNum),'og');plot(pixRectR(xNum,1),pixRectR(xNum,2),'*r');
            subplot(1,2,2);imshow(zeros(nr,nc));hold on;plot(pixRectSampleX(~isnan(indMat(:,xNum)),xNum),pixRectSampleY(~isnan(indMat(:,xNum)),xNum),'.g');plot(pixRectSampleX(~isnan(indMat0(:,xNum)),xNum),pixRectSampleY(~isnan(indMat0(:,xNum)),xNum),'og');plot(pixRectR(xNum,1),pixRectR(xNum,2),'*r');
            
            adbjvk = 1;
            if 0
                figure(10);clf;imshow(zeros(nr,nc));hold on;
                for v = 1 : length(idValidrowPix2)
                    xytmp = pixRectR(idValidrowPix2(v),:);
                    xWindow = pixRectSampleX(~isnan(indMat0(:,idValidrowPix2(v))),idValidrowPix2(v));
                    yWindow = pixRectSampleY(~isnan(indMat0(:,idValidrowPix2(v))),idValidrowPix2(v));
                    xWindowSearch = pixRectSampleX(~isnan(indMat(:,idValidrowPix2(v))),idValidrowPix2(v));
                    yWindowSearch = pixRectSampleY(~isnan(indMat(:,idValidrowPix2(v))),idValidrowPix2(v));
                    clr = rand(1,3);
                    figure(10),plot(xWindowSearch,yWindowSearch,'.','Color',clr);plot(xytmp(1),xytmp(2),'*','Color',clr); % plot(xWIndow,yWindow,'og');
                    drawnow;
                end
            end
        end
        
        
    end
    
    
    
    saviu = 1;
    
    
    
    
    % %     for j = 1 : nc
    % %
    % %
    % %
    % %     end
    
end

figure(222),imshow(zeros(nr,nc));hold on;
for j = 2 : size(rowSeg,1)
    rowseg = rowSeg{j,3};
    rowsegup = rowSeg{j-1,3};
    for jj = 1 : size(rowseg,1) - 1
        cur = rowseg(jj,:);
        nex = rowseg(jj+1,:);
        nexup = rowsegup(rowsegup(:,1) == nex(1),:);
        curup = rowsegup(rowsegup(:,1) == cur(1),:);
        if isempty(nexup)
            nexup = [nex(1) 1];
        end
        if isempty(curup)
            curup = [cur(1) 1];
        end
        if curup(2) < nexup(2)
            upY = curup(2);
        else
            upY = nexup(2);
        end
        
        if cur(2) < nex(2)
            grid = [[cur(1) upY];[nex(1) upY];[cur ];[nex(1) cur(2)]];
            figure(111),plot(grid(:,1),grid(:,2),'oy','LineWidth',5,'MarkerSize',5);
            figure(222),plot(grid([1 2 4 3 1],1),grid([1 2 4 3 1],2),'r');
            
            sagihu = 1;
        else
            grid = [[cur(1) upY];[nex(1) upY];[cur(1) nex(2)];[nex]];
            figure(111),plot(grid(:,1),grid(:,2),'oy','LineWidth',5,'MarkerSize',5);
            figure(222),plot(grid([1 2 4 3 1],1),grid([1 2 4 3 1],2),'r');
            
            asvkbj = 1;
        end
    end
    drawnow;
end





figure,imshow(~vldMask);
figure,imshow(~vldMask_in);
coverage = [sum(sum(vldMask == 0))-size(intersect(Pix2Stack,Pix2Stack,'rows'),1) sum(sum(vldMask_in == 0))-size(intersect(Pix2StackIn,Pix2StackIn,'rows'),1)];

figure,plot(SearchY);
searchtime = sum(SearchTimes);

if 0
    figure,plot(PixRectRStack(:,1),PixRectRStack(:,2),'.r');axis equal;
end

PixRectRStackX = reshape(PixRectRStack(:,1),size(pixRectR,1),nr)';
diffX = diff(PixRectRStackX')';
PixRectRStackY = reshape(PixRectRStack(:,2),size(pixRectR,1),nr)';
diffY = diff(PixRectRStackY);
figure,subplot(1,2,1);imshow(diffX,[]);title('diff x');
subplot(1,2,2);imshow(diffY,[]);title('diff y');



figure,plot(Err_table(:)-err);

return;

xRectMat_4 = bicubic4x4FastUse(xOrigMat,scaleSizeX,scaleSizeY);
xRectMat_8 = bicubic8x8FastUse(xOrigMat,scaleSizeX,scaleSizeY);
xRectMat_6 = bicubic6x6FastUse(xOrigMat,scaleSizeX,scaleSizeY);

xRectMat_8 = bicubic8(xOrigMat,scaleSizeX,scaleSizeY);





xRectMat_ = bicubic(xOrigMat,scaleSizeX,scaleSizeY);
yRectMat_ = bicubic(yOrigMat,scaleSizeX,scaleSizeY);


[xuu,yuu] = meshgrid(1:skipX:nc, 1:skipY:nr);


pixDist1 = remapRect([xuu(:) yuu(:)]', Kinit, Kinit,k, eye(3));
pixOrig = remapRect([xuu(:) yuu(:)]', KK_new, Kinit,k, eye(3));


pixOrig = remapRect([xuu(:) yuu(:)]', KK_new, Kinit,k, R);


xMat = reshape(pixOrig(:,1),size(xuu,1),size(xuu,2));
yMat = reshape(pixOrig(:,2),size(xuu,1),size(xuu,2));



dx=pixOrig(:,1)-xuu(:);dx_ = reshape(dx,size(xuu,1),size(xuu,2))';dx_ = dx_(:)';
dy=pixOrig(:,2)-yuu(:);dy_ = reshape(dy,size(xuu,1),size(xuu,2))';dy_ = dy_(:)';
% Q=quiver(px+1,py+1,dx,dy);
% figure,
figure(1),clf; imshow(zeros(nr,nc));hold on;plot(pixOrig(:,1),pixOrig(:,2),'xg');hold on;plot(xuu(:),yuu(:),'.r'); legend('orig','rect');
Q=quiver(xuu(:)+0,yuu(:)+0,dx,dy,0.3);
hold on;
plot(Kinit(1,3),Kinit(2,3),'o');
% plot((nc-1)/2+1,(nr-1)/2+1,'x');
plot(KK_new(1,3),KK_new(2,3),'x');
dr=reshape(sqrt((dx_.*dx_)+(dy_.*dy_)),size(xuu,2),size(xuu,1))';
[C,h]=contour(xuu,yuu,dr,'k');
clabel(C,h);

figure(2),clf;imshow(zeros(nr,nc));hold on;
for i = 1 : size(xMat,1)
    p1 = polyfit(xMat(i,:),yMat(i,:),2);
    if p1(1) > 0
        if min(yMat(i,:)) > marg && min(yMat(i,:)) < nr - marg
            yHeight(i,:) = repmat(min(yMat(i,:)),1,size(xMat,2));
        elseif min(yMat(i,:)) <=marg
            yHeight(i,:) = repmat(0,1,size(xMat,2));
        else
            yHeight(i,:) = repmat(nr,1,size(xMat,2));
        end
    else
        if max(yMat(i,:)) > marg && max(yMat(i,:)) < nr - marg
            yHeight(i,:) = repmat(max(yMat(i,:)),1,size(xMat,2));
        elseif max(yMat(i,:)) < marg
            yHeight(i,:) = repmat(0,1,size(xMat,2));
        else
            yHeight(i,:) = repmat(nr,1,size(xMat,2));
        end
    end
    [~,~,VV] = svd([xMat(i,:);yHeight(i,:);ones(1,size(xMat,2))]');
    initLine = VV(:,end)';
    if initLine(end) ~= 0
        linek = [initLine(1)/initLine(3) initLine(2)/initLine(3) 1];
    else
        linek = initLine;
    end
    LineHori(i,:) = linek;
    if 1
        points2 = lineToBorderPoints(linek, size(I));
        figure(2),line(points2(:, [1,3])', points2(:, [2,4])','Color',[0 1 0]);
        
        figure(1),plot(xMat(i,:),yMat(i,:),'-b');
    end
    %     figure(2),hold on;plot(xMat(i,:),yHeight(i,:),'-c');
end
for i = 1 : size(xMat,2)
    p2 = polyfit(yMat(:,i),xMat(:,i),2);
    if p2(1) > 0
        if min(xMat(:,i)) > marg && min(xMat(:,i)) < nc -marg
            xWidth(:,i) = repmat(min(xMat(:,i)),size(xMat,1),1);
        elseif min(xMat(:,i)) <= marg
            xWidth(:,i) = repmat(0,size(xMat,1),1);
        else
            xWidth(:,i) = repmat(nc,size(xMat,1),1);
        end
    else
        if max(xMat(:,i)) > marg && max(xMat(:,i)) < nc-marg
            xWidth(:,i) = repmat(max(xMat(:,i)),size(xMat,1),1);
        elseif max(xMat(:,i)) <= marg
            xWidth(:,i) = repmat(0,size(xMat,1),1);
        else
            xWidth(:,i) = repmat(nc,size(xMat,1),1);
        end
    end
    
    [~,~,VV] = svd([xWidth(:,i)';yMat(:,i)';ones(1,size(xMat,1))]');
    initLine = VV(:,end)';
    if initLine(end) ~= 0
        linek = [initLine(1)/initLine(3) initLine(2)/initLine(3) 1];
    else
        linek = initLine;
    end
    LineVerti(i,:) = linek;
    if 1
        points2 = lineToBorderPoints(linek, size(I));
        figure(2),line(points2(:, [1,3])', points2(:, [2,4])','Color',[0 1 0]);
        
        
        figure(1),plot(xMat(:,i),yMat(:,i),'-b');
    end
    %     figure(2),hold on;plot(xWidth(:,i),yMat(:,i),'-c');
end



% [C,h]=contour(xuu,yuu,dr,'k');
% clabel(C,h);
% % % figure,quiver(xuu,yuu,reshape(pixDist(:,1),size(xuu,1),size(xuu,2))-xuu, reshape(pixDist(:,2),size(xuu,1),size(xuu,2))-yuu,'r');
pixDist2 = remapRect(pixOrig', Kinit, Kinit,k, eye(3));
% % % % % % figure,imshow(zeros(nr,nc));hold on;plot(pixOrig(:,1),pixOrig(:,2),'.g');hold on;plot(xuu(:),yuu(:),'.r');legend('orig','rect');
% % % % % % figure,imshow(zeros(nr,nc));hold on;plot(pixDist2(:,1),pixDist2(:,2),'.g');hold on;plot(pixOrig(:,1),pixOrig(:,2),'.r');legend('orig','rect');


xPoint = []; yPoint = [];
for i = 1 : size(LineHori,1)
    LineHoriTmp = repmat(LineHori(i,:),size(LineVerti,1),1);
    rowPoint = cross(LineVerti,LineHoriTmp);
    rowPoint = pflat(rowPoint');
    rowPoint = rowPoint(1:2,:)';
    if  1 %mean(rowPoint(:,2)) > 10 && mean(rowPoint(:,2)) < nr-10
        rowPoint = rowPoint(rowPoint(:,1) >= 0-0.0001 & rowPoint(:,1) <= nc + 0.0001,:);
        xPoint = [xPoint;rowPoint(:,1)'];
        yPoint = [yPoint;rowPoint(:,2)'];
        % %         figure(2),plot(rowPoint(:,1),rowPoint(:,2),'or','LineWidth',3);
    else
    end
end
xPoint(:,1) = 0;
xPoint(:,end) = nc;
xPoint = round(xPoint);
yPoint(1,:) = 0;
yPoint(end,:) = nr;
yPoint = round(yPoint);
id1 = find(yPoint(:,1) > 0 & yPoint(:,1) < nr);
yPoint = yPoint(id1(1)-1:id1(end)+1,:);
xPoint = xPoint(id1(1)-1:id1(end)+1,:);
id2 = find(xPoint(1,:) > 0 & xPoint(1,:) < nc);
xPoint = xPoint(:,id2(1)-1:id2(end)+1);
yPoint = yPoint(:,id2(1)-1:id2(end)+1);

figure(2),plot(xPoint(:),yPoint(:),'or','LineWidth',3);

pixOrigNew = remapRect([round(xPoint(:)) round(yPoint(:))]', KK_new, Kinit,k, R);
figure,imshow(zeros(nr,nc));hold on;plot(round(xPoint(:)),round(yPoint(:)),'.r');plot(pixOrigNew(:,1),pixOrigNew(:,2),'.g');legend('rect','orig');
xPointOrig = reshape(pixOrigNew(:,1),size(xPoint));
yPointOrig = reshape(pixOrigNew(:,2),size(xPoint));



for i = 1 : size(xPoint,1)
    xPointTmp = xPoint(i,:);
    xPointOrigTmp = xPointOrig(i,:);
    y_ = yPoint(i,:);
    
    params = splineParams(xPointTmp', xPointOrigTmp');
    Params1{i,1} = params;
    [Y, xx] = spline_(xPointTmp', params);
    if 1
        v = splineval(xPointTmp, params, xx);
    else
        coeff = params(:,4:-1:1);
        [Y, xx] = spline_(xPointTmp', params);
        [~,index] = histc(xx',[-inf,xPointTmp(2:end-1),inf]);
        xxNew = xx-xPointTmp(index)';
        
        v = coeff(index,1);
        for ii=2:4
            v = xxNew(:).*v + coeff(index,ii);
        end
        v = reshape(v,[1,length(xx)]);
    end
    x__ = xx; %xPointTmp(1):xPointTmp(end)-1;
    y__ = repmat(y_(1),length(x__),1);
    pixOrigNew__ = remapRect([x__ y__]', KK_new, Kinit,k, R);
    figure(50),clf,plot(xPointTmp', xPointOrigTmp','-x');hold on;plot(xx,Y,'-');plot(x__,pixOrigNew__(:,1));legend('control point','fitted','golden')
    
    
end

for i = 1 : size(xPoint,2)
    yPointTmp = yPoint(:,i);
    xPointTmp_ = xPoint(:,i);
    pixOrigTmp_ = remapRect([xPointTmp_ yPointTmp]', KK_new, Kinit,k, R);
    x_ = xPoint(:,i);
    
    
    yPointOrigTmp = yPointOrig(:,i);
    params = splineParams(yPointTmp, yPointOrigTmp);
    [Y, xx] = spline_(yPointTmp, params);
    v = splineval(yPointTmp', params, xx);
    Params2{i,1} = params;
    %     x__ = xx; %xPointTmp(1):xPointTmp(end)-1;
    y__ = xx;
    x__ = repmat(x_(1),length(y__),1);
    pixOrigNew__ = remapRect([x__ y__]', KK_new, Kinit,k, R);
    
    figure(51),clf;plot(yPointTmp', yPointOrigTmp','-x');hold on;plot(xx,Y,'-');plot(y__,pixOrigNew__(:,2));legend('control point','fitted','golden')
    
    
    % %     figure,plot(yPointTmp, yPointOrigTmp,'-x');hold on; plot(xx,Y,'-');
    
    
end

% [xuuAll,yuuAll] = meshgrid(1:1:nc, 1:1:nr);
% % xuuAll = xPoint;
% % yuuAll = yPoint;

pixOrigAll = remapRect([xuuAll(:) yuuAll(:)]', KK_new, Kinit,k, R);


PixOrigTmp = [];PixNew3 = [];PixRectTmp = [];

for i = 1 : size(xPoint,1) - 1
    
    xTmp = xuuAll(yuuAll(:,1) >= yPoint(i,1) & yuuAll(:,1) < yPoint(i + 1,1),:);
    yTmp = yuuAll(yuuAll(:,1) >= yPoint(i,1) & yuuAll(:,1) < yPoint(i + 1,1),:);
    for j = 1 : size(xPoint,2)-1
        xxTmp = xTmp(:,xTmp(1,:) >= xPoint(1,j) & xTmp(1,:) < xPoint(1,j+1));
        yyTmp = yTmp(:,xTmp(1,:) >= xPoint(1,j) & xTmp(1,:) < xPoint(1,j+1));
        
        
        pixOrigTmp = remapRect([xxTmp(:) yyTmp(:)]', KK_new, Kinit,k, R);
        pixOrigSplineX1 = splineval(xPointTmp, Params1{i}, xxTmp(1,:)');
        pixOrigSplineX2 = splineval(xPointTmp, Params1{i+1}, xxTmp(1,:)');
        ratioX = (xxTmp - xPoint(1,j))./(xPoint(1,j+1) - xPoint(1,j));
        ratioX = ratioX(1,:)';
        
        xxyyTmp = [xxTmp(:) yyTmp(:)];
        
        try
            pixOrigSplineY1 = splineval(yPointTmp', Params2{j}, yyTmp(:,1));
        catch
            asvlk = 1;
        end
        pixOrigSplineY2 = splineval(yPointTmp', Params2{j+1}, yyTmp(:,1));
        ratioY = (yyTmp - yPoint(i,1))./(yPoint(i+1,1) - yPoint(i,1));
        ratioY = ratioY(:,1);
        if 0
            pixOrigSplineX = (1 - ratioY).*pixOrigSplineX1 + ratioY.*pixOrigSplineX2;
            pixOrigSplineY = (1 - ratioX).*pixOrigSplineY1 + ratioX.*pixOrigSplineY2;
        end
        pixNew = [];pixNew2 = [];
        
        
        [pixOrigSplineX1_1, index_x11] = splineval(xPointTmp, Params1{i}, xxyyTmp(:,1));
        [pixOrigSplineX2_1, index_x22]= splineval(xPointTmp, Params1{i+1}, xxyyTmp(:,1));
        [pixOrigSplineY1_1, index_y11] = splineval(yPointTmp', Params2{j}, xxyyTmp(:,2));
        [pixOrigSplineY2_1, index_y22] = splineval(yPointTmp', Params2{j+1}, xxyyTmp(:,2));
        ratioXX_ = (xxyyTmp(:,1) - xPointTmp(index_x11))./(xPointTmp(index_x11+1)-xPointTmp(index_x11));
        ratioYY_ = (xxyyTmp(:,2) - yPointTmp(index_y11))./(yPointTmp(index_y11+1)-yPointTmp(index_y11));
        
        xInterp_ = (1 - ratioYY_).*pixOrigSplineX1_1 + ratioYY_.*pixOrigSplineX2_1;
        yInterp_ = (1 - ratioXX_).*pixOrigSplineY1_1 + ratioXX_.*pixOrigSplineY2_1;
        pixNew3 = [xInterp_ yInterp_];
        
        
        if 0
            for kk = 1 : size(xxyyTmp,1)
                xxyyTmp_ = xxyyTmp(kk,:);
                [pixOrigSplineX1_, index_x1] = splineval(xPointTmp, Params1{i}, xxyyTmp_(1)');
                [pixOrigSplineX2_, index_x2]= splineval(xPointTmp, Params1{i+1}, xxyyTmp_(1)');
                [pixOrigSplineY1_, index_y1] = splineval(yPointTmp', Params2{j}, xxyyTmp_(2));
                [pixOrigSplineY2_, index_y2] = splineval(yPointTmp', Params2{j+1}, xxyyTmp_(2));
                pt1 = [pixOrigSplineX1_ pixOrigSplineY1_];
                pt2 = [pixOrigSplineX1_ pixOrigSplineY2_];
                pt3 = [pixOrigSplineX2_ pixOrigSplineY1_];
                pt4 = [pixOrigSplineX2_ pixOrigSplineY2_];
                ratio1 = [];
                ratioXX = (xxyyTmp_(1) - xPointTmp(index_x1))./(xPointTmp(index_x1+1)-xPointTmp(index_x1));
                ratioYY = (xxyyTmp_(2) - yPointTmp(index_y1))./(yPointTmp(index_y1+1)-yPointTmp(index_y1));
                
                xInterp = (1 - ratioYY).*pixOrigSplineX1_ + ratioYY.*pixOrigSplineX2_;
                yInterp = (1 - ratioXX).*pixOrigSplineY1_ + ratioXX.*pixOrigSplineY2_;
                
                XX = (xxyyTmp_(1)-xPointTmp(index_x1))/(xPointTmp(index_x1+1)-xPointTmp(index_x1));
                YY = (xxyyTmp_(2)-yPointTmp(index_y1))/(yPointTmp(index_y1+1)-yPointTmp(index_y1));
                XNew = [1-XX XX]*[pt1(1) pt2(1);pt3(1) pt4(1)]*[1-YY;YY];
                YNew = [1-XX XX]*[pt1(2) pt2(2);pt3(2) pt4(2)]*[1-YY;YY];
                
                pixNew(kk,:) = [xInterp yInterp];  %[XNew YNew];
                pixNew2(kk,:) = [XNew YNew];
                %            plot(xInterp,yInterp,'.g');
                %            drawnow;
            end
        end
        %         figure(100),clf;imshow(zeros(1080,1920));hold on;plot(pixOrigTmp(:,1),pixOrigTmp(:,2),'or'); plot(pixNew3(:,1),pixNew3(:,2),'.g');
        %         drawnow;
        %         figure(101),plot(pixOrigTmp(:,1)-pixNew3(:,1),pixOrigTmp(:,2)-pixNew3(:,2),'+r');drawnow
        %         pixOrigSpline = [pixOrigSplineX,pixOrigSplineY];
        %         [pixOrigSplineGridX, pixOrigSplineGridY] = meshgrid(pixOrigSplineX,pixOrigSplineY);
        
        PixOrigTmp = [PixOrigTmp;pixOrigTmp];
        PixNew3 = [PixNew3;pixNew3];
        PixRectTmp = [PixRectTmp; [xxTmp(:) yyTmp(:)]];
        
        if 0
            
            aa = intersect(pixOrigTmp(:,1),pixOrigTmp(:,1));
            figure,imshow(zeros(1080,1920));hold on;plot(aa,repmat(1,length(aa),1),'or');plot(pixOrigSplineX,repmat(1,length(pixOrigSplineX),1),'.g')
            bb = intersect(pixOrigTmp(:,2),pixOrigTmp(:,2));
            figure,imshow(zeros(1080,1920));hold on;plot(repmat(1,length(bb),1),bb,'or');plot(repmat(1,length(pixOrigSplineY),1),pixOrigSplineY,'.g')
            figure,imshow(zeros(1080,1920));hold on;plot(pixOrigTmp(:,1),pixOrigTmp(:,2),'or');plot(pixOrigSplineGridX(:),pixOrigSplineGridY(:),'.g');
        end
        
    end
end
idShow = randperm(size(PixNew3,1));
showNum = 10000;
figure(100),clf;imshow(zeros(1080,1920));hold on;plot(PixOrigTmp(idShow(1:showNum),1),PixOrigTmp(idShow(1:showNum),2),'or'); plot(PixNew3(idShow(1:showNum),1),PixNew3(idShow(1:showNum),2),'.g');
figure(101),plot(PixOrigTmp(idShow(1:showNum),1)-PixNew3(idShow(1:showNum),1),PixOrigTmp(idShow(1:showNum),2)-PixNew3(idShow(1:showNum),2),'+r');
[~, projErr] = NormalizeVector(PixOrigTmp-PixNew3);
idErr = find(projErr > 0.25);
figure(2),plot(PixOrigTmp(idErr,1),PixOrigTmp(idErr,2),'or'); plot(PixNew3(idErr,1),PixNew3(idErr,2),'.g');
%     [~,~,idDraw] = intersect(PixOrigTmp,pixOrigAll,'rows');
% %     imgL = rgb2gray(imread('D:\bk_20180627_\nextvpu\algorithm\output\calibration\cbccalib\stereo_pairs\iEye\zed2\imgL__left0001.png'));
imgL1 = uint8(zeros(nr,nc,3));
imgLR = imgL(:,:,1);
imgLG = imgL(:,:,2);
imgLB = imgL(:,:,3);
imgL1R = zeros(nr,nc);
imgL1G = zeros(nr,nc);
imgL1B = zeros(nr,nc);
PixNew3_ = round(PixNew3);
idIn = PixNew3_(:,1) >= 1 & PixNew3_(:,1) <= nc & PixNew3_(:,2) >= 1 & PixNew3_(:,2) <= nr;
%     idDraw_ = idDraw(idIn);
PixNew3_ = PixNew3_(idIn,:);
PixRectTmp_ = PixRectTmp(idIn,:);
ind1 = sub2ind(size(imgL1),PixNew3_(:,2),PixNew3_(:,1));
ind2 = sub2ind(size(imgL1),PixRectTmp_(:,2),PixRectTmp_(:,1));
imgL1R(ind2) = imgLR(ind1);
imgL1G(ind2) = imgLG(ind1);
imgL1B(ind2) = imgLB(ind1);
imgL1(:,:,1) = uint8(imgL1R);
imgL1(:,:,2) = uint8(imgL1G);
imgL1(:,:,3) = uint8(imgL1B);
return;


% Convert in indices:

fact = 3;

[XX,YY]= meshgrid(1:nc,1:nr);
[XXi,YYi]= meshgrid(1:1/fact:nc,1:1/fact:nr);

%tic;
Iinterp = interp2(XX,YY,I,XXi,YYi);
%toc

[nri,nci] = size(Iinterp);


ind_col = round(fact*(f(1)*xd(1,:)+c(1)))+1;
ind_row = round(fact*(f(2)*xd(2,:)+c(2)))+1;

good_points = find((ind_col >=1)&(ind_col<=nci)&(ind_row >=1)& (ind_row <=nri));
end

function pixNew = plotUndistortion(nc, nr, Kinit, kk_new,kc,R,skipNum)


% [mx,my] = meshgrid(1:nc/skipNum:(nc-0),1:nr/skipNum:(nr-0));
[mx,my] = meshgrid(0:nc/skipNum:(nc-0),0:nr/skipNum:(nr-0));
[nnx,nny]=size(mx);
px=reshape(mx',nnx*nny,1);
py=reshape(my',nnx*nny,1);
% % kk_new=[fc(1) alpha_c*fc(1) cc(1);0 fc(2) cc(2);0 0 1];
rays=inv(kk_new)*[px';py';ones(1,length(px))];
rays = R'*rays;


x=[rays(1,:)./rays(3,:);rays(2,:)./rays(3,:)];


title2=strcat('Complete Distortion Model');

% % fh1 = 2;

%if ishandle(fh1),
%    close(fh1);
%end;
% % % % % % figure; clf;
xd=apply_distortion(x,kc);
px2=Kinit(1,1)*xd(1,:)+Kinit(1,3);
py2=Kinit(2,2)*xd(2,:)+Kinit(2,3);

idIn = find(px2>=0 & px2<=nc & py2 >=0 & py2 <= nr);
idIn = [1:size(px2,2)]';

pixNew = [px2 py2];
dx=px2(idIn)'-px(idIn);
dy=py2(idIn)'-py(idIn);
% Q=quiver(px+1,py+1,dx,dy);
Q=quiver(px(idIn)+0,py(idIn)+0,dx,dy);
hold on;
% % plot(px(idIn),py(idIn),'.r');
plot(Kinit(1,3),Kinit(2,3),'o');
plot((nc-1)/2+1,(nr-1)/2+1,'x');
dr=reshape(sqrt((dx.*dx)+(dy.*dy)),nny,nnx)';
[C,h]=contour(mx,my,dr,'k');
clabel(C,h);
Mean=mean(mean(dr));
Max=max(max(dr));
title(title2);
axis equal;
end
function pixDist = remapRect(pixRect, KRect, KUndist,distCoeff, RL)

alpha = 0;

rays = inv(KRect)*pextend(pixRect);


% Rotation: (or affine transformation):

rays2 = RL'*rays;

x = [rays2(1,:)./rays2(3,:);rays2(2,:)./rays2(3,:)];


% Add distortion:
xd = apply_distortion(x,distCoeff);


% Reconvert in pixels:

px2_ = KUndist(1,1)*(xd(1,:) + alpha*xd(2,:)) + KUndist(1,3);
py2_ = KUndist(2,2)*xd(2,:) + KUndist(2,3);
pixDist = [px2_;py2_]';

end


function [v, indexx] = splineval(xPointTmp, params, xx)
coeff = params(:,4:-1:1);
% %     [Y, xx] = spline_(xPointTmp', params);
[~,index] = histc(xx',[-inf,xPointTmp(2:end-1),inf]);
xxNew = xx-xPointTmp(index)';

v = coeff(index,1);

indexx = unique(index);
for ii=2:4
    v = xxNew(:).*v + coeff(index,ii);
end
v = reshape(v,[1,length(xx)])';
end

function g1 = bicubic(f,kx,ky)
aa = [ky,0,0;0,kx,0;0.5*(1-ky),0.5*(1-kx),1];
ff = f;
[m,n]=size(f);
a=f(1,:);
c=f(m,:);             %将待插值图像矩阵前后各扩展两行两列,共扩展四行四列，当插值点为边沿点时候，确保周围有16个点
b=[f(1,1),f(1,1),f(:,1)',f(m,1),f(m,1)];
d=[f(1,n),f(1,n),f(:,n)',f(m,n),f(m,n)];
a1=[a;a;f;c;c];
b1=[b;b;a1';d;d];
f=b1';
f1=double(f);
AMat = [];
BMat = [];
for i=1:ky*m                 %利用双三次插值公式对新图象所有像素赋值
    if 1
        if 1
            temp = [i 1 1]*inv(aa);
            u = temp(1)-floor(temp(1));
            i1 = floor(temp(1)) + 2;
        else
            temp = [i 1]*inv(aa(1:2,1:2));
            u = temp(1)-floor(temp(1));
            i1 = floor(temp(1)) + 2;
        end
    else
        u=rem(i,ky)/ky;
        i1=floor(i/ky)+2;%rem取余，取余后再除以k是因为u为小数，自己代数字验证一下
    end
    %i1为B(X,Y)对应A(x,y)的横坐标整数部分，加2是因为上面扩大了四行四列，要回到原来图像的点再计算。
    A=[sw(1+u) sw(u) sw(1-u) sw(2-u)];   %四个横坐标的权重W(i)
    for j=1:kx*n
        
        
        if i == 200 && j == 200
            savdkbh = 1;
        end
        
        if 1
            if 1
                temp = [i j 1]*inv(aa);
                v = temp(2)-floor(temp(2));
                j1 = floor(temp(2)) + 2;
            else
                temp = [i j]*inv(aa(1:2,1:2));
                %             temp = [(i+0.5)*(1/kx)-0.5  (j+0.5)*(1/ky)-0.5];
                v = temp(2)-floor(temp(2));
                j1 = floor(temp(2)) + 2;
            end
        else
            v=rem(j,kx)/kx;   %纵坐标原理同上
            j1=floor(j/kx)+2;
        end
        
        C=[sw(1+v);sw(v);sw(1-v);sw(2-v)]; %转置
        B=[f1(i1-1,j1-1)     f1(i1-1,j1)     f1(i1-1,j1+1)     f1(i1-1,j1+2)    %坐标P(x+u,y+v)最近的16个点的像素值
            f1(i1    ,j1-1)     f1(i1,   j1)     f1(i1,   j1+1)     f1(i1,   j1+2)
            f1(i1+1,j1-1)     f1(i1+1,j1)    f1(i1+1,j1+1)   f1(i1+1,j1+2)
            f1(i1+2,j1-1)     f1(i1+2,j1)    f1(i1+2,j1+1)   f1(i1+2,j1+2)];
        g1(i,j)=(A*B*C);
        AMat = [AMat;A];
        BMat = [BMat;C'];
    end
end
cc = imresize(ff,[size(ff,1)*ky size(ff,2)*kx],'Method','bicubic');
figure,imshow(abs(cc(2:end-1,2:end-1)-g1(2:end-1,2:end-1)),[]);

end


function B = bicubic8(A,kx,ky)

a = [ky,0,0;0,kx,0;0.5*(1-ky),0.5*(1-kx),1];
[m,n]=size(A);
for i=1:ky*m
    for j=1:kx*n
        temp = [i j 1]*inv(a);
        di=temp(1)-floor(temp(1));
        dj=temp(2)-floor(temp(2));
        y = floor(temp(1));
        x = floor(temp(2));
        aX=max(x-3,1);
        bX=max(x-2,1);
        cX=max(x-1,1);
        dX=max(x,1);
        eX=min(x+1,n);
        fX=min(x+2,n);
        gX=min(x+3,n);
        hX=min(x+4,n);
        aY=max(y-3,1);
        bY=max(y-2,1);
        cY=max(y-1,1);
        dY=max(y,1);
        eY=min(y+1,m);
        fY=min(y+2,m);
        gY=min(y+3,m);
        hY=min(y+4,m);
        
        %  first calling for each of the 8 rows (X direction
        %  interpolation)
        temp1=bicubic8x8([A(aY,aX),A(aY,bX),A(aY,cX),A(aY,dX),A(aY,eX),A(aY,fX),A(aY,gX),A(aY,hX)],dj);
        temp2=bicubic8x8([A(bY,aX),A(bY,bX),A(bY,cX),A(bY,dX),A(bY,eX),A(bY,fX),A(bY,gX),A(bY,hX)],dj);
        temp3=bicubic8x8([A(cY,aX),A(cY,bX),A(cY,cX),A(cY,dX),A(cY,eX),A(cY,fX),A(cY,gX),A(cY,hX)],dj);
        temp4=bicubic8x8([A(dY,aX),A(dY,bX),A(dY,cX),A(dY,dX),A(dY,eX),A(dY,fX),A(dY,gX),A(dY,hX)],dj);
        temp5=bicubic8x8([A(eY,aX),A(eY,bX),A(eY,cX),A(eY,dX),A(eY,eX),A(eY,fX),A(eY,gX),A(eY,hX)],dj);
        temp6=bicubic8x8([A(fY,aX),A(fY,bX),A(fY,cX),A(fY,dX),A(fY,eX),A(fY,fX),A(fY,gX),A(fY,hX)],dj);
        temp7=bicubic8x8([A(gY,aX),A(gY,bX),A(gY,cX),A(gY,dX),A(gY,eX),A(gY,fX),A(gY,gX),A(gY,hX)],dj);
        temp8=bicubic8x8([A(hY,aX),A(hY,bX),A(hY,cX),A(hY,dX),A(hY,eX),A(hY,fX),A(hY,gX),A(hY,hX)],dj);
        
        %  Then calling bicubic for these 8 interpolated values (Y
        %  direction interpolation)
        B(i,j)=bicubic8x8([temp1,temp2,temp3,temp4,temp5,temp6,temp7,temp8],di);
    end
end
end
function [X] = bicubic8x8(v,fact)
X = 0;
if fact==0
    X = v(4);
    return
end
Y = [3 2 1 0 1 2 3 4];
Z = [1 1 1 1 -1 -1 -1 -1];
% 8x8 bicubic polynomial
% r(x) =  (67/56)|x|^3 - (123/56)|x|^2              +  1     , 0 <= |x| < 1
% r(x) = -(33/56)|x|^3 + (177/56)|x|^2 - (75/14)|x| + (39/14), 1 <= |x| < 2
% r(x) =  (9/56)|x|^3  - (75/56)|x|^2  + (51/14)|x| - (45/14), 2 <= |x| < 3
% r(x) = -(3/56)|x|^3  + (33/56)|x|^2  - (15/7)|x|  + (18/7) , 3 <= |x| < 4
% The sum of r(3+fact)*v1 + r(2+fact)*v2 + r(1+fact)*v3 + r(fact)*v4 +
% r(1-fact)*v5 + r(2-fact)*v6 + r(3-fact)*v7 + r(4-fact)*v8

A = 67/56;
B = -123/56;
C = 1;
D = -33/56;
E = 177/56;
F = -75/14;
G = 39/14;
H = 9/56;
I = -75/56;
J = 51/14;
K = -45/14;
L = -3/56;
M = 33/56;
N = -15/7;
O = 18/7;

for i = 1:8
    if v(i)~=0
        Fi = Y(i)+fact*Z(i);
        if Fi<1
            X = X + v(i)*(1 + Fi*Fi*(B + Fi*A));
        elseif Fi<2
            X = X + v(i)*(G + Fi*(F + Fi*(E + Fi*D)));
        elseif Fi<3
            X = X + v(i)*(K + Fi*(J + Fi*(I + Fi*H)));
        elseif Fi<4
            X = X + v(i)*(O + Fi*(N + Fi*(M + Fi*L)));
        end
    end
end
end



function A=sw(w1)
w=abs(w1);
a=-0.5;
if w<1&&w>=0
    A=1-(a+3)*w^2+(a+2)*w^3;
else if w>=1&&w<2
        A=a*w^3-5*a*w^2+(8*a)*w-4*a;
    else
        A=0;
    end
end
end


function B = bicubic4x4FastUse(A, kx,ky)
afact = -0.5;
[m,n] = size(A);
NewRows = size(A,1)*ky;
NewColumns = size(A,2)*kx;
a = [ky,0,0;0,kx,0;0.5*(1-ky),0.5*(1-kx),1];
[xuu,yuu] = meshgrid([1:NewRows],[1:NewColumns]);
%         Temp = [repmat(i,NewColumns,1) [1:NewColumns]' ones(NewColumns, 1)]*inv(a);
Temp = [xuu(:) yuu(:) ones(NewColumns*NewRows, 1)]*inv(a);
Di=Temp(:,1)-floor(Temp(:,1)); % di = yfract
Dj=Temp(:,2)-floor(Temp(:,2)); % dj = xfract
Y = floor(Temp(:,1));
X = floor(Temp(:,2));
AX=max(X-1,1);
BX=max(X,1);
CX=min(X+1,n);
DX=min(X+2,n);
AY=max(Y-1,1);
BY=max(Y,1);
CY=min(Y+1,m);
DY=min(Y+2,m);
ind1 = sub2ind(size(A),AY,AX);
ind2 = sub2ind(size(A),AY,BX);
ind3 = sub2ind(size(A),AY,CX);
ind4 = sub2ind(size(A),AY,DX);
ind5 = sub2ind(size(A),BY,AX);
ind6 = sub2ind(size(A),BY,BX);
ind7 = sub2ind(size(A),BY,CX);
ind8 = sub2ind(size(A),BY,DX);
ind9 = sub2ind(size(A),CY,AX);
ind10 = sub2ind(size(A),CY,BX);
ind11 = sub2ind(size(A),CY,CX);
ind12 = sub2ind(size(A),CY,DX);
ind13 = sub2ind(size(A),DY,AX);
ind14 = sub2ind(size(A),DY,BX);
ind15 = sub2ind(size(A),DY,CX);
ind16 = sub2ind(size(A),DY,DX);
Temp1=bicubic4x4Fast([A(ind1),A(ind2),A(ind3),A(ind4)],Dj,afact);
Temp2=bicubic4x4Fast([A(ind5),A(ind6),A(ind7),A(ind8)],Dj,afact);
Temp3=bicubic4x4Fast([A(ind9),A(ind10),A(ind11),A(ind12)],Dj,afact);
Temp4=bicubic4x4Fast([A(ind13),A(ind14),A(ind15),A(ind16)],Dj,afact);
%         B(i,:)=bicubic4x4Fast([Temp1,Temp2,Temp3,Temp4],Di,afact);
B=reshape(bicubic4x4Fast([Temp1,Temp2,Temp3,Temp4],Di,afact),NewColumns,NewRows)';
end
function [X] = bicubic4x4Fast(v,fact,afact)

X = zeros(size(v,1),1);
if fact==0
    X = v(2);
    return
end
Y = [1 0 1 2];
Z = [1 1 -1 -1];
% r(x) = (a + 2)|x|^3 - (a + 3)|x|^2         +  1 , 0 <= |x| < 1
% r(x) =       a|x|^3 -      5a|x|^2 + 8a|x| - 4a , 1 <= |x| < 2
% where x is {fact,fact+1,abs(fact-1),abs(fact-2)}
% then r(x) will contain the interpolation ratio which will be multiplied
% by the point value.
% The sum of r(fact+1)*v1+r(fact)*v2+r(abs(fact-1))*v3+r(abs(fact-2))*v4
% will give the interpolated value.
% The value a is taken as -0.5 as per Matlab. Play
% around with the value of a and you get some interesting results.
% ALL THE CUBIC EQUATION HAVE BEEN PICKED FROM RESEARCH PAPERS I DO NOT
% KNOW THEIR DERIVATION OR WHY THE FACTOR a EXISTS IN 4x4 but not in the
% others. The only reason I can think of is that for 6x6 & 8x8 bicubic
% equations are continous till the second differential that
% is : F(x-) = F(x+), F'(x-) = F'(x+) & is F''(x-) = F''(x+)
% while 4x4 bicubic is only : F(x-) = F(x+), F'(x-) = F'(x+)

% a = -0.5;
a = afact;
A=(2+a);
B=-(3+a);
C=1;
D=a;
E=-5*a;
F=8*a;
G=-4*a;

% for i = 1:4
%     if v(i)~=0
%         Fi = Y(i)+fact*Z(i);
%         if Fi<1
%             X = X + v(i)*(1 + Fi*Fi*(B + Fi*A));
%         elseif Fi<2
%             X = X + v(i)*(G + Fi*(F + Fi*(E + Fi*D)));
%         end
%     end
% end


% flag = v~=0;
for i = 1:4
    if 1 %v(i)~=0
        Fi = Y(i)+fact*Z(i);
        if sum(Fi<1)~=0
            X(Fi<1,1) = X(Fi<1,1) + v(Fi<1,i).*(1 + Fi(Fi<1).^2.*(B + Fi(Fi<1.)*A));
        end
        
        if sum(Fi>=1 & Fi<2)~=0
            X(Fi>=1 & Fi<2,1) = X(Fi>=1 & Fi<2,1) + v(Fi>=1 & Fi<2,i).*(G + Fi(Fi>=1 & Fi<2).*(F + Fi(Fi>=1 & Fi<2).*(E + Fi(Fi>=1 & Fi<2).*D)));
        end
        % %         if Fi<1
        % %             X = X + v(i)*(1 + Fi*Fi*(B + Fi*A));
        % %         elseif Fi<2
        % %             X = X + v(i)*(G + Fi*(F + Fi*(E + Fi*D)));
        % %         end
    end
end
end
function B = bicubic8x8FastUse(A, kx,ky)
% afact = -0.5;
[m,n] = size(A);
NewRows = size(A,1)*ky;
NewColumns = size(A,2)*kx;
a = [ky,0,0;0,kx,0;0.5*(1-ky),0.5*(1-kx),1];
[xuu,yuu] = meshgrid([1:NewRows],[1:NewColumns]);
%         Temp = [repmat(i,NewColumns,1) [1:NewColumns]' ones(NewColumns, 1)]*inv(a);
Temp = [xuu(:) yuu(:) ones(NewColumns*NewRows, 1)]*inv(a);
Di=Temp(:,1)-floor(Temp(:,1)); % di = yfract
Dj=Temp(:,2)-floor(Temp(:,2)); % dj = xfract
Y = floor(Temp(:,1));
X = floor(Temp(:,2));

AX=max(X-3,1);
BX=max(X-2,1);
CX=max(X-1,1);
DX=max(X,1);
EX=min(X+1,n);
FX=min(X+2,n);
GX=min(X+3,n);
HX=min(X+4,n);
AY=max(Y-3,1);
BY=max(Y-2,1);
CY=max(Y-1,1);
DY=max(Y,1);
EY=min(Y+1,m);
FY=min(Y+2,m);
GY=min(Y+3,m);
HY=min(Y+4,m);


ind1 = sub2ind(size(A),AY,AX);
ind2 = sub2ind(size(A),AY,BX);
ind3 = sub2ind(size(A),AY,CX);
ind4 = sub2ind(size(A),AY,DX);
ind5 = sub2ind(size(A),AY,EX);
ind6 = sub2ind(size(A),AY,FX);
ind7 = sub2ind(size(A),AY,GX);
ind8 = sub2ind(size(A),AY,HX);

ind9 = sub2ind(size(A),BY,AX);
ind10 = sub2ind(size(A),BY,BX);
ind11 = sub2ind(size(A),BY,CX);
ind12 = sub2ind(size(A),BY,DX);
ind13 = sub2ind(size(A),BY,EX);
ind14 = sub2ind(size(A),BY,FX);
ind15= sub2ind(size(A),BY,GX);
ind16 = sub2ind(size(A),BY,HX);

ind17 = sub2ind(size(A),CY,AX);
ind18 = sub2ind(size(A),CY,BX);
ind19 = sub2ind(size(A),CY,CX);
ind20 = sub2ind(size(A),CY,DX);
ind21 = sub2ind(size(A),CY,EX);
ind22 = sub2ind(size(A),CY,FX);
ind23 = sub2ind(size(A),CY,GX);
ind24 = sub2ind(size(A),CY,HX);

ind25 = sub2ind(size(A),DY,AX);
ind26 = sub2ind(size(A),DY,BX);
ind27 = sub2ind(size(A),DY,CX);
ind28 = sub2ind(size(A),DY,DX);
ind29 = sub2ind(size(A),DY,EX);
ind30 = sub2ind(size(A),DY,FX);
ind31 = sub2ind(size(A),DY,GX);
ind32 = sub2ind(size(A),DY,HX);

ind33 = sub2ind(size(A),EY,AX);
ind34 = sub2ind(size(A),EY,BX);
ind35 = sub2ind(size(A),EY,CX);
ind36 = sub2ind(size(A),EY,DX);
ind37 = sub2ind(size(A),EY,EX);
ind38 = sub2ind(size(A),EY,FX);
ind39 = sub2ind(size(A),EY,GX);
ind40 = sub2ind(size(A),EY,HX);

ind41 = sub2ind(size(A),FY,AX);
ind42 = sub2ind(size(A),FY,BX);
ind43 = sub2ind(size(A),FY,CX);
ind44 = sub2ind(size(A),FY,DX);
ind45 = sub2ind(size(A),FY,EX);
ind46 = sub2ind(size(A),FY,FX);
ind47 = sub2ind(size(A),FY,GX);
ind48 = sub2ind(size(A),FY,HX);

ind49 = sub2ind(size(A),GY,AX);
ind50 = sub2ind(size(A),GY,BX);
ind51 = sub2ind(size(A),GY,CX);
ind52 = sub2ind(size(A),GY,DX);
ind53 = sub2ind(size(A),GY,EX);
ind54 = sub2ind(size(A),GY,FX);
ind55 = sub2ind(size(A),GY,GX);
ind56 = sub2ind(size(A),GY,HX);

ind57 = sub2ind(size(A),HY,AX);
ind58 = sub2ind(size(A),HY,BX);
ind59 = sub2ind(size(A),HY,CX);
ind60 = sub2ind(size(A),HY,DX);
ind61 = sub2ind(size(A),HY,EX);
ind62 = sub2ind(size(A),HY,FX);
ind63 = sub2ind(size(A),HY,GX);
ind64 = sub2ind(size(A),HY,HX);


Temp1=bicubic8x8Fast([A(ind1),A(ind2),A(ind3),A(ind4),A(ind5),A(ind6),A(ind7),A(ind8)],Dj);
Temp2=bicubic8x8Fast([A(ind9),A(ind10),A(ind11),A(ind12),A(ind13),A(ind14),A(ind15),A(ind16)],Dj);
Temp3=bicubic8x8Fast([A(ind17),A(ind18),A(ind19),A(ind20),A(ind21),A(ind22),A(ind23),A(ind24)],Dj);
Temp4=bicubic8x8Fast([A(ind25),A(ind26),A(ind27),A(ind28),A(ind29),A(ind30),A(ind31),A(ind32)],Dj);
Temp5=bicubic8x8Fast([A(ind33),A(ind34),A(ind35),A(ind36),A(ind37),A(ind38),A(ind39),A(ind40)],Dj);
Temp6=bicubic8x8Fast([A(ind41),A(ind42),A(ind43),A(ind44),A(ind45),A(ind46),A(ind47),A(ind48)],Dj);
Temp7=bicubic8x8Fast([A(ind49),A(ind50),A(ind51),A(ind52),A(ind53),A(ind54),A(ind55),A(ind56)],Dj);
Temp8=bicubic8x8Fast([A(ind57),A(ind58),A(ind59),A(ind60),A(ind61),A(ind62),A(ind63),A(ind64)],Dj);

%  Then calling bicubic for these 8 interpolated values (Y
%  direction interpolation)

%         B(i,:)=bicubic8x8Fast([Temp1,Temp2,Temp3,Temp4],Di,afact);
B=reshape(bicubic8x8Fast([Temp1,Temp2,Temp3,Temp4,Temp5,Temp6,Temp7,Temp8],Di),NewColumns,NewRows)';
end
function [X] = bicubic8x8Fast(v,fact)
X = zeros(size(v,1),1);
if fact==0
    X = v(4);
    return
end
Y = [3 2 1 0 1 2 3 4];
Z = [1 1 1 1 -1 -1 -1 -1];
% 8x8 bicubic polynomial
% r(x) =  (67/56)|x|^3 - (123/56)|x|^2              +  1     , 0 <= |x| < 1
% r(x) = -(33/56)|x|^3 + (177/56)|x|^2 - (75/14)|x| + (39/14), 1 <= |x| < 2
% r(x) =  (9/56)|x|^3  - (75/56)|x|^2  + (51/14)|x| - (45/14), 2 <= |x| < 3
% r(x) = -(3/56)|x|^3  + (33/56)|x|^2  - (15/7)|x|  + (18/7) , 3 <= |x| < 4
% The sum of r(3+fact)*v1 + r(2+fact)*v2 + r(1+fact)*v3 + r(fact)*v4 +
% r(1-fact)*v5 + r(2-fact)*v6 + r(3-fact)*v7 + r(4-fact)*v8

A = 67/56;
B = -123/56;
C = 1;
D = -33/56;
E = 177/56;
F = -75/14;
G = 39/14;
H = 9/56;
I = -75/56;
J = 51/14;
K = -45/14;
L = -3/56;
M = 33/56;
N = -15/7;
O = 18/7;

for i = 1:8
    %     if v(i)~=0
    %         Fi = Y(i)+fact*Z(i);
    %         if Fi<1
    %             X = X + v(i)*(1 + Fi*Fi*(B + Fi*A));
    %         elseif Fi<2
    %             X = X + v(i)*(G + Fi*(F + Fi*(E + Fi*D)));
    %         elseif Fi<3
    %             X = X + v(i)*(K + Fi*(J + Fi*(I + Fi*H)));
    %         elseif Fi<4
    %             X = X + v(i)*(O + Fi*(N + Fi*(M + Fi*L)));
    %         end
    %     end
    if 1 % v(i)~=0
        Fi = Y(i)+fact*Z(i);
        
        if sum(Fi<1)~=0
            X(Fi<1,1) = X(Fi<1,1) + v(Fi<1,i).*(1 + Fi(Fi<1).^2.*(B + Fi(Fi<1.)*A));
        end
        flag2 = Fi>=1 & Fi<2;
        if sum(flag2) ~= 0
            X(flag2) = X(flag2) + v(flag2,i).*(G + Fi(flag2).*(F + Fi(flag2).*(E + Fi(flag2).*D)));
        end
        flag3 = Fi>=2 & Fi<3;
        if sum(flag3) ~= 0
            X(flag3) = X(flag3) + v(flag3,i).*(K + Fi(flag3).*(J + Fi(flag3).*(I + Fi(flag3).*H)));
        end
        flag4 = Fi>=3 & Fi<4;
        if sum(flag4) ~= 0
            X(flag4) = X(flag4) + v(flag4,i).*(O + Fi(flag4).*(N + Fi(flag4).*(M + Fi(flag4).*L)));
        end
        %
        %         if Fi<1
        %             X = X + v(i)*(1 + Fi*Fi*(B + Fi*A));
        %         elseif Fi<2
        %             X = X + v(i)*(G + Fi*(F + Fi*(E + Fi*D)));
        %         elseif Fi<3
        %             X = X + v(i)*(K + Fi*(J + Fi*(I + Fi*H)));
        %         elseif Fi<4
        %             X = X + v(i)*(O + Fi*(N + Fi*(M + Fi*L)));
        %         end
    end
end
end

function B = bicubic6x6FastUse(A,kx,ky)
[m,n] = size(A);
NewRows = size(A,1)*ky;
NewColumns = size(A,2)*kx;
a = [ky,0,0;0,kx,0;0.5*(1-ky),0.5*(1-kx),1];
[xuu,yuu] = meshgrid([1:NewRows],[1:NewColumns]);
%         Temp = [repmat(i,NewColumns,1) [1:NewColumns]' ones(NewColumns, 1)]*inv(a);
Temp = [xuu(:) yuu(:) ones(NewColumns*NewRows, 1)]*inv(a);
Di=Temp(:,1)-floor(Temp(:,1)); % di = yfract
Dj=Temp(:,2)-floor(Temp(:,2)); % dj = xfract
Y = floor(Temp(:,1));
X = floor(Temp(:,2));

AX=max(X-2,1);
BX=max(X-1,1);
CX=max(X,1);
DX=min(X+1,n);
EX=min(X+2,n);
FX=min(X+3,n);

AY=max(Y-2,1);
BY=max(Y-1,1);
CY=max(Y,1);
DY=min(Y+1,m);
EY=min(Y+2,m);
FY=min(Y+3,m);



ind1 = sub2ind(size(A),AY,AX);
ind2 = sub2ind(size(A),AY,BX);
ind3 = sub2ind(size(A),AY,CX);
ind4 = sub2ind(size(A),AY,DX);
ind5 = sub2ind(size(A),AY,EX);
ind6 = sub2ind(size(A),AY,FX);


ind7 = sub2ind(size(A),BY,AX);
ind8 = sub2ind(size(A),BY,BX);
ind9 = sub2ind(size(A),BY,CX);
ind10 = sub2ind(size(A),BY,DX);
ind11 = sub2ind(size(A),BY,EX);
ind12 = sub2ind(size(A),BY,FX);


ind13 = sub2ind(size(A),CY,AX);
ind14 = sub2ind(size(A),CY,BX);
ind15 = sub2ind(size(A),CY,CX);
ind16 = sub2ind(size(A),CY,DX);
ind17 = sub2ind(size(A),CY,EX);
ind18 = sub2ind(size(A),CY,FX);


ind19 = sub2ind(size(A),DY,AX);
ind20 = sub2ind(size(A),DY,BX);
ind21 = sub2ind(size(A),DY,CX);
ind22 = sub2ind(size(A),DY,DX);
ind23 = sub2ind(size(A),DY,EX);
ind24 = sub2ind(size(A),DY,FX);


ind25 = sub2ind(size(A),EY,AX);
ind26 = sub2ind(size(A),EY,BX);
ind27 = sub2ind(size(A),EY,CX);
ind28 = sub2ind(size(A),EY,DX);
ind29 = sub2ind(size(A),EY,EX);
ind30 = sub2ind(size(A),EY,FX);


ind31 = sub2ind(size(A),FY,AX);
ind32 = sub2ind(size(A),FY,BX);
ind33 = sub2ind(size(A),FY,CX);
ind34 = sub2ind(size(A),FY,DX);
ind35 = sub2ind(size(A),FY,EX);
ind36 = sub2ind(size(A),FY,FX);


Temp1=bicubic6x6Fast([A(ind1),A(ind2),A(ind3),A(ind4),A(ind5),A(ind6)],Dj);
Temp2=bicubic6x6Fast([A(ind7),A(ind8),A(ind9),A(ind10),A(ind11),A(ind12)],Dj);
Temp3=bicubic6x6Fast([A(ind13),A(ind14),A(ind15),A(ind16),A(ind17),A(ind18)],Dj);
Temp4=bicubic6x6Fast([A(ind19),A(ind20),A(ind21),A(ind22),A(ind23),A(ind24)],Dj);
Temp5=bicubic6x6Fast([A(ind25),A(ind26),A(ind27),A(ind28),A(ind29),A(ind30)],Dj);
Temp6=bicubic6x6Fast([A(ind31),A(ind32),A(ind33),A(ind34),A(ind35),A(ind36)],Dj);

B=reshape(bicubic6x6Fast([Temp1,Temp2,Temp3,Temp4,Temp5,Temp6],Di),NewColumns,NewRows)';
end
function [X] = bicubic6x6Fast(v,fact)
X = zeros(size(v,1),1);
if fact==0
    X = v(3);
    return
end
Y = [2 1 0 1 2 3];
Z = [1 1 1 -1 -1 -1];
% 6x6 bicubic polynomial
% r(x) =  (6/5)|x|^3 - (11/5)|x|^2              +  1    , 0 <= |x| < 1
% r(x) = -(3/5)|x|^3 + (16/5)|x|^2 -(27/5)|x|   +  14/5 , 1 <= |x| < 2
% r(x) =  (1/5)|x|^3 - (8/5)|x|^2  +(21/5)|x|   -  18/5 , 2 <= |x| < 3
%
% The sum of r(2+fact)*v1 + r(1+fact)*v2 + r(fact)*v3 + r(1-fact)*v4 +
% r(2-fact)*v5 + r(3-fact)*v6 represents the interpolated value

A =  4/3; 6/5;
B = -7/3; -11/5;
C =  1;
D = -7/12; -3/5;
E =  3; 16/5;
F = -59/12; -27/5;
G =  15/6; 14/5;
H =  1/12; 1/5;
I = -2/3; -8/5;
J =  21/12; 21/5;
K = -3/2; -18/5;

for i = 1:6
    %     if v(i)~=0
    %         Fi = Y(i)+fact*Z(i);
    %         if Fi<1
    %             X = X + v(i)*(1 + Fi*Fi*(B + Fi*A));
    %         elseif Fi<2
    %             X = X + v(i)*(G + Fi*(F + Fi*(E + Fi*D)));
    %         elseif Fi<3
    %             X = X + v(i)*(K + Fi*(J + Fi*(I + Fi*H)));
    %         end
    %     end
    
    if 1 %v(i)~=0
        Fi = Y(i)+fact*Z(i);
        flag1 = Fi < 1;
        if sum(flag1) ~= 0
            X(flag1) = X(flag1) + v(flag1,i).*(1 + Fi(flag1).*Fi(flag1).*(B + Fi(flag1).*A));
        end
        flag2 = Fi >= 1& Fi < 2;
        if sum(flag2) ~= 0
            X(flag2) = X(flag2) + v(flag2,i).*(G + Fi(flag2).*(F + Fi(flag2).*(E + Fi(flag2).*D)));
        end
        flag3 = Fi >= 2& Fi < 3;
        if sum(flag3) ~= 0
            X(flag3) = X(flag3) + v(flag3,i).*(K + Fi(flag3).*(J + Fi(flag3).*(I + Fi(flag3).*H)));
        end
        
        %
        %         if Fi<1
        %             X = X + v(i)*(1 + Fi*Fi*(B + Fi*A));
        %         elseif Fi<2
        %             X = X + v(i)*(G + Fi*(F + Fi*(E + Fi*D)));
        %         elseif Fi<3
        %             X = X + v(i)*(K + Fi*(J + Fi*(I + Fi*H)));
        %         end
    end
end


end

function pixRect = Orig2Rect(pix, KOrig, KRect, R,kc)

[pixUndist] = normalize_pixel(pix',[KOrig(1,1);KOrig(2,2)],[KOrig(1,3);KOrig(2,3)],kc,0);
pixUndistR = R*pextend(pixUndist);
pixRect = pflat(KRect*pixUndistR);
pixRect = pixRect(1:2,:)';



end
