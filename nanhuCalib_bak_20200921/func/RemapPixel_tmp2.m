% function [xRectMatUse, yRectMatUse,KK_new, Kinit,k,imgL1, LutVecCharX, LutVecCharY, SplitIndX, SplitIndY,lutVecX,lutVecY] = RemapPixel_tmp2(I,R,f,c,k,alpha,KK_new,imgL,stepXY,reverseMapping,varargin)
function [newXSAmple, newYSAmple, xRectMat1, yRectMat1, KK_new, Kinit,k,imgL1, LutVecCharX, LutVecCharY, SplitIndX, SplitIndY,lutVecX,lutVecY,nameMatX,nameMatY, imgOut] = RemapPixel_tmp2(I,R,f,c,k,alpha,KK_new,imgL,stepXY,reverseMapping,varargin)
global cfg
% reverseMapping = 1: Orig2Rect
% reverseMapping = 0: Rect2Orig

factory_speed = 1;
speedUp = 1; 0; 1;

step = -0.25;

SplitIndX = []; SplitIndY = [];
LutVecCharX = []; LutVecCharY = [];
lutVecX = []; lutVecY = [];
if (nargin == 10)
    paraDir = [];
    %     homoDir = 'D:\Temp\20180828\calibFishEyeHomo1';
    %     heightDir = 'D:\Temp\20181213\apriltag7';
    
elseif (nargin == 11)
    paraDir = varargin{1};
    whichOne = 'Left';
    newXSAmple = [];
    newYSAmple = [];
elseif (nargin == 12)
    paraDir = varargin{1};
    whichOne = varargin{2};
    newXSAmple = [];
    newYSAmple = [];
elseif (nargin == 13)
    paraDir = varargin{1};
    whichOne = varargin{2};
    newXSAmple = varargin{3};
    newYSAmple = [];
elseif (nargin == 14)
    paraDir = varargin{1};
    whichOne = varargin{2};
    newXSAmple = varargin{3};
    newYSAmple = varargin{4};
else
    error('Too many input arguments');
end

if ~isempty(paraDir)
    fishEyeModel = 1;
    
    if strcmp(whichOne, 'Left')
        if cfg.calib_stereo
            load(fullfile(paraDir,'calib.mat'),'oCamModelL','stereoParam')
        else
            load(fullfile(paraDir,'calib.mat'),'oCamModelL','camParamL')
            stereoParam = camParamL;
        end
        try
            oCamModel = oCamModelL;
        catch
            load(fullfile(paraDir,'oCamModelR.mat'));
            rectImgL = imgL;
            oCamModel = oCamModelR;
        end
        
        U_same = ocam_undistort_map(oCamModel,'OutputView','full');
        nim_same = ocam_undistort(imgL,U_same);
        try
            [rectParamL, rectParamR] = GetRectifyParam(stereoParam, size(imgL));
            [rectImgL] = RectifyOneImage(nim_same, rectParamL);
        catch
            sdfgkh = 1;
        end
    else
        load(fullfile(paraDir,'calib.mat'),'oCamModelR','stereoParam')
        oCamModel = oCamModelR;
        [rectParamL, rectParamR] = GetRectifyParam(stereoParam, size(imgL));
        U_same = ocam_undistort_map(oCamModel,'OutputView','full');
        nim_same = ocam_undistort(imgL,U_same);
        [rectImgR] = RectifyOneImage(nim_same, rectParamR);
    end
    
    
else
    fishEyeModel = 0;
end


draw = 0;
scaleSizeX = stepXY(1);
scaleSizeY = stepXY(2);
% reverseMapping = 1;

marg = 10;
skipX = 75; skipY = 75;
skipX = 85; skipY = 85;
dltErr = 0;0.5;
rowNum = size(I,1);
colNum = size(I,2);

endThr = 1;2; 3;
change = 1;

useTable = 0;1;



if change
    %     scaleSizeY = 120; 120;%40;10;40;60;20;20;60;60;10;20;2;36;2;  %rowNum/yLen;
    %     scaleSizeX = 160; 160;%40;10;40;80;20;20;80;10;20;2;48;2;  colNum/xLen;
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


[nr,nc] = size(I(:,:,1));

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
if 0
    if fishEyeModel == 0
        figure(11),plotUndistortion(nc, nr, Kinit, Kinit,k,eye(3),skipNum);
        figure(12),pixNewAll = plotUndistortion(nc, nr, Kinit, KK_new,k,R,skipNum);
        % % [xuu,yuu] = meshgrid(1:nc/skipNum:nc, 1:nr/skipNum:nr);
    else
        figure(11),plotUndistortionFishEye(nc, nr, Kinit, Kinit,k,eye(3),skipNum, oCamModel);
        figure(12),pixNewAll = plotUndistortionFishEye(nc, nr, Kinit, KK_new,k,R,skipNum, oCamModel);
    end
end
if 0
    [xuu2, yuu2] = meshgrid(1:(colNum-1)/(xLen-1):colNum,1:(rowNum-1)/(yLen-1):rowNum);
else
    
    aTmp = scaleSizeX/2+0.5:(colNum)/(xLen):colNum-1;
    bTmp = scaleSizeY/2+0.5:(rowNum)/(yLen):rowNum-1;
    Nx = 5000;
    aTmp1 = nc/2+0.5-scaleSizeX/2 : -(colNum)/(xLen) : nc/2+0.5-scaleSizeX/2-Nx*(colNum)/(xLen);
    aTmp1 = aTmp1(end:-1:1);
    aTmp2 = nc/2+0.5+scaleSizeX/2 : (colNum)/(xLen) : nc/2+0.5+scaleSizeX/2+Nx*(colNum)/(xLen);
    aTmp = [aTmp1 aTmp2];
    cntX = 0;
    goodX = 10;
    if 1
        while (~(aTmp(1)>0 || aTmp(end)<nc)) || (goodX < 3)
            if (aTmp(1)>0 || aTmp(end)<nc)
                goodX = goodX + 1;
            end
            cntX = cntX + 1;
            Nx = Nx-1;
            if step > 0
                stepList = [ scaleSizeX/2 [(colNum)/(xLen)+step : step: 10000]];
            else
                stepList = [ scaleSizeX/2 [(colNum)/(xLen)+step : step: 0]];
            end
            %         aTmp11 =
            aTmp1 = nc/2+0.5-scaleSizeX/2 : -(colNum)/(xLen) : nc/2+0.5-scaleSizeX/2-Nx*(colNum)/(xLen);
            %         aTmp1 = nc/2+0.5 - cumsum(stepList);
            aTmp1 = aTmp1(end:-1:1);
            aTmp2 = nc/2+0.5+scaleSizeX/2 : (colNum)/(xLen) : nc/2+0.5+scaleSizeX/2+Nx*(colNum)/(xLen);
            %         aTmp2 = nc/2+0.5 + cumsum(stepList);
            aTmp = [aTmp1 aTmp2];
            ATmp{cntX,1} = aTmp;
        end
        
        aTmp_ = ATmp{end-endThr};
    else
        %         while (~(aTmp(1)>0 || aTmp(end)<nc)) || (goodX < 3)
        if (aTmp(1)>0 || aTmp(end)<nc)
            goodX = goodX + 1;
        end
        cntX = cntX + 1;
        Nx = Nx-1;
        if step > 0
            stepListX = [ scaleSizeX/2 [(colNum)/(xLen)+step : step: 20000]];
        else
            stepListX = [ scaleSizeX/2 [(colNum)/(xLen)+step : step: 0]];
        end
        %         aTmp11 =
        %             aTmp1 = nc/2+0.5-scaleSizeX/2 : -(colNum)/(xLen) : nc/2+0.5-scaleSizeX/2-Nx*(colNum)/(xLen);
        aTmp1 = nc/2+0.5 - cumsum(stepListX);
        stepListXX = stepListX(end:-1:1);
        aTmp1 = aTmp1(end:-1:1);
        %             aTmp2 = nc/2+0.5+scaleSizeX/2 : (colNum)/(xLen) : nc/2+0.5+scaleSizeX/2+Nx*(colNum)/(xLen);
        aTmp2 = nc/2+0.5 + cumsum(stepListX);
        aTmp = [aTmp1 aTmp2];
        ATmp{cntX,1} = aTmp;
        idX = find(aTmp(:) > 0 & aTmp(:) <=nc);
        aTmp_ = aTmp(intersect(idX(1)-1:idX(end)+1,find([stepListXX stepListX]' > 0)));
        if length(aTmp_) ~= length(idX) + 2
            fprintf(sprintf('\n### WRONG ###\n'));
            
        end
        
        %         end
    end
    %     aTmp_ = ATmp{end};
    
    Ny = 5000;
    bTmp1 = nr/2+0.5-scaleSizeY/2 : -(rowNum)/(yLen) : nr/2+0.5-scaleSizeY/2-Ny*(rowNum)/(yLen);
    bTmp1 = bTmp1(end:-1:1);
    bTmp2 = nr/2+0.5+scaleSizeY/2 : (rowNum)/(yLen) : nr/2+0.5+scaleSizeY/2+Ny*(rowNum)/(yLen);
    bTmp = [bTmp1 bTmp2];
    cntY = 0;
    goodY = 10;
    if 1
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
    else
        %         while (~(bTmp(1)>0 || bTmp(end)<nr)) || goodY < 3
        if (bTmp(1)>0 || bTmp(end)<nr)
            goodY = goodY + 1;
        end
        cntY = cntY + 1;
        Ny = Ny-1;
        if step > 0
            stepListY = [ scaleSizeY/2 [(rowNum)/(yLen)+step : step: 20000]];
        else
            stepListY = [ scaleSizeY/2 [(rowNum)/(yLen)+step : step: -0]];
        end
        % %         bTmp1 = nc/2+0.5-scaleSizeY/2 : -(colNum)/(xLen) : nc/2+0.5-scaleSizeY/2-Nx*(colNum)/(xLen);
        % %         bTmp1 = bTmp1(end:-1:1);
        % %         bTmp2 = nc/2+0.5+scaleSizeY/2 : (colNum)/(xLen) : nc/2+0.5+scaleSizeY/2+Nx*(colNum)/(xLen);
        % %         bTmp = [bTmp1 bTmp2];
        % %
        %             bTmp1 = nr/2+0.5-scaleSizeY/2 : -(rowNum)/(yLen) : nr/2+0.5-scaleSizeY/2-Ny*(rowNum)/(yLen);
        bTmp1 = nr/2+0.5 - cumsum(stepListY);
        stepListYY = stepListY(end:-1:1);
        bTmp1 = bTmp1(end:-1:1);
        %             bTmp2 = nr/2+0.5+scaleSizeY/2 : (rowNum)/(yLen) : nr/2+0.5+scaleSizeY/2+Ny*(rowNum)/(yLen);
        bTmp2 = nr/2+0.5 + cumsum(stepListY);
        bTmp = [bTmp1 bTmp2];
        
        BTmp{cntY,1} = bTmp;
        %         end
        %         bTmp_ = BTmp{end-endThr};
        idY = find(bTmp(:) > 0 & bTmp(:) <=nr);
        bTmp_ = bTmp(intersect(idY(1)-1:idY(end)+1,find([stepListYY stepListY] > 0)));
        if length(bTmp_) ~= length(idY) + 2
            fprintf(sprintf('\n### WRONG ###\n'));
            
        end
    end
    %     bTmp_ = BTmp{end};
    if ~isempty(newXSAmple)
        aTmp_ = newXSAmple;
        ret = 1;
    end
    if ~isempty(newYSAmple)
        bTmp_ = newYSAmple;
        ret = 0;
    else
        ret = 1;
    end
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
        if fishEyeModel == 0
            pixOrigAll = remapRect([xuuAllUse(:) yuuAllUse(:)]', KK_new, Kinit,k, R);
        else
            pixOrigAll = remapRectFishEye([xuuAllUse(:) yuuAllUse(:)]', KK_new, Kinit,k, R,oCamModel);
        end
    else
        if fishEyeModel == 0
            pixOrigAll = Orig2Rect([xuuAllUse(:) yuuAllUse(:)], Kinit, KK_new, R, k);
        else
            pixOrigAll = Orig2RectFishEye([xuuAllUse(:) yuuAllUse(:)], Kinit, KK_new, R, k,oCamModel);
        end
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
    if fishEyeModel == 0
        pixOrig_2 = remapRect([xuu2(:) yuu2(:)]', KK_new, Kinit,k, R);
    else
        pixOrig_2 = remapRectFishEye([xuu2(:) yuu2(:)]', KK_new, Kinit,k, R,oCamModel);
        pixOrig_2 = pixOrig_2 + repmat([0.0 0.0], size(pixOrig_2,1),1);
    end
else
    if fishEyeModel == 0
        pixOrig_2 = Orig2Rect([xuu2(:) yuu2(:)], Kinit,KK_new, R, k);
    else
        pixOrig_2 = Orig2RectFishEye([xuu2(:) yuu2(:)], Kinit,KK_new, R, k,oCamModel);
    end
end
if 0
    optVec = OptBilinear(pixOrigAll, pixOrig_2, xuu2,yuu2, xuuAll, rowNew, colNew, nr,nc);
    pixOrig_2 = pixOrig_2 + reshape(optVec, size(pixOrig_2));
end


xOrigMat = reshape(pixOrig_2(:,1),size(xuu2));
yOrigMat = reshape(pixOrig_2(:,2),size(xuu2));

if draw
    figure,imshow(zeros(nr,nc));hold on;plot(xuu2(:),yuu2(:),'.r');plot(pixOrig_2(:,1),pixOrig_2(:,2),'.g');
end
if speedUp % 20210601 to speed up for mass production
    
    
    if 1
        optFrac = [5 12 12 9 15 15];
        %     optFrac = [5 12 12 12 9 15 15];
        % [lut lut/size^2(5.15) div*coef(14.2) out]
        
        
        %     optFrac = [23 12 22 9 13 17 17 9 13 17 17 9 9];
        doRounding = 1; 0;
        if isempty(paraDir)
            inputDir = pwd;
        else
            inputDir = paraDir;
        end
    end
    
    
    
    xRectMat = imresize(xOrigMat,size(xuuAll),'Method','bicubic');
    yRectMat = imresize(yOrigMat,size(xuuAll),'Method','bicubic');
    %     xRectMat = xRectMat + dltErr;
    %     yRectMat = yRectMat + dltErr;
elseif 0
    %     xRectMat = imresize(xOrigMat,[nr nc],'Method','bilinear');
    %     yRectMat = imresize(yOrigMat,[nr nc],'Method','bilinear');
    xRectMat = imresize(xOrigMat,size(xuuAll),'Method','bilinear');
    yRectMat = imresize(yOrigMat,size(xuuAll),'Method','bilinear');
    asfkj = 1;
    
elseif 0
    xRectMat = bicubic8x8FastUse(xOrigMat,scaleSizeX,scaleSizeY);
    yRectMat = bicubic8x8FastUse(yOrigMat,scaleSizeX,scaleSizeY);
elseif 0
    xRectMat = bicubic6x6FastUse(xOrigMat,scaleSizeX,scaleSizeY);
    yRectMat = bicubic6x6FastUse(yOrigMat,scaleSizeX,scaleSizeY);
    
elseif 1
    %     xRectMat = bicubic4x4FastUse(xOrigMat,scaleSizeX,scaleSizeY);
    %     yRectMat = bicubic4x4FastUse(yOrigMat,scaleSizeX,scaleSizeY);
    
    
    
    % %     xRectMatFixPt = bicubic4x4FastUseFixPt(xOrigMat,scaleSizeX,scaleSizeY);
    % %     yRectMatFixPt = bicubic4x4FastUseFixPt(yOrigMat,scaleSizeX,scaleSizeY);
    %     optFrac = [23 12 12 6 13 17 17 8 13 17 17 9 9];
    % % % %     optFrac = [23 12 12 6 13 17 17 9 13 17 17 9 9];
    
    %% big change !!
    %     optFrac = [23 12 12 9 13 17 17 9 13 17 17 9 9];% bicubic
    optFrac = [23 15 15 20 5 10]; %[23 12 12 12 6 9];  % [23 12 12 17 9 8];% bilinear
    
    optFrac = [5 15 15 9 15 15];
    optFrac = [5 10 12 9 15 15];
    optFrac = [5 12 12 9 15 15];
    %     optFrac = [5 12 12 12 9 15 15];
    % [lut lut/size^2(5.15) div*coef(14.2) out]
    
    
    %     optFrac = [23 12 22 9 13 17 17 9 13 17 17 9 9];
    doRounding = 1; 0;
    if isempty(paraDir)
        inputDir = pwd;
    else
        inputDir = paraDir;
    end
    %     [xRectMat1, yRectMat1, LutVecCharX, LutVecCharY, SplitIndX, SplitIndY,lutVecX,lutVecY] = BiCubicInterp(inputDir,xOrigMat,yOrigMat,xRectMatRef,yRectMatRef,scaleSizeX,scaleSizeY,nc,nr,doRounding,optFrac); %bilinear2x2Func
    %     [xRectMat1, yRectMat1, LutVecCharX, LutVecCharY, SplitIndX, SplitIndY,lutVecX,lutVecY] = BiLinearInterp(inputDir,xOrigMat,yOrigMat,xRectMatRef,yRectMatRef,scaleSizeX,scaleSizeY,nc,nr,doRounding,optFrac); %bilinear2x2Func
    [xRectMat1, yRectMat1, LutVecCharX, LutVecCharY, SplitIndX, SplitIndY,lutVecX,lutVecY,nameMatX, nameMatY] = BiLinearInterp_(reverseMapping, inputDir,xuu2, yuu2, xOrigMat,yOrigMat,xRectMatRef,yRectMatRef,scaleSizeX,scaleSizeY,nc,nr,doRounding,optFrac, speedUp); %bilinear2x2Func
    
else
    xRectMat = bicubic(xOrigMat,scaleSizeX,scaleSizeY);
    yRectMat = bicubic(yOrigMat,scaleSizeX,scaleSizeY);
end
try
    xRectMat = xRectMat + dltErr;
    yRectMat = yRectMat + dltErr;
    xRectMatUse = xRectMat((rowNew-nr)/2+1:end-(rowNew-nr)/2,(colNew-nc)/2+1:end-(colNew-nc)/2);
    yRectMatUse = yRectMat((rowNew-nr)/2+1:end-(rowNew-nr)/2,(colNew-nc)/2+1:end-(colNew-nc)/2);
    % try
catch
    xRectMatUse = xRectMat1;
    yRectMatUse = yRectMat1;
    % catch
    sdghjf = 1;
end
if 1
    idWithin = find(pixOrigAll(:,1) >= 1 & pixOrigAll(:,1) <= nc & pixOrigAll(:,2) >= 1 & pixOrigAll(:,2) <= nr);
    idOut = find(~(pixOrigAll(:,1) >= 1 & pixOrigAll(:,1) <= nc & pixOrigAll(:,2) >= 1 & pixOrigAll(:,2) <= nr));
    margg = 100;
    idWithin2 = find(pixOrigAll(:,1) >= margg & pixOrigAll(:,1) <= nc-margg & pixOrigAll(:,2) >= margg & pixOrigAll(:,2) <= nr-margg);
else
    idWithin = [1:nc*nr]';
end
errorMat = zeros([nr,nc]);
[~,err] = NormalizeVector(pixOrigAll(idWithin,:) - [xRectMatUse(idWithin)+0 yRectMatUse(idWithin)+0]);
errorMat(idWithin) = err;
if draw
    figure,imshow(errorMat < 0.5);
end
adgh = 1;
if 0
    midRowMatX = xuu2(size(xuu2,1)/2 : size(xuu2,1)/2 +1 ,:);
    midRowMatY = yuu2(size(xuu2,1)/2 : size(xuu2,1)/2 +1 ,:);
    midColMatLeftX = midRowMatX(:,1:size(midRowMatX,2)/2);
    midColMatLeftY = midRowMatY(:,1:size(midRowMatX,2)/2);
    xOrigMatMidRow = xOrigMat(size(xuu2,1)/2 : size(xuu2,1)/2 +1,1:size(xuu2,2)/2);
    yOrigMatMidRow = yOrigMat(size(xuu2,1)/2 : size(xuu2,1)/2 +1,1:size(xuu2,2)/2);
    errorMatMidLeft = errorMat(round(bTmp_(size(xuu2,1)/2)):round(bTmp_(size(xuu2,1)/2+1)),1:round(aTmp_(size(xuu2,2)/2)));
    errorMatMidLeftSum = mean(errorMatMidLeft);
    extraCnt = 1;
    for t = 1 : length(aTmp_)/2-1
        errorMatMidLeft_ = errorMat(round(bTmp_(size(xuu2,1)/2)):round(bTmp_(size(xuu2,1)/2+1)), max(1,round(aTmp_(t))):round(aTmp_(t+1)));
        errorMatMidLeftSum_(t,:) = mean(errorMatMidLeft_(:));
        if errorMatMidLeftSum_(t) > 0.5
            extraSampleX(extraCnt,:) = aTmp_(t) + scaleSizeX/2;
            extraCnt = extraCnt + 1;
        end
    end
    
    leftXSAmple = unique([aTmp_(1:length(aTmp_)/2) extraSampleX']);
    diffXSample = diff(leftXSAmple);
    diffXSample_ = diffXSample(end:-1:1);
    rightXSAmple = aTmp_(length(aTmp_)/2+1)+[0 cumsum(diffXSample_)];
    dltSample = aTmp_(length(aTmp_)/2 +1)-aTmp_(1);
    % newXSAmple = [leftXSAmple leftXSAmple + dltSample];
    newXSAmple = [leftXSAmple rightXSAmple];
else
    if ret == 1
        
        if isempty(newXSAmple)
            if reverseMapping == 1
                thrSample = 0.02;
            else
                thrSample = 0.4;
            end
            try
                newXSAmple = reSample(xuu2,yuu2,xOrigMat,yOrigMat,aTmp_,bTmp_,imrotate(errorMat,0),scaleSizeX,thrSample);
                if 0 %reverseMapping ~= 1
                    newXSAmple = unique([newXSAmple mean(newXSAmple)]);
                end
                newXSAmple0 = newXSAmple;
                if reverseMapping == 0
                    lenThr = 46;
                else
                    lenThr = 46;
                end
                %                 while length(newXSAmple) >
                
                
                
                
                
            catch
                newXSAmple = aTmp_;
            end
        else
            %             thrSample = 0.02;
            if reverseMapping == 1
                thrSample = 0.02;
            else
                thrSample = 0.2;
            end
            try
                newYSAmple = reSample(yuu2',xuu2',xOrigMat',yOrigMat',bTmp_,aTmp_,imrotate(errorMat,90),scaleSizeY,thrSample);
            catch
                newYSAmple = bTmp_;
            end
        end
        return;
    end
end
if draw
    figure,subplot(1,2,1);plot(err);title('errAll');subplot(1,2,2);plot(sort(err));
end
if ~factory_speed
    [~,errY] = NormalizeVector(pixOrigAll(idWithin,2) - [ yRectMatUse(idWithin)+0]);
    [~,errX] = NormalizeVector(pixOrigAll(idWithin,1) - [ xRectMatUse(idWithin)+0]);
    
    if draw
        figure,subplot(1,2,1);plot(errY);title('errY');subplot(1,2,2);plot(sort(errY));
    end
    idShow = randperm(size(pixOrigAll(idWithin,:),1));
    idShow2 = randperm(size(pixOrigAll(idWithin2,:),1));
    showNum = 10000;
    if draw
        figure,imshow(zeros(nr,nc));hold on;plot(pixOrigAll(idWithin(idShow(1:showNum)),1),pixOrigAll(idWithin(idShow(1:showNum)),2),'or');plot(xRectMatUse(idWithin(idShow(1:showNum)))+dltErr,yRectMatUse(idWithin(idShow(1:showNum)))+dltErr,'.g')
    end
    
    try
        dx=pixOrigAll(idWithin(idShow(1:showNum)),1)-xRectMatUse(idWithin(idShow(1:showNum)));
        dy=pixOrigAll(idWithin(idShow(1:showNum)),2)-yRectMatUse(idWithin(idShow(1:showNum)));
        dx2=pixOrigAll(idWithin2(idShow2(1:showNum)),1)-xRectMatUse(idWithin2(idShow2(1:showNum)));
        dy2=pixOrigAll(idWithin2(idShow2(1:showNum)),2)-yRectMatUse(idWithin2(idShow2(1:showNum)));
    catch
        asgk = 1;
    end
end
% figure,Q=quiver(pixOrigAll(idWithin(idShow(1:showNum)),1)+0,pixOrigAll(idWithin(idShow(1:showNum)),2)+0,dx,dy,1);title(num2str([var(abs(dx)) var(abs(dy))]));
if draw
    figure,Q=quiver(pixOrigAll(idWithin(idShow(1:showNum)),1)+0,pixOrigAll(idWithin(idShow(1:showNum)),2)+0,dx,dy,1);title(num2str([sum(abs(err))]));
    figure,Q=quiver(pixOrigAll(idWithin2(idShow2(1:showNum)),1)+0,pixOrigAll(idWithin2(idShow2(1:showNum)),2)+0,dx2,dy2,1);title(num2str([sum(abs(err))]));
end
% kx= 10;
% ky = 10;%
% optFrac = [23 12 12 6 13 17 17 8 13 17 17 9 9];
% try
%     OptFixPt(paraDir,xOrigMat,yOrigMat,xRectMatRef,yRectMatRef,scaleSizeX,scaleSizeY,nc,nr,optFrac);
% catch
%     daghkj = 1;
% end


PixRectTmp = [xuuAllUse(:) yuuAllUse(:)];
if ndims(imgL) == 3
    imgL1 = uint8(zeros(nr,nc,3));
else
    imgL1 = uint8(zeros(nr,nc));
end
imgLR = imgL(:,:,1);
if ndims(imgL) == 3
    imgLG = imgL(:,:,2);
    imgLB = imgL(:,:,3);
end
imgL1R = zeros(nr,nc);
imgL1G = zeros(nr,nc);
imgL1B = zeros(nr,nc);

imgL1R_Bilinear = nan(nr,nc);
imgL1G_Bilinear = nan(nr,nc);
imgL1B_Bilinear = nan(nr,nc);

PixNew3_ = round([xRectMatUse(:) yRectMatUse(:)]);

idIn = PixNew3_(:,1) >= 1 & PixNew3_(:,1) <= nc & PixNew3_(:,2) >= 1 & PixNew3_(:,2) <= nr;
%     idDraw_ = idDraw(idIn);
PixNew3_ = PixNew3_(idIn,:);
PixRectTmp_ = PixRectTmp(idIn,:);
ind1 = sub2ind(size(imgL1),PixNew3_(:,2),PixNew3_(:,1));
ind2 = sub2ind(size(imgL1),PixRectTmp_(:,2),PixRectTmp_(:,1));
imgL1R(ind2) = imgLR(ind1);
if ndims(imgL) == 3
    imgL1G(ind2) = imgLG(ind1);
    imgL1B(ind2) = imgLB(ind1);
end
imgL1(:,:,1) = uint8(imgL1R);
if ndims(imgL) == 3
    imgL1(:,:,2) = uint8(imgL1G);
    imgL1(:,:,3) = uint8(imgL1B);
    
end

if useTable
    PixNew3_Bilinear = [xRectMatUse(:) yRectMatUse(:)];
else
    PixNew3_Bilinear = [xRectMatRef(:) yRectMatRef(:)];
end

idIn_Bilinear = PixNew3_Bilinear(:,1) >= 1 & PixNew3_Bilinear(:,1) <= nc-1 & PixNew3_Bilinear(:,2) >= 1 & PixNew3_Bilinear(:,2) <= nr-1;
PixNew3_Bilinear = PixNew3_Bilinear(idIn_Bilinear,:);
PixNew3_Bilinear_floor = floor(PixNew3_Bilinear);
PixNew3_Bilinear_frac = PixNew3_Bilinear - PixNew3_Bilinear_floor;

PixNew3_Bilinear_floor1 = PixNew3_Bilinear_floor;
PixNew3_Bilinear_floor2 = [PixNew3_Bilinear_floor(:,1)+1 PixNew3_Bilinear_floor(:,2)];
PixNew3_Bilinear_floor3 = [PixNew3_Bilinear_floor(:,1) PixNew3_Bilinear_floor(:,2)+1];
PixNew3_Bilinear_floor4 = [PixNew3_Bilinear_floor(:,1)+1 PixNew3_Bilinear_floor(:,2)+1];

indBilinear1 = sub2ind(size(imgL1),PixNew3_Bilinear_floor1(:,2),PixNew3_Bilinear_floor1(:,1));
indBilinear2 = sub2ind(size(imgL1),PixNew3_Bilinear_floor2(:,2),PixNew3_Bilinear_floor2(:,1));
indBilinear3 = sub2ind(size(imgL1),PixNew3_Bilinear_floor3(:,2),PixNew3_Bilinear_floor3(:,1));
indBilinear4 = sub2ind(size(imgL1),PixNew3_Bilinear_floor4(:,2),PixNew3_Bilinear_floor4(:,1));


PixRectTmp_Bilinear = PixRectTmp(idIn_Bilinear,:);
ind2_Bilinear = sub2ind(size(imgL1),PixRectTmp_Bilinear(:,2),PixRectTmp_Bilinear(:,1));



coeff1 = (1 - PixNew3_Bilinear_frac(:,2)).*(1 - PixNew3_Bilinear_frac(:,1));
coeff2 = (1 - PixNew3_Bilinear_frac(:,2)).*PixNew3_Bilinear_frac(:,1);
coeff3 = PixNew3_Bilinear_frac(:,2).*(1 - PixNew3_Bilinear_frac(:,1));
coeff4 = PixNew3_Bilinear_frac(:,2).*PixNew3_Bilinear_frac(:,1);


imgL1R_Bilinear(ind2_Bilinear) = coeff1.*double(imgLR(indBilinear1)) + coeff2.*double(imgLR(indBilinear2)) + coeff3.*double(imgLR(indBilinear3)) + coeff4.*double(imgLR(indBilinear4));
if ndims(imgL) == 3
    imgL1G_Bilinear(ind2_Bilinear) = coeff1.*double(imgLG(indBilinear1)) + coeff2.*double(imgLG(indBilinear2)) + coeff3.*double(imgLG(indBilinear3)) + coeff4.*double(imgLG(indBilinear4));
    imgL1B_Bilinear(ind2_Bilinear) = coeff1.*double(imgLB(indBilinear1)) + coeff2.*double(imgLB(indBilinear2)) + coeff3.*double(imgLB(indBilinear3)) + coeff4.*double(imgLB(indBilinear4));
end
imgL1_Bilinear(:,:,1) = (imgL1R_Bilinear);
if ndims(imgL) == 3
    imgL1_Bilinear(:,:,2) = (imgL1G_Bilinear);
    imgL1_Bilinear(:,:,3) = (imgL1B_Bilinear);
    
end
% imgL1_ref = imgL;
if useTable
    imgL1_ref(:,:,1) = (interp2(xuuAllUse,yuuAllUse,double(imgL(:,:,1)),xRectMatUse,yRectMatUse));
    imgL1_ref(:,:,2) = (interp2(xuuAllUse,yuuAllUse,double(imgL(:,:,2)),xRectMatUse,yRectMatUse));
    imgL1_ref(:,:,3) = (interp2(xuuAllUse,yuuAllUse,double(imgL(:,:,3)),xRectMatUse,yRectMatUse));
else
    imgL1_ref(:,:,1) = (interp2(xuuAllUse,yuuAllUse,double(imgL(:,:,1)),xRectMatRef,yRectMatRef));
    try
        imgL1_ref(:,:,2) = (interp2(xuuAllUse,yuuAllUse,double(imgL(:,:,2)),xRectMatRef,yRectMatRef));
        imgL1_ref(:,:,3) = (interp2(xuuAllUse,yuuAllUse,double(imgL(:,:,3)),xRectMatRef,yRectMatRef));
    catch
        asgkhj = 1;
    end
end

if 0
    try
        figure(121),imshow(uint8(rectImgL(300:end-200,200:end-200,3))-uint8(imgL1_ref(300:end-200,200:end-200,3)),[]);
    catch
        load('D:\bk_20180627_\nextvpu\algorithm\output\calibration\cbccalib\stereo_pairs\iEye\zed2\calib.mat')
        [rectParamL, rectParamR] = GetRectifyParam(stereoParam, size(imgL));
        [rectImgL] = RectifyOneImage(imgL, rectParamL);
        figure(121),imshow(uint8(rectImgL(300:end-200,200:end-200,1))-uint8(imgL1_ref(300:end-200,200:end-200,1)),[]);
    end
    
    figure,imshow(abs(double(imgL1_Bilinear(:,:,3))-double(imgL1_ref(:,:,3))),[])
    figure,imshowpair(uint8(imgL1_Bilinear), uint8(imgL1_ref));
end
if ~factory_speed
    if reverseMapping == 0
        if fishEyeModel == 0
            pixRect = Orig2Rect(pixOrigAll, Kinit, KK_new, R,k);
        else
            pixRect = Orig2RectFishEye(pixOrigAll, Kinit, KK_new, R,k,oCamModel);
        end
    else
        if fishEyeModel == 0
            pixRect = remapRect(pixOrigAll', KK_new,Kinit,k,R);
        else
            pixRect = remapRectFishEye(pixOrigAll', KK_new,Kinit,k,R,oCamModel);
        end
        
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
    
    if fishEyeModel == 0
        pixRectAll1 = Orig2Rect([xuuAllUse(:) yuuAllUse(:)], Kinit, KK_new, R,k);
    else
        pixRectAll1 = Orig2RectFishEye([xuuAllUse(:) yuuAllUse(:)], Kinit, KK_new, R,k,oCamModel);
    end
    [~, RectOrigErr1] = NormalizeVector(pixRectAll1 - [xuuAllUse(:) yuuAllUse(:)]);
    [~, RectOrigErrY1] = NormalizeVector(pixRectAll1(:,2) - [yuuAllUse(:)]);
    if fishEyeModel == 0
        pixRectAll2 = remapRect([xuuAllUse(:) yuuAllUse(:)]', KK_new,Kinit,  k,R);
    else
        pixRectAll2 = remapRectFishEye([xuuAllUse(:) yuuAllUse(:)]', KK_new,Kinit,  k,R,oCamModel);
    end
    [~, RectOrigErr2] = NormalizeVector(pixRectAll2 - [xuuAllUse(:) yuuAllUse(:)]);
    [~, RectOrigErrY2] = NormalizeVector(pixRectAll2(:,2) - [yuuAllUse(:)]);
    if 0
        figure,plot(RectOrigErr1-RectOrigErr2)
        figure,plot(RectOrigErrY1-RectOrigErrY2)
    end
end



xuu2_in = xuu2(2:end-1,2:end-1);
yuu2_in = yuu2(2:end-1,2:end-1);
ceilXY = [ceil(xuu2_in(:)),ceil(yuu2_in(:))]; idInCeil = find(ceilXY(:,1) >= 1 & ceilXY(:,1) <= nc & ceilXY(:,2) >= 1 & ceilXY(:,2) <= nr);
floorXY = [floor(xuu2_in(:)),floor(yuu2_in(:))]; idInFloor = find(floorXY(:,1) >= 1 & floorXY(:,1) <= nc & floorXY(:,2) >= 1 & floorXY(:,2) <= nr);

ind1 = sub2ind([nr nc], ceilXY(idInCeil,2),ceilXY(idInCeil,1));
ind2 = sub2ind([nr nc], floorXY(idInFloor,2),floorXY(idInFloor,1));
ofstMatX = zeros(nr,nc);
ofstMatY = zeros(nr,nc);
ofstMatX([ind1;ind2]) = 1;
ofstMat = ~isnan(imgL1_Bilinear(:,:,1)) & ofstMatX;
[vldY, vldX] = ind2sub([nr nc], find(ofstMat(:) == 1));
D = pdist2([vldX vldY],[xuu2(:) yuu2(:)],'euclidean');
[minD, idD] = min(D');
vldMat = zeros(size(xuu2));
vldMat(idD) = 1;
vldMat0 = vldMat;
se = strel('square',[3]);
vldMat = imdilate(vldMat,se);
if draw
    figure,imshow(xOrigMat.*vldMat - xuu2.*vldMat, [])
    figure,imshow(yOrigMat.*vldMat - yuu2.*vldMat, [])
end
% idVldX = (ismember(ceil(xuu2(:)),vldX) | ismember(floor(xuu2(:)),vldX));
% idVldY = (ismember(ceil(yuu2(:)),vldY) | ismember(floor(yuu2(:)),vldY));
% idVld = find(idVldX & idVldY);
% vldMat = zeros(size(xuu2));
% vldMat(idVld) = 1;

% polyCnr = FindPolyon(pt0)；

idInSrc = find(xRectMatRef(:) > 1 & xRectMatRef(:) < nc & yRectMatRef(:) > 1 & yRectMatRef(:) < nr);
vldSrcMat = zeros([nr,nc]);
vldSrcMat(idInSrc) = 1;
% se = strel('disk',max(stepXY));
% BW = imdilate(ofstMat,se);
dltLutX = (xuu2-xOrigMat);
dltLutY = (yuu2-yOrigMat);
imValidX = dltLutX.*(~vldMat);
imValidY = dltLutY.*(~vldMat);


if reverseMapping == 0
    dltLutXX1 = MakeOfst1(imValidX,dltLutX,ones(size(vldMat)));%figure,imshow(diff(dltLutXX1')',[]) % col diff
    dltLutXX2 = MakeOfst2(imValidX,dltLutX,ones(size(vldMat)));%figure,imshow(diff(dltLutXX2),[]) % row diff
    dltLutYY1 = MakeOfst1(imValidY,dltLutY,ones(size(vldMat)));%figure,imshow(diff(dltLutYY1')',[]) % col diff
    dltLutYY2 = MakeOfst2(imValidY,dltLutY,ones(size(vldMat)));%figure,imshow(diff(dltLutYY2),[]) % row diff
    
    
else
    dltLutXX1 = MakeOfst1(imValidX,dltLutX,vldMat);%figure,imshow(diff(dltLutXX1')',[]) % col diff
    dltLutXX2 = MakeOfst2(imValidX,dltLutX,vldMat);%figure,imshow(diff(dltLutXX2),[]) % row diff
    dltLutYY1 = MakeOfst1(imValidY,dltLutY,vldMat);%figure,imshow(diff(dltLutYY1')',[]) % col diff
    dltLutYY2 = MakeOfst2(imValidY,dltLutY,vldMat);%figure,imshow(diff(dltLutYY2),[]) % row diff
end
if draw
    figure,subplot(2,2,1);imshow(diff(dltLutXX1')',[]);title('x col diff');
    subplot(2,2,2);imshow(diff(dltLutXX2),[]);title('x row diff');
    subplot(2,2,3);imshow(diff(dltLutYY1')',[]);title('y col diff');
    subplot(2,2,4);imshow(diff(dltLutYY2),[]);title('y row diff');
end

if reverseMapping == 0
    thr = 2^7;
else
    thr = 2^8;
end

aa = cumsum([dltLutXX2(1,:);diff(dltLutXX2)]);
if 1
    aa((aa) > thr) = (thr-1)*1; %rand(sum(sum((aa) > thr)),1);
    aa((aa) < -thr) = (-thr+1)*1; %rand(sum(sum((aa) <-thr)),1);
else
    aa((aa) > thr) = (thr-20*(rand(sum(sum((aa) > thr)),1)))*1; %rand(sum(sum((aa) > thr)),1);
    aa((aa) < -thr) = (-thr+20*(rand(sum(sum((aa) < -thr)),1)))*1; %rand(sum(sum((aa) <-thr)),1);
end
bb = xuu2 - aa;

if draw
    figure,imshow(abs(bb - xOrigMat) < 0.001,[]);
end
cc = cumsum([dltLutYY2(1,:);diff(dltLutYY2)]);
if 1
    cc((cc) > thr) = (thr-1)*1; %rand(sum(sum((cc) > thr)),1);
    cc((cc) < -thr) = (-thr+1)*1; %rand(sum(sum((cc) < -thr)),1);
else
    cc((cc) > thr) = (thr-20*rand(sum(sum((cc) > thr)),1))*1; %rand(sum(sum((cc) > thr)),1);
    cc((cc) < -thr) = (-thr+20*rand(sum(sum((cc) < -thr)),1))*1; %rand(sum(sum((cc) < -thr)),1);
    
end
dd = yuu2 - cc;
if draw
    figure,imshow(abs(dd - yOrigMat) < 0.001,[]);
end
  
% [aa1,cc1] = RecoverLut('C:\Users\ROGER\Desktop\new 1.txt',newXSAmple, newYSAmple);
[xRectMat1, yRectMat1, LutVecCharX, LutVecCharY, SplitIndX, SplitIndY,lutVecX,lutVecY,nameMatX, nameMatY] = BiLinearInterp_(reverseMapping, inputDir,xuu2, yuu2, bb,dd,xRectMatRef,yRectMatRef,scaleSizeX,scaleSizeY,nc,nr,doRounding,optFrac, speedUp); %bilinear2x2Func

imgOut = (interp2(xuuAllUse,yuuAllUse,double(imgL(:,:,1)),xRectMat1,yRectMat1));


return;

imValidXUp = imValidX(1:round(size(imValidX,1)/2),:);
imValidXDown = imValidX(round(size(imValidX,1)/2)+1:end,:);


for ii = 1 : size(imValidXUp,2)
    id = find(imValidXUp(:,ii) ~= 0);
    if ~isempty(id)
        imValidXUp(id,ii) = imValidXUp(id(end),ii);
    end
end
for ii = 1 : size(imValidXDown,2)
    id = find(imValidXDown(:,ii) ~= 0);
    if ~isempty(id)
        imValidXDown(id,ii) = imValidXDown(id(1),ii);
    end
end
imValidX = [imValidXUp; imValidXDown];
dltLutX(~vldMat) = imValidX(imValidX~=0);
figure,imshow(diff(dltLutX),[]) % row diff



midMat = size(imValidY,2)/2;
for ii = 1 : size(imValidY,1)
    id = find(imValidY(ii,1:midMat) ~= 0);
    if ~isempty(id)
        imValidY(ii,id) = imValidY(ii,id(end));
    end
end
dltLutY(~vldMat) = imValidY(imValidY~=0);
figure,imshow(diff(dltLutY')',[]) % col diff



imValidYUp = imValidY(1:round(size(imValidY,1)/2),:);
imValidYDown = imValidY(round(size(imValidY,1)/2)+1:end,:);

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
imValidY = [imValidYUp; imValidYDown];
dltLutY(~vldMat) = imValidY(imValidY~=0);
figure,imshow(diff(dltLutY),[]) % row diff

aa = cumsum([dltLutY(1,:);diff(dltLutY)]);
bb = yuu2 - aa;

end
%
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
% [xuu,yuu] = meshgrid([1:NewColumns],[1:NewRows]);
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
% B=reshape(bicubic4x4Fast([Temp1,Temp2,Temp3,Temp4],Di,afact),NewRows,NewColumns);
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
function B = bicubic4x4FastUseFixPt(A, kx,ky)
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
Temp1=bicubic4x4FastFixPt([A(ind1),A(ind2),A(ind3),A(ind4)],Dj,afact);
Temp2=bicubic4x4FastFixPt([A(ind5),A(ind6),A(ind7),A(ind8)],Dj,afact);
Temp3=bicubic4x4FastFixPt([A(ind9),A(ind10),A(ind11),A(ind12)],Dj,afact);
Temp4=bicubic4x4FastFixPt([A(ind13),A(ind14),A(ind15),A(ind16)],Dj,afact);
%         B(i,:)=bicubic4x4Fast([Temp1,Temp2,Temp3,Temp4],Di,afact);
B=reshape(bicubic4x4FastFixPt([Temp1,Temp2,Temp3,Temp4],Di,afact),NewColumns,NewRows)';
end

function [X] = bicubic4x4FastFixPt(v,fact,afact)

X = zeros(size(v,1),1);
if fact==0
    X = v(2);
    return
end
Y = [1 0 1 2];
Z = [1 1 -1 -1];
Y1 = Y(1);Y2=Y(2);Y3 = Y(3);Y4 = Y(4);
Z1 = Z(1);Z2=Z(2);Z3 =Z(3);Z4 = Z(4);
v1 = v(:,1);v2=v(:,2);v3 =v(:,3);v4 = v(:,4);
% a = -0.5;
a = afact;
A=(2+a);
B=-(3+a);
C=1;
D=a;
E=-5*a;
F=8*a;
G=-4*a;

X = func1(X, fact, Y1, Z1, E, D, F, G, v1);
X = func2(X, fact, Y2, Z2, B, A, v2);
X = func2(X, fact, Y3, Z3, B, A, v3);
X = func1(X, fact, Y4, Z4, E, D, F, G, v4);

sdvb = 1;
% F1 = Y1+fact*Z1;
% coeff1 = F1;
% coeff7 = (E + F1.*D);
% coeff8 = (F + F1.*coeff7);
% coeff9 = (G + F1.*coeff8);
% coeff10 = v1.*coeff9;
% coeff11 = X + coeff10;
% X = coeff11;


% F2 = Y2+fact*Z2;
% coeff12 = F2;
% coeff13 = (B + F2*A);
% coeff14 = F2.^2;
% coeff15 = (1 + coeff14.*coeff13);
% coeff16 = v2.*coeff15;
% coeff17 = X + coeff16;
% X = coeff17;


% F3 = Y3+fact*Z3;
% coeff23 = F3;
% coeff24 = (B + F3*A);
% coeff25 = F3.^2;
% coeff26 = (1 + coeff25.*coeff24);
% coeff27 = v3.*coeff26;
% coeff28 = X + coeff27;
% X = coeff28;


% F4 = Y4+fact*Z4;
% coeff34 = F4;
% coeff40 = (E + F4.*D);
% coeff41 = (F + F4.*coeff40);
% coeff42 = (G + F4.*coeff41);
% coeff43 = v4.*coeff42;
% coeff44 = X + coeff43;
% X = coeff44;

end
function X = func1(X, fact, Y1, Z1, E, D, F, G, v1)
F1 = Y1+fact*Z1;
coeff1 = F1;
coeff7 = (E + F1.*D);
coeff8 = (F + F1.*coeff7);
coeff9 = (G + F1.*coeff8);
coeff10 = v1.*coeff9;
coeff11 = X + coeff10;
X = coeff11;
end
function X = func2(X, fact, Y2, Z2, B, A, v2)
F2 = Y2+fact*Z2;
coeff12 = F2;
coeff13 = (B + F2*A);
coeff14 = F2.^2;
coeff15 = (1 + coeff14.*coeff13);
coeff16 = v2.*coeff15;
coeff17 = X + coeff16;
X = coeff17;
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
function pixDist = remapRectFishEye(pixRect, KRect, KUndist,distCoeff, RL,oCamModel)

alpha = 0;

rays = inv(KRect)*pextend(pixRect);


% Rotation: (or affine transformation):

rays2 = RL'*rays;

x = [rays2(1,:)./rays2(3,:);rays2(2,:)./rays2(3,:)];
if 0
    pixxDistort = pixDistort([2 1],:);
    % M2 = cam2world_fast(pixxDistort, calib_data.ocam_model);
    M2 = cam2world(pixxDistort, oCamModel);
    M2 = [M2(1,:)./M2(3,:);M2(2,:)./M2(3,:);ones(1,size(pixxDistort,2))];
    pixxUndist = intrMat_same*[-M2(2,:); -M2(1,:); M2(3,:)];
    pixUndist3D = [-M2(2,:); -M2(1,:); M2(3,:)];
    pixUndist = [pixxUndist(1,:)./pixxUndist(3,:); pixxUndist(2,:)./pixxUndist(3,:)];   %;ones(1,size(pixDistort,2))];
else
    % %     MM = inv(intrMat_same)*[pixUnDistort; ones(1,size(pixUnDistort,2))];
    MM = rays2;
    % % pixxDistort = pixDistort([2 1],:);
    % % % M2 = cam2world_fast(pixxDistort, calib_data.ocam_model);
    % % M2 = cam2world(pixxDistort, oCamModel);
    % % M2 = [M2(1,:)./M2(3,:);M2(2,:)./M2(3,:);ones(1,size(pixxDistort,2))];
    % % pixxUndist = intrMat_same*[-M2(2,:); -M2(1,:); M2(3,:)];
    % % pixUndist3D = [-M2(2,:); -M2(1,:); M2(3,:)];
    % % pixUndist = [pixxUndist(1,:)./pixxUndist(3,:); pixxUndist(2,:)./pixxUndist(3,:)];   %;ones(1,size(pixDistort,2))];
    % %
    % % MM = [-M2(2,:); -M2(1,:); M2(3,:)];
    MM_ = [-MM(2,:);-MM(1,:);MM(3,:)];
    [M4,~] = NormalizeVector(MM_');
    try
        % %                 a
        M33 = world2cam_fast(-M4',oCamModel);
    catch
        M33 = world2cam(-M4',oCamModel);
    end
    M33_ = M33([2 1],:);
    pixDistorted = M33_';
    pixDist = pixDistorted;
end

% Add distortion:
% xd = apply_distortion(x,distCoeff);
%
%
% % Reconvert in pixels:
%
% px2_ = KUndist(1,1)*(xd(1,:) + alpha*xd(2,:)) + KUndist(1,3);
% py2_ = KUndist(2,2)*xd(2,:) + KUndist(2,3);
% pixDist = [px2_;py2_]';

end
function pixRect = Orig2RectFishEye(pix, KOrig, KRect, R,kc,oCamModel)

% % % [pixUndist] = normalize_pixel(pix',[KOrig(1,1);KOrig(2,2)],[KOrig(1,3);KOrig(2,3)],kc,0);

pixxDistort = pix(:,[2 1])';
% M2 = cam2world_fast(pixxDistort, calib_data.ocam_model);
M2 = cam2world(pixxDistort, oCamModel);
M2 = [M2(1,:)./M2(3,:);M2(2,:)./M2(3,:);ones(1,size(pixxDistort,2))];
% pixxUndist = KOrig*[-M2(2,:); -M2(1,:); M2(3,:)];
pixUndist3D = [-M2(2,:); -M2(1,:); M2(3,:)];
% pixUndist = [pixxUndist(1,:)./pixxUndist(3,:); pixxUndist(2,:)./pixxUndist(3,:)];

pixUndistR = R*(pixUndist3D);
pixRect = pflat(KRect*pixUndistR);
pixRect = pixRect(1:2,:)';



end

function pixNew = plotUndistortionFishEye(nc, nr, Kinit, kk_new,kc,R,skipNum,oCamModel)


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
if 0
    xd=apply_distortion(x,kc);
    px2=Kinit(1,1)*xd(1,:)+Kinit(1,3);
    py2=Kinit(2,2)*xd(2,:)+Kinit(2,3);
else
    MM = rays;
    % % pixxDistort = pixDistort([2 1],:);
    % % % M2 = cam2world_fast(pixxDistort, calib_data.ocam_model);
    % % M2 = cam2world(pixxDistort, oCamModel);
    % % M2 = [M2(1,:)./M2(3,:);M2(2,:)./M2(3,:);ones(1,size(pixxDistort,2))];
    % % pixxUndist = intrMat_same*[-M2(2,:); -M2(1,:); M2(3,:)];
    % % pixUndist3D = [-M2(2,:); -M2(1,:); M2(3,:)];
    % % pixUndist = [pixxUndist(1,:)./pixxUndist(3,:); pixxUndist(2,:)./pixxUndist(3,:)];   %;ones(1,size(pixDistort,2))];
    % %
    % % MM = [-M2(2,:); -M2(1,:); M2(3,:)];
    MM_ = [-MM(2,:);-MM(1,:);MM(3,:)];
    [M4,~] = NormalizeVector(MM_');
    try
        M33 = world2cam_fast(-M4',oCamModel);
    catch
        M33 = world2cam(-M4',oCamModel);
    end
    M33_ = M33([2 1],:);
    pixDistorted = M33_';
    pixDist = pixDistorted;
    px2 = pixDist(:,1)';
    py2 = pixDist(:,2)';
end



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

function [xRectMat, yRectMat, LutVecCharX, LutVecCharY,SplitIndX,SplitIndY,lutVecX,lutVecY] = BiCubicInterp(inputDir,xOrigMat,yOrigMat,xRectMatRef,yRectMatRef,scaleSizeX,scaleSizeY,nc,nr,doRounding,optFrac)

optFrac = round(optFrac);
optFracCell = {[1:2];[3:4];[7:8];[9 10 13 14 26 34 41 48];[24 57];[27 28 49 50];[29 30 51 52];[31 32 53 54];[33 55];[35 36 42 43];[37 44];[38 39 45 46];[40 47];[56]};
% intLen = [0;0;0;0;13;13;6;6;6;6;6;6;0;0;1;1;2;2;2;0;2;3;2;13;1; ...        % 1-25
%     1;0;1;2;1;2;0;10; ...                                                  % 26-33
%     0;1;2;0;0;0;13; ...                                                    % 34-40
%     0;1;2;0;0;0;13; ...                                                    % 41-47
%     1;0;1;2;1;2;0;10; ...                                                  % 48-55
%     13; ...                                                                % X 56
%     13]+1;                                                                 % v1v2v2v4 57



intLen = [0;0;0;0;13;13;7;7;7;7;7;7;0;0;1;1;2;2;2;0;2;3;2;13;1; ...        % 1-25
    1;0;1;2;1;2;0;10; ...                                                  % 26-33
    0;1;2;0;0;0;13; ...                                                    % 34-40
    0;1;2;0;0;0;13; ...                                                    % 41-47
    1;0;1;2;1;2;0;10; ...                                                  % 48-55
    13; ...                                                                % X 56
    13]+1;                                                                 % v1v2v2v4 57



fixFrac = [0;0;0;0;0;0;0;1;1;1;1;0;0;0];

[xRectMat,LutVecCharX,SplitIndX,lutVecX] = bicubic4x4(xOrigMat,scaleSizeX,scaleSizeY,nc,nr,intLen,fixFrac,optFracCell,doRounding,optFrac);
[yRectMat,LutVecCharY,SplitIndY,lutVecY] = bicubic4x4(yOrigMat,scaleSizeX,scaleSizeY,nc,nr,intLen,fixFrac,optFracCell,doRounding,optFrac);
[~,errr] = NormalizeVector([xRectMat(:) yRectMat(:)] - [xRectMatRef(:) yRectMatRef(:)]);

figure,plot(errr);title(num2str(optFrac));
saveas(gcf,fullfile(inputDir,sprintf('img_%05d.png',length(dir(fullfile(inputDir,'img_*.png')))+1)));

end

function [b, LutVecChar, SplitInd,LutVec] = bicubic4x4(AA, kx,ky,nc,nr,intLen,fixFrac,optFracCell,doRounding,optFrac)

optFrac = [optFrac(1:3) max(optFrac(2:3)) optFrac(4:end)];
tmp = cell2mat(optFracCell')';
fixFracId = setdiff([1:length(intLen)],cell2mat(optFracCell')')';
wordInfo(:,1) = intLen;
for i = 1 : length(optFracCell)
    wordInfo(optFracCell{i},2) = optFrac(i);
end
wordInfo(fixFracId,2) = fixFrac;
wordInfo(fixFracId,3) = 0;
wordInfo(tmp,3) = 1;

[m,n] = size(AA);
NewRows = size(AA,1)*ky;
NewColumns = size(AA,2)*kx;
aa = [kx,0,0;0,ky,0;0.5*(1-kx),0.5*(1-ky),1];
invaa = inv(aa);

[xuuu,yuuu] = meshgrid([1:NewColumns],[1:NewRows]);

xuu = xuuu((NewRows-nr)/2+1:end-(NewRows-nr)/2,(NewColumns-nc)/2+1:end-(NewColumns-nc)/2);
yuu = yuuu((NewRows-nr)/2+1:end-(NewRows-nr)/2,(NewColumns-nc)/2+1:end-(NewColumns-nc)/2);



LutVecChar = [];
LutVecNum = [];




if doRounding
    a1 = Num2Fix(invaa(1,1),wordInfo(1,1),wordInfo(1,2));
    a3 = Num2Fix(invaa(2,2),wordInfo(2,1),wordInfo(2,2));
    a2 = Num2Fix(invaa(3,1),wordInfo(3,1),wordInfo(3,2));
    a4 = Num2Fix(invaa(3,2),wordInfo(4,1),wordInfo(4,2));
    Xuu = xuu(:);
    Yuu = yuu(:);
    a1xuu = Num2Fix(a1.*Xuu,wordInfo(7,1),wordInfo(7,2));% table size
    a3yuu = Num2Fix(a3.*Yuu,wordInfo(8,1),wordInfo(8,2)); % table size
    TempX = Num2Fix(a1xuu + a2,wordInfo(9,1),wordInfo(9,2));
    TempY = Num2Fix(a3yuu + a4,wordInfo(10,1),wordInfo(10,2));
    Y = floor(TempY);
    X = floor(TempX);
    Di =  Num2Fix(TempY - Y,wordInfo(13,1),wordInfo(13,2));% di = yfract
    Dj =  Num2Fix(TempX - X,wordInfo(14,1),wordInfo(14,2));% dj = xfract
    Y1 = 1;
    Y3 = 1;
    Y4 = 2;
    A = 1.5;
    B = -2.5;
    D = -0.5;
    E = 2.5;
    F = -4;
    G = 2;
    AA = Num2Fix(AA,wordInfo(24,1),wordInfo(24,2));
    H = 1;
    
    AAAA = AA';
    dropInt = 4;
    
    if dropInt == 0
        LutVec = [size(AA,2);size(AA,1);nc;nr;kx;ky;NewColumns;NewRows;(NewColumns-nc)/2;(NewRows-nr)/2;a1*2^(wordInfo(1,2));a2*2^wordInfo(3,2);a3*2^wordInfo(2,2);a4*2^wordInfo(4,2);AAAA(:).*2^wordInfo(24,2)];
        intPart = [6;6;13;12;8;8;13;12;10;10;23;12;23;12;sum(wordInfo(24,1:2))];
        fracPart = [0;0;0;0;0;0;0;0;0;0;wordInfo(1,2);wordInfo(3,2);wordInfo(2,2);wordInfo(4,2)];
    else
        dropInt = 4;
        LutVec = [size(AA,2);size(AA,1);nc;nr;kx;ky;a1*2^(wordInfo(1,2));a2*2^wordInfo(3,2);a3*2^wordInfo(2,2);a4*2^wordInfo(4,2);AAAA(:).*2^wordInfo(24,2)];
        intPart = [6;6;13;12;8;8;23;12;23;12;sum(wordInfo(24,1:2))];
        fracPart = [0;0;0;0;0;0;wordInfo(1,2);wordInfo(3,2);wordInfo(2,2);wordInfo(4,2)];
    end
    
    %     supposeLen = sum(intPart(1:14)) + intPart(15)*length(AAAA(:));
    supposeLen = sum(intPart(1:14-dropInt)) + intPart(15-dropInt)*length(AAAA(:));
    
    %     LutVecChar = [];
    for u = 1 : length(LutVec)
        if u <= 10-dropInt
            numstr = LutVec(u);
            [fixPt, abin] = Num2Fix_unsigned(LutVec(u), intPart(u),0);
            ttmp = abin.bin;
            bin_x = dec2bin(LutVec(u),intPart(u));
            if ~strcmp(ttmp,bin_x)
                asvnlkl = 1;
            end
            LutVecChar = [LutVecChar '' abin.bin];
            LutVecNum = [LutVecNum '' abin.bin];
            sbkj = 1;
            
        elseif u <= 14-dropInt
            
            [fixPt, abin] = Num2Fix_unsigned(LutVec(u), intPart(u),0);
            ttmp = abin.bin;
            bin_x = dec2bin(LutVec(u),intPart(u));
            if ~strcmp(ttmp,bin_x)
                asvnlkl = 1;
            end
            LutVecChar = [LutVecChar '' abin.bin];
            sbkj = 1;
        else
            if u == 14 - dropInt + 1
                SplitInd = length(LutVecChar);
            end
            
            if LutVec(u) >= 0
                %             [fixPt, abin] = Num2Fix(num, wordInfo(24,1),wordInfo(24,2));
                %              LutVecChar = [LutVecChar '' abin.bin];
                %              sbkj = 1;
                [fixPt, abin] = Num2Fix(LutVec(u), intPart(15-dropInt),0);
                ttmp = abin.bin;
                bin_x = dec2bin(LutVec(u),intPart(15-dropInt));
                if ~strcmp(ttmp,bin_x)
                    asvnlkl = 1;
                end
                LutVecChar = [LutVecChar '' bin_x];
                
            else
                bin_x = dec2bin(2^intPart(15-dropInt) + LutVec(u),intPart(15-dropInt));
                LutVecChar = [LutVecChar '' bin_x];
            end
        end
        
    end
    asvjkjb = 1;
    if length(LutVecChar) == supposeLen
        LutVecData = str2num(LutVecChar);
        ajkvb = 1;
    end
    
else
    a1 = (invaa(1,1));
    a3 = (invaa(2,2));
    a2 = (invaa(3,1));
    a4 = (invaa(3,2));
    Xuu = xuu(:);
    Yuu = yuu(:);
    a1xuu = (a1.*Xuu);
    a3yuu = (a3.*Yuu);
    TempX = (a1xuu + a2);
    TempY = (a3yuu + a4);
    Y = floor(TempY);
    X = floor(TempX);
    Di =  (TempY - Y);
    Dj =  (TempX - X);
    Y1 = 1;
    Y3 = 1;
    Y4 = 2;
    A = 1.5;
    B = -2.5;
    D = -0.5;
    E = 2.5;
    F = -4;
    G = 2;
    AA = (AA);
    H = 1;
end


AX = max(X-1,1);
BX = max(X,1);
CX = min(X+1,n);
DX = min(X+2,n);
AY = max(Y-1,1);
BY = max(Y,1);
CY = min(Y+1,m);
DY = min(Y+2,m);
try
    ind1 = sub2ind(size(AA),AY,AX);
catch
    sdghk = 1;
end
ind2 = sub2ind(size(AA),AY,BX);
ind3 = sub2ind(size(AA),AY,CX);
ind4 = sub2ind(size(AA),AY,DX);
ind5 = sub2ind(size(AA),BY,AX);
ind6 = sub2ind(size(AA),BY,BX);
ind7 = sub2ind(size(AA),BY,CX);
ind8 = sub2ind(size(AA),BY,DX);
ind9 = sub2ind(size(AA),CY,AX);
ind10 = sub2ind(size(AA),CY,BX);
ind11 = sub2ind(size(AA),CY,CX);
ind12 = sub2ind(size(AA),CY,DX);
ind13 = sub2ind(size(AA),DY,AX);
ind14 = sub2ind(size(AA),DY,BX);
ind15 = sub2ind(size(AA),DY,CX);
ind16 = sub2ind(size(AA),DY,DX);

V11 = AA(ind1); V12 = AA(ind2); V13 = AA(ind3); V14 = AA(ind4);
V21 = AA(ind5); V22 = AA(ind6); V23 = AA(ind7); V24 = AA(ind8);
V31 = AA(ind9); V32 = AA(ind10); V33 = AA(ind11); V34 = AA(ind12);
V41 = AA(ind13); V42 = AA(ind14); V43 = AA(ind15); V44 = AA(ind16);

variable{1,1} = a1; variable{2,1} = a3;
variable{3,1} = a2; variable{4,1} = a4;
variable{5,1} = Xuu; variable{6,1} = Yuu;
variable{7,1} = a1xuu; variable{8,1} = a3yuu;
variable{9,1} = TempX; variable{10,1} = TempY;
variable{11,1} = Y; variable{12,1} = X;
variable{13,1} = Di; variable{14,1} = Dj;
variable{15,1} = Y1;
variable{16,1} = Y3;
variable{17,1} = Y4;
variable{18,1} = A;
variable{19,1} = B;
variable{20,1} = D;
variable{21,1} = E;
variable{22,1} = F;
variable{23,1} = G;
variable{24,1} = AA;
variable{25,1} = H;

inherited = wordInfo([26:56 57 57 57 57 13 13 13 13 15 20 21 22 23 18 19 25 16 18 19 25 17 20 21 22 23],:);

whole = [wordInfo;inherited];

Temp1 = bicubic4x4Func(V11,V12,V13,V14,Dj,Y1,Y3,Y4,A,B,D,E,F,G,H,inherited,doRounding);
Temp2 = bicubic4x4Func(V21,V22,V23,V24,Dj,Y1,Y3,Y4,A,B,D,E,F,G,H,inherited,doRounding);
Temp3 = bicubic4x4Func(V31,V32,V33,V34,Dj,Y1,Y3,Y4,A,B,D,E,F,G,H,inherited,doRounding);
Temp4 = bicubic4x4Func(V41,V42,V43,V44,Dj,Y1,Y3,Y4,A,B,D,E,F,G,H,inherited,doRounding);

if 0
    figure,plot(sort(diff(sort([num2fixpt(([Temp1]),sfix(17),2^-4)]))));
    figure,plot((diff(sort([num2fixpt(([Temp1]),sfix(17),2^-4)]))));
    figure,plot(diff((num2fixpt(sort([Temp1]),sfix(17),2^-4)))-diff(sort([Temp1])))
    figure,plot(sort(abs(diff((num2fixpt(sort([Temp1]),sfix(17),2^-4)))-diff(sort([Temp1])))))
    figure,plot(((num2fixpt(([Temp1]),sfix(17),2^-5)))-(([Temp1])))
end

[bb] = bicubic4x4Func(Temp1,Temp2,Temp3,Temp4,Di,Y1,Y3,Y4,A,B,D,E,F,G,H,inherited, doRounding);
b = reshape(bb,nr,nc);



end
function X = bicubic4x4Func(v1,v2,v3,v4,fact,Y1,Y3,Y4,A,B,D,E,F,G,H,wordInfo,doRounding)



X = zeros(size(v1,1),1);
if fact == 0
    X = v2;
    return
end

% r(x) = (a + 2)|x|^3 - (a + 3)|x|^2         +  1 , 0 <= |x| < 1
% r(x) =       a|x|^3 -      5a|x|^2 + 8a|x| - 4a , 1 <= |x| < 2
% where x is {fact,fact+1,abs(fact-1),abs(fact-2)}
% then r(x) will contain the interpolation ratio which will be multiplied
% by the point value.
% The sum of r(fact+1)*v1+r(fact)*v2+r(abs(fact-1))*v3+r(abs(fact-2))*v4
% will give the interpolated value.
% The value a is taken as -0.5 as per Matlab.



if doRounding
    F1 =  Num2Fix(Y1 + fact,wordInfo(1,1),wordInfo(1,2));                  % N0.26  1  (1, 2)
    F1D = Num2Fix(F1.*D,wordInfo(2,1),wordInfo(2,2));                      % No.27  2  (-1, -0.5)
    EF1D = Num2Fix(E + F1D,wordInfo(3,1),wordInfo(3,2));                   % No.28  3  (1.5, 2)
    F1EF1D = Num2Fix(F1.*EF1D,wordInfo(4,1),wordInfo(4,2));                % No.29  4  (2, 3)
    FF1EF1D = Num2Fix(F + F1EF1D,wordInfo(5,1),wordInfo(5,2));             % No.30  5  (-2, -1)
    F1FF1EF1D =  Num2Fix(F1.*FF1EF1D,wordInfo(6,1),wordInfo(6,2));         % No.31  6  (-2.07, -2)
    GF1FF1EF1D =  Num2Fix(G + F1FF1EF1D,wordInfo(7,1),wordInfo(7,2));      % No.32  7  (-0.075, 0)
    v1GF1FF1EF1D = Num2Fix(v1.*GF1FF1EF1D,wordInfo(8,1),wordInfo(8,2));    % No.33  8  (-240, 11)
    X = Num2Fix(X + v1GF1FF1EF1D,wordInfo(31,1),wordInfo(31,2));           % No.56  31 (128, 4600)
    
    F2 = Num2Fix(fact,wordInfo(9,1),wordInfo(9,2));                        % No.34  9  (0, 1)
    F2A = Num2Fix(F2*A,wordInfo(10,1),wordInfo(10,2));                     % No.35  10 (0, 1.5)
    BF2A = Num2Fix(B + F2A,wordInfo(11,1),wordInfo(11,2));                 % No.36  11 (-2.5, -1)
    F2F2 = Num2Fix(F2.^2,wordInfo(12,1),wordInfo(12,2));                   % No.37  12 (0, 1)
    F2F2BF2A = Num2Fix(F2F2.*BF2A,wordInfo(13,1),wordInfo(13,2));          % No.38  13 (-1, 0)
    HF2F2BF2A = Num2Fix(H + F2F2BF2A,wordInfo(14,1),wordInfo(14,2));       % No.39  14 (0, 1)
    v2HF2F2BF2A = Num2Fix(v2.*HF2F2BF2A,wordInfo(15,1),wordInfo(15,2));    % No.40  15 (-38, 4600)
    X = Num2Fix(X + v2HF2F2BF2A,wordInfo(31,1),wordInfo(31,2));            % No.56  31 (128, 4600)
    
    F3 = Num2Fix(Y3 - fact,wordInfo(16,1),wordInfo(16,2));                 % No.41  16 (0, 1)
    F3A = Num2Fix(F3*A,wordInfo(17,1),wordInfo(17,2));                     % No.42  17 (0, 1.5)
    BF3A = Num2Fix(B + F3A,wordInfo(18,1),wordInfo(18,2));                 % No.43  18 (-2.5, -1)
    F3F3 = Num2Fix(F3.^2,wordInfo(19,1),wordInfo(19,2));                   % No.44  19 (0, 1)
    F3F3BF3A = Num2Fix(F3F3.*BF3A,wordInfo(20,1),wordInfo(20,2));          % No.45  20 (-1, 0)
    HF3F3BF3A = Num2Fix(H + F3F3BF3A,wordInfo(21,1),wordInfo(21,2));       % No.46  21 (0, 1)
    v3HF3F3BF3A = Num2Fix(v3.*HF3F3BF3A,wordInfo(22,1),wordInfo(22,2));    % No.47  22 (0, 4600)
    X = Num2Fix(X + v3HF3F3BF3A,wordInfo(31,1),wordInfo(31,2));            % No.56  31 (128, 4600)
    
    F4 = Num2Fix(Y4 - fact,wordInfo(23,1),wordInfo(23,2));                 % No.48  23 (1, 2)
    F4D = Num2Fix(F4.*D,wordInfo(24,1),wordInfo(24,2));                    % No.49  24 (-1, -0.5)
    EF4D = Num2Fix(E + F4D,wordInfo(25,1),wordInfo(25,2));                 % No.50  25 (1.5, 2)
    F4EF4D = Num2Fix(F4.*EF4D,wordInfo(26,1),wordInfo(26,2));              % No.51  26 (2, 3)
    FF4EF4D = Num2Fix(F + F4EF4D,wordInfo(27,1),wordInfo(27,2));           % No.52  27 (-2, -1)
    F4FF4EF4D = Num2Fix(F4.*FF4EF4D,wordInfo(28,1),wordInfo(28,2));        % No.53  28 (-2.075, -2)
    GF4FF4EF4D = Num2Fix(G + F4FF4EF4D,wordInfo(29,1),wordInfo(29,2));     % No.54  29 (-0.075, 0)
    v4GF4FF4EF4D = Num2Fix(v4.*GF4FF4EF4D,wordInfo(30,1),wordInfo(30,2));  % No.55  30 (-277, 0)
    X = Num2Fix(X + v4GF4FF4EF4D,wordInfo(31,1),wordInfo(31,2));           % No.56  31 (128, 4600)
    %                                                                        No.57  32 v1
    %                                                                        No.57  33 v2
    %                                                                        No.57  34 v3
    %                                                                        No.57  35 v4
    %                                                                        No.13  36 fact
    %                                                                        No.13  37 fact
    %                                                                        No.13  38 fact
    %                                                                        No.13  39 fact
    %                                                                        No.15  40 Y1
    %                                                                        No.20  41 D
    %                                                                        No.21  42 E
    %                                                                        No.22  43 F
    %                                                                        No.23  44 G
    %                                                                        No.18  45 A
    %                                                                        No.19  46 B
    %                                                                        No.25  47 H
    %                                                                        No.16  48 Y3
    %                                                                        No.18  49 A
    %                                                                        No.19  50 B
    %                                                                        No.25  51 H
    %                                                                        No.17  52 Y4
    %                                                                        No.20  53 D
    %                                                                        No.21  54 E
    %                                                                        No.22  55 F
    %                                                                        No.23  56 G
    
else
    F1 =  (Y1 + fact);
    F1D = (F1.*D);
    EF1D = (E + F1D);
    F1EF1D = (F1.*EF1D);
    FF1EF1D = (F + F1EF1D);
    F1FF1EF1D =  (F1.*FF1EF1D);
    GF1FF1EF1D =  (G + F1FF1EF1D);
    v1GF1FF1EF1D = (v1.*GF1FF1EF1D);
    X = (X + v1GF1FF1EF1D);
    
    F2 = (fact);
    F2A = (F2*A);
    BF2A = (B + F2A);
    F2F2 = (F2.^2);
    F2F2BF2A = (F2F2.*BF2A);
    HF2F2BF2A = (H + F2F2BF2A);
    v2HF2F2BF2A = (v2.*HF2F2BF2A);
    X = (X + v2HF2F2BF2A);
    
    F3 = (Y3 - fact);
    F3A = (F3*A);
    BF3A = (B + F3A);
    F3F3 = (F3.^2);
    F3F3BF3A = (F3F3.*BF3A);
    HF3F3BF3A = (H + F3F3BF3A);
    v3HF3F3BF3A = (v3.*HF3F3BF3A);
    X = (X + v3HF3F3BF3A);
    
    F4 = (Y4 - fact);
    F4D = (F4.*D);
    EF4D = (E + F4D);
    F4EF4D = (F4.*EF4D);
    FF4EF4D = (F + F4EF4D);
    F4FF4EF4D = (F4.*FF4EF4D);
    GF4FF4EF4D = (G + F4FF4EF4D);
    v4GF4FF4EF4D = (v4.*GF4FF4EF4D);
    X = (X + v4GF4FF4EF4D);
    
end

end
function [fixPt, aa] = Num2Fix_(num, intLen,fracLen)
flag = num>=0;
% a = fi(num(flag), 1, intLen+fracLen, fracLen,'RoundingMethod','Floor');
aa = fi(num(flag), 1, intLen+fracLen, fracLen); %,'RoundingMethod','Floor');
bb = fi(num(~flag), 1, intLen+fracLen, fracLen);
fixPt = zeros(length(num),1);
fixPt(flag) = aa.data;
fixPt(~flag) = bb.data;
fixPt = reshape(fixPt,size(num));
end
function [fixPt, aa] = Num2Fix(num, intLen,fracLen)
flag = num>=0;
% a = fi(num(flag), 1, intLen+fracLen, fracLen,'RoundingMethod','Floor');
aa = fi(num(flag), 1, intLen+fracLen, fracLen,'RoundingMethod','Floor');
bb = fi(num(~flag), 1, intLen+fracLen, fracLen,'RoundingMethod','Floor');
fixPt = zeros(size(num));
fixPt(flag) = aa.data;
fixPt(~flag) = bb.data;
% fixPt = reshape(fixPt,size(num));
end
function [fixPt, aa] = Num2Fix_unsigned(num, intLen,fracLen)
flag = num>=0;
% a = fi(num, 0, intLen+fracLen, fracLen,'RoundingMethod','Floor');
aa = fi(num(flag), 0, intLen+fracLen, fracLen,'RoundingMethod','Floor');
bb = fi(num(~flag), 0, intLen+fracLen, fracLen,'RoundingMethod','Floor');
% % a = fi(num, 0, intLen+fracLen, fracLen);
% fixPt = a.data;
fixPt = zeros(size(num));
fixPt(flag) = aa.data;
fixPt(~flag) = bb.data;
end

function [xRectMat, yRectMat, LutVecCharX, LutVecCharY,SplitIndX,SplitIndY,lutVecX,lutVecY] = BiLinearInterp(inputDir,xOrigMat,yOrigMat,xRectMatRef,yRectMatRef,scaleSizeX,scaleSizeY,nc,nr,doRounding,optFrac)

optFrac = round(optFrac);
optFracCell = {[1:2];[3:4];[7:8];[9 10 13 14 15 16];[17 18 19 20];[21];[22]}; % [31 32 53 54];[33 55];[35 36 42 43];[37 44];[38 39 45 46];[40 47];[56]};
% intLen = [0;0;0;0;13;13;6;6;6;6;6;6;0;0;1;1;2;2;2;0;2;3;2;13;1; ...        % 1-25
%     1;0;1;2;1;2;0;10; ...                                                  % 26-33
%     0;1;2;0;0;0;13; ...                                                    % 34-40
%     0;1;2;0;0;0;13; ...                                                    % 41-47
%     1;0;1;2;1;2;0;10; ...                                                  % 48-55
%     13; ...                                                                % X 56
%     13]+1;                                                                 % v1v2v2v4 57



intLen = [0;0;0;0;13;13;7;7;7;7;7;7;0;0;1;1;2;2;2;0;2;3;2;13;1; ...        % 1-25
    1;0;1;2;1;2;0;10; ...                                                  % 26-33
    0;1;2;0;0;0;13; ...                                                    % 34-40
    0;1;2;0;0;0;13; ...                                                    % 41-47
    1;0;1;2;1;2;0;10; ...                                                  % 48-55
    13; ...                                                                % X 56
    13]+1;                                                                 % v1v2v2v4 57

intLen = [0 0 0 0 13 12 7 6 7 6 7 6 0 0 0 0 0 0 0 0 13 13] +1;

fixFrac = [0;0;0;0;0;0;0;1;1;1;1;0;0;0];

[xRectMat,LutVecCharX,SplitIndX,lutVecX] = bilinear2x2(xOrigMat,scaleSizeX,scaleSizeY,nc,nr,intLen,fixFrac,optFracCell,doRounding,optFrac);
[yRectMat,LutVecCharY,SplitIndY,lutVecY] = bilinear2x2(yOrigMat,scaleSizeX,scaleSizeY,nc,nr,intLen,fixFrac,optFracCell,doRounding,optFrac);
[~,errr] = NormalizeVector([xRectMat(:) yRectMat(:)] - [xRectMatRef(:) yRectMatRef(:)]);

figure,plot(errr);title(num2str(optFrac));
saveas(gcf,fullfile(inputDir,sprintf('img_%05d.png',length(dir(fullfile(inputDir,'img_*.png')))+1)));

end

function [b, LutVecChar, SplitInd,LutVec] = bilinear2x2(AA, kx,ky,nc,nr,intLen,fixFrac,optFracCell,doRounding,optFrac)
SplitInd = [];  LutVec = [];
optFrac = [optFrac(1:3) max(optFrac(2:3)) optFrac(4:end)];
tmp = cell2mat(optFracCell')';
fixFracId = setdiff([1:length(intLen)],cell2mat(optFracCell')')';
wordInfo(:,1) = intLen;
for i = 1 : length(optFracCell)
    wordInfo(optFracCell{i},2) = optFrac(i);
end
% wordInfo(fixFracId,2) = fixFrac;
% wordInfo(fixFracId,3) = 0;
% wordInfo(tmp,3) = 1;


[m,n] = size(AA);
NewRows = size(AA,1)*ky;
NewColumns = size(AA,2)*kx;
aa = [kx,0,0;0,ky,0;0.5*(1-kx),0.5*(1-ky),1];
invaa = inv(aa);

[xuuu,yuuu] = meshgrid([1:NewColumns],[1:NewRows]);

xuu = xuuu((NewRows-nr)/2+1:end-(NewRows-nr)/2,(NewColumns-nc)/2+1:end-(NewColumns-nc)/2);
yuu = yuuu((NewRows-nr)/2+1:end-(NewRows-nr)/2,(NewColumns-nc)/2+1:end-(NewColumns-nc)/2);



LutVecChar = [];
LutVecNum = [];




if doRounding
    a1 = Num2Fix(invaa(1,1),wordInfo(1,1),wordInfo(1,2));
    a3 = Num2Fix(invaa(2,2),wordInfo(2,1),wordInfo(2,2));
    a2 = Num2Fix(invaa(3,1),wordInfo(3,1),wordInfo(3,2));
    a4 = Num2Fix(invaa(3,2),wordInfo(4,1),wordInfo(4,2));
    Xuu = xuu(:);
    Yuu = yuu(:);
    a1xuu = Num2Fix(a1.*Xuu,wordInfo(7,1),wordInfo(7,2));% table size
    a3yuu = Num2Fix(a3.*Yuu,wordInfo(8,1),wordInfo(8,2)); % table size
    TempX = Num2Fix(a1xuu + a2,wordInfo(9,1),wordInfo(9,2));
    TempY = Num2Fix(a3yuu + a4,wordInfo(10,1),wordInfo(10,2));
    Y = floor(TempY);
    X = floor(TempX);
    Di =  Num2Fix(TempY - Y,wordInfo(13,1),wordInfo(13,2));% di = yfract
    Dj =  Num2Fix(TempX - X,wordInfo(14,1),wordInfo(14,2));% dj = xfract
    Di1 = Num2Fix(1 - Di,wordInfo(15,1),wordInfo(15,2)); %1 - Di;
    Dj1 = Num2Fix(1 - Dj,wordInfo(16,1),wordInfo(16,2)); %1 - Dj;
    coeff1 = Num2Fix(Di1.*Dj1,wordInfo(17,1),wordInfo(17,2));
    coeff2 = Num2Fix(Di1.*Dj,wordInfo(18,1),wordInfo(18,2));
    coeff3 = Num2Fix(Di.*Dj1,wordInfo(19,1),wordInfo(19,2));
    coeff4 = Num2Fix(Di.*Dj,wordInfo(20,1),wordInfo(20,2));
    %     Y1 = 1;
    %     Y3 = 1;
    %     Y4 = 2;
    %     A = 1.5;
    %     B = -2.5;
    %     D = -0.5;
    %     E = 2.5;
    %     F = -4;
    %     G = 2;
    AA = Num2Fix(AA,wordInfo(21,1),wordInfo(21,2));
    %     H = 1;
    
    AAAA = AA';
    dropInt = 4;
    
    if dropInt == 0
        LutVec = [size(AA,2);size(AA,1);nc;nr;kx;ky;NewColumns;NewRows;(NewColumns-nc)/2;(NewRows-nr)/2;a1*2^(wordInfo(1,2));a2*2^wordInfo(3,2);a3*2^wordInfo(2,2);a4*2^wordInfo(4,2);AAAA(:).*2^wordInfo(24,2)];
        intPart = [6;6;13;12;8;8;13;12;10;10;23;12;23;12;sum(wordInfo(24,1:2))];
        fracPart = [0;0;0;0;0;0;0;0;0;0;wordInfo(1,2);wordInfo(3,2);wordInfo(2,2);wordInfo(4,2)];
    else
        dropInt = 4;
        LutVec = [size(AA,2);size(AA,1);nc;nr;kx;ky;a1*2^(wordInfo(1,2));a2*2^wordInfo(3,2);a3*2^wordInfo(2,2);a4*2^wordInfo(4,2);AAAA(:).*2^wordInfo(21,2)];
        intPart = [wordInfo(11,1)-1;wordInfo(12,1)-1;wordInfo(5,1)-1;wordInfo(6,1)-1;8;8;wordInfo(1,2);wordInfo(3,2);wordInfo(1,2);wordInfo(3,2);sum(wordInfo(21,1:2))];
        fracPart = [0;0;0;0;0;0;wordInfo(1,2);wordInfo(3,2);wordInfo(2,2);wordInfo(4,2)];
    end
    
    %     supposeLen = sum(intPart(1:14)) + intPart(15)*length(AAAA(:));
    supposeLen = sum(intPart(1:14-dropInt)) + intPart(15-dropInt)*length(AAAA(:));
    
    %     LutVecChar = [];
    for u = 1 : length(LutVec)
        if u <= 10-dropInt
            numstr = LutVec(u);
            [fixPt, abin] = Num2Fix_unsigned(LutVec(u), intPart(u),0);
            ttmp = abin.bin;
            bin_x = dec2bin(LutVec(u),intPart(u));
            if ~strcmp(ttmp,bin_x)
                asvnlkl = 1;
            end
            LutVecChar = [LutVecChar '' abin.bin];
            LutVecNum = [LutVecNum '' abin.bin];
            sbkj = 1;
            
        elseif u <= 14-dropInt
            
            [fixPt, abin] = Num2Fix_unsigned(LutVec(u), intPart(u),0);
            ttmp = abin.bin;
            bin_x = dec2bin(LutVec(u),intPart(u));
            if ~strcmp(ttmp,bin_x)
                asvnlkl = 1;
            end
            LutVecChar = [LutVecChar '' abin.bin];
            sbkj = 1;
        else
            if u == 14 - dropInt + 1
                SplitInd = length(LutVecChar);
            end
            
            if LutVec(u) >= 0
                %             [fixPt, abin] = Num2Fix(num, wordInfo(24,1),wordInfo(24,2));
                %              LutVecChar = [LutVecChar '' abin.bin];
                %              sbkj = 1;
                [fixPt, abin] = Num2Fix(LutVec(u), intPart(15-dropInt),0);
                ttmp = abin.bin;
                bin_x = dec2bin(LutVec(u),intPart(15-dropInt));
                if ~strcmp(ttmp,bin_x)
                    asvnlkl = 1;
                end
                LutVecChar = [LutVecChar '' bin_x];
                
            else
                bin_x = dec2bin(2^intPart(15-dropInt) + LutVec(u),intPart(15-dropInt));
                LutVecChar = [LutVecChar '' bin_x];
            end
        end
        
    end
    asvjkjb = 1;
    if length(LutVecChar) == supposeLen
        LutVecData = str2num(LutVecChar);
        ajkvb = 1;
    end
    
else
    a1 = (invaa(1,1));
    a3 = (invaa(2,2));
    a2 = (invaa(3,1));
    a4 = (invaa(3,2));
    Xuu = xuu(:);
    Yuu = yuu(:);
    a1xuu = (a1.*Xuu);
    a3yuu = (a3.*Yuu);
    TempX = (a1xuu + a2);
    TempY = (a3yuu + a4);
    Y = floor(TempY);
    X = floor(TempX);
    Di =  (TempY - Y);
    Dj =  (TempX - X);
    Di1 = 1 - Di;
    Dj1 = 1 - Dj;
    coeff1 = Di1.*Dj1;
    coeff2 = Di1.*Dj;
    coeff3 = Di.*Dj1;
    coeff4 = Di.*Dj;
    %     Y1 = 1;
    %     Y3 = 1;
    %     Y4 = 2;
    %     A = 1.5;
    %     B = -2.5;
    %     D = -0.5;
    %     E = 2.5;
    %     F = -4;
    %     G = 2;
    AA = (AA);
    %     H = 1;
end


% AX = max(X-1,1);
BX = max(X,1);
CX = min(X+1,n);
% DX = min(X+2,n);
% AY = max(Y-1,1);
BY = max(Y,1);
CY = min(Y+1,m);
% DY = min(Y+2,m);
try
    %     ind1 = sub2ind(size(AA),AY,AX);
catch
    sdghk = 1;
end
% ind2 = sub2ind(size(AA),AY,BX);
% ind3 = sub2ind(size(AA),AY,CX);
% ind4 = sub2ind(size(AA),AY,DX);
% ind5 = sub2ind(size(AA),BY,AX);
ind6 = sub2ind(size(AA),BY,BX);
ind7 = sub2ind(size(AA),BY,CX);
% ind8 = sub2ind(size(AA),BY,DX);
% ind9 = sub2ind(size(AA),CY,AX);
ind10 = sub2ind(size(AA),CY,BX);
ind11 = sub2ind(size(AA),CY,CX);
% ind12 = sub2ind(size(AA),CY,DX);
% ind13 = sub2ind(size(AA),DY,AX);
% ind14 = sub2ind(size(AA),DY,BX);
% ind15 = sub2ind(size(AA),DY,CX);
% ind16 = sub2ind(size(AA),DY,DX);

% V11 = AA(ind1); V12 = AA(ind2); V13 = AA(ind3); V14 = AA(ind4);
% V21 = AA(ind5);
V22 = AA(ind6); V23 = AA(ind7);% V24 = AA(ind8);
% V31 = AA(ind9);
V32 = AA(ind10); V33 = AA(ind11);% V34 = AA(ind12);
% V41 = AA(ind13); V42 = AA(ind14); V43 = AA(ind15); V44 = AA(ind16);


% Num2Fix(AA,wordInfo(21,1),wordInfo(21,2));
temp1 = Num2Fix(coeff1.*V22,wordInfo(22,1),wordInfo(22,2));
temp2 = Num2Fix(coeff2.*V23,wordInfo(22,1),wordInfo(22,2));
temp3 = Num2Fix(coeff3.*V32,wordInfo(22,1),wordInfo(22,2));
temp4 = Num2Fix(coeff4.*V33,wordInfo(22,1),wordInfo(22,2));
bb = Num2Fix(temp1 + temp2 + temp3 + temp4,wordInfo(22,1),wordInfo(22,2));

b = reshape(bb,nr,nc);
return;
if 0
    variable{1,1} = a1; variable{2,1} = a3;
    variable{3,1} = a2; variable{4,1} = a4;
    variable{5,1} = Xuu; variable{6,1} = Yuu;
    variable{7,1} = a1xuu; variable{8,1} = a3yuu;
    variable{9,1} = TempX; variable{10,1} = TempY;
    variable{11,1} = Y; variable{12,1} = X;
    variable{13,1} = Di; variable{14,1} = Dj;
    % variable{15,1} = Y1;
    % variable{16,1} = Y3;
    % variable{17,1} = Y4;
    % variable{18,1} = A;
    % variable{19,1} = B;
    % variable{20,1} = D;
    % variable{21,1} = E;
    % variable{22,1} = F;
    % variable{23,1} = G;
    variable{24,1} = AA;
    % variable{25,1} = H;
    
    inherited = wordInfo([26:56 57 57 57 57 13 13 13 13 15 20 21 22 23 18 19 25 16 18 19 25 17 20 21 22 23],:);
    
    whole = [wordInfo;inherited];
    
    % Temp1 = bilinear2x2Func(V11,V12,V13,V14,Dj,Y1,Y3,Y4,A,B,D,E,F,G,H,inherited,doRounding);
    Temp2 = bilinear2x2Func(V22,V23,Dj,Y1,Y3,Y4,A,B,D,E,F,G,H,inherited,doRounding);
    Temp3 = bilinear2x2Func(V32,V33,Dj,Y1,Y3,Y4,A,B,D,E,F,G,H,inherited,doRounding);
    % Temp4 = bilinear2x2Func(V41,V42,V43,V44,Dj,Y1,Y3,Y4,A,B,D,E,F,G,H,inherited,doRounding);
    
    if 0
        figure,plot(sort(diff(sort([num2fixpt(([Temp1]),sfix(17),2^-4)]))));
        figure,plot((diff(sort([num2fixpt(([Temp1]),sfix(17),2^-4)]))));
        figure,plot(diff((num2fixpt(sort([Temp1]),sfix(17),2^-4)))-diff(sort([Temp1])))
        figure,plot(sort(abs(diff((num2fixpt(sort([Temp1]),sfix(17),2^-4)))-diff(sort([Temp1])))))
        figure,plot(((num2fixpt(([Temp1]),sfix(17),2^-5)))-(([Temp1])))
    end
    
    [bb] = bilinear2x2Func(Temp2,Temp3,Di,Y1,Y3,Y4,A,B,D,E,F,G,H,inherited, doRounding);
    b = reshape(bb,nr,nc);
    
end

end
function X = bilinear2x2Func(v1,v2,v3,v4,fact,Y1,Y3,Y4,A,B,D,E,F,G,H,wordInfo,doRounding)



X = zeros(size(v1,1),1);
if fact == 0
    X = v2;
    return
end

% r(x) = (a + 2)|x|^3 - (a + 3)|x|^2         +  1 , 0 <= |x| < 1
% r(x) =       a|x|^3 -      5a|x|^2 + 8a|x| - 4a , 1 <= |x| < 2
% where x is {fact,fact+1,abs(fact-1),abs(fact-2)}
% then r(x) will contain the interpolation ratio which will be multiplied
% by the point value.
% The sum of r(fact+1)*v1+r(fact)*v2+r(abs(fact-1))*v3+r(abs(fact-2))*v4
% will give the interpolated value.
% The value a is taken as -0.5 as per Matlab.



if doRounding
    F1 =  Num2Fix(Y1 + fact,wordInfo(1,1),wordInfo(1,2));                  % N0.26  1  (1, 2)
    F1D = Num2Fix(F1.*D,wordInfo(2,1),wordInfo(2,2));                      % No.27  2  (-1, -0.5)
    EF1D = Num2Fix(E + F1D,wordInfo(3,1),wordInfo(3,2));                   % No.28  3  (1.5, 2)
    F1EF1D = Num2Fix(F1.*EF1D,wordInfo(4,1),wordInfo(4,2));                % No.29  4  (2, 3)
    FF1EF1D = Num2Fix(F + F1EF1D,wordInfo(5,1),wordInfo(5,2));             % No.30  5  (-2, -1)
    F1FF1EF1D =  Num2Fix(F1.*FF1EF1D,wordInfo(6,1),wordInfo(6,2));         % No.31  6  (-2.07, -2)
    GF1FF1EF1D =  Num2Fix(G + F1FF1EF1D,wordInfo(7,1),wordInfo(7,2));      % No.32  7  (-0.075, 0)
    v1GF1FF1EF1D = Num2Fix(v1.*GF1FF1EF1D,wordInfo(8,1),wordInfo(8,2));    % No.33  8  (-240, 11)
    X = Num2Fix(X + v1GF1FF1EF1D,wordInfo(31,1),wordInfo(31,2));           % No.56  31 (128, 4600)
    
    F2 = Num2Fix(fact,wordInfo(9,1),wordInfo(9,2));                        % No.34  9  (0, 1)
    F2A = Num2Fix(F2*A,wordInfo(10,1),wordInfo(10,2));                     % No.35  10 (0, 1.5)
    BF2A = Num2Fix(B + F2A,wordInfo(11,1),wordInfo(11,2));                 % No.36  11 (-2.5, -1)
    F2F2 = Num2Fix(F2.^2,wordInfo(12,1),wordInfo(12,2));                   % No.37  12 (0, 1)
    F2F2BF2A = Num2Fix(F2F2.*BF2A,wordInfo(13,1),wordInfo(13,2));          % No.38  13 (-1, 0)
    HF2F2BF2A = Num2Fix(H + F2F2BF2A,wordInfo(14,1),wordInfo(14,2));       % No.39  14 (0, 1)
    v2HF2F2BF2A = Num2Fix(v2.*HF2F2BF2A,wordInfo(15,1),wordInfo(15,2));    % No.40  15 (-38, 4600)
    X = Num2Fix(X + v2HF2F2BF2A,wordInfo(31,1),wordInfo(31,2));            % No.56  31 (128, 4600)
    
    F3 = Num2Fix(Y3 - fact,wordInfo(16,1),wordInfo(16,2));                 % No.41  16 (0, 1)
    F3A = Num2Fix(F3*A,wordInfo(17,1),wordInfo(17,2));                     % No.42  17 (0, 1.5)
    BF3A = Num2Fix(B + F3A,wordInfo(18,1),wordInfo(18,2));                 % No.43  18 (-2.5, -1)
    F3F3 = Num2Fix(F3.^2,wordInfo(19,1),wordInfo(19,2));                   % No.44  19 (0, 1)
    F3F3BF3A = Num2Fix(F3F3.*BF3A,wordInfo(20,1),wordInfo(20,2));          % No.45  20 (-1, 0)
    HF3F3BF3A = Num2Fix(H + F3F3BF3A,wordInfo(21,1),wordInfo(21,2));       % No.46  21 (0, 1)
    v3HF3F3BF3A = Num2Fix(v3.*HF3F3BF3A,wordInfo(22,1),wordInfo(22,2));    % No.47  22 (0, 4600)
    X = Num2Fix(X + v3HF3F3BF3A,wordInfo(31,1),wordInfo(31,2));            % No.56  31 (128, 4600)
    
    F4 = Num2Fix(Y4 - fact,wordInfo(23,1),wordInfo(23,2));                 % No.48  23 (1, 2)
    F4D = Num2Fix(F4.*D,wordInfo(24,1),wordInfo(24,2));                    % No.49  24 (-1, -0.5)
    EF4D = Num2Fix(E + F4D,wordInfo(25,1),wordInfo(25,2));                 % No.50  25 (1.5, 2)
    F4EF4D = Num2Fix(F4.*EF4D,wordInfo(26,1),wordInfo(26,2));              % No.51  26 (2, 3)
    FF4EF4D = Num2Fix(F + F4EF4D,wordInfo(27,1),wordInfo(27,2));           % No.52  27 (-2, -1)
    F4FF4EF4D = Num2Fix(F4.*FF4EF4D,wordInfo(28,1),wordInfo(28,2));        % No.53  28 (-2.075, -2)
    GF4FF4EF4D = Num2Fix(G + F4FF4EF4D,wordInfo(29,1),wordInfo(29,2));     % No.54  29 (-0.075, 0)
    v4GF4FF4EF4D = Num2Fix(v4.*GF4FF4EF4D,wordInfo(30,1),wordInfo(30,2));  % No.55  30 (-277, 0)
    X = Num2Fix(X + v4GF4FF4EF4D,wordInfo(31,1),wordInfo(31,2));           % No.56  31 (128, 4600)
    %                                                                        No.57  32 v1
    %                                                                        No.57  33 v2
    %                                                                        No.57  34 v3
    %                                                                        No.57  35 v4
    %                                                                        No.13  36 fact
    %                                                                        No.13  37 fact
    %                                                                        No.13  38 fact
    %                                                                        No.13  39 fact
    %                                                                        No.15  40 Y1
    %                                                                        No.20  41 D
    %                                                                        No.21  42 E
    %                                                                        No.22  43 F
    %                                                                        No.23  44 G
    %                                                                        No.18  45 A
    %                                                                        No.19  46 B
    %                                                                        No.25  47 H
    %                                                                        No.16  48 Y3
    %                                                                        No.18  49 A
    %                                                                        No.19  50 B
    %                                                                        No.25  51 H
    %                                                                        No.17  52 Y4
    %                                                                        No.20  53 D
    %                                                                        No.21  54 E
    %                                                                        No.22  55 F
    %                                                                        No.23  56 G
    
else
    F1 =  (Y1 + fact);
    F1D = (F1.*D);
    EF1D = (E + F1D);
    F1EF1D = (F1.*EF1D);
    FF1EF1D = (F + F1EF1D);
    F1FF1EF1D =  (F1.*FF1EF1D);
    GF1FF1EF1D =  (G + F1FF1EF1D);
    v1GF1FF1EF1D = (v1.*GF1FF1EF1D);
    X = (X + v1GF1FF1EF1D);
    
    F2 = (fact);
    F2A = (F2*A);
    BF2A = (B + F2A);
    F2F2 = (F2.^2);
    F2F2BF2A = (F2F2.*BF2A);
    HF2F2BF2A = (H + F2F2BF2A);
    v2HF2F2BF2A = (v2.*HF2F2BF2A);
    X = (X + v2HF2F2BF2A);
    
    F3 = (Y3 - fact);
    F3A = (F3*A);
    BF3A = (B + F3A);
    F3F3 = (F3.^2);
    F3F3BF3A = (F3F3.*BF3A);
    HF3F3BF3A = (H + F3F3BF3A);
    v3HF3F3BF3A = (v3.*HF3F3BF3A);
    X = (X + v3HF3F3BF3A);
    
    F4 = (Y4 - fact);
    F4D = (F4.*D);
    EF4D = (E + F4D);
    F4EF4D = (F4.*EF4D);
    FF4EF4D = (F + F4EF4D);
    F4FF4EF4D = (F4.*FF4EF4D);
    GF4FF4EF4D = (G + F4FF4EF4D);
    v4GF4FF4EF4D = (v4.*GF4FF4EF4D);
    X = (X + v4GF4FF4EF4D);
    
end

end
function polyCnr = FindPolyon(pt0)

DT = delaunayTriangulation(pt0(:,1),pt0(:,2));
k = convexHull(DT);
polyCnr = DT.Points(k,:);
end
function [xRectMat, yRectMat, LutVecCharX, LutVecCharY,SplitIndX,SplitIndY,lutVecX,lutVecY,nameMatX, nameMatY] = BiLinearInterp_(reverseMapping, inputDir,xuu2, yuu2, xOrigMat,yOrigMat,xRectMatRef,yRectMatRef,scaleSizeX,scaleSizeY,nc,nr,doRounding,optFrac, speedUp)

if ~exist('speedUp', 'var')
    speedUp = 1;
end
optFrac = round(optFrac);
optFracCell = {[1:2];[3:4];[7:8];[9 10 13 14 15 16];[17 18 19 20];[21];[22]}; % [31 32 53 54];[33 55];[35 36 42 43];[37 44];[38 39 45 46];[40 47];[56]};
% intLen = [0;0;0;0;13;13;6;6;6;6;6;6;0;0;1;1;2;2;2;0;2;3;2;13;1; ...        % 1-25
%     1;0;1;2;1;2;0;10; ...                                                  % 26-33
%     0;1;2;0;0;0;13; ...                                                    % 34-40
%     0;1;2;0;0;0;13; ...                                                    % 41-47
%     1;0;1;2;1;2;0;10; ...                                                  % 48-55
%     13; ...                                                                % X 56
%     13]+1;                                                                 % v1v2v2v4 57



intLen = [0;0;0;0;13;13;7;7;7;7;7;7;0;0;1;1;2;2;2;0;2;3;2;13;1; ...        % 1-25
    1;0;1;2;1;2;0;10; ...                                                  % 26-33
    0;1;2;0;0;0;13; ...                                                    % 34-40
    0;1;2;0;0;0;13; ...                                                    % 41-47
    1;0;1;2;1;2;0;10; ...                                                  % 48-55
    13; ...                                                                % X 56
    13]+1;                                                                 % v1v2v2v4 57

intLen = [0 0 0 0 13 12 7 6 7 6 7 6 0 0 0 0 0 0 0 0 13 13] +1;

fixFrac = [0;0;0;0;0;0;0;1;1;1;1;0;0;0];

[xRectMat,LutVecCharX,SplitIndX,lutVecX,nameMatX] = bilinear2x2_(1,reverseMapping, xuu2, yuu2, xOrigMat,scaleSizeX,scaleSizeY,nc,nr,intLen,fixFrac,optFracCell,doRounding,optFrac, inputDir, speedUp);
[yRectMat,LutVecCharY,SplitIndY,lutVecY,nameMatY] = bilinear2x2_(2,reverseMapping, xuu2, yuu2, yOrigMat,scaleSizeX,scaleSizeY,nc,nr,intLen,fixFrac,optFracCell,doRounding,optFrac, inputDir, speedUp);
[~,errr] = NormalizeVector([xRectMat(:) yRectMat(:)] - [xRectMatRef(:) yRectMatRef(:)]);

if 0
    figure,plot(errr);title(num2str(optFrac));
    saveas(gcf,fullfile(inputDir,sprintf('img_%05d.png',length(dir(fullfile(inputDir,'img_*.png')))+1)));
end

end

function [b, LutVecChar, SplitInd,LutVec,nameMat] = bilinear2x2_(coordType,reverseMapping, xuu2, yuu2, AA, kx,ky,nc,nr,intLen,fixFrac,optFracCell,doRounding,optFrac, inputDir, speedUp)

if ~exist('inputDir','var')
    inputDir = pwd;
end

if ~exist('speedUp','var')
    speedUp = 1;
end


%fprintf('##############\n');
if 0
    p2 = polyfit(yuu2(:,1)',1:size(yuu2,1),1);
    figure,plot(1:480,(polyval(p2,1:480)),'-x');hold on;plot(yuu2(:,1),1:size(yuu2,1),'or')
    figure,plot(1:480,round((polyval(p2,1:480))),'-x');hold on;plot(yuu2(:,1),1:size(yuu2,1),'or')
    p22 = p2; p22(2) = round(p2(2)*2^10)/2^10;
    p22 = p2; p22(2) = round(p2(2)*2^10)/2^10;figure,plot(1:480,round((polyval(p2,1:480))),'-x');hold on;plot(yuu2(:,1),1:size(yuu2,1),'or');plot(1:480,round((polyval(p22,1:480))),'-x');
    nnn = 10;p22 = p2; p22(2) = round(p2(2)*2^nnn)/2^nnn;figure,plot(1:480,round((polyval(p2,1:480))),'-o');hold on;plot(yuu2(:,1),1:size(yuu2,1),'or');plot(1:480,round((polyval(p22,1:480))),'-.');
    nnn = 5;p22 = p2; p22(2) = round(p2(2)*2^nnn)/2^nnn;figure,plot(1:480,round((polyval(p2,1:480))),'-o');hold on;plot(yuu2(:,1),1:size(yuu2,1),'or');plot(1:480,round((polyval(p22,1:480))),'-.');
    nnn = 5;p22 = p2; p22(2) = round(p2(2)*2^nnn)/2^nnn;figure,plot(1:480,round((polyval(p2,1:480))) - round((polyval(p22,1:480))),'-.');
    nnn = 1;p22 = p2; p22(2) = round(p2(2)*2^nnn)/2^nnn;figure,plot(1:480,round((polyval(p2,1:480))) - round((polyval(p22,1:480))),'-.');
    nnn = 2;p22 = p2; p22(2) = round(p2(2)*2^nnn)/2^nnn;figure,plot(1:480,round((polyval(p2,1:480))) - round((polyval(p22,1:480))),'-.');
    nnn = 3;p22 = p2; p22(2) = round(p2(2)*2^nnn)/2^nnn;figure,plot(1:480,round((polyval(p2,1:480))) - round((polyval(p22,1:480))),'-.');
    nnn = 4;p22 = p2; p22(2) = round(p2(2)*2^nnn)/2^nnn;figure,plot(1:480,round((polyval(p2,1:480))) - round((polyval(p22,1:480))),'-.');
    nnn = 5;p22 = p2; p22(2) = round(p2(2)*2^nnn)/2^nnn;figure,plot(1:480,round((polyval(p2,1:480))) - round((polyval(p22,1:480))),'-.');
end


SplitInd = [];  LutVec = [];
optFrac = [optFrac(1:3) max(optFrac(2:3)) optFrac(4:end)];
tmp = cell2mat(optFracCell')';
fixFracId = setdiff([1:length(intLen)],cell2mat(optFracCell')')';
wordInfo(:,1) = intLen;
for i = 1 : length(optFracCell)
    wordInfo(optFracCell{i},2) = optFrac(i);
end
% wordInfo(fixFracId,2) = fixFrac;
% wordInfo(fixFracId,3) = 0;
% wordInfo(tmp,3) = 1;


[m,n] = size(AA);
NewRows = size(AA,1)*ky;
NewColumns = size(AA,2)*kx;
aa = [kx,0,0;0,ky,0;0.5*(1-kx),0.5*(1-ky),1];
invaa = inv(aa);

[xuuu,yuuu] = meshgrid([1:NewColumns],[1:NewRows]);

[xAll,yAll] = meshgrid([1:nc],[1:nr]);
pixAll = [xAll(:) yAll(:)];


xuu = xuuu((NewRows-nr)/2+1:end-(NewRows-nr)/2,(NewColumns-nc)/2+1:end-(NewColumns-nc)/2);
yuu = yuuu((NewRows-nr)/2+1:end-(NewRows-nr)/2,(NewColumns-nc)/2+1:end-(NewColumns-nc)/2);



LutVecChar = [];
LutVecNum = [];




if doRounding
%     tic;
    a1 = Num2Fix(invaa(1,1),wordInfo(1,1),wordInfo(1,2));
    a3 = Num2Fix(invaa(2,2),wordInfo(2,1),wordInfo(2,2));
    a2 = Num2Fix(invaa(3,1),wordInfo(3,1),wordInfo(3,2));
    a4 = Num2Fix(invaa(3,2),wordInfo(4,1),wordInfo(4,2));
    Xuu = xuu(:);
    Yuu = yuu(:);
    a1xuu = Num2Fix(a1.*Xuu,wordInfo(7,1),wordInfo(7,2));% table size
    a3yuu = Num2Fix(a3.*Yuu,wordInfo(8,1),wordInfo(8,2)); % table size
    TempX = Num2Fix(a1xuu + a2,wordInfo(9,1),wordInfo(9,2));
    TempY = Num2Fix(a3yuu + a4,wordInfo(10,1),wordInfo(10,2));
    Y = floor(TempY);
    X = floor(TempX);
    Di =  Num2Fix(TempY - Y,wordInfo(13,1),wordInfo(13,2));% di = yfract
    Dj =  Num2Fix(TempX - X,wordInfo(14,1),wordInfo(14,2));% dj = xfract
    Di1 = Num2Fix(1 - Di,wordInfo(15,1),wordInfo(15,2)); %1 - Di;
    Dj1 = Num2Fix(1 - Dj,wordInfo(16,1),wordInfo(16,2)); %1 - Dj;
    coeff1 = Num2Fix(Di1.*Dj1,wordInfo(17,1),wordInfo(17,2));
    coeff2 = Num2Fix(Di1.*Dj,wordInfo(18,1),wordInfo(18,2));
    coeff3 = Num2Fix(Di.*Dj1,wordInfo(19,1),wordInfo(19,2));
    coeff4 = Num2Fix(Di.*Dj,wordInfo(20,1),wordInfo(20,2));
%     toc;
    %     Y1 = 1;
    %     Y3 = 1;
    %     Y4 = 2;
    %     A = 1.5;
    %     B = -2.5;
    %     D = -0.5;
    %     E = 2.5;
    %     F = -4;
    %     G = 2;
    %     AA = Num2Fix(AA,wordInfo(21,1),wordInfo(21,2));
    %     fracVec = [5 5 5 5 5 5];
    fracVec = optFrac;
    AA = Num2Fix_(AA,14,fracVec(1));
    %     H = 1;
    
    if coordType == 1 % x
        AAAAA = xuu2 - AA;
    else
        AAAAA = yuu2 - AA;
    end
    if 0
        AAAA = AAAAA';
    else
        AAAA = AAAAA;
    end
    
    if 0
        [aa1,cc1] = RecoverLut('C:\Users\ROGER\Desktop\new 1.txt',xuu2(1,:), yuu2(:,1)');
        if coordType == 1
            AA = xuu2 - aa1;
        else
            AA = xuu2 - cc1;
        end
    else
        sdvjg = 1;
    end

    
    
    
    dropInt = 4;
    
    %%
    %     header format:
    %     mutual    ic: 13.0  0                            1
    %               ir: 12.0  0                            2
    %
    %               cropX1: 11.0 0                         3
    %               cropY1: 10.0 0                         4
    %
    %               cropX2: 13.0 0                         5
    %               cropY2: 12.0 0                         6
    %
    %
    %     separate    forward  table column 5.0 0          7
    %                          table row 6.0 0             8
    %                          col*controlPointX 13.1 1    9
    %                          row*controlPointY 12.1 1    10
    %                          col*row*LutX 8.5 1          11
    %                          col*row*LutY 8.5 1          12
    %
    %                  reverse  table column 6.0 0         7
    %                           table row 6.0 0            8
    %                           col*controlPointX 13.1 1   9
    %                           row*controlPointY 12.1 1   10
    %                           col*row*LutX 7.5 1         11
    %                           col*row*LutY 7.5 1         12
    
    
    %%
    
    cropPt1 = [1 1]';
    cropPt2 = [nc nr]';
    
    wordInfo = [13 0 0;12 0 0; 11 0 0; 10 0 0; 13 0 0; 12 0 0];
    if reverseMapping == 1
        % forward case
        wordInfo = [wordInfo;[5 0 0;6 0 0; 13 1 1;12 1 1; 8 fracVec(1) 1; 8 fracVec(1) 1]];
    else
        % reverse case
        wordInfo = [wordInfo;[6 0 0;6 0 0; 13 1 1;12 1 1; 7 fracVec(1) 1; 7 fracVec(1) 1]];
        
    end
    wordInfo(:,1) = wordInfo(:,1) + wordInfo(:,3);
    WordInfo = wordInfo(:,1) + wordInfo(:,2);
    LutVec = [nc; nr;cropPt1;cropPt2; size(AA,2);size(AA,1);  xuu2(1,:)'.*2^wordInfo(9,2); yuu2(:,1).*2^wordInfo(10,2);AAAA(:).*2^wordInfo(11,2)];
    lengthMat = [wordInfo(1:2,:);wordInfo([3 4],:);wordInfo([5 6],:);wordInfo([7 8],:);repmat(wordInfo([9],:),size(xuu2,2),1);repmat(wordInfo([10],:),size(yuu2,1),1);repmat(wordInfo([11],:),size(yuu2,1)*size(yuu2,2),1) ];
    nameMat = {};
    nameMat = {['ic' ] [sum(wordInfo(1,1:2))];['ir'] [sum(wordInfo(2,1:2))];['x1'] [sum(wordInfo(3,1:2))];['y1'] [sum(wordInfo(4,1:2))];['x2'] [sum(wordInfo(5,1:2))];['y2'] [sum(wordInfo(6,1:2))];['tc'] [sum(wordInfo(7,1:2))];['tr'] [sum(wordInfo(8,1:2))];[repmat('controlX',size(xuu2,2),1)] [size(xuu2,2)*sum(wordInfo(9,1:2))];[repmat('controlY',size(yuu2,1),1)] [size(yuu2,1)*sum(wordInfo(10,1:2))];[repmat('Lut',size(yuu2,1)*size(yuu2,2),1)] [size(yuu2,1)*size(yuu2,2)*sum(wordInfo(11,1:2))]};
    lenMat = sum(cell2mat(nameMat(:,2)));
    %             intPart = [wordInfo(11,1)-1;wordInfo(12,1)-1;wordInfo(5,1)-1;wordInfo(6,1)-1;8;8;wordInfo(1,2);wordInfo(3,2);wordInfo(1,2);wordInfo(3,2);sum(wordInfo(21,1:2))];
    %             fracPart = [0;0;0;0;0;0;wordInfo(1,2);wordInfo(3,2);wordInfo(2,2);wordInfo(4,2)];
%     tic;
    for u = 1 : length(LutVec)
        %                 if lengthMat(u,3) == 0
        %                     numstr = LutVec(u);
        %                     [fixPt, abin] = Num2Fix_unsigned(LutVec(u), lengthMat(u,1),lengthMat(u,2));
        %                     ttmp = abin.bin;
        %                     bin_x = dec2bin(LutVec(u),lengthMat(u,1)+lengthMat(u,2));
        %                     if ~strcmp(ttmp,bin_x)
        %                         asvnlkl = 1;
        %                     end
        %                     LutVecChar = [LutVecChar '' abin.bin];
        %                     LutVecNum = [LutVecNum '' abin.bin];
        %                     sbkj = 1;
        %
        %                 elseif u <= 14-dropInt
        %
        %                     [fixPt, abin] = Num2Fix_unsigned(LutVec(u), intPart(u),0);
        %                     ttmp = abin.bin;
        %                     bin_x = dec2bin(LutVec(u),intPart(u));
        %                     if ~strcmp(ttmp,bin_x)
        %                         asvnlkl = 1;
        %                     end
        %                     LutVecChar = [LutVecChar '' abin.bin];
        %                     sbkj = 1;
        %                 else
        %                     if u == 14 - dropInt + 1
        %                         SplitInd = length(LutVecChar);
        %                     end
        
        if LutVec(u) >= 0
            %             [fixPt, abin] = Num2Fix(num, wordInfo(24,1),wordInfo(24,2));
            %              LutVecChar = [LutVecChar '' abin.bin];
            %              sbkj = 1;
            
            
            
            
            
            if lengthMat(u,3) == 0
                numstr = LutVec(u);
                %             [fixPt, abin] = Num2Fix_unsigned(LutVec(u), lengthMat(u,1),lengthMat(u,2));
                [fixPt, abin] = Num2Fix_unsigned(LutVec(u), lengthMat(u,1)+lengthMat(u,2),0);
                ttmp = abin.bin;
                bin_x = dec2bin(LutVec(u),lengthMat(u,1)+lengthMat(u,2));
                if ~strcmp(ttmp,bin_x)
                    asvnlkl = 1;
                end
                LutVecChar = [LutVecChar '' abin.bin];
                LutVecNum = [LutVecNum '' abin.bin];
                
            else
                
                numstr = LutVec(u);
                %             [fixPt, abin] = Num2Fix(LutVec(u), lengthMat(u,1),lengthMat(u,2));
                [fixPt, abin] = Num2Fix(LutVec(u), lengthMat(u,1)+lengthMat(u,2),0);
                ttmp = abin.bin;
                bin_x = dec2bin(LutVec(u),lengthMat(u,1)+lengthMat(u,2));
                if ~strcmp(ttmp,bin_x)
                    asvnlkl = 1;
                end
                LutVecChar = [LutVecChar '' abin.bin];
                LutVecNum = [LutVecNum '' abin.bin];
            end
            
            
            
            
            
            
            
            %                         [fixPt, abin] = Num2Fix(LutVec(u), intPart(15-dropInt),0);
            %                         ttmp = abin.bin;
            %                         bin_x = dec2bin(LutVec(u),intPart(15-dropInt));
            %                         if ~strcmp(ttmp,bin_x)
            %                             asvnlkl = 1;
            %                         end
            %         LutVecChar = [LutVecChar '' bin_x];
            
        else
            %                         bin_x = dec2bin(2^intPart(15-dropInt) + LutVec(u),intPart(15-dropInt));
            %                         LutVecChar = [LutVecChar '' bin_x];
            
            
            if lengthMat(u,3) == 0
                %                             numstr = LutVec(u);
                %                             [fixPt, abin] = Num2Fix_unsigned(LutVec(u), lengthMat(u,1),lengthMat(u,2));
                %                             ttmp = abin.bin;
                %                             bin_x = dec2bin(LutVec(u),lengthMat(u,1)+lengthMat(u,2));
                %                             if ~strcmp(ttmp,bin_x)
                %                                 asvnlkl = 1;
                %                             end
                %                             LutVecChar = [LutVecChar '' abin.bin];
                %                             LutVecNum = [LutVecNum '' abin.bin];
                
                bin_x = dec2bin(2^(lengthMat(u,1)+lengthMat(u,2)) + LutVec(u),lengthMat(u,1)+lengthMat(u,2));
                LutVecChar = [LutVecChar '' bin_x];
                
            else
                
                %                             numstr = LutVec(u);
                %                             [fixPt, abin] = Num2Fix(LutVec(u), lengthMat(u,1),lengthMat(u,2));
                %                             ttmp = abin.bin;
                %                             bin_x = dec2bin(LutVec(u),lengthMat(u,1)+lengthMat(u,2));
                %                             if ~strcmp(ttmp,bin_x)
                %                                 asvnlkl = 1;
                %                             end
                %                             LutVecChar = [LutVecChar '' abin.bin];
                %                             LutVecNum = [LutVecNum '' abin.bin];
                
                try
                    bin_x = dec2bin(2^(lengthMat(u,1)+lengthMat(u,2)) + LutVec(u),lengthMat(u,1)+lengthMat(u,2));
                catch
                    sagk = 1;
                end
                LutVecChar = [LutVecChar '' bin_x];
                
            end
            
            
            
            
        end
    end
%     toc;
    %             end
    
    
    
    
    
    
    XY = pixAll;
    cnt = 1;
    
    
    if 1
        fida = fopen(fullfile(inputDir, 'data1.txt'),'w');
        dat1 = [xuu2;yuu2]';
        fprintf(fida,'%0.10f\n',(dat1(:)));
        fclose(fida); 
    
    fida = fopen(fullfile(inputDir, 'data2.txt'),'w');
        dat2 = [AA]';
        fprintf(fida,'%0.10f\n',(dat2(:)));
        fclose(fida); 
    
    
    end
    %     tic;
    if ~speedUp
        for j = 1 : size(xuu2,1) - 1
            markY = yuu2(j,1);
            markY_next = yuu2(j+1,1);
            dltY = markY_next - markY;
            for k = 1 : size(xuu2,2) - 1
                markX = xuu2(1,k);
                markX_next = xuu2(1,k+1);
                dltX = markX_next - markX;
                dltXY = dltX*dltY;
                id = find(XY(:,1) >= markX & XY(:,1) < markX_next & XY(:,2) >= markY & XY(:,2) < markY_next);
                storeXY{cnt,1} = XY(id,:);
                storeXY{cnt,2} = Num2Fix([AA(j,k) AA(j,k+1) AA(j+1,k) AA(j+1,k+1)]./dltXY,14,fracVec(2));
                storeXY{cnt,3} = [(markX_next-XY(id,1)).*(markY_next-XY(id,2)) (-markX+XY(id,1)).*(markY_next-XY(id,2)) (markX_next-XY(id,1)).*(-markY+XY(id,2)) (-markX+XY(id,1)).*(-markY+XY(id,2))];
                %             storeXY{cnt,4} = Num2Fix_(dot(repmat(storeXY{cnt,2},length(id),1)',storeXY{cnt,3}')',14,fracVec(3));
                storeXY{cnt,4} = Num2Fix(Num2Fix(repmat(storeXY{cnt,2}(1),length(id),1).*storeXY{cnt,3}(:,1),14,fracVec(3))...
                    + Num2Fix(repmat(storeXY{cnt,2}(2),length(id),1).*storeXY{cnt,3}(:,2),14,fracVec(3)) + ...
                    + Num2Fix(repmat(storeXY{cnt,2}(3),length(id),1).*storeXY{cnt,3}(:,3),14,fracVec(3)) + ...
                    + Num2Fix(repmat(storeXY{cnt,2}(4),length(id),1).*storeXY{cnt,3}(:,4),14,fracVec(3)),14,fracVec(5));
                storeXY{cnt,5} = id;
                if 0
                    ac = find(ismember(XY(id,:),[100 100],'rows'));storeXY{cnt,4}(ac)*2^9
                end
                cnt = cnt + 1;
            end
        end
        %     toc;
        b = zeros(nr,nc);
        b(cell2mat(storeXY(:,5))) = cell2mat(storeXY(:,4));
    else
        %         tic;
        
        cmdStr = [inputDir,'\func\Release\ConsoleApplication2.exe',' ',num2str(size(dat1,2)),' ',num2str(size(dat1,1)),' ',num2str(nr),' ',num2str(nc)];
        %         cmdStr = ['ConsoleApplication2.exe',' ',num2str(size(dat1,2)),' ',num2str(size(dat1,1)),' ',num2str(nr),' ',num2str(nc)];
        strSeg = strsplit(inputDir,'\');
        
        inputName = [];
        for j = 1 : length(strSeg)
            inputName = [inputName strSeg{j},'/'];
            
        end
        inputName = inputName(1:end-1);
        if 0
            cmdStr = [inputDir,'\func\Release\ConsoleApplication2.exe',' ',num2str(size(dat1,2)),' ',num2str(size(dat1,1)),' ',num2str(nr),' ',num2str(nc) ,' ',inputName];
        else
            tempDir = pwd;
            cmdStr = [inputDir,'\func\Release\ConsoleApplication2.exe',' ',num2str(size(dat1,2)),' ',num2str(size(dat1,1)),' ',num2str(nr),' ',num2str(nc) ,' ',inputName,' ',strcat(inputName,'/data1.txt'),' ',strcat(inputName,'/data2.txt'),' ',num2str(12)];
        end
        system(cmdStr);
        if 0
            aa = load(fullfile(inputDir,'out1.txt'));
            b = reshape(aa, nc, nr)';
        else
            aa = load(fullfile(inputDir,'out12.txt'));
            b = reshape(aa, nc, nr)'./2^fracVec(5);
        end
        %         toc;
        %         figure,imshow(reshape(aa, size(b,2),size(b,1))' - b,[]);
    end
    
    
    return;
    
    
    
    
    
    
    
    
    
    
    
    if 0
        
        if dropInt == 0
            LutVec = [size(AA,2);size(AA,1);nc;nr;kx;ky;NewColumns;NewRows;(NewColumns-nc)/2;(NewRows-nr)/2;a1*2^(wordInfo(1,2));a2*2^wordInfo(3,2);a3*2^wordInfo(2,2);a4*2^wordInfo(4,2);AAAA(:).*2^wordInfo(24,2)];
            intPart = [6;6;13;12;8;8;13;12;10;10;23;12;23;12;sum(wordInfo(24,1:2))];
            fracPart = [0;0;0;0;0;0;0;0;0;0;wordInfo(1,2);wordInfo(3,2);wordInfo(2,2);wordInfo(4,2)];
        else
            dropInt = 4;
            LutVec = [size(AA,2);size(AA,1);nc;nr;kx;ky;a1*2^(wordInfo(1,2));a2*2^wordInfo(3,2);a3*2^wordInfo(2,2);a4*2^wordInfo(4,2);AAAA(:).*2^wordInfo(21,2)];
            intPart = [wordInfo(11,1)-1;wordInfo(12,1)-1;wordInfo(5,1)-1;wordInfo(6,1)-1;8;8;wordInfo(1,2);wordInfo(3,2);wordInfo(1,2);wordInfo(3,2);sum(wordInfo(21,1:2))];
            fracPart = [0;0;0;0;0;0;wordInfo(1,2);wordInfo(3,2);wordInfo(2,2);wordInfo(4,2)];
        end
        
        %     supposeLen = sum(intPart(1:14)) + intPart(15)*length(AAAA(:));
        supposeLen = sum(intPart(1:14-dropInt)) + intPart(15-dropInt)*length(AAAA(:));
        
        %     LutVecChar = [];
        for u = 1 : length(LutVec)
            if u <= 10-dropInt
                numstr = LutVec(u);
                [fixPt, abin] = Num2Fix_unsigned(LutVec(u), intPart(u),0);
                ttmp = abin.bin;
                bin_x = dec2bin(LutVec(u),intPart(u));
                if ~strcmp(ttmp,bin_x)
                    asvnlkl = 1;
                end
                LutVecChar = [LutVecChar '' abin.bin];
                LutVecNum = [LutVecNum '' abin.bin];
                sbkj = 1;
                
            elseif u <= 14-dropInt
                
                [fixPt, abin] = Num2Fix_unsigned(LutVec(u), intPart(u),0);
                ttmp = abin.bin;
                bin_x = dec2bin(LutVec(u),intPart(u));
                if ~strcmp(ttmp,bin_x)
                    asvnlkl = 1;
                end
                LutVecChar = [LutVecChar '' abin.bin];
                sbkj = 1;
            else
                if u == 14 - dropInt + 1
                    SplitInd = length(LutVecChar);
                end
                
                if LutVec(u) >= 0
                    %             [fixPt, abin] = Num2Fix(num, wordInfo(24,1),wordInfo(24,2));
                    %              LutVecChar = [LutVecChar '' abin.bin];
                    %              sbkj = 1;
                    [fixPt, abin] = Num2Fix(LutVec(u), intPart(15-dropInt),0);
                    ttmp = abin.bin;
                    bin_x = dec2bin(LutVec(u),intPart(15-dropInt));
                    if ~strcmp(ttmp,bin_x)
                        asvnlkl = 1;
                    end
                    LutVecChar = [LutVecChar '' bin_x];
                    
                else
                    bin_x = dec2bin(2^intPart(15-dropInt) + LutVec(u),intPart(15-dropInt));
                    LutVecChar = [LutVecChar '' bin_x];
                end
            end
            
        end
        asvjkjb = 1;
        if length(LutVecChar) == supposeLen
            LutVecData = str2num(LutVecChar);
            ajkvb = 1;
        end
    end
    
else
    a1 = (invaa(1,1));
    a3 = (invaa(2,2));
    a2 = (invaa(3,1));
    a4 = (invaa(3,2));
    Xuu = xuu(:);
    Yuu = yuu(:);
    a1xuu = (a1.*Xuu);
    a3yuu = (a3.*Yuu);
    TempX = (a1xuu + a2);
    TempY = (a3yuu + a4);
    Y = floor(TempY);
    X = floor(TempX);
    Di =  (TempY - Y);
    Dj =  (TempX - X);
    Di1 = 1 - Di;
    Dj1 = 1 - Dj;
    coeff1 = Di1.*Dj1;
    coeff2 = Di1.*Dj;
    coeff3 = Di.*Dj1;
    coeff4 = Di.*Dj;
    
    
    
    XY = pixAll;
    cnt = 1;
    for j = 1 : size(xuu2,1) - 1
        markY = yuu2(j,1);
        markY_next = yuu2(j+1,1);
        dltY = markY_next - markY;
        for k = 1 : size(xuu2,2) - 1
            markX = xuu2(1,k);
            markX_next = xuu2(1,k+1);
            dltX = markX_next - markX;
            dltXY = dltX*dltY;
            id = find(XY(:,1) >= markX & XY(:,1) < markX_next & XY(:,2) >= markY & XY(:,2) < markY_next);
            storeXY{cnt,1} = XY(id,:);
            storeXY{cnt,2} = [AA(j,k) AA(j,k+1) AA(j+1,k) AA(j+1,k+1)]./dltXY;
            storeXY{cnt,3} = [(markX_next-XY(id,1)).*(markY_next-XY(id,2)) (-markX+XY(id,1)).*(markY_next-XY(id,2)) (markX_next-XY(id,1)).*(-markY+XY(id,2)) (-markX+XY(id,1)).*(-markY+XY(id,2))];
            storeXY{cnt,4} = dot(repmat(storeXY{cnt,2},length(id),1)',storeXY{cnt,3}')';
            storeXY{cnt,5} = id;
            cnt = cnt + 1;
        end
    end
    
    b = zeros(nr,nc);
    b(cell2mat(storeXY(:,5))) = cell2mat(storeXY(:,4));
    
    return;
    %     Y1 = 1;
    %     Y3 = 1;
    %     Y4 = 2;
    %     A = 1.5;
    %     B = -2.5;
    %     D = -0.5;
    %     E = 2.5;
    %     F = -4;
    %     G = 2;
    AA = (AA);
    %     H = 1;
end


% AX = max(X-1,1);
BX = max(X,1);
CX = min(X+1,n);
% DX = min(X+2,n);
% AY = max(Y-1,1);
BY = max(Y,1);
CY = min(Y+1,m);
% DY = min(Y+2,m);
try
    %     ind1 = sub2ind(size(AA),AY,AX);
catch
    sdghk = 1;
end
% ind2 = sub2ind(size(AA),AY,BX);
% ind3 = sub2ind(size(AA),AY,CX);
% ind4 = sub2ind(size(AA),AY,DX);
% ind5 = sub2ind(size(AA),BY,AX);
ind6 = sub2ind(size(AA),BY,BX);
ind7 = sub2ind(size(AA),BY,CX);
% ind8 = sub2ind(size(AA),BY,DX);
% ind9 = sub2ind(size(AA),CY,AX);
ind10 = sub2ind(size(AA),CY,BX);
ind11 = sub2ind(size(AA),CY,CX);
% ind12 = sub2ind(size(AA),CY,DX);
% ind13 = sub2ind(size(AA),DY,AX);
% ind14 = sub2ind(size(AA),DY,BX);
% ind15 = sub2ind(size(AA),DY,CX);
% ind16 = sub2ind(size(AA),DY,DX);

% V11 = AA(ind1); V12 = AA(ind2); V13 = AA(ind3); V14 = AA(ind4);
% V21 = AA(ind5);
V22 = AA(ind6); V23 = AA(ind7);% V24 = AA(ind8);
% V31 = AA(ind9);
V32 = AA(ind10); V33 = AA(ind11);% V34 = AA(ind12);
% V41 = AA(ind13); V42 = AA(ind14); V43 = AA(ind15); V44 = AA(ind16);


% Num2Fix(AA,wordInfo(21,1),wordInfo(21,2));
temp1 = Num2Fix(coeff1.*V22,wordInfo(22,1),wordInfo(22,2));
temp2 = Num2Fix(coeff2.*V23,wordInfo(22,1),wordInfo(22,2));
temp3 = Num2Fix(coeff3.*V32,wordInfo(22,1),wordInfo(22,2));
temp4 = Num2Fix(coeff4.*V33,wordInfo(22,1),wordInfo(22,2));
bb = Num2Fix(temp1 + temp2 + temp3 + temp4,wordInfo(22,1),wordInfo(22,2));

b = reshape(bb,nr,nc);
return;
if 0
    variable{1,1} = a1; variable{2,1} = a3;
    variable{3,1} = a2; variable{4,1} = a4;
    variable{5,1} = Xuu; variable{6,1} = Yuu;
    variable{7,1} = a1xuu; variable{8,1} = a3yuu;
    variable{9,1} = TempX; variable{10,1} = TempY;
    variable{11,1} = Y; variable{12,1} = X;
    variable{13,1} = Di; variable{14,1} = Dj;
    % variable{15,1} = Y1;
    % variable{16,1} = Y3;
    % variable{17,1} = Y4;
    % variable{18,1} = A;
    % variable{19,1} = B;
    % variable{20,1} = D;
    % variable{21,1} = E;
    % variable{22,1} = F;
    % variable{23,1} = G;
    variable{24,1} = AA;
    % variable{25,1} = H;
    
    inherited = wordInfo([26:56 57 57 57 57 13 13 13 13 15 20 21 22 23 18 19 25 16 18 19 25 17 20 21 22 23],:);
    
    whole = [wordInfo;inherited];
    
    % Temp1 = bilinear2x2Func(V11,V12,V13,V14,Dj,Y1,Y3,Y4,A,B,D,E,F,G,H,inherited,doRounding);
    Temp2 = bilinear2x2Func(V22,V23,Dj,Y1,Y3,Y4,A,B,D,E,F,G,H,inherited,doRounding);
    Temp3 = bilinear2x2Func(V32,V33,Dj,Y1,Y3,Y4,A,B,D,E,F,G,H,inherited,doRounding);
    % Temp4 = bilinear2x2Func(V41,V42,V43,V44,Dj,Y1,Y3,Y4,A,B,D,E,F,G,H,inherited,doRounding);
    
    if 0
        figure,plot(sort(diff(sort([num2fixpt(([Temp1]),sfix(17),2^-4)]))));
        figure,plot((diff(sort([num2fixpt(([Temp1]),sfix(17),2^-4)]))));
        figure,plot(diff((num2fixpt(sort([Temp1]),sfix(17),2^-4)))-diff(sort([Temp1])))
        figure,plot(sort(abs(diff((num2fixpt(sort([Temp1]),sfix(17),2^-4)))-diff(sort([Temp1])))))
        figure,plot(((num2fixpt(([Temp1]),sfix(17),2^-5)))-(([Temp1])))
    end
    
    [bb] = bilinear2x2Func(Temp2,Temp3,Di,Y1,Y3,Y4,A,B,D,E,F,G,H,inherited, doRounding);
    b = reshape(bb,nr,nc);
    
end

end
function newXSAmple = reSample(xuu2,yuu2,xOrigMat,yOrigMat,aTmp_,bTmp_,errorMat,scaleSizeXVec,thrSample)
% midRowMatX = xuu2(size(xuu2,1)/2 : size(xuu2,1)/2 +1 ,:);
% midRowMatY = yuu2(size(xuu2,1)/2 : size(xuu2,1)/2 +1 ,:);
% midColMatLeftX = midRowMatX(:,1:size(midRowMatX,2)/2);
% midColMatLeftY = midRowMatY(:,1:size(midRowMatX,2)/2);

% xOrigMatMidRow = xOrigMat(size(xuu2,1)/2 : size(xuu2,1)/2 +1,1:size(xuu2,2)/2);
% yOrigMatMidRow = yOrigMat(size(xuu2,1)/2 : size(xuu2,1)/2 +1,1:size(xuu2,2)/2);
if mod(size(xuu2,1),2)~=0
    errorMatMidLeft = errorMat(max(1,round(bTmp_(floor(size(xuu2,1)/2)))):round(bTmp_(ceil(size(xuu2,1)/2)+1)),1:round(aTmp_(size(xuu2,2)/2)));
else
    errorMatMidLeft = errorMat(max(1,round(bTmp_(round(size(xuu2,1)/2)))):round(bTmp_(round(size(xuu2,1)/2)+1)),1:round(aTmp_(size(xuu2,2)/2)));
end
errorMatMidLeftSum = mean(errorMatMidLeft);
extraCnt = 1;
data = [];
for t = 1 : length(aTmp_)/2-1
    if mod(size(xuu2,1),2)~=0
        errorMatMidLeft_ = errorMat(round(bTmp_(floor(size(xuu2,1)/2))):round(bTmp_(ceil(size(xuu2,1)/2)+1)), max(1,round(aTmp_(t))):round(aTmp_(t+1)));
        %         errorMatMidLeft_ = errorMat(:, max(1,round(aTmp_(t))):round(aTmp_(t+1)));
    else
        errorMatMidLeft_ = errorMat(round(bTmp_(floor(size(xuu2,1)/2))):round(bTmp_(ceil(size(xuu2,1)/2)+0)), max(1,round(aTmp_(t))):round(aTmp_(t+1)));
        %         errorMatMidLeft_ = errorMat(:, max(1,round(aTmp_(t))):round(aTmp_(t+1)));
    end
    %     errorMatMidLeftSum_(t,:) = mean(errorMatMidLeft_(errorMatMidLeft_(:)~=0));
    errorMatMidLeftSum_(t,:) = mean(errorMatMidLeft_(:));
    %     errorMatMidLeftSum_(t,:) = mean(errorMatMidLeft_(errorMatMidLeft_(:)>0.5));
    if errorMatMidLeftSum_(t) > thrSample
        %         extraSampleX(extraCnt,:) = aTmp_(t) + 16; %scaleSizeXVec/2;
        extraSampleX(extraCnt,:) = aTmp_(t) + scaleSizeXVec/2;
        data = [data;t];
        extraCnt = extraCnt + 1;
    end
end

leftXSAmple = unique([aTmp_(1:length(aTmp_)/2) extraSampleX']);
diffXSample = diff(leftXSAmple);
diffXSample0 = diffXSample;
leftXSAmple0 = leftXSAmple;
if sum(diffXSample == 8) == 0
    % if sum(diffXSample == scaleSizeXVec/2) == 0
    diffXSample_ = diffXSample(end:-1:1);
    rightXSAmple = aTmp_(length(aTmp_)/2+1)+[0 cumsum(diffXSample_)];
    dltSample = aTmp_(length(aTmp_)/2 +1)-aTmp_(1);
    % newXSAmple = [leftXSAmple leftXSAmple + dltSample];
    newXSAmple = [leftXSAmple rightXSAmple];
else
    %     diffXSample(diffXSample < 16) = 16;
    diffXSample(diffXSample < scaleSizeXVec/2) = scaleSizeXVec/2;
    diffXSample_ = diffXSample(end:-1:1);
    leftXSAmpleNew = leftXSAmple(end) - [0 cumsum(diffXSample_)];
    leftXSAmpleNew = leftXSAmpleNew(end:-1:1);
    id = find(leftXSAmpleNew < 0);
    if isempty(id)
        leftXSAmpleNew__ = [leftXSAmpleNew- min(diffXSample): -min(diffXSample):-100];
        leftXSAmpleNew = [leftXSAmpleNew__(end:-1:1) leftXSAmpleNew];
        id = find(leftXSAmpleNew < 0);
    end
    leftXSAmpleNew = leftXSAmpleNew(id(end):end);
    diffXSample = diff(leftXSAmpleNew);
    diffXSample_ = diffXSample(end:-1:1);
    rightXSAmple = aTmp_(length(aTmp_)/2+1)+[0 cumsum(diffXSample_)];
    newXSAmple1 = [leftXSAmpleNew rightXSAmple];
    
    diffXSample = diffXSample0;
    leftXSAmple = leftXSAmple0;
    %     diffXSample(diffXSample < 16) = 16;
    diffXSample(diffXSample < scaleSizeXVec/2) = scaleSizeXVec/2;
    leftXSAmpleNew = leftXSAmple(1) + [0 cumsum(diffXSample)];
    %     leftXSAmpleNew = leftXSAmpleNew(end:-1:1);
    id = find(leftXSAmpleNew < mean(aTmp_));
    leftXSAmpleNew = leftXSAmpleNew(1:id(end));
    diffXSample = diff(leftXSAmpleNew);
    diffXSample_ = diffXSample(end:-1:1);
    rightXSAmple = 2* mean(aTmp_) - leftXSAmpleNew(end) +[0 cumsum(diffXSample_)];
    newXSAmple2 = [leftXSAmpleNew rightXSAmple];
    
    if (length(newXSAmple1) < length(newXSAmple2)) && ((sum(diff(newXSAmple1)) < sum(diff(newXSAmple2))))
        newXSAmple = newXSAmple1;
    else
        newXSAmple = newXSAmple2;
    end
    
end
end
function dltLutYY = MakeOfst1(imValidY,dltLutY,vldMat)
try
    for ii = 1 : size(imValidY,1)
        id = find(imValidY(ii,1:midMat) ~= 0);
        if ~isempty(id)
            imValidY(ii,id) = imValidY(ii,id(end));
        end
    end
    dltLutY(~vldMat) = imValidY(imValidY~=0);
    dltLutYY = dltLutY;
catch
    dltLutYY = dltLutY;
end
end

function dltLutYY = MakeOfst2(imValidY,dltLutY,vldMat)
try
    imValidYUp = imValidY(1:round(size(imValidY,1)/2),:);
    imValidYDown = imValidY(round(size(imValidY,1)/2)+1:end,:);
    
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
    imValidY = [imValidYUp; imValidYDown];
    dltLutY(~vldMat) = imValidY(imValidY~=0);
    dltLutYY = dltLutY;
catch
    dltLutYY = dltLutY;
end
end