function [ExpCoord_Reproj, ExpCoord_OnlyP2, errReproj, errOnlyP2, errPt2dCur,interpValGT,interpValCur, weightRaw_reshape___, ProbReprojVecAll_reshape___] = CalcP2Exp(obj, candX, candY, weightRaw, idff,pt2dCur, pixGT, imgCur,candX_reshape,candY_reshape,weightRaw_reshape, ProbReprojVecAll_reshape, compMat,depthGTInd,k2cRef0,thetaRng, Pix,r_cam,tx, ty, tz,ConfigParam)
% candX = candXX(idff,:);
% candY = candYY(idff,:);


newExp = false;

intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,obj.scaleLvl);
if  newExp
    startId = size(compMat,1)+1;
    endId = size(weightRaw_reshape,1) - size(compMat,1);
    
    weightRaw_reshape([startId endId],:,:) = 0;
    weightRaw_reshape(:,[1 end],:) = 0;
end
weightRaw_reshape_11111 = weightRaw_reshape;
% [weightRaw_reshape] = GetMidCoordXY(candX_reshape,candY_reshape, startId, endId,weightRaw_reshape);

wsdvb = 1;

if 0
    checkGT = 100;
    % tempCandX = reshape(candX(idff(checkGT),:),obj.setting.configParam.angleBin,obj.setting.configParam.disparityBin);
    % tempCandY = reshape(candY(idff(checkGT),:),obj.setting.configParam.angleBin,obj.setting.configParam.disparityBin);
    tempCandX = reshape(candX((checkGT),:),obj.setting.configParam.angleBin,obj.setting.configParam.disparityBin);
    tempCandY = reshape(candY((checkGT),:),obj.setting.configParam.angleBin,obj.setting.configParam.disparityBin);
    tempWeight = reshape(weightRaw(idff(checkGT),:),obj.setting.configParam.angleBin,obj.setting.configParam.disparityBin)*1000;
    %                                                 figure,imshow(tempWeight, [])
    tempWeight = tempWeight./sum(tempWeight(:));
    
    expCoord = [dot(tempCandX(:),tempWeight(:)) dot(tempCandY(:),tempWeight(:))];
    
end

% weightRaw_reshape_ = reshape(weightRaw_reshape, size(weightRaw_reshape,1)*size(weightRaw_reshape,2), size(weightRaw_reshape,3));
% weightRaw_reshape_sum = repmat(sum(weightRaw_reshape_),size(weightRaw_reshape_,1),1);
% weightRaw_reshape__ = weightRaw_reshape_./weightRaw_reshape_sum;
% weightRaw_reshape___ = reshape(weightRaw_reshape__, size(weightRaw_reshape));


thetaId = interp1(thetaRng,[1:length(thetaRng)],k2cRef0);



gtPix2Interp_ = [depthGTInd repmat(thetaId,length(depthGTInd),1)];
gtPix2Interp = gtPix2Interp_;
gtPix2Interp(:,1) = gtPix2Interp(:,1) + [0:size(weightRaw_reshape,2):(size(gtPix2Interp,1)-1)*size(weightRaw_reshape,2)]';







% weightRaw_reshape___1 = permute(weightRaw_reshape___, [2 3 1]);
% weightRaw_reshape___11 = weightRaw_reshape___1(:);
% weightRaw_reshape___111 = reshape(weightRaw_reshape___11, size(weightRaw_reshape___1,1)*size(weightRaw_reshape___1,2),[]);
% weightRaw_reshape___1111 = weightRaw_reshape___111';

if size(candX_reshape,1) < size(weightRaw_reshape,1)
    %     compMat = permute(compMat, [1 3 2]);
    candX_reshape = [compMat;candX_reshape;compMat];
    candY_reshape = [compMat;candY_reshape;compMat];
end

if newExp
    [weightRaw_reshape_bk] = GetMidCoordXY(candX_reshape,candY_reshape, startId, endId,weightRaw_reshape);
    weightRaw_reshape = weightRaw_reshape_bk;
end
wsdvb = 1;

weightRaw_reshape___ = NormalizeWeightMax(weightRaw_reshape);
ProbReprojVecAll_reshape___ = NormalizeWeightMax(ProbReprojVecAll_reshape);

weightRaw_reshape___1111 = ReshapeCoord(weightRaw_reshape___);
candX_reshape___1111 = ReshapeCoord(candX_reshape);
candY_reshape___1111 = ReshapeCoord(candY_reshape);



pt2dCurXMat = repmat(pt2dCur(:,1),1,size(candX_reshape,1)*size(candX_reshape,2))';
pt2dCurYMat = repmat(pt2dCur(:,2),1,size(candX_reshape,1)*size(candX_reshape,2))';

[~, pixDist] = NormalizeVector([candX_reshape(:) candY_reshape(:)] - [pt2dCurXMat(:) pt2dCurYMat(:)]);
pixDistMat = reshape(pixDist, size(pt2dCurXMat));
[pixDistMatMin,id] = min(pixDistMat);
[pt2dCurYind, pt2dCurXind] = ind2sub([size(candX_reshape,1) size(candX_reshape,2)], id);


curPix2Interp = [(pt2dCurXind' + [0:size(weightRaw_reshape,2):(size(gtPix2Interp,1)-1)*size(weightRaw_reshape,2)]') pt2dCurYind'];



if 0
    checkId = 12;
    err = norm(pt2dCur(checkId,:)-[candX_reshape(pt2dCurYind(checkId),pt2dCurXind(checkId),checkId) candY_reshape(pt2dCurYind(checkId),pt2dCurXind(checkId),checkId)]);
    
    check = pixDistMatMin(checkId) - err;
    figure,plot(diff(gtPix2Interp(:,1)) - diff(depthGTInd))
end

% [Coeff, IndBilinear] = BilinearRemap([], weightRaw_reshape___(:,:,1), [], [], weightRaw_reshape___(:,:,1), floor(gtPix2Interp), gtPix2Interp - floor(gtPix2Interp));
[interpValGT, Coeff, IndBilinear] = BilinearRemap([], weightRaw_reshape___1111, [], [], weightRaw_reshape___1111, floor(gtPix2Interp), gtPix2Interp - floor(gtPix2Interp));

% [interpValX, CoeffX, IndBilinearX] = BilinearRemap([], weightRaw_reshape___1111, [], [], weightRaw_reshape___1111, floor(curPix2Interp), curPix2Interp - floor(curPix2Interp));

indCur = sub2ind(size(weightRaw_reshape___1111), curPix2Interp(:,2), curPix2Interp(:,1));
interpValCur = weightRaw_reshape___1111(indCur);

weightRaw_reshape0 = weightRaw_reshape;
% weightRaw_reshape(weightRaw_reshape___ < -110.8) = 0;
% weightRaw_reshape(weightRaw_reshape___ < 0.95) = 0;
weightRaw_reshape(weightRaw_reshape___ < -0.8) = 0;


weightRaw_reshape___ = NormalizeWeightSum(weightRaw_reshape);
ProbReprojVecAll_reshape___ = NormalizeWeightSum(ProbReprojVecAll_reshape);




if 0
    pixKey_x = repmat(Pix(:,1),size(candX_reshape,1)*size(candX_reshape,2),1);
    pixKey_x = reshape(pixKey_x,size(Pix,1),size(candX_reshape,1),size(candX_reshape,2));
    pixKey_x = permute(pixKey_x, [2 3 1]);
    pixKey_y = repmat(Pix(:,2),size(candY_reshape,1)*size(candY_reshape,2),1);
    pixKey_y = reshape(pixKey_y,size(Pix,1),size(candY_reshape,1),size(candY_reshape,2));
    pixKey_y = permute(pixKey_y, [2 3 1]);
    
    pixKey_x(candX_reshape == 0) = 0;
    pixKey_y(candY_reshape == 0) = 0;
    
    candXY_reshape_list = [candX_reshape(:) candY_reshape(:)];
    pixXY_reshape_list = [pixKey_x(:) pixKey_y(:)];
    
    candXY_reshape_list_homo = pflat(inv(intrMat)*[candXY_reshape_list ones(size(candXY_reshape_list,1),1)]');
    pixXY_reshape_list_homo = pflat(inv(intrMat)*[pixXY_reshape_list ones(size(pixXY_reshape_list,1),1)]');
    
    
    
    [angle_result1to2,depth2In1] = VisualLocalizer.AnglePredict_Prob(pixXY_reshape_list_homo, candXY_reshape_list_homo,r_cam,tx, ty, tz);
    
    angle_result1P2C_mat = deg2rad(reshape(angle_result1to2, size(candX_reshape)));
    depthCInP = reshape(depth2In1, size(candX_reshape));
    
    dx = diff(candX_reshape);
    dy = diff(candY_reshape);
    
    dTheta = ConfigParam.theta_sample_step;
    dDisp = ConfigParam.disparity_sample_step;
    
    dTheta_mat = repmat(dTheta, size(dx));
    dDisp = repmat(dDisp, size(dx));
    dZ = diff(depthCInP);
    
    
    if 0
        jac = (dTheta./dx).*(dDisp./dy) - (dTheta./dy).*(dDisp./dx);
    else
        jac = (dTheta./dx).*(dZ./dy) - (dTheta./dy).*(dZ./dx);
    end
else
    
    % % %     startId = size(compMat,1)+1;
    % % %     endId = size(candY_reshape,1) - size(compMat,1);
    
    % %     dx = diff(candX_reshape)./2;
    % %     dxx = permute(diff(permute(candX_reshape,[2 1 3]))./2,[2 1 3]);
    % %
    % %
    % %     for i = 1 : size(candY_reshape, 3)
    % %         tpXMat = candX_reshape(startId:endId,:,i);
    % %         tpDx = dx(startId:endId-1,:,i);
    % %         tpDxx = dxx(startId:endId,:,i);
    % %
    % %         tpXMat_mid_row = tpXMat(1:end-1,1:end-1) + tpDx(:,1:end-1);
    % %         tpXMat_mid_col = tpXMat(1:end-1,1:end-1) + tpDxx(1:end-1,:);
    % %     end
    
    
    %     [tpXMat_mid_row, tpXMat_mid_col] = GetMidCoord(candX_reshape, startId, endId);
    %     [tpYMat_mid_row, tpYMat_mid_col] = GetMidCoord(candY_reshape, startId, endId);
    
    
    %     [tpYMat_mid_row, tpYMat_mid_col] = GetMidCoordXY(candX_reshape, candY_reshape, startId, endId,weightRaw_reshape___);
    if 0
        [weightRaw_reshape___] = GetMidCoordXY(candX_reshape,candY_reshape, startId, endId,weightRaw_reshape___);
    end
    %     for i = 1 : size(tpXMat_mid_row, 3)
    %         tpx = 1;
    %
    %
    %
    %     end
    
end


expX_Only2 = candX_reshape.*weightRaw_reshape___;
expX_Only2_ = reshape(expX_Only2, size(expX_Only2,1)*size(expX_Only2,2), size(expX_Only2,3));


expY_Only2 = candY_reshape.*weightRaw_reshape___;
expY_Only2_ = reshape(expY_Only2, size(expY_Only2,1)*size(expY_Only2,2), size(expY_Only2,3));

expX_Reproj = candX_reshape.*ProbReprojVecAll_reshape___;
expX_Reproj_ = reshape(expX_Reproj, size(expX_Reproj,1)*size(expX_Reproj,2), size(expX_Reproj,3));

expY_Reproj = candY_reshape.*ProbReprojVecAll_reshape___;
expY_Reproj_ = reshape(expY_Reproj, size(expY_Reproj,1)*size(expY_Reproj,2), size(expY_Reproj,3));


ExpCoord_OnlyP2 = [sum(expX_Only2_)' sum(expY_Only2_)'];

ExpCoord_Reproj = [sum(expX_Reproj_)' sum(expY_Reproj_)'];


[~, errReproj] = NormalizeVector(pixGT - ExpCoord_Reproj);

[~, errPt2dCur] = NormalizeVector(pixGT - pt2dCur);

[~, errOnlyP2] = NormalizeVector(pixGT - ExpCoord_OnlyP2);

[~, errOnlyP2Y] = NormalizeVector(pixGT(:,2) - ExpCoord_OnlyP2(:,2));



if 0
    figure,plot(errOnlyP2Y);
    figure,hist([errReproj errOnlyP2],100) ;legend('reproj exp','onlyP2 exp');
end



if 0
    tempWeight = tempWeight./max(tempWeight(:));
    idMaxWeight = find(tempWeight(:) > 0.8);
    %                                                 [idMaxWeight] = find(tempWeight(:) == max(tempWeight(:)))
    [weightY,weightX] = ind2sub(size(tempWeight),idMaxWeight);
    
    if 1
        figure,subplot(1,2,1);imshow(imresize(tempWeight,2),[]);subplot(1,2,2);imshow(imgCur);hold on;plot(tempCandX(idMaxWeight),tempCandY(idMaxWeight),'.r');plot(pixGT(checkGT,1),pixGT(checkGT,2),'.g');plot(pt2dCur(checkGT,1),pt2dCur(checkGT,2),'.b');plot(expCoord(1),expCoord(2),'.c');legend('candidate','gt','tracking','exp');
    end
end

if 0
    figure,imshow(imgCur);hold on;plot(pixGT(:,1), pixGT(:,2),'.r');plot(ExpCoord_OnlyP2(:,1), ExpCoord_OnlyP2(:,2),'.g');plot(ExpCoord_Reproj(:,1), ExpCoord_Reproj(:,2),'.b');plot(pt2dCur(:,1),pt2dCur(:,2),'.y');legend('gt','onlyp2','reproj','tracking');
    figure,imshow(imgCur);hold on;plot(pixGT(:,1), pixGT(:,2),'.r');plot(ExpCoord_OnlyP2(:,1), ExpCoord_OnlyP2(:,2),'.g');plot(ExpCoord_Reproj(:,1), ExpCoord_Reproj(:,2),'.b');legend('gt','onlyp2','reproj');
end


weightRaw_reshape___ = NormalizeWeightMax(weightRaw_reshape0);
ProbReprojVecAll_reshape___ = NormalizeWeightMax(ProbReprojVecAll_reshape);

if 0
    featId = 1;
    weightTemp = weightRaw_reshape___(:,:,featId);
    candX_reshape1 = candX_reshape(:,:,featId); candY_reshape1 = candY_reshape(:,:,featId);
    weightThr = 0.9;
    figure,subplot(1,2,1);imshow(weightTemp, []);title(sprintf('weight thr: %0.3f',weightThr));subplot(1,2,2);imshow(imgCur);hold on;plot(pixGT(featId,1), pixGT(featId,2),'ok');plot(ExpCoord_OnlyP2(featId,1),ExpCoord_OnlyP2(featId,2),'xb');plot(pt2dCur(featId,1),pt2dCur(featId,2),'.w');plot(candX_reshape1,candY_reshape1,'.r');plot(candX_reshape1(weightTemp > weightThr),candY_reshape1(weightTemp > weightThr),'.c');plot(candX_reshape1(1,:),candY_reshape1(1,:),'.y');legend('gt','new p2 exp','tracking','all cand','high weight');plot(pixGT(featId,1), pixGT(featId,2),'ok');plot(ExpCoord_OnlyP2(featId,1),ExpCoord_OnlyP2(featId,2),'xb');plot(pt2dCur(featId,1),pt2dCur(featId,2),'.w');plot(candX_reshape1(size(compMat,1)+1,:),candY_reshape1(size(compMat,1)+1,:),'.y');plot(candX_reshape1(size(candY_reshape1,1) - size(compMat,1),:),candY_reshape1(size(candY_reshape1,1) - size(compMat,1),:),'.y');
end



end

function weightRaw_reshape___ = NormalizeWeightMax(weightRaw_reshape)
weightRaw_reshape_ = reshape(weightRaw_reshape, size(weightRaw_reshape,1)*size(weightRaw_reshape,2), size(weightRaw_reshape,3));
% weightRaw_reshape_sum = repmat(sum(weightRaw_reshape_),size(weightRaw_reshape_,1),1);
weightRaw_reshape_sum = repmat(max(weightRaw_reshape_),size(weightRaw_reshape_,1),1);
weightRaw_reshape__ = weightRaw_reshape_./weightRaw_reshape_sum;
weightRaw_reshape___ = reshape(weightRaw_reshape__, size(weightRaw_reshape));
end

function weightRaw_reshape___ = NormalizeWeightSum(weightRaw_reshape)
weightRaw_reshape_ = reshape(weightRaw_reshape, size(weightRaw_reshape,1)*size(weightRaw_reshape,2), size(weightRaw_reshape,3));
weightRaw_reshape_sum = repmat(sum(weightRaw_reshape_),size(weightRaw_reshape_,1),1);
% weightRaw_reshape_sum = repmat(max(weightRaw_reshape_),size(weightRaw_reshape_,1),1);
weightRaw_reshape__ = weightRaw_reshape_./weightRaw_reshape_sum;
weightRaw_reshape___ = reshape(weightRaw_reshape__, size(weightRaw_reshape));
end

function weightRaw_reshape___1111 = ReshapeCoord(weightRaw_reshape___)

weightRaw_reshape___1 = permute(weightRaw_reshape___, [2 3 1]);
weightRaw_reshape___11 = weightRaw_reshape___1(:);
weightRaw_reshape___111 = reshape(weightRaw_reshape___11, size(weightRaw_reshape___1,1)*size(weightRaw_reshape___1,2),[]);
weightRaw_reshape___1111 = weightRaw_reshape___111';

end

function [interpVal, Coeff, IndBilinear] = BilinearRemap(imgRect, imgR, imgG, imgB, image, xyOrig_int_in, xyOrig_frac_in,foundIndIn)

numPix = size(imgR,1)*size(imgR,2);

xyOrig_int_in(xyOrig_int_in(:,1) >= size(imgR,2), 1) = size(imgR,2) - 1;
xyOrig_int_in(xyOrig_int_in(:,1) < 1,1) = 1;


xyOrig_int_in(xyOrig_int_in(:,2) >= size(imgR,1), 2) = size(imgR,1) - 1;
xyOrig_int_in(xyOrig_int_in(:,2) < 1, 2) = 1;

PixNew3_Bilinear_floor1 = xyOrig_int_in;
PixNew3_Bilinear_floor2 = [xyOrig_int_in(:,1)+1 xyOrig_int_in(:,2)];
PixNew3_Bilinear_floor3 = [xyOrig_int_in(:,1) xyOrig_int_in(:,2)+1];
PixNew3_Bilinear_floor4 = [xyOrig_int_in(:,1)+1 xyOrig_int_in(:,2)+1];

indBilinear1 = sub2ind(size(image(:,:,1)),PixNew3_Bilinear_floor1(:,2),PixNew3_Bilinear_floor1(:,1));
indBilinear2 = sub2ind(size(image(:,:,1)),PixNew3_Bilinear_floor2(:,2),PixNew3_Bilinear_floor2(:,1));
indBilinear3 = sub2ind(size(image(:,:,1)),PixNew3_Bilinear_floor3(:,2),PixNew3_Bilinear_floor3(:,1));
indBilinear4 = sub2ind(size(image(:,:,1)),PixNew3_Bilinear_floor4(:,2),PixNew3_Bilinear_floor4(:,1));

coeff1 = (1 - xyOrig_frac_in(:,2)).*(1 - xyOrig_frac_in(:,1));
coeff2 = (1 - xyOrig_frac_in(:,2)).*xyOrig_frac_in(:,1);
coeff3 = xyOrig_frac_in(:,2).*(1 - xyOrig_frac_in(:,1));
coeff4 = xyOrig_frac_in(:,2).*xyOrig_frac_in(:,1);

if 0
    [coeff1, ~] = Num2Fix_unsigned(coeff1, 1,9);
    [coeff2, ~] = Num2Fix_unsigned(coeff2, 0,9);
    [coeff3, ~] = Num2Fix_unsigned(coeff3, 0,9);
    [coeff4, ~] = Num2Fix_unsigned(coeff4, 0,9);
end

Coeff = [coeff1 coeff2 coeff3 coeff4];
IndBilinear = [indBilinear1 indBilinear2 indBilinear3 indBilinear4];

sakjb = 1;
% if ~((max(xyOrig_int_in(:,2)) == min(xyOrig_int_in(:,2))) && (min(xyOrig_int_in(:,2)) == y_in(1)))
%     ccccctttt = ccccctttt + 1;
% end

% % idIn = xyOrig(:,1) > 0 & xyOrig(:,1) <= nc & xyOrig(:,2) > 0 & xyOrig(:,2) <= nr;
% % %                                     idIn = flag_in;
% % foundIndIn = foundInd(idIn);
% % foundIndIn = foundInd(flag_in);
%                                     origInd = sub2ind([nr,nc],xyOrig(idIn,2),xyOrig(idIn,1));
%                                     imgRect([foundIndIn;numPix + foundIndIn;2*numPix + foundIndIn]) = imgOrig([origInd;numPix + origInd;2*numPix + origInd]);
%

cof1 = (coeff1.*double(imgR(indBilinear1)));
cof2 = (coeff2.*double(imgR(indBilinear2)));
cof3 = (coeff3.*double(imgR(indBilinear3)));
cof4 = (coeff4.*double(imgR(indBilinear4)));

interpVal = cof1 + cof2 + cof3 + cof4;


if 0
    cof5 = Num2Fix_unsigned(coeff1.*double(imgG(indBilinear1)), 8,9);
    cof6 = Num2Fix_unsigned(coeff2.*double(imgG(indBilinear2)), 8,9);
    cof7 = Num2Fix_unsigned(coeff3.*double(imgG(indBilinear3)), 8,9);
    cof8 = Num2Fix_unsigned(coeff4.*double(imgG(indBilinear4)), 8,9);
    
    cof9 = Num2Fix_unsigned(coeff1.*double(imgB(indBilinear1)), 8,9);
    cof10 = Num2Fix_unsigned(coeff2.*double(imgB(indBilinear2)), 8,9);
    cof11 = Num2Fix_unsigned(coeff3.*double(imgB(indBilinear3)), 8,9);
    cof12 = Num2Fix_unsigned(coeff4.*double(imgR(indBilinear4)), 8,9);
    
    imgRect(foundIndIn) =  floor(cof1 + cof2 + cof3 + cof4);
    imgRect(numPix + foundIndIn) = floor(cof5 + cof6 + cof7 + cof8);
    imgRect(2*numPix + foundIndIn) = floor(cof9 + cof10 + cof11 + cof12);
    
end
% imgRect(numPix + foundIndIn) = coeff1.*double(imgG(indBilinear1)) + coeff2.*double(imgG(indBilinear2)) + coeff3.*double(imgG(indBilinear3)) + coeff4.*double(imgG(indBilinear4));
% imgRect(2*numPix + foundIndIn) = coeff1.*double(imgB(indBilinear1)) + coeff2.*double(imgB(indBilinear2)) + coeff3.*double(imgB(indBilinear3)) + coeff4.*double(imgB(indBilinear4));

end

function [tpXMat_mid_row, tpXMat_mid_col] = GetMidCoord(candX_reshape, startId, endId)
dx = diff(candX_reshape)./2;
dxx = permute(diff(permute(candX_reshape,[2 1 3]))./2,[2 1 3]);
%
%     startId = size(compMat,1)+1;
%     endId = size(candY_reshape,1) - size(compMat,1);
for i = 1 : size(candX_reshape, 3)
    tpXMat = candX_reshape(startId:endId,:,i);
    tpDx = dx(startId:endId-1,:,i);
    tpDxx = dxx(startId:endId,:,i);
    
    tpXMat_mid_row(:,:,i) = tpXMat(1:end-1,1:end-1) + tpDx(:,1:end-1);
    tpXMat_mid_col(:,:,i) = tpXMat(1:end-1,1:end-1) + tpDxx(1:end-1,:);
end


% for j = 1 : size(candX_reshape, 3)
%    temp = candX_reshape(startId:endId,:,j);
%
%
% end



end

% function [tpXMat_mid_row, tpXMat_mid_col] = GetMidCoordXY(candX_reshape,candY_reshape, startId, endId,weightRaw_reshape)
function [weightRaw_reshape_new] = GetMidCoordXY(candX_reshape,candY_reshape, startId, endId,weightRaw_reshape)

dx = diff(candX_reshape)./2;
dxx = permute(diff(permute(candX_reshape,[2 1 3]))./2,[2 1 3]);
%
%     startId = size(compMat,1)+1;
%     endId = size(candY_reshape,1) - size(compMat,1);

[xGrid, yGrid] = meshgrid(1:size(candX_reshape,2)-1, 1:length(startId:endId)-1);
xyCoord = [xGrid(:) yGrid(:)];
xCoordMat = [xyCoord(:,1) xyCoord(:,1) + 1 xyCoord(:,1) xyCoord(:,1) + 1];
yCoordMat = [xyCoord(:,2) xyCoord(:,2) xyCoord(:,2) + 1 xyCoord(:,2) + 1];
[ind] = sub2ind([length(startId:endId) size(candX_reshape,2)], yCoordMat(:), xCoordMat(:));

[xGrid2, yGrid2] = meshgrid(1:size(candX_reshape,2), 1:length(startId:endId));
xGridWithProb = xGrid2(2:end-1, 2:end-1);
yGridWithProb = yGrid2(2:end-1, 2:end-1);
xyCoordWithProb = [xGridWithProb(:) yGridWithProb(:)];
[indWithProb] = sub2ind([length(startId:endId) size(candX_reshape,2)], xyCoordWithProb(:,2), xyCoordWithProb(:,1));

[xGrid3, yGrid3] = meshgrid(1:size(xGridWithProb,2), 1:size(xGridWithProb,1));
xyCoord3 = [xGrid3(:) yGrid3(:)];
xCoordMat3 = [xyCoord3(:,1) xyCoord3(:,1) + 1 xyCoord3(:,1) xyCoord3(:,1) + 1];
yCoordMat3 = [xyCoord3(:,2) xyCoord3(:,2) xyCoord3(:,2) + 1 xyCoord3(:,2) + 1];
[ind3] = sub2ind([length(startId:endId)-1 size(candX_reshape,2)-1], yCoordMat3(:), xCoordMat3(:));
weightRaw_reshape_new = zeros(size(weightRaw_reshape));

for i = 1 : size(candX_reshape, 3)
    tpXMat = candX_reshape(startId:endId,:,i);
    tpYMat = candY_reshape(startId:endId,:,i);
    wMat = weightRaw_reshape(startId:endId,:,i);
    wMatList = wMat(indWithProb);
    wMatList = wMatList./sum(wMatList);
    
    coord2meanX = tpXMat(ind); coord2meanY = tpYMat(ind);
    coord2meanX = reshape(coord2meanX, size(xCoordMat));
    coord2meanY = reshape(coord2meanY, size(yCoordMat));
    
    meanCoordX = mean(coord2meanX')';
    meanCoordY = mean(coord2meanY')';
    
    meanCoordXMat = reshape(meanCoordX, size(tpXMat)-1);
    meanCoordYMat = reshape(meanCoordY, size(tpYMat)-1);
    
    
    polygonX = reshape(meanCoordXMat(ind3),[],4);
    polygonY = reshape(meanCoordYMat(ind3),[],4);
    
    polygonXX = [polygonX(:,1) polygonX(:,2) polygonX(:,3) polygonX(:,2) polygonX(:,3) polygonX(:,4)];
    polygonYY = [polygonY(:,1) polygonY(:,2) polygonY(:,3) polygonY(:,2) polygonY(:,3) polygonY(:,4)];
    
    
    [~, len1] = NormalizeVector([polygonXX(:,1) polygonYY(:,1)] - [polygonXX(:,2) polygonYY(:,2)]);
    [~, len2] = NormalizeVector([polygonXX(:,1) polygonYY(:,1)] - [polygonXX(:,3) polygonYY(:,3)]);
    [~, len3] = NormalizeVector([polygonXX(:,3) polygonYY(:,3)] - [polygonXX(:,2) polygonYY(:,2)]);
    [~, len4] = NormalizeVector([polygonXX(:,4) polygonYY(:,4)] - [polygonXX(:,5) polygonYY(:,5)]);
    [~, len5] = NormalizeVector([polygonXX(:,4) polygonYY(:,4)] - [polygonXX(:,6) polygonYY(:,6)]);
    [~, len6] = NormalizeVector([polygonXX(:,5) polygonYY(:,5)] - [polygonXX(:,6) polygonYY(:,6)]);
    p1 = (len1 + len2 + len3)./2;  p2 = (len4 + len5 + len6)./2;
    
    s1 = sqrt(p1.*(p1 - len1).*(p1 - len2).*(p1 - len3));
    s2 = sqrt(p2.*(p2 - len4).*(p2 - len5).*(p2 - len6));
    s = s1 + s2;
    
    if 0
        id = 14;
        pgon = polyshape(polygonX(id,[1 2 4 3]),polygonY(id,[1 2 4 3]));
        A = area(pgon) - s(id)
    end
    
    wMatListNew = wMatList./s;
    weightRaw_reshape_new_temp = zeros(length(startId:endId), size(candX_reshape,2));
    weightRaw_reshape_new_temp(indWithProb) = wMatListNew./sum(wMatListNew);
    weightRaw_reshape_new(startId:endId,:,i) = weightRaw_reshape_new_temp;
    if 0
        figure,imshow(zeros(240,320));hold on;plot(tpXMat(:), tpYMat(:), '.r');hold on;plot(meanCoordX(:), meanCoordY(:), '.g');
    end
    
    
    % %     tpDx = dx(startId:endId-1,:,i);
    % %     tpDxx = dxx(startId:endId,:,i);
    % %
    % %     tpXMat_mid_row(:,:,i) = tpXMat(1:end-1,1:end-1) + tpDx(:,1:end-1);
    % %     tpXMat_mid_col(:,:,i) = tpYMat(1:end-1,1:end-1) + tpDxx(1:end-1,:);
end


% for j = 1 : size(candX_reshape, 3)
%    temp = candX_reshape(startId:endId,:,j);
%
%
% end



end