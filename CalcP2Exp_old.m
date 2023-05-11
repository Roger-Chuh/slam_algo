function [ExpCoord_Reproj, ExpCoord_OnlyP2, errReproj, errOnlyP2, errPt2dCur,interpValGT,interpValCur, weightRaw_reshape___] = CalcP2Exp(obj, candX, candY, weightRaw, idff,pt2dCur, pixGT, imgCur,candX_reshape,candY_reshape,weightRaw_reshape, ProbReprojVecAll_reshape, compMat,depthGTInd,k2cRef0,thetaRng)
% candX = candXX(idff,:);
% candY = candYY(idff,:);

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





weightRaw_reshape___ = NormalizeWeightMax(weightRaw_reshape);
ProbReprojVecAll_reshape___ = NormalizeWeightMax(ProbReprojVecAll_reshape);

% weightRaw_reshape___1 = permute(weightRaw_reshape___, [2 3 1]);
% weightRaw_reshape___11 = weightRaw_reshape___1(:);
% weightRaw_reshape___111 = reshape(weightRaw_reshape___11, size(weightRaw_reshape___1,1)*size(weightRaw_reshape___1,2),[]);
% weightRaw_reshape___1111 = weightRaw_reshape___111';

if size(candX_reshape,1) < size(weightRaw_reshape___,1)
    %     compMat = permute(compMat, [1 3 2]);
    candX_reshape = [compMat;candX_reshape;compMat];
    candY_reshape = [compMat;candY_reshape;compMat];
end

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
end

% [Coeff, IndBilinear] = BilinearRemap([], weightRaw_reshape___(:,:,1), [], [], weightRaw_reshape___(:,:,1), floor(gtPix2Interp), gtPix2Interp - floor(gtPix2Interp));
[interpValGT, Coeff, IndBilinear] = BilinearRemap([], weightRaw_reshape___1111, [], [], weightRaw_reshape___1111, floor(gtPix2Interp), gtPix2Interp - floor(gtPix2Interp));

% [interpValX, CoeffX, IndBilinearX] = BilinearRemap([], weightRaw_reshape___1111, [], [], weightRaw_reshape___1111, floor(curPix2Interp), curPix2Interp - floor(curPix2Interp));

indCur = sub2ind(size(weightRaw_reshape___1111), curPix2Interp(:,2), curPix2Interp(:,1));
interpValCur = weightRaw_reshape___1111(indCur);





weightRaw_reshape___ = NormalizeWeightSum(weightRaw_reshape);
ProbReprojVecAll_reshape___ = NormalizeWeightSum(ProbReprojVecAll_reshape);

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

if 0
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


weightRaw_reshape___ = NormalizeWeightMax(weightRaw_reshape);


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