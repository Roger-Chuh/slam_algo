function [origImgL1, origImgR1, undistImgL1, undistImgR1, rectImgL1, rectImgR1, meanError] = CalibStereoFishEyeUse(inputDir,paraDir)

global cfg

workDir = pwd;
% inputDirL = fullfile(workDir,'Left');
% inputDirR = fullfile(workDir,'Right');
inputDirL = fullfile(inputDir,'Left');
inputDirR = fullfile(inputDir,'Right');

if cfg.is_yuv
    readYUV( inputDir ,cfg.img_width, cfg.img_height, cfg.img_scale);
end


% if ~isdir(inputDirL)
MakeDirIfMissing(inputDirL);
MakeDirIfMissing(inputDirR);

delete(fullfile(inputDirL,'*'));
delete(fullfile(inputDirR,'*'));


if cfg.calib_stereo
    if cfg.switch_lr
        copyfile(fullfile(inputDir,'imgR*'), inputDirL);
        copyfile(fullfile(inputDir,'imgL*'), inputDirR);
    else
        copyfile(fullfile(inputDir,'imgL*'), inputDirL);
        copyfile(fullfile(inputDir,'imgR*'), inputDirR);
    end
    changeExtensionByEval(inputDirL);
    changeExtensionByEval(inputDirR)
else
    changeExtensionByEval(inputDir);
    copyfile(fullfile(inputDir,'imgR*'), inputDirL);
    copyfile(fullfile(inputDir,'imgR*'), inputDirR);
    %         copyfile(fullfile(inputDir,strcat('*.',cfg.img_suffix)), inputDirL);
end
% else
%     delete(fullfile(inputDirL,'*'));
%     delete(fullfile(inputDirR,'*'));
% end
if 0
    delete(strcat(inputDirL,'\*'));
    delete(strcat(inputDirR,'\*'));
end


try
    leftList = getImgLst(inputDirL,'png');
    rightList = getImgLst(inputDirR,'png');
catch
    try
        leftList = getImgLst(inputDirL,'bmp');
        rightList = getImgLst(inputDirR,'bmp');
    catch
        leftList = getImgLst(inputDirL,'jpg');
        rightList = getImgLst(inputDirR,'jpg');
    end
end




if cfg.calib_stereo
    [camParamL, cbcXYL, cbGridL, configL, undistImgL, cbcXYTmpL, cbGridTmpL, goodIdL, ~,origImgL] = CalibFishEyeUse(inputDirL);
    [camParamR, cbcXYR, cbGridR, configR, undistImgR, cbcXYTmpR, cbGridTmpR, goodIdR, ~, origImgR] = CalibFishEyeUse(inputDirR);
else
    [camParamL, cbcXYL, cbGridL, configL, undistImgL, cbcXYTmpL, cbGridTmpL, goodIdL, ~,origImgL] = CalibFishEyeUse(inputDirL);
    camParamR = camParamL;
end
load(fullfile(inputDirL,'oCamModelL.mat'));

if cfg.calib_stereo
    load(fullfile(inputDirR,'oCamModelR.mat'));
    for i = 1 : length(cbcXYL)
        
        
    end
    
    goodId = intersect(goodIdL,goodIdR);
    camParamL.rotVec = camParamL.rotVec(:,ismember(goodIdL,goodId));
    camParamL.rotVecErr = camParamL.rotVecErr(:,ismember(goodIdL,goodId));
    camParamL.tranVec = camParamL.tranVec(:,ismember(goodIdL,goodId));
    camParamL.tranVecErr = camParamL.tranVecErr(:,ismember(goodIdL,goodId));
    camParamR.rotVec = camParamR.rotVec(:,ismember(goodIdR,goodId));
    camParamR.rotVecErr = camParamR.rotVecErr(:,ismember(goodIdR,goodId));
    camParamR.tranVec = camParamR.tranVec(:,ismember(goodIdR,goodId));
    camParamR.tranVecErr = camParamR.tranVecErr(:,ismember(goodIdR,goodId));
    stereoParam = CbCalibStereo2(camParamL, camParamR, cbcXYL(ismember(goodIdL,goodId)), cbcXYR(ismember(goodIdR,goodId)), cbGridL(ismember(goodIdL,goodId)), configL, configR);
    
    
    % stereoParam = CbCalibStereo2(camParamL, camParamR, cbcXYL, cbcXYR, cbGridL, configL, configR);
    [rectImgL, rectImgR] = RectifyImagePair(stereoParam, undistImgL{1}, undistImgR{1});
    
    origImgL1 = origImgL{1}; origImgR1 = origImgR{1};
    undistImgL1 = undistImgL{1}; undistImgR1 = undistImgR{1};
    rectImgL1 = rectImgL; rectImgR1 = rectImgR;
    
    [rectParamL, rectParamR, rotMatLeft, rotMatRight] = GetRectifyParam2(stereoParam, size(undistImgR{1}));
    baseline = norm(stereoParam.transVecRef);
    [nr,nc,~] = size(undistImgR{1});
    
    KL = [stereoParam.focLeft(1) 0 stereoParam.cenLeft(1);0 stereoParam.focLeft(2) stereoParam.cenLeft(2);0 0 1];
    KR = [stereoParam.focRight(1) 0 stereoParam.cenRight(1);0 stereoParam.focRight(2) stereoParam.cenRight(2);0 0 1];
    cbcXYLL = cbcXYL(ismember(goodIdL,goodId));
    cbcXYRR = cbcXYR(ismember(goodIdR,goodId));
    
    leftList1 = leftList(goodIdL);
    leftList1 = leftList1(ismember(goodIdL,goodId));
    
    rightList1 = rightList(goodIdR);
    rightList1 = rightList1(ismember(goodIdR,goodId));
    
    
    [pt3dX, pt3dY] = meshgrid([0:cfg.cb_size:cfg.cb_size*(cfg.cb_col-1)], [0:cfg.cb_size:cfg.cb_size*(cfg.cb_row-1)]);
    pt3d = [pt3dX(:) pt3dY(:)];
    pt3d(:,3) = 0;
    
    [rectParamL, rectParamR, rotMatLeft, rotMatRight,  intrMatLeftNew, intrMatRightNew] = GetRectifyParam2(stereoParam, [nr nc]);
    KRectL = rectParamL.rectIntrMat;
    
    princpPtL = intrMatLeftNew(1:2,3);
    princpPtR = intrMatRightNew(1:2,3);
    
    
    
    U_sameL = ocam_undistort_map(oCamModelL,'OutputView','same');
    intrMat_sameL = U_sameL.K';
    
    U_sameR = ocam_undistort_map(oCamModelR,'OutputView','same');
    intrMat_sameR = U_sameR.K';
    try
        for i = 1 : length(cbcXYLL) %length(cbcXYL)
            if 1
                [xr] = normalize_pixel(cbcXYRR{(i)},stereoParam.focRight,stereoParam.cenRight,stereoParam.kcRight,stereoParam.alphaRight);
                [xl] = normalize_pixel(cbcXYLL{(i)},stereoParam.focLeft,stereoParam.cenLeft,stereoParam.kcLeft,stereoParam.alphaLeft);
                
                xrr = pflat((KR)*pextend(xr));
                xll = pflat((KL)*pextend(xl));
            end
            
            if 0
                pixRectR = Orig2RectFishEye(cbcXYRR{(i)}', [], intrMatRightNew, rotMatRight,  [] ,oCamModelR);
                pixRectL = Orig2RectFishEye(cbcXYLL{(i)}', [], intrMatLeftNew, rotMatLeft,  [] ,oCamModelL);
            else
                pixRectR = Orig2Rect(xrr(1:2,:)', KR, intrMatRightNew, rotMatRight, stereoParam.kcRight);
                
                pixRectL = Orig2Rect(xll(1:2,:)', KL, intrMatLeftNew, rotMatLeft, stereoParam.kcLeft);
            end
            
            %              dispTemp= abs(pixRectL(:,1) - pixRectR(:,1));
            
            
            if ~cfg.isRotate
                dispTemp= abs(pixRectL(:,1) - pixRectR(:,1));
                epilineErr = abs(pixRectL(:,2) - pixRectR(:,2));
            else
                dispTemp= abs(pixRectL(:,2) - pixRectR(:,2));
                epilineErr = abs(pixRectL(:,1) - pixRectR(:,1));
            end
            
            
            depthTemp = intrMatLeftNew(1,1) * norm(stereoParam.transVecRef) ./ (dispTemp + (princpPtR(1) - princpPtL(1)));
            

           [XYZ] = GetXYZFromDepth(intrMatLeftNew, pixRectL, depthTemp);
           
           X = reshape(XYZ(:,1),cfg.cb_row, cfg.cb_col)';
           Y = reshape(XYZ(:,2),cfg.cb_row, cfg.cb_col)';
           Z = reshape(XYZ(:,3),cfg.cb_row, cfg.cb_col)';
           xyz_comp = [X(:) Y(:) Z(:)];
           xyzMat = permute(reshape(permute(xyz_comp, [1,3,2]), [cfg.cb_col, cfg.cb_row, 3]), [2,1,3]);
           
           lenHori = sqrt(sum(diff(xyzMat,1,2).^2,3));
           lenVerti = sqrt(sum(diff(xyzMat,1,1).^2,3));
           
           horiErr = mean(abs(lenHori(:) - cfg.cb_size));
           vertiErr = mean(abs(lenVerti(:) - cfg.cb_size));
           
           xyErr(i,:) = [horiErr vertiErr];
            
            if 0
                nim_sameL = ocam_undistort(imread(leftList1{i}),U_sameL);
                nim_sameR = ocam_undistort(imread(rightList1{i}),U_sameR);
                [rectImgLtemp, rectImgRtemp] = RectifyImagePair(stereoParam, nim_sameL, nim_sameR);
                
%                 rectImgLtemp1 = ImgTransform_123(nim_sameL, intrMat_sameL, rotMatLeft);
%                 rectImgRtemp1 = ImgTransform_123(nim_sameR, intrMat_sameR, rotMatRight);
                
               figure,subplot(1,2,1);imshow(rectImgRtemp);hold on;plot(pixRectR(:,1), pixRectR(:,2),'.r'); subplot(1,2,2),imshow(nim_sameR);hold on;plot(cbcXYRR{(i)}(1,:), cbcXYRR{(i)}(2,:),'.r')
               figure,subplot(1,2,1);imshow(rectImgLtemp);hold on;plot(pixRectL(:,1), pixRectL(:,2),'.r'); subplot(1,2,2),imshow(nim_sameL);hold on;plot(cbcXYLL{(i)}(1,:), cbcXYLL{(i)}(2,:),'.r')
          
                
            end
            try
                rt = [rodrigues(stereoParam.optPose(1:3,(i))) stereoParam.optPose(4:6,(i));0 0 0 1];
                
            catch
                rt = [rodrigues(camParamL.rotVec(:,(i))) camParamL.tranVec(:,(i));0 0 0 1];
            end
%             cbPt = rt(1:3,1:3)*pextend(cbGridR{1}) + repmat(rt(1:3,4),1,126);
            cbPt = rt(1:3,1:3)*pt3d' + repmat(rt(1:3,4),1,size(pt3d,1));
            
            ptIcs = TransformAndProject(cbPt', KR, rodrigues(stereoParam.rotVecRef), stereoParam.transVecRef);
            if 0
                figure,imshow(zeros(nr,nc));hold on;plot(xrr(1,:),xrr(2,:),'or');plot(ptIcs(:,1),ptIcs(:,2),'.g');
            end
            err(i,1) = norm(mean(abs(xrr(1:2,:)'-ptIcs)));
            
        end
    catch
        askjlh = 1;
    end
    
        fprintf(sprintf('\n### average reprojection error: %0.3f pixel ###\n',mean(err)));

    
      meanError1 = mean(err);
    
    meanError = [meanError1 mean(xyErr,1)];
    
    load(fullfile(inputDirL,'oCamModelL.mat'));
    load(fullfile(inputDirR,'oCamModelR.mat'));
    paraDir = inputDir;
%     meanError = mean(err);
    save(fullfile(paraDir,'calib.mat'),'oCamModelL','oCamModelR','stereoParam','camParamL', 'cbcXYL', 'cbGridL', 'configL', 'cbcXYTmpL', 'cbGridTmpL', 'goodIdL','camParamR', 'cbcXYR', 'cbGridR', 'configR', 'cbcXYTmpR', 'cbGridTmpR', 'goodIdR', 'meanError');
else
    
    meanError = [0 0 0];
    load(fullfile(inputDirL,'oCamModelL.mat'));
    paraDir = inputDir;
    origImgL1 = origImgL{1}; origImgR1 = origImgL{1};
    undistImgL1 = origImgL{1}; undistImgR1 = origImgL{1};
    rectImgL1= origImgL{1}; rectImgR1 = origImgL{1};
    save(fullfile(paraDir,'calib.mat'),'oCamModelL','camParamL', 'cbcXYL', 'cbGridL', 'configL', 'cbcXYTmpL', 'cbGridTmpL', 'goodIdL', 'meanError');
end


end