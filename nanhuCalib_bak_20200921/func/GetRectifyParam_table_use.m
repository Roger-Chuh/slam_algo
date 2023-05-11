% function [rectParamL, rectParamR,imgL1,imgR1] = GetRectifyParam_table_use(stereoParam, imgSize,imgLeft, imgRight,varargin)
function  [imgOut, KK_newL] = GetRectifyParam_table_use(stereoParam, imgSize,imgLeft, imgRight,varargin)
global stepDir inputDir0 cfg inputDirRGB

if ~cfg.calib_stereo
    if ~isempty(inputDirRGB)
        inputDir0 = inputDirRGB;
    end
end

if (nargin == 4)
    paraDir = [];
    %     homoDir = 'D:\Temp\20180828\calibFishEyeHomo1';
    %     heightDir = 'D:\Temp\20181213\apriltag7';
    
elseif (nargin == 5)
    paraDir = varargin{1};
else
    error('Too many input arguments');
end

if ndims(imgLeft) == 2
    imgLeft = cat(3, imgLeft,imgLeft,imgLeft);
    imgRight = cat(3, imgRight,imgRight,imgRight);
end
if cfg.calib_stereo
    
    if 0
        [rotMatLeft1, rotMatRight1, intrMatLeftNew1, intrMatRightNew1, ~, ~] = GetRectifyPlane(stereoParam, imgSize);
    else
        if ~cfg.cvt_calib
            [~, ~, rotMatLeft, rotMatRight,  intrMatLeftNew, intrMatRightNew] = GetRectifyParam2(stereoParam, imgSize);
        else
            rotMatLeft = stereoParam.rotMatLeft;
            rotMatRight = stereoParam.rotMatRight;
            intrMatLeftNew = stereoParam.intrMatLeftNew;
            intrMatRightNew = stereoParam.intrMatRightNew;
        end
    end
    
    
    focLeft = stereoParam.focLeft;
    focRight = stereoParam.focRight;
    cenLeft = stereoParam.cenLeft;
    cenRight = stereoParam.cenRight;
    alphaLeft = stereoParam.alphaLeft;
    alphaRight = stereoParam.alphaRight;
    kcLeft = stereoParam.kcLeft;
    kcRight = stereoParam.kcRight;
else
    rotMatLeft = eye(3);
    
end
% Pre-compute the necessary indices and blending coefficients to enable quick rectification:

if 0
    [~,indNewL,ind1L,ind2L,ind3L,ind4L,a1L,a2L,a3L,a4L,imgL1] = rect_index_table(zeros(imgSize(1,1:2)),rotMatLeft,focLeft,cenLeft,kcLeft,alphaLeft,intrMatLeftNew,imgLeft);
    [~,indNewR,ind1R,ind2R,ind3R,ind4R,a1R,a2R,a3R,a4R,imgR1] = rect_index_table(zeros(imgSize(1,1:2)),rotMatRight,focRight,cenRight,kcRight,alphaRight,intrMatRightNew,imgRight);
elseif 0
    [~,indNewL,ind1L,ind2L,ind3L,ind4L,a1L,a2L,a3L,a4L,imgL1] = rect_index_table2(zeros(imgSize(1,1:2)),rotMatLeft,focLeft,cenLeft,kcLeft,alphaLeft,intrMatLeftNew,imgLeft);
    [~,indNewR,ind1R,ind2R,ind3R,ind4R,a1R,a2R,a3R,a4R,imgR1] = rect_index_table2(zeros(imgSize(1,1:2)),rotMatRight,focRight,cenRight,kcRight,alphaRight,intrMatRightNew,imgRight);
elseif 0
    [~,indNewL,ind1L,ind2L,ind3L,ind4L,a1L,a2L,a3L,a4L,imgL1] = rect_index_table3(zeros(imgSize(1,1:2)),rotMatLeft,focLeft,cenLeft,kcLeft,alphaLeft,intrMatLeftNew,imgLeft);
    [~,indNewR,ind1R,ind2R,ind3R,ind4R,a1R,a2R,a3R,a4R,imgR1] = rect_index_table3(zeros(imgSize(1,1:2)),rotMatRight,focRight,cenRight,kcRight,alphaRight,intrMatRightNew,imgRight);
elseif 0
    [~,indNewL,ind1L,ind2L,ind3L,ind4L,a1L,a2L,a3L,a4L,imgL1] = rect_index_table4(zeros(imgSize(1,1:2)),rotMatLeft,focLeft,cenLeft,kcLeft,alphaLeft,intrMatLeftNew,imgLeft);
    [~,indNewR,ind1R,ind2R,ind3R,ind4R,a1R,a2R,a3R,a4R,imgR1] = rect_index_table4(zeros(imgSize(1,1:2)),rotMatRight,focRight,cenRight,kcRight,alphaRight,intrMatRightNew,imgRight);
elseif 0
    [~,indNewL,ind1L,ind2L,ind3L,ind4L,a1L,a2L,a3L,a4L,imgL1] = rect_index_table5(zeros(imgSize(1,1:2)),rotMatLeft,focLeft,cenLeft,kcLeft,alphaLeft,intrMatLeftNew,imgLeft);
    [~,indNewR,ind1R,ind2R,ind3R,ind4R,a1R,a2R,a3R,a4R,imgR1] = rect_index_table5(zeros(imgSize(1,1:2)),rotMatRight,focRight,cenRight,kcRight,alphaRight,intrMatRightNew,imgRight);
else
    if 0
        close all
        [xRect2OrigL, yRect2OrigL,KK_newL, KinitL,kL,imgL1, LutVecRect2OrigX, LutVecRect2OrigY, Rect2OrigSplitIndX, Rect2OrigSplitIndY, LutVecRect2OrigXNum, LutVecRect2OrigYNum] = RemapPixel(zeros(imgSize(1,1:2)),rotMatLeft,focLeft,cenLeft,kcLeft,alphaLeft,intrMatLeftNew,imgLeft,[32 32],0,paraDir,'Left');%[160 120]
        close all
        [xOrig2RectL, yOrig2RectL,~,~,~,~, LutVecOrig2RectX, LutVecOrig2RectY,Orig2RectSplitIndX, Orig2RectSplitIndY,LutVecOrig2RectXNum, LutVecOrig2RectYNum] = RemapPixel(zeros(imgSize(1,1:2)),rotMatLeft,focLeft,cenLeft,kcLeft,alphaLeft,intrMatLeftNew,imgLeft,[32 32],1,paraDir,'Left'); % [140 100] [100 60]
    end
    
    
    crop1 = [23 15];
    crop2 = [3750 2126];
    
    crop1 = [23 15];
    crop2 = [3750-2048 2126-1024-128];
    
    crop1 = [23 15];
    crop2 = [3750-2048-32 2126-1024-128];
    
    
    % temp
    if (imgSize(2)) > 3500
        crop1 = [23 15];
        crop2 = [3750 2126-1024-512-128];
    end
    
    if 0
        crop1 = [23 15];
        crop2 = [582 414+64];
    end
    % 720p
    if (imgSize(2)) == 1280
        %         crop1 = [17 3];
        %         crop2 = [1136+128 642+64];
        crop1 = [17 3];
        crop2 = [1136+128 642+64];
    end
    if   (imgSize(2)) == 1920
        crop1 = [11 9];
        %     crop1 = [400-256 300+256-16];
        crop2 = [1834 1048];
    end
    if (imgSize(2)) == 640
        %         crop1 = [17 3];
        %         crop2 = [1136+128 642+64];
        crop1 = [17 3];
        crop2 = [1136+128 - 640  642+64 - 240];
    end
    
    %  crop1 = [23 15];
    % crop2 = [3750 2126];
    if imgSize(2) <= 640
        load(fullfile(stepDir, '640.mat'));
        
        
    elseif imgSize(2) <= 1280 && imgSize(2) > 640
        load(fullfile(stepDir, '1280.mat'));
    elseif imgSize(2) <= 1920 && imgSize(2) > 1280
        load(fullfile(stepDir, '1920.mat'));
    elseif imgSize(2) > 3500
        load(fullfile(stepDir, '4k.mat'));
    else
        sbghd = 1;
    end
    
    if 0
        if imgSize(2) <= 1280 || imgSize(2) > 640
            load(fullfile(stepDir, '1280.mat'));
        end
        
        if imgSize(2) <= 1920 || imgSize(2) > 1280
            load(fullfile(stepDir, '1920.mat'));
        end
        if imgSize(2) > 3500
            load(fullfile(stepDir, '4k.mat'));
        end
    end
    
    crop1 = [1 1];
    crop2 = imgSize([2 1]);
    
    
    [xGrid,yGrid] = meshgrid(1:imgSize(2), 1:imgSize(1));
    
    
    
    if isempty(paraDir)
        %% left
        div_level = 2; 4;
        if ~cfg.calib_stereo
            if ~cfg.cvt_calib
                intrMatLeftNew = [stereoParam.foc(1) 0 stereoParam.cen(1); 0 stereoParam.foc(2) stereoParam.cen(2); 0 0 1];
            else
                intrMatLeftNew = stereoParam.CameraMatrixRGB_new;
            end
            focLeft = stereoParam.foc;
            cenLeft = stereoParam.cen;
            kcLeft = stereoParam.kc;
            alphaLeft = stereoParam.alpha;
            %             intrMatLeftNew = intrMat;
            if ~cfg.cvt_calib
                intrMatLeftNew(1,1) = focLeft(1) + cfg.ext_focal;
                intrMatLeftNew(2,2) = focLeft(2) + cfg.ext_focal;
            end
        end
        
        
        [~, ~, xOrig2RectL, yOrig2RectL,~,~,~,~, LutVecOrig2RectX, LutVecOrig2RectY,Orig2RectSplitIndX, Orig2RectSplitIndY,LutVecOrig2RectXNum, LutVecOrig2RectYNum,Orig2RectNameMatX,Orig2RectNameMatY] = RemapPixel_tmp2(zeros([imgSize(1) imgSize(2)/div_level]),rotMatLeft,focLeft,cenLeft,kcLeft,alphaLeft,intrMatLeftNew,imgLeft(:,1:imgSize(2)/div_level,:),[64 64],1,paraDir,'Left',newXSample0,newYSample0);%[160 120]
        [~, ~, xRect2OrigL, yRect2OrigL,KK_newL, KinitL,kL,imgL1, LutVecRect2OrigX, LutVecRect2OrigY, Rect2OrigSplitIndX, Rect2OrigSplitIndY, LutVecRect2OrigXNum, LutVecRect2OrigYNum,Rect2OrigNameMatX,Rect2OrigNameMatY,imgOut] = RemapPixel_tmp2(zeros([imgSize(1) imgSize(2)/1]),rotMatLeft,focLeft,cenLeft,kcLeft,alphaLeft,intrMatLeftNew,imgLeft(:,1:imgSize(2)/1,:),[64 64],0,paraDir,'Left',newXSample,newYSample);%[160 120]
        jszgjgf = 1;
% % %         xRect2OrigLT = xRect2OrigL'; yRect2OrigLT = yRect2OrigL';
% % %         mapVec = [size(xRect2OrigL,2);size(xRect2OrigL,1);xRect2OrigLT(:); yRect2OrigLT(:)];
% % %         fida = fopen(fullfile(inputDir0, 'inverse_mapping_coord_L.txt'),'w');
% % % %         fprintf(fida,'%0.5f\n',round(mapVec));
% % %         fprintf(fida,'%d\n',round(mapVec));
% % %         fclose(fida);
%         [fianlVec, errLen] = MakeFinalVecUse('L',inputDir0,crop1,crop2, LutVecOrig2RectXNum,LutVecRect2OrigXNum,LutVecRect2OrigX,LutVecOrig2RectX,Orig2RectNameMatX,Rect2OrigNameMatX,LutVecOrig2RectY,LutVecRect2OrigY,LutVecOrig2RectYNum);
        if 0
            imgL2 = RectImg(xOrig2RectL, yOrig2RectL, xRect2OrigL, yRect2OrigL, imgLeft,KK_newL, KinitL,kL,rotMatLeft,imgLeft,paraDir,'Left');
        end
% % % % %         if ~cfg.isRotate
% % % % % %         LutDec2HexUse(inputDir0, 'L');
% % % % %         else
% % % % %             
% % % % %         end
        if ~cfg.calib_stereo
            [fianlVec, errLen] = MakeFinalVecUse('L',inputDir0,crop1,crop2, LutVecOrig2RectXNum,LutVecRect2OrigXNum,LutVecRect2OrigX,LutVecOrig2RectX,Orig2RectNameMatX,Rect2OrigNameMatX,LutVecOrig2RectY,LutVecRect2OrigY,LutVecOrig2RectYNum);
            LutDec2HexUse(inputDir0, 'L');
            
            fid1 = fopen(fullfile(inputDir0,'cam_param1.txt'),'w');%????
            fprintf(fid1,'%d %d %d',imgSize(2), imgSize(1), cfg.switch_lr);
            fprintf(fid1, '\n');
            fprintf(fid1,'pinhole');
            fprintf(fid1, '\n');
            if ~cfg.isRotate
                fprintf(fid1,'%06f %06f %06f %06f',KK_newL(1,1),KK_newL(2,2),KK_newL(1,3),KK_newL(2,3));
            else
                fprintf(fid1,'%06f %06f %06f %06f',KK_newL(2,2),KK_newL(1,1),1+imgSize(1)-KK_newL(2,3),KK_newL(1,3));
            end
            if isfield(cfg, 'L2RGB')
                fprintf(fid1, '\n');
                rotVec = rodrigues(cfg.L2RGB(1:3,1:3));
                tVec = cfg.L2RGB(1:3,4);
                fprintf(fid1,'%06f %06f %06f %06f %06f %06f',rotVec(1),rotVec(2),rotVec(3),tVec(1),tVec(2),tVec(3));
            end
            
            %             fprintf(fid1, '\n');
            %             fprintf(fid1,'%06f %06f %06f %06f',KK_newR(1,1),KK_newR(2,2),KK_newR(1,3),KK_newR(2,3));
            %             fprintf(fid1, '\n');
            %             fprintf(fid1,'%06f %06f %06f %06f %06f %06f',0,0,0,-norm(stereoParam.transVecRef),0,0);
            fprintf(fid1, '\n');
            fclose(fid1);
            figure,imshow([rgb2gray(imgLeft) uint8(imgOut)]);
            saveas(gcf,fullfile(inputDir0,sprintf('pinhole_mono__%05d.png',2)));
            if 1
                xRect2OrigLT = xRect2OrigL'; yRect2OrigLT = yRect2OrigL';
                mapVec = [size(xRect2OrigL,2);size(xRect2OrigL,1);xRect2OrigLT(:); yRect2OrigLT(:)];
                mapVec(3:end) = round(mapVec(3:end))-1;
                mapVec(mapVec < 0) = 0;
            else
                
                KOrig = [stereoParam.foc(1) 0 stereoParam.cen(1); 0 stereoParam.foc(2) stereoParam.cen(2); 0 0 1];
                pixRect = Orig2Rect([xGrid(:) yGrid(:)], KOrig, intrMatLeftNew, rotMatLeft,stereoParam.kc);
                pixRectMat = reshape(pixRect(:,1), size(xGrid));
                err1 = abs(pixRectMat(:,1:size(xOrig2RectL,2)) - xOrig2RectL);
                
                pixDistort = remapRect([xGrid(:) yGrid(:)]', intrMatLeftNew, KOrig,stereoParam.kc, rotMatLeft);
                err2 = abs(reshape(pixDistort(:,1), size(xGrid)) - xRect2OrigL);
                
            end
            
            if 1
                fida = fopen(fullfile(inputDir0, 'inverse_mapping_coord_L.txt'),'w');
                %         fprintf(fida,'%0.5f\n',round(mapVec));
                fprintf(fida,'%d\n',round(mapVec));
                fclose(fida);
            end
            
            return;
        else
            if ~cfg.isRotate
                [fianlVec, errLen] = MakeFinalVecUse('L',inputDir0,crop1,crop2, LutVecOrig2RectXNum,LutVecRect2OrigXNum,LutVecRect2OrigX,LutVecOrig2RectX,Orig2RectNameMatX,Rect2OrigNameMatX,LutVecOrig2RectY,LutVecRect2OrigY,LutVecOrig2RectYNum);
                LutDec2HexUse(inputDir0, 'L');
                xRect2OrigLT = xRect2OrigL'; yRect2OrigLT = yRect2OrigL';
                mapVec = [size(xRect2OrigL,2);size(xRect2OrigL,1);xRect2OrigLT(:); yRect2OrigLT(:)];
                mapVec(3:end) = round(mapVec(3:end))-1;
                mapVec(mapVec < 0) = 0;
                
                fida = fopen(fullfile(inputDir0, 'inverse_mapping_coord_L.txt'),'w');
                %         fprintf(fida,'%0.5f\n',round(mapVec));
                fprintf(fida,'%d\n',round(mapVec));
                fclose(fida);
            else
                [fianlVec, errLen] = MakeFinalVecUse('R',inputDir0,crop1,crop2, LutVecOrig2RectXNum,LutVecRect2OrigXNum,LutVecRect2OrigX,LutVecOrig2RectX,Orig2RectNameMatX,Rect2OrigNameMatX,LutVecOrig2RectY,LutVecRect2OrigY,LutVecOrig2RectYNum);
                LutDec2HexUse(inputDir0, 'R');
                xRect2OrigLT = xRect2OrigL'; yRect2OrigLT = yRect2OrigL';
                mapVec = [size(xRect2OrigL,2);size(xRect2OrigL,1);xRect2OrigLT(:); yRect2OrigLT(:)];
                mapVec(3:end) = round(mapVec(3:end))-1;
                mapVec(mapVec < 0) = 0;
                
                fida = fopen(fullfile(inputDir0, 'inverse_mapping_coord_R.txt'),'w');
                %         fprintf(fida,'%0.5f\n',round(mapVec));
                fprintf(fida,'%d\n',round(mapVec));
                fclose(fida);
            end
            
        end
        if cfg.is_kepler
            cfg.KK_newL = KK_newL;
        end
        %% right
        [~, ~, xOrig2RectR, yOrig2RectR,~,~,~,~, LutVecOrig2RectXR, LutVecOrig2RectYR,Orig2RectSplitIndXR, Orig2RectSplitIndYR,LutVecOrig2RectXNumR, LutVecOrig2RectYNumR,Orig2RectNameMatXR,Orig2RectNameMatYR] = RemapPixel_tmp2(zeros([imgSize(1) imgSize(2)/div_level]),rotMatRight,focRight,cenRight,kcRight,alphaRight,intrMatRightNew,imgRight(:,1:imgSize(2)/div_level,:),[64 64],1,paraDir,'Right',newXSample0,newYSample0);%[160 120]
        [~, ~, xRect2OrigR, yRect2OrigR,KK_newR, KinitR,kR,imgR1, LutVecRect2OrigXR, LutVecRect2OrigYR, Rect2OrigSplitIndXR, Rect2OrigSplitIndYR, LutVecRect2OrigXNumR, LutVecRect2OrigYNumR,Rect2OrigNameMatXR,Rect2OrigNameMatYR,imgOutR] = RemapPixel_tmp2(zeros([imgSize(1) imgSize(2)/1]),rotMatRight,focRight,cenRight,kcRight,alphaRight,intrMatRightNew,imgRight(:,1:imgSize(2)/1,:),[64 64],0,paraDir,'Right',newXSample,newYSample);%[160 120]
%         [fianlVecR, errLenR] = MakeFinalVecUse('R',inputDir0,crop1,crop2, LutVecOrig2RectXNumR,LutVecRect2OrigXNumR,LutVecRect2OrigXR,LutVecOrig2RectXR,Orig2RectNameMatXR,Rect2OrigNameMatXR,LutVecOrig2RectYR,LutVecRect2OrigYR,LutVecOrig2RectYNumR);
        if 0
            imgR2 = RectImg(xOrig2RectR, yOrig2RectR, xRect2OrigR, yRect2OrigR, imgRight,KK_newR, KinitR,kR,rotMatRight,imgRight,paraDir,'Right');
            imgR222 = RectImg(xOrig2RectR, yOrig2RectR, xRect2OrigR, yRect2OrigR, imresize(imgRight,1/2),KK_newR, KinitR,kR,rotMatRight,imresize(imgRight,1/2),paraDir,'Right',2);
            VR = imresize(imgRight,1/2)-10;
            imgR222_v = RectImg(xOrig2RectR, yOrig2RectR, xRect2OrigR, yRect2OrigR, VR,KK_newR, KinitR,kR,rotMatRight,VR,paraDir,'Right',2);
            imageRight1 = imresize(imgRight,0.5);
            % imgL22 = uint8(zeros(size(imgL222)));
            imgR22 = [imgR222 imgR222];
            imgR22(:,1:2:end,:) = imgR222;  imgR22(:,2:2:end,:) = imgR222_v;
            imgR11 = uint8(zeros(size(imgR22)));
            imgR11(:,1:2:end,:) = imageRight1;  imgR11(:,2:2:end,:) = VR;
        end
        if ~cfg.isRotate
            [fianlVecR, errLenR] = MakeFinalVecUse('R',inputDir0,crop1,crop2, LutVecOrig2RectXNumR,LutVecRect2OrigXNumR,LutVecRect2OrigXR,LutVecOrig2RectXR,Orig2RectNameMatXR,Rect2OrigNameMatXR,LutVecOrig2RectYR,LutVecRect2OrigYR,LutVecOrig2RectYNumR);
            LutDec2HexUse(inputDir0, 'R');
            xRect2OrigRT = xRect2OrigR'; yRect2OrigRT = yRect2OrigR';
            mapVecR = [size(xRect2OrigR,2);size(xRect2OrigR,1);xRect2OrigRT(:); yRect2OrigRT(:)];
            mapVecR(3:end) = round(mapVecR(3:end))-1;
            mapVecR(mapVecR < 0) = 0;
            
            fida = fopen(fullfile(inputDir0, 'inverse_mapping_coord_R.txt'),'w');
            %         fprintf(fida,'%0.5f\n',round(mapVec));
            fprintf(fida,'%d\n',round(mapVecR));
            fclose(fida);
            
        else
            [fianlVecR, errLenR] = MakeFinalVecUse('L',inputDir0,crop1,crop2, LutVecOrig2RectXNumR,LutVecRect2OrigXNumR,LutVecRect2OrigXR,LutVecOrig2RectXR,Orig2RectNameMatXR,Rect2OrigNameMatXR,LutVecOrig2RectYR,LutVecRect2OrigYR,LutVecOrig2RectYNumR);
            LutDec2HexUse(inputDir0, 'L');
            
            xRect2OrigRT = xRect2OrigR'; yRect2OrigRT = yRect2OrigR';
            mapVecR = [size(xRect2OrigR,2);size(xRect2OrigR,1);xRect2OrigRT(:); yRect2OrigRT(:)];
             mapVecR(3:end) = round(mapVecR(3:end))-1;
            mapVecR(mapVecR < 0) = 0;
            
            fida = fopen(fullfile(inputDir0, 'inverse_mapping_coord_L.txt'),'w');
            %         fprintf(fida,'%0.5f\n',round(mapVec));
            fprintf(fida,'%d\n',round(mapVecR));
            fclose(fida);
            
        end
        rectImgL = uint8(interp2(xGrid,yGrid,double(imgLeft(:,:,1)),xRect2OrigL,yRect2OrigL));
        rectImgR = uint8(interp2(xGrid,yGrid,double(imgRight(:,:,1)),xRect2OrigR,yRect2OrigR));
        save(fullfile(inputDir0,'reMap.mat'),'xGrid','yGrid', 'xRect2OrigL', 'yRect2OrigL', 'xRect2OrigR', 'yRect2OrigR');

        
        figure,imshowpair(imgOut, imgOutR);
        
        
%         fid1 = fopen(fullfile(inputDir0,'cam_param_pinhole.txt'),'w');%????
        fid1 = fopen(fullfile(inputDir0,'cam_param1.txt'),'w');%????
        fprintf(fid1,'%d %d %d',imgSize(2), imgSize(1), cfg.switch_lr);
        fprintf(fid1, '\n');
        fprintf(fid1,'pinhole');
        fprintf(fid1, '\n');
        if ~cfg.isRotate
            fprintf(fid1,'%06f %06f %06f %06f',KK_newL(1,1),KK_newL(2,2),KK_newL(1,3),KK_newL(2,3));
            fprintf(fid1, '\n');
            fprintf(fid1,'%06f %06f %06f %06f',KK_newR(1,1),KK_newR(2,2),KK_newR(1,3),KK_newR(2,3));
            fprintf(fid1, '\n');
            KK_new = KK_newL;
        else
            fprintf(fid1,'%06f %06f %06f %06f',KK_newL(2,2),KK_newL(1,1),1+imgSize(1)-KK_newL(2,3),KK_newL(1,3));
            fprintf(fid1, '\n');
            fprintf(fid1,'%06f %06f %06f %06f',KK_newR(2,2),KK_newR(1,1),1+imgSize(1)-KK_newR(2,3),KK_newR(1,3));
            fprintf(fid1, '\n');
            KK_new = KK_newL;
        end
        fprintf(fid1,'%06f %06f %06f %06f %06f %06f',0,0,0,-norm(stereoParam.transVecRef),0,0);
        fprintf(fid1, '\n');
        fclose(fid1);
        
        
        
        saveas(gcf,fullfile(inputDir0,sprintf('pinhole_%05d.png',1)));
        return;
    end
%     crop1 = [1 1];
%     crop2 = imgSize([2 1]);
    if ~isempty(paraDir)
        div_level = 2;  4;
        %% left
        if cfg.calib_stereo
            intrMat = [stereoParam.focLeft(1) 0 stereoParam.cenLeft(1); 0 stereoParam.focLeft(2) stereoParam.cenLeft(2); 0 0 1];
        else
            intrMat = [stereoParam.foc(1) 0 stereoParam.cen(1); 0 stereoParam.foc(2) stereoParam.cen(2); 0 0 1];
            intrMatLeftNew = intrMat;
            
            intrMatLeftNew(1,1) = intrMat(1,1) + cfg.ext_focal;
            intrMatLeftNew(2,2) = intrMat(2,2) + cfg.ext_focal;
        end
        [~, ~, xOrig2RectL, yOrig2RectL,~,~,~,~, LutVecOrig2RectX, LutVecOrig2RectY,Orig2RectSplitIndX, Orig2RectSplitIndY,LutVecOrig2RectXNum, LutVecOrig2RectYNum,Orig2RectNameMatX,Orig2RectNameMatY] = RemapPixel_tmp2(zeros([imgSize(1) imgSize(2)/div_level]),rotMatLeft,[intrMat(1,1);intrMat(2,2)],intrMat(1:2,3),zeros(5,1),0,intrMatLeftNew,imgLeft(:,1:imgSize(2)/div_level,:),[128 128],1,paraDir,'Left',newXSample0,newYSample0);%[160 120]
        
        [~, ~, xRect2OrigL, yRect2OrigL,KK_newL, KinitL,kL,imgL1, LutVecRect2OrigX, LutVecRect2OrigY, Rect2OrigSplitIndX, Rect2OrigSplitIndY, LutVecRect2OrigXNum, LutVecRect2OrigYNum,Rect2OrigNameMatX,Rect2OrigNameMatY,imgOut] = RemapPixel_tmp2(zeros(imgSize),rotMatLeft,[intrMat(1,1);intrMat(2,2)],intrMat(1:2,3),zeros(5,1),0,intrMatLeftNew,imgLeft,[128 128],0,paraDir,'Left',newXSample,newYSample);%[160 120]
%         [fianlVec, errLen] = MakeFinalVecUse('L',inputDir0,crop1,crop2, LutVecOrig2RectXNum,LutVecRect2OrigXNum,LutVecRect2OrigX,LutVecOrig2RectX,Orig2RectNameMatX,Rect2OrigNameMatX,LutVecOrig2RectY,LutVecRect2OrigY,LutVecOrig2RectYNum);
%         LutDec2HexUse(inputDir0, 'L');
% % % % % % %         xRect2OrigLT = xRect2OrigL'; yRect2OrigLT = yRect2OrigL';
% % % % % % %         mapVec = [size(xRect2OrigL,2);size(xRect2OrigL,1);xRect2OrigLT(:); yRect2OrigLT(:)];
% % % % % % %         fida = fopen(fullfile(inputDir0, 'inverse_mapping_coord.txt'),'w');
% % % % % % % %         fprintf(fida,'%0.5f\n',round(mapVec));
% % % % % % %         fprintf(fida,'%d\n',round(mapVec));
% % % % % % %         fclose(fida);
        
        if ~cfg.calib_stereo
            [fianlVec, errLen] = MakeFinalVecUse('L',inputDir0,crop1,crop2, LutVecOrig2RectXNum,LutVecRect2OrigXNum,LutVecRect2OrigX,LutVecOrig2RectX,Orig2RectNameMatX,Rect2OrigNameMatX,LutVecOrig2RectY,LutVecRect2OrigY,LutVecOrig2RectYNum);
             LutDec2HexUse(inputDir0, 'L');
            
            fid1 = fopen(fullfile(inputDir0,'cam_param1.txt'),'w');%????
            fprintf(fid1,'%d %d %d',imgSize(2), imgSize(1), cfg.switch_lr);
            fprintf(fid1, '\n');
            fprintf(fid1,'fisheye');
            fprintf(fid1, '\n');
            if ~cfg.isRotate
                fprintf(fid1,'%06f %06f %06f %06f',KK_newL(1,1),KK_newL(2,2),KK_newL(1,3),KK_newL(2,3));
            else
                fprintf(fid1,'%06f %06f %06f %06f',KK_newL(2,2),KK_newL(1,1),1+imgSize(1)-KK_newL(2,3),KK_newL(1,3));
            end
            if isfield(cfg, 'L2RGB')
                fprintf(fid1, '\n');
                rotVec = rodrigues(cfg.L2RGB(1:3,1:3));
                tVec = cfg.L2RGB(1:3,4);
                fprintf(fid1,'%06f %06f %06f %06f %06f %06f',rotVec(1),rotVec(2),rotVec(3),tVec(1),tVec(2),tVec(3));
            end
            
            
            %             fprintf(fid1, '\n');
            %             fprintf(fid1,'%06f %06f %06f %06f',KK_newR(1,1),KK_newR(2,2),KK_newR(1,3),KK_newR(2,3));
            %             fprintf(fid1, '\n');
            %             fprintf(fid1,'%06f %06f %06f %06f %06f %06f',0,0,0,-norm(stereoParam.transVecRef),0,0);
            fprintf(fid1, '\n');
            fclose(fid1);
            figure,imshow([rgb2gray(imgLeft) uint8(imgOut)]);
            saveas(gcf,fullfile(inputDir0,sprintf('fisheye_mono__%05d.png',2)));
            
            xRect2OrigLT = xRect2OrigL'; yRect2OrigLT = yRect2OrigL';
            mapVec = [size(xRect2OrigL,2);size(xRect2OrigL,1);xRect2OrigLT(:); yRect2OrigLT(:)];
            mapVec(3:end) = round(mapVec(3:end))-1;
            mapVec(mapVec < 0) = 0;
            fida = fopen(fullfile(inputDir0, 'inverse_mapping_coord_L.txt'),'w');
            %         fprintf(fida,'%0.5f\n',round(mapVec));
            fprintf(fida,'%d\n',round(mapVec));
            fclose(fida);
            
            return;
        else
            
            if ~cfg.isRotate
                [fianlVec, errLen] = MakeFinalVecUse('L',inputDir0,crop1,crop2, LutVecOrig2RectXNum,LutVecRect2OrigXNum,LutVecRect2OrigX,LutVecOrig2RectX,Orig2RectNameMatX,Rect2OrigNameMatX,LutVecOrig2RectY,LutVecRect2OrigY,LutVecOrig2RectYNum);
                LutDec2HexUse(inputDir0, 'L');
                
                xRect2OrigLT = xRect2OrigL'; yRect2OrigLT = yRect2OrigL';
                mapVec = [size(xRect2OrigL,2);size(xRect2OrigL,1);xRect2OrigLT(:); yRect2OrigLT(:)];
                mapVec(3:end) = round(mapVec(3:end))-1;
                mapVec(mapVec < 0) = 0;
                fida = fopen(fullfile(inputDir0, 'inverse_mapping_coord_L.txt'),'w');
                %         fprintf(fida,'%0.5f\n',round(mapVec));
                fprintf(fida,'%d\n',round(mapVec));
                fclose(fida);
                
            else
                [fianlVec, errLen] = MakeFinalVecUse('R',inputDir0,crop1,crop2, LutVecOrig2RectXNum,LutVecRect2OrigXNum,LutVecRect2OrigX,LutVecOrig2RectX,Orig2RectNameMatX,Rect2OrigNameMatX,LutVecOrig2RectY,LutVecRect2OrigY,LutVecOrig2RectYNum);
                LutDec2HexUse(inputDir0, 'R');
                
                xRect2OrigLT = xRect2OrigL'; yRect2OrigLT = yRect2OrigL';
                mapVec = [size(xRect2OrigL,2);size(xRect2OrigL,1);xRect2OrigLT(:); yRect2OrigLT(:)];
                mapVec(3:end) = round(mapVec(3:end))-1;
                mapVec(mapVec < 0) = 0;
                
                fida = fopen(fullfile(inputDir0, 'inverse_mapping_coord_R.txt'),'w');
                %         fprintf(fida,'%0.5f\n',round(mapVec));
                fprintf(fida,'%d\n',round(mapVec));
                fclose(fida);
                
            end
            
        end
        if cfg.is_kepler
            cfg.KK_newL = KK_newL;
        end
        %% right
        intrMatR = [stereoParam.focRight(1) 0 stereoParam.cenRight(1); 0 stereoParam.focRight(2) stereoParam.cenRight(2); 0 0 1];
        [~, ~, xOrig2RectR, yOrig2RectR,~,~,~,~, LutVecOrig2RectXR, LutVecOrig2RectYR,Orig2RectSplitIndXR, Orig2RectSplitIndYR, LutVecOrig2RectXNumR, LutVecOrig2RectYNumR, Orig2RectNameMatXR, Orig2RectNameMatYR] = RemapPixel_tmp2(zeros([imgSize(1) imgSize(2)/div_level]),rotMatRight,[intrMatR(1,1);intrMatR(2,2)],intrMatR(1:2,3),zeros(5,1),0,intrMatRightNew,imgRight(:,1:imgSize(2)/div_level,:),[128 128],1,paraDir,'Right',newXSample0,newYSample0);%[160 120]
        
        [~, ~, xRect2OrigR, yRect2OrigR, KK_newR, KinitR, kR, imgR1, LutVecRect2OrigXR, LutVecRect2OrigYR, Rect2OrigSplitIndXR, Rect2OrigSplitIndYR, LutVecRect2OrigXNumR, LutVecRect2OrigYNumR, Rect2OrigNameMatXR, Rect2OrigNameMatYR, imgOutR] = RemapPixel_tmp2(zeros(imgSize),rotMatRight,[intrMatR(1,1);intrMatR(2,2)],intrMatR(1:2,3),zeros(5,1),0,intrMatRightNew,imgRight,[128 128],0,paraDir,'Right',newXSample,newYSample);%[160 120]
%         [fianlVecR, errLenR] = MakeFinalVecUse('R',inputDir0,crop1,crop2, LutVecOrig2RectXNumR,LutVecRect2OrigXNumR,LutVecRect2OrigXR,LutVecOrig2RectXR,Orig2RectNameMatXR,Rect2OrigNameMatXR,LutVecOrig2RectYR,LutVecRect2OrigYR,LutVecOrig2RectYNumR);
        
        if ~cfg.isRotate
            [fianlVecR, errLenR] = MakeFinalVecUse('R',inputDir0,crop1,crop2, LutVecOrig2RectXNumR,LutVecRect2OrigXNumR,LutVecRect2OrigXR,LutVecOrig2RectXR,Orig2RectNameMatXR,Rect2OrigNameMatXR,LutVecOrig2RectYR,LutVecRect2OrigYR,LutVecOrig2RectYNumR);
            LutDec2HexUse(inputDir0, 'R');
            
            xRect2OrigRT = xRect2OrigR'; yRect2OrigRT = yRect2OrigR';
            mapVecR = [size(xRect2OrigR,2);size(xRect2OrigR,1);xRect2OrigRT(:); yRect2OrigRT(:)];
             mapVecR(3:end) = round(mapVecR(3:end))-1;
            mapVecR(mapVecR < 0) = 0;
            fida = fopen(fullfile(inputDir0, 'inverse_mapping_coord_R.txt'),'w');
            %         fprintf(fida,'%0.5f\n',round(mapVec));
            fprintf(fida,'%d\n',round(mapVecR));
            fclose(fida);
            
            
        else
            [fianlVecR, errLenR] = MakeFinalVecUse('L',inputDir0,crop1,crop2, LutVecOrig2RectXNumR,LutVecRect2OrigXNumR,LutVecRect2OrigXR,LutVecOrig2RectXR,Orig2RectNameMatXR,Rect2OrigNameMatXR,LutVecOrig2RectYR,LutVecRect2OrigYR,LutVecOrig2RectYNumR);
            LutDec2HexUse(inputDir0, 'L');
            
            xRect2OrigRT = xRect2OrigR'; yRect2OrigRT = yRect2OrigR';
            mapVecR = [size(xRect2OrigR,2);size(xRect2OrigR,1);xRect2OrigRT(:); yRect2OrigRT(:)];
             mapVecR(3:end) = round(mapVecR(3:end))-1;
            mapVecR(mapVecR < 0) = 0;
            fida = fopen(fullfile(inputDir0, 'inverse_mapping_coord_L.txt'),'w');
            %         fprintf(fida,'%0.5f\n',round(mapVec));
            fprintf(fida,'%d\n',round(mapVecR));
            fclose(fida);
        end
        %     [aa,cc] = RecoverLut('C:\Users\ROGER\Desktop\new 1.txt',newXSample0,newYSample0);
        
        rectImgL = uint8(interp2(xGrid,yGrid,double(imgLeft(:,:,1)),xRect2OrigL,yRect2OrigL));
        rectImgR = uint8(interp2(xGrid,yGrid,double(imgRight(:,:,1)),xRect2OrigR,yRect2OrigR));
        save(fullfile(inputDir0,'reMap.mat'),'xGrid','yGrid', 'xRect2OrigL', 'yRect2OrigL', 'xRect2OrigR', 'yRect2OrigR');
        
        figure,imshowpair(imgOut, imgOutR);
        
        
%          fid1 = fopen(fullfile(inputDir0,'cam_param_fisheye.txt'),'w');%????
         fid1 = fopen(fullfile(inputDir0,'cam_param1.txt'),'w');%????
        fprintf(fid1,'%d %d %d',imgSize(2), imgSize(1), cfg.switch_lr);
        fprintf(fid1, '\n');
        fprintf(fid1,'fisheye');
        fprintf(fid1, '\n');
        if ~cfg.isRotate
            fprintf(fid1,'%06f %06f %06f %06f',KK_newL(1,1),KK_newL(2,2),KK_newL(1,3),KK_newL(2,3));
            fprintf(fid1, '\n');
            fprintf(fid1,'%06f %06f %06f %06f',KK_newR(1,1),KK_newR(2,2),KK_newR(1,3),KK_newR(2,3));
            fprintf(fid1, '\n');
        else
            fprintf(fid1,'%06f %06f %06f %06f',KK_newL(2,2),KK_newL(1,1),1+imgSize(1)-KK_newL(2,3),KK_newL(1,3));
            fprintf(fid1, '\n');
            fprintf(fid1,'%06f %06f %06f %06f',KK_newR(2,2),KK_newR(1,1),1+imgSize(1)-KK_newR(2,3),KK_newR(1,3));
            fprintf(fid1, '\n');
        end
        fprintf(fid1,'%06f %06f %06f %06f %06f %06f',0,0,0,-norm(stereoParam.transVecRef),0,0);
        fprintf(fid1, '\n');
        fclose(fid1);
        
        
        
        saveas(gcf,fullfile(inputDir0,sprintf('fisheye_%05d.png',2)));
        return;
    end
    
    
    
    if 0
        [~, ~, xOrig2RectL, yOrig2RectL,~,~,~,~, LutVecOrig2RectX, LutVecOrig2RectY,Orig2RectSplitIndX, Orig2RectSplitIndY,LutVecOrig2RectXNum, LutVecOrig2RectYNum,Orig2RectNameMatX,Orig2RectNameMatY] = RemapPixel_tmp2(zeros([480 640/2]),eye(3),[intrMat(1,1);intrMat(2,2)],intrMat(1:2,3),zeros(5,1),0,intrMat,imgLeft(:,1:640/2,:),[16 16],1,paraDir,'Left',newXSAmple,newYSample);%[160 120]
        
        [~, ~, xRect2OrigL, yRect2OrigL,KK_newL, KinitL,kL,imgL1, LutVecRect2OrigX, LutVecRect2OrigY, Rect2OrigSplitIndX, Rect2OrigSplitIndY, LutVecRect2OrigXNum, LutVecRect2OrigYNum,Rect2OrigNameMatX,Rect2OrigNameMatY] = RemapPixel_tmp2(zeros([480 640]),eye(3),[intrMat(1,1);intrMat(2,2)],intrMat(1:2,3),zeros(5,1),0,intrMat,imgLeft(:,1:640,:),[16 16],0,paraDir,'Left',newXSample,newYSample);%[160 120]
    end
    
    imgL2 = RectImg(xOrig2RectL, yOrig2RectL, xRect2OrigL, yRect2OrigL, imgLeft,KK_newL, KinitL,kL,rotMatLeft,imgLeft,paraDir,'Left');
    imgL222 = RectImg(xOrig2RectL, yOrig2RectL, xRect2OrigL, yRect2OrigL, imresize(imgLeft,1/2),KK_newL, KinitL,kL,rotMatLeft,imresize(imgLeft,1/2),paraDir,'Left',2);
    V = imresize(imgLeft,1/2)-10;
    imgL222_v = RectImg(xOrig2RectL, yOrig2RectL, xRect2OrigL, yRect2OrigL, V,KK_newL, KinitL,kL,rotMatLeft,V,paraDir,'Left',2);
    
    imageLeft1 = imresize(imgLeft,0.5);
    % imgL22 = uint8(zeros(size(imgL222)));
    imgL22 = [imgL222 imgL222];
    imgL22(:,1:2:end,:) = imgL222;  imgL22(:,2:2:end,:) = imgL222_v;
    imgL11 = uint8(zeros(size(imgL22)));
    imgL11(:,1:2:end,:) = imageLeft1;  imgL11(:,2:2:end,:) = V;
    
    para = 'D:\Temp\20190223_\fisheyeQ';
    
    para = 'D:\Temp\20190107\20190107_sensor_sining1_calib_rom_4k\L';
    crop1 = [146 209];
    crop1 = [350 209];
    
    %     crop1 = [400 209];
    %     crop2 = [3713 1976];
    
    crop1 = [400 208];
    crop2 = [3713 1975];
    
    fid111 = fopen(fullfile(para,'LutCheck.txt'),'w');
    
    
    
    LutVecOrig2RectX(37:46) = '0000000001';%'0011010000';
    LutVecOrig2RectX(26:36) = '00000000001';%'00110010000'; % '00110010000';    %'00101011110';
    
    
    LutVecOrig2RectX(60:71) = '100001110000';%'011110110111';
    LutVecRect2OrigX(47:59) = '0111100000000';%'0111010000001';  %'0111010000001';
    
    
    
    
    
    
    
    
    
    LutVecOrig2RectX(37:46) = '1000011100';%'0011010000';
    LutVecOrig2RectX(26:36) = '00010010000';%'00110010000'; % '00110010000';    %'00101011110';
    
    
    LutVecOrig2RectX(60:71) = '100001101011';%'011110110111';
    LutVecRect2OrigX(47:59) = '0111010001111';%'0111010000001';  %'0111010000001';
    
    
    
    supposedLen = 25+21+25+11+12+LutVecOrig2RectXNum(7)*15+LutVecOrig2RectXNum(8)*14+LutVecRect2OrigXNum(7)*15+LutVecRect2OrigXNum(8)*14+LutVecOrig2RectXNum(7)*LutVecOrig2RectXNum(8)*28+LutVecRect2OrigXNum(7)*LutVecRect2OrigXNum(8)*26;
    finalVec = [LutVecRect2OrigX(1:25)...
        LutVecOrig2RectX(37:46) LutVecOrig2RectX(26:36)...   %crop1 Y(10) X(11)%                 LutVecOrig2RectX(60:71) LutVecOrig2RectX(47:59)...   %crop2 Y(12) X(13)
        LutVecOrig2RectX(60:71) LutVecRect2OrigX(47:59)...   %crop2 Y(12) X(13)
        LutVecOrig2RectX(72:82) LutVecRect2OrigX(72:83)...
        LutVecOrig2RectX(sum(cell2mat(Orig2RectNameMatX(1:8,2)))+1:sum(cell2mat(Orig2RectNameMatX(1:9,2)))) LutVecOrig2RectY(sum(cell2mat(Orig2RectNameMatX(1:9,2)))+1:sum(cell2mat(Orig2RectNameMatX(1:10,2))))...
        LutVecRect2OrigX(sum(cell2mat(Rect2OrigNameMatX(1:8,2)))+1:sum(cell2mat(Rect2OrigNameMatX(1:9,2)))) LutVecRect2OrigY(sum(cell2mat(Rect2OrigNameMatX(1:9,2)))+1:sum(cell2mat(Rect2OrigNameMatX(1:10,2))))];
    
    fprintf(fid111,'%s\n',(LutVecRect2OrigX(1:13)));
    fprintf(fid111,'%s\n',(LutVecOrig2RectX(14:25)));
    fprintf(fid111,'%s\n',([LutVecOrig2RectX(37:46) LutVecOrig2RectX(26:36)]));
    fprintf(fid111,'%s\n',([LutVecOrig2RectX(60:71) LutVecRect2OrigX(47:59)]));
    fprintf(fid111,'%s\n',(LutVecOrig2RectX(72:76)));
    fprintf(fid111,'%s\n',(LutVecOrig2RectX(77:82)));
    fprintf(fid111,'%s\n',(LutVecRect2OrigX(72:77)));
    fprintf(fid111,'%s\n',(LutVecRect2OrigX(78:83)));
    vec1 = [LutVecOrig2RectX(sum(cell2mat(Orig2RectNameMatX(1:8,2)))+1:sum(cell2mat(Orig2RectNameMatX(1:9,2))))];
    vec1 = reshape(vec1,15,[]); vec1 = vec1';
    for oo = 1 : size(vec1,1)
        fprintf(fid111,'%s\n',((vec1(oo,:))));
    end
    vec2 = [LutVecOrig2RectY(sum(cell2mat(Orig2RectNameMatX(1:9,2)))+1:sum(cell2mat(Orig2RectNameMatX(1:10,2))))];
    vec2 = reshape(vec2,14,[]); vec2 = vec2';
    for oo = 1 : size(vec2,1)
        fprintf(fid111,'%s\n',(vec2(oo,:)));
    end
    vec3 = [LutVecRect2OrigX(sum(cell2mat(Rect2OrigNameMatX(1:8,2)))+1:sum(cell2mat(Rect2OrigNameMatX(1:9,2))))];
    vec3 = reshape(vec3,15,[]); vec3 = vec3';
    for oo = 1 : size(vec3,1)
        fprintf(fid111,'%s\n',(vec3(oo,:)));
    end
    vec4 = [LutVecRect2OrigY(sum(cell2mat(Rect2OrigNameMatX(1:9,2)))+1:sum(cell2mat(Rect2OrigNameMatX(1:10,2))))];
    vec4 = reshape(vec4,14,[]); vec4 = vec4';
    for oo = 1 : size(vec4,1)
        fprintf(fid111,'%s\n',(vec4(oo,:)));
    end
    % forward
    for jj = 1 : LutVecOrig2RectYNum(7) * LutVecOrig2RectYNum(8)
        unitLength1 = Orig2RectNameMatX{11,2}/size(Orig2RectNameMatX{11,1},1);
        startId1 = sum(cell2mat(Orig2RectNameMatX(1:10,2)));
        tmp =  [LutVecOrig2RectY(startId1+(jj-1)*unitLength1+1:startId1+jj*unitLength1) LutVecOrig2RectX(startId1+(jj-1)*unitLength1+1:startId1+jj*unitLength1)];
        fprintf(fid111,'%s\n',(tmp));
        finalVec = [finalVec tmp];
    end
    % reverse
    for kk = 1 : LutVecRect2OrigXNum(7) * LutVecRect2OrigXNum(8)
        unitLength2 = Rect2OrigNameMatX{11,2}/size(Rect2OrigNameMatX{11,1},1);
        startId2 = sum(cell2mat(Rect2OrigNameMatX(1:10,2)));
        tmp =  [LutVecRect2OrigY(startId2+(kk-1)*unitLength2+1:startId2+kk*unitLength2) LutVecRect2OrigX(startId2+(kk-1)*unitLength2+1:startId2+kk*unitLength2)];
        fprintf(fid111,'%s\n',(tmp));
        finalVec = [finalVec tmp];
        
    end
    fclose(fid111);
    
    errLen = supposedLen - length(finalVec);
    
    
    para = 'D:\Temp\20190315\rectFile';
    
    
    Yout = (imgL2(:,:,1));
    Yin = (imgLeft(:,:,1));
    Yin = Yin';
    Yout = Yout';
    YinVec = Yin(:);
    YoutVec = Yout(:);
    calibRomFid = fopen(fullfile(paraDir,'inputY.bin'),'w');
    fwrite(calibRomFid, YinVec,'uint8'); fclose(calibRomFid);
    calibRomFid = fopen(fullfile(paraDir,'outputY.bin'),'w');
    fwrite(calibRomFid, YoutVec,'uint8'); fclose(calibRomFid);
    
    
    Yout = (imgL22(:,:,1));
    Yin = (imgL11(:,:,1));
    Yin = Yin';
    Yout = Yout';
    YinVec = Yin(:);
    YoutVec = Yout(:);
    calibRomFid = fopen(fullfile(paraDir,'inputY_uv.bin'),'w');
    fwrite(calibRomFid, YinVec,'uint8'); fclose(calibRomFid);
    calibRomFid = fopen(fullfile(paraDir,'outputY_uv.bin'),'w');
    fwrite(calibRomFid, YoutVec,'uint8'); fclose(calibRomFid);
    
    
    
    
    
    
    
    Yout = (imgR2(:,:,1));
    Yin = (imgRight(:,:,1));
    Yin = Yin';
    Yout = Yout';
    YinVec = Yin(:);
    YoutVec = Yout(:);
    calibRomFid = fopen(fullfile(paraDir,'inputYR.bin'),'w');
    fwrite(calibRomFid, YinVec,'uint8'); fclose(calibRomFid);
    calibRomFid = fopen(fullfile(paraDir,'outputYR.bin'),'w');
    fwrite(calibRomFid, YoutVec,'uint8'); fclose(calibRomFid);
    Yout = (imgR22(:,:,1));
    Yin = (imgR11(:,:,1));
    Yin = Yin';
    Yout = Yout';
    YinVec = Yin(:);
    YoutVec = Yout(:);
    calibRomFid = fopen(fullfile(paraDir,'inputYR_uv.bin'),'w');
    fwrite(calibRomFid, YinVec,'uint8'); fclose(calibRomFid);
    calibRomFid = fopen(fullfile(paraDir,'outputYR_uv.bin'),'w');
    fwrite(calibRomFid, YoutVec,'uint8'); fclose(calibRomFid);
    
    
    
    
    
    if 0
        LutVec = [LutVecOrig2RectX;LutVecOrig2RectY;LutVecRect2OrigX;LutVecRect2OrigY];
        
        fid1 = fopen(fullfile(para,'Lut.txt'),'w');%????
        for i = 1 : size(LutVec,2)
            fprintf(fid1,'%d %d %d %d \n',str2double(LutVec(1,i)),str2double(LutVec(2,i)),str2double(LutVec(3,i)),str2double(LutVec(4,i)));
        end
        fclose(fid1);
    else
        
        LutVec = [LutVecOrig2RectX LutVecOrig2RectY(Rect2OrigSplitIndX+1:end) LutVecRect2OrigX(Rect2OrigSplitIndX+1:end) LutVecRect2OrigY(Rect2OrigSplitIndX+1:end)];
        LutVecNum = [LutVecOrig2RectXNum' LutVecOrig2RectYNum(11:end)' LutVecRect2OrigXNum(11:end)' LutVecRect2OrigYNum(11:end)'];
        
        
        if 1
            fid1 = fopen(fullfile(para,'Lut.txt'),'w');%????
            for i = 1 : size(finalVec,2)
                fprintf(fid1,'%d\n',str2double(finalVec(i)));
            end
            fclose(fid1);
        else
            fid1 = fopen(fullfile(para,'Lut.txt'),'w');%????
            for i = 1 : size(LutVec,2)
                fprintf(fid1,'%d\n',str2double(LutVec(i)));
            end
            fclose(fid1);
        end
        
        
        fid1 = fopen(fullfile(para,'LutDec.txt'),'w');%????
        for i = 1 : size(LutVecNum,2)
            fprintf(fid1,'%d\n',(LutVecNum(i)));
        end
        fclose(fid1);
        
    end
    
    
    
    
    if 0
        
        %         aaa =
    else
        aaa = load(fullfile(para, 'Lut.txt'));
        aaa = aaa(:);
    end
    
    %       fid2 = fopen(fullfile(para,'Luttt.txt'),'w');%????
    %     for i = 1 : size(aaa,1)
    %         fprintf(fid2,'%d\n',aaa(i));
    %     end
    %     fclose(fid2);
    %     bbb = load(fullfile(para,'Luttt.txt'));
    
    
    intLength = floor(length(aaa)/8);
    leftover = aaa(8*intLength+1:end);
    aaa_ = aaa;
    for t = 1 : (8-length(leftover))
        aaa_ = [aaa_;0];
    end
    aaa_Mat = reshape(aaa_,8,[]);
    ccc = bin2dec(num2str(aaa_Mat'));
    calibRomFid = fopen(fullfile(para,'lut8.bin'),'wb');
    %     fwrite(calibRomFid, aaa,'ubit1');
    fwrite(calibRomFid, [ccc],'uint8');
    fclose(calibRomFid);
    
    %     leftover = leftover*
    aaaTemp = reshape(aaa(1:8*intLength),8,[]);
    aaaTemp = aaaTemp([end:-1:1],:);
    aaaTemp = aaaTemp(:);
    
    tmp = leftover;
    for u = 1 : (8 - length(leftover))
        
        tmp = [tmp;0];
        
    end
    tmp = tmp(end:-1:1);
    
    % % if length(leftover) >= 4
    % %     tmp = [];
    % %     for u = 1 : 8
    % %         if u < 8-length(leftover)
    % %             tmp = [tmp;0];
    % %         else
    % %             tmp = [tmp;leftover(8-u)];
    % %         end
    % %     end
    % % else
    % %
    % % end
    calibRomFid = fopen(fullfile(para,'lut.bin'),'wb');
    %     fwrite(calibRomFid, aaa,'ubit1');
    fwrite(calibRomFid, [aaaTemp;tmp],'ubit1');
    fclose(calibRomFid);
    
    
    %     calibRomFid2 = fopen(fullfile(para,'lut3.bin'),'wb');
    %     fwrite(calibRomFid2, bbb,'ubit1');
    %     fclose(calibRomFid2);
    
    
    fid3=fopen(fullfile(para,'lut.bin'),'r');
    A = fread(fid3,'ubit1');fclose(fid3);
    figure,plot(A(1:length(aaa))-aaa)
    
    
    % %       fid4=fopen(fullfile(para,'lut3.bin'),'r');
    % %     B = fread(fid4,'ubit1');fclose(fid4);
    % %     figure,plot(B-aaa)
    
    
    
    close all
    [xRect2OrigR, yRect2OrigR,KK_newR, KinitR,kR,imgR1] = RemapPixel(zeros(imgSize(1,1:2)),rotMatRight,focRight,cenRight,kcRight,alphaRight,intrMatRightNew,imgRight,[128 128],0,paraDir,'Right');
    close all
    [xOrig2RectR, yOrig2RectR] = RemapPixel(zeros(imgSize(1,1:2)),rotMatRight,focRight,cenRight,kcRight,alphaRight,intrMatRightNew,imgRight,[128 128],1,paraDir,'Right'); %[140 100]
    imgR2 = RectImg(xOrig2RectR, yOrig2RectR, xRect2OrigR, yRect2OrigR, imgRight,KK_newR, KinitR,kR,rotMatRight,imgRight,paraDir,'Right');
    %     [~,indNewL,ind1L,ind2L,ind3L,ind4L,a1L,a2L,a3L,a4L,imgL1] = rect_index_table6(zeros(imgSize(1,1:2)),rotMatLeft,focLeft,cenLeft,kcLeft,alphaLeft,intrMatLeftNew,imgLeft);
    %     [~,indNewR,ind1R,ind2R,ind3R,ind4R,a1R,a2R,a3R,a4R,imgR1] = rect_index_table6(zeros(imgSize(1,1:2)),rotMatRight,focRight,cenRight,kcRight,alphaRight,intrMatRightNew,imgRight);
end
rectParamL = struct('rectIntrMat', intrMatLeftNew,  'dstInd', indNewL, 'srcInd', cat(3, ind1L,ind2L,ind3L,ind4L), 'coef', cat(3, a1L,a2L,a3L,a4L));
rectParamR = struct('rectIntrMat', intrMatRightNew, 'dstInd', indNewR, 'srcInd', cat(3, ind1R,ind2R,ind3R,ind4R), 'coef', cat(3, a1R,a2R,a3R,a4R));

end


