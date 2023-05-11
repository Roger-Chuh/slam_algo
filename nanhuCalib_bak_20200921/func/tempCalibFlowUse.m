function [imgL, imgR, rectImgL, rectImgR, meanError] = tempCalibFlowUse(inputDir,scalee, yuvv,widthh,heightt,calibFuncDir,cbSizee,switchLRR)
global cfg inputDir0


usePar = 1; 0; 1; 0;1;
useEXE = 0;1;


width = str2double(widthh);
height = str2double(heightt);
scale = str2double(scalee);
cbSize = str2double(cbSizee);
yuv = str2double(yuvv);
switchLR = str2double(switchLRR);
close all

workDir = pwd;
paraDir = inputDir;
if yuv
    %     readYUV( inputDir ,1920,1080,scale);
    %     readYUV( inputDir ,1280,720,scale);
    readYUV( inputDir ,width,height,scale);
    %     pause;
end
% calibFuncDir = 'D:\bk_20180627_\nextvpu\algorithm\src\matlibs\computer_vision\camera_calibration\Scaramuzza_OCamCalib_v3.0_win';
if cfg.check_depth
    if ~cfg.isRotate
        modeE = 2;
    else
        modeE = 1;
    end
else
    modeE = 2;
end

try
    imgLst1 = getImgLst(inputDir,'png');
catch
    try
        imgLst1 = getImgLst(inputDir,'bmp');
    catch
        imgLst1 = getImgLst(inputDir,'jpg');
    end
end


config.dX = cbSize; 23.7; 24.1; 70; 18.4; config.dY = cbSize; 23.7; 24.1; 70; 18.4;
config.estDistortion = [1;1;1;1;cfg.est_k3];

if cfg.calib_stereo
imgL = imread(imgLst1{1});
imgR = imread(imgLst1{length(imgLst1)/2+1});
[nr, nc, nn] = size(imgL);

% % id = randperm(length(imgLst1)/2);
% % iD = id(1:30);
% % imgLst1 = imgLst1([iD length(imgLst1)/2+iD]);
% % % config.dX = 70; 18.4; config.dY = 70; 18.4;

% config.dX = 18.4; config.dY = 18.4;

    if ~switchLR
        leftImgList = imgLst1(1:length(imgLst1)/2);
        rightImgList = imgLst1(length(imgLst1)/2+1:end);
        
        
        
        
        
        if ~usePar
            if useEXE
                fid1 = fopen(fullfile(inputDir,'L.txt'),'w');
                fid2 = fopen(fullfile(inputDir,'R.txt'),'w');
                for nj = 1 : length(imgLst1)/2
                    fprintf(fid1,'%s\n',imgLst1{nj});
                    fprintf(fid2,'%s\n',imgLst1{length(imgLst1)/2+nj});
                end
                fclose(fid1);
                fclose(fid2);
                
                cmdStrL = [calibFuncDir(1:end-5),'\func\CalibSingleExe.exe',' ',calibFuncDir,' ',cbSizee,' L'];
                cmdStrR = [calibFuncDir(1:end-5),'\func1\CalibSingleExe.exe',' ',calibFuncDir,' ',cbSizee,' R'];
                system(cmdStrL);
                system(cmdStrR);
                skghb = 1;
                
            else
                
                [camParamL, cbcXYL, cbGridL, config, ~, cbcXYTmpL, cbGridTmpL, goodIdL] = CbCalibSingleUse4(imgLst1(1:length(imgLst1)/2), [], calibFuncDir, modeE, config);
                close all
                [camParamR, cbcXYR, cbGridR, config, ~, cbcXYTmpR, cbGridTmpR, goodIdR] = CbCalibSingleUse4(imgLst1(length(imgLst1)/2+1:end), [], calibFuncDir, modeE, config);
                close all;
            end
        else
            imList{1,1} = imgLst1(1:length(imgLst1)/2);
            imList{2,1} = imgLst1(length(imgLst1)/2+1:end);
            camParam_ = cell(2,1);
            cbcXY_ = cell(2,1);
            cbGrid_ = cell(2,1);
            config_ = cell(2,1);
            cbcXYTmp_ = cell(2,1);
            cbGridTmp_ = cell(2,1);
            goodId_ = cell(2,1);
            
            calibFuncDir_ = cell(2,1);
            modeE_ = cell(2,1);
            config__ = cell(2,1);
            calibFuncDir_{1} = calibFuncDir;calibFuncDir_{2} = strcat(calibFuncDir,'1');
            modeE_{1} = modeE;modeE_{2} = modeE;
            config__{1} = config;config__{2} = config;
            emp = cell(2,1);
            
            cfg_ = cell(2,1);
            cfg_{1} = cfg;
            cfg_{2} = cfg;
            parfor v = 1:2
             [camParam_{v,1}, cbcXY_{v,1}, cbGrid_{v,1}, config_{v,1}, ~, cbcXYTmp_{v,1}, cbGridTmp_{v,1}, goodId_{v,1}] = CbCalibSingleUse4(imList{v,1}, emp{v}, calibFuncDir_{v}, modeE_{v}, config__{v}, cfg_{v});
            end
            
            
            camParamL = camParam_{1};
            cbcXYL = cbcXY_{1};
            cbGridL = cbGrid_{1};
            configL = config_{1};
            cbcXYTmpL = cbcXYTmp_{1};
            cbGridTmpL = cbGridTmp_{1};
            goodIdL = goodId_{1};
            
            camParamR = camParam_{2};
            cbcXYR = cbcXY_{2};
            cbGridR = cbGrid_{2};
            config = config_{2};
            cbcXYTmpR = cbcXYTmp_{2};
            cbGridTmpR = cbGridTmp_{2};
            goodIdR = goodId_{2};
            
        end
        leftList = imgLst1(1:length(imgLst1)/2);
        rightList = imgLst1(length(imgLst1)/2+1:end);
    else
        
        rightImgList = imgLst1(1:length(imgLst1)/2);
        leftImgList = imgLst1(length(imgLst1)/2+1:end);
        if ~usePar
            [camParamL, cbcXYL, cbGridL, config, ~, cbcXYTmpL, cbGridTmpL, goodIdL] = CbCalibSingleUse4(imgLst1(length(imgLst1)/2+1:end), [], calibFuncDir, modeE, config);
            close all
            [camParamR, cbcXYR, cbGridR, config, ~, cbcXYTmpR, cbGridTmpR, goodIdR] = CbCalibSingleUse4(imgLst1(1:length(imgLst1)/2), [], calibFuncDir, modeE, config);
            close all;
        else
             imList{1,1} = imgLst1(length(imgLst1)/2+1:end);
            imList{2,1} = imgLst1(1:length(imgLst1)/2);
            camParam_ = cell(2,1);
            cbcXY_ = cell(2,1);
            cbGrid_ = cell(2,1);
            config_ = cell(2,1);
            cbcXYTmp_ = cell(2,1);
            cbGridTmp_ = cell(2,1);
            goodId_ = cell(2,1);
            
             calibFuncDir_ = cell(2,1);
            modeE_ = cell(2,1);
            config__ = cell(2,1);
            calibFuncDir_{1} = calibFuncDir;calibFuncDir_{2} = strcat(calibFuncDir,'1');
            modeE_{1} = modeE;modeE_{2} = modeE;
            config__{1} = config;config__{2} = config;
            emp = cell(2,1);
            
            cfg_ = cell(2,1);
            cfg_{1} = cfg;
            cfg_{2} = cfg;
            
            parfor v = 1:2
             [camParam_{v,1}, cbcXY_{v,1}, cbGrid_{v,1}, config_{v,1}, ~, cbcXYTmp_{v,1}, cbGridTmp_{v,1}, goodId_{v,1}] = CbCalibSingleUse4(imList{v,1}, emp{v}, calibFuncDir_{v}, modeE_{v}, config__{v}, cfg_{v});
            end
            
            camParamL = camParam_{1};
            cbcXYL = cbcXY_{1};
            cbGridL = cbGrid_{1};
            configL = config_{1};
            cbcXYTmpL = cbcXYTmp_{1};
            cbGridTmpL = cbGridTmp_{1};
            goodIdL = goodId_{1};
            
            camParamR = camParam_{2};
            cbcXYR = cbcXY_{2};
            cbGridR = cbGrid_{2};
            config = config_{2};
            cbcXYTmpR = cbcXYTmp_{2};
            cbGridTmpR = cbGridTmp_{2};
            goodIdR = goodId_{2};
            
        end
        rightList = imgLst1(1:length(imgLst1)/2);
        leftList = imgLst1(length(imgLst1)/2+1:end);
    end
else
    
    
    if ~cfg.is_kepler
        
%         imgLst1 = imgLst1(cfg.leftCorner{1,1},:);
        
        imgL = imread(imgLst1{1});
        % imgR = imread(imgLst1{length(imgLst1)/2+1});
        [nr, nc, nn] = size(imgL);
        [camParamL, cbcXYL, cbGridL, config, ~, cbcXYTmpL, cbGridTmpL, goodIdL] = CbCalibSingleUse4(imgLst1(1:length(imgLst1)/1), [], calibFuncDir, modeE, config);
    else
        
        imgLst1 = imgLst1(cfg.Corner{1,1},:);
        
        imgL = imread(imgLst1{1});
        % imgR = imread(imgLst1{length(imgLst1)/2+1});
        [nr, nc, nn] = size(imgL);
        [camParamL, cbcXYL, cbGridL, config, ~, cbcXYTmpL, cbGridTmpL, goodIdL] = CbCalibSingleUse4(imgLst1(1:length(imgLst1)/1), [], calibFuncDir, modeE, config);
        
        pt3d_temp = [cbGridL{1,1}' cbGridL{1,1}(2,:)'];
        pt3d_temp(:,3) = 0;
        
        camParamLL.foc = [cfg.KK_newL(1,1); cfg.KK_newL(2,2)];
        camParamLL.cen = [cfg.KK_newL(1,3); cfg.KK_newL(2,3)];
        camParamLL.alpha = 0;
        camParamLL.kc = zeros(5,1);
        
        cbcXYLL = cell(length(imgLst1),1);
        for k = 1 : length(imgLst1)
            [poseVec_temp, outlierIdtemp] = posest(cfg.Corner{k,2}, pt3d_temp,  0.95, cfg.KK_newL, 'repr_err');
            camParamLL.rotVec(:,k) = poseVec_temp(1:3);
            camParamLL.tranVec(:,k) = poseVec_temp(4:6);
            cbcXYLL{k,1} = cfg.Corner{k,2}';
        end
        
        goodIdLL = [1:length(imgLst1)]';
        goodId = intersect(goodIdLL,goodIdL);
        camParamLL.rotVec = camParamLL.rotVec(:,ismember(goodIdLL,goodId));
        %     camParamLL.rotVecErr = camParamLL.rotVecErr(:,ismember(goodIdLL,goodId));
        camParamLL.tranVec = camParamLL.tranVec(:,ismember(goodIdLL,goodId));
        %     camParamLL.tranVecErr = camParamLL.tranVecErr(:,ismember(goodIdLL,goodId));
        camParamL.rotVec = camParamL.rotVec(:,ismember(goodIdL,goodId));
        camParamL.rotVecErr = camParamL.rotVecErr(:,ismember(goodIdL,goodId));
        camParamL.tranVec = camParamL.tranVec(:,ismember(goodIdL,goodId));
        camParamL.tranVecErr = camParamL.tranVecErr(:,ismember(goodIdL,goodId));
        
        if 1%cfg.isRotate
            stereoParam = CbCalibStereo2(camParamLL, camParamL, cbcXYLL(ismember(goodIdLL,goodId)), cbcXYL(ismember(goodIdL,goodId)), cbGridL(ismember(goodIdL,goodId)), config, config);
        else
            stereoParam = CbCalibStereo2(camParamL, camParamLL, cbcXYL(ismember(goodIdL,goodId)), cbcXYLL(ismember(goodIdLL,goodId)), cbGridL(ismember(goodIdL,goodId)), config, config);
        end
        
        load(fullfile(inputDir0,'remap.mat'));
        if cfg.isRotate
            t_ = rotz(90)*stereoParam.transVecRef;
            % t_ = [-r(:,2) r(:,1) r(:,3)]*t;
            r_ = rotz(90)*rodrigues(stereoParam.rotVecRef);
            
            r__ = [-r_(:,2) r_(:,1) r_(:,3)];
        else
            
            r__ = rodrigues(stereoParam.rotVecRef);
            t_ = stereoParam.transVecRef;
        end
        
        cfg.L2RGB = [r__ t_;0 0 0 1];
        
        KL = [stereoParam.focLeft(1) 0 stereoParam.cenLeft(1);0 stereoParam.focLeft(2) stereoParam.cenLeft(2);0 0 1];
        KR = [stereoParam.focRight(1) 0 stereoParam.cenRight(1);0 stereoParam.focRight(2) stereoParam.cenRight(2);0 0 1];
        
        if cfg.isRotate
            KL_= [KL(2,2) 0 1+height-KL(2,3); 0 KL(1,1) KL(1,3); 0 0 1];
            KR_= [KR(2,2) 0 1+height-KR(2,3); 0 KR(1,1) KR(1,3); 0 0 1];
        else
            KL_ =KL;
            KR_ =KR;
        end
        cbcXYLL = cbcXYLL(ismember(goodIdLL,goodId));
        cbcXYRR = cbcXYL(ismember(goodIdL,goodId));
        
        
        imgLst11 = imgLst1(goodIdL);
        imgLst11 = imgLst11(ismember(goodIdL,goodId));
        
        
        kr1_ = stereoParam.kcRight(1);
        kr2_ = stereoParam.kcRight(2);
        kr3_ = stereoParam.kcRight(5);
        pr1_ = stereoParam.kcRight(3);
        pr2_ = stereoParam.kcRight(4);
        
        
        pt3d = [cbGridL{1,1}' cbGridL{1,1}(2,:)'];
        pt3d(:,3) = 0;
        
        pt3d_ = [-pt3d(:,2) pt3d(:,1) pt3d(:,3)];
        
        for i = 1 : length(goodId) %length(cbcXYL)
            
            if 0
                [xr] = normalize_pixel(cbcXYR{goodId(i)},stereoParam.focRight,stereoParam.cenRight,stereoParam.kcRight,stereoParam.alphaRight);
            else
                [xr] = normalize_pixel(cbcXYRR{(i)},stereoParam.focRight,stereoParam.cenRight,stereoParam.kcRight,stereoParam.alphaRight);
            end
            
            [xl] = normalize_pixel(cbcXYLL{(i)},stereoParam.focLeft,stereoParam.cenLeft,stereoParam.kcLeft,stereoParam.alphaLeft);
            
            
            
            xrr = pflat((KR)*pextend(xr));
            if cfg.isRotate
                xrr_ = [1+height-xrr(2,:);xrr(1,:)];
            else
                xrr_ = xrr(1:2,:);
            end
            
            xll = pflat((KL)*pextend(xl));
            
            if 1
                pixRectR = xrr(1:2,:)'; % Orig2Rect(xrr(1:2,:)', KR, intrMatRightNew, rotMatRight, stereoParam.kcRight);
                
                pixRectL = xll(1:2,:)'; % Orig2Rect(xll(1:2,:)', KL, intrMatLeftNew, rotMatLeft, stereoParam.kcLeft);
            else
                
                pixRectR = Orig2Rect(xrr(1:2,:)', KR, intrMatRightNew, rotMatRight, stereoParam.kcRight);
                
                pixRectL = Orig2Rect(xll(1:2,:)', KL, intrMatLeftNew, rotMatLeft, stereoParam.kcLeft);
            end
            %            cfg.leftCorner{i,1} = goodId;
            %            cfg.leftCorner{i,2} = pixRectL;
            
%             if ~cfg.isRotate
%                 dispTemp= abs(pixRectL(:,1) - pixRectR(:,1));
%                 epilineErr = abs(pixRectL(:,2) - pixRectR(:,2));
%                 cfg.Corner{i,1} = goodId;
%                 cfg.Corner{i,2} = pixRectL;
%             else
%                 dispTemp= abs(pixRectL(:,2) - pixRectR(:,2));
%                 epilineErr = abs(pixRectL(:,1) - pixRectR(:,1));
%                 cfg.Corner{i,1} = goodId;
%                 cfg.Corner{i,2} = pixRectR;
%             end
           
%            depthTemp = intrMatLeftNew(1,1) * norm(stereoParam.transVecRef) ./ (dispTemp + (princpPtR(1) - princpPtL(1)));
           

%            [XYZ] = GetXYZFromDepth(intrMatLeftNew, pixRectL, depthTemp);
           
%            X = reshape(XYZ(:,1),cfg.cb_row, cfg.cb_col)';
%            Y = reshape(XYZ(:,2),cfg.cb_row, cfg.cb_col)';
%            Z = reshape(XYZ(:,3),cfg.cb_row, cfg.cb_col)';
%            xyz_comp = [X(:) Y(:) Z(:)];
%            xyzMat = permute(reshape(permute(xyz_comp, [1,3,2]), [cfg.cb_col, cfg.cb_row, 3]), [2,1,3]);
%            
%            lenHori = sqrt(sum(diff(xyzMat,1,2).^2,3));
%            lenVerti = sqrt(sum(diff(xyzMat,1,1).^2,3));
%            
%            horiErr = mean(abs(lenHori(:) - cfg.cb_size));
%            vertiErr = mean(abs(lenVerti(:) - cfg.cb_size));
%            
% %            xyErr(i,:) = [mean(epilineErr) horiErr vertiErr];
%            xyErr(i,:) = [horiErr vertiErr];
           
           if 0
%                [rectImgLtemp, rectImgRtemp] = RectifyImagePair(stereoParam, imread(leftList1{i}), imread(rightList1{i}));
               imgL__1 = imread(cfg.Corner{i,3});
               if cfg.isRotate
                   rectImgLtemp = uint8(interp2(xGrid, yGrid, double(imgL__1(:,:,1)),xRect2OrigR,yRect2OrigR));
               else
                   rectImgLtemp = uint8(interp2(xGrid, yGrid, double(imgL__1(:,:,1)),xRect2OrigL,yRect2OrigL));
               end
               rectImgRtemp = undistortimage(imread(imgLst11{i}), stereoParam.focRight, KR(1,3), KR(2,3), kr1_, kr2_, kr3_, pr1_, pr2_);
               figure,subplot(1,2,1);imshow(rectImgRtemp);hold on;plot(pixRectR(:,1), pixRectR(:,2),'.r'); subplot(1,2,2),imshow(imread(imgLst11{i}));hold on;plot(cbcXYRR{(i)}(1,:), cbcXYRR{(i)}(2,:),'.r')
               figure,subplot(1,2,1);imshow(rectImgLtemp);hold on;plot(pixRectL(:,1), pixRectL(:,2),'.r'); subplot(1,2,2),imshow(imgL__1);hold on;plot(cbcXYLL{(i)}(1,:), cbcXYLL{(i)}(2,:),'.r')
           end
           
           
           if 0
               rt = [rodrigues(camParamL.rotVec(:,goodId(i))) camParamL.tranVec(:,goodId(i));0 0 0 1];
           else
               if 0
                   rt = [rodrigues(camParamL.rotVec(:,(i))) camParamL.tranVec(:,(i));0 0 0 1];
               else
                   rt = [rodrigues(stereoParam.optPose(1:3,(i))) stereoParam.optPose(4:6,(i));0 0 0 1];
               end
            end
            
            
            
%             cbPt = rt(1:3,1:3)*pextend(cbGridR{1}) + repmat(rt(1:3,4),1,126);
            
            cbPt = rt(1:3,1:3)*pt3d' + repmat(rt(1:3,4),1,size(pt3d,1));
            
            ptIcs_L = pflat(KL*cbPt);  ptIcs_L = ptIcs_L(1:2,:)';
            
            if 0
                imgL__1 = imread(cfg.Corner{i,3});
                if cfg.isRotate
                    rectImgLtemp = uint8(interp2(xGrid, yGrid, double(imgL__1(:,:,1)),xRect2OrigR,yRect2OrigR));
                else
                    rectImgLtemp = uint8(interp2(xGrid, yGrid, double(imgL__1(:,:,1)),xRect2OrigL,yRect2OrigL));
                end
                figure,imshow(rectImgLtemp);hold on;plot(pixRectL(:,1), pixRectL(:,2),'or');plot(ptIcs_L(:,1),ptIcs_L(:,2),'xg')
            end
            if cfg.isRotate
                cbPt_ = [-cbPt(2,:); cbPt(1,:); cbPt(3,:)];
            else
                cbPt_ = cbPt;
            end
            ptIcs = TransformAndProject(cbPt', KR, rodrigues(stereoParam.rotVecRef), stereoParam.transVecRef);
            
%             ptIcs2 = TransformAndProject(pt3d_', KR_, r__, t_);
            ptIcs_ = TransformAndProject(cbPt_', KR_, r__, t_);
            
            
            
            if 0
                figure,imshow(zeros(nr,nc));hold on;plot(xrr(1,:),xrr(2,:),'or');plot(ptIcs(:,1),ptIcs(:,2),'.g')
                
                
                rectImgRtemp = undistortimage(imread(imgLst11{i}), stereoParam.focRight, KR(1,3), KR(2,3), kr1_, kr2_, kr3_, pr1_, pr2_);
                if cfg.isRotate
                    figure,imshow(imrotate(rectImgRtemp, -90));hold on;plot(xrr_(1,:),xrr_(2,:),'or');plot(ptIcs_(:,1),ptIcs_(:,2),'.g')
                else
                     figure,imshow(imrotate(rectImgRtemp, 0));hold on;plot(xrr_(1,:),xrr_(2,:),'or');plot(ptIcs_(:,1),ptIcs_(:,2),'.g')
                end
            end
            err(i,1) = norm(mean(abs(xrr(1:2,:)'-ptIcs)));
            err_rotate(i,1) = norm(mean(abs(xrr_(1:2,:)'-ptIcs_)));
        end
         fprintf(sprintf('\n\n\n### average L to RGB error: %0.4f pixel ###\n\n\n',mean(err)));
        
        %         [rectParamL, rectParamR, rotMatLeft, rotMatRight,  intrMatLeftNew, intrMatRightNew] = GetRectifyParam2(stereoParam, [nr nc]);
        
    end
    if 0
        cfg.RGBIntrMat = [KR_];
    else
        cfg.RGBIntrMat = [KR];
        cfg.RGBIntrMat(1,1) = KR(1,1) + cfg.ext_focal;
        cfg.RGBIntrMat(2,2) = KR(2,2) + cfg.ext_focal;
    end
    close all;
end
cd(workDir)






if cfg.check_depth
    
    goodId = intersect(goodIdL,goodIdR);
    
    dispDirTemp = cfg.dispInfo(goodId,:);
    
    cbcXYLL = cbcXYL(ismember(goodIdL,goodId));
    cbcXYRR = cbcXYR(ismember(goodIdR,goodId));
    
    
    pt3d = [cbGridL{1,1}' cbGridL{1,1}(2,:)'];
    pt3d(:,3) = 0;
    
    xyErr = [];
    
    for i = 1 : length(goodId) %length(cbcXYL)
        
        if 0
            [xr] = normalize_pixel(cbcXYR{goodId(i)},stereoParam.focRight,stereoParam.cenRight,stereoParam.kcRight,stereoParam.alphaRight);
        else
            [xr] = normalize_pixel(cbcXYRR{(i)}, [cfg.KR(2,2); cfg.KR(1,1)], [cfg.KR(1,3); cfg.KR(2,3)], zeros(5,1), 0);
        end
        dispChip0 = readDisp(fullfile(cfg.inputDir, dispDirTemp(i).name), cfg.img_width, cfg.img_height);
        
        [xl] = normalize_pixel(cbcXYLL{(i)}, [cfg.KL(2,2); cfg.KL(1,1)], [cfg.KL(1,3); cfg.KL(2,3)], zeros(5,1), 0);
        
        pixRectR = pflat((cfg.KR)*pextend(xr));
        pixRectR = pixRectR(1:2,:)';
        
        pixRectL = pflat((cfg.KL)*pextend(xl));
        pixRectL = pixRectL(1:2,:)';
        
        pixInd = sub2ind(size(dispChip0), round(pixRectL(:,2)), round(pixRectL(:,1)));
        
        dispChip = dispChip0(pixInd);
        
        if 1% ~cfg.isRotate
            dispTemp = abs(pixRectL(:,1) - pixRectR(:,1));
            epilineErr = abs(pixRectL(:,2) - pixRectR(:,2));
        else
            dispTemp = abs(pixRectL(:,2) - pixRectR(:,2));
            epilineErr = abs(pixRectL(:,1) - pixRectR(:,1));
        end
        
        validDisp = find(abs(dispChip) > 0);
        %             dispTemp = dispTemp(validDisp, :);
        %             dispChip = dispChip(validDisp,:);
        
        depthTemp = cfg.KL(1,1) * norm(cfg.poseVec(4:6)) ./ (dispTemp + (cfg.KR(1,3) - cfg.KL(1,3)));
        depthChip = cfg.KL(1,1) * norm(cfg.poseVec(4:6)) ./ (dispChip + (cfg.KR(1,3) - cfg.KL(1,3)));
        
        [XYZ] = GetXYZFromDepth(cfg.KL, pixRectL, depthTemp);
        
        [BB, ~, inliersXYZ] = ransacfitplane(XYZ', 3);
        BB = BB./norm(BB(1:3));
        dist2plane = dot(repmat(BB, 1, size(pixRectL, 1)), pextend(XYZ'));
        
        
        [XYZChip] = GetXYZFromDepth(cfg.KL, pixRectL, depthChip);
        
        [poseVec, outlierId] = posest(pixRectL(validDisp, :), pt3d(validDisp, :),  0.95, cfg.KL, 'repr_err');
        validDispOut = validDisp(outlierId, :);
        [poseVec1, outlierId1] = posest(pixRectL(validDisp, :), XYZ(validDisp, :),  0.95, cfg.KL, 'repr_err');
        validDispOut1 = validDisp(outlierId1, :);
        
        rt = [rodrigues(poseVec(1:3)) poseVec(4:6); 0 0 0 1];
        
        rotAxisAng = CalcDegree(BB(1:3)', rt(1:3,3)');
        
        cbPt = (rt(1:3,1:3)*pt3d' + repmat(rt(1:3,4),1,size(pt3d,1)))';
        ptIcs = TransformAndProject(pt3d,  cfg.KL, rt(1:3,1:3), rt(1:3,4));
        
        [~, xyzCalibErr] = NormalizeVector(cbPt(validDisp, :) - XYZ(validDisp, :));
        [~, xyzChipErr] = NormalizeVector(cbPt(validDisp, :) - XYZChip(validDisp, :));
        [~, pixErr] = NormalizeVector(pixRectL(validDisp, :) - ptIcs(validDisp, :));
        
        if ~cfg.isRotate
            X = reshape(XYZ(:,1),cfg.cb_row, cfg.cb_col)';
            Y = reshape(XYZ(:,2),cfg.cb_row, cfg.cb_col)';
            Z = reshape(XYZ(:,3),cfg.cb_row, cfg.cb_col)';
            xyz_comp = [X(:) Y(:) Z(:)];
            xyzMat = permute(reshape(permute(xyz_comp, [1,3,2]), [cfg.cb_col, cfg.cb_row, 3]), [2,1,3]);
        else
            X = reshape(XYZ(:,1),cfg.cb_col, cfg.cb_row)';
            Y = reshape(XYZ(:,2),cfg.cb_col, cfg.cb_row)';
            Z = reshape(XYZ(:,3),cfg.cb_col, cfg.cb_row)';
            xyz_comp = [X(:) Y(:) Z(:)];
            xyzMat = permute(reshape(permute(xyz_comp, [1,3,2]), [cfg.cb_row, cfg.cb_col, 3]), [2,1,3]);
            
        end
        
        
        lenHori = sqrt(sum(diff(xyzMat,1,2).^2,3));
        lenVerti = sqrt(sum(diff(xyzMat,1,1).^2,3));
        
        horiErr = mean(abs(lenHori(:) - cfg.cb_size));
        vertiErr = mean(abs(lenVerti(:) - cfg.cb_size));
        
% % %         xyErr(i,:) = [mean(epilineErr) horiErr vertiErr mean(pixErr) mean(xyzChipErr) mean(xyzCalibErr) mean(abs(dist2plane)) rotAxisAng];
        xyErr(i,:) = [mean(epilineErr) horiErr vertiErr mean(pixErr) mean(xyzChipErr)  mean(abs(dist2plane))];
    end
    
    meanError = [mean(xyErr,1)];
    imgL = []; imgR = []; rectImgL = []; rectImgR = [];
    fprintf(sprintf('\n\n\n### average y coordinate error: %0.4f pixel ###\n\n\n',mean(xyErr(:,1))));
    return;
end



% cfg.Corner = {};
if cfg.calib_stereo
    cfg.Corner = {};
    goodId = intersect(goodIdL,goodIdR);
    camParamL.rotVec = camParamL.rotVec(:,ismember(goodIdL,goodId));
    camParamL.rotVecErr = camParamL.rotVecErr(:,ismember(goodIdL,goodId));
    camParamL.tranVec = camParamL.tranVec(:,ismember(goodIdL,goodId));
    camParamL.tranVecErr = camParamL.tranVecErr(:,ismember(goodIdL,goodId));
    leftImgList1 = leftImgList(ismember(goodIdL,goodId),:);
    
    
    camParamR.rotVec = camParamR.rotVec(:,ismember(goodIdR,goodId));
    camParamR.rotVecErr = camParamR.rotVecErr(:,ismember(goodIdR,goodId));
    camParamR.tranVec = camParamR.tranVec(:,ismember(goodIdR,goodId));
    camParamR.tranVecErr = camParamR.tranVecErr(:,ismember(goodIdR,goodId));
    rightImgList1 = rightImgList(ismember(goodIdR,goodId),:);
    
    stereoParam = CbCalibStereo(camParamL, camParamR, cbcXYL(ismember(goodIdL,goodId)), cbcXYR(ismember(goodIdR,goodId)), cbGridL(ismember(goodIdL,goodId)), config, config);
    
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
    cfg.Corner = cell(length(goodId), 3);
    cfg.LRIntrMat = intrMatLeftNew;
% %     cfg.LRIntrMat(1,1) = intrMatLeftNew(1,1) + cfg.ext_focal;
% %     cfg.LRIntrMat(2,2) = intrMatLeftNew(2,2) + cfg.ext_focal;
    cfg.LRBaseline = norm(stereoParam.transVecRef);
    try
        for i = 1 : length(goodId) %length(cbcXYL)
            
            if 0
                [xr] = normalize_pixel(cbcXYR{goodId(i)},stereoParam.focRight,stereoParam.cenRight,stereoParam.kcRight,stereoParam.alphaRight);
            else
                [xr] = normalize_pixel(cbcXYRR{(i)},stereoParam.focRight,stereoParam.cenRight,stereoParam.kcRight,stereoParam.alphaRight);
            end
            
            [xl] = normalize_pixel(cbcXYLL{(i)},stereoParam.focLeft,stereoParam.cenLeft,stereoParam.kcLeft,stereoParam.alphaLeft);
            
            
            
            xrr = pflat((KR)*pextend(xr));
            
            xll = pflat((KL)*pextend(xl));
            
            if 0
                pixRectR = Orig2Rect(xrr(1:2,:)', KR, intrMatRightNew, rotMatRight, stereoParam.kcRight);
                
                pixRectL = Orig2Rect(xll(1:2,:)', KL, intrMatLeftNew, rotMatLeft, stereoParam.kcLeft);
            elseif 0
                
                pixRectR = xrr(1:2,:)'; % Orig2Rect(xrr(1:2,:)', KR, intrMatRightNew, rotMatRight, stereoParam.kcRight);
                
                pixRectL = xll(1:2,:)'; % Orig2Rect(xll(1:2,:)', KL, intrMatLeftNew, rotMatLeft, stereoParam.kcLeft);
            else
                 pixRectR = Orig2Rect(cbcXYRR{(i)}', KR, intrMatRightNew, rotMatRight, stereoParam.kcRight);
                
                pixRectL = Orig2Rect(cbcXYLL{(i)}', KL, intrMatLeftNew, rotMatLeft, stereoParam.kcLeft);
                
            end
            %            cfg.leftCorner{i,1} = goodId;
            %            cfg.leftCorner{i,2} = pixRectL;
            
            if ~cfg.isRotate
                dispTemp= abs(pixRectL(:,1) - pixRectR(:,1));
                epilineErr = abs(pixRectL(:,2) - pixRectR(:,2));
                cfg.Corner{i,1} = goodId;
                cfg.Corner{i,2} = pixRectL;
                cfg.Corner{i,3} = leftImgList1{i};
            else
                dispTemp= abs(pixRectL(:,2) - pixRectR(:,2));
                epilineErr = abs(pixRectL(:,1) - pixRectR(:,1));
                cfg.Corner{i,1} = goodId;
                cfg.Corner{i,2} = pixRectR;
                cfg.Corner{i,3} = rightImgList1{i};
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
           
%            xyErr(i,:) = [mean(epilineErr) horiErr vertiErr];
           xyErr(i,:) = [horiErr vertiErr];
           
           if 0
               [rectImgLtemp, rectImgRtemp] = RectifyImagePair(stereoParam, imread(leftList1{i}), imread(rightList1{i}));
               figure,subplot(1,2,1);imshow(rectImgRtemp);hold on;plot(pixRectR(:,1), pixRectR(:,2),'.r'); subplot(1,2,2),imshow(imread(rightList1{i}));hold on;plot(cbcXYRR{(i)}(1,:), cbcXYRR{(i)}(2,:),'.r')
               figure,subplot(1,2,1);imshow(rectImgLtemp);hold on;plot(pixRectL(:,1), pixRectL(:,2),'.r'); subplot(1,2,2),imshow(imread(leftList1{i}));hold on;plot(cbcXYLL{(i)}(1,:), cbcXYLL{(i)}(2,:),'.r')
           end
           
           
           if 0
               rt = [rodrigues(camParamL.rotVec(:,goodId(i))) camParamL.tranVec(:,goodId(i));0 0 0 1];
           else
               if 0
                   rt = [rodrigues(camParamL.rotVec(:,(i))) camParamL.tranVec(:,(i));0 0 0 1];
               else
                   rt = [rodrigues(stereoParam.optPose(1:3,(i))) stereoParam.optPose(4:6,(i));0 0 0 1];
               end
            end
            
            
            
%             cbPt = rt(1:3,1:3)*pextend(cbGridR{1}) + repmat(rt(1:3,4),1,126);
            
            cbPt = rt(1:3,1:3)*pt3d' + repmat(rt(1:3,4),1,size(pt3d,1));
            
            
            ptIcs = TransformAndProject(cbPt', KR, rodrigues(stereoParam.rotVecRef), stereoParam.transVecRef);
            if 0
                figure,imshow(zeros(nr,nc));hold on;plot(xrr(1,:),xrr(2,:),'or');plot(ptIcs(:,1),ptIcs(:,2),'.g')
            end
            err(i,1) = norm(mean(abs(xrr(1:2,:)'-ptIcs)));
            
        end
    catch
        err = 1.11111111;
        askjlh = 1;
    end
    
    meanError1 = mean(err);
    
    meanError = [meanError1 mean(xyErr,1)];
    
    fprintf(sprintf('\n\n\n### average reprojection error: %0.4f pixel ###\n\n\n',mean(err)));
    close all
    % % id = find(err < 10);
    % % imgLst1 = imgLst1([id;length(imgLst1)/2+id]);
    
    
    [rectParamL, rectParamR, rotMatLeft, rotMatRight] = GetRectifyParam2(stereoParam, size((imread(imgLst1{1,1}))));
    if 0
        figure,pixNewAll = plotUndistortion(nc, nr, KL, KL,stereoParam.kcLeft,eye(3),50);
        figure,pixNewAll = plotUndistortion(nc, nr, KL, rectParamL.rectIntrMat,stereoParam.kcLeft,rotMatLeft,50);
    end
    baseline = norm(stereoParam.transVecRef);
    % paraDir = 'D:\Temp\20181018\dump_45_20181018';
    stereoParam_File = strcat(paraDir,'\stereoParam.mat');
    stereo_param_File = strcat(paraDir,'\stereo_param.mat');
    rectify_param_File = strcat(paraDir,'\rectify_param.mat');
    calib_File = strcat(paraDir,'\calib.mat');
    save(stereoParam_File,'stereoParam');
    save(stereo_param_File,'stereoParam');
    if 0
        save(rectify_param_File,'baseline','rectParamL','rectParamR');
    end
    save(calib_File,'stereoParam');
    IMU = [1 1 1 1 1 1];
    focL = stereoParam.focLeft; cenL = stereoParam.cenLeft; alphaL = stereoParam.alphaLeft; kcL = stereoParam.kcLeft;
    focR = stereoParam.focRight; cenR = stereoParam.cenRight; alphaR = stereoParam.alphaRight; kcR = stereoParam.kcRight;
    % rotMat = rodrigues(stereoParam.rotVecRef)'; transVec = stereoParam.transVecRef;
    rotMat = rodrigues(stereoParam.rotVecRef)'; transVec = stereoParam.transVecRef;
    ImuData = IMU';
    calibRomFid = fopen(fullfile(paraDir,sprintf('calib_%d.rom',nr)),'w');
    fwrite(calibRomFid, [focL;cenL;alphaL;kcL;focR;cenR;alphaR;kcR;rotMat(:);transVec;ImuData],'single'); fclose(calibRomFid);
    kl1 = stereoParam.kcLeft(1);
    kl2 = stereoParam.kcLeft(2);
    kl3 = stereoParam.kcLeft(5);
    pl1 = stereoParam.kcLeft(3);
    pl2 = stereoParam.kcLeft(4);
    kr1 = stereoParam.kcRight(1);
    kr2 = stereoParam.kcRight(2);
    kr3 = stereoParam.kcRight(5);
    pr1 = stereoParam.kcRight(3);
    pr2 = stereoParam.kcRight(4);
    
    
    
    [matX, matY] = meshgrid(1:nc, 1:nr);
    if 0
        pixRectL = Orig2Rect([matX(:) matY(:)], KL, intrMatLeftNew, rotMatLeft,stereoParam.kcLeft);
        pixRectR = Orig2Rect([matX(:) matY(:)], KR, intrMatRightNew, rotMatRight,stereoParam.kcRight);
    else
        pixRectL = remapRect([matX(:) matY(:)]', intrMatLeftNew, KL, stereoParam.kcLeft, rotMatLeft);
        pixRectR = remapRect([matX(:) matY(:)]', intrMatRightNew, KR, stereoParam.kcRight, rotMatRight);
    end
    reMapLX = reshape(pixRectL(:,1), nr, nc);
    reMapLY = reshape(pixRectL(:,2), nr, nc);
    reMapRX = reshape(pixRectR(:,1), nr, nc);
    reMapRY = reshape(pixRectR(:,2), nr, nc);
    
    if ~switchLR
        rectImgLtemp = uint8(interp2(matX, matY, double(imgL(:,:,1)),reMapLX,reMapLY));
        rectImgRtemp = uint8(interp2(matX, matY, double(imgR(:,:,1)),reMapRX,reMapRY));
    else
        rectImgLtemp = uint8(interp2(matX, matY, double(imgR(:,:,1)),reMapLX,reMapLY));
        rectImgRtemp = uint8(interp2(matX, matY, double(imgL(:,:,1)),reMapRX,reMapRY));
    end
    save(fullfile(inputDir0, 'rectMap.mat'), 'matX', 'matY', 'reMapLX', 'reMapLY', 'reMapRX', 'reMapRY');
    
    
    try
        
        nimL = undistortimage(imgL, stereoParam.focLeft, KL(1,3), KL(2,3), kl1, kl2, kl3, pl1, pl2);
        nimR = undistortimage(imgR, stereoParam.focRight, KR(1,3), KR(2,3), kr1, kr2, kr3, pr1, pr2);
        if ~cfg.switch_lr
            [rectImgL, rectImgR] = RectifyImagePair(stereoParam, imgL, imgR);
        else
            [rectImgL, rectImgR] = RectifyImagePair(stereoParam, imgR, imgL);
        end
        if 0
            figure,imshowpair(rectImgL,rectImgR)
        end
    catch
        fvjasd = 1;
    end
else
    
    meanError = [0 0 0];
    
    KL = [camParamL.foc(1) 0 camParamL.cen(1);0 camParamL.foc(2) camParamL.cen(2);0 0 1];
    %     KR = [stereoParam.focRight(1) 0 stereoParam.cenRight(1);0 stereoParam.focRight(2) stereoParam.cenRight(2);0 0 1];
    kl1 = camParamL.kc(1);
    kl2 = camParamL.kc(2);
    kl3 = camParamL.kc(5);
    pl1 = camParamL.kc(3);
    pl2 = camParamL.kc(4);
    
    imgL; % imgR, rectImgL, rectImgR
    rectImgL = undistortimage(imgL, camParamL.foc, KL(1,3), KL(2,3), kl1, kl2, kl3, pl1, pl2);
    imgR = imgL;
    rectImgR = rectImgL;
    
    stereoParam_File = strcat(paraDir,'\stereoParam.mat');
    stereo_param_File = strcat(paraDir,'\stereo_param.mat');
    rectify_param_File = strcat(paraDir,'\rectify_param.mat');
    calib_File = strcat(paraDir,'\calib.mat');
    
    
    
    
    %     save(stereoParam_File,'stereoParam');
    %     save(stereo_param_File,'stereoParam');
    %     save(rectify_param_File,'baseline','rectParamL','rectParamR');
    stereoParam = camParamL;
    save(calib_File,'stereoParam');
    
    
    
end

end
function disp = readDisp(file, w, h)
fp=fopen(file,'rb');
depth= fread(fp, 'uint16');
fclose(fp);
disp0 = reshape(depth, w, h)';
disp = disp0./2^6;
end
