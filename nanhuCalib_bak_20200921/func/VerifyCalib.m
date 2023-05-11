function   [isGood, lens] = VerifyCalib(cbImgLstLLL2check,cbImgLstRRR2check,stereoParam,config, n, calibFolder,calibFuncDir,medianPixerr,meanPixerr, dltLen,Mode)

intrMatL=[stereoParam.focLeft(1,:) 0 stereoParam.cenLeft(1,:);0 stereoParam.focLeft(1,:) stereoParam.cenLeft(2,:);0 0 1];
intrMatR=[stereoParam.focRight(1,:) 0 stereoParam.cenRight(1,:);0 stereoParam.focRight(1,:) stereoParam.cenRight(2,:);0 0 1];
rotMat=rodrigues(stereoParam.rotVecRef);
transVec=stereoParam.transVecRef;
[rectParamL, rectParamR] = GetRectifyParam(stereoParam, [480 640]);
baseline = norm(stereoParam.transVecRef);
c=rectParamR.rectIntrMat(1,3)-rectParamL.rectIntrMat(1,3);
rectFoc=rectParamR.rectIntrMat(1,1);


c = rectParamR.rectIntrMat(1,3)-rectParamL.rectIntrMat(1,3);
qMat = [1./1    0       0        -rectParamL.rectIntrMat(1,3);...
        0    1./1       0        -rectParamL.rectIntrMat(2,3);...
        0    0       0         rectParamL.rectIntrMat(1,1);...
        0    0   (1/baseline)./1            c/baseline];




      
        
        if Mode == 1
              calib_dataL = makeCalibData(cbImgLstLLL2check,8,13,config.dX,config.dY); %% verti checker board
              calib_dataR = makeCalibData(cbImgLstRRR2check,8,13,config.dX,config.dY);
        else
              calib_dataL = makeCalibData(cbImgLstLLL2check,13,8,config.dX,config.dY); %% hori checker board
              calib_dataR = makeCalibData(cbImgLstRRR2check,13,8,config.dX,config.dY);

        end
        
        dirInfo = dir(calibFolder);
%         frameNum = (length(dirInfo)-2)/2;
% % % % %         imgL = imread(fullfile(calibFolder,dirInfo(n+2).name));
% % % % %         imgR = imread(fullfile(calibFolder,dirInfo(n+2+frameNum).name));
        imgL = imread(cbImgLstLLL2check{n});
        imgR = imread(cbImgLstRRR2check{n});
        
        
        [rectImgL, rectImgR] = RectifyImagePair(stereoParam, imgL, imgR);
% % % % %         rectImgL = imgL; rectImgR = imgR;
% % % % % % % % %         [cbcXL_mat, cbcYL_mat] = DetectCbCorner(rectImgL);
        
        MakeDirIfMissing(fullfile(calibFuncDir,'Temp'));

        strL = strcat(calibFuncDir,'\Temp\tempL.bmp');
        strR = strcat(calibFuncDir,'\Temp\tempR.bmp');
        imwrite(rectImgL,strL);
        imwrite(rectImgR,strR);
        try
        [callBack, initCbcXL, initCbcYL]  = get_checkerboard_corners3(0,calib_dataL,strL,calibFuncDir);
        OCamL = 1;
        catch
            [initCbcXL, initCbcYL] = DetectCbCorner(rgb2gray(rectImgL));
            OCamL = 0;
        end
        
        try
        [callBack, initCbcXR, initCbcYR]  = get_checkerboard_corners3(0,calib_dataR,strR,calibFuncDir);
        OCamR = 1;
        catch
           [initCbcXR, initCbcYR] = DetectCbCorner(rgb2gray(rectImgR)); 
           OCamR = 0;
        end
        
        [initCbcXL, initCbcYL] = CbCoordSys(initCbcXL, initCbcYL);
        [initCbcXR, initCbcYR] = CbCoordSys(initCbcXR, initCbcYR);
        
        cbcL = cornerfinder([initCbcXL(:),initCbcYL(:)]',double(rgb2gray(rectImgL)),config.halfWinW,config.halfWinH);
        cbcR = cornerfinder([initCbcXR(:),initCbcYR(:)]',double(rgb2gray(rectImgR)),config.halfWinW,config.halfWinH);
        dltY = abs((cbcL(2,:)-cbcR(2,:)));
%         if max(dltY)<0.8 && mean(dltY)<0.25
% % % % % % % % % % % % % % % % % % % % %         if median(dltY)<0.2 && mean(dltY)<0.25 && ()
% % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % %             isGood = 1;
% % % % % % % % % % % % % % % % % % % % %         else
% % % % % % % % % % % % % % % % % % % % %             isGood = 0;
% % % % % % % % % % % % % % % % % % % % %         end
        
% % % % % % % % % %         nn = dirInfo(n+2).name;
% % % % % % % % % %         nnn = nn(5:length(nn)-4);
        
%         fprintf(strcat(fullfile(calibFolder,dirInfo(n+2).name),sprintf('\nimage number is %d, median(dltY) is %f, mean(dltY) is %f\n\n',str2double(nnn),median(dltY),mean(dltY))));
%         fprintf(sprintf('\nimage number is %d, median(dltY) is %f, mean(dltY) is %f\n\n',str2double(nnn),median(dltY),mean(dltY)));
                aAaAaAaAaAaAaAaAa = 1;
            
        cbcXL_mat = reshape(cbcL(1,:),calib_dataL.n_sq_y+1,calib_dataL.n_sq_x+1);  cbcYL_mat = reshape(cbcL(2,:),calib_dataL.n_sq_y+1,calib_dataL.n_sq_x+1);
        cbcXR_mat = reshape(cbcR(1,:),calib_dataL.n_sq_y+1,calib_dataL.n_sq_x+1);  cbcYR_mat = reshape(cbcR(2,:),calib_dataL.n_sq_y+1,calib_dataL.n_sq_x+1);
        
        

        xl = cbcXL_mat'; yl = cbcYL_mat';
        xr = cbcXR_mat'; yr = cbcYR_mat';


        ptMatches_mat_calib = [xl(:) yl(:) xr(:) yr(:)];

        xxx = (qMat*[ptMatches_mat_calib(:,1:2) -ptMatches_mat_calib(:,3)+ptMatches_mat_calib(:,1) ones((calib_dataL.n_sq_y+1)*(calib_dataL.n_sq_x+1),1)]')';
        

       ptCcs_calib = [xxx(:,1)./xxx(:,end) xxx(:,2)./xxx(:,end) xxx(:,3)./xxx(:,end)];
        
        
        %%% =======verify depth: use 'BackProjectIcs2Ccs' function, or use disparity==========%%%
        xyz = permute(reshape(permute(ptCcs_calib, [1,3,2]), [calib_dataL.n_sq_x+1,calib_dataL.n_sq_y+1,3]), [2,1,3]);
        lenX = sqrt(sum(diff(xyz,1,2).^2,3));
        lenY = sqrt(sum(diff(xyz,1,1).^2,3));
        if 0
        figure(1),subplot(1,2,1), hist(lenX(:),40);title('horizontal length(mm)');  %%%%% axis equal
        subplot(1,2,2),hist(lenY(:),40);title('vertical length(mm)');
        end
        if 0
        figure(2),subplot(2,2,3),hist((cbcL(2,:)-cbcR(2,:)),40);title('y coord diff(pixel)'); %%%%%% axis equalfigure,
        subplot(2,2,1); imshow(rectImgL);hold on;plot(cbcL(1,:),cbcL(2,:),'.m'); plot(cbcL(1,1),cbcL(2,1),'*b');  %title(strcat(num2str(OCamL),'**','Left','**Loop',num2str(n),'**Frame',dirInfo(n+2).name(16:end-4)));
        subplot(2,2,2); imshow(rectImgR);hold on;plot(cbcR(1,:),cbcR(2,:),'.m'); plot(cbcR(1,1),cbcR(2,1),'*b');  %title(strcat(num2str(OCamR),'**','Right','**Loop',num2str(n),'**Frame',dirInfo(frameNum+n+2).name(17:end-4)));
        subplot(2,2,4);plot(stereoParam.transVecRef,'*r');title(strcat(num2str(stereoParam.transVecRef(1)),'**',num2str(stereoParam.transVecRef(2)),'**',num2str(stereoParam.transVecRef(3))));
        pause(0.01);
        end
        
        
        fprintf('\nProcessing image %s...',cbImgLstLLL2check{n}(1:end));
        fprintf(sprintf('\nmedian(dltY) is %f, mean(dltY) is %f',median(dltY),mean(dltY)));
        fprintf(sprintf('\nmedian(lenX), mean(lenX), median(lenY), and mean(lenY) are %f %f %f %f respectively\n\n\n',median(lenX(:)),mean(lenX(:)), median(lenY(:)),mean(lenY(:))));
        lens = [median(lenX(:)) mean(lenX(:)) median(lenY(:)) mean(lenY(:))]';

        
        if median(dltY)<medianPixerr && mean(dltY)<meanPixerr &&...
                max(abs(dltY)) < 1 &&...
           (mean(lenX(:))<(config.dX + dltLen) && mean(lenX(:))>(config.dX - dltLen)) && ...
           (median(lenX(:))<(config.dX + dltLen) && median(lenX(:))>(config.dX - dltLen)) && ...
           (mean(lenY(:))<(config.dY + dltLen) && mean(lenY(:))>(config.dY - dltLen)) && ...
           (median(lenY(:))<(config.dY + dltLen) && median(lenY(:))>(config.dY - dltLen))

            isGood = 1;
        else
            isGood = 0;
        end



end