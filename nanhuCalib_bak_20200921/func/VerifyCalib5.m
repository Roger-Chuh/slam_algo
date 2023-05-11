function   [isGood, lens] = VerifyCalib5(cbImgLstLLL2check,cbImgLstRRR2check,stereoParam,config, whichWay, n, calibFolder,calibFuncDir,medianPixerr,meanPixerr, dltLen,modeE,svrFolder, rectFolder, videoFolder, videoFileName, dumpFolder, dumpNum,delta,ScaleDown)
% % % % % [lenX, lenY, z] = VerifyCalib( 'D:\nextvpu\algorithm\output\calibration\cbccalib\results\iEye\0007', 'calib', 2,...
% % % % % 'D:\nextvpu\algorithm\output\calibration\cbccalib\stereo_pairs\iEye\0007',...
% % % % % 'D:\nextvpu\algorithm\output\calibration\cbccalib\results\iEye\0007',...
% % % % %     'C:\Users\roger\Desktop\temp5\0004\dump_all3\undist', 'C:\Users\roger\Desktop\temp5\0004\dump_all3\video',...
% % % % %     'test___2016_09_30_12_23_21_xvid.avi', 'C:\Users\roger\Desktop\temp5\0004\dump_all3',2,0,1);

% [lenX, lenY, z] = VerifyCalib( 'D:\nextvpu\algorithm\output\calibration\cbccalib\results\iEye\0007', 'mat', 2,...
%     'D:\nextvpu\algorithm\output\calibration\cbccalib\stereo_pairs\iEye\0007',...
%     'D:\nextvpu\algorithm\output\calibration\cbccalib\results\iEye\0007',...
% 'C:\Users\roger\Desktop\temp5\0004\dump_all3\undist', 'C:\Users\roger\Desktop\temp5\0004\dump_all3\video',...
% 'test___2016_09_30_12_23_21_xvid.avi', 'C:\Users\roger\Desktop\temp5\0004\dump_all3',2,0,1);

% [lenX, lenY, z] = VerifyCalib( 'D:\nextvpu\algorithm\output\calibration\cbccalib\results\iEye\0007', 'cv', 2,...
% 'D:\nextvpu\algorithm\output\calibration\cbccalib\stereo_pairs\iEye\0007',...
% 'D:\nextvpu\algorithm\output\calibration\cbccalib\results\iEye\0007',...
% 'C:\Users\roger\Desktop\temp5\0004\dump_all3\undist', 'C:\Users\roger\Desktop\temp5\0004\dump_all3\video',...
% 'test___2016_09_30_12_23_21_xvid.avi', 'C:\Users\roger\Desktop\temp5\0004\dump_all3',3,0,1);


% 'C:\Users\roger\Desktop\temp5\0004\dump_all3\undist\rectified';


% % % % % calibFileName = 'calib.mat';
% % % % % load(fullfile(stereoParamFolder,calibFileName));



% % % % % % % % % % % % % % % L=cbcXYLL{n,1}';
% % % % % % % % % % % % % % % R=cbcXYRR{n,1}';
% % % % L=cbcXYL{n,1}';
% % % % R=cbcXYR{n,1}';
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



%% straight from calibrated immage

switch whichWay
%%
    case 'calib'
        
        
        if modeE == 1
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
        
        
% % % % % % % % % % % % % % %         [cbcXL_mat, cbcYL_mat] = CbCoordSys(cbcXL_mat, cbcYL_mat);
% % % % % % % % % % % % % % %         [cbcXR_mat, cbcYR_mat] = DetectCbCorner(rectImgR);
% % % % % % % % % % % % % % %         [cbcXR_mat, cbcYR_mat] = CbCoordSys(cbcXR_mat, cbcYR_mat);
% % % % % % %         cbcXL_mat = reshape(cbcXYLL{n}(1,:),9,14);   % left x coord
% % % % % % %         cbcYL_mat = reshape(cbcXYLL{n}(2,:),9,14);   % left y coord
        
% % % % %         k=1;
% % % % %         for i =1:14:113
% % % % %             corners1(i:i+13,:)=[cbcXL_mat(k,1) cbcYL_mat(k,1);cbcXL_mat(k,2) cbcYL_mat(k,2);...
% % % % %             cbcXL_mat(k,3) cbcYL_mat(k,3);cbcXL_mat(k,4) cbcYL_mat(k,4);...
% % % % %             cbcXL_mat(k,5) cbcYL_mat(k,5);cbcXL_mat(k,6) cbcYL_mat(k,6);...
% % % % %             cbcXL_mat(k,7) cbcYL_mat(k,7);cbcXL_mat(k,8) cbcYL_mat(k,8);...
% % % % %             cbcXL_mat(k,9) cbcYL_mat(k,9);cbcXL_mat(k,10) cbcYL_mat(k,10);...
% % % % %             cbcXL_mat(k,11) cbcYL_mat(k,11);cbcXL_mat(k,12) cbcYL_mat(k,12);...
% % % % %             cbcXL_mat(k,13) cbcYL_mat(k,13);cbcXL_mat(k,14) cbcYL_mat(k,14)];
% % % % %             k=k+1;
% % % % %         end
% % % % %         k=1;
% % % % %         for i =1:14:113
% % % % %             corners2(i:i+13,:)=[cbcXR_mat(k,1) cbcYR_mat(k,1);cbcXR_mat(k,2) cbcYR_mat(k,2);...
% % % % %             cbcXR_mat(k,3) cbcYR_mat(k,3);cbcXR_mat(k,4) cbcYR_mat(k,4);...
% % % % %             cbcXR_mat(k,5) cbcYR_mat(k,5);cbcXR_mat(k,6) cbcYR_mat(k,6);...
% % % % %             cbcXR_mat(k,7) cbcYR_mat(k,7);cbcXR_mat(k,8) cbcYR_mat(k,8);...
% % % % %             cbcXR_mat(k,9) cbcYR_mat(k,9);cbcXR_mat(k,10) cbcYR_mat(k,10);...
% % % % %             cbcXR_mat(k,11) cbcYR_mat(k,11);cbcXR_mat(k,12) cbcYR_mat(k,12);...
% % % % %             cbcXR_mat(k,13) cbcYR_mat(k,13);cbcXR_mat(k,14) cbcYR_mat(k,14)];
% % % % %             k=k+1;
% % % % %         end 
        xl = cbcXL_mat'; yl = cbcYL_mat';
        xr = cbcXR_mat'; yr = cbcYR_mat';

        % get matches with even y coordinates
% % % % % % %         ptMatches_mat_calib=[corners1 corners2];
        ptMatches_mat_calib = [xl(:) yl(:) xr(:) yr(:)];
        % get depth mat
% % %         for i = 1:size(ptMatches_mat_calib,1)
% % %             disparity_mat_calib(i,:)=ptMatches_mat_calib(i,3)-ptMatches_mat_calib(i,1)-c;
% % %             z(i,:) = -rectFoc*baseline/(ptMatches_mat_calib(i,3)-ptMatches_mat_calib(i,1)-c);
% % %         end
        
% % % % %         xxx = (qMat*[ptMatches_mat_calib(:,1:2) dispMat(ind) ones(size(pix,1),1)]')';
        xxx = (qMat*[ptMatches_mat_calib(:,1:2) -ptMatches_mat_calib(:,3)+ptMatches_mat_calib(:,1) ones((calib_dataL.n_sq_y+1)*(calib_dataL.n_sq_x+1),1)]')';
        
    % % %     xxx = (qMat*[(1/ScaleDown).*pix dispMat(~isnan(dispMat)) ones(size(pix,1),1)]')';
       ptCcs_calib = [xxx(:,1)./xxx(:,end) xxx(:,2)./xxx(:,end) xxx(:,3)./xxx(:,end)];
        
        
% % %         z_mat_calib = DepthFromStereoPair(L, R, intrMatL, intrMatR, rotMat, transVec); % 
% % % % % % % % %         ptCcs_calib = BackProjectIcs2Ccs(rectParamL.rectIntrMat, ptMatches_mat_calib(:,1:2), z);  %% must use the rectified intrinsic matrix
        
        %%% =======verify depth: use 'BackProjectIcs2Ccs' function, or use disparity==========%%%
        xyz = permute(reshape(permute(ptCcs_calib, [1,3,2]), [calib_dataL.n_sq_x+1,calib_dataL.n_sq_y+1,3]), [2,1,3]);
        lenX = sqrt(sum(diff(xyz,1,2).^2,3));
        lenY = sqrt(sum(diff(xyz,1,1).^2,3));
        if 0
        figure(1),subplot(1,2,1), hist(lenX(:),40);title('horizontal length(mm)');  %%%%% axis equal
        subplot(1,2,2),hist(lenY(:),40);title('vertical length(mm)');
        end
% %         saveas(gcf,fullfile(check70Folder,strcat('SquareLength_',num2str(n),'.jpg')));
        if 0
        figure(2),subplot(2,2,3),hist((cbcL(2,:)-cbcR(2,:)),40);title('y coord diff(pixel)'); %%%%%% axis equalfigure,
        subplot(2,2,1); imshow(rectImgL);hold on;plot(cbcL(1,:),cbcL(2,:),'.m'); plot(cbcL(1,1),cbcL(2,1),'*b');  %title(strcat(num2str(OCamL),'**','Left','**Loop',num2str(n),'**Frame',dirInfo(n+2).name(16:end-4)));
        subplot(2,2,2); imshow(rectImgR);hold on;plot(cbcR(1,:),cbcR(2,:),'.m'); plot(cbcR(1,1),cbcR(2,1),'*b');  %title(strcat(num2str(OCamR),'**','Right','**Loop',num2str(n),'**Frame',dirInfo(frameNum+n+2).name(17:end-4)));
        subplot(2,2,4);plot(stereoParam.transVecRef,'*r');title(strcat(num2str(stereoParam.transVecRef(1)),'**',num2str(stereoParam.transVecRef(2)),'**',num2str(stereoParam.transVecRef(3))));
% % %         saveas(gcf,fullfile(check70Folder,strcat('CalibInfo_',num2str(n),'.jpg')));
        pause(0.01);
        end
        
        
        fprintf('\nProcessing image %s...',cbImgLstLLL2check{n}(1:end));
        fprintf(sprintf('\nmedian(dltY) is %f, mean(dltY) is %f',median(dltY),mean(dltY)));
        fprintf(sprintf('\nmedian(lenX), mean(lenX), median(lenY), and mean(lenY) are %f %f %f %f respectively\n\n\n',median(lenX(:)),mean(lenX(:)), median(lenY(:)),mean(lenY(:))));
        lens = [median(lenX(:)) mean(lenX(:)) median(lenY(:)) mean(lenY(:))]';
% % % % %         fprintf(sprintf('\nmedian(lenX) is %f',median(lenX(:))));
% % % % %         fprintf(sprintf('\nmean(lenX) is %f',mean(lenX(:))));
% % % % %         fprintf(sprintf('\nmedian(lenY) is %f',median(lenY(:))));
% % % % %         fprintf(sprintf('\nmean(lenY) is %f\n\n\n',mean(lenY(:))));
        
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



        sgqev = 1;
        
        
        
        
        
%         figure;imshow(rectImgL); hold on
%         for i =1:size(ptMatches_mat_calib,1)
%             plot(ptMatches_mat_calib(i,1),ptMatches_mat_calib(i,2),'+g');
%         end 


% for i =1:size(ptCcs_calib,1)-1%[1:13, 15: 27 ,29:41 ,43:55, 57:69, 71:83,85:97, 99:111,113:125]%
%     len_mat_calib(i,:)=norm(ptCcs_calib(i,:)-ptCcs_calib(i+1,:));
% end
% % % % len_mat_calib=len_mat_calib(len_mat_calib(:,1)>0,1);
% % % % len_mat_calib_hori=reshape(len_mat_calib',9,13);

% figure;imshow(rectImgL); hold on
% for i =1:size(ptMatches_mat_calib,1)
%     plot(ptMatches_mat_calib(i,1),ptMatches_mat_calib(i,2),'+g');
% end
%%
    case 'mat' 
        svr = StereoVideoRectifier(svrFolder,rectFolder);
        ReadVideo(fullfile(videoFolder, videoFileName),svr);
        inputDir= strcat(rectFolder,'\rectified');
        dirInfo = dir(inputDir);
        frameNum = (length(dirInfo)-2)/2;
        I1=imread(fullfile(inputDir,dirInfo(3).name));
        I2=imread(fullfile(inputDir,dirInfo(frameNum+3).name));  
        [XL_mat, YL_mat] = DetectCbCorner(I1);
        [XL_mat, YL_mat] = CbCoordSys(XL_mat, YL_mat);
        [XR_mat, YR_mat] = DetectCbCorner(I2);
        [XR_mat, YR_mat] = CbCoordSys(XR_mat, YR_mat);
        k=1;
        for i =1:14:113   %99 %
            corners1(i:i+13,:)=[XL_mat(k,1) YL_mat(k,1);XL_mat(k,2) YL_mat(k,2);...
            XL_mat(k,3) YL_mat(k,3);XL_mat(k,4) YL_mat(k,4);...
            XL_mat(k,5) YL_mat(k,5);XL_mat(k,6) YL_mat(k,6);...
            XL_mat(k,7) YL_mat(k,7);XL_mat(k,8) YL_mat(k,8);...
            XL_mat(k,9) YL_mat(k,9);XL_mat(k,10) YL_mat(k,10);...
            XL_mat(k,11) YL_mat(k,11);XL_mat(k,12) YL_mat(k,12);...
            XL_mat(k,13) YL_mat(k,13);XL_mat(k,14) YL_mat(k,14)];
            k=k+1;
        end 
        k=1;
        for i =1:14:113  %99 %
            corners2(i:i+13,:)=[XR_mat(k,1) YR_mat(k,1);XR_mat(k,2) YR_mat(k,2);...
            XR_mat(k,3) YR_mat(k,3);XR_mat(k,4) YR_mat(k,4);...
            XR_mat(k,5) YR_mat(k,5);XR_mat(k,6) YR_mat(k,6);...
            XR_mat(k,7) YR_mat(k,7);XR_mat(k,8) YR_mat(k,8);...
            XR_mat(k,9) YR_mat(k,9);XR_mat(k,10) YR_mat(k,10);...
            XR_mat(k,11) YR_mat(k,11);XR_mat(k,12) YR_mat(k,12);...
            XR_mat(k,13) YR_mat(k,13);XR_mat(k,14) YR_mat(k,14)];
            k=k+1;
        end 
        ptMatches_mat=[corners1 corners2];
        for i=1:size(ptMatches_mat,1)
            disparity_mat(i,:)=ptMatches_mat(i,3)-ptMatches_mat(i,1)-c;
            z(i,:)=-rectFoc*baseline/(ptMatches_mat(i,3)-ptMatches_mat(i,1)-c);
        end 
        ptCcs_mat = BackProjectIcs2Ccs(rectParamL.rectIntrMat, ptMatches_mat(:,1:2), z); 
        %%% =======verify depth: use 'BackProjectIcs2Ccs' function, or use disparity==========%%%
        xyz = permute(reshape(permute(ptCcs_mat, [1,3,2]), [14,9,3]), [2,1,3]);
        lenX = sqrt(sum(diff(xyz,1,2).^2,3));
        lenY = sqrt(sum(diff(xyz,1,1).^2,3));
        figure, hist(lenX(:),40);
        figure, hist(lenY(:),40);
        
        
%         figure;imshow(I1); hold on
%         for i =1:size(ptMatches_mat,1)
%             plot(ptMatches_mat(i,1),ptMatches_mat(i,2),'+g');
%         end


%         for i =1:size(ptCcs_mat,1)-1
%             len_mat(i,:)=norm(ptCcs_mat(i,:)-ptCcs_mat(i+1,:));
%         end
%         figure;imshow(I1); hold on
%         for i =1:size(ptMatches_mat,1)
%             plot(ptMatches_mat(i,1),ptMatches_mat(i,2),'+g');
%         end 
        
        
        
%%        
    case 'cv'
        [G,g,Q,dispMat,pointCloud,img1,img2] = ReadLog(dumpFolder, dumpNum,delta,ScaleDown);
%         [g, pointCloud, Q, dispMat] =ReadLog(dumpFolder, dumpNum);%Folder)%
% % % %         fileName1 = strcat('rectImgL_GPU_',num2str(dumpNum),'.bmp');
% % % %         fileName2 = strcat('rectImgR_GPU_',num2str(dumpNum),'.bmp');
% % % %         img1=imread(fullfile(dumpFolder,fileName1));
% % % %         img2=imread(fullfile(dumpFolder,fileName2));
        [XL_cv, YL_cv] = DetectCbCorner(img1);
        [XL_cv, YL_cv] = CbCoordSys(XL_cv, YL_cv);
        [XR_cv, YR_cv] = DetectCbCorner(img2);
        [XR_cv, YR_cv] = CbCoordSys(XR_cv, YR_cv);
        k=1;
        for i =1:14:113   %99 %
            corners1(i:i+13,:)=[XL_cv(k,1) YL_cv(k,1);XL_cv(k,2) YL_cv(k,2);...
            XL_cv(k,3) YL_cv(k,3);XL_cv(k,4) YL_cv(k,4);...
            XL_cv(k,5) YL_cv(k,5);XL_cv(k,6) YL_cv(k,6);...
            XL_cv(k,7) YL_cv(k,7);XL_cv(k,8) YL_cv(k,8);...
            XL_cv(k,9) YL_cv(k,9);XL_cv(k,10) YL_cv(k,10);...
            XL_cv(k,11) YL_cv(k,11);XL_cv(k,12) YL_cv(k,12);...
            XL_cv(k,13) YL_cv(k,13);XL_cv(k,14) YL_cv(k,14)];
            k=k+1;
        end
        k=1;
        for i =1:14:113  %99 %
            corners2(i:i+13,:)=[XR_cv(k,1) YR_cv(k,1);XR_cv(k,2) YR_cv(k,2);...
            XR_cv(k,3) YR_cv(k,3);XR_cv(k,4) YR_cv(k,4);...
            XR_cv(k,5) YR_cv(k,5);XR_cv(k,6) YR_cv(k,6);...
            XR_cv(k,7) YR_cv(k,7);XR_cv(k,8) YR_cv(k,8);...
            XR_cv(k,9) YR_cv(k,9);XR_cv(k,10) YR_cv(k,10);...
            XR_cv(k,11) YR_cv(k,11);XR_cv(k,12) YR_cv(k,12);...
            XR_cv(k,13) YR_cv(k,13);XR_cv(k,14) YR_cv(k,14)];
            k=k+1;
        end
        ptMatches_cv=[corners1 corners2];
        % Q=load('C:\Users\roger\Desktop\temp5\le\Q\Q.txt');
        for i=1:size(ptMatches_cv,1)
            disparity_cv(i,:)=ptMatches_cv(i,3)-ptMatches_cv(i,1);
            z1=Q*[ptMatches_cv(i,1);ptMatches_cv(i,2);disparity_cv(i,:);1];
            zz=z1';
            zzz=[zz(1,1)/zz(1,4) zz(1,2)/zz(1,4) zz(1,3)/zz(1,4)];
            z(i,1:3)=zzz;
        end
        %%% =======verify depth: use 'BackProjectIcs2Ccs' function, or use disparity==========%%%
        xyz = permute(reshape(permute(z, [1,3,2]), [14,9,3]), [2,1,3]);
        lenX = sqrt(sum(diff(xyz,1,2).^2,3));
        lenY = sqrt(sum(diff(xyz,1,1).^2,3));
        figure, hist(lenX(:),40);
        figure, hist(lenY(:),40);
        
        
%         figure;imshow(img1); hold on
%         for i =1:size(ptMatches_cv,1)
%             plot(ptMatches_cv(i,1),ptMatches_cv(i,2),'+g');
%         end
        
    otherwise
        assert(0);
end

end