% function obj1 = ReCalcDepth(obj1)
function ReCalcDepth(obj1, imgCur, imgCurR)
obj = obj1;


% obj.angOpt2 = [];
% obj.angRng2 = [];
% obj.refAngList2 = [];


global SHOWPOLYGON DRAWPOLYGON ANGLEONLY USEGOLDENDISP UPDATEDEPTH DEPTHITERNUM probPath ...
    USELEFT2 USELEFTPLUSRIGHT2 USELEFTXRIGHT2 USERIGHT2

iterNum = 3;

[princpPtL, princpPtR] = Get(obj.camModel, 'PinholePrincpPt', obj.scaleLvl);
L2R = [rodrigues(obj.camModel.rotVec1To2) obj.camModel.transVec1To2; 0 0 0 1];


intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,obj.scaleLvl);
baseline = norm(obj.camModel.transVec1To2);
b2c = obj.coordSysAligner.pctBody2Cam(1, 1).transformMat;


inlierIdLast = obj.keyProbZ{end,2};
ProbZ_0 = obj.keyProbZ{end,4};


for ji = 1 : iterNum
    
    if ji > 1
        ProbZ = ProbZ000;
    end
    
    obj.angOpt2 = [];
    obj.angRng2 = [];
    obj.refAngList2 = [];
    obj.keyProbZ2 = {};
    
    for i = 1 : size(obj.keyProbZ,1)
        
        inlierId0 = obj.keyProbZ{i,2};
        idCommon = find(ismember(inlierId0, inlierIdLast));
        inlierId = inlierId0(idCommon);
        
        k2cRef = obj.keyProbZ{i,19};
        k2cPnp = obj.keyProbZ{i,20};
        
        thetaSamp = [obj.configParam.theta_range(1) : obj.configParam.theta_sample_step : obj.configParam.theta_range(2)];
        thetaRange = k2cRef + [obj.configParam.theta_range(1) : obj.configParam.theta_sample_step : obj.configParam.theta_range(2)];
        thetaRng = thetaRange;
        
        
        
        Pix = obj.keyProbZ{i,9}(idCommon,:);
        zList = obj.keyProbZ{i,10};
        dispList = obj.keyProbZ{i,11};
        dispGTTmp = obj.keyProbZ{i,12};
        dispCurList = obj.keyProbZ{i,13}(idCommon,:);
        pt2dCurR = obj.keyProbZ{i,14}(idCommon,:);
        dispMapCurGTList = obj.keyProbZ{i,15}(idCommon,:);
        pixGTR = obj.keyProbZ{i,16}(idCommon,:);
        depthListGT = obj.keyProbZ{i,17};
        XYZ = obj.keyProbZ{i,18}(:,idCommon);
        pt2dCur = obj.keyProbZ{i,6}(idCommon,:);
        pixGT = obj.keyProbZ{i,7}(idCommon,:);
        trackingError = obj.keyProbZ{i,8}(idCommon,:);
        % [ {pt2dCur} {pixGT} {trackingError} {Pix} {zList} {dispList} {dispGTTmp} {dispCurList} {disp2dCurR} {dispMapCurGTList} {pixGTR} {depthListGT} {XYZ}]
        % [ {pt2dCur} {pixGT} {trackingError} {Pix} {zList} {dispList} {dispGTTmp} {dispCurList} {pt2dCurR} {dispMapCurGTList} {pixGTR} {depthListGT} {XYZ}]
        
        disparityRng = [-obj.configParam.disparity_error : obj.configParam.disparity_sample_step : obj.configParam.disparity_error];
        disparityError = dispList(inlierId) - dispGTTmp(inlierId);
        disparityErrorRound = round(disparityError./obj.configParam.disparity_sample_step).*obj.configParam.disparity_sample_step;
        
        ZTrue = zList(inlierId);
        
        metricPrevPtCcs = intrMat\HomoCoord(Pix',1);
        metricPrevPtCcs = normc(metricPrevPtCcs);
        scaleAll = ZTrue./metricPrevPtCcs(3,:)';
        %                                   scale0 = zTrue./metricPrevPtCcs(3,:)';
        keyCcsXYZAll = [repmat(scaleAll',3,1).*metricPrevPtCcs];
        
        if ji == 1
            if i == 1
                ProbZ = ProbZ_0;
                ZZVec1 = obj.featPtManager.localTrace.sampleZ(inlierIdLast,:);
            else
                ProbZ = ProbZ000;
            end
        end
        
        for jkl = 1 : DEPTHITERNUM
            
            
            
            [maxFeatZ,idMax] = max(ProbZ');
            maxFeatZ = maxFeatZ';
            maxFeatZ = repmat(maxFeatZ, 1, size(ProbZ,2));
            ProbZTmp_norm = ProbZ./maxFeatZ;
            
            % %                         ProbZ = ProbZTmp_norm;
            
            depthGTOfst = round(-(disparityErrorRound)./obj.configParam.disparity_sample_step);
            depthGTInd = depthGTOfst + (size(ProbZ,2)+1)/2;
            depthGTInd(depthGTInd < 1) = 1;
            depthGTInd(depthGTInd > size(ProbZ,2)) = size(ProbZ,2);
            
            DepthGTInd(jkl,:) = depthGTInd';
            
            depthGTIndAll = sub2ind(size(ProbZ),[1:size(ProbZ,1)]', depthGTInd);
            figure(143),
            if jkl == 1 %DEPTHITERNUM
                if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                    probDir = probPath;  % fullfile(pwd, 'prob');
                    saveas(gcf,fullfile(probDir,sprintf('hist_%05d.png',length(dir(fullfile(probDir,'hist_*.png')))+1)));
                    saveas(gcf,fullfile(probDir,sprintf('hist_%05d.fig',length(dir(fullfile(probDir,'hist_*.fig')))+1)));
                    
                end
            end
            ProbZTmpTmp_norm = ProbZTmp_norm(depthGTIndAll);
            figure(143),clf;hist(ProbZTmpTmp_norm(~isnan(ProbZTmpTmp_norm)),50); title(num2str(sum(~isnan(ProbZTmpTmp_norm))));
            
            
            
            
            
            
            ReprojErrVecTmp = {}; ReprojErrVecRTmp = {};
            for jj = 1 : length(thetaRng)
                k2cBodyTmp = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(thetaRng(jj)),zeros(3,1));
                k2cCam = b2c*k2cBodyTmp.transformMat/b2c;
                homocurrCcsXYZALL = k2cCam*HomoCoord(keyCcsXYZAll,1);
                pixKey = pflat(intrMat*homocurrCcsXYZALL(1:3,:));
                pixKeyTmp = pixKey(1:2,:)';
                % % % % % % % % % % % % %                         [~, reprojErrTmp] = NormalizeVector(pixKeyTmp - pt2dCur);
                % % % % % % % % % % % % %                         [probReproj, ~] = probDensity(0, obj.configParam.disparity_sigma, thetaRng,obj.configParam.theta_sample_step, 'theta');
                
                % %                         DispRng = repmat(dispList(inlierId,:),1,length(disparityRng)) + repmat(disparityRng,length(inlierId),1);
                % %                         for jk = 1 : length(inlierId)
                % %                              [ProbZ(jk,:), ZZVec1(jk,:)] = probDensity(dispList(inlierId(jk),:), obj.configParam.disparity_sigma, DispRng(jk,:),obj.configParam.disparity_sample_interval, 'disparity');
                % %                         end
                
                
                metricPrevPtCcsKey = intrMat\HomoCoord(Pix',1);
                metricPrevPtCcsKey = normc(metricPrevPtCcsKey);
                metricPrevPtCcsKey = repmat(metricPrevPtCcsKey, 1, length(disparityRng));
                zListTmp = intrMat(1)*baseline*ZZVec1(:)';
                Scale00 = zListTmp./metricPrevPtCcsKey(3,:);
                KeyCcsXYZVec = [ Scale00.*metricPrevPtCcsKey];
                
                homocurrCcsXYZALLVec = k2cCam*HomoCoord(KeyCcsXYZVec,1); homocurrCcsXYZALLVecR = L2R*homocurrCcsXYZALLVec;
                pixKeyVec = pflat(intrMat*homocurrCcsXYZALLVec(1:3,:)); pixKeyVecR = pflat(intrMat*homocurrCcsXYZALLVecR(1:3,:));
                pixKeyVecTmp = pixKeyVec(1:2,:)'; pixKeyVecRTmp = pixKeyVecR(1:2,:)';
                PixKeyVecTmp{jj,1} = pixKeyVecTmp; PixKeyVecRTmp{jj,1} = pixKeyVecRTmp;
                % %                            figure,imshow(imgCur);hold on;plot(pixKeyVecTmp(:,1), pixKeyVecTmp(:,2),'.g');plot(pt2dCur(:,1), pt2dCur(:,2),'.r');
                %                            figure,imshow(imgCurR);hold on;plot(pixKeyVecRTmp(:,1), pixKeyVecRTmp(:,2),'.g');plot(pt2dCurR(:,1), pt2dCurR(:,2),'.r');
                
                [~, reprojErrVecTmp] = NormalizeVector(pixKeyVecTmp - repmat(pt2dCur,length(disparityRng) ,1)); [~, reprojErrVecRTmp] = NormalizeVector(pixKeyVecRTmp - repmat(pt2dCurR,length(disparityRng) ,1));
                
                ReprojErrVecTmp{jj,1} = reprojErrVecTmp;
                ReprojErrVecRTmp{jj,1} = reprojErrVecRTmp;
                
                %                            reprojErrVecTmpMat = reshape(reprojErrVecTmp, size(DispRng));
                reprojErrVecTmpMat = reshape(reprojErrVecTmp, size(ProbZ)); reprojErrVecRTmpMat = reshape(reprojErrVecRTmp, size(ProbZ));
                
                
                reprojErrVecTmpMatT = reprojErrVecTmpMat'; reprojErrVecRTmpMatT = reprojErrVecRTmpMat';
                
                [probReprojVec22, ~] = probDensity(0, obj.configParam.reproj_sigma,obj.configParam.disparity_beta,obj.configParam.reproj_beta, reprojErrVecTmpMatT(:)',obj.configParam.reproj_sample_interval, 'reproj');
                [probReprojVecR22, ~] = probDensity(0, obj.configParam.reproj_sigma_right,obj.configParam.disparity_beta,obj.configParam.reproj_beta_right, reprojErrVecRTmpMatT(:)',obj.configParam.reproj_sample_interval, 'reproj');
                
                probReprojVec = reshape(probReprojVec22, size(reprojErrVecTmpMat,2),size(reprojErrVecTmpMat,1))'; probReprojVecR = reshape(probReprojVecR22, size(reprojErrVecRTmpMat,2),size(reprojErrVecRTmpMat,1))';
                ProbReprojVec{jj,1} = probReprojVec;  ProbReprojVecR{jj,1} = probReprojVecR;
                ProbReprojVecAll(jj,:,:) = probReprojVec; ProbReprojVecRAll(jj,:,:) = probReprojVecR;
                %                            probReprojVec = [];
                %                            for jk = 1 : length(inlierId)
                %                                %                              [ProbZ(jk,:), ZZVec1(jk,:)] = probDensity(dispList(inlierId(jk),:), obj.configParam.disparity_sigma, DispRng(jk,:),obj.configParam.disparity_sample_interval, 'disparity');
                %                                [probReprojVec(jk,:), ~] = probDensity(0, obj.configParam.reproj_sigma, reprojErrVecTmpMat(jk,:),obj.configParam.reproj_sample_interval, 'reproj');
                %                                %
                %                            end
                if USERIGHT2
                    %                                     Probb(jj,:,:) = ProbZ.*probReprojVec.*probReprojVecR;
                    Probb(jj,:,:) = ProbZ.*probReprojVecR;
                end
                % % % % % % %                                     Probb(jj,:,:) = ProbZ.*(probReprojVec + probReprojVecR);
                if USELEFT2
                    Probb(jj,:,:) = ProbZ.*probReprojVec;
                end
                
                if USELEFTPLUSRIGHT2
                    Probb(jj,:,:) = ProbZ.*(probReprojVec + probReprojVecR);
                end
                
                if USELEFTXRIGHT2
                    Probb(jj,:,:) = ProbZ.*(probReprojVec.*probReprojVecR);
                end
                
                
                %                            if jj <= (length(thetaRng)+1)/2 && jj >= (length(thetaRng)+1)/2 - 8
                if jj == (length(thetaRng)+1)/2 % && jj >= (length(thetaRng)+1)/2 - 8
                    
                    randId = 10;
                    tmpProb = Probb(jj,randId,:);
                    tmpProb2 = permute(Probb(jj,:,:),[2 3 1]);
                    maxx = (max(tmpProb2'));
                    maxx(isnan(maxx) | isinf(maxx)) = [];
                    maxxsum = sum((maxx));
                    %                            figure(133),clf;subplot(2,1,1);plot(ZZVec1(randId,:),ProbZ(randId,:));hold on;plot(ZZVec1(randId,:),probReprojVec(randId,:));
                    %                                           subplot(2,1,2);plot(ZZVec1(randId,:),tmpProb(:)');
                    if 0
                        figure(133),clf;subplot(4,1,1);plot(ProbZ');title(num2str(rad2deg(thetaRng(jj))));  subplot(4,1,2);plot(probReprojVec'); subplot(4,1,3);plot(probReprojVecR'); subplot(4,1,4);plot(tmpProb2');title(num2str(maxxsum));
                        figure(135),clf;subplot(1,2,1);imshow(imgCur);hold on;plot(pixKeyVecTmp(:,1), pixKeyVecTmp(:,2),'.g');plot(pt2dCur(:,1), pt2dCur(:,2),'.r');title(num2str(rad2deg(thetaRng(jj))));
                        subplot(1,2,2);imshow(imgCurR);hold on;plot(pixKeyVecRTmp(:,1), pixKeyVecRTmp(:,2),'.g');plot(pt2dCurR(:,1), pt2dCurR(:,2),'.r'); title('cur right');
                    end
                end
                %                         for j = 1 : length(inlierId)
                for j = 1 : 0
                    pixCur = ([obj.featPtManager.localTrace.ptIcsX(inlierId(j),keyLength) obj.featPtManager.localTrace.ptIcsY(inlierId(j),keyLength)]);
                    %                                         plot(pixCur(1),pixCur(2),'.r');
                    pix = ([obj.featPtManager.localTrace.ptIcsX(inlierId(j),1) obj.featPtManager.localTrace.ptIcsY(inlierId(j),1)]);
                    zTrue = zList(inlierId(j));
                    PixCur = [PixCur;pixCur];
                    dispTrue = dispList(inlierId(j));
                    
                    if (zTrue) < 0 || isnan(dispTrue)
                        continue;
                    else
                        cntt = cntt + 1;
                    end
                    
                    try
                        if 1
                            zRng = [intrMat(1)*baseline/(dispTrue + obj.configParam.disparity_error) intrMat(1)*baseline/(max(0.1, dispTrue - obj.configParam.disparity_error))];
                            dispRng = dispTrue + disparityRng;
                        else
                            zRng = obj.configParam.depth_uncertainty_range + zTrue;
                        end
                    catch
                        if 1
                            zRng = [intrMat(1)*baseline/(dispTrue + 3.5) intrMat(1)*baseline/(dispTrue - 3.5)];
                        else
                            zRng = [-200 300] + zTrue;
                        end
                    end
                    [probZ, ZVec1] = probDensity(dispTrue, obj.configParam.disparity_sigma,obj.configParam.disparity_beta,obj.configParam.reproj_beta, dispRng,obj.configParam.disparity_sample_interval, 'disparity');
                    
                    
                    
                    % % %                             metricPrevPtCcs = intrMat\HomoCoord(pix',1);
                    % % %                             metricPrevPtCcs = normc(metricPrevPtCcs);
                    % % %                             scale1 = zRng./metricPrevPtCcs(3,:)';
                    % % %                             scale0 = zTrue./metricPrevPtCcs(3,:)';
                    % % %                             keyCcsXYZ = [scale1(1).*metricPrevPtCcs scale1(2).*metricPrevPtCcs scale0.*metricPrevPtCcs];
                    % % %                             %                         try
                    % % %                             %                             thetaRng = obj.configParam.theta_range + theta;
                    % % %                             %                             thetaRng = obj.configParam.theta_range + theta;
                    % % %                             %                         catch
                    % % %                             %                             thetaRng = deg2rad([-0.5 0.5]) + theta;
                    % % %                             %                         end
                    % % %
                    % % %                             k2cBodyPmat0 = [roty(rad2deg(theta)) [0;0;0];0 0 0 1];
                    % % %                             k2cCamPmat0 = b2c*k2cBodyPmat0/b2c;
                    % % %                             homocurrCcsXYZ0 = k2cCamPmat0*HomoCoord(keyCcsXYZ(:,3),1);
                    % % %                             pix0 = pflat(intrMat*homocurrCcsXYZ0(1:3));
                    
                    
                    
                    metricPrevPtCcs = intrMat\HomoCoord(pix',1);
                    metricPrevPtCcs = normc(metricPrevPtCcs);
                    zListTmp = intrMat(1)*baseline*ZVec1;
                    scale00 = zListTmp./metricPrevPtCcs(3,:);
                    keyCcsXYZVec = [ scale00.*repmat(metricPrevPtCcs,1,length(scale00))];
                    
                    k2cBodyPmat0 = k2cBodyTmp.transformMat; %[roty(rad2deg(theta)) [0;0;0];0 0 0 1];
                    k2cCamPmat0 = b2c*k2cBodyPmat0/b2c;
                    homocurrCcsXYZ00 = k2cCamPmat0*HomoCoord(keyCcsXYZVec,1);
                    pix00 = pflat(intrMat*homocurrCcsXYZ00(1:3,:));
                    %                             pix00 = pflat(intrMat*pixKey);
                    %                             pix00 = (pixKey);
                    [~, reprojTmp] = NormalizeVector(pix00(1:2,:)' - repmat(pixCur,length(scale00),1));
                    %                             figure,imshow(zeros(240,320));hold on;plot(pix00(1,:),pix00(2,:),'.r');hold on;plot(pixCur(1), pixCur(2),'xg')
                    
                    %                                         Pix0 = [Pix0; pix0(1:2)'];
                    % % %                             reprojError(cntt,:) = norm([pix0(1:2)'] - [pixCur]);
                    probReproj = [];
                    %                             for kk = 1 : length(reprojTmp)
                    %                                 [probReproj(kk,1), ~] = probDensity(0, obj.configParam.reproj_sigma, reprojTmp(kk),obj.configParam.reproj_sample_interval, 'theta');
                    %                             end
                    [probReproj, ~] = probDensity(0, obj.configParam.reproj_sigma, reprojTmp',obj.configParam.reproj_sample_interval, 'reproj');
                    %                             Prob1(jj,j,:) = max(probZ.*probReproj);
                    Prob(jj,j,:) = (probZ.*probReproj);
                    % %                             PixReproj = [PixReproj;[pix0(1) pix0(2)]];
                    
                    %                                         if inlierId(j) == 1222
                    %                                             figure(1232),plot([pix0(1)'] - [pixCur(1)],[pix0(2)'] - [pixCur(2)],'+');
                    %                                         end
                    vldReproj = [vldReproj; j];
                    
                    if j == (length(disparityRng)+1)/2 && jj == (length(thetaRng)+1)/2
                        figure(132),clf;subplot(2,1,1);plot(ZVec1,probZ);hold on;plot(ZVec1,probReproj);legend('depthProb','reprojProb');subplot(2,1,2);plot(ZVec1,probZ.*probReproj);legend('thetaProb');
                        dsvk = 1;
                    end
                    % % % % %                             k2cBodyPmat1 = [roty(rad2deg(thetaRng(1))) [0;0;0];0 0 0 1];
                    % % % % %                             k2cCamPmat1 = b2c*k2cBodyPmat1/b2c;
                    % % % % %                             homocurrCcsXYZ1 = k2cCamPmat1*HomoCoord(keyCcsXYZ(:,1),1);
                    % % % % %                             pix1 = pflat(intrMat*homocurrCcsXYZ1(1:3));
                    % % % % %
                    % % % % %
                    % % % % %                             k2cBodyPmat2 = [roty(rad2deg(thetaRng(2))) [0;0;0];0 0 0 1];
                    % % % % %                             k2cCamPmat2 = b2c*k2cBodyPmat2/b2c;
                    % % % % %                             homocurrCcsXYZ2 = k2cCamPmat2*HomoCoord(keyCcsXYZ(:,1),1);
                    % % % % %                             pix2 = pflat(intrMat*homocurrCcsXYZ2(1:3));
                    % % % % %
                    % % % % %                             k2cBodyPmat3 = [roty(rad2deg(thetaRng(1))) [0;0;0];0 0 0 1];
                    % % % % %                             k2cCamPmat3 = b2c*k2cBodyPmat3/b2c;
                    % % % % %                             homocurrCcsXYZ3 = k2cCamPmat3*HomoCoord(keyCcsXYZ(:,2),1);
                    % % % % %                             pix3 = pflat(intrMat*homocurrCcsXYZ3(1:3));
                    % % % % %
                    % % % % %                             k2cBodyPmat4 = [roty(rad2deg(thetaRng(2))) [0;0;0];0 0 0 1];
                    % % % % %                             k2cCamPmat4 = b2c*k2cBodyPmat4/b2c;
                    % % % % %                             homocurrCcsXYZ4 = k2cCamPmat4*HomoCoord(keyCcsXYZ(:,2),1);
                    % % % % %                             pix4 = pflat(intrMat*homocurrCcsXYZ4(1:3));
                    % % % % %
                    % % % % %                             square = [pix1(1:2) pix2(1:2) pix4(1:2) pix3(1:2)  pix1(1:2)]';
                    %          plot(square(:,1),square(:,2),'-b');
                    %                                         plot(square([1 2],1),square([1 2],2),'-c'); plot(square([3 4],1),square([3 4],2),'-c');
                    %                                         plot(square([1 4],1),square([1 4],2),'-b'); plot(square([2 3],1),square([2 3],2),'-b');
                    %                                         plot(pix0(1),pix0(2),'.g');
                    
                    
                    
                    
                    % % %                         margLen = obj.configParam.polygon_margin;
                    % % %
                    % % %                         thetaPt1 = [square([1 4],:) [1 1]']; thetaPt2 = [square([2 3],:) [1 1]'];
                    % % %                         depthPt1 = [square([1 2],:) [1 1]']; depthPt2 = [square([3 4],:) [1 1]'];
                    % % %
                    % % %
                    % % %                         thetaLine1 = cross(thetaPt1(1,:), thetaPt1(2,:)); thetaLine2 = cross(thetaPt2(1,:), thetaPt2(2,:));
                    % % %                         depthLine1 = cross(depthPt1(1,:), depthPt1(2,:)); depthLine2 = cross(depthPt2(1,:), depthPt2(2,:));
                    % % %
                    % % %                         depthLine1 = depthLine1./norm(depthLine1(1:2)); depthLine2 = depthLine2./norm(depthLine2(1:2));
                    % % %                         %                                         [intersectPt,dist] = CalcPt2Plane2([depthLine1(1:2) 0],[square(1,:) 0],[pixCur 0]);
                    % % %                         %                                         dot(intersectPt, depthLine1);
                    % % %                         [retVal1, dist1] = Pt2Line(square(1,:), square(2,:), pixCur); [retVal2, dist2] = Pt2Line(pixCur, retVal1, pix0(1:2)');
                    % % %                         [retVal3, dist3] = Pt2Line(square(3,:), square(4,:), pixCur); [retVal4, dist4] = Pt2Line(pixCur, retVal3, pix0(1:2)');
                    % % %
                    % % %                         candi = [retVal2; retVal4];
                    % % %                         [~,iderr5] = min([dist2 dist4]);
                    % % %                         candi_ = candi(iderr5,:);
                    % % %
                    % % %
                    % % %                         err1 = dot([pixCur 1], thetaLine1); err2 = dot([pixCur 1], thetaLine2);
                    % % %                         thetaLineIntersect = cross(thetaLine1, thetaLine2);
                    % % %                         thetaLineIntersect = [thetaLineIntersect(1)/thetaLineIntersect(3) thetaLineIntersect(2)/thetaLineIntersect(3)];
                    % % %                         square2 = [square(1:4,:); thetaLineIntersect];
                    % % % % % %                         if ANGLEONLY
                    % % % % % %                             if 0
                    % % % % % %                                 dist11 = norm(thetaLineIntersect - thetaPt1(1,1:2));
                    % % % % % %                                 dist12 = norm(thetaLineIntersect - thetaPt1(2,1:2));
                    % % % % % %                                 %                                             k1 = -thetaLine1(1)/thetaLine1(2);
                    % % % % % %                                 dirVec = [(thetaPt1(1,1) - thetaLineIntersect(1)) (thetaPt1(1,2) - thetaLineIntersect(2))];
                    % % % % % %                                 dirVec = dirVec./norm(dirVec);
                    % % % % % %                                 if dist11 > dist12
                    % % % % % %                                     thetaPt1_3 = thetaPt1(1,1:2) + margLen*dirVec;
                    % % % % % %                                 else
                    % % % % % %                                     thetaPt1_3 = thetaPt1(2,1:2) + margLen*dirVec;
                    % % % % % %                                 end
                    % % % % % %                                 dist13 = norm(thetaLineIntersect - thetaPt1_3);
                    % % % % % %
                    % % % % % %                                 dist21 = norm(thetaLineIntersect - thetaPt2(1,1:2));
                    % % % % % %                                 dist22 = norm(thetaLineIntersect - thetaPt2(2,1:2));
                    % % % % % %                                 %                                             k1 = -thetaLine1(1)/thetaLine1(2);
                    % % % % % %                                 dirVec2 = [(thetaPt2(1,1) - thetaLineIntersect(1)) (thetaPt2(1,2) - thetaLineIntersect(2))];
                    % % % % % %                                 dirVec2 = dirVec2./norm(dirVec2);
                    % % % % % %                                 if dist21 > dist22
                    % % % % % %                                     thetaPt2_3 = thetaPt2(1,1:2) + margLen*dirVec2;
                    % % % % % %                                 else
                    % % % % % %                                     thetaPt2_3 = thetaPt2(2,1:2) + margLen*dirVec2;
                    % % % % % %                                 end
                    % % % % % %                                 dist23 = norm(thetaLineIntersect - thetaPt2_3);
                    % % % % % %                             else
                    % % % % % %
                    % % % % % %                                 if 0
                    % % % % % %                                     num1 = dot(thetaLine1, pix0);
                    % % % % % %                                     num2 = dot(thetaLine1, [pixCur 1]);
                    % % % % % %                                     num3 = dot(thetaLine2, pix0);
                    % % % % % %                                     num4 = dot(thetaLine2, [pixCur 1]);
                    % % % % % %                                 else
                    % % % % % %                                     num1 = dot(thetaLine1, pix0);
                    % % % % % %                                     num2 = dot(thetaLine1, [candi_ 1]);
                    % % % % % %                                     num3 = dot(thetaLine2, pix0);
                    % % % % % %                                     num4 = dot(thetaLine2, [candi_ 1]);
                    % % % % % %                                 end
                    % % % % % %                                 %                                                 [intersect,dist] = CalcPt2Plane2(lineN,ptOnline,ptIso);
                    % % % % % %
                    % % % % % %                                 if (sign(num1) == sign(num2) && sign(num3) == sign(num4))  % || err5 < obj.configParam.polygon_inlier_thresh
                    % % % % % %                                     in = true; on = true;
                    % % % % % %                                 else
                    % % % % % %                                     in = false; on = false;
                    % % % % % %                                 end
                    % % % % % %                             end
                    % % % % % %
                    % % % % % %
                    % % % % % %
                    % % % % % %                             %                                             [in,on] = inpolygon(pixCur(1),pixCur(2),square2(1:5,1),square2(1:5,2));
                    % % % % % %                         else
                    % % % % % %                             [in,on] = inpolygon(pixCur(1),pixCur(2),square(1:4,1),square(1:4,2));
                    % % % % % %                         end
                    
                    % %                         if 1
                    % %                             if in || on
                    % %
                    % %                                 validId = [validId; inlierId2(j)];
                    % %                                 validXYZAll = [validXYZAll;j];
                    % %
                    % %                                 if inlierId(j) == 1222
                    % %                                     Idd = j;
                    % %                                 end
                    % %                                 %                                                 plot(square2([1 4 5],1),square2([1 4 5],2),'-m'); plot(square2([2 3 5],1),square2([2 3 5],2),'-m');
                    % %                                 if SHOWPOLYGON
                    % %                                     plot(square([1 2],1),square([1 2],2),'-c'); plot(square([3 4],1),square([3 4],2),'-c');
                    % %                                     plot(square([1 4],1),square([1 4],2),'-b'); plot(square([2 3],1),square([2 3],2),'-b');
                    % %
                    % %                                     plot([pixCur(1) retVal1(1)],[pixCur(2) retVal1(2)], '-xy');plot([pix0(1) retVal2(1)], [pix0(2) retVal2(2)], '-xy');plot([retVal1(1) retVal2(1)], [retVal1(2) retVal2(2)], '-xy');
                    % %                                     plot([pixCur(1) retVal3(1)],[pixCur(2) retVal3(2)], '-xk');plot([pix0(1) retVal4(1)], [pix0(2) retVal4(2)], '-xk');plot([retVal3(1) retVal4(1)], [retVal3(2) retVal4(2)], '-xk');
                    % %
                    % %                                     plot(pixCur(1),pixCur(2),'.r');
                    % %                                     plot(pix0(1),pix0(2),'.g');
                    % %                                 end
                    % %                                 sdvhk = 1;
                    % %                             else
                    % %                                 %                                                 plot(pixCur(1),pixCur(2),'.r');
                    % %                                 %                                                 plot(square2([1 4 5],1),square2([1 4 5],2),'-m'); plot(square2([2 3 5],1),square2([2 3 5],2),'-m');
                    % %                                 %                                                 plot(square([1 2],1),square([1 2],2),'-c'); plot(square([3 4],1),square([3 4],2),'-c');
                    % %                                 %                                                 plot(square([1 4],1),square([1 4],2),'-b'); plot(square([2 3],1),square([2 3],2),'-b');
                    % %                                 %                                                 plot(pix0(1),pix0(2),'.g');
                    % %                                 sadb = 1;
                    % %                             end
                    % %                         else
                    % %                             validId = [validId; inlierId2(j)];
                    % %                         end
                end
            end
            %                     end
            %                     a = max(permute(Prob,[3 1 2]));
            %                     aa = permute(a,[2 3 1]);
            % % %                             vsl = VisualLocalizer(cam, csa, robotConfig.tracking_config);
            b = max(permute(Probb,[3 1 2]));
            %                             b = mean(permute(Probb,[3 1 2]));
            bb = permute(b,[2 3 1]);
            
            bb(isinf(bb)) = 0;
            bb(isnan(bb)) = 0;
            thetaProb = sum(bb');
            thetaProb = thetaProb./sum(thetaProb);
            [~,idM] = max(thetaProb);
            angOpt = thetaRng(idM);
            figure(131);
            if jkl == 1  %DEPTHITERNUM
                if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                    probDir = probPath;  % fullfile(pwd, 'prob');
                    saveas(gcf,fullfile(probDir,sprintf('prob_%05d.png',length(dir(fullfile(probDir,'prob_*.png')))+1)));
                    saveas(gcf,fullfile(probDir,sprintf('prob_%05d.fig',length(dir(fullfile(probDir,'prob_*.fig')))+1)));
                    clf;
                end
            end
            figure(131),hold on;plot(rad2deg(thetaSamp), thetaProb);plot(rad2deg(thetaSamp(idM)), thetaProb(idM),'*r');title(sprintf('k2cRef: %0.5f\nk2cPnP: %0.5f\nk2cProb: %0.5f',rad2deg(k2cRef), rad2deg(k2cPnp), rad2deg(thetaRng(idM))));grid on;
            
            
            
            b2cPmat = GetPctBody2Cam(obj.coordSysAligner, 1);
            k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(thetaRng(idM)),zeros(3,1));
            k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
            
            metricPrevPtCcsGT = intrMat\HomoCoord(Pix',1);
            metricPrevPtCcsGT = normc(metricPrevPtCcsGT);
            scaleAllGT = depthListGT(inlierId)./metricPrevPtCcsGT(3,:)';
            XYZ = [repmat(scaleAllGT',3,1).*metricPrevPtCcsGT];
            
            
            k2cT = -b2cPmat.transformMat(1:3,1:3)*k2cBodyPmat.transformMat(1:3,1:3)*b2cPmat.transformMat(1:3,1:3)'*b2cPmat.transformMat(1:3,4) + b2cPmat.transformMat(1:3,4);
            k2cCam = [k2cCamPmat.transformMat(1:3,1:3) k2cT;0 0 0 1];
            homocurrCcsXYZ = k2cCam*HomoCoord(XYZ,1); %pix0_GT_ = pflat(intrMat*homocurrCcsXYZALL_(1:3,:))';
            pixGT = pflat(intrMat*homocurrCcsXYZ(1:3,:))';
            pixGT = pixGT(:,1:2);
            pixGTR = [pixGT(:,1) - dispMapCurGTList pixGT(:,2)];
            
            
            
            
            tmpProb22 = permute(Probb(idM,:,:),[2 3 1]);
            maxx2 = (max(tmpProb22'));
            maxx2(isnan(maxx2) | isinf(maxx2)) = [];
            maxxsum2 = sum((maxx2));
            %                            figure(133),clf;subplot(2,1,1);plot(ZZVec1(randId,:),ProbZ(randId,:));hold on;plot(ZZVec1(randId,:),probReprojVec(randId,:));
            %                                           subplot(2,1,2);plot(ZZVec1(randId,:),tmpProb(:)');
            
            if 0
                figure(136),clf;subplot(4,1,1);plot(ProbZ');title(num2str(rad2deg(thetaRng(idM)))); subplot(4,1,2);plot(ProbReprojVec{idM}');subplot(4,1,3);plot(ProbReprojVecR{idM}'); subplot(4,1,4);plot(tmpProb22');title(num2str(maxxsum2));
                
                
                %                             figure,plot([ProbReprojVecR{idM}(1,:) ; ProbReprojVec{idM}(1,:); (ProbReprojVecR{idM}(1,:) + ProbReprojVec{idM}(1,:)); (ProbReprojVecR{idM}(1,:) .* ProbReprojVec{idM}(1,:)).*1000]')
                
                
                figure(137),clf;subplot(1,2,1);imshow(imgCur);hold on;plot(PixKeyVecTmp{idM}(:,1), PixKeyVecTmp{idM}(:,2),'.g');plot(pt2dCur(:,1), pt2dCur(:,2),'.r');title(num2str(rad2deg(thetaRng(idM))));
                subplot(1,2,2);imshow(imgCurR);hold on;plot(PixKeyVecRTmp{idM}(:,1), PixKeyVecRTmp{idM}(:,2),'.g');plot(pt2dCurR(:,1), pt2dCurR(:,2),'.r');title('cur right');
            end
            %                     figure(131),hold on;plot( thetaProb);title(sprintf('k2cRef: %0.5f\nk2cPnP: %0.5f\nk2cProb: %0.5f',rad2deg(k2cRef), rad2deg(k2cPnp), rad2deg(thetaRng(idM))));
            %                     drawnow;
            
            
            ProbZAll = repmat(ProbZ(:),length(thetaProb),1);
            ProbZAll = reshape(ProbZAll, size(ProbZ,1), size(ProbZ,2), length(thetaProb));
            ProbZAll = permute(ProbZAll, [3 1 2]);
            thetaProbAll = repmat(thetaProb', size(ProbZ,1)*size(ProbZ,2),1);
            thetaProbAll = reshape(thetaProbAll, length(thetaProb), size(ProbZ,1), size(ProbZ,2));
            
            %                 errTheta = thetaProbAll(:,1,2) - thetaProbAll(:,34,56);
            %                 errZ = ProbZAll(1,:,:) - ProbZAll(10,:,:); errZ = errZ(:);
            errTheta = thetaProbAll(:,1,2) - thetaProbAll(:,7,3);
            errZ = ProbZAll(1,:,:) - ProbZAll(6,:,:); errZ = errZ(:);
            
            
            if USELEFTXRIGHT2
                updatedProbZ = ProbZAll.*ProbReprojVecAll.*thetaProbAll.*ProbReprojVecRAll;
                %                                 updatedProbZ = ProbZAll.*ProbReprojVecAll.*ProbReprojVecRAll;
                %                                 updatedProbZ = ProbZAll.*thetaProbAll.*(ProbReprojVecAll + ProbReprojVecRAll);
            end
            
            if USELEFTPLUSRIGHT2
                %                                 updatedProbZ = ProbZAll.*ProbReprojVecAll.*thetaProbAll.*ProbReprojVecRAll;
                %                                 updatedProbZ = ProbZAll.*ProbReprojVecRAll;
                updatedProbZ = ProbZAll.*thetaProbAll.*(ProbReprojVecAll + ProbReprojVecRAll);
            end
            
            if USERIGHT2
                %                                 updatedProbZ = ProbZAll.*ProbReprojVecAll.*thetaProbAll.*ProbReprojVecRAll;
                %                                 updatedProbZ = ProbZAll.*ProbReprojVecAll.*ProbReprojVecRAll;
                updatedProbZ = ProbZAll.*thetaProbAll.*ProbReprojVecRAll;
            end
            
            if USELEFT2
                updatedProbZ = ProbZAll.*ProbReprojVecAll.*thetaProbAll;
            end
            
            
            updatedProbZSum = sum(updatedProbZ);
            updatedProbZSum = permute(updatedProbZSum,  [2 3 1]);
            
            errZSum = sum(updatedProbZ(:,2,3)) - updatedProbZSum(2,3);
            
            figure(140),clf;subplot(2,1,1),plot(ProbZ');title('orig ProbZ');
            subplot(2,1,2),plot(updatedProbZSum'./max(updatedProbZSum(:)));title('updated ProbZ');
            
            
            area1 = trapz(rad2deg(thetaSamp(1:idM)), thetaProb(1:idM));
            area2 = trapz(rad2deg(thetaSamp(idM:end)), thetaProb(idM:end));
            percentage = obj.configParam.theta_percentage;
            
            for uu = 2 : idM
                area11 = trapz(rad2deg(thetaSamp(1:uu)), thetaProb(1:uu));
                if (area11)/area1 > 1-(percentage)
                    break;
                end
                
            end
            ind1 = uu;
            
            for vv = idM+1 : length(thetaSamp)
                area22 = trapz(rad2deg(thetaSamp(idM:vv)), thetaProb(idM:vv));
                if area22/area2 > percentage
                    break;
                end
                
            end
            ind2 = vv;
            
            angRng = [thetaRng(ind1)  thetaRng(ind2);];
            
            %                 pt1 = [rad2deg(thetaSamp([ind1; ind2]))', thetaProb([ind1; ind2])']';
            %                 pt2 = [rad2deg(thetaSamp([ind1; ind2]))', [0 0]']';
            %                 figure(131),hold on;line(pt1(:,1), pt1(:,2),'Color',[0 0 1]);
            figure(131),hold on;plot(rad2deg(thetaSamp([ind1; ind2])), thetaProb([ind1; ind2]),'*b');
            
            figure(141);
            if jkl == 1  %DEPTHITERNUM
                if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                    %                          rngDir = fullfile(pwd, 'rng');
                    saveas(gcf,fullfile(probDir,sprintf('rng_%05d.png',length(dir(fullfile(probDir,'rng_*.png')))+1)));
                    saveas(gcf,fullfile(probDir,sprintf('rng_%05d.fig',length(dir(fullfile(probDir,'rng_*.fig')))+1)));
                    clf;
                    %                                     obj.refAngList = [];
                end
            end
            
            if jkl == 1
                if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
                    obj.refAngList2 = [];
                end
            end
            
            if jkl == 1
                obj.refAngList2 = [obj.refAngList2; (k2cRef)];
            end
            ind = GetLastKeyFrameInd(obj);
            %                     angRngList = [obj.angRng2(ind-2+1:end,:);angRng];
            %                     angRngList = [obj.angRng2(end-length(obj.refAngList2)+2:end,:);angRng];
            vslAng = obj.poseWcsList(ind:end,3) - obj.poseWcsList(ind,3);
            if length(obj.angOpt2) >= length(obj.refAngList2)
                %                         optAng = obj.angOpt2(end-length(obj.refAngList2)+2:end,:) - obj.angOpt2(end-length(obj.refAngList2)+2,:);
                baseId = length(obj.angOpt2) - length(obj.refAngList2);
                optAng0 = obj.angOpt2(baseId+1:end);
                optAng = optAng0 - optAng0(1);
                if 0
                    optAngRng = obj.angRng2(baseId+1:end,:);
                    optAngRng = [optAngRng(:,1) - optAngRng(1,1)  optAngRng(:,2) - optAngRng(1,2)];
                else
                    %                             optAng0 = obj.angOpt2(baseId+1:end);
                    optAngRng = obj.angRng2(baseId+1:end,:);
                    optAngRng = [optAngRng(:,1) - optAng0(1,1)  optAngRng(:,2) - optAng0(1,1)];
                end
            else
                optAng = obj.angOpt2;
                optAngRng = obj.angRng2;
            end
            figure(141);hold on;%plot(rad2deg(obj.refAngList2),'-xr');
            % %                     plot(rad2deg(angRngList(:,1)) - rad2deg(obj.refAngList2),'-xg');
            %                     plot(rad2deg([vslAng(2:end);thetaRng(idM)]) - rad2deg(obj.refAngList2),'-xk');
            
            if ji ~= iterNum
                if length(obj.angOpt2) >= length(obj.refAngList2)
                    plot(rad2deg([optAngRng(2:end,1); thetaRng(ind1)]) - rad2deg(obj.refAngList2),'-xg');
                    plot(rad2deg([optAng(2:end);thetaRng(idM)]) - rad2deg(obj.refAngList2),'-xk');
                    plot(rad2deg([optAngRng(2:end,2); thetaRng(ind2)]) - rad2deg(obj.refAngList2),'-xb');
                else
                    if ~isempty(optAngRng)
                        plot(rad2deg([optAngRng(1:end,1); thetaRng(ind1)]) - rad2deg(obj.refAngList2),'-xg');
                        plot(rad2deg([optAng(1:end);thetaRng(idM)]) - rad2deg(obj.refAngList2),'-xk');
                        plot(rad2deg([optAngRng(1:end,2); thetaRng(ind2)]) - rad2deg(obj.refAngList2),'-xb');
                    else
                        
                        plot(rad2deg([thetaRng(ind1)]) - rad2deg(obj.refAngList2),'-xg');
                        plot(rad2deg([optAng(1:end);thetaRng(idM)]) - rad2deg(obj.refAngList2),'-xk');
                        plot(rad2deg([thetaRng(ind2)]) - rad2deg(obj.refAngList2),'-xb');
                    end
                end
            else
                if length(obj.angOpt2) >= length(obj.refAngList2)
                    plot(rad2deg([optAngRng(2:end,1); thetaRng(ind1)]) - rad2deg(obj.refAngList2),'-xr');
                    plot(rad2deg([optAng(2:end);thetaRng(idM)]) - rad2deg(obj.refAngList2),'-xc');
                    plot(rad2deg([optAngRng(2:end,2); thetaRng(ind2)]) - rad2deg(obj.refAngList2),'-xm');
                else
                    if ~isempty(optAngRng)
                        plot(rad2deg([optAngRng(1:end,1); thetaRng(ind1)]) - rad2deg(obj.refAngList2),'-xr');
                        plot(rad2deg([optAng(1:end);thetaRng(idM)]) - rad2deg(obj.refAngList2),'-xc');
                        plot(rad2deg([optAngRng(1:end,2); thetaRng(ind2)]) - rad2deg(obj.refAngList2),'-xm');
                    else
                        
                        plot(rad2deg([thetaRng(ind1)]) - rad2deg(obj.refAngList2),'-xr');
                        plot(rad2deg([optAng(1:end);thetaRng(idM)]) - rad2deg(obj.refAngList2),'-xc');
                        plot(rad2deg([thetaRng(ind2)]) - rad2deg(obj.refAngList2),'-xm');
                    end
                end

            end
            %                     plot(rad2deg(angRngList(:,2)) - rad2deg(obj.refAngList2),'-xb');
            legend('k2cUpper - k2cRef','k2cOpt - k2cRef','k2cLower - k2cRef');
            %                                        legend('k2cRef','k2cUpper','k2cOpt','k2cLower');
            
            % %                 for kl = 1 : length(thetaProb)
            % %
            % %
            % %
            % %
            % %
            % %                 end
            % % % % %                             if UPDATEDEPTH
            % % % % %                                 updatedProbZSum_ = (max(ProbZ(:))/max(updatedProbZSum(:))).*updatedProbZSum;
            % % % % %                                 obj.featPtManager.localTrace.probZ(inlierId,:) = updatedProbZSum_;
            % % % % %                                 %                 obj.keyProbZ = [obj.keyProbZ; [{sum(obj.keyFrameFlagList)} {inlierId} {ProbZ} {updatedProbZSum_} {2.^size(obj.featPtManager.localTrace.ptIcsX,2)}]];
            % % % % %                                 obj.keyProbZ = [obj.keyProbZ; [{sum(obj.keyFrameFlagList)} {inlierId} {ProbZ} {updatedProbZSum_} {(max(ProbZ(:))/max(updatedProbZSum(:)))}]];
            % % % % %                             end
            % % % % %                             drawnow;
            
            drawnow;
            updatedProbZSum_ = (max(ProbZ(:))/max(updatedProbZSum(:))).*updatedProbZSum;
            
            [depthGTInd_update, ProbZTmpTmp_update_norm, ProbZTmp_update_norm, idMax_update, depthGTIndAll_update, depthUpdateIndAll_update] = VisualLocalizer.GetDepthHist(updatedProbZSum_,disparityErrorRound,obj.configParam.disparity_sample_step);
            try
                [minVal,idididMin] = sort(ProbZTmpTmp_update_norm,'ascend');
                [maxVal,idididMax] = sort(ProbZTmpTmp_update_norm,'descend');
                %                                 checkId = inlierId([idididMin(1:5); idididMax(1:5)]);
                checkId = ([idididMin(1:5); idididMax(1:5)]);
                ArrangeCheckId;
                
                tracebackId = inlierId(checkId);
                if ~isempty(obj.feat2check)
                    tracebackId - obj.feat2check
                end
                
                doProb = 0; idM_Vec = 0; debugProb;
            catch
                svkjhj = 1;
            end
            ProbZ000 = updatedProbZSum_;
            
            
            
            
        end
        [~, trackingError] = NormalizeVector(pt2dCur - pixGT);
        obj.keyProbZ2 = [obj.keyProbZ2; [{sum(obj.keyFrameFlagList)} {inlierId} {ProbZ./max(ProbZ(:))} {updatedProbZSum_./max(updatedProbZSum_(:))} {(max(ProbZ(:))/max(updatedProbZSum(:)))} {pt2dCur} {pixGT} {trackingError} {Pix} {zList} {dispList} {dispGTTmp} {dispCurList} {pt2dCurR} {dispMapCurGTList} {pixGTR} {depthListGT} {XYZ} {k2cRef} {k2cPnp}]];
        
        obj.angOpt2 = [obj.angOpt2; angOpt];
        obj.angRng2 = [obj.angRng2; angRng];
        
    end
    
    
end


end