% 20191108  change update p2
if UpdateP2_3
    DepthProbility(DepthProbility > 10.9) = 1;
end



if doUNT
    %     [thetaProbUse, thetaProbUse_, thetaProbUse__, curPredPtIcs, weightRaw, candX, candY,candX_reshape, candY_reshape, weightRaw_reshape, idff] = UpdateNewTracking(obj, thetaSamp, thetaProb, prevFeatPtList0, ptCcsZ0, intrMat, AngleModalOrg, DepthProbility, DepthId, inlierId, DispRng,DispRefineUse__);
    [thetaProbUse, thetaProbUse_, thetaProbUse__, curPredPtIcs, weightRaw, candX, candY,candX_reshape, candY_reshape, weightRaw_reshape, idff,errPair, PixReprojExpXY,objPervPtCcsZGolden1,p2cErrList] = ...
        UpdateNewTracking(obj, thetaSamp, thetaProb, prevFeatPtList0, ptCcsZ0, intrMat, AngleModalOrg, DepthProbility, DepthId, inlierId, NewDispRng,DispRefineUse__,angOptP2C,DispKeyInPrvRng, angOpt, thetaRng, thetaProbP2C,k2cRef,k2cRef0,thetaRngP2C,thetaSampP2C,thetaProbP2CBeforeNewPlatform,thetaProbBeforeNewPlatform,thetaRngP2CBeforeNewPlatform,thetaRngBeforeNewPlatform,thetaP2C,thetaP2C0,objPervPtCcsZGolden,ThetaNoise,CamModel,CoordSysAligner,LocalTrace,imgCur,imgPrvL,REFAngList,REFAngList4,prvDepthVisual,prvDepthGT,p2cErrList,KeyFrameFlagList,ConfigParam,accumP2CTemp);
    
end
if size(weightRaw_reshape, 1) > size(ProbReprojVecAll0,1)
    compMat = zeros((size(weightRaw_reshape,1) - size(ProbReprojVecAll0,1))/2,size(weightRaw_reshape,2),size(weightRaw_reshape,3));
    
    ProbReprojVecAll0 = [compMat; permute(ProbReprojVecAll0,[1 3 2]);compMat];
else
    compMat = [];
end

if size(weightRaw_reshape, 1) < size(ProbReprojVecAll0,1)
    compMat = zeros((size(ProbReprojVecAll0,1) - size(weightRaw_reshape,1))/2,size(weightRaw_reshape,2),size(weightRaw_reshape,3));
    
    weightRaw_reshape = [compMat; weightRaw_reshape;compMat];
else
    compMat = [];
end
ProbReprojVecAll_reshape = permute(ProbReprojVecAll0, [1 3 2]);
%                                             weightRaw_reshape = permute(weightRaw_reshape, [1 3 2]);

if 0 %size(obj.featPtManager.localTrace.ptIcsX, 2) >2
    ProbReprojVecAllP2C_reshape = permute(ProbReprojVecAllP2C, [1 3 2]);
    tempNorm = NormalizeWeightMax(weightRaw_reshape).*NormalizeWeightMax(ProbReprojVecAllP2C_reshape);
else
    tempNorm = NormalizeWeightMax(weightRaw_reshape).*NormalizeWeightMax(ProbReprojVecAll_reshape);
end
%                                     if UpdateP2_3
%                                     ProbReprojVecAll = ProbReprojVecAll0.*permute(weightRaw_reshape,[1 3 2]);
%                                     end


[ExpCoord_Reproj, ExpCoord_OnlyP2, errReproj, errOnlyP2, errPt2dCur,interpValGT,interpValCur,weightRaw_reshape___1,ProbReprojVecAll_reshape___1] = CalcP2Exp(obj, candX, candY, weightRaw, idff,pt2dCur, pixGT, imgCur,candX_reshape,candY_reshape,weightRaw_reshape, ProbReprojVecAll_reshape, compMat,depthGTInd,k2cRef0,thetaRng,Pix,r_cam,tx, ty, tz,ConfigParam);

ExpCoord_OnlyP2_bak = ExpCoord_OnlyP2;
          
if 0
    ExpCoord_OnlyP2 = PixReprojExpXY;
end

if 0
    LocalTrace.ptIcsX(inlierId,end) = ExpCoord_OnlyP2(:,1);
    LocalTrace.ptIcsY(inlierId,end) = ExpCoord_OnlyP2(:,2);
end

sakjdvb = 1;

if 0
    ExpCoord_OnlyP2 = pt2dCur;
    ExpCoord_OnlyP2(:,1) = ExpCoord_OnlyP2(:,1) + 0.3*(-1)^(size(LocalTrace.ptIcsX, 2));
end

if 0
  if size(LocalTrace.ptIcsX, 2) > 2                             
      if sign(meanErrAndStd(end,7)) == sign(mean(pt2dCur(:,1) - ExpCoord_OnlyP2(:,1)))
          ExpCoord_OnlyP2(:,1) = pt2dCur(:,1) + (mean(pt2dCur(:,1) - ExpCoord_OnlyP2(:,1)))./2;
      end
  end
end
if 0
    pt2dCur0 = pt2dCur;
    pt2dCur(:,1) = pt2dCur(:,1) + mean(pt2dCur(:,1) - ExpCoord_OnlyP2(:,1));
end

errThr = 1111110.45;
if abs(mean(pt2dCur(:,1) - ExpCoord_OnlyP2(:,1))) > errThr
    ExpCoord_OnlyP2 = pt2dCur;
end

asvdhj = 1;

if 0 % size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
    ExpCoord_OnlyP2 = pt2dCur;
end


% % % % if 1 %size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
% % % %     pt2dCur = ExpCoord_OnlyP2;
% % % % end






% [ExpCoord_Reproj, ExpCoord_OnlyP2, errReproj, errOnlyP2, errPt2dCur,interpValGT,interpValCur,weightRaw_reshape___1,ProbReprojVecAll_reshape___1] = CalcP2Exp(obj, candX, candY, weightRaw, idff,pt2dCur, pixGT, imgCur,candX_reshape,candY_reshape,tempNorm, ProbReprojVecAll_reshape, compMat,depthGTInd,k2cRef0,thetaRng);



figure(FigBase + 1);
if size(LocalTrace.ptIcsX, 2) == 2
    clf;
    meanErrAndStd = [];
end
if 0
    obj.meanErrAndStd = [obj.meanErrAndStd; [mean(pt2dCur(:,1) - pixGT(:,1)) mean(pt2dCur(:,2) - pixGT(:,2)) mean(ExpCoord_OnlyP2(:,1) - pixGT(:,1)) mean(ExpCoord_OnlyP2(:,2) - pixGT(:,2)) std(errPt2dCur) std(errOnlyP2)]];
else
    meanErrAndStd = [meanErrAndStd; [mean(pt2dCur(:,1) - pixGT(:,1)) mean(pt2dCur(:,2) - pixGT(:,2)) mean(ExpCoord_OnlyP2(:,1) - pixGT(:,1)) mean(ExpCoord_OnlyP2(:,2) - pixGT(:,2)) errPair(1) errPair(2)     mean(pt2dCur(:,1) - ExpCoord_OnlyP2(:,1)) mean(pt2dCur(:,2) - ExpCoord_OnlyP2(:,2))]];
end

err1 = [(pt2dCur(:,1) - pixGT(:,1) + 0) (pt2dCur(:,2) - pixGT(:,2) + 0)];
err2 = [(ExpCoord_OnlyP2(:,1) - pixGT(:,1) + 0) (ExpCoord_OnlyP2(:,2) - pixGT(:,2) + 0)];





% figure,hold on;plot(err1(:,1),err1(:,2),'.b');plot(err2(:,1),err2(:,2),'.r');legend('old tracking','new p2 exp');quiver(err1(:,1),err1(:,2),-err1(:,1) + err2(:,1), -err1(:,2) + err2(:,2),'Color',[0 1 0]);


figure(FigBase + 1),subplot(2,2,1);cla;hist([interpValCur interpValGT],100);legend('old tracking prob','gt prob');
subplot(2,2,2),cla;plot(pt2dCur(:,1) - pixGT(:,1), pt2dCur(:,2) - pixGT(:,2),'+b');hold on;plot(ExpCoord_OnlyP2(:,1) - pixGT(:,1), ExpCoord_OnlyP2(:,2) - pixGT(:,2),'+r');axis equal; quiver(err1(:,1),err1(:,2),-err1(:,1) + err2(:,1), -err1(:,2) + err2(:,2),'Color',[0 1 0]);legend('init tracking', 'new p2 exp','change direction');axis equal;
% subplot(2,2,3),cla;plot([obj.meanErrAndStd(:,5)],'-xb');hold  on;plot([obj.meanErrAndStd(:,6)],'-xr');axis equal; legend('std(init tracking)', 'std(new p2 exp)');title('error std');
% subplot(2,2,3),cla;plot([obj.meanErrAndStd(:,5)],'-xb');hold  on;plot([obj.meanErrAndStd(:,6)],'-xr');axis equal; legend('round(k2cRef) - vsl(end-1)', 'direct calc');title('thetaP2C - gt');
% subplot(2,2,3),cla;plot([obj.meanErrAndStd(:,5)],'-xb');hold  on;plot([obj.meanErrAndStd(:,6)],'-xr');legend('round(k2cRef) - vsl(end-1)', 'direct calc');title('thetaP2C - gt');
subplot(2,2,3),cla;plot([meanErrAndStd(:,6)],'-xb');hold  on;plot([meanErrAndStd(:,6)],'-xr');legend('round(k2cRef) - vsl(end-1)', 'direct calc');title('thetaP2C - gt');
% subplot(2,2,3),cla;plot([obj.meanErrAndStd(:,5)],'-xb');hold  on;plot([obj.meanErrAndStd(:,5)],'-xr');legend('round(k2cRef) - vsl(end-1)', 'direct calc');title('thetaP2C - gt');
subplot(2,2,4),cla;hold on;plot(meanErrAndStd(:,1), meanErrAndStd(:,2),'-ob');plot(meanErrAndStd(:,3), meanErrAndStd(:,4),'-or');legend('init tracking', 'new p2 exp');
for i =  1 : size(meanErrAndStd, 1)
    
    text(double(meanErrAndStd(i,1)),double(meanErrAndStd(i,2)),num2str(i), 'Color',[0 0 1],'FontSize',15);
    text(double(meanErrAndStd(i,3)),double(meanErrAndStd(i,4)),num2str(i), 'Color',[1 0 0],'FontSize',15);
    % % if i == 1
    % %     legend('init tracking', 'new p2 exp');
    % % end
    % plot(obj.meanErrAndStd(i,1), obj.meanErrAndStd(i,2),'+b');plot(obj.meanErrAndStd(i,3), obj.meanErrAndStd(i,4),'+r');axis equal; title('tracking - gt');legend('init tracking', 'new p2 exp');
end
title('tracking - gt');
%                                             figure(FigBase + 1),clf;hist([errP2_old errP2_new],100);legend('old','new')
saveas(gcf,fullfile(probPath,sprintf(strcat('refinedTracking_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('refinedTracking_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__*.png'))))+1 + FigBase)));

sadkbj = 1;


if 0 %size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
    ExpCoord_OnlyP2 = pt2dCur;
end
if 0 %size(obj.featPtManager.localTrace.ptIcsX, 2) > 2
    pt2dCur = ExpCoord_OnlyP2;
end


if UpdateP2_3  % 0
    if 0
        %         ProbReprojVecAll = ProbReprojVecAll0.*permute(weightRaw_reshape___1.^1,[1 3 2]);
        %         ProbReprojVecAll = ProbReprojVecAll0 + permute(weightRaw_reshape___1.^1,[1 3 2]);
        ProbReprojVecAll = permute(ProbReprojVecAll_reshape___1.^1,[1 3 2]) .* permute(weightRaw_reshape___1.^1,[1 3 2]);
    else
        
        
        
        if 1
            if ~NewFlow
                reproj_sigma = ConfigParam.reproj_sigma;
                reproj_sigma_right = ConfigParam.reproj_sigma_right;
                
                ConfigParam.reproj_sigma = ConfigParam.reproj_sigma_update_z;
                ConfigParam.reproj_sigma_right = ConfigParam.reproj_sigma_update_z_right;
                
                
                [~,~,~, ~,~, ~,~,~,~,~,~,ProbReprojVecAll_1, ProbReprojVecRAll_1, ~, ~, ~, ...
                    ~,~, ~, ~,~,~,~,~,~,~]...
                    = ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, ExpCoord_OnlyP2, pt2dCurR,ProbZ, ProbZTmp_norm, imgCur, imgCurR,thetaSamp,r_cam,tx, ty, tz);
                
            else
                
                thetaProbBeforeNewPlatform1 = thetaProb;
                thetaRngBeforeNewPlatform1 = thetaRng;
                thetaSampBeforeNewPlatform1 = thetaSamp;
                
                
                [thetaProb,idM,angOpt, ReprojErrVecTmp, ReprojErrVecRTmp, ReprojErrVecTmpMatT, ReprojErrVecRTmpMatT,ValidFeatFlag,ValidFeatFlag0,ValidFeatFlagFusion,ValidFeatFlagQuant,ProbReprojVecAll_1, ProbReprojVecRAll_1, PixKeyVecTmp, PixKeyVecRTmp, Probb, ...
                    validFeatFlagFusion,validFeatFlag, ProbReprojVec, ProbReprojVecR,bb,b,pixKeyVecTmpXCoordMatStack,pixKeyVecTmpYCoordMatStack,pixKeyVecRTmpXCoordMatStack,pixKeyVecRTmpYCoordMatStack, thetaExp_tmp11,errEpiline, angOptEpi,depthC2KInd_ind]...
                    = ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, ExpCoord_OnlyP2, pt2dCurR, ProbZ, ProbZTmp_norm, imgCur, imgCurR,thetaSamp,r_cam,tx, ty, tz);
                
                
                
                [thetaRng, thetaSamp, pltfrm_1,idM_Vec00] = ArrangeThetaRng(obj,rad2deg(thetaExp_tmp11),k2cRef,sc,thetaSamp);
                
                [thetaProb,idM,angOpt, ReprojErrVecTmp, ReprojErrVecRTmp, ReprojErrVecTmpMatT, ReprojErrVecRTmpMatT,ValidFeatFlag,ValidFeatFlag0,ValidFeatFlagFusion,ValidFeatFlagQuant,ProbReprojVecAll_1, ProbReprojVecRAll_1, PixKeyVecTmp, PixKeyVecRTmp, Probb, ...
                    validFeatFlagFusion,validFeatFlag, ProbReprojVec, ProbReprojVecR,bb,b,pixKeyVecTmpXCoordMatStack,pixKeyVecTmpYCoordMatStack,pixKeyVecRTmpXCoordMatStack,pixKeyVecRTmpYCoordMatStack, thetaExp_tmp11,errEpiline, angOptEpi,depthC2KInd_ind]...
                    = ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, ExpCoord_OnlyP2, pt2dCurR, ProbZ, ProbZTmp_norm, imgCur, imgCurR,thetaSamp,r_cam,tx, ty, tz);
                
                %
%                 thetaProb = interp1(thetaRngBeforeNewPlatform1, thetaProbBeforeNewPlatform1, thetaRng);
%                 thetaProb(isnan(thetaProb)) = min(thetaProbBeforeNewPlatform1);
%                 
                if 0
                    figure,plot(rad2deg(thetaRngBeforeNewPlatform1), thetaProbBeforeNewPlatform1);hold on;plot(rad2deg(thetaRng), thetaProb);
                    
                end
                
                reproj_sigma = ConfigParam.reproj_sigma;
                reproj_sigma_right = ConfigParam.reproj_sigma_right;
                
                ConfigParam.reproj_sigma = ConfigParam.reproj_sigma_update_z;
                ConfigParam.reproj_sigma_right = ConfigParam.reproj_sigma_update_z_right;
                
                [~,~,~, ~,~, ~,~,~,~,~,~,ProbReprojVecAll_1, ProbReprojVecRAll_1, ~, ~, ~, ...
                    ~,~, ~, ~,~,~,~,~,~,~]...
                    = ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, ExpCoord_OnlyP2, pt2dCurR,ProbZ, ProbZTmp_norm, imgCur, imgCurR,thetaSamp,r_cam,tx, ty, tz);
                
                
                
                
            end
            
            
        else
            [~,~,~, ~,~, ~,~,~,~,~,~,ProbReprojVecAll_1, ProbReprojVecRAll_1, ~, ~, ~, ...
                ~,~, ~, ~,~,~,~,~,~,~]...
                = ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, pt2dCur, pt2dCurR,ProbZ, ProbZTmp_norm, imgCur, imgCurR,thetaSamp,r_cam,tx, ty, tz);
        end
        if 1
            if 1
                ProbReprojVecAll = ProbReprojVecAll_1;
                ProbReprojVecRAll = ProbReprojVecRAll_1;
            else
                ProbReprojVecAll_reshape_tmp = NormalizeWeightMax(ProbReprojVecAll_reshape);
                ProbReprojVecAll_reshape_tmp_1 = NormalizeWeightMax(permute(ProbReprojVecAll_1, [1 3 2]));
                ProbReprojVecAll = permute(ProbReprojVecAll_reshape_tmp, [1 3 2]) + permute(ProbReprojVecAll_reshape_tmp_1 , [1 3 2]);
            end
        end
        ConfigParam.reproj_sigma = reproj_sigma;
        ConfigParam.reproj_sigma_right = reproj_sigma_right;
    end
    
    
    
end


%                                            figure(FigBase + 1),clf;hist([errReproj errOnlyP2],100) ;legend('reproj exp','onlyP2 exp');
%                                            figure(FigBase + 1),clf;hist([errReproj errOnlyP2],100) ;legend('old p2 exp','new p2 exp');
%                                            figure(FigBase + 1),clf;hist([errPt2dCur errOnlyP2],100) ;legend('init NewTracking','new p2 exp');
%                                            figure(FigBase + 1),clf;histogram(errPt2dCur,100);hold on;histogram(errOnlyP2, 100);legend('init NewTracking','new p2 exp');
%                                     figure(FigBase + 1),clf;hist([errPt2dCur errOnlyP2],100);legend('init NewTracking','new p2 exp');
% % % % % % % % % % % % % % % % % % % % % % % figure(FigBase + 1);
% % % % % % % % % % % % % % % % % % % % % % % if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
% % % % % % % % % % % % % % % % % % % % % % %     clf;
% % % % % % % % % % % % % % % % % % % % % % %     obj.meanErrAndStd = [];
% % % % % % % % % % % % % % % % % % % % % % % end
% % % % % % % % % % % % % % % % % % % % % % % if 0
% % % % % % % % % % % % % % % % % % % % % % %     obj.meanErrAndStd = [obj.meanErrAndStd; [mean(pt2dCur(:,1) - pixGT(:,1)) mean(pt2dCur(:,2) - pixGT(:,2)) mean(ExpCoord_OnlyP2(:,1) - pixGT(:,1)) mean(ExpCoord_OnlyP2(:,2) - pixGT(:,2)) std(errPt2dCur) std(errOnlyP2)]];
% % % % % % % % % % % % % % % % % % % % % % % else
% % % % % % % % % % % % % % % % % % % % % % %     obj.meanErrAndStd = [obj.meanErrAndStd; [mean(pt2dCur(:,1) - pixGT(:,1)) mean(pt2dCur(:,2) - pixGT(:,2)) mean(ExpCoord_OnlyP2(:,1) - pixGT(:,1)) mean(ExpCoord_OnlyP2(:,2) - pixGT(:,2)) errPair(1) errPair(2)]];
% % % % % % % % % % % % % % % % % % % % % % % end
% % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % err1 = [(pt2dCur(:,1) - pixGT(:,1) + 0) (pt2dCur(:,2) - pixGT(:,2) + 0)];
% % % % % % % % % % % % % % % % % % % % % % % err2 = [(ExpCoord_OnlyP2(:,1) - pixGT(:,1) + 0) (ExpCoord_OnlyP2(:,2) - pixGT(:,2) + 0)];
% % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % figure,hold on;plot(err1(:,1),err1(:,2),'.b');plot(err2(:,1),err2(:,2),'.r');legend('old tracking','new p2 exp');quiver(err1(:,1),err1(:,2),-err1(:,1) + err2(:,1), -err1(:,2) + err2(:,2),'Color',[0 1 0]);
% % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % figure(FigBase + 1),subplot(2,2,1);cla;hist([interpValCur interpValGT],100);legend('old tracking prob','gt prob');
% % % % % % % % % % % % % % % % % % % % % % % subplot(2,2,2),cla;plot(pt2dCur(:,1) - pixGT(:,1), pt2dCur(:,2) - pixGT(:,2),'+b');hold on;plot(ExpCoord_OnlyP2(:,1) - pixGT(:,1), ExpCoord_OnlyP2(:,2) - pixGT(:,2),'+r');axis equal; quiver(err1(:,1),err1(:,2),-err1(:,1) + err2(:,1), -err1(:,2) + err2(:,2),'Color',[0 1 0]);legend('init tracking', 'new p2 exp','change direction');axis equal;
% % % % % % % % % % % % % % % % % % % % % % % % subplot(2,2,3),cla;plot([obj.meanErrAndStd(:,5)],'-xb');hold  on;plot([obj.meanErrAndStd(:,6)],'-xr');axis equal; legend('std(init tracking)', 'std(new p2 exp)');title('error std');
% % % % % % % % % % % % % % % % % % % % % % % % subplot(2,2,3),cla;plot([obj.meanErrAndStd(:,5)],'-xb');hold  on;plot([obj.meanErrAndStd(:,6)],'-xr');axis equal; legend('round(k2cRef) - vsl(end-1)', 'direct calc');title('thetaP2C - gt');
% % % % % % % % % % % % % % % % % % % % % % % % subplot(2,2,3),cla;plot([obj.meanErrAndStd(:,5)],'-xb');hold  on;plot([obj.meanErrAndStd(:,6)],'-xr');legend('round(k2cRef) - vsl(end-1)', 'direct calc');title('thetaP2C - gt');
% % % % % % % % % % % % % % % % % % % % % % % subplot(2,2,3),cla;plot([obj.meanErrAndStd(:,6)],'-xb');hold  on;plot([obj.meanErrAndStd(:,6)],'-xr');legend('round(k2cRef) - vsl(end-1)', 'direct calc');title('thetaP2C - gt');
% % % % % % % % % % % % % % % % % % % % % % % % subplot(2,2,3),cla;plot([obj.meanErrAndStd(:,5)],'-xb');hold  on;plot([obj.meanErrAndStd(:,5)],'-xr');legend('round(k2cRef) - vsl(end-1)', 'direct calc');title('thetaP2C - gt');
% % % % % % % % % % % % % % % % % % % % % % % subplot(2,2,4),cla;hold on;plot(obj.meanErrAndStd(:,1), obj.meanErrAndStd(:,2),'-ob');plot(obj.meanErrAndStd(:,3), obj.meanErrAndStd(:,4),'-or');legend('init tracking', 'new p2 exp');
% % % % % % % % % % % % % % % % % % % % % % % for i =  1 : size(obj.meanErrAndStd, 1)
% % % % % % % % % % % % % % % % % % % % % % %     
% % % % % % % % % % % % % % % % % % % % % % %     text(double(obj.meanErrAndStd(i,1)),double(obj.meanErrAndStd(i,2)),num2str(i), 'Color',[0 0 1],'FontSize',15);
% % % % % % % % % % % % % % % % % % % % % % %     text(double(obj.meanErrAndStd(i,3)),double(obj.meanErrAndStd(i,4)),num2str(i), 'Color',[1 0 0],'FontSize',15);
% % % % % % % % % % % % % % % % % % % % % % %     % % if i == 1
% % % % % % % % % % % % % % % % % % % % % % %     % %     legend('init tracking', 'new p2 exp');
% % % % % % % % % % % % % % % % % % % % % % %     % % end
% % % % % % % % % % % % % % % % % % % % % % %     % plot(obj.meanErrAndStd(i,1), obj.meanErrAndStd(i,2),'+b');plot(obj.meanErrAndStd(i,3), obj.meanErrAndStd(i,4),'+r');axis equal; title('tracking - gt');legend('init tracking', 'new p2 exp');
% % % % % % % % % % % % % % % % % % % % % % % end
% % % % % % % % % % % % % % % % % % % % % % % title('tracking - gt');
% % % % % % % % % % % % % % % % % % % % % % % %                                             figure(FigBase + 1),clf;hist([errP2_old errP2_new],100);legend('old','new')
% % % % % % % % % % % % % % % % % % % % % % % saveas(gcf,fullfile(probPath,sprintf(strcat('refinedTracking_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('refinedTracking_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
% % % % % % % % % % % % % % % % % % % % % % % sadkbj = 1;
function weightRaw_reshape___ = NormalizeWeightMax(weightRaw_reshape)
weightRaw_reshape_ = reshape(weightRaw_reshape, size(weightRaw_reshape,1)*size(weightRaw_reshape,2), size(weightRaw_reshape,3));
% weightRaw_reshape_sum = repmat(sum(weightRaw_reshape_),size(weightRaw_reshape_,1),1);
weightRaw_reshape_sum = repmat(max(weightRaw_reshape_),size(weightRaw_reshape_,1),1);
weightRaw_reshape__ = weightRaw_reshape_./weightRaw_reshape_sum;
weightRaw_reshape___ = reshape(weightRaw_reshape__, size(weightRaw_reshape));
end