% 20191108  change update p2
if UpdateP2_3
    DepthProbility(DepthProbility > 10.9) = 1;
end



if doUNT
    [thetaProbUse, thetaProbUse_, thetaProbUse__, curPredPtIcs, weightRaw, candX, candY,candX_reshape, candY_reshape, weightRaw_reshape, idff] = UpdateNewTracking(obj, thetaSamp, thetaProb, prevFeatPtList0, ptCcsZ0, intrMat, AngleModalOrg, DepthProbility, DepthId, inlierId);
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




%                                     if UpdateP2_3
%                                     ProbReprojVecAll = ProbReprojVecAll0.*permute(weightRaw_reshape,[1 3 2]);
%                                     end


[ExpCoord_Reproj, ExpCoord_OnlyP2, errReproj, errOnlyP2, errPt2dCur,interpValGT,interpValCur,weightRaw_reshape___1] =...
    CalcP2Exp(obj, candX, candY, weightRaw, idff,pt2dCur, pixGT, imgCur,candX_reshape,candY_reshape,weightRaw_reshape, ProbReprojVecAll_reshape, compMat,depthGTInd,k2cRef0,thetaRng);

if UpdateP2_3
    if 0
        ProbReprojVecAll = ProbReprojVecAll0.*permute(weightRaw_reshape___1.^1,[1 3 2]);
    else
        
        [~,~,~, ~,~, ~,~,~,~,~,~,ProbReprojVecAll, ProbReprojVecRAll, ~, ~, ~, ...
            ~,~, ~, ~,~,~,~,~,~,~]...
            = ReplayTheta(obj, thetaRng, b2c, keyCcsXYZAll, intrMat, Pix, disparityRng, baseline, ZZVec1, L2R, ExpCoord_OnlyP2, pt2dCurR,ProbZ, ProbZTmp_norm, imgCur, imgCurR,thetaSamp);
        
    end
    
    
    
end


%                                            figure(1),clf;hist([errReproj errOnlyP2],100) ;legend('reproj exp','onlyP2 exp');
%                                            figure(1),clf;hist([errReproj errOnlyP2],100) ;legend('old p2 exp','new p2 exp');
%                                            figure(1),clf;hist([errPt2dCur errOnlyP2],100) ;legend('init NewTracking','new p2 exp');
%                                            figure(1),clf;histogram(errPt2dCur,100);hold on;histogram(errOnlyP2, 100);legend('init NewTracking','new p2 exp');
%                                     figure(1),clf;hist([errPt2dCur errOnlyP2],100);legend('init NewTracking','new p2 exp');
figure(1);
if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
    clf;
    obj.meanErrAndStd = [];
end
obj.meanErrAndStd = [obj.meanErrAndStd; [mean(pt2dCur(:,1) - pixGT(:,1)) mean(pt2dCur(:,2) - pixGT(:,2)) mean(ExpCoord_OnlyP2(:,1) - pixGT(:,1)) mean(ExpCoord_OnlyP2(:,2) - pixGT(:,2)) std(errPt2dCur) std(errOnlyP2)]];


err1 = [(pt2dCur(:,1) - pixGT(:,1) + 0) (pt2dCur(:,2) - pixGT(:,2) + 0)];
err2 = [(ExpCoord_OnlyP2(:,1) - pixGT(:,1) + 0) (ExpCoord_OnlyP2(:,2) - pixGT(:,2) + 0)];

% figure,hold on;plot(err1(:,1),err1(:,2),'.b');plot(err2(:,1),err2(:,2),'.r');legend('old tracking','new p2 exp');quiver(err1(:,1),err1(:,2),-err1(:,1) + err2(:,1), -err1(:,2) + err2(:,2),'Color',[0 1 0]);


figure(1),subplot(2,2,1);cla;hist([interpValCur interpValGT],100);legend('old tracking prob','gt prob');
subplot(2,2,2),cla;plot(pt2dCur(:,1) - pixGT(:,1), pt2dCur(:,2) - pixGT(:,2),'+b');hold on;plot(ExpCoord_OnlyP2(:,1) - pixGT(:,1), ExpCoord_OnlyP2(:,2) - pixGT(:,2),'+r');axis equal; quiver(err1(:,1),err1(:,2),-err1(:,1) + err2(:,1), -err1(:,2) + err2(:,2),'Color',[0 1 0]);legend('init tracking', 'new p2 exp','change direction');
subplot(2,2,3),cla;plot([obj.meanErrAndStd(:,5)],'-xb');hold  on;plot([obj.meanErrAndStd(:,6)],'-xr');axis equal; legend('std(init tracking)', 'std(new p2 exp)');title('error std');
subplot(2,2,4),cla;hold on;plot(obj.meanErrAndStd(:,1), obj.meanErrAndStd(:,2),'-ob');plot(obj.meanErrAndStd(:,3), obj.meanErrAndStd(:,4),'-or');legend('init tracking', 'new p2 exp');
for i =  1 : size(obj.meanErrAndStd, 1)

text(double(obj.meanErrAndStd(i,1)),double(obj.meanErrAndStd(i,2)),num2str(i), 'Color',[0 0 1],'FontSize',15);
text(double(obj.meanErrAndStd(i,3)),double(obj.meanErrAndStd(i,4)),num2str(i), 'Color',[1 0 0],'FontSize',15);
% % if i == 1
% %     legend('init tracking', 'new p2 exp');
% % end
% plot(obj.meanErrAndStd(i,1), obj.meanErrAndStd(i,2),'+b');plot(obj.meanErrAndStd(i,3), obj.meanErrAndStd(i,4),'+r');axis equal; title('tracking - gt');legend('init tracking', 'new p2 exp');
end
title('tracking - gt');
%                                             figure(1),clf;hist([errP2_old errP2_new],100);legend('old','new')
saveas(gcf,fullfile(probPath,sprintf(strcat('refinedTracking_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('refinedTracking_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
