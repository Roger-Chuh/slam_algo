function dispErrExpStack2 = CalcGoldenTracking(obj, intrMat, dispErrExpStack, newCalc)
global probPath




% stack cam in world pose: Tc2w
% stack world in body pose: Tw2b
% [b2c_solve, err] = TSAIleastSquareCalibration(Tc2w, Tw2b);



draw = 0;


% % % % % focalLength = [309.4362 344.2161];
% % % % % principalPoint = [318.9034 257.5352];
% % % % % imageSize = [480 640];
% % % % % camIntrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);
% % % % % height = 2.1798;
% % % % % pitch = 14;
% % % % % sensor = monoCamera(camIntrinsics,height,'Pitch',pitch);
% % % % % distAheadOfSensor = 30;
% % % % % spaceToOneSide = 6;
% % % % % bottomOffset = 3;
% % % % % outView = [bottomOffset,distAheadOfSensor,-spaceToOneSide,spaceToOneSide];
% % % % % outImageSize = [NaN,250];
% % % % % birdsEye = birdsEyeView(sensor,outView,outImageSize);
% % % % % I = imread('road.png');
% % % % % B = transformImage(birdsEye,I);



noRef = false;
replaceRT = false;


% newCalc = true; false; true; false;
try
    b2cPmat = GetPctBody2Cam(obj.coordSysAligner, 1);
catch
%     b2cPmat.transformMat = [eye(3) [10;45;-170.2];0 0 0 1];
    load('E:\bk_20180627\SLAM\slam_algo\prob\20200417_204538====03_00_R_CCW_640x480_00====0.5_2_2_20____20200417_204538\b2c.mat');
    b2cPmat = k2cBodyPmat;
    b2cPmat.transformMat = [eye(3) [10;45;-170.2];0 0 0 1];
end
Rb2c = b2cPmat.transformMat(1:3,1:3);
% % % b2cPmat.transformMat(3,4) = -170.35;

b2cVec = [rodrigues( b2cPmat.transformMat(1:3,1:3));b2cPmat.transformMat(1:3,4)];
frameId00 = cell2mat(dispErrExpStack(:,1));
dispErrExpStack(setdiff(1:size(dispErrExpStack,1), frameId00),:) = [];

frameId11 = cell2mat(dispErrExpStack(:,9));
dispErrExpStack2 = dispErrExpStack(find(frameId11 > 0),:);

% trackingErrDistribution = {};


for ui = 1 : 30 % size(obj.colorMat,1)
    trackingErrDistribution{ui,1} = [];
end



inlierTrace0 = [[3;5;10;20;25;38;40;41;48;49;51;52;53;59;64;67;73;90;91;93;94;97;99;100;103;104;106;111;112;116;117;118;119;120;121;122;129;130;135;141;144;145;146;147;149;150;153;154;157;158;159;160;164;172;173;174;175;179;182;185;186;201;202;203;206]];
inlierThr = 0.3;
featInd = 3;
FeatStack = {};
FeatStackX = {};FeatStackY = {};FeatStack2 = {}; FeatStackX_differ = {};FeatStackX_differ_LK = {}; P2CTraceErr = {};
% trackingErrDistribution{size(KeyProbZ,1),1} = [obj.trackingErrDistribution{size(KeyProbZ,1),1}; [{trackingErr12345} {traceBatch} {LocalTrace.featId}] ];
for i = 1 : size(dispErrExpStack2,1)
    
    
    
      tempDisps = dispErrExpStack2{i, 3};
        tempLocalTrace = dispErrExpStack2{i, 7};
        
        
        
        
        frameRng = [dispErrExpStack2{i, 2} - dispErrExpStack2{i, 4} + 1 : dispErrExpStack2{i, 2} - dispErrExpStack2{i, 4} + 0 + size(tempDisps,1)];
        tempAngOpt = [abs(tempLocalTrace.angOpt_old) abs(tempLocalTrace.ANGOptFinal)  abs(tempLocalTrace.gtTheta) abs(tempLocalTrace.localTrace_gtAng2)];
        
        try
            gtAngList = obj.accumP2CRef(frameRng);
            gtAngList = gtAngList - gtAngList(1);
        catch
            sgkj = 1;
            noRef = true;
        end
       
    
%    tempLocalTrace = dispErrExpStack2{i, 7};
   
   
   traceBatch = dispErrExpStack2{i,1};
   if noRef
       gtTheta = tempLocalTrace.localTrace_gtAng2;
   else
       gtTheta = gtAngList(2:end);
   end
   gtTheta_gtZ = tempLocalTrace.gtTheta;
   
   validId = tempLocalTrace.cfg_gt.Valid;
   AA1 = cell2mat(tempLocalTrace.cfg_new.bbbOpt);
   BB1 = cell2mat(tempLocalTrace.cfg_gt.bbbOpt);
   
   if 1
       gtTheta_gtZ2 = mean(BB1(:,validId),2);
   else
       if 1
           gtTheta_gtZ2 = mean(BB1,2);
       else
           gtTheta_gtZ2 = BB1(:,featInd);
       end
   end
   
   pix = [tempLocalTrace.LocalTrace.ptIcsX(:,1) tempLocalTrace.LocalTrace.ptIcsY(:,1)];
   zGT = [tempLocalTrace.LocalTrace.ptCcsZGT(:,1)];
   
   
   [XYZKey] = GetXYZFromDepth(intrMat, pix, zGT);
   
   
   tempLocalTrace.LocalTrace.ptIcsXerr = [];
   tempLocalTrace.LocalTrace.ptIcsYerr = [];
   featStack = []; featStackX = []; featStackY = [];featStack2 = []; featStackX_differ = []; pixGT20_X = []; pixLK_X = []; featStackX_differ_LK = [];
   p2cTraceErr = [];
%    gtTheta_gtZ2 = [];
   reprojErr = [];
   for j = 1 : length(gtTheta)
       gtThetaUse0 = gtTheta(j);
       ptCur = [ tempLocalTrace.LocalTrace.ptIcsX(:,j+1)  tempLocalTrace.LocalTrace.ptIcsY(:,j+1)];
       if 0
           
           options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','off','MaxFunEvals',10000,'TolX',1e-8);%,'MaxFunEvals',10000);%, 'MaxIterations',5);
           [vec,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) CostFunc(b2cVec, X, double(pix), zGT, intrMat, double(ptCur)),[double(gtTheta_gtZ(j))],[],[],options);%,data_1,obs_1)
           gtTheta_gtZ2(j,1) = vec;
       end
       if 0
           CostFunc(b2cVec, gtTheta_gtZ(j), double(pix), zGT, intrMat, double(ptCur));
       end
       
       if j == 1
           gtThetaUse = gtTheta(j);
           pixGTp = pix;
           pixGTp_stack_x = pix;
           [pixGT2_stack_cur_x] = VisualLocalizer.GetGtTrace2(b2cPmat, gtThetaUse0, [pix],zGT,intrMat);
       else
           gtThetaUse = gtTheta_gtZ2(j - 1) + (gtTheta(j) - gtTheta(j-1));
           pixGTp = VisualLocalizer.GetGtTrace2(b2cPmat, gtTheta_gtZ2(j - 1), [pix],zGT,intrMat);
           
           gtTheta_gtZ2_stack = BB1(j-1,:);
           gtTheta_gtZ2_stack_cur = BB1(j-1,:) + (gtTheta(j) - gtTheta(j-1));
           pixGTp_stack_x = []; pixGT2_stack_cur_x = [];
%            for hj = 1 : length(gtTheta_gtZ2_stack)
%                pixGTp_stack = VisualLocalizer.GetGtTrace2(b2cPmat, gtTheta_gtZ2_stack(hj), [pix(hj,:)],zGT(hj),intrMat);
%                pixGTp_stack_x(hj,:) = pixGTp_stack(:,1:2);
%                pixGT2_stack_cur = VisualLocalizer.GetGtTrace2(b2cPmat, gtTheta_gtZ2_stack_cur(hj), [pix(hj,:)],zGT(hj),intrMat);
%                pixGT2_stack_cur_x(hj,:) = pixGT2_stack_cur;
%            end
       end
       if ~newCalc
           gtThetaUse = gtTheta(j);
       end
       [pixGT20] = VisualLocalizer.GetGtTrace2(b2cPmat, gtThetaUse0, [pix],zGT,intrMat);
       [pixGT2] = VisualLocalizer.GetGtTrace2(b2cPmat, gtThetaUse, [pix],zGT,intrMat);
       
       [~, tracking_error] = NormalizeVector(pixGT20 - ptCur);
       inlierTrace = find(tracking_error < inlierThr);
       inlierTrace = inlierTrace0;
       try 
           [rt, idxOutliers] = posest(double(ptCur(inlierTrace,:)), double(XYZKey(inlierTrace,:)), 0.9, intrMat, 'repr_err');
           if ~replaceRT
              RT = [rodrigues(rt(1:3)) rt(4:6); 0 0 0 1];
           else
               RT0 = [rodrigues(rt(1:3)) rt(4:6); 0 0 0 1];
               RT = [Rb2c*roty(rad2deg(double(gtThetaUse0)))*Rb2c' RT0(1:3,4); 0 0 0 1];
           end
           ptIcs = TransformAndProject(XYZKey, intrMat, RT(1:3,1:3), RT(1:3,4));
           reprojErr = [reprojErr; mean((ptIcs(inlierTrace,:) - ptCur(inlierTrace,:)), 1)];
           if 0
               figure,plot(ptIcs(inlierTrace,1) - ptCur(inlierTrace,1), ptIcs(inlierTrace,2) - ptCur(inlierTrace,2), '+r');axis equal
               figure,plot(pixGT20(inlierTrace,1) - ptCur(inlierTrace,1), pixGT20(inlierTrace,2) - ptCur(inlierTrace,2), '+r');axis equal
           end
       catch
           sghk = 1;
       end
       vberr = rad2deg((norm(rt(1:3))) - gtThetaUse0);
       vbErr(i,j) = vberr;
       if 1
           camPoseC2K{i,1}(:,:,j) = inv([rodrigues(rt(1:3)) rt(4:6);0 0 0 1]);
           bodyPoseK2C{i,1}(:,:,j) = [roty(rad2deg(double(gtThetaUse0))) [0;0;0];0 0 0 1];
       else
           camPoseC2K{i,1}(:,:,j) = inv([rodrigues(rt(1:3)) zeros(3,1);0 0 0 1]);
           bodyPoseK2C{i,1}(:,:,j) = [roty(rad2deg(double(gtThetaUse0))) [0;0;0];0 0 0 1];
       end
       if 0 % j > 1
           figure(7),clf;hist(pixGT2(:,1) - pixGT20(:,1), 100); title(sprintf('(v+gt - gt) = %0.5f deg',(rad2deg(gtTheta_gtZ2(j - 1) + (gtTheta(j) - gtTheta(j-1))-gtTheta(j)))));drawnow;
       end
       if j == 1
           tempLocalTrace.LocalTrace.ptIcsXerr(:,1) = zeros(size(pixGT2,1),1);
           tempLocalTrace.LocalTrace.ptIcsYerr(:,1) = zeros(size(pixGT2,1),1);
           pixGT20_X = pix(:,1);
           pixLK_X = pix(:,1);
       end
       tempLocalTrace.LocalTrace.ptIcsXerr = [tempLocalTrace.LocalTrace.ptIcsXerr tempLocalTrace.LocalTrace.ptIcsX(:,j+1) - pixGT2(:,1)];
       tempLocalTrace.LocalTrace.ptIcsYerr = [tempLocalTrace.LocalTrace.ptIcsYerr tempLocalTrace.LocalTrace.ptIcsY(:,j+1) - pixGT2(:,2)];
       
       
       
       
       trackingErrDistribution{j,1} = [trackingErrDistribution{j,1}; [{[tempLocalTrace.LocalTrace.ptIcsXerr(inlierTrace,end) tempLocalTrace.LocalTrace.ptIcsYerr(inlierTrace,end)]} {traceBatch} {tempLocalTrace.LocalTrace.featId(inlierTrace)} {[gtTheta_gtZ(j) - gtTheta(j) gtTheta_gtZ2(j) - gtTheta(j)]}] ];
       featStack = [featStack;[tempLocalTrace.LocalTrace.ptIcsX(featInd,j+1) tempLocalTrace.LocalTrace.ptIcsY(featInd,j+1) pixGT2(featInd,:) pixGT20(featInd,:) pixGTp(featInd,:)]];
       
        
       pixGT20_X = [pixGT20_X pixGT20(:,1)];
       pixLK_X = [pixLK_X tempLocalTrace.LocalTrace.ptIcsX(:,j+1)];
       featStack2 = [featStack2 pixGT2(:,1) - pixGT20(:,1)];
       
       p2cGT = pixGT2(:,1) - pixGTp(:,1);
       p2cLK = tempLocalTrace.LocalTrace.ptIcsX(:,j+1) - tempLocalTrace.LocalTrace.ptIcsX(:,j);
       if 1
           p2cTraceErr = [p2cTraceErr (p2cLK - p2cGT)];
       else
           p2cTraceErr = [p2cTraceErr (tempLocalTrace.LocalTrace.ptIcsX(:,j) - pixGTp(:,1))];
       end
       if 1
           featStackX_differ = [featStackX_differ pixGT2(:,1) - pixGTp(:,1)];
           featStackX_differ_LK = [featStackX_differ_LK tempLocalTrace.LocalTrace.ptIcsX(:,j+1) - pixGTp(:,1)];
       else
           featStackX_differ = [featStackX_differ pixGT2_stack_cur_x(:,1) - pixGTp_stack_x(:,1)];
           featStackX_differ_LK = [featStackX_differ_LK tempLocalTrace.LocalTrace.ptIcsX(:,j+1) - pixGTp_stack_x(:,1)];
       end
       if j > 1
           featStackX = [featStackX tempLocalTrace.LocalTrace.ptIcsX(:,j) - pixGTp(:,1)];
           featStackY = [featStackY tempLocalTrace.LocalTrace.ptIcsY(:,j) - pixGTp(:,2)];
%            featStack2 = [featStack2 pixGT2(:,1) - pixGT20(:,1)];
       end
% %        FeatStackX_differ_LK = [FeatStackX_differ_LK; {mean(featStackX_differ_LK - featStackX_differ)}];
       if 0
           
           figure,plot(diff(pixGT20_X'))
           figure,plot(diff(pixLK_X'))
           
           figure,plot(featStackX_differ')
           figure,plot(featStackX_differ_LK')
           figure,plot(featStackX_differ_LK' - featStackX_differ')
           
           figure,plot(featStackX_differ_LK(featInd,:)' - featStackX_differ(featInd,:)')
           figure,plot(mean(featStackX_differ_LK - featStackX_differ))
           
           
           figure,hist(featStackX(:), 100);title(num2str(median(featStackX(:))));
           figure,hist(featStackX(featInd,:), 100);title(num2str(mean(featStackX(featInd,:))));
           
           figure,hist(featStack2(:), 100);title(num2str(mean(featStack2(:))));
           figure,imshow(ones(size(obj.currImgL(:,:,1))));hold on;plot(featStack(:,7), featStack(:,8),'dk');plot(featStack(:,1), featStack(:,2),'xr');plot(featStack(:,3), featStack(:,4),'og');plot(featStack(:,5), featStack(:,6),'sb');legend('gtp','lk','p+gt','gt');
       end
   end
   
% %    inlierTrace = find(tracking_error < inlierThr);
   
   
   
   
   
    FeatStackX_differ_LK = [FeatStackX_differ_LK; {mean(featStackX_differ_LK - featStackX_differ)}];
   FeatStack{traceBatch,1} = traceBatch;
   FeatStack{traceBatch,2} = featStack;
   FeatStack{traceBatch,3} = featStack2;
   if 0
       FeatStackX{traceBatch,1} = featStackX;
       FeatStackX{traceBatch,2} = single(mean(featStackX(:)));
   else
       FeatStackX = [FeatStackX; [{featStackX} {single(median(featStackX(:)))}]];
       FeatStack2 = [FeatStack2; [{featStack2} {single(mean(featStack2(:)))}]];
       FeatStackX_differ = [FeatStackX_differ; [{featStackX_differ} {single(mean(featStackX_differ(:)))}]];
       P2CTraceErr = [P2CTraceErr; [{p2cTraceErr} {mean(p2cTraceErr)} {median(p2cTraceErr)}]];
   end
   dispErrExpStack2{i,7} = tempLocalTrace;
   
end

validTrace = find(abs(mean(abs(vbErr)')) < 0.008);


b2c_vec = []; err = [];
for qq = 1 : length(validTrace)
    [b2c_solve, err(:,qq)] = TSAIleastSquareCalibration(camPoseC2K{qq,1}, bodyPoseK2C{qq,1});
    b2c_vec(:,qq) = [rodrigues(b2c_solve(1:3,1:3)); b2c_solve(1:3,4)];
    
end

figure(12),clf;subplot(1,2,1);plot(b2c_vec(1:3,:)');subplot(1,2,2);plot(b2c_vec([4 6],:)');

try
    temp2 = cell2mat(FeatStack2(:,1));
    idnan = ~isnan(sum(temp2,2));
    temp = temp2(idnan,:);
    tp2 = cell2mat(P2CTraceErr(:,2));
catch
    sdgkh = 1;
end
if 0
    figure,hist(cell2mat(FeatStackX(:,2)), 100); title(num2str(median(cell2mat(FeatStackX(:,2)))))
    
    figure,plot(mean(cell2mat(FeatStackX(:,1))));
    
    figure,hist(cell2mat(FeatStack2(:,2)), 100);
    
     figure,plot(mean(tp2)); title(num2str(median(cell2mat(FeatStackX(:,2)))))
    
    
    figure,plot(cell2mat(FeatStackX_differ_LK));
    
    figure,hist(temp, 100);
    figure,plot(mean(diff(temp')'));title('diff(pgt - gt)');
    figure,plot(mean((temp')'));title('(pgt - gt)');
end


thr1 = 0.010;
thr2 = 0.0010;
ErrMat = {};
p2cStack = {};
pixThr = 1; 110.8;

trackingErrStack = [];

if newCalc
    baseFig = 600;
else
    baseFig = 400;
end

for nh = 1 :  size(trackingErrDistribution,1)
    errCell = trackingErrDistribution{nh,1};
    if ~isempty(errCell)
        errMat = [];
        for k = 1 : size(errCell,1)
            errMat(k,:) = [mean(errCell{k,1},1)  rad2deg(errCell{k,4}) errCell{k,2}];
        end
        
        ErrMat = [ErrMat; {errMat}];
        
        
        if nh > 1
            
            for kk = size(ErrMat,1) - 1 : size(ErrMat,1)
                ErrMatTmp = ErrMat{kk,1};
                if kk == size(ErrMat,1) - 1
                    idCom1 = ErrMatTmp(:,5);
                     trackingErr1 = ErrMatTmp(:,1:4);
                    
                end    
                    
                if kk == size(ErrMat,1)
                    
                    idCom2 = ErrMatTmp(:,5);
                    trackingErr2 = ErrMatTmp(:,1:4);
                    
                    idCom12 = intersect(idCom1, idCom2);
                    if 0 %~newCalc
                        trackingErr12 = [trackingErr2(ismember(idCom2,idCom12),:) - trackingErr1(ismember(idCom1,idCom12),:) idCom12];
                    else
                        trackingErr12_tmp = [trackingErr2(ismember(idCom2,idCom12),:) - trackingErr1(ismember(idCom1,idCom12),:) idCom12];
                        trackingErr12 = [trackingErr2(ismember(idCom2,idCom12),1:2) trackingErr12_tmp(:,3:5)];
%                         trackingErr12 = [ [trackingErr2(ismember(idCom2,idCom12),1:2)] [trackingErr2(ismember(idCom2,idCom12),3:4) - trackingErr1(ismember(idCom1,idCom12),3:4) idCom12]];
                    end
                end
            
            end
        end
        
        if exist('trackingErr12', 'var')
            errMat = trackingErr12;
        end
        
        track_err = errMat(:,1:2);
        track_error = track_err(abs(track_err(:,1)) < pixThr,:);
        
        trackingErrStack = [trackingErrStack; mean(track_error(:,1:2),1)];
        
        if 0
            errMat = errMat(1:70,:);
        end
        
        id = find(abs(errMat(:,1)) > thr1 & abs(errMat(:,3)) > thr2 & abs(errMat(:,4)) > thr2);
        id2 = find(~isnan(errMat(:,4)));
        
        if draw
            figure(baseFig+nh),clf;subplot(2,2,[1]);plot(errMat(:,5), errMat(:,[1 3 4]), '-');grid on;legend('LK(x) - gt(x)', 'gt(z) - gt(theta)', 'mean gt(z) - gt(theta)');title(num2str(double(newCalc)));
            subplot(2,2,2);hist(errMat(:,1), 100);title(sprintf('x err: %0.5f', mean(track_error(:,1))));
            subplot(2,2,3);hist(repmat(errMat(id,[1]),1,2)./errMat(id,[3 4]), 100);grid on;title('x err / ang err');
            subplot(2,2,4);plot(cumsum(errMat(id2,[3 4])));legend('exp theta','mean theta');
            title(sprintf('frame %d to %d',nh-1,nh));
            drawnow;
            saveas(gcf,fullfile(probPath,sprintf('final_error_%05d.fig',baseFig+nh)));
        end
    end
end
if ~newCalc
    %     figure(baseFig+nh+1);plot([trackingErrStack cumsum(trackingErrStack)]);legend('p2c tracking err','k2c tracking err');grid on;title(num2str(double(newCalc)));
    figure(baseFig+nh+1);plot([trackingErrStack ]);legend('c2c tracking err');grid on;title(num2str(double(newCalc)));
else
    %     figure(baseFig+nh+1);plot([trackingErrStack cumsum(trackingErrStack)]);legend('p2c tracking err','k2c tracking err');grid on;title(num2str(double(newCalc)));
    if 0
        figure(baseFig+nh+1);plot([trackingErrStack (trackingErrStack + mean((temp')')')]);legend('c2c tracking err');grid on;title(num2str(double(newCalc)));
    else
%         figure(baseFig+nh+1);plot([trackingErrStack ] - median(cell2mat(FeatStackX(:,2))));legend('c2c tracking err');grid on;title(num2str(double(newCalc)));
        
        figure(baseFig+nh+1);plot([trackingErrStack ] - median(tp2)');legend('c2c tracking err');grid on;title(num2str(double(newCalc)));
        
%         figure(baseFig+nh+1);plot([trackingErrStack ]+mean(cell2mat(FeatStackX(:,1)))' );legend('c2c tracking err');grid on;title(num2str(double(newCalc)));
        
        figure(baseFig+nh+1);plot([trackingErrStack ]);legend('c2c tracking err');grid on;title(num2str(double(newCalc)));
    end
end
saveas(gcf,fullfile(probPath,sprintf('final_error_%05d.fig',baseFig+nh+1)));

figure;subplot(1,2,1);plot([trackingErrStack(:,1) ]);title('x');grid on;subplot(1,2,2);plot([trackingErrStack(:,2) ]);title('y');grid on;

% tpp =cell2mat(errCell(:,1));

end
 
function errVar = CostFunc(b2cVec, theta, pix, depthListGT, intrMat,ptCur)
b2cVec(1:3);
if length(b2cVec) > 3
    b2c = [rodrigues(b2cVec(1:3)) b2cVec(4:6);0 0 0 1];
else
    b2c = [rodrigues(b2cVec(1:3)) [10;45;-170.2];0 0 0 1];
end
[pixGT, transXYZ, XYZKey] = GetGtTrace2(b2c, rad2deg(theta), pix,depthListGT,intrMat);




err = (pixGT(:,1) - ptCur(:,1)).^2 + (pixGT(:,2) - ptCur(:,2)).^2;
errVar = median(err); %err(1:round(0.9*size(err,1))) ; %mean(abs(err).^2);
end

function [pixGT, transXYZ, XYZKey] = GetGtTrace2(b2cPmat, k2cRef0, Pix,depthListGT,intrMat)

% if 0
%     k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(thetaRng(idM)),zeros(3,1));
% else
%     k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cRef0),zeros(3,1));
% end
k2cBodyPmat = [roty(double(k2cRef0)) [0;0;0];0 0 0 1];
k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;



metricPrevPtCcsGT = intrMat\HomoCoord(Pix',1);
metricPrevPtCcsGT = normc(metricPrevPtCcsGT);
if 1
    scaleAllGT = depthListGT./metricPrevPtCcsGT(3,:)';
else
    dispGTComp = dispList(inlierId) - disparityErrorRound;
    depthListGTComp = intrMat(1,1).*norm(obj.camModel.transVec1To2)./(dispGTComp + (princpPtR(1) - princpPtL(1)));
    scaleAllGT = depthListGTComp./metricPrevPtCcsGT(3,:)';
end
XYZ = [repmat(scaleAllGT',3,1).*metricPrevPtCcsGT];

XYZKey = XYZ';

k2cT = -b2cPmat(1:3,1:3)*k2cBodyPmat(1:3,1:3)*b2cPmat(1:3,1:3)'*b2cPmat(1:3,4) + b2cPmat(1:3,4);
k2cCam = [k2cCamPmat(1:3,1:3) k2cT;0 0 0 1];
homocurrCcsXYZ = k2cCam*HomoCoord(XYZ,1); %pix0_GT_ = pflat(intrMat*homocurrCcsXYZALL_(1:3,:))';
pixGT = pflat(intrMat*homocurrCcsXYZ(1:3,:))';
pixGT = pixGT(:,1:2);

transXYZ = homocurrCcsXYZ(1:3,:)';

end
function [XYZ] = GetXYZFromDepth(intrMat, Pix,depthList)
metricPrevPtCcsGT = intrMat\HomoCoord(Pix',1);
metricPrevPtCcsGT = normc(metricPrevPtCcsGT);
if 1
    if 0
        if 0
            scaleAllGT = depthListGT(inlierId)./metricPrevPtCcsGT(3,:)';
        else
            scaleAllGT = depthListGT(:)./metricPrevPtCcsGT(3,:)';
        end
    else
        scaleAllGT = depthList./metricPrevPtCcsGT(3,:)';
    end
else
    dispGTComp = dispList(inlierId) - disparityErrorRound;
    depthListGTComp = intrMat(1,1).*norm(obj.camModel.transVec1To2)./(dispGTComp + (princpPtR(1) - princpPtL(1)));
    scaleAllGT = depthListGTComp./metricPrevPtCcsGT(3,:)';
end
XYZ = [repmat(scaleAllGT',3,1).*metricPrevPtCcsGT]';

end