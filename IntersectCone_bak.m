function [sampledDispRngPrvInKey,ProbZTmpReSamp,DispKeyInPrvRng] = IntersectCone_bak(obj, inlierId, inlierIdPrv, imgKeyL, imgPrvL, b2c,DispRng, dispPrvList0, DepthProbility, DepthId,ConfigParam,KeyFrameFlagList,thetaRngMat,thetaProbMat,LocalTrace)
global Rati2 probPath NewPrvDispResamp FrmNumWithGloden FigBase


DepthProbility = DepthProbility(ismember(DepthId, inlierId),:);


DepthProbility = DepthProbility./repmat(sum(DepthProbility')',1,size(DepthProbility,2));

probDir = probPath;
radius = ConfigParam.disparity_error;


DispRng = round(DispRng,4);
DispRngCenter = DispRng(:,(size(DispRng,2)+1)/2);

DispRngStep = mean(diff(DispRng'))';

zSampCnt = size(DispRng,2);

[princpPtL, princpPtR] = Get(obj.camModel, 'PinholePrincpPt', obj.scaleLvl);
L2R = [rodrigues(obj.camModel.rotVec1To2) obj.camModel.transVec1To2; 0 0 0 1];
baseline = norm(obj.camModel.transVec1To2);
intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,obj.scaleLvl);

%% 20191230 useless code
if 0
    newestKeyId = find(KeyFrameFlagList == 1);
    newestKeyId = newestKeyId(2);
end
% % % k2pAng = obj.refAngList(end);


% k2cCam
% visualAngList = obj.angOpt;
prvThetaRng = thetaRngMat(end,:);
prvThetaProb = thetaProbMat(end,:)./sum(thetaProbMat(end,:));

thetaProb1 = prvThetaProb./max(prvThetaProb);


rati = Rati2; 0.95; 0.8;0.5;0.8; 0.5; 0.8; 0.9; 0;0.8;
thetaProb2 = thetaProb1(thetaProb1 > rati);
thetaProb2 = thetaProb2./sum(thetaProb2);

prvThetaRngUse = prvThetaRng(thetaProb1 > rati);   % 0.5 0.8 0.0
prvThetaProbUse = thetaProb2;

angOpt = deg2rad(dot(rad2deg(prvThetaRngUse),prvThetaProbUse));


pixKey = [LocalTrace.ptIcsX(inlierId,1) LocalTrace.ptIcsY(inlierId,1)];
pixPrv = [LocalTrace.ptIcsX(inlierId,end-1) LocalTrace.ptIcsY(inlierId,end-1)];
dispPrvList = dispPrvList0(inlierId,:);
DispRngPrv = dispPrvList + [-ConfigParam.disparity_error : ConfigParam.disparity_sample_step : ConfigParam.disparity_error];
DepthRngPrv = (intrMat(1,1).*norm(obj.camModel.transVec1To2))./(DispRngPrv + (princpPtR(1) - princpPtL(1)));

DepthRngKey = (intrMat(1,1).*norm(obj.camModel.transVec1To2))./(DispRng + (princpPtR(1) - princpPtL(1)));
[XYZPrvRng] = GetXYZFromDepth(intrMat, repmat(pixPrv,size(DispRng,2),1),DepthRngPrv(:));
[XYZKeyRng] = GetXYZFromDepth(intrMat, repmat(pixKey,size(DispRng,2),1),DepthRngKey(:));
ZKeyRng = reshape(XYZKeyRng(:,3),size(DispRng));
ZKeyBound = [ZKeyRng(:,end) ZKeyRng(:,(size(ZKeyRng,2)+1)/2) ZKeyRng(:,1)];



metricPrevPtCcs = intrMat\HomoCoord(pixPrv',1);
% metricPrevPtCcs = normc(metricPrevPtCcs);


for i = 1 : 1% length(prvThetaProbUse)
    if 0
        k2pBody = [roty(double(rad2deg(prvThetaRngUse(i)))) [0 0 0]';0 0 0 1];
    else
        %         k2pBody = [roty(double(rad2deg(obj.angOpt(end)))) [0 0 0]';0 0 0 1];
        k2pBody = [roty(double(rad2deg(angOpt))) [0 0 0]';0 0 0 1];
        
    end
    k2pCam = b2c * k2pBody / b2c;
    p2kCam = inv(k2pCam);
    
    XYZPrv2KeyRng = [p2kCam(1:3,1:3)*XYZPrvRng' + repmat(p2kCam(1:3,4),1,size(XYZPrvRng,1))]';
    
    XPrv2KeyRng = reshape(XYZPrv2KeyRng(:,1),size(DispRng));
    YPrv2KeyRng = reshape(XYZPrv2KeyRng(:,2),size(DispRng));
    ZPrv2KeyRng = reshape(XYZPrv2KeyRng(:,3),size(DispRng));
    
    ZPrv2KeyBound = [ZPrv2KeyRng(:,end) ZPrv2KeyRng(:,(size(ZPrv2KeyRng,2)+1)/2) ZPrv2KeyRng(:,1)];
    
    [angZ, angX] = TransVec2Pol(p2kCam(1:3,4));
    
    [reprjErrInPrv, reprjErrInKey,eplInPrv,eplInKey, fundMat] = ReprojectError2(pixPrv, pixKey, intrMat, intrMat, rodrigues(p2kCam(1:3,1:3)), real([angZ; angX]), 'extrinsic');
    if 0
        [angZ_, angX_] = TransVec2Pol(k2pCam(1:3,4));
        [reprjErrInKey_, reprjErrInPrv_,eplInKey_,eplInPrv_, fundMat_] = ReprojectError2(pixKey, pixPrv, intrMat, intrMat, rodrigues(k2pCam(1:3,1:3)), [angZ_; angX_], 'extrinsic');
    end
    
    [ptKey1, ptKey2] = IntersectLineCircle(pixKey,eplInKey,radius);
    ptKey1_norm = intrMat\HomoCoord(ptKey1',1);
    ptKey2_norm = intrMat\HomoCoord(ptKey2',1);
    
    %     X1 = intsec2views_midpoint([eye(3) [0;0;0]],k2pCam(1:3,:),ptKey1_norm,metricPrevPtCcs);
    % %     X2 =intsec2views([eye(3) [0;0;0]],k2pCam(1:3,:),ptKey1_norm,metricPrevPtCcs);
    %     projKey = pflat(intrMat*X1(1:3,:));
    %     projPrv = pflat(intrMat*(k2pCam(1:3,1:3)*X1(1:3,:) + repmat(k2pCam(1:3,4),1,size(X1,2))));
    %     [~,errKey] = NormalizeVector(ptKey1 - projKey(1:2,:)');
    %     [~,errPrv] = NormalizeVector(pixPrv - projPrv(1:2,:)');
    
    PrvCamOrigInKeyCam = p2kCam(1:3,4)';
    
    [X1, errKey1,errPrv1] = GetBoundaryDepth(intrMat, k2pCam,ptKey1_norm,metricPrevPtCcs, ptKey1, pixPrv);
    [X2, errKey2,errPrv2] = GetBoundaryDepth(intrMat, k2pCam,ptKey2_norm,metricPrevPtCcs, ptKey2, pixPrv);
    
    PrvLineDir1 = X1 - repmat(PrvCamOrigInKeyCam,size(X1,1),1);
    PrvLineDir2 = X2 - repmat(PrvCamOrigInKeyCam,size(X2,1),1);
    
    [PrvLineDir1_norm, ~] = NormalizeVector(PrvLineDir1);
    [PrvLineDir2_norm, ~] = NormalizeVector(PrvLineDir2);
    errLineDir12 = PrvLineDir1_norm - PrvLineDir2_norm;
    
    [XYZPrvInKeyRng] = GetXYZFromDepth(intrMat, repmat(pixKey,size(DispRng,2),1),ZPrv2KeyRng(:));
    XPrvInKeyRng = reshape(XYZPrvInKeyRng(:,1), size(DispRng));
    YPrvInKeyRng = reshape(XYZPrvInKeyRng(:,2), size(DispRng));
    ZPrvInKeyRng = reshape(XYZPrvInKeyRng(:,3), size(DispRng));
    DispPrvInKeyRng = (intrMat(1,1).*baseline./ZPrvInKeyRng) - (princpPtR(1) - princpPtL(1));
    DispPrvInKeyRng = round(DispPrvInKeyRng,4);
    if size(LocalTrace.ptIcsX, 2) > FrmNumWithGloden + 1   % NewPrvDispResamp
        sampledDispPrvInKey = [max([DispPrvInKeyRng(:,1) DispRng(:,1)]')' min([DispPrvInKeyRng(:,end) DispRng(:,end)]')'];
    else
        sampledDispPrvInKey = [DispRng(:,1) DispRng(:,2)];
    end
    newDispStep = (sampledDispPrvInKey(:,2) - sampledDispPrvInKey(:,1))./(size(DispRng,2)-1);
    SampMat = repmat(-(size(DispRng,2)-1)/2 : (size(DispRng,2)-1)/2,size(DispRng,1),1);
    SampMatVal = SampMat.*repmat(newDispStep,1,size(DispRng,2));
    centerDisp = mean(sampledDispPrvInKey')';
    sampledDispRngPrvInKey = repmat(centerDisp,1,size(DispRng,2)) + SampMatVal;
    
    sampledDispRngPrvInKey = round(sampledDispRngPrvInKey,4);
    
    sampledZRngPrvInKey = (intrMat(1,1).*norm(obj.camModel.transVec1To2))./(sampledDispRngPrvInKey + (princpPtR(1) - princpPtL(1)));
    % %     [XYZPrvRng] = GetXYZFromDepth(intrMat, repmat(pixPr,size(DispRng,2),1),sampledZRngPrvInKey(:));
    %     [XYZPrvRng] = GetXYZFromDepth(intrMat, repmat(pixKey,size(DispRng,2),1),DepthRngPrv(:));
    if 0
        scaleMat = (sampledZRngPrvInKey - PrvCamOrigInKeyCam(3))./(ZPrvInKeyRng - PrvCamOrigInKeyCam(3));
        sampledXRngPrvInKey = (XPrvInKeyRng - PrvCamOrigInKeyCam(1)).*scaleMat + PrvCamOrigInKeyCam(1);
        sampledYRngPrvInKey = (YPrvInKeyRng - PrvCamOrigInKeyCam(2)).*scaleMat + PrvCamOrigInKeyCam(2);
    else
        scaleMat = (sampledZRngPrvInKey - PrvCamOrigInKeyCam(3))./(ZPrv2KeyRng - PrvCamOrigInKeyCam(3));
        sampledXRngPrvInKey = (XPrv2KeyRng - PrvCamOrigInKeyCam(1)).*scaleMat + PrvCamOrigInKeyCam(1);
        sampledYRngPrvInKey = (YPrv2KeyRng - PrvCamOrigInKeyCam(2)).*scaleMat + PrvCamOrigInKeyCam(2);
    end
    [sampledXYZRngPrvInKey] = [sampledXRngPrvInKey(:) sampledYRngPrvInKey(:) sampledZRngPrvInKey(:)];
    PrvLineDir11 = sampledXYZRngPrvInKey - repmat(PrvCamOrigInKeyCam,size(scaleMat,1)*size(scaleMat,2),1);
    if 0
        PrvLineDir22 = XYZPrvInKeyRng - repmat(PrvCamOrigInKeyCam,size(scaleMat,1)*size(scaleMat,2),1);
    else
        PrvLineDir22 = XYZPrv2KeyRng - repmat(PrvCamOrigInKeyCam,size(scaleMat,1)*size(scaleMat,2),1);
    end
    [PrvLineDir11_norm, ~] = NormalizeVector(PrvLineDir11);
    [PrvLineDir22_norm, ~] = NormalizeVector(PrvLineDir22);
    errLineDir1122 = PrvLineDir11_norm - PrvLineDir22_norm;
    XYZKeyInPrvRng = [k2pCam(1:3,1:3)*sampledXYZRngPrvInKey' + repmat(k2pCam(1:3,4),1,size(sampledXYZRngPrvInKey,1))]';
    XYZKeyInPrvRngProj = pflat(intrMat*XYZKeyInPrvRng')';
    XYZKeyInPrvRngProjErr = XYZKeyInPrvRngProj(:,1:2) - repmat(pixPrv, size(DispRng,2),1);
    pixKeyProj = pflat(intrMat*sampledXYZRngPrvInKey')';
    pixKeyProj = pixKeyProj(:,1:2);
    pixKeyProj0 = pflat(intrMat*XYZPrv2KeyRng')';
    pixKeyProj0 = pixKeyProj0(:,1:2);
    
    if 0
        figure,plot(XYZKeyInPrvRngProjErr(:,1),XYZKeyInPrvRngProjErr(:,2),'.');axis equal;
        figure,imshow(zeros(240,320));hold on;plot(pixKeyProj(:,1),pixKeyProj(:,2),'.r');plot(pixKeyProj0(:,1),pixKeyProj0(:,2),'.g');plot(pixKeyProj(:,1),pixKeyProj(:,2),'.r');
        figure,plot([sampledDispRngPrvInKey(:,end) - DispRng(:,end)])
        figure,plot([sampledDispRngPrvInKey(:,1) - DispRng(:,1)])
    end
    ZKeyInPrvRng = reshape(XYZKeyInPrvRng(:,3),size(DispRng));
    DispKeyInPrvRng = (intrMat(1,1).*baseline./ZKeyInPrvRng) - (princpPtR(1) - princpPtL(1));
    
    ProbZTmpReSamp = [];
    for tp = 1 : size(DispRng,1)
        %                 ProbZTmpNormReSamp(tp,:) = interp1(DispRng(tp,:), ProbZTmpNorm(tp,:), NewDispRng(tp,:));
        ProbZTmpReSamp(tp,:) = interp1(DispRng(tp,:), DepthProbility(tp,:), sampledDispRngPrvInKey(tp,:));
    end
    ProbZTmpReSamp = ProbZTmpReSamp./repmat(max(ProbZTmpReSamp')',1,size(ProbZTmpReSamp,2));
    
    deltaDisp1 = DispRng(:,1) - sampledDispRngPrvInKey(:,1);
    deltaDisp2 = DispRng(:,end) - sampledDispRngPrvInKey(:,end);
    
    areaRatio = nan(length(deltaDisp1),1);
    StartSame = find(deltaDisp1 ==0 & deltaDisp2 ~=0 );
    EndSame = find(deltaDisp2 ==0 & deltaDisp1 ~=0 );
    StartEndSame = find(deltaDisp1 == 0 & deltaDisp2 == 0 );
    StartEndDiff = find(deltaDisp1 ~= 0 & deltaDisp2 ~= 0 );
    
    if 1
        depthGTIndStart = RoundingDispErr(DispRngCenter,sampledDispRngPrvInKey(:,1), DispRngStep,DispRng);
        depthGTIndEnd = RoundingDispErr(DispRngCenter,sampledDispRngPrvInKey(:,end), DispRngStep,DispRng);
        
        DepthProbilityNew = zeros(size(DepthProbility));
        for k = 1 : size(DispRng,1)
            DepthProbilityNew(k,depthGTIndStart(k) : depthGTIndEnd(k)) = DepthProbility(k,depthGTIndStart(k) : depthGTIndEnd(k));
        end
        
        
        sadbwgv= 1;
        
        
    else
        if ~isempty(StartSame)
            %         sampledDispRngPrvInKey(StartSame,end)
            round(deltaDisp2./DispRngStep)
            
            disparityError = DispRngCenter - sampledDispRngPrvInKey(:,end);
            
            disparityErrorRound = round(disparityError./DispRngStep).*DispRngStep;
            
            
            depthGTOfst11 = round(-(disparityErrorRound)./DispRngStep);
            depthGTInd11 = depthGTOfst11 + (size(DispRng,2)+1)/2;
            depthGTInd11(depthGTInd11 < 1) = 1;
            depthGTInd11(depthGTInd11 > size(DispRng,2)) = size(DispRng,2);
            
            
            
            
        end
        
        if ~isempty(EndSame)
            
            
        end
        
        if ~isempty(StartEndSame)
            
            
        end
        
        if ~isempty(StartEndDiff)
            
            
        end
    end
    [DispRng DispPrvInKeyRng];
    
    lowerConeZ = min([X1(:,3) X2(:,3)]')';
    upperConeZ = max([X1(:,3) X2(:,3)]')';
    newZPrv2KeyBound = [max([lowerConeZ ZPrv2KeyBound(:,1)]')'  min([upperConeZ ZPrv2KeyBound(:,3)]')'];
    
    if 1
        %         zPrv2KeyNewLower = ChoosePrvXYZ(LowerZ,UpperZ, newZPrv2KeyBound,ZPrv2KeyBound,XPrv2KeyRng,YPrv2KeyRng,ZPrv2KeyRng, X1, X2, bound);
        [xPrv2KeyNewLower11, yPrv2KeyNewLower11, zPrv2KeyNewLower11, Prv2KeyBelong2ConeLower11] = ChoosePrvXYZ(lowerConeZ,upperConeZ, newZPrv2KeyBound,ZPrv2KeyBound,XPrv2KeyRng,YPrv2KeyRng,ZPrv2KeyRng, X1, X2, 1);
        [xPrv2KeyNewUpper11, yPrv2KeyNewUpper11, zPrv2KeyNewUpper11, Prv2KeyBelong2ConeUpper11] = ChoosePrvXYZ(lowerConeZ,upperConeZ, newZPrv2KeyBound,ZPrv2KeyBound,XPrv2KeyRng,YPrv2KeyRng,ZPrv2KeyRng, X1, X2, 2);
    else
        
        Prv2KeyBelong2ConeLower = find(ismember(lowerZ,newZPrv2KeyBound(:,1)));
        
        xPrv2KeyNewLower = nan(size(ZPrv2KeyRng,1),2); yPrv2KeyNewLower = nan(size(ZPrv2KeyRng,1),2); zPrv2KeyNewLower = nan(size(ZPrv2KeyRng,1),2);
        if isempty(Prv2KeyBelong2ConeLower)
            Prv2KeyBelong2ConfigLower = [1:size(ZPrv2KeyBound,1)]';
            xPrv2KeyNewLower(:,1) = XPrv2KeyRng(:,1); yPrv2KeyNewLower(:,1) = YPrv2KeyRng(:,1); zPrv2KeyNewLower(:,1) = ZPrv2KeyRng(:,1);
        else
            
            
            chosenConeZ = lowerZ(Prv2KeyBelong2ConeLower,1);
            zPrv2KeyNewLower(Prv2KeyBelong2ConeLower,1) = chosenConeZ;
            
            pt1Id = find(ismember(X1(:,3),chosenConeZ));
            pt2Id = find(ismember(X2(:,3),chosenConeZ));
            chosenConeX = X1(pt1Id,1); chosenConeY = X1(pt1Id,2);
            chosenConeX = X2(pt2Id,1); chosenConeY = X2(pt2Id,2);
            xPrv2KeyNewLower(Prv2KeyBelong2ConeLower,1) = chosenConeX;
            yPrv2KeyNewLower(Prv2KeyBelong2ConeLower,1) = chosenConeY;
            
            if length(Prv2KeyBelong2ConeLower) < size(ZPrv2KeyRng,1)
                
                leftover = setdiff([1:size(ZPrv2KeyRng,1)]', Prv2KeyBelong2ConeLower);
                xPrv2KeyNewLower(leftover,1) = XPrv2KeyRng(leftover,1);
                yPrv2KeyNewLower(leftover,1) = YPrv2KeyRng(leftover,1);
                zPrv2KeyNewLower(leftover,1) = ZPrv2KeyRng(leftover,1);
                
                Prv2KeyBelong2ConfigLower = find(ismember(ZPrv2KeyBound(:,1),newZPrv2KeyBound(:,1)));
                
                
                
                
            end
        end
    end
    
    NewPrv2KeyLower = [xPrv2KeyNewLower11(:,1) yPrv2KeyNewLower11(:,1) zPrv2KeyNewLower11(:,1)];
    NewPrv2KeyUpper = [xPrv2KeyNewUpper11(:,2) yPrv2KeyNewUpper11(:,2) zPrv2KeyNewUpper11(:,2)];
    %     scaleNewPrv2KeyLower = newZPrv2KeyBound(:,1)./ZPrv2KeyBound(:,1);
    %     scaleNewPrv2KeyLower = newZPrv2KeyBound(:,end)./ZPrv2KeyBound(:,end);
    
    [projPrvLower, errPrvLower,dispPrvUpper] = CheckChosenRngProj(NewPrv2KeyLower, k2pCam, intrMat, pixPrv ,baseline,princpPtL ,princpPtR);
    [projPrvUpper, errPrvUpper,dispPrvLower] = CheckChosenRngProj(NewPrv2KeyUpper, k2pCam, intrMat, pixPrv ,baseline,princpPtL ,princpPtR);
    
    %     figure(FigBase + 3),clf; [radi, ptOnLineErr1, ptOnLineErr2] = ShowEpilineCircle(pixKey,pixPrv, fundMat, eplInKey, imgKeyL, [ptKey1 ptKey2]);
    %     figure(FigBase + 3),clf; hist((dispPrvUpper - dispPrvLower)./(DispRng(:,end) - DispRng(:,1)),100)
    %     figure(FigBase + 3),clf; hist((dispPrvUpper - dispPrvLower)./(DispRng(:,end) - DispRng(:,1)),100)
    %     figure(FigBase + 3),clf; hist((sampledDispPrvInKey(:,end) - sampledDispPrvInKey(:,1))./(DispRng(:,end) - DispRng(:,1)),100);
    
    
    if 0
        figure(FigBase + 3),clf; hist(sum(DepthProbilityNew'),100)
        %     sampledDispPrvInKey
        saveas(gcf,fullfile(probPath,sprintf(strcat('cone_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('cone_',probDir(end-14:end),'____',num2str(sum(KeyFrameFlagList)),'__*.png'))))+1 + FigBase)));
    end
    
end

end
function [radi, ptOnLineErr1, ptOnLineErr2] = ShowEpilineCircle(ptKey,ptPrv, fundMatP2K, eplInKey, imgKey, circlePt)
randId = randperm(size(ptKey,1));
showNum = 20;
randId = randId(1:showNum);
eplInKey_check = epipolarLine(fundMatP2K,ptPrv(randId,:));

[~, scale] = NormalizeVector(eplInKey(:,1:2));
eplInKey = eplInKey./repmat(scale,1,3);

[~, radi] = NormalizeVector(repmat(ptKey,2,1) - real([circlePt(:,1:2);circlePt(:,3:4)]));

ptOnLineErr1 = dot(eplInKey', real([circlePt(:,1:2) ones(size(circlePt,1),1)])');
ptOnLineErr2 = dot(eplInKey', real([circlePt(:,3:4) ones(size(circlePt,1),1)])');

points = lineToBorderPoints(eplInKey(randId,:), size(imgKey));
imshow(imgKey);hold on;line(points(:, [1,3])', points(:, [2,4])'); title('Left');
plot(ptKey(randId,1),ptKey(randId,2),'.r');
plot(real(circlePt(randId,1)),real(circlePt(randId,2)),'.g');
plot(real(circlePt(randId,3)),real(circlePt(randId,4)),'.b');
title('key frame');

end

function [X, errKey,errPrv] = GetBoundaryDepth(intrMat, k2pCam,ptKey1_norm,metricPrevPtCcs, ptKey1, pixPrv)
X1 = intsec2views_midpoint([eye(3) [0;0;0]],k2pCam(1:3,:),ptKey1_norm,metricPrevPtCcs);

projKey = pflat(intrMat*X1(1:3,:));
projPrv = pflat(intrMat*(k2pCam(1:3,1:3)*X1(1:3,:) + repmat(k2pCam(1:3,4),1,size(X1,2))));
[~,errKey] = NormalizeVector(ptKey1 - projKey(1:2,:)');
[~,errPrv] = NormalizeVector(pixPrv - projPrv(1:2,:)');

X = X1(1:3,:)';
end

function [XYZ] = GetXYZFromDepth(intrMat, Pix,depthList)
metricPrevPtCcsGT = intrMat\HomoCoord(Pix',1);
metricPrevPtCcsGT = normc(metricPrevPtCcsGT);
if 1
    scaleAllGT = depthList./metricPrevPtCcsGT(3,:)';
else
    dispGTComp = dispList(inlierId) - disparityErrorRound;
    depthListGTComp = intrMat(1,1).*norm(obj.camModel.transVec1To2)./(dispGTComp + (princpPtR(1) - princpPtL(1)));
    scaleAllGT = depthListGTComp./metricPrevPtCcsGT(3,:)';
end
XYZ = [repmat(scaleAllGT',3,1).*metricPrevPtCcsGT]';




end

function [xPrv2KeyNewLower,yPrv2KeyNewLower,zPrv2KeyNewLower,Prv2KeyBelong2ConeLower] = ChoosePrvXYZ(LowerConeZ,UpperConeZ, newZPrv2KeyBound,ZPrv2KeyBound,XPrv2KeyRng,YPrv2KeyRng,ZPrv2KeyRng, X1, X2, bound)

if bound == 1
    lowerConeZ = LowerConeZ;
else
    lowerConeZ = UpperConeZ;
end
Prv2KeyBelong2ConeLower = find(ismember(lowerConeZ,newZPrv2KeyBound(:,bound)));

xPrv2KeyNewLower = nan(size(ZPrv2KeyRng,1),2); yPrv2KeyNewLower = nan(size(ZPrv2KeyRng,1),2); zPrv2KeyNewLower = nan(size(ZPrv2KeyRng,1),2);
if isempty(Prv2KeyBelong2ConeLower)
    Prv2KeyBelong2ConfigLower = [1:size(ZPrv2KeyBound,1)]';
    if bound == 1
        xPrv2KeyNewLower(:,1) = XPrv2KeyRng(:,1); yPrv2KeyNewLower(:,1) = YPrv2KeyRng(:,1); zPrv2KeyNewLower(:,1) = ZPrv2KeyRng(:,1);
    else
        xPrv2KeyNewLower(:,end) = XPrv2KeyRng(:,end); yPrv2KeyNewLower(:,end) = YPrv2KeyRng(:,end); zPrv2KeyNewLower(:,end) = ZPrv2KeyRng(:,end);
    end
else
    
    
    chosenConeZ = lowerConeZ(Prv2KeyBelong2ConeLower,1);
    zPrv2KeyNewLower(Prv2KeyBelong2ConeLower,bound) = chosenConeZ;
    
    chosenConeX = nan(length(chosenConeZ),1); chosenConeY = nan(length(chosenConeZ),1);
    pt1Id = find(ismember(X1(:,3),chosenConeZ));
    pt2Id = find(ismember(X2(:,3),chosenConeZ));
    
    pt11Id = find(ismember(chosenConeZ,X1(:,3)));
    pt22Id = find(ismember(chosenConeZ,X2(:,3)));
    
    chosenConeX(pt11Id,1) = X1(pt1Id,1); chosenConeY(pt11Id,1) = X1(pt1Id,2);
    chosenConeX(pt22Id,1) = X2(pt2Id,1); chosenConeY(pt22Id,1) = X2(pt2Id,2);
    xPrv2KeyNewLower(Prv2KeyBelong2ConeLower,bound) = chosenConeX;
    yPrv2KeyNewLower(Prv2KeyBelong2ConeLower,bound) = chosenConeY;
    
    if length(Prv2KeyBelong2ConeLower) < size(ZPrv2KeyRng,1)
        
        leftover = setdiff([1:size(ZPrv2KeyRng,1)]', Prv2KeyBelong2ConeLower);
        if bound == 1
            xPrv2KeyNewLower(leftover,bound) = XPrv2KeyRng(leftover,1);
            yPrv2KeyNewLower(leftover,bound) = YPrv2KeyRng(leftover,1);
            zPrv2KeyNewLower(leftover,bound) = ZPrv2KeyRng(leftover,1);
        else
            xPrv2KeyNewLower(leftover,end) = XPrv2KeyRng(leftover,end);
            yPrv2KeyNewLower(leftover,end) = YPrv2KeyRng(leftover,end);
            zPrv2KeyNewLower(leftover,end) = ZPrv2KeyRng(leftover,end);
            
            
        end
        %             Prv2KeyBelong2ConfigLower = find(ismember(ZPrv2KeyBound(:,1),newZPrv2KeyBound(:,1)));
        
        
        
        
    end
end


end

function [projPrv_, errPrv, disparityG] = CheckChosenRngProj(NewPrv2KeyLower, k2pCam, intrMat, pixPrv,baseline,princpPtL , princpPtR)
prvXYZ = (k2pCam(1:3,1:3)*NewPrv2KeyLower' + repmat(k2pCam(1:3,4),1,size(NewPrv2KeyLower,1)));
projPrv = pflat(intrMat*prvXYZ);
[~,errPrv] = NormalizeVector(pixPrv - projPrv(1:2,:)');
projPrv_ = projPrv(1:2,:)';
disparityG = (intrMat(1,1).*baseline./prvXYZ(3,:)') - (princpPtR(1) - princpPtL(1));


end

function depthGTInd11 = RoundingDispErr(DispRngCenter,disp2round, DispRngStep,DispRng)
disparityError = DispRngCenter - disp2round;

disparityErrorRound = round(disparityError./DispRngStep).*DispRngStep;


depthGTOfst11 = round(-(disparityErrorRound)./DispRngStep);
depthGTInd11 = depthGTOfst11 + (size(DispRng,2)+1)/2;
depthGTInd11(depthGTInd11 < 1) = 1;
depthGTInd11(depthGTInd11 > size(DispRng,2)) = size(DispRng,2);
end