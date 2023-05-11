function [angErr, angErrUpper, angErrLower, angErrP2C, flipWeight, frame_k2, KeyBaseErrGtZ] = CalcAngErr(obj, tempProbNew, validFeatId)
global probPath

if ~exist('frame_k2', 'var')
    frame_k2 = [];
end
if isempty(obj.angOptManager.angOptMat)
    obj.angOptManager.angOptMat = -1.*ones(size(obj.traceManager.X));
    obj.angOptManager.angOptUpperMat = -1.*ones(size(obj.traceManager.X));
    obj.angOptManager.angOptLowerMat = -1.*ones(size(obj.traceManager.X));
else
    obj.angOptManager.angOptMat = [obj.angOptManager.angOptMat -1.*ones(size(obj.angOptManager.angOptMat,1),1)];
    obj.angOptManager.angOptUpperMat = [obj.angOptManager.angOptUpperMat -1.*ones(size(obj.angOptManager.angOptUpperMat,1),1)];
    obj.angOptManager.angOptLowerMat = [obj.angOptManager.angOptLowerMat -1.*ones(size(obj.angOptManager.angOptLowerMat,1),1)];
end


for fg = 1 : size(tempProbNew,1)
    % % %                                             if isempty(obj.angOptManager.angOptMat)
    % % %                                                 obj.angOptManager.angOptMat = zeros(size(obj.traceManager.X));
    % % %                                             else
    % % %                                                 obj.angOptManager.angOptMat = [obj.angOptManager.angOptMat zeros(size(obj.angOptManager.angOptMat,1),1)];
    % % %                                             end
    frmLenTemp = size(tempProbNew{fg,1}.LocalTrace.ptIcsX,2);
    angOptTemp_ = abs([100;tempProbNew{fg, 1}.angOpt]');
    angOptUpperTemp_ = abs([100;tempProbNew{fg, 1}.angRng(:,1)]');
    angOptLowerTemp_ = abs([100;tempProbNew{fg, 1}.angRng(:,2)]');
    
    obj.angOptManager.angOptMat(tempProbNew{fg,1}.LocalTrace.featId,end-frmLenTemp+1:end) = repmat(angOptTemp_, length(tempProbNew{fg,1}.LocalTrace.featId),1);
    obj.angOptManager.angOptUpperMat(tempProbNew{fg,1}.LocalTrace.featId,end-frmLenTemp+1:end) = repmat(angOptUpperTemp_, length(tempProbNew{fg,1}.LocalTrace.featId),1);
    obj.angOptManager.angOptLowerMat(tempProbNew{fg,1}.LocalTrace.featId,end-frmLenTemp+1:end) = repmat(angOptLowerTemp_, length(tempProbNew{fg,1}.LocalTrace.featId),1);
end
obj.validFeatIdStack = [obj.validFeatIdStack; validFeatId];

if 0
    checkManagement = ((obj.angOptManager.angOptMat(obj.validFeatIdStack,:) < 0 - obj.traceManager.X(obj.validFeatIdStack,:) < 0));
    checkManagementUpper = ((obj.angOptManager.angOptUpperMat(obj.validFeatIdStack,:) < 0 - obj.traceManager.X(obj.validFeatIdStack,:) < 0));
    checkManagementLower = ((obj.angOptManager.angOptLowerMat(obj.validFeatIdStack,:) < 0 - obj.traceManager.X(obj.validFeatIdStack,:) < 0));
    %                                         checkManagement = ((obj.angOptManager.angOptMat(obj.validFeatIdStack,:) <= 0 - obj.traceManager.X(obj.validFeatIdStack,:) <= 0));
    %                                         checkManagement = (((obj.angOptManager.angOptMat(obj.validFeatIdStack,:) >= 0) - (obj.traceManager.X(obj.validFeatIdStack,:) >= 0)));
    
    checkManagement = (((obj.angOptManager.angOptMat(obj.validFeatIdStack,:) > 0) - (obj.traceManager.X(obj.validFeatIdStack,:) > 0)));
    checkManagementUpper = (((obj.angOptManager.angOptUpperMat(obj.validFeatIdStack,:) > 0) - (obj.traceManager.X(obj.validFeatIdStack,:) > 0)));
    checkManagementLower = (((obj.angOptManager.angOptLowerMat(obj.validFeatIdStack,:) > 0) - (obj.traceManager.X(obj.validFeatIdStack,:) > 0)));
end

 


if 0
    
    a1 = obj.angOptManager.angOptMat(obj.validFeatIdStack,:);%  >= 0;
    a2 = obj.traceManager.X(obj.validFeatIdStack,:); %  >= 0;
    
    % a1_upper = obj.angOptManager.angOptUpperMat(obj.validFeatIdStack,:);%  >= 0;
    % a2_upper = obj.traceManager.X(obj.validFeatIdStack,:); %  >= 0;
    
    % a1_lower = obj.angOptManager.angOptLowerMat(obj.validFeatIdStack,:);%  >= 0;
    % a2_lower = obj.traceManager.X(obj.validFeatIdStack,:); %  >= 0;
    
    a11 = obj.angOptManager.angOptMat > 0;
    checkManagement = (a1>0) - (a2>0);
    checkManagementSum = sum(double(checkManagement(:)));
    
    % a11_upper = obj.angOptManager.angOptUpperMat > 0;
    % checkManagement_upper = (a1_upper>0) - (a2_upper>0);
    % checkManagementSum_upper = sum(double(checkManagement_upper(:)));
    
    
    
    
    angTrace = sum(a11')';
    angMatSort = immultiply(a11,obj.angOptManager.angOptMat);
    %                                         angTrace1 = sum(obj.angOptManager.angOptMat')';
    angTrace = sum(angMatSort')';
    %                                         [angTraceUnique1,a31,a41] = unique(angTrace1);
    [angTraceUnique,a3,a4] = unique(angTrace);
    
    if 0
        angTrace1 = sum(obj.angOptManager.angOptMat')';
        [angTraceUnique1,a31,a41] = unique(angTrace1);
        angMatSlim1 = obj.angOptManager.angOptMat(a31,:);
    else
        angMatSlim = angMatSort(a3,:);
    end
    
    
    
    startInd = find(angMatSlim == 100);
    [startIndY, startIndX] = ind2sub(size(angMatSlim), startInd);
    [~, sortIdStart2] = sort(startIndX , 'ascend');
    [~, sortIdStart] = sort(startIndY , 'ascend');
    startIndX = startIndX(sortIdStart);
    startIndY = startIndY(sortIdStart);
    endInd = find(angMatSlim ~= 100 & angMatSlim > 0);
    [endIndY, endIndX] = ind2sub(size(angMatSlim), endInd);
    [~, sortIdEnd] = sort(endIndY, 'ascend');
    endIndX = endIndX(sortIdEnd);
    endIndY = endIndY(sortIdEnd);
    coordAngEnd = [];
    for nm = startIndY'
        idtp = find(endIndY == nm);
        [~, idmaxx] = max(endIndX(idtp));
        coordAngEnd = [coordAngEnd; [endIndX(idtp(idmaxx)) nm]];
    end
    coordAngStart = [startIndX startIndY];
    for bn = 1 : size(coordAngStart,1)
        angMatUse(coordAngStart(bn,2), coordAngStart(bn,1):coordAngEnd(bn,1)) = angMatSlim(coordAngStart(bn,2), coordAngStart(bn,1):coordAngEnd(bn,1));
    end
    
    angMatUse2 = angMatSlim(sortIdStart2,:);
    %                                         angMatUse(angMatUse == 100) = 0;
    %                                         angMatUseDiff = diff(angMatUse')';
    uniqueStartX = unique(coordAngStart(:,1));
    coordStartEnd = [coordAngStart coordAngEnd];
    for gf1 = 1:length(uniqueStartX)
        idXX = find(coordAngStart(:,1) == uniqueStartX(gf1));
        [~, idmaxx] = max(coordAngEnd(idXX,2));
        %                                             angMatUse3((gf1), uniqueX(gf1):coordAngEnd(bn,1)) = angMatSlim(coordAngStart(bn,2), coordAngStart(bn,1):coordAngEnd(bn,1));
        angMatUseNew((gf1), uniqueStartX(gf1):coordAngEnd(idXX(idmaxx),1)) = angMatSlim(coordAngStart(idXX(idmaxx),2),  uniqueStartX(gf1):coordAngEnd(idXX(idmaxx),1));
    end
    angMatUseNewBak = angMatUseNew;
    
elseif 0
    [angMatUseNewBak, angMatUseNew] = Processing(obj, obj.angOptManager.angOptMat);
    [angMatUseNewUpperBak, angMatUseNewUpper] = Processing(obj, obj.angOptManager.angOptUpperMat);
    [angMatUseNewLowerBak, angMatUseNewLower] = Processing(obj, obj.angOptManager.angOptLowerMat);
else
    kghvjk = 1;
end
%                                         obj.angOptManager.angOptMat = angOptStackMat;
if 0
    obj.angOptManager.angOptMatUniq = angMatUseNewBak;
    obj.angOptManager.angOptUpperMatUniq = angMatUseNewUpperBak;
    obj.angOptManager.angOptLowerMatUniq = angMatUseNewLowerBak;
end
 





dispErrExpStack33 = obj.dispErrExpStack3;
frameId00 = cell2mat(dispErrExpStack33(:,1));
dispErrExpStack33(setdiff(1:size(dispErrExpStack33,1), frameId00),:) = [];

angOptMat_2 = [];angOptUpperMat_2 = []; angOptLowerMat_2 = []; angOptGTMat_2 = [];

for ko = 1 : size(dispErrExpStack33,1)
    tempAngs = abs(dispErrExpStack33{ko, 5});
    frameRng = [dispErrExpStack33{ko, 2} - dispErrExpStack33{ko, 4} + 1 : dispErrExpStack33{ko, 2} - dispErrExpStack33{ko, 4} + 1 + size(tempAngs,1)];
    angOptMat_2(ko,frameRng) = [100 tempAngs(:,1)'];
    angOptUpperMat_2(ko,frameRng) = [100 tempAngs(:,2)'];
    angOptLowerMat_2(ko,frameRng) = [100 tempAngs(:,3)'];
    angOptGTMat_2(ko,frameRng) = [100 tempAngs(:,4)'];
    
end

obj.angOptManager.angErr_before_after = [];
obj.angOptManager.keyEndAng = [];
obj.angOptManager.refAng = obj.accumP2CRef;
try
    %     angOptMat__2 = ArrangAngOpt(obj);
    if 1 % mod(length(obj.accumP2CRef),2) == 0
        flipWeight = true; false; true; false; true;
    else
        flipWeight = false;
    end
    flipWeight2 = true;
    frame_k = 5; 7; 5; 3; 6; 7; 3; 5; 50; 8; 5;
    
    
% %     frame_k3 = [];
    
    [angOptMat__2, dispErrMatDetailed_before, dispErrMatDetailed_after, dispErrMatDetailed_gt, keyEndAng, dispErrMatDetailed_gtgt,frame_k1] = ArrangAngOpt(obj,flipWeight,flipWeight2, frame_k);
  
    dispErrMatDetailed_gt_bak = dispErrMatDetailed_gt;
    dispErrMatDetailed_gtgt_bak = dispErrMatDetailed_gtgt;
    
    [~, ~, ~, ~, keyEndAng2, ~,frame_k2] = ArrangAngOpt(obj,~flipWeight,flipWeight2, frame_k);
    obj.angOptManager.keyEndAng = keyEndAng;
    obj.angOptManager.keyEndAng2 = keyEndAng2;
    angErr_before_after = [obj.angOptManager.keyEndAng(:,1) rad2deg(obj.angOptManager.keyEndAng(:,2) - obj.angOptManager.keyEndAng(:,4)) rad2deg(obj.angOptManager.keyEndAng(:,3) - obj.angOptManager.keyEndAng(:,4))];
    angErr_before_after__1 = [obj.angOptManager.keyEndAng2(:,1) rad2deg(obj.angOptManager.keyEndAng2(:,2) - obj.angOptManager.keyEndAng2(:,4)) rad2deg(obj.angOptManager.keyEndAng2(:,3) - obj.angOptManager.keyEndAng2(:,4))];
    
    try
        [keyEndAng_3, Diff, Err, keyEndAng_top, IndCC] = ArrangAngOpt2(obj, flipWeight,flipWeight2, frame_k, true, false);
        [keyEndAng_32, Diff2, Err2, keyEndAng_tail] = ArrangAngOpt2(obj, flipWeight,~flipWeight2, 11111111, true, false);
    catch
% % % %         try
% % % %             [keyEndAng_32, Diff2, Err2, keyEndAng_tail] = ArrangAngOpt2(obj, flipWeight,~flipWeight2, 11111111, true, false);
% % % %         catch
% % % %             sdghkj = 1;
% % % %         end
        asgkbj = 1;
    end
    
    try
        idCom = intersect(keyEndAng_tail(:,1), keyEndAng_top(:,1));
        id1 = find(ismember(keyEndAng_top(:,1), idCom));
        id2 = find(ismember(keyEndAng_tail(:,1), idCom));
        keyEndAngComb = [keyEndAng_tail(id2,1) keyEndAng_tail(id2,3) keyEndAng_top(id1,3)];
        Diffsl = [Diff(id1,1) Diff(id1,3) - Diff(id1,6) Diff2(id2,3) - Diff2(id2,6)];
        Diffsl2 = [Diff(id1,1) Diff(id1,3) - Diff(id1,6) Diff(id1,2) - Diff(id1,6) Diff(id1,4) - Diff(id1,6) Diff(id1,5) - Diff(id1,6)];
        Diffsl1 = [Diff(:,1) Diff(:,3) - Diff(:,6) Diff(:,2) - Diff(:,6) Diff(:,4) - Diff(:,6) Diff(:,5) - Diff(:,6)];
        
        figure(27);clf;
        subplot(2,2,1);plot(keyEndAngComb(:,1),rad2deg(cumsum(keyEndAngComb(:,2))));hold on;plot(keyEndAngComb(:,1), rad2deg(cumsum(keyEndAngComb(:,3))));
%         plot(keyEndAngComb(:,1),rad2deg(cumsum(keyEndAngComb(:,3))) -  rad2deg(cumsum(keyEndAngComb(:,2))));
%         plot(Diffsl2(:,1), rad2deg(Diffsl2(:,2 : 5)),'-x');
        plot(Diffsl1(:,1), rad2deg(Diffsl1(:,2 : 5)),'-x');
        grid on;
        legend('min','max','max', 'min','gt min','gt max');
    
        ratio = 0.1;
        max_k = max(max(abs(keyEndAngComb(:,2))));
        id333 = find(abs(keyEndAngComb(:,2)) > ratio.*max_k);
        
        subplot(2,2,3),hist(keyEndAngComb(id333,3)./keyEndAngComb(id333,2),10);
        subplot(2,2,2);plot(keyEndAngComb(:,1),rad2deg(keyEndAngComb(:,2)));hold on;plot(keyEndAngComb(:,1), rad2deg(keyEndAngComb(:,3)));
        grid on;
        legend('min','max');
        subplot(2,2,4),hist(cell2mat(IndCC));legend('max','min');
        saveas(gcf,fullfile(probPath,sprintf('final_error_%05d.fig',2999)));
    catch
        srkjb = 1;
    end
    
    
    if 0
        figure,plot([cumsum(angErr_before_after(:,3)) cumsum(angErr_before_after__1(:,3))]);legend('short','long');
        figure,hist(angErr_before_after__1(:,3)./angErr_before_after(:,3), 100);
        figure,plot([(angErr_before_after(:,3)) (angErr_before_after__1(:,3))]);legend('short','long');
        figure,plot(rad2deg(keyEndAng_3(:,2:3)));legend('short','long');
%         figure,plot([Diff(:,2)-Diff(:,4)
%         Diff(:,3)-Diff(:,4)]);legend('short','long'); 
        figure,plot(rad2deg([(Diff(:,2) - Diff(:,3)) (Diff(:,4) - Diff(:,5))]));legend('short','long');
    end
    
    
    
    if ~isempty(dispErrMatDetailed_before)
        angMatErrK2C_before = Precessing2(obj, dispErrMatDetailed_before, dispErrMatDetailed_gt);
        angMatErrK2C_after = Precessing2(obj, dispErrMatDetailed_after, dispErrMatDetailed_gt);
        
        
        
        obj.angOptManager.angMatErrK2C_before = angMatErrK2C_before;
        obj.angOptManager.angMatErrK2C_after = angMatErrK2C_after;
        
        
        dispErrMatDetailed_before(isnan(dispErrMatDetailed_before)) = 0; dispErrMatDetailed_before(dispErrMatDetailed_before == 100) = 0;
        dispErrMatDetailed_after(isnan(dispErrMatDetailed_after)) = 0; dispErrMatDetailed_after(dispErrMatDetailed_after == 100) = 0;
        dispErrMatDetailed_gt(isnan(dispErrMatDetailed_gt)) = 0; dispErrMatDetailed_gt(dispErrMatDetailed_gt == 100) = 0;
        dispErrMatDetailed_gtgt(isnan(dispErrMatDetailed_gtgt)) = 0; dispErrMatDetailed_gtgt(dispErrMatDetailed_gtgt == 100) = 0;
        
        
        
        dispErrMatDetailed_before_Diff = diff(dispErrMatDetailed_before')';
        dispErrMatDetailed_before_Diff(dispErrMatDetailed_before_Diff < 0) = 0;
        dispErrMatDetailed_after_Diff = diff(dispErrMatDetailed_after')';
        dispErrMatDetailed_after_Diff(dispErrMatDetailed_after_Diff < 0) = 0;
        dispErrMatDetailed_gt_Diff = diff(dispErrMatDetailed_gt')';
        dispErrMatDetailed_gt_Diff(dispErrMatDetailed_gt_Diff < 0) = 0;
        dispErrMatDetailed_gtgt_Diff = diff(dispErrMatDetailed_gtgt')';
        dispErrMatDetailed_gtgt_Diff(dispErrMatDetailed_gtgt_Diff < 0) = 0;
        
        validAngOpt2 = dispErrMatDetailed_before_Diff > 0;
        validAngOptSum2 = sum(validAngOpt2);
        
        %         flipWeight = true; false; true; false; true;
        evenWeight = false; true; false; true;false;
        longestWeightMax = true;  false; true;
        shortestWeightMax = true;
        
        if 0
            weightMat = [];
            for tr = 1 : size(dispErrMatDetailed_before_Diff,2)
                tempWeightMat = dispErrMatDetailed_before_Diff(:,1:tr) > 0;
                if 1
                    tempWeightMatSum = sum(tempWeightMat')';
                else
                    tempWeightMatSum = tempWeightMat(:,end) > 0;
                end
                tempWeightMatSum(tempWeightMat(:,end) == 0) = 0;
                
                if flipWeight
                    idWeight = find(tempWeightMatSum > 0);
                    tempWeightMatSum_ = tempWeightMatSum;
                    tempWeightMatSum_(idWeight) = tempWeightMatSum(flipud(idWeight));
                    
                    if shortestWeightMax
                        idShortest = find(tempWeightMatSum_ > 0);
                        if length(idShortest) > 1
                            tempWeightMatSum_(idShortest(1):idShortest(end-1)) = 0;
                        end
                        
                        
                    end
                    
                    
                    
                else
                    
                    tempWeightMatSum_ = tempWeightMatSum;
                    if longestWeightMax
                        idLongest = find(tempWeightMatSum_ > 0);
                        if length(idLongest) > 1
                            tempWeightMatSum_(idLongest(2):idLongest(end)) = 0;
                        end
                        
                        
                    end
                end
                
                if tr == 1
                    weightMat = [tempWeightMat ];
                else
                    weightMat = [weightMat  tempWeightMatSum_];
                end
                
            end
            weightMatNorm = (weightMat./repmat(sum(weightMat),size(weightMat,1),1));
        else
            weightMatNorm = CalcWeight(dispErrMatDetailed_before_Diff, flipWeight, shortestWeightMax, longestWeightMax);
        end
        weightMatNormTmp = CalcWeight(dispErrMatDetailed_before_Diff, ~flipWeight, shortestWeightMax, longestWeightMax);
        
        if size(dispErrMatDetailed_before_Diff,1) > 1
            meanDispErrMatDetailed_gt_Diff = (sum(dispErrMatDetailed_gt_Diff)./validAngOptSum2)';
            meanDispErrMatDetailed_gtgt_Diff = (sum(dispErrMatDetailed_gtgt_Diff)./validAngOptSum2)';
        else
            meanDispErrMatDetailed_gt_Diff = ((dispErrMatDetailed_gt_Diff)./1)';
            meanDispErrMatDetailed_gtgt_Diff = ((dispErrMatDetailed_gtgt_Diff)./1)';
        end
        
        
        
        if evenWeight
            if size(dispErrMatDetailed_before_Diff,1) > 1
                meanDispErrMatDetailed_before_Diff = (sum(dispErrMatDetailed_before_Diff)./validAngOptSum2)';
                meanDispErrMatDetailed_after_Diff = (sum(dispErrMatDetailed_after_Diff)./validAngOptSum2)';
                meanDispErrMatDetailed_gt_Diff = (sum(dispErrMatDetailed_gt_Diff)./validAngOptSum2)';
            else
                meanDispErrMatDetailed_before_Diff = ((dispErrMatDetailed_before_Diff)./1)';
                meanDispErrMatDetailed_after_Diff = ((dispErrMatDetailed_after_Diff)./1)';
                meanDispErrMatDetailed_gt_Diff = ((dispErrMatDetailed_gt_Diff)./1)';
                
            end
            
        else
            if size(dispErrMatDetailed_before_Diff,1) > 1
                meanDispErrMatDetailed_before_Diff = sum(weightMatNorm.*dispErrMatDetailed_before_Diff)';
                meanDispErrMatDetailed_after_Diff = sum(weightMatNorm.*dispErrMatDetailed_after_Diff)';
                meanDispErrMatDetailed_gt_Diff = sum(weightMatNorm.*dispErrMatDetailed_gt_Diff)';
                
                meanDispErrMatDetailed_before_Diff2 = sum(weightMatNormTmp.*dispErrMatDetailed_before_Diff)';
                meanDispErrMatDetailed_after_Diff2 = sum(weightMatNormTmp.*dispErrMatDetailed_after_Diff)';
                meanDispErrMatDetailed_gt_Diff2 = sum(weightMatNormTmp.*dispErrMatDetailed_gt_Diff)';
                
            else
                meanDispErrMatDetailed_before_Diff = ((dispErrMatDetailed_before_Diff)./1)';
                meanDispErrMatDetailed_after_Diff = ((dispErrMatDetailed_after_Diff)./1)';
                meanDispErrMatDetailed_gt_Diff = ((dispErrMatDetailed_gt_Diff)./1)';
                
                meanDispErrMatDetailed_before_Diff2 = ((dispErrMatDetailed_before_Diff)./1)';
                meanDispErrMatDetailed_after_Diff2 = ((dispErrMatDetailed_after_Diff)./1)';
                meanDispErrMatDetailed_gt_Diff2 = ((dispErrMatDetailed_gt_Diff)./1)';
                
            end
            
            
        end
        
        if 0
            cumsum(abs([0;meanDispErrMatDetailed_gt_Diff])) - obj.accumP2CRef(1:16);
            
            figure, plot(cumsum(abs([0;meanDispErrMatDetailed_gt_Diff])) - abs(obj.accumP2CRef(1:size(meanDispErrMatDetailed_gt_Diff,1)+1)));
            figure, plot((abs([meanDispErrMatDetailed_gt_Diff])) - diff(abs(obj.accumP2CRef(1:size(meanDispErrMatDetailed_gt_Diff,1)+1))));
            
            figure,plot( (cumsum(abs([single(meanDispErrMatDetailed_gt_Diff(1:size(keyEndAng,1)))]))) - (cumsum(keyEndAng(:,4))));
            hold on;plot( (cumsum(abs([single(meanDispErrMatDetailed_after_Diff(1:size(keyEndAng,1)))]))) - (cumsum(keyEndAng(:,3))));
            hold on;plot( (cumsum(abs([single(meanDispErrMatDetailed_before_Diff(1:size(keyEndAng,1)))]))) - (cumsum(keyEndAng(:,2))));
            figure,plot( single(cumsum(abs([(meanDispErrMatDetailed_gt_Diff(1:size(keyEndAng,1)))]))) - (cumsum(keyEndAng(:,4))));
            
             
            figure,subplot(1,2,1);plot(rad2deg(cumsum((sum(immultiply(weightMatNorm, dispErrMatDetailed_after_Diff - dispErrMatDetailed_gtgt_Diff))))));title('short')
            subplot(1,2,2),plot(rad2deg(cumsum((sum(immultiply(weightMatNormTmp, dispErrMatDetailed_after_Diff - dispErrMatDetailed_gtgt_Diff))))));title('long');
            
            
        end
        
        
        
        if 0
            meanDispErrMatDetailed_gt_Diff(isnan(meanDispErrMatDetailed_gt_Diff)) = 0;
            meanDispErrMatDetailed_gt_Diff2(isnan(meanDispErrMatDetailed_gt_Diff2)) = 0;
        end
        
        
        if length(meanDispErrMatDetailed_before_Diff) > 5
            
            diffRef = abs(diff(obj.accumP2CRef));
            
            zeroId1 = find(meanDispErrMatDetailed_before_Diff == 0) ;
            meanDispErrMatDetailed_before_Diff(zeroId1) = diffRef(zeroId1);
            meanDispErrMatDetailed_after_Diff(zeroId1) = diffRef(zeroId1);
            meanDispErrMatDetailed_gt_Diff(zeroId1) = diffRef(zeroId1);
            
            
            
            
            zeroId2 = find(meanDispErrMatDetailed_before_Diff2 == 0) ;
            meanDispErrMatDetailed_before_Diff2(zeroId2) = diffRef(zeroId2);
            meanDispErrMatDetailed_after_Diff2(zeroId2) = diffRef(zeroId2);
            meanDispErrMatDetailed_gt_Diff2(zeroId2) = diffRef(zeroId2);
            
            
        end
        
        
        
        
        
        
        
        
        obj.angOptManager.gtDispTheta = [cumsum([0;single(meanDispErrMatDetailed_gt_Diff)]) cumsum([0;single(meanDispErrMatDetailed_gt_Diff2)])];
        
        obj.angOptManager.angErr_before_after = [[cumsum([0;single(meanDispErrMatDetailed_before_Diff)])] - [cumsum(abs([0;single(meanDispErrMatDetailed_gt_Diff)]))]   [cumsum([0;single(meanDispErrMatDetailed_after_Diff)])] - [cumsum(abs([0;single(meanDispErrMatDetailed_gt_Diff)]))]];
        obj.angOptManager.angErr_before_after2 = [[cumsum([0;single(meanDispErrMatDetailed_before_Diff2)])] - [cumsum(abs([0;single(meanDispErrMatDetailed_gt_Diff)]))]   [cumsum([0;single(meanDispErrMatDetailed_after_Diff2)])] - [cumsum(abs([0;single(meanDispErrMatDetailed_gt_Diff)]))]];
        
        obj.angOptManager.meanDispErrMatDetailed_before_Diff = cumsum([0;single(meanDispErrMatDetailed_before_Diff)]);
        obj.angOptManager.meanDispErrMatDetailed_after_Diff = cumsum([0;single(meanDispErrMatDetailed_after_Diff)]);
        obj.angOptManager.meanDispErrMatDetailed_gt_Diff = cumsum([0;single(meanDispErrMatDetailed_gt_Diff)]);
        
        
        
        
    else
        obj.angOptManager.angErr_before_after = [];
    end
    
    
catch
    obj.angOptManager.angErr_before_after = [];
    wkhj = 1;
end

if 1
    angOptMat_2_0 = angOptMat_2;
    angOptGTMat_2_0 = angOptGTMat_2;
    
    angMatUseNewBak = angOptMat_2; angMatUseNew = angOptMat_2;
    angMatUseNewUpperBak = angOptUpperMat_2; angMatUseNewUpper = angOptUpperMat_2;
    angMatUseNewLowerBak = angOptLowerMat_2; angMatUseNewLower = angOptLowerMat_2;
    
    if 0
        figure,imshow(angOptMat_2 - angOptMat__2, []);
    end
    
    
end



angOptMat_2(angOptMat_2 == 100) = 0;
angOptGTMat_2(angOptGTMat_2 == 100) = 0;

angOptMat_diff_2 = diff(angOptMat_2')';
angOptMat_diff_2(angOptMat_diff_2 < 0) = 0;

angOptGTMat_diff_2 = diff(angOptGTMat_2')';
angOptGTMat_diff_2(angOptGTMat_diff_2 < 0) = 0;

angMatErr0 = rad2deg(angOptMat_2_0 - angOptGTMat_2_0);
angMatErr0(angMatErr0 == 0) = nan;

angMatErr = rad2deg(angOptMat_diff_2 - angOptGTMat_diff_2);
angMatErr = [zeros(size(angMatErr,1),1) angMatErr];
angMatErr(angMatErr == 0) = nan;



if 0
    if 0
        figure(4),clf;subplot(1,2,1);imshow(immultiply(angMatUseNewBak , angMatUseNewBak ~= 100), []);subplot(1,2,2);imshow(angMatUseNewBak & angMatUseNewBak ~= 100, [])
    elseif 0
        figure(4),clf;imshow(angMatUseNewBak & angMatUseNewBak ~= 100, [])
    else
        % %         angMatErr = angOptMat_2 - angOptGTMat_2;
        % %         angMatErr(angMatErr == 0) = nan;
        figure(3),clf; imshow(angMatErr, [])
    end
end





obj.angOptManager.angOptMatUniq = angMatUseNewBak;
obj.angOptManager.angOptUpperMatUniq = angMatUseNewUpperBak;
obj.angOptManager.angOptLowerMatUniq = angMatUseNewLowerBak;

if obj.accumP2CRef(end) > 0
    obj.angOptManager.angOptErrMatUniq = angMatErr;
    obj.angOptManager.angOptErrK2CMatUniq = angMatErr0;
else
    obj.angOptManager.angOptErrMatUniq = -angMatErr;
    obj.angOptManager.angOptErrK2CMatUniq = -angMatErr0;
end

try
    aaaa = diff(obj.angOptManager.angOptErrK2CMatUniq')';
    bbbb = obj.angOptManager.angOptErrMatUniq;
    abErr = aaaa - bbbb(:,2:end);
catch
    asgfhk = 1;
end



angMatUseNew(angMatUseNew == 100) = 0;
angMatUseNewUpper(angMatUseNewUpper == 100) = 0;
angMatUseNewLower(angMatUseNewLower == 100) = 0;
if 0
    angArrangeErr = angMatUseNew - angOptListLast;
end


angMatUseNewDiff = diff(angMatUseNew')';
angMatUseNewDiff(angMatUseNewDiff < 0) = 0;

angMatUseNewUpperDiff = diff(angMatUseNewUpper')';
angMatUseNewUpperDiff(angMatUseNewUpperDiff < 0) = 0;

angMatUseNewLowerDiff = diff(angMatUseNewLower')';
angMatUseNewLowerDiff(angMatUseNewLowerDiff < 0) = 0;






validAngOpt = angMatUseNewDiff > 0;
validAngOpt_ = validAngOpt;
validAngOptSum = sum(validAngOpt_);

% accumErrWeight = sum(validAngOpt')';
if 0
    validAngOpt_0 = ~validAngOpt;
    [L, LL] = bwlabel(validAngOpt_0,8);
    for oi = 1 : LL
        tempArea = L == oi;
        if tempArea(1,end)
            validAngOpt(find(tempArea)) = true;
            break;
        end
    end
    
else
    % %     [[1:size(validAngOpt,2)]' validAngOptSum'];
    
    
    [yy, xx] = ind2sub(size(validAngOpt), find(validAngOpt > 0));
    [a, b, c] = unique(xx);
    coord = [];
    for io = 1 : length(a)
        idTemp = find(xx == a(io));
        coord_ = [a(io) max(yy(idTemp))];
        coord = [coord; coord_];
        validAngOpt(1:coord_(2), coord_(1)) = true;
    end
    
end

accumErrWeight = sum(validAngOpt')';
if 1  % change of weight, the shorter the trace, the higher the weight 20200310
    accumErrWeight = flipud(accumErrWeight);
end

accumErrWeightMat = repmat(accumErrWeight,1,size(validAngOpt,2));
accumErrWeightMatValid = (accumErrWeightMat.*validAngOpt_)';
accumErrWeightMatValid0 = accumErrWeightMatValid;
accumErrWeightMatValid(accumErrWeightMatValid == 0) = nan;
accumErrWeightMatValid00 = accumErrWeightMatValid;
minId = min(accumErrWeightMatValid')';
minIdMat = repmat(minId-1, 1, size(accumErrWeightMatValid,2));
accumErrWeightMatValid = accumErrWeightMatValid - minIdMat;
accumErrWeightMatValid0000 = accumErrWeightMatValid;
accumErrWeightMatValid(isnan(accumErrWeightMatValid)) = 0;
if 0
    accumErrWeightMatValid = 1./accumErrWeightMatValid;
    accumErrWeightMatValid(isinf(accumErrWeightMatValid)) = 0;
elseif 0
    accumErrWeightMatValid(accumErrWeightMatValid == 0) = nan;
    accumErrWeightMatValid = max(accumErrWeightMatValid(:)) - accumErrWeightMatValid+1;
    accumErrWeightMatValid(isnan(accumErrWeightMatValid)) = 0;
else
    saknab = 1;
end

if 0
    accumErrWeightMatValid_ = 2.^(-accumErrWeightMatValid);
    accumErrWeightMatValid_(accumErrWeightMatValid_ == 1) = 0;
else
    accumErrWeightMatValid_ = accumErrWeightMatValid;
end

accumErrWeightMatValidNorm = (accumErrWeightMatValid_./repmat(sum(accumErrWeightMatValid_')',1,size(accumErrWeightMatValid_,2)))';
azjsg = 1;
if 0
    if size(angMatUseNewDiff,1) > 1
        meanAngP2C = (sum(angMatUseNewDiff)./validAngOptSum)';
        meanAngP2CUpper = (sum(angMatUseNewUpperDiff)./validAngOptSum)';
        meanAngP2CLower = (sum(angMatUseNewLowerDiff)./validAngOptSum)';
    else
        meanAngP2C = ((angMatUseNewDiff)./1)';
        meanAngP2CUpper = ((angMatUseNewUpperDiff)./1)';
        meanAngP2CLower = ((angMatUseNewLowerDiff)./1)';
        
    end
elseif 0
    meanAngP2C = sum(accumErrWeightMatValidNorm.*angMatUseNewDiff)';
    meanAngP2CUpper = sum(accumErrWeightMatValidNorm.*angMatUseNewUpperDiff)';
    meanAngP2CLower = sum(accumErrWeightMatValidNorm.*angMatUseNewLowerDiff)';
else
    if size(angMatUseNewDiff,1) > 1
        meanAngP2C = sum(accumErrWeightMatValidNorm.*angMatUseNewDiff)';
        meanAngP2CUpper = sum(accumErrWeightMatValidNorm.*angMatUseNewUpperDiff)';
        meanAngP2CLower = sum(accumErrWeightMatValidNorm.*angMatUseNewLowerDiff)';
    else
        meanAngP2C = ((angMatUseNewDiff)./1)';
        meanAngP2CUpper = ((angMatUseNewUpperDiff)./1)';
        meanAngP2CLower = ((angMatUseNewLowerDiff)./1)';
        
    end
    
end

angErr = rad2deg([0;cumsum(meanAngP2C)] - abs(obj.accumP2CRef));
angErrP2C = [0;rad2deg([(meanAngP2C)] - diff(abs(obj.accumP2CRef)))];

angErrUpper = rad2deg([0;cumsum(meanAngP2CUpper)] - abs(obj.accumP2CRef));
angErrLower = rad2deg([0;cumsum(meanAngP2CLower)] - abs(obj.accumP2CRef));

if 0
    [optAng, upperAng, lowerAng] = CalcBound(obj, angMatUseNewBak, angMatUseNewUpperBak, angMatUseNewLowerBak);
else
    
    traceLen = 155;
    try
%         [~, ~, ~, tempP2CCum] = CalcBound(obj, angMatUseNewBak, angMatUseNewUpperBak, angMatUseNewLowerBak);
        [~, ~, ~, tempP2CCumGT] = CalcBound(obj, dispErrMatDetailed_gt_bak, dispErrMatDetailed_gt_bak, dispErrMatDetailed_gt_bak,traceLen);
        [~, ~, ~, tempP2CCumRef] = CalcBound(obj, dispErrMatDetailed_gtgt_bak, dispErrMatDetailed_gtgt_bak, dispErrMatDetailed_gtgt_bak,traceLen);
        
        
        if 0
            figure(16),clf;hold on
            for jj = [5 8 15]
                [~, ~, ~, tempP2CCumGT2] = CalcBound(obj, dispErrMatDetailed_gt_bak, dispErrMatDetailed_gt_bak, dispErrMatDetailed_gt_bak,jj);
                [~, ~, ~, tempP2CCumRef2] = CalcBound(obj, dispErrMatDetailed_gtgt_bak, dispErrMatDetailed_gtgt_bak, dispErrMatDetailed_gtgt_bak,jj);
                plot(rad2deg(tempP2CCumGT2 - tempP2CCumRef2));
                plot([1:jj-1:length(tempP2CCumRef2)], rad2deg(tempP2CCumGT2([1:jj-1:length(tempP2CCumRef2)]) - tempP2CCumRef2([1:jj-1:length(tempP2CCumRef2)])), 'or');
                pause(0.5);
                drawnow;
            end
        end
        
        
        
        KeyBaseErrGtZ = rad2deg(tempP2CCumGT - tempP2CCumRef);
    catch
        KeyBaseErrGtZ = 0;
        tempP2CCum = 0;
    end
    optAng = [0;cumsum(meanAngP2C)];
    upperAng = [0;cumsum(meanAngP2CUpper)];
    lowerAng = [0;cumsum(meanAngP2CLower)];
end



splitAngLen = 16;

splitId = [1: splitAngLen : length(optAng)];

if splitId(end) < length(optAng)
    splitId = [splitId length(optAng)];
end

optAng3 = [];
upperAng3 = [];
lowerAng3 = [];
for i = 1 : length(splitId) - 1
    
    if i == 1
        optAng3 = [optAng3; optAng(splitId(i):splitId(i+1))];
        upperAng3 = [upperAng3; upperAng(splitId(i):splitId(i+1))];
        lowerAng3 = [lowerAng3; lowerAng(splitId(i):splitId(i+1))];
        
    else
        tempUpper = upperAng(splitId(i):splitId(i+1)) - upperAng(splitId(i));
        tempLower = lowerAng(splitId(i):splitId(i+1)) - lowerAng(splitId(i));
        tempOpt = optAng(splitId(i):splitId(i+1)) - optAng(splitId(i));
        
        %         upperAng3 = [upperAng3; upperAng(splitId(i):splitId(i+1))];
        
        upperAng3 = [upperAng3; optAng3(end) + tempUpper(2:end)];
        lowerAng3 = [lowerAng3; optAng3(end) + tempLower(2:end)];
        optAng3 = [optAng3; optAng3(end) + tempOpt(2:end)];
    end
    
    
    
end
angErr3 = rad2deg([optAng3] - abs(obj.accumP2CRef));
angErrUpper3 = rad2deg([upperAng3] - abs(obj.accumP2CRef));
angErrLower3 = rad2deg([lowerAng3] - abs(obj.accumP2CRef));
if 0
    figure,plot(angErr3,'b');hold on;plot([angErrUpper3 angErrLower3],'g')
end


angErr2 = rad2deg([optAng] - abs(obj.accumP2CRef));
angErrUpper2 = rad2deg([upperAng] - abs(obj.accumP2CRef));
angErrLower2 = rad2deg([lowerAng] - abs(obj.accumP2CRef));


% dlt = angMatUseNewDiff - angMatUseNewUpperDiff;
dltUpper = angMatUseNewUpperBak - angMatUseNewBak;
dltLower = angMatUseNewLowerBak - angMatUseNewBak;

validAngOptUL = abs(dltUpper) > 0;
validAngOptULSum = sum(validAngOptUL);

accumErrWeightUL = sum(validAngOptUL')';
accumErrWeightMatUL = repmat(accumErrWeightUL,1,size(validAngOptUL,2));
accumErrWeightMatValidUL = (accumErrWeightMatUL.*validAngOptUL)';


% meanAngP2C = (sum(angMatUseNewDiff)./validAngOptSum)';
meanAngP2CUpperDlt = (sum(dltUpper)./validAngOptULSum)';
meanAngP2CLowerDlt = (sum(dltLower)./validAngOptULSum)';
meanAngP2CUpperDlt(isnan(meanAngP2CUpperDlt)) = 0;
meanAngP2CLowerDlt(isnan(meanAngP2CLowerDlt)) = 0;


angErrUpper = angErr + rad2deg(meanAngP2CUpperDlt);
angErrLower = angErr + rad2deg(meanAngP2CLowerDlt);


% angErr3 = rad2deg([optAng3] - abs(obj.accumP2CRef));
% angErrUpper3 = rad2deg([upperAng3] - abs(obj.accumP2CRef));
% angErrLower3 = rad2deg([lowerAng3] - abs(obj.accumP2CRef));

end

function [angMatUseNewBak, angMatUseNew] = Processing(obj, angOptMat)

a1 = angOptMat(obj.validFeatIdStack,:);%  >= 0;
a2 = obj.traceManager.X(obj.validFeatIdStack,:); %  >= 0;

% % % % % % a1_upper = obj.angOptManager.angOptUpperMat(obj.validFeatIdStack,:);%  >= 0;
% % % % % % a2_upper = obj.traceManager.X(obj.validFeatIdStack,:); %  >= 0;
% % % % % %
% % % % % % a1_lower = obj.angOptManager.angOptLowerMat(obj.validFeatIdStack,:);%  >= 0;
% % % % % % a2_lower = obj.traceManager.X(obj.validFeatIdStack,:); %  >= 0;

a11 = angOptMat > 0;
checkManagement = (a1>0) - (a2>0);
checkManagementSum = sum(double(checkManagement(:)));

% % % % % % a11_upper = obj.angOptManager.angOptUpperMat > 0;
% % % % % % checkManagement_upper = (a1_upper>0) - (a2_upper>0);
% % % % % % checkManagementSum_upper = sum(double(checkManagement_upper(:)));




angTrace = sum(a11')';
angMatSort = immultiply(a11,angOptMat);
%                                         angTrace1 = sum(obj.angOptManager.angOptMat')';
angTrace = sum(angMatSort')';
%                                         [angTraceUnique1,a31,a41] = unique(angTrace1);
[angTraceUnique,a3,a4] = unique(angTrace);

if 0
    angTrace1 = sum(obj.angOptManager.angOptMat')';
    [angTraceUnique1,a31,a41] = unique(angTrace1);
    angMatSlim1 = obj.angOptManager.angOptMat(a31,:);
else
    angMatSlim = angMatSort(a3,:);
end



startInd = find(angMatSlim == 100);
[startIndY, startIndX] = ind2sub(size(angMatSlim), startInd);
[~, sortIdStart2] = sort(startIndX , 'ascend');
[~, sortIdStart] = sort(startIndY , 'ascend');
startIndX = startIndX(sortIdStart);
startIndY = startIndY(sortIdStart);
endInd = find(angMatSlim ~= 100 & angMatSlim > 0);
[endIndY, endIndX] = ind2sub(size(angMatSlim), endInd);
[~, sortIdEnd] = sort(endIndY, 'ascend');
endIndX = endIndX(sortIdEnd);
endIndY = endIndY(sortIdEnd);
coordAngEnd = [];
for nm = startIndY'
    idtp = find(endIndY == nm);
    [~, idmaxx] = max(endIndX(idtp));
    coordAngEnd = [coordAngEnd; [endIndX(idtp(idmaxx)) nm]];
end
coordAngStart = [startIndX startIndY];
for bn = 1 : size(coordAngStart,1)
    angMatUse(coordAngStart(bn,2), coordAngStart(bn,1):coordAngEnd(bn,1)) = angMatSlim(coordAngStart(bn,2), coordAngStart(bn,1):coordAngEnd(bn,1));
end

angMatUse2 = angMatSlim(sortIdStart2,:);
%                                         angMatUse(angMatUse == 100) = 0;
%                                         angMatUseDiff = diff(angMatUse')';
uniqueStartX = unique(coordAngStart(:,1));
coordStartEnd = [coordAngStart coordAngEnd];
for gf1 = 1:length(uniqueStartX)
    idXX = find(coordAngStart(:,1) == uniqueStartX(gf1));
    [~, idmaxx] = max(coordAngEnd(idXX,2));
    %                                             angMatUse3((gf1), uniqueX(gf1):coordAngEnd(bn,1)) = angMatSlim(coordAngStart(bn,2), coordAngStart(bn,1):coordAngEnd(bn,1));
    angMatUseNew((gf1), uniqueStartX(gf1):coordAngEnd(idXX(idmaxx),1)) = angMatSlim(coordAngStart(idXX(idmaxx),2),  uniqueStartX(gf1):coordAngEnd(idXX(idmaxx),1));
end
angMatUseNewBak = angMatUseNew;


% % % %                                         obj.angOptManager.angOptMat = angOptStackMat;




end
function [dispErrMatDetailed, dispErrMatDetailed_before, dispErrMatDetailed_after, dispErrMatDetailed_gt, keyEndAng, dispErrMatDetailed_gtgt, frame_k1] = ArrangAngOpt(obj, flipWeight,flipWeight2, frame_k)
if 0
    dispErrExpStack = obj.dispErrExpStack3(:,1:6);
else
    dispErrExpStack = obj.dispErrExpStack3;
end

frameId00 = cell2mat(dispErrExpStack(:,1));
dispErrExpStack(setdiff(1:size(dispErrExpStack,1), frameId00),:) = [];



dispErrMatDetailed = [];
for ko = 1 : size(dispErrExpStack,1)
    tempDisps = dispErrExpStack{ko, 3};
    tempLocalTrace = dispErrExpStack{ko, 7};
    frameRng = [dispErrExpStack{ko, 2} - dispErrExpStack{ko, 4} + 1 : dispErrExpStack{ko, 2} - dispErrExpStack{ko, 4} + 0 + size(tempDisps,1)];
    tempAngOpt = abs(tempLocalTrace.angOpt);
    tempAngOpt_ = [100; tempAngOpt];
    if 0
        for j = 1 : length(frameRng)
            
            %         [a1,b1] = hist(tempDisps_, 100);
            %         dispErrA1 = -dot(b1, (a1./sum(a1)));
            %         dispErrMatDetailed(ko,frameRng(j)) = [dispErrA1];
            %         dispErrMatDetailed(ko,frameRng(j)) = [dispErrA1];
        end
    end
    dispErrMatDetailed(ko,frameRng) = tempAngOpt_';
end


frame_k1 = [];

frameId11 = cell2mat(dispErrExpStack(:,9));
dispErrExpStack2 = dispErrExpStack(find(frameId11 > 0),:);

if ~isempty(dispErrExpStack2)
    
    dispErrMatDetailed_before = []; dispErrMatDetailed_after = []; dispErrMatDetailed_gt = []; dispErrMatDetailed_gtgt = []; keyEndAng = [];
    for ko = 1 : size(dispErrExpStack2,1)
        tempDisps = dispErrExpStack2{ko, 3};
        tempLocalTrace = dispErrExpStack2{ko, 7};
        frameRng = [dispErrExpStack2{ko, 2} - dispErrExpStack2{ko, 4} + 1 : dispErrExpStack2{ko, 2} - dispErrExpStack2{ko, 4} + 0 + size(tempDisps,1)];
        tempAngOpt = [abs(tempLocalTrace.angOpt_old) abs(tempLocalTrace.ANGOptFinal)  abs(tempLocalTrace.gtTheta) abs(tempLocalTrace.localTrace_gtAng2)];
        
        newAng = cell2mat(tempLocalTrace.cfg_new.bbbOpt);
        gtAng = cell2mat(tempLocalTrace.cfg_gt.bbbOpt);
        
        newAng = [zeros(1, size(newAng,2)); newAng];
        gtAng = [zeros(1, size(gtAng,2)); gtAng];
        
        newP2C = rad2deg((diff(newAng)));
        gtP2C = rad2deg((diff(gtAng)));
        diffErrAll = abs( newP2C - gtP2C);

        
        
        [~,idAll___] = max(diffErrAll);
        
        
        diffErrAll2 = diffErrAll(2:end,:);
        [meanErrP2C_,idAll] = max(diffErrAll2);
        idAll = idAll + 1;
        idCood = [[1:size(diffErrAll,2)]' idAll' ];
        idCood2 = [[1:size(diffErrAll,2)]' idAll'+1 ];
        
        
        indAll = sub2ind(size(diffErrAll), idCood(:,2), idCood(:,1));
        meanErrP2C = diffErrAll(indAll);
        
        
        indAll1 = sub2ind(size(newAng), idCood(:,2), idCood(:,1));
        indAll2 = sub2ind(size(newAng), idCood2(:,2), idCood2(:,1));
        
        k2pAng = newAng(indAll1); k2cAng = newAng(indAll2); p2cAng = k2cAng - k2pAng;
        k2pAngGT = gtAng(indAll1); k2cAngGT = gtAng(indAll2); p2cAngGT = k2cAngGT - k2pAngGT;
        
        
        
        if 0
            figure,plot(idAll___);
            figure,plot(meanErrP2C_' - meanErrP2C);
        end
        

        diffErrAll_ = abs(rad2deg((diff(newAng))) - rad2deg((diff(gtAng))));
        tempAngOpt_ = [[100 100 100 100]; tempAngOpt];
        tempAngOpt_bak = tempAngOpt_;
        
        
        if 0
            if ~flipWeight
                tempAngOpt_(end,[2 3]) = [ mean(p2cAng) - mean(p2cAngGT) 0];
                
            else
                tempAngOpt_(2,[2 3]) = [ mean(newAng(2,:)) mean(gtAng(2,:))];
            end
        end
        
        if 0
            for j = 1 : length(frameRng)
                
                %         [a1,b1] = hist(tempDisps_, 100);
                %         dispErrA1 = -dot(b1, (a1./sum(a1)));
                %         dispErrMatDetailed(ko,frameRng(j)) = [dispErrA1];
                %         dispErrMatDetailed(ko,frameRng(j)) = [dispErrA1];
            end
        end
        dispErrMatDetailed_before(ko,frameRng) = tempAngOpt_(:,1)';
        dispErrMatDetailed_after(ko,frameRng) = tempAngOpt_(:,2)';
        dispErrMatDetailed_gt(ko,frameRng) = tempAngOpt_(:,3)';
        dispErrMatDetailed_gtgt(ko,frameRng) = tempAngOpt_(:,4)';
        frameNum = size(tempAngOpt_, 1);
        if ~flipWeight
            if 0
                keyEndAng = [keyEndAng; [dispErrExpStack2{ko,1} tempAngOpt_(end,:)]];
            elseif 0
                keyEndAng = [keyEndAng; [dispErrExpStack2{ko,1} tempAngOpt_(end,:)./(size(tempAngOpt_,1)-1)]];
            elseif 0
                keyEndAng = [keyEndAng; [dispErrExpStack2{ko,1} (tempAngOpt_(end,:) - tempAngOpt_(end - 1,:))]];
            else
                
                tempAngOpt_0 = [zeros(1, size(tempAngOpt_,2)); tempAngOpt_(2:end,:)];
                diffErr = abs(rad2deg((diff(tempAngOpt_0(:,[2])))) - rad2deg((diff(tempAngOpt_0(:,[3])))));
                [~, id1] = max(diffErr);
                frame_k = max([3;id1]);
                frame_k1 = [frame_k1; frame_k];
                keyEndAng = [keyEndAng; [dispErrExpStack2{ko,1} (tempAngOpt_(min(frame_k, frameNum),:) - tempAngOpt_(min(frame_k, frameNum) - 1,:))]];
                keyEndAng(end,[3 4]) = [ mean(p2cAng) - mean(p2cAngGT) 0];
            end
        else
            keyEndAng = [keyEndAng; [dispErrExpStack2{ko,1} tempAngOpt_(2,:)]];
% %             keyEndAng(end,[3 4]) = [ mean(newAng(2,:)) mean(gtAng(2,:))];
        end
    end
else
    dispErrMatDetailed_before = [];
    dispErrMatDetailed_after = [];
    dispErrMatDetailed_gt = [];
    keyEndAng = [];
end


end
function angMatErrK2C0 = Precessing2(obj, angOptMat_2_0, angOptGTMat_2_0)
if 0
    angOptMat_2(angOptMat_2 == 100) = 0;
    angOptGTMat_2(angOptGTMat_2 == 100) = 0;
    
    angOptMat_diff_2 = diff(angOptMat_2')';
    angOptMat_diff_2(angOptMat_diff_2 < 0) = 0;
    
    angOptGTMat_diff_2 = diff(angOptGTMat_2')';
    angOptGTMat_diff_2(angOptGTMat_diff_2 < 0) = 0;
    
    angMatErr = rad2deg(angOptMat_diff_2 - angOptGTMat_diff_2);
    angMatErr = [zeros(size(angMatErr,1),1) angMatErr];
    angMatErr(angMatErr == 0) = nan;
end
angMatErr0 = rad2deg(angOptMat_2_0 - angOptGTMat_2_0);
angMatErr0(angMatErr0 == 0) = nan;
if obj.accumP2CRef(end) > 0
    
    angMatErrK2C0 = angMatErr0;
else
    angMatErrK2C0 = -angMatErr0;
end
end