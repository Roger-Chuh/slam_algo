function [keyEndAng2, Diff0, Err, keyEndAng22, IndCC] = ArrangAngOpt2(obj, flipWeight,flipWeight2, frame_k, forceMaxK, maxK1)

global probPath

% maxK1 = true;


accumP2CRef = obj.accumP2CRef;


forceAngOpt = false; true; false;


if 0
    dispErrExpStack = obj.dispErrExpStack3(:,1:6);
else
    dispErrExpStack = obj.dispErrExpStack3;
end


Err = [];
% forceMaxK = false; true;
maxKInd0 = 4;

maxKInd0 = frame_k;



frameId00 = cell2mat(dispErrExpStack(:,1));
dispErrExpStack(setdiff(1:size(dispErrExpStack,1), frameId00),:) = [];


angOptManager.angOptNewMat = [];
angOptManager.angOptGtMat = [];
angOptManager.angOptWeight = [];
angOptManager.angOptWeight0 = [];
% angOptManager.angOptLowerMat = [];

frame_k1 = [];

frameId11 = cell2mat(dispErrExpStack(:,9));
dispErrExpStack2 = dispErrExpStack(find(frameId11 > 0),:);



validIdList = [];

IndMaxCC = {};
IndMinCC = {};
IndCC = {};
if ~isempty(dispErrExpStack2)
    
    dispErrMatDetailed_before = []; dispErrMatDetailed_after = []; dispErrMatDetailed_gt = []; dispErrMatDetailed_gtgt = []; keyEndAng = [];
    keyEndAng2 = [];
    for ko = 1 : size(dispErrExpStack2,1)
        tempDisps = dispErrExpStack2{ko, 3};
        tempLocalTrace = dispErrExpStack2{ko, 7};
        
        
        
        
        frameRng = [dispErrExpStack2{ko, 2} - dispErrExpStack2{ko, 4} + 1 : dispErrExpStack2{ko, 2} - dispErrExpStack2{ko, 4} + 0 + size(tempDisps,1)];
        tempAngOpt = [abs(tempLocalTrace.angOpt_old) abs(tempLocalTrace.ANGOptFinal)  abs(tempLocalTrace.gtTheta) abs(tempLocalTrace.localTrace_gtAng2)];
        
        
        
        cfg_new = tempLocalTrace.cfg_new;
       cfg_gt = tempLocalTrace.cfg_gt;
        
        
        
         AA1 = cell2mat(cfg_new.bbbOpt);
        BB1 = cell2mat(cfg_gt.bbbOpt);
        AA1 = [zeros(1,size(AA1,2)); AA1];
        BB1 = [zeros(1,size(BB1,2)); BB1];
        CC1 = abs(rad2deg(diff(AA1) - diff(BB1)));
        [maxCC, indMaxCC] = max(CC1);
        [minCC, indMinCC] = min(CC1);
        
        IndMaxCC = [IndMaxCC; {indMaxCC'}];
        IndMinCC = [IndMinCC; {indMinCC'}];
        
        IndCC = [IndCC; [{indMaxCC'} {indMinCC'}]];
        
        newAng = abs(cell2mat(tempLocalTrace.cfg_new.bbbOpt));
        gtAng = abs(cell2mat(tempLocalTrace.cfg_gt.bbbOpt));
        
        newAng0 = newAng; gtAng0 = gtAng;      
        
        if forceAngOpt
%             newAng0 = newAng;
            newAng = repmat(tempAngOpt(:,2),1,size(newAng,2));
%             gtAng0 = gtAng;
            gtAng = repmat(tempAngOpt(:,3),1,size(gtAng,2));
            
        end
        
        
        maxKInd = min(maxKInd0, size(newAng,1));
        
        if 0
            newAng(1,:) =  abs(tempLocalTrace.ANGOptFinal(1));
            gtAng(1,:) =  abs(tempLocalTrace.gtTheta(1));
        end
        valid = intersect(intersect(intersect(tempLocalTrace.cfg_new.Valid, tempLocalTrace.cfg_gt.Valid), find(~isnan(sum(gtAng)))'), find(~isnan(sum(newAng)))');
        
        
        
        newAng = newAng(:,valid);
        gtAng = gtAng(:,valid);
        
        
        AA = cfg_new.AA;   % slope of k
        AA = AA(:,ismember(tempLocalTrace.cfg_new.Valid, valid));
        if 0
            AA = AA - AA(1,:);
            AA(AA < 0) = 0;
        end
        [maxAA,idAA] = max(AA);
        if forceMaxK
            idAA = repmat(maxKInd,1,length(idAA));
        end
        
        
        newAng100 = [100.*ones(1, size(newAng,2)); newAng];
        gtAng100 = [100.*ones(1, size(gtAng,2)); gtAng];
        AA100 = [100.*ones(1, size(AA,2)); AA];
        
        
% %         newAng100 = newAng100(:,valid);
% %         gtAng100 = gtAng100(:,valid);
        if 0
            AA100 = AA100(:,ismember(tempLocalTrace.cfg_new.Valid, valid));
        end
        
        AA100_bak = AA100;
        if 1
            AA100(2:end,:) = AA100(2:end,:) - repmat(AA100(2,:),size(AA100,1)-1,1);
        else
            AA100(2:end,:) = AA100(2:end,:) ./repmat(AA100(2,:),size(AA100,1)-1,1);
            AA100(2,:) = 0;
        end
        
        
%         AA100_bak = AA100;
        
        
        AA100(AA100 <= 0) = 0;
        
        newAng = [zeros(1, size(newAng,2)); newAng];
        gtAng = [zeros(1, size(gtAng,2)); gtAng];
        
        
        
       
        CCC = ((diff(newAng) - diff(gtAng)));
        if 0
            figure,plot(CCC);
            figure,subplot(1,2,1);plot(AA100_bak(2:end,:));subplot(1,2,2);plot(abs(CCC))
        end
        
        
        
        
          idAA_1 = [[1:size(AA,2)]' idAA' ];
          idAA_2 = [[1:size(AA,2)]' idAA'+1 ];

        
        indAA_1 = sub2ind(size(newAng), idAA_1(:,2), idAA_1(:,1));
        indAA_2 = sub2ind(size(newAng), idAA_2(:,2), idAA_2(:,1));
        
        
        indAA_ccc = sub2ind(size(CCC), idAA_1(:,2), idAA_1(:,1));

        CCCLong = mean(CCC(indAA_ccc));
        CCCShort = mean(CCC(1,:)');
        err = [sign(CCCLong) - sign(CCCShort)  sign(abs(CCCLong) - abs(CCCShort))];
        
        
         k2pAng_AA = newAng(indAA_1); k2cAng_AA = newAng(indAA_2); 
         p2cAng_AA = k2cAng_AA - k2pAng_AA;
         p2cAng_AA_12 = newAng(2,:)';
         
        k2pAngGT_AA = gtAng(indAA_1); k2cAngGT_AA = gtAng(indAA_2); 
        p2cAngGT_AA = k2cAngGT_AA - k2pAngGT_AA;
        p2cAngGT_AA_12 = gtAng(2,:)';
        
        
        p2cAng_AA_err = p2cAng_AA - p2cAngGT_AA;
        p2cAng_AA_12_err = p2cAng_AA_12 - p2cAngGT_AA_12;
        
        
        
      err = [err [ abs(mean(p2cAng_AA_err)) - abs(mean(CCC(indAA_ccc)))    abs(mean(p2cAng_AA_12_err)) - abs(mean(CCC(1,:)))]];
        
      
      Err = [Err; err];
        if 1
            keyEndAng2 = [keyEndAng2; [dispErrExpStack2{ko,1}  mean(p2cAng_AA_12_err) mean(p2cAng_AA_err)]];
        else
            keyEndAng2 = [keyEndAng2; [dispErrExpStack2{ko,1}  (tempAngOpt(1,2) - tempAngOpt(1,3)) mean(p2cAng_AA_err)]];
            
        end
        
        validIdList = [validIdList; tempLocalTrace.LocalTrace.featId(valid)];
        
        frmLenTemp = size(tempLocalTrace.LocalTrace.ptIcsX,2);
        angOptTemp_ = abs([100;tempLocalTrace.angOpt]');
        angOptUpperTemp_ = abs([100;tempLocalTrace.angRng(:,1)]');
        angOptLowerTemp_ = abs([100;tempLocalTrace.angRng(:,2)]');
        
        
        
        newAng100(1,:) = 0;
        gtAng100(1,:) = 0;
        
        
        
        if 0
            figure,plot(AA)
            figure,plot(AA - AA(1,:))
            figure,plot(AA100_bak(2:end,:))
        end
        
        
        
        angOptManager.angOptNewMat(tempLocalTrace.LocalTrace.featId(valid), frameRng(1:end-1)) = diff(newAng100)';  % repmat(angOptTemp_, length(tempLocalTrace.LocalTrace.featId),1);
        angOptManager.angOptGtMat(tempLocalTrace.LocalTrace.featId(valid), frameRng(1:end-1)) = diff(gtAng100)';  % repmat(angOptUpperTemp_, length(tempLocalTrace.LocalTrace.featId),1);
        %     angOptManager.angOptLowerMat(tempLocalTrace.LocalTrace.featId(valid), frameRng) = repmat(angOptLowerTemp_, length(tempLocalTrace.LocalTrace.featId),1);
        
        
        AA100_bak_bak = AA100;
        
        if maxK1
%             Set2ZeroInd = setdiff([1:length(AA100(:))]',unique([indAA_1; indAA_2]));
            Set2ZeroInd = setdiff([1:length(AA100(:))]',unique([indAA_1; indAA_1]));
            AA100(Set2ZeroInd) = 0;
            AA100(indAA_1) = 1;
            AA100(1,:) = 100;
        end
        
        AA100;
        
        
        if ko == 1
            
            AA100(2,:) = 1;
        end
        AA1000 = AA100;
        AA1000(3:end,:) = 0;
        AA1000(2,:) = 1;
        
        
        if forceMaxK
            AA100(maxKInd+1,:) = 1;
            AA100(3:maxKInd,:) = 0;
            AA100(maxKInd+2:end,:) = 0;
        
        end
        angOptManager.angOptWeight(tempLocalTrace.LocalTrace.featId(valid), frameRng(1:end-1)) = AA100(2:end,:)';
        angOptManager.angOptWeight0(tempLocalTrace.LocalTrace.featId(valid), frameRng(1:end-1)) = AA1000(2:end,:)';
        
        
        sdfkjg = 1;
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
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
    
    
    
    
    angOptNewMat = angOptManager.angOptNewMat(validIdList, :);
    angOptGtMat = angOptManager.angOptGtMat(validIdList, :);
    angOptWeight = angOptManager.angOptWeight(validIdList, :);
    angOptWeight0 = angOptManager.angOptWeight0(validIdList, :);
        
    angOptWeightT = angOptWeight';
    angOptWeightNorm = (angOptWeightT./repmat(sum(angOptWeightT')',1,size(angOptWeightT,2)))';
    
    angOptWeightNorm(isnan(angOptWeightNorm)) = 0;
    
    
    angOptWeightT0 = angOptWeight0';
    angOptWeightNorm0 = (angOptWeightT0./repmat(sum(angOptWeightT0')',1,size(angOptWeightT0,2)))';
    
    angOptWeightNorm0(isnan(angOptWeightNorm0)) = 0;
    
    new_Diff = sum(angOptWeightNorm.*angOptNewMat)'; new_Diff0 = sum(angOptWeightNorm0.*angOptNewMat)';
    
    
    
    
    gt_Diff = sum(angOptWeightNorm.*angOptGtMat)'; gt_Diff0 = sum(angOptWeightNorm0.*angOptGtMat)';
    
    validFrm = 1:size(keyEndAng2,1);
    
    
    id11 = find(new_Diff(validFrm) == 0);
    
    [Id, Idd] = splitIndex2(id11);
    id1 = Id{1}';
    
    
    validFrm0 = validFrm;
    if ~isempty(id1)
        validFrm = validFrm(id1(end) + 1:end);
    end
    
    validFrm = validFrm(~isnan(new_Diff0(validFrm)) & new_Diff(validFrm) ~= 0);
    accumP2CRef_ = diff(accumP2CRef(1:end));
    accumP2CRef_(validFrm);
    
    
    Diff0 = [keyEndAng2(validFrm,1) [cumsum([new_Diff0(validFrm)  new_Diff(validFrm) gt_Diff0(validFrm)  gt_Diff(validFrm)  abs(accumP2CRef_(validFrm))], 1)  ]];
    %     Diff1 = [keyEndAng2(validFrm,1) [cumsum([new_Diff0(validFrm) gt_Diff0(validFrm) new_Diff(validFrm) gt_Diff(validFrm)])  accumP2CRef_(validFrm)]];
    err1 = rad2deg(Diff0(:,2:5) - Diff0(:,6)) ;
    keyEndAng22 = keyEndAng2(validFrm,:);
    keyEndAng222 = keyEndAng2(1:length(validFrm),:);
    
    
     [~, dispErrExpMat] = PlotDispErr(obj, 5,[1 1000000], 1);
    
     if flipWeight2
         figure(17),
     else
         figure(18);
     end
    clf;subplot(3,3,1);hold on;plot(Diff0(:,1),[err1(:,1) - err1(:,3)  err1(:,2) - err1(:,4)]);legend('short','long');
%     subplot(1,2,2);plot(keyEndAng2(:,1),rad2deg(keyEndAng2(:,2)));
    subplot(3,3,3);hold on;plot(keyEndAng22(:,1),rad2deg(keyEndAng22(:,2)));
    hold on;plot(keyEndAng22(:,1),rad2deg(keyEndAng222(:,3)));
    legend('short','long');
    if forceMaxK
        subplot(3,3,1);hold on;plot(Diff0(:,1), cumsum(rad2deg(keyEndAng22(:,2))),'x');plot(Diff0(:,1), cumsum(rad2deg(keyEndAng222(:,3))),'x');
    end
    
%     subplot(2,2,3);hold on;plot(Diff0(:,1),[err1(:,1) - err1(:,3)  err1(:,2) - err1(:,4)]);legend('short','long');
    subplot(3,3,4),hold on;plot(keyEndAng22(:,1),cumsum(rad2deg(keyEndAng22(:,2)))); 
                           plot(keyEndAng22(:,1),cumsum(rad2deg(keyEndAng22(:,3))));
                           plot(keyEndAng22(:,1),cumsum(rad2deg(keyEndAng22(:,3))) - cumsum(rad2deg(keyEndAng22(:,2))));
                           grid on;
                           legend('short','long', 'long - short');
    
%     subplot(1,2,2);plot(keyEndAng2(:,1),rad2deg(keyEndAng2(:,2)));
    subplot(3,3,6);hold on;
    plot(keyEndAng22(:,1),rad2deg(keyEndAng22(:,2)));
    hold on;plot(keyEndAng22(:,1),rad2deg(keyEndAng22(:,3)));
    grid on;
    if 0
        plot(dispErrExpMat(validFrm,1), zeros(1,length(validFrm)),'-k');hold on;plot(dispErrExpMat(validFrm,1),dispErrExpMat(validFrm,2),'-b');plot(dispErrExpMat(validFrm,1),dispErrExpMat(validFrm,3),'-g');
        legend('zero','k','avg','short','long');
    else
        legend('short','long');
    end
    
    subplot(3,3,2);hold on;
    plot(keyEndAng22(:,1),rad2deg(keyEndAng22(:,2)));
    hold on;plot(keyEndAng22(:,1),rad2deg(keyEndAng22(:,3)));
    if 1
        plot(dispErrExpMat(validFrm,1), zeros(1,length(validFrm)),'-k');hold on;plot(dispErrExpMat(validFrm,1),dispErrExpMat(validFrm,2),'-b');plot(dispErrExpMat(validFrm,1),dispErrExpMat(validFrm,3),'-g');
        legend('zero','k','avg','short','long');
    else
        legend('short','long');
    end
    
    
    
    ratio = 0.25;
    max_k = max(max(abs(keyEndAng22(:,2))));
    id = find(abs(keyEndAng22(:,2)) > ratio.*max_k);
    
    subplot(3,3,5),cla;hist(keyEndAng22(id,3)./keyEndAng22(id,2),10);
    
    
    if mean(accumP2CRef_) > 0
        subplot(3,3,7),plot(Diff0(:,1), err1(:,1:2) );hold on; plot(Diff0(:,1), err1(:,3:4) ,'-x');legend('short','long','short gt','long gt');title(num2str(frame_k));
    else
        subplot(3,3,7),plot(Diff0(:,1), -err1(:,1:2) );hold on; plot(Diff0(:,1), -err1(:,3:4) ,'-x');legend('short','long','short gt','long gt');title(num2str(frame_k));
    end
    
    
    
%     figure(5);subplot(2,5, [6 7]);hold on;plot(keyEndAng22(:,1), rad2deg(keyEndAng22(:,2)),'-r');plot(keyEndAng22(:,1), rad2deg(keyEndAng22(:,3)),'-k');
if flipWeight2
    saveas(gcf,fullfile(probPath,sprintf('final_error_%05d.fig',1999)));
else
    saveas(gcf,fullfile(probPath,sprintf('final_error_%05d.fig',1998)));
end

agkja = 1;

if 0
    
    figure,hold on;plot(zeros(1,size(dispErrExpMat,1)),'-k');hold on;plot(dispErrExpMat(:,1),dispErrExpMat(:,2),'-b');plot(dispErrExpMat(:,1),dispErrExpMat(:,3),'-g');
    plot(keyEndAng22(:,1), rad2deg(keyEndAng22(:,2)),'-r');plot(keyEndAng22(:,1), rad2deg(keyEndAng22(:,3)),'-k');legend('zero','k','avg','avg p2c', 'avg p2c 2');
    % plot(angErr_before_after(:,1),angErr_before_after(:,2),'-c');plot(angErr_before_after(:,1),angErr_before_after(:,3),'-r');plot(angErr_before_after__1(:,1),angErr_before_after__1(:,3),'-k');legend('zero','k','avg','k p2c','avg p2c', 'avg p2c 2');
end
if 0
    subplot(2,2,1),hold on;plot(keyEndAng22(:,1),cumsum(rad2deg(keyEndAng22(:,2))),'x');
    subplot(2,2,1),hold on;plot(keyEndAng22(:,1),cumsum(rad2deg(keyEndAng222(:,3))),'x');
end
if 0
    figure,plot(rad2deg([(Diff0(:,2) - Diff0(:,3)) (Diff0(:,4) - Diff0(:,5))    (Diff0(:,2) - Diff0(:,5))   (Diff0(:,4) - Diff0(:,5))]));legend('short','long');
    figure,plot(rad2deg([(Diff1(:,2) - Diff1(:,3)) (Diff1(:,4) - Diff1(:,5))]));legend('short','long');
    
    if mean(accumP2CRef_) > 0
        figure,plot(Diff0(:,1), err1(:,1:2) );hold on; plot(Diff0(:,1), err1(:,3:4) ,'-x');legend('short','long','short gt','long gt');title(num2str(frame_k));
    else
        figure,plot(Diff0(:,1), err1(:,1:2) );hold on; plot(Diff0(:,1), -err1(:,3:4) ,'-x');legend('short','long','short gt','long gt');title(num2str(frame_k));
    end
    figure,plot([err1(:,1) - err1(:,3)  err1(:,2) - err1(:,3)]);legend('short','long')
    
    
    figure, plot(keyEndAng22(:,1),cumsum(rad2deg(keyEndAng22(:,2:3))))
    
    
    
    
    
    if forceAngOpt
        figure,plot(rad2deg([cumsum(new_Diff0) - cumsum(accumP2CRef_(1:length(gt_Diff0)))  cumsum(gt_Diff0) - cumsum(accumP2CRef_(1:length(gt_Diff0)))]));
        figure(5),subplot(2,5,[1 2]);hold on;plot([[0 0];rad2deg([cumsum(new_Diff0) - cumsum(accumP2CRef_(1:length(gt_Diff0)))  cumsum(gt_Diff0) - cumsum(accumP2CRef_(1:length(gt_Diff0)))])],'-m');
        
        figure,plot(rad2deg(gt_Diff0(validFrm) - accumP2CRef_(validFrm)))
        figure,plot(rad2deg(gt_Diff(validFrm) - accumP2CRef_(validFrm)))
    end
    
    
end

    asghu = 1;
    

else
    dispErrMatDetailed_before = [];
    dispErrMatDetailed_after = [];
    dispErrMatDetailed_gt = [];
    keyEndAng = [];
end


end