if doProb
    LOnly = USELEFT;
    drawProb = 0;1;
    %     drawPorb1 = 0;
    depthGTInd;
    ProbZTmpTmp_norm;
    ProbZTmp_norm;
    idMax;
    
    depthGTInd_update;
    ProbZTmpTmp_update_norm;
    ProbZTmp_update_norm;
    idMax_update;
    
    ValidFeatFlagTmp = ValidFeatFlag;
    % flag2CheckId = find(ProbZTmpTmp_norm >0.8 & ProbZTmpTmp_update_norm < 0.65);
    % flag2CheckId = find(ProbZTmpTmp_norm > 1.1.*ProbZTmpTmp_update_norm & ProbZTmpTmp_norm > 0.8);
    % Flag2CheckId = find(ismember(inlierId, checkId));
    Flag2CheckId = checkId;
    
    updatedProbZSum_2 = updatedProbZSum_./max(updatedProbZSum_(:));
    figure(61),clf;subplot(1,2,1);plot(updatedProbZSum_2(checkId(1:length(checkId)/2),:)');hold on;plot(depthGTInd_update(checkId(1:length(checkId)/2)), updatedProbZSum_2(depthGTIndAll_update(checkId(1:length(checkId)/2))),'*r');plot(idMax_update(checkId(1:length(checkId)/2)), updatedProbZSum_2(depthUpdateIndAll_update(checkId(1:length(checkId)/2))),'*b');title('low');
    subplot(1,2,2);plot(updatedProbZSum_2(checkId(length(checkId)/2 + 1:end),:)');hold on;plot(depthGTInd_update(checkId(length(checkId)/2 + 1 : end)), updatedProbZSum_2(depthGTIndAll_update(checkId(length(checkId)/2 + 1 : end))),'*r'); plot(idMax_update(checkId(length(checkId)/2 + 1 : end)), updatedProbZSum_2(depthUpdateIndAll_update(checkId(length(checkId)/2 + 1 : end))),'*b');title('high');
    
    
    
    % idMTmp = idM;
    scal = 100;
    if 1 %drawPorb1
        figure(54),clf;imshow(imgCur); hold on;title('L*R','Color','k');
    end
    PercentageGT = [];
    Buf = {};BufL = {};BufR = {};
    TravelFlagIn = []; TravelFlagIn2 = []; Errrrr = [];
    if 1 %drawPorb1
        figure(52),clf;subplot(1,2,1);imshow(imgCur);hold on;
        subplot(1,2,2);imshow(imgCurR);hold on;
    end
    checkIdPtGT = [];
    for yu = 1 : length(Flag2CheckId)
        flag2CheckId = Flag2CheckId(yu);
        %         if drawPorb1
        %             figure(52),clf;subplot(1,2,1);imshow(imgCur);hold on;
        %             subplot(1,2,2);imshow(imgCurR);hold on;
        %         end
        buf = []; bufL = []; bufR = [];
        ptGTL = []; ptGTR = [];
        ptPeakL = []; ptPeakR = [];
        
        travelPixXMat = []; travelPixYMat = [];
        travelPixXMatR = []; travelPixYMatR = [];
        
        
        idM_Vec = find(ValidFeatFlagTmp(:,flag2CheckId) > 0)' ;
        if ~CUTTHETAANDFORCEPLATFORM
            if 0
                idM_Vec = find(thetaProb > 0.0001) ;
            elseif 0
                idM_Vec = find(thetaProb./max(thetaProb) > obj.configParam.theta_prob_cutoff_threshold) ;
            else
                %             ValidFeatFlagTmp
                idM_Vec = find(ValidFeatFlagTmp(:,flag2CheckId))' ;
            end
        else
%             idM_Vec = find(thetaProb./max(thetaProb) > obj.configParam.theta_prob_cutoff_threshold) ;
            idM_Vec = find(ValidFeatFlagTmp(:,flag2CheckId))' ;

            
        end
        ValidFeatFlagTmp = zeros(size(ValidFeatFlag));
        ValidFeatFlagTmp(idM_Vec,:) = 1;
        pix11L = []; pix11R = [];
        for jh = idM_Vec  %1:length(ProbReprojVecR) % size(updatedProbZSum_2,2) %idM_Vec
            idMTmp = jh;  % + idM;
            
            %         for jh = idM_Vec
            %             idMTmp = jh + idM;
            for ui = 1 : length(flag2CheckId)
                
                buf = [buf; ((ProbReprojVecR{idMTmp}(flag2CheckId(ui),:) .* ProbReprojVec{idMTmp}(flag2CheckId(ui),:)).*500)];
                bufL = [bufL; ((ProbReprojVec{idMTmp}(flag2CheckId(ui),:)))];
                bufR = [bufR; ((ProbReprojVecR{idMTmp}(flag2CheckId(ui),:)))];
                
                if drawProb
                    figure(51),clf;plot(ProbZTmp_norm(flag2CheckId(ui),:)./scal);hold on;plot([ProbReprojVec{idMTmp}(flag2CheckId(ui),:);ProbReprojVecR{idMTmp}(flag2CheckId(ui),:); (ProbReprojVecR{idMTmp}(flag2CheckId(ui),:) + ProbReprojVec{idMTmp}(flag2CheckId(ui),:)); (ProbReprojVecR{idMTmp}(flag2CheckId(ui),:) .* ProbReprojVec{idMTmp}(flag2CheckId(ui),:)).*500]')
                    
                    tmpVec1 = zeros(1, size(ProbZTmp_norm, 2)); tmpVec1(depthGTInd(flag2CheckId(ui))) = ProbZTmp_norm(flag2CheckId(ui),idMax(flag2CheckId(ui)))./scal;
                    tmpVec2 = zeros(1, size(ProbZTmp_norm, 2)); tmpVec2(idMax(flag2CheckId(ui))) = ProbZTmp_norm(flag2CheckId(ui),idMax(flag2CheckId(ui)))./scal;
                    tmpVec3 = zeros(1, size(ProbZTmp_norm, 2)); tmpVec3(idMax_update(flag2CheckId(ui))) = ProbZTmp_update_norm(flag2CheckId(ui),idMax_update(flag2CheckId(ui)))./scal;
                    plot([tmpVec1; tmpVec2; tmpVec3]');
                    plot(ProbZTmp_update_norm(flag2CheckId(ui),:)./scal);
                    %                 legend('initDepth','LProj','RProj','L + R','L * R','depthGT','initDepthMax','updateDepthMax','updateDepth')
                    plot(depthGTInd(flag2CheckId(ui)), 1./scal,'*b');
                    title(sprintf('theta peak: %d\nfeature id: %d\ntheta id: %d\ntheta prob: %0.5f\nsigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f ',idM,(flag2CheckId(ui)),idMTmp, thetaProb(idMTmp),obj.configParam.reproj_sigma,obj.configParam.reproj_beta,obj.configParam.reproj_sigma_right,obj.configParam.reproj_beta_right));
                end
                %                 title(sprintf('sigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f',obj.configParam.reproj_sigma,obj.configParam.reproj_beta,obj.configParam.reproj_sigma_right,obj.configParam.reproj_beta_right));
                pixSpanList = PixKeyVecTmp{idMTmp};
                pixSpanMatX = reshape(pixSpanList(:,1), size(ProbZTmp_norm));
                pixSpanMatY = reshape(pixSpanList(:,2), size(ProbZTmp_norm));
                
                pixSpanListR = PixKeyVecRTmp{idMTmp};
                pixSpanMatXR = reshape(pixSpanListR(:,1), size(ProbZTmp_norm));
                pixSpanMatYR = reshape(pixSpanListR(:,2), size(ProbZTmp_norm));
                
                
                travelPixXMat = [travelPixXMat;pixSpanMatX(flag2CheckId(ui),:)];
                travelPixYMat = [travelPixYMat;pixSpanMatY(flag2CheckId(ui),:)];
                
                travelPixXMatR = [travelPixXMatR;pixSpanMatXR(flag2CheckId(ui),:)];
                travelPixYMatR = [travelPixYMatR;pixSpanMatYR(flag2CheckId(ui),:)];
                
                %                 figure(52),clf;
                
                err1 = ([pixSpanMatX(flag2CheckId(ui),end), pixSpanMatY(flag2CheckId(ui),end)] - [pt2dCur(flag2CheckId(ui),1), pt2dCur(flag2CheckId(ui),2)]);
                err2 = ([pixSpanMatX(flag2CheckId(ui),1), pixSpanMatY(flag2CheckId(ui),1)] - [pt2dCur(flag2CheckId(ui),1), pt2dCur(flag2CheckId(ui),2)]);
                ptGTL = [ptGTL; [pixSpanMatX(flag2CheckId(ui),depthGTInd(flag2CheckId(ui))), pixSpanMatY(flag2CheckId(ui),depthGTInd(flag2CheckId(ui)))]];
                ptPeakL = [ptPeakL; [pixSpanMatX(flag2CheckId(ui),idMax_update(flag2CheckId(ui))), pixSpanMatY(flag2CheckId(ui),idMax_update(flag2CheckId(ui)))]];
                if sign(err1) == sign(err2)
                    
                else
                    
                end
                
                
                err1R = ([pixSpanMatXR(flag2CheckId(ui),end), pixSpanMatYR(flag2CheckId(ui),end)] - [pt2dCurR(flag2CheckId(ui),1), pt2dCurR(flag2CheckId(ui),2)]);
                err2R = ([pixSpanMatXR(flag2CheckId(ui),1), pixSpanMatYR(flag2CheckId(ui),1)] - [pt2dCurR(flag2CheckId(ui),1), pt2dCurR(flag2CheckId(ui),2)]);
                if sign(err1R) == sign(err2R)
                    
                else
                    
                end
                
                
                
                
                ptGTR = [ptGTR; pixSpanMatXR(flag2CheckId(ui),depthGTInd(flag2CheckId(ui))), pixSpanMatYR(flag2CheckId(ui),depthGTInd(flag2CheckId(ui)))];
                ptPeakR = [ptPeakR; pixSpanMatXR(flag2CheckId(ui),idMax_update(flag2CheckId(ui))), pixSpanMatYR(flag2CheckId(ui),idMax_update(flag2CheckId(ui)))];
                
                %                 if drawProb
                if jh == idM_Vec(1) || jh == idM_Vec(end)
                    figure(52);
                    subplot(1,2,1);%imshow(imgCur);
                    
                    
                    hold on;plot(pixSpanMatX(flag2CheckId(ui),:), pixSpanMatY(flag2CheckId(ui),:),'.g');plot(pixSpanMatX(flag2CheckId(ui),end), pixSpanMatY(flag2CheckId(ui),end),'xg');plot(pixSpanMatX(flag2CheckId(ui),depthGTInd(flag2CheckId(ui))), pixSpanMatY(flag2CheckId(ui),depthGTInd(flag2CheckId(ui))),'xy');plot(pt2dCur(flag2CheckId(ui),1), pt2dCur(flag2CheckId(ui),2),'.r');plot(pixGT(flag2CheckId(ui),1), pixGT(flag2CheckId(ui),2),'.b');title(sprintf('%0.5f\n%0.5f\n%0.5f %0.5f\n%0.5f %0.5f',ValidFeatFlagTmp(idMTmp,flag2CheckId(ui)),(norm(pt2dCur(flag2CheckId(ui),:) - pixGT(flag2CheckId(ui),:))),err1(1),err1(2),err2(1),err2(2)));
                    subplot(1,2,2);% imshow(imgCurR);
                    hold on;plot(pixSpanMatXR(flag2CheckId(ui),:), pixSpanMatYR(flag2CheckId(ui),:),'.g');plot(pixSpanMatXR(flag2CheckId(ui),end), pixSpanMatYR(flag2CheckId(ui),end),'xg');plot(pixSpanMatXR(flag2CheckId(ui),depthGTInd(flag2CheckId(ui))), pixSpanMatYR(flag2CheckId(ui),depthGTInd(flag2CheckId(ui))),'xy');plot(pt2dCurR(flag2CheckId(ui),1), pt2dCurR(flag2CheckId(ui),2),'.r');plot(pixGTR(flag2CheckId(ui),1), pixGTR(flag2CheckId(ui),2),'.b');title(sprintf('%0.5f\n%0.5f %0.5f\n%0.5f %0.5f',(norm(pt2dCurR(flag2CheckId(ui),:) - pixGTR(flag2CheckId(ui),:))),err1R(1),err1R(2),err2R(1),err2R(2)));
                    drawnow;
                    
                    pix11L = [pix11L [pixSpanMatX(flag2CheckId(ui),depthGTInd(flag2CheckId(ui))), pixSpanMatY(flag2CheckId(ui),depthGTInd(flag2CheckId(ui)))]];
                    pix11R = [pix11R [pixSpanMatXR(flag2CheckId(ui),depthGTInd(flag2CheckId(ui))), pixSpanMatYR(flag2CheckId(ui),depthGTInd(flag2CheckId(ui)))]];
                end
                
                
                
                
                tempProb1 = ProbReprojVecAll(:,flag2CheckId(ui),:).*thetaProbAll(:,flag2CheckId(ui),:);
                tempProb1 = permute(tempProb1, [1 3 2]);
                tempProb1_New = ProbReprojVecAll_New(:,flag2CheckId(ui),:).*thetaProbAll(:,flag2CheckId(ui),:);
                tempProb1_New = permute(tempProb1_New, [1 3 2]);
                
                tempProb2 = thetaProbAll(:,flag2CheckId(ui),:).*ProbReprojVecRAll(:,flag2CheckId(ui),:);
                tempProb2 = permute(tempProb2, [1 3 2]);
                tempProb2_New = thetaProbAll(:,flag2CheckId(ui),:).*ProbReprojVecRAll_New(:,flag2CheckId(ui),:);
                tempProb2_New = permute(tempProb2_New, [1 3 2]);
                
                tempProb3 = ProbReprojVecAll(:,flag2CheckId(ui),:).*thetaProbAll(:,flag2CheckId(ui),:).*ProbReprojVecRAll(:,flag2CheckId(ui),:);
                tempProb3 = permute(tempProb3, [1 3 2]);
                %                 tempProb3_New = ProbReprojVecAll_New(:,flag2CheckId(ui),:).*thetaProbAll(:,flag2CheckId(ui),:).*ProbReprojVecRAll_New(:,flag2CheckId(ui),:);
                tempProb3_New = ProbReprojVecLRAll_New(:,flag2CheckId(ui),:).*thetaProbAll(:,flag2CheckId(ui),:);
                tempProb3_New = permute(tempProb3_New, [1 3 2]);
                tempProb3_New2 = ProbReprojVecAll(:,flag2CheckId(ui),:).*thetaProbAll_New(:,flag2CheckId(ui),:).*ProbReprojVecRAll(:,flag2CheckId(ui),:);
                tempProb3_New2 = permute(tempProb3_New2, [1 3 2]);
                
                
                tempProb4 = thetaProbAll(:,flag2CheckId(ui),:).*(ProbReprojVecAll(:,flag2CheckId(ui),:) + ProbReprojVecRAll(:,flag2CheckId(ui),:));
                tempProb4 = permute(tempProb4, [1 3 2]);
                tempProb4_New = thetaProbAll(:,flag2CheckId(ui),:).*(ProbReprojVecAll_New(:,flag2CheckId(ui),:) + ProbReprojVecRAll_New(:,flag2CheckId(ui),:));
                tempProb4_New = permute(tempProb4_New, [1 3 2]);
                
                if drawProb
                    figure(51),hold on;plot(sum(tempProb3)./max(sum(tempProb3))./scal,'-oc');
                    figure(51),hold on;plot(sum(tempProb3_New)./max(sum(tempProb3_New))./scal,'-ok');
                    legend('initDepth','LProj','RProj','L + R','L * R','depthGT','initDepthMax','updateDepthMax','updateDepth','gtInd','L*R*Theta','L*R*Theta(new)')
                end
                
                t1 = sum(tempProb3)./max(sum(tempProb3)).*ProbZTmp_norm(flag2CheckId(ui),:);
                t1 = t1./max(t1);
                t2 = ProbZTmp_update_norm(flag2CheckId(ui),:)./max(ProbZTmp_update_norm(flag2CheckId(ui),:));
                t1_New = sum(tempProb3_New)./max(sum(tempProb3_New)).*ProbZTmp_norm(flag2CheckId(ui),:);
                t1_New = t1_New./max(t1_New);
                t2_New = ProbZTmp_update_norm(flag2CheckId(ui),:)./max(ProbZTmp_update_norm(flag2CheckId(ui),:));
                
                t1_New2 = sum(tempProb3_New2)./max(sum(tempProb3_New2)).*ProbZTmp_norm(flag2CheckId(ui),:);
                t1_New2 = t1_New./max(t1_New2);
                t2_New2 = ProbZTmp_update_norm(flag2CheckId(ui),:)./max(ProbZTmp_update_norm(flag2CheckId(ui),:));
                
                if 0
                    figure,plot(t1-t2);
                    figure,plot([t1;t2]','-x');
                    figure,plot(t1_New - t2_New);
                    figure,plot([t1_New;t2_New]','-x');
                    figure,plot(t1_New2 - t2_New2);
                    figure,plot([t1_New2;t2_New2]','-x');
                end
                
                if 0
                    if ismember(idM, idM_Vec)
                        plotProbZId = idM;
                    elseif abs(idM_Vec(1) - idM) > abs(idM_Vec(end) - idM)
                        plotProbZId = idM_Vec(end);
                    else
                        plotProbZId = idM_Vec(1);
                    end
                else
                    
                    plotProbZId = (length(thetaSamp)+1)/2;
                    
                end
                
                if jh == plotProbZId
                    ptSpanL = [pixSpanMatX(flag2CheckId(ui),:); pixSpanMatY(flag2CheckId(ui),:)]';
                    ptSpanR = [pixSpanMatXR(flag2CheckId(ui),:); pixSpanMatYR(flag2CheckId(ui),:)]';
                    ptGTListL = [pixSpanMatX(flag2CheckId(ui),depthGTInd(flag2CheckId(ui))), pixSpanMatY(flag2CheckId(ui),depthGTInd(flag2CheckId(ui)))];
                    ptGTListR = [pixSpanMatXR(flag2CheckId(ui),depthGTInd(flag2CheckId(ui))), pixSpanMatYR(flag2CheckId(ui),depthGTInd(flag2CheckId(ui)))];
                    
                    figure(52);subplot(1,2,1);plot(ptGTListL(1), ptGTListL(2),'xw');text(double(ptGTListL(:,1)), double(ptGTListL(:,2)) + 5,strcat(num2str(yu),'--',num2str(tracebackId(yu))),'Color','m');
                    subplot(1,2,2);plot(ptGTListR(1), ptGTListR(2),'xw');text(double(ptGTListR(:,1)), double(ptGTListR(:,2)) + 5,strcat(num2str(yu),'--',num2str(tracebackId(yu))),'Color','m');
                    
                    
                    
                    initDepthProb = 3.*(ProbZTmp_norm(flag2CheckId(ui),:)./max(ProbZTmp_norm(flag2CheckId(ui),:)));
                    if ~LOnly
                        LRTheta = 3.*(sum(tempProb3_New)./max(sum(tempProb3_New)));
                    else
                        LRTheta = 3.*(sum(tempProb1_New)./max(sum(tempProb1_New)));
                    end
                    LRTheta2 = 3.*(sum(tempProb3_New2)./max(sum(tempProb3_New2)));
                    tmpCheck = initDepthProb.*LRTheta; tmpCheck = tmpCheck./max(tmpCheck);
                    tmpCheck2 = initDepthProb.*LRTheta2; tmpCheck2 = tmpCheck2./max(tmpCheck2);
                    updatedDepthProb = 3.*(ProbZTmp_update_norm(flag2CheckId(ui),:)./max(ProbZTmp_update_norm(flag2CheckId(ui),:)));
                    %                     updatedDepthProb = 3.*(ProbZTmp_update_norm(flag2CheckId(ui),:)./max(ProbZTmp_update_norm(flag2CheckId(ui),:)));
                    gtId = depthGTInd(flag2CheckId(ui));
                    if 0
                        figure,plot([tmpCheck./max(tmpCheck) - updatedDepthProb./max(updatedDepthProb);tmpCheck2./max(tmpCheck2) - updatedDepthProb./max(updatedDepthProb)]');
                    end
                    
                    wgbhj = 1;
                    
                end
                
                
                
                % % % % % % % % % % % % % % % %                 if ismember(idM, idM_Vec)
                % % % % % % % % % % % % % % % %                     if jh == idM
                % % % % % % % % % % % % % % % %                         ptSpanL = [pixSpanMatX(flag2CheckId(ui),:); pixSpanMatY(flag2CheckId(ui),:)]';
                % % % % % % % % % % % % % % % %                         ptSpanR = [pixSpanMatXR(flag2CheckId(ui),:); pixSpanMatYR(flag2CheckId(ui),:)]';
                % % % % % % % % % % % % % % % %                         ptGTListL = [pixSpanMatX(flag2CheckId(ui),depthGTInd(flag2CheckId(ui))), pixSpanMatY(flag2CheckId(ui),depthGTInd(flag2CheckId(ui)))];
                % % % % % % % % % % % % % % % %                         ptGTListR = [pixSpanMatXR(flag2CheckId(ui),depthGTInd(flag2CheckId(ui))), pixSpanMatYR(flag2CheckId(ui),depthGTInd(flag2CheckId(ui)))];
                % % % % % % % % % % % % % % % %                         initDepthProb = 3.*(ProbZTmp_norm(flag2CheckId(ui),:)./max(ProbZTmp_norm(flag2CheckId(ui),:)));
                % % % % % % % % % % % % % % % %                         LRTheta = 3.*(sum(tempProb3_New)./max(sum(tempProb3_New)));
                % % % % % % % % % % % % % % % %                         tmpCheck = initDepthProb.*LRTheta; tmpCheck = tmpCheck./max(tmpCheck);
                % % % % % % % % % % % % % % % %                         updatedDepthProb = 3.*(ProbZTmp_update_norm(flag2CheckId(ui),:)./max(ProbZTmp_update_norm(flag2CheckId(ui),:)));
                % % % % % % % % % % % % % % % %                         gtId = depthGTInd(flag2CheckId(ui));
                % % % % % % % % % % % % % % % %                         if 0
                % % % % % % % % % % % % % % % %                             figure,plot(tmpCheck./max(tmpCheck) - updatedDepthProb./max(updatedDepthProb));
                % % % % % % % % % % % % % % % %                         end
                % % % % % % % % % % % % % % % %
                % % % % % % % % % % % % % % % %
                % % % % % % % % % % % % % % % %
                % % % % % % % % % % % % % % % %                     end
                % % % % % % % % % % % % % % % %                 elseif abs(idM_Vec(1) - idM) > abs(idM_Vec(end) - idM)
                % % % % % % % % % % % % % % % %                     if jh == idM_Vec(end)
                % % % % % % % % % % % % % % % %                         ptSpanL = [pixSpanMatX(flag2CheckId(ui),:); pixSpanMatY(flag2CheckId(ui),:)]';
                % % % % % % % % % % % % % % % %                         ptSpanR = [pixSpanMatXR(flag2CheckId(ui),:); pixSpanMatYR(flag2CheckId(ui),:)]';
                % % % % % % % % % % % % % % % %                         ptGTListL = [pixSpanMatX(flag2CheckId(ui),depthGTInd(flag2CheckId(ui))), pixSpanMatY(flag2CheckId(ui),depthGTInd(flag2CheckId(ui)))];
                % % % % % % % % % % % % % % % %                         ptGTListR = [pixSpanMatXR(flag2CheckId(ui),depthGTInd(flag2CheckId(ui))), pixSpanMatYR(flag2CheckId(ui),depthGTInd(flag2CheckId(ui)))];
                % % % % % % % % % % % % % % % %                         initDepthProb = 3.*(ProbZTmp_norm(flag2CheckId(ui),:)./max(ProbZTmp_norm(flag2CheckId(ui),:)));
                % % % % % % % % % % % % % % % %                         LRTheta = 3.*(sum(tempProb3_New)./max(sum(tempProb3_New)));
                % % % % % % % % % % % % % % % %                         tmpCheck = initDepthProb.*LRTheta; tmpCheck = tmpCheck./max(tmpCheck);
                % % % % % % % % % % % % % % % %                         updatedDepthProb = 3.*(ProbZTmp_update_norm(flag2CheckId(ui),:)./max(ProbZTmp_update_norm(flag2CheckId(ui),:)));
                % % % % % % % % % % % % % % % %                         gtId = depthGTInd(flag2CheckId(ui));
                % % % % % % % % % % % % % % % %                         if 0
                % % % % % % % % % % % % % % % %                             figure,plot(tmpCheck./max(tmpCheck) - updatedDepthProb./max(updatedDepthProb));
                % % % % % % % % % % % % % % % %                         end
                % % % % % % % % % % % % % % % %
                % % % % % % % % % % % % % % % %
                % % % % % % % % % % % % % % % %
                % % % % % % % % % % % % % % % %                     end
                % % % % % % % % % % % % % % % %                 else
                % % % % % % % % % % % % % % % %                     if jh == idM_Vec(1)
                % % % % % % % % % % % % % % % %                         ptSpanL = [pixSpanMatX(flag2CheckId(ui),:); pixSpanMatY(flag2CheckId(ui),:)]';
                % % % % % % % % % % % % % % % %                         ptSpanR = [pixSpanMatXR(flag2CheckId(ui),:); pixSpanMatYR(flag2CheckId(ui),:)]';
                % % % % % % % % % % % % % % % %                         ptGTListL = [pixSpanMatX(flag2CheckId(ui),depthGTInd(flag2CheckId(ui))), pixSpanMatY(flag2CheckId(ui),depthGTInd(flag2CheckId(ui)))];
                % % % % % % % % % % % % % % % %                         ptGTListR = [pixSpanMatXR(flag2CheckId(ui),depthGTInd(flag2CheckId(ui))), pixSpanMatYR(flag2CheckId(ui),depthGTInd(flag2CheckId(ui)))];
                % % % % % % % % % % % % % % % %                         initDepthProb = 3.*(ProbZTmp_norm(flag2CheckId(ui),:)./max(ProbZTmp_norm(flag2CheckId(ui),:)));
                % % % % % % % % % % % % % % % %                         LRTheta = 3.*(sum(tempProb3_New)./max(sum(tempProb3_New)));
                % % % % % % % % % % % % % % % %                         tmpCheck = initDepthProb.*LRTheta; tmpCheck = tmpCheck./max(tmpCheck);
                % % % % % % % % % % % % % % % %                         updatedDepthProb = 3.*(ProbZTmp_update_norm(flag2CheckId(ui),:)./max(ProbZTmp_update_norm(flag2CheckId(ui),:)));
                % % % % % % % % % % % % % % % %                         gtId = depthGTInd(flag2CheckId(ui));
                % % % % % % % % % % % % % % % %                         if 0
                % % % % % % % % % % % % % % % %                             figure,plot(tmpCheck./max(tmpCheck) - updatedDepthProb./max(updatedDepthProb));
                % % % % % % % % % % % % % % % %                         end
                % % % % % % % % % % % % % % % %
                % % % % % % % % % % % % % % % %
                % % % % % % % % % % % % % % % %
                % % % % % % % % % % % % % % % %                     end
                % % % % % % % % % % % % % % % %
                % % % % % % % % % % % % % % % %
                % % % % % % % % % % % % % % % %                 end
                
                if drawPorb1
                    if jh == idM_Vec(1)
                        figure(55),clf;subplot(1,4,1);plot(tempProb1'./max(sum(tempProb1)));hold on;plot(sum(tempProb1)./max(sum(tempProb1)),'-x');title('L*theta');plot(ProbZTmp_norm(flag2CheckId(ui),:)./max(ProbZTmp_norm(flag2CheckId(ui),:)),'-xb');plot(depthGTInd(flag2CheckId(ui)), ProbZTmp_norm(flag2CheckId(ui),depthGTInd(flag2CheckId(ui)))./max(ProbZTmp_norm(flag2CheckId(ui),:)),'*g');title('L * theta');
                        subplot(1,4,2);plot(tempProb2'./max(sum(tempProb2)));hold on;plot(sum(tempProb2)./max(sum(tempProb2)),'-x');title('R*theta');plot(ProbZTmp_norm(flag2CheckId(ui),:)./max(ProbZTmp_norm(flag2CheckId(ui),:)),'-xb');plot(depthGTInd(flag2CheckId(ui)), ProbZTmp_norm(flag2CheckId(ui),depthGTInd(flag2CheckId(ui)))./max(ProbZTmp_norm(flag2CheckId(ui),:)),'*g');title('R * theta');
                        subplot(1,4,3);plot(tempProb3'./max(sum(tempProb3)));hold on;plot(sum(tempProb3)./max(sum(tempProb3)),'-x');title('L*R*theta');plot(ProbZTmp_norm(flag2CheckId(ui),:)./max(ProbZTmp_norm(flag2CheckId(ui),:)),'-xb');plot(depthGTInd(flag2CheckId(ui)), ProbZTmp_norm(flag2CheckId(ui),depthGTInd(flag2CheckId(ui)))./max(ProbZTmp_norm(flag2CheckId(ui),:)),'*g');title('L * R * theta');
                        subplot(1,4,4);plot(tempProb4'./max(sum(tempProb4)));hold on;plot(sum(tempProb4)./max(sum(tempProb4)),'-x');title('(L+R)*theta');plot(ProbZTmp_norm(flag2CheckId(ui),:)./max(ProbZTmp_norm(flag2CheckId(ui),:)),'-xb');plot(depthGTInd(flag2CheckId(ui)), ProbZTmp_norm(flag2CheckId(ui),depthGTInd(flag2CheckId(ui)))./max(ProbZTmp_norm(flag2CheckId(ui),:)),'*g');title('(L + R) * theta');
                        
                        figure(56),clf;subplot(1,4,1);plot(tempProb1_New'./max(sum(tempProb1_New)));hold on;plot(sum(tempProb1_New)./max(sum(tempProb1_New)),'-x');title('L*theta');plot(ProbZTmp_norm(flag2CheckId(ui),:)./max(ProbZTmp_norm(flag2CheckId(ui),:)),'-xb');plot(depthGTInd(flag2CheckId(ui)), ProbZTmp_norm(flag2CheckId(ui),depthGTInd(flag2CheckId(ui)))./max(ProbZTmp_norm(flag2CheckId(ui),:)),'*g');title('L * theta');
                        subplot(1,4,2);plot(tempProb2_New'./max(sum(tempProb2_New)));hold on;plot(sum(tempProb2_New)./max(sum(tempProb2_New)),'-x');title('R*theta');plot(ProbZTmp_norm(flag2CheckId(ui),:)./max(ProbZTmp_norm(flag2CheckId(ui),:)),'-xb');plot(depthGTInd(flag2CheckId(ui)), ProbZTmp_norm(flag2CheckId(ui),depthGTInd(flag2CheckId(ui)))./max(ProbZTmp_norm(flag2CheckId(ui),:)),'*g');title('R * theta');
                        subplot(1,4,3);plot(tempProb3_New'./max(sum(tempProb3_New)));hold on;plot(sum(tempProb3_New)./max(sum(tempProb3_New)),'-x');title('L*R*theta');plot(ProbZTmp_norm(flag2CheckId(ui),:)./max(ProbZTmp_norm(flag2CheckId(ui),:)),'-xb');plot(depthGTInd(flag2CheckId(ui)), ProbZTmp_norm(flag2CheckId(ui),depthGTInd(flag2CheckId(ui)))./max(ProbZTmp_norm(flag2CheckId(ui),:)),'*g');title('L * R * theta');
                        subplot(1,4,4);plot(tempProb4_New'./max(sum(tempProb4_New)));hold on;plot(sum(tempProb4_New)./max(sum(tempProb4_New)),'-x');title('(L+R)*theta');plot(ProbZTmp_norm(flag2CheckId(ui),:)./max(ProbZTmp_norm(flag2CheckId(ui),:)),'-xb');plot(depthGTInd(flag2CheckId(ui)), ProbZTmp_norm(flag2CheckId(ui),depthGTInd(flag2CheckId(ui)))./max(ProbZTmp_norm(flag2CheckId(ui),:)),'*g');title('(L + R) * theta');
                    end
                end
                zUpdt1 = sum(tempProb1)./max(sum(tempProb1)).*ProbZTmp_norm(flag2CheckId(ui),:)./max(ProbZTmp_norm(flag2CheckId(ui),:));
                zUpdt1 = zUpdt1./max(zUpdt1);
                
                
                zUpdt2 = sum(tempProb2)./max(sum(tempProb2)).*ProbZTmp_norm(flag2CheckId(ui),:)./max(ProbZTmp_norm(flag2CheckId(ui),:));
                zUpdt2 = zUpdt2./max(zUpdt2);
                zUpdt2(depthGTInd(flag2CheckId(ui)));
                
                zUpdt3 = sum(tempProb3)./max(sum(tempProb3)).*ProbZTmp_norm(flag2CheckId(ui),:)./max(ProbZTmp_norm(flag2CheckId(ui),:));
                zUpdt3 = zUpdt3./max(zUpdt3);
                zUpdt3(depthGTInd(flag2CheckId(ui)));
                percentageGT = [zUpdt1(depthGTInd(flag2CheckId(ui))) zUpdt2(depthGTInd(flag2CheckId(ui))) zUpdt3(depthGTInd(flag2CheckId(ui)))];
                PercentageGT = [PercentageGT; percentageGT];
                
                checkId0 = flag2CheckId(ui);
                tempProb11 = ProbReprojVecAll(:,checkId0,:);
                tempProb11 = permute(tempProb11, [1 3 2]);
                tempProb11_New = ProbReprojVecAll_New(:,checkId0,:);
                tempProb11_New = permute(tempProb11_New, [1 3 2]);
                
                
                tempProb22 = ProbReprojVecRAll(:,checkId0,:);
                tempProb22 = permute(tempProb22, [1 3 2]);
                tempProb22_New = ProbReprojVecRAll_New(:,checkId0,:);
                tempProb22_New = permute(tempProb22_New, [1 3 2]);
                
                
                tempProb33 = ProbReprojVecRAll(:,checkId0,:).*ProbReprojVecAll(:,checkId0,:);
                tempProb33 = permute(tempProb33, [1 3 2]);
                tempProb33_New = ProbReprojVecLRAll_New(:,checkId0,:);
                tempProb33_New = permute(tempProb33_New, [1 3 2]);
                tempProb33_New2 = ProbReprojVecRAll(:,checkId0,:).*ProbReprojVecAll(:,checkId0,:);
                tempProb33_New2 = permute(tempProb33_New2, [1 3 2]);
                %                 tempProb33_New2 = ProbReprojVecLRAll(:,checkId0,:);
                %                 tempProb33_New2 = permute(tempProb33, [1 3 2]);
                
                
                
                
                tempProb44 = ProbReprojVecRAll(:,checkId0,:) + ProbReprojVecAll(:,checkId0,:);
                tempProb44 = permute(tempProb44, [1 3 2]);
                
                
                checkProbL = [tempProb11./max(tempProb11(:)) thetaProb'./max(thetaProb)];
                checkProbL(1,depthGTInd(flag2CheckId(ui))) = 1;
                checkProbL = [checkProbL; [sum(tempProb1)./max(sum(tempProb1)) 0];[ProbZTmp_norm(flag2CheckId(ui),:)./max(ProbZTmp_norm(flag2CheckId(ui),:)) 0];[t2 0]];
                checkProbL_New = [tempProb11_New./max(tempProb11_New(:)) thetaProb'./max(thetaProb) ValidFeatFlagTmp(:,checkId0)];
                checkProbL_New(1,depthGTInd(flag2CheckId(ui))) = 1;
                checkProbL_New = [checkProbL_New; [sum(tempProb1)./max(sum(tempProb1)) 0 0];[ProbZTmp_norm(flag2CheckId(ui),:)./max(ProbZTmp_norm(flag2CheckId(ui),:)) 0 0];[t2 0 0]];
                
                
                checkProbR = [tempProb22./max(tempProb22(:)) thetaProb'./max(thetaProb)];
                checkProbR(1,depthGTInd(flag2CheckId(ui))) = 1;
                checkProbR = [checkProbR; [sum(tempProb2)./max(sum(tempProb2)) 0];[ProbZTmp_norm(flag2CheckId(ui),:)./max(ProbZTmp_norm(flag2CheckId(ui),:)) 0];[t2 0]];
                checkProbR_New = [tempProb22_New./max(tempProb22_New(:)) thetaProb'./max(thetaProb)];
                checkProbR_New(1,depthGTInd(flag2CheckId(ui))) = 1;
                checkProbR_New = [checkProbR_New; [sum(tempProb2)./max(sum(tempProb2)) 0];[ProbZTmp_norm(flag2CheckId(ui),:)./max(ProbZTmp_norm(flag2CheckId(ui),:)) 0];[t2 0]];
                
                checkProb = [tempProb33./max(tempProb33(:)) thetaProb'./max(thetaProb)];
                checkProb(1,depthGTInd(flag2CheckId(ui))) = 1;
                checkProb = [checkProb; [sum(tempProb3)./max(sum(tempProb3)) 0];[ProbZTmp_norm(flag2CheckId(ui),:)./max(ProbZTmp_norm(flag2CheckId(ui),:)) 0];[t2 0]];
                checkProb_New = [tempProb33_New./max(tempProb33_New(:)) thetaProb'./max(thetaProb) ValidFeatFlagTmp(:,checkId0)];
                checkProb_New(1,depthGTInd(flag2CheckId(ui))) = 1;
                checkProb_New = [checkProb_New; [sum(tempProb3)./max(sum(tempProb3)) 0 0];[ProbZTmp_norm(flag2CheckId(ui),:)./max(ProbZTmp_norm(flag2CheckId(ui),:)) 0 0];[t2 0 0]];
                
                pTheta2 = thetaProb.*ValidFeatFlagTmp(:,checkId0)';
                checkProb_New2 = [tempProb33_New2./max(tempProb33_New2(:)) thetaProb'./max(thetaProb) pTheta2'./max(pTheta2) ValidFeatFlagTmp(:,checkId0) ];
                checkProb_New2(1,depthGTInd(flag2CheckId(ui))) = 1;
                checkProb_New2 = [checkProb_New2; [sum(tempProb3)./max(sum(tempProb3)) 0 0 0];[ProbZTmp_norm(flag2CheckId(ui),:)./max(ProbZTmp_norm(flag2CheckId(ui),:)) 0 0 0];[t2 0 0 0]];
                checkProb_New = checkProb_New2;
                
                if LOnly
                    checkProb_New = [checkProbL_New(:,1:end-1) [pTheta2'./max(pTheta2);0; 0; 0] checkProbL_New(:,end)];
                end
                %                 checkProb_New
                
                if drawPorb1
                    if jh == idM_Vec(1)
                        figure(70),clf;subplot(2,2,1);imshow(checkProbL, []);title('L');
                        subplot(2,2,2);imshow(checkProbR, []);title('R');
                        subplot(2,2,3);imshow(checkProb, []);title('L*R');
                        subplot(2,2,4);plot([checkProb(2:end-3,depthGTInd(flag2CheckId(ui))) checkProb(2:end-3,end)]);legend('L*R','p(theta)');title(sprintf('sigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f',obj.configParam.reproj_sigma,obj.configParam.reproj_beta,obj.configParam.reproj_sigma_right,obj.configParam.reproj_beta_right));
                        
                        if ~LOnly
                            figure(71),clf;subplot(2,2,1);imshow(checkProbL_New, []);title('L');
                            subplot(2,2,2);imshow(checkProbR_New, []);title('R');
                            subplot(2,2,3);imshow(checkProb_New, []);title('L*R');
                            subplot(2,2,4);plot([checkProb_New(2:end-3,depthGTInd(flag2CheckId(ui))) checkProb_New(2:end-3,end-1) checkProb_New(2:end-3,idMax_update(flag2CheckId(ui)))]);
                            hold on;plot([find(ValidFeatFlagTmp(:,checkId0))-1 find(ValidFeatFlagTmp(:,checkId0))-1 find(ValidFeatFlagTmp(:,checkId0))-1],[checkProb_New(find(ValidFeatFlagTmp(:,checkId0)),depthGTInd(flag2CheckId(ui))) checkProb_New(find(ValidFeatFlagTmp(:,checkId0)),end-1) checkProb_New(find(ValidFeatFlagTmp(:,checkId0)),idMax_update(flag2CheckId(ui)))], 'o');
                            legend('L*R(gt)','p(theta)','L*R(peak_update)');title(sprintf('sigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f\n',obj.configParam.reproj_sigma,obj.configParam.reproj_beta,obj.configParam.reproj_sigma_right,obj.configParam.reproj_beta_right));
                        else
                            figure(71),clf;subplot(2,2,1);imshow(checkProbL_New, []);title('L');
                            subplot(2,2,2);imshow(checkProbR_New, []);title('R');
                            subplot(2,2,3);imshow(checkProb_New, []);title('L*R');
                            subplot(2,2,4);plot([checkProbL_New(2:end-3,depthGTInd(flag2CheckId(ui))) checkProbL_New(2:end-3,end-1) checkProbL_New(2:end-3,idMax_update(flag2CheckId(ui)))]);
                            hold on;plot([find(ValidFeatFlagTmp(:,checkId0))-1 find(ValidFeatFlagTmp(:,checkId0))-1 find(ValidFeatFlagTmp(:,checkId0))-1],[checkProbL_New(find(ValidFeatFlagTmp(:,checkId0)),depthGTInd(flag2CheckId(ui))) checkProbL_New(find(ValidFeatFlagTmp(:,checkId0)),end-1) checkProbL_New(find(ValidFeatFlagTmp(:,checkId0)),idMax_update(flag2CheckId(ui)))], 'o');
                            legend('L*R(gt)','p(theta)','L*R(peak_update)');title(sprintf('sigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f\n',obj.configParam.reproj_sigma,obj.configParam.reproj_beta,obj.configParam.reproj_sigma_right,obj.configParam.reproj_beta_right));
                        end
                        figure(72),clf;subplot(2,2,1);imshow(checkProbL_New, []);title('L');
                        subplot(2,2,2);imshow(checkProbR_New, []);title('R');
                        subplot(2,2,3);imshow(checkProb_New2, []);title('L*R');
                        subplot(2,2,4);plot([checkProb_New2(2:end-3,depthGTInd(flag2CheckId(ui))) checkProb_New2(2:end-3,end-1)]);
                        hold on;plot([find(ValidFeatFlagTmp(:,checkId0))-1 find(ValidFeatFlagTmp(:,checkId0))-1],[checkProb_New2(find(ValidFeatFlagTmp(:,checkId0)),depthGTInd(flag2CheckId(ui))) checkProb_New2(find(ValidFeatFlagTmp(:,checkId0)),end-1)], 'o');
                        legend('L*R','p(theta)');title(sprintf('sigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f\n',obj.configParam.reproj_sigma,obj.configParam.reproj_beta,obj.configParam.reproj_sigma_right,obj.configParam.reproj_beta_right));
                    end
                end
                if 0
                    aa = checkProbL(1:41,1:17).*checkProbR(1:41,1:17);
                    bb = checkProb(1:41,1:17);
                    cc = aa./bb;
                    figure,imshow(abs(aa-bb),[]);
                    sum(checkProb(2:41,9))./sum(checkProb(1:41,8))
                    dot(checkProb(2:41,8),checkProb(2:41,end))./dot(checkProb(2:41,9),checkProb(2:41,end))
                end
                
                
                if drawPorb1
                    if jh == idM_Vec(1)
                        figure(69),clf;subplot(1,4,1);plot(tempProb11'./max(sum(tempProb11)));hold on;plot(sum(tempProb11./max(sum(tempProb11))),'-x');title(sprintf('L\nsigL = %0.3f,  betaL = %0.3f\nsigR = %0.3f,  betaR = %0.3f',obj.configParam.reproj_sigma,obj.configParam.reproj_beta,obj.configParam.reproj_sigma_right,obj.configParam.reproj_beta_right));
                        subplot(1,4,2);plot(tempProb22'./max(sum(tempProb22)));hold on;plot(sum(tempProb22)./max(sum(tempProb22)),'-x');title('R');
                        subplot(1,4,3);plot(tempProb33'./max(sum(tempProb33)));hold on;plot(sum(tempProb33)./max(sum(tempProb33)),'-x');title('L*R');
                        subplot(1,4,4);plot(tempProb44'./max(sum(tempProb44)));hold on;plot(sum(tempProb44)./max(sum(tempProb44)),'-x');title('L + R');
                    end
                end
            end
        end
        
        if 0
            figure(52),subplot(1,2,1);plot(travelPixXMat(:),travelPixYMat(:), '.b');plot(pt2dCur(flag2CheckId(ui),1),pt2dCur(flag2CheckId(ui),2),'.w');subplot(1,2,2);plot(travelPixXMatR(:),travelPixYMatR(:), '.r');
        end
        
        
        if travelPixXMat(1,1) < travelPixXMat(end,1)
            travelPixXMatDlt1 = travelPixXMat - (pt2dCur(flag2CheckId(ui),1) - obj.configParam.tracking_trust_radius);
            travelPixXMatDlt2 = travelPixXMat - (pt2dCur(flag2CheckId(ui),1) + obj.configParam.tracking_trust_radius);
            travelFlag = (travelPixXMatDlt1(1,:) < 0 & travelPixXMatDlt2(end,:) > 0) | (travelPixXMatDlt1(1,:) > 0 & travelPixXMatDlt2(end,:) < 0);
        else
            travelPixXMatDlt1 = travelPixXMat - (pt2dCur(flag2CheckId(ui),1) + obj.configParam.tracking_trust_radius);
            travelPixXMatDlt2 = travelPixXMat - (pt2dCur(flag2CheckId(ui),1) - obj.configParam.tracking_trust_radius);
            travelFlag = (travelPixXMatDlt1(1,:) > 0 & travelPixXMatDlt2(end,:) < 0) | (travelPixXMatDlt1(1,:) < 0 & travelPixXMatDlt2(end,:) > 0);
        end
        
        init_depth = ProbZTmp_norm(flag2CheckId(ui),:)./max(ProbZTmp_norm(flag2CheckId(ui),:));
        travelFlagIn = travelFlag;
        travelFlagIn(init_depth < obj.configParam.depth_prob_cutoff_threshold) = 0;
        TravelFlagIn = [TravelFlagIn; travelFlagIn];
        
        TravelFlagIn2(yu,1) = [sum(travelFlagIn)/sum(init_depth >= obj.configParam.depth_prob_cutoff_threshold)];
        
        if drawPorb1
            figure(52);subplot(1,2,1); plot(ptGTL(:,1), ptGTL(:,2),'oy');
            plot(pt2dCur(flag2CheckId(ui),1), pt2dCur(flag2CheckId(ui),2),'or');
            
            subplot(1,2,2); plot(ptGTR(:,1), ptGTR(:,2),'oy');
            plot(pt2dCurR(flag2CheckId(ui),1), pt2dCurR(flag2CheckId(ui),2),'or')
        end
        
        if 1 %drawPorb1
            if yu <= length(Flag2CheckId)/2
                figure(54),hold on;
                %             plot(ptSpanL(1,1),ptSpanL(1,2) - initDepthProb(1),'.r'); plot(ptSpanL(1,1),ptSpanL(1,2) - LRTheta(1),'.g');plot(ptSpanL(1,1),ptSpanL(1,2) - updatedDepthProb(1),'.b');plot(ptSpanL(gtId,1),ptSpanL(1,2) - updatedDepthProb(gtId),'*c');
                
                %             if yu == 1
                %                 legend('initDepth','L*R*Theta','updateDepth','gt')
                %
                %             end
                plot(ptSpanL(:,1),ptSpanL(1,2) + initDepthProb,'-r'); plot(ptSpanL(:,1),ptSpanL(1,2) + LRTheta,'-g');plot(ptSpanL(:,1),ptSpanL(1,2) + updatedDepthProb,'-y');plot(ptSpanL(gtId,1),ptSpanL(1,2) + updatedDepthProb(gtId),'*c');
                if 0 % yu == 1
                    legend('initDepth','L*R*Theta','updateDepth','gt')
                end
                
                if 0
                    plot(ptGTL(:,1), ptGTL(1,2) - checkProb_New(find(ValidFeatFlagTmp(:,checkId0)),end-1),'-b');plot(ptGTL(:,1), ptGTL(1,2) - checkProb_New(find(ValidFeatFlagTmp(:,checkId0)),end-2),'-w');plot(ptGTL(:,1), ptGTL(1,2) - checkProb_New(find(ValidFeatFlagTmp(:,checkId0)),depthGTInd(flag2CheckId(ui))),'-m');
                    plot(ptPeakL(:,1), ptPeakL(1,2) - checkProb_New(find(ValidFeatFlagTmp(:,checkId0)),end-1),'-c');plot(ptPeakL(:,1), ptPeakL(1,2) - checkProb_New(find(ValidFeatFlagTmp(:,checkId0)),idMax_update(flag2CheckId(ui))),'-k');
                else
                    plot(ptGTL(:,1), ptGTL(1,2) - checkProb_New(idM_Vec,end-1),'-b');plot(ptGTL(:,1), ptGTL(1,2) - checkProb_New(idM_Vec,end-2),'-w');plot(ptGTL(:,1), ptGTL(1,2) - checkProb_New(idM_Vec,depthGTInd(flag2CheckId(ui))),'-m');
                    plot(ptPeakL(:,1), ptPeakL(1,2) - checkProb_New(idM_Vec,end-1),'-c');plot(ptPeakL(:,1), ptPeakL(1,2) - checkProb_New(idM_Vec,idMax_update(flag2CheckId(ui))),'-k');
                    
                end
                plot(ptSpanL(:,1),ptSpanL(:,2),'.c');plot(ptGTListL(1),ptGTListL(2),'oy');
                %             plot(ptGTL(:,1), ptGTL(:,2),'.g'); %plot(pt2dCur(flag2CheckId(ui),1), pt2dCur(flag2CheckId(ui),2),'.r');axis equal;
                plot(ptPeakL(:,1), ptPeakL(:,2),'.k');
                plot(ptGTL(:,1), ptGTL(:,2),'.g');plot(pt2dCur(flag2CheckId(ui),1), pt2dCur(flag2CheckId(ui),2),'.r');axis equal;
                %             plot(ptSpanL(:,1),ptSpanL(1,2) - initDepthProb,'-c'); plot(ptSpanL(:,1),ptSpanL(1,2) - LRTheta,'-c');plot(ptSpanL(:,1),ptSpanL(1,2) - updatedDepthProb,'-c');plot(ptSpanL(gtId,1),ptSpanL(1,2) - updatedDepthProb(gtId),'-c');
                
            else
                figure(54),hold on;
                plot(ptSpanL(:,1),ptSpanL(1,2) + initDepthProb,'-r'); plot(ptSpanL(:,1),ptSpanL(1,2) + LRTheta,'-g');plot(ptSpanL(:,1),ptSpanL(1,2) + updatedDepthProb,'-y');plot(ptSpanL(gtId,1),ptSpanL(1,2) + updatedDepthProb(gtId),'*c');
                if 0
                    plot(ptGTL(:,1), ptGTL(1,2) - checkProb_New(find(ValidFeatFlagTmp(:,checkId0)),end-1),'-b');plot(ptGTL(:,1), ptGTL(1,2) - checkProb_New(find(ValidFeatFlagTmp(:,checkId0)),end-2),'-w');plot(ptGTL(:,1), ptGTL(1,2) - checkProb_New(find(ValidFeatFlagTmp(:,checkId0)),depthGTInd(flag2CheckId(ui))),'-m');
                    plot(ptPeakL(:,1), ptPeakL(1,2) - checkProb_New(find(ValidFeatFlagTmp(:,checkId0)),end-1),'-c');plot(ptPeakL(:,1), ptPeakL(1,2) - checkProb_New(find(ValidFeatFlagTmp(:,checkId0)),idMax_update(flag2CheckId(ui))),'-k');
                else
                    plot(ptGTL(:,1), ptGTL(1,2) - checkProb_New(idM_Vec,end-1),'-b');plot(ptGTL(:,1), ptGTL(1,2) - checkProb_New(idM_Vec,end-2),'-w');plot(ptGTL(:,1), ptGTL(1,2) - checkProb_New(idM_Vec,depthGTInd(flag2CheckId(ui))),'-m');
                    plot(ptPeakL(:,1), ptPeakL(1,2) - checkProb_New(idM_Vec,end-1),'-c');plot(ptPeakL(:,1), ptPeakL(1,2) - checkProb_New(idM_Vec,idMax_update(flag2CheckId(ui))),'-k');
                    
                end
                plot(ptSpanL(:,1),ptSpanL(:,2),'.c');plot(ptGTListL(1),ptGTListL(2),'oy');
                %             plot(ptGTL(:,1), ptGTL(:,2),'.r');%plot(pt2dCur(flag2CheckId(ui),1), pt2dCur(flag2CheckId(ui),2),'.g');axis equal;
                plot(ptPeakL(:,1), ptPeakL(:,2),'.k');
                plot(ptGTL(:,1), ptGTL(:,2),'.r');plot(pt2dCur(flag2CheckId(ui),1), pt2dCur(flag2CheckId(ui),2),'.g');axis equal;
                %             plot(ptSpanL(:,1),ptSpanL(1,2) - initDepthProb,'-c'); plot(ptSpanL(:,1),ptSpanL(1,2) - LRTheta,'-c');plot(ptSpanL(:,1),ptSpanL(1,2) - updatedDepthProb,'-c');plot(ptSpanL(gtId,1),ptSpanL(1,2) - updatedDepthProb(gtId),'-c');
            end
            text(double(ptGTListL(:,1)), double(ptGTListL(:,2)) + 5,strcat(num2str(yu),'--',num2str(tracebackId(yu))),'Color','m');
        end
        
        figure(91),clf,plot([sum(TravelFlagIn'); 10.*ProbZTmpTmp_update_norm(Flag2CheckId(1:yu))']');drawnow;
        Buf{yu,1} = [buf];
        BufL{yu,1} = bufL;
        BufR{yu,1} = bufR;
        
        
        Errrrr(yu,1) = norm(pt2dCur(flag2CheckId(ui),1:2) - ptGTListL(1:2));
        checkIdPtGT(yu,:) = [pt2dCur(flag2CheckId(ui),1:2) ptGTListL(1:2) pix11L pix11R];
    end
    
    save(fullfile(probPath,sprintf(strcat('traceLogData_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.mat'),length(dir(fullfile(probPath,strcat('traceLogData_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.mat'))))+1)), 'checkIdPtGT');
    
    figure(92);clf;subplot(2,2,1);plot(abs(Errrrr));title('tracking error');subplot(2,2,2);plot([TravelFlagIn2 ProbZTmpTmp_update_norm(Flag2CheckId(1:yu))]);title(sprintf('pass rate\np(theta) > %0.5f\np(z) > %0.5f\nradius > %0.5f',obj.configParam.theta_prob_cutoff_threshold,obj.configParam.depth_prob_cutoff_threshold, obj.configParam.tracking_trust_radius));
    subplot(2,2,3),plot(abs(abs(checkIdPtGT(:,1) - checkIdPtGT(:,5)) - abs(checkIdPtGT(:,1) - checkIdPtGT(:,7))),'-x');title('symmetry');
    subplot(2,2,4),plot((abs(checkIdPtGT(:,1) - checkIdPtGT(:,5)) - abs(checkIdPtGT(:,1) - checkIdPtGT(:,7))),'-x');title('symmetry');
    saveas(gcf,fullfile(probPath,sprintf(strcat('passRate_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('passRate_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
    
    figure(52);saveas(gcf,fullfile(probPath,sprintf(strcat('traceLog_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.fig'),length(dir(fullfile(probPath,strcat('traceLog_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.fig'))))+1)));
    saveas(gcf,fullfile(probPath,sprintf(strcat('traceLog_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('traceLog_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
    
    %     saveas(gcf,fullfile(probPath,sprintf('epiLine_%05d.fig',length(dir(fullfile(probPath,'epiLine_*.fig')))+1)));
    if 1 %drawPorb1
        figure(54);
        saveas(gcf,fullfile(probPath,sprintf(strcat('epiLine_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.fig'),length(dir(fullfile(probPath,strcat('epiLine_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.fig'))))+1)));
        try
            figure(67),clf;plot(PeakValGT(end,:)' - ProbZTmpTmp_update_norm(checkId))
        catch
            asnlk = 1;
        end
    end
    adfqnh = 1;
    if 0
        checkId0 = checkId;
        %     tempProb = ProbZAll(:,checkId0,:).*ProbReprojVecAll(:,checkId0,:).*thetaProbAll(:,checkId0,:).*ProbReprojVecRAll(:,checkId0,:);
        tempProb = ProbReprojVecAll(:,checkId0,:).*thetaProbAll(:,checkId0,:).*ProbReprojVecRAll(:,checkId0,:);
        tempProb = permute(tempProb, [1 3 2]);
        sum(tempProb);
        figure,plot(sum(tempProb));
        figure,imshow(tempProb,[])
        figure,plot(tempProb')
        figure,plot(tempProb');hold on;plot(sum(tempProb),'-x');
        
        
        
        
        
        tempProb1 = ProbReprojVecAll(:,checkId0,:).*thetaProbAll(:,checkId0,:);
        tempProb1 = permute(tempProb1, [1 3 2]);
        figure,plot(tempProb1');hold on;plot(sum(tempProb1),'-x');title('L*theta');
        tempProb2 = ProbReprojVecRAll(:,checkId0,:).*thetaProbAll(:,checkId0,:);
        tempProb2 = permute(tempProb2, [1 3 2]);
        figure,plot(tempProb2');hold on;plot(sum(tempProb2),'-x');title('R*theta');
        tempProb3 = ProbReprojVecRAll(:,checkId0,:).*thetaProbAll(:,checkId0,:).*ProbReprojVecAll(:,checkId0,:);
        tempProb3 = permute(tempProb3, [1 3 2]);
        figure,plot(tempProb3');hold on;plot(sum(tempProb3),'-x');title('L*R*theta')
        
        tempProb11 = ProbReprojVecAll(:,checkId0,:);
        tempProb11 = permute(tempProb11, [1 3 2]);
        figure,plot(tempProb11');hold on;plot(sum(tempProb11),'-x');title('L');
        
        tempProb22 = ProbReprojVecRAll(:,checkId0,:);
        tempProb22 = permute(tempProb22, [1 3 2]);
        figure,plot(tempProb22');hold on;plot(sum(tempProb22),'-x');title('R');
        
        tempProb33 = ProbReprojVecRAll(:,checkId0,:).*ProbReprojVecAll(:,checkId0,:);
        tempProb33 = permute(tempProb33, [1 3 2]);
        figure,plot(tempProb33');hold on;plot(sum(tempProb33),'-x');title('L*R');
        
        
        
        
        xxx = sum(tempProb3).*ProbZTmp_norm(flag2CheckId(ui),:);
        ttt = ProbZTmp_update_norm(flag2CheckId(ui),:);
        errCheck = ttt./xxx;
        figure,plot(errCheck);
        
        
    end
end