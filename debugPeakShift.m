if doShift
    
    idMax_update;
    depthGTInd_update;
    disparityError;
    cutZ = obj.configParam.depth_prob_cutoff_threshold;
    if use1
        idKey = find(cell2mat(obj.keyProbZ(:,1)) == max(cell2mat(obj.keyProbZ(:,1))));
    else
        idKey = find(cell2mat(obj.keyProbZ2(:,1)) == max(cell2mat(obj.keyProbZ2(:,1))));
    end
    % idKey = find(cell2mat(obj.keyProbZ(:,1)) == 2);
    updtZ = {}; trackingErr = []; updtZFirst = {}; trackingErrStack = []; trackingErrStackX = []; trackingErrStackY = [];
    if length(idKey) > 0%9
        if use1
            endFeat = obj.keyProbZ{idKey(end),2};
            %         trackingErr = [obj.keyProbZ{idKey(end),8}];
            figure(1),clf;
            for qw = 1 : length(idKey) - 1
                
                commonIdTmp = find(ismember(obj.keyProbZ{idKey(qw),2}, endFeat));
                commonId2 = find(~ismember( endFeat,obj.keyProbZ{idKey(qw),2}));
                commonId2_ = find(ismember( endFeat,obj.keyProbZ{idKey(qw),2}));
                
                commonId = zeros(length(endFeat),1);
                commonId(commonId2_) = commonIdTmp; commonId(commonId2) = commonIdTmp(1);
                
                updtZ{qw,1} = obj.keyProbZ{idKey(qw),4}(commonId,:);
                
                if qw == 1
                    updtZFirst{1} = obj.keyProbZ{idKey(qw),3}(commonId,:);
                end
                
                trackingErr = [trackingErr obj.keyProbZ{idKey(qw),8}(commonId,:)];
                errTrace = [obj.keyProbZ{idKey(qw),6}(commonId,:) - obj.keyProbZ{idKey(qw),7}(commonId,:)];
                trackingErrStack = [trackingErrStack; errTrace];
                trackingErrStackX = [trackingErrStackX errTrace(:,1)];
                trackingErrStackY = [trackingErrStackY errTrace(:,2)];
                figure(1),hold on;plot(errTrace(:,1),errTrace(:,2), '+');
                drawnow;
            end
            updtZ = [updtZ; obj.keyProbZ{idKey(end),4}];
            updtZAll = [updtZFirst;updtZ];
            
            passRate = [];
            for yt = 1 : size(updtZAll,1)-1
               temp1 =  updtZAll{yt};
               maxTemp1 = max(temp1')';
               maxTemp1 = repmat(maxTemp1, 1,size(temp1,2));
               temp11 = temp1./maxTemp1;
               temp2 =  updtZAll{yt+1};
               maxTemp2 = max(temp2')';
               maxTemp2 = repmat(maxTemp2, 1,size(temp2,2));
               temp22 = temp2./maxTemp2;
               
               mask1 = temp11 > cutZ;
               mask2 = temp22 > cutZ;
                passRate(:,yt) = sum(mask2')'./sum(mask1')';
                
            end
            
            trackingErr = [trackingErr [obj.keyProbZ{idKey(end),8}]];
            %         figure(68),clf;plot(trackingErr(checkId,:));
        else
            
            endFeat = obj.keyProbZ2{idKey(end),2};
            %         trackingErr = [obj.keyProbZ2{idKey(end),8}];
            for qw = 1 : length(idKey) - 1
                %                 commonId = find(ismember(obj.keyProbZ2{idKey(qw),2}, endFeat));
                commonIdTmp = find(ismember(obj.keyProbZ{idKey(qw),2}, endFeat));
                commonId2 = find(~ismember( endFeat,obj.keyProbZ{idKey(qw),2}));
                commonId2_ = find(ismember( endFeat,obj.keyProbZ{idKey(qw),2}));
                
                commonId = zeros(length(endFeat),1);
                commonId(commonId2_) = commonIdTmp; commonId(commonId2) = commonIdTmp(1);
                
                
                
                
                
                updtZ{qw,1} = obj.keyProbZ2{idKey(qw),4}(commonId,:);
                trackingErr = [trackingErr obj.keyProbZ2{idKey(qw),8}(commonId,:)];
                
            end
            updtZ = [updtZ; obj.keyProbZ2{idKey(end),4}];
            trackingErr = [trackingErr [obj.keyProbZ2{idKey(end),8}]];
            
            
        end
        
        UpdtZ = cell2mat(updtZ); UpdtZ = UpdtZ(:);
        UpdtZ = reshape(UpdtZ, length(endFeat), size(updtZ,1), []);
        UpdtZ = permute(UpdtZ, [1 3 2]);
        
        
        AAA = UpdtZ(checkId,:,:);
        PeakValGT = [];
        for as = 1 : length(checkId)
            tempAs = UpdtZ(checkId(as),:,:);
            idMax_update_check = idMax_update(checkId(as));
            depthGTInd_update_check = depthGTInd_update(checkId(as));
            tempAs = permute(tempAs,[3 2 1]);
            [peakZ, peakId] = max(tempAs');
            peakZCoord = [peakId', ones(length(peakZ),1)];
            %             peakZCoordGT = repmat([depthGTInd_update_check 1],size(tempAs,1),1);
            peakZCoordGT = [repmat(depthGTInd_update_check,size(tempAs,1),1),[1:size(tempAs,1)]'];
            peakIndGT = sub2ind(size(tempAs), peakZCoordGT(:,2), peakZCoordGT(:,1));
            
            %             peakIdMat = repmat(peakId',1,size(tempAs,2));
            maxPeakZ = peakZ';
            maxPeakZ = repmat(maxPeakZ, 1, size(tempAs,2));
            maxPeakZ_norm = tempAs./maxPeakZ;
            
            peakValGT = maxPeakZ_norm(peakIndGT);
            PeakValGT = [PeakValGT peakValGT];
            
            errCheck = peakValGT(end) - ProbZTmpTmp_update_norm(checkId(as));
            
            figure(62),clf;plot(disparityRng, tempAs'); title(strcat(num2str(as),'/',num2str(length(checkId))));
%             figure(65),clf;plot(disparityRng, maxPeakZ_norm');hold on;plot(peakZCoord(:,1), peakZCoord(:,2),'*r');plot(peakZCoordGT(:,1),peakValGT,'og'); title(sprintf('%d/%d\nerr: %f',as, length(checkId), errCheck));%plot(peakZCoordGT(:,1),peakZCoordGT(:,2),'og');
            figure(65),clf;plot(disparityRng, maxPeakZ_norm');hold on;plot(disparityRng(peakZCoord(:,1)), peakZCoord(:,2),'*r');plot(disparityRng(peakZCoordGT(:,1)),peakValGT,'og'); title(sprintf('%d/%d\nerr: %f',as, length(checkId), errCheck));%plot(peakZCoordGT(:,1),peakZCoordGT(:,2),'og');
            drawnow;
        end
        figure(66);
        if size(obj.featPtManager.localTrace.ptIcsX, 2) == 2
            clf;
        end
        
        
        %         figure(66),hold on;plot(PeakValGT); %subplot(2,1,1);plot(PeakValGT(:,1:length(checkId)/2));subplot(2,1,2);plot(PeakValGT(:,length(checkId)/2+1:end));
        
%         figure(66),clf;subplot(1,3,1);plot(PeakValGT);title('drop rate');subplot(1,3,2),plot(trackingErr(checkId,:));title('tracking error');
%         subplot(1,3,3),plot(trackingErr(checkId(1:length(checkId)/2),:)','-g');hold on;plot(trackingErr(checkId(length(checkId)/2+1:end),:)','-r');legend('bad');
        
        figure(66),clf;subplot(1,3,1);plot(PeakValGT(:,1:length(checkId)/2),'-g');hold on;plot(PeakValGT(:,length(checkId)/2+1:end),'-r');legend('bad'); title('drop rate');% subplot(1,3,2),plot(trackingErr(checkId,:));title('tracking error');
        subplot(1,3,2),plot(trackingErr(checkId(1:length(checkId)/2),:)','-g');hold on;plot(trackingErr(checkId(length(checkId)/2+1:end),:)','-r');legend('bad');title('tracking error');
        subplot(1,3,3),plot((1:length(checkId)/2),disparityError(checkId(1:length(checkId)/2)),'-xg');hold on;plot(length(checkId)/2+1:length(checkId),disparityError(checkId(length(checkId)/2+1:end)),'-xr');legend('bad');title('disparity error');
        
        saveas(gcf,fullfile(probPath,sprintf(strcat('dropRate_trackingErr_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__%05d.png'),length(dir(fullfile(probPath,strcat('dropRate_trackingErr_',probDir(end-14:end),'____',num2str(sum(obj.keyFrameFlagList)),'__*.png'))))+1)));
                            
        
        
        
        if exist('savePicShift', 'var')
            if ~isempty(savePicShift)
                figure(66),
                saveas(gcf,fullfile(probPath,savePicShift));
                %                 figure(68),
                %                 saveas(gcf,fullfile(probPath,savePicShift(2:end)));
                %             saveas(gcf,fullfile(probDir,sprintf('rng_%05d.fig',length(dir(fullfile(probDir,'rng_*.fig')))+1)));
            end
        end
    end
end