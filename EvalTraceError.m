function EvalTraceError(track_error, dispErrExpStack)



frameId00 = cell2mat(dispErrExpStack(:,1));
dispErrExpStack(setdiff(1:size(dispErrExpStack,1), frameId00),:) = [];
frameId000 = find(cell2mat(dispErrExpStack(:,9)) > 0);
dispErrExpStack = dispErrExpStack(frameId000,:);

validTraceBatch = cell2mat(dispErrExpStack(:,1));
valid = [];
for i = 1 : size(track_error,1)
    if ~isempty(track_error{i,1})
        if 1 % ismember(track_error{i}{2}, validTrace)
            valid = [valid; i];
        end
    end
end
track_error = track_error(valid,:);
track_error = track_error(end:-1:1,:);
track_len = [size(track_error,1) : -1 : 1];
trackingErrMatX = [];
trackingErrMatY = [];
featId = {};


for i = 1 : size(track_error,1)
    tempError = track_error{i,1};
    if ~isempty(tempError)
        for j = 1 : length(tempError)
            tempError2 = tempError(j,:);
            traceBatch = tempError2{2};
            traceId = tempError2{3};
            traceError = tempError2{1};
            if ~ismember(traceBatch, validTraceBatch)
                continue;
            end
            commonId = [1 : size(traceError,1)]';
            try
                if isempty(featId{traceBatch,1})
                    featId{traceBatch,1} = traceBatch;
                    featId{traceBatch,2} = traceId;
                else
                    commonId = find(ismember(traceId, featId{traceBatch,2}));
                    assert(length(commonId) - length(featId{traceBatch,2}) == 0);
                    skbj = 1;
                end
            catch
                featId{traceBatch,1} = traceBatch;
                featId{traceBatch,2} = traceId;
            end
            
            trackingErrMatX(traceBatch, track_len(i)) = mean(traceError(commonId,1));
            trackingErrMatY(traceBatch, track_len(i)) = mean(traceError(commonId,2));
        end
        
    end
end

trackingErrMatX(trackingErrMatX == 0) = nan;
trackingErrMatY(trackingErrMatY == 0) = nan;



angAvgMat = [];
angGtDispMat = [];
angGtThetaMat = [];

for k = 1 : size(dispErrExpStack,1)
   tempStack = dispErrExpStack(k,:);
   finalTraceDone = tempStack{7};
   traceLen = length(finalTraceDone.gtTheta);
   angAvgMat(tempStack{1},1 : traceLen) =  finalTraceDone.ANGOptFinal;
   angGtDispMat(tempStack{1}, 1 : traceLen) = finalTraceDone.gtTheta;
   angGtThetaMat(tempStack{1}, 1 : traceLen) = finalTraceDone.localTrace_gtAng2;
end

angAvgErrMat = rad2deg(angAvgMat - angGtThetaMat);
angGtDispErrMat = rad2deg(angGtDispMat - angGtThetaMat);


for ii = 1 : size(trackingErrMatX,1)
    figure(1),clf;subplot(1,2,1);plot(trackingErrMatX(ii,:),'-b');hold on;plot(angGtDispErrMat(ii,:),'-r');grid on;legend('tracking err','ang err');
    subplot(1,2,2);plot([sign(trackingErrMatX(ii,:)') - sign(angGtDispErrMat(ii,:)')]);grid on;
    pause(0.3);
end


end