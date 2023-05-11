unValidFeatId0 = []; validFeatId0 = []; traceLenList0 = []; trackingLostId0 = [];validFeatId0 = [];
for ii = 1 : length(traceNumUniq)
    if traceNumUniq(ii) < 2 % || traceNumUniq(ii) > 8
        %     if traceNumUniq(ii) < 2 || traceNumUniq(ii) >= 3
        continue;
    else
        featId22 = find(traceNum == traceNumUniq(ii));
        tempZ = localZ(featId(featId22),1:end-1);
        validZ = sum(tempZ' > 0,1)';
        featId2 = featId22(validZ == traceNumUniq(ii)-1);
        if length(featId2) < 10  || traceNumUniq(ii) > numThr %  19 %15  % 17  % 15 %15 % 200  %130   % 14 %80 %80; % 80;  %50 % 80 %50 %130  %15 % 40
            unValidFeatId0 = [unValidFeatId0; featId(featId2)];
            trackingLostId0 = [trackingLostId0; featId(featId2)];
            continue;
        end
                validFeatId0 = [validFeatId0; featId(featId2)];


    end
end

svdbkj = 1;