function CalcDispErrDistribution(inputDir)

% figure(1),clf;

dirInfoGT = dir(fullfile(inputDir, 'disparityGroundTruth_*.mat'));
dirInfoImg = dir(fullfile(inputDir, 'stereoPair_*.mat'));
disparity_bound = 0.5;
disparity_sample_step = 0.05;
disparityRng = [-disparity_bound : disparity_sample_step : disparity_bound];
for i = 1 : length(dirInfoGT)
    load(fullfile(inputDir, dirInfoGT(i).name));
    dispGT(:,:,1) = disparityGroundTruth;
    load(fullfile(inputDir, dirInfoImg(i).name));
    imgLL{i,1} = stereoPair(:,1:size(stereoPair,2)/2,:);
    imgRR{i,1} = stereoPair(:,size(stereoPair,2)/2+1:end,:);
    imgSize = size(rgb2gray(imgLL{i,1}));
    imgL = imgLL{i,1}; imgR = imgRR{i,1};
    [disparityMap, validMap] = nanhu_depth_upsample(rgb2gray(imgL), rgb2gray(imgR), 0, 8, 7, 11, 21, 41, 2, 6,4);
    
    ptIcs = detectFASTFeatures(rgb2gray(imgL),'MinQuality',0.02,'MinContrast',0.02);
    
    featPtList = ptIcs.Location;
    inId = find(featPtList(:,1) > 10 & featPtList(:,2) > 10 & featPtList(:,1) < imgSize(2) - 10 & featPtList(:,2) < 124);
    featPtList = featPtList(inId,:);
    
%     figure, imshow(imgL); hold on; plot(featPtList(:,1), featPtList(:,2), '.g');
    
    Ind = sub2ind(imgSize, round(featPtList(:,2)), round(featPtList(:,1)));
    dispList = disparityMap(Ind);
    dispListGT = disparityGroundTruth(Ind);
    validInd = ~isnan(dispList) & ~isnan(dispListGT);
    
    featPtListValid = featPtList(validInd);
    dispListValid = dispList(validInd);
    dispListGTValid = dispListGT(validInd);
    
    DispRng = dispListValid + disparityRng;
%     depthGTIndEnd = RoundingDispErr(dispListValid,dispListGTValid, DispRngStep,DispRng);
%     
%     DispRng = repmat(dispList(inlierId,:),1,length(disparityRng)) + repmat(disparityRng,length(inlierId),1);
    DispRng(DispRng < 0) = 0.0001;
    disparityError = dispListValid - dispListGTValid;
    
    disparityErrorRound = round(disparityError./disparity_sample_step).*disparity_sample_step;
    
    
    depthGTOfst11 = round(-(disparityErrorRound)./disparity_sample_step);
    depthGTInd11 = depthGTOfst11 + (size(DispRng,2)+1)/2;
    depthGTInd11(depthGTInd11 < 1) = 1;
    depthGTInd11(depthGTInd11 > size(DispRng,2)) = size(DispRng,2);
    
    %                             DepthGTInd(jkl,:) = depthGTInd11';
    
    depthGTIndAll11 = sub2ind(size(DispRng),[1:size(DispRng,1)]', depthGTInd11);
    
    checkDispRounding = DispRng(depthGTIndAll11) - dispListGTValid;
    
    ProbZTmp_update_norm_sum = zeros(size(DispRng));
    ProbZTmp_update_norm_sum(:,(size(DispRng,2)+1)/2) = 1;
    ProbZTmp_update_norm_sum = ProbZTmp_update_norm_sum ./ (repmat(sum(ProbZTmp_update_norm_sum')',1,size(ProbZTmp_update_norm_sum,2)));
    
    ProbZTmp_update_norm_sum_tmp = zeros(size(ProbZTmp_update_norm_sum));
%     ProbZTmp_update_norm_sum_tmp(depthGTIndAll11) = ProbZTmp_update_norm_sum(depthGTIndAll11);
    ProbZTmp_update_norm_sum_tmp(depthGTIndAll11) = 1;
    distri(i,:) = sum(ProbZTmp_update_norm_sum_tmp);
%   figure(1),hold on; plot(disparityRng, sum(ProbZTmp_update_norm_sum));hold on;plot(disparityRng,sum(ProbZTmp_update_norm_sum_tmp));legend('refined disp','gt disp');
% figure(1),hold on;plot(disparityRng,sum(ProbZTmp_update_norm_sum_tmp));legend('gt disp');
%     drawnow;
end
figure(2);clf;hold on;subplot(1,2,1);plot(disparityRng,distri');subplot(1,2,2);plot(disparityRng,sum(distri));
 saveas(gcf,fullfile(inputDir,sprintf('final_error_%05d.fig',0+15)));
end

function depthGTInd11 = RoundingDispErr(DispRngCenter,disp2round, DispRngStep,DispRng)
disparityError = DispRngCenter - disp2round;

disparityErrorRound = round(disparityError./DispRngStep).*DispRngStep;


depthGTOfst11 = round(-(disparityErrorRound)./DispRngStep);
depthGTInd11 = depthGTOfst11 + (size(DispRng,2)+1)/2;
depthGTInd11(depthGTInd11 < 1) = 1;
depthGTInd11(depthGTInd11 > size(DispRng,2)) = size(DispRng,2);
end