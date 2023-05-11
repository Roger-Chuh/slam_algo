function ParticularFeatUpdate(startId, LocalTrace, fId, depthGTInd_update, disparityRng, figId, curUpdateRatio, holdonFlag,angSampInd_gt_list)
% d

try
    for vn = startId : size(LocalTrace.probP2,1)
        if vn == startId
            figure(figId),clf;
            K = [];
        end
        
        [probP2Mat22Sum, K] = Func(K, LocalTrace.probP2(startId:vn,:),fId, depthGTInd_update, disparityRng, figId, curUpdateRatio, holdonFlag, angSampInd_gt_list,vn);
    end
catch
    
    for vn = startId : size(LocalTrace,1)
        if vn == startId
            figure(figId),clf;
            K = [];
        end
        
        [probP2Mat22Sum, K] = Func(K, LocalTrace(startId:vn,:),fId, depthGTInd_update, disparityRng, figId, curUpdateRatio, holdonFlag, angSampInd_gt_list, vn);
    end
    
end

figure(22),clf;subplot(1,2,1);plot((abs([0;K])),'-x');subplot(1,2,2);plot(diff(abs([0;K])),'-x');
end


function [probP2Mat22Sum, K] = Func(K, probP2, fId, depthGTInd_update,disparityRng,figId, curUpdateRatio, holdonFlag, angSampInd_gt_list,vn0)


% holdonFlag = false;  true; false; true;

probP2Mat = []; probP2Mat2 = []; probP2Mat3 = []; probP2Mat_new = [];
dispMat = []; dispMat2 = []; dispMat3 = []; dispMat_new = [];
for vn = 1 : size(probP2,1)
    % for vn = length(probP2) : length(probP2)
    probP2Mat = [probP2Mat; permute(probP2{vn, 1},[2 1 3])];
    probP2Mat2 = [probP2Mat2; probP2{vn, 1}];
    
    dispMat = [dispMat; permute(probP2{vn, 2},[2 1 3])];
    dispMat2 = [dispMat2; probP2{vn, 2}];
    
    if vn == 1
        angSampledNum = size(probP2Mat2,1);
        
    end
    
    
    if vn == 1
        probP2Mat_new = probP2{vn, 1};
        dispMat_new = probP2{vn, 2};
    else
        probP2Mat_new = (1 - curUpdateRatio).*probP2Mat_new + curUpdateRatio.*probP2{vn, 1};
        dispMat_new = (1 - curUpdateRatio).*dispMat_new + curUpdateRatio.*probP2{vn, 2};
    end
    
    
end
probP2Mat3 = probP2Mat2(end-angSampledNum+1:end,:,:);
dispMat3 = dispMat2(end-angSampledNum+1:end,:,:);

probP2Mat22 = permute(probP2Mat2, [1 3 2]);
dispMat22 = permute(dispMat2, [1 3 2]);

probP2Mat33 = permute(probP2Mat3, [1 3 2]);
dispMat33 = permute(dispMat3, [1 3 2]);

probP2Mat_new_2 = permute(probP2Mat_new, [1 3 2]);
dispMat_new_2 = permute(dispMat_new, [1 3 2]);

probP2Mat22Sum = sum(probP2Mat22);
probP2Mat22Sum = permute(probP2Mat22Sum, [3 2 1]);

dispMat22Sum = sum(dispMat22);
dispMat22Sum = permute(dispMat22Sum, [3 2 1]);

probP2Mat_new_2_sum = sum(probP2Mat_new_2);
probP2Mat_new_2_sum = permute(probP2Mat_new_2_sum, [3 2 1]);
probP2Mat_new_2_sum(probP2Mat_new_2_sum == 0) = 1;
probP2Mat_new_2_sum = probP2Mat_new_2_sum./repmat(max(probP2Mat_new_2_sum')',1,size(probP2Mat_new_2_sum,2));


probP2Mat22Sum(probP2Mat22Sum == 0) = 1;
probP2Mat22Sum = probP2Mat22Sum./repmat(max(probP2Mat22Sum')',1,size(probP2Mat22Sum,2));


tempProb = probP2Mat2(:,fId,:);tempProb = permute(tempProb, [1 3 2]); tempProb2 = sum(tempProb);tempProb2 = tempProb2./max(tempProb2);
tempProb3 = probP2Mat3(:,fId,:);tempProb3 = permute(tempProb3, [1 3 2]); tempProb4 = sum(tempProb3);tempProb4 = tempProb4./max(tempProb4);

tempProb_new = probP2Mat_new(:,fId,:);tempProb_new = permute(tempProb_new, [1 3 2]); tempProb_new_2 = sum(tempProb_new); tempProb_new_2 = tempProb_new_2./max(tempProb_new_2);


tempDispProb = dispMat2(:,fId,:);tempDispProb = permute(tempDispProb, [1 3 2]); tempDispProb2 = sum(tempDispProb);tempDispProb2 = tempDispProb2./max(tempDispProb2);
tempDispProb3 = dispMat3(:,fId,:);tempDispProb3 = permute(tempDispProb3, [1 3 2]); tempDispProb4 = sum(tempDispProb3);tempDispProb4 = tempDispProb4./max(tempDispProb4);

tempDispProb_new = dispMat_new(:,fId,:);tempDispProb_new = permute(tempDispProb_new, [1 3 2]); tempDispProb_new_2 = sum(tempDispProb_new); tempDispProb_new_2 = tempDispProb_new_2./max(tempDispProb_new_2);



figure(figId),subplot(3,3,1);cla;imshow(tempProb', []);
subplot(3,3,4),hold on;
if ~holdonFlag
    cla; hold on;
end
grid on;
plot(disparityRng, tempProb2,'LineWidth',2);hold on;plot(disparityRng(depthGTInd_update(fId)), tempProb2(depthGTInd_update(fId)),'or','MarkerSize', 5, 'LineWidth', 5); title(sprintf('featId: %d, gtId: %d',fId, depthGTInd_update(fId))); % title(num2str(depthGTInd_update(fId)));
subplot(3,3,2);cla;imshow(tempProb3', []);
subplot(3,3,5),hold on;
if ~holdonFlag
    cla; hold on;
end
grid on;
plot(disparityRng, tempProb4,'LineWidth',2);hold on;plot(disparityRng(depthGTInd_update(fId)), tempProb4(depthGTInd_update(fId)),'or','MarkerSize', 5, 'LineWidth', 5); title(sprintf('featId: %d, gtId: %d',fId, depthGTInd_update(fId)));
subplot(3,3,3);cla;imshow(tempProb_new', []);
subplot(3,3,6),hold on;
if ~holdonFlag
    cla;hold on;
end 
grid on;
plot(disparityRng, tempProb_new_2,'LineWidth',2);hold on;plot(disparityRng(depthGTInd_update(fId)), tempProb_new_2(depthGTInd_update(fId)),'or','MarkerSize', 5, 'LineWidth', 5); title(sprintf('featId: %d, gtId: %d',fId, depthGTInd_update(fId)));


subplot(3,3,7);imshow(tempDispProb3,[]);

subplot(3,3,8);cla; imshow(tempProb3,[]);hold on;


midCoord = [(size(tempProb3,2)+1)/2 (size(tempProb3,1)+1)/2];

plot(repmat(midCoord(1),size(tempProb3,1),1), [1:size(tempProb3,1)]','*r');
plot( [1:size(tempProb3,2)]', repmat(midCoord(2),size(tempProb3,2),1),'*r');
% % plot(depthGTInd_update(fId), angSampInd_gt_list(vn0),'*g');
% % plot(midCoord(1), midCoord(2), '*b')


lineVec = cross([midCoord 1], [depthGTInd_update(fId) angSampInd_gt_list(vn0) 1]);
points = lineToBorderPoints(lineVec, size(tempProb3));
line(points(:, [1,3])', points(:, [2,4])','Color',[0 0 0]);title(num2str((vn0)))
plot(depthGTInd_update(fId), angSampInd_gt_list(vn0),'*g');
plot(midCoord(1), midCoord(2), '*b')

k = -lineVec(1)/lineVec(2);
% K = [K; k];
subplot(3,3,9);plot(disparityRng, sum(tempDispProb3));grid on





tempProb3_bak = tempProb3;
tempProb3_bak = tempProb3_bak./max(tempProb3_bak(:));
tempProb3_bak = (255.*tempProb3_bak);
tempProb3_bak(find(tempProb3_bak < (255) & tempProb3_bak > (10))) = 255;
tempProb3_bak = uint8(tempProb3_bak);

line00 = lmatch_detect_lines_use(tempProb3_bak, 3, 3.1,  1);
kkkk = mean(abs((line00(:,4) - line00(:,2))./(line00(:,3) - line00(:,1))));
K = [K; kkkk];


drawnow;
pause(0.2);

end