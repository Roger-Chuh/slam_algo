function EvalLKConfig4(inputDir1, inputDir2)
% 14 14 |  5 1
% EvalLKConfig2('E:\bk_20180627\SLAM\slam_algo\prob\20200602_154951====20200325_1423_5000_large_room_4_counter_clock_640====0.5_2_2_20____20200602_154951\20200602_162443', 'E:\bk_20180627\SLAM\slam_algo\prob\20200602_165449====20200325_1423_5000_large_room_4_counter_clock_640====0.5_2_2_20____20200602_165449\20200602_170123');
% 14*3 14 |  0 5
% EvalLKConfig2('E:\bk_20180627\SLAM\slam_algo\prob\20200602_154951====20200325_1423_5000_large_room_4_counter_clock_640====0.5_2_2_20____20200602_154951\20200602_155407', 'E:\bk_20180627\SLAM\slam_algo\prob\20200602_154951====20200325_1423_5000_large_room_4_counter_clock_640====0.5_2_2_20____20200602_154951\20200602_162443')
% 14*3 14 |  0 1
% EvalLKConfig2('E:\bk_20180627\SLAM\slam_algo\prob\20200602_154951====20200325_1423_5000_large_room_4_counter_clock_640====0.5_2_2_20____20200602_154951\20200602_155407', 'E:\bk_20180627\SLAM\slam_algo\prob\20200602_165449====20200325_1423_5000_large_room_4_counter_clock_640====0.5_2_2_20____20200602_165449\20200602_170123')


if ~exist(fullfile(inputDir1,'tempData.mat'))
    [XErrMat_body1, YErrMat_body1, thetaList1, traceInfoMat1, XErrMat_body_stack1, YErrMat_body_stack1, imgL1, validCnt1, xErrMat1, yErrMat1,xErrMatFull1,yErrMatFull1] = CalibB2C6(inputDir1, 1, 300000);
    [XErrMat_body2, YErrMat_body2, thetaList2, traceInfoMat2, XErrMat_body_stack2, YErrMat_body_stack2, imgL2, validCnt2, xErrMat2, yErrMat2,xErrMatFull2,yErrMatFull2] = CalibB2C6(inputDir2, 1, 100000);
    save(fullfile(inputDir1,'tempData.mat'), 'XErrMat_body1','YErrMat_body1','thetaList1','traceInfoMat1','XErrMat_body_stack1','YErrMat_body_stack1','imgL1','validCnt1','xErrMat1', 'yErrMat1','xErrMatFull1','yErrMatFull1','XErrMat_body2','YErrMat_body2','thetaList2','traceInfoMat2','XErrMat_body_stack2','YErrMat_body_stack2','imgL2','validCnt2','xErrMat2', 'yErrMat2','xErrMatFull2','yErrMatFull2');
else
    load(fullfile(inputDir1,'tempData.mat'));
end

evenErr1 = EvenWeightErr(xErrMat1);
evenErr2 = EvenWeightErr(xErrMat2);

evenErrWhole1 = EvenWeightErr(xErrMatFull1);
evenErrWhole2 = EvenWeightErr(xErrMatFull2);


traceNum1 = size(XErrMat_body1,2);
traceNum2 = size(XErrMat_body2,2);

weightMatNorm = CalcWeight(abs(xErrMat1), 1, 1, 1);
weightMatNormTmp = CalcWeight(abs(xErrMat1), 0, 1, 1);

weightMatNorm1 = CalcWeight(abs(xErrMat1), 1, 1, 1,1);
weightMatNorm3 = CalcWeight(abs(xErrMat1), 1, 1, 1,3);
weightMatNorm51 = CalcWeight(abs(xErrMat1), 1, 1, 1,60);

er1 = max(max(abs(weightMatNorm - weightMatNorm1)));
er2 = max(max(abs(weightMatNormTmp - weightMatNorm51)));

if 0
    for u = 1 : traceNum2
        weightMatNorm_temp = CalcWeight(abs(xErrMat2), 1, 1, 1,u);
        x_err1(:,u) = cumsum(sum(weightMatNorm_temp.*xErrMat2)');
        
        
    end
    x_err1_upper = max(x_err1')';
    x_err1_lower = min(x_err1')';
else
    [x_err1, x_err1_upper, x_err1_lower] = CalcEnvelope(xErrMat1, traceNum1);
    [x_err2, x_err2_upper, x_err2_lower] = CalcEnvelope(xErrMat2, traceNum2);
    
    figure,subplot(2,1,1);plot(rad2deg(thetaList1(1:size(x_err1,1))),x_err1);hold on;plot(rad2deg(thetaList1(1:size(x_err1,1))),[x_err1_upper x_err1_lower],'-xb');plot(rad2deg(thetaList1(1:size(evenErr1,1))),[evenErr1],'-xr');
           subplot(2,1,2);plot(rad2deg(thetaList2(1:size(x_err2,1))),x_err2);hold on;plot(rad2deg(thetaList2(1:size(x_err2,1))),[x_err2_upper x_err2_lower],'-xb');plot(rad2deg(thetaList2(1:size(evenErr2,1))),[evenErr2],'-xr');
end
% evenErrWhole1 = EvenWeightErr(xErrMatFull1);
% evenErrWhole2 = EvenWeightErr(xErrMatFull2);


if 0
    figure,plot(rad2deg(thetaList1(1:size(XErrMat_body1,1))), cumsum(XErrMat_body1(:,[2 end])));hold on;plot(rad2deg(thetaList2(1:size(XErrMat_body2,1))),cumsum(XErrMat_body2(:,[2 end])));
    plot(rad2deg(thetaList2(1:len)),cumsum(mean([tail31(1:len) tail32(1:len) tail33(1:len)]')));
    plot(rad2deg(thetaList2(1:len)),cumsum(mean([tail31_1(1:len) tail32_1(1:len) tail33_1(1:len)]')));
    %     plot(rad2deg(thetaList1(1:len)),cumsum(mean([tail31(1:len) tail32(1:len) tail33(1:len)]')));
    %     plot(rad2deg(thetaList1(1:len)),cumsum(mean([tail31(1:len) tail32(1:len) tail33(1:len)])));
    legend('1 head','1 tail','5 head','5 tail','1 tail 3');
else
    %     figure(4);clf,
    figure,plot(rad2deg(thetaList1(1:size(XErrMat_body1,1))), cumsum(XErrMat_body1(:,[2 end])));hold on;
    plot(rad2deg(thetaList2(1:size(XErrMat_body2,1))),cumsum(XErrMat_body2(:,[2 end])));
    plot(rad2deg(thetaList1(1:size(XErrMat_body1,1))), evenErrWhole1(1:size(XErrMat_body1,1)));
    plot(rad2deg(thetaList2(1:size(XErrMat_body2,1))), evenErrWhole2(1:size(XErrMat_body2,1)));
    legend('1 head','1 tail','5 head','5 tail','1 whole','5 whole');
end



img2 = imgL2(:,:,1);
for k = 1 : size(imgL1,3)
    err = img2 - imgL1(:,:,k);
    if max(max(abs(err))) == 0
        startId = k;
        break;
    end
end
% startId = 1;
XErrMat_body1 = XErrMat_body1(startId:end,:);
YErrMat_body1 = YErrMat_body1(startId:end,:);
thetaList1 = thetaList1(startId:end,:);thetaList1 = thetaList1 - thetaList1(1);
XErrMat_body_stack1 = XErrMat_body_stack1(startId:end,:);
YErrMat_body_stack1 = YErrMat_body_stack1(startId:end,:);
imgL1 = imgL1(:,:,startId:end);

validCnt1 = validCnt1 - (startId - 1);
validCnt1= validCnt1(validCnt1 > 0);

imgL1 = imgL1(:,:,validCnt1);
imgL2 = imgL2(:,:,validCnt2);

validCnt22 = 1+3.*(validCnt2-1);
[ind1, ind2, err] = intersection(validCnt1,validCnt22);
if 1
    XErrMat_body1 = XErrMat_body1(ind1,:);
    YErrMat_body1 = YErrMat_body1(ind1,:);
    thetaList1 = thetaList1(ind1,:);thetaList1 = thetaList1 - thetaList1(1);
    XErrMat_body_stack1 = XErrMat_body_stack1(ind1,:);
    YErrMat_body_stack1 = YErrMat_body_stack1(ind1,:);
    imgL1 = imgL1(:,:,ind1);
    
    XErrMat_body2 = XErrMat_body2(ind2,:);
    YErrMat_body2 = YErrMat_body2(ind2,:);
    thetaList2 = thetaList2(ind2,:);thetaList2 = thetaList2 - thetaList2(1);
    XErrMat_body_stack2 = XErrMat_body_stack2(ind2,:);
    YErrMat_body_stack2 = YErrMat_body_stack2(ind2,:);
    imgL2 = imgL2(:,:,ind2);
end



XErrMat_body1_k2c = cumsum(XErrMat_body1')';
XErrMat_body1_k2c;
tail3_ = sum(XErrMat_body1(:,end-2:end)')';
tail31 = tail3_(1:3:end);
tail32 = tail3_(2:3:end);
tail33 = tail3_(3:3:end);
tail3_1 = 3.*(XErrMat_body1(:,end)')';
tail31_1 = tail3_1(1:3:end);
tail32_1 = tail3_1(2:3:end);
tail33_1 = tail3_1(3:3:end);
len = min([length(tail31) length(tail32) length(tail33) length(thetaList2)]);


angList1 = [0 :2.8/3: (size(XErrMat_body1,2)-1)*2.8/3];
angList2 = [0 :2.8: (size(XErrMat_body2,2)-1)*2.8];

angList1 = round(angList1, 5);
angList2 = round(angList2, 5);

Num = min(size(XErrMat_body1,1), size(XErrMat_body2,1)) - 10;
cnt = 1;err_detect = [];
for i = 1 : Num
    try
        %         tempX1 = XErrMat_body_stack1(1+3*(i-1),:); tempY1 = YErrMat_body_stack1(1+3*(i-1),:);
        tempX1 = XErrMat_body_stack1(i,:); tempY1 = YErrMat_body_stack1(i,:);
        tempX2 = XErrMat_body_stack2(i,:); tempY2 = XErrMat_body_stack2(i,:);
        key1 = tempX1{1,1};
        key2 = tempX2{1,1};
        
        ptIcs = detectFASTFeatures(imgL1(:,:,i),'MinQuality',0.01,'MinContrast',0.01);
        pt = ptIcs.Location;
        err_detect = [err_detect;[(size(intersect(pt, key1,'rows'),1)) - size(key1,1) (size(intersect(pt, key2,'rows'),1) - size(key2,1))]];
        
        if 0
            figure,imshow([imgL1(:,:,i);imgL2(:,:,i)]);hold on;plot(key1(:,1), key1(:,2),'or');plot(key2(:,1), key2(:,2),'.g')
            %                    subplot(2,1,2);imshow(imgL2(:,:,i));hold on;
        end
        
        
        tempX1{1,1} = single(zeros(size(key1,1),1));
        tempX2{1,1} = single(zeros(size(key2,1),1));
        tempX1 = cell2mat(tempX1);
        tempX2 = cell2mat(tempX2);
        
        %     [trailMatUse, aa] = SortData({[tempX1]; [tempX2]});
        [ind1, ind2, err] = intersection(key1,key2);
        temp1 = tempX1(ind1,2:end);
        temp2 = tempX2(ind2,2:end);
        
        
        
        tempX1_ = tempX1(ind1,2:end);
        tempX11 = reshape(tempX1_, size(tempX1_,1), 3, []);
        tempX11 = permute(tempX11, [2 1 3]);
        
        
        tempX11_mean = StepMean(tempX11, 3);
        tempX11_mean_stack(cnt,:) = mean(tempX11_mean);
        
        if isempty(ind1)
            continue;
        end
        
        commonSize(i,1) = length(ind1);
        
        if 0
            checkId = 1;figure,plot(angList1(2:end), mean(tempX1_(checkId,:),1),'-x');hold on;plot(angList2(2:end-(length(angList2)-size(tempX11_mean,2))+1), mean(tempX11_mean(checkId,:),1)./3,'-x');
            figure,plot(angList1(2:end), mean(tempX1_),'-x');hold on;plot(angList2(2:end-(length(angList2)-size(tempX11_mean,2))+1), mean(tempX11_mean),'-x');
            %             subplot(1,2,2),plot(angList1(2:end), mean(temp1_inv),'-x');hold on;plot(angList2(2:end), mean(tempX11_mean_inv),'-x')
        end
        
        
        if 0
            for j = 1 : size(temp1,1)
                figure(11),clf;
                plot(angList1(2:end), temp1(j,:),'-xr');hold on;plot(angList2(2:end), tempX11_mean(j,:),'-xg');hold on;plot(angList2(2:end), temp2(j,:),'-xb');
                legend(sprintf('CCW scale level: %d',1), sprintf('CCW scale level: %d',1), sprintf('CCW scale level: %d',5));
                drawnow;
            end
        end
        temp1_stack(cnt,:) = mean(temp1(:,:));
        temp2_stack(cnt,:) = mean(temp2(:,:));
        cnt = cnt +1;
    catch
        continue;
    end
end
% figure(1);clf,
figure,plot(rad2deg(thetaList1(1:size(XErrMat_body1,1))), cumsum(XErrMat_body1(:,[2 end])));hold on;
plot(rad2deg(thetaList2(1:size(tempX11_mean_stack,1))),cumsum(tempX11_mean_stack(:,[1 end])))
legend('1 head','1 tail','1 head','1 tail');
% plot(rad2deg(thetaList2(1:size(XErrMat_body2,1))),cumsum(XErrMat_body2(:,[2 end])));
% legend('1 head','1 tail','1 head','1 tail','5 head','5 tail');


return;
CCW = false; true;

load(fullfile(inputDir1, 'congfigInfo2.mat'));
congfigInfo1 = congfigInfo;
angList1 = [0 :2.8/congfigInfo.angSize: 14*2.8];
angList1 = angList1(1:14*congfigInfo.angSize);
angList1 = round(angList1,5);
X_err_stack1 = X_err_stack; Y_err_stack1 = Y_err_stack;
X_err_stack1_inv = X_err_stack_inv; Y_err_stack1_inv = Y_err_stack_inv;

try
    validCnt1 = validCnt;
catch
    saghk = 1;
end

load(fullfile(inputDir2, 'congfigInfo2.mat'));
congfigInfo2 = congfigInfo;
angList2 = [0 :2.8/congfigInfo.angSize: 14*2.8];
angList2 = angList2(1:14*congfigInfo.angSize);
angList2 = round(angList2,5);
X_err_stack2 = X_err_stack; Y_err_stack2 = Y_err_stack;
X_err_stack2_inv = X_err_stack_inv; Y_err_stack2_inv = Y_err_stack_inv;


try
    validCnt2 = validCnt;
catch
    saghk = 1;
end


commAng = intersect(angList1, angList2);
[angId1, angId2, angErr] = intersection(angList1' ,angList2');

scale = length(angList1)/length(angList2);

Num = min(size(X_err_stack1,1), size(X_err_stack2,1));

cnt = 1;
for i = 1 : Num
    tempX1 = X_err_stack1{i,1}; tempY1 = Y_err_stack1{i,1};
    tempX2 = X_err_stack2{i,1}; tempY2 = Y_err_stack2{i,1};
    
    %     [trailMatUse, aa] = SortData({[tempX1]; [tempX2]});
    [ind1, ind2, err] = intersection(tempX1(:,1),tempX2(:,1));
    temp1 = tempX1(ind1,2:end);
    temp2 = tempX2(ind2,2:end);
    
    if isempty(ind1)
        continue;
    end
    
    if scale > 1
        tempX1_ = tempX1(ind1,2:end-scale+1);
        tempX11 = reshape(tempX1_, size(tempX1_,1), scale, []);
        tempX11 = permute(tempX11, [2 1 3]);
        
        
        if 0
            tempX11_mean = (scale).*squeeze(min(tempX11));
        elseif 0
            tempX11_temp = abs(tempX11);
            [aa, id1_temp] = min(tempX11_temp);
            tempX11_temp2 = reshape(tempX11_temp, scale,[]);
            tempX11 = reshape(tempX11,  scale,[]);
            [aa,idaa] = min(tempX11_temp2);
            minIdList = [[1 : size(tempX11_temp2,2)]' idaa'];
            idList = sub2ind(size(tempX11_temp2), minIdList(:,2), minIdList(:,1));
            err_ = aa' - tempX11_temp2(idList);
            tempX11_mean = scale.*reshape(tempX11(idList), size(tempX11_temp,2),[]);
            if 0
                tempX11_sign = sign(tempX11);
                tempX11_sign_temp = tempX11_sign(id1_temp);
                tempX11_mean = (scale).*squeeze(tempX11_sign_temp.*tempX11(id1_temp));
            end
        else
            tempX11_mean = StepMean(tempX11, scale);
        end
        
        tempX11_mean = tempX11_mean(:,1:end);
        tempX22_mean = temp2(:,1:end);
    else
        tempX2_ = tempX2(ind2,2:end - scale+1);
        tempX22 = reshape(tempX2_, size(tempX2_,1), 1/scale, []);
        tempX22 = permute(tempX22, [2 1 3]);
        %         tempX22_temp = abs(tempX22);
        if 0
            tempX22_mean = (1/scale).*squeeze(min(tempX22));
        elseif 0
            tempX22_temp = abs(tempX22);
            [~, id2_temp] = min(tempX22_temp);
            tempX22_sign = sign(tempX22);
            tempX22_sign_temp = tempX22_sign(id2_temp);
            tempX22_mean = (scale).*squeeze(tempX22_sign_temp.*tempX22(id2_temp));
        else
            tempX22_mean = StepMean(tempX22, 1/scale);
        end
        
        
        tempX22_mean = tempX22_mean(:,1:end);
        tempX11_mean = temp1(:,1:end);
    end
    
    sgfjk = 1;
    
    if 0 % CCW
        figure(1),
        for j = 1 : size(temp1,1)
            clf;
            plot(angList1(2:end), temp1(j,:),'-xr');hold on;plot(angList2(2:end), temp2(j,:),'-xb');
            legend(num2str(congfigInfo1.lkConfig.pyrL), num2str(congfigInfo2.lkConfig.pyrL));
            drawnow;
        end
    end
    %     for kk = 1 : mum
    %        temp2_inv_stack = [];
    tempX1_inv = X_err_stack1_inv{i,1}; tempY1_inv = Y_err_stack1_inv{i,1};
    tempX2_inv = X_err_stack2_inv{i,1}; tempY2_inv = Y_err_stack2_inv{i,1};
    
    [ind1_inv, ind2_inv, err_inv] = intersection(tempX1_inv(:,1),tempX2_inv(:,1));
    temp1_inv = tempX1_inv(ind1_inv,2:end);
    temp2_inv = tempX2_inv(ind2_inv,2:end);
    
    if isempty(ind1_inv)
        continue;
    end
    
    
    
    if scale > 1
        tempX1_inv_ = tempX1_inv(ind1_inv,2:end-scale+1);
        tempX11_inv = reshape(tempX1_inv_, size(tempX1_inv_,1), scale, []);
        tempX11_inv = permute(tempX11_inv, [2 1 3]);
        %         tempX11_inv_temp = abs(tempX11_inv);
        
        if 0
            tempX11_mean_inv = scale.*squeeze(min(tempX11_inv));
        else
            tempX11_mean_inv = StepMean(tempX11_inv, scale);
        end
        
        tempX11_mean_inv = tempX11_mean_inv(:,1:end);
        tempX22_mean_inv = temp2_inv(:,1:end);
    else
        tempX2_inv_ = tempX2_inv(ind2_inv,2:end-scale+1);
        tempX22_inv = reshape(tempX2_inv_, size(tempX2_inv_,1), 1/scale, []);
        tempX22_inv = permute(tempX22_inv, [2 1 3]);
        %         tempX22_inv_temp = abs(tempX22_inv);
        
        if 0
            tempX22_mean_inv = (1/scale).*squeeze(min(tempX22_inv));
        else
            tempX22_mean_inv = StepMean(tempX22_inv, 1/scale);
        end
        tempX22_mean_inv = tempX22_mean_inv(:,1:end);
        tempX11_mean_inv = temp1_inv(:,1:end);
    end
    
    temp1_stack(cnt,:) = mean(temp1(:,:));
    temp2_stack(cnt,:) = mean(temp2(:,:));
    
    temp1_inv_stack(cnt,:) = mean(temp1_inv(:,:));
    temp2_inv_stack(cnt,:) = mean(temp2_inv(:,:));
    
    
    
    temp11_stack(cnt,:) = mean(tempX11_mean(:,:));
    temp22_stack(cnt,:) = mean(tempX22_mean(:,:));
    
    temp11_inv_stack(cnt,:) = mean(tempX11_mean_inv(:,:));
    temp22_inv_stack(cnt,:) = mean(tempX22_mean_inv(:,:));
    
    
    id_inv1 = find(temp2_inv(:,1)>0);
    id_inv2 = find(temp2_inv(:,1)<0);
    cnt = cnt + 1;
    if 0
        figure(4),clf; subplot(1,3,1);plot(angList1(2:end), mean(temp1_inv(id_inv1,:)),'-xr');hold on;plot(angList2(2:end), mean(temp2_inv(id_inv1,:)),'-xb');
        legend(num2str(congfigInfo1.lkConfig.pyrL), num2str(congfigInfo2.lkConfig.pyrL));
        subplot(1,3,2);plot(angList1(2:end), mean(temp1_inv(id_inv2,:)),'-xr');hold on;plot(angList2(2:end), mean(temp2_inv(id_inv2,:)),'-xb');
        legend(num2str(congfigInfo1.lkConfig.pyrL), num2str(congfigInfo2.lkConfig.pyrL));
        subplot(1,3,3);plot(angList1(4:end-2), mean(temp1_inv(:,3:end-2)),'-xr');hold on;plot(angList2(2:end), mean(temp2_inv(:,:)),'-xb');
        legend(num2str(congfigInfo1.lkConfig.pyrL), num2str(congfigInfo2.lkConfig.pyrL));
        drawnow;
    end
    
    asfkbj = 1;
    
    if 0
        figure(2),
        for j = 1 : size(temp1_inv,1)
            clf;
            plot(angList1(2:end), temp1_inv(j,:),'-xr');hold on;plot(angList2(2:end), temp2_inv(j,:),'-xb');
            legend(num2str(congfigInfo1.lkConfig.pyrL), num2str(congfigInfo2.lkConfig.pyrL));
            drawnow;
        end
    end
    checkId = 5;
    if 0
        if scale > 1
            figure,subplot(1,2,1);plot(angList1(2:end), mean(temp1),'-x');hold on;plot(angList2(2:end), mean(tempX11_mean),'-x');
            subplot(1,2,2),plot(angList1(2:end), mean(temp1_inv),'-x');hold on;plot(angList2(2:end), mean(tempX11_mean_inv),'-x')
            
            figure,subplot(1,2,1);plot(angList1(2:end), mean(temp1(checkId,:),1),'-x');hold on;plot(angList2(2:end), mean(tempX11_mean(checkId,:),1)./scale,'-x');subplot(1,2,2),plot(angList1(2:end), mean(temp1_inv(checkId,:),1),'-x');hold on;plot(angList2(2:end), mean(tempX11_mean_inv(checkId,:),1)./scale,'-x');
        else
            figure,subplot(1,2,1);plot(angList2(2:end), mean(temp2),'-x');hold on;plot(angList1(2:end), mean(tempX22_mean),'-x');
            subplot(1,2,2),plot(angList2(2:end), mean(temp2_inv),'-x');hold on;plot(angList1(2:end), mean(tempX22_mean_inv),'-x')
            
            figure,subplot(1,2,1);plot(angList2(2:end), mean(temp2(checkId,:),1),'-x');hold on;plot(angList1(2:end), mean(tempX22_mean(checkId,:),1).*scale,'-x');subplot(1,2,2),plot(angList2(2:end), mean(temp2_inv(checkId,:),1),'-x');hold on;plot(angList1(2:end), mean(tempX22_mean_inv(checkId,:),1).*scale,'-x');
        end
    end
    afsdkgj = 1;
end



if 0
    figure,subplot(1,2,1);plot(angList1(2:end), mean(temp1_stack(:,:)),'-xr');hold on;plot(angList2(2:end), mean(temp2_stack(:,:)),'-xb');
    legend(num2str(congfigInfo1.lkConfig.pyrL), num2str(congfigInfo2.lkConfig.pyrL));title('counter clock wise');
    subplot(1,2,2);plot(angList1(2:end), mean(temp1_inv_stack(:,:)),'-xr');hold on;plot(angList2(2:end), mean(temp2_inv_stack(:,:)),'-xb');
    legend(num2str(congfigInfo1.lkConfig.pyrL), num2str(congfigInfo2.lkConfig.pyrL));title('clock wise');
else
    
    
    figure,plot(angList1(2:end), mean(temp1_stack(:,:)),'-xr');hold on;plot(angList2(2:end), mean(temp2_stack(:,:)),'-xb');
    %     legend(num2str(congfigInfo1.lkConfig.pyrL), num2str(congfigInfo2.lkConfig.pyrL));
    plot(angList1(2:end), mean(temp1_inv_stack(:,:)),'-xm');hold on;plot(angList2(2:end), mean(temp2_inv_stack(:,:)),'-xg');
    
    title(sprintf('red and blue stand for CCW\npink and green stand for CW'));
    
    if scale > 1
        plot(commAng(2:end), mean(temp11_stack),'-xk');
        plot(commAng(2:end), mean(temp11_inv_stack),'-xc');
        
        legend(sprintf('CCW scale level: %d',congfigInfo1.lkConfig.pyrL),...
            sprintf('CCW scale level: %d',congfigInfo2.lkConfig.pyrL),...
            sprintf('CW scale level: %d',congfigInfo1.lkConfig.pyrL),...
            sprintf('CW scale level: %d',congfigInfo2.lkConfig.pyrL),...
            sprintf('CCW scale level: %d',congfigInfo1.lkConfig.pyrL),...
            sprintf('CW scale level: %d',congfigInfo1.lkConfig.pyrL));
        
        figure,subplot(2,2,1),hist(abs(temp11_stack(:,1)) - abs(temp2_stack(:,1)), 100);title('CCW head frame'); grid on; legend('1deg - 3deg');
        subplot(2,2,2);hist(abs(temp11_inv_stack(:,1)) - abs(temp2_inv_stack(:,1)),100);title('CW head frame'); grid on;
        subplot(2,2,3),hist(abs(temp11_stack(:,end)) - abs(temp2_stack(:,end)), 100);title('CCW tail frame'); grid on;
        subplot(2,2,4);hist(abs(temp11_inv_stack(:,end)) - abs(temp2_inv_stack(:,end)),100);title('CW tail frame'); grid on;
        
    else
        plot(commAng(2:end), mean(temp22_stack),'-xk');
        plot(commAng(2:end), mean(temp22_inv_stack),'-xc');
        
        legend(sprintf('CCW scale level: %d',congfigInfo1.lkConfig.pyrL),...
            sprintf('CCW scale level: %d',congfigInfo2.lkConfig.pyrL),...
            sprintf('CW scale level: %d',congfigInfo1.lkConfig.pyrL),...
            sprintf('CW scale level: %d',congfigInfo2.lkConfig.pyrL),...
            sprintf('CCW scale level: %d',congfigInfo2.lkConfig.pyrL),...
            sprintf('CW scale level: %d',congfigInfo2.lkConfig.pyrL));
        
        
        figure,subplot(2,2,1),hist(abs(temp22_stack(:,1)) - abs(temp1_stack(:,1)), 100);title('CCW head frame'); grid on; legend('1deg - 3deg');
        subplot(2,2,2);hist(abs(temp22_inv_stack(:,1)) - abs(temp1_inv_stack(:,1)),100);title('CW head frame'); grid on;
        subplot(2,2,3),hist(abs(temp22_stack(:,end)) - abs(temp1_stack(:,end)), 100);title('CCW tail frame'); grid on;
        subplot(2,2,4);hist(abs(temp22_inv_stack(:,end)) - abs(temp1_inv_stack(:,end)),100);title('CW tail frame'); grid on;
        
    end
    
    
    
    
    
    
    
end


% plot(angList1(4:end-2), mean(temp1_inv_stack(:,3:end-2)),'-xr');hold on;plot(angList2(2:end), mean(temp2_inv_stack(:,:)),'-xb');
%             legend(num2str(congfigInfo1.lkConfig.pyrL), num2str(congfigInfo2.lkConfig.pyrL));



end
function [trailMatUse, aa] = SortData(xStack)
for i = 1 : size(xStack,1)
    if i == 1
        commId =  xStack{i,1}(:,1);
    else
        commId = intersect(commId, xStack{i,1}(:,1));
    end
    
end

trailMat = [];
for i = 1 :size(xStack,1)
    inId = xStack{i,1}(:,1);
    inlier = find(ismember(inId, commId));
    trailMat(:,:,i) =  xStack{i,1}(inlier,1:end);
    
end
aa = squeeze(trailMat(:,1,:));
trailMatUse = trailMat(:,2:end,:);
end

function [ind1, ind2, err] = intersection(id1,id2)
[~,ind1, ind2] = intersect(id1, id2,'rows');

[~,ind11] = sort(ind1);
ind1 = ind1(ind11,:);
ind2 = ind2(ind11,:);
err = id1(ind1,:) - id2(ind2,:);
end

function [tempX11_mean, err_] = StepMean(tempX11, scale)
tempX11_bak = tempX11;

tempX11_temp = abs(tempX11);
% [aa, id1_temp] = min(tempX11_temp);
tempX11_temp2 = reshape(tempX11_temp, scale,[]);
tempX11 = reshape(tempX11,  scale,[]);
%%
[aa,idaa] = min(tempX11_temp2);

minIdList = [[1 : size(tempX11_temp2,2)]' idaa'];
idList = sub2ind(size(tempX11_temp2), minIdList(:,2), minIdList(:,1));
err_ = aa' - tempX11_temp2(idList);
tempX11_mean = scale.*reshape(tempX11(idList), size(tempX11_temp,2),[]);

end
function [traceX, traceY] = ReadData(inputDir, startt, endd)
dirInfo = dir(fullfile(inputDir, 'Replay*.mat'));
load(fullfile(inputDir, 'judgement.mat'));

endd = min(endd, (size(dirInfo,1) - 1));


for i = startt : endd
    load(fullfile(inputDir, dirInfo(i).name));
    
    
    traceX(data{4}(:,1),cnt) = data{4}(:,2);
    traceY(data{4}(:,1),cnt) = data{4}(:,3);
    thetaList(cnt,1) = data{6};
    depthGT(:,:,cnt) = data{5};
    imgL = data{1};
    Img(:,:,cnt) = rgb2gray(imgL);
    cnt = cnt + 1;
end

end
function evenErr = EvenWeightErr(xErrMatFull)

validAngOptFull2 = abs(xErrMatFull) > 0;
validAngOptSumFull2 = sum(validAngOptFull2);
xErrMatAccumFull = (sum(xErrMatFull)./validAngOptSumFull2)';

evenErr = cumsum(xErrMatAccumFull);

end

function [x_err1, x_err1_upper, x_err1_lower] = CalcEnvelope(xErrMat2, traceNum2)
for u = 1 : traceNum2
    weightMatNorm_temp = CalcWeight(abs(xErrMat2), 1, 1, 1,u);
    x_err1(:,u) = cumsum(sum(weightMatNorm_temp.*xErrMat2)');
    
    
end
x_err1_upper = max(x_err1')';
x_err1_lower = min(x_err1')';
end