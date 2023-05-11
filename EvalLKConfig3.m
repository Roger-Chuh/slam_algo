function EvalLKConfig3(XErrMat_body1, thetaList1, XErrMat_body2, thetaList2)
% 14 14 |  5 1
% EvalLKConfig2('E:\bk_20180627\SLAM\slam_algo\prob\20200602_154951====20200325_1423_5000_large_room_4_counter_clock_640====0.5_2_2_20____20200602_154951\20200602_162443', 'E:\bk_20180627\SLAM\slam_algo\prob\20200602_165449====20200325_1423_5000_large_room_4_counter_clock_640====0.5_2_2_20____20200602_165449\20200602_170123');
% 14*3 14 |  0 5
% EvalLKConfig2('E:\bk_20180627\SLAM\slam_algo\prob\20200602_154951====20200325_1423_5000_large_room_4_counter_clock_640====0.5_2_2_20____20200602_154951\20200602_155407', 'E:\bk_20180627\SLAM\slam_algo\prob\20200602_154951====20200325_1423_5000_large_room_4_counter_clock_640====0.5_2_2_20____20200602_154951\20200602_162443')
% 14*3 14 |  0 1
% EvalLKConfig2('E:\bk_20180627\SLAM\slam_algo\prob\20200602_154951====20200325_1423_5000_large_room_4_counter_clock_640====0.5_2_2_20____20200602_154951\20200602_155407', 'E:\bk_20180627\SLAM\slam_algo\prob\20200602_165449====20200325_1423_5000_large_room_4_counter_clock_640====0.5_2_2_20____20200602_165449\20200602_170123')


angList1 = [0 :2.8/3: (size(XErrMat_body1,2)-1)*2.8/3];
angList2 = [0 :2.8: (size(XErrMat_body2,2)-1)*2.8];

angList1 = round(angList1, 5);
angList2 = round(angList2, 5);


tempX1_ = XErrMat_body1(:,2:end);
tempX11 = reshape(tempX1_, size(tempX1_,1), 3, []);
tempX11 = permute(tempX11, [2 1 3]);


tempX11_mean = StepMean(tempX11, 3);

tempX22_mean = XErrMat_body2(:,1:end);

if 0
    checkId = 15;figure,plot(angList1(2:end), mean(tempX1_(checkId,:),1),'-x');hold on;plot(angList2(2:end-(length(angList2)-size(tempX11_mean,2))+1), mean(tempX11_mean(checkId,:),1)./3,'-x');
    figure,plot(angList1(2:end), mean(tempX1_),'-x');hold on;plot(angList2(2:end-(length(angList2)-size(tempX11_mean,2))+1), mean(tempX11_mean),'-x');
    %             subplot(1,2,2),plot(angList1(2:end), mean(temp1_inv),'-x');hold on;plot(angList2(2:end), mean(tempX11_mean_inv),'-x')
end
figure,plot(angList1(2:end), mean(tempX1_,1),'-xr');hold on;plot(angList2(2:end-(length(angList2)-size(tempX11_mean,2))+1), mean(tempX11_mean,1),'-xg');
plot(angList2(2:end), mean(XErrMat_body2(:,2:end),1),'-xb');legend('scale level: 1', 'scale level: 1', 'scale level: 5');
return;
CCW = false; true;
if 0
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
end
cnt = 1;
for i = 1 : size(XErrMat_body1, 1)
    tempX1 = XErrMat_body1(i,:); 
    
 
    
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
ind1 = ind1(ind11);
ind2 = ind2(ind11);
err = id1(ind1) - id2(ind2);
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