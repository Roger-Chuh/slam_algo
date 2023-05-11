function [dispErrDistriMat1, dispErrExpMat1] = PlotDispErr(obj, figNum,range, varargin)


if (nargin <= 3)
    forceReturn = 0;
elseif (nargin == 4)
    forceReturn = varargin{1};
else
    error('Too many input arguments');
end






dispErrMargin = obj.configParam.disparity_error;

disparityRng = [-obj.configParam.disparity_error : obj.configParam.disparity_sample_step : obj.configParam.disparity_error];



dispErrExpStack = obj.dispErrExpStack3(:,1:6);

% dispErrExpStack = []
% for i = 1 : size()

frameId00 = cell2mat(dispErrExpStack(:,1));
dispErrExpStack(setdiff(1:size(dispErrExpStack,1), frameId00),:) = [];
if ~isempty(range)
    dispErrExpStack = dispErrExpStack(range(1):min(range(2), size(dispErrExpStack,1)),:);
end
frameId0 = cell2mat(dispErrExpStack(:,2));
%     frameId = [1:size(dispErrExpStack,1)]';

draw = 0;
calcAll = false; true; false;


[a, b, c] = unique(frameId0);
c_in_k_mat = {};
c_in_k_mat_all = {};
cAvg_in_k_mat = {};
k_in_k_mat = {};
dispErrDistriMat = [];
dispVar = {};
avg_gtDistri_sum_mat = [];
k_gtDistri_sum_mat = [];
dispErrExpMat = [];
% ofst = 0;
for i = 1 : length(a)
    id = find(frameId0 == a(i));
    tempMat = []; tempCell = {};
    frmId = [];
    for j = 1 : length(id)
        tempErr = dispErrExpStack(id(j), :);
        
        
        %         tempMat = [tempMat; [tempErr{2}]];
        %         tempMat = [tempMat; [tempErr{2} tempErr{2} - tempErr{1}]];
        %         tempMat = [tempMat; [tempErr{2} tempErr{1}]];
        tempMat = [tempMat; [tempErr{1}]];
        frmId = [frmId; tempErr{2} - tempErr{4} + 1];
        
        tp = (tempErr{1,3});
        
        for kk = 1 : size(tp,1)
            tp{kk,1} = single(tp{kk,1});
            tp{kk,2} = single(tp{kk,2});
            tp{kk,3} = single(tp{kk,3});
        end
        
        tpp = {};
        tpp = [{cell2mat(tp(:,1)')} {cell2mat(tp(:,2)')} {cell2mat(tp(:,3)')}];
        
        
        if ~calcAll
            tempCell = [tempCell; tempErr{1,3}(end,:)];
        else
            
            tempCell = [tempCell; tpp];
        end
        if ~calcAll
            c_in_k = single(tempErr{1,3}{end,1});
            cAvg_in_k = single(tempErr{1,3}{end,2});
            k_in_k = single(tempErr{1,3}{end,3});
        else
            c_in_k = single(cell2mat(tp(:,1)'));
            cAvg_in_k = single(cell2mat(tp(:,2)'));
            k_in_k = single(cell2mat(tp(:,3)'));
            
        end
        
        %         if draw
        
        kInk = cell2mat(tempErr{1,3}(end,3));
        avgInk = cell2mat(tempErr{1,3}(end,2));
        cInk = cell2mat(tempErr{1,3}(end,1));
        
        kInk_all = cell2mat(tp(:,3));
        
        kInk_all = kInk_all(2:end,:);
        
        
        avgInk_all = cell2mat(tp(:,2));
        
        avgInk_all = avgInk_all(2:end,:);
        
        cInk_all = cell2mat(tp(:,1));
        cInk_all = cInk_all(1:end-1,:);
        
        %         cInk_all = cInk_all(2:end,:);
        
        
        [avg_gt_distri, avg_gtDistri_sum] = CalcDispErrDistribution(avgInk', disparityRng);
        [k_gt_distri, k_gtDistri_sum] = CalcDispErrDistribution(kInk', disparityRng);
        avg_gtDistri_sum_mat = [avg_gtDistri_sum_mat; avg_gtDistri_sum];
        k_gtDistri_sum_mat = [k_gtDistri_sum_mat; k_gtDistri_sum];
        
        
        exp_old = dot(disparityRng, k_gtDistri_sum./sum(k_gtDistri_sum));
        exp_new = dot(disparityRng, avg_gtDistri_sum./sum(avg_gtDistri_sum));
        
        prob_old = interp1(disparityRng, k_gtDistri_sum, exp_old);
        prob_new = interp1(disparityRng, avg_gtDistri_sum, exp_new);
        dispErrExpMat = [dispErrExpMat; [tempMat(end) exp_old exp_new]];
        if draw
            figure(1),clf;% subplot(2,2,1);plot(-kInk);title('k in k'); subplot(2,2,2),plot(-avgInk);title('avg in k');
            % subplot(2,2,2),hist(-avgInk(:));title('avg in k');
            subplot(2,3,1);plot(-kInk_all);title('k in k');
            subplot(2,3,2),plot(-avgInk_all);title('avg in k');
            subplot(2,3,3),plot(-cInk_all);title('c in k');
            subplot(2,3,4);hist([-kInk_all(:) -avgInk_all(:) -cInk_all(:)]); %title(num2str(id(j))); 
            title(num2str(tempMat(end)));
            legend('all k in k','all avg in k','all c in k');% subplot(2,2,2),hist(-avgInk(:));title('avg in k');
            %             subplot(2,3,5);hist([-kInk(:) -avgInk(:) -cInk(:)]); legend('last k in k','last avg in k', 'last c in k');
            subplot(2,3,5);plot(disparityRng, [k_gtDistri_sum' avg_gtDistri_sum']);hold on;plot(exp_old, prob_old,'ob');plot(exp_new, prob_new,'or');grid on;legend('k','avg');title(sprintf('old exp: %0.3f\nnew exp: %0.3f', exp_old, exp_new));
            subplot(2,3,6);histogram([-cInk_all(:)]);hold on;histogram([-avgInk(:)]);legend('all','avg');
            drawnow;
        end
        
        
        % % %         [avg_gt_distri, avg_gtDistri_sum] = CalcDispErrDistribution(avgInk', disparityRng);
        % % %         [k_gt_distri, k_gtDistri_sum] = CalcDispErrDistribution(kInk', disparityRng);
        % % %         avg_gtDistri_sum_mat = [avg_gtDistri_sum_mat; avg_gtDistri_sum];
        % % %         k_gtDistri_sum_mat = [k_gtDistri_sum_mat; k_gtDistri_sum];
        if 0 % draw
            figure(2);clf;plot(disparityRng, [k_gtDistri_sum' avg_gtDistri_sum']);grid on;legend('k','avg');drawnow;
            figure,plot(avgInk_all(end,:) - mean(cInk_all))
        end
        % %        cInk_all(abs(cInk_all) > dispErrMargin) =  0; dispErrMargin;0;
        if size(cInk_all, 1) > 1
            dispVar = [dispVar; [{repmat(tempErr{1},1,size(cInk_all,2))} {tempErr{1,6}} {var(cInk_all)}  {kInk_all(1, :)}]];
        else
            dispVar = [dispVar; [{repmat(tempErr{1},1,size(cInk_all,2))} {tempErr{1,6}} {single(zeros(1,length(cInk_all)))} {kInk_all(1, :)}]];
        end
        c_in_k_mat = [c_in_k_mat {c_in_k}];
        c_in_k_mat_all = [c_in_k_mat_all {cInk_all(:)'}];
        cAvg_in_k_mat = [cAvg_in_k_mat {cAvg_in_k}];
        k_in_k_mat = [k_in_k_mat {k_in_k}];
        if 0
            figure, plot([cAvg_in_k - avgInk;c_in_k - cInk]')
        end
    end
    [aa] = unique(tempMat);
    for k = 1 : length(aa)
        idd = find(tempMat == aa(k));
        c_in_k_temp = cell2mat(tempCell(idd,1)');
        cAvg_in_k_temp = cell2mat(tempCell(idd,2)');
        k_in_k_temp = cell2mat(tempCell(idd,3)');
        [dispa1,dispb1] = hist(c_in_k_temp, 100); dispErrExp1 = -dot(dispb1, (dispa1./sum(dispa1)));
        [dispa2,dispb2] = hist(cAvg_in_k_temp, 100); dispErrExp2 = -dot(dispb2, (dispa2./sum(dispa2)));
        [dispa3,dispb3] = hist(k_in_k_temp, 100); dispErrExp3 = -dot(dispb3, (dispa3./sum(dispa3)));
        dispErrDistriMat(aa(k),:) = [frmId(idd)+1 dispErrExp1 dispErrExp2 dispErrExp3];
        %         dispErrDistriMat(aa(k),:) = [frmId(idd)+0 dispErrExp1 dispErrExp2 dispErrExp3];
    end
end


[~, idExp] = sort(dispErrExpMat(:,1),'ascend');
dispErrExpMat1 = dispErrExpMat(idExp,:);

dispErrDistriMat1 = dispErrDistriMat(dispErrDistriMat(:,1) > 0 ,:);


if forceReturn
    return;
end



dispErrMatDetailed = [];
for ko = 1 : size(dispErrExpStack,1)
    tempDisps = dispErrExpStack{ko, 3};
    frameRng = [dispErrExpStack{ko, 2} - dispErrExpStack{ko, 4} + 1 : dispErrExpStack{ko, 2} - dispErrExpStack{ko, 4} + 0 + size(tempDisps,1)];
    for j = 1 : length(frameRng)
        tempDisps_ = tempDisps{j,2};
        [a1,b1] = hist(tempDisps_, 100);
        dispErrA1 = -dot(b1, (a1./sum(a1)));
        dispErrMatDetailed(ko,frameRng(j)) = [dispErrA1];
    end
end

if 0
    dispErrMatDetailed(:,end) - dispErrDistriMat(:,3);
end

dispErrMatDetailed(dispErrMatDetailed == 0) = nan;

obj.angOptManager.dispErrMatDetailedUniq = dispErrMatDetailed;



dispAnalysis00 = [single(cell2mat(dispVar(:,1)')') cell2mat(dispVar(:,2)')' cell2mat(dispVar(:,3)')' cell2mat(dispVar(:,4)')'];
dispAnalysis0 = dispAnalysis00(abs(dispAnalysis00(:,4)) < dispErrMargin,:);

[~,id2] = sort(dispAnalysis0(:,2),'ascend');
dispAnalysis = dispAnalysis0(id2,:);


temp_c_in_k = cell2mat((c_in_k_mat))';
[y_c_in_k, x_c_in_k] = hist(-temp_c_in_k(abs(temp_c_in_k) < dispErrMargin), 50);

temp_c_in_k_all0 = cell2mat((c_in_k_mat_all))';
[y_c_in_k_all0, x_c_in_k_all] = hist(-temp_c_in_k_all0(abs(temp_c_in_k_all0) < dispErrMargin), 50);

if 0
    scale = length(cell2mat((c_in_k_mat_all)))/length(cell2mat((c_in_k_mat)));
else
    scale = sum((abs(temp_c_in_k_all0) < dispErrMargin))/sum((abs(temp_c_in_k) < dispErrMargin));
end
scale = 1;
y_c_in_k_all = y_c_in_k_all0./scale;


temp_cAvg_in_k = cell2mat((cAvg_in_k_mat))';
[y_cAvg_in_k, x_cAvg_in_k] = hist(-temp_cAvg_in_k(abs(temp_cAvg_in_k) < dispErrMargin), 50);

temp_k_in_k = cell2mat((k_in_k_mat))';
[y_k_in_k, x_k_in_k] = hist(-temp_k_in_k(abs(temp_k_in_k) < dispErrMargin), 50);


% % % % % % % % % % % % % % % X = meshgrid(sort(dispAnalysis(:,1)));
% % % % % % % % % % % % % % % Y = sort(dispAnalysis(:,2));
% % % % % % % % % % % % % % % Z = Y.*sin(X) - X.*cos(Y);



figure(figNum),
% subplot(2,2,[2 4]);
% subplot(2,2,2);


subplot(2,5,[3 ]); cla;
hold on;plot(x_k_in_k, y_k_in_k./sum(y_k_in_k),'-b');plot(x_c_in_k_all, y_c_in_k_all./sum(y_c_in_k_all),'-m');plot(x_c_in_k, y_c_in_k./sum(y_c_in_k),'-r');plot(x_cAvg_in_k, y_cAvg_in_k./sum(y_cAvg_in_k),'-g');legend('k in k','c in k all', 'c in k','cAvg in k');grid on;
try
    subplot(2,5,[4 ]); cla;plot(disparityRng, [sum(k_gtDistri_sum_mat)' sum(avg_gtDistri_sum_mat)']);grid on;legend('k','avg');
catch
    asgdkj = 1;
end
subplot(2,5,[5]); cla;plot(dispAnalysis(:,2), dispAnalysis(:,3));title('disp variation');
end
function [depthGTInd11, depthGTIndAll] = RoundingDispErr2(DispRngCenter,disp2round, DispRngStep,DispRng)
disparityError = DispRngCenter - disp2round;

disparityErrorRound = round(disparityError./DispRngStep).*DispRngStep;


depthGTOfst11 = round(-(disparityErrorRound)./DispRngStep);
depthGTInd11 = depthGTOfst11 + (size(DispRng,2)+1)/2;
depthGTInd11(depthGTInd11 < 1) = 1;
depthGTInd11(depthGTInd11 > size(DispRng,2)) = size(DispRng,2);

depthGTIndAll = sub2ind(size(DispRng),[1:size(DispRng,1)]', depthGTInd11);


end

function [ProbZTmp_update_norm_sum_tmp_c2c, ProbZTmp_update_norm_sum_tmp_c2c_sum] = CalcDispErrDistribution(dispCurList_gt, disparityRng)
DispRngC2C = zeros(length(dispCurList_gt),1) + disparityRng;

DispRngStepC2C = mean(diff(DispRngC2C'))';
[~, depthGTIndAll_cur] = RoundingDispErr2(DispRngC2C(:,(size(DispRngC2C,2)+1)/2),-dispCurList_gt, DispRngStepC2C,DispRngC2C);
try
    depthGTIndAll_cur(isnan(depthGTIndAll_cur)) = 1;
catch
    asdfgk = 1;
end
ProbZTmp_update_norm_sum_tmp_c2c = zeros(size(DispRngC2C));
%                     ProbZTmp_update_norm_sum_tmp(depthGTInd11) = ProbZTmp_update_norm_sum(depthGTInd11);
ProbZTmp_update_norm_sum_tmp_c2c(depthGTIndAll_cur) = 1;
ProbZTmp_update_norm_sum_tmp_c2c_sum = sum(ProbZTmp_update_norm_sum_tmp_c2c);
end