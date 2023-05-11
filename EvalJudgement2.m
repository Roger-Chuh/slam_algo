function EvalJudgement2(obj, Bin, zThr)

intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,obj.scaleLvl);
[princpPtL, princpPtR] = Get(obj.camModel, 'PinholePrincpPt', obj.scaleLvl);
baseline = obj.camModel.transVec1To2;

useOld = true; false;


judgement = obj.judgement;

% Bin = 50;
% zThr = [1000 5000]; 
traceThr = 10;

zMat = {}; zMatGT = {}; sumMat = {}; sumMatGT = {};
dispErr  ={};

for i = 1 : size(judgement,1)
    
    if size(judgement{i,2},1) >= traceThr
        temp0 = judgement{i,2}(1,:);
        temp = judgement{i,2}(end,:);
        zStereo = temp0{1}(3,:);
        zGT = temp0{4}(3,:); 
        zGT0 = temp0{6}(1,:); 
        dispStereo = (intrMat(1,1).*norm(obj.camModel.transVec1To2))./(zStereo + (princpPtR(1) - princpPtL(1)));
        dispGT = (intrMat(1,1).*norm(obj.camModel.transVec1To2))./(zGT + (princpPtR(1) - princpPtL(1)));
        
        dispErr = [dispErr {dispStereo - dispGT}];
        zMat = [zMat {temp0{1}(3,:)}];
        zMatGT = [zMatGT {temp{end-2}}];
        sumMat = [sumMat {sum(temp{end})}];
        sumMatGT = [sumMatGT {sum(temp{end-1})}];
        if 0
            figure,plot(temp0{4}(3,:) - temp{end-2});
            figure,plot(zGT - zGT0)
        end
    end
    
end
sumErr = cell2mat(sumMat);
if 0
    id = find(cell2mat(zMatGT) > (zThr(1))  & cell2mat(zMatGT) < (zThr(2)));
else
    id = find(cell2mat(zMat) > (zThr(1))  & cell2mat(zMat) < (zThr(2)));
end
sumErr0 = sumErr;

sumErr = sumErr(id);
gtZErr = cell2mat(zMatGT) - cell2mat(zMat);
dispErrCell = cell2mat(dispErr);
dispErrCell00 = dispErrCell;
gtZErr00 = gtZErr;
gtZErr = gtZErr(id);
dispErrCell = dispErrCell(id);
gtZErr0 = gtZErr;
dispErrCell0 = dispErrCell;
    
% [a,b] = hist(cell2mat(sumMat),Bin);
if 1
    if useOld
        [a, b] = hist(sumErr,Bin);
    else
         % [aa, bb] = CalcHist(sumErr, gtZErr, Bin);
    end
    [aa, bb, dispErrHist, dispErrMat] = CalcHist(sumErr, gtZErr0,dispErrCell, Bin);
end
dsfgjk = 1;

if 0
    [aGT,bGT] = hist(cell2mat(sumMatGT),Bin);
else
    zMatCell = cell2mat(zMat);
    zMatCell0 = zMatCell;
    zMatCell = zMatCell(id);
% % %     gtZErr = cell2mat(zMatGT) - cell2mat(zMat);
% % %     gtZErr00 = gtZErr;
    
% % %     gtZErr = gtZErr(id);
% % %     gtZErr0 = gtZErr;
    gtZErr(gtZErr0 < 0) = -1000;
    gtZErr(gtZErr0 > 0) = 1000;
    if useOld
        [aGT,bGT] = hist(gtZErr,Bin);
        [aGT1,bGT1] = hist(gtZErr(gtZErr0 < 0),Bin);
        [aGT2,bGT2] = hist(gtZErr(gtZErr0 > 0),Bin);
        [a1,b1] = hist(sumErr(gtZErr0 < 0),Bin);
        [a2,b2] = hist(sumErr(gtZErr0 > 0),Bin);
    else
        [aGT,bGT] = CalcHist(gtZErr,Bin);
        [aGT1,bGT1] = CalcHist(gtZErr(gtZErr0 < 0),Bin);
        [aGT2,bGT2] = CalcHist(gtZErr(gtZErr0 > 0),Bin);
        [a1,b1] = CalcHist(sumErr(gtZErr0 < 0),Bin);
        [a2,b2] = CalcHist(sumErr(gtZErr0 > 0),Bin);
    end
    
    
    
    
    
    
end
if 0
    figure,subplot(1,3,1);plot(cell2mat(zMat));title('z');subplot(1,3,2),hist(cell2mat(sumMat));title('stereo');subplot(1,3,3),hist(cell2mat(sumMatGT));title('gt');
elseif 0
    figure,subplot(1,3,1);plot(cell2mat(zMat));title('z');subplot(1,3,2),plot(b,abs(a.*b));title('stereo');subplot(1,3,3),plot(bGT,abs(aGT.*bGT));title('gt');
else
    if useOld
        figure,subplot(2,3,1);plot(zMatCell);title(sprintf('z range: [%d : %d]', zThr(1), zThr(2)));
        subplot(2,3,2),hold on;plot(bGT,abs(aGT.*bGT),'-r');plot(b,abs(a.*b),'-b');legend('gt','stereo'); % subplot(1,3,3),plot(bGT,abs(aGT.*bGT));title('gt');
        subplot(2,3,5);hold on;plot(bGT1,abs(aGT1.*bGT1),'-r');plot(b1,abs(a1.*b1),'-b');title('zGT < zStereo');legend('gt','stereo');
        subplot(2,3,6);hold on;plot(bGT2,abs(aGT2.*bGT2),'-r');plot(b2,abs(a2.*b2),'-b');title('zGT > zStereo');legend('gt','stereo');
        subplot(2,3,3),plot(aa,bb);grid on;hold on;plot(aa, zeros(length(aa),1));
        subplot(2,3,4),hist(dispErrMat);title('k(stereo - gt)')
        %     figure,subplot(1,2,1);plot(cell2mat(zMatGT));title('z');subplot(1,2,2),hold on;plot(b,abs(a.*b),'-r');plot(b,abs(a.*b),'-b');legend('stereo(c - k)'); % subplot(1,3,3),plot(bGT,abs(aGT.*bGT));title('gt');
    else
        figure,subplot(2,3,1);plot(zMatCell);title(sprintf('z range: [%d : %d]', zThr(1), zThr(2)));
        subplot(2,3,5),hold on;plot(aGT,(bGT),'-r');plot(a,(b),'-b');legend('gt','stereo'); % subplot(1,3,3),plot(bGT,abs(aGT.*bGT));title('gt');
        subplot(2,3,2);hold on;plot(aGT1,(bGT1),'-r');plot(a1,(b1),'-b');title('zGT < zStereo');legend('gt','stereo');
        subplot(2,3,3);hold on;plot(aGT2,(bGT2),'-r');plot(a2,(b2),'-b');title('zGT > zStereo');legend('gt','stereo');
        subplot(2,3,6),plot(aa,bb);
        %     figure,subplot(1,2,1);plot(cell2mat(zMatGT));title('z');subplot(1,2,2),hold on;plot(b,abs(a.*b),'-r');plot(b,abs(a.*b),'-b');legend('stereo(c - k)'); % subplot(1,3,3),plot(bGT,abs(aGT.*bGT));title('gt');
    end
end

end

function [binValue, ratio, dispErrCell, dispErrMat] = CalcHist(data, gtZErr,dispErr, bin)

zThr = 20; % mm

figure(19191),clf;a = histogram(data,bin);
binEdge = a.BinEdges;
binCount = a.Values;
binValue0 = binEdge(1:end-1) + diff(binEdge)/2;
Id = [];ratio = []; binValue = [];dispErrCell = {}; featSize = [];
for i = 1 : length(binCount)
    if i < length(binCount)
        id = find(data >= binEdge(i) & data < binEdge(i + 1)) ;
    else
        id = find(data >= binEdge(i) & data <= binEdge(i + 1)) ;
    end
    if 0
        tempSum = data(id);
    else
        tempSum = gtZErr(id);
        tempDispErr = dispErr(id);
    end
% % %     if length(tempSum) < 10
% % %         id = [];
% % %     end
%     if ~isempty(id)
    if length(id) > 10
        if binValue0(i) > 0
            ratio_ = 1.*(sum(tempSum > -abs(zThr))/length(tempSum) - 0.0);
        else
            ratio_ = 1.*(-sum(tempSum < abs(zThr))/length(tempSum) + 0.0);
        end
        ratio = [ratio ratio_];
        binValue = [binValue binValue0(i)];
        dispErrCell = [dispErrCell {tempDispErr}];
        featSize = [featSize length(tempDispErr)];
        Id = [Id id];
    else
        
    end
    
end
featSize0 = max(featSize);

dispErrMat = nan(featSize0, length(dispErrCell));

for j = 1 : size(dispErrMat,2)
    dispErrMat(1:length(dispErrCell{j}),j) = dispErrCell{j};
    
end

if 0
    figure,plot(binValue, ratio);
    figure,plot(binValue, ratio);
end
end
