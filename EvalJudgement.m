function EvalJudgement(obj, Bin, zThr)

intrMat = Get(obj.camModel, 'PinholeIntrMat', 1,obj.scaleLvl);
[princpPtL, princpPtR] = Get(obj.camModel, 'PinholePrincpPt', obj.scaleLvl);
baseline = obj.camModel.transVec1To2;


judgement = obj.judgement;

% Bin = 50;
% zThr = [1000 5000]; 
traceThr = 10;

zMat = {}; zMatGT = {}; sumMat = {}; sumMatGT = {};

for i = 1 : size(judgement,1)
    
    if size(judgement{i,2},1) >= traceThr
        temp0 = judgement{i,2}(1,:);
        temp = judgement{i,2}(end,:);
        zMat = [zMat {temp0{1}(3,:)}];
        zMatGT = [zMatGT {temp{end-2}}];
        sumMat = [sumMat {sum(temp{end})}];
        sumMatGT = [sumMatGT {sum(temp{end-1})}];
        if 0
            figure,plot(temp0{4}(3,:) - temp{end-2});
        end
    end
    
end
sumErr = cell2mat(sumMat);

id = find(cell2mat(zMatGT) > (zThr(1))  & cell2mat(zMatGT) < (zThr(2)));

sumErr0 = sumErr;

sumErr = sumErr(id);
% [a,b] = hist(cell2mat(sumMat),Bin);
[a,b] = hist(sumErr,Bin);
[binValue, ratio] = CalcHist(sumErr, Bin);

dsfgjk = 1;

if 0
    [aGT,bGT] = hist(cell2mat(sumMatGT),Bin);
else
    zMatCell = cell2mat(zMat);
    zMatCell0 = zMatCell;
    zMatCell = zMatCell(id);
    gtZErr = cell2mat(zMatGT) - cell2mat(zMat);
    gtZErr00 = gtZErr;
    
    gtZErr = gtZErr(id);
    gtZErr0 = gtZErr;
    gtZErr(gtZErr0 < 0) = -1000;
    gtZErr(gtZErr0 > 0) = 1000;
    
    
    [aGT,bGT] = hist(gtZErr,Bin);
    
    [aGT1,bGT1] = hist(gtZErr(gtZErr0 < 0),Bin);
    [aGT2,bGT2] = hist(gtZErr(gtZErr0 > 0),Bin);
    [a1,b1] = hist(sumErr(gtZErr0 < 0),Bin);
    [a2,b2] = hist(sumErr(gtZErr0 > 0),Bin);
    
    
end
if 0
    figure,subplot(1,3,1);plot(cell2mat(zMat));title('z');subplot(1,3,2),hist(cell2mat(sumMat));title('stereo');subplot(1,3,3),hist(cell2mat(sumMatGT));title('gt');
elseif 0
    figure,subplot(1,3,1);plot(cell2mat(zMat));title('z');subplot(1,3,2),plot(b,abs(a.*b));title('stereo');subplot(1,3,3),plot(bGT,abs(aGT.*bGT));title('gt');
else
    figure,subplot(2,2,1);plot(zMatCell);title(sprintf('z range: [%d : %d]', zThr(1), zThr(2)));
    subplot(2,2,2),hold on;plot(bGT,abs(aGT.*bGT),'-r');plot(b,abs(a.*b),'-b');legend('gt','stereo'); % subplot(1,3,3),plot(bGT,abs(aGT.*bGT));title('gt');
    subplot(2,2,3);hold on;plot(bGT1,abs(aGT1.*bGT1),'-r');plot(b1,abs(a1.*b1),'-b');title('zGT < zStereo');legend('gt','stereo');
    subplot(2,2,4);hold on;plot(bGT2,abs(aGT2.*bGT2),'-r');plot(b2,abs(a2.*b2),'-b');title('zGT > zStereo');legend('gt','stereo');
%     figure,subplot(1,2,1);plot(cell2mat(zMatGT));title('z');subplot(1,2,2),hold on;plot(b,abs(a.*b),'-r');plot(b,abs(a.*b),'-b');legend('stereo(c - k)'); % subplot(1,3,3),plot(bGT,abs(aGT.*bGT));title('gt');
end

end

function [binValue, ratio] = CalcHist(data, bin)

figure(19191),clf;a = histogram(data,bin);
binEdge = a.BinEdges;
binCount = a.Values;
binValue0 = binEdge(1:end-1) + diff(binEdge)/2;
Id = [];ratio = []; binValue = [];
for i = 1 : length(binCount)
    if i < length(binCount)
        id = find(data >= binEdge(i) & data < binEdge(i + 1)) ;
    else
        id = find(data >= binEdge(i) & data <= binEdge(i + 1)) ;
    end
    tempSum = data(id);
    
    if ~isempty(tempSum)
        if binValue0(i) > 0
            ratio_ = sum(tempSum > 0)/length(tempSum);
        else
            ratio_ = sum(tempSum < 0)/length(tempSum);
        end
        ratio = [ratio ratio_];
        binValue = [binValue binValue0(i)];
    else
        
    end
%     ratio = [ratio ratio_];
    Id = [Id id];
    
end

if 0
    figure,plot(binValue, ratio);
end
end
