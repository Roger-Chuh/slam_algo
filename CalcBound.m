function [optAng, upperAng, lowerAng, tempP2CCum] = CalcBound(obj, angMatUseNewBak, angMatUseNewUpperBak, angMatUseNewLowerBak, traceLen)
 
StartMat = angMatUseNewBak == 100;
StartMatId = find(sum(StartMat));

splitMat = zeros(size(angMatUseNewBak));
splitMat(1,1) = 1;
Id = 1;
lineNum = 1;

lengthIdThr = 1000;


while lineNum < size(angMatUseNewBak,1)
    id11 = find(abs(angMatUseNewBak(lineNum,:)) > 0);
    id1 = id11(1:min(length(id11), traceLen));
    
    ct = 0;
    while 1
        if ismember(id1(end - ct),StartMatId)
            id11 = id1(end - ct);
            break;
        else
            ct = ct + 1;
        end
    end
    
    
    
    Id = [Id; id11];
    
    if length(Id) > lengthIdThr
        optAng = 1; upperAng = 1; lowerAng = 1;  tempP2CCum = 1;
        return;
    end
    
    id2 = find(abs(angMatUseNewBak(:,id11)) > 0);
    splitMat(id2(end), id11) = 1;
    lineNum = id2(end);
end
[y, x] = ind2sub(size(splitMat), find(splitMat > 0));

optAng = [];
upperAng = [];
lowerAng = [];
tempP2C = 0;
for i = 1 : length(x)
    
    if i ~= length(x)
        tempMatOpt = angMatUseNewBak(y(i):y(i+1)-1, x(i):x(i+1));
        tempMatOptUpper = angMatUseNewUpperBak(y(i):y(i+1)-1, x(i):x(i+1));
        tempMatOptLower = angMatUseNewLowerBak(y(i):y(i+1)-1, x(i):x(i+1));
        [tempAngOptList, tempAngOptUpperList, tempAngOptLowerList] = ComposeAng(obj, tempMatOpt, tempMatOptUpper, tempMatOptLower, x(i):x(i+1));
        tempP2C = [tempP2C; diff([0 tempMatOpt(1,2:end)])'];
        
        
        if i == 1
            optAng = [optAng; tempAngOptList];
            upperAng = [upperAng; tempAngOptUpperList];
            lowerAng = [lowerAng; tempAngOptLowerList];
        else
            
            upperAng = [upperAng; optAng(end) + tempAngOptUpperList(2:end)];
            lowerAng = [lowerAng; optAng(end) + tempAngOptLowerList(2:end)];
            
            optAng = [optAng; optAng(end) + tempAngOptList(2:end)];
        end
        
        
        sekjjb = 1;
    else
        tempMatOpt = angMatUseNewBak(y(i):end, x(i):end);
        tempMatOptUpper = angMatUseNewUpperBak(y(i):end, x(i):end);
        tempMatOptLower = angMatUseNewLowerBak(y(i):end, x(i):end);
        [tempAngOptList, tempAngOptUpperList, tempAngOptLowerList] = ComposeAng(obj, tempMatOpt, tempMatOptUpper, tempMatOptLower, x(i):size(splitMat,2));
        tempP2C = [tempP2C; diff([0 tempMatOpt(1,2:end)])'];
        
        
        upperAng = [upperAng; optAng(end) + tempAngOptUpperList(2:end)];
        lowerAng = [lowerAng; optAng(end) + tempAngOptLowerList(2:end)];
        
        optAng = [optAng; optAng(end) + tempAngOptList(2:end)];
        aszlkgar = 1;
        
    end
    
    
end

tempP2CCum = cumsum(tempP2C(1:end-1));

end



function [angOptList, angOptUpperList, angOptLowerList] = ComposeAng(obj, angMatUseNew, angMatUseNewUpper, angMatUseNewLower, angId)
angMatUseNew(angMatUseNew == 100) = 0;
angMatUseNewUpper(angMatUseNewUpper == 100) = 0;
angMatUseNewLower(angMatUseNewLower == 100) = 0;
if 0
    angArrangeErr = angMatUseNew - angOptListLast;
end


angMatUseNewDiff = diff(angMatUseNew')';
angMatUseNewDiff(angMatUseNewDiff < 0) = 0;

angMatUseNewUpperDiff = diff(angMatUseNewUpper')';
angMatUseNewUpperDiff(angMatUseNewUpperDiff < 0) = 0;

angMatUseNewLowerDiff = diff(angMatUseNewLower')';
angMatUseNewLowerDiff(angMatUseNewLowerDiff < 0) = 0;






validAngOpt = angMatUseNewDiff > 0;
validAngOptSum = sum(validAngOpt,1);

accumErrWeight = sum(validAngOpt')';
accumErrWeightMat = repmat(accumErrWeight,1,size(validAngOpt,2));
accumErrWeightMatValid = (accumErrWeightMat.*validAngOpt)';

if 0
    accumErrWeightMatValid = 1./accumErrWeightMatValid;
    accumErrWeightMatValid(isinf(accumErrWeightMatValid)) = 0;
elseif 0
    accumErrWeightMatValid(accumErrWeightMatValid == 0) = nan;
    accumErrWeightMatValid = max(accumErrWeightMatValid(:)) - accumErrWeightMatValid+1;
    accumErrWeightMatValid(isnan(accumErrWeightMatValid)) = 0;
else
    saknab = 1;
end

accumErrWeightMatValidNorm = (accumErrWeightMatValid./repmat(sum(accumErrWeightMatValid')',1,size(accumErrWeightMatValid,2)))';
if 1
    meanAngP2C = (sum(angMatUseNewDiff,1)./validAngOptSum)';
    meanAngP2CUpper = (sum(angMatUseNewUpperDiff,1)./validAngOptSum)';
    meanAngP2CLower = (sum(angMatUseNewLowerDiff,1)./validAngOptSum)';
else
    meanAngP2C = sum(accumErrWeightMatValidNorm.*angMatUseNewDiff)';
    meanAngP2CUpper = sum(accumErrWeightMatValidNorm.*angMatUseNewUpperDiff)';
    meanAngP2CLower = sum(accumErrWeightMatValidNorm.*angMatUseNewLowerDiff)';
end


angRef = obj.accumP2CRef(angId);
angRef = angRef - angRef(1);
angErr = rad2deg([0;cumsum(meanAngP2C)] - abs(angRef));
angErrUpper = rad2deg([0;cumsum(meanAngP2CUpper)] - abs(angRef));
angErrLower = rad2deg([0;cumsum(meanAngP2CLower)] - abs(angRef));

angOptList = [0;cumsum(meanAngP2C)];
angOptUpperList = [0;cumsum(meanAngP2CUpper)];
angOptLowerList = [0;cumsum(meanAngP2CLower)];
end

