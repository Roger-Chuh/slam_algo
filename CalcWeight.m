function weightMatNorm = CalcWeight(dispErrMatDetailed_before_Diff, flipWeight, shortestWeightMax, longestWeightMax, varargin)

flipWeight_bak = flipWeight;

if (nargin <= 4)
    maxWeightId = [];
elseif (nargin == 5)
    maxWeightId = varargin{1};
else
    error('Too many input arguments');
end

if ~isempty(maxWeightId)
    flipWeight = false;
end


weightMat = [];
for tr = 1 : size(dispErrMatDetailed_before_Diff,2)
    tempWeightMat = dispErrMatDetailed_before_Diff(:,1:tr) > 0;
    if 1
        tempWeightMatSum = sum(tempWeightMat')';
    else
        tempWeightMatSum = tempWeightMat(:,end) > 0;
    end
    tempWeightMatSum(tempWeightMat(:,end) == 0) = 0;
    
    if flipWeight
        idWeight = find(tempWeightMatSum > 0);
        tempWeightMatSum_ = tempWeightMatSum;
        tempWeightMatSum_(idWeight) = tempWeightMatSum(flipud(idWeight));
        
        if isempty(maxWeightId)
            if shortestWeightMax
                idShortest = find(tempWeightMatSum_ > 0);
                if length(idShortest) > 1
                    tempWeightMatSum_(idShortest(1):idShortest(end-1)) = 0;
                end
                
                
            end
        else
            
            asdfgu = 1;
        end
        
        
    else
        
        tempWeightMatSum_ = tempWeightMatSum;
        
        if isempty(maxWeightId)
            if longestWeightMax
                idLongest = find(tempWeightMatSum_ > 0);
                if length(idLongest) > 1
                    tempWeightMatSum_(idLongest(2):idLongest(end)) = 0;
                end
                
                
            end
        else
            
            if 1
                if tr < maxWeightId
                    id2 = find(tempWeightMatSum_ > 0);
                    
                    if length(id2) > 1
                        tempWeightMatSum_(id2(2):id2(end)) = 0;
                    end
                else
                    idChosen = []; dlt = 0; breakFlag = false;
                    while isempty(idChosen)
                        idChosen = find(tempWeightMatSum_ == maxWeightId + dlt);
                        dlt = dlt + 1;
                        if dlt > 100
                            breakFlag = true;
                            break;
                        end
                    end
                    if breakFlag
                        
                        idChosen = []; dlt = 0; % breakFlag = false;
                        while isempty(idChosen)
                            idChosen = find(tempWeightMatSum_ == maxWeightId + dlt);
                            dlt = dlt - 1;       
                            if 0 % dlt > 100
                                breakFlag = true;
                                break;
                            end
                        end
                        
                        
                    end
                    tempWeightMatSum_ = zeros(size(tempWeightMatSum_));
                    tempWeightMatSum_(idChosen) = 1;
                    %                 idChosen = find(tempWeightMatSum_ == maxWeightId);
                end
                aghkj = 1;
            else
                if 0
                    if length(idChosen) > 1
                        tempWeightMatSum_(idLongest(2):idLongest(end)) = 0;
                    end
                else
                    
                    if ~isempty(idChosen)
                        tempWeightMatSum_ = zeros(size(tempWeightMatSum_));
                        tempWeightMatSum_(idChosen) = 1;
                        
                    else
                        id2 = find(tempWeightMatSum_ > 0);
                        
                        if length(id2) > 1
                            tempWeightMatSum_(id2(2):id2(end)) = 0;
                        end
                    end
                    
                end
            end
            asfgh = 1;
        end
    end
    
    if tr == 1
        weightMat = [tempWeightMat ];
    else
        weightMat = [weightMat  tempWeightMatSum_];
    end
    
end
weightMatNorm = (weightMat./repmat(sum(weightMat),size(weightMat,1),1));

weightMatNorm(isnan(weightMatNorm)) = 0;
end