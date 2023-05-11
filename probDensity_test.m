function [prob, xVec1_0, expectation] = probDensity_test(x0, sigma, xVec,dlt, type)
% input: disparity or theta

% [expectationZ, ZVec1] = probDensity_test(15, 20, [-100:0.00001:100],1, 'disparity');


oldProb = true; false; true; false;


xVec1_0 = 0;
if strcmp(type, 'disparity')
    x01 = x0; 
    %     xVec0 = xVec;
%             dlt = mean(diff(xVec));
    if 1
        dlt = mean(diff(xVec));
        xVec = [xVec(1)-dlt xVec xVec(end)+dlt];
        xVec1 = 1./xVec;
        xVec1_0 = xVec1(2:end-1);
        dlt2 = diff(xVec1)./2;
        xVec11 = xVec1(1:end-1) + [dlt2];
    else
        xVec = [xVec xVec(end)+dlt];
        xVec1 = 1./xVec;
        xVec1_0 = xVec1(1:end-1);
        dlt2 = diff(xVec1)./1;
        xVec11 = xVec1;
    end
    %     figure,plot(xVec1,ones(1,length(xVec1)),'-x');hold on;plot(xVec11, 1.*ones(1,length(xVec11)),'o');
    interval = abs(diff(xVec11)); 
    expectation = 1/(sqrt(2*pi)*sigma)* exp(-(1./xVec11 - x01).^2./2./sigma/sigma)./xVec11./xVec11;
    %     figure,plot(xVec11, expectation);
    if 1
        expectationMat = [expectation(1:end-1)' expectation(2:end)'];
        if oldProb
                        prob = (mean(expectationMat')'.*interval')';
%             prob = (mean(expectationMat')'.*dlt')';
        else
            %             prob = ((expectationMat(:,1)')'.*interval')';
            prob = ((expectationMat(:,1)')'.*dlt')';
        end
    else
        prob = expectation'.*interval';
    end
    id = ~isnan(prob) & ~isinf(prob);
    si = sum(prob(id));
    [~,idi] = max(prob); 
    %     figure,plot(xVec1_0, prob);
end

if strcmp(type, 'theta')
    %     dlt = mean(diff(xVec));
    if 0
        xVec = [xVec(1)-dlt xVec xVec(end)+dlt];
        xVec1 = xVec;
        xVec1_0 = xVec1(2:end-1);
        dlt2 = diff(xVec1)./2;
        xVec11 = xVec1(1:end-1) + [dlt2];
        %     figure,plot(xVec1,ones(1,length(xVec1)),'-x');hold on;plot(xVec11, 1.*ones(1,length(xVec11)),'o');
        interval = abs(diff(xVec11));
        expectation = 1/(sqrt(2*pi)*sigma)* exp(-(xVec11-x0).^2/2/sigma/sigma);
        %     figure,plot(xVec11, expectation);
        expectationMat = [expectation(1:end-1)' expectation(2:end)'];
        prob = mean(expectationMat')'.*interval';
        
    else
        xVec = [xVec xVec(end)+dlt];
        xVec1 = xVec;
        xVec1_0 = xVec1(1:end-1);
        dlt2 = diff(xVec1)./2;
        xVec11 = xVec1(1:end);
        %     figure,plot(xVec1,ones(1,length(xVec1)),'-x');hold on;plot(xVec11, 1.*ones(1,length(xVec11)),'o');
        interval = abs(diff(xVec11));
        expectation = 1/(sqrt(2*pi)*sigma)* exp(-(xVec11-x0).^2/2/sigma/sigma);
        %     figure,plot(xVec11, expectation);
        expectationMat = [expectation(1:end-1)' expectation(2:end)'];
        %         prob = expectation'.*interval';
        if oldProb
            prob = (mean(expectationMat')'.*interval')';
        else
            prob = ((expectationMat(:,1)')'.*interval')';
        end
        
        
    end
    %     figure,plot(xVec1_0, prob);
end



if strcmp(type, 'reproj')
    %     dlt = (diff(xVec));
    %     id = find(dlt < 0);
    %     xVec(id) = -xVec(id);
    %     dlt = (diff(xVec));
    if 0
        xVecTmp1 = xVec - dlt/2;
        xVecTmp2 = xVec + dlt/2;
    else
        xVecTmp1 = xVec - 0;
        xVecTmp2 = xVec + dlt/1;
    end
    xVec = [xVec(1)-dlt(1) xVec xVec(end)+dlt(end)];
    xVec1 = xVec;
    xVec1_0 = xVec1(2:end-1);
    dlt2 = diff(xVec1)./2;
    xVec11 = xVec1(1:end-1) + [dlt2];
    %     figure,plot(xVec1,ones(1,length(xVec1)),'-x');hold on;plot(xVec11, 1.*ones(1,length(xVec11)),'o');
    interval = abs(diff(xVec11));
    %     expectation = 1/(sqrt(2*pi)*sigma)* exp(-(xVec11-x0).^2/2/sigma/sigma);
    if 0
        expectation = 1/(sqrt(2*pi)*sigma)* exp(-(xVec11-x0).^2/2/sigma/sigma);
        expectationMat = [expectation(1:end-1)' expectation(2:end)'];
    else
        expectation1 = 1/(sqrt(2*pi)*sigma)* exp(-(xVecTmp1-x0).^2/2/sigma/sigma);
        expectation2 = 1/(sqrt(2*pi)*sigma)* exp(-(xVecTmp2-x0).^2/2/sigma/sigma);
        expectationMat = [expectation1' expectation2'];
    end
    %     figure,plot(xVec11, expectation);
    
    if oldProb
        prob = (mean(expectationMat')'.*dlt')';
    else
        prob = ((expectationMat(:,1)')'.*interval')';
        prob = ((expectationMat(:,1)')'.*dlt')';
    end
    %     figure,plot(xVec1_0, prob);
end



end