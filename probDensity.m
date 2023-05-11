function [prob, xVec1_0, expectation] = probDensity(x0, sigma, nDisp,nProj, xVec,dlt, type)
% input: disparity or theta

oldProb = false; true; false;
% nDisp = 4;10; 4; 4;4;50;
% nProj = 4;10; 10; 4;2; 2; 50; 15;8;6;3;4;

xVec1_0 = 0;
if strcmp(type, 'disparity')
    x01 = x0;
    %     xVec0 = xVec;
            dlt = mean(diff(xVec));
    if 1
        xVec00 = xVec;
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
    %     expectation = 1/(sqrt(2*pi)*sigma)* exp(-(1./xVec11 - x01).^n./2.*sigma^(-n))./xVec11./xVec11;
    if 0
        expectation = 1/(sqrt(2*pi)*sigma)* exp(-(abs(1./xVec11 - x01)).^nDisp./2.*sigma^(-nDisp))./xVec11./xVec11;
    else
%         expectation = 1/(sqrt(2*pi)*sigma)* exp(-(abs(1./xVec11 - x01)).^nDisp./2.*sigma^(-nDisp));
        expectation = 1/(sqrt(2*pi)*sigma)* exp(-(abs(xVec00 - x01)).^nDisp./2.*sigma^(-nDisp));
        prob = expectation;
        return;
    end
    %     figure,plot(xVec11, expectation);
    if 1
        expectationMat = [expectation(1:end-1)' expectation(2:end)'];
        if 1 % oldProb
                        prob = (mean(expectationMat')'.*interval')';
%             prob = (mean(expectationMat')'.*dlt')';
        else
                        prob = ((expectationMat(:,1)')'.*interval')';
%             prob = ((expectationMat(:,1)')'.*dlt')';
        end
    else
        prob = expectation'.*interval';
    end
    id = ~isnan(prob) & ~isinf(prob);
    si = sum(prob(id));
    sii = trapz(xVec11, expectation);
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
        expectation = 1/(sqrt(2*pi)*sigma)* exp(-(abs(xVec11-x0)).^n./2.*sigma.^(-n));
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
        expectation = 1/(sqrt(2*pi)*sigma)* exp(-(abs(xVec11-x0)).^nProj./2.*sigma.^(-nProj));
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
        expectation = 1/(sqrt(2*pi)*sigma)* exp(-(abs(xVec11-x0)).^n./2.*sigma.^(-n));
        expectationMat = [expectation(1:end-1)' expectation(2:end)'];
    else
        expectation1 = 1/(sqrt(2*pi)*sigma)* exp(-(abs(xVecTmp1-x0)).^nProj./2.*sigma.^(-nProj));
        expectation2 = 1/(sqrt(2*pi)*sigma)* exp(-(abs(xVecTmp2-x0)).^nProj./2.*sigma.^(-nProj));
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


% % prob = prob./max(prob);
end