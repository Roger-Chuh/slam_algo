function testProb
close all;
e = 0;
sigma = 0.5;

x1 = [-5:0.001:5] + e;
x2 = 1./x1;
x3 = [-3:0.0021:3];
% x1 = 1./x3;
expectation1 = 1/(sqrt(2*pi)*sigma)* exp(-(x1-e).^2/2/sigma/sigma);

expectation2 = 1/(sqrt(2*pi)*sigma)* exp(-(1./x2 - e).^2./2./sigma/sigma)./x2./x2;
expectation3 = 1/(sqrt(2*pi)*sigma)* exp(-(1./x3 - e).^2./2./sigma/sigma)./x3./x3;


dlt = mean(diff(x1));
x11 = [x1(1)-dlt x1 x1(end)+dlt];
        xVec1 = 1./x11;
        xVec1 = sort(xVec1);
        xVec1_0 = xVec1(2:end-1);
        dlt2 = diff(xVec1)./2;
        xVec11 = xVec1(1:end-1) + [dlt2];
    interval = abs(diff(xVec11));
    figure,plot(xVec1,ones(1,length(xVec1)),'-x');hold on;plot(xVec11, 1.*ones(1,length(xVec11)),'o');
    expectation = 1/(sqrt(2*pi)*sigma)* exp(-(1./xVec11 - e).^2./2./sigma/sigma)./xVec11./xVec11;
figure,plot(xVec11, expectation)
        expectationMat = [expectation(1:end-1)' expectation(2:end)'];
        
                        prob = (mean(expectationMat')'.*interval')';
                      figure,plot(xVec1_0, prob);  
                            id = ~isnan(prob) & ~isinf(prob);
                           si =  sum(prob(id));
                           
                           ekj = 1;
% [expectation1_1, xVec1_1] = probDensity(e, sigma, x1, 'theta');
% [expectation2_1, xVec2_1] = probDensity(e, sigma, x1, 'disparity');

% figure,plot([expectation1_1 - expectation1; expectation2_1 - expectation2]');
% figure,plot(xVec2_1 - x2);

figure,plot(x1, expectation1);hold on,plot(x3, expectation3);%hold on,plot(x3, expectation3);
end