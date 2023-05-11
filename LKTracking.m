function [pix2_0, inTraceFlag] = LKTracking(img1, img2, pix1, inTraceFlag, marg)


global resolution saturation ...
    wintx winty boundary boundary_t ...
    thresh levelmin levelmax ThreshQ method;


if isempty(inTraceFlag)
    inTraceFlag = true(size(pix1,1),1);
end

img1_0 = img1;
img2_0 = img2;
if size(img1,3) > 1
    img1_gray = rgb2gray(img1_0);
    img2_gray = rgb2gray(img2_0);
else
    img1_gray = (img1_0);
    img2_gray = (img2_0);
    
end
inTraceFlag0 = inTraceFlag;


if size(img1,3) > 1
    img1 = mean(double(img1),3);
    img2 = mean(double(img2),3);
else
    img1 = (double(img1));
    img2 = (double(img2));
    
end

if 0
    img1 = img1./max(img1(:));
    img2 = img2./max(img2(:));
end

pix1_0 = pix1;
pix1 = pix1(:,[2 1]);
[xtt,inTraceFlag,Qtt] = track_fcn(img1,img2,pix1',inTraceFlag);
pix2_0 = xtt([2 1],:)';
inBndFlag = pix2_0(:, 1) >= marg + 1 & ...
    pix2_0(:, 1) <= Cols(img1) - marg & ...
    pix2_0(:, 2) >= marg + 1 & ...
    pix2_0(:, 2) <= Rows(img1) - marg;
inTraceFlag = inTraceFlag & inBndFlag;

if 0
    figure,subplot(1,2,1);imshow(img1, []);hold on;plot(pix1_0(inTraceFlag,1), pix1_0(inTraceFlag,2),'.r');
    subplot(1,2,2);imshow(img2, []);hold on;plot(pix2_0(inTraceFlag,1), pix2_0(inTraceFlag,2),'.r');
    
    figure,showMatchedFeatures(img1, img2,pix1_0(inTraceFlag,:), pix2_0(inTraceFlag,:));
end

end