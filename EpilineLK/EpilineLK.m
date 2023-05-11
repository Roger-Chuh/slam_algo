function [pt2,goodfeat, FundMat00] = EpilineLK(img1, img2, intrMat,rt, pt1,goodfeat0)








global resolution winx winy saturation ...
    wintx winty spacing boundary boundary_t ...
    Nmax thresh levelmin levelmax ThreshQ ...
    N_max_feat method varThr deltVThr;

% TUNING PARAMETERS: parameters in this section of the code are free
% for the designer to choose. Choices are interconnected, and each choice affects
% the performance of the tracker.
if 0
    resolution = 0.003; 			% Desired tracking accuracy in pixels
    winx = 1; winy = 1;			% Window half-size for selection; effective size = (1+2*winx, 1+2*winy)
    % THIS IS A CRUCIAL DESIGN PARAMETER
    saturation = 7000; 			% Image saturation level (not necessary if variable 'method' chosen to be 0
    wintx = 5; winty = 5; 			% Window half-size for tracking; effective size = (1+2*wintx, 1+2*winty)
    spacing = 2;				% min spacing between 2 features (in pixel).
    boundary = 1;				% discards features selected too close to the boundary of the image
    boundary_t = 1;				% rejects tracked features too close to boundary
    Nmax = 5000;                            % maximum number of features selected
    thresh = 0.00000001;  0.01; 				% Threshold for feature selection
    % THIS IS A CRUCIAL DESIGN PARAMETER: low threshold = many features selected,
    % greater errors; high threshold = fewer features selected, better quality.
    levelmin = 0; 				% lower level in the pyramid
    levelmax = 0;				% higher level in the pyramid: large motions require more levels.
    % Inter-frame motions within 1 pixel do not require a pyramid (levelmax=0).
    ThreshQ = 0.01;				% Threshold for rejecting a feature
    % THIS IS A CRUCIAL DESIGN PARAMETER: low threshold = many features kept
    % through the track
    N_max_feat = Nmax;			% Minimum space reserved for feature storage
    method = 1;
end



varThr = 0.9;
deltVThr = wintx;
% pt1 = detectEdge(img1,(img1(:,:,1)),[0.1 0.3]);
% pt1 = pt1(pt1(:,1) <= size(img1,2)-(2+winty+boundary) & pt1(:,1) >= 2+winty+boundary & pt1(:,2) <= size(img1,1)-(2+winty+boundary) & pt1(:,2) >= 2+winty+boundary,:);

img11 = img1;
if 0
    MotionBlur = SobelEdge(img1);
    %     H = fspecial('sobel');
    %     MotionBlur = imfilter(uint8(Ipi),H,'replicate');
    % %     [gx,gy] = gradient((Ipi));
    %     ab = LocalMax(immultiply(abs(MotionBlur),abs(MotionBlur)>35));
    %     corner1 = Img2Pix(ab,ab);
    corner1 = Img2Pix(immultiply(abs(MotionBlur),abs(MotionBlur)>35),immultiply(abs(MotionBlur),abs(MotionBlur)>35));
    idx = randperm(size(corner1,1));
    corner1 = corner1(idx(1:Nmax),:);
    pt1 = corner1(corner1(:,1) <= size(img1,2)-(2+winty+boundary) & corner1(:,1) >= 2+winty+boundary & corner1(:,2) <= size(img1,1)-(2+winty+boundary) & corner1(:,2) >= 2+winty+boundary,:);
    %     idx = randperm(size(corner1,1));
    %     corner1 = corner1(idx(1:2000),:);
    % %     xtt = corner1(:,[2 1])';
end

img1 = rgb2gray(img1);
img2 = rgb2gray(img2);
img1 = mean(double(img1),3);
img2 = mean(double(img2),3);
xtt0 = pt1(:,[2 1])';
% goodfeat0 = ones(size(xtt0,2),1);



transMat = SkewSymMat(rt(1:3,4)./norm(rt(1:3,4)));
FundMat = inv(intrMat)'*transMat*rt(1:3,1:3)*inv(intrMat);
FundMat00 = FundMat./norm(FundMat);
FundMat0 = reshape(FundMat00,1,9);
% xtt0=[123;247];
% xtt0 = [216;172];
% goodfeat0 = 1;
% % tic;[xtt,goodfeat,Qtt, infoMat,~,resp] = track3(img1,img2,xtt0,goodfeat0,FundMat);toc;
% % pt2 = xtt([2 1],goodfeat&resp>0)';
% % pt11 = xtt0([2 1],goodfeat&resp>0)';
% % figure,showMatchedFeatures(img1,img2,pt11,pt2)

nim = ImgTransform(rgb2gray(img11), intrMat, rt(1:3,1:3));
pixRot = Orig2Rect(xtt0([2 1],logical(goodfeat0))', intrMat, intrMat, rt(1:3,1:3),zeros(5,1));


transMat = SkewSymMat(rt(1:3,4)./norm(rt(1:3,4)));
FundMat = inv(intrMat)'*transMat*eye(3)*inv(intrMat);
FundMat = FundMat./norm(FundMat);
FundMat = reshape(FundMat,1,9);


xtt00 = xtt0;
xtt00(:,logical(goodfeat0)) = pixRot(:,[2 1])';
nim = mean(double(nim),3);


[xtt,goodfeat,Qtt, infoMat, Var, resp] = track3_3(nim,img2,xtt00,goodfeat0,FundMat);
% tic;[xtt,goodfeat,Qtt, infoMat,~,resp] = track3_3(img1,img2,xtt0,goodfeat0,FundMat);toc
% [xtt,goodfeat,Qtt, infoMat] = track3_buggy(img1,img2,xtt0,goodfeat0,FundMat);
% pt2 = xtt([2 1],goodfeat&resp>0)';
pt2 = xtt([2 1],:)';
nanflag = isnan(pt2(:,1));
goodfeat(nanflag) = 0;
if sum(nanflag) ~= 0
    pt2(nanflag,:) = 0;
end
% pt11 = xtt0([2 1],goodfeat&resp>0)';
% figure,showMatchedFeatures(img1,img2,pt11,pt2)
% pt11 = xtt0([2 1],:)';figure,showMatchedFeatures(img1,img2,pt11,pt2)
end