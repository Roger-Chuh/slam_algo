function RectificationModel
% note: rectImgL,trectImgR are output from our model
%       rectImgL_ref,rectImgR_ref are output from standard rectification procedure





load('D:\bk_20180627_\nextvpu\algorithm\output\calibration\cbccalib\results\iEye\zed2\calib.mat');

imgL = imread('D:\bk_20180627_\nextvpu\algorithm\output\calibration\cbccalib\stereo_pairs\iEye\zed2\imgL__left0001.png');
imgR = imread('D:\bk_20180627_\nextvpu\algorithm\output\calibration\cbccalib\stereo_pairs\iEye\zed2\imgL__right0001.png');

[rectParamL, rectParamR,rectImgL,rectImgR] = GetRectifyParam_table(stereoParam, size(imgL),imgL,imgR);

[rectImgL_ref, ~, ~] = RectifyOneImage(imgL, rectParamL);
[rectImgR_ref, ~, ~] = RectifyOneImage(imgR, rectParamR);


figure,imshowpair(rectImgL_ref,rectImgL);title('left image');
figure,imshowpair(rectImgR_ref,rectImgR);title('right image');



end