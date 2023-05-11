%% NANHU DEPTH 
%% AUTHOR: SHU.FANG
%% output: dispL ----  with 4 bits subpixel, mask---0 stands for invalid 
function [dispL, maskL] = nanhu_depth(imgL, imgR, minD, maxD, w1h, w1w, w2h, w2w, p1, p2)

imgL = uint8(imgL);
imgR = uint8(imgR);
[height, width, chn]=size(imgL);
if(chn == 3)
    imgL = rgb2gray(imgL);
    imgR = rgb2gray(imgR);
end
[dispL, maskL] = nanhumex(imgL', imgR',width, height, minD, maxD, w1h, w1w, w2h, w2w, p1, p2);
dispL = reshape(dispL(:), [width, height])';
maskL = reshape(maskL(:), [width, height])';
dispL(find(~(maskL))) = nan;
dispL(dispL == 0) = nan;
maskL = maskL & (~isnan(dispL));
dispL = dispL.*2^-4;
% % dispL = dispL + 0.055;
end