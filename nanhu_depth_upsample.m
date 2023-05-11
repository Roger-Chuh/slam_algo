%% NANHU DEPTH 
%% AUTHOR: SHU.FANG
%% output: dispL ----  with 4 bits subpixel, mask---0 stands for invalid 
function [dispL, maskL] = nanhu_depth_upsample(imgL, imgR, minD, maxD, w1h, w1w, w2h, w2w, p1, p2, ratio)

imgL = uint8(imgL);
imgR = uint8(imgR);

[height,width, chn]=size(imgL);
imgL = imresize(imgL, [height, ratio * width]);
imgR = imresize(imgR, [height, ratio * width]);
maxD = ratio * maxD;
if(chn == 3)
    imgL = rgb2gray(imgL);
    imgR = rgb2gray(imgR);
end
[dispL, maskL] = nanhumex(imgL', imgR',ratio*width, height, minD, maxD, w1h, w1w, w2h, w2w, p1, p2);
dispL = reshape(dispL(:), [ratio*width, height])';
maskL = reshape(maskL(:), [ratio*width, height])';
dispL = imresize(dispL, [height, width])/ratio;
maskL = imresize(maskL, [height, width],'nearest');




dispL(find(~(maskL))) = nan;
dispL(dispL == 0) = nan;
maskL = maskL & (~isnan(dispL));
dispL = dispL.*2^-4;

end