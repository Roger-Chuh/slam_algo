function demo()

imgL = imread('rect_11_left.png');
imgR = imread('rect_11_right.png');
imgL = imresize(imgL, [480, 640]);
imgR = imresize(imgR, [480, 640]);
[dispL, maskL] = nanhu_depth(imgL, imgR, 0, 8, 7, 11, 21, 41, 2, 6);
figure(1),imshow(dispL.*maskL./2^4,[]);
figure(2), imshow(maskL,[]);
end