function [isbl, mm] = isBlur2(img,thr)


if ndims(img) == 3
    img = rgb2gray(img);
end

f = fspecial('laplacian');

imgg = imfilter(img,f);

% mm = std(double(imgg(:)));
mm = mean(double(imgg(:)));

if mm > thr    
    isbl = 0;
else
    isbl = 1;
end

end