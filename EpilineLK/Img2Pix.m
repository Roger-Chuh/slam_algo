function pix = Img2Pix(img,dispMat)

ind = find(img(:) >0);
[py, px] = ind2sub(size(dispMat),ind);
pix = [px py];

end