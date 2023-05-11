function varargout = GetCoordImage(ptCloud, imgSize, whichCoord)

img = NaN(imgSize);
ind = sub2ind(imgSize, ptCloud(:,2), ptCloud(:,1));

switch whichCoord
    case 'x'
        img(ind) = ptCloud(:,3);
%         img(ind) = [ptCloud(:,3) ptCloud(:,4) ptCloud(:,5);
    case 'y'
        img(ind) = ptCloud(:,4);
    case 'z'
        img(ind) = ptCloud(:,5);
    otherwise
        assert(0);
end
% img(img<-1000)=-1000;
% img(img<-30000)=-30000;
% img(img>30000)=30000;
% img(img(:)==0)=-30000;
if (nargout == 1)
    varargout{1} = img;
else
    imshow(img, []);title(whichCoord);
end

end