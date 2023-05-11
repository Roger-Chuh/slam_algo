function pt = FindEdgePoints(img)
    marginPix = 10;
    MotionBlur = SobelEdge(img);
    corner1 = Img2Pix(immultiply(abs(MotionBlur),abs(MotionBlur)>35),immultiply(abs(MotionBlur),abs(MotionBlur)>35));
    pt = corner1(corner1(:,1) <= size(img,2)-marginPix & corner1(:,1) >= marginPix & corner1(:,2) <= size(img,1)-marginPix & corner1(:,2) >= marginPix,:);
end