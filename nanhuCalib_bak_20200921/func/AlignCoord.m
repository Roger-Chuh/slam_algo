function rt = AlignCoord(matchPtBody,matchPtVR)
bodyCtrMass = mean(matchPtBody);
vrCtrMass = mean(matchPtVR);
bodyPt = matchPtBody - repmat(bodyCtrMass,size(matchPtBody,1),1);
vrPt = matchPtVR - repmat(vrCtrMass,size(matchPtVR,1),1);
[U,S,V] = svd(vrPt'*bodyPt);
H = U*S*V';
X = V*U';
% %     [U,S,V] = svd(bodyPt'*vrPt);
% %     H = U*S*V';
% %     X = U'*V;
if abs(det(X) - (-1)) < 0.0001
    %     if X(2,2) < 0
    V(:,3) = -V(:,3);
    R = V*U';
else
    R = X;
    
end
% %     R(3,3) = -R(3,3);
t = bodyCtrMass' - R*vrCtrMass';
rt = [R t];


mapped = (R*matchPtVR' + repmat(t,1,size(matchPtVR,1)))';
if 0
    figure,plot3(matchPtBody(:,1),matchPtBody(:,2),matchPtBody(:,3),'or');hold on;plot3(mapped(:,1),mapped(:,2),mapped(:,3),'.g');axis equal;
    
    T = reprojection(matchPtBody', mapped', 1);
    xyz_opt1 = (T(1:3, 1:3) * matchPtBody' + T(1:3, 4))';
figure,plot3(xyz_opt1(:,1), xyz_opt1(:,2), xyz_opt1(:,3),'.r');hold on;plot3(mapped(:,1), mapped(:,2), mapped(:,3),'.b');axis equal;
    
    
    [~,err] = NormalizeVector(matchPtBody - mapped);
end
end