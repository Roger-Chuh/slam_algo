function fig_3D3_pair(UU,X2)




dlt = (UU - X2)';
[dirVec,xyzErr] = NormalizeVector(dlt);

for k = 1 : size(dlt,1)
    r = rodrigues(dlt(k,:)');
    poseVec(k,:) = [X2(:,k)' r(1:3,1)' r(1:3,2)' r(1:3,3)'];
end



XX = 3000000;
ZZ  =   6000000;  0.65;  0.16;   8000; 0.3;

figure; 

%     subplot(1,2,1);
%     pcshow(X2(:,X2(1,:)<XX & X2(3,:)<ZZ)', [0 0 1], 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
%     xlabel('X (mm)');
%     ylabel('Y (mm)');
%     zlabel('Z (mm)');
%     set(gca,  'CameraUpVector',[0 -1 0],'DataAspectRatio',[1 1 1]);
%     title('proposed method');
    
    
%     subplot(1,2,2);
    hold on;
    
    quiver3(poseVec(1:end,1),poseVec(1:end,2),poseVec(1:end,3),dirVec(1:end,1),dirVec(1:end,2),dirVec(1:end,3),0.3);
    pcshow(X2(:,X2(1,:)<XX & X2(3,:)<ZZ)', [0 0 1], 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
    plot3(X2(1,1),X2(2,1),X2(3,1),'*g','MarkerSize',5,'LineWidth',5);
% %     quiver3(poseVec(1:end,1),poseVec(1:end,2),poseVec(1:end,3),poseVec(1:end,4),poseVec(1:end,5),poseVec(1:end,6),0.1);
% %     quiver3(poseVec(1:end,1),poseVec(1:end,2),poseVec(1:end,3),poseVec(1:end,7),poseVec(1:end,8),poseVec(1:end,9),0.1);
%     quiver3(poseVec(1:end,1),poseVec(1:end,2),poseVec(1:end,3),poseVec(1:end,10),poseVec(1:end,11),poseVec(1:end,12),0.1);
     
    
    
    pcshow(UU(:,UU(1,:)<XX & UU(3,:)<ZZ)', [1 0 0], 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
    legend('direction','proposed method','sfm reference');
    xlabel('X (mm)');
    ylabel('Y (mm)');
    zlabel('Z (mm)');
    set(gca,  'CameraUpVector',[0 -1 0],'DataAspectRatio',[1 1 1]);
    title('Reconstructed 3-D Scene');

   

afbv = 1;
if 0
figure, 

    
    
    
point3D = ones([480 640 3]);
pix = round(pix);
[~,ia,~] = intersect(pix,pix,'rows');
pix = pix(ia,:);
xyz = xyz(ia,:);

flag = pix(:,1)>=1&pix(:,1)<=size(dispMat,2)&pix(:,2)>=1&pix(:,2)<=size(dispMat,1);
pix = pix(flag,:);
xyz = xyz(flag,:);
point3D(:,:,1) = GetCoordImage([pix xyz],size(dispMat),'x');
point3D(:,:,2) = GetCoordImage([pix xyz],size(dispMat),'y');
point3D(:,:,3) = GetCoordImage([pix xyz],size(dispMat),'z');
    
pcshowpair(pointCloud(point3DxyzAll),pointCloud(point3DBackProj),'VerticalAxis','Y','VerticalAxisDir','Down');xlabel('X (mm)');ylabel('Y (mm)');zlabel('Z (mm)');set(gca,  'CameraUpVector',[0 -1 0],'DataAspectRatio',[1 1 1]);title('robust fit');
end






end