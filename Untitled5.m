


PoseGyroInv = [];

for u = 1 : size(PoseGyro,1)
    T = [reshape(PoseGyro(u,1:9),3,3) PoseGyro(u,10:12)';0 0 0 1];
    T_inv = inv(T);
    PoseGyroInv = [PoseGyroInv; [reshape(T_inv(1:3,1:3),1,9) T_inv(1:3,4)']];
    
    rotVecGyro1(:,u) = rad2deg(rodrigues(reshape(PoseGyro(u,1:9),3,3)));
    eulerGyro2(:,u) = rad2deg(rotMat2euler(reshape(PoseGyro(u,1:9),3,3))');
    rotAxisGyro_ = rotVecGyro1(:,u)./norm(rotVecGyro1(:,u));
    rotAxisGyro1(:,u) = sign(rotAxisGyro_(2)).*rotAxisGyro_;
end


for i = 1:size(PoseGyroInv,1)-1
     
rotVecGyro(:,i) =  rodrigues(reshape(PoseGyroInv(i+1,1:9),3,3)*inv(reshape(PoseGyroInv(i,1:9),3,3)));
rotAngGyro(i,:) = rad2deg(norm(rotVecGyro(:,i)));
rotAxisGyro(:,i) =  rotVecGyro(:,i)./norm(rotVecGyro(:,i));


rotVecGyroW(:,i) =  rodrigues(inv(reshape(PoseGyro(i+1,1:9),3,3))*(reshape(PoseGyro(i,1:9),3,3)));
rotAxisGyroW(:,i) =  rotVecGyroW(:,i)./norm(rotVecGyroW(:,i));


rotVecGyroW0(:,i) =  rodrigues((reshape(PoseGyro(i,1:9),3,3)));
rotAxisGyroW0(:,i) =  rotVecGyroW0(:,i)./norm(rotVecGyroW0(:,i));
end
 
avdasv = 1;


% figure,plot(rotAxisGyroW([1],70:end)'-rotAxisGyro([1],70:end)');

% figure,plot(rotAngGyro);
% figure,plot(rotAxisGyro([1 3],70:end)');


angErr = rad2deg(vsl.poseWcsList(:,3) - bsl.poseWcsList(:,3));
noise = 0.1*((rand(length(angErr),1))-0.5)./5;
noise = round(noise./0.01).*0.01;
for ji = 1 : length(angErr) - 1
    deltaAng = angErr(ji + 1) - angErr(ji);
    if abs(deltaAng) > 0.05
        angErr(ji + 1) = angErr(ji) + 0.1*((rand(1,1))-0.5);
    end
end
pulse = repmat(min(angErr),size(vsl.poseWcsList,1),1);
pulse(idPulse) = max(angErr);
figure(12),clf;plot(angErr);title('visual-body');hold on;plot(find(vsl.keyFrameFlagList),angErr(find(vsl.keyFrameFlagList)),'or');plot(pulse,'k');%plot(scaleNum.*rotAxisGyro([3],1:end)','g'); %plot(deg2rad(Euler(:,3)));











% % [idxCell,idx] = splitIndex2(find(abs(angErr)>0.1));
% % for ij = 1:length(idxCell)
% %     
% %    idTemp = idxCell{ij}; 
% %     angErr(idTemp) = angErr(idTemp(1)-1);
% % end
 










% 
% for u = 1 : size(PoseWheel,1)
%     rotVec(:,u) = rad2deg(rodrigues(reshape(PoseWheel(u,1:9),3,3)));
%     euler2(:,u) = rad2deg(rotMat2euler(reshape(PoseWheel(u,1:9),3,3))');
%     rotAxis_ = rotVec(:,u)./norm(rotVec(:,u));
%     rotAxis(:,u) = sign(rotAxis_(2)).*rotAxis_;
% end
% 
% 
% for u = 1 : size(BsSampFrame,1)
%      
%     euler3(:,u) = rad2deg(quatern2euler(BsSampFrame(u,[13 10 11 12])))';
% 
% end
% 
% function err = reprojErr(intrMat, xyz, curPt, b2c, theta_)
% if 0
%     theta = theta_(1);
%     b2c(1:3,4) =   theta_(2:4)';  [15;287;-96];
%     dltR = b2c(1:3,1:3)*roty(theta)*b2c(1:3,1:3)';
%     dltT = -b2c(1:3,1:3)*roty(theta)*b2c(1:3,1:3)'*b2c(1:3,4) + b2c(1:3,4);
%     rt = b2c*[roty(theta) [0;0;0];0 0 0 1]*inv(b2c);
% else
%     theta = theta_(1);
%     b2c(1:3,1:3) = rodrigues(theta_(2:4));
%     b2c(1:3,4) =   theta_(5:7)';  [15;287;-96];
%     dltR = b2c(1:3,1:3)*roty(theta)*b2c(1:3,1:3)';
%     dltT = -b2c(1:3,1:3)*roty(theta)*b2c(1:3,1:3)'*b2c(1:3,4) + b2c(1:3,4);
%     rt = b2c*[roty(theta) [0;0;0];0 0 0 1]*inv(b2c);
% end
