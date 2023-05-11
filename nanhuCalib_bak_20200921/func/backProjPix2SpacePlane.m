function intersectPt = backProjPix2SpacePlane(ptPhyPrvHomo,initGroundParam)

% (0 0 0) is always on the backProj line, because it is the origin of Ccs.


% each row of ptPhyPrvHomo(n-by-3) is a physics coordinate on the image
% plane which is also on the backProj line, (3rd column is 1)


% initGroundParam is the space plane to be intersected by the backProj
% line


initGroundParam = initGroundParam./norm(initGroundParam(1:3));


    tempa = initGroundParam(1);
    tempb = initGroundParam(2);
    tempc = initGroundParam(3);
    tempd = initGroundParam(4);


% % % % % % % % % % % % % for i = 1 : size(ptPhyPrvHomo,1)
% % % % % % % % % % % % %     
% % % % % % % % % % % % %     tempu = ptPhyPrvHomo(i,1);
% % % % % % % % % % % % %     tempv = ptPhyPrvHomo(i,2);
% % % % % % % % % % % % %     tempw = ptPhyPrvHomo(i,3);
% % % % % % % % % % % % %     
% % % % % % % % % % % % %     
% % % % % % % % % % % % %     intersectX = -tempd/(tempa+tempb*tempv/tempu+tempc*tempw/tempu);
% % % % % % % % % % % % %     intersectY = tempv*intersectX/tempu;
% % % % % % % % % % % % %     intersectZ = tempw*intersectX/tempu;
% % % % % % % % % % % % %     intersectPt(i,:) = [intersectX intersectY intersectZ];
% % % % % % % % % % % % %       
% % % % % % % % % % % % % end




intersectXX = -tempd./(tempa+tempb.*ptPhyPrvHomo(:,2)./ptPhyPrvHomo(:,1)+tempc.*ptPhyPrvHomo(:,3)./ptPhyPrvHomo(:,1));
intersectYY = ptPhyPrvHomo(:,2).*intersectXX./ptPhyPrvHomo(:,1);
intersectZZ = ptPhyPrvHomo(:,3).*intersectXX./ptPhyPrvHomo(:,1);

intersectPt = [intersectXX intersectYY intersectZZ];

if 0
    
[x,y] = meshgrid(-12:4:12, -12:4:12);
    z = (-tempd-tempa*x-tempb*y)/tempc;
    figure,surf(x,y,z);
    hold on
%     plot3(0,0,0,'Color',[1 0 0],'MarkerSize',8,'LineWidth',2)
%     axis equal
       xlabel('X (mm)');
   ylabel('Y (mm)');
   zlabel('Z (mm)');
   set(gca,  'CameraUpVector',[0 1 0],'DataAspectRatio',[1 1 1]);
    
end



end