function [pixRect, pixUndistR, M22] = Orig2RectFishEye(pix, KOrig, KRect, R,kc,oCamModel)

% % % [pixUndist] = normalize_pixel(pix',[KOrig(1,1);KOrig(2,2)],[KOrig(1,3);KOrig(2,3)],kc,0);

pixxDistort = pix(:,[2 1])';
% M2 = cam2world_fast(pixxDistort, calib_data.ocam_model);
M2 = cam2world(pixxDistort, oCamModel);
M22 = [-M2(2,:); -M2(1,:); M2(3,:)];
M2 = [M2(1,:)./M2(3,:);M2(2,:)./M2(3,:);ones(1,size(pixxDistort,2))];
% pixxUndist = KOrig*[-M2(2,:); -M2(1,:); M2(3,:)];
pixUndist3D = [-M2(2,:); -M2(1,:); M2(3,:)];
% pixUndist = [pixxUndist(1,:)./pixxUndist(3,:); pixxUndist(2,:)./pixxUndist(3,:)];


pixUndistR = R*(pixUndist3D);
pixUndistR3D_norm = normc(pixUndistR);
pixRect = pflat(KRect*pixUndistR);
pixRect = pixRect(1:2,:)';



end