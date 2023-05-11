function [pixUndist, pixUndist3D] = remapFishEyePix(pixDistort, intrMat_same, oCamModel)
pixxDistort = pixDistort([2 1],:);
% M2 = cam2world_fast(pixxDistort, calib_data.ocam_model);
M2 = cam2world(pixxDistort, oCamModel);
M2 = [M2(1,:)./M2(3,:);M2(2,:)./M2(3,:);ones(1,size(pixxDistort,2))];
pixxUndist = intrMat_same*[-M2(2,:); -M2(1,:); M2(3,:)];
pixUndist3D = [-M2(2,:); -M2(1,:); M2(3,:)];
pixUndist = [pixxUndist(1,:)./pixxUndist(3,:); pixxUndist(2,:)./pixxUndist(3,:)];   %;ones(1,size(pixDistort,2))];



% % MM = [-M2(2,:); -M2(1,:); M2(3,:)];
% % MM_ = [-MM(2,:);-MM(1,:);MM(3,:)];
% % [M4,~] = NormalizeVector(MM_');
% % M33 = world2cam(-M4',oCamModel);
% % M33_ = M33([2 1],:);
% % figure,plot(M33_'-pixDistort([1 2],:)');
end