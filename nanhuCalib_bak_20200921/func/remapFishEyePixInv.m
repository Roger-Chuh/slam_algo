function pixDistorted = remapFishEyePixInv(pixUnDistort, intrMat_same, oCamModel)

MM = inv(intrMat_same)*[pixUnDistort; ones(1,size(pixUnDistort,2))];
% % pixxDistort = pixDistort([2 1],:);
% % % M2 = cam2world_fast(pixxDistort, calib_data.ocam_model);
% % M2 = cam2world(pixxDistort, oCamModel);
% % M2 = [M2(1,:)./M2(3,:);M2(2,:)./M2(3,:);ones(1,size(pixxDistort,2))];
% % pixxUndist = intrMat_same*[-M2(2,:); -M2(1,:); M2(3,:)];
% % pixUndist3D = [-M2(2,:); -M2(1,:); M2(3,:)];
% % pixUndist = [pixxUndist(1,:)./pixxUndist(3,:); pixxUndist(2,:)./pixxUndist(3,:)];   %;ones(1,size(pixDistort,2))];
% % 
% % MM = [-M2(2,:); -M2(1,:); M2(3,:)];
MM_ = [-MM(2,:);-MM(1,:);MM(3,:)];
[M4,~] = NormalizeVector(MM_');
try
    M33 = world2cam_fast(-M4',oCamModel);
catch
    M33 = world2cam(-M4',oCamModel);
end
M33_ = M33([2 1],:);
pixDistorted = M33_';
if 0
    figure,plot(M33_'-pixDistort([1 2],:)');
end

end