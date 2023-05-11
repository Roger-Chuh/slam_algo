function pixDist = remapRectFishEye(pixRect, KRect, KUndist,distCoeff, RL,oCamModel)

alpha = 0;

rays = inv(KRect)*pextend(pixRect);


% Rotation: (or affine transformation):

rays2 = RL'*rays;

x = [rays2(1,:)./rays2(3,:);rays2(2,:)./rays2(3,:)];
if 0
    pixxDistort = pixDistort([2 1],:);
    % M2 = cam2world_fast(pixxDistort, calib_data.ocam_model);
    M2 = cam2world(pixxDistort, oCamModel);
    M2 = [M2(1,:)./M2(3,:);M2(2,:)./M2(3,:);ones(1,size(pixxDistort,2))];
    pixxUndist = intrMat_same*[-M2(2,:); -M2(1,:); M2(3,:)];
    pixUndist3D = [-M2(2,:); -M2(1,:); M2(3,:)];
    pixUndist = [pixxUndist(1,:)./pixxUndist(3,:); pixxUndist(2,:)./pixxUndist(3,:)];   %;ones(1,size(pixDistort,2))];
else
    % %     MM = inv(intrMat_same)*[pixUnDistort; ones(1,size(pixUnDistort,2))];
    MM = rays2;
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
        % %                 a
        M33 = world2cam_fast(-M4',oCamModel);
    catch
        M33 = world2cam(-M4',oCamModel);
    end
    M33_ = M33([2 1],:);
    pixDistorted = M33_';
    pixDist = pixDistorted;
end

% Add distortion:
% xd = apply_distortion(x,distCoeff);
%
%
% % Reconvert in pixels:
%
% px2_ = KUndist(1,1)*(xd(1,:) + alpha*xd(2,:)) + KUndist(1,3);
% py2_ = KUndist(2,2)*xd(2,:) + KUndist(2,3);
% pixDist = [px2_;py2_]';

end