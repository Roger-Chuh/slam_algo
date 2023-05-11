function [warped_image, valid_mask, warped_z, validMask] = warpImage_(img_curr, dep_prev, pose_rel, K)
% warp the current image to the previous image coordinate
%
% INPUT:
%   img_curr: current grayscale image
%   dep_prev: previous depth image
%   pose_rel: relative pose between the current and previous frame
%   K: intrinsic camera parameters
%
% OUTPUT:
%   warped_image: warped image by the rigid transformation
%   valid_mask: valid pixels in the warped image

[height, width] = size(img_curr);

[pointclouds, valid_mask] = reprojectDepthImage(dep_prev, K);
warped_pointclouds = warpPointCloud(pointclouds, pose_rel); % prv2cur depth
[warped_img_coords, warped_valid_mask] = projectPointCloud(warped_pointclouds, K, height, width);

warped_intensities = interp2(img_curr, warped_img_coords(:, 1), warped_img_coords(:, 2), 'linear', 0);

% warped_intensities(warped_intensities > 0) = 255;


valid_indices = find(valid_mask);
valid_indices = valid_indices(warped_valid_mask);
zWarpped = warped_pointclouds(warped_valid_mask,3);

valid_mask = zeros(height, width);
valid_mask(valid_indices) = 1;
valid_mask = logical(valid_mask);

warped_image = zeros(height, width);
warped_image(valid_indices) = warped_intensities;


ind2 = sub2ind(size(warped_image), round(warped_img_coords(:,2)), round(warped_img_coords(:,1)));

warped_z = zeros(height, width);
validMask = zeros(height, width);
if 0
    warped_z(valid_indices) = zWarpped;
else
    warped_z(ind2) = zWarpped;
    validMask(ind2) = 1;
end

end