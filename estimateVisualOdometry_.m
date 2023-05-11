function [pose_rel, score, warped_image, valid_mask, weightMap, warped_z, validMask] = estimateVisualOdometry_(img_curr, img_prev, dep_curr, dep_prev, K, num_levels, pose_init, isVerbose)
% estimate visual odometry from two adjacent frames
%
% INPUT:
%   img_curr: current RGB frame
%   img_prev: previous RGB frame
%   dep_prev: previous depth frame
%   K: intrinsic parameters of the camera
%   num_levels: pyramid levels for the estimation
%   isVerbose: whether to show intermediate results
%
% OUTPUT:
%   pose_rel: relative pose
%   score: fitting score

% construct image pyramids
img_curr_pyr = constructPyramid(img_curr, num_levels);
img_prev_pyr = constructPyramid(img_prev, num_levels);

% construct depth pyramid for the previous frame
dep_prev_pyr = constructPyramid(dep_prev, num_levels);
dep_curr_pyr = constructPyramid(dep_curr, num_levels);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% estimate the relative pose from coarse to fine scales

% initialize the relative pose and its increment
pose_rel = pose_init; % eye(4);
increment = zeros(6, 1);

% convert the vectorize motion increment to update the relative pose
increment = twistexp(increment);
pose_rel = increment * pose_rel;

% modify the camera parameters to fit each pyramid
K_pyr = K;
K_pyr(1:2, :) = K_pyr(1:2, :) / (2^(num_levels-1));

[height_end, width_end] = size(dep_prev_pyr{end});
K_pyr(1, 3) = (1 + width_end) / 2;
K_pyr(2, 3) = (1 + height_end) / 2;

minErr = 1e-10;
minStep = 1e-10;
maxIter = 100; 50;
% prev_sqError = inf;
for n = num_levels:-1:1
    
    % image size
    if 0
        [height, width] = size(dep_prev_pyr{n});
        
        % get valid point clouds in the previous frame
        [pointclouds, valid_mask] = reprojectDepthImage(dep_prev_pyr{n}, K_pyr);
        
        % warp pointclouds and prune invalid points
        
        warped_pointclouds = warpPointCloud(pointclouds, pose_rel);
        [warped_img_coordinates, valid_points, ind] = projectPointCloud(warped_pointclouds, K_pyr, height, width);
        
        warped_pointclouds = warped_pointclouds(valid_points, :);
        pointclouds = pointclouds(valid_points, :);
        
        % spatial gradient in the current frame
        [Gx_curr, Gy_curr] = imgradientxy(img_curr_pyr{n}, 'CentralDifference');
        Gx_curr = interp2(Gx_curr, warped_img_coordinates(:, 1), warped_img_coordinates(:, 2), 'linear', 0);
        Gy_curr = interp2(Gy_curr, warped_img_coordinates(:, 1), warped_img_coordinates(:, 2), 'linear', 0);
        Gs_curr = cat(2, Gx_curr, Gy_curr);
        
        % temporal visual difference
        Gt_prev = img_prev_pyr{n};
        Gt_prev = Gt_prev(valid_mask);
        Gt_prev = Gt_prev(valid_points);
        Gt_curr = img_curr_pyr{n};
        Gt_curr = interp2(Gt_curr, warped_img_coordinates(:, 1), warped_img_coordinates(:, 2), 'linear', 0);
        Gt = Gt_curr - Gt_prev;
        
        % calculate the warping Jacobian
        warp_jacobian = calculateWarpingJacobian(warped_pointclouds, pointclouds, pose_rel, K_pyr);
        
        % calculate the compositive jacobian
        comp_jacobian = squeeze(sum(bsxfun(@times, Gs_curr, warp_jacobian), 2));
        if 1
            % adjust the importance of each residual elements, when required
            % necessary pruning of bad correspondences
            % compute the weight
            variance = computeResidualVariance(Gt, 5);
            weights = sqrt((5 + 1) ./ (5 + Gt.^2./variance));
            comp_jacobian = bsxfun(@times, comp_jacobian, weights);
            Gt = Gt.*weights;
            
            weightMap = zeros(height, width);
            weightMap(ind) = weights;
        end
        
        % calculate the increment motion
        increment = -(comp_jacobian'*comp_jacobian)\(comp_jacobian'*Gt);
        
        % get the current relative pose
        increment = twistexp(increment);
        pose_rel = increment * pose_rel;
    else
        prev_sqError = inf;
        for kk = 1 : maxIter
            [pose_rel2, error, weightMap] = iterPose(pose_rel, K_pyr, img_curr_pyr, img_prev_pyr, dep_curr_pyr, dep_prev_pyr, n);
            if (error > prev_sqError(end) || error < minErr || prev_sqError(end) - error < minStep)
%                 pose_rel = pose_rel2;
                break;
            end
            prev_sqError = [prev_sqError; error];
            pose_rel = pose_rel2;
        end

    end
    % intermediate results
    if isVerbose
        [warped_image, valid_mask] = warpImage_(img_curr, dep_prev, pose_rel, K);
        error = mean((warped_image(valid_mask) - img_prev(valid_mask)).^2);
        disp(['visual consistency score in level ' num2str(n) ' is ' num2str(error)]);
        
        figure(1);
        imshow(abs(warped_image - img_prev).*valid_mask, [])
    end
    
    % increse the focal length
    if 0
        K_pyr(1:2, :) = K_pyr(1:2, :) * 2;
    else
        if n ~= 1
            K_pyr(1, 1) = K_pyr(1, 1) * 2;
            K_pyr(2, 2) = K_pyr(2, 2) * 2;
            [height_next, width_next] = size(dep_prev_pyr{n-1});
            K_pyr(1,3) = (1 + width_next) / 2;
            K_pyr(2,3) = (1 + height_next) / 2;
        end
    end
end


[UU,SS,VV] = svd(pose_rel(1:3,1:3));
pose_rel(1:3,1:3) = UU*VV';

% get the final score
[warped_image, valid_mask, warped_z, validMask] = warpImage_(img_curr, dep_prev, pose_rel, K);
score = mean((warped_image(valid_mask) - img_prev(valid_mask)).^2);

if isVerbose
    disp(['The fitting score is ' num2str(error)]);
end

end
function [pose_rel2, error, weightMap] = iterPose(pose_rel, K_pyr, img_curr_pyr, img_prev_pyr, dep_curr_pyr, dep_prev_pyr, n)
[height, width] = size(dep_prev_pyr{n});

% get valid point clouds in the previous frame
[pointclouds, valid_mask] = reprojectDepthImage(dep_prev_pyr{n}, K_pyr);

% warp pointclouds and prune invalid points
warped_pointclouds = warpPointCloud(pointclouds, pose_rel);
[warped_img_coordinates, valid_points, ind] = projectPointCloud(warped_pointclouds, K_pyr, height, width);

warped_pointclouds = warped_pointclouds(valid_points, :);
pointclouds = pointclouds(valid_points, :);

% spatial gradient in the current frame
[Gx_curr, Gy_curr] = imgradientxy(img_curr_pyr{n}, 'CentralDifference');
Gx_curr = interp2(Gx_curr, warped_img_coordinates(:, 1), warped_img_coordinates(:, 2), 'linear', 0);
Gy_curr = interp2(Gy_curr, warped_img_coordinates(:, 1), warped_img_coordinates(:, 2), 'linear', 0);
Gs_curr = cat(2, Gx_curr, Gy_curr);

% temporal visual difference
Gt_prev = img_prev_pyr{n};
Gt_prev = Gt_prev(valid_mask);
Gt_prev = Gt_prev(valid_points);
Gt_curr = img_curr_pyr{n};
Gt_curr = interp2(Gt_curr, warped_img_coordinates(:, 1), warped_img_coordinates(:, 2), 'linear', 0);
Gt = Gt_curr - Gt_prev;

% calculate the warping Jacobian
warp_jacobian = calculateWarpingJacobian(warped_pointclouds, pointclouds, pose_rel, K_pyr);

% calculate the compositive jacobian
comp_jacobian = squeeze(sum(bsxfun(@times, Gs_curr, warp_jacobian), 2));
weightMap = ones(height, width);
if 0
    % adjust the importance of each residual elements, when required
    % necessary pruning of bad correspondences
    % compute the weight
    variance = computeResidualVariance(Gt, 5);
    weights = sqrt((5 + 1) ./ (5 + Gt.^2./variance));
    comp_jacobian = bsxfun(@times, comp_jacobian, weights);
    Gt = Gt.*weights;
    
    weightMap = zeros(height, width);
    weightMap(ind) = weights;
end

% calculate the increment motion
increment = -(comp_jacobian'*comp_jacobian)\(comp_jacobian'*Gt);

% get the current relative pose
increment = twistexp(increment);
pose_rel2 = increment * pose_rel;
[warped_image, valid_mask] = warpImage_(img_curr_pyr{n}, dep_prev_pyr{n}, pose_rel2, K_pyr);
error1 = mean((warped_image(valid_mask) - img_prev_pyr{n}(valid_mask)).^2);
if 0
    [warped_image2, valid_mask2] = warpImage_(img_prev_pyr{n}, dep_curr_pyr{n}, inv(pose_rel2), K_pyr);
%     error1 = mean((warped_image(valid_mask) - img_prev_pyr{n}(valid_mask)).^2);
    error2 = mean((warped_image2(valid_mask2) - img_curr_pyr{n}(valid_mask2)).^2);
    error = error1  + error2;
else
    error = error1;
end
end