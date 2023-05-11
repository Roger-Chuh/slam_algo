function [projErr, err] = ProjectErr3dTo2d(pt2d, pt3d, intrMat, rotMat, transVec)


% pt2d current
% pt3d prv
% rotMat prv to cur


% ProjectErr3dTo2d calculates the average error between the projection of
% 3d points to 2d imaging plane and given 2d points.
% pt2d: numPt by 2 matrix, 2d reference points in the form of [x,y]
% pt3d: numPt by 3 matrix, 3d points to project in the form of [x,y,z]
% intrMat: 3 by 3 matrix, camera intrinsic matrix.
% rotMat: 3 by 3 rotation matrix which together with transVec transforms
% pt3d to the camera coordinate system
% transVec: 3 by 1 vector, translation vector
% Returned projErr is a salar which is the norm of average projection
% error (2d vector).
%
% By Ji Zhou


% load('lsq.mat');
ptProj = (intrMat * bsxfun(@plus, rotMat * pt3d', transVec))';
ptProj = bsxfun(@rdivide, ptProj(:,1:2), ptProj(:,3));
projErr = norm(mean(abs(pt2d - ptProj)));
[~,err] = NormalizeVector(pt2d - ptProj);
end