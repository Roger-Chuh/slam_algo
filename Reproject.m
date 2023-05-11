function pixCur = Reproject(intrMat, pixPrv, zPrv, theta, b2c)
Tcam = b2c * [roty(double(rad2deg(theta))) [0 0 0]'; 0 0 0 1] * inv(b2c);
[xyzPrv] = GetXYZFromDepth(intrMat, pixPrv, zPrv);
pixCur = TransformAndProject(xyzPrv, intrMat, Tcam(1:3,1:3), Tcam(1:3,4));
end
function [ptIcs, tgtPt3d] = TransformAndProject(pt3d, intrMat, rotMat2Ccs, transVec2Ccs)

% TransformAndProject transforms 3d points in a coordinate system to an
% camera coordinate system and project them to image plane of camera.
% pt3d: numPt by 3 matrix, list of 3d points. Each row is the coordinates
% of a point in the form of [x,y,z]
% intrMat: 3 by 3 intrinsic matrix
% rotMatToCcs: 3 by 3 rotation matrix transforming from current coordinate
% system to camera coordinate system.
% transVecToCcs: 3 by 1 translation vector (in ICS).
% Returned ptIcs is a numPt by 2 matrix with each row a projected image
% point corresponding to each 3d point in pt3d.
%
% By Ji Zhou

tgtPt3d = EucildTransform(pt3d, rotMat2Ccs, transVec2Ccs);
ptIcs = ProjectToImage(tgtPt3d, intrMat);

end
function [XYZ] = GetXYZFromDepth(intrMat, Pix,depthList)
metricPrevPtCcsGT = intrMat\HomoCoord(Pix',1);
metricPrevPtCcsGT = normc(metricPrevPtCcsGT);
if 1
    if 0
        if 0
            scaleAllGT = depthListGT(inlierId)./metricPrevPtCcsGT(3,:)';
        else
            scaleAllGT = depthListGT(:)./metricPrevPtCcsGT(3,:)';
        end
    else
        scaleAllGT = depthList./metricPrevPtCcsGT(3,:)';
    end
else
    dispGTComp = dispList(inlierId) - disparityErrorRound;
    depthListGTComp = intrMat(1,1).*norm(obj.camModel.transVec1To2)./(dispGTComp + (princpPtR(1) - princpPtL(1)));
    scaleAllGT = depthListGTComp./metricPrevPtCcsGT(3,:)';
end

XYZ = [repmat(scaleAllGT',3,1).*metricPrevPtCcsGT]';




end