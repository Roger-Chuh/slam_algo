function varargout = FixAxisPnp_bak(intrMat, thetaList, xyzKey, xMat, yMat, minFrameNum, RtTemp, b2c, inlierIdComm, rotAxisVec)

useORigB2C = false;
thetaList = thetaList - thetaList(1);

r = 1;
theta = rotAxisVec(1);
phi = rotAxisVec(2);
rotAxis = [r*sin(theta)*cos(phi); r*sin(theta)*sin(phi); r*cos(theta)];


pix = [xMat(:,1) yMat(:,1)];
% % indKey = sub2ind(size(depthKey), round(pix(:,2)), round(pix(:,1)));
% % depthKList = depthKey(indKey);
% % [xyzKey] = GetXYZFromDepth(intrMat, pix,depthKList);
ptNum = length(inlierIdComm);
ptIcsErr = zeros(ptNum*(minFrameNum - 1), 2);


b2c0 = b2c; %[eye(3) [10;45;-170.2];0 0 0 1];

for i = 1 : minFrameNum - 1
    pixCur = [xMat(:,i+1), yMat(:,i+1)];
    angNorm = thetaList(i+1);
    rotMat = rodrigues(angNorm*rotAxis);
    
     Tbody = [roty(rad2deg(double(angNorm))) [0;0;0];0 0 0 1];
    if 1
        if 0
            [rtTemp, ~] = posest(double(pixCur), double(xyzKey), 0.9, intrMat, 'repr_err');
        else
            rtTemp = [0;0;0;RtTemp(:,i)];
        end
        if 0
            ptIcsTemp = TransformAndProject(xyzKey, intrMat, rotMat, rtTemp(4:6));
        else
            
            Tcam = b2c0*Tbody*inv(b2c0);
            if ~useORigB2C
                ptIcsTemp = TransformAndProject(xyzKey, intrMat, rotMat, Tcam(1:3,4));
            else
                ptIcsTemp = TransformAndProject(xyzKey, intrMat, Tcam(1:3,1:3), Tcam(1:3,4));
            end
        end
    else
        ptIcsTemp = TransformAndProject(xyzKey, intrMat, rotMat, [0 0 0]');
    end
    ptIcsErr(ptNum*(i-1)+1:ptNum*(i),:) = ptIcsTemp(inlierIdComm,:) - pixCur(inlierIdComm,:);
    
    RT = [rotMat [0 0 0]';0 0 0 1];
    
    camPoseC2K(:,:,i) = inv(RT);
    camPoseK2C(:,:,i) = (RT);
    %                     bodyPoseK2C{i,1}(:,:,j) = [roty(rad2deg(double(thetaListTemp(j+1)))) [0;0;0];0 0 0 1];
    bodyPoseK2C(:,:,i) = Tbody;
    
    
    
end
if 0
    figure,plot(ptIcsErr(:,1), ptIcsErr(:,2), '+r');axis equal; drawnow;
end
[~, err] = NormalizeVector(ptIcsErr);
varargout{1} = err;


if (nargout > 1)
    varargout{2} = camPoseC2K;
    varargout{3} = camPoseK2C;
    varargout{4} = bodyPoseK2C;
else
    asgk = 1;
end


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