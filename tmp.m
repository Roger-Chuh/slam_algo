R0 = eye(3);
rotVecI2W = [];
for u = 1:size(Euler)
    rotTmp = euler2rotMat(Euler(u,2),Euler(u,3),Euler(u,4))*R0;
    rotVecI2W = [rotVecI2W;rodrigues(rotTmp)'];
    R0 = rotTmp;
end


R0 = quatern2rotMat(ImuData(32,[6 3 4 5]));
R1 = quatern2rotMat(ImuData(36,[6 3 4 5]));
rodrigues(ImuData(32,7:9)*0.02)*rodrigues(ImuData(33,7:9)*0.02)*rodrigues(ImuData(34,7:9)*0.02)*rodrigues(ImuData(35,7:9)*0.02)
rodrigues(ImuData(32,7:9)*0.02)-rodrigues(ImuData(32,7:9)*0.02)*rodrigues(ImuData(33,7:9)*0.02)*rodrigues(ImuData(34,7:9)*0.02)*rodrigues(ImuData(35,7:9)*0.02)
rodrigues(ImuData(32,7:9)*0.02)*rodrigues(ImuData(33,7:9)*0.02)*rodrigues(ImuData(34,7:9)*0.02)*rodrigues(ImuData(35,7:9)*0.02)
rodrigues(rodrigues(ImuData(32,7:9)*0.02)*rodrigues(ImuData(33,7:9)*0.02)*rodrigues(ImuData(34,7:9)*0.02)*rodrigues(ImuData(35,7:9)*0.02))
dtR0 = rodrigues(R1'*R0)
rodrigues(rodrigues(ImuData(32,7:9)*0.02)*rodrigues(ImuData(33,7:9)*0.02)*rodrigues(ImuData(34,7:9)*0.02)*rodrigues(ImuData(35,7:9)*0.02))-dtR0


R0 = quatern2rotMat(ImuData(64,[6 3 4 5]));
R1 = quatern2rotMat(ImuData(68,[6 3 4 5]));
dtR0 = rodrigues(R1'*R0);
rodrigues(rodrigues(imuData(64,2:4)*0.02)*rodrigues(imuData(65,2:4)*0.02)*rodrigues(imuData(66,2:4)*0.02)*rodrigues(imuData(67,2:4)*0.02))-dtR0



nn =1;R0 = quatern2rotMat(imuSampFrame(nn,[6 3 4 5]));R1 = quatern2rotMat(imuSampFrame(nn+1,[6 3 4 5]));dtR0 = rodrigues(R1*R0');imuSampFrame(nn,7:9)*20/1000+dtR0'
nn =2;R0 = quatern2rotMat(imuSampFrame(nn,[6 3 4 5]));R1 = quatern2rotMat(imuSampFrame(nn+1,[6 3 4 5]));dtR0 = rodrigues(R1*R0');imuSampFrame(nn,7:9)*20/1000+dtR0'
nn =3;R0 = quatern2rotMat(imuSampFrame(nn,[6 3 4 5]));R1 = quatern2rotMat(imuSampFrame(nn+1,[6 3 4 5]));dtR0 = rodrigues(R1*R0');imuSampFrame(nn,7:9)*20/1000+dtR0'


reshape(PoseGyro(end,1:9),3,3)-RCur
rodrigues(reshape(PoseGyro(end,1:9),3,3))-rodrigues(RCur)


[rodrigues(rGyroTmp) rodrigues(deltR)]


[poseMat_, G, pose,vel1,dis1] = integrateImu(imuData(68:end,5:7), imuData(68:end,2:4), meanG', eye(3), eye(3), [0;0;0], [0;0;0],[0 0 0],[0 0 0]', 0.02,reshape(PoseGyro(startPoseId,1:9),3,3));
deltR - reshape(poseMat_(end,1:9),3,3)
imul.transVecI2W(end,:) - poseMat_(end,10:end)


R0 = quatern2rotMat(ImuData(1,[6 3 4 5]));
R1 = quatern2rotMat(ImuData(3,[6 3 4 5]));
dtR0 = rodrigues(R1'*R0);
[(rodrigues(imuData(1,2:4)*0.02)*rodrigues(imuData(2,2:4)*0.02)) R1'*R0]


data = debug_integration(ImuData, imuData, 18,22)
dltt = rodrigues(rGyroTmpPrv'*rGyroTmp)
data - [dltt dltt]


a1 = find(BsSampFrame(:,2) == EulerWheel(208,1))
a2 = find(ImuData(:,2) == EulerGyro(208,1))
rad2deg(rodrigues(quatern2rotMat(BsSampFrame(a1,[13 10:12]))))
rad2deg(rodrigues(quatern2rotMat(ImuData(a2,[6 3:5]))))

a1 = find(BsSampFrame(:,2) == EulerWheel(209,1));
a2 = find(ImuData(:,2) == EulerGyro(209,1));
rad2deg(rodrigues(quatern2rotMat(BsSampFrame(a1,[13 10:12]))))
rad2deg(rodrigues(quatern2rotMat(ImuData(a2,[6 3:5]))))

