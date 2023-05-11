function [PoseWheel, EulerWheel] = CvtBsFrame2Pose(bsSampFrame,goldenVal )


    PoseWheel = []; EulerWheel = [];
    wheelData = [bsSampFrame(:,2) goldenVal(:,4:end)];
    
    for i = 1 : size(wheelData,1)
        RwheelCur = (roty(0)*rotx(0)*quatern2rotMat(wheelData(i,[5 2 3 4])));
        %     RwheelCur = [-RwheelCur(2,:); RwheelCur(3,:); -RwheelCur(1,:)];
        
        
        
        
        RwheelDelt = RwheelCur'; %(RwheelCur'*RwheelPrv)';%
        %     eulerWheel = rad2deg(rotMat2euler(RwheelDelt));
        
        eulerWheel = rad2deg(rodrigues(RwheelDelt)');
        %     eulerWheel = sign(eulerWheel(2))*eulerWheel;
        %     eulerWheel = ([-eulerWheel(:,2) -eulerWheel(:,3) eulerWheel(:,1)]);
        eulerWheel = ([-eulerWheel(:,2) -eulerWheel(:,3) eulerWheel(:,1)]);
        rWheelTmp = rodrigues(deg2rad(eulerWheel));
        PoseWheel = [PoseWheel; [rWheelTmp(:)' -goldenVal(i,2) -goldenVal(i,3) goldenVal(i,1)]];
        EulerWheel = [EulerWheel;[wheelData(i,1) eulerWheel sign(eulerWheel(2))*norm(rad2deg(rodrigues(RwheelDelt)'))]];
    end

   



end