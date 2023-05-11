if 0
    pt2dKey = Pix(8,:);
    
    localTrace = [LocalTrace.ptIcsX(8,:); LocalTrace.ptIcsY(8,:)]';
    
    goldenZ = LocalTrace.ptCcsZGT(8,1);
    stereoDisp = LocalTrace.dispList(8,1);
    gtangle = [obj.accumP2CRef(end-(keyFrameLen-1):end) - obj.accumP2CRef(end-(keyFrameLen-1))];
    goldenTheta = gtangle(2:end);
    
    goldenTheta(end) - k2cRef_00
    
    data.pt2dKey = pt2dKey;
    data.goldenZ = goldenZ;
    data.stereoDisp = stereoDisp;
    data.goldenTheta = goldenTheta;
    data.intrMat = intrMat;
    data.L2R = L2R;
    data.B2C = b2c;
    data.localTrace = localTrace;
else
    data = [];
    pt2dKey = Pix;
    
%     localTrace = [LocalTrace.ptIcsX(8,:); LocalTrace.ptIcsY(8,:)]';
    
    goldenZ = LocalTrace.ptCcsZGT;
    stereoDisp = LocalTrace.dispList;
    gtangle = [obj.accumP2CRef(end-(keyFrameLen-1):end) - obj.accumP2CRef(end-(keyFrameLen-1))];
    goldenTheta = gtangle(2:end);
    
    goldenTheta(end) - k2cRef_00
    
%     data.pt2dKey = pt2dKey;
    data.goldenZ = goldenZ;
    data.stereoDisp = stereoDisp;
    data.goldenTheta = goldenTheta;
    data.intrMat = intrMat;
    data.L2R = L2R;
    data.B2C = b2c;
    data.localTrace.ptIcsX = LocalTrace.ptIcsX;
    data.localTrace.ptIcsY = LocalTrace.ptIcsY;
    asgbkj = 1;
end