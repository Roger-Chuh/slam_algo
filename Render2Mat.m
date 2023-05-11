function Render2Mat(inputDir, bsl, vsl, varargin)


if (nargin <= 3)
    rosDump = false;
elseif (nargin == 4)
    rosDump = varargin{1};
else
    error('Too many input arguments');
end


load(fullfile(inputDir, 'dump.mat'))

if ~exist('b2c', 'var')
    b2c = eye(3);
end

delete(fullfile(inputDir,'Key*.mat'));

videoFPS = 10;
wheelFPS = 40;
imuFPS = wheelFPS;  40;
splitFrameNum = 10;


KeySeq.bsSampFrame = {};
KeySeq.imgHeader = {};
KeySeq.imgL = {};
KeySeq.imgR = {};
KeySeq.imuSampFrame = {};
KeySeq.goldenVal = {};
KeySeq.depthGT = {};
KeySeq.depthGTR = {};

imgSize = size(Depth{1,1});

angList;
videoTimeStamp = 1000.*([100 : 1/videoFPS: 1000]);
wheelTimeStamp = 1000.*([100 : 1/wheelFPS: 1000]);
imuTimeStamp = 1000.*([100 : 1/imuFPS: 1000]);

angList = angList - angList(1);

deltAng = deg2rad(mean(diff(angList)));
depthScale = 1;

for i = 1 : size(LR,1) - 1
    
    
    
%     if i > 1
%         vsl.prevRefRobotPoseWcs.transformMat = [rotMat [0 0 0]';0 0 0 1];
%         vsl.prevRefRobotRotAngWcs = deg2rad(angList(i-1));
%     end
    
    
    videoId = find(videoTimeStamp >= videoTimeStamp(i) & videoTimeStamp < videoTimeStamp(i+1));
    wheelId = find(wheelTimeStamp >= videoTimeStamp(i) & wheelTimeStamp < videoTimeStamp(i+1));
    imuId = find(imuTimeStamp >= videoTimeStamp(1) & imuTimeStamp < videoTimeStamp(2));
    
    
    
    
    wList = repmat(deltAng./length(wheelId).*wheelFPS,length(wheelId),1);
    
    
    quat = [];gyroVecList = [];
    for k = 1:length(imuId)
        rotMat = roty((angList(i+1) - angList(1))/length(imuId)*k);
        if k == 1
            gyroVec = rodrigues(rotMat)'.*imuFPS;
            gyroVec_ = [-gyroVec(1) -gyroVec(3) gyroVec(2)];
            gyroVecList = repmat(gyroVec_, length(imuId),1);
        end
        
        quat(k,:) = rotMat2quatern(rotMat);
    end
    
    bsSampFrame = [wheelId' wheelTimeStamp(wheelId)' wheelTimeStamp(wheelId)' zeros(length(wheelId),1) [wList] zeros(length(wheelId),1)];
    imuSampFrame = [imuId' imuTimeStamp(imuId)' quat(:,[2 3 4 1]) gyroVecList repmat([0 0 9.8],length(imuId),1)];
    
    goldenVal = [zeros(length(wheelId),3) quat(:,[2 3 4 1])];
    
    imgHeader = [i  videoTimeStamp(i) imgSize];
    imgL = LR{i,1};
    imgR = LR{i,2};
    depthGT = Depth{i,1};
    depthGTR = Depth{i,2};
    
    KeySeq.bsSampFrame = [KeySeq.bsSampFrame; bsSampFrame];
    KeySeq.imgHeader = [KeySeq.imgHeader; imgHeader];
    KeySeq.imgL = [KeySeq.imgL; imgL];
    KeySeq.imgR = [KeySeq.imgR; imgR];
    KeySeq.imuSampFrame = [KeySeq.imuSampFrame; imuSampFrame];
    KeySeq.goldenVal = [KeySeq.goldenVal; goldenVal];
    KeySeq.depthGT = [KeySeq.depthGT; depthGT];
    KeySeq.depthGTR = [KeySeq.depthGTR; depthGTR];
    depthGT = depthScale*depthGT;
    
    
    
    if 1 % i > 1
        vsl.prevRefRobotPoseWcs.transformMat = [rotMat [0 0 0]';0 0 0 1];
        vsl.prevRefRobotRotAngWcs = deg2rad(angList(i));
        
        
        vsl.currRefRobotPoseWcs.transformMat = [rotMat [0 0 0]';0 0 0 1];
        vsl.currRefRobotRotAngWcs = deg2rad(angList(i));
    end
    
    if mod(i,splitFrameNum) == 0
        
        vsl.featPtManager.localTrace.ptIcsX = [ones(1000,3) zeros(1000,1)];
        
        vsl.featPtManager.localTrace.ptIcsY = [ones(1000,3) zeros(1000,1)];
        
        vsl.featPtManager.localTrace.ptCcsZ = [ones(1000,1)];
        
        
        vsl.keyFrameFlagList(1) = true;
        
        save(fullfile(inputDir,sprintf('KeySeq_%05d.mat',length(dir(fullfile(inputDir,'KeySeq_*.mat')))+1)), 'KeySeq','wheelNew','gyroNew','bsl','vsl','wheelTimeAng','gyroTimeAng','visionTimeAng');
        KeySeq.bsSampFrame(1:end-1,:) = [];
        KeySeq.imgHeader(1:end-1,:) = [];
        KeySeq.imgL(1:end-1,:) = [];
        KeySeq.imgR(1:end-1,:) = [];
        KeySeq.imuSampFrame(1:end-1,:) = [];
        KeySeq.goldenVal(1:end-1,:) = [];
        KeySeq.depthGT(1:end-1,:) = [];
        KeySeq.depthGTR(1:end-1,:) = [];
        vsl.keyFrameFlagList(end) = true; 
        %         vsl.keyFrameDone = false;
    end
    wheelNew = angList(1:i)';
    gyroNew = angList(1:i)';
    wheelTimeAng = [videoTimeStamp(1:i)' wheelNew];
    gyroTimeAng = wheelTimeAng;
    visionTimeAng = wheelTimeAng;
    %         bsl = [];
    %         vsl = [];
    vsl.poseWcsList = [zeros(i,2) deg2rad(angList(1:i))'];
    bsl.poseWcsList = [zeros(i,2) deg2rad(angList(1:i))'];
    bsl.motionStateList = uint8(repmat(3, size(bsl.poseWcsList,1),1));
    vsl.motionStateList = uint8(repmat(3, size(bsl.poseWcsList,1),1));
    if i == 1
        vsl.keyFrameFlagList = true;
    else
        vsl.keyFrameFlagList = [vsl.keyFrameFlagList; false];
    end
    
    vsl.frmStampList = videoTimeStamp(1:i)';  % [vsl.frmStampList; curFrmTimeStamp];
    
    if 0
        InsertKeyFrame(obj, vsl.currImgL, depthMap);
        
        if i > 1
            [depthMap, dispMap] = GetDepthMap(vsl, vsl.prevImgL, vsl.prevImgR);
        end
    end
    
    
    vsl.prevImgL = imgL; % imresize(vsl.prevImgL, [size(vsl.depthGT) ]);
    vsl.prevImgR = imgR; % imresize(vsl.prevImgR, [size(vsl.depthGT) ]);
    
    
    

    
    vsl.currImgL = imgL; % imresize(vsl.currImgL, [size(vsl.depthGT) ]);
    
    vsl.currImgR = imgR; % imresize(vsl.currImgR, [size(vsl.depthGT) ]);
    vsl.keyFrameDepthGT = depthGT; % imresize(vsl.keyFrameDepthGT, [size(vsl.depthGT) ]);
    vsl.prvDepthGT = depthGT; % imresize(vsl.prvDepthGT, [size(vsl.depthGT) ]);
    

end





end