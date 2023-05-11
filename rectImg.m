function rectImg(inputDir)

load(fullfile(inputDir, 'stereoParam.mat'));
IntrMatL = [stereoParam.focLeft(1) 0 stereoParam.cenLeft(1); 0 stereoParam.focLeft(2) stereoParam.cenLeft(2) ; 0 0 1];
IntrMatR = [stereoParam.focRight(1) 0 stereoParam.cenRight(1); 0 stereoParam.focRight(2) stereoParam.cenRight(2) ; 0 0 1];
kcL = stereoParam.kcLeft;
kcR = stereoParam.kcRight;

[rotMat1ToCP, rotMat2ToCP, projMat1, projMat2] = OcvStereoRectify(IntrMatL, kcL, IntrMatR, kcR, [480; 640], stereoParam.rotVecRef, stereoParam.transVecRef);


dirInfo = dir(fullfile(inputDir, '*.png'));
frameNum = length(dirInfo)/2;
for i = 1 : frameNum
   imgL = imread(fullfile(inputDir,dirInfo(i).name)); 
   imgR = imread(fullfile(inputDir,dirInfo(i+frameNum).name)); 
    
end


end

function rectifyImg(imgL)
SetPinholeCamParam(obj.monoCamModel1, [projMat1(1,1); projMat1(2,2)], projMat1(1:2,3), projMat1(1,2));
SetPinholeCamParam(obj.monoCamModel2, [projMat2(1,1); projMat2(2,2)], projMat2(1:2,3), projMat2(1,2));

[mapX1, mapY1] = OcvInitUndistortRectifyMap(Get(obj, 'IntrMat', 1), Get(obj, 'DistCoeff', 1), rotMat1ToCP, Get(obj, 'PinholeIntrMat', 1), obj.imgSize);
SetMap(obj.monoCamModel1, mapX1, mapY1);
[mapX2, mapY2] = OcvInitUndistortRectifyMap(Get(obj, 'IntrMat', 2), Get(obj, 'DistCoeff', 2), rotMat2ToCP, Get(obj, 'PinholeIntrMat', 2), obj.imgSize);
SetMap(obj.monoCamModel2, mapX2, mapY2);


end