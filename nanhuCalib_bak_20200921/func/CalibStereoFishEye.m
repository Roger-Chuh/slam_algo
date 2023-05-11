function CalibStereoFishEye(inputDir,paraDir)
workDir = pwd;
% inputDirL = fullfile(workDir,'Left');
% inputDirR = fullfile(workDir,'Right');
inputDirL = fullfile(inputDir,'Left');
inputDirR = fullfile(inputDir,'Right');
if ~isdir(inputDirL)
    MakeDirIfMissing(inputDirL);
    MakeDirIfMissing(inputDirR);
    copyfile(fullfile(inputDir,'imgL*'), inputDirL);
    copyfile(fullfile(inputDir,'imgR*'), inputDirR);
    changeExtensionByEval(inputDirL);
    changeExtensionByEval(inputDirR)
end
if 0
    delete(strcat(inputDirL,'\*'));
    delete(strcat(inputDirR,'\*'));
end
[camParamL, cbcXYL, cbGridL, configL, undistImgL, cbcXYTmpL, cbGridTmpL, goodIdL] = CalibFishEye(inputDirL);
[camParamR, cbcXYR, cbGridR, configR, undistImgR, cbcXYTmpR, cbGridTmpR, goodIdR] = CalibFishEye(inputDirR);
load(fullfile(inputDirL,'oCamModelL.mat'));
load(fullfile(inputDirR,'oCamModelR.mat'));
for i = 1 : length(cbcXYL)
    
    
end

goodId = intersect(goodIdL,goodIdR);
camParamL.rotVec = camParamL.rotVec(:,ismember(goodIdL,goodId));
camParamL.rotVecErr = camParamL.rotVecErr(:,ismember(goodIdL,goodId));
camParamL.tranVec = camParamL.tranVec(:,ismember(goodIdL,goodId));
camParamL.tranVecErr = camParamL.tranVecErr(:,ismember(goodIdL,goodId));
camParamR.rotVec = camParamR.rotVec(:,ismember(goodIdR,goodId));
camParamR.rotVecErr = camParamR.rotVecErr(:,ismember(goodIdR,goodId));
camParamR.tranVec = camParamR.tranVec(:,ismember(goodIdR,goodId));
camParamR.tranVecErr = camParamR.tranVecErr(:,ismember(goodIdR,goodId));
stereoParam = CbCalibStereo2(camParamL, camParamR, cbcXYL(ismember(goodIdL,goodId)), cbcXYR(ismember(goodIdR,goodId)), cbGridL(ismember(goodIdL,goodId)), configL, configR);


% stereoParam = CbCalibStereo2(camParamL, camParamR, cbcXYL, cbcXYR, cbGridL, configL, configR);
[rectImgL, rectImgR] = RectifyImagePair(stereoParam, undistImgL{1}, undistImgR{1});
[rectParamL, rectParamR, rotMatLeft, rotMatRight] = GetRectifyParam2(stereoParam, size(undistImgR{1}));
baseline = norm(stereoParam.transVecRef);
[nr,nc,~] = size(undistImgR{1});

KL = [stereoParam.focLeft(1) 0 stereoParam.cenLeft(1);0 stereoParam.focLeft(2) stereoParam.cenLeft(2);0 0 1];
KR = [stereoParam.focRight(1) 0 stereoParam.cenRight(1);0 stereoParam.focRight(2) stereoParam.cenRight(2);0 0 1];
cbcXYLL = cbcXYL(ismember(goodIdL,goodId));
cbcXYRR = cbcXYR(ismember(goodIdR,goodId));
try
    for i = 1 : length(cbcXYLL) %length(cbcXYL)
        [xr] = normalize_pixel(cbcXYRR{(i)},stereoParam.focRight,stereoParam.cenRight,stereoParam.kcRight,stereoParam.alphaRight);
        xrr = pflat((KR)*pextend(xr));
        rt = [rodrigues(camParamL.rotVec(:,(i))) camParamL.tranVec(:,(i));0 0 0 1];
        cbPt = rt(1:3,1:3)*pextend(cbGridR{1}) + repmat(rt(1:3,4),1,126);
        ptIcs = TransformAndProject(cbPt', KR, rodrigues(stereoParam.rotVecRef), stereoParam.transVecRef);
        figure,imshow(zeros(nr,nc));hold on;plot(xrr(1,:),xrr(2,:),'or');plot(ptIcs(:,1),ptIcs(:,2),'.g')
        err(i,1) = norm(mean(abs(xrr(1:2,:)'-ptIcs)));
        
    end
catch
    askjlh = 1;
end
load(fullfile(inputDirL,'oCamModelL.mat'));
load(fullfile(inputDirR,'oCamModelR.mat'));
paraDir = inputDir;
save(fullfile(paraDir,'calib.mat'),'oCamModelL','oCamModelR','stereoParam','camParamL', 'cbcXYL', 'cbGridL', 'configL', 'cbcXYTmpL', 'cbGridTmpL', 'goodIdL','camParamR', 'cbcXYR', 'cbGridR', 'configR', 'cbcXYTmpR', 'cbGridTmpR', 'goodIdR');



end