function CvtRos2Mat(inputDir, replayDir)


MakeDirIfMissing(fullfile(pwd, replayDir));
delete(fullfile(pwd,strcat(replayDir,'\*.mat')));

topics ={'/combinationInfo'};
dirInfo = dir(fullfile(inputDir, '*.bag'));
bag = ros.Bag.load(fullfile(inputDir, dirInfo(1).name));
[msgs, meta] = bag.readAll(topics);



KeySeq.bsSampFrame = {};
KeySeq.imgHeader = {};
KeySeq.imgL = {};
KeySeq.imgR = {};
KeySeq.imuSampFrame = {};
KeySeq.goldenVal = {};
KeySeq.depthGT = {};
KeySeq.depthGTR = {};


for i = 3 : length(msgs)
[bsSampFrame, imgHeader, imgL,imgR, imuSampFrame, goldenVal, depthGT, depthGTR] = SubscribeDataStandAlone(msgs{1,i});



  KeySeq.bsSampFrame = [KeySeq.bsSampFrame; bsSampFrame];
    KeySeq.imgHeader = [KeySeq.imgHeader; imgHeader];
    KeySeq.imgL = [KeySeq.imgL; imgL];
    KeySeq.imgR = [KeySeq.imgR; imgR];
    KeySeq.imuSampFrame = [KeySeq.imuSampFrame; imuSampFrame];
    KeySeq.goldenVal = [KeySeq.goldenVal; goldenVal];
    KeySeq.depthGT = [KeySeq.depthGT; depthGT];
    KeySeq.depthGTR = [KeySeq.depthGTR; depthGTR];

end

% save(fullfile(replayDir,sprintf('KeySeq_%05d.mat',length(dir(fullfile(replayDir,'KeySeq_*.mat')))+1)), 'KeySeq','wheelNew','gyroNew','bsl','vsl','wheelTimeAng','gyroTimeAng','visionTimeAng');
save(fullfile(replayDir,sprintf('KeySeq_%05d.mat',length(dir(fullfile(replayDir,'KeySeq_*.mat')))+1)), 'KeySeq');

        
end