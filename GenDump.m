function GenDump(folderName,keySeqNum, varargin)

if (nargin <= 2)
    baseDir = pwd;
elseif (nargin == 3)
    baseDir = varargin{1};
else
    error('Too many input arguments');
end

dumpDir = baseDir;

replayDir = fullfile(baseDir, folderName);

delete(fullfile(pwd,'\temp\*.png'));
delete(fullfile(pwd,'\dispErr\*.png'));
delete(fullfile(pwd,'\prob\*.png'));


delete(fullfile(pwd,'\prob\*.fig'));
delete(fullfile(pwd,'\dump\*.png')); delete(fullfile(pwd,'\dump\*.mat'));
if 1 % isempty(id_)
    delete(strcat(fullfile(baseDir,folderName),'\TraceData*'));
else
    delete(strcat(folderName,'\TraceData*'));
end
replayDirInfo = dir(fullfile(replayDir,'KeySeq*'));

global ReplaceGolden probPath OUTPUT_ROOT RECORDED_BIN_ROOT DUMP_ROOT

ReplaceGolden = false; true;

% SetupRobotEnv('simulation_short_baseLine2');


assert(~isempty(OUTPUT_ROOT) && ~isempty(RECORDED_BIN_ROOT) && ~isempty(DUMP_ROOT), 'The directories are not correctly set');
csaParamFilePath = fullfile(DUMP_ROOT, 'coordsys_alignment.txt');
csa = CoordSysAligner;
% % Load(csa, csaParamFilePath);
robotCfgFilePath = fullfile(OUTPUT_ROOT, 'robot_config.prototxt');
robotConfig = ReadRobotConfig(robotCfgFilePath);

camParamFilePath = fullfile(DUMP_ROOT, 'camera_param.txt');
cam = BCStereoCameraModel(camParamFilePath);
CvtToRectifiedModel(cam);
vsl = VisualLocalizer(cam, csa, robotConfig.tracking_config);
if isempty(vsl.scaleLvl)
    vsl.scaleLvl = 0;
end

t0 = datetime('now','Format','dd-MMM-y HH:mm:ss');t = datestr(t0);t1 = yyyymmdd(t0);t2 = strsplit(t,' ');
tt = strcat(num2str(t1),'_',t2{2});tt(find(tt == ':')) = '';



if keySeqNum == 1
    needReplace = 0;
else
    needReplace = 1;
end

if keySeqNum == 1
    probPath = fullfile(dumpDir, 'prob',strcat(tt, '====', folderName,  '====',num2str(0),'_',num2str(0),'_',num2str(0),'_',num2str(0),'____',tt));
    MakeDirIfMissing(probPath);
end

angRef = {}; AngRef = 0;
poseGT = [];
for i = keySeqNum : length(replayDirInfo)  -1

    curKeySeqData = load(fullfile(replayDir,replayDirInfo(i).name));
    vsl = curKeySeqData.vsl;
   
    for j = 1 : size(curKeySeqData.KeySeq.imgHeader,1)-1
        if needReplace
            if keySeqNum > 1 && j ==1
                
                endKeySeqData = load(fullfile(replayDir,replayDirInfo(end).name));
                
                gtPose = endKeySeqData.goldenWheelPose;
                
                [vPoseNew, tsNew, ingIndNew] = InterpPose(gtPose(:,2), gtPose(:,4:end), gtPose(:,1),[], gtPose(:,3));
                vPoseNew(:,10:12) = 1000.*vPoseNew(:,10:12);
                
                angWhole = endKeySeqData.gyroNew;
                angWhole0 = angWhole;
                if ReplaceGolden
                    shiftVec = deg2rad([0:-0.1:-1000]');
                    shiftVecErr = 0.0025.*deg2rad((rand(length(shiftVec), 1) - 0.5));
                    shiftVecComp = rad2deg(shiftVec + shiftVecErr);
                    shiftVecComp(1) = 0;
                    angWhole = angWhole + shiftVecComp(1:length(angWhole));
                end
                motionStateWhole = endKeySeqData.bsl.motionStateList;
                prvKeySeqData = load(fullfile(replayDir,replayDirInfo(i-1).name));
                bsl = prvKeySeqData.bsl;
                vsl = prvKeySeqData.vsl;
                probPath = fullfile(dumpDir, 'prob',strcat(tt, '====', folderName,  '====',num2str(0),'_',num2str(0),'_',num2str(0),'_',num2str(0),'____',tt));
                MakeDirIfMissing(probPath);
                needReplace = 0;
            end
        end
        
        imgL = curKeySeqData.KeySeq.imgL{j,1};
        imgR = curKeySeqData.KeySeq.imgR{j,1};
        
        imgL = Remap(vsl.camModel.monoCamModel1, imgL);
        imgR = Remap(vsl.camModel.monoCamModel2, imgR);
        
        depthVisualCur = GetDepthMap(vsl, imgL, imgR);
        if ~ReplaceGolden
            depthGT = curKeySeqData.KeySeq.depthGT{j,1};
            depthGTR = curKeySeqData.KeySeq.depthGTR{j,1};
        else
            depthGT = depthVisualCur;
            depthGT(depthGT == -1) = nan;
            depthGTR = depthGT;
        end
        
        
        if ~exist('grPose_prv','var')
            
            
            idGT = find(round(ingIndNew) == round(curKeySeqData.KeySeq.imgHeader{j}(1)));
            grPose_prv = [reshape(vPoseNew(idGT-1, 1:9),3,3) vPoseNew(idGT-1, 10:12)';0 0 0 1];
            
            %             idGT = find(round(ingIndNew) == round(curKeySeqData.KeySeq.imgHeader{j}(1)));
            grPose = [reshape(vPoseNew(idGT, 1:9),3,3) vPoseNew(idGT, 10:12)';0 0 0 1];
            Tinit = inv(grPose_prv);
            grPose_ = Tinit*grPose;
            
            
            
            imwrite(vsl.prevImgL, fullfile(probPath, sprintf('imgL_%04d.png',ingIndNew(idGT-1))));
            imwrite(vsl.prevImgR, fullfile(probPath, sprintf('imgR_%04d.png',ingIndNew(idGT-1))));
            imwrite(uint16(vsl.prvDepthGT),fullfile(probPath, sprintf('depth_%04d.png',ingIndNew(idGT-1))),'png','bitdepth',16);
            
            
            
            imwrite(imgL, fullfile(probPath, sprintf('imgL_%04d.png',curKeySeqData.KeySeq.imgHeader{j}(1))));
            imwrite(imgR, fullfile(probPath, sprintf('imgR_%04d.png',curKeySeqData.KeySeq.imgHeader{j}(1))));
            imwrite(uint16(depthGT),fullfile(probPath, sprintf('depth_%04d.png',curKeySeqData.KeySeq.imgHeader{j}(1))),'png','bitdepth',16);
            
            
            
        else
            %             Tinit = inv(Tt1);
            idGT = find(round(ingIndNew) == round(curKeySeqData.KeySeq.imgHeader{j}(1)));
            grPose = [reshape(vPoseNew(idGT, 1:9),3,3) vPoseNew(idGT, 10:12)';0 0 0 1];
            grPose_ = Tinit*grPose;
            
            imwrite(imgL, fullfile(probPath, sprintf('imgL_%04d.png',curKeySeqData.KeySeq.imgHeader{j}(1))));
            imwrite(imgR, fullfile(probPath, sprintf('imgR_%04d.png',curKeySeqData.KeySeq.imgHeader{j}(1))));
            imwrite(uint16(depthGT),fullfile(probPath, sprintf('depth_%04d.png',curKeySeqData.KeySeq.imgHeader{j}(1))),'png','bitdepth',16);
            
            
        end
            
%         idGT = find(round(ingIndNew) == round(curKeySeqData.KeySeq.imgHeader{j}(1)));
%         grPose = [reshape(vPoseNew(idGT, 1:9),3,3) vPoseNew(idGT, 10:12)';0 0 0 1];
        
% %         imwrite(imgL, fullfile(probPath, sprintf('imgL_%04d.png',curKeySeqData.KeySeq.imgHeader{j}(1))));
% %         imwrite(imgR, fullfile(probPath, sprintf('imgR_%04d.png',curKeySeqData.KeySeq.imgHeader{j}(1))));
% %         imwrite(uint16(depthGT),fullfile(probPath, sprintf('depth_%04d.png',curKeySeqData.KeySeq.imgHeader{j}(1))),'png','bitdepth',16);
        
        
        
        bsl.poseWcsList = [bsl.poseWcsList;0 0 1];
        bsl.poseWcsList(:,3) = deg2rad(angWhole(1:size(bsl.poseWcsList,1)));
        
        
        angref = deg2rad(angWhole0(1:size(bsl.poseWcsList,1)));
        angRef = [angRef; {angref}];
        
        
        
        if ~exist(fullfile(probPath,'lkConfig.txt'))
            angStartId = size(bsl.poseWcsList,1) - 1;
            
            fid1 = fopen(fullfile(probPath, 'lkConfig.txt'),'w');
            fprintf(fid1, 'ShiftInterval: %d', 1);
            fclose(fid1);
            
            try
                skvb = 1;
                save(fullfile(probPath,'angWhole0.mat'), 'angWhole0');
            catch
                sgkh = 1;
            end
            
        end
        %         AngRef = angref(angStartId:end) - deg2rad(angWhole0(angStartId));
        AngRef = angref(angStartId:end) - (angref(angStartId));
        
        
        if length(dir(fullfile(probPath,'ReplayData_*.mat'))) == 0
            depthVisualPrv = GetDepthMap(vsl, vsl.prevImgL, vsl.prevImgR);
            data = {};
            data{1,1} = vsl.prevImgL;data{1,2} = vsl.prevImgR; data{1,3} = depthVisualPrv;
            data{1,4} = []; data{1,5} = vsl.prvDepthGT; data{1,6} = AngRef(1);
            
            data{1,7} = [reshape(eye(3),1,9) [0 0 0]];
            poseGT = [poseGT; [ingIndNew(idGT-1) data{1,7}]];
            
            save(fullfile(probPath,sprintf('ReplayData_%05d.mat',length(dir(fullfile(probPath,'ReplayData_*.mat')))+1)), 'data');
            data = {};
            data{1,1} = imgL;data{1,2} = imgR; data{1,3} = depthVisualCur;
            data{1,4} = []; data{1,5} = depthGT; data{1,6} = AngRef(2);
            
            data{1,7} = [reshape(grPose_(1:3,1:3),1,9) grPose_(1:3,4)'];
            poseGT = [poseGT; [curKeySeqData.KeySeq.imgHeader{j}(1) data{1,7}]];
            
            save(fullfile(probPath,sprintf('ReplayData_%05d.mat',length(dir(fullfile(probPath,'ReplayData_*.mat')))+1)), 'data');
        else
            data = {};
            data{1,1} = imgL;data{1,2} = imgR; data{1,3} = depthVisualCur;
            data{1,4} = []; data{1,5} = depthGT; data{1,6} = AngRef(end);
            
            data{1,7} = [reshape(grPose_(1:3,1:3),1,9) grPose_(1:3,4)'];
            poseGT = [poseGT; [curKeySeqData.KeySeq.imgHeader{j}(1) data{1,7}]];
            
            save(fullfile(probPath,sprintf('ReplayData_%05d.mat',length(dir(fullfile(probPath,'ReplayData_*.mat')))+1)), 'data');
        end
            

        
    

   
        
        
%         save(fullfile(probPath,'angGT.mat'), 'angRef', 'AngRef');
        
        
    end
    
end




fid1 = fopen(fullfile(probPath, 'pose.txt'),'w');


for kk = 1 : size(poseGT,1)
    for jj = 1 : size(poseGT,2)
        if jj == 1
            fprintf(fid1, '%d ', poseGT(kk,jj));
        else
            fprintf(fid1, '%06f  ', poseGT(kk,jj));
        end
    end
    fprintf(fid1, '\n');
end
fclose(fid1);


save(fullfile(probPath,'angGT.mat'), 'angRef', 'AngRef','vsl');

end