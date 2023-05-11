function SaveDumpData(obj, LocalTraceList)
global probPath


if length(dir(fullfile(probPath,'ReplayData_*.mat'))) == 0
    
    featPt = [LocalTraceList{1, 1}.featId  LocalTraceList{1, 1}.ptIcsX(:,1)  LocalTraceList{1, 1}.ptIcsY(:,1) LocalTraceList{1, 1}.ptIcsXOrig(:,1)  LocalTraceList{1, 1}.ptIcsYOrig(:,1)];
    data{1,1} = obj.prevImgL;data{1,2} = obj.prevImgR; data{1,3} = obj.prvDepthVisual;
    data{1,4} = featPt; data{1,5} = obj.prvDepthGT; data{1,6} = obj.accumP2CRef(1);
    
    data{1,7} = obj.imuSample;
    data{1,8} = obj.goldenPoseStackImu;
    data{1,9} = obj.goldenPoseStackWheel;
    
    save(fullfile(probPath,sprintf('ReplayData_%05d.mat',length(dir(fullfile(probPath,'ReplayData_*.mat')))+1)), 'data');
    
    data = {};
    featPt = [LocalTraceList{1, 1}.featId  LocalTraceList{1, 1}.ptIcsX(:,2)  LocalTraceList{1, 1}.ptIcsY(:,2) LocalTraceList{1, 1}.ptIcsXOrig(:,2)  LocalTraceList{1, 1}.ptIcsYOrig(:,2)];
    data{1,1} = obj.currImgL;data{1,2} = obj.currImgR; data{1,3} = obj.depthVisual;
    data{1,4} = featPt; data{1,5} = obj.depthGT; data{1,6} = obj.accumP2CRef(end);
    
    if 0
        data{1,7} = obj.imuSample;
        data{1,8} = obj.goldenPoseStackImu;
        data{1,9} = obj.goldenPoseStackWheel;
    end
    save(fullfile(probPath,sprintf('ReplayData_%05d.mat',length(dir(fullfile(probPath,'ReplayData_*.mat')))+1)), 'data');
    
    return;
end

dirInfo = dir(fullfile(probPath, 'ReplayData_*.mat'));
featPt = [];
for i = 1 : length(LocalTraceList)
    LocalTrace = LocalTraceList{i, 1};
    if size(LocalTrace.ptIcsX,2) > 2
        featPt = [featPt;  [LocalTrace.featId  LocalTrace.ptIcsX(:,end) LocalTrace.ptIcsY(:,end)  LocalTrace.ptIcsXOrig(:,end) LocalTrace.ptIcsYOrig(:,end)]];
        
        if i == length(LocalTraceList)
            load(fullfile(probPath, dirInfo(end).name));
            data{1,6} = obj.accumP2CRef(end-1);
            
            data{1,7} = obj.imuSample;
            data{1,8} = obj.goldenPoseStackImu;
            data{1,9} = obj.goldenPoseStackWheel;
            
            save(fullfile(probPath, dirInfo(end).name), 'data');
            
            
            data = {};
            data{1,1} = obj.currImgL;data{1,2} = obj.currImgR; data{1,3} = obj.depthVisual;
            data{1,4} = featPt; data{1,5} = obj.depthGT; data{1,6} = obj.accumP2CRef(end);
            save(fullfile(probPath,sprintf('ReplayData_%05d.mat',length(dir(fullfile(probPath,'ReplayData_*.mat')))+1)), 'data');
        end
        
        
    else
        load(fullfile(probPath, dirInfo(end).name))
        featPt_prv = data{1,4};
        featPt_prv = [featPt_prv; [LocalTrace.featId LocalTrace.ptIcsX(:,1) LocalTrace.ptIcsY(:,1) LocalTrace.ptIcsXOrig(:,1) LocalTrace.ptIcsYOrig(:,1)]];
        data{1,4} = featPt_prv;
        %         save(fullfile(probPath,sprintf('ReplayData_%05d.mat',length(dir(fullfile(probPath,'ReplayData_*.mat')))+0)), 'data');
        
        data{1,6} = obj.accumP2CRef(end-1);
        
        data{1,7} = obj.imuSample;
        data{1,8} = obj.goldenPoseStackImu;
        data{1,9} = obj.goldenPoseStackWheel;
        
        save(fullfile(probPath, dirInfo(end).name), 'data');
        
        data = {};
        
        featPt = [featPt;  [LocalTrace.featId  LocalTrace.ptIcsX(:,end) LocalTrace.ptIcsY(:,end) LocalTrace.ptIcsXOrig(:,end) LocalTrace.ptIcsYOrig(:,end)]];
        
        data{1,1} = obj.currImgL;data{1,2} = obj.currImgR; data{1,3} = obj.depthVisual;
        data{1,4} = featPt; data{1,5} = obj.depthGT; data{1,6} = obj.accumP2CRef(end);
        save(fullfile(probPath,sprintf('ReplayData_%05d.mat',length(dir(fullfile(probPath,'ReplayData_*.mat')))+1)), 'data');
    end
end



end