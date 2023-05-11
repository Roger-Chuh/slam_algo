inputDir = 'E:\bk_20180627\SLAM\slam_algo\prob\1.5_20_1.5_20____20190925_161119';
inputDir = 'E:\bk_20180627\SLAM\slam_algo\prob\1.5_20_1.5_20____20190925_173936';
inputDir = 'E:\bk_20180627\SLAM\slam_algo\prob\1.5_20_1.5_20____20190925_180851';
inputDir = 'E:\bk_20180627\SLAM\slam_algo\prob\2.5_20_2.5_20____20190925_195452';
inputDir = 'E:\bk_20180627\SLAM\slam_algo\prob\2.5_20_2.5_20____20190925_200818';
inputDir = 'E:\bk_20180627\SLAM\slam_algo\prob\1.5_10_1.5_10____20190925_202209';
dataInfo = dir(fullfile(inputDir, 'traceLogData_*.mat'));
error1 = [];
for qaz =1 : length(dataInfo)
    data1 = load(fullfile(inputDir, dataInfo(qaz).name));
    error1(:,qaz) = data1.checkIdPtGT(:,1) - data1.checkIdPtGT(:,3);
    
end

% figure,subplot(2,1,1);plot(error1(1:size(error1,1)/2,:)');title('bad');subplot(2,1,2);plot(error1(size(error1,1)/2+1:end,:)');title('good');