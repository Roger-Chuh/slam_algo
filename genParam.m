load('\\192.168.50.172\nextvpu\3.CTO°ì¹«ÊÒ\Roger\Roger_for_zhixuan\20191206_20191018_1842_5000_large_room\TraceData_00001.mat')
b2cVec = [rodrigues(b2c(1:3,1:3)); b2c(1:3,4)];
fid1 = fopen(fullfile('E:\bk_20180627\SLAM\slam_algo\param','param.txt'),'w');%????
fprintf(fid1,'%d %d %d',320, 240, 0);
fprintf(fid1, '\n');
fprintf(fid1,'%06f %06f %06f %06f %06f %06f %06f %06f %06f',intrMat(1,1),intrMat(2,2),intrMat(1,3),intrMat(2,3),0,0,0,0,0);
fprintf(fid1, '\n');
fprintf(fid1,'%06f %06f %06f %06f %06f %06f %06f %06f %06f',intrMat(1,1),intrMat(2,2),intrMat(1,3),intrMat(2,3),0,0,0,0,0);
fprintf(fid1, '\n');
fprintf(fid1,'%06f %06f %06f %06f %06f %06f',0,0,0,-20,0,0);
fprintf(fid1, '\n');
fprintf(fid1,'%06f %06f %06f %06f %06f %06f',b2cVec);
fprintf(fid1, '\n');
fclose(fid1);

% fid1 = fopen(fullfile('E:\bk_20180627\SLAM\slam_algo\param','b2c_param.txt'),'w');%????
% 
% fprintf(fid1,'%06f %06f %06f %06f %06f %06f',b2cVec);
% fprintf(fid1, '\n');
% fclose(fid1);