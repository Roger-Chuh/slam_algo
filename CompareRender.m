function CompareRender()
% imm = imread('E:\bk_20180627\SLAM\slam_algo\rosL.png');
load('E:\bk_20180627\SLAM\slam_algo\rosData.mat');
figure,imshow(imgL);
figure,imshow(depthGT, []);

obj_test = MR_obj_read('C:\Users\rongjiezhu\Documents\WXWork\1688852647983342\Cache\File\2020-05\÷Ï\01.obj');
temp = (rotx(90)*obj_test.V')';
temp = (roty(90)*temp')';
temp = (roty(90)*temp')';
temp = (roty(90)*temp')';
temp = temp.*3;
B2c = [eye(3) [10 45 -170]';0 0 0 1];
temp(:,2) = temp(:,2) - 30;
temp(:,2) = temp(:,2) + 60;
T0 = [eye(3) [0;0;0]; 0 0 0 1];
T1 = B2c*T0;
obj_test.V = temp;
depthThr = 0;
[obj_test2, inlierId] = procObj(obj_test, depthThr, T1(1:3,1:3), T1(1:3,4));
global textureFile
textureFile = 'C:\Users\rongjiezhu\Desktop\bg2.jpg';
% depthThr = 10;
[xyzOrig22, imgCur022, depthMap22, Vcam22] = render_fcn(obj_test2, T1(1:3,1:3), T1(1:3,4));figure,imshow(imgCur022);figure,imshow(depthMap22, []);
end