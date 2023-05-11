pin = [
   -1.2300    2.0780;
   -1.0820    1.3510;
    0.4392    1.8500;
    0.0449    3.0610;
];


pout =[
   493   108;
   495   503;
   871   373;
   766    76;];

 H = homography_solve(pin', pout');
 
 
Temp = H*HomoCoord(obstacle',1);

P = [Temp(1,:)./Temp(3,:);Temp(2,:)./Temp(3,:)]';

Temp1 = H*HomoCoord(trace',1);
P1 = [Temp1(1,:)./Temp1(3,:);Temp1(2,:)./Temp1(3,:)]';


figure(3), imshow(img);hold on;

plot(P(:,1),P(:,2),'.b'); hold on;
plot(P1(:,1),P1(:,2),'.b'); hold on;

img = imread('background.png');
% large = imresize(img,4);
figure,imshow(img)
[centers, radii, metric] = imfindcircles(large,[1,20])
viscircles(centers, radii,'EdgeColor','b');


pin = [-0.377722, 1.57466;
    -0.174838, 1.63125;
    -0.009394, 1.71314;
    0.028986, 2.13322;
    0.242438, 2.22612;
    0.153277, 2.44149;
    0.108907, 2.66518;
    -0.034394, 3.03727;
    -0.313873, 3.0142;
    -0.185391, 2.8358;
    -0.170463, 2.58882;
    -1.09516, 1.72068;
    -1.00963, 1.45735;
    0.139883, 1.8013;
    0.009047, 2.82749;
    -0.10639, 2.37783;
    -0.540153, 2.96338;
    -0.389064, 2.36366;
    -0.429432, 2.72139;
    -0.668181, 2.21216;
    -0.909797, 2.13867;
    -1.12837, 2.0737;
    -0.576065, 1.50851;
    -0.767945, 1.50354;
    0.341947, 1.90568;
    -0.838754, 1.74932;
    -0.581805, 1.82463;
    -0.424927, 1.88292;
    -0.219949, 1.96261;
]

figure, plot(centers(:,1),centers(:,2),'o')
figure, plot(pin(:,1),pin(:,2),'o')
centers

r1 = [];
r2 = [];
r3 = [];
for i = 1:Rows(goldenData)
[r1(i),r2(i),r3(i)]=quat2angle([goldenData(i,4) goldenData(i,5) goldenData(i,6) goldenData(i,7)],'ZYX');
end
r3 = rad2deg(r3);
figure(876), plot(-goldenData(:,2),goldenData(:,1),'-g');hold on; plot(vsl.poseWcsList(:,1),vsl.poseWcsList(:,2),'-b'); plot(bsl.poseWcsList(:,1),bsl.poseWcsList(:,2),'-r'),axis equal
figure(877), plot(bslTimeStamp,r3,'-g');hold on; plot(vslTimeStamp,rad2deg(bsl.poseWcsList(:,3)),'-r');plot(vslTimeStamp,rad2deg(vsl.poseWcsList(:,3)),'-b')

img = imread('test.png');

[rows, cols, chennels] = size(img);

img1 = img;
img1(1:round(rows/2),1:end,1:3) = 0;
% figure,imshow(img1)


img2 = img;
img2(round(rows/2)+1:end,1:end,1:3) = 0;
% figure,imshow(img2)

[pt2,~] = detectCheckerboardPoints(img2);
[pt1,~] = detectCheckerboardPoints(img1);

figure,imshow(img)
hold on
plot(pt2(:,1),pt2(:,2),'rx')
plot(pt1(:,1),pt1(:,2),'bx')

figure(999),imshow(db.background)
cx1 = -1.795;
cy1 = - 6.085;
L1 = 0.02*30;
pin1 = debug.getCheckerboardPoints(cx1, cy1, L1);
cx2 = 6.846;
cy2 = 3.305;
L2 = 0.02*15;
pin2 = debug.getCheckerboardPoints(cx2, cy2, L2);
pin = [pin1;pin2];
Temp = db.H*HomoCoord(pin',1);
P = [Temp(1,:)./Temp(3,:);Temp(2,:)./Temp(3,:)]';
hold on
plot(P(:,1),P(:,2),'*')