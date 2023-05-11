function testQuanThoughts2()

% close all
% x = [-12,-18,-21];
% y = [-26,-61,-89];
x = [-18,-21];
x = x - 2;
y = [-61,-89];
xy = [x; y]';

x3_0 = -12;
y3_0 = -26;
x3 = -12 - 2;
y3 = -26 ;

ori_1 = [-0.25,-4];
ori_2 = [-0.37,-1.8];
ori_1_far_right = ori_1(1) + 3;
ori_2_far_right = ori_2(1) + 3;
% figure,plot(x,y,'s');hold on;plot(ori_1(:,1), ori_1(:,2),'*r');plot(ori_2(:,1), ori_2(:,2),'*b');

line1 = pt2line(xy(1,:),ori_1);
line2 = pt2line(xy(2,:),ori_1);
lineComm1 = [0 1 -(ori_1(2)-1)];
% lineComm2 = [0 1 -(ori_2(2)-1)];

pt1 = line2pt(line1, lineComm1);
pt2 = line2pt(line2, lineComm1);
ratio1 = norm(pt2(1) - ori_1_far_right)/norm(pt1(1) - ori_1_far_right);
radius = norm(ori_1 - [ori_1_far_right (ori_1(2)-1)]);

figure(1),plot(x,y,'s');hold on;plot(ori_1(:,1), ori_1(:,2),'*r');plot(ori_2(:,1), ori_2(:,2),'*b');
line([ori_1_far_right -20]', [ori_1(2)-1 ori_1(2)-1]');
line([xy(1,1) ori_1(1)]', [xy(1,2) ori_1(2)]');line([xy(2,1) ori_1(1)]', [xy(2,2) ori_1(2)]');
% line([ori_2_far_right -20]', [ori_2(2)-1 ori_2(2)-1]');
% plot(ori_2_far_right, ori_2(2)-1,'*k');
plot(ori_1_far_right, ori_1(2)-1,'.k');plot(pt1(1), pt1(2),'.k');plot(pt2(1), pt2(2),'.k');axis equal




ang = CalcDegree([0 -1 0], [[ori_2_far_right, ori_2(2)-1] - ori_2, 0]);


line3 = pt2line(xy(1,:),ori_2);
line4 = pt2line(xy(2,:),ori_2);

line5 = pt2line([x3 y3], ori_1);
line6 = pt2line([x3 y3], ori_2);
line5_0 = pt2line([x3_0 y3_0], ori_1);
line6_0 = pt2line([x3_0 y3_0], ori_2);

pt5 = line2pt(line5, lineComm1);
pt5_0 = line2pt(line5_0, lineComm1);

ratio2 = [norm(pt5(1) - ori_1_far_right)/norm(pt1(1) - ori_1_far_right) norm(pt5_0(1) - ori_1_far_right)/norm(pt1(1) - ori_1_far_right)];

xList = linspace(-20,5,10);
thetaList = -20:1:20;
ptIso = [];
for i = 1 : length(thetaList)
    thetaTemp = thetaList(i);
    ptIso(i,:) = ori_2 - [sind(thetaTemp) cosd(thetaTemp)];
    ptFar(i,:) = ori_2 - radius.*[sind(thetaTemp - ang) cosd(thetaTemp - ang)];
    err(i,1) = dot(ptFar(end,:) - ptIso(end,:), ori_2 - ptIso(end,:));
    lineComm2 = pt2line(ptFar(end,:), ptIso(end,:));
    pt3 = line2pt(line3, lineComm2);
    pt4 = line2pt(line4, lineComm2);
    
% % %     line5 = pt2line([x3 y3], ori_1);
% % %     line6 = pt2line([x3 y3], ori_2);
% % %     line5_0 = pt2line([x3_0 y3_0], ori_1);
% % %     line6_0 = pt2line([x3_0 y3_0], ori_2);
%     pt5 = line2pt(line5, lineComm1);
    pt6 = line2pt(line6, lineComm2);
%     pt5_0 = line2pt(line5_0, lineComm1);
    pt6_0 = line2pt(line6_0, lineComm2);
    
    
    yList1 = (-line5(1).*xList - line5(3))./line5(2);
    yList2 = (-line6(1).*xList - line6(3))./line6(2);
    yList1_0 = (-line5_0(1).*xList - line5_0(3))./line5_0(2);
    yList2_0 = (-line6_0(1).*xList - line6_0(3))./line6_0(2);
    if 0
        figure,plot(x,y,'s');hold on;plot(ori_1(:,1), ori_1(:,2),'*r');plot(ori_2(:,1), ori_2(:,2),'*b');
        axis equal
        line([xy(1,1) ori_2(1)]', [xy(1,2) ori_2(2)]');line([xy(2,1) ori_2(1)]', [xy(2,2) ori_2(2)]');
        yList = (-lineComm2(1).*xList - lineComm2(3))./lineComm2(2);
        plot(xList,yList,'-');
        line([ptIso(end,1) ptFar(end,1)]', [ptIso(end,2) ptFar(end,2)]');
        plot(ptFar(end,1), ptFar(end,2),'*k');plot(pt3(1),pt3(2),'.k');plot(pt4(1),pt4(2),'.k');
        plot(ptIso(end,1), ptIso(end,2),'*');
        
        plot(xList, yList1,'k');plot(xList, yList2,'k');plot(pt5(1), pt5(2),'*');plot(pt6(1), pt6(2),'*');
        plot(xList, yList1_0,'m');plot(xList, yList2_0,'m');plot(pt5_0(1), pt5_0(2),'*');plot(pt6_0(1), pt6_0(2),'*');
        plot(x3,y3,'*');plot(x3_0,y3_0,'*')
    end
    ratioList(i,1) =  norm(pt4 - ptFar(end,:))/norm(pt3 - ptFar(end,:));
    ratioList2(i,:) = [norm(pt6 - ptFar(end,:))/norm(pt3 - ptFar(end,:)) norm(pt6_0 - ptFar(end,:))/norm(pt3 - ptFar(end,:))];
end
figure(1),hold on;
plot(ptIso(:,1),ptIso(:,2),'-x');
plot(ptFar(:,1),ptFar(:,2),'-x');


% line5 = pt2line([x3 y3], ori_1);
% line6 = pt2line([x3 y3], ori_2);
% line5_0 = pt2line([x3_0 y3_0], ori_1);
% line6_0 = pt2line([x3_0 y3_0], ori_2);
% pt5 = line2pt(line5, lineComm1);
% pt6 = line2pt(line6, lineComm2);
% pt5_0 = line2pt(line5_0, lineComm1);
% pt6_0 = line2pt(line6_0, lineComm2);
% 
% 
% yList1 = (-line5(1).*xList - line5(3))./line5(2);
% yList2 = (-line6(1).*xList - line6(3))./line6(2);
% yList1_0 = (-line5_0(1).*xList - line5_0(3))./line5_0(2);
% yList2_0 = (-line6_0(1).*xList - line6_0(3))./line6_0(2);




figure,plot(abs(ratio1 - ratioList));
hold on;plot(abs([ratio2 - ratioList2]))

end

function line = pt2line(pt1,pt2)

line = cross([pt1 1], [pt2 1]);
line = line./norm(line(1:2));
end
function pt = line2pt(line1, line2)
pt = cross(line1, line2);
pt = pt./pt(3);
pt = pt(1:2);
end