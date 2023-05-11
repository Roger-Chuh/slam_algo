function testQuanThoughts6()
global colorMat

load('E:\bk_20180627\SLAM\slam_algo\lineData.mat');
outDir = 'E:\bk_20180627\SLAM\slam_algo\quanOut';

delete(fullfile(outDir,'region*.png'));

close all

coeffStack = [cell2mat(lineCoeff1); cell2mat(lineCoeff2)];

xList = linspace(-50,18,1000);
xMat = repmat(xList, size(coeffStack,1),1);
yMat = (-repmat(coeffStack(:,1),1,length(xList)).*xMat - repmat(coeffStack(:,3),1,length(xList)))./repmat(coeffStack(:,2),1,length(xList));


% figure,plot(xMat', yMat','r');hold on;
uiopen('C:\Users\rongjiezhu\Desktop\d.fig',1)
if 1
    pt = [7.369762798991576 , 61.086179402189146;
        7.571427801248781 ,  90.86268516628888;
        7.547508926637793 ,  122.23255487333536;
        -21.567079933926326 ,  158.34426866217856;
        -21.83000067983201 , 126.33350422237083;
        -21.846977612124036 ,  95.05833252356226];
    
    
    pt = pt([6 5 4 1 2 3],:);
    
else
    pt = [-23.5 96; -22.9 126.8;-21.4 152.8;7 57; 7.4 86.4;7.9 121];
    
end

plot(pt(:,1),pt(:,2),'-*g');


ptX = repmat(pt(:,1),1, size(coeffStack,1))';
ptY = repmat(pt(:,2),1, size(coeffStack,1))';

ptXlist = ptX(:);
ptYlist = ptY(:);

coeffStack1 = repmat(coeffStack, size(pt,1),1);

thetaList = [-1:0.02:1];

thetaList = 0.24 + linspace(-0.04,0.04,10);
colorMat = [1 0 0;0 1 0;0 0 1;1 0 1; 0 1 1; 1 1 0];

for i = 1 : size(colorMat,1)
    plot(pt(i,1), pt(i,2), '*', 'Color', colorMat(i,:),'MarkerSize', 5, 'LineWidth',5);
    
end
% scaleList = 1.016 + linspace(-0.008,0.008,50);
scaleList = 1.016 + linspace(-0.02,0.02,10);
for j = 1 : length(thetaList)
    theta = thetaList(j);
    
    for jj = 1 : length(scaleList)
        scale = scaleList(jj);
        figure(4), clf;hold on;
        for i = 1 : size(lineCoeff1,1)/2
            
            tempLine = cell2mat(lineCoeff1(2*i-1:2*i,1));
            xList = linspace(-50,18,1000);
            xMat = repmat(xList, size(tempLine,1),1);
            yMat = (-repmat(tempLine(:,1),1,length(xList)).*xMat - repmat(tempLine(:,3),1,length(xList)))./repmat(tempLine(:,2),1,length(xList));
            %         figure,plot(xMat', yMat','r');hold on;plot(pt(i,1),pt(i,2),'*g');
            
            Pt = pt(i,:);
            testPt = [-23.5 96.23 1];
            %     Pt = testPt(1:2);
            
            
            coeffStackNew = [tempLine(:,1) tempLine(:,2) (tempLine(:,3) + (tempLine(:,1).*cosd(theta).*scale - tempLine(:,2).*sind(theta).*scale).*Pt(1) + (tempLine(:,1).*sind(theta).*scale + tempLine(:,2).*cosd(theta).*scale).*Pt(2))];
            plotLines(coeffStackNew, Pt, i); drawnow;
            
            
            
        end
        
        
        for i = 1 : size(lineCoeff2,1)/2
            
            tempLine = cell2mat(lineCoeff2(2*i-1:2*i,1));
            xList = linspace(-50,18,1000);
            xMat = repmat(xList, size(tempLine,1),1);
            yMat = (-repmat(tempLine(:,1),1,length(xList)).*xMat - repmat(tempLine(:,3),1,length(xList)))./repmat(tempLine(:,2),1,length(xList));
            %         figure,plot(xMat', yMat','r');hold on;plot(pt(3+i,1),pt(3+i,2),'*g');
            Pt = pt(3+i,:);
            
            
            
            coeffStackNew = [tempLine(:,1) tempLine(:,2) (tempLine(:,3) + (tempLine(:,1).*cosd(theta).*scale - tempLine(:,2).*sind(theta).*scale).*Pt(1) + (tempLine(:,1).*sind(theta).*scale + tempLine(:,2).*cosd(theta).*scale).*Pt(2))];
            plotLines(coeffStackNew, Pt, i + 3); drawnow;
            
        end
        title(sprintf('%0.6f degree\n%0.6f scale', theta, scale));
        saveas(gcf,fullfile(outDir,sprintf('region_%05d.png',length(dir(fullfile(outDir,'region_*.png')))+1)));
    end
        
end

test = [-0.7101 -4.634; -0.6701 -4.558; -0.6901 -4.831]

test = [-0.5791 -4.67; -0.5731 -4.634; -0.5791 -4.681]
plot(test(:,1),test(:,2))
r = scale.*[cosd(theta) sind(theta);-sind(theta) cosd(theta)]
ptNew = (r*pt' + repmat(mean(test)',1 ,6))';
figure(1),hold on;plot(ptNew(:,1), ptNew(:,2),'*k')

askg = 1;

if 0
    for i = 1 : length(thetaList)
        theta = thetaList(i);
        
        coeffStackNew = [coeffStack1(:,1) coeffStack1(:,2) coeffStack1(:,3)];
        
    end
end


end
function plotLines(tempLine, pt, ind)
global colorMat
% tempLine = cell2mat(lineCoeff1(2*i-1:2*i,1));
xList = linspace(-5,5,5000);
xMat = repmat(xList, size(tempLine,1),1);
yMat = (-repmat(tempLine(:,1),1,length(xList)).*xMat - repmat(tempLine(:,3),1,length(xList)))./repmat(tempLine(:,2),1,length(xList));
% figure,
hold on;grid on;

% plot(xMat', yMat','r');

%     for i = 1 : size(xMat,1)
%         plot(xMat(i,:)', yMat(i,:)','r');% hold on;plot(pt(1),pt(2),'*g');
%         drawnow;
%     end


id1 = find(tempLine(:,2) > 0);
id2 = find(tempLine(:,2) < 0);
mi = min(yMat(id1,:));
ma = max(yMat(id2,:));

idd = find(ma < mi);
Color = rand(1,3);
plot(xList(idd), mi(idd),'-','LineWidth',1,'Color',colorMat(ind,:));
plot(xList(idd), ma(idd),'-g','LineWidth',1,'Color',Color);
plot(xList(idd), ma(idd),'-','LineWidth',1,'Color',colorMat(ind,:));

% plot(0,0,'*k')
er = dot(tempLine',repmat([0 0 1],size(tempLine,1),1)');
end