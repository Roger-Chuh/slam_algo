function lineCoeff = testQuanThoughts5()
if 0
    data1 = load('C:\Users\rongjiezhu\Documents\WXWork\1688852647983342\Cache\File\2020-08\081500_pole5+3_show.txt');
    data2 = load('C:\Users\rongjiezhu\Documents\WXWork\1688852647983342\Cache\File\2020-08\081500_pole5-3_show.txt');
    % data2 = load('C:\Users\rongjiezhu\Documents\WXWork\1688852647983342\Cache\File\2020-08\081500_pole5-0.txt');
    figure,hold on;
    [ptlist1, xList1, yMat1] = do(data1);
    pause(1);
    [ptlist2, xList2, yMat2] = do(data2);
    
    
    range = find(xList1 > 8.55 & xList1 < 9.6);
    plot(xList1(range), max(yMat1(:,range)),'-b','LineWidth',3);
    plot(xList2(range), min(yMat2(:,range)),'-k','LineWidth',3);
    % ptlist12 = do([data1;data2]);
else
    inputDir = 'D:\Temp\20200806';
    inputDir = 'D:\Temp\20200807';
    inputDir = 'D:\Temp\20200811';
    inputDir = 'D:\Temp\20200811\1';
    inputDir = 'D:\Temp\20200811\1\1';
    dirInfo = dir(fullfile(inputDir,'*.txt'));
    
%     rangeList = [6.69 8.1;6.69 8.1; 7.05 7.779; 7.05 7.779; 7.13 7.94; 7.13 7.94];
    rangeList = [-24 -22;-24 -22; -22.91 -22.12; -22.91 -22.12; -22.39 -20.95; -22.39 -20.95];
    rangeList0 = [-10.34 -7.84;-9.65 -8.417;-9.68 -8.68];
    
    rangeList0 = [-25.71 -22;-23.75 -22.14;-23.11 -20.14];
    
    rangeList0 = [5.856 8.11;6.818 7.749;7.238 8.441];
    rangeList(2:2:2*size(rangeList0,1),:) = rangeList0;
    rangeList(1:2:2*size(rangeList0,1),:) = rangeList0;
    %   rangeList = rangeList(:);
    
    figure,hold on;
    for i = 1 : length(dirInfo)
        data = load(fullfile(inputDir, dirInfo(i).name));
        [ptlist2, xList2, yMat2, lineCoeff{i,1}] = do(data,i);
        
        range = find(xList2 > rangeList(i,1) & xList2 < rangeList(i,2));
        if mod(i,2)==1
            
            plot(xList2(range), max(yMat2(:,range)),'-b','LineWidth',3);
            
            afgd = 1;
        else
            
            plot(xList2(range), min(yMat2(:,range)),'-b','LineWidth',3);
            lineCoeff{i,1} = -lineCoeff{i,1};
            sajg = 1;
        end
        
    end
    
    
%     pt = [7.27253557, 60.97418628;
%         7.47119178,90.75336868;
%         7.44049384,122.12349368,];
%     
    
pt = [-22.4885 95.0058;
-22.6513 126.6513;
-22.6079 158.2413];

pt = [-8.987 61.086179402189146;
-9.248 90.86268516628888;
-9.224 122.23255487333536];

pt = [7.369762798991576 , 61.086179402189146;
7.571427801248781 ,  90.86268516628888;
7.547508926637793 ,  122.23255487333536;
-21.567079933926326 ,  158.34426866217856;
-21.83000067983201 , 126.33350422237083;
-21.846977612124036 ,  95.05833252356226];
    plot(pt(:,1),pt(:,2),'*g');
    
end
end
function [ptlist, xList, yMat, lineCoeff] = do(data1, ct)
[idxCell1, idx1] = splitIndex2(find(data1(:,1) ~= 0));

% figure,hold on;
cnt = 1;
xList = linspace(-50,18,1000);
yMat = [];
for i = 1 : length(idxCell1)
    if length(idxCell1{i}) > 2
        cnr = data1(idxCell1{i},2:3);
        %         plot(cnr(:,1), cnr(:,2),'-b');drawnow;
        [~,~,VV] = svd([cnr';ones(1,size(cnr,1))]');
        initLine = VV(:,end)';
        linek = [initLine(1)/initLine(3) initLine(2)/initLine(3) 1];
        [linek*[cnr';ones(1,size(cnr,1))]];
        linek =  linek./norm(linek(1:2));
        err = dot(repmat(linek,size(cnr,1),1)',[cnr ones(size(cnr,1),1)]')';
        lineCoeff(cnt,:) = linek;
        
        len1 = 80;
        len2 = 100;
        
        %         plot(curBodyOrigUncertPolygonBcsKey(1,[1:end 1]),curBodyOrigUncertPolygonBcsKey(3,[1:end 1]),'-xb');axis equal;
        %         hold on;line([(-linek(2)*len1-linek(3))/linek(1) (-linek(2)*len2-linek(3))/linek(1)]',[len1 len2]','Color',[1 0 0],'LineWidth',1);
        yList = (-linek(1).*xList - linek(3))./linek(2);
        yMat = [yMat; yList];
        if mod(ct,2)==0
            hold on;plot(xList, yList,'-m');
        else
            hold on;plot(xList, yList,'-r');
        end
        drawnow;
        points2 = lineToBorderPoints(linek, [110 20]);
        %         line(points2(:, [1,3])', points2(:, [2,4])');
        
        cnt = cnt + 1;
    end
    
end
index = nchoosek([1:size(lineCoeff,1)],2);

for i = 1 : size(index, 1)
    line1 = lineCoeff(index(i,1),:);
    line2 = lineCoeff(index(i,2),:);
    [~,~,Vv] = svd([line1;line2]);
    pt = Vv(:,end);
    ptz = [pt(1)./pt(end); pt(2)./pt(end); 1]';
    ptlist(i,:) = ptz(1:2);
end
polyCnr = FindPolyon(ptlist);
% plot(polyCnr(:,1), polyCnr(:,2),'-k','LineWidth',3);
% plot(ptlist(:,1), ptlist(:,2),'*r','LineWidth',3,'MarkerSize',3);

end