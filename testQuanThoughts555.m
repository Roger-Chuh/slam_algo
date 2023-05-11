function lineCoeff = testQuanThoughts555()
global sampleRange sampleRange1 colorMat draw draw2 rMatNew v2c
outDir = 'E:\bk_20180627\SLAM\slam_algo\quanOut';
% delete(fullfile(outDir,'region*.png'));
delete(fullfile(outDir,'transformed*.png'));


load('D:\Temp\20200831\rMatNew.mat')

load('D:\Temp\20200831\rMatNew2.mat')

load('D:\Temp\20200831\rMatNew3.mat')
% 
% load('D:\Temp\20200831\rMatNew4.mat')
% % 
% load('D:\Temp\20200831\rMatNew5.mat')
% 
% load('D:\Temp\20200831\rMatNew6.mat')
% 
% load('D:\Temp\20200831\rMatNew7.mat')
% 
% load('D:\Temp\20200831\rMatNew8.mat')
% 
load('D:\Temp\20200831\rMatNew9.mat')

draw = 1;
old = 0;


draw2 = 1;0; 1; 0;


sampleRange = [-50 50 5000];
sampleRange1 = [-20 10 2500];
sampleRange1 = [-50 50 2500];

inputDir = 'D:\Temp\20200806';
inputDir = 'D:\Temp\20200807';
inputDir = 'D:\Temp\20200811';
inputDir = 'D:\Temp\20200811\1';
% inputDir = 'D:\Temp\20200811\1\1';
inputDir = 'D:\Temp\20200817\tmp\tmp';
inputDir = 'D:\Temp\20200817\new\tmp\-100';
% inputDir = 'D:\Temp\20200817\new\tmp\-200';
% inputDir = 'D:\Temp\20200817\new\tmp\+100';
% inputDir = 'D:\Temp\20200817\new\tmp\+200';
% inputDir = 'D:\Temp\20200817\tmp(1)\tmp';

inputDir  = 'D:\Temp\20200817\new\tmp(5)\tmp';
inputDir = 'D:\Temp\20200817\new\tmp(6)\tmp';
inputDir = 'D:\Temp\20200817\new\tmp(7)\tmp';

inputDir = 'D:\Temp\20200817\new\tmp(8)\tmp';

inputDir = 'D:\Temp\20200817\new\tmp(5)\tmp';


% inputDir = 'D:\Temp\20200819\tmp(10)\tmp';

if 0
    
    [xGrid, yGrid] = meshgrid(-5:0.1:5, -10:0.1:10);
    [xGrid, yGrid] = meshgrid(-10:0.1:10, -30:0.1:30);
    
    
    [xGrid, yGrid] = meshgrid(-2:0.01:2, -8:0.01:2);
    
    [xGrid, yGrid] = meshgrid(-10:0.1:10, -50:0.1:50);
    
    
    inputDir = 'D:\Temp\20200819\tmp(9)\tmp';
    inputDir = 'D:\Temp\20200817\new\tmp(7)\tmp';
    inputDir = 'D:\Temp\20200817\new\tmp(5)\tmp';
    inputDir = 'D:\Temp\20200819\pole_reprojection_line\0820-081500+-5-0p0279';
    %         inputDir = 'D:\Temp\20200819\pole_reprojection_line\0820-081500+-3-0p0279';
    %     inputDir2 = 'D:\Temp\20200819\theta_hist\081500\img';Info = readConfig(fullfile(inputDir2, '081500_Index.txt'),'0815');
    
    inputDir = 'D:\Temp\20200824';
    
    DataAll_ = ArrangeData(fullfile(inputDir, 'ccs_disturb(2).txt'));
    
    
    golden = load(fullfile(inputDir, '081500-5000-6000-utm.txt'));
    golden = [[5000:5999]' golden];
    
    if 0
        DataAll_{10,2}(3,:) = [];DataAll_{10,3}(3,:) = [];DataAll_{10,4}(3,:) = [];
        DataAll_{11,2}(3,:) = [];DataAll_{11,3}(3,:) = [];DataAll_{11,4}(3,:) = [];
    else
        DataAll_{10,2}(end,:) = [];DataAll_{10,3}(end,:) = [];DataAll_{10,4}(end,:) = [];
        DataAll_{11,2}(end,:) = [];DataAll_{11,3}(end,:) = [];DataAll_{11,4}(end,:) = [];
    end
    
    
    timeStampList = 5180 : 10 : 5300;
    timeStampList = 5180 : 10 : 5290;
    % timeStampList = setdiff(timeStampList, 5260);
    % timeStampList = 5180;
    %     timeStampList = 5260;
    timeStampList = 5190;
    timeStampList = cell2mat(DataAll_(:,1))';
    
    tsList = cell2mat(DataAll_(:,1))';
    
    
    %     timeStampList = 5200;
    timeStampList = 5180 : 10 : 5290;
    %     timeStampList = [5260 5270];
    
    
%     timeStampList = 5200;
%     timeStampList = 5180;
%     timeStampList = 5380;

    goldenPose = golden(ismember(golden(:,1),timeStampList'),:);
    
    startFrame = 5170;
    
    
    goldenPose(:,2:3) = goldenPose(:,2:3) - golden(find(golden(:,1) == startFrame),2:3);
    
    goldenPose_ = [goldenPose(:,1:2) zeros(size(goldenPose,1),1) goldenPose(:,3)];
    goldenPose = goldenPose_;
    goldenPose(:,2:4) = goldenPose_(:,2:4)*roty(173.1245)';
    
    DataAll = DataAll_(ismember(tsList, timeStampList),:);
    
    pt = [7.369762798991576 , 61.086179402189146;
        7.571427801248781 ,  90.86268516628888;
        7.547508926637793 ,  122.23255487333536;
        -21.567079933926326 ,  158.34426866217856;
        -21.83000067983201 , 126.33350422237083;
        -21.846977612124036 ,  95.05833252356226];
    %     pt = pt([1     2     6     3     5     4],:);
    pt = pt([1     2     3  6       5     4],:);
    %     pt = pt./0.82;
    pt = pt*1;
    ptPole = {'pole4';'pole5';'pole6';'pole5l';'pole6l';'pole7l'};
elseif 1
    
    
    inputDir = 'D:\Temp\20200831';
    if 1
        startFrame = 5170;
        yAng = 173.1245;
        
        disturb_file = 'D:\Temp\20200831\ccs_disturb(5).txt';
        disturb_file = 'D:\Temp\20200831\081500_5300-5590ccs_disturb.txt';
        
        
        
        pole_utm_file = 'D:\Temp\20200831\pole_utm.txt';
%         pole_utm_file = 'D:\Temp\20200831\pole_utm_est2.txt';
%         pole_utm_file = 'D:\Temp\20200831\pole_utm_est3.txt';
%         pole_utm_file = 'D:\Temp\20200831\pole_utm_est4.txt';
        
        img_gps_file = 'D:\Temp\20200831\081500-5000-6000-utm.txt';
        img_gps_file_timeStamp_range = [5000:5999];
        timeStampList = 5300 : 10 : 5580;
        
        %
        timeStampList = 5370;
        %         timeStampList = 5480;
        %         timeStampList = 5380;
%             timeStampList = 5380;
        timeStampList = 5430;
%         timeStampList = 5460;
        %     timeStampList = 5380;
        %         timeStampList = [5430 5470];
        %         timeStampList = [5370 5480];
        %     timeStampList = 5440;
            timeStampList = 5450;
%             timeStampList = 5460;
%         timeStampList = 5490;
        %     timeStampList = [5370 5460 5480];
%         timeStampList = [5410:10:5450];
        
        lane_file = 'D:\Temp\20200903\07300_1\edge_4_10.txt';
%         lane_file = 'D:\Temp\20200903\07300_1\edge_2l_14l.txt';
        
        
        lanes = load(lane_file);
        lanes = lanes(:,[1 3]);
        
    else
        startFrame = 300;
        yAng = -6.5; 173.1245;
        disturb_file = 'D:\Temp\20200903\07300_1\073000-5370ccs_disturb.txt';
        timeStampList = 300 : 10 : 420;
        timeStampList = 340;
        timeStampList = 370;
        
        disturb_file = 'D:\Temp\20200903\07300_1\073000-5480ccs_disturb.txt';
        startFrame = 120;
        yAng = -6.5; 173.1245;
        timeStampList = 120 : 10 : 180;
        %         timeStampList = 340;
        %         timeStampList = 370;
        
        disturb_file = 'D:\Temp\20200903\07300_1\073000-510ccs_disturb.txt';
        startFrame = 120;
        yAng = -6.5; 173.1245;
        timeStampList = 510; 120 : 10 : 180;
        
        disturb_file = 'D:\Temp\20200903\07300_1\073000-270ccs_disturb(1).txt';
        disturb_file = 'D:\Temp\20200903\07300_1\073000-270ccs_disturb+-10.txt';
        startFrame = 120;
        yAng = -6.5; 173.1245;
        timeStampList = 270; 510; 120 : 10 : 180;
        
        
        
        disturb_file = 'D:\Temp\20200903\07300_1\073000-510ccs_disturb(1).txt';
        disturb_file = 'D:\Temp\20200903\07300_1\073000-510ccs_disturb+-10.txt';
        startFrame = 120;
        yAng = -6.5; 173.1245;
        timeStampList = 510; 120 : 10 : 180;
        
        






        pole_utm_file = 'D:\Temp\20200831\pole_utm.txt';
        pole_utm_file = 'D:\Temp\20200831\pole_utm_est2.txt';
        
        img_gps_file = 'D:\Temp\20200903\07300_1\073000-0-1000-utm.txt';
        img_gps_file_timeStamp_range = [0:999];
        
        lane_file = 'D:\Temp\20200903\07300_1\edge_4_10.txt';
        lane_file = 'D:\Temp\20200903\07300_1\edge_2l_14l.txt';
        lanes = load(lane_file);
        lanes = lanes(:,[1 3]);
    end
    
    
    thetaList = linspace(-2, 2, 21);
%     thetaList = linspace(-5, 5, 51);
    
    poleCoord = ReadPole(pole_utm_file);
    
    [xGrid, yGrid] = meshgrid(-10:0.1:10, -50:0.1:50);
%     inputDir = 'D:\Temp\20200831';
     
%     DataAll_ = ArrangeData(fullfile(inputDir, 'ccs_disturb(4).txt'));
    DataAll_ = ArrangeData(disturb_file);
    
    for i = 1 : size(DataAll_,1)
       tp =  DataAll_(i,:);
       unv = [];
       for ii = 1 : size(tp{4},1)
%            if 0 % strcmp(tp{4}(ii,:), 'pole9l') ||strcmp(tp{4}(ii,:), 'pole11l') %||strcmp(tp{4}(ii,:), 'pole12l')
%            if strcmp(tp{4}(ii,:), 'pole8l') ||strcmp(tp{4}(ii,:), 'pole10l') %||strcmp(tp{4}(ii,:), 'pole12l')
%            if strcmp(tp{4}(ii,:), 'pole9') ||strcmp(tp{4}(ii,:), 'pole9l') || strcmp(tp{4}(ii,:), 'pole8l') || strcmp(tp{4}(ii,:), 'pole10l')
%            if strcmp(tp{4}(ii,:), 'pole9') ||strcmp(tp{4}(ii,:), 'pole9l') || strcmp(tp{4}(ii,:), 'pole8l') % || strcmp(tp{4}(ii,:), 'pole10l')
%            if strcmp(tp{4}(ii,:), 'pole9') || strcmp(tp{4}(ii,:), 'pole8l') || strcmp(tp{4}(ii,:), 'pole10l') % ||strcmp(tp{4}(ii,:), 'pole9l')
           if 0 % strcmp(tp{4}(ii,:), 'pole9l') % || strcmp(tp{4}(ii,:), 'pole8l') || strcmp(tp{4}(ii,:), 'pole10l') % ||strcmp(tp{4}(ii,:), 'pole9l')
               unv = [unv ;ii];
           end
       end
       if 1 % size(tp{4},1) > 2
           tp{2}(unv,:) = [];
           tp{3}(unv,:) = [];
           tp{4}(unv,:) = [];
           DataAll_(i,:) = tp;
       end
    end
    
    
    
    golden = load(img_gps_file);
    golden = [img_gps_file_timeStamp_range' golden];
    
    
    tsList = cell2mat(DataAll_(:,1))';

    goldenPose = golden(ismember(golden(:,1),timeStampList'),:);
    
    
    goldenPose(:,2:3) = goldenPose(:,2:3) - golden(find(golden(:,1) == startFrame),2:3);
    
    lanes = lanes - golden(find(golden(:,1) == startFrame),2:3);
    lanes = [lanes(:,1) zeros(size(lanes,1),1) lanes(:,2)];

    goldenPose_ = [goldenPose(:,1:2) zeros(size(goldenPose,1),1) goldenPose(:,3)];
    goldenPose = goldenPose_;
    
    
    goldenPose(:,2:4) = goldenPose_(:,2:4)*roty(yAng)';
    lanes = lanes*roty(yAng)';
    Lanes = [lanes(1:size(lanes,1)/2,:) lanes(size(lanes,1)/2+1:end,:)];
    
    DataAll = DataAll_(ismember(tsList, timeStampList),:);
    pt = [7.369762798991576 , 61.086179402189146;
        7.571427801248781 ,  90.86268516628888;
        7.547508926637793 ,  122.23255487333536;
        -21.567079933926326 ,  158.34426866217856;
        -21.83000067983201 , 126.33350422237083;
        -21.846977612124036 ,  95.05833252356226];
    %     pt = pt([1     2     6     3     5     4],:);
    pt = pt([1     2     3  6       5     4],:);
    %     pt = pt./0.82;
    pt = pt*1;
    
    
    poleList = {};
    for j = 1 : size(DataAll,1)
       tp = DataAll(j,:);
       for jj = 1: size(tp{4},1)
           poleList = [poleList; tp{4}(jj,:)];
       end
        
        
    end
    ptPole = unique(poleList);
    
    pt = [];
    as = [];
    for j = 1:length(ptPole)
        for jj = 1 : length(poleCoord(:,1))
            if strcmp(poleCoord{jj,1}, ptPole{j}(5:end))
                idd = jj;
                as = [as;idd];
                break
            end
        end
        pt = [pt; poleCoord{idd,2}];
    end
    pt0 = pt(:,[1 3]) - golden(find(golden(:,1) == startFrame),2:3);
    
    
    
    pt0 = [pt0(:,1) zeros(size(pt0,1),1) pt0(:,2)];
    pt = pt0*roty(yAng)';
    pt = pt(:,[1 3]);
    
    [xGrid, yGrid] = meshgrid(-10:0.1:10, -goldenPose(1,4)-50:0.1:-goldenPose(1,4)+50);
    [xGrid, yGrid] = meshgrid(-15:0.1:15,-150:0.1:150);
    [xGrid1, yGrid1] = meshgrid(-15:0.1:15,-50:0.1:50);
    
    if length(timeStampList) == 1
%         [xGrid, yGrid] = meshgrid(-10:0.1:10, -goldenPose(1,4)-80:0.1:-goldenPose(1,4)+80);
        [xGrid, yGrid] = meshgrid(-15:0.1:15, -80:0.1:+80);
    end
    
    if length(timeStampList) == 2
%         [xGrid, yGrid] = meshgrid(-15:0.1:15, -300:0.1:300);
        [xGrid, yGrid] = meshgrid(-15:0.1:15, -80:0.1:80);
    end
%     [xGrid, yGrid] = meshgrid(-10:0.1:10, -300:0.1:150);
%     ptPole = {'pole4';'pole5';'pole6';'pole5l';'pole6l';'pole7l'};
    asfgjk = 1;
    
    
    
elseif 1 
    
%     poleCoord = ReadPole('D:\Temp\20200831\pole_utm.txt');
     poleCoord = ReadPole('D:\Temp\20200831\pole_utm.txt');
    
    
    [xGrid, yGrid] = meshgrid(-10:0.1:10, -50:0.1:50);
    inputDir = 'D:\Temp\20200901';
     
%     DataAll_ = ArrangeData(fullfile(inputDir, 'ccs_disturb(4).txt'));
    DataAll_ = ArrangeData(fullfile(inputDir, '073000-500-1000ccs_disturb.txt'));
    
%     for i = 1 : 20
%        tp =  DataAll_(i,:);
%        unv = [];
%        for ii = 1 : size(tp{4},1)
%            if strcmp(tp{4}(ii,:), 'pole9l') ||strcmp(tp{4}(ii,:), 'pole11l') %||strcmp(tp{4}(ii,:), 'pole12l')
%                unv = [unv ;ii];
%            end
%        end
%        if size(tp{4},1) > 2
%            tp{2}(unv,:) = [];
%            tp{3}(unv,:) = [];
%            tp{4}(unv,:) = [];
%            DataAll_(i,:) = tp;
%        end
%     end
    
    
    
    golden = load(fullfile(inputDir, '073000-0-1000-utm.txt'));
    golden = [[0:999]' golden];
    
    
    tsList = cell2mat(DataAll_(:,1))';
    timeStampList = [590:10: 920];
    
    
%         timeStampList = 5370;
    %     timeStampList = 5380;
    %     timeStampList = 5430;
    %     timeStampList = 5380;
    %     timeStampList = 5430;
    %     timeStampList = 5440;
    %     timeStampList = 5450;
%     timeStampList = [5370 5460 5480];
    goldenPose = golden(ismember(golden(:,1),timeStampList'),:);
    
    startFrame = 500;
    
    goldenPose(:,2:3) = goldenPose(:,2:3) - golden(find(golden(:,1) == startFrame),2:3);
    
    goldenPose_ = [goldenPose(:,1:2) zeros(size(goldenPose,1),1) goldenPose(:,3)];
    goldenPose = goldenPose_;
    
    yAng = -7;  173.1245;
    
    goldenPose(:,2:4) = goldenPose_(:,2:4)*roty(yAng)';
    
    DataAll = DataAll_(ismember(tsList, timeStampList),:);
    pt = [7.369762798991576 , 61.086179402189146;
        7.571427801248781 ,  90.86268516628888;
        7.547508926637793 ,  122.23255487333536;
        -21.567079933926326 ,  158.34426866217856;
        -21.83000067983201 , 126.33350422237083;
        -21.846977612124036 ,  95.05833252356226];
    %     pt = pt([1     2     6     3     5     4],:);
    pt = pt([1     2     3  6       5     4],:);
    %     pt = pt./0.82;
    pt = pt*1;
    
    
    poleList = {};
    for j = 1 : size(DataAll,1)
       tp = DataAll(j,:);
       for jj = 1: size(tp{4},1)
           poleList = [poleList; tp{4}(jj,:)];
       end
        
        
    end
    ptPole = unique(poleList);
    
    pt = [];
    for j = 1:length(ptPole)
        for jj = 1 : length(poleCoord(:,1))
            if strcmp(poleCoord{jj,1}, ptPole{j}(5:end))
                idd = jj;
                break
            end
        end
        pt = [pt; poleCoord{idd,2}];
    end
    pt0 = pt(:,[1 3]) - golden(find(golden(:,1) == startFrame),2:3);
    
    
    
    pt0 = [pt0(:,1) zeros(size(pt0,1),1) pt0(:,2)];
    pt = pt0*roty(yAng)';
    pt = pt(:,[1 3]);
    
    [xGrid, yGrid] = meshgrid(-10:0.1:10, -goldenPose(1,4)-50:0.1:-goldenPose(1,4)+50);
    [xGrid, yGrid] = meshgrid(-10:0.1:10,-150:0.1:150);
    [xGrid1, yGrid1] = meshgrid(-10:0.1:10,-50:0.1:50);
    
    if length(timeStampList) == 1
        [xGrid, yGrid] = meshgrid(-10:0.1:10, -goldenPose(1,4)-50:0.1:-goldenPose(1,4)+50);
    end
    
%     ptPole = {'pole4';'pole5';'pole6';'pole5l';'pole6l';'pole7l'};
    asfgjk = 1;
    
    
    
else
    [xGrid, yGrid] = meshgrid(-20:0.1:20, -5:0.1:5);
    
    
     [xGrid, yGrid] = meshgrid(-20:0.1:20, -50:0.1:50);
    
    inputDir = 'D:\Temp\20200819\tmp(10)\tmp';
    %     inputDir = 'D:\Temp\20200819\pole_reprojection_line\0820-073000+-5-0p0279';
    %     inputDir2 = 'D:\Temp\20200819\theta_hist\073000\img';Info = readConfig(fullfile(inputDir2, '073000_Index.txt'), '0730');
    
    inputDir = 'D:\Temp\20200827';
    
    DataAll_ = ArrangeData(fullfile(inputDir, '073000ccs_disturb.txt'));
    
    timeStampList = cell2mat(DataAll_(:,1))';
    
    tsList = cell2mat(DataAll_(:,1))';
    
    
    
    
    
    golden = load(fullfile(inputDir, '073000-0-1000-utm.txt'));
    golden = [[1:1000]' golden];
    
    timeStampList = [500:10: 920];
    timeStampList = [590:10: 920];
    % timeStampList = [570:10: 600];
    % timeStampList = 800; 710;
    
    
     goldenPose = golden(ismember(golden(:,1),timeStampList'),:);

     %     goldenPose(:,2:3) = goldenPose(:,2:3) - goldenPose(1,2:3);
    
    
    startFrame = 500;
    
    
%     goldenPose(:,2:3) = goldenPose(:,2:3) - goldenPose(find(golden(:,1) == startFrame),2:3);
    goldenPose(:,2:3) = goldenPose(:,2:3) - golden(find(golden(:,1) == startFrame),2:3);

    
    goldenPose_ = [goldenPose(:,1:2) zeros(size(goldenPose,1),1) goldenPose(:,3)];
    goldenPose = goldenPose_;
    goldenPose(:,2:4) = goldenPose_(:,2:4)*roty(173 + 180)';
    
    DataAll = DataAll_(ismember(tsList, timeStampList),:);
    
    
    
    pt = [8.608669271623615 , 7.837967624321777 , 256.0952048281404 ;
        7.598461176545243 , 7.311893374321777 , 227.15844255107456 ;
        7.500400499564211 , 7.645434164321777 , 197.66707390213494 ;
        7.475684730811029 , 7.674464784321778 , 165.3272509404283 ;
        7.387511121351537 , 7.740368004321777 , 133.67655004319363 ;
        7.363452372778514 , 7.546549974321778 , 102.42525446345124 ;
        7.101853027603907 , 7.942707054321776 , 70.38452807555815 ;
        7.820672860185806 , 7.849753064321778 , 43.82732527158226 ;
        7.329305130953069 , 7.849753064321778 , 14.152314299938547 ;
        7.253325235813242 , 7.681934714321777 , -14.508889447777861 ;
        5.909413146315152 , 8.922061574321777 , -47.463873946693475 ;
        6.350253440294907 , 8.453744364321778 , -74.91622542834426 ;
        7.270264200974482 , 7.418663634321778 , -102.43176033335217 ;];
    pt = pt([7 6 5 4 3 2],[1 3]);
%     pt = pt([4     5     6     1     2     3],:);
    
    ptPole = {'pole7';'pole8';'pole9';'pole10';'pole11';'pole12'};
end



if 0
    DirInfo = dir(fullfile(inputDir,'*.txt'));
end



if 0
    for i = 1 : size(DirInfo,1)
        PoleName{i,1} =  DirInfo(i).name(8:15);
    end
    
    sets = 1; 11;
    eachSet = size(DirInfo,1)/sets;
    
    Dir = {};
    for g = 1:sets
        %     tempDir = DirInfo(1:eachSet,:);
        Dir{g,1} = DirInfo(g:sets:end,:);
        PoleNameStack{g,1} = PoleName(g:sets:end,:);
    end
else
    Dir = 1;
end

% close all

% % % % % % if 1
% % % % % %     pt = [7.369762798991576 , 61.086179402189146;
% % % % % %         7.571427801248781 ,  90.86268516628888;
% % % % % %         7.547508926637793 ,  122.23255487333536;
% % % % % %         -21.567079933926326 ,  158.34426866217856;
% % % % % %         -21.83000067983201 , 126.33350422237083;
% % % % % %         -21.846977612124036 ,  95.05833252356226];
% % % % % %     pt = pt([1     2     6     3     5     4],:);
% % % % % % else
% % % % % %     pt = [-23.5 96; -22.9 126.8;-21.4 152.8;7 57; 7.4 86.4;7.9 121];
% % % % % %     [xGrid, yGrid] = meshgrid(-1:0.05:1, -1:0.05:1);
% % % % % % end

% pt(:,1) = pt(:,1) + 1.2;
% pt(:,2) = pt(:,2) - 1.7;
% pt(:,2) = pt(:,2) - 0.3;


pix000 = [xGrid(:) yGrid(:)];

pix111 = [xGrid1(:) yGrid1(:)];

% pix111 = pix000;


colorMat = [1 0 0;0 1 0;0 0 1;1 0 1; 0 1 1; 1 1 0];

colorMat = [colorMat; rand(size(pt,1) - size(colorMat,1),3)];




Mat = [];


% thetaList = linspace(-1.5,1.5,150);
% thetaList = linspace(-3,-1.5,70);
% thetaList = linspace(-1.5,0, 70);
% thetaList = linspace(-3,-2, 50);
% thetaList = linspace(3,-3, 200);
% 
% thetaList = linspace(0,1, 5);
% thetaList = linspace(-3, 3, 300);
% thetaList = linspace(-2, 2, 20);
% thetaList = linspace(-1, 2, 15);
% thetaList = linspace(-2, 2, 51);
% thetaList = [-2.156 -2.819 -1.583];

% thetaList = linspace(-5,-3, 70);
% thetaList = linspace(-2,2, 100);
% thetaList = linspace(-3,3,700);

% thetaList = -2;
% thetaList = [0.04523 0.07538];
% thetaList = [0 thetaList(end) thetaList];
% thetaList = [0 thetaList];

% thetaList = 0.6;
% thetaList = 0.58; 0.5592;
if 0
    thetaList = linspace(0.7,1,21);
    thetaList = linspace(0.8,1,21);
    thetaList = 0.83;
    % thetaList = 0.55;
    % thetaList = linspace(0.4,0.5,11);
    thetaList = 0.48;
    thetaList = -0.48;
    [xGrid, yGrid] = meshgrid(-10:0.1:10, -100:0.1:100);
    
end


thetaExtra = [ -0.7826
    -0.5435
    -0.7391
    -0.5435
    -0.3043
    -0.7174
    -0.5217
    -0.2391
    -0.3913
    -0.1087
    -1.5000
    -1.5000];

thetaExtra = [];

scale = 1;

for g = 1 : size(Dir,1)
    if 0
        dirInfo = Dir{g,1};
        poleName = PoleNameStack{g, 1};
    end
    
    
    inPolyMat = {};
    cnt = 1;
    
    for iii = 1 : length(thetaList)
        theta = thetaList(iii);
        if draw
            figure(1);clf;
        end
        
        xzOfst = [0 0];
        
        for ii = 1 : length(timeStampList)
            xzOfst =  -(goldenPose(ii,[2 4]));
            if draw
                figure(1);subplot(1,4,1);cla;hold on;grid on;axis equal;
                subplot(1,4,2);hold on;grid on;axis equal;
                subplot(1,4,4);hold on;grid on;axis equal;
                subplot(1,4,3);cla;hold on;grid on;axis equal;  axis([sampleRange1(1) sampleRange1(2) -50-goldenPose(ii,[4]) 50-goldenPose(ii,[4])])
            end
            %             figure(2);clf;hold on;grid on;axis equal;
            
            if ~isempty(thetaExtra)
                theta = thetaExtra(ii);
            end
            timeStamp = timeStampList(ii);
            
            inPolyMat{cnt,1} = [theta timeStamp];
            if 0
                try
                    Info(:,3) = [];
                catch
                    sagfh = 1;
                end
                
                for i = 1 : size(Info,1)
                    id = find(Info{i,2} == timeStamp);
                    if isempty(id)
                        id = 100;
                    end
                    Info{i,3} = id;
                end
            end
            
            seenPoles = [];
            inMat = [];
            ptUseList = [];
            Lines1 = [];
            DataTemp = DataAll(ii,:);
            for i = 1 : size(DataTemp{2},1)
                %             for i = 6:6
                %                 name = poleName(2*i);
                
                
                
                if 0
                    data1 = load(fullfile(inputDir, dirInfo(2*i - 1).name));
                    data2 = load(fullfile(inputDir, dirInfo(2*i).name));
                else
                    data1 = DataTemp{2}(i,:);
                    data2 = DataTemp{3}(i,:);
                end
                %                 poleId = FindPole(Info, name);
                
                poleName = DataTemp{4}(i,:);
                
                for o = 1 : length(ptPole)
                    if isequal(ptPole(o,:), poleName)
                        chosenId = o;
                        break;
                    end
                end
                ptUse = pt(chosenId,:);
                if draw
                    figure(1),subplot(1,4,1)
                    text(ptUse(1), ptUse(2),num2str(i), 'Color',[0 0 0],'FontSize',15);
                end
                [xList1, yMat1, lineCoeff1] = do(data1, 2*i-1, 1);
                [xList2, yMat2, lineCoeff2] = do(data2, 2*i, 1);
                
                
                
                tempLine = [lineCoeff1; lineCoeff2];
                splitInd = {[1:size(lineCoeff1,1)] [size(lineCoeff1,1) + 1: size(tempLine,1)]};
                [polyCnr1, lines] = processing(xList1, yMat1, yMat2,{[tempLine]});
                
                if 1 %size(lineCoeff1,1) >= poleId(2)
                    
                    ptUseList = [ptUseList; ptUse];
                    line1 = lineCoeff1; %(poleId(2),:);
                    line2 = lineCoeff2; %(poleId(2),:);
                    
                    if 0
                        rng = DetermineRng(line1, line2, ptUse);
                        seenPoles = [seenPoles; [name {rng}]];
                    end
                    
                    lines1 = [line1; line2];
                    Lines1 = [Lines1; lines1];
                    %                     ptUse = [0 0];
                    coeffStackNew = [lines1(:,1) lines1(:,2) (lines1(:,3) + (lines1(:,1).*cosd(theta).*scale - lines1(:,2).*sind(theta).*scale).*ptUse(1) + (lines1(:,1).*sind(theta).*scale + lines1(:,2).*cosd(theta).*scale).*ptUse(2))];
                    %                     [xList1, yMat1, yMat2] = preprocess(coeffStackNew, {[1:size(lineCoeff1,1)] [size(lineCoeff1,1) + 1: size(tempLine,1)]});
                    [xList1, yMat1, yMat2] = preprocess(coeffStackNew, {[1] [2]});
                    
                    polyCnr = processing(xList1, yMat1, yMat2);
                    
                    if ii == 1
                        pix0 = pix000 + xzOfst;
                    else
                        pix0 = pix111 + xzOfst;
                    end
                    [inn,on] = inpolygon(pix0(:,1),pix0(:,2),polyCnr(:,1),polyCnr(:,2));
                    
                    coeffStackNew2 = repmat(coeffStackNew, size(pix0,1),1);
                    pix00 = repmat([pix0 ones(size(pix0,1),1)], size(coeffStackNew, 1), 1);
                    pix00 = reshape(pix00, size(pix0,1), size(coeffStackNew, 1), 3);
                    pix00 = reshape(permute(pix00, [2 1 3]),[],3);
                    erVec = dot(coeffStackNew2',pix00');
                    erMat = reshape(erVec,size(coeffStackNew, 1),[])';
                    in = sum(erMat'< 0)' == size(coeffStackNew, 1);
                    
                    
                    
                    inMat = [inMat in];
                    if draw
                        figure(1);subplot(1,4,3); plot(polyCnr(:,1), polyCnr(:,2),'-','LineWidth',1,'Color',colorMat(chosenId,:));drawnow;
                    end
                    
                    
                end
                
                
                
                
                
                
                if 0
                    Data{i,1} = lineCoeff1;
                    Data{i,2} = lineCoeff2;
                else
                    Data{i,1} = lines(splitInd{1},:);
                    Data{i,2} = lines(splitInd{2},:);
                end
                
                
                
                
                ptMean(i,:) = mean(polyCnr1);
                if timeStamp < 0
                    if draw
                        figure(1),subplot(1,4,1);plot(polyCnr1(:,1), polyCnr1(:,2),'-','LineWidth',3,'Color',[0 0 1]);drawnow;
                    end
                end
                
            end
            
            if isempty(Lines1)
                continue;
            end
            
            intersectPt = LineIntersection(Lines1);
            sumMat = sum(inMat',1);
            inIds = find(sumMat == max(sumMat));
            
            inPolyMat{cnt,2} = [max(sumMat) size(inMat,2)];
            inPolyMat{cnt,3} = inIds;
            
            
            observedPole = unique(inMat(inIds,:),'rows');
            inlierMat = inMat(inIds,:);
            
            
            
            if draw
                figure(1),subplot(1,4,3),plot(pix0(inIds,1), pix0(inIds,2),'.k');
                title(sprintf('theta: %0.5f degree\nintersection: %d/%d', -theta,max(sumMat), size(inMat,2)))
                
                legendStr = {};
                for o = 1 : 2*size(inMat,2)+1
                    legendStr{1,o} = '';
                end
                
                figure(1),subplot(1,4,1);
                plot(intersectPt(1),intersectPt(2),'*k','MarkerSize', 5, 'LineWidth',5);
                for i2 = 1 : min(size(pt,1),size(colorMat,1))
                    try
                        plot(pt(i2,1), pt(i2,2), '*', 'Color', colorMat(i2,:),'MarkerSize', 5, 'LineWidth',5);
                        legendStr = [legendStr ptPole(i2)];
                    catch
                    end
                end
                legend(legendStr);
            end
            if 0 % max(sumMat) == size(inMat,2)
                test = [pix0(inIds,:)];
                theta0 = theta;
                scale0 = scale;
                r = scale0.*[cosd(theta0) sind(theta0);-sind(theta0) cosd(theta0)];
                ptNew = (r*ptUseList' + repmat(mean(test,1)',1 ,size(ptUseList,1)))';
                if draw
                    figure(1),subplot(1,4,1),hold on;plot(ptNew(:,1), ptNew(:,2),'*k');
                end
            end
            
            
            D = pdist2(ptMean,pt,'euclidean');
            [minVal, minId] = min(D);
            
            %             err = ptMean(minId,:) - pt;
            
            %             Data2 = Data(minId,:);
            
            
            
            try
                %                 mat = cell2mat(seenPoles(:,2) );
                %                 Mat = [Mat; mat];
                %                 minmaxz = [max(mat(:,1)) min(mat(:,2))];
                %                 title(sprintf('time stamp: %d\nrange: [%0.4f ~ %0.4f] m', timeStamp,minmaxz(1),minmaxz(2)));
                if draw
                    title(sprintf('time stamp: %d', timeStamp));
                    
                    figure(1),subplot(1,4,2);
                    plot(goldenPose(ii,2),goldenPose(ii,4),'xb','MarkerSize', 5,'LineWidth', 5);
                    plot(Lanes(:,[1 4])', Lanes(:,[3 6])','-g');
%                     line(Lanes(:,1), Lanes(:,3));
                    
                end
                if 0
                    plot(mat);legend('min z shift',' max z shift')
                else
                    
                    tList = pix0(inIds,:);
                    theta0 = theta;
                    scale0 = scale;
                    r = scale0.*[cosd(theta0) sind(theta0);-sind(theta0) cosd(theta0)];
                    tNew = (-r'*tList')';
                    if 0
                        plot(-pix0(inIds,1) + intersectPt(1), -pix0(inIds,2) + intersectPt(2),'*k','MarkerSize', 3,'LineWidth', 3);grid on;axis equal;
                        polyCnr2 = FindPolyon(-pix0(inIds,:));
                    else
                        
                        try
                            polyCnr2 = FindPolyon(tNew);
                        catch
                            asghi = 1;
                        end
                        
                        if 0
                            xz = mean(tList,1);
                        else
                            xz = -(goldenPose(ii,[2 4]));
                        end
                        if length(timeStampList) > 2
                            xzOfst = xz;
                        else
                            xzOfst = 0; % xz;
%                             xzOfst = mean(tList(tList(:,2) <= xz(2),:),1);
                        end
                        %                         inPolyMat{cnt,4} = polyCnr2 + intersectPt;
                        inPolyMat{cnt,4} = tNew + intersectPt;
                        if draw
                            plot(tNew(:,1) + intersectPt(1), tNew(:,2) + intersectPt(2),'*k','MarkerSize', 3,'LineWidth', 3);grid on;axis equal;
                            %                             plot(polyCnr2(:,1) + intersectPt(1), polyCnr2(:,2) + intersectPt(2),'-k','MarkerSize', 3,'LineWidth', 3);grid on;axis equal;
                        end
                        
                    end
                    if draw
                        plot(intersectPt(1),intersectPt(2),'sr','MarkerSize', 5,'LineWidth', 5)
                        title('shift area');
                        
                        
                        subplot(1,4,4); plot(polyCnr2(:,1), polyCnr2(:,2),'-','MarkerSize', 3,'LineWidth', 3);grid on;axis equal;
                        title(num2str(unique(inMat(inIds,:),'rows')));
                    end
                    %                     title(sprintf('shift area\nintersection: %d/%d',max(sumMat), size(inMat,2)))
                end
                if 0 % minmaxz(1) < minmaxz(2)
                    pt(:,2) = pt(:,2) + mean(minmaxz);
                    subplot(1,2,1);plot(pt(:,1), pt(:,2),'*k');
                end
                
                if draw
                    figure(1),subplot(1,4,1),hold on;
                end
                for hj = 1 : size(observedPole,1)
                    test = pix0(ismember(inMat, observedPole(hj,:), 'rows'),:);
                    %                 test = [pix0(inId,:)];
                    theta0 = theta;
                    scale0 = scale;
                    r = scale0.*[cosd(theta0) sind(theta0);-sind(theta0) cosd(theta0)];
                    ptNew = (r*ptUseList' + repmat(mean(test,1)',1 ,size(ptUseList,1)))';
                    if draw
                        plot(ptNew(:,1), ptNew(:,2),'*');
                    end
                end
                
                if draw
                    drawnow;
                end
                
            catch
                safjhv = 1;
                
            end
            if draw
                figure(1),subplot(1,4,1);plot(Lanes(:,[1 4])', Lanes(:,[3 6])','-g');
            end
            if draw2 % max(sumMat) == size(inMat,2)
                %                 saveas(gcf,fullfile(outDir,sprintf('transformed_%05d.fig',length(dir(fullfile(outDir,'transformed_*.fig')))+1)));
                
%                 subplot(1,4,1);
%                 for jk = 1:size(DataTemp{1,4},1)
%                     
%                 end
                saveas(gcf,fullfile(outDir,sprintf('transformed_%08d_%08d.fig',1000.*round(theta,3), timeStamp)));
                saveas(gcf,fullfile(outDir,sprintf('transformed_%08d_%08d.png',1000.*round(theta,3), timeStamp)));
            end
            cnt = cnt+1;
            
            
            %%
            continue;
            
            
            % thetaList = 0.24 + linspace(-0.42,0.18,400);
            % thetaList = 0 + linspace(0.05,0.6,500);
            thetaList = 0 + linspace(-1,1,100);
            thetaList = [0 thetaList];
            
            % scaleList = 1.016 + linspace(-0.010,0.04,100);
            scaleList = 0 + linspace(1.001,1.06,500);
            scaleList = [1 scaleList];
            
            inPolyMat = {};
            cnt = 1;
            
            seenPoles = [];
            
            for j = 1 : 1% length(thetaList)
                theta = thetaList(j);
                
                for jj = 1 : 1 % length(scaleList)
                    scale = scaleList(jj);
                    
                    if draw
                        figure(3),clf; hold on;grid on;
                    end
                    
                    %         axis equal;
                    %         title(sprintf('%0.6f degree\n%0.6f scale', theta, scale));
                    pix = pix0;
                    in = true(size(pix,1),1);
                    
                    inPolyMat{cnt,1} = [theta scale];
                    
                    inMat = [];
                    for i = 1 : length(minId)
                        %     data1 = Data2{i, 1};   % load(fullfile(inputDir, dirInfo(2*minId(i) - 1).name));
                        %     data2 = Data2{i, 2};   % load(fullfile(inputDir, dirInfo(2*minId(i)).name));
                        %     [xList1, yMat1, lineCoeff1] = do(data1, 2*minId(i)-1);
                        %     [xList2, yMat2, lineCoeff2] = do(data2, 2*minId(i));
                        
                        name = poleName(2*i);
                        
                        poleId = FindPole(Info, name);
                        
                        lineCoeff1 = Data2{i, 1};
                        lineCoeff2 = Data2{i, 2};
                        
                        tempLine = [lineCoeff1; lineCoeff2];
                        Pt = pt(i,:);
                        coeffStackNew = [tempLine(:,1) tempLine(:,2) (tempLine(:,3) + (tempLine(:,1).*cosd(theta).*scale - tempLine(:,2).*sind(theta).*scale).*Pt(1) + (tempLine(:,1).*sind(theta).*scale + tempLine(:,2).*cosd(theta).*scale).*Pt(2))];
                        
                        [xList1, yMat1, yMat2] = preprocess(coeffStackNew, {[1:size(lineCoeff1,1)] [size(lineCoeff1,1) + 1: size(tempLine,1)]});
                        
                        polyCnr = processing(xList1, yMat1, yMat2);
                        ptMean2(i,:) = mean(polyCnr);
                        
                        if 0
                            [in,on] = inpolygon(pix(:,1),pix(:,2),polyCnr(:,1),polyCnr(:,2));
                            pix = pix(in,:);
                        else
                            if old
                                [in,on] = inpolygon(pix0(:,1),pix0(:,2),polyCnr(:,1),polyCnr(:,2));
                            else
                                coeffStackNew2 = repmat(coeffStackNew, size(pix0,1),1);
                                pix00 = repmat([pix0 ones(size(pix0,1),1)], size(coeffStackNew, 1), 1);
                                pix00 = reshape(pix00, size(pix0,1), size(coeffStackNew, 1), 3);
                                pix00 = reshape(permute(pix00, [2 1 3]),[],3);
                                erVec = dot(coeffStackNew2',pix00');
                                erMat = reshape(erVec,size(coeffStackNew, 1),[])';
                                in = sum(erMat'< 0)' == size(coeffStackNew, 1);
                            end
                            
                            
                            
                            pix = pix0(in,:);
                            inMat = [inMat in];
                        end
                        
                        %             if i >= 3
                        %                 inPolyMat{cnt,1+i} = pix;
                        %             end
                        if draw
                            plot(polyCnr(:,1), polyCnr(:,2),'-','LineWidth',1,'Color',colorMat(i,:));drawnow;
                        end
                    end
                    
                    
                    
                    if 0
                        if ~isempty(inPolyMat{cnt,1+3})
                            intersectId = 1;
                            for k = size(inPolyMat,2):-1:2
                                if ~isempty(inPolyMat{end,k})
                                    plot(inPolyMat{end,k}(:,1), inPolyMat{end,k}(:,2),'.k');
                                    
                                    intersectId = k-1;
                                    break
                                end
                            end
                            
                            title(sprintf('%0.6f degree\n%0.6f scale\n%d intersections', theta, scale, intersectId));
                            drawnow;
                            saveas(gcf,fullfile(outDir,sprintf('region_%05d.png',length(dir(fullfile(outDir,'region_*.png')))+1)));
                        end
                        
                    else
                        sumMat = sum(inMat');
                        inIds = find(sumMat == max(sumMat));
                        
                        inPolyMat{cnt,2} = max(sumMat);
                        inPolyMat{cnt,3} = inIds;
                        if draw
                            
                            plot(pix0(inIds,1), pix0(inIds,2),'.k');
                            
                            title(sprintf('%0.6f degree\n%0.6f scale\n%d intersections', theta, scale, max(sumMat)));
                            drawnow;
                            saveas(gcf,fullfile(outDir,sprintf('region_%05d.png',length(dir(fullfile(outDir,'region_*.png')))+1)));
                        end
                    end
                    
                    
                    
                    cnt = cnt + 1;
                    
                end
            end
            if 0
                Id = find(cell2mat(inPolyMat(:,2)) == 5);
                figure,plot(cell2mat(inPolyMat(Id,1)));grid on;
            end
            
            %     save(fullfile(outDir,sprintf('inPolyMat_%05d.mat', length(dir(fullfile(outDir,'inPolyMat_*.mat')))+1)),'inPolyMat');
            
            err2 = ptMean2 - ptMean(minId,:);
            
            
            
            % test = [-0.5791 -4.67; -0.5731 -4.634; -0.5791 -4.681]
            %     test = [pix0(inIds,:)];
            %     plot(test(:,1),test(:,2))
            %     r = scale.*[cosd(theta) sind(theta);-sind(theta) cosd(theta)]
            %     ptNew = (r*pt' + repmat(mean(test,1)',1 ,6))';
            %     figure(1),hold on;plot(ptNew(:,1), ptNew(:,2),'*k');
            
            Id1 = find(cell2mat(inPolyMat(:,2)) == max(cell2mat(inPolyMat(:,2))));
            test = [pix0(inPolyMat{Id1(1),3},:)];
            theta0 = inPolyMat{Id1(1),1}(1);
            scale0 = inPolyMat{Id1(1),1}(2);
            r = scale0.*[cosd(theta0) sind(theta0);-sind(theta0) cosd(theta0)];
            ptNew = (r*pt' + repmat(mean(test,1)',1 ,6))';
            if draw
                figure(1),hold on;plot(ptNew(:,1), ptNew(:,2),'*k');
            end
            %     save(fullfile(outDir,sprintf('inPolyMat_%05d.mat', length(dir(fullfile(outDir,'inPolyMat_*.mat')))+1)),'inPolyMat');
%             saveas(gcf,fullfile(outDir,sprintf('transformed_%05d.fig',length(dir(fullfile(outDir,'transformed_*.fig')))+1)));
        end
        
    end
    ofst = 10;
    figure(1);subplot(1,4,2);hold on;plot(goldenPose(:,2), goldenPose(:,4) + ofst,'xb','MarkerSize',1,'LineWidth',1)
    
    aa = cell2mat(inPolyMat(:,2));aaa = aa(:,1)./aa(:,2); aaaMat = reshape(aaa,length(timeStampList),[]);figure,plot(sum(aaaMat));
    figure(1)
    saveas(gcf,fullfile(outDir,sprintf('transformed_%08d_%08d.fig',1000.*round(theta,3), timeStamp)));
    
    save(fullfile(outDir,sprintf('inPolyMat_%05d.mat', length(dir(fullfile(outDir,'inPolyMat_*.mat')))+1)),'inPolyMat','timeStampList','iii','thetaList');
    
    aa = cell2mat(inPolyMat(:,2));aaa = aa(:,1)./aa(:,2); aaaMat = reshape(aaa,length(timeStampList),[]);figure,plot(sum(aaaMat));bbbMat = [thetaList(1:iii);aaaMat];bbbMat = [[1 timeStampList]' bbbMat];
    thetaListMat = repmat(thetaList(1:iii), length(timeStampList),1);A = (aaaMat == 1).*aaaMat.*thetaListMat;
end

aa = cell2mat(inPolyMat(:,2));aaa = aa(:,1)./aa(:,2); aaaMat = reshape(aaa,length(timeStampList),[]);figure,plot(sum(aaaMat));bbbMat = [thetaList(1:iii);aaaMat];bbbMat = [[1 timeStampList]' bbbMat];
thetaListMat = repmat(thetaList(1:iii), length(timeStampList),1);A = (aaaMat == 1).*aaaMat.*thetaListMat;
timeStampMat = repmat(timeStampList', 1, iii);
hitMat = zeros([size(aaaMat) 2]);

for k = 1 : size(inPolyMat,1)
    tempPolyMat = inPolyMat(k,:);
    frameInd = find(ismember(timeStampList, tempPolyMat{1,1}(2)));
    thetaInd = find(ismember(thetaList(1:iii), tempPolyMat{1,1}(1)));
    hitMat(frameInd, thetaInd,2) = tempPolyMat{2}(2);
    if diff(tempPolyMat{2}) == 0 % || tempPolyMat{2}(1) >=4
        hitMat(frameInd, thetaInd,1) = tempPolyMat{2}(1); %1; % length(tempPolyMat{3});
    end
end


polyMat = cell(length(timeStampList),2);
for p = 1 : size(inPolyMat,1)
    temp = inPolyMat(p,:);
    frameId = find(ismember(timeStampList, temp{1}(2)));
    polyMat{frameId,1} = timeStampList(frameId);
    if diff(temp{2}) == 0 %|| temp{2}(1) >=4
        polyMat{frameId,2} = [polyMat{frameId,2}; temp{4}];
    end
end


figure,hold on;axis equal;grid on;
MedianMinMax = [];
validId = [];
for pp = 1 : size(polyMat,1)
    try
        plot(polyMat{pp,2}(:,1), polyMat{pp,2}(:,2),'.k');
        pos(pp,:) = median(polyMat{pp,2},1);
        [~,minZId] = min(polyMat{pp,2}(:,2));
        [~,maxZId] = max(polyMat{pp,2}(:,2));
        MedianMinMax(pp,:) = [timeStampList(pp) median(polyMat{pp,2},1) polyMat{pp,2}(minZId,:) polyMat{pp,2}(maxZId,:)];
        validId = [validId; pp];
    catch
        dghl = 1;
    end
end
goldenPose0 = goldenPose;
% goldenPose0(:,[4]) = goldenPose0(:,[4]) + pos(1,2);
plot(goldenPose0(validId,2), goldenPose0(validId,4)+0,'xr');
plot(pt(:,1),pt(:,2),'ob');




ac = hitMat(:,:,1);
[acM, acId] = max(ac');
thetaMax = thetaList(acId)';

for jkk = 1 : length(acId)
    ids = find(hitMat(jkk,:) == acM(jkk));
    thetaMax2(jkk,1) = mean(thetaList(ids));
end


figure,surface(-thetaListMat, timeStampMat, immultiply(hitMat(:,:,1), hitMat(:,:,2) > 0));
hold on;plot3(thetaMax2,timeStampList,repmat(30,length(timeStampList),1),'*r')

figure,surface(thetaListMat(:,200:end), timeStampMat(:,200:end), immultiply(hitMat(:,200:end,1), hitMat(:,200:end,2) == 3));


if 0
    fid1 = fopen(fullfile(outDir,'polygon.txt'),'w');%????
    for ij = 1 : size(inPolyMat, 1)
        fprintf(fid1,sprintf('timeStamp: %d, theta: %0.5f\n',inPolyMat{ij,1}(2),inPolyMat{ij,1}(1)));
        for ji = 1 : size(inPolyMat{ij,4},1)
            fprintf(fid1,sprintf('%0.5f, %0.5f',inPolyMat{ij,4}(ji,1), inPolyMat{ij,4}(ji,2)));
            fprintf(fid1,'\n');
        end
        fprintf(fid1,'\n\n');
    end
    fclose(fid1);
else
    
    
    for ij = 1 : size(inPolyMat, 1)
        fid1 = fopen(fullfile(outDir,sprintf('081500_%04d_0p48.txt',inPolyMat{ij,1}(2))),'w');%????
        %         fprintf(fid1,sprintf('timeStamp: %d, theta: %0.5f\n',inPolyMat{ij,1}(2),inPolyMat{ij,1}(1)));
        for ji = 1 : size(inPolyMat{ij,4},1)
            fprintf(fid1,sprintf('%0.5f, %0.5f',inPolyMat{ij,4}(ji,1), inPolyMat{ij,4}(ji,2)));
            fprintf(fid1,'\n');
        end
        %         fprintf(fid1,'\n\n');
        fclose(fid1);
    end
    
    
end
end

function [xList, yMat, lineCoeff] = do(data1, ct, which2show)
global sampleRange draw

[idxCell1, idx1] = splitIndex2(find(data1(:,1) ~= 0));

% figure,hold on;
cnt = 1;
xList = linspace(sampleRange(1), sampleRange(2), sampleRange(3));
yMat = [];
if draw
    figure(1),subplot(1,4,1);
end
for i = 1 : size(idxCell1{1},1)
    if 1 % length(idxCell1{i}) > 1
        %         cnr = data1(idxCell1{i},2:3);
        %         %         plot(cnr(:,1), cnr(:,2),'-b');drawnow;
        %         [~,~,VV] = svd([cnr';ones(1,size(cnr,1))]');
        %         initLine = VV(:,end)';
        %         linek = [initLine(1)/initLine(3) initLine(2)/initLine(3) 1];
        %         [linek*[cnr';ones(1,size(cnr,1))]];
        %         linek =  linek./norm(linek(1:2));
        %         err = dot(repmat(linek,size(cnr,1),1)',[cnr ones(size(cnr,1),1)]')';
        linek = data1(i,:);
        lineCoeff(cnt,:) = linek;
        
        len1 = 80;
        len2 = 100;
        
        %         plot(curBodyOrigUncertPolygonBcsKey(1,[1:end 1]),curBodyOrigUncertPolygonBcsKey(3,[1:end 1]),'-xb');axis equal;
        %         hold on;line([(-linek(2)*len1-linek(3))/linek(1) (-linek(2)*len2-linek(3))/linek(1)]',[len1 len2]','Color',[1 0 0],'LineWidth',1);
        yList = (-linek(1).*xList - linek(3))./linek(2);
        yMat = [yMat; yList];
        if cnt == which2show
            if draw
                if 1
                    if mod(ct,2)==0
                        hold on;plot(xList, yList,'-m');
                    else
                        hold on;plot(xList, yList,'-r');
                    end
                else
                    if mod(ct,2)==0
                        hold on;plot(xList, yList,'-g');
                    else
                        hold on;plot(xList, yList,'-c');
                    end
                end
            end
        end
        drawnow;
        points2 = lineToBorderPoints(linek, [110 20]);
        %         line(points2(:, [1,3])', points2(:, [2,4])');
        
        cnt = cnt + 1;
    end
    
end

if 0
    mi = min(yMat(id1,:));
    ma = max(yMat(id2,:));
    
    idd = find(ma < mi);
    Color = rand(1,3);
    plot(xList(idd), mi(idd),'-','LineWidth',1,'Color',colorMat(ind,:));
    plot(xList(idd), ma(idd),'-g','LineWidth',1,'Color',Color);
    plot(xList(idd), ma(idd),'-','LineWidth',1,'Color',colorMat(ind,:));
    
end

fgkjl = 1;

if 0
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

end

function [polyCnr, lines] = processing(xList, yMat1, yMat2, extenId)

global sampleRange colorMat


% xList = linspace(sampleRange(1), sampleRange(2), sampleRange(3));


if size(yMat1,1) > 1
    mi = min(yMat1);
    ma = max(yMat2);
    Idd{1,1} = find(ma < mi);
    Idd{1,2} = [mi; ma];
    
    mi = min(yMat1);
    ma = max(yMat2);
    Idd{2,1} = find(ma > mi);
    Idd{2,2} = [mi; ma];
    
    mi = max(yMat1);
    ma = min(yMat2);
    Idd{3,1} = find(ma < mi);
    Idd{3,2} = [mi; ma];
    
    mi = max(yMat1);
    ma = min(yMat2);
    Idd{4,1} = find(ma > mi);
    Idd{4,2} = [mi; ma];
    
    for i = 1:length(Idd)
        
        if length(Idd{i,1}) > 0 && length(Idd{i,1}) < length(xList) - 10 && max(diff(Idd{i,1})) == 1
            mi = Idd{i,2}(1,:);
            ma = Idd{i,2}(2,:);
            idd = Idd{i};
            break;
        end
        
    end
    
    
else
    mi = (yMat1);
    ma = (yMat2);
    Idd{1,1} = find(ma < mi);
    Idd{1,2} = [mi; ma];
    Idd{1,3} = mean(yMat1(find(ma < mi)));
    if isnan(Idd{1,3})
        Idd{1,3} = -inf;
    end
    
    mi = (yMat1);
    ma = (yMat2);
    Idd{2,1} = find(ma > mi);
    Idd{2,2} = [mi; ma];
    Idd{2,3} = mean(yMat1(find(ma > mi)));
    if isnan(Idd{2,3})
        Idd{2,3} = -inf;
    end
    %     mi = (yMat1);
    %     ma = (yMat2);
    %     Idd{3,1} = find(ma < mi);
    %     Idd{3,2} = [mi; ma];
    %
    %     mi = (yMat1);
    %     ma = (yMat2);
    %     Idd{4,1} = find(ma > mi);
    %     Idd{4,2} = [mi; ma];
    
    
    if 0
        for i = 1:length(Idd)
            
            if length(Idd{i,1}) > 0 && length(Idd{i,1}) < length(xList) - 10 && max(diff(Idd{i,1})) == 1
                mi = Idd{i,2}(1,:);
                ma = Idd{i,2}(2,:);
                idd = Idd{i};
                break;
            end
            
        end
    else
        
        if Idd{1,3} > Idd{2,3}
            mi = Idd{1,2}(1,:);
            ma = Idd{1,2}(2,:);
            idd = Idd{1,1};
        else
            mi = Idd{2,2}(1,:);
            ma = Idd{2,2}(2,:);
            idd = Idd{2,1};
            
        end
    end
    
end

% [indexCell,idx] = splitIndex2(idd);

% if max(diff(idd))



%
% if isempty(idd)
%
% mi = max(yMat1);
% ma = min(yMat2);
%
%  idd = find(ma > mi);
%
% elseif length(idd) == length(ma)
%
%
%
% end
%
% Color = rand(1,3);

% figure,hold on;plot(xList(idd), mi(idd),'-','LineWidth',1,'Color',[1 0 0]);
% % plot(xList(idd), ma(idd),'-g','LineWidth',1,'Color',Color);
% plot(xList(idd), ma(idd),'-','LineWidth',1,'Color',[0 0 1]);

ploygon = [[xList(idd) xList(idd)];[mi(idd) ma(idd)]]';
polyCnr = FindPolyon(ploygon);

% figure,
% plot(polyCnr(:,1), polyCnr(:,2))

if exist('extenId','var')
    if 0
        [xGrid, yGrid] = meshgrid(min(polyCnr(:,1)):0.1:max(polyCnr(:,1)),min(polyCnr(:,2)):0.1:max(polyCnr(:,2)));
        pix0 = [xGrid(:) yGrid(:)];
        [in,on] = inpolygon(pix0(:,1),pix0(:,2),polyCnr(:,1),polyCnr(:,2));
        pix = mean(pix0(in,:));
    end
    lines = extenId{1};
    %     er = dot(lines',repmat([pix 1],size(lines,1),1)');
    %
    %     flag = find(er>0);
    %     lines(flag,:) = -lines(flag,:);
else
    lines = [];
end
end
function [xList, yMat1, yMat2] = preprocess(lineStack, extenId)

global sampleRange1


xList = linspace(sampleRange1(1), sampleRange1(2), sampleRange1(3));


yMat = [];
for i = 1 : size(lineStack,1)
    
    linek = lineStack(i,:);
    yList = (-linek(1).*xList - linek(3))./linek(2);
    yMat = [yMat; yList];
    
    
end


yMat1 = yMat(extenId{1},:);
yMat2 = yMat(extenId{2},:);

end
function Info = readConfig(accFilePath, str)

[configFileFid, errMsg] = fopen(accFilePath);
if (configFileFid < 0)
    error('Cannot open %s for read: %s', accFilePath, errMsg);
end
% gVecBuf = [];
% inReadgVec = false;
% flagg=0;

cnt = 0;
Info = {};
while ~feof(configFileFid)
    lineBuf = strtrim(fgetl(configFileFid));
    if length(lineBuf) > 0
        %         if strcmp(lineBuf(1:4), '0815')
        if strcmp(lineBuf(1:4), str)
            cnt = cnt + 1;
            Info{cnt,1} = lineBuf(8:end);
            Info{cnt,2} = [];
            continue;
        end
        Info{cnt,2} = [Info{cnt,2}; str2double((lineBuf(6:end)))];
        
        
    end
    
    
end
% G=1;
% g=gVec;
fclose(configFileFid);


end
function poleId = FindPole(Info, name)

for i = 1 : size(Info,1)
    temp = Info(i,:);
    tp = splitIndex2(find(ismember(name{1}, temp{1}))');
    %     tp = splitIndex2(find(ismember(temp{1}, name{1}))');
    if length(tp{1}) == min([length(name{1}) length(temp{1})])
        poleId = [i temp{3}];
        break;
    end
    
end

end
function rng = DetermineRng(line1, line2, ptUse)

z1 = (-line1(1)*ptUse(1) - line1(3))/line1(2);
z2 = (-line2(1)*ptUse(1) - line2(3))/line2(2);
minz = min([z1 z2]);
maxz = max([z1 z2]);
rng = [minz maxz] - ptUse(2) ;

end


function [Rt, pose] = OptScale(line1, line, ptUse)

scaleRng = scale0 + [-10000 10000];

[optimScale, ~, exitFlag] = fminbnd(@ObjectiveFunction, scaleRng(1), scaleRng(2), optimset('TolX',0.0000001,'Display','iter'));

optimScale = scale0;

errOpt = ObjectiveFunction(optimScale);


Rt = [r optimScale*t];
pose = [rodrigues(Rt(1:3,1:3)); Rt(1:3,4)];

    function errl = ObjectiveFunction(scale)
        
        reproj_l =  reprojErrLines(r, scale*t, Ws,We, xs,xe);
        errl = sqrt(sum(sum(reproj_l.*reproj_l))/2/(size(xs,2)/1));
        
    end
end
function intersectPtt = LineIntersection(lineCoeff)
[~,~,V] = svd(lineCoeff);
pt = V(:,end);
intersectPtt = [pt(1)./pt(end); pt(2)./pt(end)]';
err = [lineCoeff*[intersectPtt';1]];
end
function Data = ArrangeData(accFilePath)

global v2c rMatNew

try
    [data, Info, pix] = ReadFile(accFilePath);
catch
    [data, Info, pix] = ReadFile2(accFilePath);
end
widen = 0;  -30;1;5;
scale = 4;
expand = 5;  35;


K = eye(3);
K(1,1) = 2.0003201001638317e+03;
K(1,3) = 9.6889274597167969e+02;
K(2,2) = 2.0003201001638317e+03;
K(2,3) = 5.4882026672363281e+02;

invK = inv(K);

frames = unique(data(find(data(:,1) ~= 0),1));

Data = {};
pt0 = [0 0 1];
for i = 1 : length(frames)
    ind = find(data(:,1) == frames(i));
    tempData_ = data(ind,:);
    tempPix_ = pix(ind,:);
    tempData = tempData_(tempData_(:,2) ~= 0,:);
    tempPix = tempPix_(tempData_(:,2) ~= 0,:);
    Ind = ind(tempData_(:,2) ~= 0,:);
    Data{i,1} = frames(i);
    Data{i,2} = [];
    Data{i,3} = [];
    Data{i,4} = [];
    Data{i,5} = [];
    Data{i,6} = [];
    for j = 1 : size(tempData,1)
        pt1 = [tempData(j,2) 1 1];
        pt1_bak = pt1;
%         if mod(j,2)~=0
        if pt1_bak(1) > 0
            try
                pix1 = [tempPix(j,2) + widen tempPix(j,4) 1]';
                pix1_0 = pix1;
                pt2d = Orig2Rect(pix1(1:2)', K, K, v2c',zeros(5,1));
                pt2dd = Orig2Rect(pt2d, K, K, rotz(-2)*rMatNew',zeros(5,1));
                
                if 0
                    img1 = imread('D:\Auto\data5\081500_all\imagel5460_081500_rectify.jpg');
                    
                    img1 = imread('D:\Auto\data5\081500_all\imagel5300_081500_rectify.jpg');
                    pt2dd = Orig2Rect(pt2d, K, K, rMatNew',zeros(5,1));
                    img11 = ImgTransform(img1, K, rMatNew');
                    figure,imshow(img1);hold on;plot(pt2d(:,1), pt2d(:,2),'or')
                    figure,imshow(img11);hold on;plot(pt2dd(:,1), pt2dd(:,2),'or')
                    testBirdView2(K,img11)
                end
                
                
                pt2ddd = pt2dd;
                
                if mod(j, 2) ~= 0
                    pt2ddd(1) = pt2ddd(1) - expand;
                else
                    pt2ddd(1) = pt2ddd(1) + expand;
                end
                
                pix1 = [pt2ddd 1]';

                
                
            catch
                pix1 = [tempPix(j,2) + widen 100 1]';
            end
% %             pix1 = [tempPix(j,2) + widen tempPix(j,4) 1]';
            pt1New = invK*pix1;
            pt1(1) = pt1New(1);
        else
            try
                pix1 = [tempPix(j,2) - widen/scale tempPix(j,4) 1]';
                pix1_0 = pix1;
                pt2d = Orig2Rect(pix1(1:2)', K, K, v2c',zeros(5,1));
                pt2dd = Orig2Rect(pt2d, K, K, rotz(-2)*rMatNew',zeros(5,1));
               
                if 0
                    img1 = imread('D:\Auto\data5\081500_all\imagel5460_081500_rectify.jpg');
                    
                    img1 = imread('D:\Auto\data5\081500_all\imagel5300_081500_rectify.jpg');
                    pt2dd = Orig2Rect(pt2d, K, K, rMatNew',zeros(5,1));
                    img11 = ImgTransform(img1, K, rMatNew');
                    figure,imshow(img1);hold on;plot(pt2d(:,1), pt2d(:,2),'or')
                    figure,imshow(img11);hold on;plot(pt2dd(:,1), pt2dd(:,2),'or')
                    testBirdView2(K,img11)
                end
                
                pt2ddd = pt2dd;
                
                if mod(j, 2) ~= 0
                    pt2ddd(1) = pt2ddd(1) - expand;
                else
                    pt2ddd(1) = pt2ddd(1) + expand;
                end
                
                
                
                pix1 = [pt2ddd 1]';
                
                
            catch
                pix1 = [tempPix(j,2) - widen/scale 100 1]';
            end
            
            pt1New = invK*pix1;
            pt1(1) = pt1New(1);
            
        end
        
        
        lineVec = cross(pt0, pt1);
        lineVec = lineVec./norm(lineVec(1:2));
        pt1_check = pt1;
        if mod(j,2)~=0
            if pt1(1) > 0
                pt1_check(2) = pt1_check(2) - 0.5;
                if dot(pt1_check, lineVec) > 0
                    lineVec = -lineVec;
                end
            else
                pt1_check(2) = pt1_check(2) + 0.5;
                if dot(pt1_check, lineVec) > 0
                    lineVec = -lineVec;
                end
            end
            Data{i,2} = [Data{i,2}; lineVec];
            Data{i,5} = [Data{i,5}; pix1(1)];
            Data{i,4} = [Data{i,4}; Info(Ind(j),:)];
        else
            
            if pt1(1) > 0
                pt1_check(2) = pt1_check(2) + 0.5;
                if dot(pt1_check, lineVec) > 0
                    lineVec = -lineVec;
                end
            else
                pt1_check(2) = pt1_check(2) - 0.5;
                if dot(pt1_check, lineVec) > 0
                    lineVec = -lineVec;
                end
            end
            Data{i,3} = [Data{i,3}; lineVec];
            Data{i,6} = [Data{i,6}; pix1(1)];
        end
    end
    
    
    
    
end

end
function [data, Info, pix] = ReadFile(accFilePath)

[configFileFid, errMsg] = fopen(accFilePath);
if (configFileFid < 0)
    error('Cannot open %s for read: %s', accFilePath, errMsg);
end
% gVecBuf = [];
% inReadgVec = false;
% flagg=0;


K = eye(3);
K(1,1) = 2.0003201001638317e+03;
K(1,3) = 9.6889274597167969e+02;
K(2,2) = 2.0003201001638317e+03;
K(2,3) = 5.4882026672363281e+02;


cnt = 1;
Info = {};
% pix
while ~feof(configFileFid)
    lineBuf = strtrim(fgetl(configFileFid));
    if length(lineBuf) > 0
        id = find(lineBuf == 'p');
        if 0 % strcmp(lineBuf(id:end),'pole9l') || strcmp(lineBuf(id:end),'pole11l')
            continue;
        end
        if ~isempty(id)
            data(cnt,:) = str2double(strsplit(lineBuf(1:id-3),','));
            pix(cnt,:) = data(cnt,:);
            if data(cnt,2) ~= 0
                pix(cnt,2) = K(1,1)*data(cnt,2) + K(1,3);
            end
            Info{cnt,1} = lineBuf(id:end);
            cnt = cnt + 1;
        end
    end
    
    
end
% G=1;
% g=gVec;
fclose(configFileFid);

data(:,2) = data(:,2).*1;  0.82;

end
function [data, Info, pix] = ReadFile2(accFilePath)

[configFileFid, errMsg] = fopen(accFilePath);
if (configFileFid < 0)
    error('Cannot open %s for read: %s', accFilePath, errMsg);
end
% gVecBuf = [];
% inReadgVec = false;
% flagg=0;


K = eye(3);
K(1,1) = 2.0003201001638317e+03;
K(1,3) = 9.6889274597167969e+02;
K(2,2) = 2.0003201001638317e+03;
K(2,3) = 5.4882026672363281e+02;


cnt = 1;
Info = {};
% pix
while ~feof(configFileFid)
    lineBuf = strtrim(fgetl(configFileFid));
    if length(lineBuf) > 0
        id = find(lineBuf == 'p');
        if 0 % strcmp(lineBuf(id:end),'pole9l') || strcmp(lineBuf(id:end),'pole11l')
            continue;
        end
        if ~isempty(id)
            data(cnt,:) = str2double(strsplit(lineBuf(1:id-3),','));
            pix(cnt,:) = data(cnt,:);
            if data(cnt,2) ~= 0
                pix(cnt,2) = K(1,1)*data(cnt,2) + K(1,3);
            end
            Info{cnt,1} = lineBuf(id:end);
            cnt = cnt + 1;
        end
    end
    
    
end
% G=1;
% g=gVec;
fclose(configFileFid);

data(:,2) = data(:,2).*1;  0.82;

end
function data = ReadPole(accFilePath)

[configFileFid, errMsg] = fopen(accFilePath);
if (configFileFid < 0)
    error('Cannot open %s for read: %s', accFilePath, errMsg);
end


data = {};
cnt = 1;
while ~feof(configFileFid)
    lineBuf = strtrim(fgetl(configFileFid));
    
    if cnt == 1
        cnt = cnt + 1;
        continue
    end
    id = find(lineBuf == 'r' | lineBuf == 'l');
    
    if strcmp(lineBuf(id),'l')
        data{cnt-1,1} = lineBuf(1:id);
        data{cnt-1,2} = str2double(strsplit(lineBuf(id+3:end-2),','));
    else
        data{cnt-1,1} = lineBuf(1:id-1);
        data{cnt-1,2} = str2double(strsplit(lineBuf(id+3:end-2),','));
    end

    
    cnt = cnt+1;
   
    
    
end

fclose(configFileFid);



end
function [indexCell,idx] = splitIndex2(idxx)

idx = [1 find(diff(idxx') ~= 1)+1  numel(idxx')+1];
indexCell = mat2cell(idxx', 1, diff(idx))';     

end
function [normVec, sqrtsumvec2] = NormalizeVector(vec)
if 1
    vec2 = vec.^2;
    sumvec2 = sum(vec2,2);
    sqrtsumvec2 = sqrt(sumvec2);
    if 0
        for j = 1:size(npixels,1)
            ndescrppp(j,:) = npixels(j,:)/norm(npixels(j,:));
        end
    else
        for k = 1 : size(vec,2)
            normVec(:,k) = vec(:,k)./sqrtsumvec2;
        end
    end
else
    [a] = normc(vec')';
    b=a;
    a(a == 0) = 1;
    scal = (vec./a);
    scal
    id = ~isnan(scal);
    idd = sum(id,1) == size(scal,1);   % idd = find(sum(id) == size(scal,1));
    scall = mean(scal(:,idd),2);
    
    sqrtsumvec2 = scall;
    normVec = b;
end
end
function polyCnr = FindPolyon(pt0)

DT = delaunayTriangulation(pt0(:,1),pt0(:,2));
k = convexHull(DT);
polyCnr = DT.Points(k,:);
end