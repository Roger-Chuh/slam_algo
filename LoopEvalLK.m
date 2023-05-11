function LoopEvalLK(vsl)

global TCur_camInit cbcXYZLInit rot_y inverseRot prvBaseLK forceSame doBlur detectCB useFast...
    newLK traceCB shortTrace...
    resolution winx winy saturation ...
    wintx winty spacing boundary boundary_t ...
    Nmax thresh levelmin levelmax ThreshQ ...
    N_max_feat method...
    textureFile b2c angSize probPath inputObj
 
if 1
    resolution = 0.01; 0.001; 0.0001; 			% Desired tracking accuracy in pixels
    winx = 1; winy = 1;			% Window half-size for selection; effective size = (1+2*winx, 1+2*winy)
    % THIS IS A CRUCIAL DESIGN PARAMETER 
    saturation = 7000; 			% Image saturation level (not necessary if variable 'method' chosen to be 0
    wintx = 3; 7; 3; 7; 3; 7; winty = 3; 7; 3; 7; 3; 7; 			% Window half-size for tracking; effective size = (1+2*wintx, 1+2*winty)
    spacing = 2;				% min spacing between 2 features (in pixel).
    boundary = 1;				% discards features selected too close to the boundary of the image
    boundary_t = 1;				% rejects tracked features too close to boundary
    Nmax = 1000;                            % maximum number of features selected
    thresh = 0.01; 				% Threshold for feature selection
    % THIS IS A CRUCIAL DESIGN PARAMETER: low threshold = many features selected,
    % greater errors; high threshold = fewer features selected, better quality.
    levelmin = 2; 0; 2; 0; 				% lower level in the pyramid
    levelmax = 4; 6;2;				% higher level in the pyramid: large motions require more levels.
    % Inter-frame motions within 1 pixel do not require a pyramid (levelmax=0).
    ThreshQ = 0.01;				% Threshold for rejecting a feature
    % THIS IS A CRUCIAL DESIGN PARAMETER: low threshold = many features kept
    % through the track
    N_max_feat = 1500;			% Minimum space reserved for feature storage
    method = 1;
    
end


b2c = [eye(3) [10 45 -170.2]';0 0 0 1];

%% 20200918 test b2c calibration 
b2c = [rodrigues([0.02; 0.003; 0.02]) [10 45 -170.2]';0 0 0 1];

textureFile = 'C:\Users\rongjiezhu\Desktop\bg.jpg';
% textureFile = 'C:\Users\rongjiezhu\Desktop\bg17.jpg';
% textureFile = 'C:\Users\rongjiezhu\Desktop\bg6.jpg';
textureFile = 'C:\Users\rongjiezhu\Desktop\bg10.jpg';


shortTrace = false; true; 
newLK =  false; true; false; true;

inverseRot = false; true; false;

prvBaseLK = true; false; true; false; true; false; true;
forceSame = false; 
doBlur = false; true; false; 
detectCB = false; true; false; true; false; true;
traceCB = false; 
if detectCB
%     prvBaseLK = false;
    doBlur = true;
end

useFast = true; false; true; false; 

rot_y = true; false;

TCur_camInit = [];
cbcXYZLInit = [];

selfMade = true; false; true; false; true;
angSize = 1; 3; 1; 3; 1; 3; 3; 1; 3; 

if selfMade
    if 1
        inputObj = 'C:\Users\rongjiezhu\Desktop\÷Ï\01.obj';
        obj_test = MR_obj_read(inputObj);
        
        obj_test.V = 3.*obj_test.V;
        obj_test.V = (rotx(90)*obj_test.V')';
        obj_test.V = (roty(180)*obj_test.V')';
        depthThr = [3000 -100000000 -200 -1500 1500]; % [minZ minY maxY minX maxX]
    elseif 1
%         obj_test = MR_obj_read('E:\bk_20180627\SLAM\slam_algo\Renderer\data\cube5.obj');
%         obj_test = MR_obj_read('E:\bk_20180627\SLAM\slam_algo\Renderer\data\cube5_2.obj');
inputObj = 'E:\bk_20180627\SLAM\slam_algo\Renderer\data\plane5.obj';
        obj_test = MR_obj_read(inputObj);
        obj_test.V = 1500.*obj_test.V;
        obj_test.V(:,2) = obj_test.V(:,2) - max(obj_test.V(:,2)); %1500.*obj_test.V;
%         figure, fig_3D3_pair(obj_test.V', [])
        depthThr = [100];
        depthThr = [3000 -100000000 -200 -1500 1500];
    else
        inputObj = 'data/ball.obj';
        obj_test = MR_obj_read(inputObj);
         obj_test.V = 1500.*obj_test.V;
        obj_test.V(:,2) = obj_test.V(:,2) - max(obj_test.V(:,2)); %1500.*obj_test.V;
%         figure, fig_3D3_pair(obj_test.V', [])
        depthThr = [100];
        depthThr = [30 -100000000 200 -150000 150000];
        
        
    end
    
    % % RRR = roty(angList(i));
    % % T_delt = [RRR [0;0;0]; 0 0 0 1];
    % % T1 = b2c*T_delt*T0;
    % % T1R = L2R*T1;
%     depthThr = [-1000000000 -100000000 -200]; % [minZ minY maxY]
%     depthThr = [3000 -100000000 -200 -1500 1500]; % [minZ minY maxY minX maxX]
%     depthThr = [3000 -100000000 -200 -5500 5500];
%     depthThr = [100];
    t = [0;200;0];
    T0 = [eye(3) t; 0 0 0 1];

    % % [obj_test2, inlierId] = procObj(obj_test, depthThr, T1(1:3,1:3), T1(1:3,4));
    % [xyzOrig22, imgCur022, depthMap22, Vcam22] = render_fcn(obj_test2, T1(1:3,1:3), T1(1:3,4));
else
    inputObj = 'data/ball.obj';
end
    
    


thetaList = 0 : 5 : 365;
if rot_y
    %     thetaList = [45 [[-10 :0.2: -0.2] [0.2 : 0.2 : 10]] + 45 + 0];
    %     thetaList = [0 [[-40 : 5: 5] [5 : 5 : 40]] + 0 + 0];
    thetaList = [0 [[-45 : 1: 1] [1 : 1 : 45]] + 0 + 0];
    thetaList = [0 [[-66 : 2: -2] [2 : 2 : 24]] + 0 + 0];
    thetaList1 = [-66 : 2: -2];
    thetaList1 = [-34 : 2: -2];
    thetaList1 = [-78 : 2: -2];
    thetaList2 = [0 : 34/length(thetaList1): 34];
    
    if selfMade
        % test ros ÷Ï01
        thetaList1 = [-135 : 2: -2];
        thetaList2 = [0 : 135/length(thetaList1): 135];
    end
    thetaList = [0  thetaList1 thetaList2];
    
%     thetaList = [0 thetaList1 ];
%     thetaList = [-180:180];
    
    %     thetaList = [0 [[-46 : 2: 2] [2 : 2 : 4]] + 0 + 0];
else
    thetaList = [90 [[-10 :0.2: -0.2] [0.2 : 0.2 : 10]] + 0 + 0];
end
X_err = []; Y_err = []; cnt = 1;validCnt  = [];MappingErr = {};
X_err_inv = []; Y_err_inv = []; cnt = 1;validCnt  = [];MappingErr_inv = {};
VecAngMatCW = {}; VecAngMatCCW = {};
t00 = [-3000;-1500;0];
t00 = [-3000;0;0];

winW = vsl.featPtTracker.configParam.win_width;
winH = vsl.featPtTracker.configParam.win_height;
pyrL = vsl.featPtTracker.configParam.max_level;

t0 = datetime('now','Format','dd-MMM-y HH:mm:ss');t = datestr(t0);t1 = yyyymmdd(t0);t2 = strsplit(t,' ');
tt = strcat(num2str(t1),'_',t2{2});tt(find(tt == ':')) = '';
MakeDirIfMissing(fullfile(probPath, tt));

congfigInfo.textureFile = textureFile;
congfigInfo.lkConfig.winW = winW;
congfigInfo.lkConfig.winH = winH;
congfigInfo.lkConfig.pyrL = pyrL;
congfigInfo.angSize = angSize;
congfigInfo.inputObj = inputObj;
congfigInfo.thetaList = thetaList;

save(fullfile(probPath,tt,'congfigInfo.mat'), 'congfigInfo');

for i = 1 : length(thetaList)
% for i = 60:80
    if selfMade
        RRR = roty(thetaList(i));
        T_delt = [RRR [0;0;0]; 0 0 0 1];
        T1 = b2c*T_delt*T0;
%         T1 = eye(4)*T_delt*T0;
        % T1R = L2R*T1;
        % depthThr = [-1000000000 -100000000 -200];
        if 0
            [obj_test2, inlierId] = procObj(obj_test, depthThr, T1(1:3,1:3), T1(1:3,4));
        end
        T11 = [RRR t00;0 0 0 1];
        obj_test3 = obj_test;
        obj_test3.V = (RRR*obj_test.V')';
        obj_test3.V(:,1) = obj_test3.V(:,1) - mean(obj_test3.V(:,1));
%         [obj_test2, inlierId] = procObj(obj_test, depthThr, T11(1:3,1:3), [0;0;0]);
        [obj_test2, inlierId] = procObj(obj_test3, depthThr, eye(3), [0;0;0]);
%         [obj_test2, inlierId] = procObj2(obj_test3, depthThr, eye(3), [0;0;0]);
%         obj_test2.V(:,2) = obj_test2.V(:,2) - mean(obj_test2.V(:,2));
        T11 = [eye(3) t00;0 0 0 1];

    end
    
    try
        if ~selfMade
            inverseRot = false;
            TCur_cam_stack = [];
            [x_err, y_err, x_err_stack, y_err_stack, TCur_cam, cbcXYZL, mappingErr, TCur_cam_stack, ptTrace,~,~,~,~,gtRemapErr] = testRender(vsl, thetaList(i), [], []);
            inverseRot = true;
            [x_err_inv, y_err_inv, x_err_stack_inv, y_err_stack_inv, TCur_cam_inv, cbcXYZL_inv, mappingErr_inv, ~, ~, vecAngMatCW, vecAngMatCCW, vecAngMatCW_y, vecAngMatCCW_y,gtRemapErr_inv] = testRender(vsl, thetaList(i), TCur_cam_stack, ptTrace);
        else
            inverseRot = false;
            TCur_cam_stack = [];
            [x_err, y_err, x_err_stack, y_err_stack, TCur_cam, cbcXYZL, mappingErr, TCur_cam_stack, ptTrace,~,~,~,~,gtRemapErr] = testRender2(vsl,obj_test2, T11, thetaList(i), [], []);
            inverseRot = true;
            [x_err_inv, y_err_inv, x_err_stack_inv, y_err_stack_inv, TCur_cam_inv, cbcXYZL_inv, mappingErr_inv, ~, ~, vecAngMatCW, vecAngMatCCW, vecAngMatCW_y, vecAngMatCCW_y,gtRemapErr_inv] = testRender2(vsl,obj_test2, T11, thetaList(i), TCur_cam_stack, ptTrace);
            
            
        end
        X_err(cnt,:) = x_err;
        Y_err(cnt,:) = y_err;
        X_err_stack{cnt, 1} = x_err_stack;
        Y_err_stack{cnt, 1} = y_err_stack;
        MappingErr{cnt,1} = mappingErr;
        
        
        X_err_inv(cnt,:) = x_err_inv;
        Y_err_inv(cnt,:) = y_err_inv;
        X_err_stack_inv{cnt, 1} = x_err_stack_inv;
        Y_err_stack_inv{cnt, 1} = y_err_stack_inv;
        MappingErr_inv{cnt,1} = mappingErr_inv;
        
        VecAngMatCW{cnt, 1} = mean(vecAngMatCW);
        VecAngMatCW{cnt, 2} = (vecAngMatCW);
        VecAngMatCW{cnt, 3} = mean(vecAngMatCW_y);
        VecAngMatCW{cnt, 4} = (vecAngMatCW_y);
        VecAngMatCW{cnt, 5} = gtRemapErr;
        
        
        
        VecAngMatCCW{cnt, 1} = mean(vecAngMatCCW);
        VecAngMatCCW{cnt, 2} = (vecAngMatCCW);
        VecAngMatCCW{cnt, 3} = mean(vecAngMatCCW_y);
        VecAngMatCCW{cnt, 4} = (vecAngMatCCW_y);
        VecAngMatCCW{cnt, 5} = gtRemapErr_inv;
        
        
        validCnt = [validCnt; i];
        cnt = cnt + 1;
        
    catch
        continue;
    end
     
    if i == 1
        TCur_camInit = TCur_cam;
        cbcXYZLInit = cbcXYZL;
    end
    save(fullfile(probPath,tt,'congfigInfo2.mat'), 'congfigInfo','X_err','Y_err','X_err_inv','Y_err_inv','X_err_stack','Y_err_stack','X_err_stack_inv','Y_err_stack_inv','validCnt');
    copyfile('LoopEvalLK.m', fullfile(probPath,tt));
    copyfile('Renderer\testRender.m', fullfile(probPath,tt));
    copyfile('testRender2.m', fullfile(probPath,tt));
    copyfile('Renderer\render_fcn.m', fullfile(probPath,tt));
    copyfile('Renderer\MR_default_params.m', fullfile(probPath,tt));
    copyfile('procObj.m', fullfile(probPath,tt));
    copyfile('procObj2.m', fullfile(probPath,tt));

end

copyfile('LoopEvalLK.m', fullfile(probPath,tt));
copyfile('Renderer\testRender.m', fullfile(probPath,tt));
copyfile('testRender2.m', fullfile(probPath,tt));
copyfile('Renderer\render_fcn.m', fullfile(probPath,tt));
copyfile('Renderer\MR_default_params.m', fullfile(probPath,tt));
copyfile('procObj.m', fullfile(probPath,tt));
copyfile('procObj2.m', fullfile(probPath,tt));

%                 copyfile(fullfile(pwd,'Tracing\*.m'), fullfile(probPath, 'Tracing'));
%                 copyfile(fullfile(pwd,'Renderer\*.m'), fullfile(probPath, 'Renderer'));
%                 copyfile(fullfile(pwd,'EpilineLK\*.m'), fullfile(probPath, 'EpilineLK'));

% save(fullfile(probPath,tt,'congfigInfo.mat'), 'congfigInfo','X_err','Y_err','X_err_inv','Y_err_inv');

% winW = vsl.featPtTracker.configParam.win_width;
% winH = vsl.featPtTracker.configParam.win_height;
% pyrL = vsl.featPtTracker.configParam.max_level;

trialRng = 1:cnt-1;


figure,subplot(2,2,1);plot((X_err(trialRng,2:end))');title('x p2c');
subplot(2,2,2),plot((Y_err(trialRng,2:end))');title('y p2c');
subplot(2,2,3);plot((mean(X_err(trialRng,2:end), 1)));title('x');
subplot(2,2,4),plot((mean(Y_err(trialRng,2:end), 1)));title('y');
subplot(2,2,3);hold on;plot((mean(X_err_inv(trialRng,2:end), 1)));title('x');%hold on;plot(mean(cell2mat(VecAngMatCW(:,1)))./1000);plot(mean(cell2mat(VecAngMatCCW(:,1)))./1000);legend('CW','CCW','CW ang','CCW ang','Location','southeast');
subplot(2,2,4),hold on;plot((mean(Y_err_inv(trialRng,2:end), 1)));title('y');%hold on;plot(mean(cell2mat(VecAngMatCW(:,3)))./10);plot(mean(cell2mat(VecAngMatCCW(:,3)))./10);legend('CW','CCW','CW ang','CCW ang','Location','southeast');


figure,subplot(2,2,1);plot((X_err(:,:))');title('x');
subplot(2,2,2),plot((Y_err(:,:))');title('y');
subplot(2,2,3);plot(mean(X_err(:,:), 1));title('x'); % hold on;plot(mean(cell2mat(VecAngMatCW(:,1)))./1000);plot(mean(cell2mat(VecAngMatCCW(:,1)))./1000);legend();
subplot(2,2,4),plot(mean(Y_err(:,:), 1));title('y');




gkjb = 1;


if 0
    figure,subplot(2,2,1);plot(cumsum((X_err(:,2:end))'));title('x k2c');
    subplot(2,2,2),plot(cumsum((Y_err(:,2:end))'));title('y k2c');
    subplot(2,2,3);plot(cumsum(mean(X_err(:,2:end), 1)));title('x');
    subplot(2,2,4),plot(cumsum(mean(Y_err(:,2:end), 1)));title('y');
    subplot(2,2,3);hold on;plot(cumsum(mean(X_err_inv(:,2:end), 1)));title('x');legend('CW','CCW');
    subplot(2,2,4),hold on;plot(cumsum(mean(Y_err_inv(:,2:end), 1)));title('y');legend('CW','CCW');
    
    
    trials = 2*floor(size(X_err,1)/2);
    
    
    figure,subplot(2,2,1);plot((X_err(:,2:end))');title('x p2c');
    subplot(2,2,2),plot((Y_err(:,2:end))');title('y p2c');
    subplot(2,2,3);plot((mean(X_err(:,2:end), 1)));title('x');
    subplot(2,2,4),plot((mean(Y_err(:,2:end), 1)));title('y');
    subplot(2,2,3);hold on;plot((mean(X_err_inv(:,2:end), 1)));title('x');%hold on;plot(mean(cell2mat(VecAngMatCW(:,1)))./1000);plot(mean(cell2mat(VecAngMatCCW(:,1)))./1000);legend('CW','CCW','CW ang','CCW ang','Location','southeast');
    subplot(2,2,4),hold on;plot((mean(Y_err_inv(:,2:end), 1)));title('y');%hold on;plot(mean(cell2mat(VecAngMatCW(:,3)))./10);plot(mean(cell2mat(VecAngMatCCW(:,3)))./10);legend('CW','CCW','CW ang','CCW ang','Location','southeast');
 
    
    
    
    figure,subplot(2,2,1);plot((X_err(1:trials/2,2:end))');title('x p2c');
    subplot(2,2,2),plot((Y_err(1:trials/2,2:end))');title('y p2c');
    subplot(2,2,3);plot((mean(X_err(1:trials/2,2:end), 1)));title('x');
    subplot(2,2,4),plot((mean(Y_err(1:trials/2,2:end), 1)));title('y');
    subplot(2,2,3);hold on;plot((mean(X_err_inv(1:trials/2,2:end), 1)));title('x');hold on;plot(mean(cell2mat(VecAngMatCW(1:trials/2,1)))./1000);plot(mean(cell2mat(VecAngMatCCW(1:trials/2,1)))./1000);legend('CW','CCW','CW ang','CCW ang','Location','southeast');
    subplot(2,2,4),hold on;plot((mean(Y_err_inv(1:trials/2,2:end), 1)));title('y');hold on;plot(mean(cell2mat(VecAngMatCW(1:trials/2,3)))./10);plot(mean(cell2mat(VecAngMatCCW(1:trials/2,3)))./10);legend('CW','CCW','CW ang','CCW ang','Location','southeast');
    
    figure,subplot(2,2,1);plot((X_err(trials/2 + 1 :end,2:end))');title('x p2c');
    subplot(2,2,2),plot((Y_err(trials/2 + 1 :end,2:end))');title('y p2c');
    subplot(2,2,3);plot((mean(X_err(trials/2 + 1 :end,2:end), 1)));title('x');
    subplot(2,2,4),plot((mean(Y_err(trials/2 + 1 :end,2:end), 1)));title('y');
    subplot(2,2,3);hold on;plot((mean(X_err_inv(trials/2 + 1 :end, 2:end), 1)));title('x');hold on;plot(mean(cell2mat(VecAngMatCW(trials/2 + 1 :end, 1)))./1000);plot(mean(cell2mat(VecAngMatCCW(trials/2 + 1 :end, 1)))./1000);legend('CW','CCW','CW ang','CCW ang','Location','southeast');
    subplot(2,2,4),hold on;plot((mean(Y_err_inv(trials/2 + 1 :end, 2:end), 1)));title('y');hold on;plot(mean(cell2mat(VecAngMatCW(trials/2 + 1 :end, 3)))./10);plot(mean(cell2mat(VecAngMatCCW(trials/2 + 1 :end, 3)))./10);legend('CW','CCW','CW ang','CCW ang','Location','southeast');
    
    
    frameList = [2 9 size(X_err,2)];
    frameList = [7 9 11];
    figure,subplot(2,2,1);plot(X_err(:,frameList));grid on;title('x CW');legend(num2str(frameList-1));subplot(2,2,2);plot(Y_err(:,frameList));grid on;title('y CW')
    subplot(2,2,3);plot(X_err_inv(:,frameList));grid on;title('x CCW');subplot(2,2,4);plot(Y_err_inv(:,frameList));grid on;title('y CCW')


end




try
    figure,subplot(2,2,1);plot((X_err_inv(:,:))');title('x');
    subplot(2,2,2),plot((Y_err_inv(:,:))');title('y');
    subplot(2,2,3);plot(mean(X_err_inv(:,:), 1));title('x');
    subplot(2,2,4),plot(mean(Y_err_inv(:,:), 1));title('y');
catch
    
    sgbjh = 1;
end

figure,subplot(2,2,1);plot(cell2mat(X_err_stack)');title('x');
subplot(2,2,2),plot(cell2mat(Y_err_stack)');title('y');
subplot(2,2,3);plot(mean(cell2mat(X_err_stack), 1));title('x');
subplot(2,2,4),plot(mean(cell2mat(Y_err_stack), 1));title('y');


figure,hist(cell2mat(MappingErr(2:end,:)),500);


for  j = 1 : size(X_err,1)
    figure, subplot(2,2,1);plot(X_err_stack{j,1}');title('x');subplot(2,2,2);plot(Y_err_stack{j,1}');title(num2str(validCnt(j)));
    subplot(2,2,3);plot(mean(X_err_stack{j,1}));title('x');subplot(2,2,4);plot(mean(Y_err_stack{j,1}));title(num2str(validCnt(j)));drawnow;
end

end