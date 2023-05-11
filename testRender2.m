function [x_err, y_err, x_err_stack, y_err_stack, TCur_cam, cbcXYZL, mappingErr, TCur_cam_stack, ptTrace, vecAngMatCW, vecAngMatCCW, vecAngMatCW_y, vecAngMatCCW_y, gtRemapErr] = testRender2(vsl, obj, T_made,  theta, poseStack, ptTrace0)

global xyzOrig_fixed b2c TCur_camInit cbcXYZLInit rot_y inverseRot prvBaseLK forceSame doBlur detectCB useFast newLK traceCB shortTrace...
    resolution winx winy saturation ...
    wintx winty spacing boundary boundary_t ...
    Nmax thresh levelmin levelmax ThreshQ ...
    N_max_feat method...
    textureFile angSize


% forceSame = false; true; false; true; false; true;
% prvBaseLK = false; true;
[marg, ~, ~, ~] = GetPredictMargin(vsl.featPtTracker);
figOfst = 0; 100;

% rot_y = false;


% shortTrace = 0;

multi_obj = false; true;




vecAngMatCW = [];
vecAngMatCCW = [];
vecAngMatCW_y = [];
vecAngMatCCW_y = []; 
mappingErr = [];
cbcXYZL = [];
draw = 0;

if 0
    b2c = [rotz(1)*roty(12)*rotx(13) [10; 45; -170.2]; 0 0 0 1];
    % b2c = [rotz(1)*roty(12)*rotx(13) [10; 45; -270.2]; 0 0 0 1];
    % b2c = [rotz(1)*roty(12)*rotx(13) [10; 45; 1370.2]; 0 0 0 1];
    
    b2c(1:3,1:3) = eye(3);
    b2c(1,4) = 0;
end
% b2c = eye(4);
% b2c(1:3,4) = [0 0 0]';
if 0 % ~isempty(poseStack)
    b2c = inv(b2c);
end

if 1
    intrMat = [554.2562580000000  0 320.5;0 554.2562580000000  240.5;0 0 1];
else
    intrMat = [1100 0 640.5;0 1100 360.5;0 0 1];
end
b2cPmat = GetPctBody2Cam(vsl.coordSysAligner);
b2cPmat.transformMat = b2c;
% % % close all

if shortTrace
    sampNum = 4;
else
    sampNum = 14*angSize;  14*3; 14; 14*6; 14; 4; 3; 2; 14; 28; 14; 8; 7; 14;100;
end
transVec = 1000.*(rand(3, sampNum) - 0.5);
transVec(3,:) = 4000 + transVec(3,:);

angList =  50.*(rand(3, sampNum) - 0.5);

if 0
    if 0
        obj = MR_obj_read('data/StanfordBunny.obj');
        % obj = MR_obj_read('data/cube.obj');
        obj.V = 10000.*(rotx(40)*rotz(180)*roty(180)*obj.V')';
    else
        if ~multi_obj
            %         obj = MR_obj_read('data/cube.obj');
            
            
%             obj = MR_obj_read('data/cruiser.obj');
                    obj = MR_obj_read('data/ball.obj');
            
            
            
            %     obj = MR_obj_read('data/ball2.obj');
            %     obj = MR_obj_read('data/tyra.obj');
            %     obj.V = 1000.*(rotz(180)*roty(180)*obj.V')';
            %     obj.V = 1000.*(rotz(90)*roty(90)*rotz(90)*obj.V')';
            %     obj.V = 1000.*(roty(20)*rotz(90)*roty(90)*rotz(90)*obj.V')';
            %     obj.V = 4.*1000.*(roty(0)*rotz(90)*roty(90)*rotz(90)*obj.V')';
            if ~shortTrace
                obj.V = 1.*1000.*(roty(0)*rotz(90)*roty(90)*rotz(90)*obj.V')';
            else
                %             obj.V = 1.*2000.*(rotz(0)*roty(90)*rotz(90)*obj.V')';
                obj.V = 1.*3000.*(rotz(0)*roty(90)*rotz(90)*obj.V')';
            end
            obj.V(:,1) = obj.V(:,1) - mean(obj.V(:,1));
            obj.V(:,2) = obj.V(:,2) - mean(obj.V(:,2));
            obj.V(:,3) = obj.V(:,3) - mean(obj.V(:,3));
            
        else
            obj = MR_obj_read('data/ball2.obj');
            %         obj.V = -1.*2000.*obj.V;
            obj.V = 1000.*(rotx(90)*obj.V')';
            
        end
    end
end
% obj.V = 100.*(rotz(180)*roty(180)*obj.V')';

max(obj.F) - size(obj.V,1)


T0 = [roty(theta) [-2000 -200 5500]'; 0 0 0 1];
T0 = [rotz(theta) [-2000 -200 5500]'; 0 0 0 1];
% T0 = [roty(theta) [-500 -200 5500]'; 0 0 0 1];
if ~rot_y
    if isempty(TCur_camInit)
        T0 = [rotz(theta) [0 -1200 5500]'; 0 0 0 1];
    else
        T0 = [rotx(theta) [0 0 0]'; 0 0 0 1]*[rotz(90) [0 -2200 5500]'; 0 0 0 1];
    end
    T0(2,4) = -1200;
    T0(3,4) = 5800;
else
    T0 = [rotz(theta) [-2000 -200 5500]'; 0 0 0 1]; % close
%     T0 = [rotz(theta) [-2500 -200 7500]'; 0 0 0 1]; % close
%     T0 = [rotz(theta) [-2000 1400 5500]'; 0 0 0 1];
    T0 = [roty(theta) [-2000 -200 5500]'; 0 0 0 1]; 
    T0 = [roty(theta) [-2000 -2000 5500]'; 0 0 0 1]; 
    T0 = [rotz(theta) [-2000 -1000 5500]'; 0 0 0 1]; 
    
    if ~shortTrace
        T0 = [roty(theta) [-2000 -1000 5500]'; 0 0 0 1];
        
    else
        T0 = [roty(theta) [-500 -0 7500]'; 0 0 0 1]; % full screen
    end
    if multi_obj
        T0 = [rotz(theta) [0 -1000 5500]'; 0 0 0 1]; 
    end
end

T0 = T_made;


XYZ_stack = [];
if rot_y
    angStep = 2.8/angSize; 0.93; 2.8;  0.5; 2.8; 0.5; 1;  2.8; 1.4; % close
%     angStep = 52,8;  % far
else
    angStep = 2; 1; 1.4; 2.8; 1.4; 2.8;  3;
end
rotAngList = 0 : angStep : (sampNum - 1)*angStep;

if inverseRot
    angStep = -angStep;
    rotAngList = 0 : angStep : (sampNum - 1)*angStep;
    if 0
        T0(1,4) = -T0(1,4);
        
        
        T0(1,4) = 2200; % -T0(1,4);   close
        T0(2,4) = -500;
    end
%     T0(1,4) = 5500; T0(2,4) = -1500; T0(3,4) = 20000;   % far
    
end


xyzOrig_fixed = [];

TCur_cam_stack = {};


TraceErr_inv = [];

gtRemapErr = []; gtRemapErr_p2c = [];
for i = 1 : sampNum
    
    if 0
        rotMat0 = rotz(angList(3, i))*roty(angList(2, i))*rotx(angList(1, i));
        transVec0 = transVec(:,i);
    else
        if rot_y
            TCur_body = (([roty(rotAngList(i)) [0 0 0]';0 0 0 1])) * T0;
        else
            TCur_body = (([rotx(-rotAngList(i)) [0 0 0]';0 0 0 1])) * T0;
        end
        TCur_cam = b2c*TCur_body;
        rotMat0 = TCur_cam(1:3,1:3);
        transVec0 = TCur_cam(1:3,4);
        
    end
    if i == 1
        
        TCur_body = T0;
        TCur_cam = b2c*TCur_body;
        
        rotMat0 = TCur_cam(1:3,1:3);
        transVec0 = TCur_cam(1:3,4);
    end
    
    TCur_cam_stack{i,1} = TCur_cam;
    
    if ~isempty(poseStack)
        if 0
            rotMat0 = poseStack{end - i + 1, 1}(1:3,1:3);
            transVec0 = poseStack{end - i + 1, 1}(1:3,4);
        else
            %             rotMat0 = [poseStack{i, 1}(1:3,2) poseStack{i, 1}(1:3,2) poseStack{i, 1}(1:3,2)];
            
            if ~forceSame % i == 1
                T_body = inv(b2c)*poseStack{i, 1};
                %                 T_body = rotz(-theta)*T_body;
                %             T_body(1:3,1:3) = T_body(1:3,1:3)';
                %             T_body(1,4) = -T_body(1,4);
                T_body = [-1 0 0 0; 0 1 0 0 ;0 0 1 0;0 0 0 1]*T_body;
                T_body(:,1) = -T_body(:,1);
                T_cam = b2c*T_body;
                
                
                rotMat0 = T_cam(1:3,1:3);
                transVec0 = T_cam(1:3,4);
            else
                
                rotMat0 = poseStack{end-i+1, 1}(1:3,1:3);
                transVec0 = poseStack{end-i+1, 1}(1:3,4);
                
                
            end
%             transVec0(1) = -transVec0(1);
        end
    end
    
    if i > 1
        imgPrvRGB = imgCur;
    end
    [xyzOrig, imgCur0, depthMap, Vcam] = render_fcn(obj, rotMat0, transVec0);
    if 0
        objj.V = obj.V;
        objj.F = obj.F;
        objj.FT = obj.FT;
        objj.VT = obj.VT;
        
        
        [xyzOrig1, imgCur01, depthMap1, Vcam1] = render_fcn(objj, rotMat0, transVec0);
        figure, subplot(1,2,1);imshow(imgCur0 - imgCur01,[]);subplot(1,2,2);imshow(depthMap - depthMap1,[]);
        
        figure, subplot(1,2,1);imshow(imgCur0,[]);subplot(1,2,2);imshow(imgCur01,[]);
    end
    ligfuvvi = 1;
    if 0
        imgCur0 = uint8(255.*imgCur0);
    end
    if doBlur
        imgCur = imgaussfilt(uint8(imgCur0), 0.7);
    else
        imgCur = imgCur0;
    end
    % %     camPoseC2K{cnt,1}(:,:,j) = inv(RT);
    % %     camPoseK2C{cnt,1}(:,:,j) = (RT);
    % %     %                     bodyPoseK2C{i,1}(:,:,j) = [roty(rad2deg(double(thetaListTemp(j+1)))) [0;0;0];0 0 0 1];
    % %     bodyPoseK2C{cnt,1}(:,:,j) = [roty(rad2deg(double(norm(rt(1:3))))) [0;0;0];0 0 0 1];
    
    
    
    Img(:,:,i) = rgb2gray(imgCur);
    DepthMap(:,:,i) = depthMap;
    VCam{i,1} = Vcam;
    if i == 1
        imgKey = imgCur;
        depthKey = depthMap;
        if ~detectCB
            if isempty(poseStack)
                if useFast
                    ptIcsKey = detectFASTFeatures(rgb2gray(imgKey), 'MinQuality', 0.02, 'MinContrast', 0.02);
                else
                    ptIcsKey = detectHarrisFeatures(rgb2gray(imgKey), 'MinQuality', 0.0001,'FilterSize', 3);
                end
                pixKey0 = ptIcsKey.Location;
            else
                if forceSame
                    idG = find(ptTrace0.X_gt(:,end) > 0);
                    if 1
                        pixKey0 = single([ptTrace0.X_gt(idG, end) ptTrace0.Y_gt(idG, end)]);
                    else
                        pixKey0 = single([ptTrace0.X(idG, end) ptTrace0.Y(idG, end)]);
                    end
                    
                else
                    if useFast
                        ptIcsKey = detectFASTFeatures(rgb2gray(imgKey), 'MinQuality', 0.02, 'MinContrast', 0.02);
                    else
                        ptIcsKey = detectHarrisFeatures(rgb2gray(imgKey), 'MinQuality', 0.0001,'FilterSize', 3);
                    end
                    pixKey0 = ptIcsKey.Location;
                    
                end
                
            end
            
            
            
            cbcXYZL = [];
        else
            
            if isempty(poseStack)
                if isempty(TCur_camInit)
                    cbcL = detectCnr(imgKey);
                    pixKey0 = single(cbcL');
                    
                    ind_ = sub2ind(size(depthKey), round(pixKey0(:,2)), round(pixKey0(:,1)));
                    [xyzKey_local] = GetXYZFromDepth(intrMat, [pixKey0],depthKey(ind_));
                    TCur_cam_inv = inv(TCur_cam);
                    xyzKey_world = EucildTransform(xyzKey_local, TCur_cam_inv(1:3,1:3), TCur_cam_inv(1:3,4));
                    cbcXYZL = xyzKey_world;
                else
                    xyzKey_world_init = cbcXYZLInit;
                    pixKey_local = TransformAndProject(xyzKey_world_init, intrMat, TCur_cam(1:3,1:3), TCur_cam(1:3,4));
                    cbcL = cornerfinder([pixKey_local]',double(rgb2gray(imgKey)),5,5);
                    if 1
                        pixKey0 = single(cbcL');
                    else
                        pixKey0 = single(pixKey_local);
                    end
                    mappingErr = pixKey0 - pixKey_local;
                    sakvakkb = 1;
                end
                
            else
                 if forceSame
                    idG = find(ptTrace0.X_gt(:,end) > 0);
                    if 1
                        pixKey0 = single([ptTrace0.X_gt(idG, end) ptTrace0.Y_gt(idG, end)]);
                    else
                        pixKey0 = single([ptTrace0.X(idG, end) ptTrace0.Y(idG, end)]);
                    end
                    
                else
%                     ptIcsKey = detectFASTFeatures(rgb2gray(imgKey), 'MinQuality', 0.02, 'MinContrast', 0.02);
%                     pixKey0 = ptIcsKey.Location;
                    
                    cbcL = detectCnr(imgKey);
                    pixKey0 = single(cbcL');
                    
                    
                end
                
                
                
            end
        end
        mask = ~isinf(depthMap) & depthMap > 0;
        se = strel('square',[5]);
        mask_ = imerode(mask,se);
        ind0 = find(mask_ > 0);
        [y, x] = ind2sub(size(depthMap), ind0);
        [xyzKey_all] = GetXYZFromDepth(intrMat, [x y],depthKey(ind0));
        
        
        ind1 = sub2ind(size(depthKey), round(pixKey0(:,2)), round(pixKey0(:,1)));
        ind = intersect(ind0, ind1);
        pixKey = pixKey0(ismember(ind1, ind), :);
        ind_check = sub2ind(size(depthKey), round(pixKey(:,2)), round(pixKey(:,1)));
        if 0
            zList = depthKey(ind);
        else
            zList = depthKey(ind_check);
        end
        if 1 % draw
            if isempty(poseStack)
                figure(1 + figOfst),subplot(2,3,1);cla; imshow(rgb2gray(imgKey));hold on;plot(pixKey(:,1), pixKey(:,2), '.r');subplot(2,3,3);imshow(depthKey, [])
            else
                figure(1 + figOfst),subplot(2,3,4);cla; imshow(rgb2gray(imgKey));hold on;plot(pixKey(:,1), pixKey(:,2), '.r');subplot(2,3,6);imshow(depthKey, [])
                
            end
            
            drawnow;
        end
        validId  = find(~isnan(zList) & ~isinf(zList));
        ptTrace.X = pixKey(validId,1);
        ptTrace.Y = pixKey(validId,2);
        ptTrace.Z = zList(validId,1);
        ptTrace.X_gt = pixKey(validId,1);
        ptTrace.Y_gt = pixKey(validId,2);
    else
        
        if rot_y
            
            Tcam = b2c * [roty(rotAngList(i)) [0 0 0]'; 0 0 0 1] * inv(b2c);
            Tcam_p2c = b2c * [roty(rotAngList(i) - rotAngList(i-1)) [0 0 0]'; 0 0 0 1] * inv(b2c);
        else
            Tcam = b2c * [rotx(-rotAngList(i)) [0 0 0]'; 0 0 0 1] * inv(b2c);
        end
        
        
        mask_cur = ~isinf(depthMap) & depthMap > 0;
        mask_cur = imerode(mask_cur,se);
        ind0_cur = find(mask_cur > 0);
        [y_cur, x_cur] = ind2sub(size(depthMap), ind0_cur);
        [xyzCur_all] = GetXYZFromDepth(intrMat, [x_cur y_cur],depthMap(ind0_cur));
        
        
        
        validPrv = find(ptTrace.X(:,end) > 0);
        pixPrv = [ptTrace.X(validPrv, end) ptTrace.Y(validPrv, end)];
        
        
        
        
        pixKey = [ptTrace.X(validPrv, 1) ptTrace.Y(validPrv, 1)];
        zKey = ptTrace.Z(validPrv, 1);
        
        if prvBaseLK
            pixPrv0 = pixPrv;
            [xyzKey_prvBase] = GetXYZFromDepth(intrMat, pixKey, zKey);
            
            if i > 2
                [pixPrv, xyzPrv] = TransformAndProject(xyzKey_prvBase, intrMat, TcamStack{i-1, 1}(1:3,1:3), TcamStack{i-1, 1}(1:3,4));
            else
                pixPrv = pixKey;
                xyzPrv = xyzKey_prvBase;
            end
            ind_prvPt = sub2ind(size(depthMap), round(pixPrv(:,2)), round(pixPrv(:,1)));
            DepthMap_prv = DepthMap(:,:,i-1);
            [xyzPrvBase] = GetXYZFromDepth(intrMat, pixPrv, DepthMap_prv(ind_prvPt));
            [pixCur_p, ~] = TransformAndProject(xyzPrvBase, intrMat, Tcam_p2c(1:3,1:3), Tcam_p2c(1:3,4));
             xyzPrvErr = xyzPrvBase - xyzPrv;
        end
        
        
         
        
        
        imgPrv = Img(:,:,end-1);
        
        
        
        if 0
%             vid= VideoReader('D:\ProgramData\opencv\opencv-4.3.0\samples\data\vtest.avi');
%             I1 =  (readFrame(vid));
%             I2 =  (readFrame(vid));
            I1 = imread('C:\Users\rongjiezhu\Desktop\I1.png');
            I2 = imread('C:\Users\rongjiezhu\Desktop\I2.png');
            pixI1 = [692 84];
            figure,imshow(I1);hold on;plot(pixI1(:,1),pixI1(:,2),'.r')
            [predPtList_I2, inTrackFlag_I1] = LKTracking(I1, I2, pixI1, [],3);
        end
        
        
        
        if ~newLK
            [predPtList, inTrackFlag, marg] = TrackFeaturePoints(vsl.featPtTracker, imgPrv, Img(:,:,end), pixPrv, [], intrMat, b2cPmat);
        else
%             [predPtList, inTrackFlag, marg] = TrackFeaturePoints(vsl.featPtTracker, imgPrv, Img(:,:,end), pixPrv, [], intrMat, b2cPmat);
            [predPtList, inTrackFlag] = LKTracking(imgPrvRGB, imgCur, pixPrv, [],marg);
        end
        
        if traceCB
            if 0
                predPtList0 = predPtList;
                predPtList = cornerfinder([predPtList0]',double(Img(:,:,end)),5,5);
                predPtList = single(predPtList');
            else
                
                [xyzKey_init] = GetXYZFromDepth(intrMat, pixKey,zKey);
                ptIcs_ref = TransformAndProject(xyzKey_init, intrMat, Tcam(1:3,1:3), Tcam(1:3,4));
                
                predPtList = cornerfinder([ptIcs_ref]',double(Img(:,:,end)),5,5);
                inTrackFlag = true(size(ptIcs_ref,1),1);
                predPtList = single(predPtList');
            end
        end

        
        
        keyPt = pixKey(inTrackFlag,:);
        curPt = predPtList(inTrackFlag,:);
        zList = zKey(inTrackFlag,:);
        if prvBaseLK
            pixCur_p_ = pixCur_p(inTrackFlag,:);
            xyzPrvBase_ = xyzPrvBase(inTrackFlag,:);
            xyzPrv_ = xyzPrv(inTrackFlag,:);
        end
%         xyzErrCur_pix = curPt;
        if ~newLK
            [prvPtList, inTrackFlag_inv, marg] = TrackFeaturePoints(vsl.featPtTracker,  Img(:,:,end), imgPrv, curPt, [], intrMat, b2cPmat);
        else
            [prvPtList, inTrackFlag_inv] = LKTracking(imgCur, imgPrvRGB, curPt, [],marg);
        end
        if traceCB
            prvPtList0 = prvPtList;
            prvPtList = cornerfinder([prvPtList0]',double(imgPrv),5,5);
            prvPtList = single(prvPtList');
        end
        
        
        
        traceErr_inv = prvPtList - pixPrv(inTrackFlag,:);
        TraceErr_inv = [TraceErr_inv; traceErr_inv];
        
        [xyzKey] = GetXYZFromDepth(intrMat, keyPt,zList);
        
        if 0
            validId = find(inTrackFlag);
        else
            validId0 = find(inTrackFlag);
            validId = validPrv(inTrackFlag);
        end
        pnp_ang_est_max_margin = [deg2rad([-1 1]) 0.5];
        if rot_y
            [k2cBodyRotAng_2,Err_2,inId] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin,intrMat,keyPt,curPt,zList, validId,true(length(validId),1),b2c,double(deg2rad(rotAngList(i))));
        else
            [k2cBodyRotAng_2,Err_2,inId] = VisualLocalizer.RotationAngleEstimate_PNP_(pnp_ang_est_max_margin,intrMat,keyPt,curPt,zList, validId,true(length(validId),1),b2c,double(-deg2rad(rotAngList(i))));
        end
        
        
        %         [xyzKey] = GetXYZFromDepth(intrMat, keyPt,zList);
        try
            [rtTemp, inlierId] = posest(double(curPt), double(xyzKey), 0.9, intrMat, 'repr_err');
            RT = [rodrigues(rtTemp(1:3)) rtTemp(4:6); 0 0 0 1];
        catch
            kfhk = 1;
        end
        SmartRejection = 100;
        [R, t, ER, maxD] = icp(VCam{end}',  VCam{1}', 20, 'Matching','kDtree','SmartRejection',SmartRejection);
        
        errrrr = ((Tcam)*pextend(VCam{1}')) -  pextend(VCam{end}');
        
        
% % % %         if rot_y
% % % %         
% % % %             Tcam = b2c * [roty(rotAngList(i)) [0 0 0]'; 0 0 0 1] * inv(b2c);
% % % %             Tcam_p2c = b2c * [roty(rotAngList(i) - rotAngList(i-1)) [0 0 0]'; 0 0 0 1] * inv(b2c);
% % % %         else
% % % %             Tcam = b2c * [rotx(-rotAngList(i)) [0 0 0]'; 0 0 0 1] * inv(b2c);
% % % %         end
            RTcomp = [R t ; 0 0 0 1];
        
        RT_err = RTcomp - Tcam;
        
        
        TcamStack{i,1} = Tcam;
        
        
        if 0
            ptIcs = TransformAndProject(xyzKey, intrMat, R, t);
        else
            ptIcs = TransformAndProject(xyzKey, intrMat, Tcam(1:3,1:3), Tcam(1:3,4));
            %             ptIcs2 = TransformAndProject(xyzKey, intrMat, Tcam_p2c(1:3,1:3), Tcam_p2c(1:3,4));
            
            if prvBaseLK
                ptIcs2_k2p = TransformAndProject(xyzPrv_, intrMat, Tcam_p2c(1:3,1:3), Tcam_p2c(1:3,4));
                ptIcs2_p = TransformAndProject(xyzPrvBase_, intrMat, Tcam_p2c(1:3,1:3), Tcam_p2c(1:3,4));
                
                
                curGT_err1 = pixCur_p_ - ptIcs2_p;
                curGT_err2 = pixCur_p_ - ptIcs2_k2p;
                bitMatch = ptIcs2_p - pixCur_p_;
                err_unMatch = ptIcs - ptIcs2_p;
                ptIcs0 = ptIcs;
                if 0
                    ptIcs = ptIcs2_p;
                end
            end
        end
        curTrackingErr = curPt - ptIcs;
        
        CurTrackingErr{i,1} = mean(curTrackingErr);
        CurTrackingErr{i,2} = (curTrackingErr);
        
        
        indCur = sub2ind(size(depthMap), round(ptIcs(:,2)), round(ptIcs(:,1)));
        indCurLK = sub2ind(size(depthMap), round(curPt(:,2)), round(curPt(:,1)));
        
        indKey = sub2ind(size(depthMap), round(keyPt(:,2)), round(keyPt(:,1)));
        
        [xyzCur] = GetXYZFromDepth(intrMat, ptIcs,depthMap(indCur));
        [xyzCurLK] = GetXYZFromDepth(intrMat, curPt,depthMap(indCurLK));
        
        [xyzKey_] = GetXYZFromDepth(intrMat, keyPt,depthKey(indKey));
        
        [R_, t_, ER_, maxD_] = icp(xyzCur',  xyzKey_', 20, 'Matching','kDtree','SmartRejection',SmartRejection);
        
        
        Tcam__inv = inv(Tcam);
        ptIcsGT_key = TransformAndProject(xyzCur, intrMat, Tcam__inv(1:3,1:3), Tcam__inv(1:3,4));
        ptIcsLK_key = TransformAndProject(xyzCurLK, intrMat, Tcam__inv(1:3,1:3), Tcam__inv(1:3,4));
        
        xyzErrGT_pix = keyPt - ptIcsGT_key;
        xyzErrLK_pix = keyPt - ptIcsLK_key;
        
        
        if 0 % ~isempty(poseStack) && forceSame && prvBaseLK
            [prvPtList, inTrackFlag_inv, marg] = TrackFeaturePoints(vsl.featPtTracker,  Img(:,:,end), imgPrv, ptIcs, [], intrMat, b2cPmat);
            traceErr_inv = prvPtList - pixPrv(inTrackFlag,:);
            
            TraceErr_inv = [TraceErr_inv; traceErr_inv];
            traceMat = [];
        end
        
        
        xyzKey_all_temp = EucildTransform(xyzKey_all, Tcam(1:3, 1:3), Tcam(1:3,4));
        temp = ([Tcam]*pextend(xyzKey'))';
        tempKey = temp(:,1:3);
        xyzErrGT = tempKey - xyzCur;
        xyzErrLK = tempKey - xyzCurLK;
        [~, errGT] = NormalizeVector(xyzErrGT);
        [~, errLK] = NormalizeVector(xyzErrLK);
        
        if 0 % i == round(sampNum/1)
            if 0
                figure(4),clf;subplot(1,2,1);pcshow(xyzKey_all_temp, 'r', 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');hold on;pcshow(xyzCur_all, 'b', 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
                subplot(1,2,2);fig_3D3_pair(tempKey', xyzCur');
            elseif 0
                figure(4),clf;subplot(2,2,1);fig_3D3_pair(tempKey', xyzCurLK');title('LK');
                subplot(2,2,2);fig_3D3_pair(tempKey', xyzCur');title('GT');
                subplot(2,2,3);hist(xyzErrLK, 100);grid on;title('LK');
                subplot(2,2,4);hist(xyzErrGT, 100);grid on;title('GT');
            elseif 1
                figure(4),clf;subplot(1,2,1);fig_3D3_pair(tempKey', xyzCurLK');title('LK');
                subplot(1,2,2);fig_3D3_pair(tempKey', xyzCur');title('GT');
            else
                figure(4),clf;pcshow(xyzKey_all_temp, 'r', 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');hold on;pcshow(Vcam, 'b', 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
            end
        end
        if i == round(sampNum/1) % draw
            figure(2),clf;subplot(3,2,1);imshow(imgPrv);hold on;plot(pixPrv(inTrackFlag,1), pixPrv(inTrackFlag,2), '.r');title('prv');
            try
                subplot(3,2,2);plot(mappingErr(:,1), mappingErr(:,2), '+r');axis equal; title('mapping - detection');
            catch
                subplot(3,2,2);imshow(Img(:,:,end));hold on;plot(predPtList(inTrackFlag,1), predPtList(inTrackFlag,2), '.r');title('cur');
            end
            subplot(3,2,3);imshow(Img(:,:,1));hold on;plot(keyPt(:,1), keyPt(:,2), '.r'); title('key');subplot(3,2,4);imshow(Img(:,:,end));hold on;plot(curPt(:,1), curPt(:,2), '.r');plot(ptIcs(:,1), ptIcs(:,2),'.g');title('cur');
            subplot(3,2,5),plot(-ptIcs(:,1) +  curPt(:,1), -ptIcs(:,2) + curPt(:,2), '.r');axis equal;title(num2str(i));
            %             subplot(3,2,6),plot([errLK errGT]);legend('LK', 'GT');
            %             subplot(3,2,6),hist([errLK errGT], 100);legend('LK', 'GT');
            subplot(3,2,6),hist([errLK - errGT], 200);grid on; legend('norm(LK err) - norm(GT err)');
            %             figure,plot([errGT errLK]);legend('GT', 'LK')
            
            if isempty(poseStack)
            figure(1 + figOfst),subplot(2,3,2);imshow(Img(:,:,end));hold on;plot(curPt(:,1), curPt(:,2), '.r');plot(ptIcs(:,1), ptIcs(:,2),'.g');title('cur');
            else
             figure(1 + figOfst),subplot(2,3,5);imshow(Img(:,:,end));hold on;plot(curPt(:,1), curPt(:,2), '.r');plot(ptIcs(:,1), ptIcs(:,2),'.g');title('cur');
    
            end
            drawnow;
        end
        if 0
            outId = setdiff([1:size(ptTrace.X,1)]', validPrv(inId));
        else
            outId = setdiff([1:size(ptTrace.X,1)]', (inId));
        end
        
        
        
        gtRemapErr = [gtRemapErr; mean(xyzErrGT_pix(ismember(validId,inId),:))];
        if prvBaseLK
            gtRemapErr_p2c = [gtRemapErr_p2c; mean(err_unMatch(ismember(validId,inId),:))];
        end
        
        ptTrace.X(inId, i) = curPt(ismember(validId,inId),1);
        ptTrace.Y(inId, i) = curPt(ismember(validId,inId),2);
        ptTrace.X_gt(inId, i) = ptIcs(ismember(validId,inId),1);
        ptTrace.Y_gt(inId, i) = ptIcs(ismember(validId,inId),2);
        
        ptTrace.X_gt(outId, i) = -1;
        ptTrace.Y_gt(outId, i) = -1;
        %        ptTrace.Z(inId, i) = curPt(:,2);
        ptTrace.X(outId, i) = -1;
        ptTrace.Y(outId, i) = -1;
        
        
        
    end
    XYZ_stack = [XYZ_stack; xyzOrig];
    if i == 1
        xyzOrig_fixed = xyzOrig;
    end
    
    
    drawnow;
    
    
end




if  ~isempty(poseStack)  % && forceSame % && ~prvBaseLK
    
    
    CW_pt = [ptTrace0.X_gt(:,1) ptTrace0.Y_gt(:,1)];
    CCW_pt = [ptTrace.X_gt(:,end) ptTrace.Y_gt(:,end)];
    id_ccw = find(CCW_pt(:,1) > 0);
    CCW_pt_ = CCW_pt(id_ccw,:);
    
    if forceSame
        
        [timeL__1, timeD__1, samePtFlagL1, samePtFlagD1] = KNN(CW_pt(:,1:2), CCW_pt_(:,1:2));
        %     [timeL__2, timeD__2, samePtFlagL2, samePtFlagD2] = KNN(CW_pt(:,2), CCW_pt_(:,2));
        
        CW_trace.X = ptTrace0.X(samePtFlagD1,:);
        CW_trace.Y = ptTrace0.Y(samePtFlagD1,:);
        CW_trace.X_gt = ptTrace0.X_gt(samePtFlagD1,:);
        CW_trace.Y_gt = ptTrace0.Y_gt(samePtFlagD1,:);
        
        CCW_trace.X = ptTrace.X(id_ccw,end:-1:1);
        CCW_trace.Y = ptTrace.Y(id_ccw,end:-1:1);
        CCW_trace.X_gt = ptTrace.X_gt(id_ccw,end:-1:1);
        CCW_trace.Y_gt = ptTrace.Y_gt(id_ccw,end:-1:1);
        
        cwErrMatX = CW_trace.X(:,2:end) - CW_trace.X_gt(:,2:end);
        cwErrMatY = CW_trace.Y(:,2:end) - CW_trace.Y_gt(:,2:end);
        cwErrMatX0 = CW_trace.X(:,1:end) - CW_trace.X_gt(:,1:end);
        cwErrMatY0 = CW_trace.Y(:,1:end) - CW_trace.Y_gt(:,1:end);
        
        ccwErrMatX = CCW_trace.X(:,1:end-1) - CCW_trace.X_gt(:,1:end-1);
        ccwErrMatY = CCW_trace.Y(:,1:end-1) - CCW_trace.Y_gt(:,1:end-1);
        ccwErrMatX0 = fliplr(CCW_trace.X(:,1:end) - CCW_trace.X_gt(:,1:end));
        ccwErrMatY0 = fliplr(CCW_trace.Y(:,1:end) - CCW_trace.Y_gt(:,1:end));
        
        try
            CW_CCW_err = CW_trace.Y_gt - CCW_trace.Y_gt;
        catch
            sfk = 1;
        end
        if 0
            figure,plot(CW_CCW_err(:))
        end
        if 0
            pixAllFrame = [CW_trace.X_gt(:) CW_trace.Y_gt(:)];
            metricAllFrame = (inv(intrMat)*pextend(pixAllFrame'))';
            [metricAllFrame_norm0, ~] = NormalizeVector(metricAllFrame);
            metricAllFrame_norm = metricAllFrame_norm0;
            metricAllFrame_norm(:,2) = 0;
            [metricAllFrame_norm, ~] = NormalizeVector(metricAllFrame_norm);
            
            metricAllFrame_norm_x_mat = reshape(metricAllFrame_norm(:,1), size(CW_trace.X_gt));
            vecAng = CalcDegree2(metricAllFrame_norm, repmat([0 0 1], size(metricAllFrame_norm,1), 1));
            vecAngMat = reshape(vecAng, size(CW_trace.X_gt));
            vecAngMat0 = vecAngMat;
            vecAngMat(metricAllFrame_norm_x_mat < 0) = -vecAngMat(metricAllFrame_norm_x_mat < 0);
        else
            
            [vecAngMatCW] = CalcAngMat(intrMat, [CW_trace.X(:) CW_trace.Y(:)], size(CW_trace.X), [0 0 1]);
            [vecAngMatCCW] = CalcAngMat(intrMat, [CCW_trace.X(:) CCW_trace.Y(:)], size(CCW_trace.X), [0 0 1]);
            [vecAngMatCW_y0] = CalcAngMat(intrMat, [CW_trace.X(:) CW_trace.Y(:)], size(CW_trace.X),[0 -1 0]);
            [vecAngMatCCW_y0] = CalcAngMat(intrMat, [CCW_trace.X(:) CCW_trace.Y(:)], size(CCW_trace.X), [0 -1 0]);
            vecAngMatCW_y = diff(vecAngMatCW_y0')'; vecAngMatCW_y(vecAngMatCW_y(:,1) - vecAngMatCW_y(:,end) > 0,:) = -vecAngMatCW_y(vecAngMatCW_y(:,1) - vecAngMatCW_y(:,end) > 0,:);
            vecAngMatCCW_y = diff(vecAngMatCCW_y0')'; vecAngMatCCW_y(vecAngMatCCW_y(:,1) - vecAngMatCCW_y(:,end) > 0,:) = -vecAngMatCCW_y(vecAngMatCCW_y(:,1) - vecAngMatCCW_y(:,end) > 0,:);
            
            
        end
    else
        id_cw = find(ptTrace0.X(:,end) > 0);
        CW_trace.X = ptTrace0.X(id_cw,:);
        CW_trace.Y = ptTrace0.Y(id_cw,:);
        CW_trace.X_gt = ptTrace0.X_gt(id_cw,:);
        CW_trace.Y_gt = ptTrace0.Y_gt(id_cw,:);
        
        id_ccw = find(ptTrace.X(:,end) > 0);
        CCW_trace.X = ptTrace.X(id_ccw,end:-1:1);
        CCW_trace.Y = ptTrace.Y(id_ccw,end:-1:1);
        CCW_trace.X_gt = ptTrace.X_gt(id_ccw,end:-1:1);
        CCW_trace.Y_gt = ptTrace.Y_gt(id_ccw,end:-1:1);
        [vecAngMatCW] = CalcAngMat(intrMat, [CW_trace.X(:) CW_trace.Y(:)], size(CW_trace.X),[0 0 1]);
        [vecAngMatCCW] = CalcAngMat(intrMat, [CCW_trace.X(:) CCW_trace.Y(:)], size(CCW_trace.X), [0 0 1]);
        
        [vecAngMatCW_y0] = CalcAngMat(intrMat, [CW_trace.X(:) CW_trace.Y(:)], size(CW_trace.X),[0 -1 0]);
        [vecAngMatCCW_y0] = CalcAngMat(intrMat, [CCW_trace.X(:) CCW_trace.Y(:)], size(CCW_trace.X), [0 -1 0]);
        vecAngMatCW_y = diff(vecAngMatCW_y0')'; vecAngMatCW_y(vecAngMatCW_y(:,1) - vecAngMatCW_y(:,end) > 0,:) = -vecAngMatCW_y(vecAngMatCW_y(:,1) - vecAngMatCW_y(:,end) > 0,:);
        vecAngMatCCW_y = diff(vecAngMatCCW_y0')'; vecAngMatCCW_y(vecAngMatCCW_y(:,1) - vecAngMatCCW_y(:,end) > 0,:) = -vecAngMatCCW_y(vecAngMatCCW_y(:,1) - vecAngMatCCW_y(:,end) > 0,:);
    end
    
    sjfhg = 1;
    
    if 0
        figure,plot(vecAngMat')
    end
    if 0 % ~prvBaseLK
        figure(7);clf;subplot(2,2,1);plot([mean(cwErrMatX); mean(ccwErrMatX)]');title('x k2c');legend('CW','CCW');
        subplot(2,2,2);plot([mean(cwErrMatY); mean(ccwErrMatY)]');title('y k2c');legend('CW p2c','CCW');
        subplot(2,2,3);plot([diff(mean(cwErrMatX0)); diff(mean(ccwErrMatX0))]');title('x p2c');legend('CW','CCW');
        subplot(2,2,4);plot([diff(mean(cwErrMatY0)); diff(mean(ccwErrMatY0))]');title('y p2c');legend('CW p2c','CCW');
    else
        
        
        
    end
                  
                  
                  
end


xErr = ptTrace.X(inId, 1:end) - ptTrace.X_gt(inId, 1:end);
yErr = ptTrace.Y(inId, 1:end) - ptTrace.Y_gt(inId, 1:end);

x_err = mean(xErr);
y_err = mean(yErr);


if 1 % draw
    if isempty(poseStack)
        figNum = 3;
        figure(figNum),clf; subplot(2,2,1);plot(xErr'); title('x err');subplot(2,2,2);plot(yErr'); title('y err');title(num2str(b2c(1:3,4)'))
        subplot(2,2,3);plot(x_err(2:end),'-r'); title('mean x err');subplot(2,2,4);plot(y_err(2:end),'-r'); title('mean y err');
    else
        figNum = 3;
        figure(figNum), subplot(2,2,1);cla; plot(xErr'); title('x err');subplot(2,2,2);cla;plot(yErr'); title('y err');title(num2str(b2c(1:3,4)'))
        if ~prvBaseLK
            if 0
                subplot(2,2,3);hold on;plot(x_err(end:-1:1)); title('mean x err');subplot(2,2,4);hold on; plot(y_err(end:-1:1)); title('mean y err');
            else
                subplot(2,2,3);hold on;plot(x_err,'-c'); title('mean x err');legend('CW err','CCW err');
                subplot(2,2,4);hold on; plot(y_err,'-c'); title('mean y err');legend('CW err','CCW err');
            end
        else
            subplot(2,2,3);hold on;plot(x_err(2:end),'-c'); title('mean x err');
%             plot([diff(mean(vecAngMatCW)); diff(mean(vecAngMatCCW))]'./100);
            plot([(mean(vecAngMatCW(:,2:end)))]'./1000,'-m');
            plot([(mean(vecAngMatCCW(:,2:end)))]'./1000,'-g');
            legend('CW err','CCW err','CW ang','CCW ang');
            
            subplot(2,2,4);hold on; plot(y_err,'-c'); title('mean y err');
            plot([(mean(vecAngMatCW_y(:,1:end)))]'./10,'-m');
            plot([(mean(vecAngMatCCW_y(:,1:end)))]'./10,'-g');
            legend('CW err','CCW err','CW ang','CCW ang');
        end
    end
    
    if 0 % prvBaseLK
        
        subplot(2,2,3);hold on;plot(cumsum(mean(xErr)));legend('p2c','k2c');
        subplot(2,2,4);hold on;plot(cumsum(mean(yErr)));legend('p2c','k2c');
        
    end
    drawnow;
end
if 0
    err = XYZ_stack - repmat(XYZ_stack(1,:), size(XYZ_stack,1), 1);
    figure,plot(err);grid on;
end

% x_err = mean(xErr);
% y_err = mean(yErr);

x_err_stack = xErr;
y_err_stack = yErr;



x_err_stack(:,1) = inId;
y_err_stack(:,1) = inId;


end


function [XYZ] = GetXYZFromDepth(intrMat, Pix,depthList)
metricPrevPtCcsGT = intrMat\HomoCoord(Pix',1);
metricPrevPtCcsGT = normc(metricPrevPtCcsGT);
if 1
    if 0
        if 0
            scaleAllGT = depthListGT(inlierId)./metricPrevPtCcsGT(3,:)';
        else
            scaleAllGT = depthListGT(:)./metricPrevPtCcsGT(3,:)';
        end
    else
        scaleAllGT = depthList./metricPrevPtCcsGT(3,:)';
    end
else
    dispGTComp = dispList(inlierId) - disparityErrorRound;
    depthListGTComp = intrMat(1,1).*norm(obj.camModel.transVec1To2)./(dispGTComp + (princpPtR(1) - princpPtL(1)));
    scaleAllGT = depthListGTComp./metricPrevPtCcsGT(3,:)';
end

XYZ = [repmat(scaleAllGT',3,1).*metricPrevPtCcsGT]';




end
function cbcL = detectCnr(imgL1)

[cbcX1, cbcY1, corner1] = DetectCbCorner(rgb2gray(imgL1));
[initCbcX1, initCbcY1] = CbCoordSys(cbcX1, cbcY1);
cbcL = cornerfinder([initCbcX1(:),initCbcY1(:)]',double(rgb2gray(imgL1)),5,5);
end
    function [timeL__, timeD__, samePtFlagL, samePtFlagD] = KNN(timeD_, timeL_)

thr = 0.001;

nearestIdx1 = knnsearch(timeD_, timeL_, 'NSMethod', 'kdtree');
% thr = mode(VecNorm(timeL_ - timeD_(nearestIdx1, :), 2));

samePtFlagL = VecNorm(timeL_ - timeD_(nearestIdx1, :), 2) <=  thr;      %0.4; % 0.2; %obj.featPtManager.configParam.radius_thresh_for_combine; %
timeL__ = timeL_(samePtFlagL,:);

nearestIdx2 = knnsearch(timeL_, timeD_, 'NSMethod', 'kdtree');
samePtFlagD = VecNorm(timeD_ - timeL_(nearestIdx2, :), 2) <=  thr;
timeD__ = timeD_(samePtFlagD,:);


    end

    
    
    function [vecAngMat] = CalcAngMat(intrMat, pixAllFrame, matSize, Vec)
    metricAllFrame = (inv(intrMat)*pextend(pixAllFrame'))';
    [metricAllFrame_norm0, ~] = NormalizeVector(metricAllFrame);
    metricAllFrame_norm = metricAllFrame_norm0;
    
    if Vec(end) == 1
        metricAllFrame_norm(:,2) = 0;
    else
        metricAllFrame_norm(:,1) = 0;
    end
    [metricAllFrame_norm, ~] = NormalizeVector(metricAllFrame_norm);
    
    metricAllFrame_norm_x_mat = reshape(metricAllFrame_norm(:,1), matSize);
    vecAng = CalcDegree2(metricAllFrame_norm, repmat(Vec, size(metricAllFrame_norm,1), 1));
    vecAngMat = reshape(vecAng, matSize);
    vecAngMat0 = vecAngMat;
    if Vec(end) == 1
        vecAngMat(metricAllFrame_norm_x_mat < 0) = -vecAngMat(metricAllFrame_norm_x_mat < 0);
    end
    end