classdef PureRotationAngPredictor < Configurable
    properties
        
    end
    
    methods
        function obj = PureRotationAngPredictor(cfgParam)
            obj@Configurable(cfgParam);
        end
        
        function [k2cBodyRotAng3, error2,activeFlag2] = RotationAngleEstimate_PNP(radius, obj,intrMat,featPtManager,coordSysAligner,k2cRef,keyFrameFlagList,frmStampList,goldenPose)
            global newRender
            k2cRef0 = k2cRef;
            %             k2cRef = deg2rad(round(rad2deg(k2cRef),1));
            
            
            usePNP = 1;
            
            idKey = find(keyFrameFlagList == 1, 1, 'last');
            keyFrmTime = frmStampList(idKey);
            curFrmTime = frmStampList(end);
            KeyCurTime = [keyFrmTime; curFrmTime];
            
            if ~newRender
                for i = 1 : size(KeyCurTime,1)
                    
                    time11 = KeyCurTime(i,1);
                    id1 = find(goldenPose(:,1) <= time11);
                    id2 = find(goldenPose(:,1) > time11);
                    if isempty(id2)
                        id2 = size(goldenPose,1);
                    end
                    t1 = goldenPose(id1(end),1);
                    q1 = rotMat2quatern(reshape(goldenPose(id1(end),2:10),3,3));
                    %                 q1 = data2(id1(end),[8 5 6 7]);
                    t2 = goldenPose(id2(1),1);
                    q2 = rotMat2quatern(reshape(goldenPose(id2(1),2:10),3,3));
                    %                 q2 = data2(id2(1),[8 5 6 7]);
                    scale = [(time11-t1)/(t2-t1) (t2-time11)/(t2-t1) ];
                    q11 = quatnormalize(q1);
                    q22 = quatnormalize(q2);
                    if t1 ~= t2
                        qi = quatinterp(q11,q22,scale(1),'slerp');
                    else
                        qi = q11;
                    end
                    if 0
                        qi1 = quatinterp(q11,q22,0,'slerp');
                        qi1-q11
                    end
                    rotMatTemp = quatern2rotMat(qi([1 2 3 4]));
                    rotMatList{i,1} = rotMatTemp;
                end
                rotMatKey2Cur = inv(rotMatList{2})*rotMatList{1};
                rotAxis = rodrigues(rotMatKey2Cur)./norm(rodrigues(rotMatKey2Cur));
                if rotAxis(2) < 0
                    rotAxis = -rotAxis;
                end
                
            end
            
            
            [keyPtIcs, curPtIcs, keyPtCcsZ] = GetFeaturePairs(featPtManager, 'key', 'last');
            activeFlag = keyPtCcsZ > 0 & keyPtCcsZ < 14500;2000;14500;
            metricPrevPtCcs = intrMat\HomoCoord(keyPtIcs',1);
            metricPrevPtCcs = normc(metricPrevPtCcs);
            scale = keyPtCcsZ./metricPrevPtCcs(3,:)';
            keyCcsXYZ = scale.*metricPrevPtCcs';
            b2cPmat = GetPctBody2Cam(coordSysAligner, 1);
            maxk2cRotAng = [k2cRef k2cRef] + [-obj.configParam.pnp_ang_est_max_margin obj.configParam.pnp_ang_est_max_margin];
            minAngGrid = obj.configParam.pnp_ang_est_min_angle_grid;
            k2cBodyRotAngCand = maxk2cRotAng(1):minAngGrid:maxk2cRotAng(2);
            numRotAng = length(k2cBodyRotAngCand);
            reProjErr = zeros(numRotAng,1);
            
            % %               options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter','TolX',1e-8,'MaxFunEvals',10000, 'MaxIterations',20);
            % %         [vec,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) errFunc3(cbPose, ceilingPose, X),[vec0],[],[],options);%,data_1,obs_1)
            
            
            
            
            for i = 1:numRotAng
                k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cBodyRotAngCand(i)),zeros(3,1));
                %                 k2cBodyPmat.transformMat = [rodrigues(k2cBodyRotAngCand(i)*rotAxis) [0 0 0]';0 0 0 1];
                k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
                if 1
                    k2cT = -b2cPmat.transformMat(1:3,1:3)*k2cBodyPmat.transformMat(1:3,1:3)*b2cPmat.transformMat(1:3,1:3)'*b2cPmat.transformMat(1:3,4) + b2cPmat.transformMat(1:3,4);
                    k2cCam = [k2cCamPmat.transformMat(1:3,1:3) k2cT;0 0 0 1];
                    homocurrCcsXYZ = k2cCam*HomoCoord(keyCcsXYZ',1);
                else
                    %                     k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
                    homocurrCcsXYZ = k2cCamPmat.transformMat*HomoCoord(keyCcsXYZ',1);
                end
                currCcsXYZ = homocurrCcsXYZ(1:3,:);
                if usePNP
                    [reProjErr(i),~,~,err(:,i)] = VisualLocalizer.CalculateReprojectError2(intrMat,currCcsXYZ',curPtIcs,[],[],activeFlag,0);
                else
                    [reProjErr(i),err(:,i)] = AngleSpace.ProjectErrorUnderTransform2(radius, k2cCam(1:3,1:3), k2cCam(1:3,4), keyPtIcs(activeFlag,:), curPtIcs(activeFlag,:), intrMat);
                    
                end
            end
            [minDis,minInd] = min(reProjErr);
            k2cBodyRotAng = k2cBodyRotAngCand(minInd);
            
            
            if ~ usePNP
                %                 activeFlag2 = activeFlag;
                
                inliers1 = find(activeFlag);
                inliers2 = find(err(:,minInd) < obj.configParam.pure_rot_reproject_outlier_threshold);
                inliers3 = inliers1(inliers2);
                activeFlag2 = false(length(activeFlag),1);
                activeFlag2(inliers3) = true;
                
                
                
                error2 = err(:,minInd);
                k2cBodyRotAng3 = k2cBodyRotAng;
                
                figure(13);clf;plot(rad2deg(k2cBodyRotAngCand),reProjErr,'-b');hold on;plot(rad2deg(k2cBodyRotAngCand(minInd)), reProjErr(minInd),'*r');plot(rad2deg(k2cRef), reProjErr(minInd),'og');
                legend('curve','calced','gt');
                title(sprintf('k2cRef: %0.3f degree\nepiLine: %0.3f degree',rad2deg(k2cRef0), rad2deg(k2cBodyRotAng3)))
                
                
                return;
            end
            
            inliers1 = find(activeFlag);
            inliers2 = find(err(:,minInd) < obj.configParam.pure_rot_reproject_outlier_threshold);
            inliers3 = inliers1(inliers2);
            activeFlag2 = false(length(activeFlag),1);
            activeFlag2(inliers3) = true;
            
            
            reProjErr2 = zeros(numRotAng,1);
            
            for i = 1:numRotAng
                k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cBodyRotAngCand(i)),zeros(3,1));
                %                 k2cBodyPmat.transformMat = [rodrigues(k2cBodyRotAngCand(i)*rotAxis) [0 0 0]';0 0 0 1];
                k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
                if 1
                    k2cT = -b2cPmat.transformMat(1:3,1:3)*k2cBodyPmat.transformMat(1:3,1:3)*b2cPmat.transformMat(1:3,1:3)'*b2cPmat.transformMat(1:3,4) + b2cPmat.transformMat(1:3,4);
                    k2cCam = [k2cCamPmat.transformMat(1:3,1:3) k2cT;0 0 0 1];
                    homocurrCcsXYZ = k2cCam*HomoCoord(keyCcsXYZ',1);
                else
                    %                     k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
                    homocurrCcsXYZ = k2cCamPmat.transformMat*HomoCoord(keyCcsXYZ',1);
                end
                currCcsXYZ = homocurrCcsXYZ(1:3,:);
                if usePNP
                    [reProjErr2(i),~,~,err2(:,i)] = VisualLocalizer.CalculateReprojectError2(intrMat,currCcsXYZ',curPtIcs,[],[],activeFlag2,0);
                else
                    [reProjErr2(i),err2(:,i)] = AngleSpace.ProjectErrorUnderTransform2(k2cCam(1:3,1:3), k2cCam(1:3,4), keyPtIcs, curPtIcs, intrMat);
                end
            end
            [minDis2,minInd2] = min(reProjErr2);
            
            if minInd2 < 4
                minInd2 = 4;
                
                minInd2_check = 4;
            else
                minInd2_check = 0;
            end
            
            k2cBodyRotAng2 = k2cBodyRotAngCand(minInd2);
            error2 = err2(:,minInd2);
            
            
            
            vec = [-3 : 3];
            %             figure(13),clf;hold on;
            I = [];
            for kk = 1 : length(vec)
                i = minInd2+vec(kk);
                I = [I;i];
                k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cBodyRotAngCand(i)),zeros(3,1));
                %                 k2cBodyPmat.transformMat = [rodrigues(k2cBodyRotAngCand(i)*rotAxis) [0 0 0]';0 0 0 1];
                k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
                if 1
                    k2cT = -b2cPmat.transformMat(1:3,1:3)*k2cBodyPmat.transformMat(1:3,1:3)*b2cPmat.transformMat(1:3,1:3)'*b2cPmat.transformMat(1:3,4) + b2cPmat.transformMat(1:3,4);
                    k2cCam = [k2cCamPmat.transformMat(1:3,1:3) k2cT;0 0 0 1];
                    homocurrCcsXYZ = k2cCam*HomoCoord(keyCcsXYZ',1);
                else
                    %                     k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
                    homocurrCcsXYZ = k2cCamPmat.transformMat*HomoCoord(keyCcsXYZ',1);
                end
                currCcsXYZ = homocurrCcsXYZ(1:3,:);
                if usePNP
                    [reProjErr3(kk,1),~,~,err3(:,kk),errXY{kk}] = VisualLocalizer.CalculateReprojectError2(intrMat,currCcsXYZ',curPtIcs,[],[],activeFlag2,0);
                else
                    a = 1;
                end
                if usePNP
                    [reProjErr32(kk,1),~,~,err32(:,kk),errXY2{kk}] = VisualLocalizer.CalculateReprojectError2(intrMat,currCcsXYZ',curPtIcs,[],[],activeFlag2,0);
                else
                    
                end
                errX(kk,1) = abs(mean(errXY{kk}(:,1)));
                %                 errX(kk,1) = abs(mean(errXY{kk}(:,1)));
                %                 plot(errXY{kk}(:,1),errXY{kk}(:,2),'+');
            end
            
            
            [minDisX,minIndX] = min(errX);
            [minDisX2,minIndX2] = min(reProjErr32);
            minIndX = minIndX2;
            k2cBodyRotAng3 = k2cBodyRotAngCand(I(minIndX));
            k2cBodyRotAng4 = k2cBodyRotAngCand(I(minIndX2));
            errNorm = err3(:,minIndX);
            
            idErrMax = find(errNorm > 0.5);
            pixValid = find(activeFlag2);
            idErrMax2 = pixValid(idErrMax);
            
            if 0
                figure(13),clf;hold on;
                plot(errXY{minIndX}(:,1),errXY{minIndX}(:,2),'+');
                plot(errXY2{minIndX2}(:,1),errXY2{minIndX2}(:,2),'+');
                %             legend('custom','L2');
                legend(num2str(minIndX),num2str(minIndX2));
                axis equal;
                
                title(sprintf('[%05f %05f]\n[%05f %05f]\n[%05f %05f]\n[%05f %05f]\n[%05f %05f]\n[%05f %05f]\n[%05f %05f]\n%d\nreproj - tracking',mean(errXY{1}),mean(errXY{2}),mean(errXY{3}),mean(errXY{4}),mean(errXY{5}),mean(errXY{6}),mean(errXY{7}),length(errXY{minIndX}(:,1))));
            end
            %             figure(16),clf;imshow(zeros(240,320));hold on;plot(curPtIcs(idErrMax2,1),curPtIcs(idErrMax2,2),'+r');
            drawnow;
            inputDir = fullfile(pwd,'temp');
            if 0
                saveas(gcf,fullfile(inputDir,sprintf('img_%05d.png',length(dir(fullfile(inputDir,'img_*.png')))+1)));
            end
            
            [minDisX2,minIndX2] = sort(errX,'ascend');
            
            % % %              if k2cBodyRotAng2 < 0
            % % %                  k2cBodyRotAng2 = k2cBodyRotAngCand(minInd2-1);
            % % %              else
            % % %                  k2cBodyRotAng2 = k2cBodyRotAngCand(minInd2+1);
            % % %              end
            
            
            %               if k2cBodyRotAng3 ~= k2cBodyRotAng2
            %                   k2cBodyRotAng2 = k2cBodyRotAng3;
            %                   k2cBodyRotAng2 = k2cBodyRotAngCand(I(minIndX2(3)));
            %
            % %                   inputDir = fullfile(pwd,'temp');
            % %                   saveas(gcf,fullfile(inputDir,sprintf('img_%05d.png',length(dir(fullfile(inputDir,'img_*.png')))+1)));
            %               else
            %                   k2cBodyRotAng2 = k2cBodyRotAngCand(I(minIndX2(2)));
            %               end
            %
            if minInd2_check == 4
                k2cBodyRotAng3 = k2cRef0;
            end
            
            if 0
                figure(914);plot(k2cBodyRotAng2,0,'*g');legend(num2str(k2cBodyRotAng2));drawnow;
            end
        end
        
        function [inLierFlag] = DeoutLier_PNP(obj,theta, intrMat,featPtManager,coordSysAligner)
            b2cPmat = GetPctBody2Cam(coordSysAligner, 1);
            k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(theta),zeros(3,1));
            k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
            [keyPtIcs, curPtIcs, keyPtCcsZ] = GetFeaturePairs(featPtManager, 'key', 'last');
            activeFlag = keyPtCcsZ > 0 &  keyPtCcsZ < 14500;2000;
            metricPrevPtCcs = intrMat\HomoCoord(keyPtIcs',1);
            metricPrevPtCcs = normc(metricPrevPtCcs);
            scale = keyPtCcsZ./metricPrevPtCcs(3,:)';
            keyCcsXYZ = scale.*metricPrevPtCcs';
            homocurrCcsXYZ = k2cCamPmat.transformMat*HomoCoord(keyCcsXYZ',1);
            currCcsXYZ = homocurrCcsXYZ(1:3,:);
            outLierThreshold = obj.configParam.pure_rot_reproject_outlier_threshold;
            [~,~,inLierFlag] = VisualLocalizer.CalculateReprojectError(intrMat,currCcsXYZ',curPtIcs,[],[],activeFlag,outLierThreshold);
        end
        
        function [k2cBodyRotAng] = RotationAngleEstimate_MinEigenMethod(obj,intrMat,featPtManager,coordSysAligner)
            
        end
        
        function [inLierFlag] = DeoutLier_MinEigenMethod(obj,theta, intrMat,featPtManager,coordSysAligner)
            
        end
        
        function [k2cBodyRotAng] = pnpLite(obj,intrMat,featPtManager,coordSysAligner,k2cRef,keyFrameFlagList,frmStampList,goldenPose,pixKey ,pt2dCur, DispRefine, camModel, scaleLvl)
            
            
            if isempty(obj) 
                obj.configParam.pnp_ang_est_max_margin = deg2rad(1);
                obj.configParam.pnp_ang_est_min_angle_grid = deg2rad(0.01);
            end
            
            
            [princpPtL, princpPtR] = Get(camModel, 'PinholePrincpPt', scaleLvl);
            depthListGTComp = intrMat(1,1).*norm(camModel.transVec1To2)./(DispRefine + (princpPtR(1) - princpPtL(1)));
            
            %             [keyPtIcs, curPtIcs, keyPtCcsZ] = GetFeaturePairs(featPtManager, 'key', 'last');
            %             activeFlag = keyPtCcsZ > 0 & keyPtCcsZ < 14500;
            
            keyPtIcs = pixKey;  curPtIcs = pt2dCur; keyPtCcsZ = depthListGTComp;
            activeFlag = keyPtCcsZ > 0 & keyPtCcsZ < 14500;2000;
            metricPrevPtCcs = intrMat\HomoCoord(keyPtIcs',1);
            metricPrevPtCcs = normc(metricPrevPtCcs);
            scale = keyPtCcsZ./metricPrevPtCcs(3,:)';
            keyCcsXYZ = scale.*metricPrevPtCcs';
            b2cPmat = GetPctBody2Cam(coordSysAligner, 1);
            maxk2cRotAng = [k2cRef k2cRef] + [-obj.configParam.pnp_ang_est_max_margin obj.configParam.pnp_ang_est_max_margin];
            minAngGrid = obj.configParam.pnp_ang_est_min_angle_grid;
            k2cBodyRotAngCand = maxk2cRotAng(1):minAngGrid:maxk2cRotAng(2);
            numRotAng = length(k2cBodyRotAngCand);
            reProjErr = zeros(numRotAng,1);
            
            
            for i = 1:numRotAng
                k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cBodyRotAngCand(i)),zeros(3,1));
                %                 k2cBodyPmat.transformMat = [rodrigues(k2cBodyRotAngCand(i)*rotAxis) [0 0 0]';0 0 0 1];
                k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
                if 1
                    k2cT = -b2cPmat.transformMat(1:3,1:3)*k2cBodyPmat.transformMat(1:3,1:3)*b2cPmat.transformMat(1:3,1:3)'*b2cPmat.transformMat(1:3,4) + b2cPmat.transformMat(1:3,4);
                    k2cCam = [k2cCamPmat.transformMat(1:3,1:3) k2cT;0 0 0 1];
                    homocurrCcsXYZ = k2cCam*HomoCoord(keyCcsXYZ',1);
                else
                    %                     k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
                    homocurrCcsXYZ = k2cCamPmat.transformMat*HomoCoord(keyCcsXYZ',1);
                end
                currCcsXYZ = homocurrCcsXYZ(1:3,:);
                [reProjErr(i),~,~,err(:,i)] = VisualLocalizer.CalculateReprojectError2(intrMat,currCcsXYZ',curPtIcs,[],[],activeFlag,0);
            end
            [minDis,minInd] = min(reProjErr);
            k2cBodyRotAng = k2cBodyRotAngCand(minInd);
            % % %             for i = 1:numRotAng
            % % %                 k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cBodyRotAngCand(i)),zeros(3,1));
            % % %                 %                 k2cBodyPmat.transformMat = [rodrigues(k2cBodyRotAngCand(i)*rotAxis) [0 0 0]';0 0 0 1];
            % % %                 k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
            % % %                 if 1
            % % %                     k2cT = -b2cPmat.transformMat(1:3,1:3)*k2cBodyPmat.transformMat(1:3,1:3)*b2cPmat.transformMat(1:3,1:3)'*b2cPmat.transformMat(1:3,4) + b2cPmat.transformMat(1:3,4);
            % % %                     k2cCam = [k2cCamPmat.transformMat(1:3,1:3) k2cT;0 0 0 1];
            % % %                     homocurrCcsXYZ = k2cCam*HomoCoord(keyCcsXYZ',1);
            % % %                 else
            % % %                     %                     k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
            % % %                     homocurrCcsXYZ = k2cCamPmat.transformMat*HomoCoord(keyCcsXYZ',1);
            % % %                 end
            % % %                 currCcsXYZ = homocurrCcsXYZ(1:3,:);
            % % %                 [reProjErr(i),~,~,err(:,i)] = VisualLocalizer.CalculateReprojectError2(intrMat,currCcsXYZ',curPtIcs,[],[],activeFlag,0);
            % % %             end
            % % %             [minDis,minInd] = min(reProjErr);
            % % %             k2cBodyRotAng = k2cBodyRotAngCand(minInd);
        end
        
    end
    
    
    methods (Static)
        function [k2cBodyRotAng, xOfst_k2cRef,angStep, minInd] = pnpLite_1(obj,intrMat,featPtManager,coordSysAligner,k2cRef,keyFrameFlagList,frmStampList,goldenPose,pixKey ,pt2dCur, DispRefine, camModel, scaleLvl)
            
            
            if isempty(obj) 
                obj.configParam.pnp_ang_est_max_margin = deg2rad(1);
                obj.configParam.pnp_ang_est_min_angle_grid = deg2rad(0.005);
            end
            angStep = deg2rad(0.01);
            
            [princpPtL, princpPtR] = Get(camModel, 'PinholePrincpPt', scaleLvl);
            depthListGTComp = intrMat(1,1).*norm(camModel.transVec1To2)./(DispRefine + (princpPtR(1) - princpPtL(1)));
            
            %             [keyPtIcs, curPtIcs, keyPtCcsZ] = GetFeaturePairs(featPtManager, 'key', 'last');
            %             activeFlag = keyPtCcsZ > 0 & keyPtCcsZ < 14500;
            
            keyPtIcs = pixKey;  curPtIcs = pt2dCur; keyPtCcsZ = depthListGTComp;
            activeFlag = keyPtCcsZ > 0 & keyPtCcsZ < 14500;2000;
            metricPrevPtCcs = intrMat\HomoCoord(keyPtIcs',1);
            metricPrevPtCcs = normc(metricPrevPtCcs);
            scale = keyPtCcsZ./metricPrevPtCcs(3,:)';
            keyCcsXYZ = scale.*metricPrevPtCcs';
            b2cPmat = GetPctBody2Cam(coordSysAligner, 1);
            maxk2cRotAng = [k2cRef k2cRef] + [-obj.configParam.pnp_ang_est_max_margin obj.configParam.pnp_ang_est_max_margin];
            minAngGrid = obj.configParam.pnp_ang_est_min_angle_grid;
            k2cBodyRotAngCand = maxk2cRotAng(1):minAngGrid:maxk2cRotAng(2);
            numRotAng = length(k2cBodyRotAngCand);
            reProjErr = zeros(numRotAng,1);
            
            
            for i = 1:numRotAng
                k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cBodyRotAngCand(i)),zeros(3,1));
                %                 k2cBodyPmat.transformMat = [rodrigues(k2cBodyRotAngCand(i)*rotAxis) [0 0 0]';0 0 0 1];
                k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
                if 1
                    k2cT = -b2cPmat.transformMat(1:3,1:3)*k2cBodyPmat.transformMat(1:3,1:3)*b2cPmat.transformMat(1:3,1:3)'*b2cPmat.transformMat(1:3,4) + b2cPmat.transformMat(1:3,4);
                    k2cCam = [k2cCamPmat.transformMat(1:3,1:3) k2cT;0 0 0 1];
                    homocurrCcsXYZ = k2cCam*HomoCoord(keyCcsXYZ',1);
                else
                    %                     k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
                    homocurrCcsXYZ = k2cCamPmat.transformMat*HomoCoord(keyCcsXYZ',1);
                end
                currCcsXYZ = homocurrCcsXYZ(1:3,:);
                [reProjErr(i),~,~,err(:,i),~,meanXOfst(i,1)] = VisualLocalizer.CalculateReprojectError2(intrMat,currCcsXYZ',curPtIcs,[],[],activeFlag,0);
            end
            [minDis,minInd] = min(reProjErr);
            k2cBodyRotAng = k2cBodyRotAngCand(minInd);
            xOfst_k2cRef = meanXOfst((length(meanXOfst)+1)/2);
            xOfst_k2cPnp = meanXOfst(minInd);
            
            
            % % %             for i = 1:numRotAng
            % % %                 k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cBodyRotAngCand(i)),zeros(3,1));
            % % %                 %                 k2cBodyPmat.transformMat = [rodrigues(k2cBodyRotAngCand(i)*rotAxis) [0 0 0]';0 0 0 1];
            % % %                 k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
            % % %                 if 1
            % % %                     k2cT = -b2cPmat.transformMat(1:3,1:3)*k2cBodyPmat.transformMat(1:3,1:3)*b2cPmat.transformMat(1:3,1:3)'*b2cPmat.transformMat(1:3,4) + b2cPmat.transformMat(1:3,4);
            % % %                     k2cCam = [k2cCamPmat.transformMat(1:3,1:3) k2cT;0 0 0 1];
            % % %                     homocurrCcsXYZ = k2cCam*HomoCoord(keyCcsXYZ',1);
            % % %                 else
            % % %                     %                     k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
            % % %                     homocurrCcsXYZ = k2cCamPmat.transformMat*HomoCoord(keyCcsXYZ',1);
            % % %                 end
            % % %                 currCcsXYZ = homocurrCcsXYZ(1:3,:);
            % % %                 [reProjErr(i),~,~,err(:,i)] = VisualLocalizer.CalculateReprojectError2(intrMat,currCcsXYZ',curPtIcs,[],[],activeFlag,0);
            % % %             end
            % % %             [minDis,minInd] = min(reProjErr);
            % % %             k2cBodyRotAng = k2cBodyRotAngCand(minInd);
        end
    end
    %     methods
    %
    %         function RefineZ(obj,intrMat,featPtManager,coordSysAligner,k2cRef,keyFrameFlagList,frmStampList,goldenPose)
    %
    %             [keyPtIcs, curPtIcs, keyPtCcsZ] = GetFeaturePairs(featPtManager, 'key', 'last');
    %             activeFlag = keyPtCcsZ > 0 & keyPtCcsZ < 14500;
    %             metricPrevPtCcs = intrMat\HomoCoord(keyPtIcs',1);
    %             metricPrevPtCcs = normc(metricPrevPtCcs);
    %             scale = keyPtCcsZ./metricPrevPtCcs(3,:)';
    %             keyCcsXYZ = scale.*metricPrevPtCcs';
    %             b2cPmat = GetPctBody2Cam(coordSysAligner, 1);
    %             maxk2cRotAng = [k2cRef k2cRef] + [-obj.configParam.pnp_ang_est_max_margin obj.configParam.pnp_ang_est_max_margin];
    %             minAngGrid = obj.configParam.pnp_ang_est_min_angle_grid;
    %             k2cBodyRotAngCand = maxk2cRotAng(1):minAngGrid:maxk2cRotAng(2);
    %             numRotAng = length(k2cBodyRotAngCand);
    %             reProjErr = zeros(numRotAng,1);
    %
    %
    %             for i = 1:numRotAng
    %                 k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cBodyRotAngCand(i)),zeros(3,1));
    % %                 k2cBodyPmat.transformMat = [rodrigues(k2cBodyRotAngCand(i)*rotAxis) [0 0 0]';0 0 0 1];
    %                 k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
    %                 if 1
    %                     k2cT = -b2cPmat.transformMat(1:3,1:3)*k2cBodyPmat.transformMat(1:3,1:3)*b2cPmat.transformMat(1:3,1:3)'*b2cPmat.transformMat(1:3,4) + b2cPmat.transformMat(1:3,4);
    %                     k2cCam = [k2cCamPmat.transformMat(1:3,1:3) k2cT;0 0 0 1];
    %                     homocurrCcsXYZ = k2cCam*HomoCoord(keyCcsXYZ',1);
    %                 else
    %                     %                     k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
    %                     homocurrCcsXYZ = k2cCamPmat.transformMat*HomoCoord(keyCcsXYZ',1);
    %                 end
    %                 currCcsXYZ = homocurrCcsXYZ(1:3,:);
    %                 [reProjErr(i),~,~,err(:,i)] = VisualLocalizer.CalculateReprojectError2(intrMat,currCcsXYZ',curPtIcs,[],[],activeFlag,0);
    %             end
    %
    %
    %
    %
    %         end
    %
    %
    %
    %     end
    methods
        function SetDefaultValue(obj, cfgParam)
            
            cfgParam = Configurable.SetField(cfgParam, 'pnp_ang_est_max_margin', 3/180*pi);
            cfgParam = Configurable.SetField(cfgParam, 'pnp_ang_est_min_angle_grid', 0.01*pi/180);
            cfgParam = Configurable.SetField(cfgParam, 'pure_rot_reproject_outlier_threshold', 2);
            cfgParam = Configurable.SetField(cfgParam, 'init_angle_range', deg2rad([-3 3]));
            obj.configParam = cfgParam;
        end
        
        function CheckConfig(obj) %#ok<MANU>
        end
    end
end