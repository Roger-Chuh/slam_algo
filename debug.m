classdef debug  < handle
   properties
        H;
        background;
        xmin = 444;
        ymin = 138;
        xmax = 990;
        ymax = 749;
   end
    
   methods
       function obj = debug()
           thetaX = rad2deg(0.007988);
           thetaY = rad2deg(9e-5);
           thetaZ = rad2deg(0.000863);
           
           thetaX = rad2deg(0);
           thetaY = rad2deg(0);
           thetaZ = rad2deg(0);
           w2bR = rotz(thetaZ)*roty(thetaY)*rotx(thetaX);
           v = angle2rod(thetaZ,thetaY, thetaX)
           pin = [-15.2292 - 1.05 + 0.15, 15.735 + 1.2 - 0.15;
               -15.1109 - 1.05 + 0.15, -16.35 + 1.2 - 0.15;
               14.1552 - 1.05 + 0.15, 15.1555+1.2 - 0.15;
               14.1264 - 1.05 + 0.15, -17.1758+1.2 - 0.15;
               ];

%            obj.background = imread('background.png');

           obj.background = imread('background_3.jpg');
            
           [rows, cols, chennels] = size(obj.background);
           
           img1 = obj.background;
           img1(round(rows/2)+1:end,1:end,1:3) = 0;
           img1(1:end,round(cols/2)+1:end,1:3) = 0;
%            figure,imshow(img1)
           
           img2 = obj.background;
           img2(1:round(rows/2),1:end,1:3) = 0;
           img2(1:end,round(cols/2)+1:end,1:3) = 0;
%            figure,imshow(img2)
           
           img3 = obj.background;
           img3(round(rows/2):end,1:end,1:3) = 0;
           img3(1:end,1:round(cols/2)+1,1:3) = 0;
%            figure,imshow(img3)
           
           
           img4 = obj.background;
           img4(1:round(rows/2),1:end,1:3) = 0;
           img4(1:end,1:round(cols/2),1:3) = 0;
%            figure,imshow(img4)


                     
           [pout1,~] = detectCheckerboardPoints(img1);
           [pout2,~] = detectCheckerboardPoints(img2);
           [pout3,~] = detectCheckerboardPoints(img3);
           [pout4,~] = detectCheckerboardPoints(img4);
           
           pout = [mean(pout1); mean(pout2);mean(pout3);mean(pout4);];
           obj.H = debug.homography_solve(pin', pout');
           Temp = obj.H*HomoCoord(pin',1);
           P = [Temp(1,:)./Temp(3,:);Temp(2,:)./Temp(3,:)]';
           figure, imshow(obj.background)
           hold on
           plot(P(:,1),P(:,2),'r*')
           plot(pout(:,1),pout(:,2),'go')
           hold off;
           obj.background = imcrop(obj.background,[obj.xmin, obj.ymin, obj.xmax - obj.xmin, obj.ymax - obj.ymin]);
           
       end
       
   end
   
   methods 
       function debug_show_local_planner(obj, vr ,vsl, lp, figNum)
           figure(figNum), clf
           imshow(obj.background);hold on;
           if ~isempty(vsl.pointCloudManager.pt3D)
               obstacle = [vsl.pointCloudManager.pt3D.Location(:,1),vsl.pointCloudManager.pt3D.Location(:,3)];
               obstacle = round(obstacle*20)/20;
               obstacle = unique(obstacle,'rows');
               position = vsl.poseWcsList(end,:);
               position(3) = position(3)+pi/2;
               trace = [vsl.poseWcsList(:,1),vsl.poseWcsList(:,2)];
               
               obstacle = debug.CalHomoTransform(obstacle,obj.H);
               trace = debug.CalHomoTransform(trace,obj.H);
               

               tri = VisualizeResult.VisionWindow(position, vr.maxDepth, vr.fovx);
               tri = debug.CalHomoTransform(tri,obj.H);
%                nextRoadMap = debug.CalHomoTransform(lp.nextRoadMap(1:2),obj.H);
               
               plot(obstacle(:,1),obstacle(:,2),'.b'); hold on;
               plot(trace(:,1),trace(:,2),'-g'),hold on;
               plot(trace(end,1),trace(end,2),'or'),hold on;
               plot([tri(1:end,1);tri(1,1)],[tri(1:end,2);tri(1,2)],'-r');hold on;
%                plot(nextRoadMap(1),nextRoadMap(2),'xr');
%                axis([-5 5 -1 10])
               
           else
               obstacle = [];      
               position = vsl.poseWcsList(end,:);
               position(3) = position(3)+pi/2;
               trace = [vsl.poseWcsList(:,1),vsl.poseWcsList(:,2)];
               
               trace = debug.CalHomoTransform(trace,obj.H);
               

               tri = VisualizeResult.VisionWindow(position, vr.maxDepth, vr.fovx);
               tri = debug.CalHomoTransform(tri,obj.H);
%                nextRoadMap = debug.CalHomoTransform(lp.nextRoadMap(1:2),obj.H);
               
               plot(trace(:,1),trace(:,2),'-g'),hold on;
               plot(trace(end,1),trace(end,2),'or'),hold on;
               plot([tri(1:end,1);tri(1,1)],[tri(1:end,2);tri(1,2)],'-r');hold on;
%                plot(nextRoadMap(1),nextRoadMap(2),'xr');
           end
           
       end
       
       function debug_show_local_planner2(obj, vr ,vsl, figNum)
           figure(figNum), clf
           imshow(obj.background);hold on;
           if ~isempty(vsl.pointCloudManager.map2D)
               obstacle = vsl.pointCloudManager.map2D;
%                obstacle = round(obstacle*20)/20;
%                obstacle = unique(obstacle,'rows');
               position = vsl.poseWcsList(end,:);
               position(3) = position(3)+pi/2;
               trace = [vsl.poseWcsList(:,1),vsl.poseWcsList(:,2)];
               
               obstacle = debug.CalHomoTransform(obstacle,obj.H);
               trace = debug.CalHomoTransform(trace,obj.H);
               
               
               tri = VisualizeResult.VisionWindow(position, vr.maxDepth, vr.fovx);
               tri = debug.CalHomoTransform(tri,obj.H);
%                nextRoadMap = debug.CalHomoTransform(lp.nextRoadMap(1:2),obj.H);


               
               plot(obstacle(:,1) - obj.xmin,obstacle(:,2) - obj.ymin,'.b'); hold on;
               plot(trace(:,1) - obj.xmin,trace(:,2) - obj.ymin,'-g','LineWidth',4),hold on;
               plot(trace(end,1) - obj.xmin,trace(end,2) - obj.ymin,'or'),hold on;
               plot([tri(1:end,1);tri(1,1)]-obj.xmin,[tri(1:end,2);tri(1,2)] - obj.ymin,'-r');hold on;
%                plot(nextRoadMap(1),nextRoadMap(2),'xr');
               %                axis([-5 5 -1 10])
               
           else
               obstacle = [];
               position = vsl.poseWcsList(end,:);
               position(3) = position(3)+pi/2;
               trace = [vsl.poseWcsList(:,1),vsl.poseWcsList(:,2)];
               
               trace = debug.CalHomoTransform(trace,obj.H);
               
               
               tri = VisualizeResult.VisionWindow(position, vr.maxDepth, vr.fovx);
               tri = debug.CalHomoTransform(tri,obj.H);
%                nextRoadMap = debug.CalHomoTransform(lp.nextRoadMap(1:2),obj.H);
               
               plot(trace(:,1) - obj.xmin,trace(:,2) - obj.ymin,'-g','LineWidth',4),hold on;
               plot(trace(end,1) - obj.xmin,trace(end,2) - obj.ymin,'or'),hold on;
               plot([tri(1:end,1);tri(1,1)]-obj.xmin,[tri(1:end,2);tri(1,2)] - obj.ymin,'-r');hold on;
%                plot(nextRoadMap(1),nextRoadMap(2),'xr');
           end
           
       end
   end
   
   methods (Static)
       function debug_915_compare_length_between_vsl_and_bsl(vsl, bsl, motionState, DEBUG)
           if DEBUG
               if Rows(vsl.poseWcsList) > 2
                  vslDeltaX = diff(vsl.poseWcsList(:,1));
                  vslDeltaY = diff(vsl.poseWcsList(:,2));
                  vslL = sqrt(vslDeltaX.^2 + vslDeltaY.^2);
                  bslDeltaX = diff(bsl.poseWcsList(:,1));
                  bslDeltaY = diff(bsl.poseWcsList(:,2));
                  bslL = sqrt(bslDeltaX.^2 + bslDeltaY.^2);
                  deltaL = vslL - bslL;
                  flag = motionState == 3;
                  deltaL(flag) = 0;
                  figure(901501),plot(cumsum(deltaL))
               end
           end
       end
       
       function debug_914_compare_goldenData(vsl,bsl,goldenData,bslTimeStamp,vslTimeStamp, motionState, DEBUG)
           if DEBUG
%                r1 = [];
%                r2 = [];
%                r3 = [];
%                for i = 1:Rows(goldenData)
%                    [r1(i),r2(i),r3(i)]=quat2angle([goldenData(i,4) goldenData(i,5) goldenData(i,6) goldenData(i,7)],'ZYX');
%                end
%                r3 = rad2deg(r3);
%                figure(901401), plot(-goldenData(:,2),goldenData(:,1),'-g');hold on; plot(vsl.poseWcsList(:,1),vsl.poseWcsList(:,2),'-b'); plot(bsl.poseWcsList(:,1),bsl.poseWcsList(:,2),'-r'),axis equal
%                hold off;
%                figure(901402), plot(bslTimeStamp(:,2),r3,'-g');hold on; plot(vslTimeStamp(:,2),rad2deg(bsl.poseWcsList(:,3)),'-r');plot(vslTimeStamp(:,2),rad2deg(vsl.poseWcsList(:,3)),'-b')
%                hold off;


               figure(901403), plot(rad2deg(vsl.poseWcsList(:,3)-bsl.poseWcsList(:,3)),'-r');
%                figure(901404), plot(rad2deg(r1))
%                figure(901405), plot(rad2deg(r2))
%                figure(901406), plot(r3)               
%                figure(901407), plot(bslTimeStamp(:,4))
%                figure(901408), plot(bslTimeStamp(:,5))
           end
       end
       
       function debug_900x_PredictRotateAngle(obj,intrMat,prevFeatPtList,predPtIcs, DEBUG)
           if DEBUG
               addpath('C:\Users\lingc\Desktop\nextvpu\nextvpu_program\SLAM\MLPnP_matlab_toolbox-master\ASPnP')
               figure(90001);
               figure(90001), title(sprintf('current'))
               hold on
               imshow(obj.currImgL)
               hold on
               plot(predPtIcs(:,1),predPtIcs(:,2),'or')
               
               
               figure(90002);
               figure(90002), title(sprintf('previous'))
               hold on
               imshow(obj.prevImgL)
               hold on
               plot(prevFeatPtList(:,1),prevFeatPtList(:,2),'or')
               
               
               figure(90003);
               figure(90003), title(sprintf('prev2curr'))
               showMatchedFeatures(obj.prevImgL,obj.currImgL,prevFeatPtList,predPtIcs);
               
               prevDepthMap = GetDepthMap(obj, obj.prevImgL, obj.prevImgR);
               prevPixInd = sub2ind(size(prevDepthMap), round(prevFeatPtList(:,2)), round(prevFeatPtList(:,1)));
               prevPtCcsZ = prevDepthMap(prevPixInd);
               activeInd = find(prevPtCcsZ>0);
               
               metricKey = intrMat\HomoCoord(prevFeatPtList(activeInd,:)',1);
               metricKey = normc(metricKey);
               scale = prevPtCcsZ(activeInd)./metricKey(3,:)';
               prevCcsXYZ = scale.*metricKey';
               [p2cR,c2pT] = ASPnP(prevCcsXYZ',predPtIcs(activeInd,:)',intrMat);
               if ~isempty(p2cR)||~isempty(c2pT)
                   reProjxyz = (p2cR*prevCcsXYZ' + repmat(c2pT,1,size(prevCcsXYZ,1)))';
                   reprojPix = intrMat*reProjxyz';
                   reprojPix = [reprojPix(1,:)./reprojPix(3,:);reprojPix(2,:)./reprojPix(3,:)];
                   figure(90004),clf;plot(reprojPix(1,:),reprojPix(2,:),'o');hold on;plot(predPtIcs(activeInd,1),predPtIcs(activeInd,2),'x');
                   figure(90004), title(sprintf('prev2currReproject'))
                   figure(90005),clf;plot(reprojPix(1,:)-predPtIcs(activeInd,1)',reprojPix(2,:)-predPtIcs(activeInd,2)','+r');
                   figure(90005), title(sprintf('prev2currReprojectError'))
               else
                   warning('debug warning cannot solve p2c R T')
               end
           end
       end
       
       function debug_901x_RotationAngleAmbiguity(camTransDirAngGrid,rotAngGrid,objVal,inPolygonFlag,k2cCamTransDirAng,k2cRotAng, DEBUG)
           if DEBUG
               try
                   couldMapVal = ones(size(camTransDirAngGrid));
                   couldMapVal(inPolygonFlag)=objVal'/max(objVal);
                   figure(90101)
                   contourf(camTransDirAngGrid,rotAngGrid,couldMapVal,15)
                   colorbar
                   hold on
                   plot(k2cCamTransDirAng,k2cRotAng,'or')
                   hold off
               catch
                   warning('debug_901x_RotationAngleAmbiguity');
               end
           end
       end
       
       function debug_902x_PureRotationTest(k2cBodyRotAng,b2cPmat,keyCcsXYZ,intrMat,activeInd,curPtIcs)
%            k2cBodyPmat = PointCoordTransformer(BaseLocalizer.RotMatFromAngle(k2cBodyRotAng),zeros(3,1));
%            k2cCamPmat = b2cPmat*k2cBodyPmat/b2cPmat;
%            homocurrCcsXYZ = k2cCamPmat.transformMat*HomoCoord(keyCcsXYZ',1);
%            currCcsXYZ = homocurrCcsXYZ(1:3,:);
%            metricCurrPtCcs = intrMat*currCcsXYZ;
%            reprojPtCurrIcs = [metricCurrPtCcs(1,:)./metricCurrPtCcs(3,:);metricCurrPtCcs(2,:)./metricCurrPtCcs(3,:)];
%            reprojPtCurrIcs = reprojPtCurrIcs';
%            figure(90201),clf;plot(curPtIcs(activeInd,1),curPtIcs(activeInd,2),'o');hold on;plot(reprojPtCurrIcs(activeInd,1),reprojPtCurrIcs(activeInd,2),'x');
%            figure(90201), title(sprintf('pureRotationKey2currReproject'))
%            figure(90202),clf;plot(curPtIcs(activeInd,1)-reprojPtCurrIcs(activeInd,1),curPtIcs(activeInd,2)-reprojPtCurrIcs(activeInd,2),'+')
%            figure(90202), title(sprintf('pureRotationKey2currReprojectError'))
       end
       
       function debug_903x_FindThetaAlphaScale(obj,k2cRotAng,k2cCamTransDirAng,k2cScaleFactor,anglePairID,keyPtIcs,intrMat,flag,keyPtCcsZ,curPtIcs)
%            rotMatCcs = BodyRotateRotMatCcs(obj.coordSysAligner, k2cRotAng, 1);
%            transVecCcs = -R(GetPctBody2Cam(obj.coordSysAligner))'*[cos(k2cCamTransDirAng); 0; sin(k2cCamTransDirAng)];
%            numberPt = Rows(keyPtIcs(flag(:,anglePairID),:));
%            metricKey = (intrMat\[keyPtIcs(flag(:,anglePairID),:) ones(numberPt,1)]')';
%            metricKey = normr(metricKey);
%            scale = keyPtCcsZ(flag(:,anglePairID))./metricKey(:,3);
%            xyz = scale.*metricKey;
%            reProjxyz = (rotMatCcs*xyz' + k2cScaleFactor.*repmat(transVecCcs,1,numberPt))';
%            reprojPix = intrMat*reProjxyz';           
%            reprojPix = [reprojPix(1,:)./reprojPix(3,:);reprojPix(2,:)./reprojPix(3,:)];
%            figure(90301),clf;plot(reprojPix(1,:),reprojPix(2,:),'o');hold on;plot(curPtIcs(flag(:,anglePairID),1),curPtIcs(flag(:,anglePairID),2),'x');
%            figure(90302),clf;plot(reprojPix(1,:)-curPtIcs(flag(:,anglePairID),1)',reprojPix(2,:)-curPtIcs(flag(:,anglePairID),2)','+')
       end
       
       function debug_904x_FindDebugCase(flag1, flag2, flag3)
%             if flag1||flag2||flag3
%                 warning('flag is on! the code is paused!')
%             end
       end
       
       function debug_905x_CheckVisionAndBodyP2CRotAng(deltaVision2BodySensor)
%            if deltaVision2BodySensor > 0.1
%                warning('the difference p2c angle between vision and bodysensor is too large!')
%            end
       end
       
       function debug_906x_CheckVisionAndBodyK2CRotAng(obj,k2cRotAng,p2cBodySensorRotAng, DEBUG)
           if DEBUG
               keyPoseWcs = obj.keyRefRobotPoseWcs;
               prevPoseWcs = obj.poseWcsList(end,:);
               curRorAngWcs = p2cBodySensorRotAng+prevPoseWcs(3);
               k2cBodySensorRotAng = curRorAngWcs - asin(keyPoseWcs.transformMat(1,3));
               deltaAng = rad2deg(k2cBodySensorRotAng - k2cRotAng);
               if deltaAng > 0.1
                   warning('k2c Rot Ang is too large!') ;
               end
           end
       end
       
       function debug_907x_CheckVisionAndBodyK2CCamTransDirAng(obj,k2cCamTransDirAng, DEBUG)
           if DEBUG
               try
                   k2wPBody = WcsPoseVecToPct(obj,obj.keyRefRobotPoseWcs);
                   c2wPBody = obj.currRefRobotPoseWcs;
                   k2cPBody = c2wPBody/k2wPBody;
                   b2cP = GetPctBody2Cam(obj.coordSysAligner, 1);
                   b2cP.transformMat(1:3,4) = b2cP.transformMat(1:3,4)/1000;
                   k2cPCam = b2cP*k2cPBody/b2cP;
                   k2cTCam = T(k2cPCam);
                   k2cTCamDir = atan(k2cTCam(3)/k2cTCam(1));
                   
                   theta = obj.currRefRobotRotAngWcs - obj.keyRefRobotPoseWcs(3);
                   alpha = k2cTCamDir - theta;
                   delta = abs(rad2deg(alpha - k2cCamTransDirAng));
                   
                   figure(90101)
                   hold on
                   plot(alpha,theta,'*g')
                   hold off
                   if delta > 0.1
                       warning('k2c alpha is too large!')
                   end
               catch
                   warning('debug_907x_CheckVisionAndBodyK2CCamTransDirAng')
               end
           end
       end
       
       function debug_908x_CheckPolygonAndLineInteraction(flag)
          if flag == 0
              warning('there is no interaction between line and polygon!')
          end
       end
       
       function debug_909x_keyFrameBCS_PolygonAndBodyTraceInteractgion(obj,curBodyOrigUncertPolygonBcsKey,robotPoseWcs, DEBUG)
           if DEBUG
               figure(90901)
               hold on
               plot(curBodyOrigUncertPolygonBcsKey(1,[1:end 1]),curBodyOrigUncertPolygonBcsKey(3,[1:end 1]),'-xg')
               lastKeyInd = GetLastKeyFrameInd(obj);
               k2wP =  WcsPoseVecToPct(obj, obj.poseWcsList(lastKeyInd,:));
               vPose = Inv(k2wP)*[robotPoseWcs(1);0;robotPoseWcs(2)];
               plot(vPose(1),vPose(3),'-*b');
               
               bPose = Inv(obj.keyRefRobotPoseWcs)*[obj.refRobotPosVec(1);0;obj.refRobotPosVec(2)];
               plot(bPose(1),bPose(3),'-or');
           end
       end
       
       function debug_910x_ShowKeyFrameLocation(obj, doInsert)
           
%            global PATH_FIG
%            figure(PATH_FIG)
%            hold on
%            if (doInsert == obj.INSERT_KEYFRAME_PREVIOUS)
%                plot(-obj.poseWcsList(end-1,1),obj.poseWcsList(end-1,2),'*b')
%            elseif (doInsert == obj.INSERT_KEYFRAME_CURRENT)
%                plot(-obj.poseWcsList(end,1),obj.poseWcsList(end,2),'*b')
%            end
%            hold off
           
       end
       
       function debug_911x_CheckRotationAngle(obj,frameInd)
%            figure(91101)
%            hold on
%            if Rows(obj.poseWcsList) < 2
%                plot(frameInd, obj.poseWcsList(end,3),'ob');
%                plot(frameInd, obj.currRefRobotRotAngWcs,'+r');
%            else
%                line([frameInd-1,frameInd],...
%                    [obj.poseWcsList(end-1,3),obj.poseWcsList(end,3)],'Color',[0 0 1],'LineWidth',1)
%                line([frameInd-1,frameInd],...
%                    [obj.prevRefRobotRotAngWcs,obj.currRefRobotRotAngWcs],'Color',[1 0 0],'LineWidth',1)
%                plot(frameInd, obj.poseWcsList(end,3),'ob');
%                plot(frameInd, obj.currRefRobotRotAngWcs,'+r');
%
%            end
%            hold off
       end
       
       function debug_912x_CheckAlphaThetaAndCandidateAngle(k2cRotAngGrids, k2cCamTransDirAngGrids, inPolygonFlag, ...
               k2cRotAngCandidates, k2cCamTransDirAngCandidates, DEBUG)
           if DEBUG
               figure(91201)
               title('Check Alpha Theta And Candidate Angle')
               xlabel('theta')
               ylabel('alpha')
               [sampleGridAlpha, sampleGridTheta] = meshgrid(k2cCamTransDirAngGrids, k2cRotAngGrids);
               plot(rad2deg(sampleGridTheta(inPolygonFlag)), rad2deg(sampleGridAlpha(inPolygonFlag)), 'go'); hold on
               plot(rad2deg(k2cRotAngCandidates), rad2deg(k2cCamTransDirAngCandidates), 'r*');               
               hold off
           end
       end
       
       function debug_913x_CheckOpticalFlowPointCloud(obj,prevPt,currPt,prevZ,currZ,goodPointID,dis,DEBUG)
           if DEBUG
               figure(91301),clf
               showMatchedFeatures(obj.prevImgL,obj.currImgL,prevPt(goodPointID,:),currPt(goodPointID,:))
               title('check good point for Optical Flow 1D');
               figure(91302),clf
               histogram(dis(prevZ>0&currZ>0))
               title('distance of the same point from two frames in WCS');
           end
       end
       

       
       function v = homography_solve(pin, pout)
           % HOMOGRAPHY_SOLVE finds a homography from point pairs
           %   V = HOMOGRAPHY_SOLVE(PIN, POUT) takes a 2xN matrix of input vectors and
           %   a 2xN matrix of output vectors, and returns the homogeneous
           %   transformation matrix that maps the inputs to the outputs, to some
           %   approximation if there is noise.
           %
           %   This uses the SVD method of
           %   http://www.robots.ox.ac.uk/%7Evgg/presentations/bmvc97/criminispaper/node3.html
           % David Young, University of Sussex, February 2008
           if ~isequal(size(pin), size(pout))
               error('Points matrices different sizes');
           end
           if size(pin, 1) ~= 2
               error('Points matrices must have two rows');
           end
           n = size(pin, 2);
           if n < 4
               error('Need at least 4 matching points');
           end
           % Solve equations using SVD
           x = pout(1, :); y = pout(2,:); X = pin(1,:); Y = pin(2,:);
           rows0 = zeros(3, n);
           rowsXY = -[X; Y; ones(1,n)];
           hx = [rowsXY; rows0; x.*X; x.*Y; x];
           hy = [rows0; rowsXY; y.*X; y.*Y; y];
           h = [hx hy];
           if n == 4
               [U, ~, ~] = svd(h);
           else
               [U, ~, ~] = svd(h, 'econ');
           end
           v = (reshape(U(:,9), 3, 3)).';
       end
       
       function pout = CalHomoTransform(pin, H)           
           Temp = H*HomoCoord(pin',1);           
           pout = [Temp(1,:)./Temp(3,:);Temp(2,:)./Temp(3,:)]';
       end
       
       function newBsSampFrame = modifySampData(bsSampFrame,curFrameId, frameId1, frameId2,vmin,wmin,a,aw)
           newBsSampFrame = bsSampFrame;
           
           if ~exist('vmin', 'var')
              vmin = 0; 
           end
           
           if ~exist('wmin', 'var')
               wmin = 0;
           end
           
           if ~exist('a', 'var')
               a = 0;
           end
           
           if ~exist('aw', 'var')
               aw = 0;
           end
               
           if curFrameId >= frameId1 && curFrameId <= frameId2
               if a == 0                   
                   newBsSampFrame(:,4) = vmin;
               else
                   vmax =  vmin+0.1*a;
                   v = vmin:(vmax-vmin)/(Cols(newBsSampFrame)-2):vmax;
                   newBsSampFrame(:,4) = v';
               end
               
               if aw == 0
                   newBsSampFrame(:,5) =wmin;
               else
                   wmax = wmin + 0.1 * aw;
                   w = wmin:(wmax-wmin)/(Cols(newBsSampFrame)-2):wmax;
                   newBsSampFrame(:,5) = w';
               end
           end
       end
       
       function pts = getCheckerboardPoints(cx, cy, L)
           pts = [];
           for j = -3:1:3
               for i = -2.5:1:2.5                   
                   pts = [pts; cx+i*L, cy+j*L];                   
               end
           end           
       end
   end
    
    
end