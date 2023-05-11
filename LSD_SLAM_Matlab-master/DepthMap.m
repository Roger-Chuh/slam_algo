classdef DepthMap < handle
    properties
        %intresnic camera parameters
        fx,fy,cx,cy,fxi,fyi,cxi,cyi;
        width, height;
        activeKeyFrame;
        newest_referenceFrame;
        oldest_referenceFrame;
        %depthHypothesis;
        currentDepthMap;
        otherDepthMap;
        K;
        priorDepth;
        validityIntegralBuffer;
    end
    methods
        function obj = DepthMap(K,w, h)
            obj.K = K;
            obj.fx = K(1,1);
            obj.fy = K(2,2);
            obj.cx = K(1,3);
            obj.cy = K(2,3);
            
            KInv = inv(K);
            obj.fxi = KInv(1,1);
            obj.fyi = KInv(2,2);
            obj.cxi = KInv(1,3);
            obj.cyi = KInv(2,3);
            
            obj.width = w;
            obj.height = h;
            
            intialVar = ones(h, w) * globalParams.VAR_GT_INIT_INITIAL;
            isValid = zeros(h, w) ;
            validityCounter = 20 * isValid;
            obj.otherDepthMap = DepthMapPixelHypothesis(zeros(h, w) , zeros(h, w), intialVar, intialVar,validityCounter,isValid);
            obj.validityIntegralBuffer = zeros(h, w);
            
        end
        
        function initializeRandomly(obj, new_frame)
            obj.activeKeyFrame = new_frame;
            %populate random / rgbd depthmap hypothis
        end
        
        
        
        
        function initializeFromGTDepth(obj, new_frame)
            %new_frame.imgZ = ones(size(new_frame.imgRGB,1),size(new_frame.imgRGB,2));
            obj.activeKeyFrame = new_frame;
            
            depth =  new_frame.imgZ;%ones(size(new_frame.imgRGB,1),size(new_frame.imgRGB,2));new_frame.imgZ;
            isValid =  (~isnan( depth) & depth  > 0) ;
            depth = isValid  .*depth;
            intialVar = isValid .* globalParams.VAR_GT_INIT_INITIAL;
            
            validityCounter = isValid .* 20;
            % obj.depthHypothesis = DepthMapPixelHypothesis(depth , depth, intialVar, intialVar);
            obj.currentDepthMap = DepthMapPixelHypothesis(depth , depth, intialVar, intialVar,validityCounter,isValid);
            %obj.currentDepthMap.validity_counter(new_frame.imgZ > 0) = 20;
            %%using this avoid nans
            obj.priorDepth =  obj.currentDepthMap.idepth; %ones(size(obj.depthHypothesis.idepth,1) , size(obj.depthHypothesis.idepth,2));%
            obj.currentDepthMap.visualize(0);
        end
        
        function updateKeyframe(obj, referanceFrames)
            %%%for now previous frame is active frame
            if size(obj.activeKeyFrame,1) == 0
                obj.activeKeyFrame = referanceFrames;
            end
            
            referanceFrames.prepareForStereoWith(obj.activeKeyFrame, inv(obj.activeKeyFrame.getScaledCamToWorld()), obj.K,0);
            obj.newest_referenceFrame = referanceFrames;
            %%obj.activeKeyFrame = referanceFrames;
        end
        %{
function [flag,pepx,pepy] = makeAndCheckEPL(obj,x , y , ref)
          %%%replace with matrix maniuplation
            epx = - obj.fx * ref.thisToOther_t(1) + ref.thisToOther_t(3)*(x - obj.cx);
            epy = - obj.fy * ref.thisToOther_t(2) + ref.thisToOther_t(3)*(y - obj.cy);

            if isNAN(epx+epy)
                flag = false;
            end

            %%======== check epl length =========
            eplLengthSquared = epx*epx+epy*epy;
            if eplLengthSquared < globalParams.MIN_EPL_LENGTH_SQUARED
                flag = false;
            end

            %% ===== check epl-grad magnitude ======
            gx =  obj.activeKeyFrame.image(x + 1,y) - obj.activeKeyFrame.image(x - 1,y) ;
            gy =  obj.activeKeyFrame.image(x, y + 1) - obj.activeKeyFrame.image(x,y - 1) ;
            eplGradSquared = gx * epx + gy * epy;
            eplGradSquared = (eplGradSquared*eplGradSquared) / eplLengthSquared;

           if eplGradSquared < globalParams.MIN_EPL_GRAD_SQUARED
                flag= false;
           end


            %% ===== check epl-grad angle ======
            if (eplGradSquared / (gx*gx+gy*gy)) < globalParams.MIN_EPL_ANGLE_SQUARED
                flag = false;
            end
    

            %% ===== DONE - return "normalized" epl =====
            float fac = GRADIENT_SAMPLE_DIST / sqrt(eplLengthSquared);
            pepx = epx * fac;
            pepy = epy * fac;

            flag = true;

            
        end
        %}
        
        function observeDepthUpdate(obj,x , y , referenceFrame)
            
            [flag,pepx,pepy, x, y ] = makeAndCheckEPLMat(obj,x , y , referenceFrame);
            [result_idepth, result_var, result_eplLength,errors] = obj.doLineStereo(x,y,pepx,pepy,referenceFrame);
            % key frmae depth
            
            
            %// if just not good for stereo (e.g. some inf / nan occured; has inconsistent minimum; ..)
            ids = false(obj.height, obj.width);
            if 0
                ids(3:obj.width-3 , 3 : obj.height - 3) =  (errors == -2);
            else
                ids(3:obj.height-3 , 3 : obj.width - 3) =  (errors == -2);
            end
            obj.currentDepthMap.validity_counter(ids) =  obj.currentDepthMap.validity_counter(ids) - globalParams.VALIDITY_COUNTER_DEC;
            % keyDepthMap
            
            
            ids2 = (ids & (obj.currentDepthMap.validity_counter == 0));
            obj.currentDepthMap.validity_counter(ids2) = 0;
            %target->nextStereoFrameMinID = 0;
            obj.currentDepthMap.idepth_var(ids)= obj.currentDepthMap.idepth_var(ids) * globalParams.FAIL_VAR_INC_FAC;
            ids2 = ids & obj.currentDepthMap.idepth_var > globalParams.MAX_VAR;
            
            obj.currentDepthMap.isValid(ids2)  = false;
            obj.currentDepthMap.blackListed(ids2) = obj.currentDepthMap.blackListed(ids2) - 1;
            
            
            
            
            
            
            % do textbook ekf update:
            %increase var by a little (prediction-uncertainty);
            if 0
                id_var =  obj.currentDepthMap.idepth_var(3:obj.width-3 , 3 : obj.height - 3) .* globalParams.SUCC_VAR_INC_FAC;
            else
                id_var =  obj.currentDepthMap.idepth_var(3:obj.height-3 , 3 : obj.width - 3) .* globalParams.SUCC_VAR_INC_FAC;
            end
            % update var with observation
            w = referenceFrame.validIDs .* (result_var ./ (result_var + id_var));
            
            %% update current depth measurement
            % (1-w)*reCalcedKeyDepth + w*keyRgbdDepth
            if 0
                new_idepth = referenceFrame.validIDs .*((1-w).*result_idepth + w.*obj.currentDepthMap.idepth(3:obj.width-3 , 3 : obj.height - 3)) ;
                diff = (result_idepth - obj.currentDepthMap.idepth_smoothed(3:obj.width-3 , 3 : obj.height - 3));
            else
                new_idepth = referenceFrame.validIDs .*((1-w).*result_idepth + w.*obj.currentDepthMap.idepth(3:obj.height-3 , 3 : obj.width - 3)) ;
                diff = (result_idepth - obj.currentDepthMap.idepth_smoothed(3:obj.height-3 , 3 : obj.width - 3));
            end
            if 0
                figure,imshow(abs(immultiply(diff, result_idepth > 0)), [])
            end
            
            
            % if inconsistent
            ids = false(obj.height, obj.width);
            ids(3:obj.height-3 , 3 : obj.width - 3)  = referenceFrame.validIDs .*((globalParams.DIFF_FAC_OBSERVE.*diff.*diff) > result_var + obj.currentDepthMap.idepth_smoothed(3:obj.height-3 , 3 : obj.width - 3));
            
            obj.currentDepthMap.idepth_var(ids) = obj.currentDepthMap.idepth_var(ids) * globalParams.FAIL_VAR_INC_FAC;
            ids2 = ids & obj.currentDepthMap.idepth_var > globalParams.MAX_VAR;
            obj.currentDepthMap.isValid(ids2) = false;
            
            
            
            
            
            
            %obj.currentDepthMap.idepth = zeros(obj.width, obj.height);
            %obj.currentDepthMap.idepth_smoothed = zeros(obj.width, obj.height);
            
            temp  = zeros(obj.height, obj.width);
            temp(3:obj.height-3 , 3 : obj.width - 3)  = (new_idepth);
            
            ids = false(obj.height, obj.width);
            ids(3:obj.height-3 , 3 : obj.width - 3) =   referenceFrame.validIDs ;
            
            obj.currentDepthMap.idepth(ids) = temp(ids);
            %obj.currentDepthMap.idepth_smoothed(ids) = temp(ids);
            obj.currentDepthMap.isValid  = zeros(obj.height, obj.width);
            obj.currentDepthMap.isValid(3:obj.height-3 , 3 : obj.width - 3)  = referenceFrame.validIDs;
            
            % variance can only decrease from observation; never increase.
            temp =  zeros(obj.height, obj.width);
            temp(3:obj.height-3 , 3 : obj.width - 3) = id_var;
            id_var = temp;
            id_var(3:obj.height-3 , 3 : obj.width - 3)  = (referenceFrame.validIDs .*(id_var(3:obj.height-3 , 3 : obj.width - 3) .* w));
            
            mask = zeros(size(id_var));
            mask(id_var < obj.currentDepthMap.idepth_var) = 1;
            
            temp  = (ids.*(mask.*id_var));
            
            obj.currentDepthMap.idepth_var(ids) = temp(ids);
            
            
            validityInc = referenceFrame.validIDs .*  globalParams.VALIDITY_COUNTER_INC;
            
            obj.currentDepthMap.validity_counter(3:obj.height-3 , 3 : obj.width - 3)  =  obj.currentDepthMap.validity_counter(3:obj.height-3 , 3 : obj.width - 3)  + validityInc;
            
            
            dsrgiuh = 1;
            
            %float absGrad = keyFrameMaxGradBuf[idx];
            %if(target->validity_counter > VALIDITY_COUNTER_MAX+absGrad*(VALIDITY_COUNTER_MAX_VARIABLE)/255.0f)
            %	target->validity_counter = VALIDITY_COUNTER_MAX+absGrad*(VALIDITY_COUNTER_MAX_VARIABLE)/255.0f;
            %{
		%increase Skip!
		if(result_eplLength < MIN_EPL_LENGTH_CROP)
		{
			float inc = activeKeyFrame->numFramesTrackedOnThis / (float)(activeKeyFrame->numMappedOnThis+5);
			if(inc < 3) inc = 3;

			inc +=  ((int)(result_eplLength*10000)%2);

			if(enablePrintDebugInfo) stats->num_observe_addSkip++;

			if(result_eplLength < 0.5*MIN_EPL_LENGTH_CROP)
				inc *= 3;


			target->nextStereoFrameMinID = refFrame->id() + inc;
		}
            %}
            %{
              [zx,zy] = find(new_idepth ~= 0);
    zz = result_idepth(new_idepth ~=0);
    %figure()
    %scatter3(zx./zz, zy./zz, zz);
   
    
    close all;
    figure();
    imshow(referenceFrame.imgRGB)
    figure ; imshow(log(result_var+1))
    figure();
    scale = 50;
   
     camPos = (referenceFrame.thisToOther_t);
    camPos(3) = camPos(3) * scale;
    plotCamera('Location',camPos,'Orientation',referenceFrame.thisToOther_R,'Opacity',0);
    hold on
    
    pcshow([zx,zy,zz * scale],'MarkerSize', 45);
    grid on
    
    pDepth = obj.priorDepth(3:obj.width-3 , 3 : obj.height - 3);
    pDepth =  referenceFrame.validIDs .* pDepth;
            %}
            
        end
        
        function [flag,pepx,pepy, x, y ] = makeAndCheckEPLMat(obj,x , y , ref)
            %%%replace with matrix maniuplation
            %x = reshape(x , [] , 1);
            %y = reshape(y , [] , 1);
            
            %%from c++ implementaion between 1 and 2 frame
            
            %{
            
            ref.thisToOther_t = [-0.0290017705 -0.000712459616 -0.00135277933];
            ref.K_otherToThis_t = [7.71646357 0.702298999 0.00128124538];
            ref.K_otherToThis_R = [253.713959 4.06102848 -0.00221992028; -5.75850105 373.589294 -0.00994038116;267.901886 235.333847 0.999948144]';
            %}
            epx = (- obj.fx * ref.thisToOther_t(1)) + (ref.thisToOther_t(3)*(x - obj.cx));
            epy = (- obj.fy * ref.thisToOther_t(2)) + (ref.thisToOther_t(3)*(y - obj.cy));
            
            
            
            ref.validIDs = ref.validIDs & (~isnan(epx+epy));
            
            epx(~ref.validIDs) = 0;
            epy(~ref.validIDs) = 0;
            %{
             [failedIDs] = (isnan(epx+epy));
             epx(failedIDs) = 0;
             epy(failedIDs) = 0;
             x(failedIDs) = 0;
             y(failedIDs) = 0;
            if size(failedIDs, 1) > 0
                flag = false;
            end
            
            %}
            
            %%======== check epl length =========
            eplLengthSquared = (epx.*epx)+(epy.*epy);
            ref.validIDs = ref.validIDs & (~(eplLengthSquared < globalParams.MIN_EPL_LENGTH_SQUARED ));
            %{
            [failedIDs] = (eplLengthSquared < globalParams.MIN_EPL_LENGTH_SQUARED );
             epx(failedIDs) = 0;
             epy(failedIDs) = 0;
             x(failedIDs) = 0;
             y(failedIDs) = 0;
            if size(failedIDs, 1) > 0
                flag = false;
            end
            %}
            %% ===== check epl-grad magnitude ======
            activeImageGray = double(obj.activeKeyFrame.imgGray);
            [h,w] = size(activeImageGray);
            %activeImageGray = activeImageGray(3:w-3 , 3 :h-3);
            filter = [-1 0 1];
            
            gx =  double(imfilter(activeImageGray,filter));
            gy =  double(imfilter(activeImageGray,filter'));
            gx = gx(3:h - 3 , 3 : w-3);
            gy = gy(3:h - 3 , 3 : w-3);
            %{
            gx =  obj.activeKeyFrame.image(x + 1,y) - obj.activeKeyFrame.image(x - 1,y) ;
            gy =  obj.activeKeyFrame.image(x, y + 1) - obj.activeKeyFrame.image(x,y - 1) ;
            %}
            eplGradSquared = (gx .* epx) + (gy .* epy);
            eplGradSquared = (eplGradSquared.*eplGradSquared)./ eplLengthSquared;
            %{
            [failedX , failedY ] = find( eplGradSquared < globalParams.MIN_EPL_GRAD_SQUARED);
             epx(failedX , failedY) = 0;
             epy(failedX , failedY) = 0;
             
             gx(failedX , failedY)  = 0;
             gy(failedX , failedY)  = 0;
            
            if size(failedX, 1) > 0
                flag = false;
            end
            %}
            
            ref.validIDs = ref.validIDs & ~(eplGradSquared < globalParams.MIN_EPL_GRAD_SQUARED);
            eplGradSquared(~ref.validIDs) = 0;
            eplLengthSquared(~ref.validIDs) = 0;
            
            %{
        [validIDs ] = ( ~(eplGradSquared < globalParams.MIN_EPL_GRAD_SQUARED));
             epx = epx(validIDs);
             epy = epy(validIDs);
             
             gx = gx(validIDs );
             gy = gy(validIDs );
             
             eplGradSquared = eplGradSquared(validIDs);
             eplLengthSquared = eplLengthSquared(validIDs);
             
             x = x(validIDs) ;
             y = y(validIDs) ;
            %}
            
            
            %{
             epx = reshape(epx , [] ,1);
             epy = reshape(epy , [] ,1);
             gx = reshape(gx , [] ,1);
             gy = reshape(gy , [] ,1);
             
             eplGradSquared = reshape(eplGradSquared , [] ,1);
             eplLengthSquared = reshape(eplLengthSquared , [] ,1);
            %}
            %{
            if size(validIDs, 1) > 0
                flag = false;
            end
            %}
            %% ===== check epl-grad angle ======
            ref.validIDs =ref.validIDs & ( ~((eplGradSquared ./ ((gx.*gx)+(gy.*gy))) < globalParams.MIN_EPL_ANGLE_SQUARED));
            
            eplLengthSquared(~ref.validIDs) = 0;
            epx(~ref.validIDs) = 0;
            epy(~ref.validIDs) = 0;
            %{
             [validIDs  ] = ( ~((eplGradSquared ./ ((gx.*gx)+(gy.*gy))) < globalParams.MIN_EPL_ANGLE_SQUARED));
            %{
             if size(validIDs, 1) > 0
                flag = false;
            end
            %}
            eplLengthSquared = eplLengthSquared(validIDs );
            epx = epx(validIDs  );
            epy = epy(validIDs  );
             x = x(validIDs) ;
             y = y(validIDs) ;
            %}
            %% ===== DONE - return "normalized" epl =====
            fac = globalParams.GRADIENT_SAMPLE_DIST ./ sqrt(eplLengthSquared);
            fac(~ref.validIDs) = 0;
            pepx = epx .* fac;
            pepy = epy .* fac;
            
            flag = true;
            %{
            min_idepth = 0;
            prior_idepth = 0.5;
            max_idepth = 1;
            %}
            %obj.doLineStereo(x,y,pepx,pepy,ref)
        end
        
        function flag = isValid(obj)
            flag =  (obj.activeKeyFrame ~=0);
        end
        
        function createKeyFrame(obj,   new_keyframe)
            assert(obj.isValid())
            assert(size(new_keyframe,1) ~= 0)
            assert(new_keyframe.hasTrackingParent());
            
            
            oldToNew_SE3 = inv(new_keyframe.pose.thisToParent_raw); %k2c
            obj.propagateDepth(new_keyframe);
            % otherDepthMap = true cur
            % currentDepthMap = propagateDepth
            
%             temp = copy(obj.currentDepthMap);
%             obj.currentDepthMap = copy(obj.otherDepthMap);
%             obj.otherDepthMap = copy(temp);
            
            
            
            obj.activeKeyFrame = new_keyframe;
            
            obj.regularizeDepthMap(true, globalParams.VAL_SUM_MIN_FOR_KEEP);
            obj.currentDepthMap.isValid(isnan(obj.currentDepthMap.idepth_smoothed))=0;
            obj.regularizeDepthMapFillHoles();
            obj.regularizeDepthMap(false, globalParams.VAL_SUM_MIN_FOR_KEEP);
            obj.currentDepthMap.isValid(isnan(obj.currentDepthMap.idepth_smoothed))=0;
            
            %regularizeDepthMap
            
            %make mean inverse depth be one.
            sumIdepth=sum(sum(obj.currentDepthMap.idepth_smoothed(obj.currentDepthMap.isValid == 1)));
            numIdepth = sum(sum(obj.currentDepthMap.isValid == 1));
            
            rescaleFactor = numIdepth / sumIdepth;
            rescaleFactor2 = rescaleFactor*rescaleFactor;
            obj.currentDepthMap.idepth = obj.currentDepthMap.idepth * rescaleFactor;
            obj.currentDepthMap.idepth_smoothed = obj.currentDepthMap.idepth_smoothed * rescaleFactor;
            obj.currentDepthMap.idepth_var = obj.currentDepthMap.idepth_var * rescaleFactor2;
            obj.currentDepthMap.idepth_var_smoothed = obj.currentDepthMap.idepth_var_smoothed * rescaleFactor2;
            
            obj.activeKeyFrame.pose.thisToParent_raw = inv(oldToNew_SE3);%sim3FromSE3(oldToNew_SE3.inverse(), rescaleFactor);
            %obj.activeKeyFrame.pose.invalidateCache();
            
            % Update depth in keyframe (or should i say prv frame?)
            
            obj.activeKeyFrame.setDepth(obj.currentDepthMap);
            
            viz = obj.currentDepthMap.visualize(1);
            %            close all;
            figure,imshow(viz);
            
        end
        
        function propagateDepth(obj,new_keyframe)
            %wipe depthmap
            obj.otherDepthMap.isValid(:,:) = false;
            obj.otherDepthMap.blackListed(:,:) = 0;
            if 0
                %% 20200803 stay unchanged
                oldToNew_SE3 = inv(new_keyframe.pose.thisToParent_raw);
            else
                %% 20200803
                oldToNew_SE3 = (new_keyframe.pose.thisToParent_raw);
            end
            trafoInv_t = oldToNew_SE3(1:3,4);
            trafoInv_R = oldToNew_SE3(1:3,1:3);
            
            activeKFImageData = obj.activeKeyFrame.imgGray;
            %change it to max gradient
            
            trackingWasGood = 1;
            %trackingWasGood = new_keyframe.getTrackingParent() == activeKeyFrame ? new_keyframe.refPixelWasGoodNoCreate() : 0;
            
            
            filter = [-1 0 1];
            [w,h] = size(activeKFImageData);
            gx =  double(imfilter(activeKFImageData,filter));
            [Gmag,Gdir] = imgradient( new_keyframe.imgGray);
            
            newKFMaxGrad =  Gmag;%gx;%new_keyframe.maxGradients;
            newKFImageData = new_keyframe.imgGray;
            
            [xx , yy] = meshgrid(1:obj.height,1 :obj.width );
            cnt0 = 0; cnt1 = 0;
            % go through all pixels of OLD image, propagating forwards.
            for y= 1:obj.height
                for x= 1: obj.width
                    %source = obj.currentDepthMap(x,y);
                    if(~obj.currentDepthMap.isValid(y,x))
                        cnt0 = cnt0 + 1;
                        continue;
                    end
                    cnt1 = cnt1 + 1;
                    pix = [x y];
                    if 1
%                         pn = trafoInv_R * [x*obj.fxi + obj.cyi,y*obj.fyi + obj.cxi,1.0]' / obj.currentDepthMap.idepth_smoothed(y,x) + trafoInv_t;
                        pn = trafoInv_R * [x*obj.fxi + obj.cxi, y*obj.fyi + obj.cyi,1.0]' / obj.currentDepthMap.idepth_smoothed(y,x) + trafoInv_t;
                    else
                        pn = trafoInv_R * [pix(1)*obj.fxi + obj.cxi,pix(2)*obj.fyi + obj.cyi,1.0]' .* obj.currentDepthMap.idepth_smoothed(y,x) + trafoInv_t;
                    end
                    if 1
                        new_idepth = 1.0 / pn(3);
                        
                        u_new = pn(1)*new_idepth*obj.fx + obj.cx;
                        v_new = pn(2)*new_idepth*obj.fy + obj.cy;
                    else
                        new_idepth = pn(3); %1.0 / pn(3);
                        
                        u_new = pn(1)*obj.fx/new_idepth + obj.cx;
                        v_new = pn(2)*obj.fy/new_idepth + obj.cy;
                        
                        if 0
                            K= [obj.fx 0 obj.cx;0 obj.fy obj.cy; 0 0 1];
                            a = inv(K)*[x; y; 1];
                            tempz = obj.currentDepthMap.idepth_smoothed(y,x);
                            XYZ = tempz*a;
                            b = K*(trafoInv_R*XYZ+trafoInv_t);
                            pflat(b)
                        end
                        
                    end
                    
                    if(~(u_new > 2.1 && v_new > 2.1 && u_new < obj.width-3.1 && v_new < obj.height-3.1))
                        
                        continue;
                    end
                    
                    %newIDX = int32(u_new+0.5) + (int32(v_new+0.5))*obj.width
                    newIDX_X = int32(u_new+0.5);
                    newIDX_y = int32(v_new+0.5);
                    
                    destAbsGrad = newKFMaxGrad(newIDX_y, newIDX_X);
                    
                    if(trackingWasGood ~= 0)
                        
                    else
                        
                        sourceColor = activeKFImageData(y , x);
                        %destColor = getInterpolatedElement(newKFImageData, u_new, v_new, width);
                        
                        destColor = interp2(xx,yy,newKFImageData,u_new,v_new, 'cubic');
                        
                        residual = destColor - sourceColor;
                        
                        
                        if(residual*residual / (globalParams.MAX_DIFF_CONSTANT + globalParams.MAX_DIFF_GRAD_MULT*destAbsGrad*destAbsGrad) > 1.0 || destAbsGrad <globalParams.MIN_ABS_GRAD_DECREASE)
                            
                            continue;
                            
                        end
                    end
                    
                    %targetBest = obj.otherDepthMap(newIDX_X,newIDX_y);
                    
                    %large idepth = point is near = large increase in variance.
                    %small idepth = point is far = small increase in variance.
                    idepth_ratio_4 = new_idepth / obj.currentDepthMap.idepth_smoothed(newIDX_y, newIDX_X);
                    idepth_ratio_4 = idepth_ratio_4 * idepth_ratio_4;
                    idepth_ratio_4 = idepth_ratio_4 * idepth_ratio_4;
                    
                    new_var =idepth_ratio_4*  obj.currentDepthMap.idepth_var(newIDX_y, newIDX_X);
                    
                    
                    
                    
                    % check for occlusion
                    
                    if( obj.otherDepthMap.isValid(newIDX_y, newIDX_X))
                        %if they occlude one another, one gets removed.
                        diff = obj.otherDepthMap.idepth(newIDX_y, newIDX_X) - new_idepth;
                        if(globalParams.DIFF_FAC_PROP_MERGE * diff * diff > new_var + obj.otherDepthMap.idepth_var(newIDX_y, newIDX_X) )
                            if(new_idepth < obj.otherDepthMap.idepth(newIDX_y, newIDX_X))
                                continue;
                            else
                                obj.otherDepthMap.isValid(newIDX_y, newIDX_X) = false;
                            end
                            
                        end
                    end
                    
                    
                    if( ~obj.otherDepthMap.isValid(newIDX_y, newIDX_X))
                        
                        
                        obj.otherDepthMap.idepth(newIDX_y, newIDX_X) = new_idepth;
                        obj.otherDepthMap.idepth_var(newIDX_y, newIDX_X) = new_var;
                        
                        obj.otherDepthMap.idepth_smoothed(newIDX_y, newIDX_X) = new_idepth;
                        obj.otherDepthMap.idepth_var_smoothed(newIDX_y, newIDX_X) = new_var;
                        
                        obj.otherDepthMap.isValid(newIDX_y, newIDX_X) = true;
                        obj.otherDepthMap.blackListed (newIDX_y, newIDX_X)= 0;
                        
                        
                        obj.otherDepthMap.validity_counter(newIDX_y, newIDX_X)= obj.currentDepthMap.validity_counter(y,x);
                        
                        
                        
                    else
                        
                        % merge idepth ekf-style
                        w = new_var / (obj.otherDepthMap.idepth_var(newIDX_y, newIDX_X) + new_var);
                        merged_new_idepth = w* obj.otherDepthMap.idepth_var(newIDX_y, newIDX_X) + (1.0-w)*new_idepth;
                        
                        % merge validity
                        merged_validity = obj.currentDepthMap.validity_counter(y,x) + obj.otherDepthMap.validity_counter(newIDX_y, newIDX_X);
                        if(merged_validity > globalParams.VALIDITY_COUNTER_MAX+(globalParams.VALIDITY_COUNTER_MAX_VARIABLE))
                            merged_validity = globalParams.VALIDITY_COUNTER_MAX+(globalParams.VALIDITY_COUNTER_MAX_VARIABLE);
                        end
                        
                        curVar = obj.otherDepthMap.idepth_var(newIDX_y, newIDX_X) ;
                        mergedVar = 1.0/(1.0/curVar + 1.0/new_var);
                        obj.otherDepthMap.idepth(newIDX_y, newIDX_X) = merged_new_idepth;
                        obj.otherDepthMap.idepth_var(newIDX_y, newIDX_X) = mergedVar;
                        
                        obj.otherDepthMap.idepth_smoothed(newIDX_y, newIDX_X) = merged_new_idepth;
                        obj.otherDepthMap.idepth_var_smoothed(newIDX_y, newIDX_X) = mergedVar;
                        
                        obj.otherDepthMap.isValid(newIDX_y, newIDX_X) = true;
                        obj.otherDepthMap.blackListed (newIDX_y, newIDX_X)= 0;
                        
                        
                        obj.otherDepthMap.validity_counter(newIDX_y, newIDX_X)=merged_validity;
                        
                        
                    end
                    
                    
                end
            end
            
            %get rid of this
            obj.otherDepthMap.idepth_smoothed(isnan(obj.otherDepthMap.idepth_smoothed)) = 0;
            obj.otherDepthMap.idepth(isnan(obj.otherDepthMap.idepth)) = 0;
            
            
            obj.otherDepthMap.idepth_var_smoothed(isnan(obj.otherDepthMap.idepth_var_smoothed)) = 0;
            obj.otherDepthMap.idepth_var(isnan(obj.otherDepthMap.idepth_var)) = 0;
            
            
            temp = copy(obj.currentDepthMap);
            obj.currentDepthMap = copy(obj.otherDepthMap);
            obj.otherDepthMap = copy(temp);
            
            delete(temp);
        end
        
        function regularizeDepthMap(obj, removeOcclusions,  validityTH)
            obj.regularizeDepthMapRow(validityTH, 2, obj.height-2,removeOcclusions);
        end
        
        function regularizeDepthMapRow(obj, validityTH,  yMin,  yMax, removeOcclusions)
            regularize_radius = 2;
            regDistVar = globalParams.REG_DIST_VAR;
            for y= yMin:yMax
                for x= regularize_radius:obj.width-regularize_radius
                    
                    % dest = obj.currentDepthMap.isvalid(x,y)
                    %destRead = obj.otherDepthMap.isvalid(x,y)
                    
                    if(~obj.otherDepthMap.isValid(y,x))
                        continue
                    end
                    
                    sum=0;
                    val_sum=0;
                    sumIvar=0;
                    numOccluding = 0;
                    numNotOccluding = 0;
                    
                    
                    for  dx = -regularize_radius:regularize_radius
                        for  dy=-regularize_radius:regularize_radius
                            
                            %source =   obj.otherDepthMap.isValid(x +dx,y + dy)
                            if(~obj.otherDepthMap.isValid(y + dy, x +dx))
                                continue;
                            end
                            %stats->num_reg_total++;
                            
                            diff =obj.otherDepthMap.idepth(y + dy,x +dx) - obj.otherDepthMap.idepth(y,x);
                            if(globalParams.DIFF_FAC_SMOOTHING * diff * diff > obj.otherDepthMap.idepth_var(y + dy,x +dx) + obj.otherDepthMap.idepth_var(y,x ) )
                                if(removeOcclusions)
                                    
                                    if(obj.otherDepthMap.idepth(y + dy,x +dx)  >  obj.otherDepthMap.idepth(y,x))
                                        numOccluding = numOccluding + 1;
                                    end
                                    
                                end
                                continue;
                            end
                            
                            val_sum = val_sum + obj.otherDepthMap.validity_counter(y + dy,x +dx);
                            
                            if(removeOcclusions)
                                numNotOccluding = numNotOccluding + 1;
                            end
                            distFac = double((dx*dx+dy*dy)*regDistVar);
                            ivar = 1.0/( obj.otherDepthMap.idepth_var(y + dy,x +dx)   + distFac);
                            
                            sum = sum + (obj.otherDepthMap.idepth(y + dy,x +dx) * ivar);
                            sumIvar = sumIvar +ivar;
                            
                            
                        end
                    end
                    
                    if(val_sum < validityTH)
                        
                        obj.currentDepthMap.isValid(y,x) = false;
                        %%if(enablePrintDebugInfo) stats->num_reg_deleted_secondary++;
                        obj.currentDepthMap.blackListed(y,x) = obj.currentDepthMap.blackListed(y,x) - 1;
                        
                        %%if(enablePrintDebugInfo) stats->num_reg_setBlacklisted++;
                        continue;
                    end
                    
                    
                    if(removeOcclusions)
                        
                        if(numOccluding > numNotOccluding)
                            
                            obj.currentDepthMap.isValid(y,x) = false;
                            %%if(enablePrintDebugInfo) stats->num_reg_deleted_occluded++;
                            
                            continue;
                        end
                    end
                    
                    sum = sum / sumIvar;
                    %sum = UNZERO(sum);
                    
                    
                    %%update!
                    obj.currentDepthMap.idepth_smoothed(y,x)= sum;
                    obj.currentDepthMap.idepth_var_smoothed(y,x)= 1.0/sumIvar;
                    
                    
                    
                end
            end
            
        end
        
        function regularizeDepthMapFillHolesRow(obj,yMin, yMax)
            activeKFImageData = obj.activeKeyFrame.imgGray;
            filter = [-1 0 1];
            [w,h] = size(activeKFImageData);
            gx =  double(imfilter(activeKFImageData,filter));
            [Gmag,Gdir] = imgradient( activeKFImageData);
            newKFMaxGrad = Gmag;% gx;%new_keyframe.maxGradients;
            
            for y= yMin:yMax
                
                for  x=4: obj.width-2
                    if (obj.otherDepthMap.isValid(y,x ))
                        continue;
                    end
                    %if(keyFrameMaxGradBuf[idx]<MIN_ABS_GRAD_DECREASE) continue;
                    if 0
                        val = obj.validityIntegralBuffer(x+2,y+2) - obj.validityIntegralBuffer(x-3,y+2) - obj.validityIntegralBuffer(x+2,y-3)- obj.validityIntegralBuffer(x-3,y-3);
                    else
                        val = obj.validityIntegralBuffer(y+2,x+2) - obj.validityIntegralBuffer(y+2,x-3) - obj.validityIntegralBuffer(y-3,x+2)- obj.validityIntegralBuffer(y-3,x-3);
                    end
                    
                    if((obj.otherDepthMap.blackListed(y,x )  && val > globalParams.VAL_SUM_MIN_FOR_CREATE) || val > globalParams.VAL_SUM_MIN_FOR_UNBLACKLIST)
                        
                        sumIdepthObs = 0;
                        sumIVarObs = 0;
                        num = 0;
                        
                        for dx = -2 :2
                            for dy = -2 : 2
                                if (obj.otherDepthMap.isValid(y + dy,x +dx))
                                    continue;
                                end
                                
                                sumIdepthObs =  sumIdepthObs + (obj.otherDepthMap.idepth(y + dy,x +dx) /obj.otherDepthMap.idepth_var(y + dy,x +dx));
                                sumIVarObs = sumIVarObs + (1.0/obj.otherDepthMap.idepth_var(y + dy,x +dx));
                                num = num+1;
                            end
                        end
                        
                        
                        idepthObs = sumIdepthObs / sumIVarObs;
                        %idepthObs = UNZERO(idepthObs);
                        
                        obj.currentDepthMap.idepth(y,x) =idepthObs;
                        obj.currentDepthMap.idepth_smoothed(y,x) = idepthObs;
                        obj.currentDepthMap.idepth_var(y,x) =globalParams.VAR_RANDOM_INIT_INITIAL;
                        obj.currentDepthMap.idepth_var_smoothed(y,x) = globalParams.VAR_RANDOM_INIT_INITIAL;
                        obj.currentDepthMap.validity_counter(y,x)  = 0;
                        
                        
                        %	if(enablePrintDebugInfo) stats->num_reg_created++;
                    end
                end
            end
            
        end
        
        function regularizeDepthMapFillHoles(obj)
            obj.buildRegIntegralBuffer();
            obj.otherDepthMap = copy(obj.currentDepthMap);
            obj.regularizeDepthMapFillHolesRow(4,obj.height-2);
        end
        
        function buildRegIntegralBufferRow1(obj,yMin, yMax)
            
            for y=yMin:yMax
                validityIntegralBufferSUM = 0;
                
                for x=0:obj.width
                    if  obj.currentDepthMap.isValid(y,x)
                        validityIntegralBufferSUM = validityIntegralBufferSUM  + 1;
                    end
                    obj.validityIntegralBuffer(y,x) = validityIntegralBufferSUM;
                    
                end
                
            end
        end
        
        function buildRegIntegralBuffer(obj)
            
            a = obj.validityIntegralBuffer;
            for  y= 2: obj.height
                for x = 1 : obj.width
                    obj.validityIntegralBuffer( y, x) = obj.validityIntegralBuffer(y, x) + obj.validityIntegralBuffer(y-1, x);
                end
            end
            
            b = cumsum(a,2); 
        end
        function [result_idepth, result_var, result_eplLength, errors] = doLineStereo(obj,u,v,epxn,epyn,referenceFrame)
            
            
            %{
              referenceFrame.thisToOther_t = [0.0730602 0.110193 -0.0344612];%[-0.00686165 , -0.01699,0.00674572];
             referenceFrame.thisToOther_R = [ 0.998751 -0.0408764 -0.0287494 ;  0.04397  0.99218 0.116814  ; 0.0237496 -0.117932  0.992738]';
             referenceFrame.K_otherToThis_R  = obj.K * inv(referenceFrame.thisToOther_R);
               
            obj.priorDepth(3,3) = 0.605377;
            u(1,1) = 3;
            v(1,1) = 3;
           
            %}
            
            %{
                KinvP = ones(size(u,1) , 3);
                KinvP(:,1) = obj.fxi*u+obj.cxi;
                KinvP(:,2) = obj.fyi*v+obj.cyi;
                %%KinvP = [obj.fxi*u+obj.cxi,obj.fyi*v+obj.cyi,1.0];
            %}
            
            
            KinvP = ones(size(u,1),size(u,2) , 3);
            KinvP(:,:, 1) = referenceFrame.validIDs .* (obj.fxi*u+obj.cxi);
            KinvP(:,:, 2) = referenceFrame.validIDs .* (obj.fyi*v+obj.cyi);
            
            
            %pInf = referenceFrame.K_otherToThis_R * KinvP';
            pInf = zeros(size(KinvP));
            pReal = zeros(size(KinvP));
            rescaleFactor = zeros(size(KinvP,1),size(KinvP,2));
            %{
                %replace by effident mathod
                for i = 1 : size(u,1)
                    pInf(i,:) = referenceFrame.K_otherToThis_R * KinvP(i,:)';
                    priorDepth = obj.priorDepth(v(i),u(i));
                    if priorDepth ~= 0
                        pReal(i,:) = pInf(i,:) ./ priorDepth + referenceFrame.K_otherToThis_t';
                        rescaleFactor(i) = pReal(i,3) * priorDepth;
                    else
                        pReal(i,:) = -1;
                        rescaleFactor(i) = -1;
                    end
                  
                end
            %}
            %replace by effident mathod
            %{%}
            %%referenceFrame.validIDs(53 - 3 + 1,3 - 3 + 1,:) = 1;
            
            
            errors = zeros(size(referenceFrame.validIDs));
            temp_t = reshape(referenceFrame.K_otherToThis_t,1,1,3);
            skipInterval = size(KinvP,1)*size(KinvP,2);
            
            validId = find(referenceFrame.validIDs);
            
            KinvPMat = [KinvP(validId) KinvP(validId + skipInterval) KinvP(validId + 2*skipInterval)];
            pInfMat = (referenceFrame.K_otherToThis_R*KinvPMat')';
            pInf([validId; (validId + skipInterval);(validId + 2*skipInterval)]) = pInfMat(:);
            priorDepth1 = obj.priorDepth(1:size(u,1), 1:size(u,2));
            priorDepthCat = cat(3, priorDepth1, priorDepth1, priorDepth1);
            pReal = pInf ./ priorDepthCat + repmat(temp_t,size(priorDepthCat,1),size(priorDepthCat,2),1);
            rescaleFactor = pReal(:,:,3) .* priorDepth1;
            
            id2 = find(priorDepth1 == 0);
            pReal([id2; id2+skipInterval; id2+2*skipInterval]) = 0;
            rescaleFactor(id2) = 0;
            referenceFrame.validIDs(id2) = 0;
            
            sauhi = 1;
            if 0
                for i = 1 : size(u,1)
                    for j  = 1 : size(u,2)
                        if referenceFrame.validIDs (i,j) == 1
                            pInf(i,j,:) =reshape(referenceFrame.K_otherToThis_R * reshape(KinvP(i,j,:),3,1),1,1,3);
                            priorDepth = obj.priorDepth(i,j);
                            if priorDepth ~= 0
                                pReal(i,j,:) = pInf(i,j,:) ./ priorDepth + temp_t;
                                rescaleFactor(i,j) = pReal(i,j,3) * priorDepth;
                            else
                                pReal(i,j,:) = 0;
                                rescaleFactor(i,j) = 0;
                                referenceFrame.validIDs(i,j)  = 0;
                            end
                        end
                    end
                end
            end
            
            %{
                validIDs = int32(find(rescaleFactor ~=-1));
                rescaleFactor = rescaleFactor(validIDs);
                u = u(validIDs);
                v = v(validIDs);
                epxn = epxn(validIDs);
                epyn = epyn(validIDs);
                pInf = pInf(validIDs,:);
                pReal = pReal(validIDs,:);
                KinvP = KinvP(validIDs,:);
            %}
            %{
                firstX = u - 2*epxn*rescaleFactor;
                firstY = v - 2*epyn*rescaleFactor;
                lastX = u + 2*epxn*rescaleFactor;
                lastY = v + 2*epyn*rescaleFactor;
            %}
            %firstX , firstY , lastX , lastY
            
            
            eplLines = zeros(size(u,1) , size(u,2) , 4);
            %eplLines = [ referenceFrame.validIDs .* ( u - 2*epxn.*rescaleFactor),  referenceFrame.validIDs .* (v - 2*epyn.*rescaleFactor) ,  referenceFrame.validIDs .* (u + 2*epxn.*rescaleFactor), referenceFrame.validIDs .* (v + 2*epyn.*rescaleFactor)];
            eplLines(:,:, 1) =  referenceFrame.validIDs .* ( u - 2*epxn.*rescaleFactor);
            eplLines(:,:, 2) =referenceFrame.validIDs .* (v - 2*epyn.*rescaleFactor) ;
            eplLines(:,:, 3) =referenceFrame.validIDs .* (u + 2*epxn.*rescaleFactor);
            eplLines(:,:, 4) = referenceFrame.validIDs .* (v + 2*epyn.*rescaleFactor);
            
            
            tempIDs = (~(eplLines(:, :,1) <= 0 | eplLines(:, :,1) >= obj.width - 2 | eplLines(:, :,2) <= 0 | eplLines(:, :,2) >= obj.height - 2 | eplLines(:, :,3) <= 0 | eplLines(:, :,3) >= obj.width - 2 | eplLines(:, :,4) <= 0 | eplLines(:, :,4) >= obj.height - 2));
            
            referenceFrame.validIDs = referenceFrame.validIDs & tempIDs;
            errors(~tempIDs) = -1;
            %not working
            %%referenceFrame.validIDs = referenceFrame.validIDs & (~(rescaleFactor > 0.7 & rescaleFactor < 1.4) & rescaleFactor ~= 0) ;
            
            %{
                %%width - 2 and height - 2 comes from the one-sided gradient calculation at the bottom
                validIDs = int32( find(~(eplLines(:,1) <= 0 | eplLines(:,1) >= obj.width - 2 | eplLines(:,2) <= 0 | eplLines(:,2) >= obj.height - 2 | eplLines(:,3) <= 0 | eplLines(:,3) >= obj.width - 2 | eplLines(:,4) <= 0 | eplLines(:,4) >= obj.height - 2)));
                eplLines = eplLines(validIDs,:);
                rescaleFactor = rescaleFactor(validIDs);
                epxn = epxn(validIDs);
                epyn = epyn(validIDs);
                 pInf = pInf(validIDs,:);
                pReal = pReal(validIDs,:);
                 u = u(validIDs);
                v = v(validIDs);
                KinvP = KinvP(validIDs,:);
            %}
            
            
            %{
                %%not wroking
                validIDs = int32(find(~(rescaleFactor > 0.7 & rescaleFactor < 1.4)));
                eplLines = eplLines(validIDs,:);
                rescaleFactor = rescaleFactor(validIDs);
                epxn = epxn(validIDs);
                epyn = epyn(validIDs);
                u = u(validIDs);
                v = v(validIDs);
                 pInf = pInf(validIDs,:);
                pReal = pReal(validIDs,:);
            %}
            
            %{
                %%draw line
                imshow(referenceFrame.imgRGB)
                l = 20;
                line([eplLines(l,1),eplLines(l,2)],[eplLines(l,3),eplLines(l,4)])
            %}
            
            %%interploation
            
            
            
            
            %{
                 [x , y] = meshgrid(1:obj.height,1 :obj.width );
                activeKeyFrameForIntepolation = im2double(obj.activeKeyFrame.imgGray);
                
                 epxnRf = referenceFrame.validIDs .* (epxn.*rescaleFactor);
                epynRf = referenceFrame.validIDs .* (epyn.*rescaleFactor);
                
                realVal_p1 = interp2(x,y,imgForIntepolation,u + epxnRf, v + epynRf, 'cubic');
                realVal_m1 = interp2(x,y,imgForIntepolation,u - epxnRf, v - epynRf, 'cubic');
                realVal = interp2(x,y,imgForIntepolation,u,v,'cubic') ;
                realVal_m2 = interp2(x,y,imgForIntepolation,u - epxnRf, v - epynRf, 'cubic');
                realVal_p2 = interp2(x,y,imgForIntepolation,u + epxnRf, v + epynRf, 'cubic');
                
            %}
            
            
            
            sv = sqrt(obj.currentDepthMap.idepth_var_smoothed);
            min_idepth = obj.currentDepthMap.idepth_smoothed - sv*globalParams.STEREO_EPL_VAR_FAC;
            max_idepth = obj.currentDepthMap.idepth_smoothed + sv*globalParams.STEREO_EPL_VAR_FAC;
            min_idepth(min_idepth < 0) = 0;
            oneByMin_Depth = 1/globalParams.MIN_DEPTH;
            max_idepth(max_idepth > oneByMin_Depth) = oneByMin_Depth;
            
            %%for testing
            %{
                 obj.priorDepth(66 - 3 + 1,13 - 3 + 1,:) = 1.433;
                obj.priorDepth(53 - 3 + 1,3 - 3 + 1,:) = 0.72 ;
                 max_idepth(66 - 3 + 1,13 - 3 + 1,:) = 2.14;
                min_idepth(66 - 3 + 1,13 - 3 + 1,:) = 0.726;
                
                max_idepth(53 - 3 + 1,3 - 3 + 1,:) = 1.42;
                min_idepth(53 - 3 + 1,3 - 3 + 1,:) = 0.0013;
            %}
            %%%
            
            pClose = zeros(size(pInf,1),size(pInf,2),3);
            pFar = zeros(size(pInf,1),size(pInf,2),3);
            
            id3 = find(referenceFrame.validIDs);
            temp_t_mat = repmat(temp_t, size(pClose,1), size(pClose,2), 1);
            max_idepth_ = max_idepth(1:size(pClose,1), 1:size(pClose,2));
            min_idepth_ = min_idepth(1:size(pClose,1), 1:size(pClose,2));
            pInf_mat0 = [pInf(id3) pInf(id3 + skipInterval) pInf(id3 + 2*skipInterval)] +...
                [temp_t_mat(id3) temp_t_mat(id3 + skipInterval) temp_t_mat(id3 + 2*skipInterval)].*...
                [max_idepth_(id3),max_idepth_(id3),max_idepth_(id3)];
            pClose([id3;id3+skipInterval;id3+2*skipInterval]) = pInf_mat0(:);
            id33 = find(pClose(:,:,3) < 0.001);
            [yId, xId] = ind2sub(size(pClose(:,:,1)), id33);
            id33Full = sub2ind(size(max_idepth), yId, xId);
            max_idepth(id33Full) =  (0.001-pInf(id33 + 2*skipInterval)) ./ referenceFrame.K_otherToThis_t(3);
            pInf_mat1 = [pInf(id33) pInf(id33 + skipInterval) pInf(id33 + 2*skipInterval)] +...
                [temp_t_mat(id33) temp_t_mat(id33 + skipInterval) temp_t_mat(id33 + 2*skipInterval)].*...
                [max_idepth_(id33) max_idepth_(id33) max_idepth_(id33)];
            pClose([id33;id33+skipInterval;id33+2*skipInterval]) = pInf_mat1(:);
            
            pClose([id3;id3+skipInterval;id3+2*skipInterval]) = pClose([id3;id3+skipInterval;id3+2*skipInterval])./pClose([id3+2*skipInterval;id3+2*skipInterval;id3+2*skipInterval]);
            
            pInf_mat2 = [pInf(id3) pInf(id3 + skipInterval) pInf(id3 + 2*skipInterval)] +...
                [temp_t_mat(id3) temp_t_mat(id3 + skipInterval) temp_t_mat(id3 + 2*skipInterval)].*...
                [min_idepth_(id3),min_idepth_(id3),min_idepth_(id3)];
            pFar([id3;id3+skipInterval;id3+2*skipInterval]) = pInf_mat2(:);
            
            id44 = find(pFar(:,:,3) < 0.001);
            id55 = find(pFar(:,:,3) >= 0.001);
            referenceFrame.validIDs(id44) = 0;
            errors(id44) = -1;
            pFar([id44;id44+skipInterval;id44+2*skipInterval]) = 0;
            
            pFar([id55;id55+skipInterval;id55+2*skipInterval])  = pFar([id55;id55+skipInterval;id55+2*skipInterval])./pFar([id55+2*skipInterval;id55+2*skipInterval;id55+2*skipInterval]);
            
            asdgkj = 1;
            if 0
                for i = 1 : size(u,1)
                    for j  = 1 : size(u,2)
                        if  referenceFrame.validIDs(i, j) ~= 0
                            pClose(i,j,:) = pInf(i,j,:) + temp_t*max_idepth(i,j);
                            if(pClose(i,j,3) < 0.001)
                                max_idepth(i,j) = (0.001-pInf(i,j,3)) / referenceFrame.K_otherToThis_t(3);
                                pClose(i,j,:)  = pInf(i,j,:) + temp_t*max_idepth(i,j);
                            end
                            
                            pClose(i,j,:)  = pClose(i,j,:)  ./ pClose(i,j,3);
                            
                            pFar(i,j,:)  = pInf(i,j,:) + temp_t *min_idepth(i,j);
                            
                            if (pFar(i,j,3) < 0.001 )
                                referenceFrame.validIDs(i, j) = 0;
                                errors(i,j) = -1;
                                pFar(i,j,:) = 0;
                            else
                                pFar(i,j,:)  = pFar(i,j,:)  ./ pFar(i,j,3);
                            end
                        end
                    end
                end
            end
            %{
                
                for i = 1: size(pInf,1)
                    pClose(i,:) = pInf(i,:) + referenceFrame.K_otherToThis_t'*max_idepth(u(i),v(i));
                    
                    if(pClose(i,3) < 0.001)
                        max_idepth(u,v) = (0.001-pInf(i,3)) / referenceFrame.K_otherToThis_t(3);
                        pClose(i,:)  = pInf(i,:) + referenceFrame.K_otherToThis_t'*max_idepth(u(i),v(i));
                    end
                    pClose(i,:)  = pClose(i,:)  ./ pClose(i,3);
                    
                    pFar(i,:)  = pInf(i,:) + referenceFrame.K_otherToThis_t'*min_idepth(u(i),v(i));
                end
            %}
            
            
            %{
                 referenceFrame.validIDs = referenceFrame.validIDs & (~(pFar(:,:,3) < 0.001 ));
                 
                 pF1 = pFar(:,:,1);
                 pF2 = pFar(:,:,2);
                 pF3 = pFar(:,:,3);
                 pF1( referenceFrame.validIDs ) =  pF1( referenceFrame.validIDs )./ pF3 (referenceFrame.validIDs );
                 pF2( referenceFrame.validIDs ) =  pF2( referenceFrame.validIDs )./ pF3 (referenceFrame.validIDs );
                 pF3( referenceFrame.validIDs ) =  pF3( referenceFrame.validIDs )./ pF3 (referenceFrame.validIDs );
                 
                 pFar(:,:,1) = pF1;
                 pFar(:,:,2) = pF2;
                 pFar(:,:,3) = pF3;
            %}
            
            tempIDs= ~(isnan(pFar(:,:,1)+pClose(:,:,1)));
            errors(~tempIDs) = -4;
            referenceFrame.validIDs = referenceFrame.validIDs & tempIDs;
            %{
                validIDs = ~(pFar(:,3) < 0.001 );%%%%% | max_idepth < min_idepth);
                pFar = pFar(validIDs,:);
                pClose = pClose(validIDs,:);
                
                pFar = pFar ./pFar(3);
                
                validIDs = ~(isnan(pFar(:,1)+pClose(:,1)));
                pFar = pFar(validIDs,:);
                pClose = pClose(validIDs,:);
                
                realVal_p1 = realVal_p1(validIDs);
                realVal_m1 = realVal_m1(validIDs);
                realVal = realVal(validIDs) ;
                realVal_m2 = realVal_m2(validIDs);
                realVal_p2 = realVal_p2 (validIDs);
                KinvP = KinvP(validIDs,:);
                u = u(validIDs);
                v = v(validIDs);
            %}
            %%calculate increments in which we will step through the epipolar line. they are sampleDist (or half sample dist) long
            incx = pClose(:,:,1) - pFar(:,:,1);
            incy = pClose(:,:,2) - pFar(:,:,2);
            %referenceFrame.validIDs = referenceFrame.validIDs & (~(eplLength < 0  | eplLength >= Inf));
            eplLength = referenceFrame.validIDs .*  sqrt(incx.*incx+incy.*incy);
            %{
                pFar = pFar(validIDs,:);
                pClose = pClose(validIDs,:);
                u = u(validIDs);
                v = v(validIDs);
            %}
            
            tempIDs = ~(eplLength < 0 | isinf(eplLength));
            errors(~tempIDs) = -4;
            referenceFrame.validIDs = referenceFrame.validIDs   & tempIDs;
            
            longerEplIds =  eplLength > globalParams.MAX_EPL_LENGTH_CROP;
            
            if(any(any(longerEplIds)))
                pad = zeros(size(incx,1) ,size(incx,2));
                pad(longerEplIds) = (globalParams.MAX_EPL_LENGTH_CROP./eplLength(longerEplIds));
                
                %{
                pClose(:,:,1) = pFar(:,:,1) + incx.*pad;
                pClose(:,:,2) = pFar(:,:,2) + incy.*pad;
                %}
                pad(longerEplIds) = incx(longerEplIds).*pad(longerEplIds);
                pClose(:,:,1) = pFar(:,:,1) + pad;
                pClose(:,:,2) = pFar(:,:,2) + pad;
            end
            incx = incx .* (globalParams.GRADIENT_SAMPLE_DIST./eplLength);
            incy = incy .* (globalParams.GRADIENT_SAMPLE_DIST./eplLength);
            
            referenceFrame.validIDs = (referenceFrame.validIDs & eplLength ~= 0);
            incx(~referenceFrame.validIDs) = 0;
            incy(~referenceFrame.validIDs) = 0;
            
            
            
            pFar(:,:,1) = pFar(:,:,1) - incx;
            pFar(:,:,2) =  pFar(:,:,2) - incy;
            pClose(:,:,1)  =  pClose(:,:,1) + incx;
            pClose(:,:,2) = pClose(:,:,2) + incy;
            
            %% make epl long enough (pad a little bit).
            shoterEplIds = eplLength < globalParams.MIN_EPL_LENGTH_CROP;
            
            if(any(any(shoterEplIds)))
                %t = ones(size(incx,1) ,size(incx,2)) * globalParams.MIN_EPL_LENGTH_CROP;
                pad = zeros(size(incx,1) ,size(incx,2));
                pad(shoterEplIds) = (globalParams.MIN_EPL_LENGTH_CROP - (eplLength(shoterEplIds))) / 2.0;
                pFar(:,:,1) = pFar(:,:,1) - incx.*pad;
                pFar(:,:,2) = pFar(:,:,2) - incy.*pad;
                
                pClose(:,:,1) =  pClose(:,:,1) + incx.*pad;
                pClose(:,:,2) = pClose(:,:,2) +incy.*pad;
                
            end
            %%If inf point is outside of image: skip pixel.
            eplInsideIDs = ~(pFar(:,:,1) <= globalParams.SAMPLE_POINT_TO_BORDER | pFar(:,:,1) >= obj.width-globalParams.SAMPLE_POINT_TO_BORDER | pFar(:,:,2) <= globalParams.SAMPLE_POINT_TO_BORDER | pFar(:,:,2) >= obj.height-globalParams.SAMPLE_POINT_TO_BORDER);
            
            
            
            tempIDs =  ~(pFar(:,:,1) <= globalParams.SAMPLE_POINT_TO_BORDER | pFar(:,:,1) >= obj.width-globalParams.SAMPLE_POINT_TO_BORDER | pFar(:,:,2) <= globalParams.SAMPLE_POINT_TO_BORDER | pFar(:,:,2) >= obj.height-globalParams.SAMPLE_POINT_TO_BORDER);
            errors(~tempIDs) = -1;
            
            referenceFrame.validIDs = referenceFrame.validIDs &tempIDs;
            
            pFar(:,:,1) = eplInsideIDs .* pFar(:,:,1);
            pFar(:,:,2) = eplInsideIDs .* pFar(:,:,2);
            pFar(:,:,3) = eplInsideIDs .* pFar(:,:,3);
            
            pClose(:,:,1) = eplInsideIDs .* pClose(:,:,1);
            pClose(:,:,2) = eplInsideIDs .* pClose(:,:,2);
            pClose(:,:,3) = eplInsideIDs .* pClose(:,:,3);
            
            
            incx = eplInsideIDs .* incx;
            incy = eplInsideIDs .* incy;
            u = eplInsideIDs .* u;
            v = eplInsideIDs .* v;
            
            
            
            KinvP(:,:,1) = eplInsideIDs .* KinvP(:,:,1);
            KinvP(:,:,2) = eplInsideIDs .* KinvP(:,:,2);
            KinvP(:,:,3) = eplInsideIDs .* KinvP(:,:,3);
            %{
                realVal_p1 = realVal_p1(eplInsideIDs);
                realVal_m1 = realVal_m1(eplInsideIDs);
                realVal = realVal(eplInsideIDs) ;
                realVal_m2 = realVal_m2(eplInsideIDs);
                realVal_p2 = realVal_p2 (eplInsideIDs);
                KinvP = KinvP(eplInsideIDs,:);
                
            %}
            %{
            %}
            %% if near point is outside: move inside, and test length again.
            
            eplInsideIDALLs = ~(pClose(:,:,1) <= globalParams.SAMPLE_POINT_TO_BORDER | pClose(:,:,1) >= obj.width-globalParams.SAMPLE_POINT_TO_BORDER | pClose(:,:,2) <= globalParams.SAMPLE_POINT_TO_BORDER | pClose(:,:,2) >= obj.height-globalParams.SAMPLE_POINT_TO_BORDER);
            
            toAdd = (globalParams.SAMPLE_POINT_TO_BORDER - pClose(:,:,1)) ./ incx;
            eplInsideIDs = referenceFrame.validIDs & (pClose(:,:,1) <= globalParams.SAMPLE_POINT_TO_BORDER);
            toAdd(~eplInsideIDs) = 0;
            
            %toAdd(eplInsideIDs) = (globalParams.SAMPLE_POINT_TO_BORDER - pClose(eplInsideIDs,1)) ./ incx(eplInsideIDs);
            pClose(:,:,1) =  pClose(:,:,1) + (toAdd .* incx);
            pClose(:,:,2) =  pClose(:,:,2) + (toAdd .* incy);
            
            eplInsideIDs = referenceFrame.validIDs & (pClose(:,:,1) >= obj.width-globalParams.SAMPLE_POINT_TO_BORDER);
            toAdd = (obj.width-globalParams.SAMPLE_POINT_TO_BORDER - pClose(:,:,1)) ./ incx;
            toAdd(~eplInsideIDs) = 0;
            
            pClose(:,:,1)  = pClose(:,:,1)  + (toAdd .* incx);
            pClose(:,:,2)  = pClose(:,:,2)  + (toAdd .* incy);
            
            
            eplInsideIDs =  referenceFrame.validIDs & (pClose(:,:,2) <= globalParams.SAMPLE_POINT_TO_BORDER) ;
            toAdd = (globalParams.SAMPLE_POINT_TO_BORDER - pClose(:,:,1)) ./ incy;
            toAdd(~eplInsideIDs) = 0;
            
            pClose(:,:,1)  = pClose(:,:,1)  + (toAdd .* incx);
            pClose(:,:,2)  = pClose(:,:,2)  + (toAdd .* incy);
            
            eplInsideIDs =  referenceFrame.validIDs & (pClose(:,:,2)  >= obj.height-globalParams.SAMPLE_POINT_TO_BORDER);
            toAdd = (obj.height-globalParams.SAMPLE_POINT_TO_BORDER - pClose(:,:,1)) ./ incy;
            toAdd(~eplInsideIDs) = 0;
            
            pClose(:,:,1)  = pClose(:,:,1) + (toAdd .* incx);
            pClose(:,:,2)  = pClose(:,:,2) + (toAdd .* incy);
            
            
            %get new epl length
            fincx = pClose(:,:,1) - pFar(:,:,1);
            fincy = pClose(:,:,2) - pFar(:,:,2);
            newEplLength = sqrt(fincx.*fincx+fincy.*fincy);
            
            pC1 = pClose(:,:,1);
            pC2 = pClose(:,:,2);
            pF1 = pFar(:,:,1);
            pF2 = pFar(:,:,2);
            te = ones(size(pFar,1),size(pFar,2));
            te((pC1(eplInsideIDALLs) <= globalParams.SAMPLE_POINT_TO_BORDER | pC1(eplInsideIDALLs) >= obj.width-globalParams.SAMPLE_POINT_TO_BORDER |pC2(eplInsideIDALLs) <= globalParams.SAMPLE_POINT_TO_BORDER | pC2(eplInsideIDALLs) >= obj.height-globalParams.SAMPLE_POINT_TO_BORDER | ~(newEplLength(eplInsideIDALLs) < 8.0) )) = 0;
            referenceFrame.validIDs = referenceFrame.validIDs & te;
            errors(te==0) = -1;
            %referenceFrame.validIDs = referenceFrame.validIDs &  ~(newEplLength < 8.0);
            %{
              eplInsideIDs = ~(pClose(:,1) <= globalParams.SAMPLE_POINT_TO_BORDER | pClose(:,1) >= obj.width-globalParams.SAMPLE_POINT_TO_BORDER | pClose(:,2) <= globalParams.SAMPLE_POINT_TO_BORDER | pClose(:,2) >= obj.height-globalParams.SAMPLE_POINT_TO_BORDER | newEplLength < 8.0);
              tempIDs = eplInsideIDs;
              
            pFar = pFar(eplInsideIDs , :);
              pClose = pClose(eplInsideIDs , :);
                incx = incx(eplInsideIDs);
                incy = incy(eplInsideIDs);
               
                realVal_p1 = realVal_p1(eplInsideIDs);
                realVal_m1 = realVal_m1(eplInsideIDs);
                realVal = realVal(eplInsideIDs) ;
                realVal_m2 = realVal_m2(eplInsideIDs);
                realVal_p2 = realVal_p2 (eplInsideIDs);
                KinvP = KinvP(eplInsideIDs,:);
                
                  u = u(eplInsideIDs);
                v = v(eplInsideIDs);
              
            %}
            %{
              
              
                pc = [16.3114052,86.9859619,1]
                pf = [15.1771603 ,74.3268051 , 1]
            imshow(referenceFrame.imgRGB)
                    hold on;
                line([pc(1,2),pf(1,2)],[pc(1,1),pf(1,1)])
                close all;
                figure();
                imshow(referenceFrame.imgRGB)
                hold on;
                [x, y] = find(referenceFrame.validIDs);
                l = 1;
                for l = 1: size(x , 1)
                line([pClose(x(l) , y(l),2),pFar(x(l) , y(l),2)],[pClose(x(l) , y(l),1),pFar(x(l) , y(l),1)])
                %line([pClose(x(l) , y(l),1),pClose(x(l) , y(l),2)],[pFar(x(l) , y(l),1),pFar(x(l) , y(l),2)])
                end
            %}
            %{
                if (firstX <= 0 || firstX >= obj.width - 2 || firstY <= 0 || firstY >= obj.height - 2 || lastX <= 0 || lastX >= obj.width - 2 || lastY <= 0 || lastY >= obj.height - 2)
                    flag = -1;
                    return
                end
            %}
            
            
            % from here on:
            % - pInf: search start-point
            % - p0: search end-point
            % - incx, incy: search steps in pixel
            % - eplLength, min_idepth, max_idepth: determines search-resolution, i.e. the result's variance.
            
            
            referenceFrameForIntepolation = double(referenceFrame.imgGray);
            
            
            pF1 = pFar(:,:,1);
            pF2 = pFar(:,:,2);
            pF1 = pF1( referenceFrame.validIDs);
            pF2 = pF2( referenceFrame.validIDs);
            
            
            epxnRf = referenceFrame.validIDs .* (epxn.*rescaleFactor);
            epynRf = referenceFrame.validIDs .* (epyn.*rescaleFactor);
            
            epxnRf = epxnRf(referenceFrame.validIDs);
            epynRf = epynRf(referenceFrame.validIDs);
            
            uu = u(referenceFrame.validIDs);
            vv = v(referenceFrame.validIDs);
            
            ix = incx( referenceFrame.validIDs);
            iy = incy( referenceFrame.validIDs);
            if 0
                [x , y] = meshgrid(1:obj.height,1 :obj.width );
            else
                [x , y] = meshgrid(1:obj.width,1 : obj.height);
            end
            val_cp_m2 =  interp2(x,y,referenceFrameForIntepolation,pF2-2.0*iy, pF1-2.0*ix,'cubic');
            val_cp_m1 = interp2(x,y,referenceFrameForIntepolation,pF2-iy,pF1-ix, 'cubic');
            val_cp = interp2(x,y,referenceFrameForIntepolation,pF2, pF1,'cubic');
            val_cp_p1 = interp2(x,y,referenceFrameForIntepolation,pF2+iy, pF1+ix,'cubic');
            val_cp_p2 = zeros(size(val_cp_p1,1),1);
            
            activeKeyFrameForIntepolation = double(obj.activeKeyFrame.imgGray);
            
            
            realVal_p1 = interp2(x,y,activeKeyFrameForIntepolation,vv + epynRf, uu + epxnRf, 'cubic');
            realVal_m1 = interp2(x,y,activeKeyFrameForIntepolation,vv - epynRf, uu - epxnRf, 'cubic');
            realVal = interp2(x,y,activeKeyFrameForIntepolation,vv,uu,'cubic') ;
            realVal_m2 = interp2(x,y,activeKeyFrameForIntepolation,vv - epynRf, uu - epxnRf, 'cubic');
            realVal_p2 = interp2(x,y,activeKeyFrameForIntepolation,vv + epynRf, uu + epxnRf, 'cubic');
            
            
            
            val_cp_m2_2  = zeros(size(pClose,1), size(pClose,2));
            val_cp_m1_2 = zeros(size(pClose,1), size(pClose,2));
            val_cp_2 = zeros(size(pClose,1), size(pClose,2));
            val_cp_p1_2 = zeros(size(pClose,1), size(pClose,2));
            val_cp_p2_2 = zeros(size(pClose,1), size(pClose,2));
            
            realVal_p1_2  = zeros(size(pClose,1), size(pClose,2));
            realVal_m1_2 = zeros(size(pClose,1), size(pClose,2));
            realVal_2 = zeros(size(pClose,1), size(pClose,2));
            realVal_m2_2 = zeros(size(pClose,1), size(pClose,2));
            realVal_p2_2 = zeros(size(pClose,1), size(pClose,2));
            
            
            uuvv1 = [uu vv];
            uuvvOfst1 = [uu-3+1 vv-3+1];
            if 0
                indUV1 = sub2ind(size(val_cp_m2_2), uuvvOfst1(:,1), uuvvOfst1(:,2));
            else
                indUV1 = sub2ind(size(val_cp_m2_2), uuvvOfst1(:,2), uuvvOfst1(:,1));
            end
            val_cp_m2_2(indUV1) = val_cp_m2;
            val_cp_m1_2(indUV1) = val_cp_m1;
            val_cp_2(indUV1) = val_cp;
            val_cp_p1_2(indUV1) = val_cp_p1;
            val_cp_p2_2(indUV1) = val_cp_p2;
            
            realVal_p1_2(indUV1) = realVal_p1;
            realVal_m1_2(indUV1) = realVal_m1;
            realVal_2(indUV1) = realVal;
            realVal_m2_2(indUV1) = realVal_m2;
            realVal_p2_2(indUV1) = realVal_p2;
            
            
            asgdkhj = 1;
            if 0
                for i = 1 : size(val_cp,1)
                    val_cp_m2_2(uu(i) - 3 + 1 , vv(i) - 3 + 1) = val_cp_m2(i,1);
                    val_cp_m1_2(uu(i) - 3 + 1 , vv(i) - 3 + 1) = val_cp_m1(i,1);
                    val_cp_2(uu(i) - 3 + 1 , vv(i) - 3 + 1) = val_cp(i,1);
                    val_cp_p1_2(uu(i) - 3 + 1 , vv(i) - 3 + 1) = val_cp_p1(i,1);
                    val_cp_p2_2(uu(i) - 3 + 1 , vv(i) - 3 + 1) = val_cp_p2(i,1);
                    
                    
                    realVal_p1_2(uu(i) - 3 + 1 , vv(i) - 3 + 1) = realVal_p1(i,1);
                    realVal_m1_2(uu(i) - 3 + 1 , vv(i) - 3 + 1) = realVal_m1(i,1);
                    realVal_2(uu(i) - 3 + 1 , vv(i) - 3 + 1) = realVal(i,1);
                    realVal_m2_2(uu(i) - 3 + 1 , vv(i) - 3 + 1) = realVal_m2(i,1);
                    realVal_p2_2(uu(i) - 3 + 1 , vv(i) - 3 + 1) = realVal_p2(i,1);
                end
            end
            %{
            val_cp_m2 = val_cp_m2_2;
            val_cp_m1 = val_cp_m1_2;
            val_cp = val_cp_2;
            val_cp_p1 = val_cp_p1_2;
            val_cp_p2= val_cp_p2_2;
            
            
            realVal_p1 = realVal_p1_2;
            realVal_m1= realVal_m1_2;
            realVal = realVal_2;
             realVal_m2 = realVal_m2_2;
             realVal_p2 = realVal_p2_2;
            %}
            
            e1A = zeros(size(val_cp_p1,1),size(val_cp_p1,2));
            e2A = zeros(size(val_cp_p1,1),size(val_cp_p1,2));
            e3A = zeros(size(val_cp_p1,1),size(val_cp_p1,2));
            e4A = zeros(size(val_cp_p1,1),size(val_cp_p1,2));
            e5A = zeros(size(val_cp_p1,1),size(val_cp_p1,2));
            
            e1B = zeros(size(val_cp_p1,1),size(val_cp_p1,2));
            e2B = zeros(size(val_cp_p1,1),size(val_cp_p1,2));
            e3B = zeros(size(val_cp_p1,1),size(val_cp_p1,2));
            e4B = zeros(size(val_cp_p1,1),size(val_cp_p1,2));
            e5B = zeros(size(val_cp_p1,1),size(val_cp_p1,2));
            
            
            
            loopCounter =  zeros(size(val_cp_p1,1),size(val_cp_p1,2));
            eeLast =  ones(size(val_cp_p1,1),size(val_cp_p1,2)) .* -1;
            loopCBest= ones(size(val_cp_p1,1),size(val_cp_p1,2)) .* -1;
            loopCSecond = ones(size(val_cp_p1,1),size(val_cp_p1,2)) .* -1;
            best_match_x = zeros(size(val_cp_p1,1),size(val_cp_p1,2));
            best_match_y = zeros(size(val_cp_p1,1),size(val_cp_p1,2));
            best_match_err = ones(size(val_cp_p1,1),size(val_cp_p1,2)) * 1e50;
            second_best_match_err = ones(size(val_cp_p1,1),size(val_cp_p1,2)) * 1e50;
            bestWasLastLoop = ones(size(val_cp_p1,1),size(val_cp_p1,2)) * false;
            best_match_errPost = zeros(size(val_cp_p1,1),size(val_cp_p1,2));
            best_match_DiffErrPost = zeros(size(val_cp_p1,1),size(val_cp_p1,2));
            secondBestMatchIds = false(size(val_cp_p1,1),size(val_cp_p1,2));
            
            
            best_match_errPre = zeros(size(val_cp_p1,1),size(val_cp_p1,2));
            best_match_DiffErrPre = zeros(size(val_cp_p1,1),size(val_cp_p1,2));
            
            
            cpx = pFar(:,:,1);
            cpy = pFar(:,:,2);
            cpx = cpx(referenceFrame.validIDs);
            cpy = cpy(referenceFrame.validIDs);
            bestMacthErrIds = false(size(cpy,1),size(cpy,2));
            
            
            
            pC1= pClose(:,:,1);
            pC2 = pClose(:,:,2);
            pC1 = pC1(referenceFrame.validIDs);
            pC2 = pC2(referenceFrame.validIDs);
            %{
                    realVal_p1 = realVal_p1(referenceFrame.validIDs);
                    realVal_m1 = realVal_m1(referenceFrame.validIDs);
                    realVal = realVal(referenceFrame.validIDs);
                    realVal_m2 = realVal_m2(referenceFrame.validIDs);
                    realVal_p2 = realVal_p2(referenceFrame.validIDs);
            %}
            
            
            
            
            
            
            
            
            %{%
            
            
            %}
            % iteratroID = (incx < 0) == (cpx > pClose(:,:,1)) & (incy < 0) == (cpy > pClose(:,:,2));
            iteratroID = (ix < 0) == (cpx > pC1) & (iy < 0) == (cpy > pC2);
            %{
              
                 linID = iteratroID;%find(uu == 66 & vv == 13 );
                val_cp_m2(linID) = 172.33;
             val_cp_m1( linID)  = 173.50;
             val_cp(linID)  = 174.89;
             val_cp_p1( linID)  = 175.914;
                   
                realVal_p1(linID)  = 177.858;
                realVal_m1( linID)  = 174.073;
                realVal( linID)  = 176;
                realVal_m2( linID)  = 173.838;
                realVal_p2( linID) = 178.885;
                    
            %}
            loopCounterAll = 0;
            while(any(iteratroID) | (loopCounterAll == 0))
                
                if 0
                    val_cp_p2(iteratroID) = interp2(x,y,referenceFrameForIntepolation,cpy(iteratroID)+ 2 *iy(iteratroID),cpx(iteratroID)+2 * ix(iteratroID), 'cubic');
                else
                    val_cp_p2(iteratroID) = interp2(x,y,referenceFrameForIntepolation,cpx(iteratroID)+ 2 *ix(iteratroID),cpy(iteratroID)+2 * iy(iteratroID), 'cubic');
                end
                %val_cp_p2(iteratroID) = 176.055;
                %%val_cp_p2(iteratroID) = 176.98;
                %val_cp_p2(iteratroID) =  177.27;
                ee = zeros(size(val_cp_p1,1),size(val_cp_p1,2));
                if mod(loopCounterAll , 2) ==0
                    e1A(iteratroID)  = val_cp_p2(iteratroID)  - realVal_p2(iteratroID) ;ee(iteratroID)  = ee(iteratroID)  + e1A(iteratroID) .*e1A(iteratroID) ;
                    e2A(iteratroID)  = val_cp_p1(iteratroID)  - realVal_p1(iteratroID) ;ee(iteratroID)  = ee(iteratroID)  + e2A(iteratroID) .*e2A(iteratroID) ;
                    e3A(iteratroID)  = val_cp(iteratroID)  - realVal(iteratroID) ;      ee(iteratroID)  = ee(iteratroID)  + e3A(iteratroID) .*e3A(iteratroID) ;
                    e4A(iteratroID)  = val_cp_m1(iteratroID)  - realVal_m1(iteratroID) ;ee(iteratroID)  = ee(iteratroID)  + e4A(iteratroID) .*e4A(iteratroID) ;
                    e5A(iteratroID)  = val_cp_m2(iteratroID)  - realVal_m2(iteratroID) ;ee(iteratroID)  = ee(iteratroID)  + e5A(iteratroID) .*e5A(iteratroID) ;
                    
                else
                    e1B(iteratroID)  = val_cp_p2(iteratroID)  - realVal_p2(iteratroID) ;ee(iteratroID)  = ee(iteratroID)  + e1B(iteratroID) .*e1B(iteratroID) ;
                    e2B(iteratroID)  = val_cp_p1(iteratroID)  - realVal_p1(iteratroID) ;ee(iteratroID)  = ee(iteratroID)  + e2B(iteratroID) .*e2B(iteratroID) ;
                    e3B(iteratroID)  = val_cp(iteratroID)  - realVal(iteratroID) ;      ee(iteratroID)  = ee(iteratroID)  + e3B(iteratroID) .*e3B(iteratroID) ;
                    e4B(iteratroID)  = val_cp_m1(iteratroID)  - realVal_m1(iteratroID) ;ee(iteratroID)  = ee(iteratroID)  + e4B(iteratroID) .*e4B(iteratroID) ;
                    e5B(iteratroID)  = val_cp_m2(iteratroID)  - realVal_m2(iteratroID) ;ee(iteratroID)  = ee(iteratroID)  + e5B(iteratroID) .*e5B(iteratroID) ;
                end
                
                
                %% do I have a new winner??
                %%if so: set.
                bestMacthErrIds = false(size(iteratroID,1),size(iteratroID,2));
                bestMacthErrIds(iteratroID) = ee(iteratroID) < best_match_err(iteratroID);
                if any(bestMacthErrIds)
                    %%put to second-best
                    second_best_match_err(bestMacthErrIds)=best_match_err(bestMacthErrIds);
                    loopCSecond(bestMacthErrIds)= loopCBest(bestMacthErrIds);
                    
                    %% set best.
                    best_match_err(bestMacthErrIds) = ee(bestMacthErrIds);
                    loopCBest(bestMacthErrIds) = loopCounter(bestMacthErrIds);
                    
                    best_match_errPre(bestMacthErrIds) = eeLast(bestMacthErrIds);
                    best_match_DiffErrPre(bestMacthErrIds) = e1A(bestMacthErrIds).*e1B(bestMacthErrIds) + e2A(bestMacthErrIds).*e2B(bestMacthErrIds) + e3A(bestMacthErrIds).*e3B(bestMacthErrIds) + e4A(bestMacthErrIds).*e4B(bestMacthErrIds) + e5A(bestMacthErrIds).*e5B(bestMacthErrIds);
                    best_match_errPost(bestMacthErrIds) = -1;
                    best_match_DiffErrPost(bestMacthErrIds) = -1;
                    
                    best_match_x(bestMacthErrIds) = cpx(bestMacthErrIds);
                    best_match_y(bestMacthErrIds) = cpy(bestMacthErrIds);
                    bestWasLastLoop = false(size(val_cp_p1,1),size(val_cp_p1,2)) ;
                    bestWasLastLoop(bestMacthErrIds) = true;
                    
                    %% otherwise: the last might be the current winner, in which case i have to save these values.
                    NotbestMacthErrIds = ~bestMacthErrIds;
                    if any(NotbestMacthErrIds)
                        if any(bestWasLastLoop(NotbestMacthErrIds))
                            
                            best_match_errPost(NotbestMacthErrIds & bestWasLastLoop) = ee(NotbestMacthErrIds &  bestWasLastLoop);
                            best_match_DiffErrPost(NotbestMacthErrIds &  bestWasLastLoop) = e1A(NotbestMacthErrIds &  bestWasLastLoop).*e1B(NotbestMacthErrIds &  bestWasLastLoop) + e2A(NotbestMacthErrIds &  bestWasLastLoop).*e2B(NotbestMacthErrIds &  bestWasLastLoop) + e3A(NotbestMacthErrIds &  bestWasLastLoop).*e3B(NotbestMacthErrIds &  bestWasLastLoop) + e4A(NotbestMacthErrIds &  bestWasLastLoop).*e4B(NotbestMacthErrIds &  bestWasLastLoop) + e5A(NotbestMacthErrIds &  bestWasLastLoop).*e5B(NotbestMacthErrIds &  bestWasLastLoop);
                            bestWasLastLoop(NotbestMacthErrIds &  bestWasLastLoop) = false;
                        end
                        
                        %% collect second-best:
                        %% just take the best of all that are NOT equal to current best.
                        secondBestMatchIds = false(size(iteratroID,1),1);
                        secondBestMatchIds(iteratroID)   =  (ee(iteratroID) < second_best_match_err(iteratroID)) ;
                        
                        second_best_match_err(NotbestMacthErrIds & secondBestMatchIds)=ee(NotbestMacthErrIds & secondBestMatchIds);
                        loopCSecond(NotbestMacthErrIds & secondBestMatchIds) = loopCounter(NotbestMacthErrIds &secondBestMatchIds);
                        
                    end
                end
                
                %% shift everything one further.
                eeLast(iteratroID)  = ee(iteratroID) ;
                val_cp_m2(iteratroID)  = val_cp_m1(iteratroID) ;
                val_cp_m1(iteratroID)  = val_cp(iteratroID) ;
                val_cp(iteratroID)  = val_cp_p1(iteratroID) ;
                val_cp_p1(iteratroID)  = val_cp_p2(iteratroID) ;
                cpx(iteratroID)  = cpx(iteratroID)  + ix(iteratroID) ;
                cpy(iteratroID)  = cpy(iteratroID)  + iy(iteratroID) ;
                loopCounter(iteratroID)  = loopCounter(iteratroID)  + 1;
                
                
                
                %iteratroID = (incx < 0) == (cpx > pClose(:,1)) & (incy < 0) == (cpy > pClose(:,2));
                iteratroID =  (ix < 0) == (cpx > pC1) & (iy < 0) == (cpy > pC2);
                loopCounterAll = loopCounterAll + 1;
            end
            %    val_cp_p2;
            
            tempIDs =  ~(best_match_err > 4.0 * globalParams.MAX_ERROR_STEREO);
            validIDs = tempIDs;
            tempIDsInd = indUV1(tempIDs);
            logiMat = false(size(errors));
            logiMat(tempIDsInd) = true;
            if 0
                errors(~tempIDs) = -3;
            else
                errors(~logiMat) = -3;
            end
            %{
	 pFar = pFar(uu - 3 + 1 , vv - 3 + 1,:);%pFar(validIDs , :);
     pClose = pClose(uu - 3 + 1 , vv - 3 + 1,:);%(validIDs , :);
     loopCBest = loopCBest(uu - 3 + 1 , vv - 3 + 1,:);%(validIDs);
     loopCSecond = loopCSecond(uu - 3 + 1 , vv - 3 + 1,:);%(validIDs);
     rescaleFactor = rescaleFactor(uu - 3 + 1 , vv - 3 + 1,:);%(validIDs);
        second_best_match_err =  second_best_match_err(uu - 3 + 1 , vv - 3 + 1,:);%(validIDs);
        KinvP = KinvP(uu - 3 + 1 , vv - 3 + 1,:);%(validIDs,:);
        
         u = u(uu - 3 + 1 , vv - 3 + 1,:);%(validIDs);
                v = v(uu - 3 + 1 , vv - 3 + 1,:);%(validIDs);
            %}
            %%check if clear enough winner
            %validIDs &
            tempIDs = ~((abs(loopCBest - loopCSecond) > 1.0 )& (globalParams.MIN_DISTANCE_ERROR_STEREO .* best_match_err > second_best_match_err));
            
             tempIDsInd1 = indUV1(tempIDs);
            logiMat1 = false(size(errors));
            logiMat1(tempIDsInd1) = true;
            
            
            validIDs = validIDs & tempIDs;
            if 0
                errors(~tempIDs) = -2;
            else
                errors(~logiMat1) = -2;
            end
            %{
    pFar = pFar(validIDs , :);
     pClose = pClose(validIDs , :);
	 rescaleFactor = rescaleFactor(validIDs);
      KinvP = KinvP(validIDs,:);
      
      
         u = u(validIDs);
                v = v(validIDs);
            %}
            %% have to implement if(useSubpixelStereo)
            
            newValidIDs = zeros(size(referenceFrame.validIDs,1),size(referenceFrame.validIDs,2));
            gradAlongLine  = zeros(size(referenceFrame.validIDs,1),size(referenceFrame.validIDs,2));
            bm_error = zeros(size(referenceFrame.validIDs,1),size(referenceFrame.validIDs,2));
            bm_x = zeros(size(referenceFrame.validIDs,1),size(referenceFrame.validIDs,2));
            bm_y = zeros(size(referenceFrame.validIDs,1),size(referenceFrame.validIDs,2));
            if 0
                for i = 1 : size(validIDs)
                    if (validIDs(i))
                        newValidIDs(uu(i) - 3 + 1 , vv(i) - 3 + 1) = 1;
                        
                        tmp = realVal_p2(i ,1) - realVal_p1(i,1);
                        gradAlongLine(uu(i) - 3 + 1 , vv(i) - 3 + 1) = gradAlongLine(uu(i) - 3 + 1 , vv(i) - 3 + 1) + (tmp.*tmp);
                        
                        tmp = realVal_p1(i ,  1) - realVal(i ,  1);
                        gradAlongLine(uu(i) - 3 + 1 , vv(i) - 3 + 1) = gradAlongLine(uu(i) - 3 + 1 , vv(i) - 3 + 1) + (tmp.*tmp);
                        
                        tmp = realVal(i ,  1) - realVal_m1(i ,  1);
                        gradAlongLine(uu(i) - 3 + 1 , vv(i) - 3 + 1) = gradAlongLine(uu(i) - 3 + 1 , vv(i) - 3 + 1) + (tmp.*tmp);
                        
                        tmp = realVal_m1(i ,  1) - realVal_m2(i ,  1);
                        gradAlongLine(uu(i) - 3 + 1 , vv(i) - 3 + 1) = gradAlongLine(uu(i) - 3 + 1 , vv(i) - 3 + 1) + (tmp.*tmp);
                        
                        bm_error(uu(i) - 3 + 1 , vv(i) - 3 + 1) = best_match_err(i,1);
                        bm_x(uu(i) - 3 + 1 , vv(i) - 3 + 1) = best_match_x(i,1);
                        bm_y(uu(i) - 3 + 1 , vv(i) - 3 + 1) = best_match_y(i,1);
                    end
                end
            end
            
            Tmp1 = realVal_p2(validIDs ,1) - realVal_p1(validIDs,1);
            Tmp2 = realVal_p1(validIDs ,  1) - realVal(validIDs ,  1);
            Tmp3 = realVal(validIDs ,  1) - realVal_m1(validIDs ,  1);
            Tmp4 = realVal_m1(validIDs ,  1) - realVal_m2(validIDs ,  1);
            
            uuvv = [uu(validIDs) vv(validIDs)];
            uuvvOfst = [uu(validIDs)-3+1 vv(validIDs)-3+1];
            if 0
                indUV = sub2ind(size(gradAlongLine), uuvvOfst(:,1), uuvvOfst(:,2));
            else
                indUV = sub2ind(size(gradAlongLine), uuvvOfst(:,2), uuvvOfst(:,1));
            end
            
            gradAlongLine(indUV) = gradAlongLine(indUV) + Tmp1.^2 + Tmp2.^2 + Tmp3.^2 + Tmp4.^2;
            newValidIDs(indUV) = 1;
            bm_error(indUV) = best_match_err(validIDs);
            bm_x(indUV) = best_match_x(validIDs,1);
            bm_y(indUV) = best_match_y(validIDs,1);
            
            %     close all;
%             figure,imshow(newValidIDs)
            referenceFrame.validIDs = newValidIDs;
            %% sampleDist is the distance in pixel at which the realVal's were sampled
            sampleDist =  referenceFrame.validIDs .*(globalParams.GRADIENT_SAMPLE_DIST.* rescaleFactor);
            gradAlongLine = referenceFrame.validIDs .*(gradAlongLine./(sampleDist.*sampleDist));
            
            
            
            %% check if interpolated error is OK. use evil hack to allow more error if there is a lot of gradient.
            %%% not working
            %%%referenceFrame.validIDs  = referenceFrame.validIDs  & ~(bm_error > ( globalParams.MAX_ERROR_STEREO + sqrt( gradAlongLine).*20));
            tempIDs = ~(bm_error > ( globalParams.MAX_ERROR_STEREO + sqrt( gradAlongLine).*20));
            
            % %             tempIDsInd2 = indUV1(tempIDs);
            % %             logiMat2 = false(size(errors));
            % %             logiMat2(tempIDsInd2) = true;
            
            if 1
                errors(~tempIDs) = -3;
            else
                
                tempIDsInd2 = indUV1(tempIDs);
                logiMat2 = false(size(errors));
                logiMat2(tempIDsInd2) = true;
                
                errors(~logiMat2) = -3;
            end
            referenceFrame.validIDs  = referenceFrame.validIDs  & tempIDs;
            
            pFar = pFar .*  referenceFrame.validIDs;
            pClose = pClose .*  referenceFrame.validIDs;
            rescaleFactor = rescaleFactor .*  referenceFrame.validIDs;
            KinvP = KinvP .*  referenceFrame.validIDs;
            
            %% ================= calc depth (in KF) ====================
            %% * KinvP = Kinv * (x,y,1); where x,y are pixel coordinates of point we search for, in the KF.
            %% * best_match_x = x-coordinate of found correspondence in the reference frame.
            %idnew_best_match;	%% depth in the new image
            %alpha;  %% d(idnew_best_match) / d(disparity in pixel) == conputed inverse depth derived by the pixel-disparity.
            
            dot0_w = zeros(size(referenceFrame.validIDs,1),size(referenceFrame.validIDs,2));
            dot2_w = zeros(size(referenceFrame.validIDs,1),size(referenceFrame.validIDs,2));
            idnew_best_match = zeros(size(referenceFrame.validIDs,1),size(referenceFrame.validIDs,2));
            alpha = zeros(size(referenceFrame.validIDs,1),size(referenceFrame.validIDs,2));
            XGrtY = (incx.*incx)>(incy.*incy);
            YGrtX = ~XGrtY;
            XGrtY = XGrtY & referenceFrame.validIDs;
            YGrtX = YGrtX & referenceFrame.validIDs;
            
            %oldX = obj.fxi*best_match_x(XGrtY)+obj.cxi;
            %nominator = (oldX*referenceFrame.otherToThis_t(3) - referenceFrame.otherToThis_t(1));
            
            oldX = XGrtY .* (obj.fxi.*bm_x + obj.cxi);
            oldY = YGrtX .* (obj.fyi.*bm_y + obj.cyi);
            nominatorX = (oldX*referenceFrame.otherToThis_t(3)) - referenceFrame.otherToThis_t(1);
            nominatorY = (oldY*referenceFrame.otherToThis_t(3)) - referenceFrame.otherToThis_t(2);
            
            [uu,vv] = find(XGrtY);
            skipInterval = size(KinvP,1)*size(KinvP,2);
            if 0
                for i = 1: size(uu,1)          
                    nom =  nominatorX(uu(i) , vv(i) );
                    
                    dot0 = reshape(KinvP(uu(i)  , vv(i),:),1,3) * (referenceFrame.otherToThis_R(1,:))';
                    dot2 = reshape(KinvP(uu(i)  , vv(i),:),1,3) * (referenceFrame.otherToThis_R(3,:))';
                    
                    dot0_w(uu(i)  , vv(i) ) = dot0;
                    dot0_w(uu(i)  , vv(i) ) = dot0;
                    
                    idnew_best_match(uu(i)  , vv(i) ,:) = (dot0 - oldX(uu(i) , vv(i) )*dot2) /nom;
                    alpha (uu(i)  , vv(i) ,:) = incx(uu(i)  , vv(i) ,:)*obj.fxi*(dot0 * referenceFrame.otherToThis_t(3) - dot2 * referenceFrame.otherToThis_t(1)) ./ (nom*nom);        
                end
            end
            indxy = find(XGrtY);
            nomList =  nominatorX(indxy);
            mat3 = [KinvP(indxy) KinvP(indxy + skipInterval) KinvP(indxy + 2*skipInterval)];
            dot0Mat = dot(mat3', repmat((referenceFrame.otherToThis_R(1,:))',1,length(indxy)));
            dot2Mat = dot(mat3', repmat((referenceFrame.otherToThis_R(3,:))',1,length(indxy)));
%             dot0_w_mat = dot0_w;
            dot0_w(indxy) = dot0Mat;
            idnew_best_match(indxy) = (dot0Mat' - oldX(indxy).*dot2Mat') ./nomList;
            alpha(indxy) = incx(indxy).*obj.fxi.*(dot0Mat' .* referenceFrame.otherToThis_t(3) - dot2Mat' .* referenceFrame.otherToThis_t(1)) ./ (nomList.^2);

            [uu,vv] = find(YGrtX);
            if 0
                for i = 1: size(uu,1)
                    nom =  nominatorY(uu(i)  , vv(i) );
                    dot1 = reshape(KinvP(uu(i)  , vv(i) ,:),1,3) * (referenceFrame.otherToThis_R(2,:))';
                    dot2 = reshape(KinvP(uu(i)  , vv(i) ,:),1,3)  * (referenceFrame.otherToThis_R(3,:))';
                    
                    idnew_best_match(uu(i) , vv(i) ,:) = (dot1  - oldY(uu(i)  , vv(i) )*dot2 ) / nom;
                    alpha(uu(i)  , vv(i) ,:)  = incy(uu(i)  , vv(i) ,:)*obj.fyi*(dot1 * referenceFrame.otherToThis_t(3)- dot2 * referenceFrame.otherToThis_t(2)) ./ (nom*nom);
                end
            end
            
            indxy = find(YGrtX);
            nomList =  nominatorY(indxy);
            mat3 = [KinvP(indxy) KinvP(indxy + skipInterval) KinvP(indxy + 2*skipInterval)];
            dot1Mat = dot(mat3', repmat((referenceFrame.otherToThis_R(2,:))',1,length(indxy)));
            dot2Mat = dot(mat3', repmat((referenceFrame.otherToThis_R(3,:))',1,length(indxy)));
%             dot0_w_mat = dot0_w;
%             dot0_w(indxy) = dot0Mat;
            idnew_best_match(indxy) = (dot1Mat' - oldY(indxy).*dot2Mat') ./nomList;
            alpha(indxy) = incy(indxy).*obj.fyi.*(dot1Mat' .* referenceFrame.otherToThis_t(3) - dot2Mat' .* referenceFrame.otherToThis_t(2)) ./ (nomList.^2);
            
            
            
            
            tempIDs = ~(idnew_best_match < 0);
            referenceFrame.validIDs = referenceFrame.validIDs & tempIDs;
            
            %             tempIDsInd3 = indUV1(tempIDs);
            %             logiMat3 = false(size(errors));
            %             logiMat3(tempIDsInd3) = true;
            
            
            if 1
                errors(~tempIDs)=-2;
            else
                tempIDsInd3 = indUV1(tempIDs);
                logiMat3 = false(size(errors));
                logiMat3(tempIDsInd3) = true;
                
                errors(~logiMat3)=-2;
            end
            %% ================= calc var (in NEW image) ====================
            
            photoDispError = referenceFrame.validIDs .*(4.0 *  globalParams.cameraPixelNoise2 ./ (gradAlongLine + globalParams.DIVISION_EPS));
            trackingErrorFac = 0.25*(1.0+referenceFrame.initialTrackedResidual);
            
            %[activeKeyFrameGmag_0,activeKeyFrameGdir_0] = imgradient(rgb2gray(obj.activeKeyFrame.imgRGB));
            activeImageGray = rgb2gray(obj.activeKeyFrame.imgRGB);
            filter = [0 -1 0; -1 0 1;0 1 0];
            grad =  double(imfilter(activeImageGray,filter));
            
            
            filter = [-1 0 1];
            [h, w] = size(activeImageGray);
            gx =  double(imfilter(activeImageGray,filter));
            gy =  double(imfilter(activeImageGray,filter'));
            gx = gx(3: h - 3, 3 : w-3);
            gy = gy(3: h - 3, 3 : w-3);
            
            %{
````%do we need this ?
    gradsInterpX =  interp2(x,y,gx,uu , vv , 'cubic');
    gradsInterpY =  interp2(x,y,gy,uu , vv , 'cubic');
    geoDispError = zeros(size(grad,1), size(grad,2));
    
     for i = 1:size(gradsInterp,1)
        geoDispError(uu(i) - 3 + 1 , vv(i) - 3 + 1) = (gradsInterpX(i)*epxn(uu(i) - 3 + 1 , vv(i) - 3 + 1,:)  + gradsInterpY(i)*epyn(uu(i) - 3 + 1 , vv(i) - 3 + 1,:) ) + globalParams.DIVISION_EPS;
    end
   
            %}
            geoDispError =  ((gx .*  epxn  + gy.*epyn) + globalParams.DIVISION_EPS);
            geoDispError = trackingErrorFac.*trackingErrorFac.*((gx.*gx) + (gy.*gy)) ./ (geoDispError.*geoDispError);
            geoDispError = referenceFrame.validIDs .* geoDispError;
            %% final error consists of a small constant part (discretization error),
            %% geometric and photometric error.
            %geoDispError(isnan(geoDispError)) = 0;
            %photoDispError(isnan(photoDispError)) = 0;
            %geoDispError(isinf(geoDispError)) = 0;
            %photoDispError(isinf(photoDispError)) = 0;
            result_var = referenceFrame.validIDs .*(alpha.*alpha .* ((0.5).*sampleDist.*sampleDist +  geoDispError + photoDispError));
            result_var(isnan(result_var)) = 0;
            
            result_idepth = referenceFrame.validIDs .*idnew_best_match;
            
            result_eplLength = referenceFrame.validIDs .*eplLength;
            
            
            
            
            %xlim([loc1(1)-5, loc1(1)+4]);
            %ylim([loc1(2)-5, loc1(2)+4]);
            %zlim([loc1(3)-1, loc1(3)+20]);
            %camorbit(0, -30);
            %{
	
	%% calculate error from geometric noise (wrong camera pose / calibration)
	Eigen::Vector2f gradsInterp = getInterpolatedElement42(activeKeyFrame->gradients(0), u, v, width);
	float geoDispError = (gradsInterp[0]*epxn + gradsInterp[1]*epyn) + DIVISION_EPS;
	geoDispError = trackingErrorFac*trackingErrorFac*(gradsInterp[0]*gradsInterp[0] + gradsInterp[1]*gradsInterp[1]) / (geoDispError*geoDispError);


	%%geoDispError *= (0.5 + 0.5 *result_idepth) * (0.5 + 0.5 *result_idepth);

	%% final error consists of a small constant part (discretization error),
	%% geometric and photometric error.
	result_var = alpha*alpha*((didSubpixel ? 0.05f : 0.5f)*sampleDist*sampleDist +  geoDispError + photoDispError);	// square to make variance

    
	%% calculate error from photometric noise
            %}
            
            
        end
        function observeDepthRow(obj, yMin , yMax)
            %%loads of code goes here
            [x , y] = meshgrid(yMin:yMax,3 :obj.width - 3 );
            obj.observeDepthCreate(x,y);
            
        end
        
        function observeDepthCreate(obj , x, y)
            obj.makeAndCheckEPLMat( x , y , obj.newest_referenceFrame);
        end
        
        
    end
end