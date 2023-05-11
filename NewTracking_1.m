function [curPredPtIcs,inTraceFlag,curPredPtIcs0,inTraceFlag0] = NewTracking_1(obj,keyFeatNum,prevFeatPtList,ptCcsZ,intrMat,k2cRef,angleModalOrg, keyDepthV, prevDepthV, currDepthV)
            
            
            
            OnlyP2 = false; true;
            
            
            f = intrMat(1); baseline = norm(obj.camModel.transVec1To2);
            fbConst = f*baseline;
            [princpPtL, princpPtR] = Get(obj.camModel, 'PinholePrincpPt', obj.scaleLvl);
            
            T_B2C = obj.coordSysAligner.pctBody2Cam(1, 1).transformMat;
            invTb2c = inv(T_B2C);
            featuresNumber = keyFeatNum;
            activeFeat =  find(obj.featPtManager.localTrace.ptIcsX(:,end) ~= -1 & obj.featPtManager.localTrace.ptIcsY(:,end) ~=-1);
            ptIcX_pre = obj.featPtManager.localTrace.ptIcsX(:,1);
            ptIcY_pre = obj.featPtManager.localTrace.ptIcsY(:,1);
%             pt1 = Points.Point(ptIcX_pre,ptIcY_pre,intrMat);
            pt0 = Points.Point(ptIcX_pre,ptIcY_pre,intrMat);
            ptCcsZGolden = obj.featPtManager.localTrace.ptCcsZ;

            %             m_ = Modals.Modal(obj.setting.configParam, featuresNumber);
%             m_ = obj.modals;
            %             angleModalOrg = m_.angleModal;
            
            
            
            
            
%             depthModalOrg = m_.depthModal;
            nextImg = obj.currImgL;
%             prevImg = obj.keyFrameImgL;
            prevImg = obj.prevImgL;
            imgSize = size(nextImg); imgSize = imgSize(1:2);
            
%             if isempty(obj.refAngList)
            if size(obj.featPtManager.localTrace.ptIcsX,2) == 1
                angleOrgin = 0;
            else
                angleOrgin = rad2deg(obj.refAngList3(end));
            end
            [pt0.matchingPtX,pt0.matchingPtY,pervPtCcsZGolden] = Points.GenerateMatchingPt(pt0,angleOrgin,T_B2C,invTb2c,ptCcsZGolden,featuresNumber,intrMat) ;         
            
%             obj.PervPtCcsZGolden = pervPtCcsZGolden;
            
            angleOrgin = rad2deg(k2cRef);
            [pt00.matchingPtX,pt00.matchingPtY,pervPtCcsZGolden00] = Points.GenerateMatchingPt(pt0,angleOrgin,T_B2C,invTb2c,ptCcsZGolden,featuresNumber,intrMat) ;
            
            ptIcX_pre = -1 * ones(featuresNumber,1);
            ptIcY_pre = -1 * ones(featuresNumber,1);
            if size(obj.featPtManager.localTrace.ptIcsX,2) == 1
                
                ptIcX_pre = obj.featPtManager.localTrace.ptIcsX;
                ptIcY_pre = obj.featPtManager.localTrace.ptIcsY;
            else
                
                existIdx0 = find(obj.featPtManager.localTrace.ptIcsX(:,end)-obj.setting.wx > 1 & obj.featPtManager.localTrace.ptIcsY(:,end)-obj.setting.wy> 1 ...
                    & obj.featPtManager.localTrace.ptIcsX(:,end)+obj.setting.wx < imgSize(2) & obj.featPtManager.localTrace.ptIcsY(:,end)+obj.setting.wy< imgSize(1) ...
                    & obj.featPtManager.localTrace.ptIcsX(:,end-1)-obj.setting.wx > 1 & obj.featPtManager.localTrace.ptIcsY(:,end-1)-obj.setting.wy> 1 ...
                    & obj.featPtManager.localTrace.ptIcsX(:,end-1)+obj.setting.wx < imgSize(2) & obj.featPtManager.localTrace.ptIcsY(:,end-1)+obj.setting.wy< imgSize(1) ...
                    &  obj.PervPtCcsZGolden~=-1);
                ptIcX_pre(existIdx0) = obj.featPtManager.localTrace.ptIcsX(existIdx0,end);
                ptIcY_pre(existIdx0) = obj.featPtManager.localTrace.ptIcsY(existIdx0,end);
                
                
            end
            pt1 = Points.Point(ptIcX_pre,ptIcY_pre,intrMat);
            ptCcsZGolden = pervPtCcsZGolden';
%             if isempty(obj.refAngList)
            if size(obj.featPtManager.localTrace.ptIcsX,2) == 1
                angleOrgin = rad2deg(k2cRef);
            else
                angleOrgin = rad2deg(k2cRef - obj.refAngList3(end));
            end
            [pt1.matchingPtX,pt1.matchingPtY,curPtCcsZGolden] = Points.GenerateMatchingPt(pt1,angleOrgin,T_B2C,invTb2c,ptCcsZGolden,featuresNumber,intrMat) ;
            
            obj.setting.angleOrginRange = angleOrgin + obj.setting.angleRange;
            obj.setting.configParam.angleOrginRange = obj.setting.angleOrginRange;
            
            
            %              validInd = sub2ind(size(obj.depthVisual), round(pt1.y), round(pt1.x));
            %               ptCcsZGolden = obj.keyFrameDepthGT(validInd);
            
            if 0
                [pt1.matchingPtX,pt1.matchingPtY] = Points.GenerateMatchingPt(pt1,angleOrgin,T_B2C,invTb2c,ptCcsZGolden,featuresNumber,intrMat) ;
                
                
                existIdx = find(pt1.matchingPtX-obj.setting.configParam.wx >= 1 & pt1.matchingPtY-obj.setting.configParam.wy>= 1 ...
                    & pt1.matchingPtX+obj.setting.configParam.wx <= imgSize(2) & pt1.matchingPtY+obj.setting.configParam.wy<= imgSize(1) ...
                    &  ptCcsZ~=-1 & ptCcsZGolden ~= -1);
            elseif 0
                [pt1.matchingPtX,pt1.matchingPtY] = Points.GenerateMatchingPt(pt1,angleOrgin,T_B2C,invTb2c,ptCcsZ,featuresNumber,intrMat) ;
                
                
                existIdx = find(pt1.matchingPtX-obj.setting.configParam.wx >= 1 & pt1.matchingPtY-obj.setting.configParam.wy>= 1 ...
                    & pt1.matchingPtX+obj.setting.configParam.wx <= imgSize(2) & pt1.matchingPtY+obj.setting.configParam.wy<= imgSize(1) ...
                    &  ptCcsZ~=-1); % & ptCcsZGolden ~= -1);
            else
                [pt1.matchingPtX,pt1.matchingPtY] = Points.GenerateMatchingPt(pt1,angleOrgin,T_B2C,invTb2c,ptCcsZ,featuresNumber,intrMat) ;
                
                if 0
                    existIdx = find(pt1.matchingPtX-obj.setting.configParam.wx >= 1 & pt1.matchingPtY-obj.setting.configParam.wy>= 1 ...
                        & pt1.matchingPtX+obj.setting.configParam.wx <= imgSize(2) & pt1.matchingPtY+obj.setting.configParam.wy<= imgSize(1) ...
                        & ptCcsZGolden ~= -1); % & ptCcsZ~=-1
                else
                    % %                     existIdx = find(pt1.matchingPtX-obj.setting.configParam.wx >= 1 & pt1.matchingPtY-obj.setting.configParam.wy>= 1 ...
                    % %                         & pt1.matchingPtX+obj.setting.configParam.wx <= imgSize(2) & pt1.matchingPtY+obj.setting.configParam.wy<= 126 ...
                    % %                         & ptCcsZGolden ~= -1);
                    
                    existIdx = find(pt1.matchingPtX-obj.setting.configParam.wx > 1 & pt1.matchingPtY-obj.setting.configParam.wy> 1 ...
                        & pt1.matchingPtX+obj.setting.configParam.wx < imgSize(2) & pt1.matchingPtY+obj.setting.configParam.wy< 126*(240/imgSize(1)) ...
                        & pt1.x-obj.setting.configParam.wx > 1 & pt1.y-obj.setting.configParam.wy> 1 ...
                        & pt1.x+obj.setting.configParam.wx < imgSize(2) & pt1.y+obj.setting.configParam.wy< 126*(240/imgSize(1)) ...
                        &  ptCcsZGolden~=-1);
                    
                    
                end
                
            end
            
            obj.PervPtCcsZGolden = ptCcsZGolden;
            existFeaturesNumber = size(existIdx,1);
            existPt1 = Points.Point(pt1.x(existIdx),pt1.y(existIdx),intrMat);
            existPt2 = Points.Point(pt1.matchingPtX(existIdx),pt1.matchingPtY(existIdx),intrMat);
            
            existPtCcsZG = ptCcsZGolden(existIdx);
            existPtDisparityG1 = fbConst./existPtCcsZG;
            existPtDisparityG = (intrMat(1,1).*norm(obj.camModel.transVec1To2)./existPtCcsZG) - (princpPtR(1) - princpPtL(1));
            
            
            
            
            obj.setting.configParam.disparityOrginRange   = existPtDisparityG + obj.setting.configParam.disparityRange;
            
%             table = LookUpTables.LookUpTable(obj.lookUpTables,existFeaturesNumber,obj.setting.configParam.tableSize); %initialize
            table = LookUpTables.LookUpTable(existFeaturesNumber,obj.setting.configParam.tableSize); %initialize
            table.winSize = obj.lookUpTables.winSize;
            table.angSize = obj.lookUpTables.angSize;
            table.angMove = obj.lookUpTables.angMove;
            if 1 %~OnlyP2
                table = LookUpTables.GenerateTable(table,obj.setting.configParam,existPt1,existFeaturesNumber, imgSize,T_B2C,fbConst,intrMat) ;
            else
                table = LookUpTables.GenerateTableOnlyP2(table,obj.setting.configParam,existPt1,existPt2,existFeaturesNumber, imgSize,T_B2C,invTb2c,fbConst,intrMat) ;        
            end
% %             m_.angleModal = angleModalOrg;
%             if ~OnlyP2
%                 % -- update modal angle -- %
%                 m_.angleModal = Modals.UpdateModalAngle(m_,obj.setting.configParam,table,existIdx);
%                 % -- update modal depth -- %
%                 m_.depthModal = Modals.UpdateModalDepth(m_,obj.setting.configParam,table,existIdx,existFeaturesNumber);
%             else
%                 m_.angleModal = Modals.UpdateModalAngleOnlyP2(m_,obj.setting.configParam,table,existIdx);
%                 
%             end
% % %             if ~OnlyP2
% % %                 table.candidatesWeight = LookUpTables.GenerateTableWeight(table,obj.setting.configParam,m_,existIdx);
% % %             else
% % %                 table.candidatesWeight = LookUpTables.GenerateTableWeightOnlyP2(table,obj.setting.configParam,m_,existIdx);
% % %             end
            % ------get tracing pts with weights------ %
            tracePt = TraceWithWeights.TraceWithWeight(existFeaturesNumber);%initialize
            tracePt = TraceWithWeights.Tracing(tracePt,obj.setting.configParam,table,prevImg,nextImg,existPt1,existFeaturesNumber,imgSize); % new tracing Pts
            
            err_x1 = pt00.matchingPtX(existIdx) - tracePt.x;
            err_y1 = pt00.matchingPtY(existIdx) - tracePt.y;
            
            
            % -----re-calculate guassian---- %
% % % %             if ~OnlyP2
% % % %                 table = LookUpTables.RecalculateTableGauss(table,obj.setting.configParam,tracePt,existFeaturesNumber);
% % % %             else
% % % %                 table = LookUpTables.RecalculateTableGaussOnlyP2(table,obj.setting.configParam,tracePt,existFeaturesNumber);
% % % %             end
            % -----re-calculate modal angle---- %
% % % % % % % %             m_.angleModal = angleModalOrg;
% % % % % % % %             
% % % % % % % %             if ~OnlyP2
% % % % % % % %                 m_.angleModal = Modals.UpdateModalAngle(m_,obj.setting.configParam,table,existIdx);
% % % % % % % %                 % -----re-calculate modal depth---- %
% % % % % % % %                 m_.depthModal = depthModalOrg;
% % % % % % % %                 m_.depthModal = Modals.UpdateModalDepth(m_,obj.setting.configParam,table,existIdx,existFeaturesNumber);
% % % % % % % %             else
% % % % % % % %                 m_.angleModal = Modals.UpdateModalAngleOnlyP2(m_,obj.setting.configParam,table,existIdx);
% % % % % % % %             end
            %             err = [ptIcX_cur(existIdx) - tracePt.x ptIcY_cur(existIdx) - tracePt.y];[~,errr]=NormalizeVector(err);
            %         figure(9),clf;plot(err(errr<1,1),err(errr<1,2),'+r')
            if 0
                figure,imshow(prevImg);hold on;plot(existPt1.x,existPt1.y,'.r')
                figure,imshow(nextImg);hold on;plot(tracePt.x,tracePt.y,'.r')
            end
            
            topMargin = obj.featPtTracker.configParam.top_margin;
            leftMargin = obj.featPtTracker.configParam.left_margin;
            bottomMargin = obj.featPtTracker.configParam.bottom_margin;
            rightMargin = obj.featPtTracker.configParam.right_margin;
            
            inBndFlag = tracePt.x(:, 1) >= leftMargin + 1 & ...
                tracePt.x(:, 1) <= Cols(nextImg) - rightMargin & ...
                tracePt.y(:, 1) >= topMargin + 1 & ...
                tracePt.y(:, 1) <= Rows(nextImg) - bottomMargin;
            inTrackFlag =  inBndFlag;
            %             validNewPt = tracePt.x;
            
            existIdx(~inTrackFlag) = [];
            inTraceFlag = false(keyFeatNum,1);
            inTraceFlag(existIdx) = true;
            
            curPredPtIcs = -1.*ones(keyFeatNum,2);
            curPredPtIcs(existIdx,:) = [tracePt.x(inTrackFlag) tracePt.y(inTrackFlag)];
            
            curPredPtIcs0 = curPredPtIcs;
            inTraceFlag0 = inTraceFlag;
            
            curPredPtIcs = curPredPtIcs(activeFeat,:);
            inTraceFlag = inTraceFlag(activeFeat,:);
%             obj.modals = m_;
        end