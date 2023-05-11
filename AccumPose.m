function accumAngSlam = AccumPose(slamPoseMat)
accumAngSlam = 0;
for q = 1 : size(slamPoseMat,1)
   
    rmatSlam = reshape(slamPoseMat(q,1:9),3,3);
    tvecSlam = reshape(slamPoseMat(q,10:12),3,1);
    if q > 1
        dltTSlam =  [rmatSlam tvecSlam;0 0 0 1]*inv([reshape(slamPoseMat(q-1,1:9),3,3) slamPoseMat(q-1,10:12)';0 0 0 1]);
        dltrSlam = dltTSlam(1:3,1:3);
        dlttSlam = dltTSlam(1:3,end);
        prvPtSlam = [slamPoseMat(q-1,10);0;slamPoseMat(q-1,12)];
        curPtSlam = [slamPoseMat(q,10);0;slamPoseMat(q,12)];
        dltRotVec = rodrigues(dltrSlam);
        dltAngSlam = rad2deg(norm(rodrigues(dltrSlam)));
        errSlam = dltrSlam*prvPtSlam + dlttSlam - curPtSlam;
%         if norm(rodrigues(roty(dltAngSlam)) - rodrigues(dltrSlam)) > norm(rodrigues(rotz(-dltAngSlam)) - rodrigues(dltrSlam))
%             accumAngSlam = [accumAngSlam; accumAngSlam(end) + dltAngSlam];
%         else
%             accumAngSlam = [accumAngSlam; accumAngSlam(end) - dltAngSlam];
%         end
         if dltRotVec(2) > 0
            accumAngSlam = [accumAngSlam; accumAngSlam(end) - dltAngSlam];
        else
            accumAngSlam = [accumAngSlam; accumAngSlam(end) + dltAngSlam];
        end
    end
    % %         if rotVecTmp(3) < 0
    % %             rotVecTmp = -rotVecTmp;
    % %         end
    
    
    
%     err = rt(1:3,1:3)*xyzCB' + repmat(rt(:,end),1,4) - initIntersectPtRnd';
%     [~,err2] = NormalizeVector(err');
%     Err(q,:) = err2';
%     camPosePath2(q,:) = [reshape(rt(:,1:3),1,9) rt(:,end)'];
    
end
end