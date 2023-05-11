function accumAngSlam = AccumAng(vslMat4)


close all

vslMat4 = vslMat4(100:end-200,:);

accumAngSlam = 0;
slamPoseMat = [];
rotVecTmp = [];
for q = 1 : size(vslMat4,1)
    
    rt = [reshape(vslMat4(q,1:9),3,3) vslMat4(q,10:12)'];

    rmatSlam = rt(1:3,1:3);
    rotVecTmp(q,:) = rodrigues(rt(1:3,1:3));
    slamPoseMat(q,:) = [reshape(rt(1:3,1:3),1,9) rt(:,end)']';
    
    if q > 1
        dltTSlam =  [rmatSlam rt(:,end);0 0 0 1]*inv([reshape(slamPoseMat(end-1,1:9),3,3) slamPoseMat(end-1,10:12)';0 0 0 1]);
        dltrSlam = dltTSlam(1:3,1:3);
        dlttSlam = dltTSlam(1:3,end);
        prvPtSlam = [slamPoseMat(end-1,10);0;slamPoseMat(end-1,12)];
        curPtSlam = [slamPoseMat(end,10);0;slamPoseMat(end,12)];
        dltAngSlam = rad2deg(norm(rodrigues(dltrSlam)));
        errSlam = dltrSlam*prvPtSlam + dlttSlam - curPtSlam;
        if norm(rodrigues(roty(dltAngSlam)) - rodrigues(dltrSlam)) > norm(rodrigues(roty(-dltAngSlam)) - rodrigues(dltrSlam))
            accumAngSlam = [accumAngSlam; accumAngSlam(end) + dltAngSlam];
        else
            accumAngSlam = [accumAngSlam; accumAngSlam(end) - dltAngSlam];
        end
        
    end

    
end
    
    B = fitplane(vslMat4(:,10:12)');
    B = B./norm(B(1:3));
    rMatNew = CorrectRot([0;1;0], B(1:3));
    
    vslMat = vslMat4(:,10:12)*rMatNew;
     C = fitplane(vslMat(:,1:3)');
     C = C./norm(C(1:3));
     vslMat(:,2) = vslMat(:,2) - mean(vslMat(:,2));
%      figure,plot(vslMat(:,2))
     
     figure,subplot(1,2,1);plot([accumAngSlam vslMat4(:,11) vslMat(:,2)]);grid on;legend('angle', 'y old', 'y new')
     subplot(1,2,2);plot(vslMat(:,1),vslMat(:,3));grid on;axis equal
%      figure,plot([accumAngSlam vslMat(:,2)]);legend('angle', 'y')
end
function rMatNew = CorrectRot(axis, gVec)
ang = CalcDegree(axis',gVec');
xVec = cross(axis,gVec);
xVec = xVec./norm(xVec);
zVec = cross(xVec, gVec);
zVec = zVec./norm(zVec);

rMatNew = [xVec'; gVec'; zVec']';
end

