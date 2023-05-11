function [camPoseMatNew,accumAng1] = reBase_(camPoseMat2,dltT0)
accumAng1 = 0;
for h = 1 : size(camPoseMat2,1)
    
    
    if h == 194
        asfkj = 1;
    end
    Told = [reshape(camPoseMat2(h,1:9),3,3) camPoseMat2(h,10:12)';0 0 0 1];
    %    dltT = [rt;0 0 0 1];
    Tnew = dltT0*Told;
%     Tnew(1:3,1:3) = [Tnew(1:3,[3 1 2])];
    Tnew(1:3,1:3) = [Tnew([3 1 2],1:3)];
    Tnew(1:3,1:3) = [-Tnew(1:3,[3]) -Tnew(1:3,[1]) Tnew(1:3,[2])];
    Tnew(1:3,4) = [Tnew([3 1 2],4)];
    camPoseMatNew(h,:) = [reshape(Tnew(1:3,1:3),1,9) Tnew(1:3,end)'];
    if size(camPoseMatNew,1) > 1
        dltT =  Tnew*inv([reshape(camPoseMatNew(end-1,1:9),3,3) camPoseMatNew(end-1,10:12)';0 0 0 1]);
        dltr = dltT(1:3,1:3);
        dltt = dltT(1:3,end);
        prvPt = [camPoseMatNew(end-1,10);0;camPoseMatNew(end-1,12)];
        curPt = [camPoseMatNew(end,10);0;camPoseMatNew(end,12)];
        prvPt = [camPoseMatNew(end-1,10);0;camPoseMatNew(end-1,12)];
        curPt = [camPoseMatNew(end,10);0;camPoseMatNew(end,12)];
        errSlam = dltr*prvPt + dltt - curPt;
        try
            dltAng = rad2deg(norm(rodrigues(dltr)));
        catch
            ewrgl = 1;
        end
        try
            if norm(rodrigues(rotz(dltAng)) - rodrigues(dltr)) < norm(rodrigues(rotz(-dltAng)) - rodrigues(dltr))
                accumAng1 = [accumAng1; accumAng1(end) + dltAng];
            else
                accumAng1 = [accumAng1; accumAng1(end) - dltAng];
            end
        catch
            weflkh = 1;
        end
        if length(accumAng1) >= 2676
            asdfgj = 1;
        end
    end
    
end









% % slamPoseMat2 = []; %poseFidd = [];
% % rt1675 = [reshape(slamPoseMat(baseNum,1:9),3,3) slamPoseMat(baseNum,10:12)';0 0 0 1];
% % for ii = baseNum:size(slamPoseMat,1)
% %     tmpRT =  [reshape(slamPoseMat(ii,1:9),3,3) slamPoseMat(ii,10:12)';0 0 0 1];
% %     tmpRt2 = [R [0;0;0];0 0 0 1]*inv(rt1675)*tmpRT;
% %     slamPoseMat2 = [slamPoseMat2;[reshape(tmpRt2(1:3,1:3),1,9) tmpRt2(1:3,4)']];
% %     %        poseFidd = [poseFidd;[tmpRt2(1:3,4)' tmpRt2(3,1:3)]];
% % end
end