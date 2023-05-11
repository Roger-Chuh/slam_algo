function [T,transformed2] = AlignCoordSys3(bodyList, vrList,figNum)
% % vrList(:,1) = -vrList(:,1);
matchThr = 20; 10; 20;5;10;5; 10; 20;10; 5; 3; 2; 3; 5;
distThr = 2;
% [~,dltRobo] = NormalizeVector(bodyList(2:end,1:3) - bodyList(1:end-1,1:3));
[~,dltVR] = NormalizeVector(vrList(1:end,1:3) - repmat(vrList(1,1:3),size(vrList,1),1));
% % [~,dltRobo] = NormalizeVector(bodyList(1:end,1:3) - repmat([0 0 0],size(bodyList,1),1));
[~,dltRobo] = NormalizeVector(bodyList(1:end,1:3) - repmat(bodyList(1,1:3),size(bodyList,1),1));
MinDistMat = [];

MinDistMatAll = [];
for i = 1 : length(dltRobo)-1
    [minDist,id] = min(abs(dltVR - dltRobo(i+1)));
    MinDistMatAll = [MinDistMatAll;[i+1 id minDist]];
    if minDist < matchThr;
        if ~isempty(MinDistMat)
            if ~isempty(find(MinDistMat(:,2) == id))
                idd = find(MinDistMat(:,2) == id);
                
                if MinDistMat(idd,3) < minDist
                    continue;
                else
                    MinDistMat(idd,:) = [i+1 id minDist];
                end
                
            else
                MinDistMat = [MinDistMat;[i+1 id minDist]];
            end
        else
            MinDistMat = [MinDistMat;[i+1 id minDist]];
        end
    end
end


try
% %     wakraj;
    matchPtBody1 = bodyList(MinDistMat(:,1),1:3);
    matchPtVR1 = vrList(MinDistMat(:,2),1:3);
    matchPtBody1 = [bodyList(1,1:3);matchPtBody1];
    matchPtVR1 = [vrList(1,1:3);matchPtVR1];
    if 0
        idx = find(diff(matchPtBody1(:,3)) > distThr);
    else
        [~,lenRobo] = NormalizeVector(matchPtBody1);
        idx = find(diff(lenRobo) > distThr);
    end
    matchPtBodyy = matchPtBody1(idx+1,:);
    matchPtVRR = matchPtVR1(idx+1,:);
    matchPtBodyy = [bodyList(1,1:3); matchPtBodyy];
    matchPtVRR = [vrList(1,1:3); matchPtVRR];
% %     massBody = mean(matchPtBodyy);
% %     massVR = mean(matchPtBodyy);
    N = 2;
    matchPtBody11 = matchPtBodyy(1:floor(size(matchPtBodyy,1)/N),:);
    matchPtBody22 = matchPtBodyy(size(matchPtBody11,1) + 1 : end,:);
    matchPtVR11 = matchPtVRR(1:floor(size(matchPtVRR,1)/N),:);
    matchPtVR22 = matchPtVRR(size(matchPtVR11,1) + 1 : end,:);
    
% %     matchPtBody = [(roty(30)*matchPtBody11')';matchPtBody22];
% %     matchPtVR = [(roty(30)*matchPtVR11')';matchPtVR22];
    matchPtBody = [(matchPtBody11 + repmat([0 10000000 0],size(matchPtBody11,1),1));matchPtBody22];
    matchPtVR = [(matchPtVR11 + repmat([0 10000000 0],size(matchPtVR11,1),1));matchPtVR22];
    
    qerge = 1;
    
    if 1
        [~,dltVR1] = NormalizeVector(matchPtVR(1:end,1:3) - repmat(vrList(1,1:3),size(matchPtBody,1),1));
        % %     [~,dltRobo1] = NormalizeVector(matchPtBody(1:end,1:3) - repmat([0 0 0],size(matchPtVR,1),1));
        [~,dltRobo1] = NormalizeVector(matchPtBody(1:end,1:3) - repmat(bodyList(1,1:3),size(matchPtVR,1),1));
        dlt = abs(dltVR1 - dltRobo1);
    end
    % try
    % %     kbj
    bodyCtrMass = mean(matchPtBody);
    vrCtrMass = mean(matchPtVR);
    bodyPt = matchPtBody - repmat(bodyCtrMass,size(matchPtBody,1),1);
    vrPt = matchPtVR - repmat(vrCtrMass,size(matchPtVR,1),1);
    [U,S,V] = svd(vrPt'*bodyPt);
    H = U*S*V';
    X = V*U';
    % %     [U,S,V] = svd(bodyPt'*vrPt);
    % %     H = U*S*V';
    % %     X = U'*V;
    if abs(det(X) - (-1)) < 0.0001
        %     if X(2,2) < 0
        V(:,3) = -V(:,3);
        R = V*U';
    else
        R = X;
        
    end
    % %     R(3,3) = -R(3,3);
    t = bodyCtrMass' - R*vrCtrMass';
    if t(1) < 0
        qerqvgj = 1;
    end
    
    rVec = rodrigues(R);
    
    % %     t = mean(matchPtBody - (R*matchPtVR')')';
    T = [R t;0 0 0 1];
    transformed = T(1:3,1:3)*matchPtVR' + repmat(T(1:3,end),1,size(matchPtVR,1));
    rotated = T(1:3,1:3)*matchPtVR';
    [~, err] = NormalizeVector(transformed' - matchPtBody);
    % %     figure(figNum+1),clf;plot(matchPtBody(:,1),matchPtBody(:,3),'.b');hold on;plot(transformed(1,:),transformed(3,:),'.r');axis equal;drawnow;
    % %     figure(figNum+1),clf;plot(matchPtBody(:,1),matchPtBody(:,3),'.b');hold on;plot(transformed(1,:),transformed(3,:),'.r');axis equal;drawnow;
    transformed2 = T(1:3,1:3)*vrList(:,1:3)' + repmat(T(1:3,end),1,size(vrList,1));
    % %     figure(figNum+1),clf;plot(bodyList(:,1),bodyList(:,3),'.b');hold on;plot(transformed2(1,:),transformed2(3,:),'.r');axis equal;drawnow;
    
    
    % %     fig_3D3_pair(matchPtBody(:,1:3)',transformed);
    % %     fig_3D3_pair(matchPtBody(:,1:3)',rotated);
    if abs(rVec(2)) < 0.00001
        T = [];
        transformed2 = [];
    end
        
    
    
    eqrgqerv = 1;
    
    if 0
        intrMat = [500 0 320;0 500 240;0 0 1];
        pixBody = pflat(intrMat*matchPtBody')';
        %     [extrVec, outlierIdx] = posest([matchPtBody(:,1)./matchPtBody(:,3) ones(size(matchPtBody,1),1)./matchPtBody(:,3)],[matchPtVR(:,1) ones(size(matchPtVR(:,1),1),1) matchPtVR(:,3)],0.75,eye(3), 'repr_err');
        % %     [extrVec, outlierIdx] = posest(pixBody(:,1:2),matchPtVR,0.75,intrMat, 'repr_err');
        % % % % % % %     tform = pcregrigid(matchPtBody,matchPtVR,'Extrapolate',true);
        % %     [R, t]=ICPMatching(matchPtBody(:,[1 3])', matchPtVR(:,[1 3])');
        
        tform = pcregrigid(pointCloud(matchPtBody),pointCloud(matchPtVR),'Extrapolate',true,'InlierRatio',0.8);
        %         tform = pcregrigid(pointCloud(bodyList(:,1:3)),pointCloud(vrList(:,1:3)),'Extrapolate',true,'InlierRatio',0.8);
        T = inv(tform.T');
        transformed = T(1:3,1:3)*matchPtVR' + repmat(T(1:3,end),1,size(matchPtVR,1));
        [~, err] = NormalizeVector(transformed' - matchPtBody);
        transformed2 = T(1:3,1:3)*vrList(:,1:3)' + repmat(T(1:3,end),1,size(vrList,1));
        erviu = 1;
    end
    fjgo = 1;
    % %     p1 = [matchPtBody(:,1)./matchPtBody(:,3) matchPtBody(:,2)./matchPtBody(:,3) ones(size(matchPtBody,1),1)];
    % %     p2 = [matchPtVR(:,1)./matchPtVR(:,3) matchPtVR(:,2)./matchPtVR(:,3) ones(size(matchPtVR,1),1)];
    % %     Evec = calibrated_fivepoint(p2,p1);
    % %     [ EEE ] = calcBestE( Evec, p1, p2);
    % %             [U,S,V] = svd(EEE);
    % %         if det(U*V') < 0
    % %             V = -V;
    % %         end
    % %         W = [0 1 0; -1 0 0 ; 0 0 1];
    % %         t = U(:,3);
    % %         R = U*W*V';
    % %         P2 = [];
    % %         P{1} = [eye(3) zeros(3,1)];
    % %         P{2} = [R t];
    
    % % %     T = [rodrigues(extrVec(1:3)) extrVec(4:end);0 0 0 1];
    
catch
    T = []; transformed2 = [];
end

if 0
    vrLst = vrList(find(dltVR > 2)+1,1:3);
    bodyLst = bodyList(bodyList(:,3) > 2,1:3);
    
    try
        origVR = vrLst(1,:)';
    catch
        wfkh = 1;
    end
    
    origBody = [0;0;0];
    t = origVR;
    
    if 0
        [Cvr, errVr] = fitline(vrLst(:,[1 3])');
        [Cbody, errRobo] = fitline(bodyLst(:,[1 3])');
        dirVR1 = [1;0;-Cvr(1)/Cvr(2)];
        dirBody1 = [1;0;-Cbody(1)/Cbody(2)];
        
    else
        
        [Vvr, Lvr, inliersvr] = ransacfitline(vrLst', distThr);
        [Vbody, Lbody, inliersbody] = ransacfitline(bodyLst', distThr);
        dirBody1 = Vbody(:,1) - Vbody(:,2);
        dirVR1 = Vvr(:,1) - Vvr(:,2);
    end
    
    dirBody1 = dirBody1./norm(dirBody1);
    dirVR1 = dirVR1./norm(dirVR1);
    
    if 0
        figure(65),clf;subplot(1,2,1);plot(vrLst(:,1),vrLst(:,3),'.');hold on;plot(Vvr(1,:),Vvr(3,:),'r');axis equal
        subplot(1,2,2);plot(bodyLst(:,1),bodyLst(:,3),'.');hold on;plot(Vbody(1,:),Vbody(3,:),'r');axis equal
        drawnow;
        
    end
    
    
    
    
    dirBody2 = [bodyLst(end,:) - bodyLst(1,:)]'; dirBody2 = dirBody2./norm(dirBody2);
    dirVR2 = [vrLst(end,:) - vrLst(1,:)]'; dirVR2 = dirVR2./norm(dirVR2);
    
    if norm(dirBody2-dirBody1) < 0.8
        dirBody = dirBody1;
    else
        dirBody = -dirBody1;
    end
    
    if norm(dirVR2-dirVR1) < 0.8
        dirVR = dirVR1;
    else
        dirVR = -dirVR1;
    end
    % % thetaBody = CalcDegree(dirBody,[1;0;0]);
    % % thetaVR = CalcDegree(dirVR,[1;0;0]);
    
    theta = dot(dirBody,dirVR)./norm(dirBody)./norm(dirVR);
    
    if 0
        R1 = roty(acosd(theta));
        R2 = roty(-acosd(theta));
        dist1 = norm(curVR - (R1*curBody + t));
        dist2 = norm(curVR - (R2*curBody + t));
        if dist1 < dist2
            T = inv([R1 t;0 0 0 1]);
        else
            T = inv([R2 t;0 0 0 1]);
        end
    else
        if dirVR(1) > dirBody(1)
            R = roty(acosd(theta));
        else
            R = roty(-acosd(theta));
        end
        T = inv([R t;0 0 0 1]);
        dist = norm(vrLst(end,1:3)' - (R*bodyLst(end,1:3)' + t));
    end
end

end