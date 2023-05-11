function [T,transformed] = AlignCoordSys2(bodyList, vrList,figNum)

distThr = 2; 5;
[~,dltRobo] = NormalizeVector(bodyList(2:end,1:3) - bodyList(1:end-1,1:3));
[~,dltVR] = NormalizeVector(vrList(2:end,1:3) - vrList(1:end-1,1:3));
dltRobo = [0;dltRobo];
vrLst = vrList(find(dltVR > 2)+1,1:3);
% % bodyLst = bodyList(bodyList(:,3) > 2,1:3);
bodyLst = bodyList(dltRobo >= 0,1:3);

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

if 1
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

transformed = T(1:3,1:3)*vrList(:,1:3)' + repmat(T(1:3,end),1,size(vrList,1));
% % transformed2 = T(1:3,1:3)*vrList(:,1:3)' + repmat(T(1:3,end),1,size(vrList,1));

% % figure(figNum+1),clf,plot(bodyList(:,1),bodyList(:,3),'.b');hold on;plot(transformed(1,:),transformed(3,:),'.r');axis equal;drawnow;
end