function T = AlignCoordSys(curBody, curVR, origVR)

origBody = [0;0;0];
t = origVR;
dirBody = curBody - origBody;
dirVR = curVR - origVR;
dirBody = dirBody./norm(dirBody);
dirVR = dirVR./norm(dirVR);
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
    dist = norm(curVR - (R*curBody + t));
end



end