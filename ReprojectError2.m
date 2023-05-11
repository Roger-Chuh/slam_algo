function [reprjErrL, reprjErrR, eplL, eplR, fundMat] = ReprojectError2(ptL, ptR, intrMatL, intrMatR, rotVec, transDir, funcVars)

ptL = [ptL, ones(size(ptL, 1), 1)];
ptR = [ptR, ones(size(ptR, 1), 1)];

% transDir = [angleWithZ, angleWithX]
transVec = [sin(transDir(1))*cos(transDir(2)), sin(transDir(1))*sin(transDir(2)), cos(transDir(1))];
rotMat = rodrigues(rotVec);
tMat = SkewSymMat(transVec);
fundMat = inv(intrMatR)'*tMat*rotMat*inv(intrMatL);
fundMat = fundMat./norm(fundMat);
eplL = ptR*fundMat;
eplR = ptL*fundMat';



if 0
    err = dist2LinePix(ptR(:,1:2)',eplR(1,:));
    err = dist2LinePix2(ptR(1,1:2),eplR(1,:))
end

numer = dot(eplL, ptL, 2); % distance from left pt to its corresponding left epipole line
denomL2 = eplL(:,1).^2 + eplL(:,2).^2;
denomL = sqrt(denomL2);
denomR2 = eplR(:,1).^2 + eplR(:,2).^2;
denomR = sqrt(denomR2);
reprjErrL = numer ./ denomL; % normalized diatance from pt to its epipole (unit: pixel)
reprjErrR = numer ./ denomR; % this constrains the left and right epipole line to be parallel





% % % L = F * x';
% % % Lp = F' * xp';
% % % num = sum(xp' .* L).^2;
% % % den = L(1,:).^2 + L(2,:).^2 + Lp(1,:).^2 + Lp(2,:).^2;
% % % err = num ./ den;




whvj = 1;







% numerL = dot(eplL, ptL, 2); % distance from left pt to its corresponding left epipole line
% numerR = dot(eplR, ptR, 2);
% denomL2 = eplL(:,1).^2 + eplL(:,2).^2;
% denomL = sqrt(denomL2);
% denomR2 = eplR(:,1).^2 + eplR(:,2).^2;
% denomR = sqrt(denomR2);
% reprjErrL = numerL ./ denomL; % normalized diatance from pt to its epipole (unit: pixel)
% reprjErrR = numerR ./ denomR; % this constrains the left and right epipole line to be parallel

if 1
if (nargout > 20)
    if any(strcmp({'intrinsic', 'all'}, funcVars))
        [dfmdfl, dfmdcxl, dfmdcyl, dfmdfr, dfmdcxr, dfmdcyr] = DfmDintr(intrMatL, intrMatR, rotMat, tMat);
        [dsdenldfl, dsdenldcxl, dsdenldcyl, dsdenldfr, dsdenldcxr, dsdenldcyr, ...
          dsdenrdfl, dsdenrdcxl, dsdenrdcyl, dsdenrdfr, dsdenrdcxr, dsdenrdcyr] = ...
          DsdenDintr(eplL, eplR, ptL, ptR, dfmdfl, dfmdcxl, dfmdcyl, dfmdfr, dfmdcxr, dfmdcyr);
      
        dnumdfl = dot(ptR*dfmdfl, ptL, 2);
        dnumdcxl = dot(ptR*dfmdcxl, ptL, 2);
        dnumdcyl = dot(ptR*dfmdcyl, ptL, 2);
        dnumdfr = dot(ptR*dfmdfr, ptL, 2);
        dnumdcxr = dot(ptR*dfmdcxr, ptL, 2);
        dnumdcyr = dot(ptR*dfmdcyr, ptL, 2);
        
        deldfl = (2*denomL2 .* dnumdfl - numer .* dsdenldfl)./(2*denomL.^3);
        deldcxl = (2*denomL2 .* dnumdcxl - numer .* dsdenldcxl)./(2*denomL.^3);
        deldcyl = (2*denomL2 .* dnumdcyl - numer .* dsdenldcyl)./(2*denomL.^3);
        deldfr = (2*denomL2 .* dnumdfr - numer .* dsdenldfr)./(2*denomL.^3);
        deldcxr = (2*denomL2 .* dnumdcxr - numer .* dsdenldcxr)./(2*denomL.^3);
        deldcyr = (2*denomL2 .* dnumdcyr - numer .* dsdenldcyr)./(2*denomL.^3);
        
        derdfl = (2*denomR2 .* dnumdfl - numer .* dsdenrdfl)./(2*denomR.^3);
        derdcxl = (2*denomR2 .* dnumdcxl - numer .* dsdenrdcxl)./(2*denomR.^3);
        derdcyl = (2*denomR2 .* dnumdcyl - numer .* dsdenrdcyl)./(2*denomR.^3);
        derdfr = (2*denomR2 .* dnumdfr - numer .* dsdenrdfr)./(2*denomR.^3);
        derdcxr = (2*denomR2 .* dnumdcxr - numer .* dsdenrdcxr)./(2*denomR.^3);
        derdcyr = (2*denomR2 .* dnumdcyr - numer .* dsdenrdcyr)./(2*denomL.^3);
    end
    if any(strcmp({'extrinsic', 'all'}, funcVars))
        [dfmdrv1, dfmdrv2, dfmdrv3] = DfmDrv(intrMatL, intrMatR, rotVec, tMat);
        [dfmdtaz, dfmdtax] = DfmDtd(intrMatL, intrMatR, rotMat, transDir);
        [dsdenldrv1, dsdenldrv2, dsdenldrv3, dsdenldtaz, dsdenldtax, ...
          dsdenrdrv1, dsdenrdrv2, dsdenrdrv3, dsdenrdtaz, dsdenrdtax] = ...
          DsdenDextr(eplL, eplR, ptL, ptR, dfmdrv1, dfmdrv2, dfmdrv3, dfmdtaz, dfmdtax);
      
         dnumdrv1 = dot(ptR*dfmdrv1, ptL, 2);
         dnumdrv2 = dot(ptR*dfmdrv2, ptL, 2);
         dnumdrv3 = dot(ptR*dfmdrv3, ptL, 2);
         dnumdtaz = dot(ptR*dfmdtaz, ptL, 2);
         dnumdtax = dot(ptR*dfmdtax, ptL, 2);
         
         deldrv1 = (2*denomL2 .* dnumdrv1 - numer .* dsdenldrv1)./(2*denomL.^3);
         deldrv2 = (2*denomL2 .* dnumdrv2 - numer .* dsdenldrv2)./(2*denomL.^3);
         deldrv3 = (2*denomL2 .* dnumdrv3 - numer .* dsdenldrv3)./(2*denomL.^3);
         deldtaz = (2*denomL2 .* dnumdtaz - numer .* dsdenldtaz)./(2*denomL.^3);
         deldtax = (2*denomL2 .* dnumdtax - numer .* dsdenldtax)./(2*denomL.^3);
         
         derdrv1 = (2*denomR2 .* dnumdrv1 - numer .* dsdenrdrv1)./(2*denomR.^3);
         derdrv2 = (2*denomR2 .* dnumdrv2 - numer .* dsdenrdrv2)./(2*denomR.^3);
         derdrv3 = (2*denomR2 .* dnumdrv3 - numer .* dsdenrdrv3)./(2*denomR.^3);
         derdtaz = (2*denomR2 .* dnumdtaz - numer .* dsdenrdtaz)./(2*denomR.^3);
         derdtax = (2*denomR2 .* dnumdtax - numer .* dsdenrdtax)./(2*denomR.^3);
    end
    
    switch funcVars
        case 'intrinsic'
            jaccL = [deldfl, deldcxl, deldcyl, deldfr, deldcxr, deldcyr];
            jaccR = [derdfl, derdcxl, derdcyl, derdfr, derdcxr, derdcyr];
        case 'extrinsic'
            jaccL = [deldrv1, deldrv2, deldrv3, deldtaz, deldtax];
            jaccR = [derdrv1, derdrv2, derdrv3, derdtaz, derdtax];
        case 'all'
            jaccL = [deldfl, deldcxl, deldcyl, deldfr, deldcxr, deldcyr, deldrv1, deldrv2, deldrv3, deldtaz, deldtax];
            jaccR = [derdfl, derdcxl, derdcyl, derdfr, derdcxr, derdcyr, derdrv1, derdrv2, derdrv3, derdtaz, derdtax];
        otherwise
            error('Unrecognized name of error function variable set %s', funcVars);
    end
        
end
end


end

function [dfmdfl, dfmdcxl, dfmdcyl, dfmdfr, dfmdcxr, dfmdcyr] = DfmDintr(intrMatL, intrMatR, rotMat, tMat)

invIntrMatL = inv(intrMatL);
dimdfl = -invIntrMatL*[1,0,0; 0,1,0; 0,0,0]*invIntrMatL;
dimdcxl = -invIntrMatL*[0,0,1; 0,0,0; 0,0,0]*invIntrMatL;
dimdcyl = -invIntrMatL*[0,0,0; 0,0,1; 0,0,0]*invIntrMatL;
invIntrMatR = inv(intrMatR);
dimdfr = -invIntrMatR*[1,0,0; 0,1,0; 0,0,0]*invIntrMatR;
dimdcxr = -invIntrMatR*[0,0,1; 0,0,0; 0,0,0]*invIntrMatR;
dimdcyr = -invIntrMatR*[0,0,0; 0,0,1; 0,0,0]*invIntrMatR;


dfmdfl = invIntrMatR' * tMat * rotMat * dimdfl;
dfmdcxl = invIntrMatR' * tMat * rotMat * dimdcxl;
dfmdcyl = invIntrMatR' * tMat * rotMat * dimdcyl;

dfmdfr = dimdfr' * tMat * rotMat * invIntrMatL;
dfmdcxr = dimdcxr' * tMat * rotMat * invIntrMatL;
dfmdcyr = dimdcyr' * tMat * rotMat * invIntrMatL;

end

function [dfmdrv1, dfmdrv2, dfmdrv3] = DfmDrv(intrMatL, intrMatR, rotVec, tMat)

om = SkewSymMat(rotVec);
theta = norm(rotVec);
if (theta > 1e-7)
    dthdrv = rotVec/theta;
else
    dthdrv = [0;0;0];
end
domd1 = SkewSymMat([1,0,0]); % [0, 0,0; 0,0,-1;  0,1,0];
domd2 = SkewSymMat([0,1,0]); % [0, 0,1; 0,0, 0; -1,0,0];
domd3 = SkewSymMat([0,0,1]); % [0,-1,0; 1,0, 0;  0,0,0];
om2 = om*om;
dom2d1 = domd1*om + om*domd1;
dom2d2 = domd2*om + om*domd2;
dom2d3 = domd3*om + om*domd3;
drmd1 = domd1*sin(theta) + om*cos(theta)*dthdrv(1) + dom2d1*(1-cos(theta)) + om2*sin(theta)*dthdrv(1);
drmd2 = domd2*sin(theta) + om*cos(theta)*dthdrv(2) + dom2d2*(1-cos(theta)) + om2*sin(theta)*dthdrv(2);
drmd3 = domd3*sin(theta) + om*cos(theta)*dthdrv(3) + dom2d3*(1-cos(theta)) + om2*sin(theta)*dthdrv(3);

invIntrMatL = inv(intrMatL);
invIntrMatR = inv(intrMatR);
dfmdrv1 = invIntrMatR' * tMat * drmd1 * invIntrMatL;
dfmdrv2 = invIntrMatR' * tMat * drmd2 * invIntrMatL;
dfmdrv3 = invIntrMatR' * tMat * drmd3 * invIntrMatL;

end

function [dfmdtaz, dfmdtax] = DfmDtd(intrMatL, intrMatR, rotMat, transDir)

angZ = transDir(1);
angX = transDir(2);

% v = [sinAcosB, sinAsinB, cosA]
cosAz = cos(angZ);
sinAz = sin(angZ);
cosAx = cos(angX);
sinAx = sin(angX);

dtmdtaz = SkewSymMat([ cosAz*cosAx, cosAz*sinAx, -sinAz]);
dtmdtax = SkewSymMat([-sinAz*sinAx, sinAz*cosAx,  0]);

invIntrMatL = inv(intrMatL);
invIntrMatR = inv(intrMatR);
dfmdtaz = invIntrMatR' * dtmdtaz * rotMat * invIntrMatL;
dfmdtax = invIntrMatR' * dtmdtax * rotMat * invIntrMatL;

end

function [dsdenldfl, dsdenldcxl, dsdenldcyl, dsdenldfr, dsdenldcxr, dsdenldcyr, ...
          dsdenrdfl, dsdenrdcxl, dsdenrdcyl, dsdenrdfr, dsdenrdcxr, dsdenrdcyr] = ...
    DsdenDintr(eplL, eplR, ptL, ptR, dfmdfl, dfmdcxl, dfmdcyl, dfmdfr, dfmdcxr, dfmdcyr)

deplldfl = ptR * dfmdfl;
deplldcxl = ptR * dfmdcxl;
deplldcyl = ptR * dfmdcyl;
deplldfr = ptR * dfmdfr;
deplldcxr = ptR * dfmdcxr;
deplldcyr = ptR * dfmdcyr;

dsdenldfl = 2*dot(deplldfl(:,1:2), eplL(:,1:2), 2);
dsdenldcxl = 2*dot(deplldcxl(:,1:2), eplL(:,1:2), 2);
dsdenldcyl = 2*dot(deplldcyl(:,1:2), eplL(:,1:2), 2);
dsdenldfr = 2*dot(deplldfr(:,1:2), eplL(:,1:2), 2);
dsdenldcxr = 2*dot(deplldcxr(:,1:2), eplL(:,1:2), 2);
dsdenldcyr = 2*dot(deplldcyr(:,1:2), eplL(:,1:2), 2);


deplrdfl = ptL * dfmdfl';
deplrdcxl = ptL * dfmdcxl';
deplrdcyl = ptL * dfmdcyl';
deplrdfr = ptL * dfmdfr';
deplrdcxr = ptL * dfmdcxr';
deplrdcyr = ptL * dfmdcyr';

dsdenrdfl = 2*dot(deplrdfl(:,1:2), eplR(:,1:2), 2);
dsdenrdcxl = 2*dot(deplrdcxl(:,1:2), eplR(:,1:2), 2);
dsdenrdcyl = 2*dot(deplrdcyl(:,1:2), eplR(:,1:2), 2);
dsdenrdfr = 2*dot(deplrdfr(:,1:2), eplR(:,1:2), 2);
dsdenrdcxr = 2*dot(deplrdcxr(:,1:2), eplR(:,1:2), 2);
dsdenrdcyr = 2*dot(deplrdcyr(:,1:2), eplR(:,1:2), 2);

end

function [dsdenldrv1, dsdenldrv2, dsdenldrv3, dsdenldtaz, dsdenldtax, ...
          dsdenrdrv1, dsdenrdrv2, dsdenrdrv3, dsdenrdtaz, dsdenrdtax] = ...
    DsdenDextr(eplL, eplR, ptL, ptR, dfmdrv1, dfmdrv2, dfmdrv3, dfmdtaz, dfmdtax)

deplldrv1 = ptR * dfmdrv1;
deplldrv2 = ptR * dfmdrv2;
deplldrv3 = ptR * dfmdrv3;
deplldtaz = ptR * dfmdtaz;
deplldtax = ptR * dfmdtax;

dsdenldrv1 = 2*dot(deplldrv1(:,1:2), eplL(:,1:2), 2);
dsdenldrv2 = 2*dot(deplldrv2(:,1:2), eplL(:,1:2), 2);
dsdenldrv3 = 2*dot(deplldrv3(:,1:2), eplL(:,1:2), 2);
dsdenldtaz = 2*dot(deplldtaz(:,1:2), eplL(:,1:2), 2);
dsdenldtax = 2*dot(deplldtax(:,1:2), eplL(:,1:2), 2);

deplrdrv1 = ptL * dfmdrv1';
deplrdrv2 = ptL * dfmdrv2';
deplrdrv3 = ptL * dfmdrv3';
deplrdtaz = ptL * dfmdtaz';
deplrdtax = ptL * dfmdtax';

dsdenrdrv1 = 2*dot(deplrdrv1(:,1:2), eplR(:,1:2), 2);
dsdenrdrv2 = 2*dot(deplrdrv2(:,1:2), eplR(:,1:2), 2);
dsdenrdrv3 = 2*dot(deplrdrv3(:,1:2), eplR(:,1:2), 2);
dsdenrdtaz = 2*dot(deplrdtaz(:,1:2), eplR(:,1:2), 2);
dsdenrdtax = 2*dot(deplrdtax(:,1:2), eplR(:,1:2), 2);

end
