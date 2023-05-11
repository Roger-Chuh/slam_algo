function [v,div,SI_, SIp,err,pixNew, goodness] = epipoleLK(gx,gy,SI,Ipi,cIx,d,i,wintx,boundary,winty,nx,ny,l,epiline,dltx)
marg = 0.2;
step = 0.01;

optVec = dltx-marg:step:dltx+marg;
useOpt = 0;

if useOpt
    options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','off','MaxFunEvals',10000,'TolX',1e-8);%,'MaxFunEvals',10000);%, 'MaxIterations',5);
    %     options = optimoptions('lsqnonlin','Algorithm','trust-region-reflective','Display','off','MaxFunEvals',10000,'TolX',1e-8);%,'MaxFunEvals',10000);%, 'MaxIterations',5);     trust-region-reflective
    lb2 = [optVec(1)];
    ub2 = [optVec(end)];
    [vec,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(X) epipoleLK_fcn(gx,gy,SI,Ipi,cIx,d,i,wintx,boundary,winty,nx,ny,l,epiline, X),[dltx],lb2,ub2,options);%,data_1,obs_1)
end

if useOpt
    [v,div,SI_, SIp,pixNew] = epipoleLK_fcn2(gx,gy,SI,Ipi,cIx,d,i,wintx,boundary,winty,nx,ny,l,epiline,vec);
    err = ones(length(optVec),1);
else
    cnt = 1;
    
    for k = optVec
        [err(cnt,1), si_,sip] = epipoleLK_fcn3(gx,gy,SI,Ipi,cIx,d,i,wintx,boundary,winty,nx,ny,l,epiline,k);
        
        compare{cnt,1} = [si_' sip'];
        cnt = cnt + 1;
    end
    [~,id] = min(err);
    
    [minv,mini11]=findpeaks(-err,'MinPeakProminence',1000);
    % [minv,mini11]=findpeaks(-err,'minpeakdistance',7,'Threshold',995);
    
    
    [~,id_] = min(err(mini11));
    idd = mini11(id_);
    if isempty(idd)
        goodness = 0;
        idd = id;
    else
        goodness = 1;
    end
    
    if 0 %goodness == 0
        figure(15),clf;plot(optVec,err);hold on;plot(optVec(mini11),err(mini11),'*g');plot(dltx,10000,'*r');
        sadvbhj = 1;
    end
    if 1
        [v,div,SI_, SIp,pixNew] = epipoleLK_fcn2(gx,gy,SI,Ipi,cIx,d,i,wintx,boundary,winty,nx,ny,l,epiline,optVec(idd));
    else
        [v,div,SI_, SIp,pixNew] = epipoleLK_fcn2(gx,gy,SI,Ipi,cIx,d,i,wintx,boundary,winty,nx,ny,l,epiline,optVec(id));
    end
end




end

function errMat = epipoleLK_fcn(gx,gy,SI,Ipi,cIx,d,i,wintx,boundary,winty,nx,ny,l,epiline,dltx)
errMat = [];
cIy = (-epiline(2)*cIx-epiline(3))/epiline(1);
cIxNew = cIx + d(1,i) + dltx;
cIyNew = (-epiline(2)*cIxNew-epiline(3))/epiline(1);
v = [dltx; cIyNew - cIy - d(2,i)];
cIpx = cIx + d(1,i) + v(1); % + extra_m;	%
cIpy = cIy + d(2,i) + v(2); % Coords. of the point
crIpx = round(cIpx); 	% on the final image
crIpy = round(cIpy); 	%

if 1 %(crIpx>=2+wintx+boundary) && (crIpx<=nx-wintx-1-boundary) && (crIpy>=2+winty+boundary) && (crIpy<=ny-winty-1-boundary),
    
    itIpx = cIpx - crIpx;
    itIpy = cIpy - crIpy;
    
    if itIpx > 0,
        vIpx = [itIpx 1-itIpx 0]';
    else
        vIpx = [0 1+itIpx -itIpx]';
    end;
    if itIpy > 0,
        vIpy = [itIpy 1-itIpy 0];
    else
        vIpy = [0 1+itIpy -itIpy];
    end;
    
    vec1  =crIpx-wintx-1:crIpx+wintx+1;
    vec2 = crIpy-winty-1:crIpy+winty+1;
    flagIn1 = vec1>=1 & vec1 <=nx;
    flagIn2 = vec2>=1 & vec2 <=ny;
    
    SIp = Ipi(vec1(flagIn1),vec2(flagIn2),l+1);
    SIp = conv2(conv2(SIp,vIpx,'same'),vIpy,'same');
    %     SIp = SIp(2:2*wintx+2,2:2*winty+2);
    
    SI_ = SI(flagIn1,flagIn2);
    
    if sum(flagIn1) ~= size(SI,1) || sum(flagIn2) ~= size(SI,2)
        askbj = 1;
    end
    
    gt = SI_ - SIp;
    t1 = sum(sum(gt .* gx(flagIn1,flagIn2)));
    t2 = sum(sum(gt .* gy(flagIn1,flagIn2)));
    
    errMat = [errMat; [abs(sum(sum(gt .* gx(flagIn1,flagIn2))))+ abs(sum(sum(gt .* gy(flagIn1,flagIn2))))]];
    %     errMat = [errMat; sum(sum(abs(gt)))];
    %     errMat = [errMat; var(gt(:))];
    
    if 0
        figure,imshow([SI SIp], [])
    end
    
    %
    %     B = [t1;t2];
    %     G = [sum(sum(gx.^2)),sum(sum(gx.*gy));...
    %         sum(sum(gx.*gy)),sum(sum(gy.^2))];
    %     tt = G(1,1)+G(2,2);
    %     dd = G(1,1)*G(2,2)-G(1,2)*G(2,1);
    %     e1 = tt/2 + sqrt((tt^2)/4-dd);
    %     e2 = tt/2 - sqrt((tt^2)/4-dd);
    %
    %     flow = G\B;
    %
    %     v_extra = [(c*t1 - b*t2)/dt;(a*t2 - b*t1)/dt];
    %     compt = compt + 1;
    %     v(:,i) = v(:,i) + v_extra;
else
    div = 1; 		% Feature xtt out of the image
    errMat = 0;
end;

end
function [v,div,SI_, SIp, pixNew] = epipoleLK_fcn2(gx,gy,SI,Ipi,cIx,d,i,wintx,boundary,winty,nx,ny,l,epiline,dltx)
errMat = [];
cIy = (-epiline(2)*cIx-epiline(3))/epiline(1);
cIxNew = cIx + d(1,i) + dltx;
cIyNew = (-epiline(2)*cIxNew-epiline(3))/epiline(1);
v = [dltx; cIyNew - cIy - d(2,i)];
cIpx = cIx + d(1,i) + v(1); % + extra_m;	%
cIpy = cIy + d(2,i) + v(2); % Coords. of the point
crIpx = round(cIpx); 	% on the final image
crIpy = round(cIpy); 	%

pixNew = [cIxNew; cIyNew];


if 1 %(crIpx>=2+wintx+boundary) && (crIpx<=nx-wintx-1-boundary) &&  (crIpy>=2+winty+boundary) && (crIpy<=ny-winty-1-boundary),
    
    itIpx = cIpx - crIpx;
    itIpy = cIpy - crIpy;
    
    if itIpx > 0,
        vIpx = [itIpx 1-itIpx 0]';
    else
        vIpx = [0 1+itIpx -itIpx]';
    end;
    if itIpy > 0,
        vIpy = [itIpy 1-itIpy 0];
    else
        vIpy = [0 1+itIpy -itIpy];
    end;
    
    if 0
        SIp = Ipi(crIpx-wintx-1:crIpx+wintx+1,crIpy-winty-1:crIpy+winty+1,l+1);
        SIp = conv2(conv2(SIp,vIpx,'same'),vIpy,'same');
        %     SIp = SIp(2:2*wintx+2,2:2*winty+2);
        
        gt = SI - SIp;
        t1 = sum(sum(gt .* gx));
        t2 = sum(sum(gt .* gy));
    else
        vec1  =crIpx-wintx-1:crIpx+wintx+1;
        vec2 = crIpy-winty-1:crIpy+winty+1;
        flagIn1 = vec1>=1 & vec1 <=nx;
        flagIn2 = vec2>=1 & vec2 <=ny;
        
        SIp = Ipi(vec1(flagIn1),vec2(flagIn2),l+1);
        SIp = conv2(conv2(SIp,vIpx,'same'),vIpy,'same');
        %     SIp = SIp(2:2*wintx+2,2:2*winty+2);
        
        SI_ = SI(flagIn1,flagIn2);
        
        gt = SI_ - SIp;
        t1 = sum(sum(gt .* gx(flagIn1,flagIn2)));
        t2 = sum(sum(gt .* gy(flagIn1,flagIn2)));
    end
    
    
    
    
    
    
    errMat = [errMat; [abs(sum(sum(gt .* gx(flagIn1,flagIn2))))+ abs(sum(sum(gt .* gy(flagIn1,flagIn2))))]];
    
    if 0
        figure,imshow([SI SIp], [])
    end
    div = 0;
    %
    %     B = [t1;t2];
    %     G = [sum(sum(gx.^2)),sum(sum(gx.*gy));...
    %         sum(sum(gx.*gy)),sum(sum(gy.^2))];
    %     tt = G(1,1)+G(2,2);
    %     dd = G(1,1)*G(2,2)-G(1,2)*G(2,1);
    %     e1 = tt/2 + sqrt((tt^2)/4-dd);
    %     e2 = tt/2 - sqrt((tt^2)/4-dd);
    %
    %     flow = G\B;
    %
    %     v_extra = [(c*t1 - b*t2)/dt;(a*t2 - b*t1)/dt];
    %     compt = compt + 1;
    %     v(:,i) = v(:,i) + v_extra;
else
    div = 1; 		% Feature xtt out of the image
    SIp = SI;
end;

end
function [errMat, SI_, SIp] = epipoleLK_fcn3(gx,gy,SI,Ipi,cIx,d,i,wintx,boundary,winty,nx,ny,l,epiline,dltx)
errMat = [];
cIy = (-epiline(2)*cIx-epiline(3))/epiline(1);
cIxNew = cIx + d(1,i) + dltx;
cIyNew = (-epiline(2)*cIxNew-epiline(3))/epiline(1);
v = [dltx; cIyNew - cIy - d(2,i)];
cIpx = cIx + d(1,i) + v(1); % + extra_m;	%
cIpy = cIy + d(2,i) + v(2); % Coords. of the point
crIpx = round(cIpx); 	% on the final image
crIpy = round(cIpy); 	%

if 1 %(crIpx>=2+wintx+boundary) && (crIpx<=nx-wintx-1-boundary) && (crIpy>=2+winty+boundary) && (crIpy<=ny-winty-1-boundary),
    
    itIpx = cIpx - crIpx;
    itIpy = cIpy - crIpy;
    
    if itIpx > 0,
        vIpx = [itIpx 1-itIpx 0]';
    else
        vIpx = [0 1+itIpx -itIpx]';
    end;
    if itIpy > 0,
        vIpy = [itIpy 1-itIpy 0];
    else
        vIpy = [0 1+itIpy -itIpy];
    end;
    
    vec1  =crIpx-wintx-1:crIpx+wintx+1;
    vec2 = crIpy-winty-1:crIpy+winty+1;
    flagIn1 = vec1>=1 & vec1 <=nx;
    flagIn2 = vec2>=1 & vec2 <=ny;
    
    SIp = Ipi(vec1(flagIn1),vec2(flagIn2),l+1);
    SIp = conv2(conv2(SIp,vIpx,'same'),vIpy,'same');
    %     SIp = SIp(2:2*wintx+2,2:2*winty+2);
    
    SI_ = SI(flagIn1,flagIn2);
    
    if sum(flagIn1) ~= size(SI,1) || sum(flagIn2) ~= size(SI,2)
        askbj = 1;
    end
    
    gt = SI_ - SIp;
    t1 = sum(sum(gt .* gx(flagIn1,flagIn2)));
    t2 = sum(sum(gt .* gy(flagIn1,flagIn2)));
    
        errMat = [errMat; [abs(sum(sum(gt .* gx(flagIn1,flagIn2))))+ abs(sum(sum(gt .* gy(flagIn1,flagIn2))))]];
%     errMat = [errMat; sum(sum(abs(gt)))];
    %     errMat = [errMat; var(gt(:))];
    
    if 0
        figure,imshow([SI SIp], [])
    end
    
    %
    %     B = [t1;t2];
    %     G = [sum(sum(gx.^2)),sum(sum(gx.*gy));...
    %         sum(sum(gx.*gy)),sum(sum(gy.^2))];
    %     tt = G(1,1)+G(2,2);
    %     dd = G(1,1)*G(2,2)-G(1,2)*G(2,1);
    %     e1 = tt/2 + sqrt((tt^2)/4-dd);
    %     e2 = tt/2 - sqrt((tt^2)/4-dd);
    %
    %     flow = G\B;
    %
    %     v_extra = [(c*t1 - b*t2)/dt;(a*t2 - b*t1)/dt];
    %     compt = compt + 1;
    %     v(:,i) = v(:,i) + v_extra;
else
    div = 1; 		% Feature xtt out of the image
    errMat = 0;
end;

end