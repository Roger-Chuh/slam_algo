function [v,div,SI_, SIp, pixNew] = epipoleLK_fcn4(gx,gy,SI,Ipi,cIx,d,i,wintx,boundary,winty,nx,ny,l,epiline,dltx)
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