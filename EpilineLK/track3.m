%function [xtt,goodfeat,Qtt] = track(Ii,Ipi,xt,goodfeat)
%image1 and image2 are two input images
%xt: input position a 2xN matrix
%goodfeat: Nx1 vector indicateing whether a point is lost if not
%present on input, it is assumed to be all 1
%xtt: new position
%Qtt: quality
%
%support routine for 'trackdemo.m' (help trackdemo)
%uses basic tracker based on translational image motion model
%
%
%Contributors to this code include: Pietro Perona, Stefano Soatto, Andrea Mennucci,
%Jean-Yves Bouguet, Xiaolin Feng, Hailin Jin, Paolo Favaro, Jana Kosecka, Yi Ma.
%Last updated 5/5/2003.
%
%DISTRIBUTED FREE FOR NON-COMMERCIAL USE
%Copyright (c) MASKS, 2003

function [xtt,goodfeat,Qtt,infoMat,Var] = track3(Ii,Ipi,xt,goodfeat,FundVec)
fMat = reshape(FundVec,3,3);
goodfeat0 = goodfeat;
global resolution saturation ...
    wintx winty boundary boundary_t ...
    thresh levelmin levelmax ThreshQ method varThr deltVThr;

if nargin<=3, error('Wrong input'); end;
if size(xt,1)~=2, error('Wrong input for xt'); end;
if nargin==3, goodfeat=ones(size(xt,2),1); end;

[nrow,ncol] = size(Ipi(:,:,1));
SampleSize(:,1) = [nrow;ncol];

% Downsampling images
for ii = 1:levelmax,
    [tmpimage,SampleSize(:,ii+1)] ...
        = downsample_(Ii(:,:,ii),SampleSize(:,ii));
    Ii(1:SampleSize(1,ii+1),1:SampleSize(2,ii+1),ii+1)=tmpimage;
    Ipi(1:SampleSize(1,ii+1),1:SampleSize(2,ii+1),ii+1) ...
        = downsample_(Ipi(:,:,ii),SampleSize(:,ii));
end;


%%

new = 0;
useFundMat = 1;

Ind = []; YY = [];
for jj = 1 : size(xt,2)
    if xt(1,jj) ~= 0
        
        epiLines3 = epipolarLine(fMat, xt([2 1],jj)');
        epiLines3 = epiLines3./norm(epiLines3(1:2));
        
        y = (-epiLines3(2)*xt(1,jj)-epiLines3(3))/epiLines3(1);
        YY = [YY;y];
        Ind = [Ind;jj];
    end
end
%     xt(2,Ind) = YY;


%%

N = length(find(goodfeat));
NMAX=length(goodfeat);
xyNew = zeros(2,NMAX);
Goodness = zeros(1,NMAX);
wekeep = zeros(NMAX,1);		% Initialization
xuu = zeros(2,NMAX);		% of the sub-pyramid
Quu = zeros(NMAX,1);		%
sub_level = levelmin;		%
maxQ = 0;
Ge = zeros(1,NMAX);
goodfeatsave = goodfeat; 	% save of all good features
er = [];
badCnt = [];
comp = zeros(NMAX,20);
% SaliencyMat = zeros(NMAX,1*(levelmax+1));
SaliencyMat = nan(NMAX,2);
Var = 1000.*ones(NMAX,3);
iscomp = 0;
infoMat = [];
while (sub_level < levelmax + 1) & (N > 0), % The loop on the levels...
    
    d = zeros(2,NMAX); 	        % initial displacement
    div_list = zeros(NMAX,1); 	% initialy, no feat diverges
    Residuals = zeros(NMAX,1); 	% Resilduals
    ratios = zeros(NMAX,1); 	% Resilduals
    ge = inf;
    
    
    for l = sub_level:-1:0,
        nx = SampleSize(1,l+1);
        ny = SampleSize(2,l+1);
        v = zeros(2,NMAX);
        d = 2*d; 				% scaling of the displacement
        xc  = (xt-1)/(2^l)+1;
        
        comp = zeros(NMAX,1);
        %         SaliencyMat = nan(NMAX,1*(levelmax+1));
        badCnt = [];
        for i = 1 : NMAX,
            
            
            if i == 1556
                asfghkj = 1;
            end
            if goodfeat(i) && (~div_list(i)),  % if it is a good feature...
                %             if (~div_list(i)),  % if it is a good feature...
                epiLines2 = epipolarLine(fMat, xt([2 1],i)');
                
                % %                 y1 = 20;y2 = 20;
                % %                 x1 = (-epiLines2(2)*y1-epiLines2(3))/epiLines2(1);
                % %
                epiLines2 = epiLines2./norm(epiLines2(1:2));
                if epiLines2(2) < 0
                    epiLines2 = -epiLines2;
                end
                points2 = lineToBorderPoints(epiLines2, size(Ipi(:,:,1)));
                points2_level = (points2-1)/(2^l)+1;
                points2_level = reshape(points2_level,2,2)'; % [x1 y1; x2 y2]
                epiDir = points2_level(1,:) - points2_level(2,:);
                epiDir = epiDir./norm(epiDir);
                
                [~, ~, line_level] = svd([points2_level [1 1]'],0);
                line_level = line_level(:,3)./norm(line_level(1:2,3));
                if line_level(2) < 0
                    line_level = -line_level;
                end
                
                aaa = line_level-epiLines2';
                % % % %                 line_level(3) = epiLines2(3)/2^l;
                
                
                cIx = xc(1,i); 	                %
                cIy = xc(2,i); 			% Coords. of the point
                crIx = round(cIx); 		% on the initial image
                crIy = round(cIy); 		%
                if l ~= 0 %& i == 1
                    dbjk = 1;
                end
                if ((crIx>=3+wintx+boundary_t) & (crIx<=nx-wintx-2-boundary_t) &  (crIy>=3+winty+boundary_t) & (crIy<=ny-winty-2-boundary_t))   % | abs(ge) < 50
                    
                    itIx = cIx - crIx;
                    itIy = cIy - crIy;
                    if itIx > 0,
                        vIx = [itIx 1-itIx 0]';
                    else
                        vIx = [0 1+itIx -itIx]';
                    end;
                    if itIy > 0,
                        vIy = [itIy 1-itIy 0];
                    else
                        vIy = [0 1+itIy -itIy];
                    end;
                    
                    SI = Ii(crIx-wintx-2:crIx+wintx+2,crIy-winty-2:crIy+winty+2,l+1);
                    SI = conv2(conv2(SI,vIx,'same'),vIy,'same');
                    SI = SI(2:2*wintx+4,2:2*winty+4);
                    [gy,gx] = gradient(SI);
                    gx_ = gx;
                    gy_ = gy;
                    gx = gx(2:2*wintx+2,2:2*winty+2);
                    gy = gy(2:2*wintx+2,2:2*winty+2);
                    
                    % %                     gy = line_level(2)*gy;
                    % %                     gx = -line_level(1)*gx;
                    
                    ge = sign(line_level(2))*line_level(2)^2*gy - sign(line_level(1))*line_level(1)^2*gx;
                    dirVec = [-line_level(2) line_level(1)];
                    
                    % %                     theta = deg2rad(90);
                    % %                     dirVec = [cos(theta) sin(theta)];
                    ge_ = dot(repmat(dirVec',1, length(gx(:))),[gy(:) gx(:)]');
                    ge__ = (reshape(ge_,size(gx)));
                    ge = sum(dot(repmat(dirVec',1, length(gx(:))),[gy(:) gx(:)]'));
                    
                    
                    %                     if 0 % abs(ge) < 20 | abs(ge) > 800
                    %                         figure(14),clf;imshow(imresize(SI,10),[]);hold on;plot(10*dirVec(1),10*dirVec(2),'.g');title(num2str(ge));figure(15);clf;imshow(Ii(:,:,l+1),[]);hold on;plot(cIy,cIx,'.r');figure(16),imshow(ge__,[]);title(num2str(ge__((length(ge__(:))-1)/2+1)))
                    %                     end
                    
                    a = sum(sum(gx .* gx));
                    b = sum(sum(gx .* gy));
                    c = sum(sum(gy .* gy));
                    SI_ = SI;
                    SI = SI(2:2*wintx+2,2:2*winty+2);
                    dt = a*c - b^2;
                    
                    v(:,i) = [0;0]; 	% initial flow
                    div = 0; 		% still on the screen
                    compt = 0; 		% no iteration yet
                    v_extra = resolution + 1; % just larger than resolution
                    if 0
                        if l~=0, compt_max = 15; else compt_max = 1; end;
                    else
                        compt_max = 50;
                    end
                    errMat = []; errMat2 = []; deltV = [];
                    if l ~= 0 %& i == 1
                        dbjk = 1;
                    end
                    while (norm(v_extra) > resolution) && (~div) && (compt<compt_max),
                        
                        cIpx = cIx + d(1,i) + v(1,i); % + extra_m;	%
                        cIy_ = xOnLine(line_level, cIx);
                        
                        
                        cIpy0 = cIy + d(2,i) + v(2,i); % Coords. of the point
                        
                        
                        if useFundMat
                            cIpy = cIy_ + d(2,i) + v(2,i); % Coords. of the point
                        else
                            cIpy = cIpy0;
                        end
                        er = [er;cIpy0-cIpy];
                        %                         cIpx = cIpx - 0.9;
                        if 0
                            cIpy = xOnLine(line_level, cIpx);
                        end
                        crIpx = round(cIpx); 	% on the final image
                        crIpy = round(cIpy); 	%
                        
                        if (crIpx>=2+wintx+boundary) && (crIpx<=nx-wintx-1-boundary) && ...
                                (crIpy>=2+winty+boundary) && (crIpy<=ny-winty-1-boundary),
                            
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
                            
                            SIp = Ipi(crIpx-wintx-1:crIpx+wintx+1,crIpy-winty-1:crIpy+winty+1,l+1);
                            SIp = conv2(conv2(SIp,vIpx,'same'),vIpy,'same');
                            SIp = SIp(2:2*wintx+2,2:2*winty+2);
                            
                            gt = SI - SIp;
                            % %                             t1 = sum(sum(gt .* gx));
                            t1 = sum(sum(gt .* gx));
                            t2 = sum(sum(gt .* gy));
                            
                            errMat = [errMat; [abs(sum(sum(gt .* gx))) abs(sum(sum(gt .* gy)))]];
                            errMat2 = [errMat2; sum(sum(abs(gt)))];
                            
                            
                            
                            
                            B = [t1;t2];
                            G = [sum(sum(gx.^2)),sum(sum(gx.*gy));...
                                sum(sum(gx.*gy)),sum(sum(gy.^2))];
                            tt = G(1,1)+G(2,2);
                            dd = G(1,1)*G(2,2)-G(1,2)*G(2,1);
                            e1 = tt/2 + sqrt((tt^2)/4-dd);
                            e2 = tt/2 - sqrt((tt^2)/4-dd);
                            
                            flow = G\B;
                            
                            vv = sum(sum(gt))*(line_level(2)/line_level(2)-line_level(1));
                            lambda = sum(sum(gt))/ge;
                            
                            vvv = [lambda*dirVec([2 1])]';
                            %                             vvv(2) = vvv(2) + (cIy_-cIy);
                            comp(i) = (cIy_-cIy);
                            if abs(comp(i)) > 20
                                badCnt = [badCnt ; i];
                            end
                            if max(abs(vvv)) > 10
                                dadfkvj = 1;
                            end
                            
                            
                            v_extra = [(c*t1 - b*t2)/dt;(a*t2 - b*t1)/dt];
                            
                            if useFundMat
                                v_extra =  vvv;
                            end
                            
                            deltV = [deltV; norm(v_extra)];
                            
                            [v_2,div_2,SI_2, SIp_2, pixNew_2] = epipoleLK_fcn4(gx_,gy_,SI_,Ipi,cIx,d,i,wintx,boundary,winty,nx,ny,l,line_level,v_extra(1));
                            
                            if 0
                                figure,imshow([SI_  1*ones(size(SI_,1),1) SIp_2], [])
                            end
                            compt = compt + 1;
                            v(:,i) = v(:,i) + v_extra;
                        else
                            div = 1; 		% Feature xtt out of the image
                        end;
                    end;
                    scale = 9; segNum = 5; upper = (segNum-1)/2; lower = upper;
                    segPix = 20;
                    ge___ = imresize(ge__,scale);
                    pixx = ((size(ge___,2)-1)/2+1);
                    pixy = ((size(ge___,1)-1)/2+1);
                    
                    
                    
                    
                    upper = [pixy-segPix:-segPix:1]; upper = upper(1:(segNum-1)/2);
                    lower = [pixy+segPix:segPix:size(ge___,1)]; lower = lower(1:(segNum-1)/2);
                    YList = [upper(end:-1:1) pixy lower]';
                    
                    
                    bTemp = YList - dirVec(2)/dirVec(1)*pixx;
                    lineTemp = [repmat([dirVec(2)/dirVec(1) -1],length(YList),1) bTemp];
                    yList = repmat([1:size(ge___,1)],segNum,1);
                    verti = 1;
                    xList = round(xOnLine(lineTemp', yList));
                    inflag = xList>=1 & xList <=size(ge___,2) & yList>=1 & yList <=size(ge___,1);
                    xyIn = {};ind = {};
                    valid = sum(inflag');
                    if new
                        for kk = 1 : segNum
                            xyIn{kk,1} = [xList(kk,inflag(kk,:)); yList(kk,inflag(kk,:))]';
                            ind{kk,1} = sub2ind(size(ge___),xyIn{kk}(:,2),xyIn{kk}(:,1));
                            
                        end
                        
                        if min(valid) < size(yList,2)/2
                            verti = 0;
                            %                     if sum(inflag) < length(yList)/2
                            %                         ind = sub2ind(size(ge___),repmat(pixy,size(ge___,2),1),yList');
                            lefter = [pixx-segPix:-segPix:1]; lefter = lefter(1:(segNum-1)/2);
                            righer = [pixy+segPix:segPix:size(ge___,1)]; righer = righer(1:(segNum-1)/2);
                            XList = [lefter(end:-1:1) pixx righer]';
                            
                            bTemp = pixy - dirVec(2)./dirVec(1).*XList;
                            lineTemp = [repmat([dirVec(2)/dirVec(1) -1],length(XList),1) bTemp];
                            xList = repmat([1:size(ge___,2)],segNum,1);
                            
                            yList = round(yOnLine(lineTemp', xList));
                            inflag = xList>=1 & xList <=size(ge___,2) & yList>=1 & yList <=size(ge___,1);
                            %                         xyIn = [xList(inflag); yList(inflag)]';
                            %                         ind = sub2ind(size(ge___),xyIn(:,2),xyIn(:,1));
                            xyIn = {};ind = {};
                            valid = sum(inflag');
                            for kk = 1 : segNum
                                xyIn{kk,1} = [xList(kk,inflag(kk,:)); yList(kk,inflag(kk,:))]';
                                ind{kk,1} = sub2ind(size(ge___),xyIn{kk}(:,2),xyIn{kk}(:,1));
                                
                            end
                        else
                            sadbab = 1;
                            %                         ylab = 1;
                        end
                        saliencyMat = [];
                        for kkk = 1:length(ind);
                            curve{kkk,1} = ge___(ind{kkk});
                            [minv,mini11]=findpeaks(-curve{kkk,1},'minpeakdistance',1);
                            [maxv,max11]=findpeaks(curve{kkk,1},'minpeakdistance',1);
                            if isempty(minv) && ~isempty(maxv)
                                maxv = min(-minv)+30;
                            end
                            if isempty(maxv) && ~isempty(minv)
                                minv = -(max(maxv)-30);
                            end
                            % % %                         if isempty(maxv) && isempty(minv)
                            % % %                             minv = 30;
                            % % %                             maxv = 30;
                            % % %                         end
                            if 0
                                figure,plot(curve{kkk});hold on;plot(mini11,-minv,'*g');plot(max11,maxv,'*r');
                            end
                            lengthCurve = length(curve{kkk,1});
                            curveInd = round(round(lengthCurve)/2-round(lengthCurve)/15:round(lengthCurve)/2+round(lengthCurve)/15);
                            try
                                % %                                 saliencyMat = [saliencyMat;[min(-minv) max(maxv)]];
                                saliencyMat = [saliencyMat;[max(curve{kkk,1}(curveInd))- min(curve{kkk,1}(curveInd)) mean([max(curve{kkk,1}(curveInd)) min(curve{kkk,1}(curveInd))])]];
                            catch
                                sfnwejk = 1;
                            end
                            
                        end
                        if i == 2000
                            asdgkb = 1;
                        end
                        try
                            %                             SaliencyMat(i,l+1) = abs([mean(saliencyMat(:,2))-mean(saliencyMat(:,1))]);
                            %                             SaliencyMat(i,l+1) = abs([mean(saliencyMat(:,1))]);
                            SaliencyMat(i,:) = abs([mean(saliencyMat(:,1))]);
                        catch
                            asgdi = 1;
                        end
                    end
                    avbjkh = 1;
                    if 0 % norm(vvv)>4 % abs(ge) < 20 | abs(ge) > 800
                        %                         figure(14),clf;imshow(imresize(SI,10),[]);hold on;plot([10*dirVec(1) 0],[10*dirVec(2) 0],'-g');plot(10+[10*dirVec(1) 0],10+[10*dirVec(2) 0],'-r');title(num2str(ge));figure(15);clf;imshow(Ii(:,:,l+1),[]);hold on;plot(cIy,cIx,'*r');title(num2str(round(vvv',3)));figure(16),clf;imshow(imresize(ge__,scale),[]);hold on;plot(10+[scale*dirVec(1) 0],10+[scale*dirVec(2) 0],'-r');quiver(pixx,pixy,dirVec(1),dirVec(2),100);quiver(pixx,pixy,-dirVec(1),-dirVec(2),100);plot(xList,yList,'or');plot(xyIn(:,1),xyIn(:,2),'xb');title(num2str(ge__((length(ge__(:))-1)/2+1)));figure(17),clf,plot(xyIn(:,2),curve);xlabel('y');ylabel('ge');
                        figure(14),clf;imshow(imresize(SI,10),[]);hold on;plot([10*dirVec(1) 0],[10*dirVec(2) 0],'-g');plot(10+[10*dirVec(1) 0],10+[10*dirVec(2) 0],'-r');title(num2str(ge));figure(15);clf;imshow(Ii(:,:,l+1),[]);hold on;plot(cIy,cIx,'*r');title(num2str(round(vvv',3)));
                        figure(16),clf;imshow(imresize(ge__,scale),[]);hold on;plot(10+[scale*dirVec(1) 0],10+[scale*dirVec(2) 0],'-r');quiver(pixx,pixy,dirVec(1),dirVec(2),100);quiver(pixx,pixy,-dirVec(1),-dirVec(2),100);title(num2str(ge__((length(ge__(:))-1)/2+1)));
                        
                        figure(17),clf,hold on;
                        for kkkk = 1:length(ind)
                            if verti == 1
                                figure(17),plot(xyIn{kkkk}(:,2),curve{kkkk}');xlabel('y');ylabel('ge');
                            else
                                figure(17),plot(xyIn{kkkk}(:,1),curve{kkkk}');xlabel('x');ylabel('ge');
                            end
                            if kkkk == 1
                                figure(16),plot(xList(kkkk,:),yList(kkkk,:),'or');plot(xyIn{kkkk}(:,1),xyIn{kkkk}(:,2),'xy');
                            else
                                figure(16),plot(xList(kkkk,:),yList(kkkk,:),'or');plot(xyIn{kkkk}(:,1),xyIn{kkkk}(:,2),'xb');
                            end
                        end
                    end
                    %                     if max(deltV)>10 && sqrt(var(deltV)) < varThr && div ==0
                    %                         weghjk = 1;
                    %                     end
                    try
                        Var(i,:) = [sqrt(var(deltV)) mean(deltV) deltV(end)-0.5*deltV(1)];
                    catch
                        aekdjb = 1;
                    end
                    skjns = 1;
                    
                    if 0 %useFundMat
                        [v_tmp, divv, SI2, SIp2, err2,pixNew,goodness] = epipoleLK(gx_,gy_,SI_,Ipi,cIx,d,i,wintx,boundary,winty,nx,ny,l,line_level,v(1,i));
                        %                         [v_tmp, divv, SI2, SIp2, err2,pixNew,goodness] = epipoleLK(gx_,gy_,SI_,Ipi,cIx,d,i,wintx,boundary,winty,nx,ny,l,line_level,vvv(1));
                        % %                     figure,imshow([SIp 1*ones(size(SI,1),1) SI 1*ones(size(SI,1),1) SIp2], [])
                        
                        if l~=0
                            try
                                figure(12),clf;subplot(2,1,1);imshow([SI_(2:end-1,2:end-1)  1*ones(size(SI_,1)-2,1) SIp_2(2:end-1,2:end-1)], []);title(strcat('lk==',num2str(sum(sum(abs(SI_(2:end-1,2:end-1)-SIp_2(2:end-1,2:end-1)))))));subplot(2,1,2),imshow( [SI2(2:end-1,2:end-1) 1*ones(size(SI2,1)-2,1) SIp2(2:end-1,2:end-1)],[]);title(strcat('fundMat==',num2str(sum(sum(abs(SI2(2:end-1,2:end-1)-SIp2(2:end-1,2:end-1)))))));
                            catch
                                asdvkbj = 1;
                            end
                            svjh = 1;
                        end
                        if max(v_tmp) > 20
                            
                            %                         figure,ShowEpipoleLine3(fMat,Ii(:,:,1),Ipi(:,:,1),xt([2 1],i)',[cIpy cIpx]);
                            try
                                %                             figure,subplot(3,1,1);imshow([SI 1*ones(size(SI,1),1) SIp],[]);title('lk');subplot(3,1,2),imshow( [SI2 1*ones(size(SI2,1),1) SIp2],[]);title('fundMat');subplot(3,1,3);plot(err2);
                            catch
                                agnk = 1;
                            end
                            
                            asbkj = 1;
                        end
                        
                        if divv ~=1
                            if max(v_tmp) < 2000000
                                v(:,i) = v_tmp;
                            end
                        end
                    end
                    if size(errMat,1) > 2
                        asvkj = 1;
                    end
                    % At that point, if we are at the very last stage, we can
                    % compute the resildual of the feature
                    
                    if (l == 0),
                        
                        if useFundMat
                            try
                                gt_ = SI_ - SIp_2;
                                %                                 errMat3 = abs(sum(sum(gt_ .* gx_))) + abs(sum(sum(gt_ .* gy_)));
                                %                                 if errMat3 > 40000
                                %                                     askhj = 1;
                                %                                 end
                                SI_1 = SI_ - mean(SI_(:));
                                SIp_21 = SIp_2 - mean(SIp_2(:));
                                try
                                    gt1 = SI_1-SIp_21;
                                catch
                                    gt1 = 50000;
                                end
                            catch
                                errMat3 = 50000;
                                gt1 = 50000;
                            end
                            errMat3 = abs(sum(sum(gt1 .* gx_))) + abs(sum(sum(gt1 .* gy_)));
                        else
                            %                             errMat3 = abs(sum(sum(gt .* gx_))) + abs(sum(sum(gt .* gy_)));
                            SI1 = SI - mean(SI(:));
                            SIp1 = SIp - mean(SIp(:));
                            gt1 = SI1 - SIp1;
                            errMat3 = abs(sum(sum(gt1 .* gx))) + abs(sum(sum(gt1 .* gy)));
                        end
                        
                        Residuals(i) = sqrt(sum(gt1(:).^2)/((2*wintx+1)*(2*winty+1)-1));
                        try
                            ratios(i) = sum(gt1(:).^2)/sum(SI1(:).^2);
                        catch
                            try
                                ratios(i) = sum(gt1(:).^2)/sum(SI_1(:).^2);
                            catch
                                ratios(i) = 100;
                            end
                        end
%                         if norm(vvv) > 3
%                             xcviu = 1;
%                         end
                        try
                        %                         infoMat = [infoMat; [[norm(vvv) errMat3 l var([gx(:);gy(:)])] Residuals(i) ratios(i) xt(2,i) xt(1,i)]];
                        infoMat(i,:) =  [[norm(vvv) errMat3 l var([gx(:);gy(:)])] Residuals(i) ratios(i) xt(2,i) xt(1,i)];
                        %                         infoMat{i} = [];
                        catch
                            kjhk = 1;
                        end
                        if size(infoMat,1) == 76
                            asgadvk = 1;
                        end
                        % %                         if max(deltV)>10 && sqrt(var(deltV)) < varThr && div ==0 &&Residuals(i) < 20 && deltV(end) >= 0.5*deltV(1)
                        % %                             weghjk = 1;
                        % %                         end
                        try
                            if max(abs(vvv)) > 2 || norm(vvv) > 2
                                if Residuals(i) > 180000 || ratios(i) > 1110.9 || errMat3 > 50000
                                    ge = 0;
                                    badCnt = [badCnt;i];
                                    if 0
                                        figure,imshow([SI_  1*ones(size(SI_,1),1) SIp_2], [])
                                    end
                                    akb = 1;
                                else
                                    sdfkj = 1;
                                end
                            end
                        catch
                            kug=1;
                        end
                        Ge(i) = sqrt(ge.^2/((2*wintx+1)*(2*winty+1)-1));
                    end;
                    
                    if i == 288
                        asvkj = 1;
                    end
                    
                    try
                        if length(deltV) > 1
                            %                                                 if deltV(end) > 0.01
                            if max(diff(deltV)) > 0
                                %                         if 0 %length(deltV) == compt_max
                                %                         if deltV(1) > 3
                                ge = 0;
                                Ge(i) = sqrt(ge.^2/((2*wintx+1)*(2*winty+1)-1));
                                svbkj = 1;
                                badCnt = [badCnt;i];
                            else
                                asdkbvj = 1;
                            end
                        end
                    catch
                        gu=1;
                    end
                    if compt_max ~=1
                        if 0 %length(deltV) == compt_max
                            badCnt = [badCnt;i];
                        end
                    end
                else
                    div = 1; 		% Feature xt out of the image
                end;
                
                div_list(i) = div_list(i) | div; % Feature out of the image
            end;
            try
                xyNew(:,i) = pixNew_2; pixNew;
                Goodness(:,i) = goodness;
                
            catch
                askbj = 1;
            end
            % % %             Ge(:,i) = ge;
        end;
        d = d + v; 			% propagation of the displacement
        if useFundMat
            if iscomp == 0
                d(2,:) = d(2,:) + comp';
                iscomp = 1;
            else
                savbjk = 1;
            end
        end
    end;
    % % % % % %
    % % % % % %     Ind = []; YY = [];
    % % % % % %     for jj = 1 : size(xt,2)
    % % % % % %         if xt(1,jj) ~= 0
    % % % % % %
    % % % % % %             epiLines3 = epipolarLine(fMat, xt([2 1],jj)');
    % % % % % %             epiLines3 = epiLines3./norm(epiLines3(1:2));
    % % % % % %
    % % % % % %             y = (-epiLines3(2)*xt(1,jj)-epiLines3(3))/epiLines3(1);
    % % % % % %             YY = [YY;y];
    % % % % % %             Ind = [Ind;jj];
    % % % % % %         end
    % % % % % %     end
    % % % % % %     xt(2,Ind) = YY;
    % % % % % %
    if 0 %useFundMat
        xtt = xyNew;
        % induced feature points
    else
        xtt = xt + d;
    end
    %     xtt = [xt(1,:); + d;
    %xtt(:,1) = xtt(:,1); 		% + extra_m;
    % of I(t+1)
    
    %computes the quality vector Qtt from xtt and Ipi
    Qtt = ComputeQuality(Ipi,xtt,goodfeat,wintx,winty);
    % used to test lost tracks!
    
    % on the image I(t+1)
    if 0,
        TF = (Qtt > ThreshQ * Qt) & (~div_list);
        Qttc = Qtt(find(TF));
        Q_max = [Qttc;maxQ];
        if ~method,
            maxQ = max(Q_max);
        else
            maxQ = max(Q_max(find(Q_max<saturation*min(wintx,winty))));
        end;
        TF = TF & (Qtt > thresh*maxQ);
        wekeep = wekeep | TF;
    else % no quality rejection, keep all the features
        %         TF = (~div_list) & (Qtt > 0) & (Residuals < 18) & (ratios < 2);
        %         TF = (~div_list) & (Qtt > 0) & (Residuals < 100) & (ratios < 5);
        badCnt = unique(badCnt);
        badCnt2 = []; find(SaliencyMat(:,1)<10);
        %         TF = (~div_list) & (Qtt > 0) & (((Residuals < 18) & (ratios < 5) )| (abs(Ge') > 12));
        TF = (~div_list) & (Qtt > 0) & (((Residuals < 18) & (ratios < 0.9) )| (abs(Ge') > 0.5));
        if 0 % sub_level == levelmax
            TF(unique([badCnt;badCnt2])) = 0;
        else
            TF(Var(:,1) > varThr | Var(:,2) > deltVThr | Var(:,3) >= 0) = 0;
        end
        Qttc = Qtt(find(TF));
        Q_max = [Qttc;maxQ];
        if ~method,
            maxQ = max(Q_max);
        else
            maxQ = max(Q_max(find(Q_max<saturation*min(wintx,winty))));
        end;
        TF = TF & (Qtt > thresh*maxQ);
        wekeep = wekeep | TF;
    end;
    
    
    % We keep the good features
    xuu = xuu + [TF,TF]' .* xtt; % (save them in xuu)
    % We save the qualities
    Quu = Quu + TF .* Qtt; 		% as well (in Quu)
    Nkept = size(find(TF),1);
    
    goodfeat = goodfeatsave & (~wekeep); % For the remaining features,
    N = size(find(goodfeat),1); 	% try the following level.
    
    if levelmax ~= levelmin,
        fprintf(1, 'At level %d, we keep %d features\n',sub_level,Nkept);
    end;
    sub_level = sub_level + 1; 		% Try the folowing depth
    
end; 					% end of the sub_level loop

%copy into output
xtt = xuu;
Qtt = Quu;
goodfeat = wekeep;
% goodfeat = goodfeat0 & logical(Goodness');
end
% figure,ShowEpipoleLine3(fMat,Ii(:,:,1),Ipi(:,:,1),xt([2 1],goodfeat)',xtt([2 1],goodfeat)');

function cIy = xOnLine(epiline, cIx)
% epiline = epiline';
for i = 1 : size(epiline,2)
    cIy(i,:) = (-epiline(2,i).*cIx(i,:)-epiline(3,i))./epiline(1,i);
end
end
function cIy = yOnLine(epiline, cIx)
for i = 1 : size(epiline,2)
    cIy(i,:) = (-epiline(1,i).*cIx(i,:)-epiline(3,i))./epiline(2,i);
end
end
