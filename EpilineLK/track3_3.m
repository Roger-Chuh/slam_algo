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

function [xtt,goodfeat,Qtt,infoMat,Varr,respp] = track3_3(Ii,Ipi,xt,goodfeat,FundVec)
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
compThr = 3;
% gaussianFilter = fspecial('gaussian',2,2);


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
error_ = [];
contourVar = 0*ones(NMAX,1);
Responses = []; Responsess = [];
Var = 1000.*ones(NMAX,3);
iscomp = 0;
infoMat = [];
Idnan = [];
done = 0;
while (sub_level < levelmax + 1) & (N > 0), % The loop on the levels...
    
    d = zeros(2,NMAX); 	        % initial displacement
    div_list = zeros(NMAX,1); 	% initialy, no feat diverges
    div_listt = zeros(NMAX,1);
    Residuals = zeros(NMAX,1); 	% Resilduals
    ratios = zeros(NMAX,1); 	% Resilduals
    ratioss= zeros(NMAX,1);
    Residualss = zeros(NMAX,1);
    
    ge = inf;
    
    Var1 = 1000.*ones(NMAX,sub_level+1);
    Var2 = 1000.*ones(NMAX,sub_level+1);
    Var222 = 1000.*ones(NMAX,sub_level+1);
    Var3 = 1000.*ones(NMAX,sub_level+1);
    Var333 = 1000.*ones(NMAX,sub_level+1);
    response = 0.*ones(NMAX,sub_level+1);
    responsee = 0.*ones(NMAX,sub_level+1);
    
    iscomp = 0;
    badCnt33 = [];
    badCnt44 = [];
    
    for l = sub_level:-1:0,
        nx = SampleSize(1,l+1);
        ny = SampleSize(2,l+1);
        v = zeros(2,NMAX);
        d = 2*d; 				% scaling of the displacement
        xc  = (xt-1)/(2^l)+1;
        
        comp = zeros(NMAX,1);
        %         SaliencyMat = nan(NMAX,1*(levelmax+1));
        badCnt = [];
        badCnt3 = [];
        badCnt4 = [];
        
        %
        %         badCnt33 = [];
        %         badCnt44 = [];
        
        EpiLines2 = epipolarLine(fMat, xt([2 1],:)');
        [~,scale] = NormalizeVector(EpiLines2(:,1:2));
        EpiLines2 = EpiLines2./repmat(scale,1,3);
        EpiLines2(EpiLines2(:,2) < 0,:) = -EpiLines2(EpiLines2(:,2) < 0,:);
        Points2 = lineToBorderPoints(EpiLines2, size(Ipi(:,:,1)));
        Points2_level = (Points2-1)/(2^l)+1;
        Line_level = [(Points2_level(:,4)-Points2_level(:,2))./(Points2_level(:,3)-Points2_level(:,1)) repmat(-1,size(Points2_level,1),1) (Points2_level(:,2).*Points2_level(:,3)-Points2_level(:,1).*Points2_level(:,4))./(Points2_level(:,3)-Points2_level(:,1))];
        [~,scale_level] = NormalizeVector(Line_level(:,1:2));
        Line_level = Line_level./repmat(scale_level,1,3);
        Line_level(Line_level(:,2) < 0,:) = -Line_level(Line_level(:,2) < 0,:);
        Line_level = Line_level';
        idnan = find(isnan(Line_level(1,:)) | isnan(Line_level(2,:)) | isnan(Line_level(3,:)));
        goodfeat(idnan) = 0;
        Idnan = [Idnan; idnan];
        
        
        CIx = xc(1,:); 	                %
        CIy = xc(2,:); 			% Coords. of the point
        CrIx = round(CIx); 		% on the initial image
        CrIy = round(CIy);
        ItIx = (CIx - CrIx);
        ItIy = (CIy - CrIy);
        %         IIi = repmat(Ii(:,:,l+1),1,1,NMAX);
        rowOfst = [(-wintx-2): (wintx+2)];
        colOfst = [(-winty-2) : (+winty+2)];
        [xu, yu] = meshgrid(colOfst, rowOfst);
        xyOfst = [xu(:) yu(:)];
        xList = [CrIy']; yList = [ CrIx'];
        xListStack = repmat(xList,1,size(xyOfst,1))';
        yListStack = repmat(yList,1,size(xyOfst,1))';
        xListStackRange = xListStack + repmat(xyOfst(:,1),1,size(xListStack,2));
        yListStackRange = yListStack + repmat(xyOfst(:,2),1,size(yListStack,2));
        xListStackRange(xListStackRange < 1) = 1;
        xListStackRange(xListStackRange > ny) = ny;
        yListStackRange(yListStackRange < 1) = 1;
        yListStackRange(yListStackRange > nx) = nx;
        
        xySub = [xListStackRange(:) yListStackRange(:)];
        indTmp = sub2ind([nx,ny],xySub(:,2),xySub(:,1));
        IiTmp = Ii(1:nx,1:ny,l+1);
        grayValue = IiTmp(indTmp);
        imgStack = reshape(grayValue,size(xu,1),size(xu,2),[]);
        imgStack_ = permute(imgStack,[1 3 2]);
        imgStack__ = reshape(imgStack_,size(xu,1)*size(imgStack,3),[]);
        xCoord = reshape(xListStackRange,size(xu,1),size(xu,2),[]);
        %         xCoord_ = permute(xCoord,[1 3 2]);
        %         xCoord_ = reshape(imgStack_,size(xu,1)*size(imgStack,3),[]);
        yCoord = reshape(yListStackRange,size(xu,1),size(xu,2),[]);
        %         xCoord_ = permute(xCoord,[1 3 2]);
        [xuuu,yuuu] = meshgrid(1:size(imgStack__,2),1:size(imgStack__,1));
        
        inMat = zeros(length(rowOfst),length(colOfst));
        inMat(3:end-2,3:end-2) = 1;
        InMat = repmat(inMat,NMAX,1);
        pix = Img2Pix(InMat,InMat);
        index = sub2ind(size(InMat),pix(:,2),pix(:,1));
        
        xFrac = repmat(ItIy,size(xu,1)*size(xu,2),1);
        xFrac = reshape(permute(reshape(xFrac,size(xu,1),size(xu,2),NMAX),[1 3 2]),size(xu,1)*size(imgStack,3),[]);
        yFrac = repmat(ItIx,size(xu,1)*size(xu,2),1);
        yFrac = reshape(permute(reshape(yFrac,size(xu,1),size(xu,2),NMAX),[1 3 2]),size(xu,1)*size(imgStack,3),[]);
        
        SI_nim = interp2(xuuu,yuuu,imgStack__,xuuu + xFrac,yuuu + yFrac);
        
        
        [Gy,Gx] = gradient(SI_nim);
        
        
        Gy = reshape(Gy(index),[],length(rowOfst)-4)';Gy = reshape(Gy,(length(rowOfst)-4)^2,[]);
        Gx = reshape(Gx(index),[],length(rowOfst)-4)';Gx = reshape(Gx,(length(rowOfst)-4)^2,[]);
        SI_nim = reshape(SI_nim(index),[],length(rowOfst)-4)';SI_nim = reshape(SI_nim,(length(rowOfst)-4)^2,[]);
        
        SI_nim_ = reshape(SI_nim,2*wintx+1,2*wintx+1,[]);
        
        DirVec = [-Line_level(2,:); Line_level(1,:)];
        DirVec(2,DirVec(1,:) == 0) = 1;
        DirVecStackX = repmat(DirVec(1,:),size(Gx,1),1);
        DirVecStackY = repmat(DirVec(2,:),size(Gx,1),1);
        
        %          Gge_ = dot(repmat(DirVec,1, length(gx(:))),[gy(:) gx(:)]');
        
        Gge_ = (DirVecStackX.*Gy) + (DirVecStackY.*Gx);
        Gge__ = permute(reshape(Gge_,length(rowOfst)-4,length(rowOfst)-4,[]),[2 1 3]);
        Gge = sum(DirVecStackX.*Gy) + sum(DirVecStackY.*Gx);
        Gge_new = sum((DirVecStackX.*Gy+DirVecStackY.*Gx).^2);
        prvPtInFlag = ((CrIx>=3+wintx+boundary_t) & (CrIx<=nx-wintx-2-boundary_t) &  (CrIy>=3+winty+boundary_t) & (CrIy<=ny-winty-2-boundary_t));
        
        %         IIi(CrIx-wintx-2:CrIx+wintx+2,CrIy-winty-2:CrIy+winty+2,l+1);
        
        done = 0;
        for i = 1 : 1 %NMAX,
            
            
            if i == 1556
                asfghkj = 1;
            end
            if 1 % goodfeat(i) && (~div_list(i)),  % if it is a good feature...
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
                if 1 %((crIx>=3+wintx+boundary_t) & (crIx<=nx-wintx-2-boundary_t) &  (crIy>=3+winty+boundary_t) & (crIy<=ny-winty-2-boundary_t))   % | abs(ge) < 50
                    if 0
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
                        
                        %                     SI = imfilter(SI, gaussianFilter,'symmetric');
                        
                        
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
                    end
                    v(:,i) = [0;0]; 	% initial flow
                    div = 0; 		% still on the screen
                    compt = 0; 		% no iteration yet
                    v_extra = resolution + 1; % just larger than resolution
                    
                    %%
                    if 1
                        if l~=0, compt_max = 15; else compt_max = 1; end;
                    else
                        compt_max = 15;
                    end
                    
                    
                    errMat = []; errMat2 = []; deltV = [];
                    if l ~= 0 %& i == 1
                        dbjk = 1;
                    end
                    
                    clear vvv;
                    if 1 %done == 0
                        V = zeros(2,NMAX);
                        idNeedIter = ones(1,NMAX);
                        DeltV = zeros(NMAX,compt_max); %nan(NMAX,compt_max);
                        Div = double(repmat(~goodfeat,1,compt_max));
                        idLeft = find(goodfeat);
                        resoThr = [~goodfeat'];
                        resoThr0 = [~goodfeat'];
                        outFlagStack = zeros(NMAX,compt_max);
                        while (sum(idNeedIter) > 0) && (sum(max(Div')) < NMAX) && (compt<compt_max),
                            if 0
                                CIpx = CIx + d(1,:) + V(1,:);
                                CIy_ = xOnLine2(Line_level, CIx);
                                %                             xyOnLine = cross(pextend([]))
                                
                                Idd = find(Line_level(1,:) == 0 & Line_level(2,:) == 1);
                                CIy_(Idd) = CIy(Idd);
                                CIpy0 = CIy + d(2,:) + V(2,:); % Coords. of the point
                                if useFundMat
                                    CIpy = CIy_ + d(2,:) + V(2,:); % Coords. of the point
                                else
                                    CIpy = CIpy0;
                                end
                            end
                            
                            % %                             proj_point = ProjPoint( [CIy;CIx],Points2_level' );
                            proj_point = ProjPoint2( [CIy;CIx],Line_level );
                            proj_point(:,Line_level(2,:) == 0) = [CIy(Line_level(2,:) == 0);CIx(Line_level(2,:) == 0)];
                            
                            CIpx = proj_point(2,:) + d(1,:) + V(1,:);
                            CIpy = proj_point(1,:) + d(2,:) + V(2,:);
                            if 0
                                
                                figure,plot(dot(pextend(proj_point),Line_level))
                                dir1 = [CIy; CIx] - proj_point;
                                dir2 = (Points2_level(:,1:2)-Points2_level(:,3:4))';
                                figure,plot(dot(dir1,dir2));
                            end
                            Comp_ = (proj_point([2 1],:)-[CIx;CIy])';
                            %                             Comp_ = (CIy_-CIy)';
                            Comp = Comp_;
                            Comp(setdiff([1:NMAX]',idLeft),:) = 0;
                            %                         er = [CIpy0-CIpy];
                            %
                            %
                            %                         cIpx = cIx + d(1,i) + V(1,i); % + extra_m;	%
                            %                         cIy_ = xOnLine(line_level, cIx);
                            %                         cIpy0 = cIy + d(2,i) + V(2,i); % Coords. of the point
                            %
                            %
                            %                         if useFundMat
                            %                             cIpy = cIy_ + d(2,i) + V(2,i); % Coords. of the point
                            %                         else
                            %                             cIpy = cIpy0;
                            %                         end
                            %                         er = [er;cIpy0-cIpy];
                            %                         cIpx = cIpx - 0.9;
                            if 0
                                cIpy = xOnLine(line_level, cIpx);
                            end
                            %                         crIpx = round(cIpx); 	% on the final image
                            %                         crIpy = round(cIpy); 	%
                            %                         crIpx_ = round(cIpx); 	% on the final image
                            %                         crIpy_ = round(cIpy);
                            %                         crIpx_frac = (cIpx - crIpx_); 	% on the final image
                            %                         crIpy_frac = (cIpy - crIpy_);
                            
                            
                            %                             [SIp_nim33, curPtInFlag33] = CalcAll([CIpx;CIpy], wintx, winty, nx, ny, Ipi, index, xuuu, yuuu,l,NMAX, boundary);
                            [SIp_nim_, curPtInFlag_] = CalcAll([CIpx(idLeft);CIpy(idLeft)], wintx, winty, nx, ny, Ipi, index, xuuu, yuuu,l,length(idLeft), boundary);
                            SIp_nim = zeros(size(SIp_nim_,1),NMAX);
                            curPtInFlag = true(1,NMAX);
                            SIp_nim(:,idLeft) = SIp_nim_;
                            
                            
                            %                             curPtInFlag(idLeft) = curPtInFlag_;
                            %                             curPtInFlag(setdiff((1:NMAX),idLeft)) = 1;
                            curPtInFlag = (round(CIpx)>=2+wintx+boundary) & (round(CIpx)<=nx-wintx-1-boundary) &  (round(CIpy)>=2+winty+boundary) & (round(CIpy)<=ny-winty-1-boundary);
                            curPtInFlag0 = curPtInFlag;
                            curPtInFlag(setdiff([1:NMAX],idLeft)) = 1;
                            
                            
                            
                            if 0
                                figure,imshow(SIp_nim-SIp_nim33,[]);
                                figure,imshow(SIp_nim(:,idLeft)-SIp_nim33(:,idLeft),[]);
                                figure,plot(curPtInFlag-curPtInFlag33);
                                
                                kk = idLeft(idin)
                                reshape(SIp_2_nim(:,:,kk),[],1)-SIp_nim(:,kk)
                            end
                            
                            
                            SIp_nim(isnan(SIp_nim)) = rand(sum(sum(isnan(SIp_nim))),1);
                            Gt = SI_nim - SIp_nim;
                            %                             Lambda = sum(Gt)./Gge;
                            Lambda = sum(Gt.*Gge_)./Gge_new;
                            
                            VVV = [repmat(Lambda,2,1).*DirVec([2 1],:)];
                            VVV(:,~goodfeat0) = 0;
                            curLim = abs(Lambda) < resolution;
                            unLim = resoThr(1,:) == 0;
                            updateState = curLim & unLim;
                            
                            VVV(:,resoThr(1,:) == 1) = 0;
                            Lambda(resoThr == 1) = 0;
                            DeltV(:,compt+1) = [abs(Lambda)]';
                            
                            
                            resoThr(1,updateState|~curPtInFlag) = 1;
                            resoThr0(1,updateState|~curPtInFlag0) = 1;
                            
                            V_extra = VVV;
                            %                         [~,dltLen] = NormalizeVector(VVV');
                            Div(~curPtInFlag,compt+1) = 1;
                            outFlagStack(:,compt+1) = [~curPtInFlag'];
                            
                            %                             V(:,curPtInFlag) = V(:,curPtInFlag) + V_extra(:,curPtInFlag);
                            V = V + V_extra;
                            V_tmp = V;
                            if 0
                                CIpx_tmp = CIx + d(1,:) + V_tmp(1,:);
                                CIy_tmp_ = xOnLine2(Line_level, CIx);
                                Idd_tmp = find(Line_level(1,:) == 0 & Line_level(2,:) == 1);
                                CIy_tmp_(Idd_tmp) = CIy(Idd_tmp);
                                
                                CIpy0_tmp = CIy + d(2,:) + V_tmp(2,:); % Coords. of the point
                                if useFundMat
                                    CIpy_tmp = CIy_tmp_ + d(2,:) + V_tmp(2,:); % Coords. of the point
                                else
                                    CIpy_tmp = CIpy0_tmp;
                                end
                            else
                                
                                CIpx_tmp = proj_point(2,:) + d(1,:) + V_tmp(1,:);
                                CIpy_tmp = proj_point(1,:) + d(2,:) + V_tmp(2,:);
                            end
                            PixNew_2 = [CIpx_tmp;CIpy_tmp];
                            
                            %                             [SIp_2_nim33, curPtInFlag_tmp33] = CalcAll(PixNew_2, wintx, winty, nx, ny, Ipi, index, xuuu, yuuu,l,NMAX, boundary);
                            try
                                PixNew_3 = PixNew_2(:,idLeft);
                                idin = (round(PixNew_3(1,:))>=2+wintx+boundary) & (round(PixNew_3(1,:))<=nx-wintx-1-boundary) &  (round(PixNew_3(2,:))>=2+winty+boundary) & (round(PixNew_3(2,:))<=ny-winty-1-boundary);
                                %                             [SIp_2_nim__, curPtInFlag_tmp__] = CalcAll([PixNew_3(:,idin)], wintx, winty, nx, ny, Ipi, index, xuuu, yuuu,l,length(idLeft), boundary);
                                try
                                    [SIp_2_nim__, curPtInFlag_tmp__] = CalcAll([PixNew_3(:,idin)], wintx, winty, nx, ny, Ipi, index, xuuu, yuuu,l,sum(idin), boundary);
                                catch
                                    SIp_2_nim = SI_nim;
                                    SIp_2_nim = reshape(SIp_2_nim,2*wintx+1,2*wintx+1,[]);
                                    break;
                                end
                                curPtInFlag_tmp_ = false(1,length(idLeft));
                                curPtInFlag_tmp_(idin) = curPtInFlag_tmp__;
                                SIp_2_nim_ = zeros(size(SIp_2_nim__,1),length(idLeft));
                                SIp_2_nim_(:,idin) = SIp_2_nim__;
                                
                                SIp_2_nim = zeros(size(SIp_2_nim_,1),NMAX);
                                curPtInFlag_tmp = false(1,NMAX);
                                SIp_2_nim(:,idLeft) = SIp_2_nim_;
                                curPtInFlag_tmp(idLeft) = curPtInFlag_tmp_;
                                curPtInFlag_tmp = (round(PixNew_2(1,:))>=2+wintx+boundary) & (round(PixNew_2(1,:))<=nx-wintx-1-boundary) &  (round(PixNew_2(2,:))>=2+winty+boundary) & (round(PixNew_2(2,:))<=ny-winty-1-boundary);
                            catch
                                [SIp_2_nim, curPtInFlag_tmp] = CalcAll(PixNew_2, wintx, winty, nx, ny, Ipi, index, xuuu, yuuu,l,NMAX, boundary);
                                break;
                            end
                            if 0
                                figure,imshow(SIp_2_nim-SIp_2_nim33,[]);
                                figure,imshow(SIp_2_nim(:,idLeft)-SIp_2_nim33(:,idLeft),[]);
                                figure,plot(curPtInFlag_tmp-curPtInFlag_tmp33);
                                
                                figure,imshow(SIp_2_nim(:,:,47954),[])
                                DeltV(47954,:)
                                norm(PixNew_2(:,47954) - [CIpx(:,47954); CIpy(:,47954)])
                                
                            end
                            
                            
                            SIp_2_nim = reshape(SIp_2_nim,2*wintx+1,2*wintx+1,[]);
                            
                            
                            
                            Lambda(resoThr == 1) = 0;
                            idNeedIter = abs(Lambda) > resolution;
                            compt = compt + 1;
                        end
                        
                        
                        
                        if size(outFlagStack,2) > 1
                            outFlagStack_ = sum(outFlagStack')';
                            outInd = outFlagStack_ > 0;
                        else
                            outInd = outFlagStack > 0;
                        end
                        
                        
                        
                        if compt_max ~=1
                            badCnt33 = [badCnt33;find(DeltV(:,end) ~=0)];
                        else
                            badCnt33 = [];
                        end
                        DeltVv = DeltV;
                        DeltVv(DeltV == 0) = nan;
                        [~,Idm] = max(DeltVv');
                        if compt_max ~=1
                            if ~ismember(DeltV,[1 2 3])
                                badCnt44 = [badCnt44; find(~ismember(Idm,[1 2 3]))'];
                            end
                        else
                            badCnt44 = [];
                        end
                        %                         response(i,l+1) = sum(sum(abs(ge__(2:end-1,2:end-1))))/((size(ge__,1)-2)*(size(ge__,2)-2));
                        responsee(:,l+1) = sum(sum(abs(Gge__(2:end-1,2:end-1,:))))./((size(Gge__,1)-2)*(size(Gge__,2)-2));
                        responsee(~goodfeat,l+1) = 0;
                        
                        
                        if compt_max ~= 1
                            [~,iddd] = min(DeltVv'); iddd = iddd';
                            iDx = (~isnan(DeltVv));
                            iddd = sum(iDx')';
                            id_good = (DeltV(:,1))<resolution;
                            iddd(id_good) = 15;
                            iddd(iddd == 0) = 1;
                            %                             id_good = (DeltV(:,1))<resolution;
                            %                             [ycoord,xcoord] = ind2sub(size(DeltV),iDx);
                            %                             [indexCellRoboStill, ~] = splitIndex2(unique(xcoord));
                            tmp1 =  max(abs(diff(DeltV')))';
                            tmp2 =  (abs((DeltV(:,1)')))';
                            %                         if length(deltV) > 1
                            %                             Var2(i,l+1) = max(abs(diff(deltV))); mean(deltV);
                            Var222(logical(goodfeat) & iddd ~= 1,l+1) = tmp1(logical(goodfeat) & iddd ~= 1); %max(abs(diff(DeltV')))';
                            %                         else
                            Var222(goodfeat & iddd == 1,l+1) = tmp2(goodfeat & iddd == 1); %max(abs((DeltV')))';
                            
                            %                         end
                            
                            
                            inD = sub2ind(size(DeltV), (1:size(DeltV,1))',iddd);
                            %                         if max(DeltV') < wintx  && length(deltV) > 1% wintx/2^l
                            ttemp3 = DeltV(inD)-0.5*DeltV(:,1);
                            Var333( max(DeltV')' < wintx & goodfeat & iddd ~= 1,l+1) = ttemp3(max(DeltV')' < wintx & goodfeat & iddd ~= 1); %deltV(end)-0.5*deltV(1);
                            %                         end
                            %                         if max(DeltV') >= wintx  && length(deltV) > 1
                            Var333(max(DeltV')' >= wintx & goodfeat & iddd ~= 1,l+1) = 1;
                            %                         end
                            %                         if max(DeltV') < wintx  && length(deltV) == 1
                            Var333(max(DeltV')' < wintx & goodfeat  & iddd == 1,l+1) = -1;
                            %                         end
                            %                         if max(DeltV') >= wintx  && length(deltV & iddd == 1) == 1
                            Var333(max(DeltV')' >= wintx & goodfeat,l+1) = 1;
                            %                         end
                            
                        else
                            temp4 = (abs((DeltV)));
                            try
                                Var222(logical(goodfeat),l+1) = temp4(logical(goodfeat)); %(abs((DeltV)));
                            catch
                                asihu = 1;
                            end
                            
                            %                             Var333( max(DeltV')' < wintx & goodfeat & iddd ~= 1,l+1) = deltV(end)-0.5*deltV(1);
                            %                             Var333(max(DeltV')' >= wintx & goodfeat & iddd ~= 1,l+1) = 1;
                            Var333((DeltV')' < wintx & goodfeat,l+1) = -1;
                            Var333((DeltV')' >= wintx & goodfeat,l+1) = 1;
                        end
                        
                        
                        
                        if l == 0
                            
                            if 0
                                sSI_4 = SI_nim_(2:end-1,2:end-1,:);
                                sSIp_2_nim = SIp_2_nim(2:end-1,2:end-1,:);
                            else
                                sSI_4 = SI_nim_;
                                sSIp_2_nim = SIp_2_nim;
                            end
                            aaa = zeros(1,1,NMAX);
                            aaa(1,1,:) = mean(reshape(sSI_4,size(sSI_4,1)^2,[]),1);
                            sSI_4_mean = repmat(aaa,size(sSI_4,1),size(sSI_4,1),1);
                            sSI_1 = sSI_4 - sSI_4_mean; % mean(sSI_4,3);
                            sSI_1_ = sSI_1;
                            sSI_1_ = reshape(sSI_1_,size(sSI_1_,1)^2,[]);
                            sSI_1_ = sSI_1_.^2;
                            bbb = zeros(1,1,NMAX);
                            bbb(1,1,:) = mean(reshape(sSIp_2_nim,size(sSIp_2_nim,1)^2,[]),1);
                            sSIp_2_nim_mean = repmat(bbb,size(sSIp_2_nim,1),size(sSIp_2_nim,1),1);
                            
                            sSIp_21 = sSIp_2_nim - sSIp_2_nim_mean; %mean(sSIp_2_nim(:));
                            Gt1 = sSI_1-sSIp_21;
                            Gt1 = reshape(Gt1,size(Gt1,1)^2,[]);
                            Gt1 = Gt1.^2;
                            Residualss = (sqrt(sum(Gt1)./((2*wintx+1)*(2*winty+1)-1)))';
                            Residualss(logical(~goodfeat)) = 0;
                            ratioss = (sum(Gt1)./sum(sSI_1_))';
                            ratioss(logical(~goodfeat)) = 0;
                            
                            
                            if 0
                                [~,flowlen] = NormalizeVector((d + V)');
                                %     flow = zeros(size(img1));
                                resid = nan(size(Ii(:,:,1))); rati = resid; res = resid; floW = resid;
                                iNd = sub2ind(size(Ii(:,:,1)), round(xt(1,:)), round(xt(2,:)));
                                resid(iNd) = Residualss;
                                rati(iNd) = ratioss;
                                res(iNd) = responsee(:,1);
                                floW(iNd) = flowlen;
                                figure,imshow((rati<0.3 & resid<10 & res > 1),[]);figure,imshow(floW,[])
                                
                                [~,flowlen] = NormalizeVector((d + V)');resid = nan(size(Ii(:,:,1))); rati = resid; res = resid; floW = resid;iNd = sub2ind(size(Ii(:,:,1)), round(xt(1,:)), round(xt(2,:)));resid(iNd) = Residualss;rati(iNd) = ratioss;res(iNd) = responsee(:,1);floW(iNd) = flowlen;figure,imshow((rati<0.3 & resid<10 & res > 1),[]);figure,imshow(floW,[]);
                            end
                            
                            
                            
                        end
                        
                        if 0
                            kk = 8747;
                            figure,imshow(reshape(SI_nim(:,kk),2*wintx+1,2*wintx+1),[])
                            figure,imshow(SIp_2_nim(:,:,kk),[])
                        end
                        
                        done = 1;
                    end
                    compt = 0;
                    %                     while (sum(idNeedIter) > 0) && (~div) && (compt<compt_max),
                    if 0
                        while (norm(v_extra) > resolution) && (~div) && (compt<compt_max),
                            %                         CIpx = CIx + d(1,:) + V(1,:);
                            %                         CIy_ = xOnLine2(Line_level, CIx);
                            %                         CIpy0 = CIy + d(2,:) + V(2,:); % Coords. of the point
                            %                         if useFundMat
                            %                             CIpy = CIy_ + d(2,:) + V(2,:); % Coords. of the point
                            %                         else
                            %                             CIpy = CIpy0;
                            %                         end
                            %                         er = [CIpy0-CIpy];
                            %
                            %
                            cIpx = cIx + d(1,i) + v(1,i); % + extra_m;	%
                            cIy_ = xOnLine(line_level, cIx);
                            cIpy0 = cIy + d(2,i) + v(2,i); % Coords. of the point
                            
                            
                            if useFundMat
                                cIpy = cIy_ + d(2,i) + v(2,i); % Coords. of the point
                            else
                                cIpy = cIpy0;
                            end
                            %                         er = [er;cIpy0-cIpy];
                            %                         cIpx = cIpx - 0.9;
                            if 0
                                cIpy = xOnLine(line_level, cIpx);
                            end
                            crIpx = round(cIpx); 	% on the final image
                            crIpy = round(cIpy); 	%
                            crIpx_ = round(cIpx); 	% on the final image
                            crIpy_ = round(cIpy);
                            crIpx_frac = (cIpx - crIpx_); 	% on the final image
                            crIpy_frac = (cIpy - crIpy_);
                            %
                            %
                            %                         [SIp_nim, curPtInFlag] = CalcAll([CIpx;CIpy], wintx, winty, nx, ny, Ipi, index, xuuu, yuuu,l,NMAX, boundary);
                            %                         Gt = SI_nim - SIp_nim;
                            %                         Lambda = sum(Gt)./Gge;
                            %                         VVV = [repmat(Lambda,2,1).*DirVec([2 1],:)];
                            %                         V_extra = VVV;
                            % %                         [~,dltLen] = NormalizeVector(VVV');
                            %                         DeltV(:,compt+1) = [abs(Lambda)]';
                            %                         Div(~curPtInFlag,compt+1) = 1;
                            %
                            %
                            %                         V(:,curPtInFlag) = V(:,curPtInFlag) + V_extra(:,curPtInFlag);
                            %                         V_tmp = V;
                            %                         CIpx_tmp = CIx + d(1,:) + V_tmp(1,:);
                            %                         CIy_tmp_ = xOnLine2(Line_level, CIx);
                            %                         CIpy0_tmp = CIy + d(2,:) + V_tmp(2,:); % Coords. of the point
                            %                         if useFundMat
                            %                             CIpy_tmp = CIy_tmp_ + d(2,:) + V_tmp(2,:); % Coords. of the point
                            %                         else
                            %                             CIpy_tmp = CIpy0_tmp;
                            %                         end
                            %                         PixNew_2 = [CIpx_tmp;CIpy_tmp];
                            %
                            %                        [SIp_2_nim, curPtInFlag_tmp] = CalcAll(PixNew_2, wintx, winty, nx, ny, Ipi, index, xuuu, yuuu,l,NMAX, boundary);
                            %
                            %                        SIp_2_nim = reshape(SIp_2_nim,2*wintx+1,2*wintx+1,[]);
                            %
                            %
                            %                        idNeedIter = Lambda > resolution;
                            
                            
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
                                [xu, yu] = meshgrid(1:7, 1:7);
                                SIp = Ipi(crIpx-wintx-1:crIpx+wintx+1,crIpy-winty-1:crIpy+winty+1,l+1);
                                nim = interp2(xu,yu,SIp,xu+crIpy_frac,yu+crIpx_frac);
                                nim2 = conv2(conv2(SIp,vIpx,'same'),vIpy,'same');
                                error_ = [error_;sum(sum(abs(nim(2:end-1,2:end-1) - nim2(2:end-1,2:end-1))))];
                                SIp = conv2(conv2(SIp,vIpx,'same'),vIpy,'same');
                                SIp = SIp(2:2*wintx+2,2:2*winty+2);
                                %                             SIp = imfilter(SIp, gaussianFilter,'symmetric');
                                
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
                                
                                %                             [v_2,div_2,SI_2, SIp_2, pixNew_2] = epipoleLK_fcn4(gx_,gy_,SI_,Ipi,cIx,d,i,wintx,boundary,winty,nx,ny,l,line_level,v_extra(1));
                                
                                
                                compt = compt + 1;
                                v(:,i) = v(:,i) + v_extra;
                                %                            [v_2,div_2,SI_2, SIp_2, pixNew_2] = epipoleLK_fcn4(gx_,gy_,SI_,Ipi,cIx,d,i,wintx,boundary,winty,nx,ny,l,line_level,v(1,i));
                                SI_ = SI; gx_ = gx; gy_ = gy;
                                [v_2,div_2,SI_2, SIp_2, pixNew_2] = epipoleLK_fcn4(gx,gy,SI,Ipi,cIx,d,i,wintx,boundary,winty,nx,ny,l,line_level,v(1,i));
                                afkbj = 1;
                                
                                if 0
                                    figure,imshow([SI_  1*ones(size(SI_,1),1) SIp_2], [])
                                end
                            else
                                div = 1; 		% Feature xtt out of the image
                            end;
                        end;
                        scale = 1; segNum = 5; upper = (segNum-1)/2; lower = upper;
                        segNum = 1;
                        segPix = 20;
                        ge___ = imresize(ge__,scale);
                        SII = imresize(SI,scale);
                        if 0
                            figure,plotContour(SII(:),[1:size(SII,2)],[1:size(SII,1)],10)
                        end
                        
                        
                        ge___ = SII;
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
                                %                             curveInd = round(round(lengthCurve)/2-round(lengthCurve)/10:round(lengthCurve)/2+round(lengthCurve)/10);
                                curveInd = round(round(lengthCurve)/2-round(lengthCurve)/2.5:round(lengthCurve)/2+round(lengthCurve)/2.5);
                                try
                                    % %                                 saliencyMat = [saliencyMat;[min(-minv) max(maxv)]];
                                    saliencyMat = [saliencyMat;[max(curve{kkk,1}(curveInd))- min(curve{kkk,1}(curveInd)) mean([max(curve{kkk,1}(curveInd)) min(curve{kkk,1}(curveInd))])]];
                                catch
                                    sfnwejk = 1;
                                end
                                
                            end
                            try
                                if i == 198;
                                    asgbk = 1;
                                end
                                sadjf = vvv;
                                if l == 0
                                    contourVar(i,1) = abs((max(curve{(segNum+1)/2}(curveInd)) - min(curve{(segNum+1)/2}(curveInd)))/mean(curve{(segNum+1)/2}(curveInd)));
                                end
                            catch
                                adgsfhk = 1;
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
                            figure(14),clf;imshow(imresize(SI,scale),[]);hold on;plot([10*dirVec(1) 0],[10*dirVec(2) 0],'-g');plot(10+[10*dirVec(1) 0],10+[10*dirVec(2) 0],'-r');title(num2str(ge));figure(15);clf;imshow(Ii(:,:,l+1),[]);hold on;plot(cIy,cIx,'*r');title(num2str(round((d(:,i)+v(:,i))',3)));
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
                            %                         Var(i,:) = [sqrt(var(deltV))/length(deltV) mean(deltV) deltV(end)-0.5*deltV(1)];
                            %                         Var1(i,l+1) = sqrt(var(deltV))/length(deltV);
                            
                            Var(i,:) = [sqrt(var(deltV))/1 mean(deltV) deltV(end)-0.5*deltV(1)];
                            Var1(i,l+1) = sqrt(var(deltV))/1;
                            if length(deltV) > 1
                                Var2(i,l+1) = max(abs(diff(deltV))); mean(deltV);
                            else
                                Var2(i,l+1) = max(abs((deltV)));
                            end
                            response(i,l+1) = sum(sum(abs(ge__(2:end-1,2:end-1))))/((size(ge__,1)-2)*(size(ge__,2)-2));
                            [~,idm] = max(deltV);
                            if ~ismember(idm,[1 2 3])
                                badCnt4 = [badCnt4;i];
                            end
                            if response(i,l+1) < 250
                                asiu = 1;
                            end
                            %                         if length(deltV) ~= 0
                            if max(deltV) < wintx  && length(deltV) > 1% wintx/2^l
                                Var3(i,l+1) = deltV(end)-0.5*deltV(1);
                            end
                            if max(deltV) >= wintx  && length(deltV) > 1
                                Var3(i,l+1) = 1;
                            end
                            if max(deltV) < wintx  && length(deltV) == 1
                                Var3(i,l+1) = -1;
                            end
                            if max(deltV) >= wintx  && length(deltV) == 1
                                Var3(i,l+1) = 1;
                            end
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
                                    SI_4 = SI_(2:end-1,2:end-1);
                                    SIp_24 = SIp_2(2:end-1,2:end-1);
                                    SI_1 = SI_4 - mean(SI_4(:));
                                    SIp_21 = SIp_24 - mean(SIp_24(:));
                                    try
                                        gt1 = SI_1-SIp_21;
                                        SI_1 = SI_1;
                                    catch
                                        gt1 = 50000;
                                    end
                                catch
                                    errMat3 = 50000;
                                    gt1 = 50000;
                                end
                                errMat3 = abs(sum(sum(gt1 .* gx_(2:end-1,2:end-1)))) + abs(sum(sum(gt1 .* gy_(2:end-1,2:end-1))));
                            else
                                %                             errMat3 = abs(sum(sum(gt .* gx_))) + abs(sum(sum(gt .* gy_)));
                                SI3 = SI(2:end-1,2:end-1);
                                SIp3 = SIp(2:end-1,2:end-1);
                                SI1 = SI3 - mean(SI3(:));
                                SIp1 = SIp3 - mean(SIp3(:));
                                gt1 = SI1 - SIp1;
                                gt1 = gt1(2:end-1,2:end-1);
                                SI1 = SI1(2:end-1,2:end-1);
                                errMat3 = abs(sum(sum(gt1 .* gx(2:end-1,2:end-1)))) + abs(sum(sum(gt1 .* gy(2:end-1,2:end-1))));
                            end
                            
                            if 0
                                figure,imshow([SI_4 SIp_24],[])
                                sdvkbj = 1;
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
                                %                         infoMat(i,:) =  [[norm(vvv) errMat3 l var([gx(:);gy(:)])] Residuals(i) ratios(i) xt(2,i) xt(1,i)];
                                infoMat(i,:) =  [[norm(d(:,1)+v(:,i))*2^l errMat3 l var([gx(:);gy(:)])] Residuals(i) ratios(i) xt(2,i) xt(1,i)];
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
                            if length(deltV) == compt_max %0
                                badCnt3 = [badCnt3;i];
                            end
                        end
                    end
                else
                    div = 1; 		% Feature xt out of the image
                end;
                
                div_list(i) = div_list(i) | div; % Feature out of the image
                
                
                if size(Div,2) == 1
                    
                else
                    Divv = sum(Div')';
                    
                end
                div_listt = div_listt | outInd;  % ~Div(:,end);
                if 0
                    figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,div_listt),xt(1,div_listt),'.')
                end
            end;
            try
                xyNew(:,i) = pixNew_2; pixNew;
                Goodness(:,i) = goodness;
                
            catch
                askbj = 1;
            end
            % % %             Ge(:,i) = ge;
        end;
        
        d = d + V; 			% propagation of the displacement
        if useFundMat
            if iscomp == 0
                %                 d(2,:) = d(2,:) + comp';
                %                 Comp(abs(Comp)  > compThr) = 0;
                [~,compLen] = NormalizeVector(Comp);
                %                 d(2,:) = d(2,:) + Comp';
                d = d + Comp';
                badComp = find((compLen) > compThr/2^l);
                %                 badComp = [];
                iscomp = 1;
            else
                savbjk = 1;
            end
        end
    end;
    if size(Var1,2) ~=1
        Var11 = max(Var1')';
        Var22 = max(Var2')';
        Var33 = max(Var3')';
        Var = [Var11 Var22 Var33];
        Response = max(response')';
    else
        Var = [Var1 Var2 Var3];
        Response = response;
    end
    
    
    
    if size(Var222,2) ~=1
        %         Var11 = max(Var1')';
        Var2222 = max(Var222')';
        Var3333 = max(Var333')';
        Varr = [Var2222 Var2222 Var3333];
        %         Responsee = max(responsee')';
        Responsee = min(responsee')';
    else
        Varr = [Var222 Var222 Var333];
        Responsee = responsee;
    end
    if 0
        figure,plot(Var2-Var222)
        figure,plot(Var3-Var333)
        figure,plot(Response -Responsee)
        figure,plot(Var(:,2:3) - Varr(:,2:3))
        figure,plot(Response -Responsee)
    end
    sdfjhg = 1;
    
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
    %     Qtt = ComputeQuality(Ipi,xtt,goodfeat,wintx,winty);
    Qtt = 10000*ones(NMAX,1);
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
        %         TF = (~div_list) & (Qtt > 0) & (((Residuals < 18) & (ratios < 0.9) )| (abs(Ge') > 0.5));
        if 0
            TF = (~div_list) & (Qtt > 0) & (((Residuals < 18) & (ratios < 0.9) )| (Response > 10));
            Responses = [Responses Response];
        else
            %             TF = (~div_list) & (Qtt > 0) & (((Residualss < 18) & (ratioss < 0.9) )| (Responsee > 10));
            %             TF = (~div_listt) & (Qtt > 0) & (((Residualss < 18) & (ratioss < 0.9) ) | (Responsee > 50));
            %             TF = (~div_listt) & (Qtt > 0) & (((Residualss < 18) & (ratioss < 0.9) ) & (Responsee > 2));
            TF = (~div_listt) & (Qtt > 0) & (((Residualss < 18) & (ratioss < 0.7) ) & (Responsee > 5));
            %             TF = (~div_listt) & (Qtt > 0) & (((Residualss < 18) & (ratioss < 0.9) ));
            %             TF = (~div_listt) & (Qtt > 0) & (((Residualss < 15) & (ratioss < 0.5) ));
            TF = (~div_listt) & (Qtt > 0) & (((Residualss < 10) & (ratioss < 0.3) ) & (Responsee > 0.3));
            TF = (~div_listt) & (Qtt > 0) & (((Residualss < 15) & (ratioss < 0.7) ) & (Responsee > 0.3));
            %            TF = (~div_listt) & (Qtt > 0) & (((Residualss < 20) & (ratioss < 1.1) ) & (Responsee > 1));
            Responsess = [Responsess Responsee];
        end
        if 0 % sub_level == levelmax
            TF(unique([badCnt;badCnt2])) = 0;
        elseif 0
            %             TF(Var(:,1) > varThr | Var(:,2) > deltVThr | Var(:,3) >= 0) = 0;
            TF(Var(:,2) > deltVThr | Var(:,3) >= 0 | contourVar < 0.2) = 0;
            TF(unique([badCnt3])) = 0;
        elseif 0
            TF(Var(:,2) > deltVThr | Var(:,3) >= 0) = 0;
            TF(unique([badCnt3;badCnt4])) = 0;
        else
            %             TF(Varr(:,2) > deltVThr | Varr(:,3) >= 0) = 0;
            TF(Varr(:,2) > deltVThr | Varr(:,3) > 0) = 0;
            TF(unique([badCnt33;badCnt44;badComp])) = 0;
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
        if 0
            figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,wekeep),xt(1,wekeep),'.')
        end
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
resp = max(Responses')';
if size(Responsess,2) >1
    respp = max(Responsess')';
    resp = max(Responses')';
else
    respp = (Responsess')';
    resp = (Responses')';
end
goodfeat(unique(Idnan)) = 0;
% goodfeat = goodfeat0 & logical(Goodness');
end
% figure,ShowEpipoleLine3(fMat,Ii(:,:,1),Ipi(:,:,1),xt([2 1],goodfeat)',xtt([2 1],goodfeat)');
% thr = 10;figure,showMatchedFeatures(Ii(:,:,1),Ipi(:,:,1),xt([2 1],goodfeat & resp >thr)',xtt([2 1],goodfeat & resp > thr)');
% figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,badCnt33),xt(1,badCnt33),'.')
% figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,badCnt44),xt(1,badCnt44),'.')
% figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,Varr(:,3) > 0),xt(1,Varr(:,3) > 0),'.')
% figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,Varr(:,2) > deltVThr),xt(1,Varr(:,2) > deltVThr),'.')


% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,ind),xt(1,ind),'.g')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,badCnt33),xt(1,badCnt33),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,Varr(:,3) == 1),xt(1,Varr(:,3) == 1),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,Varr(:,3) >= 0),xt(1,Varr(:,3) >= 0),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,badCnt333),xt(1,badCnt333),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,outInd),xt(1,outInd),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,logical(resoThr)),xt(1,logical(resoThr)),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,logical(resoThr0)),xt(1,logical(resoThr0)),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,isnan(DeltVv(:,2))),xt(1,isnan(DeltVv(:,2))),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,DeltV(:,1))<resolution),xt(1,DeltV(:,1))<resolution),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,DeltV(:,1)<resolution),xt(1,DeltV(:,1)<resolution),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,div_listt),xt(1,div_listt),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,badCnt33),xt(1,badCnt33),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,badCnt44),xt(1,badCnt44),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,Varr(:,3) > 0),xt(1,Varr(:,3) > 0),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,Varr(:,2) > deltVThr),xt(1,Varr(:,2) > deltVThr),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,Var3333 > 0),xt(1,Var3333 > 0),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,wekeep),xt(1,wekeep),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,iddd==1),xt(1,iddd==1),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,ttemp3>0),xt(1,ttemp3>0),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,ttemp3<0.1),xt(1,ttemp3<0.1),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,ttemp3<0.01),xt(1,ttemp3<0.01),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,ttemp3<0.0),xt(1,ttemp3<0.0),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,Var333<0.0),xt(1,Var333<0.0),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,Var333(:,2)<0.0),xt(1,Var333(:,2)<0.0),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,Var333(:,3)<0.0),xt(1,Var333(:,3)<0.0),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,Var3333(:,l+1)<0.0),xt(1,Var3333(:,l+1)<0.0),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,Var333(:,l+1)<0.0),xt(1,Var333(:,l+1)<0.0),'.')
% % % % % % % % % % % figure,imshow(Ii(:,:,1),[]);hold on;plot(xt(2,TF),xt(1,TF),'.')

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
function cIy = xOnLine2(epiline, cIx)
% epiline = epiline';
cIy = (-epiline(2,:).*cIx-epiline(3,:))./epiline(1,:);

end
function cIy = yOnLine2(epiline, cIx)
% for i = 1 : size(epiline,2)
cIy = (-epiline(1,:).*cIx-epiline(3,:))./epiline(2,:);
% end
end
function [SI_nim, prvPtInFlag] = CalcAll(xc, wintx, winty, nx, ny, Ii, index, xuuu, yuuu, l,NMAX,boundary)
CIx = xc(1,:); 	                %
CIy = xc(2,:); 			% Coords. of the point
CrIx = round(CIx); 		% on the initial image
CrIy = round(CIy);
CrIx0 = CrIx;
CrIy0 = CrIy;
prvPtInFlag = (CrIx>=2+wintx+boundary) & (CrIx<=nx-wintx-1-boundary) &  (CrIy>=2+winty+boundary) & (CrIy<=ny-winty-1-boundary);

CrIx(CrIx<2+wintx+boundary) = 1;
CrIx(CrIx>nx-wintx-1-boundary) = nx-wintx-1-boundary;
CrIy(CrIy<2+winty+boundary) = 1;
CrIy(CrIy>ny-winty-1-boundary) = ny-winty-1-boundary;

ItIx = (CIx-(CrIx0 - CrIx) - CrIx);
ItIy = (CIy-(CrIy0 - CrIy) - CrIy);

%         IIi = repmat(Ii(:,:,l+1),1,1,NMAX);
rowOfst = [(-wintx-2): (wintx+2)];
colOfst = [(-winty-2) : (+winty+2)];
[xu, yu] = meshgrid(colOfst, rowOfst);
xyOfst = [xu(:) yu(:)];
xList = [CrIy']; yList = [ CrIx'];
xListStack = repmat(xList,1,size(xyOfst,1))';
yListStack = repmat(yList,1,size(xyOfst,1))';
xListStackRange = xListStack + repmat(xyOfst(:,1),1,size(xListStack,2));
yListStackRange = yListStack + repmat(xyOfst(:,2),1,size(yListStack,2));
xListStackRange(xListStackRange < 1) = 1;
xListStackRange(xListStackRange > ny) = ny;
yListStackRange(yListStackRange < 1) = 1;
yListStackRange(yListStackRange > nx) = nx;

xySub = [xListStackRange(:) yListStackRange(:)];
indTmp = sub2ind([nx,ny],xySub(:,2),xySub(:,1));
IiTmp = Ii(1:nx,1:ny,l+1);
try
    grayValue = IiTmp(indTmp);
catch
    agvhkj = 1;
end
imgStack = reshape(grayValue,size(xu,1),size(xu,2),[]);
imgStack_ = permute(imgStack,[1 3 2]);
imgStack__ = reshape(imgStack_,size(xu,1)*size(imgStack,3),[]);
% % % % %         xCoord = reshape(xListStackRange,size(xu,1),size(xu,2),[]);
% % % % % %         xCoord_ = permute(xCoord,[1 3 2]);
% % % % % %         xCoord_ = reshape(imgStack_,size(xu,1)*size(imgStack,3),[]);
% % % % %         yCoord = reshape(yListStackRange,size(xu,1),size(xu,2),[]);
% % % % % %         xCoord_ = permute(xCoord,[1 3 2]);


[xuuu,yuuu] = meshgrid(1:size(imgStack__,2),1:size(imgStack__,1));
inMat = zeros(length(rowOfst),length(colOfst));
inMat(3:end-2,3:end-2) = 1;
InMat = repmat(inMat,NMAX,1);
pix = Img2Pix(InMat,InMat);
index = sub2ind(size(InMat),pix(:,2),pix(:,1));

xFrac = repmat(ItIy,size(xu,1)*size(xu,2),1);
xFrac = reshape(permute(reshape(xFrac,size(xu,1),size(xu,2),NMAX),[1 3 2]),size(xu,1)*size(imgStack,3),[]);
yFrac = repmat(ItIx,size(xu,1)*size(xu,2),1);
yFrac = reshape(permute(reshape(yFrac,size(xu,1),size(xu,2),NMAX),[1 3 2]),size(xu,1)*size(imgStack,3),[]);

SI_nim = interp2(xuuu,yuuu,imgStack__,xuuu + xFrac,yuuu + yFrac);
SI_nim = reshape(SI_nim(index),[],length(rowOfst)-4)';
SI_nim = reshape(SI_nim,(length(rowOfst)-4)^2,[]);
% prvPtInFlag = ((CrIx>=3+wintx+boundary_t) & (CrIx<=nx-wintx-2-boundary_t) &  (CrIy>=3+winty+boundary_t) & (CrIy<=ny-winty-2-boundary_t));

end
function proj_point = ProjPoint( point,line_p )

x1 = line_p(1,:);
y1 = line_p(2,:);
x2 = line_p(3,:);
y2 = line_p(4,:);

x3 = point(1,:);
y3 = point(2,:);
[~,len] = NormalizeVector([x1'-x2',y1'-y2']);len = len';
yk = ((x3-x2).*(x1-x2).*(y1-y2) + y3.*(y1-y2).^2 + y2.*(x1-x2).^2) ./ (len.^2);
xk = ((x1-x2).*x2.*(y1-y2) + (x1-x2).*(x1-x2).*(yk-y2)) ./ ((x1-x2).*(y1-y2));


% if x1 == x2
xk(x1 == x2) = x1(x1 == x2);
% end

% if y1 == y2
xk(y1 == y2) = x3(y1 == y2);
% end

proj_point = [xk;yk];

end
function mn = ProjPoint2( point,line_p )
a = line_p(1,:);
b = line_p(2,:);
c = line_p(3,:);
m = point(1,:);
n = point(2,:);
mn = [(b.*b.*m-a.*b.*n-a.*c)./(a.*a+b.*b);(a.*a.*n-a.*b.*m-b.*c)./(a.*a+b.*b)] ;
end