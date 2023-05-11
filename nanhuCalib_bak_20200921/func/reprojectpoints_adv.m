function [err,stderr,MSE]=reprojectpoints_adv(ocam_model, RRfin, ima_proc, Xp_abs, Yp_abs, M)

width=ocam_model.width;
height=ocam_model.height;
c=ocam_model.c;
d=ocam_model.d;
e=ocam_model.e;
ss=ocam_model.ss;
xc=ocam_model.xc;
yc=ocam_model.yc;

M(:,3)=1;
Mc=[];
Xpp=[];
Ypp=[];
count=0;
MSE=0;
for i=ima_proc
    count=count+1;
    Mc=RRfin(:,:,i)*M';
%     [xp,yp]=omni3d2pixel(ss,Mc, width, height);
%     xp=xp*c + yp*d + xc;
%     yp=xp*e + yp + yc;    
    m=world2cam(Mc, ocam_model);
    xp=m(1,:);
    yp=m(2,:);
    
    if 0
        
        R=RRfin(:,:,i);
        R(:,3)=cross(R(:,1),R(:,2));
        r=rodrigues(R);
        t=RRfin(:,3,i);
        Mm = M;
        Mm(:,3) = 0;
        RR = R;
        RR(1,:) = -R(2,:);
        RR(2,:) = -R(1,:);
        tt = t;
        tt(1) = -t(2);
        tt(2) = -t(1);
        Mc_= -RR*Mm' + repmat(-tt,1,size(M,1));

        
        
        
        Mcc = Mc;
        Mcc = [-Mc(2,:);-Mc(1,:);Mc(3,:)];
        load('D:\Temp\20190102\calib1\oCamModel.mat')
        U_same = ocam_undistort_map(oCamModel,'OutputView','same');
        intrMat_same = U_same.K';
        pixProj = pflat(intrMat_same*Mcc);
        pixProj_ = pflat(intrMat_same*Mc_);
        [pixUndist, pixUndist3D] = remapFishEyePix([yp;xp], intrMat_same, oCamModel);
        figure,imshow(zeros(1080,1920));hold on;plot(pixUndist(1,:),pixUndist(2,:),'o-');plot(pixProj_(1,:),pixProj_(2,:),'.r');
        figure,imshow(zeros(1080,1920));hold on;plot(yp,xp,'o-');
    end
    
    sqerr= (Xp_abs(:,:,i)-xp').^2+(Yp_abs(:,:,i)-yp').^2;
    err(count)=mean(sqrt(sqerr));
    stderr(count)=std(sqrt(sqerr));
    MSE=MSE+sum(sqerr);
end
 
fprintf(1,'\n Average reprojection error computed for each chessboard [pixels]:\n\n');
for i=1:length(err)
    fprintf(' %3.2f %c %3.2f\n',err(i),177,stderr(i));
end

fprintf(1,'\n Average error [pixels]\n\n %f\n',mean(err));
fprintf(1,'\n Sum of squared errors\n\n %f\n',MSE);