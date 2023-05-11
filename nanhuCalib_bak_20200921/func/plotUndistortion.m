function pixNew = plotUndistortion(nc, nr, Kinit, kk_new,kc,R,skipNum)


% [mx,my] = meshgrid(1:nc/skipNum:(nc-0),1:nr/skipNum:(nr-0));
[mx,my] = meshgrid(0:nc/skipNum:(nc-0),0:nr/skipNum:(nr-0));
[nnx,nny]=size(mx);
px=reshape(mx',nnx*nny,1);
py=reshape(my',nnx*nny,1);
% % kk_new=[fc(1) alpha_c*fc(1) cc(1);0 fc(2) cc(2);0 0 1];
rays=inv(kk_new)*[px';py';ones(1,length(px))];
rays = R'*rays;


x=[rays(1,:)./rays(3,:);rays(2,:)./rays(3,:)];


title2=strcat('Complete Distortion Model');

% % fh1 = 2;

%if ishandle(fh1),
%    close(fh1);
%end;
% % % % % % figure; clf;
xd=apply_distortion(x,kc);
px2=Kinit(1,1)*xd(1,:)+Kinit(1,3);
py2=Kinit(2,2)*xd(2,:)+Kinit(2,3);

idIn = find(px2>=0 & px2<=nc & py2 >=0 & py2 <= nr);
idIn = [1:size(px2,2)]';

pixNew = [px2 py2];
dx=px2(idIn)'-px(idIn);
dy=py2(idIn)'-py(idIn);
% Q=quiver(px+1,py+1,dx,dy);
Q=quiver(px(idIn)+0,py(idIn)+0,dx,dy);
hold on;
% % plot(px(idIn),py(idIn),'.r');
plot(Kinit(1,3),Kinit(2,3),'o');
plot((nc-1)/2+1,(nr-1)/2+1,'x');
dr=reshape(sqrt((dx.*dx)+(dy.*dy)),nny,nnx)';
[C,h]=contour(mx,my,dr,'k');
clabel(C,h);
Mean=mean(mean(dr));
Max=max(max(dr));
title(title2);
axis equal;
end