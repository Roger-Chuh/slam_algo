function AnimatePath(vs, bs, startt, endd)

dirLen = 0.05;
vvec = vs(startt:endd,:);
bvec = bs(startt:endd,:);
% for i = 1 : size(vs,1)
cnt = 1;
for i = startt : endd
figure(413),clf;plot([vs(startt:i,1)],[vs(startt:i,2)],'.r');hold on;axis equal;plot([vs(startt:i,1)],[vs(startt:i,2)],'.b');
plot([vs(i,1) vs(i,1)-dirLen*sin(bs(i,3))],[vs(i,2) vs(i,2)+dirLen*cos(bs(i,3))],'-k','LineWidth',1);
plot([vs(i,1) vs(i,1)-dirLen*sin(vs(i,3))],[vs(i,2) vs(i,2)+dirLen*cos(vs(i,3))],'-g','LineWidth',1);
legend('visual','body','body','visual');title(num2str(i));
figure(414);clf;plot(rad2deg(vvec(:,3) - bvec(:,3)));hold on;plot(cnt,rad2deg(vvec(cnt,3)-bvec(cnt,3)),'o');
cnt = cnt + 1;
% pause(0.1)
drawnow;
end

end