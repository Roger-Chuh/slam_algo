function TestRogThoughts3()


b2c = rodrigues([0.1;0.2;0.3]);


anglist = [10:2:20];
anglist = anglist + rand(1, length(anglist));



for i = 1 : length(anglist)    
   rotB(:,:,i) = roty(anglist(i));
   rotC(:,:,i) = b2c*roty(anglist(i));
   
   angB(i,1) = norm(rodrigues(rotB(:,:,i)));
   angC(i,1) = norm(rodrigues(rotC(:,:,i)));
end






% b2c_init = b2c;
%     for i = 1 : size(rt,3)
%         camK2C(:,:,i) = inv(rt(1:3,1:3,i));
%         camK2C_comp(:,:,i) = b2c_init(1:3,1:3)*roty(baseRotAng +0 + deltAng(i))';
%         err(i,:) = rad2deg(norm(rodrigues(camK2C(:,:,i)'*camK2C_comp(:,:,i))));
%     end





% figure,plot([diff(angB) - diff(angC)])

for i = 1 : length(anglist)-1
   rotB(:,:,i) = roty(anglist(i));
   rotC(:,:,i) = b2c*roty(anglist(i));
   
   rotB_p2c = roty(anglist(i+1) - anglist(i));
   
   rotC_p2c_comp = b2c*rotB_p2c*b2c';
   
   deltaCam1 = (rotC(:,:,i+1) * inv(rotC(:,:,i)));
   deltaCam_comp = b2c*(rotB(:,:,i+1) * inv(rotB(:,:,i))) * inv(b2c);
   
   
   deltaCam2 = inv(rotC(:,:,i+1)) * (rotC(:,:,i));
   
   
   angBB(i,1) = norm(rodrigues(rotB(:,:,i+1) \ rotB(:,:,i)));
   angCC(i,1) = norm(rodrigues(rotC(:,:,i+1) \ rotC(:,:,i)));
end




% figure,plot([diff(angBB) - diff(angCC)])

figure,plot(angB - angB(1));hold on;plot([0;cumsum(angBB)]);plot([0;cumsum(angCC)]); % plot(angC - angC(1));
% figure,plot(angC - angC(1));hold on;plot([0;cumsum(angCC)])
end