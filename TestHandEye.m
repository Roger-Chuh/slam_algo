function TestHandEye()

c2b = [rotz(1)*roty(2)*rotx(3) [-0;0;0];0 0 0 1];
b2c = inv(c2b);

rotVecB = rand(3,3);
transVecB = 100.*(rand(3,size(rotVecB,2))-0.5);

rotVecB(:,1) = [0;0;0];
transVecB(:,1) = [0;0;0];

transVecB = zeros(size(transVecB));
for i = 1: size(rotVecB,2)
    tb2w = [rodrigues(rotVecB(:,i)) transVecB(:,i);0 0 0 1];
    Tb2w(:,:,i) = tb2w;
end


b2w0 = Tb2w(:,:,1);
Tc2w = eye(4);
c2w0 = Tc2w;
for i = 1 : size(Tb2w,3) - 1
    
    delta_b_a2b = inv(Tb2w(:,:,i+1)) * b2w0;
    delta_c_a2b = b2c * delta_b_a2b * c2b;
    Tc2w(:,:,i + 1) = c2w0 * inv(delta_c_a2b);
end


for i = 1 : size(Tb2w,3)
%     Tc2w(:,:,i) = inv(Tc2w(:,:,i));
    Tw2b(:,:,i) = inv(Tb2w(:,:,i));
    
    Tw2b_eye1(:,:,i) = inv(Tw2b(:,:,1)) * Tw2b(:,:,i);
    Tw2b_eye2(:,:,i) = Tw2b(:,:,i) * inv(Tw2b(:,:,1));
    Tw2c(:,:,i) = inv(Tc2w(:,:,i));
end


for i = 1 :size(Tb2w,3)
   
   Tw2c_comp_temp1 = b2c*Tw2b_eye1(:,:,i);
   Tw2c_comp_temp2 = b2c*Tw2b_eye2(:,:,i);
   Tw2c_comp_temp = b2c*Tw2b(:,:,i);
   Tw2c_temp = Tw2c(:,:,i);
end



norm(rodrigues(Tw2b(1:3,1:3,3))) - norm(rodrigues(Tc2w(1:3,1:3,3)))


a1=b2c*Tw2b(:,:,1);
id = 2;
err = rodrigues(inv(Tc2w(1:3,1:3,id))*(a1(1:3,1:3)) *           inv(  b2c(1:3,1:3)*Tw2b(1:3,1:3,id) )    );

norm(rodrigues(Tc2w(1:3,1:3, id))) - norm(rodrigues(Tw2b(1:3,1:3, id)))

% gHc = handEye(tB, tC);
% [gHc] = handEye(Tc2w, Tb2w);
[b2c_solve, err] = TSAIleastSquareCalibration(Tc2w, Tw2b);
% X = daniilidis(Tc2w,Tb2w);
end