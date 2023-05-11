function [sol, H,HL] = DecomposeHomoGraphyMat(h, intrMat, intrMatt)

%    a   =   h*b;                          pix coord   Ka*A = h*Kb*B;
%    A   =   Hh*B;                         mm coord    A = inv(Ka)*h*Kb*B
%  HL*A  =   B;                            mm coord
%    H   =   HL./temp(2) = (R+(T/d)*N')
% intrMat: left intr mat; intrMatt: right intr mat



%% 
Hh = intrMat\h*intrMatt;   


HL = inv(Hh); 
% HL = Hh;
 
%% HL is the original H computed from estimateGromatricTransform multiplied with intrMatL
% % % HL = K\H*K;

[q,z,x] = svd(HL);
temp = sort([z(1,1),z(2,2),z(3,3)],'ascend');

%% this H satisfies   ******  H = (R+(T/d)*N')  *****
H = HL./temp(2);
% % % % H = HL;



[u,w,v] = svd(H'*H);

if det(v) == -1
    u = -u; v = -v;
end




u1 = ((sqrt(1-w(3,3))*v(:,1))+(sqrt(w(1,1)-1)*v(:,3)))/(sqrt(w(1,1)-w(3,3)));
u2 = ((sqrt(1-w(3,3))*v(:,1))-(sqrt(w(1,1)-1)*v(:,3)))/(sqrt(w(1,1)-w(3,3)));

U1 = [v(:,2), u1, cross(v(:,2),u1)];  W1 = [H*v(:,2),H*u1,cross(H*v(:,2),H*u1)];
U2 = [v(:,2), u2, cross(v(:,2),u2)];  W2 = [H*v(:,2),H*u2,cross(H*v(:,2),H*u2)];


R1 = W1*U1'; N1 = cross(v(:,2),u1); T1 = (H-R1)*N1;    %T1 = (H-R1)*N1*d;
R2 = W2*U2'; N2 = cross(v(:,2),u2); T2 = (H-R2)*N2 ;   %T2 = (H-R2)*N2*d;
R3 = R1; N3 = -N1; T3 = -T1;
R4 = R2; N4 = -N2; T4 = -T2 ;  % N1,N2 should be positive




sol(:,:,1) = [R1 T1 N1];
sol(:,:,2) = [R2 T2 N2];
sol(:,:,3) = [R3 T3 N3];
sol(:,:,4) = [R4 T4 N4];




rVec1 = rodrigues(R1);
rVec2 = rodrigues(R2);
rVec3 = rodrigues(R3);
rVec4 = rodrigues(R4);


HVerify = (R1+(T1)*N1').*temp(2);
end







