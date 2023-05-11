
% % [u,i] = eig(K\H*K);
% % 
% % 
%% golden

R = [cos(pi/10) 0 sin(pi/10); 0 1 0; -sin(pi/10) 0 cos(pi/10)];
T = [2;0;0]; N = [1;0;2];
d = 5; lambda = 4;


HL = lambda*(R+1/d*T*N');

 
%% HL is the original H computed from estimateGromatricTransform multiplied with intrMatL
% % % HL = K\H*K;

[q,z,x] = svd(HL);
temp = sort([z(1,1),z(2,2),z(3,3)],'ascend');

%% this H satisfies 'H = (R+(T/d)*N')'
H = HL./temp(2);


% H = H/eig(H);



[u,w,v] = svd(H'*H);

if det(v) == -1
    u = -u; v = -v;
end




u1 = ((sqrt(1-w(3,3))*v(:,1))+(sqrt(w(1,1)-1)*v(:,3)))/(sqrt(w(1,1)-w(3,3)));
u2 = ((sqrt(1-w(3,3))*v(:,1))-(sqrt(w(1,1)-1)*v(:,3)))/(sqrt(w(1,1)-w(3,3)));

U1 = [v(:,2), u1, cross(v(:,2),u1)];  W1 = [H*v(:,2),H*u1,cross(H*v(:,2),H*u1)];
U2 = [v(:,2), u2, cross(v(:,2),u2)];  W2 = [H*v(:,2),H*u2,cross(H*v(:,2),H*u2)];


R1 = W1*U1'; N1 = cross(v(:,2),u1); T1 = (H-R1)*N1;  %T1 = (H-R1)*N1*d;
R2 = W2*U2'; N2 = cross(v(:,2),u2); T2 = (H-R2)*N2;  %T2 = (H-R2)*N2*d;
R3 = R1; N3 = -N1; T3 = -T1;
R4 = R2; N4 = -N2; T4 = -T2;







