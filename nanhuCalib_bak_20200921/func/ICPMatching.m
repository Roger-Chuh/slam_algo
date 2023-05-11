function [R, t]=ICPMatching(data1, data2)
% ICP�A���S���Y���ɂ��A���i�x�N�g���Ɖ�]�s��̌v�Z�����{����֐�
% data1 = [x(t)1 x(t)2 x(t)3 ...]
% data2 = [x(t+1)1 x(t+1)2 x(t+1)3 ...]
% x=[x y z]'

%ICP �p�����[�^
preError=0;%��O�̃C�^���[�V������error�l
dError=1000;%�G���[�l�̍���
EPS=0.0001;%��������l
maxIter=100;%�ő�C�^���[�V������
count=0;%���[�v�J�E���^

R=eye(2);%��]�s��
t=zeros(2,1);%���i�x�N�g��

while ~(dError < EPS)
	count=count+1;
    
    [ii, error]=FindNearestPoint(data1, data2);%�ŋߖT�_�T��
    [R1, t1]=SVDMotionEstimation(data1, data2, ii);%���ْl�����ɂ��ړ��ʐ���
    %�v�Z����R��t�œ_�Q��R��t�̒l���X�V
    data2=R1*data2;
    data2=[data2(1,:)+t1(1) ; data2(2,:)+t1(2)];
    R = R1*R;
    t = R1*t + t1; 
    
    dError=abs(preError-error);%�G���[�̉��P��
    preError=error;%��O�̃G���[�̑��a�l��ۑ�
    
    if count > maxIter %�������Ȃ�����
        disp('Max Iteration');return;
    end
end
disp(['Convergence:',num2str(count)]);
end

function [index, error]=FindNearestPoint(data1, data2)
%data2�ɑ΂���data1�̍ŋߖT�_�̃C���f�b�N�X���v�Z����֐�
m1=size(data1,2);
m2=size(data2,2);
index=[];
error=0;

for i=1:m1
    dx=(data2-repmat(data1(:,i),1,m2));
    dist=sqrt(dx(1,:).^2+dx(2,:).^2);
    [dist, ii]=min(dist);
    index=[index; ii];
    error=error+dist;
end
end
function [R, t]=SVDMotionEstimation(data1, data2, index)
%���ْl����@�ɂ����i�x�N�g���ƁA��]�s��̌v�Z

%�e�_�Q�̏d�S�̌v�Z
M = data1; 
mm = mean(M,2);
S = data2(:,index);
ms = mean(S,2); 

%�e�_�Q���d�S���S�̍��W�n�ɕϊ�
Sshifted = [S(1,:)-ms(1); S(2,:)-ms(2);];
Mshifted = [M(1,:)-mm(1); M(2,:)-mm(2);];

W = Sshifted*Mshifted';
[U,A,V] = svd(W);%���ْl����

R = (U*V')';%��]�s��̌v�Z
t = mm - R*ms;%���i�x�N�g���̌v�Z
end
function radian = toRadian(degree)
% degree to radian
radian = degree/180*pi;
end