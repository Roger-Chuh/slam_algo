function testQuanThoughts8()

pt3d1 = 100.*(rand(100,3)-0.5);
pt3d1(:,3) = abs(pt3d1(:,3));

R = rodrigues([0;0.3;0]);
t = 1000.*rand(3,1);

pt3d2 = (R*pt3d1' + repmat(t, 1, size(pt3d1,1)))';

pt3d11 = (pt3d1 + (R'*repmat(t, 1, size(pt3d1,1)))');

[~, s2] = NormalizeVector(pt3d2);
[~, s1] = NormalizeVector(pt3d11);

tt.a = pt3d2 - pt3d11;
tt.b = -pt3d2 - pt3d11;

[~, s22] = NormalizeVector(pt3d11 + tt.a);
[~, s11] = NormalizeVector(pt3d11 + tt.b);


figure,plot(pt3d1 - pt3d2);
end