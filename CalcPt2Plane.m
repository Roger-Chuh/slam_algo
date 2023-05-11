function [intersect,dist] = CalcPt2Plane(lineN,ptOnline,ptIso)

% calc dist from ptIso to line, whose direction vector is lineN, and has a
% pointOnline

%[v1 v2 v3] = lineN;
%[vp1 vp2 vp3] = lineN, means the normal vector of plane through ptIso;



m1 = ptOnline(1);
m2 = ptOnline(2);
m3 = ptOnline(3);
v1 = lineN(1); vp1 = v1;
v2 = lineN(2); vp2 = v2;
v3 = lineN(3); vp3 = v3;
n1 = ptIso(1); n2 = ptIso(2); n3 = ptIso(3);

t = ((n1 - m1)*vp1+(n2 - m2)*vp2+(n3 - m3)*vp3) / (vp1* v1+ vp2* v2+ vp3* v3);


x = m1+ v1 * t;
y = m2+ v2 * t;
z = m3+ v3 * t;

intersect = [x y z];

dist = norm(intersect-ptIso);

end