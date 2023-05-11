function PoleCircle()
intrMat = [2.0003201001638317e+03 0 9.6889274597167969e+02; 0 2.0003201001638317e+03 5.4882026672363281e+02; 0 0 1];

pole3d_2d = [[-20.1736, 0, -73.8878 1300.94, 0, 1];[-21.0955, 0, -88.2711 1257.09, 0, 1];[-18.5468, 0, -61.1193 1365.25, 0, 1];[4.30781, 0, -126.786 732.299, 0, 1];[-12.4645, 0, -25.6527 1722.24, 0, 1];...
    [12.2089, 0, -84.9235 437.271, 0, 1];[-25.8442, 0, -120.101 1195.39, 0, 1];[4.01227, 0, -17.5042 273.484, 0, 1];[3.21816, 0, -24.1802 510.967, 0, 1]];
% pole3d_2d(:,5) = 10;


figure,hold on;


[cen, r] = CalcCircle(pole3d_2d(1,:), pole3d_2d(2,:), intrMat);



end
function [cen, r] = CalcCircle(x1, x2, intrMat)

x3 = (x1(1:3) + x2(1:3))./2;

vec1 = inv(intrMat)*x1(4:6)';
vec2 = inv(intrMat)*x2(4:6)';
vec1(2) = 0; vec2(2) = 0;
ang = CalcDegree(vec1', vec2');
[~,dist] = NormalizeVector(x3 - x1(1:3));
[~, q] = NormalizeVector  (x1(1:3) - x2(1:3));
r = dist./sind(ang);
cen = [(x3(1) + sqrt(r.^2 - (q./2).^2)*((x1(3) - x2(3))/q)) 0 (x3(3) - sqrt(r.^2 - (q./2).^2)*((x1(1) - x2(1))/q));...
    (x3(1) - sqrt(r.^2 - (q./2).^2)*((x1(3) - x2(3))/q)) 0 (x3(3) + sqrt(r.^2 - (q./2).^2)*((x1(1) - x2(1))/q))];

rectangle('Position',[cen(1, 1) - r, cen(1, 3) - r, 2*r, 2*r],'Curvature',[1,1]),axis equal;hold on;plot(cen(1,1), cen(1,3),'*r');
rectangle('Position',[cen(2, 1) - r, cen(2, 3) - r, 2*r, 2*r],'Curvature',[1,1]),axis equal;hold on;plot(cen(2,1), cen(2,3),'*r');
plot(x1(1), x1(3),'*g');plot(x2(1), x2(3),'*b');

end