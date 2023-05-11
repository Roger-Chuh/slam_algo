function PrepareMap()

close all

corner1 = [121.597471509 31.248058897];
corner2 = [121.597474425 31.248100698];
corner3 = [121.59732666 31.24808036];
corner4 = [121.59732180 31.24803710];
Corner = [corner1; corner2; corner3; corner4];
for i = 1 : size(Corner)
   xz(i,:) = GPS2UTM(Corner(i,:)); 
    
end


yAng = -173.1245  -90 + 180-1.7;
yAng2 = -10;
xz = xz*roty(yAng)';
xz = xz - xz(1,:);
figure,plot(xz(:,1), xz(:,3),'-x');axis equal;grid on;

len1 = norm(xz(2,:));
dir1 = xz(2,:)./norm(xz(2,:));

pt3 = xz(3,:) - len1.*dir1;
xz0 = xz;
xz(4,:) = pt3;
figure,plot(xz(:,1), xz(:,3),'-x');axis equal;grid on;
err1 = norm(xz(2,:) - xz(3,:)) - norm(xz(4,:));
err2 = norm(xz(2,:) - xz(1,:)) - norm(xz(4,:) - xz(3,:));

ang1 = CalcDegree(xz(2,:) - xz(1,:), xz(2,:) - xz(3,:));
ang11 = CalcDegree(xz(4,:) - xz(3,:), xz(4,:) - xz(1,:));

ang2 = CalcDegree(xz(4,:) - xz(3,:), xz(2,:) - xz(3,:));
ang22 = CalcDegree(xz(4,:) - xz(1,:), xz(2,:) - xz(1,:));


xz2 = xz*roty(yAng2)';
figure,plot(xz2(:,1), xz2(:,3),'-x');axis equal;grid on;

end

function xz = GPS2UTM(input)
% lat = 40.0691643333333;
% lon = 116.242161333333;  

lat = input(1);
lon = input(2);

kD2R = pi/180;
ZoneNumber = floor((lon - 1.5) / 3.0) + 1;
  L0 = ZoneNumber * 3.0;

  a = 6378137.0;
  F = 298.257223563;
  f = 1 / F;
  b = a * (1 - f);
  ee = (a * a - b * b) / (a * a);
  e2 = (a * a - b * b) / (b * b);
  n = (a - b) / (a + b); n2 = (n * n); n3 = (n2 * n); n4 = (n2 * n2); n5 = (n4 * n);
  al = (a + b) * (1 + n2 / 4 + n4 / 64) / 2.0;
  bt = -3 * n / 2 + 9 * n3 / 16 - 3 * n5 / 32.0;
  gm = 15 * n2 / 16 - 15 * n4 / 32;
  dt = -35 * n3 / 48 + 105 * n5 / 256;
  ep = 315 * n4 / 512;
  B = lat * kD2R;
  L = lon * kD2R;
  L0 = L0 * kD2R;
  l = L - L0;cl = (cos(B) * l);cl2 = (cl * cl); cl3 = (cl2 * cl); cl4 = (cl2 * cl2); cl5 = (cl4 * cl); cl6 = (cl5 * cl); cl7 = (cl6 * cl); cl8 = (cl4 * cl4);
  lB = al * (B + bt * sin(2 * B) + gm * sin(4 * B) + dt * sin(6 * B) + ep * sin(8 * B));
  t = tan(B); t2 = (t * t); t4 = (t2 * t2); t6 = (t4 * t2);
  Nn = a / sqrt(1 - ee * sin(B) * sin(B));
  yt = e2 * cos(B) * cos(B);
  N = lB;
  N = N + t * Nn * cl2 / 2;
  N = N + t * Nn * cl4 * (5 - t2 + 9 * yt + 4 * yt * yt) / 24;
  N = N + t * Nn * cl6 * (61 - 58 * t2 + t4 + 270 * yt - 330 * t2 * yt) / 720;
  N = N + t * Nn * cl8 * (1385 - 3111 * t2 + 543 * t4 - t6) / 40320;
  E = Nn * cl;
  E = E + Nn * cl3 * (1 - t2 + yt) / 6;
  E = E + Nn * cl5 * (5 - 18 * t2 + t4 + 14 * yt - 58 * t2 * yt) / 120;
  E = E + Nn * cl7 * (61 - 479 * t2 + 179 * t4 - t6) / 5040;
  E = E + 500000;
  N = 0.9996 * N;
  E = 0.9996 * (E - 500000.0) + 500000.0;
  xz = [E 0 N];
end
function theta = CalcDegree(vec1, vec2)
if 0
    theta = min([rad2deg(acos(dot(vec1, vec2)/norm(vec1)/norm(vec2))); rad2deg(acos(dot(vec1, -vec2)/norm(vec1)/norm(vec2)))]);
else
    
    [vec1_norm, ~] = NormalizeVector(vec1);
    [vec2_norm, ~] = NormalizeVector(vec2);
    theta = min([rad2deg(acos(dot(vec1_norm', vec2_norm')')) rad2deg(acos(dot(vec1_norm', -vec2_norm')'))]')';
    
end
end