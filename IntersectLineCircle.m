function [pt1, pt2] = IntersectLineCircle(cen,Line,r)
% cen: N x 2
% line: N x 3, a*x + b*y + c = 0

% tansform line to 'y = k*x + b'

k = -Line(:,1)./Line(:,2);
b = -Line(:,3)./Line(:,2);


A = k.^2 + 1;
B = 2.*k.*b - 2.*cen(:,2).*k - 2.*cen(:,1);
C = cen(:,1).^2 + cen(:,2).^2 - repmat(r.^2,size(cen,1),1) + b.^2 - 2.*cen(:,2).*b;
X1 = (-B + sqrt(B.^2 - 4.*A.*C))./2./A;
X2 = (-B - sqrt(B.^2 - 4.*A.*C))./2./A;

Y1 = k.*X1 + b;
Y2 = k.*X2 + b;

pt1 = real([X1 Y1]);
pt2 = real([X2 Y2]);

if 0
    
    [~,r1] = NormalizeVector([X1 Y1]-cen);
    [~,r2] = NormalizeVector([X2 Y2]-cen);
%     [Line(1,:)*[X1(1);Y1(1);1] Line*[estIntersectPt;1]];
    for k = 1 : size(Line,1)
        err1(k,:) = dist2LinePix2([X1(k) Y1(k)],Line(k,:));
        err2(k,:) = dist2LinePix2([X2(k) Y2(k)],Line(k,:));
    end
    
figure,imshow(ones(480,640));hold on;
color = rand(size(Line,1),3);
for i = 1 : size(Line,1)
%     plot(PT{i}(:,1),PT{i}(:,2), '.', 'Color', color(i,:));
    points = lineToBorderPoints(Line(i,:), [480 640]);
    line(points(:, [1,3])', points(:, [2,4])','Color', color(i,:));
end
plot(X1,Y1,'.g');plot(X2,Y2,'or');plot(cen(:,1),cen(:,2),'+b');


end









end