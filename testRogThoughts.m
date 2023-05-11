function testRogThoughts()
xz2 = [-3.7167         0    1.0000
    -3.6796   -2.8078    1.0000
    0.1999   -2.8078    1.0000
    3.4821   -2.8078    1.0000
    3.4477   -0.0212    1.0000
    0.2666    0.0534    1.0000];
xz2(:,2) = xz2(:,2) + 10;

xz2 = [xz2(:,1) xz2(:,3) xz2(:,2)];


rotVec = [0.1; 0.2; 0.3];
% rotVec = [0.1; 0; 0];
rotAxis = rotVec./norm(rotVec);
rMat = rodrigues(rotVec);

intrMat = 1000.*[1.6755         0    0.9676
    0    1.6755    0.5475
    0         0    0.0010];




intrMat = eye(3);
intrMat(1,1) = 2.0003201001638317e+03;
intrMat(1,3) = 9.6889274597167969e+02;
intrMat(2,2) = 2.0003201001638317e+03;
intrMat(2,3) = 5.4882026672363281e+02;


[pt2d, pt3d] = TransformAndProject(xz2, intrMat, rMat, [1;2;3]);


pt2dMetric = (inv(intrMat)*pextend(pt2d'))';



xVec = cross([0; 1; 0], rMat(:,2));
xVec = xVec./norm(xVec);
zVec = cross(xVec, rMat(:,2));
zVec = zVec./norm(zVec);

rMatNew = [xVec'; rMat(:,2)'; zVec'];
rMatNew2 = rMat';

pt3dNew = (rMatNew*pt3d')';
pt3dNew2 = (rMatNew2*pt3d')';

pixRot = Orig2Rect(pt2d, intrMat, intrMat, rMatNew',zeros(5,1));


[H,Hnorm,inv_Hnorm] = compute_homography(pt3d', pt2dMetric');


[sol1, H1] = DecomposeHomoGraphyMat((H), eye(3),eye(3));
for e = 1 : 4
    rTmp = sol1(1:3,1:3,e);
    tTmp = sol1(1:3,4,e);
    projErr1(e,1) = ProjectErr3dTo2d(pt2d, pt3d, intrMat, rTmp, tTmp);
end


end