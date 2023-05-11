function testQuanThoughts()


global TCur_camInit cbcXYZLInit rot_y inverseRot prvBaseLK forceSame doBlur detectCB useFast...
newLK traceCB shortTrace...
resolution winx winy saturation ...
wintx winty spacing boundary boundary_t ...
Nmax thresh levelmin levelmax ThreshQ ...
N_max_feat method...
textureFile b2c angSize probPath inputObj


textureFile = 'C:\Users\rongjiezhu\Desktop\bg16.jpg';

inputObj = 'C:\Users\rongjiezhu\Desktop\÷Ï\01.obj';
obj_test = MR_obj_read(inputObj);

obj_test.V = 6.*obj_test.V;
obj_test.V = (rotx(90)*obj_test.V')';
obj_test.V = (roty(180)*obj_test.V')';
obj_test.V(:,2) = obj_test.V(:,2) + 1400;




rotAng = 10;


intrMat = [554.256258 0 320.5; 0 554.256258 240.5; 0 0 1];

depthThr = [1000 -100000000 1380 -15000000 15000000]; % [minZ minY maxY minX maxX]


[obj1, inlierId1] = procObj2(obj_test, depthThr, eye(3), [0;0;0]);

[~, err1] = NormalizeVector(obj1.V - [-195.7 1340 9915]); [err11,id1] = min(err1);
[~, err2] = NormalizeVector(obj1.V - [-195.7 1340 4322]); [err22,id2] = min(err2);

[~, err3] = NormalizeVector(obj1.V - [-706.2 1340 9915]); [err33,id3] = min(err3);
[~, err4] = NormalizeVector(obj1.V - [-706.2 1340 4322]); [err44,id4] = min(err4);

[~, err5] = NormalizeVector(obj1.V - [825.3 1340 9000]); [err55,id5] = min(err5);
[~, err6] = NormalizeVector(obj1.V - [825.3 1340 4000]); [err66,id6] = min(err6);




idList1 = [id1 id2 id3 id4 id5 id6]';
idList = inlierId1(idList1);


figure,fig_3D3_pair(obj1.V',[]); hold on;
line(obj1.V(idList1(1:2),1), obj1.V(idList1(1:2),2),obj1.V(idList1(1:2),3),'color','r','linewidth',4);
line(obj1.V(idList1(3:4),1), obj1.V(idList1(3:4),2),obj1.V(idList1(3:4),3),'color','g','linewidth',4);
line(obj1.V(idList1(5:6),1), obj1.V(idList1(5:6),2),obj1.V(idList1(5:6),3),'color','b','linewidth',4);


[xyzOrig1, imgCur1, depthMap1, Vcam1] = render_fcn(obj1, eye(3), [0;0;0]);

proj1 = pflat(intrMat*Vcam1(idList1,:)')';
proj1 = proj1(:,1:2);

proj1Vec = (inv(intrMat)*pextend(proj1'))';
proj1Vec(:,3) = 0;
ang1 = CalcDegree(proj1Vec(1,:), proj1Vec(3,:));
ang2 = CalcDegree(proj1Vec(2,:), proj1Vec(4,:));

lineMat1 = [cross([proj1(1,:) 1], [proj1(2,:) 1]); cross([proj1(3,:) 1], [proj1(4,:) 1]); cross([proj1(5,:) 1], [proj1(6,:) 1])];  % [a b c]

lineDir11 = [proj1(1,:) - proj1(2,:) 0];
lineDir12 = [proj1(3,:) - proj1(4,:) 0];
ang3 = CalcDegree(lineDir11, lineDir12);


angErr = ang1 - ang2;

[obj2, inlierId2] = procObj2(obj_test, depthThr, roty(rotAng), [0;0;0]);
idList2 = find(ismember(inlierId2, idList));

idd = inlierId2(idList2);
for i = 1 : length(idd)
   iD(i,1) = find(idd == idList(i)); 
end
idList2 = idList2(iD);
idd2 = inlierId2(idList2);
idd2 - idList

[xyzOrig2, imgCur2, depthMap2, Vcam2] = render_fcn(obj2, roty(rotAng), [0;0;0]);
proj2 = pflat(intrMat*Vcam2(idList2,:)')';
proj2 = proj2(:,1:2);

lineDir21 = [proj2(1,:) - proj2(2,:) 0];
lineDir22 = [proj2(3,:) - proj2(4,:) 0];
ang23 = CalcDegree(lineDir21, lineDir22);

lineMat = [cross([proj2(1,:) 1], [proj2(2,:) 1]); cross([proj2(3,:) 1], [proj2(4,:) 1]); cross([proj2(5,:) 1], [proj2(6,:) 1])];  % [a b c]
[~, s] = NormalizeVector(lineMat(:,1:2));
lineMat = lineMat./repmat(s, 1,3);
[~,~,V] = svd(lineMat);
pt = V(:,end);
pt2 = [pt(1)/pt(3) pt(2)/pt(3)];

ptVec2 = inv(intrMat)*[pt2 1]';
ang = CalcDegree(ptVec2',[0 0 1]);

err = dot(repmat([pt2 1], 3,1)',lineMat');
points2 = lineToBorderPoints(lineMat, size(rgb2gray(imgCur2)));



figure,subplot(1,2,1);imshow(imgCur1);hold on;plot(proj1(1:2,1)', proj1(1:2,2)','r');plot(proj1(3:4,1)', proj1(3:4,2)','g');plot(proj1(5:6,1)', proj1(5:6,2)','b');subplot(1,2,2);imshow(imgCur2);hold on;line(points2(:, [1,3])', points2(:, [2,4])','Color',[1 0 1]);plot(pt2(1), pt2(2),'*r');plot(proj2(1:2,1)', proj2(1:2,2)','r');plot(proj2(3:4,1)', proj2(3:4,2)','g');plot(proj2(5:6,1)', proj2(5:6,2)','b');


nim = ImgTransform(rgb2gray(imgCur1), intrMat, roty(rotAng));

figure,imshowpair(rgb2gray(imgCur2), nim);

figure,fig_3D3_pair(Vcam2', [])

figure,subplot(1,2,1);imshow([imgCur1; imgCur2]);subplot(1,2,2);imshow([depthMap1; depthMap2],[]);

end