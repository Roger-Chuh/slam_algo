function err = dist2LinePix2(VanPtZ,VanLine)


b = VanPtZ(2) - VanLine(2)/VanLine(1)*VanPtZ(1);

% b = VanPt(2) - VanLine(2)/VanLine(1)*VanPt(1);
linePassingVanPt = [VanLine(2)/VanLine(1) -1 b];
linePassingVanPt = [linePassingVanPt(1)/linePassingVanPt(3) linePassingVanPt(2)/linePassingVanPt(3) 1];
intersectVanPt = cross(VanLine,linePassingVanPt);
intersectVanPt = [intersectVanPt(1)/intersectVanPt(3);intersectVanPt(2)/intersectVanPt(3)];

err = norm(VanPtZ - intersectVanPt');
end