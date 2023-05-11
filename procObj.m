function [M_new, inlierId] = procObj(M, thr, R, t)

if 0
    FN_VN_unchange = true;
    
    newCalc = true;
else
    
    FN_VN_unchange = false;
    
    newCalc = false;
end
pt3d = (R* M.V' + repmat(t, 1, size(M.V,1)))';

if length(thr) == 1
    inRangeId = find(pt3d(:,3) > thr);
else
    inRangeId = find(pt3d(:,3) > thr(1) & pt3d(:,2) > thr(2) & pt3d(:,2) < thr(3) & pt3d(:,1) > thr(4) & pt3d(:,1) < thr(5));
end
flag1 = (ismember(M.F(:,1), inRangeId));
flag2 = (ismember(M.F(:,2), inRangeId));
flag3 = (ismember(M.F(:,3), inRangeId));
flag = find(flag1 & flag2 & flag3);

F2 = M.F(flag, :);
inRangeId2 = sort(unique(F2(:)));
idPair = [[1:length(inRangeId2)]' inRangeId2];
M_new.V = M.V(idPair(:,2), :);
row1 = F2(:,1); row2 = F2(:,2); row3 = F2(:,3);
[idComm,id1,id2] = intersect(idPair(:,2), row1);
[~,id11] = sort(id1);
id1 = id1(id11);
id2 = id2(id11);
err = idPair(id1,2) - row1(id2);

nearestIdx1 = knnsearch(idPair(:,2), row1, 'NSMethod', 'kdtree');
nearestIdx2 = knnsearch(idPair(:,2), row2, 'NSMethod', 'kdtree');
nearestIdx3 = knnsearch(idPair(:,2), row3, 'NSMethod', 'kdtree');
idxErr = row1 - idPair(nearestIdx1,2);
if ~newCalc
    M_new.VT = M.VT(idPair(:,2),:);
else
    M_new.VT = M.VT;
end

if ~FN_VN_unchange
    M_new.VN = M.VN(idPair(:,2),:);
end

M_new.F(:,1) = nearestIdx1;
M_new.F(:,2) = nearestIdx2;
M_new.F(:,3) = nearestIdx3;

if ~FN_VN_unchange
    M_new.FN = [nearestIdx1 nearestIdx2 nearestIdx3];
end

if ~newCalc
    M_new.FT = [nearestIdx1 nearestIdx2 nearestIdx3];
else
    M_new.FT = M.FT; % [nearestIdx1 nearestIdx2 nearestIdx3];
end

if 0
    %     figure,plot3(M_new.V(:,1),M_new.V(:,2),M_new.V(:,3),'.r');hold on;axis equal;temp = M_new.V(M_new.F(230,:)', :);plot3(temp(:,1), temp(:,2),temp(:,3),'og')
    
    figure,fig_3D3_pair(M_new.V', []);temp = M_new.V(M_new.F(2,:)', :);plot3(temp(:,1), temp(:,2),temp(:,3),'og');
end

inlierId = idPair(:,2);
end