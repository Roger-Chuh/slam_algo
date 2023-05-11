function [M_new, inlierId] = procObj2(M, thr, R, t)

if 1
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

M_new.F(:,1) = nearestIdx1;
M_new.F(:,2) = nearestIdx2;
M_new.F(:,3) = nearestIdx3;

% oldF = [M.F M];
newF = [nearestIdx1 nearestIdx2 nearestIdx3 F2];
oldStack = [M.F M.FT M.FN];
newStack = [M.F(flag,:) M.FT(flag,:) M.FN(flag,:)];

[nearestIdx_F, idxErr_F, idPair_F] = CalcIdx(newStack(:,1:3));
% M_new.FT = nearestIdx_F;


errF = nearestIdx_F - M_new.F;
errIdx = (idPair_F(:,2) - inRangeId2);
errV = M_new.V - M.V(idPair_F(:,2),:);

assert(sum(abs([errF(:); errIdx; errV(:)])) == 0);

[nearestIdx_FT, idxErr_FT, idPair_FT] = CalcIdx(newStack(:,4:6));
M_new.FT = nearestIdx_FT;
M_new.VT = M.VT(idPair_FT(:,2), :);

[nearestIdx_FN, idxErr_FN, idPair_FN] = CalcIdx(newStack(:,7:9));
M_new.FN = nearestIdx_FN;
M_new.VN = M.VN(idPair_FN(:,2), :);

idxStack = [];
inlierId = idPair(:,2);
if 0
    figure,plot(diff(idPair_F'))
    figure,plot(diff(idPair_FT'))
    figure,plot(diff(idPair_FN'))
    
    figure,plot(idPair_F(:,2) - idPair_FT(:,2))
    [nearestIdx_F nearestIdx_FT nearestIdx_FN];
    
    figure,fig_3D3_pair(M_new.V', []);temp = M_new.V(M_new.F(2,:)', :);plot3(temp(:,1), temp(:,2),temp(:,3),'og');
end


return;

if ~newCalc
    M_new.VT = M.VT(idPair(:,2),:);
else
    M_new.VT = M.VT;
end

if ~FN_VN_unchange
    M_new.VN = M.VN(idPair(:,2),:);
end


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
function [nearestIdx, idxErr, idPair] = CalcIdx(F2)
inRangeId2 = sort(unique(F2(:)));
idPair = [[1:length(inRangeId2)]' inRangeId2];
% M_new.V = M.V(idPair(:,2), :);
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
nearestIdx = [nearestIdx1 nearestIdx2 nearestIdx3];
end