aaa = unique(cell2mat(vsl.keyProbZ(:,1)));
HIST = {}; HIST_cnt = [];
% frmCnt = 1;
cnttt1 = 1;
for zxc = 1 : length(aaa)
    tmp = find(cell2mat(vsl.keyProbZ(:,1)) == aaa(zxc));
    HIST{zxc,1} = cell2mat(vsl.keyProbZ(tmp,22));
    try
        HIST_cnt(cnttt1,:) = HIST{zxc,1}(frmCnt,:);
        cnttt1 = cnttt1 + 1;
    catch
        szfkj = 1;
    end
end




