function DeletePNG(inputDir,startTime, endTime)


dirList = dir(fullfile(inputDir,'*____*'));

[~, ind] = sort([dirList.datenum], 'ascend');

for i = 1 : length(dirList)
   tempStr = dirList((i)).name;
   idd = find(ismember(tempStr, '____'));
   [idxCell,idx] = splitIndex2(idd');
   iddd = [];
   for ii = 1 : length(idxCell)
       if length(idxCell{ii}) == 4
           iddd = idxCell{ii};
           break;
       end
   end
   
   
%    time(i,1) = str2double(tempStr(end-14:end-7));
   time(i,1) = str2double(tempStr(iddd(end) + 1 : iddd(end) + 8));
end
id = find(time >= startTime & time <= endTime);
time_ = time(id);
dirList_ = dirList(id,:);
[~, ind] = sort(time_, 'ascend');




for i = 1:length(dirList_)
    delete(fullfile(inputDir,dirList_(ind(i)).name,'*.png'));
% % %     delete(fullfile(inputDir,dirList_(ind(i)).name,'*.mat'));

end



end