function testQuanThoughts4()

inputDir = 'D:\Temp\20200729';
dirInfo = dir(fullfile(inputDir, '*.txt'));

for i = 1 : length(dirInfo)
    data = readFile(fullfile(inputDir, dirInfo(i).name));
    
    
    for j = 1 : length(data)
        
       if size(data{j},1) > 3
           
       end
        
    end
    
    
end



end

function data = readFile(accFilePath)

[configFileFid, errMsg] = fopen(accFilePath);
if (configFileFid < 0)
    error('Cannot open %s for read: %s', accFilePath, errMsg);
end
% gVecBuf = [];
% inReadgVec = false;
% flagg=0;

cnt = 1;
data{cnt} = [];
while ~feof(configFileFid)
    lineBuf = strtrim(fgetl(configFileFid));
    if length(lineBuf) > 3
       
        data{cnt,1} = [data{cnt}; str2double(strsplit(lineBuf, ' '))];
    else
        cnt = cnt + 1;
        data{cnt,1} = [];

    end

end
fclose(configFileFid);
end