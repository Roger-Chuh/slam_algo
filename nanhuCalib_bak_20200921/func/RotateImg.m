 % inputDir = 'D:\temp5\20171010\1\1';
function RotateImg(inputDir, varargin)

global cfg
if (nargin == 1)
    nameStr = 'imgR';
elseif (nargin == 2)
    nameStr = varargin{1};
else
    error('Too many input arguments');
end

dirInfo = dir(fullfile(inputDir,'*.png'));
if length(dirInfo) == 0
    dirInfo = dir(fullfile(inputDir,'*.bmp'));
end
if length(dirInfo) == 0
    dirInfo = dir(fullfile(inputDir,'*.jpg'));
end
if 0
    dircell=struct2cell(dirInfo);
    aaAa = dircell(2,:)';
    sss = datetime(aaAa,'Format','dd-MMM-y HH:mm:ss');
    [B,I] = sort(sss);
    tempInfo = cell(length(I),1);
    for fg = 1:length(I)
        tempInfo{fg,1} = dirInfo(I(fg)).name;
    end
    for fgg = 1:length(I)
        dirInfo(fgg).name = tempInfo{fgg};
    end
end


frameNum = length(dirInfo); 1/2;  %/2;
workDir = pwd;
cd (inputDir)
for i = 1:    frameNum  %length(dirInfo)
    %    img = imread(fullfile(inputDir,dirInfo(i).name));
    %    name = str2double(dirInfo(i).name(5:end-4));
    %    name = name.*100; name = round(name);
    %    name = name*10000000 + 1000000000000000000;
    %    imwrite(img,strcat(inputDir,'\',sprintf('%19.0f',name),'.png'));
    
    
    
    %    eval(['!rename' 32 dirInfo(i).name 32 sprintf('t1_%04d.png', i)]);
    img = imread(fullfile(inputDir, dirInfo(i).name));
    imwrite(imrotate(img, 90), fullfile(inputDir, sprintf(strcat(nameStr, '_%04d.png'), i)));
%     eval(['!rename' 32 dirInfo(i).name 32 sprintf(strcat(nameStr, '_%04d.png'), i)]);
    %    eval(['!rename' 32 dirInfo(i + frameNum).name 32 sprintf('imgR_%04d.png', i)]);
    
    %     eval(['!rename' 32 dirInfo(i).name 32 sprintf('rectR_%04d.png', i)]);
    
    
end
cd (workDir)
end