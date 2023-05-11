function CropImgPair(inputDir,outputDir,varargin)

if (nargin == 2)
    switchLR = 1;
    sufix = 'png';
elseif (nargin == 3)
    switchLR = varargin{1};
    sufix = 'png';
elseif (nargin == 4)
    switchLR = varargin{1};
    sufix = varargin{2};
else
    error('Too many input arguments');
end


dirInfo = dir(fullfile(inputDir,strcat('*.',sufix)));
frameNum = length(dirInfo)/1;

MakeDirIfMissing(outputDir);

for i = 1 : frameNum
    img = imread(fullfile(inputDir,dirInfo(i).name));
    if switchLR == 1
        imgL = img(:,size(img,2)/2 + 1:end,:);
        imgR = img(:,1:size(img,2)/2,:);
    else
        imgR = img(:,size(img,2)/2 + 1:end,:);
        imgL = img(:,1:size(img,2)/2,:);
    end
    
    imwrite(imgL, fullfile(outputDir, sprintf(strcat('cropL_%04d.',sufix), i)));
    imwrite(imgR, fullfile(outputDir, sprintf(strcat('cropR_%04d.',sufix), i)));
    
    
end






end