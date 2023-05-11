function [leftImgLst, rightImgLst, cbcXYL, cbcXYR] = CaptureAndDetect(inputDir, calibFuncDir,outputDir,DXX,modeE,blbl,filename,swapLR)
baseDir = pwd;

Dx = str2double(DXX);

tempDir = fullfile(calibFuncDir,'Temp');
MakeDirIfMissing(tempDir);

outputDirPath = outputDir;

MakeDirIfMissing(outputDirPath);  % creat corresponding folder to store images used for calibration


tempSrcDir = fullfile(inputDir,'Temp2');
MakeDirIfMissing(tempSrcDir);
tempInputDir = fullfile(inputDir,'Temp3');
MakeDirIfMissing(tempInputDir);

for wreeqb = 1:2
    pause(0.01);
while length(dir(fullfile(tempSrcDir,strcat('*.',filename(end-2:end))))) ~= 0
delete(fullfile(tempSrcDir,strcat('*.',filename(end-2:end)))); % delete previously captured images
end
end

for erq = 1:3
    pause(0.01);
if length(dir(fullfile(inputDir,strcat('*.',filename(end-2:end))))) > 0 %%%|| length(dir(fullfile(inputDir,strcat('*.',filename(end-2:end)))))>length(capedImgLst)
srcInfo = dir(fullfile(inputDir,strcat('*.',filename(end-2:end))));
end
end




dircell=struct2cell(srcInfo);
aaAa = dircell(2,:)'; %% sort files by time
sss = datetime(aaAa,'Format','dd-MMM-y HH:mm:ss');
[B,I] = sort(sss);
tempInfo = cell(length(I),1);
for fg = 1:length(I)
    tempInfo{fg,1} = srcInfo(I(fg)).name;
end
for fgg = 1:length(I)
   srcInfo(fgg).name = tempInfo{fgg}; 
end

for aghfjwb = 1:3
copyfile(fullfile(inputDir,srcInfo(1).name), tempInputDir); 
end

for fvewuj = 1:5
    try
        movefile(fullfile(inputDir,srcInfo(1).name), tempSrcDir);
    catch
        dbbhjww = 1;
    end
end
nameLR = srcInfo(1).name;
cropDir = fullfile(inputDir,'Temp');



MakeDirIfMissing(cropDir);
for fg = 1:5
    pause(0.01);
while length(dir(fullfile(cropDir,strcat('*.',filename(end-2:end))))) ~= 0
delete(fullfile(cropDir,strcat('*.',filename(end-2:end))))
end
end
config.dX = Dx; config.dY = Dx;
for i = 1 : length(dir(fullfile(tempSrcDir,strcat('*.',filename(end-2:end)))))
    
   
    
    [cbImgListL, cbImgListR] = PreprocZedImgPair1(tempSrcDir, cropDir,filename,swapLR);
% %     [isblL, blL] = isBlur(imread(cbImgListL{1}),blbl);
% %     [isblR, blR] = isBlur(imread(cbImgListR{1}),blbl);
    [isblL, blL] = isBlur2(imread(cbImgListL{1}),blbl);
    [isblR, blR] = isBlur2(imread(cbImgListR{1}),blbl);


    if isblL + isblR == 0
    [goodIdL,cbcXYLLL] = CbCheckSingle1(cbImgListL,calibFuncDir, modeE,config);
    [goodIdR,cbcXYRRR] = CbCheckSingle1(cbImgListR, calibFuncDir,modeE,config);
    cd(baseDir);

    nameL = cbImgListL{1}(length(cropDir)+2:end);
    nameR = cbImgListR{1}(length(cropDir)+2:end);
    if ~isempty(goodIdL) && ~isempty(goodIdR)
        movefile(cbImgListL{1}, outputDirPath); movefile(cbImgListR{1}, outputDirPath);
        leftImgLst = fullfile(outputDirPath,nameL);
        rightImgLst = fullfile(outputDirPath,nameR);
        outInfo = dir(fullfile(outputDirPath,strcat('*.',filename(end-2:end))));
        cbcXYL = cbcXYLLL;
        cbcXYR = cbcXYRRR;

    else
        leftImgLst = []; rightImgLst = []; cbcXYL = []; cbcXYR = [];
        fprintf('\n\n============================= capture failed, no corner ======\n\n');
    end
    
    else
        
        leftImgLst = []; rightImgLst = []; cbcXYL = []; cbcXYR = [];
        fprintf(sprintf('\n\n============================= capture failed, too blur %f and %f ======\n\n',blL,blR));
    end
    

    
end


cd(baseDir);
for sd = 1:3
pause(0.01)
while length(dir(fullfile(inputDir,nameLR))) ~= 0
delete(fullfile(inputDir,nameLR)); % delete previously captured images
end

while length(dir(fullfile(tempSrcDir,strcat('*.',filename(end-2:end))))) ~= 0
delete(fullfile(tempSrcDir,strcat('*.',filename(end-2:end)))); % delete previously captured images
end

end




end