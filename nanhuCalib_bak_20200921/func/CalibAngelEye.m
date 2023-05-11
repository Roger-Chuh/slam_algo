function CalibAngelEye(filename,SWAPLR,MODE,inputDirr, resultPrefixx, calibFuncDirr, DXX,imgNumm,imgNumUsedd,timess,Nn,leastNumm,medianPixErr,meanPixErr,dltLenn,blblbl,repErrr)


baseDir = pwd;
inputDir = fullfile(baseDir,inputDirr);
resultPrefix = fullfile(baseDir,resultPrefixx);
calibFuncDir = fullfile(baseDir,calibFuncDirr);


meanPixerr = str2double(meanPixErr);
medianPixerr = str2double(medianPixErr);
swapLR = str2double(SWAPLR);
Mode = str2double(MODE);
blbl = str2double(blblbl);
repErr = str2double(repErrr);


DX = str2double(DXX);
dltLen = str2double(dltLenn);
imgNum = str2double(imgNumm);
imgNumUsed = str2double(imgNumUsedd);
times = str2double(timess);
leastNum = str2double(leastNumm);
N = str2double(Nn);



MakeDirIfMissing(resultPrefix);
cropPrefix = fullfile(resultPrefix,'pair');
paraPrefix = fullfile(resultPrefix,'result');
MakeDirIfMissing(calibFuncDir);
for qw = 1:3
    pause(0.01);
while length(dir(fullfile(cropPrefix,strcat('*.',filename(end-2:end))))) ~= 0
    delete(fullfile(cropPrefix,strcat('*.',filename(end-2:end))));
    delete(fullfile(paraPrefix,'*'));
end
end


leftLst = cell(imgNum,1); rightLst = cell(1,imgNum);
cbcL = cell(imgNum,1); cbcR = cell(imgNum,1);
k = 1;
tempInputDir = fullfile(inputDir,'Temp3');
MakeDirIfMissing(tempInputDir);

for qw = 1:3
    pause(0.01);
while length(dir(fullfile(tempInputDir,strcat('*.',filename(end-2:end))))) ~= 0
    delete(fullfile(tempInputDir,strcat('*.',filename(end-2:end))));
end


end

if Mode == 1
    fprintf('\n\n======== using vertical checker board ===========\n\n');
else
    fprintf('\n\n======== using horizontal checker board ===========\n\n');
end

for qwdsgh = 1:3
    pause(0.01);
delete(fullfile(cropPrefix,strcat('*.',filename(end-2:end)))); % delete previously cropped images
delete(fullfile(inputDir,strcat('*.',filename(end-2:end)))); % delete previously cropped images
end


import java.awt.Robot;
import java.awt.event.*;
robot=Robot;


while length(dir(fullfile(tempInputDir,strcat('*.',filename(end-2:end))))) < imgNum

    if 0
    robot.keyPress(double(' '));
    robot.keyRelease(double(' '));
    end
    
    
try
  [leftImgLst, rightImgLst, cbcXYL, cbcXYR] = CaptureAndDetect(inputDir, calibFuncDir,cropPrefix,DXX,Mode,blbl,filename,swapLR);
  if ~isempty(leftImgLst)
    if k ~=1
       tp = cbcL(1:k-1);
       projErr = zeros(length(tp),1);
       for hj = 1:length(tp)
           projErr(hj) = norm(mean(abs(cbcXYL{1}' -tp{hj}')));
       end
       if min(projErr) > repErr
       leftLst{k} = leftImgLst; rightLst{k} = rightImgLst; cbcL(k) = cbcXYL; cbcR(k) = cbcXYR;
       k = k+1;
       else
           delete(leftImgLst); delete(rightImgLst);
           fprintf(sprintf('\n\n=========================================== too similar, %f =========\n\n',(min(projErr))));
       end
    else
        leftLst{k} = leftImgLst; rightLst{k} = rightImgLst; cbcL(k) = cbcXYL; cbcR(k) = cbcXYR;
        k = k+1;
    end
  end
   fprintf(sprintf('\n\n==== %d images captured ====\n\n',length(dir(fullfile(cropPrefix,strcat('*.',filename(end-2:end)))))/2));


  
catch
    awdcw = 1;
end


end

fprintf(sprintf('\ncb size is %f, imgNum is %d, imgNumUsed(pool) is %d, loop times is %d, calibImgUsed is %d, leastNum is %d\n',DX,imgNum,imgNumUsed,times,N,leastNum));

   for iqw = 1:3   
    pause(0.01);
    delete(fullfile(inputDir,'check.txt')); % delete  previously generated 'check.txt'
   end
    
     fprintf('\n\n');
        fprintf('=========== Calibrating Glass ============');
        fprintf('\n\n\n');
    
    try
    [paraDir, cropDir, calibFrameNum, good, stereoParam,config,cbImgLstLLL2check,cbImgLstRRR2check] = CalibWorkFlow(cropPrefix,paraPrefix,calibFuncDir,DX,imgNumUsed,times,N, leastNum, leftLst, rightLst, cbcL, cbcR,Mode,imgNum);

    catch
        good = 0;
        fprintf('\n====== something is wrong in calibration === too few valid images =====\n\n');
    end
    if good == 0
       
       info = [good];
       MakeDirIfMissing((paraPrefix));
       paraDir = (paraPrefix);
       cd(baseDir);
       save(fullfile(paraDir,'calibResult.txt'),'info','-ascii'); 
% %        exit(0);

    else
    
    Good = 0; LENS = zeros(4*length(cbImgLstLLL2check),1);
    for j = 1 : length(cbImgLstLLL2check)
        try
            [isGood, Lens] = VerifyCalib(cbImgLstLLL2check,cbImgLstRRR2check,stereoParam,config,j, cropDir,calibFuncDir,medianPixerr,meanPixerr,dltLen,Mode);
        catch
            isGood = 0;
            fprintf('\n======= something is wrong in verifying ========\n\n');
        end
        Good = Good + isGood;
        LENS(4*j-3:4*j,1) = Lens;
    end
    if Good == length(cbImgLstLLL2check) && length(cbImgLstLLL2check)~=0
        Gd = 1;
    else
        Gd = 0;
    end
    
    good = Gd;
    
    
    info = [good; LENS];
    save(fullfile(paraDir,'calibResult.txt'),'info','-ascii');
    
    
    if good == 1
        cd(baseDir);
        
        save(fullfile(inputDir,'check.txt'),'info','-ascii');
% %         exit(1);
    else
        cd(baseDir);
% %         exit(0);
    end
    end






end