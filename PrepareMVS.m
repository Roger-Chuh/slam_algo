function PrepareMVS(probPath,inputDir,intrMat, vslMat, Num, dirInfo, frameList,scaleMat, kitti, dInfo)

if 0
    probPath = 'C:\Users\rongjiezhu\Pictures\Camera Roll\zedRect\office010\depthOpt';
%     load('\\192.168.50.172\nextvpu\3.CTO办公室\Roger\temp\extFunc\21\loopData21.mat')
%     load('\\192.168.50.172\nextvpu\3.CTO办公室\Roger\temp\extFunc\21\key_21_007.mat')
    
    load('C:\Users\rongjiezhu\Downloads\key_21_007.mat')
load('C:\Users\rongjiezhu\Downloads\loopData21.mat')
    Num = [1 905];
    vslMat = poseMat;
    dirInfo = dir('C:\Users\rongjiezhu\Pictures\Camera Roll\zedRect\office010\image_0\*.png');
    scaleMat='C:\Users\rongjiezhu\Pictures\Camera Roll\zedRect\office010\image_1';
    kitti=1;
    inputDir = 'C:\Users\rongjiezhu\Pictures\Camera Roll\zedRect\office010\image_0';
    intrMat = intrMatL;
    intrMat = intrMat./2;
    intrMat(3,3) = 1;
    frameList = [1:910];
    dInfo = [];
    PrepareMVS(probPath,inputDir,intrMat, vslMat, Num, dirInfo, frameList,scaleMat, kitti, dInfo);
end



MakeDirIfMissing(probPath);

cxOfst = 1; 161;
cyOfst = 1; 401;
intrMat(1,3) = intrMat(1,3) - cxOfst + 1;
intrMat(2,3) = intrMat(2,3) - cyOfst + 1;

fid1 = fopen(fullfile(probPath,'cameras.txt'),'w');%????
fprintf(fid1,sprintf('%d\n\n',Num(2) - Num(1) + 1));
cnt = 1;
for i = Num(1) : Num(2)
    if ~kitti
        load(fullfile(inputDir, dirInfo(frameList(i)).name));
    else
        if ~isempty(dInfo)
            img = imread(fullfile(inputDir,dInfo(1).name, dirInfo(frameList(i)).name));
        else
            img = imresize(imread(fullfile(inputDir,dirInfo(frameList(i)).name)),0.5);
            if ischar(scaleMat)
                img_r = imresize(imread(fullfile(scaleMat,dirInfo(frameList(i)).name)),0.5);
            end
        end
        colEnd = floor(size(img,2)/2)*2;
        rowEnd = floor(size(img,1)/2)*2;
        img = img(1:rowEnd, 1:colEnd);
        data{1} = cat(3, img, img,img);
    end
    r = reshape(vslMat(i,1:9), 3,3);
    if 0
        t = vslMat(i,10:12)./10; % mm->cm
    else
        t = vslMat(i,10:12).*100; % m->cm
    end
    rt = [r;t];
    krt = [intrMat;rt];
    %     imwrite(data{1}(1:240,:,:), fullfile(probPath, sprintf('test%04d.jpg',cnt-1)));
    %     imwrite(data{1}(1:600,:,:), fullfile(probPath, sprintf('test%04d.jpg',cnt-1)));
    if 0
        imwrite(data{1}(cyOfst : end, cxOfst : end, :), fullfile(probPath, sprintf('test%04d.jpg',cnt-1)));
    end
    if 0 % ~kitti
        imwrite(data{1}(cyOfst : end, cxOfst : end, :), fullfile(probPath, sprintf('imgL_%04d.png',cnt-1)));
        try
            imwrite(data{2}(cyOfst : end, cxOfst : end, :), fullfile(probPath, sprintf('imgR_%04d.png',cnt-1)));
        catch
        end
        try
            imwrite(uint16(scaleMat.*data{5}),fullfile(probPath, sprintf('depth_%04d.png',cnt-1)),'png','bitdepth',16);
        catch
            imwrite(uint16(scaleMat.*data{3}),fullfile(probPath, sprintf('depth_%04d.png',cnt-1)),'png','bitdepth',16);
        end
    end
    %     imwrite(data{1}, fullfile(probPath, sprintf('test%04d.jpg',cnt-1)));
    
    for ji = 1 : size(krt,1)
        fprintf(fid1,sprintf('%0.7f    %0.7f    %0.7f \n',krt(ji,1), krt(ji,2), krt(ji,3)));
%         fprintf(fid1,'\n');
    end
    fprintf(fid1,'\n\n');
    cnt = cnt + 1;
end
fprintf(fid1,'\n');
fclose(fid1);

end