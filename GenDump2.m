function GenDump2(inputDir, outputDir)

MakeDirIfMissing(outputDir);
delete(fullfile(outputDir, 'ReplayData_*.mat'));
poseMat = LoadGTPose('D:\Auto\simu\rpg_urban_pinhole_info\info\groundtruth.txt');
imgInfo = dir(fullfile(inputDir, '*.png'));
depthInfo = dir(fullfile(inputDir, '*.depth'));

for i = 1 : length(imgInfo)
    data = {};
    img1 = imread(fullfile(inputDir, imgInfo(i).name));
    depth1 = 1000.*reshape(load(fullfile(inputDir, depthInfo(i).name)), 640,480)';
    data{1, 1} = img1;
    data{1, 3} = depth1;
%     data{1, 5} = depth1;
    data{1, 7} = poseMat(i,:);
    save(fullfile(outputDir,sprintf('ReplayData_%05d.mat',length(dir(fullfile(outputDir,'ReplayData_*.mat')))+1)), 'data');
end


end
function poseMat = LoadGTPose(gtFile)
goldenData = load(gtFile);

poseMat = zeros(size(goldenData, 1), 12);
if 0
    for i = 1 : size(goldenData, 1)
        rotMat = quatern2rotMat(goldenData(i, [8 5 6 7]));
        T = [rotMat goldenData(i, 2:4)'; 0 0 0 1];
        rotMatNew = [rotMat(1,:); rotMat(3,:); -rotMat(2,:)];
        xyz = [goldenData(i, 2) -goldenData(i, 3) -goldenData(i, 4)];
        
        poseMat(i,:) = [rotMat(:)' goldenData(i, 2:4)];
        % %     poseMat(i,:) = [rotMatNew(:)' xyz];
        %     poseMat(i,:) = [rotMatNew(:)'  goldenData(i, 2:4)];
    end
end
for i = 1 : size(goldenData, 1)
    rotMat = quatern2rotMat(goldenData(i, [8 5 6 7]));
    T = [rotMat goldenData(i, 2:4)'; 0 0 0 1];
%     T(1:3,1:3) = [T(1,1:3); -T(3,1:3); T(2,1:3)];
%     T(1:3,1:3) = rotx(-90)*T(1:3,1:3);
%     T(1:3,4) = 1000.*[T(1,4); -T(2,4); -T(3,4)];
    if i == 1
        T0 = inv(T);
    end
    
    TT = T0 * T;
    %     TT(1:3,1:3) = [TT(1,1:3); -TT(2,1:3); -TT(3,1:3)];
    TT(1:3,1:3) = rotx(-90)*TT(1:3,1:3);
%     TT(1:3,1:3) = [TT(1,1:3); -TT(3,1:3); TT(2,1:3)];
    TT(1:3,1:3) = [TT(1:3,1) TT(1:3,3) -TT(1:3,2)];
    TT(1:3,4) = 1000.*[TT(1,4); -TT(2,4); -TT(3,4)];
    
    poseMat(i,:) = [reshape(TT(1:3,1:3), 1, 9) TT(1:3, 4)'];
    % %     poseMat(i,:) = [rotMatNew(:)' xyz];
    %     poseMat(i,:) = [rotMatNew(:)'  goldenData(i, 2:4)];
end

figure,plotPath(poseMat(1:20:end,:));
end