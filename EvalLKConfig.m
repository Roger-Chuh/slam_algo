function EvalLKConfig(inputDir)

dirInfo = dir(fullfile(inputDir,'2020*'));

xStack = {}; xStack_inv = {};
cnt = 1;
for i = 1 : length(dirInfo)
    
    if exist(fullfile(inputDir,dirInfo(i).name,'congfigInfo2.mat'))
        load(fullfile(inputDir,dirInfo(i).name,'congfigInfo2.mat'));
        if exist('Y_err_stack', 'var')
            
            xStack{cnt,1} = X_err_stack{1,1};
            xStack_inv{cnt,1} = X_err_stack_inv{1,1};
            cnt = cnt + 1;
            clear  Y_err_stack
        end
    end
    
end


[trailMatUse, aa] = SortData(xStack);


if 0
    for i = 1 : size(trailMatUse,1)
        
        temp = squeeze(trailMatUse(i,:,:));
        
        figure,plot(temp);legend('51','81','35');drawnow;
        
    end
    
else
    [trailMatUse_inv, aa_inv] = SortData(xStack_inv);
    
    for i = 1 : size(trailMatUse_inv,1)
        
        temp_inv = squeeze(trailMatUse_inv(i,:,:));
        
        figure,plot(temp_inv);legend('51','81','35');drawnow;
        
    end
end

return;

id1 = xStack{1,1}(:,1);
id1_inv = xStack_inv{1,1}(:,1);

id2 = xStack{2,1}(:,1);
id2_inv = xStack_inv{2,1}(:,1);


[ind1, ind2, err] = intersection(id1,id2);
trial1 = xStack{1,1}(ind1,2:end);
trial2 = xStack{2,1}(ind2,2:end);


[ind1_inv, ind2_inv, err_inv] = intersection(id1_inv, id2_inv);
trial1_inv = xStack_inv{1,1}(ind1_inv,2:end);
trial2_inv = xStack_inv{2,1}(ind2_inv,2:end);





for k = 1 : size(trial1,1)
    checkId = k;
    figure,plot([trial1(checkId,:);trial2(checkId,:)]');
    drawnow;
end



for k = 1 : size(trial1_inv,1)
    checkId = k;
    figure,plot([trial1_inv(checkId,:);trial2_inv(checkId,:)]');
    drawnow;
end



end

function [ind1, ind2, err] = intersection(id1,id2)
[~,ind1, ind2] = intersect(id1, id2,'rows');

[~,ind11] = sort(ind1);
ind1 = ind1(ind11);
ind2 = ind2(ind11);
err = id1(ind1) - id2(ind2);
end

function [trailMatUse, aa] = SortData(xStack)
for i = 1 : size(xStack,1)
    if i == 1
        commId =  xStack{i,1}(:,1);
    else
        commId = intersect(commId, xStack{i,1}(:,1));
    end
    
end

trailMat = [];
for i = 1 :size(xStack,1)
    inId = xStack{i,1}(:,1);
    inlier = find(ismember(inId, commId));
    trailMat(:,:,i) =  xStack{i,1}(inlier,1:end);
    
end
aa = squeeze(trailMat(:,1,:));
trailMatUse = trailMat(:,2:end,:);
end
