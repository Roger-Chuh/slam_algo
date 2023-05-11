for i = 1:length(camMatSet)
R = rodrigues(camMatSet{i}(:,1:3))/pi*180
T = camMatSet{i}(:,4)
GT = groundTruthPointSet{i}
end
