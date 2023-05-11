function good = checkCorner(cbcL,cbcR,calib_data,imgL,imgR)

   vertL = cbcL(:,1:calib_data.n_sq_y+1);
   vertR = cbcR(:,1:calib_data.n_sq_y+1);
   [~,indVertL] = sort(vertL(2,:));
   [~,indVertR] = sort(vertR(2,:));
    
   horiL = cbcL(:,1:calib_data.n_sq_y+1:size(cbcL,2)-calib_data.n_sq_y);
   horiR = cbcR(:,1:calib_data.n_sq_y+1:size(cbcR,2)-calib_data.n_sq_y);
   [~,indHoriL] = sort(horiL(1,:));
   [~,indHoriR] = sort(horiR(1,:));
   
   
   cbcXL_mat = reshape(cbcL(1,:),calib_data.n_sq_y+1,calib_data.n_sq_x+1);  cbcYL_mat = reshape(cbcL(2,:),calib_data.n_sq_y+1,calib_data.n_sq_x+1);
   cbcXR_mat = reshape(cbcR(1,:),calib_data.n_sq_y+1,calib_data.n_sq_x+1);  cbcYR_mat = reshape(cbcR(2,:),calib_data.n_sq_y+1,calib_data.n_sq_x+1);
   [~,indVertAllL] = sort(cbcYL_mat,1); [~,indVertAllR] = sort(cbcYR_mat,1);
   [~,indHoriAllL] = sort(cbcXL_mat,2); [~,indHoriAllR] = sort(cbcXR_mat,2);
   
   if 0
        figure, 
        subplot(1,2,1); imshow(imgL); hold on; plot(cbcL(1, :), cbcL(2,:), '+m', cbcL(1, 1), cbcL(2,1), 'og','linewidth',2); plot(cbcL(1, (1:calib_data.n_sq_y+1)), cbcL(2, (1:calib_data.n_sq_y+1)),'-b','linewidth',2);title(sprintf('Left Frame %d',i));
        subplot(1,2,2); imshow(imgR); hold on; plot(cbcR(1, :), cbcR(2,:), '+m', cbcR(1, 1), cbcR(2,1), 'og','linewidth',2); plot(cbcR(1, (1:calib_data.n_sq_y+1)), cbcR(2, (1:calib_data.n_sq_y+1)),'-b','linewidth',2);title(sprintf('Right Frame %d',i));
        hold off
    end
   
   
   if size(cbcL,2) == (calib_data.n_sq_x+1)*(calib_data.n_sq_y+1) && ...
      size(cbcR,2) == (calib_data.n_sq_x+1)*(calib_data.n_sq_y+1) && ...
      sum(sum(abs(indVertL - [1:(calib_data.n_sq_y+1)]))) == 0 && sum(sum(abs(indVertR - [1:(calib_data.n_sq_y+1)]))) == 0 && ...
      sum(sum(abs(indHoriL - [1:(calib_data.n_sq_x+1)]))) == 0 && sum(sum(abs(indHoriR - [1:(calib_data.n_sq_x+1)]))) == 0 && ...
      sum(sum(abs(indVertAllL - repmat([1:(calib_data.n_sq_y+1)]',1,calib_data.n_sq_x+1)))) == 0 && ...
      sum(sum(abs(indVertAllR - repmat([1:(calib_data.n_sq_y+1)]',1,calib_data.n_sq_x+1)))) == 0 && ...
      sum(sum(abs(indHoriAllL - repmat([1:(calib_data.n_sq_x+1)],calib_data.n_sq_y+1,1)))) == 0 && ...
      sum(sum(abs(indHoriAllR - repmat([1:(calib_data.n_sq_x+1)],calib_data.n_sq_y+1,1)))) == 0
    
% % % % % %        imwrite(img, fullfile(outputDirPath,dstFileName{k,1}));
       
% %        cd(inputDir);
% %        eval(['!rename' 32 srcInfo(i).name 32 dstFileName{k,1}]);
% %        cd(workDir);
% %        movefile(fullfile(inputDir,dstFileName{k,1}), outputDirPath);  

% % % % % % %        fprintf(fullfile(outputDirPath,dstFileName{k,1}));
       good = 1;
   else
       good = 0;
   end
   
   
end