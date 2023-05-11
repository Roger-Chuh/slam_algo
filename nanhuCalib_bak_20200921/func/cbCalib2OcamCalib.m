function calib_data_tmp = cbCalib2OcamCalib(calib_data, paraDir)
global cfg
draw = 0;
load(fullfile(paraDir, 'calib.mat'));


calib_data_tmp = calib_data;
% if draw
% figure,
% end
for i = 1 : length(calib_data.ima_proc)   %size(calib_data_tmp.Xp_abs,1)  %size(calib_data_tmp.Xp_abs,3)
    xy = cbcXY{i};
    if draw
        figure, subplot(1,2,1);imshow(zeros(2160,4096));hold on;plot(xy(1,:),xy(2,:),'-r');plot(xy(1,1),xy(2,1),'*b');
    end
    xy = fliplr(xy);
    xMat = reshape(xy(1,:),cfg.cb_row,cfg.cb_col); yMat = reshape(xy(2,:),cfg.cb_row,cfg.cb_col);
    xMat = flipud(xMat); yMat = flipud(yMat);
    xy = [xMat(:) yMat(:)]';
    if draw
        subplot(1,2,2);imshow(zeros(2160,4096));hold on;plot(xy(1,:),xy(2,:),'-r');plot(xy(1,1),xy(2,1),'*b');
        drawnow;
    end
    calib_data_tmp.Xp_abs(:,:,i) = xy(2,:)';
    calib_data_tmp.Yp_abs(:,:,i) = xy(1,:)';
    
end

end