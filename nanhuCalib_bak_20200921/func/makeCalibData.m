function calib_data = makeCalibData(imgLst,x,y,dX,dY)


set_up_global
calib_data.active_images = ones(1, length(imgLst));
calib_data.ind_active = [1: length(imgLst)];
calib_data.ind_read = [1: length(imgLst)];
calib_data.taylor_order_default = 4;
% % % % calib_data.n_sq_x = 13;
% % % % calib_data.n_sq_y = 8;
calib_data.n_sq_x = x;
calib_data.n_sq_y = y;


calib_data.dX = dX;
calib_data.dY = dY;
for iPic = 1:length(imgLst)
    calib_data.L{1,iPic} = imgLst{iPic};
    
end






end