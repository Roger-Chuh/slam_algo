function imuSampFrame_ = checkImuSample(imuSampFrame,imuSampRate)
imuSampFrame_ = imuSampFrame;
for i = 1 : size(imuSampFrame,1)-1
    nn =i;
    R0 = quatern2rotMat(imuSampFrame(nn,[6 3 4 5]));
    R1 = quatern2rotMat(imuSampFrame(nn+1,[6 3 4 5]));
    if 0
        %%
        dtR0 = rodrigues(R1*R0');
        err = norm(imuSampFrame(nn,7:9)/imuSampRate+dtR0');
        if 1 %err > 0.1
            %%
            imuSampFrame_(i,7:9) =  -dtR0'*imuSampRate;
        end
    elseif 0
        %%         dtR0 = rodrigues(R1*R0');
        dtR0 = rodrigues(R1'*R0);
        err = norm(imuSampFrame(nn,7:9)/imuSampRate-dtR0');
        if 1 %err > 0.1
            %%
            imuSampFrame_(i,7:9) =  dtR0'*imuSampRate;
        end
    else
        
        R00 = quatern2rotMat(imuSampFrame(nn,[6 3 4 5]));
        R00_ = R00'; %(RgyroCur'*RgyroPrv)'; %RgyroCur; %
        eulerGyro = rad2deg(rodrigues(R00_)');
        eulerGyro = ([-eulerGyro(:,2) -eulerGyro(:,3) eulerGyro(:,1)]);
        R0 = rodrigues(deg2rad(eulerGyro));
        
        R11 = quatern2rotMat(imuSampFrame(nn+1,[6 3 4 5]));
        R11_ = R11'; %(RgyroCur'*RgyroPrv)'; %RgyroCur; %
        eulerGyro = rad2deg(rodrigues(R11_)');
        eulerGyro = ([-eulerGyro(:,2) -eulerGyro(:,3) eulerGyro(:,1)]);
        R1 = rodrigues(deg2rad(eulerGyro));
        
        %% dtR0 = rodrigues(R1'*R0);
        %% 
        dtR0 = rodrigues(R0'*R1);
        err = norm(imuSampFrame(nn,7:9)/imuSampRate-dtR0');
        if 1 %err > 0.1
            %%
            imu_tmp = dtR0'*imuSampRate;
%             imuSampFrame_(i,7:9) =  [-imu_tmp(3) imu_tmp(1) imu_tmp(2)];
            imuSampFrame_(i,7:9) =  [imu_tmp(3) -imu_tmp(1) -imu_tmp(2)];
        end
    end
end



end