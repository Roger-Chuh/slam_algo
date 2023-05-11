function data = debug_integration(ImuData, imuData, n1, n2)

if 0
    R0 = quatern2rotMat(ImuData(n1,[6 3 4 5]));
    R1 = quatern2rotMat(ImuData(n2,[6 3 4 5]));
    % dtR0 = rodrigues(R1*R0');
    dtR00 = rodrigues(R0'*R1);
    
else
    
    R00 = quatern2rotMat(ImuData(n1,[6 3 4 5]));
    R00_ = R00'; %(RgyroCur'*RgyroPrv)'; %RgyroCur; %
    eulerGyro = rad2deg(rodrigues(R00_)');
    eulerGyro = ([-eulerGyro(:,2) -eulerGyro(:,3) eulerGyro(:,1)]);
    R0 = rodrigues(deg2rad(eulerGyro));
    
    R11 = quatern2rotMat(ImuData(n2,[6 3 4 5]));
    R11_ = R11'; %(RgyroCur'*RgyroPrv)'; %RgyroCur; %
    eulerGyro = rad2deg(rodrigues(R11_)');
    eulerGyro = ([-eulerGyro(:,2) -eulerGyro(:,3) eulerGyro(:,1)]);
    R1 = rodrigues(deg2rad(eulerGyro));
    
    %% dtR0 = rodrigues(R1'*R0);
    %%
    dtR0 = rodrigues(R0'*R1);
    
end




dt = 1/50;

vec =  n1: (n2-1);

R01 = eye(3);
for ij = 1 : length(vec)
    
    R01 = R01*rodrigues(imuData(vec(ij),2:4)*dt);
    %     R01 = rodrigues(imuData(vec(ij),2:4)*dt)*R01;
    
end


% data = [rodrigues(R01') dtR0];
data = [rodrigues(R01) dtR0];



end