% function CvtRos2Mat(inputDir, replayDir)


% delete(fullfile(pwd,strcat(replayDir,'\*.mat')));

% convert dump camera bin and bosysensor bin to matlab .mat 
function bin2mat(bodysensor_bin_file, camera_imu_bin_file, dumpDir)
% clc;
MakeDirIfMissing(fullfile(pwd, dumpDir));
delete(fullfile(pwd,strcat(dumpDir,'\*.mat')));
% bodysensor_bin_file = './bodysensor.bin';
% camera_imu_bin_file = './camera_imu.bin';

KeySeq.bsSampFrame = {};
KeySeq.imgHeader = {};
KeySeq.imgL = {};
KeySeq.imgR = {};
KeySeq.imuSampFrame = {};
KeySeq.goldenVal = {};
KeySeq.depthGT = {};
KeySeq.depthGTR = {};


% read bodysensor.bin
disp(['start read bodysensor.bin ...']);
f_body = fopen(bodysensor_bin_file, 'rb');
bodysensor = [];
pose_x = 0;
pose_y = 0;
pose_theta = 0;
while  ~feof(f_body)
    magic = fread(f_body, 1, 'uint32');
    Velocity_sample_count = fread(f_body, 1, 'uint32');
    pkg.magic = magic;
    pkg.pose = [];
    last_Time_system = 0;
    first_time = true;
    for j = 1: Velocity_sample_count
        id = fread(f_body, 1, 'uint32');
        Time_sequence = fread(f_body, 1, 'uint32');
        Time_system = fread(f_body, 1, 'uint32');
        [Linear_velocity, c] = fread(f_body, 1, 'float');
        if c ~= 1
            break;
        end
        Angular_velocity = fread(f_body, 1, 'float');
        Distance_by_sonar = fread(f_body, 1, 'float');
        
        if first_time
            first_time = false;
            last_Time_system = Time_system;
        end
        
        delta_t = (Time_system - last_Time_system)*1e-3;
        delta_s = Linear_velocity*delta_t;
        delta_theta = -Angular_velocity*delta_t;
        pose_x = pose_x + cos(delta_theta)*delta_s;
        pose_y = pose_y + sin(delta_theta)*delta_s;
        pose_theta = pose_theta + delta_theta;
        
        pose.id = id;
        pose.Time_sequence = Time_sequence;
        pose.Time_system = Time_system;
        pose.Linear_velocity = Linear_velocity;
        pose.Angular_velocity = Angular_velocity;
        pose.Distance_by_sonar = Distance_by_sonar;
        pose.x = pose_x;
        pose.y = pose_y;
        pose.theta = pose_theta;
        pkg.pose = [pkg.pose; pose];
        
        last_Time_system = Time_system;
    end
    bodysensor = [bodysensor; pkg];
end
fclose(f_body);
disp(['finish read bodysensor.bin ...']);
disp(bodysensor);
px = [];
py = [];
% plot pose
all_pose_stamp = [];
all_pose_id = [];
for i = 1:  length(bodysensor)
    for j = 1: length(bodysensor(i).pose)
        px = [px; bodysensor(i).pose(j).x];
        py = [py; bodysensor(i).pose(j).y];
        all_pose_stamp = [all_pose_stamp; bodysensor(i).pose(j).Time_sequence];
        all_pose_id = [all_pose_id; bodysensor(i).pose(j).id];
    end
end
figure('Name', 'robot trace');
plot(px, py);
diff_pose_stamp  = all_pose_stamp(2:end) - all_pose_stamp(1:end-1);
figure('Name', 'diff_pose_stamp');
plot(diff_pose_stamp);
figure('Name', 'all_pose_stamp');
plot(all_pose_stamp);
figure('Name', 'all_pose_id');
plot(all_pose_id);


disp(['start read camera_imu.bin ...']);
f1 = fopen(camera_imu_bin_file, 'rb');
camera_imu = [];
all_img_stamp = [];
while  ~feof(f1)
    magic = fread(f1, 1, 'uint32');
    imu_sample_cnt = fread(f1, 1, 'uint16');
    img_w = fread(f1, 1, 'uint16');
    img_h = fread(f1, 1, 'uint16');
    img_c = fread(f1, 1, 'uint16');
    data_size = fread(f1, 1, 'uint32');

    imu_data = fread(f1, 9, 'uint32');
    mId = fread(f1, 1, 'uint32');
    mTime = fread(f1, 1, 'uint32');
    mSysTime  = fread(f1, 1, 'uint32');

    img_size = 640*480;
    left_img = fread(f1, img_size, 'uint8');
    right_img = fread(f1, img_size, 'uint8');

    if length(left_img) ~= 640*480 ...
        || length(right_img) ~= 640*480
       break; 
    end
%     figure(1);
%     imshow(reshape(left_img, [640, 480] )', []);
%     figure(2);
%     imshow(reshape(right_img, [640, 480] )', []);

    pkg.magic = magic;
    pkg.img_w = img_w;
    pkg.img_h = img_h;
    pkg.img_c = img_c;
    pkg.imu_data = imu_data;
    pkg.mId = mId;
    pkg.mTime = mTime;
    pkg.mSysTime = mSysTime;
    pkg.left_img = left_img;
    pkg.right_img = right_img;
    
    all_img_stamp = [all_img_stamp; mSysTime];
    
    camera_imu = [camera_imu; pkg];
end
fclose(f1);
disp(['finish read camera_imu.bin !']);
disp(camera_imu);
figure('Name', 'img_stamp');
plot(all_img_stamp);
diff_img_stamp  = all_img_stamp(2:end) - all_img_stamp(1:end-1);
figure('Name', 'diff_img_stamp');
plot(diff_img_stamp);


disp(['sync camera and bodysensor data ...']);

for img_idx = 2: length(camera_imu)
    img_id = camera_imu(img_idx).mId;
    img_stamp = camera_imu(img_idx).mSysTime;
    last_img_stamp = camera_imu(img_idx-1).mSysTime;
    img_w = 640;
    img_h = 480;

    bsSampFrame = [];
    for i = 1:  length(bodysensor)
        for j = 1: length(bodysensor(i).pose)
            pose_stamp = bodysensor(i).pose(j).Time_sequence;
            if img_stamp > pose_stamp && last_img_stamp < pose_stamp
                id = bodysensor(i).pose(j).id;
                stamp = bodysensor(i).pose(j).Time_sequence;
                v = bodysensor(i).pose(j).Linear_velocity;
                w = bodysensor(i).pose(j).Angular_velocity;
                bsSampFrame = [bsSampFrame ; [id stamp stamp v w 0]];
            end
        end
    end
    
    if length(camera_imu(img_idx).left_img) ~= 640*480 ...
        || length(camera_imu(img_idx).right_img) ~= 640*480
       break; 
    end
    
    imgHeader = [img_id img_id img_w img_h ];
    imgLC1 = reshape(camera_imu(img_idx).left_img, 640, 480);
    imgRC1 = reshape(camera_imu(img_idx).right_img, 640, 480);
    imgL = uint8(imgRC1'); %uint8(cat(3, imgLC1', imgLC1', imgLC1'));
    imgR = uint8(imgLC1'); %uint8(cat(3, imgRC1', imgRC1', imgRC1'));
    imuSampFrame = zeros(5, 12);
    goldenVal = zeros(size(bsSampFrame, 1), 7);
    depthGT = zeros(640, 480)';
    depthGTR = zeros(640, 480)';
    
    KeySeq.bsSampFrame = [KeySeq.bsSampFrame; bsSampFrame];
    KeySeq.imgHeader = [KeySeq.imgHeader; imgHeader];
    KeySeq.imgL = [KeySeq.imgL; imgL];
    KeySeq.imgR = [KeySeq.imgR; imgR];
    KeySeq.imuSampFrame = [KeySeq.imuSampFrame; imuSampFrame];
    KeySeq.goldenVal = [KeySeq.goldenVal; goldenVal];
    KeySeq.depthGT = [KeySeq.depthGT; depthGT];
    KeySeq.depthGTR = [KeySeq.depthGTR; depthGTR];
    
    len = round(length(camera_imu)/4);
    if  mod(img_idx, len) == 0
        number = floor(img_idx/len);
        file_name = sprintf('KeySeq_%05d.mat', number);
        disp(['dump to ', file_name]);
        save(fullfile(strcat(pwd,'\', dumpDir),file_name), 'KeySeq');
        
        KeySeq.bsSampFrame = {};
        KeySeq.imgHeader = {};
        KeySeq.imgL = {};
        KeySeq.imgR = {};
        KeySeq.imuSampFrame = {};
        KeySeq.goldenVal = {};
        KeySeq.depthGT = {};
        KeySeq.depthGTR = {};
    end
end


disp(['convert to .mat ...']);


% save(fullfile(replayDir,sprintf('KeySeq_%05d.mat',length(dir(fullfile(replayDir,'KeySeq_*.mat')))+1)), 'KeySeq');





end




















