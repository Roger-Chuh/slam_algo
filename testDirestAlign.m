function testDirestAlign(k2c_comp, LR, Depth, intrMat)





load('pba.mat');

Pose = [];
for i = 0 : length(LR)-1
    if 0
        imwrite(rgb2gray(LR{i+1,1}), sprintf('./simulation2/img_%05d.png', i));
        imwrite(uint16(10.*Depth{i+1,1}),sprintf('./simulation2/depth_%05d.png',i),'png','bitdepth',16);
    end
    pose = inv(k2c_comp{i+1});
    Pose = [Pose; rodrigues(pose(1:3,1:3))' pose(1:3,4)'];
end



% close all

num_levels = 3; 1; 3; 2; 4; 2; 3; 2; 4; 3; 4; 2; 3; 1; 2; 4;  6; 4; 2;
areaRatio = 0.08; 0.001; 0.08; 0.25; 0.08; 0.005;
initial_sigma = 5;
default_dof = 5;
para = 5;
isVerbose = 0;
alpha0 = 0;
beta0 = 0;

alpha1 = 0;%-0.0847; 0;  -0.0623;
beta1 = 0;10.5487; 0; 94.0413;


dlt = 4;%1; 2;
for i = 1 : length(k2c_comp)-dlt
    relPose_check =  k2c_comp{i+dlt} * inv(k2c_comp{i});
    relPose = inv([rodrigues(Pose(i+dlt,1:3)) Pose(i+dlt,4:6)';0 0 0 1]) * [rodrigues(Pose(i,1:3)) Pose(i,4:6)';0 0 0 1];
    pose_init_error = relPose * [rodrigues([0.01 -0.01 0.01 ]) 20.*(rand(3,1)-0.5); 0 0 0 1];
    pose_init_error = relPose * [rodrigues([0.005 -0.005 0.005 ]) 10.*(rand(3,1)-0.5); 0 0 0 1];
    pose_init_error = relPose;
    [pose_rel, score, warped_image1, valid_mask, weightMap, warped_z, validMask] = estimateVisualOdometry(double(rgb2gray(LR{i+dlt,1})), double(rgb2gray(LR{i,1})), Depth{i+dlt,1}, Depth{i,1}, intrMat, num_levels, pose_init_error, isVerbose,initial_sigma,default_dof,para);
    %     OptimizeFlow(double(rgb2gray(LR{i,1})),Depth{i,1}, double(rgb2gray(LR{i+1,1})), pose_init_error, intrMat, alpha0, beta0, alpha1, beta1);
    OptimizeFlow(double(rgb2gray(LR{i,1})),Depth{i,1}, double(rgb2gray(LR{i+dlt,1})), pose_init_error, relPose, intrMat, alpha0, beta0, alpha1, beta1);
end





end
function OptimizeFlow(host,depth_host, target, T_th, T_th_gt, intrMat, alpha0, beta0, alpha1, beta1)

target0 = target;
% target = 0.5*target0 + 50;
if 0
    target = 1.5*target0 - 100;
    target(target>255) = 255;
end

fx = intrMat(1,1);
fy = intrMat(2,2);
cx = intrMat(1,3);
cy = intrMat(2,3);


width = size(host, 2);
height = size(host, 1);

point_host = detectFASTFeatures(uint8(host),'MinQuality',0.2,'MinContrast',0.2);

%  [pix,imggggg] = detectEdge(uint8(host),size(host),vec);

pt_host = double(point_host.Location);
[pt_host,imggggg] = detectEdge(uint8(host),ones(size(host)),[0.1 0.2 ]);

if 0
    figure,imshow(host, []);hold on;plot(pt_host(:,1), pt_host(:,2),'.r')
end

[xMat, yMat] = meshgrid(1: width, 1: height);
pix = [xMat(:) yMat(:)];
pix_norm = pflat(inv(intrMat) * (pextend(pix')))';

pix_norm_mat = reshape(pix_norm, height, width, 3);
pix_norm_mat_x = pix_norm_mat(:,:,1);
pix_norm_mat_y = pix_norm_mat(:,:,2);


pt_host = pt_host(pt_host(:,1) > 10 & pt_host(:,1)< 620 & pt_host(:,2) > 10 & pt_host(:,2)< 460,: );

gx = zeros(size(host));
gy = gx;

gx(:,2 : width-1) = 0.5 * (host(:,3:width) - host(:, 1:width-2));
gy(2 : height-1, :) = 0.5 * (host(3:height,:) - host(1:height-2,:));


offset = [0 0; 0 1; 1 0; 0 -1; -1 0];


est_affine = 1;

Twc = inv(T_th);
cost_change = [];
delta_pose = [];
mean_reproj_err = [];
Match = {};
for iter = 1 : 100
    
    PatchInfo0 = {};
    
    
    if iter < -5
        est_affine = 0;
    else 
        est_affine = 1;
    end
    Match{iter,1} = [];
    % FrameHessian1 只会维持一次迭代，solve后hessian要重置，
    % dsol中T_w_cur，a， b在跨层时才会更新，在同一层的多次迭代中，它们都不会更新，关于affine的雅可比全不变
    % 同一层不不同迭代中pose当然要更新，不然target的投影点会一直不变了
    cnt = 1;
    T_th = inv(Twc);
    FrameHessian1.H = zeros(8,8);
    FrameHessian1.b = zeros(8,1);
    FrameHessian1.c = 0;
    for i = 1 : size(pt_host,1)
        
        pt0 = pt_host(i,:);
        pt0s = pt0 + offset;
        ind = sub2ind(size(host), pt0s(:,2), pt0s(:,1));
        pt3d = inv(intrMat)*[pt0 1]'* depth_host(ind(1));
        pt3ds = (inv(intrMat)*[pt0s ones(5,1)]')'.* depth_host(ind(1));
        
        X = pt3d(1);
        Y = pt3d(2);
        Z = pt3d(3);
        
        pt1 = pflat(intrMat * (T_th(1:3,1:3)*pt3d + T_th(1:3,4)));
        
        pt1_gt = pflat(intrMat * (T_th_gt(1:3,1:3)*pt3d + T_th_gt(1:3,4)));
        pt1_gt = pt1_gt(1:2)';
        
        pt1s = pflat(intrMat * (T_th(1:3,1:3)*pt3ds' + repmat(T_th(1:3,4),1,size(offset,1))));
        pt1 = pt1(1:2)';
        pt1s = pt1s(1:2,:)';
        
        %    FrameHessian1.H = zeros(8,8);
        %    FrameHessian1.b = zeros(8,1);
        %     FrameHessian1.c = 0;
        if pt1(1) > 10 && pt1(1)< width-10 && pt1(2) > 10 && pt1(2)< height-10
            
            
             Match{iter,1} = [ Match{iter,1}; [pt1 pt1_gt]];
            
            warped_intensities = interp2(double(target), pt1s(:, 1), pt1s(:, 2), 'linear', 0);
            
            if 0
                figure, subplot(1,2,1);imshow(host, []);hold on;plot(pt0s(:,1),pt0s(:,2),'.r');  subplot(1,2,2);imshow(target, []);hold on;plot(pt1s(:,1),pt1s(:,2),'.r');
            end
            PatchInfo0{cnt,1}.pt0s = pt0s;
            PatchInfo0{cnt,1}.intensity = host(ind);
            PatchInfo0{cnt,1}.gx = gx(ind);
            PatchInfo0{cnt,1}.gy = gy(ind);
            PatchInfo0{cnt,1}.idepth = 1/depth_host(ind(1));
            PatchInfo0{cnt,1}.pt_norm = [pix_norm_mat_x(ind) pix_norm_mat_y(ind)];
            
            d_uv_d_pt3d = [fx/Z 0 -fx*X/Z/Z; 0 fy/Z -fy*Y/Z/Z];
            d_pt3d_d_host = [-SkewSymMat(pt3d) eye(3)];
            
            PatchInfo0{cnt,1}.JacGeo = d_uv_d_pt3d * d_pt3d_d_host;
            PatchInfo0{cnt,1}.JacPhoto = [gx(ind) gy(ind)];
            PatchInfo0{cnt,1}.d_photo_d_host = PatchInfo0{cnt,1}.JacPhoto * PatchInfo0{cnt,1}.JacGeo;
            PatchInfo0{cnt,1};
            % TODO 1相对0的相对光照系数 calcRelative
            e_a0ma1 = exp(alpha0) / exp(alpha1);
            % TODO 图像1下的根据当前ab补偿后的灰度值
            patch2_comp = e_a0ma1 * (warped_intensities - beta1);
            rs = (host(ind) - beta0) - patch2_comp;
            It = [gx(ind) gy(ind)]';
            PatchHessian1 =  SetI(It, rs, 1);
            
            At(1, :) = patch2_comp';
            At(2, :) = (e_a0ma1);
            
            PatchHessian1 = SetA(PatchHessian1, It, At, rs, 1);
            
            dn_dx = PatchInfo0{cnt,1}.JacGeo;
            
            FrameHessian1 = AddPatchHess(FrameHessian1, PatchHessian1, dn_dx, est_affine);
            cnt = cnt + 1;
        end
    end
    
    [warped_image, valid_mask, warped_z, validMask] = warpImage(e_a0ma1 * (target - beta1), depth_host, inv(Twc), intrMat, intrMat, host-beta0);
    
    if 0
        figure(10),hold on;imshowpair(warped_image, host);drawnow;
        saveas(gcf,  sprintf('./dump/align_iter_%05d.png', iter));
    end
    
    if est_affine
        dx = inv(FrameHessian1.H) * FrameHessian1.b;
        
        
       
        %% 右乘，求雅可比是AtIdentity，是相对位姿的扰动，而更新状态量是绝对位姿，要把相对增量转成绝对增量。
        Twc = Twc * [rodrigues(dx(1:3)) dx(4:6); 0 0 0 1];
        alpha1 = alpha1 + dx(7);
        beta1 = beta1 + dx(8);
    else
        dx = inv(FrameHessian1.H(1:6,1:6)) * FrameHessian1.b(1:6);
        Twc = Twc * [rodrigues(dx(1:3)) dx(4:6); 0 0 0 1];
    end
    cost_change = [cost_change; FrameHessian1.c];
    
    if (iter > 10)
        if(cost_change(end) > cost_change(end-1))
            break;
        end
    end
    
%     [T_th_gt inv(Twc) Twc * T_th_gt]
  [~,reproj_err] = NormalizeVector(Match{iter,1}(:,1:2) - Match{iter,1}(:,3:4));
    dlt_pose = Twc * T_th_gt;
    mean_reproj_err = [mean_reproj_err; mean(reproj_err)];
    delta_pose = [delta_pose; rad2deg(norm(rodrigues(dlt_pose(1:3,1:3)))) norm(dlt_pose(1:3,4))];
end

if 0
    figure,imshow([target e_a0ma1 * (target - beta1) host-beta0],[])
    
    
    figure,subplot(3,1,1);plot(cost_change);title('total loss (intensity^2)');grid on;
    subplot(3,1,2);plot(delta_pose);title('pose error');legend('rot err (deg)','trans err(mm)');grid on;
    subplot(3,1,3);plot(mean_reproj_err);grid on;title('reproj err (pixel)');
end

end

function FrameHessian2 = AddPatchHess(FrameHessian1, ph, G, est_affine)
% 构造hession 和 b
pose_offset = 6;
FrameHessian1.H(1:pose_offset, 1:pose_offset) = FrameHessian1.H(1:pose_offset, 1:pose_offset) + G' * ph.ItI * G;

FrameHessian1.b(1:pose_offset) = FrameHessian1.b(1:pose_offset) - G' * ph.Itr;

FrameHessian1.c = FrameHessian1.c + ph.r2;

if est_affine
    affine_offset = 2;
    FrameHessian1.H(pose_offset+1: pose_offset+affine_offset,1:pose_offset) =  FrameHessian1.H(pose_offset+1: pose_offset+affine_offset,1:pose_offset) + ph.ItA' * G;
    
    FrameHessian1.H(1:pose_offset, pose_offset+1: pose_offset+affine_offset) = FrameHessian1.H(pose_offset+1: pose_offset+affine_offset,1:pose_offset)';
    
    
    FrameHessian1.H(pose_offset+1: pose_offset+affine_offset,pose_offset+1: pose_offset+affine_offset) = FrameHessian1.H(pose_offset+1: pose_offset+affine_offset,pose_offset+1: pose_offset+affine_offset) +  ph.AtA;
    
    
    FrameHessian1.b(pose_offset+1: pose_offset+affine_offset) = FrameHessian1.b(pose_offset+1: pose_offset+affine_offset)-ph.Atr;
end
FrameHessian2 = FrameHessian1;
end

function A = SetI(It, r, w)
wr = 1.0 * r;
A.ItI = It * w * It';
A.Itr = It * wr;
A.r2 = sum(r .* r);
A.wr2 = sum(wr .* r);
end
function B = SetA(A,It,At, r, w)

wA = w * At';
A.ItA = It * wA;
A.AtA = At * wA;
A.Atr = At * (w * r);
B = A;
end

function UpdateHess()


end