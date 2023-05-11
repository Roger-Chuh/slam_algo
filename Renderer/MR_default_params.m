function [cameraparams,renderparams, Vcam] = MR_default_params(V,width,flip)
%MR_DEFAULT_PARAMS Create some default parameters for basic rendering
%
% Returns camera and rendering parameters to give a basic rendering of a
% mesh. The camera is perspective, with the image centred on the centre of
% mass of the mesh. The camera-object distance is 3x the diagonal length of
% the bounding box.
%
%   Inputs:
%      V     - nverts x 3 matrix containing mesh vertices
%      width - desired width of rendered image
%      flip  - (Default = true) determines whether a 180 degree rotation
%              about the x axis is applied (required if y is up in your
%              model)
%   Outputs:
%      cameraparams,renderparams - structures that can be passed to
%                                  MR_render_mesh
%
% William Smith
% University of York
%
% Part of the Matlab Renderer (https://github.com/waps101/MatlabRenderer)


global rotMat transVec

if nargin==2
    flip=true;
end

if 1
    imgW = 640; 1280; 640;
    imgH = 480; 720; 480;
    focal = 554.256258; 300; %   550;  1100; 550; 200;
else
    imgW = 1280; 640;
    imgH = 720; 480;
    focal = 1100; 550;

end


T0 = [roty(0) [0 0 4000]'; 0 0 0 1];  % world to cam / world in cam


T0 = [rotMat transVec; 0 0 0 1];


% Diagonal size of bounding box
diagsize = norm(min(V,[],1)-max(V,[],1));

if flip
    R = rotz(180)*roty(180);
else
    R = eye(3);
end
 
if 1
    R = roty(30)*R;
end

T = [eye(3) [0;0;3.*diagsize]; 0 0 0 1]*[R zeros(3,1); 0 0 0 1]*[eye(3) -0.5.*(min(V,[],1)+max(V,[],1))'; 0 0 0 1];
% % T(1,4) = 0.1;
% % T(3,4) = 0.9;
T = T0;
% cameraparams.type = 'perspectiveWithDistortion';
cameraparams.type = 'perspective';

if 1
    cameraparams.cx = 0;
    cameraparams.cy = 0;
    cameraparams.f = 1;
    cameraparams.T = T;
else
    cameraparams.cx = (1 + imgW) / 2;
    cameraparams.cy = (1 + imgH) / 2;
    cameraparams.f = focal;
    cameraparams.T = T;
end
if 0
%     -0.162634913921236 0.012270673412312 -0.002178680927890 -0.002448749761278 0.003953800470367
% 0.153221,0.101447,0.00371338,0.00427771,0.002554
    cameraparams.k1 = -0.162634913921236;
    cameraparams.k2 = 0.012270673412312;
    cameraparams.k3 = -0.002178680927890;
    cameraparams.p1 = -0.002448749761278;
    cameraparams.p2 = 0.003953800470367;
    
    cameraparams.k1 = 1.62634913921236;
    cameraparams.k2 = 0.602270673412312;
    cameraparams.k3 = 0.002178680927890;
    cameraparams.p1 = 0.00448749761278;
    cameraparams.p2 = 0.00253800470367;
else
    cameraparams.k1 = 0.;
    cameraparams.k2 = 0.;
    cameraparams.k3 = 0;
    cameraparams.p1 = 0.000;
    cameraparams.p2 = -0.000;
end
[Vxy, Vcam] = MR_project(V,cameraparams);
if 0
    figure,plot(Vxy(:,1),Vxy(:,2),'.');axis equal
end
width2d = 2.*max(max(Vxy(:,1)),abs(min(Vxy(:,1))));
height2d = 2.*max(max(Vxy(:,2)),abs(min(Vxy(:,2))));
cameraparams.f = focal; width/width2d;

aspect = height2d/width2d;

cameraparams.w = imgW; width;
cameraparams.h = imgH; ceil(width*aspect);
cameraparams.cx = (1 + imgW) / 2; cameraparams.w/2;
cameraparams.cy = (1 + imgH) / 2;cameraparams.h/2;

renderparams.lightingModel =  'localPointSource';
% renderparams.lightingModel = 'distantPointSource';
renderparams.shininess = 40; 20;
renderparams.backfacelighting = 'lit'; 'unlit'; 'lit'; 'unlit';
if 1
    renderparams.ka = 0;
    renderparams.kd = 0.9; 0; % 0.9;
    renderparams.ks = 0.3; 0; % 0.3;
else
    renderparams.ka = 0.1;
    renderparams.kd = 0.1; 0; % 0.9;
    renderparams.ks = 0.1; 0; % 0.3;
end
renderparams.textureMode = 'usePerMeshColour';
renderparams.permeshcolour = [0;0;1];
% renderparams.sourceposition = -T(1:3,1:3)'*T(1:3,4); % Position light at camera
renderparams.sourceposition = [0;-1500;0];-T(1:3,1:3)'*T(1:3,4); % Position light at camera


% renderparams.sourceposition = [-1000;-1500;0]; % for ball;
renderparams.sourceposition = [-1000;-1500;0]; % for ball;

% renderparams.sourceposition = [-0;30000;30000]; % for ros;

% renderparams.sourceposition = [10000;-5500;1000];-T(1:3,1:3)'*T(1:3,4); % Position light at camera % for multi objs
% renderparams.sourceposition = -T(1:3,1:3)'*T(1:3,4); % Position light at camera
% renderparams.sourceposition =  [0;6500;6000];-T(1:3,1:3)'*T(1:3,4);;-T(1:3,1:3)'*T(1:3,4); % Position light at camera
% renderparams.sourceposition = roty(-90)*(T(1:3,1:3)'*T(1:3,4))-(T(1:3,1:3)'*T(1:3,4)); % position light right of camera
renderparams.sourcecolour = [1; 1; 1];
renderparams.shadows = false;
renderparams.sourcedirection = [0 0 1]';
end

