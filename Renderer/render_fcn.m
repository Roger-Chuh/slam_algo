
% close all
%% Load the Stanford Bunny mesh
function [XYZ_orig, render,zbuffer,Vcam] = render_fcn(obj, rotMat0, transVec0)


global rotMat transVec xyzOrig_fixed textureFile


rotMat = rotMat0;
transVec = transVec0;
if 0
    obj = MR_obj_read('data/StanfordBunny.obj');
    obj.V = 10000.*(rotz(180)*roty(180)*obj.V')';
    obj.V(:,1) = obj.V(:,1) - mean(obj.V(:,1));
    obj.V(:,2) = obj.V(:,2) - mean(obj.V(:,2));
    obj.V(:,3) = obj.V(:,3) - mean(obj.V(:,3));
    max(obj.F) - size(obj.V,1)
end


% obj = MR_obj_read('E:\bk_20180627\pc\Work\MyFuncs\ø…¿÷πﬁ\keleguan.obj');
% % obj = MR_obj_read('E:\bk_20180627\pc\Work\MyFuncs\MatlabRenderer-master\MatlabRenderer-master\data\81-old_barrel\old_barrel\old_barrel_final.obj');
% obj = MR_obj_read('data/Handgun_obj.obj');
% obj = MR_obj_read('data/banana_sliced(quads).obj');
% obj = MR_obj_read('data/boat.obj');
% obj.V(:,3) = obj.V(:,3) - min( obj.V(:,3))  + 10;
% obj.V(:,1) = obj.V(:,1) - min( obj.V(:,1))  + 10;
% % % obj.V = 1000.*obj.V;
% % % obj.V(:,3) = obj.V(:,3) + 5000;

if 0
    figure,pcshow(obj.V)
end
%% Setup some default camera parameters and render
if 0
    [cameraparams,renderparams] = MR_default_params(obj.V,400);
    
    render = MR_render_mesh(obj.F,obj.V,cameraparams,renderparams);
    render1 = render;
    ptIcs = detectFASTFeatures(rgb2gray(uint8(255.*render1)),'MinQuality',0.2,'MinContrast',0.2);
    figure; imshow(render);hold on;plot(ptIcs.Location(:,1), ptIcs.Location(:,2), '.r');
end
%% More advanced: load a texture map and use per-vertex normals from obj
if 1
    [cameraparams,renderparams, Vcam] = MR_default_params(obj.V,400);
    
    renderparams.VT = obj.VT;
    renderparams.FT = obj.FT;
    
    if 0 % isfield(obj, 'VN')
        renderparams.VN = obj.VN;
        renderparams.FN = obj.FN;
    end
    renderparams.textureMode = 'useTextureMap';
    if 1
        %         renderparams.texmap = im2double(imread('data/StanfordBunny.jpg'));
        %         renderparams.texmap = im2double(imread('C:\Users\rongjiezhu\Desktop\bg2.jpg'));
        %         renderparams.texmap = im2double(imread('C:\Users\rongjiezhu\Desktop\bg2_bak.jpg'));
        if 0
            renderparams.texmap = im2double(imread('C:\Users\rongjiezhu\Desktop\bg6.jpg'));
        else
            renderparams.texmap = im2double(imread(textureFile));
        end
        %         renderparams.texmap = im2double(imread('C:\Users\rongjiezhu\Desktop\bg7.jpg'));
        % renderparams.texmap = im2double(imread('C:\Users\rongjiezhu\Desktop\bg8.jpg'));

%         renderparams.texmap = im2double(imread('C:\Users\rongjiezhu\Desktop\bg.jpg'));
        
        
%         renderparams.texmap = im2double(imread('C:\Users\rongjiezhu\Desktop\bg_big.jpg'));
        
        %         renderparams.texmap = im2double(imresize(imread('C:\Users\rongjiezhu\Desktop\bg.jpg'),[3072 3072]));
        %         renderparams.texmap = im2double((imread('C:\Users\rongjiezhu\Desktop\bg2.jpg')));
        %         renderparams.texmap(982:982+10,2170:2180,:) = 0;
        %     renderparams.texmap = im2double(imread('data/handgun_S.jpg'));
        
        %         renderparams.texmap = im2double(imread('C:\Users\rongjiezhu\Desktop\ros1.jpg'));
        
        
    else
        renderparams.texmap = im2double(imread('data/banana_diffuse.jpg'));
    end
    
    
    
    tic;
    [render, zbuffer] = MR_render_mesh(obj.F,obj.V,cameraparams,renderparams);
    toc
%     render = render./max(render(:));
    render = uint8(255.*render);
    intrMat = [cameraparams.f 0 cameraparams.cx; 0 cameraparams.f cameraparams.cy; 0 0 1];
    
    pixCoord = [372 296];
%     pixCoord = [743 472];
    
    [XYZ] = GetXYZFromDepth(intrMat, pixCoord, zbuffer(pixCoord(2),pixCoord(1)));
    if 0
        [~, disp] = NormalizeVector(Vcam - repmat(XYZ, size(Vcam,1),1));
        [~, PtId] = min(disp);
    end
    
    XYZ_orig = inv(cameraparams.T)*[XYZ 1]';
    XYZ_orig = XYZ_orig(1:3)';
    
    ptId = 1; 2729;
    if isempty(xyzOrig_fixed)
        [Vxy, ~] = MR_project(obj.V(ptId,:),cameraparams);
        ptProj = pflat(intrMat * Vcam(ptId,:)');
        err = Vxy - ptProj(1:2)';
    else
%         XYZ_orig = (cameraparams.T)*[xyzOrig_fixed 1]';
        [Vxy, ~] = MR_project(xyzOrig_fixed,cameraparams);
        pixCoord_ = round(Vxy);
        try
            [XYZ_] = GetXYZFromDepth(intrMat, pixCoord_, zbuffer(pixCoord_(2),pixCoord_(1)));
            XYZ_orig = inv(cameraparams.T)*[XYZ_ 1]';
            XYZ_orig = XYZ_orig(1:3)';
        catch
            XYZ_orig = [];
        end
    end
    
    if 0
        figure,imshow(render);
        try
            if 0
                cbcL = detectCnr(render);
                hold on;plot(cbcL(1,:),cbcL(2,:),'.r');
            else
                hold on; plot(Vxy(:,1), Vxy(:,2), '*r');
            end
        catch
            asgkj = 1;
        end
        drawnow;
    end
%     figure; imshow(render)
end
%% View the zbuffer and foreground mask
return;
[cameraparams,renderparams] = MR_default_params(obj.V,400);

[~,zbuffer] = MR_render_mesh(obj.F,obj.V,cameraparams,renderparams);

figure; imshow(zbuffer,[]);

foregroundmask = ~isinf(zbuffer);

figure; imshow(foregroundmask);

%% Modify camera parameters to perspective with distortion

[cameraparams,renderparams] = MR_default_params(obj.V,400);

cameraparams.type = 'perspectiveWithDistortion';
cameraparams.k1 = 10; % 0.179443360170886; % 10;
cameraparams.k2 = 200; %-0.221316927016224; %2000;
cameraparams.k3 = 0; %200;
cameraparams.p1 = 0; %-0.001796423328993; % 0;
cameraparams.p2 = 0; %0.003308372933534; %0;
cameraparams.f = cameraparams.f.*0.6;
render = MR_render_mesh(obj.F,obj.V,cameraparams,renderparams);

figure; imshow(render)

%% Modify some rendering parameters, switch on shadows

[cameraparams,renderparams] = MR_default_params(obj.V,400);

% Set the light source colour to white
renderparams.sourcecolour = [1;1;1];
% Switch to a distant point source coming from slightly right and above
renderparams.lightingModel = 'distantPointSource';
renderparams.sourcedirection = [-1 1 0];
% Make the surface shinier
renderparams.shininess = 100;
renderparams.ka = 0; % 0;
renderparams.kd = 0.8;
renderparams.ks = 0.8;
% Use a single colour for the whole mesh
renderparams.textureMode = 'usePerMeshColour';
% % renderparams.textureMode = 'usePerVertexColours';
renderparams.permeshcolour = [1;0.5;0.5];
% Don't light backfaces
renderparams.backfacelighting = 'unlit';
% Switch on shadow mapping
renderparams.shadows = true;

[render,zbuffer,texbuffer,normalbuffer,viewbuffer,sourcebuffer,fbuffer,wbuffer,shadowbuffer,visibility] = MR_render_mesh(obj.F,obj.V,cameraparams,renderparams);

figure; imshow(render)
end
function cbcL = detectCnr(imgL1)
[cbcX1, cbcY1, corner1] = DetectCbCorner(rgb2gray(imgL1));
[initCbcX1, initCbcY1] = CbCoordSys(cbcX1, cbcY1);
cbcL = cornerfinder([initCbcX1(:),initCbcY1(:)]',double(rgb2gray(imgL1)),5,5);
end

function [XYZ] = GetXYZFromDepth(intrMat, Pix,depthList)
metricPrevPtCcsGT = intrMat\HomoCoord(Pix',1);
metricPrevPtCcsGT = normc(metricPrevPtCcsGT);
if 1
    if 0
        if 0
            scaleAllGT = depthListGT(inlierId)./metricPrevPtCcsGT(3,:)';
        else
            scaleAllGT = depthListGT(:)./metricPrevPtCcsGT(3,:)';
        end
    else
        scaleAllGT = depthList./metricPrevPtCcsGT(3,:)';
    end
else
    dispGTComp = dispList(inlierId) - disparityErrorRound;
    depthListGTComp = intrMat(1,1).*norm(obj.camModel.transVec1To2)./(dispGTComp + (princpPtR(1) - princpPtL(1)));
    scaleAllGT = depthListGTComp./metricPrevPtCcsGT(3,:)';
end

XYZ = [repmat(scaleAllGT',3,1).*metricPrevPtCcsGT]';




end
