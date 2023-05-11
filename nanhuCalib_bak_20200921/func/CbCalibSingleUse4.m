function [camParam, cbcXY, cbGrid, config, calib_data, cbcXYTmp, cbGridTmp, goodId] = CbCalibSingleUse4(cbImgList,cbcXYDetected,calibFuncDir,modeE, varargin)
% CbCalibSingle automatically calibrate one camera's intrinsic parameter
% and extrinsic parameter list according to the images of a checkerboard
% cbImgList is the name list (cell array) of image files contining a
% checkerboard pattern
% Returned camParam is a structure containing the intrinsic parameters and
% a set of extrinsic parameters with each corresponding to an image of
% checkerboard
% By Ji Zhou

global cfg

set_up_global
if (nargin == 4)
    config = [];
elseif (nargin == 5)
    config = varargin{1};
elseif (nargin == 6)
    config = varargin{1};
    cfg = varargin{2};
else
    error('Too many input arguments');
end
config = SetDefaultConfig(config);


if length(cbImgList) >= 30000
    plotCorner = 1;
else
    plotCorner = 0;
end


dX = config.dX;
dY = config.dY;
halfWinW = config.halfWinW;
halfWinH = config.halfWinH;
estAspectRatio = config.estAspectRatio;



calib_data.active_images = ones(1, length(cbImgList));
calib_data.ind_active = [1: length(cbImgList)];
calib_data.ind_read = [1: length(cbImgList)];
calib_data.taylor_order_default = 4;

if modeE == 1
    calib_data.n_sq_x = cfg.cb_row - 1; 8; %% vert
    calib_data.n_sq_y = cfg.cb_col - 1;  13;
else
    calib_data.n_sq_x = cfg.cb_col - 1; 7; 13; %% hori
    calib_data.n_sq_y = cfg.cb_row - 1;  5 ;8;
    
% % % % %     calib_data.n_sq_x = 8; %% hori
% % % % %     calib_data.n_sq_y = 6;
    
end

calib_data.dX = dX;
calib_data.dY = dY;
for iPic = 1:length(cbImgList)
    calib_data.L{1,iPic} = cbImgList{iPic};
    
end
baseDir = pwd;

goodId = [];
nImg = length(cbImgList);
cbcXY = cell(nImg, 1);
cbGrid = cell(nImg, 1);

if (~isfield(config, 'cbcXY') || ~isfield(config, 'cbGrid'))
    % Detect corner points in the chessboard images
    for iImg = 1:nImg
        cd (baseDir);
        cbImg = imread(cbImgList{iImg});
        
%         if cfg.isRotate
%             
%             cbImg = imrotate(cbImg, 90);
%         end
        
        if (size(cbImg, 3) > 1)
            cbImg = rgb2gray(cbImg);
        end
        % %     [initCbcX, initCbcY] = DetectCbCorner(cbImg);
        %%
        if 1
            try
% % % % % % % %                         a;
                [callBack, initCbcX, initCbcY]  = get_checkerboard_corners4(iImg,0,calib_data,calibFuncDir);
                if size(initCbcX,1)*size(initCbcX,2) ~= (calib_data.n_sq_x + 1)*(calib_data.n_sq_y + 1)
                    dddd
                end
                %                 if size(initCbcX,1) ~= calib_data.n_sq_y + 1;
                %                     initCbcX = initCbcX';
                %                     initCbcY = initCbcY';
                %                 end
            catch
                if 1
                    %% 20210607
                    cd (baseDir);
                    continue;
                end
                if 0
                    cbImg0 = double(cbImg);
                    cbImg0(cbImg>110) = 1;
                    cbImg0 = 255.*cbImg0./max(cbImg0(:));
                    cbImg = uint8(cbImg0);
                end
                try
                    [initCbcX, initCbcY] = DetectCbCorner(cbImg);
                catch
                    if 0 % 20201013
                        camParam = [];
                        return;
                    else
                        cd (baseDir);
                       continue; 
                    end
                end
            end
            cd (baseDir);
            if iImg == 9
                akdbejv = 1;
            end
            if ~isempty(initCbcX) && ~isempty(initCbcY)
                if size(initCbcX,1) ~= calib_data.n_sq_y + 1;
                    initCbcX = initCbcX';
                    initCbcY = initCbcY';
                end
                try
                    if initCbcX(1,1) > initCbcX(1,end)
                        initCbcX = fliplr(initCbcX);
                        initCbcY = fliplr(initCbcY);
                        
                    end
                catch
                    jghl = 1;
                end
                if initCbcY(1,1) > initCbcY(end,1)
                    initCbcX = flipud(initCbcX);
                    initCbcY = flipud(initCbcY);
                    
                end
                %%
                [initCbcX, initCbcY] = CbCoordSys(initCbcX, initCbcY);
            end
            % With right hand rule, the z should point upward
        else
            initCbcX = reshape(cbcXYDetected{iImg}(1,:),calib_data.n_sq_y+1,calib_data.n_sq_x+1);
            initCbcY = reshape(cbcXYDetected{iImg}(2,:),calib_data.n_sq_y+1,calib_data.n_sq_x+1);
        end
        
        nPtX = size(initCbcX, 1);
        nPtY = size(initCbcX, 2);
        nPtX = calib_data.n_sq_y+1;
        nPtY = calib_data.n_sq_x+1;
        
        
        [nImgR, nImgC] = size(cbImg);
        [gridX, gridY] = meshgrid((0:nPtX-1)*dX, (0:nPtY-1)*dY);
        gridX = gridX'; gridY = gridY';
        
        % % % % % switch position
        cbGrid{iImg} = [gridY(:),gridX(:)]';
        %    cbcXY{iImg} = cornerfinder([initCbcX(:),initCbcY(:)]',double(cbImg),halfWinW,halfWinH) - 1; % zero based
        cbcXY{iImg} = cornerfinder([initCbcX(:),initCbcY(:)]',double(cbImg),halfWinW,halfWinH);
        %     good = checkCalib(cbcXY{iImg},cbcXY{iImg},calib_data,cbImg,cbImg);
        try
            good = checkCorner(cbcXY{iImg},cbcXY{iImg},calib_data,cbImg,cbImg);
        catch
            good = 0;
        end
        % %     good = 0;
        if good == 1;
            %             camParam = [];
            goodId = [goodId; iImg];
            %             return;
        end
        if ~isempty(cbcXY{iImg})
            if plotCorner == 1
                map = gray(256); figure(iImg), image(cbImg); hold on; colormap(map); axis equal;
                plot(cbcXY{iImg}(1, :), cbcXY{iImg}(2,:), '-+', cbcXY{iImg}(1, 1), cbcXY{iImg}(2,1), 'o', 'color',[1.000 0.314 0.510],'linewidth',2);
                plot(cbcXY{iImg}(1, (1:nPtX)), cbcXY{iImg}(2, (1:nPtX)),'-b','linewidth',2);
                plot(cbcXY{iImg}(1, (nPtX)), cbcXY{iImg}(2, (nPtX)),'*y','linewidth',2);
                hold off
                drawnow;
            end
        end
    end
    
else
    % for debug
    cbImg = imread(cbImgList{1});
    if (size(cbImg, 3) > 1)
        cbImg = rgb2gray(cbImg);
    end
    [nImgR, nImgC] = size(cbImg);
    cbcXY = config.cbcXY;
    cbGrid = config.cbGrid;
end
cbcXYTmp = cbcXY;
cbGridTmp = cbGrid;
cbcXY = cbcXY(goodId);
cbGrid = cbGrid(goodId);
nImg = length(goodId);

if ~cfg.check_depth
    [initFoc, initCen, initK, initAlpha] = InitIntrinsic(cbcXY, cbGrid, nImgC, nImgR, estAspectRatio);
    [rotVec, tranVec, ~] = InitExtrinsic(cbcXY, cbGrid, initFoc, initCen, initK, initAlpha);
    
    rtVec = [rotVec; tranVec];
    initParam = [initFoc; initCen; initAlpha; initK; zeros(5,1); rtVec(:)];
    
    try
        param = OptimizeIter(cbcXY, cbGrid, initParam, nImgC, nImgR, config);
        
        paramErr = EstimateError(param, cbcXY, cbGrid, config);
        
        % intrinsic param and error
        camParam.foc = param(1:2);
        camParam.focErr = paramErr(1:2);
        camParam.cen = param(3:4);
        camParam.cenErr = paramErr(3:4);
        camParam.alpha = param(5);
        camParam.alphaErr = paramErr(5);
        camParam.kc = param(6:10);
        camParam.kcErr = paramErr(6:10);
        % extrinc param and error
        camParam.rotVec = zeros(3,nImg);
        camParam.rotVecErr = zeros(3,nImg);
        camParam.tranVec = zeros(3,nImg);
        camParam.tranVecErr = zeros(3,nImg);
        for iImg = 1:nImg
            camParam.rotVec(:,iImg) = param(15+6*(iImg-1) + 1:15+6*(iImg-1) + 3);
            camParam.rotVecErr(:,iImg) = paramErr(15+6*(iImg-1) + 1:15+6*(iImg-1) + 3);
            camParam.tranVec(:,iImg) = param(15+6*(iImg-1) + 4:15+6*(iImg-1) + 6);
            camParam.tranVecErr(:,iImg) = paramErr(15+6*(iImg-1) + 4:15+6*(iImg-1) + 6);
        end
    catch
        camParam = [];
    end
else
    camParam = [];
end

end


function [initFoc, initCen, initK, initAlpha] = InitIntrinsic(cbcXY, cbGrid, nImgC, nImgR, estAspectRatio)

nImg = length(cbcXY);
% initialize at the center of the image
initCen = [nImgC; nImgR]/2-0.5;
% initialize to zero (no distortion)
initK = [0;0;0;0;0];
subCenMat = [1,0,-initCen(1)
    0,1,-initCen(2)
    0,0,1];

A = zeros(2*nImg, 2);
b = zeros(2*nImg, 1);
for iImg = 1:nImg
    homo = compute_homography(cbcXY{iImg}, cbGrid{iImg});
    % transform center to origin
    homo = subCenMat * homo;
    % Extract vanishing points (direct and diagonals)
    vanMat = homo * [1,0,0.5,0.5; 0,1,0.5,-0.5; 0,0,0,0];
    normVec = sqrt(sum(vanMat.^2));
    vanMat = vanMat./normVec(ones(3,1), :);
    A((iImg-1)*2+1:iImg*2, :) = [vanMat(1,1)*vanMat(1,2), vanMat(2,1)*vanMat(2,2); vanMat(1,3)*vanMat(1,4), vanMat(2,3)*vanMat(2,4)];
    b((iImg-1)*2+1:iImg*2) = -[vanMat(3,1)*vanMat(3,2); vanMat(3,3)*vanMat(3,4)];
end

% use all the vanishing points to estimate focal length
% Different fx, fy (two) or the same (one)
if (b'*(sum(A, 2)) < 0)
    % Use a two focals estimate:
    initFoc = sqrt(abs(1./((A'*A)\(A'*b)))); % if using a two-focal model for initial guess
else
    initFoc = sqrt(b'*(sum(A, 2))/(b'*b)) * ones(2,1); % if single focal length model is used
end

if ~estAspectRatio
    initFoc = mean(initFoc)*one(1,2);
end

initAlpha = 0;

initIntrinsicMat = [ ...
    initFoc(1)    initAlpha*initFoc(1)   initCen(1)
    0             initFoc(2)             initCen(2)
    0             0                      1];

end

function [rotVec, tranVec, rotMat] = InitExtrinsic(cbcXY, cbGrid, foc, cen, k, alpha)

condTh = 1e6; % threshold of conditional number

nImg = length(cbcXY);
rotVec = zeros(3, nImg);
tranVec = zeros(3, nImg);
rotMat = zeros(3,3,nImg);
for iImg = 1:nImg
    gridHomo = [cbGrid{iImg}; zeros(1, size(cbGrid{iImg}, 2))];
    [initRotVec,initTranVec] = compute_extrinsic_init(cbcXY{iImg},gridHomo,foc,cen,k,alpha);
    [rotVec(:,iImg),tranVec(:,iImg),rotMat(:,:,iImg),jacMat] = ...
        compute_extrinsic_refine(initRotVec,initTranVec,cbcXY{iImg},gridHomo,foc,cen,k,alpha,20,condTh);
    if (cond(jacMat)> condTh)
        error('View #%d ill-conditioned', iImg);
    end;
end

end

function param = OptimizeIter(cbcXY, cbGrid, initParam, imgW, imgH, config)

condTh = 1e6; % threshold of conditional number
maxIter = 30;
change = 1;
iter = 0;
alpha_smooth = 0.1;
estFocalLen = config.estFocalLen;
centerOptim = config.centerOptim;
estAlpha = config.estAlpha;
estDistortion = config.estDistortion;
param = initParam;


nImg = length(cbcXY);
selected_variables = [estFocalLen; centerOptim*ones(2,1); estAlpha; estDistortion; zeros(5,1); ...
    ones(6*nImg, 1)];
ind_Jac = find(selected_variables)';
while (change > 1e-9 && iter < maxIter)
    foc = param(1:2);
    cen = param(3:4);
    alpha = param(5);
    k = param(6:10);
    
    JJ3 = sparse([],[],[], 15+6*nImg, 15+6*nImg);
    ex3 = zeros(15+6*nImg, 1);
    
    % The first step consists of updating the whole vector of knowns
    % (intrinsic + extrinsic of active images) through a one step steepest
    % gradient descent.
    for iImg = 1:nImg
        gridHomo = [cbGrid{iImg}; zeros(1, size(cbGrid{iImg}, 2))];
        r = param(15+6*(iImg-1) + 1:15+6*(iImg-1) + 3);
        t = param(15+6*(iImg-1) + 4:15+6*(iImg-1) + 6);
        %% important function
        [xPrj,dxdr,dxdt,dxdf,dxdc,dxdk,dxdalpha] = project_points2(gridHomo,r,t,foc,cen,k,alpha);
        xErr = cbcXY{iImg} - xPrj;
        A = [dxdf dxdc dxdalpha dxdk]';
        B = [dxdr dxdt]';
        if (cond(B') > condTh)
            error('View #%d ill-conditioned', iImg);
        end
        JJ3(1:10,1:10) = JJ3(1:10,1:10) + sparse(A*A');
        JJ3(15+6*(iImg-1) + 1:15+6*(iImg-1) + 6,15+6*(iImg-1) + 1:15+6*(iImg-1) + 6) = sparse(B*B');
        
        AB = sparse(A*B');
        JJ3(1:10,15+6*(iImg-1) + 1:15+6*(iImg-1) + 6) = AB;
        JJ3(15+6*(iImg-1) + 1:15+6*(iImg-1) + 6,1:10) = (AB)';
        
        ex3(1:10) = ex3(1:10) + A*xErr(:);
        ex3(15+6*(iImg-1) + 1:15+6*(iImg-1) + 6) = B*xErr(:);
    end
    
    JJ3 = JJ3(ind_Jac,ind_Jac);
    ex3 = ex3(ind_Jac);
    alpha_smooth2 = 1-(1-alpha_smooth)^(iter+1);
    param_innov = alpha_smooth2*(JJ3\ex3);
    param(ind_Jac) = param(ind_Jac) + param_innov;
    
    % Second step: (optional) - It makes convergence faster, and the
    % region of convergence LARGER!!!
    % Recompute the extrinsic parameters only using compute_extrinsic.m
    % (this may be useful sometimes)
    % The complete gradient descent method is useful to precisely update
    % the intrinsic parameters.
    focCurr = param(1:2);
    cenCurr = param(3:4);
    if 0% (centerOptim && (cenCurr(1) < 0 || cenCurr(1) > imgW || cenCurr(2) < 0 || cenCurr(2) > imgH))
        error('Principal point cannot be estimated.');
    end
    
    alphaCurr = param(5);
    kcCurr = param(6:10);
    change = norm([focCurr;cenCurr] - [foc;cen])/norm([focCurr;cenCurr]);
    
    % Recompute extrinsic
    for iImg = 1:nImg
        gridHomo = [cbGrid{iImg}; zeros(1, size(cbGrid{iImg}, 2))];
        [r, t] = compute_extrinsic_init(cbcXY{iImg},gridHomo,focCurr,cenCurr,kcCurr,alphaCurr);
        [r, t, ~, jacMat] = compute_extrinsic_refine(r, t, cbcXY{iImg}, gridHomo, focCurr,cenCurr,kcCurr,alphaCurr,20,condTh);
        if (cond(jacMat)> condTh)
            error('View #%d ill-conditioned', iImg);
        end;
        param(15+6*(iImg-1) + 1:15+6*(iImg-1) + 3) = r;
        param(15+6*(iImg-1) + 4:15+6*(iImg-1) + 6) = t;
    end
    
    iter = iter + 1;
end

end

function paramErr = EstimateError(param, cbcXY, cbGrid, config)

foc = param(1:2);
cen = param(3:4);
alpha = param(5);
kc = param(6:10);
estFocalLen = config.estFocalLen;
centerOptim = config.centerOptim;
estAlpha = config.estAlpha;
estDistortion = config.estDistortion;

nImg = length(cbcXY);
xErr = [];

JJ3 = sparse([],[],[], 15+6*nImg, 15+6*nImg);
selected_variables = [estFocalLen; centerOptim*ones(2,1); estAlpha; estDistortion; zeros(5,1); ...
    ones(6*nImg, 1)];
ind_Jac = find(selected_variables)';
for iImg = 1:nImg
    gridHomo = [cbGrid{iImg}; zeros(1, size(cbGrid{iImg}, 2))];
    rotVec = param(15+6*(iImg-1) + 1:15+6*(iImg-1) + 3);
    tranVec = param(15+6*(iImg-1) + 4:15+6*(iImg-1) + 6);
    
    [xPrj,dxdr,dxdt,dxdf,dxdc,dxdk,dxdalpha] = project_points2(gridHomo,rotVec,tranVec,foc,cen,kc,alpha);
    xErr = [xErr, cbcXY{iImg} - xPrj];
    
    A = [dxdf dxdc dxdalpha dxdk]';
    B = [dxdr dxdt]';
    JJ3(1:10,1:10) = JJ3(1:10,1:10) + sparse(A*A');
    JJ3(15+6*(iImg-1) + 1:15+6*(iImg-1) + 6,15+6*(iImg-1) + 1:15+6*(iImg-1) + 6) = sparse(B*B');
    
    AB = sparse(A*B');
    JJ3(1:10,15+6*(iImg-1) + 1:15+6*(iImg-1) + 6) = AB;
    JJ3(15+6*(iImg-1) + 1:15+6*(iImg-1) + 6,1:10) = (AB)';
end

sigma_x = std(xErr(:));

JJ3 = JJ3(ind_Jac,ind_Jac);

JJ2_inv = inv(JJ3); % not bad for sparse matrices!!

paramErr = zeros(6*nImg+15,1);
paramErr(ind_Jac) =  3*sqrt(full(diag(JJ2_inv)))*sigma_x;

end

function config = SetDefaultConfig(config)

% dX and dY is the width and height in mm of a white or black rectangle
dxdy = isfield(config, {'dX', 'dY'});
if all(~dxdy)
    config.dX = 100;
    config.dY = 100;
elseif (dxdy(1) && ~dxdy(2))
    config.dY = config.dX;
elseif (dxdy(2) && ~dxdy(1))
    config.dX = config.dY;
end

% halfWinW and halfWinH are the width and height of the rectangular area
% for corner search
wh = isfield(config, {'halfWinW', 'halfWinH'});
if all(~wh)
    config.halfWinW = 5;
    config.halfWinH = 5;
elseif (wh(1) && ~wh(2))
    config.halfWinH = config.halfWinW;
elseif (wh(2) && ~wh(1))
    config.halfWinW = config.halfWinH;
end

if ~isfield(config, 'estAspectRatio')
    config.estAspectRatio = 1;
end

if ~isfield(config, 'estFocalLen')
    config.estFocalLen = [1;1];
end

if ~isfield(config, 'centerOptim')
    config.centerOptim = 1;
end

if ~isfield(config, 'estAlpha')
    config.estAlpha = 0;
end

if ~isfield(config, 'estDistortion')
    config.estDistortion = [1;1;1;1;0];
%         config.estDistortion = [1;1;1;1;1];
end

end