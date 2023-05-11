function [ T , Y ] = ICP( P , X )
%ICP Iterative Closest Point algorithm
%   [T,Y] = ICP(P,X) applies the Iterative Closest Point algorithm to the
%   matrix P so that it is registered to the model X. Since points in P may
%   not have the same order they have in model X, they are re-arranged so
%   that they match their best closest neighbour. Re-ordered and registered
%   P set is saved in Y, while T contains the homogeneous transform that
%   can register P to X reference frame (i.e. X = T*P)
%   P,X (and Y) must be 4-by-n matrices with points in homogeneous
%   coordinates (i.e. [x;y;z;1]).

if size(P,1)~=4 || size(X,1)~=4
    fprintf('Input matrices must be 4-by-N\n');
    return
end
    for i=1:size(P,2)
        if P(4,i)~=1
            fprintf('Input matrices must follow this convention:\n');
            fprintf('dimension:4-by-n; rows 1 to 3 are cartesian coordinates, row 4 contains only ones (homogeneous coordinates)\n');
            return
        end
    end
    for i=1:size(X,2)
        if X(4,i)~=1
            fprintf('Input matrices must follow this convention:\n');
            fprintf('dimension:4-by-n; rows 1 to 3 are cartesian coordinates, row 4 contains only ones (homogeneous coordinates)\n');
            return
        end
    end    
        
I_max = 1000;       % max num of iteration
threshold = 1e-99;  % accuracy threshold
T0 = eye(4);        % initial guess
T_k = T0;           % guess at iter k
iter = 0;           % # of the current iteration
fcost = 1000;       % function cost to be minimized
delta_fcost = 1000; % delta fcost between two iterations

k = 0;
P_0 = P;
T = eye(4);

while( delta_fcost > threshold && k < I_max )
    if k>0  % can we remove this if?
        Y_k = closest_points(P_k,X);
    else
        Y_k = closest_points(P_0,X);
    end
    T_k = quaternion_matching(Y_k,X);
        T = T_k*T;  % update wrt previous step
    P_k = T*P_0;
    
    Y = closest_points(P_k,X);  % need to re-arrange points again
    Y = [Y;ones(1,size(Y,2))];
    % the cost function is the sum of the RMS errors
    e = rms(Y(1:3,:)-P_k(1:3,:));
    fcost(k+1) = sum(e);
    % compute error riduction between two successive steps
    if k==0
        if T == eye(4)
            fprintf('P and X are the same set\n');
            return
        end
        delta_fcost = 1000;
    else
        delta_fcost = abs(fcost(k+1) - fcost(k));
    end
    k = k+1;
end

% Transform
display(['Stopped at iteration: ' num2str(k)])
display(['Cost function: ' num2str(fcost(end))])

end
function [ Y ] = closest_points( P, X )
% Find closest points between two sets of points.
%   P, X must be 4-by-n in homogeneous coordinates or 3-by-n in cartesian
%   coordinates.
%   [Y] = closest_points(P,X) will reorder points of P so they match the
%   same order of the relative closest neighbour in X and will save the
%   re-ordered P set in Y.
%   Works only if X and P have the same number of points. See issue #3 on
%   repository if you'd like to contribute.

P = P(1:3,:);
X = X(1:3,:);

n_points = size(P,2);
Y = zeros(3,n_points);

% mean value to remove errors and disturbances
P_mean = mean(P,2);
X_mean = mean(X,2);

P = P - repmat(P_mean, 1, n_points);
X = X - repmat(X_mean, 1, n_points);

for i = 1:n_points
    % create a matrix the same size of P with only one point of X,
    diff_matrix = (repmat(X(:,i), 1, n_points) - P);
    % find distances between all these points,
    distance = sqrt(sum(diff_matrix.^2, 1));
    % we are interested in the closest one,
    [~, index] = min(distance);
    % select that points (add mean value)
    Y(:, i) = P(:,index) + P_mean;
end
end
function [T] = quaternion_matching(P,X)
%Quaternion Matching computes homogeneous transformation
%   [T] = quaternion_matching(P,X)
%   P is a measured data point set to be aligned with a model point set X.
%   P and X must have homogeneous coordinates and be 4-by-n
%   (e.g. [x;y;z;1]), where n is the number of points. T is the transform
%   in homogeneous coordinates that will register points in P so they are
%   expressed in the same reference frame as X. If P and X are the same set
%   of points expressed in two different reference frames, then X = T*P.

P = P(1:3,:);   % back to cartesian coordinates
X = X(1:3,:);

N_P = size(P,2);    % number of points in P
N_X = size(X,2);    % number of points in X, currently unused

ni_P = mean(P,2);    % Center of mass
ni_X = mean(X,2);

% Cross-covariance matrix
Sigma_px = zeros(3);
for i = 1 : N_P
    Sigma_px = Sigma_px + ((P(:,i) - ni_P) * (X(:,i) - ni_X)');
end
Sigma_px = 1 / N_P * Sigma_px;

% Compute parameters for the quaternion extraction
    % Anti-symmetric matrix
    A = Sigma_px - Sigma_px';
    % cyclic components of the anti-symmetric matrix
    D = [A(2,3) A(3,1) A(1,2)]';
    % 4-by-4 symmetric matrix Q(Sigma_px)
    Q = [trace(Sigma_px) D';...
         D               (Sigma_px + Sigma_px' - trace(Sigma_px) * eye(3))];

% the unit eigenvector q_R corresponding to the maximum eigenvalue of the
% matrix Q(Sigma_px) is selected as the optimal rotation
    % Eigenval/Eigenvect extraction
    [E_vec,E_val] = eig(Q);

    % Find the maximum eigenval and its position
    [~,index] = max(diag(E_val));

    % Extract q_R
    q_R = E_vec(:,index);
    
% Calculates the quaternion <- this is a class
quat = Quaternion(q_R);
 
% Extract the homogeneous transform <- class method
T = quat.T; % <- has no trans component

% The optimal translation vector is given by
T(1:3,4) = (ni_X - (quat.R) * ni_P)';   % <- using class method
end