function nv = Normalize(v, dim)

% Nomalize normalizes vectors by dividing it by their norms.
% v: a M by N matrix, vectors to normalize. Which dim represents a vector
% depends on the second parameter, dim. By default a column is a vector.
% dim: an integer scalar indicating which dim of v represents a vector.
% Returned nv is a matrix with the same size as v, which are resultant
% normalized vectors.
%
% By Ji Zhou

if ~exist('dim', 'var')
    dim = 1;
end

nv = bsxfun(@rdivide, v, VecNorm(v, dim));

end