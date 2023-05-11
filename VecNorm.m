function n = VecNorm(v, dim)

% VecNorm computes the 2-norms of a list of vectors. 
% v: M by N matrix, a list of vectors. In which dimension it's a vector is
% specified by the second parameter dim.
% dim: integer scalar specifying in which dimension the data in v are seen
% as vectors. Default is 1.
% Returned n is a vector of size M (if dim == 2) or N (if dim == 1) which
% are the 2-norms of given vectors.
%
% By Ji Zhou

if ~exist('dim', 'var')
    dim = 1;
end

n = sqrt(dot(v,v,dim));

end