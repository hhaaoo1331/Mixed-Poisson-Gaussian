 function p = prox_gamma_huber(x0, gamma, w)
%function p = prox_huber(x, gamma, w)
%
% This procedure computes the proximity operator of the function:
%
%           / gamma * ( |x|^2 ./ (2*w) )     if |x| <= w
%   f(x) = |                                                with w > 0
%           \ gamma * ( |x| - w/2 )          otherwise
%
% When the input 'x' is an array, the output 'p' is computed element-wise.
%
%  INPUTS
% ========
%  x     - ND array
%  gamma - positive, scalar or ND array with the same size as 'x'
%  w     - positive, scalar or ND array with the same size as 'x'





% check input
if any( gamma(:) <= 0 ) || ~isscalar(gamma) && any(size(gamma) ~= size(x))
    error('''gamma'' must be positive and either scalar or the same size as ''x''')
end
if any( w(:) <= 0 ) || ~isscalar(w) && any(size(w) ~= size(x))
    error('''w'' must be positive and either scalar or the same size as ''x''')
end
%-----%


% compute the "square" branch
p = (abs(x0)-gamma).*sign(x0);
% compute the "abs" branch
t = x0./(gamma/w + 1);

% select the branches
mask = abs(x0) < w*(gamma/w + 1);
p(mask) = t(mask);