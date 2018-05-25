function K = OptimalGain(Q,rho,R,varargin)
%OPTIMALGAIN Summary of this function goes here
%   Detailed explanation goes here

switch length(varargin)
    case 2
        C = eye(size(varargin{1}));
        D = [];
        sys = ss(varargin{1},varargin{2},C,D);
    case 3
        D = [];
        sys = ss(varargin{1},varargin{2},varargin{3},D);
    case 4 
        sys = ss(varargin{1},varargin{2},varargin{3},varargin{4});
    otherwise
        error('Error');
end

[K,~,~] = lqr(sys,Q,rho*R);
end

