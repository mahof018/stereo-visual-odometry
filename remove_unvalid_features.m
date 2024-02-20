function varargout = remove_unvalid_feautures(varargin)
% Iterates through points and removes not inlier indices of unlimited
% number of feauture points
%   input
%       varargin:   a variable number of arguments, where each argument is
%                   a set of features (output of detect...Features matlab)
%
%                   except for the last one which is
%                       inlier: indices of the inlier points

    % indices of the inlier points
    inlier = varargin{end};
    % number of point sets
    num_sets = nargin - 1;
    points_valid = cell(1, num_sets);

    % Iteration over the point sets and removal of invalid indices
    for i = 1:num_sets
        points = varargin{i};
        size(points);
        points_valid{i} = points(inlier);
    end

    % return results
    varargout = points_valid;
end



