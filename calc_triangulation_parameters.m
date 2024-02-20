function [b, u0, v0, f]  = calc_triangulation_parameters(reprojection_matrix)
%
%   input
%       reprojection_matrix, which is returned when using rectifyStereoImages

% extraction of parameters for triangulation
b = 1 / reprojection_matrix(4, 3) * 1e-3;
u0 = -reprojection_matrix(1, 4);
v0 = -reprojection_matrix(2, 4);
f = reprojection_matrix(3, 4);

end

