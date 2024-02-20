function plot_matches(u1,v1,u2,v2,win_size)
%%
%   Plots the result of feature matching in the current figure.
%   The features (u1,v1) are plotted with plot_features().
%   The correspondence between (u1,v1) and (u2,v2) are
%   plotted with lines
%   
%   input:
%       u1,v1:      coordinates of the features in image 1
%       u2,v2:      coordinates of the features in image 2
%       win_size:   size of the squares for the features
%

plot_features(u1,v1,win_size);

hold on;
for ii = 1:1:length(v1)
    ud = [u1(ii) u2(ii)];
    vd = [v1(ii) v2(ii)];
    plot(ud,vd,'Color','r','LineStyle','-');
end
hold off;