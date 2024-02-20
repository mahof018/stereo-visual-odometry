function plot_features(u,v,win_size)
%%
%   Plots the features as red squares of size win_size in the
%   current figure.
%
%   input:
%       u,v:        coordinates of the features
%       win_size:   size of the squares
%

win_rad =(win_size-1)/2;
v =v(:);
u =u(:);

ud = [u-win_rad u+win_rad u+win_rad u-win_rad u-win_rad]';
vd = [v-win_rad v-win_rad v+win_rad v+win_rad v-win_rad]';

hold on;
plot(ud,vd,'Color','r','LineStyle','-');
hold off;
