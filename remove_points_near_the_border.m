function  features_removed_borders = remove_points_near_the_border(features,border_thresh,height_img,width_img)
% removes points that are near the edge, where distortion could not be handled as
% as in the middle
%  
%   input
%       features: features returned by detect...Features() from matlab
%       border_thresh: feature Locations near the image border which should
%       not be used
%       height_img,width_img: image width and height

 features_removed_borders =  features(features.Location(:, 1)> border_thresh& ...
                        features.Location(:, 1) < width_img - border_thresh & ...
                        features.Location(:, 2) > border_thresh & ...
                        features.Location(:, 2) < height_img - border_thresh);
end

