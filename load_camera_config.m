function cam_conf_struct = load_camera_config(filename)
%%
%   loads the ZED camera config file
%   current figure.
%
%   input:
%       filename:        filename if camera config file
%       
%

% Read the cam.conf file data
fid = fopen(filename, 'r');
data = textscan(fid, '%s %f','Delimiter',"=");
fclose(fid);

% Create a structure array and save cam.conf as struct
cam_conf_struct = struct();
current_key = '';
current_index = 1;

%iterate through camera conf
for i = 1:length(data{1})
    item = data{1}{i};
    if startsWith(item, '[') && endsWith(item, ']')
        current_key = item(2:end-1);
        current_index = current_index +1;
        cam_conf_struct.(current_key) = struct();
    else
        cam_conf_struct.(current_key).(item) = data{2}(current_index);
        current_index = current_index + 1;
    end
end

