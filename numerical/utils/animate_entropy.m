function animate_entropy(states, system_parameters)
%ANIMATE_SOLN

f = figure; % create the figure
ax = axes('Parent', f, 'position', [0.13 0.2 0.77 0.77]); % fix the axes dimensions

p = states{1}.rov_probability_matrix;
H = -(p .* log(p + 1e-16) + (1-p) .* log((1-p) + 1e-16));
h = mesh(H); % main plot function (pretend the int32 Matrix is an image)
% clim([0 1])
colorbar;

% for ii = 1:system_parameters.num_drones
%     [~, drones_state, ~] = states{1}.get_state;
%     drone_state_ii = drones_state(ii,:);
%     drone_state_ii = drone_state_ii';
%     z = drone_state_ii(5); % vertical distance of drone "ii" above water
%     FOV_radius = z * sin(system_parameters.drone_fov);
%     FOV_prj_water_surface = polyshape_circle(FOV_radius, drone_state_ii(1), drone_state_ii(3), 100); % 100 seems pretty good
%     hold on; plot(FOV_prj_water_surface);
% end
% hold off

% start with move number "0" representing the starting scrambled board
sample_number = 1;

% draw the UI slider control at the bottom of the figure
b = uicontrol('Parent', f, 'Style', 'slider', ...
    'Position', [81,45,419,23], 'value', sample_number, 'min', 1, ...
    'max', length(states));
bgcolor = f.Color;
bl1 = uicontrol('Parent',f,'Style','text','Position',[50,45,23,23],...
                'String','0','BackgroundColor', bgcolor);
bl2 = uicontrol('Parent', f, 'Style', 'text', 'Position', [500,45,23,23],...
                'String', num2str(length(states)), 'BackgroundColor', bgcolor);
bl3 = uicontrol('Parent', f, 'Style', 'text', 'Position', [240,20,100,23],...
                'String', 'Sample #', 'BackgroundColor', bgcolor);

% create the slider callback function to redraw the figure with updates to
% the slider
b.Callback = @(es,ed) animate_entropy_callback(h, states, system_parameters, es.Value);

end