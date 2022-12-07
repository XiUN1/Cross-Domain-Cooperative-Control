function animate_entropy(states, system_parameters)
%ANIMATE_SOLN

f = figure; % create the figure
ax = axes('Parent', f, 'position', [0.13 0.2 0.77 0.77]); % fix the axes dimensions
x = linspace(-double(system_parameters.grid_cols) * system_parameters.grid_unit_length / 2, double(system_parameters.grid_cols) * system_parameters.grid_unit_length / 2, system_parameters.grid_cols);
y = linspace(-double(system_parameters.grid_rows) * system_parameters.grid_unit_length / 2, double(system_parameters.grid_rows) * system_parameters.grid_unit_length / 2, system_parameters.grid_rows);
p = states{1}.rov_probability_matrix;
H = -(p .* log(p + 1e-16) + (1-p) .* log((1-p) + 1e-16));
h = mesh(x,y,H); % main plot function (pretend the int32 Matrix is an image)
view(0,90)
% clim([0 1])
colorbar;

hold on;
rov_states = states{1}.rovs_state;
rov_x = rov_states(1);
rov_y = rov_states(3);
% rov_z = rov_states(5);
scatter3(rov_x, rov_y, 5, 100, 'red', 'square', 'filled');
hold off;

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