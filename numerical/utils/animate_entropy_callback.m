function animate_entropy_callback(h, states,  system_parameters, sample_number)
%ANIMATE_ENTROPY_CALLBACK 

sample_number = int32(sample_number);
x = linspace(-double(system_parameters.grid_cols) * system_parameters.grid_unit_length / 2, double(system_parameters.grid_cols) * system_parameters.grid_unit_length / 2, system_parameters.grid_cols);
y = linspace(-double(system_parameters.grid_rows) * system_parameters.grid_unit_length / 2, double(system_parameters.grid_rows) * system_parameters.grid_unit_length / 2, system_parameters.grid_rows);
p = states{sample_number}.rov_probability_matrix;
H = -(p .* log(p + 1e-16) + (1-p) .* log((1-p) + 1e-16));
h = mesh(x,y,H); % replot the exact same image as in animate_soln
view(0,90)
% clim([0 1.0])
colorbar;

hold on;
rov_states = states{sample_number}.rovs_state;
rov_x = rov_states(1);
rov_y = rov_states(3);
rov_z = rov_states(5);
scatter3(rov_x, rov_y, 5, 100, 'red', 'square', 'filled');
hold off;

end