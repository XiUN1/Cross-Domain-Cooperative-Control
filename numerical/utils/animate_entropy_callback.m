function animate_entropy_callback(h, states,  system_parameters, sample_number)
%ANIMATE_ENTROPY_CALLBACK 

sample_number = int32(sample_number);
p = states{sample_number}.rov_probability_matrix;
H = -(p .* log(p + 1e-16) + (1-p) .* log((1-p) + 1e-16));
h = mesh(H); % replot the exact same image as in animate_soln
% clim([0 1.0])
colorbar;


% for ii = 1:system_parameters.num_drones
%     [~, drones_state, ~] = states{sample_number}.get_state;
%     drone_state_ii = drones_state(ii,:);
%     drone_state_ii = drone_state_ii';
%     z = drone_state_ii(5); % vertical distance of drone "ii" above water
%     FOV_radius = z * sin(system_parameters.drone_fov);
%     FOV_prj_water_surface = polyshape_circle(FOV_radius, drone_state_ii(1), drone_state_ii(3), 100); % 100 seems pretty good
%     hold on; plot(FOV_prj_water_surface);
% end
% hold off

end