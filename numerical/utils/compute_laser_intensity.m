function laser_intensity = compute_laser_intensity(state, system_parameters, drone_number)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

[rovs_state, drones_state, ~] = state.get_state();
drone_state = drones_state(drone_number, :)';
drone_pos = [drone_state(1); drone_state(3); drone_state(5)];

P = system_parameters.laser_power;
n = double(system_parameters.beam_order);
Theta = system_parameters.laser_divergence;

intensity_ii = [];

for ii = 1:system_parameters.num_rovs
    % compute the relative distance btw each ROV and drone "drone_number"
    % in xy-plane (i.e. projection of cartesian space looking directly down
    % at water)
    rov_state_ii = rovs_state(ii, :)';
    rov_pos_ii = [rov_state_ii(1); rov_state_ii(3); rov_state_ii(5)];

    %
    relative_pos_centered_at_drone = rov_pos_ii - drone_pos;
    [~,elev,~] = cart2sph(relative_pos_centered_at_drone(1),relative_pos_centered_at_drone(2),relative_pos_centered_at_drone(3));

    %
    rand_noise = random('Normal', 0, system_parameters.laser_noise_std);
    intensity_ii(end+1) = rand_noise;

    if ~(elev > system_parameters.drone_fov / 2.0 - pi / 2.0)
        relative_pos = drone_pos - rov_pos_ii; % position of the drone relative to the iith ROV in Cartesian
        r = sqrt(relative_pos(1)^2 + relative_pos(2)^2);
        z = relative_pos(3);
    
        sigma = (1/10)^(-1/n) * z * sin(Theta / 2);
        I0_z = n * P / (2 * pi * 4^(1/n) * sigma^2 * gamma(2/n));
        I = I0_z * exp(-1/2 * (r / sigma)^n);
        intensity_ii(end) = I;
        % could compute the SNR here, just out of interest.
    end

    
end

laser_intensity = sum(intensity_ii);

end