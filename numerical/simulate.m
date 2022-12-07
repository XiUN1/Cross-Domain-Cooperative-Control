addpath("utils")

% Define the system parameters
% - notice: the laser detection threshold is 3 laser noise sigma values
system_parameters = SystemParameters(int32(1), int32(3), 2.58*0.0025, 1, deg2rad(10), int32(2), 4, deg2rad(20), 0.0025, int32(200), int32(200), 0.1);

% Define control algorithm
controller = Controller(system_parameters);

% Define initial condition of numerical simulation
initial_state = State(system_parameters);
rng(10); % assign same random seed!
initial_state = initial_state.randomize(system_parameters);

% Define the time span of the numerical simulation
t0 = 0; tfinal = 10;
sample_time = 0.1; % in [seconds]
% samples = t0:sample_time:tfinal;
samples = linspace(t0, tfinal, (tfinal - t0) / sample_time);

states = {initial_state};

% Discrete-Time simulation...
for k = 1:length(samples)
    current_state = states{k};
    next_state = State(system_parameters);

    % Update ROV probability matrix to account
    filter_radius = ceil(system_parameters.rov_max_speed / system_parameters.grid_unit_length); % yeah
    conv_filter = fspecial('disk', filter_radius);
    convoluted = conv2(current_state.rov_probability_matrix, conv_filter, 'same');
%     med = median(median(convoluted));
%     convoluted(1:filter_radius, :) = med * ones(size(convoluted(1:filter_radius,:)));
%     convoluted(end-9:end, :) = med * ones(size(convoluted(end-9:end,:)));
%     convoluted(:, 1:filter_radius) = med * ones(size(convoluted(:, 1:filter_radius)));
%     convoluted(:, end-9:end) = med * ones(size(convoluted(:, end-9:end)));
%     convoluted = convoluted / sum(sum(convoluted));
    next_state.rov_probability_matrix = convoluted;
    assert(sum(sum(next_state.rov_probability_matrix)) - 1.0 < 1e-12);

    % Compute Intensity of each Drone (i.e. each receiver)
    % - the intensity of any given receiver is just a function of relative
    % position of drones and ROVs
    for ii = 1:system_parameters.num_drones
        next_state.laser_intensity(ii) = compute_laser_intensity(current_state, system_parameters, ii);
    end
    
    % Modify ROV probability matrix to account for observations
    for ii = 1:system_parameters.num_drones
        [~, drones_state, ~] = current_state.get_state;
        [~, ~, rov_probability_matrix] = next_state.get_state;

        assert(sum(sum(rov_probability_matrix - 1 < 1e-12)));
        
        drone_state_ii = drones_state(ii,:);
        drone_state_ii = drone_state_ii';

        % find the radius of the FOV projected on the surface of the
        % water.
        z = drone_state_ii(5); % vertical distance of drone "ii" above water
        FOV_radius = z * tan(system_parameters.drone_fov);
        FOV_prj_water_surface = polyshape_circle(FOV_radius, drone_state_ii(1), drone_state_ii(3), 100); % 100 seems pretty good
        min_x = drone_state_ii(1) - FOV_radius;
        max_x = + drone_state_ii(1) + FOV_radius;
        min_y = drone_state_ii(3) - FOV_radius;
        max_y = drone_state_ii(3) + FOV_radius;
%         plot(FOV_prj_water_surface); pause;

        x = linspace(-double(system_parameters.grid_cols) * system_parameters.grid_unit_length / 2, double(system_parameters.grid_cols) * system_parameters.grid_unit_length / 2, system_parameters.grid_cols);
        min_x_index = find(sign(x - min_x) - 1, 1, 'last'); % find the last negative diff btw x and min_x
        max_x_index = find(sign(x - max_x) + 1, 1, 'first');
        y = linspace(-double(system_parameters.grid_rows) * system_parameters.grid_unit_length / 2, double(system_parameters.grid_rows) * system_parameters.grid_unit_length / 2, system_parameters.grid_rows);
        min_y_index = find(sign(y - min_y) - 1, 1, 'last'); % find the last negative diff btw x and min_x
        max_y_index = find(sign(y - max_y) + 1, 1, 'first');

        min_x_index = max(1, min_x_index);
        max_x_index = min(length(x)-1, max_x_index);
        min_y_index = max(1, min_y_index);
        max_y_index = min(length(y)-1, max_y_index);

        % if the laser intensity for drone 'ii' is > threshold, then we
        % have detected an ROV
        if next_state.laser_intensity(ii) > 0.01 % arbitrary threshold for testing purposes
            % drone "ii" detected an ROV
            rov_probability_matrix = zeros(size(rov_probability_matrix)); %  this assumes single ROV. must modify later
            modified_squares = zeros(size(rov_probability_matrix));

            for jj = min_y_index:max_y_index
                for kk = min_x_index:max_x_index
                    grid_jk = polyshape([x(kk) x(kk+1) x(kk+1) x(kk)], [y(jj) y(jj) y(jj+1) y(jj+1)]);
                    intersection = intersect(grid_jk, FOV_prj_water_surface);
                    assert(intersection.NumRegions == 0 || intersection.NumRegions == 1);
                    if intersection.NumRegions == 1
                        area_ratio = area(intersection) / area(FOV_prj_water_surface);

                        % what this means is, if area_ratio = 0.0042, for 
                        % example, then 0.42% of the area of the FOV is
                        % taken up by this current grid.

                        rov_probability_matrix(jj, kk) = area_ratio; % now this assumes single ROV, must modify later
                        modified_squares(jj, kk) = 1.0;
                    end
                end
            end
            
            rov_probability_matrix = rov_probability_matrix .* modified_squares;
        else
            % drone "ii" did not detect an ROV
            previous_probability = 0; % name this something better...
            modified_squares = ones(size(rov_probability_matrix));

            for jj = min_y_index:max_y_index
                for kk = min_x_index:max_x_index
                    grid_jk = polyshape([x(kk) x(kk+1) x(kk+1) x(kk)], [y(jj) y(jj) y(jj+1) y(jj+1)]);
                    intersection = intersect(grid_jk, FOV_prj_water_surface);
                    assert(intersection.NumRegions == 0 || intersection.NumRegions == 1);
                    if intersection.NumRegions == 1
                        % if the intersection is non-empty set, then...
                        area_ratio = area(intersection) / area(grid_jk);

                        % what this means is, if area_ratio = 0.0042, for 
                        % example, then 0.42% of the area of the grid is
                        % taken up by the current FOV.
                        previous_probability = previous_probability + rov_probability_matrix(jj, kk) * area_ratio;
                        rov_probability_matrix(jj, kk) = rov_probability_matrix(jj, kk) * (1 - area_ratio);  % works for single ROV, modify later
                        modified_squares(jj, kk) = 0.0;
                    end
                end
            end

            % how much to increment the probability by:
            probability_increment = previous_probability / length(find(modified_squares));
            rov_probability_matrix = rov_probability_matrix + probability_increment * modified_squares;
        end

        assert(sum(sum(rov_probability_matrix)) - 1.0 < 1e-12);
        next_state.rov_probability_matrix = rov_probability_matrix;
    end

    % Update ROV state for next sample time (shouldn't touch anymore)
    for ii = 1:system_parameters.num_rovs
        [rovs_state, ~, ~] = current_state.get_state;
        rov_state_ii = rovs_state(ii, :);
        rov_state_ii = rov_state_ii'; % turn row vector into column vector
        
        % x[k+1] = A[k] x[k] + w[k], w[k] ~ N(0,Q), Q = q * [T^3/3 T^2/2; T^2/2 T]
        A = [1 sample_time 0 0 0 0; 0 1 0 0 0 0; 0 0 1 sample_time 0 0; 0 0 0 1 0 0; 0 0 0 0 1 sample_time; 0 0 0 0 0 1];
        T = sample_time;
        Q = system_parameters.rov_noise_level * [T^3/3 T^2/2 0 0 0 0; T^2/2 T 0 0 0 0; 0 0 T^3/3 T^2/2 0 0; 0 0 T^2/2 T 0 0; 0 0 0 0 T^3/3 T^2/2; 0 0 0 0 T^2/2 T];
        w = mvnrnd(zeros(6,1), Q, 1)';
%         % idk?
%         w(2) = max(-system_parameters.rov_max_speed, min(system_parameters.rov_max_speed, w(2)));
%         w(4) = max(-system_parameters.rov_max_speed, min(system_parameters.rov_max_speed, w(4)));
%         w(6) = max(-system_parameters.rov_max_speed, min(system_parameters.rov_max_speed, w(6)));

        next_state_ii = A * rov_state_ii + w;

        % Update "next_state" object
        next_state.rovs_state(ii,:) = next_state_ii';
    end

    % Update drone state for next sample time (shouldn't touch anymore)
    u = controller.get_control(k, current_state, system_parameters, sample_time);
    for ii = 1:system_parameters.num_drones
        [~, drones_state, ~] = current_state.get_state;
        drone_state_ii = drones_state(ii,:);
        drone_state_ii = drone_state_ii';
        
        % x[k+1] = A[k] x[k] + B[k] u[k]
        A = [1 sample_time 0 0 0 0; 0 1 0 0 0 0; 0 0 1 sample_time 0 0; 0 0 0 1 0 0; 0 0 0 0 1 sample_time; 0 0 0 0 0 1];
        T = sample_time;
        B = [0 0 0; 1 0 0; 0 0 0; 0 1 0; 0 0 0; 0 0 1];

        next_state_ii = A * drone_state_ii + B * u(ii,:)';

        % update "next_state" object
        next_state.drones_state(ii,:) = next_state_ii';
    end

    states{k+1} = next_state; % must remain as final line of iteration
    disp(k / length(samples) * 100 + "% Done");
end


plot_rov_data(samples(1:length(states)-1), {states{2:end}}, 1);
plot_drone_data(samples(1:length(states)-1), {states{2:end}}, 2);
plot_laser_intensity_data(samples(1:length(states)-1), {states{2:end}}, 3);
animate_entropy({states{1:end-1}}, system_parameters)
