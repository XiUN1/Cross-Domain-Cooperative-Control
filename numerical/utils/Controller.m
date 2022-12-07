classdef Controller < handle
    %CONTROLLER Summary of this class goes here
    %   Detailed explanation goes here

    properties
        guidance_phase  % search track or track phase
    end

    methods
        function obj = Controller(system_parameters)
            obj.guidance_phase = GuidancePhase.Search;
        end

        function control = constrain_control(obj, control, params)
            for ii = 1:params.num_drones
                if norm(control(ii, :)) > 1
                    % if the norm of the desired control is > 1, normalize each row
                    control(ii, :) = control(ii,:) / norm(control(ii,:));
                end
            end
        end

        function out = get_control(obj, sample, state, system_parameters, sample_time, laser_intensity)
            % out = matrix of size (NUM_DRONES x 3) where columns are
            % (xdot, ydot, zdot)
            out = zeros(system_parameters.num_drones, 3);
            persistent u_old_z e_old_z u_old_x e_old_x u_old_y e_old_y;
            if isempty(u_old_z)
                u_old_z = zeros(1, system_parameters.num_drones);
            end

            if isempty(e_old_z)
                e_old_z = zeros(1, system_parameters.num_drones);
            end

            if isempty(u_old_x)
                u_old_x = zeros(1, system_parameters.num_drones);
            end

            if isempty(e_old_x)
                e_old_x = zeros(1, system_parameters.num_drones);
            end

            if isempty(u_old_y)
                u_old_y = zeros(1, system_parameters.num_drones);
            end

            if isempty(e_old_y)
                e_old_y = zeros(1, system_parameters.num_drones);
            end

            if obj.guidance_phase == GuidancePhase.Search
                % apply the entropy based search approach here...

                % recall we are applying a control input to "num_drones"
                % number of drones. So we may need a for-loop or
                % something...
                [~, drones_state, ~] = state.get_state();
                T = sample_time;
                
                % Compute z-direction control
                u_left_vec = -[0.506449719108587];
                e_left_vec = [95.871889860291304 -89.515570805993178];
                for ii = 1:system_parameters.num_drones
                    drone_state_ii = drones_state(ii, :);
                    drone_state_ii = drone_state_ii';

                    % z position
                    z = drone_state_ii(5);

                    % desired z
                    z_des = 10; % 15

                    u = (u_left_vec * u_old_z(:, ii) + e_left_vec * [(z_des - z); e_old_z(:,ii)]);
                    e_old_z(:, ii) = (z_des - z);
                    u_old_z(:, ii) = u;
                    out(ii, 3) = u;
                end

                % Use potential-field to guide to areas of high-entropy
                for ii = 1:system_parameters.num_drones
                    %
                    x = state.drones_state(ii, 1);
                    y = state.drones_state(ii, 3);

                    % omega = 3.21
                    x_des = double(ii - 1) * 7.5 - 7.5 + (20/3 - 5) * cos(1 * sample * sample_time);
                    y_des = 7.5 * cos(1.0 * sample * sample_time + double(ii - 1) * deg2rad(120));

                    u = (u_left_vec * u_old_y(:, ii) + e_left_vec * [(y_des - y); e_old_y(:,ii)]);
                    e_old_y(:, ii) = (y_des - y);
                    u_old_y(:, ii) = u;
                    out(ii, 2) = u;

                    u = (u_left_vec * u_old_x(:, ii) + e_left_vec * [(x_des - x); e_old_x(:,ii)]);
                    e_old_x(:, ii) = (x_des - x);
                    u_old_x(:, ii) = u;
                    out(ii, 1) = u;
                    % we want to go here [mir(mic), mic]
                end                

                % switch when we detect a target above a threshold
            elseif obj.guidance_phase == GuidancePhase.Track
                % "elseif" unnecessary since only other is "track", but for
                % clarity...
                disp("tracking on")

                % who is the detector who is lowest in the sky
                detectors = [];
                for ii = 1:system_parameters.num_drones 
                    if laser_intensity(ii) > 2.58 * 0.0025
                        detectors(end+1) = ii;
                    end
                end

                if isempty(detectors)
                    % turn into handle class?
                    obj.guidance_phase = GuidancePhase.Search;
                    out = obj.get_control(sample, state, system_parameters, sample_time, laser_intensity);
                else
                    % detectors is not empty, find the detector who is
                    % lowest in the sky
                    lowest_detector = detectors(1);
                    for ii = 2:length(detectors)
                        if state.drones_state(detectors(ii), 5) < state.drones_state(lowest_detector, 5)
                            lowest_detector = detectors(ii);
                        end
                    end

                    disp("lowest detector " + lowest_detector);

                    % lowest detector must hold position...

                    out(:, lowest_detector) = -[state.drones_state(lowest_detector, 2:2:6)]';

                    % other drones must fly to 1/2 the height and go to

                    [~, drones_state, ~] = state.get_state();
                    T = sample_time;
                    
                    % Compute z-direction control
                    u_left_vec = -[0.506449719108587];
                    e_left_vec = [95.871889860291304 -89.515570805993178];
                    for ii = 1:system_parameters.num_drones
                        if ii == lowest_detector
                            continue;
                        end
                        drone_state_ii = drones_state(ii, :);
                        drone_state_ii = drone_state_ii';
    
                        % z position
                        z = drone_state_ii(5);
    
                        % desired z
                        z_des = state.drones_state(lowest_detector, 5) / 2; % half height of other lowest
    
                        u = (u_left_vec * u_old_z(:, ii) + e_left_vec * [(z_des - z); e_old_z(:,ii)]);
                        e_old_z(:, ii) = (z_des - z);
                        u_old_z(:, ii) = u;
                        out(ii, 3) = u;
                    end
    
                    % Use potential-field to guide to areas of high-entropy
                    for ii = 1:system_parameters.num_drones
                        %
                        if ii == lowest_detector
                            continue;
                        end
                        x = state.drones_state(ii, 1);
                        y = state.drones_state(ii, 3);
    
                        % omega = 3.21
                        x_des = state.drones_state(lowest_detector, 1) + state.drones_state(lowest_detector, 5) / 2 * tan(system_parameters.drone_fov) * cos(3.1 * sample_time * sample + double(ii - 1) * deg2rad(120));
                        y_des = state.drones_state(lowest_detector, 3) + state.drones_state(lowest_detector, 5) / 2 * tan(system_parameters.drone_fov) * sin(3.1 * sample_time * sample + double(ii - 1) * deg2rad(120));
    
                        u = (u_left_vec * u_old_y(:, ii) + e_left_vec * [(y_des - y); e_old_y(:,ii)]);
                        e_old_y(:, ii) = (y_des - y);
                        u_old_y(:, ii) = u;
                        out(ii, 2) = u;
    
                        u = (u_left_vec * u_old_x(:, ii) + e_left_vec * [(x_des - x); e_old_x(:,ii)]);
                        e_old_x(:, ii) = (x_des - x);
                        u_old_x(:, ii) = u;
                        out(ii, 1) = u;
                        % we want to go here [mir(mic), mic]
                    end
                end





                % apply "triangulation" approach...

            end
            out = obj.constrain_control(out, system_parameters);
        end
    end
end