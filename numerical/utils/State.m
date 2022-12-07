classdef State
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        rovs_state  % matrix of size (NUM_ROVS x 3)... column order (x,y,z)

        drones_state  % matrix of size (NUM_DRONES x 3)... column order (x,y,z)

        laser_intensity  % instantaneous laser intensity measured by drone matrix of size (NUM_DRONES)

        rov_probability_matrix  % matrix of size (NUM_ROVS x GRID_ROWS x GRID_COLS)
    end

    methods
        function obj = State(system_parameters)

            obj.rovs_state = [];
            obj.drones_state = [];
            obj.laser_intensity = zeros(system_parameters.num_drones, 1);
            obj.rov_probability_matrix = [];

            % make sure the type of input argument is "SystemParameters"
            assert(isa(system_parameters, "SystemParameters"));

            for ii = 1:system_parameters.num_rovs
                obj.rovs_state = [obj.rovs_state; zeros(1,6)];
            end

            for ii = 1:system_parameters.num_drones
                obj.drones_state = [obj.drones_state; zeros(1,6)];
            end

            for ii = 1:system_parameters.num_rovs
                nc = double(system_parameters.grid_cols);
                nr = double(system_parameters.grid_rows);
                m = 1 / (nc * nr) * ones(nc, nr);
                obj.rov_probability_matrix = [obj.rov_probability_matrix; m];
            end

        end

        function [rov, drone, prob_map] = get_state(obj)
            rov = obj.rovs_state;
            drone = obj.drones_state;
            prob_map = obj.rov_probability_matrix;
        end

        function obj = randomize(obj, system_parameters)
            % put drone somewhere [1,10] meters in z, and somewhere in
            % arena. put drone somewhere [-5,-10] meters in z and somewhere
            % in arena.

            drone_upper_z_bound = 5;
            drone_lower_z_bound = 1;
            drone_z_bound_range = (drone_upper_z_bound - drone_lower_z_bound);

            rov_upper_z_bound = -5;
            rov_lower_z_bound = -10;
            rov_z_bound_range = (rov_upper_z_bound - rov_lower_z_bound);

            arena_x = double(system_parameters.grid_rows * system_parameters.grid_unit_length / 2);
            arena_y = double(system_parameters.grid_cols * system_parameters.grid_unit_length / 2);

            for ii = 1:system_parameters.num_rovs
                % x position is random
                obj.rovs_state(ii,1) = (2 * arena_x * rand - arena_x) / 2;
                obj.rovs_state(ii,3) = (2 * arena_y * rand - arena_y) / 2;
                obj.rovs_state(ii,5) = (rov_z_bound_range) * rand + rov_lower_z_bound;
            end

            for ii = 1:system_parameters.num_drones
                obj.drones_state(ii,1) = 2 * arena_x * rand - arena_x;
                obj.drones_state(ii,3) = 2 * arena_y * rand - arena_y;
                obj.drones_state(ii,5) = (drone_z_bound_range) * rand + drone_lower_z_bound;
            end
        end
    end
end