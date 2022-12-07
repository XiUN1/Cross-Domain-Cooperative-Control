classdef SystemParameters
    %SYSTEMPARAMETERS defines all the parameters of the system defined by
    %the position, velocity of all the drones and ROVs as well as their
    %sensors
    %   Assumptions:
    %   1. All drone receivers have the same FOV
    %   2. All ROV lasers have the same modeling parameters

    properties
        num_rovs  % integer
        num_drones  % integer

        % ROV noise level
        rov_noise_level  % double
        rov_max_speed  % double

        % laser receiver parameters
        drone_fov  % (radians) assume that FOV is same for all drones
 
        % laser model parameters
        beam_order  % beam order (integer). "n" in equations for intensity of beam
        laser_power  % laser's rated power (Watts). "P" in equations for intensity of beam
        laser_divergence  % laser's divergence (radians). "Theta" in equations for the intensity of beam
        laser_noise_std  % assume Gaussian distribution to noise

        % probability grid parameters
        grid_cols  % integer
        grid_rows  % integer
        grid_unit_length  % (double) how long is a side of each grid square (in meters)?
    end

    methods
        function obj = SystemParameters(num_rovs, num_drones, rov_noise_level, rov_max_speed, drone_fov, beam_order, laser_power, laser_divergence, laser_noise_std, grid_cols, grid_rows, grid_unit_length)
            assert(num_rovs >= 1); % there must be at least one ROV
            assert(isinteger(num_rovs)); % must be integer (cannot be float)
            obj.num_rovs = num_rovs;

            assert(num_drones >= 1); % there must be at least one drone
            assert(isinteger(num_drones)); % must be an integer (cannot be float)
            obj.num_drones = num_drones;

            assert(rov_noise_level > 0);
            assert(isa(rov_noise_level, 'double'));
            obj.rov_noise_level = rov_noise_level;

            assert(rov_max_speed > 0);
            assert(isa(rov_max_speed, 'double'));
            obj.rov_max_speed = rov_max_speed;

            assert(drone_fov > 0 && drone_fov < pi); % 0 < FOV < pi (in radians)
            assert(isa(drone_fov, 'double')); % make sure "drone_fov" is 'double'
            obj.drone_fov = drone_fov;

            assert(beam_order > 0 && mod(beam_order, 2) == 0); % make sure the beam order is a positive even integer
            assert(isinteger(beam_order)); % beam order must be an integer
            obj.beam_order = beam_order;

            assert(laser_power > 0); % make sure the beam power is a positive
            assert(isa(laser_power, 'double')); % make sure "laser_power" is 'double'
            obj.laser_power = laser_power;

            assert(laser_divergence > 0); % make sure the beak
            assert(isa(laser_divergence, 'double')); % make sure "laser_divergence" is 'double'
            obj.laser_divergence = laser_divergence;

            assert(laser_noise_std > 0);
            assert(isa(laser_noise_std, 'double'));
            obj.laser_noise_std = laser_noise_std;

            assert(grid_cols > 0);
            assert(isinteger(grid_cols));
            obj.grid_cols = grid_cols;

            assert(grid_rows > 0);
            assert(isinteger(grid_rows));
            obj.grid_rows = grid_rows;

            assert(grid_unit_length > 0);
            assert(isa(grid_unit_length, 'double'));
            obj.grid_unit_length = grid_unit_length;
        end
    end
end