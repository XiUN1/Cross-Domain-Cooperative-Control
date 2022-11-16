classdef SystemParameters
    %SYSTEMPARAMETERS defines all the parameters of the system defined by
    %the position, velocity of all the drones and ROVs as well as their
    %sensors
    %   Assumptions:
    %   1. All drone receivers have the same FOV
    %   2. All ROV lasers have the same modeling parameters

    properties
        num_rovs % integer
        num_drones % integer

        % laser receiver parameters
        drone_fov % (radians) assume that FOV is same for all drones
 
        % laser model parameters
        beam_order % beam order (integer). "n" in equations for intensity of beam
        laser_power % laser's rated power (Watts). "P" in equations for intensity of beam
        laser_divergence % laser's divergence (radians). "Theta" in equations for the intensity of beam
    end

    methods
        function obj = SystemParameters(num_rovs, num_drones, drone_fov, beam_order, laser_power, laser_divergence)
            
            assert(num_rovs >= 1); % there must be at least one ROV
            assert(isinteger(num_rovs)); % must be integer (cannot be float)
            obj.num_rovs = num_rovs;

            assert(num_drones >= 1); % there must be at least one drone
            assert(isinteger(num_drones)); % must be an integer (cannot be float)
            obj.num_drones = num_drones;

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


        end
    end
end