classdef State
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        rovs_state  % matrix of size (NUM_ROVS x 3)... column order (x,y,z)

        drones_state  % matrix of size (NUM_DRONES x 3)... column order (x,y,z)
    end

    methods
        function obj = State(system_parameters)

            obj.rovs_state = [];
            obj.drones_state = [];

            % make sure the type of input argument is "SystemParameters"
            assert(isa(system_parameters, "SystemParameters"));

            for ii = 1:system_parameters.num_rovs
                obj.rovs_state = [obj.rovs_state; zeros(1,3)];
            end

            for ii = 1:system_parameters.num_drones
                obj.drones_state = [obj.drones_state; zeros(1,3)];
            end

        end

        function out = get_state(obj)
            out = [obj.rovs_state; obj.drones_state];
        end
    end
end