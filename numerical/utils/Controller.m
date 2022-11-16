classdef Controller
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        
    end

    methods
        function obj = Controller()
            
        end

        function out = get_control(obj, state, params)
            % for each drone?
            % out = matrix of size (NUM_DRONES x 3) where columns are
            % (xdot, ydot, zdot)
            out = zeros(params.num_drones, 3);

            for ii = 1:params.num_drones
                if norm(out(ii, :)) > 1
                    % if the norm of the desired control is > 1, normalize each row
                    out(ii,:) = out(ii,:) / norm(out(ii,:));
                end 
            end
        end
    end
end