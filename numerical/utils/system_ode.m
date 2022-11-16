function out = system_ode(t, state, system_parameters, controller)
%SYSTEM_ODE 

% ROV Motion (possibly random)
rovs_velocity = zeros(system_parameters.num_rovs, 3);

% Drone Motion (we control this)
u = controller.get_control(state, system_parameters); % matrix of size (NUM_DRONES x 3) where columns are xdot, ydot, zdot

out = [
    rovs_velocity;
    u
];

out = reshape(out, size(out,1) * size(out,2), 1);

end