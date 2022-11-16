addpath("utils")

% Define the system parameters
system_parameters = SystemParameters(int32(1), int32(3), deg2rad(10), int32(2), 4, deg2rad(20));

% Define control algorithm
controller = Controller();

% Define initial condition of numerical simulation
initial_state = State(system_parameters);
% initial_state.randomize();

% Define the time span of the numerical simulation
tspan = [0 20];

% asdf
[t, y] = ode45(@(t, x)system_ode(t, x, system_parameters, controller), tspan, reshape(initial_state.get_state(), size(initial_state.get_state(), 1) * size(initial_state.get_state(), 2), 1));

