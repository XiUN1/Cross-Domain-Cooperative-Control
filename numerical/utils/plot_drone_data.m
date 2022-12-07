function f = plot_drone_data(samples, states, fig_num)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

all_states = [states{:}];
drones_states = {all_states.drones_state};

x_pos = [];
y_pos = [];
z_pos = [];

x_vel = [];
y_vel = [];
z_vel = [];

for ii = 1:length(drones_states)
    % for each time sample
    drones = drones_states{ii};

    for jj = 1:size(drones, 1)
        % for each DRONE
        drone_jj = drones(jj,:);
        if jj == 1
            % ugly hackery
            x_pos(jj,end+1) = drone_jj(1);
            y_pos(jj,end+1) = drone_jj(3);
            z_pos(jj,end+1) = drone_jj(5);
            x_vel(jj,end+1) = drone_jj(2);
            y_vel(jj,end+1) = drone_jj(4);
            z_vel(jj,end+1) = drone_jj(6);
        else
            x_pos(jj,end) = drone_jj(1);
            y_pos(jj,end) = drone_jj(3);
            z_pos(jj,end) = drone_jj(5);
            x_vel(jj,end) = drone_jj(2);
            y_vel(jj,end) = drone_jj(4);
            z_vel(jj,end) = drone_jj(6);
        end
        
    end
    
end

f = figure(fig_num);
subplot(2,3,1);
plot(samples, x_pos, 'LineWidth', 1.5); grid on; xlabel("Time [s]"); ylabel("Position along x-axis [m]")
subplot(2,3,2);
plot(samples, y_pos, 'LineWidth', 1.5); grid on; xlabel("Time [s]"); ylabel("Position along y-axis [m]")
subplot(2,3,3);
plot(samples, z_pos, 'LineWidth', 1.5); grid on; xlabel("Time [s]"); ylabel("Position along z-axis [m]")
subplot(2,3,4);
plot(samples, x_vel, 'LineWidth', 1.5); grid on; xlabel("Time [s]"); ylabel("Velocity along x-axis [m/s]")
subplot(2,3,5);
plot(samples, y_vel, 'LineWidth', 1.5); grid on; xlabel("Time [s]"); ylabel("Velocity along y-axis [m/s]")
subplot(2,3,6);
plot(samples, z_vel, 'LineWidth', 1.5); grid on; xlabel("Time [s]"); ylabel("Velocity along z-axis [m/s]")
end