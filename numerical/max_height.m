% Define System parameters
n = 2; % increasing beam order increases height of black line, more flat
P = 4; % increasing Wattage increases height of black line
Theta = deg2rad(20); % increasing Divergence increases height of black line
laser_noise_std = 0.0025; % 0.0025
drone_fov = deg2rad(10); % increases slope of red line

% at a certain confidence assuming Gaussian nosie on intensity signal
detection_threshold = 2.58 * laser_noise_std; % increasing this lowers the black line

syms z

% z*sin(drone_fov)
z_vec = [];
r_vec = 0:0.5:5;

for ii = 1:length(r_vec)
    r = r_vec(ii);

    sigma = (1/10)^(-1/n) * z * sin(Theta / 2);
    I0_z = n * P / (2 * pi * 4^(1/n) * sigma^2 * gamma(2/n));
    I = I0_z * exp(-1/2 * (r / sigma)^n);

    soln = max(double(solve(I == detection_threshold, z)));

    z_vec(end+1) = soln;
end


plot(r_vec, z_vec, 'k', 'LineWidth', 1.5)
xlabel('r [m]')
ylabel('z [m]')
hold on;
plot(r_vec, r_vec / tan(drone_fov), 'r', 'LineWidth', 1.5)
grid on; 
hold off


% snr = [];
% 
% for r = 0:0.5:3.5
%     for z = 0:1:18
%         sigma = (1/10)^(-1/n) * z * sin(Theta / 2);
%         I0_z = n * P / (2 * pi * 4^(1/n) * sigma^2 * gamma(2/n));
%         I = I0_z * exp(-1/2 * (r / sigma)^n);
%         snr(end+1) = I / laser_noise_std;
%     end
% end
% 
% scatter(snr);