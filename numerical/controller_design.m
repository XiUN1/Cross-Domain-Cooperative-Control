% controller_design
A = [0 1; 0 0];
B = [0; 1];
C = [1 0];
D = 0;

sys = ss(A,B,C,D);
my_tf = tf(sys)

Atilde = [A zeros(2,1); C 0];
Btilde = [B; 0];
rank(ctrb(Atilde, Btilde)) % good

K = lqr(Atilde, Btilde, eye(3), 10)

T = 0.1;
s = tf('s');
test = my_tf * (1-T/4*s) / (1+T/4*s)

%%
P = 1/s^2;

T = 0.1;
sampling_frequency = 2 * pi / T; % 62.8 rad/s => we want bandwidth to be 6.28 rad/s
Pd = c2d(P, T, 'zoh');
set(Pd, 'Ts', T);
Pd_sbar = d2c(Pd, 'tustin');
zpk(Pd_sbar)
margin(Pd_sbar);

K_db = 32; % +3.3 for buffer
K = 10^(K_db / 20);

margin(K * Pd_sbar);
Phi = deg2rad(60 - (-17.9));
a = (1 + sin(Phi)) / (1 - sin(Phi));
omega_gc_sbar = 6.47; % rad/s
tau = 1 / (omega_gc_sbar * sqrt(a));
C_sbar = 1 / sqrt(a) * (1 + a * tau * s) / (1 + tau * s);

Cd = c2d(K * C_sbar, T, 'tustin');

margin(K * C_sbar * Pd_sbar)
margin(Cd * Pd)

CLS = feedback(Cd * Pd, 1);
step(CLS);
stepinfo(CLS);


omega = 3.21; % rad/s
k = [0:T:10]';
r = cos(omega * k);
[y,t] = lsim(CLS, r, k)



% stelling time = 2.6 seconds
% overshoot = 9.79 %