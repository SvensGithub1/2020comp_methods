%{
numerical solution to an oscillating system
using forward Euler
%}


clear
close all

omega = 2;
P = 2*pi/omega;
dt = P/20;
T = 4*P;
N_t = floor(T/dt);
t = linspace(0, N_t*dt, N_t+1);

u = zeros(N_t+1, 1);
v = zeros(N_t+1, 1);

% Initial condition
X_0 = 2;
u(1) = X_0;
v(1) = 0;

% Step equations forward in time
for n = 1:N_t
    u(n+1) = u(n) + dt*v(n);
    v(n+1) = v(n) - dt*omega^2*u(n);
end


plot(t, u, 'b-', t, X_0*cos(omega*t), 'r--');
legend('numerical', 'exact', 'Location','northwest');
xlabel('t');
ylabel('y');
title('Comparing Numerical Solution to Analytical')

figure %plotting total energy
[pot_energy, kin_energy] = osc_energy(u,v,omega);
energy_all = pot_energy + kin_energy;
plot(t, energy_all, 'b-');
xlabel('t');
ylabel('Energy');
title('Total Energy in Oscillating System')