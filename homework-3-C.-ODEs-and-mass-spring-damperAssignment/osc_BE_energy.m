%{
numerical solution to an oscillating system
using backward Euler
%}

close all
clear 
omega = 2;
P = 2*pi/omega;
dt = P/20;
T = 4*P;
N_t = floor(round(T/dt));
t = linspace(0, N_t*dt, N_t+1);
fprintf('N_t: %d\n', N_t);

u = zeros(N_t+1, 1);
v = zeros(N_t+1, 1);

% Initial condition
X_0 = 2;
u(1) = X_0;
v(1) = 0;

% Step equations forward in time
for n = 1:N_t
    v(n+1) = v(n) - dt*omega^2 * (u(n)+dt*v(n))/(1+dt^2*omega^2);
    u(n+1) = u(n) + dt*v(n+1);
end
% Plot the last four periods to illustrate the accuracy
% in long time simulations
N4l = floor(round(4*P/dt)); % No of intervals to be plotted
true_sol = X_0*cos(omega*t);
plot(t(length(t)-N4l:end), u(length(u)-N4l:end), 'b-',...
       t(length(t)-N4l:end), true_sol(length(true_sol)-N4l:end), 'r--');
   hold on
legend('numerical', 'exact', 'Location','northwest');
xlabel('t');
fprintf('%.16f %.16f \n', u(end), v(end));
ylabel('y');
title('Comparing Numerical Solution to Analytical')

figure %plotting total energy
[pot_energy, kin_energy] = osc_energy(u,v,omega);
energy_all = pot_energy + kin_energy;
plot(t, energy_all, 'b-');
xlabel('t');
ylabel('Energy');
title('Total Energy in Oscillating System')
