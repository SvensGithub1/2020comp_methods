% Hands on solution for Simple Mechanism task

% u = [phi_2; d];
a = 0.1;
b = 0.2;
t=0:0.01:10;
phi_1 = deg2rad(30) + t;

u_ges =[];
% set a reasonable starting point
u0 = [0; b + a];

for ii=1:length(t)
% create function handles
F = @(u) constraint(u, a, b, phi_1(ii));
J = @(u) jacobian(u, b);

eps = 1e-9;

[u, iteration_counter] = NR_method(F, J, u0, eps);
u_ges = [u_ges,u];
fprintf('\n\tMechanism valid position is for d = %gm and phi1 = %gdeg\n\n', ...
    u(2), rad2deg(u(1)));
end
plot(t,u_ges(1,:))
hold on
plot(t,u_ges(2,:))
function P = constraint(u, a, b, phi_1)
phi_2 = u(1);
d = u(2);

P = [a * cos(phi_1) + b * cos(phi_2) - d
    a * sin(phi_1) - b * sin(phi_2)];
end

function P = jacobian(u, b)
phi_2 = u(1);
P = [-b * sin(phi_2), -1
    -b * cos(phi_2), 0];
end