
%% constraons and jacobian matrix construction
function P = constraint(u, a, b, phi)
theta = u(1);
d = u(2);

P = [a * cos(phi) + b * cos(theta) - d
    a * sin(phi) - b * sin(theta)];
end

function P = jacobian(u, b)
theta = u(1);
P = [-b * sin(theta), -1
    -b * cos(theta), 0];
end