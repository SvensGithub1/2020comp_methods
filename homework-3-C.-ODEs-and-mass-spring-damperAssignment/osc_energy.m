%{
this function approximates the potential and the kinetic energy of a system
%}

function [pot_energy, kin_energy] = osc_energy(u, v, omega)
pot_energy = 0.5 .* omega.^2 .* u.^2;
kin_energy = 0.5 .* v.^2;
end