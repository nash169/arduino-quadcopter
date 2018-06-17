function u = AltitudeController(q, z_ref, K)
%ALTITUDECONTROLLER Summary of this function goes here
%   Detailed explanation goes here
e = q(3) - z_ref;

K_z = K(:,3);
K_zdot = K(:,9);

u = 0.8*9.81 - K_z*e- K_zdot*e;
% u =  - K_z*e- K_zdot*e;
% u = - K_z*e - K_zdot*e;
end

