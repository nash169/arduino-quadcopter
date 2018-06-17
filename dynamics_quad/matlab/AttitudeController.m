function u = AttitudeController(q, K)

K_theta = K(:,4:6);
K_omega = K(:,10:12);
    
u = -K_theta*q(4:6) -K_omega*q(10:12);
end