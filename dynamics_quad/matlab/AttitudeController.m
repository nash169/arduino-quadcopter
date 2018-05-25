function u = AttitudeController(q, K)

K_theta = K(:,4:5);
K_omega = K(:,10:11);
    
u = -K_theta*q(4:5) -K_omega*q(10:11);
end