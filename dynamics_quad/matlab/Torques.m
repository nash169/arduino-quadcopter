function tau = Torques(u, dat)
    tau = [dat.b * dat.k * (u(1) - u(2) - u(3) + u(4))
           dat.a * dat.k * (-u(1) - u(2) + u(3) + u(4))
           dat.d * (-u(1) + u(2) - u(3) + u(4))
    ];
end
