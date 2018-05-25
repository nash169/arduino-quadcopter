function omegadot = OmegaDot(state, dat)
    tau = Torques(state, dat);
    omegadot = inv(dat.I) * (tau - cross(state.omega, dat.I * state.omega)); 
end

