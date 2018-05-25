function W = Omega2ThetaDot(q)
theta = q(4:6);
st1 = sin(theta(1));
ct1 = cos(theta(1));
tt2 = tan(theta(2));
ct2 = cos(theta(2));

% XYZ (airplane) convention
W = [1, st1*tt2, ct1*tt2; 
    0, ct1, -st1;
    0, st1/ct2, ct1/ct2];
end