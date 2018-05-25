function [R] = RotMatrix(q)
theta = q(4:6);
cz = cos(theta(3)); 
cy = cos(theta(2));
cx = cos(theta(1));
sz = sin(theta(3));
sy = sin(theta(2));
sx = sin(theta(1));

R = [cz*cy,   cz*sy*sx - cx*sz,   sz*sx + cz*cx*sy; 
     cy*sz,   cz*cx + sz*sy*sx,   cx*sz*sy - cz*sx;
     -sy,    cy*sx,              cy*cx];  

end