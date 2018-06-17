function [A, B, C, D] = GetLinearSys(dat)
%GETLINEARSYS Summary of this function goes here
%   Detailed explanation goes here

A = [ zeros(3) zeros(3) eye(3) zeros(3)
      zeros(3) zeros(3) zeros(3) eye(3)
      zeros(3) [0 dat.g 0; -dat.g 0 0; 0 0 0] -dat.r/dat.m*eye(3) zeros(3)
      zeros(3) zeros(3) zeros(3) zeros(3)];
  
B = [zeros(3,4)
     zeros(3,4)
     zeros(2,4)
     1/dat.m*dat.k*[1 1 1 1]
     dat.I_x^-1*dat.b*dat.k*[1 -1 -1 1]
     dat.I_y^-1*dat.a*dat.k*[-1 -1 1 1]
     dat.I_z^-1*dat.d*[-1 1 -1 1]];

C = zeros(6,12);
C(1:3,:) = A(7:9,:);
C(4:6,10:12) = eye(3);
D = zeros(6,4);
D(4:6,:) = B(7:9,:);
end

