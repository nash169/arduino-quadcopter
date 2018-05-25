function PlotDrone(Q_log, t_step, dat)

x = Q_log(t_step,1:3)';
x_log = Q_log(1:t_step,1:3)';
R = RotMatrix(Q_log(t_step,:)');

p1 = R*[dat.L; 0; 0] + x;
p2 = R*[0; dat.L; 0] + x;
p3 = R*[-dat.L; 0; 0] + x;
p4 = R*[0; -dat.L; 0] + x;
p5 = R*[dat.L; 0; dat.L/2] + x;
p6 = R*[0; dat.L; dat.L/2] + x;
p7 = R*[-dat.L; 0; dat.L/2] + x;
p8 = R*[0; -dat.L; dat.L/2] + x;

h = plot3 ([p1(1) p3(1)], [p1(2) p3(2)], [p1(3) p3(3)], 'b', ...
       [p2(1) p4(1)], [p2(2) p4(2)], [p2(3) p4(3)], 'g', ...
       [p1(1) p5(1)], [p1(2) p5(2)], [p1(3) p5(3)], 'r', ...
       [p2(1) p6(1)], [p2(2) p6(2)], [p2(3) p6(3)], 'r',...
       [p3(1) p7(1)], [p3(2) p7(2)], [p3(3) p7(3)], 'k',...
       [p4(1) p8(1)], [p4(2) p8(2)], [p4(3) p8(3)], 'k',...
       x_log(1,:), x_log(2,:), x_log(3,:), 'k');
axis([-10 10 -10 10 0 15]);
drawnow; 
end