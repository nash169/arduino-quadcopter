function [dat] = GetData(model)

switch model
    case 'drone'      
        % Weight
        dat.m = 1;
        dat.g = 9.81;
        % Motor
        dat.k = 1;
        % Drag
        dat.r = 0;
        dat.d = 1;
        % Geometry
        dat.a = 1;
        dat.b = 1;
        dat.L = 1;
        % Inertia
        dat.I_x = 1;
        dat.I_y = 1;
        dat.I_z = 1;
        dat.I = [dat.I_x 0 0
                 0 dat.I_y  0
                 0 0 dat.I_z];
    case 'drone_test'
        
    otherwise
        error('Error');
end