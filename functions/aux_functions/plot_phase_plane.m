function [outputArg1,outputArg2] = plot_phase_plane(PAR, x_max, Nstep)
if ~exist('Nstep','VAR')
    Nstep = 10;
end
dx = 2*x_max/Nstep;
[X1,X2] = meshgrid(-x_max:dx:x_max,-x_max:dx:x_max);

num_x = size(X1,1);

F1 = zeros(num_x);
F2 = zeros(num_x);

for k1 = 1:num_x
    for k2 = 1:num_x
        x = [X1(k1,k2);X2(k1,k2)];
        [f,u] = PAR.sys_func(k1,x,PAR);
     
        F1(k1,k2) = f(1);
        F2(k1,k2) = f(2);
    end
end

quiver(X1,X2,F1,F2,'k'); hold all
xlim([-1,1]*x_max)
ylim([-1,1]*x_max)
end

