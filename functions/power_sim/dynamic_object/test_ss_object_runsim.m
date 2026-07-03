function [y] = test_ss_object_runsim(PAR,times,opt)
% Joakim Björk, joakim.bjork@svk.se, 2023-06-02
sys_func = @fg_xy_ss_object;

tole=1e-9; %Simulation tolerance
t_start=0;
t_end=times(end);

T = length(times);

nx = PAR.nx;
ny = length(PAR.y0);
assert(ny==0);
PAR.xy0 = [PAR.x0;PAR.y0];

G = PAR.G;

%% Simulate existing ss_object
% u = u0 + du = u0 + G*y

% G = in1;
% x0 = in2;
% y = in3;
% u0 = in4;

[n_out,n_in] = size(G.sys);





%%
% The following codes are used to make the DAE system in matlab
MM=[eye(nx)       zeros(nx,ny) ;
    zeros(ny,nx)  zeros(ny,ny)];

options=odeset('Mass',MM,'relTol',tole,'AbsTol',ones(1,nx+ny)*tole);

%% Initate simulation
t1 = t_start;
if length(times) == 1
    times = [1, times];    
end

%% Signals
if exist('opt','VAR') && isfield(opt,'dy')
    dy = opt.dy;
else
    dy = ones(n_in,T);
    dy = [zeros(n_in,1), dy(:,1)]; %
end

if exist('opt','VAR') && isfield(opt,'u0')
    PAR.u0 = opt.u0;
else
    PAR.u0 = zeros(n_in,1);
end

for i = 1:length(times)
    t2=times(i);
    TSPAN=[t1 t2];
    
    PAR.dy = dy(:,i);
       
    
    [TIME_,X_Y_]=ode15s(sys_func,TSPAN,PAR.xy0,options,PAR);
    PAR.xy0 = X_Y_(end,:)';
% If you get the error
% Warning: Failure at t=xxx. Unable to meet integration tolerances without
%          reducing the step size below the smallest value allowed
%          (1.421085e-14) at time t.
% alt1: Try lowering 'RelTol'
% alt2: Use ode23t
%     [TIME_,X_Y_]=ode23t(sys_func,TSPAN,PAR.xy0,options,PAR);
% ode15s should however be the first choice since it is faster.
    
    u_ = [];
    for k = 1:numel(TIME_)
        [~,u_(k,:)] = sys_func(TIME_(k),X_Y_(k,:)',PAR);
    end
    
    if i == 1
        TIME = TIME_;
        X_Y = X_Y_;
        u = u_;
    else
        TIME=[TIME;TIME_];
        X_Y=[X_Y;X_Y_];
        u = [u;u_];
    end
    
    t1 = t2;
end

%% Save variables
y = struct();
y.TIME = TIME;
y.X = X_Y;
y.u = u;

end
