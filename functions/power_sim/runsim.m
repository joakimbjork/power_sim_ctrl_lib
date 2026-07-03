function [y, X_Y] = runsim(PAR,times,opt)
% Joakim Björk, joakim.bjork@svk.se, 2022-09-22

sys_func=PAR.sys_func;
tole=1e-7; %Simulation tolerance

N_times = length(times);

% PL1 = PAR.PL0;
% QL1 = PAR.QL0;

nx = PAR.nx;
ng = PAR.ng;
ny = PAR.ny;
nbus = PAR.nbus;

%% Inputs
% Constant power loads
event = false; % If there is no distrubance event, this avoids creating duplicate time stamps

if isfield(opt,'dPL')
    dPL = opt.dPL;
    event = true;
else
    dPL = zeros(nbus,N_times);
end
if isfield(opt,'dQL')
    dQL = opt.dQL;
    event = true;
else
    dQL = zeros(nbus,N_times);
end

% ZIP power loads
PL0 = PAR.PL0;
QL0 = PAR.QL0;
if isfield(opt,'dPL0')
    dPL0 = opt.dPL0;
    event = true;
else
    dPL0 = zeros(nbus,N_times);
end
if isfield(opt,'dQL0')
    dQL0 = opt.dQL0;
    event = true;
else
    dQL0 = zeros(nbus,N_times);
end

%%
% The following codes are used to make the DAE system in matlab
MM=[eye(nx)       zeros(nx,ny) ;
    zeros(ny,nx)  zeros(ny,ny)];

options=odeset('Mass',MM,'relTol',tole,'AbsTol',ones(1,nx+ny)*tole);

%% Initate simulation
if N_times == 1 
    if event == true % Standard event occurs at T = 1s
        N_times = N_times+1;
        times = [1, times];
        dPL = [zeros(nbus,1), dPL(:,1)]; %
        dQL = [zeros(nbus,1), dQL(:,1)]; %
        dPL0 = [zeros(nbus,1), dPL0(:,1)]; %
        dQL0 = [zeros(nbus,1), dQL0(:,1)]; %
    end
end

t1 = 0; % Start at time 0
for i = 1:N_times
    t2=times(i);
    TSPAN=[t1 t2];
    
    PAR.dPL = dPL(:,i);
    PAR.dQL = dQL(:,i);
    PAR.PL0 = PL0+dPL0(:,i);
    PAR.QL0 = QL0+dQL0(:,i);
    
    if isfield(opt,'YBUS') % Reconfigure network
        if any(opt.YBUS{i})
            PAR.YBUS = opt.YBUS{i};
        end
    end
    
    
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
y.DELTA = X_Y(:, 1 : ng );
y.OMEGA = X_Y(:, ng+1 : 2*ng);
if PAR.ns >= 3
    y.EQP=X_Y(:,2*ng+1:3*ng);
else
    y.EQP = ones(length(TIME),1).*PAR.EQP';
end
if PAR.ns >= 4
    y.EDP=X_Y(:,3*ng+1:4*ng);
    if PAR.ns == 6
        y.F1D  = X_Y(:,4*ng+1:5*ng);
        y.F2Q  = X_Y(:,5*ng+1:6*ng);
    end
else
    y.EDP = ones(length(TIME),1).*PAR.EDP';
end

% y.x_KA=X_Y(:,3*ng+1:3*ng+PAR.KA.nx);
y.ANG=X_Y(:,nx+(1:nbus));
y.VOLT=X_Y(:,nx+(nbus+1:2*nbus));

%% Modify units
y.DELTA = y.DELTA*180/pi; % rad -> deg
y.ANG = y.ANG*180/pi; % rad -> deg
y.OMEGA = y.OMEGA/(2*pi) + 50; % rad/s -> Hz (+ recenter at 50Hz)
y.OMEGA_COI = y.OMEGA*PAR.M/sum(PAR.M);

%% Save control signals
% Standard control signals
i1 = 1; i2 = ng;
y.u_PM = u(:,i1:i2);

i1 = i2+1; i2 = i2+ng;
y.u_EF = u(:,i1:i2);

i1 = i2+1; i2 = i2 + nbus;
y.u_PL = u(:,i1:i2);

i1 = i2+1; i2 = i2 + nbus;
y.u_QL = u(:,i1:i2);

for k = 1:numel(TIME)
    y.u_PQ_GEN(k,:) = h_PQ_GEN(X_Y(k,:)',PAR);
end

% Extra signals
if isfield(PAR,'EXC')
    y.u_EF_ref = u(:,PAR.EXC.u_idx);
end
if isfield(PAR,'PSS')
    y.u_pss = u(:,PAR.PSS.u_idx);
end
if isfield(PAR,'PMU')
    y.u_FREQ = u(:,PAR.PMU.u_idx) + 50; % Recenter at 50Hz
end
if isfield(PAR,'CONV')
    %     u = [u; EF_ref];
    y.u_CONV_PQ = u(:,PAR.CONV.PQ_idx);
    if isfield(PAR.CONV,'Storage')
        y.u_CONV_Echarge = u(:,PAR.CONV.Storage.u_idx);
        y.u_CONV_dP_charge = u(:,PAR.CONV.Storage.Charge_ctrl.u_idx);
    elseif isfield(PAR.CONV,'HVDC_wind')
        y.HVDC_wind_Vdc = u(:,PAR.CONV.HVDC_wind.Gdc_idx);
        y.HVDC_wind_P1 = u(:,PAR.CONV.HVDC_wind.P1_idx);
        y.HVDC_wind_P2 = u(:,PAR.CONV.HVDC_wind.P2_idx);
        y.HVDC_wind_Pwind = u(:,PAR.CONV.HVDC_wind.Pwind_idx);
        y.HVDC_wind_dP_K1 = u(:,PAR.CONV.HVDC_wind.dP_K1_idx);
        y.HVDC_wind_dP_K2 = u(:,PAR.CONV.HVDC_wind.dP_K2_idx);
        y.HVDC_wind_rotor_speed = u(:,PAR.CONV.HVDC_wind.rotor_speed_idx);
    end
    if PAR.CONV.type == 2 || PAR.CONV.type == 3
       y.u_DELTAC = u(:,PAR.CONV.DELTA_idx)*180/pi; % rad -> deg; 
       y.u_OMEGAC = u(:,PAR.CONV.OMEGA_idx)/(2*pi) + 50; % rad/s -> Hz (+ recenter at 50Hz)
    end
    y.Ed_CONV = u(:,PAR.CONV.Ed_idx);
    y.Eq_CONV = u(:,PAR.CONV.Eq_idx);
    y.Id_CONV = u(:,PAR.CONV.Id_idx);
    y.Iq_CONV = u(:,PAR.CONV.Iq_idx);
    y.Pref_CONV = u(:,PAR.CONV.Pref_idx);
    y.Qref_CONV = u(:,PAR.CONV.Qref_idx);
end
if isfield(PAR,'WIND')
    y.u_dPwind_ref = u(:,PAR.WIND.FFR.u_idx);
    y.u_dPwind = u(:,PAR.WIND.u_idx);
    y.d_wind_rot_speed = u(:,PAR.WIND.SPEED.u_idx);
end
if isfield(PAR,'PODQ')
    y.u_Q_PODQ = u(:,PAR.PODQ.u_idx);
    if PAR.PODQ.type == 3 
        y.u_Q_1 = u(:,PAR.PODQ.K1.u_idx);
        y.u_Q_2 = u(:,PAR.PODQ.K2.u_idx);
        y.u_Q_aux = u(:,PAR.PODQ.Kaux.u_idx);
    elseif PAR.PODQ.type == 4
        y.u_Q_1 = u(:,PAR.PODQ.K1.u_idx);
        y.u_Q_2 = u(:,PAR.PODQ.K2.u_idx);
        y.u_Q_com = u(:,PAR.PODQ.COM.u_idx);
        y.u_Q_aux = u(:,PAR.PODQ.Kaux.u_idx);
    end
end
if isfield(PAR,'PODP')
    y.u_P_PODP = u(:,PAR.PODP.u_idx);
    y.u_FREQ_PODP = u(:,PAR.PODP.PLL.u_idx)+50;
    if isfield(PAR.PODP,'input_func_filter')
        y.u_FREQ_PODP_test = u(:,PAR.PODP.input_func_filter.u_idx);
    end
end
if isfield(PAR,'GOV')
    if isfield(PAR.GOV,'idx_ramp_error')
        y.GOV_ramp_error = X_Y(:,PAR.GOV.idx_ramp_error);
        y.u_PM_ramp_error = u(:,PAR.GOV.u_idx);
    end
end

if isfield(opt,'show_active_power_flow') && opt.show_active_power_flow == true
    for k = 1:numel(TIME)
        y.Pline(k,:) = h_P(X_Y(k,:)',PAR);
    end
    % Example on how to pick out specific line flow
    % from = 4;
    % to = 3;
    % y.Pline(:,from + (to-1)*PAR.nbus)); 
end

if isfield(opt,'show_reactive_power_flow') && opt.show_reactive_power_flow == true
    for k = 1:numel(TIME)
        y.Qline(k,:) = h_Q(X_Y(k,:)',PAR);
    end
    % Example on how to pick out specific line flow
    % from = 4;
    % to = 3;
    % y.Qline(:,from + (to-1)*PAR.nbus)); 
end
