function G = lin_fg_xy(xy0,PAR)
% Joakim Björk, joakim.bjork@svk.se, 2022-09-22

input = 'constant_P_and_Q';
output = 'ANG_and_VOLT';
if isfield(PAR,'input')
    if ~isempty(PAR.input)
        input = PAR.input;
    end
end
if isfield(PAR,'output')
    if ~isempty(PAR.output)
        output = PAR.output;
    end
end

sys_func = PAR.lin_sys_func;
% out_func = PAR.lin_out_func;
fg0=feval(sys_func,0,xy0,PAR);

tole_=1E-9;
nx = PAR.nx; % Number of state variables
ny = PAR.ny; % Number of algebraic variables
nbus = PAR.nbus; % Number of buses
x0=xy0(1:nx);
f_0=fg0(1:nx,1); %derivative of state variables
y0=xy0(nx+1:nx+ny);
g_0=fg0(nx+1:nx+ny,1); %should always be zero

%% A
% clearvars fxx fxy fyx fyy
fxx = zeros(nx,nx);
fxy = zeros(nx,ny);
fyx = zeros(ny,nx);
fyy = zeros(ny,ny);
for i=1:nx,
    % Goes through all the state variables,
    dx = [ zeros(i-1,1); tole_; zeros(nx-i,1)];
    x_=x0+dx; %New state variables with deviation in one state variable
    xy_=[x_;y0];
    fg=feval(sys_func,0,xy_,PAR);
    % Calculate the new state and algebraic variables using the
    % full nonlinear system model
    f_=fg(1:nx,1); % State variables
    g_=fg(nx+1:nx+ny,1); % Algebraic variables
    fxx(:,i) = ( f_ - f_0)/tole_; % Difference: new-initial
    % (normalized with step size tole_)
    fyx(:,i) = ( g_ - g_0)/tole_;
    %Same but for algebraic expression due to change in state variables
end
% This gives a l_x0-by-l_x0 matrix where with each column represent
% state changes due do deviation in that state variable

for i=1:ny,
    % Goes through all the algebraic variables,
    dy = [ zeros(i-1,1); tole_; zeros(ny-i,1)];
    y_=y0+dy;
    xy_=[x0;y_];
    fg=feval(sys_func,0,xy_,PAR);
    f_=fg(1:nx,1);
    g_=fg(nx+1:nx+ny,1);
    fxy(:,i) = ( f_ - f_0)/tole_;
    fyy(:,i) = ( g_ - g_0)/tole_;
end


%
%% B: Input
if strcmp(input,'constant_P_and_Q')
    disp('Input: Power injection (constant power load), -PL [pu], -QL [pu]')
    nu = nbus;
    PAR_U = PAR;
    fxu = []; fyu =[];
    u0=zeros(nu,1);
    for i=1:nu,
        du = [ zeros(i-1,1); tole_; zeros(nu-i,1)];
        u_=u0 - du; % Negative since we are injecting power (Modifying load)
        PAR_U.dPL=u_; %<---- EDIT HERE
        fg=feval(sys_func,0,xy0,PAR_U);
        f_=fg(1:nx,1);
        g_=fg(nx+1:nx+ny,1);
        fxu(:,i) = ( f_ - f_0)/tole_;
        fyu(:,i) = ( g_ - g_0)/tole_;
    end

    PAR_U = PAR;
    u0=zeros(nu,1);
    for i=1:nu,
        du = [ zeros(i-1,1); tole_; zeros(nu-i,1)];
        u_=u0 - du; % Negative since we are injecting power (Modifying load)
        PAR_U.dQL=u_; %<---- EDIT HERE
        fg=feval(sys_func,0,xy0,PAR_U);
        f_=fg(1:nx,1);
        g_=fg(nx+1:nx+ny,1);
        fxu(:,nu+i) = ( f_ - f_0)/tole_;
        fyu(:,nu+i) = ( g_ - g_0)/tole_;
    end

elseif strcmp(input,'P_and_Q')
    disp('Input: Power injection, -PL [pu], -QL [pu]')
    nu = nbus;
    PAR_U = PAR;
    fxu = []; fyu =[];
    u0=PAR_U.PL0;
    for i=1:nu,
        du = [ zeros(i-1,1); tole_; zeros(nu-i,1)];
        u_=u0 - du; % Negative since we are injecting power (Modifying load)
        PAR_U.PL0=u_; %<---- EDIT HERE
        fg=feval(sys_func,0,xy0,PAR_U);
        f_=fg(1:nx,1);
        g_=fg(nx+1:nx+ny,1);
        fxu(:,i) = ( f_ - f_0)/tole_;
        fyu(:,i) = ( g_ - g_0)/tole_;
    end

    PAR_U = PAR;
    u0=PAR_U.QL0;
    for i=1:nu,
        du = [ zeros(i-1,1); tole_; zeros(nu-i,1)];
        u_=u0 - du; % Negative since we are injecting power (Modifying load)
        PAR_U.QL0=u_; %<---- EDIT HERE
        fg=feval(sys_func,0,xy0,PAR_U);
        f_=fg(1:nx,1);
        g_=fg(nx+1:nx+ny,1);
        fxu(:,nu+i) = ( f_ - f_0)/tole_;
        fyu(:,nu+i) = ( g_ - g_0)/tole_;
    end


elseif strcmp(input,'UREF') || strcmp(input,'EF')
    if strcmp(input,'UREF') && isfield(PAR,'EXC')
        PAR.input = 'UREF';
        disp('Input: Excitation set point, UREF [pu]')
        nu = PAR.ng;
        PAR_U = PAR;
        fxu = []; fyu =[];
        %     du=u0(1:nu); %Chooses all buses
        u0=zeros(nu,1);
        for i=1:nu,
            du = [ zeros(i-1,1); tole_; zeros(nu-i,1)];
            u_=u0 + du; %
            PAR_U.u_UREF=u_; %<---- EDIT HERE
            fg=feval(sys_func,0,xy0,PAR_U);
            f_=fg(1:nx,1);
            g_=fg(nx+1:nx+ny,1);
            fxu(:,i) = ( f_ - f_0)/tole_;
            fyu(:,i) = ( g_ - g_0)/tole_;
        end
    else
        PAR.input = 'EF';
        disp('Input: Excitation, EF [pu]')
        nu = PAR.ng;
        PAR_U = PAR;
        fxu = []; fyu =[];
        %     du=u0(1:nu); %Chooses all buses
        u0=PAR_U.UREF;
        for i=1:nu,
            du = [ zeros(i-1,1); tole_; zeros(nu-i,1)];
            u_=u0 + du; %
            PAR_U.u_UREF=u_; %<---- EDIT HERE
            fg=feval(sys_func,0,xy0,PAR_U);
            f_=fg(1:nx,1);
            g_=fg(nx+1:nx+ny,1);
            fxu(:,i) = ( f_ - f_0)/tole_;
            fyu(:,i) = ( g_ - g_0)/tole_;
        end
    end

elseif strcmp(input,'PM')
    disp('Input: Turbine, Pm [pu]')
    nu = PAR.ng;
    %     nu_2 = nu;
    PAR_U = PAR;
    fxu = []; fyu =[];
    %     du=u0(1:nu); %Chooses all buses
    u0=PAR_U.PM;
    for i=1:nu,
        du = [ zeros(i-1,1); tole_; zeros(nu-i,1)];
        u_=u0 + du; %
        PAR_U.PM=u_; %<---- EDIT HERE
        fg=feval(sys_func,0,xy0,PAR_U);
        f_=fg(1:nx,1);
        g_=fg(nx+1:nx+ny,1);
        fxu(:,i) = ( f_ - f_0)/tole_;
        fyu(:,i) = ( g_ - g_0)/tole_;
    end

elseif strcmp(input,'GOV_OMEGA_ref') || strcmp(input,'GOV_ref')
    disp('Input: GOV_ref, OMEGA [pu]')
    assert(isfield(PAR, 'GOV'))
    nu = PAR.ng;
    PAR_U = PAR;
    fxu = []; fyu =[];
    %     du=u0(1:nu); %Chooses all buses
    PAR_U.GOV.lin_fg_xy_input=zeros(nu,1);
    for i=1:nu,
        tole2_ = tole_*1e3;
        du = [ zeros(i-1,1); tole2_; zeros(nu-i,1)];
        PAR_U.GOV.lin_fg_xy_input=du; %<---- EDIT HERE
        fg=feval(sys_func,0,xy0,PAR_U);
        f_=fg(1:nx,1);
        g_=fg(nx+1:nx+ny,1);
        fxu(:,i) = ( f_ - f_0)/tole2_;
        fyu(:,i) = ( g_ - g_0)/tole2_;
    end

elseif strcmp(input,'PODP_OMEGA_ref') || strcmp(input,'PODP_ref')
    disp('Input: PODP_ref, OMEGA [pu]')
    assert(isfield(PAR, 'PODP'))
    nu = PAR.nbus;
    PAR_U = PAR;
    fxu = []; fyu =[];
    %     du=u0(1:nu); %Chooses all buses
    PAR_U.PODP.lin_fg_xy_input=zeros(nu,1);
    for i=1:nu,
        tole2_ = tole_*1e3;
        du = [ zeros(i-1,1); tole2_; zeros(nu-i,1)];
        PAR_U.PODP.lin_fg_xy_input=du; %<---- EDIT HERE
        fg=feval(sys_func,0,xy0,PAR_U);
        f_=fg(1:nx,1);
        g_=fg(nx+1:nx+ny,1);
        fxu(:,i) = ( f_ - f_0)/tole2_;
        fyu(:,i) = ( g_ - g_0)/tole2_;
    end

elseif strcmp(input,'PODQ_VOLT_ref') || strcmp(input,'PODQ_ref')
    disp('Input: PODQ_ref, VOLT [pu]')
    assert(isfield(PAR, 'PODQ'))
    nu = PAR.nbus;
    PAR_U = PAR;
    fxu = []; fyu =[];
    %     du=u0(1:nu); %Chooses all buses
    PAR_U.PODQ.lin_fg_xy_input=zeros(nu,1);
    for i=1:nu,
        tole2_ = tole_*1e3;
        du = [ zeros(i-1,1); tole2_; zeros(nu-i,1)];
        PAR_U.PODQ.lin_fg_xy_input=du; %<---- EDIT HERE
        fg=feval(sys_func,0,xy0,PAR_U);
        f_=fg(1:nx,1);
        g_=fg(nx+1:nx+ny,1);
        fxu(:,i) = ( f_ - f_0)/tole2_;
        fyu(:,i) = ( g_ - g_0)/tole2_;
    end

elseif strcmp(input,'bypass_PODP')
    disp('Input: PODP power setpoint, P [pu]')
    assert(isfield(PAR, 'PODP'))
    nu = PAR.nbus;
    PAR_U = PAR;
    fxu = []; fyu =[];
    %     du=u0(1:nu); %Chooses all buses
    PAR_U.PODP.bypass_lin_fg_xy_input=zeros(nu,1);
    for i=1:nu,
        tole2_ = tole_*1e3;
        du = [ zeros(i-1,1); tole2_; zeros(nu-i,1)];
        PAR_U.PODP.bypass_lin_fg_xy_input=du; %<---- EDIT HERE
        fg=feval(sys_func,0,xy0,PAR_U);
        f_=fg(1:nx,1);
        g_=fg(nx+1:nx+ny,1);
        fxu(:,i) = ( f_ - f_0)/tole2_;
        fyu(:,i) = ( g_ - g_0)/tole2_;
    end

elseif strcmp(input,'PC') || strcmp(input,'Pref')
    disp('Input: Converter, Pref [pu]')
    assert(isfield(PAR, 'CONV'))
    % Cbus = PAR.CONV.Cbus;
    % nu = length(Cbus);
    nu = PAR.nbus;
    %     nu_2 = nu;
    PAR_U = PAR;
    fxu = []; fyu =[];
    %     du=u0(1:nu); %Chooses all buses
    % u0=PAR_U.P0(Cbus);
    u0=PAR_U.P0;
    for i=1:nu,
        du = [ zeros(i-1,1); tole_; zeros(nu-i,1)];
        u_=u0 + du; %
        % PAR_U.P0(Cbus)=u_; %<---- EDIT HERE
        PAR_U.P0=u_; %<---- EDIT HERE
        fg=feval(sys_func,0,xy0,PAR_U);
        f_=fg(1:nx,1);
        g_=fg(nx+1:nx+ny,1);
        fxu(:,i) = ( f_ - f_0)/tole_;
        fyu(:,i) = ( g_ - g_0)/tole_;
    end

elseif strcmp(input,'QC') || strcmp(input,'Qref')
    disp('Input: Converter, Qref [pu]')
    assert(isfield(PAR, 'CONV'))
    % Cbus = PAR.CONV.Cbus;
    nu = nbus;
    %     nu_2 = nu;
    PAR_U = PAR;
    fxu = []; fyu =[];
    %     du=u0(1:nu); %Chooses all buses
    u0=PAR_U.Q0;
    for i=1:nu,
        du = [ zeros(i-1,1); tole_; zeros(nu-i,1)];
        u_=u0 + du; %
        PAR_U.Q0=u_; %<---- EDIT HERE
        fg=feval(sys_func,0,xy0,PAR_U);
        f_=fg(1:nx,1);
        g_=fg(nx+1:nx+ny,1);
        fxu(:,i) = ( f_ - f_0)/tole_;
        fyu(:,i) = ( g_ - g_0)/tole_;
    end

elseif strcmp(input,'PQref_CONV')
    disp('Input: Converter, Qref [pu]')
    warning('Obsolete function')
    assert(isfield(PAR, 'CONV'))
    Cbus = PAR.CONV.Cbus;
    nu_half = length(Cbus);
    nu = 2*nu_half;
    %     nu_2 = nu;
    PAR_U = PAR;
    fxu = []; fyu =[];
    %     du=u0(1:nu); %Chooses all buses
    u0=[PAR_U.PC; PAR_U.QC];
    for i=1:nu,
        du = [ zeros(i-1,1); tole_; zeros(nu-i,1)];
        u_=u0 + du; %
        PAR_U.PC=u_(1 : nu_half); %<---- EDIT HERE
        PAR_U.QC=u_( (nu_half+1) : end); %<---- EDIT HERE
        fg=feval(sys_func,0,xy0,PAR_U);
        f_=fg(1:nx,1);
        g_=fg(nx+1:nx+ny,1);
        fxu(:,i) = ( f_ - f_0)/tole_;
        fyu(:,i) = ( g_ - g_0)/tole_;
    end

elseif strcmp(input,'Pref_CONV')
    disp('Input: Converter active power setpoint, Pref [pu]')
    assert(isfield(PAR, 'CONV'))
    Cbus = PAR.CONV.Cbus;
    nu = length(Cbus);
    PAR_U = PAR;
    fxu = []; fyu =[];
    %     du=u0(1:nu); %Chooses all buses
    PAR_U.CONV.Pref_lin_fg_xy_input=zeros(nu,1);
    for i=1:nu,
        tole2_ = tole_*1e3;
        du = [ zeros(i-1,1); tole2_; zeros(nu-i,1)];
        PAR_U.CONV.Pref_lin_fg_xy_input=du; %<---- EDIT HERE
        fg=feval(sys_func,0,xy0,PAR_U);
        f_=fg(1:nx,1);
        g_=fg(nx+1:nx+ny,1);
        fxu(:,i) = ( f_ - f_0)/tole2_;
        fyu(:,i) = ( g_ - g_0)/tole2_;
    end

elseif strcmp(input,'VOLT')
    disp('Input: Change bus voltage, VOLT [pu]')
    nu = PAR.nbus;
    %     nu_2 = nu;
    PAR_U = PAR;
    fxu = []; fyu =[];
    %     du=u0(1:nu); %Chooses all buses
    PAR_U.VOLT_pert = struct();
    for i=1:nu
        PAR_U.VOLT_pert.bus = i;
        PAR_U.VOLT_pert.u = tole_; %
        fg=feval(sys_func,0,xy0,PAR_U);
        f_=fg(1:nx,1);
        g_=fg(nx+1:nx+ny,1);
        fxu(:,i) = ( f_ - f_0)/tole_;
        fyu(:,i) = ( g_ - g_0)/tole_;
    end


end

%% C: Output ANG and VOLT
ng = PAR.ng; % Number of machines


if strcmp(output,'ANG_and_VOLT')
    disp('Output: Algebraic variables, ANG [rad], VOLT [pu]')
    hx = zeros(nbus*2,nx);
    hy = [eye(nbus*2), zeros(nbus*2, ny - nbus*2)];
elseif strcmp(output,'VOLT')
    disp('Output: Algebraic variables, VOLT [pu]')
    hx = zeros(nbus,nx);
    hy = zeros(nbus,ny);
    hy(:,nbus+1:nbus*2) = eye(nbus);
elseif strcmp(output,'DELTA')
    disp('Output: Rotor angle, DELTA [rad]')
    hx = zeros(ng,nx);
    hx(1:ng,1:ng) = eye(ng);
    hy = zeros(ng,ny);
elseif strcmp(output,'OMEGA')
    disp('Output: Rotor speed, OMEGA/2pi [Hz]')
    hx = zeros(ng,nx);
    hx(1:ng,(1:ng)+ng) = eye(ng)/(2*pi); % rad/s -> Hz
    hy = zeros(ng,ny);
elseif strcmp(output,'EQP')
    assert(PAR.ns>=3);
    disp('Output: Q-axis transient voltage, EQP [pu]')
    hx = zeros(ng,nx);
    hx(1:ng,(1:ng)+2*ng) = eye(ng);
    hy = zeros(ng,ny);
elseif strcmp(output,'EDP')
    assert(PAR.ns>=4);
    disp('Output: D-axis transient voltage, EDP [pu]')
    hx = zeros(ng,nx);
    hx(1:ng,(1:ng)+3*ng) = eye(ng);
    hy = zeros(ng,ny);
else % Use function h_xxxx.m for outputs that are not state variables
    if strcmp(output,'PQ_GEN')
        disp('Output: Power injection from generators [PE; QE] [pu]')
        out_func = 'h_PQ_GEN';
    elseif strcmp(output,'PQ_CONV')
        disp('Output: Power injection from converters [PC; QC] [pu]')
        out_func = 'h_PQ_CONV';
    end
    h_0=feval(out_func,xy0,PAR);
    % sys_func(TIME_(k),X_Y_(k,:)',PAR)
    hx = []; hy = [];
    for i=1:nx,
        dx = [ zeros(i-1,1); tole_; zeros(nx-i,1)];
        x_=x0+dx;
        xy_=[x_;y0];
        h_=feval(out_func,xy_,PAR);
        hx(:,i) = ( h_ - h_0)/tole_;
    end

    for i=1:nbus*2,
        dy = [ zeros(i-1,1); tole_; zeros(ny-i,1)];
        y_=y0+dy;
        xy_=[x0;y_];
        h_=feval(out_func,xy_,PAR);
        hy(:,i) = ( h_ - h_0)/tole_;
    end
    [d1, d2] = size(hy);
    hy = [hy, zeros(d1, ny-d2)];
end


A=fxx-fxy*inv(fyy)*fyx;
B=fxu-fxy*inv(fyy)*fyu;
C=hx-hy*inv(fyy)*fyx;
D = - hy*inv(fyy)*fyu;

G = ss(A,B,C,D);

if PAR.infbus ~=0
    idx = PAR.infbus:ng:PAR.ns*ng;
    G = modred(G,idx,'truncate');
    %     if nx > 3*ng
    %         disp('Verify that truncation of infinite bus works when modeling more than the generator dynamics')
    %     end
end
end