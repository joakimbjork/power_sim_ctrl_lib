function [PAR] = data_two_machine_four_bus(opt)

tole=1e-6;
maxiter=10;
fs=50; % Nominal frequency [Hz]
ws=2*pi*fs; % Nominal frequency [rad/s]
Sb = 1000; % MW system base

PAR = struct();
PAR.sys_func = @fg_xy_SM; % Function handle for simulation
PAR.lin_sys_func='fg_xy_SM'; % Function handle for linearization
PAR.lin_out_func='h_y'; % output algebraic variables: [ANG, VOLT]

disp('This model is outdate. Consider using "data_PODQ_test_system" instead')

PAR.ws = ws;
if isfield(opt,'mp')
    PAR.mp = opt.mp;
else
    PAR.mp = 0;  % Active load characteristic
end
if isfield(opt,'mq')
    PAR.mq = opt.mq;
else
    PAR.mq = 0;  % Reactive load characteristic
end

PAR.infbus = [];
PAR.Sb = Sb;

%% Generator data
% Classic machine model (ns = 2) or one-axis mpdel (ns = 3)?
if isfield(opt,'ns')
    PAR.ns = opt.ns;
else
    PAR.ns = 3;
end
if PAR.ns == 2
    disp('Loading classical machine model');
elseif PAR.ns == 3
    disp('Loading one-axis machine model');
else
    PAR.ns = 2;
    disp('Loading classical machine model');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Sr = 10000; % Rated power [MW]
if isfield(opt,'machine_damping')
    D = opt.machine_damping/(Sb*2*pi);
else
    D = zeros(2,1);
end

if isfield(opt,'inertia')
    H = opt.inertia;
else
    H = 6*ones(2,1);
end


%            1    2     3    4   5     6       7
%          Gen   xt   xdp   xd   H     D     Tdop
GENDATA = [Sr  0.1   0.1    0.7  H(1)   D(1)   5e6
           Sr  0.1   0.1    0.7  H(2)   D(2)   5e6 ];%

PAR.MBASE = GENDATA(:,1)/Sb;
PAR.XDP=(GENDATA(:,2)+GENDATA(:,3))./PAR.MBASE;
PAR.XD=(GENDATA(:,2)+GENDATA(:,4))./PAR.MBASE;
PAR.M=2*(GENDATA(:,5)).*PAR.MBASE/ws;
PAR.D=GENDATA(:,6);
PAR.TDOP=GENDATA(:,7);

%% Line Data


if isfield(opt,'scenario')
    PAR.scenario = opt.scenario;
else
    PAR.scenario = 1;
end

if isfield(opt,'a')
    a = opt.a;
else
    a = 0.5;
end

NS = 0; % Not specified
Xtot = 1;
% a = 0.1;
PL = 225/Sb; 

switch PAR.scenario
    %           Line  from  to  R    X    B   tap
    case 1
        LINEDATA = [ 
        % Line  from  to   R   X   B  tap 
             1     1   3  0    Xtot*a   0   1
             1     2   3  0    Xtot*(1-a)   0   1
             1     3   4  0    0.1021   0   1];  
         BUSDATA=[
%          1     2   3     4       5       6    7    8      9      10
%        BUS  Type   Pgen  Qgen   Pload   Qload   YL  Ysh      V   Angle
           1     2   PL    NS      PL     0    0    0   1.0       0
           2     1   NS    NS     -PL     0    0    0   1.0      0
           1     3   NS    NS      0      0    0    0   1.0       0
           2     3   NS    NS      0      0    0    0   1.0      0]; 
    case 2  
        LINEDATA = [ 
        % Line  from  to   R   X   B  tap 
             1     1   3  0    Xtot*a   0   1
             1     2   3  0    Xtot*(1-a)   0   1
             1     3   4  0    0.1021   0   1];  
         BUSDATA=[
%          1     2   3     4       5       6    7    8      9      10
%        BUS  Type   Pgen  Qgen   Pload   Qload   YL  Ysh      V   Angle
           1     2   PL    NS      0     0    0    0   1.0       0
           2     1   NS    NS      0     0    0    0   1.0      0
           1     3   NS    NS      0      0    0    0   1.0       0
           2     3   NS    NS      0      0    0    0   1.0      0]; 
       
    case 3
        LINEDATA = [ 
        % Line  from  to   R   X   B  tap 
             1     1   3  0    Xtot*a   0   1
             1     2   3  0    Xtot*(1-a)   0   1
             1     3   4  0    0.1021   0   1];  
         BUSDATA=[
%          1     2   3     4       5       6    7    8      9      10
%        BUS  Type   Pgen  Qgen   Pload   Qload   YL  Ysh      V   Angle
           1     2   -PL    NS      0     0    0    0   1.0       0
           2     1   NS    NS     0     0    0    0   1.0      0
           1     3   NS    NS      0      0    0    0   1.0       0
           2     3   NS    NS      0      0    0    0   1.0      0];   
end


% BUSDATA
% Type=1 means slack-bus, Type=2 means PU-bus, Type=3 means PQ-bus
%if exact initial, flag_init=1
%if slack initial, flag_init=2
%if flat  initial, flag_init=3

flag_init=3;

Gbus=find(BUSDATA(:,2)==1 | BUSDATA(:,2)==2); % Generator buses
ng=length(Gbus); %Number of generators
nbus=length(BUSDATA(:,1)); %Number of buses

PAR.infbus = [];

% if PAR.infbus ~=0
%     Gbus(PAR.infbus) = [];
%     ng = ng - length(PAR.infbus);
% end

PAR.ng = ng;
PAR.nbus = nbus;
PAR.Gbus = Gbus;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Run load flow calculations
[VOLT,ANG,LINFLW,P_LOSS,YBUS,Pmat,Qmat,Pinj,Qinj,iter]=lflow(BUSDATA,LINEDATA,flag_init,tole,maxiter);

PAR.YBUS = YBUS;

% Find the generated active and reactive powers
PG= Pinj(Gbus)'+BUSDATA(Gbus,5) ; % must be an ngx1 vector containing the generated active powers (see equation (A-14) and BUSDATA)
QG= Qinj(Gbus)'+BUSDATA(Gbus,6); % must be an ngx1 vector containing the generated reactive powers (see equation (A-15) and BUSDATA)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%
%UG is an ngx1 vector containing the voltage magnitudes of the generator buses
UG=VOLT(Gbus);
%ANGG is an ngx1 vector containing the phase angles (rad.) of the generator buses
ANGG=ANG(Gbus);

% Analyticaly solve for the initial states.
I=conj((PG+j*QG)./(UG.*exp(j*ANGG)));
Eq=j*PAR.XDP.*I+UG.*exp(j*ANGG);
DELTA=angle(Eq);
OMEGA=zeros(ng,1);
EQP=abs(Eq);
EF=(PAR.XD.*EQP./PAR.XDP)-((PAR.XD-PAR.XDP).*UG.*cos(DELTA-ANGG)./PAR.XDP);

PAR.UREF = EF;
PAR.PM = PG;
PAR.EQP = EQP;
PAR.ANG0 = ANG;
PAR.U0 = VOLT;
PAR.PL0 = BUSDATA(:,5);
PAR.QL0 = BUSDATA(:,6);
PAR.LINFLW = LINFLW;

PAR.x0 = [DELTA;OMEGA];
if PAR.ns >= 3
    PAR.x0 = [PAR.x0; EQP];
end
PAR.nx = length(PAR.x0);

PAR.y0 = [PAR.ANG0; PAR.U0]; % two algebraic variables at each bus
PAR.ny = length(PAR.y0);

%% Governor
if isfield(opt,'GOV')
    if ~isfield(opt, 'GOV_type')
        opt.GOV_type = 1;
    end
    if opt.GOV_type == 1
        disp('Loading dynamic governors')
        [PAR.GOV, PAR.x0, PAR.nx] = ss_object(1, opt.GOV/(Sb*2*pi), PAR.x0, PAR.MBASE);
    elseif opt.GOV_type == 2 % the object "ramp_rat_limit.m" is outdated. Use ss_object
        disp('Loading dynamic governors with ramp rate limiters')
        [PAR.GOV, PAR.x0, PAR.nx] = ramp_rate_limit(1, opt.GOV/(Sb*2*pi), PAR.x0, PAR.MBASE);
    end
end

%% Exciter Control
if isfield(opt,'EXC')
    if ~isfield(opt, 'EXC_type')
        opt.EXC_type = 1;
    end
    if ~isfield(opt, 'EXC_max_out')
        opt.EXC_max_out = ones(ng,1).*[0,3];
    end
    
    if opt.EXC_type == 1 % Exciter with AVR
        if ~isfield(opt, 'EXC_ramp_lim')
            disp('Loading dynamic excitation controller')
            [PAR.EXC, PAR.x0, PAR.nx] = ss_object(1, opt.EXC, PAR.x0,  opt.EXC_max_out);
        else
            if size(opt.EXC_ramp_lim,1) == 1
                opt.EXC_ramp_lim = opt.EXC_ramp_lim(1,1)*ones(ng,1);
            end
            disp('Loading dynamic excitation controller with ramp rate limiter')
            [PAR.EXC, PAR.x0, PAR.nx] = ss_object(1, opt.EXC, PAR.x0, opt.EXC_max_out, opt.EXC_ramp_lim);
        end
        
        
    elseif opt.EXC_type == 2 % Exciter with AVR and PSS
        if ~isfield(opt, 'EXC_ramp_lim')
            disp('Loading dynamic excitation controller')
            [PAR.EXC, PAR.x0, PAR.nx] = ss_object(1, opt.EXC, PAR.x0,  opt.EXC_max_out);
        else
            if size(opt.EXC_ramp_lim,1) == 1
                opt.EXC_ramp_lim = opt.EXC_ramp_lim(1,1)*ones(ng,1);
            end
            disp('Loading dynamic excitation controller with ramp rate limiter')
            [PAR.EXC, PAR.x0, PAR.nx] = ss_object(1, opt.EXC, PAR.x0, opt.EXC_max_out, opt.EXC_ramp_lim);
        end
            PAR.EXC.type = 2; disp('Loading PSS')
            [PAR.PSS, PAR.x0, PAR.nx] = ss_object(1, opt.PSS, PAR.x0,  opt.PSS_max_out);
    end
end

%% Wind Control (Linear modell)
if isfield(opt,'PMU')  
    [PAR.PMU, PAR.x0, PAR.nx] = ss_object(1, opt.PMU, PAR.x0, ones(nbus,1).*[-1e6,1e6]);
end
if isfield(opt,'WIND')  
    assert(isfield(opt,'PMU'),'Wind turbine needs a PMU since it uses frequency measurement')  
    if ~isfield(opt, 'WIND_type')
        opt.WIND_type = 1;
    end
    if ~isfield(opt, 'WIND_max_out')
        opt.WIND_max_out = ones(nbus,1).*[-1e6,1e6];
    end
    
    if opt.WIND_type == 1
        disp('Loading linear wind turbine model')
        [PAR.WIND, PAR.x0, PAR.nx] = ss_object(1, opt.WIND/(Sb*2*pi), PAR.x0,  opt.WIND_max_out);
        [PAR.WIND_speed, PAR.x0, PAR.nx] = ss_object(1, opt.WIND_speed/(2*pi), PAR.x0,  ones(nbus,1).*[-1e6,1e6]);
    end
end

%% PODQ
if isfield(opt,'PODQ')  
    if ~isfield(opt, 'PODQ_type')
        opt.PODQ_type = 1;
    end
    
    if ~isfield(opt, 'PODQ_max_out')
        opt.PODQ_max_out = ones(nbus,1).*[-1e6,1e6];
    end
    
    if opt.PODQ_type == 1
        disp('Loading linear wind turbine model')
        [PAR.PODQ, PAR.x0, PAR.nx] = ss_object(1, opt.PODQ, PAR.x0,  opt.PODQ_max_out);
    end
end

%% Include algbraic variables
PAR.xy0=[PAR.x0; PAR.y0];
end