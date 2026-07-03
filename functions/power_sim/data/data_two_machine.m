function [PAR] = data_two_machine(opt)
ng = 2; % Number of machines

fs=50; % Nominal frequency [Hz]
ws=2*pi*fs; % Nominal frequency [rad/s]
Sb = 1000; % MW system base

PAR = struct();
PAR.infbus = [];
PAR.Sb = Sb;
PAR.sys_func = @fg_xy_SM; % Function handle for simulation
PAR.lin_sys_func='fg_xy_SM'; % Function handle for linearization

disp('This model is outdate. Consider using "data_PODQ_test_system" instead')


%% Generator data
% Total kinetic energy [MWs].
if isfield(opt,'Wkin') 
    Wkin = opt.Wkin; 
else
    Wkin = 110*1000; 
end

% Total rated power of machines participating in FCR
if isfield(opt,'Sr_tot') 
    Sr_tot = opt.Sr_tot; % Rated power [MW]
else
    Sr_tot = 21250; % Rated power [MW]  
end

% Total load damping [MW/Hz].
if isfield(opt,'LoadDamping') 
    Dtot = opt.LoadDamping/(Sb*2*pi); 
elseif isfield(opt,'machine_damping') 
    Dtot = opt.machine_damping/(Sb*2*pi);
else
    Dtot = 0/(Sb*2*pi); 
end

% Classic machine model (ns = 2) or one-axis mpdel (ns = 3)?
if isfield(opt,'ns') 
    PAR.ns = opt.ns;
else
    PAR.ns = 2;    
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
H = Wkin/Sr_tot; % Inertia constant [s]
Sr = Sr_tot/ng;
D = Dtot/ng;

%            1  2   3    4   5     6       7      
%          Gen  H   D    xt   xdp   xd        Tdop     
GENDATA = [Sr   H   D    0.15   0.35  1       5   ;
           Sr   H   D    0.15   0.35  1        5 ];% 
PAR.MBASE = GENDATA(:,1)/Sb;
PAR.M=2*(GENDATA(:,2)).*PAR.MBASE/ws;
PAR.D=GENDATA(:,3);

PAR.XDP=(GENDATA(:,4)+GENDATA(:,5))./PAR.MBASE;
PAR.XD=(GENDATA(:,4)+GENDATA(:,6))./PAR.MBASE;
PAR.TDOP=GENDATA(:,7);

%% Network data
if isfield(opt,'f_interarea_mode')
    assert(length(PAR.MBASE)==2, 'Only designed for two machine system'); % 
    w = opt.f_interarea_mode*2*pi; % frequency of interarea mode [Hz] -> [rad/s]
else
    w = 0.5*2*pi; % Default freqeuncy is 0.5*2*pi [rad/s]
end

Xtot = sum(PAR.M)/(prod(PAR.M) * w^2);
Xline = Xtot-sum(PAR.XDP);
PAR.Xline = Xline;

if isfield(opt,'P12')
    Pg1 = opt.P12/Sb;
else
    Pg1 = 0;
end

%% Network data
if isfield(opt,'PL')
    PL1 = opt.PL/Sb/ng;
    PL2 = PL1;
else
    PL1 = 0;
    PL2 = PL1;
end

if isfield(opt,'QL')
    QL1 = opt.QL/Sb/ng;
    QL2 = QL1;
else
    QL1 = 0;
    QL2 = QL1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Line  from  to   R   X       B  tap 
LINEDATA = [ 1     1   2   0   Xline   0   1];       

NS=0; % "not specified"
%          1  2     3     4     5      6      7   8    9    10
%        BUS  Type  Pgen  Qgen  Pload  Qload  YL  Ysh  V    Angle
BUSDATA=[  1  2   PL1+Pg1 QL1   PL1    QL1      0   0    1.0  0
           2  1     NS    NS    PL2    QL2      0   0    1.0  0];

% BUSDATA
% Type=1 means slack-bus, Type=2 means PU-bus, Type=3 means PQ-bus
%if exact initial, flag_init=1
%if slack initial, flag_init=2
%if flat  initial, flag_init=3

%% Standard initiation information
data_initiation;

% flag_init=3;
% 
% Gbus=find(BUSDATA(:,2)==1 | BUSDATA(:,2)==2); % Generator buses
% ng=length(Gbus); %Number of generators
% nbus=length(BUSDATA(:,1)); %Number of buses
% 
% % if PAR.infbus ~=0
% %     Gbus(PAR.infbus) = [];
% %     ng = ng - length(PAR.infbus);
% % end
% 
% PAR.ng = ng;
% PAR.nbus = nbus;
% PAR.Gbus = Gbus;
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Run load flow calculations
% [VOLT,ANG,LINFLW,P_LOSS,YBUS,Pmat,Qmat,Pinj,Qinj,iter]=lflow(BUSDATA,LINEDATA,flag_init,tole,maxiter);
% 
% PAR.YBUS = YBUS;
% 
% % Find the generated active and reactive powers
% PG= Pinj(Gbus)'+BUSDATA(Gbus,5) ; % must be an ngx1 vector containing the generated active powers (see equation (A-14) and BUSDATA)
% QG= Qinj(Gbus)'+BUSDATA(Gbus,6); % must be an ngx1 vector containing the generated reactive powers (see equation (A-15) and BUSDATA)
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% 
% %% 
% %UG is an ngx1 vector containing the voltage magnitudes of the generator buses 
% UG=VOLT(Gbus);
% %ANGG is an ngx1 vector containing the phase angles (rad.) of the generator buses 
% ANGG=ANG(Gbus);
% 
% % Analyticaly solve for the initial states.
% I=conj((PG+j*QG)./(UG.*exp(j*ANGG)));
% Eq=j*PAR.XDP.*I+UG.*exp(j*ANGG);
% DELTA=angle(Eq); 
% OMEGA=zeros(ng,1);
% EQP=abs(Eq); 
% EF=(PAR.XD.*EQP./PAR.XDP)-((PAR.XD-PAR.XDP).*UG.*cos(DELTA-ANGG)./PAR.XDP);
% 
% PAR.UREF = EF;
% PAR.PM = PG;
% PAR.EQP = EQP;
% PAR.ANG0 = ANG;
% PAR.U0 = VOLT;
% PAR.PL0 = BUSDATA(:,5);
% PAR.QL0 = BUSDATA(:,6);
% PAR.LINFLW = LINFLW;
% 
% PAR.x0 = [DELTA;OMEGA];
% if PAR.ns >= 3
%     PAR.x0 = [PAR.x0; EQP];
% end
% PAR.nx = length(PAR.x0);
% 
% PAR.y0 = [PAR.ANG0; PAR.U0]; % two algebraic variables at each bus
% PAR.ny = length(PAR.y0); 
% 
% %% Governor
% if isfield(opt,'GOV')
%     if ~isfield(opt, 'GOV_type')
%         opt.GOV_type = 1;
%     end
%     if opt.GOV_type == 1
%         disp('Loading dynamic governors')
%         [PAR.GOV, PAR.x0, PAR.nx] = ss_object(1, opt.GOV/(Sb*2*pi), PAR.x0, PAR.MBASE);        
%     elseif opt.GOV_type == 2
%         disp('Loading dynamic governors with ramp rate limiters')
%         [PAR.GOV, PAR.x0, PAR.nx] = ramp_rate_limit(1, opt.GOV/(Sb*2*pi), PAR.x0, PAR.MBASE);
%     end
% end
% 
% %% Include algbraic variables
% PAR.xy0=[PAR.x0; PAR.y0];
% end