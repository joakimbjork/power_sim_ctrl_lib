function [PAR] = data_one_machine(opt)
ng = 1; % Number of machines

tole=1e-6;
maxiter=10;
fs=50; % Nominal frequency [Hz]
ws=2*pi*fs; % Nominal frequency [rad/s]
Sb = 1000; % MW system base

PAR = struct();
PAR.sys_func = @fg_xy_SM; % Function handle for simulation
PAR.lin_sys_func='fg_xy_SM'; % Function handle for linearization

PAR.ws = ws;
PAR.mp = 0;  % Active load characteristic
PAR.mq = 0; % Reactive load characteristic
PAR.infbus = [];
PAR.Sb = Sb;

%% Generator data
% Total kinetic energy [MWs].

Wkin = 10000*100;

% Total rated power of machines participating in FCR

Sr_tot = 21250; % Rated power [MW]  
Sr_tot = 1e9; % Rated power [MW]  

% Total load damping [MW/Hz].
Dtot = 0;

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

%            1  2  3   4   5     6       7      
%          Gen  H  D  xt   xdp   xd       Tdop     
GENDATA = [Sr   H  D  0.15   0.35  1        5   ];% 

PAR.MBASE = GENDATA(:,1)/Sb;
PAR.M=2*(GENDATA(:,2)).*PAR.MBASE/ws; 
PAR.D=GENDATA(:,3); 
PAR.XDP=(GENDATA(:,4)+GENDATA(:,5))./PAR.MBASE; 
PAR.XD=(GENDATA(:,4)+GENDATA(:,6))./PAR.MBASE;
PAR.TDOP=GENDATA(:,7);

%% Line data 
% The parameter, Xline, not matter for the one machine model this is only 
% to get the initiation to work
if isfield(opt,'Xline') 
    Xline = opt.Xline;
else
    Xline = 0.1;  
end


%% Network data
if isfield(opt,'PL')
    PL1 = opt.PL/Sb;
else
    PL1 = 0;
end

if isfield(opt,'QL')
    QL1 = opt.QL/Sb;
else
    QL1 = 0;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Line  from  to   R   X       B  tap 
LINEDATA = [ 1     1   2   0   Xline   0   1]; 

PC0 = opt.CONV.MBASE*[0.5];
NS=0; % "not specified"
%          1  2     3     4     5      6      7   8    9    10
%        BUS  Type  Pgen  Qgen  Pload  Qload  YL  Ysh  V    Angle
BUSDATA=[  1  1     PL1   NS    PL1    QL1      0   0    1.0  0;
           2  3     PC0     0     0      0        0     0     1.0  0];


%% Standard initiation information
data_initiation;


% % BUSDATA
% % Type=1 means slack-bus, Type=2 means PU-bus, Type=3 means PQ-bus
% %if exact initial, flag_init=1
% %if slack initial, flag_init=2
% %if flat  initial, flag_init=3
% 
% 
% 
% flag_init=3;
% 
% Gbus=find(BUSDATA(:,2)==1 | BUSDATA(:,2)==2); % Generator buses
% assert(ng==length(Gbus)); %Number of generators
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
% %% Modifications for one machine model
% % Remove network data. Model should only contain one machine and its
% % machine terminal
% PAR.nbus = 1;
% PAR.YBUS = [];
% VOLT = VOLT(1);
% ANG = ANG(1);
% BUSDATA = BUSDATA(1,:);
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
% PAR.EDP = 0*EQP;
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
% PAR.u_count = PAR.ng*2+PAR.nbus*2; % Number of standard control signals [PM; EF; PL; QL]
% 
% %% Governor
% [PAR,opt] = load_GOV(PAR,opt);
% 
% %% PMU
% [PAR,opt] = load_PMU(PAR,opt);
% 
% %% Wind Turbine
% [PAR,opt] = load_WIND(PAR,opt);
% 
% %% Include algbraic variables
% PAR.xy0=[PAR.x0; PAR.y0];
end