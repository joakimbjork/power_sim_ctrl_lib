function [PAR, opt] = data_Nordic5(opt)
% Joakim Björk, joakim.bjork@svk.se, 2022-09-22
disp('Initiating Nordic-machine test system');
ng = 5; % Number of machines

fs=50; % Nominal frequency [Hz]
ws=2*pi*fs; % Nominal frequency [rad/s]
Sb = 100; % MW system base

PAR = struct();
PAR.infbus = [];
PAR.Sb = Sb;
PAR.sys_func = @fg_xy_SM; % Function handle for simulation
PAR.lin_sys_func='fg_xy_SM'; % Function handle for linearization

% Classic machine model (ns = 2) or one-axis model (ns = 3)
if isfield(opt,'ns')
    PAR.ns = opt.ns;
else
    PAR.ns = 3;
end
if PAR.ns == 2
    disp('Loading classical machine model');
elseif PAR.ns == 3
    disp('Loading one-axis machine model');
elseif PAR.ns == 4
    disp('Loading two-axis machine model');
elseif PAR.ns == 6
    disp('Loading 6-th order machine model');
else
    PAR.ns = 2;
    disp('Loading classical machine model');
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Default dynamic object
if ~isfield(opt,'GOV')
    k = 1400/0.4; % Static FCR gain [MW/Hz];
    s = tf('s');
    Fgov = k*(1+7*s)/((1+2*s)*(1+14*s));%*(2-s)/(2+s); % Dynamic regulator/governor [MW/Hz];
    c_FCR = diag([0.6,0.3,0.1,0,0]); % Static participation factors, machines 1-5
    opt.GOV = Fgov*c_FCR;
end

%% Production
% %       1         2         3         4              5
% %       NO-hydro, SE-hydro, FI-hydro, SE-thermal, FI-thermal,
% Pgen = [9000;     6000;     2000;     5000;       2000]; % 110GWS case
% Pgen = [18000; 12000; 2000; 11000; 7000]; % 240GWS case
% Sgen = Pgen./[0.8;0.8;0.8;0.9;0.9];
% Wkin = Sgen.*[3;3;3;6;6]; % Gives kinetic energy =  110 GWs
% sum(Wkin)

H = [3; 3; 3; 6; 6];
Pgen = [9000; 6000; 2000; 5000; 2000];
S = Pgen./[0.8;0.8;0.8;0.9;0.9]*230/110;
S = Pgen./[0.8;0.8;0.8;0.7;0.9];
if isfield(opt,'machine_damping')
    D_tot = opt.machine_damping;
else
    D_tot = 0;
end

D = [150; 60; 20; 120; 50];
D = D*D_tot/sum(D);
D = D/(Sb*2*pi);

%            1    2     3    4      5     6      7      8    9    10
%          Gen   H     D     xt     xdp   xd     Tdop   xqp  xq   Tqop
GENDATA = [S(1)  H(1)  D(1)  0.15   0.25  1.1    5      0.7  0.7  1.5;
    S(2)  H(2)  D(2)  0.15   0.25  1.1    5      0.7  0.7  1.5;
    S(3)  H(3)  D(3)  0.15   0.25  1.1    5      0.7  0.7  1.5;
    S(4)  H(4)  D(4)  0.15   0.30  2.2    7      0.4  2.0  1.5;
    S(5)  H(5)  D(5)  0.15   0.30  2.2    7      0.4  2.0  1.5];%

if PAR.ns == 6 % Load default parameters for the subtransient dynamics
    %               11    12     13    14      15
    %               Xdpp  Tdopp  Xqpp  Tqopp   Xl
    GENDATA2 = [0.2   0.05   0.2    0.1   0.15 ;
        0.2   0.05   0.2    0.1   0.15 ;
        0.2   0.05   0.2    0.1   0.15 ;
        0.2   0.05   0.2    0.05   0.15 ;
        0.2   0.05   0.2    0.05   0.15 ];
    GENDATA = [GENDATA GENDATA2];
end

if isfield(opt,'TDOP') % Overide constant
    GENDATA(:,7) = opt.TDOP;
end
if isfield(opt,'TQOP') % Overide constant
    GENDATA(:,10) = opt.TQOP;
end
if isfield(opt,'one_ax') % Only idx "one_ax" will be modelled as one axis machines
    TDOP = GENDATA(:,7);
    GENDATA(:,7) = ones(ng,1)*1e9;
    GENDATA(opt.one_ax,7) = TDOP(opt.one_ax);
end
if isfield(opt,'two_ax') % Only idx "two_ax" will be modelled as two axis machines
    TQOP = GENDATA(:,10);
    GENDATA(:,10) = ones(ng,1)*1e6;
    GENDATA(opt.two_ax,10) = TQOP(opt.two_ax);
    XDP = GENDATA(:,5);
    XQP = GENDATA(:,8);
    GENDATA(:,8) =  XDP;
    GENDATA(opt.two_ax,8) = XQP(opt.two_ax);
    XQ = GENDATA(:,9);
    GENDATA(:,9) =  XDP;
    GENDATA(opt.two_ax,9) = XQ(opt.two_ax);
end

if isfield(opt,'four_ax') % Only idx "four_ax" will be modelled as four axis machines
    XDPP= GENDATA(:,11);
    XQPP = GENDATA(:,13);
    GENDATA(:,11) = GENDATA(:,5);
    GENDATA(:,13) = GENDATA(:,8);
    GENDATA(opt.four_ax,11) = XDPP(opt.four_ax);
    GENDATA(opt.four_ax,13) = XQPP(opt.four_ax);
end

%% Network data
% L_net = [0   -0.1364         0   -0.2182         0
%         -0.1364    0   -0.5455   -0.5455         0
%          0        -0.5455   0         0   -0.3636
%         -0.2182   -0.5455         0    0        0
%          0         0        -0.3636         0    0]*1e4; % MW*2*pi/rad

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
X0 = 0.9337*ws/1000; % X/km
Zbase = 400^2/Sb;
X = X0/Zbase;

NS=0; % "not specified"
P = Pgen/Sb;

% Line  from  to   R   X        B  tap
LINEDATA = [ 1     1     6    0   150*X/2    0   1
    2     2     4    0   130*X    0   1
    3     2     3    0   100*X    0   1
    4     3     5    0   150*X    0   1
    5     1     2    0   300*X    0   1
    6     6     4    0   150*X/2    0   1];

if isfield(opt,'subsystem')
    Zbase = 110^2/Sb;
    X = X0/Zbase;
    if opt.subsystem == 1
                % Line  from  to   R   X        B  tap
        SUB = [ 7      6     7    0   10*X    0   1];
        LINEDATA = [LINEDATA; SUB];

        if isfield(opt,'CONV')
            Cbus = opt.CONV.bus;
            if isfield(opt,'PC0')
                PC0 = opt.PC0;
            else
                PC0 = opt.CONV.MBASE.*[0.8];
            end
        else
            PC0 = zeros(SUB(end,1));
        end

        %            1  2     3     4     5      6      7   8    9    10
        %            BUS  Type  Pgen  Qgen  Pload  Qload  YL  Ysh  V    Angle
        SUB_BUSDATA =[7  3     PC0(1)    NS    0      0      0   0    1.0  0];
    elseif opt.subsystem == 2
                % Line  from  to   R   X        B  tap
        SUB = [ 7      1     7    0   10*X    0   1
                8      4     8    0   10*X    0   1];
        LINEDATA = [LINEDATA; SUB];

        if isfield(opt,'CONV')
            Cbus = opt.CONV.bus;
            if isfield(opt,'PC0')
                PC0 = opt.PC0;
            else
                PC0 = opt.CONV.MBASE.*[0.8; 0.8];
            end
        else
            PC0 = zeros(SUB(end,1));
        end

        %            1  2     3     4     5      6      7   8    9    10
        %            BUS  Type  Pgen  Qgen  Pload  Qload  YL  Ysh  V    Angle
        SUB_BUSDATA =[7  3     PC0(1)    NS    0      0      0   0    1.0  0
                      8  3     PC0(2)    NS    0      0      0   0    1.0  0];

    else % Base caase used in ISGT-paper
        % Line  from  to   R   X        B  tap
        SUB = [ 7      2     7    0   100*X    0   1
            8      7     8    0   10*X    0   1
            9      7     9    0   50*X    0   1
            10      6     10    0   100*X    0   1
            11      10     11    0   10*X    0   1
            12      10     12    0   10*X    0   1
            13      5     13    0   10*X    0   1
            14      1     14    0   10*X    0   1];
        LINEDATA = [LINEDATA; SUB];

        if isfield(opt,'CONV')
            Cbus = opt.CONV.bus;
            if isfield(opt,'PC0')
                PC0 = opt.PC0;
            else
                PC0 = opt.CONV.MBASE.*[0.6; 0.7; 0.5; 0.6; 0.8; 0.8];
            end
        else
            PC0 = zeros(SUB(end,1));
        end

        %            1  2     3     4     5      6      7   8    9    10
        %            BUS  Type  Pgen  Qgen  Pload  Qload  YL  Ysh  V    Angle
        SUB_BUSDATA =[7  3     NS    NS    0      0      0   0    1.0  0
            8  3     PC0(1)    NS    0      0      0   0    1.0  0
            9  3     PC0(2)    NS    0      0      0   0    1.0  0
            10  3     NS    NS    0      0      0   0    1.0  0
            11  3     PC0(3)    PC0(3)*0.1    0      0      0   0    1.0  0
            12  3     PC0(4)    PC0(4)*0.1    0      0      0   0    1.0  0
            13  3     PC0(5)    NS    0      0      0   0    1.0  0
            14  3     PC0(6)    NS    0      0      0   0    1.0  0];
    end
end

% Line resistance?
if isfield(opt,'Rline')
    LINEDATA(:,4) = opt.Rline*LINEDATA(:,5);
end

%            1                  2          3         4                5
%            NO-hydro,    SE-hydro,  FI-hydro, SE-thermal,      FI-thermal
PL = Pgen + [1250;        -5500;     0;        3750;            500];
PL = Pgen + [500;        -4000;     0;        2500;            500];


% Changing load on generator terminal buses
if isfield(opt,'dPL')
    dPL = opt.dPL;
else
    dPL = zeros(ng,1);
end
if isfield(opt,'dPL_Norway')
    dPL(1) = dPL(1)+opt.dPL_Norway; % Shifts load between Norwary and Sweden (south)
    dPL(4) = dPL(4)-opt.dPL_Norway;
end

PL = PL + dPL;

if isfield(opt,'zero_power_flow') % Overrides load distribution settings
    PL = Pgen;
    disp('Loading model with no power flow on transmission lines')
end

PL = PL/Sb;
if isfield(opt,'dQL')
    dQL = opt.dQL;
else
    dQL = zeros(6,1);
end

QL = dQL;
% Adjust Y shunt to fix voltage/reactive power issues, if necessary
% Ysh = PL*0.03+ [0;         1000;         0;        2000;   500];
%          1  2     3     4     5      6      7   8    9    10
%        BUS  Type  Pgen  Qgen  Pload  Qload  YL  Ysh  V    Angle
BUSDATA=[  1  2     P(1)  NS    PL(1)  QL(1)  0   0    1.0  0
           2  2     P(2)  NS    PL(2)  QL(2)  0   0    1.0  0
           3  2     P(3)  NS    PL(3)  QL(3)  0   0    1.0  0
           4  1     P(4)  NS    PL(4)  QL(4)  0   0    1.0  0
           5  2     P(5)  NS    PL(5)  QL(5)  0   0    1.0  0
           6  3     NS    NS    0      QL(6)  0   0    1.0  0];
if isfield(opt,'subsystem')
    BUSDATA = [BUSDATA;SUB_BUSDATA];
end

% BUSDATA
% Type=1 means slack-bus, Type=2 means PU-bus, Type=3 means PQ-bus
%if exact initial, flag_init=1
%if slack initial, flag_init=2
%if flat  initial, flag_init=3

%% Standard initiation information
data_initiation;
