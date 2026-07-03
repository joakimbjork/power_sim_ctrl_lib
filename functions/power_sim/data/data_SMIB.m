function [PAR] = data_SMIB(opt)

fs=50; % Nominal frequency [Hz]
ws=2*pi*fs; % Nominal frequency [rad/s]
Sb = 1000; % MW system base

PAR = struct();
PAR.infbus = 2;
PAR.Sb = Sb;
PAR.sys_func = @fg_xy_SM; % Function handle for simulation
PAR.lin_sys_func='fg_xy_SM'; % Function handle for linearization

%% Generator data
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
Sr = 377; % Rated power [MW]
if isfield(opt,'machine_damping')
    D = opt.machine_damping/(Sb*2*pi);
else
    D = 0;
end

% Typical parameter values (Kundur, p. 153)
% Thermal machine:
% Xd = [1.0 - 2.3]
% Xq = [1.0 - 2.3]
% Xdp = [0.15 - 0.4]
% Xqp = [0.3 - 1.0]
% Xdpp = [0.12 - 0.25]
% Xqpp = [0.12 - 0.25]
% Xl = [0.1 - 0.2]
% Tdop = [3.0 - 10.0]
% Tqop = [0.5 - 2.0]
% Tdopp = [0.02 - 0.05]
% Tqopp = [0.02 - 0.05]
%            1    2     3  4     5     6    7       8    9    10 
%          Gen    H     D  xt    xdp   xd   Tdop  xqp    xq  Tqop  
GENDATA = [Sr     2.28  D  0.15  0.35  1    5     0.59    0.59   1.5];%
% GENDATA = [Sr     2.28  D  0.1  0.1   0.7    5     0.4    2.0   1.5];%


if PAR.ns == 6
%           1 - 10      11    12     13    14      15
%           -------    Xdpp  Tdopp  Xqpp  Tqopp   Xl
    % GENDATA = [GENDATA 0.25  0.074   0.25    0.13   0.16 ];
    GENDATA = [GENDATA 0.25  0.074   0.25    0.13   0.16 ];
end

GENDATA = GENDATA.*[1;1]; % The implementation requires a two machine model
GENDATA(2,1) = GENDATA(2,1)*1e6;

%% Line Data

% Example
% Strong connection: X = 1;
% Weak connection: X = 2.4;

if isfield(opt,'scenario')
    PAR.scenario = opt.scenario;
else
    PAR.scenario = 1;
end

switch PAR.scenario
    %           Line  from  to  R    X    B   tap
    case 1
        PG1 = 0.9*Sr/Sb;
        LINEDATA = [1     1     2   0  1    0   1];
    case 2
        PG1 = 0.9*Sr/Sb;
        LINEDATA = [1     1     2   0  2.4  0   1];
    case 3
        PG1 = -0.9*Sr/Sb;
        LINEDATA = [1     1     2   0    1    0   1];
    case 4
        PG1 = -0.9*Sr/Sb;
        LINEDATA = [1     1     2   0    2.4  0   1];
end

if isfield(opt,'PG1')
    PG1 = opt.PG1*Sr/Sb;
end
if isfield(opt,'PL1')
    PL1 = opt.PL1*Sr/Sb;
else
    PL1 = 0;
end

NS = 0; % Not specified

BUSDATA=[
    %          1     2     3     4       5       6    7    8      9      10
    %        BUS  Type  Pgen  Qgen   Pload   Qload   YL  Ysh      V   Angle
              1     2   PG1    NS    PL1      0    0    0   1.0       0
              2     1   NS     NS    0      0    0    0   1.0      0];


% BUSDATA
% Type=1 means slack-bus, Type=2 means PU-bus, Type=3 means PQ-bus
%if exact initial, flag_init=1
%if slack initial, flag_init=2
%if flat  initial, flag_init=3

%% Standard initiation information
data_initiation;
