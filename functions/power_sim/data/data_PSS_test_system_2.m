function [PAR] = data_PSS_test_system_2(opt)
ng = 2;

fs=50; % Nominal frequency [Hz]
ws=2*pi*fs; % Nominal frequency [rad/s]
Sb = 1000; % MW system base

PAR = struct();
PAR.infbus = [];
PAR.Sb = Sb;
PAR.sys_func = @fg_xy_SM; % Function handle for simulation
PAR.lin_sys_func='fg_xy_SM'; % Function handle for linearization

% Classic machine model (ns = 2) 
if isfield(opt,'ns')
    PAR.ns = opt.ns;
else
    PAR.ns = 2;
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

if isfield(opt,'machine_damping')
    D = opt.machine_damping/(Sb*2*pi);
else
    D = 1e-7*ones(2,1);
end

% if isfield(opt,'Wkin')
%     Wkin = opt.Wkin;
% else
%     Wkin = 180*1e3; % Default system inertia is 180 GWs 
% %     H = period_time^2*50/(10*2*pi)/(0.5);
% %     H = H*ones(2,1);
% end

% Havg = 3.8; % Inertia constant in each of the large two areas.

% Stot = Wkin/Havg; % Total rated power in the two areas. Default should be around 47 368 MW
Savg = 10000;

Xtot = 1;

if isfield(opt,'period_time')
    period_time = opt.period_time;
else
    period_time = 2;
end
PAR.period_time = period_time;

Havg = period_time^2 * 50/(2*pi) * Sb/Savg * 1/Xtot;

% Machine 3
Sr3 = 377; % Rated power [MW]
H3 = 2.28; 
D3 = 0;
PG3 = Sr3*0.9/Sb;

%            1    2     3    4      5     6      7      8    9    10
%          Gen    H     D     xt     xdp   xd     Tdop   xqp  xq   Tqop
GENDATA = [Savg   Havg  D(1)  0.1   0.1    0.7    inf      0.4  2.0  inf;
           Savg   Havg  D(2)  0.1   0.1    0.7    inf      0.4  2.0  inf;
           Sr3    H3    D3    0.15  0.35   1      5     0.59    0.59  1.5];

if PAR.ns == 6 % Load default parameters for the subtransient dynamics
%           1 - 10      11    12     13    14      15
%           -------    Xdpp  Tdopp  Xqpp  Tqopp   Xl
    GENDATA = [GENDATA [0.25  0.074   0.25    0.13   0.16 ].*ones(size(GENDATA,1),1)];
end

GENDATA(3,11:15) = [ 0.25  0.074   0.25    0.13   0.16 ];
%% Network data




if isfield(opt,'a')
    a = opt.a;
else
    a = 0.1;
end

if isfield(opt,'X34')
    X34 = opt.X34;
else
    X34 = 2;
end



if isfield(opt,'PL') && ~isempty(opt.PL)
    PL = opt.PL;
else
    PL = zeros(4,1);
    PL(1) = 225/Sb;
    PL(2) = -PL(1);
end

if isfield(opt,'PG') % 
    PG = opt.PG;
else
    PG = zeros(4,1);
    PG(1) = 225/Sb;
    %OBS (bus 2 is slack bus)
end

NS = 0; % Not specified

LINEDATA = [ 
% Line  from  to   R   X            B  tap 
     1     1   4  0    Xtot*a       0   1
     1     2   4  0    Xtot*(1-a)   0   1
     1     3   4  0    X34          0   1];  
BUSDATA=[
%  1     2   3      4       5     6     7    8   9      10
%  BUS  Type Pgen   Qgen  Pload   Qload YL   Ysh V    Angle
   1     1   PG(1)  NS    PL(1)   0     0    0   1.0  0
   2     2   PG(2)  NS    PL(2)   0     0    0   1.0  0
   3     2   PG3    NS    PL(3)   0     0    0   1.0  0
   4     3   PG(4)  NS    PL(4)   0     0    0   1.0  0]; 



% BUSDATA
% Type=1 means slack-bus, Type=2 means PU-bus, Type=3 means PQ-bus
%if exact initial, flag_init=1
%if slack initial, flag_init=2
%if flat  initial, flag_init=3

%% Standard initiation information
data_initiation;