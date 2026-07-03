function [PAR] = data_two_machine_park(opt)
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
Sr = 10000; % Rated power [MW]
if isfield(opt,'machine_damping')
    D = opt.machine_damping/(Sb*2*pi);
else
    D = 1e-7*ones(2,1);
end

if isfield(opt,'period_time')
    period_time = opt.period_time;
else
    period_time = 2;
end
PAR.period_time = period_time;

if isfield(opt,'Wkin')
    Wkin = opt.Wkin;
else
    Wkin = 200e3;
end
if isscalar(Wkin) 
    H = [0.5,0.5]*Wkin/Sr;
else
    H = Wkin/Sr;
end



%            1    2     3    4      5     6      7      8    9    10
%          Gen   H     D     xt     xdp   xd     Tdop   xqp  xq   Tqop
GENDATA = [Sr    H(1)  D(1)  0.1   0.1    0.7    5      0.4  2.0  1.5;
           Sr    H(2)  D(2)  0.1   0.1    0.7    5      0.4  2.0  1.5];

if PAR.ns == 6 % Load default parameters for the subtransient dynamics
%           1 - 10      11    12     13    14      15
%           -------    Xdpp  Tdopp  Xqpp  Tqopp   Xl
    GENDATA = [GENDATA [0.25  0.074   0.25    0.13   0.16 ].*ones(size(GENDATA,1),1)];
end


%% Network data
if isfield(opt,'a')
    a = opt.a;
else
    a = 0.102;
end

if isfield(opt,'X34')
    X34 = opt.X34;
else
    X34 = 0.1021;
end

NS = 0; % Not specified
M = [0.5,0.5]*2*sum(Wkin)/ws; % Assume symetrical inertias when calculating impedance
freq_osc = 0.5*2*pi; % Frequency of interarea mode [rad/s]
Xtot = sum(M)/(prod(M) * freq_osc^2)*Sb;
% a = 0.1;

LINEDATA = [ 
% Line  from  to   R   X            B  tap 
     1     1   3  0    Xtot*a       0   1
     2     2   3  0    Xtot*(1-a)   0   1
     3     3   4  0    X34          0   1];  

BUSDATA=[
    %  1     2   3      4       5     6     7    8   9      10    
    %  BUS  Type Pgen   Qgen  Pload   Qload YL   Ysh V    Angle   
       1     2   0.5    0     0       0    0    0   1.0  0 
       2     1   -0.5   0     0       0    0    0   1.0  0  
       3     3   0      0     0.3     0    0    0   1.0  0 
       4     3   0.3    0     0       0    0    0   1.0  0  ]; 
if isfield(opt,'PL')
    PL = opt.PL;
    nbus = length(PL);
    assert(nbus>=4)
    QL = opt.QL;
    PG = opt.PG;
    QG = opt.QG;
    
        BUSDATA=[
    %  1     2   3      4       5     6     7    8   9      10    
    %  BUS  Type Pgen   Qgen  Pload   Qload YL   Ysh V    Angle   
       1     2   PG(1)  QG(1)    PL(1)   QL(1)    0    0   1.0  0 
       2     1   PG(2)  QG(2)    PL(2)   QL(2)    0    0   1.0  0  
       3     3   PG(3)  QG(3)    PL(3)   QL(3)    0    0   1.0  0 
       4     3   PG(4)  QG(4)    PL(4)   QL(4)    0    0   1.0  0  ]; 
    for i = 5:nbus
        BUSDATA = [BUSDATA;
       i     3   PG(i)  QG(i)    PL(i)   QL(i)    0    0   1.0  0 ];
            line_n = size(LINEDATA,1)+1;
        if i == 7
            X4i = X34;
        else
            X4i = X34/10;
        end
        LINEDATA = [LINEDATA;
            % Line   from  to   R   X            B  tap 
             line_n  4     i  X4i*0   X4i       0   1];
    end

end





% BUSDATA
% Type=1 means slack-bus, Type=2 means PU-bus, Type=3 means PQ-bus
%if exact initial, flag_init=1
%if slack initial, flag_init=2
%if flat  initial, flag_init=3


%% Standard initiation information
data_initiation;