opt = struct();
include_exc = true;
if include_exc
    opt.ns = 6; % States/machine (ns = 2 -> Classical model)
end
% opt.zero_power_flow =0; % Load model with no power flow on transmission lines (to not care about transient stability etc)
opt.mq = 2;
opt.Rline = 0.1;
% cbus = [5,6];
% cbus = [2,3,5,6];
opt.subsystem = 1;
cbus = [7]; % Closest node [2,4]
CONV_BASE = [20]';
opt.PC0 = CONV_BASE.*[0.4];

% opt.subsystem = 2;
% cbus = [7,8]; % Closest node [2,4]
% CONV_BASE = [20,20]';
% opt.PC0 = CONV_BASE.*[0.8,0.8];


%% SCP at network nodes
[PAR] = data_Nordic5(opt); % Modell 
G = lin_fg_xy(PAR.xy0,PAR);
u_idx = PAR.nbus + (1:PAR.nbus);% + idx; % -QL(idx);
y_idx = PAR.nbus + (1:PAR.nbus);% + idx; % V(idx);
G = (G(y_idx,u_idx));
SCP = 1./(diag(G.D));

opt.data.SCP = SCP; % Short circuit power (admittance)


%% Hydro governors at machine nodes (5 machine nodes)
opt.machine_damping = 400;

k = 1400/0.4; % Static gain [MW/Hz];
s = tf('s');
% F_desired = k/(1+5*s);
F_desired = k*(7*s+1)/(2*s+1)/(17*s+1);
z_hydro = 2;
H_hydro = 2*(z_hydro-s)/(2*z_hydro+s); % Dynamic regulator/governor [MW/Hz];
c_FCR = diag([0.6,0.3,0.1,0,0]); % Participation factors (sum=1)
opt.GOV = minreal(H_hydro*F_desired*c_FCR);

if include_exc
    %% Excitation system at machines (5 machine nodes)
    % See "https://github.com/joakimbjork/Nordic5/Nordic5_doc.pdf"
    s= tf('s'); 
    % K1 = 100/(0.001*s+1);
    % K2 = 0.001*s/(0.1*s+1);
    % EXCfilter = 1/(0.02*s+1);
    % EXC = minreal(EXCfilter*K1/(1+K1*K2));
    
    TB = 13; TA = 0.2539*TB; TE = 0.02;
    EXC = (s*TA+1)/(s*TB+1)/(s*TE+1)*150; % Exciter med AVR
    
    opt.EXC_max_out = ones(PAR.ng,1).*[0,5]*1;
    opt.EXC = eye(PAR.ng)*EXC; %
    
    %% PSS at machines (5 machine nodes)
    % [PAR] = data_Nordic5(opt); % Modell 
    % PAR.input = 'UREF'; 
    % G = lin_fg_xy(PAR.xy0,PAR);
    % Vt_Uref = G((PAR.nbus+1):(PAR.nbus+PAR.ng),:);
    % [fig,name,co] = figureLatex(3.5,3.5/sqrt(2));
    % bodeopt = struct();
    % bodeopt.wrap = false;
    % % bodeopt.FreqUnits = 'Hz';
    % for i = 1:PAR.ng
    %     bodeopt  = bodeLatex(inv(Vt_Uref(i,i)),bodeopt);
    % end
    % grid on
    
    % % Tuning based on "average" PSS in "https://github.com/joakimbjork/Nordic5/Nordic5_doc.pdf"
    % T1 = 0.18;
    % T2 = 0.97;
    % T3 = 0.18;
    % T4 = 0.97;
    PSS = 4.5*s/(4.5*s+1) *(s/10+1)/(s/40+1)*(s/10+1)/(s/40+1)/(s*0.01+1);
    PSS = minreal(PSS);
    % bodeopt.linestyle = '--';
    % bodeopt  = bodeLatex(PSS,bodeopt);
    
    % 
    opt.PSS_type=1; % Rotor speed feedback
    opt.PSS_max_out = ones(PAR.ng,1).*[-0.05,0.05]*1;
    opt.PSS = diag([0.9,0.9,0.9,0.9,1])*PSS*0.08;
end
%% Converter buses at network nodes  (6 network nodes)

m = length(cbus);

opt.CONV = struct();
opt.CONV.bus = cbus;

opt.CONV.GFM = 1;
opt.CONV.MBASE = CONV_BASE; % pu on system base
% opt.CONV.Storage = ss(0.2*eye(m));
opt.CONV.X = ones(m,1)*0.05;

[PAR] = data_Nordic5(opt); % Modell 
opt.data.ESCP = 1./(1./SCP(cbus)+PAR.CONV.X);

% Each converter uses an HVDC_wind object for its energy storage. Default
% settings -> load_HVDC_wind.m
opt.CONV.HVDC_wind = struct();
opt.CONV.HVDC_wind.bus = cbus;
opt.CONV.HVDC_wind.MBASE = opt.CONV.MBASE;

%% POD-P control
% c_POD = opt.CONV.MBASE/sum(opt.CONV.MBASE);
% k_CONV = c_POD*k; % MW/Hz
k_CONV = opt.CONV.MBASE*PAR.Sb*0.5; % MW/Hz
% k_CONV = ones(m,1)*k/m; % MW/Hz
opt.PODP = tf(zeros(PAR.nbus));
w1 = 0.25*2*pi;
wa = w1/2;
w2 = 1*2*pi;
wb = w2*2;
for i = 1:m % POD-P at non-converter buses models ideal constant power loads
    ii = cbus(i);
    opt.PODP(ii,ii) = ss(k_CONV(i)*s/(s+wa)*wb/(s+wb));
    % opt.PODP(ii,ii) = ss(k_CONV(i)*100/(s+100));
end


%% POD-Q control

opt.PODQ_type = 4;
% Sum block
u_max = 1/3; % Maximum input [p.u.]
opt.PODQ = tf(zeros(PAR.nbus));
vec = zeros(PAR.nbus,1);
vec(cbus) = opt.CONV.MBASE;
opt.PODQ_max_out = [-1,1].*vec*u_max*PAR.Sb;
% Com delay
opt.PODQ_COM = tf(zeros(PAR.nbus));
% Derivative controller
opt.PODQ1 = tf(zeros(PAR.nbus));
u_max = 0.05; % Maximum input [p.u.]
opt.PODQ1_max_out = [-1,1].*ones(PAR.nbus,1)*u_max*PAR.Sb;
% Proportional controller
opt.PODQ2 = tf(zeros(PAR.nbus));
% opt.PODQ2_max_out = [-1,1].*ones(PAR.nbus,1)*u_max*PAR.Sb;
% Aux dead band controller
u_max = 1/3 - 0.05;
db = 0.03;
deadband = [-db, +db];
opt.PODQaux = tf(zeros(PAR.nbus));
opt.PODQaux_dead_band= deadband.*ones(PAR.nbus,1); % Relative deadband, ex. voltage within [-0.02, 0.02] p.u.
opt.PODQaux_max_out = [-1,1].*ones(PAR.nbus,1)*u_max*PAR.Sb;

for i = 1:m
    ii = cbus(i);
    Te = 0.01; 
    t_1 = 1;
    Pdc = opt.CONV.MBASE(i);
    Qmax = 1/3;
    ESCP = opt.data.ESCP(i); % Short circuit power [pu] (system base)
    SCR = ESCP/opt.CONV.MBASE(i); % Short circuit ratio 
    opt.data.SCR(i) = SCR;
    R = 0.03;
    % c1 = -log(0.1)/t_1  * R*SCR/(1+R*SCR);
    c1 = 0.7;
    c1 = c1*1.5;
    c2 = c1*10;
    Kc = Qmax/R * c1/(s+c1)*(s+c2)/c2; % Voltage controller
    
    f_low = 0.25;
    f_high = 1;
    
    k_q = 50;
    
    a1 = (2*pi*f_low)/10;
    a2 = (2*pi*f_high)*10;
    b1 = a1*1;
    b2 = b1*k_q*a1/a2*R/Qmax;
    
    % Deadband ctrl
    Kc_10 = Qmax/R * c1/(s+c1/10)*(s+c2)/c2;
    Ka =  k_q *s/(s+a2)*(s+b1)/(s+b2);
    Kb =  k_q *a1/(s+a2)*(s+b1)/(s+b2);
   
    % Sum block
    opt.PODQ(ii,ii) = Pdc/(s*Te+1); % [Mvar/p.u. voltage]
    % Com delay
    opt.PODQ_COM(ii,ii) = tf(1); % [Mvar/p.u. voltage]
    % Derivative controller
    opt.PODQ1(ii,ii) = Ka*PAR.Sb; % [Mvar/p.u. voltage]
    % Proportional controller
    opt.PODQ2(ii,ii) = Kb*PAR.Sb; % [Mvar/p.u. voltage]
    % Aux dead band controller
    opt.PODQaux(ii,ii) = Kc_10*PAR.Sb; % [Mvar/p.u. voltage]
end

%% Load system
[PAR, opt] = data_Nordic5(opt); % Modell 