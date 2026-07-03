opt = struct();
opt.a = 0.25;
opt.period_time = 1.5;
opt.mp = 2;
% opt.X34 = 1;
% opt.machine_damping = [1,1]*50;

f_comp = 1;

opt.PL = [2, 2, 0, 0]*0;
opt.PG = 1*[0.5, -0.5 , 0, 0]+opt.PL; 

PAR = data_PODQ_test_system(opt);
nbus = PAR.nbus;
G = lin_fg_xy(PAR.xy0,PAR);
bus_idx = 4;
u_idx = nbus + bus_idx; % -QL(4);
y_idx = nbus + bus_idx; % V(4);
G = minreal(G(y_idx,u_idx));

%% Actuator dynamics
s = tf('s');
wE = 25;
TD = 0.075;
H0 = wE/(s+wE);
Hcom = 1*exp(-s*TD);
H = Hcom*H0;


%% Voltage Controller
t_1 = 1;
SCP = 1/G.D;
Qmax = 1; % Rate power on per unit base
SCR = SCP/Qmax;
R = 0.03;
c1 = -log(0.1)/t_1  * R*SCR/(1+R*SCR);
c1 = c1*1.5;
c2 = c1*10;
Kc = Qmax/R * c1/(s+c1)*(s+c2)/c2;
Kc_10 = Qmax/R * c1/(s+c1/10)*(s+c2)/c2;

%% PODQ controller
f_low = 0.25;
f_high = 1;
k = 8;
a1 = (2*pi*f_low)/10;
a2 = (2*pi*f_high)*10;
b1 = a1*5;
b2 = b1*k*a1/a2*R/Qmax;

Ka = k*s/(s+a2)*(s+b1)/(s+b2);
Kb = k*a1/(s+a2)*(s+b1)/(s+b2);
K = k*(s+a1)/(s+a2)*(s+b1)/(s+b2); % PODQ-controller

K_POD_0 = (s+a1)/(s+a2);
K_POD = k*(s+a1)/(s+a2);

%% PODQ controller (optimal)
k_opt = 51;
b2_opt = b1*k_opt*a1/a2*R/Qmax;

Ka_opt = k_opt*s/(s+a2)*(s+b1)/(s+b2_opt);
Kb_opt = k_opt*a1/(s+a2)*(s+b1)/(s+b2_opt);
K_opt = k_opt*(s+a1)/(s+a2)*(s+b1)/(s+b2_opt); % PODQ-controller

%% Loading default control objects
opt_db_default = opt;
opt_db_default.PODQ_type = 4;
db = 0.03;
deadband = [-db, +db];
% IBR
u_max = 1; % Maximum input [p.u.]
opt_db_default.PODQ = tf(zeros(PAR.nbus));
opt_db_default.PODQ(bus_idx,bus_idx) = H0 ; % [Mvar/Mvar];
opt_db_default.PODQ_max_out = [-1,1].*ones(PAR.nbus,1)*u_max*PAR.Sb;

% Communication delay
opt_db_default.PODQ_COM = tf(zeros(PAR.nbus));
opt_db_default.PODQ_COM(bus_idx,bus_idx) = pade(Hcom,5); % Communication delay
% opt_db.PODQ_COM(bus_idx,bus_idx) = 1; % No delay

% Derivative controller
u_max = 0.05; % Maximum input [p.u.]
opt_db_default.PODQ1 = tf(zeros(PAR.nbus));
opt_db_default.PODQ1(bus_idx,bus_idx) = Ka*PAR.Sb; % [Mvar/p.u. voltage]
opt_db_default.PODQ1_max_out = [-1,1].*ones(PAR.nbus,1)*u_max*PAR.Sb;

% Proportional controller
u_max = 1; % Maximum input [p.u.]
opt_db_default.PODQ2 = tf(zeros(PAR.nbus));
opt_db_default.PODQ2(bus_idx,bus_idx) = Kb*PAR.Sb; % [Mvar/p.u. voltage]
% opt_db.PODQ2_max_out = [-1,1].*ones(PAR.nbus,1)*u_max*PAR.Sb;

% Aux dead band controller
u_max = 1/3; % Maximum input [p.u.]
opt_db_default.PODQaux = tf(zeros(PAR.nbus));
opt_db_default.PODQaux(bus_idx,bus_idx) = Kc_10*PAR.Sb; % [Mvar/p.u. voltage]
opt_db_default.PODQaux_dead_band= deadband.*ones(PAR.nbus,1); % Relative deadband, ex. voltage within [-0.02, 0.02] p.u.
opt_db_default.PODQaux_max_out = [-1,1].*ones(PAR.nbus,1)*u_max*PAR.Sb;

%% Voltage controller
opt_default = opt_db_default;
opt_default.PODQaux = tf(zeros(PAR.nbus));
opt_default.PODQ_COM(bus_idx,bus_idx) = 1; % No delay
opt_default.PODQ1 = tf(zeros(PAR.nbus));
opt_default.PODQ2 = tf(zeros(PAR.nbus));
opt_default.PODQaux = tf(zeros(PAR.nbus));

