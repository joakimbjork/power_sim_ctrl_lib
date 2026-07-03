opt = struct();
opt.ns = 2; % States/machine (ns = 2 -> Classical model)
opt.zero_power_flow =0; % Load model with no power flow on transmission lines (to not care about transient stability etc)

opt.machine_damping = 400;

% Hydro governors at machine nodes (5 machine nodes)
k = 1400/0.4; % Static gain [MW/Hz];
s = tf('s');
F_desired = k/(1+5*s);
F_hydro = F_desired*(2-s)/(2+s); % Dynamic regulator/governor [MW/Hz];
c_FCR = diag([0.6,0.3,0.1,0,0]); % Participation factors (sum=1)
opt.GOV = F_hydro*c_FCR;

% Converter buses at network nodes  (6 machine nodes)
cbus = [5,6];
opt.CONV = struct();
opt.CONV.bus = cbus;
opt.CONV.Storage = ss(0.1*eye(length(cbus)));
opt.CONV.GFM=1;
opt.CONV.MBASE = [10;10]; % pu on system base

[PAR0] = data_Nordic5(opt); % Modell without FFR

opt.PODP =  tf(zeros(PAR0.nbus)); % 

H_battery = s/(s+0.01);
% H_wind = (s-0.05)/(s+0.05);
F_PODP = s/(1*s+1);

F_inertia = s/(0.01*s+1);

opt1 = opt;
opt2 = opt;

for i = cbus % POD-P at non-converter buses models ideal constant power loads
    opt1.PODP(i,i) = 2*k*F_PODP*H_battery/length(cbus);
    opt2.PODP(i,i) = 2*k*F_inertia*H_battery/length(cbus);
end
[PAR1] = data_Nordic5(opt1);
[PAR2] = data_Nordic5(opt2);

%% Draw network
[Gm, ~, ~, fig] = lin_fg_xy_manual(PAR0,2,100); % Rita figur (manuell linjärisering)
camroll(0); pos = get(fig,'position'); set(fig,'position',[pos(1:2),8,4])

%% Bode diagram of controllers
[fig,~,co] = figureLatex(5,3.5);
bodeopt = struct();
% bodeopt.wrap = true;
bodeopt.FreqUnits = 'Hz';
bodeopt.omega_lim = [5e-3,5e2] ;

bodeopt = bodeLatex(k*F_hydro,bodeopt); hold all

bodeopt = bodeLatex(k*F_PODP*H_battery,bodeopt); hold all
bodeopt = bodeLatex(k*F_inertia*H_battery,bodeopt); hold all

bode_draw_window2(0.25,1,-90,90,[0.2,1,0.2])
bode_draw_window(0.25,1)

legendLatex(fig,{'$F_\mathrm{FCR}$ (hydro)','$F_\mathrm{FFR}$','$F_\mathrm{inertia}$'});

%% Simulate load disturbance with different converter controllers
Tend = 30;
d_bus = [2]; % Välj var störningen ska ske

simopt = struct();
simopt.dPL = zeros(PAR0.nbus,1); % Störning aktiv effekt
simopt.dPL(d_bus) = [9]; % Störning i per unit, P_bas = PAR.Sb

% 
y1 = runsim(PAR0, Tend, simopt);
plot2_freq_control;
set(fig,'name','Simulated load disturbance without FFR')

% 
y1 = runsim(PAR1, Tend, simopt);
plot2_freq_control;
set(fig,'name','Simulated load disturbance with FFR (Battery storage)')

% 
y1 = runsim(PAR2, Tend, simopt);
plot2_freq_control;
set(fig,'name','Simulated load disturbance with Synthetic inertia (Battery storage)')

