opt_CIGRE2024;
opt.machine_damping = [1,1]*1;
otp.mp=2;

PAR1 = data_PODQ_test_system(opt); % Network with default setting from opt_CIGRE2024
opt.PL = [2, 2, 0, 0]*0;
opt.PG = -1*[0.5, -0.5 , 0, 0]+opt.PL; % Reversing direction of power flow
PAR2 = data_PODQ_test_system(opt); % Network with reversed active power flow 

[~, YBUS, ~, fig] = lin_fg_xy_manual(PAR1,0); % Rita figur (manuell linjärisering)
% camroll(0); pos = get(fig,'position'); set(fig,'position',[pos(1:2),9,6])
Z0 = pinv(YBUS);
SCP = -imag(1/Z0(4,4))
G = lin_fg_xy(PAR1.xy0,PAR1);
[mode] = modal_shape(G.A, G.C((1:PAR1.nbus),:), []);
[~,idx] = sort(mode.e(mode.f>0.4/PAR1.period_time)); 
[mode] = pick_out_modes(mode,idx);
[fig, l] = plot_mode_shapes(mode,1,1, {'BUS 1','BUS 2','BUS 3','BUS 4'}, [4,3]);

bus_idx = 4;
nbus = PAR1.nbus;
omega_lim = log10( [0.02,8]*2*pi );
bodeopt = struct(); 
% bodeopt.FreqUnits = 'Hz';
% bodeopt.omega = logspace(omega_lim(1),omega_lim(2),1000);
bodeopt.omega_lim = [0.02,8]*2*pi ;

G1 = lin_fg_xy(PAR1.xy0,PAR1);
G2 = lin_fg_xy(PAR2.xy0,PAR2);
Z0 = pinv(YBUS);
SCP = -imag(1/Z0(4,4))
SCP=1/G1.D(8,8);

[fig,name,co] = figureLatex(figwidth,figheight);

u_idx = nbus + bus_idx; % -QL(4);
y_idx = nbus + bus_idx; % V(4);
plot_bode_and_res_2;

saveLatex(fig,'pairing_QV',fig_save_opt)

[fig,name,co] = figureLatex(figwidth,figheight);
u_idx = bus_idx; % -PL(4);
y_idx = nbus + bus_idx; % V(4);
plot_bode_and_res_2;
saveLatex(fig,'pairing_PV',fig_save_opt)
s = tf('s');

[fig,name,co] = figureLatex(figwidth,figheight);
u_idx = nbus + bus_idx; % -QL(4);
y_idx = bus_idx; % ANG(4);
plot_bode_and_res_2;
saveLatex(fig,'pairing_Qw',fig_save_opt)


[fig,name,co] = figureLatex(figwidth,figheight);
u_idx = bus_idx; % -PL(4);
y_idx = bus_idx; % ANG(4);
plot_bode_and_res_2;
saveLatex(fig,'pairing_Pw',fig_save_opt)

if 0
PAR1.output = 'Pline';
PAR2.output = 'Pline';
G1 = lin_fg_xy(PAR1.xy0,PAR1);
G2 = lin_fg_xy(PAR2.xy0,PAR2);


[fig,name,co] = figureLatex(figwidth,figheight);
u_idx = nbus + bus_idx; % -QL(4);
y_idx = 1; % P from BUS1 to BUS 3
plot_bode_and_res_2;
saveLatex(fig,'pairing_QPline',fig_save_opt)

[fig,name,co] = figureLatex(figwidth,figheight);
u_idx = bus_idx; % -PL(4);
y_idx = 1; % P from BUS1 to BUS 3
plot_bode_and_res_2;
saveLatex(fig,'pairing_PPline',fig_save_opt)
end